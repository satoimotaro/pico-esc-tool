#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""posctl — host-side position servo for the RP2040 esc_tool protocol.

Closes a position loop around the integrated AS5600 encoder by driving the ESC over the
existing signed `thrust` command. Intended for BlueGill **S1 forced-commutation stepper
mode** (Pgm_Sine_Mode=1): the ESC then creeps smoothly far below the old ~185 RPM 6-step
floor and, at thrust 0, HOLDS with detent torque — so a true velocity servo works.

CASCADE PID: an outer position PID turns position error into a velocity setpoint
(vel_sp = clamp(Kp·err − Kd·vel, ±vmax)); an inner law turns that into signed thrust
(thrust = Kff·vel_sp + Ki·∫(vel_sp − vel), anti-windup, clamped ±tmax). Kff is the
firmware full-scale (thrust→eRPM) computed in tools/sim/sine_drive_model.py. Inside
tolerance AND slow, thrust drops to 0 and the ESC HOLDS — no pulsing, no bang-bang.

Requires the ESC in Bidirectional (3D) mode with S1 sine mode enabled
(host/profiles/posctl_930kv_sine.yaml). Achievable hold resolution is ~one stepper detent
(12N14P ⇒ ~8.6°); use --tol ~6°. Finer needs S2 microstepping, not tuning.

TUNING (930 KV 12N14P, retuned 2026-07-15 after the bench showed tiny overshoot + slow
reach): defaults are Kp=14, Kd=0.6, Ki=0.4, Kff=0.47, vmax=500. Raising Kp from the
original 6 ~halves the reach time AND reduces the systematic undershoot (the servo coasts
deeper into the tol band before it hands off to the detent hold), with Kd=0.6 pairing up
to keep overshoot near zero. These were picked with the offline plant sweep
`host/tune_posctl.py` (first-order rotor model in SimEncEscHost) and are deliberately
moderate — the real motor has stiction/detent/inertia the model underplays, so there is
overshoot headroom. Bench fine-tune: raise --kp until the real motor first shows a small
overshoot, back off ~20%, then raise --kd until it is gone; bump --ki only if a steady
final-error bias persists. Re-run `python tune_posctl.py` (drop --dry-run to sweep on real
hardware) to re-tune for the 300 KV motor when it arrives.

Keep-alive: one `enc` + one `thrust` per ~20 ms tick (50 Hz), well under the firmware's
500 ms spin deadman. Safety: --max-secs / --max-revs / --vel-abort aborts, encoder magnet-
health + unwrap-fault + expected-vs-measured stall + wrong-way guards, and on EVERY exit
path (normal, error, SIGINT/SIGTERM) it triple-sends `thrust 0` then `disarm`.

  python posctl.py move --deg 90 --dry-run
  python posctl.py step --seq 90,-90,360 --dry-run
  python posctl.py hold --deg 0 --dry-run

This is now a thin CLI wrapper: the controller, drive session, sim host, clocks, and guard
constants live in the pico_esc package. The names below are re-exported so
`from posctl import (PosDrive, COUNTS_PER_REV, DT, FULLSCALE_RPM, VEL_LP_ALPHA,
DELTA_FAULT_FRAC, Aborted, _pace)` (tune_sine_amp.py) keeps working, and the dry-run output
is byte-identical (same seeded sim, same cmd() call order).
"""
from __future__ import annotations

import argparse
import csv
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import esctool  # noqa: E402  (esctool lives next to this file)
from esctool import EscHost  # noqa: E402

from pico_esc.link import RealClock, SimClock  # noqa: E402
from pico_esc.sim import SimEncEscHost, SimEscHost  # noqa: E402,F401
from pico_esc.types import EncReading  # noqa: E402,F401
from pico_esc.drive import PosDrive, Aborted, ARM_WAIT  # noqa: E402,F401
from pico_esc.control import (  # noqa: E402,F401
    LOOP_HZ, DT, COUNTS_PER_REV, VEL_LP_ALPHA, ENC_FAIL_MAX, MEAN_WIN, FAULT_DT_MULT,
    DELTA_FAULT_FRAC, WRONGWAY_TICKS, WRONGWAY_MIN_DPOS, TELE_EVERY_S, PROBE_STEP,
    PROBE_MAX_THRUST, FULLSCALE_RPM, KFF_COMPUTED, STALL_TICKS, STALL_MOVE_FRAC,
    STALL_MIN_EXPECTED, PositionController, PIDServo, SegMetrics,
    _pace, _read_valid_enc, calibrate_direction, _rebaseline, run_segments,
)

HERE = os.path.dirname(os.path.abspath(__file__))
REPORT_DIR = os.path.join(HERE, "reports")


# ---------------------------------------------------------------------------
# Host selection + CSV.
# ---------------------------------------------------------------------------
def open_host(opts):
    """Return (host, clock).  --dry-run NEVER opens a serial port."""
    if opts.dry_run:
        inv = getattr(opts, "sim_invert", False)
        print(f"# DRY-RUN: SimEncEscHost (no serial port opened)"
              f"{' [--sim-invert: +thrust -> -encoder]' if inv else ''}")
        clock = SimClock()
        return SimEncEscHost(clock, seed=opts.seed, invert=inv), clock
    return EscHost(opts.port), RealClock()


def open_csv(opts, mode):
    os.makedirs(REPORT_DIR, exist_ok=True)
    if opts.csv:
        path = opts.csv
    else:
        ts = time.strftime("%Y%m%d-%H%M%S")
        path = os.path.join(REPORT_DIR, f"posctl_{mode}_{ts}.csv")
    fh = open(path, "w", encoding="utf-8", newline="")
    w = csv.writer(fh)
    w.writerow(["t", "raw", "pos_deg", "vel", "pos_setpoint", "vel_setpoint", "thrust"])
    return fh, w, path


def targets_for(mode, opts):
    if mode == "move":
        return [opts.deg]
    if mode == "hold":
        return [opts.deg]
    # step: comma-separated relative deltas
    try:
        seq = [float(x) for x in opts.seq.split(",") if x.strip() != ""]
    except ValueError:
        sys.exit(f"bad --seq '{opts.seq}' (want e.g. 360,-360,720)")
    if not seq:
        sys.exit("--seq is empty")
    return seq


# ---------------------------------------------------------------------------
# CLI.
# ---------------------------------------------------------------------------
def add_common(p):
    p.add_argument("--esc-index", type=int, default=1, help="ESC index (default 1)")
    p.add_argument("--tol", type=float, default=6.0,
                   help="settle tolerance, deg (default 6 ~ one S1 stepper detent hold resolution)")
    p.add_argument("--tmax", type=int, default=300, help="thrust magnitude ceiling, 0..1000 (default 300)")
    p.add_argument("--tmin", type=int, default=40, help="min thrust the guards treat as 'driving' (default 40)")
    # --- cascade PID gains (S1 velocity servo) ---
    p.add_argument("--kp", type=float, default=14.0,
                   help="outer position gain: velocity setpoint (deg/s) per deg of error "
                        "(default 14; retuned from 6 for faster reach — see module docstring TUNING)")
    p.add_argument("--kd", type=float, default=0.6,
                   help="outer damping: subtract Kd*vel from the velocity setpoint "
                        "(default 0.6; pairs with the higher Kp to keep overshoot ~0)")
    p.add_argument("--ki", type=float, default=0.4,
                   help="inner integral on velocity error -> thrust, anti-windup (default 0.4)")
    p.add_argument("--kff", type=float, default=0.47,
                   help="inner feedforward: thrust per (deg/s) of velocity setpoint. Firmware full "
                        "scale ~0.47 (see tools/sim/sine_drive_model.py stepper section) (default 0.47)")
    p.add_argument("--vmax", type=float, default=500.0,
                   help="velocity setpoint clamp, deg/s (default 500 ~ 83 RPM; keep low for gentle creep)")
    p.add_argument("--max-temp", type=float, default=80.0,
                   help="poll ESC telemetry temperature and abort at this many C (0=off, no poll). "
                        "The only thermal backstop when raising sine_amp_max/hold_amp — no current "
                        "sensing on EFM8BB21 (default 80)")
    p.add_argument("--max-secs", type=float, default=30.0, help="runaway/time abort (default 30)")
    p.add_argument("--max-revs", type=float, default=20.0, help="runaway travel abort, revs (default 20)")
    p.add_argument("--vel-abort", type=float, default=12000.0, help="hard |vel| abort, deg/s (default 12000)")
    p.add_argument("--settle-dwell", type=float, default=1.5, help="s within tol to declare settled")
    # direction convention (+thrust must map to +measured motion)
    p.add_argument("--invert-encoder", action="store_true",
                   help="force enc_sign=-1 (+thrust -> -encoder wiring) and skip auto-cal")
    p.add_argument("--no-autocal", action="store_true",
                   help="skip the direction probe and assume enc_sign=+1")
    p.add_argument("--probe-secs", type=float, default=0.5,
                   help="direction-cal forward-probe duration, s (default 0.5)")
    p.add_argument("--probe-min-deg", type=float, default=3.0,
                   help="min encoder travel to accept the motor broke away, deg (default 3)")
    p.add_argument("--probe-confirm-deg", type=float, default=30.0,
                   help="clear rotation arc required before trusting the direction SIGN, deg "
                        "(default 30; must exceed the cog pitch ~8.6 deg so cog-settling can't "
                        "flip the sign)")
    p.add_argument("--port", help="serial port (default: auto-detect)")
    p.add_argument("--csv", help="CSV output path (default: auto-named in host/reports/)")
    p.add_argument("--dry-run", action="store_true", help="run against the simulated ESC (no hardware)")
    p.add_argument("--sim-invert", action="store_true",
                   help="(dry-run) model inverted wiring: +thrust drives the encoder negative")
    p.add_argument("--seed", type=int, default=1234, help="RNG seed for --dry-run (deterministic)")


def build_parser():
    ap = argparse.ArgumentParser(description="Host-side cascade position controller (AS5600 + ESC)")
    sub = ap.add_subparsers(dest="mode", required=True)
    mv = sub.add_parser("move", help="move to an absolute target (deg from start)")
    mv.add_argument("--deg", type=float, required=True, help="target angle, deg")
    add_common(mv)
    st = sub.add_parser("step", help="run a sequence of relative steps (deg)")
    st.add_argument("--seq", required=True, help="comma-separated relative deltas, e.g. 360,-360,720")
    add_common(st)
    hd = sub.add_parser("hold", help="hold a target and measure the limit-cycle")
    hd.add_argument("--deg", type=float, default=0.0, help="hold angle, deg (default 0)")
    add_common(hd)
    return ap


def _validate(opts):
    # thrust/velocity limits are magnitudes; a negative --tmax would break the
    # symmetric clamp max(-tmax, min(tmax, u)) and pin thrust to +tmax (m5).
    opts.tmax = abs(int(opts.tmax))
    opts.tmin = abs(int(opts.tmin))
    for name in ("tmax", "tmin", "vmax", "kff", "vel_abort", "tol",
                 "max_secs", "max_revs", "probe_secs", "probe_min_deg"):
        if getattr(opts, name) <= 0:
            sys.exit(f"--{name.replace('_', '-')} must be > 0")
    for name in ("kp", "kd", "ki"):                      # gains may be 0 but not negative
        if getattr(opts, name) < 0:
            sys.exit(f"--{name.replace('_', '-')} must be >= 0")
    if opts.tmin >= opts.tmax:
        sys.exit("--tmin must be < --tmax")


def main():
    opts = build_parser().parse_args()
    _validate(opts)
    host, clock = open_host(opts)
    drive = PosDrive(host, opts.esc_index, opts.tmax, clock)
    ctrl = PIDServo(opts)
    targets = targets_for(opts.mode, opts)

    # Sanity: --kff should match the plant gain implied by FULLSCALE_RPM (the firmware full
    # scale). A large mismatch means feedforward is fighting the plant (bad tracking) —
    # usually a stale FULLSCALE_RPM vs a changed SINE_TICK_T2/SINE_RCP_SHIFT in the asm.
    note = ""
    if KFF_COMPUTED and abs(opts.kff - KFF_COMPUTED) / KFF_COMPUTED > 0.15:
        note = f"  [WARNING: differs >15% from firmware full-scale kff — check FULLSCALE_RPM/asm EQUs]"
    print(f"# kff={opts.kff:.4f} thrust/(deg/s)  (firmware full-scale implies {KFF_COMPUTED:.4f}; "
          f"FULLSCALE_RPM={FULLSCALE_RPM:.0f}){note}")

    def _panic(*_):
        drive.disarm()
        try:
            host.close()
        finally:
            os._exit(1)
    signal.signal(signal.SIGINT, _panic)
    signal.signal(signal.SIGTERM, _panic)

    fh, writer, csv_path = open_csv(opts, opts.mode)
    failure = None
    reason, metrics = "aborted", []
    try:
        drive.prepare()
        drive.arm()
        reason, metrics = run_segments(drive, ctrl, clock, writer, opts,
                                       targets, hold=(opts.mode == "hold"))
    except Aborted as e:
        failure = str(e)
    finally:
        drive.disarm()                       # ALWAYS disarm (mirror drive_hold.py)
        fh.close()
        host.close()

    print(f"# wrote CSV: {csv_path}")
    if drive.peak_temp is not None:
        print(f"# ESC temp: last {drive.last_temp}C, peak {drive.peak_temp}C")
    print(f"# exit reason: {reason}")
    for i, m in enumerate(metrics, 1):
        st = f"{m.settle_t:.2f}s" if m.settle_t is not None else "never"
        # When the segment ended via the servo's hand-off to hold (settle_t never latched),
        # pk-pk / mean-err come from the 0.3 s APPROACH-transient window, NOT a real
        # post-hold limit cycle — label them honestly. (The `hold` subcommand does latch.)
        if m.settle_t is not None:
            band = (f"limit-cycle_pk-pk={m.limit_cycle_amp():.1f}deg  "
                    f"steady-state_mean_err={m.steady_mean_err():+.1f}deg")
        else:
            band = (f"approach-window_pk-pk={m.limit_cycle_amp():.1f}deg(not post-hold)  "
                    f"approach_mean_err={m.steady_mean_err():+.1f}deg")
        print(f"# segment {i}: target={m.target:+.1f}deg  settle={st}  "
              f"overshoot={m.peak_overshoot:.1f}deg  {band}  "
              f"final_err={m.target - m.last_pos:+.1f}deg")
    if failure:
        sys.exit(f"POSCTL ABORTED: {failure}")


if __name__ == "__main__":
    main()
