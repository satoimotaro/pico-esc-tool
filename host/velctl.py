#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""velctl — feed-forward + telemetry-trimmed velocity control for the RP2040 esc_tool protocol.

Runs a target mechanical SPEED by inverting one per-motor calibrated curve (from velcal): it
gently slews the setpoint and, each 50 Hz tick, looks up the ESC command for that speed and adds
a PI trim on (target - measured mech RPM) whose authority FADES with telemetry liveness (Phase A1
closed loop; see docs/velocity-control.md). Above the crossover seam (6-step, live `tele`) the
loop closes; below it (forced sine, stale telemetry) it degrades to pure feed-forward. The command
still goes over the existing signed `thrust` command.

  --rpm is the TARGET SPEED. The controller looks up the ESC command ("thrust", -1000..1000 —
  an ESC DRIVE COMMAND, NOT a physical force) from the calibrated curve. There is NO force
  sensor. Reverse is just a negative --rpm (the inverse curve is odd-symmetric).

  python velctl.py speed --rpm 100 --dry-run                 # below the S3 seam (forced sine)
  python velctl.py speed --rpm 320 --crossover --dry-run     # crosses the sine<->BEMF seam

ONE calibrated curve carries the feed-forward across the S3 sine<->BEMF crossover; --crossover
enables the firmware crossover (writes the profile's crossover bytes; in --dry-run it configures
the sim only). The PI trim (tune with --kp/--ki/--trim-max/--blend-secs; --kp 0 --ki 0 = pure FF)
engages automatically only where telemetry is live. --encoder adds an INDEPENDENT verify-log
column that NEVER feeds the command; --debug-csv appends tele_rpm,trim diagnostics.

Safety mirrors posctl and is REUSED, not reimplemented: all thrust via ESC.thrust ->
PosDrive.send_thrust (single clamp/choke), one command per ~20 ms tick (< 500 ms deadman),
temperature poll + abort, and on EVERY exit (normal, error, SIGINT/SIGTERM) it disarms.
--dry-run never opens a port and never writes EEPROM.
"""
from __future__ import annotations

import argparse
import csv
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import esctool  # noqa: E402,F401
from esctool import EscHost  # noqa: E402

from pico_esc.esc import ESC  # noqa: E402
from pico_esc.link import RealClock, SimClock  # noqa: E402
from pico_esc.sim import SimEncEscHost  # noqa: E402
from pico_esc.drive import Aborted  # noqa: E402
from pico_esc.velocity import DEFAULT_GAINS, SpeedProfile, VelocityController  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
REPORT_DIR = os.path.join(HERE, "reports")
DEFAULT_PROFILE = os.path.join(HERE, "profiles", "vel_930kv_12n14p_sim.yaml")


def open_host(opts):
    """Return (host, clock).  --dry-run NEVER opens a serial port."""
    if opts.dry_run:
        inv = getattr(opts, "sim_invert", False)
        print(f"# DRY-RUN: SimEncEscHost (no serial port opened)"
              f"{' [--sim-invert: +thrust -> -encoder]' if inv else ''}")
        clock = SimClock()
        return SimEncEscHost(clock, seed=opts.seed, invert=inv), clock
    return EscHost(opts.port), RealClock()


def open_csv(opts, use_encoder):
    os.makedirs(REPORT_DIR, exist_ok=True)
    if opts.csv:
        path = opts.csv
    else:
        ts = time.strftime("%Y%m%d-%H%M%S")
        path = os.path.join(REPORT_DIR, f"velctl_speed_{ts}.csv")
    fh = open(path, "w", encoding="utf-8", newline="")
    w = csv.writer(fh)
    cols = ["t", "rpm_setpoint", "rpm_slewed", "thrust", "temp"]
    if use_encoder:
        cols.append("enc_rpm")
    if opts.debug_csv:                      # closed-loop diagnostics (opt-in; default header unchanged)
        cols += ["tele_rpm", "trim"]
    w.writerow(cols)
    return fh, w, path


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)
    sp = sub.add_parser("speed", help="hold a target speed (RPM) sensorlessly")
    sp.add_argument("--rpm", type=float, required=True,
                    help="TARGET SPEED (mech RPM; negative = reverse). The ESC command is looked "
                         "up from the calibrated curve — --rpm is a SPEED, not a force.")
    sp.add_argument("--profile", default=DEFAULT_PROFILE,
                    help="calibrated ESC-command->speed curve YAML (default: sim-derived 930KV)")
    sp.add_argument("--crossover", action="store_true",
                    help="enable the S3 sine<->BEMF crossover (writes the profile's crossover "
                         "bytes; in --dry-run configures the sim only)")
    sp.add_argument("--encoder", action="store_true",
                    help="add an INDEPENDENT encoder verify-log column (NEVER feeds the command)")
    sp.add_argument("--slew", type=float, default=200.0,
                    help="setpoint slew rate, RPM/s (default 200; keeps the first command gentle)")
    sp.add_argument("--stop-below-rpm", type=float, default=0.0,
                    help="a target at/below this |RPM| is a STOP (command thrust 0, disengage the loop) "
                         "— speed can't be held sensorlessly near 0 (default 0: only --rpm 0 stops)")
    sp.add_argument("--kp", type=float, default=None,
                    help="closed-loop PI proportional gain on (target - tele mech RPM); 0 = pure FF "
                         f"(default: profile control.kp, else {DEFAULT_GAINS['kp']:g})")
    sp.add_argument("--ki", type=float, default=None,
                    help="closed-loop PI integral gain (trim units per RPM-second); 0 = no integral "
                         f"(default: profile control.ki, else {DEFAULT_GAINS['ki']:g})")
    sp.add_argument("--trim-max", type=float, default=None,
                    help="PI trim clamp, ESC-command units "
                         f"(default: profile control.trim_max, else {DEFAULT_GAINS['trim_max']:g})")
    sp.add_argument("--blend-secs", type=float, default=None,
                    help="seconds for the PI authority to fade in/out with telemetry liveness "
                         f"(default: profile control.blend_secs, else {DEFAULT_GAINS['blend_secs']:g})")
    sp.add_argument("--debug-csv", action="store_true",
                    help="append closed-loop tele_rpm,trim columns to the CSV (default header unchanged)")
    sp.add_argument("--max-temp", type=float, default=80.0,
                    help="poll ESC temperature and abort at this C (0=off, no poll; default 80)")
    sp.add_argument("--secs", type=float, default=5.0, help="hold duration, s (default 5)")
    sp.add_argument("--tmax", type=int, default=1000, help="ESC command magnitude ceiling (default 1000)")
    sp.add_argument("--esc-index", type=int, default=1, help="ESC index (default 1)")
    sp.add_argument("--invert-encoder", action="store_true",
                    help="(--encoder) +thrust -> -encoder wiring for the verify-log")
    sp.add_argument("--port", help="serial port (default: auto-detect)")
    sp.add_argument("--csv", help="CSV output path (default: auto-named in host/reports/)")
    sp.add_argument("--dry-run", action="store_true", help="run against the simulated ESC (no hardware)")
    sp.add_argument("--sim-invert", action="store_true",
                    help="(dry-run) model inverted wiring: +thrust drives the encoder negative")
    sp.add_argument("--seed", type=int, default=1234, help="RNG seed for --dry-run (deterministic)")
    return ap


def _validate(opts):
    opts.tmax = abs(int(opts.tmax))
    for name in ("slew", "secs", "tmax"):
        if getattr(opts, name) <= 0:
            sys.exit(f"--{name.replace('_', '-')} must be > 0")
    if opts.max_temp < 0:
        sys.exit("--max-temp must be >= 0")
    if not os.path.exists(opts.profile):
        sys.exit(f"profile not found: {opts.profile}")


def main():
    opts = build_parser().parse_args()
    _validate(opts)

    try:
        profile = SpeedProfile.load(opts.profile)
    except (ValueError, KeyError) as e:
        sys.exit(f"bad profile {opts.profile}: {e}")
    if abs(opts.rpm) > profile.max_rpm:
        print(f"# note: --rpm {opts.rpm:g} exceeds the curve max {profile.max_rpm:.0f} RPM; "
              f"the command clamps to the curve endpoint (open-loop, no extrapolation)")

    # Effective gains: explicit CLI flag wins, else the profile's own control block, else the
    # built-in DEFAULT_GAINS. Gains are plant-dependent (see DEFAULT_GAINS) so a bench-tuned profile
    # carries them; this is where the three tiers merge.
    def _gain(name):
        cli = getattr(opts, name)               # --trim-max -> opts.trim_max, --blend-secs -> blend_secs
        return cli if cli is not None else profile.control_gain(name, DEFAULT_GAINS[name])
    kp, ki, trim_max, blend_secs = (_gain("kp"), _gain("ki"),
                                    _gain("trim_max"), _gain("blend_secs"))

    host, clock = open_host(opts)
    esc = ESC(host, opts.esc_index, tmax=opts.tmax, clock=clock)
    enc_sign = -1 if (opts.invert_encoder or opts.sim_invert) else 1
    ctrl = VelocityController(esc, profile, kp=kp, ki=ki, trim_max=trim_max,
                              blend_secs=blend_secs, slew_rpm_s=opts.slew,
                              max_temp=opts.max_temp, max_secs=opts.secs,
                              use_encoder=opts.encoder, enc_sign=enc_sign,
                              stop_below_rpm=opts.stop_below_rpm)
    ctrl.set_speed(opts.rpm)

    def _panic(*_):
        esc.disarm()
        try:
            host.close()
        finally:
            os._exit(1)
    signal.signal(signal.SIGINT, _panic)
    signal.signal(signal.SIGTERM, _panic)

    fh, writer, csv_path = open_csv(opts, opts.encoder)

    def _row(t, target, sp, thrust, temp, enc_rpm, tele_rpm, trim):
        row = [f"{t:.4f}", f"{target:.3f}", f"{sp:.3f}", int(thrust),
               "" if temp is None else temp]
        if opts.encoder:
            row.append("" if enc_rpm is None else f"{enc_rpm:.1f}")
        if opts.debug_csv:
            row += ["" if tele_rpm is None else f"{tele_rpm:.1f}", f"{trim:.1f}"]
        writer.writerow(row)

    print(f"# profile: {opts.profile}  (motor={profile.motor}, {len(profile.points)} points, "
          f"max {profile.max_rpm:.0f} RPM); target {opts.rpm:g} RPM -> "
          f"start command {int(profile.thrust_for(opts.rpm))} (regime: {ctrl.regime(opts.rpm)})")
    _gsrc = "profile" if profile.control else "default"
    print(f"# gains: kp={kp:g} ki={ki:g} trim_max={trim_max:g} blend_secs={blend_secs:g} "
          f"(source: CLI overrides > {_gsrc})")

    failure = None
    reason = "aborted"
    try:
        # Enable the S3 crossover (profile's bytes) — EEPROM on hardware, sim cfg in --dry-run.
        if opts.crossover:
            cx = profile.crossover
            if not cx or not cx.get("bytes"):
                sys.exit("--crossover: the profile carries no crossover bytes")
            cu, cd = cx["bytes"]
            esc.config.set(sine_cross_up=int(cu), sine_cross_dn=int(cd))
            esc.restart()
            print(f"# crossover enabled: sine_cross_up={cu} sine_cross_dn={cd}")
        esc.prepare()
        esc.arm()                          # bidir
        reason = ctrl.run(clock, on_row=_row)
    except Aborted as e:
        failure = str(e)
    finally:
        was_armed = esc.drive.armed
        esc.disarm()                       # ALWAYS disarm
        if was_armed:
            print("# DISARMED")
        fh.close()
        host.close()

    print(f"# wrote CSV: {csv_path}")
    if ctrl.peak_temp is not None:
        print(f"# ESC temp: last {ctrl.last_temp}C, peak {ctrl.peak_temp}C")
    print(f"# exit reason: {reason if failure is None else 'aborted'}")
    print(f"# target={opts.rpm:g} RPM  final setpoint={ctrl.setpoint:.1f} RPM  "
          f"final command={int(profile.thrust_for(ctrl.setpoint))}")
    if failure:
        sys.exit(f"VELCTL ABORTED: {failure}")


if __name__ == "__main__":
    main()
