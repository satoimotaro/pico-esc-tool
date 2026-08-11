#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""azimuth — position servo for the two-motor DIFFERENTIAL azimuth pod.

posctl.py servos one motor against an AS5600 on that same rotor. This pod is different in
exactly one way that changes everything: the encoder reads the AZIMUTH AXIS, and the axis is
driven by the SUM of two motors through a differential (pico_esc.differential):

    s = (A + B) / 2  -> azimuth        d = (A - B) / 2  -> propeller

So the control law is unchanged — PositionController's cascade PID from pico_esc.control, the
same one posctl uses — and what is new is only that its output is `s`, split across two ESCs by
DifferentialMixer, and that `enc` reads the controlled axis directly rather than a rotor.

WHY THE ENCODER PLACEMENT IS THE DESIGN. Azimuth positioning is low-speed POSITION control, which
is precisely where this stack's sensorless feedback is worthless: DShot telemetry floors at a
constant 182 rpm below the seam and honestly reports 0 throughout forced sine. Integrating that to
get an angle drifts without bound. Reading the axis directly removes the dependency outright — the
azimuth channel never touches telemetry. The propeller channel runs high, in the 6-step region
where eRPM is honest, so it is fed forward open-loop.

HOLDING ANGLE UNDER THRUST IS A CLOSED-LOOP MODE, NOT COUNTER-ROTATION. Nominal A = -B is only the
feedforward. Two different motors on two different ESCs under different loads never match exactly,
and the mismatch integrates straight into azimuth drift, so `hold --thrust` keeps the loop closed
and trims s around zero continuously. Commanding counter-rotation open-loop rotates the pod slowly.
`hold --thrust` also reports the drift it had to trim out, which is the number that says whether
the mechanism is good.

  python3 azimuth.py identify --dry-run          # resolve the sign convention FIRST
  python3 azimuth.py move --deg 90  --dry-run
  python3 azimuth.py hold --deg 0 --thrust 300 --dry-run

--dry-run EXERCISES THE CODE PATH, IT DOES NOT PREDICT THE MECHANISM. SimEncEscHost models one
rotor with one encoder on it; both channels here address the same sim, so there is no differential
in the loop and no second motor to mismatch. Dry-run numbers say the servo runs and the mixer is
wired up — they are not a forecast of the pod's angle error, and `hold --thrust` in particular has
nothing physical to hold against. The mixer's own kinematics ARE covered properly, by
tests/test_differential.py.

Run `identify` before anything else and pass its --sign-b through: in a bevel differential the
second motor is normally mounted mirrored, and an unverified sign convention SWAPS the steering
and thrust channels — pure thrust steers, and a turn command drives the propeller.
"""
from __future__ import annotations

import argparse
import signal
import sys
import time

from pico_esc.control import PositionController, _pace
from pico_esc.differential import DifferentialMixer, DualDrive
from pico_esc.drive import Aborted, PosDrive
from pico_esc.link import EscHost, RealClock, SimClock
from pico_esc.sim import SimEncEscHost

DT = 0.02                      # 50 Hz tick — same as posctl, well under the 500 ms spin deadman
COUNTS_PER_REV = 4096

# Command at which the ESC hands off from forced sine to 6-step, per `sine_cross_up`
# (docs/thruster-bringup.md §2 — the handoff fires at a duty floor, so this is KV-independent).
HANDOFF_CMD = {34: 484, 36: 515, 40: 578, 44: 640}

# MEASURED ON THIS POD 2026-08-11: at command 650 — just past the 515 handoff for cross_up 36 —
# the azimuth axis reached ~554 rpm, i.e. more than NINE REVOLUTIONS PER SECOND of the pod itself.
# Extrapolating the 6-step duty map back to the handoff still leaves several hundred rpm at the
# very first 6-step point, so there is no slow end to that region: the azimuth goes from a few
# degrees of forced-sine creep straight to hundreds of rpm the instant it crosses the seam.
#
# THE 6-STEP REGION IS THEREFORE UNUSABLE FOR AZIMUTH POSITIONING, and entering it by accident is
# a hazard, not just a bad control result — it was fast enough to be called dangerous at the bench.
# Azimuth authority must stay entirely inside forced sine, which is exactly the regime posctl's S1
# forced-commutation + detent hold is written for.
AZIMUTH_RATE_AT_650_RPM = 554.0


# ---------------------------------------------------------------------------------
def _mk_host(args):
    """Real Pico, or the deterministic sim host for --dry-run."""
    if args.dry_run:
        clock = SimClock()
        return SimEncEscHost(clock), clock
    return EscHost(args.port), RealClock()


def _enc_deg(drive):
    r = drive.read_enc()
    if r is None:
        return None, None
    return r.raw, r.raw * 360.0 / COUNTS_PER_REV


def _guard_encoder(drive, allow_weak=False):
    """Refuse to servo without a live, healthy azimuth encoder.

    A blind encoder reads exactly like a stalled axis: raw stops changing, the controller sees a
    huge standing error and winds the sum command up against a mechanism that may well be moving.
    This is the same failure startlog hit — a dead read fabricating a mechanical fault — so it is
    checked before arming rather than discovered during the run.

    `allow_weak` downgrades the ML (magnet-too-weak) bit from a refusal to a warning. On the AS5600,
    ML is set precisely when AGC saturates at maximum gain, so ML=1 and AGC=128 (at 3.3 V) are one
    fact, not two: the field is at the bottom of the usable range. Measured on this pod at that
    setting, tracking was still clean — 10.8 hand-turned revolutions with zero dropped samples, a
    monotonic accumulator, MAG stable to ±4 %, and the on-device velocity agreeing with the
    accumulated travel. So it is usable, but with **no margin**: temperature, vibration or any
    increase in the gap starts dropping samples, and a dropped sample is what this guard exists to
    catch. Closing the gap is the fix; this flag exists so running without it is a deliberate,
    logged choice rather than a silently weakened check.
    """
    r = drive.read_enc()
    if r is None:
        sys.exit("no azimuth encoder: `enc` returned nothing.\n"
                 "  The firmware re-probes at ~1 kHz and self-heals on hot-plug, so a persistent\n"
                 "  failure is a live I2C fault, not boot order. Check 3V3/GND at the sensor,\n"
                 "  SDA=GP16 / SCL=GP17 (I2C0, addr 0x36 — physical pins 21/22, NOT 16/17), the\n"
                 "  bus pull-ups, and whether the part is an AS5600L (configurable address).")
    if r.md != 1 or r.mh != 0:
        sys.exit(f"azimuth encoder unhealthy (md={r.md} ml={r.ml} mh={r.mh} agc={r.agc}): "
                 "no magnet detected, or it is too strong. Fix the magnet before servoing — a "
                 "marginal magnet drops samples, and a dropped sample is indistinguishable from a "
                 "stalled axis.")
    if r.ml != 0:
        if not allow_weak:
            sys.exit(f"azimuth magnet TOO WEAK (ml=1, agc={r.agc} — AGC is saturated at maximum "
                     "gain, which is the same fact stated twice). Close the axial gap so AGC comes "
                     "off the rail, or pass --allow-weak-magnet to run at zero margin deliberately.")
        print(f"# WARNING: magnet weak (ml=1, agc={r.agc} = AGC saturated). Running anyway on "
              f"--allow-weak-magnet. Tracking has NO margin: watch for dropped samples.")
    return r


# ---------------------------------------------------------------------------------
def cmd_identify(args):
    """Drive ONE motor at a time and measure the azimuth rate each produces.

    This is the run that resolves sign_a / sign_b and the azimuth gain. It deliberately uses
    PosDrive per channel rather than DualDrive: the whole point is to move one motor while the
    other is left un-armed, which DualDrive refuses to do.
    """
    host, clock = _mk_host(args)
    rates = {}
    try:
        for name, idx in (("A", args.idx_a), ("B", args.idx_b)):
            drive = PosDrive(host, idx, args.tmax, clock, verbose=True)
            drive.prepare()
            _guard_encoder(drive, args.allow_weak_magnet)
            drive.arm()
            try:
                _, deg0 = _enc_deg(drive)
                t0 = clock.now()
                drive.send_thrust(args.cmd)
                clock.sleep(args.secs)
                _, deg1 = _enc_deg(drive)
                dt = clock.now() - t0
                drive.send_thrust(0)
                # unwrapped over the window: the axis may cross the 0/360 seam
                d = ((deg1 - deg0 + 180.0) % 360.0) - 180.0
                rates[name] = d / dt if dt > 0 else 0.0
                print(f"  motor {name} (esc {idx}) @ cmd {args.cmd}: "
                      f"{deg0:7.2f} -> {deg1:7.2f} deg  ==>  {rates[name]:+8.3f} deg/s")
            finally:
                drive.disarm()

        sign_a, sign_b, gain = DifferentialMixer.identify(rates["A"], rates["B"])
        # a lone motor at command u puts s = u/2 on the azimuth channel
        gain_per_s = gain / (args.cmd / 2.0)
        print(f"\n  sign_a = {sign_a:+d}   sign_b = {sign_b:+d}")
        print(f"  azimuth gain ~ {gain_per_s:.4f} deg/s per unit s  "
              f"(=> kff ~ {1.0 / gain_per_s:.3f})")
        if sign_b != sign_a:
            print("\n  B is mounted MIRRORED. Without --sign-b -1 the steering and thrust\n"
                  "  channels are swapped: pure thrust would steer and a turn would spin the prop.")
        print(f"\n  pass through:  --sign-a {sign_a} --sign-b {sign_b} --kff {1.0 / gain_per_s:.3f}")
    finally:
        host.close()


# ---------------------------------------------------------------------------------
def _run_servo(args, targets, thrust):
    """Shared servo loop for move/hold: cascade PID on s, feedforward d, both via the mixer."""
    host, clock = _mk_host(args)
    mixer = DifferentialMixer(tmax=args.tmax, sign_a=args.sign_a, sign_b=args.sign_b)
    drive = DualDrive(host, mixer, args.idx_a, args.idx_b, args.tmax, clock)
    ctrl = PositionController(args)
    stop = {"flag": False}

    def _bail(*_):
        stop["flag"] = True
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, _bail)

    try:
        drive.prepare()
        _guard_encoder(drive, args.allow_weak_magnet)
        drive.arm()
        raw0, _ = _enc_deg(drive)
        ctrl.update_encoder(raw0, DT)
        ctrl.pos_deg = 0.0

        for target in targets:
            ctrl.set_target(target)
            t_end = clock.now() + args.max_secs
            worst = 0.0
            while not stop["flag"] and clock.now() < t_end:
                tick = clock.now()
                r = drive.read_enc()
                if r is None:
                    raise Aborted("azimuth encoder read failed mid-run — disarming")
                ctrl.update_encoder(r.raw, DT, suspect=not r.healthy)
                u, _vel_sp = ctrl.step(DT)
                m = drive.send(u, thrust)
                worst = max(worst, abs(ctrl.target - ctrl.pos_deg))
                if args.verbose:
                    print(f"  t={clock.now():6.2f}  pos={ctrl.pos_deg:8.2f}  err="
                          f"{ctrl.target - ctrl.pos_deg:+7.2f}  s={m.s:+7.1f}  d={m.d:+7.1f}"
                          f"  A={m.a:+7.1f}  B={m.b:+7.1f}{'  SAT' if m.saturated else ''}")
                if ctrl.holding and thrust == 0:
                    break                       # settled and no thrust to fight: done
                _pace(clock, tick)
            err = ctrl.target - ctrl.pos_deg
            print(f"target {target:+8.2f} deg -> pos {ctrl.pos_deg:+8.2f}  err {err:+6.2f}  "
                  f"worst {worst:6.2f}" + (f"  (thrust d={thrust})" if thrust else ""))
    finally:
        try:
            drive.disarm()
        finally:
            host.close()


def cmd_move(args):
    _run_servo(args, [float(x) for x in args.deg.split(",")], 0.0)


def cmd_hold(args):
    """Hold an angle WHILE the propeller runs. The loop stays closed on purpose."""
    if args.thrust:
        print(f"# holding {args.deg} deg with propeller command d={args.thrust}.\n"
              f"# The loop stays CLOSED: A=-B is only the feedforward, and any mismatch between\n"
              f"# the two motors integrates into azimuth drift. Reported err is what it trimmed.")
    _run_servo(args, [float(args.deg)], float(args.thrust))


# ---------------------------------------------------------------------------------
def add_common(p):
    """Shared options on every subcommand — posctl's convention, so `… move --deg 90 --dry-run`
    works the way the docstring advertises rather than only before the subcommand."""
    p.add_argument("--port", help="serial port (default: auto-detect VID 2E8A)")
    p.add_argument("--dry-run", action="store_true", help="run against the sim host, no hardware")
    p.add_argument("--idx-a", type=int, default=0, help="ESC index of motor A (GP10)")
    p.add_argument("--idx-b", type=int, default=1, help="ESC index of motor B (GP11)")
    p.add_argument("--tmax", type=int, default=400, help="per-channel thrust ceiling")
    p.add_argument("--sign-a", type=int, default=1, choices=(1, -1))
    p.add_argument("--sign-b", type=int, default=1, choices=(1, -1),
                   help="RUN `identify` FIRST — an unverified sign swaps steering and thrust")
    p.add_argument("--cross-up", type=int, default=36,
                   help="the ESC's sine_cross_up — sets where forced sine hands off to 6-step")
    p.add_argument("--allow-6step", action="store_true",
                   help="permit commands past the seam; the azimuth reaches hundreds of rpm there")
    p.add_argument("--allow-weak-magnet", action="store_true",
                   help="run with ML=1 / AGC saturated — zero margin, see _guard_encoder")
    p.add_argument("-v", "--verbose", action="store_true")


def add_gains(p):
    """posctl's 930KV single-motor tune, as a STARTING POINT ONLY. Gains do not transfer between
    motors, and this is not even the same plant — the load here is the whole azimuth axis through
    a reduction, not a bare rotor. Tune in situ."""
    p.add_argument("--kp", type=float, default=14.0)
    p.add_argument("--kd", type=float, default=0.6)
    p.add_argument("--ki", type=float, default=0.4)
    p.add_argument("--kff", type=float, default=0.47)
    p.add_argument("--vmax", type=float, default=500.0)
    p.add_argument("--tol", type=float, default=6.0)
    p.add_argument("--tmin", type=float, default=0.0)
    p.add_argument("--max-secs", type=float, default=15.0)


def _validate(args):
    args.tmax = abs(int(args.tmax))
    if args.tmax <= 0:
        sys.exit("--tmax must be > 0")
    # Keep the azimuth channel inside forced sine. Crossing the seam is not a degraded control
    # result, it is a hazard: measured, the first 6-step points already spin the pod at hundreds of
    # rpm (see AZIMUTH_RATE_AT_650_RPM). tmax is the per-channel ceiling, and `identify` also drives
    # a single channel at --cmd, so both are checked.
    handoff = HANDOFF_CMD.get(args.cross_up)
    if handoff is None:
        print(f"# WARNING: no handoff command known for sine_cross_up={args.cross_up} "
              f"(known: {sorted(HANDOFF_CMD)}). Cannot check that --tmax stays in forced sine.")
    elif args.tmax >= handoff and not args.allow_6step:
        sys.exit(f"--tmax {args.tmax} reaches the 6-step seam (handoff is command {handoff} at "
                 f"sine_cross_up={args.cross_up}).\n"
                 f"  Measured on this pod: ~{AZIMUTH_RATE_AT_650_RPM:.0f} rpm of AZIMUTH at command "
                 f"650, and the region has no slow end — it jumps from forced-sine creep to hundreds\n"
                 f"  of rpm at the seam. Keep --tmax below {handoff}, or pass --allow-6step if you "
                 f"really mean to spin the pod up.")
    ident_cmd = args.cmd if isinstance(getattr(args, "cmd", None), int) else 0
    if ident_cmd and handoff and ident_cmd >= handoff and not args.allow_6step:
        sys.exit(f"identify --cmd {ident_cmd} reaches the 6-step seam (handoff {handoff}). Identify "
                 f"in forced sine; --allow-6step to override.")
    if args.idx_a == args.idx_b:
        sys.exit("--idx-a and --idx-b must differ: a differential needs two distinct ESCs")
    for name in ("kff", "vmax", "tol", "max_secs"):
        if hasattr(args, name) and getattr(args, name) <= 0:
            sys.exit(f"--{name.replace('_', '-')} must be > 0")
    for name in ("kp", "kd", "ki"):                      # gains may be 0 but not negative
        if hasattr(args, name) and getattr(args, name) < 0:
            sys.exit(f"--{name.replace('_', '-')} must be >= 0")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="subcmd", required=True)   # NOT "cmd": identify has a --cmd

    ident = sub.add_parser("identify", help="resolve sign_a/sign_b + azimuth gain from the encoder")
    ident.add_argument("--cmd", type=int, default=150, help="single-motor command for the run")
    ident.add_argument("--secs", type=float, default=2.0, help="how long to drive each motor")
    add_common(ident)
    ident.set_defaults(fn=cmd_identify)

    mv = sub.add_parser("move", help="servo to an absolute azimuth angle (deg from start)")
    mv.add_argument("--deg", required=True, help="target angle, or a comma-separated sequence")
    add_gains(mv)
    add_common(mv)
    mv.set_defaults(fn=cmd_move)

    hd = sub.add_parser("hold", help="hold an azimuth angle, optionally while the propeller runs")
    hd.add_argument("--deg", type=float, default=0.0)
    hd.add_argument("--thrust", type=float, default=0.0,
                    help="propeller command d to hold the angle against")
    add_gains(hd)
    add_common(hd)
    hd.set_defaults(fn=cmd_hold)

    args = ap.parse_args()
    _validate(args)
    try:
        args.fn(args)
    except Aborted as e:
        sys.exit(f"aborted: {e}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
