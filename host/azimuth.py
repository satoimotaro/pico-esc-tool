#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""azimuth — position servo for the two-motor DIFFERENTIAL azimuth pod.

posctl.py servos one motor against an AS5600 on that same rotor. This pod is different in
exactly one way that changes everything: the encoder reads the AZIMUTH AXIS, and the axis is
driven by the SUM of two motors through a differential (pico_esc.differential):

    s = (A + B) / 2  -> azimuth        d = (A - B) / 2  -> propeller

in MOTOR space. Note that in COMMAND space this pod is the other way round — measured, azimuth
tracks (A - B) and the propeller tracks (A + B) — because motor B is mounted mirrored and so runs
with sign_b = -1. The mixer folds that in; do not reason about raw commands without it.

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
# So steering by ONE MOTOR'S SPEED is unusable in 6-step, and drifting into it by accident is a
# hazard rather than merely a bad control result — it was fast enough to be called dangerous at the
# bench. That is what the rate guard below exists to prevent.
AZIMUTH_RATE_AT_650_RPM = 554.0

# BUT THE 6-STEP REGION IS USABLE AFTER ALL, VIA THE DIFFERENTIAL ITSELF — the conclusion above
# only rules out steering with a single motor's speed, which was never what the mechanism does.
# Azimuth rate is set by the DIFFERENCE of the two motors, not by either one's speed. Carry both
# motors deep inside 6-step (|command| ~600, well clear of the 515 seam) and trim the difference:
# the ESC never goes near its unusable low end, and the azimuth still creeps.
#
# MEASURED ON THIS POD 2026-08-12, continuous trim ramp at carrier -600, motors never stopped:
#     azimuth_rpm = 3.629 * delta + 178.5      (R2 = 0.912, delta = A - B in command counts)
# so with s = delta/2:
AZIMUTH_RPM_PER_S = 7.26          # rpm of azimuth per unit of s (command counts)
AZIMUTH_NULL_S    = -29.8         # s that actually holds still — the motors are not identical.
                                  # Refined from -24.6 (ramp fit) by direct hold measurement, and
                                  # it DRIFTS: three nulls in one 30 s run read +24.4, +33.9, +45.0
                                  # rpm, monotonically rising. Feedforward alone therefore creeps —
                                  # the position loop has to stay closed to hold a heading.
SIGN_B_MEASURED   = -1            # MEASURED on this pod: motor B is mounted mirrored, so a +ve
                                  # command on B turns the differential the other way. Left at the
                                  # +1 placeholder the mixer commands the two motors in OPPOSITE
                                  # directions — the maximum-azimuth configuration — while the
                                  # caller believes it is carrying them together. Every closed-loop
                                  # run on 2026-08-12 failed this way before it was spotted in a
                                  # trace showing A=-589 with B=+610. Re-identify per installation.
KFF_SENTINEL      = -1.0          # 'not given' — the right Kff depends on the mode
CARRIER_CMD       = 600           # |command| both motors are held at; 60+ counts clear of the seam
#
# Resolution is 0.435 deg per 50 Hz tick per command count — far finer than the 6 deg tolerance.
# The costs are real and must not be forgotten: A + B stays large, so THE PROPELLER ALWAYS RUNS in
# this mode (stationary pointing still needs forced sine), and the open-loop fit scatters by about
# +/-30 rpm, so the loop has to stay closed rather than run on feedforward.


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
def _suspect(r, args):
    """Is THIS sample untrustworthy enough to refuse to integrate it?

    Not the same question as `healthy`. EncReading.healthy also requires ML=0, but ML only says the
    AGC has saturated — the field is at the bottom of its usable range, not that the reading is
    wrong. Measured on this pod at ML=1: 10.8 hand-turned revolutions, zero dropped samples, a
    monotonic accumulator. What genuinely invalidates a sample is MD=0 (no magnet) or MH=1.

    This mattered: passing `not r.healthy` made EVERY tick a fault under the weak magnet, so
    PositionController never integrated position and the servo ran completely blind — and reported
    err 0.00, worst 0.00, a perfect-looking run. A blind controller that claims success is the exact
    failure this file's guards exist to prevent, wearing a different hat.
    """
    if r.md != 1 or r.mh != 0:
        return True
    return (r.ml != 0) and not args.allow_weak_magnet


def _feed_encv(ctrl, ev, base):
    """Drive the controller from the ON-DEVICE accumulator instead of unwrapping `enc` on the host.

    `PositionController.update_encoder` unwraps the 12-bit angle between 50 Hz ticks and REJECTS any
    delta beyond DELTA_FAULT_FRAC of half a revolution as a possible alias. That is the right call
    for a slow axis sampled slowly — but this pod reaches thousands of deg/s in a transient, and
    3000 deg/s is 683 counts per 20 ms tick, close enough to the threshold that samples get thrown
    away exactly when the azimuth is moving fastest. A rejected sample does not integrate, so
    position FREEZES at the worst possible moment, which is the signature of a loop that sometimes
    recovers and sometimes ends hundreds of degrees out.

    `encv` removes the problem rather than tuning around it: the RP2040 samples the AS5600 at
    ~1.25 kHz, so per-sample travel is far below half a revolution at any speed this mechanism can
    reach, and it unwraps into a signed accumulator on-device. The velocity comes from the same
    high-rate stream instead of being a 50 Hz difference of a noisy angle.
    """
    ctrl.pos_deg = ctrl.enc_sign * (ev.accum - base) * 360.0 / COUNTS_PER_REV
    ctrl.vel = ctrl.enc_sign * ev.rpm * 6.0          # signed mech rpm -> deg/s


def _servo_tick(drive, ctrl, carrier, args, rate_guard):
    """One closed-loop tick at a given carrier. Returns (MixResult, encoder reading).

    Split out of the main loop because SPIN-UP has to run the identical control law while the
    carrier is still ramping — the transition is where heading is lost, so it cannot be an
    open-loop prelude that the servo only cleans up afterwards.
    """
    if args.use_encv:
        ev = drive.read_encv()
        if ev is None:
            raise Aborted("encv read failed mid-run — disarming")
        if ev.md != 1:
            raise Aborted("azimuth magnet lost mid-run (md=0) — disarming")
        _feed_encv(ctrl, ev, args._encv_base)
        r = ev
    else:
        r = drive.read_enc()
        if r is None:
            raise Aborted("azimuth encoder read failed mid-run — disarming")
        ctrl.update_encoder(r.raw, DT, suspect=_suspect(r, args))
    rate_guard(ctrl)
    u, _ = ctrl.step(DT)
    return drive.send(u, carrier), r


def _make_rate_guard(args):
    """Abort if the pod itself exceeds --max-azimuth-rpm, debounced over consecutive ticks.

    The controller's own clamp bounds the COMMAND; this bounds the OUTCOME, which is what the
    operator called dangerous. Debounced because one aliased encoder tick must not stop a healthy
    run — the same reason the firmware's over-speed guard needs three live frames.
    """
    # The limit is PHASE-DEPENDENT. Spin-up has an unavoidable transient — both motors start from
    # rest and reach the carrier at slightly different rates, and the differential integrates that
    # difference straight into heading. Measured here at 94-220 rpm. Holding the steady-state limit
    # across it would abort every healthy run, so the guard is loosened for the transition only and
    # tightened again the moment the carrier is reached.
    state = {"n": 0, "limit": args.max_azimuth_rpm * 6.0}
    if args.dry_run and args.carrier:
        # SimEncEscHost is one rotor with the encoder on it, so under a carrier its "azimuth" is
        # really the rotor spinning at carrier speed and the guard would trip on a fiction. Dry-run
        # exists to exercise the code path, not to predict the pod (see the module docstring).
        print("# dry-run + carrier: rate guard disabled — the sim has no differential to slow it")
        return lambda ctrl: None

    def guard(ctrl):
        if abs(ctrl.vel) > state["limit"]:
            state["n"] += 1
            if state["n"] >= 3:
                raise Aborted(f"azimuth {ctrl.vel/6:.0f} rpm exceeds the {state['limit']/6:.0f} rpm "
                              f"limit for this phase on 3 consecutive ticks — disarming")
        else:
            state["n"] = 0

    def set_limit(rpm):
        state["limit"] = rpm * 6.0
        state["n"] = 0
    guard.set_limit = set_limit
    return guard


def _spinup(drive, ctrl, clock, args, carrier, rate_guard):
    """Bring the carrier from 0 to `carrier` with the azimuth loop ALREADY CLOSED.

    Why this is its own phase: below the seam the ESC is in forced sine, where s has almost no
    authority over the azimuth — the loop can see error it cannot correct, and an integrator left
    running there winds up and then dumps that windup the instant authority appears. So the
    integral is held at zero until the carrier is past the seam, and the crossing itself is made
    quickly, because time spent in the low-authority band is drift you can never get back.

    Modes, so the strategy can be chosen by measurement rather than argument:
      jump      — straight to the carrier. Shortest time in the dead band, biggest torque step.
      ramp      — linear 0 -> carrier over --spinup-secs. Gentlest, longest exposure.
      two-stage — snap past the seam, then ramp. The compromise this pod is expected to want.
    Returns the azimuth drift accrued during the transition, in degrees.
    """
    seam = HANDOFF_CMD.get(args.cross_up, 515)
    sgn = 1.0 if carrier > 0 else -1.0
    # AUTHORITY, not gentleness, is what bounds the transition. The ESCs start the two motors on
    # independent ramps, so there is always an interval with one turning and the other not — and in
    # air the differential sends nearly all of that to the azimuth, because the pod's bearings are a
    # far lighter load than the propeller. The mismatch is therefore up to a WHOLE MOTOR, which no
    # trim can cancel; the loop needs comparable authority or it just watches.
    # The ceiling on that authority is the seam: s must stay small enough that BOTH commands remain
    # in 6-step, i.e. |s| + |s_trim| <= |carrier| - seam.
    room = int(abs(carrier) - seam - abs(args.s_trim))
    spin_smax = min(args.spinup_smax, max(1, room))
    if spin_smax < args.spinup_smax:
        print(f"# spin-up authority capped at {spin_smax} (not {args.spinup_smax}): more would push "
              f"a motor below the seam at carrier {carrier:+.0f}")
    held_tmax = ctrl.tmax
    ctrl.tmax = spin_smax
    start_pos = ctrl.pos_deg
    rate_guard.set_limit(args.spinup_max_azimuth_rpm)
    t0 = clock.now()

    def profile(el):
        f = min(1.0, el / max(args.spinup_secs, 1e-3))
        if args.spinup_mode == "jump":
            return carrier
        if args.spinup_mode == "ramp":
            return sgn * abs(carrier) * f
        past = sgn * (seam + 40)                       # clear of the seam, then ramp the rest
        return past if el < 0.4 else past + (carrier - past) * min(1.0, (el - 0.4) /
                                                                  max(args.spinup_secs - 0.4, 1e-3))

    while True:
        el = clock.now() - t0
        if el >= args.spinup_secs:
            break
        tick = clock.now()
        c = profile(el)
        # No integral authority until the ESC can actually act on s.
        if abs(c) < seam:
            ctrl.ivel = 0.0
        _servo_tick(drive, ctrl, c, args, rate_guard)
        _pace(clock, tick)
    rate_guard.set_limit(args.max_azimuth_rpm)      # back to the steady-state limit
    ctrl.tmax = held_tmax
    ctrl.ivel = 0.0                                 # do not carry transition windup into the hold
    drift = ctrl.pos_deg - start_pos
    print(f"# spin-up [{args.spinup_mode}] {args.spinup_secs:.1f}s -> carrier {carrier:+.0f}: "
          f"azimuth drift {drift:+.1f} deg")
    return drift


def _run_servo(args, targets, thrust):
    """Shared servo loop for move/hold: cascade PID on s, feedforward d, both via the mixer."""
    host, clock = _mk_host(args)
    # In carrier mode the propeller command IS the carrier: it is what keeps both motors deep in
    # 6-step, so it overrides whatever `thrust` asked for, and tmax has to clear carrier + smax.
    carrier = float(args.carrier)
    if carrier:
        thrust = carrier
        print(f"# 6-STEP CARRIER {carrier:+.0f} on both motors — the propeller WILL run. "
              f"Azimuth authority smax={args.smax} (~{args.smax*AZIMUTH_RPM_PER_S:.0f} rpm max), "
              f"s_trim={args.s_trim:+.1f}")
    # The ceiling must clear carrier + trim + authority. Leaving s_trim out of it silently CLAMPS
    # the trim, so the null is never reached and the pod slews at the residual — which is exactly
    # how this failed on the bench: a -24.6 trim was clipped to -12 and the azimuth ran at 226 rpm
    # while nominally commanded to hold still.
    ceiling = int(abs(carrier) + abs(args.s_trim) + args.smax) if carrier else args.tmax
    mixer = DifferentialMixer(tmax=ceiling, sign_a=args.sign_a, sign_b=args.sign_b,
                              s_trim=args.s_trim, carrier=bool(carrier))
    drive = DualDrive(host, mixer, args.idx_a, args.idx_b, ceiling, clock)
    # The position loop's own output is s, so its thrust clamp IS the azimuth authority.
    args.tmax = args.smax if carrier else args.tmax
    # Kff maps a velocity setpoint straight to the command that produces it. Under a carrier that
    # is 1 / (deg/s per unit s), which we MEASURED — posctl's 0.47 belongs to a bare 930KV rotor,
    # a different plant entirely, and using it here would feed forward ~20x too much.
    ctrl = PositionController(args)
    if carrier:
        # NEVER hand off to a passive hold under a carrier. PositionController drops thrust to 0 and
        # ZEROES THE INTEGRAL once inside tol — correct for posctl, whose S1 detent holds position
        # with no power, and wrong here, where the pod is spinning and s=0 means "trim only", which
        # does not hold: the null drifts (+24 -> +45 rpm within one 30 s run). The loop would reach
        # tolerance, go open-loop, drift out, re-converge, and land anywhere. Measured final errors
        # across four identically-configured runs: -1.6, -9.8, -34.4, -233.3 deg.
        # The integral IS the thing that nulls steady-state drift, so it has to keep running.
        # Thresholds that can never be met disable the branch without forking the shared class.
        ctrl.tol = -1.0
        ctrl.hold_vel = -1.0
    stop = {"flag": False}

    def _bail(*_):
        stop["flag"] = True
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, _bail)

    try:
        drive.prepare()
        _guard_encoder(drive, args.allow_weak_magnet)
        drive.arm()
        args._encv_base = 0
        if args.use_encv:
            ev0 = drive.read_encv()
            if ev0 is None:
                sys.exit("--use-encv but the firmware has no `encv` — use --no-use-encv")
            args._encv_base = ev0.accum
        else:
            raw0, _ = _enc_deg(drive)
            ctrl.update_encoder(raw0, DT)
        ctrl.pos_deg = 0.0
        rate_guard = _make_rate_guard(args)
        if carrier:
            ctrl.set_target(0.0)                       # hold the heading we started at
            _spinup(drive, ctrl, clock, args, carrier, rate_guard)

        for target in targets:
            ctrl.set_target(target)
            t_end = clock.now() + args.max_secs
            worst = 0.0
            while not stop["flag"] and clock.now() < t_end:
                tick = clock.now()
                m, _r = _servo_tick(drive, ctrl, thrust, args, rate_guard)
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
    p.add_argument("--sign-b", type=int, default=SIGN_B_MEASURED, choices=(1, -1),
                   help="default is this pod's MEASURED value; re-identify per installation")
    p.add_argument("--carrier", type=int, default=0,
                   help=f"6-step carrier command on BOTH motors (e.g. {CARRIER_CMD}); 0 = forced-sine mode")
    p.add_argument("--smax", type=int, default=8,
                   help="azimuth authority in command counts (NOT tmax) — see AZIMUTH_RPM_PER_S")
    p.add_argument("--max-azimuth-rpm", type=float, default=90.0,
                   help="hard ceiling on pod rotation rate; --smax is checked against it")
    p.add_argument("--s-trim", type=float, default=AZIMUTH_NULL_S,
                   help="per-installation s that holds the azimuth still (measured, not assumed)")
    p.add_argument("--cross-up", type=int, default=36,
                   help="the ESC's sine_cross_up — sets where forced sine hands off to 6-step")
    p.add_argument("--allow-6step", action="store_true",
                   help="permit commands past the seam; the azimuth reaches hundreds of rpm there")
    p.add_argument("--use-encv", action="store_true", default=True,
                   help="feed the loop from the on-device de-aliased accumulator (default)")
    p.add_argument("--no-use-encv", dest="use_encv", action="store_false",
                   help="fall back to host-side unwrap of `enc` — aliases in fast transients")
    p.add_argument("--allow-weak-magnet", action="store_true",
                   help="run with ML=1 / AGC saturated — zero margin, see _guard_encoder")
    p.add_argument("-v", "--verbose", action="store_true")


def add_gains(p):
    """posctl's 930KV single-motor tune, as a STARTING POINT ONLY. Gains do not transfer between
    motors, and this is not even the same plant — the load here is the whole azimuth axis through
    a reduction, not a bare rotor. Tune in situ."""
    p.add_argument("--kp", type=float, default=3.0)
    p.add_argument("--kd", type=float, default=0.6)
    p.add_argument("--ki", type=float, default=0.4)
    p.add_argument("--kff", type=float, default=KFF_SENTINEL,
                   help="default: 1/(measured deg/s per unit s) under a carrier, else posctl's 0.47")
    p.add_argument("--vmax", type=float, default=200.0)   # deg/s of AZIMUTH, not rotor
    p.add_argument("--tol", type=float, default=6.0)
    p.add_argument("--tmin", type=float, default=0.0)
    p.add_argument("--max-secs", type=float, default=15.0)
    p.add_argument("--spinup-secs", type=float, default=3.0)
    p.add_argument("--spinup-smax", type=int, default=55,
                   help="azimuth authority DURING the transition; the mismatch there is up to a whole motor")
    p.add_argument("--spinup-max-azimuth-rpm", type=float, default=260.0,
                   help="looser rate limit DURING the transition; measured 94-220 rpm here")
    p.add_argument("--spinup-mode", choices=("jump","ramp","two-stage"), default="two-stage")


def _validate(args):
    args.tmax = abs(int(args.tmax))
    if args.tmax <= 0:
        sys.exit("--tmax must be > 0")
    # What must be bounded is the AZIMUTH RATE, not the raw command. Under a 6-step carrier both
    # commands sit far above the seam on purpose, so a command-magnitude limit would forbid the one
    # mode that works; and in forced sine a modest command can still be fast. Limit what actually
    # hurts.
    # Resolve the Kff sentinel here, before the >0 checks: the right feedforward depends on the
    # mode, and posctl's 0.47 belongs to a bare 930KV rotor — ~20x too much for the azimuth plant.
    if args.kff == KFF_SENTINEL:
        args.kff = (1.0 / (AZIMUTH_RPM_PER_S * 6.0)) if args.carrier else 0.47
    args.smax = abs(int(args.smax))
    rate = args.smax * AZIMUTH_RPM_PER_S
    if rate > args.max_azimuth_rpm:
        sys.exit(f"--smax {args.smax} implies ~{rate:.0f} rpm of AZIMUTH "
                 f"({rate*6:.0f} deg/s), over the --max-azimuth-rpm {args.max_azimuth_rpm} limit.\n"
                 f"  Measured on this pod: {AZIMUTH_RPM_PER_S:.2f} rpm of azimuth per unit s. "
                 f"Use --smax <= {int(args.max_azimuth_rpm / AZIMUTH_RPM_PER_S)}.")
    if args.carrier and abs(args.carrier) < HANDOFF_CMD.get(args.cross_up, 515) + 40:
        sys.exit(f"--carrier {args.carrier} is too close to the seam "
                 f"({HANDOFF_CMD.get(args.cross_up, 515)} at cross_up={args.cross_up}).\n"
                 f"  The carrier exists to hold BOTH motors clear of it; sitting near the seam is "
                 f"where the azimuth jumps to hundreds of rpm.")
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
