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

Keep-alive: one `enc` + one `thrust` per ~20 ms tick (50 Hz), well under the firmware's
500 ms spin deadman. Safety: --max-secs / --max-revs / --vel-abort aborts, encoder magnet-
health + unwrap-fault + expected-vs-measured stall + wrong-way guards, and on EVERY exit
path (normal, error, SIGINT/SIGTERM) it triple-sends `thrust 0` then `disarm`.

  python posctl.py move --deg 90 --dry-run
  python posctl.py step --seq 90,-90,360 --dry-run
  python posctl.py hold --deg 0 --dry-run
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import sys
import time
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import esctool  # noqa: E402  (esctool lives next to this file)
from esctool import EscHost  # noqa: E402
import autocal  # noqa: E402
from autocal import SimEscHost, ARM_WAIT  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
REPORT_DIR = os.path.join(HERE, "reports")

LOOP_HZ = 50.0
DT = 1.0 / LOOP_HZ                 # nominal control period (< 500 ms deadman)
COUNTS_PER_REV = 4096              # AS5600 12-bit
VEL_LP_ALPHA = 0.30               # velocity low-pass (fraction of instantaneous)
ENC_FAIL_MAX = 8                  # consecutive enc failures/faults -> abort
MEAN_WIN = 0.30                   # s, sliding window for the settle-mean test
FAULT_DT_MULT = 3.0               # measured dt > this*DT -> tick's enc delta is suspect
DELTA_FAULT_FRAC = 0.9            # |delta| > this*(half rev) -> implausible unwrap, reject
WRONGWAY_TICKS = 12               # push-follow ticks where motion opposes thrust -> abort
WRONGWAY_MIN_DPOS = 5.0           # deg; per-tick motion below this is ignored (noise)

# Firmware S1 full-scale: mechanical RPM at |thrust|=1000. This is the PLANT GAIN the
# feedforward (--kff) inverts, so it is tied to the firmware fixed-point constants:
#   eRPM      = Rcp * (1<<SINE_RCP_SHIFT) * (F_TIMER2/SINE_TICK_T2) / 65536 * (60/6)
#   mech RPM  = eRPM / POLE_PAIRS,   Rcp ≈ 2.047 * thrust  (bidir DShot mapping)
# with the ESC-firmware asm EQUs SINE_TICK_T2=4000, SINE_RCP_SHIFT=3, Timer2=4 MHz,
# POLE_PAIRS=7 => 356.97 mech RPM / kff 0.4669. Printed by
# ESC-firmware/tools/sim/sine_drive_model.py (stepper section) — keep these in sync if
# either the asm EQUs or the sim change (the startup note below flags a mismatch).
FULLSCALE_RPM = 357.0
KFF_COMPUTED = 1000.0 / (FULLSCALE_RPM * 6.0)   # thrust per deg/s implied by FULLSCALE_RPM (~0.467)
# Stall watchdog re-tuned for creep: a PID legitimately commands tiny sustained thrust at
# low speed, so a fixed counts-per-tick floor false-trips. Instead compare measured motion
# against the motion the commanded thrust SHOULD have produced (stepper: no floor).
STALL_TICKS = 40                  # ticks of powered-but-not-moving -> abort (~0.8 s)
STALL_MOVE_FRAC = 0.25            # measured < this * expected motion == "not moving"
STALL_MIN_EXPECTED = 2.0          # counts/tick of expected motion below which we don't judge


# ---------------------------------------------------------------------------
# Clock abstraction: real wall-clock for hardware, deterministic virtual time
# for --dry-run (so dt and integrated position are exactly reproducible).
# ---------------------------------------------------------------------------
class RealClock:
    def now(self):
        return time.time()

    def sleep(self, dt):
        if dt > 0:
            time.sleep(dt)


class SimClock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def sleep(self, dt):
        if dt > 0:
            self.t += dt


# ---------------------------------------------------------------------------
# Simulated ESC host for --dry-run: extends autocal.SimEscHost to also answer
# `thrust` (signed) and `enc`, integrating a toy signed-RPM motor model into a
# wrapped 0-4095 encoder count.  Deterministic (seeded).  NEVER opens a port.
# ---------------------------------------------------------------------------
class SimEncEscHost(SimEscHost):
    """SimEscHost + a signed-thrust S1 forced-commutation STEPPER model.

    thrust in [-1000,1000] -> signed target mech RPM = thrust * FULLSCALE_RPM / 1000
    (NO deadband, NO floor — the stepper drives at any rate incl. ~0). First-order
    rotor lag toward that target. At zero thrust the rotor does not coast freely: it
    holds on the nearest commutation DETENT via a damped detent spring (models S1's
    zero-speed holding torque), so a converged servo sits with a small limit cycle.
    Position is the time-integral of RPM, wrapped to a 12-bit count and reported
    through the real `enc|…` line format.  Deterministic (seeded).
    """

    TAU = 0.06                    # first-order rotor time constant (s)
    DETENTS_PER_REV = 42          # 12N14P, 7 pole-pairs * 6 sectors -> 42 detents/rev
    HOLD_STIFF = 12.0             # detent-spring rate at hold (1/s); pos -> nearest detent
    NOISE_RPM = 2.0               # under-power RPM jitter while driving
    IDLE_NOISE_RPM = 0.5          # always-on sensor/load dither (smoke-tests the hold metric)

    def __init__(self, clock, seed=1234, io_time=0.0, invert=False):
        super().__init__(seed)
        self.clock = clock
        self.io_time = io_time    # simulated serial round-trip per cmd (pacing self-test)
        # invert: model hardware where +thrust drives the encoder count NEGATIVE
        # (the bench wiring/DIR/3D convention that broke the naive controller).
        self.enc_hw_sign = -1 if invert else 1
        self.thrust = 0
        self.rpm = 0.0            # signed mechanical RPM
        self.pos_counts = 0.0    # continuous (unwrapped) encoder counts
        self.raw0 = self._rng.randrange(COUNTS_PER_REV)   # random magnet offset
        self.last_t = clock.now()

    def _target_rpm(self, thrust):
        # Stepper: linear thrust->RPM, no deadband, no floor.
        return thrust * FULLSCALE_RPM / 1000.0

    def _advance(self, now):
        dt = now - self.last_t
        self.last_t = now
        if dt <= 0:
            return
        if abs(self.thrust) < 1:
            # zero thrust: hold on the nearest detent with a damped spring (holding torque)
            step = COUNTS_PER_REV / self.DETENTS_PER_REV
            nearest = round(self.pos_counts / step) * step
            err_counts = nearest - self.pos_counts
            self.rpm = self.HOLD_STIFF * err_counts / COUNTS_PER_REV * 60.0
            self.rpm += self._rng.gauss(0.0, self.IDLE_NOISE_RPM)
        else:
            tgt = self._target_rpm(self.thrust)
            alpha = 1.0 - math.exp(-dt / self.TAU)
            self.rpm += (tgt - self.rpm) * alpha
            self.rpm += self._rng.gauss(0.0, self.NOISE_RPM)
            self.rpm += self._rng.gauss(0.0, self.IDLE_NOISE_RPM)
        self.pos_counts += self.rpm / 60.0 * COUNTS_PER_REV * dt

    def cmd(self, line, timeout=30.0):
        line = line.strip()
        head, _, arg = line.partition(" ")
        if head == "thrust":
            if self.io_time:
                self.clock.sleep(self.io_time)   # simulate serial round-trip work
            self._advance(self.clock.now())
            self.thrust = int(arg.split()[-1]) if arg.split() else 0
            return []
        if head == "enc":
            if self.io_time:
                self.clock.sleep(self.io_time)
            self._advance(self.clock.now())
            raw = int(round(self.raw0 + self.enc_hw_sign * self.pos_counts)) % COUNTS_PER_REV
            deg = raw * 360.0 / COUNTS_PER_REV
            # raw|ang|deg|md|ml|mh|agc|mag — healthy magnet (md=1, ml=mh=0)
            return [f"enc|{raw}|{raw}|{deg:.1f}|1|0|0|64|1800"]
        if head == "arm":
            self.thrust = 0
            self.rpm = 0.0
            self.pos_counts = 0.0
            self.last_t = self.clock.now()
        elif head in ("disarm", "run", "disconnect"):
            self.thrust = 0
            self.rpm = 0.0
        return super().cmd(line, timeout)


# ---------------------------------------------------------------------------
# Drive session: prepare -> arm bidir -> keep-alive thrust; ALWAYS disarms.
# Mirrors autocal.DriveSession / drive_hold.py but for the signed `thrust` cmd.
# ---------------------------------------------------------------------------
class EncReading:
    __slots__ = ("raw", "md", "ml", "mh", "agc", "mag")

    def __init__(self, raw, md, ml, mh, agc, mag):
        self.raw, self.md, self.ml, self.mh, self.agc, self.mag = raw, md, ml, mh, agc, mag

    @property
    def healthy(self):
        return self.md == 1 and self.ml == 0 and self.mh == 0


class PosDrive:
    """One host + one ESC index.  send_thrust is the single thrust choke point."""

    def __init__(self, host, idx, tmax, clock, verbose=True):
        self.host = host
        self.idx = idx
        self.tmax = tmax
        self.clock = clock
        self.verbose = verbose
        self.armed = False

    def _log(self, msg):
        if self.verbose:
            print(msg)

    def prepare(self):
        # release any held bootloader session so the ESC app is running before arm
        for c in ("run", "disconnect"):
            try:
                self.host.cmd(c, timeout=5)
            except Exception:
                pass
        self.clock.sleep(0.4)

    def arm(self):
        self._log("# arming (bidir)…")
        self.host.cmd(f"arm {self.idx} bidir", timeout=6)
        self.armed = True
        self.clock.sleep(ARM_WAIT)

    def disarm(self):
        if not self.armed:
            return
        for _ in range(3):                       # triple thrust 0 (mirror drive_hold)
            try:
                self.send_thrust(0)
            except Exception:
                pass
        for c in (f"disarm {self.idx}", "disarm"):
            try:
                self.host.cmd(c, timeout=2)
            except Exception:
                pass
        self.armed = False
        self._log("# DISARMED")

    # -- the ONLY place a thrust value is sent to the ESC --
    def send_thrust(self, u):
        u = int(u)
        u = max(-1000, min(1000, u))
        u = max(-self.tmax, min(self.tmax, u))   # single thrust-ceiling choke point
        try:
            self.host.cmd(f"thrust {self.idx} {u}", timeout=2)
        except Aborted:
            raise
        except Exception as e:                   # never leak a raw traceback mid-drive
            raise Aborted(f"thrust command failed ({e}) — disarming")
        return u

    def read_enc(self):
        """Return EncReading or None on any read/parse failure."""
        try:
            lines = self.host.cmd("enc", timeout=2)
        except Exception:
            return None
        for ln in lines:
            if ln.startswith("enc|"):
                p = ln.split("|")
                try:
                    return EncReading(int(p[1]), int(p[4]), int(p[5]),
                                      int(p[6]), int(p[7]), int(p[8]))
                except (ValueError, IndexError):
                    return None
        return None


# ---------------------------------------------------------------------------
# Cascade PID position servo (for S1 forced-commutation stepper mode).
# ---------------------------------------------------------------------------
class PIDServo:
    """Cascade PID: outer position loop -> velocity setpoint -> inner thrust law.

    Assumes the ESC can creep and hold (BlueGill S1 sine mode), so a real velocity servo
    works instead of pulse-and-settle:

      outer:  vel_sp = clamp(Kp*err - Kd*vel, +-vmax)          (deg/s)
      inner:  thrust = clamp(Kff*vel_sp + Ki*∫(vel_sp - vel), +-tmax)

    Kff maps a velocity setpoint straight to the thrust that produces it (firmware full
    scale), so feedforward carries the motion and the integral only trims stiction; the
    integral is back-calculated on saturation (anti-windup). Inside tol AND slow, thrust
    is 0 and the ESC HOLDS on its detent. (Name kept as a drop-in for the run loop.)
    """

    def __init__(self, opts):
        self.tmax = opts.tmax
        self.tmin = opts.tmin
        self.tol = opts.tol
        self.kp = opts.kp
        self.kd = opts.kd
        self.ki = opts.ki
        self.kff = opts.kff
        self.vmax = opts.vmax
        self.hold_vel = getattr(opts, "hold_vel", 40.0)   # deg/s below which "slow enough to hand to hold"
        # encoder / velocity state (interface the run loop + guards depend on)
        self.enc_sign = 1          # +1/-1: maps raw counts so +thrust => +measured motion
        self.prev_raw = None
        self.pos_deg = 0.0         # continuous position, zeroed at start
        self.vel = 0.0             # low-passed velocity (deg/s)
        self.last_delta_counts = 0
        self.target = 0.0
        self.holding = False       # True once within tol and slow (ESC holds) — run loop reads this
        self.ivel = 0.0            # inner-loop velocity-error integral (thrust units)

    def set_target(self, deg):
        self.target = deg
        self.ivel = 0.0
        self.holding = False

    def update_encoder(self, raw, dt, suspect=False):
        """Unwrap 0-4095 -> continuous position + low-passed velocity.

        Returns "ok" or "fault". The modulo unwrap is only valid when true travel is
        < half a rev per tick; a stalled/slow tick (suspect dt) or a near-half-rev
        delta is ambiguous (could be an aliased wrong-direction jump), so we reject
        it instead of corrupting pos_deg / flipping the velocity sign (M2).
        """
        if self.prev_raw is None:
            self.prev_raw = raw
            self.last_delta_counts = 0
            return "ok"
        delta = ((raw - self.prev_raw + COUNTS_PER_REV // 2) % COUNTS_PER_REV) - COUNTS_PER_REV // 2
        if suspect or abs(delta) > DELTA_FAULT_FRAC * (COUNTS_PER_REV // 2):
            self.prev_raw = raw            # resync baseline, but DON'T integrate it
            self.last_delta_counts = 0     # hold last velocity; caller counts the fault
            return "fault"
        self.prev_raw = raw
        self.last_delta_counts = delta   # raw magnitude (for the stall watchdog)
        # enc_sign folds the wiring/DIR convention in so the controller always sees
        # "+thrust => +motion" (set by direction auto-cal / --invert-encoder).
        dpos = self.enc_sign * delta * 360.0 / COUNTS_PER_REV
        self.pos_deg += dpos
        if dt > 0:
            inst = dpos / dt
            self.vel += VEL_LP_ALPHA * (inst - self.vel)
        return "ok"

    def step(self, dt):
        """Cascade PID tick. Returns (signed thrust, velocity setpoint). PosDrive re-clamps
        thrust to tmax; the ESC (S1) holds on its detent when thrust is 0.
        """
        err = self.target - self.pos_deg

        # Inside tol AND slow -> hand off to the ESC's holding torque (thrust 0). This is a
        # stable resting state, not a give-up: the detent hold keeps position without power.
        if abs(err) <= self.tol and abs(self.vel) <= self.hold_vel:
            self.holding = True
            self.ivel = 0.0                              # drop integral so re-entry starts clean
            return 0.0, 0.0
        self.holding = False

        # Outer position loop -> velocity setpoint (deg/s), clamped to +-vmax.
        vel_sp = self.kp * err - self.kd * self.vel
        vel_sp = max(-self.vmax, min(self.vmax, vel_sp))

        # Inner loop: feedforward (Kff maps vel_sp -> thrust directly) + integral trim on the
        # velocity error, with back-calculation anti-windup on the thrust clamp.
        u_ff = self.kff * vel_sp
        self.ivel += self.ki * (vel_sp - self.vel) * dt
        u = u_ff + self.ivel
        u_clamped = max(-self.tmax, min(self.tmax, u))
        if u != u_clamped:                              # saturated -> unwind the integral
            self.ivel = max(-self.tmax, min(self.tmax, u_clamped - u_ff))
            u = u_clamped
        return u, vel_sp


# ---------------------------------------------------------------------------
# Segment / run metrics.
# ---------------------------------------------------------------------------
class SegMetrics:
    def __init__(self, target, start_pos):
        self.target = target
        self.start_pos = start_pos
        self.sign = 1.0 if target >= start_pos else -1.0
        self.settle_t = None       # first time the mean position sits within tol
        self.peak_overshoot = 0.0  # deg beyond target in the direction of travel
        self.samples = deque()     # (t, pos) short window for the settle-mean test
        self.post = []             # ALL positions after settle_t (hold-hunt metric, m6)
        self.last_pos = start_pos  # last measured position in this segment
        self._cvg_since = None

    def update(self, t, pos, target, tol, settle_dwell):
        self.last_pos = pos
        # overshoot: travel past the target in the commanded direction
        over = (pos - target) * self.sign
        if over > self.peak_overshoot:
            self.peak_overshoot = over
        # short window (last MEAN_WIN seconds) -> current mean position, for settle test
        self.samples.append((t, pos))
        while self.samples and t - self.samples[0][0] > MEAN_WIN:
            self.samples.popleft()
        mean = sum(p for _, p in self.samples) / len(self.samples)
        within = abs(mean - target) <= tol
        if within:
            if self._cvg_since is None:
                self._cvg_since = t
                if self.settle_t is None:
                    self.settle_t = t
        else:
            self._cvg_since = None
        # accumulate the whole post-settle history for an honest limit-cycle amplitude
        # (a slow hold-hunt is under-reported by the 0.3 s settle window)
        if self.settle_t is not None:
            self.post.append(pos)
        return within and (t - self._cvg_since) >= settle_dwell

    def _hold_positions(self):
        return self.post if self.post else [p for _, p in self.samples]

    def limit_cycle_amp(self):
        ps = self._hold_positions()
        return (max(ps) - min(ps)) if ps else 0.0

    def steady_mean_err(self):
        ps = self._hold_positions()
        if not ps:
            return self.target - self.last_pos
        return self.target - sum(ps) / len(ps)


# ---------------------------------------------------------------------------
# Control run.
# ---------------------------------------------------------------------------
class Aborted(RuntimeError):
    pass


def _pace(clock, tick_start):
    """Fixed-cadence sleep (B1): sleep only the remainder of DT after this tick's work,
    so the period holds ~LOOP_HZ instead of growing by the serial round-trip each tick
    (which would inflate the control dt and, worse, let the 500 ms deadman zero the
    motor between commands)."""
    clock.sleep(max(0.0, DT - (clock.now() - tick_start)))


def _read_valid_enc(drive):
    """Read one encoder sample; return EncReading or None if unreadable/unhealthy."""
    enc = drive.read_enc()
    return enc if (enc is not None and enc.healthy) else None


def calibrate_direction(drive: PosDrive, clock, opts, baseline_raw):
    """Probe the wiring/DIR convention: command a bounded FORWARD thrust briefly and
    measure the net raw-encoder travel (guarded unwrap).  Returns +1 if +thrust moved
    the count positive, -1 if it moved it negative.  Aborts if the motor did not move
    at all (which doubles as a 'does the motor actually start?' check).

    The probe thrust and duration are bounded; the caller disarms on any raised abort.
    """
    probe_thrust = min(opts.tmax, max(opts.tmin, 30))   # S1 creeps: a small probe suffices
    ticks = max(1, round(opts.probe_secs / DT))
    prev = baseline_raw
    net_deg = 0.0
    fails = 0
    for _ in range(ticks):
        tick_start = clock.now()
        enc = _read_valid_enc(drive)
        if enc is None:
            fails += 1
            drive.send_thrust(0)
            if fails >= ENC_FAIL_MAX:
                raise Aborted("direction calibration failed: encoder unreadable during probe")
            _pace(clock, tick_start)
            continue
        delta = ((enc.raw - prev + COUNTS_PER_REV // 2) % COUNTS_PER_REV) - COUNTS_PER_REV // 2
        prev = enc.raw
        if abs(delta) <= DELTA_FAULT_FRAC * (COUNTS_PER_REV // 2):   # ignore implausible jumps
            net_deg += delta * 360.0 / COUNTS_PER_REV
        drive.send_thrust(probe_thrust)                              # keep-alive forward probe
        _pace(clock, tick_start)
    drive.send_thrust(0)

    if abs(net_deg) < opts.probe_min_deg:
        raise Aborted(f"direction calibration failed: motor did not respond to thrust "
                      f"(|Δ|={abs(net_deg):.1f}° < {opts.probe_min_deg}°) — check "
                      f"power / startup_power / wiring")
    enc_sign = 1 if net_deg > 0 else -1
    arrow = "+encoder" if enc_sign > 0 else "-encoder (inverted)"
    print(f"# direction cal: +thrust -> {arrow}; enc_sign={enc_sign} "
          f"(probe Δ={net_deg:+.1f}° at thrust {probe_thrust})")
    return enc_sign


def _rebaseline(drive, ctrl, clock, settle_secs=0.4):
    """Coast to rest (keep-alive zero) then zero the continuous position on the current
    raw count, so the control loop's targets are relative to the post-probe rest pose."""
    ticks = max(1, round(settle_secs / DT))
    last = None
    for _ in range(ticks):
        tick_start = clock.now()
        drive.send_thrust(0)
        enc = _read_valid_enc(drive)
        if enc is not None:
            last = enc
        _pace(clock, tick_start)
    if last is None:
        last = _read_valid_enc(drive)
    if last is None:
        raise Aborted("encoder unreadable after direction probe")
    ctrl.prev_raw = last.raw
    ctrl.pos_deg = 0.0
    ctrl.vel = 0.0
    return last.raw


def run_segments(drive: PosDrive, ctrl: PIDServo, clock, writer, opts,
                 targets, hold=False):
    """Drive through a list of position targets (relative deltas).  For `hold`
    the single target runs until --max-secs.  Returns (reason, [SegMetrics])."""
    # magnet-health gate: refuse to start on a bad/absent magnet
    first = drive.read_enc()
    if first is None:
        raise Aborted("no encoder reading (AS5600 not responding) — refusing to start")
    if not first.healthy:
        raise Aborted(f"magnet health check failed (md={first.md} ml={first.ml} mh={first.mh}) "
                      f"— refusing to start")
    print(f"# encoder OK (md=1, ml=mh=0, agc={first.agc}, mag={first.mag}); "
          f"start raw={first.raw}")

    # direction convention: +thrust must map to +measured motion or the loop runs away
    if opts.invert_encoder:
        ctrl.enc_sign = -1
        print("# direction: --invert-encoder forced enc_sign=-1 (auto-cal skipped)")
    elif opts.no_autocal:
        ctrl.enc_sign = 1
        print("# direction: --no-autocal assumes enc_sign=+1 (auto-cal skipped)")
    else:
        ctrl.enc_sign = calibrate_direction(drive, clock, opts, first.raw)
    _rebaseline(drive, ctrl, clock)

    t0 = clock.now()
    prev_tick = t0
    enc_fails = 0
    stuck_ticks = 0
    wrongway_ticks = 0
    prev_sent = 0                 # thrust commanded last tick (for the wrong-way guard)
    metrics = []

    seg_i = 0
    base = ctrl.pos_deg
    target = base + targets[0]
    ctrl.set_target(target)
    seg = SegMetrics(target, base)
    metrics.append(seg)
    print(f"# segment 1/{len(targets)}: target {target:+.1f} deg")

    reason = "max-secs"
    while True:
        tick_start = clock.now()
        t = tick_start - t0
        dt = tick_start - prev_tick          # measured period, for control math only
        prev_tick = tick_start
        if dt <= 0:
            dt = DT
        if t > opts.max_secs:
            reason = "max-secs"
            break

        # a tick that ran far longer than DT (serial stall, GC, USB hiccup) makes the
        # encoder delta ambiguous -> flag this sample suspect so the unwrap rejects it.
        suspect_dt = dt > FAULT_DT_MULT * DT

        enc = drive.read_enc()
        if enc is None or not enc.healthy:
            enc_fails += 1
            stuck_ticks = 0
            drive.send_thrust(0)              # feed the deadman with a safe zero
            if enc_fails >= ENC_FAIL_MAX:
                raise Aborted(f"{enc_fails} consecutive encoder read failures")
            _pace(clock, tick_start)
            continue

        status = ctrl.update_encoder(enc.raw, dt, suspect=suspect_dt)
        if status == "fault":
            # implausible jump / stalled tick: don't integrate it, hold last velocity,
            # command zero, and count it toward the failure budget (M2).
            enc_fails += 1
            stuck_ticks = 0
            drive.send_thrust(0)
            if enc_fails >= ENC_FAIL_MAX:
                raise Aborted(f"{enc_fails} unusable encoder samples "
                              f"(implausible jumps / stalled ticks)")
            _pace(clock, tick_start)
            continue
        enc_fails = 0

        # wrong-way / positive-feedback guard (faster than --max-revs): the controller
        # only ever pushes TOWARD the target, so this tick's motion (post enc_sign)
        # should agree with the thrust we sent last tick. If instead the rotor moved
        # AWAY while we were actively pushing, the direction convention is wrong
        # (mis-cal / inverted wiring) and the loop is a positive-feedback runaway.
        # Coast ticks (prev_sent ~ 0) are skipped so the count survives push/coast
        # cycling; a correct push clears it.
        dpos = ctrl.enc_sign * ctrl.last_delta_counts * 360.0 / COUNTS_PER_REV
        if abs(prev_sent) >= opts.tmin and abs(dpos) > WRONGWAY_MIN_DPOS:
            if (dpos > 0) != (prev_sent > 0):
                wrongway_ticks += 1
                if wrongway_ticks >= WRONGWAY_TICKS:
                    drive.send_thrust(0)
                    raise Aborted("wrong-way runaway: rotor moved opposite to thrust for "
                                  f"{wrongway_ticks} pushes — direction cal wrong / inverted "
                                  "wiring? (try --invert-encoder, or drop --no-autocal)")
            else:
                wrongway_ticks = 0

        u, vel_sp = ctrl.step(dt)

        # safety rails
        if abs(ctrl.pos_deg - base) / 360.0 > opts.max_revs:
            drive.send_thrust(0)
            raise Aborted(f"runaway: |travel| exceeded --max-revs ({opts.max_revs})")
        if abs(ctrl.vel) > opts.vel_abort:
            drive.send_thrust(0)
            raise Aborted(f"over-velocity: |vel|={ctrl.vel:.0f} > --vel-abort "
                          f"({opts.vel_abort}) deg/s")

        sent = drive.send_thrust(u)

        # frozen-encoder stall watchdog (M3), re-tuned for creep: compare the measured
        # motion to what the commanded thrust SHOULD have produced (stepper: RPM is linear
        # in thrust, no floor). A tiny sustained creep thrust legitimately moves little, so
        # we only judge when the expected motion is non-trivial, and trip if the measured
        # motion is far below it — a powered runaway the pos/vel guards can't see (~0 motion).
        exp_counts = abs(sent) * FULLSCALE_RPM / 1000.0 / 60.0 * COUNTS_PER_REV * dt
        if (abs(sent) >= opts.tmin and exp_counts >= STALL_MIN_EXPECTED
                and abs(ctrl.last_delta_counts) < STALL_MOVE_FRAC * exp_counts):
            stuck_ticks += 1
            if stuck_ticks >= STALL_TICKS:
                drive.send_thrust(0)
                raise Aborted(f"stall: |thrust|>={opts.tmin} for {stuck_ticks} ticks "
                              f"(~{stuck_ticks * DT:.1f}s) moved <{STALL_MOVE_FRAC:.0%} of "
                              f"expected (~{exp_counts:.1f} counts/tick)")
        else:
            stuck_ticks = 0
        prev_sent = sent

        writer.writerow([f"{t:.4f}", enc.raw, f"{ctrl.pos_deg:.3f}", f"{ctrl.vel:.2f}",
                         f"{ctrl.target:.3f}", f"{vel_sp:.2f}", sent])

        settled = seg.update(t, ctrl.pos_deg, ctrl.target, opts.tol, opts.settle_dwell)
        # a move segment ends when the mean is within tol (settled) OR the servo is within
        # tol and slow enough that it has handed off to the ESC's detent hold (ctrl.holding).
        if (settled or ctrl.holding) and not hold:
            fin = ctrl.target - ctrl.pos_deg
            tag = "settled" if settled else "holding (within tol, ESC detent hold)"
            print(f"#   {tag}: t={t:.2f}s final_err={fin:+.1f}deg "
                  f"peak_overshoot={seg.peak_overshoot:.1f}deg")
            seg_i += 1
            if seg_i >= len(targets):
                reason = "converged"
                break
            base = ctrl.pos_deg
            target = base + targets[seg_i]
            ctrl.set_target(target)
            stuck_ticks = 0
            seg = SegMetrics(target, base)
            metrics.append(seg)
            print(f"# segment {seg_i + 1}/{len(targets)}: target {target:+.1f} deg")

        _pace(clock, tick_start)

    return reason, metrics


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
    p.add_argument("--kp", type=float, default=6.0,
                   help="outer position gain: velocity setpoint (deg/s) per deg of error (default 6)")
    p.add_argument("--kd", type=float, default=0.3,
                   help="outer damping: subtract Kd*vel from the velocity setpoint (default 0.3)")
    p.add_argument("--ki", type=float, default=0.4,
                   help="inner integral on velocity error -> thrust, anti-windup (default 0.4)")
    p.add_argument("--kff", type=float, default=0.47,
                   help="inner feedforward: thrust per (deg/s) of velocity setpoint. Firmware full "
                        "scale ~0.47 (see tools/sim/sine_drive_model.py stepper section) (default 0.47)")
    p.add_argument("--vmax", type=float, default=400.0,
                   help="velocity setpoint clamp, deg/s (default 400 ~ 66 RPM; keep low for gentle creep)")
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
                   help="min encoder travel during probe to accept a direction, deg (default 3)")
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
