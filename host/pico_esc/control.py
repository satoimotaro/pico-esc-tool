# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""pico_esc.control — host-side cascade position servo (AS5600 + ESC), for BlueGill S1 sine.

PositionController (aliased PIDServo for the run loop) is the outer-position -> velocity-setpoint
-> inner-thrust cascade; run_segments is the guarded 50 Hz drive loop (keep-alive, magnet-health
gate, direction auto-cal confirm-arc, wrong-way / stall / over-temp / over-velocity aborts);
calibrate_direction / _rebaseline set the +thrust->+motion convention. Moved verbatim from
posctl.py — the control math, guard constants, printed strings, and (with the seeded sim)
the cmd() call order are all byte-for-byte unchanged.
"""
from __future__ import annotations

from collections import deque

from .constants import COUNTS_PER_REV, FULLSCALE_RPM, KFF_COMPUTED  # noqa: F401 (re-exported)
from .drive import Aborted, PosDrive

LOOP_HZ = 50.0
DT = 1.0 / LOOP_HZ                 # nominal control period (< 500 ms deadman)
VEL_LP_ALPHA = 0.30               # velocity low-pass (fraction of instantaneous)
ENC_FAIL_MAX = 8                  # consecutive enc failures/faults -> abort
MEAN_WIN = 0.30                   # s, sliding window for the settle-mean test
FAULT_DT_MULT = 3.0               # measured dt > this*DT -> tick's enc delta is suspect
DELTA_FAULT_FRAC = 0.9            # |delta| > this*(half rev) -> implausible unwrap, reject
WRONGWAY_TICKS = 12               # push-follow ticks where motion opposes thrust -> abort
WRONGWAY_MIN_DPOS = 5.0           # deg; per-tick motion below this is ignored (noise)
TELE_EVERY_S = 0.5                # temperature poll period while driving (bidir DShot `tele`)
PROBE_STEP = 40                   # direction-cal: thrust step when ramping up to find break-away
PROBE_MAX_THRUST = 250            # direction-cal: cap on the ramped probe thrust (also bounded by --tmax)

# FULLSCALE_RPM (firmware S1 full-scale plant gain) and KFF_COMPUTED are imported from
# pico_esc.constants (shared with the sim) and re-exported above.
# Stall watchdog re-tuned for creep: a PID legitimately commands tiny sustained thrust at
# low speed, so a fixed counts-per-tick floor false-trips. Instead compare measured motion
# against the motion the commanded thrust SHOULD have produced (stepper: no floor).
STALL_TICKS = 40                  # ticks of powered-but-not-moving -> abort (~0.8 s)
STALL_MOVE_FRAC = 0.25            # measured < this * expected motion == "not moving"
STALL_MIN_EXPECTED = 2.0          # counts/tick of expected motion below which we don't judge


# ---------------------------------------------------------------------------
# Cascade PID position servo (for S1 forced-commutation stepper mode).
# ---------------------------------------------------------------------------
class PositionController:
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


# PIDServo is the historical name the run loop / posctl CLI import.
PIDServo = PositionController


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
    """Probe the wiring/DIR convention: command a FORWARD thrust and measure the net
    raw-encoder travel (guarded unwrap).  Returns +1 if +thrust moved the count positive,
    -1 if negative.  Aborts if the motor did not move at all even at the ramp cap.

    RAMPS the probe thrust from --tmin up to min(--tmax, PROBE_MAX_THRUST) in PROBE_STEP
    steps and takes the sign of the net travel only AFTER a clear confirm arc
    (>= --probe-confirm-deg). A strongly-cogging motor's first few degrees can settle EITHER
    way as it snaps to the nearest cog, so a tiny --probe-min-deg break-away is NOT a reliable
    direction (it flipped +/- run-to-run on the 12N14P). Requiring a >>cog-pitch arc before
    deciding makes the sign robust. Bounded in thrust and time; caller disarms on any abort.
    """
    start = max(opts.tmin, 30)
    cap = min(int(opts.tmax), max(start, PROBE_MAX_THRUST))
    confirm = max(float(opts.probe_min_deg), float(opts.probe_confirm_deg))
    ticks_per = max(1, round(opts.probe_secs / DT))
    prev = baseline_raw
    net_deg = 0.0
    fails = 0
    probe_thrust = start
    # bound total time: one probe_secs window per ramp level, plus a couple spare
    max_ticks = ticks_per * (int((cap - start) / PROBE_STEP) + 3)
    level_ticks = 0
    for _ in range(max_ticks):
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
        if abs(delta) <= DELTA_FAULT_FRAC * (COUNTS_PER_REV // 2):       # ignore implausible jumps
            net_deg += delta * 360.0 / COUNTS_PER_REV
        if abs(net_deg) >= confirm:                                      # clear arc -> sign trusted
            break
        # this level's window elapsed without a confirmed arc -> step the thrust up
        level_ticks += 1
        if level_ticks >= ticks_per and probe_thrust < cap:
            probe_thrust = min(cap, probe_thrust + PROBE_STEP)
            level_ticks = 0
            net_deg = 0.0                                               # re-measure the arc cleanly at the new level
        drive.send_thrust(probe_thrust)                                 # keep-alive forward probe
        _pace(clock, tick_start)
    drive.send_thrust(0)

    if abs(net_deg) < confirm:
        raise Aborted(f"direction calibration failed: motor did not rotate a clear arc "
                      f"(|Δ|={abs(net_deg):.1f}° < {confirm:.0f}° even at thrust {probe_thrust}). "
                      f"Raise sine_hold_amp/sine_amp_max or --tmax, check power/wiring, or pass "
                      f"--invert-encoder / --no-autocal if you already know the direction.")
    enc_sign = 1 if net_deg > 0 else -1
    arrow = "+encoder" if enc_sign > 0 else "-encoder (inverted)"
    print(f"# direction cal: +thrust -> {arrow}; enc_sign={enc_sign} "
          f"(rotated {net_deg:+.0f}° at thrust {probe_thrust})")
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


def run_segments(drive: PosDrive, ctrl: PositionController, clock, writer, opts,
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
    next_tele = t0                # first temperature poll fires on the first tick
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

        # temperature watch (bidir DShot `tele`): poll ~every TELE_EVERY_S, print it live, and
        # abort if it reaches --max-temp. The ESC has NO current sensing, so at hold / low speed
        # the sine amplitude drives near-DC winding current with only this thermal backstop —
        # raise sine_amp_max / sine_hold_amp on the bench WATCHING this number. Polled right after
        # the thrust keep-alive so the 500 ms deadman is never starved; a missing sample is skipped.
        if opts.max_temp and tick_start >= next_tele:
            next_tele = tick_start + TELE_EVERY_S
            temp = drive.read_temp()
            if temp is not None:
                drive.last_temp = temp
                drive.peak_temp = temp if drive.peak_temp is None else max(drive.peak_temp, temp)
                print(f"#   temp={temp}C  (t={t:.1f}s vel={ctrl.vel:+.0f} err={ctrl.target - ctrl.pos_deg:+.0f})")
                if temp >= opts.max_temp:
                    drive.send_thrust(0)
                    raise Aborted(f"over-temperature: ESC {temp}C >= --max-temp {opts.max_temp:.0f}C "
                                  f"— lower sine_amp_max / sine_hold_amp (no current sensing to trip on)")

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
