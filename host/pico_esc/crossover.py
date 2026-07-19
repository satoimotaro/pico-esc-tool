# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""pico_esc.crossover — one proven measurement loop for the S3 sine<->6-step crossover.

Both xover_debug (the hardware connection/handoff checker) and autocal's tune-crossover-lock
phase need to ramp the ESC command up through the crossover into 6-step and measure the lock
quality. This module is the single source of truth for that loop (via the shared `_Ticker`) so
the callers cannot drift apart.

Lock quality is the SLIP ratio:

    slip = |telemetry_mech_RPM| / |encoder_mech_RPM|

which is ~1.0 at true BEMF lock. BOTH sides are already MECHANICAL RPM: the RP2040 firmware divides
the DShot eRPM by the motor pole pairs (ESC_MOTOR_POLES/2) before sending `tele`, and the AS5600 is a
2-pole shaft magnet (1 cycle / mechanical rev). Do NOT divide either side by POLE_PAIRS again — that
was a latent double-division that made a real lock read as ~0.143 (=1/7). In forced sine the
telemetry is stale, and a mis-tuned 6-step lock commutates off real BEMF (slip away from 1). Auto-cal
minimises |slip - 1| across a (comm_timing, demag_compensation) grid.

TWO measurement modes:
  * measure_crossover_lock          — hold at the TOP of the ramp and measure.
  * measure_crossover_lock_lowspeed — cross into 6-step, then DESCEND the command (navigating by
                                      telemetry) to a lower speed and hold there.

DE-ALIASED ENCODER (the fix that made reverse honest): the rotor really spins FAST in 6-step (~6000-
7000 mech), far past the 50 Hz host sampler's ~1350 mech Nyquist, so the old host-side unwrap of the
`enc` angle ALIASED — worse in reverse (a touch faster), printing fake slip 3-21 = "over-commutation"
that never existed. The RP2040 now samples the AS5600 at ~1.25 kHz and reports a de-aliased signed
MECHANICAL velocity over the `encv` line; `_Ticker` prefers it and only falls back to host-side unwrap
of the slow `enc` line when the firmware/sim lacks `encv`. With encv, BOTH directions read slip ~1.0 —
reverse locks exactly like forward.

Robustness the bench taught us (all in `_Ticker`, so every caller gets it):
  * device-side de-aliased velocity (encv) when available, else guarded modulo unwrap + VEL_LP_ALPHA
    low-pass from posctl (no aliased sign flips) — the host-unwrap path still aliases at high speed,
  * _pace every tick so the <500 ms spin deadman is never starved; esc.thrust() re-sent each tick,
  * telemetry frame validity keyed on |eRPM| (NOT volts — this firmware reports volts==0 ALWAYS;
    the garbage first-few-frames-after-arm read rpm~0 and are rejected by the eRPM floor),
  * median (not mean) over the hold window.
"""
from __future__ import annotations

import statistics
from collections import deque
from dataclasses import dataclass, field

from .constants import POLE_PAIRS  # noqa: F401 (re-exported: xover_debug imports it from here)
from .constants import TELE_MIN_MECH_RPM
from .control import DT, VelReader, _pace
from .drive import Aborted

# POLE_PAIRS is imported from constants (single source of truth) and re-exported here for the
# callers that still do `from pico_esc.crossover import POLE_PAIRS` (xover_debug). Reminder: the
# `tele` line is ALREADY mechanical RPM, so slip does NOT divide by POLE_PAIRS — it is only for the
# eRPM<->mech display of the electrical Cross_Up/Cross_Dn threshold bytes.
TELE_MIN_ERPM = TELE_MIN_MECH_RPM  # a valid (spinning, 6-step) telemetry frame; garbage early-after-arm
                                   # frames read rpm~0. NOTE: this firmware's telemetry reports
                                   # volts==0 ALWAYS, so frame validity is keyed on eRPM, NOT volts.
                                   # Value/meaning live in constants.TELE_MIN_MECH_RPM (honest name:
                                   # `tele` is ALREADY mechanical); kept here as TELE_MIN_ERPM for the
                                   # callers that import it from crossover (no behavior change).
ENC_MIN_RPM = 20.0                 # |mech RPM| below this is "not clearly turning" (slip divide guard)
REV_FLOOR_RPM = 150.0              # opposite-direction |mech RPM| that counts as a REAL reversal (a
                                   # true reversal runs to ~1000s; end-of-ramp settling noise << this)
REV_MIN_CMD = 150                  # only check for reversal while still meaningfully commanded (not
                                   # the final coast to 0, where near-zero encoder noise is normal)
REV_TICKS = 20                     # sustained opposite-direction ticks = a REAL runaway
ENC_ALIAS_RPM = 1350.0             # 50 Hz HOST sampling aliases the enc angle above ~this (Nyquist).
                                   # The device `encv` path de-aliases (samples at ~1.25 kHz), so this
                                   # only bounds the legacy host-unwrap fallback.
OVER_SPEED_FLOOR = 13000.0         # mech-RPM floor for the over-speed guard: above this 930 KV motor's
                                   # physical max (~KV*V ~= 11000 mech), so it clears real 6-step entry
                                   # speed and trips only on garbage. TRUE mech (no /pole-pairs).


# Encoder velocity reading (device de-aliased encv, else host unwrap) is centralized in control.py:
# VelReader + unwrap_rpm, shared by _Ticker here and velocity.measure_steady_speed.


@dataclass
class CrossoverResult:
    """Outcome of one measurement (top-of-ramp hold or low-speed descend+hold)."""
    enc_rpm: float = 0.0            # median mech RPM over the steady window (signed)
    tele_erpm: float = 0.0         # median telemetry MECH RPM over the steady window (signed; the
                                   # firmware already divides eRPM by pole pairs — name kept for
                                   # back-compat with callers, but the value is mechanical)
    slip: float | None = None      # |tele_mech| / |enc_mech|, or None if not measurable (~1.0 = lock)
    reversed: bool = False         # rotor turned opposite to the command during the down/coast ramp
    top_cmd: int = 0               # signed command the measurement was taken at (hold command)
    ceiling_hit: bool = False      # measured-speed ceiling stopped the up-ramp early (top-of-ramp mode)
    peak_temp: float | None = None  # peak ESC temp seen on valid (spinning) frames
    n_enc: int = 0                 # steady-window encoder samples used for the median
    n_tele: int = 0                # steady-window valid telemetry samples used for the median
    aborted: str | None = None     # reason the run could not produce a clean measurement

    @property
    def slip_err(self) -> float:
        """|slip - 1|, or +inf if slip could not be measured (treated as worst)."""
        return abs(self.slip - 1.0) if self.slip is not None else float("inf")


@dataclass
class _Windows:
    enc: list = field(default_factory=list)
    tele: list = field(default_factory=list)


class _Ticker:
    """Shared per-tick mechanics for the crossover ramp loops (single source of truth).

    Each tick: read the encoder + guarded-unwrap velocity, re-send esc.thrust() (deadman
    keep-alive), poll telemetry every tele_period (validity keyed on |eRPM|>floor since this
    firmware reports volts==0 always), fold peak-temp + over-temp abort, enforce over-speed /
    stall safety, fire on_sample, and _pace. It holds all per-tick state; the CALLER drives the
    phase sequence and collects the measurement windows from `.vel` / `.tele_erpm`.

    The 50 Hz encoder ALIASES above ~ENC_ALIAS_RPM, so with use_tele_speed=True the over-speed /
    stall guards use the telemetry-derived mech speed while 6-step is live (reliable at any speed)
    and fall back to the encoder otherwise. The top-of-ramp mode leaves it off (its forward
    operating point is encoder-reliable)."""

    def __init__(self, esc, clock, *, sign, over_speed_rpm, max_temp, tele_period,
                 stall_guard, on_sample, result, use_tele_speed=False):
        self.esc, self.clock, self.sign = esc, clock, sign
        self.over_speed_rpm = over_speed_rpm
        self.max_temp, self.tele_period = max_temp, tele_period
        self.stall_guard, self.on_sample = stall_guard, on_sample
        self.res, self.use_tele_speed = result, use_tele_speed
        self._vr = VelReader()         # shared de-aliased mech-RPM reader (encv, fallback host unwrap)
        self.tele_erpm = 0.0           # last valid telemetry MECH RPM (signed; firmware pre-divides)
        self.tele_live = False         # last poll was a live 6-step frame (|eRPM|>floor)
        self.tele_fresh = False        # a telemetry poll happened THIS tick
        now = clock.now()
        self.last_t = now
        self.t0 = now
        self.next_tele = now
        self.stall_ticks = 0

    @property
    def vel(self):
        """Signed mechanical RPM from the shared VelReader (device encv, else host unwrap)."""
        return self._vr.vel

    @property
    def mech_speed(self):
        """Best available |mech RPM|: telemetry (reliable at any speed) while 6-step is live and
        use_tele_speed is on, else the (de-aliased device / possibly-aliasing host) encoder.

        NOTE: `tele_erpm` is ALREADY mechanical RPM — the RP2040 firmware divides the DShot eRPM by
        the pole pairs (ESC_MOTOR_POLES/2, esc_session.h) before sending it. So it is NOT divided
        again here (doing so was a latent 7x under-count)."""
        if self.use_tele_speed and self.tele_live:
            return abs(self.tele_erpm)
        return abs(self.vel)

    def tick(self, cmd, phase):
        tick_start = self.clock.now()
        now = self.clock.now()
        dt = now - self.last_t
        self.last_t = now
        # Signed MECHANICAL RPM via the shared reader: device de-aliased `encv` when available (does
        # NOT alias at the ~6000-9000 mech the rotor really reaches in 6-step), else the host unwrap
        # fallback. At true BEMF lock this equals tele_erpm (also mechanical) so slip == 1.0 in BOTH
        # directions — the old "reverse over-commutates 3-21x" was pure Nyquist aliasing.
        self._vr.read(self.esc, dt)
        self.esc.thrust(cmd)                               # keep-alive (feeds the deadman)

        self.tele_fresh = False
        if tick_start >= self.next_tele:
            self.next_tele = tick_start + self.tele_period
            tel = self.esc.telemetry()
            self.tele_fresh = True
            # Validity by |eRPM| (this firmware reports volts==0 always): a live 6-step frame reads
            # |eRPM|>floor; stale-sine / garbage-startup frames read ~0. Temp trusted only when live.
            if tel is not None and tel.rpm is not None and abs(float(tel.rpm)) > TELE_MIN_ERPM:
                self.tele_erpm = float(tel.rpm)
                self.tele_live = True
                if tel.temp is not None:
                    self.res.peak_temp = (tel.temp if self.res.peak_temp is None
                                          else max(self.res.peak_temp, tel.temp))
                    if self.max_temp and tel.temp >= self.max_temp:
                        raise Aborted(f"over-temp {tel.temp}C")
            else:
                self.tele_live = False

        spd = self.mech_speed
        if spd > self.over_speed_rpm:                      # hard safety net
            raise Aborted(f"over-speed {spd:.0f}RPM > limit {self.over_speed_rpm:.0f}")
        # Stall/desync = powered but not turning. Live telemetry is positive evidence the motor IS
        # commutating (spinning), so never call it a stall while 6-step telemetry is live.
        spinning_by_tele = self.use_tele_speed and self.tele_live
        if self.stall_guard and abs(cmd) >= 200 and spd < 15.0 and not spinning_by_tele:
            self.stall_ticks += 1
            if self.stall_ticks >= 30:                     # ~0.6 s powered but not turning
                raise Aborted(f"stall/desync: cmd={cmd} but ~0 RPM for {self.stall_ticks} ticks")
        else:
            self.stall_ticks = 0

        if self.on_sample is not None:
            self.on_sample(tick_start - self.t0, cmd, self.vel, self.tele_erpm, phase, self.res.peak_temp)
        _pace(self.clock, tick_start)
        return self.vel


def _reversal_watch(tk, sign, cmd, rev_ticks, res):
    """Flag a REAL reversal: rotor spinning strongly opposite the command for many ticks."""
    if abs(cmd) > REV_MIN_CMD and sign * tk.vel < -REV_FLOOR_RPM:
        rev_ticks += 1
        if rev_ticks >= REV_TICKS:
            res.reversed = True
        return rev_ticks
    return 0


def _finalize(res, steady, tail=None):
    """Reduce the steady window (falling back to the up-ramp tail) to robust medians + slip."""
    tail = tail or _Windows()
    enc_win = list(steady.enc) or list(tail.enc)
    tele_win = list(steady.tele) or list(tail.tele)
    res.n_enc = len(enc_win)
    res.n_tele = len(tele_win)
    if enc_win:
        res.enc_rpm = statistics.median(enc_win)
    if tele_win:
        res.tele_erpm = statistics.median(tele_win)
    if enc_win and tele_win and abs(res.enc_rpm) > ENC_MIN_RPM:
        # Both are mechanical RPM (tele already /pole-pairs on-device; encoder is a 2-pole shaft
        # magnet). slip = tele_mech / enc_mech ~= 1.0 at true BEMF lock. (The historical /POLE_PAIRS
        # here was a double-division that made a real lock read as ~0.143.)
        res.slip = abs(res.tele_erpm) / abs(res.enc_rpm)


def _coast_to_zero(tk, from_cmd, *, sign=1, res=None, secs=2.0):
    """Ramp the command from from_cmd down to 0 (safe stop), optionally watching for reversal.
    Never raises out (a late over-temp during the coast is moot — we are already stopping)."""
    n = max(1, round(secs / DT))
    start = abs(int(from_cmd))
    s = 1 if from_cmd >= 0 else -1
    rev_ticks = 0
    for i in range(n):
        cmd = s * round(start * (n - i - 1) / n)
        try:
            tk.tick(cmd, "down")
        except Aborted:
            break
        if res is not None:
            rev_ticks = _reversal_watch(tk, sign, cmd, rev_ticks, res)


def measure_crossover_lock(esc, clock, *, target_cmd, sign=1,
                           ramp_secs=8.0, hold_secs=1.5, down_secs=8.0,
                           rpm_ceiling=450.0, max_temp=60.0, tele_period=0.2,
                           on_sample=None, stall_guard=True):
    """Top-of-ramp mode: ramp |cmd| 0 -> target_cmd through the crossover, hold at the top, measure
    steady slip, then ramp down and detect reversal. Returns a CrossoverResult.

    The ESC must already be armed (bidir). Drives esc.thrust() every tick as the keep-alive. Safety
    trips (over-speed past 1.5x rpm_ceiling, stall/desync, over-temp) raise drive.Aborted — the
    caller disarms in a finally. Merely reaching rpm_ceiling is NOT an abort: the up-ramp stops there
    (no dwell at the ceiling) and the down-ramp begins. on_sample(t, cmd, enc_rpm, tele_erpm, phase,
    peak_temp) is called once per tick (phase "up"|"hold"|"down") for CSV / handoff reporting."""
    res = CrossoverResult()
    tk = _Ticker(esc, clock, sign=sign, over_speed_rpm=rpm_ceiling * 1.5, max_temp=max_temp,
                 tele_period=tele_period, stall_guard=stall_guard, on_sample=on_sample, result=res)
    steady = _Windows()
    hold_ticks = max(1, round(hold_secs / DT))
    tail = _Windows(enc=deque(maxlen=hold_ticks),
                    tele=deque(maxlen=max(1, round(hold_secs / tele_period) + 1)))

    def collect(phase):
        win = steady if phase == "hold" else tail
        win.enc.append(tk.vel)
        if tk.tele_fresh and tk.tele_live:
            win.tele.append(tk.tele_erpm)

    # ---- ramp up (stop early if the measured-speed ceiling is hit) ----
    n = max(1, round(ramp_secs / DT))
    top_cmd = sign * int(target_cmd)
    for i in range(n):
        cmd = sign * round(target_cmd * (i + 1) / n)
        tk.tick(cmd, "up")
        collect("up")
        if abs(tk.vel) >= rpm_ceiling:
            res.ceiling_hit = True
            top_cmd = cmd
            break
    res.top_cmd = top_cmd

    # ---- hold at the top and measure (skip the dwell if the ceiling cut us off: the up-ramp
    #      tail is scored instead) ----
    if not res.ceiling_hit:
        for _ in range(hold_ticks):
            tk.tick(top_cmd, "hold")
            collect("hold")

    # ---- ramp down through the reverse handoff, watching for reversal ----
    n = max(1, round(down_secs / DT))
    rev_ticks = 0
    for i in range(n):
        cmd = sign * round(abs(top_cmd) * (n - i - 1) / n)
        tk.tick(cmd, "down")
        rev_ticks = _reversal_watch(tk, sign, cmd, rev_ticks, res)

    _finalize(res, steady, tail)
    return res


def measure_crossover_lock_lowspeed(esc, clock, *, target_cmd, sign=1,
                                    ramp_secs=8.0, descend_secs=8.0, hold_secs=1.5,
                                    measure_rpm=700.0, rpm_ceiling=None, max_temp=60.0,
                                    tele_period=0.2, hold_bump=30, hold_retries=3,
                                    on_sample=None, stall_guard=True):
    """Low-speed mode — measure the 6-step lock at a LOW, encoder-reliable speed so BOTH directions
    work. Returns a CrossoverResult.

    Why: the encoder samples once per 50 Hz control tick, so it aliases above ~1350 mech RPM
    (Nyquist). Forward at the crossover holds ~1055 mech (measurable); reverse runs faster
    (~1600-2000 mech) and aliases into garbage / sign flips. The firmware also refuses Cross_Up
    below the ~1333 eRPM BEMF floor, so the crossover itself can't be lowered.

    Approach: ramp up past Cross_Up into 6-step (detected by telemetry going LIVE), then DESCEND the
    command — navigating by TELEMETRY eRPM, which the ESC measures reliably at any speed — until the
    mech speed reaches `measure_rpm`; HOLD there and measure the encoder-based slip (the encoder is
    trustworthy at ~700 mech = ~0.23 rev/sample). Direction-adaptive: the hold command is FOUND by
    descending, not hardcoded, so reverse (faster) settles at a lower command than forward.

    Robustness: if the hold command is a touch too low and the ESC hands back to sine (telemetry
    goes stale mid-hold), the hold retries at a slightly higher command (+hold_bump), bounded by
    hold_retries. If it still can't hold 6-step, res.aborted explains why."""
    res = CrossoverResult()
    # Over-speed guard (TRUE mech RPM now that the double-division is gone): the rotor genuinely
    # spins fast in 6-step (~6000-7000 mech at the entry for this 930 KV motor) before the descend
    # drops it, so the limit must clear the motor's real top speed (KV*V ~= 11000 mech max). The
    # OVER_SPEED_FLOOR keeps it above legitimate operation while still catching a true garbage/
    # runaway reading; an explicit rpm_ceiling or a high measure_rpm can raise it further.
    over = max((rpm_ceiling or 0.0) * 1.5, measure_rpm * 5.0, OVER_SPEED_FLOOR)
    # Low-speed mode NAVIGATES by telemetry (ramp until it goes live == 6-step; descend until the
    # tele/enc speed reaches measure_rpm), so it must sample telemetry every control tick: the real
    # sine->6-step handoff and the measure_rpm crossing are narrow events that a coarse tele_period
    # would step over between polls (with honest stale-sine telemetry the up-handoff window at the top
    # of the ramp is only a couple of ticks wide). Poll at DT (temp/over-speed ride along on the same
    # frames). The top-of-ramp mode keeps the coarser caller-supplied tele_period.
    tk = _Ticker(esc, clock, sign=sign, over_speed_rpm=over, max_temp=max_temp,
                 tele_period=min(tele_period, DT), stall_guard=stall_guard, on_sample=on_sample,
                 result=res, use_tele_speed=True)

    # ---- 1) ramp up until telemetry goes LIVE == we crossed into 6-step (don't dwell up high) ----
    n = max(1, round(ramp_secs / DT))
    top = sign * int(target_cmd)
    for i in range(n):
        cmd = sign * round(target_cmd * (i + 1) / n)
        tk.tick(cmd, "up")
        top = cmd
        if tk.tele_live:                                   # handoff fired -> in 6-step
            break
    res.top_cmd = top
    if not tk.tele_live:
        res.aborted = "never reached 6-step (telemetry stale through the up-ramp)"
        _coast_to_zero(tk, top, sign=sign)
        return res

    # ---- 2) STEPPED descend: hold each command briefly and navigate by the SETTLED 6-step speed,
    #         not the LAGGING instantaneous speed. A smooth fast descend overshoots the command below
    #         the down-handoff (the rotor lags), dropping out of 6-step — and once in forced sine the
    #         ESC won't re-hand-off at a low command, so the measurement is lost (or, if we chased it
    #         back up, degenerates to ~top-of-ramp). Stepping down and letting each command settle
    #         keeps us in the live regime and stops at a genuine ~measure_rpm 6-step point. ----
    n = max(1, round(descend_secs / DT))                   # descend tick budget
    settle_ticks = max(2, round(0.06 / DT))                # ~one rotor time-constant to settle
    step = max(hold_bump, round(abs(top) / 30.0))
    hold_cmd = None
    last_hold = top                                        # last command still ABOVE measure_rpm (6-step)
    cmd = top
    budget = n
    while budget > 0 and abs(cmd) >= 1:
        live = False
        spd = None
        for _ in range(settle_ticks):
            tk.tick(cmd, "descend")
            budget -= 1
            if tk.tele_fresh:
                live = tk.tele_live
                if tk.tele_live:
                    spd = tk.mech_speed
        if not live:                                       # overshot into sine -> fall back to last_hold
            break
        if spd is not None and spd <= measure_rpm:         # reached the target while still 6-step
            hold_cmd = last_hold                            # the last command safely ABOVE measure_rpm
            break
        last_hold = cmd
        cmd = sign * max(1, abs(cmd) - step)
    if hold_cmd is None:
        hold_cmd = last_hold                               # settled at the lowest holdable 6-step command

    # ---- 3) hold at hold_cmd and measure; bump up one descend-step if a longer hold sags out of
    #         6-step (marginal point just above the down-handoff). ----
    steady = _Windows()
    held = False
    hold_ticks = max(1, round(hold_secs / DT))
    for _ in range(hold_retries + 1):
        steady = _Windows()
        stale = False
        for _ in range(hold_ticks):
            tk.tick(hold_cmd, "hold")
            steady.enc.append(tk.vel)
            if tk.tele_fresh:
                if tk.tele_live:
                    steady.tele.append(tk.tele_erpm)
                else:
                    stale = True                           # handed back to sine at this command
                    break
        if not stale and steady.tele:
            held = True
            break
        hold_cmd = sign * (abs(hold_cmd) + step)           # sagged -> step up one descend-step and retry
    res.top_cmd = hold_cmd
    if not held:
        res.aborted = (res.aborted or
                       f"could not hold 6-step near {measure_rpm:.0f} mech RPM — no stable point above "
                       f"the down-handoff for this crossover config; raise --measure-speed")

    _finalize(res, steady)                                 # medians from the hold window only

    # ---- 4) safe coast to zero, watching for reversal (encoder reliable at this low speed) ----
    _coast_to_zero(tk, hold_cmd, sign=sign, res=res)
    return res
