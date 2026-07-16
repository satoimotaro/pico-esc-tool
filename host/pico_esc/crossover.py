# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""pico_esc.crossover — one proven measurement loop for the S3 sine<->6-step crossover.

Both xover_debug (the hardware connection/handoff checker) and autocal's tune-crossover-lock
phase need to do the SAME thing: ramp the ESC command up through the crossover into 6-step,
hold, measure steady-state lock quality, then ramp back down and see whether the rotor
reversed. This module is the single source of truth for that loop so the two callers cannot
drift apart.

Lock quality is the SLIP ratio:

    slip = (telemetry_eRPM / POLE_PAIRS) / |encoder_mech_RPM|

which is ~1.0 at true BEMF lock. In forced sine the telemetry eRPM is stale, and a mis-tuned
6-step lock commutates near the seed rate rather than real BEMF (slip well above 1). Auto-cal
minimises |slip - 1| across a (comm_timing, demag_compensation) grid.

Robustness the bench taught us (all handled here so every caller gets it):
  * the guarded modulo unwrap + VEL_LP_ALPHA low-pass from posctl (no aliased sign flips),
  * _pace every tick so the <500 ms spin deadman is never starved, and esc.thrust() re-sent
    every tick as the keep-alive,
  * SKIP invalid telemetry frames (volts == 0): the first ~3 frames after arm return garbage
    temp/volts=0 and would otherwise spike peak-temp and poison the slip median,
  * median (not mean) over the hold window.
"""
from __future__ import annotations

import statistics
from collections import deque
from dataclasses import dataclass, field

from .constants import COUNTS_PER_REV
from .control import DT, DELTA_FAULT_FRAC, VEL_LP_ALPHA, _pace
from .drive import Aborted

POLE_PAIRS = 7                     # 12N14P: telemetry eRPM = mech RPM * POLE_PAIRS
TELE_MIN_ERPM = 50.0               # a valid (spinning) telemetry frame; garbage early-after-arm
                                   # frames read rpm~0. NOTE: this firmware's telemetry reports
                                   # volts==0 ALWAYS, so frame validity is keyed on eRPM, NOT volts.
ENC_MIN_RPM = 20.0                 # |mech RPM| below this is "not clearly turning" (slip divide guard)
REV_FLOOR_RPM = 150.0              # opposite-direction |mech RPM| that counts as a REAL reversal (a
                                   # true reversal runs to ~1000s; end-of-ramp settling noise << this)
REV_MIN_CMD = 150                  # only check for reversal while still meaningfully commanded (not
                                   # the final coast to 0, where near-zero encoder noise is normal)
REV_TICKS = 20                     # sustained opposite-direction ticks = a REAL runaway (a true
                                   # reversal reverses for dozens of ticks; a brief high-speed
                                   # encoder-aliasing blip on the down-ramp is only a few ticks)


def unwrap_rpm(prev_raw, raw, dt, vel):
    """Return (new_vel_rpm, new_prev_raw). Guarded modulo unwrap + low-pass, like posctl.

    The modulo unwrap is only valid when true travel is < half a rev per tick; an implausible
    delta (aliased wrong-direction jump) or a non-positive dt holds the last velocity instead of
    flipping the sign."""
    if prev_raw is None:
        return vel, raw
    d = ((raw - prev_raw + COUNTS_PER_REV // 2) % COUNTS_PER_REV) - COUNTS_PER_REV // 2
    if abs(d) > DELTA_FAULT_FRAC * (COUNTS_PER_REV // 2) or dt <= 0:
        return vel, raw
    inst = (d / COUNTS_PER_REV) / dt * 60.0                 # mech RPM (signed)
    return vel + VEL_LP_ALPHA * (inst - vel), raw


@dataclass
class CrossoverResult:
    """Outcome of one ramp-up / hold / ramp-down measurement."""
    enc_rpm: float = 0.0            # median mech RPM over the steady window (signed)
    tele_erpm: float = 0.0         # median telemetry eRPM over the steady window (signed)
    slip: float | None = None      # (|tele| / POLE_PAIRS) / |enc|, or None if not measurable
    reversed: bool = False         # rotor turned opposite to the command during the down-ramp
    top_cmd: int = 0               # signed command actually reached at the top of the ramp
    ceiling_hit: bool = False      # measured-speed ceiling stopped the up-ramp early
    peak_temp: float | None = None  # peak ESC temp seen on VALID (volts>0) frames
    n_enc: int = 0                 # steady-window encoder samples used for the median
    n_tele: int = 0                # steady-window valid telemetry samples used for the median
    aborted: str | None = None     # abort reason if a safety trip cut the ramp short

    @property
    def slip_err(self) -> float:
        """|slip - 1|, or +inf if slip could not be measured (treated as worst)."""
        return abs(self.slip - 1.0) if self.slip is not None else float("inf")


@dataclass
class _Windows:
    enc: list = field(default_factory=list)
    tele: list = field(default_factory=list)


def measure_crossover_lock(esc, clock, *, target_cmd, sign=1,
                           ramp_secs=8.0, hold_secs=1.5, down_secs=8.0,
                           rpm_ceiling=450.0, max_temp=60.0, tele_period=0.2,
                           on_sample=None, stall_guard=True):
    """Ramp |cmd| 0 -> target_cmd (in direction `sign`) through the crossover, hold, measure
    steady slip, then ramp back down and detect reversal. Returns a CrossoverResult.

    The ESC must already be armed (bidir). This drives esc.thrust() every tick as the keep-alive
    and never sends a throttle. Safety trips (over-speed past 1.5x rpm_ceiling, stall/desync,
    over-temp) raise drive.Aborted — the caller is expected to disarm in a finally. A measured
    speed that merely reaches rpm_ceiling is NOT an abort: the up-ramp stops there and the
    down-ramp begins (bounds top speed, as on the first hardware run).

    on_sample(t, cmd, enc_rpm, tele_erpm, phase, peak_temp) is called once per tick (phase is
    "up" | "hold" | "down") so a caller can log CSV / report handoff regimes without duplicating
    the loop. tele_erpm is the last VALID (volts>0) telemetry eRPM, or 0.0 before the first one.
    """
    res = CrossoverResult()
    steady = _Windows()
    # rolling tail of the up-ramp so a ceiling-limited run (no dwell at the ceiling) is still
    # scored, from the samples just before the ramp was cut.
    hold_ticks = max(1, round(hold_secs / DT))
    tail = _Windows(enc=deque(maxlen=hold_ticks),
                    tele=deque(maxlen=max(1, round(hold_secs / tele_period) + 1)))

    prev_raw = None
    vel = 0.0
    last_erpm = 0.0
    last_t = clock.now()
    t0 = last_t
    next_tele = t0
    stall_ticks = 0
    rev_ticks = 0

    def tick(cmd, phase):
        nonlocal prev_raw, vel, last_erpm, last_t, next_tele, stall_ticks, rev_ticks
        tick_start = clock.now()
        enc = esc.encoder()
        now = clock.now()
        dt = now - last_t
        last_t = now
        if enc is not None and enc.healthy:
            vel, prev_raw = unwrap_rpm(prev_raw, enc.raw, dt, vel)
        esc.thrust(cmd)                                     # keep-alive (feeds the deadman)

        if tick_start >= next_tele:
            next_tele = tick_start + tele_period
            tel = esc.telemetry()
            # SKIP invalid frames by eRPM (not volts — this firmware always reports volts==0):
            # the first ~3 frames after arm return rpm~0 with garbage temp, and near-zero/stale
            # eRPM must not poison the slip median. Temp is trusted only on valid spinning frames.
            if tel is not None and tel.rpm is not None and abs(float(tel.rpm)) > TELE_MIN_ERPM:
                last_erpm = float(tel.rpm)
                if tel.temp is not None:
                    res.peak_temp = tel.temp if res.peak_temp is None else max(res.peak_temp, tel.temp)
                    if max_temp and tel.temp >= max_temp:
                        raise Aborted(f"over-temp {tel.temp}C")
                if phase == "hold":
                    steady.tele.append(last_erpm)
                elif phase == "up":
                    tail.tele.append(last_erpm)

        if abs(vel) > rpm_ceiling * 1.5:                   # hard safety net
            raise Aborted(f"over-speed {vel:.0f}RPM > 1.5x ceiling")
        if stall_guard and abs(cmd) >= 200 and abs(vel) < 15.0:
            stall_ticks += 1
            if stall_ticks >= 30:                          # ~0.6 s powered but not turning
                raise Aborted(f"stall/desync: cmd={cmd} but ~0 RPM for {stall_ticks} ticks")
        else:
            stall_ticks = 0

        if phase == "hold":
            steady.enc.append(vel)
        elif phase == "up":
            tail.enc.append(vel)
        if phase == "down":                                # reversal: sustained large opposite spin
            if abs(cmd) > REV_MIN_CMD and sign * vel < -REV_FLOOR_RPM:
                rev_ticks += 1
                if rev_ticks >= REV_TICKS:
                    res.reversed = True
            else:
                rev_ticks = 0

        if on_sample is not None:
            on_sample(tick_start - t0, cmd, vel, last_erpm, phase, res.peak_temp)
        _pace(clock, tick_start)
        return vel

    # ---- ramp up (stop early if the measured-speed ceiling is hit) ----
    n = max(1, round(ramp_secs / DT))
    top_cmd = sign * int(target_cmd)
    for i in range(n):
        cmd = sign * round(target_cmd * (i + 1) / n)
        v = tick(cmd, "up")
        if abs(v) >= rpm_ceiling:
            res.ceiling_hit = True
            top_cmd = cmd
            break
    res.top_cmd = top_cmd

    # ---- hold at the top and measure steady slip. If the measured-speed ceiling cut the ramp
    #      short we do NOT dwell at that speed (safety); the up-ramp tail below is scored instead. ----
    if not res.ceiling_hit:
        for _ in range(hold_ticks):
            tick(top_cmd, "hold")

    # ---- ramp down through the reverse handoff, watching for reversal ----
    n = max(1, round(down_secs / DT))
    for i in range(n):
        tick(sign * round(abs(top_cmd) * (n - i - 1) / n), "down")

    # ---- reduce the steady window to robust medians (fall back to the up-ramp tail if we
    #      never held, e.g. a ceiling-limited run) ----
    enc_win = list(steady.enc) or list(tail.enc)
    tele_win = list(steady.tele) or list(tail.tele)
    res.n_enc = len(enc_win)
    res.n_tele = len(tele_win)
    if enc_win:
        res.enc_rpm = statistics.median(enc_win)
    if tele_win:
        res.tele_erpm = statistics.median(tele_win)
    if enc_win and tele_win and abs(res.enc_rpm) > ENC_MIN_RPM:
        res.slip = (abs(res.tele_erpm) / POLE_PAIRS) / abs(res.enc_rpm)
    return res
