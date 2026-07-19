# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""pico_esc.sim — seeded, hardware-free ESC host models for --dry-run.

SimEscHost is a toy brushless motor answering the throttle/tele/arm/editpage protocol
(autocal). SimEncEscHost extends it with a signed-thrust S1 forced-commutation stepper model
and the AS5600 `enc|…` line (posctl). Both are DETERMINISTIC: the seeded RNG is drawn in a
fixed order per cmd(), so the dry-run output is the regression oracle for this refactor — DO
NOT add/remove cmd() calls or reorder RNG draws. Moved verbatim from autocal.py / posctl.py.
"""
from __future__ import annotations

import math

from .config import (FIELD_OFF, SINE_CROSS_DN_ERPM_NUM,
                     SINE_CROSS_UP_ERPM_PER_UNIT)
from .constants import COUNTS_PER_REV, FULLSCALE_RPM


# ---------------------------------------------------------------------------
# Simulated ESC host (for --dry-run): a toy motor model, no hardware/serial.
# ---------------------------------------------------------------------------
class SimEscHost:
    """Mimics EscHost.cmd() with a toy brushless motor model.

    Cold start needs throttle >= a start threshold (eased by startup_power_max);
    once spinning it stalls below a sustain threshold; RPM is ~linear in throttle
    with additive noise whose amplitude grows as comm_timing / demag move away
    from their sweet spot (so tune-smooth has a real minimum to find).
    """

    def __init__(self, seed=1234):
        import random
        self.armed = False
        self.spinning = False
        self.rpm = 0.0
        self._rng = random.Random(seed)    # seeded -> reproducible --dry-run
        self.cfg = {                       # decoded config the model reacts to
            "startup_power_min": 40, "startup_power_max": 60,
            "comm_timing": 3, "demag_compensation": 1,
        }
        self._off = {v: k for k, v in FIELD_OFF.items()}

    # --- model tuning constants ---
    START_BASE = 95.0          # cold-start throttle at nominal startup_power_max
    STALL_THR = 42.0           # below this while spinning -> stall
    RPM_GAIN = 12.0            # mechanical rpm per throttle unit above offset
    RPM_OFFSET = 30.0
    BASE_NOISE = 6.0

    def _start_threshold(self):
        # More startup power -> easier (lower) cold-start throttle.
        return self.START_BASE - (self.cfg["startup_power_max"] - 60) * 0.4 \
                                - (self.cfg["startup_power_min"] - 40) * 0.2

    def _noise_amp(self):
        dt = abs(self.cfg["comm_timing"] - 3) + abs(self.cfg["demag_compensation"] - 1)
        return self.BASE_NOISE * (1.0 + 1.6 * dt)

    def _target_rpm(self, thr):
        return max(0.0, (thr - self.RPM_OFFSET) * self.RPM_GAIN)

    def _tick(self, thr):
        if not self.armed or thr <= 0:
            self.spinning, self.rpm = False, 0.0
            return
        if not self.spinning:
            if thr >= self._start_threshold():
                self.spinning = True
            else:
                self.rpm = 0.0
                return
        if thr < self.STALL_THR:                # lost sync at too-low throttle
            self.spinning, self.rpm = False, 0.0
            return
        target = self._target_rpm(thr)
        self.rpm += (target - self.rpm) * 0.5   # first-order ramp
        self.rpm = max(0.0, self.rpm + self._rng.gauss(0.0, self._noise_amp()))

    def _apply_editpage(self, arg):
        # arg = "IDX off:byte,off:byte,..." -> update decoded cfg
        parts = arg.split()
        pairs = parts[1] if len(parts) > 1 else ""
        for tok in pairs.split(","):
            if ":" not in tok:
                continue
            off, byte = (int(x, 16) for x in tok.split(":"))
            name = self._off.get(off)
            if name in self.cfg:
                self.cfg[name] = byte

    def cmd(self, line, timeout=30.0):
        line = line.strip()
        head, _, arg = line.partition(" ")
        if head == "throttle":
            thr = int(arg.split()[-1])
            self._tick(thr)
            return []
        if head == "tele":
            rpm = int(self.rpm)
            return [f"tele|{rpm}|12.30|0|25|0"]     # rpm|volts|amps|temp|stress
        if head == "arm":
            self.armed, self.spinning, self.rpm = True, False, 0.0
            return ["armed|bidir"]
        if head in ("disarm", "run", "disconnect"):
            self.armed = self.spinning = False
            self.rpm = 0.0
            return []
        if head == "editpage":
            self._apply_editpage(arg)
            return ["applied"]
        if head == "scan":
            return ["esc|1|1|1|E8B2|8|#A_H_30#|Sim|0.21"]
        if head in ("enter", "read"):
            return []
        return []

    def close(self):
        pass


# ---------------------------------------------------------------------------
# Simulated ESC host for --dry-run: extends SimEscHost to also answer
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

    # --- S3 sine<->BEMF crossover model (only active when sine_cross_up/dn are configured) ---
    POLE_PAIRS = 7                # 12N14P: eRPM = mech RPM * POLE_PAIRS
    # 6-step BEMF load-line: |thrust| -> eRPM, two bench anchors from working_930kv_low
    # (thr 55 -> ~190 eRPM breakaway, thr 700 -> ~3800 eRPM). Above Cross_Up the firmware runs
    # this faster, more-efficient regime; the calibration curve subsumes the handoff jump.
    LINE_THR_LO, LINE_ERPM_LO = 55.0, 190.0
    LINE_THR_HI, LINE_ERPM_HI = 700.0, 3800.0

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
        # S3 crossover: track the two threshold bytes (dropped by the base _apply_editpage
        # unless they exist in cfg) so a dry-run editpage can gate the crossover model. Both
        # default OFF (0), so the model is a no-op and _advance stays byte-identical to HEAD.
        self.cfg["sine_cross_up"] = 0
        self.cfg["sine_cross_dn"] = 0
        self._regime = "sine"    # sticky forced-sine <-> 6-step BEMF load-line (hysteresis)

    def _target_rpm(self, thrust):
        # Stepper: linear thrust->RPM, no deadband, no floor.
        return thrust * FULLSCALE_RPM / 1000.0

    def _crossover_on(self):
        """True iff both S3 crossover thresholds are configured (0/0xFF = OFF)."""
        cu = self.cfg.get("sine_cross_up", 0)
        cd = self.cfg.get("sine_cross_dn", 0)
        return cu not in (0, 0xFF) and cd not in (0, 0xFF)

    def _line_target_rpm(self, thrust):
        """6-step BEMF load-line: |thrust| -> eRPM (bench anchors) -> signed mech RPM."""
        slope = ((self.LINE_ERPM_HI - self.LINE_ERPM_LO)
                 / (self.LINE_THR_HI - self.LINE_THR_LO))
        erpm = self.LINE_ERPM_LO + slope * (abs(thrust) - self.LINE_THR_LO)
        mech = max(0.0, erpm) / self.POLE_PAIRS
        return math.copysign(mech, thrust)

    def _regime_target_rpm(self):
        """Target mech RPM for the current thrust, crossover-aware.

        Crossover OFF -> the original forced-sine law, byte-for-byte (no state read, no branch
        change, NO RNG). Configured -> sticky hysteresis: forced-sine below Cross_Up, hand off
        to the 6-step BEMF load-line once commanded eRPM exceeds Cross_Up, drop back to sine
        only after the ACTUAL eRPM falls below Cross_Dn (dn < up = a real hysteresis band).
        """
        if not self._crossover_on():
            return self._target_rpm(self.thrust)
        up_erpm = self.cfg["sine_cross_up"] * SINE_CROSS_UP_ERPM_PER_UNIT
        dn_erpm = SINE_CROSS_DN_ERPM_NUM / self.cfg["sine_cross_dn"]
        cmd_erpm = abs(self.thrust) * FULLSCALE_RPM / 1000.0 * self.POLE_PAIRS
        act_erpm = abs(self.rpm) * self.POLE_PAIRS
        if self._regime == "sine":
            if cmd_erpm > up_erpm:
                self._regime = "line"
        elif act_erpm < dn_erpm:
            self._regime = "sine"
        if self._regime == "line":
            return self._line_target_rpm(self.thrust)
        return self._target_rpm(self.thrust)

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
            tgt = self._regime_target_rpm()
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
        if head == "tele" and self._crossover_on():
            # Live eRPM above the crossover: _advance FIRST so `tele` reports the fresh rotor
            # speed (int mech RPM, signed). GATED on a configured crossover — with the
            # crossover OFF this branch is skipped and `tele` falls through to SimEscHost.cmd
            # exactly as before (no _advance, draws 0 RNG), so the dry-run oracle is untouched.
            if self.io_time:
                self.clock.sleep(self.io_time)
            self._advance(self.clock.now())
            # Regime-aware freshness, matching HARDWARE: bidir-DShot telemetry is derived from BEMF
            # zero-crossings, so it is LIVE only in 6-step ("line") and STALE (reports ~0) in forced
            # sine — the OPPOSITE of the pre-A1 sim, which reported a live rpm in BOTH regimes. The
            # _advance above (and its RNG draws) are UNCHANGED, so the crossover-ON dry-run stays
            # deterministic; only the reported number is gated. (Crossover-OFF is untouched above.)
            rpm = int(self.rpm) if self._regime == "line" else 0
            return [f"tele|{rpm}|12.30|0|25|0"]
        if head == "arm":
            self.thrust = 0
            self.rpm = 0.0
            self.pos_counts = 0.0
            self.last_t = self.clock.now()
            self._regime = "sine"
        elif head in ("disarm", "run", "disconnect"):
            self.thrust = 0
            self.rpm = 0.0
        return super().cmd(line, timeout)
