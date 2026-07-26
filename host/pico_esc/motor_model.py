# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""pico_esc.motor_model — a PARAMETRIC no-load motor model: Motor(KV, pole_pairs, V).

Instead of a fully bench-measured per-motor profile, predict the thrust->rpm curve, the regime
boundaries, and the (near-universal) ESC crossover config from first principles + a UNIVERSAL firmware
"duty shape" that is calibrated ONCE (it is a firmware/DShot property, KV-independent). A light eRPM
self-cal (using bidir-DShot `tele`, no encoder) fits the residual scale for a specific motor/supply.

Physics (validated on the F2838 350KV, RMSE 2.8 rpm):
  * no-load rpm  = KV * V * duty(cmd)      -> the 6-step curve SCALES with KV*V; duty(cmd) is universal.
  * sine forced-commutation is PP-governed: mech = (cmd/1000) * SINE_FULLSCALE_ERPM / PP  (KV-independent).
  * regime split is PP-governed: sine below, 6-step above; handoff fires at the firmware floor
    (SINE_CROSS_UP_ERPM_MIN eRPM) => mech handoff = that / PP.

So Motor(KV, PP, V) alone predicts: no-load top, the sine + 6-step FF curves, the regime boundaries,
and the derivable ESC config. What it CANNOT predict is the dynamics (tau ~ inertia/load) and thus the
FB gains — those stay load-dependent (tune under load, or a conservative KV-scaled seed).

The universal-shape constants below are the 350KV bench fit; confirm/refresh them when a second KV
(e.g. 1000KV) is measured (2nd-order IR-drop/tolerance shows up as a per-motor `eff` the self-cal fits).
"""
from __future__ import annotations

from .config import (SINE_CROSS_UP_ERPM_MIN, SINE_CROSS_UP_ERPM_PER_UNIT, sine_crossover_bytes)
from .velocity import SpeedProfile

# --- universal firmware/DShot shapes (calibrated once; KV-independent) ---------------------
# 6-step: duty_eff(cmd) = rpm / (KV*V) = A6*cmd + B6, for cmd in the 6-step region. (350KV fit.)
DUTY6_A = 0.00265
DUTY6_B = -1.233
# sine: mech RPM = (cmd/1000) * SINE_FULLSCALE_MECH, SINE_FULLSCALE_MECH = SINE_FULLSCALE_ERPM / PP.
SINE_FULLSCALE_ERPM = 2499.0      # firmware S1 full-scale eRPM at cmd 1000 (SINE_RCP_SHIFT=3 etc.)
HANDOFF_ERPM = SINE_CROSS_UP_ERPM_MIN     # ~1333 eRPM: the lowest the firmware will hand off (min slip)


class MotorModel:
    """Motor(KV [rpm/V], pole_pairs, v_supply [V]). eff (0..1] absorbs IR-drop / KV tolerance / losses
    and is what the eRPM self-cal fits (default 1.0 = ideal). Optional cmd_floor overrides the predicted
    6-step floor cmd."""

    def __init__(self, kv, pole_pairs, v_supply, eff=1.0, name=""):
        self.kv = float(kv)
        self.pp = int(pole_pairs)
        self.v = float(v_supply)
        self.eff = float(eff)
        self.name = name or f"{int(kv)}kv"

    # ---- scalar predictions ----
    def no_load_top(self) -> float:
        """No-load top mech RPM ~ KV*V*eff."""
        return self.kv * self.v * self.eff

    def sine_fullscale_mech(self) -> float:
        """Forced-sine full-scale mech RPM (cmd 1000) = SINE_FULLSCALE_ERPM / PP. KV-INDEPENDENT."""
        return SINE_FULLSCALE_ERPM / self.pp

    def handoff_mech(self) -> float:
        """Mech RPM at which the sine->6step handoff fires (firmware floor / PP)."""
        return HANDOFF_ERPM / self.pp

    def handoff_cmd(self) -> float:
        """cmd at which forced sine reaches the handoff speed (where the up-crossover fires)."""
        return self.cmd_for_sine(self.handoff_mech())

    def sixstep_floor_rpm(self) -> float:
        """The 6-step 'landing' rpm — the 6-step speed at the handoff thrust. The up-crossover jumps
        from sine (~handoff_mech) straight to here; the band between is the no-steady-state gap. Scales
        with KV (it is a 6-step rpm)."""
        return self.rpm_6step(self.handoff_cmd())

    # ---- FF curves ----
    def rpm_sine(self, cmd: float) -> float:
        """Forced-sine mech RPM at a signed cmd (PP-governed)."""
        return (cmd / 1000.0) * self.sine_fullscale_mech()

    def rpm_6step(self, cmd: float) -> float:
        """6-step mech RPM at cmd (KV*V-scaled universal shape), clamped [0, no-load top]."""
        duty = DUTY6_A * abs(cmd) + DUTY6_B
        if duty <= 0:
            return 0.0
        rpm = self.kv * self.v * self.eff * duty
        return min(rpm, self.no_load_top())

    def cmd_for_6step(self, rpm: float) -> float:
        """Invert the 6-step curve: cmd that produces `rpm` mech (6-step region)."""
        duty = rpm / (self.kv * self.v * self.eff)
        return (duty - DUTY6_B) / DUTY6_A

    def cmd_for_sine(self, rpm: float) -> float:
        return rpm / self.sine_fullscale_mech() * 1000.0

    def thrust_for(self, rpm: float) -> float:
        """Regime-aware inverse: sine below the handoff, 6-step above the 6-step floor. The gap between
        (sine reachable) and (6-step floor) has NO steady state — a target there snaps to the nearest
        reachable point (6-step floor), and the caller should treat it as 'not holdable'."""
        s = 1.0 if rpm >= 0 else -1.0
        a = abs(rpm)
        if a <= self.handoff_mech():
            return s * self.cmd_for_sine(a)
        if a >= self.sixstep_floor_rpm():
            return s * self.cmd_for_6step(a)
        # in the gap: snap up to the 6-step floor (cannot hold between)
        return s * self.cmd_for_6step(self.sixstep_floor_rpm())

    def in_gap(self, rpm: float) -> bool:
        """True if `rpm` falls in the no-steady-state crossover gap (FB should be gain-scheduled off here)."""
        return self.handoff_mech() < abs(rpm) < self.sixstep_floor_rpm()

    def regime(self, rpm: float) -> str:
        a = abs(rpm)
        if a <= self.handoff_mech():
            return "sine"
        if a >= self.sixstep_floor_rpm():
            return "6step"
        return "gap"

    # ---- ESC config derivation (near-universal recipe) ----
    def esc_config(self, seamless_down=False) -> dict:
        """The working full-range ESC settings. cross_up = the firmware handoff floor (min slip);
        cross_dn disabled (255) unless a seamless down-handoff is available (motor-specific)."""
        cross_up = round(HANDOFF_ERPM / SINE_CROSS_UP_ERPM_PER_UNIT)     # =34 (eRPM-based -> KV-independent)
        cross_dn = 255 if not seamless_down else self._seamless_cross_dn()
        return {
            "sine_mode": 2, "motor_direction": "Bidirectional",
            "sine_cross_up": cross_up, "sine_cross_dn": cross_dn,
            "sine_hold_amp": 16, "sine_amp_max": 45, "sine_ramp": 16,
            "comm_timing": "MediumHigh", "demag_compensation": "High",
            "low_rpm_power_protection": 0, "startup_power_min": 30, "startup_power_max": 18,
        }

    def _seamless_cross_dn(self) -> int:
        # a valid down threshold just below the up eRPM (used only if seamless down is proven working)
        up_erpm = round(HANDOFF_ERPM / SINE_CROSS_UP_ERPM_PER_UNIT) * SINE_CROSS_UP_ERPM_PER_UNIT
        _, dn = sine_crossover_bytes(up_erpm, up_erpm - 20)
        return dn

    # ---- eRPM self-cal (no encoder): fit `eff` from (cmd, tele_erpm) 6-step samples ----
    def self_cal_erpm(self, samples) -> float:
        """samples = [(cmd, tele_erpm), ...] taken in the 6-step region. tele_erpm/PP = mech; fit eff so
        rpm_6step matches. Returns the fitted eff and stores it. Encoder NOT needed."""
        ratios = []
        for cmd, erpm in samples:
            mech = abs(erpm) / self.pp
            ideal = self.kv * self.v * (DUTY6_A * abs(cmd) + DUTY6_B)   # eff=1 prediction
            if ideal > 1:
                ratios.append(mech / ideal)
        if ratios:
            self.eff = sum(ratios) / len(ratios)
        return self.eff

    # ---- emit a SpeedProfile ("motor pack" curve) for velctl/veltune ----
    def to_profile(self, cmd_lo=None, cmd_hi=700, n=14) -> SpeedProfile:
        """Sample the predicted curve into a monotone SpeedProfile (sine points + 6-step points)."""
        floor = self.handoff_cmd()
        # sine points below the handoff cmd, 6-step points from the handoff landing up
        sine_hi_cmd = self.cmd_for_sine(self.handoff_mech())
        pts = []
        for i in range(5):
            c = 60 + (sine_hi_cmd - 60) * i / 4
            pts.append((int(round(c)), round(self.rpm_sine(c), 1)))
        for i in range(n):
            c = floor + (cmd_hi - floor) * i / (n - 1)
            pts.append((int(round(c)), round(self.rpm_6step(c), 1)))
        mono, last_c, last_r = [], None, None
        for c, r in sorted(pts):
            if (last_c is None or c > last_c) and (last_r is None or r > last_r + 0.1):
                mono.append((c, r)); last_c, last_r = c, r
        return SpeedProfile(mono, motor=self.name, pole_pairs=self.pp,
                            source=f"parametric KV={self.kv:g} V={self.v:g} eff={self.eff:.3f}")

    def summary(self) -> str:
        return (f"Motor {self.name}: KV={self.kv:g} PP={self.pp} V={self.v:g}V eff={self.eff:.3f}\n"
                f"  no-load top   : {self.no_load_top():.0f} rpm\n"
                f"  sine top      : {self.sine_fullscale_mech():.0f} mech (PP-governed)\n"
                f"  handoff       : {self.handoff_mech():.0f} mech ; 6-step landing ~"
                f"{self.sixstep_floor_rpm():.0f} rpm ; gap {self.handoff_mech():.0f}-"
                f"{self.sixstep_floor_rpm():.0f}\n"
                f"  6-step curve  : rpm = KV*V*eff*({DUTY6_A}*cmd {DUTY6_B:+g})\n"
                f"  ESC config    : cross_up={self.esc_config()['sine_cross_up']} "
                f"cross_dn={self.esc_config()['sine_cross_dn']} low_rpm_prot=0")
