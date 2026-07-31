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
  * the sine->6-step UP-HANDOFF fires on a DUTY, not on a speed: when the ESC's 6-step duty demand
    clears a minimum-modulation floor DUTY6_HANDOFF_FLOOR => handoff_cmd = cmd_for_duty6(FLOOR,
    cross_up), which is KV- AND PP-independent in COMMAND and KV-scaled in the landing rpm. The
    CONFIGURED seam cross_up_erpm() is a different quantity: it still derives the ESC config and the
    C++ profile's Crossover, but nothing about the handoff comes through it any more. And at a low
    enough cross_up the firmware does not clamp, it REFUSES — see handoff_possible().

So Motor(KV, PP, V) alone predicts: no-load top, the sine + 6-step FF curves, the regime boundaries,
and the derivable ESC config. What it CANNOT predict is the dynamics (tau ~ inertia/load) and thus the
FB gains — those stay load-dependent (tune under load, or a conservative KV-scaled seed).

The universal-shape constants below are the 350KV bench fit; confirm/refresh them when a second KV
(e.g. 1000KV) is measured (2nd-order IR-drop/tolerance shows up as a per-motor `eff` the self-cal fits).
"""
from __future__ import annotations

from .config import (SINE_CROSS_TICKS_MAX, SINE_CROSS_TICKS_MIN, SINE_CROSS_UP_ERPM_MIN,
                     SINE_CROSS_UP_ERPM_PER_UNIT, handoff_ticks, sine_crossover_bytes,
                     up_erpm_handoffable)
from .velocity import SpeedProfile

# --- universal firmware/DShot shapes (calibrated once; KV-independent) ---------------------
# 6-step: duty_eff(cmd) = rpm / (KV*V) = A6*cmd + B6, for cmd in the 6-step region. (350KV fit.)
DUTY6_A = 0.00265
DUTY6_B = -1.233
# The ESC's `cross_rescale_duty` maps the 6-step demand relative to `Cross_Up`, so changing Cross_Up
# SHIFTS this affine's intercept and leaves its slope alone. DUTY6_A/B above were fitted at Cross_Up=34.
#
# RE-FITTED 2026-07-31 over a 14-UNIT baseline: 0.0421 -> 0.04134 (-1.8 %). The old value came from
# `host/duty_sweep.py` at cross_up 34 and 36 — TWO units, so a 2 % error in either intercept became a
# 2 % error in K, and it did: the frozen K put the handoff +3.90 counts out at cross_up 50 while
# landing inside the bracket at 34 and 36, i.e. a TREND, which is the signature of a wrong slope
# rather than a wrong offset.
#
# What is actually measured here is the RATIO K/DUTY6_A = command counts per cross_up unit, and three
# independent routes agree on it (`host/reports/`):
#     15.60  duty_sweep_20260731_{124644,124916}.csv — no-load pairs at cross_up 36 and 50, joint fit
#            with a SHARED slope (fitting two free slopes is what produced the earlier wrong answer)
#     15.63  the measured 6-step branches of sinesweep_20260731_{014004,012848}.csv, 34 <-> 50
#     15.59  the measured handoff commands themselves, (734.5 - 485.0) / 16
# against the old model's K/A = 15.89. Residuals go from -0.79 / +0.49 / +3.90 counts at cross_up
# 34/36/50 to -0.79 / -0.09 / -0.69, i.e. a trend becomes a constant ~half-count.
#
# WHY THE RATIO AND NOT K DIRECTLY. The 2026-07-31 sweeps carry no PSU current readings, so their
# ABSOLUTE duty is biased low (duty_sweep says so in its own header) and their apparent DUTY6_A is 4 %
# under. That bias is a function of rpm, and the two sweeps were run over the SAME rpm range
# (1480..2813 vs 1523..2847) precisely so it cancels out of the intercept DIFFERENCE. So K/A is sound
# and the absolute scale is not — hence `K = DUTY6_A * 15.5998`, with DUTY6_A and DUTY6_B left frozen.
# Re-measuring the absolute duty map needs the PSU current column (`duty_sweep.py --currents`).
#
# Two derived bounds move with this: the emittable band max stays 47 at cmd_hi=700, and the
# handoff band max goes 66 -> 67. Both are computed, never written down — see
# _max_emittable_cross_up() / _max_handoff_cross_up(). Mirrors vel::DUTY6_CROSSUP_* in
# lib/vel_control/motor_model.h — keep the two in step.
DUTY6_CROSSUP_REF = 34.0
DUTY6_CROSSUP_K = 0.04134

# --- the sine->6-step UP-HANDOFF TRIGGER: a minimum-modulation DUTY floor ------------------
# The ESC hands off when its 6-STEP DUTY DEMAND clears a floor, NOT when the forced field reaches a
# speed. Measured 2026-07-31 on a 930KV at 11.2 V (host/sinesweep.py, 1-count sweeps, AS5600 encoder):
# a 2.66x different KV from the 350KV that DUTY6_A/B were fitted on, and the two motors hand off at
# the SAME COMMAND — which is what a duty law predicts and a speed law forbids.
#
# THE READING. On the sweep-DOWN leg, below the command the up leg handed off at, the rotor stays in
# 6-step but LOCKS at one speed and ignores the command: the modulator has bottomed out.
#
# ROW-SELECTION RULE, stated first because the number is only worth what its rule is worth. Both
# clauses are load-bearing; the second is what makes the boundary row unambiguous:
#   (a) DOWN-leg rows only, AT OR BELOW the command at which the UP leg first handed off; and
#   (b) of those, the contiguous run from the LOWEST command in the file over which |enc_mean| is
#       constant to within the cogging spread — i.e. every row before the 6-step branch starts rising.
# Locked rows are told apart from forced sine by enc_spread: 11..20 rpm of cogging when locked, 100..250
# rpm of dither when the rotor is chasing the field. Clause (b) applied ALONE keeps cross_up 50's cmd
# 735 (537.8 rpm, already on the rising branch) and gives n=31 / 523.50; clause (a) excludes it. Applied
# to all five cited fixtures:
#   host/reports/sinesweep_20260731_014004.csv  cross_up 34  down cmd 465..484  mean 523.44  n=6
#   host/reports/sinesweep_20260731_013410.csv  cross_up 36  down cmd 510..516  mean 522.56  n=7
#   host/reports/sinesweep_20260731_013236.csv  cross_up 50  down cmd 728..734  mean 522.19  n=7
#   host/reports/sinesweep_20260731_012848.csv  cross_up 50  down cmd 690..730  mean 522.15  n=9  (coarse)
#   host/reports/sinesweep_20260731_013633.csv  cross_up 30  NO LOCKED ROWS — this ESC never handed off
#                                               on either leg, which is the refusal case; see
#                                               handoff_possible(). It contributes 0 rows by the rule.
# ALL FIVE: mean 522.525 rpm, sd 0.81, range 3.8, n=29 -> COMMITTED as 522.53. Per cross_up: 523.44
# (n=6) / 522.56 (n=7) / 522.17 (n=16). The three 1-count "fold" files alone give 522.694 (n=20), so a
# reader who reduces only those lands within 0.17 rpm = 0.006 command counts of the committed number.
# That the value is the same at cross_up 34, 36 and 50 — a 16-unit span, 250 commands apart — is the
# evidence that it is a property of the modulator and not of the seam.
#
# SUPERSEDED: 526.7 +- 0.8 rpm. An earlier reduction of the same five files committed 526.7. It is
# WRONG, not a defensible alternative: its row filter (enc_spread < 60 and |enc_mean| < 600) admitted
# the first rows ABOVE the lock, where the 6-step branch has already started rising — cross_up 36
# down-leg cmd 517 = 552.4 rpm, cross_up 50 cmd 735 = 537.8, cross_up 34 cmd 486 = 553.2 — and those
# pulled the mean up by 4.2 rpm. The rule above excludes them by construction. Recorded here so the
# same filter is not written again. (The error was small — 0.8 % in duty, 0.15 command counts — but it
# was in the direction that made the cross_up 36 prediction MISS its measured bracket.)
#
# THE LAW AND ITS RESIDUALS. handoff_cmd = cmd_for_duty6(DUTY6_HANDOFF_FLOOR, cross_up) gives
# 484.21 / 515.41 / 733.81 at cross_up 34 / 36 / 50, against the measured up-leg brackets
# (484, 486] / (515, 516] / (734, 735] = 485.0 +- 1.0 / 515.5 +- 0.5 / 734.5 +- 0.5, i.e. residuals
# -0.79 / -0.09 / -0.69 counts, mean -0.52 with a 0.70 spread over a 250-command span.
#
# That flatness is the point, and it is NEW. Against the pre-2026-07-31 DUTY6_CROSSUP_K = 0.0421 the
# residuals were -0.79 / +0.49 / +3.90 — a TREND of +0.29 counts per cross_up unit, which is the
# signature of a wrong SLOPE, not of a wrong floor. Re-fitting K (see its own comment above) removed
# it: the trend is now +0.006 counts/unit, i.e. 0.10 counts across the whole 16-unit span. What is
# left is a constant ~half-count offset, below the 1-count resolution of the sweeps that produced the
# brackets and well inside the +-0.5..1.0 the brackets themselves carry. Nothing in this floor
# absorbed the K error and nothing about it changed when K moved — DUTY6_HANDOFF_FLOOR is a duty read
# off the cross_up 34 reference leg, where the K term is identically zero.
#
# THE ALTERNATIVE THIS REFUTES. A fixed-SPEED floor ("hand off once the 6-step branch reaches 522.5
# rpm" — the other reading of the same lock) is KV-dependent, and for the 350KV at 11.1 V / cross_up 36
# it predicts cmd 547.2. That motor's own independently measured bracket is (515, 520]
# (host/tests/data/sinesweep_fold_noprop_20260730_033417.csv: cmd 515 enc 192 with 122 rpm of spread,
# still riding the field; cmd 520 enc 205.7 with 25 rpm of spread, off it). The duty law predicts
# 515.4, inside the bracket; the speed law misses it by 27 counts.
#
# THIS RUN ALSO RE-CONFIRMS DUTY6_A/B ON A SECOND MOTOR — and neither they nor DUTY6_CROSSUP_K are
# re-fitted here: 930KV, cross_up 34, cmd 520 -> measured duty 1519.09/(930*11.2) = 0.14584 vs model
# 0.00265*520 - 1.233 = 0.14500, 0.6 % out on a KV 2.66x away from the fit; and K to 1.6 % over a
# 16-unit baseline (above).
#
# HONEST LIMIT (docs/README.md: "either a measured fact or an explicit statement that something is NOT
# known"). ONE motor, ONE loom, ONE supply, read off the DOWN leg only. "Minimum modulation" is an
# INTERPRETATION of a flat, command-independent lock: no modulator register was read, and nothing here
# proves the floor is a firmware constant rather than this ESC's. The refusal predicate below has bench
# evidence on its MAX side only. Calibrate in situ.
DUTY6_HANDOFF_FLOOR = 522.53 / (930.0 * 11.2)     # = 0.050166 effective duty


def duty6(cmd, cross_up=DUTY6_CROSSUP_REF):
    """EFFECTIVE 6-step duty = rpm_measured/(KV*V_supply) at command `cmd` on an ESC set to `cross_up`.

    NOTE THIS IS A LOADED CURVE. DUTY6_A/B were fitted to measured rpm on a real bench, so the drag of
    that rotor is already inside them: this is the duty implied by the speed the rotor SETTLED at, not
    the electrical duty the winding sees. Anything that puts this into a torque balance must add the
    calibration drag's duty cost back first — see pico_esc.plant.duty_ref_offset, and the drag
    double-count it exists to prevent.

    The `cross_up` DEFAULT survives here (and in cmd_for_duty6) so that MotorModel and every existing
    caller stay numerically identical to before the term existed. Every call site in host/ passes it
    explicitly, which `test_measured_plant.test_no_call_site_relies_on_a_duty_default` enforces by AST
    scan — but that scan covers `host/` and `host/pico_esc/` only, so a NEW module elsewhere could still
    reach the default. Prefer `MotorModel.duty6`/`MeasuredPlant.duty_6step`, which cannot."""
    return DUTY6_A * abs(cmd) + DUTY6_B - DUTY6_CROSSUP_K * (cross_up - DUTY6_CROSSUP_REF)


def cmd_for_duty6(duty, cross_up=DUTY6_CROSSUP_REF):
    """Exact inverse of duty6(). Same default caveat as duty6()."""
    return (duty - DUTY6_B + DUTY6_CROSSUP_K * (cross_up - DUTY6_CROSSUP_REF)) / DUTY6_A


def _min_handoff_cross_up() -> int:
    """Lowest `sine_cross_up` byte the firmware will hand off at (= 34). DERIVED, because the tick
    window is truncated: `int(40000/up) <= MAX` iff `up > 40000/(MAX+1)`, so the bound is one byte
    above that eRPM, not above SINE_CROSS_UP_ERPM_MIN."""
    return int(4 * 10000.0 / ((SINE_CROSS_TICKS_MAX + 1) * SINE_CROSS_UP_ERPM_PER_UNIT)) + 1


def _max_handoff_cross_up() -> int:
    """Highest `sine_cross_up` byte whose handoff command is still inside +-CMD_FULL_SCALE (= 67).

    DERIVED rather than written as 67, because it is a function of DUTY6_CROSSUP_K: solving
    cmd_for_duty6(FLOOR, cu) == CMD_FULL_SCALE for cu gives REF + (CMD_FULL_SCALE*A - FLOOR + B) / K,
    and the byte must not exceed it (67.06 today). It moved 66 -> 67 when K was re-fitted on
    2026-07-31, which is exactly why it is computed and never written down."""
    return int(DUTY6_CROSSUP_REF
               + (CMD_FULL_SCALE * DUTY6_A - DUTY6_HANDOFF_FLOOR + DUTY6_B) / DUTY6_CROSSUP_K)


def _max_emittable_cross_up(cmd_hi: float) -> int:
    """Highest byte that still fits a 6-step SECTION inside `cmd_hi` (= 47 at the default 700).

    A DIFFERENT bound from `_max_handoff_cross_up()` (= 67) and the distinction has bitten twice: 67
    is where the handoff command stops being a command at all, while this is where the handoff
    outruns the top of the curve being sampled. Bytes between the two hand off perfectly well and
    still cannot be given a parametric FF curve, so quoting 34..67 as the remedy for a curve failure
    sends the operator straight back into the other raise. Same solve as the sibling, with `cmd_hi`
    in place of CMD_FULL_SCALE. (The 2026-07-31 K re-fit moved the sibling 66 -> 67 and left this one
    at 47, which is the clearest demonstration that they are not the same number.)"""
    return int(DUTY6_CROSSUP_REF
               + (cmd_hi * DUTY6_A - DUTY6_HANDOFF_FLOOR + DUTY6_B) / DUTY6_CROSSUP_K)
# sine: mech RPM = (cmd/1000) * SINE_FULLSCALE_MECH, SINE_FULLSCALE_MECH = SINE_FULLSCALE_ERPM / PP.
SINE_FULLSCALE_ERPM = 2499.0      # firmware S1 full-scale eRPM at cmd 1000 (SINE_RCP_SHIFT=3 etc.)
# The command magnitude the protocol tops out at. A "handoff command" above it is not a command at all
# — nothing can ever be sent that reaches it — which is why handoff_possible() treats it as a refusal
# rather than reporting an unreachable number. Mirrors vel::CMD_FULL_SCALE.
CMD_FULL_SCALE = 1000.0
HANDOFF_ERPM = SINE_CROSS_UP_ERPM_MIN     # ~1333 eRPM: the lowest the firmware will hand off (min slip)


class MotorModel:
    """Motor(KV [rpm/V], pole_pairs, v_supply [V]). eff (0..1] absorbs IR-drop / KV tolerance / losses
    and is what the eRPM self-cal fits (default 1.0 = ideal). Optional cmd_floor overrides the predicted
    6-step floor cmd."""

    def __init__(self, kv, pole_pairs, v_supply, eff=1.0, name="", cross_up=DUTY6_CROSSUP_REF):
        self.kv = float(kv)
        self.pp = int(pole_pairs)
        self.v = float(v_supply)
        self.eff = float(eff)
        self.name = name or f"{int(kv)}kv"
        # The ESC's sine_cross_up. Defaults to the value DUTY6_A/B were fitted at, so an existing
        # caller that never passes it gets byte-identical numbers to before.
        self.cross_up = float(cross_up)

    # ---- scalar predictions ----
    def no_load_top(self) -> float:
        """No-load top mech RPM ~ KV*V*eff."""
        return self.kv * self.v * self.eff

    def sine_fullscale_mech(self) -> float:
        """Forced-sine full-scale mech RPM (cmd 1000) = SINE_FULLSCALE_ERPM / PP. KV-INDEPENDENT."""
        return SINE_FULLSCALE_ERPM / self.pp

    def cross_up_erpm(self) -> float:
        """The SEAM the configured `sine_cross_up` puts the handoff at, in eRPM — floored at the
        firmware's own minimum, because the ESC cannot hand off below it however low the byte is.

        cross_up does TWO independent things and only one of them used to be modelled: it shifts the
        6-step duty (DUTY6_CROSSUP_K) AND IT MOVES THE SEAM. Anything that derives a handoff speed or
        command must come through here, or it silently keeps using the firmware minimum while the
        operator has configured something else (at cross_up 50 that is a ~220-count band of pure
        forced sine reported as valid 6-step).

        Mirrors vel::MotorModel::crossUpErpm() in lib/vel_control/motor_model.h. The C++ side floors
        at the literal 1333.0f while this floors at SINE_CROSS_UP_ERPM_MIN = 4*10000/30 = 1333.333...,
        so the two agree to 0.34 eRPM (0.03 %) — inside the 0.5 eRPM parity band the tests assert, and
        far inside one cross_up unit (39.0625 eRPM). cross_up 34 -> 1328.1 eRPM is just UNDER the
        floor, which is why 34 reads as ~the firmware minimum.
        """
        return max(self.cross_up * SINE_CROSS_UP_ERPM_PER_UNIT, SINE_CROSS_UP_ERPM_MIN)

    def handoff_possible(self) -> bool:
        """False when this `sine_cross_up` has no reachable up-handoff at all.

        TWO conditions, and both mean the same operational thing — no command you can send makes this
        ESC hand off:

        1. THE FIRMWARE REFUSES. It seeds Comm_Period4x from a 4-sector tick window and only fires the
           up-handoff when that window is in [SINE_CROSS_TICKS_MIN, SINE_CROSS_TICKS_MAX] ticks;
           outside it the ESC stays in forced sine for ever. Delegated to
           `config.up_erpm_handoffable()`, the ONE definition of that window (mirrored by
           vel::handoffPossibleAt in lib/vel_control/motor_model.h) — read that docstring for where it
           deliberately disagrees with `config.sine_crossover_bytes()`, which tests a continuous
           pre-rounding eRPM band and rejects the very cross_up 34 the bench hands off at. Keyed on the
           RAW byte (`cross_up * SINE_CROSS_UP_ERPM_PER_UNIT`), never on `cross_up_erpm()`: that one
           FLOORS a low threshold up to the firmware minimum, which is precisely the mistake this
           predicate exists to stop — the firmware does not clamp, it refuses.
        2. THE COMMAND IS UNREACHABLE. The trigger is a duty and cross_up shifts it by
           DUTY6_CROSSUP_K/DUTY6_A = 15.6 counts per unit, so a high enough byte puts the handoff
           command past the +-CMD_FULL_SCALE the protocol carries: 999.0 at cross_up 67, 1014.6 at 68,
           1513.8 at 100. Folding this in here is what keeps `handoff_cmd()` and
           `vel::handoffCmdFor()` agreeing on the same bytes, and what stops the C++ HANDOFF_CMD_NONE
           sentinel (1001, the value a refusal reports) from colliding with a genuine handoff command.
           This half is arithmetic on the protocol's range, NOT a measurement.

        Measured 2026-07-31 (host/reports/sinesweep_20260731_013633.csv): at cross_up 30 -> 1171.9
        eRPM -> a 34-tick window, the 930KV never handed off on either leg all the way to cmd 500,
        while the same ESC at 34/36/50 handed off exactly where DUTY6_HANDOFF_FLOOR says.

        Truncate vs round is NOT discriminated by the data and is not claimed: ticks = 40000/up_erpm
        and the byte grid is coarse, so the boundary is unambiguous either way (cross_up 33 -> 31.03
        ticks refuses, 34 -> 30.12 accepts) and no reachable configuration lands near a half tick. The
        tick window's MIN side has no bench evidence and is now unreachable in any case: it would take
        cross_up 114, and (2) refuses everything from 67 up.

        One bit is deliberately the primary API — every non-message caller in both languages wants
        exactly "can this hand off?". Callers that need to TELL the operator which condition fired read
        `_handoff_refusal()`, which is the single place the reasons are enumerated."""
        return self._handoff_refusal() is None

    def _handoff_refusal(self):
        """None when a reachable handoff exists; otherwise the phrase naming WHICH condition refused.

        ONE enumeration of the reasons, because the set is open: written out at each message site, a
        third condition would silently misattribute at every site that assumed "not the tick one =>
        the command one". The phrases are fragments, so a caller supplies its own sentence."""
        up_erpm = self.cross_up * SINE_CROSS_UP_ERPM_PER_UNIT
        if not up_erpm_handoffable(up_erpm):
            return (f"a {handoff_ticks(up_erpm):.1f}-tick handoff seed window, outside the firmware's "
                    f"[{SINE_CROSS_TICKS_MIN}, {SINE_CROSS_TICKS_MAX}], which it refuses")
        cmd = cmd_for_duty6(DUTY6_HANDOFF_FLOOR, cross_up=self.cross_up)
        if not 0.0 < cmd <= CMD_FULL_SCALE:
            return (f"a handoff command of {cmd:.0f}, unreachable past the "
                    f"+-{CMD_FULL_SCALE:g} the protocol carries")
        return None

    def handoff_cmd(self) -> float | None:
        """The command the sine->6-step up-handoff FIRES at: where the 6-step duty demand clears the
        minimum-modulation floor. None when the firmware refuses (see handoff_possible).

        THE TRIGGER, and deliberately not the same quantity as pico_esc.plant.MeasuredPlant.seam_cmd(),
        which is where the 6-step BRANCH OVERTAKES the forced field — a NECESSARY condition for the
        handoff to be STABLE, not the thing that fires it. On the 930KV the branches cross at 471.4 /
        503.6 / 728.9 at cross_up 34 / 36 / 50 while the bench measured the handoff at 485.0 / 515.5 /
        734.5 — 6 to 14 counts apart, in the same direction every time — and the seam moves with KV and
        supply where this does not. So they are kept separate and each documents the other. See
        DUTY6_HANDOFF_FLOOR for the measurement.

        KV- and PP-INDEPENDENT (it is a duty, and duty6/cmd_for_duty6 carry no motor terms): 484.21 /
        515.41 / 733.81 at cross_up 34 / 36 / 50 on every motor. It is `sixstep_floor_rpm` that scales
        with KV. Returns None rather than NaN on refusal: `nan <= x` is False, so a NaN would route
        `regime`/`thrust_for` silently into the 6-step branch, where None raises TypeError at the first
        arithmetic — the direction a refused handoff has to fail in."""
        if not self.handoff_possible():
            return None
        return cmd_for_duty6(DUTY6_HANDOFF_FLOOR, cross_up=self.cross_up)

    def handoff_mech(self) -> float | None:
        """Mech RPM the ROTOR is at when the handoff fires — still on the forced field, so it is the
        sine curve evaluated at handoff_cmd (172.9 / 184.0 / 262.0 mech at PP=7 and cross_up 34/36/50).
        PP-governed and KV-independent. None when the firmware refuses to hand off."""
        cmd = self.handoff_cmd()
        return None if cmd is None else self.rpm_sine(cmd)

    def sixstep_floor_rpm(self) -> float | None:
        """The 6-step 'landing' rpm — the 6-step speed at the handoff command. The up-crossover jumps
        from sine (~handoff_mech) straight to here; the band between is the no-steady-state gap.

        Equal to KV*V*eff*DUTY6_HANDOFF_FLOOR by construction (cmd_for_duty6 is the exact inverse of
        duty6), so it scales with KV*V and is INDEPENDENT of cross_up: 522.5 rpm on the 930KV at
        11.2 V, 194.9 on the 350KV at 11.1 V, 196.7 at 11.2 V. The old law made it fall as cross_up
        rose; that was an artifact of deriving the trigger from the seam. None when the firmware
        refuses to hand off."""
        cmd = self.handoff_cmd()
        return None if cmd is None else self.rpm_6step(cmd)

    # ---- FF curves ----
    def rpm_sine(self, cmd: float) -> float:
        """Forced-sine mech RPM at a signed cmd (PP-governed)."""
        return (cmd / 1000.0) * self.sine_fullscale_mech()

    def rpm_6step(self, cmd: float) -> float:
        """6-step mech RPM at cmd (KV*V-scaled universal shape), clamped [0, no-load top]."""
        duty = duty6(cmd, self.cross_up)
        if duty <= 0:
            return 0.0
        rpm = self.kv * self.v * self.eff * duty
        return min(rpm, self.no_load_top())

    def cmd_for_6step(self, rpm: float) -> float:
        """Invert the 6-step curve: cmd that produces `rpm` mech (6-step region)."""
        duty = rpm / (self.kv * self.v * self.eff)
        return cmd_for_duty6(duty, self.cross_up)

    def cmd_for_sine(self, rpm: float) -> float:
        return rpm / self.sine_fullscale_mech() * 1000.0

    def thrust_for(self, rpm: float) -> float:
        """Regime-aware inverse: sine below the handoff, 6-step above the 6-step floor. The gap between
        (sine reachable) and (6-step floor) has NO steady state — a target there snaps to the nearest
        reachable point (6-step floor), and the caller should treat it as 'not holdable'.

        When the firmware refuses to hand off (handoff_possible False) the whole range is forced sine,
        so every target inverts through the sine curve — including ones no rotor will reach, which is
        the truthful answer: this ESC config has no 6-step branch to ask for."""
        s = 1.0 if rpm >= 0 else -1.0
        a = abs(rpm)
        handoff = self.handoff_mech()
        if handoff is None:
            return s * self.cmd_for_sine(a)
        if a <= handoff:
            return s * self.cmd_for_sine(a)
        if a >= self.sixstep_floor_rpm():
            return s * self.cmd_for_6step(a)
        # in the gap: snap up to the 6-step floor (cannot hold between)
        return s * self.cmd_for_6step(self.sixstep_floor_rpm())

    def in_gap(self, rpm: float) -> bool:
        """True if `rpm` falls in the no-steady-state crossover gap (FB should be gain-scheduled off here).

        The gap is [handoff_mech, sixstep_floor_rpm] and under the duty law it is no longer wide on
        every motor. It can be NARROW (350KV at 11.1 V, cross_up 36: 184.0 -> 194.9 rpm, 11 rpm) or
        EMPTY/INVERTED (350KV, cross_up 50: the trigger fires at 262.0 mech while the landing is still
        194.9, so nothing is in the gap) — an inverted gap means the handoff fires where the 6-step
        branch is still BELOW the forced field, which is exactly the instability
        pico_esc.plant.MeasuredPlant.seam_cmd() is the necessary-condition test for. False everywhere
        when the firmware refuses to hand off: there is no 6-step side to gap to."""
        handoff = self.handoff_mech()
        return handoff is not None and handoff < abs(rpm) < self.sixstep_floor_rpm()

    def regime(self, rpm: float) -> str:
        """"sine" / "gap" / "6step". Sine is tested FIRST and on purpose: below the handoff command the
        ESC IS in forced sine, so an empty or inverted gap (see in_gap) must read as sine, not as 6-step.
        Everything is "sine" when the firmware refuses to hand off."""
        a = abs(rpm)
        handoff = self.handoff_mech()
        if handoff is None:
            return "sine"
        if a <= handoff:
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
            ideal = self.kv * self.v * duty6(cmd, self.cross_up)        # eff=1 prediction
            if ideal > 1:
                ratios.append(mech / ideal)
        if ratios:
            self.eff = sum(ratios) / len(ratios)
        return self.eff

    # ---- emit a SpeedProfile ("motor pack" curve) for velctl/veltune ----
    def to_profile(self, cmd_lo=None, cmd_hi=700, n=14) -> SpeedProfile:
        """Sample the predicted curve into a monotone SpeedProfile (sine points + 6-step points).

        Raises ValueError, rather than emitting a curve with no usable 6-step section, in BOTH ways
        that section can fail to exist. A FF curve whose upper half is missing or is a backwards ramp
        does not fail — it silently mis-scales velctl, which is the whole failure mode this model is
        built to avoid:

          * the handoff is REFUSED at this `sine_cross_up` (see handoff_possible), so there is no
            6-step branch at all; or
          * the handoff command is at or above `cmd_hi`, so fewer than two 6-step points fit inside the
            sampled range. At the default cmd_hi=700 that is any cross_up >= 48 — INCLUDING cross_up
            50, the setting this bench run itself used. Left unguarded it produced 8 points ending
            (736, 444.8), (739, 522.5): the 6-step loop steps BACKWARDS, and the inverted curve then
            returns the same command for 600, 1500 and 3000 rpm alike.

        No `to_profile()` call site is fed from a `--cross-up` flag today, so neither raise is
        reachable from the CLI: motorpack.py:37,90 and velrun.py:182 never pass `cross_up` at all
        (neither tool has the flag), and sim_obs.py:227 reads `plant.motor` from a plant that
        `startup_run()` re-derives with a bare `preset(cfg)` — its own `--cross-up` flag is dropped on
        that path (contrast sim_obs.py:343, which does honour it, but never calls to_profile). Five
        tools DO have a `--cross-up` flag (sim_obs, softsensor, obschar, duty_sweep, loadtest), so the
        moment one of them is wired to a MotorModel it must catch this and print the message."""
        why = self._handoff_refusal()
        if why is not None:
            up_erpm = self.cross_up * SINE_CROSS_UP_ERPM_PER_UNIT
            raise ValueError(
                f"sine_cross_up={self.cross_up:g} ({up_erpm:.1f} eRPM) gives {why}: this ESC will "
                f"never hand off to 6-step, so there is no 6-step curve to emit. Use a sine_cross_up "
                f"in {_min_handoff_cross_up()}..{_max_handoff_cross_up()} "
                f"(measured: cross_up 30 never handed off, 34 did) — but only "
                f"{_min_handoff_cross_up()}..{_max_emittable_cross_up(cmd_hi)} also fits a 6-step "
                f"section inside cmd_hi={cmd_hi:g}; above that you land on the other raise below. "
                f"If this ESC runs STOCK Bluejay or BLHeli-S it has no forced-sine crossover at all "
                f"and `sine_cross_up` read back as an unwritten sentinel (0xFF=255, or 0) — there is "
                f"no byte to fix, and this parametric low-speed branch does not describe that firmware")
        floor = self.handoff_cmd()
        if floor >= cmd_hi:
            raise ValueError(
                f"the handoff command at sine_cross_up={self.cross_up:g} is {floor:.1f}, at or above "
                f"cmd_hi={cmd_hi:g}: fewer than two 6-step points fit, so the sampled curve would step "
                f"BACKWARDS and invert to one command for every rpm. Raise cmd_hi above {floor:.0f} "
                f"(and check the ESC can actually hold it) or lower sine_cross_up")
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
        # CAPABILITIES, which this used to omit entirely — and omitting them is not neutral, because
        # SpeedProfile._capability defaults every flag to False. So a parametric profile advertised
        # `sine_lowspeed: false` while its own first five points ARE the forced-sine branch: a curve
        # contradicting its own metadata, and that metadata is what gets written to YAML and shipped.
        #   sine_lowspeed True — we only get here when handoff_possible(), and the five points below
        #     `floor` are sampled from rpm_sine(). On stock Bluejay / BLHeli-S there is no such branch,
        #     and that case cannot reach here: the refusal above fires on the sentinel cross_up.
        #   down_catch stays absent (=> False). The DOWN handoff is NOT modelled and, where it has been
        #     measured, it does not work: the 930KV never fell back at all (its 6-step floor sits 2.8x
        #     above the cross_dn threshold). Claiming it from a parametric model would be an invention.
        return SpeedProfile(mono, motor=self.name, pole_pairs=self.pp,
                            capabilities={"sine_lowspeed": True},
                            source=f"parametric KV={self.kv:g} V={self.v:g} eff={self.eff:.3f}")

    def summary(self) -> str:
        if self.handoff_possible():
            up = self.handoff_mech()
            land = self.sixstep_floor_rpm()
            handoff = (f"  handoff       : {up:.0f} mech (cmd {self.handoff_cmd():.0f}) ; 6-step "
                       f"landing ~{land:.0f} rpm ; gap {up:.0f}-{land:.0f}\n")
        else:
            # WHICH refusal it is, because the two have opposite fixes (raise/lower the byte).
            handoff = (f"  handoff       : none — cross_up {self.cross_up:g} gives "
                       f"{self._handoff_refusal()}; forced sine at every command\n")
        return (f"Motor {self.name}: KV={self.kv:g} PP={self.pp} V={self.v:g}V eff={self.eff:.3f}\n"
                f"  no-load top   : {self.no_load_top():.0f} rpm\n"
                f"  sine top      : {self.sine_fullscale_mech():.0f} mech (PP-governed)\n"
                + handoff +
                f"  6-step curve  : rpm = KV*V*eff*({DUTY6_A}*cmd {DUTY6_B:+g}"
                f"{' ' if self.cross_up == DUTY6_CROSSUP_REF else f' - {DUTY6_CROSSUP_K}*(cross_up-{DUTY6_CROSSUP_REF:g})'})\n"
                f"  ESC config    : cross_up={self.esc_config()['sine_cross_up']} "
                f"cross_dn={self.esc_config()['sine_cross_dn']} low_rpm_prot=0")
