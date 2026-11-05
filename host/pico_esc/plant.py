# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""pico_esc.plant — the 350KV plant with its STRUCTURE fixed by physics and its NUMBERS provenanced.

Three layers, with a hard boundary at each so that changing the load touches exactly one of them:

  1. FIRMWARE / ELECTRICAL — `duty6(cmd, cross_up)`, the forced-sine field law, the seam as
     `max(sine field, 6-step)`, the up-only crossover slew, the `tele` floor. Nothing here knows the
     rotor exists.
  2. ROTOR — `J*dw/dt = tau_drive(cmd, w) - tau_drag(w)`. `tau_drive` is ONE voltage-source torque law
     that DEGENERATES into the brake at zero duty (see below), so the static curve, the step response
     and the zero-throttle brake all come out of the same expression.
  3. LOAD — a single swappable `drag` callable. Water changes this and nothing else.

WHY THE OLD MODEL WAS REPLACED. `sim_obs.PlantObs` used a regime-dependent first-order lag
(0.18 s in forced sine / 0.025 s in 6-step), a hard 250 rpm regime threshold and a 0.28 s
handoff deadtime. Every one of those absorbs drag, so a load change invalidates all four — the exact
failure this module exists to end. And the 0.18 s lag was fitted to cogging noise.

`duty6` IS A LOADED CURVE, AND THAT MATTERS (fixed 2026-07-30, iteration 2)
------------------------------------------------------------------------
`DUTY6_A/B` were fitted as `rpm_measured/(KV*V)` on a real bench, so the drag of that rotor is already
baked into them: `KV*V*eff*duty6(cmd)` is the speed the rotor SETTLES at, not the speed a frictionless
rotor would reach. Subtracting `tau_drag` on top of that drive law counts drag twice, and the first
version of this module did exactly that — integrated to convergence it landed 12.2 % BELOW `steady()`
at PROP cmd 543, while claiming in this very docstring that it could not.

The fix is to recover the ELECTRICAL duty the winding actually sees:

    duty_elec(cmd) = duty6(cmd) + tau_drag_ref(w6) * (R^2 + X^2) / (K_T * R * V * eff),  w6 = KV*V*eff*duty6

`tau_drag_ref` is the drag the CALIBRATION was taken against (`PlantParams.drag_ref`) — a property of
`duty6`, not of the rotor now being simulated, so it must NOT move when the load changes. In air, where
`drag == drag_ref`, the offset cancels exactly and the ODE's fixed point IS `steady(cmd)`
(asserted by `test_step_converges_to_steady`, which integrates rather than inspecting `tau_6step`).
In water `drag > drag_ref` and the plant slows down — which is the entire point of the layering.

That correction is +12.2 % of duty at PROP cmd 543 falling to +5.6 % at cmd 680, i.e. exactly the
deficit the double-count produced.

THE ONE TORQUE LAW. A voltage-source winding driven at duty `d` against back-EMF `w/KV_rad`, with the
in-phase (torque-producing) component of the current only:

    tau_drive = K_T * (V*eff*d - w/KV_rad) * R / (R^2 + (pp*w*L)^2)

  * At `d = duty_elec(cmd)` and `drag = drag_ref` its zero is `rpm = KV*V*eff*duty6(cmd, cross_up)` —
    the measured static curve, for any L.
  * At `d = 0` it becomes `-K_T^2*w*R/(R^2+(pp*w*L)^2)` — the textbook short-circuit / asynchronous
    brake, peaking at `w = R/(pp*L)`. So the zero-throttle brake is not a separate model; it is this
    law with the duty removed. `tau_drive(0, w) == -tau_short_circuit(w)` is asserted exactly.
  * `L/R ~ 1 ms`, so the current loop is treated as algebraic (see docs/measured-plant.md).

FORCED SINE IS A DIFFERENT LAW, ON PURPOSE. Below the seam the ESC is not sensing anything: it FORCES
commutation at a rate set by the command, and a synchronous rotor runs at EXACTLY that rate with a load
angle — not at a slip. So the sine branch is a synchronous lock with a saturation
(`SINE_PULLOUT_NM`), not the short-circuit law. That is why the open-loop down-step (600 -> 480)
decelerates at a CONSTANT ~20 krpm/s: the rotor is out of step and being pulled at the pull-out torque
until it captures. Modelling it with the short-circuit law instead predicts ~4x too fast.

WHAT IS NOT A STEADY SPEED. Below `SINE_COHERENT_CMD_MIN` the rotor cogs, stalls and reverses (up to 7
reversals inside one 3 s dwell). `coherent(cmd)` returns False there and `steady()` refuses to return a
speed; the measured dispersion is exposed as data (`spread_at`, `reversals_at`) instead of being
smoothed into a mean that looks like a measurement. See the Lake note "a mean is already an ideal
low-pass" — this module deliberately does not filter that regime, it declines to model it.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace   # noqa: F401  (replace is re-exported)
from typing import Callable, Optional

try:                                   # the measured inventory is a SEPARATE, per-installation asset.
    from . import measured as M        # Without it this module still gives you the plant STRUCTURE;
except ImportError:                    # you supply your own PlantParams. `preset()` is what needs it.
    M = None
from .motor_model import MotorModel, duty6


def _require_inventory(what):
    if M is None:
        raise RuntimeError(
            f"{what} needs a measured inventory (pico_esc/measured.py), which is not part of this "
            "distribution because every value in it is specific to one motor, one wiring loom and one "
            "supply. Build a PlantParams yourself from your own calibration — see the class docstring "
            "— or run host/coastdown.py, duty_sweep.py and sinesweep.py to measure your own.")


RAD = 2.0 * math.pi / 60.0          # rpm -> rad/s

# WHICH ESC FIRMWARE, because in forced sine the two report different things and every closed-loop
# conclusion below the handoff turns on which one you are talking to. Deliberately HERE and not in
# `measured`: the inventory is per-installation (one motor, one loom, one supply) and is not
# distributed, while firmware behaviour is a property of the firmware and true for everybody. Putting
# them there would also make this module fail to IMPORT without the inventory, since the default below
# is evaluated at class-definition time and `M` is None in that case.
TELE_SINE_S3 = "s3-virtual-erpm"    # reports a constant floor (measured 182 mech). bluegill <= 7f5cd61.
TELE_SINE_S4 = "s4-not-measuring"   # reports 0, the bidir-DShot not-spinning sentinel. bluegill f0b5339.


@dataclass(frozen=True)
class PlantParams:
    """Everything the plant needs. `drag` is the ONLY thing water changes.

    FROZEN on purpose: the transferability guarantee ("swapping the load touches one field") is only a
    property of the type if the type cannot be mutated behind `with_drag`'s back. Build variants with
    `with_drag` / `with_cross_up` / `with_brake` / `with_crossover_clamp`, all of which go through
    `dataclasses.replace`."""
    kv: float
    pole_pairs: int
    v_supply: float
    eff: float
    cross_up: float
    r_total: float
    l_phase: float
    j: float
    drag: Callable[[float], float]                 # |rpm| -> N*m, always opposes motion
    # The drag the `duty6` CALIBRATION was taken against. A property of the curve, not of this rotor:
    # it is what converts the loaded `duty6` back into an electrical duty, so it must stay put when
    # `drag` is swapped. Changing it would silently move the static curve.
    drag_ref: Callable[[float], float]
    # THE BRAKE CUT-OUT IS A BACK-EMF, NOT A SPEED, so it is stored as the voltage and the rpm is
    # derived per motor. Settled 2026-07-31 out-of-sample on a 930KV (see
    # measured.BRAKE_CUTOUT_BEMF_V): stored as an rpm it transferred WRONG — a plant handed
    # kv=930 kept braking to 670 rpm where the bench says ~1780, over-braking by 2.4x across a
    # 1100 rpm band. `brake_cutout_rpm` below keeps every existing reader working.
    brake_cutout_bemf_v: float
    sine_pullout_nm: float
    duty_slew_up_per_s: float
    duty_crossover_clamp: float
    tele_floor_mech: float
    tele_live_s: float
    sine_coherent_cmd_min: float
    sine_spread: dict = field(default_factory=dict)
    sine_reversals: dict = field(default_factory=dict)
    cut_release_s: dict = field(default_factory=dict)
    brake_engage_s: dict = field(default_factory=dict)
    # WHICH ESC FIRMWARE, because in forced sine S3 and S4 report different things (TELE_SINE_* above)
    # and every closed-loop conclusion below the handoff turns on which one the bench was running.
    # Defaults to S3 so that every trace and every conclusion taken before the S4 flash keeps meaning
    # what it meant; a caller comparing the two sets it explicitly.
    tele_sine: str = TELE_SINE_S3
    mech_config: str = ""
    name: str = ""

    @property
    def k_t(self) -> float:
        """K_T [N*m/A] == K_E [V*s/rad] in SI. Exact 60/(2*pi) so K_T*(rpm/KV) == K_T^2*w_rad in
        floating point — see measured.K_T."""
        return 60.0 / (2.0 * math.pi) / self.kv

    @property
    def kv_rad(self) -> float:
        return self.kv * RAD

    @property
    def brake_cutout_rpm(self) -> float:
        """Speed the zero-throttle brake cuts out at, FOR THIS MOTOR = BEMF * kv."""
        return self.brake_cutout_bemf_v * self.kv

    @property
    def no_load_top(self) -> float:
        """KV*V*eff — the speed at duty 1.0. Nothing may exceed it (C2)."""
        return self.kv * self.v_supply * self.eff


def _measured_drag(rpm):
    """The one measured drag torque, used for BOTH mechanical configurations in air.

    Not a shortcut — a measured result. Three independent lines say the propeller contributes nothing
    resolvable in air, so there is no second air drag law to have:
      * the free-coast decelerations: this table with J_BARE = 7.22e-6 and J_PROP = 8.8e-6 predicts
        5552 rpm/s bare (measured 5550) and 4966 rpm/s prop-on (measured 4860);
      * the bare-shaft static curve at cmd 555/570/600 matches a `duty6` that was CALIBRATED prop-on to
        <= 1 %, which it could not if the two rotors had materially different drag;
      * the table itself FALLS with speed, which no drag law permits — it is a lumped bearing/iron loss
        plus a K_T*I_dc bias, and those are propeller-independent.
    """
    _require_inventory("the default air drag law")
    return M.drag_at(rpm)


def preset(mech_config: str) -> PlantParams:
    """Build the PlantParams for one MECHANICAL CONFIGURATION.

    Every measured value is pulled through `M.value(name)` so that `measured.recorded_reads()` can tell
    the provenance test EXACTLY which parameters this function touched — a hand-maintained parallel list
    let `SINE_COHERENT_CMD_MIN` leak into the PROP preset undeclared."""
    _require_inventory("preset()")
    v = M.value
    if mech_config not in (M.PROP, M.BARE):
        raise KeyError(f"unknown mechanical configuration {mech_config!r}")
    bare = mech_config == M.BARE
    with M.recording(mech_config):
        return PlantParams(
            kv=v("KV"), pole_pairs=v("POLE_PAIRS"), eff=v("EFF_350KV"),
            v_supply=v("V_SUPPLY_BARE") if bare else v("V_SUPPLY_PROP"),
            cross_up=36.0 if bare else 34.0,
            r_total=v("R_TOTAL"), l_phase=v("L_PHASE_FITTED"),
            j=v("J_BARE_FITTED") if bare else v("J_PROP"),
            # read explicitly so the provenance recorder SEES it: `_measured_drag` closes over
            # DRAG_TABLE_PROP, and a read the recorder cannot see is a hole in the guard.
            drag=(v("DRAG_TABLE_PROP") and _measured_drag), drag_ref=_measured_drag,
            brake_cutout_bemf_v=v("BRAKE_CUTOUT_BEMF_V"), sine_pullout_nm=v("SINE_PULLOUT_NM"),
            duty_slew_up_per_s=v("DUTY_SLEW_UP_PER_S"),
            duty_crossover_clamp=v("DUTY_CROSSOVER_CLAMP"),
            tele_floor_mech=v("TELE_FLOOR_MECH"), tele_live_s=v("TELE_LIVE_S"),
            sine_coherent_cmd_min=v("SINE_COHERENT_CMD_MIN"),
            sine_spread=dict(v("SINE_SPREAD_TABLE_BARE")),
            sine_reversals=dict(v("SINE_REVERSALS_BARE")),
            cut_release_s=dict(v("CUT_RELEASE_S")), brake_engage_s=dict(v("BRAKE_ENGAGE_S")),
            mech_config=mech_config,
            name="BARE_SHAFT_350KV_20260730" if bare else "PROP_350KV_20260726")


def PROP_350KV_20260726() -> PlantParams:
    return preset(M.PROP)


def BARE_SHAFT_350KV_20260730() -> PlantParams:
    return preset(M.BARE)


# ---- the only sanctioned ways to build a variant (PlantParams is frozen) --------------------
def with_drag(params: PlantParams, drag: Callable[[float], float], name=None) -> PlantParams:
    """THE TRANSFERABILITY INTERFACE. Moving this plant to water changes the drag callable and
    NOTHING else — not `drag_ref` (which belongs to the `duty6` calibration), not the brake, not L,
    not J, not a single lag. There are no lags to change."""
    return replace(params, drag=drag, name=name or (params.name + "+drag"))


def with_cross_up(params: PlantParams, cross_up: float) -> PlantParams:
    """For the E.9 guard: run the same code against the other ESC config and watch it break."""
    return replace(params, cross_up=float(cross_up))


def with_brake(params: PlantParams, l_phase=None, j=None, brake_cutout_rpm=None) -> PlantParams:
    """For `plantid fit` only. `l_phase`, `j` and `brake_cutout_rpm` are a SET with the bare-shaft
    brake data; the provenance records say so."""
    return replace(params, l_phase=params.l_phase if l_phase is None else float(l_phase),
                   j=params.j if j is None else float(j),
                   brake_cutout_bemf_v=(params.brake_cutout_bemf_v if brake_cutout_rpm is None
                                        else float(brake_cutout_rpm) / params.kv))


def with_crossover_clamp(params: PlantParams, clamp: float) -> PlantParams:
    return replace(params, duty_crossover_clamp=float(clamp))


class MeasuredPlant:
    """The plant. `steady()`/`tele()`/`spread_at()` are the static & sensor layer; `step()` is the
    rotor layer; `p.drag` is the load layer."""

    def __init__(self, params: PlantParams, cut_mode: str = "disarm"):
        self.p = params
        self.motor = MotorModel(params.kv, params.pole_pairs, params.v_supply, params.eff,
                                name=params.name, cross_up=params.cross_up)
        self.cut_mode = cut_mode
        # Explicit Euler is only stable for dt well inside the electromechanical time constant
        # J*R/K_T^2 (~12 ms here). `step()` substeps internally rather than trusting the caller: the
        # explicit Euler diverges badly on a single oversized step, and `step(cmd, dt)` is the
        # public API — a future 20 ms-tick caller must not get a silently wrong answer.
        self._max_sub_dt = 0.05 * params.j * params.r_total / (params.k_t ** 2)
        self.reset()

    # ---------------------------------------------------------------- state
    def reset(self, rpm0: float = 0.0, t0: float = 0.0):
        self.rpm = float(rpm0)
        self.t = float(t0)
        # A plant reset AT a 6-step speed starts already handed off with the ramp complete: the
        # crossover slew models the TRANSITION, not the steady state. `handoff_mech()` is None when
        # this ESC config cannot hand off at all, and then no reset speed starts handed off.
        handoff = self.motor.handoff_mech()
        self._handed_off = handoff is not None and abs(rpm0) > handoff
        self._slew_duty = 1.0 if self._handed_off else 0.0
        self._zero_s = 0.0           # how long the command has been zero
        return self

    # ------------------------------------------------- layer 1: firmware/electrical
    def sine_field(self, cmd: float) -> float:
        """Mech rpm the ESC is FORCING the field to turn at. PP-governed, KV-independent."""
        return self.motor.rpm_sine(abs(cmd))

    def duty_6step(self, cmd: float) -> float:
        """EFFECTIVE 6-step duty (`rpm_measured/(KV*V)`) at this command and this ESC's cross_up.

        Clamped to [0, 1]: `duty6` is an affine fit to a modulator that cannot exceed full modulation,
        so past duty 1 it is extrapolating outside where it was measured. Unclamped, `sixstep_rpm` grew
        linearly without bound — +15 % over the no-load top at cmd 900 and +42 % at cmd 1000, inside
        this codebase's own `tmax=800` envelope, and disagreeing with the sibling
        `MotorModel.rpm_6step`, which has always clamped."""
        d = duty6(abs(cmd), self.p.cross_up)
        if d <= 0.0:
            return 0.0
        return d if d < 1.0 else 1.0

    def duty_ref_offset(self, cmd: float) -> float:
        """The duty `duty6` is MISSING because it was fitted on a LOADED rotor.

        `duty6` reports the duty implied by the speed the rotor settled at, so the drag torque that was
        holding it back is invisible in it. Recovering the electrical duty means adding back the duty
        that drag cost, evaluated at the speed the calibration settled at. See the module docstring."""
        w = self.sixstep_rpm(cmd)
        if w <= 0.0:
            return 0.0
        x = self.p.pole_pairs * w * RAD * self.p.l_phase
        base = self.p.k_t * self.p.r_total * self.p.v_supply * self.p.eff
        return abs(self.p.drag_ref(w)) * (self.p.r_total ** 2 + x * x) / base if base > 0 else 0.0

    def electrical_duty(self, cmd: float) -> float:
        """What the winding actually sees. Clamped at 1.0 (full modulation)."""
        d = self.duty_6step(cmd) + self.duty_ref_offset(cmd)
        return d if d < 1.0 else 1.0

    def sixstep_rpm(self, cmd: float) -> float:
        """KV*V*eff*duty6(cmd, cross_up) — the 6-step branch's measured equilibrium speed, clamped at
        the no-load top exactly as `MotorModel.rpm_6step` does."""
        rpm = self.p.no_load_top * self.duty_6step(cmd)
        return rpm if rpm < self.p.no_load_top else self.p.no_load_top

    def seam_duty(self, cmd: float) -> float:
        """The ELECTRICAL duty at which the 6-step branch would just match the forced field — i.e. the
        point the crossover slew has to climb to before the handoff can fire. In electrical units
        because `DUTY_CROSSOVER_CLAMP`, which the slew is compared against, is electrical."""
        base = self.p.no_load_top
        if base <= 0:
            return 0.0
        d6 = self.sine_field(cmd) / base
        w = self.sine_field(cmd)
        x = self.p.pole_pairs * w * RAD * self.p.l_phase
        b = self.p.k_t * self.p.r_total * self.p.v_supply * self.p.eff
        return d6 + (abs(self.p.drag_ref(w)) * (self.p.r_total ** 2 + x * x) / b if b > 0 else 0.0)

    def seam_cmd(self) -> float:
        """The command where the 6-step branch OVERTAKES the forced field. No hysteresis parameter
        exists: the seam is where two curves cross, so it is one number, not a band. Bisected rather
        than solved in closed form so it stays right if either branch's shape changes.

        NOT THE TRIGGER. This is a NECESSARY condition for a handoff to be STABLE — below it the 6-step
        branch would be slower than the field the rotor is already following — whereas what actually
        FIRES the handoff is the minimum-modulation duty floor in
        pico_esc.motor_model.MotorModel.handoff_cmd(). Folding the trigger into this was considered and
        rejected: on the 930KV the two branches cross at 471.4 / 503.6 / 728.9 at cross_up 34 / 36 / 50
        while the bench measured the handoff at 485.0 / 515.5 / 734.5, i.e. 6-14 counts apart in the
        same direction at every setting. The clincher is that they scale differently — this seam is
        where two SPEED curves meet, so it moves with KV and supply (482.0 / 514.9 / 745.3 for the
        350KV at 11.1 V), while the trigger is a DUTY and is the same command on every motor. Both are
        kept, and each docstring names the other."""
        lo, hi = 1.0, 1000.0
        if self.sixstep_rpm(hi) <= self.sine_field(hi):
            return float("nan")
        for _ in range(80):
            mid = 0.5 * (lo + hi)
            if self.sixstep_rpm(mid) > self.sine_field(mid):
                hi = mid
            else:
                lo = mid
        return 0.5 * (lo + hi)

    def steady(self, cmd: float):
        """(rpm, regime) at a held command. `max(sine field, 6-step)` — parameter-free, and the seam
        EMERGES where the branches cross instead of being a fitted threshold.

        Returns rpm None when the command is in the incoherent forced-sine band: the rotor there is
        not turning at a speed, it is oscillating (see `coherent`)."""
        s = 1.0 if cmd >= 0 else -1.0
        six, sine = self.sixstep_rpm(cmd), self.sine_field(cmd)
        if six > sine:
            return s * six, "6step"
        if not self.coherent(cmd):
            return None, "incoherent"
        return s * sine, "sine"

    def coherent(self, cmd: float) -> bool:
        """False where the rotor reverses inside a dwell, so a mean speed is not a speed."""
        if self.sixstep_rpm(cmd) > self.sine_field(cmd):
            return True
        return abs(cmd) >= self.p.sine_coherent_cmd_min

    def spread_at(self, cmd: float) -> float:
        """Measured peak-to-peak cogging dispersion at the nearest tabulated command [rpm]."""
        if not self.p.sine_spread:
            return 0.0
        k = min(self.p.sine_spread, key=lambda c: abs(c - abs(cmd)))
        return self.p.sine_spread[k]

    def reversals_at(self, cmd: float) -> int:
        if not self.p.sine_reversals:
            return 0
        k = min(self.p.sine_reversals, key=lambda c: abs(c - abs(cmd)))
        return self.p.sine_reversals[k]

    def tele(self, rpm: Optional[float] = None, t: Optional[float] = None) -> float:
        """What the ESC's telemetry REPORTS, not what the rotor does.

        Nothing at all (exactly 0) until the first frame arrives, `TELE_LIVE_S` after arming — that
        one is firmware-independent. After that it depends on the REGIME, because forced sine has no
        BEMF period to measure, so whatever comes out of the wire down there is the firmware's number
        and not the rotor's:
          * `_handed_off` — BEMF 6-step. The ESC measures, and both firmwares report the truth.
          * forced sine, S3 (`TELE_SINE_S3`) — a constant `TELE_FLOOR_MECH` = 182 mech rpm, measured at
            every command 60..450 in BOTH mechanical configurations. That lie is why a closed loop can
            wind up while the rotor is stalled: 182 sits above the controller's 50 rpm liveness floor.
          * forced sine, S4 (`TELE_SINE_S4`) — 0, the not-spinning sentinel. The gate rejects it and
            the loop stays open across the whole sine region, which is the contract it was written to.

        The S3 branch replaces an unconditional `max(TELE_FLOOR_MECH, |rpm|)` and is behaviour-identical
        wherever the bench can go: above the handoff the slowest reachable landing is the 350KV's 197
        mech rpm, already above the floor, so the old `max` never bit there either."""
        r = self.rpm if rpm is None else rpm
        tt = self.t if t is None else t
        if tt < self.p.tele_live_s:
            return 0.0
        return self.tele_reported(r)

    def tele_reported(self, rpm: Optional[float] = None) -> float:
        """The REGIME/FIRMWARE half of `tele()`, with the startup-silence window left to the caller.

        Split out for `MeasuredEscHost`, whose `tele_live` ablation owns that window and so cannot go
        through `tele()` without the silence coming back on. The two used to carry independent copies
        of the floor rule; one rule, one place."""
        r = self.rpm if rpm is None else rpm
        if self._bemf_locked(r):
            return abs(r)
        return self.p.tele_floor_mech if self.p.tele_sine == TELE_SINE_S3 else 0.0

    def _bemf_locked(self, r: float) -> bool:
        """Is the ESC measuring a real BEMF period, or imposing a forced one?

        During a stepped run `_handed_off` is the authority — it is the same state machine the real
        crossover runs. But `tele()` is also used as a PURE FUNCTION of an already-measured rpm (fixture
        replay in test_plant_replay), where no stepping has happened and `_handed_off` is still its
        post-reset False. There the only available evidence is the speed, so fall back to the handoff
        LANDING: below it a 6-step regime cannot be sustained.

        That fallback is ambiguous for a motor whose forced-sine range OVERLAPS its landing — the 350KV
        lands at 197 mech and sine reaches past it, so a 250 rpm sample is genuinely regime-ambiguous
        from speed alone. Harmless for replaying S3 traces (S3 reports 182 in sine and the landing sits
        above it, so the two rules disagree only inside a 15 rpm band, far under the fixtures' 25 rpm
        tolerance) but it must NOT be used to draw S4 conclusions: S4 reports 0 across that whole band.
        Step the plant for those."""
        if self._handed_off:
            return True
        h = self.motor.handoff_mech()
        return h is not None and abs(r) >= h

    # ------------------------------------------------- layer 2: rotor torques
    def tau_short_circuit(self, w_rotor_rpm: float) -> float:
        """The ESC's zero-throttle brake torque [N*m]:

            K_T^2 * w * R / (R^2 + (pp*w*L)^2)

        This is the `duty = 0` degenerate case of `tau_6step`, i.e. the winding shorted through the
        low-side FETs. Peaks at `w = R/(pp*L)` with magnitude `K_T^2*w_peak/(2R)` and falls off on BOTH
        sides, which is what makes the peaked-then-falling measured shape identifiable at all.

        Named for the rotor speed, not for "slip": the field is stationary when this fires, so slip IS
        rotor speed, and the 6-step branch never uses a slip law. Signed: positive speed brakes."""
        s = 1.0 if w_rotor_rpm >= 0 else -1.0
        w = abs(w_rotor_rpm) * RAD
        x = self.p.pole_pairs * w * self.p.l_phase
        # K_T^2*w == K_T*(rpm/KV) exactly, because K_T = 60/(2*pi*KV). Written the second way so this
        # is the SAME arithmetic as tau_6step at duty 0, not merely equal to it to within rounding.
        bemf = abs(w_rotor_rpm) / self.p.kv
        return s * self.p.k_t * bemf * self.p.r_total / (self.p.r_total ** 2 + x * x)

    def tau_6step(self, duty: float, rpm: float, direction: float = 1.0) -> float:
        """The voltage-source 6-step torque at an ELECTRICAL duty. At duty 0 this IS
        `-tau_short_circuit(rpm)`, for either direction.

        `direction` is the sign of the COMMAND, not of the rotor: the ESC drives the way it was told
        to, so a positive duty on a negative rotor is a plugging brake, not an accelerator. Getting
        this from sign(rpm) instead lets a rotor that has been dragged through zero run away."""
        s = 1.0 if direction >= 0 else -1.0
        w = abs(rpm) * RAD
        x = self.p.pole_pairs * w * self.p.l_phase
        # (s*rpm)/KV is the back-EMF in volts SEEN BY THE DRIVE; K_T*V/ohm has units N*m.
        drive = self.p.v_supply * self.p.eff * duty - (s * rpm) / self.p.kv
        return s * self.p.k_t * drive * self.p.r_total / (self.p.r_total ** 2 + x * x)

    def tau_drag(self, rpm: float) -> float:
        """Load torque, always opposing motion."""
        return (1.0 if rpm >= 0 else -1.0) * abs(self.p.drag(abs(rpm)))

    def brake_state(self, cmd: float) -> str:
        """`driving` | `released` | `braking` — the ESC's zero-throttle stop sequence.

        The rotor never freely coasts, but the brake does not bite when the command goes away either:
        |w| holds FLAT for `CUT_RELEASE_S` (the ESC is still driving), then decays on drag alone for
        `BRAKE_ENGAGE_S`, and only THEN does the brake bite. That fixed delay is what makes the
        deceleration peak appear at 2390 rpm bare-shaft and 1540 rpm prop-on — the peak never moved,
        the speed the rotor had reached by then did."""
        if abs(cmd) >= 1.0:
            return "driving"
        rel = self.p.cut_release_s.get(self.cut_mode, 0.0)
        eng = self.p.brake_engage_s.get(self.cut_mode, 0.0)
        if self._zero_s < rel:
            return "driving"
        if self._zero_s < rel + eng:
            return "released"
        if abs(self.rpm) < self.p.brake_cutout_rpm:
            # THE BRAKE CUTS OUT AT LOW SPEED, and this is measured, not assumed. Subtract the drag the
            # table predicts from the measured deceleration and the brake's contribution goes
            # ~60 krpm/s at 1900 rpm -> ~10 krpm/s at 600 -> ~4 krpm/s at 450 -> ZERO (within noise)
            # below ~250, in all four `disarm` decays. The short-circuit law's own low-speed branch
            # falls only as w, which at 450 rpm would still be 36 krpm/s — 7x the measured value.
            # A short circuit the ESC has stopped COMMUTATING (BEMF zero-crossing detection fails at
            # low speed) is stationary in the rotor frame, so its torque averages to ~0 over a
            # revolution. That is the one mechanism that explains a cut-out rather than a decay, and it
            # also explains the `deadman`/`zero_held` traces springing back through zero.
            # Gated as a STATE, not folded into tau_short_circuit(), so that function stays exactly
            # `tau_6step` at duty 0.
            return "released"
        return "braking"

    def tau_drive(self, cmd: float, rpm: Optional[float] = None) -> float:
        """Net ESC torque on the rotor [N*m]. One expression per regime, and the zero-command case is
        the 6-step expression with the duty removed — `tau_drive(0, w) == -tau_short_circuit(w)`."""
        r = self.rpm if rpm is None else rpm
        state = self.brake_state(cmd)
        if state == "braking":
            return -self.tau_short_circuit(r)
        if state == "released":
            return 0.0
        if state == "driving" and abs(cmd) < 1.0:
            # still driving through the release delay: hold the speed (the measured flat plateau)
            return self.tau_drag(r)
        if self._handed_off or self._slew_duty > 0.0:
            return self.tau_6step(self.applied_duty(cmd), r, 1.0 if cmd >= 0 else -1.0)
        # forced sine: synchronous lock with a pull-out saturation, NOT the short-circuit law
        s = 1.0 if cmd >= 0 else -1.0
        slip = r - s * self.sine_field(cmd)
        if slip == 0.0:
            return self.tau_drag(r)
        return -(1.0 if slip > 0 else -1.0) * self.p.sine_pullout_nm

    # ------------------------------------------------- layer 2: integrate
    def step(self, cmd: float, dt: float) -> float:
        """Advance one dt and return the new rpm.

        SUBSTEPS INTERNALLY. Explicit Euler on this plant is stable only for dt well inside
        `J*R/K_T^2` (~12 ms), and this is the public API: a 20 ms-tick caller would otherwise get an
        overshoot through zero, and `step(600, 10.0)` diverged to 28513 rpm."""
        if not math.isfinite(cmd):
            # A NaN command used to be absorbed into a WRONG behaviour rather than an error:
            # duty_6step(nan) -> 0.0, brake_state(nan) -> "driving", and `1.0 if cmd >= 0 else -1.0`
            # -> -1.0, so NaN silently became a large plugging command and step() returned a plausible
            # finite speed.
            raise ValueError(f"cmd must be finite, got {cmd!r}")
        dt = max(1e-9, float(dt))
        if not math.isfinite(dt):
            raise ValueError(f"dt must be finite, got {dt!r}")
        n = int(math.ceil(dt / self._max_sub_dt))
        for _ in range(max(1, n)):
            self._step_once(cmd, dt / max(1, n))
        return self.rpm

    def _step_once(self, cmd: float, dt: float) -> float:
        self._zero_s = self._zero_s + dt if abs(cmd) < 1.0 else 0.0
        self._update_handoff(cmd, dt)
        tau = self.tau_drive(cmd) - self.tau_drag(self.rpm)
        w = self.rpm * RAD + tau / self.p.j * dt
        new = w / RAD
        # forced-sine CAPTURE: once the pull-out torque can absorb the rest of the slip inside one
        # tick the rotor is in step, so land it exactly on the field instead of ringing about it.
        if self._slew_duty == 0.0 and abs(cmd) >= 1.0:
            s = 1.0 if cmd >= 0 else -1.0
            f = s * self.sine_field(cmd)
            band = self.p.sine_pullout_nm / self.p.j * dt / RAD
            if abs(new - f) <= band or (self.rpm - f) * (new - f) < 0:
                new = f
        if self.brake_state(cmd) == "braking" and abs(new) < 1.0:
            new = 0.0                     # the brake holds it; no coast through zero
        elif abs(cmd) < 1.0 and self.rpm * new < 0:
            new = 0.0
        self.rpm = new
        self.t += dt
        return self.rpm

    def settle(self, cmd: float, secs: float = 4.0) -> float:
        """Integrate a held command to convergence and return the rpm. This is what the static tests
        must compare against `steady()`, not `tau_6step` in isolation — the drag double-count was
        invisible precisely because nothing integrated."""
        n = max(1, int(secs / self._max_sub_dt))
        for _ in range(n):
            self._step_once(cmd, self._max_sub_dt)
        return self.rpm

    def applied_duty(self, cmd: float) -> float:
        """The ELECTRICAL duty the ESC is applying right now, which is the commanded one only once the
        crossover ramp has caught up with it (see `_update_handoff`)."""
        if self._handed_off:
            return self.electrical_duty(cmd)
        return min(self._slew_duty, self.p.duty_crossover_clamp)

    def _update_handoff(self, cmd: float, dt: float):
        """The crossover UP handoff is SLEW-limited, and only upward.

        Measured on the prop-on step 407 -> 680: the rotor sits at 360-415 rpm for ~0.26 s and only
        then jumps to 2200 in ~40 ms. That is neither the old 0.28 s dead time nor the old 0.18 s
        sine lag — it is the ESC ramping its 6-step duty demand at a finite rate while holding
        a low duty CLAMP, and releasing the clamp when the ramp catches the command.

        HONEST LIMIT, and it is a real one. Because `applied_duty` returns `min(_slew_duty, clamp)`,
        the ramp itself is only visible for the first `clamp/slew` ~= 80 ms and the CLAMP holds the
        remaining ~320 ms; the plateau ends at `t = electrical_duty(cmd)/slew`, which makes the handoff
        LATER for a LARGER command (0.18 s at cmd 560, 0.42 s at 680, 0.63 s at 800). For a BEMF-lock
        handoff that is backwards, and it is untested: only cmd 680 was ever measured. So this is
        behaviourally a COMMAND-SCALED DEAD TIME plus a duty clamp, and the command scaling is an
        UNTESTED PREDICTION, not a measured law. It differs from the old fixed 0.28 s deadtime in that
        the plateau LEVEL is now a modelled speed rather than zero — which is what the trace shows —
        but do not read more into it than that.

        Down is INSTANT (the down-step collapses from the very first sample), so the slew is
        up-direction only — asymmetric on purpose, and measured that way."""
        target = self.electrical_duty(cmd)
        if abs(cmd) < 1.0 or self.duty_6step(cmd) <= 0.0 or target <= self.seam_duty(cmd):
            self._handed_off = False        # below the seam the ESC is in forced sine, full stop
            self._slew_duty = 0.0
            return
        if not self.motor.handoff_possible():
            # This `sine_cross_up` gives a seed-tick window the firmware rejects, so the ESC never
            # leaves forced sine however long the command is held (measured at cross_up 30,
            # host/reports/sinesweep_20260731_013633.csv: no handoff on either leg up to cmd 500).
            self._handed_off = False
            self._slew_duty = 0.0
            return
        # THE UP TRIGGER IS THE DUTY FLOOR, NOT THE SEAM [fixed 2026-08-01]. Clearing the seam is a
        # NECESSARY condition — below it the 6-step branch would be slower than the field the rotor
        # already follows — but what FIRES the handoff is the minimum-modulation duty floor, which sits
        # 6-14 counts higher and in the same direction every time: on the 930KV the branches cross at
        # 471.4 / 503.6 / 728.9 at cross_up 34 / 36 / 50 while the bench measured 485.0 / 515.5 / 734.5
        # (motor_model.handoff_cmd, DUTY6_HANDOFF_FLOOR). Gating on the seam alone handed off up to 12
        # counts EARLY, which is the inconsistency test_closed_loop_matrix's "gap" cell flagged.
        #
        # Applied only on the UP transition. Once in 6-step the ESC stays there until its own down
        # hysteresis (`cross_dn`), so re-testing the up trigger on every tick would drop an already
        # handed-off plant back into sine anywhere between the seam and the floor.
        hoc = self.motor.handoff_cmd()
        if not self._handed_off and hoc is not None and abs(cmd) < hoc:
            self._slew_duty = 0.0
            return
        if self._handed_off or abs(self.rpm) >= self.sixstep_rpm(cmd):
            # Nothing left to ramp INTO: the rotor is already turning at the speed this command's
            # 6-step duty holds, so the crossover is complete on this tick.
            #
            # This used to gate on `motor.sixstep_floor_rpm()` and was justified as "the rotor is past
            # the LANDING, so there is no crossover GAP left to traverse". That justification died with
            # the eRPM handoff law: under the duty law the 350KV landing is 194.9 rpm at 11.1 V, and
            # the rotor sits at 387 rpm — 190 ABOVE it — throughout the measured 407 -> 680 plateau. So
            # the ramp is not a gap traversal at all; it is a duty ramp that runs on EVERY up-transition
            # into 6-step, and the shortcut has to compare against this command's own 6-step speed
            # (2240 rpm at cmd 680 prop-on) instead. Gating on the landing here would end the 0.26 s
            # dwell the step exists to measure (t95 0.159 s against a measured 0.442 s).
            #
            # `self._handed_off or` keeps it STICKY: once the ESC is in 6-step, a command that stays
            # above the seam raises the duty, it does not drop back to forced sine and re-ramp under
            # the crossover clamp. Without it every 6-step -> larger-6-step step would collapse the
            # rotor to the clamp speed first.
            self._handed_off = True
            self._slew_duty = target
            return
        if self._slew_duty >= target:
            self._handed_off = True
            self._slew_duty = target
            return
        self._handed_off = False
        self._slew_duty = min(target, self._slew_duty + self.p.duty_slew_up_per_s * dt)
        if self._slew_duty >= target:
            self._handed_off = True

    # ------------------------------------------------- reporting
    def regime(self, cmd: float) -> str:
        return self.steady(cmd)[1]

    def in_gap(self, rpm: float) -> bool:
        """The nominal no-steady-state crossover gap, `[handoff_mech, sixstep_floor_rpm]`.

        The modelled gap now AGREES with the open-loop measurement instead of apologising to it. Under
        the old eRPM handoff law this band was 201..676 rpm on the bare shaft, and the measured
        open-loop sweep holds commands 520..545 stably at 205..453 rpm — inside it, which the old
        docstring had to excuse. The duty law puts the bare-shaft gap at 184.0..194.9 rpm (11.1 V,
        cross_up 36), so every one of those measured points is now OUTSIDE it and no excuse is needed.
        What survives unchanged is what the flag is FOR: it marks where the CLOSED loop must not close
        (no steady state to hold), not where no speed exists."""
        return self.motor.in_gap(rpm)

    def mech_time_constant_ms(self) -> float:
        return M.mech_time_constant_ms(self.p.j, self.p.r_total)

    def summary(self) -> str:
        p = self.p
        if self.motor.handoff_possible():
            handoff = (f"  handoff/land  : {self.motor.handoff_mech():.0f} / "
                       f"{self.motor.sixstep_floor_rpm():.0f} rpm\n")
        else:
            # deliberately not naming WHICH refusal — MotorModel.summary() does that, and this line
            # would otherwise go stale the next time a refusal condition is added.
            handoff = (f"  handoff/land  : none — cross_up {p.cross_up:g} has no reachable handoff "
                       f"(see MotorModel.handoff_possible); forced sine at every command\n")
        return (f"MeasuredPlant {p.name} [{p.mech_config}]\n"
                f"  KV={p.kv:g} PP={p.pole_pairs} V={p.v_supply:g} eff={p.eff:g} "
                f"cross_up={p.cross_up:g}\n"
                f"  seam cmd      : {self.seam_cmd():.1f}  (6-step overtakes the forced field)\n"
                + handoff +
                f"  R={p.r_total:g} ohm  L={p.l_phase*1e3:.3f} mH  J={p.j:.3e} kg*m^2  "
                f"J*R/K_T^2={self.mech_time_constant_ms():.1f} ms\n"
                f"  brake         : {M.brake_peak_torque(p.r_total, p.l_phase)*1e3:.1f} mN*m peak at "
                f"{M.brake_peak_rpm(p.r_total, p.l_phase):.0f} rpm "
                f"(SCALE UNVALIDATED — see docs/measured-plant.md)\n"
                f"  drag / ref    : {p.drag.__name__} / {p.drag_ref.__name__} "
                f"({p.drag(2000)*1e3:.2f} mN*m at 2000 rpm)")


COUNTS_PER_REV = 4096.0


class MeasuredEscHost:
    """An `EscLink`-shaped adapter that answers `thrust`/`enc`/`encv`/`tele` from a `MeasuredPlant`.

    A NEW class on purpose. `pico_esc/sim.py`'s `SimEncEscHost` is pinned byte-identically by
    `test_regression_baseline.py` through its RNG DRAW ORDER — adding or removing one random draw
    shifts every subsequent sample and breaks the golden CSV — and a measured plant model has no
    business inside the dry-run protocol mock anyway. This one draws no randomness at all.

    Speaks the same `.cmd(line, timeout) -> [str]` string protocol, so it is used as
    `ESC(link=MeasuredEscHost(plant, clock))` and the whole existing `ESC`/`PosDrive`/
    `VelocityController` stack runs on top of it unmodified. Drive it with a `SimClock`: the plant is
    advanced to `clock.now()` on every command, exactly as the real one is advanced by wall time.

    `tele` reports what the ESC would REPORT, floor and startup silence included — which is the whole
    point of putting the sensor model in the loop instead of feeding the controller the truth. The two
    lies are SEPARATE knobs (`tele_floor`, `tele_live`) so an A/B changes one thing at a time.

    `tele_floor=False` is an ABLATION, not a firmware: it asks "what if the sensor were perfect?" and
    reports the true `|rpm|` in sine. That is NOT what S4 does — S4 reports 0 down there, because a
    perfect sensor is not available to an ESC that has no BEMF period to measure. Select a firmware
    with `plant.p.tele_sine` (TELE_SINE_*); keep `tele_floor` for the counterfactual.
    """

    def __init__(self, plant: MeasuredPlant, clock, tele_floor: bool = True,
                 tele_live: bool = True):
        self.plant = plant
        self.clock = clock
        self.tele_floor = bool(tele_floor)   # False = ABLATION: report |rpm| however low it is
        self.tele_live = bool(tele_live)     # False = telemetry is live from t=0 (no startup silence)
        self.thrust = 0
        self.armed = False
        self.pos_counts = 0.0
        self.t0 = clock.now()
        self.last_t = clock.now()

    def _advance(self, now):
        dt = now - self.last_t
        self.last_t = now
        if dt <= 0:
            return
        # sub-step so the ~12 ms electromechanical time constant is integrated, not sampled. (step()
        # would substep anyway; this keeps the position integral at the same resolution.)
        n = max(1, int(dt / 1e-3))
        for _ in range(n):
            self.plant.step(self.thrust, dt / n)
            self.pos_counts += self.plant.rpm / 60.0 * (dt / n) * COUNTS_PER_REV

    def cmd(self, line, timeout=30.0):
        line = line.strip()
        head, _, arg = line.partition(" ")
        if head == "thrust":
            self._advance(self.clock.now())
            self.thrust = int(arg.split()[-1]) if arg.split() else 0
            return []
        if head == "encv":
            self._advance(self.clock.now())
            return [f"encv|{int(round(self.pos_counts))}|{self.plant.rpm:.2f}|8|1"]
        if head == "enc":
            self._advance(self.clock.now())
            raw = int(round(self.pos_counts)) % int(COUNTS_PER_REV)
            deg = raw * 360.0 / COUNTS_PER_REV
            return [f"enc|{raw}|{raw}|{deg:.1f}|1|0|0|64|1800"]
        if head == "tele":
            self._advance(self.clock.now())
            t = self.clock.now() - self.t0
            if self.tele_live and t < self.plant.p.tele_live_s:
                rpm = 0.0
            elif self.tele_floor:
                rpm = self.plant.tele_reported()  # regime+firmware aware (S3 floor vs S4 sentinel)
            else:
                rpm = abs(self.plant.rpm)        # ablation: a perfect sensor, which no firmware has
            # rpm|volts|amps|tempC|stress — magnitude, as a bidir ESC reports; 31 C flat because no
            # fixture has thermal data (see the not-modelled list).
            return [f"tele|{rpm:.0f}|{self.plant.p.v_supply:.2f}|0|31|0"]
        if head == "arm":
            self.armed = True
            self.plant.reset(0.0, 0.0)
            self.t0 = self.last_t = self.clock.now()
            return ["armed|bidir"]
        if head in ("disarm", "run", "disconnect"):
            self.armed = False
            self.thrust = 0
            return []
        if head == "scan":
            return ["esc|1|1|1|E8B2|8|#A_H_30#|Measured|0.21"]
        return []

    def close(self):
        pass
