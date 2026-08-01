# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""WHICH CLOSED-LOOP CONFIGURATIONS ACTUALLY WORK — the matrix, on the MEASURED plant.

Nobody knew. `velctl` has run against `pico_esc.sim`'s toy plant since it was written, and the toy
plant implements the SPEC: `test_sim_crossover.py::test_sim_reports_stale_tele_in_sine_but_live_in_6step`
pins that it reports NOTHING below the seam. `MeasuredPlant.tele()` implements the REALITY — a floored
182 mech rpm that never goes stale — and until 4e77eba nothing but `sim_obs` was wired to it. So every
closed-loop test in this suite has been rehearsing an ESC that does not exist, and the two bugs found
on the bench on 2026-08-01 were both invisible here.

THE SPEC IS NOW BECOMING TRUE, which is why every cell runs on BOTH firmwares (see `FIRMWARES`).
bluegill f0b5339 replaces the 182 with the not-spinning sentinel, so after that flash the reality
column and the spec column converge. Until the flash is bench-verified they are two different ESCs and
both have to be represented, or every trace taken before it silently changes meaning. Note what the S4
column already shows: not one new xfail. Running the whole forced-sine region OPEN, on feed-forward
alone, costs nothing the 10 %/25 rpm tolerance can see — the closed loop down there was never buying
anything, because there was never a measurement down there to close on.

WHAT THE BENCH KNOWS (930KV and 350KV bare shaft, 2026-08-01):
  * sine OFF, 6-step only .......... no stalls (5/5), but 47 % steady-state error, because the FF
                                     curve is calibrated for sine_mode = 2
  * sine ON, target above the landing ~50 % stall at 4000 on the 930KV (38 % of its no-load top)
  * sine ON, target inside the sine region ... UNTESTED on hardware
  * sine ON, target inside the gap ........... UNTESTED on hardware
  * the 350KV fails at a different target: 2400, i.e. 62 % of ITS no-load top

WHAT THIS FILE IS, AND IS NOT. It is the model's answer for the same grid, so that a fix can be
SHOWN to move a cell. It is not a substitute for the bench: two of the bench facts above are not in
the model at all, and saying so is part of the deliverable —
  * the 47 % sine-off error is a firmware effect (`sine_mode` also changes `cross_rescale_duty`, so
    the 6-step duty map itself moves). `NoSineplant` below models only the MISSING SINE BRANCH, not
    the moved duty map, so the sine-off cells here are optimistic at high targets by construction;
  * the ~50 % stall is a rotor that fails to catch the forced field. `MeasuredPlant` has no
    pull-out dynamics (`sine_pullout_nm` is carried as data, not simulated), so the stall is
    INJECTED here by `StuckPlant` rather than emerging. That injection is the point: the operator's
    requirement is that a control-level blow-up is impossible REGARDLESS of what the sensor says,
    which is a statement about the controller, not about how often the rotor really cogs.

Cells that fail are marked `xfail` with the measured reason, so this file is honest today and turns
green as fixes land rather than being quietly re-tuned.
"""
import os
import sys
from dataclasses import replace

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

pytest.importorskip("pico_esc.measured",
                    reason="the measured plant inventory is per-installation and not distributed "
                           "(see plant._require_inventory); this matrix needs it")

from pico_esc import measured as M                                    # noqa: E402
from pico_esc.control import DT                                       # noqa: E402
from pico_esc.esc import ESC                                          # noqa: E402
from pico_esc.link import SimClock                                    # noqa: E402
from pico_esc.motor_model import MotorModel                           # noqa: E402
from pico_esc.plant import (MeasuredEscHost, MeasuredPlant, TELE_SINE_S3,   # noqa: E402
                            TELE_SINE_S4, preset)
from pico_esc.velocity import VelocityController                      # noqa: E402


# --------------------------------------------------------------------------- the plants
class NoSinePlant(MeasuredPlant):
    """`sine_mode = 0`: the ESC has no forced-sine branch at all, so below the 6-step floor there is
    no torque and the rotor does not turn.

    HONEST LIMIT (see the module docstring): the real `sine_mode = 0` ALSO moves the 6-step duty map,
    because `cross_rescale_duty` is a sine-mode feature — the bench measured 47 % steady-state error
    against a curve calibrated at `sine_mode = 2`. That is not modelled here; only the missing sine
    branch is. So these cells are optimistic wherever the 6-step branch carries the motion."""

    def sine_field(self, cmd):
        return 0.0


class StuckPlant(MeasuredPlant):
    """FAULT INJECTION: the rotor never catches the forced field (it cogs in place).

    The telemetry model is untouched, which is the whole point — `MeasuredEscHost` keeps reporting
    the measured 182 mech rpm floor at every command, so the controller's liveness gate sees a live
    frame on every tick while the shaft is motionless. This is the 2026-08-01 bench failure with the
    probability taken out of it."""

    def step(self, cmd, dt):
        self.t += dt
        return 0.0


BARE_350 = "350KV"
BARE_930 = "930KV"

# THE FIRMWARE AXIS. Every cell below is run on BOTH, because "which closed-loop configurations work"
# has a different answer depending on what the ESC says while it is forcing the field, and the answer
# for the firmware on the bench today is not the answer for the firmware about to be flashed:
#   S3 (bluegill <= 7f5cd61) reports a constant 182 mech rpm in sine — above TELE_MIN_MECH_RPM = 50,
#     so the loop believes it and `track_min_frac` is the only thing standing between it and windup.
#   S4 (bluegill f0b5339)    reports the not-spinning sentinel — the liveness gate rejects it and the
#     loop runs open across the whole sine region, which is the contract vel_control.h was written to.
# Keeping both here is the point: a trace taken before the flash must keep comparing against S3, and
# the S4 column says what to expect after it, in one file rather than by editing a constant.
FIRMWARES = (TELE_SINE_S3, TELE_SINE_S4)
FW_ID = {TELE_SINE_S3: "s3-lies", TELE_SINE_S4: "s4-honest"}


def _params(motor, tele_sine=TELE_SINE_S3):
    """The bare-shaft inventory, optionally re-KV'd and on a chosen firmware.

    `MeasuredPlant`'s presets are 350KV only. `kv` is the one field a 930KV needs: `k_t`,
    `no_load_top` and `brake_cutout_rpm` are all derived properties of it (the last one only since
    0b54afa — stored as an rpm it transferred WRONG, over-braking a 930KV by 2.4x)."""
    p = preset(M.BARE)
    if motor != BARE_350:
        p = replace(p, kv=930.0, name="BARE_930KV_SYNTH")
    return replace(p, tele_sine=tele_sine)


def _bands(motor):
    """The four target bands, DERIVED from the model's own edges so they cannot drift apart from it.

    The two `high` targets are the bench's own failing fractions of no-load top: 62 % on the 350KV
    (2400) and 38 % on the 930KV (4000). That they are such different fractions is itself a finding —
    whatever the 930KV hits at 4000, it is not a fraction-of-top limit."""
    p = _params(motor)
    m = MeasuredPlant(p).motor
    handoff, landing = m.handoff_mech(), m.sixstep_floor_rpm()
    return {
        "below-seam":    0.80 * handoff,                       # forced sine, under the trigger
        # NOTE the two modules disagree about this band, and the matrix makes it visible rather than
        # papering over it: `MotorModel.in_gap()` says 350 rpm has no steady state on the 930KV
        # (handoff 184, landing 522), while `MeasuredPlant.steady(509)` returns (345.5, '6step') and
        # the cell below tracks it. The plant's `_update_handoff` still fires on the SEAM
        # (seam_cmd 503) where the motor model's trigger is the duty floor (handoff_cmd 515.4) —
        # a known open inconsistency, not a bug in this file. Recorded here because a "gap" cell that
        # tracks perfectly is exactly the sort of result that should not pass unremarked.
        "gap":           0.5 * (handoff + landing),            # the nominal no-steady-state band
        "above-landing": 1.25 * landing,                       # just into genuine 6-step
        "high":          (0.62 if motor == BARE_350 else 0.38) * p.no_load_top,
    }


# --------------------------------------------------------------------------- the harness
# Per-motor closed-loop gains. These are the BENCH's, not DEFAULT_GAINS (which are sim-tuned and
# ~30x too hot for a real plant): the 2026-08-01 investigation ran kp 0.03 / ki 0.12 / trim_max 80,
# and trim_max 80 is half of the 230 counts of authority that produced the 872-vs-641 command.
GAINS = dict(kp=0.03, ki=0.12, trim_max=80.0)
SECS = 6.0
SETTLE_FROM = 0.8 * SECS


def _run(motor, target, *, plant_cls=MeasuredPlant, profile_cross_up=None, secs=SECS,
         tele_sine=TELE_SINE_S3, **kw):
    """One closed-loop run. Returns the OUTCOMES, never the internals — a cell must be judged on
    what the operator would see."""
    p = _params(motor, tele_sine)
    plant = plant_cls(p, cut_mode="disarm")
    clock = SimClock()
    esc = ESC(MeasuredEscHost(plant, clock), 1, tmax=1000, clock=clock)
    # The FF curve. `profile_cross_up` builds it for a DIFFERENT cross_up than the plant runs, which
    # is the "sustained model error" case (a wrong byte is a documented 13-55 % static error).
    mm = MotorModel(p.kv, p.pole_pairs, p.v_supply, p.eff,
                    cross_up=p.cross_up if profile_cross_up is None else profile_cross_up)
    prof = mm.to_profile(cmd_hi=700.0)
    gains = dict(GAINS); gains.update(kw)
    vc = VelocityController(esc, prof, tele_period=DT, max_secs=secs, max_temp=0.0,
                            stall_secs=1e9, over_speed_rpm=1e9, slew_rpm_s=2000.0, **gains)
    esc.arm(bidir=True)
    vc.set_speed(target)
    rows = []
    vc.run(clock, on_row=lambda t, tg, sp, thr, tp, er, tr, trm: rows.append((t, plant.rpm, thr)))
    tail = [r for r in rows if r[0] >= SETTLE_FROM]
    ff = prof.thrust_for(target)
    return {
        "ff": ff,
        "rpm_ss": sum(r[1] for r in tail) / len(tail),
        "cmd_peak": max(abs(r[2]) for r in rows),
        "cmd_end": rows[-1][2],
        "authority": vc._w,
        "excursion": max(abs(r[2]) for r in rows) - ff,     # command over the FF for the TARGET
        "target": target,
    }


def _reached(o, tol_frac=0.10, tol_abs=25.0):
    """Did it get there? 10 %, or 25 rpm, whichever is LOOSER.

    The absolute floor is not a fudge: the low bands sit in the forced-sine region, where the bench's
    own `sine_spread` table measures 100-250 rpm PEAK-TO-PEAK cogging dispersion at a held command.
    Demanding better than +-25 rpm of the MEAN there would be asking the model for a precision the
    hardware does not have — and would make this matrix a test of the tolerance rather than of the
    configuration."""
    return abs(o["rpm_ss"] - o["target"]) <= max(tol_frac * o["target"], tol_abs)


def _bounded(o):
    """THE STRUCTURAL INVARIANT: the command never exceeds the feed-forward for the target by more
    than the feedback authority the operator actually granted.

    The curve is monotone, so FF(setpoint) <= FF(target) throughout a slew — every count above
    FF(target) is feedback. In this Python loop the PI is the only feedback path, so the budget is
    trim_max; the firmware also has the disturbance observer, and bounding the pair of them is what
    `vel::VelocityController::fb_max` exists for."""
    return o["excursion"] <= GAINS["trim_max"] + 1.0


# =========================================================================== A. the grid
MATRIX = [(motor, band, sine, fw)
          for motor in (BARE_350, BARE_930)
          for band in ("below-seam", "gap", "above-landing", "high")
          for sine in (True, False)
          for fw in FIRMWARES]

# Cells the model says do not work, with the reason. `strict=True`: if one starts passing, this file
# has to be re-read rather than silently going green.
KNOWN_BAD = {
    (BARE_350, "below-seam", False, TELE_SINE_S3):
        "sine_mode=0 has no torque below the 6-step floor, so the rotor never turns at all — and "
        "`tele` still reports its 182 rpm floor, so nothing in the loop can tell",
    (BARE_350, "below-seam", False, TELE_SINE_S4):
        "same missing torque; on S4 the loop at least KNOWS it is blind (tele reads the not-spinning "
        "sentinel) instead of being told 182, but knowing does not make the rotor turn",
    (BARE_930, "below-seam", False, TELE_SINE_S3):
        "same as the 350KV: with no forced-sine branch the sub-landing region is unreachable, and "
        "the 930KV's landing is 522 rpm, so the unreachable band is 3x wider",
    (BARE_930, "below-seam", False, TELE_SINE_S4):
        "same unreachable 3x-wider band on the honest firmware",
}


@pytest.mark.parametrize("motor,band,sine,fw", MATRIX,
                         ids=[f"{m}-{b}-sine{'on' if s else 'off'}-{FW_ID[f]}"
                              for m, b, s, f in MATRIX])
def test_the_closed_loop_configuration_matrix(motor, band, sine, fw, request):
    """Does this configuration reach its target, and does it stay inside its authority budget?

    Asserted on OUTCOMES, not on tuned numbers: `_reached` is 10 % or 25 rpm, whichever is looser,
    and `_bounded` is the structural invariant that no feedback path may exceed what was granted.
    Run on BOTH firmwares (see FIRMWARES) so the flash cannot silently move a cell."""
    if (motor, band, sine, fw) in KNOWN_BAD:
        request.node.add_marker(
            pytest.mark.xfail(strict=True, reason=KNOWN_BAD[(motor, band, sine, fw)]))
    target = _bands(motor)[band]
    o = _run(motor, target, plant_cls=MeasuredPlant if sine else NoSinePlant, tele_sine=fw)
    where = f"{motor}/{band}/sine={sine}/{FW_ID[fw]}"
    assert _bounded(o), (f"{where}: command peaked {o['cmd_peak']:.0f} against a "
                         f"feed-forward of {o['ff']:.0f} — {o['excursion']:.0f} counts of feedback "
                         f"where only {GAINS['trim_max']:.0f} was granted")
    assert _reached(o), (f"{where}: settled at {o['rpm_ss']:.0f} rpm against a "
                         f"target of {target:.0f} ({100 * (o['rpm_ss'] - target) / target:+.0f} %)")


# =========================================================================== B. the fault
# The row the whole exercise is about. A rotor that never catches, behind telemetry that never admits
# it. The requirement is NOT that the loop tracks — it cannot, there is nothing turning — it is that
# it degrades instead of blowing up.
@pytest.mark.parametrize("fw", FIRMWARES, ids=[FW_ID[f] for f in FIRMWARES])
@pytest.mark.parametrize("motor", (BARE_350, BARE_930))
def test_a_stuck_rotor_degrades_to_feed_forward_on_either_firmware(motor, fw):
    """DEGRADED TRACKING IS ACCEPTABLE; A CONTROL-LEVEL BLOW-UP IS NOT.

    The same fault, seen through both firmwares, because they fail the loop by DIFFERENT routes and
    only one of them is a route the flash removes:

      * S3 — `TELE_MIN_MECH_RPM = 50` sits BELOW the 182 rpm floor, so the liveness gate can never
        separate "the rotor is at 182" from "the ESC is lying about a rotor that is not turning".
        Before `track_min_frac` the PI took full authority against a fabricated error and parked at
        +trim_max for ever; on the firmware the observer stacked another +dob_max on top, which is the
        measured 872-against-641. Here `track_min_frac` is the ONLY thing withdrawing authority.
      * S4 — the sentinel reads 0, the liveness gate rejects it, and authority never rises in the
        first place. The corroboration gate is not exercised at all.

    The flash therefore does not make `track_min_frac` redundant: it removes the one lie we happen to
    know about, on the one firmware we control. Any other ESC — stock Bluejay, a different fork — is
    free to report whatever it likes down there, and the S3 column is what that looks like.

    What must hold on BOTH: the authority is withdrawn, and the loop does not reach the target —
    which it should not, because nothing is turning."""
    target = _bands(motor)["high"]
    o = _run(motor, target, plant_cls=StuckPlant, tele_sine=fw)
    assert o["rpm_ss"] == 0.0                                  # the fault really is injected
    assert o["authority"] < 0.01, (f"authority stayed at {o['authority']:.2f} on a motionless rotor "
                                   f"({FW_ID[fw]})")
    assert _bounded(o), (f"{FW_ID[fw]}: command peaked {o['cmd_peak']:.0f} against a feed-forward of "
                         f"{o['ff']:.0f} on a rotor that never moved")
    assert o["excursion"] <= 1.0, (f"{motor}: command {o['cmd_peak']:.0f} vs feed-forward "
                                   f"{o['ff']:.0f} — the loop is still pushing against a lie")
    assert not _reached(o)                                     # honest: it does NOT track


@pytest.mark.parametrize("motor", (BARE_350, BARE_930))
def test_the_same_fault_slams_the_command_with_the_corroboration_gate_disabled(motor):
    """The guard above is only worth what its counterfactual is worth, so run the identical scenario
    with `track_min_frac = 0` — the pre-2026-08-01 liveness-only behaviour — and watch it saturate.

    This is the delete-experiment as a test: it fails if the gate ever stops being what removes the
    slam (if some other change starts masking it, this cell goes green and says so)."""
    target = _bands(motor)["high"]
    o = _run(motor, target, plant_cls=StuckPlant, track_min_frac=0.0)
    assert o["authority"] > 0.99, "liveness alone should still hand out full authority"
    assert o["excursion"] >= GAINS["trim_max"] - 1.0, (
        f"expected the PI to park at +trim_max on the fake error, got {o['excursion']:.0f}")


# =========================================================================== C. model error
# The operator's other named case: the loop must not blow up when the MODEL is wrong, not just when
# the sensor is. A wrong `sine_cross_up` is the documented example (13-55 % on the static curve).
@pytest.mark.parametrize("motor,profile_cross_up", [(BARE_350, 34.0), (BARE_930, 34.0)])
def test_a_feed_forward_built_for_the_wrong_cross_up_stays_inside_its_authority(motor, profile_cross_up):
    """The plant runs cross_up 36; the curve was built at 34, so the feed-forward under-commands by a
    fixed duty offset (31 counts at the 930KV's high target). The PI is supposed to absorb exactly
    this — and it does, inside its granted authority, with the corroboration gate on.

    That is the load-bearing half of "do not just lower the clamps": the fix must leave the feedback
    its legitimate job."""
    target = _bands(motor)["high"]
    o = _run(motor, target, profile_cross_up=profile_cross_up)
    assert _bounded(o)
    assert o["authority"] > 0.99, "a merely mis-calibrated FF must NOT withdraw authority"
    assert _reached(o), (f"{motor}: settled at {o['rpm_ss']:.0f} against {target:.0f} — the PI did "
                         f"not absorb a wrong-cross_up feed-forward")


# =========================================================================== D. the invariant
def test_the_corroboration_gate_is_inert_on_every_healthy_cell():
    """The gate must be a no-op wherever the plant is really being driven, or it is just a lower
    clamp wearing a disguise. Every cell of the grid, with and without it, to the command count."""
    for motor, band, sine, fw in MATRIX:
        if (motor, band, sine, fw) in KNOWN_BAD:
            continue
        target = _bands(motor)[band]
        cls = MeasuredPlant if sine else NoSinePlant
        on = _run(motor, target, plant_cls=cls, tele_sine=fw)
        off = _run(motor, target, plant_cls=cls, tele_sine=fw, track_min_frac=0.0)
        assert abs(on["cmd_peak"] - off["cmd_peak"]) <= 1.0, (
            f"{motor}/{band}/sine={sine}/{FW_ID[fw]}: the gate moved the command "
            f"{off['cmd_peak']:.0f} -> {on['cmd_peak']:.0f} on a HEALTHY plant")
        assert abs(on["rpm_ss"] - off["rpm_ss"]) <= 1.0
