# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests for the parametric MotorModel (KV+PP+V -> curve/regimes/config)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest

from pico_esc.config import up_erpm_handoffable
from pico_esc.motor_model import (CMD_FULL_SCALE, DUTY6_HANDOFF_FLOOR, MotorModel, cmd_for_duty6,
                                 _max_emittable_cross_up, _max_handoff_cross_up,
                                 _min_handoff_cross_up)


def test_no_load_top_scales_with_kv_and_v():
    assert MotorModel(350, 7, 11.1).no_load_top() == 350 * 11.1
    assert MotorModel(1000, 7, 11.1).no_load_top() == 1000 * 11.1


def test_sixstep_curve_scales_with_kv():
    a = MotorModel(350, 7, 11.1)
    b = MotorModel(1000, 7, 11.1)
    # same universal shape -> ratio equals the KV ratio at any 6-step cmd
    assert abs(b.rpm_6step(600) / a.rpm_6step(600) - 1000 / 350) < 1e-6


def test_sine_is_pp_governed_not_kv():
    # forced-sine full-scale depends on PP only, not KV
    assert MotorModel(350, 7, 11.1).sine_fullscale_mech() == MotorModel(999, 7, 5).sine_fullscale_mech()
    assert MotorModel(350, 7, 11.1).sine_fullscale_mech() > MotorModel(350, 14, 11.1).sine_fullscale_mech()


def test_regime_boundaries():
    """Pinned against the MEASURED edges, not against whatever the model happens to say.

    This test used to read every boundary back out of the model, so it passed identically with the
    handoff 47 counts too high and the landing 3.6x too fast — which is exactly why it did not catch
    the bug. The numbers below are the 350KV at 11.1 V / cross_up 34: handoff 172.9 mech (the rotor is
    still on the forced field, host/reports/sinesweep_20260731_014004.csv brackets the command at
    485.0 +- 1.0) and a landing of 194.9 rpm = KV*V*eff*DUTY6_HANDOFF_FLOOR."""
    m = MotorModel(350, 7, 11.1)
    assert m.handoff_mech() == pytest.approx(172.86, abs=0.05)
    assert m.sixstep_floor_rpm() == pytest.approx(194.90, abs=0.05)
    assert m.regime(100) == "sine"          # below handoff
    assert m.regime(172.0) == "sine"        # just under the measured handoff
    assert m.regime(m.handoff_mech() - 1) == "sine"
    assert m.regime(m.sixstep_floor_rpm() + 50) == "6step"
    assert m.regime(2000) == "6step"
    # the gap is NARROW under the duty law — 172.9..194.9 rpm, 22 rpm wide, not the 190..707 the
    # seam-derived law claimed. 185 rpm is inside it; 250 rpm, which the old law called the gap, is not.
    assert m.regime(185.0) == "gap" and m.in_gap(185.0)
    assert not m.in_gap(250.0) and m.regime(250.0) == "6step"
    mid = (m.handoff_mech() + m.sixstep_floor_rpm()) / 2
    assert m.regime(mid) == "gap" and m.in_gap(mid)


# --------------------------------------------------------------- the 2026-07-31 up-handoff law
# Every number here is from the 930KV encoder-truth bench (11.2 V, 1-count sine sweeps) or from the
# 350KV fold sweep, and the provenance is at DUTY6_HANDOFF_FLOOR in pico_esc/motor_model.py. The
# fixtures themselves are per-installation assets (host/reports/ is gitignored), so the law is pinned
# inline and this file needs no CSV.
MEASURED_HANDOFF_CMD = {34: (485.0, 1.0), 36: (515.5, 0.5), 50: (734.5, 0.5)}
PREDICTED_HANDOFF_CMD = {34: 484.21, 36: 515.41, 50: 733.81}


@pytest.mark.parametrize("cross_up", (34, 36, 50))
def test_the_handoff_command_is_the_duty_floor(cross_up):
    """handoff_cmd == cmd_for_duty6(DUTY6_HANDOFF_FLOOR, cross_up), and it matches the bench.

    The old law inverted the CONFIGURED SEAM through the forced-sine curve: 533.5 / 562.7 / 781.6 at
    cross_up 34 / 36 / 50, ~47 counts high at every setting and on every motor."""
    m = MotorModel(350, 7, 11.2, cross_up=cross_up)
    assert m.handoff_cmd() == pytest.approx(PREDICTED_HANDOFF_CMD[cross_up], abs=0.1)
    assert m.handoff_cmd() == pytest.approx(
        cmd_for_duty6(DUTY6_HANDOFF_FLOOR, cross_up=float(cross_up)), abs=1e-9)


def test_the_predictions_sit_inside_the_measured_brackets_and_the_residuals_are_named():
    """Residuals -0.79 / -0.09 / -0.69 counts at cross_up 34 / 36 / 50: a CONSTANT, not a trend.

    The measured up-leg brackets are (484, 486] / (515, 516] / (734, 735] — the last forced-sine
    command and the first handed-off one, from sinesweep_20260731_014004 / _013410 / _013236.csv.

    THE SHAPE OF THE RESIDUAL IS THE EVIDENCE, and this test exists to hold it flat. Until the
    2026-07-31 re-fit of DUTY6_CROSSUP_K these read -0.79 / +0.49 / +3.90 — a slope of +0.29 counts
    per cross_up unit across a 16-unit span. A wrong FLOOR moves all three residuals together; only a
    wrong cross_up SLOPE fans them out like that, so the trend was diagnostic of K and not of the
    duty-floor law. K was then measured three independent ways (the duty_sweep pair at cross_up 36 and
    50, the sine sweeps' 6-step branches, and the handoff commands themselves: 15.60 / 15.63 / 15.59
    counts per unit against the model's 15.89) and re-fitted to 0.04134. The trend collapsed to +0.006
    counts/unit — 0.10 counts over the whole span — leaving a constant ~-0.5 count offset that is
    below the 1-count resolution of the sweeps the brackets came from.

    Nothing was absorbed to achieve this: DUTY6_HANDOFF_FLOOR is read off the cross_up 34 leg, where
    the K term is identically zero, so it did not move when K did — and the cross_up 34 residual is
    the same -0.79 it always was. That is the control."""
    resid = {}
    for cross_up, (meas, tol) in MEASURED_HANDOFF_CMD.items():
        m = MotorModel(350, 7, 11.2, cross_up=cross_up)
        resid[cross_up] = m.handoff_cmd() - meas
    assert resid[34] == pytest.approx(-0.79, abs=0.05)
    assert resid[36] == pytest.approx(-0.09, abs=0.05)
    assert resid[50] == pytest.approx(-0.69, abs=0.05)
    # THE TREND IS GONE. Slope across the 16-unit span, which is what a wrong K shows up as: it was
    # (3.90 - -0.79)/16 = +0.293 counts/unit, and is now (-0.69 - -0.79)/16 = +0.006.
    assert abs(resid[50] - resid[34]) / 16.0 == pytest.approx(0.006, abs=0.002)
    assert max(resid.values()) - min(resid.values()) <= 0.75      # was 4.69
    # ...and what is left is an OFFSET: all three on the same side of the measurement, with a mean
    # pinned to a hundredth of a count. (Each residual is already pinned individually to +-0.05 above,
    # and the spread to 0.75, so a per-element band here would only restate them.)
    mean = sum(resid.values()) / 3.0
    assert mean == pytest.approx(-0.52, abs=0.05)
    assert all(r < 0.0 for r in resid.values())
    # Stated against the bracket EDGES too. 34 and 36 are plainly INSIDE the bracket the bench
    # measured; 50 now sits 0.19 counts BELOW its lower edge instead of 3.40 above its upper one —
    # i.e. inside one command count of a bracket the sweep resolved to one command.
    # (The superseded 526.7 reduction of the floor put cross_up 36 at 516.14, i.e. 0.14 counts OUTSIDE
    # (515, 516]. Correcting the row filter moved it in — see DUTY6_HANDOFF_FLOOR.)
    assert 484.0 < MotorModel(350, 7, 11.2, cross_up=34).handoff_cmd() <= 486.0
    assert 515.0 < MotorModel(350, 7, 11.2, cross_up=36).handoff_cmd() <= 516.0
    assert 734.0 - MotorModel(350, 7, 11.2, cross_up=50).handoff_cmd() == pytest.approx(0.19, abs=0.05)


def test_a_fixed_speed_floor_is_refuted_by_the_350kv():
    """The falsifiable alternative: "the ESC hands off once the 6-step branch reaches 522.5 rpm".

    That is the other reading of the same locked down-leg, and it is KV-DEPENDENT. On the 350KV at
    11.1 V / cross_up 36 it predicts cmd 547.2, while that motor's own measured departure bracket is
    (515, 520] (host/tests/data/sinesweep_fold_noprop_20260730_033417.csv: cmd 515 rides the field with
    122 rpm of spread, cmd 520 is off it at 205.7 rpm with 25). The duty law predicts 515.4 — inside.
    Both motors handing off at the same COMMAND is the whole discrimination."""
    m = MotorModel(350, 7, 11.1, cross_up=36)
    assert 515.0 < m.handoff_cmd() <= 520.0
    fixed_speed_cmd = m.cmd_for_6step(MotorModel(930, 7, 11.2, cross_up=36).sixstep_floor_rpm())
    assert fixed_speed_cmd == pytest.approx(547.2, abs=0.2)
    assert fixed_speed_cmd > 520.0                      # outside the bracket: refuted


def test_the_handoff_command_is_kv_and_pp_independent_but_the_landing_is_not():
    """The trigger is a DUTY, so no motor term survives into the command; the landing is a 6-step rpm."""
    a = MotorModel(350, 7, 11.2, cross_up=36)
    b = MotorModel(930, 12, 22.2, cross_up=36)
    assert a.handoff_cmd() == pytest.approx(b.handoff_cmd(), abs=1e-9)
    assert b.sixstep_floor_rpm() / a.sixstep_floor_rpm() == pytest.approx(
        (930 * 22.2) / (350 * 11.2), rel=1e-9)


@pytest.mark.parametrize("kv,v,rpm", [(930, 11.2, 522.53), (350, 11.1, 194.90), (350, 11.2, 196.65)])
def test_the_landing_is_kv_v_times_the_duty_floor(kv, v, rpm):
    m = MotorModel(kv, 7, v)
    assert m.sixstep_floor_rpm() == pytest.approx(kv * v * m.eff * DUTY6_HANDOFF_FLOOR, abs=1e-9)
    assert m.sixstep_floor_rpm() == pytest.approx(rpm, abs=0.05)


def test_the_landing_does_not_move_with_cross_up():
    """INVERTED ASSERTION, and this is the measurement that justifies inverting it.

    The old law made the landing FALL as cross_up rose (707 -> 685 -> 640 rpm at 34 -> 36 -> 50), which
    was an artifact of deriving the trigger from the seam. The trigger is a fixed DUTY, so the landing
    is the SAME speed at every cross_up — and that is what the bench says: the down-leg lock reads
    523.44 / 522.56 / 522.19 rpm at cross_up 34 / 36 / 50 (sinesweep_20260731_014004 / _013410 /
    _013236.csv), a 1.2 rpm spread over a 16-unit span and 250 command counts."""
    landings = [MotorModel(350, 7, 11.2, cross_up=cu).sixstep_floor_rpm() for cu in (34, 36, 50)]
    # equality to the float round-trip epsilon of cmd_for_duty6 -> duty6 (2e-13 rpm), against a
    # measured spread of 1.2 rpm — not the old strict inequality, which pointed the wrong way.
    assert max(landings) - min(landings) < 1e-9
    assert all(x == pytest.approx(landings[0], abs=1e-9) for x in landings)
    # ...while the handoff COMMAND still rises with cross_up (the same duty costs more command).
    cmds = [MotorModel(350, 7, 11.2, cross_up=cu).handoff_cmd() for cu in (34, 36, 50)]
    assert cmds[0] < cmds[1] < cmds[2]


def test_handoff_mech_is_the_rotor_on_the_forced_field():
    """At the handoff the rotor is still synchronous with the forced field, so handoff_mech is the SINE
    curve at handoff_cmd — 172.9 / 184.0 / 262.0 mech at pp = 7. It is no longer cross_up_erpm()/pp."""
    for cross_up, mech in ((34, 172.86), (36, 184.00), (50, 261.97)):
        m = MotorModel(350, 7, 11.2, cross_up=cross_up)
        assert m.handoff_mech() == pytest.approx(m.rpm_sine(m.handoff_cmd()), abs=1e-9)
        assert m.handoff_mech() == pytest.approx(mech, abs=0.05)
        assert m.cmd_for_sine(m.handoff_mech()) == pytest.approx(m.handoff_cmd(), abs=1e-9)


# --------------------------------------------------------------- the firmware's REFUSAL
@pytest.mark.parametrize("cross_up,ticks,ok", [(30, 34, False), (33, 31, False), (34, 30, True),
                                               (36, 28, True), (50, 20, True)])
def test_handoff_possible_is_the_raw_bytes_tick_window(cross_up, ticks, ok):
    """The firmware does not clamp a too-low cross_up, it REFUSES to hand off (measured at cross_up 30,
    host/reports/sinesweep_20260731_013633.csv: no handoff on either leg up to cmd 500).

    Keyed on the RAW byte * 39.0625, never on cross_up_erpm(), which floors a low threshold up to the
    firmware minimum and would report a confident handoff for a config that has none."""
    m = MotorModel(350, 7, 11.2, cross_up=cross_up)
    assert int(4 * 10000.0 / (cross_up * 39.0625)) == ticks
    assert m.handoff_possible() is ok
    # the floored seam would have said yes to every one of them
    assert m.cross_up_erpm() >= 1333.0


def test_a_refused_handoff_cannot_become_a_usable_command():
    """None, not NaN: `nan <= x` is False, so a NaN handoff would route regime()/thrust_for() silently
    into the 6-step branch and hand a NaN command to the caller. None raises TypeError at the first
    arithmetic, and every derived path here is guarded so it never gets that far."""
    m = MotorModel(350, 7, 11.2, cross_up=30)
    assert m.handoff_cmd() is None
    assert m.handoff_mech() is None
    assert m.sixstep_floor_rpm() is None
    assert m.in_gap(200.0) is False and m.in_gap(5000.0) is False
    assert m.regime(200.0) == "sine" and m.regime(5000.0) == "sine"
    assert m.thrust_for(200.0) == pytest.approx(m.cmd_for_sine(200.0))
    assert m.thrust_for(-200.0) == pytest.approx(-m.cmd_for_sine(200.0))
    with pytest.raises(ValueError) as e:
        m.to_profile()
    assert "30" in str(e.value) and "tick" in str(e.value)
    s = m.summary()
    assert "none" in s and "refuse" in s
    # ...and the accepted byte next door still works, so the guard is a predicate and not a blanket.
    ok = MotorModel(350, 7, 11.2, cross_up=34)
    assert ok.handoff_cmd() is not None and ok.to_profile().points


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), 0.0, -36.0, 1e-300, 1e300])
def test_a_nonsense_cross_up_refuses_rather_than_raising(bad):
    """A refusal predicate must have TWO outcomes, not three, and it must agree with the C++ mirror.

    The window is written as a bounds test on the seam eRPM rather than as `int(40000/up_erpm)`
    precisely so these cases come out False: `int(inf)` is an OverflowError and `int(nan)` a
    ValueError, and either would have escaped `handoff_possible()` as an exception on a path whose
    whole job is to answer one bit. vel::handoffPossibleAt() in lib/vel_control/motor_model.h is the
    same expression (every comparison against a NaN is false, so a NaN refuses), and
    test_sense_gate.cpp pins `handoffCmdFor(NAN) == 0.0f` on the other side."""
    assert up_erpm_handoffable(bad) is False
    m = MotorModel(350, 7, 11.2, cross_up=bad)
    assert m.handoff_possible() is False
    # ...and every derived path stays sane rather than propagating the nonsense
    assert m.handoff_cmd() is None and m.handoff_mech() is None and m.sixstep_floor_rpm() is None
    assert m.in_gap(500.0) is False and m.regime(500.0) == "sine"
    assert isinstance(m._handoff_refusal(), str)
    with pytest.raises(ValueError):
        m.to_profile()
    m.summary()                                   # renders without a traceback


def test_a_handoff_past_the_command_range_is_a_refusal_too():
    """The OTHER half of `handoff_possible`, and the one that keeps the C++ sentinel honest.

    cross_up shifts the trigger by DUTY6_CROSSUP_K/DUTY6_A = 15.6 counts per unit, so the handoff
    command walks off the end of the protocol: 999.0 at cross_up 67, 1014.6 at 68, 1513.8 at 100.
    Nothing can be SENT that hands off up there, so it is a refusal in exactly the same operational
    sense as a rejected tick window — and folding it into the predicate is also what stops the C++
    `HANDOFF_CMD_NONE` (1001.0f) from aliasing a REAL handoff command, which it would have done from
    cross_up 68 up. Pinned at both bytes so a future move of the duty map is caught here — and it has
    already moved once: the 2026-07-31 K re-fit walked this boundary from 66/67 to 67/68, which is
    what this pin is for."""
    lo, hi = MotorModel(350, 7, 11.2, cross_up=67), MotorModel(350, 7, 11.2, cross_up=68)
    assert lo.handoff_possible() and lo.handoff_cmd() == pytest.approx(999.01, abs=0.05)
    assert lo.handoff_cmd() <= CMD_FULL_SCALE
    assert not hi.handoff_possible() and hi.handoff_cmd() is None
    assert cmd_for_duty6(DUTY6_HANDOFF_FLOOR, cross_up=68.0) == pytest.approx(1014.61, abs=0.05)
    assert not MotorModel(350, 7, 11.2, cross_up=100).handoff_possible()
    # the derived bounds in the diagnostics are the same two bytes, not literals (they are functions
    # of DUTY6_CROSSUP_K, and the upper one moved 66 -> 67 when it was re-fitted)
    assert (_min_handoff_cross_up(), _max_handoff_cross_up()) == (34, 67)
    # the tick window alone would have accepted every one of these
    assert up_erpm_handoffable(68 * 39.0625) and up_erpm_handoffable(100 * 39.0625)
    # ...and the diagnostics must name the RIGHT refusal — the two have opposite fixes (raise the
    # byte vs lower it), so a message that always blames the tick window is worse than none.
    assert "unreachable" in hi.summary() and "1015" in hi.summary()
    assert "seed window" in MotorModel(350, 7, 11.2, cross_up=30).summary()
    with pytest.raises(ValueError) as e:
        hi.to_profile()
    assert "1015" in str(e.value) and "seed window" not in str(e.value)


def test_to_profile_refuses_a_handoff_above_cmd_hi_instead_of_stepping_backwards():
    """cross_up 50 — the setting this bench run itself used — hands off at 733.8, above the default
    cmd_hi=700. Unguarded, `to_profile()` emitted 8 points ending (736, 444.8), (739, 522.5): the
    6-step loop steps BACKWARDS, and the curve then inverts to the same command for 600, 1500 and
    3000 rpm alike. That is the silent FF mis-scale the refusal guard exists to prevent, so it raises
    for the same reason — and it is fixed by sampling high enough, not by clamping."""
    m = MotorModel(930, 7, 11.2, cross_up=50)
    assert m.handoff_possible() and m.handoff_cmd() > 700.0      # NOT the refusal case
    with pytest.raises(ValueError) as e:
        m.to_profile()
    assert "733" in str(e.value) and "700" in str(e.value)
    prof = m.to_profile(cmd_hi=900)                               # sample past the handoff: fine
    thr = [p[0] for p in prof.points]
    rpm = [p[1] for p in prof.points]
    assert len(prof.points) > 2
    assert thr == sorted(thr) and len(set(thr)) == len(thr)
    assert rpm == sorted(rpm) and len(set(rpm)) == len(rpm)
    # and the boundary: cross_up 47 fits under cmd_hi=700, 48 does not
    assert MotorModel(930, 7, 11.2, cross_up=47).to_profile().points
    with pytest.raises(ValueError):
        MotorModel(930, 7, 11.2, cross_up=48).to_profile()


def test_thrust_for_roundtrips_in_6step():
    m = MotorModel(350, 7, 11.1)
    for rpm in (800, 1500, 2400):
        cmd = m.thrust_for(rpm)
        assert abs(m.rpm_6step(cmd) - rpm) < 1.0


def test_self_cal_fits_eff():
    m = MotorModel(350, 7, 11.1)
    # synthesise samples from a plant that is 5% slower (eff 0.95): mech = 0.95*ideal
    samples = []
    for cmd in (550, 620, 690):
        ideal = 350 * 11.1 * (0.00265 * cmd - 1.233)
        mech = 0.95 * ideal
        samples.append((cmd, mech * 7))     # eRPM = mech*pp
    eff = m.self_cal_erpm(samples)
    assert abs(eff - 0.95) < 0.01


def test_esc_config_is_kv_independent_recipe():
    c1 = MotorModel(350, 7, 11.1).esc_config()
    c2 = MotorModel(1000, 14, 22.2).esc_config()
    # the working crossover recipe is eRPM/firmware-based -> same bytes regardless of motor
    assert c1["sine_cross_up"] == c2["sine_cross_up"] == 34
    assert c1["sine_cross_dn"] == 255
    assert c1["low_rpm_power_protection"] == 0


def test_to_profile_monotone_and_loadable():
    from pico_esc.velocity import SpeedProfile
    prof = MotorModel(350, 7, 11.1, name="t").to_profile()
    thr = [p[0] for p in prof.points]
    rpm = [p[1] for p in prof.points]
    assert thr == sorted(thr) and len(set(thr)) == len(thr)      # strictly increasing thrust
    assert rpm == sorted(rpm)                                     # monotone rpm
    assert isinstance(prof, SpeedProfile)


def test_a_parametric_profile_declares_the_sine_branch_it_actually_contains():
    """`to_profile()` used to omit `capabilities` entirely, and that is not neutral.

    `SpeedProfile._capability` defaults every flag to False when the block is absent, so a
    parametric profile advertised `sine_lowspeed: false` while its own first five points ARE the
    forced-sine branch — a curve contradicting its own metadata, and the metadata is what gets
    written to YAML and shipped to another machine. Measured profiles have always declared it
    explicitly (host/profiles/vel_930kv_12n14p_6step.yaml says true, vel_930kv_sineoff.yaml false).

    `down_catch` must stay False: the DOWN handoff is not modelled, and where it HAS been measured
    it does not work — the 930KV never fell back at all, because its 6-step floor sits 2.8x above
    the cross_dn threshold. Claiming it from a parametric model would be an invention.
    """
    m = MotorModel(350, 7, 11.1, cross_up=36)
    p = m.to_profile()
    assert p.sine_lowspeed is True
    assert p.down_catch is False
    # ...and the claim is true of the points: the ones below the handoff command are the sine branch
    below = [(c, r) for c, r in p.points if c < m.handoff_cmd()]
    assert below, "the profile should carry forced-sine points below the handoff command"
    for c, r in below:
        assert r == pytest.approx(m.rpm_sine(c), rel=0.02), f"cmd {c} is not on the sine branch"


def test_the_emittable_band_is_narrower_than_the_handoff_band_and_both_are_derived():
    """Two DIFFERENT upper bounds, and conflating them shipped a wrong recovery message twice.

    `_max_handoff_cross_up()` (67) is where the handoff command stops being a command at all
    (> CMD_FULL_SCALE). `_max_emittable_cross_up(cmd_hi)` (47 at the default 700) is where the
    handoff outruns the top of the curve being sampled. Bytes in 48..67 hand off perfectly well and
    still cannot be given a parametric FF curve, so quoting 34..67 as the remedy for a CURVE failure
    sends the operator into the other raise. Both are derived from DUTY6_CROSSUP_K — and the
    2026-07-31 re-fit moved ONE of them (66 -> 67) while leaving the other at 47, which is the
    sharpest possible demonstration that they are not interchangeable.
    """
    assert _min_handoff_cross_up() == 34
    assert _max_handoff_cross_up() == 67
    assert _max_emittable_cross_up(700) == 47
    assert _max_emittable_cross_up(700) < _max_handoff_cross_up()
    # the boundary is exactly where a curve stops fitting, checked against to_profile itself
    for cu in (46, 47):
        MotorModel(350, 7, 11.1, cross_up=cu).to_profile()          # must not raise
    for cu in (48, 67):
        assert MotorModel(350, 7, 11.1, cross_up=cu).handoff_possible() is True
        with pytest.raises(ValueError, match="cmd_hi"):
            MotorModel(350, 7, 11.1, cross_up=cu).to_profile()
    # raising cmd_hi is the documented remedy, and it works
    MotorModel(350, 7, 11.1, cross_up=50).to_profile(cmd_hi=900)
