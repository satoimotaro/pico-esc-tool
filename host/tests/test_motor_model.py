# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests for the parametric MotorModel (KV+PP+V -> curve/regimes/config)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pico_esc.motor_model import MotorModel


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
    m = MotorModel(350, 7, 11.1)
    assert m.regime(100) == "sine"          # below handoff
    assert m.regime(m.handoff_mech() - 1) == "sine"
    assert m.regime(m.sixstep_floor_rpm() + 50) == "6step"
    # something strictly between handoff and the landing is the gap
    mid = (m.handoff_mech() + m.sixstep_floor_rpm()) / 2
    assert m.regime(mid) == "gap" and m.in_gap(mid)


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
