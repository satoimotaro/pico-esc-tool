# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""SpeedProfile inversion + VelocityController slew/seam tests (velctl v1).

Feed-forward inversion must be exact at the calibration points, clamped at both endpoints,
odd-symmetric for reverse, and reject a non-monotonic curve. The controller's setpoint slew
must ramp a step (so the first command is never a full-scale jump), and the eRPM-PI seam must
be a genuine no-op in v1 (the encoder never feeds the command).
"""
import pytest

from pico_esc import ESC, SpeedProfile, VelocityController
from pico_esc.link import SimClock
from pico_esc.sim import SimEncEscHost

POINTS = [(0, 0), (100, 50), (200, 120), (300, 200), (400, 320)]


def _profile(**kw):
    return SpeedProfile(POINTS, motor="t", **kw)


def test_inverse_exact_at_cal_points():
    p = _profile()
    for thr, rpm in POINTS:
        assert p.thrust_for(rpm) == thr


def test_inverse_interpolates_between_points():
    p = _profile()
    assert p.thrust_for(25) == 50            # midway on the (0,0)-(100,50) segment
    assert p.thrust_for(160) == 250          # midway on (200,120)-(300,200)


def test_inverse_clamps_both_endpoints():
    p = _profile()
    assert p.thrust_for(-999) == -400        # magnitude clamps to the last point
    assert p.thrust_for(9999) == 400
    assert p.thrust_for(0) == 0


def test_inverse_odd_symmetric():
    p = _profile()
    for rpm in (10, 50, 133, 320, 500):
        assert p.thrust_for(-rpm) == -p.thrust_for(rpm)


def test_inverse_monotonic_nondecreasing():
    p = _profile()
    prev = -1
    for r in range(0, 330, 5):
        t = p.thrust_for(r)
        assert t >= prev
        prev = t


def test_rejects_non_monotonic_rpm():
    with pytest.raises(ValueError):
        SpeedProfile([(0, 0), (100, 80), (200, 60)])       # rpm decreases


def test_rejects_non_increasing_thrust():
    with pytest.raises(ValueError):
        SpeedProfile([(0, 0), (100, 50), (100, 90)])       # thrust repeats


def test_round_trip_yaml(tmp_path):
    p = _profile(pole_pairs=7, source="unit",
                 crossover={"up_erpm": 2100.0, "dn_erpm": 1600.0, "bytes": [54, 195]})
    path = tmp_path / "p.yaml"
    p.save(str(path), header="test header")
    q = SpeedProfile.load(str(path))
    assert q.points == p.points
    assert q.crossover == p.crossover
    assert (q.motor, q.pole_pairs, q.source) == (p.motor, p.pole_pairs, p.source)


def test_round_trip_yaml_with_regime_tags(tmp_path):
    regimes = ["sine", "sine", "sine", "line", "line"]
    p = SpeedProfile(POINTS, motor="t", regimes=regimes)
    path = tmp_path / "p.yaml"
    p.save(str(path))
    q = SpeedProfile.load(str(path))
    assert q.points == p.points
    assert q.regimes == regimes                  # per-point seam tag survives the round-trip


def test_regimes_length_must_match_points():
    with pytest.raises(ValueError):
        SpeedProfile(POINTS, regimes=["sine"])   # wrong length


def test_regimes_track_point_sort_order():
    # points passed out of thrust order -> regimes must be reordered in lockstep
    pts = [(200, 120), (0, 0), (100, 50)]
    regs = ["line", "sine", "mid"]
    p = SpeedProfile(pts, regimes=regs)
    assert p.points == [(0.0, 0.0), (100.0, 50.0), (200.0, 120.0)]
    assert p.regimes == ["sine", "mid", "line"]


def test_slew_ramps_setpoint_not_a_step():
    clock = SimClock()
    esc = ESC(SimEncEscHost(clock, seed=1234), 1, clock=clock)
    esc.arm()
    ctrl = VelocityController(esc, _profile(), slew_rpm_s=100.0, max_secs=0.5, max_temp=0)
    ctrl.set_speed(320)
    firsts = []
    ctrl.run(clock, on_row=lambda t, tg, sp, thr, tp, e: firsts.append((sp, thr)))
    esc.disarm()
    sp0, thr0 = firsts[0]
    assert sp0 < 20                          # first setpoint is a small slew step, not 320
    assert abs(thr0) < 100                    # so the first command is gentle, not full-scale
    assert firsts[-1][0] > sp0               # setpoint climbs toward the target


def test_closed_loop_trim_is_zero_seam():
    clock = SimClock()
    esc = ESC(SimEncEscHost(clock, seed=1234), 1, clock=clock)
    ctrl = VelocityController(esc, _profile())
    assert ctrl._closed_loop_trim(100.0, 95.0) == 0.0      # v1: encoder never feeds the command


def test_regime_seam_classifies_by_crossover():
    p = _profile(crossover={"up_erpm": 2100.0, "dn_erpm": 1600.0, "bytes": [54, 195]})
    ctrl = VelocityController(ESC(object(), 1), p)
    assert ctrl.regime(100) == "sine"        # 100*7 = 700 eRPM < 2100
    assert ctrl.regime(400) == "line"        # 400*7 = 2800 eRPM >= 2100
