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
from pico_esc.drive import Aborted
from pico_esc.link import SimClock
from pico_esc.sim import SimEncEscHost
from pico_esc.types import Telem


class _FakeESC:
    """Minimal ESC stand-in for run() unit tests: clamps thrust to +-tmax, returns a fixed
    telemetry frame, and has no encoder. Lets us drive the closed loop against a controlled plant."""

    def __init__(self, *, tmax=1000, tele=None):
        self.tmax = tmax
        self._tele = tele
        self.last_sent = 0

    def thrust(self, x):
        self.last_sent = max(-self.tmax, min(self.tmax, int(x)))
        return self.last_sent

    def telemetry(self):
        return self._tele

    def temperature(self):
        return None

    def encoder(self):
        return None

    def encoder_velocity(self):
        return None

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
    ctrl.run(clock, on_row=lambda t, tg, sp, thr, tp, e, tr, trim: firsts.append((sp, thr)))
    esc.disarm()
    sp0, thr0 = firsts[0]
    assert sp0 < 20                          # first setpoint is a small slew step, not 320
    assert abs(thr0) < 100                    # so the first command is gentle, not full-scale
    assert firsts[-1][0] > sp0               # setpoint climbs toward the target


def test_regime_seam_classifies_by_crossover():
    p = _profile(crossover={"up_erpm": 2100.0, "dn_erpm": 1600.0, "bytes": [54, 195]})
    ctrl = VelocityController(ESC(object(), 1), p)
    assert ctrl.regime(100) == "sine"        # 100*7 = 700 eRPM < 2100
    assert ctrl.regime(400) == "line"        # 400*7 = 2800 eRPM >= 2100


# ---------------------------------------------------------------------------
# Phase A1 closed loop: PI trim on tele mech RPM, faded by telemetry liveness.
# ---------------------------------------------------------------------------
def _ctrl(**kw):
    c = VelocityController(ESC(object(), 1), _profile(**kw.pop("profile_kw", {})), **kw)
    c._live = True                               # PI-math tests assume a LIVE frame (integrator armed)
    return c


def test_measure_uses_tele_as_mechanical_no_pole_division():
    # INVARIANT: tele.rpm is ALREADY mechanical — a Telem(rpm=700) must measure 700, NOT 100 (700/7).
    tel = Telem(700, 0.0, 0, 25, 0)
    assert VelocityController._measure(tel, 320.0) == 700.0       # sign follows the +setpoint
    assert VelocityController._measure(tel, -320.0) == -700.0     # magnitude + commanded sign
    assert VelocityController._measure(Telem(30, 0.0, 0, 25, 0), 320.0) is None   # below the live floor
    assert VelocityController._measure(None, 320.0) is None


def test_pi_proportional_sign():
    c = _ctrl(ki=0.0)                          # pure P for a clean sign check
    c._tele_mech = 250.0                        # measured below the 300 setpoint -> push up (+)
    assert c._closed_loop_trim(300.0, 0.02) > 0
    c._i = 0.0
    c._tele_mech = 350.0                        # measured above -> pull down (-)
    assert c._closed_loop_trim(300.0, 0.02) < 0


def test_pi_integral_accumulates():
    c = _ctrl(kp=0.0, ki=2.0, trim_max=1000.0)  # pure I, wide clamp
    c._tele_mech = 200.0                         # constant +100 error
    t1 = c._closed_loop_trim(300.0, 0.02)
    t2 = c._closed_loop_trim(300.0, 0.02)
    assert t2 > t1 > 0                           # integral keeps building on a sustained error


def test_pi_trim_clamped_to_trim_max():
    c = _ctrl(kp=1.0, ki=5.0, trim_max=200.0)
    c._tele_mech = 0.0                           # huge +300 error
    for _ in range(50):
        trim = c._closed_loop_trim(300.0, 0.02)
    assert trim == 200.0                         # saturates at the clamp, no runaway


def test_pi_antiwindup_reversal_crosses_zero_quickly():
    # Saturate the trim positive on a sustained error, then reverse the sign of the error; with
    # back-calculation the trim must cross zero within a few PI updates (no wound-up lag).
    c = _ctrl(kp=0.5, ki=4.0, trim_max=200.0)
    c._tele_mech = 0.0
    for _ in range(60):
        trim = c._closed_loop_trim(300.0, 0.02)
    assert trim == 200.0                         # saturated positive
    c._tele_mech = 600.0                         # error now -300 (reversed)
    crossed = None
    for k in range(1, 8):
        trim = c._closed_loop_trim(300.0, 0.02)
        if trim <= 0:
            crossed = k
            break
    assert crossed is not None and crossed <= 5  # unwinds within a few updates


def test_measure_signs_a_magnitude_by_last_command_at_zero_setpoint():
    # setpoint==0 is sign-ambiguous: the tele MAGNITUDE takes the LAST-commanded sign, not +.
    tel = Telem(700, 0.0, 0, 25, 0)
    assert VelocityController._measure(tel, 0.0, fallback_sign=-1.0) == -700.0
    assert VelocityController._measure(tel, 0.0, fallback_sign=1.0) == 700.0


def test_stall_aborts_when_line_setpoint_stays_stale():
    # Crossover profile with NO regime tags -> _line_floor == up_erpm/pole_pairs == 300. A setpoint
    # of 320 (6-step-reachable) whose telemetry NEVER goes live must trip the stall abort.
    p = _profile(crossover={"up_erpm": 2100.0, "dn_erpm": 1600.0, "bytes": [54, 195]})
    assert p.regimes is None and abs(p.thrust_for(320)) >= 1        # curve is smooth (no gap tags)
    esc = _FakeESC(tmax=1000, tele=Telem(0, 0.0, 0, 25, 0))         # rpm 0 -> forever stale
    ctrl = VelocityController(esc, p, slew_rpm_s=1e6, max_temp=0, max_secs=5.0, stall_secs=0.3)
    ctrl.set_speed(320)
    with pytest.raises(Aborted, match="stall"):
        ctrl.run(SimClock())


def test_no_stall_for_gap_setpoint_below_reachable_line_floor():
    # A profile that TAGS a crossover gap (last sine rpm 200, first line rpm 700): a setpoint of 320
    # lands in the unreachable gap, so its command runs in sine (stale tele) — that is pure FF, NOT a
    # stall. _line_floor == 700 (the min line-tagged rpm), so 320 < 700 never arms the stall guard.
    pts = [(0, 0), (100, 50), (200, 100), (300, 200), (400, 700), (500, 780)]
    regimes = ["sine", "sine", "sine", "sine", "line", "line"]
    p = SpeedProfile(pts, motor="gap", regimes=regimes,
                     crossover={"up_erpm": 2100.0, "dn_erpm": 1600.0, "bytes": [54, 195]})
    esc = _FakeESC(tmax=1000, tele=Telem(0, 0.0, 0, 25, 0))         # stale (would stall if mis-keyed)
    ctrl = VelocityController(esc, p, slew_rpm_s=1e6, max_temp=0, max_secs=2.0, stall_secs=0.3)
    ctrl.set_speed(320)                                             # in the [200,700] gap
    assert ctrl.run(SimClock()) == "completed"                     # no stall


def test_outer_clamp_backcalc_does_not_flip_integrator_sign():
    # FF alone saturates the ESC (ff=400 > tmax=350) while the true error still calls for MORE push
    # (measured 250 < target 320). The outer-clamp back-calc must NOT flip the integrator negative.
    p = _profile()                                                 # thrust_for(320) == 400
    esc = _FakeESC(tmax=350, tele=Telem(250, 0.0, 0, 25, 0))       # err = +70, FF over tmax
    ctrl = VelocityController(esc, p, kp=0.4, ki=1.5, trim_max=200.0, blend_secs=0.05,
                              slew_rpm_s=1e6, max_temp=0, max_secs=2.0)
    ctrl.set_speed(320)
    ctrl.run(SimClock())
    assert ctrl._i >= 0.0                                          # stays on the correct side of err


def test_capabilities_round_trip_yaml(tmp_path):
    p = _profile(capabilities={"down_catch": False, "sine_lowspeed": True})
    path = tmp_path / "p.yaml"
    p.save(str(path))
    q = SpeedProfile.load(str(path))
    assert q.capabilities == p.capabilities
    assert q.sine_lowspeed is True and q.down_catch is False


def test_capabilities_absent_defaults_false():
    p = _profile()                               # no capabilities block
    assert p.capabilities is None
    assert p.down_catch is False and p.sine_lowspeed is False


def test_control_gains_round_trip_yaml(tmp_path):
    p = _profile(control={"kp": 0.03, "ki": 0.12, "trim_max": 400.0, "blend_secs": 0.3})
    path = tmp_path / "p.yaml"
    p.save(str(path))
    q = SpeedProfile.load(str(path))
    assert q.control == p.control
    assert q.control_gain("kp") == 0.03 and q.control_gain("trim_max") == 400.0


def test_control_gains_absent_returns_default():
    p = _profile()                               # no control block
    assert p.control is None
    assert p.control_gain("kp") is None
    assert p.control_gain("kp", 0.4) == 0.4      # velctl's fallback path


def test_set_speed_stages_line_to_sine_through_zero():
    # A crossover profile, no down_catch: dropping from above the seam to below it stages the real
    # target and routes the setpoint through ~0 first (re-catch from below).
    p = _profile(crossover={"up_erpm": 2100.0, "dn_erpm": 1600.0, "bytes": [54, 195]})
    c = VelocityController(ESC(object(), 1), p)
    c.setpoint = 400.0                           # currently above the seam (400*7 = 2800 >= 2100)
    c.set_speed(100.0)                           # below the seam
    assert c.target == 0.0 and c._pending == 100.0


def test_set_speed_direct_when_down_catch_or_no_crossover():
    # down_catch advertised -> no staging.
    p = _profile(crossover={"up_erpm": 2100.0, "dn_erpm": 1600.0, "bytes": [54, 195]},
                 capabilities={"down_catch": True})
    c = VelocityController(ESC(object(), 1), p)
    c.setpoint = 400.0
    c.set_speed(100.0)
    assert c.target == 100.0 and c._pending is None
    # stock-like profile (no crossover) -> inert, direct set.
    c2 = VelocityController(ESC(object(), 1), _profile())
    c2.setpoint = 400.0
    c2.set_speed(100.0)
    assert c2.target == 100.0 and c2._pending is None
