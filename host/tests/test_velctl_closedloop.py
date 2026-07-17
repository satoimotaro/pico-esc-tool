# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""Phase A1 closed-loop convergence tests (VelocityController against SimEncEscHost).

These PROVE the loop, not the curve: a deliberately mis-scaled feed-forward profile (rpm x1.25 ->
the FF commands only ~80% of the thrust the target needs) must still converge to the target once
the PI trim closes on live telemetry above the crossover seam, while the SAME run with kp=ki=0
(pure FF) misses badly. Below the seam telemetry is stale (forced sine), so the loop degrades to
exact feed-forward. The sim is a MODEL of the S3 firmware, not hardware truth.
"""
import statistics

from pico_esc import ESC, SpeedProfile, VelocityController
from pico_esc.config import sine_crossover_bytes
from pico_esc.link import SimClock
from pico_esc.sim import SimEncEscHost

# Crossover configured LOW enough (up ~1600 eRPM) that a mis-scaled-FF command still crosses into
# 6-step, but above the firmware BEMF floor. cross_dn well below so the loop stays in 6-step.
UP_ERPM, DN_ERPM = 1600.0, 1400.0
POLE_PAIRS = 7
FS = 357.0                                   # sim S1 full-scale mech RPM at |thrust|=1000


def _line_mech(thr):
    """The sim's 6-step BEMF load-line: |thrust| -> mech RPM (mirrors SimEncEscHost anchors)."""
    slope = (3800.0 - 190.0) / (700.0 - 55.0)
    return (190.0 + slope * (abs(thr) - 55.0)) / POLE_PAIRS


def _profile(scale=1.0):
    """A finely-sampled sine+load-line curve. scale multiplies the rpm axis: scale>1 makes the FF
    OVER-report speed, so inverting a target UNDER-commands the thrust (the mis-calibration we
    correct with the loop). Monotonic in both axes for any scale >= 1."""
    sine = [(0, 0.0), (100, 35.7), (300, 107.1), (500, 178.5), (600, 214.2)]
    line = [(t, _line_mech(t)) for t in (640, 700, 760, 820, 880, 940, 1000)]
    pts = [(t, r * scale) for t, r in sine + line]
    cu, cd = sine_crossover_bytes(UP_ERPM, DN_ERPM)
    return SpeedProfile(pts, motor="conv",
                        crossover={"up_erpm": UP_ERPM, "dn_erpm": DN_ERPM, "bytes": [cu, cd]})


def _sim(profile, seed=1234, crossover=True):
    clock = SimClock()
    h = SimEncEscHost(clock, seed=seed)
    if crossover and profile.crossover:
        cu, cd = profile.crossover["bytes"]
        h.cmd(f"editpage 1 32:{cu:02X},33:{cd:02X}")
    h.cmd("arm 1 bidir")
    return ESC(h, 1, clock=clock), clock, h


def _run(profile, target, *, kp=0.4, ki=1.5, secs=5.0, slew=500.0, seed=1234, crossover=True,
         stop_below_rpm=0.0):
    esc, clock, h = _sim(profile, seed=seed, crossover=crossover)
    ctrl = VelocityController(esc, profile, kp=kp, ki=ki, slew_rpm_s=slew, max_temp=0,
                              max_secs=secs, stall_secs=2.0, stop_below_rpm=stop_below_rpm)
    ctrl.set_speed(target)
    rows = []

    def on_row(t, tg, sp, sent, temp, enc, tele_rpm, trim):
        rows.append((t, sp, sent, tele_rpm, trim))
    reason = ctrl.run(clock, on_row=on_row)
    esc.disarm()
    return reason, rows


def _steady_meas(rows, since=4.0):
    return [tele for t, sp, sent, tele, trim in rows if t >= since and tele is not None]


# ---------------------------------------------------------------------------
# Real convergence: the loop, not the curve.
# ---------------------------------------------------------------------------
def test_closed_loop_converges_with_misscaled_ff():
    prof = _profile(scale=1.25)                       # FF ~80% of the needed thrust
    reason, rows = _run(prof, 700, kp=0.4, ki=1.5)
    assert reason == "completed"
    meas = _steady_meas(rows)
    assert meas, "expected live telemetry in the last second (loop should be in 6-step)"
    err = statistics.mean(abs(m - 700) for m in meas)
    assert err <= 0.05 * 700, f"closed loop did not converge (mean |err|={err:.1f} RPM)"


def test_set_speed_zero_commands_stop():
    # `rpm 0` must command a true thrust 0 (motor stops), not creep via the FF/slew/PI.
    prof = _profile(scale=1.0)
    reason, rows = _run(prof, 0.0, secs=2.0)
    assert reason == "completed"
    assert rows and all(sent == 0 for _, _, sent, _, _ in rows), "rpm 0 must hold thrust 0"


def test_stop_below_rpm_stops_subfloor_target():
    # A sub-floor target with stop_below_rpm set must STOP (thrust 0) rather than servo a speed the FF
    # would otherwise command (proves the stop overrides a would-be-driving command).
    prof = _profile(scale=1.0)
    assert prof.thrust_for(100.0) != 0                # the FF alone would drive here
    reason, rows = _run(prof, 100.0, secs=2.0, stop_below_rpm=150.0)
    assert rows and all(sent == 0 for _, _, sent, _, _ in rows), "sub-floor target must stop"


def test_pure_feedforward_misses_the_target():
    # SAME mis-scaled profile + sim, but kp=ki=0 (pure FF) -> the command is ~80% and the speed
    # misses low by >=15%, proving the convergence above is the LOOP, not the curve.
    prof = _profile(scale=1.25)
    reason, rows = _run(prof, 700, kp=0.0, ki=0.0)
    assert reason == "completed"
    meas = _steady_meas(rows)
    assert meas
    err = statistics.mean(abs(m - 700) for m in meas)
    assert err >= 0.15 * 700, f"pure FF missed by only {err:.1f} RPM (expected >=15%)"


def test_closed_loop_deterministic_same_seed():
    prof = _profile(scale=1.25)
    _, a = _run(prof, 700, seed=7)
    _, b = _run(prof, 700, seed=7)
    assert a == b                                      # byte-for-byte reproducible (loop included)


# ---------------------------------------------------------------------------
# Below the seam: telemetry stale (forced sine) -> exact feed-forward, PI inert.
# ---------------------------------------------------------------------------
def test_below_seam_is_pure_feedforward():
    prof = _profile(scale=1.0)
    reason, rows = _run(prof, 150, kp=0.4, ki=1.5, secs=3.0)   # 150 mech -> below the seam (sine)
    assert reason == "completed"
    # once the setpoint has settled, every command is EXACTLY the FF lookup: trim 0, tele stale.
    for t, sp, sent, tele_rpm, trim in rows[-30:]:
        assert sent == int(prof.thrust_for(sp))
        assert trim == 0.0
        assert tele_rpm is None                         # forced sine reports stale telemetry


def test_sim_reports_stale_tele_in_sine_but_live_in_6step():
    # subtask 2: with the crossover ON the sim's `tele` is STALE (0) in forced sine and LIVE in
    # 6-step (the OPPOSITE of the pre-A1 sim) — the honest hardware behaviour the loop keys on.
    prof = _profile()
    _, clock, h = _sim(prof)
    for _ in range(60):
        h.cmd("thrust 1 300")                           # below the seam -> forced sine
        clock.sleep(0.02)
    assert h._regime == "sine"
    assert int(h.cmd("tele 1")[0].split("|")[1]) == 0   # stale
    for _ in range(80):
        h.cmd("thrust 1 950")                           # above Cross_Up -> 6-step
        clock.sleep(0.02)
    assert h._regime == "line"
    assert int(h.cmd("tele 1")[0].split("|")[1]) != 0   # live


# ---------------------------------------------------------------------------
# ESC-agnostic: a stock-Bluejay-like profile (NO crossover, NO capabilities) runs and closes.
# ---------------------------------------------------------------------------
def test_stock_like_profile_closes_on_tele_liveness():
    # No crossover, no capabilities anywhere. In the sim with the crossover OFF, `tele` is live
    # (plain 6-step-like), so the PI authority engages PURELY on tele-liveness — never on any
    # crossover/capability config.
    pts = [(0, 0.0), (100, 35.7), (300, 107.1), (500, 178.5), (700, 249.9), (900, 321.3)]
    prof = SpeedProfile(pts, motor="stock")
    assert prof.crossover is None and prof.capabilities is None
    esc, clock, h = _sim(prof, crossover=False)         # crossover OFF -> stock-like tele
    ctrl = VelocityController(esc, prof, kp=0.4, ki=1.5, slew_rpm_s=500.0, max_temp=0, max_secs=5.0)
    ctrl.set_speed(200)
    rows = []
    ctrl.run(clock, on_row=lambda t, tg, sp, s, tp, e, tr, trim: rows.append((t, tr)))
    esc.disarm()
    meas = [tr for t, tr in rows if t >= 4.0 and tr is not None]
    assert meas, "stock-like ESC telemetry should be live"
    assert statistics.mean(abs(m - 200) for m in meas) <= 0.05 * 200
