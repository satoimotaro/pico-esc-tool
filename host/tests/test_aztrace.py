# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""Tests for pico_esc.aztrace — the `az dump` reader and its metrics.

The point of these is the distinction that the bench measurements kept losing: a pod STOPPED
70 deg off target and a pod SWINGING +/-70 deg produce the same |err| min..max and need opposite
fixes. test_stopped_offset_vs_limit_cycle is the regression guard for that.
"""
import math

import pytest

from pico_esc.aztrace import (
    AzTrace, fit_step, metrics, parse_dump, seam_margin, suggest_gains, verdict,
)

HDR = ("azd|n=3|decim=5|kp=3.000|ki=0.500|kd=0.300|kff=0.02296|trim=-29.8|smax=40|vmax=200|"
       "carrier=-600|maxrate=4000|verrclamp=0|signa=1|signb=-1|seam=515|tgt=0.00")


def _dump(rows, hdr=HDR):
    return [hdr, "t_ms,pos,vel,s,ivel"] + [",".join(f"{v}" for v in r) for r in rows] + ["ok"]


def _synth(n, dt, pos_fn, vel_fn, s=-29.8, cfg_extra=""):
    rows = [(i * dt * 1000.0, pos_fn(i * dt), vel_fn(i * dt), s, 0.0) for i in range(n)]
    return parse_dump(_dump(rows, HDR + cfg_extra))


# --- parsing ---------------------------------------------------------------------
def test_parses_header_and_rows():
    tr = parse_dump(_dump([(0.0, 1.0, 10.0, -29.8, 0.5),
                           (5.0, 1.5, 12.0, -30.0, 0.6),
                           (10.0, 2.0, 11.0, -30.2, 0.7)]))
    assert len(tr) == 3
    assert tr.config["kp"] == 3.0 and tr.config["ki"] == 0.5
    assert tr.config["carrier"] == -600 and tr.config["seam"] == 515
    assert tr.t == [0.0, 0.005, 0.010]          # t_ms -> seconds
    assert tr.vel == [10.0, 12.0, 11.0]


def test_header_is_mandatory():
    """A trace that cannot say what configuration produced it is not evidence — a sweep once
    inherited vmax=120 from a previous trial and the result was filed against the wrong number."""
    with pytest.raises(ValueError, match="azd"):
        parse_dump(["t_ms,pos,vel,s,ivel", "0.0,1,2,3,4", "ok"])


def test_device_refusal_is_reported():
    with pytest.raises(ValueError, match="refused"):
        parse_dump([HDR, "err az: stop the loop first (az off) — dump blocks poll()"])


def test_rate_hz_is_measured_not_assumed():
    """Self-paced firmware: the real rate is whatever poll() managed, not the nominal decimation."""
    tr = _synth(50, 0.01, lambda t: 0.0, lambda t: 0.0)      # 100 Hz actual, decim says 5
    assert tr.rate_hz == pytest.approx(100.0, rel=0.05)


# --- the distinction the old metric could not make --------------------------------
def test_stopped_offset_vs_limit_cycle():
    # 4 s at 200 Hz: two full cycles of the 0.5 Hz hunt, so pk-pk sees both rails.
    stopped = _synth(800, 0.005, lambda t: 70.0, lambda t: 0.0)
    cycling = _synth(800, 0.005, lambda t: 70.0 * math.sin(2 * math.pi * 0.5 * t),
                     lambda t: 70.0 * 2 * math.pi * 0.5 * math.cos(2 * math.pi * 0.5 * t))

    ms, mc = metrics(stopped), metrics(cycling)

    # Both wander over ~the same |err| range — the number the bench reported for months.
    assert max(abs(e) for e in (ms["err_mean"],)) == pytest.approx(70.0, abs=1)
    assert mc["pos_pkpk"] > 100                      # ...but only one of them is moving
    assert ms["pos_pkpk"] == pytest.approx(0.0, abs=1e-6)
    assert ms["vel_rms"] == pytest.approx(0.0, abs=1e-6)
    assert mc["vel_rms"] > 100

    assert "CONVERGED" in verdict(ms)                # stopped, with an offset for I to remove
    assert "NOT converged" in verdict(mc)


def test_convergence_is_about_velocity_not_error():
    """Zero steady-state error while still moving must NOT read as converged."""
    tr = _synth(200, 0.005, lambda t: 200.0 * t - 100.0, lambda t: 200.0)
    m = metrics(tr)
    assert abs(m["err_mean"]) < 1.0                  # sweeps straight through the target
    assert "NOT converged" in verdict(m)


def test_window_takes_the_tail_only():
    tr = _synth(400, 0.01, lambda t: 0.0 if t > 1.0 else 500.0,
                lambda t: 0.0 if t > 1.0 else 900.0)
    assert metrics(tr, window=1.5)["vel_rms"] == pytest.approx(0.0, abs=1e-6)
    assert metrics(tr, window=None)["vel_rms"] > 100     # whole trace includes the transient


# --- actuator health --------------------------------------------------------------
def test_seam_margin_matches_the_measured_asymmetry():
    """trim -29.8 + smax 40 -> s spans [-69.8, +10.2]; at carrier -600 that bottoms motor B at
    530 against a 515 seam, i.e. 15 counts — not the 60-85 the docs claimed for a while."""
    tr = _synth(3, 0.005, lambda t: 0.0, lambda t: 0.0, s=-69.8)
    assert seam_margin(tr)[0] == pytest.approx(600 - 69.8 - 515, abs=0.01)   # 15.2
    tr2 = _synth(3, 0.005, lambda t: 0.0, lambda t: 0.0, s=10.2)
    assert seam_margin(tr2)[0] == pytest.approx(600 - 10.2 - 515, abs=0.01)  # 74.8


def test_saturation_fraction_flags_the_authority_rail():
    tr = _synth(10, 0.005, lambda t: 0.0, lambda t: 0.0, s=-69.8)   # u = s - trim = -40 = -smax
    assert metrics(tr)["sat_frac"] == pytest.approx(1.0)
    tr2 = _synth(10, 0.005, lambda t: 0.0, lambda t: 0.0, s=-29.8)  # u = 0
    assert metrics(tr2)["sat_frac"] == pytest.approx(0.0)


# --- plant identification ---------------------------------------------------------
def test_fit_step_recovers_a_known_first_order_plant():
    tau, gain, ds = 0.15, 43.6, 20.0             # deg/s per count, 20 counts of step
    v_inf = gain * ds
    tr = _synth(600, 0.005,
                lambda t: v_inf * (t - tau * (1 - math.exp(-t / tau))),
                lambda t: v_inf * (1 - math.exp(-t / tau)),
                s=ds - 29.8)
    f = fit_step(tr, s_cmd=ds)
    assert f["ok"], f
    assert f["tau"] == pytest.approx(tau, rel=0.05)
    assert f["gain"] == pytest.approx(gain, rel=0.05)
    assert f["r2"] > 0.99


def test_fit_step_infers_the_step_from_the_delivered_command():
    """s carries the trim, so the plant saw s - trim. Getting that wrong scales the gain."""
    tau, gain, ds = 0.12, 40.0, 15.0
    v_inf = gain * ds
    tr = _synth(400, 0.005, lambda t: 0.0, lambda t: v_inf * (1 - math.exp(-t / tau)),
                s=ds - 29.8)
    f = fit_step(tr)                              # no s_cmd -> infer
    assert f["step_s"] == pytest.approx(ds, rel=0.01)
    assert f["gain"] == pytest.approx(gain, rel=0.05)


def test_fit_step_refuses_a_pod_that_never_moved():
    tr = _synth(200, 0.005, lambda t: 0.0, lambda t: 0.0, s=-29.8)
    assert not fit_step(tr, s_cmd=20.0)["ok"]


def test_suggest_gains_inverts_the_damping_relation():
    """zeta = (1+kd)/(2*sqrt(kp*tau)) — the relation that says kp 3/kd 0.3 is near critical at
    tau 0.15 (0.97) while kp 8/kd 0.6 is not (0.73)."""
    g = suggest_gains({"ok": True, "tau": 0.15, "gain": 43.6}, zeta=1.0, kd=0.3)
    assert g["ok"]
    zeta_back = (1.0 + g["kd"]) / (2.0 * math.sqrt(g["kp"] * g["tau"]))
    assert zeta_back == pytest.approx(1.0, rel=1e-6)
    assert g["kp"] == pytest.approx(2.82, rel=0.02)      # lands on the config that converged
    assert g["kff"] == pytest.approx(1.0 / 43.6, rel=1e-6)


def test_suggest_gains_will_not_invent_ki():
    g = suggest_gains({"ok": True, "tau": 0.15, "gain": 43.6})
    assert "ki" not in g and "drift" in g["note"]


def test_empty_trace_is_not_a_pass():
    assert metrics(AzTrace(config={"tgt": 0.0})) == {"n": 0}
    assert verdict({"n": 0}) == "no data"
