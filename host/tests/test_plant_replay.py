# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""Quantitative replay of the committed bench fixtures through `pico_esc.plant.MeasuredPlant`.

Every threshold here is a NUMBER FROM THE BENCH, and the two that the model does NOT meet are asserted
at the value actually achieved with the miss spelled out, rather than loosened silently:

  * the telemetry model's worst row on the fold sweep is 33.8 rpm, not the 25 rpm the rest of the data
    supports (one point at cmd 545 where `tele` reads 7.5 % high mid-seam);
  * `deadman` and `zero_held` brake 2-3x harder than `disarm` from the same speed, so the single
    fitted short-circuit torque is out by that factor on them. It is asserted AS that factor.

THE BRAKE METRIC IS THE BRAKE PHASE, not time-to-stop. Time-to-stop measured from t = 0 is ~72 % fixed
cut delay (0.307 s of 0.406 s on `disarm`), so it stays inside 13 % while the brake torque is out by 2x —
which is exactly how the first version of this file certified a 2x-too-strong brake as "out of sample
within 11 %". Revolutions travelled and duration, both measured FROM ENGAGEMENT and taken straight off
encoder position, are what actually constrain the torque.

Everything is argued in docs/measured-plant.md. A passing test that hid a 2x error is worth nothing.
"""
import csv
import json
import os

import pytest

from pico_esc import measured as M
from pico_esc.plant import MeasuredPlant, preset, with_cross_up

import coastdown
import plantid

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
HOST = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _plant(cfg, cross_up=None, cut_mode="disarm"):
    p = preset(cfg)
    if cross_up is not None:
        p = with_cross_up(p, cross_up)
    return MeasuredPlant(p, cut_mode=cut_mode)


# ------------------------------------------------------------------ static curve
PROP_POINTS = [(543, 794.4), (612, 1510.4), (680, 2212.7)]        # models/350kv_postfix_sysid_raw.json
BARE_POINTS = [(555, 600.556), (570, 750.363), (600, 1069.269),   # sinesweep up leg + fold
               (570, 751.698), (600, 1064.109)]                   # ...and the down leg


def test_prop_on_static_curve_within_4pct():
    pl = _plant(M.PROP)
    for cmd, meas in PROP_POINTS:
        err = 100.0 * (pl.steady(cmd)[0] - meas) / meas
        assert abs(err) <= 4.0, f"cmd {cmd}: {err:+.2f}%"


def test_bare_shaft_static_curve_within_3pct_on_both_legs():
    pl = _plant(M.BARE)
    for cmd, meas in BARE_POINTS:
        err = 100.0 * (pl.steady(cmd)[0] - meas) / meas
        assert abs(err) <= 3.0, f"cmd {cmd}: {err:+.2f}%"


def test_the_wrong_cross_up_blows_the_static_curve_up():
    """The cross_up guard: the SAME code with the other ESC config must be badly wrong, or the term is
    decorative. Measured spread 13.7-45.0 %. The guard is WEAKEST at the top of the range (13.7 % at
    cmd 680) because the shift is a fixed DUTY offset and cmd 680 runs the largest duty — so the
    threshold here is the honest 13 %, with the mean asserted well above the plan's 15 %."""
    errs = []
    for cfg, wrong, pts in ((M.PROP, 36.0, PROP_POINTS), (M.BARE, 34.0, BARE_POINTS)):
        pl = _plant(cfg, cross_up=wrong)
        for cmd, meas in pts:
            errs.append(abs(100.0 * (pl.steady(cmd)[0] - meas) / meas))
    assert min(errs) >= 13.0, f"the weakest point is only {min(errs):.1f}% out"
    assert sum(errs) / len(errs) >= 20.0, f"mean error only {sum(errs) / len(errs):.1f}%"
    assert max(errs) >= 40.0
    assert 13.0 <= min(errs) <= 15.0, f"the weakest point moved: {min(errs):.1f}%"


def test_the_seam_lands_in_the_measured_departure_brackets():
    assert 510.0 <= _plant(M.BARE).seam_cmd() <= 525.0     # measured departure bracket 515..520
    assert 470.0 <= _plant(M.PROP).seam_cmd() <= 545.0     # measured prop-on bracket 475..543


# ------------------------------------------------------------------ telemetry
def test_tele_model_reproduces_the_sinesweep_fixtures():
    pl = _plant(M.BARE)
    worst_all, all_errs = 0.0, []
    for name, cap in (("sinesweep_noprop_20260730_033014.csv", 25.0),
                      ("sinesweep_fold_noprop_20260730_033417.csv", 34.0)):
        errs = [abs(pl.tele(float(r["enc_mean"]), t=99.0) - float(r["tele_rpm"]))
                for r in csv.DictReader(open(os.path.join(DATA, name)))]
        all_errs += errs
        worst_all = max(worst_all, max(errs))
        assert max(errs) <= cap, f"{name}: max {max(errs):.1f} rpm"
    assert len(all_errs) == 60
    assert sum(all_errs) / len(all_errs) <= 12.0
    # DOCUMENTED MISS: the plan's band was 25 rpm on both files. The fold sweep's cmd 545 up-leg row
    # reads tele 487 against an encoder mean of 453 (+7.5 %) mid-seam, which no floor model reproduces.
    assert 25.0 < worst_all <= 34.0


def test_tele_model_reproduces_the_startup_silence_then_the_floor():
    pl = _plant(M.BARE)
    worst = {}
    for r in csv.DictReader(open(os.path.join(
            DATA, "steptrace_startup_noprop_20260730_033906.csv"))):
        t = float(r["t"])
        if t >= 0.5:
            continue
        e = abs(pl.tele(float(r["enc_rpm"]), t=t) - float(r["tele_rpm"]))
        worst[r["run"]] = max(worst.get(r["run"], 0.0), e)
    within = [k for k, v in worst.items() if v <= 5.0]
    assert len(within) >= 2, worst
    # DOCUMENTED MISS: run 2 has ONE row (t=0.474) where the rotor transiently reached 208 rpm while
    # tele still reported the 182 floor — the ESC had not achieved BEMF lock, which the floor model
    # does not represent. 26.3 rpm on 1 of 158 rows.
    assert max(worst.values()) <= 27.0


# ------------------------------------------------------------------ the brake
def _sim_phase(cfg, mode, w_on):
    p = preset(cfg)
    return plantid.sim_brake_phase(cfg, mode, w_on, p.l_phase, p.j, p.brake_cutout_rpm)


def test_brake_phase_fits_the_disarm_decays_in_sample():
    """The four `disarm` bare-shaft decays are what (L, BRAKE_CUTOUT_RPM) were fitted to, against
    revolutions AND duration measured from engagement. In-sample, so the bound is tight."""
    seen = []
    for lbl, mode, cfg, ph in plantid.decay_phases("bare"):
        if mode != "disarm":
            continue
        d, r = _sim_phase(cfg, mode, ph["w_on"])
        e_d = 100.0 * (d - ph["dur"]) / ph["dur"]
        e_r = 100.0 * (r - ph["rev"]) / ph["rev"]
        seen.append((lbl, e_d, e_r))
        assert abs(e_d) <= 12.0, f"{lbl}: brake-phase duration {e_d:+.1f}%"
        assert abs(e_r) <= 12.0, f"{lbl}: brake-phase revolutions {e_r:+.1f}%"
    assert len(seen) == 4


def test_brake_phase_out_of_sample_on_the_prop_on_decays():
    """THE out-of-sample brake check, and the only one that exists. The three prop-on decays reuse the
    bare-shaft-fitted L and cut-out with the prop-on J and drag; nothing was refitted for them.

    The first version of this file claimed this at "within 11 %" using time-to-stop from t = 0, of
    which ~72 % is fixed delay. On the brake-phase metric the true figures are below."""
    seen = []
    for lbl, mode, cfg, ph in plantid.decay_phases("prop"):
        d, r = _sim_phase(cfg, mode, ph["w_on"])
        e_d = 100.0 * (d - ph["dur"]) / ph["dur"]
        e_r = 100.0 * (r - ph["rev"]) / ph["rev"]
        seen.append((lbl, e_d, e_r))
        assert abs(e_d) <= 25.0, f"{lbl}: brake-phase duration {e_d:+.1f}%"
        assert abs(e_r) <= 20.0, f"{lbl}: brake-phase revolutions {e_r:+.1f}%"
    assert len(seen) == 3
    # it must be a REAL transfer, not an accident of a loose bound: the prop-on rotor is 22 % heavier
    # and starts the brake phase 850 rpm lower, and the model tracks both.
    assert all(abs(e) >= 3.0 for _l, e, _r in seen), "suspiciously exact — check the fixtures"


def test_the_mode_dependence_is_in_the_torque_and_is_not_modelled():
    """`deadman` and `zero_held` brake 2-3x harder than `disarm` from the same ~2400 rpm. One
    short-circuit torque cannot be all three, so the model fits `disarm` (the default, and the mode the
    prop-on decays used) and is wrong on the other two BY THAT FACTOR. Asserted as the factor, so a
    future single-law "fix" that quietly splits the difference fails here."""
    by_mode = {mode: ph for _l, mode, _c, ph in plantid.decay_phases("bare")}
    assert by_mode["disarm"]["dur"] > by_mode["deadman"]["dur"] > by_mode["zero_held"]["dur"]
    assert by_mode["disarm"]["dur"] / by_mode["zero_held"]["dur"] >= 2.5
    for mode in ("deadman", "zero_held"):
        d, _r = _sim_phase(M.BARE, mode, by_mode[mode]["w_on"])
        ratio = d / by_mode[mode]["dur"]
        assert ratio >= 1.7, f"{mode}: model/measured brake phase {ratio:.2f} — the model is fitted "
        assert M.BRAKE_PHASE_S[mode] == pytest.approx(by_mode[mode]["dur"], abs=0.006)
    # and the measured data is carried so a caller can see it
    assert set(M.BRAKE_PHASE_S) == {"disarm", "deadman", "zero_held"} == set(M.BRAKE_PHASE_REV)


def test_time_to_stop_from_zero_is_delay_dominated_and_says_so():
    """Why the metric above replaced this one: from t = 0 the two fixed cut delays are ~72 % of the
    number, so it cannot see a 2x torque error. Demonstrated by DOUBLING the brake torque (halving L)
    and watching this metric barely move while the brake-phase metric moves a lot."""
    import pico_esc.plant as PP
    lbl, mode, cfg, ph = next(x for x in plantid.decay_phases("bare") if x[1] == "disarm")
    p = preset(cfg)
    delays = M.CUT_RELEASE_S[mode] + M.BRAKE_ENGAGE_S[mode]
    assert delays / ph["t_stop"] >= 0.70, "the delays should dominate time-to-stop"

    def stop_and_phase(l_phase):
        pl = MeasuredPlant(PP.with_brake(p, l_phase=l_phase), cut_mode=mode)
        pl.reset(ph["w_on"], 0.0)
        t, dt = 0.0, 5e-4
        while t < 1.2 and not (t > delays and abs(pl.rpm) < 40.0):
            pl.step(0.0, dt)
            t += dt
        d, r = plantid.sim_brake_phase(cfg, mode, ph["w_on"], l_phase, p.j, p.brake_cutout_rpm)
        return t, d, r

    t_ref, _d, r_ref = stop_and_phase(p.l_phase)
    t_2x, _d2, r_2x = stop_and_phase(p.l_phase * 0.5)     # ~2x the brake torque at these speeds
    s_stop = abs(t_2x - t_ref) / t_ref
    s_rev = abs(r_2x - r_ref) / r_ref
    assert s_stop < 0.05, f"time-to-stop moved {s_stop:.1%} for a 2x torque change — it should not"
    assert s_rev > 0.20, f"brake-phase revolutions moved only {s_rev:.1%} — the metric is blind too"
    assert s_rev > 10 * s_stop, (
        f"revolutions {s_rev:.1%} vs time-to-stop {s_stop:.1%}: the whole reason the metric changed")


def test_the_brake_cutout_is_visible_in_the_measured_residual():
    """BRAKE_CUTOUT_RPM is not a free knob: subtract the drag table's prediction from the measured
    deceleration and the brake's contribution has died by ~250 rpm, while the short-circuit law's own
    low-speed branch (falling only as w) would still be ~7x that. Checked here on the fixtures."""
    pl = _plant(M.BARE)
    rows = plantid._rows("cutmode_noprop_20260730_032448.csv", "mode")["disarm"]
    tj = plantid.decay_traj(rows)
    t_on = plantid.brake_onset(tj)
    hi, lo = [], []
    for i in range(len(tj) - 1):
        (t, w), (t2, w2) = tj[i], tj[i + 1]
        if t < t_on or w2 < 25:
            continue
        wm = 0.5 * (w + w2)
        drag_decel = M.drag_at(wm) / pl.p.j / plantid.RAD      # N*m -> rad/s^2 -> rpm/s
        resid = (w - w2) / (t2 - t) - drag_decel
        (hi if wm > 1500 else lo).append((wm, resid))
    assert hi and lo
    assert max(r for _w, r in hi) > 40000.0, "the brake should be tens of krpm/s above 1500 rpm"
    bottom = [r for w, r in lo if w < 250.0]
    assert bottom and min(bottom) < 0.0, (
        "below ~250 rpm the measured deceleration should be at or below the drag-only prediction, "
        "i.e. the brake has stopped contributing: " + repr(bottom))
    assert 200.0 <= M.BRAKE_CUTOUT_RPM <= 900.0
    # the law without a cut-out would still be braking hard down there
    assert pl.tau_short_circuit(300.0) / pl.p.j / plantid.RAD > 20000.0


def test_the_measured_cut_delays_match_coastdown_coast_start():
    """coastdown.coast_start() reports ~0.17 / 0.35 / 0.05 s for disarm / deadman / zero_held. The
    parameters in measured.py must be within 30 ms of that, or they are not the same measurement."""
    by_mode = {}
    with open(os.path.join(DATA, "cutmode_noprop_20260730_032448.csv")) as fh:
        for r in csv.DictReader(fh):
            by_mode.setdefault(r["mode"], []).append(
                (float(r["t"]), int(r["accum"]), float(r["encv_rpm"])))
    for mode, rows in by_mode.items():
        res = coastdown.derive(rows, 0.06)
        t_start = res[coastdown.coast_start(res)][0]
        assert abs(t_start - M.CUT_RELEASE_S[mode]) <= 0.030, (
            f"{mode}: coast_start says {t_start:.3f}s, parameter is {M.CUT_RELEASE_S[mode]:.3f}s")
    assert "brake_on_stop=0` does NOT disable" in coastdown.__doc__


def test_the_brake_engage_delay_is_measured_not_assumed():
    """The second delay (drive removed -> brake bites) is read off the fixtures the same way, and it
    is what makes the deceleration peak appear at a different SPEED in the two configurations while
    the peak LOCATION never moved. Independently re-derived by review from raw position."""
    onsets = {}
    for lbl, mode, cfg, obs in plantid.decays("bare") + plantid.decays("prop"):
        t_on = plantid.brake_onset(obs)
        t_rel, _w0, _a = plantid.two_segment(obs, t_on)
        onsets.setdefault((cfg, mode), []).append(t_on - t_rel)
        assert abs((t_on - t_rel) - M.BRAKE_ENGAGE_S[mode]) <= 0.030, (lbl, mode, t_on - t_rel)
    bare = [v for (cfg, m), vs in onsets.items() if cfg == M.BARE and m == "disarm" for v in vs]
    prop = [v for (cfg, m), vs in onsets.items() if cfg == M.PROP for v in vs]
    assert abs(sum(bare) / len(bare) - sum(prop) / len(prop)) <= 0.020
    # the mech_independent tag on those dicts is evidenced for `disarm` ONLY, and the note says so
    for n in ("CUT_RELEASE_S", "BRAKE_ENGAGE_S"):
        assert "`disarm`" in M.PROVENANCE[n]["note"]


# ------------------------------------------------------------------ down-step
def test_open_loop_down_step_half_time_and_trough():
    """thrust 600 -> 480, measured 1048 -> 175 rpm with t_half 24-26 ms and a trough of 114-122 rpm.

    NOTE this is NOT out-of-sample: SINE_PULLOUT_NM was identified from this trace. It is the forced-
    sine pull-out, a different law from the zero-throttle brake (which would predict ~3x too fast)."""
    meas = []
    by_run = {}
    with open(os.path.join(DATA, "steptrace_down_noprop_20260730_033706.csv")) as fh:
        for r in csv.DictReader(fh):
            by_run.setdefault(r["run"], []).append((float(r["t"]), abs(float(r["enc_rpm"]))))
    for _run, rr in sorted(by_run.items()):
        tail = [v for t, v in rr if t > 1.0]
        fin = sum(tail) / len(tail)
        half = (rr[0][1] + fin) / 2.0
        meas.append((next(t for t, v in rr if v <= half), min(v for t, v in rr if t > 0.05), fin))
    m_half = sum(x[0] for x in meas) / len(meas)
    assert 0.022 <= m_half <= 0.027, m_half

    pl = _plant(M.BARE)
    pl.reset(1048.0, 0.0)
    sim, t, dt = [], 0.0, 2e-4
    while t < 1.0:
        pl.step(480, dt)
        t += dt
        sim.append((t, abs(pl.rpm)))
    s_half = next(t for t, v in sim if v <= (1048.0 + 175.0) / 2.0)
    trough = min(v for t, v in sim if t > 0.005)
    assert abs(s_half - m_half) / m_half <= 0.50, f"t_half {s_half * 1e3:.1f} ms vs {m_half * 1e3:.1f}"
    assert 100.0 <= trough <= 200.0, trough
    assert sim[-1][1] > 100.0, "must NOT stall — it settles on the forced field"
    assert abs(sim[-1][1] - pl.sine_field(480)) < 1.0


# ------------------------------------------------------------------ sysid steps
def test_the_crossing_step_reproduces_the_plateau_then_the_jump():
    """407 -> 680 prop-on: the rotor sits at 360-415 rpm for ~0.26 s and only then covers the last
    1800 rpm in 40 ms. Reproduced by the ESC duty slew + the crossover duty clamp, up-direction only."""
    raw = os.path.join(HOST, "models", "350kv_postfix_sysid_raw.json")
    if not os.path.exists(raw):
        pytest.skip("models/ is gitignored; the tracked sysid file is absent")
    st = next(s for s in json.load(open(raw))["steps"] if s["bias"] == 407 and s["target"] == 680)
    real = [(t, r) for t, r, *_ in st["rows"] if r is not None]
    t95m = next(t for t, r in real if r >= 0.95 * 2212.7)

    pl = _plant(M.PROP)
    pl.reset(real[0][1], 0.0)
    sim, t, dt = [], 0.0, 5e-4
    while t < 1.5:
        pl.step(680, dt)
        t += dt
        sim.append((t, pl.rpm))
    t95s = next(t for t, r in sim if r >= 0.95 * 2212.7)
    assert abs(t95s - t95m) / t95m <= 0.25, f"t95 {t95s:.3f}s vs measured {t95m:.3f}s"
    plateau = [r for t, r in sim if 0.15 <= t <= 0.38]
    assert 300.0 <= min(plateau) and max(plateau) <= 500.0, (min(plateau), max(plateau))
    # and the slew is UP-ONLY: stepping back down is immediate, not another 0.4 s ramp
    pl.reset(2213.0, 0.0)
    pl.step(407, 1e-3)
    assert pl.applied_duty(407) == 0.0


def test_the_sine_region_steps_are_reported_as_cogging_not_fitted():
    """The two sine-region sysid steps are the ones that produced the old 0.18 s lag. A model has
    nothing to fit there: the rotor spans -29..249 rpm around a 145 rpm forced field."""
    raw = os.path.join(HOST, "models", "350kv_postfix_sysid_raw.json")
    if not os.path.exists(raw):
        pytest.skip("models/ is gitignored")
    for s in json.load(open(raw))["steps"]:
        if s["bias"] == 65 and s["target"] == 407:
            r = [x[1] for x in s["rows"] if x[1] is not None]
            span = max(r) - min(r)
            field = _plant(M.PROP).sine_field(407)
            assert span > 1.5 * field, (span, field)      # dispersion exceeds the signal itself


# ------------------------------------------------------------------ forced sine
def test_forced_sine_mean_speed_within_30pct_over_cmd_270_510():
    pl = _plant(M.BARE)
    worst = 0.0
    n = 0
    for name in ("sinesweep_noprop_20260730_033014.csv",
                 "sinesweep_fold_noprop_20260730_033417.csv"):
        for r in csv.DictReader(open(os.path.join(DATA, name))):
            cmd, enc = int(r["cmd"]), abs(float(r["enc_mean"]))
            if not 270 <= cmd <= 510:
                continue
            n += 1
            worst = max(worst, abs(100.0 * (abs(pl.steady(cmd)[0]) - enc) / enc))
    assert n >= 20
    assert worst <= 30.0, f"worst {worst:.1f}%"


def test_the_incoherent_band_is_carried_as_data():
    pl = _plant(M.BARE)
    for r in csv.DictReader(open(os.path.join(DATA, "sinesweep_noprop_20260730_033014.csv"))):
        cmd, spread, rev = int(r["cmd"]), float(r["enc_spread"]), int(r["reversals"])
        if cmd > 510:
            continue
        assert pl.spread_at(cmd) >= spread * 0.5, cmd
        if rev > 0:
            assert pl.reversals_at(cmd) >= 1, cmd


# ------------------------------------------------------------------ tool plumbing
def test_plantid_is_offline_and_dry_run_touches_nothing(capsys):
    assert plantid.main(["--dry-run"]) == 0
    out = capsys.readouterr().out
    assert "never a serial port" in out
    # No import can reach hardware: `esctool` (EscHost/find_pico) and `serial` are the only routes.
    import ast
    tree = ast.parse(open(os.path.join(HOST, "plantid.py")).read())
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported |= {a.name.split(".")[0] for a in node.names}
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
            imported |= {a.name for a in node.names}
    for forbidden in ("esctool", "serial", "EscHost", "find_pico", "link"):
        assert forbidden not in imported, f"plantid.py imports {forbidden}"


def test_plantid_provenance_and_fit_helpers_run_offline(capsys):
    assert plantid.main(["provenance"]) == 0
    out = capsys.readouterr().out
    assert "L_PHASE_FITTED" in out and "OPEN" in out
    assert len(plantid.decays("bare")) == 6 and len(plantid.decays("prop")) == 3
