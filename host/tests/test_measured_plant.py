# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""Structural tests for the measured plant: provenance, scoping, the seam, and the torque identities.

These are the guards that make the model auditable rather than merely plausible:

  * a number cannot enter `pico_esc.measured` without a provenance record naming the tool, the file
    and the MECHANICAL CONFIGURATION it came from;
  * a preset cannot silently mix a bare-shaft number with a prop-on one;
  * the dynamic torque law's equilibrium must land EXACTLY on the static curve, or the static
    validation is an accident;
  * the brake must be the same expression as the 6-step drive with the duty removed;
  * and none of the five fitted constants the old sim used may exist anywhere.
"""
import os
import re
import subprocess

import pytest

from pico_esc import measured as M
from pico_esc import plant as P
from pico_esc.config import SINE_CROSS_UP_ERPM_MIN
from pico_esc.motor_model import MotorModel

HOST = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(HOST)
DATA = os.path.join(HOST, "tests", "data")


# --------------------------------------------------------------- provenance
def test_provenance_covers_exactly_the_exported_parameters():
    """A number without provenance is a magic constant. This is the gate that makes that impossible."""
    assert set(M.PROVENANCE) == set(M.parameter_names()), (
        f"missing provenance: {sorted(set(M.parameter_names()) - set(M.PROVENANCE))}; "
        f"provenance for a non-parameter: {sorted(set(M.PROVENANCE) - set(M.parameter_names()))}")
    assert set(M.PARAMS) == set(M.PROVENANCE)


def test_every_provenance_record_is_complete():
    for name, p in M.PROVENANCE.items():
        for key in ("units", "tool", "source", "mech_config", "date", "kind", "mech_independent",
                    "note"):
            assert key in p, f"{name} provenance is missing {key}"
        assert p["kind"] in ("measured", "fitted", "derived"), (name, p["kind"])
        assert p["mech_config"] in (M.PROP, M.BARE), (name, p["mech_config"])
        assert p["mech_independent"] in (True, False, None), (name, p["mech_independent"])
        assert len(p["note"]) > 20, f"{name} has no real note"
        assert re.match(r"^\d{4}-\d{2}-\d{2}$", p["date"]), (name, p["date"])


def test_bare_shaft_inertia_is_fitted_and_prop_inertia_is_measured():
    """Only the prop-on J was ever measured. The bare-shaft one is a fit, and saying so in the code is
    the difference between a model and a claim."""
    assert M.PROVENANCE["J_BARE_FITTED"]["kind"] == "fitted"
    assert M.PROVENANCE["J_PROP"]["kind"] == "measured"
    assert M.PROVENANCE["L_PHASE_FITTED"]["kind"] == "fitted"
    # ...and L's transferability is explicitly UNRESOLVED, not quietly assumed
    assert M.PROVENANCE["L_PHASE_FITTED"]["mech_independent"] is None
    # J_bare comes from the FREE-COAST window, not from the brake. An earlier draft claimed the
    # brake-peak magnitude pinned it, which was false — nothing in the brake fit constrained the
    # brake torque scale at all.
    assert "free-coast" in M.PROVENANCE["J_BARE_FITTED"]["tool"].lower()
    assert "NOT from the brake" in M.PROVENANCE["J_BARE_FITTED"]["note"]
    # and both crossover parameters go through the prop drag table, so neither may claim independence
    for n in ("DUTY_CROSSOVER_CLAMP", "DUTY_SLEW_UP_PER_S"):
        assert M.PROVENANCE[n]["mech_independent"] is None, n


def test_no_preset_mixes_mechanical_configurations_undeclared():
    """The set of parameters a preset draws on is RECORDED from `plant.preset()`, not declared beside
    it. The hand-maintained list this replaces had already drifted: `SINE_COHERENT_CMD_MIN`
    (bare-shaft, mech_independent=False) was read by the PROP preset while appearing in neither its
    `shared` nor its `borrowed` entry, so the guard reported "no undeclared mixing" while
    `MeasuredPlant(preset(PROP)).steady(180)` was answering on bare-shaft data."""
    for cfg in (M.PROP, M.BARE):
        with pytest.raises(RuntimeError):
            M.preset_param_names("not_a_config")
        P.preset(cfg)                       # populates the recorded read set
        borrowed = M.preset_borrowed(cfg)
        for name in M.preset_param_names(cfg):
            p = M.PROVENANCE[name]
            if p["mech_config"] == cfg:
                continue
            assert p["mech_independent"] is True or name in borrowed, (
                f"preset {cfg} uses {name} (measured on {p['mech_config']}) without tagging it "
                f"mech_independent or declaring it in PRESETS[...]['borrowed']")
        for name, why in borrowed.items():
            assert len(why) > 40, f"{cfg} borrows {name} without a real reason"
            assert name in M.preset_param_names(cfg), (
                f"{cfg} declares {name} borrowed but never reads it — the borrow map must describe "
                f"the code, not the other way round")
        # the specific hole that was live: PROP reads the bare-shaft coherence band
        if cfg == M.PROP:
            assert "SINE_COHERENT_CMD_MIN" in borrowed
            assert P.MeasuredPlant(P.preset(M.PROP)).steady(180)[1] == "incoherent"


def test_the_two_presets_carry_the_evidenced_cross_up():
    assert P.preset(M.PROP).cross_up == 34.0
    assert P.preset(M.BARE).cross_up == 36.0
    assert P.PROP_350KV_20260726().cross_up == 34.0
    assert P.BARE_SHAFT_350KV_20260730().cross_up == 36.0


def test_provenance_table_renders_every_parameter():
    txt = M.provenance_table()
    for name in M.PARAMS:
        assert name in txt


# --------------------------------------------------------------- fixture manifest
def test_manifest_describes_every_committed_fixture():
    import json
    mf = json.load(open(os.path.join(DATA, "plant_fixtures.json")))
    assert "_cross_up_provenance" in mf, "the cross_up evidence must live with the fixtures"
    ev = mf["_cross_up_provenance"]
    assert "duty_sweep_20260729_220535" in " ".join(ev["evidence"])
    assert "flash" in " ".join(ev["reconciliation_with_git"]).lower()
    total = 0
    for name, meta in mf["fixtures"].items():
        path = os.path.join(DATA, name)
        assert os.path.exists(path), name
        total += os.path.getsize(path)
        for key in ("tool", "role", "mech_config", "cross_up", "run_key", "runs", "rows", "note"):
            assert key in meta, (name, key)
        assert meta["mech_config"] in (M.PROP, M.BARE)
        assert meta["cross_up"] == (34 if meta["mech_config"] == M.PROP else 36)
    assert total < 300 * 1024, f"fixtures are {total / 1024:.0f} KB, budget is 300 KB"
    # the open-loop commands are recorded (and flagged as inferred, because no CSV carried them)
    cmds = mf["_open_loop_commands"]
    assert "600" in cmds["steptrace_down"] and "480" in cmds["steptrace_down"]
    assert "1200" in cmds["steptrace_startup"]
    assert "2400" in cmds["steptrace_startup2400"]
    assert "inferred" in cmds["note"].lower()


def test_the_down_step_fixture_still_resolves_its_24ms_half_time():
    """The trim was a TIME trim, not a decimation, precisely so this survives."""
    import csv
    rows = [r for r in csv.DictReader(open(os.path.join(
        DATA, "steptrace_down_noprop_20260730_033706.csv"))) if r["run"] == "1"]
    early = [(float(r["t"]), abs(float(r["enc_rpm"]))) for r in rows if float(r["t"]) < 0.05]
    assert len(early) >= 8, "too few samples inside the 24 ms half-time to resolve it"
    assert early[0][1] > 1000.0


# --------------------------------------------------------------- E.9: the seam
@pytest.mark.parametrize("cross_up,erpm", [(34.0, SINE_CROSS_UP_ERPM_MIN), (36.0, 1406.25),
                                           (50.0, 1953.125)])
def test_cross_up_erpm_matches_the_cpp_mirror(cross_up, erpm):
    """Same table as lib/vel_control/test/test_sense_gate.cpp. The C++ floors at the literal 1333.0f
    and Python at 4*10000/30 = 1333.333..., so the parity band is 0.5 eRPM (0.03 %) — inside one
    cross_up unit (39.0625) by two orders of magnitude."""
    m = MotorModel(350, 7, 11.2, cross_up=cross_up)
    assert abs(m.cross_up_erpm() - erpm) <= 0.5


def test_cross_up_34_is_numerically_the_old_default_path():
    """The default must stay byte-identical, or every existing caller silently moves."""
    m = MotorModel(350, 7, 11.1)
    assert m.cross_up == 34.0
    assert abs(m.handoff_mech() - SINE_CROSS_UP_ERPM_MIN / 7) < 1e-9


def test_every_seam_derived_quantity_tracks_cross_up():
    a, b = MotorModel(350, 7, 11.2, cross_up=34.0), MotorModel(350, 7, 11.2, cross_up=50.0)
    assert b.handoff_mech() > a.handoff_mech()
    assert b.handoff_cmd() > a.handoff_cmd()
    # the landing FALLS as cross_up rises: the extra command buys less duty than the seam gains
    assert b.sixstep_floor_rpm() < a.sixstep_floor_rpm()
    assert a.in_gap(250.0) and not b.in_gap(250.0)
    assert a.regime(250.0) == "gap" and b.regime(250.0) == "sine"
    assert b.thrust_for(300.0) != a.thrust_for(300.0)


def test_softsensor_requires_cross_up():
    import softsensor
    with pytest.raises(TypeError):
        softsensor.duty(600)
    with pytest.raises(TypeError):
        softsensor.estimate(350, 7, [(600, 1000), (650, 1400)])
    assert softsensor.duty(600, 36) < softsensor.duty(600, 34)


def test_no_call_site_relies_on_a_duty_default():
    """Every duty / cmd_for_duty6 / estimate CALL in host/ passes cross_up explicitly.

    Parsed with `ast`, not grepped: prose like "rpm = KV*V*duty(cmd)" appears in four module
    docstrings and a text search cannot tell it from a real call site. One remaining default is the
    whole E.9 bug back."""
    import ast
    watched = {"duty": 2, "duty6": 2, "cmd_for_duty6": 2, "estimate": 4, "duty_model": 2}
    bad = []
    # The SHIPPED tools and library only. The tests deliberately call duty(600) bare to assert the
    # TypeError (test_softsensor.test_cross_up_is_required_not_defaulted), so scanning them would
    # flag the very check that proves the default is gone.
    for root in (HOST, os.path.join(HOST, "pico_esc")):
        for f in sorted(os.listdir(root)):
            if not f.endswith(".py"):
                continue
            path = os.path.join(root, f)
            for node in ast.walk(ast.parse(open(path).read(), filename=path)):
                if not isinstance(node, ast.Call):
                    continue
                fn = node.func
                name = fn.attr if isinstance(fn, ast.Attribute) else getattr(fn, "id", None)
                if name not in watched:
                    continue
                kw = {k.arg for k in node.keywords}
                if len(node.args) < watched[name] and "cross_up" not in kw:
                    bad.append(f"{path}:{node.lineno}: {name}() with {len(node.args)} positional "
                               f"args and no cross_up=")
    assert not bad, "call sites relying on the default cross_up:\n" + "\n".join(bad)


# --------------------------------------------------------------- static + seam
def test_steady_is_the_max_of_the_two_branches():
    pl = P.MeasuredPlant(P.preset(M.BARE))
    for cmd in (300, 450, 520, 600, 700):
        r, reg = pl.steady(cmd)
        assert r == pytest.approx(max(pl.sine_field(cmd), pl.sixstep_rpm(cmd)))
        assert reg in ("sine", "6step")


def test_the_seam_emerges_where_the_branches_cross():
    assert 510.0 <= P.MeasuredPlant(P.preset(M.BARE)).seam_cmd() <= 525.0
    assert 470.0 <= P.MeasuredPlant(P.preset(M.PROP)).seam_cmd() <= 545.0


def test_no_hysteresis_parameter_exists_in_the_seam_model():
    """The seam is where two curves cross, so it is ONE number. Anything that stores a second
    threshold, a fold width or a sticky regime would be a hysteresis parameter by another name."""
    import ast
    src = open(os.path.join(HOST, "pico_esc", "plant.py")).read()
    tree = ast.parse(src)
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Name):
            names.add(node.id.lower())
        elif isinstance(node, ast.Attribute):
            names.add(node.attr.lower())
        elif isinstance(node, ast.arg):
            names.add(node.arg.lower())
    for token in ("hyster", "fold", "deadtime", "dead_time", "regime_lag", "lag"):
        offenders = [n for n in names if token in n]
        assert not offenders, f"{token} appears as an identifier in plant.py: {offenders}"
    # PlantParams must not carry a second seam threshold: exactly one cross_up, and no *_dn field
    import dataclasses
    fields = {f.name for f in dataclasses.fields(P.PlantParams)}
    assert "cross_up" in fields
    assert not [f for f in fields if f.endswith("_dn") or "cross_down" in f]


# --------------------------------------------------------------- torque identities
def test_six_step_equilibrium_is_exactly_the_static_curve():
    """If tau_drive's zero is not KV*V*eff*duty6(cmd, cross_up) then the static and dynamic layers have
    drifted apart and every static criterion is luck."""
    for cfg in (M.PROP, M.BARE):
        pl = P.MeasuredPlant(P.preset(cfg))
        for cmd in (540, 600, 680, 800):
            d = pl.duty_6step(cmd)
            w_eq = pl.p.kv * pl.p.v_supply * pl.p.eff * d
            assert abs(pl.tau_6step(d, w_eq)) < 1e-9, (cfg, cmd, pl.tau_6step(d, w_eq))
            assert abs(w_eq - pl.sixstep_rpm(cmd)) < 1e-9


def test_the_brake_is_the_drive_law_with_the_duty_removed():
    """tau_drive(cmd=0, w) == -tau_short_circuit(w) EXACTLY, for either direction of rotation."""
    pl = P.MeasuredPlant(P.preset(M.BARE))
    pl._zero_s = 99.0          # past release + engage: the brake is biting
    for w in (-3000.0, -800.0, 1800.0, 3100.0):     # all above the measured brake cut-out
        pl.rpm = w
        assert pl.tau_drive(0.0, w) == pytest.approx(-pl.tau_short_circuit(w), abs=1e-15)
        assert pl.tau_6step(0.0, w) == pytest.approx(-pl.tau_short_circuit(w), abs=1e-15)
        assert pl.tau_6step(0.0, w, direction=-1.0) == pytest.approx(-pl.tau_short_circuit(w),
                                                                     abs=1e-15)
    # ...and BELOW the measured cut-out the brake is gated OFF as a STATE, so the identity above is
    # untouched while the plant coasts on drag alone (see plant.brake_state).
    pl.rpm = 0.5 * M.BRAKE_CUTOUT_RPM
    assert pl.brake_state(0.0) == "released"
    assert pl.tau_drive(0.0, pl.rpm) == 0.0
    assert pl.tau_short_circuit(pl.rpm) > 0.0       # the LAW is unchanged; only the state gates it


def test_tau_short_circuit_is_the_textbook_asynchronous_brake():
    """K_T^2*w*R/(R^2+(pp*w*L)^2): peaks at w = R/(pp*L) with magnitude K_T^2*w_peak/(2R). No fitted
    exponent, no shaping term — a free exponent improved the six-decay fit by only ~17 % and was
    rejected as unjustified."""
    pl = P.MeasuredPlant(P.preset(M.BARE))
    p = pl.p
    for rpm in (100.0, 1200.0, 2400.0, 4000.0):
        w = rpm * P.RAD
        want = p.k_t ** 2 * w * p.r_total / (p.r_total ** 2 + (p.pole_pairs * w * p.l_phase) ** 2)
        assert pl.tau_short_circuit(rpm) == pytest.approx(want, rel=1e-12)
    w_peak = M.brake_peak_rpm(p.r_total, p.l_phase)
    assert pl.tau_short_circuit(w_peak) == pytest.approx(
        M.brake_peak_torque(p.r_total, p.l_phase), rel=1e-9)
    assert pl.tau_short_circuit(w_peak) > pl.tau_short_circuit(w_peak * 0.5)
    assert pl.tau_short_circuit(w_peak) > pl.tau_short_circuit(w_peak * 2.0)   # falls off BOTH sides
    assert pl.tau_short_circuit(-500.0) == pytest.approx(-pl.tau_short_circuit(500.0))


def test_the_brake_engagement_is_delayed_per_cut_mode():
    pl = P.MeasuredPlant(P.preset(M.BARE), cut_mode="disarm")
    pl.reset(3100.0, 0.0)
    assert pl.brake_state(0.0) == "driving"           # still driving through CUT_RELEASE_S
    pl._zero_s = M.CUT_RELEASE_S["disarm"] + 0.01
    assert pl.brake_state(0.0) == "released"          # drive gone, brake not yet biting
    assert pl.tau_drive(0.0, 3100.0) == 0.0
    pl._zero_s = M.CUT_RELEASE_S["disarm"] + M.BRAKE_ENGAGE_S["disarm"] + 0.01
    assert pl.brake_state(0.0) == "braking"
    assert pl.tau_drive(0.0, 3100.0) < 0.0
    for mode in ("disarm", "deadman", "zero_held"):
        assert mode in M.CUT_RELEASE_S and mode in M.BRAKE_ENGAGE_S


def test_zero_throttle_stops_3100_rpm_within_300ms_of_engagement():
    pl = P.MeasuredPlant(P.preset(M.BARE), cut_mode="disarm")
    pl.reset(3100.0, 0.0)
    t_engage = M.CUT_RELEASE_S["disarm"] + M.BRAKE_ENGAGE_S["disarm"]
    t, dt = 0.0, 5e-4
    while t < t_engage + 0.30:
        pl.step(0.0, dt)
        t += dt
    assert abs(pl.rpm) < 40.0, f"still at {pl.rpm:.0f} rpm 0.30 s after the brake engaged"


# --------------------------------------------------------------- sensor + sine
def test_tele_reports_the_floor_and_the_startup_silence():
    pl = P.MeasuredPlant(P.preset(M.BARE))
    assert pl.tele(0.0, t=0.0) == 0.0                       # no frame has arrived yet
    assert pl.tele(0.0, t=1.0) == M.TELE_FLOOR_MECH          # the 182 rpm lie
    assert pl.tele(50.0, t=1.0) == M.TELE_FLOOR_MECH
    assert pl.tele(1500.0, t=1.0) == 1500.0                  # honest once it is above the floor
    assert pl.tele(-1500.0, t=1.0) == 1500.0
    assert M.PROVENANCE["TELE_FLOOR_MECH"]["mech_independent"] is True


def test_the_sine_region_declines_to_be_a_speed_source():
    pl = P.MeasuredPlant(P.preset(M.BARE))
    for cmd in (60, 120, 180, 200):
        assert not pl.coherent(cmd)
        assert pl.steady(cmd)[0] is None
        assert pl.steady(cmd)[1] == "incoherent"
    for cmd in (240, 330, 450, 510):
        assert pl.coherent(cmd)
        assert pl.steady(cmd)[0] == pytest.approx(pl.sine_field(cmd))
    # the measured dispersion is exposed as DATA, not smoothed into the mean
    assert min(v for c, v in pl.p.sine_spread.items() if 60 <= c <= 210) >= 90.0
    assert max(pl.p.sine_reversals.values()) >= 3
    assert pl.spread_at(120) == M.SINE_SPREAD_TABLE_BARE[120]
    assert pl.reversals_at(90) == M.SINE_REVERSALS_BARE[90]


# --------------------------------------------------------------- transferability
def test_water_changes_the_drag_callable_and_nothing_else():
    """THE actual deliverable. `drag` is the only thing a load change may touch."""
    import dataclasses
    air = P.preset(M.PROP)
    k = 4.0e-9
    water = P.with_drag(air, lambda rpm: air.drag_ref(rpm) + k * rpm * rpm, name="water")
    a = {n: v for n, v in dataclasses.asdict(air).items() if n not in ("drag", "name")}
    w = {n: v for n, v in dataclasses.asdict(water).items() if n not in ("drag", "name")}
    assert a == w, "swapping drag changed something else"
    # (d) the brake law's fitted parameters are untouched
    assert water.l_phase == air.l_phase and water.j == air.j
    assert water.cut_release_s == air.cut_release_s
    # (a) the plant runs unchanged, (c) steady 6-step speed drops monotonically
    prev = None
    for cmd in (600, 640, 680):
        air_rpm = P.MeasuredPlant(air).steady(cmd)[0]
        pw = P.MeasuredPlant(water)
        pw.reset(0.0, 0.0)
        pw.settle(cmd, 8.0)
        assert pw.rpm < air_rpm, (cmd, pw.rpm, air_rpm)
        drop = air_rpm - pw.rpm
        if prev is not None:
            assert drop > prev            # the deficit grows with speed, as w^2 drag must
        prev = drop
    # and the brake still behaves in water: same law, same parameters
    pw = P.MeasuredPlant(water, cut_mode="disarm")
    pw.reset(2200.0, 0.0)
    assert pw.tau_short_circuit(1500.0) == pytest.approx(
        P.MeasuredPlant(air).tau_short_circuit(1500.0), rel=1e-12)
    # `drag_ref` belongs to the duty6 CALIBRATION, so it must NOT move with the load
    assert water.drag_ref is air.drag_ref


def test_step_converges_to_steady():
    """THE TEST THAT SHOULD HAVE EXISTED. `duty6` was fitted on a LOADED rotor, so the drive law
    already contains drag; the first version subtracted `tau_drag` again and the coupled ODE settled
    12.2 % BELOW `steady()` at PROP cmd 543 while the module docstring claimed it could not.

    It was invisible because `test_six_step_equilibrium_is_exactly_the_static_curve` inspects
    `tau_6step` WITHOUT drag and every static test calls `steady()` instead of integrating. This one
    integrates."""
    for cfg, cmds in ((M.PROP, (543, 612, 680, 750)), (M.BARE, (555, 570, 600, 700))):
        pl = P.MeasuredPlant(P.preset(cfg))
        for cmd in cmds:
            target = pl.steady(cmd)[0]
            pl.reset(0.0, 0.0)
            got = pl.settle(cmd, 6.0)
            assert got == pytest.approx(target, rel=2e-4), (
                f"{cfg} cmd {cmd}: step() settles at {got:.2f}, steady() says {target:.2f} "
                f"({100 * (got / target - 1):+.2f} %) — drag is being counted twice")
        # and starting from ABOVE converges down to the same fixed point
        pl.reset(3000.0, 0.0)
        assert pl.settle(cmds[0], 6.0) == pytest.approx(pl.steady(cmds[0])[0], rel=2e-4)


def test_the_duty_offset_is_what_makes_that_work_and_is_not_zero():
    """The correction is real and sizeable — 12-16 % of duty over the measured 6-step commands. If it
    were zero the drag double-count would be back, and it was exactly 12.2 % of duty at PROP cmd 543
    that the coupled ODE came out low by before this existed."""
    pl = P.MeasuredPlant(P.preset(M.PROP))
    for cmd in (543, 612, 680):
        frac = pl.duty_ref_offset(cmd) / pl.duty_6step(cmd)
        assert 0.08 < frac < 0.22, (cmd, frac)
        assert pl.electrical_duty(cmd) > pl.duty_6step(cmd)
    # it is the DRAG that produces it: a drag-free reference means no offset and no correction
    free = P.replace(P.preset(M.PROP), drag_ref=lambda rpm: 0.0)
    assert P.MeasuredPlant(free).duty_ref_offset(612) == 0.0


def test_nothing_exceeds_the_no_load_top():
    """C2: `duty6` is an affine fit to a modulator that cannot exceed full modulation. Unclamped,
    `sixstep_rpm` grew linearly for ever — +15 % over the no-load top at cmd 900 and +42 % at 1000,
    INSIDE this codebase's own tmax=800 envelope, while the sibling `MotorModel.rpm_6step` clamped."""
    for cfg in (M.PROP, M.BARE):
        pl = P.MeasuredPlant(P.preset(cfg))
        top = pl.p.no_load_top
        for cmd in (700, 800, 840, 900, 1000, 2000):
            assert pl.duty_6step(cmd) <= 1.0
            assert pl.electrical_duty(cmd) <= 1.0
            assert pl.sixstep_rpm(cmd) <= top + 1e-9, (cfg, cmd)
            assert pl.steady(cmd)[0] <= top + 1e-9
            # and the two layers of the same architecture must now agree
            assert pl.sixstep_rpm(cmd) == pytest.approx(pl.motor.rpm_6step(cmd), rel=1e-9)
        # the settled speed cannot exceed it either
        pl.reset(0.0, 0.0)
        assert pl.settle(1000, 4.0) <= top + 1e-9


def test_step_is_stable_at_any_dt_and_rejects_a_nan_command():
    """MINORs. Explicit Euler on this plant is stable only well inside J*R/K_T^2 (~12 ms), and
    `step(cmd, dt)` is the public API: step(600, 0.5) used to overshoot to -29.6 rpm, step(600, 10.0)
    diverged to 28513 rpm, and a NaN command silently became a large PLUGGING command that returned a
    plausible finite 286.6 rpm."""
    pl = P.MeasuredPlant(P.preset(M.BARE))
    target = pl.steady(600)[0]
    for dt in (0.001, 0.02, 0.1, 0.5, 1.0, 2.0, 10.0):
        pl.reset(0.0, 0.0)
        for _ in range(int(8.0 / dt) + 1):
            pl.step(600, dt)
        assert 0.0 <= pl.rpm <= 1.05 * target, f"dt={dt}: settled at {pl.rpm:.1f}"
        assert pl.rpm == pytest.approx(target, rel=0.01), f"dt={dt}"
    pl.reset(1000.0, 0.0)
    for bad in (float("nan"), float("inf"), float("-inf")):
        with pytest.raises(ValueError):
            pl.step(bad, 0.01)
    assert pl.rpm == 1000.0, "a rejected command must not have advanced the state"


def test_plant_params_is_frozen_and_variants_go_through_replace():
    """The transferability guarantee is only a property of the type if the type cannot be mutated
    behind `with_drag`'s back. Three call sites used to do `p.l_phase = ...` in place."""
    import dataclasses
    p = P.preset(M.BARE)
    with pytest.raises(dataclasses.FrozenInstanceError):
        p.l_phase = 1.0
    with pytest.raises(dataclasses.FrozenInstanceError):
        p.drag = None
    assert P.with_cross_up(p, 34.0).cross_up == 34.0 and p.cross_up == 36.0
    assert P.with_brake(p, l_phase=1e-3).l_phase == 1e-3 and p.l_phase != 1e-3
    assert P.with_crossover_clamp(p, 0.5).duty_crossover_clamp == 0.5


def test_drag_table_prop_falls_with_speed_and_says_so():
    """The honest note. This table cannot be prop drag, and the code must not pretend otherwise."""
    taus = [t for _rpm, t in M.DRAG_TABLE_PROP]
    assert taus == sorted(taus, reverse=True), "the table is supposed to FALL with speed"
    note = M.PROVENANCE["DRAG_TABLE_PROP"]["note"].lower()
    assert "not prop drag" in note and "unmeasured" in note
    src = open(os.path.join(HOST, "pico_esc", "measured.py")).read()
    assert "no drag law permits" in src


# --------------------------------------------------------------- anti-regression
def test_the_five_fitted_constants_are_gone():
    """`grep -rn "<the five names>" host/` must return NOTHING.

    The names are assembled from pieces at RUNTIME on purpose: writing them out here would put them
    into this file (and into its .pyc), and the whole point of the check is that they appear nowhere
    under host/ at all — including in the test that looks for them."""
    pieces = [("tau", "sine"), ("tau", "line"), ("cross", "mech"), ("zeta", "sine"),
              ("cross", "delay")]
    pattern = "\\|".join("_".join(p) for p in pieces)
    out = subprocess.run(["grep", "-rn", pattern, HOST], capture_output=True, text=True)
    assert out.returncode == 1, f"fitted-lag constants still present:\n{out.stdout}"


def test_the_regression_pinned_sim_is_untouched():
    for path in ("host/pico_esc/sim.py", "host/tests/data/posctl_move_d90_s1234.golden.csv"):
        out = subprocess.run(["git", "diff", "--stat", "66ff9ef", "--", path],
                             cwd=REPO, capture_output=True, text=True)
        assert out.stdout.strip() == "", f"{path} must not change:\n{out.stdout}"


def test_mech_time_constant_is_far_smaller_than_the_loop_sees():
    """8.3 ms at R=0.7, 14.2 ms at the measured R=1.20, against the ~37 ms the loop sees. The residual
    is the ESC's command shaping, not inertia — which is why no lag parameter exists."""
    assert M.mech_time_constant_ms(M.J_PROP, 0.7) == pytest.approx(8.3, abs=0.2)
    assert M.mech_time_constant_ms(M.J_PROP, M.R_TOTAL) == pytest.approx(14.2, abs=0.2)
    assert M.mech_time_constant_ms(M.J_PROP, M.R_TOTAL) < 37.0 / 2


def test_r_total_band_propagates_into_l_but_not_into_j():
    """L scales with R at a fixed brake-peak location, so the +-0.26 ohm band is a real uncertainty on
    it. J does NOT move with R: it comes from the free-coast window, which has no electrical term."""
    lo, hi = M.R_TOTAL - M.R_TOTAL_TOL, M.R_TOTAL + M.R_TOTAL_TOL
    l_lo = M.L_PHASE_FITTED * lo / M.R_TOTAL
    l_hi = M.L_PHASE_FITTED * hi / M.R_TOTAL
    assert l_lo < M.L_PHASE_FITTED < l_hi
    assert l_hi / l_lo == pytest.approx(hi / lo, rel=1e-9)
    # the peak LOCATION is what the fit pinned, so it does not move with R; the MAGNITUDE does
    assert M.brake_peak_rpm(lo, l_lo) == pytest.approx(M.brake_peak_rpm(hi, l_hi), rel=1e-9)
    assert M.brake_peak_torque(lo, l_lo) > M.brake_peak_torque(hi, l_hi)
    assert 0.4e-3 < M.L_PHASE_FITTED < 3.0e-3, "a fitted L outside ~0.4-3 mH is not a BLDC phase"
    # J from the free-coast window, computed independently of anything electrical
    j = M.drag_at(2757.0) / (M.FREE_DECEL_BARE_RPM_S * P.RAD)
    assert j == pytest.approx(M.J_BARE_FITTED, rel=0.01)
    j_prop = M.drag_at(1877.0) / (M.FREE_DECEL_PROP_RPM_S * P.RAD)
    assert j_prop == pytest.approx(M.J_PROP, rel=0.05), "the same arithmetic must recover measured J"
