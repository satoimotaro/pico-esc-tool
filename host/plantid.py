#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""plantid — identify and validate the measured plant, OFFLINE. Never opens a serial port.

Three jobs, all replaying fixtures already committed in `host/tests/data/`:

  `provenance` — print the parameter inventory with its mechanical configuration per row, so it is
                 obvious at a glance which numbers came from the rotor WITH the propeller and which
                 came from the bare shaft. Mixing them is the failure mode this whole file guards.
  `fit`        — the unmeasured parameters, each against the ONE phase of the decay that isolates it:
                 `J` from the FREE-COAST window (the only phase with no ESC torque, so J = drag/alpha
                 straight off the measured drag table), then `(L, BRAKE_CUTOUT_RPM)` from the BRAKE
                 PHASE — revolutions travelled and duration, both measured FROM ENGAGEMENT. That
                 distinction is the whole point: an earlier version fitted against time-to-stop
                 measured from t = 0, of which 0.31 s of ~0.38 s is fixed delay, so the metric passed
                 at 2x brake-torque error and L was not identified by anything.
  `validate`   — the OUT-OF-SAMPLE table. Steady-state error against both static curves at both
                 cross_up values (including the deliberately-wrong one, which must blow up), the
                 telemetry model, the six bare-shaft decays, and the three prop-on decays which use
                 the bare-shaft-fitted (L) with the prop-on (J, drag) and are therefore a genuine
                 out-of-sample test of the brake law.

WHAT THIS TOOL WILL NOT DO. It will not fit a lag. `sim_obs`'s old 0.18 s sine lag came from fitting a
first-order rise to the sysid step 65 -> 407, where the rotor wanders -29..249 rpm around a 145 rpm
forced field — the "improvement" was fitting cogging dispersion. Those two sine-region steps are
reported here as `n/a (cogging-dominated)` and are used to justify NOT adding a parameter.

  python3 host/plantid.py --dry-run
  python3 host/plantid.py provenance
  python3 host/plantid.py fit --config bare_shaft_350kv_20260730
  python3 host/plantid.py validate --all
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from pico_esc import measured as M                       # noqa: E402
from pico_esc.plant import (RAD, MeasuredPlant, preset, with_brake,   # noqa: E402
                            with_cross_up, with_drag)
from worldmodel import fit_stats                         # noqa: E402

DATA = os.path.join(HERE, "tests", "data")
MANIFEST = os.path.join(DATA, "plant_fixtures.json")


# ---------------------------------------------------------------- fixtures
def manifest():
    with open(MANIFEST) as fh:
        return json.load(fh)


def _rows(name, key):
    by = {}
    with open(os.path.join(DATA, name)) as fh:
        for r in csv.DictReader(fh):
            by.setdefault(r[key], []).append(r)
    return by


def decay_traj(rows, win=0.020, step=0.005):
    """(t, |rpm|) from ENCODER POSITION by sliding least-squares.

    Differentiating `encv_rpm` would be differentiating a device-side windowed average, which lags
    exactly where the signal moves fastest. And differencing adjacent position samples is hopeless
    here: the host timestamps jitter +-0.5 ms on a 1.4 ms poll, which is a +-35 % speed error. A
    sliding linear fit of position over 20 ms uses every sample at once and averages the jitter out.
    """
    a0 = int(rows[0]["accum"])
    pts = [(float(r["t"]), (int(r["accum"]) - a0) / 4096.0 * 60.0) for r in rows]
    out, t, t1 = [], pts[0][0], pts[-1][0]
    while t + win <= t1:
        w = [p for p in pts if t <= p[0] <= t + win]
        if len(w) >= 6:
            n = len(w)
            tm = sum(p[0] for p in w) / n
            ym = sum(p[1] for p in w) / n
            sxx = sum((p[0] - tm) ** 2 for p in w)
            sxy = sum((p[0] - tm) * (p[1] - ym) for p in w)
            if sxx > 0:
                out.append((tm, abs(sxy / sxx)))
        t += step
    return out


def decel_peak(series):
    """(rpm at the deceleration peak, peak d|w|/dt). Median-filtered: the raw derivative carries
    +-40 krpm/s timestamp-jitter spikes that would otherwise BE the peak.

    USE WITH CARE AS A MODEL METRIC. The model's brake torque peaks at engagement (the rotor is already
    past the torque peak when the brake bites), so the "model peak rpm" this reports is set by the
    sampling interval it is handed, not by the model. It is kept because the MEASURED peak location is
    the finding that resolved the load-shift question, not because it validates a torque."""
    raw = [((series[i][0] + series[i + 1][0]) / 2, (series[i][1] + series[i + 1][1]) / 2,
            (series[i][1] - series[i + 1][1]) / (series[i + 1][0] - series[i][0]))
           for i in range(len(series) - 1)]
    med = []
    for i, (t, w, _) in enumerate(raw):
        seg = sorted(d for _, _, d in raw[max(0, i - 2):i + 3])
        med.append((t, w, seg[len(seg) // 2]))
    pk = max(med, key=lambda d: d[2])
    return pk[1], pk[2], pk[0]


def time_to_stop(series, thresh=40.0, after=0.02):
    return next((t for t, w in series if w < thresh and t > after), None)


def two_segment(series, t_end):
    """Least-squares (flat, then linear) split of the pre-brake region -> (t_release, w0, decel)."""
    pre = [(t, v) for t, v in series if t <= t_end]
    best = None
    for i in range(1, max(2, len(pre) - 3)):
        tr = pre[i][0]
        w0 = sum(v for _, v in pre[:i]) / i
        den = sum((t - tr) ** 2 for t, _ in pre[i:])
        if den <= 0:
            continue
        a = sum((w0 - v) * (t - tr) for t, v in pre[i:]) / den
        if a <= 0:
            continue
        e = (sum((v - w0) ** 2 for _, v in pre[:i])
             + sum((v - (w0 - a * (t - tr))) ** 2 for t, v in pre[i:]))
        if best is None or e < best[0]:
            best = (e, tr, w0, a)
    return best[1:] if best else (pre[0][0], pre[0][1], float("nan"))


def brake_phase(rows, t_on, stop_rpm=40.0):
    """The metric that actually isolates the brake: revolutions travelled and duration, both measured
    FROM ENGAGEMENT, straight off the raw encoder POSITION.

    Time-to-stop measured from t = 0 is ~72 % fixed delay (0.307 s of 0.406 s on `disarm`), so it stays
    within 13 % while the brake torque is out by 2x. Revolutions come from position differences with no
    differentiation and no smoothing at all."""
    a0 = int(rows[0]["accum"])
    pts = [(float(r["t"]), (int(r["accum"]) - a0) / 4096.0) for r in rows]
    tj = decay_traj(rows)
    t_stop = next((t for t, w in tj if t > t_on and w < stop_rpm), None)
    if t_stop is None:
        return None
    rev_on = next(p for p in pts if p[0] >= t_on)[1]
    rev_stop = next(p for p in pts if p[0] >= t_stop)[1]
    return {"t_on": t_on, "t_stop": t_stop, "dur": t_stop - t_on,
            "rev": abs(rev_stop - rev_on), "w_on": next(w for t, w in tj if t >= t_on)}


def brake_onset(series):
    """Time the brake bites — walk back from the deceleration peak to where it is still < 25 % of it."""
    raw = [((series[i][0] + series[i + 1][0]) / 2,
            (series[i][1] - series[i + 1][1]) / (series[i + 1][0] - series[i][0]))
           for i in range(len(series) - 1)]
    med = []
    for i, (t, _) in enumerate(raw):
        seg = sorted(d for _, d in raw[max(0, i - 2):i + 3])
        med.append((t, seg[len(seg) // 2]))
    ipk = max(range(len(med)), key=lambda i: med[i][1])
    i = ipk
    while i > 0 and med[i][1] > 0.25 * med[ipk][1]:
        i -= 1
    return med[i][0]


def decays(which="bare"):
    """[(label, cut_mode, mech_config, series)] for every committed decay fixture."""
    out = []
    for name, meta in manifest()["fixtures"].items():
        if meta["tool"] != "coastdown" or meta["role"] != which:
            continue
        for k, rows in _rows(name, meta["run_key"]).items():
            mode = k if meta["run_key"] == "mode" else meta["cut"]
            out.append((f"{name.split('_2026')[0]}/{k}", mode, meta["mech_config"],
                        decay_traj(rows)))
    return out


def decay_phases(which="bare"):
    """[(label, cut_mode, mech_config, brake_phase dict)] — the brake-isolating metrics per fixture."""
    out = []
    for name, meta in manifest()["fixtures"].items():
        if meta["tool"] != "coastdown" or meta["role"] != which:
            continue
        for k, rows in _rows(name, meta["run_key"]).items():
            mode = k if meta["run_key"] == "mode" else meta["cut"]
            ph = brake_phase(rows, brake_onset(decay_traj(rows)))
            if ph:
                out.append((f"{name.split('_2026')[0]}/{k}", mode, meta["mech_config"], ph))
    return out


# ---------------------------------------------------------------- fit
def sim_brake_phase(cfg, cut_mode, w_on, l_phase, j, cutout, stop_rpm=40.0, dt=2e-4):
    """Integrate the BRAKE PHASE ONLY, from engagement. Returns (duration, revolutions)."""
    p = with_brake(preset(cfg), l_phase=l_phase, j=j, brake_cutout_rpm=cutout)
    pl = MeasuredPlant(p, cut_mode=cut_mode)
    pl.reset(w_on, 0.0)
    # place the state past both measured delays so the brake is already engaged: this metric is about
    # the brake, and mixing the delays back in is exactly what made the old metric uninformative.
    pl._zero_s = p.cut_release_s[cut_mode] + p.brake_engage_s[cut_mode] + 1e-9
    t, rev = 0.0, 0.0
    while t < 2.0 and abs(pl.rpm) >= stop_rpm:
        prev = pl.rpm
        pl._step_once(0.0, dt)
        rev += 0.5 * (abs(prev) + abs(pl.rpm)) / 60.0 * dt
        t += dt
    return t, rev


def _sim_decay(cfg, cut_mode, w0, t_end, l_phase, j, dt=5e-4):
    """The WHOLE decay from the cut, delays included. Only used to print the old delay-dominated
    time-to-stop alongside the brake-phase metric, so the difference is visible."""
    pl = MeasuredPlant(with_brake(preset(cfg), l_phase=l_phase, j=j), cut_mode=cut_mode)
    pl.reset(w0, 0.0)
    out, t = [], 0.0
    while t < t_end:
        pl.step(0.0, dt)
        t += dt
        out.append((t, abs(pl.rpm)))
    return out


def j_from_free_coast(cfg):
    """J from the FREE-COAST window: the one phase of the decay with no ESC torque in it, so
    `J = tau_drag(w_mid)/alpha_free` with the measured drag table and nothing fitted.

    This is what actually identifies J. The brake fit's cost function never constrained the brake
    torque SCALE, so an earlier claim that J came from the brake-peak magnitude was simply wrong."""
    if cfg == M.BARE:
        alpha, w_mid = M.FREE_DECEL_BARE_RPM_S, 2757.0
    else:
        alpha, w_mid = M.FREE_DECEL_PROP_RPM_S, 1877.0
    return M.drag_at(w_mid) / (alpha * RAD), alpha, w_mid


def _phase_cost(sets, l_phase, j, cutout):
    tot = 0.0
    for _lbl, mode, cfg, ph in sets:
        d, r = sim_brake_phase(cfg, mode, ph["w_on"], l_phase, j, cutout)
        tot += ((d - ph["dur"]) / ph["dur"]) ** 2 + ((r - ph["rev"]) / ph["rev"]) ** 2
    return tot


def fit(cfg, verbose=True):
    """J from the free coast, then (L, BRAKE_CUTOUT_RPM) from the brake phase. R pinned throughout."""
    j, alpha, w_mid = j_from_free_coast(cfg)
    sets = [x for x in decay_phases("bare") if x[1] == "disarm"]
    best = None
    for i in range(46):
        l = 3.0e-4 * (1.07 ** i)
        for cut in range(150, 1150, 50):
            c = _phase_cost(sets, l, j, float(cut))
            if best is None or c < best[0]:
                best = (c, l, float(cut))
    for _ in range(4):
        c, l, cut = best
        for kl in (-3, -2, -1, 0, 1, 2, 3):
            for kc in (-3, -2, -1, 0, 1, 2, 3):
                c2 = _phase_cost(sets, l * (1 + 0.025 * kl), j, cut * (1 + 0.03 * kc))
                if c2 < best[0]:
                    best = (c2, l * (1 + 0.025 * kl), cut * (1 + 0.03 * kc))
    c, l, cut = best
    if verbose:
        print(f"# R_total PINNED at {M.R_TOTAL} ohm throughout. Nothing below is fitted to a metric")
        print("# that contains the two fixed cut delays.\n")
        print("## J — from the FREE-COAST window (no ESC torque in it at all)")
        print(f"   tau_drag({w_mid:.0f} rpm) = {M.drag_at(w_mid) * 1e3:.3f} mN*m  /  "
              f"alpha_free = {alpha:.0f} rpm/s   ->  J = {j:.4e} kg*m^2")
        if cfg == M.PROP:
            print(f"   cross-check: this is the configuration where J WAS measured (coastdown.py): "
                  f"{M.J_PROP:.3e}, i.e. {100 * (j / M.J_PROP - 1):+.1f} %")
        print(f"\n## (L, brake cut-out) — from the BRAKE PHASE of the {len(sets)} `disarm` decays")
        print("   metric: revolutions travelled AND duration, both measured FROM ENGAGEMENT")
        print(f"   L = {l * 1e3:.3f} mH   BRAKE_CUTOUT_RPM = {cut:.0f} rpm   cost = {c:.4f}")
        print(f"   short-circuit peak {M.brake_peak_torque(M.R_TOTAL, l) * 1e3:.1f} mN*m at "
              f"{M.brake_peak_rpm(M.R_TOTAL, l):.0f} rpm")
        print(f"   for measured.py: L_PHASE_FITTED = {l:.4g}   BRAKE_CUTOUT_RPM = {cut:.0f}")
        print("\n# R_total +- band propagated (L scales with R at a fixed peak location; J does NOT")
        print("#   move with R, because the free-coast window has no electrical term in it):")
        for r in (M.R_TOTAL - M.R_TOTAL_TOL, M.R_TOTAL, M.R_TOTAL + M.R_TOTAL_TOL):
            lr = l * r / M.R_TOTAL
            print(f"    R = {r:.2f} ohm -> L = {lr * 1e3:.3f} mH, brake peak "
                  f"{M.brake_peak_torque(r, lr) * 1e3:.1f} mN*m at "
                  f"{M.brake_peak_rpm(r, lr):.0f} rpm, J*R/K_T^2 = "
                  f"{M.mech_time_constant_ms(j, r):.1f} ms")
        print("\n# L IS A PREDICTION, NOT A MEASUREMENT. Check it with an LCR meter across two phases")
        print(f"#   (expect ~2L line-to-line, i.e. ~{2 * l * 1e3:.1f} mH) before reusing it anywhere.")
        print("# The BRAKE TORQUE SCALE is in-sample for `disarm` only. The pointwise deceleration")
        print("#   residual puts the peak ~1.5x higher than this L does, and the `deadman`/`zero_held`")
        print("#   cut modes brake 2-3x harder still (measured.BRAKE_PHASE_S). See docs.")
        print("# J_bare is IDENTIFIED, not measured. Only J_PROP was ever measured directly.")
    return l, j, cut


# ---------------------------------------------------------------- validate
def val_static(rows_out):
    print("\n## static curve — steady() = max(sine field, KV*V*eff*duty6(cmd, cross_up))")
    print(f"  {'dataset':30s} {'cross_up':>8} {'cmd':>5} {'measured':>9} {'model':>9} {'err':>8}")
    cases = [
        (M.PROP, "350kv_postfix_sysid_raw (prop)", 34.0, [(543, 794.4), (612, 1510.4), (680, 2212.7)]),
        (M.BARE, "sinesweep_fold_noprop (bare)", 36.0, [(555, 600.556)]),
        (M.BARE, "sinesweep_noprop up (bare)", 36.0, [(570, 750.363), (600, 1069.269)]),
        (M.BARE, "sinesweep_noprop down (bare)", 36.0, [(570, 751.698), (600, 1064.109)]),
        (M.PROP, "  ...with the WRONG cross_up", 36.0, [(543, 794.4), (612, 1510.4), (680, 2212.7)]),
        (M.BARE, "  ...with the WRONG cross_up", 34.0, [(570, 750.363), (600, 1069.269)]),
    ]
    for cfg, label, cu, pts in cases:
        pl = MeasuredPlant(with_cross_up(preset(cfg), cu))
        for cmd, meas in pts:
            r, _reg = pl.steady(cmd)
            e = 100.0 * (r - meas) / meas
            rows_out.append((label, cmd, meas, r, e))
            print(f"  {label:30s} {cu:8.0f} {cmd:5d} {meas:9.1f} {r:9.1f} {e:+7.2f}%")
    for cfg, want in ((M.BARE, (510, 525)), (M.PROP, (470, 545))):
        pl = MeasuredPlant(preset(cfg))
        print(f"  seam cmd ({cfg}) = {pl.seam_cmd():.1f}  (criterion {want[0]}..{want[1]}) "
              f"— no hysteresis parameter exists")


def val_tele():
    print("\n## telemetry sensor model — tele = max(TELE_FLOOR_MECH, |rpm|)")
    pl = MeasuredPlant(preset(M.BARE))
    for name in ("sinesweep_noprop_20260730_033014.csv", "sinesweep_fold_noprop_20260730_033417.csv"):
        errs = []
        with open(os.path.join(DATA, name)) as fh:
            for r in csv.DictReader(fh):
                errs.append(abs(pl.tele(float(r["enc_mean"]), t=99.0) - float(r["tele_rpm"])))
        worst = max(errs)
        print(f"  {name[:44]:44s} n={len(errs):3d} max {worst:5.1f} rpm  "
              f"mean {sum(errs) / len(errs):5.1f} rpm")
    # startup: the model must also reproduce "no telemetry at all yet"
    for run, rows in sorted(_rows("steptrace_startup_noprop_20260730_033906.csv", "run").items()):
        errs = [abs(pl.tele(float(r["enc_rpm"]), t=float(r["t"])) - float(r["tele_rpm"]))
                for r in rows if float(r["t"]) < 0.5]
        print(f"  startup run {run}, t < 0.5 s          n={len(errs):3d} max {max(errs):5.1f} rpm "
              f"(TELE_LIVE_S = {M.TELE_LIVE_S}s of silence, then the 182 rpm floor)")


def val_brake():
    print("\n## the rotor never freely coasts — brake replay")
    print("#  The metric is the BRAKE PHASE — revolutions travelled and duration, both measured FROM")
    print("#  ENGAGEMENT, straight off encoder position. Time-to-stop from t = 0 is ~72 % fixed delay")
    print("#  and stays inside 13 % while the brake torque is out by 2x, so it is reported for")
    print("#  continuity but proves nothing about the brake.")
    print(f"  {'fixture':26s} {'cut mode':10s} {'w_on':>6} {'dur meas/model':>16} {'err':>7} "
          f"{'rev meas/model':>15} {'err':>7} {'t_stop err':>10}")
    for role in ("bare", "prop"):
        for lbl, mode, cfg, ph in decay_phases(role):
            p = preset(cfg)
            d, r = sim_brake_phase(cfg, mode, ph["w_on"], p.l_phase, p.j, p.brake_cutout_rpm)
            # the old, delay-dominated metric, for comparison only
            sim = _sim_decay(cfg, mode, ph["w_on"], ph["t_stop"] + 0.25, p.l_phase, p.j)
            thin = [(t, v) for i, (t, v) in enumerate(sim) if i % 20 == 0]
            s_stop = time_to_stop(thin) or float("nan")
            tag = "" if role == "bare" else "  OUT OF SAMPLE"
            if mode != "disarm":
                tag = "  NOT MODELLED (mode torque)"
            print(f"  {lbl:26s} {mode:10s} {ph['w_on']:6.0f} {ph['dur']:7.3f}/{d:8.3f} "
                  f"{100 * (d - ph['dur']) / ph['dur']:+6.1f}% {ph['rev']:6.2f}/{r:8.2f} "
                  f"{100 * (r - ph['rev']) / ph['rev']:+6.1f}% "
                  f"{100 * (s_stop - ph['t_stop']) / ph['t_stop']:+9.1f}%{tag}")
    print("\n  THE MODE DEPENDENCE IS IN THE TORQUE, NOT ONLY THE DELAY. From the same ~2400 rpm the")
    print("  measured brake phase is "
          + " / ".join(f"{v:.3f} s (`{k}`)" for k, v in M.BRAKE_PHASE_S.items())
          + " — a systematic ~3x")
    print("  ordering. One short-circuit torque cannot be all three, so the model is fitted to `disarm`")
    print("  (the default, and the mode the prop-on decays used) and is simply wrong on the other two.")
    print("\n  measured cut delays vs the parameters in measured.py "
          "(coastdown.coast_start reports ~0.17/0.35/0.05):")
    for lbl, mode, cfg, obs in decays("bare") + decays("prop"):
        t_on = brake_onset(obs)
        t_rel, w0, a_free = two_segment(obs, t_on)
        print(f"    {lbl:26s} {mode:10s} release {t_rel:.3f}s (param "
              f"{M.CUT_RELEASE_S.get(mode, float('nan')):.3f})  engage {t_on - t_rel:.3f}s (param "
              f"{M.BRAKE_ENGAGE_S.get(mode, float('nan')):.3f})  free decel {a_free:.0f} rpm/s")


def val_downstep():
    print("\n## open-loop down-step (thrust 600 -> 480) — the forced-sine pull-out, not the brake")
    meas = []
    for run, rows in sorted(_rows("steptrace_down_noprop_20260730_033706.csv", "run").items()):
        rr = [(float(r["t"]), abs(float(r["enc_rpm"]))) for r in rows]
        tail = [v for t, v in rr if t > 1.0]
        fin = sum(tail) / len(tail)
        half = (rr[0][1] + fin) / 2.0
        th = next(t for t, v in rr if v <= half)
        meas.append((th, min(v for t, v in rr if t > 0.05), fin))
        print(f"  measured run {run}: t_half {th * 1e3:5.1f} ms  trough {meas[-1][1]:6.1f} rpm  "
              f"settles {fin:6.1f} rpm (forced field at cmd 480 = "
              f"{MeasuredPlant(preset(M.BARE)).sine_field(480):.1f})")
    pl = MeasuredPlant(preset(M.BARE))
    pl.reset(1048.0, 0.0)
    sim, dt, t = [], 2e-4, 0.0
    while t < 1.0:
        pl.step(480, dt)
        t += dt
        sim.append((t, abs(pl.rpm)))
    fin = sim[-1][1]
    half = (1048.0 + 175.0) / 2.0
    th = next(t for t, v in sim if v <= half)
    trough = min(v for t, v in sim if t > 0.005)
    mm = sum(x[0] for x in meas) / len(meas)
    print(f"  MODEL          : t_half {th * 1e3:5.1f} ms ({100 * (th - mm) / mm:+.0f}% vs measured)  "
          f"trough {trough:6.1f} rpm  settles {fin:6.1f} rpm")
    print("  NOTE: SINE_PULLOUT_NM was identified FROM this trace, so this row is a consistency")
    print("  check, not an out-of-sample one. The out-of-sample brake evidence is the three prop-on")
    print("  decays above, which reuse the bare-shaft-fitted L with the prop-on J and drag.")


def val_steps():
    print("\n## sysid step replay (prop-on, cross_up 34)")
    raw = os.path.join(HERE, "models", "350kv_postfix_sysid_raw.json")
    if not os.path.exists(raw):
        print("  (models/350kv_postfix_sysid_raw.json missing — skipped)")
        return
    steps = json.load(open(raw))["steps"]
    print(f"  {'step':14s} {'n':>4} {'VAF':>22} note")
    for s in steps:
        rows = [(t, r) for t, r, *_ in s["rows"] if r is not None]
        pl = MeasuredPlant(preset(M.PROP))
        pl.reset(rows[0][1], 0.0)
        sim, prev = [pl.rpm], rows[0][0]
        for t, _r in rows[1:]:
            # integrate at a fine step and only SAMPLE at the recorded 20 ms grid
            n = max(1, int((t - prev) / 5e-4))
            for _ in range(n):
                pl.step(s["target"], (t - prev) / n)
            sim.append(pl.rpm)
            prev = t
        real = [r for _t, r in rows]
        sine_region = max(abs(pl.steady(s["bias"])[0] or 0), abs(pl.steady(s["target"])[0] or 0)) < 500
        if sine_region:
            note = "n/a (cogging-dominated)"
            print(f"  {s['bias']:5d}->{s['target']:<7d} {len(rows):4d} {note:>22} "
                  f"rotor wanders {min(real):.0f}..{max(real):.0f} rpm around a "
                  f"{pl.sine_field(s['target']):.0f} rpm field")
        else:
            st = fit_stats(real, sim)
            print(f"  {s['bias']:5d}->{s['target']:<7d} {len(rows):4d} {st['vaf']:21.1f}% "
                  f"rmse {st['rmse']:.0f} rpm")
    # the crossing step's shape criteria
    st = next((s for s in steps if s["bias"] == 407 and s["target"] == 680), None)
    if st:
        real = [(t, r) for t, r, *_ in st["rows"] if r is not None]
        t95m = next(t for t, r in real if r >= 0.95 * 2212.7)
        pl = MeasuredPlant(preset(M.PROP))
        pl.reset(real[0][1], 0.0)
        sim, t, dt = [], 0.0, 5e-4
        while t < 1.5:
            pl.step(680, dt)
            t += dt
            sim.append((t, pl.rpm))
        t95s = next((t for t, r in sim if r >= 0.95 * 2212.7), float("nan"))
        plat = [r for t, r in sim if 0.15 <= t <= 0.38]
        print(f"  407->680 crossing: time to 95 % of 2213 rpm  measured {t95m:.3f} s  "
              f"model {t95s:.3f} s  ({100 * (t95s - t95m) / t95m:+.1f}%)")
        print(f"                     plateau over t=0.15..0.38  measured 360..415 rpm  "
              f"model {min(plat):.0f}..{max(plat):.0f} rpm")
    print(f"  J*R/K_T^2 = {M.mech_time_constant_ms(M.J_PROP, 0.7):.1f} ms at R=0.7 and "
          f"{M.mech_time_constant_ms(M.J_PROP, M.R_TOTAL):.1f} ms at R={M.R_TOTAL} — against the "
          f"~37 ms the loop sees.")
    print("  The residual is NOT inertia: it is the ESC's command shaping (the crossover slew and "
          "duty clamp).")


def val_sine():
    print("\n## forced sine (the regime telemetry cannot see)")
    pl = MeasuredPlant(preset(M.BARE))
    worst = (0.0, None)
    for name in ("sinesweep_noprop_20260730_033014.csv", "sinesweep_fold_noprop_20260730_033417.csv"):
        with open(os.path.join(DATA, name)) as fh:
            for r in csv.DictReader(fh):
                cmd, enc = int(r["cmd"]), abs(float(r["enc_mean"]))
                if not 270 <= cmd <= 510:
                    continue
                mr, _reg = pl.steady(cmd)
                e = abs(100.0 * (abs(mr) - enc) / enc)
                if e > worst[0]:
                    worst = (e, (name, cmd, enc, abs(mr)))
    n, c, enc, mr = worst[1]
    print(f"  cmd 270..510: worst |err| = {worst[0]:.1f}% (criterion <=30 %) at cmd {c} "
          f"(measured {enc:.0f}, model {mr:.0f})")
    lo = min(v for k, v in pl.p.sine_spread.items() if 60 <= k <= 210)
    print(f"  cmd 60..210: coherent(150) = {pl.coherent(150)}, steady(150) = {pl.steady(150)[0]} "
          f"— the plant declines to be a speed source there")
    print(f"               exposed cogging spread >= {lo:.0f} rpm (fixture range 92..234 rpm); "
          f"reversals up to {max(pl.p.sine_reversals.values())} per 3 s dwell")


def val_transfer():
    print("\n## transferability — water changes the drag callable and NOTHING else")
    air = preset(M.PROP)
    water = with_drag(air, lambda rpm: air.drag_ref(rpm) + 4.0e-9 * rpm * rpm, name="PROP+water_w2")
    import dataclasses
    a = {k: v for k, v in dataclasses.asdict(air).items() if k not in ("drag", "name")}
    w = {k: v for k, v in dataclasses.asdict(water).items() if k not in ("drag", "name")}
    print(f"  every parameter except `drag` identical: {a == w}")
    print(f"  `drag_ref` (the duty6 calibration's drag) unmoved: {water.drag_ref is air.drag_ref}")
    for cmd in (600, 680):
        ra = MeasuredPlant(air).steady(cmd)[0]
        pw = MeasuredPlant(water)
        pw.reset(0.0, 0.0)
        for _ in range(20000):
            pw.step(cmd, 5e-4)
        print(f"  cmd {cmd}: air steady {ra:.0f} rpm -> water settles {pw.rpm:.0f} rpm "
              f"({100 * (pw.rpm - ra) / ra:+.1f}%, monotonically slower)")


# ---------------------------------------------------------------- CLI
def cmd_provenance(_opts):
    # Build both presets FIRST: which parameters a preset reads is RECORDED from the code, so the
    # report below is what preset() actually touched rather than a list maintained beside it.
    preset(M.PROP)
    preset(M.BARE)
    print(M.provenance_table())
    print(f"\n# {len(M.PARAMS)} parameters. mech_config tells you which rotor each came from; "
          f"`indep` = OPEN means it is not established either way.")
    for cfg in (M.PROP, M.BARE):
        names = M.preset_param_names(cfg)
        borrowed = M.preset_borrowed(cfg)
        mixed = sorted(n for n in names
                       if M.PROVENANCE[n]["mech_config"] != cfg
                       and M.PROVENANCE[n]["mech_independent"] is not True
                       and n not in borrowed)
        print(f"# preset {cfg}: {len(names)} parameters, "
              f"{'no undeclared cross-config mixing' if not mixed else 'MIXES ' + ', '.join(mixed)}")
        for n, why in sorted(borrowed.items()):
            print(f"#     borrowed {n}: {why}")
    return 0


def cmd_fit(opts):
    l, j, cut = fit(opts.config)
    out = os.path.join(HERE, "models", "350kv_measured_plant.json")
    if opts.write:
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, "w") as fh:
            json.dump({"mech_config": opts.config, "r_total_pinned": M.R_TOTAL,
                       "l_phase_h": l, "j_kgm2": j, "brake_cutout_rpm": cut,
                       "brake_peak_rpm": M.brake_peak_rpm(M.R_TOTAL, l),
                       "identified_from": {"j": "free-coast window vs the measured drag table",
                                           "l+cutout": "brake-phase revolutions + duration, "
                                                       "`disarm` only"}}, fh, indent=2)
        print(f"\n# -> {out} (gitignored)")
    return 0


def cmd_validate(opts):
    print("# plantid validate — offline replay of host/tests/data/ fixtures. No serial port opened.")
    rows = []
    if opts.all or opts.static:
        val_static(rows)
    if opts.all or opts.tele:
        val_tele()
    if opts.all or opts.brake:
        val_brake()
        val_downstep()
    if opts.all or opts.steps:
        val_steps()
    if opts.all or opts.sine:
        val_sine()
    if opts.all or opts.transfer:
        val_transfer()
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="identify/validate the measured plant, offline")
    ap.add_argument("--dry-run", action="store_true",
                    help="print what each subcommand would replay and exit; touches nothing")
    ap.add_argument("--analyze", default=None,
                    help="re-report one fixture CSV (same spelling as the other host tools)")
    sub = ap.add_subparsers(dest="cmd")
    sub.add_parser("provenance")
    f = sub.add_parser("fit")
    f.add_argument("--config", default=M.BARE, choices=(M.BARE, M.PROP))
    f.add_argument("--write", action="store_true", help="also write models/350kv_measured_plant.json")
    v = sub.add_parser("validate")
    v.add_argument("--all", action="store_true")
    for name in ("static", "tele", "brake", "steps", "sine", "transfer"):
        v.add_argument(f"--{name}", action="store_true")
    opts = ap.parse_args(argv)

    if opts.dry_run:
        mf = manifest()
        print("# DRY RUN — plantid is OFFLINE ONLY: it opens fixtures, never a serial port.")
        print(f"#   fixtures: {len(mf['fixtures'])} files in host/tests/data/ "
              f"({sum(m['rows'] for m in mf['fixtures'].values())} rows)")
        print("#   provenance : print the inventory (no file reads)")
        print(f"#   fit        : J from the free-coast window, then (L, brake cut-out) from the BRAKE "
              f"PHASE of the 4 `disarm` decays; R pinned at {M.R_TOTAL} ohm")
        print("#   validate   : static / tele / brake / steps / sine / transfer replays")
        return 0

    if opts.analyze:
        rows = list(csv.DictReader(open(opts.analyze)))
        key = "mode" if "mode" in rows[0] else "run"
        by = {}
        for r in rows:
            by.setdefault(r[key], []).append(r)
        print(f"# {len(rows)} rows, {len(by)} run(s), key column '{key}'")
        for k, rr in by.items():
            if "accum" not in rr[0]:
                print(f"  {k}: not a decay fixture (no `accum` column)")
                continue
            s = decay_traj(rr)
            pk, dec, t = decel_peak(s)
            print(f"  {k}: plateau {max(v for _, v in s[:8]):.0f} rpm, decel peak {dec:.0f} rpm/s "
                  f"at {pk:.0f} rpm (t={t:.3f}s), stops at t={time_to_stop(s):.3f}s")
        return 0

    if not opts.cmd:
        ap.error("give a subcommand (provenance | fit | validate) or --dry-run")
    return {"provenance": cmd_provenance, "fit": cmd_fit, "validate": cmd_validate}[opts.cmd](opts)


if __name__ == "__main__":
    sys.exit(main())
