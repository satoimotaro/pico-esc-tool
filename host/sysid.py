#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""sysid — hands-off plant IDENTIFICATION + characterization for one ESC/motor.

velcal already measures the STATIC thrust->RPM curve; nothing on the host fits the DYNAMIC
plant. This tool fills that gap and runs the whole characterization unattended in one shot:

  1. pole-count ID   — spin at a safe mid thrust, compare the AS5600 mechanical RPM (encv, true)
                       against the firmware `tele` RPM (eRPM / configured pole-pairs). Since the
                       firmware pre-divides by the CONFIGURED pp (constants.POLE_PAIRS=7),
                       real_pp = 7 * tele_rpm / enc_rpm  (nearest integer).
  2. direction       — confirm +thrust -> +encv (sets enc_sign for the rest of the run).
  3. deadband        — from 0, step thrust up until the shaft sustains rotation (both directions):
                       the low-speed commutation floor.
  4. static curve    — sweep thrust over the reachable range, measure STEADY encv at each level
                       (via velocity.measure_steady_speed). = the SpeedProfile FF curve + static
                       gain map. Also logs tele RPM and current for cross-check.
  5. step-response   — at several operating points, apply thrust STEPS (up and down) and log encv
                       at the 50 Hz loop rate. Fit a first-order (+deadtime) model per step:
                       r(t) = r_inf + (r0 - r_inf) * exp(-(t-L)/tau), grid-search tau,L by SSE.
  6. emit            — a model JSON (pole pairs, no-load top, deadband, static curve, per-op
                       {K, tau, delay, overshoot}, and a world-model sim seed {tau, fullscale}),
                       a SpeedProfile YAML for velctl/posctl, and per-stage CSVs.

Safety (motor is a real thing): every drive send goes through ESC.send_thrust (tmax clamp);
an overspeed guard cuts to 0 above --max-rpm; a current guard backs off near --max-amps; every
stage is time-capped; and a try/finally ALWAYS disarms. --dry-run drives SimEncEscHost (no serial,
no motor) and is the regression oracle — it should recover tau ~= sim TAU and a linear curve.

Usage:
  python3 sysid.py --dry-run --name sim_check                 # validate the pipeline (no hardware)
  python3 sysid.py --name 350kv --max-rpm 3000 --max-thrust 800   # real hardware, hands-off
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import statistics
import sys
import time

from pico_esc.constants import FULLSCALE_RPM, POLE_PAIRS, TELE_MIN_MECH_RPM
from pico_esc.control import DT, VelReader, _pace
from pico_esc.drive import Aborted
from pico_esc.esc import ESC
from pico_esc.link import EscHost, RealClock, SimClock
from pico_esc.velocity import SpeedProfile, measure_steady_speed

REPORT_DIR = os.path.join(os.path.dirname(__file__), "reports")
MODEL_DIR = os.path.join(os.path.dirname(__file__), "models")


# ---------------------------------------------------------------------------
# host / clock (mirror posctl.open_host so --dry-run uses the SAME sim oracle).
# ---------------------------------------------------------------------------
def open_host(opts):
    if opts.dry_run:
        from pico_esc.sim import SimEncEscHost
        print(f"# DRY-RUN: SimEncEscHost (no serial port, no motor)"
              f"{' [--sim-invert]' if opts.sim_invert else ''}")
        clock = SimClock()
        return SimEncEscHost(clock, seed=opts.seed, invert=opts.sim_invert), clock
    return EscHost(opts.port), RealClock()


def log(msg):
    print(msg, flush=True)


# ---------------------------------------------------------------------------
# first-order (+deadtime) fit — pure stdlib grid search, no numpy/scipy.
# ---------------------------------------------------------------------------
def _model_rmse(ts, ys, r0, r_inf, tau, L):
    span = r_inf - r0
    if tau <= 0:
        return float("inf")
    inv, sse = 1.0 / tau, 0.0
    for t, y in zip(ts, ys):
        # r(t) = r_inf + (r0 - r_inf)*exp(-(t-L)/tau) = r_inf - span*exp(...), span = r_inf - r0
        model = r0 if t < L else r_inf - span * math.exp(-(t - L) * inv)
        d = y - model
        sse += d * d
    return math.sqrt(sse / len(ts))


def _smooth(ys, w=3):
    if len(ys) < w:
        return list(ys)
    out = []
    for i in range(len(ys)):
        a, b = max(0, i - w // 2), min(len(ys), i + w // 2 + 1)
        out.append(sum(ys[a:b]) / (b - a))
    return out


def fit_first_order(ts, ys, r0, r_inf):
    """Fit r(t) = r_inf + (r0 - r_inf) * exp(-(t - L) / tau) to (ts, ys) (t = 0 at the step).

    Primary estimator = the standard log-linear method: detect deadtime L (first sample that has
    moved > 10% of the span off r0), then linear-regress z = ln|r_inf - y| against t over the
    active-decay window (slope = -1/tau). This is robust to the DC offset the (tau, L) grid used to
    trade against. A coarse grid is kept as a fallback; whichever reconstructs with lower RMSE wins.
    Returns (tau, L, sse, rmse); degenerate (r_inf == r0) -> tau 0."""
    span = r_inf - r0
    if abs(span) < 1e-6 or len(ts) < 5:
        return 0.0, 0.0, 0.0, 0.0
    ys_s = _smooth(ys, 3)
    aspan = abs(span)
    # deadtime: first t where the (smoothed) response has left r0 by > 10% of span
    L = 0.0
    for t, y in zip(ts, ys_s):
        if abs(y - r0) > 0.10 * aspan:
            L = max(0.0, t)
            break
    # log-linear regression over [L, first time within 12% of r_inf]
    xs, zs = [], []
    for t, y in zip(ts, ys_s):
        if t < L:
            continue
        rem = r_inf - y
        if rem * span <= 0:                    # overshoot / crossed the asymptote -> stop the window
            break
        frac = abs(rem) / aspan
        if frac < 0.12:
            break
        xs.append(t - L)
        zs.append(math.log(frac))             # ln(|r_inf - y| / |span|) ~ -(t-L)/tau
    tau_ll = 0.0
    if len(xs) >= 3:
        n = len(xs)
        sx, sz = sum(xs), sum(zs)
        sxx = sum(x * x for x in xs)
        sxz = sum(x * z for x, z in zip(xs, zs))
        denom = n * sxx - sx * sx
        if abs(denom) > 1e-12:
            slope = (n * sxz - sx * sz) / denom
            if slope < -1e-6:
                tau_ll = -1.0 / slope
    # coarse grid fallback (tau x L) minimizing RMSE directly
    tmax = ts[-1]
    best = None
    for k in range(28):
        tau = 0.004 * (1.28 ** k)             # ~4 ms .. ~1.4 s
        for Lg in (0.0, L, 0.02, 0.04):
            if Lg >= 0.6 * tmax:
                continue
            r = _model_rmse(ts, ys_s, r0, r_inf, tau, Lg)
            if best is None or r < best[2]:
                best = (tau, Lg, r)
    cands = []
    if tau_ll > 0:
        cands.append((tau_ll, L, _model_rmse(ts, ys_s, r0, r_inf, tau_ll, L)))
    if best:
        cands.append(best)
    tau, L, rmse = min(cands, key=lambda c: c[2])
    return tau, L, rmse * rmse * len(ts), rmse


# ---------------------------------------------------------------------------
# drive primitives (all keep-alive paced; every send via esc.thrust choke).
# ---------------------------------------------------------------------------
def _guard(esc, vel, amps, opts):
    """Return an abort reason string if a safety limit is exceeded, else None."""
    if abs(vel) > opts.max_rpm:
        return f"overspeed {vel:.0f} > {opts.max_rpm} rpm"
    if amps is not None and opts.max_amps and amps >= opts.max_amps:
        return f"over-current {amps} >= {opts.max_amps} A"
    return None


def hold(esc, clock, thrust, secs, enc_sign, opts, tele_every=0.25):
    """Hold a constant thrust for `secs`, keep-alive paced, returning (mean_rpm, mean_amps,
    peak_amps, peak_temp). Raises Aborted on a safety-guard trip (caller cuts to 0 + disarms)."""
    vr = VelReader(sign=enc_sign)
    vels, amps_l, peak_temp, peak_amps = [], [], None, None
    t_end = clock.now() + secs
    next_tele = clock.now()
    while clock.now() < t_end:
        tick = clock.now()
        vel = vr.read(esc, DT)
        vels.append(vel)
        amps = None
        if tick >= next_tele:
            next_tele = tick + tele_every
            tel = esc.telemetry()
            if tel is not None:
                amps = tel.amps
                amps_l.append(amps)
                peak_amps = amps if peak_amps is None else max(peak_amps, amps)
                if tel.temp is not None:
                    peak_temp = tel.temp if peak_temp is None else max(peak_temp, tel.temp)
        rsn = _guard(esc, vel, amps, opts)
        if rsn:
            esc.thrust(0)
            raise Aborted(rsn)
        esc.thrust(thrust)
        _pace(clock, tick)
    esc.thrust(0)
    mean_rpm = statistics.mean(vels[len(vels) // 2:]) if len(vels) >= 4 else 0.0
    mean_amps = statistics.mean(amps_l) if amps_l else None
    return mean_rpm, mean_amps, peak_amps, peak_temp


def step_capture(esc, clock, bias, target, settle_secs, capture_secs, enc_sign, opts):
    """Hold `bias` for settle_secs, then STEP to `target` and log encv at the loop rate for
    capture_secs. Returns (rows, r0, r_inf) where rows = [(t_rel, vel, amps_or_None)] with
    t_rel = 0 at the step edge. Keep-alive paced; safety-guarded."""
    vr = VelReader(sign=enc_sign)
    # settle at bias
    t_end = clock.now() + settle_secs
    while clock.now() < t_end:
        tick = clock.now()
        vel = vr.read(esc, DT)
        rsn = _guard(esc, vel, None, opts)
        if rsn:
            esc.thrust(0)
            raise Aborted(rsn)
        esc.thrust(bias)
        _pace(clock, tick)
    r0 = vr.vel
    # step + capture
    rows = []
    t0 = clock.now()
    t_end = t0 + capture_secs
    next_tele = t0
    while clock.now() < t_end:
        tick = clock.now()
        vel = vr.read(esc, DT)
        amps = None
        if tick >= next_tele:
            next_tele = tick + 0.1
            tel = esc.telemetry()
            amps = tel.amps if tel is not None else None
        rows.append((tick - t0, vel, amps))
        rsn = _guard(esc, vel, amps, opts)
        if rsn:
            esc.thrust(0)
            raise Aborted(rsn)
        esc.thrust(target)
        _pace(clock, tick)
    esc.thrust(0)
    tail = [v for _, v, _ in rows[int(0.75 * len(rows)):]]
    r_inf = statistics.mean(tail) if tail else r0
    return rows, r0, r_inf


# ---------------------------------------------------------------------------
# stages
# ---------------------------------------------------------------------------
def stage_direction(esc, clock, opts):
    """Nudge forward at a safe thrust; return enc_sign so +thrust -> +vel (>0)."""
    probe = opts.dir_thrust
    log(f"[direction] probing at thrust {probe} …")
    rpm, *_ = hold(esc, clock, probe, 1.2, +1, opts)
    sign = 1 if rpm >= 0 else -1
    log(f"[direction] +thrust -> {rpm:+.0f} rpm  => enc_sign {sign:+d}")
    return sign


def stage_pole_id(esc, clock, enc_sign, opts):
    """Spin at a safe mid thrust; real_pp = configured_pp * tele_rpm / enc_rpm."""
    thrust = opts.pole_thrust
    log(f"[pole-id] holding thrust {thrust} to compare encv vs tele …")
    vr = VelReader(sign=enc_sign)
    encs, teles = [], []
    t_end = clock.now() + 2.5
    next_tele = clock.now()
    while clock.now() < t_end:
        tick = clock.now()
        vel = vr.read(esc, DT)
        rsn = _guard(esc, vel, None, opts)
        if rsn:
            esc.thrust(0)
            raise Aborted(rsn)
        if clock.now() - (t_end - 2.5) > 1.0:            # after a 1 s settle
            encs.append(abs(vel))
            if tick >= next_tele:
                next_tele = tick + 0.15
                tel = esc.telemetry()
                if tel is not None and abs(tel.rpm) > TELE_MIN_MECH_RPM:
                    teles.append(abs(tel.rpm))
        esc.thrust(thrust)
        _pace(clock, tick)
    esc.thrust(0)
    enc_rpm = statistics.mean(encs) if encs else 0.0
    if not teles or enc_rpm < 1.0:
        log(f"[pole-id] enc_rpm={enc_rpm:.0f}, tele unavailable ({len(teles)} live frames) "
            f"-> cannot ID pole count (fine in --dry-run). Keeping configured pp={POLE_PAIRS}.")
        return {"enc_rpm": enc_rpm, "tele_rpm": None, "configured_pp": POLE_PAIRS,
                "real_pp": None, "ratio": None}
    tele_rpm = statistics.mean(teles)
    ratio = tele_rpm / enc_rpm
    real_pp = POLE_PAIRS * ratio
    nearest = max(1, round(real_pp))
    log(f"[pole-id] enc={enc_rpm:.0f} tele={tele_rpm:.0f} rpm  ratio={ratio:.3f}  "
        f"real_pp={real_pp:.2f} -> nearest {nearest} pole-pairs ({nearest * 2} poles). "
        f"configured pp={POLE_PAIRS}{' (MATCH)' if nearest == POLE_PAIRS else ' (MISMATCH!)'}")
    return {"enc_rpm": enc_rpm, "tele_rpm": tele_rpm, "configured_pp": POLE_PAIRS,
            "real_pp": real_pp, "nearest_pp": nearest, "ratio": ratio}


def stage_deadband(esc, clock, enc_sign, opts, direction=+1):
    """Step thrust up from 0 until the shaft sustains > deadband_rpm. Return the first thrust
    that spins (the commutation floor) for this direction, or None if it never spins by max."""
    lo, step = opts.deadband_start, opts.deadband_step
    log(f"[deadband {'fwd' if direction > 0 else 'rev'}] climbing from {lo} step {step} …")
    thr = lo
    while thr <= opts.deadband_max:
        rpm, *_ = hold(esc, clock, direction * thr, opts.deadband_settle, enc_sign, opts)
        log(f"  thrust {direction * thr:+5d} -> {rpm:+7.1f} rpm")
        if abs(rpm) >= opts.deadband_rpm:
            log(f"[deadband {'fwd' if direction > 0 else 'rev'}] floor = {direction * thr:+d} "
                f"({rpm:+.0f} rpm)")
            return direction * thr, rpm
        thr += step
    log(f"[deadband] never reached {opts.deadband_rpm} rpm by {opts.deadband_max}")
    return None, 0.0


def stage_static_curve(esc, clock, enc_sign, floor_thrust, opts):
    """Sweep thrust floor..max over N points; steady encv (+ tele/current) at each. Returns the
    list of point dicts (monotone by construction of the sweep)."""
    start = max(floor_thrust or opts.deadband_start, opts.deadband_start)
    thrusts = _linspace_int(start, opts.max_thrust, opts.curve_points)
    log(f"[curve] {len(thrusts)} points {thrusts[0]}..{thrusts[-1]} "
        f"(settle {opts.settle}s, measure {opts.measure}s each)")
    pts = []
    for thr in thrusts:
        mean, ripple, peak_temp = measure_steady_speed(
            esc, clock, thr, enc_sign, opts.settle, opts.measure, opts.max_temp)
        # cross-read one tele frame for current/tele-rpm at this level
        tel = esc.telemetry()
        pts.append({"thrust": thr, "rpm": round(mean, 1), "ripple": round(ripple, 2),
                    "tele_rpm": (tel.rpm if tel else None),
                    "amps": (tel.amps if tel else None),
                    "peak_temp": peak_temp})
        log(f"  thrust {thr:4d} -> {mean:7.1f} rpm  (ripple {ripple:5.1f}, "
            f"tele {tel.rpm if tel else '--'}, {tel.amps if tel else '--'} A)")
        esc.thrust(0)
        clock.sleep(opts.cooldown)
    return pts


def stage_steps(esc, clock, enc_sign, curve_pts, opts):
    """Run up/down thrust steps between operating points; fit first-order per step."""
    # choose bias/target pairs spanning the reachable range from the static curve
    thr_vals = [p["thrust"] for p in curve_pts if p["rpm"] and abs(p["rpm"]) > opts.deadband_rpm]
    if len(thr_vals) < 2:
        log("[steps] not enough reachable points to build steps — skipping")
        return []
    lo, mid, hi = thr_vals[0], thr_vals[len(thr_vals) // 2], thr_vals[-1]
    pairs = [(lo, mid), (mid, hi), (hi, mid), (mid, lo)]      # up, up, down, down
    log(f"[steps] {len(pairs)} steps around thr {lo}/{mid}/{hi}")
    results = []
    for i, (bias, target) in enumerate(pairs):
        rows, r0, r_inf = step_capture(
            esc, clock, bias, target, opts.settle, opts.step_secs, enc_sign, opts)
        ts = [t for t, _, _ in rows]
        ys = [v for _, v, _ in rows]
        tau, L, sse, rmse = fit_first_order(ts, ys, r0, r_inf)
        span = r_inf - r0
        dcmd = target - bias
        K = (span / dcmd) if dcmd else 0.0
        # overshoot relative to the commanded change (only meaningful on an up-step)
        peak = max(ys) if span > 0 else min(ys)
        overshoot = ((peak - r_inf) / span * 100.0) if abs(span) > 1e-6 else 0.0
        res = {"id": i, "bias": bias, "target": target, "r0": round(r0, 1),
               "r_inf": round(r_inf, 1), "K_rpm_per_cmd": round(K, 4),
               "tau_s": round(tau, 4), "delay_s": round(L, 4),
               "overshoot_pct": round(overshoot, 1), "rmse_rpm": round(rmse, 2),
               "rows": rows}
        log(f"  step {bias:4d}->{target:4d}: {r0:6.0f}->{r_inf:6.0f} rpm  "
            f"K={K:.3f} tau={tau*1000:5.0f}ms L={L*1000:3.0f}ms ov={overshoot:4.0f}% "
            f"rmse={rmse:.1f}")
        results.append(res)
        esc.thrust(0)
        clock.sleep(opts.cooldown)
    return results


def _linspace_int(a, b, n):
    if n <= 1:
        return [int(round(a))]
    return sorted(set(int(round(a + (b - a) * k / (n - 1))) for k in range(n)))


# ---------------------------------------------------------------------------
# emit
# ---------------------------------------------------------------------------
def write_step_csv(path, steps):
    with open(path, "w", encoding="utf-8", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["step_id", "bias", "target", "t", "rpm", "amps"])
        for s in steps:
            for t, v, a in s["rows"]:
                w.writerow([s["id"], s["bias"], s["target"], round(t, 4), round(v, 2),
                            "" if a is None else a])


def write_curve_csv(path, pts):
    with open(path, "w", encoding="utf-8", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["thrust", "rpm", "ripple", "tele_rpm", "amps", "peak_temp"])
        for p in pts:
            w.writerow([p["thrust"], p["rpm"], p["ripple"],
                        "" if p["tele_rpm"] is None else p["tele_rpm"],
                        "" if p["amps"] is None else p["amps"],
                        "" if p["peak_temp"] is None else p["peak_temp"]])


def build_model(name, pole, deadband, curve_pts, steps, opts):
    taus = [s["tau_s"] for s in steps if s["tau_s"] > 0]
    Ks = [s["K_rpm_per_cmd"] for s in steps if s["K_rpm_per_cmd"] > 0]
    delays = [s["delay_s"] for s in steps]
    top_rpm = max((abs(p["rpm"]) for p in curve_pts), default=0.0)
    tau_med = statistics.median(taus) if taus else None
    K_med = statistics.median(Ks) if Ks else None
    return {
        "name": name,
        "generated": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "supply_v": opts.supply_v,
        "pole": pole,
        "no_load_top_rpm": round(top_rpm, 1),
        "deadband_thrust": deadband,
        "static_curve": [{"thrust": p["thrust"], "rpm": p["rpm"]} for p in curve_pts],
        "dynamic": {
            "tau_s_median": round(tau_med, 4) if tau_med else None,
            "K_rpm_per_cmd_median": round(K_med, 4) if K_med else None,
            "delay_s_median": round(statistics.median(delays), 4) if delays else None,
            "per_step": [{k: s[k] for k in
                          ("bias", "target", "r0", "r_inf", "K_rpm_per_cmd",
                           "tau_s", "delay_s", "overshoot_pct", "rmse_rpm")} for s in steps],
        },
        # world-model sim seed: what to set in sim.py to reproduce THIS motor's response.
        "sim_seed": {
            "tau_s": round(tau_med, 4) if tau_med else None,
            "fullscale_rpm_at_1000cmd": round(top_rpm / (opts.max_thrust / 1000.0), 1)
            if top_rpm and opts.max_thrust else None,
            "deadband_thrust_fwd": deadband.get("fwd"),
            "note": "first-order rotor lag rpm += (tgt-rpm)*(1-exp(-dt/tau)); "
                    "compare against sim.py TAU/FULLSCALE_RPM",
        },
    }


def emit_profile(name, curve_pts, pole, opts):
    """Emit a monotone SpeedProfile YAML (thrust->rpm) for velctl/posctl."""
    pts = [(p["thrust"], p["rpm"]) for p in curve_pts if p["rpm"] is not None]
    # ensure strictly monotone rpm (SpeedProfile rejects non-monotone)
    mono = []
    last = None
    for thr, rpm in sorted(pts):
        if last is None or rpm > last + 0.1:
            mono.append((thr, rpm))
            last = rpm
    if len(mono) < 2:
        return None
    pp = pole.get("nearest_pp") or pole.get("configured_pp") or POLE_PAIRS
    prof = SpeedProfile(mono, motor=name, pole_pairs=pp, source="bench-sysid")
    path = os.path.join(os.path.dirname(__file__), "profiles", f"{name}_sysid.yaml")
    prof.save(path)
    return path


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def run(opts):
    os.makedirs(REPORT_DIR, exist_ok=True)
    os.makedirs(MODEL_DIR, exist_ok=True)
    ts = time.strftime("%Y%m%d-%H%M%S")
    host, clock = open_host(opts)
    esc = ESC(host, opts.esc, tmax=opts.max_thrust, clock=clock)
    model = None
    try:
        esc.prepare()
        log(f"# arming ESC {opts.esc} (bidir), tmax={opts.max_thrust}, "
            f"max_rpm={opts.max_rpm}, max_amps={opts.max_amps}")
        esc.arm(bidir=True)

        enc_sign = stage_direction(esc, clock, opts)
        pole = stage_pole_id(esc, clock, enc_sign, opts)
        clock.sleep(opts.cooldown)

        deadband = {"fwd": None, "rev": None}
        f_thr, _ = stage_deadband(esc, clock, enc_sign, opts, +1)
        deadband["fwd"] = f_thr
        if opts.both_dirs:
            r_thr, _ = stage_deadband(esc, clock, enc_sign, opts, -1)
            deadband["rev"] = r_thr
        clock.sleep(opts.cooldown)

        curve = stage_static_curve(esc, clock, enc_sign, deadband["fwd"], opts)
        write_curve_csv(os.path.join(REPORT_DIR, f"sysid_{opts.name}_{ts}_curve.csv"), curve)

        steps = stage_steps(esc, clock, enc_sign, curve, opts)
        write_step_csv(os.path.join(REPORT_DIR, f"sysid_{opts.name}_{ts}_steps.csv"), steps)

        model = build_model(opts.name, pole, deadband, curve, steps, opts)
        prof_path = emit_profile(opts.name, curve, pole, opts)
        model["profile_yaml"] = prof_path
        model_path = os.path.join(MODEL_DIR, f"{opts.name}_sysid.json")
        with open(model_path, "w", encoding="utf-8") as fh:
            json.dump({k: v for k, v in model.items()}, fh, indent=2)
        # also drop the raw step rows next to the model for the report/world-model fit
        raw_path = os.path.join(MODEL_DIR, f"{opts.name}_sysid_raw.json")
        with open(raw_path, "w", encoding="utf-8") as fh:
            json.dump({"curve": curve,
                       "steps": [{k: v for k, v in s.items()} for s in steps]}, fh)
        log("")
        log("=== sysid summary ===")
        log(f"  pole:      {pole.get('nearest_pp', '?')} pole-pairs "
            f"(ratio {pole.get('ratio')})")
        log(f"  deadband:  fwd {deadband['fwd']}  rev {deadband['rev']}")
        log(f"  top rpm:   {model['no_load_top_rpm']} @ thrust {opts.max_thrust}")
        log(f"  dynamics:  tau~{model['dynamic']['tau_s_median']}s  "
            f"K~{model['dynamic']['K_rpm_per_cmd_median']} rpm/cmd  "
            f"delay~{model['dynamic']['delay_s_median']}s")
        log(f"  model:     {model_path}")
        log(f"  profile:   {prof_path}")
    except Aborted as e:
        log(f"!! ABORTED: {e}")
    except KeyboardInterrupt:
        log("!! interrupted")
    finally:
        try:
            esc.disarm()
        except Exception:
            pass
        if not opts.dry_run:
            try:
                host.close()
            except Exception:
                pass
    return model


def build_parser():
    ap = argparse.ArgumentParser(description="hands-off plant identification for one ESC/motor")
    ap.add_argument("--name", default="motor", help="label for output files")
    ap.add_argument("--esc", type=int, default=1, help="ESC index")
    ap.add_argument("--port", help="serial port (default: auto-detect VID 2E8A)")
    ap.add_argument("--dry-run", action="store_true", help="drive SimEncEscHost (no serial/motor)")
    ap.add_argument("--sim-invert", action="store_true", help="dry-run: +thrust -> -encoder")
    ap.add_argument("--seed", type=int, default=0, help="dry-run RNG seed")
    # safety envelope
    ap.add_argument("--max-rpm", type=float, default=3000, help="overspeed cut (mech RPM)")
    ap.add_argument("--max-thrust", type=int, default=800, help="thrust magnitude ceiling (tmax)")
    ap.add_argument("--max-amps", type=float, default=4.5, help="over-current back-off (0=off)")
    ap.add_argument("--max-temp", type=float, default=0.0, help="over-temp abort C (0=off, telem noisy)")
    # probes
    ap.add_argument("--dir-thrust", type=int, default=200, help="direction-probe thrust")
    ap.add_argument("--pole-thrust", type=int, default=350, help="pole-id hold thrust")
    ap.add_argument("--both-dirs", action="store_true", help="also characterize reverse")
    # deadband
    ap.add_argument("--deadband-start", type=int, default=20)
    ap.add_argument("--deadband-step", type=int, default=15)
    ap.add_argument("--deadband-max", type=int, default=300)
    ap.add_argument("--deadband-settle", type=float, default=0.8)
    ap.add_argument("--deadband-rpm", type=float, default=30.0, help="rpm that counts as spinning")
    # static curve
    ap.add_argument("--curve-points", type=int, default=10)
    ap.add_argument("--settle", type=float, default=1.0)
    ap.add_argument("--measure", type=float, default=0.8)
    # steps
    ap.add_argument("--step-secs", type=float, default=1.5, help="capture window per step")
    ap.add_argument("--cooldown", type=float, default=0.4, help="idle between stages/points")
    ap.add_argument("--supply-v", type=float, default=11.1, help="supply voltage (metadata)")
    return ap


def main(argv=None):
    opts = build_parser().parse_args(argv)
    run(opts)


if __name__ == "__main__":
    main()
