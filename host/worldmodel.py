#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""worldmodel — fit a plant "world model" of the motor and VALIDATE it reproduces the response.

This is the sim-reproduction stretch goal: given the sysid results (static thrust->RPM curve +
first-order dynamics), build a plant model that predicts rpm(t) from the thrust command, then prove
it by REPLAYING an out-of-sample CLOSED-LOOP trace (the velocity-tune run's thrust log) through the
model and overlaying the prediction against the real encoder. A physics model (static curve +
first-order lag + transport delay) — not an RL net — because it is small, interpretable, and directly
portable into sim.py (SimEncEscHost) so the whole dry-run stack reproduces THIS motor.

  fit      — read models/<name>_sysid.json (+ _raw.json); assemble the plant params; report the
             in-sample fit against the step-response rows.
  validate — replay a veltune trace CSV (t,seg,target,sp,meas,thrust) through the plant and report
             out-of-sample VAF/RMSE; write an overlay CSV + a self-contained HTML.

Pure offline (reads JSON/CSV, writes JSON/CSV/HTML) — no serial port, safe to run any time.

Usage:
  python3 worldmodel.py --name 350kv_sine fit
  python3 worldmodel.py --name 350kv_sine validate --trace reports/veltune_tuned_350kv_*.csv
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import statistics

MODEL_DIR = os.path.join(os.path.dirname(__file__), "models")
REPORT_DIR = os.path.join(os.path.dirname(__file__), "reports")


# ---------------------------------------------------------------------------
# the plant world-model
# ---------------------------------------------------------------------------
class Plant:
    """Static curve (thrust->steady rpm, piecewise-linear, odd-symmetric) + first-order lag with a
    transport delay. rpm advances toward the delayed static target each dt. This is exactly the
    structure of sim.py's SimEncEscHost (first-order rotor lag), but with THIS motor's fitted
    params, so the fitted (tau, curve) can be dropped straight into the sim."""

    def __init__(self, curve, tau, delay=0.0):
        # curve: sorted list of (thrust, rpm) with thrust >= 0 (odd-symmetric use for negative)
        self.curve = sorted(curve)
        self.tau = max(1e-3, tau)
        self.delay = max(0.0, delay)
        self.reset()

    def reset(self, rpm0=0.0):
        self.rpm = rpm0
        self._buf = []                 # (t, target) transport-delay FIFO

    def static(self, thrust):
        s = 1.0 if thrust >= 0 else -1.0
        a = abs(thrust)
        c = self.curve
        if a <= c[0][0]:
            # below the lowest measured thrust: linear from origin to the first point
            return s * (a / c[0][0] * c[0][1] if c[0][0] > 0 else 0.0)
        if a >= c[-1][0]:
            return s * c[-1][1]
        for i in range(1, len(c)):
            if a <= c[i][0]:
                t0, r0 = c[i - 1]
                t1, r1 = c[i]
                f = (a - t0) / (t1 - t0) if t1 > t0 else 0.0
                return s * (r0 + f * (r1 - r0))
        return s * c[-1][1]

    def step(self, thrust, dt, t_now):
        # transport delay: use the target from `delay` seconds ago
        target_now = self.static(thrust)
        self._buf.append((t_now, target_now))
        tgt = self._buf[0][1]
        while len(self._buf) > 1 and self._buf[0][0] <= t_now - self.delay:
            self._buf.pop(0)
            tgt = self._buf[0][1] if self._buf else target_now
        alpha = 1.0 - math.exp(-dt / self.tau)
        self.rpm += (tgt - self.rpm) * alpha
        return self.rpm


# ---------------------------------------------------------------------------
# assemble the plant from sysid output
# ---------------------------------------------------------------------------
def build_plant(model, raw):
    curve = [(p["thrust"], p["rpm"]) for p in model["static_curve"] if p["rpm"] is not None]
    tau = model["dynamic"].get("tau_s_median") or 0.15
    delay = model["dynamic"].get("delay_s_median") or 0.0
    return Plant(curve, tau, delay), tau, delay


def insample_fit(plant, raw):
    """Replay each recorded step through the plant; report per-step + overall RMSE/VAF."""
    out = []
    for s in raw.get("steps", []):
        rows = s.get("rows", [])
        if len(rows) < 5:
            continue
        # rows: [t_rel, rpm, amps]; input is a step from bias to target at t_rel=0
        plant.reset(rows[0][1])
        sim = []
        prev_t = rows[0][0]
        for t, y, _a in rows:
            dt = max(1e-3, t - prev_t)
            prev_t = t
            sim.append(plant.step(s["target"], dt, t))
        real = [r[1] for r in rows]
        out.append({"bias": s["bias"], "target": s["target"],
                    **_fit_stats(real, sim)})
    return out


def _fit_stats(real, sim):
    n = min(len(real), len(sim))
    real, sim = real[:n], sim[:n]
    err = [r - m for r, m in zip(real, sim)]
    rmse = math.sqrt(sum(e * e for e in err) / n) if n else 0.0
    var_real = statistics.pvariance(real) if n > 1 else 0.0
    var_err = statistics.pvariance(err) if n > 1 else 0.0
    vaf = 100.0 * (1.0 - var_err / var_real) if var_real > 1e-9 else 0.0
    return {"rmse": round(rmse, 2), "vaf": round(vaf, 1), "n": n}


# ---------------------------------------------------------------------------
# out-of-sample validation on a closed-loop veltune trace
# ---------------------------------------------------------------------------
def read_trace(path):
    rows = []
    with open(path, encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            rows.append({"t": float(r["t"]), "meas": float(r["meas"]),
                         "thrust": float(r["thrust"]), "target": float(r.get("target", 0))})
    return rows


def validate_trace(plant, rows):
    plant.reset(rows[0]["meas"])
    sim = []
    prev_t = rows[0]["t"]
    for r in rows:
        dt = max(1e-3, r["t"] - prev_t)
        prev_t = r["t"]
        sim.append(plant.step(r["thrust"], dt, r["t"]))
    real = [r["meas"] for r in rows]
    stats = _fit_stats(real, sim)
    return sim, stats


def write_overlay_csv(path, rows, sim):
    with open(path, "w", encoding="utf-8", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "thrust", "target", "real_rpm", "sim_rpm"])
        for r, m in zip(rows, sim):
            w.writerow([round(r["t"], 4), round(r["thrust"], 1), round(r["target"], 1),
                        round(r["meas"], 1), round(m, 1)])


def write_html(path, name, rows, sim, stats, tau, delay):
    real = [r["meas"] for r in rows]
    tgt = [r["target"] for r in rows]
    ts = [r["t"] for r in rows]
    data = json.dumps({"t": [round(x, 3) for x in ts],
                       "real": [round(x, 1) for x in real],
                       "sim": [round(x, 1) for x in sim],
                       "tgt": [round(x, 1) for x in tgt]})
    html = f"""<!doctype html><meta charset=utf-8>
<title>world-model overlay — {name}</title>
<style>
 :root{{color-scheme:light dark}}
 body{{font:14px/1.5 system-ui,sans-serif;margin:2rem;max-width:960px}}
 canvas{{width:100%;height:420px;border:1px solid #8884;border-radius:8px}}
 .k{{display:inline-block;margin-right:1.5rem}} b{{font-variant-numeric:tabular-nums}}
</style>
<h1>World-model overlay — {name}</h1>
<p>Out-of-sample: the fitted plant (static curve + first-order lag tau={tau:.3f}s, delay={delay:.3f}s)
replayed through the <b>closed-loop velocity-tune thrust log</b> vs the real encoder.</p>
<p><span class=k>VAF <b>{stats['vaf']}%</b></span><span class=k>RMSE <b>{stats['rmse']} rpm</b></span>
<span class=k>n=<b>{stats['n']}</b></span></p>
<canvas id=c></canvas>
<script>
const D={data};
const c=document.getElementById('c'),x=c.getContext('2d');
function draw(){{
 const w=c.width=c.clientWidth*devicePixelRatio,h=c.height=c.clientHeight*devicePixelRatio;
 const P=44*devicePixelRatio;
 const xs=D.t,all=D.real.concat(D.sim,D.tgt);
 const t0=xs[0],t1=xs[xs.length-1];
 let lo=Math.min(...all),hi=Math.max(...all);lo=Math.min(lo,0);
 const px=t=>P+(t-t0)/(t1-t0)*(w-2*P), py=v=>h-P-(v-lo)/(hi-lo)*(h-2*P);
 x.clearRect(0,0,w,h);
 x.strokeStyle='#8886';x.lineWidth=devicePixelRatio;
 x.strokeRect(P,P,w-2*P,h-2*P);
 x.font=(12*devicePixelRatio)+'px system-ui';x.fillStyle='#888';
 for(let k=0;k<=5;k++){{const v=lo+(hi-lo)*k/5;const y=py(v);
   x.fillText(v.toFixed(0),4,y+4);x.strokeStyle='#8883';
   x.beginPath();x.moveTo(P,y);x.lineTo(w-P,y);x.stroke();}}
 function line(arr,col,wd){{x.strokeStyle=col;x.lineWidth=wd*devicePixelRatio;x.beginPath();
   arr.forEach((v,i)=>{{const X=px(xs[i]),Y=py(v);i?x.lineTo(X,Y):x.moveTo(X,Y);}});x.stroke();}}
 line(D.tgt,'#f9a825',1);line(D.real,'#1e88e5',1.5);line(D.sim,'#e53935',1.5);
 x.fillStyle='#1e88e5';x.fillText('real (encoder)',P+8,P+18*devicePixelRatio);
 x.fillStyle='#e53935';x.fillText('world-model',P+8,P+36*devicePixelRatio);
 x.fillStyle='#f9a825';x.fillText('target',P+8,P+54*devicePixelRatio);
}}
draw();addEventListener('resize',draw);
</script>"""
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(html)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def load_models(name):
    model = json.load(open(os.path.join(MODEL_DIR, f"{name}_sysid.json")))
    raw_path = os.path.join(MODEL_DIR, f"{name}_sysid_raw.json")
    raw = json.load(open(raw_path)) if os.path.exists(raw_path) else {"steps": []}
    return model, raw


def main(argv=None):
    ap = argparse.ArgumentParser(description="fit + validate a plant world-model")
    ap.add_argument("--name", default="350kv_sine")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("fit")
    v = sub.add_parser("validate")
    v.add_argument("--trace", help="veltune trace CSV (glob ok; newest match used)")
    opts = ap.parse_args(argv)

    model, raw = load_models(opts.name)
    plant, tau, delay = build_plant(model, raw)
    print(f"# world-model {opts.name}: tau={tau:.3f}s delay={delay:.3f}s, "
          f"curve {len(plant.curve)} pts, top {plant.curve[-1][1]:.0f} rpm")

    ins = insample_fit(plant, raw)
    if ins:
        print("[in-sample step fit]")
        for s in ins:
            print(f"  step {s['bias']:4.0f}->{s['target']:4.0f}: VAF {s['vaf']:5.1f}%  "
                  f"RMSE {s['rmse']:5.1f} rpm")
        print(f"  mean VAF {statistics.mean(s['vaf'] for s in ins):.1f}%")

    out = {"name": opts.name, "tau_s": tau, "delay_s": delay,
           "curve": plant.curve, "insample": ins,
           "sim_seed": {"TAU": round(tau, 4),
                        "FULLSCALE_RPM": plant.curve[-1][1],
                        "note": "drop into sim.py SimEncEscHost to reproduce this motor"}}

    if opts.cmd == "validate":
        pat = opts.trace or os.path.join(REPORT_DIR, f"veltune_tuned_*{'350kv' if '350' in opts.name else ''}*.csv")
        matches = sorted(glob.glob(pat))
        if not matches:
            print(f"!! no trace matching {pat}")
            return
        trace_path = matches[-1]
        rows = read_trace(trace_path)
        sim, stats = validate_trace(plant, rows)
        print(f"[out-of-sample validate] {os.path.basename(trace_path)}")
        print(f"  VAF {stats['vaf']}%  RMSE {stats['rmse']} rpm  n={stats['n']}")
        base = os.path.join(REPORT_DIR, f"worldmodel_{opts.name}")
        write_overlay_csv(base + "_overlay.csv", rows, sim)
        write_html(base + "_overlay.html", opts.name, rows, sim, stats, tau, delay)
        out["validate"] = {"trace": trace_path, **stats}
        print(f"  overlay: {base}_overlay.html")

    json.dump(out, open(os.path.join(MODEL_DIR, f"{opts.name}_worldmodel.json"), "w"), indent=2)
    print(f"# wrote {os.path.join(MODEL_DIR, opts.name + '_worldmodel.json')}")


if __name__ == "__main__":
    main()
