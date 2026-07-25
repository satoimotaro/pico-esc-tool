#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""campaign_report — one self-contained HTML performance report for a motor campaign.

Pulls together the sysid model, the velocity-tune trace, the position step, and the world-model
overlay into a single theme-aware page with graphs. Pure offline; open the file in a browser.

  python3 campaign_report.py --name 350kv
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(HERE, "models")
REPORT = os.path.join(HERE, "reports")


def newest(pat):
    m = sorted(glob.glob(os.path.join(REPORT, pat)))
    return m[-1] if m else None


def read_csv(path):
    with open(path, encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def jload(path):
    return json.load(open(path)) if path and os.path.exists(path) else None


def build(name):
    sysid = jload(os.path.join(MODEL, f"{name}_sine_sysid.json"))
    raw = jload(os.path.join(MODEL, f"{name}_sine_sysid_raw.json")) or {"curve": [], "steps": []}
    wm = jload(os.path.join(MODEL, f"{name}_sine_worldmodel.json"))
    gains = jload(newest(f"veltune_gains_{name}_*.json"))
    vtrace = newest(f"veltune_tuned_{name}_*.csv")
    pstep = newest("posctl_step_*.csv")

    # ---- static curve + linear fit ----
    curve = [(p["thrust"], p["rpm"]) for p in raw["curve"] if p["rpm"] is not None]
    n = len(curve)
    sx = sum(t for t, _ in curve); sy = sum(r for _, r in curve)
    sxx = sum(t * t for t, _ in curve); sxy = sum(t * r for t, r in curve)
    denom = n * sxx - sx * sx
    slope = (n * sxy - sx * sy) / denom if denom else 0
    inter = (sy - slope * sx) / n if n else 0

    charts = []
    charts.append({
        "id": "curve", "title": "Static map: thrust → steady RPM (encoder)",
        "xlabel": "thrust cmd", "ylabel": "mech RPM",
        "series": [
            {"label": f"measured (K≈{slope:.3f} rpm/cmd)", "color": "#1e88e5", "width": 2,
             "pts": [[t, r] for t, r in curve]},
            {"label": "linear fit", "color": "#9e9e9e", "width": 1, "dash": True,
             "pts": [[curve[0][0], slope * curve[0][0] + inter],
                     [curve[-1][0], slope * curve[-1][0] + inter]]},
        ]})

    # ---- step response + first-order fit (largest up-step) ----
    steps = raw["steps"]
    up = max(steps, key=lambda s: (s["r_inf"] - s["r0"]), default=None) if steps else None
    if up:
        r0, r_inf, tau, L = up["r0"], up["r_inf"], up["tau_s"], up["delay_s"]
        real = [[t, y] for t, y, *_ in up["rows"]]
        fit = [[t, (r0 if t < L else r_inf - (r_inf - r0) * math.exp(-(t - L) / max(1e-3, tau)))]
               for t, *_ in up["rows"]]
        charts.append({
            "id": "step", "title": f"Step response {up['bias']}→{up['target']} cmd  "
                                   f"(τ={tau*1000:.0f} ms, delay={L*1000:.0f} ms, "
                                   f"overshoot {up['overshoot_pct']:.0f}%)",
            "xlabel": "t (s)", "ylabel": "mech RPM",
            "series": [
                {"label": "encoder", "color": "#1e88e5", "width": 1.5, "pts": real},
                {"label": "first-order fit", "color": "#e53935", "width": 2, "pts": fit},
            ]})

    # ---- velocity tracking (tuned) ----
    if vtrace:
        rows = read_csv(vtrace)
        t = [float(r["t"]) for r in rows]
        charts.append({
            "id": "vel", "title": "Velocity control (tuned): target vs measured",
            "xlabel": "t (s)", "ylabel": "mech RPM",
            "series": [
                {"label": "target", "color": "#f9a825", "width": 1,
                 "pts": [[t[i], float(r["target"])] for i, r in enumerate(rows)]},
                {"label": "setpoint (slewed)", "color": "#9e9e9e", "width": 1, "dash": True,
                 "pts": [[t[i], float(r["sp"])] for i, r in enumerate(rows)]},
                {"label": "measured (encoder)", "color": "#1e88e5", "width": 1.5,
                 "pts": [[t[i], float(r["meas"])] for i, r in enumerate(rows)]},
            ]})

    # ---- position step ----
    if pstep:
        rows = read_csv(pstep)
        charts.append({
            "id": "pos", "title": "Position control (tuned): setpoint vs encoder angle",
            "xlabel": "t (s)", "ylabel": "deg",
            "series": [
                {"label": "setpoint", "color": "#f9a825", "width": 1,
                 "pts": [[float(r["t"]), float(r["pos_setpoint"])] for r in rows]},
                {"label": "encoder", "color": "#1e88e5", "width": 1.5,
                 "pts": [[float(r["t"]), float(r["pos_deg"])] for r in rows]},
            ]})

    # ---- world-model overlay ----
    wm_overlay = os.path.join(REPORT, f"worldmodel_{name}_sine_overlay.csv")
    if os.path.exists(wm_overlay):
        rows = read_csv(wm_overlay)
        charts.append({
            "id": "wm", "title": "World-model: prediction vs real (out-of-sample closed loop)",
            "xlabel": "t (s)", "ylabel": "mech RPM",
            "series": [
                {"label": "real (encoder)", "color": "#1e88e5", "width": 1.5,
                 "pts": [[float(r["t"]), float(r["real_rpm"])] for r in rows]},
                {"label": "world-model", "color": "#e53935", "width": 1.5,
                 "pts": [[float(r["t"]), float(r["sim_rpm"])] for r in rows]},
            ]})

    # ---- summary tables ----
    dyn = sysid["dynamic"] if sysid else {}
    summary = {
        "K_rpm_per_cmd": round(slope, 3),
        "tau_s": dyn.get("tau_s_median"),
        "delay_s": dyn.get("delay_s_median"),
        "deadband_thrust": sysid["deadband_thrust"]["fwd"] if sysid else None,
        "top_rpm": sysid["no_load_top_rpm"] if sysid else None,
        "vel_gains": gains["gains"] if gains else None,
        "vel_segments": gains["segments"] if gains else None,
        "wm_vaf": (wm.get("validate", {}) or {}).get("vaf") if wm else None,
        "wm_rmse": (wm.get("validate", {}) or {}).get("rmse") if wm else None,
    }
    return charts, summary


HTML = """<!doctype html><meta charset=utf-8>
<title>Motor campaign report — {name}</title>
<style>
 :root{{color-scheme:light dark}}
 body{{font:14px/1.55 system-ui,-apple-system,sans-serif;margin:0 auto;max-width:920px;padding:2rem}}
 h1{{margin:.2rem 0}} h2{{margin:2rem 0 .5rem;font-size:1.15rem}}
 .sub{{color:#888}} .find{{background:#f9a82522;border-left:3px solid #f9a825;padding:.8rem 1rem;
   border-radius:6px;margin:1rem 0}}
 table{{border-collapse:collapse;margin:.5rem 0;font-variant-numeric:tabular-nums}}
 td,th{{border:1px solid #8884;padding:.3rem .7rem;text-align:right}} th{{text-align:left}}
 canvas{{width:100%;height:340px;border:1px solid #8884;border-radius:8px;margin:.3rem 0}}
 .leg span{{display:inline-block;margin-right:1.2rem}} .sw{{display:inline-block;width:16px;height:3px;
   vertical-align:middle;margin-right:4px}}
 code{{background:#8882;padding:.1rem .35rem;border-radius:4px}}
</style>
<h1>Motor characterization &amp; control — {name}</h1>
<p class=sub>350KV bare-shaft (no prop, in air) · 11.1&nbsp;V / 5&nbsp;A · AS5600 encoder · pure forced-sine envelope</p>

<div class=find>
<b>Key finding.</b> At no load this motor <b>cannot enter 6-step</b> (weak BEMF + zero inertia): stock
sine desyncs ~184&nbsp;rpm and the rotor stalls. Disabling the crossover gives a clean pure-sine plant
<b>0–~300&nbsp;rpm</b> — the ~300&nbsp;rpm ceiling is the firmware sine full-scale, not the motor
(which could do ~3900&nbsp;rpm only in 6-step). All results below are on this sine envelope; the
<b>encoder is the only usable feedback</b> (tele is virtual/frozen below ~250&nbsp;rpm).
</div>

<h2>Identified plant &amp; tuned controllers</h2>
{tables}

{charts}

<p class=sub>Self-contained report generated by campaign_report.py. Tools: sysid.py · veltune.py ·
tune_posctl.py · worldmodel.py · campaign.py.</p>

<script>
const CH={charts_json};
function chart(cv, ch){{
 const x=cv.getContext('2d');
 function draw(){{
  const w=cv.width=cv.clientWidth*devicePixelRatio, h=cv.height=cv.clientHeight*devicePixelRatio;
  const P=46*devicePixelRatio;
  let xs=[],ys=[];
  ch.series.forEach(s=>s.pts.forEach(p=>{{xs.push(p[0]);ys.push(p[1]);}}));
  let x0=Math.min(...xs),x1=Math.max(...xs),y0=Math.min(...ys),y1=Math.max(...ys);
  if(y1-y0<1)y1=y0+1; if(x1-x0<1e-6)x1=x0+1;
  const pad=(y1-y0)*0.08; y0-=pad; y1+=pad;
  const PX=v=>P+(v-x0)/(x1-x0)*(w-1.5*P), PY=v=>h-P-(v-y0)/(y1-y0)*(h-2*P);
  x.clearRect(0,0,w,h);
  x.font=(11*devicePixelRatio)+'px system-ui'; x.fillStyle='#888'; x.strokeStyle='#8883';
  for(let k=0;k<=5;k++){{const v=y0+(y1-y0)*k/5,Y=PY(v);
    x.beginPath();x.moveTo(P,Y);x.lineTo(w-.5*P,Y);x.stroke();x.fillText(v.toFixed(0),4,Y+4);}}
  x.strokeStyle='#8886';x.strokeRect(P,P,w-1.5*P,h-2*P);
  ch.series.forEach(s=>{{
   x.strokeStyle=s.color;x.lineWidth=(s.width||1.5)*devicePixelRatio;
   x.setLineDash(s.dash?[6*devicePixelRatio,4*devicePixelRatio]:[]);
   x.beginPath();s.pts.forEach((p,i)=>{{const X=PX(p[0]),Y=PY(p[1]);i?x.lineTo(X,Y):x.moveTo(X,Y);}});
   x.stroke();
  }});
  x.setLineDash([]);
 }}
 draw();addEventListener('resize',draw);
}}
document.querySelectorAll('canvas[data-ch]').forEach(cv=>chart(cv,CH[cv.dataset.ch]));
</script>
"""


def render(name, charts, summary):
    # tables
    g = summary.get("vel_gains") or {}
    t1 = ("<table><tr><th>plant</th><th></th></tr>"
          f"<tr><th>static gain K</th><td>{summary['K_rpm_per_cmd']} rpm/cmd</td></tr>"
          f"<tr><th>time constant τ</th><td>{summary['tau_s']} s</td></tr>"
          f"<tr><th>transport delay</th><td>{summary['delay_s']} s</td></tr>"
          f"<tr><th>spin floor (deadband)</th><td>thrust {summary['deadband_thrust']}</td></tr>"
          f"<tr><th>no-load top</th><td>{summary['top_rpm']} rpm</td></tr></table>")
    t2 = ("<table><tr><th>velocity PID (encoder)</th><th></th></tr>"
          f"<tr><th>kp</th><td>{g.get('kp')}</td></tr>"
          f"<tr><th>ki</th><td>{g.get('ki')}</td></tr>"
          f"<tr><th>kd</th><td>{g.get('kd')}</td></tr></table>")
    t3 = ("<table><tr><th>world-model</th><th></th></tr>"
          f"<tr><th>VAF (out-of-sample)</th><td>{summary['wm_vaf']}%</td></tr>"
          f"<tr><th>RMSE</th><td>{summary['wm_rmse']} rpm</td></tr></table>")
    # velocity tracking metric table
    segs = summary.get("vel_segments") or []
    seg_rows = "".join(
        f"<tr><td>{s['target']:.0f}</td><td>{s.get('steady_err',0):+.1f}</td>"
        f"<td>{s.get('ripple',0):.0f}</td><td>{s.get('rise_s')}</td></tr>" for s in segs)
    t4 = ("<table><tr><th>vel target</th><th>steady err</th><th>ripple</th><th>rise s</th></tr>"
          f"{seg_rows}</table>") if segs else ""
    tables = f"<div style='display:flex;gap:2rem;flex-wrap:wrap'>{t1}{t2}{t3}</div>{t4}"

    # charts html
    ch_html = []
    for c in charts:
        leg = " ".join(
            f"<span><span class=sw style='background:{s['color']}'></span>{s['label']}</span>"
            for s in c["series"])
        ch_html.append(f"<h2>{c['title']}</h2><canvas data-ch='{c['id']}'></canvas>"
                       f"<div class=leg>{leg}</div>")
    ch_map = {c["id"]: {"series": c["series"]} for c in charts}
    return HTML.format(name=name, tables=tables, charts="\n".join(ch_html),
                       charts_json=json.dumps(ch_map))


def main(argv=None):
    ap = argparse.ArgumentParser(description="consolidated campaign HTML report")
    ap.add_argument("--name", default="350kv")
    ap.add_argument("--out")
    opts = ap.parse_args(argv)
    charts, summary = build(opts.name)
    html = render(opts.name, charts, summary)
    out = opts.out or os.path.join(REPORT, f"campaign_{opts.name}.html")
    with open(out, "w", encoding="utf-8") as fh:
        fh.write(html)
    print(f"# wrote {out}  ({len(charts)} charts)")
    print(f"# summary: K={summary['K_rpm_per_cmd']} rpm/cmd  τ={summary['tau_s']}s  "
          f"top={summary['top_rpm']}rpm  vel_gains={summary.get('vel_gains')}  "
          f"wm_VAF={summary['wm_vaf']}%")


if __name__ == "__main__":
    main()
