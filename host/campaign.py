#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""campaign — hands-off motor characterization + control-tuning pipeline (one command, no babysitting).

Sequences the lab tools into an unattended campaign and records a resumable state file so a long run
survives interruption / context loss:

  1. sysid    — plant identification (pole/deadband/static-curve/step first-order fit)
  2. veltune  — encoder-feedback velocity PID auto-tune
  3. posctl   — position-gain grid auto-tune on hardware (tune_posctl --real)
  4. world    — fit + validate the plant world-model (offline)
  5. report   — consolidated HTML performance report (campaign_report.py)

Each stage shells out to the standalone tool (same as tune_posctl calls posctl), appends to
<name>_campaign.json, and is skipped on --resume if already done. --dry-run threads through to the
hardware stages (sim). Crossover is disabled first (pure forced-sine) unless --keep-crossover — the
350KV motor cannot 6-step at no load (see the findings note).

Usage:
  python3 campaign.py --name 350kv --max-thrust 850 --max-rpm 400          # full hands-off run
  python3 campaign.py --name 350kv --resume                                # continue where it stopped
  python3 campaign.py --name sim --dry-run --profile-only                  # pipeline smoke on the sim
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
STATE = lambda name: os.path.join(HERE, "models", f"{name}_campaign.json")


def load_state(name):
    p = STATE(name)
    return json.load(open(p)) if os.path.exists(p) else {"name": name, "stages": {}}


def save_state(st):
    os.makedirs(os.path.join(HERE, "models"), exist_ok=True)
    json.dump(st, open(STATE(st["name"]), "w"), indent=2)


def run_tool(argv, log_tag):
    """Run a host tool as a subprocess, streaming output. Returns (ok, tail_text)."""
    print(f"\n===== [{log_tag}] {' '.join(argv)} =====", flush=True)
    proc = subprocess.run([sys.executable] + argv, cwd=HERE, capture_output=True, text=True)
    out = (proc.stdout or "") + (proc.stderr or "")
    print(out[-2000:], flush=True)
    return proc.returncode == 0, out


def stage_done(st, key):
    return st["stages"].get(key, {}).get("ok")


def mark(st, key, ok, note=""):
    st["stages"][key] = {"ok": ok, "note": note, "ts": time.strftime("%Y-%m-%dT%H:%M:%S")}
    save_state(st)


def main(argv=None):
    ap = argparse.ArgumentParser(description="hands-off characterization + tuning campaign")
    ap.add_argument("--name", default="motor")
    ap.add_argument("--port")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--resume", action="store_true", help="skip stages already marked ok")
    ap.add_argument("--enc-sign", type=int, default=-1, choices=(-1, 1))
    ap.add_argument("--max-thrust", type=int, default=850)
    ap.add_argument("--max-rpm", type=float, default=400)
    ap.add_argument("--keep-crossover", action="store_true",
                    help="do NOT disable the sine->6step crossover first")
    ap.add_argument("--vel-targets", default="120,200,160,240")
    ap.add_argument("--only", help="run only these comma stages (sysid,veltune,posctl,world,report)")
    opts = ap.parse_args(argv)

    st = load_state(opts.name)
    st["config"] = vars(opts)
    save_state(st)
    dry = ["--dry-run"] if opts.dry_run else []
    port = ["--port", opts.port] if opts.port else []
    only = set(opts.only.split(",")) if opts.only else None
    want = lambda k: (only is None or k in only) and not (opts.resume and stage_done(st, k))

    prof = os.path.join("profiles", f"{opts.name}_sine_sysid.yaml")
    model = os.path.join("models", f"{opts.name}_sine_sysid.json")

    # 0. disable crossover (hardware only) — the no-load 350KV cannot 6-step
    if not opts.dry_run and not opts.keep_crossover and want("crossover"):
        ok, _ = run_tool(["esctool.py", *port, "set", "1",
                          "sine_cross_up=0", "sine_cross_dn=0"], "crossover-off")
        mark(st, "crossover", ok, "pure forced-sine")

    # 1. sysid
    if want("sysid"):
        ok, _ = run_tool(["sysid.py", *dry, *port, "--name", f"{opts.name}_sine",
                          "--max-thrust", str(opts.max_thrust), "--max-rpm", str(opts.max_rpm),
                          "--curve-points", "8", "--step-secs", "1.5"], "sysid")
        mark(st, "sysid", ok)

    # 2. velocity PID tune
    if want("veltune"):
        ok, _ = run_tool(["veltune.py", *dry, *port, "--name", opts.name, "--profile", prof,
                          "--model", model, "--enc-sign", str(opts.enc_sign),
                          "--max-rpm", str(opts.max_rpm), "--tmax", str(opts.max_thrust),
                          "--slew", "500", "--meas-tau", "0.15", "--rounds", "2",
                          "tune", "--targets", opts.vel_targets], "veltune")
        mark(st, "veltune", ok)

    # 3. position gain tune (hardware grid; sim grid under --dry-run)
    if want("posctl"):
        real = [] if opts.dry_run else ["--real"]
        ok, _ = run_tool(["tune_posctl.py", *real, "--kp", "6,9,12", "--kd", "1.0,1.6,2.2",
                          "--vmax", "400", "--moves", "360,-360", "--tmax",
                          str(min(600, opts.max_thrust)), "--ov-penalty", "0.1"], "posctl-tune")
        mark(st, "posctl", ok)

    # 4. world-model fit + validate (offline)
    if want("world"):
        ok, _ = run_tool(["worldmodel.py", "--name", f"{opts.name}_sine", "validate"], "worldmodel")
        mark(st, "world", ok)

    # 5. consolidated report (offline)
    if want("report"):
        rep = os.path.join(HERE, "campaign_report.py")
        if os.path.exists(rep):
            ok, _ = run_tool(["campaign_report.py", "--name", opts.name], "report")
            mark(st, "report", ok)
        else:
            print("# campaign_report.py not present — skipping report stage")

    print("\n===== campaign state =====")
    for k, v in st["stages"].items():
        print(f"  {k:12} {'OK ' if v['ok'] else 'FAIL'}  {v.get('note', '')}")
    print(f"# state: {STATE(opts.name)}")


if __name__ == "__main__":
    main()
