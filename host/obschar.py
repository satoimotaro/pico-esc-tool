#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""obschar — soft-sensor / disturbance-observer characterization via PROPELLER self-loading.

No external load rig: a propeller's drag grows ~rpm^2, so an OPEN-LOOP thrust sweep varies the load
automatically. At each level we log the steady mech rpm (AS5600 encv) and compute the host-side
V_eff = rpm/(KV*duty(cmd)) (same model as softsensor.py) — no firmware baseline needed. The V_eff
droop vs load (rpm^2) is the observer characteristic; the fit + raw points feed a plant/observer sim.

Open-loop (RAW thrust) = no closed-loop instability; CONSERVATIVE by default (modest top rpm, short
dwells, stall-abort, ALWAYS disarm on every exit) — safe to run unattended on a fixed bench.
"""
from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from esctool import EscHost, find_pico          # noqa: E402
import softsensor                                # noqa: E402  (duty + estimate)


def _cmd(dev, c, to=3):
    try:
        return dev.cmd(c, timeout=to)
    except Exception:
        return []


def _encv(dev):
    for ln in _cmd(dev, "encv"):
        if ln.startswith("encv|"):
            try:
                return abs(float(ln.split("|")[2]))
            except (ValueError, IndexError):
                return None
    return None


def run(opts):
    dev = EscHost(find_pico(opts.port))
    armed = False
    samples = []          # (cmd, rpm_mech)
    rows = []             # per-level detail
    try:
        _cmd(dev, "run"); _cmd(dev, "disconnect"); time.sleep(0.4)
        print(f"# obschar: KV={opts.kv} pp={opts.pp}  levels={opts.levels}  (open-loop, self-loaded by prop)")
        print("# arming (bidir)…")
        _cmd(dev, f"arm {opts.esc_index} bidir", to=6)
        armed = True
        time.sleep(4.0)
        sign = opts.sign
        t_hard = time.time() + opts.max_secs
        for cmd in opts.levels:
            if time.time() > t_hard:
                print("# hard time cap reached — stopping sweep"); break
            vs = []
            t0 = time.time()
            while time.time() - t0 < opts.dwell:
                _cmd(dev, f"thrust {opts.esc_index} {sign*cmd}")
                v = _encv(dev)
                if v is not None and time.time() - t0 > opts.settle:
                    vs.append(v)
                time.sleep(0.1)
            if not vs:
                print(f"  cmd={cmd}: NO encv — aborting"); break
            rpm = statistics.mean(vs)
            # stall guard: expect the motor to actually be turning in 6-step
            if rpm < opts.stall_rpm:
                print(f"  cmd={cmd}: rpm {rpm:.0f} < stall floor {opts.stall_rpm} — aborting (desync?)"); break
            d = softsensor.duty(cmd)
            vest = rpm / (opts.kv * d) if d > 0.02 else 0.0
            samples.append((cmd, rpm))
            rows.append({"cmd": cmd, "rpm": rpm, "rpm_std": statistics.pstdev(vs),
                         "duty": d, "vest": vest, "load_proxy_rpm2": rpm * rpm})
            print(f"  cmd={cmd:4d}  rpm={rpm:6.0f} (+/-{statistics.pstdev(vs):3.0f})  duty={d:.3f}  vest={vest:5.2f} V")
        _cmd(dev, f"thrust {opts.esc_index} 0")
    finally:
        if armed:
            for _ in range(3):
                _cmd(dev, f"thrust {opts.esc_index} 0")
            _cmd(dev, f"disarm {opts.esc_index}"); _cmd(dev, "disarm")
            print("# DISARMED")
        dev.close()

    if len(samples) < 2:
        print("# too few points to characterize."); return
    est = softsensor.estimate(opts.kv, opts.pp, samples)
    # supply estimate = the lightest-load (lowest rpm^2, i.e. highest V_eff) point
    v_supply = max(r["vest"] for r in rows)
    print("\n# ==== observer characterization ====")
    print(f"# V_eff (least-squares supply)   = {est.get('v_eff', 0):.2f} V")
    print(f"# V_supply (max V_eff, lightest) = {v_supply:.2f} V")
    print(f"# V_eff spread across the sweep  = {est.get('v_spread', 0):.2f} V   (droop range = load signal)")
    print(f"# load coeff  k_load (~rpm^2)    = {est.get('k_load_rpm2', 0):.3e}")
    print(f"# load_present flag              = {est.get('load_present')}")
    model = {"motor": opts.motor, "kv": opts.kv, "pp": opts.pp,
             "v_supply_est": v_supply, "v_eff_ls": est.get("v_eff"),
             "k_load_rpm2": est.get("k_load_rpm2"), "points": rows}
    with open(opts.out, "w") as f:
        json.dump(model, f, indent=2)
    if opts.csv:
        with open(opts.csv, "w") as f:
            f.write("cmd,rpm,rpm_std,duty,vest,load_proxy_rpm2\n")
            for r in rows:
                f.write(f"{r['cmd']},{r['rpm']:.1f},{r['rpm_std']:.1f},{r['duty']:.4f},{r['vest']:.3f},{r['load_proxy_rpm2']:.0f}\n")
    print(f"# wrote model {opts.out}" + (f" + csv {opts.csv}" if opts.csv else ""))


def main():
    ap = argparse.ArgumentParser(description="soft-sensor / observer characterization via prop self-loading")
    ap.add_argument("--kv", type=float, default=350.0)
    ap.add_argument("--pp", type=int, default=7)
    ap.add_argument("--motor", default="f2838_350kv")
    ap.add_argument("--levels", type=lambda s: [int(x) for x in s.split(",")],
                    default=[560, 600, 640, 660], help="open-loop thrust levels (6-step region), ascending")
    ap.add_argument("--sign", type=int, default=-1, help="thrust sign (this motor spins - for + cmd)")
    ap.add_argument("--dwell", type=float, default=2.5, help="s per level")
    ap.add_argument("--settle", type=float, default=1.2, help="s before sampling each level")
    ap.add_argument("--stall-rpm", type=float, default=300.0, help="abort a level below this mech rpm")
    ap.add_argument("--max-secs", type=float, default=40.0, help="hard total-run cap (conservative)")
    ap.add_argument("--esc-index", type=int, default=1)
    ap.add_argument("--port", default=None)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                  "models", "obschar_f2838_350kv.json"))
    ap.add_argument("--csv", default=None)
    opts = ap.parse_args()
    os.makedirs(os.path.dirname(opts.out), exist_ok=True)
    run(opts)


if __name__ == "__main__":
    main()
