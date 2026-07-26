#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""dob_bench — bench-test the rpm-domain disturbance observer (DOB) on real hardware.

No external load rig: we inject an INPUT disturbance (a transient offset added to the ESC command
the loop doesn't know about) — rpm suddenly drops as if a load appeared. The DOB infers it from the
model deficit `rpm_static(cmd_intended) - rpm_measured` and feeds it forward. Runs PI+FF then PI+FF+DOB
against the SAME injected disturbance and compares peak error / settle / IAE.

Conservative: moderate 6-step target, bounded offset, short window, ALWAYS disarms on every exit.
"""
from __future__ import annotations

import argparse
import math
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from esctool import EscHost, find_pico            # noqa: E402
import yaml                                        # noqa: E402


def load_curve(path):
    d = yaml.safe_load(open(path))
    return sorted((p["thrust"], p["rpm"]) for p in d["points"])


def rpm_static(curve, cmd):
    x = abs(cmd)
    if x <= curve[0][0]:
        return curve[0][1]
    if x >= curve[-1][0]:
        return curve[-1][1]
    for i in range(1, len(curve)):
        if x <= curve[i][0]:
            (x0, y0), (x1, y1) = curve[i - 1], curve[i]
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    return curve[-1][1]


def cmd_for(curve, rpm):
    for i in range(1, len(curve)):
        if rpm <= curve[i][1]:
            (x0, y0), (x1, y1) = curve[i - 1], curve[i]
            return x0 + (x1 - x0) * (rpm - y0) / max(1e-6, y1 - y0)
    return curve[-1][0]


def slope(curve, cmd, h=10.0):
    return max(0.05, (rpm_static(curve, cmd + h) - rpm_static(curve, cmd - h)) / (2 * h))


def _encv(dev):
    try:
        lns = dev.cmd("encv", timeout=2)
    except Exception:
        return None
    for ln in lns:
        if ln.startswith("encv|"):
            try:
                return abs(float(ln.split("|")[2]))
            except (ValueError, IndexError):
                return None
    return None


def run_loop(dev, curve, opts, use_dob):
    """One closed-loop run with an injected command-offset disturbance. Returns [(t, err)]."""
    ff = cmd_for(curve, opts.target)
    integ = d_hat = 0.0
    applied_prev = ff
    series = []
    t0 = time.time()
    while time.time() - t0 < opts.secs:
        t = time.time() - t0
        rpm = _encv(dev)
        if rpm is None:
            continue
        e = opts.target - rpm
        integ = max(-opts.trim, min(opts.trim, integ + opts.ki * e * opts.dt))
        trim = max(-opts.trim, min(opts.trim, opts.kp * e + integ))
        comp = 0.0
        if use_dob:
            d_meas = rpm_static(curve, applied_prev) - rpm
            a = 1.0 - math.exp(-opts.dt / opts.qtau)
            d_hat += a * (d_meas - d_hat)
            comp = d_hat / slope(curve, ff)
        cmd_intended = ff + trim + comp
        applied_prev = cmd_intended
        # injected INPUT disturbance: the loop doesn't know about this offset
        disturb = -opts.dist if opts.t_on <= t < opts.t_on + opts.dwin else 0.0
        cmd_sent = max(0, min(1000, cmd_intended + disturb))
        dev.cmd(f"thrust {opts.esc_index} {opts.sign * int(cmd_sent)}", timeout=2)
        series.append((t, opts.target - rpm))
        time.sleep(opts.dt)
    return series


def metrics(series, target, t_on):
    w = [abs(e) for t, e in series if t_on <= t < t_on + 1.5]
    if not w:
        return 0, None, 0
    peak = max(w)
    settle = None
    for t, e in series:
        if t >= t_on and abs(e) <= 0.03 * target:
            # require it to STAY settled briefly
            settle = t - t_on
    iae = sum(abs(e) for t, e in series if t >= t_on) * (series[1][0] - series[0][0] if len(series) > 1 else 0.03)
    return peak, settle, iae


def main():
    ap = argparse.ArgumentParser(description="bench-test the DOB via an injected input disturbance")
    ap.add_argument("--profile", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                      "profiles", "f2838_350kv_bench_20260726.yaml"))
    ap.add_argument("--target", type=float, default=1600.0, help="mech rpm setpoint (6-step)")
    ap.add_argument("--dist", type=float, default=40.0, help="injected cmd offset magnitude (disturbance)")
    ap.add_argument("--t-on", type=float, default=3.0, help="disturbance onset (s into the run)")
    ap.add_argument("--dwin", type=float, default=1.5, help="disturbance window (s)")
    ap.add_argument("--secs", type=float, default=7.0)
    ap.add_argument("--dt", type=float, default=0.03)
    ap.add_argument("--kp", type=float, default=0.06)
    ap.add_argument("--ki", type=float, default=0.25)
    ap.add_argument("--trim", type=float, default=120.0)
    ap.add_argument("--qtau", type=float, default=0.08)
    ap.add_argument("--sign", type=int, default=-1)
    ap.add_argument("--esc-index", type=int, default=1)
    ap.add_argument("--port", default=None)
    opts = ap.parse_args()
    curve = load_curve(opts.profile)

    dev = EscHost(find_pico(opts.port))
    armed = False
    try:
        for c in ("run", "disconnect"):
            try:
                dev.cmd(c, timeout=3)
            except Exception:
                pass
        time.sleep(0.4)
        print(f"# target {opts.target:.0f} rpm, injected cmd offset -{opts.dist:.0f} "
              f"at t={opts.t_on}s for {opts.dwin}s")
        print("# arming…")
        dev.cmd(f"arm {opts.esc_index} bidir", timeout=6)
        armed = True
        time.sleep(4.2)
        results = {}
        traces = {}
        for use_dob in (False, True):
            tag = "PI+FF+DOB" if use_dob else "PI+FF"
            print(f"# run: {tag}")
            series = run_loop(dev, curve, opts, use_dob)
            results[tag] = metrics(series, opts.target, opts.t_on)
            traces[tag] = series
            # effective loop rate + dip depth
            if len(series) > 2:
                rate = len(series) / (series[-1][0] - series[0][0])
                dip = max((e for t, e in series if opts.t_on <= t < opts.t_on + opts.dwin), default=0)
                print(f"#   loop ~{rate:.0f} Hz, max rpm dip during disturbance = {dip:.0f}")
            # let it re-settle between runs
            t0 = time.time()
            while time.time() - t0 < 2.0:
                r = _encv(dev)
                dev.cmd(f"thrust {opts.esc_index} {opts.sign * int(cmd_for(curve, opts.target))}", timeout=2)
                time.sleep(opts.dt)
        print(f"\n  {'':10s} {'peak|err|':>10} {'settle3%':>9} {'IAE':>9}")
        for tag, (p, s, i) in results.items():
            print(f"  {tag:10s} {p:8.0f}rpm {(f'{s:.2f}s' if s else 'no'):>9} {i:9.0f}")
        pf, df = results.get("PI+FF"), results.get("PI+FF+DOB")
        if pf and df and pf[0] > 0:
            print(f"  -> DOB peak {100*(1-df[0]/pf[0]):+.0f}%, IAE {100*(1-df[2]/max(1,pf[2])):+.0f}%")
    finally:
        if armed:
            for _ in range(3):
                try:
                    dev.cmd(f"thrust {opts.esc_index} 0", timeout=2)
                except Exception:
                    pass
            dev.cmd(f"disarm {opts.esc_index}", timeout=2)
            dev.cmd("disarm", timeout=2)
            print("# DISARMED")
        dev.close()


if __name__ == "__main__":
    main()
