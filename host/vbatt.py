#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""vbatt — rough battery-voltage estimate from the ESC soft-sensor (no voltage sensor).

The firmware `sense` reports V_eff = rpm/(KV*duty) = V_supply - I*R/duty, so a single reading sits
BELOW the supply by the load droop. Over gated, steady 6-step samples the MAX (highest percentile)
approaches the true supply (the lightest-load / highest-duty moment). One linear calibration against a
known voltage fixes the absolute scale (a per-motor/config constant — NOT per-startup, NOT per-battery).

  calibrate: spin steady at a known battery voltage, capture the gated high-percentile V_eff,
             store scale = known_V / V_eff  into a small JSON (reused forever).
  monitor:   spin steady, every --period s report V_batt = scale * rolling-high-percentile(V_eff).

Gating (rejects garbage / crossover region): sense valid==1, mech rpm >= --rpm-floor, and the setpoint
has been steady for --settle s. Meant as a bench prototype for the "opportunistic while the thruster
runs" monitor; a lean version can move into firmware later. ALWAYS disarms on exit.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from esctool import EscHost, find_pico  # noqa: E402

DEFAULT_CAL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "vbatt_cal.json")


def _pctl(xs, q):
    """q-th percentile (0..1) of xs by nearest-rank; xs need not be sorted."""
    s = sorted(xs)
    if not s:
        return 0.0
    return s[min(len(s) - 1, int(q * (len(s) - 1) + 0.5))]


def _sense(dev, idx):
    for ln in _cmd(dev, f"sense {idx}"):
        if ln.startswith("sense|"):
            f = {kv.split("=")[0]: kv.split("=")[1] for kv in ln.split("|")[2:] if "=" in kv}
            try:
                return float(f["vest"]), int(f["valid"])
            except (KeyError, ValueError):
                return None
    return None


def _tele_rpm(dev, idx):
    for ln in _cmd(dev, f"tele {idx}"):
        if ln.startswith("tele|"):
            try:
                return abs(int(ln.split("|")[1]))   # MECH rpm (firmware pre-divides by pole pairs)
            except (ValueError, IndexError):
                return None
    return None


def _cmd(dev, c, to=3):
    try:
        return dev.cmd(c, timeout=to)
    except Exception:
        return []


def _sample_loop(dev, idx, opts, on_reading):
    """Drive a steady rpm and feed gated V_eff samples to on_reading(window). Returns the window."""
    win = []
    _cmd(dev, f"motor {idx} {opts.kv} {opts.pp} {opts.known_v if hasattr(opts, 'known_v') and opts.known_v else 11.1}")
    print("# arming…")
    _cmd(dev, f"arm {idx} bidir", to=6)
    time.sleep(4.2)
    _cmd(dev, f"rpm {idx} {opts.rpm}")
    t_steady = time.time() + opts.settle          # setpoint must be steady before we trust samples
    t_end = time.time() + opts.secs
    next_report = time.time() + opts.period
    while time.time() < t_end:
        time.sleep(0.2)
        s = _sense(dev, idx)
        rpm = _tele_rpm(dev, idx)
        if s is None or rpm is None:
            continue
        vest, valid = s
        gated = valid == 1 and rpm >= opts.rpm_floor and time.time() >= t_steady
        if gated and vest > 0:
            win.append(vest)
            win[:] = win[-opts.window:]
        if time.time() >= next_report:
            next_report = time.time() + opts.period
            on_reading(win, rpm, len(win))
    return win


def cmd_calibrate(opts):
    dev = EscHost(find_pico(opts.port))
    try:
        print(f"# calibrate: known battery V = {opts.known_v}, spinning rpm={opts.rpm}")
        win = _sample_loop(dev, opts.esc_index, opts,
                           lambda w, rpm, n: print(f"  rpm={rpm} gated_n={n} "
                                                   f"V_eff.p90={_pctl(w, 0.9):.3f}" if w else f"  rpm={rpm} (warming)"))
        if len(win) < 5:
            sys.exit("not enough gated samples — check the motor spun into 6-step (raise --rpm)")
        veff = _pctl(win, 0.9)
        scale = opts.known_v / veff
        cal = {"scale": scale, "known_v": opts.known_v, "veff_p90": veff,
               "kv": opts.kv, "pp": opts.pp, "rpm": opts.rpm}
        with open(opts.store, "w") as f:
            json.dump(cal, f, indent=2)
        print(f"# V_eff.p90={veff:.3f} @ known {opts.known_v} V  ->  scale={scale:.4f}")
        print(f"# stored {opts.store} — reuse forever (per motor+config; NOT per-startup)")
    finally:
        _disarm(dev, opts.esc_index)


def cmd_monitor(opts):
    scale = 1.0
    if os.path.exists(opts.scale_from):
        cal = json.load(open(opts.scale_from))
        scale = cal["scale"]
        print(f"# scale={scale:.4f} (from {opts.scale_from}, cal @ {cal['known_v']} V)")
    else:
        print(f"# no cal file {opts.scale_from} — reporting RAW V_eff (scale=1.0; run `calibrate` first)")
    dev = EscHost(find_pico(opts.port))

    def report(w, rpm, n):
        if not w:
            print(f"  rpm={rpm} (warming / no gated samples yet)")
            return
        vb = scale * _pctl(w, 0.9)
        print(f"  V_batt ~= {vb:.2f} V   (rpm={rpm}, gated_n={n}, V_eff.p90={_pctl(w, 0.9):.3f})")

    try:
        _sample_loop(dev, opts.esc_index, opts, report)
    finally:
        _disarm(dev, opts.esc_index)


def _disarm(dev, idx):
    for _ in range(3):
        _cmd(dev, f"rpm {idx} 0")
    _cmd(dev, f"disarm {idx}")
    print("# disarmed")
    dev.close()


def main():
    ap = argparse.ArgumentParser(description="battery-voltage estimate from the ESC soft-sensor")
    sub = ap.add_subparsers(dest="mode", required=True)
    for name in ("calibrate", "monitor"):
        p = sub.add_parser(name)
        p.add_argument("--esc-index", type=int, default=1)
        p.add_argument("--kv", type=float, default=350.0)
        p.add_argument("--pp", type=int, default=7)
        p.add_argument("--rpm", type=float, default=1400.0, help="steady mech-rpm setpoint to sample at")
        p.add_argument("--rpm-floor", type=float, default=800.0, help="gate: ignore samples below this (avoids crossover)")
        p.add_argument("--settle", type=float, default=5.0, help="s after the setpoint before trusting samples")
        p.add_argument("--window", type=int, default=25, help="rolling window of gated samples")
        p.add_argument("--period", type=float, default=5.0, help="report/estimate period s (1/5 Hz default)")
        p.add_argument("--secs", type=float, default=20.0, help="total run seconds")
        p.add_argument("--port", default=None)
        if name == "calibrate":
            p.add_argument("--known-v", type=float, required=True, help="the real battery voltage right now")
            p.add_argument("--store", default=DEFAULT_CAL)
        else:
            p.add_argument("--scale-from", default=DEFAULT_CAL)
    opts = ap.parse_args()
    (cmd_calibrate if opts.mode == "calibrate" else cmd_monitor)(opts)


if __name__ == "__main__":
    main()
