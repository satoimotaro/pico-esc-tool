#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""softsensor — infer HIDDEN state (supply voltage, load) from eRPM + the parametric motor model.

"ESC perception": the bidir-DShot ESC gives us eRPM for free. Combined with the KV-parametric model
(rpm = KV*V*eff*duty(cmd) at no load), eRPM + the commanded cmd let us SOLVE for state the ESC has no
sensor for:

  * effective supply V*eff:  from the best-fit SCALE of measured rpm vs KV*duty(cmd). At no/known load
    this IS the battery voltage (x the motor eff). So a voltage sensor is not needed — the motor is the
    sensor.  V_eff = argmin_s  sum( (rpm_meas - s*KV*duty(cmd))^2 ).
  * LOAD signature: with V fixed, the systematic DROOP of measured rpm below the no-load prediction,
    growing with speed, is the load (a prop is ~rpm^2). Reported as a residual trend + a load coeff.

CAVEAT (honest): a single steady point cannot separate V from load — both drop rpm at a given cmd. The
SWEEP separates them: a uniform scale = voltage/eff, a speed-growing droop = load. A dynamic/observer
version (next) uses the transient to separate them online.

Usage:
  python3 softsensor.py --kv 350 --pp 7 estimate --data reports/<file>.json   # offline from (cmd,rpm)
  python3 softsensor.py --kv 350 --pp 7 measure   [--v0 15.2]                  # live: sweep + estimate
"""
from __future__ import annotations

import argparse
import json
import os
import statistics

from pico_esc.motor_model import DUTY6_A, DUTY6_B


def duty(cmd):
    d = DUTY6_A * abs(cmd) + DUTY6_B
    return d if d > 0 else 0.0


def estimate(kv, pp, samples):
    """samples = [(cmd, rpm_mech), ...] in the 6-step region. Returns a dict with V_eff (least-squares
    scale), per-point V, the load-droop residual trend, and a crude load coefficient (rpm^2 fit)."""
    pts = [(c, r) for c, r in samples if duty(c) > 0.02 and abs(r) > 100]
    if len(pts) < 2:
        return {"error": "need >=2 six-step points"}
    # least-squares scale: rpm = s * (KV*duty) => s = <rpm, base> / <base, base>
    num = sum(abs(r) * (kv * duty(c)) for c, r in pts)
    den = sum((kv * duty(c)) ** 2 for c, r in pts)
    v_eff = num / den if den else 0.0
    per_point_v = [(c, abs(r) / (kv * duty(c))) for c, r in pts]
    # residual (measured - no-load prediction at v_eff): a downward trend growing with rpm = load
    resid = [(abs(r), abs(r) - v_eff * kv * duty(c)) for c, r in pts]   # (rpm, residual)
    # crude load coeff: fit residual ~ -k_load * rpm^2  (least squares through origin)
    n2 = sum(-e * (rp * rp) for rp, e in resid)
    d2 = sum((rp * rp) ** 2 for rp, e in resid)
    k_load = n2 / d2 if d2 else 0.0
    droop_hi = resid[-1][1] if resid else 0.0
    return {"v_eff": v_eff, "per_point_v": per_point_v,
            "v_spread": (max(v for _, v in per_point_v) - min(v for _, v in per_point_v)),
            "k_load_rpm2": k_load, "droop_top_rpm": droop_hi,
            "load_present": abs(droop_hi) > 0.05 * (v_eff * kv * max(duty(c) for c, _ in pts))}


def cmd_estimate(kv, pp, opts):
    data = json.load(open(opts.data))
    # accept [[cmd,rpm],...] or a sysid raw {"curve":[{thrust,rpm}]}
    if isinstance(data, dict) and "curve" in data:
        samples = [(p["thrust"], p["rpm"]) for p in data["curve"] if p.get("rpm")]
    else:
        samples = [(c, r) for c, r in data]
    res = estimate(kv, pp, samples)
    _report(res, kv)


def cmd_measure(kv, pp, opts):
    import statistics as st
    from pico_esc.control import DT, VelReader, _pace
    from pico_esc.esc import ESC
    from pico_esc.link import EscHost, RealClock
    host, clock = EscHost(opts.port), RealClock()
    esc = ESC(host, opts.esc, tmax=opts.tmax, clock=clock)
    samples = []
    try:
        esc.prepare(); print("# arming, sweeping 6-step points for the soft-sensor…", flush=True)
        esc.arm(bidir=True)
        vr = VelReader(sign=opts.enc_sign)
        for cmd in opts.cmds:
            teles = []
            t_end = clock.now() + 2.2
            while clock.now() < t_end:
                tick = clock.now()
                vr.read(esc, DT)
                if clock.now() > t_end - 1.0:
                    tel = esc.telemetry()
                    if tel is not None and abs(tel.rpm) > 200:
                        teles.append(abs(tel.rpm))
                esc.thrust(opts.enc_sign * int(cmd))
                _pace(clock, tick)
            if teles:
                samples.append((cmd, st.mean(teles)))
                print(f"  cmd {cmd:4.0f} -> {st.mean(teles):.0f} rpm (tele)", flush=True)
            esc.thrust(0); clock.sleep(0.4)
    finally:
        try: esc.disarm()
        except Exception: pass
        host.close()
    res = estimate(kv, pp, samples)
    _report(res, kv, opts.v0)


def _report(res, kv, v_known=None):
    if "error" in res:
        print("!! " + res["error"]); return
    print(f"\n=== soft-sensor estimate (KV={kv:g}) ===")
    print(f"  V_eff (supply x motor-eff) = {res['v_eff']:.2f} V   (per-point spread {res['v_spread']:.2f} V)")
    if v_known:
        print(f"    known supply {v_known:.1f} V -> implied eff {res['v_eff']/v_known:.3f} "
              f"(load/IR pulls this below 1)")
    print(f"  per-point V:  " + "  ".join(f"{v:.1f}" for _, v in res['per_point_v']))
    print(f"  load signature: droop@top {res['droop_top_rpm']:+.0f} rpm, k_load {res['k_load_rpm2']:.2e} /rpm^2"
          f"  -> {'LOAD detected' if res['load_present'] else '~no load'}")


def main(argv=None):
    ap = argparse.ArgumentParser(description="infer voltage/load from eRPM + KV model (ESC soft-sensor)")
    ap.add_argument("--kv", type=float, required=True)
    ap.add_argument("--pp", type=int, required=True)
    ap.add_argument("--esc", type=int, default=1)
    ap.add_argument("--port")
    ap.add_argument("--enc-sign", type=int, default=-1, choices=(-1, 1))
    ap.add_argument("--tmax", type=int, default=700)
    sub = ap.add_subparsers(dest="cmd", required=True)
    e = sub.add_parser("estimate"); e.add_argument("--data", required=True)
    m = sub.add_parser("measure")
    m.add_argument("--v0", type=float, help="known supply V (for eff readout)")
    m.add_argument("--cmds", type=lambda s: [float(x) for x in s.split(",")], default=[560, 620, 680])
    opts = ap.parse_args(argv)
    {"estimate": cmd_estimate, "measure": cmd_measure}[opts.cmd](opts.kv, opts.pp, opts)


if __name__ == "__main__":
    main()
