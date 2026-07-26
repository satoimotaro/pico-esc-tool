#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""velrun — the bundled velocity-control operational flow, from just KV + pole-pairs, ENCODER-FREE.

Ties the pieces together into one command:
  1. MotorModel(KV, PP, V) predicts the FF curve + ESC crossover config.
  2. (optional) apply the derived ESC config.
  3. (optional) eRPM SELF-CAL using bidir-DShot `tele` only — fits `eff`, no encoder (works in water /
     with the encoder unpotted).
  4. run velocity control: FF from the (self-calibrated) curve + a regime-gated, settle-gated FB trim,
     closing on `tele` (mech RPM). The FB only trims — the FF (kept close by self-cal) does the driving,
     so overshoot stays low; the FB rejects the residual + disturbances (voltage/load changes, water).

Feedback = `tele` (encoder-free). The regime gate keeps FB off below the crossover fold (sine region,
where tele is virtual); the settle gate keeps FB off during setpoint slews (no transient overshoot).
Encoder, if present, is logged as an independent check only.

Usage:
  python3 velrun.py --kv 350 --pp 7 --v 11.1 --apply --selfcal run --targets 900,1500,2000
  python3 velrun.py --kv 350 --pp 7 --v 11.1 --eff 0.99 run --targets 1200,1800 --no-fb   # FF only
"""
from __future__ import annotations

import argparse
import csv
import os
import statistics
import time

from pico_esc.control import DT, VelReader, _pace
from pico_esc.drive import Aborted
from pico_esc.esc import ESC
from pico_esc.link import EscHost, RealClock
from pico_esc.motor_model import MotorModel
from veltune import VelLoop, _seg_metrics, print_segments

REPORT_DIR = os.path.join(os.path.dirname(__file__), "reports")


def log(m):
    print(m, flush=True)


def apply_config(host, idx, cfg):
    """Push the derived ESC config via the same editpage path esctool uses."""
    esc = ESC(host, idx)
    n = esc.config.set(cfg)
    log(f"# applied {n} config byte(s): cross_up={cfg['sine_cross_up']} cross_dn={cfg['sine_cross_dn']} "
        f"low_rpm_prot={cfg['low_rpm_power_protection']}")
    esc.config.restart()


def selfcal(esc, clock, m, cmds, enc_sign):
    """Spin a few 6-step points, read tele eRPM, fit eff. Encoder-free."""
    samples = []
    for cmd in cmds:
        t_end = clock.now() + 2.5
        erpms = []
        while clock.now() < t_end:
            tick = clock.now()
            if clock.now() > t_end - 1.0:
                tel = esc.telemetry()
                if tel is not None and abs(tel.rpm) > 200:
                    erpms.append(abs(tel.rpm) * m.pp)
            esc.thrust(enc_sign * int(cmd))
            _pace(clock, tick)
        if erpms:
            samples.append((cmd, statistics.mean(erpms)))
        esc.thrust(0)
        clock.sleep(0.4)
    if samples:
        eff = m.self_cal_erpm(samples)
        log(f"# self-cal (tele, no encoder): eff={eff:.3f} from {len(samples)} pts")
    else:
        log("# self-cal: no 6-step tele — check config / reachability; keeping eff as-is")
    return m.eff


def run_control(esc, clock, loop, targets, hold, enc_sign, max_rpm, log_enc=True, track_model=None):
    """FF + gated FB velocity control on TELE feedback. Encoder logged as an independent check.

    track_model: if given (a MotorModel used as the FF source), update its `.v` from the live tele
    voltage each tick so the FF auto-compensates for battery sag/change — OPTIONAL, and a no-op until
    the ESC actually reports EDT voltage (tele.volts>0). Warns once if voltage tracking was asked for
    but the ESC reports 0 V."""
    vr = VelReader(sign=enc_sign) if log_enc else None
    loop.reset(0.0)
    rows, segments = [], []
    warned = [False]
    t0 = clock.now()
    for ti, tgt in enumerate(targets):
        seg = []
        t_end = clock.now() + hold
        while clock.now() < t_end:
            tick = clock.now()
            tel = esc.telemetry()
            meas = abs(tel.rpm) if (tel is not None) else 0.0   # tele mech RPM (encoder-free)
            if track_model is not None and tel is not None:
                if tel.volts > 0.5:
                    track_model.v = tel.volts                   # live voltage -> FF auto-compensates
                elif not warned[0]:
                    warned[0] = True
                    log("# !! --track-voltage but tele.volts=0 (EDT voltage not enabled in firmware) "
                        "— FF using the static --v; FB still guarantees.")
            if meas > max_rpm:
                esc.thrust(0)
                raise Aborted(f"overspeed {meas:.0f} > {max_rpm}")
            u, dbg = loop.step(tgt, meas, DT)
            esc.thrust(enc_sign * int(round(u)))
            enc = vr.read(esc, DT) if vr else 0.0
            rows.append({"t": tick - t0, "seg": ti, "target": tgt, "sp": dbg["sp"],
                         "tele": meas, "enc": abs(enc), "thrust": dbg["thrust"], "gate": dbg["gate"]})
            seg.append((tick - t_end + hold, meas, tgt))
            _pace(clock, tick)
        segments.append(_seg_metrics(seg, tgt, 0.6))
    esc.thrust(0)
    return rows, segments


def write_csv(path, rows):
    with open(path, "w", encoding="utf-8", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "seg", "target", "sp", "tele", "enc", "thrust", "gate"])
        for r in rows:
            w.writerow([round(r["t"], 4), r["seg"], round(r["target"], 1), round(r["sp"], 1),
                        round(r["tele"], 1), round(r["enc"], 1), round(r["thrust"], 1),
                        round(r["gate"], 3)])


def _floats(s):
    return [float(x) for x in s.split(",") if x.strip()]


def main(argv=None):
    ap = argparse.ArgumentParser(description="bundled encoder-free velocity control from KV+PP")
    ap.add_argument("--kv", type=float, required=True)
    ap.add_argument("--pp", type=int, required=True)
    ap.add_argument("--v", type=float, default=11.1)
    ap.add_argument("--eff", type=float, default=1.0)
    ap.add_argument("--name", default=None)
    ap.add_argument("--esc", type=int, default=1)
    ap.add_argument("--port")
    ap.add_argument("--enc-sign", type=int, default=-1, choices=(-1, 1))
    ap.add_argument("--apply", action="store_true", help="apply the derived ESC config first")
    ap.add_argument("--selfcal", action="store_true", help="eRPM self-cal (tele, no encoder) before run")
    ap.add_argument("--cal-cmds", type=_floats, default=[560, 620, 690])
    ap.add_argument("--tmax", type=int, default=700)
    ap.add_argument("--max-rpm", type=float, default=2800)
    # controller
    ap.add_argument("--no-fb", action="store_true", help="feed-forward only (no FB trim)")
    ap.add_argument("--kp", type=float, default=0.06)
    ap.add_argument("--ki", type=float, default=2.5)
    ap.add_argument("--kd", type=float, default=0.02)
    ap.add_argument("--slew", type=float, default=800)
    ap.add_argument("--meas-tau", type=float, default=0.14)
    ap.add_argument("--fb-floor", type=float, default=720, help="FB off below this (crossover fold)")
    ap.add_argument("--fb-band", type=float, default=150)
    ap.add_argument("--settle-band", type=float, default=60, help="FB engages within this |target-sp|")
    ap.add_argument("--track-voltage", action="store_true",
                    help="OPTIONAL: auto-compensate FF from live tele voltage (needs EDT voltage in fw)")
    ap.add_argument("--hold", type=float, default=2.5)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("run")
    r.add_argument("--targets", type=_floats, default=[900, 1500, 2000])
    opts = ap.parse_args(argv)

    m = MotorModel(opts.kv, opts.pp, opts.v, eff=opts.eff, name=opts.name or f"{int(opts.kv)}kv")
    log(m.summary())
    host, clock = EscHost(opts.port), RealClock()
    if opts.apply:
        apply_config(host, opts.esc, m.esc_config())
    esc = ESC(host, opts.esc, tmax=opts.tmax, clock=clock)
    ts = time.strftime("%Y%m%d-%H%M%S")
    try:
        esc.prepare()
        log(f"# arming (bidir) enc_sign={opts.enc_sign}")
        esc.arm(bidir=True)
        if opts.selfcal:
            selfcal(esc, clock, m, opts.cal_cmds, opts.enc_sign)
        # FF source: the live MotorModel (voltage-tracking) or a baked static profile
        ff_src = m if opts.track_voltage else m.to_profile()
        kp, ki, kd = (0.0, 0.0, 0.0) if opts.no_fb else (opts.kp, opts.ki, opts.kd)
        loop = VelLoop(ff_src, kp, ki, kd, opts.slew, opts.meas_tau, opts.tmax,
                       fb_floor=opts.fb_floor, fb_band=opts.fb_band, err_band=opts.settle_band)
        rows, segs = run_control(esc, clock, loop, opts.targets, opts.hold, opts.enc_sign, opts.max_rpm,
                                 track_model=(m if opts.track_voltage else None))
        log(f"=== {'FF-only' if opts.no_fb else 'FF + gated FB'} (tele feedback, encoder-free) ===")
        print_segments(segs)
        path = os.path.join(REPORT_DIR, f"velrun_{m.name}_{'ff' if opts.no_fb else 'fb'}_{ts}.csv")
        write_csv(path, rows)
        log(f"  trace: {path}  (eff={m.eff:.3f})")
    except Aborted as e:
        log(f"!! ABORTED: {e}")
    finally:
        try:
            esc.disarm()
        except Exception:
            pass
        host.close()


if __name__ == "__main__":
    main()
