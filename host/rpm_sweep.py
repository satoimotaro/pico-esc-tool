#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""
rpm_sweep — closed-loop velocity-controller evaluation harness.

Drives one ESC through a stepped RPM schedule over the Pico's USB-serial CLI (`arm`, `rpm <i> <v>`,
`tele <i>`), logs measured shaft RPM at the full telemetry rate, computes per-segment tracking metrics
(steady-state error, over/undershoot, rise, settle), and writes a self-contained HTML report.

    python3 host/rpm_sweep.py 1                       # default staircase, writes host/reports/rpm_sweep.*
    python3 host/rpm_sweep.py 1 --targets 1500,3000,4500,0 --hold 4
    python3 host/rpm_sweep.py 1 --out host/reports/tuned   # -> tuned.json + tuned.html
    python3 host/rpm_sweep.py 1 --no-report               # data only

Shaft RPM comes from bidirectional-DShot eRPM telemetry, which the firmware has ALREADY divided by
pole pairs (so it is mechanical). The slew reference reconstructs the controller's internal setpoint
ramp (--slew, default 4000 rpm/s = main.cpp's esc1.vc.slew_rpm_s) for a fair tracking overlay.
"""
import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__)))
from pico_esc.link import EscHost  # noqa: E402

# A segment is (hold_seconds, setpoint_rpm). The default exercises a cold start (rest -> sine -> 6-step),
# up/down staircase steps, a big down-step, the low-speed floor, and the neutral stop.
DEFAULT_SCHEDULE = [
    (5.0, 2500), (4.0, 1500), (4.0, 3500), (4.0, 4500), (4.0, 2500),
    (4.0, 4000), (4.0, 1500), (4.0, 3000), (3.0, 800), (4.0, 0),
]


def probe_encoder(host):
    """True if an AS5600 shaft encoder answers `encv` (independent ground-truth speed)."""
    try:
        for ln in host.cmd("encv", timeout=1.0):
            if ln.startswith("encv|"):
                return True
    except RuntimeError:                                     # device replies 'err' when no sensor
        pass
    return False


def _tele_rpm(host, esc):
    for ln in host.cmd(f"tele {esc}", timeout=1.0):
        if ln.startswith("tele|"):
            try:
                return int(ln.split("|")[1])
            except (ValueError, IndexError):
                return None
    return None


def _enc_rpm(host):
    # `encv` -> "encv|<accum>|<rpm>|<samples>|<md>"; field [2] is the de-aliased mechanical RPM.
    try:
        for ln in host.cmd("encv", timeout=1.0):
            if ln.startswith("encv|"):
                try:
                    return float(ln.split("|")[2])
                except (ValueError, IndexError):
                    return None
    except RuntimeError:
        return None
    return None


def run_sweep(host, esc, schedule, arm_wait=3.3, use_enc=False):
    """Arm, walk the schedule, sample `tele` (and `encv` if present) at the link rate. Returns samples."""
    print(f"arming ESC {esc} ...")
    host.cmd(f"arm {esc}")
    time.sleep(arm_wait)

    samples = []
    t0 = time.time()
    seg = 0
    host.cmd(f"rpm {esc} {schedule[0][1]}")
    setpoint = schedule[0][1]
    deadline = t0 + schedule[0][0]
    while True:
        now = time.time()
        if now >= deadline:
            seg += 1
            if seg >= len(schedule):
                break
            setpoint = schedule[seg][1]
            host.cmd(f"rpm {esc} {setpoint}")
            deadline = now + schedule[seg][0]
        r = _tele_rpm(host, esc)
        if r is None:
            continue
        smp = {"t": round(now - t0, 4), "sp": setpoint, "rpm": r}
        if use_enc:
            e = _enc_rpm(host)
            if e is not None:
                smp["enc"] = round(e, 1)
        samples.append(smp)
    host.cmd(f"disarm {esc}")
    return samples


def analyze(samples, schedule, slew):
    """Attach the slew-limited reference to each sample and compute per-segment metrics."""
    segs, t = [], 0.0
    for dur, sp in schedule:
        segs.append({"t0": t, "t1": t + dur, "sp": sp})
        t += dur

    def sp_at(tt):
        for s in segs:
            if s["t0"] <= tt < s["t1"]:
                return s["sp"]
        return segs[-1]["sp"]

    cur, prev_t = 0.0, 0.0
    for smp in samples:
        dt = smp["t"] - prev_t
        prev_t = smp["t"]
        tgt = sp_at(smp["t"])
        cur = min(tgt, cur + slew * dt) if cur < tgt else max(tgt, cur - slew * dt)
        smp["ref"] = round(cur, 1)

    metrics = []
    for i, s in enumerate(segs):
        ss = [x for x in samples if s["t0"] <= x["t"] < s["t1"]]
        if not ss:
            continue
        sp, prev = s["sp"], (segs[i - 1]["sp"] if i > 0 else 0)
        tail_s = [x for x in ss if x["t"] >= s["t1"] - 1.2]
        tail = [x["rpm"] for x in tail_s]
        ssv = sum(tail) / len(tail) if tail else float("nan")
        err_pct = None if sp == 0 else round((ssv - sp) / sp * 100, 2)
        # encoder ground-truth steady-state, if logged (independent of the eRPM feedback)
        enc_tail = [x["enc"] for x in tail_s if "enc" in x]
        ss_enc = round(sum(enc_tail) / len(enc_tail), 1) if enc_tail else None
        rpms = [x["rpm"] for x in ss]
        over = rise = settle = None
        if sp > prev and sp > 0:                              # up-step: overshoot + 10-90 rise
            over = round((max(rpms) - sp) / sp * 100, 1)
            lo, hi = prev + 0.1 * (sp - prev), prev + 0.9 * (sp - prev)
            tl = th = None
            for x in ss:
                if tl is None and x["rpm"] >= lo:
                    tl = x["t"]
                if th is None and x["rpm"] >= hi:
                    th = x["t"]
                    break
            if tl is not None and th is not None:
                rise = round(th - tl, 3)
        elif sp < prev and sp > 0:                            # down-step: undershoot
            over = round((sp - min(rpms)) / sp * 100, 1)
        if sp > 0:                                            # settle: last exit from +/-5% band
            band = max(50.0, 0.05 * sp)
            lastout = s["t0"]
            for x in ss:
                if abs(x["rpm"] - sp) > band:
                    lastout = x["t"]
            settle = round(lastout - s["t0"], 2)
        metrics.append({
            "i": i, "sp": sp, "prev": prev, "ss": round(ssv, 1),
            "err_pct": err_pct, "err_abs": round(ssv - sp, 1),
            "ss_enc": ss_enc,
            "overshoot_pct": over, "rise_s": rise, "settle_s": settle,
        })
    return segs, metrics


def summarize(metrics):
    drive = [m for m in metrics if m["sp"] >= 1500 and m["err_pct"] is not None]
    ups = [m["overshoot_pct"] for m in metrics
           if m["overshoot_pct"] is not None and m["sp"] > m["prev"] and m["sp"] > 0]
    return {
        "mean_ss_err": round(sum(abs(m["err_pct"]) for m in drive) / len(drive), 2) if drive else None,
        "max_ss_err": round(max(abs(m["err_pct"]) for m in drive), 2) if drive else None,
        "max_overshoot": round(max(ups), 1) if ups else None,
    }


def decimate(samples, n=1600):
    step = max(1, len(samples) // n)
    return samples[::step]


def main():
    ap = argparse.ArgumentParser(description="Closed-loop RPM tracking sweep + evaluation report.")
    ap.add_argument("esc", type=int, help="ESC index to drive (e.g. 1)")
    ap.add_argument("--port", help="serial port (default: auto-detect VID 2E8A)")
    ap.add_argument("--targets", help="comma-separated RPM setpoints (overrides the default schedule)")
    ap.add_argument("--hold", type=float, default=4.0, help="seconds per --targets step (default 4)")
    ap.add_argument("--slew", type=float, default=4000.0,
                    help="controller slew rate for the reference overlay (default 4000 rpm/s)")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VAL",
                    help="push a gain before the sweep, e.g. --set slew=1500 --set kd=0.008 "
                         "(KEY in kp|ki|kd|dtau|trim|slew). Repeatable. slew= also sets the reference.")
    ap.add_argument("--out", default="host/reports/rpm_sweep",
                    help="output path prefix (writes <out>.json and <out>.html)")
    ap.add_argument("--no-report", action="store_true", help="write only the JSON data")
    ap.add_argument("--title", default="", help="extra label shown in the report header")
    args = ap.parse_args()

    if args.targets:
        schedule = [(args.hold, int(v)) for v in args.targets.split(",")]
    else:
        schedule = DEFAULT_SCHEDULE

    host = EscHost(args.port)
    try:
        for kv in args.set:                                  # push gains before driving
            k, _, v = kv.partition("=")
            host.cmd(f"gain {args.esc} {k.strip()} {v.strip()}")
            print(f"set gain {k.strip()} = {v.strip()}")
            if k.strip() == "slew":
                args.slew = float(v)                         # keep the reference overlay consistent
        use_enc = probe_encoder(host)
        print("shaft encoder (AS5600): " + ("present -> logging ground-truth speed" if use_enc else "not found"))
        samples = run_sweep(host, args.esc, schedule, use_enc=use_enc)
    finally:
        host.close()
    if not samples:
        sys.exit("no samples collected")

    segs, metrics = analyze(samples, schedule, args.slew)
    summ = summarize(metrics)
    dur = samples[-1]["t"]
    has_enc = any("enc" in x for x in samples)
    print(f"\nlogged {len(samples)} samples over {dur:.1f}s (~{len(samples)/dur:.0f} Hz)"
          + ("  [+encoder]" if has_enc else ""))
    print(f"steady-state error (>=1500 rpm): mean {summ['mean_ss_err']}%  worst {summ['max_ss_err']}%")
    print(f"peak up-step overshoot: {summ['max_overshoot']}%")
    enc_hdr = f"{'enc':>7}" if has_enc else ""
    print(f"\n{'sp':>5} {'tele':>7}{enc_hdr} {'err%':>7} {'over%':>7} {'rise':>6} {'settle':>7}")
    for m in metrics:
        enc_col = f"{m['ss_enc']:>7.0f}" if (has_enc and m.get('ss_enc') is not None) else (f"{'—':>7}" if has_enc else "")
        print(f"{m['sp']:>5} {m['ss']:>7.0f}{enc_col} {str(m['err_pct']):>7} "
              f"{str(m['overshoot_pct']):>7} {str(m['rise_s']):>6} {str(m['settle_s']):>7}")

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    payload = {
        "title": args.title, "slew": args.slew, "schedule": schedule,
        "duration": dur, "rate_hz": round(len(samples) / dur, 1),
        "summary": summ, "segs": segs, "metrics": metrics, "samples": samples,
    }
    with open(args.out + ".json", "w") as f:
        json.dump(payload, f)
    print(f"\ndata  -> {args.out}.json")

    if not args.no_report:
        from rpm_sweep_report import build_report          # local module (report template)
        trace = decimate(samples)
        html = build_report(trace, segs, metrics, summ, args.slew, dur, args.title, has_enc)
        with open(args.out + ".html", "w") as f:
            f.write(html)
        print(f"report-> {args.out}.html")


if __name__ == "__main__":
    main()
