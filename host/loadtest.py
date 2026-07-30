#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""loadtest — hold one rpm while an operator varies the load by hand, and test whether the control
effort predicts the current (i.e. the torque) without a current sensor.

THE CLAIM UNDER TEST. At a fixed rpm the feed-forward already supplies the no-load duty, so every
extra bit of duty the loop adds is there to push current through the winding:

    V*(d_ff + dd) = E + I*R   and   V*d_ff = E   =>   I_phase = V*dd/R,   dd = trim * DUTY6_A
    torque = K_T * I_phase = (9.5493/KV) * V * trim * DUTY6_A / R

so `trim` — a number the controller already has — becomes an ABSOLUTE torque estimate given only
KV, V and R. No current sensor, no force sensor. That is the whole point of making the feed-forward a
true no-load physical model rather than a curve measured under some load.

The bus current is what falsifies it: `I_dc = I_phase * duty`, and the PSU meters that directly.
A straight line through the origin with the predicted slope confirms the relation; curvature, an
offset, or a wrong slope does not.

DOB is forced OFF: with it on the correction SPLITS between `trim` and the observer's compensation
and neither alone is the control effort.

Load by hand against the motor BELL through a wood/rubber block — never the propeller. A moderate
rpm keeps the bell surface speed low. `trim_max` (80) is left alone on purpose: it caps how hard the
loop can fight the hand, and it still allows ~1 A of added bus current.

  # 1) run it; film the PSU display. The console prints a wall clock to align the video to.
  python3 host/loadtest.py 1 --rpm 1200 --minutes 3

  # 2) read currents off the video and pair them by clock time — no second motor run
  python3 host/loadtest.py --analyze host/reports/loadtest_<ts>.csv \
      --currents 10:31:05=0.061,10:31:20=0.118,10:31:35=0.190
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from esctool import EscHost, find_pico            # noqa: E402
from pico_esc.motor_model import DUTY6_A, DUTY6_CROSSUP_REF, duty6   # noqa: E402


class Aborted(RuntimeError):
    pass


def predicted_i_dc_per_trim(kv, v, r, cross_up, rpm):
    """d(I_dc)/d(trim) [A per ESC command unit] at this operating point."""
    duty = duty6(cmd_for_rpm(kv, v, cross_up, rpm), cross_up)
    return v * DUTY6_A / r * duty


def cmd_for_rpm(kv, v, cross_up, rpm):
    from pico_esc.motor_model import cmd_for_duty6
    return cmd_for_duty6(abs(rpm) / (kv * v), cross_up)


def _fields(dev, cmd, prefix):
    try:
        lines = dev.cmd(cmd, timeout=4)
    except Exception:
        return None
    for ln in lines:
        if ln.startswith(prefix):
            return {kv.split("=")[0]: kv.split("=")[1] for kv in ln.split("|")[1:] if "=" in kv}
    return None


def _tele_rpm(dev, idx):
    try:
        for ln in dev.cmd(f"tele {idx}", timeout=4):
            if ln.startswith("tele|"):
                return float(ln.split("|")[1])
    except Exception:
        pass
    return float("nan")


def run(dev, opts, state):
    idx = opts.index
    dev.cmd(f"gain {idx} dob 0", timeout=4)
    print("# dob -> 0 (the observer must not absorb part of the load)")
    dev.cmd(f"arm {idx} bidir", timeout=8)
    time.sleep(3.0)
    dev.cmd(f"rpm {idx} {opts.rpm:g}", timeout=6)

    t_wait = time.time()
    while True:
        s = _fields(dev, f"sense {idx}", "sense|") or {}
        if s.get("valid") == "1" and float(s.get("settled", 0)) >= 1.0:
            break
        if time.time() - t_wait > opts.settle_timeout:
            raise Aborted(f"no settled 6-step lock in {opts.settle_timeout:g}s (valid={s.get('valid')})")
        time.sleep(0.5)
    time.sleep(3.0)                                    # let the PI's own transient die first
    print(f"# locked and settled at {opts.rpm:g} rpm\n")

    slope = predicted_i_dc_per_trim(opts.kv, opts.v, opts.r, opts.cross_up, opts.rpm)
    print(f"# PREDICTION: I_dc rises {slope:.4f} A per command unit of trim")
    print("# apply load to the BELL through a block — NOT the propeller. Vary it: "
          "none -> light -> medium -> hard -> release, a few times.\n")
    print(f"# {'clock':>10} {'t':>6} {'trim':>7} {'cmd':>6} {'rpm':>7} {'vest':>6}  predicted I_dc")

    # Optional SCHEDULE: announce a target PSU current and hold it. Steady beats varied — the first
    # attempt varied the load on purpose and the hand fought the loop (trim swung 27..80, rpm
    # 960..1524), so two nominally identical plateaus disagreed by 33%. Holding the CURRENT constant
    # also demotes the PSU display from a measurement that must be time-aligned to the operator's
    # control target, which removes its lag from the problem entirely.
    sched = []
    if opts.levels:
        sched = [float(x) for x in opts.levels.split(",")]
        total = opts.baseline_secs * 2 + len(sched) * opts.level_secs
        print(f"# SCHEDULE ({total:.0f}s): baseline, then hold each level {opts.level_secs:g}s, "
              f"then baseline. Watch the PSU AMPS and adjust pressure to keep it AT the target.")

    t0, deadline, blind = time.time(), time.time() + opts.minutes * 60.0, 0
    if sched:
        deadline = t0 + opts.baseline_secs * 2 + len(sched) * opts.level_secs
    phase, said = None, None
    while time.time() < deadline:
        now = time.time()
        s = _fields(dev, f"sense {idx}", "sense|") or {}
        rpm = _tele_rpm(dev, idx)
        if not s:
            blind += 1
            if blind >= opts.max_blind:
                raise Aborted(f"no sense reply for {blind} samples — stopping rather than running blind")
            time.sleep(opts.period)
            continue
        blind = 0
        trim = float(s.get("trim", "nan"))
        row = {"clock": time.strftime("%H:%M:%S", time.localtime(now)), "t": now - t0,
               "trim": trim, "cmd": float(s.get("cmd", "nan")), "tele_rpm": rpm,
               "vest": float(s.get("vest", "nan")), "load": float(s.get("load", "nan")),
               "dhat": float(s.get("dhat", "nan")), "valid": s.get("valid", "0")}
        state["rows"].append(row)

        if rpm == rpm and abs(rpm) > opts.rpm_ceiling:
            raise Aborted(f"overspeed: {rpm:.0f} > {opts.rpm_ceiling:.0f} rpm")
        if rpm == rpm and abs(rpm) < opts.stall_rpm:
            raise Aborted(f"stall/desync: {rpm:.0f} < {opts.stall_rpm:.0f} rpm — release the load")

        if sched:
            el = time.time() - t0
            if el < opts.baseline_secs:
                phase = "BASELINE — hands OFF"
            elif el < opts.baseline_secs + len(sched) * opts.level_secs:
                k = int((el - opts.baseline_secs) // opts.level_secs)
                left = opts.baseline_secs + (k + 1) * opts.level_secs - el
                phase = f"HOLD {sched[k]:.2f} A   ({left:.0f}s left)"
            else:
                phase = "BASELINE — hands OFF"
            if phase.split("(")[0] != (said or "").split("(")[0]:
                print(f"\n>>> {phase}\n")
            said = phase
        row["phase"] = phase or ""

        if len(state["rows"]) % opts.print_every == 0:
            print(f"  {row['clock']:>10} {row['t']:6.0f} {trim:7.1f} {row['cmd']:6.0f} "
                  f"{rpm:7.0f} {row['vest']:6.2f}   +{trim * slope:.3f} A")
        time.sleep(opts.period)


def analyze(rows, pairs, opts):
    """Pair operator-read PSU currents (by clock time) with the logged trim, and fit I_dc vs trim."""
    by_clock = {}
    for r in rows:
        by_clock.setdefault(r["clock"], []).append(r)
    pts = []
    for clock, i_dc in pairs:
        got = by_clock.get(clock)
        if not got:
            print(f"# no log sample at {clock} — skipped")
            continue
        trim = sum(float(g["trim"]) for g in got) / len(got)
        rpm = sum(float(g["tele_rpm"]) for g in got) / len(got)
        pts.append((trim, i_dc - opts.idle, rpm))

    if len(pts) < 3:
        print("# need at least 3 paired points")
        return
    slope_pred = predicted_i_dc_per_trim(opts.kv, opts.v, opts.r, opts.cross_up, opts.rpm)
    print(f"\n{'trim':>8}{'I_dc-idle[A]':>14}{'rpm':>8}{'predicted[A]':>14}")
    for trim, i, rpm in sorted(pts):
        print(f"{trim:8.1f}{i:14.4f}{rpm:8.0f}{trim * slope_pred:14.4f}")

    # least squares through a free intercept: the intercept IS the no-load current, which must come
    # out near the drag-only value rather than zero — trim is the load ABOVE the feed-forward.
    n = len(pts)
    sx = sum(p[0] for p in pts); sy = sum(p[1] for p in pts)
    sxx = sum(p[0] ** 2 for p in pts); sxy = sum(p[0] * p[1] for p in pts)
    den = n * sxx - sx * sx
    if abs(den) < 1e-12:
        print("# all points at the same trim — vary the load more")
        return
    a = (n * sxy - sx * sy) / den
    b = (sy - a * sx) / n
    print(f"\nmeasured   dI_dc/dtrim = {a:.4f} A/unit   (intercept {b:+.4f} A = the no-load draw)")
    print(f"predicted  dI_dc/dtrim = {slope_pred:.4f} A/unit")
    print(f"ratio measured/predicted = {a / slope_pred:.3f}")
    kt = 9.5493 / opts.kv
    print(f"\n=> torque per command unit of trim: predicted {kt * opts.v * DUTY6_A / opts.r * 1e3:.2f} mN*m, "
          f"scaled by the measured ratio {kt * opts.v * DUTY6_A / opts.r * (a / slope_pred) * 1e3:.2f} mN*m")
    print("A ratio near 1 confirms `trim` is an absolute torque signal given only KV, V and R.")


def main():
    ap = argparse.ArgumentParser(description="hand-applied load: does trim predict the current?")
    ap.add_argument("index", type=int, nargs="?", default=1)
    ap.add_argument("--rpm", type=float, default=1200.0)
    ap.add_argument("--minutes", type=float, default=3.0)
    ap.add_argument("--period", type=float, default=0.2)
    ap.add_argument("--kv", type=float, default=350.0)
    ap.add_argument("--v", type=float, default=11.2)
    ap.add_argument("--r", type=float, default=0.7)
    ap.add_argument("--cross-up", type=float, default=DUTY6_CROSSUP_REF + 2)   # the bench runs 36
    ap.add_argument("--idle", type=float, default=0.0055, help="ESC+logic idle current to subtract")
    ap.add_argument("--rpm-ceiling", type=float, default=3000.0)
    ap.add_argument("--stall-rpm", type=float, default=400.0)
    ap.add_argument("--settle-timeout", type=float, default=45.0)
    ap.add_argument("--max-blind", type=int, default=5)
    ap.add_argument("--print-every", type=int, default=5)
    ap.add_argument("--levels", default=None,
                    help="comma-separated PSU currents to hold in turn, e.g. 0.15,0.30,0.45,0.60")
    ap.add_argument("--level-secs", type=float, default=15.0)
    ap.add_argument("--baseline-secs", type=float, default=12.0)
    ap.add_argument("--currents", default=None,
                    help="operator readings as HH:MM:SS=amps, comma separated")
    ap.add_argument("--analyze", default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--port", default=None)
    ap.add_argument("--dry-run", action="store_true")
    opts = ap.parse_args()

    pairs = []
    if opts.currents:
        for tok in opts.currents.split(","):
            c, _, i = tok.strip().partition("=")
            pairs.append((c.strip(), float(i)))

    if opts.analyze:
        with open(opts.analyze) as f:
            rows = list(csv.DictReader(f))
        analyze(rows, pairs, opts)
        return 0

    if opts.dry_run:
        s = predicted_i_dc_per_trim(opts.kv, opts.v, opts.r, opts.cross_up, opts.rpm)
        print(f"# DRY RUN — hold ESC {opts.index} at {opts.rpm:g} rpm for {opts.minutes:g} min, dob off")
        print(f"#   predicted I_dc slope {s:.4f} A per trim unit; trim_max 80 => up to {80 * s:.2f} A added")
        print("#   operator loads the BELL by hand (never the prop) and films the PSU current")
        print("#   aborts: overspeed, stall, no telemetry; always stops and disarms")
        return 0

    out = opts.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), "reports",
                                   f"loadtest_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    dev = EscHost(find_pico(opts.port))
    state, status = {"rows": []}, "completed"
    try:
        dev.cmd("run", timeout=6); dev.cmd("disconnect", timeout=6); time.sleep(0.4)
        run(dev, opts, state)
    except (Aborted, KeyboardInterrupt) as e:
        status = f"stopped: {e}"
        print(f"\n!! {status}")
    finally:
        for step in (f"rpm {opts.index} 0", f"disarm {opts.index}", f"gain {opts.index} dob 0.8"):
            try:
                dev.cmd(step, timeout=6)
            except Exception as e:
                print(f"# WARNING: '{step}' failed ({e})")
            time.sleep(0.5)
        print("# stopped, disarmed, dob restored to 0.8")

    rows = state["rows"]
    if rows:
        with open(out, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader(); w.writerows(rows)
        print(f"# {len(rows)} samples -> {out}")
        print(f"# pair the video readings with:\n"
              f"#   python3 host/loadtest.py --analyze {out} --currents HH:MM:SS=A,HH:MM:SS=A,...")
    print(f"# status: {status}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
