#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""driftsoak — hold ONE fixed rpm setpoint for a long time and log the control effort.

The point: a closed velocity loop is blind to plant drift by design. As the winding heats
(R +0.393 %/degC) and the magnets weaken (KV +0.11 %/degC), the loop just works harder and `rpm`
keeps reporting the target. The drift lives in `trim` / `cmd`, not in the tracking error.

So: fix the setpoint, change nothing else, and watch `trim` walk. Everything else in the log
(`vest`, `dhat`, `tempC`, encoder rpm) is a covariate for attributing that walk.

DOB is turned OFF by default (`--dob 0`) so the PI trim is the SOLE correction channel. With the
shipped dob=0.8 the drift would SPLIT between `trim` and `dhat` and neither alone is the answer.

Units gotcha: `trim`/`cmd` are ESC COMMAND units, not rpm. Divide by the curve slope
(~10.3 rpm/unit in the 350KV 6-step segment) to read them as rpm.

Caveat that limits interpretation: `tele.tempC` is the *ESC* MCU temperature, not the winding.
It is a lagging proxy at best. The winding temperature is not observable on this hardware.

Safety: single fixed setpoint, host-side temp / overspeed / stall / duration aborts, and it ALWAYS
ramps down and disarms on every exit path. Restores the gains it changed.

  python3 host/driftsoak.py 1 --rpm 1600 --minutes 30
  python3 host/driftsoak.py 1 --dry-run
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from esctool import EscHost, find_pico            # noqa: E402


class Aborted(RuntimeError):
    pass


def _fields(dev, cmd, prefix):
    """Send `cmd`, return the key=value dict of the first line starting with `prefix`."""
    try:
        lines = dev.cmd(cmd, timeout=4)
    except Exception:
        return None
    for ln in lines:
        if ln.startswith(prefix):
            return {kv.split("=")[0]: kv.split("=")[1]
                    for kv in ln.split("|")[1:] if "=" in kv}
    return None


def _sense(dev, idx):
    return _fields(dev, f"sense {idx}", "sense|")


def _tele(dev, idx):
    """tele|<rpm>|<volts>|<amps>|<tempC>|<stress> — positional, not key=value."""
    try:
        lines = dev.cmd(f"tele {idx}", timeout=4)
    except Exception:
        return None                      # 'err no-telem' until the ESC is live
    for ln in lines:
        if ln.startswith("tele|"):
            p = ln.split("|")
            try:
                return {"rpm": float(p[1]), "volts": float(p[2]),
                        "amps": float(p[3]), "tempC": float(p[4]), "stress": float(p[5])}
            except (IndexError, ValueError):
                return None
    return None


def _encv(dev):
    """encv|<accum>|<rpm>|<samples>|<md> — de-aliased mechanical rpm from the AS5600."""
    try:
        lines = dev.cmd("encv", timeout=4)
    except Exception:
        return None
    for ln in lines:
        if ln.startswith("encv|"):
            try:
                return float(ln.split("|")[2])
            except (IndexError, ValueError):
                return None
    return None


def _cfg(dev, idx):
    for ln in dev.cmd("cfg show", timeout=6):
        if ln.startswith(f"cfg|{idx}|"):
            return {kv.split("=")[0]: kv.split("=")[1]
                    for kv in ln.split("|")[2:] if "=" in kv}
    return {}


def sample(dev, idx):
    s = _sense(dev, idx) or {}
    t = _tele(dev, idx) or {}
    return {
        "trim":    float(s.get("trim", "nan")),
        "cmd":     float(s.get("cmd", "nan")),
        "dhat":    float(s.get("dhat", "nan")),
        "settled": float(s.get("settled", "nan")),
        "vest":    float(s.get("vest", "nan")),
        "vbatt":   float(s.get("vbatt", "nan")),
        "load":    float(s.get("load", "nan")),
        "valid":   int(float(s.get("valid", 0))),
        # Soft-sensor validity window (firmware vel::SenseGate). `why` is the whole point of logging
        # these: the gate is deliberately stricter than the old rpm-only test, so a valid=0 row has to
        # say WHICH condition closed it instead of looking like a regression. loadok/vbok are the two
        # sub-tiers — `load` is only comparable near the command `sense zero` was taken at, and `vbatt`
        # only advances at light load. `.get` defaults keep an OLD-format firmware parsing fine.
        "why":     s.get("why", ""),
        "loadok":  int(float(s.get("loadok", 0))),
        "vbok":    int(float(s.get("vbok", 0))),
        # Firmware thermal limiter: tlim = the command-ceiling scale it is applying (1.00 = none),
        # tstate = none|ok|derate|stale. A derate means the firmware is withholding command, so `trim`
        # and `cmd` stop being a pure drift signal — the soak has to be read differently from there on.
        "tlim":    float(s.get("tlim", "nan")),
        "tstate":  s.get("tstate", ""),
        "tele_rpm": t.get("rpm", float("nan")),
        "tempC":    t.get("tempC", float("nan")),
        "stress":   t.get("stress", float("nan")),
        "enc_rpm":  _encv(dev),
    }


def run(dev, opts, state):
    """`state` is the caller's dict; fill it AS WE GO. An abort is exactly when the partial data and
    the pre-run gains matter most, so nothing may depend on this function returning normally."""
    idx = opts.index
    before = _cfg(dev, idx)
    state["before"] = before
    print(f"# cfg before: {before}")
    slope = opts.slope

    # DOB off (default) so the PI trim is the only correction channel. NOT persisted: no `cfg save`,
    # and the original value is restored in the caller's finally.
    dev.cmd(f"gain {idx} dob {opts.dob}", timeout=4)
    print(f"# dob -> {opts.dob} (live only; stored config untouched)")

    dev.cmd(f"arm {idx} bidir", timeout=8)
    time.sleep(3.0)
    dev.cmd(f"rpm {idx} {opts.rpm:g}", timeout=6)
    print(f"# rpm -> {opts.rpm:g}, waiting for a settled 6-step lock "
          f"(<= {opts.settle_timeout:g}s)")

    # --- wait for a solid 6-step lock before t0, so the soak window is drift only, not startup ---
    t_wait = time.time()
    while True:
        s = sample(dev, idx)
        if s["valid"] and s["settled"] >= 1.0 and s["tele_rpm"] > opts.stall_rpm:
            break
        if time.time() - t_wait > opts.settle_timeout:
            # `why` names the condition that blocked the lock (cmdlow = the setpoint never asked for a
            # command above the soft-sensor's floor; rpmlow = it did and the rotor never caught; ...).
            # Without it "valid=0" is the same message for every possible cause.
            raise Aborted(f"never reached a settled 6-step lock in {opts.settle_timeout:g}s "
                          f"(valid={s['valid']} why={s['why'] or '?'} settled={s['settled']} "
                          f"rpm={s['tele_rpm']})")
        time.sleep(0.5)
    print(f"# locked after {time.time() - t_wait:.1f}s")

    # --- settle out the PI's own convergence transient BEFORE the baseline ---
    # Measured on a 2 min trial: trim lands at ~11.6 right after the lock and decays to ~0 within a
    # minute. Baselining on sample 0 would book that startup transient as thermal drift.
    if opts.baseline_skip > 0:
        print(f"# settling {opts.baseline_skip:g}s before the baseline (PI convergence, not drift)")
        time.sleep(opts.baseline_skip)

    # Baseline the soft-sensor reference here too, so `load` (= vref - vest) becomes a SECOND drift
    # channel alongside trim instead of a constant 0.
    try:
        dev.cmd(f"sense {idx} zero", timeout=4)
        print("# sense zero: vref baselined -> `load` now tracks the vest deficit")
    except Exception as e:
        print(f"# note: sense zero failed ({e}); `load` stays 0")

    rows, t0, stall_run, blind_run = state["rows"], time.time(), 0, 0
    base, last_tstate = None, ""
    deadline = t0 + opts.minutes * 60.0
    next_tick = t0

    while time.time() < deadline:
        next_tick += opts.period
        s = sample(dev, idx)
        s["t"] = time.time() - t0
        rows.append(s)
        if base is None:
            base = s

        # --- guards ---
        # A MISSING sample must never read as a healthy one. `_tele` returns None on any serial error
        # or `err no-telem`, which lands here as NaN — and every `x >= limit` test on a NaN is False, so
        # a dropout silently switched off the temp, overspeed and stall aborts for the rest of an
        # unattended run. Losing telemetry is also the desync signature the stall guard exists to catch,
        # so it is treated as its own abort condition rather than as an absence of evidence.
        # The firmware's 500 ms deadman does NOT back this up: in RPM submode Thruster::poll() re-sends
        # a command every tick, so the host going quiet leaves the motor running at setpoint.
        have_tele = s["tele_rpm"] == s["tele_rpm"]
        if not have_tele or s["tempC"] != s["tempC"]:
            blind_run += 1
            if blind_run >= opts.max_blind:
                raise Aborted(f"no telemetry for {blind_run} consecutive samples — guards are blind "
                              f"(link dropout or desync); stopping rather than running on unguarded")
        else:
            blind_run = 0

        # The FIRMWARE thermal limiter changes what this log MEANS: under a derate `trim`/`cmd` are
        # bounded by a ceiling instead of tracking the plant, and `stale` means the firmware has no
        # thermal protection at all any more. Report every transition; the host --max-temp abort below
        # is unchanged and still fires first at its own threshold.
        if s["tstate"] and s["tstate"] != last_tstate:
            if s["tstate"] in ("derate", "stale"):
                print(f"  !! firmware thermal {s['tstate']} at t={s['t']:.0f}s "
                      f"(tlim={s['tlim']:.2f}, ESC {s['tempC']}C) — `trim`/`cmd` are no longer a "
                      f"pure drift signal")
            last_tstate = s["tstate"]

        if s["tempC"] == s["tempC"] and s["tempC"] >= opts.max_temp:
            raise Aborted(f"ESC over-temperature {s['tempC']:.0f}C >= {opts.max_temp:.0f}C")
        for label, r in (("tele", s["tele_rpm"]), ("enc", s["enc_rpm"])):
            if r is not None and r == r and abs(r) > opts.rpm_ceiling:
                raise Aborted(f"overspeed on {label}: {r:.0f} > {opts.rpm_ceiling:.0f} rpm")
        # Only a sample that is PRESENT and above the floor clears the stall counter.
        if have_tele and abs(s["tele_rpm"]) < opts.stall_rpm:
            stall_run += 1
            if stall_run >= 3:
                raise Aborted(f"stall/desync: tele rpm {s['tele_rpm']:.0f} < {opts.stall_rpm:.0f} "
                              f"for {stall_run} samples")
        elif have_tele:
            stall_run = 0

        if len(rows) % opts.print_every == 0:
            d = s["trim"] - base["trim"]
            print(f"  t={s['t']:6.0f}s  rpm={s['tele_rpm']:7.1f}  trim={s['trim']:6.1f} "
                  f"(d{d:+5.1f} = {d * slope:+6.1f} rpm)  cmd={s['cmd']:5.0f}  "
                  f"dhat={s['dhat']:6.1f}  vest={s['vest']:5.2f}  temp={s['tempC']:.0f}C")

        time.sleep(max(0.0, next_tick - time.time()))


def summarize(rows, slope, window=30):
    """Compare the first and last `window` samples — averaging beats endpoint-vs-endpoint."""
    def avg(sub, k):
        v = [r[k] for r in sub if r[k] is not None and r[k] == r[k]]
        # the AS5600 reads the opposite sign to the commanded direction on this bench; compare
        # magnitudes so tele and enc deltas are directly readable against each other.
        if k in ("tele_rpm", "enc_rpm"):
            v = [abs(x) for x in v]
        return sum(v) / len(v) if v else float("nan")

    head, tail = rows[:window], rows[-window:]
    out = {}
    for k in ("trim", "cmd", "tele_rpm", "enc_rpm", "dhat", "vest", "vbatt", "tempC", "load"):
        out[k] = (avg(head, k), avg(tail, k))
    print("\n=== drift over the soak (mean of first vs last "
          f"{window} samples, {rows[-1]['t']:.0f}s apart) ===")
    print(f"{'field':>9}  {'start':>9}  {'end':>9}  {'delta':>9}")
    for k, (a, b) in out.items():
        print(f"{k:>9}  {a:9.2f}  {b:9.2f}  {b - a:+9.2f}")
    dtrim = out["trim"][1] - out["trim"][0]
    dtemp = out["tempC"][1] - out["tempC"][0]
    print(f"\ntrim drift  {dtrim:+.2f} command units = {dtrim * slope:+.1f} rpm of correction")
    print(f"ESC temp    {dtemp:+.1f} C (MCU, NOT the winding — lagging proxy)")
    if dtemp > 0.5:
        print(f"ratio       {dtrim * slope / dtemp:+.2f} rpm of correction per degC of ESC rise")
    print(f"rpm error   {out['tele_rpm'][0] - out['tele_rpm'][1]:+.2f} rpm start->end "
          f"(should stay ~0: the loop hides the drift — that is the point)")
    return out


def main():
    ap = argparse.ArgumentParser(description="hold one rpm and log the control-effort drift")
    ap.add_argument("index", type=int, nargs="?", default=1, help="ESC index (default 1)")
    ap.add_argument("--rpm", type=float, default=1600.0,
                    help="fixed setpoint, mech rpm (default 1600 = the reference point all the "
                         "existing dhat/dob_bench numbers were taken at)")
    ap.add_argument("--minutes", type=float, default=30.0, help="soak duration (default 30)")
    ap.add_argument("--period", type=float, default=1.0, help="sample period, s (default 1)")
    ap.add_argument("--dob", type=float, default=0.0,
                    help="DOB gain during the soak (default 0 = OFF, so trim is the only channel)")
    ap.add_argument("--slope", type=float, default=10.3,
                    help="curve slope, rpm per command unit, for reporting trim as rpm (default 10.3)")
    ap.add_argument("--max-temp", type=float, default=80.0, help="ESC temp abort, C (default 80)")
    ap.add_argument("--rpm-ceiling", type=float, default=3000.0, help="overspeed abort (default 3000)")
    ap.add_argument("--stall-rpm", type=float, default=300.0, help="stall abort floor (default 300)")
    ap.add_argument("--max-blind", type=int, default=5,
                    help="abort after this many consecutive samples with no telemetry — the guards "
                         "cannot see anything without it (default 5)")
    ap.add_argument("--settle-timeout", type=float, default=60.0, help="s to reach a locked 6-step")
    ap.add_argument("--baseline-skip", type=float, default=120.0,
                    help="s to run before the baseline sample, so the PI's own convergence "
                         "transient is not booked as drift (default 120)")
    ap.add_argument("--print-every", type=int, default=30, help="print every Nth sample (default 30)")
    ap.add_argument("--out", default=None, help="CSV path (default host/reports/driftsoak_<ts>.csv)")
    ap.add_argument("--port", default=None)
    ap.add_argument("--dry-run", action="store_true", help="print the plan, touch no hardware")
    opts = ap.parse_args()

    if opts.dry_run:
        print(f"# DRY RUN — would hold ESC {opts.index} at {opts.rpm:g} rpm for {opts.minutes:g} min")
        print(f"#   dob={opts.dob} (0 = trim is the sole correction channel), sample {opts.period:g}s")
        print(f"#   aborts: temp>={opts.max_temp:g}C, |rpm|>{opts.rpm_ceiling:g}, "
              f"|rpm|<{opts.stall_rpm:g} x3, {opts.minutes:g} min")
        print("#   always: rpm 0 -> disarm -> restore dob on every exit")
        return 0

    out = opts.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), "reports",
                                   f"driftsoak_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    os.makedirs(os.path.dirname(out), exist_ok=True)

    port = find_pico(opts.port)
    print(f"# port {port}")
    dev = EscHost(port)
    state, status = {"rows": [], "before": {}}, "completed"
    try:
        dev.cmd("run", timeout=6)
        dev.cmd("disconnect", timeout=6)
        time.sleep(0.4)
        run(dev, opts, state)
    except Aborted as e:
        status = f"ABORTED: {e}"
        print(f"\n!! {status}")
    except KeyboardInterrupt:
        status = "interrupted by user"
        print(f"\n!! {status}")
    finally:
        rows, before = state["rows"], state["before"]
        # Each step guarded INDEPENDENTLY: one shared try meant a benign first failure (e.g. `rpm`
        # returning err when already disarmed) skipped BOTH the disarm and the gain restore, leaving
        # the bench in a modified state. Restoring gains is not optional book-keeping.
        steps = [f"rpm {opts.index} 0", f"disarm {opts.index}"]
        if before.get("dob") is not None:
            steps.append(f"gain {opts.index} dob {before['dob']}")
        for step in steps:
            try:
                dev.cmd(step, timeout=6)
            except Exception as e:
                print(f"# WARNING: '{step}' failed ({e})")
            time.sleep(0.5)
        if before.get("dob") is not None:
            print(f"# stopped, disarmed, dob restored to {before['dob']}")
        else:
            print(f"# stopped, disarmed — WARNING: pre-run dob unknown, it is still {opts.dob:g}. "
                  f"Set it back by hand: gain {opts.index} dob <v>")

    if rows:
        keys = ["t", "trim", "cmd", "dhat", "settled", "vest", "vbatt", "load", "valid",
                "why", "loadok", "vbok", "tlim", "tstate",
                "tele_rpm", "enc_rpm", "tempC", "stress"]
        with open(out, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            w.writeheader()
            w.writerows(rows)
        print(f"# {len(rows)} samples -> {out}")
        if len(rows) >= 10:
            summarize(rows, opts.slope, window=min(30, len(rows) // 3))
    print(f"# status: {status}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
