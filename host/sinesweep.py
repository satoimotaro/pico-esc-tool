#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""sinesweep — the forced-sine regime measured against the encoder, where telemetry is a lie.

WHY THIS EXISTS. Below the crossover seam the ESC is not sensing anything: it FORCES commutation at
a rate set purely by the command, and it reports that commanded rate back as eRPM. `tele` is
therefore not a noisy measurement of the rotor, it is an open-loop *intention* — on this bench it
sits pinned at a virtual 182 rpm while the shaft does something else entirely. Every characterisation
so far has been done in 6-step, where tele is honest, so this whole regime has never been checked
against truth. The AS5600 is bench-only, so it can only be checked now.

WHAT IT MEASURES. Forced commutation has a known law — the field advances at

    rpm_field = cmd/1000 * 2499 / pole_pairs        (357 rpm full scale at PP=7)

and a synchronous rotor must follow it exactly. So the interesting quantity is SLIP:

    slip = 1 - |rpm_encoder| / rpm_field

which is 0 for a rotor locked to the field, grows as the rotor falls behind, and goes wild once it
stops following at all. Unlike a 6-step lock check (`duty_sweep`'s tele-vs-encoder agreement) this
needs no telemetry, which is the point: there is no honest telemetry down here.

It also records what the rotor does BESIDES rotating at the mean rate. At low command the shaft on
this bench cogs, stalls and reverses — a mean alone reports "49 rpm" for a rotor that is oscillating
about zero — so each point keeps min/max/spread and the sign, not just an average.

BOTH DIRECTIONS, on purpose. The regime has hysteresis: sweeping up crosses into 6-step at the seam,
sweeping down falls back out of it at a different command. `--updown` captures both legs so the fold
is measured rather than assumed.

Open loop only. `thrust` (a fixed COMMAND) is the swept variable; the closed loop would move cmd to
hold speed and destroy the measurement. Raw mode needs a keep-alive (500 ms deadman) and does not
update `sense`.

  python3 host/sinesweep.py 1 --updown
  python3 host/sinesweep.py --analyze host/reports/sinesweep_*.csv
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from esctool import EscHost, find_pico            # noqa: E402

# Bluejay's forced-sine full scale: the field advances 2499 eRPM at full command (SINE_RCP_SHIFT=3,
# constants.FULLSCALE_RPM=357 mech at 7 pole pairs). Mechanical rate = eRPM / pole_pairs.
SINE_FULLSCALE_ERPM = 2499.0


def field_rpm(cmd, pole_pairs):
    """Mechanical rate the ESC is FORCING the field to turn at, for a raw command."""
    return abs(cmd) / 1000.0 * SINE_FULLSCALE_ERPM / pole_pairs


def _encv(dev):
    try:
        for ln in dev.cmd("encv", timeout=2):
            if ln.startswith("encv|"):
                return float(ln.split("|")[2])
    except Exception:
        pass
    return None


def _tele(dev, idx):
    """tele|<mech rpm>|... -> rpm, or None when the ESC is not sending telemetry at all."""
    try:
        for ln in dev.cmd(f"tele {idx}", timeout=2):
            if ln.startswith("tele|"):
                return float(ln.split("|")[1])
    except Exception:
        pass
    return None


def hold(dev, idx, cmd, opts):
    """Hold one command and sample the encoder; returns the settled-window statistics."""
    t_end = time.time() + opts.dwell
    samples, teles = [], []
    while time.time() < t_end:
        dev.cmd(f"thrust {idx} {cmd:d}", timeout=4)      # keep-alive: 500 ms deadman
        time.sleep(opts.read_period)
        r = _encv(dev)
        if r is not None:
            samples.append((time.time(), r))
        t = _tele(dev, idx)
        if t is not None:
            teles.append(t)
    if not samples:
        return None
    # Settled window only: the leading fraction is the rotor still responding to the step.
    t0 = samples[0][0]
    span = samples[-1][0] - t0
    settled = [r for t, r in samples if t - t0 >= span * opts.settle_frac] or [r for _, r in samples]
    n = len(settled)
    mean = sum(settled) / n
    return {
        "n": n,
        "mean": mean,
        "min": min(settled),
        "max": max(settled),
        "spread": max(settled) - min(settled),
        "reversals": sum(1 for a, b in zip(settled, settled[1:]) if (a > 0) != (b > 0)),
        "tele": (sum(teles) / len(teles)) if teles else float("nan"),
    }


def sweep(dev, idx, cmds, opts, leg, rows):
    for cmd in cmds:
        st = hold(dev, idx, cmd, opts)
        if st is None:
            print(f"  {leg} cmd {cmd}: no encoder samples — skipping")
            continue
        f = field_rpm(cmd, opts.pole_pairs)
        slip = (1.0 - abs(st["mean"]) / f) if f > 0 else float("nan")
        rows.append((leg, cmd, f, st["mean"], st["min"], st["max"], st["spread"],
                     st["reversals"], st["tele"], slip, st["n"]))
        print(f"  {leg} cmd {cmd:>4}: field {f:>6.1f}  enc {st['mean']:>8.1f} "
              f"[{st['min']:>7.1f}..{st['max']:>7.1f}] spread {st['spread']:>6.1f} "
              f"rev {st['reversals']:>2}  tele {st['tele']:>7.1f}  slip {slip:>7.2%}")
        if opts.abort_rpm and abs(st["mean"]) > opts.abort_rpm:
            print(f"  !! {abs(st['mean']):.0f} rpm exceeds --abort-rpm — stopping this leg")
            return


def in_six_step(row, opts):
    """Has the ESC handed off to 6-step at this point?

    The rotor outrunning the forced field is the signature — in 6-step the field rate no longer sets
    the speed at all. But the ratio test ALONE is not enough: down at the bottom of the regime the
    rotor does not rotate, it oscillates about zero and reverses several times per dwell, so a mean
    of 39 rpm against a 21 rpm field trips "1.5x the field" while the ESC is still firmly in sine.
    The handoff physically cannot fire below cross_up in eRPM (~190 mech at cross_up=34-36), so an
    absolute speed floor is the honest second condition.
    """
    field, enc = row[2], abs(row[3])
    return field > 0 and enc > field * opts.seam_factor and enc >= opts.seam_min_rpm


def transition(sub, opts):
    """(last_row_before, first_row_after) across the sine<->6-step boundary, or None.

    Direction-agnostic on purpose. A descending leg STARTS in 6-step, so "the first point that looks
    like 6-step" is its own first sample and reports a handoff at the top of the sweep — which is
    both useless and wrong-way-round. What matters on either leg is the bracket the classification
    flips in; comparing that bracket between the up and down legs is how the fold's hysteresis gets
    measured instead of assumed.
    """
    for a, b in zip(sub, sub[1:]):
        if in_six_step(a, opts) != in_six_step(b, opts):
            return a, b
    return None


def report(rows, opts):
    if not rows:
        print("  (no rows)")
        return
    print("\n=== forced-sine following, per leg ===")
    brackets = {}
    for leg in sorted({r[0] for r in rows}):
        sub = [r for r in rows if r[0] == leg]
        print(f"\n  {leg}:")
        tr = transition(sub, opts)
        if tr:
            a, b = tr
            lo, hi = sorted((a[1], b[1]))
            brackets[leg] = (lo, hi)
            print(f"    sine <-> 6-step boundary between cmd {lo} and {hi} "
                  f"({abs(a[3]):.0f} rpm -> {abs(b[3]):.0f} rpm, "
                  f"while the forced field only moves {a[2]:.0f} -> {b[2]:.0f})")
        else:
            state = "6-step" if sub and in_six_step(sub[0], opts) else "forced sine"
            print(f"    no boundary crossed — the whole leg stayed in {state}")
        pre = [r for r in sub if not in_six_step(r, opts)]
        following = [r for r in pre if abs(r[9]) <= opts.follow_slip]
        if following:
            lo, hi = min(r[1] for r in following), max(r[1] for r in following)
            print(f"    follows the field (|slip| <= {opts.follow_slip:.0%}) over cmd {lo}..{hi} "
                  f"= {field_rpm(lo, opts.pole_pairs):.0f}..{field_rpm(hi, opts.pole_pairs):.0f} rpm")
        else:
            print(f"    NEVER follows the field within {opts.follow_slip:.0%} slip")
        bad = [r for r in pre if r[7] > 0]
        if bad:
            print(f"    REVERSES direction while held at cmd {', '.join(str(r[1]) for r in bad)} "
                  f"— a mean speed is meaningless there")
        if pre:
            worst = max(pre, key=lambda r: r[6])
            print(f"    worst spread {worst[6]:.0f} rpm at cmd {worst[1]} "
                  f"(mean {worst[3]:.0f}) — this is the cogging floor, not measurement noise")
            liars = [r for r in pre if r[2] > 0 and abs(r[8] - r[2]) > r[2] * 0.25]
            if liars:
                print(f"    tele disagrees with the forced field by >25% at {len(liars)}/{len(pre)} "
                      f"points; e.g. cmd {liars[0][1]}: tele says {liars[0][8]:.0f}, "
                      f"field is {liars[0][2]:.0f}, rotor is {liars[0][3]:.0f}")

    if "up" in brackets and "down" in brackets:
        u, d = brackets["up"], brackets["down"]
        print("\n  hysteresis of the fold:")
        if u == d:
            print(f"    NONE resolved — both legs cross in the same cmd {u[0]}..{u[1]} bracket. "
                  f"Re-run with a finer --cmds through that bracket to resolve it, or take it as "
                  f"evidence the up and down handoffs really do fire at the same command here.")
        else:
            print(f"    up crosses at cmd {u[0]}..{u[1]}, down at cmd {d[0]}..{d[1]} "
                  f"— the fold is {abs(u[0] - d[0])} command units wide")


def main():
    ap = argparse.ArgumentParser(description="forced-sine regime vs encoder truth")
    ap.add_argument("index", type=int, nargs="?", default=1)
    ap.add_argument("--cmds", default=None,
                    help="comma-separated raw commands (default 60..600 step 30, through the seam)")
    ap.add_argument("--updown", action="store_true",
                    help="sweep up then back down — the fold is hysteretic, so both legs matter")
    ap.add_argument("--dwell", type=float, default=3.0, help="s to hold each command")
    ap.add_argument("--read-period", type=float, default=0.12,
                    help="s between samples; also the thrust keep-alive period (deadman is 500 ms)")
    ap.add_argument("--settle-frac", type=float, default=0.4,
                    help="discard this leading fraction of each dwell as transient")
    ap.add_argument("--pole-pairs", type=int, default=7)
    ap.add_argument("--follow-slip", type=float, default=0.10,
                    help="|slip| at or below this counts as following the forced field")
    ap.add_argument("--seam-factor", type=float, default=1.5,
                    help="encoder rpm above this x the field rate means it has handed off to 6-step")
    ap.add_argument("--seam-min-rpm", type=float, default=250.0,
                    help="...and it must also be turning at least this fast; the handoff cannot fire "
                         "below cross_up in eRPM (~190 mech), and a rotor merely oscillating about "
                         "zero at the bottom of the regime otherwise trips the ratio test")
    ap.add_argument("--abort-rpm", type=float, default=3200.0,
                    help="stop a leg if the rotor runs away past this (0 disables)")
    ap.add_argument("--analyze", default=None, help="re-report an existing CSV; no hardware")
    ap.add_argument("--out", default=None)
    ap.add_argument("--port", default=None)
    ap.add_argument("--dry-run", action="store_true")
    opts = ap.parse_args()

    cmds = ([int(c) for c in opts.cmds.split(",")] if opts.cmds
            else list(range(60, 601, 30)))

    if opts.analyze:
        rows = []
        with open(opts.analyze) as f:
            for r in csv.DictReader(f):
                rows.append((r["leg"], int(r["cmd"]), float(r["field_rpm"]), float(r["enc_mean"]),
                             float(r["enc_min"]), float(r["enc_max"]), float(r["enc_spread"]),
                             int(r["reversals"]), float(r["tele_rpm"]), float(r["slip"]),
                             int(r["n"])))
        print(f"# re-report {len(rows)} points from {opts.analyze}")
        report(rows, opts)
        return 0

    if opts.dry_run:
        legs = "up then down" if opts.updown else "up"
        print(f"# DRY RUN — ESC {opts.index}, open-loop thrust {cmds[0]}..{cmds[-1]} ({legs}), "
              f"{opts.dwell:g}s each, encoder truth vs the forced field")
        print(f"#   field at cmd {cmds[0]} = {field_rpm(cmds[0], opts.pole_pairs):.1f} rpm, "
              f"at cmd {cmds[-1]} = {field_rpm(cmds[-1], opts.pole_pairs):.1f} rpm")
        print("#   raw thrust does NOT update `sense`; tele is a forced-commutation intention here.")
        return 0

    out = opts.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), "reports",
                                   f"sinesweep_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    dev = EscHost(find_pico(opts.port))
    rows = []
    try:
        dev.cmd("run", timeout=6)
        time.sleep(0.4)
        dev.cmd(f"arm {opts.index} bidir", timeout=8)
        time.sleep(3.0)
        print(f"=== up: cmd {cmds[0]}..{cmds[-1]} ===")
        sweep(dev, opts.index, cmds, opts, "up", rows)
        if opts.updown:
            print(f"\n=== down: cmd {cmds[-1]}..{cmds[0]} ===")
            sweep(dev, opts.index, list(reversed(cmds)), opts, "down", rows)
    except KeyboardInterrupt:
        print("\n!! interrupted")
    finally:
        for step in (f"thrust {opts.index} 0", f"disarm {opts.index}"):
            try:
                dev.cmd(step, timeout=6)
            except Exception as e:
                print(f"# WARNING: '{step}' failed ({e})")
            time.sleep(0.4)
        print("# stopped, disarmed")

    if rows:
        with open(out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["leg", "cmd", "field_rpm", "enc_mean", "enc_min", "enc_max", "enc_spread",
                        "reversals", "tele_rpm", "slip", "n"])
            w.writerows(rows)
        print(f"# {len(rows)} points -> {out}")
        report(rows, opts)
    return 0


if __name__ == "__main__":
    sys.exit(main())
