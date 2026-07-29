#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""curvepush — upload a MEASURED feed-forward curve (a profile YAML) into the Pico firmware.

Until now the only curve the firmware could take at runtime was the 3-scalar PARAMETRIC prediction
(`motor <i> <kv> <pp> <v>`); a real velcal/sysid curve reached it only via
`gen_profile_header.py -> recompile -> reflash`. That gap is what holds the disturbance observer back:
the DOB inverts the forward map, so with a curve that over-predicts it reads model error as load and
has to be clamped. Push the measured curve and the deficit becomes genuine load.

    python3 host/curvepush.py 1 host/profiles/f2838_350kv_bench_20260726.yaml --gains --save
    python3 host/curvepush.py 1 <profile.yaml> --dry-run        # no hardware; validates the transfer

The wire protocol is one point per line (`curve <i> add <thrust> <rpm> [s|l]`) because the firmware's
input buffer is 600 B and truncates silently past it. Points are staged and committed atomically, then
read back and compared — a partial or mangled upload never becomes the live curve.

`--save` also runs `cfg save`, without which the curve is lost on the next reset.
"""
from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pico_esc.esc import EscConfig                    # noqa: E402
from pico_esc.link import EscHost, SimClock          # noqa: E402
from pico_esc.sim import SimEncEscHost                # noqa: E402
from pico_esc.velocity import SpeedProfile            # noqa: E402

# The firmware's owned FF buffer (Thruster::ffbuf_ / settings::CFG_MAX_CURVE).
FW_MAX_POINTS = 24


def open_host(opts):
    """Mirror the other tools: --dry-run swaps in the same sim oracle, no hardware needed."""
    if opts.dry_run:
        print("# DRY-RUN: SimEncEscHost (no serial port opened)")
        return SimEncEscHost(SimClock())
    return EscHost(opts.port)


def thin(points, regimes, cap):
    """Reduce to <= cap points, ALWAYS keeping the endpoints and both sides of the seam (the largest
    rpm jump). Straight decimation could drop either: the seam is the one place the curve is not
    smooth, and the last point defines the top of the feed-forward range. So the mandatory points are
    reserved FIRST and the remaining budget is filled by even decimation — never the other way round,
    which would let the fill crowd the mandatory ones out of the cap."""
    n = len(points)
    if n <= cap:
        return points, regimes
    if cap < 2:
        raise ValueError("cap must be >= 2")
    jumps = sorted(range(1, n), key=lambda i: points[i][1] - points[i - 1][1], reverse=True)
    must = {0, n - 1, jumps[0] - 1, jumps[0]}                    # ends + both sides of the seam
    if len(must) >= cap:
        idx = sorted(must)[:cap]
    else:
        rest = [i for i in range(n) if i not in must]
        slots = cap - len(must)
        step = (len(rest) - 1) / float(slots - 1) if slots > 1 else 0.0
        fill = {rest[min(len(rest) - 1, int(round(k * step)))] for k in range(slots)}
        idx = sorted(must | fill)
    return [points[i] for i in idx], ([regimes[i] for i in idx] if regimes else None)


def check_crossover(dev, idx, prof, apply_it):
    """A measured FF curve is only valid at the `sine_cross_up` it was measured at.

    The BlueGill firmware's `cross_rescale_duty` maps the 6-step demand to an affine function of the
    throttle RELATIVE to Cross_Up, so moving that threshold shifts the whole command->speed mapping.
    Bench-measured on the 350KV: the same profile is 0.5% accurate at its own cross_up=36 and 27-46%
    over-speed at cross_up=34, while the parametric model is the exact mirror image. Pushing a curve
    without matching the threshold therefore installs a silently-invalid curve, which is worse than
    leaving the parametric one in place — so this REFUSES by default rather than warning.

    Returns True when it is safe to proceed.
    """
    want = (prof.crossover or {}).get("bytes")
    if not want:
        print("note: profile carries no crossover bytes — cannot verify the ESC threshold the curve "
              "was measured at; the curve is only valid at that threshold")
        return True
    cfg = EscConfig(dev, idx)
    cur = cfg.read()
    cfg.restart()                                   # never leave the ESC held in the bootloader
    if not cur:
        print("note: could not read the ESC config (dry-run / no bootloader answer) — crossover unverified")
        return True
    s = cur.get("settings", cur)
    cu, cd = int(want[0]), int(want[1])
    have_up, have_dn = s.get("sine_cross_up"), s.get("sine_cross_dn")
    if (have_up, have_dn) == (cu, cd):
        print(f"crossover: ESC matches the profile (sine_cross_up={cu} sine_cross_dn={cd})")
        return True
    print(f"CROSSOVER MISMATCH: ESC has sine_cross_up={have_up} sine_cross_dn={have_dn}, "
          f"the curve was measured at {cu}/{cd}.")
    if not apply_it:
        print("  The firmware rescales 6-step duty relative to Cross_Up, so this curve would be wrong "
              "by tens of percent.\n  Re-run with --apply-crossover, or push a profile measured at the "
              "ESC's current threshold.")
        return False
    cfg.set(sine_cross_up=cu, sine_cross_dn=cd)
    cfg.restart()
    print(f"  applied sine_cross_up={cu} sine_cross_dn={cd} and restarted the ESC")
    return True


def push(dev, idx, prof, cap=FW_MAX_POINTS):
    """Stage + commit the profile's curve on ESC `idx`. Returns the points actually sent."""
    pts = [(float(t), float(r)) for t, r in prof.points]
    regs = list(prof.regimes) if prof.regimes else None
    pts, regs = thin(pts, regs, cap)

    # Round to the wire resolution (one decimal) BEFORE sending, so the returned list is exactly what
    # the device stores and the read-back comparison is meaningful rather than off by the rounding.
    # Rounding can collide two thrusts into one, which the firmware would reject as non-monotone, so
    # drop any point whose rounded thrust repeats.
    rounded, kept = [], []
    for i, (t, r) in enumerate(pts):
        rt, rr = round(t, 1), round(r, 1)
        if rounded and rt <= rounded[-1][0]:
            continue
        rounded.append((rt, rr))
        kept.append(i)
    if len(rounded) < len(pts):
        print(f"note: {len(pts) - len(rounded)} point(s) dropped — thrust collided at 0.1 resolution")
    pts = rounded
    regs = [regs[i] for i in kept] if regs else None

    cx = prof.crossover or {}
    up = float(cx.get("up_erpm", 0.0))
    dn = float(cx.get("dn_erpm", up))
    if up <= 0.0:
        raise SystemExit("profile has no crossover.up_erpm — the firmware needs the seam to tag regimes "
                         "and to gate the soft-sensor; add a `crossover:` block to the YAML")

    dev.cmd(f"curve {idx} begin {len(pts)}")
    for i, (t, r) in enumerate(pts):
        tag = ""
        if regs and regs[i]:
            tag = " l" if str(regs[i]).lower().startswith("l") else " s"
        dev.cmd(f"curve {idx} add {t:.1f} {r:.1f}{tag}")
    dev.cmd(f"curve {idx} commit {prof.pole_pairs} {up:.0f} {dn:.0f}")
    return pts


def readback(dev, idx):
    """Dump the live curve: returns (meta dict, [(thrust, rpm, regime_char), ...])."""
    meta, pts = {}, []
    for ln in dev.cmd(f"curve {idx}"):
        if ln.startswith("curve|"):
            meta = {kv.split("=")[0]: kv.split("=")[1] for kv in ln.split("|")[2:] if "=" in kv}
        elif ln.startswith("pt|"):
            f = ln.split("|")
            pts.append((float(f[2]), float(f[3]), f[4]))
    return meta, pts


def main(argv=None):
    ap = argparse.ArgumentParser(description="upload a measured FF curve to the Pico firmware")
    ap.add_argument("index", type=int, help="ESC index (as used by arm/rpm/sense)")
    ap.add_argument("profile", help="profile YAML (host/profiles/*.yaml)")
    ap.add_argument("--port", default=None, help="serial port (default: auto-detect)")
    ap.add_argument("--dry-run", action="store_true", help="run against the sim host, no hardware")
    ap.add_argument("--gains", action="store_true", help="also push the profile's control: gains")
    ap.add_argument("--motor", metavar="KV,PP,V",
                    help="run `motor <i> <kv> <pp> <v>` first — the soft-sensor still needs KV/V "
                         "(estimateV) even when the FF curve is measured")
    ap.add_argument("--save", action="store_true", help="run `cfg save` so the curve survives a reset")
    ap.add_argument("--apply-crossover", action="store_true",
                    help="write the profile's sine_cross_up/dn to the ESC instead of refusing on a "
                         "mismatch (the curve is only valid at the threshold it was measured at)")
    ap.add_argument("--skip-crossover-check", action="store_true",
                    help="push anyway without verifying the ESC threshold (you are on your own)")
    ap.add_argument("--max-points", type=int, default=FW_MAX_POINTS,
                    help=f"thin the curve to at most this many points (firmware cap {FW_MAX_POINTS})")
    opts = ap.parse_args(argv)

    prof = SpeedProfile.load(opts.profile)
    if opts.max_points > FW_MAX_POINTS:
        raise SystemExit(f"--max-points exceeds the firmware buffer ({FW_MAX_POINTS})")

    dev = open_host(opts)
    try:
        if not opts.skip_crossover_check:
            if not check_crossover(dev, opts.index, prof, opts.apply_crossover):
                return 2

        if opts.motor:
            kv, pp, v = (x.strip() for x in opts.motor.split(","))
            print("motor:", *dev.cmd(f"motor {opts.index} {kv} {pp} {v}"))

        sent = push(dev, opts.index, prof, opts.max_points)
        print(f"pushed {len(sent)} point(s) from {os.path.basename(opts.profile)} "
              f"(profile has {len(prof.points)})")

        meta, got = readback(dev, opts.index)
        print(f"readback: n={meta.get('n')} measured={meta.get('measured')} pp={meta.get('pp')} "
              f"up={meta.get('up')} dn={meta.get('dn')}")
        bad = 0
        if len(got) != len(sent):
            print(f"MISMATCH: sent {len(sent)} points, device reports {len(got)}")
            bad += 1
        for (ts, rs), (tg, rg, _) in zip(sent, got):
            # the device prints one decimal, so compare at that resolution
            if abs(ts - tg) > 0.05 or abs(rs - rg) > 0.05:
                print(f"MISMATCH: sent ({ts:.1f}, {rs:.1f}) got ({tg:.1f}, {rg:.1f})")
                bad += 1
        print("verify: OK — the live curve matches what was sent" if not bad
              else f"verify: {bad} MISMATCH(es)")

        if opts.gains and prof.control:
            c = prof.control
            for name, key in (("kp", "kp"), ("ki", "ki"), ("trim", "trim_max")):
                if key in c:
                    dev.cmd(f"gain {opts.index} {name} {float(c[key])}")
            print("gains:", {k: c[k] for k in ("kp", "ki", "trim_max") if k in c})

        if opts.save:
            print("cfg save:", *dev.cmd("cfg save"))
        elif not opts.dry_run:
            print("NOTE: not persisted — run `cfg save` (or pass --save) or this is lost on reset")
        return 1 if bad else 0
    finally:
        close = getattr(dev, "close", None)
        if close:
            close()


if __name__ == "__main__":
    sys.exit(main())
