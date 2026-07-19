#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""tune_posctl — grid-sweep the posctl cascade-PID gains and rank them.

Runs `posctl.py move` for a grid of (Kp, Kd, vmax) over a few move sizes and seeds,
parses the reach time / overshoot / final-error it prints, and ranks the sets by a
reach-time + overshoot-penalty score. Use it to pick a starting gain set, then bench
fine-tune (raise Kp until the real motor first overshoots, back off ~20%, add Kd).

  python tune_posctl.py                 # sweep against the SimEncEscHost plant model (fast, safe)
  python tune_posctl.py --real          # sweep on the REAL ESC+motor (spins it — watch it!)
  python tune_posctl.py --kp 8,12,16,20 --kd 0.3,0.6,1.0 --vmax 400,600 --moves 30,180,720

--real drops --dry-run so posctl drives the actual hardware; keep --tmax/--vmax gentle and
stay at the bench. The plant model underplays stiction/detent/inertia, so treat the sim
ranking as a starting point, not the final word — the real motor has overshoot headroom.
"""
from __future__ import annotations

import argparse
import itertools
import re
import statistics
import subprocess
import sys
from pathlib import Path

POSCTL = str(Path(__file__).with_name("posctl.py"))
_LINE = re.compile(
    r"(?:holding|settled).*t=([\d.]+)s final_err=([+-][\d.]+)deg peak_overshoot=([\d.]+)deg"
)


def _run(kp, kd, vmax, deg, seed, real, tmax, extra):
    cmd = [sys.executable, POSCTL, "move", "--deg", str(deg),
           "--kp", str(kp), "--kd", str(kd), "--vmax", str(vmax), "--tmax", str(tmax)]
    if real:
        cmd += ["--seed", str(seed)] if False else []   # seed is dry-run only
    else:
        cmd += ["--dry-run", "--no-autocal", "--seed", str(seed)]
    cmd += extra
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    except subprocess.TimeoutExpired:
        return None
    t = ov = fe = None
    for ln in r.stdout.splitlines():
        m = _LINE.search(ln)
        if m:
            t, fe, ov = float(m.group(1)), float(m.group(2)), float(m.group(3))
    if t is None:
        return None
    return t, ov, abs(fe)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--kp", default="6,10,14,18,24", help="comma list of Kp to try")
    ap.add_argument("--kd", default="0.3,0.6,1.0,1.5", help="comma list of Kd to try")
    ap.add_argument("--vmax", default="400,700", help="comma list of vmax (deg/s) to try")
    ap.add_argument("--moves", default="30,180,720", help="comma list of move sizes (deg)")
    ap.add_argument("--seeds", default="1234,7,99", help="dry-run RNG seeds to average over")
    ap.add_argument("--tmax", type=int, default=300, help="thrust ceiling passed through (default 300)")
    ap.add_argument("--ov-penalty", type=float, default=0.05,
                    help="score weight on max overshoot deg (default 0.05)")
    ap.add_argument("--top", type=int, default=16, help="how many rows to print (default 16)")
    ap.add_argument("--real", action="store_true",
                    help="sweep on REAL hardware (drops --dry-run; spins the motor)")
    ap.add_argument("rest", nargs=argparse.REMAINDER,
                    help="extra args passed to posctl (after --), e.g. --port, --ki, --tol")
    opts = ap.parse_args()

    kps = [float(x) for x in opts.kp.split(",")]
    kds = [float(x) for x in opts.kd.split(",")]
    vmaxs = [float(x) for x in opts.vmax.split(",")]
    moves = [float(x) for x in opts.moves.split(",")]
    seeds = [int(x) for x in opts.seeds.split(",")] if not opts.real else [0]
    extra = [a for a in opts.rest if a != "--"]

    if opts.real:
        print("# REAL-HARDWARE sweep: the motor will spin for every gain set. Watch it; Ctrl-C aborts.")
    grid = []
    for kp, kd, vmax in itertools.product(kps, kds, vmaxs):
        rows, ok = [], True
        for deg in moves:
            ts, ovs, fes = [], [], []
            for s in seeds:
                res = _run(kp, kd, vmax, deg, s, opts.real, opts.tmax, extra)
                if res is None:
                    ok = False
                    break
                ts.append(res[0]); ovs.append(res[1]); fes.append(res[2])
            if not ok:
                break
            rows.append((deg, statistics.mean(ts), max(ovs), statistics.mean(fes)))
        if not ok:
            print(f"# skip Kp={kp} Kd={kd} vmax={vmax} (a move failed/aborted)")
            continue
        reach = sum(r[1] for r in rows)
        maxov = max(r[2] for r in rows)
        meanfe = statistics.mean(r[3] for r in rows)
        grid.append((reach + opts.ov_penalty * maxov, kp, kd, vmax, reach, maxov, meanfe, rows))

    grid.sort()
    print(f"\n{'score':>6} {'kp':>4} {'kd':>4} {'vmax':>5} {'sumT':>6} {'maxOv':>6} {'meanFE':>6}   per-move(deg:t/ov)")
    for score, kp, kd, vmax, reach, maxov, meanfe, rows in grid[:opts.top]:
        pm = " ".join(f"{int(d)}:{t:.2f}/{o:.1f}" for d, t, o, f in rows)
        print(f"{score:6.2f} {kp:4g} {kd:4g} {vmax:5g} {reach:6.2f} {maxov:6.1f} {meanfe:6.1f}   {pm}")
    if grid:
        _, kp, kd, vmax, *_ = grid[0]
        print(f"\n# best-by-score: --kp {kp:g} --kd {kd:g} --vmax {vmax:g}  "
              f"(sim ranking — bench-confirm; real motor has overshoot headroom)")


if __name__ == "__main__":
    main()
