#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""coastdown — spin up, cut the drive, and log the encoder while the rotor coasts to rest.

WHAT IT GETS YOU. With no drive torque the only thing decelerating the rotor is its own drag:

    J * dw/dt = -tau_drag(w)

A coast-down ALONE is rank-deficient — it only ever yields the RATIO tau/J, which is why the classic
recipe adds a known-inertia disc to get a second equation. But `host/duty_sweep.py` + the PSU current
already measured tau_drag(w) independently (tau = K_T * I_phase, 4.16..5.90 mN*m over 753..2922 rpm on
the 350KV bench), so J follows directly from the measured deceleration with NO disc:

    J = tau_drag(w) / (-dw/dt)

CAVEAT worth stating: tau_drag was measured while DRIVEN. Coasting has no commutation ripple and no
winding current, so its drag can be slightly lower than the driven torque — that biases J LOW. Treat
the result as a good first number, not a calibration.

ENCODER ONLY. Cutting the drive stops commutation, so telemetry dies with it — this measurement
cannot be done once the AS5600 comes off the bench. Position (`accum`) is differentiated rather than
trusting the device's windowed `rpm` field, which lags exactly when the signal is changing fastest.

The decay is FAST (~0.3-0.6 s from 2000 rpm), so the poll loop reads `encv` and nothing else.

  python3 host/coastdown.py 1 --rpm 2200 --repeats 3
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from esctool import EscHost, find_pico            # noqa: E402

COUNTS_PER_REV = 4096.0

# tau_drag(rpm) measured 2026-07-29 (duty_sweep + PSU current, K_T = 9.5493/350) for THIS rotor+prop.
# Hard-coded on purpose — it is a measurement, not a tunable — but that makes the J it yields valid for
# this bench ONLY. On any other rotor, re-measure with duty_sweep + a PSU current reading and replace it.
DRAG_TABLE = [(753, 5.90e-3), (1073, 5.21e-3), (1378, 4.89e-3), (1685, 4.68e-3),
              (2000, 4.51e-3), (2310, 4.35e-3), (2616, 4.23e-3), (2922, 4.16e-3)]


def drag_at(rpm):
    """Linear interpolation on the measured drag table; clamped outside it."""
    a = abs(rpm)
    if a <= DRAG_TABLE[0][0]:
        return DRAG_TABLE[0][1]
    if a >= DRAG_TABLE[-1][0]:
        return DRAG_TABLE[-1][1]
    for i in range(1, len(DRAG_TABLE)):
        x0, y0 = DRAG_TABLE[i - 1]
        x1, y1 = DRAG_TABLE[i]
        if a <= x1:
            return y0 + (y1 - y0) * (a - x0) / (x1 - x0)
    return DRAG_TABLE[-1][1]


def _encv(dev):
    """encv|<accum>|<rpm>|<samples>|<md> -> (accum_counts, windowed_rpm)."""
    try:
        for ln in dev.cmd("encv", timeout=2):
            if ln.startswith("encv|"):
                p = ln.split("|")
                return int(p[1]), float(p[2])
    except Exception:
        pass
    return None, None


def spin_up(dev, idx, rpm, opts):
    dev.cmd(f"arm {idx} bidir", timeout=8)
    time.sleep(3.0)
    dev.cmd(f"rpm {idx} {rpm:g}", timeout=6)
    t_end = time.time() + opts.settle
    while time.time() < t_end:
        _, r = _encv(dev)
        if r is not None and abs(r) > abs(rpm) * 0.9:
            time.sleep(1.5)          # let it actually settle, not just arrive
            return True
        time.sleep(0.25)
    return False


def coast(dev, idx, opts):
    """Cut the drive and poll position as fast as the link allows."""
    dev.cmd(f"disarm {idx}", timeout=4)
    t0 = time.time()
    rows = []
    while time.time() - t0 < opts.window:
        a, r = _encv(dev)
        if a is None:
            continue
        rows.append((time.time() - t0, a, r))
        if len(rows) > 6 and all(abs(x[2] or 0) < opts.stop_rpm for x in rows[-4:]):
            break
    return rows


def _quadfit(ts, ys):
    """Least-squares y = a + b*dt + c*dt^2 about the window mean. Returns (b, c) or None.

    Normal equations solved by hand — a 3x3 symmetric system, no numpy dependency. t is centred
    first: raw wall-clock times make the Vandermonde matrix hopelessly ill-conditioned.
    """
    n = len(ts)
    if n < 6:
        return None
    tm = sum(ts) / n
    d = [t - tm for t in ts]
    s0, s1, s2 = float(n), sum(d), sum(x * x for x in d)
    s3, s4 = sum(x ** 3 for x in d), sum(x ** 4 for x in d)
    y0, y1, y2 = sum(ys), sum(x * y for x, y in zip(d, ys)), sum(x * x * y for x, y in zip(d, ys))
    # [s0 s1 s2; s1 s2 s3; s2 s3 s4] [a b c]^T = [y0 y1 y2]^T
    m = [[s0, s1, s2, y0], [s1, s2, s3, y1], [s2, s3, s4, y2]]
    for col in range(3):                                  # Gaussian elimination w/ partial pivoting
        piv = max(range(col, 3), key=lambda r: abs(m[r][col]))
        if abs(m[piv][col]) < 1e-18:
            return None
        m[col], m[piv] = m[piv], m[col]
        for r in range(3):
            if r == col:
                continue
            f = m[r][col] / m[col][col]
            for k in range(col, 4):
                m[r][k] -= f * m[col][k]
    b = m[1][3] / m[1][1]
    c = m[2][3] / m[2][2]
    return b, c


def derive(rows, win_s=0.06, step=6):
    """Windowed QUADRATIC fit of POSITION -> (t, rpm, dwdt_rpm_per_s) at each window centre.

    Differentiating a quantised position signal TWICE by central differences is hopeless here: over an
    8 ms span the speed changes ~3 %, so the second difference is pure noise (a first attempt spread J
    over three decades). Fitting theta = a + b*dt + c*dt^2 across a whole window instead uses every
    sample at once — b IS the speed and 2c IS the acceleration, both least-squares, no differencing.
    """
    out = []
    i = 0
    while i < len(rows):
        t_c = rows[i][0]
        w = [r for r in rows if t_c <= r[0] <= t_c + win_s]
        if len(w) >= 8:
            ts = [r[0] for r in w]
            ys = [r[1] / COUNTS_PER_REV * 60.0 for r in w]        # counts -> revolutions*60
            fit = _quadfit(ts, ys)
            if fit:
                b, c = fit                                        # b = rev/min... (rpm), c = half accel
                out.append((sum(ts) / len(ts), b, 2.0 * c))
        i += step
    return out


def coast_start(res, frac=0.97):
    """Index of the first window where the speed has actually begun to fall.

    `disarm` does NOT cut the drive instantly — measured on the bench, |w| holds flat for ~0.16 s
    (DShot frame timing + the ESC's own stop handling) before the true coast begins. Fitting that
    plateau in mixes still-driven samples into a drag measurement and drags the answer by ~10x, so the
    coast is defined to start where the speed has dropped `1-frac` below its plateau.
    """
    if not res:
        return 0
    peak = max(abs(r[1]) for r in res)
    for i, r in enumerate(res):
        if abs(r[1]) < peak * frac:
            return i
    return len(res)


def report(all_runs, opts):
    print("\n=== J from the measured drag: J = tau_drag(w) / (-dw/dt) ===")
    print(f"{'run':>4}{'rpm':>9}{'dw/dt[rpm/s]':>14}{'tau[mN*m]':>11}{'J[kg*m^2]':>12}")
    js = []
    for k, res in enumerate(all_runs, 1):
        res = res[coast_start(res):]          # drop the still-driven plateau before the real coast
        for t, rpm, dwdt in res:
            a = abs(rpm)
            # DECELERATING means |w| is falling, i.e. dw/dt OPPOSES w — not "dw/dt < 0". The AS5600
            # reads negative for a positive command on this bench, so a raw `dwdt < 0` test silently
            # discarded every valid sample and kept only noise (it put J across three decades).
            slowing = (rpm * dwdt) / a           # d|w|/dt = sign(w)*dw/dt; negative while coasting down
            if a < opts.fit_lo or a > opts.fit_hi or slowing >= 0:
                continue
            tau = drag_at(a)
            j = tau / (abs(slowing) * 2 * 3.141592653589793 / 60.0)
            dwdt = slowing
            js.append(j)
            if len(js) <= 40:
                print(f"{k:>4}{a:>9.0f}{dwdt:>14.0f}{tau * 1e3:>11.2f}{j:>12.3e}")
    if not js:
        print("  (no usable samples — try a higher --rpm or a wider --fit-lo/--fit-hi)")
        return
    js.sort()
    med = js[len(js) // 2]
    print(f"\n  n={len(js)}  median J = {med:.3e} kg*m^2   "
          f"(spread {js[0]:.2e} .. {js[-1]:.2e})")
    print(f"  mechanical time constant J*R/K_T^2 = "
          f"{med * opts.r / (9.5493 / opts.kv) ** 2 * 1e3:.1f} ms "
          f"(vs the ~37 ms the loop actually sees => still firmware-dominated if much smaller)")


def main():
    ap = argparse.ArgumentParser(description="coast-down: drag-based inertia + b(w) baseline")
    ap.add_argument("index", type=int, nargs="?", default=1)
    ap.add_argument("--rpm", type=float, default=2200.0, help="spin-up target before the cut")
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--window", type=float, default=6.0, help="s to log after the cut")
    ap.add_argument("--settle", type=float, default=25.0, help="s to reach the target")
    ap.add_argument("--stop-rpm", type=float, default=40.0, help="treat as stopped below this")
    # Below ~1500 rpm the measured deceleration is ~4x what the drag table predicts (consistent across
    # runs), i.e. something brakes the rotor there that the DRIVEN torque measurement does not see —
    # the ESC's zero-throttle state is the prime suspect. Only the high-speed window is usable for J.
    ap.add_argument("--fit-lo", type=float, default=1550.0, help="only use samples above this rpm")
    ap.add_argument("--fit-hi", type=float, default=2900.0, help="...and below this")
    ap.add_argument("--kv", type=float, default=350.0)
    ap.add_argument("--r", type=float, default=0.7)
    ap.add_argument("--win", type=float, default=0.06, help="quadratic-fit window, s (default 0.06)")
    ap.add_argument("--analyze", default=None, help="re-fit an existing CSV; no motor run")
    ap.add_argument("--out", default=None)
    ap.add_argument("--port", default=None)
    ap.add_argument("--dry-run", action="store_true")
    opts = ap.parse_args()

    if opts.analyze:
        by_run = {}
        with open(opts.analyze) as f:
            for r in csv.DictReader(f):
                by_run.setdefault(int(r["run"]), []).append(
                    (float(r["t"]), int(r["accum"]), float(r["encv_rpm"])))
        runs = [derive(rows, opts.win) for rows in by_run.values()]
        print(f"# re-fit {sum(len(v) for v in by_run.values())} samples from {len(by_run)} run(s)")
        report(runs, opts)
        return 0

    if opts.dry_run:
        print(f"# DRY RUN — {opts.repeats}x: spin ESC {opts.index} to {opts.rpm:g} rpm, disarm, "
              f"log encv for {opts.window:g}s")
        print("#   drive is CUT, so telemetry dies — the encoder is the only instrument. Always disarms.")
        return 0

    out = opts.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), "reports",
                                   f"coastdown_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    dev = EscHost(find_pico(opts.port))
    runs, all_rows = [], []
    try:
        dev.cmd("run", timeout=6)
        dev.cmd("disconnect", timeout=6)
        time.sleep(0.4)
        for k in range(opts.repeats):
            print(f"\n=== run {k + 1}/{opts.repeats}: spinning to {opts.rpm:g} rpm ===")
            if not spin_up(dev, opts.index, opts.rpm, opts):
                print("  !! never reached the target — skipping this run")
                dev.cmd(f"rpm {opts.index} 0", timeout=6)
                dev.cmd(f"disarm {opts.index}", timeout=6)
                continue
            rows = coast(dev, opts.index, opts)
            if not rows:
                # No encoder samples (AS5600 unplugged, or `encv` erroring, which _encv swallows).
                # Skip rather than IndexError out — that would also throw away the CSV of any earlier
                # successful repeats, which is the expensive part.
                print("  !! no encoder samples — skipping this run")
                continue
            res = derive(rows, opts.win)
            print(f"  {len(rows)} samples over {rows[-1][0]:.2f}s "
                  f"({len(rows) / max(rows[-1][0], 1e-3):.0f} Hz), {len(res)} usable derivatives")
            runs.append(res)
            all_rows += [(k + 1,) + r for r in rows]
            time.sleep(2.0)
    except KeyboardInterrupt:
        print("\n!! interrupted")
    finally:
        for step in (f"rpm {opts.index} 0", f"disarm {opts.index}"):
            try:
                dev.cmd(step, timeout=6)
            except Exception as e:
                print(f"# WARNING: '{step}' failed ({e})")
            time.sleep(0.4)
        print("# stopped, disarmed")

    if all_rows:
        with open(out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["run", "t", "accum", "encv_rpm"])
            w.writerows(all_rows)
        print(f"# {len(all_rows)} samples -> {out}")
    if runs:
        report(runs, opts)
    return 0


if __name__ == "__main__":
    sys.exit(main())
