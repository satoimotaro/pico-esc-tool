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

THE ROTOR DOES NOT ACTUALLY COAST — measured 2026-07-30, bare shaft, no propeller. The ESC applies
ACTIVE BRAKING whenever the commanded throttle is zero, and it dominates the decay:

  * bare-shaft, no prop: 3100 rpm -> dead stop in 0.19-0.54 s. Aerodynamic drag on a propeller-less
    shaft cannot do that, so the braking torque is not aerodynamic.
  * the deceleration PEAKS PART-WAY DOWN and then falls away again — it is ~3500-5000 rpm/s above
    2500 rpm, peaks at ~48000 rpm/s at 1455-1596 rpm (3/3, +-5 %), and is back to ~4000 rpm/s below
    500 rpm. No passive law permits even the rise: aero drag goes as w^2, viscous as w, Coulomb is
    constant — all flat or falling, none can grow 10x while the speed collapses. And the ROLL-OFF on
    the far side rules out a constant brake too: a peak with fall-off on both sides is the signature
    of a SLIP-dependent torque (short-circuit / asynchronous braking, which peaks near w_e = R/L),
    not an ever-growing one. That shape is what makes the mechanism identifiable at all.
    THE "PEAK MOVES WITH LOAD" QUESTION IS SETTLED (2026-07-30, docs/measured-plant.md): it does not.
    The brake ENGAGES ON A FIXED DELAY — the ESC keeps driving for CUT_RELEASE_S (0.17 s on `disarm`),
    the rotor then coasts on drag alone for a further BRAKE_ENGAGE_S (~0.13 s), and only then does the
    brake bite. Bare-shaft the rotor is at ~2390 rpm by then and prop-on at ~1540 rpm, which is exactly
    the ratio the apparent peak shift (1455-1596 vs 865-973 rpm) showed. Both delays were measured in
    BOTH configurations and agree within 5 %, so they are ESC state-machine timings, not rotor
    properties. The brake TORQUE's load-independence is still open.
    NOT explained by one slip law: `zero_held` brakes ~3x harder than `disarm` from the same speed
    (32 ms vs 107 ms to a stop) and drives the rotor THROUGH zero, springing back to 105-148 rpm.
  * identical with `disarm`, with the 500 ms deadman simply expiring, and while ARMED holding zero
    throttle (fastest of the three, 0.19 s). So it is tied to ZERO COMMANDED THROTTLE, not to the
    disarm path. `brake_on_stop=0` does NOT disable it.

That resolves the long-standing "below ~1500 rpm the rotor decelerates ~4x faster than the drag table
predicts" anomaly: it was never a drag-table error, it is the ESC braking.

CONSEQUENCE FOR J. Only the brief window between the cut and the brake biting is a real drag
measurement, so `brake_onset()` finds it per run instead of trusting a fixed rpm band — the brake
bites at whatever speed the ESC decides, which moves with the rotor and the bench. The prop-on
J = 8.8e-6 survives because its 1550-2900 rpm fit window happened to sit inside that pre-brake phase,
but that was luck, not a law: do not reuse those rpm limits on another rotor.

ENCODER ONLY. Cutting the drive stops commutation, so telemetry dies with it — this measurement
cannot be done once the AS5600 comes off the bench. Position (`accum`) is differentiated rather than
trusting the device's windowed `rpm` field, which lags exactly when the signal is changing fastest.

The decay is FAST (~0.2-0.6 s), so the poll loop reads `encv` and nothing else.

  python3 host/coastdown.py 1 --rpm 2200 --repeats 3
  python3 host/coastdown.py 1 --thrust 800 --cut zero     # bare shaft: open-loop up, brake-mode study
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
# It lives in pico_esc.measured now, with its provenance record and the honest note that it FALLS with
# speed (which no drag law permits, so it is bearing/iron loss plus a K_T*I_dc bias, not prop drag).
# Re-exported here under the old names so every existing caller and test is untouched.
from pico_esc.measured import DRAG_TABLE_PROP as DRAG_TABLE     # noqa: E402,F401
from pico_esc.measured import drag_at                            # noqa: E402,F401


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
    """Closed-loop spin-up to `rpm`, or open-loop to a raw command with --thrust.

    WHY OPEN-LOOP EXISTS. The velocity controller cannot always traverse the crossover gap: with no
    propeller the shipped gains stall the rotor at ~450 rpm and `vc` correctly returns ABORT_STALL and
    stops, so a closed-loop spin-up simply never arrives. Raw `thrust` sails through the same region
    (measured: cmd 550 hands off, cmd 900 -> 4060 rpm bare-shaft). The coast itself is identical —
    `disarm` is still the cut — so the drag measurement does not care which way the rotor got there,
    and open-loop is the only way to get a bare-shaft baseline at all.
    """
    dev.cmd(f"arm {idx} bidir", timeout=8)
    time.sleep(3.0)
    if opts.thrust is not None:
        # Raw thrust is deadman-protected (500 ms), so it must be re-sent throughout the ramp AND the
        # settle — one write then sleep() would just let the ESC time out mid-measurement.
        t_end = time.time() + opts.settle
        last = None
        while time.time() < t_end:
            dev.cmd(f"thrust {idx} {opts.thrust:d}", timeout=4)
            time.sleep(0.15)
            _, r = _encv(dev)
            if r is None:
                continue
            # Plateau, not a target: open-loop has no setpoint to compare against, so settle on the
            # speed no longer changing by more than --plateau over a keep-alive tick.
            if last is not None and abs(r) > opts.stop_rpm and abs(abs(r) - abs(last)) < opts.plateau:
                t_hold = time.time() + 1.5
                while time.time() < t_hold:              # hold the command while it truly settles
                    dev.cmd(f"thrust {idx} {opts.thrust:d}", timeout=4)
                    time.sleep(0.15)
                return True
            last = r
        return False
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
    """Cut the drive and poll position as fast as the link allows.

    --cut selects HOW the drive is removed. All three brake (see the module docstring); the option
    exists because comparing them is what proved the brake belongs to zero throttle rather than to
    `disarm`, and re-running that comparison is the way to check any future ESC/config.
    """
    keepalive = False
    if opts.cut == "disarm":
        dev.cmd(f"disarm {idx}", timeout=4)
    elif opts.cut == "zero":
        keepalive = True                 # stay armed and keep commanding exactly zero
    # "deadman": send nothing at all and let the 500 ms deadman zero the command itself.
    t0 = time.time()
    rows = []
    while time.time() - t0 < opts.window:
        if keepalive:
            try:
                dev.cmd(f"thrust {idx} 0", timeout=3)
            except Exception:
                pass
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


def brake_onset(res, start, factor=2.0, probe=6):
    """Index where the ESC's zero-throttle brake bites — the end of the usable drag window.

    Everything after this point is the ESC braking, not drag, so feeding it to `J = tau/(-dw/dt)`
    understates J by up to ~10x. Detected as the first window whose |d|w|/dt| exceeds `factor` x the
    median of the first `probe` windows after the cut, rather than a fixed rpm threshold: the brake
    bites wherever the ESC decides, which moves with the rotor, the config and the bench.

    Returns len(res) when it never trips (a genuinely free coast — good, use all of it).
    """
    seg = res[start:start + probe]
    if len(seg) < 3:
        return len(res)
    base = sorted(abs((r[1] * r[2]) / abs(r[1])) for r in seg if abs(r[1]) > 1)
    if not base:
        return len(res)
    thresh = base[len(base) // 2] * factor
    for i in range(start + len(seg), len(res)):
        rpm, dwdt = res[i][1], res[i][2]
        if abs(rpm) > 1 and abs((rpm * dwdt) / abs(rpm)) > thresh:
            return i
    return len(res)


def report(all_runs, opts):
    print("\n=== J from the measured drag: J = tau_drag(w) / (-dw/dt) ===")
    print(f"{'run':>4}{'rpm':>9}{'dw/dt[rpm/s]':>14}{'tau[mN*m]':>11}{'J[kg*m^2]':>12}")
    js = []
    for k, res in enumerate(all_runs, 1):
        s = coast_start(res)                  # drop the still-driven plateau before the real coast
        e = len(res) if opts.no_brake_gate else brake_onset(res, s, opts.brake_factor)
        if e < len(res):
            print(f"#  run {k}: brake bites at t={res[e][0]:.3f}s / {abs(res[e][1]):.0f} rpm — "
                  f"{e - s} of {len(res) - s} post-cut windows are drag, the rest is the ESC")
        res = res[s:e]
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


def main(argv=None):
    ap = argparse.ArgumentParser(description="coast-down: drag-based inertia + b(w) baseline")
    ap.add_argument("index", type=int, nargs="?", default=1)
    ap.add_argument("--rpm", type=float, default=2200.0, help="spin-up target before the cut")
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--window", type=float, default=6.0, help="s to log after the cut")
    ap.add_argument("--settle", type=float, default=25.0, help="s to reach the target")
    ap.add_argument("--stop-rpm", type=float, default=40.0, help="treat as stopped below this")
    # These two used to be the ONLY guard against the ESC's brake, hand-picked for the prop-on 350KV
    # rotor back when the excess deceleration was still unexplained. They are now a secondary trim on
    # top of the automatic brake gate — keep them wide unless a specific rpm band is wanted.
    ap.add_argument("--fit-lo", type=float, default=1550.0, help="only use samples above this rpm")
    ap.add_argument("--fit-hi", type=float, default=2900.0, help="...and below this")
    ap.add_argument("--brake-factor", type=float, default=2.0,
                    help="brake gate: |dw/dt| above this x the post-cut baseline ends the drag window")
    ap.add_argument("--no-brake-gate", action="store_true",
                    help="disable the automatic brake gate and fit the whole post-cut decay")
    ap.add_argument("--cut", choices=("disarm", "deadman", "zero"), default="disarm",
                    help="how to remove drive: disarm | let the 500ms deadman expire | hold zero armed")
    ap.add_argument("--kv", type=float, default=350.0)
    ap.add_argument("--r", type=float, default=0.7)
    ap.add_argument("--win", type=float, default=0.06, help="quadratic-fit window, s (default 0.06)")
    ap.add_argument("--thrust", type=int, default=None,
                    help="spin up OPEN-LOOP to this raw command instead of closed-loop --rpm "
                         "(the only way to reach 6-step bare-shaft: the vc stalls in the gap)")
    ap.add_argument("--plateau", type=float, default=25.0,
                    help="open-loop settle: rpm change below this between keep-alives = arrived")
    ap.add_argument("--analyze", default=None, help="re-fit an existing CSV; no motor run")
    ap.add_argument("--out", default=None)
    ap.add_argument("--port", default=None)
    ap.add_argument("--dry-run", action="store_true")
    opts = ap.parse_args(argv)

    if opts.analyze:
        by_run = {}
        with open(opts.analyze) as f:
            rd = csv.DictReader(f)
            # `--cut`-comparison CSVs are keyed by `mode` (disarm/deadman/zero_held), plain repeat
            # CSVs by `run`. Only `run` was accepted, so the cutmode file — the one that PROVED the
            # brake belongs to zero throttle rather than to disarm — was unreadable by its own tool.
            key = "mode" if (rd.fieldnames and "mode" in rd.fieldnames) else "run"
            for r in rd:
                by_run.setdefault(r[key], []).append(
                    (float(r["t"]), int(r["accum"]), float(r["encv_rpm"])))
        runs = [derive(rows, opts.win) for rows in by_run.values()]
        print(f"# re-fit {sum(len(v) for v in by_run.values())} samples from {len(by_run)} "
              f"{key}(s): {', '.join(str(k) for k in by_run)}")
        report(runs, opts)
        return 0

    if opts.dry_run:
        how = (f"OPEN-LOOP to raw command {opts.thrust}" if opts.thrust is not None
               else f"closed-loop to {opts.rpm:g} rpm")
        cut = {"disarm": "disarm", "deadman": "let the 500ms deadman expire",
               "zero": "hold zero throttle while ARMED"}[opts.cut]
        print(f"# DRY RUN — {opts.repeats}x: spin ESC {opts.index} {how}, {cut}, "
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
                # Guarded like the finally-block below: a stalled spin-up leaves the vc already
                # ABORT_STALL'd and disarmed, so `rpm 0` answers "err not-armed" and an unguarded call
                # here raised straight out of main() — discarding the CSV of every earlier repeat, the
                # exact failure the no-samples path further down was already hardened against.
                for step in (f"rpm {opts.index} 0", f"disarm {opts.index}"):
                    try:
                        dev.cmd(step, timeout=6)
                    except Exception as e:
                        print(f"#   ('{step}' failed: {e})")
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
