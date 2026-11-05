#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""azsumdiff — the azimuth plant, identified the only way this mechanism permits.

THE METHOD RULE (operator, 2026-08-15). **Never evaluate this pod with one motor stopped.** With one
input braked the bevel differential still has a free degree of freedom, so the split between CARRIER
(azimuth) and PLANET (propeller) is set by FRICTION, not by the mechanism. The kinematics constrain
nothing until BOTH motors are driven. Every solo / braked measurement in
`docs/azimuth-closed-loop.md` is void for that reason, including "motor A is 4.8x weaker".

WHAT THIS MEASURES. With SIGNED motor speeds and 1:1 on both sides (operator), the mechanism is

    azimuth   = (wA + wB) / 2  =  S / 2        <- the SUM channel
    propeller = (wA - wB) / 2  =  D / 2        <- the DIFFERENCE channel

so the experiment is to drive both motors always, and move ONE CHANNEL AT A TIME:

  * sweep S at constant D  -> the azimuth should move linearly and the propeller not at all;
  * sweep D at constant S=0 -> the propeller should move and the azimuth should stay put.

That separation is the entire point. The pair of points this file's §13 was built on (same-sign vs
opposite-sign at matched magnitude) changes BOTH channels at once, so it confounds the kinematic
term with prop reaction torque — and the pod's azimuth axis is free in air, so that torque is not
negligible. Holding D fixed makes the reaction torque a CONSTANT across the sweep, where it lands in
the intercept instead of in the slope.

PREDICTION, if the geometry above is right:

    rate = 3.00 * S  +  0.00 * D  +  0      (deg/s;  1 rpm of azimuth = 6 deg/s, azimuth = S/2)

DISCIPLINE INHERITED FROM azrpmcal (both were paid for the hard way):
  * the host is SILENT through the integration window — USB round trips land in the same core0 loop
    that must call As5600Tracker::poll() every 800 us, and polling inside the window was worth a
    factor of TWO in the answer (§7);
  * `encd` certifies every point against the 2048-count unwrap fold — never samples/s (§7);
  * ESC telemetry is PROVENANCE ONLY, never a regressor: `Telem.rpm` is averaged well below the
    3-10 Hz band the pod moves in and correlates ~0.07 with the true azimuth velocity (§9). It is
    here to answer "was the pair GIVEN what the fit assumes", nothing more.
  * ESC tempC is recorded at every point. The bench is not stationary: a 16 C asymmetry between the
    two ESCs is enough to move the operating point between sweeps (§16), and it was never logged.

    python3 azsumdiff.py --diff -2400 --sum -800 -400 0 400 800     # the SUM sweep
    python3 azsumdiff.py --sum 0 --diff -1600 -2000 -2400 -2800     # the DIFFERENCE sweep
    python3 azsumdiff.py --both                                     # both, in one session
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pico_esc.link import EscHost                                   # noqa: E402

COUNTS_PER_REV = 4096.0
DEG_S_PER_RPM = 6.0


def _encv(d):
    """(accum_counts, rpm, samples) from `encv|accum|rpm|samples|md`."""
    for line in d.cmd("encv", timeout=3):
        p = line.strip().split("|")
        if p[0] == "encv":
            return int(p[1]), float(p[2]), int(p[3])
    raise RuntimeError("no encv reply")


def _encd(d):
    """`encd|samples|miss|maxgap_us|maxstep|alias_margin` — the aliasing verdict for this window."""
    for line in d.cmd("encd", timeout=3):
        p = line.strip().split("|")
        if p[0] == "encd":
            return {"samples": int(p[1]), "miss": int(p[2]), "maxgap_us": int(p[3]),
                    "maxstep": int(p[4]), "margin": float(p[5])}
    return {"samples": 0, "miss": 0, "maxgap_us": 0, "maxstep": 0, "margin": float("nan")}


def _tele(d, i):
    """(|mech rpm|, tempC) — magnitude only, and PROVENANCE ONLY. See the module docstring."""
    try:
        for line in d.cmd(f"tele {i}", timeout=3):
            p = line.strip().split("|")
            if p[0] == "tele":
                return float(p[1]), float(p[4])
    except Exception:
        return None, None
    return None, None


def _sense(d, i):
    """(duty cmd, trim, valid, why) — the ESC's OWN opinion of whether it is tracking.

    `Telem.rpm` is unsigned and goes stale (§3), so a magnitude that disagrees with the target is
    ambiguous: the motor may really be running fast, or the number may be a leftover. `cmd` is the
    duty the velocity loop is actually issuing, so a large |cmd| alongside a large |rpm| means the
    over-speed is REAL, and a normal |cmd| alongside a large |rpm| means the telemetry is the liar.
    """
    try:
        for line in d.cmd(f"sense {i}", timeout=3):
            p = line.strip().split("|")
            if p[0] != "sense":
                continue
            kv = dict(x.split("=", 1) for x in p[2:] if "=" in x)
            return (float(kv.get("cmd", "nan")), float(kv.get("trim", "nan")),
                    int(kv.get("valid", "0")), kv.get("why", "?"))
    except Exception:
        pass
    return float("nan"), float("nan"), 0, "?"


def measure(d, args, S, D):
    """One point: command the pair as (S, D), settle, then integrate the encoder over the window."""
    a_t = (S + D) / 2.0
    b_t = (S - D) / 2.0
    d.cmd(f"rpm {args.idx_a} {a_t:.0f}", timeout=5)
    d.cmd(f"rpm {args.idx_b} {b_t:.0f}", timeout=5)
    time.sleep(args.settle)
    ta = tb = ca = cb = 0.0
    n = 0
    for _ in range(args.tele_n):                                  # OUTSIDE the window, on purpose
        ra, tca = _tele(d, args.idx_a)
        rb, tcb = _tele(d, args.idx_b)
        if ra is not None and rb is not None:
            ta += ra
            tb += rb
            ca += tca or 0.0
            cb += tcb or 0.0
            n += 1
    sa = _sense(d, args.idx_a)
    sb = _sense(d, args.idx_b)
    d.cmd("encd reset", timeout=3)
    acc0, _, _ = _encv(d)
    t0 = time.time()
    time.sleep(args.window)                                       # <-- nothing on the wire
    acc1, _, _ = _encv(d)
    dt = time.time() - t0
    diag = _encd(d)
    rate = (acc1 - acc0) * 360.0 / COUNTS_PER_REV / dt            # deg/s, signed
    ma, mb = (ta / n, tb / n) if n else (float("nan"), float("nan"))
    tempa, tempb = (ca / n, cb / n) if n else (float("nan"), float("nan"))
    # Delivery check on MAGNITUDES (tele.rpm is unsigned). This decides whether the point is a
    # measurement of the plant at all — a motor on the 6-step seam is not delivering |target|.
    da = ma - abs(a_t)
    db = mb - abs(b_t)
    return {"S": S, "D": D, "rate": rate, "a_tgt": a_t, "b_tgt": b_t, "rpm_a": ma, "rpm_b": mb,
            "d_a": da, "d_b": db, "temp_a": tempa, "temp_b": tempb, "dt": dt,
            "cmd_a": sa[0], "trim_a": sa[1], "valid_a": sa[2], "why_a": sa[3],
            "cmd_b": sb[0], "trim_b": sb[1], "valid_b": sb[2], "why_b": sb[3], **diag}


def lstsq3(rows, ys):
    """Solve the 3-column normal equations by Gaussian elimination. rows = [[1, S, D], ...]."""
    k = 3
    ata = [[sum(r[i] * r[j] for r in rows) for j in range(k)] for i in range(k)]
    aty = [sum(r[i] * y for r, y in zip(rows, ys)) for i in range(k)]
    m = [ata[i] + [aty[i]] for i in range(k)]
    for c in range(k):
        piv = max(range(c, k), key=lambda r: abs(m[r][c]))
        if abs(m[piv][c]) < 1e-12:
            return None
        m[c], m[piv] = m[piv], m[c]
        for r in range(k):
            if r == c:
                continue
            f = m[r][c] / m[c][c]
            for j in range(c, k + 1):
                m[r][j] -= f * m[c][j]
    return [m[i][k] / m[i][i] for i in range(k)]


def fit1(xs, ys):
    """rate = slope*x + b."""
    n = len(xs)
    if n < 2:
        return None, None
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0:
        return None, None
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return slope, my - slope * mx


def refit(paths):
    """Re-fit saved runs, offline. This is how a MECHANICAL change gets judged.

    The two coefficients are the whole result, and the only honest test of a repair (a grub screw
    against a slipping coupling, say) is the same operating points before and after: a hand check is
    low-torque by nature, so it can prove a drive path is broken but never that it holds at speed.
    """
    import csv
    print(f"{'run':>34s} {'n':>3s} {'kA':>8s} {'kB':>8s} {'const':>7s} {'rms':>6s} "
          f"{'|kB/kA|':>8s}   [1:1 bevel differential: kA = kB = +-0.5]")
    groups = [(p, [p]) for p in paths]
    if len(paths) > 1:
        groups.append(("POOLED", paths))
    for label, ps in groups:
        rows = []
        for p in ps:
            with open(p) as f:
                rows += list(csv.DictReader(ln for ln in f if not ln.startswith("#")))
        ok = [r for r in rows if abs(float(r["d_a"])) < 60 and abs(float(r["d_b"])) < 60]
        if len(ok) < 3:
            print(f"{os.path.basename(str(label)):>34s} {len(ok):3d}   too few delivered points")
            continue
        X = [[1.0, float(r["a_tgt"]), float(r["b_tgt"])] for r in ok]
        y = [float(r["rate_deg_s"]) for r in ok]
        c = lstsq3(X, y)
        if not c:
            # One sweep holds its other channel CONSTANT, so that column is rank-deficient and both
            # coefficients are not identifiable from it alone. Say so — do not drop the row.
            print(f"{os.path.basename(str(label)):>34s} {len(ok):3d}   one channel held constant "
                  f"— pool it with the other sweep to identify kA and kB")
            continue
        res = [yy - (c[0] + c[1] * x[1] + c[2] * x[2]) for x, yy in zip(X, y)]
        rms = (sum(r * r for r in res) / len(res)) ** 0.5
        ka, kb = c[1] / DEG_S_PER_RPM, c[2] / DEG_S_PER_RPM
        ratio = abs(kb / ka) if ka else float("inf")
        print(f"{os.path.basename(str(label)):>34s} {len(ok):3d} {ka:+8.3f} {kb:+8.3f} "
              f"{c[0]:+7.0f} {rms:6.0f} {ratio:8.1f}")
    print("#\n# kA is the one that matters: +-0.5 means motor A reaches the azimuth as the "
          "mechanism\n#   requires; ~0 means it does not, whatever the motor itself is doing.")


def report_signs(pts):
    """Score the four sign combinations against every hypothesis that fits a single point.

    `Telem.rpm` is UNSIGNED, so no amount of telemetry can say whether a motor commanded positive
    actually reversed. One point cannot separate "azimuth = sum, and B ignored its sign" from
    "azimuth = difference, and B honoured it" — both predict the same magnitude. Four points can:
    each hypothesis predicts a DIFFERENT pattern of which combinations spin and which stand still.
    """
    got = {}
    for q in pts:
        a, b = q["a_tgt"], q["b_tgt"]
        got.setdefault(((a > 0) - (a < 0), (b > 0) - (b < 0)), []).append(q["rate"])
    mag = max((abs(q["a_tgt"]) for q in pts), default=0.0)
    full = mag * DEG_S_PER_RPM                       # rate if the azimuth runs at one motor's speed
    # (sgn A, sgn B) -> predicted rate, per hypothesis. Signs as COMMANDED.
    hyps = {
        "azimuth = (A+B)/2, both signs honoured  [the CAD geometry, §2c]":
            lambda sa, sb: (sa + sb) / 2.0 * full,
        "azimuth = (A-B)/2, both signs honoured  [the roles are swapped]":
            lambda sa, sb: (sa - sb) / 2.0 * full,
        "azimuth = (A+B)/2, but B never reverses [B's sign is ignored]":
            lambda sa, sb: (sa + -1) / 2.0 * full,
        "azimuth = (A+B)/2, but A never reverses [A's sign is ignored]":
            lambda sa, sb: (-1 + sb) / 2.0 * full,
        "azimuth = (A+B)/2, neither reverses     [both signs ignored]":
            lambda sa, sb: -1.0 * full,
    }
    keys = sorted(got)
    print(f"#\n# SIGN TEST at |{mag:.0f}| rpm — one motor's speed on the azimuth would be "
          f"{full:.0f} deg/s")
    cols = [(-1, -1), (-1, 1), (1, -1), (1, 1)]
    lab = "sgn(A)sgn(B):"
    print("#   " + lab + "  ".join(
        f"{('+' if a > 0 else '-') + ('+' if b > 0 else '-'):>7s}" for a, b in cols))
    print("#   " + "measured:".ljust(len(lab)) + "  ".join(
        f"{sum(got[k]) / len(got[k]):+7.0f}" if k in got else f"{'?':>7s}" for k in cols))
    best, besterr = None, None
    for name, f in hyps.items():
        err = sum(abs(f(*k) - sum(got[k]) / len(got[k])) for k in keys) / len(keys)
        print("#   " + "".ljust(len(lab)) + "  ".join(f"{f(*k):+7.0f}" for k in cols) +
              f"   mean|err| {err:6.0f}   {name}")
        if besterr is None or err < besterr:
            best, besterr = name, err
    print(f"#   => best fit: {best}  (mean|err| {besterr:.0f} deg/s)")


def interleave(vals):
    """Visit extremes alternately so the pod does not wind one way for the whole sweep."""
    s = sorted(vals, key=abs, reverse=True)
    out, lo, hi = [], [v for v in s if v < 0], [v for v in s if v >= 0]
    while lo or hi:
        if hi:
            out.append(hi.pop(0))
        if lo:
            out.append(lo.pop(0))
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port")
    p.add_argument("--idx-a", type=int, default=0)
    p.add_argument("--idx-b", type=int, default=1)
    p.add_argument("--sum", type=float, nargs="+", default=[0.0],
                   help="S = wA + wB values to visit (the AZIMUTH channel)")
    p.add_argument("--diff", type=float, nargs="+", default=[-2400.0],
                   help="D = wA - wB values to visit (the PROPELLER channel)")
    p.add_argument("--both", action="store_true",
                   help="run the canonical pair of sweeps: S swept at D=-2400, then D swept at S=0")
    p.add_argument("--pairs", type=float, nargs="+", default=None, metavar="A B",
                   help="explicit signed (A B) targets, flattened. Overrides --sum/--diff. This is "
                        "how the SIGN question gets decided: `Telem.rpm` is unsigned, so it cannot "
                        "say whether a motor commanded positive actually reversed. The four sign "
                        "combinations at one magnitude give a different signature for every "
                        "hypothesis -- see --signs")
    p.add_argument("--signs", type=float, default=None, metavar="MAG",
                   help="shorthand for the four sign combinations at magnitude MAG: "
                        "(-M,-M) (-M,+M) (+M,-M) (+M,+M)")
    p.add_argument("--repeat", type=int, default=1,
                   help="visits per point. The scatter IS a result — do not average it away")
    p.add_argument("--min-rpm", type=float, default=750.0,
                   help="refuse any point that would command a motor below this magnitude. The "
                        "measured low-side delivery floor is ~800 rpm: below it the ESC does not "
                        "deliver the target and the point is not a measurement of the plant")
    p.add_argument("--max-rate", type=float, default=5000.0,
                   help="stop the sweep if a point exceeds this azimuth rate, deg/s")
    p.add_argument("--prime-rpm", type=float, default=700.0,
                   help="come up MATCHED at -this on both motors before the first point, so the "
                        "pair crosses the 6-step seam on identical commands (cross_zero_offset's "
                        "rule). 0 goes straight to the first point")
    p.add_argument("--tele-n", type=int, default=4)
    p.add_argument("--settle", type=float, default=1.5, help="seconds before the window opens")
    p.add_argument("--window", type=float, default=2.0, help="seconds integrated per point")
    p.add_argument("--slew", type=float, default=2000.0)
    p.add_argument("--arm-secs", type=float, default=4.5)
    p.add_argument("--name", default="sumdiff")
    p.add_argument("--refit", nargs="+", default=None, metavar="CSV",
                   help="do not touch the hardware: re-fit saved runs and print the two "
                        "coefficients. Pass several CSVs to pool them, or one set before a "
                        "mechanical change and one after to compare kA/kB directly")
    args = p.parse_args()

    if args.refit:
        refit(args.refit)
        return

    if args.signs:
        m = abs(args.signs)
        args.pairs = [-m, -m, -m, m, m, -m, m, m]
    if args.pairs:
        if len(args.pairs) % 2:
            sys.exit("--pairs takes an even number of values: A B A B ...")
        prs = list(zip(args.pairs[0::2], args.pairs[1::2]))
        plan = [("P", a + b, a - b) for a, b in prs]
    elif args.both:
        plan = ([("S", s, -2400.0) for s in interleave([-800, -400, 0, 400, 800])] +
                [("D", 0.0, dd) for dd in interleave([-1600, -2000, -2400, -2800])])
    else:
        plan = ([("S", s, args.diff[0]) for s in interleave(args.sum)] if len(args.sum) > 1 else
                [("D", args.sum[0], dd) for dd in interleave(args.diff)])

    skipped = []
    runplan = []
    for tag, S, D in plan:
        a_t, b_t = (S + D) / 2.0, (S - D) / 2.0
        if min(abs(a_t), abs(b_t)) < args.min_rpm:
            skipped.append((S, D, a_t, b_t))
            continue
        runplan.append((tag, S, D))
    for S, D, a_t, b_t in skipped:
        print(f"# SKIP S={S:+.0f} D={D:+.0f} -> A {a_t:+.0f} B {b_t:+.0f}: below --min-rpm "
              f"{args.min_rpm:.0f}, would sit on the seam")
    if not runplan:
        sys.exit("every point is below --min-rpm — widen |D| or lower --min-rpm knowingly")

    d = EscHost(args.port)
    pts = []
    stopped = None
    try:
        for i in (args.idx_a, args.idx_b):
            d.cmd(f"arm {i} bidir", timeout=10)
            d.cmd(f"gain {i} slew {args.slew}", timeout=5)
        time.sleep(args.arm_secs)
        if args.prime_rpm > 0:
            d.cmd(f"rpm {args.idx_a} {-args.prime_rpm:.0f}", timeout=5)
            d.cmd(f"rpm {args.idx_b} {-args.prime_rpm:.0f}", timeout=5)
            time.sleep(2.0)
        print(f"{'ch':>2s} {'S':>6s} {'D':>6s} {'A_tgt':>6s} {'B_tgt':>6s} {'rate':>8s} "
              f"{'|A|':>6s} {'|B|':>6s} {'dA':>6s} {'dB':>6s} {'cmdA':>6s} {'cmdB':>6s} "
              f"{'whyA':>7s} {'whyB':>7s} {'Ta':>4s} {'Tb':>4s} {'margin':>6s}")
        for tag, S, D in runplan:
            for _k in range(args.repeat):
                pt = measure(d, args, S, D)
                pt["ch"] = tag
                pts.append(pt)
                bad = "" if abs(pt["d_a"]) < 60 and abs(pt["d_b"]) < 60 else "  <== NOT DELIVERED"
                thin = "" if pt["margin"] > 3.0 else "  <== ALIAS MARGIN THIN"
                print(f"{tag:>2s} {S:6.0f} {D:6.0f} {pt['a_tgt']:6.0f} {pt['b_tgt']:6.0f} "
                      f"{pt['rate']:8.0f} {pt['rpm_a']:6.0f} {pt['rpm_b']:6.0f} {pt['d_a']:6.0f} "
                      f"{pt['d_b']:6.0f} {pt['cmd_a']:6.0f} {pt['cmd_b']:6.0f} "
                      f"{pt['why_a']:>7s} {pt['why_b']:>7s} "
                      f"{pt['temp_a']:4.0f} {pt['temp_b']:4.0f} {pt['margin']:6.1f}{bad}{thin}")
                if abs(pt["rate"]) > args.max_rate:
                    stopped = f"S={S:+.0f} D={D:+.0f} exceeded --max-rate ({pt['rate']:.0f} deg/s)"
                    raise KeyboardInterrupt
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            for i in (args.idx_a, args.idx_b):
                for c in (f"rpm {i} 0", f"thrust {i} 0"):
                    try:
                        d.cmd(c, timeout=3)
                    except Exception:
                        pass
        for i in (args.idx_a, args.idx_b):
            try:
                d.cmd(f"disarm {i}", timeout=3)
            except Exception:
                pass

    if stopped:
        print(f"\n# STOPPED: {stopped}")

    ok = [q for q in pts if abs(q["d_a"]) < 60 and abs(q["d_b"]) < 60]
    print(f"\n# {len(ok)}/{len(pts)} points delivered what was commanded.")
    if any(q["ch"] == "P" for q in pts):
        report_signs(pts)
    sw = [q for q in ok if q["ch"] == "S"]
    dw = [q for q in ok if q["ch"] == "D"]
    if len(sw) >= 2:
        k, b = fit1([q["S"] for q in sw], [q["rate"] for q in sw])
        print(f"# SUM channel   (D fixed): d(rate)/dS = {k:7.3f} deg/s per rpm   "
              f"[geometry predicts {DEG_S_PER_RPM / 2:.2f}]   intercept {b:8.0f} deg/s")
        print(f"#   -> azimuth/motor ratio {k / (DEG_S_PER_RPM / 2):.3f}   "
              f"(1.000 == the 1:1 differential; the intercept is the constant-D reaction torque)")
    if len(dw) >= 2:
        k, b = fit1([q["D"] for q in dw], [q["rate"] for q in dw])
        print(f"# DIFF channel (S fixed 0): d(rate)/dD = {k:7.3f} deg/s per rpm   "
              f"[geometry predicts 0.00]   intercept {b:8.0f} deg/s")
        print(f"#   -> a nonzero slope here is the propeller LEAKING into the azimuth: reaction "
              f"torque against a free axis, or the two sides not actually 1:1")
    if len(ok) >= 4 and sw and dw:
        c = lstsq3([[1.0, q["S"], q["D"]] for q in ok], [q["rate"] for q in ok])
        if c:
            res = [q["rate"] - (c[0] + c[1] * q["S"] + c[2] * q["D"]) for q in ok]
            rms = (sum(r * r for r in res) / len(res)) ** 0.5
            print(f"\n# joint fit over all {len(ok)} points:")
            print(f"#   rate = {c[0]:+.0f} {c[1]:+.4f}*S {c[2]:+.4f}*D      "
                  f"rms residual {rms:.0f} deg/s")
            print(f"#   geometry says {0:+d} {DEG_S_PER_RPM / 2:+.4f}*S {0:+.4f}*D")

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "reports",
                       f"azsumdiff_{args.name}_{time.strftime('%Y%m%d-%H%M%S')}.csv")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        f.write(f"# settle={args.settle}|window={args.window}|slew={args.slew}|"
                f"prime={args.prime_rpm}|min_rpm={args.min_rpm}\n")
        f.write("ch,S,D,a_tgt,b_tgt,rate_deg_s,rpm_a,rpm_b,d_a,d_b,cmd_a,cmd_b,trim_a,trim_b,"
                "valid_a,valid_b,why_a,why_b,temp_a,temp_b,maxgap_us,maxstep,margin,dt\n")
        for q in pts:
            f.write(f"{q['ch']},{q['S']:.0f},{q['D']:.0f},{q['a_tgt']:.0f},{q['b_tgt']:.0f},"
                    f"{q['rate']:.1f},{q['rpm_a']:.1f},{q['rpm_b']:.1f},{q['d_a']:.1f},"
                    f"{q['d_b']:.1f},{q['cmd_a']:.0f},{q['cmd_b']:.0f},{q['trim_a']:.1f},"
                    f"{q['trim_b']:.1f},{q['valid_a']},{q['valid_b']},{q['why_a']},{q['why_b']},"
                    f"{q['temp_a']:.0f},{q['temp_b']:.0f},{q['maxgap_us']},"
                    f"{q['maxstep']},{q['margin']:.2f},{q['dt']:.3f}\n")
    print(f"# wrote {out}")


if __name__ == "__main__":
    main()
