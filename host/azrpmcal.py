#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""azrpmcal — the RPM-SPACE plant of the differential azimuth pod, measured STATICALLY.

    A target = -(spin_rpm) + r          azimuth rate = slope * (r - r_null)
    B target = -(spin_rpm) - r

WHY A SEPARATE TOOL. The constants the on-device loop is built on — `rpm_per_count 7.26` (from an
assumed 6.0 deg/s per rpm) and `rpm_trim -137` — were fitted on 2026-08-14 from points taken while
every run was aborting on the rate guard within 1-4 s, i.e. from SPIN-UP TRANSIENTS. They do not
survive contact with a run that lasts: on 2026-08-15 the loop sat railed at r = -200 for 11 s with
both ESCs holding 1000/600 rpm to +/-3 rpm, and the azimuth drifted +430 deg/s — where those
constants predict -378. Wrong sign, 800 deg/s out.

This measures the same plant with NO loop and NO abort in the way: hold r, let it settle, integrate
the encoder over a window, move on. The azimuth is free to rotate continuously, which is what makes
a static measurement possible at all — nothing here is a transient.

It also reads each ESC's telemetry at every point, because the calibration is only meaningful if the
plant was GIVEN what the fit assumes: if the pair does not actually sit at -(spin_rpm)+r / -(spin_rpm)-r,
the fitted slope belongs to some other experiment. Points where the delivery is off are flagged, not
silently averaged in.

    python3 azrpmcal.py --spin-rpm 800 --r -400 -300 -200 -100 0 100 200
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pico_esc.link import EscHost                                   # noqa: E402

COUNTS_PER_REV = 4096.0


def _encv(d):
    """(accum_counts, rpm, samples) from `encv|accum|rpm|samples|md`."""
    for line in d.cmd("encv", timeout=3):
        p = line.strip().split("|")
        if p[0] == "encv":
            return int(p[1]), float(p[2]), int(p[3])
    raise RuntimeError("no encv reply")


def _tele(d, i):
    """Unsigned mech rpm, or None if the ESC has nothing to report."""
    try:
        for line in d.cmd(f"tele {i}", timeout=3):
            p = line.strip().split("|")
            if p[0] == "tele":
                return float(p[1])
    except Exception:
        return None
    return None


def _encd(d):
    """`encd|samples|miss|maxgap_us|maxstep|alias_margin` -> the aliasing verdict for the last window.

    maxstep is the worst single unwrapped step in counts; the fold is at 2048, so alias_margin says
    how many times worse it would have to get before the accumulator starts losing revolutions.
    """
    for line in d.cmd("encd", timeout=3):
        p = line.strip().split("|")
        if p[0] == "encd":
            return {"samples": int(p[1]), "miss": int(p[2]), "maxgap_us": int(p[3]),
                    "maxstep": int(p[4]), "margin": float(p[5])}
    return {"samples": 0, "miss": 0, "maxgap_us": 0, "maxstep": 0, "margin": float("nan")}


def measure(d, args, r, pair=None):
    """One point: command the pair, settle, then integrate the encoder over the window.

    `pair` gives the two targets DIRECTLY, which is how the balance ratio has to be measured: the
    r-parametrisation ties |A| + |B| to 2*spin_rpm, so every offset large enough to reach the null
    also drags the low motor onto the 6-step seam, where the ESC cannot hold speed and the point is
    not a measurement of the plant at all (2026-08-15: b_tgt 600 delivered 332). Naming both speeds
    keeps BOTH above the seam while the ratio between them still sweeps.
    """
    if pair is not None:
        a_t, b_t = pair
    else:
        a_t = -args.spin_rpm + r
        b_t = -args.spin_rpm - r
    d.cmd(f"rpm {args.idx_a} {a_t:.0f}", timeout=5)
    d.cmd(f"rpm {args.idx_b} {b_t:.0f}", timeout=5)
    time.sleep(args.settle)
    # THE HOST MUST BE SILENT THROUGH THE WINDOW. The first version of this tool polled `tele 0` and
    # `tele 1` in a tight loop while integrating, i.e. it drove USB round trips into the same core0
    # loop that has to call As5600Tracker::poll() every 800 us. The tracker's sample rate fell from
    # 1004/s at rest to 626/s under exactly that load, and a stall longer than half a revolution of
    # travel makes the unwrap lose a whole turn without any error. That is the most likely reason the
    # SAME operating point (A 900, B 900) measured 2507 deg/s in one sweep and 5014 in another.
    # So: sample the motors OUTSIDE the window, and leave the link quiet inside it.
    ta, tb, n = 0.0, 0.0, 0
    for _ in range(args.tele_n):
        ra, rb = _tele(d, args.idx_a), _tele(d, args.idx_b)
        if ra is not None and rb is not None:
            ta += ra
            tb += rb
            n += 1
    d.cmd("encd reset", timeout=3)
    acc0, _, _ = _encv(d)
    t0 = time.time()
    time.sleep(args.window)                                     # <-- nothing on the wire
    acc1, _, _ = _encv(d)
    dt = time.time() - t0
    diag = _encd(d)
    rate = (acc1 - acc0) * 360.0 / COUNTS_PER_REV / dt          # deg/s, signed
    ma, mb = (ta / n, tb / n) if n else (float("nan"), float("nan"))
    # The delivered offset, from the MEASURED speeds. tele.rpm is unsigned, and both targets are
    # negative here, so |A| = spin_rpm - r and |B| = spin_rpm + r  =>  r_delivered = (|B| - |A|) / 2.
    r_del = (mb - ma) / 2.0
    return {"r": r, "rate": rate, "a_tgt": a_t, "b_tgt": b_t, "rpm_a": ma, "rpm_b": mb,
            "r_del": r_del, "dt": dt, **diag}


def fit(points):
    """Least squares rate = slope * (r - r_null), over the DELIVERED r."""
    xs = [p["r_del"] for p in points]
    ys = [p["rate"] for p in points]
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0:
        return None, None
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return slope, (mx - my / slope) if slope else None


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port")
    p.add_argument("--idx-a", type=int, default=0)
    p.add_argument("--idx-b", type=int, default=1)
    p.add_argument("--spin-rpm", type=float, default=800.0)
    p.add_argument("--r", type=float, nargs="+",
                   default=[-400, 200, -300, 100, -200, 0, -100],
                   help="offsets to visit. INTERLEAVED by default so the pod does not wind one way "
                        "for the whole sweep")
    p.add_argument("--pairs", type=float, nargs="+", default=None,
                   metavar="A B",
                   help="explicit (A_target B_target) pairs, flattened: `--pairs -1500 -900 -2000 "
                        "-900`. Overrides --r. Use this to find the BALANCE RATIO with both motors "
                        "clear of the seam.")
    p.add_argument("--repeat", type=int, default=1,
                   help="measure every point this many times. The scatter IS the result: on "
                        "2026-08-15 one operating point read 2507 and 5014 deg/s in two sweeps and "
                        "a hardware conclusion was drawn from the difference.")
    p.add_argument("--tele-n", type=int, default=6,
                   help="telemetry samples taken OUTSIDE the integration window")
    p.add_argument("--settle", type=float, default=1.2, help="seconds before the window opens")
    p.add_argument("--window", type=float, default=1.5, help="seconds integrated per point")
    p.add_argument("--slew", type=float, default=2000.0, help="rpm/s — high on purpose: this is a "
                                                              "STATIC measurement, the ramp is waste")
    p.add_argument("--arm-secs", type=float, default=4.5)
    p.add_argument("--name", default="rpmcal")
    args = p.parse_args()
    if args.pairs:
        if len(args.pairs) % 2:
            sys.exit("--pairs takes an even number of values: A B A B ...")
        args.pairs = list(zip(args.pairs[0::2], args.pairs[1::2]))

    d = EscHost(args.port)
    pts = []
    try:
        for i in (args.idx_a, args.idx_b):
            d.cmd(f"arm {i} bidir", timeout=10)
            d.cmd(f"gain {i} slew {args.slew}", timeout=5)
        time.sleep(args.arm_secs)
        # Come up MATCHED (r = 0) before the first point: starting a point straight from rest
        # measures the seam crossing on top of the plant, which is the mistake this replaces.
        d.cmd(f"rpm {args.idx_a} {-args.spin_rpm:.0f}", timeout=5)
        d.cmd(f"rpm {args.idx_b} {-args.spin_rpm:.0f}", timeout=5)
        time.sleep(2.0)
        print(f"{'r_cmd':>6s} {'r_del':>7s} {'rate':>9s} {'rpm_a':>7s} {'rpm_b':>7s} "
              f"{'a_tgt':>7s} {'b_tgt':>7s}")
        if args.pairs:
            print(f"{'ratio':>6s} {'rate':>9s} {'rpm_a':>7s} {'rpm_b':>7s} {'a_tgt':>7s} "
                  f"{'b_tgt':>7s} {'a/b meas':>9s} {'gap_ms':>7s} {'mxstep':>6s} {'margin':>6s}")
            for a_t, b_t in args.pairs:
                for _k in range(args.repeat):
                    pt = measure(d, args, 0.0, pair=(a_t, b_t))
                    pt["ratio"] = abs(a_t / b_t) if b_t else float("inf")
                    pts.append(pt)
                    meas = pt["rpm_a"] / pt["rpm_b"] if pt["rpm_b"] else float("nan")
                    print(f"{pt['ratio']:6.2f} {pt['rate']:9.0f} {pt['rpm_a']:7.0f} "
                          f"{pt['rpm_b']:7.0f} {a_t:7.0f} {b_t:7.0f} {meas:9.2f} "
                          f"{pt['maxgap_us'] / 1000.0:7.1f} {pt['maxstep']:6d} {pt['margin']:6.2f}")
        else:
            for r in args.r:
                pt = measure(d, args, r)
                pts.append(pt)
                flag = "" if abs(pt["r_del"] - r) < 40 else "   <== NOT DELIVERED"
                print(f"{pt['r']:6.0f} {pt['r_del']:7.0f} {pt['rate']:9.0f} {pt['rpm_a']:7.0f} "
                      f"{pt['rpm_b']:7.0f} {pt['a_tgt']:7.0f} {pt['b_tgt']:7.0f}{flag}")
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

    if args.pairs:
        # The balance ratio: where does the rate cross zero as |A|/|B| grows?
        srt = sorted(pts, key=lambda q: q["ratio"])
        cross = None
        for u, v in zip(srt, srt[1:]):
            if (u["rate"] <= 0) != (v["rate"] <= 0):
                fr = -u["rate"] / (v["rate"] - u["rate"])
                cross = u["ratio"] + fr * (v["ratio"] - u["ratio"])
        print(f"\n# azimuth nulls at |A|/|B| = {cross:.2f}" if cross else
              "\n# no zero crossing in this range - widen it")
        good = []
    else:
        good = [p for p in pts if abs(p["r_del"] - p["r"]) < 40]
    if len(good) >= 2:
        slope, null = fit(good)
        print(f"\n# fit over {len(good)}/{len(pts)} delivered points:")
        print(f"#   slope   {slope:8.3f} deg/s per rpm of offset")
        print(f"#   null at r = {null:7.1f} rpm")
        print(f"#   => rpm_per_count = 1/(slope*kff) with kff = 1/(rpm_per_count*slope): pick "
              f"rpm_per_count so the feedforward is exact")
        print(f"#   az gain rpmtrim {null:.0f}")
        # kff and rpm_per_count size together: the loop wants rate = vsp for a demand vsp, i.e.
        # slope * rpm_per_count * kff * vsp = vsp.
        print(f"#   with rpm_per_count 7.26 -> kff {1.0 / (7.26 * slope):.5f} "
              f"(currently {1.0 / (7.26 * 6.0):.5f})")

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "reports",
                       f"azrpmcal_{args.name}_{time.strftime('%Y%m%d-%H%M%S')}.csv")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        f.write(f"# spin_rpm={args.spin_rpm}|settle={args.settle}|window={args.window}|slew={args.slew}\n")
        f.write("r_cmd,r_delivered,rate_deg_s,rpm_a,rpm_b,a_tgt,b_tgt,dt\n")
        for q in pts:
            f.write(f"{q['r']:.0f},{q['r_del']:.1f},{q['rate']:.1f},{q['rpm_a']:.1f},"
                    f"{q['rpm_b']:.1f},{q['a_tgt']:.0f},{q['b_tgt']:.0f},{q['dt']:.3f}\n")
    print(f"# wrote {out}")


if __name__ == "__main__":
    main()
