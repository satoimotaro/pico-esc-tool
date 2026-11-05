#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""encwatch — is the AS5600 still a sensor while the two motors are spinning?

WHY THIS EXISTS. On 2026-08-15 a matched open-loop spin-up (`az trace --kp 0 --ki 0 --kd 0
--rpm-trim 0 --rpm-off-max 0`, i.e. BOTH motors given the identical speed target and the azimuth
loop given literally zero authority) aborted 3/3 on the rate guard at 3896-4231 deg/s, while the
two ESCs reported speeds within 2-12 rpm of each other. Under the differential's kinematics matched
speeds hold the azimuth still, so either the telemetry is lying or the encoder is. The trace records
neither magnet health nor sample count, so it cannot tell the two apart.

This does: it polls `enc` (raw angle + STATUS md/ml/mh + AGC + MAG) and `encv` (accumulator, velocity,
and the tracker's own SAMPLE COUNT) at rest, then through a spin, then at rest again. What each
column decides:
  - agc / mag / ml / mh moving with motor speed  -> the sensor is seeing the MOTORS' stray field,
    not just the target magnet, and the "azimuth motion" is partly an artefact.
  - samples/s collapsing during the spin         -> the tracker is being STARVED (I2C or core0 time),
    so a stale accumulator releases as one apparent lurch. Also an artefact.
  - both steady, accum genuinely displaced       -> the pod really is moving; the ESCs' matched
    telemetry is the thing that is wrong.
The three are mutually exclusive and the run costs one ordinary 1.2 s abort.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pico_esc.link import EscHost                                   # noqa: E402


def _fields(line, tag):
    """`tag|a|b|c` -> [a, b, c] as floats. None if this is not that reply."""
    parts = line.strip().split("|")
    if not parts or parts[0] != tag:
        return None
    out = []
    for p in parts[1:]:
        try:
            out.append(float(p))
        except ValueError:
            return None
    return out


def sample(d, idx_a, idx_b):
    """One (enc, encv, tele A, tele B) row. None if a read failed — never a partial row.

    The motor speeds are NOT optional here. Every azimuth conclusion on record that involved a motor
    speed turned out to need the DELIVERED one: on 2026-08-15 a solo-motor test read -430 deg/s for
    A and +2051 for B at the same commanded 800 rpm, which is either a 4.8x mechanical asymmetry or
    simply A failing to reach 800 against a braked B — and without tele the two are indistinguishable.
    """
    e = v = None
    for line in d.cmd("enc", timeout=3):
        e = _fields(line, "enc") or e
    for line in d.cmd("encv", timeout=3):
        v = _fields(line, "encv") or v
    if e is None or v is None:
        return None
    out = {"raw": e[0], "deg": e[2], "md": e[3], "ml": e[4], "mh": e[5], "agc": e[6], "mag": e[7],
           "accum": v[0], "rpm": v[1], "samples": v[2]}
    for key, i in (("rpm_a", idx_a), ("rpm_b", idx_b)):
        out[key] = float("nan")
        try:
            for line in d.cmd(f"tele {i}", timeout=3):
                f = _fields(line, "tele")
                if f:
                    out[key] = f[0]
        except Exception:
            pass
    return out


def watch(d, secs, label, rows, idx_a, idx_b):
    t0 = time.time()
    while time.time() - t0 < secs:
        s = sample(d, idx_a, idx_b)
        if s:
            s["t"] = time.time() - t0
            s["phase"] = label
            rows.append(s)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port")
    p.add_argument("--idx-a", type=int, default=0)
    p.add_argument("--idx-b", type=int, default=1)
    p.add_argument("--rpm", type=float, default=800.0,
                   help="matched speed target for BOTH motors, signed as the azimuth loop drives "
                        "them (negative: `applyRpm_` uses sgn = -1 for a negative carrier)")
    p.add_argument("--rpm-b", type=float, default=None,
                   help="motor B's target. Defaults to --rpm, i.e. what `applyRpm_` actually sends "
                        "(BOTH ESCs the same signed speed). Pass the OPPOSITE sign to test whether "
                        "B is really mounted mirrored — azimuth.h claims sign_b = -1 makes the pair "
                        "carry together, and the raw mixer applies it while the rpm path does not.")
    p.add_argument("--slew", type=float, default=300.0, help="rpm/s, matching spin_slew")
    p.add_argument("--secs", type=float, default=2.5, help="seconds of spin — keep it short, "
                                                           "there is no rate guard on this path")
    p.add_argument("--rest-secs", type=float, default=1.0, help="baseline before and after")
    p.add_argument("--arm-secs", type=float, default=4.5,
                   help="wait after `arm` before a target is accepted (`arming ~3s`)")
    p.add_argument("--name", default="encwatch")
    args = p.parse_args()

    d = EscHost(args.port)
    rows = []
    try:
        print("# baseline (motors idle)")
        watch(d, args.rest_secs, "rest", rows, args.idx_a, args.idx_b)
        for i in (args.idx_a, args.idx_b):
            d.cmd(f"arm {i} bidir", timeout=10)
            d.cmd(f"gain {i} slew {args.slew}", timeout=5)
        # `arm` replies "arming ~3s" and only THEN accepts a target — the same wait azimuth.py
        # spends in --spinup-secs. Sending `rpm` early just gets `err not-armed`.
        time.sleep(args.arm_secs)
        print(f"# spin: A -> {args.rpm:.0f} rpm, B -> "
              f"{args.rpm if args.rpm_b is None else args.rpm_b:.0f} rpm, "
              f"slew {args.slew:.0f} rpm/s, {args.secs:.1f}s")
        rb = args.rpm if args.rpm_b is None else args.rpm_b
        d.cmd(f"rpm {args.idx_a} {args.rpm}", timeout=5)
        d.cmd(f"rpm {args.idx_b} {rb}", timeout=5)
        watch(d, args.secs, "spin", rows, args.idx_a, args.idx_b)
    finally:
        # Stop in the order that leaves nothing spinning even if a step above raised.
        for _ in range(3):
            for i in (args.idx_a, args.idx_b):
                try:
                    d.cmd(f"rpm {i} 0", timeout=3)
                except Exception:
                    pass
                try:
                    d.cmd(f"thrust {i} 0", timeout=3)
                except Exception:
                    pass
        for i in (args.idx_a, args.idx_b):
            try:
                d.cmd(f"disarm {i}", timeout=3)
            except Exception:
                pass
    print("# settle (motors stopped)")
    watch(d, args.rest_secs, "after", rows, args.idx_a, args.idx_b)

    os.makedirs(os.path.join(os.path.dirname(os.path.abspath(__file__)), "reports"), exist_ok=True)
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "reports",
                        f"enc_{args.name}_{time.strftime('%Y%m%d-%H%M%S')}.csv")
    with open(path, "w") as f:
        f.write("phase,t_s,accum,deg,rpm,samples,md,ml,mh,agc,mag,rpm_a,rpm_b\n")
        for r in rows:
            f.write(f"{r['phase']},{r['t']:.3f},{r['accum']:.0f},{r['deg']:.1f},{r['rpm']:.1f},"
                    f"{r['samples']:.0f},{r['md']:.0f},{r['ml']:.0f},{r['mh']:.0f},"
                    f"{r['agc']:.0f},{r['mag']:.0f},{r['rpm_a']:.0f},{r['rpm_b']:.0f}\n")
    print(f"# wrote {path} ({len(rows)} samples)")

    # The comparison this exists for: per phase, what moved.
    print(f"\n{'phase':6s} {'n':>4s} {'samp/s':>7s} {'agc':>12s} {'mag':>12s} "
          f"{'md/ml/mh':>10s} {'net deg':>9s} {'rate deg/s':>10s} {'rpm_a':>6s} {'rpm_b':>6s}")
    for ph in ("rest", "spin", "after"):
        g = [r for r in rows if r["phase"] == ph]
        if len(g) < 2:
            continue
        dt = g[-1]["t"] - g[0]["t"]
        sps = (g[-1]["samples"] - g[0]["samples"]) / dt if dt > 0 else float("nan")
        ag = [r["agc"] for r in g]
        mg = [r["mag"] for r in g]
        acc = [r["accum"] * 360.0 / 4096.0 for r in g]
        st = f"{max(r['md'] for r in g):.0f}/{max(r['ml'] for r in g):.0f}/{max(r['mh'] for r in g):.0f}"
        # NET, not span: span is unsigned and cannot tell a pod that ran away one turn from one
        # that oscillated half a turn either side of where it started.
        net = acc[-1] - acc[0]
        ra = [r["rpm_a"] for r in g if r["rpm_a"] == r["rpm_a"]]
        rb = [r["rpm_b"] for r in g if r["rpm_b"] == r["rpm_b"]]
        print(f"{ph:6s} {len(g):4d} {sps:7.0f} {min(ag):5.0f}..{max(ag):-5.0f} "
              f"{min(mg):5.0f}..{max(mg):-5.0f} {st:>10s} {net:9.0f} {net / dt:10.0f} "
              f"{(sum(ra) / len(ra) if ra else float('nan')):6.0f} "
              f"{(sum(rb) / len(rb) if rb else float('nan')):6.0f}")


if __name__ == "__main__":
    main()
