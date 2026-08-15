#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""encmon — live encoder readout, for checking the premise by hand with no power anywhere.

Every azimuth conclusion rests on one thing: `enc` reads the AZIMUTH AXIS. That has been verified by
hand before and is on record, but three instruments failed inside one session on 2026-08-15
(telemetry, the trace-length completion test, a camera burst sampled below the motion it was
watching), so it is worth ten seconds to re-establish rather than assume.

Nothing is armed and nothing spins. Turn the axis by hand and read the total off the screen.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pico_esc.link import EscHost                                   # noqa: E402


def read(d):
    acc = rpm = None
    for line in d.cmd("encv", timeout=3):
        p = line.strip().split("|")
        if p[0] == "encv":
            acc, rpm = int(p[1]), float(p[2])
    raw = agc = mag = None
    for line in d.cmd("enc", timeout=3):
        p = line.strip().split("|")
        if p[0] == "enc":
            raw, agc, mag = int(p[1]), int(p[7]), int(p[8])
    return acc, rpm, raw, agc, mag


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port")
    p.add_argument("--secs", type=float, default=25.0)
    args = p.parse_args()

    d = EscHost(args.port)
    a0, _, _, _, _ = read(d)
    print(f"# zeroed at accum {a0}. TURN THE AZIMUTH AXIS BY HAND — one full turn, then back.")
    print(f"#   {'t':>5s} {'total deg':>10s} {'turns':>7s} {'raw':>5s} {'agc':>4s} {'mag':>5s}")
    t0 = time.time()
    last = 0.0
    while time.time() - t0 < args.secs:
        acc, rpm, raw, agc, mag = read(d)
        deg = (acc - a0) * 360.0 / 4096.0
        t = time.time() - t0
        if t - last >= 0.4:
            last = t
            print(f"#   {t:5.1f} {deg:10.1f} {deg / 360.0:7.2f} {raw:5d} {agc:4d} {mag:5d}")
    acc, _, _, _, _ = read(d)
    deg = (acc - a0) * 360.0 / 4096.0
    print(f"\n# FINAL: {deg:+.1f} deg = {deg / 360.0:+.2f} turns")
    print("# If a hand turn of the AZIMUTH axis moved this, the sensor is where the docs say it is.")


if __name__ == "__main__":
    main()
