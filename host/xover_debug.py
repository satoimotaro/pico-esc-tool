#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""xover_debug — ramp the ESC command up through the S3 sine<->6-step crossover and back,
to CHECK the connection and the handoff (both directions) on real hardware.

WHAT IT CHECKS
- 接続 (connection): the link stays alive the whole ramp (enc + tele respond, deadman fed).
- 切り替え (switching): the S3 crossover fires sine->6-step on the way up and 6-step->sine on
  the way down. The signal is the telemetry eRPM: it is STALE in sine (Comm_Period4x not
  updated) and LIVE in 6-step. So `tele_erpm/POLE_PAIRS ~= enc_rpm` => 6-step (handed off);
  divergent/zero => sine (forced). We log both and report where the handoff happened.

"thrust" here = the signed ESC COMMAND (-1000..1000), NOT physical force.

The ramp/measure loop itself lives in pico_esc.crossover.measure_crossover_lock (shared with
autocal's tune-crossover-lock phase): guarded unwrap + low-pass, keep-alive + deadman pacing,
volts>0 telemetry filtering, safety aborts. This CLI adds CSV logging + handoff-regime reporting.

SAFETY (this is the S3 firmware's FIRST hardware crossover test — be gentle):
- --rpm-ceiling caps the ACTUAL measured speed: the moment the encoder exceeds it we stop
  ramping and cut toward zero — this bounds the top speed regardless of regime (6-step can
  accelerate hard for a small command). Keep it LOW for the first run.
- --max-temp abort, stall abort, always-disarm on EVERY exit (incl SIGINT/SIGTERM), and it
  RESTORES sine_cross_up/dn = 0 on exit so the ESC is left in its normal (no-crossover) state.
- Start with a small --cmd-max and a low --rpm-ceiling; watch the motor.

  python xover_debug.py --dry-run                       # sim (models the crossover)
  python xover_debug.py --up-erpm 2200 --dn-erpm 1800 --rpm-ceiling 450 --cmd-max 900
"""
from __future__ import annotations

import argparse
import csv
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pico_esc import ESC, EscLink, RealClock, SimClock          # noqa: E402
from pico_esc.sim import SimEncEscHost                          # noqa: E402
from pico_esc.drive import Aborted                              # noqa: E402
from pico_esc.config import sine_crossover_bytes                # noqa: E402
from pico_esc.crossover import (POLE_PAIRS, measure_crossover_lock,  # noqa: E402
                                measure_crossover_lock_lowspeed)

REPORT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "reports")

SINE_SLOPE = 357.0 / 1000.0     # max mech RPM per command unit in sine (forced rate)
HANDOFF_MIN_CMD = 200           # only trust the regime heuristic above this |cmd| (low-speed
                                # sine creep otherwise produces spurious "handoff at cmd=15")


def _regime_of(enc_rpm, cmd, tele_rpm):
    # Robust, works on sim AND hardware: sine's forced rate CANNOT exceed cmd*SINE_SLOPE, so a
    # measured speed clearly above that line means the ESC handed off to 6-step. Below
    # HANDOFF_MIN_CMD the line is too close to the noise floor to trust, so hold "sine".
    if abs(cmd) < HANDOFF_MIN_CMD:
        return "sine"
    sine_max = abs(cmd) * SINE_SLOPE
    if abs(enc_rpm) > sine_max * 1.15 + 25.0:
        return "6step"
    return "sine"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--up-erpm", type=float, default=2200.0, help="Cross_Up threshold eRPM (default 2200)")
    ap.add_argument("--dn-erpm", type=float, default=1800.0, help="Cross_Dn threshold eRPM (default 1800)")
    ap.add_argument("--cmd-max", type=int, default=900, help="max ESC command reached on the ramp (default 900)")
    ap.add_argument("--rpm-ceiling", type=float, default=450.0,
                    help="ABORT-DOWN the ramp if measured speed exceeds this mech RPM (default 450)")
    ap.add_argument("--up-secs", type=float, default=8.0, help="ramp-up duration, s (default 8)")
    ap.add_argument("--hold-secs", type=float, default=1.5, help="hold at top, s (default 1.5)")
    ap.add_argument("--down-secs", type=float, default=8.0, help="ramp-down duration, s (default 8)")
    ap.add_argument("--lowspeed", action="store_true",
                    help="measure at a LOW encoder-reliable speed (descend into 6-step then hold) so "
                         "reverse (--sign -1) doesn't alias the 50Hz encoder at the top of the ramp")
    ap.add_argument("--measure-speed", type=float, default=700.0,
                    help="--lowspeed target mech RPM to descend to and measure at (default 700)")
    ap.add_argument("--descend-secs", type=float, default=8.0,
                    help="--lowspeed ramp-DOWN duration while finding the measure speed, s (default 8)")
    ap.add_argument("--max-temp", type=float, default=60.0, help="temp abort, C (0=off; default 60)")
    ap.add_argument("--sign", type=int, default=1, choices=(1, -1),
                    help="thrust direction to ramp: +1 forward (default), -1 reverse (bidirectional thruster)")
    ap.add_argument("--esc-index", type=int, default=1)
    ap.add_argument("--port", default=None)
    ap.add_argument("--csv", default=None)
    ap.add_argument("--dry-run", action="store_true", help="SimEncEscHost, no hardware")
    ap.add_argument("--sim-invert", action="store_true")
    ap.add_argument("--seed", type=int, default=1234)
    opts = ap.parse_args()

    try:
        cross_up, cross_dn = sine_crossover_bytes(opts.up_erpm, opts.dn_erpm)
    except ValueError as e:
        sys.exit(f"crossover thresholds rejected: {e}")
    up_eff = cross_up * (10000.0 / 256.0)
    dn_eff = 312500.0 / cross_dn
    print(f"# crossover: up=0x{cross_up:02X} (~{up_eff:.0f} eRPM, {up_eff/POLE_PAIRS:.0f} mech RPM), "
          f"dn=0x{cross_dn:02X} (~{dn_eff:.0f} eRPM, {dn_eff/POLE_PAIRS:.0f} mech RPM)")

    if opts.dry_run:
        print("# DRY-RUN: SimEncEscHost (models the crossover; no hardware)")
        clock = SimClock()
        host = SimEncEscHost(clock, seed=opts.seed, invert=opts.sim_invert)
    else:
        clock = RealClock()
        host = EscLink(opts.port)
    esc = ESC(host, opts.esc_index, tmax=1000, clock=clock)

    def _panic(*_):
        try:
            esc.disarm()
        finally:
            try:
                if not opts.dry_run:
                    esc.config.set(sine_cross_up=0, sine_cross_dn=0); esc.restart()
            finally:
                os._exit(1)
    signal.signal(signal.SIGINT, _panic)
    signal.signal(signal.SIGTERM, _panic)

    os.makedirs(REPORT_DIR, exist_ok=True)
    path = opts.csv or os.path.join(REPORT_DIR, f"xover_{time.strftime('%Y%m%d-%H%M%S')}.csv")
    fh = open(path, "w", encoding="utf-8", newline="")
    w = csv.writer(fh); w.writerow(["t", "cmd", "enc_rpm", "tele_erpm", "regime", "temp"])

    # handoff-regime reporting state, driven from the shared measurement loop's per-tick callback.
    handoff = {"prev": "sine", "up": None, "down": None}

    def on_sample(t, cmd, enc_rpm, tele_erpm, phase, peak_temp):
        reg = _regime_of(enc_rpm, cmd, tele_erpm)
        if reg != handoff["prev"]:
            if reg == "6step" and handoff["up"] is None:
                handoff["up"] = (cmd, round(enc_rpm, 1), round(tele_erpm))
                print(f"#   >>> UP handoff sine->6step at cmd={cmd} enc={enc_rpm:.0f}RPM tele={tele_erpm:.0f}eRPM")
            elif reg == "sine" and handoff["up"] is not None and handoff["down"] is None:
                handoff["down"] = (cmd, round(enc_rpm, 1), round(tele_erpm))
                print(f"#   <<< DOWN handoff 6step->sine at cmd={cmd} enc={enc_rpm:.0f}RPM")
            handoff["prev"] = reg
        w.writerow([f"{t:.3f}", cmd, f"{enc_rpm:.1f}", f"{tele_erpm:.0f}", reg,
                    "" if peak_temp is None else peak_temp])

    failure = None
    result = None
    try:
        if not opts.dry_run:
            esc.config.set(sine_cross_up=cross_up, sine_cross_dn=cross_dn); esc.restart()
        else:
            esc.config.set(sine_cross_up=cross_up, sine_cross_dn=cross_dn)   # into the sim
        esc.prepare(); esc.arm(bidir=True)
        print("# armed; ramping up through the crossover…"
              + (f" (lowspeed: measure at ~{opts.measure_speed:.0f} mech RPM)" if opts.lowspeed else ""))
        if opts.lowspeed:
            result = measure_crossover_lock_lowspeed(
                esc, clock, target_cmd=opts.cmd_max, sign=opts.sign,
                ramp_secs=opts.up_secs, descend_secs=opts.descend_secs, hold_secs=opts.hold_secs,
                measure_rpm=opts.measure_speed, rpm_ceiling=opts.rpm_ceiling,
                max_temp=opts.max_temp, on_sample=on_sample)
        else:
            result = measure_crossover_lock(
                esc, clock, target_cmd=opts.cmd_max, sign=opts.sign,
                ramp_secs=opts.up_secs, hold_secs=opts.hold_secs, down_secs=opts.down_secs,
                rpm_ceiling=opts.rpm_ceiling, max_temp=opts.max_temp, on_sample=on_sample)
        if result.aborted:
            print(f"#   note: {result.aborted}")
        if result.ceiling_hit:
            print(f"#   ceiling {opts.rpm_ceiling:.0f}RPM reached at cmd={result.top_cmd} "
                  f"— held briefly then ramped down")
    except Aborted as e:
        failure = str(e)
    except Exception as e:                                       # noqa: BLE001
        failure = f"error: {e}"
    finally:
        esc.disarm()
        try:
            if not opts.dry_run:
                esc.config.set(sine_cross_up=0, sine_cross_dn=0); esc.restart()
                print("# crossover restored to 0 (normal mode)")
        finally:
            fh.close(); host.close()

    peak_temp = result.peak_temp if result else None
    slip = result.slip if result else None
    print(f"# wrote CSV: {path}")
    print(f"# peak temp: {peak_temp}C")
    if result is not None:
        print(f"# steady: enc={result.enc_rpm:.0f} mechRPM  tele={result.tele_erpm:.0f} eRPM  "
              f"slip={'%.3f' % slip if slip is not None else 'n/a'}  reversed={result.reversed}")
    print(f"# UP handoff (sine->6step): {handoff['up'] if handoff['up'] else 'NOT observed'}")
    print(f"# DOWN handoff (6step->sine): {handoff['down'] if handoff['down'] else 'NOT observed'}")
    ok = handoff["up"] is not None and handoff["down"] is not None
    print(f"# RESULT: crossover {'OK both directions' if ok else 'INCOMPLETE — see log'}"
          + (f"  [ABORTED: {failure}]" if failure else ""))
    if failure:
        sys.exit(f"XOVER ABORTED: {failure}")


if __name__ == "__main__":
    main()
