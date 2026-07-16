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
import math
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pico_esc import ESC, EscLink, RealClock, SimClock          # noqa: E402
from pico_esc.sim import SimEncEscHost                          # noqa: E402
from pico_esc.drive import Aborted                              # noqa: E402
from pico_esc.control import _pace, DT, DELTA_FAULT_FRAC, VEL_LP_ALPHA  # noqa: E402
from pico_esc.constants import COUNTS_PER_REV                   # noqa: E402
from pico_esc.config import sine_crossover_bytes                # noqa: E402

POLE_PAIRS = 7
REPORT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "reports")


def _unwrap_rpm(prev_raw, raw, dt, vel):
    """Return (new_vel_rpm, new_prev_raw). Guarded modulo unwrap + low-pass, like posctl."""
    if prev_raw is None:
        return vel, raw
    d = ((raw - prev_raw + COUNTS_PER_REV // 2) % COUNTS_PER_REV) - COUNTS_PER_REV // 2
    if abs(d) > DELTA_FAULT_FRAC * (COUNTS_PER_REV // 2) or dt <= 0:
        return vel, raw                                          # implausible / stalled tick: hold
    inst = (d / COUNTS_PER_REV) / dt * 60.0                      # mech RPM (signed)
    return vel + VEL_LP_ALPHA * (inst - vel), raw


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
    ap.add_argument("--max-temp", type=float, default=60.0, help="temp abort, C (0=off; default 60)")
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

    up_handoff = down_handoff = None
    peak_temp = None
    prev_regime = "sine"
    failure = None
    SINE_SLOPE = 357.0 / 1000.0     # max mech RPM per command unit in sine (forced rate)

    def regime_of(enc_rpm, cmd, tele_rpm):
        # Robust, works on sim AND hardware: sine's forced rate CANNOT exceed cmd*SINE_SLOPE,
        # so a measured speed clearly above that line means the ESC handed off to 6-step.
        # (On hardware the telemetry rpm also goes from stale->live-and-matching; logged too.)
        sine_max = abs(cmd) * SINE_SLOPE
        if abs(enc_rpm) > sine_max * 1.15 + 25.0:
            return "6step"
        return "sine"

    try:
        if not opts.dry_run:
            esc.config.set(sine_cross_up=cross_up, sine_cross_dn=cross_dn); esc.restart()
        else:
            esc.config.set(sine_cross_up=cross_up, sine_cross_dn=cross_dn)   # into the sim
        esc.prepare(); esc.arm(bidir=True)
        print("# armed; ramping up through the crossover…")

        prev_raw = None; vel = 0.0; last_t = clock.now()
        t0 = clock.now(); next_tele = t0; last_erpm = 0.0
        phase = "up"; ceiling_hit = False; stall_ticks = 0

        def tick(cmd):
            nonlocal prev_raw, vel, last_t, next_tele, last_erpm, peak_temp, prev_regime
            nonlocal up_handoff, down_handoff, stall_ticks
            ts = clock.now()
            enc = esc.encoder()
            now = clock.now(); dt = now - last_t; last_t = now
            if enc is not None and enc.healthy:
                vel, prev_raw = _unwrap_rpm(prev_raw, enc.raw, dt, vel)
            esc.thrust(cmd)                                       # keep-alive (deadman fed)
            if ts >= next_tele:
                next_tele = ts + 0.2
                tel = esc.telemetry()
                if tel is not None:
                    last_erpm = tel.rpm
                    if tel.temp is not None:
                        peak_temp = tel.temp if peak_temp is None else max(peak_temp, tel.temp)
                        if opts.max_temp and tel.temp >= opts.max_temp:
                            raise Aborted(f"over-temp {tel.temp}C")
            if abs(vel) > opts.rpm_ceiling * 1.5:                # hard safety net
                raise Aborted(f"over-speed {vel:.0f}RPM > 1.5x ceiling")
            if abs(cmd) >= 200 and abs(vel) < 15.0:              # powered but not turning = desync/stall
                stall_ticks += 1
                if stall_ticks >= 30:                           # ~0.6s
                    raise Aborted(f"stall/desync: cmd={cmd} but ~0 RPM for {stall_ticks} ticks")
            else:
                stall_ticks = 0
            reg = regime_of(vel, cmd, last_erpm)
            if reg != prev_regime:
                if reg == "6step" and up_handoff is None:
                    up_handoff = (cmd, round(vel, 1), round(last_erpm))
                    print(f"#   >>> UP handoff sine->6step at cmd={cmd} enc={vel:.0f}RPM tele={last_erpm:.0f}eRPM")
                elif reg == "sine" and up_handoff is not None and down_handoff is None:
                    down_handoff = (cmd, round(vel, 1), round(last_erpm))
                    print(f"#   <<< DOWN handoff 6step->sine at cmd={cmd} enc={vel:.0f}RPM")
                prev_regime = reg
            w.writerow([f"{ts - t0:.3f}", cmd, f"{vel:.1f}", f"{last_erpm:.0f}", reg,
                        "" if peak_temp is None else peak_temp])
            _pace(clock, ts)
            return vel

        # ramp up (stop early if the measured-speed ceiling is hit)
        n = max(1, round(opts.up_secs / DT))
        for i in range(n):
            cmd = round(opts.cmd_max * (i + 1) / n)
            v = tick(cmd)
            if abs(v) >= opts.rpm_ceiling:
                ceiling_hit = True
                print(f"#   ceiling {opts.rpm_ceiling:.0f}RPM reached at cmd={cmd} — starting ramp-down now")
                top_cmd = cmd; break
        else:
            top_cmd = opts.cmd_max
        # hold at the top ONLY if we didn't hit the speed ceiling (else go straight down)
        if not ceiling_hit:
            for _ in range(max(1, round(opts.hold_secs / DT))):
                tick(top_cmd)
        # ramp down through the reverse handoff
        n = max(1, round(opts.down_secs / DT))
        for i in range(n):
            tick(round(top_cmd * (n - i - 1) / n))
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

    print(f"# wrote CSV: {path}")
    print(f"# peak temp: {peak_temp}C")
    print(f"# UP handoff (sine->6step): {up_handoff if up_handoff else 'NOT observed'}")
    print(f"# DOWN handoff (6step->sine): {down_handoff if down_handoff else 'NOT observed'}")
    ok = up_handoff is not None and down_handoff is not None
    print(f"# RESULT: crossover {'OK both directions' if ok else 'INCOMPLETE — see log'}"
          + (f"  [ABORTED: {failure}]" if failure else ""))
    if failure:
        sys.exit(f"XOVER ABORTED: {failure}")


if __name__ == "__main__":
    main()
