#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""velcal — calibrate a per-motor ESC-command -> speed curve for sensorless velocity control.

Sweeps the SIGNED ESC COMMAND ("thrust", -1000..+1000 — an ESC drive command, NOT a physical
force) from ~0 up to --max-thrust, and at each level measures the STEADY mechanical RPM with
the AS5600 encoder (settle, then average). The result is one monotonic SpeedProfile YAML that
velctl inverts at runtime to run SENSORLESSLY (encoder needed ONLY here, for this one
calibration). With --crossover-erpm the S3 sine<->BEMF crossover is ENABLED for the sweep so
the ONE curve spans the whole range and bakes in the seam / handoff jump — velctl then needs
no regime knowledge.

  python velcal.py --dry-run --crossover-erpm 2100,1600 --seed 1234 --profile-out /tmp/p.yaml
  python velcal.py --crossover-erpm 2100,1600 --profile-out profiles/vel_mymotor.yaml   # bench

Crossover bytes are written ONCE via esc.config.set (editpage — the only EEPROM writer) then
esc.restart(); an out-of-band split is REPORTED (config.sine_crossover_bytes rejects), never
silently clamped. Mirrors tune_sine_amp.py: writes go through the ESC config facade, temp is
guarded (no current sense at hold), and it ALWAYS disarms (incl. SIGINT/SIGTERM).

WARNING: the S3 crossover firmware is BENCH-UNTESTED. A real `velcal --crossover` run is the
firmware's FIRST hardware crossover test — supervise it. --dry-run exercises a MODEL, not HW.
"""
from __future__ import annotations

import argparse
import os
import signal
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import esctool  # noqa: E402,F401
from esctool import EscHost  # noqa: E402

from pico_esc.esc import ESC  # noqa: E402
from pico_esc.link import RealClock, SimClock  # noqa: E402
from pico_esc.sim import SimEncEscHost  # noqa: E402
from pico_esc.drive import Aborted  # noqa: E402
from pico_esc.config import sine_crossover_bytes, SINE_CROSS_UP_ERPM_PER_UNIT  # noqa: E402
from pico_esc.constants import FULLSCALE_RPM  # noqa: E402
from pico_esc.control import calibrate_direction  # noqa: E402
from pico_esc.velocity import SpeedProfile, measure_steady_speed, POLE_PAIRS  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PROFILE_DIR = os.path.join(HERE, "profiles")


def _dir_opts(opts):
    """Minimal namespace of the fields calibrate_direction reads."""
    return argparse.Namespace(
        tmin=opts.min_thrust, tmax=opts.max_thrust,
        probe_secs=opts.probe_secs, probe_min_deg=opts.probe_min_deg,
        probe_confirm_deg=opts.probe_confirm_deg)


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--crossover-erpm", default=None,
                    help="enable the S3 crossover for the sweep: 'up,dn' eRPM (e.g. 2100,1600). "
                         "Omit to calibrate a single-regime (sine-only) curve.")
    ap.add_argument("--min-thrust", type=int, default=50,
                    help="lowest swept ESC command (default 50; below breakaway is just 0 RPM)")
    ap.add_argument("--max-thrust", type=int, default=1000,
                    help="highest swept ESC command, 0..1000 (default 1000)")
    ap.add_argument("--points", type=int, default=12, help="sweep points min..max (default 12)")
    ap.add_argument("--settle-secs", type=float, default=1.5, help="spin-up before measuring")
    ap.add_argument("--measure-secs", type=float, default=2.0, help="averaging window")
    ap.add_argument("--max-temp", type=float, default=80.0, help="abort at this C (0=off)")
    ap.add_argument("--motor", default="930kv_12n14p", help="motor name stored in the profile")
    ap.add_argument("--esc-index", type=int, default=1)
    ap.add_argument("--port", default=None)
    ap.add_argument("--invert-encoder", action="store_true", help="known wiring: enc_sign=-1")
    ap.add_argument("--no-autocal", action="store_true", help="skip the direction probe (enc_sign=+1)")
    ap.add_argument("--probe-secs", type=float, default=0.5, help="direction-cal probe window, s")
    ap.add_argument("--probe-min-deg", type=float, default=3.0, help="min break-away travel, deg")
    ap.add_argument("--probe-confirm-deg", type=float, default=30.0,
                    help="clear arc required before trusting the direction sign, deg")
    ap.add_argument("--dry-run", action="store_true", help="SimEncEscHost; no writes, no port")
    ap.add_argument("--sim-invert", action="store_true", help="(dry-run) inverted wiring model")
    ap.add_argument("--seed", type=int, default=1234)
    ap.add_argument("--profile-out", required=True, help="YAML profile output path")
    return ap


def main():
    opts = build_parser().parse_args()
    if not 0 < opts.max_thrust <= 1000:
        sys.exit("--max-thrust must be in 1..1000")
    if opts.min_thrust <= 0 or opts.min_thrust >= opts.max_thrust:
        sys.exit("--min-thrust must be > 0 and < --max-thrust")
    if opts.points < 2:
        sys.exit("--points must be >= 2")

    # Resolve the crossover split into bytes FIRST — reject an out-of-band split, don't clamp.
    crossover_meta = None
    cu = cd = None
    if opts.crossover_erpm:
        try:
            up_s, dn_s = opts.crossover_erpm.split(",")
            up_erpm, dn_erpm = float(up_s), float(dn_s)
        except ValueError:
            sys.exit("--crossover-erpm wants 'up,dn' eRPM, e.g. 2100,1600")
        try:
            cu, cd = sine_crossover_bytes(up_erpm, dn_erpm)
        except ValueError as e:
            sys.exit(f"crossover split rejected (not clamped): {e}")
        crossover_meta = {"up_erpm": up_erpm, "dn_erpm": dn_erpm, "bytes": [cu, cd]}
        print(f"# crossover: up={up_erpm:g} dn={dn_erpm:g} eRPM -> "
              f"sine_cross_up={cu} sine_cross_dn={cd}")

    if opts.dry_run:
        print("# DRY-RUN: SimEncEscHost (no serial port opened, no EEPROM written)")
        clock = SimClock()
        host = SimEncEscHost(clock, seed=opts.seed, invert=opts.sim_invert)
    else:
        clock = RealClock()
        host = EscHost(opts.port)
    esc = ESC(host, opts.esc_index, tmax=opts.max_thrust, clock=clock)
    enc_sign = -1 if (opts.invert_encoder or opts.sim_invert) else 1

    def _panic(*_):
        esc.disarm()
        try:
            host.close()
        finally:
            os._exit(1)
    signal.signal(signal.SIGINT, _panic)
    signal.signal(signal.SIGTERM, _panic)

    # Dedupe the swept levels (a narrow min..max over many --points can collapse integer
    # thrusts into duplicates, which would break the strictly-increasing-thrust profile).
    thrusts = sorted({round(opts.min_thrust + (opts.max_thrust - opts.min_thrust) * i
                            / (opts.points - 1)) for i in range(opts.points)})
    # Effective Cross_Up eRPM from the rounded byte (what the sweep believes the seam is).
    up_erpm_eff = cu * SINE_CROSS_UP_ERPM_PER_UNIT if cu is not None else None
    points = [(0.0, 0.0)]
    regimes = ["sine"] if cu is not None else None   # velcal's believed regime per point
    failure = None
    try:
        # Write the crossover ONCE (editpage) and restart the app so the sweep runs with it.
        if cu is not None:
            esc.config.set(sine_cross_up=cu, sine_cross_dn=cd)
            esc.restart()
        esc.prepare()
        esc.arm()                                  # bidir
        # Direction: probe unless told the wiring (mirrors posctl auto-cal).
        if not (opts.invert_encoder or opts.no_autocal):
            enc0 = esc.encoder()
            base = enc0.raw if enc0 is not None else 0
            enc_sign = calibrate_direction(esc.drive, clock, _dir_opts(opts), base)
        else:
            print(f"# direction: enc_sign={enc_sign} (from flag, no probe)")

        print(f"# sweep {opts.points} points, thrust {opts.min_thrust}..{opts.max_thrust}")
        prev_rpm = 0.0
        for thr in thrusts:
            mean_rpm, ripple, tp = measure_steady_speed(
                esc, clock, thr, enc_sign, opts.settle_secs, opts.measure_secs, opts.max_temp)
            rpm = round(abs(mean_rpm), 2)
            # Enforce a monotonic (non-decreasing) curve so the runtime inverse is unambiguous;
            # the runtime loader strict-REJECTS non-monotonic, so clamp+warn here instead.
            if rpm < prev_rpm:
                print(f"#   [warn] thr={thr} rpm={rpm:.1f} < previous {prev_rpm:.1f} "
                      f"(noise/seam) -> clamped to keep the curve monotonic")
                rpm = prev_rpm
            prev_rpm = rpm
            # Believed regime: which side of Cross_Up the COMMANDED eRPM sits (auditable seam;
            # makes a mislabelled/clamped non-monotonic seam reading visible in the profile).
            believed = None
            if up_erpm_eff is not None:
                cmd_erpm = abs(thr) * FULLSCALE_RPM / 1000.0 * POLE_PAIRS
                believed = "line" if cmd_erpm > up_erpm_eff else "sine"
                regimes.append(believed)
            rtag = f"  regime={believed}" if believed else ""
            tstr = f" temp={tp}C" if tp is not None else ""
            print(f"#   thr={thr:4d}  rpm={rpm:8.1f}  eRPM={rpm * POLE_PAIRS:8.0f}"
                  f"  ripple={ripple:6.1f}{rtag}{tstr}")
            points.append((float(thr), rpm))
        esc.thrust(0)                              # ramp down handled by measure's final thrust 0
    except Aborted as e:
        failure = str(e)
    except Exception as e:                         # never leak a raw traceback mid-drive
        failure = f"error: {e}"
    finally:
        was_armed = esc.drive.armed
        esc.disarm()                               # ALWAYS disarm
        if was_armed:
            print("# DISARMED")
        host.close()

    if failure:
        sys.exit(f"VELCAL ABORTED: {failure}")

    # Build + save AFTER the always-disarm block. A degenerate sweep (too few distinct thrust
    # levels / a non-invertible curve) makes SpeedProfile raise ValueError; the motor is
    # already safely disarmed, so report cleanly instead of leaking a raw traceback.
    try:
        profile = SpeedProfile(points, motor=opts.motor, pole_pairs=POLE_PAIRS,
                               source=("sim" if opts.dry_run else "bench"),
                               crossover=crossover_meta, regimes=regimes)
        header = None
        if opts.dry_run:
            header = ("SIM-DERIVED — replace with a bench velcal run.\n"
                      "Generated by velcal.py --dry-run (a MODEL of the ESC, not HW truth).")
        profile.save(opts.profile_out, header=header)
    except ValueError as e:
        sys.exit(f"VELCAL ABORTED: {e}")
    print(f"# wrote profile: {opts.profile_out}  ({len(points)} points, "
          f"max {profile.max_rpm:.0f} RPM)")


if __name__ == "__main__":
    main()
