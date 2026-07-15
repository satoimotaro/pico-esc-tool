#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""tune_sine_amp — objective auto-tuner for BlueGill S2/S1 sine amplitude params.

Sweeps sine_hold_amp x sine_amp_max, drives the motor at a few constant low speeds, and
scores each combo by the SMOOTHNESS of the measured rotation (velocity ripple from the
AS5600 encoder), rejecting combos that SLIP (mean speed far below commanded) or overheat.
Prints a ranked table and the recommended params. Writes each combo to the ESC once, so a
full sweep costs O(hold_amps x amp_maxs) EEPROM writes (default 4x3 = 12) — bounded.

The encoder is needed ONLY for this bench characterization; once the amplitude params are
chosen and stored they run open-loop (sensorless) in the field. This is exactly the
"feed-forward, calibrate-with-encoder" split.

  python tune_sine_amp.py --dry-run                 # exercise the mechanics (no hardware)
  python tune_sine_amp.py --invert-encoder          # real bench, known direction
  python tune_sine_amp.py --hold-amps 10,14,18,22 --amp-maxs 30,45,60 \
      --thrusts 120,220 --measure-secs 3 --max-temp 70

Writes go through esctool (the only safe EEPROM writer); flashing/other config is untouched.
Always disarms. Requires the ESC on sine mode (Pgm_Sine_Mode 1 or 2) + Bidirectional.
"""
from __future__ import annotations

import argparse
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import posctl  # noqa: E402  (reuse PosDrive / EscHost / SimEncEscHost / constants)
from posctl import (PosDrive, COUNTS_PER_REV, DT, FULLSCALE_RPM, VEL_LP_ALPHA,  # noqa: E402
                    DELTA_FAULT_FRAC, Aborted, _pace)
import esctool  # noqa: E402


# commanded mech deg/s for a given |thrust| (S1/S2 stepper full scale)
def _commanded_vel(thrust):
    return abs(thrust) / 1000.0 * FULLSCALE_RPM * 6.0


def _drive_measure(drive, clock, thrust, enc_sign, settle_secs, measure_secs, max_temp):
    """Command a constant thrust; after a settle window, sample encoder velocity and return
    (mean_vel, ripple_std, peak_temp).  ripple_std is the std-dev of the low-passed velocity
    over the measure window — the smoothness metric. Raises Aborted on over-temp / stall."""
    prev_raw = None
    vel = 0.0
    samples = []
    peak_temp = None
    t_end = clock.now() + settle_secs + measure_secs
    t_measure = clock.now() + settle_secs
    next_tele = clock.now()
    while clock.now() < t_end:
        tick = clock.now()
        enc = drive.read_enc()
        if enc is not None and enc.healthy:
            if prev_raw is None:
                prev_raw = enc.raw
            else:
                d = ((enc.raw - prev_raw + COUNTS_PER_REV // 2) % COUNTS_PER_REV) - COUNTS_PER_REV // 2
                prev_raw = enc.raw
                if abs(d) <= DELTA_FAULT_FRAC * (COUNTS_PER_REV // 2):
                    inst = enc_sign * d * 360.0 / COUNTS_PER_REV / DT
                    vel += VEL_LP_ALPHA * (inst - vel)
                    if tick >= t_measure:
                        samples.append(vel)
        drive.send_thrust(thrust)
        if max_temp and tick >= next_tele:
            next_tele = tick + 0.5
            tp = drive.read_temp()
            if tp is not None:
                peak_temp = tp if peak_temp is None else max(peak_temp, tp)
                if tp >= max_temp:
                    drive.send_thrust(0)
                    raise Aborted(f"over-temperature {tp}C >= {max_temp:.0f}C")
        _pace(clock, tick)
    drive.send_thrust(0)
    if len(samples) < 5:
        return 0.0, float("inf"), peak_temp
    mean_v = statistics.mean(samples)
    ripple = statistics.pstdev(samples)
    return mean_v, ripple, peak_temp


def _set_amps(opts, hold_amp, amp_max):
    """Write the two amplitude params via esctool (real hardware only). O(1) writes/combo."""
    if opts.dry_run:
        return
    dev = esctool.EscHost(opts.port)
    try:
        dev.cmd("run", timeout=5)
        dev.cmd("disconnect", timeout=5)
        time.sleep(0.3)
        # esctool.cmd_set-style single write of both fields
        argns = argparse.Namespace(index=str(opts.esc_index),
                                   assign=[f"sine_hold_amp={hold_amp}", f"sine_amp_max={amp_max}"])
        argns.sine_crossover_erpm = None
        esctool.cmd_set(dev, argns)
    finally:
        dev.close()
        time.sleep(0.3)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hold-amps", default="10,14,18,22", help="sine_hold_amp values to try")
    ap.add_argument("--amp-maxs", default="30,45,60", help="sine_amp_max values to try")
    ap.add_argument("--thrusts", default="120,220", help="constant test thrusts (low speeds)")
    ap.add_argument("--settle-secs", type=float, default=1.5, help="spin-up before measuring")
    ap.add_argument("--measure-secs", type=float, default=3.0, help="velocity-ripple window")
    ap.add_argument("--max-temp", type=float, default=70.0, help="abort a combo at this C (0=off)")
    ap.add_argument("--slip-min", type=float, default=0.7,
                    help="reject if mean speed < this fraction of commanded (slip/stall)")
    ap.add_argument("--esc-index", type=int, default=1)
    ap.add_argument("--port", default=None)
    ap.add_argument("--invert-encoder", action="store_true", help="known wiring: enc_sign=-1")
    ap.add_argument("--dry-run", action="store_true", help="SimEncEscHost; no writes, no port")
    ap.add_argument("--sim-invert", action="store_true", help="(dry-run) inverted wiring model")
    ap.add_argument("--seed", type=int, default=1234)
    opts = ap.parse_args()

    hold_amps = [int(x) for x in opts.hold_amps.split(",")]
    amp_maxs = [int(x) for x in opts.amp_maxs.split(",")]
    thrusts = [int(x) for x in opts.thrusts.split(",")]
    enc_sign = -1 if (opts.invert_encoder or opts.sim_invert) else 1
    n_writes = len(hold_amps) * len(amp_maxs)
    print(f"# sweep {len(hold_amps)}x{len(amp_maxs)} = {n_writes} combos "
          f"({n_writes} ESC writes), {len(thrusts)} speeds each")

    results = []
    for hold_amp in hold_amps:
        for amp_max in amp_maxs:
            if amp_max < hold_amp:
                continue                                  # amp_max is a ceiling on hold+Inc
            try:
                _set_amps(opts, hold_amp, amp_max)
            except Exception as e:
                print(f"# hold={hold_amp} amax={amp_max}: WRITE FAILED ({e}); skipped")
                continue
            clock = posctl.SimClock() if opts.dry_run else posctl.RealClock()
            host = (posctl.SimEncEscHost(clock, seed=opts.seed, invert=opts.sim_invert)
                    if opts.dry_run else esctool.EscHost(opts.port))
            drive = PosDrive(host, opts.esc_index, 1000, clock, verbose=False)
            ripples, slips, peak = [], [], None
            aborted = None
            try:
                drive.prepare()
                drive.arm()
                for thr in thrusts:
                    mean_v, ripple, tp = _drive_measure(
                        drive, clock, thr, enc_sign, opts.settle_secs,
                        opts.measure_secs, opts.max_temp)
                    cmd_v = _commanded_vel(thr)
                    slip = (mean_v / cmd_v) if cmd_v else 0.0
                    # normalize ripple by commanded speed so speeds compare fairly
                    ripples.append(ripple / cmd_v if cmd_v else float("inf"))
                    slips.append(slip)
                    if tp is not None:
                        peak = tp if peak is None else max(peak, tp)
            except Aborted as e:
                aborted = str(e)
            finally:
                drive.disarm()
                host.close()
            if aborted:
                print(f"# hold={hold_amp:2} amax={amp_max:2}: ABORTED ({aborted})")
                continue
            rel_ripple = statistics.mean(ripples) if ripples else float("inf")
            min_slip = min(slips) if slips else 0.0
            ok = min_slip >= opts.slip_min
            tag = "" if ok else f"  SLIP(min {min_slip:.2f})"
            tstr = f"{peak}C" if peak is not None else "n/a"
            print(f"# hold={hold_amp:2} amax={amp_max:2}: rel_ripple={rel_ripple:6.3f} "
                  f"slip_min={min_slip:.2f} temp={tstr}{tag}")
            if ok:
                results.append((rel_ripple, hold_amp, amp_max, min_slip, peak))

    results.sort()
    print("\n# ranked (smoothest first):")
    print(f"#  {'hold':>4} {'amax':>4} {'rel_ripple':>10} {'slip_min':>8} {'peakT':>5}")
    for rr, ha, am, sl, pk in results[:10]:
        print(f"#  {ha:4} {am:4} {rr:10.3f} {sl:8.2f} {('%dC' % pk) if pk is not None else '  n/a':>5}")
    if results:
        _, ha, am, *_ = results[0]
        print(f"\n# recommended: sine_hold_amp={ha} sine_amp_max={am}")
        print(f"#   apply:  python esctool.py set {opts.esc_index} sine_hold_amp={ha} sine_amp_max={am}")
    else:
        print("\n# no combo passed the slip/temp gate — widen --amp-maxs or lower --slip-min")


if __name__ == "__main__":
    main()
