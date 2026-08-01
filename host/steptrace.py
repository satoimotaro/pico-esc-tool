#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""steptrace — command a step and watch the rotor on the encoder, at full poll rate.

WHY. Two of this project's known-but-unmeasured behaviours are transients, and telemetry is blind
during exactly the window that matters:

  * THE STARTUP OVERSHOOT. Spinning up from rest overshoots roughly 2x before settling. `tele` is not
    live during startup — the ESC has not achieved BEMF lock yet and reports a forced-commutation
    intention — so the overshoot has only ever been seen through a broken sensor, and its real peak,
    time-to-peak and settling time are unknown.
  * THE DOWN HANDOFF. Stepping down through the crossover seam is the historically fragile direction:
    with an active down-handoff (`sine_cross_dn` below 0xF0) the motor has been seen to stall to zero
    at ~300-324 rpm. A slow staircase does NOT reproduce it — `sinesweep --updown` walks down through
    the seam cleanly — so the failure, if it is still there, lives in the STEP.

The distinction between those two matters and is the reason both live in one tool: they need the same
instrument (a step, then the encoder at full rate), and comparing a staircase against a step is what
separates "the handoff is fixed" from "the handoff only survives a gentle descent".

MODES. `--mode thrust` steps a raw COMMAND open-loop; `--mode rpm` steps the closed-loop velocity
target. They answer different questions — open loop shows what the ESC does, closed loop shows what
the controller does on top of it, and the startup overshoot is a closed-loop phenomenon. Raw thrust
needs a keep-alive (500 ms deadman) and does not update `sense`.

  python3 host/steptrace.py 1 --mode rpm --to 1200 --startup      # the 2x startup overshoot
  python3 host/steptrace.py 1 --mode thrust --from 600 --to 480   # step DOWN through the seam
  python3 host/steptrace.py --analyze host/reports/steptrace_*.csv
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from esctool import EscHost, find_pico            # noqa: E402


def _encv(dev):
    try:
        for ln in dev.cmd("encv", timeout=2):
            if ln.startswith("encv|"):
                return float(ln.split("|")[2])
    except Exception:
        pass
    return None


def _tele(dev, idx):
    try:
        for ln in dev.cmd(f"tele {idx}", timeout=2):
            if ln.startswith("tele|"):
                return float(ln.split("|")[1])
    except Exception:
        pass
    return float("nan")


def _send(dev, idx, mode, value):
    """Send the command; False if the device refused it (e.g. the vc has already aborted).

    A STALL IS DATA, NOT AN ERROR. When the velocity controller hits ABORT_STALL it commands zero and
    disarms itself, after which every further `rpm` write answers "err not-armed". Letting that raise
    tears down the run and discards the trace of the stall — which is the one thing worth capturing.
    Measured bare-shaft: `rpm 1200` starts cleanly, `rpm 2400` aborts partway up, and the difference
    between them is only visible if the aborted run is recorded.
    """
    try:
        dev.cmd(f"{'thrust' if mode == 'thrust' else 'rpm'} {idx} {value:d}", timeout=4)
        return True
    except Exception:
        return False


def trace(dev, idx, opts, rows):
    """Hold the pre-step command, step, and log until --window expires or the drive gives up.

    Returns True if the run completed, False if the device stopped accepting commands mid-trace.
    """
    # Pre-step hold. In thrust mode this is also the keep-alive; in rpm mode one command is enough,
    # but it is re-sent anyway so both modes spend the same time settling before the step.
    if opts.start is not None:
        t_end = time.time() + opts.pre
        while time.time() < t_end:
            if not _send(dev, idx, opts.mode, opts.start):
                return False
            time.sleep(0.15)
    t0 = time.time()
    _send(dev, idx, opts.mode, opts.to)
    while time.time() - t0 < opts.window:
        # Raw thrust dies to the 500 ms deadman, so it has to be re-sent all the way through the
        # trace. Re-sending a closed-loop `rpm` target is harmless (poll() re-sends every tick anyway)
        # and keeps the two modes' timing comparable.
        if not _send(dev, idx, opts.mode, opts.to):
            print(f"  !! drive refused the command at t={time.time() - t0:.2f}s — "
                  f"the controller aborted (ABORT_STALL disarms) and the rotor is coasting")
            # Keep logging: the decay after the abort shows whether it stalled outright or fell back.
            t_end = time.time() + min(2.0, opts.window - (time.time() - t0))
            while time.time() < t_end:
                r = _encv(dev)
                if r is not None:
                    rows.append((time.time() - t0, r, _tele(dev, idx), opts.to))
            return False
        r = _encv(dev)
        if r is None:
            continue
        rows.append((time.time() - t0, r, _tele(dev, idx), opts.to))
    return True


def quiet_settle(dev, idx, opts, n=5, gap=0.4, warmup=1.5):
    """The settled speed measured with the LINK IDLE — because this tool perturbs what it measures.

    THE OBSERVER EFFECT IS REAL AND IT IS BIG. `trace()` above issues `encv` + `tele` back to back as
    fast as the link allows (~175-370 Hz), and the Pico services that CLI on core0 — the same loop
    that runs `Thruster::poll()`, i.e. the velocity controller. Measured 2026-08-01 on the 930KV
    holding a 4000 rpm target, changing NOTHING but the poll rate:

        0.5 Hz  -> 3978.5 rpm      231 Hz -> 3872.5 rpm      0.5 Hz again -> 3980.1 rpm

    106 rpm, 2.7 % of the target, and it comes straight back when the polling stops. So the settled
    value taken from the trace tail is biased LOW by the act of tracing, and every absolute
    steady-state number this tool has ever printed carries that bias. The transient shape (rise time,
    overshoot, the stall) still needs the full rate, so the fix is not to slow the trace down — it is
    to measure the settled value separately, after the window, with the link quiet.

    Returns (mean, samples) or (nan, []) if the drive is gone (a stall disarms it)."""
    t_end = time.time() + warmup                     # let the loop recover from the polled window
    while time.time() < t_end:
        if not _send(dev, idx, opts.mode, opts.to):
            return float("nan"), []
        time.sleep(0.25)
    vals = []
    for _ in range(n):
        time.sleep(gap)
        if not _send(dev, idx, opts.mode, opts.to):
            break
        r = _encv(dev)
        if r is not None:
            vals.append(abs(r))
    return (sum(vals) / len(vals) if vals else float("nan")), vals


def analyse(rows, opts):
    rows = [r[:3] for r in rows]                     # tolerate rows that carry the trailing cmd
    if len(rows) < 5:
        print("  (too few samples)")
        return
    speeds = [abs(r[1]) for r in rows]
    ts = [r[0] for r in rows]
    # Final value from the tail, not the last sample: one sample is ripple, and this rotor's ripple
    # at low speed is the same size as the quantity being measured.
    t_tail = ts[-1] * (1 - opts.tail_frac)
    tail = [s for t, s in zip(ts, speeds) if t >= t_tail] or speeds[-1:]
    final = sum(tail) / len(tail)
    start = speeds[0]
    rising = final >= start
    print(f"\n=== step {opts.mode} {opts.start} -> {opts.to} ===")
    print(f"  {len(rows)} samples over {ts[-1]:.2f}s ({len(rows) / max(ts[-1], 1e-3):.0f} Hz)")
    print(f"  {start:.0f} rpm -> {final:.0f} rpm (final = mean of the last {opts.tail_frac:.0%})")
    # Overshoot has to be measured in the direction of travel. On a step DOWN the largest speed in
    # the trace is simply where it started, so a max()-based "overshoot" reports ~500% for a rotor
    # that in fact descended monotonically and never overshot anything.
    if rising:
        ext = max(speeds)
        label, t_ext = "peak", ts[speeds.index(ext)]
        over = (ext / final - 1) * 100 if final > 1.0 else float("nan")
    else:
        ext = min(speeds)
        label, t_ext = "trough", ts[speeds.index(ext)]
        over = (1 - ext / final) * 100 if final > 1.0 else float("nan")
    print(f"  {label} {ext:.0f} rpm at t={t_ext:.2f}s")
    if over == over:
        word = "overshoot" if rising else "undershoot"
        print(f"  {word} {over:.0f}% of the settled value"
              if over > 1.0 else f"  no {word} worth reporting ({over:.0f}%)")
    # Settling: the last time it was outside the band, not the first time it was inside — a decaying
    # oscillation enters the band repeatedly before it stays there. A "settling time" inside the tail
    # window is not a settling time at all: the tail is what DEFINED final, so the run simply ran out
    # of trace before settling.
    band = final * opts.settle_band
    out = [t for t, s in zip(ts, speeds) if abs(s - final) > band]
    if not out:
        print("  never leaves the settling band — no transient to speak of")
    elif out[-1] < t_tail:
        print(f"  settles within +-{opts.settle_band:.0%} at t={out[-1]:.2f}s")
    else:
        print(f"  NEVER settles within +-{opts.settle_band:.0%} inside the {ts[-1]:.1f}s window "
              f"(still {abs(speeds[-1] - final) / max(final, 1e-9):.0%} off at the end)")
    if final < opts.stall_rpm:
        print(f"  !! STALLED — ended at {final:.0f} rpm, below --stall-rpm {opts.stall_rpm:g}")
    # What telemetry claimed while all this was happening.
    live = [(t, s, tl) for t, s, tl in rows if tl == tl]        # drop NaN (no-telem)
    if live:
        blind = [x for x in live if abs(x[1]) > 1 and abs(x[2] - abs(x[1])) > abs(x[1]) * 0.25]
        print(f"  tele: {len(live)}/{len(rows)} samples reported; "
              f"{len(blind)} of them off the true speed by >25%")
        if blind:
            t, s, tl = blind[0]
            print(f"        first at t={t:.2f}s: tele said {tl:.0f}, rotor was at {abs(s):.0f}")
    else:
        print(f"  tele: SILENT for the whole {ts[-1]:.1f}s — the encoder is the only witness")


def main(argv=None):
    ap = argparse.ArgumentParser(description="step response on the encoder, at full poll rate")
    ap.add_argument("index", type=int, nargs="?", default=1)
    ap.add_argument("--mode", choices=("thrust", "rpm"), default="rpm",
                    help="step a raw open-loop COMMAND, or the closed-loop velocity target")
    ap.add_argument("--from", dest="start", type=int, default=None,
                    help="command/target to hold before the step (omit with --startup)")
    ap.add_argument("--to", type=int, required=False, help="command/target to step to")
    ap.add_argument("--startup", action="store_true",
                    help="step from REST: arm, then command --to. This is the 2x-overshoot case.")
    ap.add_argument("--pre", type=float, default=6.0, help="s to hold --from before stepping")
    ap.add_argument("--window", type=float, default=12.0, help="s to log after the step")
    ap.add_argument("--tail-frac", type=float, default=0.25,
                    help="fraction of the trace averaged for the settled value")
    ap.add_argument("--settle-band", type=float, default=0.05, help="settling band, fraction of final")
    ap.add_argument("--stall-rpm", type=float, default=30.0, help="below this the run counts as stalled")
    ap.add_argument("--repeats", type=int, default=1)
    ap.add_argument("--no-quiet-settle", action="store_true",
                    help="skip the post-window idle-link settle measurement (see quiet_settle: this "
                         "tool's own polling costs ~2.7 %% of the settled speed, so the trace tail "
                         "reads LOW; the quiet number is the honest one)")
    ap.add_argument("--analyze", default=None, help="re-analyse an existing CSV; no hardware")
    ap.add_argument("--out", default=None)
    ap.add_argument("--port", default=None)
    ap.add_argument("--dry-run", action="store_true")
    opts = ap.parse_args(argv)

    if opts.analyze:
        by_run, cmds = {}, {}
        with open(opts.analyze) as f:
            rd = csv.DictReader(f)
            # The `cmd` column was added 2026-07-30 and APPENDED, so pre-2026-07-30 traces (which have
            # only run,t,enc_rpm,tele_rpm) still read. Without it a replay has to guess what was
            # commanded from the filename, which is how a fixture manifest ends up with inferred
            # commands instead of recorded ones.
            has_cmd = bool(rd.fieldnames) and "cmd" in rd.fieldnames
            for r in rd:
                by_run.setdefault(int(r["run"]), []).append(
                    (float(r["t"]), float(r["enc_rpm"]), float(r["tele_rpm"])))
                if has_cmd:
                    cmds.setdefault(int(r["run"]), set()).add(float(r["cmd"]))
        meta = os.path.basename(opts.analyze)
        print(f"# re-analyse {sum(len(v) for v in by_run.values())} samples from {meta} "
              f"({'with' if has_cmd else 'NO'} cmd column)")
        for k, rows in sorted(by_run.items()):
            print(f"\n--- run {k} ---")
            if has_cmd:
                print(f"  commanded: {', '.join(f'{c:g}' for c in sorted(cmds[k]))} "
                      f"({opts.mode} units, as recorded)")
            analyse(rows, opts)
        return 0

    if opts.to is None:
        ap.error("--to is required unless --analyze is given")
    if opts.startup:
        opts.start = None

    if opts.dry_run:
        src = "REST (arm, then step)" if opts.startup else f"{opts.mode} {opts.start}"
        print(f"# DRY RUN — ESC {opts.index}: {src} -> {opts.mode} {opts.to}, "
              f"{opts.repeats}x, logging encv+tele for {opts.window:g}s each")
        if opts.mode == "thrust":
            print("#   raw thrust: 500ms deadman, re-sent throughout; does NOT update `sense`.")
        return 0

    out = opts.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), "reports",
                                   f"steptrace_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    dev = EscHost(find_pico(opts.port))
    all_rows = []
    try:
        dev.cmd("run", timeout=6)
        time.sleep(0.4)
        for k in range(opts.repeats):
            print(f"\n=== run {k + 1}/{opts.repeats} ===")
            dev.cmd(f"arm {opts.index} bidir", timeout=8)
            time.sleep(3.0)
            rows = []
            ok = trace(dev, opts.index, opts, rows)
            analyse(rows, opts)
            if not ok:
                print("  (run aborted by the controller — the trace above is what it managed)")
            elif not opts.no_quiet_settle:
                # The settled number that is NOT biased by this tool's own polling — see quiet_settle.
                qs, qv = quiet_settle(dev, opts.index, opts)
                if qv:
                    polled = [abs(r[1]) for r in rows if r[0] >= rows[-1][0] * (1 - opts.tail_frac)]
                    pm = sum(polled) / len(polled) if polled else float("nan")
                    print(f"  QUIET settle: {qs:.0f} rpm (link idle, {len(qv)} samples) vs {pm:.0f} "
                          f"polled — {qs - pm:+.0f} rpm of observer effect")
            all_rows += [(k + 1,) + r for r in rows]
            for step in (f"{'thrust' if opts.mode == 'thrust' else 'rpm'} {opts.index} 0",
                         f"disarm {opts.index}"):
                try:
                    dev.cmd(step, timeout=6)
                except Exception as e:
                    print(f"#   ('{step}' failed: {e})")
            time.sleep(3.0)              # let it come to rest before the next repeat
    except KeyboardInterrupt:
        print("\n!! interrupted")
    finally:
        for step in (f"thrust {opts.index} 0", f"rpm {opts.index} 0", f"disarm {opts.index}"):
            try:
                dev.cmd(step, timeout=6)
            except Exception as e:
                print(f"# WARNING: '{step}' failed ({e})")
            time.sleep(0.3)
        print("# stopped, disarmed")

    if all_rows:
        with open(out, "w", newline="") as f:
            w = csv.writer(f)
            # `cmd` is APPENDED, never inserted: --analyze on an older CSV must keep working.
            w.writerow(["run", "t", "enc_rpm", "tele_rpm", "cmd"])
            w.writerows(all_rows)
        print(f"# {len(all_rows)} samples -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
