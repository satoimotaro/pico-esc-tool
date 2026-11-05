#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""startlog — a journal of every open-loop start attempt, so the intermittent failure is CAUGHT
rather than remembered.

THE FAILURE. Twice, `thrust 650` produced 0 rpm with the supply at 0.153 A / 1.7 W: the ESC was
energised and simply never caught the rotor. Never reproducible on demand. Step-vs-ramp was refuted
(both work) and RPM-then-RAW was refuted (A/B both ~4130). Both events followed an ESC reflash. The
leading hypothesis is the ROTOR PARKING ANGLE, and nothing has ever recorded it.

WHY A JOURNAL AND NOT A TRAP. A trap that fires on failure gives you the failures and nothing to
compare them against. The parking angle is only diagnostic as a DISTRIBUTION: if failures cluster at
particular electrical angles and successes do not, the hypothesis is confirmed in one plot; if the
two distributions are indistinguishable over enough attempts, it is refuted and the guess was wrong.
So every attempt is logged, successes included — the successes are the control group.

THE ROTOR MUST BE SCATTERED ON PURPOSE — measured, not assumed. This tool was first written on the
assumption that repeated attempts would spread over the 84 cogging detents by themselves. The first
43-attempt baseline REFUTED that: 41 of 43 parked inside one 60-degree electrical band (25 in
0-30 deg, 16 in 330-360 deg). The zero-throttle brake pulls the rotor to the SAME electrical detent
every time, so an un-nudged campaign samples one angle and would never find an angle-dependent
failure however long it ran.

That is also the best evidence yet for the hypothesis, and it explains the intermittency: in normal
use the brake parks the rotor at a consistent angle and starts always work, while BOTH observed
failures followed a REFLASH — after which the ESC was reset and the rotor was left wherever it
happened to be, not where a brake had put it.

So each attempt is preceded by a NUDGE: a short forced-sine pulse (default cmd 350, below the
cmd ~515 handoff and far below the ~670 rpm brake cut-out, so the rotor creeps and no brake fires on
release) of randomised duration and sign. The rotor still settles into a cogging detent — it is just
no longer always the same one. `--analyze` prints the coverage so a campaign that failed to scatter
is visible rather than silently uninformative.

WHAT THE NUDGE DOES NOT FIX, stated because the coverage line will keep saying POOR. It took the
first 12 nudged attempts from 4/12 electrical bins to 7/12 and spread the MECHANICAL angle right
across the turn, but the electrical angle still piles up near 0-20 deg. The rotor lands on a cogging
detent either way, and the detents are neither uniformly deep nor uniformly reachable by releasing a
free-running rotor. Covering all 12 needs the rotor PLACED, i.e. a closed-loop park against the
encoder (posctl's PosDrive), which is a bigger piece of work and is deliberately not done here. So:
this journal can confirm the hypothesis (failures clustering at a covered angle) but it cannot yet
fully refute it, and `--analyze` says so rather than implying a clean sweep.

  python3 host/startlog.py 1 --repeats 40              # run a campaign, append to the journal
  python3 host/startlog.py --analyze                   # angle-vs-outcome, over ALL sessions

The journal is APPENDED ACROSS SESSIONS on purpose (host/reports/startlog_journal.csv): the event
rate is ~2 in a week of benching, so no single campaign will contain one. The instrument is meant to
be left in place and re-run, with `--analyze` reducing everything accumulated so far.

TWO WITNESSES, ALWAYS. `encv` alone cannot tell a stopped rotor from a blind encoder, and on
2026-08-08 that difference mattered: with the AS5600 unplugged the bench still reported md=1 (magnet
detected) and a freely advancing sample counter while `accum` sat frozen, so every liveness signal
the encoder offers about itself said "fine" while it saw nothing. Five attempts logged as FAILED,
four of them "reproducing" at an identical parking angle with concentration R=1.000 — because a
frozen accum cannot vary. That is fabricated evidence for the exact hypothesis this file exists to
test, which is worse than no evidence. Every attempt now also reads the ESC's own telemetry, and a
disagreement (tele turning, encoder not) is reported as NO-ENC and aborts the campaign. It is never
a failure.

OPEN LOOP ONLY, deliberately: the observed failures were open loop, and open loop is immune to the
CLI-polling starvation that invalidates closed-loop steady-state numbers on this rig.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import math
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from esctool import EscHost, find_pico, _read_raw      # noqa: E402
from pico_esc.constants import COUNTS_PER_REV          # noqa: E402
from pico_esc.motor_model import duty6                 # noqa: E402

KEEPALIVE_S = 0.2                                      # < the 500 ms raw-throttle deadman
JOURNAL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "reports",
                       "startlog_journal.csv")
FIELDS = ["stamp", "session", "attempt", "esc", "thrust", "pole_pairs", "park_accum",
          "park_mech_deg", "park_elec_deg", "outcome", "t_catch_s", "peak_rpm", "settle_rpm",
          "still_ok", "note"]


class Aborted(RuntimeError):
    pass


def _encv(dev):
    """(accum, rpm) or (None, None). accum is absolute, 4096 counts per MECHANICAL revolution."""
    try:
        for ln in dev.cmd("encv", timeout=3):
            if ln.startswith("encv|"):
                p = ln.split("|")
                return int(p[1]), float(p[2])
    except Exception:
        pass
    return None, None


def _tele_rpm(dev, idx):
    """The ESC's own mechanical rpm, or None when telemetry is not live.

    The SECOND WITNESS, and it is not optional. `encv` alone cannot tell a stopped rotor from a
    blind encoder: with the AS5600 unplugged this bench still reported md=1 (magnet detected) and a
    freely advancing sample counter while `accum` sat frozen — so every liveness signal the encoder
    itself offers said "fine" while it saw nothing. Telemetry is independent of that failure and
    caught it immediately (tele climbed to 1398 rpm against encv 0.00)."""
    try:
        for ln in dev.cmd(f"tele {idx}", timeout=3):
            if ln.startswith("tele|"):
                return float(ln.split("|")[1])
    except Exception:
        return None
    return None


def park_angles(accum, pole_pairs):
    """Mechanical and ELECTRICAL parking angle, in degrees, from the absolute count.

    The electrical angle is the one the hypothesis is about: a start failure that depends on where
    the rotor sits relative to the WINDINGS repeats every electrical revolution, i.e. every
    COUNTS_PER_REV/pole_pairs = 585.1 counts on a 7-pole-pair motor, not every mechanical turn."""
    mech = (accum % COUNTS_PER_REV) * 360.0 / COUNTS_PER_REV
    per_elec = COUNTS_PER_REV / float(pole_pairs)
    return mech, (accum % per_elec) * 360.0 / per_elec


def wait_still(dev, opts):
    """Block until the rotor is genuinely stopped, then return its parking count.

    Returns (accum, still_ok). still_ok=0 means it never went quiet inside --still-timeout; the
    attempt still runs (a moving rotor is itself a condition worth having in the journal) but the
    parking angle for that row is not meaningful."""
    t_end = time.time() + opts.still_timeout
    last = None
    stable_since = None
    while time.time() < t_end:
        accum, rpm = _encv(dev)
        if accum is None:
            time.sleep(0.05)
            continue
        moved = last is None or abs(accum - last) > opts.still_counts
        last = accum
        if moved or abs(rpm) > opts.still_rpm:
            stable_since = None
        elif stable_since is None:
            stable_since = time.time()
        elif time.time() - stable_since >= opts.still_secs:
            return accum, 1
        time.sleep(0.05)
    return (last if last is not None else 0), 0


def nudge(dev, idx, opts, rng):
    """Move the rotor off the electrical detent the BRAKE parks it at.

    Forced sine on purpose: below the handoff the rotor follows the commanded field at ~125 rpm, so
    it creeps a fraction of a turn and never approaches the ~670 rpm brake cut-out — release
    therefore leaves it on a cogging detent of its own rather than one the brake chose. Randomised
    sign as well as duration, so the sampling does not walk one way around the rotor."""
    if opts.nudge_cmd <= 0:
        return
    cmd = int(opts.nudge_cmd) * rng.choice((1, -1))
    dur = rng.uniform(opts.nudge_min_s, opts.nudge_max_s)
    dev.cmd(f"arm {idx} bidir", timeout=8)
    time.sleep(opts.arm_settle)
    t0 = time.time()
    while time.time() - t0 < dur:
        dev.cmd(f"thrust {idx} {cmd}", timeout=3)
        _, rpm = _encv(dev)
        if rpm is not None and abs(rpm) > opts.nudge_rpm_max:
            break                                          # never let a nudge reach braking speed
        time.sleep(KEEPALIVE_S)
    stop(dev, idx)
    time.sleep(opts.rest)


def attempt(dev, idx, opts, trace_path=None):
    """One open-loop start from rest. Returns the journal row (minus the campaign columns)."""
    park, still_ok = wait_still(dev, opts)
    mech, elec = park_angles(park, opts.pole_pairs)
    print(f"    parked at accum {park} -> mech {mech:6.1f} deg, elec {elec:6.1f} deg"
          f"{'' if still_ok else '   ** NEVER WENT STILL **'}")

    dev.cmd(f"arm {idx} bidir", timeout=8)
    time.sleep(opts.arm_settle)

    rows, t0, t_catch, peak = [], time.time(), None, 0.0
    tele_peak, next_ka = 0.0, 0.0
    while True:
        el = time.time() - t0
        if el >= opts.window:
            break
        if el >= next_ka:                                  # feed the 500 ms raw deadman
            dev.cmd(f"thrust {idx} {opts.thrust}", timeout=3)
            next_ka = el + KEEPALIVE_S
        accum, rpm = _encv(dev)
        t = _tele_rpm(dev, idx)
        if t is not None:
            tele_peak = max(tele_peak, abs(t))
        if accum is None:
            continue
        rows.append((el, accum, rpm))
        peak = max(peak, abs(rpm))
        if abs(rpm) > opts.rpm_ceiling:
            raise Aborted(f"overspeed during start: {rpm:.0f} > {opts.rpm_ceiling:.0f}")
        if t_catch is None and abs(rpm) >= opts.catch_rpm:
            t_catch = el

    settle = ([abs(r) for _, _, r in rows[-8:]] or [0.0])
    settle = sum(settle) / len(settle)
    # SLOW is not a nicety. The 2026-08-01 screen of what a flash resets found ONE setting that
    # stops the motor (startup_power_max) and THREE that let it start and then hold a small
    # fraction of the right speed — low_rpm_power_protection=9 at 380 rpm, sine_cross_up=255 at
    # 230 (handoff refused, stuck in forced sine), motor_direction=Normal at 180, all against 1600.
    # Classifying purely on "did it turn" would log every one of those as a clean CAUGHT, i.e. the
    # instrument would miss three of the four known failure modes.
    expect = opts.kv * opts.v * duty6(opts.thrust, opts.cross_up)
    # NO-ENC before anything else. A blind encoder reads exactly like a rotor that never moved, and
    # on 2026-08-08 that manufactured five FAILED rows — including four that "reproduced" at an
    # identical parking angle with R=1.000, because a frozen accum cannot vary. Fabricated evidence
    # for the very hypothesis this file exists to test is worse than no evidence, so the disagreement
    # is now a first-class outcome and NEVER a failure.
    if t_catch is None and tele_peak >= opts.catch_rpm:
        print(f"    ** NO-ENC: the ESC reports {tele_peak:.0f} rpm while the encoder never moved. "
              f"The ROTOR IS FINE and the ENCODER is not. Not logged as a failure.")
        outcome = "NO-ENC"
    else:
        outcome = ("FAILED" if t_catch is None else
                   "SLOW" if expect > 0 and settle < opts.slow_frac * expect else "CAUGHT")
    if outcome == "SLOW":
        print(f"    ** SLOW: settled {settle:.0f} rpm against {expect:.0f} predicted — it started "
              f"and is running WRONG, which is the mode that hides")
    # The trace is what makes a FAILED row diagnosable rather than merely counted, so it is always
    # written on failure. Successes are kept only with --trace-all: they are numerous and the row
    # itself carries what the control group needs.
    if trace_path and (outcome in ("FAILED", "SLOW", "NO-ENC") or opts.trace_all):
        with open(trace_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "accum", "rpm"])
            w.writerows(rows)
        print(f"    trace -> {os.path.relpath(trace_path)}")
    return {"thrust": opts.thrust, "pole_pairs": opts.pole_pairs, "park_accum": park,
            "park_mech_deg": f"{mech:.2f}", "park_elec_deg": f"{elec:.2f}", "outcome": outcome,
            "t_catch_s": "" if t_catch is None else f"{t_catch:.3f}",
            "peak_rpm": f"{peak:.1f}", "settle_rpm": f"{settle:.1f}", "still_ok": still_ok}


def stop(dev, idx):
    """Each step guarded INDEPENDENTLY, so a benign first failure cannot skip the disarm.

    `not-armed` is EXPECTED here and not worth printing: the campaign stops after every attempt and
    the outer `finally` stops again, so the second pass always finds an already-disarmed ESC. A real
    warning has to stay visible, which it cannot do if it is buried under one per attempt."""
    for step in (f"thrust {idx} 0", f"disarm {idx}"):
        try:
            dev.cmd(step, timeout=6)
        except Exception as e:
            if "not-armed" not in str(e):
                print(f"# WARNING: '{step}' failed ({e})")
        time.sleep(0.4)


def append_journal(rows, path=JOURNAL):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    new = not os.path.exists(path)
    with open(path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
        if new:
            w.writeheader()
        w.writerows(rows)
    print(f"\n# {len(rows)} attempt(s) appended -> {os.path.relpath(path)}")


def analyze(path, bins):
    if not os.path.exists(path):
        sys.exit(f"no journal at {path} — run a campaign first")
    with open(path) as f:
        rows = [r for r in csv.DictReader(f)]
    ok = [r for r in rows if r["outcome"] == "CAUGHT"]
    bad = [r for r in rows if r["outcome"] in ("FAILED", "SLOW")]
    slow = [r for r in rows if r["outcome"] == "SLOW"]
    print(f"=== {len(rows)} attempts over {len({r['session'] for r in rows})} session(s): "
          f"{len(ok)} caught, {len(bad) - len(slow)} FAILED, {len(slow)} SLOW")
    if not rows:
        return
    if not bad:
        # Say what the run can and cannot support. An all-green journal bounds the rate; it does not
        # show the failure is gone, and the difference matters when the event is ~2 per week.
        hi = 1.0 - 0.05 ** (1.0 / len(rows))
        print(f"\nNo failure in the journal yet. With {len(rows)} clean attempts the failure rate is\n"
              f"below {100 * hi:.1f} % at 95 % confidence — an UPPER BOUND, not a clean bill of "
              f"health.\nKeep re-running; the instrument is the point, not this number.")

    def hist(sub, label):
        if not sub:
            return
        width = 360.0 / bins
        counts = [0] * bins
        for r in sub:
            counts[min(bins - 1, int(float(r["park_elec_deg"]) / width))] += 1
        peak = max(counts) or 1
        print(f"\n{label} by ELECTRICAL parking angle (n={len(sub)}):")
        for i, c in enumerate(counts):
            print(f"  {i * width:5.0f}-{(i + 1) * width:5.0f} deg | {'#' * int(20 * c / peak):<20} {c}")

    hist(ok, "CAUGHT")
    hist(bad, "FAILED")
    width = 360.0 / bins
    filled = {min(bins - 1, int(float(r["park_elec_deg"]) / width)) for r in rows}
    print(f"\nangle COVERAGE: {len(filled)}/{bins} electrical bins visited.")
    if len(filled) < bins * 0.6:
        print("  ** POOR — the rotor is not being scattered, so this journal cannot rule out an\n"
              "     angle-dependent failure however many attempts it holds. Check --nudge-cmd.")
    if bad and ok:
        # The comparison the whole tool exists for, stated as the two means on the unit circle so a
        # clustered set of failures is visible even when it straddles 0/360.
        def circ_mean(sub):
            a = [math.radians(float(r["park_elec_deg"])) for r in sub]
            s, c = sum(map(math.sin, a)), sum(map(math.cos, a))
            return math.degrees(math.atan2(s, c)) % 360.0, math.hypot(s, c) / len(a)
        for lbl, sub in (("CAUGHT", ok), ("FAILED", bad)):
            m, r = circ_mean(sub)
            print(f"{lbl:>7}: circular mean {m:6.1f} deg, concentration R={r:.3f} "
                  f"({'clustered' if r > 0.5 else 'spread out'})")
        print("\nR near 0 = the angle is uniform, i.e. no parking-angle dependence. A FAILED set "
              "with\nhigh R at an angle the CAUGHT set avoids is the positive result.")


def main():
    ap = argparse.ArgumentParser(description="journal every open-loop start attempt")
    ap.add_argument("index", type=int, nargs="?", default=1)
    ap.add_argument("--repeats", type=int, default=20)
    ap.add_argument("--thrust", type=int, default=650, help="the command the failures were seen at")
    ap.add_argument("--pole-pairs", type=int, default=7)
    ap.add_argument("--window", type=float, default=3.0, help="s to watch for a catch")
    ap.add_argument("--catch-rpm", type=float, default=150.0,
                    help="|rpm| that counts as caught. Well above cogging dither and well below any "
                         "real 6-step speed, so the classification is not a judgement call")
    ap.add_argument("--rpm-ceiling", type=float, default=3000.0)
    ap.add_argument("--kv", type=float, default=350.0, help="for the expected-speed check")
    ap.add_argument("--v", type=float, default=11.20)
    ap.add_argument("--cross-up", type=int, default=36)
    ap.add_argument("--slow-frac", type=float, default=0.7,
                    help="settling below this fraction of the predicted speed is SLOW, not CAUGHT")
    ap.add_argument("--arm-settle", type=float, default=3.0, help="s after arm before commanding")
    ap.add_argument("--rest", type=float, default=2.0, help="s between attempts, after the brake")
    ap.add_argument("--still-secs", type=float, default=0.6, help="s of no motion = parked")
    ap.add_argument("--still-counts", type=int, default=3, help="counts of drift still called still")
    ap.add_argument("--still-rpm", type=float, default=5.0)
    ap.add_argument("--still-timeout", type=float, default=15.0)
    ap.add_argument("--nudge-cmd", type=int, default=350,
                    help="forced-sine command used to move the rotor off the brake's detent before "
                         "each attempt (0 disables). Must stay BELOW the handoff, or the nudge "
                         "becomes a start")
    ap.add_argument("--nudge-min-s", type=float, default=0.2)
    ap.add_argument("--nudge-max-s", type=float, default=0.8)
    ap.add_argument("--nudge-rpm-max", type=float, default=400.0,
                    help="bail out of a nudge above this, so it can never reach the ~670 rpm brake "
                         "cut-out and re-park the rotor the very way we are trying to avoid")
    ap.add_argument("--seed", type=int, default=None, help="seed the nudge RNG for a repeatable run")
    ap.add_argument("--note", default="", help="free text stamped on every row (e.g. 'after reflash')")
    ap.add_argument("--trace-all", action="store_true", help="keep traces for successes too")
    ap.add_argument("--analyze", nargs="?", const=JOURNAL, default=None,
                    help="reduce the journal (default: all accumulated sessions); no hardware")
    ap.add_argument("--bins", type=int, default=12, help="histogram bins over 360 electrical deg")
    ap.add_argument("--journal", default=JOURNAL,
                    help="where the cross-session journal lives. The default is under host/reports/, "
                         "which is GITIGNORED — fine for a tool heading for public release, wrong "
                         "for the data. Point it somewhere versioned — a per-installation bench "
                         "registry is the intended home")
    ap.add_argument("--port", default=None)
    opts = ap.parse_args()

    if opts.analyze:
        analyze(opts.analyze if opts.analyze is not JOURNAL else opts.journal, opts.bins)
        return 0

    session = time.strftime("%Y%m%d_%H%M%S")
    outdir = os.path.join(os.path.dirname(JOURNAL), f"startlog_{session}")
    os.makedirs(outdir, exist_ok=True)
    dev = EscHost(find_pico(opts.port))
    print(f"# ESC {opts.index}, thrust {opts.thrust}, {opts.repeats} attempts, session {session}")
    dev.cmd("run", timeout=6)
    dev.cmd("disconnect", timeout=6)
    time.sleep(0.4)

    # Stamp the ESC's CONFIG on every row. The 2026-08-01 experiment showed that post-flash
    # defaults cause a hard, deterministic start failure (startup_power_max=5 alone is enough), so
    # "what was the ESC configured as" is at least as diagnostic as the parking angle — and a flash
    # wipes the config every time. Once per session: it does not change mid-campaign.
    raw = _read_raw(dev, opts.index)
    cfg_tag = f"cfg:{hashlib.sha256(raw).hexdigest()[:12]}" if raw else "cfg:UNREADABLE"
    print(f"# ESC config {cfg_tag} — compare against a known-good config snapshot if a row FAILS")
    dev.cmd("disconnect", timeout=6)
    time.sleep(0.4)

    rng = random.Random(opts.seed)
    rows, status = [], "completed"
    try:
        for n in range(1, opts.repeats + 1):
            print(f"\n=== attempt {n}/{opts.repeats} ===")
            nudge(dev, opts.index, opts, rng)
            r = attempt(dev, opts.index, opts,
                        trace_path=os.path.join(outdir, f"attempt_{n:03d}.csv"))
            r.update(stamp=time.strftime("%Y-%m-%dT%H:%M:%S"), session=session, attempt=n,
                     esc=opts.index,
                     note=(f"nudge={opts.nudge_cmd} " if opts.nudge_cmd > 0 else "unnudged ")
                          + f"{cfg_tag} " + opts.note)
            if r["outcome"] == "NO-ENC":
                raise Aborted("the encoder is not reading while the ESC says the rotor is turning "
                              "— fix the encoder before collecting more rows, they would all be "
                              "artifacts")
            rows.append(r)
            mark = "ok" if r["outcome"] == "CAUGHT" else "*** FAILED ***"
            print(f"    {r['outcome']}  t_catch={r['t_catch_s'] or '-':>6}  "
                  f"peak={r['peak_rpm']}  settle={r['settle_rpm']}   {mark}")
            stop(dev, opts.index)
            time.sleep(opts.rest)
    except Aborted as e:
        status = f"ABORTED: {e}"
        print(f"\n!! {status}")
    except KeyboardInterrupt:
        status = "interrupted by user"
        print(f"\n!! {status}")
    finally:
        stop(dev, opts.index)
        print("# stopped, disarmed")

    if rows:
        append_journal(rows, opts.journal)
        analyze(opts.journal, opts.bins)
    print(f"# status: {status}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
