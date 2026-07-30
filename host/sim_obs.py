#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""sim_obs — the MEASURED plant, the soft-sensor observer, the DOB, and the startup-overshoot study.

REWRITTEN 2026-07-30. What this file used to be, and why none of it survived:

    PlantObs(sine lag 0.18 s, 6-step lag 0.025 s, regime threshold 250 rpm, damping 0.28) + 0.28 s deadtime

Four fitted constants and a deadtime, none physical, and a headline of "~79 % VAF".

  * the 0.18 s SINE LAG cannot be rotor mechanics: `J*R/K_T^2` is 8.3 ms at R=0.7 ohm and 14.2 ms at the
    measured R_total = 1.20 — 13 to 22x smaller. It was fitted to the sysid step 65 -> 407, where the
    rotor wanders -29..249 rpm around a 145 rpm forced field. There is no first-order rise there to
    fit. That lag was fitted to COGGING NOISE, and `_smooth()` existed to hide that.
  * the 250 rpm THRESHOLD + the 0.28 s DEADTIME encoded the up-handoff as a threshold plus dead time. The
    measured fold is a smooth ramp with no resolvable hysteresis, and the "dead time" is a duty ramp
    plus a duty clamp: the rotor sits at 360-415 rpm (not 0) for ~0.26 s and then covers the last
    1800 rpm in 40 ms.
  * the 0.28 DAMPING RATIO modelled a 2nd-order sine "overshoot" that was measurement ripple.
  * and `from softsensor import duty as duty_of` took the DEFAULT cross_up of 34 while the bench runs
    36 — so the sim silently modelled a different ESC than the hardware it was validated against.

It is now a thin driver over `pico_esc.plant.MeasuredPlant`, whose structure is physics and whose
numbers are provenance-tagged in `pico_esc.measured`. `--cross-up` is REQUIRED on the CLI.

  python3 host/sim_obs.py --cross-up 36                  # bare shaft (the current bench)
  python3 host/sim_obs.py --cross-up 34 --config prop_350kv_20260726
"""
from __future__ import annotations

import argparse
import json
import math
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from pico_esc import measured as M                                   # noqa: E402
from pico_esc.link import SimClock                                   # noqa: E402
from pico_esc.plant import (MeasuredEscHost, MeasuredPlant, preset,   # noqa: E402
                            with_cross_up, with_crossover_clamp, with_drag)
from softsensor import duty as duty_of                               # noqa: E402
from worldmodel import fit_stats                                     # noqa: E402


# ---------- (a) plant fidelity against the recorded sysid steps ----------
def validate_dynamics(cfg, steps):
    """Replay the recorded sysid steps through MeasuredPlant.

    VAF is reported ONLY where it means something. In the two sine-region steps the rotor is
    cogging-dominated, so a VAF there measures how well a model reproduces dispersion — which is
    exactly the number that justified the old 0.18 s sine lag. They are reported as
    `n/a (cogging-dominated)` and are the reason no lag parameter exists in the new plant."""
    print("## (a) Plant fidelity — replay the recorded sysid steps (no fitted lag exists)")
    print(f"  {'step':16s} {'n':>4} {'VAF':>24} {'rmse':>8}")
    vafs = []
    for s in steps:
        rows = [(t, r) for t, r, *_ in s["rows"] if r is not None]
        if len(rows) < 5:
            continue
        plant = MeasuredPlant(preset(cfg))
        plant.reset(rows[0][1], 0.0)
        sim, prev = [plant.rpm], rows[0][0]
        for t, _r in rows[1:]:
            n = max(1, int((t - prev) / 5e-4))
            for _ in range(n):
                plant.step(s["target"], (t - prev) / n)
            sim.append(plant.rpm)
            prev = t
        real = [r for _t, r in rows]
        cogging = max(abs(plant.steady(s["bias"])[0] or 0.0),
                      abs(plant.steady(s["target"])[0] or 0.0)) < 500.0
        if cogging:
            print(f"  {str(s['bias']) + '->' + str(s['target']):16s} {len(rows):4d} "
                  f"{'n/a (cogging-dominated)':>24} "
                  f"     -- rotor spans {min(real):.0f}..{max(real):.0f} rpm around a "
                  f"{plant.sine_field(s['target']):.0f} rpm field")
        else:
            st = fit_stats(real, sim)
            vafs.append(st["vaf"])
            print(f"  {str(s['bias']) + '->' + str(s['target']):16s} {len(rows):4d} "
                  f"{st['vaf']:23.1f}% {st['rmse']:8.0f}")
    if vafs:
        print(f"  {'MEAN (6-step only)':16s} {'':4s} {statistics.mean(vafs):23.1f}%")
    return statistics.mean(vafs) if vafs else 0.0


# ---------- (a) the soft-sensor observer ----------
def validate_observer(plant, cross_up, obs):
    """`V_eff = rpm/(KV*duty(cmd, cross_up))` against the recorded obschar sweep.

    Skipped when `models/obschar_f2838_350kv.json` is absent: `host/models/` is gitignored, so this
    section simply does not exist on a clean checkout and must degrade rather than crash."""
    print("\n## (a) Observer — reproduce the obschar V_eff droop")
    if obs is None:
        print("  (models/obschar_f2838_350kv.json absent — gitignored, so skipped on a clean "
              "checkout. Re-measure with `python3 host/obschar.py --cross-up <byte>`.)")
        return
    rec = obs.get("cross_up")
    if rec is None:
        rec = preset(M.PROP).cross_up
        print(f"  (that file predates obschar.py recording its cross_up. It was taken prop-on on "
              f"2026-07-26, so it is a cross_up={rec:g} sweep — re-run obschar.py to stamp it.)")

    def rmse_at(cu):
        errs = []
        for pt in obs["points"]:
            d = duty_of(pt["cmd"], cu)
            vs = abs(pt["rpm"]) / (plant.p.kv * d) if d > 0.02 and abs(pt["rpm"]) > 100 else 0.0
            errs.append((pt, vs, vs - pt["vest"]))
        return errs, math.sqrt(statistics.mean([e * e for _p, _v, e in errs]))

    errs, rms = rmse_at(rec)
    print(f"  {'cmd':>5} {'rpm':>6} {'vest_meas':>10} {'vest_sim':>9} {'err':>6}")
    for pt, vs, e in errs:
        print(f"  {pt['cmd']:5d} {pt['rpm']:6.0f} {pt['vest']:10.2f} {vs:9.2f} {e:+6.2f}")
    print(f"  observer RMSE at the RECORDING's cross_up={rec:g}: {rms:.3f} V "
          f"(same rpm/(KV*duty) model as the firmware -> a consistency check, not a measurement)")
    if cross_up != rec:
        _e2, rms2 = rmse_at(cross_up)
        print(f"  the SAME data at your --cross-up {cross_up:g}: RMSE {rms2:.3f} V "
              f"({rms2 / max(rms, 1e-9):.0f}x worse) — this is E.9's whole point in one number.")


# ---------- (b) DOB vs PI under a step load disturbance ----------
def dob_study(cfg):
    """rpm-domain disturbance observer on the MEASURED plant. The disturbance is now an honest extra
    DRAG TORQUE, not an "rpm-equivalent drop off the baseline" — the plant has a drag callable, so a
    load step is a load step."""
    print("\n## (b) Disturbance rejection — PI+FF vs PI+FF+DOB (step drag in 6-step)")
    DT, T = 0.02, 4.0
    n = int(T / DT)
    target = 1600.0
    base = preset(cfg)
    extra = {"tau": 0.0}
    inner = base.drag

    def drag(rpm):
        return inner(rpm) + extra["tau"]

    # through with_drag, because PlantParams is frozen — the disturbance IS a load change, which is
    # exactly the one thing the layering says may be swapped.
    p = with_drag(base, drag, name=base.name + "+step_load")
    KP, KI, TRIM, QTAU = 0.02, 0.20, 200.0, 0.08

    def run(use_dob):
        plant = MeasuredPlant(p)
        ff = plant.motor.thrust_for(target)
        # Settle at the FF command first. The DOB's reference model is the NO-LOAD curve, so the
        # baseline drag is itself a standing "disturbance" — seed d_hat with it, exactly as an observer
        # that has been running would have. Without the seed the DOB pushes a permanent offset and
        # "improves" the response into an oscillation.
        extra["tau"] = 0.0
        plant.reset(target, 0.0)
        for _ in range(2000):
            plant.step(ff, 1e-3)
        d_hat = (plant.steady(ff)[0] or 0.0) - plant.rpm
        integ = 0.0
        applied_prev = ff
        out = []
        for k in range(n):
            t = k * DT
            extra["tau"] = 3.0e-3 if t >= 1.0 else 0.0     # +3 mN*m of drag at t = 1 s
            e = target - plant.rpm
            integ = max(-TRIM, min(TRIM, integ + KI * e * DT))
            trim = max(-TRIM, min(TRIM, KP * e + integ))
            comp = 0.0
            if use_dob:
                d_meas = (plant.steady(applied_prev)[0] or 0.0) - plant.rpm
                d_hat += (1.0 - math.exp(-DT / QTAU)) * (d_meas - d_hat)
                slope = max(0.05, ((plant.steady(ff + 10)[0] or 0.0)
                                   - (plant.steady(ff - 10)[0] or 0.0)) / 20.0)
                comp = d_hat / slope
            cmd = ff + trim + comp
            applied_prev = cmd
            for _ in range(int(DT / 1e-3)):
                plant.step(cmd, 1e-3)
            out.append((t, target - plant.rpm))
        return out

    def metrics(series):
        # error relative to the pre-step settled value: the plant droops below the no-load target under
        # its own drag, and calling that droop a "control error" would flatter the DOB.
        base = statistics.mean([e for t, e in series if t < 1.0])
        series = [(t, e - base) for t, e in series]
        w = [abs(e) for t, e in series if t >= 1.0]
        settle = next((t - 1.0 for t, e in series if t >= 1.05 and abs(e) <= 0.02 * target), None)
        return max(w), settle, sum(abs(e) for t, e in series if t >= 1.0) * DT

    pp, ps, pi_iae = metrics(run(False))
    dp, ds, dob_iae = metrics(run(True))
    print(f"  target {target:.0f} rpm, +3 mN*m of drag at t = 1.0 s")
    print(f"  {'':10s} {'peak |err|':>11} {'settle 2%':>10} {'IAE':>9}")
    print(f"  {'PI+FF':10s} {pp:9.0f}rpm {(f'{ps:.2f}s' if ps else 'no'):>10} {pi_iae:9.0f}")
    print(f"  {'PI+FF+DOB':10s} {dp:9.0f}rpm {(f'{ds:.2f}s' if ds else 'no'):>10} {dob_iae:9.0f}")
    if pp > 0:
        print(f"  -> DOB: peak error {100 * (1 - dp / pp):+.0f}%, IAE {100 * (1 - dob_iae / pi_iae):+.0f}%"
              f" (positive = better)")
        print("     HONEST NOTE: the win is much smaller than the old sim's '8x faster settling'. That")
        print("     figure came from the fitted plant's 25 ms 6-step lag plus its 0.28 s handoff deadtime;")
        print("     the measured plant's own J*R/K_T^2 is ~14 ms, so PI alone already recovers in ~0.1 s")
        print("     and the DOB's remaining value is the integrated error, not the settling time.")


# ---------- (c) the startup overshoot, and WHERE IT COMES FROM ----------
def startup_run(cfg, target=1200.0, kp=0.4, ki=1.5, trim_max=90.0, slew=1200.0,
                tele_floor=True, tele_live=True, crossover_clamp=None, secs=6.0):
    """One closed-loop run from rest on the measured plant. Returns a dict of shape metrics.

    Uses the EXISTING `pico_esc.velocity.VelocityController` over `MeasuredEscHost` — the controller
    is not reimplemented, and `pico_esc/sim.py` is not touched."""
    from pico_esc.control import DT
    from pico_esc.esc import ESC
    from pico_esc.velocity import VelocityController

    p = preset(cfg)
    if crossover_clamp is not None:
        p = with_crossover_clamp(p, crossover_clamp)
    clock = SimClock()
    plant = MeasuredPlant(p)
    host = MeasuredEscHost(plant, clock, tele_floor=tele_floor, tele_live=tele_live)
    esc = ESC(host, 1, tmax=800, clock=clock)
    prof = plant.motor.to_profile(cmd_hi=700)
    vc = VelocityController(esc, prof, tele_period=DT, max_secs=secs, max_temp=0.0,
                            stall_secs=1e9, over_speed_rpm=1e9, slew_rpm_s=slew,
                            kp=kp, ki=ki, trim_max=trim_max)
    esc.arm(bidir=True)
    vc.set_speed(target)
    rows = []
    vc.run(clock, on_row=lambda t, tg, sp, thr, tp, er, tr, trm:
           rows.append((t, plant.rpm, thr, trm)))
    peak = max(abs(r[1]) for r in rows)
    t_peak = next(t for t, r, *_ in rows if abs(r) == peak)
    tail = [abs(r) for t, r, *_ in rows if t > secs * 0.75]
    final = sum(tail) / len(tail)
    ripple = max(tail) - min(tail)
    return {"peak": peak, "t_peak": t_peak, "final": final,
            "over": 100.0 * (peak / max(final, 1.0) - 1.0),
            # A "settled value" is only a settled value if the tail is actually flat. Quoting an
            # overshoot percentage against the MEAN of a swinging tail is how a limit cycle gets
            # reported as a 170 % overshoot.
            "settled": ripple <= 0.10 * target, "tail_ripple": ripple, "rows": rows}


def startup_study(cfg, target=1200.0):
    """Behaviour 4, and an HONEST negative result.

    The measured bare-shaft startup overshoots 53-54 % with t_peak 1.19 s and settles by 3.0-3.2 s.
    The old sim could not overshoot at all (a first-order lag toward a static curve cannot), so
    reproducing the mechanism was the goal. The hypothesis on the table was: the overshoot is the
    measured SENSOR LIE (`tele` pinned at 182 rpm while the rotor is slower) feeding integrator windup.

    THAT HYPOTHESIS DOES NOT SURVIVE THIS MODEL. Removing the sensor lies entirely — feeding the
    controller the rotor's true speed from t = 0 — barely changes the overshoot. The 182 rpm FLOOR and
    the `TELE_LIVE_S` startup SILENCE are separate knobs (`MeasuredEscHost(tele_floor=, tele_live=)`)
    so each is tested alone rather than both at once. The reason is arithmetic: the
    182 rpm floor can only lie while the rotor is BELOW 182 rpm, and on this plant that window is under
    0.1 s, far too short to wind an integrator up by 500 rpm.

    What the model DOES show is that the loop's problem is the CROSSOVER SEAM's incremental gain: at
    cross_up 36 the steady speed goes 186 -> 259 rpm over cmd 515 -> 522 and then 259 -> 1194 rpm over
    cmd 522 -> 612. Any integral trim large enough to cross that seam bang-bangs across it, which is
    the Lake note's "do NOT close aggressive feedback across the crossover gap" reproduced from
    measured numbers. With the shipped gains the loop does not settle at all; with a trim clamp small
    enough to stay above the seam it settles with ~0 % overshoot.

    So: the magnitude was already explicitly deferred (the 53-54 % came from the FIRMWARE
    `vel::VelocityController`, and this is the host mirror with different gains). The MECHANISM is now
    also reported as unresolved rather than claimed. Nothing here was tuned to hit 53 %.
    """
    print("\n## (c) Startup from rest (behaviour 4) — reported as an OPEN mechanism, not a match")
    print(f"  {'variant':44s} {'peak':>8} {'t_peak':>7} {'final':>7} {'over':>7} "
          f"{'ripple':>8}  verdict")
    cases = [("shipped kp/ki, trim 90, tele floor ON", dict()),
             ("...182 rpm FLOOR off (startup silence kept)", dict(tele_floor=False)),
             ("...startup SILENCE off (floor kept)", dict(tele_live=False)),
             ("...both sensor lies off", dict(tele_floor=False, tele_live=False)),
             ("...crossover duty clamp removed", dict(crossover_clamp=1.0)),
             ("gentle kp=0.1/ki=0.6, tele floor ON", dict(kp=0.1, ki=0.6)),
             ("gentle gains, both sensor lies off", dict(kp=0.1, ki=0.6, tele_floor=False,
                                                        tele_live=False))]
    res = {}
    for label, kw in cases:
        r = startup_run(cfg, target, **kw)
        res[label] = r
        print(f"  {label:44s} {r['peak']:8.0f} {r['t_peak']:7.2f} {r['final']:7.0f} "
              f"{r['over']:6.1f}% {r['tail_ripple']:8.0f}  "
              f"{'settles' if r['settled'] else 'LIMIT CYCLE — the % is meaningless'}")
    shipped, both_off = cases[0][0], cases[3][0]
    gentle_on, gentle_off = cases[5][0], cases[6][0]
    tgt = target
    print("  Only the two SETTLING rows have a meaningful overshoot at all — an 'overshoot %' taken")
    print("  against the mean of a swinging tail is not a number. Reading them:")
    print(f"  (a) NOT MET as the plan framed it. With gains that settle, the overshoot is "
          f"{res[gentle_on]['over']:.1f} % — nowhere near 25 %. With the SHIPPED gains the peak does")
    print(f"      exceed the {tgt:.0f} rpm target (by "
          f"{100 * (res[shipped]['peak'] / tgt - 1):.0f} %, and by "
          f"{100 * (res[both_off]['peak'] / tgt - 1):.0f} % with the sensor lies removed) but the loop")
    print(f"      never settles (ripple {res[shipped]['tail_ripple']:.0f} rpm), so that is a limit "
          f"cycle across the seam, not the measured decaying overshoot. The old fitted-lag sim could")
    print("      not overshoot at all, so the plant is no longer STRUCTURALLY incapable of it — but "
          "this is not a reproduction of the 53-54 % measurement and is not offered as one.")
    print("  (b) NOT SUPPORTED. In the only comparison where the number means something — the two")
    print(f"      settling rows — removing BOTH sensor lies moves the overshoot "
          f"{res[gentle_on]['over']:.1f} % -> {res[gentle_off]['over']:.1f} %, i.e. it gets very "
          f"slightly WORSE, not better.")
    print("      The 182 rpm floor can only lie while the rotor is below 182 rpm, which on this plant")
    print("      is under 0.1 s — far too short to wind an integrator up by 500 rpm. The floor and the")
    print("      startup silence are separate knobs so this A/B changes one thing at a time; neither")
    print("      alone nor both together produces the measured overshoot.")
    print("  What the model DOES show is the seam's incremental gain: the steady speed goes 186 -> 259")
    print("      rpm over cmd 515 -> 522 and then 259 -> 1194 over cmd 522 -> 612, so any integral")
    print("      trim large enough to cross the seam bang-bangs across it. That is the Lake note 'do")
    print("      NOT close aggressive feedback across the crossover gap', from measured numbers.")
    print(f"  measured bare-shaft, FIRMWARE controller: {M.STARTUP_OVERSHOOT_PCT:.0f} % overshoot, "
          f"t_peak {M.STARTUP_T_PEAK_S:.2f} s, settle 3.0-3.2 s — NOT tuned for, NOT matched.")
    print(f"  measured `rpm {M.STARTUP_ABORT_TARGET_RPM}` ABORT_STALLs bare-shaft while `rpm 1200` "
          f"starts. That gate is on traversing the crossover gap, so it does not port to any other "
          f"rotor — and the limit cycle above is the same seam gain seen from the other side.")
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(description="measured plant + observer + DOB + startup study (offline)")
    ap.add_argument("--cross-up", type=float, required=True,
                    help="the ESC's sine_cross_up. REQUIRED — there is no safe default. The old code "
                         "defaulted to 34 while the bench ran 36, which is a 20-40 %% static error "
                         "and it was silent")
    ap.add_argument("--config", default=M.BARE, choices=(M.BARE, M.PROP),
                    help="which MECHANICAL CONFIGURATION to model (they do not share J or drag)")
    opts = ap.parse_args(argv)

    cfg = opts.config
    want = preset(cfg).cross_up
    print("# sim_obs — the MEASURED plant (pico_esc.plant), offline. No fitted lags exist.\n")
    if opts.cross_up != want:
        print(f"# !! --cross-up {opts.cross_up:g} but the {cfg} fixtures were recorded at {want:g}. "
              f"Continuing, so you can SEE the error this causes (that is the point of E.9).\n")
    plant = MeasuredPlant(with_cross_up(preset(cfg), opts.cross_up))
    print(plant.summary())
    print()

    raw_path = os.path.join(HERE, "models", "350kv_postfix_sysid_raw.json")
    if os.path.exists(raw_path):
        raw = json.load(open(raw_path))
        curve = [(c["thrust"], c["rpm"]) for c in raw["curve"]]
        print(f"# tracked sysid curve: {len(curve)} pts, cmd {curve[0][0]}-{curve[-1][0]}, "
              f"rpm {curve[0][1]:.0f}-{curve[-1][1]:.0f} (recorded at cross_up=34, prop fitted)")
        # Replayed in the configuration the data was RECORDED in (prop fitted, cross_up 34),
        # not in whatever --config the caller asked for: replaying a prop-on dataset with bare-shaft
        # J and drag would be the exact provenance mixing this workstream exists to stop.
        validate_dynamics(M.PROP, raw["steps"])
    else:
        print("## (a) Plant fidelity — models/350kv_postfix_sysid_raw.json absent, skipped")

    obs_path = os.path.join(HERE, "models", "obschar_f2838_350kv.json")
    obs = json.load(open(obs_path)) if os.path.exists(obs_path) else None
    validate_observer(plant, opts.cross_up, obs)
    dob_study(cfg)
    startup_study(cfg)

    print("""
## Findings (measured physics with provenance, not fitted VAF)
1. THE STATIC CURVE AND THE SEAM ARE PARAMETER-FREE. `max(sine field, KV*V*eff*duty6(cmd, cross_up))`
   fits the tracked prop-on curve to <=2.1 % (cmd 543/612/680) and the bare-shaft sweeps to <=1.0 %
   (cmd 555/570/600) with eff = 1, and the seam EMERGES where the branches cross: cmd 514.7 at
   cross_up 36 (measured departure bracket 515..520) and cmd 481.8 at 34 (measured bracket 475..543).
   One correctly-passed integer replaced the old 250 rpm threshold. Run this with the WRONG cross_up and the
   same code is 13.7-45.0 % out — the term is wired in, not decorative.
2. `duty6` IS A LOADED CURVE, and forgetting that counts drag twice. It was fitted as
   rpm_measured/(KV*V) on a real bench, so recovering the ELECTRICAL duty means adding the reference
   drag's duty cost back — 12-16 % of duty over the measured 6-step commands. Without that the coupled
   ODE settles 12.2 % BELOW the static curve it claims to reproduce, which the first version of this
   file did while asserting the opposite. `test_step_converges_to_steady` integrates rather than
   inspecting the torque law in isolation, which is why it catches this and the old test did not.
3. THE ROTOR NEVER FREELY COASTS, AND THE BRAKE IS DELAYED, NOT INSTANT. At zero throttle the ESC keeps
   driving for CUT_RELEASE_S (0.17 s on `disarm`), coasts on drag alone for BRAKE_ENGAGE_S (0.14 s), and
   only then applies a short-circuit brake `K_T^2*w*R/(R^2+(pp*w*L)^2)` — the SAME torque law as the
   6-step drive with the duty removed — which then CUTS OUT below ~700 rpm (measured: the deceleration
   residual over the drag table has died by ~250 rpm, where the law's own w-branch would still be 7x
   that). The fixed delay is why the deceleration peak was measured at 1455-1596 rpm bare-shaft and
   865-973 rpm prop-on: the peak never moved, the speed the rotor had reached in 0.13 s did.
   THE BRAKE TORQUE SCALE IS FITTED ON `disarm` ONLY. `deadman` and `zero_held` brake 2-3x harder from
   the same ~2400 rpm and are NOT modelled; the mode dependence is in the torque, not the delay. But it
   TRANSFERS: replaying the protocol-matched prop-on decays (2026-07-30, +29 % inertia, nothing refitted)
   gives brake-phase duration -4.9..+4.9 % and revolutions -11.3..+12.9 %, as good as the in-sample fit.
   L and the cut-out are still tagged OPEN — one inertia change on the same motor is not water.
4. THE DYNAMICS ARE ESC-DOMINATED. `J*R/K_T^2` is 8.3 ms at R=0.7 and 14.2 ms at the measured
   R_total=1.20, against the ~37 ms the loop sees. The residual is the ESC's command shaping — a
   crossover duty ramp plus a duty clamp — not inertia and not a lag.
5. THE SINE REGION IS NOT A SPEED SOURCE. Below cmd 210 the rotor cogs and reverses (up to 7 reversals
   in a 3 s dwell, spread 92..234 rpm). The plant returns `coherent=False` and no speed rather than a
   mean that looks like a measurement.
6. THE PROPELLER CHANGES INERTIA AND NOTHING ELSE WE CAN FIND. Controlled pair, 2026-07-30, same tools
   and same starting speeds: J +29 % (7.585e-6 -> 9.761e-6) and the free coast -23 %, while the seam
   bracket (cmd 525..530), the 182 rpm tele floor, the forced-sine spread, the down-step and the closed-
   loop startup overshoot (53-54 % both) are all unchanged. TWO EXPECTATIONS FALSIFIED: the prop's
   inertia does NOT help the rotor follow the forced field, and it does NOT stop `rpm 2400`
   ABORT_STALLing — so that failure is a CONTROL / crossover-gap problem, not an inertia problem.
   The drag TORQUE is unchanged to 0.5 % (4.20 vs 4.18 mN*m), so there is ONE air drag law; it is
   bearing/iron loss plus a K_T*I_dc bias, NOT prop drag, and the w^2 coefficient water needs is
   UNMEASURED. The same pair bounds that table's shape error at 8.4 % (J_SHAPE_UNCERTAINTY_PCT).
7. THE STARTUP OVERSHOOT MECHANISM IS UNRESOLVED — section (c). The 182 rpm sensor floor is NOT the
   cause in this model, and nothing here was tuned toward the measured 53-54 %.
8. WATER CHANGES ONE CALLABLE: `PlantParams.drag`. Not `drag_ref` (that belongs to the `duty6`
   calibration), not the brake, not L, not J. `plantid.py validate --transfer` asserts it, and
   `PlantParams` is frozen, so the guarantee is a property of the type rather than a convention.""")
    return 0


if __name__ == "__main__":
    sys.exit(main())
