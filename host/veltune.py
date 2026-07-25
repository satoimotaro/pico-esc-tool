#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""veltune — encoder-feedback velocity PID + a model-seeded auto-tuner.

The shipped velocity controller (pico_esc.velocity) closes on the bidir-DShot `tele` frame
(sensorless). On the 350KV bare-shaft motor `tele` is frozen/virtual below ~250 rpm, so this LAB
tool closes the velocity loop on the AS5600 ENCODER (the ground truth), which is what makes tuning
on this plant possible at all. It is a bench/tuning instrument, not the deployment controller.

Loop (50 Hz): slew the setpoint toward the target; feed-forward the thrust from the identified
SpeedProfile; add a PID trim on (setpoint - filtered_enc_rpm) with derivative-on-measurement and
back-calculation anti-windup; clamp to +-tmax.

  run   — drive a target schedule with fixed gains; log the tracking trace + per-segment metrics.
  tune  — seed PI gains from the fitted plant (IMC / lambda tuning off model K, tau, delay), then
          coordinate-search (kp, ki, kd) minimizing a tracking cost (IAE + overshoot + jitter).

Sign: the SpeedProfile and VelReader share the same enc_sign abstraction (both built by sysid), so
the loop works in the sign-corrected frame — send esc.thrust(+u), read VelReader(sign=enc_sign);
+u -> +measured. Targets are positive (forward) for tuning.

Safety: overspeed cut (--max-rpm), tmax clamp, always-disarm. --dry-run drives SimEncEscHost.

Usage:
  python3 veltune.py --dry-run --profile profiles/simcheck_sysid.yaml run --targets 120,220,160
  python3 veltune.py --profile profiles/350kv_sine_sysid.yaml --model models/350kv_sine_sysid.json \
      --enc-sign -1 --max-rpm 400 tune --targets 120,220,160,260
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import statistics
import time

from pico_esc.control import DT, VelReader, _pace
from pico_esc.drive import Aborted
from pico_esc.esc import ESC
from pico_esc.link import EscHost, RealClock, SimClock
from pico_esc.velocity import SpeedProfile

REPORT_DIR = os.path.join(os.path.dirname(__file__), "reports")


def open_host(opts):
    if opts.dry_run:
        from pico_esc.sim import SimEncEscHost
        print(f"# DRY-RUN: SimEncEscHost (no serial/motor)"
              f"{' [--sim-invert]' if opts.sim_invert else ''}")
        clock = SimClock()
        return SimEncEscHost(clock, seed=opts.seed, invert=opts.sim_invert), clock
    return EscHost(opts.port), RealClock()


def log(m):
    print(m, flush=True)


# ---------------------------------------------------------------------------
# controller
# ---------------------------------------------------------------------------
class VelLoop:
    """FF (from the profile) + PID trim on filtered encoder error. Sign-corrected frame."""

    def __init__(self, profile, kp, ki, kd, slew, meas_tau, tmax, i_max=None, kback=None,
                 fb_floor=0.0, fb_band=0.0, err_band=0.0):
        self.p = profile
        self.kp, self.ki, self.kd = kp, ki, kd
        self.slew = slew                 # rpm/s setpoint rate limit
        self.meas_tau = meas_tau         # s, encoder low-pass for P/D
        self.tmax = tmax
        self.i_max = i_max if i_max is not None else tmax
        self.kback = kback if kback is not None else (1.0 / max(1e-3, math.sqrt(max(ki, 1e-6))))
        # regime gain-scheduling: FB authority = 0 below fb_floor, ramps to 1 over fb_band above it.
        # Keeps the loop from driving the operating point down across the crossover fold (no steady
        # state 175..~700 rpm) — that fold is what makes feedback collapse near the handoff. fb_band=0
        # disables scheduling (FB always full).
        self.fb_floor, self.fb_band = fb_floor, fb_band
        self.err_band = err_band
        self.reset()

    def _regime_gate(self):
        """FB authority in [0,1] from the filtered measurement vs the regime floor."""
        if self.fb_band <= 0:
            return 1.0
        return max(0.0, min(1.0, (abs(self.mf) - self.fb_floor) / self.fb_band))

    def _settle_gate(self, target):
        """Gate FB by SETPOINT-SETTLED, not error magnitude: ~0 while the setpoint is still slewing to
        the target (let the feed-forward own the transient -> no slew-overshoot), 1 once the setpoint
        has arrived (FB then trims the FULL steady error, however large — the new-motor / water-load
        case where FF is badly off). err_band = the |target - setpoint| band over which it ramps in;
        0 disables (FB always full)."""
        if self.err_band <= 0:
            return 1.0
        return max(0.0, min(1.0, 1.0 - abs(target - self.sp) / self.err_band))

    def reset(self, meas0=0.0):
        self.sp = meas0
        self.mf = meas0
        self.prev_mf = meas0
        self.integ = 0.0

    def step(self, target, meas, dt):
        # setpoint slew
        dmax = self.slew * dt
        self.sp += max(-dmax, min(dmax, target - self.sp))
        # measurement low-pass
        a = dt / (self.meas_tau + dt) if self.meas_tau > 0 else 1.0
        self.mf += a * (meas - self.mf)
        err = self.sp - self.mf
        ff = self.p.thrust_for(self.sp)
        dmeas = (self.mf - self.prev_mf) / dt if dt > 0 else 0.0
        self.prev_mf = self.mf
        g = self._regime_gate() * self._settle_gate(target)  # regime x setpoint-settled FB authority
        u_unsat = ff + g * (self.kp * err + self.integ - self.kd * dmeas)
        u = max(-self.tmax, min(self.tmax, u_unsat))
        # integrate (gated) with back-calculation anti-windup — no windup while FB is gated off
        self.integ += g * (self.ki * err + self.kback * (u - u_unsat)) * dt
        self.integ = max(-self.i_max, min(self.i_max, self.integ))
        return u, {"sp": self.sp, "mf": self.mf, "err": err, "ff": ff, "thrust": u, "gate": g}


# ---------------------------------------------------------------------------
# run a target schedule
# ---------------------------------------------------------------------------
def run_schedule(esc, clock, loop, targets, hold, enc_sign, max_rpm, warmup=0.6):
    """Drive each target for `hold` s. Returns (rows, segments). rows: list of dicts with t, target,
    sp, meas, thrust. segments: per-target metric dict. Keep-alive paced; overspeed-guarded."""
    vr = VelReader(sign=enc_sign)
    # gentle warmup to the first target so the first step isn't from a dead stop mid-schedule
    loop.reset(0.0)
    rows, segments = [], []
    t_global = clock.now()
    for ti, tgt in enumerate(targets):
        seg = []
        t_end = clock.now() + hold
        while clock.now() < t_end:
            tick = clock.now()
            meas = vr.read(esc, DT)
            if abs(meas) > max_rpm:
                esc.thrust(0)
                raise Aborted(f"overspeed {meas:.0f} > {max_rpm}")
            u, dbg = loop.step(tgt, meas, DT)
            esc.thrust(int(round(u)))          # sign-corrected frame: +u -> +measured
            row = {"t": tick - t_global, "target": tgt, "sp": dbg["sp"],
                   "meas": meas, "thrust": dbg["thrust"], "seg": ti}
            rows.append(row)
            seg.append((tick - t_end + hold, meas, tgt))   # t within segment
            _pace(clock, tick)
        segments.append(_seg_metrics(seg, tgt, warmup))
    esc.thrust(0)
    return rows, segments


def _seg_metrics(seg, target, warmup):
    """Metrics for one target hold: steady error, overshoot, 10-90 rise, IAE (post-warmup)."""
    if not seg:
        return {"target": target}
    ts = [s[0] for s in seg]
    ys = [s[1] for s in seg]
    # smooth (5-pt) for peak/rise/overshoot so the ~50 rpm low-speed ripple isn't read as overshoot;
    # steady mean and ripple std stay on the RAW signal.
    ysm = []
    for i in range(len(ys)):
        a, b = max(0, i - 2), min(len(ys), i + 3)
        ysm.append(sum(ys[a:b]) / (b - a))
    tail = ys[int(0.6 * len(ys)):]
    steady = statistics.mean(tail) if tail else 0.0
    y0 = ysm[0]
    span = target - y0
    peak = max(ysm) if span >= 0 else min(ysm)
    overshoot = ((peak - target) / abs(span) * 100.0) if abs(span) > 1e-6 else 0.0
    overshoot_rpm = max(0.0, (peak - target) if span >= 0 else (target - peak))
    # IAE excluding the warmup window
    iae = 0.0
    for t_, y, _ in seg:
        if t_ >= warmup:
            iae += abs(y - target)
    iae *= DT
    # 10-90 rise (only for a rising step)
    rise = None
    if span > 1e-6:
        lo, hi = y0 + 0.1 * span, y0 + 0.9 * span
        t_lo = t_hi = None
        for t_, y in zip(ts, ysm):
            if t_lo is None and y >= lo:
                t_lo = t_
            if t_hi is None and y >= hi:
                t_hi = t_
                break
        if t_lo is not None and t_hi is not None:
            rise = t_hi - t_lo
    ripple = statistics.pstdev(tail) if len(tail) > 2 else 0.0
    return {"target": target, "steady": round(steady, 1),
            "steady_err": round(steady - target, 1), "overshoot_pct": round(overshoot, 1),
            "overshoot_rpm": round(overshoot_rpm, 1),
            "rise_s": (round(rise, 3) if rise is not None else None),
            "iae": round(iae, 1), "ripple": round(ripple, 1)}


def schedule_cost(segments, rows):
    """Scalar tuning cost: mean IAE + overshoot penalty + steady-error penalty + control jitter."""
    if not segments:
        return 1e9
    iae = statistics.mean(s.get("iae", 0.0) for s in segments)
    ov = statistics.mean(s.get("overshoot_rpm", 0.0) for s in segments)
    se = statistics.mean(abs(s.get("steady_err", 0.0)) for s in segments)
    # control jitter: mean abs diff of thrust between consecutive ticks (chattering penalty)
    jit = 0.0
    if len(rows) > 2:
        jit = statistics.mean(abs(rows[i]["thrust"] - rows[i - 1]["thrust"])
                              for i in range(1, len(rows)))
    return iae + 1.5 * ov + 3.0 * se + 0.15 * jit


# ---------------------------------------------------------------------------
# model-seeded IMC / lambda tuning
# ---------------------------------------------------------------------------
def imc_seed(model, lam_frac=1.0):
    """PI seed from a first-order+delay plant (K rpm/cmd, tau, delay). Returns (kp, ki).
    IMC-PI: Kc = tau / (K*(lambda+L)); Ti = tau; lambda = lam_frac*tau (closed-loop speed)."""
    dyn = model.get("dynamic", {}) if model else {}
    K = dyn.get("K_rpm_per_cmd_median") or 0.35
    tau = dyn.get("tau_s_median") or 0.18
    L = dyn.get("delay_s_median") or 0.04
    lam = max(0.05, lam_frac * tau)
    Kc = tau / (max(1e-3, K) * (lam + L))         # cmd per rpm
    Ti = tau
    ki = Kc / Ti
    return round(Kc, 3), round(ki, 3)


# ---------------------------------------------------------------------------
# auto-tune (coordinate search)
# ---------------------------------------------------------------------------
def autotune(esc, clock, profile, model, opts):
    kp0, ki0 = imc_seed(model, opts.lam_frac)
    kd0 = 0.0
    log(f"[tune] IMC seed: kp={kp0} ki={ki0} (lambda={opts.lam_frac}*tau)")
    params = {"kp": kp0, "ki": ki0, "kd": kd0}

    def evaluate(p):
        loop = VelLoop(profile, p["kp"], p["ki"], p["kd"], opts.slew, opts.meas_tau, opts.tmax,
                       fb_floor=opts.fb_floor, fb_band=opts.fb_band, err_band=opts.err_band)
        rows, segs = run_schedule(esc, clock, loop, opts.targets, opts.hold, opts.enc_sign,
                                  opts.max_rpm)
        c = schedule_cost(segs, rows)
        esc.thrust(0)
        clock.sleep(opts.cooldown)
        return c, rows, segs

    best_c, best_rows, best_segs = evaluate(params)
    log(f"[tune] seed cost={best_c:.1f}")
    # coordinate descent: multiply each gain by factors, keep improvements
    steps = [("kp", 1.5), ("kp", 0.67), ("ki", 1.6), ("ki", 0.6),
             ("kd", None), ("kp", 1.25), ("ki", 1.25)]
    for rnd in range(opts.rounds):
        improved = False
        for key, fac in steps:
            trial = dict(params)
            if key == "kd" and fac is None:
                # kd is additive from 0: try a few absolute values scaled to kp
                trial["kd"] = round(0.3 * params["kp"] * (rnd + 1), 3)
            else:
                trial[key] = round(params[key] * fac, 3)
            if trial == params or trial[key] < 0:
                continue
            c, rows, segs = evaluate(trial)
            tag = f"kp={trial['kp']} ki={trial['ki']} kd={trial['kd']}"
            if c < best_c - 0.5:
                best_c, best_rows, best_segs, params = c, rows, segs, trial
                improved = True
                log(f"[tune] r{rnd} {tag} cost={c:.1f}  <-- keep")
            else:
                log(f"[tune] r{rnd} {tag} cost={c:.1f}")
        if not improved:
            log(f"[tune] round {rnd}: no improvement, stopping")
            break
    log(f"[tune] BEST kp={params['kp']} ki={params['ki']} kd={params['kd']} cost={best_c:.1f}")
    return params, best_rows, best_segs


# ---------------------------------------------------------------------------
# io
# ---------------------------------------------------------------------------
def write_csv(path, rows):
    with open(path, "w", encoding="utf-8", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "seg", "target", "sp", "meas", "thrust"])
        for r in rows:
            w.writerow([round(r["t"], 4), r["seg"], round(r["target"], 1),
                        round(r["sp"], 1), round(r["meas"], 1), round(r["thrust"], 1)])


def print_segments(segments):
    log(f"  {'target':>7} {'steady':>7} {'err':>6} {'over%':>6} {'overrpm':>7} "
        f"{'rise':>6} {'iae':>7} {'ripple':>6}")
    for s in segments:
        log(f"  {s['target']:>7.0f} {s.get('steady', 0):>7.1f} {s.get('steady_err', 0):>6.1f} "
            f"{s.get('overshoot_pct', 0):>6.1f} {s.get('overshoot_rpm', 0):>7.1f} "
            f"{str(s.get('rise_s')):>6} {s.get('iae', 0):>7.1f} {s.get('ripple', 0):>6.1f}")


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def run(opts):
    os.makedirs(REPORT_DIR, exist_ok=True)
    profile = SpeedProfile.load(opts.profile)
    model = json.load(open(opts.model)) if opts.model and os.path.exists(opts.model) else None
    ts = time.strftime("%Y%m%d-%H%M%S")
    host, clock = open_host(opts)
    esc = ESC(host, opts.esc, tmax=opts.tmax, clock=clock)
    try:
        esc.prepare()
        log(f"# arming ESC {opts.esc} (bidir) tmax={opts.tmax} enc_sign={opts.enc_sign}")
        esc.arm(bidir=True)

        if opts.cmd == "tune":
            params, rows, segs = autotune(esc, clock, profile, model, opts)
            log("")
            log("=== tuned result ===")
            print_segments(segs)
            path = os.path.join(REPORT_DIR, f"veltune_tuned_{opts.name}_{ts}.csv")
            write_csv(path, rows)
            log(f"  gains: kp={params['kp']} ki={params['ki']} kd={params['kd']} "
                f"slew={opts.slew} meas_tau={opts.meas_tau}")
            log(f"  trace: {path}")
            # persist gains next to the profile for reuse
            gpath = os.path.join(REPORT_DIR, f"veltune_gains_{opts.name}_{ts}.json")
            json.dump({"profile": opts.profile, "gains": params, "slew": opts.slew,
                       "meas_tau": opts.meas_tau, "targets": opts.targets,
                       "segments": segs}, open(gpath, "w"), indent=2)
            log(f"  gains json: {gpath}")
        else:
            loop = VelLoop(profile, opts.kp, opts.ki, opts.kd, opts.slew, opts.meas_tau, opts.tmax,
                           fb_floor=opts.fb_floor, fb_band=opts.fb_band, err_band=opts.err_band)
            rows, segs = run_schedule(esc, clock, loop, opts.targets, opts.hold, opts.enc_sign,
                                      opts.max_rpm)
            log("=== run metrics ===")
            print_segments(segs)
            path = os.path.join(REPORT_DIR, f"veltune_run_{opts.name}_{ts}.csv")
            write_csv(path, rows)
            log(f"  cost={schedule_cost(segs, rows):.1f}  trace: {path}")
    except Aborted as e:
        log(f"!! ABORTED: {e}")
    except KeyboardInterrupt:
        log("!! interrupted")
    finally:
        try:
            esc.disarm()
        except Exception:
            pass
        if not opts.dry_run:
            try:
                host.close()
            except Exception:
                pass


def _targets(s):
    return [float(x) for x in s.split(",") if x.strip()]


def build_parser():
    ap = argparse.ArgumentParser(description="encoder-feedback velocity PID + auto-tuner")
    ap.add_argument("--name", default="motor")
    ap.add_argument("--esc", type=int, default=1)
    ap.add_argument("--port")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--sim-invert", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--profile", required=True, help="SpeedProfile YAML (FF curve)")
    ap.add_argument("--model", help="sysid model JSON (for IMC seed)")
    ap.add_argument("--enc-sign", type=int, default=-1, choices=(-1, 1))
    ap.add_argument("--tmax", type=int, default=850)
    ap.add_argument("--max-rpm", type=float, default=400)
    ap.add_argument("--slew", type=float, default=600, help="setpoint rate limit rpm/s")
    ap.add_argument("--meas-tau", type=float, default=0.08, help="encoder low-pass s")
    ap.add_argument("--fb-floor", type=float, default=0.0,
                    help="regime gain-schedule: FB authority 0 below this rpm (near the crossover fold)")
    ap.add_argument("--fb-band", type=float, default=0.0,
                    help="rpm band over which FB ramps 0->1 above fb-floor (0 = FB always full)")
    ap.add_argument("--err-band", type=float, default=0.0,
                    help="FB-as-trim: FB authority ramps to 0 when |error| exceeds this (0 = off)")
    ap.add_argument("--hold", type=float, default=1.8, help="s per target")
    ap.add_argument("--cooldown", type=float, default=0.4)
    ap.add_argument("--lam-frac", type=float, default=1.0, help="IMC lambda = frac*tau")
    ap.add_argument("--rounds", type=int, default=3, help="coordinate-search rounds")
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("run", "tune"):
        sp = sub.add_parser(name)
        sp.add_argument("--targets", type=_targets, default=[120, 220, 160, 260],
                        help="comma RPM list")
        if name == "run":
            sp.add_argument("--kp", type=float, default=1.0)
            sp.add_argument("--ki", type=float, default=5.0)
            sp.add_argument("--kd", type=float, default=0.0)
    return ap


def main(argv=None):
    opts = build_parser().parse_args(argv)
    run(opts)


if __name__ == "__main__":
    main()
