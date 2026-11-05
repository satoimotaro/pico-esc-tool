#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""duty_sweep — measure the firmware's effective duty(cmd) map directly, at no load.

WHY. `vest = rpm / (KV * duty_eff(cmd))` and the firmware models `duty_eff` as ONE fixed affine
(motor_model.h: 0.00265*cmd - 1.233) with **no cross_up term** — while the ESC's `cross_rescale_duty`
maps the 6-step demand relative to `Cross_Up`. That mismatch is the leading suspect for both the
`vest` bias and the "the FF curve is only valid at the cross_up it was measured at" problem.

At NO LOAD the I*R term nearly vanishes, so the relation inverts cleanly and duty_eff becomes
directly measurable from rpm alone. No load is the IDEAL condition here, not a limitation.

  V_supply * duty = rpm/KV + I_phase * R   and   I_dc = I_phase * duty
  =>  V*d^2 - (rpm/KV)*d - I_dc*R = 0
  =>  duty = [ E + sqrt(E^2 + 4*V*I_dc*R) ] / (2V),   E = rpm/KV

so a PSU current reading per point removes even the small residual I*R error (~2.6 % at 1600 rpm on
the bench, growing as rpm^2). With no current reading it degrades gracefully to duty = E/V.

Run the SAME sweep at two `sine_cross_up` values and the pair gives the affine law empirically.

Open loop on purpose: `thrust` (a fixed COMMAND), never `rpm` — the closed loop would move cmd to
hold speed, which is exactly the variable being swept. Raw mode needs a keep-alive (500 ms deadman,
esc_session.h) and does NOT update `sense`, so the 6-step lock is checked via tele-vs-encoder
agreement (they track to ~0.5 % once BEMF-locked; in sine the virtual eRPM does not).

  # 1) drive the sweep, note the PSU current at each step as it holds
  python3 host/duty_sweep.py 1 --dwell 20

  # 2) fold the readings in afterwards — no motor run needed
  python3 host/duty_sweep.py --analyze host/reports/duty_sweep_<ts>.csv \
      --currents 0.121,0.108,0.095,0.083,0.072,0.061
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from esctool import EscHost, find_pico            # noqa: E402

# The firmware's model — imported, not re-declared, so this tool can never drift from what it validates.
from pico_esc.motor_model import DUTY6_A, DUTY6_B, duty6   # noqa: E402
KEEPALIVE_S = 0.2                                  # < the 500 ms raw-throttle deadman


class Aborted(RuntimeError):
    pass


def duty_model(cmd, cross_up):
    """The model's prediction INCLUDING its cross_up term, so the printed ratio is what the firmware
    would actually now compute — not what it computed before the term existed.

    `cross_up` is REQUIRED. It used to fall back to DUTY6_CROSSUP_REF, which in a tool whose entire
    purpose is to MEASURE the cross_up term meant the reference row silently compared the bench against
    a different ESC config than the one it was running."""
    return duty6(cmd, cross_up)


def duty_measured(rpm, kv, v_supply, i_dc=None, r=None):
    """Effective duty from the winding equation. i_dc = MOTOR-only bus current (idle subtracted)."""
    e = abs(rpm) / kv
    if not i_dc or not r:
        return e / v_supply                        # no-load approximation: I*R -> 0
    return (e + math.sqrt(e * e + 4.0 * v_supply * i_dc * r)) / (2.0 * v_supply)


def _tele(dev, idx):
    try:
        lines = dev.cmd(f"tele {idx}", timeout=3)
    except Exception:
        return None
    for ln in lines:
        if ln.startswith("tele|"):
            try:
                p = ln.split("|")
                return {"rpm": float(p[1]), "tempC": float(p[4])}
            except (IndexError, ValueError):
                return None
    return None


def _encv(dev):
    try:
        lines = dev.cmd("encv", timeout=3)
    except Exception:
        return None
    for ln in lines:
        if ln.startswith("encv|"):
            try:
                return float(ln.split("|")[2])
            except (IndexError, ValueError):
                return None
    return None


def _locked(tele_rpm, enc_rpm, tol):
    """6-step BEMF lock: tele and the encoder agree in MAGNITUDE (the AS5600 sign is arbitrary)."""
    if tele_rpm is None or enc_rpm is None or abs(enc_rpm) < 1.0:
        return False
    return abs(abs(tele_rpm) / abs(enc_rpm) - 1.0) <= tol


def hold(dev, idx, cmd, secs, opts, collect_from=0.0):
    """Hold a raw command for `secs`, keep-alive'ing, and average the tail samples."""
    t0, next_read = time.time(), 0.0
    tele_s, enc_s, temp = [], [], float("nan")
    while True:
        el = time.time() - t0
        if el >= secs:
            break
        dev.cmd(f"thrust {idx} {cmd}", timeout=3)          # feed the 500 ms deadman
        if el >= next_read:
            t, e = _tele(dev, idx), _encv(dev)
            next_read = el + opts.read_period
            if t:
                temp = t["tempC"]
                if abs(t["rpm"]) > opts.rpm_ceiling or (e and abs(e) > opts.rpm_ceiling):
                    raise Aborted(f"overspeed at cmd {cmd}: tele={t['rpm']:.0f} enc={e}")
                if el >= collect_from:
                    tele_s.append(t["rpm"])
                    if e is not None:
                        enc_s.append(e)
        time.sleep(KEEPALIVE_S)
    mean = lambda v: sum(v) / len(v) if v else float("nan")      # noqa: E731
    return mean(tele_s), mean([abs(x) for x in enc_s]), temp, len(tele_s)


def ramp_in(dev, idx, target, opts):
    """Ramp up through the sine->6-step handoff to `target`, then confirm the BEMF lock."""
    print(f"# ramping 100 -> {target} to cross the sine->6-step handoff")
    cmd = 100
    while cmd < target:
        cmd = min(target, cmd + opts.ramp_step)
        dev.cmd(f"thrust {idx} {cmd}", timeout=3)
        time.sleep(KEEPALIVE_S)
    t_end = time.time() + opts.lock_timeout
    while time.time() < t_end:
        dev.cmd(f"thrust {idx} {target}", timeout=3)
        t, e = _tele(dev, idx), _encv(dev)
        # The overspeed abort used to live only in hold(): a motor that never reports a lock sat at the
        # full sweep command for the whole --lock-timeout with nothing watching it.
        for lbl, r in (("tele", t["rpm"] if t else None), ("enc", e)):
            if r is not None and abs(r) > opts.rpm_ceiling:
                raise Aborted(f"overspeed on {lbl} during ramp-in: {r:.0f} > {opts.rpm_ceiling:.0f}")
        if t and _locked(t["rpm"], e, opts.lock_tol):
            print(f"# 6-step lock at cmd {target}: tele={t['rpm']:.0f} enc={abs(e):.0f} "
                  f"(ratio {abs(t['rpm']) / abs(e):.4f})")
            return
        time.sleep(KEEPALIVE_S)
    raise Aborted(f"no 6-step lock at cmd {target} within {opts.lock_timeout:g}s "
                  "(tele and encoder disagree -> still in sine)")


def sweep(dev, opts, state):
    idx = opts.index
    cmds = sorted(opts.cmds, reverse=True)          # descend: entered from above, stays in 6-step
    print(f"# cmds (descending): {cmds}")
    dev.cmd(f"arm {idx} bidir", timeout=8)
    time.sleep(3.0)
    ramp_in(dev, idx, cmds[0], opts)

    # Wall-clock + elapsed stamps on every step boundary: the operator films the PSU display, so the
    # log has to be alignable to the video after the fact rather than read live.
    t_start = time.time()
    print(f"\n# T0 = {time.strftime('%H:%M:%S', time.localtime(t_start))} "
          f"(sweep start — align the video from here)")
    for n, cmd in enumerate(cmds, 1):
        now = time.time()
        print(f"\n=== [{n}/{len(cmds)}] cmd {cmd} — holding {opts.dwell:g}s | "
              f"clock {time.strftime('%H:%M:%S', time.localtime(now))} | "
              f"T+{now - t_start:6.1f}s ===")
        tele, enc, temp, ns = hold(dev, idx, cmd, opts.dwell, opts,
                                   collect_from=opts.dwell * 0.4)
        locked = _locked(tele, enc, opts.lock_tol)
        state["rows"].append({"cmd": cmd, "tele_rpm": tele, "enc_rpm": enc,
                              "tempC": temp, "samples": ns, "locked": int(locked)})
        ratio = abs(tele / enc) if enc else float("nan")
        print(f"    tele={tele:7.1f}  enc={enc:7.1f}  ratio={ratio:.4f}  "
              f"{'LOCKED' if locked else '** NOT LOCKED (sine?) — point unusable **'}  {temp:.0f}C")
        if not locked and opts.stop_on_unlock:
            raise Aborted(f"lost the 6-step lock at cmd {cmd}; descending further is pointless")


def analyze(rows, opts):
    cur = list(opts.currents) if opts.currents else []
    if cur and len(cur) != len(rows):
        print(f"# WARNING: {len(cur)} currents for {len(rows)} points — pairing in row order, "
              "extra/missing treated as unknown")
    print("\n=== duty_eff(cmd): measured vs the firmware's fixed affine ===")
    print(f"# KV={opts.kv:g}  V_supply={opts.v:g}  R={opts.r:g}  "
          f"idle={opts.idle:g}A  cross_up={opts.cross_up}")
    if not cur:
        print("# NO currents given -> I*R ignored (duty = rpm/(KV*V)); "
              "the result is biased LOW, worst at high rpm")
    print(f"\n{'cmd':>5} {'rpm':>8} {'I_dc':>7} {'duty_meas':>10} {'duty_fw':>8} "
          f"{'fw/meas':>8} {'V_eff':>7}")
    out = []
    for i, r in enumerate(rows):
        rpm = r["enc_rpm"] if r.get("enc_rpm") == r.get("enc_rpm") else r["tele_rpm"]
        i_dc = None
        if i < len(cur):
            i_dc = max(0.0, cur[i] - opts.idle)        # PSU reading minus the ESC/logic idle draw
        d_meas = duty_measured(rpm, opts.kv, opts.v, i_dc, opts.r)
        d_fw = duty_model(r["cmd"], opts.cross_up)
        veff = abs(rpm) / (opts.kv * d_meas) if d_meas > 0 else float("nan")
        ratio = d_fw / d_meas if d_meas > 0 else float("nan")
        out.append({**r, "i_dc_motor": i_dc, "duty_meas": d_meas, "duty_fw": d_fw,
                    "fw_over_meas": ratio, "v_eff": veff})
        print(f"{r['cmd']:>5} {abs(rpm):8.1f} "
              f"{('%7.4f' % i_dc) if i_dc is not None else '      -'} "
              f"{d_meas:10.4f} {d_fw:8.4f} {ratio:8.3f} {veff:7.2f}")

    ok = [o for o in out if o["duty_meas"] > 0 and o.get("locked", 1)]
    if len(ok) >= 2:
        # Least-squares affine fit of the MEASURED duty, directly comparable to DUTY6_A/DUTY6_B.
        n = len(ok)
        sx = sum(o["cmd"] for o in ok); sy = sum(o["duty_meas"] for o in ok)
        sxx = sum(o["cmd"] ** 2 for o in ok); sxy = sum(o["cmd"] * o["duty_meas"] for o in ok)
        den = n * sxx - sx * sx
        if abs(den) > 1e-9:
            a = (n * sxy - sx * sy) / den
            b = (sy - a * sx) / n
            print(f"\nmeasured affine : duty = {a:.6f}*cmd + {b:.4f}")
            print(f"firmware affine : duty = {DUTY6_A:.6f}*cmd + {DUTY6_B:.4f}")
            print(f"slope ratio fw/meas = {DUTY6_A / a:.3f}" if a else "")
        rs = [o["fw_over_meas"] for o in ok if o["fw_over_meas"] == o["fw_over_meas"]]
        if rs:
            print(f"\nfw/measured duty over the sweep: {min(rs):.3f} .. {max(rs):.3f} "
                  f"(mean {sum(rs) / len(rs):.3f})")
            print("A ratio that is CONSTANT means one scalar (vcal) can fix it; a ratio that VARIES "
                  "with cmd means the affine itself is wrong and no single scalar will do.")
    return out


def main():
    ap = argparse.ArgumentParser(description="measure duty_eff(cmd) at no load")
    ap.add_argument("index", type=int, nargs="?", default=1)
    ap.add_argument("--cmds", default="780,740,700,660,620,580",
                    help="commands to hold, any order (swept descending; default stays inside the "
                         "measured curve's 6-step segment, cmd 558..780 = 628..2921 rpm)")
    ap.add_argument("--dwell", type=float, default=20.0, help="s to hold each point (default 20)")
    ap.add_argument("--read-period", type=float, default=0.5,
                    help="s between tele+encv reads (default 0.5; lower it for short dwells so the "
                         "tail average still has samples)")
    ap.add_argument("--kv", type=float, default=350.0)
    ap.add_argument("--v", type=float, default=11.2, help="supply voltage at the PSU")
    ap.add_argument("--r", type=float, default=0.7, help="line-to-line winding resistance, ohm")
    ap.add_argument("--idle", type=float, default=0.0055,
                    help="ESC+logic idle current to subtract from each PSU reading (default 0.0055)")
    ap.add_argument("--currents", default=None,
                    help="comma-separated PSU readings, in the swept (descending) order")
    ap.add_argument("--cross-up", type=int, required=True,
                    help="the sine_cross_up the ESC is actually set to (read it with `esctool.py "
                         "dump`). REQUIRED: it is stamped into the CSV header and it is what the "
                         "model column is computed at — two bare CSVs taken at different cross_up "
                         "are indistinguishable afterwards")
    ap.add_argument("--rpm-ceiling", type=float, default=3000.0)
    ap.add_argument("--lock-tol", type=float, default=0.05, help="|tele/enc - 1| for a 6-step lock")
    ap.add_argument("--lock-timeout", type=float, default=25.0)
    ap.add_argument("--ramp-step", type=int, default=25, help="cmd increment per 200 ms on entry")
    ap.add_argument("--stop-on-unlock", action="store_true",
                    help="abort as soon as a point is not BEMF-locked")
    ap.add_argument("--analyze", default=None, help="re-analyze an existing CSV; no hardware")
    ap.add_argument("--out", default=None)
    ap.add_argument("--port", default=None)
    ap.add_argument("--dry-run", action="store_true")
    opts = ap.parse_args()
    opts.cmds = [int(x) for x in opts.cmds.split(",") if x.strip()]
    opts.currents = [float(x) for x in opts.currents.split(",")] if opts.currents else None

    if opts.analyze:
        with open(opts.analyze) as f:
            head = [ln for ln in f if ln.startswith("#")]
            f.seek(0)
            body = (ln for ln in f if not ln.startswith("#"))
            rows = [{k: (float(v) if v not in ("", None) else float("nan"))
                     for k, v in r.items()} for r in csv.DictReader(body)]
        for ln in head:
            print(f"# recorded conditions: {ln.lstrip('# ').rstrip()}")
        for r in rows:
            r["cmd"] = int(r["cmd"])
        analyze(rows, opts)
        return 0

    if opts.dry_run:
        print(f"# DRY RUN — ESC {opts.index}, open-loop `thrust` sweep at no load")
        print(f"#   cmds (descending): {sorted(opts.cmds, reverse=True)}, {opts.dwell:g}s each")
        print(f"#   ramp in to {max(opts.cmds)}, require tele/enc agreement within "
              f"{opts.lock_tol:.0%}")
        print(f"#   aborts: |rpm| > {opts.rpm_ceiling:g}; always disarms")
        print(f"#   total motor-on time ~ {len(opts.cmds) * opts.dwell + 30:.0f}s")
        return 0

    out = opts.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), "reports",
                                   f"duty_sweep_{time.strftime('%Y%m%d_%H%M%S')}.csv")
    os.makedirs(os.path.dirname(out), exist_ok=True)

    port = find_pico(opts.port)
    print(f"# port {port}")
    dev = EscHost(port)
    state, status = {"rows": []}, "completed"
    try:
        dev.cmd("run", timeout=6)
        dev.cmd("disconnect", timeout=6)
        time.sleep(0.4)
        sweep(dev, opts, state)
    except Aborted as e:
        status = f"ABORTED: {e}"
        print(f"\n!! {status}")
    except KeyboardInterrupt:
        status = "interrupted by user"
        print(f"\n!! {status}")
    finally:
        # Each step guarded INDEPENDENTLY: one shared try meant a benign first failure (e.g. `thrust`
        # returning err when already disarmed) skipped the disarm that actually matters.
        for step in (f"thrust {opts.index} 0", f"disarm {opts.index}"):
            try:
                dev.cmd(step, timeout=6)
            except Exception as e:
                print(f"# WARNING: '{step}' failed ({e})")
            time.sleep(0.5)
        print("# stopped, disarmed")

    rows = state["rows"]
    if rows:
        keys = ["cmd", "tele_rpm", "enc_rpm", "tempC", "samples", "locked"]
        with open(out, "w", newline="") as f:
            # The conditions BELONG in the file: the whole point of this tool is differencing two sweeps
            # taken at different cross_up, and two bare CSVs are indistinguishable afterwards. Pairing
            # them the wrong way round flips the sign of the fitted coefficient, which then gets
            # compiled into firmware.
            f.write(f"# cross_up={opts.cross_up} kv={opts.kv:g} v={opts.v:g} r={opts.r:g} "
                    f"idle={opts.idle:g} dwell={opts.dwell:g}\n")
            w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            w.writeheader()
            w.writerows(rows)
        print(f"\n# {len(rows)} points -> {out}")
        analyze(rows, opts)
        print(f"\n# fold in the PSU readings with:\n"
              f"#   python3 host/duty_sweep.py --analyze {out} --currents <a,b,c,...>")
    print(f"# status: {status}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
