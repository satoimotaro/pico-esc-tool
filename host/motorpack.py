#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""motorpack — drive a motor from just its KV + pole-pairs (+ supply V), no full bench measurement.

Uses the parametric pico_esc.motor_model: KV+PP+V predict the thrust->rpm curve, the regime boundaries,
and the ESC crossover config. A light eRPM self-cal (bidir-DShot `tele`, no encoder) fits the residual
scale for THIS motor/supply. Generic across KV (350KV, 1000KV, ...).

  predict  — offline: print the model, emit a SpeedProfile YAML + the esctool config command.
  selfcal  — hardware: spin a few 6-step points, read `tele` eRPM, fit `eff`, re-emit the profile.
  apply    — hardware: push the derived ESC config (esctool set).

Usage (shared args BEFORE the subcommand):
  python3 motorpack.py --kv 350 --pp 7 --v 11.1 --name f2838_350kv predict
  python3 motorpack.py --kv 350 --pp 7 --v 11.1 --name f2838_350kv selfcal     # refine eff on hardware
  python3 motorpack.py --kv 350 --pp 7 --v 11.1 apply                           # set the ESC config
"""
from __future__ import annotations

import argparse
import os

from pico_esc.motor_model import MotorModel

PROF_DIR = os.path.join(os.path.dirname(__file__), "profiles")


def _esctool_cmd(cfg, idx=1):
    kv = " ".join(f"{k}={v}" for k, v in cfg.items())
    return f"python3 esctool.py set {idx} {kv}"


def cmd_predict(m, opts):
    print(m.summary())
    path = os.path.join(PROF_DIR, f"{m.name}_parametric.yaml")
    m.to_profile().save(path)
    print(f"\n  profile: {path}")
    print(f"  apply ESC config:\n    {_esctool_cmd(m.esc_config())}")


def cmd_apply(m, opts):
    import subprocess, sys
    cfg = m.esc_config()
    print(f"# applying derived ESC config for {m.name}:\n  {cfg}")
    subprocess.run([sys.executable, "esctool.py", "set", str(opts.esc)]
                   + [f"{k}={v}" for k, v in cfg.items()], cwd=os.path.dirname(__file__))


def cmd_selfcal(m, opts):
    """Spin a few 6-step points, read tele eRPM, fit eff. No encoder needed."""
    from pico_esc.esc import ESC
    from pico_esc.link import EscHost, RealClock
    from pico_esc.control import DT, _pace
    host, clock = EscHost(opts.port), RealClock()
    esc = ESC(host, opts.esc, tmax=opts.tmax, clock=clock)
    samples = []
    try:
        esc.prepare()
        print(f"# self-cal: arming, sampling tele eRPM in 6-step (enc_sign {opts.enc_sign})")
        esc.arm(bidir=True)
        for cmd in opts.cal_cmds:
            cmd = int(cmd)                          # [#6.5] --cal-cmds parses floats; {cmd:4d} + int thrust need int
            t_end = clock.now() + 2.5
            erpms = []
            while clock.now() < t_end:
                tick = clock.now()
                if clock.now() > t_end - 1.0:
                    tel = esc.telemetry()
                    if tel is not None and abs(tel.rpm) > 200:
                        erpms.append(abs(tel.rpm) * m.pp)   # tele.rpm is mech; *pp -> eRPM
                esc.thrust(opts.enc_sign * cmd)
                _pace(clock, tick)
            if erpms:
                erpm = sum(erpms) / len(erpms)
                samples.append((cmd, erpm))
                print(f"  cmd {cmd:4d} -> tele {erpm/m.pp:.0f} mech ({erpm:.0f} eRPM)")
            esc.thrust(0)
            clock.sleep(0.4)
    finally:
        try:
            esc.disarm()
        except Exception:
            pass
        host.close()
    if samples:
        eff = m.self_cal_erpm(samples)
        print(f"# fitted eff = {eff:.3f}")
        path = os.path.join(PROF_DIR, f"{m.name}_selfcal.yaml")
        m.to_profile().save(path)
        print(f"# refined profile: {path}")
    else:
        print("# no 6-step tele samples — is the motor reaching 6-step? check the config/cal_cmds")


def _floats(s):
    return [float(x) for x in s.split(",") if x.strip()]


def main(argv=None):
    ap = argparse.ArgumentParser(description="drive a motor from KV + pole-pairs (parametric model)")
    ap.add_argument("--kv", type=float, required=True)
    ap.add_argument("--pp", type=int, required=True, help="pole pairs (poles/2)")
    ap.add_argument("--v", type=float, default=11.1, help="supply voltage")
    ap.add_argument("--eff", type=float, default=1.0, help="efficiency scale (self-cal fits this)")
    ap.add_argument("--name", default=None)
    ap.add_argument("--esc", type=int, default=1)
    ap.add_argument("--port")
    ap.add_argument("--tmax", type=int, default=700)
    ap.add_argument("--enc-sign", type=int, default=-1, choices=(-1, 1))
    ap.add_argument("--cal-cmds", type=_floats, default=[550, 620, 690],
                    help="6-step cmds to sample for self-cal")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("predict")
    sub.add_parser("apply")
    sub.add_parser("selfcal")
    opts = ap.parse_args(argv)
    m = MotorModel(opts.kv, opts.pp, opts.v, eff=opts.eff,
                   name=opts.name or f"{int(opts.kv)}kv")
    {"predict": cmd_predict, "apply": cmd_apply, "selfcal": cmd_selfcal}[opts.cmd](m, opts)


if __name__ == "__main__":
    main()
