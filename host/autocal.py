#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""autocal — host auto-calibration for thruster ESCs on the RP2040 esc_tool protocol.

Automates the per-thruster low-speed tuning that BLHeli configurators don't:
  * direction     — confirm bidirectional spin direction / telemetry sign
  * coldstart     — bisect the minimum reliable cold-start throttle (k-of-k)
  * minrpm        — descend throttle to find the minimum sustainable RPM
  * curve         — sweep throttle -> RPM and build a linearization table
  * tune-startup  — bisect startup_power_max / startup_power_min for reliable starts
  * tune-smooth   — grid comm_timing x demag, score by RPM variance (smoothest)
  * all           — the full pipeline, in order

Talks to the ESC through the same keep-alive drive pattern as drive_hold.py
(throttle re-sent every 200 ms to beat the firmware's 500 ms spin deadman) and the
same config path as esctool.py (editpage between trials, motor stopped).

Safety:
  * every run disarms in a finally block and on SIGINT/SIGTERM,
  * a single throttle-ceiling choke point clamps every throttle send,
  * standstill-required phases refuse to run if the motor is turning,
  * --dry-run drives a built-in SimEscHost and NEVER opens a serial port.

  python autocal.py all --dry-run --esc-index 1
  python autocal.py coldstart --esc-index 1 --max-throttle 500
"""
from __future__ import annotations

import argparse
import csv
import os
import signal
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import esctool  # noqa: E402  (esctool lives next to this file)
from esctool import encode_overrides, overrides_str  # noqa: E402
# SimEscHost (the dry-run motor model), the Tele sample, and ARM_WAIT now live in the package;
# re-exported here so `from autocal import SimEscHost, ARM_WAIT` (posctl.py) keeps working.
from pico_esc.sim import SimEscHost  # noqa: E402,F401
from pico_esc.types import Tele  # noqa: E402,F401
from pico_esc.drive import ARM_WAIT  # noqa: E402,F401

HERE = os.path.dirname(os.path.abspath(__file__))
PROFILE_DIR = os.path.join(HERE, "profiles")
REPORT_DIR = os.path.join(HERE, "reports")

class CalibrationError(RuntimeError):
    """Raised when a phase cannot produce a trustworthy result (e.g. motor never
    started anywhere in range). Never emit a fabricated value in this case."""


KA = 0.2                    # keep-alive throttle resend period (< 500 ms deadman)
TELE_EVERY = 3              # poll telemetry every N keep-alive ticks (~0.6 s)
SPIN_RPM = 100             # mechanical rpm above which the motor is "spinning"
CONFIRM_THROTTLE = 800     # max_throttle above this needs interactive confirmation


# ---------------------------------------------------------------------------
# Drive session: keep-alive throttle + telemetry, always disarms.
# ---------------------------------------------------------------------------
class DriveSession:
    """Wraps a host (EscHost or SimEscHost) with the drive_hold.py keep-alive pattern.

    All throttle sends funnel through _send_throttle -> single ceiling choke point.
    """

    def __init__(self, host, idx, max_throttle, sleep=time.sleep, verbose=True):
        self.host = host
        self.idx = idx
        self.max_throttle = max_throttle
        self.sleep = sleep
        self.verbose = verbose
        self.armed = False

    # -- lifecycle --
    def prepare(self):
        # end any held bootloader session so the app is running before we arm
        try:
            self.host.cmd("run", timeout=5)
            self.host.cmd("disconnect", timeout=5)
        except Exception:
            pass
        self.sleep(0.4)

    def arm(self):
        self._log("# arming (bidir)…")
        self.host.cmd(f"arm {self.idx} bidir", timeout=6)
        self.armed = True
        self.sleep(ARM_WAIT)

    def disarm(self):
        if not self.armed:
            return
        for _ in range(3):
            try:
                self._send_throttle(0)          # through the single ceiling choke point
            except Exception:
                pass
        for cmd in (f"disarm {self.idx}", "disarm"):
            try:
                self.host.cmd(cmd, timeout=2)
            except Exception:
                pass
        self.armed = False
        self._log("# DISARMED")

    # -- throttle: the ONLY place a throttle value is sent to the ESC --
    def _send_throttle(self, thr):
        thr = int(thr)
        if thr < 0:
            thr = 0
        if thr > self.max_throttle:          # single throttle-ceiling choke point
            thr = self.max_throttle
        self.host.cmd(f"throttle {self.idx} {thr}", timeout=2)
        return thr

    def read_tele(self):
        for ln in self.host.cmd(f"tele {self.idx}", timeout=2):
            if ln.startswith("tele|"):
                p = ln.split("|")
                try:
                    return Tele(int(p[1]), float(p[2]), int(p[3]), int(p[4]), int(p[5]))
                except (ValueError, IndexError):
                    return None
        return None

    def current_rpm(self):
        t = self.read_tele()
        return t.rpm if t else 0

    def hold(self, thr, secs):
        """Keep-alive hold at `thr` for `secs`; returns list[Tele] samples."""
        ticks = max(1, round(secs / KA))
        samples = []
        for i in range(ticks):
            self._send_throttle(thr)
            if i % TELE_EVERY == 0:
                t = self.read_tele()
                if t:
                    samples.append(t)
            self.sleep(KA)
        t = self.read_tele()
        if t:
            samples.append(t)
        return samples

    def stop_and_confirm_stopped(self, settle=1.0, tries=10):
        """Command zero throttle and wait until telemetry reports a stopped motor."""
        for _ in range(tries):
            self._send_throttle(0)
            self.sleep(settle / tries + KA)
            if self.current_rpm() <= 5:
                return True
        return False

    def assert_standstill(self):
        rpm = self.current_rpm()
        if rpm > SPIN_RPM:
            raise RuntimeError(f"phase requires standstill but motor is turning ({rpm} rpm)")

    def _log(self, msg):
        if self.verbose:
            print(msg)


# ---------------------------------------------------------------------------
# Config session: set params between trials (motor stopped), reusing esctool.
# ---------------------------------------------------------------------------
class ConfigSession:
    def __init__(self, host, idx, sleep=time.sleep):
        self.host = host
        self.idx = idx
        self.sleep = sleep

    def set(self, settings):
        ovs = encode_overrides(settings)
        if not ovs:
            return
        self.host.cmd(f"editpage {self.idx} {overrides_str(ovs)}", timeout=30)

    def restart(self):
        # editpage leaves the ESC in the bootloader; restart the app and let it boot
        try:
            self.host.cmd(f"run {self.idx}", timeout=10)
        except Exception:
            pass
        self.sleep(1.0)


# ---------------------------------------------------------------------------
# Calibrator: the phases.
# ---------------------------------------------------------------------------
class Calibrator:
    def __init__(self, drive: DriveSession, config: ConfigSession, rows: list):
        self.d = drive
        self.c = config
        self.rows = rows            # CSV accumulator: list[dict]
        self.results = {}           # calibration outputs

    def _row(self, phase, **kw):
        self.rows.append({"phase": phase, **kw})

    # ---- config-trial helper: stop, reconfigure, restart, re-arm ----
    def apply_config(self, settings):
        if not self.d.stop_and_confirm_stopped():
            raise CalibrationError(
                "motor did not confirm stopped before reconfiguring — refusing to editpage")
        self.d.disarm()
        self.c.set(settings)
        self.c.restart()
        self.d.prepare()
        self.d.arm()

    # ---- phase: direction ----
    def direction(self, test_throttle=None):
        thr = test_throttle or min(120, self.d.max_throttle)
        self.d.assert_standstill()
        samples = self.d.hold(thr, 2.5)
        rpm = max((s.rpm for s in samples), default=0)
        ok = rpm > SPIN_RPM
        self.d.stop_and_confirm_stopped()
        self._row("direction", throttle=thr, rpm=rpm, ok=int(ok))
        self.results["direction"] = {"test_throttle": thr, "rpm": rpm, "spins_forward": bool(ok)}
        print(f"[direction] throttle={thr} -> rpm={rpm}  spins_forward={ok}")
        return ok

    # ---- phase: coldstart (bisect min start throttle, k-of-k) ----
    def coldstart(self, lo=40, hi=None, k=3, tol=6):
        hi = hi if hi is not None else min(300, self.d.max_throttle)
        print(f"[coldstart] bisecting min start throttle in [{lo},{hi}] (k={k})")
        # Validate the ceiling first — bisection assumes the top of the range starts.
        # If even the ceiling fails, the motor never started: raise, never fabricate.
        if not self._start_trials(hi, k):
            self._row("coldstart", throttle=hi, k=k, success=0)
            raise CalibrationError(
                f"cold start failed even at ceiling throttle {hi} — check motor/wiring/rotor")
        best = hi
        while hi - lo > tol:
            mid = (lo + hi) // 2
            ok = self._start_trials(mid, k)
            self._row("coldstart", throttle=mid, k=k, success=int(ok))
            print(f"  try {mid}: {'OK' if ok else 'fail'}")
            if ok:
                best = mid
                hi = mid
            else:
                lo = mid
        self.results["coldstart_throttle"] = best
        print(f"[coldstart] min reliable start throttle ~ {best}")
        return best

    def _start_trials(self, thr, k):
        for _ in range(k):
            self.d.assert_standstill()
            samples = self.d.hold(thr, 2.0)
            spun = max((s.rpm for s in samples), default=0) > SPIN_RPM
            self.d.stop_and_confirm_stopped()
            if not spun:
                return False
        return True

    # ---- phase: minrpm (descend to stall) ----
    def minrpm(self, start=None, step=10, floor=30):
        start = start or self.results.get("coldstart_throttle") or min(150, self.d.max_throttle)
        print(f"[minrpm] descending from {start} (step {step}) to stall")
        # get spinning first
        self.d.assert_standstill()
        self.d.hold(max(start, 120), 2.0)
        last_rpm, last_thr = 0, start
        thr = start
        while thr >= floor:
            samples = self.d.hold(thr, 1.6)
            rpm = int(statistics.median([s.rpm for s in samples])) if samples else 0
            self._row("minrpm", throttle=thr, rpm=rpm)
            print(f"  throttle {thr}: rpm={rpm}")
            if rpm <= SPIN_RPM:
                break
            last_rpm, last_thr = rpm, thr
            thr -= step
        self.d.stop_and_confirm_stopped()
        self.results["min_rpm"] = {"throttle": last_thr, "rpm": last_rpm}
        print(f"[minrpm] min sustainable ~ {last_rpm} rpm at throttle {last_thr}")
        return last_rpm

    # ---- phase: curve (sweep -> linearization table) ----
    def curve(self, points=8, lo=None, hi=None):
        lo = lo or (self.results.get("coldstart_throttle") or 60)
        hi = hi or self.d.max_throttle
        step = max(1, (hi - lo) // max(1, points - 1))
        levels = list(range(lo, hi + 1, step))
        print(f"[curve] sweeping {levels}")
        self.d.assert_standstill()
        self.d.hold(max(lo, 120), 2.0)       # ensure started
        table = []
        for thr in levels:
            samples = self.d.hold(thr, 1.6)
            rpm = int(statistics.median([s.rpm for s in samples])) if samples else 0
            table.append((thr, rpm))
            self._row("curve", throttle=thr, rpm=rpm)
            print(f"  throttle {thr}: rpm={rpm}")
        self.d.stop_and_confirm_stopped()
        # build rpm -> throttle linearization (evenly spaced rpm targets)
        table = [(t, r) for t, r in table if r > 0]
        lin = self._linearize(table)
        self.results["curve"] = {"throttle_rpm": [[t, r] for t, r in table],
                                 "linearization_rpm_to_throttle": lin}
        print(f"[curve] {len(table)} valid points; linearization has {len(lin)} entries")
        return table

    @staticmethod
    def _linearize(table, n=6):
        if len(table) < 2:
            return []
        rmin, rmax = table[0][1], table[-1][1]
        out = []
        for i in range(n):
            r = rmin + (rmax - rmin) * i / (n - 1)
            # find bracketing points and interpolate throttle
            thr = table[-1][0]
            for (t0, r0), (t1, r1) in zip(table, table[1:]):
                if r0 <= r <= r1 and r1 != r0:
                    thr = t0 + (t1 - t0) * (r - r0) / (r1 - r0)
                    break
            out.append([round(r), round(thr)])
        return out

    # ---- phase: tune-startup (bisect startup_power_max/min) ----
    def tune_startup(self, target_throttle=None):
        target = target_throttle or self.results.get("coldstart_throttle") or 80
        print(f"[tune-startup] finding startup_power_max for reliable start at throttle {target}")
        lo, hi = 20, 80
        # Validate the max power first; if even full startup power won't start reliably
        # at the target throttle, there is no valid answer — raise, never fabricate.
        self.apply_config({"startup_power_max": hi})
        if not self._start_trials(target, 3):
            self._row("tune_startup", startup_power_max=hi, success=0)
            raise CalibrationError(
                f"no startup_power_max in [{lo},{hi}] gives a reliable start at throttle {target}")
        best = hi
        while hi - lo > 5:
            mid = (lo + hi) // 2
            self.apply_config({"startup_power_max": mid})
            ok = self._start_trials(target, 3)
            self._row("tune_startup", startup_power_max=mid, success=int(ok))
            print(f"  startup_power_max={mid}: {'OK' if ok else 'fail'}")
            if ok:
                best, hi = mid, mid
            else:
                lo = mid
        self.results["startup_power_max"] = best
        print(f"[tune-startup] startup_power_max = {best}")
        return best

    # ---- phase: tune-smooth (grid comm_timing x demag, min rpm variance) ----
    def tune_smooth(self, test_throttle=None, timings=(1, 2, 3, 4, 5), demags=(1, 2, 3)):
        thr = test_throttle or min(200, self.d.max_throttle)
        print(f"[tune-smooth] grid comm_timing x demag @ throttle {thr} (min rpm variance)")
        best, best_score = None, None
        for ct in timings:
            for dm in demags:
                self.apply_config({"comm_timing": ct, "demag_compensation": dm})
                self.d.assert_standstill()
                self.d.hold(thr, 1.5)                    # warm up to steady state
                samples = self.d.hold(thr, 3.0)          # measure steady-state jitter
                rpms = [s.rpm for s in samples if s.rpm > SPIN_RPM]
                var = statistics.pvariance(rpms) if len(rpms) > 1 else float("inf")
                self.d.stop_and_confirm_stopped()
                self._row("tune_smooth", comm_timing=ct, demag=dm,
                          samples=len(rpms), rpm_variance=round(var, 2))
                print(f"  comm_timing={ct} demag={dm}: var={var:.1f} (n={len(rpms)})")
                if best_score is None or var < best_score:
                    best, best_score = (ct, dm), var
        self.results["smooth"] = {"comm_timing": best[0], "demag_compensation": best[1],
                                  "rpm_variance": round(best_score, 2)}
        print(f"[tune-smooth] best comm_timing={best[0]} demag={best[1]} (var={best_score:.1f})")
        return best


# ---------------------------------------------------------------------------
# Output writers
# ---------------------------------------------------------------------------
def write_reports(name, results, rows):
    os.makedirs(PROFILE_DIR, exist_ok=True)
    os.makedirs(REPORT_DIR, exist_ok=True)
    ts = time.strftime("%Y%m%d-%H%M%S")

    settings = {}
    if "startup_power_max" in results:
        settings["startup_power_max"] = results["startup_power_max"]
    if "smooth" in results:
        settings["comm_timing"] = results["smooth"]["comm_timing"]
        settings["demag_compensation"] = results["smooth"]["demag_compensation"]

    profile = {
        "identity": {"name": name},
        "settings": settings,        # apply-compatible (esctool apply reads this)
        "calibration": results,      # extra section, ignored by esctool apply
    }
    prof_path = os.path.join(PROFILE_DIR, f"{name}_autocal.yaml")
    with open(prof_path, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(esctool._emit_yaml(profile))

    csv_path = os.path.join(REPORT_DIR, f"{name}_autocal_{ts}.csv")
    cols = sorted({k for r in rows for k in r})
    cols = ["phase"] + [c for c in cols if c != "phase"]
    with open(csv_path, "w", encoding="utf-8", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow(r)
    return prof_path, csv_path


# ---------------------------------------------------------------------------
# Host selection + main
# ---------------------------------------------------------------------------
def open_host(args):
    """Return a host. --dry-run NEVER opens a serial port."""
    if args.dry_run:
        print("# DRY-RUN: SimEscHost (no serial port opened)")
        return SimEscHost(), (lambda _s=0.0: None)
    host = esctool.EscHost(args.port)
    return host, time.sleep


PHASES = ("direction", "coldstart", "minrpm", "curve", "tune-startup", "tune-smooth")


def run_pipeline(cal: Calibrator, which):
    if which in ("all", "direction"):
        cal.direction()
    if which in ("all", "coldstart"):
        cal.coldstart()
    if which in ("all", "minrpm"):
        cal.minrpm()
    if which in ("all", "curve"):
        cal.curve()
    if which in ("all", "tune-startup"):
        cal.tune_startup()
    if which in ("all", "tune-smooth"):
        cal.tune_smooth()


def main():
    ap = argparse.ArgumentParser(description="ESC low-speed auto-calibration")
    ap.add_argument("phase", choices=("all",) + PHASES)
    ap.add_argument("--esc-index", type=int, default=1, help="ESC index (default 1)")
    ap.add_argument("--max-throttle", type=int, default=600,
                    help="throttle ceiling (default 600; >800 needs confirmation)")
    ap.add_argument("--name", default="thruster", help="profile/report base name")
    ap.add_argument("--port", help="serial port (default: auto-detect)")
    ap.add_argument("--dry-run", action="store_true",
                    help="run the whole pipeline against a simulated ESC (no hardware)")
    ap.add_argument("--yes", action="store_true", help="skip the high-throttle confirmation")
    args = ap.parse_args()

    if args.max_throttle > CONFIRM_THROTTLE and not args.yes and not args.dry_run:
        reply = input(f"max-throttle {args.max_throttle} > {CONFIRM_THROTTLE}. Proceed? [y/N] ")
        if reply.strip().lower() not in ("y", "yes"):
            sys.exit("aborted")

    host, sleep = open_host(args)
    drive = DriveSession(host, args.esc_index, args.max_throttle, sleep=sleep)
    config = ConfigSession(host, args.esc_index, sleep=sleep)
    rows: list = []
    cal = Calibrator(drive, config, rows)

    def _panic(*_):
        drive.disarm()
        try:
            host.close()
        finally:
            os._exit(1)
    signal.signal(signal.SIGINT, _panic)
    signal.signal(signal.SIGTERM, _panic)

    failure = None
    try:
        drive.prepare()
        drive.arm()
        run_pipeline(cal, args.phase)
    except CalibrationError as e:
        failure = str(e)                 # partial results are kept, no fabricated value
    finally:
        drive.disarm()                   # ALWAYS disarm
        prof, csvp = write_reports(args.name, cal.results, rows)
        print(f"# wrote profile: {prof}")
        print(f"# wrote report:  {csvp}")
        host.close()
    if failure:
        sys.exit(f"CALIBRATION FAILED: {failure}")


if __name__ == "__main__":
    main()
