# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""CLI dry-run smoke tests: the wrappers must still run standalone, never open a serial port
in --dry-run, and print their signature lines. Kept fast (a handful of short sim runs).
"""
import os
import subprocess
import sys

HOST = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _run(*args, expect=0):
    r = subprocess.run([sys.executable, *args], cwd=HOST,
                       capture_output=True, text=True, timeout=120)
    assert r.returncode == expect, f"{args} -> rc={r.returncode}\n{r.stdout}\n{r.stderr}"
    return r


def test_posctl_move_dry_run():
    r = _run("posctl.py", "move", "--deg", "90", "--dry-run", "--seed", "1234")
    assert "# DRY-RUN: SimEncEscHost (no serial port opened)" in r.stdout
    assert "# exit reason: converged" in r.stdout


def test_posctl_wrongway_abort():
    # inverted sim + --no-autocal => the wrong-way guard must trip and exit non-zero
    r = _run("posctl.py", "move", "--deg", "90", "--dry-run", "--sim-invert",
             "--no-autocal", "--seed", "1234", expect=1)
    assert "# exit reason: aborted" in r.stdout
    assert "wrong-way runaway" in r.stderr  # sys.exit("POSCTL ABORTED: …") -> stderr


def test_autocal_all_dry_run():
    r = _run("autocal.py", "all", "--dry-run", "--esc-index", "1")
    assert "# DRY-RUN: SimEscHost (no serial port opened)" in r.stdout


def test_tune_sine_amp_dry_run():
    r = _run("tune_sine_amp.py", "--dry-run", "--seed", "1234")
    assert "# sweep" in r.stdout


def test_tune_crossover_lock_dry_run():
    # The crossover-lock phase must run against SimEncEscHost (no serial), sweep the grid,
    # pick a winner, and write an apply-compatible profile. Kept fast with short ramps.
    r = _run("autocal.py", "tune-crossover-lock", "--dry-run", "--name", "xotest",
             "--grid-timing", "1,4", "--grid-demag", "3", "--test-cmd", "900",
             "--ramp-secs", "1.0", "--hold-secs", "0.5", "--down-secs", "0.8", "--cooldown", "0")
    assert "# DRY-RUN: SimEncEscHost (no serial port opened)" in r.stdout
    assert "# CHOSEN: comm_timing=" in r.stdout
    # locate the emitted profile and confirm it carries an apply-compatible (enum-named) tune
    line = next(l for l in r.stdout.splitlines() if l.startswith("# wrote profile:"))
    prof = line.split(":", 1)[1].strip()
    sys.path.insert(0, HOST)
    from pico_esc.config import load_yaml, encode_overrides, TIMING, DEMAG
    settings = load_yaml(prof).get("settings", {})
    assert settings.get("comm_timing") in TIMING.values()
    assert settings.get("demag_compensation") in DEMAG.values()
    assert encode_overrides(settings)              # names encode to override bytes (apply-compatible)


def test_esctool_help():
    r = _run("esctool.py", "--help")
    assert "BLHeli-S ESC CLI" in r.stdout


def test_package_import_smoke():
    r = _run("-c", "from pico_esc import EscLink, ESC; "
                   "from pico_esc.control import PositionController; print('ok')")
    assert r.stdout.strip() == "ok"
