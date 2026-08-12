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


def test_tune_crossover_lock_lowspeed_reverse_dry_run():
    # The low-speed mode (descend into 6-step, hold at an encoder-reliable speed) must work in the
    # REVERSE direction too (--sign -1) — the whole point is that reverse aliases the encoder at the
    # top of the ramp. It must run, measure a (sim-plumbing) slip, and write an apply-compatible tune.
    # --measure-speed is TRUE mechanical RPM (both tele and encoder are mechanical; no /pole-pairs).
    # 280 sits inside the sim's 6-step band (~27-543 mech) and above its down-handoff, so the descend
    # can settle and measure there. (Was 40 back when a latent double-division scaled speeds 7x down.)
    r = _run("autocal.py", "tune-crossover-lock", "--dry-run", "--lowspeed", "--sign", "-1",
             "--name", "xolow", "--test-cmd", "900", "--measure-speed", "280",
             "--grid-timing", "1,4", "--grid-demag", "3",
             "--ramp-secs", "1.0", "--descend-secs", "1.0", "--hold-secs", "0.5", "--cooldown", "0")
    assert "# DRY-RUN: SimEncEscHost (no serial port opened)" in r.stdout
    assert "mode=lowspeed" in r.stdout
    assert "# CHOSEN: comm_timing=" in r.stdout
    line = next(l for l in r.stdout.splitlines() if l.startswith("# wrote profile:"))
    prof = line.split(":", 1)[1].strip()
    sys.path.insert(0, HOST)
    from pico_esc.config import load_yaml, encode_overrides, TIMING, DEMAG
    settings = load_yaml(prof).get("settings", {})
    assert settings.get("comm_timing") in TIMING.values()
    assert settings.get("demag_compensation") in DEMAG.values()
    assert encode_overrides(settings)


def test_esctool_help():
    r = _run("esctool.py", "--help")
    assert "BLHeli-S ESC CLI" in r.stdout


def test_package_import_smoke():
    r = _run("-c", "from pico_esc import EscLink, ESC; "
                   "from pico_esc.control import PositionController; print('ok')")
    assert r.stdout.strip() == "ok"


def test_azimuth_ondevice_subcommands_refuse_dry_run():
    """`trace` and `step` drive the ON-DEVICE loop, which the sim host cannot emulate. They must
    say so and exit non-zero rather than silently opening a port or faking a result — a dry-run
    that appears to succeed is how a blind controller once reported err 0.00 as a perfect run."""
    for sub in ("trace", "step"):
        r = _run("azimuth.py", sub, "--dry-run", expect=1)
        assert "sim host has no `az`" in r.stderr
        assert "test_aztrace.py" in r.stderr


def test_azimuth_ondevice_defaults_are_the_config_that_converged():
    """Defaults must be kp 3 / ki 0.5 / kd 0.3 / vmax 200 with the velocity-error clamp OFF —
    the 2026-08-11T17:44Z host run — NOT the firmware's own defaults, which descend from
    kp 8 / ki 1.5, a run that never converged and was picked on a spin-up-transient metric."""
    import argparse

    import azimuth

    p = argparse.ArgumentParser()
    azimuth.add_ondevice(p)
    d = vars(p.parse_args([]))
    assert (d["kp"], d["ki"], d["kd"]) == (3.0, 0.5, 0.3)
    assert d["vmax"] == 200.0
    assert d["verr_clamp"] == 0.0          # the clamp was not present in the run that converged
    assert d["smax"] == 40.0               # authority, NOT the thing to shrink when detuning
    assert d["s_trim"] == azimuth.AZIMUTH_NULL_S
    assert d["sign_b"] == -1               # motor B is mirrored — measured, not assumed
