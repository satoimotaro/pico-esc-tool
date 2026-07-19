# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""velctl closed-loop gain resolution: explicit CLI flag > profile control block > DEFAULT_GAINS.

Gains are plant-dependent (the real 930KV 6-step plant is ~30x the sim), so a bench-tuned profile
carries its own `control:` block; these tests pin the three-tier merge that velctl performs.
"""
import os
import subprocess
import sys

from pico_esc.velocity import DEFAULT_GAINS, SpeedProfile

HOST = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _run(*args, expect=0):
    r = subprocess.run([sys.executable, *args], cwd=HOST,
                       capture_output=True, text=True, timeout=120)
    assert r.returncode == expect, f"{args} -> rc={r.returncode}\n{r.stdout}\n{r.stderr}"
    return r


def _gains_line(stdout):
    return next(l for l in stdout.splitlines() if l.startswith("# gains:"))


def _profile_with_control(path, control):
    pts = [(0, 0.0), (100, 35.7), (300, 107.1), (700, 350.0)]
    SpeedProfile(pts, motor="gaintest", control=control).save(str(path))


def test_gains_from_profile_control(tmp_path):
    prof = tmp_path / "p.yaml"
    _profile_with_control(prof, {"kp": 0.03, "ki": 0.12, "trim_max": 400.0, "blend_secs": 0.25})
    r = _run("velctl.py", "speed", "--rpm", "100", "--dry-run", "--seed", "1234",
             "--secs", "2", "--profile", str(prof), "--csv", str(tmp_path / "v.csv"))
    line = _gains_line(r.stdout)
    assert "kp=0.03" in line and "ki=0.12" in line and "trim_max=400" in line
    assert "blend_secs=0.25" in line and "source: CLI overrides > profile" in line


def test_cli_flag_overrides_profile_control(tmp_path):
    prof = tmp_path / "p.yaml"
    _profile_with_control(prof, {"kp": 0.03, "ki": 0.12})
    r = _run("velctl.py", "speed", "--rpm", "100", "--dry-run", "--seed", "1234", "--secs", "2",
             "--profile", str(prof), "--kp", "0.5", "--csv", str(tmp_path / "v.csv"))
    line = _gains_line(r.stdout)
    assert "kp=0.5" in line          # CLI wins for kp
    assert "ki=0.12" in line         # profile still supplies ki


def test_gains_fall_back_to_default_without_control(tmp_path):
    # No control block -> DEFAULT_GAINS (the sim profile, unchanged, exercises this).
    prof = tmp_path / "p.yaml"
    SpeedProfile([(0, 0.0), (100, 35.7), (700, 350.0)], motor="nocontrol").save(str(prof))
    r = _run("velctl.py", "speed", "--rpm", "100", "--dry-run", "--seed", "1234", "--secs", "2",
             "--profile", str(prof), "--csv", str(tmp_path / "v.csv"))
    line = _gains_line(r.stdout)
    assert f"kp={DEFAULT_GAINS['kp']:g}" in line and "source: CLI overrides > default" in line
