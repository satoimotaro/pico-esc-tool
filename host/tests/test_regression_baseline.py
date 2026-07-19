# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""HARD regression gate for the velctl v1 work (crossover-off byte-identity).

The velctl feature EXTENDS SimEncEscHost. The #1 constraint is that with the crossover
OFF (cross_up==0 — the existing posctl/tune dry-run configs) the sim's _advance path and
its seeded RNG draw order stay BIT-FOR-BIT identical, so the existing dry-run output is
unchanged. This test re-runs the exact command whose CSV was captured from HEAD BEFORE any
sim.py edit and asserts byte-equality against that committed golden. It must pass BEFORE and
AFTER every sim change; a diff means the byte-identity constraint was broken.

SimClock is fully virtual (rows carry sim-time only, no wall-clock / no paths), so the CSV
is reproducible and can be diffed directly.
"""
import os
import subprocess
import sys

HOST = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GOLDEN = os.path.join(HOST, "tests", "data", "posctl_move_d90_s1234.golden.csv")


def test_posctl_move_d90_s1234_byte_identical(tmp_path):
    out = tmp_path / "chk.csv"
    r = subprocess.run(
        [sys.executable, "posctl.py", "move", "--deg", "90", "--dry-run",
         "--seed", "1234", "--csv", str(out)],
        cwd=HOST, capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, f"posctl move failed rc={r.returncode}\n{r.stdout}\n{r.stderr}"
    with open(GOLDEN, "rb") as fh:
        golden = fh.read()
    with open(out, "rb") as fh:
        got = fh.read()
    assert got == golden, "crossover-off dry-run CSV diverged from the golden oracle"
