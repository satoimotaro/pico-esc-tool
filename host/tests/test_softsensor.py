# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""Tests for softsensor.estimate: V_eff recovery, load detection, and sort-independence (#7.2)."""
import softsensor
from softsensor import duty


def _noload(kv, v, cmds):
    # 6-step no-load (eff=1): rpm = V * KV * duty(cmd)
    return [(c, v * kv * duty(c)) for c in cmds]


CMDS = [560, 600, 640, 700]


def test_estimate_recovers_v_eff_no_load():
    kv, v = 350.0, 11.1
    s = softsensor.estimate(kv, 7, _noload(kv, v, CMDS))
    assert abs(s["v_eff"] - v) < 0.05, s
    assert not s["load_present"]


def test_estimate_flags_load_under_droop():
    kv, v = 350.0, 11.1
    kd = 1e-4  # rpm^2-growing droop = current/load
    drooped = [(c, r - kd * r * r) for c, r in _noload(kv, v, CMDS)]
    s = softsensor.estimate(kv, 7, drooped)
    assert s["load_present"], s
    assert s["droop_top_rpm"] < 0, s  # the top (max-duty) point sits below the no-load line


def test_estimate_sort_independent():
    # the #7.2 fix: droop_top_rpm must be the max-rpm point regardless of input order
    kv, v = 350.0, 11.1
    kd = 1e-4
    drooped = [(c, r - kd * r * r) for c, r in _noload(kv, v, CMDS)]
    a = softsensor.estimate(kv, 7, drooped)
    b = softsensor.estimate(kv, 7, list(reversed(drooped)))
    assert a["load_present"] == b["load_present"]
    assert abs(a["droop_top_rpm"] - b["droop_top_rpm"]) < 1e-6, (a, b)
