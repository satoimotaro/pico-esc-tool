# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""Tests for softsensor.estimate: V_eff recovery, load detection, and sort-independence (#7.2).

`cross_up` is a REQUIRED argument everywhere here (E.9, 2026-07-30). It used to default to 34 while
the bench ran 36, which biases V_eff by ~0.6 % at cmd 780 and ~44 % at cmd 570 — and the load/droop
numbers then absorb that model error as if it were physics. These tests pass it explicitly and one of
them asserts that omitting it raises."""
import pytest

import softsensor
from softsensor import duty

# 34 = DUTY6_CROSSUP_REF, the value the universal duty shape was fitted at, so these synthetic
# no-load points are the same numbers this test has always used. The POINT of E.9 is not that 36 is
# right, it is that the caller must SAY which ESC it means — asserted in
# test_cross_up_is_required_not_defaulted below.
CROSS_UP = 34.0


def _noload(kv, v, cmds):
    # 6-step no-load (eff=1): rpm = V * KV * duty(cmd, cross_up)
    return [(c, v * kv * duty(c, CROSS_UP)) for c in cmds]


CMDS = [560, 600, 640, 700]


def test_cross_up_is_required_not_defaulted():
    with pytest.raises(TypeError):
        duty(600)
    with pytest.raises(TypeError):
        softsensor.estimate(350.0, 7, [(600, 1000.0), (650, 1400.0)])
    # and it MATTERS: a higher cross_up gives less duty for the same command
    assert duty(600, 36.0) < duty(600, 34.0)


def test_estimate_recovers_v_eff_no_load():
    kv, v = 350.0, 11.1
    s = softsensor.estimate(kv, 7, _noload(kv, v, CMDS), CROSS_UP)
    assert abs(s["v_eff"] - v) < 0.05, s
    assert not s["load_present"]


def test_estimate_flags_load_under_droop():
    kv, v = 350.0, 11.1
    kd = 1e-4  # rpm^2-growing droop = current/load
    drooped = [(c, r - kd * r * r) for c, r in _noload(kv, v, CMDS)]
    s = softsensor.estimate(kv, 7, drooped, CROSS_UP)
    assert s["load_present"], s
    assert s["droop_top_rpm"] < 0, s  # the top (max-duty) point sits below the no-load line


def test_estimate_sort_independent():
    # the #7.2 fix: droop_top_rpm must be the max-rpm point regardless of input order
    kv, v = 350.0, 11.1
    kd = 1e-4
    drooped = [(c, r - kd * r * r) for c, r in _noload(kv, v, CMDS)]
    a = softsensor.estimate(kv, 7, drooped, CROSS_UP)
    b = softsensor.estimate(kv, 7, list(reversed(drooped)), CROSS_UP)
    assert a["load_present"] == b["load_present"]
    assert abs(a["droop_top_rpm"] - b["droop_top_rpm"]) < 1e-6, (a, b)
