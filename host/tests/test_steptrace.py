# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""Unit tests for the step-response analysis.

Both guards here are regressions from the first real traces:

1. Overshoot measured with max() regardless of direction. Stepping DOWN through the crossover seam
   (1048 -> 175 rpm) reported "overshoot 497%" — the largest speed in a descending trace is simply
   where it started. A step down can only UNDERshoot, and that is the number that matters, because
   undershooting to zero is the stall this test is looking for.
2. A settling time taken from inside the tail window. The tail is what DEFINED the final value, so a
   run that is still moving at the end reported "settles at t=10.00s" — the exact moment it ran out
   of trace — instead of admitting it never settled.
"""
import steptrace


class Opts:
    mode = "thrust"
    start = None
    to = 0
    tail_frac = 0.25
    settle_band = 0.05
    stall_rpm = 30.0


def _rows(speeds, dt=0.01):
    return [(i * dt, s, float("nan")) for i, s in enumerate(speeds)]


def _report(rows, capsys, **over):
    o = Opts()
    for k, v in over.items():
        setattr(o, k, v)
    steptrace.analyse(rows, o)
    return capsys.readouterr().out


def test_step_up_reports_overshoot(capsys):
    # Rises to 1855, settles at ~1191 — the measured bare-shaft startup.
    out = _report(_rows([0] * 10 + [1855] * 10 + [1191] * 80), capsys)
    assert "peak" in out and "overshoot" in out
    assert "undershoot" not in out


def test_step_down_reports_undershoot_not_overshoot(capsys):
    """1048 -> 175 rpm through the seam: max() called that a 497% overshoot."""
    out = _report(_rows([1048] * 10 + [114] * 10 + [175] * 80), capsys)
    assert "undershoot" in out
    assert "overshoot" not in out
    assert "trough" in out


def test_settling_time_is_not_taken_from_the_tail(capsys):
    """Still drifting at the end must read as NEVER settling, not as settling at the last sample."""
    out = _report(_rows(list(range(0, 100))), capsys)          # monotonic ramp, never settles
    assert "NEVER settles" in out


def test_settling_time_reported_when_it_really_settles(capsys):
    out = _report(_rows([0] * 5 + [200] * 5 + [100] * 90), capsys)
    assert "settles within" in out
    assert "NEVER" not in out


def test_stall_is_flagged(capsys):
    """The aborted rpm-2400 startup: climbs, then the controller gives up and it coasts to rest."""
    out = _report(_rows([0] * 5 + [657] * 20 + [0] * 75), capsys)
    assert "STALLED" in out


def test_final_value_uses_the_tail_not_the_last_sample(capsys):
    """This rotor's low-speed ripple is the same size as the quantity being measured."""
    rows = _rows([1000] * 75 + [180, 220] * 12 + [500])        # one wild last sample
    out = _report(rows, capsys)
    assert "-> 500 rpm" not in out


def test_silent_telemetry_is_called_out(capsys):
    """During startup the ESC has no BEMF lock; the encoder is the only witness."""
    out = _report(_rows([0] * 10 + [1000] * 90), capsys)
    assert "SILENT" in out


def test_too_few_samples_bails_out(capsys):
    out = _report(_rows([100, 200]), capsys)
    assert "too few samples" in out
