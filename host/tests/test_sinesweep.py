# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""Unit tests for the forced-sine regime classifier.

Two traps this file exists to hold shut, both hit on the first real sweep:

1. Classifying 6-step by "the rotor outruns the forced field" ALONE. At the bottom of the regime the
   rotor does not rotate — it oscillates about zero and reverses several times per dwell — so a mean
   of 39 rpm against a 21 rpm field satisfies the ratio test while the ESC is firmly in sine. That
   false positive landed on the FIRST command of the leg and silently discarded every later point.
2. Finding the seam by scanning for the first 6-step-looking point. A descending leg starts in
   6-step, so that reports a handoff at the top of the sweep — useless and backwards. The boundary
   has to be found as a change of classification, not a first match.
"""
import sinesweep


class Opts:
    seam_factor = 1.5
    seam_min_rpm = 250.0
    pole_pairs = 7


def _row(cmd, enc, pole_pairs=7):
    """(leg, cmd, field_rpm, enc_mean, min, max, spread, reversals, tele, slip, n)."""
    f = sinesweep.field_rpm(cmd, pole_pairs)
    slip = (1.0 - abs(enc) / f) if f else 0.0
    return ("up", cmd, f, enc, enc, enc, 0.0, 0, 182.0, slip, 10)


def test_field_rpm_matches_the_firmware_full_scale():
    # Bluejay forced sine: 2499 eRPM at full command; 357 mech full scale at 7 pole pairs.
    assert sinesweep.field_rpm(1000, 7) == 2499.0 / 7
    assert round(sinesweep.field_rpm(1000, 7), 1) == 357.0
    assert sinesweep.field_rpm(0, 7) == 0.0
    assert sinesweep.field_rpm(-500, 7) == sinesweep.field_rpm(500, 7)   # magnitude only


def test_oscillating_rotor_at_the_bottom_is_not_six_step():
    """The measured cmd-60 point: 39 rpm mean against a 21 rpm field = 1.8x, but still sine."""
    assert not sinesweep.in_six_step(_row(60, 39.3), Opts)


def test_genuine_handoff_is_six_step():
    """cmd 540: the rotor jumps to 430 rpm while the field only asks for 193."""
    assert sinesweep.in_six_step(_row(540, 430.4), Opts)


def test_following_the_field_closely_is_not_six_step():
    """cmd 510: 186 rpm against a 182 rpm field — locked to the field, not handed off."""
    assert not sinesweep.in_six_step(_row(510, 185.6), Opts)


def test_six_step_test_is_sign_agnostic():
    """The AS5600 reads negative for a positive command on this bench."""
    assert sinesweep.in_six_step(_row(540, -430.4), Opts)


def test_transition_finds_the_bracket_on_an_ascending_leg():
    sub = [_row(c, e) for c, e in ((480, 178.0), (510, 185.6), (540, 430.4), (570, 750.4))]
    a, b = sinesweep.transition(sub, Opts)
    assert (a[1], b[1]) == (510, 540)


def test_transition_finds_the_bracket_on_a_descending_leg():
    """A descending leg STARTS in 6-step; a first-match scan would report the very first point."""
    sub = [_row(c, e) for c, e in ((570, 751.7), (540, 430.7), (510, 190.6), (480, 176.9))]
    a, b = sinesweep.transition(sub, Opts)
    assert (a[1], b[1]) == (540, 510)


def test_transition_is_none_when_the_leg_never_leaves_sine():
    sub = [_row(c, e) for c, e in ((60, 39.3), (90, 30.0), (120, 53.0), (150, 60.3))]
    assert sinesweep.transition(sub, Opts) is None


def test_transition_is_none_for_a_leg_entirely_in_six_step():
    sub = [_row(c, e) for c, e in ((540, 430.4), (570, 750.4), (600, 1069.3))]
    assert sinesweep.transition(sub, Opts) is None
