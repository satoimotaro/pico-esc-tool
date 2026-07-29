# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""Unit tests for the coast-down brake gate.

The rotor on this ESC does not freely coast: at zero commanded throttle the ESC brakes it, and the
braking deceleration RISES as the rotor slows (measured bare-shaft: -5000 -> -72000 rpm/s). Feeding
those windows to `J = tau_drag/(-dw/dt)` understates J by up to ~10x, which is exactly the bug the
old hand-picked `--fit-lo 1550` was silently papering over on one specific rotor.

`brake_onset` has to find the end of the real drag window from the SHAPE of the decay, with no rpm
constants, because the brake bites at whatever speed the ESC decides.
"""
import coastdown


def _res(pairs, t0=0.0, dt=0.02):
    """[(rpm, dwdt), ...] -> the (t, rpm, dwdt) triples `derive()` produces."""
    return [(t0 + i * dt, rpm, dwdt) for i, (rpm, dwdt) in enumerate(pairs)]


def test_brake_onset_finds_the_step_up_in_deceleration():
    # Six windows of honest drag at -5000, then the brake bites an order of magnitude harder.
    res = _res([(3000.0, -5000.0)] * 6 + [(2000.0, -50000.0)] * 4)
    assert coastdown.brake_onset(res, 0) == 6


def test_brake_onset_returns_end_when_the_coast_is_free():
    # A genuinely free coast never trips the gate — every window must stay usable.
    res = _res([(3000.0 - 50 * i, -5000.0) for i in range(12)])
    assert coastdown.brake_onset(res, 0) == len(res)


def test_brake_onset_is_sign_agnostic():
    """The AS5600 reads NEGATIVE for a positive command on this bench.

    Deceleration means |w| falling, i.e. dw/dt OPPOSING w — not `dwdt < 0`. A gate written against
    the raw sign passes the whole braking phase through on a reversed encoder, which is the same
    trap that once spread J across three decades in `report()`.
    """
    fwd = _res([(3000.0, -5000.0)] * 6 + [(2000.0, -50000.0)] * 4)
    rev = _res([(-3000.0, 5000.0)] * 6 + [(-2000.0, 50000.0)] * 4)
    assert coastdown.brake_onset(rev, 0) == coastdown.brake_onset(fwd, 0) == 6


def test_brake_onset_respects_the_start_index():
    """The still-driven plateau before the cut must not set the baseline.

    `coast_start` hands over an index; windows before it are ~zero deceleration, and letting those
    into the median makes the threshold ~0 so the FIRST real drag window reads as a brake.
    """
    res = _res([(3000.0, -20.0)] * 4                    # plateau: drive not cut yet
               + [(3000.0, -5000.0)] * 6                # real drag
               + [(2000.0, -50000.0)] * 4)              # brake
    assert coastdown.brake_onset(res, 4) == 10


def test_brake_onset_gives_up_gracefully_on_a_short_run():
    # Fewer windows than the probe needs: no basis to judge, so keep everything rather than invent a
    # cut point (silently dropping samples would look like a successful fit).
    res = _res([(3000.0, -5000.0)] * 2)
    assert coastdown.brake_onset(res, 0) == len(res)


def test_drag_table_is_clamped_outside_its_measured_range():
    """`drag_at` extrapolating would invent torque for speeds nobody measured."""
    lo_rpm, lo_tau = coastdown.DRAG_TABLE[0]
    hi_rpm, hi_tau = coastdown.DRAG_TABLE[-1]
    assert coastdown.drag_at(lo_rpm - 500) == lo_tau
    assert coastdown.drag_at(hi_rpm + 500) == hi_tau
    assert coastdown.drag_at(-2000) == coastdown.drag_at(2000)      # magnitude only
