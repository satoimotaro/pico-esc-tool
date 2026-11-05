# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""Differential azimuth mixer — the kinematics, the saturation policy, and the sign identification.

These are the properties the pod's two operating modes rest on, so they are asserted rather than
assumed: counter-rotation must hold the angle exactly, same-direction must produce no thrust, and
saturation must never convert a turn command into thrust.
"""
import pytest

from pico_esc.differential import DifferentialMixer


# -- the two operating modes ----------------------------------------------------------
def test_counter_rotation_holds_the_azimuth():
    """A = -B is pure thrust: the azimuth channel must be exactly zero, not merely small."""
    m = DifferentialMixer(tmax=1000)
    s, d = m.unmix(300, -300)
    assert s == 0.0
    assert d == 300.0


def test_same_direction_is_pure_steering():
    """A = B must produce no propeller command at all."""
    m = DifferentialMixer(tmax=1000)
    s, d = m.unmix(300, 300)
    assert s == 300.0
    assert d == 0.0


def test_mix_and_unmix_round_trip():
    m = DifferentialMixer(tmax=1000)
    for s, d in ((0, 0), (100, 0), (0, 250), (120, -80), (-300, 200)):
        r = m.mix(s, d)
        assert m.unmix(r.a, r.b) == pytest.approx((s, d))


def test_azimuth_is_the_mean_of_the_two_motors():
    """The operator's convention: azimuth = (A+B)/2, propeller = (A-B)/2."""
    m = DifferentialMixer(tmax=1000)
    s, d = m.unmix(400, 100)
    assert s == 250.0
    assert d == 150.0


# -- the mirrored second motor --------------------------------------------------------
def test_mirrored_b_swaps_the_channels_if_uncorrected():
    """The failure the sign convention exists to prevent.

    If B is physically mirrored and sign_b is left at +1, what the caller believes is pure thrust
    is delivered to the hardware as pure steering. Asserting it here means the swap can never be
    a surprise found on the bench.
    """
    naive = DifferentialMixer(tmax=1000, sign_b=1)
    corrected = DifferentialMixer(tmax=1000, sign_b=-1)
    # caller asks for pure thrust (no azimuth motion)
    n, c = naive.mix(s=0, d=300), corrected.mix(s=0, d=300)
    # both send the same magnitudes, but with opposite handedness on B
    assert (n.a, n.b) == (300, -300)
    assert (c.a, c.b) == (300, 300)
    # ...and in the mirrored hardware's own frame those are opposite channels
    assert corrected.unmix(c.a, c.b) == (0.0, 300.0)


def test_sign_b_is_folded_into_both_directions():
    m = DifferentialMixer(tmax=1000, sign_a=-1, sign_b=-1)
    r = m.mix(s=100, d=50)
    assert m.unmix(r.a, r.b) == pytest.approx((100, 50))


# -- saturation policy ----------------------------------------------------------------
def test_saturation_sheds_thrust_and_keeps_the_angle():
    """Angle hold outranks thrust: losing the angle points the thruster somewhere unintended."""
    m = DifferentialMixer(tmax=1000)
    r = m.mix(s=800, d=800)
    assert r.s == 800                      # azimuth command preserved in full
    assert r.d == 200                      # propeller shed to fit the ceiling
    assert r.d_shed == 600
    assert r.saturated
    assert abs(r.a) <= 1000 and abs(r.b) <= 1000


def test_saturation_never_injects_thrust_into_a_turn():
    """Independent per-channel clamping would distort BOTH channels; this must not.

    Clamping A and B separately at, say, s=900 d=0 (A=B=900, in range) is harmless, but at
    s=1200 d=0 it would give A=B=1000 — fine — whereas s=600 d=900 clamped independently gives
    A=1000 (from 1500) and B=-300, i.e. d drops to 650 AND s rises to 350. Shedding d keeps s exact.
    """
    m = DifferentialMixer(tmax=1000)
    r = m.mix(s=600, d=900)
    s_out, d_out = m.unmix(r.a, r.b)
    assert s_out == 600                    # the commanded turn is delivered exactly
    assert d_out == 400                    # only the propeller gave way
    assert abs(r.a) <= 1000 and abs(r.b) <= 1000


def test_s_alone_is_clamped_when_it_exceeds_the_ceiling():
    m = DifferentialMixer(tmax=1000)
    r = m.mix(s=1500, d=300)
    assert r.s == 1000
    assert r.d == 0                        # no headroom left
    assert r.saturated


def test_unsaturated_mix_reports_no_shed():
    m = DifferentialMixer(tmax=1000)
    r = m.mix(s=100, d=200)
    assert not r.saturated
    assert r.d_shed == 0


# -- identification -------------------------------------------------------------------
def test_identify_same_handed_motors():
    sign_a, sign_b, gain = DifferentialMixer.identify(rate_from_a=12.0, rate_from_b=12.0)
    assert (sign_a, sign_b) == (1, 1)
    assert gain == pytest.approx(12.0)


def test_identify_detects_a_mirrored_b():
    sign_a, sign_b, _ = DifferentialMixer.identify(rate_from_a=12.0, rate_from_b=-11.0)
    assert (sign_a, sign_b) == (1, -1)


def test_identify_handles_a_globally_inverted_axis():
    """Both motors turning the axis the other way is a wiring/encoder convention, not a mirror."""
    sign_a, sign_b, _ = DifferentialMixer.identify(rate_from_a=-12.0, rate_from_b=-12.0)
    assert (sign_a, sign_b) == (-1, -1)


def test_identify_refuses_to_infer_a_sign_from_noise():
    """A near-zero rate carries no sign; fabricating one would silently swap the channels."""
    with pytest.raises(ValueError, match="barely moved"):
        DifferentialMixer.identify(rate_from_a=0.01, rate_from_b=12.0)
    with pytest.raises(ValueError, match="barely moved"):
        DifferentialMixer.identify(rate_from_a=12.0, rate_from_b=0.0)


# -- constructor guards ---------------------------------------------------------------
def test_rejects_a_bogus_sign():
    with pytest.raises(ValueError):
        DifferentialMixer(sign_a=0)
    with pytest.raises(ValueError):
        DifferentialMixer(sign_b=2)


def test_rejects_a_nonpositive_ceiling():
    with pytest.raises(ValueError):
        DifferentialMixer(tmax=0)
