# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""Tests for curvepush: the measured-curve upload transfer.

The point of the tool is that the curve which ends up LIVE is exactly the one measured — so these
cover the round-trip and the rejections, against the sim host that mirrors the firmware's staging
rules (SimEscHost._curve_cmd).
"""
import pytest

import curvepush
from pico_esc.link import SimClock
from pico_esc.sim import SimEncEscHost
from pico_esc.velocity import SpeedProfile


def _host():
    return SimEncEscHost(SimClock())


def _profile(points, regimes=None, up=1400.0, dn=1308.0, pp=7):
    return SpeedProfile(points, pole_pairs=pp, regimes=regimes,
                        crossover={"up_erpm": up, "dn_erpm": dn})


SPAN = [(0.0, 0.0), (200.0, 60.0), (503.0, 186.8), (540.0, 850.0), (700.0, 1900.0)]


def test_round_trip_matches_exactly():
    dev = _host()
    sent = curvepush.push(dev, 1, _profile(SPAN))
    meta, got = curvepush.readback(dev, 1)
    assert meta["measured"] == "1"
    assert int(meta["n"]) == len(sent) == len(SPAN)
    assert [(t, r) for t, r, _ in got] == sent


def test_regimes_derived_from_the_seam_when_untagged():
    # No regime tags in the YAML -> the device derives them from up_erpm, and the first point above
    # the seam must come back tagged LINE (that is what lineFloor()/the soft-sensor gate keys off).
    dev = _host()
    curvepush.push(dev, 1, _profile(SPAN))
    _, got = curvepush.readback(dev, 1)
    tags = [g for _, _, g in got]
    assert tags == ["s", "s", "s", "l", "l"], tags        # 850*7 = 5950 >= 1400, 186.8*7 = 1308 < 1400


def test_explicit_line_tag_below_the_seam_is_kept():
    # A point measured in the hysteresis band coming down is real information, so an explicit LINE tag
    # below the seam is NOT demoted. (503*7 = 3521 is above 1400 here, so use a higher seam.)
    dev = _host()
    regs = ["sine", "sine", "line", "line", "line"]
    curvepush.push(dev, 1, _profile(SPAN, regimes=regs, up=4000.0, dn=3800.0))
    _, got = curvepush.readback(dev, 1)
    assert [g for _, _, g in got] == ["s", "s", "l", "l", "l"]


def test_sine_tag_above_the_seam_is_promoted_to_line():
    # velcal mislabels the handoff point (the bench profile tags 628 rpm = 4397 eRPM as "sine" against
    # a 1400 eRPM seam). Taking that at face value would push lineFloor() — and therefore the
    # soft-sensor gate — up to the NEXT point, blanking a band where the estimate is valid.
    dev = _host()
    regs = ["sine", "sine", "sine", "sine", "line"]       # index 3 = 850 rpm, 5950 eRPM >> 1400
    curvepush.push(dev, 1, _profile(SPAN, regimes=regs))
    _, got = curvepush.readback(dev, 1)
    assert [g for _, _, g in got] == ["s", "s", "s", "l", "l"]


def test_non_monotone_curve_is_rejected_and_nothing_goes_live():
    # SpeedProfile refuses to construct a non-monotone curve, so drive the raw commands: this is the
    # DEVICE-side guard, i.e. what protects the live curve if a malformed table reaches the wire.
    dev = _host()
    curvepush.push(dev, 1, _profile(SPAN))                # install a good curve first
    dev.cmd("curve 1 begin 3")
    dev.cmd("curve 1 add 0 0")
    dev.cmd("curve 1 add 200 90")
    dev.cmd("curve 1 add 150 60")                         # thrust goes BACKWARDS
    with pytest.raises(RuntimeError, match="curve-commit"):
        dev.cmd("curve 1 commit 7 1400 1308")
    _, got = curvepush.readback(dev, 1)
    assert [(t, r) for t, r, _ in got] == list(SPAN)      # the good curve is still the live one


def test_short_upload_is_rejected():
    dev = _host()
    dev.cmd("curve 1 begin 3")
    dev.cmd("curve 1 add 0 0")
    dev.cmd("curve 1 add 100 50")
    with pytest.raises(RuntimeError, match="staged 2 of 3"):
        dev.cmd("curve 1 commit 7 1400 1308")


def test_profile_without_a_crossover_is_refused():
    dev = _host()
    prof = SpeedProfile(SPAN, pole_pairs=7)
    with pytest.raises(SystemExit, match="crossover"):
        curvepush.push(dev, 1, prof)


def test_thin_keeps_the_endpoints_and_both_sides_of_the_seam():
    pts = [(float(i * 10), float(i)) for i in range(40)]
    pts[25] = (250.0, 900.0)                              # a big jump = the seam
    out, _ = curvepush.thin(pts, None, 12)
    assert len(out) <= 12
    assert out[0] == pts[0] and out[-1] == pts[-1]
    assert pts[24] in out and pts[25] in out


def test_thin_is_a_no_op_below_the_cap():
    out, regs = curvepush.thin(SPAN, None, 24)
    assert out == SPAN and regs is None
