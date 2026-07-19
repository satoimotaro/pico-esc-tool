# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""SimEncEscHost S3 crossover-model tests.

The crossover extension is GATED: with the crossover OFF (the existing posctl/tune configs)
the _advance path + RNG draws are byte-identical (proven separately by the golden-CSV
regression test). These exercise the CONFIGURED behaviour: forced-sine below the seam, the
6-step BEMF load-line above Cross_Up, hysteresis on the way down, a live `tele` eRPM, and
determinism. The sim is a MODEL of the (bench-untested) S3 firmware, not hardware truth.
"""
from pico_esc.config import sine_crossover_bytes
from pico_esc.constants import FULLSCALE_RPM
from pico_esc.link import SimClock
from pico_esc.sim import SimEncEscHost


def _mk(seed=1234, crossover=True):
    clock = SimClock()
    h = SimEncEscHost(clock, seed=seed)
    if crossover:
        cu, cd = sine_crossover_bytes(2100.0, 1600.0)
        h.cmd(f"editpage 1 32:{cu:02X},33:{cd:02X}")
    h.cmd("arm 1 bidir")
    return h, clock


def _settle(h, clock, thrust, secs=3.0):
    end = clock.now() + secs
    while clock.now() < end:
        h.cmd(f"thrust 1 {thrust}")
        clock.sleep(0.02)
    return h.rpm


def test_editpage_tracks_crossover_bytes():
    h, _ = _mk()
    assert h.cfg["sine_cross_up"] == 54 and h.cfg["sine_cross_dn"] == 195
    assert h._crossover_on()


def test_crossover_off_is_pure_sine():
    # No editpage -> crossover OFF -> forced-sine law everywhere (target = thrust*FULLSCALE/1000).
    h, clock = _mk(crossover=False)
    assert not h._crossover_on()
    rpm = _settle(h, clock, 900)
    assert abs(rpm - 900 * FULLSCALE_RPM / 1000.0) < 15    # ~321 mech, no handoff jump


def test_below_seam_follows_sine():
    h, clock = _mk()
    rpm = _settle(h, clock, 500)                            # commanded eRPM ~1250 < up 2109
    assert h._regime == "sine"
    assert abs(rpm - 500 * FULLSCALE_RPM / 1000.0) < 15     # ~178 mech


def test_above_seam_jumps_to_load_line():
    h, clock = _mk()
    below = _settle(h, clock, 800)                          # just below the seam (sine)
    assert h._regime == "sine"
    above = _settle(h, clock, 900)                          # crosses Cross_Up -> load-line
    assert h._regime == "line"
    assert above > below * 1.8                              # a clear handoff speed jump


def test_hysteresis_sticky_on_the_way_down():
    h, clock = _mk()
    _settle(h, clock, 950)                                  # up into the load-line
    assert h._regime == "line"
    # step back to a thrust whose commanded eRPM is below Cross_Up but whose actual eRPM is
    # still above Cross_Dn -> must STAY in the load-line (hysteresis, no chatter).
    _settle(h, clock, 700, secs=1.0)
    assert h._regime == "line"
    # drop far enough that the actual eRPM falls below Cross_Dn -> back to sine.
    _settle(h, clock, 200)
    assert h._regime == "sine"


def test_tele_reports_live_rpm_above_seam():
    h, clock = _mk()
    _settle(h, clock, 950)
    line = h.cmd("tele 1")[0].split("|")
    assert int(line[1]) > 400                               # live mech eRPM-ish, not stale 0
    assert int(line[4]) == 25                               # temp field unchanged


def test_deterministic_same_seed():
    ha, ca = _mk(seed=7)
    hb, cb = _mk(seed=7)
    assert _settle(ha, ca, 900) == _settle(hb, cb, 900)


def test_crossover_off_tele_no_advance():
    # With the crossover OFF, `tele` must NOT advance the rotor (byte-identity guard): calling
    # tele repeatedly at zero thrust leaves pos_counts untouched by tele itself.
    h, clock = _mk(crossover=False)
    h.cmd("thrust 1 0")
    before = h.pos_counts
    for _ in range(5):
        h.cmd("tele 1")                                     # no clock advance, no _advance
    assert h.pos_counts == before
