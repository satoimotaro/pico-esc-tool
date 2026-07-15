# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""ESC drive-facet + telemetry-reader tests (additive foundation for a future controller).

These exercise the new library surface only; the CLI drive path (PosDrive/run_segments) is
covered by the dry-run CSV oracle and is intentionally untouched here.
"""
from pico_esc import ESC, EscConfig, Telem
from pico_esc.drive import PosDrive
from pico_esc.link import RealClock, SimClock
from pico_esc.sim import SimEncEscHost


class _FakeLink:
    def __init__(self, tele_lines):
        self._tele = tele_lines

    def cmd(self, line, timeout=30.0):
        return self._tele if line.startswith("tele") else []


def test_read_tele_parses_full_line():
    d = PosDrive(_FakeLink(["tele|1234|12.30|5|42|7"]), 1, 300, RealClock(), verbose=False)
    t = d.read_tele()
    assert isinstance(t, Telem)
    assert (t.rpm, t.volts, t.amps, t.temp, t.stress) == (1234, 12.30, 5, 42, 7)


def test_read_temp_delegates_to_read_tele():
    d = PosDrive(_FakeLink(["tele|1234|12.30|5|42|7"]), 1, 300, RealClock(), verbose=False)
    assert d.read_temp() == 42


def test_read_tele_none_when_absent():
    d = PosDrive(_FakeLink([]), 1, 300, RealClock(), verbose=False)
    assert d.read_tele() is None
    assert d.read_temp() is None


def test_esc_exposes_full_goal_api():
    esc = ESC(_FakeLink([]), 1)
    assert isinstance(esc.config, EscConfig)
    for verb in ("arm", "disarm", "thrust", "throttle", "encoder", "telemetry",
                 "temperature", "prepare", "enter", "restart"):
        assert callable(getattr(esc, verb)), f"ESC missing {verb}()"
    assert callable(esc.config.set) and callable(esc.config.write)


def test_esc_drives_sim_end_to_end():
    clock = SimClock()
    sim = SimEncEscHost(clock, seed=1234)
    esc = ESC(sim, 1, clock=clock)
    esc.arm()                       # bidir
    esc.thrust(200)
    enc = esc.encoder()
    assert enc is not None and enc.healthy
    tel = esc.telemetry()
    assert isinstance(tel, Telem) and tel.temp == 25
    assert esc.temperature() == 25
    assert esc.throttle(120) == 120
    esc.disarm()


def test_esc_config_set_kwargs():
    clock = SimClock()
    esc = ESC(SimEncEscHost(clock, seed=1234), 1, clock=clock)
    # comm_timing is a field the sim tracks; set(**kwargs) must write it (not "unchanged")
    assert esc.config.set(comm_timing=4) == 1
    assert esc.link.cfg["comm_timing"] == 4
