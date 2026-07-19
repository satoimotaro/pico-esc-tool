# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""Config codec round-trips: encode_overrides -> raw block -> decode must recover the values,
and the sine<->BEMF crossover math must produce a valid, ordered hysteresis window.
"""
import pytest

from pico_esc import config


def _apply(ovs, base=None):
    """Apply [(offset, byte)] to a 255-byte block (0x00-filled unless base given)."""
    raw = bytearray(base if base is not None else bytes(255))
    for off, byte in ovs:
        raw[off] = byte
    return bytes(raw)


def test_numeric_fields_roundtrip():
    settings = {
        "startup_power_min": 40, "startup_power_max": 66, "beep_strength": 60,
        "beacon_strength": 55, "temperature_protection": 1, "brake_on_stop": 1,
        "max_erpm": 100, "sine_hold_amp": 16, "sine_amp_max": 45, "sine_ramp": 7,
    }
    raw = _apply(config.encode_overrides(settings))
    out = config.decode(raw)["settings"]
    for k, v in settings.items():
        assert out[k] == v, f"{k}: {out[k]} != {v}"


def test_enum_fields_roundtrip():
    settings = {"motor_direction": "Bidirectional", "comm_timing": "Medium",
                "demag_compensation": "High"}
    raw = _apply(config.encode_overrides(settings))
    out = config.decode(raw)["settings"]
    assert out["motor_direction"] == "Bidirectional"
    assert out["comm_timing"] == "Medium"
    assert out["demag_compensation"] == "High"


def test_name_override_writes_16_bytes():
    ovs = config.encode_overrides({"name": "BlueGill1"})
    assert len(ovs) == config.NAME_LEN
    assert [o for o, _ in ovs] == list(range(config.NAME_OFF, config.NAME_OFF + config.NAME_LEN))
    raw = _apply(ovs)
    assert config.decode(raw)["identity"]["name"] == "BlueGill1"


def test_overrides_str_format():
    assert config.overrides_str([(0x0B, 0x03), (0x2F, 0x10)]) == "0B:03,2F:10"


def test_max_erpm_clamped_with_warning(capsys):
    (off, byte), = config.encode_overrides({"max_erpm": 200})
    assert off == config.FIELD_OFF["max_erpm"]
    assert byte == config.MAX_ERPM_UNITS
    assert "clamping" in capsys.readouterr().err


@pytest.mark.parametrize("up,dn", [(2000.0, 1500.0), (3000.0, 1600.0), (4000.0, 1400.0)])
def test_sine_crossover_roundtrip_ordered(up, dn):
    cu, cd = config.sine_crossover_bytes(up, dn)
    assert 1 <= cu <= 255
    assert 1 <= cd <= config.SINE_CROSS_DN_MAX_BYTE
    up_eff = cu * config.SINE_CROSS_UP_ERPM_PER_UNIT
    dn_eff = config.SINE_CROSS_DN_ERPM_NUM / cd
    assert dn_eff < up_eff  # a real hysteresis window


def test_sine_crossover_rejects_inverted_window():
    with pytest.raises(ValueError):
        config.sine_crossover_bytes(1400.0, 3000.0)  # down above up


def test_sine_crossover_rejects_out_of_band_up():
    with pytest.raises(ValueError):
        config.sine_crossover_bytes(100000.0, 1500.0)
