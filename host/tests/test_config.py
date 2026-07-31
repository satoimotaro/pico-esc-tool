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


# --------------------------------------------------------------------------------------------
# STOCK (non-BlueGill) FIRMWARE. The README claims this tool "works with stock Bluejay and
# BLHeli-S too", and the config codec is built for it: 0x04..0x27 are the shared BLHeli-S family
# offsets and the BlueGill parameters were APPENDED at 0x2B..0x33, where a stock ESC has never
# written anything. Nothing pinned that claim, so these do.
#
# The two stock representations are both covered because they are genuinely different: a Bluejay
# ESC whose EEPROM page was erased reads 0xFF in the unused slots, while a block that was zero-
# filled (a fresh BLHeli-S layout, or our own `_apply()` default) reads 0x00. Both must come out
# as "there is no sine crossover here", and neither may be mistaken for a configured one.
# --------------------------------------------------------------------------------------------
_SHARED = {                       # only fields a stock BLHeli-S/Bluejay ESC actually has
    "motor_direction": "Bidirectional", "comm_timing": "MediumHigh", "demag_compensation": "High",
    "startup_power_min": 30, "startup_power_max": 25, "beep_strength": 40, "beacon_strength": 80,
    "beacon_delay": 4, "temperature_protection": 7, "low_rpm_power_protection": 1,
    "brake_on_stop": 0,
}
_BLUEGILL_SLOTS = ("comm_timing_angle", "max_erpm", "lowspeed_damping", "sine_mode",
                   "sine_hold_amp", "sine_amp_max", "sine_ramp", "sine_cross_up", "sine_cross_dn")


@pytest.mark.parametrize("fill,sentinel", [(0xFF, 255), (0x00, 0)])
def test_a_stock_block_decodes_its_shared_fields_and_reports_no_bluegill_params(fill, sentinel):
    """Reading a stock ESC must recover every shared field and leave the BlueGill slots as the
    sentinel — not as a plausible-looking configuration."""
    raw = _apply(config.encode_overrides(_SHARED), base=bytes([fill]) * 255)
    out = config.decode(raw)["settings"]
    for k, v in _SHARED.items():
        assert out[k] == v, f"shared field {k}: {out[k]!r} != {v!r}"
    for k in _BLUEGILL_SLOTS:
        assert out[k] == sentinel, f"{k} should read the {fill:#04x} sentinel, got {out[k]!r}"


@pytest.mark.parametrize("cross_up", [255, 0])
def test_a_stock_cross_up_can_never_be_mistaken_for_a_configured_crossover(cross_up):
    """THE ONE THAT MATTERS. `sine_cross_up` read off a stock ESC is a sentinel, not a threshold,
    and the parametric model must refuse to derive a handoff from it — otherwise every regime
    boundary, the crossover gap and the soft sensor's validity window are computed for a
    forced-sine branch that this firmware does not have.

    Both sentinels land outside the firmware's handoff-able seed-tick window from opposite sides
    (0xFF -> 9961 eRPM -> 4 ticks, too FAST; 0x00 -> 0 eRPM -> infinite ticks), so the refusal is
    a property of the band, not a special case anyone has to remember to write."""
    from pico_esc.motor_model import MotorModel
    m = MotorModel(350, 7, 11.1, cross_up=cross_up)
    assert m.handoff_possible() is False
    assert m.handoff_cmd() is None and m.handoff_mech() is None and m.sixstep_floor_rpm() is None
    # and every derived path degrades to "forced sine everywhere", not to a fictitious 6-step one
    assert m.in_gap(300) is False and m.regime(300) == "sine"
    assert m.thrust_for(300) == pytest.approx(m.cmd_for_sine(300))
    with pytest.raises(ValueError, match="cross_up"):
        m.to_profile()                        # a curve with no 6-step section would mis-scale velctl
