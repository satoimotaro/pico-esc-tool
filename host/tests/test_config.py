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


# --- ONE handoff-able window, not two [2026-08-01] -------------------------------------------------
# `sine_crossover_bytes` used to apply the CONTINUOUS pre-rounding band, one tick's worth stricter on
# the low side than the firmware's own truncated-tick test, and so refused cross_up 34 — a seam the
# 2026-07-31 bench sweeps hand off at on two different motors, and the byte `esc_config()` recommends.
def test_the_validator_accepts_exactly_what_the_firmware_hands_off_at():
    """The two predicates agree across the whole byte range. A validator that refuses a seam the
    hardware accepts costs a working configuration; one that accepts a seam the hardware refuses
    silently produces an ESC that never leaves sine. Neither is allowed, so they must be one test."""
    for byte in range(1, 256):
        up = byte * config.SINE_CROSS_UP_ERPM_PER_UNIT
        handoffable = config.up_erpm_handoffable(up)
        try:
            config.sine_crossover_bytes(up, up - 20.0)
            accepted = True
        except ValueError:
            accepted = False
        # sine_crossover_bytes has FURTHER checks (hysteresis, byte range) that can reject a
        # handoff-able seam, but it must never reject one for being OUT OF BAND when it is not, and
        # never accept one that is.
        if not handoffable:
            assert not accepted, f"byte {byte} ({up:.1f} eRPM) is not handoff-able but was accepted"


def test_cross_up_34_is_accepted_because_the_bench_hands_off_there():
    """The specific regression: 34 -> 1328.125 eRPM -> 30.12 ticks. Inside the firmware's truncated
    window (int() -> 30 <= SINE_CROSS_TICKS_MAX), outside the old continuous band."""
    up = 34 * config.SINE_CROSS_UP_ERPM_PER_UNIT
    assert config.up_erpm_handoffable(up)
    assert config.sine_crossover_bytes(up, up - 20.0)[0] == 34


def test_esc_config_seamless_down_no_longer_raises():
    """Its one concrete victim: `_seamless_cross_dn()` feeds `sine_crossover_bytes` the very seam
    `esc_config()` recommends, so the recommended configuration used to be unrepresentable."""
    mm = pytest.importorskip("pico_esc.motor_model")
    cfg = mm.MotorModel(930.0, 7, 11.2, 1.0, cross_up=34).esc_config(seamless_down=True)
    assert cfg["sine_cross_up"] == 34
    assert 1 <= cfg["sine_cross_dn"] <= config.SINE_CROSS_DN_MAX_BYTE


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


# --- shipped profiles must tag their own regimes consistently with the handoff [2026-08-01] --------
def test_every_shipped_profile_tags_regimes_consistently_with_the_handoff():
    """A `regime` tag is not decoration: `VelLoop._line_floor()` takes the MINIMUM rpm among the
    line-tagged points as the lowest speed genuinely reachable in 6-step, and the ABORT_STALL guard
    only arms at or above it. So one 6-step point mis-tagged `sine` raises that floor to the next
    line point and opens a band where the guard silently never fires.

    That is not hypothetical. `f2838_350kv_bench_20260726.yaml` tagged cmd 558 as `sine` while it
    measured 628.1 rpm — the handoff for its cross_up byte 36 fires at command 515.4, and 186.8 rpm at
    cmd 503 jumping to 628.1 at cmd 558 is a handoff, not forced sine. `_line_floor()` returned 1211.1
    instead of 628.1, leaving 583 rpm unguarded.

    The tag is checked against `MotorModel.handoff_cmd()` — the measured duty-floor trigger — because
    that is what the firmware actually switches on.
    """
    import glob
    import os
    import yaml
    from pico_esc.motor_model import MotorModel
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    bad = []
    for path in sorted(glob.glob(os.path.join(root, "profiles", "*.yaml"))):
        d = yaml.safe_load(open(path)) or {}
        pts = d.get("points")
        if not pts:
            continue
        # SIM profiles are exempt: their tags describe the MODEL that generated them, and flipping one
        # would assert a hardware fact that was never measured. They self-declare `source: sim` and say
        # so in their header. `vel_930kv_12n14p_sim.yaml` is currently one such disagreement (cmd 827
        # tagged sine where handoff_cmd says line) and it is velctl's --dry-run default, so it is noted
        # in the file itself rather than silently corrected here.
        if d.get("source") == "sim":
            continue
        cx = d.get("crossover") or {}
        byts = cx.get("bytes")
        if not byts:
            continue
        # handoff_cmd is KV- and PP-independent (it is a duty), so the placeholder motor is fine here.
        hoc = MotorModel(350.0, int(d.get("pole_pairs", 7)), 11.25, 1.0,
                         cross_up=float(byts[0])).handoff_cmd()
        if hoc is None:
            continue
        for p in pts:
            t, tag = float(p["thrust"]), p.get("regime")
            if tag is None or t <= 0:
                continue
            want = "line" if t >= hoc else "sine"
            if tag != want:
                bad.append(f"{os.path.basename(path)}: cmd {t:.0f} tagged {tag!r}, handoff_cmd "
                           f"{hoc:.1f} says {want!r} (measured {p['rpm']:.1f} rpm)")
    assert not bad, "regime tags disagree with the handoff:\n  " + "\n  ".join(bad)


# --- the write guard: which ESCs may these offsets be written to? ------------------------------
# The three headers are real, bench-observed on one ESC, not invented:
# BlueGill S4 = 0.21/226, stock Bluejay v0.21.0 = 0.21/208, BLHeli-S Rev16.7 = 16.7/33. All three
# carry the SAME layout tag `#A_H_30#`, which is exactly why the flash compat check cannot catch
# this — it compares the FET pin map, not the config layout.
def _hdr(eeprom_hi, eeprom_lo, layout_rev):
    raw = bytearray(bytes(255))
    raw[0x00], raw[0x01], raw[0x02] = eeprom_hi, eeprom_lo, layout_rev
    return bytes(raw)


BLUEGILL_HDR = _hdr(0, 21, 226)
BLUEJAY_HDR = _hdr(0, 21, 208)
BLHELI_S_HDR = _hdr(16, 7, 33)
ALL_FIELDS = sorted(config.FIELD_OFF)


def test_bluegill_esc_accepts_every_field():
    fatal, refused = config.write_compat(BLUEGILL_HDR, ALL_FIELDS)
    assert fatal is None and refused == {}


def test_blheli_s_refuses_everything_including_the_shared_settings():
    """The whole block is fatal, not just the BlueGill-only tail: on a different EEPROM revision
    even `brake_on_stop` is not promised to live at 0x27."""
    fatal, refused = config.write_compat(BLHELI_S_HDR, ALL_FIELDS)
    assert fatal is not None and "16.7" in fatal
    assert refused == {}


def test_stock_bluejay_takes_the_shared_settings_but_not_the_sine_block():
    fatal, refused = config.write_compat(BLUEJAY_HDR, ALL_FIELDS)
    assert fatal is None
    assert set(refused) == set(config.BLUEGILL_ONLY_FIELDS)
    # This is the case that must NOT become a blanket refusal: snapshots/stock-comparable.yaml
    # writes these 13 to stock Bluejay on purpose, as the firmware control.
    assert "brake_on_stop" not in refused and "comm_timing" not in refused


def test_the_bluegill_only_set_is_exactly_the_appended_block():
    assert config.BLUEGILL_ONLY_FIELDS == {
        "comm_timing_angle", "max_erpm", "lowspeed_damping",
        "sine_mode", "sine_hold_amp", "sine_amp_max", "sine_ramp",
        "sine_cross_up", "sine_cross_dn",
    }
    assert all(config.FIELD_OFF[k] >= 0x2B for k in config.BLUEGILL_ONLY_FIELDS)


def test_only_the_requested_keys_are_refused():
    _, refused = config.write_compat(BLUEJAY_HDR, ["brake_on_stop", "sine_mode"])
    assert set(refused) == {"sine_mode"}


@pytest.mark.parametrize("raw", [None, b"", bytes(2)])
def test_an_unreadable_block_fails_CLOSED(raw):
    """Not knowing the layout is the hazard itself, so it must refuse rather than assume."""
    fatal, refused = config.write_compat(raw, ALL_FIELDS)
    assert fatal is not None and refused == {}


# --- the CLI wiring around it -----------------------------------------------------------------
def test_guard_write_filters_and_exits(monkeypatch, capsys):
    """`write_compat` is only useful if `set`/`apply` actually consult it, so test the wiring:
    drop the refused keys, keep the rest, and exit non-zero rather than write on a fatal."""
    import esctool

    settings = {"brake_on_stop": 1, "sine_mode": 2}
    monkeypatch.setattr(esctool, "_read_raw", lambda dev, i: BLUEJAY_HDR)
    assert esctool._guard_write(None, 1, settings, force=False) == {"brake_on_stop": 1}
    assert "skipping 'sine_mode'" in capsys.readouterr().out

    monkeypatch.setattr(esctool, "_read_raw", lambda dev, i: BLHELI_S_HDR)
    with pytest.raises(SystemExit):
        esctool._guard_write(None, 1, settings, force=False)
    assert esctool._guard_write(None, 1, settings, force=True) == settings   # --force still writes

    monkeypatch.setattr(esctool, "_read_raw", lambda dev, i: BLUEGILL_HDR)
    assert esctool._guard_write(None, 1, settings, force=False) == settings
