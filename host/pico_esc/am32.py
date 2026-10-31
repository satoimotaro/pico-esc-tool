# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""pico_esc.am32 — AM32 settings (EEPROM) codec and bootloader access over the Pico CLI.

AM32's bootloader speaks the same signal-wire command set as the SiLabs BLHeli one (BootInit,
SET_ADDRESS, SET_BUFFER, READ_FLASH 0x03, PROG_FLASH 0x01, RUN), so the firmware's generic
`enter` / `readflash` / `writeflash` / `run` commands reach it unchanged. What differs is the map:
the 192-byte settings block lives in flash (Inc/eeprom.h in am32-firmware/AM32), the firmware
name sits 32 bytes before it, and the bootloader's deviceInfo carries a pin code and a flash-size
code where SiLabs puts its signature.

Protocol-1 bootloaders (what an Aria flashed in 2024 carries) do not know the 0x20 "magic EEPROM"
address, so we address the block by its real flash offset, derived from the flash-size code.
"""
from __future__ import annotations

# deviceInfo byte 4 (flash-size code) -> settings block offset from 0x08000000.
# From AM32-bootloader bootloader/main.c (EEPROM_START_ADD per MCU).
EEPROM_OFFSET = {
    0x1F: 0x7C00,   # 32 KB parts: F051, F031, G031 ...
    0x35: 0xF800,   # 64 KB parts: AT32F421, G071, E230 ...
}
EEPROM_LEN = 192
NAME_LEN = 32           # firmware FILE_NAME, NUL padded, just below the settings block

# (offset, name, note). Names follow Inc/eeprom.h. Bytes 0..4 are identity, not settings.
FIELDS = [
    (0, "reserved_0", "0x01 once written; the bootloader won't start the app otherwise"),
    (1, "eeprom_version", ""),
    (2, "bootloader_version", "rewritten by the bootloader on every write"),
    (3, "version_major", ""),
    (4, "version_minor", ""),
    (5, "max_ramp", "0.1 %/ms units, 10..250"),
    (6, "minimum_duty_cycle", "0.2 % units, 1..50"),
    (7, "disable_stick_calibration", "0/1"),
    (8, "absolute_voltage_cutoff", "0.5 V units"),
    (9, "current_P", ""),
    (10, "current_I", ""),
    (11, "current_D", ""),
    (12, "active_brake_power", "1..5 % duty"),
    (13, "brake_on_zero_throttle", "1 coast, 2 brake, 3..9 delayed brake"),
    (17, "dir_reversed", "0/1"),
    (18, "bi_direction", "0/1 (3D: 48..1047 reverse, 1048.. forward)"),
    (19, "use_sine_start", "0/1, needs comp_pwm"),
    (20, "comp_pwm", "0/1"),
    (21, "variable_pwm", "0 fixed, 1/2 auto"),
    (22, "stuck_rotor_protection", "0/1"),
    (23, "advance_level", "10..42 (= 0..32 steps of timing advance)"),
    (24, "pwm_frequency", "kHz, 8..144"),
    (25, "startup_power", "50..150"),
    (26, "motor_kv", "kv = value*40 + 20"),
    (27, "motor_poles", "magnet poles"),
    (28, "brake_on_stop", "0 off, 1 drag, 2 active"),
    (29, "stall_protection", "0/1"),
    (30, "beep_volume", "0..11"),
    (31, "telemetry_on_interval", "0/1"),
    (32, "servo_low_threshold", ""),
    (33, "servo_high_threshold", ""),
    (34, "servo_neutral", ""),
    (35, "servo_dead_band", ""),
    (36, "low_voltage_cut_off", "0 off, 1 per cell, 2 absolute"),
    (37, "low_cell_volt_cutoff", "cell V = 2.50 + value/100"),
    (38, "rc_car_reverse", "0/1 (overrides a lot)"),
    (39, "use_hall_sensors", "0/1"),
    (40, "sine_mode_changeover_thottle_level", "5..25 %"),
    (41, "drag_brake_strength", "1..10"),
    (42, "driving_brake_strength", "1..10"),
    (43, "limit_temperature", "70..140 C, else off"),
    (44, "limit_current", "1..100 A, else off"),
    (45, "sine_mode_power", "1..10"),
    (46, "input_type", "0 auto, 1 dshot, 2 servo, 3 serial, 4 edtarm"),
    (47, "auto_advance", "0/1"),
]
BY_NAME = {name: off for off, name, _ in FIELDS}
SETTINGS = [f for f in FIELDS if f[0] >= 5]

# A sane starting point for a sensorless thruster on DShot. AM32 ships no defaults table (neither
# the firmware nor the configurator has one), and a block left at 0xFF reads as e.g. bi_direction
# ON, dir_reversed ON, and garbage limits — which is what an Aria re-flashed through a flight
# controller came back with. Values follow the ranges loadEEpromSettings() accepts.
DEFAULTS = {
    "max_ramp": 160, "minimum_duty_cycle": 1, "disable_stick_calibration": 0,
    "absolute_voltage_cutoff": 10, "current_P": 100, "current_I": 0, "current_D": 100,
    "active_brake_power": 0, "brake_on_zero_throttle": 0,
    "dir_reversed": 0, "bi_direction": 0, "use_sine_start": 0, "comp_pwm": 1, "variable_pwm": 1,
    "stuck_rotor_protection": 1, "advance_level": 26, "pwm_frequency": 24, "startup_power": 100,
    "brake_on_stop": 0, "stall_protection": 0, "beep_volume": 5, "telemetry_on_interval": 0,
    "servo_low_threshold": 128, "servo_high_threshold": 128, "servo_neutral": 128,
    "servo_dead_band": 50, "low_voltage_cut_off": 0, "low_cell_volt_cutoff": 50,
    "rc_car_reverse": 0, "use_hall_sensors": 0, "sine_mode_changeover_thottle_level": 5,
    "drag_brake_strength": 10, "driving_brake_strength": 10, "limit_temperature": 255,
    "limit_current": 255, "sine_mode_power": 5, "input_type": 1, "auto_advance": 0,
}


def eeprom_offset(flash_code: int) -> int:
    if flash_code not in EEPROM_OFFSET:
        raise ValueError(f"unknown AM32 flash-size code 0x{flash_code:02X}")
    return EEPROM_OFFSET[flash_code]


def decode(raw: bytes) -> dict[str, int]:
    return {name: raw[off] for off, name, _ in FIELDS}


def kv(raw: bytes) -> int:
    return raw[BY_NAME["motor_kv"]] * 40 + 20


def encode(raw: bytes, changes: dict[str, int]) -> bytes:
    """Return a copy of `raw` with `changes` applied. Identity bytes 0..4 cannot be set."""
    out = bytearray(raw)
    for name, v in changes.items():
        if name not in BY_NAME or BY_NAME[name] < 5:
            raise KeyError(f"not a settable AM32 field: {name}")
        if not 0 <= v <= 255:
            raise ValueError(f"{name}={v} out of byte range")
        out[BY_NAME[name]] = v
    return bytes(out)


class Am32:
    """An AM32 ESC held in its bootloader through the Pico (an EscHost)."""

    def __init__(self, host, idx: int):
        self.host, self.idx = host, idx
        self.flash_code = None

    def enter(self):
        # dev|<sig>|<bootVer>|<pages>: for AM32 sig = flash code<<8 | 0x06, bootVer = 0x06
        line = next(l for l in self.host.cmd(f"enter {self.idx}", 60) if l.startswith("dev|"))
        sig = int(line.split("|")[1], 16)
        self.flash_code = sig >> 8
        return line

    def read(self, off: int, n: int) -> bytes:
        out = b""
        while len(out) < n:
            k = min(128, n - len(out))
            line = next(l for l in self.host.cmd(f"readflash {self.idx} {off + len(out):X} {k}")
                        if l.startswith("data|"))
            out += bytes.fromhex(line.split("|")[1])
        return out

    def read_eeprom(self) -> bytes:
        return self.read(eeprom_offset(self.flash_code), EEPROM_LEN)

    def read_name(self) -> str:
        raw = self.read(eeprom_offset(self.flash_code) - NAME_LEN, NAME_LEN)
        return raw.split(b"\0")[0].decode("ascii", "replace")

    def write_eeprom(self, raw: bytes) -> bytes:
        """Write the whole block, then read it back. Returns what is on the ESC now."""
        if len(raw) != EEPROM_LEN or raw[0] != 0x01:
            raise ValueError("refusing to write a block that isn't 192 bytes starting 0x01")
        self.host.cmd(f"writeflash {self.idx} {eeprom_offset(self.flash_code):X} {raw.hex()}", 30)
        back = self.read_eeprom()
        # byte 2 is overwritten with the bootloader's own version on every write
        if back[:2] + back[3:] != raw[:2] + raw[3:]:
            raise RuntimeError("read-back differs from what was written")
        return back

    def run(self):
        self.host.cmd("run", 5)
