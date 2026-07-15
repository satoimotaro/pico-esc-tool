#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""esctool — BLHeli-Configurator-like CLI for the RP2040 ESC tool (firmware: esc_tool).

Talks to the Pico over USB-CDC serial (auto-detected by VID 2E8A) with a small text protocol.
Commands: list, connect, read, set, apply <profile.yaml>, flash <hex>, run/disconnect.

  python esctool.py list
  python esctool.py read 0 -o config.yaml
  python esctool.py apply all host/profiles/blheli-s-default.yaml
"""
from __future__ import annotations

import argparse
import sys
import time

try:  # Windows consoles default to cp932 and crash on any non-ASCII output
    sys.stdout.reconfigure(errors="replace")
except Exception:
    pass

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial required:  pip install pyserial")

try:
    import yaml  # optional; a minimal emitter is used if absent
except ImportError:
    yaml = None

RPI_VID = 0x2E8A

DIRECTION = {1: "Normal", 2: "Reversed", 3: "Bidirectional", 4: "Bidirectional-Reversed"}
TIMING    = {1: "Low", 2: "MediumLow", 3: "Medium", 4: "MediumHigh", 5: "High"}
DEMAG     = {1: "Off", 2: "Low", 3: "High"}
MODE      = {0x55AA: "Multi", 0xA55A: "Main", 0x5AA5: "Tail"}

# Writable settings: field name -> offset in the config block. (identity/mode/raw_hex are read-only.)
FIELD_OFF = {
    "motor_direction": 0x0B, "comm_timing": 0x15, "demag_compensation": 0x1F,
    "startup_power_min": 0x04, "startup_power_max": 0x07, "startup_beep": 0x05,
    "pwm_frequency": 0x0A, "beep_strength": 0x1B, "beacon_strength": 0x1C,
    "beacon_delay": 0x1D, "temperature_protection": 0x23,
    "low_rpm_power_protection": 0x09, "brake_on_stop": 0x27,
    # BlueGill-only params (appended after Bluejay's block; ignored by stock Bluejay).
    # On a stock Bluejay ESC these slots read 0xFF ("off").
    "comm_timing_angle": 0x2B, "max_erpm": 0x2C, "lowspeed_damping": 0x2D,
    # BlueGill S1 forced-commutation stepper mode (0xFF = off/default on stock/older fw).
    "sine_mode": 0x2E, "sine_hold_amp": 0x2F, "sine_amp_max": 0x30, "sine_ramp": 0x31,
    # BlueGill S3 sine<->BEMF crossover thresholds (0 = off; see --sine-crossover-erpm).
    "sine_cross_up": 0x32, "sine_cross_dn": 0x33,
}
FIELD_ENUM = {"motor_direction": DIRECTION, "comm_timing": TIMING, "demag_compensation": DEMAG}
NAME_OFF, NAME_LEN = 0x60, 16

# BlueGill max_erpm (0x2C) is in units of 1000 eRPM. The firmware's 80000/N decode is
# exact only for N <= 136 and the high-rpm path bypasses the governor above ~156k eRPM,
# so the effective ceiling is <= 136k eRPM; values above that are clamped (with a warning).
# Note: for all three BlueGill params a stored byte of 255 (0xFF) reads as OFF on firmware,
# so 255 is not a usable magnitude.
MAX_ERPM_UNITS = 136

# BlueGill S3 sine<->BEMF crossover unit math. MUST match the firmware and
# tools/sim/sine_drive_model.py print_s3_crossover_section() exactly.
#  * Up threshold (Cross_Up, 0x32) is in Sine_Inc_H units: up eRPM = Sine_Inc*10000/65536,
#    and one Sine_Inc_H unit = 256 in Sine_Inc, so eRPM/unit = 256*10000/65536 = 39.0625.
#  * Down threshold (Cross_Dn, 0x33) is in Comm_Period4x_H units (INVERSE): eRPM = 80e6/
#    Comm_Period4x, ~312500 per Comm_Period4x_H unit. Firmware clamps Cross_Dn to <= 0xEF so
#    the down-handoff always fires before the stock 0xF0 (~1302 eRPM) min-speed exit.
SINE_CROSS_UP_ERPM_PER_UNIT = 10000.0 / 256.0          # = 39.0625
SINE_CROSS_DN_ERPM_NUM = 312500.0                      # eRPM = NUM / Cross_Dn
SINE_CROSS_DN_MAX_BYTE = 0xEF                          # firmware clamp ceiling (>=0xF0 disabled)
# The firmware seeds Comm_Period4x from a 4-sector tick window and only fires the up-handoff
# when that window is in [SINE_CROSS_TICKS_MIN, SINE_CROSS_TICKS_MAX] ticks (else it refuses
# and stays in forced sine). The window is 4*10000/up_eRPM ticks, so the usable up-eRPM band is
# [4*10000/MAX, 4*10000/MIN]. Requesting an up threshold outside it would silently never hand
# off, so reject it here with the reason. Keep MIN/MAX in lockstep with Bluejay.asm.
SINE_CROSS_TICKS_MIN = 9                                # matches SINE_CROSS_TICKS_MIN (asm)
SINE_CROSS_TICKS_MAX = 30                               # matches SINE_CROSS_TICKS_MAX (asm)
SINE_CROSS_UP_ERPM_MIN = 4 * 10000.0 / SINE_CROSS_TICKS_MAX   # ~1333 eRPM
SINE_CROSS_UP_ERPM_MAX = 4 * 10000.0 / SINE_CROSS_TICKS_MIN   # ~4444 eRPM


def sine_crossover_bytes(up_erpm: float, dn_erpm: float) -> tuple[int, int]:
    """(up_erpm, dn_erpm) -> (cross_up, cross_dn) bytes, validating exactly as the firmware
    guards + hysteresis require. Returns rounded bytes; raises ValueError on any violation.
    Recomputes the EFFECTIVE speeds back from the rounded bytes and rejects an inverted or
    collapsed hysteresis window (dn_eff >= up_eff), and an up threshold outside the firmware's
    handoff-able band (where the seed would be too coarse / below the BEMF floor)."""
    if up_erpm <= 0 or dn_erpm <= 0:
        raise ValueError("both eRPM values must be positive")
    if not SINE_CROSS_UP_ERPM_MIN <= up_erpm <= SINE_CROSS_UP_ERPM_MAX:
        raise ValueError(f"up eRPM {up_erpm:g} outside the handoff-able band "
                         f"~{SINE_CROSS_UP_ERPM_MIN:.0f}..{SINE_CROSS_UP_ERPM_MAX:.0f} eRPM: below it "
                         f"is under the BEMF floor, above it the 4-sector seed gets too coarse "
                         f"(>~{100.0/SINE_CROSS_TICKS_MIN:.0f}%) and the firmware would refuse to hand off")
    cross_up = round(up_erpm / SINE_CROSS_UP_ERPM_PER_UNIT)
    cross_dn = round(SINE_CROSS_DN_ERPM_NUM / dn_erpm)
    if not 1 <= cross_up <= 255:
        raise ValueError(f"up eRPM {up_erpm:g} -> Cross_Up byte {cross_up} out of range 1..255 "
                         f"(usable up range ~{SINE_CROSS_UP_ERPM_PER_UNIT:.2f}.."
                         f"{255*SINE_CROSS_UP_ERPM_PER_UNIT:.0f} eRPM)")
    if cross_dn > SINE_CROSS_DN_MAX_BYTE:
        min_dn = SINE_CROSS_DN_ERPM_NUM / SINE_CROSS_DN_MAX_BYTE
        raise ValueError(f"down eRPM {dn_erpm:g} too low -> Cross_Dn byte {cross_dn} exceeds "
                         f"0x{SINE_CROSS_DN_MAX_BYTE:02X}; must be >= ~{min_dn:.0f} eRPM "
                         f"(the firmware would clamp it past the stock min-speed exit)")
    if cross_dn < 1:
        raise ValueError(f"down eRPM {dn_erpm:g} -> Cross_Dn byte {cross_dn} out of range 1..255")
    up_eff = cross_up * SINE_CROSS_UP_ERPM_PER_UNIT
    dn_eff = SINE_CROSS_DN_ERPM_NUM / cross_dn
    if dn_eff >= up_eff:
        raise ValueError(f"no hysteresis window: effective down {dn_eff:.0f} eRPM >= effective up "
                         f"{up_eff:.0f} eRPM (down must be below up). Widen the split.")
    return cross_up, cross_dn


def encode_value(field: str, value) -> int:
    """Field value (int, numeric string, or enum name) -> byte."""
    enum = FIELD_ENUM.get(field)
    if enum and isinstance(value, str) and not value.lstrip("-").isdigit():
        rev = {v.lower(): k for k, v in enum.items()}
        if value.lower() not in rev:
            raise ValueError(f"{field}: '{value}' not in {list(enum.values())}")
        return rev[value.lower()]
    v = int(value)
    if field == "max_erpm" and v > MAX_ERPM_UNITS:
        print(f"warning: max_erpm={v} exceeds effective ceiling {MAX_ERPM_UNITS} "
              f"(~{MAX_ERPM_UNITS}k eRPM); clamping to {MAX_ERPM_UNITS}", file=sys.stderr)
        v = MAX_ERPM_UNITS
    if field == "sine_mode" and not 0 <= v <= 2:
        # 0=off (stock), 1=S1 forced-commutation stepper, 2=S2 min-clamp micro-stepping.
        print(f"warning: sine_mode={v} out of range 0..2; clamping to "
              f"{max(0, min(2, v))}", file=sys.stderr)
        v = max(0, min(2, v))
    return v & 0xFF


def encode_overrides(settings: dict) -> list[tuple[int, int]]:
    """Settings dict -> [(page_offset, byte)]. Unknown keys are skipped; 'name' -> 16 bytes."""
    ovs: list[tuple[int, int]] = []
    for k, v in settings.items():
        if k == "name":
            nm = str(v)[:NAME_LEN].ljust(NAME_LEN)
            ovs += [(NAME_OFF + j, ord(c) & 0xFF) for j, c in enumerate(nm)]
        elif k in FIELD_OFF:
            ovs.append((FIELD_OFF[k], encode_value(k, v)))
    return ovs


def overrides_str(ovs: list[tuple[int, int]]) -> str:
    return ",".join(f"{o:02X}:{b:02X}" for o, b in ovs)


def find_pico(port: str | None) -> str:
    if port:
        return port
    # retry briefly: right after an upload the CDC port takes a moment to re-enumerate
    for _ in range(40):
        for p in list_ports.comports():
            if p.vid == RPI_VID:
                return p.device
        time.sleep(0.1)
    sys.exit("no RP2040 (VID 2E8A) found - is esc_tool flashed and the monitor closed?")


class EscHost:
    """Line-based transport: send a command, collect reply lines until 'ok'/'err'."""

    def __init__(self, port: str | None = None):
        self.ser = None
        p = find_pico(port)
        for _ in range(30):                         # port can be briefly un-openable after an upload
            try:
                self.ser = serial.Serial(p, 115200, timeout=10)
                break
            except serial.SerialException:
                time.sleep(0.15)
                p = find_pico(port)
        if self.ser is None:
            sys.exit(f"could not open {p}")
        time.sleep(0.3)
        self.ser.reset_input_buffer()

    def cmd(self, line: str, timeout: float = 30.0) -> list[str]:
        self.ser.write((line + "\n").encode())
        self.ser.flush()
        out, end = [], time.time() + timeout
        while time.time() < end:
            ln = self.ser.readline().decode("utf-8", "replace").strip()
            if not ln:
                continue
            if ln == "ok":
                return out
            if ln.startswith("err"):
                raise RuntimeError(f"device: {ln}")
            out.append(ln)
        raise TimeoutError(f"no 'ok' for: {line}")

    def close(self):
        self.ser.close()


def _tag(raw: bytes, off: int, n: int = 16) -> str:
    s = bytearray()
    for b in raw[off:off + n]:
        if b in (0x00, 0xFF):
            break
        s.append(b if 32 <= b < 127 else ord("."))
    return s.decode().rstrip()


def decode(raw: bytes) -> dict:
    """Decode the BLHeli-S/BlueJay config block (offsets shared across the family)."""
    g = lambda o: raw[o]
    mode = (raw[0x0D] << 8) | raw[0x0E]
    return {
        "identity": {
            "name": _tag(raw, 0x60),
            "layout": _tag(raw, 0x40),
            "mcu": _tag(raw, 0x50),
            "eeprom_revision": f"{g(0x00)}.{g(0x01)}",
            "layout_revision": g(0x02),
            "mode": MODE.get(mode, f"0x{mode:04X}"),
        },
        "settings": {
            "motor_direction": DIRECTION.get(g(0x0B), g(0x0B)),
            "comm_timing": TIMING.get(g(0x15), g(0x15)),
            "demag_compensation": DEMAG.get(g(0x1F), g(0x1F)),
            "startup_power_min": g(0x04),
            "startup_power_max": g(0x07),
            "startup_beep": g(0x05),
            "pwm_frequency": g(0x0A),
            "beep_strength": g(0x1B),
            "beacon_strength": g(0x1C),
            "beacon_delay": g(0x1D),
            "temperature_protection": g(0x23),
            "low_rpm_power_protection": g(0x09),
            "brake_on_stop": g(0x27),
            # BlueGill params: raw ints (0 = off/default, 0xFF = off on stock Bluejay).
            "comm_timing_angle": g(0x2B),
            "max_erpm": g(0x2C),
            "lowspeed_damping": g(0x2D),
            # BlueGill S1 forced-commutation stepper mode (0xFF = off/default).
            "sine_mode": g(0x2E),
            "sine_hold_amp": g(0x2F),
            "sine_amp_max": g(0x30),
            "sine_ramp": g(0x31),
            # BlueGill S3 crossover thresholds (0/0xFF = off). Raw bytes; up is in Sine_Inc_H
            # units (~39.06 eRPM/unit), dn is in Comm_Period4x_H units (~312500/unit, inverse).
            "sine_cross_up": g(0x32),
            "sine_cross_dn": g(0x33),
        },
        "raw_hex": raw.hex().upper(),
    }


def _emit_yaml(d: dict) -> str:
    if yaml:
        return yaml.safe_dump(d, sort_keys=False, default_flow_style=False)
    def q(v):
        if isinstance(v, str):                      # always quote strings so '#', ':', '0.21'
            return '"' + v.replace("\\", "\\\\").replace('"', '\\"') + '"'
        return v
    def rec(o, ind=0):
        out = []
        pad = "  " * ind
        for k, v in o.items():
            if isinstance(v, dict):
                out.append(f"{pad}{k}:")
                out.append(rec(v, ind + 1))
            else:
                out.append(f"{pad}{k}: {q(v)}")
        return "\n".join(out)
    return rec(d) + "\n"


def load_yaml(path: str) -> dict:
    if yaml:
        with open(path, encoding="utf-8") as fh:
            return yaml.safe_load(fh) or {}
    # minimal fallback for our own 2-level export format (key: value, one nesting level)
    root: dict = {}
    cur = root
    for raw in open(path, encoding="utf-8"):
        if not raw.strip() or raw.lstrip().startswith("#"):
            continue
        indent = len(raw) - len(raw.lstrip())
        key, _, val = raw.strip().partition(":")
        key, val = key.strip(), val.strip()
        if val == "":
            cur = root[key] = {}
        else:
            if val[0] in "\"'":                       # quoted string (keep as-is, drop trailer)
                q = val[0]
                end = val.find(q, 1)
                val = val[1:end] if end > 0 else val[1:]
            else:
                cpos = val.find(" #")                 # strip an inline comment
                if cpos >= 0:
                    val = val[:cpos].rstrip()
                try:
                    val = int(val)
                except ValueError:
                    pass
            (cur if indent else root)[key] = val
    return root


def _finish(dev: EscHost, index: int, run: bool):
    """Config commands keep the ESC in the bootloader (a session, so repeated commands don't
    reboot it). Restart it now if --run, else remind the user."""
    if run:
        dev.cmd(f"run {index}")
        print(f"ESC {index} restarted.")
    else:
        print(f"ESC {index} held in bootloader (motor off): `esctool run {index}` to restart it, "
              f"or run another command (it reuses the session).")


def _apply_overrides(dev: EscHost, index: int, ovs: list[tuple[int, int]]):
    if not ovs:
        sys.exit("no writable settings to apply")
    lines = dev.cmd(f"editpage {index} {overrides_str(ovs)}", timeout=30)
    if any("unchanged" in l for l in lines):
        print(f"ESC {index}: already matches (no flash write)")
    else:
        print(f"applied {len(ovs)} byte(s) to ESC {index}; verified on device")


def resolve_indices(dev: EscHost, index_arg) -> list[int]:
    """An index, or 'all' -> every present ESC (via scan)."""
    if str(index_arg).lower() == "all":
        rows = [ln for ln in dev.cmd("scan") if ln.startswith("esc|")]
        idx = [int(r.split("|")[1]) for r in rows if len(r.split("|")) > 3 and r.split("|")[3] == "1"]
        if not idx:
            sys.exit("no ESCs present")
        return idx
    return [int(index_arg)]


def cmd_set(dev: EscHost, args):
    settings = {}
    for kv in args.assign:
        k, _, v = kv.partition("=")
        if not v:
            sys.exit(f"bad assignment '{kv}' (want key=value)")
        settings[k.strip()] = v.strip()
    if getattr(args, "sine_crossover_erpm", None):
        parts = args.sine_crossover_erpm.split(",")
        if len(parts) != 2:
            sys.exit("--sine-crossover-erpm expects UP,DN (two eRPM values, comma-separated)")
        try:
            up_erpm, dn_erpm = float(parts[0]), float(parts[1])
            cross_up, cross_dn = sine_crossover_bytes(up_erpm, dn_erpm)
        except ValueError as e:
            sys.exit(f"--sine-crossover-erpm: {e}")
        settings["sine_cross_up"] = cross_up
        settings["sine_cross_dn"] = cross_dn
        up_eff = cross_up * SINE_CROSS_UP_ERPM_PER_UNIT
        dn_eff = SINE_CROSS_DN_ERPM_NUM / cross_dn
        print(f"sine crossover: up Cross_Up=0x{cross_up:02X} (~{up_eff:.0f} eRPM), "
              f"dn Cross_Dn=0x{cross_dn:02X} (~{dn_eff:.0f} eRPM)")
    ovs = encode_overrides(settings)
    for i in resolve_indices(dev, args.index):
        _apply_overrides(dev, i, ovs)
        _finish(dev, i, args.run)


def cmd_apply(dev: EscHost, args):
    doc = load_yaml(args.profile)
    settings = dict(doc.get("settings", {}))
    if args.with_name and isinstance(doc.get("identity"), dict) and "name" in doc["identity"]:
        settings["name"] = doc["identity"]["name"]
    if args.name is not None:
        settings["name"] = args.name
    ovs = encode_overrides(settings)
    for i in resolve_indices(dev, args.index):
        print(f"applying '{args.profile}' to ESC {i}: {', '.join(settings)}")
        _apply_overrides(dev, i, ovs)
        _finish(dev, i, args.run)


def cmd_connect(dev: EscHost, args):
    for i in resolve_indices(dev, args.index):
        line = next((l for l in dev.cmd(f"enter {i}") if l.startswith("dev|")), None)
        if not line:
            print(f"ESC {i}: could not connect")
            continue
        _, sig, boot, pages = line.split("|")
        print(f"ESC {i} connected (held): sig={sig} bootVer={boot} bootPages={pages}")


def cmd_run(dev: EscHost, args):
    dev.cmd("disconnect")
    print("released bootloader session; ESC(s) restarted.")


APP_END, EEPROM_BASE, BOOT_BASE = 0x1A00, 0x1A00, 0x1C00
# MCU-tag fragment -> signature. Mirror of lib/blheli_bl kMcuTable (keep in sync when adding MCUs).
SIG_FOR_MCU = {"B10": "E8B1", "B21": "E8B2", "B51": "E8B5"}


def parse_hex(path: str):
    """Intel-HEX -> (app{addr:byte} <0x1A00, ident{addr:byte} 0x1A00..0x1BFF, boot_byte_count)."""
    app, ident, boot, upper = {}, {}, 0, 0
    for ln in open(path, encoding="utf-8"):
        ln = ln.strip()
        if not ln.startswith(":"):
            continue
        rec = bytes.fromhex(ln[1:])
        if sum(rec) & 0xFF:
            raise ValueError(f"bad checksum: {ln}")
        bc, addr, tt, data = rec[0], (rec[1] << 8) | rec[2], rec[3], rec[4:4 + rec[0]]
        if tt == 4:
            upper = (data[0] << 8) | data[1]
        elif tt == 0:
            for k, b in enumerate(data):
                a = (upper << 16) | (addr + k)
                if a < APP_END:
                    app[a] = b
                elif a < BOOT_BASE:
                    ident[a] = b
                else:
                    boot += 1
    return app, ident, boot


def hex_tag(ident: dict, off: int) -> str:
    s = bytearray()
    for j in range(16):
        b = ident.get(EEPROM_BASE + off + j, 0xFF)
        if b in (0, 0xFF):
            break
        s.append(b)
    return s.decode("ascii", "replace").rstrip()


def _pages_from(app: dict, ident: dict) -> dict:
    """Assemble {page_addr: bytearray(512)} for the app pages plus the config page (firmware
    defaults from the HEX's eeprom section -> auto-applied config)."""
    pages: dict[int, bytearray] = {}
    for a, b in app.items():
        pages.setdefault(a & ~0x1FF, bytearray(b"\xff" * 512))[a & 0x1FF] = b
    if ident:
        buf = bytearray(b"\xff" * 512)
        for a, b in ident.items():
            buf[a - EEPROM_BASE] = b
        pages[EEPROM_BASE] = buf
    return pages


def cmd_flash(dev: EscHost, args):
    app, ident, boot = parse_hex(args.hexfile)
    if not app:
        sys.exit("HEX has no application data")
    fw_layout, fw_mcu = hex_tag(ident, 0x40), hex_tag(ident, 0x50)
    i = int(args.index)

    dev_line = next((l for l in dev.cmd(f"enter {i}") if l.startswith("dev|")), None)
    if not dev_line:
        sys.exit(f"ESC {i}: could not connect")
    esc_sig = dev_line.split("|")[1]
    cfg = next((l for l in dev.cmd(f"read {i}") if l.startswith("cfg|")), None)
    esc_layout = _tag(bytes.fromhex(cfg.split("|", 1)[1]), 0x40) if cfg else ""

    exp_sig = next((v for k, v in SIG_FOR_MCU.items() if k in fw_mcu), None)
    mcu_ok = exp_sig is not None and exp_sig == esc_sig
    layout_ok = bool(fw_layout) and fw_layout == esc_layout
    print(f"ESC {i}: sig={esc_sig} layout='{esc_layout}'   HEX: mcu='{fw_mcu}' layout='{fw_layout}'")
    print(f"compat: MCU {'OK' if mcu_ok else 'MISMATCH'}, layout {'OK' if layout_ok else 'MISMATCH'}")
    if not (mcu_ok and layout_ok) and not args.force:
        dev.cmd(f"run {i}")
        sys.exit("INCOMPATIBLE firmware - refusing (use --force to override).")
    if not args.yes:
        dev.cmd(f"run {i}")
        sys.exit("this ERASES + writes the ESC app. Re-run with --yes to proceed.")
    if boot:
        print(f"note: {boot} bootloader byte(s) in HEX are skipped (BL preserved)")

    pages = _pages_from(app, ident)
    for n, p in enumerate(sorted(pages), 1):
        buf = pages[p]
        dev.cmd(f"erase {i} {p:04X}")
        for off in (0, 256):
            dev.cmd(f"writeflash {i} {p + off:04X} {buf[off:off + 256].hex()}")
        rb = bytearray()
        for off in (0, 256):
            r = next((l for l in dev.cmd(f"readflash {i} {p + off:04X} 256") if l.startswith("data|")), "data|")
            rb += bytes.fromhex(r.split("|", 1)[1])
        if rb != buf:
            dev.cmd(f"run {i}")
            sys.exit(f"verify FAILED at page 0x{p:04X}")
        print(f"  [{n}/{len(pages)}] page 0x{p:04X} written + verified")
    dev.cmd(f"run {i}")
    print("FLASH OK: app programmed + verified, firmware default config applied. ESC restarted.")


def cmd_list(dev: EscHost, args):
    rows = [ln for ln in dev.cmd("scan") if ln.startswith("esc|")]
    if not rows:
        print("no ESCs reported")
        return
    print(f"{'idx':>3}  {'pin':>3}  {'sig':>5}  {'layout':<12} {'name':<16} {'fw':>6}")
    for r in rows:
        f = r.split("|")
        idx, pin, present = f[1], f[2], f[3]
        if present != "1":
            print(f"{idx:>3}  {pin:>3}  {'--':>5}  {'(no ESC / not entering bootloader)'}")
            continue
        sig, _boot, layout, name, fw = f[4], f[5], f[6], f[7], f[8]
        print(f"{idx:>3}  {pin:>3}  {sig:>5}  {layout:<12} {name:<16} {fw:>6}")


def cmd_read(dev: EscHost, args):
    indices = resolve_indices(dev, args.index)
    for i in indices:
        lines = dev.cmd(f"read {i}")
        cfg = next((l for l in lines if l.startswith("cfg|")), None)
        if not cfg:
            print(f"ESC {i}: no config returned")
            continue
        raw = bytes.fromhex(cfg.split("|", 1)[1])
        text = _emit_yaml({"esc": i, **decode(raw)})
        if args.out and len(indices) == 1:
            with open(args.out, "w", encoding="utf-8", newline="\n") as fh:
                fh.write(text)
            print(f"wrote {args.out} ({len(raw)} config bytes)")
        else:
            sys.stdout.write(text)
        _finish(dev, i, args.run)


def main():
    ap = argparse.ArgumentParser(description="BLHeli-S ESC CLI (esc_tool firmware)")
    ap.add_argument("--port", help="serial port (default: auto-detect VID 2E8A)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list", help="scan and list connected ESCs")
    rd = sub.add_parser("read", help="read ESC config (index or 'all')")
    rd.add_argument("index", help="ESC index from 'list', or 'all'")
    rd.add_argument("-o", "--out", help="write YAML to this file (single ESC only)")
    rd.add_argument("-r", "--run", action="store_true", help="restart the ESC afterward (else held)")
    st = sub.add_parser("set", help="change settings on an ESC (index or 'all')")
    st.add_argument("index", help="ESC index or 'all'")
    st.add_argument("assign", nargs="*", help="e.g. motor_direction=Reversed beep_strength=60")
    st.add_argument("--sine-crossover-erpm", metavar="UP,DN",
                    help="set sine<->BEMF crossover from two eRPM values (up,down); down must be "
                         "below up. Converts to the Cross_Up/Cross_Dn bytes with validation.")
    st.add_argument("-r", "--run", action="store_true", help="restart the ESC afterward (else held)")
    ap_ = sub.add_parser("apply", help="apply a YAML profile (index or 'all')")
    ap_.add_argument("index", help="ESC index or 'all'")
    ap_.add_argument("profile", help="YAML file (a read -o export or a hand-written profile)")
    ap_.add_argument("--name", help="also set the ESC name")
    ap_.add_argument("--with-name", action="store_true", help="also apply identity.name from the profile")
    ap_.add_argument("-r", "--run", action="store_true", help="restart the ESC afterward (else held)")
    cn = sub.add_parser("connect", help="enter the bootloader and hold the session (index or 'all')")
    cn.add_argument("index", help="ESC index or 'all'")
    rn = sub.add_parser("run", aliases=["disconnect"], help="restart held ESC(s) (end the session)")
    rn.add_argument("index", type=int, nargs="?", default=0)
    fl = sub.add_parser("flash", help="flash BLHeli-S firmware (Intel-HEX) to an ESC")
    fl.add_argument("index", type=int)
    fl.add_argument("hexfile", help="BLHeli-S .HEX matching the ESC's layout + MCU")
    fl.add_argument("--yes", action="store_true", help="confirm the erase+write (required)")
    fl.add_argument("--force", action="store_true", help="flash even if the compat check fails (danger)")
    args = ap.parse_args()

    dev = EscHost(args.port)
    try:
        {"list": cmd_list, "read": cmd_read, "set": cmd_set, "apply": cmd_apply,
         "connect": cmd_connect, "run": cmd_run, "disconnect": cmd_run,
         "flash": cmd_flash}[args.cmd](dev, args)
    finally:
        dev.close()


if __name__ == "__main__":
    main()
