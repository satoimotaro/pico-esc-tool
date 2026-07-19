# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""pico_esc.flash — Intel-HEX parsing + page assembly for BLHeli-S app flashing.

Splits a HEX into the app image (<0x1A00), the identity/eeprom section (0x1A00..0x1BFF, the
firmware default config), and a bootloader-byte count (skipped, the BL is preserved), then
assembles the 512-byte flash pages. Moved verbatim from esctool.py; esctool.cmd_flash drives
these over an EscLink. The wire writes (erase/writeflash/readflash) are unchanged.
"""
from __future__ import annotations

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
