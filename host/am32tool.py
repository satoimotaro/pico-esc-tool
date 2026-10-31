#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""am32tool — read and change AM32 ESC settings through the Pico, no flight controller.

  python am32tool.py read 0                     # identity + every setting, named
  python am32tool.py read 0 -o aria.bin         # also save the raw 192-byte block
  python am32tool.py set 0 bi_direction=1 input_type=1
  python am32tool.py defaults 0                 # write pico_esc.am32.DEFAULTS (keeps kv/poles)
  python am32tool.py restore 0 aria.bin

Every write saves the block it replaces to host/am32_backups/ first and is read back. The ESC is
left running its app afterwards (`run`), so it beeps its startup tune when this exits.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pico_esc.am32 import Am32, DEFAULTS, FIELDS, decode, encode, kv  # noqa: E402
from pico_esc.link import EscHost  # noqa: E402

BACKUPS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "am32_backups")


def show(esc: Am32, raw: bytes):
    print(f"firmware  {esc.read_name()}  v{raw[3]}.{raw[4]}  eeprom v{raw[1]}  bootloader v{raw[2]}")
    print(f"motor     {kv(raw)} kv, {raw[27]} poles")
    vals = decode(raw)
    for off, name, note in FIELDS:
        if off < 5:
            continue
        v = vals[name]
        flag = "  <- unset (0xFF)" if v == 0xFF else ""
        print(f"  {off:3d} {name:36s} {v:4d}   {note}{flag}")


def backup(raw: bytes, tag: str) -> str:
    os.makedirs(BACKUPS, exist_ok=True)
    path = os.path.join(BACKUPS, time.strftime("%Y%m%d_%H%M%S") + f"_{tag}.bin")
    with open(path, "wb") as f:
        f.write(raw)
    return path


def parse_kv(items: list[str]) -> dict[str, int]:
    out = {}
    for it in items:
        k, _, v = it.partition("=")
        out[k] = int(v, 0)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port")
    sub = ap.add_subparsers(dest="cmd", required=True)
    rd = sub.add_parser("read")
    rd.add_argument("idx", type=int)
    rd.add_argument("-o", "--out", help="save the raw block here")
    st = sub.add_parser("set")
    st.add_argument("idx", type=int)
    st.add_argument("kv", nargs="+", help="name=value")
    df = sub.add_parser("defaults")
    df.add_argument("idx", type=int)
    rs = sub.add_parser("restore")
    rs.add_argument("idx", type=int)
    rs.add_argument("file")
    a = ap.parse_args()

    esc = Am32(EscHost(a.port), a.idx)
    try:
        print(esc.enter())
        raw = esc.read_eeprom()
        if a.cmd == "read":
            show(esc, raw)
            if a.out:
                with open(a.out, "wb") as f:
                    f.write(raw)
                print("saved", a.out)
            return 0
        if a.cmd == "restore":
            with open(a.file, "rb") as f:
                new = f.read()
        else:
            changes = dict(DEFAULTS) if a.cmd == "defaults" else parse_kv(a.kv)
            new = encode(raw, changes)
        diff = [(name, raw[off], new[off]) for off, name, _ in FIELDS if raw[off] != new[off]]
        if not diff:
            print("no change")
            return 0
        print("backup", backup(raw, f"esc{a.idx}"))
        for name, old, nv in diff:
            print(f"  {name:36s} {old:4d} -> {nv}")
        show(esc, esc.write_eeprom(new))
        return 0
    finally:
        esc.run()


if __name__ == "__main__":
    sys.exit(main())
