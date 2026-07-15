#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""esctool — BLHeli-Configurator-like CLI for the RP2040 ESC tool (firmware: esc_tool).

Talks to the Pico over USB-CDC serial (auto-detected by VID 2E8A) with a small text protocol.
Commands: list, connect, read, set, apply <profile.yaml>, flash <hex>, run/disconnect.

  python esctool.py list
  python esctool.py read 0 -o config.yaml
  python esctool.py apply all host/profiles/blheli-s-default.yaml

This is now a thin CLI wrapper: the transport, config codec, and flash helpers live in the
pico_esc package. The names below are re-exported so `from esctool import EscHost`,
`esctool.encode_overrides`, `esctool._apply_overrides`, etc. keep working for autocal.py /
tune_sine_amp.py, and the protocol / printed strings are byte-identical.
"""
from __future__ import annotations

import argparse
import os
import sys

try:  # Windows consoles default to cp932 and crash on any non-ASCII output
    sys.stdout.reconfigure(errors="replace")
except Exception:
    pass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Re-exported for backward compatibility (autocal.py / tune_sine_amp.py import these from here).
from pico_esc.config import (  # noqa: E402,F401
    DIRECTION, TIMING, DEMAG, MODE, FIELD_OFF, FIELD_ENUM, NAME_OFF, NAME_LEN,
    MAX_ERPM_UNITS, SINE_CROSS_UP_ERPM_PER_UNIT, SINE_CROSS_DN_ERPM_NUM,
    SINE_CROSS_DN_MAX_BYTE, SINE_CROSS_TICKS_MIN, SINE_CROSS_TICKS_MAX,
    SINE_CROSS_UP_ERPM_MIN, SINE_CROSS_UP_ERPM_MAX, sine_crossover_bytes,
    encode_value, encode_overrides, overrides_str, _tag, decode, _emit_yaml, load_yaml, yaml,
)
from pico_esc.link import RPI_VID, find_pico, EscHost  # noqa: E402,F401
from pico_esc.flash import (  # noqa: E402,F401
    APP_END, EEPROM_BASE, BOOT_BASE, SIG_FOR_MCU, parse_hex, hex_tag, _pages_from,
)


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
