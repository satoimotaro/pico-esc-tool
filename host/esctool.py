#!/usr/bin/env python3
"""esctool — BLHeli-Configurator-like CLI for the RP2040 ESC tool (firmware: esc_host).

Talks to the Pico over USB-CDC serial (auto-detected by VID 2E8A) with a small text protocol.
Phase 1 commands: list, read. (set/apply/flash/profiles come next.)

  python esctool.py list
  python esctool.py read 0 -o config.yaml
"""
from __future__ import annotations

import argparse
import sys
import time

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


def find_pico(port: str | None) -> str:
    if port:
        return port
    # retry briefly: right after an upload the CDC port takes a moment to re-enumerate
    for _ in range(40):
        for p in list_ports.comports():
            if p.vid == RPI_VID:
                return p.device
        time.sleep(0.1)
    sys.exit("no RP2040 (VID 2E8A) found - is esc_host flashed and the monitor closed?")


class EscHost:
    """Line-based transport: send a command, collect reply lines until 'ok'/'err'."""

    def __init__(self, port: str | None = None):
        self.ser = serial.Serial(find_pico(port), 115200, timeout=10)
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
            "low_rpm_power_protection": g(0x24),
            "brake_on_stop": g(0x27),
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
    lines = dev.cmd(f"read {args.index}")
    cfg = next((l for l in lines if l.startswith("cfg|")), None)
    if not cfg:
        sys.exit("no config returned")
    raw = bytes.fromhex(cfg.split("|", 1)[1])
    doc = {"esc": args.index, **decode(raw)}
    text = _emit_yaml(doc)
    if args.out:
        with open(args.out, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(text)
        print(f"wrote {args.out} ({len(raw)} config bytes)")
    else:
        sys.stdout.write(text)


def main():
    ap = argparse.ArgumentParser(description="BLHeli-S ESC CLI (esc_host firmware)")
    ap.add_argument("--port", help="serial port (default: auto-detect VID 2E8A)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list", help="scan and list connected ESCs")
    rd = sub.add_parser("read", help="read one ESC's config")
    rd.add_argument("index", type=int, help="ESC index (from 'list')")
    rd.add_argument("-o", "--out", help="write YAML to this file (else stdout)")
    args = ap.parse_args()

    dev = EscHost(args.port)
    try:
        {"list": cmd_list, "read": cmd_read}[args.cmd](dev, args)
    finally:
        dev.close()


if __name__ == "__main__":
    main()
