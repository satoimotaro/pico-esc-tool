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
    "low_rpm_power_protection": 0x24, "brake_on_stop": 0x27,
}
FIELD_ENUM = {"motor_direction": DIRECTION, "comm_timing": TIMING, "demag_compensation": DEMAG}
NAME_OFF, NAME_LEN = 0x60, 16


def encode_value(field: str, value) -> int:
    """Field value (int, numeric string, or enum name) -> byte."""
    enum = FIELD_ENUM.get(field)
    if enum and isinstance(value, str) and not value.lstrip("-").isdigit():
        rev = {v.lower(): k for k, v in enum.items()}
        if value.lower() not in rev:
            raise ValueError(f"{field}: '{value}' not in {list(enum.values())}")
        return rev[value.lower()]
    return int(value) & 0xFF


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
    dev.cmd(f"editpage {index} {overrides_str(ovs)}", timeout=30)
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


def cmd_run(dev: EscHost, args):
    dev.cmd("disconnect")
    print("released bootloader session; ESC(s) restarted.")


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
    ap = argparse.ArgumentParser(description="BLHeli-S ESC CLI (esc_host firmware)")
    ap.add_argument("--port", help="serial port (default: auto-detect VID 2E8A)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list", help="scan and list connected ESCs")
    rd = sub.add_parser("read", help="read ESC config (index or 'all')")
    rd.add_argument("index", help="ESC index from 'list', or 'all'")
    rd.add_argument("-o", "--out", help="write YAML to this file (single ESC only)")
    rd.add_argument("-r", "--run", action="store_true", help="restart the ESC afterward (else held)")
    st = sub.add_parser("set", help="change settings on an ESC (index or 'all')")
    st.add_argument("index", help="ESC index or 'all'")
    st.add_argument("assign", nargs="+", help="e.g. motor_direction=Reversed beep_strength=60")
    st.add_argument("-r", "--run", action="store_true", help="restart the ESC afterward (else held)")
    ap_ = sub.add_parser("apply", help="apply a YAML profile (index or 'all')")
    ap_.add_argument("index", help="ESC index or 'all'")
    ap_.add_argument("profile", help="YAML file (a read -o export or a hand-written profile)")
    ap_.add_argument("--name", help="also set the ESC name")
    ap_.add_argument("--with-name", action="store_true", help="also apply identity.name from the profile")
    ap_.add_argument("-r", "--run", action="store_true", help="restart the ESC afterward (else held)")
    rn = sub.add_parser("run", aliases=["disconnect"], help="restart held ESC(s) (end the session)")
    rn.add_argument("index", type=int, nargs="?", default=0)
    args = ap.parse_args()

    dev = EscHost(args.port)
    try:
        {"list": cmd_list, "read": cmd_read, "set": cmd_set, "apply": cmd_apply,
         "run": cmd_run, "disconnect": cmd_run}[args.cmd](dev, args)
    finally:
        dev.close()


if __name__ == "__main__":
    main()
