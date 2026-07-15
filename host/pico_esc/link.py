# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""pico_esc.link — line-based USB-CDC transport to the RP2040 esc_tool firmware + clocks.

EscLink (== EscHost) is the send-a-command / collect-reply-lines-until 'ok'/'err' transport,
auto-detecting the Pico by VID 2E8A. RealClock/SimClock are the wall-clock vs deterministic
virtual-time abstraction used by the control loop. Moved verbatim from esctool.py / posctl.py;
the sys.exit / RuntimeError / TimeoutError messages are byte-identical.
"""
from __future__ import annotations

import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial required:  pip install pyserial")

RPI_VID = 0x2E8A


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


# EscLink is the library-facing name; EscHost is kept as the historical alias so the CLIs
# (and `from esctool import EscHost`) keep importing the same class.
EscLink = EscHost


# ---------------------------------------------------------------------------
# Clock abstraction: real wall-clock for hardware, deterministic virtual time
# for --dry-run (so dt and integrated position are exactly reproducible).
# ---------------------------------------------------------------------------
class RealClock:
    def now(self):
        return time.time()

    def sleep(self, dt):
        if dt > 0:
            time.sleep(dt)


class SimClock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def sleep(self, dt):
        if dt > 0:
            self.t += dt
