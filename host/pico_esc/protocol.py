# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""pico_esc.protocol — the FROZEN esc_tool wire protocol, as data.

This is the single host-side registry of every command name and response-format literal that
the RP2040 firmware (src/apps/esc_tool.cpp handleSerial) speaks. It is deliberately data-only:
the CLIs keep building their command strings inline (byte-for-byte unchanged), and
host/tests/test_protocol.py greps the firmware source to prove every literal below still
exists on the device — so the two sides can never silently drift apart.

DO NOT edit a literal here without making the identical change on BOTH sides (firmware +
whichever host parser consumes it). Substrings are chosen to appear verbatim in esc_tool.cpp.
"""
from __future__ import annotations

# Command tokens the firmware matches with strcmp(cmd, "..."). Grepped WITH the quotes so a
# command can't be renamed on one side only.
COMMANDS = (
    "ping", "pins", "fwlist", "mode", "scan", "read", "enter", "run", "disconnect",
    "editpage", "erase", "writeflash", "readflash", "arm", "throttle", "spin", "thrust",
    "disarm", "spinstop", "pwm", "enc", "tele",
)

# Response-format literals (Serial.print/printf). Each MUST appear verbatim in the firmware.
RESPONSES = (
    "esc|%u|%u|0\\n",
    "esc|%u|%u|1|%04X|%u|%s|%s|%u.%u\\n",
    "cfg|",
    "dev|%04X|%u|%u\\n",
    "data|",
    "fw| %s  %u bytes\\n",
    "fwlist %d\\n",
    "pwm|%d|%dus\\n",
    "enc|%d|%d|%.1f|%d|%d|%d|%d|%d\\n",
    "tele|%lu|%.2f|%lu|%lu|%lu\\n",
    "arming ~3s (mode %s, %s)\\n",
    "unchanged (flash write skipped)",
    "edited %d byte(s)\\n",
    "id esc_tool v1",
    "mode %s (wifi %s)\\n",
    "ok",
)

# Error responses. `cmd()` treats any line starting with "err" as a device error.
ERRORS = (
    "err bad-index",
    "err no-connect",
    "err bad-args",
    "err bad-override",
    "err read-failed",
    "err write-verify-failed",
    "err erase-failed",
    "err bad-hex",
    "err write-failed",
    "err not-armed",
    "err dshot-init-failed (no free PIO SM?)",
    "err no-encoder",
    "err no-telem",
    "err unknown-cmd",
)

# Response-line tags host parsers key off (line.startswith(TAG)).
TAG_ESC = "esc|"
TAG_CFG = "cfg|"
TAG_DEV = "dev|"
TAG_DATA = "data|"
TAG_ENC = "enc|"
TAG_TELE = "tele|"
