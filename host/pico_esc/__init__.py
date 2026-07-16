# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""pico_esc — host-side library for the RP2040 esc_tool firmware.

Two-sided librarization of the bench tooling: this package holds the reusable transport,
protocol, config codec, flash helpers, simulated hosts, drive session, and cascade position
controller; the CLI scripts (esctool.py / posctl.py / autocal.py / …) are thin wrappers that
import from here and keep their exact argv, stdout, and the frozen Pico<->host wire protocol.

  from pico_esc import EscLink, ESC
  from pico_esc.control import PositionController
"""
from __future__ import annotations

from .link import EscLink, EscHost, find_pico, RealClock, SimClock
from .esc import ESC, EscConfig
from .types import EncReading, Tele, Telem
from .velocity import SpeedProfile, VelocityController, measure_steady_speed
from . import config, protocol, flash

__all__ = [
    "EscLink", "EscHost", "find_pico", "RealClock", "SimClock",
    "ESC", "EscConfig", "EncReading", "Tele", "Telem",
    "SpeedProfile", "VelocityController", "measure_steady_speed",
    "config", "protocol", "flash",
]
