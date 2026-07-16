# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""pico_esc.types — small value objects shared across the host tools.

EncReading is one AS5600 `enc|…` sample (with a magnet-health property); Tele is one bidir
DShot `tele|…` sample. Moved verbatim from posctl.py / autocal.py.
"""
from __future__ import annotations


class EncReading:
    __slots__ = ("raw", "md", "ml", "mh", "agc", "mag")

    def __init__(self, raw, md, ml, mh, agc, mag):
        self.raw, self.md, self.ml, self.mh, self.agc, self.mag = raw, md, ml, mh, agc, mag

    @property
    def healthy(self):
        return self.md == 1 and self.ml == 0 and self.mh == 0


class EncVel:
    """One `encv|accum|rpm|samples|md` sample — the DE-ALIASED encoder velocity computed
    on-device (RP2040) at ~1.25 kHz. `rpm` is signed mech RPM (raw-angle-increasing = +), valid
    at any speed (unlike the host-side 50 Hz unwrap of `enc`, which aliases above ~1350 mech)."""
    __slots__ = ("accum", "rpm", "samples", "md")

    def __init__(self, accum, rpm, samples, md):
        self.accum, self.rpm, self.samples, self.md = accum, rpm, samples, md

    @property
    def healthy(self):
        return self.md == 1


class Tele:
    __slots__ = ("rpm", "volts", "amps", "temp", "stress")

    def __init__(self, rpm, volts, amps, temp, stress):
        self.rpm, self.volts, self.amps, self.temp, self.stress = rpm, volts, amps, temp, stress


# Telem is the library-facing alias for Tele.
Telem = Tele
