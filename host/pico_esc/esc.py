# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""pico_esc.esc — host-side ESC handle: a thin, per-instance facade over an EscLink.

ESC(link, index) is the library embedding surface: `esc.config` for read/write, plus the
drive verbs (arm / thrust / throttle / encoder / telemetry / disarm) layered over a private
PosDrive (keep-alive + triple-thrust-0-then-disarm safety). A higher-level controller (e.g. a
VelocityController) is written against this object.

It reuses the exact same wire commands, config encoders, and drive session the CLIs use — no
new protocol, no global state. ESC is ADDITIVE: the position CLIs keep driving PosDrive /
run_segments directly, so ESC never changes that code path (the dry-run oracle is untouched).
"""
from __future__ import annotations

from .config import decode, encode_overrides, overrides_str
from .drive import ARM_WAIT, PosDrive
from .link import RealClock


class EscConfig:
    """Config sub-facade for one ESC: read the decoded block, write overrides, restart."""

    def __init__(self, link, index: int):
        self.link = link
        self.index = index

    def read_raw(self) -> bytes | None:
        """The raw 255-byte config block, or None if the ESC did not answer `read`."""
        for ln in self.link.cmd(f"read {self.index}"):
            if ln.startswith("cfg|"):
                return bytes.fromhex(ln.split("|", 1)[1])
        return None

    def read(self) -> dict | None:
        """The decoded config (identity + settings + raw_hex), or None if unavailable."""
        raw = self.read_raw()
        return decode(raw) if raw is not None else None

    def write(self, settings: dict) -> int:
        """Apply a settings dict via editpage (flash-wear guarded on the device).

        Returns the number of override bytes written, or 0 if nothing writable / the device
        reported the config already matched ("unchanged (flash write skipped)").
        """
        ovs = encode_overrides(settings)
        if not ovs:
            return 0
        lines = self.link.cmd(f"editpage {self.index} {overrides_str(ovs)}", timeout=30)
        return 0 if any("unchanged" in l for l in lines) else len(ovs)

    def set(self, settings: dict | None = None, **kw) -> int:
        """Write by dict and/or kwargs — e.g. esc.config.set(sine_hold_amp=16, sine_amp_max=45).
        A thin alias for write() matching the goal-API vocabulary."""
        merged = dict(settings or {})
        merged.update(kw)
        return self.write(merged)

    def restart(self):
        """Leave the bootloader session and restart the ESC app."""
        self.link.cmd(f"run {self.index}")


class ESC:
    """One ESC on one EscLink. `esc.config` handles config; the drive verbs
    (arm / thrust / throttle / encoder / telemetry / temperature / disarm) run over a private
    PosDrive. `esc.enter()` holds the bootloader session for config/flash.
    """

    def __init__(self, link, index: int = 1, *, tmax: int = 1000, clock=None):
        self.link = link
        self.index = index
        self.config = EscConfig(link, index)
        # Private drive session (verbose off — the library surface prints nothing). tmax is the
        # thrust/throttle magnitude ceiling; clock is virtual for a SimEncEscHost dry-run.
        self._drive = PosDrive(link, index, tmax, clock or RealClock(), verbose=False)

    # ---- bootloader / config lifecycle ----
    def enter(self) -> str | None:
        """Enter the bootloader (held session). Returns the `dev|sig|boot|pages` line or None."""
        return next((l for l in self.link.cmd(f"enter {self.index}") if l.startswith("dev|")), None)

    def restart(self):
        self.config.restart()

    # ---- drive session ----
    @property
    def drive(self) -> PosDrive:
        """The underlying PosDrive (keep-alive + triple-thrust-0-then-disarm safety)."""
        return self._drive

    def prepare(self):
        """Release any held bootloader session so the ESC app is running before arm."""
        self._drive.prepare()

    def arm(self, bidir: bool = True):
        """Arm for driving. bidir=True (default) gives signed thrust + eRPM/EDT telemetry;
        bidir=False arms a plain one-way (normal DShot) drive. Returns self for chaining."""
        if bidir:
            self._drive.arm()
        else:
            self._drive._log("# arming (normal)…")
            self.link.cmd(f"arm {self.index} normal", timeout=6)
            self._drive.armed = True
            self._drive.clock.sleep(ARM_WAIT)
        return self

    def disarm(self):
        """Triple thrust-0 then disarm (always safe to call)."""
        self._drive.disarm()

    def thrust(self, x):
        """Signed thrust -1000..+1000 (reversible/3D); returns the clamped value sent."""
        return self._drive.send_thrust(x)

    def throttle(self, x):
        """Unidirectional throttle 0..tmax; returns the clamped value sent."""
        return self._drive.send_throttle(x)

    def encoder(self):
        """One AS5600 EncReading (raw + magnet-health), or None on a read/parse failure."""
        return self._drive.read_enc()

    def telemetry(self):
        """One full Telem sample (rpm/volts/amps/temp/stress), or None if absent."""
        return self._drive.read_tele()

    def temperature(self):
        """ESC temperature in C, or None if telemetry is absent."""
        return self._drive.read_temp()
