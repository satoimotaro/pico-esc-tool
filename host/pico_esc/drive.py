# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""pico_esc.drive — signed-thrust keep-alive drive session for the position servo.

PosDrive owns one host (EscLink or SimEncEscHost) + one ESC index and funnels every thrust
send through the single send_thrust choke point (magnitude + tmax clamp), mirroring
drive_hold.py's keep-alive / triple-thrust-0-then-disarm safety. Moved verbatim from posctl.py.
"""
from __future__ import annotations

from .types import EncReading, Telem

ARM_WAIT = 4.0             # seconds to wait after arming before driving


class Aborted(RuntimeError):
    pass


class PosDrive:
    """One host + one ESC index.  send_thrust is the single thrust choke point."""

    def __init__(self, host, idx, tmax, clock, verbose=True):
        self.host = host
        self.idx = idx
        self.tmax = tmax
        self.clock = clock
        self.verbose = verbose
        self.armed = False
        self.last_temp = None      # last ESC temperature (C) read from `tele`, if any
        self.peak_temp = None      # peak ESC temperature (C) seen this run

    def _log(self, msg):
        if self.verbose:
            print(msg)

    def prepare(self):
        # release any held bootloader session so the ESC app is running before arm
        for c in ("run", "disconnect"):
            try:
                self.host.cmd(c, timeout=5)
            except Exception:
                pass
        self.clock.sleep(0.4)

    def arm(self):
        self._log("# arming (bidir)…")
        self.host.cmd(f"arm {self.idx} bidir", timeout=6)
        self.armed = True
        self.clock.sleep(ARM_WAIT)

    def disarm(self):
        if not self.armed:
            return
        for _ in range(3):                       # triple thrust 0 (mirror drive_hold)
            try:
                self.send_thrust(0)
            except Exception:
                pass
        for c in (f"disarm {self.idx}", "disarm"):
            try:
                self.host.cmd(c, timeout=2)
            except Exception:
                pass
        self.armed = False
        self._log("# DISARMED")

    # -- the ONLY place a thrust value is sent to the ESC --
    def send_thrust(self, u):
        u = int(u)
        u = max(-1000, min(1000, u))
        u = max(-self.tmax, min(self.tmax, u))   # single thrust-ceiling choke point
        try:
            self.host.cmd(f"thrust {self.idx} {u}", timeout=2)
        except Aborted:
            raise
        except Exception as e:                   # never leak a raw traceback mid-drive
            raise Aborted(f"thrust command failed ({e}) — disarming")
        return u

    def read_enc(self):
        """Return EncReading or None on any read/parse failure."""
        try:
            lines = self.host.cmd("enc", timeout=2)
        except Exception:
            return None
        for ln in lines:
            if ln.startswith("enc|"):
                p = ln.split("|")
                try:
                    return EncReading(int(p[1]), int(p[4]), int(p[5]),
                                      int(p[6]), int(p[7]), int(p[8]))
                except (ValueError, IndexError):
                    return None
        return None

    def read_tele(self):
        """One full bidir-DShot telemetry sample as a Telem, or None if absent/unparseable.

        Parses the EXISTING `tele|rpm|volts|amps|tempC|stress` response (bidir DShot only) —
        no new command, no protocol change. Provided for closed-loop consumers that need eRPM
        (velocity control); it is NOT called from run_segments, so the dry-run RNG order is
        untouched. A miss just returns None (callers skip that cycle, never abort).
        """
        try:
            lines = self.host.cmd(f"tele {self.idx}", timeout=2)
        except Exception:
            return None
        for ln in lines:
            if ln.startswith("tele|"):
                p = ln.split("|")
                try:
                    return Telem(int(p[1]), float(p[2]), int(p[3]), int(p[4]), int(p[5]))
                except (ValueError, IndexError):
                    return None
        return None

    def read_temp(self):
        """ESC temperature in C from `tele`, or None if telemetry is absent/unparseable.

        Format: tele|rpm|volts|amps|tempC|stress (bidir DShot only). A miss just skips the
        thermal check this cycle — it never aborts on a missing sample. Delegates to read_tele
        (same single `tele` command, identical call order — the dry-run oracle is preserved).
        """
        t = self.read_tele()
        return t.temp if t is not None else None

    def send_throttle(self, thr):
        """Unidirectional throttle send (0..tmax ceiling). Additive helper for the ESC drive
        facade; NOT used by the position CLIs (which drive signed thrust via send_thrust)."""
        thr = int(thr)
        thr = max(0, min(self.tmax, thr))
        try:
            self.host.cmd(f"throttle {self.idx} {thr}", timeout=2)
        except Aborted:
            raise
        except Exception as e:
            raise Aborted(f"throttle command failed ({e}) — disarming")
        return thr
