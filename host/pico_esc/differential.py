# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""pico_esc.differential — the two-motor differential azimuth pod: mixer + dual drive session.

Two BLDCs drive one bevel differential. Neither motor moves; the differential IS the azimuth
actuator (no servo). With A and B the two signed motor commands:

    azimuth channel   s = (A + B) / 2        both motors the SAME direction -> the pod steers
    propeller channel d = (A - B) / 2        counter-rotating               -> pure thrust

    inverse           A = s + d
                      B = s - d

so counter-rotating at equal speed gives s = 0 (angle held) and |d| = |A| (prop turning), and
same-direction gives d = 0 (no thrust) and s = A (pure steering). That is the whole point of the
mechanism: one pair of motors, two independent degrees of freedom.

WHAT IS NOT KNOWN, AND MUST NOT BE ASSUMED
------------------------------------------
`sign_a` / `sign_b` map "positive `thrust` on this ESC" onto "positive rotation in the
differential's own frame". In a bevel differential the two motors are normally mounted facing
each other, so a positive command on B may well produce the opposite-handed rotation to the same
command on A — in which case the naive mix silently swaps the two channels: commanding pure thrust
steers, and commanding a turn drives the prop. `identify()` measures these from the azimuth
encoder instead of guessing. Until it has run, treat s/d as unverified.

`SATURATION PRIORITISES s.` When A or B would exceed the thrust ceiling, the propeller command is
shed and the azimuth command is kept, because losing angle hold is the failure that points the
thruster somewhere unintended, while losing thrust merely slows the vehicle. Clamping A and B
independently instead would distort the mix — it changes BOTH s and d, so a saturating turn would
also inject unintended thrust.
"""
from __future__ import annotations


class MixResult:
    """One mixer evaluation: the two motor commands plus what saturation cost."""

    __slots__ = ("a", "b", "s", "d", "d_requested", "saturated")

    def __init__(self, a, b, s, d, d_requested, saturated):
        self.a, self.b = a, b
        self.s, self.d = s, d                    # the (s, d) actually delivered
        self.d_requested = d_requested           # what was asked for before shedding
        self.saturated = saturated

    @property
    def d_shed(self):
        """How much propeller command was given up to protect the azimuth channel."""
        return abs(self.d_requested) - abs(self.d)

    def __repr__(self):
        return (f"MixResult(a={self.a:.1f}, b={self.b:.1f}, s={self.s:.1f}, d={self.d:.1f}"
                f"{', SAT' if self.saturated else ''})")


class DifferentialMixer:
    """(s, d) <-> (A, B) for one differential azimuth pod.

    sign_a / sign_b are the per-channel command polarities in the differential's frame; both
    default to +1, which is a PLACEHOLDER, not a measurement (see the module docstring).
    """

    def __init__(self, tmax=1000, sign_a=1, sign_b=1):
        if sign_a not in (1, -1) or sign_b not in (1, -1):
            raise ValueError("sign_a/sign_b must be +1 or -1")
        if tmax <= 0:
            raise ValueError("tmax must be positive")
        self.tmax = tmax
        self.sign_a = sign_a
        self.sign_b = sign_b

    # -- forward ------------------------------------------------------------------
    def mix(self, s, d):
        """(azimuth cmd, propeller cmd) -> MixResult with the two per-ESC thrust values.

        s is preserved and d is shed on saturation; if |s| alone already exceeds the ceiling,
        s is clamped too and d goes to zero.
        """
        s_req, d_req = s, d
        s = max(-self.tmax, min(self.tmax, s))       # s alone cannot exceed the ceiling
        headroom = self.tmax - abs(s)                # what is left for the propeller channel
        if abs(d) > headroom:
            d = headroom if d > 0 else -headroom
        a = self.sign_a * (s + d)
        b = self.sign_b * (s - d)
        return MixResult(a=a, b=b, s=s, d=d, d_requested=d_req,
                         saturated=(s != s_req or d != d_req))

    # -- inverse ------------------------------------------------------------------
    def unmix(self, a, b):
        """(A, B) as sent -> (s, d). The inverse of mix() ignoring saturation; used by
        identify() to turn measured per-motor speeds back into channel terms."""
        a = self.sign_a * a
        b = self.sign_b * b
        return (a + b) / 2.0, (a - b) / 2.0

    # -- identification -----------------------------------------------------------
    @staticmethod
    def identify(rate_from_a, rate_from_b, tol=0.25):
        """Resolve the sign convention from two single-motor runs.

        **CAVEAT, MEASURED ON THIS BENCH 2026-08-11: a lone motor may not move the carrier at all.**
        A disarmed ESC leaves its motor's windings open, so that motor free-wheels — and a
        differential sends torque down whichever path resists least, which is the free input rather
        than the loaded output. Same as a car differential with one wheel on ice. Driving ch0 alone
        for 9 s at a command well past handoff produced ZERO azimuth motion here. If a single-motor
        run does not move the axis, brake the idle channel (`brake_on_stop`, or hold it armed at a
        commanded zero instead of disarmed) so the differential has something to react against, and
        only then read the rate.

        Drive ONE motor at a time at the same low command and measure the resulting azimuth rate
        on the encoder (deg/s, signed): `rate_from_a` with only ESC A driven, `rate_from_b` with
        only ESC B. In the differential's frame a lone motor turns the carrier either way, so:

            same sign  -> "both motors the same direction steers" holds: the mix above is right
            opposite   -> B is mounted mirrored: sign_b = -1, and WITHOUT that correction the
                          steering and thrust channels are swapped

        Returns (sign_a, sign_b, azimuth_gain) where azimuth_gain is deg/s per unit s — the
        number PositionController's Kff needs. Raises if either run barely moved, because a
        near-zero rate cannot carry a sign and averaging it in would fabricate a convention.
        """
        if abs(rate_from_a) < tol or abs(rate_from_b) < tol:
            raise ValueError(
                f"azimuth barely moved (a={rate_from_a:.3f}, b={rate_from_b:.3f} deg/s; tol={tol}): "
                "cannot infer a sign from noise — raise the identification command or check that "
                "both channels actually spin")
        sign_a = 1 if rate_from_a > 0 else -1
        # Express B relative to A so that +s (both positive) is a positive azimuth rate.
        sign_b = sign_a if (rate_from_a > 0) == (rate_from_b > 0) else -sign_a
        # With both signs folded in, a lone motor at command u gives s = u/2, so the per-unit-s
        # gain is the mean of the two |rate| / (u/2) — returned normalised per unit command here
        # and scaled by the caller, which knows u.
        gain = (abs(rate_from_a) + abs(rate_from_b)) / 2.0
        return sign_a, sign_b, gain


# ---------------------------------------------------------------------------------
# Dual-ESC drive session.
# ---------------------------------------------------------------------------------
class DualDrive:
    """Two ESC indices driven as one pod, through a DifferentialMixer.

    Wraps two PosDrive sessions so arm/disarm/keep-alive/abort apply to BOTH channels — a pod
    with one channel armed and one not is the state that steers when told to hold, so there is
    deliberately no way to arm just one from here (use PosDrive directly for identification).
    """

    def __init__(self, host, mixer, idx_a=0, idx_b=1, tmax=1000, clock=None, verbose=True):
        from .drive import PosDrive
        self.host = host
        self.mixer = mixer
        self.a = PosDrive(host, idx_a, tmax, clock, verbose=False)
        self.b = PosDrive(host, idx_b, tmax, clock, verbose=False)
        self.clock = clock
        self.verbose = verbose
        self.last_mix = None

    def _log(self, msg):
        if self.verbose:
            print(msg)

    def prepare(self):
        self.a.prepare()

    def arm(self):
        self._log("# arming BOTH channels (bidir)…")
        self.a.arm()
        self.b.arm()

    def disarm(self):
        """Both channels, always — and B even if A raised on the way out."""
        try:
            self.a.disarm()
        finally:
            self.b.disarm()
        self._log("# DISARMED (both)")

    def send(self, s, d=0.0):
        """Drive the pod: azimuth command s, propeller command d. Returns the MixResult."""
        m = self.mixer.mix(s, d)
        self.a.send_thrust(m.a)
        self.b.send_thrust(m.b)
        self.last_mix = m
        return m

    def read_enc(self):
        """Azimuth angle — ONE encoder on the azimuth axis, so it is read once, not per channel."""
        return self.a.read_enc()
