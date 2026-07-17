# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""pico_esc.velocity — sensorless feed-forward velocity control (velctl v1).

A VelocityController drives a target mechanical RPM SENSORLESSLY by inverting ONE per-motor
calibrated curve of ESC command -> measured speed. The curve is a SpeedProfile: a list of
(thrust, rpm) points measured once by velcal (encoder-in-the-loop). At runtime the controller
gently slews the setpoint and looks the command up from the curve — no runtime feedback.

TERMINOLOGY (the user was confused — say it plainly): "thrust" here is the SIGNED ESC COMMAND
(-1000..+1000, bidirectional DShot), the controllable INPUT we SET — it is NOT physical force
or thrust. The calibrated curve maps command("thrust") -> measured mechanical RPM (via the
AS5600 encoder). Runtime inverts it: target RPM -> command. There is NO force sensor.

ONE curve subsumes the S3 sine<->BEMF crossover: because it is measured across the whole
range (crossover enabled during velcal), the seam / handoff jump is baked into the points, so
pure feed-forward needs no regime knowledge — the firmware handles the transition itself.

SCOPE (Phase A1): feed-forward + a telemetry-fed PI trim whose AUTHORITY FADES with telemetry
liveness. Each tick the command is `thrust_for(setpoint) + w*trim`: the feed-forward carries the
motion, and a PI on (setpoint - tele_mech) trims it. `tele.rpm` is ALREADY mechanical RPM (the
firmware pre-divides eRPM by pole pairs) — it is used DIRECTLY, never divided by POLE_PAIRS again.
The PI weight `w` ramps 0->1 as a live 6-step `tele` frame (|rpm|>TELE_MIN_MECH_RPM) is seen and
1->0 while it goes stale (forced sine below the seam has no BEMF telemetry), so below the seam the
loop degrades to pure feed-forward; the integrator resets when w reaches 0. Authority is keyed
PURELY on tele-liveness (never on the profile's crossover / capabilities), so a stock-Bluejay-like
profile with no crossover still runs. The AS5600 encoder is used ONLY for calibration (velcal) and,
at runtime with --encoder, for an independent VERIFY-LOG that NEVER feeds the command. This Python
loop is the A1 reference / bench tool; the runtime home is the Pico C++ port (A2). Sensorless FF
is the default; the PI trim engages automatically only where telemetry is live.

All thrust is routed through ESC.thrust() (-> PosDrive.send_thrust, the single clamp/choke);
every loop branch paces at DT (well under the 500 ms firmware deadman); temperature is polled
and the run aborts over --max-temp; the CLI always disarms on every exit.
"""
from __future__ import annotations

import math
import statistics

from . import config
from .constants import (COUNTS_PER_REV, FULLSCALE_RPM, POLE_PAIRS,  # noqa: F401 (re-exported)
                       TELE_MIN_MECH_RPM)
from .control import (DELTA_FAULT_FRAC, DT, ENC_FAIL_MAX, TELE_EVERY_S,
                      VEL_LP_ALPHA, VelReader, _pace)
from .drive import Aborted

# POLE_PAIRS is imported from constants (single source of truth). Here it is used ONLY to convert a
# mechanical RPM to eRPM for the crossover-regime classifier and the eRPM display — NEVER to divide
# the already-mechanical tele.rpm. See constants.POLE_PAIRS.

# Built-in closed-loop PI gains — the single source shared by VelocityController's constructor
# defaults and velctl's fallback. These are the SIM-tuned values; the real-hardware plant gain is
# ~30x higher (930KV 6-step ~23 mech RPM per command-unit vs the sim's ~0.8), so a bench-verified
# profile SHOULD carry its own `control:` block (kp/ki/trim_max/blend_secs) — e.g. 930KV wants
# kp~0.03, ki~0.12, trim_max~400. Gains are plant-dependent, like the speed curve itself, so they
# live PER-PROFILE; these defaults only apply when a profile omits `control`.
DEFAULT_GAINS = {"kp": 0.4, "ki": 1.5, "trim_max": 200.0, "blend_secs": 0.3}


# ---------------------------------------------------------------------------
# The calibrated command->speed curve (per motor), and its runtime inverse.
# ---------------------------------------------------------------------------
class SpeedProfile:
    """A per-motor calibrated curve: monotonic (thrust, rpm) points + metadata.

    thrust is the SIGNED ESC COMMAND (only the non-negative up-sweep is stored; the inverse is
    odd-symmetric for reverse). thrust_for(rpm) is the runtime inverse: piecewise-linear,
    exact at the calibration points, clamped at both endpoints, odd-symmetric for -rpm.

    Loaded/saved via config.load_yaml / config._emit_yaml (same YAML io as the config codec).
    A non-monotonic curve is REJECTED on load (thrust must be strictly increasing, rpm
    non-decreasing) — a bad curve would invert ambiguously.
    """

    def __init__(self, points, *, motor="", pole_pairs=POLE_PAIRS, source="", crossover=None,
                 regimes=None, capabilities=None, control=None):
        # points: iterable of (thrust, rpm). Store sorted by thrust, validated. regimes (optional)
        # is a parallel per-point tag ("sine"/"line") velcal BELIEVED each point was in — kept in
        # lockstep order with points, for auditability. It is metadata only; the FF inverse
        # (thrust_for) never reads it.
        pts = [(float(t), float(r)) for t, r in points]
        if regimes is not None:
            regimes = list(regimes)
            if len(regimes) != len(pts):
                raise ValueError("regimes length must match points")
            order = sorted(range(len(pts)), key=lambda i: pts[i][0])
            pts = [pts[i] for i in order]
            regimes = [regimes[i] for i in order]
        else:
            pts.sort(key=lambda p: p[0])
        self._validate(pts)
        self.points = pts
        self.regimes = regimes
        self.motor = motor
        self.pole_pairs = int(pole_pairs)
        self.source = source
        self.crossover = dict(crossover) if crossover else None
        # Optional per-ESC capability flags (velcal/bench metadata) the runtime controller reads —
        # e.g. whether the firmware catches from above the crossover (`down_catch`) or has a proven
        # low-speed sine regime (`sine_lowspeed`). Purely additive: absent -> every flag False, so a
        # stock-Bluejay-like profile (no crossover, no capabilities) behaves exactly as before.
        self.capabilities = dict(capabilities) if capabilities else None
        # Optional per-motor closed-loop PI gains (kp/ki/trim_max/blend_secs). Plant-dependent, like
        # the speed curve — a bench-tuned profile carries its own, and velctl merges any explicit CLI
        # flag over these over DEFAULT_GAINS. Absent -> the runtime uses DEFAULT_GAINS unchanged.
        self.control = dict(control) if control else None

    def control_gain(self, name, default=None):
        """A closed-loop gain from the profile's `control:` block, or `default` when absent."""
        return (self.control or {}).get(name, default)

    def _capability(self, name):
        """A capability flag, defaulting False when the profile carries no `capabilities` block."""
        return bool(self.capabilities.get(name, False)) if self.capabilities else False

    @property
    def sine_lowspeed(self):
        """True iff the ESC has a proven low-speed forced-sine regime (velcal-attested)."""
        return self._capability("sine_lowspeed")

    @property
    def down_catch(self):
        """True iff the firmware can re-catch commutation when crossing the seam FROM ABOVE
        (descending through it). When False/absent the controller stages a set_speed that drops
        below the seam through ~0 so the rotor is re-acquired from below."""
        return self._capability("down_catch")

    @staticmethod
    def _validate(pts):
        if len(pts) < 2:
            raise ValueError("SpeedProfile needs >= 2 points")
        if pts[0][0] < 0:
            raise ValueError("SpeedProfile thrust points must be >= 0 (odd-symmetric inverse)")
        for (t0, r0), (t1, r1) in zip(pts, pts[1:]):
            if not t1 > t0:
                raise ValueError(f"thrust not strictly increasing at {t1} (after {t0})")
            if r1 < r0:
                raise ValueError(f"rpm decreases at thrust {t1} ({r1} < {r0}) — non-invertible")

    # -- runtime inverse: target RPM -> signed ESC command --
    def thrust_for(self, rpm):
        """Signed ESC command for a target mechanical RPM (piecewise-linear inverse).

        Exact at the calibration points, clamped to the endpoints outside the range, and
        odd-symmetric so thrust_for(-rpm) == -thrust_for(rpm) (thrust_for(0) == 0)."""
        sign = -1.0 if rpm < 0 else 1.0
        a = abs(float(rpm))
        pts = self.points
        if a <= pts[0][1]:
            return sign * pts[0][0]
        if a >= pts[-1][1]:
            return sign * pts[-1][0]
        for (t0, r0), (t1, r1) in zip(pts, pts[1:]):
            if r0 <= a <= r1:
                if r1 == r0:                     # rpm plateau -> take the lower command
                    return sign * t0
                frac = (a - r0) / (r1 - r0)
                return sign * (t0 + frac * (t1 - t0))
        return sign * pts[-1][0]                  # unreachable (guarded above)

    @property
    def max_rpm(self):
        return self.points[-1][1]

    # -- YAML io (reuse the config codec's emitter/loader; do NOT hand-roll) --
    def to_dict(self):
        d = {
            "motor": self.motor,
            "pole_pairs": self.pole_pairs,
            "source": self.source,
        }
        if self.crossover:
            d["crossover"] = self.crossover
        if self.capabilities:
            d["capabilities"] = self.capabilities
        if self.control:
            d["control"] = self.control
        pts_out = []
        for i, (t, r) in enumerate(self.points):
            pt = {"thrust": t, "rpm": r}
            if self.regimes is not None and self.regimes[i] is not None:
                pt["regime"] = self.regimes[i]     # velcal's believed regime (auditable seam)
            pts_out.append(pt)
        d["points"] = pts_out
        return d

    def save(self, path, header=None):
        text = config._emit_yaml(self.to_dict())
        with open(path, "w", encoding="utf-8") as fh:
            if header:
                for line in header.splitlines():
                    fh.write(f"# {line}\n" if not line.startswith("#") else f"{line}\n")
            fh.write(text)

    @classmethod
    def from_dict(cls, d):
        raw = d.get("points") or []
        pts, regimes = [], []
        have_regime = False
        for p in raw:
            if isinstance(p, dict):
                pts.append((p["thrust"], p["rpm"]))
                if "regime" in p:
                    have_regime = True
                regimes.append(p.get("regime"))
            else:                                 # tolerate [thrust, rpm] pairs
                pts.append((p[0], p[1]))
                regimes.append(None)
        return cls(pts, motor=d.get("motor", ""),
                   pole_pairs=d.get("pole_pairs", POLE_PAIRS),
                   source=d.get("source", ""), crossover=d.get("crossover"),
                   regimes=regimes if have_regime else None,
                   capabilities=d.get("capabilities"),
                   control=d.get("control"))

    @classmethod
    def load(cls, path):
        return cls.from_dict(config.load_yaml(path))


# ---------------------------------------------------------------------------
# Steady-speed measurement (velcal calibration helper).
# ---------------------------------------------------------------------------
def measure_steady_speed(esc, clock, thrust, enc_sign, settle_secs, measure_secs, max_temp):
    """Command a constant thrust; after a settle window sample the encoder speed and return
    (mean_rpm, ripple_std_rpm, peak_temp). Mirrors tune_sine_amp._drive_measure but reports
    MECH RPM (not deg/s). Paces every tick (< 500 ms deadman), polls temperature and raises
    Aborted over max_temp, and bails early on a clear stall. Encoder here is CALIBRATION only.
    """
    # Shared de-aliased reader: device `encv` (mechanical, valid at any speed) when available, else
    # the host-side guarded unwrap — the SAME math as the previous inline code, so the sim path is
    # byte-for-byte unchanged. This de-aliases velcal at high speed (where the raw 50 Hz unwrap folded).
    vr = VelReader(sign=enc_sign)
    samples = []
    peak_temp = None
    cmd_rpm = abs(thrust) / 1000.0 * FULLSCALE_RPM
    stall_rpm = 0.15 * cmd_rpm                     # below this = not really turning
    t_end = clock.now() + settle_secs + measure_secs
    t_measure = clock.now() + settle_secs
    next_tele = clock.now()
    while clock.now() < t_end:
        tick = clock.now()
        vel = vr.read(esc, DT)                      # signed mech RPM (encv, else host unwrap)
        if tick >= t_measure:
            samples.append(vel)
        # early stall-out: an excited/locked combo cooks the winding with no current sense —
        # if it isn't turning shortly into the measure window, stop NOW.
        if len(samples) >= 15 and abs(statistics.mean(samples[-15:])) < stall_rpm:
            esc.thrust(0)
            return statistics.mean(samples), float("inf"), peak_temp
        esc.thrust(thrust)
        if max_temp and tick >= next_tele:
            next_tele = tick + TELE_EVERY_S
            tp = esc.temperature()
            if tp is not None:
                peak_temp = tp if peak_temp is None else max(peak_temp, tp)
                if tp >= max_temp:
                    esc.thrust(0)
                    raise Aborted(f"over-temperature {tp}C >= {max_temp:.0f}C")
        _pace(clock, tick)
    esc.thrust(0)
    if len(samples) < 5:
        return 0.0, float("inf"), peak_temp
    return statistics.mean(samples), statistics.pstdev(samples), peak_temp


# ---------------------------------------------------------------------------
# Sensorless feed-forward velocity controller.
# ---------------------------------------------------------------------------
class VelocityController:
    """Feed-forward speed control + a liveness-faded PI trim (Phase A1 closed loop).

    Each tick: slew the setpoint toward the target at slew_rpm_s (so a step never demands a
    full-scale command jump), look the feed-forward command up from the calibrated curve, and add
    a PI trim on (setpoint - measured mech RPM). The measurement is the bidir-DShot `tele` frame's
    rpm (ALREADY mechanical — NEVER divided by pole pairs); a frame is LIVE only when
    |rpm| > TELE_MIN_MECH_RPM. The PI's AUTHORITY (weight w) ramps 0->1 over blend_secs while a live
    frame is seen and 1->0 while it goes stale, and the integrator resets when w hits 0. So above the
    seam (6-step, live telemetry) the loop closes; below it (forced sine, stale telemetry) it fades
    to pure feed-forward. Authority is keyed PURELY on tele-liveness (never the profile's crossover),
    so a stock profile with no crossover runs. The PI has a ±trim_max clamp and back-calculation
    anti-windup on BOTH that clamp and the outer ESC.thrust tmax clamp (mirrors
    PositionController.step). Telemetry is polled at most every tele_period; the SAME frame drives the
    temperature abort (over max_temp; max_temp=0 disables ONLY the temp check, not the feedback).
    With use_encoder the encoder speed is logged for an INDEPENDENT verify only — it NEVER feeds the
    command.
    """

    _W_BACKCALC_FLOOR = 0.1        # below this PI authority, skip the outer-clamp back-calc (the trim
                                   # barely reaches the ESC, so a clamp is FF-driven; the inner
                                   # trim_max clamp already bounds the integrator and w->0 resets it)

    def __init__(self, esc, profile, *, kp=DEFAULT_GAINS["kp"], ki=DEFAULT_GAINS["ki"],
                 trim_max=DEFAULT_GAINS["trim_max"], blend_secs=DEFAULT_GAINS["blend_secs"],
                 tele_period=DT, over_speed_rpm=None, stall_secs=1.0, slew_rpm_s=200.0,
                 max_temp=80.0, max_secs=30.0, use_encoder=False, enc_sign=1, stop_below_rpm=0.0):
        self.esc = esc
        self.profile = profile
        self.kp = float(kp)
        self.ki = float(ki)
        self.trim_max = float(trim_max)
        self.blend_secs = float(blend_secs)
        self.tele_period = float(tele_period)
        # over-speed net (live tele only): default clears twice the curve's top speed (and a 1200
        # floor so a tiny-range curve still has headroom); an explicit value overrides.
        self.over_speed_rpm = (float(over_speed_rpm) if over_speed_rpm is not None
                               else max(2.0 * profile.max_rpm, 1200.0))
        self.stall_secs = float(stall_secs)
        self.slew_rpm_s = float(slew_rpm_s)
        # A target at/below this |RPM| is a STOP request: the speed can't be held sensorlessly near
        # zero (no BEMF), so instead of servoing to it (which jitters and creeps) the loop commands a
        # true thrust 0 and disengages. Default 0 => only an exact `set_speed(0)` stops; raise it to
        # make any sub-floor target coast to a stop. This is what makes `rpm 0` actually STOP (arm/
        # disarm stay for enable/kill; motion is by speed command).
        self.stop_below_rpm = float(stop_below_rpm)
        self.max_temp = float(max_temp)
        self.max_secs = float(max_secs)
        self.use_encoder = bool(use_encoder)
        self.enc_sign = enc_sign
        self.target = 0.0            # commanded target mech RPM
        self.setpoint = 0.0          # slew-limited setpoint actually looked up
        self.last_temp = None
        self.peak_temp = None
        # --- closed-loop state (no module-level mutable state) ---
        self._i = 0.0               # PI integrator (trim units)
        self._w = 0.0               # PI authority weight, 0..1 (faded with tele liveness)
        self._tele_mech = None      # last LIVE measured mech RPM (signed to the setpoint), else None
        self._live = False          # last telemetry poll was a live 6-step frame
        self._stale_since = None    # time tele first went stale while commanding into "line"
        self._pending = None        # a staged set_speed target (down-catch through ~0)
        self._last_sign = 1.0       # last non-zero setpoint sign (for signing a tele MAGNITUDE at sp==0)

    # -- setpoint --
    def set_speed(self, rpm):
        """Set the commanded target speed (RPM). The loop slews the setpoint toward it.

        DOWN-CATCH staging: if the profile has a crossover and this call drops FROM ABOVE the seam
        to BELOW it on an ESC that cannot re-catch from above (capabilities.down_catch False/absent),
        route the setpoint through ~0 first (stage the real target in _pending) so the rotor is
        re-acquired from below rather than dropped across the handoff. Inert (a plain target set)
        when the profile has no crossover, the ESC advertises down_catch, or the move isn't a
        line->sine descent."""
        rpm = float(rpm)
        if (self.profile.crossover and not self.profile.down_catch
                and self.regime(rpm) == "sine" and self.regime(self.setpoint) == "line"):
            self._pending = rpm
            self.target = 0.0
        else:
            self._pending = None
            self.target = rpm

    def _slew(self, dt):
        step = self.slew_rpm_s * dt
        if self.setpoint < self.target:
            self.setpoint = min(self.target, self.setpoint + step)
        elif self.setpoint > self.target:
            self.setpoint = max(self.target, self.setpoint - step)
        return self.setpoint

    # -- ADVISORY regime classifier (display / stall heuristic / down-catch routing) --
    def regime(self, rpm):
        """Which firmware regime a speed falls in per the profile's crossover: "sine" below
        the seam, "line" above. Returns "sine" if the profile carries no crossover.

        ADVISORY ONLY. This is used for display, the stall heuristic, and down-catch routing — it
        is NEVER the PI-authority signal (that is telemetry liveness alone; see run()). It classifies
        the NOMINAL rpm against the profile's `up_erpm`, which can DISAGREE with (a) the effective
        rounded-byte threshold (`up_erpm` vs cross_up*39.0625) and (b) the command's ACTUAL firmware
        regime near the seam (the firmware switches on commanded/actual eRPM + hysteresis, not on the
        nominal rpm). So near the seam a nominal-"line" target can actually run in sine (the curve
        "gap"); the stall guard guards against this by also requiring the setpoint to be at/above the
        profile's genuinely 6-step-REACHABLE floor (`_line_floor`), not just above `up_erpm`."""
        cx = self.profile.crossover
        if not cx:
            return "sine"
        up = cx.get("up_erpm")
        if up is None:
            return "sine"
        return "line" if abs(rpm) * self.profile.pole_pairs >= up else "sine"

    def _line_floor(self):
        """Lowest setpoint |RPM| genuinely REACHABLE in the 6-step ("line") regime per THIS profile
        (not the sim's plant gain): the min rpm among the profile's line-tagged calibration points if
        it tags regimes, else the seam rpm (up_erpm / pole_pairs). Setpoints below this fall in the
        crossover GAP / sine band — pure feed-forward with no telemetry — so the stall guard must NOT
        treat them as "should be 6-step". None when the profile has no usable crossover."""
        cx = self.profile.crossover
        if not cx:
            return None
        if self.profile.regimes is not None:
            line_rpms = [r for (_, r), reg in zip(self.profile.points, self.profile.regimes)
                         if reg == "line"]
            if line_rpms:
                return min(line_rpms)
        up = cx.get("up_erpm")
        return (up / self.profile.pole_pairs) if up is not None else None

    @staticmethod
    def _measure(tel, setpoint, fallback_sign=1.0):
        """A `tele` frame -> signed measured mechanical RPM if the frame is LIVE, else None.

        INVARIANT: `tel.rpm` is ALREADY mechanical RPM (the firmware pre-divides the DShot eRPM by
        pole pairs), so it is used DIRECTLY — dividing by POLE_PAIRS here would be the 1/7 double-
        division bug. A frame is live only when |rpm| > TELE_MIN_MECH_RPM (rejects the garbage
        after-arm frames and forced-sine's stale ~0). Hardware reports a MAGNITUDE; we re-attach the
        COMMANDED sign, i.e. meas = copysign(|rpm|, setpoint) — matching err = setpoint - meas. At
        setpoint==0 the commanded sign is ambiguous, so fall back to the last-commanded sign."""
        if tel is None or tel.rpm is None:
            return None
        rpm = float(tel.rpm)
        if abs(rpm) <= TELE_MIN_MECH_RPM:
            return None
        if setpoint > 0:
            sign = 1.0
        elif setpoint < 0:
            sign = -1.0
        else:
            sign = 1.0 if fallback_sign >= 0 else -1.0
        return sign * abs(rpm)

    def _closed_loop_trim(self, setpoint, dt):
        """PI trim on (setpoint - measured mech RPM), clamped to ±trim_max with back-calculation
        anti-windup on the clamp (mirrors PositionController.step). Returns the UNBLENDED trim; the
        run loop multiplies by the liveness weight w and folds the outer ESC tmax clamp back into
        the integrator. Returns 0 (and does not integrate) when there is no live measurement.

        The integrator only ACCUMULATES on a live frame: during the stale->live fade-out the last
        measurement is frozen, so integrating against it would wind the integral against stale data
        (a wrong-signed dip at the down-crossing). The proportional term still fades out via w."""
        if self._tele_mech is None:
            return 0.0
        err = setpoint - self._tele_mech
        if self._live:                                   # accumulate only against a FRESH live frame
            self._i += self.ki * err * dt
        u = self.kp * err + self._i
        u_clamped = max(-self.trim_max, min(self.trim_max, u))
        if u != u_clamped:                               # saturated -> unwind the integral (back-calc)
            self._i = max(-self.trim_max, min(self.trim_max, u_clamped - self.kp * err))
            u = u_clamped
        return u

    def _read_enc_rpm(self, prev_raw, vel, dt):
        """Verify-only encoder speed (mech RPM), low-passed. Returns (rpm|None, prev_raw, vel).
        NEVER used to compute the command — pure logging (the closed loop trims on TELEMETRY)."""
        enc = self.esc.encoder()
        if enc is None or not enc.healthy:
            return None, prev_raw, vel
        if prev_raw is None:
            return None, enc.raw, vel
        d = ((enc.raw - prev_raw + COUNTS_PER_REV // 2) % COUNTS_PER_REV) - COUNTS_PER_REV // 2
        if abs(d) > DELTA_FAULT_FRAC * (COUNTS_PER_REV // 2):
            return None, enc.raw, vel
        inst = self.enc_sign * d / COUNTS_PER_REV / max(dt, 1e-6) * 60.0
        vel += VEL_LP_ALPHA * (inst - vel)
        return vel, enc.raw, vel

    def run(self, clock, on_row=None):
        """Drive the target speed for up to max_secs. Calls on_row(t, target, setpoint, thrust,
        temp, enc_rpm, tele_rpm, trim) per tick if given (tele_rpm = the live measured mech RPM or
        None; trim = the BLENDED trim actually added to the feed-forward). Returns an exit-reason
        string. Raises Aborted on over-temperature / over-speed / stall. The caller ALWAYS disarms
        (finally)."""
        t0 = clock.now()
        last_t = t0
        next_tele = t0
        prev_raw = None
        vel = 0.0
        enc_fails = 0
        enc_warned = False
        reason = "completed"
        while clock.now() - t0 < self.max_secs:
            tick = clock.now()
            dt = tick - last_t
            last_t = tick
            if dt <= 0:
                dt = DT
            t = tick - t0

            # STOP request: a target at/below stop_below_rpm can't be held sensorlessly (no BEMF near
            # zero) -> command a true thrust 0 and disengage the loop, so `set_speed(0)` actually
            # STOPS instead of the FF/slew/PI creeping the motor. Snap the setpoint to 0 too.
            if abs(self.target) <= self.stop_below_rpm:
                self.setpoint = 0.0
                self._i = 0.0
                self._w = 0.0
                self._live = False
                self._pending = None
                self._stale_since = None
                sent = self.esc.thrust(0)
                if on_row is not None:
                    on_row(t, self.target, 0.0, sent, None, None, None, 0.0)
                _pace(clock, tick)
                continue

            sp = self._slew(dt)
            if sp > 0:
                self._last_sign = 1.0
            elif sp < 0:
                self._last_sign = -1.0
            # DOWN-CATCH: a staged (line->sine) set_speed promotes to its real target once we have
            # descended to ~0 / dropped below the seam (telemetry gone stale in the sine regime).
            if self._pending is not None and (abs(self.setpoint) < 1.0
                                              or (not self._live and self.regime(sp) == "sine")):
                self.target = self._pending
                self._pending = None

            # -- independent --encoder verify-log (NEVER feeds the command) --
            enc_rpm = None
            if self.use_encoder:
                enc_rpm, prev_raw, vel = self._read_enc_rpm(prev_raw, vel, dt)
                if enc_rpm is None and prev_raw is not None:
                    enc_fails += 1
                    if enc_fails >= ENC_FAIL_MAX and not enc_warned:
                        print("#   [warn] encoder verify unreliable (magnet/unwrap faults); "
                              "sensorless command unaffected")
                        enc_warned = True
                else:
                    enc_fails = 0

            # -- telemetry poll (throttled by tele_period): the SAME frame drives feedback AND the
            #    temperature abort. Each poll draws sim RNG when the crossover is ON, hence the pace. --
            temp = None
            if tick >= next_tele:
                next_tele = tick + self.tele_period
                tel = self.esc.telemetry()
                meas = self._measure(tel, sp, self._last_sign)
                self._live = meas is not None
                if self._live:
                    self._tele_mech = meas
                if tel is not None and tel.temp is not None:
                    temp = tel.temp
                    self.last_temp = temp
                    self.peak_temp = temp if self.peak_temp is None else max(self.peak_temp, temp)
                    if self.max_temp and temp >= self.max_temp:
                        self.esc.thrust(0)
                        raise Aborted(f"over-temperature: ESC {temp}C >= --max-temp "
                                      f"{self.max_temp:.0f}C — lower speed / cool down")

            # -- PI authority (w) fades with tele liveness: 0->1 while live, 1->0 while stale, over
            #    blend_secs. On w==0 reset the integrator so a re-arm from below the seam starts clean. --
            rate = dt / self.blend_secs if self.blend_secs > 0 else 1.0
            if self._live:
                self._w = min(1.0, self._w + rate)
            else:
                self._w = max(0.0, self._w - rate)
            if self._w <= 0.0:
                self._i = 0.0

            # -- feed-forward + blended PI trim --
            ff = self.profile.thrust_for(sp)
            trim = self._closed_loop_trim(sp, dt)
            applied = self._w * trim
            cmd = ff + applied
            sent = self.esc.thrust(cmd)

            # -- back-calculation anti-windup on the OUTER ESC tmax clamp: if the ESC clamped the
            #    command, fold the trim the ESC ACTUALLY delivered back into the integrator so a
            #    saturated command can't wind it up (mirrors PositionController). Only when the trim
            #    materially drove the clamp: skip when w is below a real floor (the integrator is
            #    already bounded by trim_max and about to reset as w->0), and skip when the FF term
            #    alone saturated (delivered opposes the desired trim) so the integrator can't flip
            #    against the true error. `delivered` is bounded by the desired trim, so no /w blow-up. --
            if (self._tele_mech is not None and self._w >= self._W_BACKCALC_FLOOR
                    and sent != int(cmd)):
                desired = applied                              # w*trim we tried to add above the FF
                delivered = sent - int(ff)                     # trim the ESC actually delivered
                if desired != 0.0 and delivered * desired > 0.0 and abs(delivered) < abs(desired):
                    err = sp - self._tele_mech
                    unblended = delivered / self._w            # |unblended| < |trim| <= trim_max
                    self._i = max(-self.trim_max, min(self.trim_max, unblended - self.kp * err))

            # -- safety guards on the LIVE measurement (never on the profile/crossover config) --
            if self._live and abs(self._tele_mech) > self.over_speed_rpm:
                self.esc.thrust(0)
                raise Aborted(f"over-speed: |tele|={abs(self._tele_mech):.0f} RPM > "
                              f"{self.over_speed_rpm:.0f} — lower speed / check the curve")
            # stall: the setpoint is at/above the profile's genuinely 6-step-REACHABLE floor (so we
            # EXPECT live telemetry) yet the loop is still commanding into that region (the trim
            # hasn't backed the command off) and telemetry never went live -> the ESC failed to reach
            # 6-step. Keyed on the PROFILE'S OWN reachable floor (not the sim's plant gain, and not the
            # raw nominal regime), so a "gap" setpoint whose command sits in sine is NOT a stall.
            floor = self._line_floor()
            commanding_line = (floor is not None and abs(sp) >= floor
                               and abs(sent) >= abs(ff) - self.trim_max)
            if commanding_line and not self._live:
                if self._stale_since is None:
                    self._stale_since = tick
                elif tick - self._stale_since >= self.stall_secs:
                    self.esc.thrust(0)
                    raise Aborted(f"stall: setpoint {sp:.0f} RPM is in the 6-step range but telemetry "
                                  f"stayed stale for {self.stall_secs:.1f}s (never reached 6-step)")
            else:
                self._stale_since = None

            if on_row is not None:
                tele_rpm = self._tele_mech if self._live else None
                on_row(t, self.target, sp, sent, temp, enc_rpm, tele_rpm, applied)
            _pace(clock, tick)
        return reason
