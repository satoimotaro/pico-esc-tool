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

SCOPE (v1): feed-forward ONLY. There is NO runtime closed loop — the eRPM-PI trim is the NEXT
phase. regime()/_closed_loop_trim() are a documented, regime-aware SEAM that returns 0 today.
The encoder is used ONLY for calibration (velcal) and, at runtime with --encoder, for an
independent VERIFY-LOG that NEVER feeds back into the command. Sensorless is the default.

All thrust is routed through ESC.thrust() (-> PosDrive.send_thrust, the single clamp/choke);
every loop branch paces at DT (well under the 500 ms firmware deadman); temperature is polled
and the run aborts over --max-temp; the CLI always disarms on every exit.
"""
from __future__ import annotations

import statistics

from . import config
from .constants import COUNTS_PER_REV, FULLSCALE_RPM, POLE_PAIRS  # noqa: F401 (re-exported)
from .control import (DELTA_FAULT_FRAC, DT, ENC_FAIL_MAX, TELE_EVERY_S,
                      VEL_LP_ALPHA, VelReader, _pace)
from .drive import Aborted

# POLE_PAIRS is imported from constants (single source of truth). Here it is used ONLY to convert a
# mechanical RPM to eRPM for the crossover-regime classifier and the eRPM display — NEVER to divide
# the already-mechanical tele.rpm. See constants.POLE_PAIRS.


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
                 regimes=None):
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
                   regimes=regimes if have_regime else None)

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
    """Sensorless feed-forward speed control: slew the setpoint, look the command up from the
    calibrated curve, send it. NO runtime feedback in v1.

    The setpoint slews toward the commanded target at slew_rpm_s so a step never demands a
    full-scale command jump. Temperature is polled every TELE_EVERY_S and the run aborts over
    max_temp. With use_encoder the encoder speed is logged for an INDEPENDENT verify only — it
    NEVER feeds the command (that is the deferred eRPM-PI phase).
    """

    def __init__(self, esc, profile, *, slew_rpm_s=200.0, max_temp=80.0, max_secs=30.0,
                 use_encoder=False, enc_sign=1):
        self.esc = esc
        self.profile = profile
        self.slew_rpm_s = float(slew_rpm_s)
        self.max_temp = float(max_temp)
        self.max_secs = float(max_secs)
        self.use_encoder = bool(use_encoder)
        self.enc_sign = enc_sign
        self.target = 0.0            # commanded target mech RPM
        self.setpoint = 0.0          # slew-limited setpoint actually looked up
        self.last_temp = None
        self.peak_temp = None

    # -- setpoint --
    def set_speed(self, rpm):
        """Set the commanded target speed (RPM). The loop slews the setpoint toward it."""
        self.target = float(rpm)

    def _slew(self, dt):
        step = self.slew_rpm_s * dt
        if self.setpoint < self.target:
            self.setpoint = min(self.target, self.setpoint + step)
        elif self.setpoint > self.target:
            self.setpoint = max(self.target, self.setpoint - step)
        return self.setpoint

    # -- documented regime-aware SEAM for the DEFERRED eRPM-PI closed loop (v1: no-op) --
    def regime(self, rpm):
        """Which firmware regime a speed falls in per the profile's crossover: "sine" below
        the seam, "line" above. Informational in v1 — the FUTURE eRPM-PI trim will be
        regime-aware (different gains either side of the handoff). Returns "sine" if the
        profile carries no crossover.

        SEAM CAVEAT for the next phase: this classifies off the NOMINAL target rpm against the
        profile's `up_erpm`, which can DISAGREE with (a) the effective rounded-byte threshold
        (`up_erpm` vs cross_up*39.0625) and (b) the command's ACTUAL firmware regime near the
        seam (the firmware switches on commanded/actual eRPM + hysteresis, not on target rpm).
        When wiring the eRPM-PI, key the classifier off the expected COMMAND's regime (or the
        measured eRPM), not the raw target. The v1 FF command path never calls regime(), so
        this disagreement is harmless today."""
        cx = self.profile.crossover
        if not cx:
            return "sine"
        up = cx.get("up_erpm")
        if up is None:
            return "sine"
        return "line" if abs(rpm) * self.profile.pole_pairs >= up else "sine"

    def _closed_loop_trim(self, setpoint, meas_rpm):
        """DEFERRED eRPM-PI hook. v1 is pure feed-forward, so this ALWAYS returns 0 (the
        encoder/eRPM NEVER feeds the command). The next phase adds a regime-aware PI trim on
        (setpoint - meas_rpm) here; the run loop already threads the measured speed in.

        NEXT-DEV CONTRACT (make the seam unambiguous):
          * Today `meas_rpm` is ONLY the --encoder verify value (None unless use_encoder), so
            feedback is effectively OFF. Turning it ON means (a) removing the always-0 return
            and (b) DECIDING the feedback source per regime:
              - encoder (AS5600) speed: valid in BOTH regimes but is the calibration sensor;
              - esc.telemetry().rpm (bidir-DShot, MECHANICAL RPM — firmware pre-divides eRPM by pole
                pairs): LIVE in 6-step ("line"), but STALE/absent in forced-sine ("sine") — do not
                trust it below the seam.
            A robust trim likely uses the encoder below the seam and telemetry above it (see
            regime()), with anti-windup and a clamp so a bad sample can't runaway the command.
          * Keep the FF term (thrust_for) as the feed-forward; this returns only the trim."""
        return 0.0

    def _read_enc_rpm(self, prev_raw, vel, dt):
        """Verify-only encoder speed (mech RPM), low-passed. Returns (rpm|None, prev_raw, vel).
        NEVER used to compute the command — pure logging in v1."""
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
        temp, enc_rpm) per tick if given. Returns an exit-reason string. Raises Aborted on
        over-temperature. The caller ALWAYS disarms (finally)."""
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
            t = tick - t0

            sp = self._slew(dt if dt > 0 else DT)
            enc_rpm = None
            if self.use_encoder:
                enc_rpm, prev_raw, vel = self._read_enc_rpm(prev_raw, vel, dt if dt > 0 else DT)
                if enc_rpm is None and prev_raw is not None:
                    enc_fails += 1
                    if enc_fails >= ENC_FAIL_MAX and not enc_warned:
                        print("#   [warn] encoder verify unreliable (magnet/unwrap faults); "
                              "sensorless command unaffected")
                        enc_warned = True
                else:
                    enc_fails = 0

            # Pure feed-forward: command = curve inverse + (deferred, always-0) closed-loop trim.
            cmd = self.profile.thrust_for(sp) + self._closed_loop_trim(sp, enc_rpm)
            sent = self.esc.thrust(cmd)

            # temperature watch (polled right after the keep-alive so the deadman never starves)
            temp = None
            if self.max_temp and tick >= next_tele:
                next_tele = tick + TELE_EVERY_S
                temp = self.esc.temperature()
                if temp is not None:
                    self.last_temp = temp
                    self.peak_temp = temp if self.peak_temp is None else max(self.peak_temp, temp)
                    if temp >= self.max_temp:
                        self.esc.thrust(0)
                        raise Aborted(f"over-temperature: ESC {temp}C >= --max-temp "
                                      f"{self.max_temp:.0f}C — lower speed / cool down")

            if on_row is not None:
                on_row(t, self.target, sp, sent, temp, enc_rpm)
            _pace(clock, tick)
        return reason
