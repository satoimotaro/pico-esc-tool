# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""pico_esc.aztrace — parse `az dump` traces and score them.

WHY THIS MODULE EXISTS, AND WHAT IT REFUSES TO DO
-------------------------------------------------
Every azimuth result on record came from polling `az status` at 0.6-1.2 s and reporting
`settled |err| min..max` over the last 6-8 samples. That single number conflates three
independent questions, and answering them together is what sent the tuning sideways:

    convergence          did the pod STOP?              rms(vel), peak|vel|   <- primary
    limit cycle          is it oscillating?             peak-to-peak of pos
    steady-state error   where did it stop?             mean(err)             <- this is what I is for

A pod sitting still 70 deg off target and a pod swinging +/-70 deg produce the SAME
`|err| min..max`. They need opposite fixes. So `metrics()` reports the three separately and
there is deliberately no combined "score" to sort on.

Convergence is a statement about VELOCITY. The later sweeps read only `err` from `az status`
and never recorded `vel` at all, which is why no run on record can be shown to have converged
or not. The firmware ring (`az dump`) exists to end that, and this module is its reader.

`worst` — max|target - pos| over a whole run — is NOT here either. It is dominated by the
spin-up transient, and selecting on it is exactly how the one configuration that did converge
(kp 3, ki 0.5, kd 0.3) was discarded in favour of one that did not.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

# Command counts at which the ESC hands off forced sine -> 6-step. Below it the azimuth channel
# has no authority; above it the pod jumps to hundreds of rpm. Overridden by the dump header.
DEFAULT_SEAM = 515


@dataclass
class AzTrace:
    """One `az dump`: the configuration that produced it, plus the sample columns."""

    config: dict = field(default_factory=dict)
    t: list[float] = field(default_factory=list)      # seconds, relative to the first sample
    pos: list[float] = field(default_factory=list)    # deg, relative to where the run started
    vel: list[float] = field(default_factory=list)    # deg/s, signed
    s: list[float] = field(default_factory=list)      # delivered azimuth command, counts (trim in)
    ivel: list[float] = field(default_factory=list)   # integral state, counts
    carrier: list[float] = field(default_factory=list)  # LIVE carrier — varies through the spin-up

    def __len__(self):
        return len(self.t)

    @property
    def duration(self):
        return (self.t[-1] - self.t[0]) if len(self.t) > 1 else 0.0

    @property
    def rate_hz(self):
        """Actual sample rate, from the median interval — NOT the nominal decimation.

        The firmware self-paces off micros(), so the real rate is whatever the loop managed.
        Trusting the nominal rate is how a loop that silently ran at half speed would go unnoticed.
        """
        if len(self.t) < 2:
            return 0.0
        d = sorted(self.t[i + 1] - self.t[i] for i in range(len(self.t) - 1))
        med = d[len(d) // 2]
        return (1.0 / med) if med > 0 else 0.0

    def window(self, secs):
        """Indices of the last `secs` seconds (all of it if the trace is shorter)."""
        if not self.t:
            return []
        if secs is None or secs <= 0 or secs >= self.duration:
            return list(range(len(self.t)))
        cut = self.t[-1] - secs
        return [i for i, tv in enumerate(self.t) if tv >= cut]


def parse_dump(lines) -> AzTrace:
    """Parse the reply of `az dump` (header line, CSV header, rows). Tolerates the trailing 'ok'.

    Raises ValueError if the `azd|` header is missing — a trace with no header cannot say what
    configuration produced it, and an unattributable trace is not evidence.
    """
    if isinstance(lines, str):
        lines = lines.splitlines()
    rows, cfg, seen_header = [], None, False
    for raw in lines:
        ln = raw.strip()
        if not ln or ln in ("ok",):
            continue
        if ln.startswith("azd|"):
            cfg = {}
            for f in ln.split("|")[1:]:
                if "=" not in f:
                    continue
                k, v = f.split("=", 1)
                try:
                    cfg[k] = float(v) if ("." in v or "e" in v.lower()) else int(v)
                except ValueError:
                    cfg[k] = v
            continue
        if ln.startswith("t_ms,"):
            seen_header = True
            continue
        if ln.startswith("err "):
            raise ValueError(f"device refused the dump: {ln}")
        parts = ln.split(",")
        if len(parts) not in (5, 6):          # 6 since the live carrier was added
            continue
        try:
            rows.append(tuple(float(p) for p in parts))
        except ValueError:
            continue
    if cfg is None:
        raise ValueError("no 'azd|' header in the dump — cannot attribute the trace to a config")
    tr = AzTrace(config=cfg)
    for row in rows:
        t_ms, pos, vel, s, ivel = row[:5]
        tr.t.append(t_ms / 1000.0)
        tr.pos.append(pos)
        tr.vel.append(vel)
        tr.s.append(s)
        tr.ivel.append(ivel)
        tr.carrier.append(row[5] if len(row) > 5 else float(tr.config.get("carrier", 0.0)))
    if seen_header and not rows and cfg.get("n", 0):
        raise ValueError(f"header claims n={cfg['n']} but no rows parsed")
    return tr


def seam_margin(tr: AzTrace) -> list[float]:
    """Per-sample margin, in command counts, from the NEARER motor to the 6-step seam.

    Both motors sit at |carrier| +/- s, so the nearer one is |carrier| - |s|. This is the channel
    that predicted the host blow-ups: in the 17:38Z trace every excursion is immediately preceded
    by saturation with motor B at 531 against a 515 seam. Sign conventions do not matter here —
    only magnitudes reach the seam.
    """
    seam = abs(float(tr.config.get("seam", DEFAULT_SEAM)))
    # Use the LIVE carrier where the trace has it: during the transition the margin is set by the
    # carrier of that instant, and taking the final value hides exactly the interval that matters.
    cs = tr.carrier if len(tr.carrier) == len(tr.s) else [float(tr.config.get("carrier", 0.0))] * len(tr.s)
    return [abs(c) - abs(sv) - seam for c, sv in zip(cs, tr.s)]


def metrics(tr: AzTrace, window=5.0, target=None) -> dict:
    """Score the last `window` seconds. Three independent answers, deliberately not combined.

    `target` defaults to the dump header's `tgt`. Errors are target - pos, in the same frame the
    firmware uses (pos is zeroed at `az on`).
    """
    idx = tr.window(window)
    if not idx:
        return {"n": 0}
    tgt = float(tr.config.get("tgt", 0.0)) if target is None else float(target)
    vel = [tr.vel[i] for i in idx]
    pos = [tr.pos[i] for i in idx]
    err = [tgt - p for p in pos]
    sv = [tr.s[i] for i in idx]
    marg = [seam_margin(tr)[i] for i in idx]
    n = len(idx)
    mean_err = sum(err) / n
    # authority rail: |u| = |s - trim| at the +/-smax clamp
    trim = float(tr.config.get("trim", 0.0))
    smax = float(tr.config.get("smax", 0.0))
    railed = sum(1 for x in sv if smax > 0 and abs(abs(x - trim) - smax) < 0.05)
    return {
        "n": n,
        "secs": tr.t[idx[-1]] - tr.t[idx[0]],
        "rate_hz": tr.rate_hz,
        # --- convergence: PRIMARY. Did it stop? ---
        "vel_rms": math.sqrt(sum(v * v for v in vel) / n),
        "vel_peak": max(abs(v) for v in vel),
        # --- limit cycle: is it oscillating? ---
        "pos_pkpk": max(pos) - min(pos),
        # --- steady-state error: where did it stop? (what the integral is for) ---
        "err_mean": mean_err,
        "err_std": math.sqrt(sum((e - mean_err) ** 2 for e in err) / n),
        # --- actuator health ---
        "s_mean": sum(sv) / n,
        "seam_margin_min": min(marg),
        "sat_frac": railed / n,
    }


def verdict(m: dict, vel_rms_ok=25.0, pkpk_ok=10.0) -> str:
    """A blunt reading of `metrics()`. Thresholds are provisional — see the caveat below.

    The plant time constant has never been measured, so there is no principled number yet for
    "stopped". 25 deg/s is roughly half of one command count's worth of azimuth rate (1 count
    ~= 43.6 deg/s in air), i.e. below the actuator's own resolution — it cannot do better.
    Re-derive both thresholds once `az step` gives the real gain and tau.
    """
    if not m.get("n"):
        return "no data"
    if m["vel_rms"] <= vel_rms_ok and m["pos_pkpk"] <= pkpk_ok:
        return f"CONVERGED (offset {m['err_mean']:+.1f} deg)"
    if m["vel_rms"] <= vel_rms_ok:
        return f"drifting slowly (pk-pk {m['pos_pkpk']:.1f} deg)"
    return f"NOT converged (vel_rms {m['vel_rms']:.0f} deg/s, pk-pk {m['pos_pkpk']:.1f} deg)"


# ---------------------------------------------------------------------------------
# Plant identification from an `az step` trace.
# ---------------------------------------------------------------------------------
def _median(xs):
    s = sorted(xs)
    return s[len(s) // 2]


def find_step_onset(tr: AzTrace):
    """Index where the commanded `s` steps, or None if it never does.

    `az step` holds s = 0 (i.e. s = trim as delivered) for a pre-roll before applying the step, so
    both phases are in the one trace. Reading the transition off the COMMAND column makes the fit
    independent of `s_trim` — the step is (post - pre), whatever the trim happened to be.
    """
    n = len(tr)
    if n < 8:
        return None
    k = max(2, n // 20)
    s_pre, s_post = _median(tr.s[:k]), _median(tr.s[-k:])
    ds = s_post - s_pre
    if abs(ds) < 0.5:                       # under half a command count: no step happened
        return None
    thresh = s_pre + 0.5 * ds
    for i in range(n):
        if (tr.s[i] - thresh) * (1 if ds > 0 else -1) >= 0:
            return i
    return None


def fit_step(tr: AzTrace, s_cmd=None, settle_frac=0.25) -> dict:
    """First-order fit of an open-loop step: v(t) = v0 + K*ds*(1 - exp(-t/tau)).

    Returns gain (deg/s per command count), tau (s), v_inf and an R^2 over the fitted region.

    EVERY GAIN ON THIS POD RESTS ON AN ASSUMED tau OF 0.1-0.2 s THAT HAS NEVER BEEN MEASURED.
    This is the measurement. With K and tau in hand the stable maximum kp is computed rather than
    swept: position is the integral of a velocity source of lag tau, so the closed loop is
    tau*s^2 + (1+kd)*s + kp = 0 and zeta = (1+kd)/(2*sqrt(kp*tau)).

    Two things this does NOT assume:
    - that the pod was still when the step landed. The two ESCs start on independent ramps and the
      differential sends nearly all of that mismatch to the azimuth, so the pre-roll's measured
      velocity is subtracted as a baseline rather than taken to be zero.
    - that `s_cmd` is what the plant saw. beginStep clamps to +/-smax, and the delivered command
      carries `s_trim`; the step is read as (post - pre) off the command column instead.

    tau comes from a least-squares line through ln(1 - (v-v0)/(v_inf-v0)), not from the 63%
    crossing: one noisy sample decides the crossing, whereas the regression uses the whole rise.
    """
    if len(tr) < 8:
        return {"ok": False, "why": "trace too short"}

    # The device reports the onset in the dump header. Prefer it: the pre-roll runs the CLOSED
    # loop, so `s` varies continuously and cannot be read as "constant, then a step".
    onset = None
    if "step_ms" in tr.config and float(tr.config["step_ms"]) > 0:
        t_step = float(tr.config["step_ms"]) / 1000.0 + tr.t[0]
        onset = next((i for i, tv in enumerate(tr.t) if tv >= t_step), None)
    if onset is None:
        onset = find_step_onset(tr)
    if onset is None or onset < 3:
        # No pre-roll in this trace (or the step landed immediately): fall back to a zero baseline
        # and the delivered command minus the trim.
        onset, v0 = 0, 0.0
        ds = float(s_cmd) if s_cmd is not None else (
            (sum(tr.s) / len(tr.s)) - float(tr.config.get("trim", 0.0)))
    else:
        k = max(2, onset // 3)
        v0 = sum(tr.vel[onset - k:onset]) / k         # settled baseline, measured
        ds = float(tr.config["step_s"]) if "step_s" in tr.config else (
            _median(tr.s[onset:]) - _median(tr.s[:onset]))
    if abs(ds) < 1e-6:
        return {"ok": False, "why": "step size is zero — nothing to identify"}

    post = list(range(onset, len(tr)))
    t_on = tr.t[onset]
    tail_cut = tr.t[-1] - max((tr.t[-1] - t_on) * settle_frac, 1e-9)
    tail = [i for i in post if tr.t[i] >= tail_cut]
    v_inf = sum(tr.vel[i] for i in tail) / len(tail)
    dv = v_inf - v0
    if abs(dv) < 1e-6:
        return {"ok": False, "why": "no velocity change across the step — the pod never responded"}

    xs, ys = [], []
    for i in post:
        r = (tr.vel[i] - v0) / dv
        if 0.02 < r < 0.92:                    # skip the noisy foot and the flat top
            xs.append(tr.t[i] - t_on)
            ys.append(math.log(1.0 - r))       # = -t/tau
    if len(xs) < 4:
        return {"ok": False, "why": f"only {len(xs)} usable points on the rise — step too short?"}

    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    if sxx <= 0:
        return {"ok": False, "why": "degenerate time axis"}
    slope = sxy / sxx                          # = -1/tau
    if slope >= 0:
        return {"ok": False, "why": "velocity is not rising toward a limit — not a first-order step"}
    tau = -1.0 / slope
    inter = my - slope * mx
    ss_tot = sum((y - my) ** 2 for y in ys)
    ss_res = sum((y - (slope * x + inter)) ** 2 for x, y in zip(xs, ys))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    return {"ok": True, "tau": tau, "gain": dv / ds, "v_inf": v_inf, "v0": v0,
            "step_s": ds, "r2": r2, "n_fit": n, "onset_s": t_on - tr.t[0]}


def suggest_gains(fit: dict, zeta=1.0, kd=0.3) -> dict:
    """Turn a measured (gain, tau) into gains, instead of sweeping for them.

    zeta = (1+kd)/(2*sqrt(kp*tau))  ->  kp = ((1+kd)/(2*zeta))^2 / tau
    kff  = 1/gain, the feedforward that makes a velocity setpoint cost the command that produces it.

    Deliberately silent on ki: the integral has to track the DRIFT of the null (measured at
    +24 -> +45 rpm within one 30 s run), so its gain comes from that drift rate, not from tau.
    Measure the drift with the loop off before choosing it.
    """
    if not fit.get("ok"):
        return {"ok": False, "why": fit.get("why", "no fit")}
    tau = fit["tau"]
    kp = ((1.0 + kd) / (2.0 * zeta)) ** 2 / tau
    return {"ok": True, "kp": kp, "kd": kd, "kff": 1.0 / fit["gain"], "tau": tau,
            "zeta": zeta, "note": "ki must come from the null-drift rate, not from tau"}
