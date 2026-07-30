# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""pico_esc.measured — the MEASURED plant inventory, one provenance record per number.

WHY THIS FILE EXISTS. `host/sim_obs.py` used to model this plant with four constants
(a 0.18 s forced-sine lag, a 0.025 s 6-step lag, a 250 rpm crossover threshold, a 0.28 damping ratio) that were fitted to a trace
with no units, no source and no mechanical configuration. The 0.18 s lag in particular was fitted to
COGGING NOISE — the sysid step 65->407 wanders 19..249 rpm around a 145 rpm forced field, so any
first-order rise fitted there is fitting the dispersion, not the rotor. Numbers like that cannot be
re-measured, cannot be checked, and silently mix rotors: `J` came from the rotor WITH the propeller,
the 2026-07-30 brake data came from the BARE shaft, and nothing in the code said so.

So every number here carries a `PROVENANCE[name]` record: value, units, the tool and file it came
from, the MECHANICAL CONFIGURATION it was measured on, the date, whether it is `measured`/`fitted`/
`derived`, and whether it is `mech_independent` (safe to reuse on a different rotor/load). A test
asserts PROVENANCE covers exactly the exported parameter set, so a number cannot be added without one.

THE TWO MECHANICAL CONFIGURATIONS ARE NOT INTERCHANGEABLE:
  * `prop_350kv_20260726` — F2838 350KV + propeller, 11.25 V. Source of `J_PROP`, `DRAG_TABLE_PROP`,
    the shipped calibration and the only tracked dynamics dataset (`models/350kv_postfix_sysid_raw.json`).
    Recorded at ESC `sine_cross_up = 34`.
  * `bare_shaft_350kv_20260730` — same motor, propeller REMOVED, 11.2 V. Source of every 2026-07-30
    encoder-truth fixture (sinesweep / coastdown / cutmode / steptrace). Recorded at `sine_cross_up = 36`.

WHY THOSE TWO cross_up VALUES (the manifest's numbers are evidenced, not assumed). Two no-load duty
sweeps recorded minutes apart (`reports/duty_sweep_20260729_220535.csv` = A and `..._223122.csv` = B)
are offset by exactly 30 command counts at every point — a pure intercept shift, which is what
`cross_rescale_duty` does. The ESC was read directly and reports `sine_cross_up: 36`; at cmd 600 the
encoder measured 1064.3 and 1069.3 rpm on two runs, and sweep A interpolates to 1072.8 there (0.4-0.8 %)
while sweep B gives 1391.7 (30 % away, far beyond any load effect). So A = 36, B = 34. Interpolating
both to the postfix sysid's three 6-step commands: at cmd 543/612/680 the postfix curve reads
794.4/1510.4/2212.7 rpm, sweep B (34) predicts 800.1/1514.6/2213.6 (+0.7/+0.3/+0.0 %) and sweep A (36)
predicts 465.7/1194.8/1894.6 (-41/-21/-14 %). The postfix dataset is therefore a cross_up=34 recording
even though the velcal commit two hours earlier describes cross_up=36 — reconciled by this project's
recurring gotcha: `esctool.py flash` WIPES the config to firmware defaults and leaves no commit trace.

See `docs/measured-plant.md` for the load-dependence table and the re-measurement recipe.
"""
from __future__ import annotations

import contextlib
import math

# --- the motor itself (nameplate + the one rotor both configurations share) -----------------
KV = 350.0                       # rpm/V
POLE_PAIRS = 7
# SI torque/EMF constant: K_T [N*m/A] == K_E [V*s/rad] == 60/(2*pi*KV) ~= 9.5493/KV. Written as the
# exact 60/(2*pi) rather than the rounded 9.5493 so that K_T*(rpm/KV) and K_T^2*w_rad are the SAME
# number in floating point — that identity is what makes the zero-throttle brake literally the 6-step
# drive law with the duty removed, and a 3.6e-6 rounding gap would make the test assert an approximation.
K_T = 60.0 / (2.0 * math.pi) / KV        # 0.0272837 N*m/A
KV_RAD = KV * 2.0 * math.pi / 60.0        # rad/s per volt = 36.65

V_SUPPLY_PROP = 11.25            # V, the 2026-07-26 prop-on bench supply
V_SUPPLY_BARE = 11.2             # V, the 2026-07-30 bare-shaft bench supply
EFF_350KV = 1.0                  # KV*V*eff*duty6 fits both curves to <=2.1 % with eff = 1

# --- winding resistance: 34 % of it is WIRING, so it is per-installation and it corrodes ----
R_TOTAL = 1.20                   # ohm, winding 0.79 + connectors/leads 0.41
R_TOTAL_TOL = 0.26               # ohm, +- band; propagated into L and J by plantid.py

# --- inertia. ONLY the prop-on number was measured; the bare-shaft one is IDENTIFIED. -------
J_PROP = 8.8e-6                  # kg*m^2, coast-down against the measured drag table
# Identified from the FREE-COAST window — the phase after the drive is removed and before the brake
# bites — against the same measured drag table: J = tau_drag(w_mid)/alpha_free. NOT from the brake, and
# NOT from the brake-peak magnitude (an earlier draft claimed that; nothing in the brake fit's cost
# function constrained the brake torque scale at all, so the claim was false).
#   bare: tau_drag(2757 rpm) = 4.198 mN*m / (5550 rpm/s) => 7.222e-6
# Cross-check on the configuration where J WAS measured: the same arithmetic on the prop-on decays
# (tau_drag(1877 rpm) = 4.576 mN*m / 4860 rpm/s) gives 8.99e-6 against the measured 8.8e-6, +2.2 %.
J_BARE_FITTED = 7.222e-6         # kg*m^2

# --- the one genuinely new electrical parameter: phase inductance --------------------------
# NOT measured. Fitted from where the zero-throttle brake's deceleration PEAKS: a short-circuit
# (asynchronous) brake torque K_T^2*w*R/(R^2+(pp*w*L)^2) peaks at w = R/(pp*L), so the peak LOCATION
# pins L. It is fitted JOINTLY with BRAKE_CUTOUT_RPM against the BRAKE PHASE of the four `disarm`
# decays — revolutions travelled and duration, both measured FROM ENGAGEMENT — with J pinned by the
# free-coast window. An earlier draft fitted it against time-to-stop measured from t = 0, which is 80 %
# fixed delay: that metric passed at 2x torque error, so the earlier L (0.682 mH) was not identified by
# anything. This is a PREDICTION: check it with an LCR meter across two phases (expect ~2L
# line-to-line, i.e. ~2.7 mH at 1 kHz) before trusting it anywhere else.
# CAVEAT: the pointwise deceleration residual (measured decel minus the drag table, per window) puts
# the brake peak nearer 60 mN*m at ~1900 rpm, where this L puts it at 40 mN*m at 1226 rpm. The
# trajectory fit uses the integral and is far less noisy than single-window residuals, so it wins — but
# the two disagree by ~1.5x and that is the honest uncertainty on the brake torque scale.
L_PHASE_FITTED = 1.335e-3        # H

# --- drag ---------------------------------------------------------------------------------
# tau_drag(rpm) measured 2026-07-29 (duty_sweep + PSU current, tau = K_T*I_dc) WITH the propeller.
# HONEST WARNING: this table FALLS with speed (5.90 -> 4.16 mN*m over 753 -> 2922 rpm). No drag law
# permits that — aero goes as w^2, viscous as w, Coulomb is flat; all are flat or rising. So it is
# NOT prop drag: it is a lumped bearing/iron loss plus a K_T*I_dc measurement bias (the idle current
# subtraction leaves a speed-independent offset that reads as falling torque once divided out).
# Confirmed independently here: the BARE-shaft free-coast deceleration (4.86-5.55 krpm/s) is within
# 15 % of the prop-on one, so the propeller contributes almost none of this torque in air.
# CONSEQUENCE: the w^2 coefficient that water needs is NOT resolved by any present measurement.
# `PlantParams.drag` is a callable precisely so water can replace it without touching anything else.
DRAG_TABLE_PROP = [(753, 5.90e-3), (1073, 5.21e-3), (1378, 4.89e-3), (1685, 4.68e-3),
                   (2000, 4.51e-3), (2310, 4.35e-3), (2616, 4.23e-3), (2922, 4.16e-3)]

# Free-coast deceleration in the window between the drive being removed and the ESC's brake biting —
# a two-segment (flat, then linear) least-squares fit of encoder position per decay.
FREE_DECEL_BARE_RPM_S = 5550.0   # mean of 6 bare decays (5042..6113)
FREE_DECEL_PROP_RPM_S = 4860.0   # mean of 3 prop-on decays (4592..5046)
# Bare-shaft drag as a VISCOUS coefficient: B = J_BARE*(FREE_DECEL@2700rpm)/w. Viscous rather than
# Coulomb because the forced-sine regime holds the rotor at 96..214 rpm on a tiny sine amplitude,
# which a 4.3 mN*m Coulomb torque could not permit. Derived, not independently measured.

# --- the ESC's zero-throttle brake: two measured delays per cut mode -----------------------
# The rotor NEVER freely coasts. But the brake does not bite when the command goes away either:
#   1. CUT_RELEASE_S  — the command is gone, the ESC is still driving. |w| holds flat this long.
#   2. BRAKE_ENGAGE_S — drive removed, rotor free (drag only). THEN the brake bites, hard.
# THIS RESOLVES THE "PEAK MOVES WITH LOAD" QUESTION. The brake bites at a fixed TIME after the cut,
# not at a speed: bare shaft is at ~2390 rpm by then and prop-on at ~1540 rpm, which is exactly why
# the deceleration peak was measured at 1455-1596 rpm bare and 865-973 rpm prop-on. The peak LOCATION
# never moved; the speed the rotor happened to have reached did. (Still not proof the brake TORQUE is
# load-independent — see docs/measured-plant.md.)
# BOTH DELAYS ARE MEASURED IN BOTH CONFIGURATIONS AND AGREE, which is why they are tagged
# mech_independent: the three prop-on `disarm` decays give release 0.163/0.167/0.166 s and engage
# 0.137/0.128/0.128 s against the bare shaft's 0.170 / 0.137 s (within 5 %). They are ESC state
# machine timings, so a rotor change should not move them — and here it demonstrably did not.
CUT_RELEASE_S = {"disarm": 0.170, "deadman": 0.332, "zero_held": 0.024}
BRAKE_ENGAGE_S = {"disarm": 0.137, "deadman": 0.113, "zero_held": 0.072}
# THE MODE DEPENDENCE IS NOT ONLY IN THE DELAY — IT IS IN THE TORQUE. Duration of the BRAKE PHASE
# itself (engagement -> |rpm| < 40), all three from the same ~2400 rpm, plus the revolutions travelled:
# the ordering is systematic and spans ~3x, so one short-circuit torque cannot be all three. Carried as
# DATA, not as a model parameter: the model fits `disarm` (the default, and the mode the prop-on decays
# used) and is simply wrong on the other two by that factor. See docs/measured-plant.md.
BRAKE_PHASE_S = {"disarm": 0.107, "deadman": 0.062, "zero_held": 0.033}
BRAKE_PHASE_REV = {"disarm": 1.18, "deadman": 0.51, "zero_held": 0.62}
# The speed below which the brake stops braking. MEASURED, not assumed: subtracting the drag table's
# prediction from the measured deceleration leaves a brake contribution of ~60 krpm/s at 1900 rpm,
# ~10 krpm/s at 600, ~4 krpm/s at 450 and ZERO within noise below ~250 — in all four `disarm` decays.
# The short-circuit law's own low-speed branch falls only as w and would still be ~36 krpm/s at
# 450 rpm, 7x too much. A short circuit the ESC has stopped COMMUTATING is stationary in the rotor
# frame, so its torque averages out over a revolution; that is a cut-out, not a decay.
BRAKE_CUTOUT_RPM = 705.0

# --- forced sine (the regime telemetry cannot see) -----------------------------------------
# Peak torque the forced-sine drive can pull with before the rotor falls out of step. Fitted from the
# open-loop down-step 600 -> 480 (1048 -> 175 rpm, t_half 24.1-26.4 ms over 3 runs, repeatable to 1 ms):
# a constant pull-out torque gives a CONSTANT deceleration, which is what that trace shows.
# BOUNDED ABOVE AND BELOW, which is why it is credible rather than merely fitted:
#   * it must exceed the drag it has to hold against, max 5.90 mN*m over the coherent band cmd 240..510
#     — otherwise the measured synchronous lock could not exist at all;
#   * it must sit below the sine STALL torque K_T*V*(sine_amp_max/255)/R ~= 44.9 mN*m.
# It lands at 7.6, i.e. 1.3x the drag and 17 % of the stall bound. NOTE it was 16.0 mN*m before the
# 2026-07-30 drag double-count fix: with drag counted twice the sine branch needed twice the torque to
# produce the same trace, which is a good illustration of how a downstream fit absorbs an upstream bug.
SINE_PULLOUT_NM = 7.577e-3      # N*m
# Below this command the rotor does not rotate coherently — it cogs, stalls and REVERSES, so a mean
# speed is not a speed. The plant declines to be a speed source there rather than lying.
SINE_COHERENT_CMD_MIN = 210
# Peak-to-peak encoder dispersion while HELD at one command, worst of the up and down legs. This is
# the cogging floor, not measurement noise: 92..234 rpm on a rotor whose mean is 30..190 rpm.
SINE_SPREAD_TABLE_BARE = {60: 220.2, 90: 153.3, 120: 219.6, 150: 211.0, 180: 201.5, 210: 166.9,
                          240: 173.0, 270: 234.0, 300: 183.8, 330: 232.5, 360: 130.3, 390: 148.5,
                          420: 145.4, 450: 164.4, 480: 157.4, 510: 125.4}
# Direction reversals counted inside one 3 s dwell (worst leg). Carried as DATA, not smoothed away.
SINE_REVERSALS_BARE = {60: 3, 90: 7, 120: 6, 150: 2, 180: 3, 210: 2, 240: 0, 270: 2, 300: 0,
                       330: 2, 360: 0, 390: 0, 420: 0, 450: 0, 480: 0, 510: 0}

# --- the crossover UP handoff is slew-limited, and only in the up direction ----------------
# The prop-on step 407 -> 680 sits at 360-415 rpm for ~0.26 s and only THEN jumps to 2200 in ~40 ms.
# That is not the old sim's 0.28 s dead time and not a lag: the ESC ramps its 6-step
# duty demand at a finite rate, holds a low duty CLAMP until the ramp catches up with the command, and
# then releases — after which the rotor's own 14 ms electromechanical time constant does the rest.
#   DUTY_SLEW_UP_PER_S = 0.643 (the ELECTRICAL duty at cmd 680) / 0.40 s (measured time to the jump).
#   DUTY_CROSSOVER_CLAMP = the duty whose 6-step torque BALANCES the prop-on drag at the measured
#     390 rpm plateau. Reading it off as 390/(KV*V) = 0.099 would be wrong: the plateau is drag-limited,
#     not no-load, and that 20 % error moves the modelled plateau to 298 rpm.
# The clamp is almost certainly BlueJay's `startup_power_max` (18) limiting the duty until the ESC has
# confirmed BEMF lock.
# WHAT IS AND IS NOT PINNED. Each is pinned by one feature of that single trace — the plateau LEVEL
# pins the clamp, its DURATION pins the slew — so they ARE separable from each other (an earlier draft
# said otherwise). What is missing is corroboration at any OTHER command: only cmd 680 was measured.
# Because `applied_duty` returns min(slew, clamp), the ramp is visible for only ~80 ms and the clamp
# holds the remaining ~320 ms, so the plateau ends at electrical_duty(cmd)/slew — i.e. LATER for a
# LARGER command. That command scaling is an UNTESTED PREDICTION.
# NEITHER IS LOAD-INDEPENDENT. The clamp was obtained by inverting the prop drag table at the measured
# 390 rpm plateau, and the slew is `electrical_duty(680)/0.40 s`, which also goes through that table.
# Both are therefore functions of the inventory's least trustworthy entry and are tagged OPEN.
DUTY_SLEW_UP_PER_S = 1.6085      # duty/s (ELECTRICAL duty)
DUTY_CROSSOVER_CLAMP = 0.1244    # ELECTRICAL duty (dimensionless)

# --- telemetry: the sensor lies, identically in both configurations ------------------------
# `tele` never reports below 182 mech rpm (= 1274 eRPM at PP=7). Present at EVERY command 60..450 in
# the bare-shaft sinesweep AND at cmd 65..407 in the prop-on sysid curve, so it is a property of the
# ESC/DShot path, not of the rotor. MECHANISM UNRESOLVED: either a bidir-DShot eRPM encoding floor or
# the ESC reporting its forced-commutation INTENTION. Distinguishable only with a different-PP motor.
TELE_FLOOR_MECH = 182.0          # mech rpm
TELE_LIVE_S = 0.109              # s after arming before any telemetry frame arrives at all (reads 0)

# --- reporting-only numbers (explicitly NOT transferable) ----------------------------------
TORQUE_PER_CMD = 0.68e-3         # N*m per command count; 34 % of R_TOTAL is wiring, so this is
                                 # per-installation. Carried for reporting; never for control.
STARTUP_OVERSHOOT_PCT = 53.5     # % over the 1200 rpm target, bare shaft, closed loop (3/3: 53/53/54)
STARTUP_T_PEAK_S = 1.19          # s to the overshoot peak (3/3: 1.19/1.18/1.21)
STARTUP_ABORT_TARGET_RPM = 2400  # rpm target at which the velocity controller ABORT_STALLs bare-shaft


def drag_at(rpm):
    """Linear interpolation on the measured prop-on drag table; clamped outside it."""
    a = abs(rpm)
    if a <= DRAG_TABLE_PROP[0][0]:
        return DRAG_TABLE_PROP[0][1]
    if a >= DRAG_TABLE_PROP[-1][0]:
        return DRAG_TABLE_PROP[-1][1]
    for i in range(1, len(DRAG_TABLE_PROP)):
        x0, y0 = DRAG_TABLE_PROP[i - 1]
        x1, y1 = DRAG_TABLE_PROP[i]
        if a <= x1:
            return y0 + (y1 - y0) * (a - x0) / (x1 - x0)
    return DRAG_TABLE_PROP[-1][1]


def brake_peak_rpm(r_total=R_TOTAL, l_phase=L_PHASE_FITTED):
    """Rotor speed the short-circuit brake torque peaks at: w = R/(pp*L). L was FITTED to put this
    at the measured 1800 rpm, so this is a restatement of L, not an independent number."""
    return r_total / (POLE_PAIRS * l_phase) * 60.0 / (2.0 * math.pi)


def brake_peak_torque(r_total=R_TOTAL, l_phase=L_PHASE_FITTED):
    """Peak short-circuit brake torque [N*m] = K_T^2*w_peak/(2*R)."""
    return K_T * K_T * brake_peak_rpm(r_total, l_phase) * 2.0 * math.pi / 60.0 / (2.0 * r_total)


def mech_time_constant_ms(j, r_total=R_TOTAL):
    """J*R/K_T^2 in ms — the rotor's own electromechanical time constant."""
    return j * r_total / (K_T * K_T) * 1e3


# =====================================================================================
# provenance registry
# =====================================================================================
PROP = "prop_350kv_20260726"
BARE = "bare_shaft_350kv_20260730"

_SINESWEEP = "tests/data/sinesweep_noprop_20260730_033014.csv"
_FOLD = "tests/data/sinesweep_fold_noprop_20260730_033417.csv"
_COAST_BARE = "tests/data/coastdown_noprop_20260730_032130.csv"
_CUTMODE = "tests/data/cutmode_noprop_20260730_032448.csv"
_COAST_PROP = "tests/data/coastdown_20260730_011123.csv"
_DOWN = "tests/data/steptrace_down_noprop_20260730_033706.csv"
_STARTUP = "tests/data/steptrace_startup_noprop_20260730_033906.csv"
_STARTUP2400 = "tests/data/steptrace_startup2400_noprop_20260730_034108.csv"
_SYSID = "models/350kv_postfix_sysid_raw.json"


def _p(units, tool, source, mech_config, date, kind, mech_independent, note):
    return {"units": units, "tool": tool, "source": source, "mech_config": mech_config,
            "date": date, "kind": kind, "mech_independent": mech_independent, "note": note}


PROVENANCE = {
    "KV": _p("rpm/V", "nameplate + rpm_sweep", "profiles/f2838_350kv_bench_20260726.yaml",
             PROP, "2026-07-26", "measured", True,
             "same motor in both configurations, so KV cannot be load-dependent"),
    "POLE_PAIRS": _p("-", "nameplate", "profiles/f2838_350kv_bench_20260726.yaml", PROP,
                     "2026-07-26", "measured", True, "counted; sets the forced-sine mech rate"),
    "K_T": _p("N*m/A", "identity 9.5493/KV", "-", PROP, "2026-07-26", "derived", True,
              "K_T == K_E in SI; the brake law's only torque scale"),
    "KV_RAD": _p("rad/s/V", "identity KV*2*pi/60", "-", PROP, "2026-07-26", "derived", True,
                 "back-EMF constant in rad/s per volt"),
    "V_SUPPLY_PROP": _p("V", "vbatt/obschar", "models/obschar_f2838_350kv.json", PROP,
                        "2026-07-26", "measured", False,
                        "bench supply on the day; a parameter, never a state (no sag model)"),
    "V_SUPPLY_BARE": _p("V", "duty_sweep + PSU readout", "reports/duty_sweep_20260729_223122.csv",
                        BARE, "2026-07-30", "measured", False, "bare-shaft bench supply"),
    "EFF_350KV": _p("-", "duty_sweep / sysid curve fit", _SYSID, PROP, "2026-07-26", "measured",
                    True, "1.0 fits both the prop-on and bare-shaft static curves to <=2.1 %"),
    "R_TOTAL": _p("ohm", "multimeter (winding + leads)", "reports/loadtest_20260730_021330.csv",
                  PROP, "2026-07-30", "measured", True,
                  "0.79 winding + 0.41 connectors; ELECTRICAL, so it survives a load change, but "
                  "34 % of it is per-installation wiring and it corrodes"),
    "R_TOTAL_TOL": _p("ohm", "multimeter repeats", "-", PROP, "2026-07-30", "measured", True,
                      "+-band; L scales with R and J scales with 1/R, both propagated by plantid"),
    "J_PROP": _p("kg*m^2", "coastdown.py (J = tau_drag/(-dw/dt))", _COAST_PROP, PROP,
                 "2026-07-30", "measured", False,
                 "the ONLY measured inertia. Biased low: tau_drag was measured while DRIVEN"),
    "J_BARE_FITTED": _p("kg*m^2", "plantid.py fit (free-coast window vs the drag table)", _COAST_BARE,
                        BARE, "2026-07-30", "fitted", False,
                        "NOT measured. Identified from the FREE-COAST window (the only phase with no "
                        "ESC torque) as tau_drag(w_mid)/alpha_free — NOT from the brake. The same "
                        "arithmetic reproduces the measured J_PROP to +2.2 %"),
    "L_PHASE_FITTED": _p("H", "plantid.py fit (brake-phase revolutions + duration)", _COAST_BARE,
                         BARE, "2026-07-30", "fitted", None,
                         "OPEN whether it transfers: electrical in principle, but only ever "
                         "identified on the bare shaft, and only against the `disarm` cut mode. "
                         "Verify with an LCR meter before reuse. The brake TORQUE SCALE it sets is "
                         "in-sample only — see BRAKE_PHASE_S and docs/measured-plant.md"),
    "DRAG_TABLE_PROP": _p("(rpm, N*m)", "duty_sweep + PSU current", "reports/duty_sweep_20260729_220535.csv",
                          PROP, "2026-07-29", "measured", False,
                          "FALLS with speed, which no drag law permits => lumped bearing/iron loss "
                          "plus a K_T*I_dc bias, NOT prop drag. Water's w^2 coeff is UNMEASURED. Also "
                          "the `duty6` REFERENCE drag (see plant.duty_ref_offset) and the only air "
                          "drag law: it fits BOTH free-coast decelerations to <=2.2 %"),
    "FREE_DECEL_BARE_RPM_S": _p("rpm/s", "plantid.py two-segment fit", _COAST_BARE, BARE,
                                "2026-07-30", "measured", False,
                                "deceleration between drive removal and the brake biting"),
    "FREE_DECEL_PROP_RPM_S": _p("rpm/s", "plantid.py two-segment fit", _COAST_PROP, PROP,
                                "2026-07-30", "measured", False,
                                "within 15 % of the bare-shaft value => the propeller adds almost "
                                "no torque in air, so DRAG_TABLE_PROP is not prop drag"),
    "CUT_RELEASE_S": _p("s", "coastdown.py coast_start()", _CUTMODE + " + " + _COAST_PROP, BARE,
                        "2026-07-30", "measured", True,
                        "per cut mode: how long the ESC keeps driving after the command is gone. The "
                        "mech_independent tag is EVIDENCED FOR `disarm` ONLY (bare 0.170 vs prop-on "
                        "0.163-0.167) — `deadman` and `zero_held` were only ever recorded bare-shaft, "
                        "so for those two keys it is an assumption"),
    "BRAKE_ENGAGE_S": _p("s", "plantid.py brake-onset detector", _CUTMODE + " + " + _COAST_PROP,
                         BARE, "2026-07-30", "measured", True,
                         "per cut mode: drive-removed -> brake bites. Same caveat as CUT_RELEASE_S: "
                         "cross-configuration agreement is evidenced for `disarm` only (bare 0.137 vs "
                         "prop-on 0.128-0.137). brake_on_stop=0 does NOT disable any of this"),
    "BRAKE_PHASE_S": _p("s", "plantid.py (engagement -> |rpm| < 40)", _CUTMODE, BARE, "2026-07-30",
                        "measured", False,
                        "per cut mode, from the SAME ~2400 rpm: 0.118 / 0.062 / 0.033 s. A ~3x "
                        "systematic ordering in the brake TORQUE that one short-circuit law cannot "
                        "reproduce. Carried as data; the model fits `disarm` only"),
    "BRAKE_CUTOUT_RPM": _p("rpm", "plantid.py fit (brake-phase revolutions + duration)", _CUTMODE,
                           BARE, "2026-07-30", "fitted", None,
                           "the brake stops braking below this. Visible directly in the residual "
                           "(measured decel minus the drag table) but its exact value is fitted "
                           "jointly with L. Fitted against `disarm` only; OPEN whether an ESC state "
                           "transition like this moves with the rotor"),
    "BRAKE_PHASE_REV": _p("rev", "plantid.py (encoder position, engagement -> stop)", _CUTMODE, BARE,
                          "2026-07-30", "measured", False,
                          "revolutions travelled during the brake phase. The metric that MATTERS for "
                          "the brake torque: time-to-stop measured from t=0 is 80 % fixed delay and "
                          "passes at 2x torque error"),
    "SINE_PULLOUT_NM": _p("N*m", "plantid.py fit on the open-loop down-step", _DOWN, BARE,
                          "2026-07-30", "fitted", False,
                          "forced-sine pull-out torque; set by the sine AMPLITUDE, so the torque is "
                          "fixed but the load it must beat is not. Bracketed by the drag it holds "
                          "against (5.90 mN*m) and the sine stall torque (44.9 mN*m)"),
    "SINE_COHERENT_CMD_MIN": _p("cmd", "sinesweep.py (reversal counts)", _SINESWEEP, BARE,
                                "2026-07-30", "measured", False,
                                "below this the rotor reverses inside one dwell; a mean is not a speed"),
    "SINE_SPREAD_TABLE_BARE": _p("(cmd, rpm)", "sinesweep.py", _SINESWEEP, BARE, "2026-07-30",
                                 "measured", False,
                                 "cogging dispersion, worst leg. Shrinks with damping (water)"),
    "SINE_REVERSALS_BARE": _p("(cmd, count)", "sinesweep.py", _SINESWEEP, BARE, "2026-07-30",
                              "measured", False, "reversals per 3 s dwell, worst leg"),
    "DUTY_SLEW_UP_PER_S": _p("duty/s", "plantid.py fit on the sysid 407->680 step", _SYSID, PROP,
                             "2026-07-26", "fitted", None,
                             "firmware crossover ramp rate, up direction only. Pinned by the plateau "
                             "DURATION of a single trace at cmd 680 and corroborated at no other "
                             "command; goes through the prop drag table via electrical_duty, so "
                             "load-independence is OPEN"),
    "DUTY_CROSSOVER_CLAMP": _p("duty", "plantid.py fit on the sysid 407->680 step", _SYSID, PROP,
                               "2026-07-26", "fitted", None,
                               "duty ceiling held during the crossover ramp; pinned by the plateau "
                               "LEVEL. Obtained by INVERTING THE PROP DRAG TABLE at the measured "
                               "390 rpm plateau, so it is a function of the inventory's least "
                               "trustworthy entry: load-independence is OPEN, not True"),
    "TELE_FLOOR_MECH": _p("mech rpm", "sinesweep.py + sysid curve", _SINESWEEP, BARE, "2026-07-30",
                          "measured", True,
                          "identical (182) in BOTH configurations and at every cmd 60..450, so it "
                          "belongs to the ESC/DShot path. Mechanism unresolved"),
    "TELE_LIVE_S": _p("s", "steptrace.py --startup", _STARTUP, BARE, "2026-07-30", "measured", True,
                      "telemetry reads exactly 0 until the first frame arrives. Bracketed by the 3 "
                      "startup runs to (0.108, 0.109) s; the ~9 ms poll period is the resolution"),
    "TORQUE_PER_CMD": _p("N*m/cmd", "loadtest.py", "reports/loadtest_20260730_021330.csv", PROP,
                         "2026-07-30", "measured", False,
                         "REPORTING ONLY. Absolute torque is not modelled: R_TOTAL is 34 % wiring"),
    "STARTUP_OVERSHOOT_PCT": _p("%", "steptrace.py --mode rpm --startup", _STARTUP, BARE,
                                "2026-07-30", "measured", False,
                                "closed loop, so it belongs to the FIRMWARE controller + this rotor"),
    "STARTUP_T_PEAK_S": _p("s", "steptrace.py --mode rpm --startup", _STARTUP, BARE, "2026-07-30",
                           "measured", False, "time to the overshoot peak"),
    "STARTUP_ABORT_TARGET_RPM": _p("rpm", "steptrace.py --mode rpm --startup", _STARTUP2400, BARE,
                                   "2026-07-30", "measured", False,
                                   "rpm 1200 starts, rpm 2400 ABORT_STALLs: gated on the plant's "
                                   "ability to traverse the crossover gap, so strongly load-dependent"),
}

# Every exported parameter, in the order the provenance table prints. Adding a number without adding
# it here (and to PROVENANCE) fails test_measured_plant.py.
PARAMS = tuple(PROVENANCE)

# Module globals that are deliberately NOT plant parameters (config keys, helpers, the borrow maps).
_NON_PARAMS = frozenset({"PROVENANCE", "PARAMS", "PRESET_BORROWED", "PROP", "BARE"})


def parameter_names():
    """The UPPER_CASE module globals that are plant parameters (i.e. need provenance)."""
    return tuple(n for n, v in sorted(globals().items())
                 if n.isupper() and not n.startswith("_") and n not in _NON_PARAMS)


# --- the RECORDING accessor -----------------------------------------------------------------
# Which parameters a preset draws on used to be a hand-maintained parallel list, with nothing tying it
# to what `plant.preset()` actually read. The list drifted immediately: `SINE_COHERENT_CMD_MIN` (bare
# shaft, mech_independent=False) was being read by the PROP preset while appearing in neither its
# `shared` nor its `borrowed` entry, so the guard reported "no undeclared cross-config mixing" while it
# was happening. Now `preset()` pulls every value through `value()` inside `recording()`, and the set is
# OBSERVED from the code rather than declared beside it.
_PRESET_READS: dict = {}
_recording = [None]


@contextlib.contextmanager
def recording(mech_config):
    """Record every `value()` read while building the preset for `mech_config`."""
    prev = _recording[0]
    _recording[0] = mech_config
    _PRESET_READS[mech_config] = set()
    try:
        yield
    finally:
        _recording[0] = prev


def value(name):
    """Read a parameter, recording the read if a preset is being built."""
    v = globals()[name]
    if _recording[0] is not None:
        _PRESET_READS[_recording[0]].add(name)
    return v


def preset_param_names(mech_config):
    """Every PROVENANCE key the preset for `mech_config` actually READ, observed from the code.

    Raises until `pico_esc.plant.preset(mech_config)` has been called at least once — deliberately,
    because a value returned without the code having run would be exactly the parallel list this
    replaces."""
    if mech_config not in _PRESET_READS:
        raise RuntimeError(f"call pico_esc.plant.preset({mech_config!r}) first — this set is RECORDED "
                           f"from the code, not declared")
    return tuple(sorted(_PRESET_READS[mech_config]))


# Parameters each preset KNOWINGLY takes from the other mechanical configuration, with the reason.
# Anything a preset reads whose `mech_config` differs and which is not `mech_independent=True` must
# appear here, or test_no_preset_mixes_mechanical_configurations_undeclared fails.
PRESET_BORROWED = {
    PROP: {
        "L_PHASE_FITTED": "there is only one L and it was only ever identified bare-shaft; "
                          "mech-independence is OPEN. Replaying the prop-on decays with it is the "
                          "out-of-sample test, so the borrow is the POINT here",
        "SINE_PULLOUT_NM": "no prop-on down-step was ever recorded, so the forced-sine pull-out torque "
                           "is the bare-shaft one. The pull-out TORQUE is set by the sine amplitude "
                           "(electrical) but the load it must beat is not, so any prop-on forced-sine "
                           "prediction from this preset is unvalidated",
        "SINE_COHERENT_CMD_MIN": "the incoherent-rotation band was only measured bare-shaft, and the "
                                 "module's own note says it shrinks with damping — so PROP's "
                                 "`coherent()`/`steady()` below cmd 210 is bare-shaft data. It is "
                                 "carried so the prop preset refuses to invent a speed there, not "
                                 "because 210 is the prop-on number",
        "SINE_SPREAD_TABLE_BARE": "cogging dispersion, bare-shaft only. Reported as data by the prop "
                                  "preset; a propeller's damping would shrink it",
        "SINE_REVERSALS_BARE": "reversal counts, bare-shaft only. Same caveat as the spread table",
        "BRAKE_CUTOUT_RPM": "the speed the brake stops braking at was only identified on the "
                            "bare-shaft `disarm` decays. It should be an ESC state transition (loss "
                            "of BEMF zero-crossing detection) and therefore rotor-independent, but "
                            "the prop-on decays were held out to test the brake rather than to fit "
                            "it, so nothing here establishes that",
    },
    BARE: {
        "DRAG_TABLE_PROP": "the only measured drag torque there is, and the drag `duty6` was "
                           "CALIBRATED against, so it is both `drag` and `drag_ref` here. Justified by "
                           "three independent lines (both free-coast decelerations, the bare static "
                           "curve matching a prop-calibrated duty6 to <=1 %, and the table's own "
                           "unphysical fall) — but it is still a prop-on measurement",
        "DUTY_SLEW_UP_PER_S": "the crossover ramp was only ever measured on the prop-on 407->680 step, "
                              "and it goes through the prop drag table via electrical_duty. It sets "
                              "the plateau in the sim_obs section-(c) startup study, which runs on "
                              "THIS preset, so that plateau is a borrowed number",
        "DUTY_CROSSOVER_CLAMP": "same trace, and obtained by inverting the prop drag table at the "
                                "measured 390 rpm plateau. A bare-shaft startup fixture should show a "
                                "clamp-limited plateau too; re-deriving it there is the follow-up",
    },
}


def preset_borrowed(mech_config):
    """{parameter: why} for parameters this preset knowingly takes from the OTHER configuration."""
    return dict(PRESET_BORROWED[mech_config])


def provenance_table():
    """Human-readable inventory: one line per parameter, mechanical configuration included."""
    rows = [f"{'parameter':24s} {'value':>14s} {'units':<12s} {'kind':<9s} "
            f"{'mech_config':<26s} {'indep':<6s} source"]
    rows.append("-" * 132)
    for name in PARAMS:
        p = PROVENANCE[name]
        v = value(name)
        if isinstance(v, (list, dict)):
            vs = f"<{len(v)} entries>"
        elif isinstance(v, float) and (abs(v) < 1e-3 or abs(v) >= 1e5):
            vs = f"{v:.4g}"
        else:
            vs = f"{v:g}" if isinstance(v, (int, float)) else str(v)
        ind = {True: "yes", False: "no", None: "OPEN"}[p["mech_independent"]]
        rows.append(f"{name:24s} {vs:>14s} {p['units']:<12s} {p['kind']:<9s} "
                    f"{p['mech_config']:<26s} {ind:<6s} {p['tool']} <- {p['source']}")
    return "\n".join(rows)
