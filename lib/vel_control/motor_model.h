// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// motor_model — parametric no-load motor model for the Pico: MotorModel(KV, pole_pairs, V).
// -----------------------------------------------------------------------------------------
// A faithful C++ port of host/pico_esc/motor_model.py. Instead of hand-measuring a per-motor
// SpeedProfile, PREDICT the feed-forward thrust->rpm curve + regime boundaries from KV + pole-pairs +
// supply V, using a UNIVERSAL firmware/DShot "duty shape" (a firmware property, calibrated once, KV-
// independent). fillProfile() writes the predicted curve into a caller-owned CurvePoint[] which a
// vel::SpeedProfile then references — so `motor <kv> <pp> <v>` over serial reconfigures the whole FF.
//
// Physics (validated on the F2838 350KV, RMSE ~12 rpm from KV alone):
//   * 6-step (high speed) is KV-governed:  rpm(cmd) = KV*V*eff*(A6*cmd + B6)  (bracket = universal shape)
//   * forced-sine (low speed) is PP-governed: mech = (cmd/1000)*SINE_FULLSCALE_ERPM/PP  (KV-independent)
//   * the up-handoff fires on a DUTY, not on a speed: when the 6-step duty demand clears the
//     minimum-modulation floor DUTY6_HANDOFF_FLOOR => handoffCmd() = cmdForDuty6(FLOOR), which is KV-
//     AND PP-independent in COMMAND and KV-scaled in the landing rpm. crossUpErpm() (the CONFIGURED
//     seam) still feeds the ESC config and Thruster's ffcx_, but no handoff quantity comes through it.
//     At a low enough cross_up the firmware REFUSES to hand off at all — see handoffPossible().
// What KV canNOT predict: the dynamics (tau ~ inertia/load) and thus the FB gains — those stay
// load-dependent (tune under load, or the tele-live PI trim in VelocityController closes the residual).
//
// PORTABLE + TESTABLE: no Arduino/PIO headers (stdint + math only), like vel_control.h. Self-cal (eff)
// takes (cmd, tele_erpm) samples — encoder-free. Refresh the universal-shape constants when a 2nd KV
// (e.g. 1000KV) is measured; a per-motor IR-drop residual shows up as `eff` the self-cal fits.
#pragma once
#include <stdint.h>
#include <math.h>
#include "vel_control.h"     // for vel::CurvePoint

namespace vel {

// Universal firmware/DShot shapes (calibrated once; KV-independent). See the Python source + Lake note.
static const float DUTY6_A = 0.00265f;      // 6-step: duty_eff(cmd) = rpm/(KV*V) = A6*cmd + B6
static const float DUTY6_B = -1.233f;

// ---- sine_cross_up dependence of the 6-step duty (measured 2026-07-29, host/duty_sweep.py) ----
// The ESC's `cross_rescale_duty` maps the 6-step demand relative to `Rcp_cross = Cross_Up << 5`, so
// changing Cross_Up SHIFTS the duty by a constant. DUTY6_A/B above were fitted at Cross_Up = 34; at
// any other Cross_Up the pair is wrong by that shift and NOTHING else — two no-load sweeps at 34 and
// 36 gave slopes agreeing to 0.19 % (0.002611 vs 0.002616) and intercepts differing by 0.0843/2 units.
//
// Why this matters beyond the FF: it is an ADDITIVE error, so the multiplicative `vbattScale` (vcal)
// can never cancel it across operating points — which is why `vest` needed re-calibrating whenever the
// operating point moved. Correcting here fixes the FF, the soft-sensor and the DOB's `rpmFor` at once.
//
// Applying it drops the cmd 570..780 error at Cross_Up=36 from 11.9..44.4 % to <= 0.6 %.
static const float DUTY6_CROSSUP_REF = 34.0f;      // the Cross_Up DUTY6_A/B were fitted at
// RE-FITTED 2026-07-31 over a 14-UNIT baseline: 0.0421f -> 0.04134f (-1.8 %). The old value came
// from two no-load duty sweeps at cross_up 34 and 36 -- TWO units, so a 2 % error in either
// intercept became a 2 % error in K, and it did. What is measured is the RATIO
// K/DUTY6_A = command counts per cross_up unit, and three independent routes agree: 15.60
// (duty_sweep pair at 36 and 50, joint fit with a SHARED slope), 15.63 (the measured 6-step
// branches of the 2026-07-31 sine sweeps, 34 <-> 50), 15.59 (the measured handoff commands
// themselves) -- against the old 15.89. Handoff residuals go from -0.79/+0.49/+3.90 counts at
// cross_up 34/36/50 to -0.79/-0.09/-0.69: a TREND becomes a constant half-count.
// The absolute duty scale from that run is NOT trustworthy (no PSU currents), which is why
// DUTY6_A/B stay frozen and only the ratio moved. Full provenance in the Python mirror.
static const float DUTY6_CROSSUP_K   = 0.04134f;   // measured d(duty)/d(Cross_Up); theory
                                                   // (32*2.5/2047) = 0.0391, measured runs 7.9 % higher
static const float SINE_FULLSCALE_ERPM = 2499.0f;   // firmware S1 full-scale eRPM at cmd 1000
static const float HANDOFF_ERPM = 1333.0f;          // lowest handoff the firmware allows (min slip)
// The ESC's `sine_cross_up` BYTE -> the seam it actually hands off at, in eRPM. Cross_Up is in
// Sine_Inc_H units (one unit = 256 in Sine_Inc, and up eRPM = Sine_Inc*10000/65536), so
// eRPM/unit = 256*10000/65536 = 39.0625. MUST stay in lockstep with
// host/pico_esc/config.py:SINE_CROSS_UP_ERPM_PER_UNIT and tools/sim/sine_drive_model.py.
// This exists because cross_up has TWO independent effects and only one of them was modelled: it
// shifts the 6-step duty (DUTY6_CROSSUP_K, above) AND it MOVES THE SEAM. Anything that derives a
// handoff speed/command must go through crossUpErpm(), or it silently keeps using the firmware
// minimum while the operator has configured something else.
static const float SINE_CROSS_UP_ERPM_PER_UNIT = 10000.0f / 256.0f;   // = 39.0625
// The firmware seeds Comm_Period4x from a 4-sector tick window and only fires the up-handoff when that
// window is within these bounds; outside them it REFUSES and stays in forced sine for ever (it does not
// clamp). Lockstep with host/pico_esc/config.py:SINE_CROSS_TICKS_MIN/MAX and Bluejay.asm.
static const int SINE_CROSS_TICKS_MIN = 9;
static const int SINE_CROSS_TICKS_MAX = 30;

// ---- the sine->6-step UP-HANDOFF TRIGGER: a minimum-modulation DUTY floor --------------------------
// Mirrors host/pico_esc/motor_model.py:DUTY6_HANDOFF_FLOOR — same arithmetic, and the full provenance
// (row-selection rule, per-file means, the superseded 526.7 and why it was wrong) lives there. In
// short: measured 2026-07-31 on a 930KV at 11.2 V (host/sinesweep.py 1-count sweeps, AS5600), the ESC
// hands off when its 6-STEP DUTY DEMAND clears a floor, not when the forced field reaches a speed.
// Read off the sweep-DOWN leg, where the rotor stays in 6-step but LOCKS at one speed and ignores the
// command: 523.44 rpm at cross_up 34, 522.56 at 36, 522.17 at 50 — mean 522.53 over all five cited
// fixtures (n=29, sd 0.81).
//   handoffCmd() -> 484.21 / 515.41 / 733.81 at cross_up 34 / 36 / 50, against measured brackets
//   485.0 +- 1.0 / 515.5 +- 0.5 / 734.5 +- 0.5: residuals -0.79 / -0.09 / -0.69 counts — a CONSTANT
//   ~half-count offset, below the 1-count resolution of the sweeps the brackets came from. Until
//   DUTY6_CROSSUP_K was re-fitted on 2026-07-31 they read -0.79 / +0.49 / +3.90, a +0.29 counts-per-
//   unit TREND; that was the signature of a wrong cross_up SLOPE, and re-fitting K flattened it to
//   +0.006. This floor did not move with it — it is read off the cross_up 34 leg where the K term is
//   identically zero, which is why the 34 residual is unchanged.
// A fixed-SPEED floor is refuted: it would put the 350KV at 11.1 V / cross_up 36 at cmd 547.2, against
// that motor's own measured bracket (515, 520] which the duty law's 515.4 lands inside.
// HONEST LIMIT: one motor, one loom, one supply, down-leg lock only; "minimum modulation" is an
// interpretation of a flat command-independent lock, not a modulator measurement.
static const float DUTY6_HANDOFF_FLOOR = 522.53f / (930.0f * 11.2f);  // = 0.050166 effective duty
// The command magnitude the protocol tops out at. Used as a REFUSAL bound, not a clamp: a handoff
// command past it cannot be sent, so that cross_up has no reachable handoff (see handoffPossibleAt).
static const float CMD_FULL_SCALE = 1000.0f;
// "No handoff command exists" — above CMD_FULL_SCALE, so every `cmd >= floor` comparison in the
// sense/FF cascade fails CLOSED on it. vel::SENSE_CMD_UNREACHABLE in sense_gate.h is deliberately THE
// SAME constant (it is defined as this one): a refused handoff and an unconfigured seam are the same
// fact on the wire — "no command can validate this" — and two independent 1001.0f literals in two
// headers would be a trap the first time one of them moved.
// THE INVARIANT THE ALIAS DEPENDS ON, and it is enforced by handoffPossibleAt(), not by luck:
// HANDOFF_CMD_NONE > CMD_FULL_SCALE >= every command handoffCmd() can return. Without the
// CMD_FULL_SCALE half of the refusal that would be FALSE — the duty law crosses 1001 between cross_up
// 67 (999.0) and 68 (1014.6), so a real handoff command would have aliased the sentinel and `>=
// HANDOFF_CMD_NONE` would have been a silently wrong refusal test from cross_up 68 up. (That crossing
// sat at 66/67 before the 2026-07-31 K re-fit: it is a function of DUTY6_CROSSUP_K, never a constant.)
static const float HANDOFF_CMD_NONE = 1001.0f;

// The 6-step duty map's exact inverse, as a free function of (duty, cross_up). MotorModel::cmdForDuty6
// and vel::handoffCmdFor (sense_gate.h) BOTH call this, so the gate's command floor cannot drift from
// the model's — the alternative was sense_gate.h constructing a dummy MotorModel and relying on a
// comment that its KV/pp/V are never read. Mirrors host/pico_esc/motor_model.py:cmd_for_duty6().
inline float cmdForDuty6At(float duty, float crossUp) {
	return (duty - DUTY6_B + DUTY6_CROSSUP_K * (crossUp - DUTY6_CROSSUP_REF)) / DUTY6_A;
}

// Is there ANY reachable command that makes an ESC at this `sine_cross_up` hand off? Two conditions,
// both meaning the same operational thing. Mirrors host/pico_esc/motor_model.py:handoff_possible()
// and host/pico_esc/config.py:up_erpm_handoffable().
//   1. THE FIRMWARE REFUSES. It seeds Comm_Period4x from a 4-sector tick window and only fires the
//      up-handoff for a window inside [SINE_CROSS_TICKS_MIN, SINE_CROSS_TICKS_MAX]; outside it the ESC
//      stays in forced sine for ever (it does not clamp). Measured 2026-07-31: cross_up 30 (1171.9
//      eRPM, a 34-tick window) never handed off on either sweep leg, while 34/36/50 handed off exactly
//      where DUTY6_HANDOFF_FLOOR says. Written in FLOAT rather than as `(int)(40000/up)`: the integer
//      cast is UB for a tiny `up` (setCrossUp(1e-9f) gives 1.02e12, past INT_MAX) and this file must
//      never fail open. Truncation-equivalent by construction — `int(40000/up) <= MAX` iff
//      `40000/up < MAX+1` iff `up > 40000/(MAX+1)`, and likewise on the MIN side. A NaN crossUp makes
//      both comparisons false, i.e. refused.
//   2. THE COMMAND IS UNREACHABLE. cross_up shifts the trigger by DUTY6_CROSSUP_K/DUTY6_A = 15.6
//      counts per unit, so a high enough byte puts the handoff past +-CMD_FULL_SCALE: 999.0 at
//      cross_up 67, 1014.6 at 68, 1513.8 at 100. Nothing can be sent that hands off there. This is also
//      what keeps HANDOFF_CMD_NONE from colliding with a real command (see above). Arithmetic on the
//      protocol's range, NOT a measurement.
inline bool handoffPossibleAt(float crossUp) {
	float up = crossUp * SINE_CROSS_UP_ERPM_PER_UNIT;
	if (!(up > 4.0f * 10000.0f / (float)(SINE_CROSS_TICKS_MAX + 1) &&
	      up <= 4.0f * 10000.0f / (float)SINE_CROSS_TICKS_MIN)) return false;
	float cmd = cmdForDuty6At(DUTY6_HANDOFF_FLOOR, crossUp);
	return cmd > 0.0f && cmd <= CMD_FULL_SCALE;
}

// Highest byte that still fits a 6-step SECTION inside `cmdHi` (= 47 at fillProfile's 700).
//
// A DIFFERENT bound from handoffPossibleAt()'s upper edge (67), and conflating the two has already
// shipped a wrong recovery message: 67 is where the handoff command stops being a command at all,
// this is where the handoff outruns the top of the curve being SAMPLED. Bytes in 48..67 hand off
// perfectly well and still cannot be given a parametric FF curve, so telling an operator whose
// `motor` failed to "use 34..67" walks them straight back into the same failure.
//
// DERIVED, not written as 47, because it moves with DUTY6_CROSSUP_K. The 2026-07-31 re-fit of K
// (0.0421 -> 0.04134) is the proof that mattered: it moved the SIBLING bound 66 -> 67 and left this
// one at 47, so the two are demonstrably not the same number. Solving
// cmdForDuty6At(FLOOR, cu) == cmdHi for cu. Mirrors host/pico_esc/motor_model.py's
// _max_emittable_cross_up(); _min is handoffPossibleAt()'s own lower edge, 34.
inline int maxEmittableCrossUp(float cmdHi) {
	return (int)(DUTY6_CROSSUP_REF
	             + (cmdHi * DUTY6_A - DUTY6_HANDOFF_FLOOR + DUTY6_B) / DUTY6_CROSSUP_K);
}
inline int minHandoffCrossUp() {
	return (int)(4.0f * 10000.0f
	             / ((float)(SINE_CROSS_TICKS_MAX + 1) * SINE_CROSS_UP_ERPM_PER_UNIT)) + 1;
}

class MotorModel {
public:
	MotorModel(float kv, int pole_pairs, float v_supply, float eff = 1.0f)
		: kv_(kv), pp_(pole_pairs), v_(v_supply), eff_(eff) {}

	// live setters (for --track-voltage style updates over serial)
	void set(float kv, int pp, float v) { kv_ = kv; pp_ = pp; v_ = v; eff_ = 1.0f; }
	void setVoltage(float v) { v_ = v; }
	void setEff(float e)     { eff_ = e; }
	// The ESC's `sine_cross_up`. Defaults to the reference, so an operator who never sets it gets
	// exactly the pre-2026-07-29 behaviour rather than a silent shift.
	void  setCrossUp(float cu) { if (cu > 0.0f) crossUp_ = cu; }
	float crossUp() const      { return crossUp_; }
	// The SEAM the configured cross_up puts the handoff at, in eRPM — floored at the firmware's own
	// minimum (HANDOFF_ERPM), because the ESC cannot hand off below it however low the byte is set
	// (cross_up 34 => 1328 eRPM, i.e. just under the floor: that is why 34 reads as ~the minimum).
	// This is the ONE place a seam-derived quantity may come from on the parametric path.
	float crossUpErpm() const {
		float e = crossUp_ * SINE_CROSS_UP_ERPM_PER_UNIT;
		return e > HANDOFF_ERPM ? e : HANDOFF_ERPM;
	}

	// THE 6-step duty map and its exact inverse. Every duty user goes through this pair — four call
	// sites open-coded `DUTY6_A*cmd + DUTY6_B` before, which is how the Cross_Up term would get
	// forgotten in one of them.
	float duty6(float cmd) const {
		return DUTY6_A * fabsf(cmd) + DUTY6_B - DUTY6_CROSSUP_K * (crossUp_ - DUTY6_CROSSUP_REF);
	}
	float cmdForDuty6(float duty) const { return cmdForDuty6At(duty, crossUp_); }

	// SOFT-SENSOR (ESC perception): infer effective supply V from the live command + measured 6-step
	// mech rpm, no voltage sensor. V_eff = rpm/(KV*duty(cmd)) = V_supply - I*R/duty. At no/light load
	// V_eff ~= V_supply; the DEFICIT (V_supply - V_eff) is proportional to current => torque => thrust
	// (a relative load/thrust signal without a force or current sensor). Returns 0 if not in 6-step.
	float estimateV(float cmd, float rpm) const {
		float d = duty6(cmd);
		return (d > 0.02f && kv_ > 0.0f && fabsf(rpm) > 100.0f) ? fabsf(rpm) / (kv_ * d) : 0.0f;
	}
	// Relative load signal = (V_supply_ref - V_eff): >0 and growing = more load (IR drop). 0 if not 6-step.
	float estimateLoad(float cmd, float rpm) const {
		float ve = estimateV(cmd, rpm);
		return ve > 0.0f ? (v_ - ve) : 0.0f;
	}
	float kv() const         { return kv_; }
	float voltage() const    { return v_; }
	float eff() const        { return eff_; }
	int   polePairs() const  { return pp_; }

	// ---- scalar predictions ----
	float noLoadTop()         const { return kv_ * v_ * eff_; }
	float sineFullscaleMech() const { return SINE_FULLSCALE_ERPM / (float)pp_; }   // PP-governed
	// See handoffPossibleAt() above for both conditions and their evidence. Keyed on the RAW byte,
	// never on crossUpErpm(), which FLOORS a low threshold up to the firmware minimum — the exact
	// mistake this stops. Truncate-vs-round is not discriminated by the data and is not claimed:
	// 33 -> 31.03 ticks refuses and 34 -> 30.12 accepts either way.
	// Mirrors host/pico_esc/motor_model.py:MotorModel.handoff_possible().
	bool handoffPossible() const { return handoffPossibleAt(crossUp_); }
	// THE TRIGGER: the command whose 6-step duty demand clears the minimum-modulation floor. This used
	// to be cmdForSine(crossUpErpm()/pp) — the CONFIGURED SEAM — which put the handoff at 533/563/781
	// at cross_up 34/36/50 against measured 485/515.5/734.5, ~47 counts high on every motor, and made
	// sixstepFloorRpm() 1813 rpm on a 930KV at cross_up 36, where the encoder says 522.5. KV- and PP-free
	// (it is a duty): 484.21 / 515.41 / 733.81. Not to be confused with the seam itself, which is where
	// the 6-step branch OVERTAKES the field — a necessary condition for a STABLE handoff, modelled in
	// host/pico_esc/plant.py:MeasuredPlant.seam_cmd(). Mirrors
	// host/pico_esc/motor_model.py:MotorModel.handoff_cmd().
	// Refused => HANDOFF_CMD_NONE / INFINITY, so every `<=` / `>=` downstream fails CLOSED (thrustFor
	// stays on the sine branch, inGap is false, fillProfile emits nothing).
	float handoffCmd()  const { return handoffPossible() ? cmdForDuty6(DUTY6_HANDOFF_FLOOR)
	                                                     : HANDOFF_CMD_NONE; }
	// Where the ROTOR is when the trigger fires: still on the forced field, so the sine curve at
	// handoffCmd() (172.9 / 184.0 / 262.0 mech at pp = 7 and cross_up 34 / 36 / 50).
	float handoffMech() const { return handoffPossible() ? rpmSine(handoffCmd()) : INFINITY; }
	// The 6-step "landing" = KV*V*eff*DUTY6_HANDOFF_FLOOR exactly (cmdForDuty6 inverts duty6), so it
	// scales with KV*V and is INDEPENDENT of cross_up: 522.5 rpm on the 930KV at 11.2 V, 194.9 on the
	// 350KV at 11.1 V. The old law made it FALL as cross_up rose; that was an artifact of the bug.
	float sixstepFloorRpm() const { return handoffPossible() ? rpm6step(handoffCmd()) : INFINITY; }

	// ---- FF curves ----
	float rpmSine(float cmd) const { return (cmd / 1000.0f) * sineFullscaleMech(); }
	float rpm6step(float cmd) const {
		float duty = duty6(cmd);
		if (duty <= 0.0f) return 0.0f;
		float rpm = kv_ * v_ * eff_ * duty;
		float top = noLoadTop();
		return rpm < top ? rpm : top;
	}
	float cmd6stepFor(float rpm) const {
		float duty = rpm / (kv_ * v_ * eff_);
		return cmdForDuty6(duty);
	}
	float cmdForSine(float rpm) const { return rpm / sineFullscaleMech() * 1000.0f; }

	// Regime-aware inverse: sine below the handoff, 6-step above the landing; the gap between snaps up.
	// When the handoff is REFUSED both bounds are INFINITY, so the first test swallows everything and
	// the whole range inverts through the sine curve — which is the truth: that ESC config has no
	// 6-step branch to ask for. Same decision as the Python side's `None` guard, by construction.
	float thrustFor(float rpm) const {
		float s = rpm < 0.0f ? -1.0f : 1.0f;
		float a = fabsf(rpm);
		if (a <= handoffMech())     return s * cmdForSine(a);
		if (a >= sixstepFloorRpm()) return s * cmd6stepFor(a);
		return s * cmd6stepFor(sixstepFloorRpm());       // gap: cannot hold -> snap to the landing
	}
	// The no-steady-state gap [handoffMech, sixstepFloorRpm]. Under the duty law it can be NARROW
	// (350KV at 11.1 V, cross_up 36: 184.0 -> 194.9 rpm) or EMPTY/INVERTED (350KV at cross_up 50: the
	// trigger fires at 262.0 mech while the landing is still 194.9), and false everywhere when the
	// handoff is refused — `INFINITY < a` is false, so this fails closed with no extra branch.
	bool inGap(float rpm) const {
		float a = fabsf(rpm);
		return handoffMech() < a && a < sixstepFloorRpm();
	}

	// ---- eRPM self-cal (no encoder): fit eff from (cmd, tele_erpm) 6-step samples ----
	float selfCalErpm(const float* cmds, const float* erpms, int n) {
		float sum = 0.0f; int used = 0;
		for (int i = 0; i < n; i++) {
			float mech = fabsf(erpms[i]) / (float)pp_;
			float ideal = kv_ * v_ * duty6(cmds[i]);
			if (ideal > 1.0f) { sum += mech / ideal; used++; }
		}
		if (used > 0) eff_ = sum / (float)used;
		return eff_;
	}

	// ---- fill a caller-owned CurvePoint[] with the predicted curve (monotone). Returns the count. ----
	// nSine sine points below the handoff cmd, then (cap - nSine) 6-step points from the landing up to
	// cmdHi. Strictly increasing thrust + non-decreasing rpm (as SpeedProfile requires).
	// regimesOut (optional, same capacity) is tagged SINE/LINE per point — the generator knows which side
	// of the seam each point came from, and SpeedProfile::lineFloor() needs those tags to report the real
	// 6-step landing (~sixstepFloorRpm) instead of falling back to the seam eRPM. Pass it whenever the
	// profile will be asked where genuine 6-step starts.
	int fillProfile(CurvePoint* buf, int cap, float cmdHi = 700.0f, int nSine = 4,
	                Regime* regimesOut = nullptr) const {
		if (cap < 2) return 0;
		// TWO ways the 6-step section can fail to exist, and BOTH must emit nothing rather than a curve
		// that is silently unusable. Reuses the `cap < 2` convention: Thruster::setMotor reads a
		// 0/1-point return as "keep the current curve" (src/apps/thruster.h, setMotor's `n < 2` guard,
		// which also reports it up through setMotor's bool and Thruster::curveStale()), and that is the
		// fail-safe answer. Python raises ValueError at the same two points.
		//   * the handoff is REFUSED at this cross_up -> there is no 6-step branch at all;
		//   * the handoff command is at or above cmdHi -> fewer than two 6-step points fit, the loop
		//     below steps BACKWARDS and the resulting curve inverts to one command for every rpm. At
		//     the default cmdHi = 700 that is any cross_up >= 48, INCLUDING the cross_up 50 this bench
		//     run used; unguarded it returned 4 points, which sails straight past the `n < 2` guard.
		if (!handoffPossible() || handoffCmd() >= cmdHi) return 0;
		int n6 = cap - nSine;
		if (n6 < 2) { n6 = cap - 1; nSine = 1; }
		float sineHiCmd = cmdForSine(handoffMech());
		float floor = handoffCmd();
		int k = 0; int lastC = -1; float lastR = -1.0f;
		auto push = [&](float c, float r, Regime reg) {
			int ci = (int)(c + 0.5f);
			if (ci > lastC && r > lastR + 0.1f && k < cap) {
				buf[k].thrust = (float)ci; buf[k].rpm = r; lastC = ci; lastR = r;
				if (regimesOut) regimesOut[k] = reg;
				k++;
			}
		};
		for (int i = 0; i < nSine; i++) {
			float c = 60.0f + (sineHiCmd - 60.0f) * (float)i / (float)(nSine - 1 > 0 ? nSine - 1 : 1);
			push(c, rpmSine(c), Regime::SINE);
		}
		for (int i = 0; i < n6; i++) {
			float c = floor + (cmdHi - floor) * (float)i / (float)(n6 - 1 > 0 ? n6 - 1 : 1);
			push(c, rpm6step(c), Regime::LINE);
		}
		return k;
	}

	// ---- the derived ESC crossover config bytes (near-universal; eRPM-based -> KV-independent) ----
	// cross_up = the firmware handoff floor; cross_dn = 255 (disable the active down-handoff, sine
	// fallback — the working recipe; a seamless down-handoff is still firmware TODO).
	// RECOMMENDED, not current: the lowest handoff the firmware allows (~34), i.e. what to CONFIGURE on
	// a fresh ESC. Deliberately not `crossUp()` — that is what the ESC is actually set to, and the two
	// disagree the moment an operator moves the threshold. Writing this value to an ESC running 36 would
	// re-introduce exactly the duty bias DUTY6_CROSSUP_K exists to remove, so keep the names distinct.
	int recommendedCrossUpByte() const { return (int)(HANDOFF_ERPM / (10000.0f / 256.0f) + 0.5f); }
	int crossDnByte() const { return 255; }

private:
	float kv_; int pp_; float v_; float eff_;
	float crossUp_ = DUTY6_CROSSUP_REF;   // ESC sine_cross_up; REF => the historical behaviour
};

}  // namespace vel
