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
//   * handoff fires at the firmware floor HANDOFF_ERPM => mech handoff = HANDOFF_ERPM/PP.
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
static const float DUTY6_CROSSUP_K   = 0.0421f;    // measured d(duty)/d(Cross_Up); theory
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
	float cmdForDuty6(float duty) const {
		return (duty - DUTY6_B + DUTY6_CROSSUP_K * (crossUp_ - DUTY6_CROSSUP_REF)) / DUTY6_A;
	}

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
	// The CONFIGURED seam, not the firmware floor. This used to read HANDOFF_ERPM directly, so
	// crossUpErpm() existed but nothing downstream used it: at cross_up 36 the handoff command came
	// out 533 instead of 563 and at 50 it was 533 instead of 781. Mirrors
	// host/pico_esc/motor_model.py:MotorModel.handoff_mech().
	float handoffMech()       const { return crossUpErpm() / (float)pp_; }
	float handoffCmd()        const { return cmdForSine(handoffMech()); }
	float sixstepFloorRpm()   const { return rpm6step(handoffCmd()); }             // the 6-step "landing"

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
	float thrustFor(float rpm) const {
		float s = rpm < 0.0f ? -1.0f : 1.0f;
		float a = fabsf(rpm);
		if (a <= handoffMech())     return s * cmdForSine(a);
		if (a >= sixstepFloorRpm()) return s * cmd6stepFor(a);
		return s * cmd6stepFor(sixstepFloorRpm());       // gap: cannot hold -> snap to the landing
	}
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
