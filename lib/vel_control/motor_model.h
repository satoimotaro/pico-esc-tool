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
static const float SINE_FULLSCALE_ERPM = 2499.0f;   // firmware S1 full-scale eRPM at cmd 1000
static const float HANDOFF_ERPM = 1333.0f;          // lowest handoff the firmware allows (min slip)

class MotorModel {
public:
	MotorModel(float kv, int pole_pairs, float v_supply, float eff = 1.0f)
		: kv_(kv), pp_(pole_pairs), v_(v_supply), eff_(eff) {}

	// live setters (for --track-voltage style updates over serial)
	void set(float kv, int pp, float v) { kv_ = kv; pp_ = pp; v_ = v; eff_ = 1.0f; }
	void setVoltage(float v) { v_ = v; }
	void setEff(float e)     { eff_ = e; }

	// SOFT-SENSOR (ESC perception): infer effective supply V from the live command + measured 6-step
	// mech rpm, no voltage sensor. V_eff = rpm/(KV*duty(cmd)) = V_supply - I*R/duty. At no/light load
	// V_eff ~= V_supply; the DEFICIT (V_supply - V_eff) is proportional to current => torque => thrust
	// (a relative load/thrust signal without a force or current sensor). Returns 0 if not in 6-step.
	float estimateV(float cmd, float rpm) const {
		float d = DUTY6_A * fabsf(cmd) + DUTY6_B;
		return (d > 0.02f && kv_ > 0.0f && fabsf(rpm) > 100.0f) ? fabsf(rpm) / (kv_ * d) : 0.0f;
	}
	// Relative load signal = (V_supply_ref - V_eff): >0 and growing = more load (IR drop). 0 if not 6-step.
	float estimateLoad(float cmd, float rpm) const {
		float ve = estimateV(cmd, rpm);
		return ve > 0.0f ? (v_ - ve) : 0.0f;
	}
	float voltage() const    { return v_; }
	float eff() const        { return eff_; }
	int   polePairs() const  { return pp_; }

	// ---- scalar predictions ----
	float noLoadTop()         const { return kv_ * v_ * eff_; }
	float sineFullscaleMech() const { return SINE_FULLSCALE_ERPM / (float)pp_; }   // PP-governed
	float handoffMech()       const { return HANDOFF_ERPM / (float)pp_; }
	float handoffCmd()        const { return cmdForSine(handoffMech()); }
	float sixstepFloorRpm()   const { return rpm6step(handoffCmd()); }             // the 6-step "landing"

	// ---- FF curves ----
	float rpmSine(float cmd) const { return (cmd / 1000.0f) * sineFullscaleMech(); }
	float rpm6step(float cmd) const {
		float duty = DUTY6_A * fabsf(cmd) + DUTY6_B;
		if (duty <= 0.0f) return 0.0f;
		float rpm = kv_ * v_ * eff_ * duty;
		float top = noLoadTop();
		return rpm < top ? rpm : top;
	}
	float cmd6stepFor(float rpm) const {
		float duty = rpm / (kv_ * v_ * eff_);
		return (duty - DUTY6_B) / DUTY6_A;
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
			float ideal = kv_ * v_ * (DUTY6_A * fabsf(cmds[i]) + DUTY6_B);
			if (ideal > 1.0f) { sum += mech / ideal; used++; }
		}
		if (used > 0) eff_ = sum / (float)used;
		return eff_;
	}

	// ---- fill a caller-owned CurvePoint[] with the predicted curve (monotone). Returns the count. ----
	// nSine sine points below the handoff cmd, then (cap - nSine) 6-step points from the landing up to
	// cmdHi. Strictly increasing thrust + non-decreasing rpm (as SpeedProfile requires).
	int fillProfile(CurvePoint* buf, int cap, float cmdHi = 700.0f, int nSine = 4) const {
		if (cap < 2) return 0;
		int n6 = cap - nSine;
		if (n6 < 2) { n6 = cap - 1; nSine = 1; }
		float sineHiCmd = cmdForSine(handoffMech());
		float floor = handoffCmd();
		int k = 0; int lastC = -1; float lastR = -1.0f;
		auto push = [&](float c, float r) {
			int ci = (int)(c + 0.5f);
			if (ci > lastC && r > lastR + 0.1f && k < cap) { buf[k].thrust = (float)ci; buf[k].rpm = r; lastC = ci; lastR = r; k++; }
		};
		for (int i = 0; i < nSine; i++) {
			float c = 60.0f + (sineHiCmd - 60.0f) * (float)i / (float)(nSine - 1 > 0 ? nSine - 1 : 1);
			push(c, rpmSine(c));
		}
		for (int i = 0; i < n6; i++) {
			float c = floor + (cmdHi - floor) * (float)i / (float)(n6 - 1 > 0 ? n6 - 1 : 1);
			push(c, rpm6step(c));
		}
		return k;
	}

	// ---- the derived ESC crossover config bytes (near-universal; eRPM-based -> KV-independent) ----
	// cross_up = the firmware handoff floor; cross_dn = 255 (disable the active down-handoff, sine
	// fallback — the working recipe; a seamless down-handoff is still firmware TODO).
	int crossUpByte() const { return (int)(HANDOFF_ERPM / (10000.0f / 256.0f) + 0.5f); }  // ~34
	int crossDnByte() const { return 255; }

private:
	float kv_; int pp_; float v_; float eff_;
};

}  // namespace vel
