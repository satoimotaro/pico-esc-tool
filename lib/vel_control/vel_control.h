// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// vel_control — general closed-loop velocity control for a bidir-DShot ESC (Phase A2).
// -----------------------------------------------------------------------------------
// A faithful C++ port of the verified Python reference (host/pico_esc/velocity.py): signed target
// mechanical RPM in -> signed DShot thrust out, feed-forward from a calibrated speed curve plus a PI
// trim on (target - measured mech RPM) whose authority FADES with telemetry liveness. Above the
// firmware sine<->6-step seam the 6-step eRPM telemetry is live and the loop closes; below it (forced
// sine) telemetry is stale and the controller degrades to pure feed-forward.
//
// PORTABLE + TESTABLE: this header pulls in NO Arduino / PIO / hardware headers — it takes an
// injected `EscIo` backend (the hardware adapter) and an elapsed-dt each `step()`, so it compiles and
// unit-tests on a host with a mock backend (see test/), exactly like the Python sim. The RP2040 glue
// (an EscIo over escs:: from esc_session.h) lives in the app, not here.
//
// USAGE (the declaring main owns the gains — set them directly, `esc1.kp = ...`):
//     static const vel::CurvePoint PTS[] = { {0,0}, {620,2576}, {800,7173}, {900,9447} };
//     vel::SpeedProfile prof(PTS, 4, /*pole_pairs=*/7);
//     MyEscIo io1(/*index=*/1);                 // your adapter over the DShot engine
//     vel::VelocityController esc1(io1, prof);
//     esc1.kp = 0.03f; esc1.ki = 0.12f; esc1.trim_max = 400.0f;   // per-motor, plant-dependent
//     esc1.setTarget(5000.0f);                  // signed mech RPM
//     // every loop, with the real elapsed seconds:
//     if (esc1.step(dt) != vel::Status::OK) { /* aborted: over-speed / stall / over-temp */ }
//
// INVARIANT (do NOT break): the telemetry rpm the backend returns is ALREADY MECHANICAL — the ESC
// firmware pre-divides the DShot eRPM by pole pairs. It is used DIRECTLY; dividing by pole pairs here
// is the 1/7 double-division bug. Pole pairs is used ONLY to convert an rpm to eRPM for the seam.
#pragma once
#include <stdint.h>
#include <math.h>

namespace vel {

// A `tele` frame counts as a LIVE 6-step sample once |mech RPM| exceeds this floor (rejects the
// garbage first-few-frames-after-arm and forced-sine's stale ~0). Mirrors constants.TELE_MIN_MECH_RPM.
static const float TELE_MIN_MECH_RPM = 50.0f;
static const float VEL_DT_DEFAULT     = 0.02f;   // dt fallback when a caller passes <=0 (50 Hz)
static const float W_BACKCALC_FLOOR   = 0.1f;    // outer-clamp back-calc only above this authority

// Built-in PI gains — the single default shared with the constructor below. These are the SIM-tuned
// values; a real motor's plant gain is ~30x higher (930KV 6-step ~23 mech RPM per command-unit), so
// set the per-motor gains on the controller (esc1.kp = 0.03f; ...). Mirrors velocity.DEFAULT_GAINS.
struct Gains { float kp, ki, trim_max, blend_secs; };
static const Gains DEFAULT_GAINS = { 0.4f, 1.5f, 200.0f, 0.3f };

enum class Regime : uint8_t { SINE, LINE };
enum class Status : uint8_t { OK, ABORT_OVERSPEED, ABORT_STALL, ABORT_TEMP };

// The firmware sine<->6-step crossover seam, in eRPM (as configured on the ESC).
struct Crossover { float up_erpm; float dn_erpm; };

// One calibration point: signed ESC command magnitude (0..1000, up-sweep only) -> mechanical RPM.
struct CurvePoint { float thrust; float rpm; };

// ---------------------------------------------------------------------------------------------------
// SpeedProfile — a per-motor calibrated command<->speed curve + its runtime inverse. The points array
// is OWNED BY THE CALLER (static storage, no allocation); the profile just references it.
// ---------------------------------------------------------------------------------------------------
class SpeedProfile {
public:
	// pts: monotonic (thrust strictly increasing, rpm non-decreasing), pts[0].thrust >= 0. crossover /
	// regimes may be null. regimes (optional, length n) tags each point's believed firmware regime so
	// lineFloor() can report the lowest genuinely-6-step-reachable rpm.
	SpeedProfile(const CurvePoint* pts, int n, int pole_pairs = 7,
	             const Crossover* crossover = nullptr, const Regime* regimes = nullptr,
	             bool down_catch = false)
		: pts_(pts), n_(n), pole_pairs_(pole_pairs), cx_(crossover), regimes_(regimes),
		  down_catch_(down_catch) {}

	// Signed ESC command for a target mechanical RPM (piecewise-linear inverse). Exact at the points,
	// clamped to the endpoints outside the range, odd-symmetric so thrustFor(-rpm) == -thrustFor(rpm).
	float thrustFor(float rpm) const {
		float sign = rpm < 0 ? -1.0f : 1.0f;
		float a = fabsf(rpm);
		if (a <= pts_[0].rpm)      return sign * pts_[0].thrust;
		if (a >= pts_[n_ - 1].rpm) return sign * pts_[n_ - 1].thrust;
		for (int i = 0; i + 1 < n_; i++) {
			float r0 = pts_[i].rpm, r1 = pts_[i + 1].rpm;
			if (r0 <= a && a <= r1) {
				if (r1 == r0) return sign * pts_[i].thrust;         // rpm plateau -> lower command
				float frac = (a - r0) / (r1 - r0);
				return sign * (pts_[i].thrust + frac * (pts_[i + 1].thrust - pts_[i].thrust));
			}
		}
		return sign * pts_[n_ - 1].thrust;                          // unreachable (guarded above)
	}

	float maxRpm() const { return pts_[n_ - 1].rpm; }
	int   polePairs() const { return pole_pairs_; }
	bool  hasCrossover() const { return cx_ != nullptr; }
	bool  downCatch() const { return down_catch_; }

	// ADVISORY regime of a nominal rpm (display / stall heuristic / down-catch routing) — NEVER the
	// PI-authority signal (that is telemetry liveness alone). SINE when there is no crossover.
	Regime regime(float rpm) const {
		if (!cx_) return Regime::SINE;
		return (fabsf(rpm) * (float)pole_pairs_ >= cx_->up_erpm) ? Regime::LINE : Regime::SINE;
	}

	// Lowest setpoint |RPM| genuinely REACHABLE in 6-step per this profile: the min rpm among
	// line-tagged points if tagged, else the seam rpm (up_erpm / pole_pairs). false if no crossover.
	bool lineFloor(float& out) const {
		if (!cx_) return false;
		if (regimes_) {
			bool any = false; float lo = 0.0f;
			for (int i = 0; i < n_; i++)
				if (regimes_[i] == Regime::LINE && (!any || pts_[i].rpm < lo)) { lo = pts_[i].rpm; any = true; }
			if (any) { out = lo; return true; }
		}
		out = cx_->up_erpm / (float)pole_pairs_;
		return true;
	}

private:
	const CurvePoint* pts_;
	int               n_;
	int               pole_pairs_;
	const Crossover*  cx_;
	const Regime*     regimes_;
	bool              down_catch_;
};

// ---------------------------------------------------------------------------------------------------
// EscIo — the hardware backend the controller drives. Implement it once per ESC (an adapter over your
// DShot engine); inject it into the controller. Keeps this library free of hardware headers.
// ---------------------------------------------------------------------------------------------------
struct EscIo {
	virtual ~EscIo() {}
	// Drive the ESC with a signed command (already clamped to the controller's ±tmax). No return.
	virtual void thrust(int cmd) = 0;
	// Latest telemetry: on a FRESH live 6-step frame, set mechRpm (ALREADY mechanical) + tempC and
	// return true. Return false when there is no fresh frame (forced sine, dropout, pre-arm) — the
	// controller then treats this tick as "no measurement" and its PI authority fades out.
	virtual bool readTele(float& mechRpm, float& tempC) = 0;
};

// ---------------------------------------------------------------------------------------------------
// VelocityController — FF + a liveness-faded PI trim. Deps (backend + profile) are constructor-
// injected; the tunables below are PUBLIC and meant to be set from the declaring main.
// ---------------------------------------------------------------------------------------------------
class VelocityController {
public:
	VelocityController(EscIo& io, const SpeedProfile& profile) : io_(io), profile_(profile) {}

	// --- tunables: set directly in main (esc1.kp = 0.03f;). Defaults = sim-tuned DEFAULT_GAINS. ---
	float kp        = DEFAULT_GAINS.kp;
	float ki        = DEFAULT_GAINS.ki;
	float trim_max  = DEFAULT_GAINS.trim_max;
	float blend_secs = DEFAULT_GAINS.blend_secs;
	// Derivative trim on the MEASURED velocity (not the error) so a slewed setpoint step gives no
	// derivative kick — it only damps how fast the shaft is actually accelerating, which curbs the
	// overshoot on hard up-steps. Low-passed (d_tau) because the eRPM telemetry is quantized, and it
	// rides the same liveness fade as the PI (contributes 0 in forced sine). 0 => disabled (default).
	float kd        = 0.0f;
	float d_tau     = 0.04f;    // derivative low-pass time constant (s)
	float slew_rpm_s   = 200.0f;    // setpoint slew rate (keeps the first command gentle)
	float over_speed_rpm = 0.0f;    // 0 => auto: max(2*maxRpm, 1200)
	float stop_below_rpm = 0.0f;    // a target at/below this |RPM| is a STOP: command thrust 0 and
	                                // disengage (can't hold speed sensorlessly near 0). 0 => only an
	                                // exact setTarget(0) stops. This makes `rpm 0` actually STOP.
	float stall_secs   = 1.0f;      // abort if commanding into 6-step but tele stays stale this long
	float max_temp     = 0.0f;      // 0 => no temp abort (EDT temp is unreliable on these ESCs)
	int   tmax         = 1000;      // command magnitude ceiling

	// Set the commanded target (signed mech RPM). Down-catch staging: on a profile with a crossover
	// and no down_catch, a drop from above the seam to below it routes the setpoint through ~0 first
	// (re-acquire from below) rather than dropping across the handoff. Inert otherwise.
	void setTarget(float rpm) {
		if (profile_.hasCrossover() && !profile_.downCatch()
		    && profile_.regime(rpm) == Regime::SINE && profile_.regime(setpoint_) == Regime::LINE) {
			pending_ = rpm; have_pending_ = true; target_ = 0.0f;
		} else {
			have_pending_ = false; target_ = rpm;
		}
	}

	// Clear all closed-loop state (integrator/authority/measurement). Call before starting a fresh
	// run so a re-arm from below the seam starts clean.
	void reset() {
		i_ = 0.0f; w_ = 0.0f; have_tele_ = false; tele_mech_ = 0.0f; live_ = false;
		stale_accum_ = 0.0f; last_sent_ = 0; last_applied_ = 0.0f;
		prev_meas_ = 0.0f; have_prev_meas_ = false; d_filt_ = 0.0f; last_d_ = 0.0f;
	}

	// One control tick. dt = elapsed seconds since the last step (<=0 -> VEL_DT_DEFAULT). Reads
	// telemetry, computes FF + blended PI trim, drives the ESC, runs the guards. Returns OK, or an
	// ABORT_* status after commanding thrust 0 (the caller should then disarm).
	Status step(float dt) {
		if (dt <= 0.0f) dt = VEL_DT_DEFAULT;

		// STOP request: a target at/below stop_below_rpm can't be held sensorlessly (no BEMF near
		// zero) -> command a true thrust 0 and disengage the loop, so setTarget(0) actually STOPS
		// instead of the FF/slew/PI creeping the motor. (arm/disarm stay for enable/kill.)
		if (fabsf(target_) <= stop_below_rpm) {
			setpoint_ = 0.0f; i_ = 0.0f; w_ = 0.0f; live_ = false;
			have_pending_ = false; stale_accum_ = 0.0f;
			io_.thrust(0); last_sent_ = 0; last_applied_ = 0.0f;
			return Status::OK;
		}

		float sp = slew(dt);
		if (sp > 0.0f) last_sign_ = 1.0f; else if (sp < 0.0f) last_sign_ = -1.0f;

		// A staged (line->sine) target promotes once we have descended to ~0 / dropped below the seam.
		if (have_pending_ && (fabsf(setpoint_) < 1.0f
		                      || (!live_ && profile_.regime(sp) == Regime::SINE))) {
			target_ = pending_; have_pending_ = false;
		}

		// -- telemetry: the SAME frame drives feedback AND the temp abort --
		bool have_meas = false;
		float mr = 0.0f, tc = 0.0f;
		if (io_.readTele(mr, tc)) {
			if (max_temp > 0.0f && tc > 0.0f) {
				last_temp_ = tc; peak_temp_ = (peak_temp_ < tc) ? tc : peak_temp_;
				if (tc >= max_temp) { io_.thrust(0); return Status::ABORT_TEMP; }
			}
			if (fabsf(mr) > TELE_MIN_MECH_RPM) {                 // live: re-attach the commanded sign
				float sign = (sp > 0.0f) ? 1.0f : (sp < 0.0f) ? -1.0f : (last_sign_ >= 0.0f ? 1.0f : -1.0f);
				tele_mech_ = sign * fabsf(mr); have_tele_ = true; have_meas = true;
			}
		}
		live_ = have_meas;

		// -- PI authority (w) fades with liveness: 0->1 live, 1->0 stale, over blend_secs. Reset the
		//    integrator at w==0 so a re-arm from below the seam starts clean. --
		float rate = (blend_secs > 0.0f) ? dt / blend_secs : 1.0f;
		if (live_) { w_ += rate; if (w_ > 1.0f) w_ = 1.0f; }
		else       { w_ -= rate; if (w_ < 0.0f) w_ = 0.0f; }
		if (w_ <= 0.0f) i_ = 0.0f;

		// -- feed-forward + blended PI trim --
		float ff = profile_.thrustFor(sp);
		float trim = closedLoopTrim(sp, dt);
		float applied = w_ * trim;
		float cmd = ff + applied;
		int sent = clampCmd(cmd);
		io_.thrust(sent);
		last_sent_ = sent; last_applied_ = applied;

		// -- back-calculation anti-windup on the OUTER ±tmax clamp. Only when the trim materially drove
		//    the clamp: skip below a real authority floor, and skip when the FF term alone saturated
		//    (delivered opposes the desired trim) so the integrator can't flip against the true error. --
		if (have_tele_ && w_ >= W_BACKCALC_FLOOR && sent != (int)cmd) {
			float desired = applied;                         // w*trim we tried to add above the FF
			float delivered = (float)(sent - (int)ff);       // trim the ESC actually delivered
			if (desired != 0.0f && delivered * desired > 0.0f && fabsf(delivered) < fabsf(desired)) {
				float err = sp - tele_mech_;
				float unblended = delivered / w_;            // |unblended| < |trim| <= trim_max
				i_ = clampf(unblended - kp * err - last_d_, -trim_max, trim_max);
			}
		}

		// -- safety guards on the LIVE measurement (never on the profile/crossover config) --
		if (live_ && fabsf(tele_mech_) > effectiveOverSpeed()) { io_.thrust(0); return Status::ABORT_OVERSPEED; }

		// stall: the setpoint is at/above the profile's genuinely 6-step-reachable floor (so we EXPECT
		// live telemetry) yet the command is still driving into that region and telemetry never went
		// live -> the ESC failed to reach 6-step.
		float floor;
		bool commanding_line = profile_.lineFloor(floor) && fabsf(sp) >= floor
		                       && (float)abs(sent) >= fabsf(ff) - trim_max;
		if (commanding_line && !live_) {
			stale_accum_ += dt;
			if (stale_accum_ >= stall_secs) { io_.thrust(0); return Status::ABORT_STALL; }
		} else {
			stale_accum_ = 0.0f;
		}
		return Status::OK;
	}

	// --- introspection ---
	float setpoint() const { return setpoint_; }
	float target()   const { return target_; }
	int   command()  const { return last_sent_; }
	float trim()     const { return last_applied_; }     // BLENDED trim actually added to the FF
	float measured() const { return tele_mech_; }        // last live mech RPM (signed); stale if !live()
	bool  live()     const { return live_; }
	float authority() const { return w_; }               // PI weight 0..1

private:
	float slew(float dt) {
		float step = slew_rpm_s * dt;
		if (setpoint_ < target_)      { setpoint_ += step; if (setpoint_ > target_) setpoint_ = target_; }
		else if (setpoint_ > target_) { setpoint_ -= step; if (setpoint_ < target_) setpoint_ = target_; }
		return setpoint_;
	}

	// PI trim on (setpoint - measured), ±trim_max with back-calc anti-windup on the clamp. Integrates
	// ONLY on a fresh live frame (during fade-out the last measurement is frozen). 0 with no measurement.
	float closedLoopTrim(float sp, float dt) {
		if (!have_tele_) return 0.0f;
		float err = sp - tele_mech_;
		if (live_) i_ += ki * err * dt;                  // accumulate only against a FRESH live frame
		// derivative on MEASUREMENT (low-passed), updated only on a fresh live frame; -kd opposes rising
		// speed so it damps the approach. Frozen (not decayed) during a fade-out; w_ scales it to 0 anyway.
		if (kd > 0.0f && live_) {
			if (have_prev_meas_ && dt > 0.0f) {
				float dmeas = (tele_mech_ - prev_meas_) / dt;
				float a = (d_tau > 0.0f) ? dt / (d_tau + dt) : 1.0f;
				d_filt_ += (dmeas - d_filt_) * a;
			}
			prev_meas_ = tele_mech_; have_prev_meas_ = true;
		}
		last_d_ = (kd > 0.0f) ? -kd * d_filt_ : 0.0f;
		float u = kp * err + i_ + last_d_;
		float uc = clampf(u, -trim_max, trim_max);
		if (u != uc) { i_ = clampf(uc - kp * err - last_d_, -trim_max, trim_max); u = uc; }
		return u;
	}

	int clampCmd(float cmd) const {
		int v = (int)cmd;                                // truncate toward zero (matches Python int())
		if (v >  1000) v =  1000;
		if (v < -1000) v = -1000;
		if (v >  tmax) v =  tmax;
		if (v < -tmax) v = -tmax;
		return v;
	}
	float effectiveOverSpeed() const {
		if (over_speed_rpm > 0.0f) return over_speed_rpm;
		float twice = 2.0f * profile_.maxRpm();
		return twice > 1200.0f ? twice : 1200.0f;
	}
	static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

	EscIo&             io_;
	const SpeedProfile& profile_;

	// --- state (no static/global mutable state) ---
	float target_ = 0.0f, setpoint_ = 0.0f;
	float pending_ = 0.0f; bool have_pending_ = false;
	float i_ = 0.0f, w_ = 0.0f;
	float tele_mech_ = 0.0f; bool have_tele_ = false, live_ = false;
	float prev_meas_ = 0.0f; bool have_prev_meas_ = false; float d_filt_ = 0.0f, last_d_ = 0.0f;
	float last_sign_ = 1.0f;
	float stale_accum_ = 0.0f;
	int   last_sent_ = 0; float last_applied_ = 0.0f;
	float last_temp_ = 0.0f, peak_temp_ = 0.0f;
};

}  // namespace vel
