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
static const float DOB_LIVE_FLOOR     = 0.9f;    // DOB engages only when liveness is (near) fully established
// Below this the profile's forward prediction carries no information worth checking a measurement
// against: it is the curve's own low-command region, where the segment resolution and the ESC's
// telemetry floor are the same size as the prediction. See VelocityController::track_min_frac.
static const float TRACK_MIN_PREDICTED_RPM = 100.0f;

// Built-in PI gains — the single default shared with the constructor below. These are the SIM-tuned
// values; a real motor's plant gain is ~30x higher (930KV 6-step ~23 mech RPM per command-unit), so
// set the per-motor gains on the controller (esc1.kp = 0.03f; ...). Mirrors velocity.DEFAULT_GAINS.
struct Gains { float kp, ki, trim_max, blend_secs; };
static const Gains DEFAULT_GAINS = { 0.4f, 1.5f, 200.0f, 0.3f };

enum class Regime : uint8_t { SINE, LINE };
enum class Status : uint8_t { OK, ABORT_OVERSPEED, ABORT_STALL, ABORT_TEMP };

// ---- staged thermal protection (see step()/updateTemp_ below) --------------------------------------
// The ONLY thermal observable this hardware has is the EDT temperature of the ESC's OWN MCU — there is
// no winding sensor, and Bluejay's EDT emits temperature/status/debug/demag only (no V, no A). Treat it
// as a lagging proxy, never as a winding temperature.
// The band is chosen to sit ENTIRELY BELOW the ESC's own protection so the two can never fight: the
// BlueGill/Bluejay limiter (src/Modules/Scheduler.asm + Settings.asm) is a step-down PWM staircase
// (255->200->150->100->50, i.e. ~100/80/60/40/20 %, in ~5 C steps) whose LOWEST selectable onset is
// 80 C (Layouts/Base.inc: TEMP_LIMIT_2S/1S). 60 C is also the most conservative host abort already in
// this repo (autocal.py / xover_debug.py --max-temp 60) and 80 C is the driftsoak/velcal/posctl
// default, so on the bench the HOST abort still fires first and the firmware is only the backstop.
static const float DEFAULT_TEMP_WARN_C     = 60.0f;   // derate starts here
static const float DEFAULT_TEMP_ABORT_C    = 80.0f;   // derate is at its floor here, and we abort
static const float DEFAULT_TEMP_MIN_SCALE  = 0.35f;   // keep ~a third of the command authority at the
                                                      // bottom of the band: ABOVE the 20 % PWM floor
                                                      // the ESC's own staircase bottoms out at (its
                                                      // source calls that "forced landing"), so this
                                                      // derate is never what makes a vehicle
                                                      // unmanoeuvrable. Not bench-validated.
static const float DEFAULT_TEMP_STALE_SECS = 2.0f;    // temperature this old is no longer evidence.
                                                      // EDT frames are interleaved with the eRPM ones,
                                                      // so 2 s should be many frames — but that cadence
                                                      // was never measured, which is exactly why this
                                                      // is a named constant and not a literal.
static const int   TEMP_ABORT_READINGS     = 3;       // consecutive over-limit READINGS before aborting
                                                      // — EDT temperature is an integer C in a frame
                                                      // that can garble, and one bad frame must not
                                                      // stop a thruster underwater.

// What the limiter currently knows. NONE and STALE both mean "no protection" — the difference is that
// NONE never had any and STALE lost it, and both are reported so the absence is never silent.
enum class TempState : uint8_t { NONE, OK, DERATE, STALE };
inline const char* tempStateName(TempState s) {
	switch (s) {
		case TempState::NONE:   return "none";
		case TempState::OK:     return "ok";
		case TempState::DERATE: return "derate";
		case TempState::STALE:  return "stale";
	}
	return "?";
}

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

	// True when segment [i, i+1] STRADDLES the firmware's sine<->6-step seam. thrustFor() documents that
	// the rpm band across the seam "has NO steady state" — the rotor cannot hold anything in between, so
	// on the forward map that segment is a STEP, not a ramp, and interpolating it invents both an
	// unreachable rpm and a wildly wrong local gain. Always false without crossover metadata.
	bool gapSegment(int i) const {
		if (!cx_ || i < 0 || i + 1 >= n_) return false;
		return regime(pts_[i].rpm) != regime(pts_[i + 1].rpm);
	}

	// Index of the segment bracketing |thrust| (clamped to the end segments). Requires n_ >= 2.
	int segmentAt(float thrust) const {
		float a = fabsf(thrust);
		for (int i = 0; i + 1 < n_; i++)
			if (pts_[i].thrust <= a && a <= pts_[i + 1].thrust) return i;
		return (a < pts_[0].thrust) ? 0 : n_ - 2;
	}

	// Forward map (thrust -> mech RPM): the inverse of thrustFor, used by the DOB to predict the rpm a
	// command SHOULD produce (the deficit vs measured = the disturbance). Same odd-symmetric interp,
	// except ACROSS THE SEAM (gapSegment) where it snaps to the nearer endpoint instead of interpolating
	// an rpm the plant provably cannot hold.
	float rpmFor(float thrust) const {
		float sign = thrust < 0 ? -1.0f : 1.0f;
		float a = fabsf(thrust);
		if (a <= pts_[0].thrust)      return sign * pts_[0].rpm;
		if (a >= pts_[n_ - 1].thrust) return sign * pts_[n_ - 1].rpm;
		for (int i = 0; i + 1 < n_; i++) {
			float t0 = pts_[i].thrust, t1 = pts_[i + 1].thrust;
			if (t0 <= a && a <= t1) {
				if (t1 == t0) return sign * pts_[i].rpm;
				float frac = (a - t0) / (t1 - t0);
				if (gapSegment(i)) return sign * (frac < 0.5f ? pts_[i].rpm : pts_[i + 1].rpm);
				return sign * (pts_[i].rpm + frac * (pts_[i + 1].rpm - pts_[i].rpm));
			}
		}
		return sign * pts_[n_ - 1].rpm;
	}

	// Local gain drpm/dthrust at a command (>= a small floor), to convert a DOB rpm deficit into a
	// command compensation. Clamped positive so a plateau can't blow up the compensation. On a segment
	// that straddles the seam the slope is meaningless (the jump / a few command units), so fall back to
	// the nearest REAL segment — searching upward first, since that is the 6-step side the DOB runs on.
	float rpmSlopeAt(float thrust) const {
		int i = segmentAt(thrust);
		if (gapSegment(i)) {
			int j = -1;
			for (int k = i + 1; k + 1 < n_; k++) if (!gapSegment(k)) { j = k; break; }
			if (j < 0) for (int k = i - 1; k >= 0; k--) if (!gapSegment(k)) { j = k; break; }
			if (j >= 0) i = j;
		}
		float dth = pts_[i + 1].thrust - pts_[i].thrust;
		float s = (dth > 0.0f) ? (pts_[i + 1].rpm - pts_[i].rpm) / dth : 0.0f;
		return s > 0.05f ? s : 0.05f;
	}

	// Re-point the curve at runtime (e.g. after `motor <kv> <pp> <v>` regenerates the buffer via
	// MotorModel::fillProfile into the SAME caller-owned array). The profile object is stable, so a
	// VelocityController holding a reference to it transparently picks up the new curve. pts must stay
	// monotone (thrust strictly increasing, rpm non-decreasing).
	void setPoints(const CurvePoint* pts, int n) { pts_ = pts; n_ = n; }

	float maxRpm() const { return pts_[n_ - 1].rpm; }
	int   polePairs() const { return pole_pairs_; }
	bool  hasCrossover() const { return cx_ != nullptr; }
	bool  hasRegimes()   const { return regimes_ != nullptr; }   // lineFloor() is the real landing, not the seam
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
// Curve helpers — the SEMANTICS of an uploaded measured curve, kept here rather than in the Thruster
// so they are hardware-free and can be tested natively (the firmware wrapper is Arduino-bound).
// ---------------------------------------------------------------------------------------------------

// A curve is usable iff it is a strictly increasing, non-decreasing, non-negative table of >= 2 points
// — exactly what SpeedProfile's interpolation and its inverse assume.
inline bool validateCurve(const CurvePoint* pts, int n) {
	if (!pts || n < 2) return false;
	for (int i = 0; i < n; i++) {
		if (pts[i].thrust < 0.0f || pts[i].rpm < 0.0f) return false;
		if (i && (pts[i].thrust <= pts[i - 1].thrust || pts[i].rpm < pts[i - 1].rpm)) return false;
	}
	return true;
}

// Decide each point's regime for an uploaded curve.
//   in == null  -> derive purely from the seam.
//   in != null  -> honour the tags, EXCEPT that a SINE tag at or above the seam is PROMOTED to LINE.
// The promotion exists because rpm*pp >= up_erpm is the same test SpeedProfile::regime() applies
// everywhere else, so a contradicting tag would leave lineFloor() (and the soft-sensor gate built on
// it) disagreeing with the rest of the library. velcal mislabels the handoff point in practice. An
// explicit LINE tag below the seam is never demoted: a measurement taken in the hysteresis band coming
// down is real information the seam test cannot represent.
inline void tagRegimes(const CurvePoint* pts, int n, int polePairs, float upErpm,
                       const Regime* in, Regime* out) {
	for (int i = 0; i < n; i++) {
		bool aboveSeam = pts[i].rpm * (float)polePairs >= upErpm;
		out[i] = (!in || aboveSeam) ? (aboveSeam ? Regime::LINE : Regime::SINE) : in[i];
	}
}

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
	// Latest ESC TEMPERATURE + the timestamp of the frame it came from, INDEPENDENT of eRPM freshness —
	// forced sine has stale rpm (readTele false) and is the hottest regime, so a thermal limiter riding
	// on readTele is blind exactly where it is needed.
	// STATELESS CONTRACT, deliberately unlike readTele: just report the last decoded temperature and its
	// stamp (any monotonic per-frame tick — millis(), a frame counter — only CHANGES matter), and return
	// false only when nothing has ever been decoded. The controller derives newness (the stamp moved) and
	// staleness (temp_stale_secs since it last moved) itself, so the abort debounce counts EDT FRAMES
	// rather than control ticks without every adapter having to keep dedupe state and re-implement the
	// aging policy. DEFAULTED to "no temperature", so an adapter written against the old EscIo still
	// compiles and keeps its readTele tempC path.
	virtual bool readTemp(float& tempC, uint32_t& stampMs) { (void)tempC; (void)stampMs; return false; }
};

// ---------------------------------------------------------------------------------------------------
// VelocityController — FF + a liveness-faded PI trim. Deps (backend + profile) are constructor-
// injected; the tunables below are PUBLIC and meant to be set from the declaring main.
// ---------------------------------------------------------------------------------------------------
class VelocityController {
public:
	VelocityController(EscIo& io, const SpeedProfile& profile) : io_(io), profile_(&profile) {}

	// Swap the FF curve at runtime (e.g. after `motor <kv> <pp> <v>` regenerates it via MotorModel).
	// The profile object must outlive the controller. With SpeedProfile::setPoints this makes the Pico
	// velocity loop reconfigurable from serial without recompiling.
	void setProfile(const SpeedProfile& p) { profile_ = &p; }
	const SpeedProfile& profile() const { return *profile_; }

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
	// Disturbance observer (DOB): d_hat = LPF(rpmFor(last cmd) - measured mech RPM) — the deficit vs the
	// profile's forward prediction IS the load disturbance (fouled prop / flow / entanglement). Fed back
	// as command compensation (dob * d_hat / local-gain), it rejects load steps far faster than the PI
	// integral alone. Rides the same liveness fade as the PI (0 in forced sine). 0 => disabled (default,
	// opt-in). Set per-motor (esc1.dob = 0.6f;) or over serial (gain <i> dob 0.6).
	float dob       = 0.0f;
	float dob_tau   = 0.08f;    // DOB low-pass time constant (s)
	float dob_max   = 150.0f;   // clamp on the DOB command compensation — a MIS-calibrated FF makes the
	                            // model deficit look like a huge load; this bounds the over-drive (the PI
	                            // then trims the residual). Use an ACCURATE profile for the full benefit.
	// TOTAL feedback authority: the largest deviation from the feed-forward that ALL closed-loop paths
	// together may command. <= 0 (default) derives it as max(trim_max, dob_max).
	//
	// WHY THIS EXISTS. trim_max and dob_max were clamped INDEPENDENTLY, so the reachable deviation was
	// their SUM — a number nobody chose. Measured 2026-08-01 on the 930KV: trim_max 80 + dob_max 150 =
	// 230 against a steady-state feed-forward of 641, and the startup command was 872. The operator
	// authorised 80 for the PI and 150 for the observer; they never authorised 230. Deriving the budget
	// as the LARGER of the two says exactly that, and it needs no new number to tune.
	//
	// It is a bound on the SIGNED total, and the OBSERVER is the path that gives way: `applied` (the PI
	// trim) is already <= trim_max <= the derived budget, so it is never cut and the integrator's
	// back-calculation is untouched by this. When the two paths OPPOSE (a DOB pushing down against a PI
	// pushing up) their signed sum is small and nothing is clamped at all — this bounds stacking, not
	// disagreement. With dob == 0 it is a mathematical no-op.
	//
	// Raising it is a deliberate act: it says a genuine load may need both paths at once. Degraded
	// tracking under a load that big is the acceptable failure; a command slam is not.
	float fb_max    = 0.0f;
	// AUTHORITY REQUIRES CORROBORATION: the fraction of the profile's own forward prediction that the
	// measurement must reach before the PI is allowed to gain authority. 0 disables the test.
	//
	// WHY THIS EXISTS, and it is a spec violation this file was written around. The header above says
	// the liveness gate rejects "forced-sine's stale ~0" — i.e. this controller assumed an ESC that
	// reports ~0 when it is NOT BEMF-locked. BlueGill reports a floored 182 mech rpm instead, so
	// TELE_MIN_MECH_RPM = 50 can never separate "the rotor is turning at 182" from "the ESC is lying
	// about a rotor that is not turning at all". The gate said LIVE while the rotor cogged in place,
	// and both feedback paths then integrated against a fabricated ~3800 rpm error.
	//
	// Liveness ("a frame arrived, and it is not obviously garbage") is not evidence that the plant is
	// TRACKING. This is: the profile predicts the rpm the command we actually sent should produce, and
	// a measurement far below that fraction of it is not a plant we are controlling. A real load in
	// water — the case the DOB exists for — costs tens of percent of speed, not 95 %, so 0.25 leaves
	// the observer its whole legitimate range while catching the 182-against-4000 case by 5x.
	//
	// It gates the PI AUTHORITY only. live_ itself is left alone on purpose: the over-speed and stall
	// aborts are safety guards and must keep firing on the raw frame, whatever we think of it.
	float track_min_frac = 0.25f;
	// SETTLE gate (the firmware counterpart of the host loop's veltune.VelLoop._settle_gate): rpmFor()
	// predicts the STEADY rpm of a command, but the plant is still rising toward it with tau ~25-130 ms in
	// 6-step — so while the setpoint is slewing the raw deficit is the plant transient, not load, and
	// compensating it partly defeats slew_rpm_s. So: hold the DOB OFF for as long as the setpoint is
	// moving, then fade it back in over dob_settle_secs once the setpoint has ARRIVED (the fade has to
	// extend past arrival, because that is exactly where the plant lag peaks). Keyed on
	// |target - setpoint| — the remaining SLEW, never the tracking error: once the slew finishes the gate
	// reaches 1 however large the steady error, so the DOB still trims the full load. 0 => no gating.
	float dob_settle_secs = 0.25f;   // fade-in after the setpoint arrives (>> the 6-step plant tau)
	float slew_rpm_s   = 200.0f;    // setpoint slew rate (keeps the first command gentle)
	float over_speed_rpm = 0.0f;    // 0 => auto: max(2*maxRpm, 1200)
	float stop_below_rpm = 0.0f;    // a target at/below this |RPM| is a STOP: command thrust 0 and
	                                // disengage (can't hold speed sensorlessly near 0). 0 => only an
	                                // exact setTarget(0) stops. This makes `rpm 0` actually STOP.
	float stall_secs   = 1.0f;      // abort if commanding into 6-step but tele stays stale this long
	// Thermal staging. max_temp keeps its ORIGINAL meaning — the abort — and 0 still means "no temp
	// check at all" (neither derate nor abort), so a caller that never sets these is unchanged. The
	// derate runs over [warn_temp, max_temp] and needs BOTH set (warn_temp > 0 && max_temp > warn_temp);
	// the composition root sets them from DEFAULT_TEMP_* above. Never persisted: see src/main.cpp.
	float warn_temp    = 0.0f;      // 0 => no derate (library default: exactly the pre-B.7 behaviour)
	float max_temp     = 0.0f;      // 0 => no temp check at all (EDT temp is the ESC MCU, not the winding)
	float temp_min_scale = DEFAULT_TEMP_MIN_SCALE;   // command ceiling at/above max_temp, as a fraction
	float temp_stale_secs = DEFAULT_TEMP_STALE_SECS; // temperature older than this releases the limit
	int   tmax         = 1000;      // command magnitude ceiling

	// Set the commanded target (signed mech RPM). Down-catch staging: on a profile with a crossover
	// and no down_catch, a drop from above the seam to below it routes the setpoint through ~0 first
	// (re-acquire from below) rather than dropping across the handoff. Inert otherwise.
	void setTarget(float rpm) {
		if (profile_->hasCrossover() && !profile_->downCatch()
		    && profile_->regime(rpm) == Regime::SINE && profile_->regime(setpoint_) == Regime::LINE) {
			pending_ = rpm; have_pending_ = true; target_ = 0.0f;
		} else {
			have_pending_ = false; target_ = rpm;
		}
	}

	// Clear all closed-loop state (integrator/authority/measurement). Call before starting a fresh
	// run so a re-arm from below the seam starts clean.
	void reset() {
		// setpoint_ IS closed-loop state, and leaving it behind DEFEATED slew_rpm_s entirely on real
		// hardware. `setpoint_ = 0` lives in exactly one other place — step()'s stop branch — and
		// Thruster::poll() has its OWN stop branch that returns BEFORE calling step(), so on the Pico
		// that assignment is unreachable. The setpoint therefore survived every stop at its last
		// target, and the next setTarget(same) found setpoint_ == target_ and slewed by nothing:
		// measured 2026-08-01 on the 930KV, slew_rpm_s 2000 / 500 / 50 gave IDENTICAL trajectories
		// (peak ~4620 rpm at t~1.2 s), and at slew=50 the command was 792 at t=0.44 s where a slewed
		// setpoint of 22 rpm asks for ~60. Only the first run after a power-on ever slewed.
		setpoint_ = 0.0f;
		i_ = 0.0f; w_ = 0.0f; have_tele_ = false; tele_mech_ = 0.0f; live_ = false; corroborated_ = true;
		d_hat_ = 0.0f; settle_ = 0.0f;
		stale_accum_ = 0.0f; last_sent_ = 0; last_applied_ = 0.0f;
		prev_meas_ = 0.0f; have_prev_meas_ = false; d_filt_ = 0.0f; last_d_ = 0.0f;
		// The thermal DEBOUNCE COUNTER and the clamp flag are loop state, so they clear here with
		// everything else — a counter that outlived the reset clearing every other latch would re-abort
		// a freshly re-armed controller. The temperature itself (last/peak/age/state) deliberately does
		// NOT clear: it is an observation about the hardware, and an arm/disarm does not cool the MCU.
		over_readings_ = 0; temp_clamped_ = false;
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
		                      || (!live_ && profile_->regime(sp) == Regime::SINE))) {
			target_ = pending_; have_pending_ = false;
		}

		// -- telemetry: this frame drives the feedback (temperature is acquired separately, below) --
		bool have_meas = false;
		float mr = 0.0f, tc = 0.0f;
		if (io_.readTele(mr, tc)) {
			if (fabsf(mr) > TELE_MIN_MECH_RPM) {                 // live: re-attach the commanded sign
				float sign = (sp > 0.0f) ? 1.0f : (sp < 0.0f) ? -1.0f : (last_sign_ >= 0.0f ? 1.0f : -1.0f);
				tele_mech_ = sign * fabsf(mr); have_tele_ = true; have_meas = true;
			}
		} else {
			tc = 0.0f;                                           // no frame => no legacy temperature
		}
		live_ = have_meas;

		// -- staged thermal protection. Deliberately NOT on the readTele path any more: temperature has
		//    to be observed even when eRPM is stale (forced sine — the hottest regime). The derate is a
		//    ceiling on the COMMAND (see clampCmd/cmdCeiling), so the limit stays visible in `cmd`
		//    instead of hiding inside the tracking error where the DOB would fight it. --
		if (updateTemp_(tc, dt)) { io_.thrust(0); return Status::ABORT_TEMP; }

		// -- PI authority (w) fades with liveness AND CORROBORATION: 0->1 while a live frame is also
		//    consistent with the plant we think we are driving, 1->0 otherwise, over blend_secs. Reset
		//    the integrator at w==0 so a re-arm from below the seam starts clean.
		//
		//    Corroboration compares the measurement against the profile's forward prediction for the
		//    command we ACTUALLY SENT last tick — the same map the DOB already trusts for d_meas. Two
		//    deliberate escapes: last_sent_ == 0 on the first tick predicts ~0, and any prediction
		//    below TRACK_MIN_PREDICTED_RPM is in the region where the curve's own resolution and the
		//    ESC's telemetry floor dominate. Both mean "no information", and no information must not
		//    withdraw authority — only a prediction we can actually check, and fail, does. --
		corroborated_ = true;
		if (live_ && track_min_frac > 0.0f) {
			float predicted = profile_->rpmFor((float)last_sent_);
			if (fabsf(predicted) > TRACK_MIN_PREDICTED_RPM)
				corroborated_ = fabsf(tele_mech_) >= track_min_frac * fabsf(predicted);
		}
		float rate = (blend_secs > 0.0f) ? dt / blend_secs : 1.0f;
		if (live_ && corroborated_) { w_ += rate; if (w_ > 1.0f) w_ = 1.0f; }
		else                        { w_ -= rate; if (w_ < 0.0f) w_ = 0.0f; }
		if (w_ <= 0.0f) i_ = 0.0f;

		// -- feed-forward + blended PI trim --
		float ff = profile_->thrustFor(sp);
		float trim = closedLoopTrim(sp, dt);
		float applied = w_ * trim;
		// -- disturbance-observer compensation (opt-in, dob>0): the deficit between the profile's forward
		//    prediction for the LAST command and the measured rpm IS the load disturbance; compensate via
		//    the local gain. Two gates, both required (the GATE, not the w_ factor, is what protects this
		//    path — d_hat_ only ever updates at w_ >= 0.9, so the fade's reachable range is 0.9..1.0):
		//      liveness — only SOLIDLY live, i.e. past the sine->6-step catch and its fade transient;
		//      settle   — only once the setpoint slew has (nearly) arrived, so the plant's own rise is
		//                 not read as load. While slewing the estimate is FROZEN rather than cleared, so
		//                 a genuine load learned before the step survives it instead of being re-learned. --
		float comp = 0.0f;
		float gs = updateSettleGate(dt);
		if (dob > 0.0f && live_ && w_ >= DOB_LIVE_FLOOR) {
			if (gs > 0.0f) {
				// FREEZE THE UPDATE, NEVER THE OUTPUT. temp_clamped_ holds LAST tick's verdict, so
				// suppressing `comp` itself made the thermal ceiling self-oscillating: clamped tick ->
				// comp 0 next tick -> command drops below the ceiling -> unclamped -> comp returns ->
				// clamped again. Measured on a 60->80 C ramp with dob=0.8 and a real 100 rpm load, the
				// command square-waved 537<->590 for seconds at the core0 loop rate, flickering the
				// integrator freeze and the ABORT_STALL suppression with it. Holding the last honest
				// estimate in the output keeps the loop continuous while still refusing to LEARN from a
				// deficit we caused ourselves by withholding command.
				if (!temp_clamped_) {
					float d_meas = profile_->rpmFor((float)last_sent_) - tele_mech_;
					float a = (dob_tau > 0.0f) ? dt / dob_tau : 1.0f; if (a > 1.0f) a = 1.0f;
					d_hat_ += a * (d_meas - d_hat_);
				}
				comp = clampf(gs * w_ * dob * d_hat_ / profile_->rpmSlopeAt((float)last_sent_),
				              -dob_max, dob_max);
			}
		} else {
			d_hat_ = 0.0f;   // no wound-up estimate from the catch / forced-sine transient
		}
		// -- ONE BUDGET for all feedback paths (see fb_max). The two clamps above are independent, so
		//    without this the reachable deviation from the feed-forward is their SUM. Bound the SIGNED
		//    total and let the OBSERVER absorb the whole cut: `applied` is already <= trim_max <= the
		//    budget, so the PI is never trimmed here and its back-calculation below sees exactly what
		//    it saw before. Opposing paths sum small and are untouched; dob == 0 makes this a no-op. --
		float budget = fbMax();
		float fb = applied + comp;
		float fbc = clampf(fb, -budget, budget);
		comp += (fbc - fb);
		float cmd = ff + applied + comp;
		int sent = clampCmd(cmd);
		io_.thrust(sent);
		last_sent_ = sent; last_applied_ = applied;
		// Was the THERMAL ceiling what bit this tick? Used by the guards below and by the next tick's
		// observer: while we are the ones withholding command, neither "load" nor "the ESC failed to
		// reach 6-step" is a sound inference. Three conditions, and all three are needed:
		//   a derate is active; the clamp ACTUALLY changed the command (sent != (int)cmd — comparing
		//   |cmd| against the ceiling instead false-positives on the fractional part, e.g. cmd 350.7
		//   with ceiling 350 truncates to 350 and was never clamped); and the request was within the
		//   UNDERATED outer ceiling, because past that the ±tmax/±1000 clamp would have bound anyway and
		//   attributing it to temperature would suppress ABORT_STALL for a genuine stall that merely
		//   coincided with a mild derate.
		temp_clamped_ = (temp_scale_ < 1.0f) && (sent != (int)cmd)
		                && (fabsf(cmd) <= (float)outerCeiling());

		// -- back-calculation anti-windup on the OUTER ±tmax clamp. Only when the trim materially drove
		//    the clamp: skip below a real authority floor, and skip when the FF term alone saturated
		//    (delivered opposes the desired trim) so the integrator can't flip against the true error.
		//    A THERMAL clamp is handled PRIMARILY by freezing the integrator (closedLoopTrim) — not
		//    exclusively: when the trim itself is what the ceiling cut, delivered and desired still share
		//    a sign and this fold runs in the same tick, which is correct (both drive i_ toward what was
		//    actually delivered). What the freeze adds is the case the fold declines: FF alone over the
		//    ceiling, where the fold is skipped and the integrator would otherwise wind to ±trim_max. --
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
		// While the thermal ceiling binds, the ESC may legitimately never reach 6-step BECAUSE WE ASKED
		// IT NOT TO — that is a derate, not a failure to commutate, so the stall accumulator stays clear.
		float floor;
		bool commanding_line = profile_->lineFloor(floor) && fabsf(sp) >= floor
		                       && (float)abs(sent) >= fabsf(ff) - trim_max && !temp_clamped_;
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
	// Did the last live frame CORROBORATE the profile's forward prediction? False = the loop is running
	// open (feed-forward only) because the measurement is not evidence of a plant we are driving.
	bool  corroborated() const { return corroborated_; }
	// The total feedback budget in force this tick (see fb_max).
	float fbMax() const {
		if (fb_max > 0.0f) return fb_max;
		return trim_max > dob_max ? trim_max : dob_max;
	}
	float authority() const { return w_; }               // PI weight 0..1
	float dhat()     const { return d_hat_; }             // DOB load estimate (rpm-equivalent)
	float settled()  const { return settle_; }            // DOB settle gate 0..1 (1 = settled, DOB full)
	float temp()      const { return last_temp_; }        // last ESC MCU temperature seen (C; 0 = never)
	float peakTemp()  const { return peak_temp_; }        // high-water mark since power-on
	float tempLimit() const { return temp_scale_; }       // command-ceiling scale 0..1 (1 = unlimited)
	TempState tempState() const { return temp_state_; }

private:
	// Staged thermal limiter. Returns true when the ABORT stage has fired. Fails OPEN (scale 1.0) but
	// never SILENTLY: "never seen a temperature" and "had one and lost it" are distinct reported states,
	// because a permanent limp mode and a silent absence of protection are the two failure modes here.
	bool updateTemp_(float teleTemp, float dt) {
		// NEWNESS IS DECIDED HERE, not by the adapter: a reading counts once, when its stamp moves. That
		// is what makes TEMP_ABORT_READINGS count EDT frames instead of control ticks — an adapter that
		// simply reports the value it is holding (the obvious implementation) would otherwise turn one
		// garbled frame into an abort in 3 ticks (~15 ms), the exact failure the debounce prevents.
		float tc = 0.0f; uint32_t stamp = 0;
		bool have = io_.readTemp(tc, stamp) && tc > 0.0f;
		if (have) have_readtemp_ = true;
		bool fresh = have && (stamp != temp_stamp_ || !have_temp_);
		// Back-compat: an adapter written against the OLD EscIo contract delivers its temperature in the
		// readTele frame, with no stamp to dedupe on. Honour it only while readTemp has never produced
		// anything, and accept that on that path a reading is per-tick (the pre-B.7 behaviour).
		if (!have && !have_readtemp_ && teleTemp > 0.0f) { tc = teleTemp; fresh = true; }

		if (fresh) {
			last_temp_ = tc; if (tc > peak_temp_) peak_temp_ = tc;
			temp_stamp_ = stamp; temp_age_ = 0.0f; have_temp_ = true;
		} else if (have_temp_) {
			temp_age_ += dt;                   // ages from the last NEW reading, however often we ask
		}

		if (!have_temp_) {                     // never seen one: bit-identical to the limiter disabled
			temp_scale_ = 1.0f; temp_state_ = TempState::NONE; over_readings_ = 0; return false;
		}
		if (temp_age_ > temp_stale_secs) {     // seen, then lost: RELEASE the limit and say so
			temp_scale_ = 1.0f; temp_state_ = TempState::STALE; over_readings_ = 0; return false;
		}
		// -- abort stage: max_temp keeps its original meaning, now debounced over TEMP_ABORT_READINGS
		//    consecutive readings so one garbled EDT frame cannot stop a thruster. --
		if (fresh) {
			if (max_temp > 0.0f && last_temp_ >= max_temp) over_readings_++;
			else                                           over_readings_ = 0;
		}
		// -- derate stage: authority ramps linearly down over [warn_temp, max_temp] to temp_min_scale --
		float scale = 1.0f;
		if (warn_temp > 0.0f && max_temp > warn_temp) {
			if (last_temp_ >= max_temp) scale = temp_min_scale;
			else if (last_temp_ > warn_temp)
				scale = 1.0f - (last_temp_ - warn_temp) / (max_temp - warn_temp) * (1.0f - temp_min_scale);
		}
		temp_scale_ = scale;
		temp_state_ = (scale < 1.0f) ? TempState::DERATE : TempState::OK;
		// Gated on `fresh` so the abort can only fire ON a reading. Without it, a re-armed controller
		// whose counter had reached the threshold would re-abort on its very first tick — before any new
		// EDT frame has had a chance to say the ESC has cooled (and reset() now clears the counter too).
		return fresh && over_readings_ >= TEMP_ABORT_READINGS;
	}

	// DOB settle gate: forced to 0 while the setpoint is still slewing, then faded back to 1 over
	// dob_settle_secs from the moment it ARRIVES. Deliberately keyed on |target - setpoint| and not on
	// the tracking error, so a large steady error never gates the observer off.
	float updateSettleGate(float dt) {
		if (dob_settle_secs <= 0.0f)               settle_ = 1.0f;
		else if (fabsf(target_ - setpoint_) > 0.5f) settle_ = 0.0f;    // slewing: prediction is invalid
		else { settle_ += dt / dob_settle_secs; if (settle_ > 1.0f) settle_ = 1.0f; }
		return settle_;
	}

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
		// Accumulate only against a FRESH live frame — and NOT while the thermal ceiling is what is
		// clamping the command (temp_clamped_ holds the previous tick's verdict). No increase of the
		// trim can be delivered through that ceiling, so integrating there is pure windup: it would
		// saturate i_ at ±trim_max and the release would then be a step (measured: with back-calc alone
		// the integrator parks at -trim_max and takes ~2.8 s to unwind). Frozen, the release is
		// bump-free — the loop resumes at exactly the command it was holding before the derate.
		if (live_ && !temp_clamped_) i_ += ki * err * dt;
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

	// The command-magnitude ceiling with NO derate: the pre-B.7 clamp, min(1000, tmax).
	int outerCeiling() const { int lim = tmax < 1000 ? tmax : 1000; return lim < 0 ? 0 : lim; }
	// ...and the effective one, scaled by the thermal limiter. A CEILING rather than a setpoint scale on
	// purpose — scaling the setpoint would hide the limit inside the tracking error, and the DOB would
	// then read the reduction as load and fight it. With no derate this IS outerCeiling().
	int cmdCeiling() const {
		int lim = outerCeiling();
		return (temp_scale_ >= 1.0f) ? lim : (int)((float)lim * temp_scale_);
	}
	int clampCmd(float cmd) const {
		int v = (int)cmd;                                // truncate toward zero (matches Python int())
		int lim = cmdCeiling();
		if (v >  lim) v =  lim;
		if (v < -lim) v = -lim;
		return v;
	}
	float effectiveOverSpeed() const {
		if (over_speed_rpm > 0.0f) return over_speed_rpm;
		float twice = 2.0f * profile_->maxRpm();
		return twice > 1200.0f ? twice : 1200.0f;
	}
	static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

	EscIo&             io_;
	const SpeedProfile* profile_;

	// --- state (no static/global mutable state) ---
	float target_ = 0.0f, setpoint_ = 0.0f;
	float pending_ = 0.0f; bool have_pending_ = false;
	float i_ = 0.0f, w_ = 0.0f, d_hat_ = 0.0f, settle_ = 0.0f;
	float tele_mech_ = 0.0f; bool have_tele_ = false, live_ = false, corroborated_ = true;
	float prev_meas_ = 0.0f; bool have_prev_meas_ = false; float d_filt_ = 0.0f, last_d_ = 0.0f;
	float last_sign_ = 1.0f;
	float stale_accum_ = 0.0f;
	int   last_sent_ = 0; float last_applied_ = 0.0f;
	float last_temp_ = 0.0f, peak_temp_ = 0.0f;
	// thermal limiter state (NOT cleared by reset(): a temperature is a physical observation about the
	// hardware, not closed-loop state, and an arm/disarm does not cool the MCU down)
	float     temp_scale_ = 1.0f, temp_age_ = 0.0f;
	uint32_t  temp_stamp_ = 0;              // stamp of the last NEW reading (see updateTemp_)
	bool      have_temp_ = false, have_readtemp_ = false, temp_clamped_ = false;
	int       over_readings_ = 0;
	TempState temp_state_ = TempState::NONE;
};

}  // namespace vel
