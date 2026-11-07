// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// thruster — a per-ESC OBJECT you DECLARE (the composition root, usually main.cpp, owns them). This
// is the reusable library primitive: one Thruster per ESC, each carrying its OWN config (DShot
// bitrate, motor pole count, calibrated speed profile, and PI gains) and its own closed-loop
// velocity controller. It delegates the heavy lifting to the proven singleton escs:: engine
// (2 PIO SMs + one core1 1-wire worker) by its index_ — that layer is inherently one-per-board, so a
// Thruster is a lightweight handle + per-ESC config + controller on top.
//
//   main:                                                  ROV (no tool, main drives directly):
//     static Thruster t1(&profiles::M_930KV, 300, 14);       for (auto* t : thrusters) { t->setRpm(mix); t->poll(); }
//     ... t1.bind(1); t1.vc.kp = 0.03f; ...                  escs::spinPoll();
//
// DRIVE has two submodes: RAW (a direct thrust/throttle target escs:: holds; poll() is a no-op) and
// RPM (the closed velocity loop: setRpm() sets the target, poll() runs vc.step(dt) each core0 tick).
#pragma once
#include <Arduino.h>
#include "esc_config.h"
#include "console.h"
#include "esc_session.h"
#include "vel_control.h"
#include "motor_model.h"
#include "sense_gate.h"
#include "profiles.h"

// An rpm whose telemetry stamp is older than this = stale (forced sine / dropout) — the PI authority
// then fades and the controller degrades to pure feed-forward.
static const uint32_t THRUSTER_TELE_FRESH_MS = 100;
// NOTE: there is deliberately NO temperature counterpart to the constant above. Io::readTemp reports the
// last decoded EDT temperature WITH ITS STAMP and lets vel::VelocityController own both edges — newness
// (the stamp moved) and expiry (vel::DEFAULT_TEMP_STALE_SECS since it last moved). Two independently
// tunable 2-second windows for the same quantity, on either side of the EscIo boundary, would have summed
// on the first-read-after-a-gap path and been impossible to reason about from either file alone.

// vbatt (resident battery-voltage estimate): gate the rolling max to solid 6-step (above the crossover,
// where the duty model is accurate), and leak it slowly so it tracks a draining pack.
// [#8] The gate rpm is DERIVED from the active profile (SpeedProfile::lineFloor() = the lowest genuinely
// 6-step-reachable speed), because `motor <i> <kv> <pp> <v>` moves that boundary while a literal would
// stay put — the old 650 sat INSIDE the 350KV's 190..834 no-steady-state gap. These literals remain only
// as the fallback for a profile with no crossover/regime metadata (e.g. the RAW linear default).
static const float THRUSTER_SENSE_RPM_FLOOR = 650.0f;  // fallback only: no lineFloor() available
static const float THRUSTER_VBATT_RPM_FLOOR = 800.0f;  // fallback only
static const float THRUSTER_VBATT_MARGIN    = 1.1f;    // vbatt gates this much above the 6-step landing
static const float THRUSTER_VBATT_DECAY = 2.0e-5f;   // per-poll (~0.001 V/s @ 50 Hz)

class Thruster {
public:
	using Info  = escs::Info;
	using Telem = escs::Telem;
	using Drive = escs::Drive;
	enum Submode { RAW, RPM };

	// Declared by the composition root. profile = the calibrated FF curve for THIS motor (defaults to a
	// trivial linear curve for RAW-only ESCs); dshotKbaud / motorPoles are this ESC's DShot bitrate and
	// pole count (default to the esc_config.h globals). The controller's gains stay at the library
	// DEFAULT_GAINS until the declaring side sets the per-motor values (th.vc.kp = 0.03f; ...).
	explicit Thruster(const vel::SpeedProfile* profile = &profiles::M_LINEAR,
	                  uint16_t dshotKbaud = ESC_DSHOT_KBAUD, uint8_t motorPoles = ESC_MOTOR_POLES)
		: profile_(profile ? profile : &profiles::M_LINEAR),
		  vc(io_, *profile_), kbaud_(dshotKbaud), poles_(motorPoles) {
		io_.owner = this;
	}

	// Attach this object to an escs:: index (its wired pin = escs::PINS[index], the pin list) and push its
	// per-ESC DShot config into the engine. Call once, before arming.
	void bind(uint8_t index) {
		index_ = index;
		escs::setKbaud(index_, kbaud_);
		escs::setPoles(index_, poles_);
	}
	uint8_t index() const { return index_; }

	// ---- config / flash: pure pass-throughs to escs:: by this ESC's index ----
	bool scan(Info& out)                                { return escs::scan(index_, out); }
	bool connect(Info& out)                             { return escs::connect(index_, out); }
	bool readConfig(uint8_t* out255)                    { return escs::readConfig(index_, out255); }
	int  editConfig(const uint16_t* offs, const uint8_t* vals, int n, bool& changed) {
		return escs::editConfig(index_, offs, vals, n, changed);
	}
	bool erasePage(uint16_t addr)                       { return escs::erasePage(index_, addr); }
	bool writeFlash(uint16_t addr, const uint8_t* d, uint16_t len) { return escs::writeFlash(index_, addr, d, len); }
	bool readFlash(uint16_t addr, uint8_t* out, uint16_t len)      { return escs::readFlash(index_, addr, out, len); }
	void release()                                      { escs::release(); }

	// ---- drive ----
	// [#5] arm() clears BOTH soft-sensor baselines: the load reference (senseVref_) and the vbatt rolling
	// max. The max only ever decays at THRUSTER_VBATT_DECAY (~0.001 V/s), so a swap to a MORE DEPLETED
	// pack would otherwise keep reading the old, higher pack for ~27 min of running — high, which is the
	// wrong direction for the one decision this number exists to inform. The calibration SCALE is a model
	// constant (persisted by `cfg save`) and is deliberately kept.
	// false = refused (FAILED: scan or a configurator read reads it again) or no free PIO SM (initOk()).
	bool arm(Drive mode = Drive::AUTO) {
		bool ok = escs::spinArm(index_, mode); vc.reset(); submode_ = RAW;
		senseVref_ = 0.0f; vbattMax_ = 0.0f; senseZeroCmd_ = 0.0f;
		gate_.reset();   // dwell, hysteresis latch and both tiers: an arm ends the condition under which
		                 // anything measured before it is comparable to anything measured after.
		return ok;
	}
	// RAW target: signed thrust on a reversible (3D) ESC, else unidirectional throttle, by the identity
	// read from the ESC (escs::drive). A negative value on a one-way ESC is refused (REVERSE_ONEWAY),
	// never cast: (uint16_t)-300 used to clamp to full throttle. Setting a RAW
	// target disengages the rpm loop (submode_ -> RAW) so a stray step() can't fight it.
	// [B.6-fix] ...and that disengagement must close the soft-sensor with it: poll() returns early at
	// submode_ != RPM, so a latched gate would otherwise survive `rpm 1 1600` -> `thrust 1 300` and
	// `sense` would report a minutes-old vest as valid=1|why=ok while the motor runs at an arbitrary
	// RAW throttle — and `sense zero` would succeed there, recording a stale command as the off-point
	// centre and poisoning every later `load`. Same reasoning as stop(), same fix.
	lc::Refuse setRaw(int v) {
		submode_ = RAW;
		gate_.reset();
		return escs::drive(index_, v);
	}
	// RPM target: engage the closed loop. Reset the step clock so the first dt is sane.
	void setRpm(float rpm) { submode_ = RPM; vc.setTarget(rpm); lastStepUs_ = micros(); }
	// RETARGET WITHOUT RESETTING THE STEP CLOCK. setRpm() deliberately reseeds lastStepUs_ so a fresh
	// engagement starts with a sane dt — but a caller that retargets at LOOP RATE (the azimuth loop
	// does, at ~1 kHz) reseeds it immediately before poll() reads it, so dt comes out ~0 on every
	// tick and the velocity loop's integral and derivative silently contribute nothing. Measured
	// 2026-08-14: the two motors then wandered independently under a weak proportional term alone,
	// and the differential turned that wander straight into azimuth motion the position loop could
	// not counter at any gain. Retarget through here; engage through setRpm().
	void setRpmTarget(float rpm) {
		if (submode_ != RPM) { submode_ = RPM; lastStepUs_ = micros(); }
		vc.setTarget(rpm);
	}
	// Stopping ends the only condition under which the soft-sensor is meaningful, so the validity flag
	// has to fall with it: poll() returns early when not in RPM mode, so the gate would otherwise keep
	// its last value and `sense` would report a stale vest as valid on a motionless motor.
	void stop()   { escs::spinStop(index_); submode_ = RAW; gate_.reset(); }
	void disarm() { stop(); }

	// Copy a profile's calibrated PI gains onto the controller (e.g. the generated M_<NAME>_GAINS).
	// The declaring side may still override any field afterward (th.vc.slew_rpm_s = ...).
	void applyGains(const vel::Gains& g) { vc.kp = g.kp; vc.ki = g.ki; vc.trim_max = g.trim_max; vc.blend_secs = g.blend_secs; }

	// Regenerate the feed-forward curve PARAMETRICALLY from motor KV + pole-pairs + supply V (the
	// parametric vel::MotorModel) and switch the controller onto it — the runtime `motor <i> <kv> <pp>
	// <v>` path. No recompile, no bench sweep: KV+PP+V predict the curve (RMSE ~12 rpm), and an eRPM
	// self-cal can refine it later. The owned ffprof_/ffbuf_ outlive vc, so setProfile is safe.
	// crossUp = the ESC's `sine_cross_up`. It shifts the 6-step duty by a constant (cross_rescale_duty
	// maps the demand relative to Cross_Up), so the parametric curve is only right at the Cross_Up its
	// shape constants were fitted at. 0 keeps whatever is already set (default = the fit reference).
	// Returns false when no parametric curve could be generated for this configuration (the handoff is
	// refused, or it lands above the 700 the curve is sampled to — see MotorModel::fillProfile). The
	// PREVIOUS curve is then kept, exactly as a rejected `curve` upload keeps it, but the seam metadata
	// is still updated: see below. Both existing call sites (src/apps/esc_tool_app.h `motor`,
	// src/apps/settings.h apply()) call this as a statement, so the void -> bool change is source
	// compatible; the CLI one now reports the failure instead of printing `ok`.
	bool setMotor(float kv, int polePairs, float vSupply, float crossUp = 0.0f) {
		mm_.set(kv, polePairs, vSupply);
		mm_.setCrossUp(crossUp > 0.0f ? crossUp : mm_.crossUp());   // set() must not clear it
		// [#8] tag each generated point's regime so the profile's lineFloor() reports the real 6-step
		// LANDING (~sixstepFloorRpm) rather than the seam — the soft-sensor gates below key off it.
		int n = mm_.fillProfile(ffbuf_, (int)(sizeof(ffbuf_) / sizeof(ffbuf_[0])), 700.0f, 4, ffreg_);
		// [B.6-fix] The seam is DERIVED from the configured cross_up, not pinned to the firmware
		// minimum. It used to be hardcoded to HANDOFF_ERPM here, which made crossUpErpm() — the source
		// the soft-sensor gate's command floor comes from — stop tracking `crossup <i> <byte>`
		// entirely: at cross_up 36 the real seam is 1406 eRPM (handoff command 563) while the gate
		// still opened at 560, so the advertised +5 % margin was really -0.5 %; at cross_up 50 the gate
		// would have latched valid across a ~220-count band that is ENTIRELY forced sine — exactly the
		// synthesised-telemetry regime B.6 exists to exclude, reachable through a persisted CLI command.
		//
		// THIS ASSIGNMENT MUST STAY ABOVE THE `n < 2` BAIL-OUT. The seam is a property of cross_up
		// alone, not of whether a curve came out: once fillProfile started returning 0 for a cross_up
		// whose handoff is above the sampled range (>= 48, which includes the 50 the 2026-07-31 bench
		// ran), an early return left ffcx_ on the PREVIOUS seam while mm_ moved — and poll()'s
		// `gate_.configure(crossUpErpm(), ...)` would then have opened the soft sensor at
		// handoffCmdFor(1333)*1.05 = 510 against a real handoff of 738, i.e. the 228-count forced-sine
		// band this comment exists to exclude, with duty6(510, cross_up 50) = -0.56 feeding estimateV.
		// The live profile holds `&ffcx_`, so writing it here updates the installed curve's metadata in
		// place — the same mechanism setCrossUp() below relies on.
		//
		// The `n >= 2 ||` half preserves the MEASURED-curve exemption: on the success path the seam is
		// always overwritten (the parametric curve replaces any uploaded one), but when nothing was
		// installed and the live curve is a measured one, `curve commit <pp> <up> <dn>`'s observed seam
		// outranks this arithmetic — the identical rule setCrossUp() states.
		if (n >= 2 || !curveMeasured_) ffcx_ = vel::Crossover{mm_.crossUpErpm(), mm_.crossUpErpm()};
		if (n < 2) {
			// Nothing was installed, so whatever curve is live was NOT generated for the model that is
			// now in mm_. Record it: the `crossup` reply is generated after this returns and would
			// otherwise report a regeneration that did not happen. A MEASURED curve is not stale — it
			// is not a function of mm_ at all — which is the same exemption used two lines above.
			curveStale_ = !curveMeasured_;
			return false;
		}
		installCurve_(n, polePairs);
		motorSet_ = true;
		curveMeasured_ = false;   // the parametric prediction REPLACES any uploaded curve
		curveStale_ = false;
		return true;
	}

	// Install a MEASURED feed-forward curve (host/curvepush.py -> the `curve` command), replacing the
	// parametric prediction. This is what unlocks the DOB: the observer inverts the forward map, so with
	// the parametric curve — which over-predicts — it reads model error as load and has to be clamped.
	// A velcal/sysid curve makes the deficit genuine load. Points must be monotone (thrust strictly
	// increasing, rpm non-decreasing) exactly as SpeedProfile requires; on any violation the CURRENT
	// curve is kept and false is returned, so a bad upload can never leave a half-written table live.
	// regimes may be null -> derived from the crossover (a measured curve has no points inside the gap,
	// so the first point above the seam IS the 6-step landing, which is what lineFloor() wants).
	bool setCurve(const vel::CurvePoint* pts, int n, int polePairs, float upErpm, float dnErpm,
	              const vel::Regime* regimes = nullptr) {
		const int cap = (int)(sizeof(ffbuf_) / sizeof(ffbuf_[0]));
		if (n > cap || polePairs < 1 || upErpm <= 0.0f) return false;
		if (!vel::validateCurve(pts, n)) return false;
		for (int i = 0; i < n; i++) ffbuf_[i] = pts[i];
		vel::tagRegimes(pts, n, polePairs, upErpm, regimes, ffreg_);
		ffcx_ = vel::Crossover{upErpm, dnErpm};
		installCurve_(n, polePairs);
		curveMeasured_ = true;
		curveStale_ = false;      // a MEASURED curve is not a function of mm_, so it cannot go stale
		return true;
	}

	// Motor identity as last configured by `motor` — what `cfg save` persists (nothing to persist until
	// the operator has actually set it; the compile-time profile is the fallback).
	bool  motorConfigured() const { return motorSet_; }
	float motorKv()         const { return mm_.kv(); }
	int   motorPolePairs()  const { return mm_.polePairs(); }
	float motorVolts()      const { return mm_.voltage(); }
	float motorCrossUp()    const { return mm_.crossUp(); }

	// Set the ESC's `sine_cross_up` WITHOUT touching the feed-forward source. Needed because it stays
	// relevant while a MEASURED curve is installed: the curve supplies rpmFor/thrustFor, but the
	// soft-sensor (estimateV/estimateLoad) still inverts the parametric duty map, so a wrong cross_up
	// keeps biasing `vest`/`load` even when the FF itself is measured. Routing this through `motor`
	// would have forced the operator to destroy the measured curve just to correct a soft-sensor input.
	// When the PARAMETRIC curve is the live one, regenerate it — there cross_up does move the FF.
	// The return value means "the BYTE was in range" and nothing else — deliberately NOT overloaded
	// with "and a curve came out of it", because a caller has to be able to tell those apart: the byte
	// really has been applied to mm_ (and so to the soft-sensor's duty map) even when the parametric
	// curve could not be regenerated. Read curveStale() for the second question; setMotor() maintains
	// it across this call. Without that, `crossup <i> 50` reported "parametric (regenerated)" while the
	// live curve was still the cross_up-36 one — whose thrustFor(1500) is 643, BELOW the 738 handoff at
	// cross_up 50, so the loop could never reach 6-step and would forced-sine until ABORT_STALL.
	bool setCrossUp(float crossUp) {
		if (crossUp < 1.0f || crossUp > 255.0f) return false;
		mm_.setCrossUp(crossUp);
		if (motorSet_ && !curveMeasured_) setMotor(mm_.kv(), mm_.polePairs(), mm_.voltage(), crossUp);
		// [B.6-fix] cross_up MOVES THE SEAM as well as shifting the duty, so the seam metadata has to
		// follow it even when there is no parametric curve to regenerate — it is what the soft-sensor's
		// command floor is derived from. A MEASURED curve is exempt: `curve commit <pp> <up> <dn>`
		// supplies a seam that was actually observed, and that measurement outranks this arithmetic.
		else if (!curveMeasured_) ffcx_ = vel::Crossover{mm_.crossUpErpm(), mm_.crossUpErpm()};
		return true;
	}
	float vbattScale()      const { return vbattScale_; }
	void  setVbattScale(float s)  { if (s > 0.0f) vbattScale_ = s; }
	// The live feed-forward curve, for `curve <i>` readback and `cfg save`. curveMeasured() is false
	// while the curve is the parametric prediction (which is regenerable from kv/pp/v, so it is NOT
	// persisted point-by-point).
	bool  curveMeasured() const { return curveMeasured_; }
	// True when the live PARAMETRIC curve was NOT regenerated for the model currently in mm_, because
	// fillProfile() had nothing to emit at this cross_up (its handoff is refused, or above the cmd 700
	// the curve is sampled to => cross_up >= 48). The FF then predicts speeds for a cross_up the ESC is
	// no longer on, which is a plausible-looking wrong answer rather than an obvious one — so `crossup`
	// and `motor` report it instead of claiming a fresh curve. Always false while a measured curve is
	// live, and cleared by any successful setMotor()/setCurve().
	bool  curveStale()    const { return curveStale_; }
	int   curveCount()    const { return curveMeasured_ ? ffn_ : 0; }
	// [#11-1] The pole count the LIVE CURVE was installed with — NOT mm_'s. `curve commit <pp>` sets the
	// profile's pp without touching the parametric model, so the two can differ; persisting or reporting
	// mm_'s would restore the curve under the wrong pp after a reset, mis-scaling both the tele mech-rpm
	// (poles_) and the regime test (rpm*pp >= up_erpm) that lineFloor() — hence the soft-sensor gate —
	// depends on. Read it from the profile, which is the single source of truth for the installed curve.
	int   curvePolePairs() const { return vc.profile().polePairs(); }
	// Bounds-clamped: these are public and the natural loop bound (curveCount()) is a separate call,
	// so an off-by-one at a call site should not read past the buffer.
	const vel::CurvePoint& curvePoint(int i) const {
		return ffbuf_[i < 0 ? 0 : (i >= (int)(sizeof(ffbuf_)/sizeof(ffbuf_[0]))
		                           ? (int)(sizeof(ffbuf_)/sizeof(ffbuf_[0])) - 1 : i)];
	}
	vel::Regime curveRegime(int i) const {
		return ffreg_[i < 0 ? 0 : (i >= (int)(sizeof(ffreg_)/sizeof(ffreg_[0]))
		                           ? (int)(sizeof(ffreg_)/sizeof(ffreg_[0])) - 1 : i)];
	}
	float crossUpErpm()   const { return ffcx_.up_erpm; }
	float crossDnErpm()   const { return ffcx_.dn_erpm; }

	// ESC perception (soft-sensor): infer effective supply voltage + a relative load signal from the
	// LIVE command + measured 6-step tele rpm — no voltage/current/force sensor. senseVoltage ~= battery
	// V at light load; senseLoad = (Vref - Vest) grows with current => torque => thrust (relative). 0
	// outside 6-step (needs live BEMF tele) — that 0 means NO READING, not zero volts; see the *Has()
	// predicates below. Set the motor first with setMotor()/`motor`.
	float senseVoltage() { return gate_.valid() ? vestFilt_ : 0.0f; }       // gated + low-passed
	// [#7.1] load = deviation from the LEARNED light-load reference (0 until it settles), not the typed V.
	// [B.6] ...and only while we are still NEAR the operating point that reference was captured at: the
	// residual no-load I*R/duty deficit is operating-point dependent (it flips worst-case between air and
	// water), so a difference taken somewhere else is two different biases subtracted, not a load.
	float senseLoad()    { return (gate_.loadOk() && senseVref_ > 0.0f) ? (senseVref_ - vestFilt_) : 0.0f; }
	float senseVref()    { return senseVref_; }                             // learned no-load V_eff (0 = not yet)
	// [B.8] "is there a reading at all?" for the three fields above, so a caller can print n/a instead
	// of the 0.0f they fall back to. Each of those zeros is ALSO a plausible measurement — 0 V is a dead
	// supply, 0 V of droop is an unloaded prop — so on the wire the absence and the measurement are the
	// same token. That cost a misread on the bench (2026-08-18; vbatt, same shape). The accessors keep
	// returning 0.0f because a numeric API needs a number; it is the PRINTER that must not lie, so
	// anything formatting these for a human or a log pairs them with the matching predicate here.
	// vest's predicate is senseValid() itself, already public above — no second name for one condition.
	bool  senseVrefHas() const { return senseVref_ > 0.0f; }                // `sense <i> zero` has run
	bool  senseLoadHas() const { return gate_.loadOk() && senseVref_ > 0.0f; }
	// Re-baseline the reference on command. Refused unless the gate is valid — a reference captured on an
	// invalid estimate poisons every `load` reading taken against it. Records the COMMAND it was captured
	// at, which is what the off-point band is measured against afterwards.
	bool  senseZero()    {
		if (!gate_.valid()) return false;
		senseVref_ = vestFilt_; senseZeroCmd_ = fabsf((float)vc.command());
		return true;
	}
	float senseZeroCmd() const { return senseZeroCmd_; }
	bool  senseValid()   { return gate_.valid(); }                          // false = not BEMF-live 6-step
	// [B.6] the three-flag split, and WHY the gate is closed when it is. `load` must survive load (that is
	// the sensor's primary product), so the load conditions are separate flags rather than terms of valid.
	bool  senseLoadOk()  const { return gate_.loadOk(); }                   // comparable to the zero point
	bool  senseVbattOk() const { return gate_.vbattOk(); }                  // light-load subset (vcal tier)
	const char* senseWhy() const { return gate_.whyName(); }
	float senseCmdFloor() const { return gate_.cmdFloor(); }                // command needed to OPEN the gate
	// [#8] The |rpm| gates above which the soft-sensor is trusted, DERIVED from the ACTIVE profile — its
	// lowest genuinely 6-step-reachable speed — so `motor <kv> <pp> <v>` (which moves that boundary) can't
	// leave a literal behind. vbatt sits a margin higher: its rolling max wants the well-modelled,
	// high-duty end. senseRpmFloor() is reported by `sense` so the gate is inspectable.
	float senseRpmFloor() const { float lf = lineFloor_(); return lf > 0.0f ? lf : THRUSTER_SENSE_RPM_FLOOR; }
	float vbattRpmFloor() const {
		float lf = lineFloor_();
		return lf > 0.0f ? lf * THRUSTER_VBATT_MARGIN : THRUSTER_VBATT_RPM_FLOOR;
	}
	// [vbatt] resident battery-voltage estimate = scale * gated rolling-max of vest (~= supply). Per-motor
	// calibration: senseVcal(known_battery_V) sets the scale so the absolute value tracks. The
	// scale is a MOTOR/model constant so it survives arms and is persisted by `cfg save`; the rolling max
	// itself is per-session (arm() clears it — see #5 there). Returns false when there is no estimate to
	// calibrate against yet, so the caller can report it instead of silently no-op'ing.
	float senseVbatt()   { return vbattScale_ * vbattMax_; }
	// [B.8] "is there an estimate at all?", so a caller can print n/a instead of a NUMBER. An ungated
	// vbatt reads exactly 0.00, and 0.00 is a plausible measurement — a dead supply — so the two are
	// indistinguishable on the wire. That misread happened on the bench (2026-08-18): the motors were
	// not spinning, vbatt was 0.00 for that reason alone, and it was quoted as evidence the supply was
	// off. valid=0/vbok=0 said so in the same line and were not read. The fix is to make the absent
	// case UNPRINTABLE as a number rather than to document it harder.
	bool  senseVbattHas() const { return vbattMax_ > 0.1f; }
	// [B.6] vbattMax_ only ever advances while the gate's LIGHT-LOAD tier is open (see poll()), so "there
	// is an estimate" now also means "it was taken where vcal's multiplicative scale can actually cancel
	// the residual deficit". Refusing here is what keeps that true for the stored scale.
	bool  senseVcal(float knownV) {
		if (!senseVbattHas()) return false;
		vbattScale_ = knownV / vbattMax_;
		return true;
	}

	bool        armed()      { return escs::spinArmed(index_); }
	bool        reversible() { return escs::spinReversible(index_); }
	const char* spinMode()   { return escs::spinMode(index_); }
	bool        initOk()     { return escs::spinInitOk(index_); }
	bool        tele(Telem& out) { return escs::spinTele(index_, out); }
	Submode     submode() const { return submode_; }

	// Called every core0 loop. Only RPM submode does work: run one closed-loop tick with the REAL
	// elapsed dt (faster than 50 Hz). On a non-OK status the loop aborted (over-speed / stall /
	// over-temp) — stop the motor and fall back to RAW. Callers still run the shared escs::spinPoll().
	void poll() {
		if (submode_ != RPM || !armed()) return;
		// STOP request (|target| <= stop_below_rpm): hold the PROPER NEUTRAL while staying ARMED. A
		// throttle-0 command sets the firmware's Flag_Rcp_Stop; with the BlueGill sine loop now exiting on
		// Rcp_Stop (SineMode.asm), the ESC leaves the drive loop to wait_for_start (ARMED idle, signal
		// still flowing) so the rotor coasts to a real stop instead of limping at the low sine floor.
		// Keep streaming 0 (don't cut the signal, don't disarm) -> the next non-zero target restarts the
		// motor immediately from wait_for_start, with NO re-arm. Reset the integrator so restart is bump-free.
		if (fabsf(vc.target()) <= vc.stop_below_rpm) {
			escs::drive(index_, 0);        // neutral: keep armed + signal alive; firmware stops via Rcp_Stop
			vc.reset();
			// [B.6-fix] The gate has to fall with the controller. This branch RETURNS before the
			// soft-sensor update below, so a gate latched at speed would stay valid=1 with a frozen
			// vest/load for as long as the thruster is held at zero-but-armed. And it is not even
			// coasting: this PR's own coastdown evidence is 3100 rpm -> dead stop in 0.19-0.54 s,
			// because zero throttle BRAKES. Mirrors stop(); keeps the promise made at senseValid().
			gate_.reset();
			lastStepUs_ = micros();
			return;
		}
		uint32_t now = micros();
		float dt = (now - lastStepUs_) * 1e-6f;
		lastStepUs_ = now;
		vel::Status st = vc.step(dt);
		if (st != vel::Status::OK) {
			stop();
			console::out().printf("# ESC %u RPM abort status=%d (1=overspeed 2=stall 3=temp) — stopped\n",
			              index_, (int)st);
			return;   // stop() already reset the gate; do NOT re-open it on a dead motor's last frame
		}
		// SOFT-SENSOR update, decided by vel::SenseGate (lib/vel_control/sense_gate.h). The gate keys on
		// the COMMAND we chose rather than on `tele`, because below the seam the ESC synthesises a virtual
		// eRPM: the 2026-07-30 bench run saw a constant 182 rpm reported for every command from 60 to 450
		// while the rotor went 39 -> 170. The old test (`vc.live() && |measured| >= senseRpmFloor()`) read
		// BOTH its terms from that fiction and so could not detect its own invalidity.
		// Configured every tick rather than cached at install time: the seam and the 6-step landing are
		// properties of the LIVE profile, and `curve commit <pp>` / `cfg load` can move either without
		// passing through the same code path (the class of divergence PR #11 found).
		gate_.configure(crossUpErpm(), senseRpmFloor());
		float cmd = (float)vc.command();
		float ve  = mm_.estimateV(cmd, vc.measured());
		// The droop handed to the gate is the INSTANTANEOUS one (raw estimate, not the filter): the vbatt
		// tier is a per-sample light-load selector, so it has to judge THIS sample, not a history.
		float droop = (senseVref_ > 0.0f) ? senseVref_ - ve : 0.0f;
		// The off-point band is in DUTY (see SENSE_OFFPOINT_DUTY), so the zero point goes through the same
		// duty map as the live command. Both shift by the same constant if cross_up changes, which makes
		// the comparison invariant to it — a command-unit band was not.
		float zeroDuty = (senseZeroCmd_ > 0.0f) ? mm_.duty6(senseZeroCmd_) : 0.0f;
		gate_.update(vc.live(), cmd, vc.measured(), mm_.duty6(cmd), droop, senseVref_, zeroDuty, dt);
		if (gate_.inWindow()) {
			// Filter DURING the dwell (not only once valid) so the first published sample is already
			// settled; but RE-SEED on a fresh entry — continuing a filter whose history came from a
			// different, invalid operating point is how a stale estimate survives into a valid window.
			vestFilt_ = gate_.freshEntry() ? ve : vestFilt_ + 0.12f * (ve - vestFilt_);
			// [vbatt] gated rolling-MAX of the estimate ~= supply voltage: V_eff <= V_supply, so the
			// lightest-load / highest-duty sample approaches the true supply. Gated on the gate's
			// LIGHT-LOAD tier (vbattOk) and, as before, a margin ABOVE the 6-step landing (where duty is
			// well modelled). Leaky so it TRACKS a draining battery (slow decay) while rejecting transient
			// load droops (which recover before the decay bites).
			if (gate_.vbattOk() && fabsf(vc.measured()) >= vbattRpmFloor()) {
				if (vestFilt_ > vbattMax_) vbattMax_ = vestFilt_;
				else                       vbattMax_ -= THRUSTER_VBATT_DECAY;
			}
		}
		// [#7.1] The no-load REFERENCE is captured on command via `sense <i> zero` (senseZero()) at a
		// known steady light-load operating point — NOT the typed supply V (which read ~-2.2 V under
		// no load and needed the operator to measure the battery). load stays 0 until baselined.
	}

	// ---- io adapter: the ONLY place that knows escs:: telemetry -> vel::EscIo. Reads owner->index_. --
	struct Io : public vel::EscIo {
		Thruster* owner = nullptr;
		// REVERSIBLE (3D) ESCs ONLY: cmd is signed 3D thrust (escs::drive maps it by identity), and
		// the bench profiles are calibrated that way. The CLI and HTTP refuse `rpm` on a one-way ESC;
		// should a loop run on one anyway, it is sent 0 (on a one-way ESC a drive value is throttle).
		void thrust(int cmd) override { escs::drive(owner->index_, owner->reversible() ? cmd : 0); }
		bool readTele(float& mechRpm, float& tempC) override {
			escs::Telem t;
			if (!escs::spinTele(owner->index_, t) || !t.valid) return false;
			if (millis() - t.rpmStampMs > THRUSTER_TELE_FRESH_MS) return false;   // stale eRPM -> sine
			if (t.rpm == 0) return false;                                        // no live 6-step sample
			mechRpm = (float)t.rpm;                                              // ALREADY mechanical (fw /pp)
			tempC   = (float)t.tempC;
			return true;
		}
		// Temperature, INDEPENDENT of eRPM freshness — forced sine has stale eRPM and is the hottest
		// regime, so this must not ride on the rpm window. STATELESS by contract: just report the last
		// decoded value and the stamp of the frame it came from; the controller decides what counts as a
		// new reading and when to age it out (see EscIo::readTemp and updateTemp_), so the debounce
		// counts EDT FRAMES rather than control ticks without dedupe state living out here.
		// `t.valid` is deliberately NOT required: it is set by the ERPM decode alone, which is exactly
		// what this must be independent of.
		bool readTemp(float& tempC, uint32_t& stampMs) override {
			escs::Telem t;
			if (!escs::spinTele(owner->index_, t)) return false;
			if (t.tempStampMs == 0) return false;      // no EDT TEMPERATURE frame ever (pre-arm / EDT off)
			tempC = (float)t.tempC;
			stampMs = t.tempStampMs;
			return true;
		}
	};

	// Member order matters: profile_ and io_ construct before vc (which binds references to them).
	const vel::SpeedProfile* profile_;
	Io                       io_;
	vel::VelocityController  vc;   // PUBLIC so the declaring side sets gains: th.vc.kp = 0.03f;

private:
	// Shared tail of setMotor()/setCurve(): ffbuf_/ffreg_/ffcx_ are already populated; publish them.
	void installCurve_(int n, int polePairs) {
		// [#6.1] keep the engine's pole count in sync with the new curve — tele mech-rpm is eRPM/pp,
		// so a differing pp (the whole point of `motor`) would otherwise scale the feedback wrong.
		poles_ = (uint8_t)(polePairs * 2);
		escs::setPoles(index_, poles_);
		// [#6.3] reconstruct WITH the crossover metadata (&ffcx_) so hasCrossover()/lineFloor() stay
		// live — otherwise the "commanded 6-step but tele never went live" ABORT_STALL goes inert.
		ffprof_ = vel::SpeedProfile(ffbuf_, n, polePairs, &ffcx_, ffreg_);
		vc.setProfile(ffprof_);
		ffn_ = n;
	}

	// The active profile's 6-step landing, or 0 when it carries no regime tags to trust. Requires the
	// TAGS, not merely a crossover: untagged, lineFloor() degrades to the seam eRPM = the BOTTOM of the
	// no-steady-state gap, which is the unsafe direction for a soft-sensor validity gate.
	float lineFloor_() const {
		const vel::SpeedProfile& p = vc.profile();
		float lf = 0.0f;
		if (p.hasRegimes() && p.lineFloor(lf) && lf > 0.0f) return lf;
		return 0.0f;
	}

	uint16_t kbaud_;
	uint8_t  poles_;
	uint8_t  index_      = 0;
	Submode  submode_    = RAW;
	uint32_t lastStepUs_ = 0;
	// Owned FF curve for the parametric `motor` path: MotorModel::fillProfile writes ffbuf_, ffprof_
	// references it, and vc.setProfile(ffprof_) swaps the controller onto it. Untouched until setMotor.
	vel::CurvePoint   ffbuf_[24] = {{0.0f, 0.0f}, {1.0f, 0.1f}};  // 2-pt stub until setMotor (no OOB read)
	// FF regime metadata attached to ffprof_ so hasCrossover()/lineFloor() (=> ABORT_STALL) stay live
	// once `motor` swaps the curve in, AND the seam the soft-sensor's command floor is derived from.
	// The initialiser is the firmware handoff floor, which is exactly what mm_'s default cross_up (34,
	// the duty-fit reference) resolves to — setMotor()/setCrossUp() then keep it tracking the configured
	// byte, and `curve commit` replaces it with a MEASURED seam. Static init cannot call mm_, hence the
	// literal; it must stay equal to MotorModel{...}.crossUpErpm() at DUTY6_CROSSUP_REF.
	vel::Crossover    ffcx_{vel::HANDOFF_ERPM, vel::HANDOFF_ERPM};
	// [#8] per-point regime tags (filled by MotorModel::fillProfile) so lineFloor() reports the real
	// 6-step landing instead of the seam. The stub's two points are both below it => SINE.
	vel::Regime       ffreg_[24] = {vel::Regime::SINE, vel::Regime::SINE};
	vel::SpeedProfile ffprof_{ffbuf_, 2, 7, &ffcx_, ffreg_};
	vel::MotorModel   mm_{350.0f, 7, 11.1f};   // parametric model (KV/PP/V) for FF + the soft-sensor
	int               ffn_ = 0;                // live point count in ffbuf_ (0 = still the compile-time profile)
	bool              motorSet_ = false;       // has `motor`/setMotor run? (else mm_ holds only defaults)
	bool              curveMeasured_ = false;  // is ffbuf_ a MEASURED curve (`curve`) rather than parametric?
	bool              curveStale_ = false;     // parametric curve not regenerated for the live mm_ (see curveStale())
	float             vestFilt_ = 0.0f;        // low-passed voltage estimate (soft-sensor)
	// [B.6] the soft-sensor's validity window — command-derived, hysteretic, dwelled. Owns what used to
	// be the inline `senseValid_` test in poll(); reset by arm()/stop().
	vel::SenseGate    gate_;
	float             senseVref_ = 0.0f;       // [#7.1] no-load V_eff reference (0 = not baselined; set via `sense zero`)
	float             senseZeroCmd_ = 0.0f;    // [B.6] |command| senseVref_ was captured at (0 = never)
	float             vbattMax_ = 0.0f;        // [vbatt] gated rolling-max of vest ~= supply V (cleared by arm(), #5)
	float             vbattScale_ = 1.0f;      // [vbatt] calibration scale (senseVcal; persisted by `cfg save`)
};
