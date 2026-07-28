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
#include "esc_session.h"
#include "vel_control.h"
#include "motor_model.h"
#include "profiles.h"

// An rpm whose telemetry stamp is older than this = stale (forced sine / dropout) — the PI authority
// then fades and the controller degrades to pure feed-forward.
static const uint32_t THRUSTER_TELE_FRESH_MS = 100;

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

	// Attach this object to an escs:: index (its wired pin = ESC_SIGNAL_PINS[index]) and push its
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
	void arm(Drive mode = Drive::AUTO) {
		escs::spinArm(index_, mode); vc.reset(); submode_ = RAW;
		senseVref_ = 0.0f; vbattMax_ = 0.0f; senseValid_ = false;
	}
	// RAW target: signed thrust on a reversible (3D) ESC, else unidirectional throttle. Setting a RAW
	// target disengages the rpm loop (submode_ -> RAW) so a stray step() can't fight it.
	void setRaw(int v) {
		submode_ = RAW;
		if (reversible()) escs::spinThrust(index_, (int16_t)v);
		else              escs::spinThrottle(index_, (uint16_t)v);
	}
	// RPM target: engage the closed loop. Reset the step clock so the first dt is sane.
	void setRpm(float rpm) { submode_ = RPM; vc.setTarget(rpm); lastStepUs_ = micros(); }
	// Stopping ends the only condition under which the soft-sensor is meaningful, so the validity flag
	// has to fall with it: poll() returns early when not in RPM mode, so senseValid_ would otherwise keep
	// its last value and `sense` would report a stale vest as valid on a motionless motor.
	void stop()   { escs::spinStop(index_); submode_ = RAW; senseValid_ = false; }
	void disarm() { stop(); }

	// Copy a profile's calibrated PI gains onto the controller (e.g. the generated M_<NAME>_GAINS).
	// The declaring side may still override any field afterward (th.vc.slew_rpm_s = ...).
	void applyGains(const vel::Gains& g) { vc.kp = g.kp; vc.ki = g.ki; vc.trim_max = g.trim_max; vc.blend_secs = g.blend_secs; }

	// Regenerate the feed-forward curve PARAMETRICALLY from motor KV + pole-pairs + supply V (the
	// parametric vel::MotorModel) and switch the controller onto it — the runtime `motor <i> <kv> <pp>
	// <v>` path. No recompile, no bench sweep: KV+PP+V predict the curve (RMSE ~12 rpm), and an eRPM
	// self-cal can refine it later. The owned ffprof_/ffbuf_ outlive vc, so setProfile is safe.
	void setMotor(float kv, int polePairs, float vSupply) {
		mm_.set(kv, polePairs, vSupply);
		// [#8] tag each generated point's regime so the profile's lineFloor() reports the real 6-step
		// LANDING (~sixstepFloorRpm) rather than the seam — the soft-sensor gates below key off it.
		int n = mm_.fillProfile(ffbuf_, (int)(sizeof(ffbuf_) / sizeof(ffbuf_[0])), 700.0f, 4, ffreg_);
		if (n < 2) return;
		// [#6.1] keep the engine's pole count in sync with the new curve — tele mech-rpm is eRPM/pp,
		// so a differing pp (the whole point of `motor`) would otherwise scale the feedback wrong.
		poles_ = (uint8_t)(polePairs * 2);
		escs::setPoles(index_, poles_);
		// [#6.3] reconstruct WITH the crossover metadata (&ffcx_) so hasCrossover()/lineFloor() stay
		// live — otherwise the "commanded 6-step but tele never went live" ABORT_STALL goes inert.
		ffprof_ = vel::SpeedProfile(ffbuf_, n, polePairs, &ffcx_, ffreg_);
		vc.setProfile(ffprof_);
		motorSet_ = true;
	}

	// Motor identity as last configured by `motor` — what `cfg save` persists (nothing to persist until
	// the operator has actually set it; the compile-time profile is the fallback).
	bool  motorConfigured() const { return motorSet_; }
	float motorKv()         const { return mm_.kv(); }
	int   motorPolePairs()  const { return mm_.polePairs(); }
	float motorVolts()      const { return mm_.voltage(); }
	float vbattScale()      const { return vbattScale_; }
	void  setVbattScale(float s)  { if (s > 0.0f) vbattScale_ = s; }

	// ESC perception (soft-sensor): infer effective supply voltage + a relative load signal from the
	// LIVE command + measured 6-step tele rpm — no voltage/current/force sensor. senseVoltage ~= battery
	// V at light load; senseLoad = (Vref - Vest) grows with current => torque => thrust (relative). 0
	// outside 6-step (needs live BEMF tele). Set the motor first with setMotor()/`motor`.
	float senseVoltage() { return senseValid_ ? vestFilt_ : 0.0f; }         // gated + low-passed
	// [#7.1] load = deviation from the LEARNED light-load reference (0 until it settles), not the typed V.
	float senseLoad()    { return (senseValid_ && senseVref_ > 0.0f) ? (senseVref_ - vestFilt_) : 0.0f; }
	float senseVref()    { return senseVref_; }                             // learned no-load V_eff (0 = not yet)
	void  senseZero()    { if (senseValid_) senseVref_ = vestFilt_; }        // re-baseline the reference on command
	bool  senseValid()   { return senseValid_; }                            // false = not BEMF-live 6-step
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
	bool  senseVcal(float knownV) {
		if (vbattMax_ <= 0.1f) return false;
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
			escs::spinThrust(index_, 0);   // neutral: keep armed + signal alive; firmware stops via Rcp_Stop
			vc.reset();
			lastStepUs_ = micros();
			return;
		}
		uint32_t now = micros();
		float dt = (now - lastStepUs_) * 1e-6f;
		lastStepUs_ = now;
		vel::Status st = vc.step(dt);
		if (st != vel::Status::OK) {
			stop();
			Serial.printf("# ESC %u RPM abort status=%d (1=overspeed 2=stall 3=temp) — stopped\n",
			              index_, (int)st);
		}
		// SOFT-SENSOR update: sample the voltage estimate ONLY in GENUINE 6-step. vc.live() alone can be
		// fooled by the firmware's virtual-sine eRPM (it reports a fresh non-zero rpm in forced sine), so
		// ALSO require the speed to be at/above the profile's own 6-step landing (senseRpmFloor(), #8) —
		// where V_eff = rpm/(KV*duty) actually holds. Below it (sine / gap / a fooled-live floor) the
		// estimate is meaningless -> mark invalid.
		if (vc.live() && fabsf(vc.measured()) >= senseRpmFloor()) {
			float ve = mm_.estimateV((float)vc.command(), vc.measured());
			vestFilt_ = senseValid_ ? vestFilt_ + 0.12f * (ve - vestFilt_) : ve;
			senseValid_ = true;
			// [vbatt] gated rolling-MAX of the estimate ~= supply voltage: V_eff <= V_supply, so the
			// lightest-load / highest-duty sample approaches the true supply. Gated a margin ABOVE the
			// 6-step landing (where duty is well modelled). Leaky so it TRACKS a draining battery
			// (slow decay) while rejecting transient load droops (which recover before the decay bites).
			if (fabsf(vc.measured()) >= vbattRpmFloor()) {
				if (vestFilt_ > vbattMax_) vbattMax_ = vestFilt_;
				else                       vbattMax_ -= THRUSTER_VBATT_DECAY;
			}
		} else {
			senseValid_ = false;
		}
		// [#7.1] The no-load REFERENCE is captured on command via `sense <i> zero` (senseZero()) at a
		// known steady light-load operating point — NOT the typed supply V (which read ~-2.2 V under
		// no load and needed the operator to measure the battery). load stays 0 until baselined.
	}

	// ---- io adapter: the ONLY place that knows escs:: telemetry -> vel::EscIo. Reads owner->index_. --
	struct Io : public vel::EscIo {
		Thruster* owner = nullptr;
		void thrust(int cmd) override { escs::spinThrust(owner->index_, (int16_t)cmd); }
		bool readTele(float& mechRpm, float& tempC) override {
			escs::Telem t;
			if (!escs::spinTele(owner->index_, t) || !t.valid) return false;
			if (millis() - t.rpmStampMs > THRUSTER_TELE_FRESH_MS) return false;   // stale eRPM -> sine
			if (t.rpm == 0) return false;                                        // no live 6-step sample
			mechRpm = (float)t.rpm;                                              // ALREADY mechanical (fw /pp)
			tempC   = (float)t.tempC;
			return true;
		}
	};

	// Member order matters: profile_ and io_ construct before vc (which binds references to them).
	const vel::SpeedProfile* profile_;
	Io                       io_;
	vel::VelocityController  vc;   // PUBLIC so the declaring side sets gains: th.vc.kp = 0.03f;

private:
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
	// once `motor` swaps the curve in. up==dn==the firmware handoff floor (mech = HANDOFF_ERPM/pp).
	vel::Crossover    ffcx_{vel::HANDOFF_ERPM, vel::HANDOFF_ERPM};
	// [#8] per-point regime tags (filled by MotorModel::fillProfile) so lineFloor() reports the real
	// 6-step landing instead of the seam. The stub's two points are both below it => SINE.
	vel::Regime       ffreg_[24] = {vel::Regime::SINE, vel::Regime::SINE};
	vel::SpeedProfile ffprof_{ffbuf_, 2, 7, &ffcx_, ffreg_};
	vel::MotorModel   mm_{350.0f, 7, 11.1f};   // parametric model (KV/PP/V) for FF + the soft-sensor
	bool              motorSet_ = false;       // has `motor`/setMotor run? (else mm_ holds only defaults)
	float             vestFilt_ = 0.0f;        // low-passed voltage estimate (soft-sensor)
	bool              senseValid_ = false;     // true only while BEMF-live (6-step) -> estimate valid
	float             senseVref_ = 0.0f;       // [#7.1] no-load V_eff reference (0 = not baselined; set via `sense zero`)
	float             vbattMax_ = 0.0f;        // [vbatt] gated rolling-max of vest ~= supply V (cleared by arm(), #5)
	float             vbattScale_ = 1.0f;      // [vbatt] calibration scale (senseVcal; persisted by `cfg save`)
};
