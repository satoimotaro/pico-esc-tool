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
#include "profiles.h"

// An rpm whose telemetry stamp is older than this = stale (forced sine / dropout) — the PI authority
// then fades and the controller degrades to pure feed-forward.
static const uint32_t THRUSTER_TELE_FRESH_MS = 100;

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
	void arm(Drive mode = Drive::AUTO) { escs::spinArm(index_, mode); vc.reset(); submode_ = RAW; }
	// RAW target: signed thrust on a reversible (3D) ESC, else unidirectional throttle. Setting a RAW
	// target disengages the rpm loop (submode_ -> RAW) so a stray step() can't fight it.
	void setRaw(int v) {
		submode_ = RAW;
		if (reversible()) escs::spinThrust(index_, (int16_t)v);
		else              escs::spinThrottle(index_, (uint16_t)v);
	}
	// RPM target: engage the closed loop. Reset the step clock so the first dt is sane.
	void setRpm(float rpm) { submode_ = RPM; vc.setTarget(rpm); lastStepUs_ = micros(); }
	void stop()   { escs::spinStop(index_); submode_ = RAW; }
	void disarm() { stop(); }

	// Copy a profile's calibrated PI gains onto the controller (e.g. the generated M_<NAME>_GAINS).
	// The declaring side may still override any field afterward (th.vc.slew_rpm_s = ...).
	void applyGains(const vel::Gains& g) { vc.kp = g.kp; vc.ki = g.ki; vc.trim_max = g.trim_max; vc.blend_secs = g.blend_secs; }

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
		// STOP request (|target| <= stop_below_rpm): hold a signal-loss stop. A thrust-0 command does NOT
		// stop a 3D ESC (DShot 0 isn't the firmware's Rcp_Stop; the rotor idles at the weak-BEMF floor),
		// so pause the DShot signal each tick -> the ESC times out -> exits run mode -> STOPS. The driver
		// stays alive, so the next non-zero target resumes within the pause window (no 3 s re-arm).
		if (fabsf(vc.target()) <= vc.stop_below_rpm) {
			escs::spinPauseSignal(index_, 40);
			if (!stopping_) { vc.reset(); stopping_ = true; }
			return;
		}
		// Leaving a stop: the signal-loss cold-disarmed the ESC, so re-arm it once before driving again
		// (the arm streams zero ~SPIN_ARM_MS, then armed() goes true and the loop below runs).
		if (stopping_) {
			stopping_ = false;
			escs::spinArm(index_);   // re-arm the cold-disarmed ESC (keeps RPM submode); ~SPIN_ARM_MS
			lastStepUs_ = micros();
			return;
		}
		if (!armed()) return;   // re-arm in progress
		uint32_t now = micros();
		float dt = (now - lastStepUs_) * 1e-6f;
		lastStepUs_ = now;
		vel::Status st = vc.step(dt);
		if (st != vel::Status::OK) {
			stop();
			Serial.printf("# ESC %u RPM abort status=%d (1=overspeed 2=stall 3=temp) — stopped\n",
			              index_, (int)st);
		}
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
	uint16_t kbaud_;
	uint8_t  poles_;
	uint8_t  index_      = 0;
	Submode  submode_    = RAW;
	bool     stopping_   = false;   // in a signal-loss stop (target ~0)
	uint32_t lastStepUs_ = 0;
};
