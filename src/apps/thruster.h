// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// thruster — a per-ESC OBJECT bound to one index into the escs:: engine. This is the object-oriented
// layer that replaces the "flash a separate app per job" model: EscManager owns one Thruster per
// wired ESC (Thruster th_[Esc::COUNT]) and binds each in begin(), so the ESC count scales with
// ESC_SIGNAL_PINS automatically. The heavy lifting still lives in the proven singleton escs:: engine
// (2 PIO SMs + one core1 1-wire worker) — a Thruster just delegates hardware to escs:: by its index_
// and adds the per-ESC closed-loop velocity controller (lib/vel_control) on top.
//
// DRIVE has two submodes:
//   RAW  — a direct thrust/throttle target (escs:: holds it; poll() does nothing).
//   RPM  — the closed velocity loop: setRpm() sets the target, poll() runs vc.step(dt) each core0 tick.
#pragma once
#include <Arduino.h>
#include "esc_session.h"
#include "vel_control.h"

// An rpm whose telemetry stamp is older than this = stale (forced sine / dropout) — the PI authority
// then fades and the controller degrades to pure feed-forward. Mirrors vel_demo's TELE_FRESH_MS.
static const uint32_t THRUSTER_TELE_FRESH_MS = 100;

// ---- calibrated 930KV 12N14P curve (bench-measured 2026-07-16/17). thrust (0..1000) -> mech RPM. --
// Copied verbatim from vel_demo.cpp: sine seed (<=250) + 6-step points; the seam gap (sine caps ~84,
// 6-step starts ~2576) is baked in. The closed loop corrects the residual FF error at runtime.
// File-scope statics (owned by the caller, no allocation) referenced by every Thruster's SpeedProfile.
static const vel::CurvePoint THRUSTER_CURVE[] = {
	{  0,    0.0f}, { 60,   20.3f}, {108,   35.6f}, {155,   51.8f}, {202,   67.6f}, {250,   83.6f},
	{620, 2576.0f}, {700, 4645.0f}, {800, 7173.0f}, {900, 9447.0f}, {1000, 11000.0f},
};
static const vel::Regime THRUSTER_REGIMES[] = {
	vel::Regime::SINE, vel::Regime::SINE, vel::Regime::SINE, vel::Regime::SINE, vel::Regime::SINE,
	vel::Regime::SINE, vel::Regime::LINE, vel::Regime::LINE, vel::Regime::LINE, vel::Regime::LINE,
	vel::Regime::LINE,
};
static const int            THRUSTER_NCURVE = sizeof(THRUSTER_CURVE) / sizeof(THRUSTER_CURVE[0]);
static const vel::Crossover THRUSTER_CROSSOVER = { 1500.0f, 1350.0f };   // up_erpm / dn_erpm (on-bench)

class Thruster {
public:
	using Info  = escs::Info;
	using Telem = escs::Telem;
	using Drive = escs::Drive;
	enum Submode { RAW, RPM };

	// Default-constructible so EscManager can hold a plain array and bind() each in begin(). The io
	// adapter, calibrated profile, and controller are wired here; the controller's gains stay at their
	// library defaults until the declaring side (EscManager::begin) sets the per-motor values.
	Thruster()
		: profile_(THRUSTER_CURVE, THRUSTER_NCURVE, /*pole_pairs=*/7, &THRUSTER_CROSSOVER, THRUSTER_REGIMES),
		  vc(io_, profile_) {
		io_.owner = this;
	}

	void bind(uint8_t index) { index_ = index; }
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
	void arm(Drive mode = Drive::AUTO) {
		escs::spinArm(index_, mode);
		vc.reset(); submode_ = RAW;
	}
	// RAW target: pick signed thrust on a reversible (3D) ESC, else unidirectional throttle. Setting a
	// RAW target disengages the rpm loop (submode_ -> RAW) so a stray step() can't fight it.
	void setRaw(int v) {
		submode_ = RAW;
		if (reversible()) escs::spinThrust(index_, (int16_t)v);
		else              escs::spinThrottle(index_, (uint16_t)v);
	}
	// RPM target: engage the closed loop. Reset the step clock so the first dt is sane.
	void setRpm(float rpm) {
		submode_ = RPM;
		vc.setTarget(rpm);
		lastStepUs_ = micros();
	}
	void stop()   { escs::spinStop(index_); submode_ = RAW; }
	void disarm() { stop(); }

	bool        armed()      { return escs::spinArmed(index_); }
	bool        reversible() { return escs::spinReversible(index_); }
	const char* spinMode()   { return escs::spinMode(index_); }
	bool        initOk()     { return escs::spinInitOk(index_); }
	bool        tele(Telem& out) { return escs::spinTele(index_, out); }
	Submode     submode() const { return submode_; }

	// Called every core0 loop (EscManager::poll). Only the RPM submode does work here: run one closed-
	// loop tick with the REAL elapsed dt (faster than the host's 50 Hz). On a non-OK status the loop
	// aborted (over-speed / stall / over-temp) — stop the motor and fall back to RAW. RAW does nothing
	// (escs:: already holds the target); both submodes still rely on EscManager's shared escs::spinPoll.
	void poll() {
		if (submode_ != RPM || !armed()) return;
		uint32_t now = micros();
		float dt = (now - lastStepUs_) * 1e-6f;
		lastStepUs_ = now;
		vel::Status st = vc.step(dt);
		if (st != vel::Status::OK) {
			stop();   // sets submode_ = RAW
			Serial.printf("# ESC %u RPM abort status=%d (1=overspeed 2=stall 3=temp) — stopped\n",
			              index_, (int)st);
		}
	}

	// PUBLIC controller so the firmware sets gains directly: th.vc.kp = 0.03f; (the "esc1.kp=" style).
	// Declared last so it constructs after io_/profile_ (member init order = declaration order).
	// ---- io adapter: the ONLY place that knows escs:: telemetry -> vel::EscIo. Reads owner->index_. --
	struct Io : public vel::EscIo {
		Thruster* owner = nullptr;
		void thrust(int cmd) override { escs::spinThrust(owner->index_, (int16_t)cmd); }
		bool readTele(float& mechRpm, float& tempC) override {
			escs::Telem t;
			if (!escs::spinTele(owner->index_, t) || !t.valid) return false;
			if (millis() - t.rpmStampMs > THRUSTER_TELE_FRESH_MS) return false;  // stale eRPM -> sine
			if (t.rpm == 0) return false;                                        // no live 6-step sample
			mechRpm = (float)t.rpm;                                              // ALREADY mechanical (fw /pp)
			tempC   = (float)t.tempC;
			return true;
		}
	};

	Io                     io_;
	vel::SpeedProfile      profile_;
	vel::VelocityController vc;

private:
	uint8_t  index_    = 0;
	Submode  submode_  = RAW;
	uint32_t lastStepUs_ = 0;
};
