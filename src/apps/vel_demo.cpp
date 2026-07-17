// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// vel_demo — closed-loop VELOCITY control demo (Phase A2). Drive ONE bidir-DShot ESC to a target
// mechanical RPM: the RP2040 closes the loop on the ESC's own eRPM telemetry, faster than the host's
// 50 Hz. This is the on-device home of the control law verified in Python (host/pico_esc/velocity.py)
// and ported to the portable lib/vel_control library — see docs/velctl-generalization.md (Phase A2).
//
// It shows the intended LIBRARY usage: the declaring main owns the plant-dependent gains and sets
// them directly (esc1.kp = 0.03f; ...). The controller takes an injected backend (EscSessionIo, the
// glue over the escs:: DShot engine) + a calibrated SpeedProfile; everything else is generic.
//
// Wire / poles / DShot rate come from esc_config.h. Build & run:
//     pio run -e vel_demo -t upload -t monitor
//
// Serial protocol (newline mode), driving ESC index VEL_ESC_INDEX:
//   arm            arm the ESC (streams zero ~3 s; required before vel)
//   vel <rpm>      set the signed target mech RPM (negative = reverse); starts the closed loop
//   stop           target 0 and hold (loop keeps running)
//   disarm         stop the motor and release DShot
//   g kp <v> | g ki <v> | g trim <v> | g slew <v>   tune a gain live
//   ?              print status header / help
#include <Arduino.h>
#include "esc_config.h"
#include "esc_session.h"
#include "vel_control.h"

#ifndef VEL_ESC_INDEX
#define VEL_ESC_INDEX 1          // which ESC (index into ESC_SIGNAL_PINS) this demo drives
#endif
static const uint32_t TELE_FRESH_MS = 100;   // an rpm older than this = stale (forced sine / dropout)

// ---- backend: adapt the escs:: engine (one ESC index) to vel::EscIo -------------------------------
// Keeps the control library hardware-free — this is the ONLY place that knows about escs::.
class EscSessionIo : public vel::EscIo {
public:
	explicit EscSessionIo(uint8_t idx) : idx_(idx) {}
	void thrust(int cmd) override { escs::spinThrust(idx_, (int16_t)cmd); }
	bool readTele(float& mechRpm, float& tempC) override {
		escs::Telem t;
		if (!escs::spinTele(idx_, t) || !t.valid) return false;
		if (millis() - t.rpmStampMs > TELE_FRESH_MS) return false;   // stale rpm (no fresh eRPM) -> sine
		mechRpm = (float)t.rpm;                                      // ALREADY mechanical (firmware /pp)
		tempC   = (float)t.tempC;
		return true;
	}
private:
	uint8_t idx_;
};

// ---- calibrated 930KV 12N14P curve (bench-measured 2026-07-16/17). thrust (0..1000) -> mech RPM. --
// Sine seed (<=250) + 6-step points; the seam gap (sine caps ~84, 6-step starts ~2576) is baked in.
// A full velcal would refine this; the closed loop corrects the residual FF error at runtime.
static const vel::CurvePoint CURVE[] = {
	{  0,    0.0f}, { 60,   20.3f}, {108,   35.6f}, {155,   51.8f}, {202,   67.6f}, {250,   83.6f},
	{620, 2576.0f}, {700, 4645.0f}, {800, 7173.0f}, {900, 9447.0f}, {1000, 11000.0f},
};
static const vel::Regime REGIMES[] = {
	vel::Regime::SINE, vel::Regime::SINE, vel::Regime::SINE, vel::Regime::SINE, vel::Regime::SINE,
	vel::Regime::SINE, vel::Regime::LINE, vel::Regime::LINE, vel::Regime::LINE, vel::Regime::LINE,
	vel::Regime::LINE,
};
static const int        NCURVE = sizeof(CURVE) / sizeof(CURVE[0]);
static const vel::Crossover CROSSOVER = { 1500.0f, 1350.0f };   // up_erpm / dn_erpm (config used on-bench)

static EscSessionIo        io(VEL_ESC_INDEX);
static vel::SpeedProfile   prof(CURVE, NCURVE, /*pole_pairs=*/7, &CROSSOVER, REGIMES);
static vel::VelocityController esc1(io, prof);   // <- the object the app tunes: esc1.kp = ...

static bool     velActive = false;         // closed loop engaged (vel command seen since arm)
static uint32_t lastStepUs = 0;

static void printHelp() {
	Serial.println("# vel_demo — closed-loop velocity control");
	Serial.printf("# ESC index %d (pin %u), curve max %.0f RPM, seam up=%.0f eRPM\n",
	              VEL_ESC_INDEX, escs::PINS[VEL_ESC_INDEX], prof.maxRpm(), CROSSOVER.up_erpm);
	Serial.printf("# gains: kp=%.3f ki=%.3f trim_max=%.0f blend_secs=%.2f slew=%.0f\n",
	              esc1.kp, esc1.ki, esc1.trim_max, esc1.blend_secs, esc1.slew_rpm_s);
	Serial.println("# cmds: arm | vel <rpm> | stop | disarm | g kp|ki|trim|slew <v> | ?");
	Serial.println("t\ttarget\tsetp\tcmd\ttele\ttrim\tw\tlive");
}

static void handleSerial() {
	if (!Serial.available()) return;
	delay(3);                                     // let the whole line arrive
	String line = Serial.readStringUntil('\n');
	line.trim();
	if (line.length() == 0) return;
	int sp = line.indexOf(' ');
	String cmd = (sp < 0) ? line : line.substring(0, sp);
	String rest = (sp < 0) ? String("") : line.substring(sp + 1);
	rest.trim();
	cmd.toLowerCase();

	if (cmd == "arm") {
		escs::spinArm(VEL_ESC_INDEX, escs::Drive::AUTO);
		if (!escs::spinInitOk(VEL_ESC_INDEX)) { Serial.println("err dshot-init-failed"); return; }
		esc1.reset(); velActive = false;
		Serial.printf("arming ~3s (mode %s, %s)\n", escs::spinMode(VEL_ESC_INDEX),
		              escs::spinReversible(VEL_ESC_INDEX) ? "reversible" : "one-way");
	} else if (cmd == "vel") {
		if (!escs::spinArmed(VEL_ESC_INDEX)) { Serial.println("err not-armed (send 'arm' first)"); return; }
		float rpm = rest.toFloat();
		esc1.setTarget(rpm);
		velActive = true; lastStepUs = micros();
		Serial.printf("vel -> target %.0f RPM\n", rpm);
	} else if (cmd == "stop") {
		esc1.setTarget(0.0f);
		Serial.println("stop -> target 0");
	} else if (cmd == "disarm") {
		velActive = false; escs::spinStop(VEL_ESC_INDEX);
		Serial.println("disarmed");
	} else if (cmd == "g") {                       // live gain tuning: g <name> <value>
		int s2 = rest.indexOf(' ');
		String name = (s2 < 0) ? rest : rest.substring(0, s2);
		float v = (s2 < 0) ? 0.0f : rest.substring(s2 + 1).toFloat();
		if      (name == "kp")   esc1.kp = v;
		else if (name == "ki")   esc1.ki = v;
		else if (name == "trim") esc1.trim_max = v;
		else if (name == "slew") esc1.slew_rpm_s = v;
		else { Serial.println("err bad-gain (kp|ki|trim|slew)"); return; }
		Serial.printf("gain %s = %.3f\n", name.c_str(), v);
	} else if (cmd == "?") {
		printHelp();
	} else {
		Serial.println("err unknown-cmd (?)");
	}
}

void setup() {
	Serial.begin(115200);
	delay(2000);                                   // give the USB host time to attach
	// PLANT-DEPENDENT GAINS set right here in main — the requested `esc1.kp = ...` style. These are
	// the 930KV bench-verified values (the sim defaults 0.4/1.5 would saturate this ~30x-hotter plant).
	esc1.kp = 0.03f;
	esc1.ki = 0.12f;
	esc1.trim_max = 400.0f;
	esc1.blend_secs = 0.3f;
	esc1.slew_rpm_s = 4000.0f;                     // reach a 6-step target within a couple seconds
	esc1.max_temp = 0.0f;                          // EDT temp unreliable on these ESCs -> no temp abort
	printHelp();
}

void loop() {
	handleSerial();

	// Closed-loop tick: run only once armed + engaged. dt is the real elapsed time (faster than 50 Hz).
	if (velActive && escs::spinArmed(VEL_ESC_INDEX)) {
		uint32_t now = micros();
		float dt = (now - lastStepUs) * 1e-6f;
		lastStepUs = now;
		vel::Status st = esc1.step(dt);
		if (st != vel::Status::OK) {
			velActive = false;
			Serial.printf("# ABORT status=%d (0=ok 1=overspeed 2=stall 3=temp) — motor stopped\n", (int)st);
		}
	}
	escs::spinPoll();                              // keep DShot frames flowing + read telemetry

	static uint32_t lastPrint = 0;
	if (velActive && millis() - lastPrint > 200) {
		lastPrint = millis();
		Serial.printf("%lu\t%.0f\t%.0f\t%d\t%.0f\t%.0f\t%.2f\t%d\n",
		              (unsigned long)millis(), esc1.target(), esc1.setpoint(), esc1.command(),
		              esc1.live() ? esc1.measured() : 0.0f, esc1.trim(), esc1.authority(), esc1.live() ? 1 : 0);
	}
}

void setup1() {}
void loop1()  { escs::core1Poll(); }
