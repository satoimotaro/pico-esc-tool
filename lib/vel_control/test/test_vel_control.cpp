// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// Native (host g++) test for the portable vel_control library — the C++ port must behave like the
// verified Python reference (host/tests/test_velctl_closedloop.py): a deliberately mis-scaled feed-
// forward converges once the PI trim closes on live 6-step telemetry, while pure FF (kp=ki=0) misses.
// No Arduino/PIO — vel_control.h is hardware-free, so this compiles and runs anywhere:
//     g++ -std=c++17 -O2 -Wall -o /tmp/tvc test_vel_control.cpp && /tmp/tvc
#include "../vel_control.h"
#include <cstdio>
#include <cmath>
#include <cstdlib>

using namespace vel;

static const int   POLE_PAIRS = 7;
static const float UP_ERPM = 1600.0f, DN_ERPM = 1400.0f;

// The sim's 6-step BEMF load line: |thrust| -> mech RPM (mirrors the Python test's anchors / small
// plant gain, FS~357). Deliberately LOW gain so the sim-tuned gains (0.4/1.5) are stable here.
static float lineMech(float thr) {
	float slope = (3800.0f - 190.0f) / (700.0f - 55.0f);
	return (190.0f + slope * (fabsf(thr) - 55.0f)) / POLE_PAIRS;
}

// A first-order plant + honest telemetry regime: telemetry is LIVE only in 6-step (eRPM >= dn seam);
// in forced sine it is stale (readTele -> false), exactly like the firmware. Hardware reports a
// MAGNITUDE, so readTele returns |rpm| (the controller re-attaches the commanded sign).
struct SimEsc : EscIo {
	float rpm = 0.0f;      // current mech RPM (signed)
	float dt = 0.02f;
	float tau = 0.15f;     // plant time constant (s)
	int   last_sent = 0;
	bool  never_live = false;   // force a stall scenario (plant that never reaches 6-step)

	void thrust(int cmd) override {
		last_sent = cmd;
		float target = never_live ? 0.0f : copysignf(lineMech(cmd), (float)cmd);
		if (cmd == 0) target = 0.0f;
		rpm += (target - rpm) * (dt / tau);
	}
	bool readTele(float& mechRpm, float& tempC) override {
		tempC = 0.0f;
		if (fabsf(rpm) * POLE_PAIRS < DN_ERPM) return false;   // forced sine / below seam -> stale
		mechRpm = fabsf(rpm);                                  // hardware magnitude
		return true;
	}
};

// Finely-sampled sine+line curve; scale multiplies the rpm axis (scale>1 => FF OVER-reports speed,
// so inverting a target UNDER-commands thrust: the mis-calibration the loop must correct).
struct Prof {
	CurvePoint pts[12];
	Crossover  cx{UP_ERPM, DN_ERPM};
	SpeedProfile profile;
	static SpeedProfile build(CurvePoint* pts, Crossover* cx, float scale) {
		const float sineT[] = {0, 100, 300, 500, 600};
		const float sineR[] = {0, 35.7f, 107.1f, 178.5f, 214.2f};
		int k = 0;
		for (int i = 0; i < 5; i++) { pts[k].thrust = sineT[i]; pts[k].rpm = sineR[i] * scale; k++; }
		const float lineT[] = {640, 700, 760, 820, 880, 940, 1000};
		for (int i = 0; i < 7; i++) { pts[k].thrust = lineT[i]; pts[k].rpm = lineMech(lineT[i]) * scale; k++; }
		return SpeedProfile(pts, k, POLE_PAIRS, cx);
	}
};

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); failures++; } \
	else printf("ok: %s\n", msg); } while (0)

// Run the controller to a target for `secs`; return the mean |measured| over the last 1 s.
static float run(SpeedProfile& prof, float target, float kp, float ki, float trim_max = 200.0f,
                 float secs = 6.0f, bool stallPlant = false) {
	SimEsc sim; sim.never_live = stallPlant;
	VelocityController vc(sim, prof);
	vc.kp = kp; vc.ki = ki; vc.trim_max = trim_max; vc.slew_rpm_s = 500.0f; vc.stall_secs = 2.0f;
	vc.setTarget(target);
	float dt = 0.02f; sim.dt = dt;
	int n = (int)(secs / dt);
	float sum = 0.0f; int cnt = 0;
	for (int t = 0; t < n; t++) {
		Status s = vc.step(dt);
		if (s != Status::OK) { printf("   (aborted: status=%d at t=%.2f)\n", (int)s, t * dt); break; }
		if (t >= n - (int)(1.0f / dt)) { sum += fabsf(vc.measured()) * (vc.live() ? 1.0f : 0.0f); cnt += vc.live() ? 1 : 0; }
	}
	return cnt ? sum / cnt : 0.0f;
}

// A stall must ABORT (command into 6-step, telemetry never lives).
static Status runExpectStatus(SpeedProfile& prof, float target, bool stallPlant) {
	SimEsc sim; sim.never_live = stallPlant;
	VelocityController vc(sim, prof);
	vc.kp = 0.4f; vc.ki = 1.5f; vc.slew_rpm_s = 2000.0f; vc.stall_secs = 0.5f;
	vc.setTarget(target);
	for (int t = 0; t < 500; t++) { Status s = vc.step(0.02f); if (s != Status::OK) return s; }
	return Status::OK;
}

int main() {
	// -- thrustFor: exact at points, clamped, odd-symmetric --
	CurvePoint p1[12]; Crossover c1; SpeedProfile sp = Prof::build(p1, &c1, 1.0f);
	CHECK(fabsf(sp.thrustFor(0.0f)) < 1e-6f, "thrustFor(0)==0");
	CHECK(sp.thrustFor(-500.0f) == -sp.thrustFor(500.0f), "thrustFor odd-symmetric");
	CHECK(sp.thrustFor(1e9f) == 1000.0f, "thrustFor clamps to endpoint");

	// -- closed loop: mis-scaled FF (x1.25) converges with the PI, misses with pure FF --
	CurvePoint pm[12]; Crossover cm; SpeedProfile mis = Prof::build(pm, &cm, 1.25f);
	float target = 400.0f;                         // 400*7 = 2800 eRPM, clearly 6-step
	float pi  = run(mis, target, 0.4f, 1.5f);
	float ff  = run(mis, target, 0.0f, 0.0f);
	printf("   PI mean=%.1f (err %.1f%%)   pureFF mean=%.1f (err %.1f%%)   target=%.0f\n",
	       pi, fabsf(pi - target) / target * 100.0f, ff, fabsf(ff - target) / target * 100.0f, target);
	CHECK(fabsf(pi - target) / target < 0.05f, "PI converges within 5%");
	CHECK(fabsf(ff - target) / target > 0.10f, "pure FF misses by >10% (mis-scaled)");

	// -- guards --
	CHECK(runExpectStatus(mis, target, /*stallPlant=*/true) == Status::ABORT_STALL,
	      "stall aborts when 6-step never lives");

	// -- STOP at zero: setTarget(0) commands thrust 0 and disengages (doesn't creep) --
	{
		SimEsc sim; sim.rpm = 400.0f;                 // motor already spinning
		VelocityController vc(sim, mis);
		vc.kp = 0.4f; vc.ki = 1.5f;
		vc.setTarget(0.0f);
		for (int t = 0; t < 20; t++) vc.step(0.02f);
		CHECK(vc.command() == 0 && vc.authority() == 0.0f, "setTarget(0) -> command 0, loop disengaged");
	}
	{   // stop_below_rpm: a sub-floor target also stops
		SimEsc sim; sim.rpm = 400.0f;
		VelocityController vc(sim, mis);
		vc.stop_below_rpm = 150.0f;
		vc.setTarget(100.0f);
		for (int t = 0; t < 10; t++) vc.step(0.02f);
		CHECK(vc.command() == 0, "target below stop_below_rpm -> command 0");
	}

	printf(failures ? "\n%d CHECK(S) FAILED\n" : "\nALL CHECKS PASSED\n", failures);
	return failures ? 1 : 0;
}
