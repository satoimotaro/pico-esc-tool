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
	float disturb = 0.0f;       // external load: reduces the achieved rpm for a given command (rpm)

	void thrust(int cmd) override {
		last_sent = cmd;
		float target = never_live ? 0.0f : copysignf(lineMech(cmd), (float)cmd);
		if (cmd == 0) target = 0.0f;
		if (target != 0.0f) {                                   // load droop
			float m = fabsf(target) - disturb; if (m < 0.0f) m = 0.0f;
			target = copysignf(m, target);
		}
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

// DOB: run to a 6-step target, apply a step load at t=4s, return mean |error| over [4s, 6s].
static float run_dob(SpeedProfile& prof, float target, float dob, float load) {
	SimEsc sim; sim.dt = 0.02f;
	VelocityController vc(sim, prof);
	vc.kp = 0.06f; vc.ki = 0.25f; vc.trim_max = 300.0f; vc.slew_rpm_s = 2000.0f;
	vc.dob = dob; vc.dob_tau = 0.08f;
	vc.setTarget(target);
	float dt = 0.02f; int on = (int)(4.0f / dt), win = (int)(6.0f / dt);
	float sum = 0.0f; int cnt = 0;
	for (int t = 0; t < win; t++) {
		sim.disturb = (t >= on) ? load : 0.0f;
		if (vc.step(dt) != Status::OK) break;
		if (t >= on) { sum += fabsf(target - fabsf(vc.measured())); cnt++; }
	}
	return cnt ? sum / cnt : 1e9f;
}

// DOB on a SLEWED step with NO load: rpmFor() predicts the STEADY rpm of a command while the plant is
// still rising toward it, so an ungated observer reads that lag as a PHANTOM LOAD and pushes the command
// up — partly defeating slew_rpm_s (the same mechanism that desynced the bench at spin-up, at a smaller
// amplitude). Returns the mean |d_hat| over the ramp (the phantom load itself) and, via peakCmd, the peak
// command. Both endpoints sit in the LINE region where the sim plant and the profile agree, and the plant
// runs at a 6-STEP tau (the sysid measured 25-130 ms there, vs ~200 ms in sine) — the DOB is gated to
// 6-step, so that is the regime this must hold in.
static float run_dob_ramp(SpeedProfile& prof, float lo, float hi, float dob, float settle_secs,
                          int* peakCmd = nullptr) {
	SimEsc sim; sim.dt = 0.02f; sim.tau = 0.04f;
	VelocityController vc(sim, prof);
	vc.kp = 0.06f; vc.ki = 0.25f; vc.trim_max = 300.0f; vc.slew_rpm_s = 800.0f;
	vc.dob = dob; vc.dob_tau = 0.08f; vc.dob_settle_secs = settle_secs;
	vc.setTarget(lo);
	float dt = 0.02f;
	for (int t = 0; t < (int)(4.0f / dt); t++) if (vc.step(dt) != Status::OK) return 1e9f;   // settle at lo
	vc.setTarget(hi);
	int peak = 0; float sum = 0.0f; int cnt = 0;
	for (int t = 0; t < (int)(1.5f / dt); t++) {
		if (vc.step(dt) != Status::OK) return 1e9f;
		int c = vc.command() < 0 ? -vc.command() : vc.command();
		if (c > peak) peak = c;
		sum += fabsf(vc.dhat()); cnt++;
	}
	if (peakCmd) *peakCmd = peak;
	return cnt ? sum / cnt : 1e9f;
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
	// NB: these Crossovers MUST be brace-initialised. Prof's in-class initializer belongs to Prof, not to
	// a local `Crossover c1;` — that leaves up_erpm/dn_erpm indeterminate, and lineFloor() (the stall
	// guard, the DOB gap check) then reads garbage that happens to pass or fail with the stack layout.
	CurvePoint p1[12]; Crossover c1{UP_ERPM, DN_ERPM}; SpeedProfile sp = Prof::build(p1, &c1, 1.0f);
	CHECK(fabsf(sp.thrustFor(0.0f)) < 1e-6f, "thrustFor(0)==0");
	CHECK(sp.thrustFor(-500.0f) == -sp.thrustFor(500.0f), "thrustFor odd-symmetric");
	CHECK(sp.thrustFor(1e9f) == 1000.0f, "thrustFor clamps to endpoint");

	// -- closed loop: mis-scaled FF (x1.25) converges with the PI, misses with pure FF --
	CurvePoint pm[12]; Crossover cm{UP_ERPM, DN_ERPM}; SpeedProfile mis = Prof::build(pm, &cm, 1.25f);
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

	{   // DOB rejects a step load faster/better than PI alone.
		// NOTE on what this proves: SimEsc models load as `target = lineMech(cmd) - disturb`, a CONSTANT
		// additive rpm deficit — precisely the disturbance shape the DOB assumes. So the headline ratio
		// confirms the implementation is correct, NOT that it predicts bench performance: a real prop is
		// ~rpm^2, which TILTS the curve rather than offsetting it, making d_hat speed-dependent. The
		// honest hardware number is the bench 1650 -> 1596 @ 1600.
		CurvePoint pdpts[12]; Crossover pdcx{UP_ERPM, DN_ERPM};
		SpeedProfile pdp = Prof::build(pdpts, &pdcx, 1.0f);
		float target = 600.0f, load = 100.0f;              // 6-step target, 100-rpm step load at t=4s
		float e_pi  = run_dob(pdp, target, 0.0f, load);
		float e_dob = run_dob(pdp, target, 0.8f, load);
		printf("   DOB: mean|err| over the load window  PI=%.1f  DOB=%.1f rpm\n", e_pi, e_dob);
		CHECK(e_dob < 0.6f * e_pi, "DOB cuts step-load error >40% vs PI-only");
		CHECK(run_dob(pdp, target, 0.0f, 0.0f) < 15.0f, "no load: baseline tracks target (sanity)");
		// The cell the original test was missing: with the observer ENABLED and nothing to observe, does
		// it stay quiet? Direct regression guard for reading a model/plant mismatch as a phantom load.
		float e_quiet = run_dob(pdp, target, 0.8f, 0.0f);
		printf("   DOB quiet check: mean|err| with dob=0.8, load=0 -> %.1f rpm\n", e_quiet);
		CHECK(e_quiet < 15.0f, "no load: DOB stays quiet (nothing to observe)");

		// ...and the same question during a SLEW, which is where the steady-state prediction lags the
		// plant. Ungated, the DOB reads that lag as load and partly defeats slew_rpm_s.
		int p_pi = 0, p_gated = 0, p_open = 0;
		run_dob_ramp(pdp, 520.0f, 700.0f, 0.0f, 0.25f, &p_pi);
		float d_gated = run_dob_ramp(pdp, 520.0f, 700.0f, 0.8f, 0.25f, &p_gated);
		float d_open  = run_dob_ramp(pdp, 520.0f, 700.0f, 0.8f,   0.0f, &p_open);   // settle gate disabled
		printf("   DOB ramp (no load): mean|d_hat| gated=%.1f ungated=%.1f rpm   peak cmd PI=%d gated=%d ungated=%d\n",
		       d_gated, d_open, p_pi, p_gated, p_open);
		CHECK(d_gated < 0.5f * d_open, "settle gate: no phantom load accumulates during a slew");
		CHECK(p_gated <= p_pi + 10, "settle gate: DOB does not over-drive a slewed step");
		CHECK(p_open > p_gated, "ungated DOB DOES over-drive the ramp (the gate is doing work)");
	}

	printf(failures ? "\n%d CHECK(S) FAILED\n" : "\nALL CHECKS PASSED\n", failures);
	return failures ? 1 : 0;
}
