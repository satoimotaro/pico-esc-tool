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
static const float TDT = 0.02f;      // the tick the thermal cases run at (50 Hz)

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
	// Thermal (B.7). tempC is driven by the test; 0 means the ESC never reports a temperature at all
	// (the "never seen" path, which must behave exactly like the limiter disabled). legacy_temp routes it
	// through readTele's tempC instead, i.e. the pre-B.7 EscIo contract.
	// temp_every models the real EDT cadence: a NEW temperature frame only every Nth control tick. The
	// adapter keeps reporting the value it holds (the stateless contract) and the stamp is what moves, so
	// this is what proves the abort debounce counts FRAMES and not ticks.
	float    tempC = 0.0f;
	bool     legacy_temp = false;
	int      temp_every = 1;
	uint32_t temp_stamp = 1;
	int      temp_tick = 0;

	void thrust(int cmd) override {
		last_sent = cmd;
		if (++temp_tick >= temp_every) { temp_tick = 0; temp_stamp++; }   // one EDT frame per temp_every ticks
		float target = never_live ? 0.0f : copysignf(lineMech(cmd), (float)cmd);
		if (cmd == 0) target = 0.0f;
		if (target != 0.0f) {                                   // load droop
			float m = fabsf(target) - disturb; if (m < 0.0f) m = 0.0f;
			target = copysignf(m, target);
		}
		rpm += (target - rpm) * (dt / tau);
	}
	bool readTele(float& mechRpm, float& tempC_out) override {
		tempC_out = legacy_temp ? tempC : 0.0f;
		if (fabsf(rpm) * POLE_PAIRS < DN_ERPM) return false;   // forced sine / below seam -> stale
		mechRpm = fabsf(rpm);                                  // hardware magnitude
		return true;
	}
	// eRPM-independent, exactly like the firmware adapter: reports even while readTele is stale, and is
	// STATELESS — it hands over the held value plus the stamp of the frame it came from and lets the
	// controller decide what is new and what is stale.
	bool readTemp(float& tempC_out, uint32_t& stampMs) override {
		if (legacy_temp || tempC <= 0.0f) return false;
		tempC_out = tempC; stampMs = temp_stamp; return true;
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

	{   // -- uploaded-curve semantics (the `curve` command's rules), tested in C++ rather than only via
		//    the Python mirror in SimEscHost — the two had already begun to diverge.
		CurvePoint ok3[] = {{0, 0}, {100, 50}, {200, 900}};
		CHECK(validateCurve(ok3, 3), "validateCurve accepts a monotone curve");
		CHECK(!validateCurve(ok3, 1), "validateCurve rejects fewer than 2 points");
		CurvePoint backT[] = {{0, 0}, {200, 50}, {150, 90}};      // thrust goes backwards
		CHECK(!validateCurve(backT, 3), "validateCurve rejects non-increasing thrust");
		CurvePoint backR[] = {{0, 0}, {100, 90}, {200, 50}};      // rpm decreases -> non-invertible
		CHECK(!validateCurve(backR, 3), "validateCurve rejects decreasing rpm");
		CurvePoint neg[] = {{0, 0}, {100, -5}};
		CHECK(!validateCurve(neg, 2), "validateCurve rejects negative rpm");

		// pp=7, seam 1400 eRPM => 200 mech. Point 1 (50 rpm) is below, point 2 (900) far above.
		const float UP = 1400.0f;
		Regime out[3];
		tagRegimes(ok3, 3, POLE_PAIRS, UP, nullptr, out);
		CHECK(out[0] == Regime::SINE && out[1] == Regime::SINE && out[2] == Regime::LINE,
		      "tagRegimes derives from the seam when untagged");

		Regime sineEverywhere[] = {Regime::SINE, Regime::SINE, Regime::SINE};
		tagRegimes(ok3, 3, POLE_PAIRS, UP, sineEverywhere, out);
		CHECK(out[2] == Regime::LINE,
		      "tagRegimes PROMOTES a sine tag above the seam (velcal mislabels the handoff point)");

		// ...and never demotes: an explicit LINE below the seam is a real hysteresis-band measurement.
		Regime lineLow[] = {Regime::SINE, Regime::LINE, Regime::LINE};
		tagRegimes(ok3, 3, POLE_PAIRS, UP, lineLow, out);
		CHECK(out[1] == Regime::LINE, "tagRegimes keeps an explicit line tag below the seam");

		// The promotion is what moves lineFloor(), i.e. the soft-sensor gate: with the tags taken at
		// face value the floor would be the NEXT point up.
		Crossover cxu{UP, UP};
		Regime tagged[3];
		tagRegimes(ok3, 3, POLE_PAIRS, UP, sineEverywhere, tagged);
		SpeedProfile up(ok3, 3, POLE_PAIRS, &cxu, tagged);
		float lf = 0.0f;
		CHECK(up.lineFloor(lf) && fabsf(lf - 900.0f) < 1e-3f, "lineFloor reports the promoted point");
	}

	{   // ================= staged thermal protection (B.7) =================
		// The ESC's own protection starts at 80 C (its lowest selectable onset) and steps the PWM down
		// in ~5 C stages; this limiter lives entirely BELOW that, so the two never fight. The rules it
		// has to obey: never seen a temperature == disabled (no limp mode), seen-then-lost releases the
		// limit (fail open) but reports STALE (never silent), and while the ceiling is what is clamping
		// the command neither the observer nor the stall guard may draw conclusions from it.
		CurvePoint tpt[12]; Crossover tcx{UP_ERPM, DN_ERPM};
		SpeedProfile tp = Prof::build(tpt, &tcx, 1.0f);
		const float TGT = 600.0f;

		struct Rig {                       // a controller wired the way main.cpp wires one
			SimEsc sim;
			VelocityController vc;
			Rig(SpeedProfile& p, bool thermal) : vc(sim, p) {
				vc.kp = 0.06f; vc.ki = 0.25f; vc.trim_max = 200.0f; vc.slew_rpm_s = 2000.0f;
				vc.stall_secs = 1.0f;
				if (thermal) { vc.warn_temp = DEFAULT_TEMP_WARN_C; vc.max_temp = DEFAULT_TEMP_ABORT_C; }
			}
			void run(int n, float temp) { for (int i = 0; i < n; i++) { sim.tempC = temp; vc.step(TDT); } }
		};

		// (1) below warn_temp: BIT-IDENTICAL to a run that never reports a temperature at all.
		{
			Rig warm(tp, true), none(tp, true);
			warm.vc.setTarget(TGT); none.vc.setTarget(TGT);
			bool same = true;
			for (int t = 0; t < 400; t++) {
				warm.sim.tempC = 50.0f; none.sim.tempC = 0.0f;
				warm.vc.step(TDT); none.vc.step(TDT);
				if (warm.vc.command() != none.vc.command()) same = false;
			}
			CHECK(same, "thermal: below warn_temp the command is bit-identical to the no-temperature run");
			CHECK(warm.vc.tempState() == TempState::OK && none.vc.tempState() == TempState::NONE,
			      "thermal: states differ though — OK (seen, cool) vs NONE (never seen)");
			CHECK(warm.vc.tempLimit() == 1.0f && none.vc.tempLimit() == 1.0f, "thermal: no derate below warn_temp");
			CHECK(none.vc.temp() == 0.0f, "thermal: 'never seen' reports no temperature (not a stale 0)");
		}

		// (2) inside the band: the command is monotonically reduced, and the release is bump-free.
		{
			Rig r(tp, true);
			r.vc.setTarget(TGT);
			r.run(400, 50.0f);
			int cold = r.vc.command();
			int prev = cold, lowest = cold; bool monotone = true; float trim_peak = 0.0f;
			for (int t = 0; t < 140; t++) {                       // 50 -> 78 C over 2.8 s
				r.sim.tempC = 50.0f + 0.2f * (float)(t + 1);
				r.vc.step(TDT);
				if (fabsf(r.vc.trim()) > trim_peak) trim_peak = fabsf(r.vc.trim());
				int c = r.vc.command();
				// While the ceiling is the binding constraint the command IS the ceiling, which falls
				// monotonically with temperature. (Above the ceiling the PI is free to move either way.)
				if (c == (int)(1000.0f * r.vc.tempLimit()) && c > prev) monotone = false;
				prev = c; if (c < lowest) lowest = c;
			}
			printf("   thermal derate: cold cmd=%d  hot cmd=%d  limit=%.2f  state=%s\n",
			       cold, r.vc.command(), r.vc.tempLimit(), tempStateName(r.vc.tempState()));
			CHECK(r.vc.tempState() == TempState::DERATE, "thermal: in-band state is DERATE");
			CHECK(monotone, "thermal: inside the band the commanded magnitude only falls");
			CHECK(lowest < cold * 6 / 10, "thermal: the derate really cuts the command (>40 %)");
			// Cool back down. The integrator was FROZEN under the ceiling, so the only thing left on
			// release is proportional action against the real tracking error (the rotor genuinely did
			// slow down): no integral hump, and the trim never goes anywhere near trim_max. What paces
			// the return to the pre-heat command is the PLANT re-accelerating, not the loop unwinding.
			int back = -1, cmd_peak = 0;
			for (int t = 0; t < 200; t++) {
				r.sim.tempC = 50.0f; r.vc.step(TDT);
				if (fabsf(r.vc.trim()) > trim_peak) trim_peak = fabsf(r.vc.trim());
				if (r.vc.command() > cmd_peak) cmd_peak = r.vc.command();
				if (back < 0 && abs(r.vc.command() - cold) <= 3) back = t;
			}
			printf("   thermal release: peak trim=%.1f (trim_max=%.0f)  peak cmd=%d vs pre-heat %d  "
			       "back within 3 units after %d tick(s)\n", trim_peak, r.vc.trim_max, cmd_peak, cold, back);
			CHECK(trim_peak < 0.25f * r.vc.trim_max,
			      "thermal: NO integrator windup — the trim never approaches trim_max under the ceiling");
			CHECK(cmd_peak <= cold + 25, "thermal: the release has no integral hump (proportional action only)");
			CHECK(back >= 0 && back <= 120, "thermal: the loop returns to the pre-heat command as the rotor recovers");
			CHECK(r.vc.tempLimit() == 1.0f && r.vc.tempState() == TempState::OK, "thermal: limit released on cooling");
		}

		// (3) one spurious over-limit frame must NOT abort; a sustained one must.
		{
			Rig r(tp, true);
			r.vc.setTarget(TGT);
			r.run(200, 50.0f);
			r.sim.tempC = 85.0f;                                  // ONE garbled frame
			Status s1 = r.vc.step(TDT);
			r.sim.tempC = 50.0f;
			Status s2 = r.vc.step(TDT), s3 = r.vc.step(TDT);
			CHECK(s1 == Status::OK && s2 == Status::OK && s3 == Status::OK,
			      "thermal: a single spurious over-limit reading does not abort");
			Status sus = Status::OK;
			for (int t = 0; t < 10 && sus == Status::OK; t++) { r.sim.tempC = 85.0f; sus = r.vc.step(TDT); }
			CHECK(sus == Status::ABORT_TEMP, "thermal: a SUSTAINED over-limit aborts (ABORT_TEMP)");
			CHECK(r.vc.peakTemp() >= 85.0f, "thermal: peakTemp keeps the high-water mark");
		}

		// (3b) max_temp == 0 still means "no temp check at all" — the pre-B.7 contract.
		{
			Rig r(tp, false);                                     // warn_temp/max_temp both left at 0
			r.vc.setTarget(TGT);
			Status s = Status::OK;
			for (int t = 0; t < 300 && s == Status::OK; t++) { r.sim.tempC = 200.0f; s = r.vc.step(TDT); }
			CHECK(s == Status::OK && r.vc.tempLimit() == 1.0f,
			      "thermal: max_temp==0 disables the check entirely (200 C changes nothing)");
		}

		// (4) seen, then lost: RELEASE the limit (fail open) and report STALE (never silently).
		{
			Rig r(tp, true);
			r.vc.setTarget(TGT);
			r.run(400, 50.0f);
			int cold = r.vc.command();
			r.run(100, 75.0f);                                    // derating
			CHECK(r.vc.tempLimit() < 1.0f && r.vc.tempState() == TempState::DERATE, "thermal: derating before the dropout");
			for (int t = 0; t < 40; t++) { r.sim.tempC = 0.0f; r.vc.step(TDT); }   // 0.8 s: still inside the grace window
			CHECK(r.vc.tempLimit() < 1.0f, "thermal: a short dropout does NOT release the limit (grace window)");
			for (int t = 0; t < 200; t++) { r.sim.tempC = 0.0f; r.vc.step(TDT); }  // past temp_stale_secs
			CHECK(r.vc.tempState() == TempState::STALE && r.vc.tempLimit() == 1.0f,
			      "thermal: seen-then-stale releases to 1.0 and reports STALE (fails open, loudly)");
			CHECK(abs(r.vc.command() - cold) <= 3, "thermal: ...and the loop recovers the pre-heat command");
		}

		// (5) while the thermal ceiling is the binding clamp, ABORT_STALL cannot fire...
		{
			Rig r(tp, true);
			r.sim.never_live = true;                              // the ESC never reaches 6-step
			r.vc.setTarget(TGT);
			Status s = Status::OK;
			for (int t = 0; t < 200 && s == Status::OK; t++) { r.sim.tempC = 69.0f; s = r.vc.step(TDT); }
			CHECK(r.vc.tempLimit() < 1.0f && s == Status::OK,
			      "thermal: a binding ceiling suppresses ABORT_STALL (we withheld the command, not the ESC)");
			Status s2 = Status::OK;                               // ...and the guard is NOT dead: cool it
			for (int t = 0; t < 200 && s2 == Status::OK; t++) { r.sim.tempC = 50.0f; s2 = r.vc.step(TDT); }
			CHECK(s2 == Status::ABORT_STALL, "thermal: once the ceiling releases, the stall guard fires again");
		}

		// (6) ...and d_hat does not accumulate the withheld command as load.
		{
			Rig r(tp, true);
			r.vc.dob = 0.8f; r.vc.dob_tau = 0.08f;
			r.sim.disturb = 100.0f;                               // a REAL load, so d_hat is non-zero
			r.vc.setTarget(TGT);
			r.run(500, 50.0f);
			float d_cold = r.vc.dhat();
			CHECK(fabsf(d_cold) > 20.0f, "thermal: (DOB case) a real load is being observed first");
			r.run(150, 78.0f);                                    // hard derate for 3 s
			printf("   thermal DOB: d_hat cold=%.2f  under derate=%.2f rpm\n", d_cold, r.vc.dhat());
			CHECK(r.vc.tempLimit() < 1.0f, "thermal: (DOB case) the ceiling is binding");
			CHECK(fabsf(r.vc.dhat() - d_cold) < 1e-3f, "thermal: d_hat is FROZEN while the ceiling binds");
		}

		// (6b) THE DERATE MUST NOT SELF-OSCILLATE. temp_clamped_ is read one tick late, so suppressing
		//      the DOB's OUTPUT (rather than only its update) made the ceiling self-exciting: clamped ->
		//      comp 0 next tick -> command drops below the ceiling -> unclamped -> comp returns ->
		//      clamped. Measured before the fix on this exact rig: the command square-waved 537<->590
		//      for seconds, 118 ticks with a >10-count single-tick jump, max jump 70 (~7 % of full
		//      scale), at the core0 loop rate — with the integrator freeze and the ABORT_STALL
		//      suppression flickering along with it. Only reachable with dob > 0, which is what the
		//      bench ESC is configured with today.
		{
			Rig r(tp, true);
			r.vc.dob = 0.8f; r.vc.dob_tau = 0.08f;
			r.sim.disturb = 100.0f;
			r.vc.setTarget(TGT);
			r.run(500, 50.0f);
			int prev = r.vc.command(), big = 0, worst = 0, flips = 0;
			bool wasClamped = false;
			for (int t = 0; t < 200; t++) {                   // 60 -> 80 C over 4 s: the ceiling sweeps
				r.sim.tempC = 60.0f + 0.1f * (float)t;        // straight past the operating command
				r.vc.step(TDT);
				int c = r.vc.command(), j = abs(c - prev); prev = c;
				if (j > 10) big++;
				if (j > worst) worst = j;
				bool clamped = r.vc.tempLimit() < 1.0f && c == (int)(1000.0f * r.vc.tempLimit());
				if (clamped != wasClamped) flips++;
				wasClamped = clamped;
			}
			printf("   thermal derate ramp with dob=0.8: ticks with >10-count jump=%d  worst jump=%d  "
			       "clamp edges=%d\n", big, worst, flips);
			CHECK(big == 0, "thermal: no tick-rate command limit cycle under a slow derate ramp (dob on)");
			CHECK(worst <= 10, "thermal: worst single-tick command jump stays small through the sweep");
			CHECK(flips <= 2, "thermal: the clamp engages once and stays engaged (no chatter)");
		}

		// (6c) EDT frames are slower than the control loop, so the abort debounce has to count FRAMES.
		//      With temp_every = 20 (one frame per 0.4 s) three consecutive over-limit READINGS take
		//      ~1.2 s, not 3 ticks — an adapter-side dedupe would have made this untestable from here.
		{
			Rig r(tp, true);
			r.sim.temp_every = 20;
			r.vc.setTarget(TGT);
			r.run(500, 50.0f);
			int ticks = 0; Status s = Status::OK;
			for (; ticks < 200 && s == Status::OK; ticks++) { r.sim.tempC = 85.0f; s = r.vc.step(TDT); }
			printf("   thermal debounce with 1 EDT frame per %d ticks: aborted after %d ticks (%.2f s)\n",
			       r.sim.temp_every, ticks, ticks * TDT);
			// Three readings span TWO inter-frame gaps (the first can land on the very next tick), so the
			// floor is 2 * temp_every — and the point is that it is nowhere near TEMP_ABORT_READINGS
			// ticks, which is what an adapter reporting its held value every tick would have collapsed to.
			CHECK(s == Status::ABORT_TEMP && ticks >= 2 * r.sim.temp_every
			      && ticks > TEMP_ABORT_READINGS,
			      "thermal: the abort debounce counts EDT FRAMES, not control ticks");
		}

		// (6d) reset() clears the debounce counter, so a re-armed controller cannot instantly re-abort
		//      on a counter left over from the previous run (and the abort only fires ON a reading).
		{
			Rig r(tp, true);
			r.vc.setTarget(TGT);
			r.run(200, 50.0f);
			Status s = Status::OK;
			for (int t = 0; t < 10 && s == Status::OK; t++) { r.sim.tempC = 85.0f; s = r.vc.step(TDT); }
			CHECK(s == Status::ABORT_TEMP, "thermal: (reset case) aborted as expected");
			r.vc.reset();                                     // what arm() does
			r.vc.setTarget(TGT);
			r.sim.tempC = 50.0f;                              // ESC has cooled; no new frame yet this tick
			Status again = r.vc.step(TDT);
			CHECK(again == Status::OK, "thermal: reset() clears the over-limit counter (no instant re-abort)");
		}

		// (7) the legacy EscIo contract (temperature only via readTele) still gets its abort.
		{
			Rig r(tp, true);
			r.sim.legacy_temp = true;
			r.vc.setTarget(TGT);
			r.run(200, 50.0f);
			CHECK(r.vc.temp() == 50.0f, "thermal: a readTele-only adapter still reports temperature");
			Status s = Status::OK;
			for (int t = 0; t < 20 && s == Status::OK; t++) { r.sim.tempC = 90.0f; s = r.vc.step(TDT); }
			CHECK(s == Status::ABORT_TEMP, "thermal: ...and still aborts over max_temp");
		}
	}

	// reset() must clear setpoint_, or slew_rpm_s is defeated on the second and every later run.
	// `setpoint_ = 0` exists in exactly one other place (step()'s stop branch) and Thruster::poll()
	// returns before step() on a stop, so on hardware that assignment is unreachable: the setpoint
	// survived every stop at its last target and the next setTarget(same) slewed by nothing. Measured
	// 2026-08-01 on the 930KV: slew 2000/500/50 rpm/s gave identical trajectories.
	{
		CurvePoint pts[] = {{60,20},{200,300},{400,1500},{700,4200}};
		SpeedProfile prof(pts, 4, 7);
		SimEsc sim; VelocityController vc(sim, prof);
		vc.slew_rpm_s = 50.0f;
		vc.setTarget(4000.0f);
		for (int i = 0; i < 20; i++) vc.step(0.01f);          // 0.2 s at 50 rpm/s -> ~10 rpm
		float crept = vc.setpoint();
		CHECK(crept > 0.0f && crept < 60.0f, "the setpoint really is slewing (< 60 rpm after 0.2 s)");
		vc.setTarget(0.0f);
		vc.reset();
		CHECK(vc.setpoint() == 0.0f, "reset() clears the setpoint — without this, slew is a no-op on run 2");
		vc.setTarget(4000.0f);
		for (int i = 0; i < 20; i++) vc.step(0.01f);
		CHECK(vc.setpoint() < 60.0f, "...so the SECOND run slews exactly like the first");
	}

	// ================================================================================================
	// STRUCTURAL AUTHORITY LIMITS (2026-08-01). Both of these exist because of one bench failure and
	// one spec violation, and the point of them is that neither depends on the sensor being fixed.
	//
	// THE SPEC VIOLATION: the header of vel_control.h says the liveness gate rejects "forced-sine's
	// stale ~0" — this controller was written for an ESC that reports ~0 when it is NOT BEMF-locked.
	// BlueGill reports a floored 182 mech rpm instead, so TELE_MIN_MECH_RPM = 50 cannot separate "the
	// rotor is turning at 182" from "the ESC is lying about a rotor that is not turning". The stall
	// scenario already in this file (SimEsc::never_live) models the SPEC — readTele returns false — so
	// it could never have caught this. LyingEsc below models the HARDWARE.
	// ================================================================================================
	struct LyingEsc : EscIo {
		float rpm = 0.0f, dt = 0.02f, tau = 0.15f, floor_rpm = 60.0f, disturb = 0.0f;
		bool  stuck = false;            // the rotor never catches the field (cogs in place)
		int   last_sent = 0;
		void thrust(int cmd) override {
			last_sent = cmd;
			float target = stuck ? 0.0f : copysignf(lineMech(cmd), (float)cmd);
			if (cmd == 0) target = 0.0f;
			if (target != 0.0f) { float m = fabsf(target) - disturb; if (m < 0.0f) m = 0.0f;
			                      target = copysignf(m, target); }
			rpm += (target - rpm) * (dt / tau);
		}
		// THE LIE: a frame always arrives, and it never reads below the floor however still the rotor is.
		bool readTele(float& mechRpm, float& tempC_out) override {
			tempC_out = 0.0f;
			mechRpm = fabsf(rpm) > floor_rpm ? fabsf(rpm) : floor_rpm;
			return true;
		}
		bool readTemp(float&, uint32_t&) override { return false; }
	};

	// -- (b) AUTHORITY REQUIRES CORROBORATION. A stuck rotor behind a floored telemetry: the gate says
	//    LIVE on every tick, so before this the PI took full authority against a fabricated error and
	//    both feedback paths saturated. Measured on the 930KV: trim_max 80 + dob_max 150 = 230 over a
	//    steady-state feed-forward of 641, and the startup command was 872. --
	{
		CurvePoint lp[12]; Crossover lc{UP_ERPM, DN_ERPM};
		SpeedProfile P = Prof::build(lp, &lc, 1.0f);
		const float TARGET = 400.0f;
		float ff = P.thrustFor(TARGET);
		// The switch is track_min_frac, so run BOTH sides of it on the identical scenario.
		float cmd_off = 0.0f, cmd_on = 0.0f, w_off = 0.0f, w_on = 0.0f, end_off = 0.0f, end_on = 0.0f;
		for (int pass = 0; pass < 2; pass++) {
			LyingEsc sim; sim.stuck = true;
			VelocityController vc(sim, P);
			vc.kp = 0.06f; vc.ki = 0.25f; vc.trim_max = 80.0f; vc.slew_rpm_s = 2000.0f;
			vc.dob = 0.8f; vc.dob_max = 150.0f; vc.dob_settle_secs = 0.2f;
			vc.stall_secs = 1e9f; vc.over_speed_rpm = 1e9f;
			vc.track_min_frac = (pass == 0) ? 0.0f : 0.25f;   // pass 0 = the old behaviour
			vc.setTarget(TARGET);
			float peak = 0.0f, last = 0.0f;
			for (int t = 0; t < 400; t++) {
				vc.step(0.02f);
				float c = fabsf((float)vc.command());
				if (c > peak) peak = c;
				last = c;
			}
			if (pass == 0) { cmd_off = peak; w_off = vc.authority(); end_off = last; }
			else           { cmd_on  = peak; w_on  = vc.authority(); end_on  = last; }
		}
		printf("   lying sensor, stuck rotor: ff=%.0f   track_min_frac 0 -> peak %.0f end %.0f (w %.2f)"
		       "   0.25 -> peak %.0f end %.0f (w %.2f)\n",
		       ff, cmd_off, end_off, w_off, cmd_on, end_on, w_on);
		CHECK(cmd_off > ff + 100.0f, "WITHOUT corroboration the loop slams the command far above the FF");
		CHECK(w_off > 0.99f, "...because liveness alone gave the PI FULL authority on a lying sensor");
		CHECK(w_on < 0.01f, "WITH corroboration the authority is withdrawn to zero");
		CHECK(end_on <= ff + 1.0f, "...and the command settles at pure feed-forward, which is the ask");
		// The excursion is not zero and must not be claimed to be: w_ FADES over blend_secs, so the PI
		// keeps a shrinking share of authority for ~0.3 s before the withdrawal completes. What the
		// fix does is bound that to the fade instead of to trim_max + dob_max.
		CHECK(cmd_on - ff < 0.15f * (cmd_off - ff),
		      "...and the transient excursion is a fraction of the unguarded slam (the blend_secs fade)");
		// live() itself must NOT be suppressed: the over-speed and stall aborts are safety guards and
		// have to keep firing on the raw frame whatever the authority logic thinks of it.
		{
			LyingEsc sim; sim.stuck = true;
			VelocityController vc(sim, P);
			vc.track_min_frac = 0.25f; vc.stall_secs = 1e9f;
			vc.setTarget(TARGET);
			for (int t = 0; t < 100; t++) vc.step(0.02f);
			CHECK(vc.live() && !vc.corroborated(),
			      "liveness stays TRUE (the guards still see the frame); only corroboration goes false");
		}
	}

	// -- (b) MUST NOT break the case the DOB exists for: a REAL load in water is a rpm deficit of tens
	//    of percent, not 95 %, so corroboration has to leave the observer its whole legitimate range. --
	{
		CurvePoint lp[12]; Crossover lc{UP_ERPM, DN_ERPM};
		SpeedProfile P = Prof::build(lp, &lc, 1.0f);
		LyingEsc sim; sim.disturb = 120.0f;          // a genuine load: ~30 % of the achieved speed
		VelocityController vc(sim, P);
		vc.kp = 0.06f; vc.ki = 0.25f; vc.trim_max = 300.0f; vc.slew_rpm_s = 2000.0f;
		vc.dob = 0.8f; vc.dob_max = 150.0f; vc.dob_settle_secs = 0.2f;
		vc.track_min_frac = 0.25f;
		vc.setTarget(400.0f);
		for (int t = 0; t < 400; t++) vc.step(0.02f);
		printf("   real 120 rpm load: w=%.2f corroborated=%d dhat=%.0f err=%.0f\n",
		       vc.authority(), (int)vc.corroborated(), vc.dhat(), 400.0f - fabsf(vc.measured()));
		CHECK(vc.corroborated() && vc.authority() > 0.99f,
		      "a genuine load still CORROBORATES — the loop keeps full authority under it");
		CHECK(fabsf(400.0f - fabsf(vc.measured())) < 40.0f, "...and still tracks the target under it");
	}

	// -- (a) ONE BUDGET for both feedback paths. They were clamped independently, so the reachable
	//    deviation from the FF was their SUM — a number nobody chose. Corroboration is disabled here on
	//    purpose, to test the budget ALONE against the same slam. --
	{
		CurvePoint lp[12]; Crossover lc{UP_ERPM, DN_ERPM};
		SpeedProfile P = Prof::build(lp, &lc, 1.0f);
		const float TARGET = 400.0f;
		float ff = P.thrustFor(TARGET);
		float stacked[2] = {0.0f, 0.0f};
		for (int pass = 0; pass < 2; pass++) {
			LyingEsc sim; sim.stuck = true;
			VelocityController vc(sim, P);
			vc.kp = 0.06f; vc.ki = 0.25f; vc.trim_max = 80.0f; vc.slew_rpm_s = 2000.0f;
			vc.dob = 0.8f; vc.dob_max = 150.0f; vc.dob_settle_secs = 0.2f;
			vc.stall_secs = 1e9f; vc.over_speed_rpm = 1e9f; vc.track_min_frac = 0.0f;
			vc.fb_max = (pass == 0) ? 1e6f : 0.0f;      // pass 0 = the old independent clamps
			vc.setTarget(TARGET);
			float peak = 0.0f;
			for (int t = 0; t < 400; t++) { vc.step(0.02f); float d = fabsf((float)vc.command()) - ff;
			                                if (d > peak) peak = d; }
			stacked[pass] = peak;
		}
		printf("   stacking: trim_max 80 + dob_max 150 -> deviation %.0f unbounded, %.0f budgeted "
		       "(budget = max(80,150) = 150)\n", stacked[0], stacked[1]);
		CHECK(stacked[0] > 200.0f, "unbudgeted, the two paths STACK to more than either was allowed");
		CHECK(stacked[1] <= 150.0f + 1.0f, "budgeted, the total cannot exceed max(trim_max, dob_max)");
		CHECK(stacked[1] < stacked[0] - 50.0f, "...which is a real bound, not a rounding difference");
	}

	// -- (a) is a NO-OP where it should be: with the observer off there is only one path, and opposing
	//    paths sum small. This bounds STACKING, not disagreement, and it never trims the PI. --
	{
		CurvePoint lp[12]; Crossover lc{UP_ERPM, DN_ERPM};
		SpeedProfile P = Prof::build(lp, &lc, 1.2f);   // FF over-reports speed -> the PI must work
		LyingEsc sim;
		VelocityController vc(sim, P);
		vc.kp = 0.06f; vc.ki = 0.25f; vc.trim_max = 200.0f; vc.slew_rpm_s = 2000.0f; vc.dob = 0.0f;
		vc.setTarget(400.0f);
		float peak = 0.0f;
		for (int t = 0; t < 400; t++) { vc.step(0.02f); float a = fabsf(vc.trim()); if (a > peak) peak = a; }
		CHECK(peak > 20.0f, "with dob = 0 the PI still uses its own authority freely");
		CHECK(vc.fbMax() == 200.0f, "...and the derived budget is just trim_max when there is no observer");
		VelocityController v2(sim, P);
		v2.trim_max = 80.0f; v2.dob_max = 150.0f;
		CHECK(v2.fbMax() == 150.0f, "the derived budget is the LARGER of the two, never their sum");
		v2.fb_max = 42.0f;
		CHECK(v2.fbMax() == 42.0f, "an explicit fb_max overrides the derivation");
	}

	printf(failures ? "\n%d CHECK(S) FAILED\n" : "\nALL CHECKS PASSED\n", failures);
	return failures ? 1 : 0;
}
