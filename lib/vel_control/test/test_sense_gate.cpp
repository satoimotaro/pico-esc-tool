// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// Native (host g++) test for the soft-sensor validity window (sense_gate.h). The gate exists because
// the OLD gate — `vc.live() && |vc.measured()| >= lineFloor()` — read both of its terms from `tele`,
// and the 2026-07-30 bench run proved `tele` is not a witness below the seam: a constant VIRTUAL
// 182 rpm for every command from 60 to 450 while the rotor went 39 -> 170 rpm. The headline case here
// is exactly that tick: live, plausible rpm, and it MUST come out invalid.
//     g++ -std=c++17 -O2 -Wall -o /tmp/tsg test_sense_gate.cpp && /tmp/tsg
#include "../sense_gate.h"
#include <cstdio>
#include <cmath>

using namespace vel;

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); failures++; } \
	else printf("ok: %s\n", msg); } while (0)

static const float DT       = 0.02f;      // 50 Hz, the rate the rest of the suite ticks at
static const float CROSS_UP = 36.0f;      // the bench ESC's sine_cross_up (moves the seam AND the duty)

// The 350KV bench motor + its gate, wired exactly the way Thruster::poll() wires them: the seam and the
// duty BOTH come from the parametric model at the configured cross_up (so a wrong cross_up shows up
// here), the rpm floor from the profile's 6-step landing, and the off-point band compares DUTIES.
struct Bench {
	MotorModel mm{350.0f, 7, 11.1f};
	SenseGate  g;
	float loadV = 0.0f, vref = 0.0f, zeroCmd = 0.0f;

	explicit Bench(float crossUp = CROSS_UP, bool configure = true) {
		mm.setCrossUp(crossUp);
		if (configure) g.configure(mm.crossUpErpm(), mm.sixstepFloorRpm());
	}
	void tick(float cmd, float rpm, bool live = true, float dt = DT) {
		float zeroDuty = (zeroCmd > 0.0f) ? mm.duty6(zeroCmd) : 0.0f;
		g.update(live, cmd, rpm, mm.duty6(cmd), loadV, vref, zeroDuty, dt);
	}
	void ticks(int n, float cmd, float rpm, bool live = true, float dt = DT) {
		for (int i = 0; i < n; i++) tick(cmd, rpm, live, dt);
	}
};

int main() {
	// -- the command floor is a firmware property, but it is the DUTY floor, not the seam (2026-07-31).
	//    handoffCmdFor(1400) was pinned at 560.2 while the floor was the seam inverted through the
	//    forced-sine curve; the 1-count sweeps put the real handoff ~47 counts lower at every cross_up
	//    (measured 485.0/515.5/734.5 at 34/36/50). 1400 eRPM is cross_up 35.84, so 513.6. --
	CHECK(fabsf(handoffCmdFor(1400.0f) - 513.4f) < 0.5f, "handoffCmdFor(1400) ~= 513.4 (was 560)");
	{
		// A ROUND TRIP against the model, and a stronger pin than the old handoffCmdFor(1333.0f) ~= 533:
		// 1333 is the FLOORED HANDOFF_ERPM, not a byte value, so it pinned a seam no configuration
		// produces. 34 * 39.0625 is the real default byte, and the new law is KV-, supply- AND
		// pole-pair-independent, so all three models must land on the same command as the free function.
		MotorModel m7(350.0f, 7, 11.1f), m6(350.0f, 6, 11.1f), m12(120.0f, 12, 22.2f);
		const float SEAM34 = 34.0f * SINE_CROSS_UP_ERPM_PER_UNIT;
		CHECK(fabsf(handoffCmdFor(SEAM34) - 484.21f) < 0.1f, "handoffCmdFor(34 bytes) ~= 484.21");
		CHECK(fabsf(handoffCmdFor(SEAM34) - m7.handoffCmd()) < 0.05f &&
		      fabsf(handoffCmdFor(SEAM34) - m6.handoffCmd()) < 0.05f &&
		      fabsf(handoffCmdFor(SEAM34) - m12.handoffCmd()) < 0.05f,
		      "handoffCmdFor is KV/pp/V independent (matches MotorModel::handoffCmd for pp=6/7/12)");
		// ...and it must never fail OPEN. A seam the firmware refuses to hand off at, an unconfigured
		// one, and a garbage one all report 0 -> the caller's gate closes.
		CHECK(handoffCmdFor(30.0f * SINE_CROSS_UP_ERPM_PER_UNIT) == 0.0f,
		      "a refused seed-tick window (cross_up 30) reports no floor at all");
		// ...and the OTHER refusal, which this function must agree with MotorModel on: above cross_up 66
		// the handoff command is past the +-1000 the protocol carries. Before that was folded into the
		// predicate, handoffCmdFor bounds-checked and MotorModel::handoffCmd did not, so the two paths
		// disagreed for cross_up 67..113 and a real command aliased HANDOFF_CMD_NONE.
		CHECK(handoffCmdFor(66.0f * SINE_CROSS_UP_ERPM_PER_UNIT) > 990.0f,
		      "cross_up 66 still has a reachable floor (992.6)");
		CHECK(handoffCmdFor(67.0f * SINE_CROSS_UP_ERPM_PER_UNIT) == 0.0f,
		      "cross_up 67 (1008.5, past CMD_FULL_SCALE) closes the gate instead of reporting it");
		{
			MotorModel m66(350.0f, 7, 11.1f), m67(350.0f, 7, 11.1f);
			m66.setCrossUp(66.0f); m67.setCrossUp(67.0f);
			CHECK(fabsf(handoffCmdFor(66.0f * SINE_CROSS_UP_ERPM_PER_UNIT) - m66.handoffCmd()) < 0.05f &&
			      !m67.handoffPossible(),
			      "handoffCmdFor and MotorModel::handoffCmd agree on BOTH sides of the boundary");
		}
		CHECK(handoffCmdFor(0.0f) == 0.0f && handoffCmdFor(-1400.0f) == 0.0f && handoffCmdFor(NAN) == 0.0f
		      && handoffCmdFor(INFINITY) == 0.0f && handoffCmdFor(1e-30f) == 0.0f,
		      "zero / negative / NaN / inf / denormal seam -> no floor (never a negative one, which "
		      "would open the gate). Python pins the same six in test_motor_model.py.");
	}

	// -- THE COMMAND FLOOR MUST TRACK `sine_cross_up`. cross_up does two independent things: it shifts
	//    the 6-step duty AND it moves the seam (eRPM = byte * 39.0625, floored at the firmware minimum).
	//    Pinning the seam to that minimum left the gate opening at cmd 560 while the real handoff at
	//    cross_up 50 is 781 — a ~220-count band of pure forced sine reported as valid. --
	{
		MotorModel m34(350.0f, 7, 11.1f), m36(350.0f, 7, 11.1f), m50(350.0f, 7, 11.1f);
		m34.setCrossUp(34.0f); m36.setCrossUp(36.0f); m50.setCrossUp(50.0f);
		CHECK(fabsf(m34.crossUpErpm() - HANDOFF_ERPM) < 0.5f,
		      "cross_up 34 floors at the firmware minimum handoff (1328 < 1333)");
		// Thruster's ffcx_ initialiser is the literal HANDOFF_ERPM because static init cannot call mm_;
		// that is only correct while the DEFAULT cross_up resolves to the same seam. Pin it.
		MotorModel mdef(350.0f, 7, 11.1f);
		CHECK(fabsf(mdef.crossUpErpm() - HANDOFF_ERPM) < 0.5f &&
		      fabsf(mdef.crossUp() - DUTY6_CROSSUP_REF) < 0.01f,
		      "the DEFAULT cross_up resolves to the firmware handoff floor (thruster.h's ffcx_ init)");
		CHECK(fabsf(m36.crossUpErpm() - 1406.25f) < 0.01f, "cross_up 36 -> 1406 eRPM");
		CHECK(fabsf(m50.crossUpErpm() - 1953.125f) < 0.01f, "cross_up 50 -> 1953 eRPM");
		Bench b34(34.0f), b36(36.0f), b50(50.0f);
		printf("   cmdfloor tracks cross_up: 34 -> %.0f   36 -> %.0f   50 -> %.0f\n",
		       b34.g.cmdFloor(), b36.g.cmdFloor(), b50.g.cmdFloor());
		CHECK(b36.g.cmdFloor() > b34.g.cmdFloor() && b50.g.cmdFloor() > b36.g.cmdFloor(),
		      "raising cross_up RAISES cmdfloor (the seam moved, so the handoff command did too)");
		// 541.8 = 515.99 * 1.05, was 590.9 = 563 * 1.05. The window moved ~49 counts OUTWARD, so the
		// safety argument is explicit: the duty law's worst LOW-side residual against the three
		// measured brackets is -0.79 counts, against a 26-count enter margin here (37 at cross_up 50).
		CHECK(fabsf(b36.g.cmdFloor() - 541.8f) < 1.0f, "cross_up 36 -> cmdfloor 541.8 (516.0 * 1.05)");
		CHECK(fabsf(b36.g.cmdExit() - 526.3f) < 1.0f, "cross_up 36 -> cmdexit 526.3 (516.0 * 1.02)");
		// The 930KV's telemetry is honest from the first handed-off command (cmd 516: tele 536.8 vs
		// encoder 522.3) and on a floor below it (cmd 515: tele 203 vs a rotor oscillating about 198),
		// so the floor sits above the onset it is meant to protect, by the enter margin and no more.
		CHECK(b36.g.cmdExit() > 515.99f, "even the STAY threshold is above the measured honest onset");
		// The regression that mattered: at cross_up 50 a command of 600 is deep in forced sine, and the
		// seam-pinned gate would have latched valid there for as long as the caller held it.
		b50.ticks(60, 600.0f, 1500.0f);
		CHECK(!b50.g.valid() && b50.g.why() == SenseWhy::CMD_LOW,
		      "at cross_up 50, cmd 600 (forced sine) is CMD_LOW, not valid");
		b36.ticks(60, 600.0f, 1500.0f);
		CHECK(b36.g.valid(), "...while at cross_up 36 the same command is genuinely above the seam");
		// And the `loadtest` operating point stays inside the window at the bench's cross_up, which is
		// what bounds the enter margin from above (614 / 563 = 1.091, so 5 % fits and 9 % would not).
		Bench lt(36.0f);
		lt.ticks(40, 613.6f, 1200.0f);
		CHECK(lt.g.valid(), "loadtest's 1200 rpm (cmd 614) stays inside the window at cross_up 36");
	}

	// -- FAIL CLOSED ON GARBAGE. Every condition is tested as the negation of what must hold, so a NaN
	//    (which makes every one-directional comparison false) is rejected instead of walking through to
	//    why=OK. Latent today — Thruster feeds int-cast commands through a linear duty map — but this
	//    header is documented as a portable fail-closed library, so it has to hold on its own terms. --
	{
		const float nan = NAN, inf = INFINITY;
		Bench b;
		b.ticks(50, nan, nan);
		CHECK(!b.g.valid() && b.g.why() == SenseWhy::CMD_LOW, "NaN command/rpm/duty -> gate CLOSED (CMD_LOW)");
		Bench c;
		c.ticks(50, 652.0f, nan);
		CHECK(!c.g.valid() && c.g.why() == SenseWhy::RPM_LOW, "NaN rpm alone -> RPM_LOW, not OK");
		Bench d;
		d.g.update(true, 652.0f, 1600.0f, nan, 0.0f, 0.0f, 0.0f, DT);
		CHECK(!d.g.valid() && d.g.why() == SenseWhy::DUTY_HIGH, "NaN duty alone -> DUTY_HIGH, not OK");
		Bench e;
		e.ticks(50, inf, inf);
		CHECK(!e.g.valid(), "Inf command -> still closed (duty ceiling rejects it)");
		// ...and a NaN in the tier inputs closes the TIERS rather than opening them.
		Bench f;
		f.vref = 11.0f; f.zeroCmd = 652.0f; f.loadV = nan;
		f.ticks(40, 652.0f, 1600.0f);
		CHECK(f.g.valid() && !f.g.vbattOk(), "NaN droop -> vbattOk closed (valid unaffected)");
		Bench h;
		h.vref = nan; h.zeroCmd = 652.0f;
		h.ticks(40, 652.0f, 1600.0f);
		CHECK(h.g.valid() && !h.g.vbattOk(), "NaN vref -> vbattOk closed");
	}

	// -- THE BENCH CASE: forced sine reports a virtual 182 rpm for cmd 60..450. Live, plausible, and
	//    the old rpm-only gate could not see it. Every one of those commands must read CMD_LOW. --
	{
		Bench b;
		bool all_invalid = true, all_cmdlow = true;
		for (float cmd = 60.0f; cmd <= 450.0f; cmd += 10.0f) {
			b.ticks(40, cmd, 182.0f);                       // 0.8 s — far past the dwell
			if (b.g.valid()) all_invalid = false;
			if (b.g.why() != SenseWhy::CMD_LOW) all_cmdlow = false;
		}
		CHECK(all_invalid, "forced-sine virtual 182 rpm (cmd 60..450, live) is INVALID");
		CHECK(all_cmdlow, "...and says why: CMD_LOW (the exact case the old tele-only gate missed)");
	}

	// -- the driftsoak operating point (1600 rpm => cmd ~= 652 at cross_up 36): valid, but only after
	//    the dwell, and invalid on the SAME tick the command falls out of the window. --
	{
		Bench b;
		const float CMD = 652.0f, RPM = 1600.0f;
		CHECK(b.mm.duty6(CMD) < SENSE_DUTY_CEILING && CMD > b.g.cmdFloor(),
		      "driftsoak point (cmd 652 @ 1600 rpm) is inside the window");
		b.tick(CMD, RPM);
		CHECK(!b.g.valid() && b.g.why() == SenseWhy::SETTLING, "first in-window tick: SETTLING, not valid");
		CHECK(b.g.freshEntry(), "first in-window tick reports freshEntry (re-seed the vest filter)");
		b.ticks(13, CMD, RPM);                              // 14 ticks = 0.28 s, still short of 0.30
		CHECK(!b.g.valid(), "still invalid at 0.28 s (dwell not elapsed)");
		b.ticks(3, CMD, RPM);                               // 0.34 s
		CHECK(b.g.valid() && b.g.why() == SenseWhy::OK, "valid after the 0.30 s dwell");
		CHECK(!b.g.freshEntry(), "freshEntry is one-shot");
		// ...and the dwell is in TIME, not ticks: at 200 Hz it takes ~4x as many ticks.
		Bench f;
		f.ticks(15, CMD, RPM, true, 0.005f);
		CHECK(!f.g.valid(), "dwell is in seconds, not ticks (15 ticks @ 200 Hz is not enough)");
		f.ticks(50, CMD, RPM, true, 0.005f);
		CHECK(f.g.valid(), "...and latches once 0.30 s of ticks have actually elapsed");
		// leaving the window is immediate — one tick below the EXIT threshold.
		float below = b.g.cmdExit() - 1.0f;
		b.tick(below, RPM);
		CHECK(!b.g.valid() && b.g.why() == SenseWhy::CMD_LOW,
		      "one tick below the exit threshold invalidates immediately");
	}

	// -- hysteresis: once latched, dithering between the exit and enter thresholds never transitions --
	{
		Bench b;
		b.ticks(40, 652.0f, 1600.0f);
		CHECK(b.g.valid(), "latched before the dither");
		float lo = b.g.cmdExit() + 0.5f, hi = b.g.cmdFloor() - 0.5f;   // 526.8 .. 541.3
		CHECK(lo < hi, "the hysteresis band is non-empty");
		int transitions = 0;
		for (int i = 0; i < 200; i++) {
			b.tick((i & 1) ? lo : hi, 1000.0f);
			if (!b.g.valid()) transitions++;
		}
		CHECK(transitions == 0, "no validity transitions while dithering inside the hysteresis band");
		// The same dither from a CLOSED gate never opens it — the enter margin is the bar to latch.
		Bench c;
		for (int i = 0; i < 200; i++) c.tick(lo, 1000.0f);
		CHECK(!c.g.valid() && c.g.why() == SenseWhy::CMD_LOW,
		      "a command inside the band cannot OPEN the gate (enter margin is the bar)");
	}

	// -- upper end: a near-saturation command flattens rpm6step against noLoadTop, so vest stops being
	//    sensitive to rpm. cmd 900 -> duty 1.09 on this bench. --
	{
		Bench b;
		CHECK(b.mm.duty6(900.0f) > SENSE_DUTY_CEILING, "cmd 900 really is above the duty ceiling");
		b.ticks(40, 900.0f, 3400.0f);
		CHECK(!b.g.valid() && b.g.why() == SenseWhy::DUTY_HIGH, "duty ceiling rejects a near-saturation command");
	}

	// -- secondary rpm floor: commanded well into 6-step, rotor never caught --
	{
		Bench b;
		b.ticks(40, 652.0f, 100.0f);
		CHECK(!b.g.valid() && b.g.why() == SenseWhy::RPM_LOW, "commanded high but the rotor never caught -> RPM_LOW");
	}

	// -- liveness is still necessary (it is just not sufficient) --
	{
		Bench b;
		b.ticks(40, 652.0f, 1600.0f, /*live=*/false);
		CHECK(!b.g.valid() && b.g.why() == SenseWhy::NOT_LIVE, "no live telemetry -> NOT_LIVE");
	}

	// -- an unconfigured seam CLOSES the gate (it must not open it): the floor is unreachable --
	{
		Bench b(CROSS_UP, /*configure=*/false);          // up_erpm left at 0
		b.ticks(40, 1000.0f, 3000.0f);
		CHECK(!b.g.valid() && b.g.why() == SenseWhy::CMD_LOW, "no seam configured -> the gate closes");
		CHECK(b.g.cmdFloor() > 1000.0f, "...and reports a floor no command can reach");
	}

	// -- vbatt tier: the load signal must SURVIVE load, so a droop past the ceiling kills vbattOk only --
	{
		Bench b;
		b.vref = 11.0f; b.zeroCmd = 652.0f;
		b.loadV = 0.5f;                                     // 4.5 % droop: still light load
		b.ticks(40, 652.0f, 1600.0f);
		CHECK(b.g.valid() && b.g.vbattOk(), "light load (0.5 V of 11.0) keeps the vbatt tier open");
		b.loadV = 1.8f;                                     // 16 % droop: loaded
		b.tick(652.0f, 1600.0f);
		CHECK(b.g.valid() && b.g.loadOk(), "under load, valid and loadOk STAY true (load survives load)");
		CHECK(!b.g.vbattOk(), "...but vbattOk goes false (vcal only cancels the deficit at light load)");
		// With no reference captured yet, the rolling max is itself the light-load selector.
		Bench n;
		n.ticks(40, 652.0f, 1600.0f);
		CHECK(n.g.valid() && n.g.vbattOk(), "no vref yet -> the rolling max is the light-load selector");
	}

	// -- load tier: `load` is a difference against the zero point, so it only means anything near the
	//    DUTY it was captured at (the band is in duty, not command units — see SENSE_OFFPOINT_DUTY) --
	{
		Bench b;
		b.vref = 11.0f; b.zeroCmd = 652.0f;                 // duty6(652) = 0.411
		b.ticks(40, 652.0f, 1600.0f);
		CHECK(b.g.loadOk(), "at the zeroing command, loadOk");
		b.ticks(40, 675.0f, 1830.0f);                       // dduty 0.061: inside the 0.10 band
		CHECK(b.g.valid() && b.g.loadOk(), "+0.06 duty off the zero point is still comparable");
		b.ticks(40, 700.0f, 2090.0f);                       // dduty 0.127: outside
		CHECK(b.g.valid() && !b.g.loadOk(), "+0.13 duty off the zero point -> loadOk false, valid unaffected");
		// ...and the band is symmetric, i.e. dropping below the zero point is just as off-point.
		b.ticks(40, 600.0f, 1400.0f);                       // dduty -0.138
		CHECK(b.g.valid() && !b.g.loadOk(), "...and symmetric: below the zero point is off-point too");
		// never zeroed => there is no load number at all
		Bench z;
		z.ticks(40, 652.0f, 1600.0f);
		CHECK(z.g.valid() && !z.g.loadOk(), "never zeroed -> loadOk false (there is no load reading yet)");
	}

	// -- THE REASON the band is in duty and this wide: `load` must SURVIVE load. Holding rpm through a
	//    droop costs duty * 1/(1-droop), so the 10 % droop that closes the vbatt tier moves duty by
	//    0.046 — which must stay INSIDE the off-point band or loadOk would close exactly when a load
	//    appears, killing the sensor's primary product. In command units that is only ~17 counts. --
	{
		Bench b;
		b.vref = 11.0f; b.zeroCmd = 652.0f;
		b.ticks(40, 652.0f, 1600.0f);
		CHECK(b.g.valid() && b.g.loadOk() && b.g.vbattOk(), "light load at the zero point: all three open");
		// 10 % droop: the PI holds 1600 rpm by pushing the command from 652 to 652/0.9 in DUTY terms.
		float loaded = b.mm.cmdForDuty6(b.mm.duty6(652.0f) / 0.9f);
		b.loadV = 1.2f;                                     // 10.9 % of vref: past the light-load ceiling
		b.ticks(40, loaded, 1600.0f);
		printf("   load survival: zero cmd 652 (duty %.3f) -> loaded cmd %.0f (duty %.3f), dduty %.3f\n",
		       b.mm.duty6(652.0f), loaded, b.mm.duty6(loaded), b.mm.duty6(loaded) - b.mm.duty6(652.0f));
		CHECK(b.mm.duty6(loaded) - b.mm.duty6(652.0f) < SENSE_OFFPOINT_DUTY,
		      "a 10 % droop moves duty less than the off-point band (by construction)");
		CHECK(b.g.valid() && b.g.loadOk(), "LOAD SURVIVES LOAD: valid and loadOk stay open under it");
		CHECK(!b.g.vbattOk(), "...while the vbatt tier closes, which is the tier that must be light-load");
	}

	// -- reset() (arm()/stop()) drops every latch: dwell, hysteresis and the tiers --
	{
		Bench b;
		b.vref = 11.0f; b.zeroCmd = 652.0f;
		b.ticks(40, 652.0f, 1600.0f);
		CHECK(b.g.valid() && b.g.loadOk() && b.g.vbattOk(), "latched before reset");
		b.g.reset();
		CHECK(!b.g.valid() && !b.g.loadOk() && !b.g.vbattOk() && b.g.dwell() == 0.0f &&
		      b.g.why() == SenseWhy::NOT_LIVE, "reset() clears validity, dwell and both tiers");
		b.tick(652.0f, 1600.0f);
		CHECK(!b.g.valid() && b.g.freshEntry(), "after reset the dwell restarts from scratch");
	}

	printf(failures ? "\n%d CHECK(S) FAILED\n" : "\nALL CHECKS PASSED\n", failures);
	return failures ? 1 : 0;
}
