// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// Native (host g++) test for the C++ MotorModel — must match host/pico_esc/motor_model.py.
//   g++ -std=c++17 -O2 -Wall -o /tmp/tmm test_motor_model.cpp && /tmp/tmm
#include "../motor_model.h"
#include <cstdio>
#include <cmath>
#include <initializer_list>

using namespace vel;

static int fails = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); fails++; } } while (0)
#define NEAR(a, b, tol, msg) CHECK(fabsf((float)(a) - (float)(b)) <= (tol), msg)

int main() {
	// no-load top scales with KV*V
	NEAR(MotorModel(350, 7, 11.1f).noLoadTop(), 350 * 11.1f, 0.1f, "top 350");
	NEAR(MotorModel(1000, 7, 11.1f).noLoadTop(), 1000 * 11.1f, 0.1f, "top 1000");

	// 6-step curve scales with KV at any cmd
	MotorModel a(350, 7, 11.1f), b(1000, 7, 11.1f);
	NEAR(b.rpm6step(600) / a.rpm6step(600), 1000.0f / 350.0f, 1e-3f, "6step KV scaling");

	// sine is PP-governed, not KV
	NEAR(MotorModel(350, 7, 11.1f).sineFullscaleMech(),
	     MotorModel(999, 7, 5.0f).sineFullscaleMech(), 0.1f, "sine KV-independent");
	CHECK(MotorModel(350, 7, 11.1f).sineFullscaleMech() > MotorModel(350, 14, 11.1f).sineFullscaleMech(),
	      "sine PP-governed");

	// regime boundaries + gap
	CHECK(!a.inGap(100) && a.inGap((a.handoffMech() + a.sixstepFloorRpm()) / 2)
	      && !a.inGap(a.sixstepFloorRpm() + 50), "regime/gap");

	// thrustFor round-trips in 6-step
	for (float rpm : {800.0f, 1500.0f, 2400.0f}) {
		float cmd = a.thrustFor(rpm);
		NEAR(a.rpm6step(cmd), rpm, 1.0f, "thrustFor roundtrip");
	}

	// predicts the MEASURED 350KV @ 11.1V steady 6-step curve to < ~15 rpm from KV alone
	const float mc[] = {520, 545, 570, 595, 620, 645, 670, 700};
	const float mr[] = {551, 808, 1067, 1329, 1584, 1841, 2092, 2400};
	float sse = 0.0f;
	for (int i = 0; i < 8; i++) { float d = a.rpm6step(mc[i]) - mr[i]; sse += d * d; }
	float rmse = sqrtf(sse / 8.0f);
	printf("350KV curve RMSE (eff=1, no cal) = %.1f rpm\n", rmse);
	CHECK(rmse < 15.0f, "350KV curve predicted from KV");

	// eRPM self-cal fits eff (plant 5% slower)
	float cc[] = {550, 620, 690}, ee[3];
	for (int i = 0; i < 3; i++) ee[i] = 0.95f * (350 * 11.1f * (DUTY6_A * cc[i] + DUTY6_B)) * 7.0f;
	float eff = a.selfCalErpm(cc, ee, 3);
	NEAR(eff, 0.95f, 0.01f, "self-cal eff");
	a.setEff(1.0f);   // restore

	// fillProfile is monotone + loadable by SpeedProfile
	CurvePoint buf[18];
	int k = a.fillProfile(buf, 18);
	CHECK(k >= 6, "fillProfile count");
	bool mono = true;
	for (int i = 1; i < k; i++) if (!(buf[i].thrust > buf[i - 1].thrust && buf[i].rpm > buf[i - 1].rpm)) mono = false;
	CHECK(mono, "fillProfile monotone");
	SpeedProfile prof(buf, k, 7);
	NEAR(prof.thrustFor(1500), a.thrustFor(1500), 5.0f, "profile matches model");

	// ESC config recipe is the KV-independent working one
	CHECK(a.recommendedCrossUpByte() == 34 && a.crossDnByte() == 255, "esc config bytes");

	// soft-sensor: estimateV inverts rpm6step back to the supply voltage (a=350/7/11.1, eff=1)
	float cmdE = 600.0f, rpmE = a.rpm6step(cmdE);
	NEAR(a.estimateV(cmdE, rpmE), 11.1f, 0.1f, "estimateV recovers supply V");
	NEAR(a.estimateLoad(cmdE, rpmE), 0.0f, 0.1f, "estimateLoad ~0 at model rpm");
	CHECK(a.estimateLoad(cmdE, rpmE * 0.9f) > 0.5f, "estimateLoad > 0 under droop");   // slower than model => load
	CHECK(a.estimateV(0.0f, 0.0f) == 0.0f, "estimateV 0 outside live 6-step");         // no BEMF -> invalid

	// ---- sine_cross_up term (bench 2026-07-29, host/duty_sweep.py, F2838 350KV @ 11.2 V) ----
	// Default == DUTY6_CROSSUP_REF, so an operator who never calls setCrossUp keeps the old behaviour.
	MotorModel c34(350, 7, 11.2f);
	CHECK(c34.crossUp() == DUTY6_CROSSUP_REF, "cross_up defaults to the fit reference");
	NEAR(c34.duty6(650), DUTY6_A * 650 + DUTY6_B, 1e-6f, "at the reference the term vanishes");

	// Raising Cross_Up LOWERS the duty by K per unit (cross_rescale_duty shifts the affine's intercept).
	MotorModel c36(350, 7, 11.2f); c36.setCrossUp(36);
	NEAR(c34.duty6(650) - c36.duty6(650), 2 * DUTY6_CROSSUP_K, 1e-6f, "cross_up shifts duty by K/unit");
	NEAR(c34.duty6(780) - c36.duty6(780), c34.duty6(570) - c36.duty6(570), 1e-6f,
	     "the shift is a pure OFFSET — cmd-independent, no slope change");

	// The point of the whole exercise: the measured no-load duty at Cross_Up=36.
	// NOTE these are the RAW rpm/(KV*V_supply) values, NOT the PSU-current-corrected ones. duty6() is
	// the model's EFFECTIVE duty and V_supply is its reference, so it is definitionally the raw
	// quantity; the current-corrected duty (0.7550/0.5205/0.3627/0.2057 for these cmds) is the true
	// ELECTRICAL duty and sits ~1-6 % higher. Comparing against those would be an apples-to-oranges
	// test — and the gap between them IS the no-load I*R the model silently absorbs.
	//
	// DOCUMENTED MISS, and it is a CONSEQUENCE OF THE 2026-07-31 DUTY6_CROSSUP_K RE-FIT, not a
	// regression. These four rows are the cross_up-36 half of the 2026-07-29 pair that the OLD
	// K = 0.0421 was fitted TO, so they were never an independent check of K — they agreed with it by
	// construction (+0.58/+0.24/+0.23/+0.57 %). Re-fitting K to 0.04134 on a 14-unit baseline lowers
	// the cross_up-36 subtraction by a CONSTANT 2*(0.0421-0.04134) = 0.00152 duty, so the model reads
	// that much higher at every command: +0.20 % of the duty at cmd 780 but +0.79 % at cmd 570, where
	// the duty is only 0.1922. The ratio at cmd 570 therefore crosses 1 % (+1.36 %) while the other
	// three stay inside it (+0.78/+0.53/+0.66 %).
	// It is pinned AT the value achieved rather than by widening the band to 1.5 %, because the two
	// facts worth holding are different: that the miss is bounded, and that it is an OFFSET and not a
	// slope. In DUTY the four errors are +0.0058/+0.0027/+0.0023/+0.0026 — flat across a 210-command
	// span, i.e. the residual lives in DUTY6_B / the no-load I*R this map absorbs, which is exactly
	// what the re-fit deliberately did NOT touch (only the ratio K/DUTY6_A was re-measured; that run
	// carries no PSU currents, so its absolute scale is not trustworthy).
	{
		const float cmds[] = {780, 690, 630, 570};
		const float meas[] = {0.7455f, 0.5101f, 0.3515f, 0.1922f};
		for (int i = 0; i < 4; i++) {
			float rel = c36.duty6(cmds[i]) / meas[i] - 1.0f;
			if (cmds[i] != 570)
				CHECK(fabsf(rel) < 0.01f, "cross_up=36 duty within 1% of bench (cmd 780/690/630)");
			else
				CHECK(rel > 0.010f && rel < 0.014f,
				      "cmd 570 misses by +1.36% — the K re-fit's constant 0.00152 duty over a 0.19 duty");
			// the model is high by a near-constant DUTY at every command: an offset, not a slope error
			CHECK(c36.duty6(cmds[i]) - meas[i] > 0.0f && c36.duty6(cmds[i]) - meas[i] < 0.006f,
			      "...and in DUTY the error is a bounded offset at every command");
			// ...and that the UNcorrected model would have missed by a lot at the low end
			if (cmds[i] == 570) CHECK(fabsf(c34.duty6(570) / meas[3] - 1.0f) > 0.30f,
			                          "uncorrected model is >30% off at cmd 570 (the bug being fixed)");
		}
	}
	// The inverse must undo the forward map WITH the term applied, or thrustFor() would fight rpm6step().
	NEAR(c36.cmdForDuty6(c36.duty6(712)), 712.0f, 1e-2f, "cmdForDuty6 inverts duty6 at cross_up=36");
	NEAR(c36.cmd6stepFor(c36.rpm6step(712)), 712.0f, 1e-2f, "cmd6stepFor inverts rpm6step at cross_up=36");
	// Same rpm needs a HIGHER command once Cross_Up rises (less duty per command).
	CHECK(c36.cmd6stepFor(1600) > c34.cmd6stepFor(1600), "higher cross_up costs more command for 1600 rpm");

	// ---- the SEAM must track cross_up too, not just the duty (E.9) ----
	// crossUpErpm() is the CONFIGURED seam and still feeds the ESC config + Thruster's ffcx_. It is no
	// longer where the handoff comes from (see the block after this one). Table pinned against
	// host/pico_esc/motor_model.py:MotorModel.cross_up_erpm() and test_sense_gate.cpp (same three
	// bytes, 0.5 eRPM parity band). 34 floors at the firmware minimum because 34*39.0625 = 1328.1 < 1333.
	{
		MotorModel s34(350, 7, 11.2f), s36(350, 7, 11.2f), s50(350, 7, 11.2f);
		s36.setCrossUp(36); s50.setCrossUp(50);
		NEAR(s34.crossUpErpm(), HANDOFF_ERPM, 0.5f, "cross_up 34 -> the firmware floor");
		NEAR(s36.crossUpErpm(), 1406.25f, 0.01f, "cross_up 36 -> 1406.25 eRPM");
		NEAR(s50.crossUpErpm(), 1953.125f, 0.01f, "cross_up 50 -> 1953.125 eRPM");
	}

	// ---- the UP-HANDOFF IS A DUTY FLOOR, not the seam (measured 2026-07-31, 930KV @ 11.2 V) ----
	// The old law inverted the CONFIGURED SEAM through the forced-sine curve and gave 533/563/781 at
	// cross_up 34/36/50 on every motor. The 1-count sweeps say 485.0 +- 1.0 / 515.5 +- 0.5 /
	// 734.5 +- 0.5 (host/reports/sinesweep_20260731_014004 / _013410 / _013236.csv), i.e. the old law
	// was ~47 counts high everywhere. handoffCmd() is now DUTY6_HANDOFF_FLOOR inverted through the
	// 6-step duty map. Mirrors host/tests/test_motor_model.py::test_the_handoff_command_is_the_duty_floor.
	{
		MotorModel s34(350, 7, 11.2f), s36(350, 7, 11.2f), s50(350, 7, 11.2f);
		s36.setCrossUp(36); s50.setCrossUp(50);
		NEAR(s34.handoffCmd(), 484.21f, 0.1f, "handoffCmd cross_up 34 = 484.21 (inside the measured (484, 486])");
		NEAR(s36.handoffCmd(), 515.41f, 0.1f, "handoffCmd cross_up 36 = 515.41 (inside the measured (515, 516])");
		// -0.69 counts against the measurement, i.e. 0.19 below the bracket's lower edge. This pin used
		// to be 738.40 / +3.90, which was DUTY6_CROSSUP_K running 1.8 % high: the residuals fanned out
		// with cross_up (+0.29 counts/unit), the signature of a wrong SLOPE. K was re-fitted on
		// 2026-07-31 (0.0421 -> 0.04134, three independent routes agreeing on 15.60/15.63/15.59 counts
		// per unit against the model's 15.89) and the trend collapsed to +0.006 counts/unit.
		NEAR(s50.handoffCmd(), 733.81f, 0.1f,
		     "handoffCmd cross_up 50 = 733.81, not 781 (measured 734.5 +- 0.5, i.e. -0.69)");
		// THE RESIDUAL IS NOW A CONSTANT, NOT A TREND — the whole point of the re-fit, pinned as a shape
		// and not just as three numbers. Mirrors host/tests/test_motor_model.py's residual test.
		{
			float r34 = s34.handoffCmd() - 485.0f, r36 = s36.handoffCmd() - 515.5f,
			      r50 = s50.handoffCmd() - 734.5f;
			CHECK(r34 < 0.0f && r36 < 0.0f && r50 < 0.0f, "all three residuals are on the same side");
			NEAR(fabsf(r50 - r34) / 16.0f, 0.006f, 0.002f, "the cross_up TREND is gone (was 0.293/unit)");
			float hi = r34 > r36 ? (r34 > r50 ? r34 : r50) : (r36 > r50 ? r36 : r50);
			float lo = r34 < r36 ? (r34 < r50 ? r34 : r50) : (r36 < r50 ? r36 : r50);
			CHECK(hi - lo <= 0.75f, "...leaving a spread under one command count (was 4.69)");
		}
		// KV- AND PP-independent, because a duty carries no motor terms. Same three commands on a
		// 930KV 12-pole at 22.2 V.
		MotorModel big(930, 12, 22.2f); big.setCrossUp(36);
		NEAR(big.handoffCmd(), s36.handoffCmd(), 0.01f,
		     "the handoff COMMAND is KV/pp/V independent (930KV pp=12 @ 22.2 V == 350KV pp=7 @ 11.2 V)");
		// ...while the LANDING is a 6-step rpm and scales with KV*V. It is KV*V*eff*FLOOR exactly.
		NEAR(s36.sixstepFloorRpm(), 350.0f * 11.2f * DUTY6_HANDOFF_FLOOR, 0.05f,
		     "sixstepFloorRpm == KV*V*eff*DUTY6_HANDOFF_FLOOR (196.7 rpm on 350KV @ 11.2 V)");
		NEAR(MotorModel(930, 7, 11.2f).sixstepFloorRpm(), 522.53f, 0.1f,
		     "...and 522.5 rpm on the 930KV @ 11.2 V it was measured on");
		// THE INVERTED ASSERTION. This block used to require the landing to FALL as cross_up rises
		// ("the extra command buys less duty than the seam gains"). That was an artifact of deriving
		// the trigger from the seam: the trigger is a fixed DUTY, so the landing is the SAME speed at
		// every cross_up — measured 522.19 / 522.56 / 523.44 rpm at cross_up 50 / 36 / 34, a 1.2 rpm
		// spread over a 16-unit span and 250 commands. Equality, not an inequality.
		NEAR(s50.sixstepFloorRpm(), s34.sixstepFloorRpm(), 1e-3f,
		     "the 6-step landing is INDEPENDENT of cross_up (measured 523.4/522.6/522.2 at 34/36/50)");
		NEAR(s36.sixstepFloorRpm(), s34.sixstepFloorRpm(), 1e-3f, "...at cross_up 36 too");
		CHECK(s50.handoffCmd() > s36.handoffCmd() && s36.handoffCmd() > s34.handoffCmd(),
		      "raising cross_up still raises the handoff COMMAND (the duty costs more command)");
		// handoffMech is the rotor on the forced field at that command, so it DOES move with cross_up.
		NEAR(s34.handoffMech(), 172.86f, 0.05f, "handoffMech cross_up 34 = 172.9 mech (pp=7)");
		NEAR(s36.handoffMech(), 184.00f, 0.05f, "handoffMech cross_up 36 = 184.0 mech (pp=7)");
		NEAR(s50.handoffMech(), 261.97f, 0.05f, "handoffMech cross_up 50 = 262.0 mech (pp=7)");
		// So the gap 172.9..196.7 rpm at cross_up 34 is NARROW, and at cross_up 50 it is EMPTY/INVERTED
		// (the trigger fires at 262.0 while the landing is still 196.7). Probes re-picked for those
		// edges — 250 rpm is now 6-step at cross_up 34, and forced sine at 50.
		CHECK(s34.inGap(185.0f) && !s50.inGap(185.0f), "185 rpm is in the gap at 34, forced sine at 50");
		CHECK(!s34.inGap(250.0f) && !s50.inGap(250.0f), "250 rpm is 6-step at 34 and forced sine at 50");
		CHECK(!s50.inGap(220.0f), "cross_up 50's gap is EMPTY: the trigger fires above the landing");
	}

	// ---- the firmware REFUSES to hand off outside its seed-tick window ----
	// Measured 2026-07-31 (host/reports/sinesweep_20260731_013633.csv): at cross_up 30 -> 1171.9 eRPM
	// -> a 34-tick window against SINE_CROSS_TICKS_MAX = 30, the 930KV never handed off on either leg
	// up to cmd 500. crossUpErpm() would have floored that to 1333 and reported a confident handoff.
	{
		MotorModel r(350, 7, 11.2f), ok(350, 7, 11.2f);
		r.setCrossUp(30); ok.setCrossUp(34);
		CHECK(!r.handoffPossible(), "cross_up 30 -> 34-tick window -> the firmware refuses");
		MotorModel r33(350, 7, 11.2f); r33.setCrossUp(33);
		CHECK(!r33.handoffPossible(), "cross_up 33 -> 31 ticks -> still refused (byte granularity)");
		CHECK(ok.handoffPossible(), "cross_up 34 -> 30 ticks -> accepted (the boundary byte)");
		// ...and a refused config cannot produce a usable command or speed. HANDOFF_CMD_NONE is above
		// the reachable +-1000 range and the rpms are INFINITY, so every downstream comparison fails
		// CLOSED without a special case.
		NEAR(r.handoffCmd(), HANDOFF_CMD_NONE, 1e-3f, "refused -> handoffCmd is the NONE sentinel");
		CHECK(isinf(r.handoffMech()) && isinf(r.sixstepFloorRpm()), "refused -> both rpms are INFINITY");
		CHECK(!r.inGap(200.0f) && !r.inGap(5000.0f), "refused -> nothing is in the gap");
		NEAR(r.thrustFor(200.0f), r.cmdForSine(200.0f), 1e-3f, "refused -> thrustFor stays on sine");
		CurvePoint buf[16];
		CHECK(r.fillProfile(buf, 16) == 0, "refused -> fillProfile emits no curve (thruster keeps its own)");
		CHECK(ok.fillProfile(buf, 16) > 2, "...while an accepted cross_up still fills one");
	}

	// ---- the OTHER refusal: the handoff command is past the reachable +-CMD_FULL_SCALE ----
	// cross_up shifts the trigger by DUTY6_CROSSUP_K/DUTY6_A = 15.6 counts per unit, so the handoff
	// command walks off the end of the protocol's range: 999.0 at cross_up 67, 1014.6 at 68. Two things
	// ride on this boundary. (1) Nothing can be SENT that hands off above it, so it is a refusal in the
	// same operational sense as a rejected tick window. (2) It is what keeps HANDOFF_CMD_NONE (1001)
	// from aliasing a REAL handoff command — without it, `>= HANDOFF_CMD_NONE` would have been a
	// silently wrong refusal test from cross_up 68 up, and the sense-gate wire contract with it.
	// The boundary sat at 66/67 until DUTY6_CROSSUP_K was re-fitted on 2026-07-31 — it is a function
	// of K, which is exactly why it is pinned on both sides here rather than assumed.
	{
		MotorModel lo(350, 7, 11.2f), hi(350, 7, 11.2f);
		lo.setCrossUp(67); hi.setCrossUp(68);
		NEAR(lo.handoffCmd(), 999.01f, 0.1f, "cross_up 67 -> handoff 999.0, still reachable");
		CHECK(lo.handoffPossible(), "cross_up 67 is VALID (the last byte with a reachable handoff)");
		CHECK(!hi.handoffPossible(), "cross_up 68 -> 1014.6 > CMD_FULL_SCALE -> refused");
		NEAR(hi.handoffCmd(), HANDOFF_CMD_NONE, 1e-3f, "...and reports the sentinel, not 1014.6");
		MotorModel huge(350, 7, 11.2f); huge.setCrossUp(100);
		CHECK(!huge.handoffPossible(), "cross_up 100 (1514 counts) is refused too, not reported");
		CHECK(lo.handoffCmd() < CMD_FULL_SCALE && CMD_FULL_SCALE < HANDOFF_CMD_NONE,
		      "the sentinel invariant: every reachable handoffCmd < CMD_FULL_SCALE < HANDOFF_CMD_NONE");
	}

	// ---- a nonsense cross_up REFUSES; it does not raise, saturate or wrap ----
	// The tick window is a bounds test on the seam eRPM rather than `(int)(40000/up)` for exactly this:
	// the cast is UB past INT_MAX (setCrossUp(1e-9f) would give 1.02e12) and this predicate feeds the
	// soft-sensor gate, which must never fail open. Every comparison against a NaN is false, so a NaN
	// refuses. Mirrors host/tests/test_motor_model.py::test_a_nonsense_cross_up_refuses_rather_than_raising,
	// which pins the same six inputs on the Python side (there `int(inf)`/`int(nan)` would have thrown).
	{
		const float bad[] = {NAN, INFINITY, -INFINITY, 0.0f, -36.0f, 1e-30f, 1e30f};
		bool allRefused = true, allSane = true;
		for (float b : bad) {
			if (handoffPossibleAt(b)) allRefused = false;
			// setCrossUp() itself guards cu > 0, so the non-positive ones never reach crossUp_ — check
			// the ones that DO land (a denormal and a huge byte) end up refused and inert.
			MotorModel m(350, 7, 11.2f);
			m.setCrossUp(b);
			if (b > 0.0f && !handoffPossibleAt(b)) {
				if (m.handoffPossible() || m.inGap(500.0f) || m.handoffCmd() != HANDOFF_CMD_NONE)
					allSane = false;
			}
		}
		CHECK(allRefused, "NaN / +-inf / 0 / negative / denormal / huge cross_up all REFUSE");
		CHECK(allSane, "...and the ones setCrossUp accepts leave inGap false and the NONE sentinel set");
	}

	// ---- fillProfile must not emit a BACKWARDS 6-step ramp when the handoff is above cmdHi ----
	// At the default cmdHi = 700 that is any cross_up >= 48 — including cross_up 50, the setting the
	// 2026-07-31 bench itself ran. Unguarded it returned 4 points whose 6-step half descends, which
	// sails past Thruster::setMotor's `n < 2` guard and installs a curve that inverts to one command
	// for every rpm. Emitting nothing makes the thruster keep its previous curve instead.
	{
		MotorModel m50(350, 7, 11.2f); m50.setCrossUp(50);
		CurvePoint buf[16];
		CHECK(m50.handoffPossible(), "cross_up 50 CAN hand off (733.8) — this is not the refusal case");
		CHECK(m50.handoffCmd() > 700.0f, "...but its handoff is above the default cmdHi of 700");
		CHECK(m50.fillProfile(buf, 16) == 0, "handoff above cmdHi -> no curve emitted");
		int n = m50.fillProfile(buf, 16, 900.0f);
		CHECK(n > 2, "...and raising cmdHi past the handoff makes it fill again");
		bool rising = true;
		for (int i = 1; i < n; i++) if (buf[i].thrust <= buf[i-1].thrust || buf[i].rpm <= buf[i-1].rpm)
			rising = false;
		CHECK(rising, "...with a strictly increasing curve (no backwards 6-step ramp)");
	}

	// The EMITTABLE band vs the HANDOFF band. Two different upper bounds, and conflating them shipped
	// a wrong recovery message twice: 67 is where the handoff command stops being a command at all,
	// 47 is where the handoff outruns the cmdHi the curve is SAMPLED to. Bytes in 48..67 hand off
	// perfectly well and still cannot be given a parametric FF curve. Both derive from
	// DUTY6_CROSSUP_K, so neither may be a literal — and the 2026-07-31 re-fit proved it by moving
	// ONE of them (66 -> 67) and leaving the other at 47. Both must agree with
	// host/pico_esc/motor_model.py's _min_handoff_cross_up/_max_emittable_cross_up.
	{
		CHECK(vel::minHandoffCrossUp() == 34, "min handoff byte derives to 34 (python parity)");
		CHECK(vel::maxEmittableCrossUp(700.0f) == 47, "max emittable at cmdHi 700 is 47 (python parity)");
		CHECK(vel::maxEmittableCrossUp(900.0f) == 60, "max emittable at cmdHi 900 is 60 (python parity)");
		CHECK(vel::maxEmittableCrossUp(700.0f) < 67, "the emittable band is NARROWER than the handoff band");
		CurvePoint buf2[24];
		for (int cu = 34; cu <= 67; cu++) {
			vel::MotorModel m(350.0f, 7, 11.1f); m.setCrossUp((float)cu);
			bool emitted = m.fillProfile(buf2, 24) >= 2;
			CHECK(emitted == (cu <= vel::maxEmittableCrossUp(700.0f)),
			      "fillProfile succeeds exactly on the derived emittable band");
			CHECK(m.handoffPossible(), "...while every byte in 34..67 can still hand off");
		}
	}

	printf(fails ? "\n%d CHECK(s) FAILED\n" : "\nall motor_model checks passed\n", fails);
	return fails ? 1 : 0;
}
