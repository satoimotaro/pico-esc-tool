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
	{
		const float cmds[] = {780, 690, 630, 570};
		const float meas[] = {0.7455f, 0.5101f, 0.3515f, 0.1922f};
		for (int i = 0; i < 4; i++) {
			CHECK(fabsf(c36.duty6(cmds[i]) / meas[i] - 1.0f) < 0.01f, "cross_up=36 duty within 1% of bench");
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
	// crossUpErpm() existed before this but handoffMech()/handoffCmd()/sixstepFloorRpm() went straight
	// to HANDOFF_ERPM, so the seam LOCATION stayed pinned to the firmware minimum at every cross_up.
	// Table pinned against host/pico_esc/motor_model.py:MotorModel.cross_up_erpm() and
	// lib/vel_control/test/test_sense_gate.cpp (same three bytes, 0.5 eRPM parity band). 34 floors at
	// the firmware minimum because 34*39.0625 = 1328.1 < 1333.
	{
		MotorModel s34(350, 7, 11.2f), s36(350, 7, 11.2f), s50(350, 7, 11.2f);
		s36.setCrossUp(36); s50.setCrossUp(50);
		NEAR(s34.crossUpErpm(), HANDOFF_ERPM, 0.5f, "cross_up 34 -> the firmware floor");
		NEAR(s36.crossUpErpm(), 1406.25f, 0.01f, "cross_up 36 -> 1406.25 eRPM");
		NEAR(s50.crossUpErpm(), 1953.125f, 0.01f, "cross_up 50 -> 1953.125 eRPM");
		// handoffMech = seam/PP, and every seam-derived quantity moves with it
		NEAR(s36.handoffMech(), 1406.25f / 7.0f, 0.01f, "handoffMech follows the configured seam");
		NEAR(s50.handoffCmd(), 1953.125f / SINE_FULLSCALE_ERPM * 1000.0f, 0.05f,
		     "handoffCmd follows the configured seam (cross_up 50 -> cmd 781, not 533)");
		CHECK(s50.handoffCmd() > s36.handoffCmd() && s36.handoffCmd() > s34.handoffCmd(),
		      "raising cross_up raises the handoff command");
		// The LANDING falls as cross_up rises even though the handoff command rises: the extra command
		// buys less duty (DUTY6_CROSSUP_K), and that costs more than the higher seam gains. So the gap
		// narrows from BOTH ends — 34: 190..707 rpm, 36: 201..685, 50: 279..640.
		CHECK(s50.sixstepFloorRpm() < s36.sixstepFloorRpm()
		      && s36.sixstepFloorRpm() < s34.sixstepFloorRpm(),
		      "raising cross_up LOWERS the 6-step landing (less duty per command)");
		CHECK(s34.inGap(250.0f) && !s50.inGap(250.0f), "the gap's lower edge rises with cross_up");
		CHECK(s34.inGap(670.0f) && !s50.inGap(670.0f), "the gap's upper edge falls with cross_up");
	}

	printf(fails ? "\n%d CHECK(s) FAILED\n" : "\nall motor_model checks passed\n", fails);
	return fails ? 1 : 0;
}
