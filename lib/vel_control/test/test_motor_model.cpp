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
	CHECK(a.crossUpByte() == 34 && a.crossDnByte() == 255, "esc config bytes");

	// soft-sensor: estimateV inverts rpm6step back to the supply voltage (a=350/7/11.1, eff=1)
	float cmdE = 600.0f, rpmE = a.rpm6step(cmdE);
	NEAR(a.estimateV(cmdE, rpmE), 11.1f, 0.1f, "estimateV recovers supply V");
	NEAR(a.estimateLoad(cmdE, rpmE), 0.0f, 0.1f, "estimateLoad ~0 at model rpm");
	CHECK(a.estimateLoad(cmdE, rpmE * 0.9f) > 0.5f, "estimateLoad > 0 under droop");   // slower than model => load
	CHECK(a.estimateV(0.0f, 0.0f) == 0.0f, "estimateV 0 outside live 6-step");         // no BEMF -> invalid

	printf(fails ? "\n%d CHECK(s) FAILED\n" : "\nall motor_model checks passed\n", fails);
	return fails ? 1 : 0;
}
