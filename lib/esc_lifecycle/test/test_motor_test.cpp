// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// Native (host g++) test for the motor-test mapping (motor_test.h): every pairing of a channel's
// meaning (3D or one-way, declared or not) with the ESC's (3D or one-way), over the values that
// matter, against the rules the firmware relies on: stop is stop, reverse only when both are 3D,
// nothing out of range, and more value never means less speed.
//     g++ -std=c++17 -O2 -Wall -Wextra -Werror -o /tmp/tmt test_motor_test.cpp && /tmp/tmt
#include "../motor_test.h"
#include <cstdio>
#include <cstdlib>
#include <initializer_list>

using namespace mt;

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); failures++; } \
	else printf("ok: %s\n", msg); } while (0)

static const uint16_t VALUES[] = { 0, 1, 999, 1000, 1001, 1250, 1499, 1500, 1501, 1750, 1999, 2000, 2001, 3000 };

int main() {
	bool stopOk = true, revOk = true, rangeOk = true, monoOk = true, pwmStop = true, pwmRev = true, pwmRange = true;
	bool wantStop = true, wantStock = true;
	for (int c3d = 0; c3d < 2; c3d++)
	for (int rev = 0; rev < 2; rev++) {
		uint16_t stop = stopOf(c3d);
		// stop (and 0, a short frame) -> drive 0 and the ESC's own stop pulse
		for (uint16_t v : { (uint16_t)0, stop }) {
			stopOk = stopOk && driveValue(v, c3d, rev) == 0;
			pwmStop = pwmStop && pwmValue(v, c3d, rev) == (rev ? 1500 : 1000);
		}
		int last = -100000; uint16_t lastUs = 0;
		for (uint16_t v : VALUES) {
			int d = driveValue(v, c3d, rev);
			uint16_t us = pwmValue(v, c3d, rev);
			// reverse (negative / below 1500 us on a 3D ESC) only when value and ESC are both 3D
			if (d < 0 && !(c3d && rev)) revOk = false;
			if (rev && us < 1500 && !c3d) pwmRev = false;
			if (rev && us < 1500 && c3d && v >= 1500) pwmRev = false;
			if (c3d && rev && v > 1500 && d <= 0) revOk = false;     // forward stays forward
			// in range: signed thrust +-1000 on a 3D ESC, throttle 0..2000 on a one-way one
			if (rev ? (d < -1000 || d > 1000) : (d < 0 || d > 2000)) rangeOk = false;
			if (us < 1000 || us > 2000) pwmRange = false;
			if (!rev && us < 1000) pwmRange = false;
			// monotonic over v (0 = stop is not part of the order)
			if (v) { if (d < last || us < lastUs) monoOk = false; last = d; lastUs = us; }
		}
		// one-way ESC never sees reverse
		for (uint16_t v : VALUES) if (!rev && driveValue(v, c3d, rev) < 0) revOk = false;
	}
	// wanted: stop and 0 never take an ESC over; a stock client (not declared) never at or below 1500
	for (int c3d = 0; c3d < 2; c3d++)
	for (int decl = 0; decl < 2; decl++) {
		wantStop = wantStop && !wanted(0, c3d, decl) && !wanted(stopOf(c3d), c3d, decl);
		if (c3d && !decl) for (uint16_t v : VALUES) if (v <= 1500 && wanted(v, true, false)) wantStock = false;
	}
	CHECK(stopOk,   "drive: stop and 0 are 0 for every pairing");
	CHECK(revOk,    "drive: reverse only when the value and the ESC are both 3D");
	CHECK(rangeOk,  "drive: +-1000 on a 3D ESC, 0..2000 on a one-way one, whatever comes in");
	CHECK(monoOk,   "drive and PWM: more value never means less");
	CHECK(pwmStop,  "PWM: stop and 0 are the ESC's own stop (1500 on 3D, 1000 one-way)");
	CHECK(pwmRev,   "PWM: below 1500 us on a 3D ESC only for a 3D value below its stop");
	CHECK(pwmRange, "PWM: always 1000..2000 us");
	CHECK(wantStop, "wanted: stop and 0 never take an ESC over");
	CHECK(wantStock,"wanted: an undeclared 3D channel takes an ESC over only above 1500");
	CHECK(wanted(1400, true, true) && !wanted(1400, true, false), "wanted: a declared 3D channel may start in reverse");
	// the exact points the page and the bench were calibrated on
	CHECK(driveValue(2000, true, true) == 1000 && driveValue(1000, true, true) == -1000, "3D/3D: full scale both ways");
	CHECK(driveValue(2000, false, true) == 1000 && driveValue(1000, false, true) == 0, "one-way value on a 3D ESC: forward half range");
	CHECK(driveValue(2000, true, false) == 2000 && driveValue(1400, true, false) == 0, "3D value on a one-way ESC: 1500..2000 -> 0..2000");
	CHECK(driveValue(1500, false, false) == 1000 && driveValue(2000, false, false) == 2000, "one-way/one-way: 1000..2000 -> 0..2000");
	CHECK(pwmValue(2000, false, true) == 2000 && pwmValue(1500, true, false) == 1000, "PWM end points");

	printf(failures ? "\n%d CHECK(S) FAILED\n" : "\nALL CHECKS PASSED\n", failures);
	return failures ? 1 : 0;
}
