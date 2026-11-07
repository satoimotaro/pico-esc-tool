// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// motor_test — what a motor-test value means for one ESC. A configurator's motor test sends
// 1000..2000 per channel: one-way (1000 = stop) or 3D (1500 = stop, below it reverse), as declared
// for that channel (the web page's `mtmode`) or as FEATURE_3D said (a stock configurator). The ESC
// itself is 3D or one-way by its own settings. Every pairing is safe: reverse only when the value is
// 3D AND the ESC is, and any other pairing runs forward or not at all. 0 (a short or cleared frame)
// is stop, never "below full reverse".
//
// PORTABLE + TESTABLE: no Arduino headers (test/test_motor_test.cpp checks every pairing).
#pragma once
#include <stdint.h>

namespace mt {

// The stop value of a channel.
inline uint16_t stopOf(bool c3d) { return c3d ? 1500 : 1000; }

// A value that takes an ESC over (arms it). declared: the page declared this channel's meaning, and
// may start a 3D channel in reverse; a stock client never takes an ESC over at or below its stop,
// so a value it did not mean as 3D cannot reach an ESC as reverse.
inline bool wanted(uint16_t v, bool c3d, bool declared) {
	if (!v) return false;
	if (!c3d) return v > 1000;
	return declared ? v != 1500 : v > 1500;
}

// The value as an escs::drive() value for an ESC that is reversible (rev) or not: signed thrust
// -1000..1000 on a 3D ESC, throttle 0..2000 on a one-way one.
//   ESC 3D,  value 3D:      1000..1500..2000 -> -1000..0..1000
//   ESC 3D,  value one-way: 1000..2000 -> 0..1000 (forward, half range)
//   one-way, value 3D:      1500..2000 -> 0..2000 (at or below 1500: stop)
//   one-way, value one-way: 1000..2000 -> 0..2000
inline int driveValue(uint16_t v, bool c3d, bool rev) {
	if (!v) v = stopOf(c3d);
	if (v > 2000) v = 2000;
	if (v < 1000) v = 1000;
	if (rev) return c3d ? (int(v) - 1500) * 2 : (v > 1000 ? int(v) - 1000 : 0);
	return c3d ? (v > 1500 ? (int(v) - 1500) * 4 : 0) : (v > 1000 ? (int(v) - 1000) * 2 : 0);
}

// The value as a servo pulse (us) for that ESC, by the same pairing rules: 1500 is stop on a 3D ESC
// (1000 is full reverse there), 1000 on a one-way one.
inline uint16_t pwmValue(uint16_t v, bool c3d, bool rev) {
	if (!v) v = stopOf(c3d);
	if (v > 2000) v = 2000;
	if (v < 1000) v = 1000;
	if (rev) return c3d ? v : (uint16_t)(1500 + (v - 1000) / 2);
	return c3d ? (uint16_t)(v > 1500 ? 1000 + (v - 1500) * 2 : 1000) : v;
}

}  // namespace mt
