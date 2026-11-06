// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// esc_pins — the ESC signal pin list (docs/esc-lifecycle.md "Signal pins"): how a list is read from
// text (the `pins` command, /cfg/pins) and which lists are allowed. ESC_SIGNAL_PINS (esc_config.h) is
// the compile-time default; escs:: (src/apps/esc_session.h) applies a stored list at boot.
//
// PORTABLE + TESTABLE: no Arduino / pico-sdk headers (native test: test/test_esc_pins.cpp).
#pragma once
#include <stdint.h>

namespace escpins {

// Capacity: the configurators show at most 8 motors (MSP_MOTOR, the passthrough's channel byte).
static const uint8_t MAX = 8;
// RP2040: GP0..GP29.
static const uint8_t GPIO_COUNT = 30;
// The Pico W's CYW43 radio: GP23 (power), GP24 (data / IRQ), GP25 (chip select), GP29 (clock).
static constexpr uint32_t RADIO = (1u << 23) | (1u << 24) | (1u << 25) | (1u << 29);

// BUSY: a valid list that cannot be applied now (escs::setPins after boot).
enum class Err : uint8_t { OK, EMPTY, TOO_MANY, BAD_NUMBER, NOT_GPIO, DUPLICATE, RESERVED, BUSY };
inline const char* errName(Err e) {
	static const char* const N[] = { "ok", "no pins", "more than 8 pins", "not a number", "not a GPIO (0..29)",
	                                 "listed twice", "used by this build", "ESCs in use (boot only)" };
	uint8_t k = (uint8_t)e;
	return k < 8 ? N[k] : "?";
}

// A list is allowed when it has 1..MAX entries, each a GPIO, none twice, none in `reserved` (bit g =
// GP g is used for something else by this build). bad = the first offending GPIO.
constexpr Err check(const uint8_t* p, uint8_t n, uint32_t reserved, uint8_t* bad = nullptr) {
	if (n == 0) return Err::EMPTY;
	if (n > MAX) return Err::TOO_MANY;
	for (uint8_t i = 0; i < n; i++) {
		Err e = Err::OK;
		if (p[i] >= GPIO_COUNT) e = Err::NOT_GPIO;
		else if (reserved >> p[i] & 1) e = Err::RESERVED;
		else for (uint8_t k = 0; k < i; k++) if (p[k] == p[i]) e = Err::DUPLICATE;
		if (e != Err::OK) { if (bad) *bad = p[i]; return e; }
	}
	return Err::OK;
}

// Text -> list: GPIO numbers separated by spaces or commas, each optionally written GP16 / gp16
// ("16 17 18 19", "16,17", "GP10 GP11"). Stops at the end of the string or a newline. Only the
// form is checked here (numbers, at most MAX); check() decides whether the list may be used.
// bad = the offending number (255 when it does not fit).
inline Err parse(const char* s, uint8_t* out, uint8_t& n, uint8_t* bad = nullptr) {
	n = 0;
	while (s && *s && *s != '\n' && *s != '\r') {
		if (*s == ' ' || *s == ',' || *s == '\t') { s++; continue; }
		if ((s[0] == 'G' || s[0] == 'g') && (s[1] == 'P' || s[1] == 'p')) s += 2;
		if (*s < '0' || *s > '9') return Err::BAD_NUMBER;
		unsigned v = 0;
		while (*s >= '0' && *s <= '9') { v = v * 10 + (unsigned)(*s - '0'); if (v > 255) v = 256; s++; }
		if (*s && *s != ' ' && *s != ',' && *s != '\t' && *s != '\n' && *s != '\r') return Err::BAD_NUMBER;
		if (v > 255) { if (bad) *bad = 255; return Err::NOT_GPIO; }
		if (n == MAX) return Err::TOO_MANY;
		out[n++] = (uint8_t)v;
	}
	return n ? Err::OK : Err::EMPTY;
}

// Two lists are the same (order counts: the list order is the ESC index order).
inline bool same(const uint8_t* a, uint8_t na, const uint8_t* b, uint8_t nb) {
	if (na != nb) return false;
	for (uint8_t i = 0; i < na; i++) if (a[i] != b[i]) return false;
	return true;
}

}  // namespace escpins
