// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// Native (host g++) test for the ESC signal pin list (esc_pins.h): what `pins <gpio>...` and a
// stored /cfg/pins are allowed to be.
//     g++ -std=c++17 -O2 -Wall -Wextra -Werror -o /tmp/tpins test_esc_pins.cpp && /tmp/tpins
#include "../esc_pins.h"
#include <cstdio>

using namespace escpins;

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); failures++; } \
	else printf("ok: %s\n", msg); } while (0)

// The panel build: LCD on GP5..15, the mode pin GP22, the radio.
static const uint32_t PANEL = RADIO | (0x7FFu << 5) | (1u << 22);
// The UART build: GP0/1 for the console, the mode pin GP22, the radio.
static const uint32_t UART = RADIO | 0x3u | (1u << 22);

// The compile-time defaults are checked at compile time too (esc_session.h static_assert).
static constexpr uint8_t PANEL_DEFAULT[] = { 16, 17, 18, 19 };
static_assert(check(PANEL_DEFAULT, 4, PANEL) == Err::OK, "the panel default passes on the panel");

int main() {
	{   // check
		uint8_t bad = 0;
		const uint8_t ok4[] = { 16, 17, 18, 19 };
		CHECK(check(ok4, 4, PANEL) == Err::OK, "GP16..19 are allowed on the panel");
		CHECK(check(ok4, 0, PANEL) == Err::EMPTY, "an empty list is refused");
		const uint8_t nine[] = { 0, 2, 3, 4, 16, 17, 18, 19, 20 };
		CHECK(check(nine, 9, RADIO) == Err::TOO_MANY, "9 pins are refused (8 is the passthrough limit)");
		CHECK(check(nine, 8, RADIO) == Err::OK, "8 pins are allowed");
		const uint8_t dup[] = { 16, 17, 16 };
		CHECK(check(dup, 3, PANEL, &bad) == Err::DUPLICATE && bad == 16, "a pin listed twice is refused, naming it");
		const uint8_t lcd[] = { 16, 10 };
		CHECK(check(lcd, 2, PANEL, &bad) == Err::RESERVED && bad == 10, "an LCD pin is refused on the panel");
		CHECK(check(lcd, 2, RADIO) == Err::OK, "... and allowed where there is no LCD");
		const uint8_t radio[] = { 25 };
		CHECK(check(radio, 1, PANEL, &bad) == Err::RESERVED && bad == 25, "a Pico W radio pin is refused");
		const uint8_t uart[] = { 1, 10 };
		CHECK(check(uart, 2, UART, &bad) == Err::RESERVED && bad == 1, "the UART console pins are refused on the UART build");
		const uint8_t mode[] = { 22 };
		CHECK(check(mode, 1, UART) == Err::RESERVED, "the mode pin is refused");
		const uint8_t big[] = { 16, 30 };
		CHECK(check(big, 2, PANEL, &bad) == Err::NOT_GPIO && bad == 30, "GP30 is not a GPIO");
		const uint8_t first[] = { 99, 25 };
		CHECK(check(first, 2, PANEL, &bad) == Err::NOT_GPIO && bad == 99, "the first offending pin is the one reported");
	}
	{   // parse
		uint8_t p[MAX] = {0}, n = 0, bad = 0;
		CHECK(parse("16 17 18 19", p, n) == Err::OK && n == 4 && p[0] == 16 && p[3] == 19, "spaces");
		CHECK(parse("10,11", p, n) == Err::OK && n == 2 && p[1] == 11, "commas");
		CHECK(parse(" GP16, gp17 \r\n", p, n) == Err::OK && n == 2 && p[0] == 16 && p[1] == 17, "GP prefix, padding, CRLF");
		CHECK(parse("16 17\n18", p, n) == Err::OK && n == 2, "stops at a newline (one line of /cfg/pins)");
		CHECK(parse("", p, n) == Err::EMPTY && n == 0, "nothing is EMPTY");
		CHECK(parse("  ,", p, n) == Err::EMPTY, "only separators is EMPTY");
		CHECK(parse("16 x", p, n) == Err::BAD_NUMBER, "a word is not a number");
		CHECK(parse("16a", p, n) == Err::BAD_NUMBER, "a number with a tail is not a number");
		CHECK(parse("-1", p, n) == Err::BAD_NUMBER, "a sign is not a number");
		CHECK(parse("16 300", p, n, &bad) == Err::NOT_GPIO && bad == 255, "a number past 255 is not a GPIO");
		CHECK(parse("1 2 3 4 5 6 7 8 9", p, n) == Err::TOO_MANY, "a 9th pin is TOO_MANY");
		CHECK(parse("40", p, n) == Err::OK && check(p, n, 0) == Err::NOT_GPIO, "parse takes 40; check refuses it");
		CHECK(parse(nullptr, p, n) == Err::EMPTY, "a null string is EMPTY");
	}
	{   // same
		const uint8_t a[] = { 16, 17 }, b[] = { 17, 16 };
		CHECK(same(a, 2, a, 2) && !same(a, 2, b, 2) && !same(a, 2, a, 1), "same: order and length count");
	}
	CHECK(errName(Err::BUSY)[0] != '?' && errName((Err)200)[0] == '?', "every Err has a name");
	printf(failures ? "\n%d CHECK(S) FAILED\n" : "\nALL CHECKS PASSED\n", failures);
	return failures ? 1 : 0;
}
