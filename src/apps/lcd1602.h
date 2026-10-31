// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// lcd1602 — a 16x2 HD44780 character LCD (SC1602) on an 8-bit parallel bus, write-only.
// RW is held LOW for good, so the 5 V LCD never drives the bus and never reaches the Pico.
// Fixed delays stand in for the busy flag (most commands take 37 us).
//
// The display NEVER blocks the core0 loop for long: callers set whole rows with `row()`, and
// `poll()` writes at most a few changed characters per call (~50 us each), so DShot frames keep
// flowing while the screen catches up. Only `begin()` waits (≈ 60 ms, at boot).
#pragma once
#include <Arduino.h>

class Lcd1602 {
public:
	// d = D0..D7 GPIOs.
	Lcd1602(uint8_t rs, uint8_t rw, uint8_t e, const uint8_t (&d)[8]) : rs_(rs), rw_(rw), e_(e) {
		memcpy(d_, d, 8);
	}

	void begin() {
		digitalWrite(rw_, LOW);       // set the level before enabling the output
		pinMode(rw_, OUTPUT);
		pinMode(rs_, OUTPUT);
		digitalWrite(e_, LOW);
		pinMode(e_, OUTPUT);
		for (uint8_t p : d_) pinMode(p, OUTPUT);
		delay(50);                    // power-on settling
		write(false, 0x30, 4500);     // reset sequence for 8-bit mode
		write(false, 0x30, 150);
		write(false, 0x30);
		write(false, 0x38);           // 8-bit, 2 lines, 5x8 font
		write(false, 0x08);           // display off
		write(false, 0x01, 2000);     // clear
		write(false, 0x06);           // cursor moves right, no shift
		write(false, 0x0C);           // display on, no cursor
		memset(shown_, ' ', sizeof(shown_));
		memset(want_, ' ', sizeof(want_));
		addr_ = 0xFF;
	}

	// Set a row's wanted text (padded/truncated to 16). Nothing is written until poll().
	void row(uint8_t r, const char* s) {
		if (r > 1) return;
		size_t n = strlen(s);
		for (uint8_t i = 0; i < 16; i++) want_[r][i] = i < n ? s[i] : ' ';
	}

	// Write up to `budget` changed characters. Call every loop.
	void poll(uint8_t budget = 4) {
		for (uint8_t k = 0; k < 32 && budget; k++) {
			uint8_t r = scan_ / 16, c = scan_ % 16;
			scan_ = (scan_ + 1) % 32;
			if (want_[r][c] == shown_[r][c]) continue;
			uint8_t a = (r ? 0x40 : 0x00) + c;
			if (a != addr_) { write(false, 0x80 | a); budget--; }
			write(true, (uint8_t)want_[r][c]);
			shown_[r][c] = want_[r][c];
			addr_ = a + 1;            // the LCD's address counter auto-increments
			budget--;
		}
	}

private:
	void write(bool rs, uint8_t v, uint32_t waitUs = 45) {
		digitalWrite(rs_, rs);
		for (uint8_t i = 0; i < 8; i++) digitalWrite(d_[i], (v >> i) & 1);
		delayMicroseconds(1);
		digitalWrite(e_, HIGH);
		delayMicroseconds(1);
		digitalWrite(e_, LOW);
		delayMicroseconds(waitUs);
	}

	uint8_t rs_, rw_, e_, d_[8];
	char    shown_[2][16], want_[2][16];
	uint8_t scan_ = 0, addr_ = 0xFF;
};
