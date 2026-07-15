// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// as5600 — AS5600 magnetic encoder (I2C0, SDA=GP16 SCL=GP17, addr 0x36) — #4B position
// feedback. Read-only; independent of the ESC signal pins (GP10/GP11) and DShot PIO. Runs on
// core0. Thin class facade over the same Wire register reads the esc_tool serial `enc` command
// and web UI use — behavior byte-identical to the previous enc:: free functions.
#pragma once
#include <Arduino.h>
#include <Wire.h>

class As5600 {
public:
	static const uint8_t ADDR = 0x36;
	void begin() { Wire.setSDA(16); Wire.setSCL(17); Wire.begin(); Wire.setClock(400000); }
	bool rd(uint8_t reg, uint8_t* b, uint8_t n) {
		Wire.beginTransmission(ADDR); Wire.write(reg);
		if (Wire.endTransmission(false) != 0) return false;      // repeated-start
		if ((uint8_t)Wire.requestFrom((int)ADDR, (int)n) != n) return false;
		for (uint8_t i = 0; i < n; i++) b[i] = Wire.read();
		return true;
	}
	int u12(uint8_t reg) { uint8_t b[2]; if (!rd(reg, b, 2)) return -1; return ((b[0] & 0x0F) << 8) | b[1]; }
};
