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

// As5600Tracker — high-rate, DE-ALIASING velocity tracker over As5600.
//
// WHY: the host used to poll the `enc` line at ~50 Hz and unwrap the 12-bit angle itself. Above
// ~1350 mech RPM the rotor turns more than half a revolution per 20 ms sample, so the host-side
// modulo unwrap aliases (Nyquist) — reverse (which runs faster than forward) folded into garbage /
// wrong-sign velocity, which blocked clean reverse crossover measurement. FIX: sample the AS5600
// on-device at ~1 kHz (per-sample travel << half a rev at any real speed), unwrap into a signed
// accumulator, and compute a windowed signed velocity HERE. The host reads the de-aliased velocity
// directly (`encv` line) instead of unwrapping a slow, aliasing angle stream.
//
// Runs on ONE core only (core0, from loop()); poll() is gated to SAMPLE_US so it costs one ~150 us
// I2C read per ~1 ms. All snapshot fields are 32-bit aligned scalars (atomic single-word reads on
// the RP2040 Cortex-M0+), so the same-core command handlers read them without locking.
class As5600Tracker {
public:
	// Poll cadence + velocity window. SAMPLE_US=800 -> ~1.25 kHz -> de-aliases up to
	// 1/(2*800us) = 625 rev/s = 37500 mech RPM (far above any real operating point).
	static const uint16_t SAMPLE_US  = 800;
	static const uint32_t VEL_WIN_US = 20000;   // 20 ms velocity window (matches host tick)
	static const uint32_t HEALTH_US  = 50000;   // magnet-health regs refreshed at 20 Hz
	static const uint8_t  NWIN       = 40;       // ring depth: 40*0.8ms = 32ms > VEL_WIN_US

	void begin() {
		dev_.begin();
		dev_.rd(0x0B, &status_, 1);              // probe: STATUS read implies the sensor ACKed
		present_ = (dev_.u12(0x0C) >= 0);
	}

	// Call frequently from the owning core's loop. Self-gated to SAMPLE_US; cheap when not due.
	void poll() {
		uint32_t now = micros();
		if ((uint32_t)(now - last_us_) < SAMPLE_US) return;
		last_us_ = now;
		int a = dev_.u12(0x0C);                  // RAW_ANGLE 0..4095
		if (a < 0) { miss_++; return; }
		present_ = true;
		if (have_prev_) {
			int d = ((a - prev_raw_ + 2048) & 0x0FFF) - 2048;   // signed -2048..2047
			accum_ += d;
		}
		prev_raw_ = a;
		have_prev_ = true;
		raw_ = (uint16_t)a;
		buf_[head_].t = now;
		buf_[head_].acc = accum_;
		uint8_t nh = (uint8_t)((head_ + 1) % NWIN);
		if (nh == tail_) tail_ = (uint8_t)((tail_ + 1) % NWIN);  // full -> drop oldest
		head_ = nh;
		samples_++;
		computeRpm(now);
		if ((uint32_t)(now - lastHealth_) >= HEALTH_US) {
			lastHealth_ = now;
			uint8_t st = 0, g = 0;
			dev_.rd(0x0B, &st, 1);
			dev_.rd(0x1A, &g, 1);
			int m = dev_.u12(0x1B);
			status_ = st;
			agc_ = g;
			if (m >= 0) mag_ = (uint16_t)m;
		}
	}

	// snapshot accessors (call from the SAME core as poll())
	float    rpm()     const { return rpm_; }       // de-aliased signed mech RPM (raw-increasing = +)
	int32_t  accum()   const { return accum_; }     // unwrapped signed position (ticks, 4096/rev)
	uint16_t raw()     const { return raw_; }
	uint32_t samples() const { return samples_; }
	bool     present() const { return present_; }
	uint8_t  status()  const { return status_; }
	uint8_t  agc()     const { return agc_; }
	uint16_t mag()     const { return mag_; }

private:
	void computeRpm(uint32_t now) {
		// Estimate velocity over ~VEL_WIN_US: from the oldest ring sample still inside the window
		// to the newest. This de-quantizes low speed (long baseline) AND de-aliases high speed
		// (short per-sample travel). Guarded: needs >=2 samples and dt>0.
		uint8_t i = tail_, chosen = tail_;
		while (i != head_) {
			if ((uint32_t)(now - buf_[i].t) <= VEL_WIN_US) { chosen = i; break; }
			i = (uint8_t)((i + 1) % NWIN);
		}
		uint32_t dt = now - buf_[chosen].t;
		if (dt == 0) return;
		int32_t dacc = accum_ - buf_[chosen].acc;
		rpm_ = (float)dacc / 4096.0f * (60.0e6f / (float)dt);
	}

	struct Snap { uint32_t t; int32_t acc; };
	As5600  dev_;
	Snap     buf_[NWIN];
	uint8_t  head_ = 0, tail_ = 0;
	int      prev_raw_ = 0;
	bool     have_prev_ = false;
	volatile int32_t  accum_ = 0;
	volatile float    rpm_ = 0.0f;
	volatile uint16_t raw_ = 0;
	uint32_t last_us_ = 0, lastHealth_ = 0, samples_ = 0, miss_ = 0;
	bool     present_ = false;
	uint8_t  status_ = 0, agc_ = 0;
	uint16_t mag_ = 0;
};
