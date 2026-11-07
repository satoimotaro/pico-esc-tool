// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// Host stand-in for pico-bidir-dshot (test/sim): every frame goes to the simulated ESC on the pin,
// and telemetry comes back from it. PIO state machines are counted like the Pico W's: 4 on pio0,
// 3 free on pio1 (the radio has one and 6 instructions, so the 28-instruction bidir program fits
// only pio0, and a pio holds one of the two programs); a driver with none left reports initError().
#pragma once
#include <cstdint>
#define ESC_STATUS_MAX_STRESS_MASK 0b00001111

struct pio_sim { int id; };
typedef pio_sim* PIO;
extern PIO pio0, pio1;

enum class BidirDshotTelemetryType : uint8_t {
	ERPM, OTHER_VALUE, CHECKSUM_ERROR, NO_PACKET, VOLTAGE, CURRENT, TEMPERATURE, STATUS, STRESS,
	DEBUG_FRAME_1, DEBUG_FRAME_2,
};

namespace sim {
bool takeSm(PIO pio, bool bidir);
void giveSm(PIO pio);
void frame(uint8_t pin, bool bidir, bool command, uint16_t value);   // one DShot frame on the wire
void driverGone(uint8_t pin);
BidirDshotTelemetryType telemetry(uint8_t pin, uint32_t* value);
}

class BidirDShotX1 {
public:
	BidirDShotX1(uint8_t pin, uint32_t speed = 600, PIO pio = pio0, int8_t sm = -1)
	    : pin_(pin), pio_(pio) { (void)speed; (void)sm; ok_ = sim::takeSm(pio, true); }
	~BidirDShotX1() { if (ok_) { sim::giveSm(pio_); sim::driverGone(pin_); } }
	void sendThrottle(uint16_t t) { if (ok_) sim::frame(pin_, true, false, t); }
	void sendRaw11Bit(uint16_t d) { if (ok_) sim::frame(pin_, true, true, d); }
	BidirDshotTelemetryType getTelemetryPacket(uint32_t* v) {
		return ok_ ? sim::telemetry(pin_, v) : BidirDshotTelemetryType::NO_PACKET;
	}
	bool initError() { return !ok_; }
private:
	uint8_t pin_; PIO pio_; bool ok_;
};

class DShotX4 {
public:
	DShotX4(uint8_t pinBase, uint8_t pinCount, uint32_t speed = 600, PIO pio = pio0, int8_t sm = -1)
	    : pin_(pinBase), pio_(pio) { (void)pinCount; (void)speed; (void)sm; ok_ = sim::takeSm(pio, false); }
	~DShotX4() { if (ok_) { sim::giveSm(pio_); sim::driverGone(pin_); } }
	void sendThrottles(uint16_t t[4]) { if (ok_) sim::frame(pin_, false, false, t[0]); }
	void sendRaw11Bit(uint16_t d[4]) { if (ok_) sim::frame(pin_, false, true, d[0]); }
	bool initError() { return !ok_; }
private:
	uint8_t pin_; PIO pio_; bool ok_;
};
