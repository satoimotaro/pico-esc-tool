// SPDX-License-Identifier: GPL-3.0-or-later
//
// blheli_bl — BLHeli-S / SiLabs EFM8 1-wire bootloader client for RP2040.
//
// Speaks the ESC signal-wire bootloader protocol DIRECTLY (no flight controller):
// connect, read device signature, read/write flash + EEPROM, erase, run.
// esc_flash (firmware programming) and esc_setup (config params) build on this.
//
// Protocol constants (baud, framing, command bytes, CRC) live in PROTOCOL.md,
// derived from Betaflight's 4-way avr/silabs bootloader + esc-configurator.
// >>> Values marked [TODO:proto] are filled from that research. <<<
#pragma once
#include <Arduino.h>
#include <stdint.h>

namespace blheli_bl {

enum class McuType : uint8_t { UNKNOWN, SILABS_EFM8, ATMEL_AVR, ARM_BB51 };

struct DeviceInfo {
	bool        valid = false;
	uint8_t     signature[2] = {0, 0};  // device signature (e.g. EFM8BB21)
	uint8_t     bootInfo[8]  = {0};     // raw BootInfo / BootMsg response
	McuType     mcu  = McuType::UNKNOWN;
	const char* name = nullptr;         // resolved chip name, or nullptr
};

struct Config {
	uint8_t  signalPin;        // shared ESC signal wire (same pin DShot uses)
	uint32_t baud = 0;         // 0 => protocol default (see PROTOCOL.md) [TODO:proto]
};

class Bootloader {
public:
	explicit Bootloader(const Config& cfg) : cfg_(cfg) {}

	// Bring up the single-wire half-duplex transport on the signal pin.
	bool begin();

	// Send BootInit/hello and confirm the ESC sits in its bootloader.
	// The SiLabs bootloader only listens for a short window after power-up, so the
	// ESC usually must be (re)powered while connect() retries. Returns true once the
	// expected boot response is seen.
	bool connect();

	// Read device signature / boot info (identify EFM8BB21 etc).
	bool readDeviceInfo(DeviceInfo& out);

	// Read `len` bytes from `addr` (flash or EEPROM space) into buf.
	bool readMemory(uint16_t addr, uint8_t* buf, uint16_t len);

	// --- write path (Phase A1) ---
	bool setAddress(uint16_t addr);
	bool erasePage(uint16_t addr);
	bool writeMemory(uint16_t addr, const uint8_t* buf, uint16_t len);
	bool run();                // exit bootloader → start app

	void end();
	bool connected() const { return connected_; }

private:
	Config cfg_;
	bool   connected_ = false;

	// --- transport: single-wire UART on cfg_.signalPin (SerialPIO/bit-bang) ---
	// Implementation chosen once baud/framing is confirmed (see PROTOCOL.md §A).
	bool sendFrame(const uint8_t* data, uint16_t len);
	bool recvFrame(uint8_t* buf, uint16_t maxLen, uint16_t& outLen, uint32_t timeoutMs);

	// Protocol CRC (poly/algorithm per PROTOCOL.md §C) [TODO:proto]
	static uint16_t crc16(const uint8_t* data, uint16_t len);
};

// Resolve a device signature to a human name (table in blheli_bl.cpp) [TODO:proto].
const char* signatureName(const uint8_t sig[2]);

} // namespace blheli_bl
