// SPDX-License-Identifier: GPL-3.0-or-later
//
// blheli_bl implementation.
//
// STATUS: scaffold. The transport (single-wire half-duplex UART) and the exact
// command frames / CRC / baud are being finalized from PROTOCOL.md (Betaflight
// 4-way + esc-configurator research). Command bodies below are stubs marked
// [TODO:proto]; they compile and fail safely (return false) until filled in.
#include "blheli_bl.h"

namespace blheli_bl {

// ---- Protocol constants -----------------------------------------------------
// Filled from PROTOCOL.md. Placeholders until research lands.
// static constexpr uint32_t kDefaultBaud = 19200;      // [TODO:proto] confirm
// static constexpr uint8_t  CMD_...       = 0x00;      // [TODO:proto] command bytes
// static const uint8_t      BOOT_INIT[]   = {...};     // [TODO:proto] hello bytes

// ---- Transport --------------------------------------------------------------
// Plan (see PROTOCOL.md §A/§F): use earlephilhower SerialPIO for a single-pin,
// half-duplex UART on cfg_.signalPin, with line-turnaround handling. The same pin
// is shared with DShot, so the caller must ensure DShot is stopped before begin().

bool Bootloader::begin() {
	// [TODO:proto] init single-wire UART @ baud on cfg_.signalPin
	return false;
}

bool Bootloader::sendFrame(const uint8_t*, uint16_t) {
	return false; // [TODO:proto]
}

bool Bootloader::recvFrame(uint8_t*, uint16_t, uint16_t& outLen, uint32_t) {
	outLen = 0;
	return false; // [TODO:proto]
}

uint16_t Bootloader::crc16(const uint8_t*, uint16_t) {
	return 0; // [TODO:proto] poly/algorithm per PROTOCOL.md §C
}

// ---- Bootloader commands ----------------------------------------------------

bool Bootloader::connect() {
	// [TODO:proto] send BOOT_INIT, read boot response, verify, set connected_.
	connected_ = false;
	return false;
}

bool Bootloader::readDeviceInfo(DeviceInfo& out) {
	out = DeviceInfo{};
	if (!connected_) return false;
	// [TODO:proto] issue device-info/signature command, fill out.signature/bootInfo,
	// resolve out.name via signatureName().
	return false;
}

bool Bootloader::readMemory(uint16_t, uint8_t*, uint16_t) {
	if (!connected_) return false;
	return false; // [TODO:proto] setAddress + read command
}

bool Bootloader::setAddress(uint16_t)               { return false; } // [TODO:proto] A1
bool Bootloader::erasePage(uint16_t)                { return false; } // [TODO:proto] A1
bool Bootloader::writeMemory(uint16_t, const uint8_t*, uint16_t) { return false; } // A1
bool Bootloader::run()                              { return false; } // [TODO:proto]
void Bootloader::end()                              { connected_ = false; }

// ---- Signature table --------------------------------------------------------
const char* signatureName(const uint8_t /*sig*/[2]) {
	// [TODO:proto] map EFM8BB21/BB2x signatures → names (source: esc-configurator).
	return nullptr;
}

} // namespace blheli_bl
