// SPDX-License-Identifier: GPL-3.0-or-later
//
// blheli_bl implementation — SiLabs EFM8BB 1-wire bootloader ("BLB").
// Ported from Betaflight serial_4way_avrootloader.c (author 4712 / H. Reddmann),
// cross-checked with esc-configurator + BLHeli_S. See PROTOCOL.md for every value.
//
// UNTESTED ON HARDWARE — timing and turnaround must be validated on the bench.
#include "blheli_bl.h"
#include <string.h>

namespace blheli_bl {

// ---- command bytes (BF-AVR) ----
static constexpr uint8_t CMD_RUN            = 0x00;
static constexpr uint8_t CMD_PROG_FLASH     = 0x01;
static constexpr uint8_t CMD_ERASE_FLASH    = 0x02;
static constexpr uint8_t CMD_READ_FLASH_SIL = 0x03;
static constexpr uint8_t CMD_READ_EEPROM    = 0x04;
static constexpr uint8_t CMD_PROG_EEPROM    = 0x05;
static constexpr uint8_t CMD_KEEP_ALIVE     = 0xFD;
static constexpr uint8_t CMD_SET_BUFFER     = 0xFE;
static constexpr uint8_t CMD_SET_ADDRESS    = 0xFF;

// BootInit: 8 zeros, 0x0D, "BLHeli", then fixed init checksum F4 7D. Sent w/o CRC.
static const uint8_t BOOT_INIT[] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0x0D, 'B', 'L', 'H', 'e', 'l', 'i', 0xF4, 0x7D };

// ---- CRC-16, poly 0xA001 (reflected), init 0x0000, transmitted little-endian ----
uint16_t Bootloader::crcAdd(uint16_t crc, uint8_t b) {
	for (uint8_t i = 0; i < 8; i++) {
		if (((b ^ crc) & 0x0001) != 0) crc = (crc >> 1) ^ 0xA001;
		else                            crc =  crc >> 1;
		b >>= 1;
	}
	return crc;
}
uint16_t Bootloader::crcBuf(const uint8_t* data, uint16_t len) {
	uint16_t c = 0;
	for (uint16_t i = 0; i < len; i++) c = crcAdd(c, data[i]);
	return c;
}

// ---- transport ----
void Bootloader::setTx() { pinMode(cfg_.signalPin, OUTPUT); digitalWrite(cfg_.signalPin, HIGH); }
void Bootloader::setRx() { pinMode(cfg_.signalPin, INPUT_PULLUP); }

void Bootloader::txByte(uint8_t b) {
	const uint8_t pin = cfg_.signalPin;
	noInterrupts();
	digitalWrite(pin, LOW);                       // start bit
	delayMicroseconds(bitTimeUs_);
	for (uint8_t i = 0; i < 8; i++) {             // 8 data bits, LSB first
		digitalWrite(pin, (b >> i) & 1);
		delayMicroseconds(bitTimeUs_);
	}
	digitalWrite(pin, HIGH);                      // stop bit
	delayMicroseconds(bitTimeUs_);
	interrupts();
}

bool Bootloader::rxByte(uint8_t& out, uint32_t timeoutMs) {
	const uint8_t pin = cfg_.signalPin;
	uint32_t t0 = millis();
	while (digitalRead(pin)) {                     // wait for start-bit falling edge
		if (millis() - t0 > timeoutMs) return false;
	}
	noInterrupts();
	delayMicroseconds(bitTime34Us_);               // land ~mid start bit
	uint16_t bits = 0;
	for (uint8_t i = 0; i < 10; i++) {             // start + 8 data + stop
		if (digitalRead(pin)) bits |= (1u << i);
		delayMicroseconds(bitTimeUs_);
	}
	interrupts();
	if (bits & 0x0001) return false;               // start bit must be 0
	if (!(bits & 0x0200)) return false;            // stop bit must be 1
	out = (bits >> 1) & 0xFF;
	return true;
}

// ---- framing ----
void Bootloader::sendCmd(const uint8_t* data, uint16_t len) {
	setTx();
	uint16_t c = 0;
	for (uint16_t i = 0; i < len; i++) { txByte(data[i]); c = crcAdd(c, data[i]); }
	if (connected_) { txByte(c & 0xFF); txByte((c >> 8) & 0xFF); }
	setRx();
}

bool Bootloader::getAck(uint8_t& ack, uint32_t timeoutMs) {
	return rxByte(ack, timeoutMs);
}

bool Bootloader::readBuf(uint8_t* buf, uint16_t n, uint32_t timeoutMs) {
	uint16_t c = 0;
	for (uint16_t i = 0; i < n; i++) {
		if (!rxByte(buf[i], timeoutMs)) return false;
		c = crcAdd(c, buf[i]);
	}
	if (connected_) {
		uint8_t cl, ch;
		if (!rxByte(cl, timeoutMs) || !rxByte(ch, timeoutMs)) return false;
		if ((c & 0xFF) != cl || ((c >> 8) & 0xFF) != ch) return false;  // CRC mismatch
	}
	uint8_t ack;
	if (!rxByte(ack, timeoutMs)) return false;
	return ack == br_SUCCESS;
}

bool Bootloader::setAddress(uint16_t addr) {
	if (addr == 0xFFFF) return true;               // 0xFFFF = keep current
	uint8_t cmd[4] = { CMD_SET_ADDRESS, 0, uint8_t(addr >> 8), uint8_t(addr & 0xFF) };
	sendCmd(cmd, 4);
	uint8_t ack;
	return getAck(ack) && ack == br_SUCCESS;
}

bool Bootloader::setBuffer(const uint8_t* data, uint16_t len) {
	// length byte 0 => 256 (per protocol); BF sets hi/lo of the 16-bit length.
	uint8_t cmd[4] = { CMD_SET_BUFFER, 0, uint8_t(len >> 8), uint8_t(len & 0xFF) };
	if ((len & 0xFF) == 0 && (len >> 8) == 1) { /* 0x0100 = 256, sent as is */ }
	sendCmd(cmd, 4);
	uint8_t ack;
	if (!getAck(ack)) return false;                // first ACK (0xFF) then buffer follows
	// stream the data buffer (with CRC when connected)
	sendCmd(data, len);
	return getAck(ack) && ack == br_SUCCESS;
}

// ---- public ----
bool Bootloader::begin() {
	if (cfg_.baud == 0) cfg_.baud = 19200;
	bitTimeUs_   = 1000000UL / cfg_.baud;          // ~52 us @ 19200
	bitTime34Us_ = (bitTimeUs_ * 3) / 4;           // ~39 us
	setRx();                                        // idle: input, pulled high
	return true;
}

bool Bootloader::connect() {
	connected_ = false;
	dev_ = DeviceInfo{};
	// BootInit is sent WITHOUT CRC (connected_ is false).
	sendCmd(BOOT_INIT, sizeof(BOOT_INIT));
	// Reply: "471" + BootMsgLast + sigHi + sigLo + bootVer + bootPages (8 bytes, no CRC/ACK).
	uint8_t info[8] = {0};
	for (uint8_t i = 0; i < 8; i++) {
		if (!rxByte(info[i], 100)) return false;
	}
	memcpy(dev_.bootInfo, info, 8);
	if (!(info[0] == '4' && info[1] == '7' && info[2] == '1')) return false;
	dev_.signature[0] = info[4];                    // sigHi
	dev_.signature[1] = info[5];                    // sigLo
	dev_.bootVersion  = info[6];
	dev_.bootPages    = info[7];
	uint16_t w = dev_.signatureWord();
	if (w == 0) return false;
	dev_.mcu   = mcuTypeFor(w);
	dev_.name  = signatureName(w);
	dev_.valid = true;
	connected_ = true;                              // from here on, frames carry CRC
	return true;
}

bool Bootloader::readDeviceInfo(DeviceInfo& out) {
	if (!connected_) return false;
	out = dev_;
	return dev_.valid;
}

bool Bootloader::readEeprom(uint16_t addr, uint8_t* buf, uint16_t len) {
	if (!connected_ || len == 0 || len > 256) return false;
	if (!setAddress(addr)) return false;
	uint8_t cmd[2] = { CMD_READ_EEPROM, uint8_t(len & 0xFF) };  // 0 => 256
	sendCmd(cmd, 2);
	return readBuf(buf, len);
}

bool Bootloader::readFlash(uint16_t addr, uint8_t* buf, uint16_t len) {
	if (!connected_ || len == 0 || len > 256) return false;
	if (!setAddress(addr)) return false;
	uint8_t cmd[2] = { CMD_READ_FLASH_SIL, uint8_t(len & 0xFF) };
	sendCmd(cmd, 2);
	return readBuf(buf, len);
}

bool Bootloader::erasePage(uint16_t addr) {
	if (!connected_) return false;
	if (!setAddress(addr)) return false;
	uint8_t cmd[2] = { CMD_ERASE_FLASH, 0x01 };
	sendCmd(cmd, 2);
	uint8_t ack;
	return getAck(ack, 1000) && ack == br_SUCCESS;  // erase is slow
}

bool Bootloader::writeFlash(uint16_t addr, const uint8_t* buf, uint16_t len) {
	if (!connected_) return false;
	if (!setAddress(addr) || !setBuffer(buf, len)) return false;
	uint8_t cmd[2] = { CMD_PROG_FLASH, 0x01 };
	sendCmd(cmd, 2);
	uint8_t ack;
	return getAck(ack, 500) && ack == br_SUCCESS;
}

bool Bootloader::writeEeprom(uint16_t addr, const uint8_t* buf, uint16_t len) {
	if (!connected_) return false;
	if (!setAddress(addr) || !setBuffer(buf, len)) return false;
	uint8_t cmd[2] = { CMD_PROG_EEPROM, 0x01 };
	sendCmd(cmd, 2);
	uint8_t ack;
	return getAck(ack, 500) && ack == br_SUCCESS;
}

bool Bootloader::keepAlive() {
	if (!connected_) return false;
	uint8_t cmd[2] = { CMD_KEEP_ALIVE, 0x00 };
	sendCmd(cmd, 2);
	uint8_t ack;
	if (!getAck(ack)) return false;
	return ack == br_ERRORCOMMAND;                  // NAK to invalid cmd = alive
}

bool Bootloader::run() {
	if (!connected_) return false;
	uint8_t cmd[2] = { CMD_RUN, 0x00 };
	sendCmd(cmd, 2);
	connected_ = false;
	return true;
}

void Bootloader::end() { connected_ = false; setRx(); }

// ---- signature table (esc-configurator Silabs.js) ----
McuType mcuTypeFor(uint16_t w) {
	if (w > 0xE800 && w < 0xF900) return McuType::SILABS_EFM8;
	return McuType::UNKNOWN;
}
const char* signatureName(uint16_t w) {
	switch (w) {
		case 0xE8B1: return "EFM8BB10x";
		case 0xE8B2: return "EFM8BB21x";   // LittleBee Spring 30A
		case 0xE8B5: return "EFM8BB51x";
		default:     return nullptr;
	}
}

} // namespace blheli_bl
