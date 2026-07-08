// SPDX-License-Identifier: GPL-3.0-or-later
//
// blheli_bl — BLHeli-S / SiLabs EFM8BB 1-wire bootloader client for RP2040.
//
// Speaks the ESC signal-wire bootloader protocol DIRECTLY (no flight controller):
// connect, read device signature, read/write flash + EEPROM, erase, run.
// esc_flash (firmware programming) and esc_setup (config params) build on this.
//
// Protocol authority: Betaflight serial_4way_avrootloader.c (SiLabs "BLB"), cross-
// checked with esc-configurator + BLHeli_S source. Details in PROTOCOL.md.
// Physical layer: 19200 8N1, non-inverted, half-duplex single wire (idle HIGH).
//
// STATUS: full read path + write primitives implemented, but UNTESTED ON HARDWARE
// (no Pico/ESC available at authoring time). Verify on the bench before trusting.
#pragma once
#include <Arduino.h>
#include <stdint.h>

namespace blheli_bl {

// ---- bootloader ACK / response codes (BF-AVR) ----
static constexpr uint8_t br_SUCCESS      = 0x30;
static constexpr uint8_t br_ERRORVERIFY  = 0xC0;
static constexpr uint8_t br_ERRORCOMMAND = 0xC1;  // also = "alive" NAK to keep-alive
static constexpr uint8_t br_ERRORCRC     = 0xC2;
static constexpr uint8_t br_NONE         = 0xFF;

enum class McuType : uint8_t { UNKNOWN, SILABS_EFM8, ATMEL_AVR, ARM_BB51 };

struct DeviceInfo {
	bool        valid = false;
	uint8_t     signature[2] = {0, 0};   // [0]=sigHi [1]=sigLo; EFM8BB21 => E8 B2
	uint8_t     bootVersion  = 0;
	uint8_t     bootPages    = 0;
	uint8_t     bootInfo[8]  = {0};      // raw connect reply
	McuType     mcu  = McuType::UNKNOWN;
	const char* name = nullptr;          // resolved chip name, or nullptr
	uint16_t    signatureWord() const { return (uint16_t(signature[0]) << 8) | signature[1]; }
};

struct Config {
	uint8_t  signalPin;        // shared ESC signal wire (same pin DShot uses)
	uint32_t baud = 19200;     // SiLabs bootloader is 19200 (52 us bit time)
};

class Bootloader {
public:
	explicit Bootloader(const Config& cfg) : cfg_(cfg) {}

	// Bring up the single-wire transport; leaves the line idle (input, pulled high).
	bool begin();

	// Send BootInit and confirm the ESC is in its bootloader ("471" + signature).
	// The bootloader only listens briefly at power-up, so retry while (re)powering
	// the ESC. On success, DeviceInfo is captured (see lastDevice()).
	bool connect();

	// Return the DeviceInfo captured during connect().
	bool readDeviceInfo(DeviceInfo& out);
	const DeviceInfo& lastDevice() const { return dev_; }

	// Read `len` bytes (1..256) from flash (`readFlash`) or EEPROM (`readEeprom`).
	bool readEeprom(uint16_t addr, uint8_t* buf, uint16_t len);
	bool readFlash (uint16_t addr, uint8_t* buf, uint16_t len);

	// --- write path (Phase A1; frames correct per BF-AVR, untested) ---
	bool erasePage (uint16_t addr);                                   // page erase (512B on BB2)
	bool writeFlash (uint16_t addr, const uint8_t* buf, uint16_t len);
	bool writeEeprom(uint16_t addr, const uint8_t* buf, uint16_t len);

	bool keepAlive();          // true if bootloader NAKs 0xFD with br_ERRORCOMMAND
	bool run();                // exit bootloader → start app

	void end();
	bool connected() const { return connected_; }

private:
	Config     cfg_;
	bool       connected_ = false;
	DeviceInfo dev_;
	uint32_t   bitTimeUs_    = 52;   // 1e6 / baud
	uint32_t   bitTime34Us_  = 39;   // 3/4 bit — sample offset into start bit

	// --- 1-wire transport (bit-banged, mirrors BF-AVR suart) ---
	void setTx();                    // push-pull output, idle high
	void setRx();                    // input, pull-up
	void txByte(uint8_t b);
	bool rxByte(uint8_t& out, uint32_t timeoutMs);

	// --- framing ---
	// Send payload, appending the 2-byte CRC iff connected_ (stateful, per BF-AVR).
	void sendCmd(const uint8_t* data, uint16_t len);
	// Read a single ACK byte.
	bool getAck(uint8_t& ack, uint32_t timeoutMs = 250);
	// Read N data bytes (+CRC when connected) then the ACK; verify CRC; ack==0x30.
	bool readBuf(uint8_t* buf, uint16_t n, uint32_t timeoutMs = 250);
	// setAddress (big-endian) + a read/write command helper.
	bool setAddress(uint16_t addr);
	bool setBuffer(const uint8_t* data, uint16_t len);

	static uint16_t crcAdd(uint16_t crc, uint8_t b);   // poly 0xA001, reflected
	static uint16_t crcBuf(const uint8_t* data, uint16_t len);
};

// Resolve a device signature word to a human name (EFM8BB10x/21x/51x), or nullptr.
const char* signatureName(uint16_t sigWord);
McuType     mcuTypeFor(uint16_t sigWord);

} // namespace blheli_bl
