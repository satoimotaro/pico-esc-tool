// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// blheli_bl — BLHeli-S / SiLabs EFM8BB 1-wire bootloader client for RP2040.
//
// Speaks the ESC signal-wire bootloader protocol DIRECTLY (no flight controller):
// connect, read device signature, read/write flash + EEPROM, erase, run.
// esc_flash (firmware programming) and esc_setup (config params) build on this.
//
// Protocol reference: Betaflight serial_4way_avrootloader.c (SiLabs "BLB"), cross-
// checked with esc-configurator + BLHeli_S source. Details in PROTOCOL.md.
// Physical layer: 19200 8N1, non-inverted, half-duplex single wire (idle HIGH).
//
// STATUS: proven on hardware (EFM8BB21) — connect, read/write config, erase, program flash.
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

	// Retarget the transport to a different signal pin (for multi-ESC hosts). Call before begin().
	// Also clears the connected state (a new pin means a new device).
	void setSignalPin(uint8_t p) { cfg_.signalPin = p; connected_ = false; }
	uint8_t signalPin() const { return cfg_.signalPin; }

	// Bring up the single-wire transport; leaves the line idle (input, pulled high).
	bool begin();

	// Send BootInit and confirm the ESC is in its bootloader ("471" + signature).
	// The bootloader only listens briefly at power-up, so retry while (re)powering
	// the ESC. On success, DeviceInfo is captured (see lastDevice()).
	bool connect();

	// Return the DeviceInfo captured during connect().
	bool readDeviceInfo(DeviceInfo& out);
	const DeviceInfo& lastDevice() const { return dev_; }

	// --- bring-up diagnostic (does NOT change connected_) ---
	// Send BootInit once and read up to 8 reply bytes into out[8], using a short
	// per-byte timeout so the caller can hammer this in a tight loop to catch the
	// SiLabs power-up window. Returns the number of bytes actually received (0..8).
	// A valid bootloader reply starts with '4','7','1'; out[4]/out[5] = signature.
	int connectRawProbe(uint8_t out[8], uint32_t perByteTimeoutMs = 40);

	// Send the 17-byte BootInit hello WITHOUT reading a reply (leaves line in RX).
	void sendBootInit();

	// Diagnostic (turns the Pico into a logic analyzer): send BootInit, then sample the
	// signal line for `ms` and report raw activity — decoupled from UART framing. Fills
	// fallingEdges (high->low transitions) and lowSamples (# of samples read as 0). If the
	// ESC is in its bootloader and replies, fallingEdges > 0; if it's genuinely silent
	// (never entered the BL) both stay 0. Distinguishes "not in BL" from "RX mis-decodes".
	void probeReplyActivity(uint32_t ms, uint32_t& fallingEdges, uint32_t& lowSamples, uint32_t& totalSamples);

	// Drive the signal line HIGH (idle) for `ms` milliseconds, transmitting nothing.
	// CRITICAL for bootloader entry: BLHeli_S/BlueJay only jumps to its bootloader if,
	// at power-up, the input pin stays continuously HIGH for ~15 ms (init_no_signal in
	// BLHeli_S.asm). Any LOW in that window makes it run the app instead. So hold the
	// line high across the ESC power-up, THEN send BootInit.
	void holdIdleHigh(uint32_t ms);

	// Read `len` bytes (1..256) from flash (`readFlash`) or EEPROM (`readEeprom`).
	bool readEeprom(uint16_t addr, uint8_t* buf, uint16_t len);
	bool readFlash (uint16_t addr, uint8_t* buf, uint16_t len);

	// --- write path (Phase A1; frames correct per BF-AVR, untested) ---
	bool erasePage (uint16_t addr);                                   // page erase (512B on BB2)
	bool writeFlash (uint16_t addr, const uint8_t* buf, uint16_t len);
	bool writeEeprom(uint16_t addr, const uint8_t* buf, uint16_t len);

	bool keepAlive();          // true if bootloader NAKs 0xFD with br_ERRORCOMMAND
	// Diagnostic: send FD 00 (+CRC) and capture the raw reply byte. Returns 1 if a byte
	// was received (value in rawAck), 0 if rxByte timed out (no reply). Lets the caller
	// see whether failures are "no byte" (timing) vs "wrong value" (misframe/CRC-NAK 0xC2).
	int keepAliveRaw(uint8_t& rawAck, uint32_t timeoutMs = 250);
	// Diagnostic: send FD 00 (+CRC), then sample the raw line for `us` microseconds
	// (NO UART decode). Reports falling edges seen, and the timer offset (us) of the FIRST
	// falling edge after the command was sent (0 if none). Shows whether/when the ESC drives.
	void keepAliveCapture(uint32_t us, uint32_t& edges, uint32_t& firstEdgeUs, uint32_t& lowSamples);
	// Diagnostic: transmit FD 00, appending the CRC iff `withCrc` (OVERRIDES connected_), then
	// decode ONE reply byte — waiting up to timeoutMs for the start bit (interrupts on during the
	// wait, so it's USB-safe on core1) and sampling with the timer. Reports the ACK value, whether
	// a byte arrived (got), and firstEdgeUs = µs from end-of-TX to the reply's start bit. THE
	// decisive test: does this bootloader answer CRC-framed frames, bare frames, or neither?
	void keepAliveProbe(bool withCrc, uint32_t timeoutMs, uint8_t& ack, int& got, uint32_t& firstEdgeUs);
	// Diagnostic: send ONE arbitrary command frame (CRC appended iff withCrc, OVERRIDES
	// connected_) after an optional pre-gap (ms) of idle-high line — so the ESC's frame parser
	// times out any partial frame and starts clean, isolating CRC-value correctness from
	// back-to-back desync — then decode one reply byte (start-bit wait up to timeoutMs, IRQs on).
	// Reports ack, got (1=byte received), firstEdgeUs (µs from end-of-TX to the reply start bit).
	void cmdProbe(const uint8_t* data, uint16_t len, bool withCrc, uint32_t preGapMs,
	              uint32_t timeoutMs, uint8_t& ack, int& got, uint32_t& firstEdgeUs);
	// Ground-truth diagnostic for the READ path. SET_ADDRESS(addr)+crc, then rcmd(want)+crc,
	// then RAW-capture the line for 15ms (NO decode) to see whether the ESC actually streams
	// data for this read command. Reports saAck (want 0x30), falling edges seen, firstEdgeUs
	// (µs to the device's first low), lowSamples. edges>0 => device streams (RX timing is the
	// bug); edges==0 => device silent to this read command (wrong command / addr).
	void probeRead(uint16_t addr, uint8_t rcmd, uint16_t want, uint8_t& saAck,
	               uint32_t& edges, uint32_t& firstEdgeUs, uint32_t& lowSamples);
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
	void txByte(uint8_t b, bool releaseForRx = false);
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
