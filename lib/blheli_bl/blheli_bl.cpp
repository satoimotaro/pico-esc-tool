// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// blheli_bl implementation — SiLabs EFM8BB 1-wire bootloader ("BLB").
// Wire format implemented with reference to Betaflight serial_4way_avrootloader.c
// (author 4712 / H. Reddmann) and esc-configurator + BLHeli_S. See PROTOCOL.md for every value.
//
// Proven on hardware (EFM8BB21): connect, read, write, and firmware flash.
#include "blheli_bl.h"
#include <string.h>
#include <hardware/timer.h>          // timer_hw: 1 MHz free-running counter
#include <hardware/structs/sio.h>    // sio_hw: single-cycle GPIO set/clr/in/oe
#include <hardware/gpio.h>           // gpio_set_function / gpio_pull_up (pad setup, once)

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
// Fast, glitch-free half-duplex turnaround via the SIO output-enable register (single write
// each). The pad (SIO function + pull-up) is configured ONCE in begin(); here we only flip the
// driver on/off. This replaces pinMode()/digitalWrite() whose many register ops + reconfig were
// too slow, so a fast ESC ACK's start bit was missed (~1/8 keepAlive success). idle = pull-up HIGH.
void Bootloader::setTx() {
	const uint32_t mask = 1u << cfg_.signalPin;
	sio_hw->gpio_set  = mask;          // preset output latch HIGH (idle) so enabling OE = clean high
	sio_hw->gpio_oe_set = mask;        // drive the line (push-pull output)
}
void Bootloader::setRx() {
	const uint32_t mask = 1u << cfg_.signalPin;
	sio_hw->gpio_oe_clr = mask;        // release the line; pull-up (set in begin) holds it HIGH
}

// Precise software UART. Edges are scheduled at ABSOLUTE offsets from a single
// timer anchor (timer_hw->timerawl, 1 MHz), so digitalWrite overhead and per-bit
// rounding do NOT accumulate — every edge lands at i*1e6/baud µs, exactly. Levels
// via single-cycle SIO registers. MUST run on core1 (masks IRQs); see spike notes.
void Bootloader::txByte(uint8_t b, bool releaseForRx) {
	const uint32_t mask = 1u << cfg_.signalPin;
	const uint32_t baud = cfg_.baud;
	noInterrupts();
	uint32_t t0 = timer_hw->timerawl;
	for (uint32_t i = 0; i < 9; i++) {                 // start + 8 data (LSB first)
		bool level = (i == 0) ? false : ((b >> (i - 1)) & 1u);
		if (level) sio_hw->gpio_set = mask;
		else       sio_hw->gpio_clr = mask;
		uint32_t target = ((i + 1) * 1000000UL) / baud;
		while ((uint32_t)(timer_hw->timerawl - t0) < target) { /* busy-wait */ }
	}
	// stop bit (bit 9): drive the line HIGH for a FULL bit-time. This must be a clean,
	// complete stop bit even on the LAST byte: the SiLabs bootloader only treats the frame
	// as complete once it sees a valid stop bit, and it replies MILLISECONDS later (raw-line
	// capture: edges=0 for >3ms after the command, yet the NAK decodes within 250ms). So an
	// early release (skipping this busy-wait) buys nothing — the reply isn't fast — and risks
	// handing the ESC a truncated final byte, making it re-sync and answer late/inconsistently
	// (that was the ~5/8 keepAlive "--" misses). Full stop bit first, THEN release to RX.
	sio_hw->gpio_set = mask;                           // drive stop bit high (still TX)
	uint32_t stopEnd = (10 * 1000000UL) / baud;
	while ((uint32_t)(timer_hw->timerawl - t0) < stopEnd) { /* busy-wait */ }
	if (releaseForRx) sio_hw->gpio_oe_clr = mask;      // release; pull-up holds it high (== RX)
	interrupts();
}

bool Bootloader::rxByte(uint8_t& out, uint32_t timeoutMs) {
	const uint32_t pin  = cfg_.signalPin;
	const uint32_t baud = cfg_.baud;
	uint32_t tms = millis();
	while ((sio_hw->gpio_in >> pin) & 1u) {            // wait for start-bit falling edge
		if (millis() - tms > timeoutMs) return false;
	}
	noInterrupts();
	uint32_t t0 = timer_hw->timerawl;                 // ~falling edge of start bit
	uint16_t val = 0;
	for (uint32_t i = 0; i < 8; i++) {                 // sample each data bit at its CENTER
		uint32_t target = ((2 * i + 3) * 500000UL) / baud;   // (i + 1.5) * 1e6/baud
		while ((uint32_t)(timer_hw->timerawl - t0) < target) { /* busy-wait */ }
		if ((sio_hw->gpio_in >> pin) & 1u) val |= (1u << i);
	}
	uint32_t stopTgt = (19 * 500000UL) / baud;         // 9.5 * 1e6/baud
	while ((uint32_t)(timer_hw->timerawl - t0) < stopTgt) { /* busy-wait */ }
	bool stop = (sio_hw->gpio_in >> pin) & 1u;
	interrupts();
	if (!stop) return false;                           // stop bit must be 1 (framing)
	out = (uint8_t)val;
	return true;
}

// ---- framing ----
// Turnaround gap (ms) before a command once connected: after the ESC transmits a reply it needs
// time to switch back to RX before it will hear our next command. Our OE-register turnaround is
// so fast that the read command sent right after the SET_ADDRESS ack was missed entirely (ESC
// stayed silent) — proven on HW: adding this gap makes the ESC stream read data. BootInit/connect
// (not yet connected_) skip it, so connect stays fast.
static constexpr uint32_t kCmdGapMs = 5;

void Bootloader::sendCmd(const uint8_t* data, uint16_t len) {
	if (connected_) delay(kCmdGapMs);          // let the ESC finish its TX->RX turnaround
	setTx();
	uint16_t c = 0;
	const bool crc = connected_;               // CRC appended => last byte is the CRC hi byte
	for (uint16_t i = 0; i < len; i++) {
		bool last = !crc && (i == len - 1);
		txByte(data[i], last);                 // last data byte releases to RX iff no CRC
		c = crcAdd(c, data[i]);
	}
	if (crc) {
		txByte(c & 0xFF, false);
		txByte((c >> 8) & 0xFF, true);         // CRC hi = final byte -> release to RX
	}
	setRx();                                    // idempotent: OE already cleared by last byte
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
		// Per BF-AVR BL_ReadBuf: a read response is [data][CRC-lo][CRC-hi][ACK] — CRC then a
		// trailing ACK byte (0x30 on success). Read all three.
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
	// SET_BUFFER header: {FE, 0, hi, lo}. len==256 sends {FE,0,1,0}; else {FE,0,0,len}.
	uint8_t cmd[4] = { CMD_SET_BUFFER, 0, uint8_t(len >> 8), uint8_t(len & 0xFF) };
	sendCmd(cmd, 4);
	// Per BF-AVR BL_SendCMDSetBuffer: the device does NOT ack the header — it waits for the
	// buffer bytes (BF asserts the ACK slot is brNONE; a byte here is an error). So expect
	// SILENCE, then stream the buffer (with CRC when connected), then read the real SUCCESS ack.
	uint8_t stray;
	if (rxByte(stray, 5)) return false;            // unexpected byte after header => abort
	sendCmd(data, len);
	uint8_t ack;
	return getAck(ack) && ack == br_SUCCESS;
}

// ---- public ----
bool Bootloader::begin() {
	if (cfg_.baud == 0) cfg_.baud = 19200;
	bitTimeUs_   = 1000000UL / cfg_.baud;          // ~52 us @ 19200
	bitTime34Us_ = (bitTimeUs_ * 3) / 4;           // ~39 us
	// Configure the pad ONCE so setTx/setRx only flip the output-enable bit afterwards.
	gpio_set_function(cfg_.signalPin, GPIO_FUNC_SIO);
	gpio_pull_up(cfg_.signalPin);                   // ~50k pull-up holds the 1-wire idle HIGH
	sio_hw->gpio_set = 1u << cfg_.signalPin;         // output latch HIGH (used when we enable OE)
	setRx();                                        // idle: released, pulled high
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

void Bootloader::sendBootInit() {
	connected_ = false;                 // BootInit carries no CRC
	sendCmd(BOOT_INIT, sizeof(BOOT_INIT));
}

void Bootloader::probeReplyActivity(uint32_t ms, uint32_t& fallingEdges,
                                    uint32_t& lowSamples, uint32_t& totalSamples) {
	const uint32_t pin = cfg_.signalPin;
	connected_ = false;
	sendCmd(BOOT_INIT, sizeof(BOOT_INIT));   // ends in setRx() (input, pulled high)
	fallingEdges = lowSamples = totalSamples = 0;
	uint32_t tms = millis();
	uint32_t prev = (sio_hw->gpio_in >> pin) & 1u;
	while (millis() - tms < ms) {
		uint32_t cur = (sio_hw->gpio_in >> pin) & 1u;
		if (prev == 1u && cur == 0u) fallingEdges++;
		if (cur == 0u) lowSamples++;
		prev = cur;
		totalSamples++;
	}
}

void Bootloader::holdIdleHigh(uint32_t ms) {
	setTx();                            // push-pull output, driven HIGH
	uint32_t t0 = millis();
	while (millis() - t0 < ms) { /* keep the line solidly high, transmit nothing */ }
}

int Bootloader::connectRawProbe(uint8_t out[8], uint32_t perByteTimeoutMs) {
	// BootInit is sent WITHOUT CRC (connected_ stays false).
	connected_ = false;
	sendCmd(BOOT_INIT, sizeof(BOOT_INIT));
	int n = 0;
	for (int i = 0; i < 8; i++) {
		if (!rxByte(out[i], perByteTimeoutMs)) break;
		n++;
	}
	return n;
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

int Bootloader::keepAliveRaw(uint8_t& rawAck, uint32_t timeoutMs) {
	if (!connected_) return 0;
	uint8_t cmd[2] = { CMD_KEEP_ALIVE, 0x00 };
	sendCmd(cmd, 2);                        // FD 00 (+CRC, since connected_)
	return rxByte(rawAck, timeoutMs) ? 1 : 0;
}

void Bootloader::keepAliveCapture(uint32_t us, uint32_t& edges,
                                  uint32_t& firstEdgeUs, uint32_t& lowSamples) {
	const uint32_t pin = cfg_.signalPin;
	uint8_t cmd[2] = { CMD_KEEP_ALIVE, 0x00 };
	sendCmd(cmd, 2);                        // ends in setRx() (input, pulled high)
	edges = firstEdgeUs = lowSamples = 0;
	uint32_t t0 = timer_hw->timerawl;
	uint32_t prev = (sio_hw->gpio_in >> pin) & 1u;
	for (;;) {
		uint32_t now = timer_hw->timerawl;
		if ((uint32_t)(now - t0) >= us) break;
		uint32_t cur = (sio_hw->gpio_in >> pin) & 1u;
		if (prev == 1u && cur == 0u) {
			edges++;
			if (firstEdgeUs == 0) firstEdgeUs = (uint32_t)(now - t0);
		}
		if (cur == 0u) lowSamples++;
		prev = cur;
	}
}

void Bootloader::keepAliveProbe(bool withCrc, uint32_t timeoutMs, uint8_t& ack,
                                int& got, uint32_t& firstEdgeUs) {
	ack = 0xFF; got = 0; firstEdgeUs = 0;
	const uint32_t pin  = cfg_.signalPin;
	const uint32_t baud = cfg_.baud;
	uint8_t cmd[2] = { CMD_KEEP_ALIVE, 0x00 };
	// --- transmit, appending CRC only if withCrc (independent of connected_) ---
	setTx();
	if (withCrc) {
		uint16_t c = crcBuf(cmd, 2);
		txByte(cmd[0], false);
		txByte(cmd[1], false);
		txByte(c & 0xFF, false);
		txByte((c >> 8) & 0xFF, true);        // full stop bit, then release to RX
	} else {
		txByte(cmd[0], false);
		txByte(cmd[1], true);                 // full stop bit, then release to RX
	}
	setRx();
	// --- wait for the reply's start bit (interrupts ON so long waits stay USB-safe on core1) ---
	uint32_t tms   = millis();
	uint32_t tAnch = timer_hw->timerawl;      // ~end of our transmission
	while ((sio_hw->gpio_in >> pin) & 1u) {
		if (millis() - tms > timeoutMs) return;   // no reply within timeout
	}
	noInterrupts();
	uint32_t t0 = timer_hw->timerawl;         // reply start-bit falling edge
	firstEdgeUs = (uint32_t)(t0 - tAnch);
	uint16_t val = 0;
	for (uint32_t i = 0; i < 8; i++) {        // sample each data bit at its center
		uint32_t target = ((2 * i + 3) * 500000UL) / baud;
		while ((uint32_t)(timer_hw->timerawl - t0) < target) { /* busy-wait */ }
		if ((sio_hw->gpio_in >> pin) & 1u) val |= (1u << i);
	}
	uint32_t stopTgt = (19 * 500000UL) / baud;
	while ((uint32_t)(timer_hw->timerawl - t0) < stopTgt) { /* busy-wait */ }
	bool stop = (sio_hw->gpio_in >> pin) & 1u;
	interrupts();
	if (stop) { ack = (uint8_t)val; got = 1; }   // valid framing => report the byte
}

void Bootloader::cmdProbe(const uint8_t* data, uint16_t len, bool withCrc, uint32_t preGapMs,
                          uint32_t timeoutMs, uint8_t& ack, int& got, uint32_t& firstEdgeUs) {
	ack = 0xFF; got = 0; firstEdgeUs = 0;
	const uint32_t pin  = cfg_.signalPin;
	const uint32_t baud = cfg_.baud;
	if (preGapMs) { setRx(); delay(preGapMs); }   // idle-high gap: let the ESC parser reset
	// --- transmit the frame, appending CRC only if withCrc (independent of connected_) ---
	setTx();
	uint16_t c = 0;
	for (uint16_t i = 0; i < len; i++) {
		bool last = !withCrc && (i == len - 1);
		txByte(data[i], last);                     // last payload byte releases to RX iff no CRC
		c = crcAdd(c, data[i]);
	}
	if (withCrc) {
		txByte(c & 0xFF, false);
		txByte((c >> 8) & 0xFF, true);             // CRC hi = final byte -> full stop bit + release
	}
	setRx();
	// --- wait for the reply's start bit (interrupts ON so long waits stay USB-safe on core1) ---
	uint32_t tms   = millis();
	uint32_t tAnch = timer_hw->timerawl;
	while ((sio_hw->gpio_in >> pin) & 1u) {
		if (millis() - tms > timeoutMs) return;
	}
	noInterrupts();
	uint32_t t0 = timer_hw->timerawl;
	firstEdgeUs = (uint32_t)(t0 - tAnch);
	uint16_t val = 0;
	for (uint32_t i = 0; i < 8; i++) {
		uint32_t target = ((2 * i + 3) * 500000UL) / baud;
		while ((uint32_t)(timer_hw->timerawl - t0) < target) { /* busy-wait */ }
		if ((sio_hw->gpio_in >> pin) & 1u) val |= (1u << i);
	}
	uint32_t stopTgt = (19 * 500000UL) / baud;
	while ((uint32_t)(timer_hw->timerawl - t0) < stopTgt) { /* busy-wait */ }
	bool stop = (sio_hw->gpio_in >> pin) & 1u;
	interrupts();
	if (stop) { ack = (uint8_t)val; got = 1; }
}

void Bootloader::probeRead(uint16_t addr, uint8_t rcmd, uint16_t want, uint8_t& saAck,
                           uint32_t& edges, uint32_t& firstEdgeUs, uint32_t& lowSamples) {
	saAck = 0xFF; edges = firstEdgeUs = lowSamples = 0;
	const uint32_t pin = cfg_.signalPin;
	// 1) SET_ADDRESS(addr) +crc — proven to reply 0x30
	uint8_t sa[4] = { CMD_SET_ADDRESS, 0x00, uint8_t(addr >> 8), uint8_t(addr & 0xFF) };
	{ int got = 0; uint32_t fe = 0; cmdProbe(sa, 4, true, 20, 250, saAck, got, fe);
	  if (!got || saAck != br_SUCCESS) return; }
	// 2) READ command +crc — with a gap AFTER the SET_ADDRESS ack so the ESC has finished its
	// TX->RX turnaround (the untested case: readFlash sends this with no gap). Then RAW-capture.
	setRx(); delay(5);
	setTx();
	uint16_t c = 0; uint8_t rc[2] = { rcmd, uint8_t(want & 0xFF) };
	txByte(rc[0], false); c = crcAdd(c, rc[0]);
	txByte(rc[1], false); c = crcAdd(c, rc[1]);
	txByte(c & 0xFF, false);
	txByte((c >> 8) & 0xFF, true);
	setRx();
	uint32_t t0 = timer_hw->timerawl;
	uint32_t prev = (sio_hw->gpio_in >> pin) & 1u;
	const uint32_t win = 15000;              // 15ms window (a 112B burst at 19200 is ~60ms; this
	                                         // just confirms whether the device starts driving)
	for (;;) {
		uint32_t now = timer_hw->timerawl;
		if ((uint32_t)(now - t0) >= win) break;
		uint32_t cur = (sio_hw->gpio_in >> pin) & 1u;
		if (prev == 1u && cur == 0u) { edges++; if (firstEdgeUs == 0) firstEdgeUs = (uint32_t)(now - t0); }
		if (cur == 0u) lowSamples++;
		prev = cur;
	}
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
