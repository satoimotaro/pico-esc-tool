// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// fourway — lets the stock configurators (esc-configurator.com for BLHeli_S/Bluejay, the AM32
// configurator) use the Pico as if it were a Betaflight flight controller: a minimal MSP responder,
// then the BLHeli 4-way interface after MSP_SET_PASSTHROUGH, translated onto escs::'s bootloader
// session. Mirrors Betaflight src/main/io/serial_4way.c (protocol 108, interface 20.0.06).
//
// Shares the CLI stream: no mode switch. A '$' at the start of a line opens an MSP frame (a CLI
// command never starts with one); after SET_PASSTHROUGH every byte is 4-way until InterfaceExit.
// The configurators' motor test (MSP_SET_MOTOR) is only recorded here; EscTool::motorTestPoll turns
// it into spins through the same arm / deadman path as the CLI, and stops it when the link goes.
#pragma once
#include <Arduino.h>
#include "console.h"
#include "esc_session.h"

class FourWay {
public:
	bool active() const { return fourWay_; }   // in passthrough (the LCD shows it)
	int8_t selected() const { return sel_; }   // escs:: index, not the configurator's channel

	// Motor test: the last MSP_SET_MOTOR value per configurator channel (1000 = stop, or 1500 when
	// 3D is reported), and when MSP was last heard. esc-configurator sends SET_MOTOR only when a
	// slider moves, but polls BATTERY_STATE every second while the motor panel is open.
	uint16_t motor(uint8_t ch) const { return ch < MAX_CH ? motor_[ch] : 0; }
	uint32_t mspMs() const { return mspMs_; }
	bool     motorWanted() const { for (uint8_t c = 0; c < MAX_CH; c++) if (wanted(c)) return true; return false; }
	uint16_t stopValue() const { return threeD_ ? 1500 : 1000; }
	void     clearMotors() { for (uint16_t& v : motor_) v = 0; declSet_ = false; }
	void     setThreeD(bool on) { threeD_ = on; }   // report FEATURE_3D (a reversible ESC is wired)
	bool     threeD() const { return threeD_; }
	// What a channel's value MEANS: 3D (1500 = stop) or one-way (1000 = stop). A stock configurator
	// gets the one FEATURE_3D answer for every channel; this firmware's web page declares it per
	// channel (`mtmode <mask>`), from the ESCs it read, until the test ends (clearMotors).
	void     declare(uint8_t mask) { declMask_ = mask; declSet_ = true; }
	bool     chThreeD(uint8_t ch) const { return declSet_ ? (declMask_ >> ch & 1) : threeD_; }
	bool     wanted(uint8_t ch) const {   // a non-stop value on that channel
		uint16_t v = motor(ch);
		return v && (chThreeD(ch) ? v != 1500 : v > 1000);
	}
	// A configurator channel -> escs:: index: the wired ESCs, numbered without gaps.
	static int8_t channel(uint8_t ch) { return ch < MAX_CH ? escs::presentIndex(ch) : -1; }

	// Feed one byte from the console. Returns true if it was consumed here; false hands it to the
	// CLI line reader. lineStart = the CLI has no partial line pending.
	bool feed(uint8_t c, bool lineStart) {
		uint32_t now = millis();
		if (n_ && now - lastMs_ > FRAME_GAP_MS) n_ = 0;          // a stalled frame is dropped
		lastMs_ = now;
		// A configurator that closed the port without InterfaceExit leaves us in 4-way. Its next
		// session opens with MSP, and a '$' can never start a 4-way frame (always 0x2F): end the
		// old session, then take the byte as MSP.
		if (fourWay_ && n_ == 0 && c == '$') { escs::fwEnd(); fourWay_ = false; sel_ = -1; }
		if (fourWay_) { fourWayByte(c); return true; }
		if (n_ == 0 && !(lineStart && c == '$')) return false;
		mspByte(c);
		return true;
	}

private:
	static const uint32_t FRAME_GAP_MS = 200;
	static const uint8_t  MAX_CH = 8;           // MSP_MOTOR carries 8; so do the configurators
	static constexpr const char* IF_NAME = "m4wFCIntf";
	static const uint8_t PROTOCOL_VER = 108, IF_VER_HI = 200, IF_VER_LO = 6;   // 20006 -> 200, 06
	enum : uint8_t { imSIL_BLB = 1, imATM_BLB = 2, imARM_BLB = 4 };
	enum : uint8_t { ACK_OK = 0x00, ACK_I_INVALID_CMD = 0x02, ACK_I_INVALID_CRC = 0x03,
	                 ACK_I_INVALID_CHANNEL = 0x08, ACK_I_INVALID_PARAM = 0x09, ACK_D_GENERAL_ERROR = 0x0F };

	uint8_t  buf_[256 + 8];
	uint16_t n_ = 0;
	uint8_t  len_ = 0;                          // payload length of the MSP frame being handled
	uint32_t lastMs_ = 0;
	bool     fourWay_ = false;
	int8_t   sel_ = -1;
	uint8_t  mode_ = imSIL_BLB;
	uint16_t motor_[8] = {0};
	uint32_t mspMs_ = 0;
	bool     threeD_ = false;
	bool     declSet_ = false;
	uint8_t  declMask_ = 0;

	// ---------------------------------------------------------------- MSP v1: $ M < len cmd data ck
	void mspByte(uint8_t c) {
		buf_[n_++] = c;
		if (n_ == 2 && c != 'M') { n_ = 0; return; }              // not MSP v1 ($X v2 isn't needed)
		if (n_ == 3 && c != '<') { n_ = 0; return; }
		if (n_ < 5) return;
		uint8_t len = buf_[3];
		if (n_ < (uint16_t)(6 + len)) return;
		uint8_t ck = 0;
		for (uint16_t i = 3; i < 5 + len; i++) ck ^= buf_[i];
		uint8_t cmd = buf_[4];
		len_ = len; n_ = 0;
		if (ck == buf_[5 + len]) mspCommand(cmd);
	}

	void mspReply(uint8_t cmd, const uint8_t* d, uint8_t len, bool error = false) {
		uint8_t h[5] = { '$', 'M', uint8_t(error ? '!' : '>'), len, cmd };
		uint8_t ck = len ^ cmd;
		for (uint8_t i = 0; i < len; i++) ck ^= d[i];
		console::out().write(h, 5);
		if (len) console::out().write(d, len);
		console::out().write(&ck, 1);
		console::out().flush();
	}

	void mspCommand(uint8_t cmd) {
		uint8_t d[8 * 13 + 1] = {0};
		mspMs_ = millis();
		uint8_t n = escs::presentCount();
		if (n > MAX_CH) n = MAX_CH;
		switch (cmd) {
			case 1:   d[0] = 0; d[1] = 1; d[2] = 46; mspReply(cmd, d, 3); break;             // API_VERSION 1.46
			case 2:   memcpy(d, "BTFL", 4); mspReply(cmd, d, 4); break;                      // FC_VARIANT
			case 3:   d[0] = 4; d[1] = 5; d[2] = 0; mspReply(cmd, d, 3); break;              // FC_VERSION
			case 4:   memcpy(d, "PICO", 4); mspReply(cmd, d, 6); break;                      // BOARD_INFO
			case 5:   memcpy(d, __DATE__, 11); memcpy(d + 11, __TIME__, 8); mspReply(cmd, d, 19); break;  // BUILD_INFO
			case 36:  if (threeD_) d[1] = 0x10;                                              // FEATURE_CONFIG: FEATURE_3D = 1<<12
			          mspReply(cmd, d, 4); break;
			case 101: d[15] = 0; d[16] = 1; d[17] = 0x04; mspReply(cmd, d, 22); break;      // STATUS: RX_FAILSAFE (radio off)
			case 104: for (uint8_t i = 0; i < 8; i++) { uint16_t v = i < n ? 1000 : 0;      // MOTOR: >0 = an ESC is there
			                                            d[2 * i] = v & 0xFF; d[2 * i + 1] = v >> 8; }
			          mspReply(cmd, d, 16); break;
			case 130: mspReply(cmd, d, 11); break;                                           // BATTERY_STATE: none
			case 131: d[6] = n; d[7] = 14; mspReply(cmd, d, 10); break;                      // MOTOR_CONFIG: count at [6]
			case 160: mspReply(cmd, d, 12); break;                                           // UID
			case 139: {                                                                      // MOTOR_TELEMETRY
				// per motor: rpm u32, invalid % u16 (x100), temp u8 C, volts u16 (x100), amps u16 (x100),
				// mAh u16 — Betaflight's layout. Bidir DShot only; a stale or absent packet reads 100 % invalid.
				d[0] = n;
				for (uint8_t c = 0; c < n; c++) {
					uint8_t* q = d + 1 + 13 * c;
					escs::Telem t; int8_t e = channel(c);          // Telem zero-initialises: no ESC = zeros
					bool fresh = e >= 0 && escs::spinTele(e, t) && t.valid && millis() - t.rpmStampMs < 500;
					uint32_t rpm = fresh ? t.rpm : 0;
					uint16_t bad = fresh ? 0 : 10000, v = uint16_t(t.voltage * 100), a = uint16_t(t.current * 100);
					q[0] = rpm; q[1] = rpm >> 8; q[2] = rpm >> 16; q[3] = rpm >> 24;
					q[4] = bad; q[5] = bad >> 8; q[6] = uint8_t(t.tempC);
					q[7] = v; q[8] = v >> 8; q[9] = a; q[10] = a >> 8;
					// mAh (unused here) carries what the web tool's motor test can do with this ESC now,
					// 0x5A marks it: 2 arming, 3 armed (moves at once), 4 not streaming (a move arms it,
					// ~3 s), 5 not wired / its read failed.
					uint8_t ui = 5;
					if (e >= 0) ui = escs::spinArmed(e) ? 3 : escs::spinArming(e) ? 2
					               : (escs::isPresent(e) && !escs::identFailed(e)) ? 4 : 5;
					q[11] = ui; q[12] = 0x5A;
				}
				mspReply(cmd, d, 1 + 13 * n); break;
			}
			case 214:                                                                        // SET_MOTOR: 8 x u16, 1000..2000
				for (uint8_t i = 0; i < MAX_CH && 2 * i + 1 < len_; i++) motor_[i] = buf_[5 + 2 * i] | buf_[6 + 2 * i] << 8;
				mspReply(cmd, d, 0); break;
			case 217: mspReply(cmd, d, 0); break;                                            // SET_3D: ignored
			case 245:                                                                        // SET_PASSTHROUGH -> 4-way
				clearMotors();                    // EscTool stops a motor test when it sees passthrough
				d[0] = n; mspReply(cmd, d, 1);   // answer first: fwBegin primes the ESCs for 0.5 s
				escs::fwBegin();
				fourWay_ = true; sel_ = -1; mode_ = imSIL_BLB; n_ = 0;
				break;
			default:  mspReply(cmd, d, 0, true); break;
		}
	}

	// ---------------------------------------------------------------- 4-way: 2F cmd aH aL len p.. crcH crcL
	static uint16_t crcXmodem(uint16_t crc, uint8_t b) {
		crc ^= (uint16_t)b << 8;
		for (uint8_t i = 0; i < 8; i++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
		return crc;
	}

	void fourWayByte(uint8_t c) {
		if (n_ == 0 && c != 0x2F) return;                         // wait for the local escape
		buf_[n_++] = c;
		if (n_ < 5) return;
		uint16_t plen = buf_[4] ? buf_[4] : 256;
		if (n_ < 5 + plen + 2) return;
		uint16_t crc = 0;
		for (uint16_t i = 0; i < 5 + plen; i++) crc = crcXmodem(crc, buf_[i]);
		uint16_t got = ((uint16_t)buf_[5 + plen] << 8) | buf_[6 + plen];
		n_ = 0;
		fourWayCommand(buf_[1], buf_[2], buf_[3], buf_ + 5, plen, crc == got);
	}

	void fourWayCommand(uint8_t cmd, uint8_t aH, uint8_t aL, uint8_t* p, uint16_t plen, bool crcOk) {
		static uint8_t out[256];
		uint16_t olen = 1; out[0] = 0;
		uint8_t ack = crcOk ? ACK_OK : ACK_I_INVALID_CRC;
		bool exit = false;
		uint16_t addr = ((uint16_t)aH << 8) | aL;
		if (ack == ACK_OK) switch (cmd) {
			case 0x30: if (sel_ >= 0 && !escs::connected(sel_)) ack = ACK_D_GENERAL_ERROR; break;   // TestAlive
			case 0x31: out[0] = PROTOCOL_VER; break;                                               // ProtocolGetVersion
			case 0x32: olen = strlen(IF_NAME); memcpy(out, IF_NAME, olen); break;                  // InterfaceGetName
			case 0x33: olen = 2; out[0] = IF_VER_HI; out[1] = IF_VER_LO; break;                    // InterfaceGetVersion
			case 0x34: exit = true; break;                                                         // InterfaceExit
			case 0x3F: if (p[0] == imSIL_BLB || p[0] == imATM_BLB || p[0] == imARM_BLB) mode_ = p[0];   // SetMode
			           else ack = ACK_I_INVALID_PARAM; break;
			case 0x35: {                                                                           // DeviceReset
				int8_t e = channel(p[0]);
				if (e < 0) { ack = ACK_I_INVALID_CHANNEL; break; }
				escs::fwRun(e);
				if (aL == 1) { pinMode(escs::PINS[e], OUTPUT); digitalWrite(escs::PINS[e], LOW);
				               delay(300); digitalWrite(escs::PINS[e], HIGH); }
				if (sel_ == e) sel_ = -1;
				break;
			}
			case 0x37: {                                                                           // DeviceInitFlash
				int8_t e = channel(p[0]);
				if (e < 0) { ack = ACK_I_INVALID_CHANNEL; break; }
				uint8_t raw[8]; bool am32 = false;
				sel_ = -1; olen = 4; memset(out, 0, 4);
				if (!escs::fwSelect(e, raw, am32)) { ack = ACK_D_GENERAL_ERROR; break; }
				sel_ = e;
				// DeviceInfo as Betaflight's BL_ConnectEx packs it: [sigLo, sigHi, 4th id char, mode]
				out[0] = raw[5]; out[1] = raw[4]; out[2] = raw[3];
				uint16_t w = ((uint16_t)out[1] << 8) | out[0];
				if (w > 0xE800 && w < 0xF900)                         mode_ = imSIL_BLB;
				else if (out[0] == 0x06 && out[1] > 0 && out[1] < 0x90) mode_ = imARM_BLB;
				else                                                  mode_ = imATM_BLB;
				out[3] = mode_;
				break;
			}
			case 0x39: {                                                                           // DevicePageErase
				if (sel_ < 0) { ack = ACK_D_GENERAL_ERROR; break; }
				uint16_t a = (uint16_t)(mode_ == imARM_BLB ? p[0] << 2 : p[0] << 1) << 8;
				if (!escs::erasePage(sel_, a)) ack = ACK_D_GENERAL_ERROR;
				break;
			}
			case 0x3A: {                                                                           // DeviceRead
				uint16_t want = p[0] ? p[0] : 256;
				if (sel_ < 0 || mode_ == imATM_BLB || !escs::readFlash(sel_, addr, out, want)) { ack = ACK_D_GENERAL_ERROR; break; }
				olen = want;
				break;
			}
			case 0x3B:                                                                             // DeviceWrite
				if (sel_ < 0 || !escs::writeFlash(sel_, addr, p, plen)) ack = ACK_D_GENERAL_ERROR;
				break;
			default: ack = ACK_I_INVALID_CMD; break;   // EEPROM (Atmel only), EraseAll, C2, Verify
		}
		uint8_t h[5] = { 0x2E, cmd, aH, aL, uint8_t(olen & 0xFF) };
		uint16_t crc = 0;
		for (uint8_t b : h) crc = crcXmodem(crc, b);
		for (uint16_t i = 0; i < olen; i++) crc = crcXmodem(crc, out[i]);
		crc = crcXmodem(crc, ack);
		uint8_t tail[3] = { ack, uint8_t(crc >> 8), uint8_t(crc & 0xFF) };
		console::out().write(h, 5);
		console::out().write(out, olen);
		console::out().write(tail, 3);
		console::out().flush();
		if (exit) { escs::fwEnd(); fourWay_ = false; sel_ = -1; }
	}
};
