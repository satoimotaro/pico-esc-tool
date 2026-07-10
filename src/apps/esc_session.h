// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// esc_session — shared, transport-agnostic BLHeli-S bootloader session for the RP2040 ESC tool.
// core0 owns DShot (PIO) for bootloader entry; core1 owns the 1-wire bit-bang (blheli_bl). Both
// the USB-serial firmware (esc_host) and the Wi-Fi web firmware (esc_web) drive this same API and
// supply only their own transport (serial parser / HTTP handlers). Header-only: exactly one app
// translation unit compiles it per PlatformIO env, so the file-local statics have a single copy.
//
// A persistent SESSION holds one ESC in its bootloader (motor off) and reuses it across calls so
// config ops don't reboot the ESC each time; core1 keeps it alive. Call release() to restart it.
#pragma once
#include <Arduino.h>
#include <PIO_DShot.h>
#include <string.h>
#include "blheli_bl.h"
#include "esc_setup.h"

namespace escs {

static const uint8_t PINS[]  = { 10 };     // one signal pin per ESC (add more for multi-ESC)
static const uint8_t COUNT   = sizeof(PINS) / sizeof(PINS[0]);

struct Info {
	bool     present = false;
	uint8_t  pin = 0;
	uint16_t sig = 0;
	uint8_t  bootVer = 0, bootPages = 0;
	char     layout[17] = {0};
	char     name[17]   = {0};
	uint8_t  fwMain = 0, fwSub = 0;
};

namespace detail {
	static const uint32_t DSHOT_KBAUD = 600, PRIME_MS = 500, HIGH_HOLD_MS = 1000;
	static BidirDShotX1*          dsh = nullptr;                              // core0 only (PIO)
	static blheli_bl::Bootloader  bl({ .signalPin = PINS[0], .baud = 0 });    // core1 only

	enum class Op : uint8_t { NONE, CONNECT, READCFG, READPAGE, WRITEPAGE, ERASE, WRITEFLASH, READFLASH, RUN };
	static volatile Op       op = Op::NONE;
	static volatile bool     opDone = false, opOk = false;
	static volatile uint8_t  opPin = PINS[0];
	static volatile uint8_t  sig[2] = {0, 0};
	static volatile uint8_t  bootVer = 0, bootPages = 0;
	static uint8_t  cfg[esc_setup::kEepromLen];
	static uint8_t  page[esc_setup::kPageLen];
	static uint8_t  flBuf[256];
	static volatile uint16_t flAddr = 0, flLen = 0;
	static volatile int8_t   session = -1;

	static bool runOp(Op o, uint8_t pin) {          // core0: hand an op to core1, block till done
		opPin = pin; opOk = false; opDone = false; op = o;
		while (!opDone) delay(1);
		op = Op::NONE; return opOk;
	}
	static bool enterBootloader(uint8_t pin) {      // DShot-prime -> signal-loss -> connect (retry)
		for (int a = 0; a < 3; a++) {
			dsh = new BidirDShotX1(pin, DSHOT_KBAUD);
			uint32_t t0 = millis();
			while (millis() - t0 < PRIME_MS) { dsh->sendThrottle(0); delayMicroseconds(300); }
			delete dsh; dsh = nullptr;
			pinMode(pin, OUTPUT); digitalWrite(pin, HIGH); delay(HIGH_HOLD_MS);
			if (runOp(Op::CONNECT, pin)) return true;
		}
		return false;
	}
	static bool ensureConnected(uint8_t i) {        // reuse the session; enter only if needed
		if (session == (int8_t)i) return true;
		if (session >= 0) { runOp(Op::RUN, PINS[session]); session = -1; }
		if (enterBootloader(PINS[i])) { session = (int8_t)i; return true; }
		return false;
	}
}

// ---- transport-agnostic API (call from core0) --------------------------------------------------

// Connect ESC idx (via the session, no reboot) and fill its identity. false if it won't enter BL.
inline bool connect(uint8_t idx, Info& out) {
	using namespace detail;
	out = Info{}; if (idx >= COUNT) return false; out.pin = PINS[idx];
	if (!ensureConnected(idx)) return false;
	esc_setup::Settings s;
	bool cfgok = runOp(Op::READCFG, PINS[idx]);
	if (cfgok) esc_setup::decode(cfg, esc_setup::kEepromLen, s);
	out.present = true; out.sig = (uint16_t)((sig[0] << 8) | sig[1]);
	out.bootVer = bootVer; out.bootPages = bootPages;
	strncpy(out.layout, cfgok ? s.layoutTag : "", 16);
	strncpy(out.name,   cfgok ? s.name      : "", 16);
	out.fwMain = s.mainRevision; out.fwSub = s.subRevision;
	return true;
}
inline bool scan(uint8_t idx, Info& out) { return connect(idx, out); }

// Read the 255-byte config block into out255. Requires idx connectable.
inline bool readConfig(uint8_t idx, uint8_t* out255) {
	using namespace detail;
	if (idx >= COUNT || !ensureConnected(idx) || !runOp(Op::READCFG, PINS[idx])) return false;
	memcpy(out255, cfg, esc_setup::kEepromLen);
	return true;
}

// Apply (offset,value) overrides to the config page with a flash-wear guard (skip write if the
// bytes already match). Returns: 1 written+verified, 0 unchanged (no write), <0 error.
inline int editConfig(uint8_t idx, const uint16_t* offs, const uint8_t* vals, int n, bool& changed) {
	using namespace detail;
	changed = false;
	if (idx >= COUNT || !ensureConnected(idx)) return -1;
	if (!runOp(Op::READPAGE, PINS[idx])) return -2;
	for (int k = 0; k < n; k++) {
		if (offs[k] >= esc_setup::kPageLen) return -3;
		if (page[offs[k]] != vals[k]) { page[offs[k]] = vals[k]; changed = true; }
	}
	if (!changed) return 0;
	return runOp(Op::WRITEPAGE, PINS[idx]) ? 1 : -4;
}

// Raw flash primitives (for firmware flashing). Each requires idx connectable.
inline bool erasePage (uint8_t idx, uint16_t addr) {
	using namespace detail;
	if (idx >= COUNT || !ensureConnected(idx)) return false;
	flAddr = addr; return runOp(Op::ERASE, PINS[idx]);
}
inline bool writeFlash(uint8_t idx, uint16_t addr, const uint8_t* data, uint16_t len) {
	using namespace detail;
	if (idx >= COUNT || len > sizeof(flBuf) || !ensureConnected(idx)) return false;
	memcpy(flBuf, data, len); flAddr = addr; flLen = len; return runOp(Op::WRITEFLASH, PINS[idx]);
}
inline bool readFlash (uint8_t idx, uint16_t addr, uint8_t* out, uint16_t len) {
	using namespace detail;
	if (idx >= COUNT || len > sizeof(flBuf) || !ensureConnected(idx)) return false;
	flAddr = addr; flLen = len;
	if (!runOp(Op::READFLASH, PINS[idx])) return false;
	memcpy(out, flBuf, len); return true;
}

inline bool connected(uint8_t idx) { return detail::session == (int8_t)idx; }
inline void release() { using namespace detail; if (session >= 0) { runOp(Op::RUN, PINS[session]); session = -1; } }

// ---- core1 worker: call once from loop1() ------------------------------------------------------
inline void core1Poll() {
	using namespace detail;
	if (op == Op::NONE || opDone) {
		static uint32_t lastKa = 0;                 // keep the held ESC in its bootloader
		if (session >= 0 && bl.connected() && millis() - lastKa > 100) { bl.keepAlive(); lastKa = millis(); }
		delay(1); return;
	}
	Op o = op; bool ok = false;
	if (o == Op::CONNECT) {
		bl.setSignalPin(opPin); bl.begin();
		uint32_t t0 = millis();
		while (millis() - t0 < 500 && !ok) {
			uint8_t raw[8] = {0};
			int n = bl.connectRawProbe(raw, 15);
			if (n >= 8 && raw[0] == '4' && raw[1] == '7' && raw[2] == '1') {
				if (bl.connect()) {
					const auto& d = bl.lastDevice();
					sig[0] = d.signature[0]; sig[1] = d.signature[1];
					bootVer = d.bootVersion; bootPages = d.bootPages; ok = true;
				}
			}
		}
	} else if (o == Op::READCFG)   { esc_setup::Settings s; ok = esc_setup::read(bl, s); if (ok) memcpy(cfg, s.raw, esc_setup::kEepromLen); }
	else if (o == Op::READPAGE)    { ok = esc_setup::readPage(bl, page); }
	else if (o == Op::WRITEPAGE)   { ok = esc_setup::writePage(bl, page); }
	else if (o == Op::ERASE)       { ok = bl.erasePage(flAddr); }
	else if (o == Op::WRITEFLASH)  { ok = bl.writeFlash(flAddr, flBuf, flLen); }
	else if (o == Op::READFLASH)   { ok = bl.readFlash(flAddr, flBuf, flLen); }
	else if (o == Op::RUN)         { ok = bl.run(); }
	opOk = ok; opDone = true;
}

}  // namespace escs
