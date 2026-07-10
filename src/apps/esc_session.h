// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// esc_session — shared, transport-agnostic BLHeli-S bootloader session + DShot drive for the RP2040
// ESC tool. core0 owns DShot (PIO) for bootloader entry and thruster drive; core1 owns the 1-wire
// bit-bang (blheli_bl). The unified `esc_tool` firmware drives this same API from both its transports
// (USB-serial parser and Wi-Fi HTTP handlers). Header-only: exactly one app translation unit compiles
// it per PlatformIO env, so the file-local statics have a single copy.
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

// Live DShot telemetry (Extended DShot Telemetry) for a spinning thruster.
struct Telem {
	bool     valid = false;
	uint32_t rpm = 0, current = 0, tempC = 0, stress = 0, status = 0;
	float    voltage = 0.0f;
};

static const uint16_t SPIN_MAX        = 2000;   // max throttle
static const uint32_t SPIN_DEADMAN_MS = 500;    // auto-zero throttle if no command within this
static const uint32_t SPIN_ARM_MS     = 3000;   // stream zero throttle this long to ARM the ESC
                                                // (covers the ESC's boot beep after leaving the BL)
static const uint16_t MOTOR_POLES     = 14;     // magnet poles (for eRPM->RPM); motor-dependent

// Per-ESC DShot mode. BLHeli-S understands only *normal* DShot; *bidir* DShot (inverted, with eRPM/
// EDT telemetry back over the wire) needs firmware that supports it (Bluejay/JESC). AUTO picks bidir
// if the firmware name says so, else normal — so any stock BLHeli-S ESC still spins.
enum class Drive : uint8_t { AUTO, NORMAL, BIDIR };

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
	// drive/spin: one DShot driver per pin (core0). Only one of drvB/drvN is non-null per pin.
	static BidirDShotX1*     drvB[COUNT] = { nullptr };   // bidir DShot: idle HIGH, eRPM/EDT telemetry
	static DShotX4*          drvN[COUNT] = { nullptr };   // normal DShot: idle LOW, throttle only
	static volatile uint16_t drvTarget[COUNT] = { 0 };
	static uint32_t          drvLast[COUNT] = { 0 };
	static bool              drvArmed[COUNT] = { false };
	static uint32_t          drvArmStart[COUNT] = { 0 };
	static uint8_t           drvEdt[COUNT] = { 0 };       // frames of EDT-enable left to send (bidir arm)
	static bool              drvRev[COUNT] = { false };   // ESC configured for reversible (3D) rotation
	static Telem             drvTele[COUNT];
	// last-known firmware/direction, cached by connect()/scan() so spinArm() needn't re-enter the BL
	static bool              infoBidir[COUNT] = { false };  // firmware supports bidir DShot (Bluejay/JESC)
	static bool              infoRev[COUNT]   = { false };  // configured reversible (3D)
	static bool              infoKnown[COUNT] = { false };

	static inline bool drvActive(uint8_t i) { return drvB[i] || drvN[i]; }
	static inline void drvSend(uint8_t i, uint16_t t) {
		if (drvB[i]) drvB[i]->sendThrottle(t);
		else if (drvN[i]) { uint16_t a[4] = { t, 0, 0, 0 }; drvN[i]->sendThrottles(a); }
	}
	static inline void drvFree(uint8_t i) {
		if (drvB[i]) { drvB[i]->sendThrottle(0); delete drvB[i]; drvB[i] = nullptr; }
		if (drvN[i]) { uint16_t a[4] = { 0, 0, 0, 0 }; drvN[i]->sendThrottles(a); delete drvN[i]; drvN[i] = nullptr; }
	}

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
		if (drvActive(i)) drvFree(i);                                             // free pin from drive
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
	if (cfgok) {
		esc_setup::decode(cfg, esc_setup::kEepromLen, s);
		infoBidir[idx] = strstr(s.name, "Bluejay") || strstr(s.name, "JESC");   // cache for spinArm
		infoRev[idx]   = (cfg[0x0B] == 3 || cfg[0x0B] == 4);
		infoKnown[idx] = true;
	}
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

// ---- drive / spin (core0) ----------------------------------------------------------------------
// spinArm() leaves any bootloader session (the ESC must run its app), picks the DShot mode, and
// starts the arming stream for SPIN_ARM_MS. Then spinThrottle()/spinThrust() set the target;
// spinStop()/spinDisarm() release it. Mode AUTO reads the ESC's firmware name: Bluejay/JESC -> bidir
// DShot (idle HIGH, eRPM + EDT telemetry); otherwise normal DShot (idle LOW), which any BLHeli-S
// spins. Reversible (3D) rotation is read from the config so throttle can be a signed thrust.
// Call spinPoll() every core0 loop to keep frames flowing, run arming + deadman, and read telemetry.
inline void spinArm(uint8_t idx, Drive mode = Drive::AUTO) {
	using namespace detail;
	if (idx >= COUNT) return;
	// Resolve firmware/direction from the cache (populated by scan/connect). Only re-read config when
	// unknown, or when already connected to this ESC (cheap — no bootloader re-entry). This keeps a
	// post-scan arm fast; a cold arm (never scanned) pays one bootloader entry to detect.
	if (!infoKnown[idx] || session == (int8_t)idx) {
		if (ensureConnected(idx) && runOp(Op::READCFG, PINS[idx])) {
			esc_setup::Settings s; esc_setup::decode(cfg, esc_setup::kEepromLen, s);
			infoBidir[idx] = strstr(s.name, "Bluejay") || strstr(s.name, "JESC");
			infoRev[idx]   = (cfg[0x0B] == 3 || cfg[0x0B] == 4);   // 1=Norm 2=Rev 3=3D 4=3D-Rev
			infoKnown[idx] = true;
		}
	}
	if (mode == Drive::AUTO) mode = infoBidir[idx] ? Drive::BIDIR : Drive::NORMAL;
	drvRev[idx] = infoRev[idx];
	release();                                             // ESC must run its app (not the bootloader)
	drvFree(idx);
	if (mode == Drive::BIDIR) { drvB[idx] = new BidirDShotX1(PINS[idx], DSHOT_KBAUD); drvEdt[idx] = 20; }
	else                      { drvN[idx] = new DShotX4(PINS[idx], 1, DSHOT_KBAUD);   drvEdt[idx] = 0;  }
	drvTarget[idx] = 0; drvArmed[idx] = false; drvArmStart[idx] = millis(); drvLast[idx] = millis();
}
inline void spinThrottle(uint8_t idx, uint16_t throttle) {          // unidirectional: 0..SPIN_MAX
	using namespace detail;
	if (idx >= COUNT || !drvActive(idx) || !drvArmed[idx]) return;   // must be armed first
	drvTarget[idx] = throttle > SPIN_MAX ? SPIN_MAX : throttle;
	drvLast[idx] = millis();
}
inline void spinThrust(uint8_t idx, int16_t s) {   // reversible (3D): -1000..+1000, 0 = stop
	using namespace detail;
	if (idx >= COUNT || !drvActive(idx) || !drvArmed[idx]) return;
	if (s >  1000) s =  1000;
	if (s < -1000) s = -1000;
	uint16_t t = (s == 0) ? 0 : (s > 0 ? (uint16_t)(1000 + s)      // forward -> DShot 1048..2047
	                                   : (uint16_t)(1001 + s));    // reverse -> DShot   48..1047
	drvTarget[idx] = t; drvLast[idx] = millis();
}
inline bool spinArmed(uint8_t idx)      { return idx < COUNT && detail::drvActive(idx) && detail::drvArmed[idx]; }
inline bool spinReversible(uint8_t idx) { return idx < COUNT && detail::drvRev[idx]; }
inline const char* spinMode(uint8_t idx) {
	using namespace detail;
	if (idx >= COUNT) return "none";
	return drvB[idx] ? "bidir" : (drvN[idx] ? "normal" : "none");
}
inline void spinStop(uint8_t idx) {
	using namespace detail;
	if (idx >= COUNT) return;
	drvTarget[idx] = 0; drvArmed[idx] = false; drvFree(idx);
}
inline void spinDisarm(uint8_t idx) { spinStop(idx); }
inline void spinStopAll() { for (uint8_t i = 0; i < COUNT; i++) spinStop(i); }
inline bool spinning()    { for (uint8_t i = 0; i < COUNT; i++) if (detail::drvActive(i)) return true; return false; }
inline bool spinTele(uint8_t idx, Telem& out) {
	if (idx >= COUNT || !detail::drvB[idx]) return false;   // telemetry only on bidir DShot
	out = detail::drvTele[idx]; return true;
}
inline void spinPoll() {   // call every core0 loop while spinning: arm + frames + deadman + telemetry
	using namespace detail;
	bool any = false;
	for (uint8_t i = 0; i < COUNT; i++) {
		if (!drvActive(i)) continue;
		any = true;
		if (!drvArmed[i]) { drvTarget[i] = 0; if (millis() - drvArmStart[i] > SPIN_ARM_MS) drvArmed[i] = true; }
		else if (millis() - drvLast[i] > SPIN_DEADMAN_MS) drvTarget[i] = 0;   // deadman (armed only)
		if (drvB[i]) {
			uint32_t v = 0;
			switch (drvB[i]->getTelemetryPacket(&v)) {
				case BidirDshotTelemetryType::ERPM:        drvTele[i].rpm = (MOTOR_POLES > 1) ? v / (MOTOR_POLES / 2) : v; drvTele[i].valid = true; break;
				case BidirDshotTelemetryType::VOLTAGE:     drvTele[i].voltage = (float)v / 4.0f; break;
				case BidirDshotTelemetryType::CURRENT:     drvTele[i].current = v; break;
				case BidirDshotTelemetryType::TEMPERATURE: drvTele[i].tempC = v; break;
				case BidirDshotTelemetryType::STRESS:      drvTele[i].stress = v & ESC_STATUS_MAX_STRESS_MASK; break;
				case BidirDshotTelemetryType::STATUS:      drvTele[i].status = v; break;
				default: break;
			}
			// Enable EDT (extended telemetry: V/A/temp) once armed and still stopped — the ESC ignores
			// commands during its post-bootloader boot beep, so this must come after the arm window.
			if (drvArmed[i] && drvTarget[i] == 0 && drvEdt[i] > 0) { drvB[i]->sendRaw11Bit(13); drvEdt[i]--; continue; }
		}
		drvSend(i, drvTarget[i]);
	}
	if (any) delayMicroseconds(200);
}

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
