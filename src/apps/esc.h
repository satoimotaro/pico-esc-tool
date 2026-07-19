// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// esc — thin per-tool facade over the proven escs:: engine in esc_session.h. Esc delegates
// every call straight to the transport-agnostic free functions there (bootloader session,
// config edit, raw flash, and the DShot drive/spin state machine); it deliberately does NOT
// reimplement anything — the dual-core deadman/handshake in esc_session.h detail:: stays the
// single source of truth. The esc_tool firmware drives one global `Esc` instance from both its
// transports (USB-serial parser + Wi-Fi HTTP handlers) so the handlers only touch class methods.
#pragma once
#include "esc_session.h"

// SINGLE GLOBAL INSTANCE: Esc carries no per-object state — all state lives in escs:: (file-
// local statics in esc_session.h, one copy per app TU). Use the one global `esc`; do NOT
// instantiate a second Esc expecting independent state (they alias the same engine).
class Esc {
public:
	using Info  = escs::Info;
	using Telem = escs::Telem;
	using Drive = escs::Drive;
	static const uint8_t COUNT = escs::COUNT;
	static uint8_t pin(uint8_t i) { return escs::PINS[i]; }

	// bootloader session + config
	bool connect(uint8_t idx, Info& out)          { return escs::connect(idx, out); }
	bool scan(uint8_t idx, Info& out)             { return escs::scan(idx, out); }
	bool readConfig(uint8_t idx, uint8_t* out255) { return escs::readConfig(idx, out255); }
	int  editConfig(uint8_t idx, const uint16_t* offs, const uint8_t* vals, int n, bool& changed) {
		return escs::editConfig(idx, offs, vals, n, changed);
	}
	// raw flash primitives
	bool erasePage(uint8_t idx, uint16_t addr)    { return escs::erasePage(idx, addr); }
	bool writeFlash(uint8_t idx, uint16_t addr, const uint8_t* data, uint16_t len) {
		return escs::writeFlash(idx, addr, data, len);
	}
	bool readFlash(uint8_t idx, uint16_t addr, uint8_t* out, uint16_t len) {
		return escs::readFlash(idx, addr, out, len);
	}
	void release()                                { escs::release(); }
	// drive / spin
	void spinArm(uint8_t idx, Drive mode = Drive::AUTO) { escs::spinArm(idx, mode); }
	void spinThrottle(uint8_t idx, uint16_t throttle)   { escs::spinThrottle(idx, throttle); }
	void spinThrust(uint8_t idx, int16_t s)             { escs::spinThrust(idx, s); }
	bool spinArmed(uint8_t idx)                   { return escs::spinArmed(idx); }
	bool spinReversible(uint8_t idx)              { return escs::spinReversible(idx); }
	const char* spinMode(uint8_t idx)             { return escs::spinMode(idx); }
	bool spinInitOk(uint8_t idx)                  { return escs::spinInitOk(idx); }
	void spinStop(uint8_t idx)                    { escs::spinStop(idx); }
	void spinStopAll()                            { escs::spinStopAll(); }
	bool spinTele(uint8_t idx, Telem& out)        { return escs::spinTele(idx, out); }
	void spinPoll()                               { escs::spinPoll(); }
	// core1 worker (call from loop1)
	void core1Poll()                              { escs::core1Poll(); }
};
