// SPDX-License-Identifier: GPL-3.0-or-later
//
// esc_host — unified host-driven firmware for the RP2040 ESC tool. Replaces the one-off spikes
// with a single build that a PC-side CLI (host/esctool.py) drives over USB-CDC serial with a
// small line-based text protocol. core0 parses commands + owns DShot (PIO) for bootloader entry;
// core1 owns the 1-wire bit-bang (blheli_bl) — the split USB-safe layout proven by the spikes.
//
// Multi-ESC: ESC_PINS lists every signal pin; commands take a 0-based ESC index. (Start with one.)
//
// Protocol (ASCII, one command per line '\n'; replies end with a final "ok" or "err <msg>"):
//   ping                 -> "id esc_host v0" / ok
//   pins                 -> "pins <n> <p0> <p1> ..." / ok
//   scan                 -> per pin: "esc|<i>|<pin>|<present>|<sigHHLL>|<boot>|<layout>|<name>|<fw>" / ok
//   read <i>             -> "cfg|<510-hex>" / ok            (enter BL, read 255B config, run app)
//   enter <i>            -> "dev|<sigHHLL>|<boot>|<pages>" / ok | err   (leave ESC in the bootloader)
//   run <i>              -> ok                              (exit bootloader -> app)
// Fields with spaces (name) never contain '|', so the CLI splits on '|'.
#include <Arduino.h>
#include <PIO_DShot.h>
#include <string.h>
#include "blheli_bl.h"
#include "esc_setup.h"

#define DSHOT_KBAUD  600
#define PRIME_MS     500
#define HIGH_HOLD_MS 1000

static const uint8_t ESC_PINS[] = { 10 };           // add more pins for multi-ESC
static const uint8_t NUM_ESC = sizeof(ESC_PINS) / sizeof(ESC_PINS[0]);

static BidirDShotX1*         dsh = nullptr;                            // core0 only (PIO)
static blheli_bl::Bootloader bl({ .signalPin = ESC_PINS[0], .baud = 0 });  // core1 only

// --- core0 <-> core1 op handshake ---
enum class Op : uint8_t { NONE, CONNECT, READCFG, RUN };
volatile Op       g_op     = Op::NONE;
volatile bool     g_opDone = false;
volatile bool     g_opOk   = false;
volatile uint8_t  g_opPin  = ESC_PINS[0];
volatile uint8_t  g_sig[2] = {0, 0};
volatile uint8_t  g_bootVer = 0, g_bootPages = 0;
static   uint8_t  g_cfg[esc_setup::kEepromLen];   // last READCFG result (core1 -> core0)
volatile bool     g_cfgOk  = false;

// core0: hand an op to core1 and block until it finishes.
static bool runOp(Op op, uint8_t pin) {
	g_opPin = pin;
	g_opOk = false;
	g_opDone = false;
	g_op = op;
	while (!g_opDone) delay(1);
	g_op = Op::NONE;
	return g_opOk;
}

// core0: DShot-prime the ESC then drop the signal (line held HIGH) so it jumps to init_no_signal's
// bootloader, then have core1 connect. Returns true if the bootloader answered.
static bool enterBootloader(uint8_t pin) {
	dsh = new BidirDShotX1(pin, DSHOT_KBAUD);
	uint32_t t0 = millis();
	while (millis() - t0 < PRIME_MS) { dsh->sendThrottle(0); delayMicroseconds(300); }
	delete dsh; dsh = nullptr;
	pinMode(pin, OUTPUT);
	digitalWrite(pin, HIGH);
	delay(HIGH_HOLD_MS);
	return runOp(Op::CONNECT, pin);
}

static void printHex(const uint8_t* p, uint16_t n) {
	for (uint16_t i = 0; i < n; i++) {
		uint8_t b = p[i];
		Serial.print("0123456789ABCDEF"[b >> 4]);
		Serial.print("0123456789ABCDEF"[b & 0xF]);
	}
}

// Emit one "esc|..." line for a scan: enter BL, read identity, run app again (non-destructive).
static void scanOne(uint8_t idx) {
	uint8_t pin = ESC_PINS[idx];
	if (!enterBootloader(pin)) { Serial.printf("esc|%u|%u|0\n", idx, pin); return; }
	esc_setup::Settings s;
	bool cfg = runOp(Op::READCFG, pin);
	if (cfg) esc_setup::decode(g_cfg, esc_setup::kEepromLen, s);
	Serial.printf("esc|%u|%u|1|%02X%02X|%u|%s|%s|%u.%02u\n",
		idx, pin, g_sig[0], g_sig[1], g_bootVer,
		cfg ? s.layoutTag : "?", cfg ? s.name : "?", s.mainRevision, s.subRevision);
	runOp(Op::RUN, pin);   // leave the ESC running normally
}

void setup() { Serial.begin(115200); }

void loop() {
	static char line[64];
	static uint8_t len = 0;
	while (Serial.available()) {
		int c = Serial.read();
		if (c == '\r') continue;
		if (c == '\n') {
			line[len] = '\0';
			// --- dispatch ---
			char* cmd = strtok(line, " ");
			if (!cmd) { len = 0; continue; }
			if (!strcmp(cmd, "ping")) {
				Serial.println("id esc_host v0"); Serial.println("ok");
			} else if (!strcmp(cmd, "pins")) {
				Serial.printf("pins %u", NUM_ESC);
				for (uint8_t i = 0; i < NUM_ESC; i++) Serial.printf(" %u", ESC_PINS[i]);
				Serial.println(); Serial.println("ok");
			} else if (!strcmp(cmd, "scan")) {
				for (uint8_t i = 0; i < NUM_ESC; i++) scanOne(i);
				Serial.println("ok");
			} else if (!strcmp(cmd, "read")) {
				char* a = strtok(nullptr, " ");
				int i = a ? atoi(a) : -1;
				if (i < 0 || i >= NUM_ESC) { Serial.println("err bad-index"); }
				else if (!enterBootloader(ESC_PINS[i])) { Serial.println("err no-connect"); }
				else {
					bool cfg = runOp(Op::READCFG, ESC_PINS[i]);
					if (cfg) { Serial.print("cfg|"); printHex(g_cfg, esc_setup::kEepromLen); Serial.println(); }
					runOp(Op::RUN, ESC_PINS[i]);
					Serial.println(cfg ? "ok" : "err read-failed");
				}
			} else if (!strcmp(cmd, "enter")) {
				char* a = strtok(nullptr, " ");
				int i = a ? atoi(a) : -1;
				if (i < 0 || i >= NUM_ESC) Serial.println("err bad-index");
				else if (!enterBootloader(ESC_PINS[i])) Serial.println("err no-connect");
				else { Serial.printf("dev|%02X%02X|%u|%u\n", g_sig[0], g_sig[1], g_bootVer, g_bootPages); Serial.println("ok"); }
			} else if (!strcmp(cmd, "run")) {
				char* a = strtok(nullptr, " ");
				int i = a ? atoi(a) : -1;
				if (i < 0 || i >= NUM_ESC) Serial.println("err bad-index");
				else { runOp(Op::RUN, ESC_PINS[i]); Serial.println("ok"); }
			} else {
				Serial.println("err unknown-cmd");
			}
			len = 0;
		} else if (len < sizeof(line) - 1) {
			line[len++] = (char)c;
		}
	}
}

// =================== core1: 1-wire bit-bang worker ===================
void setup1() {}

void loop1() {
	if (g_op == Op::NONE || g_opDone) { delay(1); return; }
	Op op = g_op;
	bool ok = false;
	if (op == Op::CONNECT) {
		bl.setSignalPin(g_opPin);
		bl.begin();
		uint32_t t0 = millis();
		while (millis() - t0 < 500 && !ok) {
			uint8_t raw[8] = {0};
			int n = bl.connectRawProbe(raw, 15);
			if (n >= 8 && raw[0] == '4' && raw[1] == '7' && raw[2] == '1') {
				if (bl.connect()) {
					const auto& d = bl.lastDevice();
					g_sig[0] = d.signature[0]; g_sig[1] = d.signature[1];
					g_bootVer = d.bootVersion; g_bootPages = d.bootPages;
					ok = true;
				}
			}
		}
	} else if (op == Op::READCFG) {
		esc_setup::Settings s;
		ok = esc_setup::read(bl, s);
		if (ok) memcpy(g_cfg, s.raw, esc_setup::kEepromLen);
		g_cfgOk = ok;
	} else if (op == Op::RUN) {
		ok = bl.run();
	}
	g_opOk = ok;
	g_opDone = true;
}
