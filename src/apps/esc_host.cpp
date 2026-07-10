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
enum class Op : uint8_t { NONE, CONNECT, READCFG, READPAGE, WRITEPAGE, ERASE, WRITEFLASH, READFLASH, RUN };
volatile Op       g_op     = Op::NONE;
volatile bool     g_opDone = false;
volatile bool     g_opOk   = false;
volatile uint8_t  g_opPin  = ESC_PINS[0];
volatile uint8_t  g_sig[2] = {0, 0};
volatile uint8_t  g_bootVer = 0, g_bootPages = 0;
static   uint8_t  g_cfg[esc_setup::kEepromLen];   // last READCFG result (core1 -> core0)
static   uint8_t  g_page[esc_setup::kPageLen];    // 512B page shared for READPAGE/WRITEPAGE
static   uint8_t  g_flBuf[256];                   // flash write/read chunk (ERASE/WRITEFLASH/READFLASH)
volatile uint16_t g_flAddr = 0;
volatile uint16_t g_flLen  = 0;
volatile bool     g_cfgOk  = false;
// Persistent session: which ESC is currently held in the bootloader (-1 = none). Config commands
// reuse it instead of re-entering (which would reboot the ESC each time); core1 keeps it alive.
volatile int8_t   g_session = -1;

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
	// The DShot-prime -> signal-loss entry is occasionally missed on the first try; retry a few
	// times so the CLI never sees spurious "no-connect".
	for (int attempt = 0; attempt < 3; attempt++) {
		dsh = new BidirDShotX1(pin, DSHOT_KBAUD);
		uint32_t t0 = millis();
		while (millis() - t0 < PRIME_MS) { dsh->sendThrottle(0); delayMicroseconds(300); }
		delete dsh; dsh = nullptr;
		pinMode(pin, OUTPUT);
		digitalWrite(pin, HIGH);
		delay(HIGH_HOLD_MS);
		if (runOp(Op::CONNECT, pin)) return true;
	}
	return false;
}

// Ensure ESC `i` is the one held in the bootloader session, entering it only if needed (so a
// sequence of config commands doesn't reboot the ESC between each). Runs/releases a different ESC
// first if one was held.
static bool ensureConnected(uint8_t i) {
	if (g_session == (int8_t)i) return true;
	if (g_session >= 0) { runOp(Op::RUN, ESC_PINS[g_session]); g_session = -1; }
	if (enterBootloader(ESC_PINS[i])) { g_session = (int8_t)i; return true; }
	return false;
}

// Release the session (reboot the held ESC back to its app).
static void releaseSession() {
	if (g_session >= 0) { runOp(Op::RUN, ESC_PINS[g_session]); g_session = -1; }
}

static void printHex(const uint8_t* p, uint16_t n) {
	for (uint16_t i = 0; i < n; i++) {
		uint8_t b = p[i];
		Serial.print("0123456789ABCDEF"[b >> 4]);
		Serial.print("0123456789ABCDEF"[b & 0xF]);
	}
}
static int hexVal(char c) {
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	return -1;
}
// Parse a hex string into buf (max cap bytes). Returns the byte count, or -1 on a bad digit/overflow.
static int parseHex(const char* s, uint8_t* buf, int cap) {
	int n = 0;
	for (; s[0] && s[1]; s += 2) {
		if (n >= cap) return -1;
		int hi = hexVal(s[0]), lo = hexVal(s[1]);
		if (hi < 0 || lo < 0) return -1;
		buf[n++] = (uint8_t)((hi << 4) | lo);
	}
	return n;
}

// Emit one "esc|..." line for a scan: connect (via the session, no reboot) and read identity.
// The ESC is LEFT in the bootloader session — `run`/`disconnect` restarts it when you're done.
static void scanOne(uint8_t idx) {
	if (!ensureConnected(idx)) { Serial.printf("esc|%u|%u|0\n", idx, ESC_PINS[idx]); return; }
	esc_setup::Settings s;
	bool cfg = runOp(Op::READCFG, ESC_PINS[idx]);
	if (cfg) esc_setup::decode(g_cfg, esc_setup::kEepromLen, s);
	Serial.printf("esc|%u|%u|1|%02X%02X|%u|%s|%s|%u.%u\n",
		idx, ESC_PINS[idx], g_sig[0], g_sig[1], g_bootVer,
		cfg ? s.layoutTag : "?", cfg ? s.name : "?", s.mainRevision, s.subRevision);
}

void setup() { Serial.begin(115200); }

void loop() {
	static char line[600];             // room for a writeflash chunk (256 B = 512 hex) or override list
	static uint16_t len = 0;
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
				else if (!ensureConnected(i)) { Serial.println("err no-connect"); }
				else {
					bool cfg = runOp(Op::READCFG, ESC_PINS[i]);   // stays in the session (no reboot)
					if (cfg) { Serial.print("cfg|"); printHex(g_cfg, esc_setup::kEepromLen); Serial.println(); }
					Serial.println(cfg ? "ok" : "err read-failed");
				}
			} else if (!strcmp(cmd, "enter")) {
				char* a = strtok(nullptr, " ");
				int i = a ? atoi(a) : -1;
				if (i < 0 || i >= NUM_ESC) Serial.println("err bad-index");
				else if (!ensureConnected(i)) Serial.println("err no-connect");
				else { Serial.printf("dev|%02X%02X|%u|%u\n", g_sig[0], g_sig[1], g_bootVer, g_bootPages); Serial.println("ok"); }
			} else if (!strcmp(cmd, "run") || !strcmp(cmd, "disconnect")) {
				releaseSession();                       // reboot the held ESC back to its app
				Serial.println("ok");
			} else if (!strcmp(cmd, "editpage")) {
				// editpage <i> <off:val,off:val,...>  (hex) — read-modify-write the config page
				char* a   = strtok(nullptr, " ");
				char* ovr = strtok(nullptr, " ");
				int i = a ? atoi(a) : -1;
				if (i < 0 || i >= NUM_ESC || !ovr) { Serial.println("err bad-args"); }
				else if (!ensureConnected(i)) { Serial.println("err no-connect"); }
				else if (!runOp(Op::READPAGE, ESC_PINS[i])) { Serial.println("err read-failed"); }
				else {
					bool bad = false; int applied = 0;
					for (char* tok = strtok(ovr, ","); tok; tok = strtok(nullptr, ",")) {
						char* colon = strchr(tok, ':');
						if (!colon) { bad = true; break; }
						*colon = '\0';
						long off = strtol(tok, nullptr, 16), val = strtol(colon + 1, nullptr, 16);
						if (off < 0 || off >= (long)esc_setup::kPageLen || val < 0 || val > 255) { bad = true; break; }
						g_page[off] = (uint8_t)val; applied++;
					}
					if (bad) { Serial.println("err bad-override"); }   // stays in session
					else {
						bool wok = runOp(Op::WRITEPAGE, ESC_PINS[i]);
						Serial.printf("edited %d byte(s)\n", applied);
						Serial.println(wok ? "ok" : "err write-verify-failed");
					}
				}
			} else if (!strcmp(cmd, "erase")) {           // erase <i> <pageAddrHex>
				char* a = strtok(nullptr, " "); char* ad = strtok(nullptr, " ");
				int i = a ? atoi(a) : -1;
				if (i < 0 || i >= NUM_ESC || !ad) Serial.println("err bad-args");
				else if (!ensureConnected(i)) Serial.println("err no-connect");
				else { g_flAddr = (uint16_t)strtol(ad, nullptr, 16);
					Serial.println(runOp(Op::ERASE, ESC_PINS[i]) ? "ok" : "err erase-failed"); }
			} else if (!strcmp(cmd, "writeflash")) {      // writeflash <i> <addrHex> <dataHex>
				char* a = strtok(nullptr, " "); char* ad = strtok(nullptr, " "); char* hx = strtok(nullptr, " ");
				int i = a ? atoi(a) : -1;
				if (i < 0 || i >= NUM_ESC || !ad || !hx) Serial.println("err bad-args");
				else if (!ensureConnected(i)) Serial.println("err no-connect");
				else {
					int n = parseHex(hx, g_flBuf, sizeof(g_flBuf));
					if (n <= 0) Serial.println("err bad-hex");
					else { g_flAddr = (uint16_t)strtol(ad, nullptr, 16); g_flLen = (uint16_t)n;
						Serial.println(runOp(Op::WRITEFLASH, ESC_PINS[i]) ? "ok" : "err write-failed"); }
				}
			} else if (!strcmp(cmd, "readflash")) {       // readflash <i> <addrHex> <len>
				char* a = strtok(nullptr, " "); char* ad = strtok(nullptr, " "); char* ln = strtok(nullptr, " ");
				int i = a ? atoi(a) : -1;
				int len = ln ? atoi(ln) : -1;
				if (i < 0 || i >= NUM_ESC || !ad || len < 1 || len > (int)sizeof(g_flBuf)) Serial.println("err bad-args");
				else if (!ensureConnected(i)) Serial.println("err no-connect");
				else { g_flAddr = (uint16_t)strtol(ad, nullptr, 16); g_flLen = (uint16_t)len;
					bool k = runOp(Op::READFLASH, ESC_PINS[i]);
					if (k) { Serial.print("data|"); printHex(g_flBuf, len); Serial.println(); }
					Serial.println(k ? "ok" : "err read-failed"); }
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
	if (g_op == Op::NONE || g_opDone) {
		// keep the held ESC in its bootloader (else it times out and reboots -> spurious beep)
		static uint32_t lastKa = 0;
		if (g_session >= 0 && bl.connected() && millis() - lastKa > 100) {
			bl.keepAlive();
			lastKa = millis();
		}
		delay(1);
		return;
	}
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
	} else if (op == Op::READPAGE) {
		ok = esc_setup::readPage(bl, g_page);
	} else if (op == Op::WRITEPAGE) {
		ok = esc_setup::writePage(bl, g_page);   // erase + write + read-back verify
	} else if (op == Op::ERASE) {
		ok = bl.erasePage(g_flAddr);
	} else if (op == Op::WRITEFLASH) {
		ok = bl.writeFlash(g_flAddr, g_flBuf, g_flLen);
	} else if (op == Op::READFLASH) {
		ok = bl.readFlash(g_flAddr, g_flBuf, g_flLen);
	} else if (op == Op::RUN) {
		ok = bl.run();
	}
	g_opOk = ok;
	g_opDone = true;
}
