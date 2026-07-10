// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// esc_host — USB-serial transport for the RP2040 ESC tool. A thin line-based command parser over
// the shared esc_session API; host/esctool.py drives it. (The Wi-Fi web front-end is esc_web.)
//
// Protocol (ASCII, one command per line '\n'; replies end with a final "ok" or "err <msg>"):
//   ping / pins
//   scan                 -> per pin: "esc|<i>|<pin>|<present>|<sigHHLL>|<boot>|<layout>|<name>|<fw>"
//   read <i>             -> "cfg|<510-hex>"                 (holds the ESC in the session)
//   enter <i>            -> "dev|<sigHHLL>|<boot>|<pages>"
//   run|disconnect       -> restart the held ESC
//   editpage <i> <off:val,...>   erase/write config page (skips write if unchanged)
//   erase|writeflash|readflash   raw flash access (firmware flashing)
#include <Arduino.h>
#include "esc_session.h"

static void printHex(const uint8_t* p, uint16_t n) {
	for (uint16_t i = 0; i < n; i++) {
		Serial.print("0123456789ABCDEF"[p[i] >> 4]);
		Serial.print("0123456789ABCDEF"[p[i] & 0xF]);
	}
}
static int hexVal(char c) {
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	return -1;
}
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

void setup()  { Serial.begin(115200); }
void setup1() {}
void loop1()  { escs::core1Poll(); }

void loop() {
	static char line[600];
	static uint16_t len = 0;
	static uint8_t flbuf[256];
	while (Serial.available()) {
		int c = Serial.read();
		if (c == '\r') continue;
		if (c != '\n') { if (len < sizeof(line) - 1) line[len++] = (char)c; continue; }
		line[len] = '\0'; len = 0;

		char* cmd = strtok(line, " ");
		if (!cmd) continue;
		auto argi = []() { char* a = strtok(nullptr, " "); return a ? atoi(a) : -1; };

		if (!strcmp(cmd, "ping")) {
			Serial.println("id esc_host v0"); Serial.println("ok");
		} else if (!strcmp(cmd, "pins")) {
			Serial.printf("pins %u", escs::COUNT);
			for (uint8_t i = 0; i < escs::COUNT; i++) Serial.printf(" %u", escs::PINS[i]);
			Serial.println(); Serial.println("ok");
		} else if (!strcmp(cmd, "scan")) {
			for (uint8_t i = 0; i < escs::COUNT; i++) {
				escs::Info in;
				if (!escs::scan(i, in)) Serial.printf("esc|%u|%u|0\n", i, escs::PINS[i]);
				else Serial.printf("esc|%u|%u|1|%04X|%u|%s|%s|%u.%u\n",
					i, in.pin, in.sig, in.bootVer, in.layout, in.name, in.fwMain, in.fwSub);
			}
			Serial.println("ok");
		} else if (!strcmp(cmd, "read")) {
			int i = argi();
			uint8_t cfg[esc_setup::kEepromLen];
			if (i < 0 || i >= escs::COUNT) Serial.println("err bad-index");
			else if (!escs::readConfig((uint8_t)i, cfg)) Serial.println("err no-connect");
			else { Serial.print("cfg|"); printHex(cfg, esc_setup::kEepromLen); Serial.println(); Serial.println("ok"); }
		} else if (!strcmp(cmd, "enter")) {
			int i = argi();
			escs::Info in;
			if (i < 0 || i >= escs::COUNT) Serial.println("err bad-index");
			else if (!escs::connect((uint8_t)i, in)) Serial.println("err no-connect");
			else { Serial.printf("dev|%04X|%u|%u\n", in.sig, in.bootVer, in.bootPages); Serial.println("ok"); }
		} else if (!strcmp(cmd, "run") || !strcmp(cmd, "disconnect")) {
			escs::release(); Serial.println("ok");
		} else if (!strcmp(cmd, "editpage")) {
			int i = argi();
			char* ovr = strtok(nullptr, " ");
			if (i < 0 || i >= escs::COUNT || !ovr) { Serial.println("err bad-args"); continue; }
			uint16_t offs[160]; uint8_t vals[160]; int n = 0; bool bad = false;
			for (char* tok = strtok(ovr, ","); tok && !bad; tok = strtok(nullptr, ",")) {
				char* colon = strchr(tok, ':');
				if (!colon || n >= 160) { bad = true; break; }
				*colon = '\0';
				long off = strtol(tok, nullptr, 16), val = strtol(colon + 1, nullptr, 16);
				if (off < 0 || off >= (long)esc_setup::kPageLen || val < 0 || val > 255) { bad = true; break; }
				offs[n] = (uint16_t)off; vals[n] = (uint8_t)val; n++;
			}
			if (bad) { Serial.println("err bad-override"); continue; }
			bool changed = false;
			int r = escs::editConfig((uint8_t)i, offs, vals, n, changed);
			if      (r == -1) Serial.println("err no-connect");
			else if (r == -2) Serial.println("err read-failed");
			else if (r == -3) Serial.println("err bad-override");
			else if (r == -4) Serial.println("err write-verify-failed");
			else if (r ==  0) { Serial.println("unchanged (flash write skipped)"); Serial.println("ok"); }
			else              { Serial.printf("edited %d byte(s)\n", n); Serial.println("ok"); }
		} else if (!strcmp(cmd, "erase")) {
			int i = argi(); char* ad = strtok(nullptr, " ");
			if (i < 0 || i >= escs::COUNT || !ad) Serial.println("err bad-args");
			else Serial.println(escs::erasePage((uint8_t)i, (uint16_t)strtol(ad, nullptr, 16)) ? "ok" : "err erase-failed");
		} else if (!strcmp(cmd, "writeflash")) {
			int i = argi(); char* ad = strtok(nullptr, " "); char* hx = strtok(nullptr, " ");
			if (i < 0 || i >= escs::COUNT || !ad || !hx) { Serial.println("err bad-args"); continue; }
			int n = parseHex(hx, flbuf, sizeof(flbuf));
			if (n <= 0) Serial.println("err bad-hex");
			else Serial.println(escs::writeFlash((uint8_t)i, (uint16_t)strtol(ad, nullptr, 16), flbuf, (uint16_t)n) ? "ok" : "err write-failed");
		} else if (!strcmp(cmd, "readflash")) {
			int i = argi(); char* ad = strtok(nullptr, " "); char* ln = strtok(nullptr, " ");
			int rlen = ln ? atoi(ln) : -1;
			if (i < 0 || i >= escs::COUNT || !ad || rlen < 1 || rlen > (int)sizeof(flbuf)) Serial.println("err bad-args");
			else if (!escs::readFlash((uint8_t)i, (uint16_t)strtol(ad, nullptr, 16), flbuf, (uint16_t)rlen)) Serial.println("err read-failed");
			else { Serial.print("data|"); printHex(flbuf, rlen); Serial.println(); Serial.println("ok"); }
		} else {
			Serial.println("err unknown-cmd");
		}
	}
}
