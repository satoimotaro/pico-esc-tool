// SPDX-License-Identifier: GPL-3.0-or-later
//
// A1 spike (FLASH WRITE): connect to the BLHeli-S bootloader, then — only after an explicit
// 'Y' typed on the serial link — ERASE + PROGRAM + VERIFY the firmware image embedded in
// firmware_hex.h (BLHeli-S 16.7; see that file for how to fill it in).
//
// Uses the SAME DShot-primed signal-loss entry as spike_setup/spike_flash: prime the ESC
// with valid DShot, then drop the signal with the line held HIGH to jump into init_no_signal.
//   core0: [1] prime DShot -> [2] stop + hold HIGH (signal loss) -> gate on 'Y' -> report
//   core1: [3] BootInit/connect -> keepAlive while waiting -> on go, parse+program+verify
//
// SAFETY: esc_flash refuses any byte at/above 0x1A00, so the EEPROM parameter page and the
// bootloader (0x1C00-0x1FFF) are never touched. A wrong/oversized HEX errors out; it cannot
// brick the ESC. Nothing is erased until you type 'Y'. EFM8BB21 should report signature E8 B2.
#include <Arduino.h>
#include <PIO_DShot.h>
#include <hardware/structs/sio.h>
#include "blheli_bl.h"
#include "esc_flash.h"
#include "esc_setup.h"
#include "firmware_hex.h"

#define SIGNAL_PIN   10
#define DSHOT_KBAUD  600
#define PRIME_MS     500
#define HIGH_HOLD_MS 1000  // CONFIRMED on HW: 200 fails, 600 connects; 1000 = margin (past the beep melody)
#define ALLOW_INCOMPAT 0   // 1 = permit 'Y' even if the compat check fails (DANGER: wrong layout/MCU)

static BidirDShotX1*         esc = nullptr;                       // core0 only
static blheli_bl::Bootloader bl({ .signalPin = SIGNAL_PIN, .baud = 0 });  // core1 only
static esc_flash::HexImage   g_img;                              // parsed once by core1

// --- shared state (core0 sequences + Serial; core1 bit-bangs) ---
volatile bool     g_startEntry  = false;
volatile bool     g_connected   = false;
volatile bool     g_done        = false;   // core1 -> core0: connect attempt finished
volatile int      g_bestN       = 0;
volatile uint8_t  g_raw[8]       = {0};
volatile uint32_t g_attempts     = 0;
volatile uint8_t  g_sig[2]       = {0, 0};
volatile uint8_t  g_bootVer      = 0;
volatile uint8_t  g_bootPages    = 0;
// ESC identity read from its config (core1 -> core0), for the compatibility guard
static   char     g_escLayout[17] = {0};   // ESC LAYOUT tag (0x1A40)
volatile uint8_t  g_escFwMain    = 0;
volatile uint8_t  g_escFwSub     = 0;
volatile bool     g_escIdOk      = false;  // config read succeeded
// image parse result (core1 -> core0)
volatile bool     g_parsed       = false;
volatile bool     g_parseOk      = false;
volatile uint16_t g_imgMin       = 0;
volatile uint16_t g_imgMax       = 0;
volatile uint16_t g_bootSkip     = 0;      // bootloader-region bytes in the HEX (not flashed)
const char* volatile g_parseErr  = "";
static   esc_flash::Compat g_compat;       // firmware<->ESC match result (filled at parse)
// flash go/result handshake
volatile bool     g_flashGo      = false;   // core0 -> core1: user confirmed, do it
volatile bool     g_flashDoneOk  = false;   // core1 -> core0
volatile bool     g_flashDoneErr = false;
volatile uint16_t g_progDone     = 0;       // page progress (core1 -> core0)
volatile uint16_t g_progTotal    = 0;
volatile bool     g_verifying    = false;
volatile bool     g_writingCfg   = false;   // core1 -> core0: writing the firmware's default config
volatile bool     g_cfgWritten   = false;   // default config applied after flash
// keepAlive stats while we wait for the user
volatile uint32_t g_keepAliveOk  = 0;
volatile uint32_t g_keepAliveNo  = 0;

static void progressCb(uint16_t done, uint16_t total, void*) {
	g_progDone = done; g_progTotal = total;
}

void setup() { Serial.begin(115200); }

// =================== core0: DShot priming + sequencing + Serial + confirm gate ===============
void loop() {
	static int      phase = 0;
	static uint32_t t0 = 0, cycle = 0, lastKa = 0, lastProg = 0;
	static bool     banner = false, connMsg = false, doneMsg = false;
	static bool     infoShown = false, prompted = false;

	if (!banner && millis() > 800) {
		banner = true;
		Serial.println(F("[spike_program] FLASH WRITER: DShot-primed connect, then 'Y' to erase+program+verify"));
		Serial.printf("firmware image: \"%s\"\n", kFirmwareHexName);
	}
	if (!banner) return;

	switch (phase) {
	case 0:
		cycle++;
		Serial.printf("\n== cycle %lu ==\n[1] priming ESC with DShot throttle 0 (%dms)...\n",
			(unsigned long)cycle, PRIME_MS);
		esc = new BidirDShotX1(SIGNAL_PIN, DSHOT_KBAUD);
		t0 = millis();
		phase = 1;
		break;

	case 1:
		esc->sendThrottle(0);
		delayMicroseconds(300);
		if (millis() - t0 >= PRIME_MS) {
			delete esc; esc = nullptr;
			pinMode(SIGNAL_PIN, OUTPUT);
			digitalWrite(SIGNAL_PIN, HIGH);
			Serial.printf("[2] DShot stopped; holding HIGH %dms (signal-loss -> BL entry)\n", HIGH_HOLD_MS);
			delay(HIGH_HOLD_MS);
			Serial.println(F("[3] core1 streaming BootInit, expecting E8 B2..."));
			connMsg = doneMsg = infoShown = prompted = false;
			g_connected = g_done = false;
			g_flashGo = g_flashDoneOk = g_flashDoneErr = false;
			g_bestN = 0; g_attempts = 0;
			g_startEntry = true;
			phase = 2;
		}
		break;

	case 2:  // wait for connect result
		if (g_connected && !connMsg) {
			connMsg = true;
			Serial.printf("== CONNECTED (%lu probes) ==\nsignature : %02X %02X   bootVer: %u  bootPages: %u\n",
				(unsigned long)g_attempts, g_sig[0], g_sig[1], g_bootVer, g_bootPages);
			lastKa = millis();
			phase = 3;
		}
		if (g_done && !g_connected && !doneMsg) {
			doneMsg = true;
			Serial.printf("no reply (best=%d bytes). Retrying full cycle...\n", g_bestN);
			g_startEntry = false;
			delay(200);
			phase = 0;
		}
		break;

	case 3:  // connected: show parsed image + confirm gate
		if (!infoShown && g_parsed) {
			infoShown = true;
			if (!g_parseOk) {
				Serial.printf("HEX parse FAILED: %s\n", g_parseErr ? g_parseErr : "?");
				Serial.println(F("Fill in firmware_hex.h with a valid BLHeli-S 16.7 HEX. Idle (keepAlive only)."));
				phase = 4;   // park; nothing to flash
				break;
			}
			uint16_t bytes = (uint16_t)(g_imgMax - g_imgMin);
			uint16_t p0 = (uint16_t)(g_imgMin / esc_flash::kPageSize);
			uint16_t p1 = (uint16_t)((g_imgMax - 1) / esc_flash::kPageSize);
			Serial.printf("image     : 0x%04X..0x%04X (%u bytes), pages %u..%u of app region [0x0000,0x%04X)\n",
				g_imgMin, g_imgMax, bytes, p0, p1, (unsigned)esc_flash::kAppEnd);
			if (g_bootSkip) Serial.printf("note      : HEX carries %u bootloader byte(s) (>=0x1C00) — NOT flashed (BL preserved)\n",
				g_bootSkip);
			Serial.printf("ESC id    : sig %02X %02X  layout \"%s\"  fw %u.%02u\n",
				g_sig[0], g_sig[1], g_escIdOk ? g_escLayout : "(unread)", g_escFwMain, g_escFwSub);
			Serial.printf("compat    : %s => %s\n", g_compat.detail, g_compat.ok ? "OK" : "BLOCK");
		}
		if (infoShown && g_parseOk && !prompted) {
			prompted = true;
			bool canFlash = g_compat.ok || ALLOW_INCOMPAT;
			if (!canFlash) {
				Serial.println(F("\n!!! INCOMPATIBLE firmware — refusing to flash (would mis-map FETs / wrong MCU)."));
				Serial.println(F("Fix firmware_hex.h to a matching layout+MCU, or set ALLOW_INCOMPAT=1 to override."));
				Serial.println(F("Press any key to re-cycle. (idle, keepAlive only)"));
				phase = 4;   // park; do NOT accept a flash confirmation
				break;
			}
			if (!g_compat.ok && ALLOW_INCOMPAT)
				Serial.println(F("\n*** WARNING: compat check FAILED but ALLOW_INCOMPAT=1 — proceeding is DANGEROUS. ***"));
			Serial.println(F("\n*** ERASE + FLASH the ESC now? This overwrites the application. ***"));
			Serial.println(F("*** Bootloader (0x1C00+) and EEPROM (0x1A00) are preserved.       ***"));
			Serial.println(F("Type Y then Enter to proceed; any other key aborts this cycle."));
		}
		if (infoShown && g_parseOk && (g_compat.ok || ALLOW_INCOMPAT)) {
			// pump keepAlive status while waiting for input
			if (millis() - lastKa > 1500) {
				lastKa = millis();
				Serial.printf("(waiting for Y; keepAlive %lu ok / %lu no)\n",
					(unsigned long)g_keepAliveOk, (unsigned long)g_keepAliveNo);
			}
			if (Serial.available()) {
				int c = Serial.read();
				while (Serial.available()) Serial.read();   // drain rest of line
				if (c == 'Y' || c == 'y') {
					Serial.println(F(">> confirmed: erasing + programming + verifying..."));
					lastProg = millis();
					g_flashGo = true;
					phase = 5;
				} else {
					Serial.println(F(">> aborted by user. Retrying full cycle..."));
					g_startEntry = false;
					delay(200);
					phase = 0;
				}
			}
		}
		break;

	case 4:  // parse failed: idle so the link/keepAlive is observable, allow re-cycle on any key
		if (Serial.available()) { while (Serial.available()) Serial.read(); g_startEntry = false; delay(200); phase = 0; }
		break;

	case 5:  // flashing in progress on core1: stream progress
		if (millis() - lastProg > 250) {
			lastProg = millis();
			if (g_writingCfg) Serial.println(F("  writing default config..."));
			else Serial.printf("  %s page %u/%u\n", g_verifying ? "verify" : "program",
				g_progDone, g_progTotal);
		}
		if (g_flashDoneOk) {
			Serial.println(F("\n*** FLASH OK: program + verify succeeded. ***"));
			Serial.println(g_cfgWritten ? F("Default config applied (name/settings/version reset to this firmware's).")
			                            : F("(no config section in HEX; settings unchanged)"));
			Serial.println(F("Power-cycle the ESC to run the new firmware. (idle)"));
			phase = 6;
		} else if (g_flashDoneErr) {
			Serial.println(F("\n!!! FLASH FAILED (erase/write/verify error). ESC app may be partial."));
			Serial.println(F("The bootloader is intact — re-run and retry. (idle)"));
			phase = 6;
		}
		break;

	case 6:  // terminal: park
		delay(10);
		break;
	}
}

// =================== core1: bootloader bit-bang (connect, parse, program, verify) ============
void setup1() {}

void loop1() {
	if (!g_startEntry) { delay(2); return; }

	// --- connect ---
	if (!g_connected && !g_done) {
		bl.begin();
		uint32_t t0 = millis();
		while (millis() - t0 < 500 && !g_connected) {
			uint8_t raw[8] = {0};
			int n = bl.connectRawProbe(raw, 15);
			g_attempts++;
			for (int i = 0; i < 8; i++) g_raw[i] = raw[i];
			if (n > g_bestN) g_bestN = n;
			if (n >= 8 && raw[0] == '4' && raw[1] == '7' && raw[2] == '1') {
				if (bl.connect()) {
					const auto& d = bl.lastDevice();
					g_sig[0] = d.signature[0]; g_sig[1] = d.signature[1];
					g_bootVer = d.bootVersion; g_bootPages = d.bootPages;
					// read the ESC's own config to learn its layout/MCU for the compat guard
					esc_setup::Settings s;
					if (esc_setup::read(bl, s)) {
						strncpy(g_escLayout, s.layoutTag, sizeof(g_escLayout) - 1);
						g_escFwMain = s.mainRevision; g_escFwSub = s.subRevision;
						g_escIdOk = true;
					}
					g_connected = true;
				}
			}
		}
		g_done = true;
		return;
	}

	if (!g_connected) { delay(2); return; }

	// --- parse the embedded HEX once (pure CPU, safe on core1) ---
	if (!g_parsed) {
		const char* err = "";
		bool ok = esc_flash::parseIntelHex(kFirmwareHex, strlen(kFirmwareHex), g_img, &err);
		g_parseOk = ok;
		g_parseErr = err;
		g_imgMin = g_img.minAddr;
		g_imgMax = g_img.maxAddr;
		g_bootSkip = g_img.bootSkipped;
		if (ok) {
			uint16_t escSig = (uint16_t)((g_sig[0] << 8) | g_sig[1]);
			g_compat = esc_flash::checkCompatibility(escSig, g_escIdOk ? g_escLayout : "", g_img);
		}
		g_parsed = true;
		return;
	}

	// --- flash when core0 confirms ---
	if (g_flashGo && !g_flashDoneOk && !g_flashDoneErr) {
		esc_flash::ProgressCb cb{ progressCb, nullptr };
		g_verifying = false;
		if (!esc_flash::programImage(bl, g_img, cb)) { g_flashDoneErr = true; return; }
		g_verifying = true;
		if (!esc_flash::verifyImage(bl, g_img, cb))  { g_flashDoneErr = true; return; }
		// AUTO: apply the firmware's own default config (from the HEX's 0x1A00 section) so the ESC's
		// stored settings/name/version match the new firmware instead of keeping stale pre-flash
		// values. g_img.identity is exactly the 512B EEPROM page image (0xFF where undefined).
		if (g_img.hasIdentity) {
			g_writingCfg = true;
			if (!esc_setup::writePage(bl, g_img.identity)) { g_flashDoneErr = true; return; }
			g_cfgWritten = true;
		}
		g_flashDoneOk = true;
		return;
	}

	// idle: keep the bootloader alive while core0 waits for the user / after done
	if (bl.keepAlive()) g_keepAliveOk++;
	else                g_keepAliveNo++;
	delay(50);
}
