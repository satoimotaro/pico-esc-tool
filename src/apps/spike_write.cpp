// SPDX-License-Identifier: GPL-3.0-or-later
//
// A1 spike: CONFIG WRITE (change one setting) via the BLHeli-S bootloader, with a hard safety
// gate. Same DShot-primed bootloader entry as spike_setup, then:
//   1. read the whole 512-B config flash page (0x1A00),
//   2. copy it and change ONE byte (default: BEEP_STRENGTH @0x1B — the beep/startup volume),
//   3. DRY-RUN by default: print the before/after + NOTHING is written,
//   4. only if ARM_WRITE=1: erase the page + write it back + read-back-verify (esc_setup::writePage).
// The change is audible on the next ESC power-up and is fully reversible (config is re-writable).
//
// ⚠ ARM_WRITE=1 ERASES + WRITES the ESC's config flash. Leave it 0 until a write is intended.
#include <Arduino.h>
#include <PIO_DShot.h>
#include <hardware/structs/sio.h>
#include <string.h>
#include "blheli_bl.h"
#include "esc_setup.h"

#define SIGNAL_PIN   10
#define DSHOT_KBAUD  600
#define PRIME_MS     500
#define HIGH_HOLD_MS 1000

#define ARM_WRITE    0        // 0 = DRY-RUN (read + show diff, write nothing); 1 = real erase+write
#define TARGET_OFF   0x1B     // byte offset within the 0x1A00 block: 0x1B = BEEP_STRENGTH
#define TARGET_NEW   0x28     // BEEP_STRENGTH 0x28=40 (default). (0x60=96 was audibly too loud.)

static BidirDShotX1*         esc = nullptr;
static blheli_bl::Bootloader bl({ .signalPin = SIGNAL_PIN, .baud = 0 });

volatile bool     g_startEntry = false;
volatile bool     g_connected  = false;
volatile bool     g_done       = false;
volatile uint32_t g_attempts   = 0;
volatile uint8_t  g_raw[8]      = {0};
// results (core1 -> core0)
volatile bool     g_readOk   = false;   // page read succeeded
volatile bool     g_wrote    = false;   // a real write was attempted (ARM_WRITE)
volatile bool     g_writeOk  = false;   // write + read-back-verify passed
volatile uint8_t  g_before   = 0;       // target byte before
volatile uint8_t  g_after    = 0;       // target byte in the buffer we would/did write
volatile uint8_t  g_readback = 0;       // target byte re-read after a real write
volatile uint8_t  g_ctx[16]  = {0};     // 16 bytes around the target (before)

void setup() { Serial.begin(115200); }

void loop() {
	static int phase = 0;
	static uint32_t t0 = 0, cycle = 0;
	static bool banner = false, connMsg = false, doneMsg = false;

	if (!banner && millis() > 800) {
		banner = true;
		Serial.printf("[spike_write] config write, ARM_WRITE=%d (0=dry-run). target off=0x%02X -> 0x%02X\n",
			ARM_WRITE, TARGET_OFF, TARGET_NEW);
	}
	if (!banner) return;

	switch (phase) {
	case 0:
		cycle++;
		Serial.printf("\n== cycle %lu ==\n[1] priming DShot (%dms)...\n", (unsigned long)cycle, PRIME_MS);
		esc = new BidirDShotX1(SIGNAL_PIN, DSHOT_KBAUD);
		t0 = millis(); phase = 1;
		break;
	case 1:
		esc->sendThrottle(0);
		delayMicroseconds(300);
		if (millis() - t0 >= PRIME_MS) {
			delete esc; esc = nullptr;
			pinMode(SIGNAL_PIN, OUTPUT); digitalWrite(SIGNAL_PIN, HIGH);
			Serial.printf("[2] holding HIGH %dms (signal-loss -> BL)\n", HIGH_HOLD_MS);
			delay(HIGH_HOLD_MS);
			Serial.println(F("[3] core1: BootInit + read/modify config..."));
			connMsg = doneMsg = false;
			g_connected = g_done = g_readOk = g_wrote = g_writeOk = false;
			g_attempts = 0;
			g_startEntry = true;
			phase = 2;
		}
		break;
	case 2:
		if (g_connected && !connMsg) {
			connMsg = true;
			Serial.printf("== CONNECTED == sig %02X %02X\n", g_raw[4], g_raw[5]);
		}
		if (g_done && !doneMsg) {
			doneMsg = true;
			if (!g_connected) {
				Serial.println(F("connect failed; re-priming..."));
				g_startEntry = false; delay(200); phase = 0; break;
			}
			if (!g_readOk) {
				Serial.println(F("page READ failed (cannot proceed to write)"));
				phase = 3; break;
			}
			Serial.print(F("context @0x1A10..0x1A1F (before): "));
			for (int i = 0; i < 16; i++) Serial.printf("%02X ", g_ctx[i]);
			Serial.println();
			Serial.printf("target 0x1A%02X (BEEP_STRENGTH): before=0x%02X (%u)  ->  new=0x%02X (%u)\n",
				TARGET_OFF, g_before, g_before, g_after, g_after);
			if (!g_wrote) {
				Serial.println(F("DRY-RUN: nothing written. Set ARM_WRITE=1 to erase+write this change."));
			} else if (g_writeOk) {
				Serial.printf("WRITE OK + verified: read-back target=0x%02X (%u). Power-cycle the ESC to hear it.\n",
					g_readback, g_readback);
			} else {
				Serial.println(F("WRITE FAILED (erase/write/verify) — config may be partially written; re-check via spike_setup."));
			}
			phase = 3;
		}
		break;
	default: break;
	}
}

void setup1() {}

void loop1() {
	if (!g_startEntry || g_done) { delay(2); return; }
	bl.begin();
	uint32_t t0 = millis();
	while (millis() - t0 < 500 && !g_connected) {
		uint8_t raw[8] = {0};
		int n = bl.connectRawProbe(raw, 15);
		g_attempts++;
		for (int i = 0; i < 8; i++) g_raw[i] = raw[i];
		if (n >= 8 && raw[0] == '4' && raw[1] == '7' && raw[2] == '1') {
			if (bl.connect()) g_connected = true;
		}
	}
	if (g_connected) {
		static uint8_t page[esc_setup::kPageLen];
		if (esc_setup::readPage(bl, page)) {
			g_readOk = true;
			for (int i = 0; i < 16; i++) g_ctx[i] = page[0x10 + i];   // 0x1A10..0x1A1F
			g_before = page[TARGET_OFF];
			static uint8_t mod[esc_setup::kPageLen];
			memcpy(mod, page, esc_setup::kPageLen);
			mod[TARGET_OFF] = TARGET_NEW;
			g_after = mod[TARGET_OFF];
#if ARM_WRITE
			g_wrote = true;
			if (esc_setup::writePage(bl, mod)) {
				uint8_t rb[esc_setup::kPageLen];
				if (esc_setup::readPage(bl, rb)) { g_readback = rb[TARGET_OFF]; g_writeOk = (rb[TARGET_OFF] == TARGET_NEW); }
			}
#endif
		}
	}
	g_done = true;
}
