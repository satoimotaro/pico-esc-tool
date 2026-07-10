// SPDX-License-Identifier: GPL-3.0-or-later
//
// A0 spike (link test): connect to the BLHeli-S bootloader, read the device signature,
// and hold the link with keep-alive. Uses the SAME DShot-primed signal-loss entry as
// spike_setup (see that file / bootloader-entry notes): the ESC must be fed a valid
// signal, then lose it with the line held HIGH, to jump into `init_no_signal`'s BL.
//   core0: [1] prime ESC with DShot -> [2] stop DShot + hold HIGH 40ms (signal loss)
//   core1: [3] stream BootInit; on connect report signature + repeated keepAlive
// EFM8BB21 (LittleBee Spring 30A) should report signature E8 B2. No power-cycle needed.
#include <Arduino.h>
#include <PIO_DShot.h>
#include <hardware/structs/sio.h>
#include "blheli_bl.h"

#define SIGNAL_PIN   10
#define DSHOT_KBAUD  600
#define PRIME_MS     500
#define HIGH_HOLD_MS 200   // must exceed RC-timeout (~100ms) + init_no_signal's 15ms all-high check; see PROTOCOL.md §B

static BidirDShotX1*         esc = nullptr;                       // core0 only
static blheli_bl::Bootloader bl({ .signalPin = SIGNAL_PIN, .baud = 0 });  // core1 only

// --- shared state (core0 sequences, core1 bit-bangs) ---
volatile bool     g_startEntry = false;
volatile bool     g_connected  = false;
volatile bool     g_done       = false;   // core1 -> core0: this attempt finished
volatile int      g_bestN      = 0;
volatile uint8_t  g_raw[8]      = {0};
volatile uint32_t g_attempts    = 0;
// device info snapshot (filled by core1 on connect)
volatile uint8_t  g_sig[2]      = {0, 0};
volatile uint8_t  g_bootVer     = 0;
volatile uint8_t  g_bootPages   = 0;
volatile uint32_t g_keepAliveOk = 0;
volatile uint32_t g_keepAliveNo = 0;

void setup() { Serial.begin(115200); }

// =================== core0: DShot priming + sequencing + Serial ===================
void loop() {
	static int      phase = 0;
	static uint32_t t0 = 0, cycle = 0, lastKa = 0;
	static bool     banner = false, connMsg = false, doneMsg = false;

	if (!banner && millis() > 800) {
		banner = true;
		Serial.println(F("[spike_flash] link test: DShot-primed bootloader connect + keepAlive"));
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
			connMsg = doneMsg = false;
			g_connected = g_done = false;
			g_bestN = 0; g_attempts = 0;
			g_startEntry = true;
			phase = 2;
		}
		break;

	case 2:  // wait for connect result
		if (g_connected && !connMsg) {
			connMsg = true;
			Serial.printf("== CONNECTED (%lu probes) ==\nsignature : %02X %02X\n",
				(unsigned long)g_attempts, g_sig[0], g_sig[1]);
			Serial.printf("bootVer   : %u   bootPages: %u\nbootInfo  : ",
				g_bootVer, g_bootPages);
			for (int i = 0; i < 8; i++) Serial.printf("%02X ", g_raw[i]);
			Serial.println(F("\n(streaming keepAlive; link should stay 'alive')"));
			lastKa = millis();
			phase = 3;  // stay connected, report keepAlive
		}
		if (g_done && !g_connected && !doneMsg) {
			doneMsg = true;
			Serial.printf("no reply (best=%d bytes). Retrying full cycle...\n", g_bestN);
			g_startEntry = false;
			delay(200);
			phase = 0;
		}
		break;

	case 3:  // connected: report rolling keepAlive stats
		if (millis() - lastKa > 1000) {
			lastKa = millis();
			Serial.printf("keepAlive : %lu alive / %lu no-response\n",
				(unsigned long)g_keepAliveOk, (unsigned long)g_keepAliveNo);
		}
		break;
	}
}

// =================== core1: bootloader bit-bang ===================
void setup1() {}

void loop1() {
	if (!g_startEntry) { delay(2); return; }

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
					g_connected = true;
				}
			}
		}
		g_done = true;
		return;
	}

	// connected: keep the bootloader alive so we can observe a stable link
	if (g_connected) {
		if (bl.keepAlive()) g_keepAliveOk++;
		else                g_keepAliveNo++;
		delay(50);
	}
}
