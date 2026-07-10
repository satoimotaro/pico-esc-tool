// SPDX-License-Identifier: GPL-3.0-or-later
//
// Minimal bootloader-entry TEST (no DShot): hammer BootInit continuously; the USER
// power-cycles the ESC while this runs. Tests the "SiLabs BL listens only briefly at
// reset" hypothesis — if we're streaming BootInit at the instant of power-on, the BL
// should catch it and stay resident instead of jumping to the app.
//   core0: Serial banner + report best/attempts + signature on connect
//   core1: continuous connectRawProbe; on "471"+sig -> connect() -> keepAlive
// EFM8BB21 should report signature E8 B2.
//
// HOW TO USE: flash this, open the monitor, THEN power-cycle the ESC (cut & restore ESC
// power) a few times. Watch for "== CONNECTED ==". No DShot priming happens here at all.
#include <Arduino.h>
#include "blheli_bl.h"

#define SIGNAL_PIN 10

static blheli_bl::Bootloader bl({ .signalPin = SIGNAL_PIN, .baud = 0 });  // core1 only

volatile bool     g_connected  = false;
volatile int      g_bestN      = 0;
volatile uint8_t  g_raw[8]      = {0};
volatile uint32_t g_attempts    = 0;
volatile uint8_t  g_sig[2]      = {0, 0};
volatile uint8_t  g_bootVer     = 0;
volatile uint8_t  g_bootPages   = 0;
volatile uint32_t g_keepAliveOk = 0;
volatile uint32_t g_keepAliveNo = 0;
// raw line-activity diagnostic (core1 -> core0)
volatile uint32_t g_edges = 0;      // falling edges seen after BootInit (ESC driving = >0)
volatile uint32_t g_low   = 0;      // samples read LOW
volatile uint32_t g_total = 0;      // total samples in the window
volatile uint32_t g_edgesMax = 0;   // best (max) edges across all probes

void setup() { Serial.begin(115200); }

// =================== core0: Serial only ===================
void loop() {
	static bool     banner = false, connMsg = false;
	static uint32_t last = 0;

	if (!banner && millis() > 800) {
		banner = true;
		Serial.println(F("[spike_boot] hammering BootInit (no DShot). POWER-CYCLE the ESC now, repeatedly."));
		Serial.println(F("Watch for signature E8 B2. Ctrl-C the monitor to stop."));
		last = millis();
	}
	if (!banner) return;

	if (g_connected && !connMsg) {
		connMsg = true;
		Serial.printf("\n== CONNECTED (%lu probes) ==\nsignature : %02X %02X   bootVer: %u  bootPages: %u\nbootInfo  : ",
			(unsigned long)g_attempts, g_sig[0], g_sig[1], g_bootVer, g_bootPages);
		for (int i = 0; i < 8; i++) Serial.printf("%02X ", g_raw[i]);
		Serial.println();
	}
	if (!g_connected && millis() - last > 1000) {
		last = millis();
		Serial.printf("probing... %lu tries, bytes=%d  | LINE ACTIVITY after BootInit: edges=%lu (max %lu), low=%lu/%lu samples\n",
			(unsigned long)g_attempts, g_bestN,
			(unsigned long)g_edges, (unsigned long)g_edgesMax, (unsigned long)g_low, (unsigned long)g_total);
		if (g_edgesMax > 0)
			Serial.println(F("  >> ESC IS DRIVING THE LINE (edges>0) => it's in the bootloader; RX decode is the bug."));
		else
			Serial.println(F("  >> line stayed HIGH (edges=0) => ESC sent NOTHING => not entering the bootloader."));
	}
	if (g_connected && millis() - last > 1000) {
		last = millis();
		Serial.printf("keepAlive : %lu ok / %lu no\n",
			(unsigned long)g_keepAliveOk, (unsigned long)g_keepAliveNo);
	}
}

// =================== core1: hold-HIGH then BootInit probe (repeat) ===================
// Each iteration holds GP10 solidly HIGH for HOLD_MS (so if the user power-cycles the ESC
// during this window, the running app times out -> init_no_signal -> 15ms high check ->
// ljmp 1C00h), THEN sends one BootInit and reads the reply. No DShot / PIO anywhere.
#define HOLD_MS 150

void setup1() { bl.begin(); }

void loop1() {
	if (!g_connected) {
		bl.holdIdleHigh(HOLD_MS);            // solid push-pull HIGH, glitch-free
		// (1) raw activity probe: send BootInit, watch the line for 25ms (no UART decode)
		uint32_t e = 0, lo = 0, tot = 0;
		bl.probeReplyActivity(25, e, lo, tot);
		g_edges = e; g_low = lo; g_total = tot;
		if (e > g_edgesMax) g_edgesMax = e;
		// (2) normal decode probe
		uint8_t raw[8] = {0};
		int n = bl.connectRawProbe(raw, 20); // sends BootInit (line back to idle-high), reads 8
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
		return;
	}
	if (bl.keepAlive()) g_keepAliveOk++;
	else                g_keepAliveNo++;
	delay(50);
}
