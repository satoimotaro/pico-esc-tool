// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// A0 spike (DShot-primed bootloader entry): read the ESC EEPROM over the bootloader,
// replicating exactly how a flight-controller 4-way interface enters it.
//
// WHY THIS SEQUENCE (from analysing Betaflight serial_4way.c + BLHeli_S.asm):
//   BLHeli_S/BlueJay enters its bootloader from the `init_no_signal` routine, which runs
//   when the ESC LOSES a previously-valid signal and then sees the line held HIGH for
//   ~15 ms. Betaflight passthrough requires DShot configured for exactly this reason: it
//   feeds DShot, then `motorDisable()`s (signal loss) and holds the pin high, and the ESC
//   jumps to its bootloader. No power-cycle. Our earlier spikes never fed a signal first,
//   so the ESC never took the signal-loss path. This build does it properly:
//     core0: [1] prime ESC with valid DShot (throttle 0) -> [2] stop DShot + hold HIGH 40ms
//     core1: [3] stream BootInit + read the reply (bit-bang; kept off core0 for USB safety)
//   Auto-retries the whole cycle until it connects. Build/flash then `pio device monitor`.
#include <Arduino.h>
#include <PIO_DShot.h>
#include <hardware/structs/sio.h>
#include "blheli_bl.h"
#include "esc_setup.h"

#define SIGNAL_PIN   10
#define DSHOT_KBAUD  600
#define PRIME_MS     500     // feed valid DShot this long so the ESC locks onto it
#define HIGH_HOLD_MS 1000  // CONFIRMED: 200ms fails, 600ms connects; hold high past the ESC's
                           // beep melody until it reaches init_no_signal. 1000ms = margin.

static BidirDShotX1*        esc = nullptr;                       // core0 only
static blheli_bl::Bootloader bl({ .signalPin = SIGNAL_PIN, .baud = 0 });  // core1 only

// --- shared: core0 drives the sequence, core1 does the bit-bang, both via volatiles ---
volatile bool     g_startEntry = false;   // core0 -> core1: line is HIGH, do BootInit now
volatile bool     g_connected  = false;
volatile bool     g_done       = false;   // core1 -> core0: this attempt finished
volatile bool     g_eepromOk   = false;
volatile int      g_bestN      = 0;
volatile uint8_t  g_raw[8]      = {0};
volatile uint32_t g_attempts    = 0;
volatile uint32_t g_edgesMax    = 0;   // raw falling edges on line after BootInit (ESC driving?)
volatile uint32_t g_lowMax      = 0;
// Single-command probes (see PROBES[] labels in core0). Each sends ONE frame after a 20ms
// idle gap so there's no back-to-back desync; ACK is the raw reply byte (0xFF = no reply).
static const int  NPROBE     = 6;
volatile uint8_t  g_pAck[6]   = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}; // raw reply byte per probe
volatile uint8_t  g_pGot[6]   = {0};   // 1 = a byte was received, 0 = start-bit timeout (no reply)
volatile uint16_t g_pFirst[6] = {0};   // us from end-of-TX to the reply's start bit (0 = none)
volatile uint8_t  g_app0[32]  = {0};   // first 32 bytes of APP flash @0x0000 (identifies the firmware)
volatile bool     g_appOk     = false;
static esc_setup::Settings g_settings;

// =================== core0: DShot priming + sequencing + Serial ===================
void setup() {
	Serial.begin(115200);
}

void loop() {
	static int      phase = 0;
	static uint32_t t0 = 0;
	static uint32_t cycle = 0;
	static bool     banner = false, connMsg = false, doneMsg = false;

	if (!banner && millis() > 800) {
		banner = true;
		Serial.println(F("[spike_setup] DShot-primed bootloader entry (no power-cycle needed)"));
	}
	if (!banner) return;

	switch (phase) {
	case 0:  // start a cycle: create DShot, begin priming
		cycle++;
		Serial.printf("\n== cycle %lu ==\n[1] priming ESC with DShot throttle 0 (%dms)...\n",
			(unsigned long)cycle, PRIME_MS);
		esc = new BidirDShotX1(SIGNAL_PIN, DSHOT_KBAUD);
		t0 = millis();
		phase = 1;
		break;

	case 1:  // keep valid DShot frames flowing so the ESC locks on
		esc->sendThrottle(0);
		delayMicroseconds(300);
		if (millis() - t0 >= PRIME_MS) {
			// CONFIRMED on HW: 200ms fails, 600ms connects (ESC needs time past its beep
			// melody to reach init_no_signal). 1000ms for margin.
			uint16_t hold = HIGH_HOLD_MS;
			delete esc; esc = nullptr;                    // stop DShot => signal loss
			pinMode(SIGNAL_PIN, OUTPUT);                  // immediately hold the line HIGH
			digitalWrite(SIGNAL_PIN, HIGH);               // STRONG push-pull HIGH for the whole hold
			Serial.printf("[2] DShot stopped; holding HIGH %ums (signal-loss -> BL entry)\n", hold);
			delay(hold);
			Serial.println(F("[3] core1 streaming BootInit, listening for E8 B2..."));
			connMsg = doneMsg = false;
			g_connected = g_done = g_eepromOk = false;
			g_bestN = 0; g_attempts = 0;
			g_startEntry = true;                          // hand the pin to core1
			phase = 2;
		}
		break;

	case 2:  // report what core1 found
		if (g_connected && !connMsg) {
			connMsg = true;
			Serial.printf("== CONNECTED (%lu probes) ==\nsignature: %02X %02X  bootInfo: ",
				(unsigned long)g_attempts, g_raw[4], g_raw[5]);
			for (int i = 0; i < 8; i++) Serial.printf("%02X ", g_raw[i]);
			Serial.println();
		}
		if (g_done && !doneMsg) {
			doneMsg = true;
			{
				static const char* L[6] = {
					"SETADDR +crc  #1 (expect 30)", "SETADDR +crc  #2 (expect 30)",
					"SETADDR +crc  #3 (expect 30)", "KEEPALIVE +crc   (expect C1)",
					"SETADDR +crc  #4 (expect 30)", "KEEPALIVE +crc   (expect C1)" };
				Serial.println(F("consecutive VALID +crc probes (1 frame each, 20ms gap):"));
				for (int i = 0; i < 6; i++) {
					Serial.printf("  %-30s -> ", L[i]);
					if (g_pGot[i]) Serial.printf("%02X", g_pAck[i]);
					else           Serial.print("--");        // -- = no reply within 250ms
					Serial.printf("   (%uus)\n", g_pFirst[i]);
				}
				Serial.println(F("  key: all 30/C1 => consecutive cmds OK (bug is in readBuf); drops after #1 => turnaround"));
			}
			Serial.print(F("app @0x0000 : "));
			if (g_appOk) { for (int i = 0; i < 32; i++) Serial.printf("%02X ", g_app0[i]); Serial.println(); }
			else Serial.println(F("(read failed)"));
			Serial.println(F("  (BLHeli-S 16.7 J_H_25 app starts: 02 19 FD 02 03 1C ...)"));
			if (g_connected && g_eepromOk) {
				esc_setup::print(g_settings, Serial);
				Serial.printf("raw @0x1A00 (%u bytes, CRC-verified):\n", (unsigned)g_settings.rawLen);
				for (int i = 0; i < (int)g_settings.rawLen; i++) {
					Serial.printf("%02X ", g_settings.raw[i]);
					if ((i & 15) == 15) Serial.printf("  ; +0x%02X\n", i - 15);
				}
				Serial.println();
				Serial.println(F("-- SUCCESS --"));
				phase = 3;                                // stop; leave results on screen
			} else if (g_connected) {
				Serial.println(F("EEPROM read FAILED (link ok, framing off)"));
				phase = 3;
			} else {
				Serial.printf("no reply (best=%d bytes). LINE ACTIVITY after BootInit: edges=%lu low=%lu -> %s\n",
					g_bestN, (unsigned long)g_edgesMax, (unsigned long)g_lowMax,
					g_edgesMax > 0 ? "ESC IS DRIVING THE LINE => it's in the BL, RX decode is the bug"
					               : "line stayed HIGH => ESC silent => NOT entering the bootloader");
				g_startEntry = false;                     // release pin from core1
				delay(200);
				phase = 0;                                // re-prime and try again
			}
		}
		break;

	default:  // phase 3: done
		break;
	}
}

// =================== core1: bootloader bit-bang after signal loss ===================
void setup1() {}

void loop1() {
	if (!g_startEntry || g_done) { delay(2); return; }

	// The ESC just lost DShot and has been held HIGH >15ms by core0 -> it should now be
	// sitting in its bootloader. Stream BootInit and grab the "471"+signature reply.
	bl.begin();
	// (diagnostic) first, one raw line-activity probe: is the ESC driving the line AT ALL
	// after BootInit? edges>0 => it IS in the bootloader (RX decode would be the bug);
	// edges==0 => ESC silent => not entering the bootloader even with DShot priming.
	{
		uint32_t e = 0, lo = 0, tot = 0;
		bl.probeReplyActivity(25, e, lo, tot);
		g_edgesMax = e; g_lowMax = lo;
	}
	uint32_t t0 = millis();
	while (millis() - t0 < 500 && !g_connected) {
		uint8_t raw[8] = {0};
		int n = bl.connectRawProbe(raw, 15);
		g_attempts++;
		for (int i = 0; i < 8; i++) g_raw[i] = raw[i];
		if (n > g_bestN) g_bestN = n;
		if (n >= 8 && raw[0] == '4' && raw[1] == '7' && raw[2] == '1') {
			if (bl.connect()) g_connected = true;
		}
	}

	if (g_connected) {
		// DECISIVE FRAMING TEST with an UNAMBIGUOUS command. SET_ADDRESS(0x1A00) must reply
		// br_SUCCESS 0x30 when the frame+CRC are right; a bad/missing CRC replies br_ERRORCRC
		// 0xC2. Each probe is a SINGLE frame after a 20ms idle gap (no back-to-back desync), so
		// this isolates CRC-value correctness. keepAlive (FD 00) probes kept for comparison.
		// ALL VALID +crc frames (no malformed frame that would desync a timeout-less parser).
		// Tests whether CONSECUTIVE valid commands work — the thing readEeprom needs.
		static const uint8_t SA[4] = { 0xFF, 0x00, 0x1A, 0x00 };   // SET_ADDRESS 0x1A00
		static const uint8_t KA[2] = { 0xFD, 0x00 };               // keepAlive
		struct P { const uint8_t* d; uint16_t n; bool crc; };
		const P probes[NPROBE] = {
			{ SA, 4, true },   // 0: SETADDR +crc     -> expect 30
			{ SA, 4, true },   // 1: SETADDR +crc     -> expect 30 (2nd consecutive)
			{ SA, 4, true },   // 2: SETADDR +crc     -> expect 30 (3rd)
			{ KA, 2, true },   // 3: KEEPALIVE +crc   -> expect C1 (or 30)
			{ SA, 4, true },   // 4: SETADDR +crc     -> expect 30
			{ KA, 2, true },   // 5: KEEPALIVE +crc   -> expect C1
		};
		for (int i = 0; i < NPROBE; i++) {
			uint8_t ack = 0xFF; int got = 0; uint32_t first = 0;
			bl.cmdProbe(probes[i].d, probes[i].n, probes[i].crc, 20, 250, ack, got, first);
			g_pAck[i]   = ack;
			g_pGot[i]   = (uint8_t)got;
			g_pFirst[i] = (uint16_t)(first > 65535 ? 65535 : first);
		}
		esc_setup::Settings s;
		g_eepromOk = esc_setup::read(bl, s);
		g_settings = s;
		// read the start of APP flash to identify the RUNNING firmware (EEPROM version is stale)
		uint8_t a0[32];
		if (bl.readFlash(0x0000, a0, 32)) { for (int i = 0; i < 32; i++) g_app0[i] = a0[i]; g_appOk = true; }
	}
	g_done = true;   // tell core0 this attempt is finished (connected or not)
}
