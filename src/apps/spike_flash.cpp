// SPDX-License-Identifier: GPL-3.0-or-later
//
// A0 spike: connect to the BLHeli-S bootloader and read the ESC device signature.
// Build:  pio run -e spike_flash -t upload ; pio device monitor -b 115200
//
// The SiLabs bootloader only listens briefly after power-up, so this loops connect()
// while you (re)power the ESC. Signal wire on SIGNAL_PIN, common ground, ESC on its
// own power. EFM8BB21 (LittleBee Spring 30A) should report signature E8 B2.
#include <Arduino.h>
#include "blheli_bl.h"

#define SIGNAL_PIN 10   // shared ESC signal wire (see construction/wiring/)

static blheli_bl::Bootloader bl({ .signalPin = SIGNAL_PIN, .baud = 19200 });

void setup() {
	Serial.begin(115200);
	delay(3000);
	Serial.println(F("[spike_flash] BLHeli-S bootloader connect + device ID"));
	Serial.println(F("Power-cycle the ESC now; streaming BootInit..."));
	bl.begin();
}

void loop() {
	static uint32_t last = 0;
	if (millis() - last < 300) return;
	last = millis();

	if (!bl.connected()) {
		if (!bl.connect()) { Serial.println(F(".")); return; }
		const auto& d = bl.lastDevice();
		Serial.println(F("== CONNECTED =="));
		Serial.printf("signature : %02X %02X  (%s)\n",
			d.signature[0], d.signature[1], d.name ? d.name : "unknown MCU");
		Serial.printf("bootVer   : %u   bootPages: %u\n", d.bootVersion, d.bootPages);
		Serial.print (F("bootInfo  : "));
		for (uint8_t i = 0; i < 8; i++) Serial.printf("%02X ", d.bootInfo[i]);
		Serial.println();
		return;
	}

	// keep the bootloader alive so we can observe a stable link
	Serial.printf("keepAlive : %s\n", bl.keepAlive() ? "alive" : "no response");
}
