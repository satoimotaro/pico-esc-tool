// SPDX-License-Identifier: GPL-3.0-or-later
//
// A0 spike: connect to the bootloader and dump the ESC EEPROM parameter block.
// Build:  pio run -e spike_setup -t upload ; pio device monitor -b 115200
#include <Arduino.h>
#include "blheli_bl.h"
#include "esc_setup.h"

#define SIGNAL_PIN 10

static blheli_bl::Bootloader bl({ .signalPin = SIGNAL_PIN, .baud = 0 });

void setup() {
	Serial.begin(115200);
	delay(3000);
	Serial.println(F("[spike_setup] BLHeli-S EEPROM read/dump"));
	bl.begin();
}

void loop() {
	static bool done = false;
	if (done) return;

	if (!bl.connected()) {
		Serial.println(F("connecting... (power-cycle the ESC now)"));
		if (!bl.connect()) { delay(500); return; }
		Serial.println(F("connected!"));
	}

	esc_setup::Settings s;
	if (esc_setup::read(bl, s)) {
		esc_setup::print(s, Serial);
	} else {
		Serial.println(F("EEPROM read failed (command not implemented yet)"));
	}
	done = true;
}
