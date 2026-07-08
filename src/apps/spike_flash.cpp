// SPDX-License-Identifier: GPL-3.0-or-later
//
// A0 spike: connect to the BLHeli-S bootloader and read the ESC device signature.
// Build:  pio run -e spike_flash -t upload ; pio device monitor -b 115200
//
// Because the SiLabs bootloader only listens briefly after power-up, this loops
// connect() while you (re)power the ESC. On success it prints the device info.
#include <Arduino.h>
#include "blheli_bl.h"

#define SIGNAL_PIN 10   // shared ESC signal wire (see construction/wiring/)

static blheli_bl::Bootloader bl({ .signalPin = SIGNAL_PIN, .baud = 0 });

void setup() {
	Serial.begin(115200);
	delay(3000);
	Serial.println(F("[spike_flash] BLHeli-S bootloader connect + device ID"));
	if (!bl.begin()) {
		Serial.println(F("begin() failed — transport not implemented yet (see PROTOCOL.md)"));
	}
}

void loop() {
	static uint32_t last = 0;
	if (millis() - last < 500) return;
	last = millis();

	if (!bl.connected()) {
		Serial.println(F("connecting... (power-cycle the ESC now)"));
		if (!bl.connect()) return;   // stub returns false until PROTOCOL.md is filled in
		Serial.println(F("connected!"));
	}

	blheli_bl::DeviceInfo info;
	if (bl.readDeviceInfo(info) && info.valid) {
		Serial.printf("sig=%02X %02X  name=%s\n",
			info.signature[0], info.signature[1], info.name ? info.name : "(unknown)");
	} else {
		Serial.println(F("device info unavailable (command not implemented yet)"));
	}
}
