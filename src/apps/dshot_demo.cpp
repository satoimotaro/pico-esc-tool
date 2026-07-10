// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// dshot_demo — standalone bidirectional-DShot drive demo (the `pico` / `picow` envs).
// -----------------------------------------------------------------------------------
// Minimal single-ESC driver over bidirectional DShot, from a PC over USB-C CDC serial: spin the
// motor and read RPM (+ EDT: voltage/current/temp/stress). A small reference/bring-up app on the
// same level as the spikes — the full tool is `esc_tool` (config/flash + drive + Wi-Fi web UI).
// Kept as the default entry for the plain `pico` (non-W) board, which can't build esc_tool's Wi-Fi.
//
// Built on pico-bidir-dshot (Bastian2001, GPL-3.0): DShot TX, bidirectional eRPM, and Extended
// DShot Telemetry decoding on the RP2040 PIO. Adapted from that library's advanced EDT example.
//
// Host protocol (Serial Monitor, newline mode):
//   T<0-2000>  set throttle (0 = stop)
//   C<0-47>    send special command (only when stopped); e.g. C3 = beacon 3
//   E          enable Extended DShot Telemetry (== C13)
//   A          arm    (permit throttle)
//   D          disarm (force throttle 0)
//   ?          print status header again
//
// Pins / motor poles come from esc_config.h (shared with the tool); this demo drives ESC 0's pin.

#include <Arduino.h>
#include <PIO_DShot.h>
#include "esc_config.h"

static const uint8_t kDemoPins[] = ESC_SIGNAL_PINS;
#define SIGNAL_PIN   kDemoPins[0]     // ESC 0's signal wire (see esc_config.h, construction/wiring/)
#define MOTOR_POLES  ESC_MOTOR_POLES  // magnet poles of the target motor
#define DSHOT_KBAUD  ESC_DSHOT_KBAUD  // DShot bitrate

BidirDShotX1 *esc = nullptr;

static uint16_t throttle = 0;
static bool     armed    = false;

// latest telemetry
static uint32_t rpm = 0, current = 0, temp = 0, stress = 0, lastStatus = 0;
static float    voltage = 0.0f;

static void printHeader() {
	Serial.println("Thrott\tRPM\tVolt\tAmp\tTemp\tStress\tStatus  (Tn/Cn/E/A/D/?)");
}

static void sendSpecialCommand(uint16_t cmd) {
	// special commands must be sent while stopped; repeat for reliability
	for (int i = 0; i < 10; i++) {
		esc->sendRaw11Bit(cmd);
		delayMicroseconds(200);
	}
}

void setup() {
	Serial.begin(115200);           // USB-C CDC
	delay(3000);                    // give the host time to attach
	esc = new BidirDShotX1(SIGNAL_PIN, DSHOT_KBAUD);
	if (esc->initError()) {
		Serial.println("ERR: DShot init failed (see DSHOT_DEBUG in the lib)");
	}
	printHeader();
}

static void pollTelemetry() {
	uint32_t v = 0;
	switch (esc->getTelemetryPacket(&v)) {
	case BidirDshotTelemetryType::ERPM:        rpm     = v / (MOTOR_POLES / 2); break;
	case BidirDshotTelemetryType::VOLTAGE:     voltage = (float)v / 4.0f;       break; // 250mV steps
	case BidirDshotTelemetryType::CURRENT:     current = v;                     break; // 1A steps
	case BidirDshotTelemetryType::TEMPERATURE: temp    = v;                     break; // °C
	case BidirDshotTelemetryType::STRESS:      stress  = v & ESC_STATUS_MAX_STRESS_MASK; break;
	case BidirDshotTelemetryType::STATUS:      lastStatus = v;                  break;
	default: break; // NO_PACKET / CHECKSUM_ERROR / DEBUG — ignore
	}
}

static void handleHostCommand() {
	if (!Serial.available()) return;
	delay(3);                       // let the rest of the line arrive
	int c = Serial.read();
	if (c < 0) return;
	char cmd = toupper((char)c);    // accept lower-case commands too
	String s;
	while (Serial.available()) s += (char)Serial.read();
	s.trim();                       // strip CR/LF/space so toInt() is clean
	uint32_t value = s.toInt();

	switch (cmd) {
	case 'T':
		throttle = armed ? (uint16_t)value : 0;
		Serial.printf("> T%u -> throttle=%u (armed=%d)\n", value, throttle, armed);
		break;
	case 'C':
		if (throttle == 0) { sendSpecialCommand(value); Serial.printf("> C%u sent\n", value); }
		else               Serial.println("> C ignored: stop first (T0)");
		break;
	case 'E':
		if (throttle == 0) { sendSpecialCommand(DSHOT_CMD_EXTENDED_TELEMETRY_ENABLE); Serial.println("> EDT enabled"); }
		else               Serial.println("> E ignored: stop first (T0)");
		break;
	case 'A': armed = true;              Serial.println("> ARMED");    break;
	case 'D': armed = false; throttle = 0; Serial.println("> DISARMED"); break;
	case '?': printHeader();             break;
	case '\r': case '\n': case ' ': case '\t': break;   // ignore stray whitespace
	default:  Serial.printf("> unknown cmd '%c' (0x%02X)\n", cmd, cmd); break; // do NOT zero throttle
	}
}

void loop() {
	delayMicroseconds(200);         // spacing between DShot frames
	pollTelemetry();
	handleHostCommand();

	// periodic telemetry print
	static uint32_t last = 0;
	if (millis() - last > 100) {
		last = millis();
		Serial.printf("%u\t%u\t%.2f\t%u\t%u\t%u\t", throttle, rpm, voltage, current, temp, stress);
		Serial.println(lastStatus, BIN);
	}

	esc->sendThrottle(throttle);    // must be called regularly (>500Hz) to keep the ESC alive
}
