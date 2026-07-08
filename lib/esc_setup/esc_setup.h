// SPDX-License-Identifier: GPL-3.0-or-later
//
// esc_setup — read/decode/encode the BLHeli-S EEPROM configuration block.
// Uses blheli_bl for 1-wire access. Offsets & fields: see EEPROM.md [TODO:proto].
#pragma once
#include <Arduino.h>
#include "blheli_bl.h"

namespace esc_setup {

// Decoded BLHeli-S settings (subset; extend from EEPROM.md as fields are confirmed).
struct Settings {
	bool     valid = false;
	uint8_t  layoutRevision = 0;      // EEPROM layout/version byte
	char     layoutName[16] = {0};    // e.g. "#S_H_50#"
	char     mcuName[16]    = {0};
	// common params (encodings per EEPROM.md) [TODO:proto]
	uint8_t  motorDirection        = 0;  // normal / reversed / bidirectional
	uint8_t  ppmMinThrottle        = 0;
	uint8_t  ppmMaxThrottle        = 0;
	uint8_t  ppmCenterThrottle     = 0;
	uint8_t  beepStrength          = 0;
	uint8_t  beaconStrength        = 0;
	uint8_t  beaconDelay           = 0;
	uint8_t  motorTiming           = 0;
	uint8_t  pwmFrequency          = 0;
	uint8_t  demagCompensation     = 0;
	uint8_t  temperatureProtection = 0;
	uint8_t  lowVoltageProtection  = 0;
	uint8_t  brakeOnStop           = 0;
	uint8_t  startupPower          = 0;
	// full raw block kept for safe read-modify-write round trips
	uint8_t  raw[256] = {0};
	uint16_t rawLen = 0;
};

// EEPROM parameter block base address on SiLabs BLHeli-S [TODO:proto confirm].
constexpr uint16_t kEepromAddr = 0x1A00;   // placeholder
constexpr uint16_t kEepromLen  = 0x70;     // placeholder

bool  read (blheli_bl::Bootloader& bl, Settings& out);          // read + decode
bool  write(blheli_bl::Bootloader& bl, const Settings& in);     // encode + write (A1)
void  print(const Settings& s, Stream& out);                    // human-readable dump

} // namespace esc_setup
