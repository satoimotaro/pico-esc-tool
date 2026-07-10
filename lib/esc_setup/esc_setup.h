// SPDX-License-Identifier: GPL-3.0-or-later
//
// esc_setup — read/decode/encode the BLHeli-S EEPROM configuration block.
// Uses blheli_bl for 1-wire access. Offsets & encodings: see EEPROM.md
// (layout revision 33; base 0x1A00; 112 bytes). Verified vs esc-configurator +
// BLHeli_S.asm. Decode is read-only for A0; encode/write is Phase A1.
#pragma once
#include <Arduino.h>
#include "blheli_bl.h"

namespace esc_setup {

// EEPROM parameter block on SiLabs BLHeli-S (EFM8BB1/BB2).
constexpr uint16_t kEepromAddr = 0x1A00;
// BlueJay's config block is 255 bytes (esc-configurator Bluejay/eeprom.js LAYOUT_SIZE=0xFF) —
// it includes the 128-byte STARTUP_MELODY at 0x70. (Plain BLHeli-S only uses 0x70=112.)
constexpr uint16_t kEepromLen  = 0xFF;   // 255 bytes (covers name 0x60 + melody 0x70..0xEF + wait 0xF0)
constexpr uint16_t kMelodyOff  = 0x70;   // STARTUP_MELODY offset within the block
constexpr uint16_t kMelodyLen  = 128;    // 0x70..0xEF

// Byte offsets within the block (relative to kEepromAddr).
enum Off : uint8_t {
	OFF_MAIN_REVISION   = 0x00,
	OFF_SUB_REVISION    = 0x01,
	OFF_LAYOUT_REVISION = 0x02,
	OFF_STARTUP_POWER   = 0x09,
	OFF_DIRECTION       = 0x0B,
	OFF_MODE_L          = 0x0D,
	OFF_MODE_H          = 0x0E,
	OFF_TX_PROGRAM      = 0x0F,
	OFF_COMM_TIMING     = 0x15,
	OFF_MIN_THROTTLE    = 0x19,
	OFF_MAX_THROTTLE    = 0x1A,
	OFF_BEEP_STRENGTH   = 0x1B,
	OFF_BEACON_STRENGTH = 0x1C,
	OFF_BEACON_DELAY    = 0x1D,
	OFF_DEMAG_COMP      = 0x1F,
	OFF_CENTER_THROTTLE = 0x21,
	OFF_TEMP_PROTECT    = 0x23,
	OFF_LOW_RPM_PROTECT = 0x24,
	OFF_BRAKE_ON_STOP   = 0x27,
	OFF_LED_CONTROL     = 0x28,
	OFF_LAYOUT_TAG      = 0x40,  // 16 ASCII
	OFF_MCU_TAG         = 0x50,  // 16 ASCII
	OFF_NAME            = 0x60,  // 16 ASCII
};

struct Settings {
	bool     valid = false;
	uint8_t  mainRevision  = 0;
	uint8_t  subRevision   = 0;
	uint8_t  layoutRevision = 0;
	uint8_t  motorDirection = 0;   // 1=Normal 2=Reversed 3=Bidir 4=Bidir-rev
	uint8_t  startupPower   = 0;    // 1..13
	uint8_t  commTiming     = 0;    // 1..5 (Low..High)
	uint8_t  minThrottle    = 0;    // us = 1000 + 4*byte
	uint8_t  maxThrottle    = 0;
	uint8_t  centerThrottle = 0;
	uint8_t  beepStrength   = 0;
	uint8_t  beaconStrength = 0;
	uint8_t  beaconDelay    = 0;    // 1..5
	uint8_t  demagComp      = 0;    // 1=Off 2=Low 3=High
	uint8_t  tempProtect    = 0;    // rev33: 0=Off,1..7 levels
	uint8_t  lowRpmProtect  = 0;    // bool
	uint8_t  brakeOnStop    = 0;    // bool
	uint8_t  txProgram      = 0;    // bool
	uint16_t modeSignature  = 0;    // 0x55AA=multi 0xA55A=main 0x5AA5=tail
	char     layoutTag[17]  = {0};
	char     mcuTag[17]     = {0};
	char     name[17]       = {0};
	uint8_t  raw[kEepromLen] = {0};
	uint16_t rawLen = 0;
};

// us pulse width for a throttle byte (min/max/center). 1000 + 4*byte.
static inline uint16_t throttleUs(uint8_t b) { return 1000 + 4 * (uint16_t)b; }

bool read (blheli_bl::Bootloader& bl, Settings& out);        // readFlash + decode
bool write(blheli_bl::Bootloader& bl, const Settings& in);   // encode + write (A1)
void decode(const uint8_t* raw, uint16_t len, Settings& out);
void print(const Settings& s, Stream& out);

// --- config flash page (read-modify-write) -------------------------------------------------
// EFM8BB21 flash page = 512 B; the BlueJay/BLHeli-S config occupies the first 255 B of the page
// at kEepromAddr (0x1A00). A safe write must preserve the whole page.
constexpr uint16_t kPageLen = 512;

// Read the current 512-B config page into out (two 256-B reads). CRC-verified per read.
bool readPage (blheli_bl::Bootloader& bl, uint8_t* out512);
// Erase the page and write page512 back (two 256-B writes), then read back and verify byte-exact.
// Returns true ONLY if the read-back matches. ⚠ ERASES + WRITES FLASH — gate the call site.
bool writePage(blheli_bl::Bootloader& bl, const uint8_t* page512);

} // namespace esc_setup
