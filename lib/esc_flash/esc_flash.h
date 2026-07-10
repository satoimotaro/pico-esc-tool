// SPDX-License-Identifier: GPL-3.0-or-later
//
// esc_flash — program/verify BLHeli-S ESC firmware (Intel-HEX) via the 1-wire bootloader.
// Builds on blheli_bl. Phase A1. Target: EFM8BB21 (LittleBee Spring 30A) unless overridden.
//
// SAFETY: the SiLabs bootloader lives at the TOP of flash (0x1C00-0x1FFF on BB2) and MUST
// NOT be erased/overwritten — doing so bricks the ESC unrecoverably (no way back in without
// a C2 debugger). The EEPROM parameter page (0x1A00) is also excluded. This module refuses
// any HEX byte at/above kAppEnd, so a wrong/oversized image errors out instead of bricking.
#pragma once
#include <Arduino.h>
#include "blheli_bl.h"

namespace esc_flash {

// --- EFM8BB21 (BB2) flash geometry, from PROTOCOL.md §D ---
constexpr uint16_t kPageSize   = 512;       // erase granularity
constexpr uint16_t kAppBase    = 0x0000;    // application starts here
constexpr uint16_t kEepromBase = 0x1A00;    // BLHeli-S parameter page (do NOT flash)
constexpr uint16_t kBootBase   = 0x1C00;    // bootloader (NEVER touch)
constexpr uint16_t kAppEnd     = kEepromBase;   // app region = [kAppBase, kAppEnd)  (0x1A00)
constexpr uint16_t kEepromEnd  = kBootBase;     // eeprom/identity region = [0x1A00, 0x1C00)
constexpr uint16_t kMaxWriteChunk = 256;    // blheli_bl writeFlash max per call
constexpr uint16_t kIdLayoutOff = 0x40;     // LAYOUT tag within the eeprom region
constexpr uint16_t kIdMcuOff    = 0x50;     // MCU tag within the eeprom region

struct ProgressCb { void (*fn)(uint16_t done, uint16_t total, void* ctx) = nullptr; void* ctx = nullptr; };

struct HexImage {
	uint8_t  data[kAppEnd];        // flat app image, 0xFF = erased/unused
	bool     used[kAppEnd] = {};   // true where a HEX record actually placed a byte
	uint16_t minAddr = kAppEnd;    // lowest used address (kAppEnd if empty)
	uint16_t maxAddr = 0;          // highest used address + 1
	bool     valid = false;
	// Firmware identity from the HEX's EEPROM section [0x1A00,0x1C00) — captured for the
	// compatibility check but NEVER flashed (flashing keeps the ESC's current settings).
	uint8_t  identity[kEepromEnd - kEepromBase];  // 0x200 bytes, 0xFF = absent
	bool     hasIdentity = false;   // the HEX included eeprom records
	char     fwLayoutTag[17] = {0}; // @0x1A40: FET/pin map the firmware is built for (must match ESC)
	char     fwMcuTag[17]    = {0}; // @0x1A50: e.g. "#BLHELI$EFM8B21#"
	uint16_t bootSkipped = 0;       // # of >=0x1C00 (bootloader) bytes in the HEX — NEVER flashed
	                                // (a full stock BLHeli-S HEX carries the BL for C2/SWD; via the
	                                // 1-wire bootloader we flash app-only and leave the BL intact)
	HexImage() { memset(data, 0xFF, sizeof(data)); memset(identity, 0xFF, sizeof(identity)); }
};

// Result of matching a parsed image against the connected ESC's identity.
struct Compat {
	bool sizeOk       = false;  // image non-empty and within the app region
	bool identityKnown= false;  // the HEX carried an eeprom identity section to compare
	bool mcuOk        = false;  // ESC signature consistent with the firmware's target MCU
	bool layoutOk     = false;  // ESC layout tag == firmware layout tag
	bool ok           = false;  // ALL required checks pass -> safe to flash
	char detail[128]  = {0};    // human-readable reason
};

// Compare a parsed firmware image to the connected ESC: escSig = bootloader signature word
// (EFM8BB21 = 0xE8B2), escLayoutTag = the ESC's LAYOUT tag read from its config (0x1A40).
// Returns ok=true only when the image fits the app region AND the firmware's MCU + layout match
// the ESC. If the HEX has no identity section, ok=false (can't verify) unless the caller overrides.
Compat checkCompatibility(uint16_t escSig, const char* escLayoutTag, const HexImage& img);

// Parse Intel-HEX text into an app-region image. Returns false (img.valid=false) on a
// malformed record, a checksum error, or ANY byte at/above kAppEnd (bootloader/EEPROM).
// `err` (optional) receives a short reason for logging.
bool parseIntelHex(const char* hex, size_t len, HexImage& img, const char** err = nullptr);

// Erase + program only the pages the image touches, then verify. Requires bl.connected().
// Never erases/writes at/above kAppEnd. Returns true iff every touched page verified.
bool programImage(blheli_bl::Bootloader& bl, const HexImage& img, ProgressCb cb = {});
bool verifyImage (blheli_bl::Bootloader& bl, const HexImage& img, ProgressCb cb = {});

} // namespace esc_flash
