// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// esc_setup implementation. Decode is verified against esc-configurator
// BlheliS/settings.js + BLHeli_S.asm Eep_* offsets (layout rev 32/33).
#include "esc_setup.h"

namespace esc_setup {

static void copyTag(char* dst, const uint8_t* src, uint8_t n) {
	uint8_t j = 0;
	for (uint8_t i = 0; i < n; i++) {
		uint8_t c = src[i];
		if (c == 0x00 || c == 0xFF) break;   // padding / erased
		dst[j++] = (c >= 32 && c < 127) ? (char)c : '.';
	}
	while (j > 0 && dst[j - 1] == ' ') j--;  // trim trailing pad spaces (so tags compare cleanly)
	dst[j] = '\0';
}

void decode(const uint8_t* r, uint16_t len, Settings& s) {
	if (len < kEepromLen) { s.valid = false; return; }
	s.mainRevision   = r[OFF_MAIN_REVISION];
	s.subRevision    = r[OFF_SUB_REVISION];
	s.layoutRevision = r[OFF_LAYOUT_REVISION];
	s.startupPower   = r[OFF_STARTUP_POWER];
	s.motorDirection = r[OFF_DIRECTION];
	// MODE is big-endian on the wire: raw[0x0D] is the HIGH byte. (MULTI=0x55AA, MAIN=0xA55A,
	// TAIL=0x5AA5 per esc-configurator Bluejay/eeprom.js MODES.)
	s.modeSignature  = (uint16_t(r[OFF_MODE_L]) << 8) | r[OFF_MODE_H];
	s.txProgram      = r[OFF_TX_PROGRAM];
	s.commTiming     = r[OFF_COMM_TIMING];
	s.minThrottle    = r[OFF_MIN_THROTTLE];
	s.maxThrottle    = r[OFF_MAX_THROTTLE];
	s.beepStrength   = r[OFF_BEEP_STRENGTH];
	s.beaconStrength = r[OFF_BEACON_STRENGTH];
	s.beaconDelay    = r[OFF_BEACON_DELAY];
	s.demagComp      = r[OFF_DEMAG_COMP];
	s.centerThrottle = r[OFF_CENTER_THROTTLE];
	s.tempProtect    = r[OFF_TEMP_PROTECT];
	s.lowRpmProtect  = r[OFF_LOW_RPM_PROTECT];
	s.brakeOnStop    = r[OFF_BRAKE_ON_STOP];
	copyTag(s.layoutTag, r + OFF_LAYOUT_TAG, 16);
	copyTag(s.mcuTag,    r + OFF_MCU_TAG,    16);
	copyTag(s.name,      r + OFF_NAME,       16);
	// Sanity: a programmed BLHeli-S block has a known mode signature.
	s.valid = (s.modeSignature == 0x55AA || s.modeSignature == 0xA55A ||
	           s.modeSignature == 0x5AA5);
}

bool read(blheli_bl::Bootloader& bl, Settings& out) {
	out = Settings{};
	if (!bl.connected()) return false;
	// SiLabs EFM8 has NO EEPROM — BLHeli-S stores its config block in FLASH at 0x1A00, so it's
	// read with the flash-read command (0x03), not the Atmel EEPROM-read (0x04). readEeprom(0x04)
	// returned zero bytes on this MCU; readFlash(0x03) is what esc-configurator/BLHeliSuite use.
	if (!bl.readFlash(kEepromAddr, out.raw, kEepromLen)) return false;
	out.rawLen = kEepromLen;
	decode(out.raw, out.rawLen, out);
	return true;
}

bool write(blheli_bl::Bootloader& /*bl*/, const Settings& /*in*/) {
	return false;  // Phase A1: encode fields into raw (read-modify-write) + writeEeprom + verify
}

bool readPage(blheli_bl::Bootloader& bl, uint8_t* out512) {
	if (!bl.connected()) return false;
	return bl.readFlash(kEepromAddr,       out512,       256)
	    && bl.readFlash(kEepromAddr + 256, out512 + 256, 256);
}

bool writePage(blheli_bl::Bootloader& bl, const uint8_t* page512) {
	if (!bl.connected()) return false;
	if (!bl.erasePage(kEepromAddr)) return false;               // erase 0x1A00..0x1BFF
	if (!bl.writeFlash(kEepromAddr,       page512,       256)) return false;
	if (!bl.writeFlash(kEepromAddr + 256, page512 + 256, 256)) return false;
	uint8_t rb[kPageLen];                                        // read back and verify byte-exact
	if (!readPage(bl, rb)) return false;
	for (uint16_t i = 0; i < kPageLen; i++) if (rb[i] != page512[i]) return false;
	return true;
}

static const char* dirName(uint8_t d) {
	switch (d) { case 1: return "Normal"; case 2: return "Reversed";
	             case 3: return "Bidir"; case 4: return "Bidir-rev"; default: return "?"; }
}
static const char* timingName(uint8_t t) {
	switch (t) { case 1: return "Low"; case 2: return "MedLow"; case 3: return "Med";
	             case 4: return "MedHigh"; case 5: return "High"; default: return "?"; }
}
static const char* demagName(uint8_t d) {
	switch (d) { case 1: return "Off"; case 2: return "Low"; case 3: return "High"; default: return "?"; }
}

void print(const Settings& s, Stream& out) {
	out.println(F("--- ESC settings (BLHeli-S) ---"));
	if (s.rawLen == 0) { out.println(F("(no data read)")); return; }
	out.printf("firmware      : %u.%u  (layout rev %u)\n", s.mainRevision, s.subRevision, s.layoutRevision);
	out.printf("name / layout : \"%s\" / \"%s\"\n", s.name, s.layoutTag);
	out.printf("mcu tag       : \"%s\"\n", s.mcuTag);
	if (!s.valid) out.println(F("WARNING: mode signature not recognized - block may be unprogrammed/misread"));
	out.printf("direction     : %u (%s)\n", s.motorDirection, dirName(s.motorDirection));
	out.printf("comm timing   : %u (%s)\n", s.commTiming, timingName(s.commTiming));
	out.printf("demag comp    : %u (%s)\n", s.demagComp, demagName(s.demagComp));
	out.printf("startup power : %u\n", s.startupPower);
	out.printf("throttle us   : min=%u max=%u center=%u\n",
	           throttleUs(s.minThrottle), throttleUs(s.maxThrottle), throttleUs(s.centerThrottle));
	out.printf("beep/beacon   : beep=%u beacon=%u delay=%u\n", s.beepStrength, s.beaconStrength, s.beaconDelay);
	out.printf("protection    : temp=%u lowRpm=%u\n", s.tempProtect, s.lowRpmProtect);
	out.printf("brake on stop : %u\n", s.brakeOnStop);
	out.print(F("raw: "));
	for (uint16_t i = 0; i < s.rawLen; i++) {
		if (i % 16 == 0) { out.println(); out.printf("%04X: ", kEepromAddr + i); }
		out.printf("%02X ", s.raw[i]);
	}
	out.println();
}

} // namespace esc_setup
