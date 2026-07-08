// SPDX-License-Identifier: GPL-3.0-or-later
//
// esc_setup implementation — scaffold. Decoding offsets pending EEPROM.md [TODO:proto].
#include "esc_setup.h"

namespace esc_setup {

bool read(blheli_bl::Bootloader& bl, Settings& out) {
	out = Settings{};
	if (!bl.connected()) return false;
	if (!bl.readMemory(kEepromAddr, out.raw, kEepromLen)) return false;
	out.rawLen = kEepromLen;
	// [TODO:proto] decode out.raw → fields per EEPROM.md offset map.
	out.valid = false;  // set true once decoding is implemented
	return out.rawLen > 0;
}

bool write(blheli_bl::Bootloader& /*bl*/, const Settings& /*in*/) {
	return false; // [TODO:proto] Phase A1: encode fields → raw, writeMemory + verify
}

void print(const Settings& s, Stream& out) {
	out.println(F("--- ESC settings ---"));
	if (s.rawLen == 0) { out.println(F("(no data read)")); return; }
	out.print(F("layoutRev=")); out.println(s.layoutRevision);
	if (!s.valid) {
		out.println(F("(decode not yet implemented — raw dump:)"));
		for (uint16_t i = 0; i < s.rawLen; i++) {
			if (i % 16 == 0) { out.println(); out.printf("%04X: ", kEepromAddr + i); }
			out.printf("%02X ", s.raw[i]);
		}
		out.println();
		return;
	}
	out.print(F("dir="));   out.println(s.motorDirection);
	out.print(F("timing=")); out.println(s.motorTiming);
	out.print(F("pwmFreq=")); out.println(s.pwmFrequency);
	// ... extend as fields are decoded
}

} // namespace esc_setup
