// SPDX-License-Identifier: GPL-3.0-or-later
//
// esc_flash implementation — scaffold. Phase A1 (erase/program/verify) pending
// PROTOCOL.md write-path commands. A0 only needs blheli_bl connect + device ID.
#include "esc_flash.h"

namespace esc_flash {

bool programImage(blheli_bl::Bootloader& bl, const uint8_t*, size_t, ProgressCb) {
	if (!bl.connected()) return false;
	// A1: parse Intel-HEX → for each 512B page: erasePage(addr), then writeFlash(addr,buf,len)
	// in <=256B chunks (blheli_bl primitives now exist), then verifyImage().
	return false;
}

bool verifyImage(blheli_bl::Bootloader& bl, const uint8_t*, size_t, ProgressCb) {
	if (!bl.connected()) return false;
	// A1: readFlash page-by-page and compare against the image.
	return false;
}

} // namespace esc_flash
