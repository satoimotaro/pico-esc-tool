// SPDX-License-Identifier: GPL-3.0-or-later
//
// esc_flash implementation — scaffold. Phase A1 (erase/program/verify) pending
// PROTOCOL.md write-path commands. A0 only needs blheli_bl connect + device ID.
#include "esc_flash.h"

namespace esc_flash {

bool programImage(blheli_bl::Bootloader& bl, const uint8_t*, size_t, ProgressCb) {
	if (!bl.connected()) return false;
	// [TODO:proto] A1: for each page → setAddress, erasePage, writeMemory, then verify.
	return false;
}

bool verifyImage(blheli_bl::Bootloader& bl, const uint8_t*, size_t, ProgressCb) {
	if (!bl.connected()) return false;
	// [TODO:proto] A1: readMemory page-by-page and compare.
	return false;
}

} // namespace esc_flash
