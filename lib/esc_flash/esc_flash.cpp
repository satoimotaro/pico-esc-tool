// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// esc_flash implementation (Phase A1) — Intel-HEX parse + page erase/program/verify over
// the BLHeli-S 1-wire bootloader. Strictly refuses the EEPROM/bootloader region (>=kAppEnd).
#include "esc_flash.h"
#include <string.h>

namespace esc_flash {

static int hexNib(char c) {
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	return -1;
}
static int hexByte(const char* p) {  // two nibbles -> byte, or -1
	int h = hexNib(p[0]), l = hexNib(p[1]);
	return (h < 0 || l < 0) ? -1 : (h << 4) | l;
}

// Copy a 16-char BLHeli tag (stop at 0x00/0xFF pad), map non-printables to '.', trim trailing
// spaces so "#J_H_25#        " compares equal to "#J_H_25#".
static void copyTag(char* dst, const uint8_t* src, uint8_t n) {
	uint8_t j = 0;
	for (uint8_t i = 0; i < n; i++) {
		uint8_t c = src[i];
		if (c == 0x00 || c == 0xFF) break;
		dst[j++] = (c >= 32 && c < 127) ? (char)c : '.';
	}
	while (j > 0 && dst[j - 1] == ' ') j--;   // trim trailing pad spaces
	dst[j] = '\0';
}

bool parseIntelHex(const char* hex, size_t len, HexImage& img, const char** err) {
	auto fail = [&](const char* m) { if (err) *err = m; img.valid = false; return false; };
	uint32_t upper = 0;                 // from type-04 extended-linear-address
	size_t i = 0;
	bool sawEof = false;
	while (i < len) {
		while (i < len && hex[i] != ':') i++;        // skip whitespace/line-ends
		if (i >= len) break;
		i++;                                         // consume ':'
		if (i + 8 > len) return fail("truncated record header");
		int bc = hexByte(&hex[i]);       // byte count
		int ah = hexByte(&hex[i + 2]);   // addr hi
		int al = hexByte(&hex[i + 4]);   // addr lo
		int tt = hexByte(&hex[i + 6]);   // type
		if (bc < 0 || ah < 0 || al < 0 || tt < 0) return fail("bad hex digit");
		uint16_t addr = (uint16_t)((ah << 8) | al);
		size_t need = (size_t)8 + bc * 2 + 2;        // header + data + checksum (nibbles*2)
		if (i + need > len) return fail("truncated record body");
		// checksum: sum of all bytes (count..data..cksum) must be 0 (mod 256)
		uint8_t sum = (uint8_t)(bc + ah + al + tt);
		uint8_t rd[256];
		for (int k = 0; k < bc; k++) {
			int b = hexByte(&hex[i + 8 + k * 2]);
			if (b < 0) return fail("bad data digit");
			rd[k] = (uint8_t)b;
			sum = (uint8_t)(sum + b);
		}
		int ck = hexByte(&hex[i + 8 + bc * 2]);
		if (ck < 0) return fail("bad checksum digit");
		sum = (uint8_t)(sum + ck);
		if (sum != 0) return fail("record checksum mismatch");

		switch (tt) {
		case 0x00: {                                 // data
			for (int k = 0; k < bc; k++) {
				uint32_t a = (upper << 16) | (uint32_t)(addr + k);
				if (a < kAppEnd) {                   // application region -> flash it
					img.data[a] = rd[k];
					img.used[a] = true;
					if (a < img.minAddr) img.minAddr = (uint16_t)a;
					if (a + 1 > img.maxAddr) img.maxAddr = (uint16_t)(a + 1);
				} else if (a < kEepromEnd) {         // eeprom/identity -> capture, DON'T flash
					img.identity[a - kEepromBase] = rd[k];
					img.hasIdentity = true;
				} else {                             // bootloader region (>=0x1C00) -> SKIP, never
					img.bootSkipped++;               // flash it (can't reflash the BL we speak through)
				}
			}
			break;
		}
		case 0x01:                                   // EOF
			sawEof = true;
			break;
		case 0x04:                                   // extended linear address
			if (bc != 2) return fail("bad type-04 length");
			upper = (uint32_t)((rd[0] << 8) | rd[1]);
			if (upper != 0) return fail("HEX addresses above 64K not supported");
			break;
		case 0x02:                                   // extended segment address
			return fail("type-02 segment records not supported");
		default:
			break;                                   // ignore 03/05 (start addr)
		}
		i += need;
		if (sawEof) break;
	}
	if (!sawEof)                return fail("no EOF (:00000001FF) record");
	if (img.maxAddr == 0)       return fail("no application data records");
	if (img.hasIdentity) {      // pull the firmware's layout/mcu tags from its eeprom section
		copyTag(img.fwLayoutTag, &img.identity[kIdLayoutOff], 16);
		copyTag(img.fwMcuTag,    &img.identity[kIdMcuOff],    16);
	}
	img.valid = true;
	if (err) *err = "ok";
	return true;
}

Compat checkCompatibility(uint16_t escSig, const char* escLayoutTag, const HexImage& img) {
	Compat c;
	c.sizeOk = img.valid && img.maxAddr > img.minAddr && img.maxAddr <= kAppEnd;
	c.identityKnown = img.hasIdentity;
	const char* el = escLayoutTag ? escLayoutTag : "";
	if (!c.sizeOk) {
		snprintf(c.detail, sizeof(c.detail), "image invalid/empty or exceeds app region");
		return c;
	}
	if (!c.identityKnown) {
		snprintf(c.detail, sizeof(c.detail),
			"HEX has no identity section: cannot verify MCU/layout (ESC layout '%s') - override required", el);
		return c;   // ok stays false: block by default
	}
	uint16_t fwSig = blheli_bl::signatureForMcuTag(img.fwMcuTag);   // shared MCU table (blheli_bl)
	c.mcuOk    = (fwSig != 0) && (fwSig == escSig);
	c.layoutOk = (el[0] != '\0') && (strcmp(el, img.fwLayoutTag) == 0);
	c.ok = c.sizeOk && c.mcuOk && c.layoutOk;
	snprintf(c.detail, sizeof(c.detail), "MCU esc=%04X fw='%s'(%04X)%s  layout esc='%s' fw='%s'%s",
		escSig, img.fwMcuTag, fwSig, c.mcuOk ? "=OK" : "=MISMATCH",
		el, img.fwLayoutTag, c.layoutOk ? "=OK" : "=MISMATCH");
	return c;
}

// program/verify one 512B page. buf points at img.data[pageAddr]. Returns verified==true.
static bool programAndVerifyPage(blheli_bl::Bootloader& bl, uint16_t pageAddr, const uint8_t* buf) {
	if (pageAddr >= kAppEnd) return false;           // hard guard, never touch EEPROM/boot
	if (!bl.erasePage(pageAddr)) return false;
	for (uint16_t off = 0; off < kPageSize; off += kMaxWriteChunk) {
		uint16_t n = (uint16_t)min<int>(kMaxWriteChunk, kPageSize - off);
		if (!bl.writeFlash((uint16_t)(pageAddr + off), buf + off, n)) return false;
	}
	uint8_t rb[kPageSize];
	for (uint16_t off = 0; off < kPageSize; off += kMaxWriteChunk) {
		uint16_t n = (uint16_t)min<int>(kMaxWriteChunk, kPageSize - off);
		if (!bl.readFlash((uint16_t)(pageAddr + off), rb + off, n)) return false;
	}
	return memcmp(rb, buf, kPageSize) == 0;
}

bool programImage(blheli_bl::Bootloader& bl, const HexImage& img, ProgressCb cb) {
	if (!bl.connected() || !img.valid) return false;
	uint16_t firstPage = (uint16_t)(img.minAddr / kPageSize * kPageSize);
	uint16_t lastPage  = (uint16_t)(((img.maxAddr - 1) / kPageSize) * kPageSize);
	if (lastPage >= kAppEnd) return false;           // shouldn't happen (parser guards)

	uint16_t total = (uint16_t)((lastPage - firstPage) / kPageSize + 1), done = 0;
	for (uint16_t p = firstPage; p <= lastPage; p += kPageSize) {
		// skip fully-unused pages (leave them erased/untouched)
		bool any = false;
		for (uint16_t k = 0; k < kPageSize; k++) if (img.used[p + k]) { any = true; break; }
		if (any && !programAndVerifyPage(bl, p, &img.data[p])) return false;
		done++;
		if (cb.fn) cb.fn(done, total, cb.ctx);
	}
	return true;
}

bool verifyImage(blheli_bl::Bootloader& bl, const HexImage& img, ProgressCb cb) {
	if (!bl.connected() || !img.valid) return false;
	uint16_t firstPage = (uint16_t)(img.minAddr / kPageSize * kPageSize);
	uint16_t lastPage  = (uint16_t)(((img.maxAddr - 1) / kPageSize) * kPageSize);
	uint16_t total = (uint16_t)((lastPage - firstPage) / kPageSize + 1), done = 0;
	for (uint16_t p = firstPage; p <= lastPage; p += kPageSize) {
		bool any = false;
		for (uint16_t k = 0; k < kPageSize; k++) if (img.used[p + k]) { any = true; break; }
		if (any) {
			uint8_t rb[kPageSize];
			for (uint16_t off = 0; off < kPageSize; off += kMaxWriteChunk) {
				uint16_t n = (uint16_t)min<int>(kMaxWriteChunk, kPageSize - off);
				if (!bl.readFlash((uint16_t)(p + off), rb + off, n)) return false;
			}
			// only compare the bytes the image actually defined
			for (uint16_t k = 0; k < kPageSize; k++)
				if (img.used[p + k] && rb[k] != img.data[p + k]) return false;
		}
		done++;
		if (cb.fn) cb.fn(done, total, cb.ctx);
	}
	return true;
}

} // namespace esc_flash
