// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// presets — named ESC settings for a motor, applied from the CLI (`preset`). A preset is a list of
// (offset, value) bytes in the ESC's own settings block, for ONE firmware family: the BLHeli-S
// layout at 0x1A00 that Bluejay and BlueGill share (BlueGill adds its sine block), or AM32's block
// (0x7C00 / 0xF800). Identity bytes (revisions, layout and name strings, the startup tune) are
// never part of one, so applying a preset leaves the firmware what it is.
//
// The built-in ones are configs that ran a motor on the bench, each traced to where it was taken.
// `preset save` adds more in LittleFS (PRESET_DIR), in the same form.
#pragma once
#include <Arduino.h>
#include <LittleFS.h>

namespace presets {

// Which settings map a preset is written for. Told apart on the ESC by its bootloader id (AM32) or
// the layout-revision byte at 0x1A02 (Bluejay 0.21 = 0xD0, BlueGill = 0xE2).
enum Family : uint8_t { BLUEJAY = 1, BLUEGILL = 2, AM32 = 3 };
inline const char* familyName(uint8_t f) { return f == BLUEJAY ? "bluejay" : f == BLUEGILL ? "bluegill" : f == AM32 ? "am32" : "?"; }

struct Byte { uint8_t off, val; };
struct Preset {
	const char* name;
	uint8_t     family;
	const char* note;
	const Byte* bytes;
	uint8_t     n;
};

// Settings range a `preset save` captures: Bluejay/BlueGill 0x03..0x3F (BlueGill's sine block is
// 0x2B..0x33), AM32 5..47 (host/pico_esc/am32.py FIELDS). Everything else is identity.
static const uint8_t BJ_FIRST = 0x03, BJ_LAST = 0x3F, AM_FIRST = 5, AM_LAST = 47;

// --- built-in ---------------------------------------------------------------------------------
// The Bluejay/BlueGill ones carry every byte of 0x03..0x3F as captured, not a diff against the
// defaults: what a flash leaves there depends on the flasher (esc-configurator's defaults have
// Medium timing where the firmware image has MediumHigh), so a diff would land differently.
//
// F2838 350KV 12N14P potted pod on stock Bluejay 0.21 (A_H_30). bench-registry snapshot
// 20261023T191345Z-potted-pod-gp16-bluejay: startup power min/max 30/25 (defaults 21/5),
// low-RPM power protection off (with it on the pod sat at ~310 rpm for any throttle 200..550),
// timing MediumHigh, demag High. Then 150/300/450/600 -> 234/552/877/1208 rpm in air.
static const Byte F2838_BJ[] = {
	{0x03,0xFF}, {0x04,0x1E}, {0x05,0x01}, {0x06,0xFF}, {0x07,0x19}, {0x08,0xFF}, {0x09,0x00}, {0x0A,0x18},
	{0x0B,0x01}, {0x0C,0xFF}, {0x0D,0x55}, {0x0E,0xAA}, {0x0F,0xFF}, {0x10,0xFF}, {0x11,0xFF}, {0x12,0xFF},
	{0x13,0xFF}, {0x14,0xFF}, {0x15,0x04}, {0x16,0xFF}, {0x17,0xFF}, {0x18,0xFF}, {0x19,0xFF}, {0x1A,0xFF},
	{0x1B,0x28}, {0x1C,0x50}, {0x1D,0x04}, {0x1E,0xFF}, {0x1F,0x03}, {0x20,0xFF}, {0x21,0xFF}, {0x22,0xFF},
	{0x23,0x00}, {0x24,0xFF}, {0x25,0xFF}, {0x26,0xFF}, {0x27,0x00}, {0x28,0x00}, {0x29,0x02}, {0x2A,0x00},
	{0x2B,0xFF}, {0x2C,0xFF}, {0x2D,0xFF}, {0x2E,0xFF}, {0x2F,0xFF}, {0x30,0xFF}, {0x31,0xFF}, {0x32,0xFF},
	{0x33,0xFF}, {0x34,0xFF}, {0x35,0xFF}, {0x36,0xFF}, {0x37,0xFF}, {0x38,0xFF}, {0x39,0xFF}, {0x3A,0xFF},
	{0x3B,0xFF}, {0x3C,0xFF}, {0x3D,0xFF}, {0x3E,0xFF}, {0x3F,0xFF} };
// Same, bidirectional (3D): the 20260801T051045Z-bluejay-stock-configured config, which the 930KV
// 12N14P ran on as well.
static const Byte F2838_BJ_3D[] = {
	{0x03,0xFF}, {0x04,0x1E}, {0x05,0x01}, {0x06,0xFF}, {0x07,0x19}, {0x08,0xFF}, {0x09,0x00}, {0x0A,0x18},
	{0x0B,0x03}, {0x0C,0xFF}, {0x0D,0x55}, {0x0E,0xAA}, {0x0F,0xFF}, {0x10,0xFF}, {0x11,0xFF}, {0x12,0xFF},
	{0x13,0xFF}, {0x14,0xFF}, {0x15,0x04}, {0x16,0xFF}, {0x17,0xFF}, {0x18,0xFF}, {0x19,0xFF}, {0x1A,0xFF},
	{0x1B,0x28}, {0x1C,0x50}, {0x1D,0x04}, {0x1E,0xFF}, {0x1F,0x03}, {0x20,0xFF}, {0x21,0xFF}, {0x22,0xFF},
	{0x23,0x00}, {0x24,0xFF}, {0x25,0xFF}, {0x26,0xFF}, {0x27,0x00}, {0x28,0x00}, {0x29,0x02}, {0x2A,0x00},
	{0x2B,0xFF}, {0x2C,0xFF}, {0x2D,0xFF}, {0x2E,0xFF}, {0x2F,0xFF}, {0x30,0xFF}, {0x31,0xFF}, {0x32,0xFF},
	{0x33,0xFF}, {0x34,0xFF}, {0x35,0xFF}, {0x36,0xFF}, {0x37,0xFF}, {0x38,0xFF}, {0x39,0xFF}, {0x3A,0xFF},
	{0x3B,0xFF}, {0x3C,0xFF}, {0x3D,0xFF}, {0x3E,0xFF}, {0x3F,0xFF} };
// BlueGill S4 forced-sine start (sine_mode 2) for 12N14P outriggers, bidirectional: snapshot
// 20260801T050542Z-bluegill-s4 (930KV, 20/20 starts to 4000 rpm), kept through the 350KV swap.
static const Byte BG_SINE_3D[] = {
	{0x03,0xFF}, {0x04,0x1E}, {0x05,0x01}, {0x06,0xFF}, {0x07,0x19}, {0x08,0xFF}, {0x09,0x00}, {0x0A,0x18},
	{0x0B,0x03}, {0x0C,0xFF}, {0x0D,0x55}, {0x0E,0xAA}, {0x0F,0xFF}, {0x10,0xFF}, {0x11,0xFF}, {0x12,0xFF},
	{0x13,0xFF}, {0x14,0xFF}, {0x15,0x04}, {0x16,0xFF}, {0x17,0xFF}, {0x18,0xFF}, {0x19,0xFF}, {0x1A,0xFF},
	{0x1B,0x28}, {0x1C,0x50}, {0x1D,0x04}, {0x1E,0xFF}, {0x1F,0x03}, {0x20,0xFF}, {0x21,0xFF}, {0x22,0xFF},
	{0x23,0x00}, {0x24,0xFF}, {0x25,0xFF}, {0x26,0xFF}, {0x27,0x00}, {0x28,0x00}, {0x29,0x02}, {0x2A,0x00},
	{0x2B,0x00}, {0x2C,0x00}, {0x2D,0x00}, {0x2E,0x02}, {0x2F,0x18}, {0x30,0x3C}, {0x31,0x10}, {0x32,0x24},
	{0x33,0xEF}, {0x34,0xFF}, {0x35,0xFF}, {0x36,0xFF}, {0x37,0xFF}, {0x38,0xFF}, {0x39,0xFF}, {0x3A,0xFF},
	{0x3B,0xFF}, {0x3C,0xFF}, {0x3D,0xFF}, {0x3E,0xFF}, {0x3F,0xFF} };
// F2838 350KV on the DYS Aria 70A (AM32 2.21), as read back 2026-11-03 after the in-air runs
// (100 no start, 200/300/450/600 -> 315/513/827/1115 rpm): host am32.py DEFAULTS plus sine start,
// startup_power 150, kv 340, 14 poles. Every setting byte, so it also overwrites one left at 0xFF
// by a reflash through a flight controller (the identity bytes 0..4 still have to be valid).
static const Byte F2838_AM32[] = {
	{5,160}, {6,1}, {7,0}, {8,10}, {9,100}, {10,0}, {11,100}, {12,0}, {13,0}, {17,0}, {18,0},
	{19,1}, {20,1}, {21,1}, {22,1}, {23,26}, {24,24}, {25,150}, {26,8}, {27,14}, {28,0}, {29,0},
	{30,5}, {31,0}, {32,128}, {33,128}, {34,128}, {35,50}, {36,0}, {37,50}, {38,0}, {39,0},
	{40,5}, {41,10}, {42,10}, {43,255}, {44,255}, {45,5}, {46,1}, {47,0} };
// Same, bidirectional.
static const Byte F2838_AM32_3D[] = {
	{5,160}, {6,1}, {7,0}, {8,10}, {9,100}, {10,0}, {11,100}, {12,0}, {13,0}, {17,0}, {18,1},
	{19,1}, {20,1}, {21,1}, {22,1}, {23,26}, {24,24}, {25,150}, {26,8}, {27,14}, {28,0}, {29,0},
	{30,5}, {31,0}, {32,128}, {33,128}, {34,128}, {35,50}, {36,0}, {37,50}, {38,0}, {39,0},
	{40,5}, {41,10}, {42,10}, {43,255}, {44,255}, {45,5}, {46,1}, {47,0} };

// A2212 930KV on the same Aria (AM32 2.21), 2026-11-05: F2838_AM32 with kv 23 (= 940, the nearest
// step to 930). Through the web motor test, in air: 1050/1100/1150/1200 -> 380/1004/1522/2021 rpm.
static const Byte A2212_AM32[] = {
	{5,160}, {6,1}, {7,0}, {8,10}, {9,100}, {10,0}, {11,100}, {12,0}, {13,0}, {17,0}, {18,0},
	{19,1}, {20,1}, {21,1}, {22,1}, {23,26}, {24,24}, {25,150}, {26,23}, {27,14}, {28,0}, {29,0},
	{30,5}, {31,0}, {32,128}, {33,128}, {34,128}, {35,50}, {36,0}, {37,50}, {38,0}, {39,0},
	{40,5}, {41,10}, {42,10}, {43,255}, {44,255}, {45,5}, {46,1}, {47,0} };

#define PRESET_N(a) uint8_t(sizeof(a) / sizeof(a[0]))
static const Preset BUILTIN[] = {
	{ "f2838",       BLUEJAY,  "F2838 350KV 12N14P, one-way (potted pod, 2026-10-23)", F2838_BJ,      PRESET_N(F2838_BJ) },
	{ "f2838-3d",    BLUEJAY,  "F2838 350KV / 930KV 12N14P, bidirectional",            F2838_BJ_3D,   PRESET_N(F2838_BJ_3D) },
	{ "sine-3d",     BLUEGILL, "BlueGill S4 forced-sine start, bidirectional",         BG_SINE_3D,    PRESET_N(BG_SINE_3D) },
	{ "f2838-am32",  AM32,     "F2838 350KV on AM32, one-way (Aria, 2026-11-03)",      F2838_AM32,    PRESET_N(F2838_AM32) },
	{ "f2838-am32-3d", AM32,   "F2838 350KV on AM32, bidirectional",                   F2838_AM32_3D, PRESET_N(F2838_AM32_3D) },
	{ "a2212-am32",  AM32,     "A2212 930KV on AM32, one-way (Aria, 2026-11-05)",      A2212_AM32,    PRESET_N(A2212_AM32) },
};
#undef PRESET_N
static const uint8_t BUILTIN_N = sizeof(BUILTIN) / sizeof(BUILTIN[0]);

// --- saved (LittleFS) ---------------------------------------------------------------------------
// File = 'P', family, n, then n (offset, value) pairs. Name = file name, [a-z0-9_-]{1,24}.
static const char* PRESET_DIR = "/presets";
static const uint8_t MAX_BYTES = 64;

struct Loaded { char name[25]; uint8_t family = 0, n = 0; Byte b[MAX_BYTES]; bool builtin = false; const char* note = ""; };

inline bool validName(const char* s) {
	size_t n = strlen(s);
	if (n < 1 || n > 24) return false;
	for (size_t i = 0; i < n; i++) { char c = s[i]; if (!(isdigit(c) || (c >= 'a' && c <= 'z') || c == '-' || c == '_')) return false; }
	return true;
}
inline String path(const char* name) { return String(PRESET_DIR) + "/" + name; }

inline bool find(const char* name, Loaded& out) {
	for (const Preset& p : BUILTIN) if (!strcmp(p.name, name)) {
		strncpy(out.name, p.name, 24); out.name[24] = 0;
		out.family = p.family; out.n = p.n; memcpy(out.b, p.bytes, p.n * sizeof(Byte));
		out.builtin = true; out.note = p.note; return true;
	}
	if (!validName(name)) return false;
	File f = LittleFS.open(path(name), "r");
	if (!f) return false;
	uint8_t h[3];
	bool ok = f.read(h, 3) == 3 && h[0] == 'P' && h[2] <= MAX_BYTES &&
	          f.read((uint8_t*)out.b, h[2] * sizeof(Byte)) == h[2] * sizeof(Byte);
	f.close();
	if (!ok) return false;
	strncpy(out.name, name, 24); out.name[24] = 0;
	out.family = h[1]; out.n = h[2]; out.builtin = false; out.note = "saved";
	return true;
}

inline bool save(const char* name, uint8_t family, const Byte* b, uint8_t n) {
	if (!validName(name) || n > MAX_BYTES) return false;
	for (const Preset& p : BUILTIN) if (!strcmp(p.name, name)) return false;   // built-ins are fixed
	LittleFS.mkdir(PRESET_DIR);
	File f = LittleFS.open(path(name), "w");
	if (!f) return false;
	uint8_t h[3] = { 'P', family, n };
	bool ok = f.write(h, 3) == 3 && f.write((const uint8_t*)b, n * sizeof(Byte)) == n * sizeof(Byte);
	f.close();
	return ok;
}

inline bool remove(const char* name) { return validName(name) && LittleFS.remove(path(name)); }

}  // namespace presets
