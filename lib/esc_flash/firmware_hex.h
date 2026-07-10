// SPDX-License-Identifier: GPL-3.0-or-later
//
// firmware_hex.h - the BLHeli-S firmware image to flash, as Intel-HEX text.
//
// !!! PLACEHOLDER (empty) !!!  spike_program refuses to erase/write an empty image, so as-is it
// does nothing. Paste the BLHeli-S HEX matching your ESC before flashing (see below).
//
// HOW TO FILL IN:
//   1) Run `esctool list` (or spike_setup) and note the ESC's layout tag (e.g. #J_H_25#) and
//      signature (EFM8BB21 = E8 B2). Do NOT guess the layout - a wrong FET map is recoverable
//      but wrong.
//   2) Get the matching HEX from github.com/bitdump/BLHeli -> "BLHeli_S SiLabs/Hex files/"
//      (e.g. J_H_25_REV16_7.HEX for layout J_H_25 rev 16.7).
//   3) Paste its full text between the R"HEX( ... )HEX" delimiters below.
//
// GUARDS: spike_program refuses to flash unless esc_flash's compat check matches the ESC's
// layout + MCU. The parser flashes app-only (0x0000-0x19FF), captures the HEX's eeprom section
// for the compat check + to auto-apply the firmware's default config after flashing, and SKIPS
// the bootloader region (0x1C00+) - the BL we speak through is never overwritten.
#pragma once

inline const char* kFirmwareHexName = "PLACEHOLDER (empty) - paste BLHeli-S HEX here";

inline const char kFirmwareHex[] = R"HEX(
:00000001FF
)HEX";
