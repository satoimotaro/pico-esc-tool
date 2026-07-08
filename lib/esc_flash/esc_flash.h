// SPDX-License-Identifier: GPL-3.0-or-later
//
// esc_flash — program/verify ESC firmware (.hex) via the 1-wire bootloader.
// Builds on blheli_bl. Full path is Phase A1; A0 only exercises connect + device ID.
#pragma once
#include <Arduino.h>
#include "blheli_bl.h"

namespace esc_flash {

struct ProgressCb { void (*fn)(uint16_t done, uint16_t total, void* ctx) = nullptr; void* ctx = nullptr; };

// Program an Intel-HEX / raw image to flash, then verify. (A1) [TODO:proto]
bool programImage(blheli_bl::Bootloader& bl, const uint8_t* image, size_t len, ProgressCb cb = {});
bool verifyImage (blheli_bl::Bootloader& bl, const uint8_t* image, size_t len, ProgressCb cb = {});

} // namespace esc_flash
