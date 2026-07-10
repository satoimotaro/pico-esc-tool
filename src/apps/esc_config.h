// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// esc_config.h — user HARDWARE configuration for the RP2040 ESC tool.
//
// This is the ONE place to edit for your wiring. Everything here is compile-time (the DShot/PIO
// and 1-wire bootloader bind to specific GPIOs at build time), so changing a pin means rebuilding
// and re-flashing the Pico. Runtime-changeable things do NOT live here: the SETUP/DRIVE mode is
// switched live with the `mode` command, and ESC settings are read/written over the tool itself.
#pragma once

// --- ESC signal wiring ------------------------------------------------------------------------
// One GPIO per ESC. Each line carries BOTH the 1-wire bootloader (config/flash) AND DShot
// (drive/telemetry) for that ESC. List them in ESC-index order; add/remove entries to match how
// many ESCs you wired. See construction/wiring/.
#define ESC_SIGNAL_PINS   { 10, 11 }

// --- Mode-select GPIO (esc_tool only) ---------------------------------------------------------
// Read once at boot: tie LOW for DRIVE (Wi-Fi off), or leave UNCONNECTED (internal pull-up => HIGH)
// for SETUP. Wiring it is optional. Set to -1 to ignore the pin entirely (always boot into SETUP;
// switch anytime with the `mode` command).
#define ESC_MODE_PIN      22

// --- Wi-Fi Access Point (esc_tool SETUP mode) -------------------------------------------------
#define ESC_AP_SSID       "pico-esc-tool"
#define ESC_AP_PASS       "esctool1234"    // >= 8 chars (WPA2); change before real use

// --- DShot / motor ----------------------------------------------------------------------------
#define ESC_DSHOT_KBAUD   600              // DShot bitrate in kbaud (600 = DShot600)
#define ESC_MOTOR_POLES   14               // motor magnet poles (for eRPM -> RPM); motor-dependent
