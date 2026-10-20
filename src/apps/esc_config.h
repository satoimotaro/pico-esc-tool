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

// --- CLI transport -----------------------------------------------------------------------------
// 0 = USB CDC (default; host/esctool.py on a laptop). 1 = UART0 on the pins below, for a board with
// no USB host — the ROV's Pi, or the thrust bench's fixture Pico relaying the CLI over Wi-Fi. Same
// line protocol either way; the `#` boot notices follow the CLI to the chosen transport.
#ifndef ESC_CONSOLE_UART
#define ESC_CONSOLE_UART  0
#endif
#define ESC_UART_TX       0                // GP0 -> peer RX
#define ESC_UART_RX       1                // GP1 <- peer TX
#define ESC_UART_BAUD     921600
// The CLI is polled once per core0 loop and a flash step holds the loop, so the RX side needs more
// than the 32-byte default to take a full 600-char command line without dropping bytes.
#define ESC_UART_FIFO     1024

// --- Wi-Fi Access Point (esc_tool SETUP mode) -------------------------------------------------
#define ESC_AP_SSID       "pico-esc-tool"
#define ESC_AP_PASS       "esctool1234"    // >= 8 chars (WPA2); change before real use

// --- DShot / motor ----------------------------------------------------------------------------
#define ESC_DSHOT_KBAUD   300              // DShot bitrate in kbaud  [TEST: A_H_30 ESCs may need 300, not 600]
#define ESC_MOTOR_POLES   14               // motor magnet poles (for eRPM -> RPM); motor-dependent
