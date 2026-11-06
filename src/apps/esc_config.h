// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// esc_config.h — user HARDWARE configuration for the RP2040 ESC tool.
//
// This is the ONE place to edit for your wiring. Everything here is compile-time, so changing it
// means rebuilding and re-flashing the Pico, with one exception: the ESC signal pins below are only
// the default, and the `pins` command replaces them without a rebuild (stored, used from the next
// boot). Other runtime-changeable things do NOT live here either: the SETUP/DRIVE mode is switched
// live with the `mode` command, and ESC settings are read/written over the tool itself.
#pragma once

// --- ESC signal wiring ------------------------------------------------------------------------
// One GPIO per ESC. Each line carries BOTH the 1-wire bootloader (config/flash) AND DShot
// (drive/telemetry) for that ESC. List them in ESC-index order. This is the build's default list:
// `pins 16 17 18 19` stores another one in LittleFS (/cfg/pins) and `pins default` goes back to
// this; either takes effect after a reboot. 1..8 GPIOs, none twice, none used by this build (the
// Pico W radio GP23/24/25/29, the LCD bus, the UART console, ESC_MODE_PIN; checked at compile time
// here and by `pins`). See construction/wiring/ and docs/esc-lifecycle.md "Signal pins and presence".
// How many: the 1-wire bootloader is bit-banged, so any GPIO works for config/flash, and the
// configurator passthrough (fourway.h) shows up to 8. Driving is the limit: each wired ESC holds one
// PIO state machine (the idle hold streams to every one of them). The bidir program (28 of a PIO's 32
// instructions) fits only on pio0, since the Pico W radio sits on pio1, so at most 4 ESCs can run
// bidir DShot; a normal-DShot stream goes to pio1's 3 free SMs first (esc_session.h newNormal). A pin
// past that budget is init-fail and retried every second, so list the wired pins first.
// Which of the listed pins have an ESC on them is learned at run time (telemetry, reads, the
// passthrough's probe), not set here.
// ESC_BOARD_PANEL (env:main_panel) is the old prayer-wheel display board reused as a bench panel:
// Pico W + SC1602 LCD + PWM1..4 headers on GP16..19. Its LCD bus takes GP5..15, so GP10/11 are not
// free there.
#ifndef ESC_BOARD_PANEL
#define ESC_BOARD_PANEL   0
#endif
#if ESC_BOARD_PANEL
#define ESC_SIGNAL_PINS   { 16, 17, 18, 19 }   // PWM1..PWM4
#else
#define ESC_SIGNAL_PINS   { 10, 11 }
#endif

// --- Status LCD (16x2 HD44780, 8-bit bus, write-only) -------------------------------------------
// Shows each ESC's state, and for the armed one throttle / rpm / volts / current code / temp.
#ifndef ESC_LCD
#define ESC_LCD           ESC_BOARD_PANEL
#endif
#define ESC_LCD_RS        15
#define ESC_LCD_RW        14               // held LOW: the 5 V LCD never drives the bus
#define ESC_LCD_E         13
#define ESC_LCD_D         { 12, 11, 10, 9, 8, 7, 6, 5 }   // D0..D7

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

// --- Home Wi-Fi + CLI over TCP (net_console.h) ---------------------------------------------------
// The Pico joins the LAN as a station and serves the same CLI on ESC_TCP_PORT, next to USB:
// host tools take `--port socket://<ip>:2323`. Needs src/apps/wifi_secrets.h (gitignored; copy
// wifi_secrets.h.example). With it on, the web UI is served on the LAN address in SETUP mode
// instead of from the pico-esc-tool access point, and the radio stays on in DRIVE.
#ifndef ESC_WIFI_STA
#if ESC_BOARD_PANEL && __has_include("wifi_secrets.h")
#define ESC_WIFI_STA      1
#else
#define ESC_WIFI_STA      0
#endif
#endif
#define ESC_TCP_PORT      2323
#define ESC_HOSTNAME      "pico-esc"

// --- DShot / motor ----------------------------------------------------------------------------
#define ESC_DSHOT_KBAUD   300              // DShot bitrate in kbaud  [TEST: A_H_30 ESCs may need 300, not 600]
#define ESC_MOTOR_POLES   14               // motor magnet poles (for eRPM -> RPM); motor-dependent
