// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// main — the DEFAULT, integrated object-oriented RP2040 ESC firmware (Pico W). ONE build does every
// job the old "flash a separate app per task" model split across esc_tool.cpp + vel_demo.cpp:
//   * config / flash BLHeli-S ESCs over the 1-wire bootloader,
//   * DShot drive in RAW (direct thrust/throttle) OR RPM (closed-loop velocity) submode,
//   * a USB-serial CLI (host/esctool.py) AND, in SETUP mode, a Wi-Fi web UI.
//
// It is composed of objects: a single EscManager owns one Thruster per wired ESC (so the ESC count
// scales with ESC_SIGNAL_PINS in esc_config.h), the AS5600 encoder, the web server, and the flash
// state machine. Each Thruster delegates hardware to the proven escs:: singleton (esc_session.h) by
// index and carries its own closed-loop velocity controller (lib/vel_control). See esc_manager.h /
// thruster.h. The legacy per-app builds (esc_tool, vel is now folded in, spikes) remain as fallbacks.
#include "apps/esc_manager.h"

static EscManager mgr;

void setup()  { mgr.begin(); }
void loop()   { mgr.poll(); }
void setup1() {}
void loop1()  { mgr.pollCore1(); }
