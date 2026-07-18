// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// main — the COMPOSITION ROOT of the integrated RP2040 ESC firmware (Pico W). You declare the ESCs
// here as Thruster objects (each with its own DShot bitrate, motor pole count, and calibrated speed
// profile), set their per-motor gains, and compose whatever surface you want on top:
//
//   * Tool build (below): hand the Thrusters to an EscTool — config/flash + USB-serial CLI
//     (host/esctool.py) + a Wi-Fi web UI. main stays tiny.
//   * Standalone ROV: DON'T create an EscTool. Drive the Thrusters directly from your own loop, e.g.
//       void loop() {
//         auto cmd = readCmdVel();                       // from the Pi / RC over serial
//         for (uint8_t i = 0; i < NTHR; i++) { THRUSTERS[i]->setRpm(cmd[i]); THRUSTERS[i]->poll(); }
//         escs::spinPoll();                              // one shared call keeps all DShot frames flowing
//       }
//
// The escs:: engine (esc_session.h) is the singleton hardware layer (2 PIO SMs + one core1 1-wire
// worker); each Thruster is a per-ESC object delegating to it by index. Add an ESC = add a pin to
// ESC_SIGNAL_PINS (esc_config.h) and a Thruster below.
#include "apps/thruster.h"
#include "apps/esc_tool_app.h"
#include "apps/profiles.h"

// --- Declare the ESCs (one Thruster each). pin = ESC_SIGNAL_PINS[bind index]; the rest is per-ESC.
//     esc1 = 930KV, SINE config (sine_mode=2): forced-sine gives a smooth low-speed start and the
//     firmware's virtual-eRPM telemetry lets the loop climb through sine into 6-step from rest. `rpm 0`
//     stops cleanly via the Thruster's signal-loss stop (thruster.h) — a 3D DShot 0 does NOT stop the
//     ESC otherwise. Apply the matching ESC config once:
//         esctool apply 1 host/profiles/rpm_930kv_sine2.yaml
//     (For a pure 6-step / no-sine setup use profiles::M_930KV_SINEOFF + host/profiles/rpm_930kv_sineoff.yaml.) ---
static Thruster esc0(&profiles::M_LINEAR, ESC_DSHOT_KBAUD, ESC_MOTOR_POLES);        // pin 10: RAW / uncalibrated
static Thruster esc1(&profiles::M_930KV_12N14P_6STEP, /*dshotKbaud=*/300, /*motorPoles=*/14);  // pin 11: 930KV sine closed-loop

static Thruster*    THRUSTERS[] = { &esc0, &esc1 };
static const uint8_t NTHR = sizeof(THRUSTERS) / sizeof(THRUSTERS[0]);

// --- The tool surface over those ESCs (config/flash + serial CLI + Wi-Fi). Omit for a bare ROV. ---
static EscTool tool(THRUSTERS, NTHR);

void setup() {
	for (uint8_t i = 0; i < NTHR; i++) THRUSTERS[i]->bind(i);   // attach each to escs:: index i

	// Per-motor closed-loop PI gains — from the same calibrated profile (the ~30x-hotter real plant
	// needs these, not the sim DEFAULT_GAINS). esc0 stays at library defaults.
	esc1.applyGains(profiles::M_930KV_12N14P_6STEP_GAINS);
	esc1.vc.slew_rpm_s = 4000.0f; esc1.vc.max_temp = 0.0f;   // non-PI controller settings
	esc1.vc.stop_below_rpm = 0.0f;   // only an exact rpm 0 stops (sine low-speed still runs); raise to
	                                 // cut off a marginal low band and treat it as stop.

	tool.begin();
}
void loop()   { tool.poll(); }        // serial + Wi-Fi + each Thruster's RPM loop + shared spinPoll
void setup1() {}
void loop1()  { tool.pollCore1(); }
