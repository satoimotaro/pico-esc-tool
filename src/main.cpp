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
//       // in setup(): telemetry LOST while running keeps streaming (open loop) instead of stopping,
//       // and the zero-throttle hold (also hot-plug detection) is on:
//       //   escs::setPolicy(lc::Policy::ROV); escs::setHold(true);
//       void loop() {
//         auto cmd = readCmdVel();                       // from the Pi / RC over serial
//         for (uint8_t i = 0; i < NTHR; i++) { THRUSTERS[i]->setRpm(cmd[i]); THRUSTERS[i]->poll(); }
//         escs::spinPoll();                              // one shared call keeps all DShot frames flowing
//       }
//
// The escs:: engine (esc_session.h) is the singleton hardware layer (2 PIO SMs + one core1 1-wire
// worker); each Thruster is a per-ESC object delegating to it by index. There is one Thruster per
// possible ESC (escs::MAX); which GPIOs, and how many, is the pin list: ESC_SIGNAL_PINS (esc_config.h)
// by default, or `pins <gpio>...` at run time (stored, applied at the next boot). EscTool uses the
// first escs::COUNT of them.
#include "apps/thruster.h"
#include "apps/esc_tool_app.h"
#include "apps/profiles.h"

// --- Declare the ESCs (one Thruster each). pin = the pin list's [bind index]; the rest is per-ESC.
//     esc1 = 930KV, SINE config (sine_mode=2): forced-sine gives a smooth low-speed start and the
//     firmware's virtual-eRPM telemetry lets the loop climb through sine into 6-step from rest. `rpm 0`
//     stops cleanly via a proper NEUTRAL (thruster.h holds throttle 0 while armed; the BlueGill firmware
//     detects the near-zero magnitude and parks the ESC in wait_for_start) so the rotor really stops and
//     the next non-zero target restarts with no re-arm. Apply the matching ESC config once:
//         esctool apply 1 host/profiles/rpm_930kv_sine2.yaml
//     (For a pure 6-step / no-sine setup use profiles::M_930KV_SINEOFF + host/profiles/rpm_930kv_sineoff.yaml.) ---
static Thruster esc0(&profiles::M_LINEAR, ESC_DSHOT_KBAUD, ESC_MOTOR_POLES);        // ESC 0 (GP10 / PWM1): RAW / uncalibrated
static Thruster esc1(&profiles::M_930KV_12N14P_6STEP, /*dshotKbaud=*/300, /*motorPoles=*/14);  // ESC 1 (GP11 / PWM2): 930KV sine closed-loop
// ESC 2..7: RAW / uncalibrated, used when the pin list has that many pins (the panel's PWM3/PWM4 by default)
static Thruster esc2(&profiles::M_LINEAR, ESC_DSHOT_KBAUD, ESC_MOTOR_POLES);
static Thruster esc3(&profiles::M_LINEAR, ESC_DSHOT_KBAUD, ESC_MOTOR_POLES);
static Thruster esc4(&profiles::M_LINEAR, ESC_DSHOT_KBAUD, ESC_MOTOR_POLES);
static Thruster esc5(&profiles::M_LINEAR, ESC_DSHOT_KBAUD, ESC_MOTOR_POLES);
static Thruster esc6(&profiles::M_LINEAR, ESC_DSHOT_KBAUD, ESC_MOTOR_POLES);
static Thruster esc7(&profiles::M_LINEAR, ESC_DSHOT_KBAUD, ESC_MOTOR_POLES);
static Thruster*    THRUSTERS[escs::MAX] = { &esc0, &esc1, &esc2, &esc3, &esc4, &esc5, &esc6, &esc7 };
static const uint8_t NTHR = sizeof(THRUSTERS) / sizeof(THRUSTERS[0]);

// --- The tool surface over those ESCs (config/flash + serial CLI + Wi-Fi). Omit for a bare ROV. ---
static EscTool tool(THRUSTERS, NTHR);

void setup() {
	for (uint8_t i = 0; i < NTHR; i++) {
		THRUSTERS[i]->bind(i);                                  // attach each to escs:: index i
		// STAGED THERMAL PROTECTION, on EVERY thruster (the gains below are per-motor; a thermal limit
		// is not — an un-tuned ESC needs it at least as much). Read this first: the EDT temperature is
		// the ESC's OWN MCU, measured on the ESC board. It is NOT a winding sensor and NOT a motor
		// temperature — this hardware has no winding sensor at all — so treat it as a lagging proxy.
		// [60, 80] C sits entirely BELOW the ESC's own protection, whose lowest selectable onset is 80 C
		// (bluegill Layouts/Base.inc TEMP_LIMIT_2S/1S; Scheduler.asm then steps the PWM 100/80/60/40/20 %
		// in ~5 C stages), so the two can never fight. 60 C is also the most conservative host abort in
		// this repo (autocal.py / xover_debug.py --max-temp 60) and 80 C the driftsoak/velcal default, so
		// on the bench the HOST abort still fires first and this is only the backstop.
		// DELIBERATELY NOT PERSISTED: these are not in settings::Record, so `cfg save` cannot store a
		// loosened limit and a power cycle always restores this conservative band. Loosening it has to be
		// a conscious `gain <i> warntemp|maxtemp` each session. (It also avoids a CFG_VERSION v4 whose
		// migration would have to choose between "reproduce the old behaviour" and "safe default".)
		// NOTE: only the RPM submode runs vc.step(), so RAW drive (`thrust`/`throttle`) is UNPROTECTED.
		THRUSTERS[i]->vc.warn_temp      = vel::DEFAULT_TEMP_WARN_C;
		THRUSTERS[i]->vc.max_temp       = vel::DEFAULT_TEMP_ABORT_C;
		THRUSTERS[i]->vc.temp_min_scale = vel::DEFAULT_TEMP_MIN_SCALE;
	}

	// Per-motor closed-loop PI gains — from the same calibrated profile (the ~30x-hotter real plant
	// needs these, not the sim DEFAULT_GAINS). esc0 stays at library defaults.
	esc1.applyGains(profiles::M_930KV_12N14P_6STEP_GAINS);
	esc1.vc.slew_rpm_s = 2000.0f;   // 2000 balances overshoot vs. rise: a
	                       // 1500->3500 step went 40%->33% peak overshoot vs. 4000 for ~0.16 s more rise
	                       // (steady-state still 0.1%). Lower (~1200) cuts overshoot further but the
	                       // response gets sluggish; a prop under water load damps the spike anyway.
	esc1.vc.kd = 0.008f;   // derivative-on-measurement: damps steady-state hunting (bench: jitter down,
	                       // SS error 0.2%->0.07%); above ~0.02 the eRPM quantization adds jitter. It does
	                       // NOT fix the up-step overshoot on the unloaded bench (a low-inertia momentum
	                       // spike — lower `slew_rpm_s` for that; a prop load damps it). Tune: `gain 1 kd`.
	esc1.vc.stop_below_rpm = 0.0f;   // only an exact rpm 0 stops (sine low-speed still runs); raise to
	                                 // cut off a marginal low band and treat it as stop.

#if ESC_CONSOLE_UART
	Serial1.setTX(ESC_UART_TX);
	Serial1.setRX(ESC_UART_RX);
	Serial1.setFIFOSize(ESC_UART_FIFO);
	Serial1.begin(ESC_UART_BAUD);
	tool.begin(Serial1);                    // CLI over UART0 (see esc_config.h)
#else
	tool.begin();                           // CLI over USB CDC
#endif
}
void loop()   { tool.poll(); }        // serial + Wi-Fi + each Thruster's RPM loop + shared spinPoll
void setup1() {}
void loop1()  { tool.pollCore1(); }
