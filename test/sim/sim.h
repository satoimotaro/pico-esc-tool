// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// sim — a host simulation of the board under escs:: (src/apps/esc_session.h) and fourway.h: one clock,
// core0 and core1 as two cooperative contexts on it, the signal lines, and an ESC model on each pin.
// The firmware code is compiled unchanged against the stand-ins in stubs/.
//
// TIME. One clock in microseconds. A core runs until it waits (delay, delayMicroseconds, millis()
// and micros() count 1 us each); then whichever core is due first runs. So a core0 wait that pumps
// the other pins runs alongside a core1 bootloader op, as on the Pico, and every run is the same.
//
// ESC MODELS. What the firmware's timing rules are built on, as measured on the bench (2026-11-07,
// docs/esc-lifecycle.md "Bootloader entry timing") and read from the firmware sources:
//   Bluejay   boots for BOOT_MS (BOOT_TUNE_MS with its startup tune), deaf to the signal, then wants ARM_ZERO_MS of zero throttle and arms: from then
//             (wait_for_start) it sends telemetry on bidir frames and takes DShot commands. EDT
//             (command 13, 6 frames in a row) only in wait_for_start; cleared at the end of every run.
//             With FORCE_EDT_ARM a start without EDT is refused (it stays at 0 and waits again).
//             After valid frames, a line held high enters its bootloader at BL_QUIET_MS, unless a
//             BootInit came before that: then it stays in its app until it gets frames again.
//   BLHeli_S  the same entry and arming, normal DShot only, no telemetry; bidir frames are noise
//             (or, with Spec::bidirPrimes, count only as the frames before a bootloader entry).
//   AM32      boots for BOOT_MS, answers bidir frames as soon as it has booted (before it arms),
//             takes EDT any time; a line held high reboots it into its bootloader at AM32_BL_MS,
//             BootInit or not.
// In its bootloader an ESC answers BootInit with its 8-byte id and serves its settings block.
#pragma once
#include <cstdint>
#include <string>

namespace sim {

enum class Fw : uint8_t { BLUEJAY, BLHELI_S, AM32 };
struct Spec {
	Fw   fw = Fw::BLUEJAY;
	bool rev = false;          // 3D (bidirectional rotation) set in its settings
	bool forceEdtArm = false;  // Bluejay: FORCE_EDT_ARM
	bool tune = false;         // startup tune on: boots for BOOT_TUNE_MS instead of BOOT_MS (the bench ESCs: off)
	// BLHeli_S: inverted (bidir) frames count as the valid frames its bootloader entry needs.
	// Not measured: no stock BLHeli_S on the bench. The test runs both.
	bool bidirPrimes = false;
};
enum class Mode : uint8_t { OFF, BOOT, APP, BL };
struct Esc {
	bool     present = false;
	Spec     spec;
	Mode     mode = Mode::OFF;
	uint64_t bootEndUs = 0;
	bool     armed = false;     // in its app, armed (wait_for_start or running)
	bool     edt = false;
	bool     running = false;
	bool     stuck = false;     // Bluejay/BLHeli_S polled too early: stays in its app
	bool     hadFrames = false; // valid frames since its app started (BLHeli-S's entry needs them)
	float    rpm = 0;           // mechanical, signed
	uint16_t lastWire = 0;      // last throttle on the wire (0, or 48..2047)
	uint64_t zeroSinceUs = 0, lastFrameUs = 0;
	uint8_t  cmdRun = 0;
	uint32_t frames = 0, telePackets = 0, refusedStarts = 0, blEntries = 0;
	uint8_t  mem[256] = {0};    // its settings block (SiLabs 0x1A00, AM32 0x7C00)
	uint8_t  name[32] = {0};    // AM32 FILE_NAME (0x7C00 - 32)
	// telemetry waiting for the driver (only the newest is read, as the PIO FIFO drain does)
	uint8_t  teleType = 0xFF; uint32_t teleVal = 0;
};

static const uint32_t BOOT_MS = 150, BOOT_TUNE_MS = 1500, ARM_ZERO_MS = 300, BL_QUIET_MS = 700,
                      AM32_BOOT_MS = 150, AM32_BL_MS = 540, POLES = 14;

// The test drives these.
void   plug(uint8_t pin, const Spec& s);     // power one up (in its bootloader if the line is high)
void   unplug(uint8_t pin);
Esc&   esc(uint8_t pin);
void   startCore1(void (*loop)());           // loop1() body, called forever on core1
double seconds();
// Called for every DShot frame: pin, bidir, command, wire value (0 or 48..2047; or the command).
typedef void (*FrameHook)(uint8_t pin, bool bidir, bool command, uint16_t wire);
void   onFrame(FrameHook h);
uint16_t pwmWidth(uint8_t pin);              // last servo pulse width on the pin (0: none)

}  // namespace sim
