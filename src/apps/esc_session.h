// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// esc_session — shared, transport-agnostic BLHeli-S bootloader session + DShot drive for the RP2040
// ESC tool. core0 owns DShot (PIO) for bootloader entry and thruster drive; core1 owns the 1-wire
// bit-bang (blheli_bl). The unified `esc_tool` firmware drives this same API from both its transports
// (USB-serial parser and Wi-Fi HTTP handlers). Header-only: exactly one app translation unit compiles
// it per PlatformIO env, so the file-local statics have a single copy.
//
// A persistent SESSION holds one ESC in its bootloader (motor off) and reuses it across calls so
// config ops don't reboot the ESC each time; core1 keeps it alive. Call release() to restart it.
//
// LIFECYCLE (docs/esc-lifecycle.md, lib/esc_lifecycle): every pin has one lc::Pin state. spinPoll()
// feeds it telemetry and time, applies what it says may be sent (the idle hold: bidir zero on an
// unread pin, which also detects a plug-in; zero in the ESC's own protocol once READY), and reads a
// newly found ESC on core1 (Op::READID) while core0 keeps streaming. The explicit config ops stay
// synchronous, but every core0 wait pumps the other pins' frames, telemetry and deadman.
#pragma once
#include <Arduino.h>
#include <PIO_DShot.h>
#include <string.h>
#include "esc_config.h"
#include "blheli_bl.h"
#include "esc_setup.h"
#include "esc_lifecycle.h"
#include <hardware/pwm.h>
#include <hardware/clocks.h>
#include <hardware/sync.h>
#include <pico/time.h>

namespace escs {

static const uint8_t PINS[]  = ESC_SIGNAL_PINS;   // signal pin(s) per ESC — edit in esc_config.h
static const uint8_t COUNT   = sizeof(PINS) / sizeof(PINS[0]);

struct Info {
	bool     present = false;
	uint8_t  pin = 0;
	uint16_t sig = 0;
	uint8_t  bootVer = 0, bootPages = 0;
	char     layout[17] = {0};
	char     name[17]   = {0};
	uint8_t  fwMain = 0, fwSub = 0;
};

// Live DShot telemetry (Extended DShot Telemetry) for a spinning thruster.
struct Telem {
	bool     valid = false;
	uint32_t rpm = 0, current = 0, tempC = 0, stress = 0, status = 0;
	float    voltage = 0.0f;
	uint32_t rpmStampMs = 0;   // millis() of the last ERPM packet — lets a reader detect a STALE rpm
	                           // (the field holds the last 6-step value even after the ESC drops to
	                           // forced sine and stops sending eRPM; freshness distinguishes the two).
	uint32_t tempStampMs = 0;  // millis() of the last TEMPERATURE packet — same idea, but SEPARATE on
	                           // purpose: EDT temperature keeps arriving in forced sine (where eRPM does
	                           // not), and forced sine is the hottest regime. 0 = never seen one, which
	                           // is also the pre-arm state (EDT is only enabled after arming, below).
	uint32_t pktStampMs = 0;   // millis() of the last valid packet of any type (what LOST is timed on)
};

static const uint16_t SPIN_MAX        = 2000;   // max throttle
static const uint32_t SPIN_DEADMAN_MS = 500;    // auto-zero throttle if no command within this
static const uint32_t SPIN_ARM_MS     = 3000;   // stream zero throttle this long to ARM the ESC
                                                // (covers the ESC's boot beep after leaving the BL)
// A bidir ESC counts as armed only once it answers and has taken the EDT enable. Bluejay sends its
// first telemetry from wait_for_start, i.e. once it has armed, and takes DShot commands only there;
// with EDT safety arm (FORCE_EDT_ARM) set it refuses to start without EDT, beeps and waits again
// (Bluejay.asm wait_for_start_nonzero), and a reboot clears EDT. EDT used to go out only in a
// zero-throttle gap after the arm, which a motor test that sends throttle at once never leaves:
// the Bluejay then never started after a passthrough. An ESC that never answers is armed after
// SPIN_ARM_TELE_MAX_MS, as before.
static const uint32_t SPIN_ARM_TELE_HOLD_MS = 300;    // zero throttle after its first answer
static const uint32_t SPIN_ARM_TELE_MAX_MS  = 8000;
// A stalled Bluejay/BLHeli-derived bidir ESC is armed again on the same stream and held at 0 until it
// is sent 0 (lc::Stall: the rules, STALL_ZERO_MS / STALL_SILENT_MS / STALL_MAX / STALL_REST_MS).
static const uint16_t PWM_MIN_US      = 1000;   // servo PWM pulse range
static const uint16_t PWM_MAX_US      = 2000;
static const uint16_t PWM_STOP_US     = 1000;   // zero throttle / arming pulse, one-way ESC
static const uint16_t PWM_MID_US      = 1500;   // the same on a 3D ESC (1000 us is full reverse there)
static const uint32_t PWM_ARM_MS      = 5000;   // hold PWM_STOP_US this long after switching to PWM
static const uint32_t PWM_RELEASE_MS  = 3000;   // after the arm, unfed this long: the pin leaves PWM
static const uint16_t MOTOR_POLES     = ESC_MOTOR_POLES;   // eRPM->RPM (see esc_config.h)

// Per-ESC DShot mode. BLHeli-S understands only *normal* DShot; *bidir* DShot (inverted, with eRPM/
// EDT telemetry back over the wire) needs firmware that supports it (Bluejay/JESC). AUTO picks bidir
// if the firmware name says so, else normal — so any stock BLHeli-S ESC still spins.
enum class Drive : uint8_t { AUTO, NORMAL, BIDIR };

namespace detail {
	static const uint32_t DSHOT_KBAUD = ESC_DSHOT_KBAUD, PRIME_MS = 500, HIGH_HOLD_MS = 1000,
	                      AM32_HOLD_MS = 4000;
	static BidirDShotX1*          dsh = nullptr;                              // core0 only (PIO)
	static blheli_bl::Bootloader  bl({ .signalPin = PINS[0], .baud = 0 });    // core1 only

	enum class Op : uint8_t { NONE, CONNECT, READCFG, READPAGE, WRITEPAGE, ERASE, WRITEFLASH, READFLASH, RUN, PROBE,
	                          FWSELECT, FWRUN, LEARN, READID };
	// One mailbox for the synchronous ops (runOp) and the one asynchronous job (READID): runOp first
	// collects or aborts the job, so the two never overlap.
	static volatile Op       op = Op::NONE;
	static volatile bool     opDone = false, opOk = false;
	static volatile uint8_t  opPin = PINS[0], opIdx = 0;
	static volatile uint8_t  sig[2] = {0, 0};
	static volatile uint8_t  bootVer = 0, bootPages = 0;
	static uint32_t prHoldMs = 0, prEdges = 0, prLows = 0, prTotal = 0;   // Op::PROBE in/out
	static uint8_t  prRaw[8] = {0}; static int prN = 0;
	static uint8_t  cfg[esc_setup::kEepromLen];
	static uint8_t  page[esc_setup::kPageLen];
	static uint8_t  flBuf[256];
	static volatile uint16_t flAddr = 0, flLen = 0;
	static volatile int8_t   session = -1;
	static uint8_t  lastRaw[8] = {0}; static bool lastAm32 = false;   // the last CONNECT's raw reply
	// The session's own state. A write sets sessionWrote, and closing the session then reads the
	// identity again (Op::LEARN) before RUN, so nothing after a write runs on an assumed identity.
	// core1 counts keepalives: two failed in a row = sessionLost, and the next op re-enters with a
	// fresh CONNECT (fresh signature), so a write never goes to whatever replaced the ESC.
	static bool          sessLearned = false, sessionWrote = false;
	static uint32_t      sessLastOpMs = 0;             // last op on the held session (schedule's idle rule)
	static int8_t        armBusy = -1;                 // the written session that refused the last spinArm
	static volatile bool sessionLost = false;
	static uint8_t       kaMiss = 0;                   // core1
	static Info          infoCache[COUNT];             // Op::LEARN results for the session's pin
	static lc::Identity  identCache[COUNT];
	// The async read (Op::READID) writes only these; core0 adopts them when it collects the job.
	static Info          jobInfo;
	static lc::Identity  jobIdent;
	static volatile bool jobAbort = false;
	static int8_t        jobIdx = -1;                  // core0: the pin being primed / read
	static uint8_t       jobPhase = 0;                 // 0 none, 1 prime (core0), 2 core1 runs READID
	static uint8_t       jobRr = 0;                    // round robin over PENDING pins
	// LittleFS writes pause the other core (flash erase), which would cut a bit-banged bootloader
	// byte in half: core1 parks in its idle branch first (FsGuard).
	static volatile bool parkReq = false, parked = false, core1Up = false;
	// 4-way passthrough (fourway.h): an ESC once connected stays in its bootloader while another is
	// selected, so coming back to it re-adopts its cached reply instead of a second BootInit (which
	// AM32 does not answer).
	static bool     fwHave[COUNT] = { false };
	static uint8_t  fwRaw[COUNT][8];
	static bool     fwAm32[COUNT] = { false };
	static volatile uint8_t fwIdx = 0;
	// An ESC that never answered stays failed until the next fwBegin, so a configurator polling every
	// reported channel pays the timeout once per session, not on every read.
	static bool     fwMiss[COUNT] = { false };
	static bool     fwDone[COUNT] = { false };   // fwRun already left its bootloader (DeviceReset)
	static bool     fwActive = false;            // the configurator owns every pin: no hold, no jobs
	static uint32_t fwBeginMs = 0;
	// Which of PINS have an ESC on them, as last found by a scan (EscTool keeps it in LittleFS).
	// Only these are offered to a configurator. Bit i = PINS[i]; all set until something is known.
	static uint32_t present = ~0u;
	static inline bool armId(const uint8_t raw[8]) { return raw[5] == 0x06 && raw[4] > 0 && raw[4] < 0x90; }
	// The lifecycle: one state per pin, and what the hold / LOST handling do with it.
	static lc::Pin    pin[COUNT];
	static lc::Policy policy = lc::Policy::TOOL;
	static bool       holdOn = false;              // EscTool turns it on; the legacy esc_tool keeps it off
	static uint8_t    evIdx[8]; static lc::Ev evEv[8]; static lc::State evSt[8];
	static uint8_t    evHead = 0, evTail = 0;
	static inline void evPush(uint8_t i, lc::Ev e) {   // with the state it left, not the state at print time
		if (e == lc::Ev::NONE) return;
		uint8_t n = (evHead + 1) & 7;
		if (n == evTail) evTail = (evTail + 1) & 7;   // full: the oldest goes
		evIdx[evHead] = i; evEv[evHead] = e; evSt[evHead] = pin[i].st; evHead = n;
	}
	static bool presentAdded = false;              // an async read found an ESC on an unlisted pin
	// drive/spin: one DShot driver per pin (core0). Only one of drvB/drvN is non-null per pin.
	static BidirDShotX1*     drvB[COUNT] = { nullptr };   // bidir DShot: idle HIGH, eRPM/EDT telemetry
	static DShotX4*          drvN[COUNT] = { nullptr };   // normal DShot: idle LOW, throttle only
	static volatile uint16_t drvTarget[COUNT] = { 0 };
	static uint32_t          drvLast[COUNT] = { 0 };
	static bool              drvArmed[COUNT] = { false };
	static uint32_t          drvArmStart[COUNT] = { 0 };
	static uint16_t          drvArmMs[COUNT] = { 0 };   // this arm's window (0 => SPIN_ARM_MS default);
	static uint32_t          drvAnswer[COUNT] = { 0 };  // this arm's first telemetry (0 = none yet)
	                                                    // a re-arm after a run-mode stop can be short
	static uint8_t           drvEdt[COUNT] = { 0 };       // frames of EDT-enable left to send (bidir arm)
	static bool              drvRan[COUNT] = { false };   // a non-zero target since the last EDT burst
	static uint32_t          drvEdtMs[COUNT] = { 0 };     // last EDT burst while stopped
	static const uint32_t    EDT_REFRESH_MS = 1000;
	static lc::Stall         stall[COUNT];                // the stall watch (streamPin) and its hold
	static bool              drvInitFail[COUNT] = { false }; // the last driver start found no free PIO SM
	static uint32_t          drvRetryMs[COUNT] = { 0 };   // ... the hold tries again from here
	static uint32_t          drvStartMs[COUNT] = { 0 };   // when this stream started (the prime counts it)
	static uint32_t          drvPollMs[COUNT] = { 0 };    // when its telemetry was last polled
	static uint16_t          drvKb[COUNT] = { 0 };        // the bitrate it was started at
	static Telem             drvTele[COUNT];
	static Drive             armMode[COUNT] = { Drive::AUTO };   // what the pending/last arm asked for
	static uint16_t          armMsReq[COUNT] = { 0 };
	static bool              armStart[COUNT] = { false };  // RUNNING after a read: start the arm stream
	// Per-ESC overrides (0 = use the global ESC_DSHOT_KBAUD / ESC_MOTOR_POLES default). Set via
	// escs::setKbaud/setPoles so different ESCs/motors on the same board can differ (e.g. a 300 kbaud
	// A_H_30 next to a 600 kbaud unit, or motors with different pole counts). Additive: untouched =>
	// identical behavior to before, so the legacy esc_tool build is unaffected.
	static uint16_t          drvKbaud[COUNT] = { 0 };
	static uint8_t           drvPoles[COUNT] = { 0 };
	static inline uint16_t   kbaudOf(uint8_t i) { return drvKbaud[i] ? drvKbaud[i] : (uint16_t)DSHOT_KBAUD; }
	static inline uint8_t    polesOf(uint8_t i) { return drvPoles[i] ? drvPoles[i] : (uint8_t)MOTOR_POLES; }
	// Servo PWM (50 Hz, 1000..2000 us) instead of DShot, for firmwares that take it (AM32, stock
	// BLHeli-S; Bluejay does not). Hardware PWM slice, not PIO. pwmUs = 0: the pin is not on PWM.
	static volatile uint16_t pwmUs[COUNT] = { 0 }, pwmStop[COUNT] = { 0 };   // read by the deadman IRQ; pwmStop: 1500 on a 3D ESC
	static volatile uint32_t pwmLast[COUNT] = { 0 };
	static uint32_t          pwmStart[COUNT] = { 0 };
	static bool              pwmZero[COUNT] = { false };   // a stop width came after the arm (lc::pwmGate)
	static uint8_t           blDepth = 0;                  // inside a sync bootloader visit: PWM sends its stop width
	static inline bool pwmOn(uint8_t i) { return pwmUs[i] != 0; }
	// IRQs off: a slice's two pins share one CC register, and the deadman alarm below writes it too.
	static inline void pwmLevel(uint8_t i, uint16_t us) {
		uint32_t s = save_and_disable_interrupts(); pwm_set_gpio_level(PINS[i], us); restore_interrupts(s);
	}
	// The slice keeps its last width whatever core0 does, so the deadman also runs from a timer IRQ:
	// a loop held up (a blocking LittleFS write, a hang) still falls back to the stop width after 500 ms.
	static repeating_timer_t pwmTimer; static bool pwmTimerOn = false;
	static bool pwmWatch(repeating_timer_t*) {
		for (uint8_t i = 0; i < COUNT; i++)
			if (pwmOn(i) && millis() - pwmLast[i] > SPIN_DEADMAN_MS) pwm_set_gpio_level(PINS[i], pwmStop[i]);
		return true;
	}
	static inline void pwmSafe() { for (uint8_t i = 0; i < COUNT; i++) if (pwmOn(i)) pwmLevel(i, pwmStop[i]); }
	static inline void pwmOff(uint8_t i) {
		if (!pwmOn(i)) return;
		pwmLevel(i, 0); pwmUs[i] = 0;
		gpio_set_function(PINS[i], GPIO_FUNC_SIO); pinMode(PINS[i], INPUT);
	}

	static inline uint8_t idxOf(uint8_t p) { for (uint8_t i = 0; i < COUNT; i++) if (PINS[i] == p) return i; return 0; }
	// The identity that applies to pin i now: the session's own LEARN while it is held, else the pin's.
	static inline const lc::Identity& curIdent(uint8_t i) {
		return (session == (int8_t)i && sessLearned) ? identCache[i] : pin[i].id;
	}
	static inline bool drvActive(uint8_t i) { return drvB[i] || drvN[i]; }
	static inline void drvSend(uint8_t i, uint16_t t) {
		if (drvB[i]) drvB[i]->sendThrottle(t);
		else if (drvN[i]) { uint16_t a[4] = { t, 0, 0, 0 }; drvN[i]->sendThrottles(a); }
	}
	static inline void drvFree(uint8_t i) {
		pwmOff(i);                                   // whoever takes the pin next owns it alone
		if (drvB[i]) { drvB[i]->sendThrottle(0); delete drvB[i]; drvB[i] = nullptr; }
		if (drvN[i]) { uint16_t a[4] = { 0, 0, 0, 0 }; drvN[i]->sendThrottles(a); delete drvN[i]; drvN[i] = nullptr; }
		pin[i].onStreamEnd();
	}
	// PIO SM budget. The Pico W's CYW43 radio claims, at boot (before any stream here), the
	// highest-numbered PIO with room (pio1: 1 SM + 6 instructions), and the bidir program (28 of 32
	// instructions) does not fit beside it: on a Pico W, pio0's 4 SMs are the only bidir capacity (the
	// pio1 fallback can only help a board without the radio). So bidir goes to pio0 first, and normal DShot (4 instructions) to pio1 first, leaving
	// pio0's SMs to bidir. One stream per pin (a prime replaces its pin's stream, never adds one). The
	// result may still carry initError(): every caller checks it.
	static BidirDShotX1* newBidir(uint8_t p, uint16_t kb) {
		BidirDShotX1* d = new BidirDShotX1(p, kb, pio0);
		if (d->initError()) { delete d; d = new BidirDShotX1(p, kb, pio1); }
		return d;
	}
	static DShotX4* newNormal(uint8_t p, uint16_t kb) {
		DShotX4* d = new DShotX4(p, 1, kb, pio1);
		if (d->initError()) { delete d; d = new DShotX4(p, 1, kb, pio0); }
		return d;
	}
	// Start a fresh zero stream on pin i (arm window from now). false: no PIO SM — nothing is left on
	// the pin, its state stays as it is, and spinMode() says init-fail.
	static bool drvStart(uint8_t i, bool bidir) {
		drvFree(i);
		uint16_t kb = kbaudOf(i);
		uint32_t now = millis();
		if (bidir) { drvB[i] = newBidir(PINS[i], kb); if (drvB[i]->initError()) { delete drvB[i]; drvB[i] = nullptr; } }
		else       { drvN[i] = newNormal(PINS[i], kb); if (drvN[i]->initError()) { delete drvN[i]; drvN[i] = nullptr; } }
		if (!drvActive(i)) {
			if (!drvInitFail[i]) evPush(i, lc::Ev::INIT_FAIL);
			drvInitFail[i] = true; drvRetryMs[i] = now + 1000;
			return false;
		}
		drvInitFail[i] = false; drvKb[i] = kb;
		drvTarget[i] = 0; drvArmed[i] = false; drvArmStart[i] = drvLast[i] = drvStartMs[i] = drvPollMs[i] = now;
		drvArmMs[i] = 0; drvAnswer[i] = 0; drvEdt[i] = bidir ? 20 : 0; stall[i].reset();
		pin[i].onStreamStart(now, bidir);
		return true;
	}

	static const uint32_t POLL_GAP_MS = 50;      // a pin not polled this long: core0 was away
	static const uint8_t  SLOW_GAPS = 5;         // gaps in a row: say so (Ev::SLOW_LOOP), once per run
	static uint8_t        gapRun[COUNT] = { 0 };
	// A loop held up (a blocking op that does not pump) is not an ESC gone quiet: the lifecycle's
	// clocks move on by the gap, and the stall clocks restart. A loop slow on every pass would keep
	// LOST, SILENT and the stall's silence rule from ever firing, so repeated gaps are reported. Only
	// spinPoll's own check counts them (count): the streamPin/pump checks right after it see no gap
	// and must not reset the run.
	static void gapCheck(uint8_t i, uint32_t now, bool count = false) {
		bool gap = drvActive(i) && now - drvPollMs[i] > POLL_GAP_MS;
		if (gap) { pin[i].rebase(now - drvPollMs[i]); stall[i].onGap(); }
		if (count && drvActive(i)) {
			if (!gap) gapRun[i] = 0;
			else if (gapRun[i] < 255 && ++gapRun[i] == SLOW_GAPS) evPush(i, lc::Ev::SLOW_LOOP);
		}
		drvPollMs[i] = now;
	}
	// One loop's worth of pin i's stream: arm window, deadman, telemetry, stall watch, EDT, one frame.
	// Shared by spinPoll() and pump(), so a synchronous wait streams exactly what the loop would.
	static void streamPin(uint8_t i) {
		if (!drvActive(i)) return;
		uint32_t now = millis();
		gapCheck(i, now);
		if (pin[i].st != lc::State::RUNNING) drvTarget[i] = 0;   // only RUNNING ever sends non-zero
		if (!drvArmed[i]) {
			drvTarget[i] = 0;
			uint32_t t = millis() - drvArmStart[i], win = drvArmMs[i] ? drvArmMs[i] : SPIN_ARM_MS;
			// an explicit (short) window is a re-arm of an ESC that is armed already
			bool heard = !drvB[i] || drvArmMs[i] ||
			             (drvAnswer[i] && millis() - drvAnswer[i] >= SPIN_ARM_TELE_HOLD_MS && drvEdt[i] == 0);
			if (t > win && (heard || t > SPIN_ARM_TELE_MAX_MS)) {
				drvArmed[i] = true;
				if (drvB[i]) drvEdt[i] = 20;   // again once armed and stopped: AM32 may answer before it arms
			}
		}
		else if (millis() - drvLast[i] > SPIN_DEADMAN_MS) drvTarget[i] = 0;   // deadman (armed only)
		if (drvB[i]) {
			uint32_t v = 0;
			BidirDshotTelemetryType ty = drvB[i]->getTelemetryPacket(&v);
			switch (ty) {
				case BidirDshotTelemetryType::ERPM:        { uint8_t pp = polesOf(i); drvTele[i].rpm = (pp > 1) ? v / (pp / 2) : v; } drvTele[i].valid = true; drvTele[i].rpmStampMs = millis(); if (!drvAnswer[i]) drvAnswer[i] = lc::stamp(millis()); break;
				case BidirDshotTelemetryType::VOLTAGE:     drvTele[i].voltage = (float)v / 4.0f; break;
				case BidirDshotTelemetryType::CURRENT:     drvTele[i].current = v; break;
				case BidirDshotTelemetryType::TEMPERATURE: drvTele[i].tempC = v; drvTele[i].tempStampMs = millis(); break;
				case BidirDshotTelemetryType::STRESS:      drvTele[i].stress = v & ESC_STATUS_MAX_STRESS_MASK; break;
				case BidirDshotTelemetryType::STATUS:      drvTele[i].status = v; break;
				default: break;
			}
			// Any valid packet is a sign of life (EDT keeps coming in forced sine, where eRPM does not).
			if (ty != BidirDshotTelemetryType::CHECKSUM_ERROR && ty != BidirDshotTelemetryType::NO_PACKET) {
				drvTele[i].pktStampMs = now;
				evPush(i, pin[i].onPacket(now));
				// an unconfirmed identity's first answer sends the pin to PENDING: nothing more this frame
				if (pin[i].st != lc::State::RUNNING) { drvTarget[i] = 0; armStart[i] = false; }
			}
			// Stall watch (lc::Stall): armed, under throttle, on a stream that has answered, Bluejay/
			// BLHeli-derived (AM32 stays in its run). Its silence there is the watch's, not LOST
			// (lc::Pin::stallWatch). ZERO / QUIET arm the same stream again (zero, answer, EDT: Bluejay
			// cleared it) with the target held at 0; SILENT is LOST (TOOL) or the open loop (ROV).
			bool watched = pin[i].stallWatched(drvArmed[i], drvTarget[i]);
			lc::Ev se;
			lc::StallAct act = pin[i].onStall(stall[i].step(now, watched, ty == BidirDshotTelemetryType::ERPM, v, pin[i].pktMs),
			                                  policy, now, se);
			evPush(i, se);
			if (act == lc::StallAct::REARM) {
				stall[i].restart(now);
				drvArmed[i] = false; drvArmStart[i] = now; drvArmMs[i] = 0; drvAnswer[i] = 0;
				drvEdt[i] = 20; drvTarget[i] = 0; watched = false;
			} else if (act == lc::StallAct::LOST) {   // TOOL: stop that ESC only, as LOST_STOPPED in spinPoll
				drvTarget[i] = 0; armStart[i] = false; stall[i].reset(); watched = false;
			}
			pin[i].stallWatch = watched;
			// Enable EDT (extended telemetry: V/A/temp) once armed and still stopped — the ESC ignores
			// commands during its post-bootloader boot beep, so this must come after the arm window.
			// While arming that is once it has answered (above); otherwise once armed and stopped.
			bool edtNow = drvArmed[i] ? drvTarget[i] == 0 : drvAnswer[i] != 0;
			// Bluejay clears EDT at the end of every run (Bluejay.asm exit_run_mode_no_stall), and with
			// EDT safety arm (FORCE_EDT_ARM) the next start is then refused: every other start beeped and was
			// re-armed as a stall. So EDT goes out again whenever the throttle is back at 0 after a
			// run, and every EDT_REFRESH_MS while stopped (a command frame in place of a zero frame).
			if (drvArmed[i] && drvTarget[i] == 0 && drvEdt[i] == 0 &&
			    (drvRan[i] || now - drvEdtMs[i] >= EDT_REFRESH_MS)) { drvEdt[i] = 10; drvRan[i] = false; drvEdtMs[i] = now; }
			if (drvTarget[i]) drvRan[i] = true;
			if (edtNow && drvEdt[i] > 0) { drvB[i]->sendRaw11Bit(13); drvEdt[i]--; return; }
		}
		drvSend(i, drvTarget[i]);
	}
	static void pwmPoll() {
		for (uint8_t i = 0; i < COUNT; i++) {
			if (!pwmOn(i)) continue;
			uint32_t now = millis();
			bool arming = now - pwmStart[i] < PWM_ARM_MS, dead = now - pwmLast[i] > SPIN_DEADMAN_MS;
			// nothing is fed during a sync bootloader visit (core0 waits on core1): the stop width
			pwmLevel(i, arming || dead || blDepth ? pwmStop[i] : pwmUs[i]);
		}
	}
	// What every core0 wait runs instead of delay(): the other pins' frames, telemetry, deadman, stall
	// watch and PWM. Telemetry can still move a pin's state (onPacket: EMPTY/LOST -> PENDING; a stall
	// verdict: re-arm, LOST) and push events, but there are no ticks, no scheduling, no jobs and no
	// driver created or freed, so it may run inside runOp anywhere.
	static void pump() {
		for (uint8_t i = 0; i < COUNT; i++) streamPin(i);
		pwmPoll();
	}
	static void pumpFor(uint32_t ms) { uint32_t t0 = millis(); while (millis() - t0 < ms) { pump(); delayMicroseconds(200); } }

	// core0: adopt the finished READID. Never creates a driver: a RUNNING result (the arm waited for
	// this read) is flagged, and spinPoll starts the arm stream.
	static void jobCollect() {
		__sync_synchronize();                        // opDone seen: core1's results are complete
		uint8_t i = (uint8_t)jobIdx; bool ok = opOk, aborted = jobAbort;
		op = Op::NONE; jobIdx = -1; jobPhase = 0; jobAbort = false;
		if (aborted && !ok) { pin[i].onReadAborted(); return; }
		lc::Ev e = pin[i].onReadDone(ok, jobIdent);
		if (ok) {
			infoCache[i] = jobInfo; identCache[i] = jobIdent;
			if (!(present >> i & 1)) { present |= 1u << i; presentAdded = true; }   // found on an unlisted pin
		}
		evPush(i, e);
		if (pin[i].st == lc::State::RUNNING) armStart[i] = true;
	}
	// core0: end the async job now: a prime is just dropped (the pin stays PENDING), a core1 read is
	// aborted (abort) or waited for, the other pins streaming meanwhile.
	static void jobStop(bool abort) {
		if (jobIdx < 0) return;
		if (jobPhase == 1) { jobIdx = -1; jobPhase = 0; return; }
		if (abort) jobAbort = true;
		while (!opDone) { pump(); delayMicroseconds(200); }
		jobCollect();
	}
	static bool runOp(Op o, uint8_t gpio) {         // core0: hand an op to core1, pump till done
		jobStop(true);                               // a user op takes the line from a background read
		blDepth++; pwmSafe();                        // nothing is fed while this waits
		opIdx = idxOf(gpio); opPin = gpio; opOk = false; opDone = false; op = o;
		while (!opDone) { pump(); delayMicroseconds(200); }
		__sync_synchronize();                        // opDone seen: core1's results are complete
		if (session >= 0 && opIdx == (uint8_t)session) sessLastOpMs = millis();
		blDepth--;
		op = Op::NONE; return opOk;
	}
	static bool enterBootloader(uint8_t gpio, int tries = 3) {   // DShot-prime -> signal-loss -> connect (retry)
		blDepth++; pwmSafe();
		bool in = false;
		for (int a = 0; a < tries && !in; a++) {
			// AM32 only reboots into its bootloader after ~2 s without a valid frame, plus its startup
			// tune, and its bootloader bails back to the app on garbage. So prime only on the first
			// (BLHeli-S) try; later tries just hold the line high long enough to straddle that reboot.
			if (a == 0) {
				dsh = newBidir(gpio, DSHOT_KBAUD);   // the pin's own stream was freed: no extra SM
				uint32_t t0 = millis();
				while (millis() - t0 < PRIME_MS) { if (!dsh->initError()) dsh->sendThrottle(0); pump(); delayMicroseconds(300); }
				delete dsh; dsh = nullptr;
			}
			pinMode(gpio, OUTPUT); digitalWrite(gpio, HIGH); pumpFor(a == 0 ? HIGH_HOLD_MS : AM32_HOLD_MS);
			in = runOp(Op::CONNECT, gpio);
		}
		blDepth--;
		return in;
	}
	static bool learnSession(uint8_t i) {          // identity + Info of the session's ESC, read now
		sessLearned = runOp(Op::LEARN, PINS[i]);
		if (sessLearned) sessionWrote = false;
		return sessLearned;
	}
	// Leave the session: learn it first if it was written (or never learned), then run its app.
	static void closeSession() {
		if (session < 0) return;
		uint8_t i = (uint8_t)session;
		if (fwActive) { runOp(Op::RUN, PINS[i]); session = -1; return; }   // the passthrough maps its own
		// A lost session (keepalives failed: swapped or reset behind it) never closes on what it learned
		// before: it goes PENDING and is read again.
		bool ok = sessLearned && !sessionWrote && !sessionLost;
		if (!ok && !sessionLost) ok = learnSession(i);
		runOp(Op::RUN, PINS[i]);
		session = -1; sessLearned = sessionWrote = false;
		pin[i].onSessionClose(ok, identCache[i]);
		if (pin[i].st == lc::State::RUNNING) armStart[i] = true;   // an arm waited for this read
	}
	static bool ensureConnected(uint8_t i) {        // reuse the session; enter only if needed
		jobStop(true);                               // core1 may be reading this very pin: take it back first
		if (session == (int8_t)i) {
			if (!sessionLost) return true;
			if (fwActive) return false;          // the configurator selects it again; never re-enter under it
			// Stopped answering: a fresh CONNECT and signature. RUN first is a mailbox round trip, so
			// core1 is off the line (and out of its keepalive) before core0 primes the pin.
			runOp(Op::RUN, PINS[i]);
			session = -1; sessLearned = false;
		}
		if (session >= 0) closeSession();
		if (drvActive(i) || pwmOn(i)) drvFree(i);                                 // free pin from drive
		armStart[i] = false;
		if (!fwActive) pin[i].onSessionOpen(millis());
		if (enterBootloader(PINS[i])) { session = (int8_t)i; sessLearned = sessionWrote = false; sessLastOpMs = millis(); return true; }
		if (!fwActive) pin[i].onReadDone(false, lc::Identity{});
		return false;
	}
	static bool park() {                            // core0: FsGuard
		if (!core1Up) return true;                   // core1 not polling yet (boot): nothing on the line
		if (op != Op::NONE && !opDone) return false; // an op is in flight: refuse at once, never wait on it
		parkReq = true;
		uint32_t t0 = millis();
		while (!parked) {                            // core1 is mid-keepalive: a few ms, streams pumped
			if (millis() - t0 > 300) { parkReq = false; return false; }
			pump(); delayMicroseconds(200);
		}
		return true;
	}
}

// ---- transport-agnostic API (call from core0) --------------------------------------------------

// Connect ESC idx (via the session, no reboot) and fill its identity. false if it won't enter BL.
inline bool connect(uint8_t idx, Info& out) {
	using namespace detail;
	out = Info{}; if (idx >= COUNT) return false; out.pin = PINS[idx];
	if (!ensureConnected(idx)) return false;
	// Always read it now: dev| and the flasher's compatibility check see the live ESC, never a cache.
	if (learnSession(idx)) { out = infoCache[idx]; return true; }
	out.present = true; out.sig = (uint16_t)((sig[0] << 8) | sig[1]);   // answered, settings unread
	out.bootVer = bootVer; out.bootPages = bootPages;
	return true;
}
inline bool scan(uint8_t idx, Info& out) { return connect(idx, out); }

// Read the 255-byte config block into out255. Requires idx connectable.
inline bool readConfig(uint8_t idx, uint8_t* out255) {
	using namespace detail;
	if (idx >= COUNT || !ensureConnected(idx) || !runOp(Op::READCFG, PINS[idx])) return false;
	memcpy(out255, cfg, esc_setup::kEepromLen);
	return true;
}

// Apply (offset,value) overrides to the config page with a flash-wear guard (skip write if the
// bytes already match). Returns: 1 written+verified, 0 unchanged (no write), <0 error.
inline int editConfig(uint8_t idx, const uint16_t* offs, const uint8_t* vals, int n, bool& changed) {
	using namespace detail;
	changed = false;
	if (idx >= COUNT || !ensureConnected(idx)) return -1;
	if (!runOp(Op::READPAGE, PINS[idx])) return -2;
	for (int k = 0; k < n; k++) {
		if (offs[k] >= esc_setup::kPageLen) return -3;
		if (page[offs[k]] != vals[k]) { page[offs[k]] = vals[k]; changed = true; }
	}
	if (!changed) return 0;
	sessionWrote = true;                  // config changed -> read again (direction) before it runs
	return runOp(Op::WRITEPAGE, PINS[idx]) ? 1 : -4;
}

// Raw flash primitives (for firmware flashing). Each requires idx connectable.
inline bool erasePage (uint8_t idx, uint16_t addr) {
	using namespace detail;
	if (idx >= COUNT || !ensureConnected(idx)) return false;
	sessionWrote = true;                  // flashing -> firmware (bidir/normal) may change
	flAddr = addr; return runOp(Op::ERASE, PINS[idx]);
}
inline bool writeFlash(uint8_t idx, uint16_t addr, const uint8_t* data, uint16_t len) {
	using namespace detail;
	if (idx >= COUNT || len > sizeof(flBuf) || !ensureConnected(idx)) return false;
	sessionWrote = true;
	memcpy(flBuf, data, len); flAddr = addr; flLen = len;
	return runOp(Op::WRITEFLASH, PINS[idx]);
}
inline bool readFlash (uint8_t idx, uint16_t addr, uint8_t* out, uint16_t len) {
	using namespace detail;
	if (idx >= COUNT || len > sizeof(flBuf) || !ensureConnected(idx)) return false;
	flAddr = addr; flLen = len;
	if (!runOp(Op::READFLASH, PINS[idx])) return false;
	memcpy(out, flBuf, len); return true;
}

// Bring-up diagnostic, no session: drive the line high for holdMs (straddles an ESC reboot into its
// bootloader), then send BootInit and report the raw reply bytes and the line activity after a
// second BootInit. Tells "ESC never answers" apart from "answers but we mis-decode".
inline int probe(uint8_t idx, uint32_t holdMs, uint8_t raw[8], uint32_t& edges, uint32_t& lows, uint32_t& total) {
	using namespace detail;
	if (idx >= COUNT) return -1;
	jobStop(true); closeSession();
	if (drvActive(idx) || pwmOn(idx)) drvFree(idx);
	pin[idx].onSessionOpen(millis());
	prHoldMs = holdMs;
	runOp(Op::PROBE, PINS[idx]);
	pin[idx].onSessionClose(false, lc::Identity{});   // whatever it left behind is read again
	memcpy(raw, prRaw, 8); edges = prEdges; lows = prLows; total = prTotal; return prN;
}

inline bool connected(uint8_t idx) { return detail::session == (int8_t)idx; }
inline bool sessionOpen() { return detail::session >= 0; }   // some ESC is held in its bootloader
inline void release() { detail::closeSession(); }


// Per-ESC drive config (call BEFORE spinArm; takes effect at the next arm). 0 restores the global
// ESC_DSHOT_KBAUD / ESC_MOTOR_POLES default. Lets a Thruster carry its own bitrate / pole count.
inline void setKbaud(uint8_t idx, uint16_t kbaud) { if (idx < COUNT) detail::drvKbaud[idx] = kbaud; }
inline void setPoles(uint8_t idx, uint8_t poles)  { if (idx < COUNT) detail::drvPoles[idx] = poles; }

// ---- lifecycle (core0) -------------------------------------------------------------------------
// The idle hold: zero throttle on every wired pin that is not driven (bidir until it is read, then
// its own protocol). It is also the hot-plug detector, so with it off a plug-in is not seen either.
inline void setHold(bool on) {
	using namespace detail;
	holdOn = on;
	if (!on) for (uint8_t i = 0; i < COUNT; i++)   // at once, not on the next poll: `idle off` reads held=0
		if (drvActive(i) && pin[i].st != lc::State::RUNNING && (int8_t)i != jobIdx) drvFree(i);
}
inline bool holdActive() { return detail::holdOn; }
inline uint32_t heldMask() {   // pins carrying a zero stream the hold owns
	uint32_t m = 0;
	for (uint8_t i = 0; i < COUNT; i++) if (detail::drvActive(i) && detail::pin[i].st != lc::State::RUNNING) m |= 1u << i;
	return m;
}
inline void setPolicy(lc::Policy p) { detail::policy = p; }
inline lc::State state(uint8_t i) { return i < COUNT ? detail::pin[i].st : lc::State::EMPTY; }
// What a motor-test UI can do with pin i now: uiState() below (after the PWM API it reads).
inline const char* stateName(uint8_t i) { return lc::stateName(state(i)); }
inline lc::Identity identity(uint8_t i) { return i < COUNT ? detail::curIdent(i) : lc::Identity{}; }
inline bool openLoop(uint8_t i) { return i < COUNT && detail::pin[i].openLoop; }   // ROV: telemetry gone
// The next lifecycle event (appeared, lost, read ...), oldest first. false when there is none.
inline bool pollEvent(uint8_t& idx, lc::Ev& e, lc::State& st) {
	using namespace detail;
	if (evTail == evHead) return false;
	idx = evIdx[evTail]; e = evEv[evTail]; st = evSt[evTail]; evTail = (evTail + 1) & 7;
	return true;
}
// An async read found an ESC on a pin not in presentMask(): it is added (true once, so the caller
// can persist the mask).
inline bool presentGrew() { bool g = detail::presentAdded; detail::presentAdded = false; return g; }
inline bool passthroughActive() { return detail::fwActive; }

// ---- drive / spin (core0) ----------------------------------------------------------------------
// spinArm() asks the pin's lifecycle for RUNNING. READY: the arming stream for SPIN_ARM_MS starts now
// (or at once, if the hold has already armed it in that protocol). Not read yet: the arm waits for
// the async read (spinArming() meanwhile) and starts when it lands. FAILED: refused (false). Then
// drive() (or spinThrottle()/spinThrust()) sets the target; spinStop()/spinDisarm() release it.
// Mode AUTO uses the identity read from the ESC: Bluejay/JESC/AM32 -> bidir DShot (idle HIGH, eRPM +
// EDT telemetry); otherwise normal DShot (idle LOW), which any BLHeli-S spins. Reversible (3D)
// rotation comes from the same identity, so a command can be a signed thrust.
// Call spinPoll() every core0 loop to keep frames flowing, run arming + deadman, and read telemetry.
// The held session (being written) that refused the last spinArm, else -1.
inline int8_t armBusySession() { return detail::armBusy; }
inline bool spinArmed(uint8_t idx) {
	using namespace detail;
	return idx < COUNT && pin[idx].st == lc::State::RUNNING && drvActive(idx) && drvArmed[idx];
}
namespace detail {
	static bool startArm(uint8_t i) {           // RUNNING: the drive stream in the asked protocol
		Drive m = armMode[i];
		if (m == Drive::AUTO) m = pin[i].id.bidir ? Drive::BIDIR : Drive::NORMAL;
		bool wantB = m == Drive::BIDIR;
		// The hold's stream in that protocol is kept, armed or still arming: a new driver cuts the line,
		// and a Bluejay that loses its signal starts over from its boot (it did, on every first slider
		// move of a motor test that came within the hold's arm window).
		if (drvActive(i) && (drvB[i] != nullptr) == wantB && drvKb[i] == kbaudOf(i) && !armMsReq[i]) {
			drvTarget[i] = 0; drvLast[i] = millis(); stall[i].reset(); return true;
		}
		if (!drvStart(i, wantB)) { pin[i].onStop(); return false; }   // no SM: READY again, no false ARMED
		drvArmMs[i] = armMsReq[i];   // 0 => SPIN_ARM_MS (see streamPin)
		return true;
	}
}
inline bool spinArm(uint8_t idx, Drive mode = Drive::AUTO, uint16_t armMs = 0) {
	using namespace detail;
	if (idx >= COUNT) return false;
	// Held in a sync session (enter/read): it is learned there (one LEARN, no second bootloader visit)
	// and run. A session on another pin is left alone.
	armBusy = -1; drvInitFail[idx] = false;
	if (fwActive) return false;                  // the configurator owns every pin
	if (session == (int8_t)idx) closeSession();
	// An arm that must wait for a read would have to run the other held ESC, but one being written
	// (a writeflash/erase image half done) must not be run under the host: refused (armBusySession()).
	lc::State st = pin[idx].st;
	if (session >= 0 && sessionWrote && st != lc::State::READY && st != lc::State::RUNNING && st != lc::State::FAILED) {
		armBusy = session; return false;
	}
	armMode[idx] = mode; armMsReq[idx] = armMs; drvInitFail[idx] = false;
	switch (pin[idx].requestArm()) {
		case lc::Arm::REFUSED:    return false;
		case lc::Arm::AFTER_READ:
			// The read runs only with no session held (core1 has one bootloader line): run the other
			// ESC's app now, as 4baed53's arm did, so this arm does not wait for a `run`. (Not written:
			// checked above.)
			if (session >= 0) closeSession();
			return true;
		default:                  return startArm(idx);
	}
}
// A semantic drive value: signed thrust -1000..1000 on a 3D ESC, throttle 0..2000 on a one-way one.
// Refused (target untouched, deadman not fed) unless RUNNING and armed, and a reverse request to a
// one-way ESC is refused rather than clamped.
inline lc::Refuse drive(uint8_t idx, int v) {
	using namespace detail;
	if (idx >= COUNT) return lc::Refuse::STATE;
	lc::Refuse r = pin[idx].gate(drvActive(idx) && drvArmed[idx]);
	if (r != lc::Refuse::OK) return r;
	bool use3d; int16_t s; uint16_t thr;
	r = lc::mapDrive(pin[idx].id, v, use3d, s, thr);
	if (r != lc::Refuse::OK) return r;
	drvTarget[idx] = stall[idx].gate(millis(), thr); drvLast[idx] = millis();   // held at 0 after a stall
	return lc::Refuse::OK;
}
inline void spinThrottle(uint8_t idx, uint16_t throttle) {          // unidirectional: 0..SPIN_MAX
	using namespace detail;
	if (!spinArmed(idx)) return;                                     // must be armed first
	drvTarget[idx] = stall[idx].gate(millis(), throttle > SPIN_MAX ? SPIN_MAX : throttle);
	drvLast[idx] = millis();
}
// Reversible (3D): -1000..+1000, 0 = stop, full scale at +-953 (lc::thrust3d clamps past that).
inline void spinThrust(uint8_t idx, int16_t s) {
	using namespace detail;
	if (!spinArmed(idx)) return;
	if (s < 0 && !pin[idx].id.rev) return;          // reverse on a one-way ESC: refused
	drvTarget[idx] = stall[idx].gate(millis(), lc::thrust3d(s)); drvLast[idx] = millis();
}
// Restarted after a stall within the last few seconds (a UI can say why it is arming again).
inline bool spinStalled(uint8_t idx) { return idx < COUNT && detail::stall[idx].recent(millis()); }
// Held at 0 after a stall until it is sent 0 (lc::Stall).
inline bool spinHeld(uint8_t idx) {
	using namespace detail;
	return idx < COUNT && pin[idx].st == lc::State::RUNNING && drvActive(idx) && stall[idx].hold;
}
// An answered bidir stream at zero throttle, not held, that has gone quiet (lc::Pin::silent): only at
// zero, because under throttle a silent ESC is the stall watch's (a failed start beeps without
// telemetry and is re-armed); calling it gone there made the page forget a healthy ESC.
inline bool spinSilent(uint8_t idx) {
	using namespace detail;
	uint32_t now = millis();
	// not polled lately (MSP answered right after a blocking op, before spinPoll): no evidence
	return idx < COUNT && drvB[idx] && drvArmed[idx] && drvTarget[idx] == 0 && !stall[idx].hold &&
	       now - drvPollMs[idx] <= POLL_GAP_MS && pin[idx].silent(now);
}
inline bool spinReversible(uint8_t idx) { return idx < COUNT && detail::curIdent(idx).known && detail::curIdent(idx).rev; }
inline const char* spinMode(uint8_t idx) {   // the protocol while driven, else the state (lowercase)
	using namespace detail;
	if (idx >= COUNT) return "none";
	if (drvInitFail[idx]) return "init-fail";
	if (pwmOn(idx)) return "pwm";
	if (pin[idx].st == lc::State::RUNNING && drvActive(idx)) return drvB[idx] ? "bidir" : "normal";
	return lc::stateName(pin[idx].st, true);
}
inline const char* streamProto(uint8_t idx) {   // what is on the wire now, whatever the state
	using namespace detail;
	if (idx >= COUNT) return "none";
	if (drvInitFail[idx]) return "init-fail";
	return pwmOn(idx) ? "pwm" : drvB[idx] ? "bidir" : drvN[idx] ? "normal" : "none";
}
inline bool spinInitOk(uint8_t idx) { return idx < COUNT && !detail::drvInitFail[idx]; }
inline bool spinArming(uint8_t idx) {
	using namespace detail;
	if (idx >= COUNT) return false;
	return pin[idx].armWanted || (pin[idx].st == lc::State::RUNNING && drvActive(idx) && !drvArmed[idx]) ||
	       (armStart[idx] && pin[idx].st == lc::State::RUNNING);
}
inline uint16_t spinTarget(uint8_t idx) { return idx < COUNT ? detail::drvTarget[idx] : 0; }   // DShot value being sent
inline bool inBootloader(uint8_t idx)   { return detail::session == (int8_t)idx; }
inline bool isPresent(uint8_t i);
// RUNNING -> READY. The hold keeps the (armed) zero stream, so the next arm from READY is instant.
inline void spinStop(uint8_t idx) {
	using namespace detail;
	if (idx >= COUNT) return;
	drvTarget[idx] = 0; drvInitFail[idx] = false; armStart[idx] = false; stall[idx].reset();
	pin[idx].onStop();
	if (pwmOn(idx)) pwmOff(idx);
	if ((!holdOn || !isPresent(idx)) && (int8_t)idx != jobIdx) drvFree(idx);
}
inline void spinDisarm(uint8_t idx) { spinStop(idx); }
inline void spinStopAll() { for (uint8_t i = 0; i < COUNT; i++) spinStop(i); }
inline bool spinning()    { for (uint8_t i = 0; i < COUNT; i++) if (detail::drvActive(i)) return true; return false; }
inline bool spinTele(uint8_t idx, Telem& out) {
	if (idx >= COUNT || !detail::drvB[idx]) return false;   // telemetry only on bidir DShot
	out = detail::drvTele[idx]; return true;
}

namespace detail {
	static const uint32_t SESSION_IDLE_MS = 5000; // a held session this long without an op may be closed
	// Nothing else owns the line: no held session, no passthrough, no LittleFS write, no op.
	static inline bool canRunJob() { return session < 0 && !fwActive && !parkReq && op == Op::NONE; }
	// The async scheduler: one job at a time, round robin over PENDING pins.
	static void schedule(uint32_t now) {
		if (jobIdx >= 0) {
			uint8_t i = (uint8_t)jobIdx;
			if (jobPhase == 2) { if (opDone) jobCollect(); return; }
			if (pin[i].st != lc::State::PENDING || !canRunJob()) { jobIdx = -1; jobPhase = 0; return; }   // stays PENDING
			// The prime: BLHeli-S needs valid frames before the line goes high. A pin whose bidir zero
			// has run PRIME_MS already is primed; no SM for it -> go on unprimed (AM32 needs none).
			if (drvB[i] && now - drvStartMs[i] < PRIME_MS) return;
			pin[i].onReadStart(now);                 // (latches whether its telemetry is fresh first)
			drvFree(i); pinMode(PINS[i], OUTPUT); digitalWrite(PINS[i], HIGH);   // core1 owns it from here
			jobAbort = false; opIdx = i; opPin = PINS[i]; opOk = false; opDone = false; op = Op::READID;
			jobPhase = 2;
			return;
		}
		bool waiting = false;
		for (uint8_t k = 0; k < COUNT; k++) waiting = waiting || pin[k].st == lc::State::PENDING;
		if (!waiting) return;
		// A held session (scan/read/enter leave the last ESC in its bootloader) would hold every read
		// back until a `run`: an ESC that is waiting to be read wins once the session is idle — no op
		// on it for SESSION_IDLE_MS, and never one that has been written (a host's erase/writeflash
		// sequence must not see its ESC run mid-image). Closing learns and runs it; the next CLI config
		// op on that ESC enters it again.
		if (session >= 0 && !sessionWrote && now - sessLastOpMs >= SESSION_IDLE_MS &&
		    !fwActive && !parkReq && op == Op::NONE) closeSession();
		if (!canRunJob()) return;
		for (uint8_t k = 0; k < COUNT; k++) {
			uint8_t i = (uint8_t)((jobRr + k) % COUNT);
			if (pin[i].st != lc::State::PENDING) continue;
			jobRr = (uint8_t)(i + 1); jobIdx = (int8_t)i; jobPhase = 1;
			if (pwmOn(i)) pwmOff(i);
			if (!drvB[i]) drvStart(i, true);
			return;
		}
	}
	// Put on pin i what its state allows (lc::Pin::output): free, bidir zero, zero in its protocol,
	// or the drive stream an arm asked for.
	static void applyOutput(uint8_t i, uint32_t now) {
		if (pwmOn(i) || (int8_t)i == jobIdx || (int8_t)i == session) return;
		lc::Out o = pin[i].output(holdOn && isPresent(i));
		if (o == lc::Out::NONE) { if (drvActive(i)) drvFree(i); return; }
		if (o == lc::Out::DRIVE) { if (armStart[i]) { armStart[i] = false; startArm(i); } return; }
		bool wantB = o == lc::Out::ZERO_BIDIR || pin[i].id.bidir;
		if (drvActive(i) && (drvB[i] != nullptr) == wantB && drvKb[i] == kbaudOf(i)) return;
		if (drvInitFail[i] && (int32_t)(now - drvRetryMs[i]) < 0) return;   // no SM: retry in a second
		drvStart(i, wantB);
	}
}

inline void spinPoll() {   // call every core0 loop: lifecycle + hold + async reads + frames + telemetry
	using namespace detail;
	uint32_t now = millis();
	for (uint8_t i = 0; i < COUNT; i++) {
		gapCheck(i, now, true);
		stall[i].poll(now);
		lc::Ev e = pin[i].tick(now, policy);
		if (e == lc::Ev::LOST_STOPPED) { drvTarget[i] = 0; armStart[i] = false; }   // TOOL: stop that ESC only
		evPush(i, e);
	}
	if (!fwActive) {
		schedule(now);
		for (uint8_t i = 0; i < COUNT; i++) applyOutput(i, now);
	}
	bool any = false;
	for (uint8_t i = 0; i < COUNT; i++) { if (drvActive(i)) { any = true; streamPin(i); } }
	// a PWM pin left unfed after its arm: back to READY, and the hold takes the pin on the next loop
	// (no longer driving()). Here, not in pwmPoll: pump() runs inside sync ops and frees no driver.
	for (uint8_t i = 0; i < COUNT; i++) {
		if (!pwmOn(i)) continue;
		if (now - pwmStart[i] > PWM_ARM_MS + PWM_RELEASE_MS && millis() - pwmLast[i] > PWM_RELEASE_MS) {
			pwmOff(i); pin[i].onStop();
		}
	}
	pwmPoll();
	if (any) delayMicroseconds(200);
}

// ---- servo PWM (core0) -------------------------------------------------------------------------
// pwmBegin() frees the pin from DShot / the bootloader and starts 50 Hz pulses at the stop width for
// PWM_ARM_MS: long enough for an AM32 to notice DShot is gone (2 s), reboot, detect PWM and arm on
// it. The stop width is 1000 us, or 1500 us on a 3D ESC, where 1000 us is full reverse: so the ESC
// must be READY (read by scan, a configurator or the hold) and pwmBegin refuses one that is not.
// pwmSet() then gives the pulse width, fed like spinThrottle (the same 500 ms deadman falls back
// to the stop width, also from a timer IRQ while core0 is held up; it does not cut the signal). A
// width sent during the arm is not kept, and after it nothing but the stop width passes until one
// stop width has come (lc::pwmGate): a value never waits behind the arm and lands as a jump. During
// a sync bootloader visit the stop width goes out. Unfed for PWM_RELEASE_MS after the arm, the pin
// leaves PWM (READY, the hold takes it). spinStop()/spinArm() end PWM on the pin.
inline bool pwmActive(uint8_t idx) { return idx < COUNT && detail::pwmOn(idx); }
// Arming, or waiting for its first stop width after the arm.
inline bool pwmArming(uint8_t idx) {
	return pwmActive(idx) && (millis() - detail::pwmStart[idx] < PWM_ARM_MS || !detail::pwmZero[idx]);
}
// Fed a width above its stop within the deadman: a blocking bootloader visit must wait.
inline bool pwmDriving(uint8_t idx) {
	using namespace detail;
	return pwmActive(idx) && pwmUs[idx] != pwmStop[idx] && millis() - pwmLast[idx] <= SPIN_DEADMAN_MS;
}
inline bool pwmBegin(uint8_t idx) {
	using namespace detail;
	if (idx >= COUNT) return false;
	if (pwmOn(idx)) return true;
	if (fwActive) return false;
	if (session == (int8_t)idx) closeSession();   // held in its bootloader: learn it, run its app
	if (pin[idx].st != lc::State::READY && pin[idx].st != lc::State::RUNNING) return false;
	// no alarm slot for the IRQ deadman: no PWM (a held-up loop would keep the last width)
	if (!pwmTimerOn) pwmTimerOn = add_repeating_timer_ms(50, pwmWatch, nullptr, &pwmTimer);
	if (!pwmTimerOn) return false;
	drvTarget[idx] = 0; drvArmed[idx] = false; drvFree(idx);
	pin[idx].onRunStart();
	uint gp = PINS[idx], slice = pwm_gpio_to_slice_num(gp);
	gpio_set_function(gp, GPIO_FUNC_PWM);
	pwm_set_clkdiv(slice, clock_get_hz(clk_sys) / 1e6f);       // 1 tick = 1 us
	pwm_set_wrap(slice, 20000 - 1);                             // 50 Hz; a slice's two pins share it
	pwmStop[idx] = pin[idx].id.rev ? PWM_MID_US : PWM_STOP_US;
	pwmLevel(idx, pwmStop[idx]);
	pwm_set_enabled(slice, true);
	pwmStart[idx] = pwmLast[idx] = millis(); pwmZero[idx] = false;
	pwmUs[idx] = pwmStop[idx];                   // last: pwmOn() is what the IRQ checks
	return true;
}
inline void pwmSet(uint8_t idx, uint16_t us) {
	using namespace detail;
	if (!pwmActive(idx)) return;
	us = us < PWM_MIN_US ? PWM_MIN_US : (us > PWM_MAX_US ? PWM_MAX_US : us);
	pwmUs[idx] = lc::pwmGate(millis() - pwmStart[idx], PWM_ARM_MS, pwmZero[idx], us, pwmStop[idx]);
	pwmLast[idx] = millis();
}
inline uint16_t pwmTarget(uint8_t idx) { return pwmActive(idx) ? detail::pwmUs[idx] : 0; }

// What the web tool's motor test can do with ESC i now (lc::Ui, from its lifecycle state). Sent in
// MOTOR_TELEMETRY's mAh field with a 0x5A marker (fourway.h); mirrored once in web/js/app.js (UI).
using UiState = lc::Ui;
inline UiState uiState(uint8_t i) {
	using namespace detail;
	if (i >= COUNT) return UiState::ABSENT;
	lc::UiIn in;
	in.pwm = pwmActive(i); in.pwmArming = pwmArming(i);
	in.stream = drvActive(i) && !armStart[i]; in.armed = in.stream && drvArmed[i];
	in.stalled = spinHeld(i) || (spinStalled(i) && pin[i].st == lc::State::RUNNING && !drvArmed[i]);
	in.silent = spinSilent(i);
	return lc::uiCode(pin[i], in);
}

// ---- 4-way passthrough (fourway.h) -------------------------------------------------------------
// fwBegin: stop driving, and hold every signal line high so each ESC drops into its bootloader
// (what Betaflight's esc4wayInit does). fwSelect: make idx the session — the cached reply if it is
// still in its bootloader, else a BootInit, else the full DShot-prime entry — and return its raw
// 8-byte reply. Switching does NOT run the previous ESC; fwEnd runs every ESC that was entered.
// fwBegin primes every wired ESC with zero-throttle DShot for PRIME_MS, all at once, then holds
// the lines high. Both firmwares take that signal loss as "go to the bootloader": BLHeli-S needs
// the valid frames first (a bare high line leaves it waiting for signal), AM32 needs ~2 s without
// them plus its reboot. So by fwBeginMs + AM32_HOLD_MS every ESC is in, in parallel, and the
// configurator's first InitFlash of each costs at most that wait. One that still won't answer gets
// the full entry ladder once, then stays failed for the session.
// Lifecycle: every wired pin is READING for the whole passthrough (no hold, no async read). fwRun
// learns the ESC before it runs it (READY, or FAILED); fwEnd sends a pin that was never selected
// back to EMPTY/FAILED if it started there, else to PENDING (it may have been swapped meanwhile).
inline void fwEnd();
inline void fwBegin() {
	using namespace detail;
	if (fwActive) fwEnd();
	jobStop(true);                               // a background read gives its pin back first
	spinStopAll(); release();
	fwActive = true;
	BidirDShotX1* pr[COUNT] = { nullptr };
	for (uint8_t i = 0; i < COUNT; i++) {
		drvFree(i);                              // the hold too: the configurator owns every pin
		fwHave[i] = fwMiss[i] = fwDone[i] = false;
		if (present >> i & 1) { pin[i].onPassthroughBegin(); pr[i] = newBidir(PINS[i], DSHOT_KBAUD); }
	}
	uint32_t t0 = millis();
	while (millis() - t0 < PRIME_MS) {
		for (uint8_t i = 0; i < COUNT; i++) if (pr[i] && !pr[i]->initError()) pr[i]->sendThrottle(0);
		pump(); delayMicroseconds(300);
	}
	for (uint8_t i = 0; i < COUNT; i++) {
		delete pr[i];
		pinMode(PINS[i], OUTPUT); digitalWrite(PINS[i], HIGH);
	}
	fwBeginMs = millis();
}
inline bool fwSelect(uint8_t idx, uint8_t raw[8], bool& am32) {
	using namespace detail;
	if (idx >= COUNT) return false;
	if (drvActive(idx)) drvFree(idx);
	if (fwMiss[idx]) return false;
	if (fwDone[idx]) { fwDone[idx] = false; pin[idx].onReadStart(millis()); }   // DeviceReset ran it; selected again
	session = -1; fwIdx = idx;
	bool ok;
	if (fwHave[idx]) ok = runOp(Op::FWSELECT, PINS[idx]);      // re-select: compare or adopt the cached reply
	else {
		// Stay quiet until the hold has done its work: a BootInit while the ESC is still on its way
		// into the bootloader is garbage to it, and sends it back to the app.
		// AM32 reboots into its bootloader only ~2 s after the frames stop, plus its startup tune.
		// Nothing persisted says which firmware is on the pin, so every ESC gets the AM32 wait.
		uint32_t settle = AM32_HOLD_MS + 500;
		while (millis() - fwBeginMs < settle) { pump(); delayMicroseconds(200); }
		ok = runOp(Op::CONNECT, PINS[idx]);
	}
	if (!ok) ok = enterBootloader(PINS[idx]);
	fwHave[idx] = ok; fwMiss[idx] = !ok;
	if (!ok) return false;
	session = (int8_t)idx; sessLearned = sessionWrote = false; sessLastOpMs = millis();
	memcpy(fwRaw[idx], lastRaw, 8); fwAm32[idx] = lastAm32;
	memcpy(raw, lastRaw, 8); am32 = lastAm32;
	return true;
}
inline void fwRun(uint8_t idx) {   // leave idx's bootloader (start its app); forget its session
	using namespace detail;
	if (idx >= COUNT || !fwHave[idx]) return;
	fwIdx = idx;
	// Learn it again while it is still in its bootloader: whatever the configurator wrote or flashed
	// is in by now, and the motor test that usually follows then arms without a second bootloader
	// visit (that one stalled the board for ~5 s, and the web test gave up on it).
	bool learned = runOp(Op::FWSELECT, PINS[idx]) && runOp(Op::LEARN, PINS[idx]);
	runOp(Op::FWRUN, PINS[idx]);
	fwHave[idx] = false; if (session == (int8_t)idx) session = -1;
	pin[idx].onPassthroughEnd(true, learned, identCache[idx]); fwDone[idx] = true;
	if (pin[idx].st == lc::State::RUNNING) armStart[idx] = true;   // spinPoll starts it after fwEnd
}
inline void fwEnd() {
	using namespace detail;
	for (uint8_t i = 0; i < COUNT; i++) fwRun(i);
	for (uint8_t i = 0; i < COUNT; i++)
		if ((present >> i & 1) && !fwDone[i]) pin[i].onPassthroughEnd(false, false, lc::Identity{});
	fwActive = false;
}
inline uint8_t count() { return COUNT; }

// ---- which ESCs are wired ----------------------------------------------------------------------
inline uint32_t presentMask() { return detail::present & ((1u << COUNT) - 1); }
inline void     setPresent(uint32_t m) { detail::present = m; }
inline bool     isPresent(uint8_t i) { return i < COUNT && (presentMask() >> i & 1); }
inline bool     identFailed(uint8_t i) { return i < COUNT && detail::pin[i].st == lc::State::FAILED; }
// Configured reversible (3D), as read from the ESC on the pin now (false while unknown).
inline bool     knownReversible(uint8_t i) { return spinReversible(i); }
// The AM32 settings block of ESC i, from its identity (0 = not AM32 / not read).
inline uint16_t am32Eeprom(uint8_t i) { return i < COUNT && detail::curIdent(i).am32 ? detail::curIdent(i).am32Eep : 0; }
inline uint8_t  presentCount() { uint8_t n = 0; for (uint8_t i = 0; i < COUNT; i++) n += isPresent(i); return n; }
// The k-th wired ESC's index (a configurator numbers them 0..presentCount()-1, gaps closed); -1 if none.
inline int8_t   presentIndex(uint8_t k) {
	for (uint8_t i = 0; i < COUNT; i++) if (isPresent(i) && k-- == 0) return (int8_t)i;
	return -1;
}

// LittleFS mutations go inside one of these: core1 parks in its idle branch (never mid-op or
// mid-keepalive) until it goes out of scope. ok=false (an ESC read is in flight, or core1 did not
// park in time): do not write; report busy, or retry later.
struct FsGuard {
	bool ok;
	FsGuard() : ok(detail::park()) {}
	~FsGuard() { if (ok) detail::parkReq = false; }
	FsGuard(const FsGuard&) = delete;
	FsGuard& operator=(const FsGuard&) = delete;
};

// ---- core1 worker: call once from loop1() ------------------------------------------------------
namespace detail {
// core1: BootInit on the current pin for up to 500 ms; on a reply, adopt it as the session.
static bool connectHere(bool abortable) {
	uint32_t t0 = millis();
	while (millis() - t0 < 500) {
		if (abortable && jobAbort) return false;
		uint8_t raw[8] = {0};
		int n = bl.connectRawProbe(raw, 15);
		if (n >= 8 && raw[0] == '4' && raw[1] == '7' && raw[2] == '1') {
			// AM32 answers only the FIRST BootInit of a session, so a second hello from connect()
			// gets silence. Adopt the probe's reply as the session instead of re-sending.
			if (bl.adoptReply(raw)) return true;
		}
	}
	return false;
}
// core1: the adopted reply becomes the session's (sig/bootVer/lastRaw), keepalive count restarts.
static void takeHello() {
	const auto& d = bl.lastDevice();
	sig[0] = d.signature[0]; sig[1] = d.signature[1];
	bootVer = d.bootVersion; bootPages = d.bootPages;
	memcpy(lastRaw, d.bootInfo, 8); lastAm32 = d.am32;
	kaMiss = 0; sessionLost = false;
}
// core1: the connected ESC's identity and Info. AM32 (an ARM bootloader id) keeps bi_direction and
// FILE_NAME in its own settings block; the BLHeli-S map at 0x1A00 is application code there, so
// reading it would make up a direction.
static bool learnHere(Info& out, lc::Identity& id) {
	const auto& d = bl.lastDevice();
	out = Info{}; id = lc::Identity{};
	out.present = true; out.pin = bl.signalPin();
	out.sig = (uint16_t)((d.signature[0] << 8) | d.signature[1]);
	out.bootVer = d.bootVersion; out.bootPages = d.bootPages;
	if (armId(d.bootInfo)) {
		lc::Identity a = lc::identFromAm32(d.bootInfo[4], 0);
		uint8_t dir = 0, nm[15];
		if (!bl.readFlash(a.am32Eep + 18, &dir, 1)) return false;           // bi_direction (host/pico_esc/am32.py)
		id = lc::identFromAm32(d.bootInfo[4], dir);
		if (bl.readFlash(a.am32Eep - 32, nm, 15)) { memcpy(out.name, nm, 15); out.name[15] = 0; }   // FILE_NAME
		strncpy(out.layout, "AM32", 16);
		return true;
	}
	esc_setup::Settings s;
	if (!esc_setup::read(bl, s)) return false;
	id = lc::identFromSilabs(s.name, s.raw[0x0B]);
	strncpy(out.layout, s.layoutTag, 16); strncpy(out.name, s.name, 16);
	out.fwMain = s.mainRevision; out.fwSub = s.subRevision;
	return true;
}
// core1: hold the line high for ms, in slices, so an abort lands within ~20 ms. false = aborted.
static bool holdHigh(uint32_t ms) {
	uint32_t t0 = millis();
	while (millis() - t0 < ms) {
		if (jobAbort) return false;
		uint32_t left = ms - (millis() - t0);
		bl.holdIdleHigh(left < 20 ? left : 20);
	}
	return true;
}
}  // namespace detail

inline void core1Poll() {
	using namespace detail;
	core1Up = true;
	if (op == Op::NONE || opDone) {
		if (parkReq) {                               // core0 writes LittleFS: off the line till it is done
			parked = true;
			while (parkReq) {}
			parked = false; return;
		}
		static uint32_t lastKa = 0;                 // keep the held ESC in its bootloader
		if (session >= 0 && bl.connected() && millis() - lastKa > 100) {
			kaMiss = bl.keepAlive() ? 0 : (uint8_t)(kaMiss + 1);
			if (kaMiss >= 2) sessionLost = true;     // not the ESC we connected to any more (or none)
			lastKa = millis();
		}
		delay(1); return;
	}
	Op o = op; bool ok = false;
	if (o == Op::CONNECT) {
		bl.setSignalPin(opPin); bl.begin();
		ok = connectHere(false);
		if (ok) takeHello();
	} else if (o == Op::FWSELECT) {
		bl.setSignalPin(opPin); bl.begin();
		if (fwHave[fwIdx]) {
			// A SiLabs bootloader answers BootInit again: the fresh reply is adopted (and fwSelect stores
			// it as fwRaw), so an ESC swapped behind the configurator's back is reported as what it is
			// now, and fwRun learns that one. AM32 answers only the first BootInit, and a silent SiLabs
			// falls back the same way: re-adopt the cached reply if it still takes a keepalive.
			if (!fwAm32[fwIdx]) {
				uint8_t raw[8] = {0};
				int n = bl.connectRawProbe(raw, 15);
				ok = n >= 8 && raw[0] == '4' && raw[1] == '7' && raw[2] == '1' && bl.adoptReply(raw);
			}
			if (!ok) ok = bl.adoptReply(fwRaw[fwIdx], fwAm32[fwIdx]) && bl.keepAlive();
		} else ok = connectHere(false);
		if (ok) takeHello();
	} else if (o == Op::FWRUN) {
		bl.setSignalPin(opPin); bl.begin();
		ok = bl.adoptReply(fwRaw[fwIdx], fwAm32[fwIdx]) && bl.run();
	} else if (o == Op::LEARN) {
		Info in; lc::Identity id;
		ok = learnHere(in, id);
		if (ok) { infoCache[opIdx] = in; identCache[opIdx] = id; }
	} else if (o == Op::READID) {
		// The async read of a newly found ESC (core0 has primed it and set the line high): the same
		// ladder as enterBootloader (1 s, then 4 s twice for an AM32 reboot), learn, run its app.
		// Job-local results only; jobAbort is honoured between steps and inside every wait.
		static const uint32_t HOLD[3] = { HIGH_HOLD_MS, AM32_HOLD_MS, AM32_HOLD_MS };
		bl.setSignalPin(opPin); bl.begin();
		bool in = false;
		for (uint8_t a = 0; a < 3 && !in && !jobAbort; a++) {
			if (!holdHigh(HOLD[a])) break;
			bl.begin();
			in = connectHere(true);
		}
		ok = in && !jobAbort && learnHere(jobInfo, jobIdent);
		if (bl.connected()) bl.run();               // its app runs again, aborted or not
	}
	else if (o == Op::READCFG)     { esc_setup::Settings s; ok = esc_setup::read(bl, s); if (ok) memcpy(cfg, s.raw, esc_setup::kEepromLen); }
	else if (o == Op::READPAGE)    { ok = esc_setup::readPage(bl, page); }
	else if (o == Op::WRITEPAGE)   { ok = esc_setup::writePage(bl, page); }
	else if (o == Op::ERASE)       { ok = bl.erasePage(flAddr); }
	else if (o == Op::WRITEFLASH)  { ok = bl.writeFlash(flAddr, flBuf, flLen); }
	else if (o == Op::READFLASH)   { ok = bl.readFlash(flAddr, flBuf, flLen); }
	else if (o == Op::RUN)         { ok = bl.run(); }
	else if (o == Op::PROBE)       { bl.setSignalPin(opPin); bl.begin(); bl.holdIdleHigh(prHoldMs); bl.begin();
	                                 memset(prRaw, 0, sizeof(prRaw)); prN = bl.connectRawProbe(prRaw, 40);
	                                 bl.probeReplyActivity(200, prEdges, prLows, prTotal); ok = true; }
	opOk = ok;
	__sync_synchronize();                            // results (cfg, flBuf, job/ident caches) before opDone
	opDone = true;
}

}  // namespace escs
