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
#include "esc_pins.h"
#include <hardware/pwm.h>
#include <hardware/clocks.h>
#include <hardware/sync.h>
#include <pico/time.h>

namespace escs {

// The signal pin per ESC, in ESC-index order. ESC_SIGNAL_PINS (esc_config.h) is the default; a list
// stored in LittleFS (/cfg/pins, the `pins` command) replaces it at boot, through setPins() below.
// Every per-ESC array is sized MAX; COUNT is how many of them this boot uses.
static const uint8_t MAX = escpins::MAX;
static constexpr uint8_t DEFAULT_PINS[] = ESC_SIGNAL_PINS;
static const uint8_t DEFAULT_COUNT = sizeof(DEFAULT_PINS) / sizeof(DEFAULT_PINS[0]);
static uint8_t PINS[MAX] = ESC_SIGNAL_PINS;
static uint8_t COUNT     = DEFAULT_COUNT;
static bool    pinsFromCfg = false;                // PINS came from /cfg/pins (else the default)

// GPIOs this build uses for something else, never an ESC signal pin: the Pico W radio, the LCD bus
// (ESC_LCD), the UART console (ESC_CONSOLE_UART), the mode pin (ESC_MODE_PIN). The AS5600 encoder's
// GP16/17 are not here: an ESC on them switches the encoder off (EscTool::begin).
namespace detail {
	static constexpr uint8_t LCD_PINS[] = ESC_LCD_D;
	constexpr uint32_t lcdMask() {
		uint32_t m = (1u << ESC_LCD_RS) | (1u << ESC_LCD_RW) | (1u << ESC_LCD_E);
		for (uint8_t p : LCD_PINS) m |= 1u << p;
		return m;
	}
}
static constexpr uint32_t RESERVED = escpins::RADIO
	| (ESC_LCD ? detail::lcdMask() : 0u)
	| (ESC_CONSOLE_UART ? (1u << ESC_UART_TX) | (1u << ESC_UART_RX) : 0u)
	| (ESC_MODE_PIN >= 0 ? 1u << (ESC_MODE_PIN & 31) : 0u);
static_assert(escpins::check(DEFAULT_PINS, DEFAULT_COUNT, RESERVED) == escpins::Err::OK,
              "ESC_SIGNAL_PINS: 1..8 distinct GPIOs, none used by this build (radio, LCD, UART, mode pin)");
// What a reserved GPIO is used for (an `err` reason).
inline const char* reservedFor(uint8_t g) {
	if (escpins::RADIO >> g & 1) return "Pico W radio";
	if (ESC_LCD && (detail::lcdMask() >> g & 1)) return "LCD";
	if (ESC_CONSOLE_UART && (g == ESC_UART_TX || g == ESC_UART_RX)) return "UART console";
	if (ESC_MODE_PIN >= 0 && g == ESC_MODE_PIN) return "mode pin";
	return "this build";
}
// Is p[0..n) allowed as the pin list (escpins::check against RESERVED)? bad = the offending GPIO.
inline escpins::Err checkPins(const uint8_t* p, uint8_t n, uint8_t* bad = nullptr) {
	return escpins::check(p, n, RESERVED, bad);
}

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
// Stream zero throttle this long to ARM the ESC (covers its boot beep after leaving the bootloader).
// A bidir ESC counts as armed only once it answers and has taken the EDT enable (lc::ArmGate); EDT
// used to go out only in a zero-throttle gap after the arm, which a motor test that sends throttle
// at once never leaves: the Bluejay then never started after a passthrough.
static const uint32_t SPIN_ARM_MS           = lc::ARM_MS;
static const uint32_t SPIN_ARM_TELE_HOLD_MS = lc::ARM_ANSWER_HOLD_MS;   // zero throttle after its first answer
static const uint32_t SPIN_ARM_TELE_MAX_MS  = lc::ANSWER_MAX_MS;        // never answered: armed anyway
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
// if its identity says so (lc::silabsBidir: the layout revision; AM32 always), else normal — so any
// stock BLHeli-S ESC still spins.
enum class Drive : uint8_t { AUTO, NORMAL, BIDIR };

namespace detail {
	static const uint32_t DSHOT_KBAUD = ESC_DSHOT_KBAUD;
	// Bootloader entry: prime, line high, quiet, BootInit polled (lc::Entry: the rules and the bench
	// measurements behind them).
	static const uint32_t PRIME_MS = lc::ENTRY_PRIME_MS, ENTRY_QUIET_MS = lc::ENTRY_QUIET_MS,
	                      ENTRY_MAX_MS = lc::ENTRY_MAX_MS, ENTRY_BARE_MS = lc::ENTRY_BARE_MS,
	                      CONNECT_MIN_MS = lc::ENTRY_POLL_MIN_MS;
	static blheli_bl::Bootloader  bl({ .signalPin = PINS[0], .baud = 0 });    // core1 only

	enum class Op : uint8_t { NONE, CONNECT, READCFG, READPAGE, WRITEPAGE, ERASE, WRITEFLASH, READFLASH, RUN, PROBE,
	                          FWSELECT, FWRUN, LEARN, READID };
	// One mailbox for the synchronous ops (runOp) and the one asynchronous job (READID): runOp first
	// collects or aborts the job, so the two never overlap.
	static volatile Op       op = Op::NONE;
	static volatile bool     opDone = false, opOk = false;
	static volatile uint8_t  opPin = PINS[0], opIdx = 0;
	static volatile uint32_t opUntil = 0;   // Op::CONNECT / READID: poll BootInit until this millis() (0: CONNECT_MIN_MS from now)
	static volatile uint8_t  sig[2] = {0, 0};
	static volatile uint8_t  bootVer = 0, bootPages = 0;
	static uint32_t prHoldMs = 0, prEdges = 0, prLows = 0, prTotal = 0;   // Op::PROBE in/out
	static uint32_t prPollMs = 0, prAtMs = 0;   // Op::PROBE poll mode: BootInit until a reply or pollMs; when it came
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
	static volatile uint8_t fwIdx = 0;
	static bool     fwActive = false;            // the configurator owns every pin: no hold, no jobs
	static uint32_t fwBeginMs = 0;
	// A bootloader entry's prime stream: bidir when a bidir ESC is known there (its answer ends the
	// prime), else normal DShot, which every firmware takes (a stock BLHeli_S may not count inverted
	// frames as the signal it needs before the line goes high).
	struct Prime {
		BidirDShotX1* b = nullptr; DShotX4* n = nullptr;
		void end() { delete b; b = nullptr; delete n; n = nullptr; }
	};
	static inline bool armId(const uint8_t raw[8]) { return raw[5] == 0x06 && raw[4] > 0 && raw[4] < 0x90; }

	// Everything one ESC pin carries, ch[i] for pin index i.
	struct Chan {
		lc::Pin      pin;                    // its lifecycle: one state, and what the hold / LOST handling do with it
		Info         info;                   // Op::LEARN results for the session on this pin
		lc::Identity ident;
		// drive/spin: one DShot driver (core0). Only one of b/n is non-null.
		struct Drv {
			BidirDShotX1*     b = nullptr;       // bidir DShot: idle HIGH, eRPM/EDT telemetry
			DShotX4*          n = nullptr;       // normal DShot: idle LOW, throttle only
			volatile uint16_t target = 0;
			uint32_t          last = 0;          // when the target was last fed (deadman)
			lc::ArmGate       arm;               // the stream's arm window (streamPin)
			lc::EdtSched      edt;               // its EDT enable (bidir)
			lc::Stall         stall;             // the stall watch (streamPin) and its hold
			bool              initFail = false;  // the last driver start found no free PIO SM
			uint32_t          retryMs = 0;       // ... the hold tries again from here
			uint32_t          startMs = 0;       // when this stream started (the prime counts it)
			uint32_t          pollMs = 0;        // when its telemetry was last polled
			uint16_t          kb = 0;            // the bitrate it was started at
			uint8_t           gapRun = 0;        // poll gaps in a row (gapCheck)
			Telem             tele;
			// A fresh stream on the driver just made (b or n): zero, the arm window and EDT from now.
			void begin(uint32_t now, uint16_t kbps, bool bidir) {
				initFail = false; kb = kbps;
				target = 0; last = startMs = pollMs = now;
				arm.begin(now); edt.begin(now, bidir); stall.reset();
			}
		} drv;
		struct ArmReq {                          // what the pending/last arm asked for
			Drive    mode = Drive::AUTO;
			uint16_t ms = 0;
			bool     start = false;              // RUNNING after a read: start the arm stream
		} armReq;
		// Overrides (0 = the global ESC_DSHOT_KBAUD / ESC_MOTOR_POLES), set via escs::setKbaud/setPoles so
		// ESCs/motors on one board can differ (a 300 kbaud A_H_30 next to a 600 kbaud unit, other poles).
		uint16_t kbaud = 0;
		uint8_t  poles = 0;
		// Servo PWM (50 Hz, 1000..2000 us) instead of DShot, for firmwares that take it (AM32, stock
		// BLHeli-S; Bluejay does not). Hardware PWM slice, not PIO. us = 0: the pin is not on PWM.
		struct Pwm {
			volatile uint16_t us = 0, stop = 0;  // read by the deadman IRQ; stop: 1500 on a 3D ESC
			volatile uint32_t last = 0;
			uint32_t          start = 0;
			bool              zero = false;      // a stop width came after the arm (lc::pwmGate)
		} pwm;
		// 4-way passthrough (fourway.h): an ESC once connected stays in its bootloader while another is
		// selected, so coming back to it re-adopts its cached reply instead of a second BootInit (which
		// AM32 does not answer). One that never answered stays failed until the next fwBegin, so a
		// configurator polling every reported channel pays the timeout once per session.
		struct Fw {
			bool      have = false, am32 = false, miss = false;
			bool      done = false;              // fwRun already left its bootloader (DeviceReset)
			uint8_t   raw[8] = {0};
			lc::Entry entry;                     // its bootloader entry (fwBegin, fwSelect)
			Prime     prime;                     // ... its prime stream, kept across InitFlash calls
		} fw;
	};
	static Chan ch[MAX];
	static lc::Policy policy = lc::Policy::TOOL;
	static bool       holdOn = false;              // EscTool turns it on
	static uint8_t    evIdx[8]; static lc::Ev evEv[8]; static lc::State evSt[8];
	static uint8_t    evHead = 0, evTail = 0;
	static inline void evPush(uint8_t i, lc::Ev e) {   // with the state it left, not the state at print time
		if (e == lc::Ev::NONE) return;
		uint8_t n = (evHead + 1) & 7;
		if (n == evTail) evTail = (evTail + 1) & 7;   // full: the oldest goes
		evIdx[evHead] = i; evEv[evHead] = e; evSt[evHead] = ch[i].pin.st; evHead = n;
	}
	static inline uint16_t   kbaudOf(uint8_t i) { return ch[i].kbaud ? ch[i].kbaud : (uint16_t)DSHOT_KBAUD; }
	static inline uint8_t    polesOf(uint8_t i) { return ch[i].poles ? ch[i].poles : (uint8_t)MOTOR_POLES; }
	static uint8_t           blDepth = 0;                  // inside a sync bootloader visit: PWM sends its stop width
	static inline bool pwmOn(uint8_t i) { return ch[i].pwm.us != 0; }
	// IRQs off: a slice's two pins share one CC register, and the deadman alarm below writes it too.
	static inline void pwmLevel(uint8_t i, uint16_t us) {
		uint32_t s = save_and_disable_interrupts(); pwm_set_gpio_level(PINS[i], us); restore_interrupts(s);
	}
	// The slice keeps its last width whatever core0 does, so the deadman also runs from a timer IRQ:
	// a loop held up (a blocking LittleFS write, a hang) still falls back to the stop width after 500 ms.
	static repeating_timer_t pwmTimer; static bool pwmTimerOn = false;
	static bool pwmWatch(repeating_timer_t*) {
		for (uint8_t i = 0; i < COUNT; i++)
			if (pwmOn(i) && millis() - ch[i].pwm.last > SPIN_DEADMAN_MS) pwm_set_gpio_level(PINS[i], ch[i].pwm.stop);
		return true;
	}
	static inline void pwmSafe() { for (uint8_t i = 0; i < COUNT; i++) if (pwmOn(i)) pwmLevel(i, ch[i].pwm.stop); }
	static inline void pwmOff(uint8_t i) {
		if (!pwmOn(i)) return;
		pwmLevel(i, 0); ch[i].pwm.us = 0;
		gpio_set_function(PINS[i], GPIO_FUNC_SIO); pinMode(PINS[i], INPUT);
	}

	static inline uint8_t idxOf(uint8_t p) { for (uint8_t i = 0; i < COUNT; i++) if (PINS[i] == p) return i; return 0; }
	// The identity that applies to pin i now: the session's own LEARN while it is held, else the pin's.
	static inline const lc::Identity& curIdent(uint8_t i) {
		return (session == (int8_t)i && sessLearned) ? ch[i].ident : ch[i].pin.id;
	}
	static inline bool drvActive(uint8_t i) { return ch[i].drv.b || ch[i].drv.n; }
	static inline void drvSend(uint8_t i, uint16_t t) {
		if (ch[i].drv.b) ch[i].drv.b->sendThrottle(t);
		else if (ch[i].drv.n) { uint16_t a[4] = { t, 0, 0, 0 }; ch[i].drv.n->sendThrottles(a); }
	}
	static inline void drvFree(uint8_t i) {
		pwmOff(i);                                   // whoever takes the pin next owns it alone
		if (ch[i].drv.b) { ch[i].drv.b->sendThrottle(0); delete ch[i].drv.b; ch[i].drv.b = nullptr; }
		if (ch[i].drv.n) { uint16_t a[4] = { 0, 0, 0, 0 }; ch[i].drv.n->sendThrottles(a); delete ch[i].drv.n; ch[i].drv.n = nullptr; }
		ch[i].pin.onStreamEnd();
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
		if (bidir) { ch[i].drv.b = newBidir(PINS[i], kb); if (ch[i].drv.b->initError()) { delete ch[i].drv.b; ch[i].drv.b = nullptr; } }
		else       { ch[i].drv.n = newNormal(PINS[i], kb); if (ch[i].drv.n->initError()) { delete ch[i].drv.n; ch[i].drv.n = nullptr; } }
		if (!drvActive(i)) {
			if (!ch[i].drv.initFail) evPush(i, lc::Ev::INIT_FAIL);
			ch[i].drv.initFail = true; ch[i].drv.retryMs = now + 1000;
			return false;
		}
		ch[i].drv.begin(now, kb, bidir);
		ch[i].pin.onStreamStart(now, bidir);
		return true;
	}

	static const uint32_t POLL_GAP_MS = 50;      // a pin not polled this long: core0 was away
	static const uint8_t  SLOW_GAPS = 5;         // gaps in a row: say so (Ev::SLOW_LOOP), once per run
	// A loop held up (a blocking op that does not pump) is not an ESC gone quiet: the lifecycle's
	// clocks move on by the gap, and the stall clocks restart. A loop slow on every pass would keep
	// LOST, SILENT and the stall's silence rule from ever firing, so repeated gaps are reported. Only
	// spinPoll's own check counts them (count): the streamPin/pump checks right after it see no gap
	// and must not reset the run.
	static void gapCheck(uint8_t i, uint32_t now, bool count = false) {
		bool gap = drvActive(i) && now - ch[i].drv.pollMs > POLL_GAP_MS;
		if (gap) { ch[i].pin.rebase(now - ch[i].drv.pollMs); ch[i].drv.stall.onGap(); }
		if (count && drvActive(i)) {
			if (!gap) ch[i].drv.gapRun = 0;
			else if (ch[i].drv.gapRun < 255 && ++ch[i].drv.gapRun == SLOW_GAPS) evPush(i, lc::Ev::SLOW_LOOP);
		}
		ch[i].drv.pollMs = now;
	}
	// One loop's worth of pin i's stream: arm window, deadman, telemetry, stall watch, EDT, one frame.
	// Shared by spinPoll() and pump(), so a synchronous wait streams exactly what the loop would.
	static void streamPin(uint8_t i) {
		if (!drvActive(i)) return;
		uint32_t now = millis();
		gapCheck(i, now);
		if (ch[i].pin.st != lc::State::RUNNING) ch[i].drv.target = 0;   // only RUNNING ever sends non-zero
		if (!ch[i].drv.arm.armed) {
			ch[i].drv.target = 0;
			if (ch[i].drv.arm.tick(now, ch[i].drv.b != nullptr, ch[i].drv.edt.left > 0, ch[i].pin.id.tune) && ch[i].drv.b) ch[i].drv.edt.burst();   // AM32 may answer before it arms
		}
		else if (millis() - ch[i].drv.last > SPIN_DEADMAN_MS) ch[i].drv.target = 0;   // deadman (armed only)
		if (ch[i].drv.b) {
			uint32_t v = 0;
			BidirDshotTelemetryType ty = ch[i].drv.b->getTelemetryPacket(&v);
			switch (ty) {
				case BidirDshotTelemetryType::ERPM:        { uint8_t pp = polesOf(i); ch[i].drv.tele.rpm = (pp > 1) ? v / (pp / 2) : v; } ch[i].drv.tele.valid = true; ch[i].drv.tele.rpmStampMs = millis(); ch[i].drv.arm.onAnswer(millis()); break;
				case BidirDshotTelemetryType::VOLTAGE:     ch[i].drv.tele.voltage = (float)v / 4.0f; break;
				case BidirDshotTelemetryType::CURRENT:     ch[i].drv.tele.current = v; break;
				case BidirDshotTelemetryType::TEMPERATURE: ch[i].drv.tele.tempC = v; ch[i].drv.tele.tempStampMs = millis(); break;
				case BidirDshotTelemetryType::STRESS:      ch[i].drv.tele.stress = v & ESC_STATUS_MAX_STRESS_MASK; break;
				case BidirDshotTelemetryType::STATUS:      ch[i].drv.tele.status = v; break;
				default: break;
			}
			// Any valid packet is a sign of life (EDT keeps coming in forced sine, where eRPM does not).
			if (ty != BidirDshotTelemetryType::CHECKSUM_ERROR && ty != BidirDshotTelemetryType::NO_PACKET) {
				ch[i].drv.tele.pktStampMs = now;
				evPush(i, ch[i].pin.onPacket(now));
				// an unconfirmed identity's first answer sends the pin to PENDING: nothing more this frame
				if (ch[i].pin.st != lc::State::RUNNING) { ch[i].drv.target = 0; ch[i].armReq.start = false; }
			}
			// Stall watch (lc::Stall): armed, under throttle, on a stream that has answered, Bluejay/
			// BLHeli-derived (AM32 stays in its run). Its silence there is the watch's, not LOST
			// (lc::Pin::stallWatch). ZERO / QUIET arm the same stream again (zero, answer, EDT: Bluejay
			// cleared it) with the target held at 0; SILENT is LOST (TOOL) or the open loop (ROV).
			bool watched = ch[i].pin.stallWatched(ch[i].drv.arm.armed, ch[i].drv.target);
			lc::Ev se;
			lc::StallAct act = ch[i].pin.onStall(ch[i].drv.stall.step(now, watched, ty == BidirDshotTelemetryType::ERPM, v, ch[i].pin.pktMs),
			                                  policy, now, se);
			evPush(i, se);
			if (act == lc::StallAct::REARM) {
				ch[i].drv.stall.restart(now);
				ch[i].drv.arm.begin(now); ch[i].drv.edt.burst(); ch[i].drv.target = 0; watched = false;
			} else if (act == lc::StallAct::LOST) {   // TOOL: stop that ESC only, as LOST_STOPPED in spinPoll
				ch[i].drv.target = 0; ch[i].armReq.start = false; ch[i].drv.stall.reset(); watched = false;
			}
			ch[i].pin.stallWatch = watched;
			// EDT enable in place of this frame (lc::EdtSched: at the start, once armed, after every run)
			if (ch[i].drv.edt.frame(now, ch[i].drv.arm.armed, ch[i].drv.arm.answer != 0, ch[i].drv.target, ch[i].drv.tele.rpm, ch[i].drv.tele.rpmStampMs)) {
				ch[i].drv.b->sendRaw11Bit(13); return;
			}
		}
		drvSend(i, ch[i].drv.target);
	}
	static void pwmPoll() {
		for (uint8_t i = 0; i < COUNT; i++) {
			if (!pwmOn(i)) continue;
			uint32_t now = millis();
			bool arming = now - ch[i].pwm.start < PWM_ARM_MS, dead = now - ch[i].pwm.last > SPIN_DEADMAN_MS;
			// nothing is fed during a sync bootloader visit (core0 waits on core1): the stop width
			pwmLevel(i, arming || dead || blDepth ? ch[i].pwm.stop : ch[i].pwm.us);
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
		if (aborted && !ok) { ch[i].pin.onReadAborted(); return; }
		lc::Ev e = ch[i].pin.onReadDone(ok, jobIdent);
		if (ok) { ch[i].info = jobInfo; ch[i].ident = jobIdent; }
		evPush(i, e);
		if (ch[i].pin.st == lc::State::RUNNING) ch[i].armReq.start = true;
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
		opIdx = idxOf(gpio); opPin = gpio; opOk = false; opDone = false;
		__sync_synchronize();                        // the op's inputs (flBuf, page, ...) before op
		op = o;
		while (!opDone) { pump(); delayMicroseconds(200); }
		__sync_synchronize();                        // opDone seen: core1's results are complete
		if (session >= 0 && opIdx == (uint8_t)session) sessLastOpMs = millis();
		blDepth--;
		op = Op::NONE; return opOk;
	}
	// CONNECT polling until `until` (millis(); 0 = CONNECT_MIN_MS from now).
	static bool connectUntil(uint8_t gpio, uint32_t until) {
		opUntil = until; bool ok = runOp(Op::CONNECT, gpio); opUntil = 0; return ok;
	}
	// Run entry `en` on pin i (lc::Entry) until the ESC is in its bootloader (IN), the entry misses
	// (MISS), or `stop` (millis) comes first (LATER: call again, it carries on). pd is its prime
	// stream, kept from one call to the next; the pin's own stream must be freed already.
	enum class EntryRes : uint8_t { IN, MISS, LATER };
	static EntryRes runEntry(uint8_t i, lc::Entry& en, Prime& pd, uint32_t stop) {
		uint8_t gpio = PINS[i];
		EntryRes r = EntryRes::LATER;
		blDepth++; pwmSafe();
		for (;;) {
			uint32_t now = millis();
			lc::Entry::Do d = en.step(now);
			if (d == lc::Entry::Do::PRIME) {
				if ((int32_t)(now - stop) >= 0) break;
				// no SM: it goes on unprimed (AM32 needs none)
				if (en.waitAnswer && !pd.b) pd.b = newBidir(gpio, DSHOT_KBAUD);
				if (!en.waitAnswer && !pd.n) pd.n = newNormal(gpio, DSHOT_KBAUD);
				if (pd.b && !pd.b->initError()) {
					pd.b->sendThrottle(0);
					uint32_t v; BidirDshotTelemetryType ty = pd.b->getTelemetryPacket(&v);
					if (ty != BidirDshotTelemetryType::CHECKSUM_ERROR && ty != BidirDshotTelemetryType::NO_PACKET) en.onAnswer();
				}
				if (pd.n && !pd.n->initError()) { uint16_t z[4] = { 0, 0, 0, 0 }; pd.n->sendThrottles(z); }
				pump(); delayMicroseconds(300);
				continue;
			}
			pd.end();
			pinMode(gpio, OUTPUT); digitalWrite(gpio, HIGH);
			if (d == lc::Entry::Do::QUIET) {           // no BootInit here: it keeps Bluejay in its app
				uint32_t until = (int32_t)(en.quietEnd() - stop) < 0 ? en.quietEnd() : stop;
				while ((int32_t)(millis() - until) < 0) { pump(); delayMicroseconds(200); }
				if ((int32_t)(millis() - en.quietEnd()) < 0) break;
				continue;
			}
			if (d == lc::Entry::Do::MISS) { r = EntryRes::MISS; break; }
			uint32_t end = en.pollEnd();
			bool last = (int32_t)(end - stop) <= 0;
			if (connectUntil(gpio, last ? end : stop)) r = EntryRes::IN;
			else if (last) r = EntryRes::MISS;
			break;
		}
		blDepth--;
		return r;
	}
	// One full entry, waited for (`read i`, `scan`, `enter`): a bidir ESC known on the pin is primed
	// until it answers (lc::Entry).
	static bool enterBootloader(uint8_t gpio) {
		uint8_t i = idxOf(gpio);
		lc::Entry en; Prime pd;
		en.begin(millis(), ch[i].pin.id.known ? ch[i].pin.id.bidir : ch[i].pin.seen, ENTRY_MAX_MS, ch[i].pin.id.tune);
		return runEntry(i, en, pd, millis() + 60000) == EntryRes::IN;
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
		ch[i].pin.onSessionClose(ok, ch[i].ident);
		if (ch[i].pin.st == lc::State::RUNNING) ch[i].armReq.start = true;   // an arm waited for this read
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
		ch[i].armReq.start = false;
		if (!fwActive) ch[i].pin.onSessionOpen(millis());
		if (enterBootloader(PINS[i])) { session = (int8_t)i; sessLearned = sessionWrote = false; sessLastOpMs = millis(); return true; }
		if (!fwActive) ch[i].pin.onReadDone(false, lc::Identity{});
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
	if (learnSession(idx)) { out = ch[idx].info; return true; }
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
inline int probe(uint8_t idx, uint32_t holdMs, uint8_t raw[8], uint32_t& edges, uint32_t& lows, uint32_t& total,
                 uint32_t pollMs = 0, uint32_t* atMs = nullptr) {
	using namespace detail;
	if (idx >= COUNT) return -1;
	jobStop(true); closeSession();
	if (drvActive(idx) || pwmOn(idx)) drvFree(idx);
	ch[idx].pin.onSessionOpen(millis());
	prHoldMs = holdMs; prPollMs = pollMs;
	runOp(Op::PROBE, PINS[idx]);
	prPollMs = 0; if (atMs) *atMs = prAtMs;
	ch[idx].pin.onSessionClose(false, lc::Identity{});   // whatever it left behind is read again
	memcpy(raw, prRaw, 8); edges = prEdges; lows = prLows; total = prTotal; return prN;
}

inline bool connected(uint8_t idx) { return detail::session == (int8_t)idx; }
inline bool sessionOpen() { return detail::session >= 0; }   // some ESC is held in its bootloader
inline void release() { detail::closeSession(); }


// Per-ESC drive config (call BEFORE spinArm; takes effect at the next arm). 0 restores the global
// ESC_DSHOT_KBAUD / ESC_MOTOR_POLES default. Lets a Thruster carry its own bitrate / pole count.
// Any index below MAX: main binds its Thrusters before EscTool::begin applies the pin list.
inline void setKbaud(uint8_t idx, uint16_t kbaud) { if (idx < MAX) detail::ch[idx].kbaud = kbaud; }
inline void setPoles(uint8_t idx, uint8_t poles)  { if (idx < MAX) detail::ch[idx].poles = poles; }

// ---- lifecycle (core0) -------------------------------------------------------------------------
// The idle hold: zero throttle on every configured pin that is not driven (bidir until it is read,
// then its own protocol). It is also the hot-plug detector, so with it off a plug-in is not seen
// either. Each held pin takes a PIO SM: past the budget (newBidir/newNormal) a pin gets init-fail
// and is tried again every second, so with more than 4 pins list the wired ones first.
inline void setHold(bool on) {
	using namespace detail;
	holdOn = on;
	if (!on) for (uint8_t i = 0; i < COUNT; i++)   // at once, not on the next poll: `idle off` reads held=0
		if (drvActive(i) && ch[i].pin.st != lc::State::RUNNING && (int8_t)i != jobIdx) drvFree(i);
}
inline bool holdActive() { return detail::holdOn; }
inline uint32_t heldMask() {   // pins carrying a zero stream the hold owns
	uint32_t m = 0;
	for (uint8_t i = 0; i < COUNT; i++) if (detail::drvActive(i) && detail::ch[i].pin.st != lc::State::RUNNING) m |= 1u << i;
	return m;
}
inline void setPolicy(lc::Policy p) { detail::policy = p; }
inline lc::State state(uint8_t i) { return i < COUNT ? detail::ch[i].pin.st : lc::State::EMPTY; }
// What a motor-test UI can do with pin i now: uiState() below (after the PWM API it reads).
inline const char* stateName(uint8_t i) { return lc::stateName(state(i)); }
inline lc::Identity identity(uint8_t i) { return i < COUNT ? detail::curIdent(i) : lc::Identity{}; }
// ms from its stream's start to its first answer, the last time it answered a fresh stream (0: never).
inline uint32_t answerDelay(uint8_t i) { return i < COUNT ? detail::ch[i].pin.answerDelay : 0; }
inline bool openLoop(uint8_t i) { return i < COUNT && detail::ch[i].pin.openLoop; }   // ROV: telemetry gone
// The next lifecycle event (appeared, lost, read ...), oldest first. false when there is none.
inline bool pollEvent(uint8_t& idx, lc::Ev& e, lc::State& st) {
	using namespace detail;
	if (evTail == evHead) return false;
	idx = evIdx[evTail]; e = evEv[evTail]; st = evSt[evTail]; evTail = (evTail + 1) & 7;
	return true;
}
inline bool passthroughActive() { return detail::fwActive; }

// ---- drive / spin (core0) ----------------------------------------------------------------------
// spinArm() asks the pin's lifecycle for RUNNING. READY: the arming stream for SPIN_ARM_MS starts now
// (or at once, if the hold has already armed it in that protocol). Not read yet: the arm waits for
// the async read (spinArming() meanwhile) and starts when it lands. FAILED: refused (false). Then
// drive() sets the target (the one drive path: by identity); spinStop()/spinDisarm() release it.
// Mode AUTO uses the identity read from the ESC: Bluejay/JESC/AM32 -> bidir DShot (idle HIGH, eRPM +
// EDT telemetry); otherwise normal DShot (idle LOW), which any BLHeli-S spins. Reversible (3D)
// rotation comes from the same identity, so a command can be a signed thrust.
// Call spinPoll() every core0 loop to keep frames flowing, run arming + deadman, and read telemetry.
// The held session (being written) that refused the last spinArm, else -1.
inline int8_t armBusySession() { return detail::armBusy; }
inline bool spinArmed(uint8_t idx) {
	using namespace detail;
	return idx < COUNT && ch[idx].pin.st == lc::State::RUNNING && drvActive(idx) && ch[idx].drv.arm.armed;
}
namespace detail {
	static bool startArm(uint8_t i) {           // RUNNING: the drive stream in the asked protocol
		Drive m = ch[i].armReq.mode;
		if (m == Drive::AUTO) m = ch[i].pin.id.bidir ? Drive::BIDIR : Drive::NORMAL;
		bool wantB = m == Drive::BIDIR;
		// The hold's stream in that protocol is kept, armed or still arming: a new driver cuts the line,
		// and a Bluejay that loses its signal starts over from its boot (it did, on every first slider
		// move of a motor test that came within the hold's arm window).
		if (drvActive(i) && (ch[i].drv.b != nullptr) == wantB && ch[i].drv.kb == kbaudOf(i) && !ch[i].armReq.ms) {
			ch[i].drv.target = 0; ch[i].drv.last = millis(); ch[i].drv.stall.reset(); return true;
		}
		if (!drvStart(i, wantB)) { ch[i].pin.onStop(); return false; }   // no SM: READY again, no false ARMED
		ch[i].drv.arm.ms = ch[i].armReq.ms;   // 0 => SPIN_ARM_MS; a short one re-arms an ESC that is armed already
		return true;
	}
}
inline bool spinArm(uint8_t idx, Drive mode = Drive::AUTO, uint16_t armMs = 0) {
	using namespace detail;
	if (idx >= COUNT) return false;
	// Held in a sync session (enter/read): it is learned there (one LEARN, no second bootloader visit)
	// and run. A session on another pin is left alone.
	armBusy = -1; ch[idx].drv.initFail = false;
	if (fwActive) return false;                  // the configurator owns every pin
	if (session == (int8_t)idx) closeSession();
	// An arm that must wait for a read would have to run the other held ESC, but one being written
	// (a writeflash/erase image half done) must not be run under the host: refused (armBusySession()).
	lc::State st = ch[idx].pin.st;
	if (session >= 0 && sessionWrote && st != lc::State::READY && st != lc::State::RUNNING && st != lc::State::FAILED) {
		armBusy = session; return false;
	}
	ch[idx].armReq.mode = mode; ch[idx].armReq.ms = armMs; ch[idx].drv.initFail = false;
	switch (ch[idx].pin.requestArm()) {
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
	lc::Refuse r = ch[idx].pin.gate(drvActive(idx) && ch[idx].drv.arm.armed);
	if (r != lc::Refuse::OK) return r;
	bool use3d; int16_t s; uint16_t thr;
	r = lc::mapDrive(ch[idx].pin.id, v, use3d, s, thr);
	if (r != lc::Refuse::OK) return r;
	ch[idx].drv.target = ch[idx].drv.stall.gate(millis(), thr); ch[idx].drv.last = millis();   // held at 0 after a stall
	return lc::Refuse::OK;
}
// Restarted after a stall within the last few seconds (a UI can say why it is arming again).
inline bool spinStalled(uint8_t idx) { return idx < COUNT && detail::ch[idx].drv.stall.recent(millis()); }
// Held at 0 after a stall until it is sent 0 (lc::Stall).
inline bool spinHeld(uint8_t idx) {
	using namespace detail;
	return idx < COUNT && ch[idx].pin.st == lc::State::RUNNING && drvActive(idx) && ch[idx].drv.stall.hold;
}
// An answered bidir stream at zero throttle, not held, that has gone quiet (lc::Pin::silent): only at
// zero, because under throttle a silent ESC is the stall watch's (a failed start beeps without
// telemetry and is re-armed); calling it gone there made the page forget a healthy ESC.
inline bool spinSilent(uint8_t idx) {
	using namespace detail;
	uint32_t now = millis();
	// not polled lately (MSP answered right after a blocking op, before spinPoll): no evidence
	return idx < COUNT && ch[idx].drv.b && ch[idx].drv.arm.armed && ch[idx].drv.target == 0 && !ch[idx].drv.stall.hold &&
	       now - ch[idx].drv.pollMs <= POLL_GAP_MS && ch[idx].pin.silent(now);
}
inline bool spinReversible(uint8_t idx) { return idx < COUNT && detail::curIdent(idx).known && detail::curIdent(idx).rev; }
inline const char* spinMode(uint8_t idx) {   // the protocol while driven, else the state (lowercase)
	using namespace detail;
	if (idx >= COUNT) return "none";
	if (ch[idx].drv.initFail) return "init-fail";
	if (pwmOn(idx)) return "pwm";
	if (ch[idx].pin.st == lc::State::RUNNING && drvActive(idx)) return ch[idx].drv.b ? "bidir" : "normal";
	return lc::stateName(ch[idx].pin.st, true);
}
inline const char* streamProto(uint8_t idx) {   // what is on the wire now, whatever the state
	using namespace detail;
	if (idx >= COUNT) return "none";
	if (ch[idx].drv.initFail) return "init-fail";
	return pwmOn(idx) ? "pwm" : ch[idx].drv.b ? "bidir" : ch[idx].drv.n ? "normal" : "none";
}
inline bool spinInitOk(uint8_t idx) { return idx < COUNT && !detail::ch[idx].drv.initFail; }
inline bool spinArming(uint8_t idx) {
	using namespace detail;
	if (idx >= COUNT) return false;
	return ch[idx].pin.armWanted || (ch[idx].pin.st == lc::State::RUNNING && drvActive(idx) && !ch[idx].drv.arm.armed) ||
	       (ch[idx].armReq.start && ch[idx].pin.st == lc::State::RUNNING);
}
inline uint16_t spinTarget(uint8_t idx) { return idx < COUNT ? detail::ch[idx].drv.target : 0; }   // DShot value being sent
inline bool inBootloader(uint8_t idx)   { return detail::session == (int8_t)idx; }
// RUNNING -> READY. The hold keeps the (armed) zero stream, so the next arm from READY is instant.
inline void spinStop(uint8_t idx) {
	using namespace detail;
	if (idx >= COUNT) return;
	ch[idx].drv.target = 0; ch[idx].drv.initFail = false; ch[idx].armReq.start = false; ch[idx].drv.stall.reset();
	ch[idx].pin.onStop();
	if (pwmOn(idx)) pwmOff(idx);
	if (!holdOn && (int8_t)idx != jobIdx) drvFree(idx);
}
inline void spinDisarm(uint8_t idx) { spinStop(idx); }
inline void spinStopAll() { for (uint8_t i = 0; i < COUNT; i++) spinStop(i); }
inline bool spinning()    { for (uint8_t i = 0; i < COUNT; i++) if (detail::drvActive(i)) return true; return false; }
inline bool spinTele(uint8_t idx, Telem& out) {
	if (idx >= COUNT || !detail::ch[idx].drv.b) return false;   // telemetry only on bidir DShot
	out = detail::ch[idx].drv.tele; return true;
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
			if (ch[i].pin.st != lc::State::PENDING || !canRunJob()) { jobIdx = -1; jobPhase = 0; return; }   // stays PENDING
			// The prime: BLHeli-S needs valid frames before the line goes high. A pin whose zero stream
			// has run PRIME_MS already is primed; no SM for it -> go on unprimed (AM32 needs none).
			if (drvActive(i) && now - ch[i].drv.startMs < PRIME_MS) return;
			ch[i].pin.onReadStart(now);                 // (latches whether its telemetry is fresh first)
			drvFree(i); pinMode(PINS[i], OUTPUT); digitalWrite(PINS[i], HIGH);   // core1 owns it from here
			jobAbort = false; opIdx = i; opPin = PINS[i]; opOk = false; opDone = false;
			__sync_synchronize();                    // its inputs before op (core1 reads op first)
			op = Op::READID;
			jobPhase = 2;
			return;
		}
		bool waiting = false;
		for (uint8_t k = 0; k < COUNT; k++) waiting = waiting || ch[k].pin.st == lc::State::PENDING;
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
			if (ch[i].pin.st != lc::State::PENDING) continue;
			jobRr = (uint8_t)(i + 1); jobIdx = (int8_t)i; jobPhase = 1;
			if (pwmOn(i)) pwmOff(i);
			// a pin that has answered keeps its bidir stream; one that never did (an `arm` / read of a
			// pin with nothing heard) is primed with normal DShot, which every firmware takes (Prime)
			bool bidir = ch[i].pin.seen;
			if (!drvActive(i) || (ch[i].drv.b != nullptr) != bidir) drvStart(i, bidir);
			return;
		}
	}
	// Put on pin i what its state allows (lc::Pin::output): free, bidir zero, zero in its protocol,
	// or the drive stream an arm asked for.
	static void applyOutput(uint8_t i, uint32_t now) {
		if (pwmOn(i) || (int8_t)i == jobIdx || (int8_t)i == session) return;
		lc::Out o = ch[i].pin.output(holdOn);
		if (o == lc::Out::NONE) { if (drvActive(i)) drvFree(i); return; }
		if (o == lc::Out::DRIVE) { if (ch[i].armReq.start) { ch[i].armReq.start = false; startArm(i); } return; }
		bool wantB = o == lc::Out::ZERO_BIDIR || ch[i].pin.id.bidir;
		if (drvActive(i) && (ch[i].drv.b != nullptr) == wantB && ch[i].drv.kb == kbaudOf(i)) return;
		if (ch[i].drv.initFail && (int32_t)(now - ch[i].drv.retryMs) < 0) return;   // no SM: retry in a second
		drvStart(i, wantB);
	}
}

inline void spinPoll() {   // call every core0 loop: lifecycle + hold + async reads + frames + telemetry
	using namespace detail;
	uint32_t now = millis();
	for (uint8_t i = 0; i < COUNT; i++) {
		gapCheck(i, now, true);
		ch[i].drv.stall.poll(now);
		lc::Ev e = ch[i].pin.tick(now, policy);
		if (e == lc::Ev::LOST_STOPPED) { ch[i].drv.target = 0; ch[i].armReq.start = false; }   // TOOL: stop that ESC only
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
		if (now - ch[i].pwm.start > PWM_ARM_MS + PWM_RELEASE_MS && millis() - ch[i].pwm.last > PWM_RELEASE_MS) {
			pwmOff(i); ch[i].pin.onStop();
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
// pwmSet() then gives the pulse width, fed like drive() (the same 500 ms deadman falls back
// to the stop width, also from a timer IRQ while core0 is held up; it does not cut the signal). A
// width sent during the arm is not kept, and after it nothing but the stop width passes until one
// stop width has come (lc::pwmGate): a value never waits behind the arm and lands as a jump. During
// a sync bootloader visit the stop width goes out. Unfed for PWM_RELEASE_MS after the arm, the pin
// leaves PWM (READY, the hold takes it). spinStop()/spinArm() end PWM on the pin.
inline bool pwmActive(uint8_t idx) { return idx < COUNT && detail::pwmOn(idx); }
// Arming, or waiting for its first stop width after the arm.
inline bool pwmArming(uint8_t idx) {
	return pwmActive(idx) && (millis() - detail::ch[idx].pwm.start < PWM_ARM_MS || !detail::ch[idx].pwm.zero);
}
// Fed a width above its stop within the deadman: a blocking bootloader visit must wait.
inline bool pwmDriving(uint8_t idx) {
	using namespace detail;
	return pwmActive(idx) && ch[idx].pwm.us != ch[idx].pwm.stop && millis() - ch[idx].pwm.last <= SPIN_DEADMAN_MS;
}
inline bool pwmBegin(uint8_t idx) {
	using namespace detail;
	if (idx >= COUNT) return false;
	if (pwmOn(idx)) return true;
	if (fwActive) return false;
	if (session == (int8_t)idx) closeSession();   // held in its bootloader: learn it, run its app
	if (ch[idx].pin.st != lc::State::READY && ch[idx].pin.st != lc::State::RUNNING) return false;
	// no alarm slot for the IRQ deadman: no PWM (a held-up loop would keep the last width)
	if (!pwmTimerOn) pwmTimerOn = add_repeating_timer_ms(50, pwmWatch, nullptr, &pwmTimer);
	if (!pwmTimerOn) return false;
	ch[idx].drv.target = 0; ch[idx].drv.arm.armed = false; drvFree(idx);
	ch[idx].pin.onRunStart();
	uint gp = PINS[idx], slice = pwm_gpio_to_slice_num(gp);
	gpio_set_function(gp, GPIO_FUNC_PWM);
	pwm_set_clkdiv(slice, clock_get_hz(clk_sys) / 1e6f);       // 1 tick = 1 us
	pwm_set_wrap(slice, 20000 - 1);                             // 50 Hz; a slice's two pins share it
	ch[idx].pwm.stop = ch[idx].pin.id.rev ? PWM_MID_US : PWM_STOP_US;
	pwmLevel(idx, ch[idx].pwm.stop);
	pwm_set_enabled(slice, true);
	ch[idx].pwm.start = ch[idx].pwm.last = millis(); ch[idx].pwm.zero = false;
	ch[idx].pwm.us = ch[idx].pwm.stop;                   // last: pwmOn() is what the IRQ checks
	return true;
}
inline void pwmSet(uint8_t idx, uint16_t us) {
	using namespace detail;
	if (!pwmActive(idx)) return;
	us = us < PWM_MIN_US ? PWM_MIN_US : (us > PWM_MAX_US ? PWM_MAX_US : us);
	ch[idx].pwm.us = lc::pwmGate(millis() - ch[idx].pwm.start, PWM_ARM_MS, ch[idx].pwm.zero, us, ch[idx].pwm.stop);
	ch[idx].pwm.last = millis();
}
inline uint16_t pwmTarget(uint8_t idx) { return pwmActive(idx) ? detail::ch[idx].pwm.us : 0; }

// What the web tool's motor test can do with ESC i now (lc::Ui, from its lifecycle state). Sent in
// MOTOR_TELEMETRY's mAh field with a 0x5A marker (fourway.h); mirrored once in web/js/app.js (UI).
using UiState = lc::Ui;
inline UiState uiState(uint8_t i) {
	using namespace detail;
	if (i >= COUNT) return UiState::ABSENT;
	lc::UiIn in;
	in.pwm = pwmActive(i); in.pwmArming = pwmArming(i);
	in.stream = drvActive(i) && !ch[i].armReq.start; in.armed = in.stream && ch[i].drv.arm.armed;
	in.stalled = spinHeld(i) || (spinStalled(i) && ch[i].pin.st == lc::State::RUNNING && !ch[i].drv.arm.armed);
	in.silent = spinSilent(i);
	return lc::uiCode(ch[i].pin, in);
}

// ---- 4-way passthrough (fourway.h) -------------------------------------------------------------
// fwBegin: stop driving, and hold every signal line high so each ESC drops into its bootloader
// (what Betaflight's esc4wayInit does). fwSelect: make idx the session — the cached reply if it is
// still in its bootloader, else a BootInit, else the full DShot-prime entry — and return its raw
// 8-byte reply. Switching does NOT run the previous ESC; fwEnd runs every ESC that was entered.
// The configurator's channel k is PINS[k], like every other index (MSP motor k, CLI index k): no
// numbering depends on which ESCs answer, so nothing can shift a channel onto another ESC.
// fwBegin primes every configured pin with zero-throttle DShot for PRIME_MS, all at once (in turns
// of as many as there are free PIO SMs), then holds the lines high. A pin with a known bidir ESC that
// did not answer that prime (booting, playing its melody) is primed again by its own InitFlash until
// it answers (lc::Entry). Both firmwares take that signal
// loss as "go to the bootloader": BLHeli-S needs the valid frames first (a bare high line leaves it
// waiting for signal), AM32 needs its reboot. Every pin then has ENTRY_QUIET_MS of silence, in
// parallel, before the first BootInit, and polls from there (see ENTRY_QUIET_MS).
// Each channel's InitFlash then polls its own pin with BootInit: until fwHigh + ENTRY_MAX_MS if its
// lifecycle knew an ESC there (lc::Pin::ptRetry: it may be slow, or stuck in its app) or it answered
// earlier in this passthrough, else until fwHigh + ENTRY_QUIET_MS + ENTRY_BARE_MS (at least
// CONNECT_MIN_MS from its first BootInit either way). fwBegin's prime was the full entry for every
// pin, so a pin that misses its poll is absent for this passthrough: no second entry. A pulled pin
// whose ESC was known costs at most ENTRY_MAX_MS from the lines going high, an EMPTY one at most
// 0.5 s past the quiet window, each charged to its own channel only. Only a pin whose app was run in
// this passthrough (DeviceReset, then selected again) gets a fresh prime: fwBegin's no longer
// applies to it. A miss stays failed for the passthrough: retries answer at once.
// No reply waits past budgetMs: esc-configurator times a 4-way reply out after 1 s and sends the
// command again, and it takes whatever frame comes next as the answer, so a late reply would answer
// the command after it. Not done within the budget: PENDING (the 4-way NACKs it), and the next
// InitFlash for that channel carries on toward the same deadline.
// Lifecycle: every configured pin is READING for the whole passthrough (no hold, no async read).
// fwRun learns the ESC before it runs it (READY, or FAILED); fwEnd sends a pin that was never
// selected back to EMPTY/FAILED if it started there, else to PENDING (it may have been swapped).
inline void fwEnd();
inline void fwBegin() {
	using namespace detail;
	if (fwActive) fwEnd();
	jobStop(true);                               // a background read gives its pin back first
	spinStopAll(); release();
	fwActive = true;
	bool waits[MAX] = { false }, heard[MAX] = { false }, tune[MAX] = { false };
	uint32_t primeStart[MAX] = { 0 };
	for (uint8_t i = 0; i < COUNT; i++) {
		drvFree(i);                              // the hold too: the configurator owns every pin
		ch[i].fw.have = ch[i].fw.miss = ch[i].fw.done = false;
		ch[i].fw.prime.end();
		// a bidir ESC is known here: the prime must hear it (it may be booting, deaf, in its tune)
		waits[i] = ch[i].pin.st != lc::State::EMPTY && (ch[i].pin.id.known ? ch[i].pin.id.bidir : ch[i].pin.seen);
		tune[i] = ch[i].pin.id.known && ch[i].pin.id.tune;
		ch[i].pin.onPassthroughBegin();
	}
	// A pin with a bidir ESC known is primed with bidir DShot (its answer says it heard the prime),
	// every other one with normal DShot (Prime). Bidir fits only pio0's 4 SMs on a Pico W and normal
	// DShot pio1's 3: pins past that are primed in a second turn. Each line goes high right after its
	// own prime (BLHeli-S checks it as it resets into its bootloader).
	uint32_t left = (1u << COUNT) - 1;
	while (left) {
		Prime pr[MAX];
		uint32_t turn = 0;
		for (uint8_t i = 0; i < COUNT; i++) {
			if (!(left >> i & 1)) continue;
			bool ok;
			if (waits[i]) { pr[i].b = newBidir(PINS[i], DSHOT_KBAUD); ok = !pr[i].b->initError(); }
			else          { pr[i].n = newNormal(PINS[i], DSHOT_KBAUD); ok = !pr[i].n->initError(); }
			if (!ok) pr[i].end(); else turn |= 1u << i;
		}
		uint32_t t0 = millis();
		for (uint8_t i = 0; i < COUNT; i++) if (turn >> i & 1) primeStart[i] = t0;
		while (turn && millis() - t0 < PRIME_MS) {
			for (uint8_t i = 0; i < COUNT; i++) {
				if (pr[i].n) { uint16_t z[4] = { 0, 0, 0, 0 }; pr[i].n->sendThrottles(z); }
				if (!pr[i].b) continue;
				pr[i].b->sendThrottle(0);
				uint32_t v; BidirDshotTelemetryType ty = pr[i].b->getTelemetryPacket(&v);
				if (ty != BidirDshotTelemetryType::CHECKSUM_ERROR && ty != BidirDshotTelemetryType::NO_PACKET) heard[i] = true;
			}
			pump(); delayMicroseconds(300);
		}
		for (uint8_t i = 0; i < COUNT; i++) {
			if (!(turn >> i & 1)) continue;
			pr[i].end();
			pinMode(PINS[i], OUTPUT); digitalWrite(PINS[i], HIGH);
		}
		left = turn ? left & ~turn : 0;          // no SM at all: the rest go unprimed (AM32 needs none)
	}
	for (uint8_t i = 0; i < COUNT; i++) { pinMode(PINS[i], OUTPUT); digitalWrite(PINS[i], HIGH); }
	fwBeginMs = millis();
	// A pin whose known bidir ESC did not answer the prime (it was booting, e.g. run at the end of the
	// last passthrough and playing its tune) is primed again by its InitFlash until it answers.
	for (uint8_t i = 0; i < COUNT; i++) {
		uint32_t poll = ch[i].pin.ptRetry() ? ENTRY_MAX_MS : ENTRY_QUIET_MS + ENTRY_BARE_MS;
		if (waits[i] && !heard[i]) ch[i].fw.entry.begin(primeStart[i], true, poll, tune[i]);
		else ch[i].fw.entry.primed(fwBeginMs, poll);
	}
}
enum class FwSel : uint8_t { OK, MISS, PENDING };
inline FwSel fwSelect(uint8_t idx, uint8_t raw[8], bool& am32, uint32_t budgetMs) {
	using namespace detail;
	if (idx >= COUNT) return FwSel::MISS;
	if (drvActive(idx)) drvFree(idx);
	if (ch[idx].fw.miss) return FwSel::MISS;
	uint32_t stop = millis() + budgetMs;
	if (ch[idx].fw.done) {                  // DeviceReset ran its app (the page's flash, reset, read again)
		bool bidir = !ch[idx].pin.id.known || ch[idx].pin.id.bidir;   // it plays its tune now: wait for it
		bool tune = ch[idx].pin.id.known && ch[idx].pin.id.tune;
		ch[idx].fw.done = false; ch[idx].pin.onReadStart(millis());
		ch[idx].fw.entry.begin(millis(), bidir, ENTRY_MAX_MS, tune);
	}
	session = -1; fwIdx = idx;
	bool ok;
	if (ch[idx].fw.have) ok = runOp(Op::FWSELECT, PINS[idx]);      // re-select: compare or adopt the cached reply
	else {                                                         // its entry, carried on from the last call
		EntryRes r = runEntry(idx, ch[idx].fw.entry, ch[idx].fw.prime, stop);
		if (r == EntryRes::LATER) return FwSel::PENDING;
		ok = r == EntryRes::IN;
	}
	ch[idx].fw.have = ok; ch[idx].fw.miss = !ok;
	if (!ok) { ch[idx].pin.onProbeMiss(); return FwSel::MISS; }   // EMPTY before: confirmed absent
	session = (int8_t)idx; sessLearned = sessionWrote = false; sessLastOpMs = millis();
	memcpy(ch[idx].fw.raw, lastRaw, 8); ch[idx].fw.am32 = lastAm32;
	memcpy(raw, lastRaw, 8); am32 = lastAm32;
	return FwSel::OK;
}
inline void fwRun(uint8_t idx) {   // leave idx's bootloader (start its app); forget its session
	using namespace detail;
	if (idx >= COUNT || !ch[idx].fw.have) return;
	fwIdx = idx;
	// Learn it again while it is still in its bootloader: whatever the configurator wrote or flashed
	// is in by now, and the motor test that usually follows then arms without a second bootloader
	// visit (that one stalled the board for ~5 s, and the web test gave up on it).
	bool learned = runOp(Op::FWSELECT, PINS[idx]) && runOp(Op::LEARN, PINS[idx]);
	runOp(Op::FWRUN, PINS[idx]);
	ch[idx].fw.have = false; if (session == (int8_t)idx) session = -1;
	ch[idx].pin.onPassthroughEnd(true, learned, ch[idx].ident); ch[idx].fw.done = true;
	if (ch[idx].pin.st == lc::State::RUNNING) ch[idx].armReq.start = true;   // spinPoll starts it after fwEnd
}
inline void fwEnd() {
	using namespace detail;
	for (uint8_t i = 0; i < COUNT; i++) ch[i].fw.prime.end();
	for (uint8_t i = 0; i < COUNT; i++) fwRun(i);
	for (uint8_t i = 0; i < COUNT; i++)
		if (!ch[i].fw.done) ch[i].pin.onPassthroughEnd(false, false, lc::Identity{});
	fwActive = false;
}
inline uint8_t count() { return COUNT; }

// ---- which ESCs are wired ----------------------------------------------------------------------
// Nothing is stored: a pin is wired when its lifecycle state is not EMPTY (telemetry seen, a read, a
// passthrough probe that answered). Only `escs` and the 3D answer use it: every index (CLI, MSP
// motor, passthrough channel) is the pin index, wired or not, and an unwired pin reports ABSENT.
// A stock BLHeli_S (no telemetry) is EMPTY until it is read once (`scan`, a configurator read).
inline bool     isPresent(uint8_t i) { return i < COUNT && detail::ch[i].pin.st != lc::State::EMPTY; }
inline uint8_t  presentCount() { uint8_t n = 0; for (uint8_t i = 0; i < COUNT; i++) n += isPresent(i); return n; }
// Confirmed absent (lc::Pin::implicitArmSkipped): an implicit arm (the MSP motor test) leaves it alone.
inline bool     absentConfirmed(uint8_t i) { return i < COUNT && detail::ch[i].pin.implicitArmSkipped(); }
inline bool     identFailed(uint8_t i) { return i < COUNT && detail::ch[i].pin.st == lc::State::FAILED; }
// Configured reversible (3D), as read from the ESC on the pin now (false while unknown).
inline bool     knownReversible(uint8_t i) { return spinReversible(i); }
// The AM32 settings block of ESC i, from its identity (0 = not AM32 / not read).
inline uint16_t am32Eeprom(uint8_t i) { return i < COUNT && detail::curIdent(i).am32 ? detail::curIdent(i).am32Eep : 0; }

// ---- the pin list (boot) -----------------------------------------------------------------------
// Use p[0..n) as the signal pins instead of ESC_SIGNAL_PINS. Boot only (EscTool::begin, after
// LittleFS and before the hold starts): nothing may be streaming, held or read yet. Refused (the
// default stays) unless checkPins() allows it; bad = the offending GPIO.
inline escpins::Err setPins(const uint8_t* p, uint8_t n, uint8_t* bad = nullptr) {
	using namespace detail;
	escpins::Err e = checkPins(p, n, bad);
	if (e != escpins::Err::OK) return e;
	if (spinning() || session >= 0 || jobIdx >= 0 || fwActive) return escpins::Err::BUSY;
	memcpy(PINS, p, n); COUNT = n; pinsFromCfg = true;
	return escpins::Err::OK;
}
inline bool pinsCustom() { return pinsFromCfg; }

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
// core1: BootInit on the current pin, again and again until `until` (millis(); 0 = CONNECT_MIN_MS
// from now; already past = none); on a reply, adopt it as the session.
static bool connectHere(bool abortable, uint32_t until = 0) {
	uint32_t t0 = millis();
	if (!until) until = t0 + CONNECT_MIN_MS;
	while ((int32_t)(until - millis()) > 0) {
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
		if (!a.known) return false;                                           // unknown flash code: no guessed block
		uint8_t dir = 0, nm[15];
		if (!bl.readFlash(a.am32Eep + 18, &dir, 1)) return false;           // bi_direction (host/pico_esc/am32.py)
		id = lc::identFromAm32(d.bootInfo[4], dir);
		// tune[] at 48: a melody unless tune[0] is 0xFF (its three short beeps) or the first note is 0
		uint8_t t[6];
		id.tune = bl.readFlash(a.am32Eep + 48, t, 6) && t[0] != 0xFF && (t[4] || t[5]);
		if (bl.readFlash(a.am32Eep - 32, nm, 15)) { memcpy(out.name, nm, 15); out.name[15] = 0; }   // FILE_NAME
		strncpy(out.layout, "AM32", 16);
		return true;
	}
	esc_setup::Settings s;
	if (!esc_setup::read(bl, s)) return false;
	id = lc::identFromSilabs(s.name, s.raw[esc_setup::OFF_LAYOUT_REVISION], s.raw[0x0B]);
	// Bluejay / BlueGill play the startup melody at 0x70 unless its first byte is 0xFF
	id.tune = lc::silabsBidir(s.raw[esc_setup::OFF_LAYOUT_REVISION], nullptr) && s.raw[0x70] != 0xFF;
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
		ok = connectHere(false, opUntil);
		if (ok) takeHello();
	} else if (o == Op::FWSELECT) {
		bl.setSignalPin(opPin); bl.begin();
		if (ch[fwIdx].fw.have) {
			// A SiLabs bootloader answers BootInit again: the fresh reply is adopted (and fwSelect stores
			// it as fw.raw), so an ESC swapped behind the configurator's back is reported as what it is
			// now, and fwRun learns that one. AM32 answers only the first BootInit, and a silent SiLabs
			// falls back the same way: re-adopt the cached reply if it still takes a keepalive.
			if (!ch[fwIdx].fw.am32) {
				uint8_t raw[8] = {0};
				int n = bl.connectRawProbe(raw, 15);
				ok = n >= 8 && raw[0] == '4' && raw[1] == '7' && raw[2] == '1' && bl.adoptReply(raw);
			}
			if (!ok) ok = bl.adoptReply(ch[fwIdx].fw.raw, ch[fwIdx].fw.am32) && bl.keepAlive();
		} else ok = connectHere(false);
		if (ok) takeHello();
	} else if (o == Op::FWRUN) {
		bl.setSignalPin(opPin); bl.begin();
		ok = bl.adoptReply(ch[fwIdx].fw.raw, ch[fwIdx].fw.am32) && bl.run();
	} else if (o == Op::LEARN) {
		Info in; lc::Identity id;
		ok = learnHere(in, id);
		if (ok) { ch[opIdx].info = in; ch[opIdx].ident = id; }
	} else if (o == Op::READID) {
		// The async read of a newly found ESC (core0 has primed it and set the line high): the same
		// single entry as enterBootloader (quiet, then BootInit polled to ENTRY_MAX_MS from line
		// high), learn, run its app. Job-local results only; jobAbort is honoured inside every wait.
		bl.setSignalPin(opPin); bl.begin();
		uint32_t high = millis();
		bool in = holdHigh(ENTRY_QUIET_MS) && (bl.begin(), connectHere(true, high + ENTRY_MAX_MS));
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
	else if (o == Op::PROBE && prPollMs) {   // blpoll: hold, then BootInit again and again; when did it answer
		bl.setSignalPin(opPin); bl.begin(); uint32_t t0 = millis(); bl.holdIdleHigh(prHoldMs); bl.begin();
		prN = 0; prAtMs = 0;
		while (millis() - t0 < prHoldMs + prPollMs) {
			memset(prRaw, 0, sizeof(prRaw)); int n = bl.connectRawProbe(prRaw, 15);
			if (n >= 8 && prRaw[0] == '4' && prRaw[1] == '7' && prRaw[2] == '1') { prN = n; prAtMs = millis() - t0; break; }
		}
		prEdges = prLows = prTotal = 0; ok = true;
	}
	else if (o == Op::PROBE)       { bl.setSignalPin(opPin); bl.begin(); bl.holdIdleHigh(prHoldMs); bl.begin();
	                                 memset(prRaw, 0, sizeof(prRaw)); prN = bl.connectRawProbe(prRaw, 40);
	                                 bl.probeReplyActivity(200, prEdges, prLows, prTotal); ok = true; }
	opOk = ok;
	__sync_synchronize();                            // results (cfg, flBuf, job/ident caches) before opDone
	opDone = true;
}

}  // namespace escs
