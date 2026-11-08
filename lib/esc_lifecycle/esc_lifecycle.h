// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// esc_lifecycle — one state per ESC signal pin (docs/esc-lifecycle.md): what is on the pin, what may
// be sent to it, and what a drive command means for it. Pure decisions only: time comes in as `now`
// (ms), telemetry as "one valid packet", reads as their result. escs:: (src/apps/esc_session.h) owns
// the hardware and acts on what this returns.
//
// PORTABLE + TESTABLE: no Arduino / PIO / pico-sdk headers, so the native g++ test in test/ covers
// every transition (the same pattern as lib/vel_control).
#pragma once
#include <stdint.h>
#include <cstring>

namespace lc {

// LOST: no valid packet for this long once the pin has answered on its current stream (the doc's
// 300-500 ms: a few bad frames are not a loss). Counted only after the first answer, because the gap
// while an ESC reboots out of its bootloader is expected.
static const uint32_t LOST_MS          = 400;
// EMPTY/LOST -> PENDING: this many valid packets inside the window. One frame never holds authority
// (lake: one-frame-never-holds-safety-authority); the window restarts rather than holding the count
// through dropouts, which is the stricter of the two. core0 polls every ~200 us, so a packet counts
// only APPEAR_GAP_MS after the last counted one: 3 packets are >= 30 ms of evidence, not 1 ms.
static const uint8_t  APPEAR_PKTS      = 3;
static const uint32_t APPEAR_WINDOW_MS = 250;
static const uint32_t APPEAR_GAP_MS    = 15;
// A bidir READY pin that never answers on its stream within this long (ArmGate arms it then anyway)
// is reported once (Ev::NO_ANSWER) and kept READY: it is armed after 8 s anyway, as in rev0, and its
// telemetry may just not decode. A RUNNING pin re-armed after a stall (Stall below) that never
// answers again within this long is LOST: it had answered before.
// ANSWER_MAX_TUNE_MS for an ESC with a startup melody (Identity::tune): it is deaf while it plays
// (8.3 s on the bench; at 8 s the pin was taken as unconfirmed and read again, which played it
// again: a read every 9 s, for good).
static const uint32_t ANSWER_MAX_MS    = 8000, ANSWER_MAX_TUNE_MS = 15000;
// After a read the ESC runs its app: it boots (deaf), arms and then answers. Its first answer on the
// stream that follows is due within ANSWER_DUE_MS; a later one may be another ESC, swapped in while
// this one booted and before it could answer (nothing on the wire tells the two apart). That answer
// sends the pin to PENDING, once: a second late answer after the read that follows is taken as this
// ESC's own slow boot. Past ANSWER_MAX_MS the NO_ANSWER rules below apply instead. Measured (the
// `state` ans= column): Bluejay 0.21 0.75 s, AM32 2.21 0.03 s.
// Not with a startup melody (Identity::tune): it plays for seconds (8.3 s on the bench), long
// enough for any swap, and a read again would only play it again.
static const uint32_t ANSWER_DUE_MS = 1500;
// An answered bidir stream at zero with no packet for this long is reported SILENT (Ui::SILENT): the
// web page's hint that LOST (LOST_MS) is coming, or, under Policy::ROV, the open loop.
static const uint32_t SILENT_MS        = 200;

enum class State : uint8_t { EMPTY, PENDING, READING, READY, RUNNING, LOST, FAILED };
inline const char* stateName(State s, bool lower = false) {
	static const char* const UP[] = { "EMPTY", "PENDING", "READING", "READY", "RUNNING", "LOST", "FAILED" };
	static const char* const LO[] = { "empty", "pending", "reading", "ready", "running", "lost", "failed" };
	uint8_t k = (uint8_t)s;
	return k < 7 ? (lower ? LO[k] : UP[k]) : "?";
}

// What LOST-while-RUNNING means. TOOL: stop that ESC and forget it. ROV: keep streaming the last
// command (open loop) and report; the identity is kept.
enum class Policy : uint8_t { TOOL, ROV };

// What the ESC on the pin is, as read through its bootloader. known=false: nothing may be mapped.
struct Identity {
	bool     known = false, bidir = false, rev = false, am32 = false;
	bool     tune = false;  // plays a startup melody: it boots for seconds, deaf (Bluejay 0x70, AM32 tune[])
	uint16_t am32Eep = 0;   // AM32 settings block address (0 = not AM32)
};
// AM32 settings block by the bootloader's flash-size code (web/js/layouts.js, host/pico_esc/am32.py):
// 0x1F = 32 KB part (0x7C00), 0x35 = 64 KB part (0xF800). Any other code: 0, unknown. A guessed
// block would give a made-up bi_direction, and a 3D ESC taken as one-way runs reverse.
inline uint16_t am32EepFor(uint8_t flashCode) {
	return flashCode == 0x1F ? 0x7C00 : flashCode == 0x35 ? 0xF800 : 0;
}
// AM32 (an ARM bootloader id) always speaks bidir DShot and keeps bi_direction in its own settings
// block. An unknown flash code gives known=false: nothing may be mapped.
inline Identity identFromAm32(uint8_t flashCode, uint8_t biDir) {
	Identity id; id.am32Eep = am32EepFor(flashCode);
	if (!id.am32Eep) return id;
	id.known = true; id.bidir = true; id.am32 = true; id.rev = biDir == 1;
	return id;
}
// SiLabs: bidir from the settings layout revision (0x1A02), as the web page and the presets read
// it: Bluejay 200..209, BlueGill 226. The NAME at 0x60 is the user's to edit, so it only marks JESC,
// which keeps the BLHeli_S layout. 3D from motor_direction (1=Norm 2=Rev 3=3D 4=3D-Rev).
inline bool silabsBidir(uint8_t layoutRev, const char* name) {
	return (layoutRev >= 200 && layoutRev <= 209) || layoutRev == 226 || (name && strstr(name, "JESC"));
}
inline Identity identFromSilabs(const char* name, uint8_t layoutRev, uint8_t dirByte) {
	Identity id; id.known = true;
	id.bidir = silabsBidir(layoutRev, name);
	id.rev   = dirByte == 3 || dirByte == 4;
	return id;
}

enum class Ev : uint8_t { NONE, APPEARED, GONE, LOST_IDLE, LOST_STOPPED, LOST_OPENLOOP, RETURNED,
                          READ_OK, READ_FAILED, READ_EMPTY, INIT_FAIL, NO_ANSWER, SLOW_LOOP, LATE_ANSWER };
inline const char* evName(Ev e) {
	static const char* const N[] = { "none", "appeared", "gone", "lost", "lost, stopped", "lost, open loop",
	                                 "telemetry back", "read", "read failed", "nothing there", "init-fail (no free PIO SM)",
	                                 "no telemetry within 8 s (kept, armed anyway; a pull is not seen)",
	                                 "core0 loop slow: LOST / stall checks paused",
	                                 "first answer after its read came late (swapped?): read again" };
	uint8_t k = (uint8_t)e;
	return k < 14 ? N[k] : "?";
}

// Why a drive command was not applied. A refusal never touches the target or feeds the deadman.
enum class Refuse : uint8_t { OK, STATE, NOT_ARMED, REVERSE_ONEWAY };
// What core0 puts on the pin: nothing, bidir zero (detection), zero in the identity's protocol, or the
// drive stream (arm window + target).
enum class Out : uint8_t { NONE, ZERO_BIDIR, ZERO_ID, DRIVE };
enum class Arm : uint8_t { NOW, AFTER_READ, REFUSED };
// The stall watch's verdict (Stall, below) and what the pin does about it (Pin::onStall).
enum class StallKind : uint8_t { NONE, ZERO, QUIET, SILENT };
enum class StallAct : uint8_t { NONE, REARM, LOST, OPEN_LOOP };

struct Pin {
	State    st = State::EMPTY;
	Identity id;
	bool     seen = false;         // a valid packet since the pin was last EMPTY or LOST
	bool     seenAtRead = false;   // ... and fresh (< LOST_MS) when the current read started
	bool     answered = false;     // ... on the current stream
	bool     armWanted = false;    // `arm` came before the identity: RUNNING once the read is OK
	bool     openLoop = false;     // ROV: RUNNING with its telemetry gone
	bool     queued = false;       // PENDING by request (user, arm, abort, a write): never times out
	bool     streaming = false, streamBidir = false;
	// RUNNING under throttle on a stream the stall watch judges (escs:: sets it every loop): its silence
	// there is a stall (a failed start beeps without telemetry), not LOST. rearmed: the stream started
	// over after a stall and must answer again within ANSWER_MAX_MS, or the pin is LOST after all.
	bool     stallWatch = false, rearmed = false;
	bool     noAnswerSaid = false;   // Ev::NO_ANSWER went out for the current stream
	// READY (or RUNNING) on an identity no telemetry has ever confirmed (NO_ANSWER): the ESC may have
	// been swapped before its first answer, so that answer sends the pin to PENDING (read again)
	// instead of trusting the identity.
	bool     unconfirmed = false;
	// The identity was just read: its first answer must come in time (ANSWER_DUE_MS). lateOnce: a late
	// one already sent it to be read again. answerDelay: the last first answer, ms after its stream began.
	bool     fresh = false, lateOnce = false;
	uint32_t answerDelay = 0;
	uint32_t streamMs = 0, pktMs = 0, burstMs = 0, cntMs = 0;
	uint8_t  burst = 0;
	State    prePt = State::EMPTY; // state when the passthrough began
	// Absent, confirmed: a read found nothing there (READ_EMPTY), or the passthrough's probe got no
	// answer on a pin that was EMPTY before it. Any valid packet or a read that succeeds clears it.
	// Only implicit arms (a configurator's motor test sends every channel) skip such a pin
	// (implicitArmSkipped): each would otherwise queue a 5.5 s read in front of the real ESCs.
	bool     absent = false;

	void toEmpty() { st = State::EMPTY; id = Identity{}; seen = queued = armWanted = openLoop = unconfirmed = fresh = lateOnce = false; burst = 0; }
	// LOST forgets `seen` too: what answered is gone, so a read that then fails finds nothing (EMPTY,
	// detection on) rather than FAILED (silent until the user reads it again).
	void toLost()  { st = State::LOST; id = Identity{}; seen = armWanted = openLoop = unconfirmed = fresh = lateOnce = false; burst = 0; }
	void toPending(bool byRequest) {
		st = State::PENDING; id = Identity{}; queued = byRequest; burst = 0; unconfirmed = false;
		if (!byRequest) armWanted = false;   // found by telemetry: nobody asked to drive it
	}
	// Every "read finished OK" exit (async read, sync session, passthrough) lands here, so an arm that
	// waited for the read is honoured exactly once and never outlives it.
	void finish(const Identity& ident) {
		id = ident; st = armWanted ? State::RUNNING : State::READY;
		armWanted = openLoop = unconfirmed = absent = false; fresh = true;
	}
	// Telemetry that is fresh when a read starts is what makes its failure FAILED (something answers
	// but will not read: no automatic retry loop). Old telemetry from before a session or passthrough
	// does not count: the ESC may have been pulled meanwhile, and EMPTY keeps detection on.
	void latchSeen(uint32_t now) { seenAtRead = seen && now - pktMs < LOST_MS; }

	void onStreamStart(uint32_t now, bool bidir) {
		streaming = true; streamBidir = bidir; streamMs = now; answered = false; stallWatch = rearmed = noAnswerSaid = false;
	}
	void onStreamEnd() { streaming = false; answered = false; stallWatch = rearmed = false; }
	// The same stream arms again after a stall: what it answered before no longer counts.
	void onRearm(uint32_t now) { streamMs = now; answered = false; stallWatch = false; rearmed = true; }
	// What the stall watch judges: RUNNING, armed, under throttle, on a stream that has answered (one
	// that never answered is not watched: its telemetry may just not decode), not AM32 (it stays in its
	// run on a stall).
	bool stallWatched(bool armed, uint16_t target) const {
		return st == State::RUNNING && armed && target && streaming && streamBidir && answered && !id.am32;
	}
	// A stall verdict on a RUNNING pin. ZERO / QUIET: the ESC is there and stalled, so the stream arms
	// again with the identity kept (the caller restarts the Stall: held at 0). SILENT (it had turned,
	// then nothing at all): under TOOL it is LOST like a silence at zero, identity dropped, so an ESC
	// swapped in meanwhile is read before it is driven; under ROV it is the open loop (keep streaming
	// the command, report once), as LOST while running is there.
	StallAct onStall(StallKind k, Policy policy, uint32_t now, Ev& ev) {
		ev = Ev::NONE;
		if (k == StallKind::NONE || st != State::RUNNING) return StallAct::NONE;
		if (k == StallKind::SILENT) {
			if (policy == Policy::ROV) { if (!openLoop) { openLoop = true; ev = Ev::LOST_OPENLOOP; } return StallAct::OPEN_LOOP; }
			toLost(); ev = Ev::LOST_STOPPED; return StallAct::LOST;
		}
		onRearm(now); return StallAct::REARM;
	}
	// An answered bidir stream gone quiet (see SILENT_MS); the caller adds "at zero, not held".
	bool silent(uint32_t now) const {
		return (st == State::READY || st == State::RUNNING) && streaming && streamBidir && answered &&
		       (openLoop || now - pktMs >= SILENT_MS);
	}
	// One valid telemetry packet (not CHECKSUM_ERROR / NO_PACKET). The driver keeps only the newest
	// word per read, so these are loop samples, not frames: the debounce counts one per APPEAR_GAP_MS.
	Ev onPacket(uint32_t now) {
		if (st == State::FAILED || st == State::READING) return Ev::NONE;   // FAILED sends nothing anyway
		bool first = streaming && !answered;
		seen = answered = true; absent = false; pktMs = now;
		if (first) {
			answerDelay = now - streamMs;
			if (fresh && !unconfirmed && (st == State::READY || st == State::RUNNING)) {
				fresh = false;
				// past answerMax() it is NO_ANSWER's (READY: unconfirmed; RUNNING: kept, its telemetry
				// may just not decode)
				bool late = !id.tune && answerDelay > ANSWER_DUE_MS && answerDelay < answerMax();
				if (late && !lateOnce) { lateOnce = true; toPending(false); return Ev::LATE_ANSWER; }
				if (!late) lateOnce = false;
			}
		}
		if (burst == 0 || now - cntMs >= APPEAR_GAP_MS) {
			if (burst == 0 || now - burstMs > APPEAR_WINDOW_MS) { burstMs = now; burst = 0; }
			if (burst < 255) burst++;
			cntMs = now;
		}
		if ((st == State::EMPTY || st == State::LOST) && burst >= APPEAR_PKTS) { toPending(false); return Ev::APPEARED; }
		// the first answer on an identity that never had one: read it again before it is driven
		if (unconfirmed && (st == State::READY || st == State::RUNNING)) { toPending(false); return Ev::APPEARED; }
		if (st == State::RUNNING && openLoop) { openLoop = false; return Ev::RETURNED; }
		return Ev::NONE;
	}
	uint32_t answerMax() const { return id.tune ? ANSWER_MAX_TUNE_MS : ANSWER_MAX_MS; }
	// core0 did not poll this pin for gapMs (a long blocking command): its silence then is no evidence.
	void rebase(uint32_t gapMs) { pktMs += gapMs; streamMs += gapMs; burstMs += gapMs; cntMs += gapMs; }
	Ev tick(uint32_t now, Policy policy) {
		switch (st) {
		case State::PENDING:
			if (!queued && seen && now - pktMs >= LOST_MS) { toEmpty(); return Ev::GONE; }
			break;
		case State::READY:
			if (streaming && streamBidir && answered && now - pktMs >= LOST_MS) { toLost(); return Ev::LOST_IDLE; }
			if (streaming && streamBidir && !answered && !noAnswerSaid && now - streamMs >= answerMax()) {
				noAnswerSaid = unconfirmed = true; return Ev::NO_ANSWER;   // kept READY (see ANSWER_MAX_MS)
			}
			break;
		case State::RUNNING:   // unanswered RUNNING keeps 4baed53's "armed after 8 s anyway", except after a stall
			if (streaming && streamBidir &&
			    ((answered && !stallWatch && now - pktMs >= LOST_MS) ||
			     (rearmed && !answered && now - streamMs >= answerMax()))) {
				if (policy == Policy::ROV) { if (!openLoop) { openLoop = true; return Ev::LOST_OPENLOOP; } }
				else { toLost(); return Ev::LOST_STOPPED; }
			}
			break;
		default: break;
		}
		return Ev::NONE;
	}

	// READY arms at once (escs:: reuses an armed zero stream). Before the identity the arm waits for
	// the read; FAILED refuses (no automatic retry of a read that failed).
	Arm requestArm() {
		switch (st) {
		case State::FAILED:  return Arm::REFUSED;
		case State::READY:   onRunStart(); return Arm::NOW;
		case State::RUNNING: return Arm::NOW;
		case State::PENDING: armWanted = queued = true; return Arm::AFTER_READ;
		case State::READING: armWanted = true; return Arm::AFTER_READ;
		default:             toPending(true); armWanted = true; return Arm::AFTER_READ;   // EMPTY, LOST
		}
	}
	void onReadStart(uint32_t now) { latchSeen(now); st = State::READING; id = Identity{}; queued = false; }
	Ev onReadDone(bool ok, const Identity& ident) {
		if (ok && ident.known) { finish(ident); return Ev::READ_OK; }
		armWanted = false; id = Identity{};
		// Nothing answered just before the read (e.g. `arm` on a bare pin, or `scan` after a pull):
		// EMPTY, so FAILED never needs telemetry to leave.
		if (!seenAtRead) { toEmpty(); absent = true; return Ev::READ_EMPTY; }
		st = State::FAILED; return Ev::READ_FAILED;
	}
	void onReadAborted() { toPending(true); }   // a sync op took the line: read it again later
	// A synchronous bootloader session (scan, read, write) holds the ESC: core1 owns the pin. It
	// supersedes an arm that was waiting for a read.
	void onSessionOpen(uint32_t now) { latchSeen(now); st = State::READING; id = Identity{}; armWanted = openLoop = false; queued = false; }
	void onSessionClose(bool learnedOk, const Identity& ident) {
		if (learnedOk && ident.known) finish(ident);
		else { armWanted = false; toPending(true); }
	}
	bool onRunStart() {
		if (st == State::READY) { st = State::RUNNING; armWanted = openLoop = false; }
		return st == State::RUNNING;
	}
	void onStop() {
		armWanted = openLoop = stallWatch = rearmed = false;
		if (st == State::RUNNING) st = State::READY;
	}
	void onPassthroughBegin() { prePt = st; st = State::READING; id = Identity{}; armWanted = openLoop = queued = false; }
	// The passthrough probes every pin after its quiet window (BootInit polled). A pin where an ESC was
	// known when the passthrough began is polled longer (to ENTRY_MAX_MS after the lines went high):
	// that ESC may be slow. A miss there is final for the passthrough (its prime already was the full
	// entry). One that was EMPTY is absent after a short poll (a stock BLHeli_S with no telemetry is in
	// its bootloader by then, and answers).
	bool ptRetry() const { return prePt != State::EMPTY; }
	// The probe got no answer.
	void onProbeMiss() { if (prePt == State::EMPTY) absent = true; }
	// An implicit arm (the MSP motor test) leaves this pin alone: confirmed absent and still EMPTY.
	// An explicit `arm i` / `read i` still tries it.
	bool implicitArmSkipped() const { return absent && st == State::EMPTY; }
	// ran: its bootloader was left by fwRun (learned or not); else it was never selected.
	void onPassthroughEnd(bool ran, bool ok, const Identity& ident) {
		if (ran && ok && ident.known) { finish(ident); return; }
		armWanted = false;
		if (prePt == State::EMPTY) { toEmpty(); return; }
		if (ran || prePt == State::FAILED) { st = State::FAILED; id = Identity{}; return; }
		toPending(true);   // never selected: a swap is possible, read it again
	}

	Out output(bool hold) const {
		switch (st) {
		case State::READING: case State::FAILED: return Out::NONE;
		case State::RUNNING: return Out::DRIVE;
		case State::READY:   return hold ? Out::ZERO_ID : Out::NONE;
		default:             return hold ? Out::ZERO_BIDIR : Out::NONE;   // EMPTY, PENDING, LOST
		}
	}
	// Any command (zero included) needs RUNNING and an armed stream. READY says NOT_ARMED (arm it
	// first: after a disarm or a firmware abort, host scripts rely on that); other states say STATE.
	Refuse gate(bool armed) const {
		if (st == State::RUNNING && armed) return Refuse::OK;
		return (st == State::RUNNING || st == State::READY) ? Refuse::NOT_ARMED : Refuse::STATE;
	}
};

// Signed thrust -> DShot 3D throttle. -1000..+1000, 0 = stop, but full scale is reached at +-953:
// anything past that is clamped (the top 47 units of either direction do nothing more).
inline uint16_t thrust3d(int16_t s) {
	// DShot 3D on the wire: 48..1047 = reverse (48 slow .. 1047 fast), 1048..2047 = forward (1048 slow
	// .. 2047 fast), 0 = stop. sendThrottle() adds 47 to every non-zero value, so the result below is
	// the wire value minus 47: forward 1047+s, reverse 47-s, and past +-953 forward saturates at 2047
	// while reverse would run past 1047 into the FORWARD band (full reverse read as slow forward).
	// Hence the clamp; it keeps every calibrated cmd where it was measured.
	// (Old mapping used 1000+s / 1001+s: forward leaked into the reverse band for s<48, and reverse
	// was INVERTED — tiny |s| -> ~1000 (near max reverse), large |s| -> ~1 (below the 48 floor) —
	// so reverse ran fast at low command, slowed as command rose, and handed off to 6-step almost
	// immediately.)
	if (s >  953) s =  953;
	if (s < -953) s = -953;
	return (s == 0) ? 0 : (s > 0 ? (uint16_t)(1047 + s)      // forward s=1..953 -> wire 1095..2047
	                             : (uint16_t)(47 - s));       // reverse s=-1..-953 -> wire 95..1047
}

// A semantic drive value by identity: a 3D ESC takes signed thrust -1000..1000, a one-way ESC
// throttle 0..2000, and a reverse request to a one-way ESC is refused (not clamped to zero).
inline Refuse mapDrive(const Identity& id, int v, bool& use3d, int16_t& s3d, uint16_t& thr) {
	use3d = false; s3d = 0; thr = 0;
	if (!id.known) return Refuse::STATE;
	if (id.rev) {
		use3d = true; s3d = (int16_t)(v > 1000 ? 1000 : (v < -1000 ? -1000 : v));
		thr = thrust3d(s3d);
		return Refuse::OK;
	}
	if (v < 0) return Refuse::REVERSE_ONEWAY;
	thr = (uint16_t)(v > 2000 ? 2000 : v);
	return Refuse::OK;
}


// A time stamp where 0 means "none": never in the future (millis()|1 was, by 1 ms when millis() was
// even, and now - stamp then wrapped to ~4e9: the 300 ms answer hold and the stall timeouts passed
// at once).
inline uint32_t stamp(uint32_t now) { return now ? now : 1; }
// ms from stamp t to now, 0 if t is ahead (the stamp(0) = 1 case).
inline uint32_t since(uint32_t now, uint32_t t) { int32_t d = (int32_t)(now - t); return d > 0 ? (uint32_t)d : 0; }

// BOOTLOADER ENTRY: one timeline for every way into an ESC's bootloader (escs:: enterBootloader,
// the passthrough's channels). PRIME: zero-throttle DShot, ENTRY_PRIME_MS at least (BLHeli-S needs
// valid frames before the line goes high; a bare high line leaves it waiting for signal). A bidir
// ESC known on the pin is primed until it has answered, up to ENTRY_PRIME_MAX_MS: one that was just
// run out of its bootloader is deaf while it boots and plays its startup tune, and a prime that ends
// inside the tune leaves it waiting in its app. QUIET: the line high, ENTRY_QUIET_MS with no traffic
// at all (a BootInit earlier than ~700 ms keeps Bluejay in its app). POLL: BootInit again and again
// until an answer or pollMs after the line went high (at least ENTRY_POLL_MIN_MS). Measured
// 2026-11-07 (`blpoll`, line high right after the zero-throttle hold, app running):
//   AM32 2.21 (ARIA, startup tune off): a single BootInit answers from 700 ms; polling from 0 ms
//     does not disturb it (answered at 538 ms, 6/6).
//   Bluejay 0.21 (EFM8BB21): a single BootInit answers from 700 ms; polling from 0 or 300 ms keeps
//     it out of its bootloader (no answer within 6 s, 6/6). Hence the quiet window.
//   Quiet 800 or 1000 ms, then poll: both answer the first BootInit (814 / 1014 ms), 12/12.
// ENTRY_MAX_MS covers a slower AM32 build and a startup tune. ENTRY_BARE_MS: the poll past the quiet
// window for a pin with no ESC known (a stock BLHeli_S has the Bluejay timing; an AM32 that sits in
// its bootloader answers at once), and for one whose wait for an answer ran out (likely not there).
// ENTRY_PRIME_MAX_MS / ENTRY_PRIME_TUNE_MS: how long a known bidir ESC is primed for its answer,
// without and with a startup melody (the bench's Bluejay 0.21 with its melody on answers 8.3 s after
// it was run; with it off 0.75 s, the AM32 0.03 s).
static const uint32_t ENTRY_PRIME_MS = 500, ENTRY_PRIME_MAX_MS = 3000, ENTRY_PRIME_TUNE_MS = 15000, ENTRY_QUIET_MS = 1000,
                      ENTRY_MAX_MS = 5000, ENTRY_BARE_MS = 500, ENTRY_POLL_MIN_MS = 500;
struct Entry {
	enum class Do : uint8_t { PRIME, QUIET, POLL, MISS };
	bool     high = false, waitAnswer = false, answered = false;
	uint32_t start = 0, highMs = 0, from = 0, pollMs = ENTRY_MAX_MS, primeMax = ENTRY_PRIME_MAX_MS;
	// A fresh entry, primed from now. pollMs: ENTRY_MAX_MS for a known ESC, else the bare poll. tune:
	// the ESC plays a startup melody (Identity::tune): its answer may take ENTRY_PRIME_TUNE_MS.
	void begin(uint32_t now, bool waitForAnswer, uint32_t poll, bool tune = false) {
		*this = Entry{}; start = now; waitAnswer = waitForAnswer; pollMs = poll;
		primeMax = tune ? ENTRY_PRIME_TUNE_MS : ENTRY_PRIME_MAX_MS;
	}
	// Primed already (the passthrough primes every pin at once), the line high since highAt.
	void primed(uint32_t highAt, uint32_t poll) { *this = Entry{}; high = true; highMs = highAt; pollMs = poll; }
	void onAnswer() { answered = true; }
	// What to do now. PRIME -> QUIET is the moment the line goes high (the caller does it at once).
	Do step(uint32_t now) {
		if (!high) {
			uint32_t t = since(now, start);
			if (t < ENTRY_PRIME_MS || (waitAnswer && !answered && t < primeMax)) return Do::PRIME;
			high = true; highMs = now;
			if (waitAnswer && !answered) pollMs = ENTRY_QUIET_MS + ENTRY_BARE_MS;
		}
		if (since(now, highMs) < ENTRY_QUIET_MS) return Do::QUIET;
		if (!from) from = now;
		return (int32_t)(now - pollEnd()) < 0 ? Do::POLL : Do::MISS;
	}
	uint32_t quietEnd() const { return highMs + ENTRY_QUIET_MS; }
	uint32_t pollEnd() const {
		uint32_t e = highMs + pollMs;
		return (int32_t)(e - from) < (int32_t)ENTRY_POLL_MIN_MS ? from + ENTRY_POLL_MIN_MS : e;
	}
};

// ARM GATE. A drive stream counts as armed once its window (ARM_MS, or the short window of a re-arm
// of an ESC that is armed already) has passed and the ESC has been heard: a normal-DShot stream at
// once, a bidir one once it answered, ARM_ANSWER_HOLD_MS of zero have gone by since, and its EDT
// enable has gone out (EdtSched). Bluejay sends its first telemetry from wait_for_start, i.e. once it
// has armed, and takes DShot commands only there; with EDT safety arm (FORCE_EDT_ARM) it refuses to
// start without EDT (Bluejay.asm wait_for_start_nonzero), and a reboot clears EDT. A bidir stream
// that never answers is armed after ANSWER_MAX_MS anyway (ANSWER_MAX_TUNE_MS with a startup melody).
// Nothing but zero goes out until it is armed.
static const uint32_t ARM_MS = 3000, ARM_ANSWER_HOLD_MS = 300;
struct ArmGate {
	bool     armed = false;
	uint32_t start = 0, answer = 0;   // window start; first answer on this stream (stamp, 0 = none)
	uint16_t ms = 0;                  // this window (0 = ARM_MS)
	void begin(uint32_t now, uint16_t windowMs = 0) { armed = false; start = now; answer = 0; ms = windowMs; }
	void onAnswer(uint32_t now) { if (!answer) answer = stamp(now); }
	// true on the call that arms it. edtPending: EDT enable frames still to go out. tune: it plays a
	// startup melody (Identity::tune), so an unanswered stream waits ANSWER_MAX_TUNE_MS.
	bool tick(uint32_t now, bool bidir, bool edtPending, bool tune = false) {
		if (armed) return false;
		uint32_t t = now - start, win = ms ? ms : ARM_MS;
		bool heard = !bidir || ms || (answer && since(now, answer) >= ARM_ANSWER_HOLD_MS && !edtPending);
		if (t > win && (heard || t > (tune ? ANSWER_MAX_TUNE_MS : ANSWER_MAX_MS))) armed = true;
		return armed;
	}
};

// EDT SCHEDULE: when a bidir stream sends the EDT enable (DShot command 13, extended telemetry:
// V/A/temp, and what FORCE_EDT_ARM waits for) in place of a throttle frame. A burst of
// EDT_BURST_ARM at the stream's start and again once armed (AM32 may answer before it arms), sent
// while arming only once the ESC has answered (it ignores commands during its boot beep), and once
// armed only at zero. Bluejay clears EDT at the end of every run (Bluejay.asm
// exit_run_mode_no_stall) and takes the command only once it is back in wait_for_start, after the
// motor has coasted down: a burst right at the stop is dropped. So after a run a burst goes out every
// EDT_RAN_MS until a fresh eRPM of 0 says it has left run mode (one more burst then), and every
// EDT_REFRESH_MS while stopped.
static const uint32_t EDT_REFRESH_MS = 1000, EDT_RAN_MS = 100, EDT_STILL_MS = 100;
static const uint8_t  EDT_BURST_ARM = 20, EDT_BURST = 10;
struct EdtSched {
	uint8_t  left = 0;        // frames of the current burst still to send
	bool     ran = false;     // driven since the ESC was last seen stopped (eRPM 0)
	uint32_t lastMs = 0;      // last burst while stopped
	void begin(uint32_t now, bool bidir) { left = bidir ? EDT_BURST_ARM : 0; ran = false; lastMs = now; }
	void burst() { left = EDT_BURST_ARM; }
	// One frame. rpm/rpmMs: the last eRPM reading and when it came. true: send EDT, not the throttle.
	bool frame(uint32_t now, bool armed, bool answered, uint16_t target, uint32_t rpm, uint32_t rpmMs) {
		if (armed && target == 0 && left == 0) {
			bool still = rpm == 0 && (int32_t)(now - rpmMs) < (int32_t)EDT_STILL_MS;
			if ((ran && still) || now - lastMs >= (ran ? EDT_RAN_MS : EDT_REFRESH_MS)) {
				left = EDT_BURST; lastMs = now;
				if (still) ran = false;
			}
		}
		if (target) ran = true;
		bool may = armed ? target == 0 : answered;
		if (may && left > 0) { left--; return true; }
		return false;
	}
};

// STALL WATCH (rev0 8805f12, e6ddda2). A bidir ESC that stalls drops out of its run: Bluejay beeps,
// clears EDT and waits to be armed again (Bluejay.asm exit_run_mode), and with EDT safety arm (FORCE_EDT_ARM) it
// then refuses every start. It is watched only armed, under throttle, on a stream that has answered
// (one that never answered is not watched: its telemetry may just not decode), and never for AM32,
// which stays in its run on a stall. Three verdicts (StallKind):
//   ZERO   eRPM read 0 under throttle for STALL_ZERO_MS (so a throttle below the start point counts
//          too: raise past it within 1.5 s). The clock starts only on an eRPM reading of 0.
//   QUIET  not turned yet, and no valid packet of any kind for STALL_SILENT_MS (a failed start beeps
//          without telemetry). Forced sine sends EDT but no eRPM: that flow is never a stall.
//   SILENT the same total silence once it has turned in this run: pulled or unpowered mid-run as
//          likely as stalled, so Pin::onStall makes it LOST (TOOL) or the open loop (ROV).
// ZERO and QUIET re-arm the same stream (zero, answer, EDT; restart()) and hold the target at 0
// until it is sent 0: a jam is never restarted on its own. After STALL_MAX restarts with no run in
// between, only a zero held for STALL_REST_MS frees it.
static const uint32_t STALL_ZERO_MS   = 1500;
static const uint32_t STALL_SILENT_MS = 500;   // so "EDT-only flow is never a stall" holds only while
                                                // a valid EDT packet comes at least every 500 ms
static const uint8_t  STALL_MAX       = 3;
static const uint32_t STALL_REST_MS   = 1000;
static const uint32_t STALL_RECENT_MS = 5000;   // "restarted after a stall" for a UI, this long
struct Stall {
	uint32_t zeroMs = 0;     // since when eRPM has read 0 under throttle
	uint32_t runMs = 0;      // since when eRPM has read > 0 under throttle
	uint32_t stallMs = 0;    // last stall restart (0 = none)
	uint32_t zeroCmd = 0;    // since when the commands have been 0
	uint8_t  stalls = 0;     // restarts since it last ran STALL_ZERO_MS
	bool     spun = false;   // eRPM > 0 seen since the target left 0
	bool     hold = false;   // stalled: target 0 until a zero command

	void reset() { *this = Stall{}; }            // a fresh arm
	void onGap() { zeroMs = 0; }                 // core0 was held up: its silence is no evidence
	// One loop of pin i. watched: see above (anything else clears the run clocks). erpm: this loop's
	// packet was an eRPM reading v; pktMs: the last valid packet of any kind.
	StallKind step(uint32_t now, bool watched, bool erpm, uint32_t v, uint32_t pktMs) {
		if (!watched) { zeroMs = runMs = 0; spun = false; return StallKind::NONE; }
		if (erpm) {
			if (v) {
				zeroMs = 0; spun = true;
				if (!runMs) runMs = stamp(now);
				else if (since(now, runMs) > STALL_ZERO_MS) stalls = 0;   // it runs
			} else { runMs = 0; if (!zeroMs) zeroMs = stamp(now); }
		}
		if (zeroMs && since(now, zeroMs) > STALL_ZERO_MS) return StallKind::ZERO;
		if (since(now, pktMs) > STALL_SILENT_MS) return spun ? StallKind::SILENT : StallKind::QUIET;
		return StallKind::NONE;
	}
	// A re-arm after a ZERO / QUIET stall: counted, and the target held at 0.
	void restart(uint32_t now) {
		zeroMs = runMs = 0; spun = false;
		stallMs = stamp(now); if (stalls < 255) stalls++;
		hold = true;
	}
	// A command after a stall: held at 0 until a zero comes, or after STALL_MAX restarts that never
	// ran, until zeros have come for STALL_REST_MS.
	uint16_t gate(uint32_t now, uint16_t t) {
		if (t) { zeroCmd = 0; return hold ? 0 : t; }
		if (!zeroCmd) zeroCmd = stamp(now);
		if (stalls < STALL_MAX || since(now, zeroCmd) >= STALL_REST_MS) hold = false;
		return 0;
	}
	// The last command was 0, long enough ago: free it even if no more come (the hold sends none).
	void poll(uint32_t now) { if (hold && zeroCmd && since(now, zeroCmd) >= STALL_REST_MS) hold = false; }
	bool recent(uint32_t now) const { return stallMs && since(now, stallMs) < STALL_RECENT_MS; }
};

// SERVO PWM width gate (rev0 e6ddda2). A width sent during the PWM arm (sinceStart < armMs) is not
// kept, and after it nothing but the stop width passes until one stop width has come (zeroSeen): a
// value never waits behind the arm and lands as a jump. us is already clamped to the pulse range.
inline uint16_t pwmGate(uint32_t sinceStart, uint32_t armMs, bool& zeroSeen, uint16_t us, uint16_t stop) {
	if (sinceStart < armMs) return stop;   // arming: not kept
	if (!zeroSeen) { if (us == stop) zeroSeen = true; else return stop; }
	return us;
}

// What the web tool's motor test can do with a pin now. Sent in MSP_MOTOR_TELEMETRY's mAh field with
// a 0x5A marker (src/apps/fourway.h); mirrored once in web/js/app.js (UI).
enum class Ui : uint8_t {
	NONE = 0,
	READING = 1,   // PENDING / READING: being read (an arm waiting for the read says ARMING)
	ARMING = 2,    // DShot arm, PWM arm / waiting for a stop width, or an arm waiting for its read
	ARMED = 3,     // moves at once
	IDLE = 4,      // READY, no stream: a move arms it (~3 s)
	ABSENT = 5,    // EMPTY / LOST / FAILED: not wired, gone, or its read failed
	STALLED = 6,   // stalled: arming again, or held at 0 until it is sent 0
	SILENT = 7,    // armed bidir stream that answered, at zero and not held, silent (Pin::silent)
};
// What escs:: knows about the pin's output besides its state.
struct UiIn {
	bool pwm = false, pwmArming = false;   // on servo PWM; still arming / waiting for a stop width
	bool stream = false, armed = false;    // a DShot stream on the pin; its arm window is over
	bool stalled = false;                  // Stall::hold, or restarted after a stall and not armed yet
	bool silent = false;                   // Pin::silent and at zero, not held
};
inline Ui uiCode(const Pin& p, const UiIn& in) {
	switch (p.st) {
	case State::PENDING: case State::READING: return p.armWanted ? Ui::ARMING : Ui::READING;
	case State::RUNNING:
		if (in.pwm) return in.pwmArming ? Ui::ARMING : Ui::ARMED;
		if (in.stalled) return Ui::STALLED;
		if (!in.stream || !in.armed) return Ui::ARMING;
		return in.silent ? Ui::SILENT : Ui::ARMED;
	case State::READY:
		if (!in.stream) return Ui::IDLE;
		if (!in.armed) return Ui::ARMING;   // the hold is arming it: a move would wait behind that
		return in.silent ? Ui::SILENT : Ui::ARMED;
	default: return Ui::ABSENT;
	}
}

}  // namespace lc
