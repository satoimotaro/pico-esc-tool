// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// Native (host g++) test for the per-pin ESC lifecycle (esc_lifecycle.h). Every transition escs::
// relies on, with time passed in explicitly. A note on what a "packet" is here: the bidir driver's
// getTelemetryPacket() drains the RX FIFO and returns only the newest word, so escs:: calls
// onPacket() at most once per pin per core0 loop (~200 us). The debounce therefore counts loop
// samples, one per APPEAR_GAP_MS at most, and the cases below feed onPacket() at explicit times.
//     g++ -std=c++17 -O2 -Wall -Wextra -Werror -o /tmp/tlc test_esc_lifecycle.cpp && /tmp/tlc
#include "../esc_lifecycle.h"
#include <cstdio>
#include <cstring>

using namespace lc;

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); failures++; } \
	else printf("ok: %s\n", msg); } while (0)

static Identity bidirOneWay() { return identFromAm32(0x35, 0); }                // AM32: bidir, not 3D
static Identity silabs3d()    { return identFromSilabs("BLHeli_S", 3); }        // normal DShot, 3D
static Identity bluejay()     { return identFromSilabs("Bluejay 0.21", 1); }    // bidir, one-way

// Feed n packets, one per loop sample, dt ms apart, starting at t. Returns the time after the last.
static uint32_t packets(Pin& p, uint32_t t, int n, uint32_t dt, Ev* last = nullptr) {
	for (int k = 0; k < n; k++, t += dt) { Ev e = p.onPacket(t); if (last && e != Ev::NONE) *last = e; }
	return t;
}
// A pin read OK as `id` (a user read of a bare pin) and streaming its zero hold from t.
static Pin readyPin(const Identity& id, uint32_t t) {
	Pin p; p.requestArm(); p.onStop(); p.onReadStart(t); p.onReadDone(true, id); p.onStreamStart(t, id.bidir);
	return p;
}
// A pin found by telemetry (3 packets, 20 ms apart from t): PENDING.
static Pin appeared(uint32_t t) { Pin p; p.onStreamStart(t, true); packets(p, t, 3, 20); return p; }

int main() {
	{   // initial state
		Pin p;
		CHECK(p.st == State::EMPTY && !p.id.known, "a new pin is EMPTY with no identity");
		CHECK(p.output(true) == Out::ZERO_BIDIR && p.output(false) == Out::NONE, "EMPTY gets bidir zero only while the hold is on");
	}
	{   // debounce: one sample never leaves EMPTY, three within the window do
		Pin p; p.onStreamStart(0, true);
		p.onPacket(10); p.tick(11, Policy::TOOL);
		CHECK(p.st == State::EMPTY, "1 packet does not leave EMPTY");
		Pin q; q.onStreamStart(0, true);
		Ev e = Ev::NONE; packets(q, 100, 3, 100, &e);      // 100, 200, 300: spans 200 ms
		CHECK(q.st == State::PENDING && e == Ev::APPEARED, "3 packets within 250 ms give PENDING (APPEARED)");
		Pin r; r.onStreamStart(0, true);
		r.onPacket(0); r.onPacket(200); r.onPacket(400);   // the window restarts at 400: count 1
		CHECK(r.st == State::EMPTY, "3 packets spread over 400 ms stay EMPTY (window restarts, not held)");
	}
	{   // m4: loop samples ~200 us apart are one piece of evidence per APPEAR_GAP_MS
		Pin p; p.onStreamStart(0, true);
		for (uint32_t us = 0; us < 10000; us += 200) p.onPacket(us / 1000);   // 50 samples in 10 ms
		CHECK(p.st == State::EMPTY, "50 loop samples within 10 ms do not give PENDING (counted once per 15 ms)");
		Pin q; q.onStreamStart(0, true);
		Ev e = Ev::NONE;
		for (uint32_t ms = 0; ms <= 29; ms++) { Ev x = q.onPacket(ms); if (x != Ev::NONE) e = x; }
		CHECK(q.st == State::EMPTY, "a sample every 1 ms for 29 ms is only 2 counted packets: EMPTY");
		for (uint32_t ms = 30; ms <= 31; ms++) { Ev x = q.onPacket(ms); if (x != Ev::NONE) e = x; }
		CHECK(q.st == State::PENDING && e == Ev::APPEARED, "the third counted packet at 30 ms gives PENDING");
		Pin s; s.onStreamStart(0, true);
		packets(s, 0, 3, 14);
		CHECK(s.st == State::EMPTY, "3 packets 14 ms apart count as 2 (the 14 ms one is inside the gap)");
	}
	{   // PENDING found by telemetry goes back if the telemetry goes
		Pin p = appeared(0);                                // last packet at 40
		CHECK(p.tick(40 + 399, Policy::TOOL) == Ev::NONE && p.st == State::PENDING, "PENDING holds for 399 ms of silence");
		CHECK(p.tick(40 + 400, Policy::TOOL) == Ev::GONE && p.st == State::EMPTY, "PENDING with telemetry gone for 400 ms goes back to EMPTY");
		Pin q; q.requestArm();                              // PENDING by request: no telemetry needed
		CHECK(q.st == State::PENDING && q.tick(100000, Policy::TOOL) == Ev::NONE && q.st == State::PENDING,
		      "PENDING by request never times out");
	}
	{   // read results
		Pin p = appeared(0);
		p.onReadStart(50);
		CHECK(p.st == State::READING && p.output(true) == Out::NONE, "READING sends nothing");
		CHECK(p.onReadDone(true, bluejay()) == Ev::READ_OK && p.st == State::READY && p.id.bidir && !p.id.rev,
		      "read OK gives READY with that identity");
		Pin f = appeared(0); f.onReadStart(50);
		CHECK(f.onReadDone(false, Identity{}) == Ev::READ_FAILED && f.st == State::FAILED, "read fail with fresh telemetry gives FAILED");
		Pin e; e.requestArm(); e.onReadStart(0);
		CHECK(e.onReadDone(false, Identity{}) == Ev::READ_EMPTY && e.st == State::EMPTY && !e.armWanted,
		      "read fail with no telemetry ever seen gives EMPTY (arm on a bare pin)");
		Pin a; a.requestArm(); a.onReadStart(0); a.onReadAborted();
		CHECK(a.st == State::PENDING && a.queued, "an aborted read goes back to PENDING (queued), not FAILED");
	}
	{   // m2: only telemetry fresh at the start of the read makes a failure FAILED
		Pin p = readyPin(bluejay(), 0); p.onPacket(100);   // last answered at 100
		p.onSessionOpen(5000);                              // `read` long after: it may have been pulled
		p.onSessionClose(false, Identity{});                // closed without a learn: PENDING, re-read
		p.onReadStart(9000);
		CHECK(p.onReadDone(false, Identity{}) == Ev::READ_EMPTY && p.st == State::EMPTY,
		      "a session/passthrough re-read of a pulled ESC gives EMPTY (stale telemetry), detection stays on");
		Pin q = readyPin(bluejay(), 0); q.onPacket(100);
		q.onSessionOpen(300);                               // a scan right after it answered
		CHECK(q.onReadDone(false, Identity{}) == Ev::READ_FAILED && q.st == State::FAILED,
		      "a failed read with telemetry fresh (< 400 ms) at its start gives FAILED");
		Pin l = readyPin(bluejay(), 0); l.onPacket(100); l.tick(500, Policy::TOOL);
		l.onSessionOpen(600);
		CHECK(l.onReadDone(false, Identity{}) == Ev::READ_EMPTY && l.st == State::EMPTY,
		      "a scan that fails after LOST gives EMPTY, so a re-plug is seen");
	}
	{   // FAILED is left only by the user
		Pin p = appeared(0); p.onReadStart(50); p.onReadDone(false, Identity{});
		packets(p, 100, 10, 20);
		CHECK(p.st == State::FAILED, "FAILED ignores packets");
		CHECK(p.requestArm() == Arm::REFUSED && p.st == State::FAILED, "FAILED refuses arm");
		CHECK(p.output(true) == Out::NONE && p.tick(100000, Policy::TOOL) == Ev::NONE && p.st == State::FAILED,
		      "FAILED sends nothing and never leaves by itself");
		p.onSessionOpen(1000);
		CHECK(p.st == State::READING, "a user read (scan / configurator session) takes FAILED to READING");
	}
	{   // READY: arm first
		Pin p = readyPin(silabs3d(), 0);
		CHECK(p.gate(false) == Refuse::NOT_ARMED, "READY refuses a non-zero command: NOT_ARMED");
		CHECK(p.output(true) == Out::ZERO_ID && p.output(false) == Out::NONE, "READY gets zero in its own protocol while held");
		CHECK(p.requestArm() == Arm::NOW && p.st == State::RUNNING, "arm makes READY RUNNING at once");
		CHECK(p.gate(false) == Refuse::NOT_ARMED, "RUNNING still in its arm window: NOT_ARMED");
		CHECK(p.gate(true) == Refuse::OK && p.output(true) == Out::DRIVE, "RUNNING and armed accepts");
		Pin e;
		CHECK(e.gate(true) == Refuse::STATE, "EMPTY refuses with its state");
		p.onStop();
		CHECK(p.st == State::READY && p.gate(true) == Refuse::NOT_ARMED, "stop takes RUNNING back to READY");
	}
	{   // LOST while RUNNING, both policies
		for (int rov = 0; rov < 2; rov++) {
			Policy pol = rov ? Policy::ROV : Policy::TOOL;
			Pin p = readyPin(bluejay(), 0); p.requestArm(); p.onStreamStart(1000, true);
			p.onPacket(2000);
			CHECK(p.tick(2399, pol) == Ev::NONE && p.st == State::RUNNING, rov ? "ROV: 399 ms of silence keeps RUNNING"
			                                                                  : "TOOL: 399 ms of silence keeps RUNNING");
			Ev e = p.tick(2401, pol);
			if (!rov) CHECK(e == Ev::LOST_STOPPED && p.st == State::LOST && !p.id.known, "TOOL: silence over 400 ms gives LOST, identity dropped");
			else {
				CHECK(e == Ev::LOST_OPENLOOP && p.st == State::RUNNING && p.openLoop && p.id.known,
				      "ROV: silence over 400 ms stays RUNNING open loop, identity kept");
				CHECK(p.tick(3000, pol) == Ev::NONE, "ROV: open loop is reported once");
				CHECK(p.onPacket(3100) == Ev::RETURNED && !p.openLoop, "ROV: telemetry back is reported, closed loop again");
			}
		}
		Pin q = readyPin(bluejay(), 0); q.requestArm(); q.onStreamStart(0, true);
		CHECK(q.tick(20000, Policy::TOOL) == Ev::NONE && q.st == State::RUNNING, "RUNNING that never answered is not LOST (armed after 8 s anyway)");
		Pin n = readyPin(silabs3d(), 0); n.requestArm(); n.onStreamStart(0, false);
		CHECK(n.tick(20000, Policy::TOOL) == Ev::NONE && n.st == State::RUNNING, "normal DShot (no telemetry) is never LOST");
	}
	{   // LOST comes back through PENDING
		Pin p = readyPin(bluejay(), 0); p.onPacket(100);
		CHECK(p.tick(500, Policy::TOOL) == Ev::LOST_IDLE && p.st == State::LOST, "READY bidir silent 400 ms: LOST");
		CHECK(p.output(true) == Out::ZERO_BIDIR, "LOST gets bidir zero (identity dropped)");
		Ev e = Ev::NONE; packets(p, 1000, 3, 20, &e);
		CHECK(p.st == State::PENDING && e == Ev::APPEARED, "LOST plus packets gives PENDING");
	}
	{   // a swap never carries the old identity
		Pin p = readyPin(identFromSilabs("Bluejay", 3), 0); p.onPacket(10);
		p.tick(500, Policy::TOOL);
		bool clean = p.st == State::LOST && !p.id.known;
		packets(p, 1000, 3, 20);
		clean = clean && p.st == State::PENDING && !p.id.known;
		p.onReadStart(1050);
		clean = clean && !p.id.known;
		p.onReadDone(true, bidirOneWay());
		CHECK(clean && p.st == State::READY && p.id.am32 && !p.id.rev, "swap READY->LOST->PENDING->READING->READY never carries the old identity");
	}
	{   // arm before the identity
		Pin p = appeared(0);
		CHECK(p.requestArm() == Arm::AFTER_READ && p.armWanted, "arm in PENDING sets armWanted");
		p.onReadStart(50);
		CHECK(p.requestArm() == Arm::AFTER_READ && p.armWanted, "arm in READING sets armWanted");
		p.onReadDone(true, bluejay());
		CHECK(p.st == State::RUNNING && !p.armWanted, "a successful read with armWanted makes it RUNNING");
		Pin q; q.requestArm(); q.onStop();
		CHECK(!q.armWanted, "stop clears armWanted");
		q.onReadStart(0); q.onReadDone(true, bluejay());
		CHECK(q.st == State::READY, "a read after stop gives READY, not RUNNING");
	}
	{   // m1: armWanted never outlives the read it waited for, on every exit
		Pin a = readyPin(bluejay(), 0); a.onPassthroughBegin(); a.requestArm();
		CHECK(a.armWanted, "arm during a passthrough read waits (armWanted)");
		a.onPassthroughEnd(true, true, bluejay());
		CHECK(a.st == State::RUNNING && !a.armWanted, "passthrough end, learned: the waiting arm makes it RUNNING once");
		Pin b = readyPin(bluejay(), 0); b.onPassthroughBegin(); b.requestArm();
		b.onPassthroughEnd(false, false, Identity{});
		CHECK(b.st == State::PENDING && !b.armWanted, "passthrough end, never selected: PENDING, armWanted cleared");
		Pin c = readyPin(bluejay(), 0); c.onPassthroughBegin(); c.requestArm();
		c.onPassthroughEnd(true, false, Identity{});
		CHECK(c.st == State::FAILED && !c.armWanted, "passthrough end, not learned: FAILED, armWanted cleared");
		Pin d = readyPin(bluejay(), 0); d.onSessionOpen(0); d.requestArm();
		d.onSessionClose(true, bluejay());
		CHECK(d.st == State::RUNNING && !d.armWanted, "session closed with a learn: the waiting arm makes it RUNNING once");
		Pin e = readyPin(bluejay(), 0); e.onSessionOpen(0); e.requestArm();
		e.onSessionClose(false, Identity{});
		CHECK(e.st == State::PENDING && !e.armWanted, "session closed without a learn: PENDING, armWanted cleared");
		Pin f = readyPin(bluejay(), 0); f.onSessionOpen(0);
		f.onSessionClose(true, bluejay());
		CHECK(f.st == State::READY, "session closed with a learn and no arm waiting: READY");
	}
	{   // the passthrough probe: the full entry only where an ESC was known
		Pin e; e.onPassthroughBegin();
		CHECK(!e.ptRetry(), "passthrough probe: an EMPTY pin that does not answer is absent (no entry ladder)");
		e.onPassthroughEnd(false, false, Identity{});
		CHECK(e.st == State::EMPTY, "... and stays EMPTY after the passthrough");
		Pin r = readyPin(bluejay(), 0); r.onPassthroughBegin();
		CHECK(r.ptRetry(), "a READY pin gets the entry ladder (its ESC may be stuck in its app)");
		Pin p = appeared(0); p.onPassthroughBegin();
		CHECK(p.ptRetry(), "a PENDING pin gets the entry ladder");
		Pin l = readyPin(bluejay(), 0);
		uint32_t t = packets(l, 10, 3, 20);
		l.tick(t + LOST_MS, Policy::TOOL);
		CHECK(l.st == State::LOST, "(a READY bidir pin whose telemetry stops is LOST)");
		l.onPassthroughBegin();
		CHECK(l.ptRetry(), "a LOST pin gets the entry ladder (identity dropped, but something was there)");
		Pin f; f.onPassthroughBegin(); f.onPassthroughEnd(true, false, Identity{});
		CHECK(f.st == State::EMPTY, "an EMPTY pin that answered but did not read stays EMPTY");
	}
	{   // the absent latch: implicit arms (a configurator's motor test) skip a confirmed-absent pin
		Pin p;
		CHECK(!p.implicitArmSkipped(), "a new pin is not confirmed absent (it may hold a stock BLHeli_S)");
		p.requestArm(); p.onReadStart(10); p.onReadDone(false, Identity{});
		CHECK(p.st == State::EMPTY && p.implicitArmSkipped(), "READ_EMPTY confirms it absent");
		p.onStreamStart(20, true); p.onPacket(30);
		CHECK(!p.implicitArmSkipped(), "any valid packet clears it");
		Pin q; q.onPassthroughBegin(); q.onProbeMiss(); q.onPassthroughEnd(false, false, Identity{});
		CHECK(q.st == State::EMPTY && q.implicitArmSkipped(), "a probe miss on a pin EMPTY before the passthrough confirms it");
		q.onPassthroughBegin(); q.onPassthroughEnd(true, true, bluejay());
		CHECK(q.st == State::READY && !q.absent, "a read that succeeds clears it");
		Pin r = readyPin(bluejay(), 0); r.onPassthroughBegin(); r.onProbeMiss();
		CHECK(!r.absent, "a probe miss on a pin that knew an ESC does not confirm absence");
		Pin t; t.requestArm(); t.onReadStart(10); t.onReadDone(false, Identity{});
		t.requestArm();
		CHECK(t.st == State::PENDING && !t.implicitArmSkipped(), "an explicit arm still tries it (PENDING, read queued)");
		Pin u = appeared(0);
		uint32_t tt = u.pktMs + LOST_MS; u.tick(tt, Policy::TOOL);
		CHECK(u.st == State::EMPTY && !u.implicitArmSkipped(), "GONE (telemetry stopped before a read) is not a confirmed absence");
	}
	{   // a bidir READY pin that never answers
		Pin p = readyPin(bluejay(), 5000);
		CHECK(p.tick(5000 + 7999, Policy::TOOL) == Ev::NONE, "bidir READY unanswered for 7999 ms stays READY");
		CHECK(p.tick(5000 + 8000, Policy::TOOL) == Ev::NO_ANSWER && p.st == State::READY && p.id.known,
		      "M2: bidir READY with no answer within 8 s is reported (NO_ANSWER) and kept READY");
		CHECK(p.tick(5000 + 9000, Policy::TOOL) == Ev::NONE && p.st == State::READY, "... reported once");
		p.onPacket(14500);
		CHECK(p.st == State::PENDING && !p.id.known && !p.unconfirmed,
		      "an identity no answer confirmed: the first answer sends it to PENDING (read again before driven)");
		Pin c = readyPin(bluejay(), 0); c.tick(8000, Policy::TOOL);
		CHECK(c.unconfirmed && c.st == State::READY && c.requestArm() == Arm::NOW, "unconfirmed READY still arms (armed after 8 s anyway)");
		CHECK(c.onPacket(9000) == Ev::APPEARED && c.st == State::PENDING && c.gate(true) != Refuse::OK,
		      "... and a swapped ESC's first answer while RUNNING stops it and reads it (never driven on the old identity)");
		c.onReadStart(9100); c.onReadDone(true, bluejay());
		CHECK(c.st == State::READY && !c.unconfirmed, "a read confirms it again");
		Pin a = readyPin(bluejay(), 0); a.onPacket(100);
		CHECK(a.st == State::READY && !a.unconfirmed, "an answer within 8 s: nothing unconfirmed, READY stays");
		Pin n = readyPin(silabs3d(), 0);
		CHECK(n.tick(100000, Policy::TOOL) == Ev::NONE && n.st == State::READY, "normal-DShot READY never goes LOST");
	}
	{   // a core0 gap is not silence
		Pin p = readyPin(bluejay(), 0); p.onPacket(100);
		p.rebase(5000);
		CHECK(p.tick(5399, Policy::TOOL) == Ev::NONE && p.st == State::READY, "rebase: a 5 s blocking gap does not read as LOST");
	}
	{   // passthrough mapping
		Pin e, f, r, u;
		f = appeared(0); f.onReadStart(50); f.onReadDone(false, Identity{});
		r = readyPin(bluejay(), 0);
		u = readyPin(bluejay(), 0);
		e.onPassthroughBegin(); f.onPassthroughBegin(); r.onPassthroughBegin(); u.onPassthroughBegin();
		CHECK(e.st == State::READING && f.st == State::READING && r.st == State::READING && !r.id.known,
		      "passthrough begin: every present pin READING, identity dropped");
		e.onPassthroughEnd(false, false, Identity{});
		f.onPassthroughEnd(false, false, Identity{});
		u.onPassthroughEnd(false, false, Identity{});
		CHECK(e.st == State::EMPTY && f.st == State::FAILED && u.st == State::PENDING && u.queued,
		      "never selected: EMPTY/FAILED stay, others PENDING (a swap is possible)");
		r.onPassthroughEnd(true, true, silabs3d());
		CHECK(r.st == State::READY && r.id.rev, "run and learned: READY with the new identity");
		Pin x; x.onPassthroughBegin(); x.onPassthroughEnd(true, false, Identity{});
		Pin y = readyPin(bluejay(), 0); y.onPassthroughBegin(); y.onPassthroughEnd(true, false, Identity{});
		CHECK(x.st == State::EMPTY && y.st == State::FAILED, "run but not learned: FAILED, or EMPTY if it was EMPTY");
		Pin s = readyPin(bluejay(), 0); s.onSessionOpen(0);
		CHECK(s.st == State::READING && !s.id.known, "a sync session makes the pin READING");
		s.onSessionClose(false, Identity{});
		CHECK(s.st == State::PENDING && s.queued, "a session closed without a learn goes PENDING");
		s.onSessionOpen(0); s.onSessionClose(true, silabs3d());
		CHECK(s.st == State::READY && s.id.rev, "a session closed with a learn gives READY");
	}
	{   // semantic drive
		bool use3d; int16_t s; uint16_t thr;
		CHECK(mapDrive(bluejay(), -300, use3d, s, thr) == Refuse::REVERSE_ONEWAY, "reverse on a one-way identity: REVERSE_ONEWAY");
		CHECK(mapDrive(bluejay(), 2500, use3d, s, thr) == Refuse::OK && !use3d && thr == 2000, "one-way: 0..2000, clamped");
		CHECK(mapDrive(Identity{}, 100, use3d, s, thr) == Refuse::STATE, "no identity: nothing is mapped");
		CHECK(mapDrive(silabs3d(), -300, use3d, s, thr) == Refuse::OK && use3d && s == -300 && thr == 347, "3D: signed thrust");
		CHECK(mapDrive(silabs3d(), -40000, use3d, s, thr) == Refuse::OK && s == -1000 && thr == 1000, "3D: an int beyond int16 clamps, never wraps");
		CHECK(thrust3d(1) == 1048 && thrust3d(-1) == 48, "3D map: +1 -> 1048, -1 -> 48");
		CHECK(thrust3d(1000) == 2000 && thrust3d(-1000) == 1000, "3D map: +-1000 clamp to +-953 (2000 and 1000)");
		CHECK(thrust3d(0) == 0, "3D map: 0 -> 0 (stop)");
	}
	{   // identity rules
		Identity a = identFromAm32(0x35, 1), b = identFromAm32(0x1F, 0);
		CHECK(a.bidir && a.rev && a.am32 && a.am32Eep == 0xF800 && b.am32Eep == 0x7C00 && !b.rev, "AM32 identity: always bidir, eep by flash code");
		Identity j = identFromSilabs("JESC", 4), s = identFromSilabs("BLHeli_S", 1);
		CHECK(j.bidir && j.rev && !s.bidir && !s.rev && !s.am32, "SiLabs identity: bidir by name, 3D by direction byte");
	}
	{   // rev0 stamp(): 0 means "none", and a stamp is never in the future
		CHECK(stamp(0) == 1 && stamp(7) == 7 && stamp(8) == 8, "stamp(): 0 -> 1, anything else unchanged (millis()|1 was 9 for 8)");
	}
	{   // stall watch: the zero rule (eRPM 0 under throttle for STALL_ZERO_MS)
		Stall s;
		StallKind k = StallKind::NONE;
		for (uint32_t t = 1000; t <= 1000 + STALL_ZERO_MS && k == StallKind::NONE; t += 10) k = s.step(t, true, true, 0, t);
		CHECK(k == StallKind::NONE, "eRPM 0 under throttle for exactly 1.5 s: not yet a stall");
		CHECK(s.step(1000 + STALL_ZERO_MS + 1, true, true, 0, 2501) == StallKind::ZERO, "eRPM 0 for over 1.5 s: ZERO");
		s.restart(2501);
		CHECK(s.hold && s.stalls == 1, "restart(): held and counted");
		CHECK(s.gate(2600, 300) == 0, "held: a throttle after the stall is sent as 0");
		CHECK(s.gate(2610, 0) == 0 && !s.hold, "a zero command frees it (fewer than STALL_MAX stalls)");
		CHECK(s.gate(2620, 300) == 300, "then a throttle passes again");
	}
	{   // M1: forced sine (EDT, no eRPM) is never a stall; a not-yet-turned pin stalls on total silence
		Stall s; StallKind k = StallKind::NONE;
		for (uint32_t t = 100; t <= 10000 && k == StallKind::NONE; t += 1) k = s.step(t, true, false, 0, t);   // EDT every ms
		CHECK(k == StallKind::NONE, "EDT-only under throttle for 10 s is never a stall (forced sine)");
		Stall q; k = StallKind::NONE; uint32_t at = 0;
		for (uint32_t t = 100; t <= 2000 && k == StallKind::NONE; t += 10) { k = q.step(t, true, false, 0, 100); at = t; }
		CHECK(k == StallKind::QUIET && at > 100 + STALL_SILENT_MS && at <= 100 + STALL_SILENT_MS + 10,
		      "not turned, no packet of any kind for over 500 ms: QUIET (a failed start beeps without telemetry)");
		Stall z; z.step(100, true, false, 0, 100);
		CHECK(z.zeroMs == 0, "the zero clock does not start without an eRPM reading of 0");
	}
	{   // stall watch: the silence rule after it has turned
		Stall s;
		s.step(10, true, true, 500, 10);                     // it turns
		CHECK(s.step(400, true, false, 0, 10) == StallKind::NONE, "turned, then 390 ms without a packet: no stall yet");
		CHECK(s.step(511, true, false, 0, 10) == StallKind::SILENT, "turned, then over 500 ms without any packet: SILENT");
		Stall q;
		StallKind k = StallKind::NONE;
		for (uint32_t t = 10; t < 1400 && k == StallKind::NONE; t += 10) k = q.step(t, true, t % 100 == 0, 0, t - t % 100);
		CHECK(k == StallKind::NONE, "eRPM 0 every 100 ms for 1.4 s: neither rule yet");
		Stall w;
		w.step(10, true, true, 500, 10);
		CHECK(w.step(5000, false, false, 0, 10) == StallKind::NONE && !w.spun, "not watched (target 0, AM32, not armed, never answered): clocks cleared, never a stall");
		Stall g; g.step(10, true, true, 0, 10); g.onGap();
		CHECK(g.step(1600, true, true, 0, 1600) == StallKind::NONE, "a loop held up restarts the zero clock (no stall counted across the gap)");
	}
	{   // stall watch: STALL_MAX restarts that never ran need a zero held STALL_REST_MS
		Stall s; uint32_t t = 10;
		for (int k = 0; k < STALL_MAX; k++) {
			for (uint32_t u = 0; u <= STALL_ZERO_MS + 10 && s.step(t, true, true, 0, t) != StallKind::ZERO; u += 10) t += 10;
			s.restart(t);
			t += 10; s.gate(t, 0); t += 10; s.gate(t, 300);
		}
		CHECK(s.stalls == STALL_MAX && s.hold, "after STALL_MAX stalls in a row a zero command does not free it at once");
		s.gate(t + 10, 0);
		CHECK(s.hold && s.gate(t + 20, 300) == 0, "... nor a throttle right after it");
		s.gate(t + 30, 0); s.poll(t + 30 + STALL_REST_MS);
		CHECK(!s.hold, "a zero held STALL_REST_MS frees it (also with no command coming, via poll)");
		Stall r; r.step(10, true, true, 0, 10); bool rs = r.step(1511, true, true, 0, 1511) == StallKind::ZERO;
		r.restart(1511); r.gate(1520, 0);
		r.step(2000, true, true, 900, 2000); r.step(3600, true, true, 900, 3600);
		CHECK(rs && r.stalls == 0, "running for over 1.5 s clears the stall count");
		CHECK(r.recent(1511 + STALL_RECENT_MS - 1) && !r.recent(1511 + STALL_RECENT_MS), "recent(): within 5 s of the restart");
	}
	{   // stall verdicts on the pin: ZERO/QUIET re-arm (identity kept); SILENT is LOST (TOOL, m1) or open loop (ROV, M3)
		Ev e;
		Pin a = readyPin(bluejay(), 0); a.requestArm(); a.onPacket(100);
		CHECK(a.onStall(StallKind::ZERO, Policy::TOOL, 1700, e) == StallAct::REARM && a.st == State::RUNNING && a.id.known &&
		      a.rearmed && !a.answered && e == Ev::NONE, "ZERO: re-armed on the same stream, identity kept");
		Pin q = readyPin(bluejay(), 0); q.requestArm(); q.onPacket(100);
		CHECK(q.onStall(StallKind::QUIET, Policy::ROV, 700, e) == StallAct::REARM && q.st == State::RUNNING,
		      "QUIET (failed start): re-armed under ROV too");
		Pin t = readyPin(bluejay(), 0); t.requestArm(); t.onPacket(100);
		CHECK(t.onStall(StallKind::SILENT, Policy::TOOL, 700, e) == StallAct::LOST && t.st == State::LOST && !t.id.known &&
		      e == Ev::LOST_STOPPED, "m1 TOOL: SILENT after turning is LOST, identity dropped");
		packets(t, 800, 3, 20);
		CHECK(t.st == State::PENDING, "... so the next answer goes PENDING (read before it is driven)");
		Pin r = readyPin(bluejay(), 0); r.requestArm(); r.onPacket(100);
		CHECK(r.onStall(StallKind::SILENT, Policy::ROV, 700, e) == StallAct::OPEN_LOOP && r.st == State::RUNNING && r.openLoop &&
		      r.id.known && e == Ev::LOST_OPENLOOP, "M3 ROV: SILENT after turning is the open loop, identity kept");
		CHECK(r.onStall(StallKind::SILENT, Policy::ROV, 710, e) == StallAct::OPEN_LOOP && e == Ev::NONE, "... reported once");
		CHECK(r.onPacket(800) == Ev::RETURNED && !r.openLoop, "... and telemetry back ends it");
		Pin n = readyPin(bluejay(), 0);
		CHECK(n.onStall(StallKind::ZERO, Policy::TOOL, 100, e) == StallAct::NONE && n.st == State::READY, "a verdict on a READY pin does nothing");
	}
	{   // stall vs LOST in the lifecycle: under throttle the watch owns silence; a re-arm must answer
		Pin p = readyPin(bluejay(), 0); p.requestArm(); p.onPacket(100);
		p.stallWatch = true;
		CHECK(p.tick(100 + LOST_MS + 50, Policy::TOOL) == Ev::NONE && p.st == State::RUNNING,
		      "RUNNING under throttle (stall watch): 450 ms of silence is not LOST");
		p.onRearm(600);
		CHECK(p.tick(600 + LOST_MS + 50, Policy::TOOL) == Ev::NONE && p.st == State::RUNNING, "re-armed after a stall: not answered yet, not LOST");
		CHECK(p.tick(600 + ANSWER_MAX_MS, Policy::TOOL) == Ev::LOST_STOPPED && p.st == State::LOST && !p.id.known,
		      "a stall re-arm that never answers within 8 s: LOST, identity dropped");
		Pin q = readyPin(bluejay(), 0); q.requestArm(); q.onPacket(100); q.onRearm(200); q.onPacket(900);
		CHECK(q.tick(1200, Policy::TOOL) == Ev::NONE && q.tick(900 + LOST_MS, Policy::TOOL) == Ev::LOST_STOPPED,
		      "a re-arm that answered: LOST again on 400 ms of silence at zero");
		Pin r = readyPin(bluejay(), 0); r.requestArm(); r.onPacket(100); r.stallWatch = true; r.onStop();
		CHECK(!r.stallWatch && !r.rearmed, "stop clears the stall flags");
		Pin u = readyPin(bluejay(), 0); u.requestArm();   // M2: never answered on its stream
		CHECK(!u.answered && u.tick(20000, Policy::TOOL) == Ev::NONE && u.st == State::RUNNING,
		      "M2: RUNNING that never answered is not LOST (armed after 8 s anyway)");
		CHECK(!u.stallWatched(true, 300), "M2: a stream that never answered is not stall-watched under throttle");
		u.onPacket(30000);
		CHECK(u.stallWatched(true, 300) && !u.stallWatched(true, 0) && !u.stallWatched(false, 300),
		      "answered, armed, under throttle: watched; at zero or not armed: not");
		Pin m = readyPin(bidirOneWay(), 0); m.requestArm(); m.onPacket(100);
		CHECK(!m.stallWatched(true, 300), "AM32 is never stall-watched");
	}
	{   // SILENT: an answered bidir stream gone quiet, before LOST
		Pin p = readyPin(bluejay(), 0); p.onPacket(100);
		CHECK(!p.silent(100 + SILENT_MS - 1) && p.silent(100 + SILENT_MS), "READY answered stream: silent after 200 ms");
		Pin n = readyPin(silabs3d(), 0);
		CHECK(!n.silent(5000), "normal DShot (no telemetry) is never silent");
		Pin e; e.onStreamStart(0, true); e.onPacket(10);
		CHECK(!e.silent(5000), "EMPTY is never silent (only READY/RUNNING)");
	}
	{   // PWM width gate (rev0): arm widths dropped, then a stop width first
		bool z = false;
		CHECK(pwmGate(100, 5000, z, 1600, 1000) == 1000 && !z, "a width during the arm is not kept");
		CHECK(pwmGate(5000, 5000, z, 1600, 1000) == 1000 && !z, "after the arm, a width before any stop width: stop");
		CHECK(pwmGate(5100, 5000, z, 1000, 1000) == 1000 && z, "a stop width opens it");
		CHECK(pwmGate(5200, 5000, z, 1600, 1000) == 1600, "then widths pass");
		bool z3 = false;
		CHECK(pwmGate(6000, 5000, z3, 1000, 1500) == 1500 && !z3, "3D: 1000 us (full reverse) is not its stop and does not open it");
	}
	{   // UI codes (MSP_MOTOR_TELEMETRY mAh field) from the lifecycle state
		UiIn none;
		Pin e;
		CHECK(uiCode(e, none) == Ui::ABSENT, "EMPTY -> 5");
		Pin pe = appeared(0);
		CHECK(uiCode(pe, none) == Ui::READING, "PENDING -> 1");
		pe.requestArm();
		CHECK(uiCode(pe, none) == Ui::ARMING, "PENDING with an arm waiting for the read -> 2");
		Pin r = readyPin(bluejay(), 0);
		CHECK(uiCode(r, none) == Ui::IDLE, "READY without a stream -> 4");
		UiIn st; st.stream = true;
		CHECK(uiCode(r, st) == Ui::ARMING, "READY, hold stream still arming -> 2");
		st.armed = true;
		CHECK(uiCode(r, st) == Ui::ARMED, "READY with an armed hold stream -> 3");
		UiIn si = st; si.silent = true;
		CHECK(uiCode(r, si) == Ui::SILENT, "READY, armed, silent at zero -> 7");
		r.requestArm();
		UiIn sh = st; sh.stalled = true;
		CHECK(uiCode(r, sh) == Ui::STALLED, "RUNNING, stalled / held -> 6");
		CHECK(uiCode(r, st) == Ui::ARMED && uiCode(r, none) == Ui::ARMING, "RUNNING armed -> 3; RUNNING before its stream -> 2");
		UiIn pw; pw.pwm = true; pw.pwmArming = true;
		CHECK(uiCode(r, pw) == Ui::ARMING, "PWM arming / waiting for a stop width -> 2");
		pw.pwmArming = false;
		CHECK(uiCode(r, pw) == Ui::ARMED, "PWM past its arm -> 3");
		Pin l = readyPin(bluejay(), 0); l.onPacket(10); l.tick(10 + LOST_MS, Policy::TOOL);
		CHECK(l.st == State::LOST && uiCode(l, st) == Ui::ABSENT, "LOST -> 5");
		CHECK((uint8_t)Ui::STALLED == 6 && (uint8_t)Ui::SILENT == 7 && (uint8_t)Ui::ABSENT == 5, "the codes the page reads are fixed");
	}
	{   // names
		const char* want[] = { "EMPTY", "PENDING", "READING", "READY", "RUNNING", "LOST", "FAILED" };
		bool all = true;
		for (uint8_t k = 0; k < 7; k++) all = all && !strcmp(stateName((State)k), want[k]);
		CHECK(all && !strcmp(stateName(State::LOST, true), "lost"), "stateName() returns all 7 names");
	}

	printf(failures ? "\n%d CHECK(S) FAILED\n" : "\nALL CHECKS PASSED\n", failures);
	return failures ? 1 : 0;
}
