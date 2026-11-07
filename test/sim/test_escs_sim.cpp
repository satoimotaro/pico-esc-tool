// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// The escs simulation test: src/apps/esc_session.h and fourway.h, compiled unchanged on the host
// against the board stand-ins and the ESC models in sim.h, through the scenarios the bench found
// (hot-plug, a pull under throttle, Bluejay's FORCE_EDT_ARM restarts, a 3D swap, the passthrough
// with esc-configurator's 1 s reply timeout). One invariant holds on EVERY DShot frame: a non-zero
// throttle goes only to a RUNNING pin whose identity is the ESC's (3D and bidir), in its protocol,
// within the deadman of a drive command, and exactly the value that command asked for as the ESC
// reads it (so a one-way value sent to a 3D ESC, or 3D thrust to a one-way one, is caught).
//     make -C test/sim        (or see the Makefile for the g++ line)
#include "sim.h"
#include "esc_session.h"
#include "fourway.h"
#include <vector>
#include <cstdlib>
#include <cstdarg>
#include <string>

static int failures = 0;
static std::vector<std::string> notes;
static void note(const char* f, ...) __attribute__((format(printf, 1, 2)));
static void note(const char* f, ...) { char b[300]; va_list a; va_start(a, f); vsnprintf(b, sizeof b, f, a); va_end(a); notes.push_back(b); }
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); failures++; } \
	else printf("ok: %s\n", msg); } while (0)

// ---- the invariant ---------------------------------------------------------------------------------
static int lastDrive[escs::MAX] = { 0 };
static uint32_t violations = 0, throttleFrames = 0;
static double lastThrottleAt[32] = { 0 };
static char firstViolation[200] = "";
static int idxOf(uint8_t pin) { for (uint8_t i = 0; i < escs::COUNT; i++) if (escs::PINS[i] == pin) return i; return -1; }
static void violate(const char* what, uint8_t pin, uint16_t wire) {
	if (!violations++) snprintf(firstViolation, sizeof firstViolation, "%s (pin %u, wire %u, t=%.3f s)", what, pin, wire, sim::seconds());
}
static void frameHook(uint8_t gp, bool bidir, bool command, uint16_t wire) {
	if (command || !wire) return;
	throttleFrames++; lastThrottleAt[gp & 31] = sim::seconds();
	int i = idxOf(gp);
	if (i < 0) return violate("throttle on a pin that is not an ESC pin", gp, wire);
	using namespace escs::detail;
	sim::Esc& e = sim::esc(gp);
	if (pin[i].st != lc::State::RUNNING) return violate("throttle on a pin that is not RUNNING", gp, wire);
	if (!pin[i].id.known) return violate("throttle with no identity", gp, wire);
	if (!e.present) return;   // pulled: the next frames find out (LOST); the identity was right
	bool escBidir = e.spec.fw != sim::Fw::BLHELI_S;
	if (pin[i].id.rev != e.spec.rev) return violate("throttle on an identity whose 3D is not the ESC's", gp, wire);
	if (pin[i].id.bidir != escBidir || bidir != escBidir) return violate("throttle in a protocol that is not the ESC's", gp, wire);
	if (millis() - drvLast[i] > escs::SPIN_DEADMAN_MS + 5) return violate("throttle past the deadman", gp, wire);
	// exactly the value asked for, as this ESC (3D or not, whatever the pin believes) reads the wire
	int v = lastDrive[i];
	uint16_t want = e.spec.rev ? (v ? lc::thrust3d((int16_t)(v > 1000 ? 1000 : v < -1000 ? -1000 : v)) + 47 : 0)
	                           : (v > 0 ? (v > 2000 ? 2000 : v) + 47 : 0);
	if (wire != want) violate(e.spec.rev && (wire < 1048) != (v < 0) ? "3D direction not the one asked for"
	                                                                 : "throttle not the value asked for", gp, wire);
}

// ---- driving it ------------------------------------------------------------------------------------
static void loop1() { escs::core1Poll(); }
static bool trace = getenv("SIM_TRACE") != nullptr;
static void events() {                          // the lifecycle events, as EscTool prints them
	uint8_t i; lc::Ev e; lc::State st;
	while (escs::pollEvent(i, e, st))
		if (trace) printf("  %8.3f s  esc %u: %s (%s) -> %s\n", sim::seconds(), i, lc::evName(e), lc::stateName(st), escs::stateName(i));
}
static void poll() { escs::spinPoll(); events(); delayMicroseconds(100); }
static void runFor(uint32_t ms) {               // the core0 loop: spinPoll, as EscTool::poll does
	uint32_t t0 = millis();
	while (millis() - t0 < ms) poll();
}
// Drive i at v every 50 ms for ms (the page sends a frame every 50 ms), tracking what was asked.
static lc::Refuse driveFor(uint8_t i, int v, uint32_t ms) {
	uint32_t t0 = millis(); lc::Refuse r = lc::Refuse::OK; uint32_t next = 0;
	while (millis() - t0 < ms) {
		if (millis() >= next) { r = escs::drive(i, v); if (r == lc::Refuse::OK) lastDrive[i] = v; next = millis() + 50; }
		poll();
	}
	return r;
}
static bool waitState(uint8_t i, lc::State s, uint32_t maxMs) {
	uint32_t t0 = millis();
	while (millis() - t0 < maxMs) { if (escs::state(i) == s) return true; poll(); }
	return escs::state(i) == s;
}
static bool armAndWait(uint8_t i, uint32_t maxMs = 12000) {
	if (!escs::spinArm(i)) return false;
	uint32_t t0 = millis();
	while (millis() - t0 < maxMs) { if (escs::spinArmed(i)) return true; poll(); }
	return false;
}

// ---- esc-configurator, as its source does it (FourWay.js, QueueProcessor.js) ------------------------
static FourWay fw;
static uint16_t crc16(const uint8_t* b, size_t n) {
	uint16_t c = 0;
	for (size_t k = 0; k < n; k++) { c ^= (uint16_t)b[k] << 8; for (int j = 0; j < 8; j++) c = (c & 0x8000) ? (c << 1) ^ 0x1021 : c << 1; }
	return c;
}
struct Reply { bool got = false; uint8_t cmd = 0, ack = 0; std::vector<uint8_t> data; double ms = 0; };
static uint32_t lateReplies = 0;
// Send one frame; the reply is what console::out() got while the firmware handled it (it answers
// inside feed()), its latency the sim time that took.
static Reply send4(uint8_t cmd, uint16_t addr, const std::vector<uint8_t>& params) {
	std::vector<uint8_t> f = { 0x2F, cmd, (uint8_t)(addr >> 8), (uint8_t)addr, (uint8_t)params.size() };
	f.insert(f.end(), params.begin(), params.end());
	uint16_t c = crc16(f.data(), f.size()); f.push_back(c >> 8); f.push_back(c & 0xFF);
	Serial.out.clear();
	double t0 = sim::seconds();
	for (uint8_t b : f) fw.feed(b, false);
	Reply r; r.ms = (sim::seconds() - t0) * 1000;
	const std::string& o = Serial.out;
	if (o.size() >= 9 && (uint8_t)o[0] == 0x2E) {
		uint16_t n = (uint8_t)o[4] ? (uint8_t)o[4] : 256;
		if (o.size() >= 8u + n) { r.got = true; r.cmd = o[1]; r.ack = o[5 + n]; r.data.assign(o.begin() + 5, o.begin() + 5 + n); }
	}
	if (r.ms > 1000) lateReplies++;
	return r;
}
// Its retry: up to `tries`, 250 ms * k apart, any NACK or timeout tried again.
static Reply cmd4(uint8_t cmd, uint16_t addr, const std::vector<uint8_t>& p, int tries = 10, int* used = nullptr) {
	Reply r;
	for (int k = 1; k <= tries; k++) {
		r = send4(cmd, addr, p);
		if (used) *used = k;
		if (r.got && r.ack == 0 && r.ms <= 1000) return r;
		runFor(250 * k);
	}
	return r;
}
static void mspPassthrough() {
	const uint8_t f[] = { '$', 'M', '<', 0, 245, 245 };
	Serial.out.clear();
	for (uint8_t b : f) fw.feed(b, b == '$');
}

int main() {
	sim::onFrame(frameHook);
	sim::startCore1(loop1);
	escs::setHold(true);
	// ESC_BOARD_PANEL: pins 16..19 = ESC 0..3. A Bluejay with FORCE_EDT_ARM on 18, an AM32 set 3D on 17.
	sim::plug(18, { sim::Fw::BLUEJAY, false, true });
	sim::plug(17, { sim::Fw::AM32, true, false });

	{   // boot: the hold finds both, the async reads learn them; the empty pins stay EMPTY
		runFor(12000);
		lc::Identity a = escs::identity(1), b = escs::identity(2);
		CHECK(escs::state(1) == lc::State::READY && a.am32 && a.rev && a.bidir, "boot: the AM32 is READY, 3D, bidir");
		CHECK(escs::state(2) == lc::State::READY && !b.am32 && !b.rev && b.bidir, "boot: the Bluejay is READY, one-way, bidir (layout revision 208)");
		CHECK(escs::state(0) == lc::State::EMPTY && escs::state(3) == lc::State::EMPTY, "boot: the empty pins stay EMPTY");
	}
	{   // Bluejay with FORCE_EDT_ARM: arm, run, stop, start again after 0.2..2 s gaps (ae4917a)
		sim::Esc& bj = sim::esc(18);
		bool armed = armAndWait(2);
		CHECK(armed && bj.edt, "Bluejay: armed, and EDT is on before the first start");
		int started = 0; const uint32_t gaps[] = { 200, 400, 600, 800, 1000, 1200, 2000 };
		for (uint32_t g : gaps) {
			driveFor(2, 300, 1500);
			started += bj.running && bj.rpm > 500;
			if (trace) printf("  gap %u: running %d rpm %.0f refused %u edt %d state %s\n", g, bj.running, bj.rpm, bj.refusedStarts, bj.edt, escs::stateName(2));
			driveFor(2, 0, g);
		}
		CHECK(started == 7 && bj.refusedStarts == 0, "Bluejay: 7 of 7 starts after a stop (EDT is sent again after every run)");
		escs::spinStop(2); runFor(500);
	}
	{   // the AM32 in 3D: both directions, and a one-way value never reverses it
		sim::Esc& am = sim::esc(17);
		CHECK(armAndWait(1), "AM32: armed");
		driveFor(1, -400, 1500); float rv = am.rpm;
		driveFor(1, 400, 1500);  float fw_ = am.rpm;
		driveFor(1, 0, 800);
		CHECK(rv < -500 && fw_ > 500, "AM32 3D: drive -400 runs it in reverse, +400 forward");
		escs::spinStop(1); runFor(500);
	}
	{   // a one-way ESC refuses reverse; the rpm loop's path gives it 0
		CHECK(armAndWait(2), "Bluejay: armed again");
		CHECK(escs::drive(2, -100) == lc::Refuse::REVERSE_ONEWAY, "one-way: reverse refused, not clamped");
		escs::spinStop(2); runFor(300);
	}
	{   // hot-plug at zero: pulled -> LOST/EMPTY, back -> READY with a fresh read
		sim::unplug(18);
		runFor(1500);
		lc::State s = escs::state(2);
		CHECK(s == lc::State::LOST || s == lc::State::EMPTY || s == lc::State::PENDING, "hot-plug: a pulled Bluejay leaves READY within 1.5 s");
		sim::plug(18, { sim::Fw::BLUEJAY, false, true });
		CHECK(waitState(2, lc::State::READY, 12000), "hot-plug: plugged back, READY again");
	}
	{   // a pull under throttle: TOOL policy stops it and forgets it; no throttle goes out after
		CHECK(armAndWait(2), "pull: armed");
		driveFor(2, 300, 1000);
		sim::unplug(18);
		double pulled = sim::seconds();
		driveFor(2, 300, 2500);
		CHECK(escs::state(2) != lc::State::RUNNING, "pull under throttle: no longer RUNNING");
		CHECK(lastThrottleAt[18] - pulled < 0.6, "pull under throttle: no throttle 0.6 s after the pull (stall watch's silence, 500 ms)");
		sim::plug(18, { sim::Fw::BLUEJAY, false, true });
		CHECK(waitState(2, lc::State::READY, 12000), "pull: plugged back, READY again");
	}
	{   // a swap at zero: a 3D Bluejay where the one-way one was -> read again, 3D, and it may reverse
		runFor(3000);   // a person takes longer than its first answer after the read (see the note at the end)
		sim::unplug(18);
		runFor(1000);
		sim::plug(18, { sim::Fw::BLUEJAY, true, false });
		CHECK(waitState(2, lc::State::READY, 12000) && escs::identity(2).rev, "swap: the new ESC is read, and it is 3D");
		CHECK(armAndWait(2), "swap: armed");
		driveFor(2, -300, 1200);
		CHECK(sim::esc(18).rpm < -500, "swap: reverse on the 3D one");
		driveFor(2, 0, 500); escs::spinStop(2); runFor(500);
	}
	{   // the passthrough, with esc-configurator's semantics: every reply within 1 s
		lateReplies = 0;
		mspPassthrough();
		CHECK(escs::passthroughActive(), "passthrough: started");
		int found = 0, tries = 0; bool okRead = true;
		for (uint8_t ch = 0; ch < 4; ch++) {
			Reply r = cmd4(0x37, 0, { ch }, 5, &tries);
			if (!(r.got && r.ack == 0)) continue;
			found++;
			uint16_t addr = r.data[3] == 4 ? 0x7C00 : 0x1A00;
			Reply rd = cmd4(0x3A, addr, { 16 });
			okRead = okRead && rd.got && rd.ack == 0 && rd.cmd == 0x3A && rd.data.size() == 16;
		}
		CHECK(found == 2 && okRead, "passthrough: both ESCs selected and read, the empty pins not");
		// DeviceReset, then InitFlash again (after a flash): primed again, polled across pending replies
		Reply rs = cmd4(0x35, 0, { 2 });
		Reply again = cmd4(0x37, 0, { 2 }, 10, &tries);
		if (trace) printf("  reset: got %d ack %u (%.0f ms); again: got %d ack %u tries %d (%.0f ms) mode %d\n", rs.got, rs.ack, rs.ms, again.got, again.ack, tries, again.ms, (int)sim::esc(18).mode);
		CHECK(rs.got && rs.ack == 0 && again.got && again.ack == 0, "passthrough: re-selected after DeviceReset");
		CHECK(lateReplies == 0, "passthrough: no 4-way reply took more than 1 s (esc-configurator's timeout)");
		send4(0x34, 0, { 0 });   // InterfaceExit
		runFor(12000);
		CHECK(escs::state(1) == lc::State::READY && escs::state(2) == lc::State::READY, "passthrough end: both READY again");
	}
	{   // stock BLHeli_S on the empty pin 16: no telemetry, so the hold never sees it; `arm` reads it.
		// Whether it enters its bootloader after the bidir prime is not measured (Spec::bidirPrimes):
		// either way nothing may spin it wrongly, and with it, it is read and runs on normal DShot.
		sim::plug(16, { sim::Fw::BLHELI_S, false, false, false, true });
		runFor(3000);
		CHECK(escs::state(0) == lc::State::EMPTY, "BLHeli_S: not seen by the hold (no telemetry)");
		CHECK(armAndWait(0, 15000), "BLHeli_S (bidir prime counts): `arm` reads it and arms it");
		CHECK(!escs::identity(0).bidir && sim::esc(16).armed, "BLHeli_S: normal DShot, and the ESC armed on it");
		driveFor(0, 400, 1000);
		CHECK(sim::esc(16).rpm > 500, "BLHeli_S: it runs");
		driveFor(0, 0, 300); escs::spinStop(0); runFor(300);
		// (a pull of a non-bidir ESC is not seen: pin 16 keeps its identity, so the other variant
		// goes on the never-used pin 19)
		sim::plug(19, { sim::Fw::BLHELI_S, false, false, false, false });
		runFor(1000);
		bool armed = armAndWait(3, 15000);
		note("BLHeli_S whose bootloader entry ignores the bidir prime: `arm` %s (state %s, %u bootloader entries)",
		     armed ? "reads and arms it" : "finds nothing there", escs::stateName(3), sim::esc(19).blEntries);
		CHECK(!sim::esc(19).running, "BLHeli_S (bidir prime does not count): it does not run");
		escs::spinStop(3); sim::unplug(19); runFor(2000);
	}
	{   // known limits, reported rather than checked
		// (a) a swap inside the first answer after a read: the old identity is kept
		sim::unplug(18); runFor(2000);
		sim::plug(18, { sim::Fw::BLUEJAY, false, false });
		waitState(2, lc::State::READY, 12000);
		sim::unplug(18); runFor(100);
		sim::plug(18, { sim::Fw::BLUEJAY, true, false });
		runFor(4000);
		note("swap within 0.1 s of a read (before its first answer): identity %s, the ESC is 3D",
		     escs::identity(2).rev ? "3D (caught)" : "one-way (missed: forward would run it in reverse)");
		sim::unplug(18); runFor(2000);
		sim::plug(18, { sim::Fw::BLUEJAY, false, true });
		waitState(2, lc::State::READY, 12000);
		// (b) DeviceReset then InitFlash on a Bluejay that plays its startup tune
		sim::unplug(18); runFor(2000);
		sim::plug(18, { sim::Fw::BLUEJAY, false, true, true });
		waitState(2, lc::State::READY, 15000);
		runFor(3000);   // its tune is over
		mspPassthrough();
		Reply sel = cmd4(0x37, 0, { 2 }, 5);
		cmd4(0x35, 0, { 2 });
		int tries = 0; Reply again = cmd4(0x37, 0, { 2 }, 10, &tries);
		note("startup tune on: selected %s; after DeviceReset %s (%d tries)", sel.got && !sel.ack ? "yes" : "no",
		     again.got && !again.ack ? "selected again" : "not found again", tries);
		send4(0x34, 0, { 0 }); runFor(3000);
	}

	for (auto& n : notes) printf("note: %s\n", n.c_str());
	printf("throttle frames checked: %u\n", throttleFrames);
	CHECK(violations == 0, violations ? firstViolation : "invariant held on every frame");
	printf(failures ? "\n%d CHECK(S) FAILED\n" : "\nALL CHECKS PASSED\n", failures);
	return failures ? 1 : 0;
}
