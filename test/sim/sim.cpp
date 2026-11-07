// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// sim — the clock, the two cores, the lines, the ESC models and the simulated bootloader (sim.h).
#include "sim.h"
#include <Arduino.h>
#include <PIO_DShot.h>
#include <hardware/pwm.h>
#include <pico/time.h>
#include "blheli_bl.h"
#include <ucontext.h>
#include <cstdlib>
#include <vector>

CaptureStream Serial;
static pio_sim pioS[2] = { {0}, {1} };
PIO pio0 = &pioS[0], pio1 = &pioS[1];

namespace sim {

// ---- clock and cores ------------------------------------------------------------------------------
static uint64_t T = 0;                 // now (us)
static uint64_t ready[2] = { 0, 0 };   // when each core may run again
static int      cur = 0;
static bool     core1On = false, inTimer = false;
static ucontext_t ctx[2];
static void (*core1Loop)() = nullptr;
static std::vector<repeating_timer*> timers;
static std::vector<uint64_t> timerDue;
static void stepModels();

uint64_t nowUs() { return T; }
double seconds() { return T / 1e6; }

static void runTimers() {
	if (inTimer) return;
	inTimer = true;
	for (size_t k = 0; k < timers.size(); k++)
		while (timers[k] && timerDue[k] <= T) { timerDue[k] += timers[k]->ms * 1000ull; timers[k]->cb(timers[k]); }
	inTimer = false;
}
void advance(uint32_t us) {
	if (inTimer) return;               // an IRQ handler does not wait
	ready[cur] = T + us;
	int next = cur;
	if (core1On) next = ready[1] < ready[0] ? 1 : 0;
	if (next != cur) {
		int was = cur; cur = next;
		T = ready[next];
		runTimers(); stepModels();
		swapcontext(&ctx[was], &ctx[next]);   // back here when `was` is due again
		T = ready[was];
	} else {
		T = ready[cur];
		runTimers(); stepModels();
	}
}
static void core1Main() { for (;;) { core1Loop(); advance(50); } }
void startCore1(void (*loop)()) {
	static char stack[1 << 20];
	core1Loop = loop;
	getcontext(&ctx[1]);
	ctx[1].uc_stack.ss_sp = stack; ctx[1].uc_stack.ss_size = sizeof stack; ctx[1].uc_link = nullptr;
	makecontext(&ctx[1], core1Main, 0);
	ready[1] = T; core1On = true;
}
bool addTimer(repeating_timer* t) { timers.push_back(t); timerDue.push_back(T + t->ms * 1000ull); return true; }
void cancelTimer(repeating_timer* t) { for (auto& x : timers) if (x == t) x = nullptr; }

// ---- lines ---------------------------------------------------------------------------------------
struct Line { int mode = INPUT; int level = LOW; bool driver = false; uint16_t pwm = 0; uint64_t highUs = 0; };
static Line line[32];
static Esc  escs_[32];
static FrameHook hook = nullptr;
void onFrame(FrameHook h) { hook = h; }
Esc& esc(uint8_t pin) { return escs_[pin & 31]; }
uint16_t pwmWidth(uint8_t pin) { return line[pin & 31].pwm; }

static bool held(uint8_t p) { const Line& l = line[p]; return !l.driver && !l.pwm && l.mode == OUTPUT && l.level == HIGH; }
void gpioMode(uint8_t p, int m) {
	Line& l = line[p & 31]; bool was = held(p & 31); l.mode = m; if (m == OUTPUT) l.pwm = 0;
	if (!was && held(p & 31)) l.highUs = T;
}
void gpioWrite(uint8_t p, int v) { Line& l = line[p & 31]; bool was = held(p & 31); l.level = v; if (!was && held(p & 31)) l.highUs = T; }
int  gpioRead(uint8_t p) { return line[p & 31].level; }
void gpioFunc(uint8_t p, int fn) { if (fn == GPIO_FUNC_PWM) line[p & 31].mode = -1; else line[p & 31].pwm = 0; }
void pwmLevel(uint8_t p, uint16_t us) { line[p & 31].pwm = us; }

// PIO SMs: 4 on pio0, 3 free on pio1; one program per pio (the bidir one does not fit pio1).
static int smUsed[2] = { 0, 0 }; static int smKind[2] = { -1, -1 };
bool takeSm(PIO pio, bool bidir) {
	int k = pio->id, kind = bidir ? 1 : 0;
	if (bidir && k == 1) return false;
	if (smUsed[k] >= (k == 0 ? 4 : 3)) return false;
	if (smUsed[k] && smKind[k] != kind) return false;
	smUsed[k]++; smKind[k] = kind; return true;
}
void giveSm(PIO pio) { int k = pio->id; if (smUsed[k] > 0 && --smUsed[k] == 0) smKind[k] = -1; }
void driverGone(uint8_t p) {   // its RX FIFO goes with it
	Line& l = line[p & 31]; l.driver = false; l.mode = INPUT; l.level = LOW; escs_[p & 31].teleType = 0xFF;
}

// ---- ESC models ------------------------------------------------------------------------------------
static void fillMem(Esc& e) {
	memset(e.mem, 0xFF, sizeof e.mem); memset(e.name, 0, sizeof e.name);
	if (e.spec.fw == Fw::AM32) {
		const uint8_t head[] = { 0x01, 0x04, 0x0d, 0x02, 0x15, 0xa0, 0x01, 0x00, 0x0a, 0x64, 0x00, 0x64, 0x00, 0x00 };
		memcpy(e.mem, head, sizeof head);
		e.mem[18] = e.spec.rev ? 1 : 0;                       // bi_direction
		strcpy((char*)e.name, "SIM_AM32_F051");
		return;
	}
	bool bj = e.spec.fw == Fw::BLUEJAY;
	e.mem[0] = bj ? 0x00 : 16; e.mem[1] = bj ? 21 : 7; e.mem[2] = bj ? 208 : 33;
	e.mem[0x0B] = e.spec.rev ? 3 : 1;
	e.mem[0x0D] = 0x55; e.mem[0x0E] = 0xAA;                    // MULTI mode signature
	strcpy((char*)e.mem + 0x40, "#S_H_50#");
	strcpy((char*)e.mem + 0x50, "#BLHELI$EFM8B21#");
	strcpy((char*)e.mem + 0x60, bj ? "Bluejay (.0)" : "");
}
static uint32_t bootMs(const Spec& s) { return s.tune ? BOOT_TUNE_MS : s.fw == Fw::AM32 ? AM32_BOOT_MS : BOOT_MS; }
void plug(uint8_t p, const Spec& s) {
	Esc& e = escs_[p & 31]; e = Esc{}; e.present = true; e.spec = s; fillMem(e);
	if (held(p)) { e.mode = Mode::BL; e.blEntries++; }       // powered with the line high: its bootloader
	else { e.mode = Mode::BOOT; e.bootEndUs = T + bootMs(s) * 1000ull; }
}
void unplug(uint8_t p) { escs_[p & 31] = Esc{}; }
static void appStart(Esc& e) {
	e.mode = Mode::BOOT; e.armed = e.edt = e.running = e.stuck = e.hadFrames = false; e.rpm = 0;
	e.bootEndUs = T + bootMs(e.spec) * 1000ull;
}
static void toBl(Esc& e) { e.mode = Mode::BL; e.armed = e.edt = e.running = false; e.rpm = 0; e.blEntries++; }

static float targetRpm(const Esc& e, uint16_t wire) {
	if (!wire) return 0;
	if (!e.spec.rev) return (wire - 47) * 4.0f;                       // one-way 48..2047
	return wire >= 1048 ? (wire - 1047) * 4.0f : -(float)(wire - 47) * 4.0f;   // 3D bands
}
static uint64_t lastStep = 0;
static void stepModels() {
	float dt = (T - lastStep) / 1e6f; lastStep = T;
	for (int p = 0; p < 32; p++) {
		Esc& e = escs_[p];
		if (!e.present) continue;
		if (e.mode == Mode::BOOT && T >= e.bootEndUs) { e.mode = Mode::APP; e.zeroSinceUs = 0; }
		if (e.mode != Mode::APP) continue;
		// signal loss on a line held high: the bootloader
		if (held(p)) {
			uint64_t since = T - line[p].highUs;
			if (e.spec.fw == Fw::AM32) { if (since >= AM32_BL_MS * 1000ull) toBl(e); }
			else if (e.hadFrames && !e.stuck && since >= BL_QUIET_MS * 1000ull) toBl(e);
			continue;
		}
		// the motor: towards the throttle while running, coasting down after
		float tgt = e.running ? targetRpm(e, e.lastWire) : 0;
		// coasting is quick (the bench's A2212 restarts 0.2 s after a stop: Bluejay is back in
		// wait_for_start by then)
		float k = dt / (e.running ? 0.1f : 0.04f); if (k > 1) k = 1;
		e.rpm += (tgt - e.rpm) * k;
		if (!e.running && e.rpm != 0 && std::fabs(e.rpm) < 50) e.rpm = 0;
	}
}
void frame(uint8_t p, bool bidir, bool command, uint16_t v) {
	Line& l = line[p & 31]; l.driver = true;
	uint16_t wire = command ? v : (v ? (uint16_t)((v > 2000 ? 2000 : v) + 47) : 0);
	if (hook) hook(p, bidir, command, wire);
	Esc& e = escs_[p & 31];
	if (!e.present || e.mode != Mode::APP) return;
	bool am32 = e.spec.fw == Fw::AM32;
	if (bidir && e.spec.fw == Fw::BLHELI_S) { if (e.spec.bidirPrimes) e.hadFrames = true; return; }   // inverted frames
	e.frames++; e.lastFrameUs = T; e.hadFrames = true; e.stuck = false;
	if (command) {
		bool takes = am32 || (e.armed && !e.running && e.rpm == 0);
		if (v == 13 && takes) { if (++e.cmdRun >= 6) e.edt = true; } else e.cmdRun = 0;
	} else {
		e.cmdRun = 0; e.lastWire = wire;
		if (!e.armed) {
			if (wire) e.zeroSinceUs = 0;
			else if (!e.zeroSinceUs) e.zeroSinceUs = T;
			else if (T - e.zeroSinceUs >= ARM_ZERO_MS * 1000ull) e.armed = true;
		} else if (wire && !e.running) {
			if (e.spec.forceEdtArm && !e.edt) e.refusedStarts++;   // beeps, waits again
			else e.running = true;
		} else if (!wire && e.running) {
			e.running = false;
			if (!am32) e.edt = false;                              // Bluejay: cleared at the end of a run
		}
	}
	// telemetry back on a bidir frame: Bluejay from wait_for_start on, AM32 once booted
	if (bidir && (am32 || e.armed)) {
		e.telePackets++;
		if (e.edt && e.frames % 16 == 0) { e.teleType = (uint8_t)BidirDshotTelemetryType::TEMPERATURE; e.teleVal = 30; }
		else { e.teleType = (uint8_t)BidirDshotTelemetryType::ERPM; e.teleVal = (uint32_t)(std::fabs(e.rpm) * POLES / 2); }
	}
}
BidirDshotTelemetryType telemetry(uint8_t p, uint32_t* v) {
	Esc& e = escs_[p & 31];
	if (!e.present || e.teleType == 0xFF) return BidirDshotTelemetryType::NO_PACKET;
	*v = e.teleVal; auto t = (BidirDshotTelemetryType)e.teleType; e.teleType = 0xFF;
	return t;
}

}  // namespace sim

// ---- the bootloader, against the models ---------------------------------------------------------------
namespace blheli_bl {
static sim::Esc* at(uint8_t pin) { sim::Esc& e = sim::esc(pin); return e.present ? &e : nullptr; }
static uint16_t blockAddr(const sim::Esc& e) { return e.spec.fw == sim::Fw::AM32 ? 0x7C00 : 0x1A00; }

bool Bootloader::begin() { pinMode(cfg_.signalPin, OUTPUT); digitalWrite(cfg_.signalPin, HIGH); connected_ = false; return true; }
void Bootloader::end() { connected_ = false; }
void Bootloader::holdIdleHigh(uint32_t ms) { pinMode(cfg_.signalPin, OUTPUT); digitalWrite(cfg_.signalPin, HIGH); delay(ms); }
void Bootloader::sendBootInit() { uint8_t r[8]; connectRawProbe(r, 15); }
int Bootloader::connectRawProbe(uint8_t out[8], uint32_t perByteTimeoutMs) {
	begin();
	delayMicroseconds(9000);                                  // BootInit: 17 bytes at 19200 baud
	sim::Esc* e = at(cfg_.signalPin);
	if (e && e->mode == sim::Mode::APP && e->spec.fw != sim::Fw::AM32) e->stuck = true;   // too early
	if (!e || e->mode != sim::Mode::BL) { delay(perByteTimeoutMs); return 0; }
	delayMicroseconds(5000);
	bool am32 = e->spec.fw == sim::Fw::AM32;
	const uint8_t silabs[8] = { '4', '7', '1', 'c', 0xE8, 0xB2, 0x06, 0x04 };
	const uint8_t arm[8]    = { '4', '7', '1', 0x64, 0x1F, 0x06, 0x01, 0x30 };
	memcpy(out, am32 ? arm : silabs, 8);
	lastProbeAm32_ = am32;
	return 8;
}
bool Bootloader::adoptReply(const uint8_t info[8], bool am32) {
	connected_ = false; dev_ = DeviceInfo{};
	if (!(info[0] == '4' && info[1] == '7' && info[2] == '1')) return false;
	memcpy(dev_.bootInfo, info, 8);
	dev_.am32 = am32; dev_.signature[0] = info[4]; dev_.signature[1] = info[5];
	dev_.bootVersion = info[6]; dev_.bootPages = info[7];
	if (!dev_.signatureWord()) return false;
	dev_.valid = true; connected_ = true;
	return true;
}
bool Bootloader::connect() { uint8_t r[8]; return connectRawProbe(r, 40) == 8 && adoptReply(r); }
bool Bootloader::readDeviceInfo(DeviceInfo& out) { out = dev_; return dev_.valid; }
static bool inBl(uint8_t pin) { sim::Esc* e = at(pin); return e && e->mode == sim::Mode::BL; }
bool Bootloader::keepAlive() { delay(5); return connected_ && inBl(cfg_.signalPin); }
bool Bootloader::readFlash(uint16_t addr, uint8_t* buf, uint16_t len) {
	delayMicroseconds(10000 + len * 550u);
	sim::Esc* e = at(cfg_.signalPin);
	if (!connected_ || !e || e->mode != sim::Mode::BL) return false;
	uint16_t base = blockAddr(*e);
	for (uint16_t i = 0; i < len; i++) {
		uint32_t a = (uint32_t)addr + i;
		if (a >= base && a < base + 256u) buf[i] = e->mem[a - base];
		else if (e->spec.fw == sim::Fw::AM32 && a >= base - 32u && a < base) buf[i] = e->name[a - (base - 32u)];
		else buf[i] = 0xFF;
	}
	return true;
}
bool Bootloader::readEeprom(uint16_t addr, uint8_t* buf, uint16_t len) { return readFlash(addr, buf, len); }
bool Bootloader::erasePage(uint16_t addr) {
	delay(30);
	sim::Esc* e = at(cfg_.signalPin);
	if (!connected_ || !e || e->mode != sim::Mode::BL) return false;
	uint16_t base = blockAddr(*e);
	if (addr <= base && base < addr + 512u) memset(e->mem, 0xFF, sizeof e->mem);
	return true;
}
bool Bootloader::writeFlash(uint16_t addr, const uint8_t* buf, uint16_t len) {
	delayMicroseconds(10000 + len * 600u);
	sim::Esc* e = at(cfg_.signalPin);
	if (!connected_ || !e || e->mode != sim::Mode::BL) return false;
	uint16_t base = blockAddr(*e);
	for (uint16_t i = 0; i < len; i++) { uint32_t a = (uint32_t)addr + i; if (a >= base && a < base + 256u) e->mem[a - base] = buf[i]; }
	return true;
}
bool Bootloader::writeEeprom(uint16_t addr, const uint8_t* buf, uint16_t len) { return writeFlash(addr, buf, len); }
bool Bootloader::run() {
	delay(5);
	sim::Esc* e = at(cfg_.signalPin);
	connected_ = false;
	if (!e || e->mode != sim::Mode::BL) return false;
	sim::appStart(*e); return true;
}
void Bootloader::probeReplyActivity(uint32_t ms, uint32_t& edges, uint32_t& lows, uint32_t& total) {
	delay(ms); edges = lows = total = 0;
}
const char* signatureName(uint16_t) { return "sim"; }
McuType mcuTypeFor(uint16_t w) { return (w >> 8) == 0xE8 ? McuType::SILABS_EFM8 : McuType::ARM_BB51; }
}  // namespace blheli_bl
