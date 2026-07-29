// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// settings — persist the per-Thruster RUNTIME configuration to LittleFS so it survives a reset.
//
// Everything set over serial (`motor`, `gain`, `sense vcal`, and later `curve`) lived in RAM only: a
// calibration was "one-time" per power cycle, which is the same overclaim the host-side vbatt.py made.
// This module writes one small binary blob holding, per ESC: the parametric motor identity, the control
// gains, the DOB tunables, the vbatt calibration scale, and a slot for a MEASURED feed-forward curve.
//
// Deliberate design choices:
//   * EXPLICIT save (`cfg save`), never autosave — flash has finite erase cycles and the control loop
//     writes these fields continuously during tuning.
//   * magic + version + CRC32, and a size check: a blob from another build is IGNORED, not applied. A
//     corrupt or stale config silently reverting to compile-time defaults is much safer than half-
//     applying one to a motor.
//   * apply() refuses while ARMED — it can replace the live feed-forward curve.
//
// VERSIONING: a Record layout change bumps CFG_VERSION and DISCARDS every stored config — there is no
// migration path, so a bump costs everyone their gains and their `sense vcal` scale. v1 -> v2 added the
// measured-curve points and their regime tags (the upload path). Prefer using a reserved field over
// bumping; if you must bump, say so in the release note, because the symptom (silently back to
// compile-time defaults after a reset) is easy to mistake for a bug.
#pragma once
#include <Arduino.h>
#include <LittleFS.h>
#include "thruster.h"

namespace settings {

static const char*    CFG_DIR     = "/cfg";
static const char*    CFG_PATH    = "/cfg/settings.bin";
static const uint32_t CFG_MAGIC   = 0x54534550;   // 'PEST'
static const uint16_t CFG_VERSION = 2;   // v2 added the measured-curve points + their regime tags
static const int      CFG_MAX_CURVE = 24;         // == Thruster::ffbuf_ capacity

// One ESC's persisted state. Plain floats, no padding surprises worth worrying about: the CRC + the
// exact-size check reject anything written by a differently-laid-out build.
struct Record {
	uint8_t  hasMotor;      // 0 = `motor` never ran; the compile-time profile stays in charge
	uint8_t  curveN;        // 0 = no measured curve stored (the `curve` upload path fills this)
	uint16_t polePairs;
	float    kv, volts;
	float    kp, ki, kd, dTau, trimMax, blendSecs, slewRpmS, stopBelowRpm;
	float    dob, dobTau, dobMax, dobSettleSecs;
	float    vbattScale;
	float    crossUpErpm, crossDnErpm;
	float    curveThrust[CFG_MAX_CURVE];
	float    curveRpm[CFG_MAX_CURVE];
	uint8_t  curveRegime[CFG_MAX_CURVE];   // 1 = LINE (6-step), 0 = SINE
};

struct Header {
	uint32_t magic;
	uint16_t version;
	uint16_t count;         // number of Records following
	uint32_t crc;           // CRC32 over the Record array only
};

// Bitwise CRC32 (poly 0xEDB88320) — no 1 KiB table; this runs a handful of times per session.
inline uint32_t crc32(const uint8_t* d, size_t n) {
	uint32_t c = 0xFFFFFFFFu;
	for (size_t i = 0; i < n; i++) {
		c ^= d[i];
		for (int b = 0; b < 8; b++) c = (c >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(c & 1)));
	}
	return ~c;
}

// Snapshot one Thruster into a Record.
inline void capture(const Thruster& t, Record& r) {
	memset(&r, 0, sizeof(r));
	r.hasMotor  = t.motorConfigured() ? 1 : 0;
	// [#11-1] When a MEASURED curve is live, its pole count is the authoritative one: `curve commit <pp>`
	// sets the profile's pp without touching the parametric model, so storing mm_'s would restore the
	// curve under the wrong pp — mis-scaling the tele mech-rpm (poles_) AND the rpm*pp >= up_erpm regime
	// test that lineFloor(), i.e. the soft-sensor gate, is built on. A single field is right because a
	// motor has ONE pole count; a disagreement is an operator error, which `curve commit` warns about.
	r.polePairs = (uint16_t)(t.curveMeasured() ? t.curvePolePairs() : t.motorPolePairs());
	r.kv        = t.motorKv();
	r.volts     = t.motorVolts();
	r.kp = t.vc.kp; r.ki = t.vc.ki; r.kd = t.vc.kd; r.dTau = t.vc.d_tau;
	r.trimMax = t.vc.trim_max; r.blendSecs = t.vc.blend_secs;
	r.slewRpmS = t.vc.slew_rpm_s; r.stopBelowRpm = t.vc.stop_below_rpm;
	r.dob = t.vc.dob; r.dobTau = t.vc.dob_tau; r.dobMax = t.vc.dob_max;
	r.dobSettleSecs = t.vc.dob_settle_secs;
	r.vbattScale = t.vbattScale();
	// A MEASURED curve is stored point-by-point; the parametric one is not — it is regenerable from
	// kv/pp/v, so persisting it would just be a second, staler copy of the same three numbers.
	if (t.curveMeasured()) {
		int n = t.curveCount(); if (n > CFG_MAX_CURVE) n = CFG_MAX_CURVE;
		for (int i = 0; i < n; i++) {
			r.curveThrust[i] = t.curvePoint(i).thrust;
			r.curveRpm[i]    = t.curvePoint(i).rpm;
			r.curveRegime[i] = (t.curveRegime(i) == vel::Regime::LINE) ? 1 : 0;
		}
		r.curveN      = (uint8_t)n;
		r.crossUpErpm = t.crossUpErpm();
		r.crossDnErpm = t.crossDnErpm();
	}
}

// Apply a Record to a Thruster. Motor identity FIRST (it rebuilds the feed-forward curve), then the
// gains, then the calibration — the reverse order would have setMotor() stomp nothing, but this keeps
// the dependency obvious. Values that fail a sanity check are skipped rather than poisoning the loop.
inline void apply(Thruster& t, const Record& r) {
	if (r.hasMotor && r.kv > 0.0f && r.polePairs >= 1 && r.volts > 0.0f)
		t.setMotor(r.kv, (int)r.polePairs, r.volts);
	// A stored MEASURED curve wins over the parametric one — that is the whole point of having uploaded
	// it. setCurve() re-validates, so a plausible-but-corrupt table is rejected rather than installed.
	if (r.curveN >= 2 && r.curveN <= CFG_MAX_CURVE && r.polePairs >= 1 && r.crossUpErpm > 0.0f) {
		// Locals, not statics: setCurve() copies into the Thruster's own buffer immediately, so keeping
		// these alive afterwards would just hold ~216 B of BSS for nothing.
		vel::CurvePoint pts[CFG_MAX_CURVE];
		vel::Regime     regs[CFG_MAX_CURVE];
		for (int i = 0; i < r.curveN; i++) {
			pts[i].thrust = r.curveThrust[i];
			pts[i].rpm    = r.curveRpm[i];
			regs[i]       = r.curveRegime[i] ? vel::Regime::LINE : vel::Regime::SINE;
		}
		// Report a rejection: setCurve() correctly keeps the existing curve, but silently falling back
		// to the parametric one after a reset looks identical to "the save never happened".
		if (!t.setCurve(pts, r.curveN, (int)r.polePairs, r.crossUpErpm, r.crossDnErpm, regs))
			Serial.printf("# cfg: stored curve REJECTED (%u points, pp=%u) — keeping the current curve\n",
			              (unsigned)r.curveN, (unsigned)r.polePairs);
	}
	t.vc.kp = r.kp; t.vc.ki = r.ki; t.vc.kd = r.kd; t.vc.d_tau = r.dTau;
	t.vc.trim_max = r.trimMax; t.vc.blend_secs = r.blendSecs;
	t.vc.slew_rpm_s = r.slewRpmS; t.vc.stop_below_rpm = r.stopBelowRpm;
	t.vc.dob = r.dob; t.vc.dob_tau = r.dobTau; t.vc.dob_max = r.dobMax;
	t.vc.dob_settle_secs = r.dobSettleSecs;
	t.setVbattScale(r.vbattScale);
}

// Write every Thruster's state. Returns false on any filesystem error (the caller reports it).
inline bool save(Thruster** th, uint8_t n) {
	if (n == 0) return false;
	LittleFS.mkdir(CFG_DIR);
	static Record recs[8];
	if (n > (uint8_t)(sizeof(recs) / sizeof(recs[0]))) return false;
	for (uint8_t i = 0; i < n; i++) capture(*th[i], recs[i]);

	Header h;
	h.magic = CFG_MAGIC; h.version = CFG_VERSION; h.count = n;
	h.crc = crc32((const uint8_t*)recs, sizeof(Record) * n);

	File f = LittleFS.open(CFG_PATH, "w");
	if (!f) return false;
	bool ok = f.write((const uint8_t*)&h, sizeof(h)) == sizeof(h)
	       && f.write((const uint8_t*)recs, sizeof(Record) * n) == sizeof(Record) * n;
	f.close();
	return ok;
}

// Read the blob and apply it. Returns the number of ESCs restored, or -1 when there is nothing valid
// (no file / wrong magic / wrong version / wrong size / bad CRC) — all of which mean "keep the
// compile-time defaults", never "apply what we could parse".
inline int load(Thruster** th, uint8_t n, uint16_t* storedVersion = nullptr) {
	if (storedVersion) *storedVersion = 0;
	File f = LittleFS.open(CFG_PATH, "r");
	if (!f) return -1;
	Header h;
	if (f.read((uint8_t*)&h, sizeof(h)) != sizeof(h) || h.magic != CFG_MAGIC) { f.close(); return -1; }
	// Report the stored version even when we reject it: "your config was written by an older build"
	// is a very different message from "your config is corrupt", and after a bump it is the one the
	// operator needs to hear before they wonder why their calibration vanished.
	if (storedVersion) *storedVersion = h.version;
	if (h.version != CFG_VERSION || h.count == 0 || h.count > n) { f.close(); return -1; }
	static Record recs[8];
	size_t want = sizeof(Record) * h.count;
	if (h.count > (uint16_t)(sizeof(recs) / sizeof(recs[0]))
	    || f.read((uint8_t*)recs, want) != (int)want
	    || crc32((const uint8_t*)recs, want) != h.crc) {
		f.close(); return -1;
	}
	f.close();
	for (uint16_t i = 0; i < h.count; i++) apply(*th[i], recs[i]);
	return (int)h.count;
}

inline bool exists() { return LittleFS.exists(CFG_PATH); }
inline bool clear()  { return LittleFS.remove(CFG_PATH); }

}  // namespace settings
