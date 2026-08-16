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
// VERSIONING: a Record layout change bumps CFG_VERSION. v1 -> v2 added the measured-curve points and
// their regime tags; v2 -> v3 added the ESC's `sine_cross_up`.
//
// v3 ADDED A MIGRATION PATH. Until then a bump DISCARDED every stored config, so it cost everyone
// their gains and their `sense vcal` scale — which pushed changes toward hacks like overloading an
// existing field. `upgrade()` below reads an older layout and fills the new fields with the values
// that reproduce the OLD behaviour, so a bump is now cheap and honest. When you add a field:
//   1. snapshot the current layout as RecordV<n> (byte-exact, never edited again),
//   2. append the field to Record and bump CFG_VERSION,
//   3. extend upgrade() with the default that preserves pre-existing behaviour.
// Anything still unreadable (bad magic/CRC/size, or a version with no upgrade path) falls back to the
// compile-time defaults rather than half-applying — a silently mis-applied config is worse than none.
#pragma once
#include <Arduino.h>
#include <string.h>    // memcpy (the version migration)
#include <stddef.h>    // offsetof (the layout static_assert)
#include <LittleFS.h>
#include "thruster.h"

namespace settings {

static const char*    CFG_DIR     = "/cfg";
static const char*    CFG_PATH    = "/cfg/settings.bin";
static const uint32_t CFG_MAGIC   = 0x54534550;   // 'PEST'
static const uint16_t CFG_VERSION = 3;   // v3 added the ESC's sine_cross_up (+ the migration path)
static const int      CFG_MAX_CURVE = 24;         // == Thruster::ffbuf_ capacity

// FROZEN v2 layout. Kept byte-exact so `upgrade()` can still read blobs written by older builds.
// Never edit this — add a RecordV3 snapshot instead when v4 comes.
struct RecordV2 {
	uint8_t  hasMotor;
	uint8_t  curveN;
	uint16_t polePairs;
	float    kv, volts;
	float    kp, ki, kd, dTau, trimMax, blendSecs, slewRpmS, stopBelowRpm;
	float    dob, dobTau, dobMax, dobSettleSecs;
	float    vbattScale;
	float    crossUpErpm, crossDnErpm;
	float    curveThrust[CFG_MAX_CURVE];
	float    curveRpm[CFG_MAX_CURVE];
	uint8_t  curveRegime[CFG_MAX_CURVE];
};

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
	// --- v3 ---
	// The ESC's `sine_cross_up`. It shifts the 6-step duty map by a constant, so the parametric FF and
	// the soft-sensor that inverts it are both wrong by that shift if it is not the one the shape
	// constants were fitted at. Persisted because a power cycle silently reverting it would put the FF
	// back to a state that LOOKS fine (the PI hides it) while biasing every duty-derived estimate.
	float    crossUp;
};

// Fill a v3 Record from a v2 one. Defaults must reproduce the OLD behaviour exactly, so upgrading a
// stored config never changes how the bench runs — only what it is able to express.
inline void upgrade(const RecordV2& o, Record& r) {
	memcpy(&r, &o, sizeof(o));                 // the v3 prefix IS the v2 layout, field for field
	r.crossUp = vel::DUTY6_CROSSUP_REF;        // pre-v3 builds had no term => they behaved as the ref
}
// The memcpy above is only valid while Record's first fields are byte-identical to RecordV2. Reordering
// or retyping any v2 field would silently shift every value in a migrated config — loud at compile time
// instead. New fields must be APPENDED, after crossUp.
// Size alone is NOT enough: swapping kv/volts or retyping polePairs keeps sizeof identical while the
// memcpy silently reinterprets every migrated field (a v2 blob's KV landing in volts installs an FF
// curve wrong by ~30x, and it would look perfectly valid). Anchor a few offsets spread across the
// struct so a reorder is a compile error rather than a corrupted calibration.
static_assert(sizeof(RecordV2) == offsetof(Record, crossUp)
              && offsetof(Record, kv)          == offsetof(RecordV2, kv)
              && offsetof(Record, volts)       == offsetof(RecordV2, volts)
              && offsetof(Record, dob)         == offsetof(RecordV2, dob)
              && offsetof(Record, vbattScale)  == offsetof(RecordV2, vbattScale)
              && offsetof(Record, curveThrust) == offsetof(RecordV2, curveThrust)
              && offsetof(Record, curveRegime) == offsetof(RecordV2, curveRegime),
              "Record's v2 prefix diverged from RecordV2 — the v2 migration would corrupt configs");

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
	r.crossUp    = t.motorCrossUp();
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
	// cross_up FIRST, and independently of hasMotor: Thruster ships a live default MotorModel, and the
	// soft-sensor (estimateV/estimateLoad/vbatt) inverts its duty map whether or not `motor` ever ran.
	// Gating this behind hasMotor would silently drop it in exactly the workflow `curvepush` now creates
	// — sync_crossup() with the compile-time profile in charge — so the blob would hold the right value
	// while the running model reverted to the reference and re-introduced the bias on every boot.
	if (r.crossUp >= 1.0f) t.setCrossUp(r.crossUp);
	// Report a failure for the same reason the stored-curve rejection below does: setMotor() correctly
	// keeps the current curve, but silently falling back to the compile-time profile after a reset looks
	// identical to "the save never happened". It is worse here, because motorSet_ is left FALSE while
	// kv/pp/v stay applied — so the next capture() writes hasMotor=0 alongside a populated kv/volts/
	// polePairs, i.e. the following `cfg save` demotes a configured motor to never-configured.
	if (r.hasMotor && r.kv > 0.0f && r.polePairs >= 1 && r.volts > 0.0f) {
		// crossUp 0 = never stored (or a migrated v2 blob that predates it) -> setMotor keeps its default.
		if (!t.setMotor(r.kv, (int)r.polePairs, r.volts, r.crossUp))
			// Re-running `motor` is the ONLY remedy named here on purpose. Pushing a measured curve
			// fixes the feed-forward but NOT the persisted motor identity: setCurve() never touches
			// motorSet_ (only setMotor() does), so capture() would still write hasMotor=0 and the next
			// boot's apply() would skip setMotor entirely — kv/pp/v never restored and the soft-sensor
			// duty map silently back on the MotorModel defaults, under a measured curve that loads fine.
			Serial.printf("# cfg: no parametric curve at cross_up %.0f (kv=%.0f pp=%u) — keeping the "
			              "current curve; `motor` is NOT marked configured, so re-run `motor` with a "
			              "cross_up in %d..%d before the next `cfg save` (a measured curve fixes the FF "
			              "but not the persisted motor identity)\n",
			              (double)r.crossUp, (double)r.kv, (unsigned)r.polePairs,
			              vel::minHandoffCrossUp(), vel::maxEmittableCrossUp(700.0f));
	}
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
	// Record size depends on the STORED version — that is what makes the migration work, and it is why
	// the size is derived here rather than assumed to be sizeof(Record).
	size_t recSize = 0;
	if      (h.version == CFG_VERSION) recSize = sizeof(Record);
	else if (h.version == 2)           recSize = sizeof(RecordV2);
	if (recSize == 0 || h.count == 0 || h.count > n) { f.close(); return -1; }

	// Raw byte buffer sized for the LARGEST layout, so an older (smaller) blob always fits.
	static uint8_t raw[8 * sizeof(Record)];
	size_t want = recSize * h.count;
	if (h.count > (uint16_t)(sizeof(raw) / sizeof(Record))
	    || f.read(raw, want) != (int)want
	    || crc32(raw, want) != h.crc) {
		f.close(); return -1;
	}
	f.close();

	// One record at a time through a stack temp: no in-place widening of the buffer (the v3 records are
	// larger than the v2 ones, so converting in place would overwrite records not yet read).
	for (uint16_t i = 0; i < h.count; i++) {
		Record r;
		if (h.version == CFG_VERSION) {
			memcpy(&r, raw + (size_t)i * recSize, sizeof(r));
		} else {
			RecordV2 o;
			memcpy(&o, raw + (size_t)i * recSize, sizeof(o));
			upgrade(o, r);
		}
		apply(*th[i], r);
	}
	if (h.version != CFG_VERSION)
		Serial.printf("# cfg: migrated a v%u config to v%u — `cfg save` to store it in the new format\n",
		              (unsigned)h.version, (unsigned)CFG_VERSION);
	return (int)h.count;
}

inline bool exists() { return LittleFS.exists(CFG_PATH); }
inline bool clear()  { return LittleFS.remove(CFG_PATH); }

// --- THE AZIMUTH ZERO, in its own blob ---------------------------------------------------------
// The heading reference is a property of the RIG (where the magnet sits relative to the hull), not
// of an ESC, so it does not belong in the per-Thruster Record: that would store one copy per ESC and
// make "which one is authoritative" a question nobody should have to answer. Its own file also means
// a heading calibration never bumps CFG_VERSION, i.e. never costs the operator their gains.
//
// Stored as an AS5600 RAW_ANGLE count, NOT as the tracker's accumulator: `accum_` starts at 0 at
// boot and is only meaningful within one session, while RAW_ANGLE is absolute within a turn and is
// therefore the only part of the encoder that survives a power cycle.
static const char*    AZ_PATH    = "/cfg/azimuth.bin";
static const uint32_t AZ_MAGIC   = 0x545A4150;   // 'PAZT'
static const uint16_t AZ_VERSION = 1;
static const int32_t  AZ_COUNTS  = 4096;         // == Azimuth::COUNTS_PER_REV

struct AzZero {
	uint32_t magic;
	uint16_t version;
	uint16_t pad;
	int32_t  zeroRaw;      // the RAW_ANGLE count that reads as 0 deg
	uint32_t crc;          // over zeroRaw
};

inline bool azSaveZero(int32_t zeroRaw) {
	if (zeroRaw < 0 || zeroRaw >= AZ_COUNTS) return false;
	LittleFS.mkdir(CFG_DIR);
	AzZero z;
	memset(&z, 0, sizeof(z));
	z.magic = AZ_MAGIC; z.version = AZ_VERSION; z.zeroRaw = zeroRaw;
	z.crc = crc32((const uint8_t*)&z.zeroRaw, sizeof(z.zeroRaw));
	File f = LittleFS.open(AZ_PATH, "w");
	if (!f) return false;
	bool ok = f.write((const uint8_t*)&z, sizeof(z)) == sizeof(z);
	f.close();
	return ok;
}

// -1 means "no absolute reference stored", and every caller must treat that as REFUSE — never as
// "the zero is at raw 0", which would silently point the pod's idea of forward at the magnet's
// arbitrary mounting angle.
inline int32_t azLoadZero() {
	File f = LittleFS.open(AZ_PATH, "r");
	if (!f) return -1;
	AzZero z;
	bool ok = f.read((uint8_t*)&z, sizeof(z)) == (int)sizeof(z);
	f.close();
	if (!ok || z.magic != AZ_MAGIC || z.version != AZ_VERSION) return -1;
	if (crc32((const uint8_t*)&z.zeroRaw, sizeof(z.zeroRaw)) != z.crc) return -1;
	if (z.zeroRaw < 0 || z.zeroRaw >= AZ_COUNTS) return -1;
	return z.zeroRaw;
}

inline bool azZeroStored() { return LittleFS.exists(AZ_PATH); }
inline bool azClearZero()  { return LittleFS.remove(AZ_PATH); }

}  // namespace settings
