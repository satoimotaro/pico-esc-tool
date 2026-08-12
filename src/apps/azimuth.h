// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// azimuth — ON-DEVICE closed loop for the two-motor DIFFERENTIAL azimuth pod.
//
// WHY THIS IS ON THE PICO AND NOT THE HOST. Read the CORRECTION below first: "the ceiling is
// bandwidth, not tuning" was concluded from kp 8 / ki 1.5, which is at damping ratio ~0.73 and
// never should have been the reference. A LOWER-gain host config (kp 3, ki 0.5, kd 0.3, ~1.0) DID
// converge at 50 Hz. More rate is still worth having, but it was not what was missing.
//
// The host loop works (host/azimuth.py) and holds to a
// couple of degrees on a good run, but it is not repeatable: the same gains gave -2.2, -184.8,
// +0.9 and -74.2 deg across four runs. The host closes at
// 50 Hz over USB; raising the gain to stiffen the hold pushed the loop into a saturated limit
// cycle (s bang-banging between its rails while position swung +/-80 deg), because the motors need
// ~0.1-0.2 s to change speed and a 50 Hz loop has no phase margin left to spend.
//
// Here the same control law runs against the As5600Tracker's own ~1.25 kHz sample stream with no
// round trip at all — about 20x the rate, which buys back the phase margin that the gain needs.
// That is what makes a stiff hold (and prompt disturbance rejection) possible rather than a tuning
// wish: rejection bandwidth is loop bandwidth.
//
// The mixer is the same one the host uses, and its constants are MEASUREMENTS from this pod, not
// defaults — see host/azimuth.py. sign_b in particular is -1 because motor B is mounted mirrored;
// left at +1 the two motors are commanded in OPPOSITE directions (maximum azimuth) while the
// caller believes they are being carried together.
#pragma once
#include <Arduino.h>
#include "thruster.h"

class Azimuth {
public:
	// --- plant constants, MEASURED on this pod (see docs/azimuth-closed-loop.md) ---
	static constexpr float COUNTS_PER_REV = 4096.0f;
	static constexpr int   SEAM_CMD       = 515;    // handoff at sine_cross_up = 36

	// --- tunables (all settable at runtime via `az gain`) ---
	// TUNED ON THIS POD, on-device. ki is 30x SMALLER than the host loop's 1.5, and that is not a
	// port artefact: the inner term integrates (vsp - vel), vsp is clamped to vmax while vel reaches
	// thousands of deg/s in a transient, so the integral is fed an error it can never null and slams
	// to the rail — at 1 kHz it does so 20x faster than it did at 50 Hz. Running the loop quicker
	// does not help by itself, because the motors still need 0.1-0.2 s to change speed; the rate
	// buys phase margin only once the gains stop over-driving that lag.
	// CORRECTION 2026-08-12 — THESE DEFAULTS DESCEND FROM THE WRONG HOST RUN. See
	// docs/azimuth-closed-loop.md "THE PORT COPIED THE WRONG RUN". The host configuration that
	// actually converged (final err +0.00, and it recovered from a hand disturbance) was
	// kp 3, ki 0.5, kd 0.3, vmax 200 — NOT the kp 8 / ki 1.5 ported here, which was picked on
	// `worst`, a spin-up-transient metric, and never reproduced (-184.8, +0.88, -74.2).
	// The old claim "ki = 0 RUNS AWAY, so the integral is required" was also wrong: at vmax 200
	// nothing could supply the 5-30 counts of null bias, by EITHER path. ki and vmax size together
	// (ki 0.5 + vmax 200 converged; ki 0.05 + vmax 200 ran away; ki 0 + vmax 2000 is merely bounded).
	// Try kp 3 / ki 0.5 / kd 0.3 / vmax 200 / verr_clamp 0 first, with a trace running.
	float kp = 8.0f, ki = 0.1f, kd = 0.6f;
	float kff       = 1.0f / (7.26f * 6.0f);   // 1 / (deg/s per unit s), measured
	float s_trim    = -29.8f;                  // s that holds still; the integral trims around it
	float smax      = 40.0f;                   // azimuth authority, command counts
	// vmax MUST NOT throttle the proportional path. vsp is clamped to vmax and the feedforward is
	// kff*vsp, so the P term can never command more than kff*vmax counts — at vmax=200 that was 4.6
	// counts out of an smax of 40, i.e. nine tenths of the authority was reachable ONLY through the
	// integral. That is why pure PD "ran away" (|err| 5300-7700 deg): not instability, simply not
	// enough proportional authority to cancel a null offset needing ~15-30 counts. At vmax=2000 the
	// same PD is bounded and stable (|err| 20.6-135.0, mean 73.1).
	float vmax      = 2000.0f;                 // deg/s setpoint clamp — see above before lowering
	int   sign_a    = 1, sign_b = -1;          // B is mirrored — measured, not assumed
	float carrier   = -600.0f;                 // both motors held here, 85 counts clear of the seam
	// The spin-up transient alone reaches thousands of deg/s (the two ESCs start on independent
	// ramps, and in air the differential sends nearly all of the mismatch to the azimuth), so a
	// 400 deg/s ceiling aborts before the loop ever runs.
	float verr_clamp = 50.0f;   // bound on the velocity error fed to the integral; 50 measured best
	float max_rate  = 4000.0f;                 // deg/s abort threshold (debounced over 3 ticks)

	// --- trace ring -------------------------------------------------------------------------
	// WHY THIS EXISTS. Every azimuth result on record was read by polling `az status` from the host
	// at 0.6-1.2 s. That cannot separate "stopped 70 deg off" from "oscillating +/-70 deg", it
	// aliases anything above ~0.4 Hz, and the later sweeps logged only `err` — so `vel`, which is
	// what CONVERGENCE actually means, was never recorded at all. A loop running at 1 kHz has to
	// keep its own history; there is no host tick fast enough to stand in for it.
	// 16 bytes/sample: at the default decimation of 5 (~200 Hz) the ring holds the last ~10 s,
	// which is the window the convergence metrics are defined over.
	struct Sample { uint32_t t_us; float pos; float vel; int16_t s_q; int16_t ivel_q; };
	static const uint16_t TRACE_N = 2048;

	void bind(Thruster* a, Thruster* b, As5600Tracker* enc) { a_ = a; b_ = b; enc_ = enc; }

	bool begin(float target_deg) {
		if (!enc_ || !enc_->present()) return false;
		base_   = enc_->accum();
		target_ = target_deg;
		ivel_ = 0.0f; vel_ = 0.0f; pos_ = 0.0f; over_ = 0;
		traceClear();
		last_us_ = micros();
		mode_ = HOLD; on_ = true;
		return true;
	}

	// OPEN-LOOP STEP, for identifying the plant. The loop's every gain rests on an ASSUMED 0.1-0.2 s
	// motor time constant that has never been measured; this is what measures it. Both motors are
	// carried and a known azimuth command `s_cmd` is applied with NO feedback, so the recorded
	// velocity rise gives tau directly and its final value gives the gain in deg/s per count.
	// The rate abort still applies, `s_cmd` is held inside the loop's own authority, and the step
	// self-terminates — an open-loop command on a pod that accelerates to thousands of deg/s must
	// not depend on the host still being there to stop it.
	bool beginStep(float carrier_cmd, float s_cmd, uint32_t ms) {
		if (!enc_ || !enc_->present() || !a_ || !b_) return false;
		if (ms == 0 || ms > 10000) return false;          // typo guard: never an unbounded open loop
		carrier = carrier_cmd;
		base_   = enc_->accum();
		target_ = 0.0f;
		ivel_ = 0.0f; vel_ = 0.0f; pos_ = 0.0f; over_ = 0;
		traceClear();
		step_s_     = constrain(s_cmd, -smax, smax);
		last_us_    = micros();
		step_end_us_ = last_us_ + ms * 1000UL;
		mode_ = STEP; on_ = true;
		return true;
	}

	void stop() { on_ = false; mode_ = OFF; ivel_ = 0.0f;
	              if (a_) a_->setRaw(0); if (b_) b_->setRaw(0); }   // NOTE: keeps the trace — it is
	                                                               // read back after the run stops.
	void setTarget(float deg) { target_ = deg; }

	// --- trace readout (oldest first) ---
	void     traceClear()            { tr_head_ = 0; tr_count_ = 0; dec_n_ = 0; }
	uint16_t traceCount()      const { return tr_count_; }
	uint16_t decim()           const { return decim_; }
	void     setDecim(uint16_t d)    { decim_ = d ? d : 1; }
	const Sample& traceAt(uint16_t i) const {
		uint16_t start = (uint16_t)((tr_head_ + TRACE_N - tr_count_) % TRACE_N);
		return trace_[(uint16_t)((start + i) % TRACE_N)];
	}

	// Call from loop() as often as possible. Self-paced off micros(), so it runs at whatever rate
	// the caller can manage rather than assuming one — on this firmware that is far above the
	// host's 50 Hz, which is the entire point.
	void poll() {
		if (!on_ || !a_ || !b_ || !enc_) return;
		uint32_t now = micros();
		float dt = (now - last_us_) * 1e-6f;
		if (dt < 1e-3f) return;                       // ~1 kHz ceiling; also guards a zero dt
		last_us_ = now;

		// Position and velocity both come from the tracker's de-aliased accumulator. No host-side
		// unwrap, so nothing is discarded as a possible alias at the speeds this pod reaches.
		pos_ = (enc_->accum() - base_) * 360.0f / COUNTS_PER_REV;
		vel_ = enc_->rpm() * 6.0f;                    // signed mech rpm -> deg/s

		if (fabsf(vel_) > max_rate) { if (++over_ >= 3) { stop(); return; } }
		else over_ = 0;

		if (mode_ == STEP) {
			apply_(step_s_);
			record_(now);
			if ((int32_t)(now - step_end_us_) >= 0) stop();
			return;
		}

		float err = target_ - pos_;
		float vsp = kp * err - kd * vel_;
		vsp = constrain(vsp, -vmax, vmax);

		float uff = kff * vsp;
		// CLAMP THE VELOCITY ERROR BEFORE INTEGRATING IT. vsp is bounded by vmax but vel_ is the raw
		// plant velocity, which reaches thousands of deg/s in a transient the loop never asked for —
		// so (vsp - vel_) becomes essentially -vel_ and the integral degenerates into an integrator
		// of VELOCITY, slamming to the rail in milliseconds. Bounding the error keeps the integral
		// doing its actual job (cancelling the drifting null) instead of chasing a transient it
		// cannot influence. verr_clamp <= 0 disables, restoring the old behaviour for comparison.
		float verr = vsp - vel_;
		if (verr_clamp > 0.0f) verr = constrain(verr, -verr_clamp, verr_clamp);
		ivel_ += ki * verr * dt;
		float u = uff + ivel_;
		float uc = constrain(u, -smax, smax);
		if (u != uc) { ivel_ = constrain(uc - uff, -smax, smax); u = uc; }   // back-calculation

		apply_(u);
		record_(now);
	}

	bool  on()     const { return on_; }
	bool  stepping() const { return mode_ == STEP; }
	float stepS()  const { return step_s_; }   // AFTER the +/-smax clamp — the step the plant saw,
	                                           // which is what the gain fit must divide by.
	float pos()    const { return pos_; }
	float target() const { return target_; }
	float vel()    const { return vel_; }
	float s()      const { return s_; }

private:
	enum Mode : uint8_t { OFF, HOLD, STEP };

	// Mix. The carrier is what keeps BOTH motors past the seam, so it is never shed; u is.
	// NOTE the asymmetry this leaves, which the docs got wrong for a while: `s_trim` is added
	// OUTSIDE the +/-smax clamp, so with trim -29.8 and smax 40 the delivered s spans
	// [-69.8, +10.2] — motor B bottoms at |carrier|-69.8, i.e. 530 against a 515 seam at
	// carrier -600. Raise |carrier| if that margin matters, do not shrink smax (that costs
	// authority the loop needs to track the drifting null).
	void apply_(float u) {
		float s = u + s_trim;
		float room = fabsf(carrier) - SEAM_CMD;
		s = constrain(s, -room, room);
		a_->setRaw((int)lroundf(sign_a * (s + carrier)));
		b_->setRaw((int)lroundf(sign_b * (s - carrier)));
		s_ = s;
	}

	void record_(uint32_t now) {
		if (++dec_n_ < decim_) return;
		dec_n_ = 0;
		Sample& x = trace_[tr_head_];
		x.t_us   = now;
		x.pos    = pos_;
		x.vel    = vel_;
		x.s_q    = (int16_t)lroundf(constrain(s_    * 10.0f, -32000.0f, 32000.0f));
		x.ivel_q = (int16_t)lroundf(constrain(ivel_ * 10.0f, -32000.0f, 32000.0f));
		tr_head_ = (uint16_t)((tr_head_ + 1) % TRACE_N);
		if (tr_count_ < TRACE_N) tr_count_++;
	}

	Thruster*      a_ = nullptr;
	Thruster*      b_ = nullptr;
	As5600Tracker* enc_ = nullptr;
	bool     on_ = false;
	Mode     mode_ = OFF;
	int32_t  base_ = 0;
	uint32_t last_us_ = 0, step_end_us_ = 0;
	uint8_t  over_ = 0;
	float    target_ = 0, pos_ = 0, vel_ = 0, ivel_ = 0, s_ = 0, step_s_ = 0;

	Sample   trace_[TRACE_N];
	uint16_t tr_head_ = 0, tr_count_ = 0, decim_ = 5, dec_n_ = 0;
};
