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

	void bind(Thruster* a, Thruster* b, As5600Tracker* enc) { a_ = a; b_ = b; enc_ = enc; }

	bool begin(float target_deg) {
		if (!enc_ || !enc_->present()) return false;
		base_   = enc_->accum();
		target_ = target_deg;
		ivel_ = 0.0f; vel_ = 0.0f; pos_ = 0.0f; over_ = 0;
		last_us_ = micros();
		on_ = true;
		return true;
	}
	void stop() { on_ = false; ivel_ = 0.0f; if (a_) a_->setRaw(0); if (b_) b_->setRaw(0); }
	void setTarget(float deg) { target_ = deg; }

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

		// Mix. The carrier is what keeps BOTH motors past the seam, so it is never shed; s is.
		float s = u + s_trim;
		float room = fabsf(carrier) - SEAM_CMD;
		s = constrain(s, -room, room);
		a_->setRaw((int)lroundf(sign_a * (s + carrier)));
		b_->setRaw((int)lroundf(sign_b * (s - carrier)));
		s_ = s;
	}

	bool  on()     const { return on_; }
	float pos()    const { return pos_; }
	float target() const { return target_; }
	float vel()    const { return vel_; }
	float s()      const { return s_; }

private:
	Thruster*      a_ = nullptr;
	Thruster*      b_ = nullptr;
	As5600Tracker* enc_ = nullptr;
	bool     on_ = false;
	int32_t  base_ = 0;
	uint32_t last_us_ = 0;
	uint8_t  over_ = 0;
	float    target_ = 0, pos_ = 0, vel_ = 0, ivel_ = 0, s_ = 0;
};
