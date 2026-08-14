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
	// --- carrier transition (see spinProfile_) ---
	float spin_secs = 4.0f;                    // 0 -> carrier; the loop is closed throughout
	float spin_smax = 55.0f;                   // authority during it, capped by the seam
	// KEEP THE LOWER MOTOR THIS FAR ABOVE THE SEAM. Without it the authority formula permits the
	// delivered s to reach exactly |carrier| - SEAM, i.e. the lower motor lands ON the seam — a
	// zero margin guaranteed BY CONSTRUCTION, and the loop always spends all the room it is given.
	// MEASURED 2026-08-14 at carrier -585: ivel wound to its rail, that pushed B from 544 down to
	// 515 exactly, B fell out of 6-step and the azimuth went from ~0 to 3021 deg/s. The same
	// signature is in the 8/11 host traces (every excursion preceded by SAT with B = -531).
	float seam_keep = 35.0f;
	// Seconds at the snap carrier with the DELIVERED s forced to zero, so both motors are given the
	// identical command and cross the seam together. s_trim splits them by 2*|s_trim| (~60 counts),
	// and a pair straddling the seam is a whole motor of mismatch (~1500 rpm) that no azimuth
	// authority can cancel — measured at -944 deg/s on 2026-08-14.
	float spin_hold = 0.8f;
	// --- RPM-SUBMODE SPIN-UP. The differential's azimuth is driven by the DIFFERENCE of the two
	// motors, so two independent open-loop ramps put a whole motor of mismatch (~1500 rpm) into the
	// azimuth faster than any trim can cancel — measured 2026-08-14, every raw spin-up aborted on
	// the rate guard at 3800-4300 deg/s within 1.4 s. Giving both motors the SAME closed-loop speed
	// target instead makes each ESC regulate its own speed, so the mismatch is cancelled at its
	// source rather than trimmed afterwards. It is also the only path that reliably crosses the
	// seam at all: raw drive could not start a motor from rest, the RPM submode held 2431 rpm.
	float spin_rpm      = 1200.0f;   // matched target for BOTH motors during the transition
	float spin_rpm_ok   = 400.0f;    // both must report at least this, FRESH, to count as past the seam
	float spin_rpm_secs = 8.0f;      // give up if they never get there
	// Telem.rpm is UNSIGNED and KEEPS ITS LAST 6-STEP VALUE after the ESC drops back to forced sine,
	// so the value alone cannot tell "spinning" from "was spinning". Freshness is what distinguishes
	// them (see the Telem comment in esc_session.h), and skipping it would read a stale rpm as proof
	// the motor is up.
	static const uint32_t TELE_FRESH_MS = 150;
	// --- RPM-SPACE ACTUATION. The raw carrier drives each ESC open-loop, so the azimuth sees the
	// DIFFERENCE of two uncontrolled speeds: the seam cliff, the motor mismatch and the drifting
	// null are all consequences of that. Commanding the two SPEEDS instead makes each ESC regulate
	// its own, so the azimuth channel is (R + r) - (R - r) = 2r and the mismatch is cancelled by
	// the speed loops rather than trimmed by the position loop.
	//   thrust  = spin_rpm   (raise this to push harder)
	//   azimuth = r          (what the position loop commands)
	// Measured on this pod 2026-08-14: the two velocity loops were configured differently — slew
	// 200 vs 2000 rpm/s — so the same target ramped one motor ten times faster than the other, and
	// that difference went straight into the azimuth. That, not physics, was the "whole motor of
	// mismatch" the spin-up notes had treated as unavoidable since 8/11.
	bool  rpm_drive     = true;
	// MEASURED 2026-08-14 in rpm space (each point approached from r = 0, R = -800):
	//   r = -200 -> -371 deg/s, +100 -> +1251, +200 -> +2011  =>  slope +6.0 deg/s per rpm,
	//   null at r ~ -137 rpm. Chosen so the feedforward is exact: rate = 6.0 * (kff*vsp) *
	//   rpm_per_count must equal vsp, i.e. rpm_per_count = 1/(6.0*kff) = 7.26.
	float rpm_per_count = 7.26f;
	float rpm_trim      = -137.0f;   // rpm offset that holds the azimuth still — the integral trims
	                                 // around it, exactly as s_trim does for the raw path
	float rpm_offset_max = 400.0f;   // authority, mech rpm

	// --- trace ring -------------------------------------------------------------------------
	// WHY THIS EXISTS. Every azimuth result on record was read by polling `az status` from the host
	// at 0.6-1.2 s. That cannot separate "stopped 70 deg off" from "oscillating +/-70 deg", it
	// aliases anything above ~0.4 Hz, and the later sweeps logged only `err` — so `vel`, which is
	// what CONVERGENCE actually means, was never recorded at all. A loop running at 1 kHz has to
	// keep its own history; there is no host tick fast enough to stand in for it.
	// 16 bytes/sample: at the default decimation of 5 (~200 Hz) the ring holds the last ~10 s,
	// which is the window the convergence metrics are defined over.
	// ra/rb are the MEASURED motor speeds. Without them a trace cannot say whether the plant did what
	// the loop asked — every remaining question on 2026-08-14 was of that form, and answering it by
	// polling `tele` from the host at 0.8 s aliased a pod moving at thousands of deg/s.
	struct Sample { uint32_t t_us; float pos; float vel;
	                int16_t s_q; int16_t ivel_q; int16_t c_q; int16_t ra; int16_t rb; };
	static const uint16_t TRACE_N = 2048;

	void bind(Thruster* a, Thruster* b, As5600Tracker* enc) { a_ = a; b_ = b; enc_ = enc; }

	bool begin(float target_deg) {
		if (!enc_ || !enc_->present()) return false;
		base_   = enc_->accum();
		target_ = target_deg;
		ivel_ = 0.0f; vel_ = 0.0f; pos_ = 0.0f; over_ = 0;
		traceClear();
		last_us_ = t0_us_ = micros();
		spinning_ = true;
		mode_ = HOLD; on_ = true;
		beginRpmSpinup_();
		return true;
	}

	// OPEN-LOOP STEP, for identifying the plant. The loop's every gain rests on an ASSUMED 0.1-0.2 s
	// motor time constant that has never been measured; this is what measures it. Both motors are
	// carried and a known azimuth command `s_cmd` is applied with NO feedback, so the recorded
	// velocity rise gives tau directly and its final value gives the gain in deg/s per count.
	// The rate abort still applies, `s_cmd` is held inside the loop's own authority, and the step
	// self-terminates — an open-loop command on a pod that accelerates to thousands of deg/s must
	// not depend on the host still being there to stop it.
	// THE PRE-ROLL IS CLOSED-LOOP, AND IT HAS TO BE. Applying the step straight from rest measures
	// two things at once: both ESCs ramping up from zero on independent schedules — a mismatch
	// documented at up to a WHOLE motor, which the differential sends almost entirely to the azimuth
	// in air — and the azimuth's actual response to the step.
	// Holding s at the STATIC null through the ramp does not fix it, and MEASURED 2026-08-14 it
	// diverges: with s pinned at s_trim the pod reached 4090 deg/s in 1.3 s and tripped the rate
	// abort before the motors had even reached the carrier. There is no null during a ramp — the
	// azimuth is a pure integrator, so the ramp mismatch accumulates into heading without limit.
	// So `pre_ms` runs the ORDINARY closed loop (the host reached the same conclusion in 5d59aea,
	// "close the loop through the SPIN-UP, not just after it"), and the step is applied relative to
	// the command the loop settled on. That also makes the identified step exactly step_s_ and the
	// baseline velocity a measured zero rather than an assumption.
	bool beginStep(float carrier_cmd, float s_cmd, uint32_t ms, uint32_t pre_ms) {
		if (!enc_ || !enc_->present() || !a_ || !b_) return false;
		if (ms == 0 || ms > 10000 || pre_ms > 10000) return false;   // never an unbounded open loop
		carrier = carrier_cmd;
		base_   = enc_->accum();
		target_ = 0.0f;
		ivel_ = 0.0f; vel_ = 0.0f; pos_ = 0.0f; over_ = 0;
		traceClear();
		step_s_       = constrain(s_cmd, -smax, smax);
		step_armed_   = false; step_base_u_ = 0.0f;
		last_us_ = t0_us_ = micros();
		spinning_ = true;
		// pre_ms is settle time AFTER the carrier transition, not counted from the very start.
		step_pre_us_ = (uint32_t)(spin_secs * 1e6f) + pre_ms * 1000UL;
		step_run_us_ = ms * 1000UL;
		step_begin_us_ = last_us_ + step_pre_us_;      // re-based when the RPM spin-up hands over
		step_end_us_   = step_begin_us_ + step_run_us_;
		mode_ = STEP; on_ = true;
		return true;
	}

	void stop() { on_ = false; mode_ = OFF; ivel_ = 0.0f; step_armed_ = false; rpm_phase_ = false;
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

		// The loop runs from the first tick — including through the spin-up, which is the whole
		// point (the host reached the same conclusion in 5d59aea). What the spin-up window gates is
		// only WHEN the heading is zeroed: drift accrued before both motors are actually up is a
		// transition artefact, not tracking error.
		if (rpm_phase_) {
			escs::Telem ta, tb; uint32_t ms = millis();
			bool oka = a_->tele(ta) && ta.valid && ta.rpm >= (uint32_t)spin_rpm_ok
			           && (uint32_t)(ms - ta.rpmStampMs) < TELE_FRESH_MS;
			bool okb = b_->tele(tb) && tb.valid && tb.rpm >= (uint32_t)spin_rpm_ok
			           && (uint32_t)(ms - tb.rpmStampMs) < TELE_FRESH_MS;
			if (oka && okb) {
				rpm_phase_ = false;
				base_ = enc_->accum(); pos_ = 0.0f; ivel_ = 0.0f;
				t0_us_ = now; spinning_ = false;
				if (mode_ == STEP) { step_begin_us_ = now + step_pre_us_;
				                     step_end_us_   = step_begin_us_ + step_run_us_; }
			} else if ((uint32_t)(now - rpm_t0_) > (uint32_t)(spin_rpm_secs * 1e6f)) { stop(); return; }
		}

		// --- carrier transition. NOT a jump; see spinProfile_(). ---
		float c_now = spinProfile_(now);
		bool  below = fabsf(c_now) < (float)SEAM_CMD;
		// Authority is capped by the seam: s = u + s_trim must keep BOTH motors in 6-step, so
		// |u| <= |carrier| - seam - |s_trim|. During the transition the loop is fighting a mismatch
		// of up to a whole motor, so it gets all the room that leaves; afterwards it uses smax.
		float room_u = fabsf(c_now) - (float)SEAM_CMD - fabsf(s_trim) - seam_keep;
		// In rpm space the seam is the ESC's problem, not ours: each velocity loop keeps its own
		// motor wherever it needs to be. So the authority is bounded by rpm_offset_max, not by the
		// carrier's headroom above the seam.
		float u_max  = rpm_drive ? (rpm_offset_max / fmaxf(rpm_per_count, 1e-3f))
		                         : fmaxf(1.0f, fminf(spinning_ ? spin_smax : smax, room_u));

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
		// NO INTEGRAL BELOW THE SEAM. There `s` has almost no authority over the azimuth, so the
		// loop can see an error it cannot act on; an integrator left running there winds up and
		// then dumps the whole windup the instant authority appears. Held at zero instead.
		if (below) ivel_ = 0.0f; else ivel_ += ki * verr * dt;
		float u = uff + ivel_;
		float uc = constrain(u, -u_max, u_max);
		if (u != uc) { ivel_ = constrain(uc - uff, -u_max, u_max); u = uc; }   // back-calculation

		// STEP mode: the pre-roll above ran the CLOSED loop (see beginStep); from step_begin_us_ the
		// loop is frozen at whatever command it had settled on and the step is added open-loop.
		// Stepping relative to the settled command — not to the static s_trim — is what makes the
		// identified step size exactly step_s_ and the baseline velocity a measured zero.
		// CROSS THE SEAM WITH s == 0. s_trim splits the two commands by 2*|s_trim| (~60-70 counts
		// here), so while the carrier is near the seam one motor is already in 6-step and the other
		// is still in forced sine. MEASURED 2026-08-14: that state drives the azimuth at -944 deg/s,
		// a whole motor's worth of mismatch (~1500 rpm), which no azimuth authority can cancel —
		// spin-up aborted on the rate guard every time. Holding the DELIVERED s at zero makes both
		// motors identical, so they cross together and the only mismatch left is the intrinsic one
		// the trim exists to cancel. The trim (and the loop) come back once both are clear.
		if (!rpm_drive && spinning_ && (int32_t)(now - t0_us_) < (int32_t)(spin_hold * 1e6f)) {
			ivel_ = 0.0f;
			u = -s_trim;                       // => apply_ delivers s = u + s_trim = 0
		}

		if (mode_ == STEP && (int32_t)(now - step_begin_us_) >= 0) {
			if (!step_armed_) { step_armed_ = true; step_base_u_ = u; }
			u = constrain(step_base_u_ + step_s_, -u_max, u_max);
		}

		if (rpm_drive) applyRpm_(u); else apply_(u, c_now);
		record_(now);
		if (mode_ == STEP && (int32_t)(now - step_end_us_) >= 0) stop();
	}

	bool  on()     const { return on_; }
	bool  stepping() const { return mode_ == STEP; }
	bool  spinningRpm() const { return rpm_phase_; }
	float stepS()  const { return step_s_; }
	uint32_t stepBeginUs() const { return step_begin_us_; }   // AFTER the +/-smax clamp — the step the plant saw,
	                                           // which is what the gain fit must divide by.
	float pos()    const { return pos_; }
	float target() const { return target_; }
	float vel()    const { return vel_; }
	float s()      const { return s_; }
	float carrierNow() const { return c_; }
	bool  spinning()   const { return spinning_; }

private:
	enum Mode : uint8_t { OFF, HOLD, STEP };

	// Mix. The carrier is what keeps BOTH motors past the seam, so it is never shed; u is.
	// NOTE the asymmetry this leaves, which the docs got wrong for a while: `s_trim` is added
	// OUTSIDE the +/-smax clamp, so with trim -29.8 and smax 40 the delivered s spans
	// [-69.8, +10.2] — motor B bottoms at |carrier|-69.8, i.e. 530 against a 515 seam at
	// carrier -600. Raise |carrier| if that margin matters, do not shrink smax (that costs
	// authority the loop needs to track the drifting null).
	void apply_(float u, float c) {
		float s = u + s_trim;
		float room = fabsf(c) - SEAM_CMD - seam_keep;
		s = constrain(s, -room, room);
		a_->setRaw((int)lroundf(sign_a * (s + c)));
		b_->setRaw((int)lroundf(sign_b * (s - c)));
		s_ = s; c_ = c;
	}

	// TWO-STAGE, and the stages are not cosmetic. Below the seam the ESC is in forced sine and `s`
	// has almost no authority over the azimuth, so time spent there is drift that can never be
	// recovered — hence the snap PAST the seam first. Above it the loop can act, so the rest is a
	// ramp, which keeps the two ESCs' independent start ramps from diverging faster than the loop
	// can trim. MEASURED on the host: two-stage +7.8 deg of drift, vs ramp +182 and jump -210.
	// A plain jump is what the firmware used to do, and on 2026-08-14 it put the pod at 4090 deg/s.
	float spinProfile_(uint32_t now) {
		float el = (float)(uint32_t)(now - t0_us_) * 1e-6f;
		if (el >= spin_secs) {
			if (spinning_) { spinning_ = false; ivel_ = 0.0f; }  // no transition windup into the hold
			return carrier;
		}
		spinning_ = true;
		float sgn  = (carrier < 0.0f) ? -1.0f : 1.0f;
		// THE SNAP TARGET MUST CLEAR THE SEAM FOR THE *LOWER* MOTOR, NOT FOR THE CARRIER.
		// s_trim splits the two commands by 2*|s_trim| (A = c + s, B = c - s), so at carrier
		// SEAM+40 = 555 with trim -35 the pair is A=590 (6-step) and B=520 (still forced sine) —
		// exactly the one-motor-in-one-motor-out state, which MEASURED -944 deg/s on 2026-08-14.
		// The profile used to park the pod there on every start.
		// The snap must fund EVERYTHING the loop needs at once: the trim's split, the authority the
		// loop will actually use, and the margin that keeps the lower motor off the seam. Snapping
		// to SEAM+|trim|+40 = 585 looked safe and left room_u = 5 counts — the loop had no authority
		// at all and the pod ran away regardless (measured 2026-08-14). Fund all three or the
		// transition is unrecoverable by construction.
		float past = sgn * ((float)SEAM_CMD + fabsf(s_trim) + spin_smax + seam_keep);
		if (fabsf(past) > fabsf(carrier)) past = carrier;        // never overshoot the target carrier
		if (el < 0.4f) return past;
		float f = fminf(1.0f, (el - 0.4f) / fmaxf(spin_secs - 0.4f, 1e-3f));
		return past + (carrier - past) * f;
	}

	// (R + r) and (R - r): the sum is thrust, the difference is azimuth. s_ / c_ are still recorded
	// in COMMAND units so one set of metrics reads both actuation modes.
	void applyRpm_(float u) {
		float sgn = (carrier < 0.0f) ? -1.0f : 1.0f;
		float r   = constrain(u * rpm_per_count + rpm_trim, -rpm_offset_max, rpm_offset_max);
		// PARALLEL TO THE RAW MIX, and the parallel has to be exact: raw is a = c + s, b = c - s,
		// so with a NEGATIVE carrier a more negative s makes |a| LARGER. Writing sgn*(R + r) puts
		// the sign outside and inverts that — the loop then drives the error instead of correcting
		// it. Measured 2026-08-14: u pinned at its -28.6 rail while pos ran 85 -> 599 deg and
		// accelerated to 4048 deg/s. Same class of error as the sign_b placeholder, new coordinate.
		a_->setRpmTarget(sgn * spin_rpm + r);
		b_->setRpmTarget(sgn * spin_rpm - r);
		s_ = u; c_ = spin_rpm;
	}

	void beginRpmSpinup_() {
		rpm_phase_ = true; rpm_t0_ = micros();
		float sgn = (carrier < 0.0f) ? -1.0f : 1.0f;
		if (a_) a_->setRpm(sgn * spin_rpm);
		if (b_) b_->setRpm(sgn * spin_rpm);
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
		x.c_q    = (int16_t)lroundf(constrain(c_,           -32000.0f, 32000.0f));
		escs::Telem ta, tb;
		x.ra = (a_ && a_->tele(ta) && ta.valid) ? (int16_t)min<uint32_t>(ta.rpm, 32000) : -1;
		x.rb = (b_ && b_->tele(tb) && tb.valid) ? (int16_t)min<uint32_t>(tb.rpm, 32000) : -1;
		tr_head_ = (uint16_t)((tr_head_ + 1) % TRACE_N);
		if (tr_count_ < TRACE_N) tr_count_++;
	}

	Thruster*      a_ = nullptr;
	Thruster*      b_ = nullptr;
	As5600Tracker* enc_ = nullptr;
	bool     on_ = false;
	Mode     mode_ = OFF;
	int32_t  base_ = 0;
	uint32_t last_us_ = 0, t0_us_ = 0, step_begin_us_ = 0, step_end_us_ = 0;
	uint8_t  over_ = 0;
	float    target_ = 0, pos_ = 0, vel_ = 0, ivel_ = 0, s_ = 0, c_ = 0;
	float    step_s_ = 0, step_base_u_ = 0;
	bool     step_armed_ = false, spinning_ = false, rpm_phase_ = false;
	uint32_t rpm_t0_ = 0, step_pre_us_ = 0, step_run_us_ = 0;

	Sample   trace_[TRACE_N];
	uint16_t tr_head_ = 0, tr_count_ = 0, decim_ = 5, dec_n_ = 0;
};
