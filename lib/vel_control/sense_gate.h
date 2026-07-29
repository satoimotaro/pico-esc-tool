// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// sense_gate — the VALIDITY WINDOW of the ESC soft-sensor: "is `vest` invertible at THIS operating
// point?", decided from the values WE CHOSE (the command) rather than from the telemetry we are trying
// to qualify. Extracted out of the Arduino-bound Thruster so the one piece of logic that decides
// whether a number may be published is hardware-free and natively testable — the same move the library
// already made for validateCurve()/tagRegimes().
//
// WHY THE COMMAND AND NOT THE RPM (the bug this file exists to fix). The old gate was
// `vc.live() && |vc.measured()| >= lineFloor()`, i.e. both terms came from `tele`. The 2026-07-30 bench
// run proved `tele` is not a witness down there: it reported a constant VIRTUAL 182 rpm for every
// command from 60 to 450 while the rotor actually went 39 -> 170 rpm, and only became honest from
// ~cmd 520 (cmd 530: tele 319 vs encoder 316; cmd 540: 436 vs 431). A gate that reads `tele` cannot
// detect its own invalidity, so it published a number the caller could not tell apart from a good one.
// In forced sine the firmware runs mech = (cmd/1000) * SINE_FULLSCALE_ERPM / pp, so the handoff
// COMMAND is up_erpm / SINE_FULLSCALE_ERPM * 1000 — the pole pairs cancel exactly (see
// MotorModel::cmdForSine(handoffMech())), and for the bench's up_erpm = 1400 that is 560, right where
// telemetry was measured to become honest.
//
// THREE FLAGS, NOT ONE. Folding the load conditions into `valid` would make `load` unavailable exactly
// when a load appears — killing the sensor's primary product. So:
//   valid()    the vest <-> V_eff inversion holds here at all (the publishable-at-all gate);
//   loadOk()   this command is comparable to the operating point `sense zero` was captured at;
//   vbattOk()  this sample is the light-load / high-duty subset where `vcal` cancels the residual
//              I*R/duty deficit by construction.
//
// PORTABLE: stdint + math + motor_model.h only. No Arduino/PIO headers (see test/test_sense_gate.cpp).
#pragma once
#include <stdint.h>
#include <math.h>
#include "motor_model.h"     // SINE_FULLSCALE_ERPM — the forced-sine full scale the cmd floor inverts

namespace vel {

// ---------------------------------------------------------------------------------------------------
// THE THRESHOLDS — deliberately all in one place. None of them is bench-validated (they were chosen
// with no hardware available); each carries the reason and the number behind it so a bench pass can
// move them with evidence rather than by taste.
// ---------------------------------------------------------------------------------------------------
// Enter/stay margins above the derived handoff command. The bench runs cross_up = 36, i.e. a seam of
// 36 * 39.0625 = 1406 eRPM and a handoff command of 563, so enter = 591 and stay = 574 — against the
// measured honest onset of ~520. A real margin without being greedy: the existing `loadtest` operating
// point (1200 rpm => cmd ~= 614 at cross_up = 36) must stay inside the window, and 614 / 563 = 1.091,
// so 5 % keeps it in with 3.8 % to spare while 9 % would have excluded it outright.
// The gap between the two is the hysteresis: a command dithering around the floor must not chatter the
// validity flag (every transition re-seeds the vest filter and restarts the dwell).
static const float SENSE_CMD_ENTER_MARGIN = 1.05f;
static const float SENSE_CMD_EXIT_MARGIN  = 1.02f;
// Upper end: the 6-step duty map is an AFFINE fit (duty = A*cmd + B) to a firmware modulator that
// cannot exceed full modulation, so as duty approaches 1 the fit is extrapolating past where it was
// measured and rpm6step() saturates against noLoadTop — `vest` then inverts a duty the ESC is not
// actually delivering. 0.90 keeps 10 % of headroom below that breakdown; on the 350KV/11.1 V bench at
// cross_up = 36 it is cmd ~= 837, well above every operating point the repo uses (the parametric curve
// tops out at cmd 700).
static const float SENSE_DUTY_CEILING     = 0.90f;
// Dwell: stay continuously in-window this long before latching valid. Longer than the 130 ms worst-case
// 6-step plant tau measured in sysid (25-130 ms), so the estimate is taken on a settled rotor and not
// on the catch transient. Leaving the window invalidates on the SAME tick — the fail-safe direction.
static const float SENSE_DWELL_SECS       = 0.30f;
// vbatt tier: a sample counts as light-load while the measured droop is under 10 % of the learned
// reference (~1.1 V on an 11.1 V pack). The `vcal` scale is MULTIPLICATIVE, so it can only cancel the
// residual no-load I*R/duty deficit at the operating point it was taken at; restricting the rolling max
// to light load is what makes "V_eff -> V_supply" true by construction rather than by hope.
static const float SENSE_LOAD_CEILING     = 0.10f;
// load tier: `load` is a DIFFERENCE against the zero point, and the residual deficit is operating-point
// dependent — it flips worst-case between air (worst at low rpm) and water (worst at high rpm) — so a
// reading taken far from the zero point subtracts two different biases.
//
// The band is in DUTY, not in command units, for two reasons. First, duty is what the deficit actually
// scales with (V_eff = rpm/(KV*duty)); second, the duty map has a large negative intercept
// (duty = 0.00265*cmd - 1.317 at cross_up 36), so a percentage of the COMMAND is a wildly different
// fraction of the duty depending where you are: +-15 % of the 1600 rpm point (cmd 652, duty 0.411) was
// +-98 counts = duty 0.15..0.67, i.e. ~590..2600 rpm — nearly the whole 6-step envelope, at which width
// `loadok` degenerates into "a zero point exists". A further bonus: cross_up shifts duty6() by a
// constant, so a DIFFERENCE of two duties is invariant to it, where a command band is not.
//
// 0.10 duty is bounded from BOTH sides and both bounds are worth writing down:
//   * FLOOR — `load` has to survive load, which is the whole reason this is a separate flag from
//     `valid`. Holding rpm through a droop of d costs duty * 1/(1-d): the 10 % droop that closes the
//     vbatt tier moves duty 0.411 -> 0.457, i.e. 0.046. So the band must exceed ~0.046 or `loadok`
//     would close exactly when a load appears. 0.10 tolerates ~20 % droop, twice the vbatt ceiling.
//     (In command units that droop is only ~17 counts / 2.6 % — which is WHY `load` survives load; the
//     old command-fraction band got the right answer for a reason it never stated.)
//   * CEILING — it still has to mean "the same operating point". 0.10 duty is +-38 command counts,
//     ~+-390 rpm at 350KV/11.1 V: the driftsoak point stays comparable to itself, a deliberate move to
//     1200 rpm does not. Zeroed at one setpoint, `load` reads 0 at the other — by design.
// Not bench-validated (nothing here is); a bench pass should tighten toward the 0.046 floor.
static const float SENSE_OFFPOINT_DUTY    = 0.10f;
// An UNCONFIGURED seam (up_erpm <= 0) must CLOSE the gate, not open it. Reporting a floor above the
// +-1000 command range says exactly that on the wire ("no command can validate this"), where a 0 floor
// would have read as "everything qualifies".
static const float SENSE_CMD_UNREACHABLE  = 1001.0f;

// Why the gate is closed. Ordered by the sequence the conditions are tested in, which is also their
// diagnostic value: NOT_LIVE (no telemetry at all) < CMD_LOW (we never asked for 6-step) < DUTY_HIGH
// < RPM_LOW (we asked, the rotor did not catch) < SETTLING (in-window, dwell not yet elapsed).
enum class SenseWhy : uint8_t { OK, NOT_LIVE, CMD_LOW, DUTY_HIGH, RPM_LOW, SETTLING };

// Short wire/CSV-safe name (no '|' or ',' — the `sense` line and driftsoak's CSV both carry it).
inline const char* senseWhyName(SenseWhy w) {
	switch (w) {
		case SenseWhy::OK:        return "ok";
		case SenseWhy::NOT_LIVE:  return "notlive";
		case SenseWhy::CMD_LOW:   return "cmdlow";
		case SenseWhy::DUTY_HIGH: return "dutyhigh";
		case SenseWhy::RPM_LOW:   return "rpmlow";
		case SenseWhy::SETTLING:  return "settling";
	}
	return "?";
}

// The forced-sine command that lands exactly on a seam of `upErpm`: mech = (cmd/1000)*FS/pp and the
// seam is upErpm/pp, so the pole pairs cancel and the command is a pure firmware property. 0 when the
// seam is unknown (callers turn that into a closed gate, see SenseGate::cmdFloor()).
inline float handoffCmdFor(float upErpm) {
	return upErpm > 0.0f ? upErpm / SINE_FULLSCALE_ERPM * 1000.0f : 0.0f;
}

// ---------------------------------------------------------------------------------------------------
// SenseGate — one per soft-sensor. Configure it from the PROFILE's seam (Thruster::crossUpErpm(), not
// the parametric model's: `curve commit <pp>` can legitimately make the two diverge) and the profile's
// own 6-step landing (SpeedProfile::lineFloor()), then update() it once per control tick.
// ---------------------------------------------------------------------------------------------------
struct SenseGate {
	// --- INSTALLED per tick by configure(), NOT tunables: both are properties of the live profile, so
	//     anything written here directly is overwritten on the next poll(). Read them, don't set them. ---
	float up_erpm      = 0.0f;                       // profile seam, eRPM. 0 => gate permanently closed
	float rpm_floor    = 0.0f;                       // secondary "the rotor actually caught" floor, mech

	// --- genuine tunables: defaults are the constants above, and nothing in the firmware moves them ---
	float duty_ceiling = SENSE_DUTY_CEILING;
	float dwell_secs   = SENSE_DWELL_SECS;
	float enter_margin = SENSE_CMD_ENTER_MARGIN;
	float exit_margin  = SENSE_CMD_EXIT_MARGIN;
	float load_ceiling = SENSE_LOAD_CEILING;
	float offpoint_duty = SENSE_OFFPOINT_DUTY;

	void configure(float upErpm, float rpmFloor) { up_erpm = upErpm; rpm_floor = rpmFloor; }

	// The command a caller must reach to OPEN the window, and the (lower) one it must hold to stay in.
	float cmdFloor() const { float h = handoffCmdFor(up_erpm); return h > 0.0f ? h * enter_margin : SENSE_CMD_UNREACHABLE; }
	float cmdExit()  const { float h = handoffCmdFor(up_erpm); return h > 0.0f ? h * exit_margin  : SENSE_CMD_UNREACHABLE; }

	// Clear every latch. Called from arm()/stop(): both end the condition under which anything measured
	// before them is comparable to anything measured after.
	void reset() {
		valid_ = false; in_window_ = false; fresh_ = false; load_ok_ = false; vbatt_ok_ = false;
		dwell_ = 0.0f; why_ = SenseWhy::NOT_LIVE;
	}

	// One tick.
	//   live    telemetry liveness as the controller sees it (necessary, NOT sufficient — see the header)
	//   cmd     the SIGNED command we are actually sending (the term the whole gate is keyed on)
	//   rpm     measured mech rpm (used only for the secondary "the rotor caught" floor)
	//   duty    the model's 6-step duty for that command (MotorModel::duty6 — passed in, so this stays
	//           independent of which model/cross_up the caller inverts)
	//   loadV   the INSTANTANEOUS droop (vref - vest) for the light-load tier; 0 when not baselined
	//   vrefV   the learned light-load reference the droop is measured against; <= 0 => no reference
	//   zeroDuty the model duty at the command `sense zero` captured the reference at; <= 0 => never
	//           captured. Duty, not command — see SENSE_OFFPOINT_DUTY.
	//   dt      elapsed seconds (the dwell is in TIME: poll() runs at the core0 loop rate, not 50 Hz)
	void update(bool live, float cmd, float rpm, float duty, float loadV, float vrefV,
	            float zeroDuty, float dt) {
		float a = fabsf(cmd);
		SenseWhy why = SenseWhy::OK;
		// EVERY test below is written as the NEGATION of the condition that must HOLD, never as the
		// condition that must fail. That is what makes the cascade fail CLOSED on a NaN or Inf: a
		// one-directional comparison (`a < floor`) is false for a NaN operand, so garbage would have
		// walked straight through to why = OK and latched valid on a rotor nobody was measuring.
		// `!(a >= floor)` is true for a NaN, so each non-finite input is rejected at the first test that
		// touches it — and rejected with the `why` that names the term that was unusable.
		if (!live)                                             why = SenseWhy::NOT_LIVE;
		// Hysteresis: the bar to STAY in is the lower one, so a command dithering around the floor
		// cannot chatter the latch. Not yet latched => the full enter margin has to be cleared.
		else if (!(a >= (valid_ ? cmdExit() : cmdFloor())))    why = SenseWhy::CMD_LOW;
		else if (!(duty <= duty_ceiling))                      why = SenseWhy::DUTY_HIGH;
		else if (!(rpm_floor <= 0.0f || fabsf(rpm) >= rpm_floor)) why = SenseWhy::RPM_LOW;

		bool was_in = in_window_;
		in_window_ = (why == SenseWhy::OK);
		fresh_ = in_window_ && !was_in;                   // caller re-seeds its filter on this tick
		if (!in_window_) {
			valid_ = false; dwell_ = 0.0f;                // leaving invalidates IMMEDIATELY (fail-safe)
		} else {
			if (dwell_ < dwell_secs) dwell_ += (dt > 0.0f ? dt : 0.0f);
			if (dwell_ >= dwell_secs) valid_ = true; else why = SenseWhy::SETTLING;
		}
		why_ = why;

		// --- the two sub-tiers. Both REQUIRE valid_ (they are subsets of it, never a way around it). They
		//     are POSITIVE conjunctions — every term must come out true to OPEN a tier — which fails closed
		//     on a non-finite input for the mirror-image reason the cascade above does: a comparison
		//     against a NaN is false, and false here means the tier stays shut. ---
		// load: only comparable to the zero point while we are near the DUTY it was captured at. With no
		// zero point there is no load number at all (senseLoad() is 0), so this reads false — "the load
		// reading is not usable", which is exactly true.
		load_ok_ = valid_ && zeroDuty > 0.0f && (fabsf(duty - zeroDuty) <= offpoint_duty);
		// vbatt: light load only. With NO reference yet (vrefV <= 0) there is nothing to measure the
		// droop against — the consumer of this flag is a rolling MAX, which is itself a light-load
		// selector (V_eff <= V_supply always), so admit the sample rather than freezing the estimate out
		// of existence before the operator has ever run `sense zero`.
		vbatt_ok_ = valid_ && (vrefV <= 0.0f || loadV <= load_ceiling * vrefV);
	}

	// --- introspection ---
	bool     valid()      const { return valid_; }
	bool     loadOk()     const { return load_ok_; }
	bool     vbattOk()    const { return vbatt_ok_; }
	SenseWhy why()        const { return why_; }
	const char* whyName() const { return senseWhyName(why_); }
	bool     inWindow()   const { return in_window_; }   // conditions met; the dwell may still be running
	bool     freshEntry() const { return fresh_; }       // first in-window tick => re-seed the vest filter
	float    dwell()      const { return dwell_; }

private:
	bool     valid_ = false, in_window_ = false, fresh_ = false, load_ok_ = false, vbatt_ok_ = false;
	float    dwell_ = 0.0f;
	SenseWhy why_ = SenseWhy::NOT_LIVE;
};

}  // namespace vel
