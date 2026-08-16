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
	// DEFAULTS ARE THE MEASURED BEST, 2026-08-15, at spin_rpm 800 in rpm-space actuation — chosen on
	// the DISTRIBUTION of 3 runs each across 7 configurations, not on a single run (the same config
	// has given pos pk-pk 89 and 447 deg back to back, and picking on one run is how this thread
	// went wrong before). Medians of pos pk-pk:
	//     kp 0.5 ki 0.3 lpf 1     ->  135   <- this
	//     kp 0.5 ki 0.3 lpf 2     ->  366
	//     kp 0.5 ki 0.3 lpf 0.5   ->  382      (over-filtering costs phase)
	//     kp 1.0 ki 0.3 lpf 1     ->  334
	//     kp 0.5 ki 0.15 lpf 1    ->  479
	//     kp 0.5 ki 0.3 lpf 0     ->  460      (and err mean swung -226..+13)
	//     kp 0.5 ki 0.3 kd 0.3+   ->  766      (kd is destabilising even filtered)
	// kd is DELIBERATELY ZERO: the disturbance sits at ~8 Hz and the velocity estimate's own lag
	// puts the derivative term in phase to reinforce it.
	float kp = 0.5f, ki = 0.3f, kd = 0.0f;
	// INTEGRAL ACTION ON POSITION. `ki` integrates the VELOCITY error, which removes a rate bias and
	// is why the pod can be made to stop drifting; nothing in this loop ever removed a POSITION bias,
	// so it stopped wherever it happened to be. MEASURED 2026-08-15 at the best rate-hold found
	// (kp 0.6, ki 0.5, verr_clamp 15): net rotation over 12 s went to a median of -15 deg — drift
	// solved — while mean|err| sat at 105-121 deg against 90 for a free spin, i.e. a large standing
	// heading offset. Raising kp did not touch it (0.6 / 1.5 / 3.0 gave -15 / 272 / 760 deg of net
	// rotation, worse, and mean|err| unchanged at 116-121).
	// So: make the OUTER loop a PI on position producing the velocity setpoint, which is the ordinary
	// cascade. 0 disables and restores the pure-P outer loop.
	float ki_pos = 0.0f;
	float ipos_max = 500.0f;         // deg-seconds, bound on the position integrator
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
	// THE RATE GUARD HAS TO BE PHASE-DEPENDENT, BECAUSE CROSSING THE SEAM IS NOT A FAULT.
	// MEASURED 2026-08-15 with the position loop given LITERALLY ZERO authority — kp=ki=kd=0,
	// rpm_trim 0, rpm_offset_max 0, so both motors carry the IDENTICAL speed target and the loop
	// cannot move the pod at all — 3/3 runs still aborted on this guard at 3896/4109/4231 deg/s,
	// 1.2 s in. Nothing any gain does can prevent that, which is why 21+ configurations swept over
	// eight hours were all "insensitive": the swept parameter has no path to the event that stops
	// the run. It is the seam crossing itself (host/azimuth.py records the same thing from 8/11 —
	// "the azimuth goes from a few degrees of forced-sine creep straight to hundreds of rpm the
	// instant it crosses the seam", ~554 rpm = 3324 deg/s measured).
	// AFTER the handover the same pod is well behaved: at a matched, steady 800 rpm in 6-step it
	// drifts at a STEADY 952 deg/s (encwatch, 5 s, 10.5 revolutions), which is the static plant's
	// own prediction for r = 0 — 6.0 * (0 - rpm_trim) = 822 deg/s, 16 % out. So the tight guard
	// belongs there, and only there. A single 4000 kills every run before the hold it protects
	// has begun; the host loop has been phase-dependent since 8/11 and the firmware had not caught up.
	// THE GUARD IS ABOUT SUSTAINED ROTATION, NOT ABOUT A PEAK.
	// `over_ >= 3` at the ~1 kHz tick meant THREE MILLISECONDS above the threshold ended the run, so
	// every seam crossing — a transient that is expected, that scales with thrust, and that the
	// operator explicitly accepts — was fatal. What is actually dangerous is turning fast for a
	// WHILE: that is what winds a loom, and it is what makes a runaway a runaway rather than a kick.
	// So integrate the TIME spent above max_rate (leaky, so brief excursions recover) and abort on
	// the budget; keep a separate hard_rate ceiling, still debounced over 3 ticks, for the real thing.
	// At the defaults a runaway is stopped after 0.5 s above 4000 deg/s = ~5.5 revolutions.
	float max_rate  = 4000.0f;                 // deg/s: "fast" for the purposes of the budget below
	float over_secs = 0.5f;                    // how long |vel| may stay above it before aborting
	float hard_rate = 12000.0f;                // instantaneous ceiling, debounced over 3 ticks
	float spin_max_rate = 6000.0f;             // ... while crossing the seam, where 4300 is NORMAL
	// The lurch does not stop at the handover — in az_ZERO0 the pair reported "both up" at 0.932 s
	// and the 4231 deg/s peak came at 1.24 s. The wide guard has to outlast the crossing, not the
	// bookkeeping event that follows it.
	float spin_guard_secs = 0.6f;              // ... and this long after the pair reports up
	// LOW-PASS THE VELOCITY THE CONTROL LAW SEES — not the one the rate guard sees.
	// MEASURED 2026-08-14/15: the azimuth carries a ~8 Hz disturbance whose frequency barely moves
	// with loop gain (7.4 / 8.0 / 9.3 Hz at ki 0.1 / 0.3 / 0.9) while its amplitude changes a lot,
	// and the inner speed loops do not pass it at all — so it is a disturbance, not something the
	// loop creates. Feeding it raw into kd*vel put it straight back into the command (kd 0.15 took
	// pk-pk from 89 to 555 deg), and the integral chases it through (vsp - vel) just the same.
	// Filtering below it lets kd do its job again. The guard keeps the RAW velocity so a real
	// runaway is never hidden behind a filter.
	float vel_lpf_hz = 1.0f;                   // 0 disables; 1 Hz measured best (see above)
	// THE AZIMUTH IS A CONTINUOUSLY ROTATING AXIS, SO THE ERROR HAS TO BE WRAPPED.
	// pos_ comes from the tracker's UNWRAPPED accumulator on purpose — the rate guard, the velocity
	// estimate and the drift accounting all need revolutions rather than a modulo angle. But "hold
	// this heading" is a modulo-360 statement, and servoing the accumulator instead makes the loop
	// try to unwind every revolution the pod has already turned.
	// MEASURED 2026-08-15, and it is the whole reason the proportional path looked unusable: at
	// kp = 0 the loop holds the RATE and the pod drifts to err ~ -2500 deg — seven revolutions —
	// while holding 3/3. Switch kp on and it reads those seven revolutions as the error, demands
	// kp * 2500 deg/s, rails the integral and dies in ~3 s (kp 0.5: 0/3 held, peak |vel| 4019-4331).
	// Wrapped, the same error is at most 180 deg and the same gain asks for something the pod can
	// actually do — and kff is sized so that uff = kff*vsp delivers exactly vsp, so the feedforward
	// stays self-consistent at the smaller numbers.
	bool  wrap_err = true;
	// ABSOLUTE HEADING. Off, the target is "+X from wherever the spin-up happened to end", because
	// poll() re-zeroes `base_` at the handover; the pod wanders +/-10 deg (once 102) before the loop
	// captures, so that origin is not a heading at all. The AS5600 is an ABSOLUTE encoder within one
	// turn and that information was being thrown away — which is fine on a bench and useless on a
	// vehicle, where "hold 90" has to mean the same direction on every power cycle.
	// On, `zero_raw` (a persisted RAW_ANGLE count, settings::azLoadZero) defines 0 deg, begin()
	// REFUSES without one, and the handover no longer re-zeroes the heading — only the integrators,
	// which really are transition artefacts. The error is still wrapped, so the worst move the pod
	// can be asked for at capture is 180 deg, at `vmax` and inside the same rate guards.
	bool    absolute = false;
	int32_t zero_raw = -1;               // RAW count that reads as 0 deg; <0 = not calibrated
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
	// ABSOLUTE, and it has to stay absolute. A gate relative to each motor's own target was tried on
	// 2026-08-15 so the pair could come up already split at the null (low motor ~270 rpm): it fires on
	// the TELEMETRY FLOOR. `Telem.rpm` reads a constant ~182 below the seam, so a 160 rpm threshold is
	// satisfied by a motor that is not turning at all — the handover then fires early, the loop starts
	// from a bad state, and net rotation went from 17 deg to 2868-3506. Come up MATCHED (both clear
	// 400 honestly), then ramp the trim in — see trim_secs.
	float spin_rpm_ok   = 400.0f;    // both must report at least this, FRESH, to count as past the seam
	float spin_ok_frac  = 0.0f;      // 0 = absolute gate only. Kept so the experiment is repeatable.
	float spin_rpm_secs = 8.0f;      // give up if they never get there
	// Seconds of ONE-MOTOR-ONLY running that ends the run. 0 disables. Default is well under the
	// 0.7 s of the measured failure and well over anything a healthy pair shows.
	float spin_asym_secs = 0.5f;
	// SNAP THROUGH THE SEAM INSTEAD OF RAMPING THROUGH IT.
	// Ramping the rpm target makes each ESC cross at the moment ITS OWN command reaches 515, and the
	// two are not calibrated identically (vscale 1.0000 vs 0.9225 on this pod), so the pair straddles
	// the seam — one in 6-step at ~560 rpm, the other still in forced sine — for however long the
	// ramp takes to cover the difference. That straddle is a whole-motor mismatch and no authority
	// can cancel it, because the offset is forced to zero through the crossing (cross_zero_offset).
	// MEASURED 2026-08-16, pre-handover excursion vs ramp rate (encoder only — sub-seam eRPM does
	// not exist, so the rpm columns say nothing here):
	//     slew  150: 4.8 / 8.0 / 10.5 deg   (t_hand 2.90 s — slower ramp, longer to drift)
	//     slew  300: 2.4 / 2.6 / 4.5 deg    (t_hand 1.51 s)  <- the shipped default, and an optimum
	//     slew  900: 1.7 / 16.7 / 32.3 deg  (t_hand 1.13 s)
	//     slew 2000: 0.6 / 32.1 / 33.8 deg  (t_hand 1.15 s)
	// so the rate itself is not the lever: faster shortens the window but enlarges the straddle.
	// Dwell just BELOW the seam instead, then command both motors past it in the SAME poll() tick,
	// so the two DShot frames leave microseconds apart and the straddle is one command edge rather
	// than a ramp interval.
	float snap_rpm  = 0.0f;          // 0 = off. Dwell speed, must be below the seam (~560 rpm here)
	float snap_hold = 0.4f;          // seconds to settle at snap_rpm before the step
	// Bounded azimuth authority BELOW the seam, where the loop used to have none. See spinCorr_().
	float spin_r_max = 0.0f;         // mech rpm, 0 = off (the old "zero through the whole ramp")
	float spin_corr_margin = 0.80f;  // stop correcting once the pair reaches this fraction of the
	                                 // 6-step floor — the crossing itself stays untouched
	// THE VELOCITY LOOPS' SLEW HAS TO BE PHASE-DEPENDENT, because one number cannot serve both
	// jobs. During the transition it sets how gently the pair comes up (high slew = a violent
	// azimuth kick while they are still unmatched); during the hold it sets how fast the azimuth
	// loop can move its authority (low slew = the loop cannot chase the disturbance and the pod
	// walks away). MEASURED 2026-08-15: at slew 1500 every run aborted on the rate guard within
	// 1-2 s; at 300 they survived to ~4 s but then drifted off monotonically because the loop had
	// no bandwidth left. The host has separated these phases since 8/11; the firmware had not.
	float spin_slew = 300.0f;        // rpm/s while coming up
	float hold_slew = 3000.0f;       // rpm/s once both motors are past the seam
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
	// EXPRESS THE NULL AS A SPEED RATIO, NOT AN RPM OFFSET, BECAUSE THAT IS WHAT IT IS.
	// MEASURED 2026-08-15 at low thrust with both motors delivering an identical 310 rpm:
	//   same sign   (-400, -400) -> +1248 deg/s        opposite (-400, +400) -> -1995 / -2005
	// Two equations, two unknowns:  rate = -1.21*wA + 5.24*wB  (deg/s per rpm, signed), and the
	// independent 22-point fit from the same day gave -1.06 / +5.04. So the azimuth is neither the
	// pure sum nor the pure difference: the two coaxial shafts run through gears of DIFFERENT
	// diameter (one shaft is inside the other), so their reductions differ by construction and the
	// weighting is ~4.3 : 1. That is geometry, not a fault, and `mix_swap` is NOT the answer.
	// The null is therefore at a fixed SPEED RATIO |A| / |B| = null_ratio, i.e.
	//     r_trim = thrust * (1 - null_ratio) / (1 + null_ratio)
	// which SCALES WITH THRUST — so a fixed `rpm_trim` is only ever right at one thrust setting, and
	// wrong the moment thrust_rpm ramps. Every trim tried before this (-137/-300/-450/-600) was a
	// ratio of 1.5/2.2/3.0/3.7; the measured null is 4.2-4.3 and the sweep had stopped short of it.
	// 0 disables and falls back to the fixed rpm_trim.
	float null_ratio    = 0.0f;      // |A| / |B| that holds the azimuth. 0 = use rpm_trim
	float rpm_offset_max = 400.0f;   // authority ceiling, mech rpm — see rLimit_()
	// AUTHORITY IS BOUNDED BY THE SEAM, NOT BY A HAND-SET CONSTANT.
	// r splits the pair: |A| = spin_rpm - r, |B| = spin_rpm + r. A fixed rpm_offset_max knows
	// nothing about spin_rpm, so the same 400 is timid at spin_rpm 1600 and drives the low motor
	// onto the 6-step seam at spin_rpm 800. MEASURED 2026-08-15, commanded vs delivered on the low
	// side: 1200 -> 1168, 1000 -> 1030, 850 -> 835, 700 -> 550, 650 -> 354. Below ~800 rpm the ESC
	// simply does not hold the speed it is given, and the resulting whole-motor mismatch is the
	// largest disturbance this axis ever sees — it is what every spin-up runaway in this file is.
	// So bound r by the LOW motor's floor and let the limit scale with thrust for free.
	float rpm_floor = 800.0f;        // neither motor may be commanded below this
	// CROSS THE SEAM WITH THE PAIR IDENTICAL — the rpm path's missing half of a rule the raw path
	// has had since 2026-08-14 (see the s == 0 block in poll()). There the lesson was measured and
	// written down: while the carrier is near the seam, a split of 2*|trim| leaves one motor in
	// 6-step and the other still in forced sine, which drove the azimuth at -944 deg/s — a whole
	// motor of mismatch no authority can cancel. `applyRpm_` never got the same treatment, so
	// rpm_trim was applied from the very first tick, tearing the pair apart while both were still
	// ramping. MEASURED 2026-08-15: at spin_rpm 1200 with rpm_trim -450 the pair is commanded
	// 1650/750 from t=0 and 3/3 runs died at 1.5 s, mid-ramp. Holding the DELIVERED offset at zero
	// until both motors report up makes them cross together; the trim and the loop arrive after.
	bool  cross_zero_offset = true;
	// --- WHICH CHANNEL IS WHICH. See docs/azimuth-closed-loop.md §12.
	// The CAD says the two coaxial shafts feed a bevel differential whose PLANETS are the propeller
	// shafts and whose CARRIER is the azimuth body, so in motor space
	//     azimuth   = (wU + wL)/2     <- COMMON mode
	//     propeller = (wU - wL)/2     <- ANTI-symmetric mode
	// and holding the angle means running the two motors in OPPOSITE directions, with the thrust
	// being exactly that opposition. `applyRpm_` has always put `spin_rpm` in the common mode and
	// `r` in the anti-symmetric one, i.e. the roles swapped — which is only correct if motor B is
	// physically mirrored (the `sign_b = -1` assumption). The CAD shows two like motors side by
	// side, each through ONE external spur mesh, so it is not.
	// MEASURED 2026-08-15, and it is what this predicts: at MATCHED commands the encoder tracks the
	// motor speed at 0.964 / 0.965 / 0.954 (384 / 806 / 1209 rpm) — the pod was being commanded to
	// spin at exactly the requested "thrust", with the props nearly still — and no value of `r` ever
	// nulled the azimuth at any thrust, because `r` does not reach the azimuth at all.
	// DEFAULT OFF: one measurement (ch1 Bidirectional-Reversed) does not fit yet, and this commands
	// one ESC in REVERSE, where the crossover on this stack is uncharacterised. Verify on the bench
	// with `encwatch.py --rpm -800 --rpm-b 800` first — opposite commands must give spinning props
	// and a still pod.
	bool  mix_swap = false;
	// --- THRUST RAMP. Get into 6-step at a speed the CROSSING survives, capture the angle, and only
	// then push the thrust up — with the loop already closed. `spin_rpm` is a single constant applied
	// from the first tick, so asking for the operating thrust means asking the SEAM CROSSING to
	// happen at that thrust, and the crossing transient scales with it: MEASURED 2026-08-15, the
	// same configuration reaches the guard at 2735-2901 deg/s at spin_rpm 800 and 5983-6351 at
	// spin_rpm 2000, dying 1.5-3.4 s in before the loop has done anything. Nothing about the 6-step
	// region itself is the problem — it is how the pod gets there.
	// The authority follows for free: rLimit_() is min(rpm_offset_max, thrust - rpm_floor), so every
	// rpm of thrust the ramp adds is an rpm of azimuth authority the loop did not have at capture.
	float thrust_rpm  = 0.0f;        // 0 = disabled: stay at spin_rpm. Else ramp to this after handover
	float thrust_secs = 6.0f;        // seconds for spin_rpm -> thrust_rpm
	float trim_secs   = 0.5f;        // seconds to fade the null feedforward in after the handover

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

	// (-180, +180]. fmodf rather than a while-loop: after a runaway the raw error is thousands of
	// degrees, and a loop that iterates once per revolution does not belong in a 1 kHz tick.
	static float wrapDeg(float d) {
		d = fmodf(d + 180.0f, 360.0f);
		if (d < 0.0f) d += 360.0f;
		return d - 180.0f;
	}

	void bind(Thruster* a, Thruster* b, As5600Tracker* enc) { a_ = a; b_ = b; enc_ = enc; }

	// Declare the pod's CURRENT physical heading to be `deg`. The caller persists the returned raw
	// count; nothing here writes to flash. Not `deg = 0` only, because the calibration pose is
	// whatever the rig can be held at repeatably, which is rarely dead ahead.
	bool setZeroHere(float deg) {
		if (!enc_ || !enc_->present()) return false;
		int32_t c = (int32_t)COUNTS_PER_REV;
		int32_t z = (int32_t)enc_->raw() - (int32_t)lrintf(deg * COUNTS_PER_REV / 360.0f);
		zero_raw = ((z % c) + c) % c;
		return true;
	}

	// The hull-referenced heading right now, in (-180,+180]. NAN when there is no reference — a
	// caller must never substitute 0 for it.
	float heading() const {
		if (zero_raw < 0 || !enc_ || !enc_->present()) return NAN;
		return wrapDeg(((float)((int32_t)enc_->raw() - zero_raw)) * 360.0f / COUNTS_PER_REV);
	}
	bool  absoluteReady() const { return !absolute || zero_raw >= 0; }

	bool begin(float target_deg) {
		if (!enc_ || !enc_->present()) return false;
		if (!absoluteReady()) return false;           // no reference => refuse, never fall back
		// pos_ has to READ as the hull heading and then stay UNWRAPPED (the rate guard and the drift
		// accounting count revolutions), so the accumulator's origin is biased by the heading the
		// absolute encoder reports right now instead of being pinned to "here is zero".
		base_   = absolute ? enc_->accum() - (int32_t)lrintf(heading() * COUNTS_PER_REV / 360.0f)
		                   : enc_->accum();
		target_ = target_deg;
		ivel_ = 0.0f; vel_ = 0.0f; velf_ = 0.0f; pos_ = 0.0f; over_ = 0; over_s_ = 0.0f; hand_us_ = 0;
		ipos_ = 0.0f;
		// poll() recomputes pos_ on its first tick anyway; set it here so `az status` between `az on`
		// and that tick reports the heading rather than a 0 that would read as "already on target".
		pos_ = (enc_->accum() - base_) * 360.0f / COUNTS_PER_REV;
		traceClear();
		last_us_ = t0_us_ = micros();
		spinning_ = true;
		thr_ = spin_rpm;
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
		ivel_ = 0.0f; vel_ = 0.0f; velf_ = 0.0f; pos_ = 0.0f; over_ = 0; over_s_ = 0.0f; hand_us_ = 0;
		ipos_ = 0.0f;
		traceClear();
		step_s_       = constrain(s_cmd, -smax, smax);
		step_armed_   = false; step_base_u_ = 0.0f; thr_ = spin_rpm;
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

		// Wide while the pair is crossing the seam (and for spin_guard_secs after it reports up),
		// tight for the hold. fmaxf so a caller who lowers spin_max_rate below max_rate cannot
		// accidentally make the transition guard TIGHTER than the one it exists to relax.
		bool  in_spin = rpm_phase_
		             || (hand_us_ && (uint32_t)(now - hand_us_) < (uint32_t)(spin_guard_secs * 1e6f));
		float rate_lim = in_spin ? fmaxf(max_rate, spin_max_rate) : max_rate;
		if (fabsf(vel_) > rate_lim) over_s_ += dt;
		else                        over_s_ = fmaxf(0.0f, over_s_ - dt);
		if (over_s_ > over_secs) { stop_why_ = STOP_RATE; stop(); return; }
		if (fabsf(vel_) > hard_rate) { if (++over_ >= 3) { stop_why_ = STOP_HARD_RATE; stop(); return; } }
		else over_ = 0;

		// The loop runs from the first tick — including through the spin-up, which is the whole
		// point (the host reached the same conclusion in 5d59aea). What the spin-up window gates is
		// only WHEN the heading is zeroed: drift accrued before both motors are actually up is a
		// transition artefact, not tracking error.
		if (rpm_phase_) {
			snapPoll_(now);
			escs::Telem ta, tb; uint32_t ms = millis();
			bool oka = a_->tele(ta) && ta.valid && ta.rpm >= (uint32_t)spin_rpm_ok
			           && (uint32_t)(ms - ta.rpmStampMs) < TELE_FRESH_MS;
			bool okb = b_->tele(tb) && tb.valid && tb.rpm >= (uint32_t)spin_rpm_ok
			           && (uint32_t)(ms - tb.rpmStampMs) < TELE_FRESH_MS;
			if (oka && okb) {
				rpm_phase_ = false;
				hand_us_ = now ? now : 1u;          // 0 is the "never handed over" sentinel
				// THE REBASE IS WHAT MAKES THE TARGET RELATIVE. Re-zeroing `base_` here declares
				// "wherever the spin-up ended is 0 deg", which is why `--deg 90` meant +90 from an
				// origin that itself wandered +/-10 deg run to run. In absolute mode the origin is
				// the hull, so only the genuine transition artefacts are cleared — the integrators
				// and the filtered velocity — and the heading survives the handover.
				if (!absolute) { base_ = enc_->accum(); pos_ = 0.0f; }
				ivel_ = 0.0f; velf_ = 0.0f; ipos_ = 0.0f;
				t0_us_ = now; spinning_ = false;
				a_->vc.slew_rpm_s = hold_slew;      // hand the loop its bandwidth back
				b_->vc.slew_rpm_s = hold_slew;
				if (mode_ == STEP) { step_begin_us_ = now + step_pre_us_;
				                     step_end_us_   = step_begin_us_ + step_run_us_; }
			} else if ((uint32_t)(now - rpm_t0_) > (uint32_t)(spin_rpm_secs * 1e6f)) {
				stop_why_ = STOP_SPIN_TIMEOUT; stop(); return;
			}
			// ONE MOTOR RUNNING WHILE THE OTHER HAS NOT CAUGHT is the only spin-up failure that
			// matters, and no gain can answer it: through the crossing the offset is forced to zero,
			// so a whole-motor mismatch turns the pod at will. MEASURED 2026-08-16 — B reported 0
			// while A ran 197-449 rpm for ~0.7 s, and the pod went to -737 deg (two revolutions) at
			// ~1000 deg/s before B caught up. Below the seam there is NO eRPM, so this is a coarse
			// test on the little telemetry that exists: one side clearly turning, the other
			// reporting nothing, for longer than a start normally takes to become symmetric (good
			// runs track within ~20 rpm the whole way).
			bool a_run  = ta.valid && (uint32_t)(ms - ta.rpmStampMs) < TELE_FRESH_MS
			              && ta.rpm >= (uint32_t)(spin_rpm_ok * 0.5f);
			bool b_run  = tb.valid && (uint32_t)(ms - tb.rpmStampMs) < TELE_FRESH_MS
			              && tb.rpm >= (uint32_t)(spin_rpm_ok * 0.5f);
			bool a_dead = !(ta.valid && (uint32_t)(ms - ta.rpmStampMs) < TELE_FRESH_MS) || ta.rpm < 100;
			bool b_dead = !(tb.valid && (uint32_t)(ms - tb.rpmStampMs) < TELE_FRESH_MS) || tb.rpm < 100;
			if ((a_run && b_dead) || (b_run && a_dead)) asym_s_ += dt;
			else                                        asym_s_ = 0.0f;
			if (spin_asym_secs > 0.0f && asym_s_ > spin_asym_secs) {
				stop_why_ = STOP_ASYMMETRIC; stop(); return;
			}
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
		float u_max  = rpm_drive ? (rLimit_() / fmaxf(rpm_per_count, 1e-3f))
		                         : fmaxf(1.0f, fminf(spinning_ ? spin_smax : smax, room_u));

		now_us_ = now;
		thr_ = thrustNow_(now);
		float a_lp = (vel_lpf_hz > 0.0f) ? (1.0f - expf(-2.0f * 3.14159265f * vel_lpf_hz * dt)) : 1.0f;
		velf_ += a_lp * (vel_ - velf_);

		float err = wrap_err ? wrapDeg(target_ - pos_) : (target_ - pos_);
		// Hold the integrator while the pair is still crossing the seam: the heading there is a
		// transition artefact, exactly as the rebase at the handover assumes.
		if (ki_pos > 0.0f && !rpm_phase_) ipos_ = constrain(ipos_ + err * dt, -ipos_max, ipos_max);
		float vsp = kp * err + ki_pos * ipos_ - kd * velf_;
		vsp = constrain(vsp, -vmax, vmax);
		// Back-calculate the position integrator against the SAME clamp, so a saturated vsp does not
		// keep winding it (the velocity path already does this against u_max).
		if (ki_pos > 0.0f) {
			float unsat = kp * err + ki_pos * ipos_ - kd * velf_;
			if (unsat != vsp) ipos_ = constrain((vsp - kp * err + kd * velf_) / ki_pos,
			                                    -ipos_max, ipos_max);
		}

		float uff = kff * vsp;
		// CLAMP THE VELOCITY ERROR BEFORE INTEGRATING IT. vsp is bounded by vmax but vel_ is the raw
		// plant velocity, which reaches thousands of deg/s in a transient the loop never asked for —
		// so (vsp - vel_) becomes essentially -vel_ and the integral degenerates into an integrator
		// of VELOCITY, slamming to the rail in milliseconds. Bounding the error keeps the integral
		// doing its actual job (cancelling the drifting null) instead of chasing a transient it
		// cannot influence. verr_clamp <= 0 disables, restoring the old behaviour for comparison.
		float verr = vsp - velf_;
		if (verr_clamp > 0.0f) verr = constrain(verr, -verr_clamp, verr_clamp);
		// NO INTEGRAL BELOW THE SEAM. There `s` has almost no authority over the azimuth, so the
		// loop can see an error it cannot act on; an integrator left running there winds up and
		// then dumps the whole windup the instant authority appears. Held at zero instead.
		if (below) ivel_ = 0.0f; else ivel_ += ki * verr * dt;
		// No windup against an offset that is not being delivered (see cross_zero_offset).
		if (rpm_drive && cross_zero_offset && rpm_phase_) ivel_ = 0.0f;
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
	// WHEN THE PAIR REPORTED UP. Everything before it is open-loop drift (the offset is forced to
	// zero through the crossing) and everything after is the loop tracking, so a trace that cannot
	// say where the boundary is cannot separate "the pod wandered" from "the pod was slewing to
	// target" — which is exactly the mistake made on 2026-08-16. 0 = never handed over.
	uint32_t handUs() const { return hand_us_; }
	uint8_t  stopWhy() const { return stop_why_; }
	static const char* stopWhyName(uint8_t w) {
		switch (w) { case STOP_RATE: return "rate"; case STOP_HARD_RATE: return "hardrate";
		            case STOP_SPIN_TIMEOUT: return "spintimeout"; case STOP_ASYMMETRIC: return "asymstart";
		            case STOP_COMMAND: return "command"; default: return "none"; }
	}
	float pos()    const { return pos_; }
	float target() const { return target_; }
	float vel()    const { return vel_; }
	float s()      const { return s_; }
	float carrierNow() const { return c_; }
	bool  spinning()   const { return spinning_; }

private:
	enum Mode : uint8_t { OFF, HOLD, STEP };
	// WHY THE RUN ENDED. A stop with no reason attached made every abort look the same from the
	// host, and "it just stopped" is the one thing a bench log must never say.
	enum StopWhy : uint8_t { STOP_NONE, STOP_RATE, STOP_HARD_RATE, STOP_SPIN_TIMEOUT,
	                        STOP_ASYMMETRIC, STOP_COMMAND };

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
	// THE OFFSET IS FORCED TO ZERO THROUGH THE CROSSING — but only the CROSSING needs it.
	// Below the seam the ESC is in forced sine and there is NO eRPM at all, so each velocity loop is
	// running on its feed-forward curve and neither motor's actual speed is regulated. That is where
	// the pre-handover drift comes from, and on a late/asymmetric start it reached -737 deg
	// (measured 2026-08-16, handover 2.23 s). The one sensor that works in that window is the
	// azimuth encoder, so let the loop use it — bounded, and ONLY while both motors stay clear of
	// the seam, so the pair can never straddle it (the -944 deg/s failure of 2026-08-14).
	// `senseRpmFloor()` is the profile's own 6-step landing, so this scales with the motor/voltage
	// instead of hard-coding 560 rpm.
	float spinCorr_(float u) const {
		if (spin_r_max <= 0.0f || !a_ || !b_) return 0.0f;
		float floor_rpm = a_->senseRpmFloor();
		if (floor_rpm <= 0.0f) return 0.0f;
		float hi = fmaxf(fabsf(a_->vc.setpoint()), fabsf(b_->vc.setpoint()));
		if (hi + spin_r_max >= floor_rpm * spin_corr_margin) return 0.0f;   // too close to the seam
		return constrain(u * rpm_per_count, -spin_r_max, spin_r_max);
	}

	void applyRpm_(float u) {
		float sgn = (carrier < 0.0f) ? -1.0f : 1.0f;
		// THE TRIM IS THE OPERATING POINT; THE CLAMP BOUNDS THE LOOP AROUND IT.
		// This used to be constrain(u*rpm_per_count + rpm_trim, +/-lim), i.e. the feedforward and the
		// authority shared one budget — so setting rpm_offset_max to "how far the loop may move"
		// silently clipped the trim as well. MEASURED 2026-08-15: the null needs r = -434 at
		// spin_rpm 700, and with rpm_offset_max 150 the delivered r could never go past -150; the pod
		// turned 9900 deg in 12 s. Giving the clamp room for BOTH (584) took it to 256-970 deg.
		// Separating them removes the trap: `rpm_offset_max` now means what its name says.
		float lim = rLimit_();
		// RAMP THE TRIM IN, do not step it. At the handover the feedforward jumps from 0 to the null
		// (-433 rpm at spin_rpm 700, i.e. the pair splits 4:1 in one tick), and a step that size is
		// itself a disturbance the loop then has to clean up. `trim_secs` spreads it over the window
		// right after the pair reports up, while the loop is already closed and can follow it.
		float trim = trimFor_(thr_);
		if (trim_secs > 1e-3f && hand_us_) {
			float el = (float)(uint32_t)(now_us_ - hand_us_) * 1e-6f;
			trim *= fminf(1.0f, el / trim_secs);
		}
		float r   = trim + constrain(u * rpm_per_count, -lim, lim);
		// ZERO while the pair is crossing the seam, and it has to be zero. Under the real kinematics
		// a matched pair IS maximum azimuth rate, so this looks like the worst possible choice — and
		// delivering the trim instead was tried on 2026-08-15. It fails for a different reason: the
		// trim splits the pair ~4:1, the low motor sits at ~270 rpm, and the handover gate needs it
		// at 400. The pair never reports up honestly, and net rotation went from 17 deg to ~1400.
		// The pod does rotate through the crossing; `trim_secs` is what shortens the recovery.
		if (cross_zero_offset && rpm_phase_) r = spinCorr_(u);
		if (mix_swap) {
			// r is the COMMON offset (zero holds the angle); the thrust is the opposition.
			// Note this makes cross_zero_offset exactly right for the crossing: r = 0 gives pure
			// thrust and a stationary pod while both motors come up through the seam.
			a_->setRpmTarget(+sgn * thr_ + r);
			b_->setRpmTarget(-sgn * thr_ + r);
			s_ = u; c_ = thr_;
			return;
		}
		// PARALLEL TO THE RAW MIX, and the parallel has to be exact: raw is a = c + s, b = c - s,
		// so with a NEGATIVE carrier a more negative s makes |a| LARGER. Writing sgn*(R + r) puts
		// the sign outside and inverts that — the loop then drives the error instead of correcting
		// it. Measured 2026-08-14: u pinned at its -28.6 rail while pos ran 85 -> 599 deg and
		// accelerated to 4048 deg/s. Same class of error as the sign_b placeholder, new coordinate.
		a_->setRpmTarget(sgn * thr_ + r);
		b_->setRpmTarget(sgn * thr_ - r);
		s_ = u; c_ = thr_;                                  // the trace records the LIVE thrust
	}

	// The feedforward offset for a given thrust — the null, expressed however the caller set it up.
	float trimFor_(float thrust) const {
		return (null_ratio > 0.0f) ? fabsf(thrust) * (1.0f - null_ratio) / (1.0f + null_ratio)
		                           : rpm_trim;
	}

	// How far the pair may be split, in mech rpm. Zero when the thrust setting cannot even reach
	// the floor — the caller then has no azimuth authority at all, which is the honest answer and
	// far better than silently commanding a motor into forced sine.
	float rLimit_() const {
		return fminf(rpm_offset_max, fmaxf(0.0f, fabsf(thr_) - rpm_floor));
	}

	// The thrust the pair is actually being held at RIGHT NOW: spin_rpm until the handover, then a
	// linear ramp to thrust_rpm. Before the handover there is no capture to protect, so the ramp
	// cannot start — that is the whole point of it.
	float thrustNow_(uint32_t now) const {
		if (thrust_rpm <= 0.0f || hand_us_ == 0) return spin_rpm;
		float el = (float)(uint32_t)(now - hand_us_) * 1e-6f;
		float f  = (thrust_secs > 1e-3f) ? fminf(1.0f, el / thrust_secs) : 1.0f;
		return spin_rpm + (thrust_rpm - spin_rpm) * f;
	}

	void beginRpmSpinup_() {
		rpm_phase_ = true; rpm_t0_ = micros();
		snap_us_ = 0; snap_done_ = (snap_rpm <= 0.0f); asym_s_ = 0.0f; stop_why_ = STOP_NONE;
		if (a_) a_->vc.slew_rpm_s = spin_slew;
		if (b_) b_->vc.slew_rpm_s = spin_slew;
		float sgn = (carrier < 0.0f) ? -1.0f : 1.0f;
		// Matched, because that is the only state the handover gate can honestly detect (see
		// spin_rpm_ok). The pod does rotate during this window; `trim_secs` is what keeps it short.
		float tgt = snap_done_ ? spin_rpm : snap_rpm;      // dwell below the seam first, if enabled
		if (a_) a_->setRpm(sgn * tgt);
		if (b_) b_->setRpm(mix_swap ? -sgn * tgt : sgn * tgt);
	}

	// The seam snap, driven from poll() while rpm_phase_ is still true. Timed rather than gated on
	// telemetry ON PURPOSE: below the seam the ESC is in forced sine and there IS no eRPM, so no
	// reading from that window can say when the dwell has settled. The ramp is slew-limited and
	// therefore deterministic — snap_rpm/spin_slew seconds — which is a thing the Pico knows.
	void snapPoll_(uint32_t now) {
		if (snap_done_) return;
		float ramp_us = (spin_slew > 1.0f ? snap_rpm / spin_slew : 0.0f) * 1e6f;
		if (!snap_us_) {
			if ((float)(uint32_t)(now - rpm_t0_) >= ramp_us) snap_us_ = now ? now : 1u;
			return;
		}
		if ((float)(uint32_t)(now - snap_us_) < snap_hold * 1e6f) return;
		// BOTH IN THIS TICK. The slew is raised first, so neither ESC's own ramp re-introduces the
		// difference the dwell just removed.
		float sgn = (carrier < 0.0f) ? -1.0f : 1.0f;
		if (a_) { a_->vc.slew_rpm_s = hold_slew; a_->setRpm(sgn * spin_rpm); }
		if (b_) { b_->vc.slew_rpm_s = hold_slew; b_->setRpm(mix_swap ? -sgn * spin_rpm : sgn * spin_rpm); }
		snap_done_ = true;
	}

	// -1 when the number would not be a MEASUREMENT. Telem.rpm is unsigned and KEEPS its last 6-step
	// value after the ESC drops back to forced sine (see the Telem comment in esc_session.h), so
	// recording it unqualified writes a stale reading into the one channel the trace ring exists to
	// answer with. The handover gate in poll() has always applied this freshness test; record_ had
	// not — so a trace could show two perfectly matched motors while one had actually fallen out of
	// 6-step, which is exactly the reading that has to be trusted when the pod runs away.
	int16_t freshRpm_(Thruster* t, uint32_t ms) {
		escs::Telem tl;
		if (!t || !t->tele(tl) || !tl.valid) return -1;
		if ((uint32_t)(ms - tl.rpmStampMs) >= TELE_FRESH_MS) return -1;
		return (int16_t)min<uint32_t>(tl.rpm, 32000);
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
		uint32_t ms = millis();
		x.ra = freshRpm_(a_, ms);
		x.rb = freshRpm_(b_, ms);
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
	float    over_s_ = 0.0f;                     // leaky time-above-max_rate budget
	float    target_ = 0, pos_ = 0, vel_ = 0, velf_ = 0, ivel_ = 0, s_ = 0, c_ = 0;
	float    step_s_ = 0, step_base_u_ = 0;
	bool     step_armed_ = false, spinning_ = false, rpm_phase_ = false;
	uint32_t rpm_t0_ = 0, step_pre_us_ = 0, step_run_us_ = 0;
	uint8_t  stop_why_ = STOP_NONE;
	float    asym_s_ = 0.0f;                     // seconds of one-motor-only running
	uint32_t snap_us_ = 0;                       // when the pre-seam dwell began; 0 = still ramping
	bool     snap_done_ = true;                  // true also when the snap is disabled (snap_rpm 0)
	uint32_t hand_us_ = 0;                       // when the pair reported up; 0 = not yet
	float    thr_ = 0.0f;                        // live thrust, see thrustNow_()
	float    ipos_ = 0.0f;                       // position integrator, deg-seconds
	uint32_t now_us_ = 0;                        // this tick, for applyRpm_'s trim ramp

	Sample   trace_[TRACE_N];
	uint16_t tr_head_ = 0, tr_count_ = 0, decim_ = 5, dec_n_ = 0;
};
