// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// profiles — per-motor calibrated speed curves (thrust <-> mech RPM) for the closed-loop RPM mode.
// These are the FF seeds the velocity controller inverts; the PI trim corrects residual error at
// runtime. A curve is per MOTOR MODEL (bench-measured once, reused), so it lives here, separate from
// the Thruster class and chosen by the declaring main (one Thruster gets one profile). A future
// host-side codegen can emit more of these from the YAML velcal profiles in host/profiles/.
#pragma once
#include "vel_control.h"

namespace profiles {

// --- 930KV 12N14P (bench-measured 2026-07-16/17) ---------------------------------------------------
// Sine seed (<=250) + 6-step points; the seam gap (sine caps ~84, 6-step starts ~2576) is baked in.
static const vel::CurvePoint CURVE_930KV[] = {
	{  0,    0.0f}, { 60,   20.3f}, {108,   35.6f}, {155,   51.8f}, {202,   67.6f}, {250,   83.6f},
	{620, 2576.0f}, {700, 4645.0f}, {800, 7173.0f}, {900, 9447.0f}, {1000, 11000.0f},
};
static const vel::Regime CURVE_930KV_REGIMES[] = {
	vel::Regime::SINE, vel::Regime::SINE, vel::Regime::SINE, vel::Regime::SINE, vel::Regime::SINE,
	vel::Regime::SINE, vel::Regime::LINE, vel::Regime::LINE, vel::Regime::LINE, vel::Regime::LINE,
	vel::Regime::LINE,
};
static const vel::Crossover CROSSOVER_930KV = { 1500.0f, 1350.0f };   // up_erpm / dn_erpm (config on-bench)

// The ready-to-use profile object. Pass &profiles::M_930KV to a Thruster for closed-loop RPM control.
static const vel::SpeedProfile M_930KV(
	CURVE_930KV, (int)(sizeof(CURVE_930KV) / sizeof(CURVE_930KV[0])),
	/*pole_pairs=*/7, &CROSSOVER_930KV, CURVE_930KV_REGIMES);

// --- Fallback: a trivial linear curve so a Thruster with no calibrated profile still constructs (its
//     RPM mode is uncalibrated / for RAW-only ESCs). thrust 0..1000 -> 0..6000 mech, no crossover. ---
static const vel::CurvePoint CURVE_LINEAR[] = { {0, 0.0f}, {1000, 6000.0f} };
static const vel::SpeedProfile M_LINEAR(CURVE_LINEAR, 2, /*pole_pairs=*/7);

}  // namespace profiles
