// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// profiles — per-motor calibrated speed curves (thrust <-> mech RPM) + PI gains for closed-loop RPM.
// The real curves are GENERATED from the calibrated YAML profiles in host/profiles/ (the single
// source of truth) into profiles_gen.h by host/gen_profile_header.py — never hand-typed here, so the
// firmware numbers can't drift from the bench calibration. After a velcal, regenerate with:
//     python3 host/gen_profile_header.py
// and each `vel_<name>.yaml` becomes `profiles::M_<NAME>` (+ `M_<NAME>_GAINS`). Pass one to a Thruster.
#pragma once
#include "vel_control.h"
#include "profiles_gen.h"   // GENERATED: profiles::M_930KV_12N14P_6STEP, ..._GAINS, etc.

namespace profiles {

// Hand-written fallback: a trivial linear curve so a Thruster with no calibrated profile still
// constructs (its RPM mode is uncalibrated; fine for RAW-only ESCs). thrust 0..1000 -> 0..6000 mech,
// no crossover. Everything else lives in profiles_gen.h.
static const vel::CurvePoint CURVE_LINEAR[] = { {0.0f, 0.0f}, {1000.0f, 6000.0f} };
static const vel::SpeedProfile M_LINEAR(CURVE_LINEAR, 2, /*pole_pairs=*/7);

}  // namespace profiles
