# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""Tests for gen_profile_header.py — the YAML-profile -> C++ header code generator.

Guards the two things a bad generator would break: (1) the emitted C++ is well-formed (valid float
literals, matching array sizes, the vel::SpeedProfile/Gains it should), and (2) the checked-in
src/apps/profiles_gen.h is not stale w.r.t. host/profiles/ (so the firmware curve tracks the YAML).
"""
import os
import re
import sys

HOST = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HOST)
import gen_profile_header as gen  # noqa: E402
from pico_esc.velocity import SpeedProfile  # noqa: E402

YAML_6STEP = os.path.join(HOST, "profiles", "vel_930kv_12n14p_6step.yaml")


def test_cident_from_filename_no_collision():
    # bench / sim / 6step share the motor field but must get distinct identifiers (filename-based).
    a = gen._cident("x/vel_930kv_12n14p_6step.yaml")
    b = gen._cident("x/vel_930kv_12n14p_bench.yaml")
    assert a == "930KV_12N14P_6STEP" and b == "930KV_12N14P_BENCH" and a != b


def test_float_literals_are_valid():
    # every float literal must carry a '.' — `2576f` (f on an integer constant) would not compile.
    text = gen.generate([YAML_6STEP])
    lits = re.findall(r"[-\d][\d.eE+-]*f\b", text)
    assert lits, "no float literals emitted"
    for lit in lits:
        assert "." in lit or "e" in lit or "E" in lit, f"invalid float literal {lit!r}"


def test_emitted_curve_matches_profile():
    p = SpeedProfile.load(YAML_6STEP)
    text = gen.generate([YAML_6STEP])
    # the CurvePoint array has exactly the profile's points, and the SpeedProfile passes the count.
    assert "CURVE_930KV_12N14P_6STEP[]" in text
    assert text.count("{", text.index("CURVE_930KV_12N14P_6STEP[]")) >= len(p.points)
    assert f"CURVE_930KV_12N14P_6STEP, {len(p.points)}," in text
    # crossover + gains round-trip
    assert "CROSSOVER_930KV_12N14P_6STEP = { 1500.0f, 1350.0f }" in text
    assert "M_930KV_12N14P_6STEP_GAINS = { 0.03f, 0.12f, 400.0f, 0.3f }" in text
    # pole_pairs + down_catch (930KV weak BEMF -> false)
    assert "/*pole_pairs=*/7," in text and text.rstrip().endswith("}  // namespace profiles")
    assert "CROSSOVER_930KV_12N14P_6STEP, CURVE_930KV_12N14P_6STEP_REGIMES, false)" in text


def test_no_crossover_profile_emits_nullptr():
    # the sine-only bench profile has no crossover/regimes -> nullptr, and no _GAINS only if no control.
    bench = os.path.join(HOST, "profiles", "vel_930kv_12n14p_bench.yaml")
    text = gen.generate([bench])
    assert "nullptr, nullptr, false)" in text


def test_checked_in_header_is_not_stale():
    # profiles_gen.h must equal a fresh generation from all host/profiles/vel_*.yaml.
    import glob
    paths = sorted(glob.glob(gen.DEFAULT_GLOB))
    fresh = gen.generate(paths)
    on_disk = open(gen.DEFAULT_OUT, encoding="utf-8").read()
    assert on_disk == fresh, "src/apps/profiles_gen.h is stale — run: python3 host/gen_profile_header.py"
