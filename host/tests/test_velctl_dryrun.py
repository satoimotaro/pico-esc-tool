# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""velctl / velcal CLI dry-run smoke tests: the wrappers must run standalone, never open a
serial port in --dry-run, span both regimes, and always disarm.
"""
import csv
import os
import subprocess
import sys

import pytest

HOST = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _run(*args, expect=0):
    r = subprocess.run([sys.executable, *args], cwd=HOST,
                       capture_output=True, text=True, timeout=120)
    assert r.returncode == expect, f"{args} -> rc={r.returncode}\n{r.stdout}\n{r.stderr}"
    return r


def test_velctl_below_seam(tmp_path):
    out = tmp_path / "v.csv"
    r = _run("velctl.py", "speed", "--rpm", "100", "--dry-run", "--seed", "1234",
             "--secs", "2", "--csv", str(out))
    assert "# DRY-RUN: SimEncEscHost (no serial port opened)" in r.stdout
    assert "regime: sine" in r.stdout
    assert "# exit reason: completed" in r.stdout
    rows = list(csv.reader(open(out)))
    assert rows[0] == ["t", "rpm_setpoint", "rpm_slewed", "thrust", "temp"]
    # slew: the first slewed setpoint is well below the 100 RPM target (gentle start)
    assert float(rows[1][2]) < 20
    assert float(rows[-1][2]) == 100.0


def test_velctl_above_seam_crossover(tmp_path):
    out = tmp_path / "v.csv"
    r = _run("velctl.py", "speed", "--rpm", "320", "--crossover", "--dry-run",
             "--seed", "1234", "--secs", "3", "--csv", str(out))
    assert "regime: line" in r.stdout
    assert "crossover enabled" in r.stdout
    assert "# exit reason: completed" in r.stdout


def test_velctl_encoder_verify_column(tmp_path):
    out = tmp_path / "v.csv"
    _run("velctl.py", "speed", "--rpm", "100", "--encoder", "--dry-run",
         "--seed", "1234", "--secs", "2", "--csv", str(out))
    rows = list(csv.reader(open(out)))
    assert rows[0][-1] == "enc_rpm"


def test_velcal_dry_run_spans_both_regimes(tmp_path):
    out = tmp_path / "p.yaml"
    r = _run("velcal.py", "--dry-run", "--crossover-erpm", "2100,1600",
             "--seed", "1234", "--profile-out", str(out))
    assert "# DRY-RUN: SimEncEscHost" in r.stdout
    assert "wrote profile" in r.stdout
    # profile parses, is monotonic, and spans the seam (a big jump in the middle)
    from pico_esc import SpeedProfile
    p = SpeedProfile.load(str(out))
    rpms = [r for _, r in p.points]
    assert rpms == sorted(rpms)                        # non-decreasing (monotonic)
    assert p.max_rpm > 400                             # reached the load-line regime
    assert p.crossover["bytes"] == [54, 195]


def test_velcal_profile_records_per_point_regime(tmp_path):
    out = tmp_path / "p.yaml"
    _run("velcal.py", "--dry-run", "--crossover-erpm", "2100,1600",
         "--seed", "1234", "--profile-out", str(out))
    from pico_esc import SpeedProfile
    p = SpeedProfile.load(str(out))
    assert p.regimes is not None and len(p.regimes) == len(p.points)
    assert set(p.regimes) == {"sine", "line"}          # spans both regimes
    # the believed regime flips sine->line exactly where the rpm jumps (auditable seam)
    seam = p.regimes.index("line")
    assert p.regimes[seam - 1] == "sine"
    assert p.points[seam][1] > p.points[seam - 1][1] * 1.8


def test_velcal_narrow_range_dedupes_no_crash(tmp_path):
    # a narrow min..max over many points collapses integer thrusts to duplicates; the sweep
    # must dedupe and still write a valid (strictly-increasing-thrust) profile, not crash.
    out = tmp_path / "p.yaml"
    r = _run("velcal.py", "--dry-run", "--min-thrust", "998", "--max-thrust", "1000",
             "--points", "12", "--seed", "1234", "--profile-out", str(out))
    assert "wrote profile" in r.stdout
    from pico_esc import SpeedProfile
    p = SpeedProfile.load(str(out))
    thrusts = [t for t, _ in p.points]
    assert thrusts == sorted(set(thrusts))             # strictly increasing, no dupes


def test_velcal_rejects_out_of_band_crossover(tmp_path):
    out = tmp_path / "p.yaml"
    r = _run("velcal.py", "--dry-run", "--crossover-erpm", "100000,1600",
             "--seed", "1234", "--profile-out", str(out), expect=1)
    assert "rejected" in r.stderr and not out.exists()


def test_velctl_default_profile_reproducible():
    # the committed sim-derived default must still parse and invert
    from pico_esc import SpeedProfile
    p = SpeedProfile.load(os.path.join(HOST, "profiles", "vel_930kv_12n14p_sim.yaml"))
    assert p.thrust_for(0) == 0
    assert p.thrust_for(p.max_rpm) == p.points[-1][0]


def _needs_inventory():
    """`--plant measured` needs pico_esc/measured.py, which is a per-installation asset and is NOT
    shipped (see the 0ebf709 split). Without it these must SKIP, not fail: a public checkout has the
    tools and none of the numbers. Mirrors test_closed_loop_matrix.py's importorskip."""
    pytest.importorskip("pico_esc.measured",
                        reason="--plant measured needs the local measured inventory")


def test_the_measured_dry_run_plant_is_selectable_and_is_not_the_toy_one(tmp_path):
    """`--plant measured` puts the BENCH-IDENTIFIED plant into the dry run instead of the toy one.

    Why it exists: the toy `SimEncEscHost` is rpm ~linear in throttle plus noise, and it is pinned
    byte-for-byte through its RNG draw order by test_regression_baseline.py — so it cannot be taught
    the three behaviours that actually bite a closed loop here (the ESC BRAKES at zero throttle
    rather than coasting, `tele` never reads below 182 mech rpm however slowly the rotor turns, and
    the sine->6-step seam is a discontinuity). `MeasuredEscHost` already spoke the same link
    protocol; nothing was wired to it but sim_obs. A dry run that never sees those is not rehearsing
    this bench.

    Pinned here: the option selects it, the whole velctl loop runs on top of it unmodified, and the
    default stays `toy` so every existing invocation is untouched.
    """
    _needs_inventory()
    prof = os.path.join(HOST, "profiles", "f2838_350kv_bench_20260726.yaml")
    out = tmp_path / "m.csv"
    r = _run("velctl.py", "speed", "--dry-run", "--plant", "measured", "--rpm", "400",
             "--secs", "1", "--profile", prof, "--csv", str(out))
    assert "MeasuredEscHost" in r.stdout and "BARE_SHAFT_350KV" in r.stdout
    assert "182 mech rpm" in r.stdout          # the telemetry floor is announced, not silently on
    assert "DISARMED" in r.stdout
    rows = list(csv.DictReader(open(out)))
    assert rows and float(rows[-1]["rpm_setpoint"]) == 400.0

    # ...and the default is still the toy one, so nothing that exists today moved.
    r2 = _run("velctl.py", "speed", "--dry-run", "--rpm", "400", "--secs", "1",
              "--profile", prof, "--csv", str(tmp_path / "t.csv"))
    assert "SimEncEscHost" in r2.stdout and "MeasuredEscHost" not in r2.stdout


def test_the_measured_dry_run_plant_rejects_a_mechanical_config_it_has_no_inventory_for(tmp_path):
    """Mixing mechanical configurations is the failure mode plant.py exists to prevent, so an
    unknown one must be a loud KeyError from preset(), not a silent fallback to some default rotor."""
    _needs_inventory()
    prof = os.path.join(HOST, "profiles", "f2838_350kv_bench_20260726.yaml")
    r = _run("velctl.py", "speed", "--dry-run", "--plant", "measured", "--mech-config", "no_such_rotor",
             "--rpm", "400", "--secs", "1", "--profile", prof, "--csv", str(tmp_path / "x.csv"),
             expect=1)
    assert "no_such_rotor" in (r.stderr + r.stdout)
