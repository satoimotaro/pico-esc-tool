# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""Unit tests for the coast-down brake gate.

The rotor on this ESC does not freely coast: at zero commanded throttle the ESC brakes it, and the
braking deceleration RISES as the rotor slows (measured bare-shaft: -5000 -> -72000 rpm/s). Feeding
those windows to `J = tau_drag/(-dw/dt)` understates J by up to ~10x, which is exactly the bug the
old hand-picked `--fit-lo 1550` was silently papering over on one specific rotor.

`brake_onset` has to find the end of the real drag window from the SHAPE of the decay, with no rpm
constants, because the brake bites at whatever speed the ESC decides.
"""
import pytest

import coastdown

# The drag table is a per-installation measurement and is not distributed; the tests that need it skip
# rather than fail, so the shipped suite is green without it while the local one still covers it.
_HAVE_INVENTORY = coastdown.DRAG_TABLE is not None
_needs_inventory = pytest.mark.skipif(not _HAVE_INVENTORY,
                                      reason="no measured drag table in this distribution")


def _res(pairs, t0=0.0, dt=0.02):
    """[(rpm, dwdt), ...] -> the (t, rpm, dwdt) triples `derive()` produces."""
    return [(t0 + i * dt, rpm, dwdt) for i, (rpm, dwdt) in enumerate(pairs)]


def test_brake_onset_finds_the_step_up_in_deceleration():
    # Six windows of honest drag at -5000, then the brake bites an order of magnitude harder.
    res = _res([(3000.0, -5000.0)] * 6 + [(2000.0, -50000.0)] * 4)
    assert coastdown.brake_onset(res, 0) == 6


def test_brake_onset_returns_end_when_the_coast_is_free():
    # A genuinely free coast never trips the gate — every window must stay usable.
    res = _res([(3000.0 - 50 * i, -5000.0) for i in range(12)])
    assert coastdown.brake_onset(res, 0) == len(res)


def test_brake_onset_is_sign_agnostic():
    """The AS5600 reads NEGATIVE for a positive command on this bench.

    Deceleration means |w| falling, i.e. dw/dt OPPOSING w — not `dwdt < 0`. A gate written against
    the raw sign passes the whole braking phase through on a reversed encoder, which is the same
    trap that once spread J across three decades in `report()`.
    """
    fwd = _res([(3000.0, -5000.0)] * 6 + [(2000.0, -50000.0)] * 4)
    rev = _res([(-3000.0, 5000.0)] * 6 + [(-2000.0, 50000.0)] * 4)
    assert coastdown.brake_onset(rev, 0) == coastdown.brake_onset(fwd, 0) == 6


def test_brake_onset_respects_the_start_index():
    """The still-driven plateau before the cut must not set the baseline.

    `coast_start` hands over an index; windows before it are ~zero deceleration, and letting those
    into the median makes the threshold ~0 so the FIRST real drag window reads as a brake.
    """
    res = _res([(3000.0, -20.0)] * 4                    # plateau: drive not cut yet
               + [(3000.0, -5000.0)] * 6                # real drag
               + [(2000.0, -50000.0)] * 4)              # brake
    assert coastdown.brake_onset(res, 4) == 10


def test_brake_onset_gives_up_gracefully_on_a_short_run():
    # Fewer windows than the probe needs: no basis to judge, so keep everything rather than invent a
    # cut point (silently dropping samples would look like a successful fit).
    res = _res([(3000.0, -5000.0)] * 2)
    assert coastdown.brake_onset(res, 0) == len(res)


@_needs_inventory
def test_drag_table_is_clamped_outside_its_measured_range():
    """`drag_at` extrapolating would invent torque for speeds nobody measured."""
    lo_rpm, lo_tau = coastdown.DRAG_TABLE[0]
    hi_rpm, hi_tau = coastdown.DRAG_TABLE[-1]
    assert coastdown.drag_at(lo_rpm - 500) == lo_tau
    assert coastdown.drag_at(hi_rpm + 500) == hi_tau
    assert coastdown.drag_at(-2000) == coastdown.drag_at(2000)      # magnitude only


def test_analyze_accepts_the_mode_column_as_well_as_run(tmp_path, capsys):
    """`--cut`-comparison CSVs are keyed by `mode`, plain repeat CSVs by `run`.

    Only `run` used to be accepted, so `cutmode_noprop_*.csv` — the file that PROVED the brake belongs
    to zero commanded throttle rather than to `disarm` — raised KeyError inside its own tool. Both
    schemas must read, and the report must say which key it used so a mis-keyed file is visible.
    """
    import csv

    def _write(path, key, keys):
        with open(path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow([key, "t", "accum", "encv_rpm"])
            for k in keys:
                for i in range(40):
                    t = i * 0.004
                    # 3000 rpm falling linearly: accum in counts, 4096/rev
                    rpm = 3000.0 - 2000.0 * t
                    accum = int(-(3000.0 * t - 1000.0 * t * t) / 60.0 * 4096.0)
                    w.writerow([k, f"{t:.4f}", accum, f"{-rpm:.1f}"])

    for key, keys in (("mode", ["disarm", "deadman", "zero_held"]), ("run", ["1", "2"])):
        p = tmp_path / f"{key}.csv"
        _write(p, key, keys)
        assert coastdown.main(["--analyze", str(p), "--no-brake-gate",
                              "--fit-lo", "0", "--fit-hi", "9000"]) == 0
        out = capsys.readouterr().out
        assert f"{key}(s)" in out, out.splitlines()[0]
        for k in keys:
            assert str(k) in out


@_needs_inventory
def test_the_drag_table_is_re_exported_from_measured():
    """The table moved into pico_esc.measured (with its provenance record and the honest note that it
    FALLS with speed). It is re-exported here under the old names so no caller had to change."""
    from pico_esc import measured
    assert coastdown.DRAG_TABLE is measured.DRAG_TABLE_PROP
    assert coastdown.drag_at is measured.drag_at
