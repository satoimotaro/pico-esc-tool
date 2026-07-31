# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""Tests for curvepush: the measured-curve upload transfer.

The point of the tool is that the curve which ends up LIVE is exactly the one measured — so these
cover the round-trip and the rejections, against the sim host that mirrors the firmware's staging
rules (SimEscHost._curve_cmd).
"""
import re

import pytest

import curvepush
from pico_esc.link import SimClock
from pico_esc.sim import SimEncEscHost
from pico_esc.velocity import SpeedProfile


def _host():
    return SimEncEscHost(SimClock())


def _profile(points, regimes=None, up=1400.0, dn=1308.0, pp=7):
    return SpeedProfile(points, pole_pairs=pp, regimes=regimes,
                        crossover={"up_erpm": up, "dn_erpm": dn})


SPAN = [(0.0, 0.0), (200.0, 60.0), (503.0, 186.8), (540.0, 850.0), (700.0, 1900.0)]


def test_round_trip_matches_exactly():
    dev = _host()
    sent, expect = curvepush.push(dev, 1, _profile(SPAN))
    meta, got = curvepush.readback(dev, 1)
    assert meta["measured"] == "1"
    assert int(meta["n"]) == len(sent) == len(SPAN)
    assert [(t, r) for t, r, _ in got] == sent
    assert [g for *_, g in got] == expect          # push() predicts the device's regime rules


def test_regimes_derived_from_the_seam_when_untagged():
    # No regime tags in the YAML -> the device derives them from up_erpm, and the first point above
    # the seam must come back tagged LINE (that is what lineFloor()/the soft-sensor gate keys off).
    dev = _host()
    curvepush.push(dev, 1, _profile(SPAN))
    _, got = curvepush.readback(dev, 1)
    tags = [g for _, _, g in got]
    assert tags == ["s", "s", "s", "l", "l"], tags        # 850*7 = 5950 >= 1400, 186.8*7 = 1308 < 1400


def test_explicit_line_tag_below_the_seam_is_kept():
    # A point measured in the hysteresis band coming down is real information, so an explicit LINE tag
    # below the seam is NOT demoted. (503*7 = 3521 is above 1400 here, so use a higher seam.)
    dev = _host()
    regs = ["sine", "sine", "line", "line", "line"]
    curvepush.push(dev, 1, _profile(SPAN, regimes=regs, up=4000.0, dn=3800.0))
    _, got = curvepush.readback(dev, 1)
    assert [g for _, _, g in got] == ["s", "s", "l", "l", "l"]


def test_sine_tag_above_the_seam_is_promoted_to_line():
    # velcal mislabels the handoff point (the bench profile tags 628 rpm = 4397 eRPM as "sine" against
    # a 1400 eRPM seam). Taking that at face value would push lineFloor() — and therefore the
    # soft-sensor gate — up to the NEXT point, blanking a band where the estimate is valid.
    dev = _host()
    regs = ["sine", "sine", "sine", "sine", "line"]       # index 3 = 850 rpm, 5950 eRPM >> 1400
    curvepush.push(dev, 1, _profile(SPAN, regimes=regs))
    _, got = curvepush.readback(dev, 1)
    assert [g for _, _, g in got] == ["s", "s", "s", "l", "l"]


def test_non_monotone_curve_is_rejected_and_nothing_goes_live():
    # SpeedProfile refuses to construct a non-monotone curve, so drive the raw commands: this is the
    # DEVICE-side guard, i.e. what protects the live curve if a malformed table reaches the wire.
    dev = _host()
    curvepush.push(dev, 1, _profile(SPAN))                # install a good curve first
    dev.cmd("curve 1 begin 3")
    dev.cmd("curve 1 add 0 0")
    dev.cmd("curve 1 add 200 90")
    dev.cmd("curve 1 add 150 60")                         # thrust goes BACKWARDS
    with pytest.raises(RuntimeError, match="curve-commit"):
        dev.cmd("curve 1 commit 7 1400 1308")
    _, got = curvepush.readback(dev, 1)
    assert [(t, r) for t, r, _ in got] == list(SPAN)      # the good curve is still the live one


def test_short_upload_is_rejected():
    dev = _host()
    dev.cmd("curve 1 begin 3")
    dev.cmd("curve 1 add 0 0")
    dev.cmd("curve 1 add 100 50")
    with pytest.raises(RuntimeError, match="staged 2 of 3"):
        dev.cmd("curve 1 commit 7 1400 1308")


def test_profile_without_a_crossover_is_refused():
    dev = _host()
    prof = SpeedProfile(SPAN, pole_pairs=7)
    with pytest.raises(SystemExit, match="crossover"):
        curvepush.push(dev, 1, prof)


def test_thin_keeps_the_endpoints_and_both_sides_of_the_seam():
    pts = [(float(i * 10), float(i)) for i in range(40)]
    pts[25] = (250.0, 900.0)                              # a big jump = the seam
    out, _ = curvepush.thin(pts, None, 12)
    assert len(out) <= 12
    assert out[0] == pts[0] and out[-1] == pts[-1]
    assert pts[24] in out and pts[25] in out


def test_thin_is_a_no_op_below_the_cap():
    out, regs = curvepush.thin(SPAN, None, 24)
    assert out == SPAN and regs is None


@pytest.mark.parametrize("cap", [2, 3, 4, 5])
def test_thin_keeps_the_endpoints_at_every_cap(cap):
    # The mandatory points outnumber the cap at 2 and 3, and truncating a sorted SET dropped the
    # highest index — the last point, which defines the top of the feed-forward range.
    pts = [(float(i * 10), float(i)) for i in range(40)]
    pts[25] = (250.0, 900.0)
    out, _ = curvepush.thin(pts, None, cap)
    assert len(out) <= cap
    assert out[0] == pts[0], f"cap {cap} dropped the first point"
    assert out[-1] == pts[-1], f"cap {cap} dropped the LAST point"


def test_rounding_collision_below_two_points_explains_itself():
    # Sub-0.1 thrust spacing collapses on the wire; the device would only say "need 2..24 points".
    prof = _profile([(0.0, 0.0), (0.02, 10.0), (0.04, 20.0)])
    with pytest.raises(SystemExit, match="0.1 wire resolution"):
        curvepush.push(_host(), 1, prof)


def test_mixed_regime_tags_are_downgraded_to_untagged():
    # The device's tagged-flag is table-wide, so one missing tag discards them ALL. push() must not
    # send a half-tagged table and let an explicit `line` vanish without a word.
    dev = _host()
    regs = ["sine", None, "line", "line", "line"]      # index 2 (503 rpm) is below the 4000 seam
    curvepush.push(dev, 1, _profile(SPAN, regimes=regs, up=4000.0, dn=3800.0))
    _, got = curvepush.readback(dev, 1)
    # untagged => derived purely from the seam: only 850 and 1900 rpm clear 4000/7 = 571
    assert [g for *_, g in got] == ["s", "s", "s", "l", "l"]


# --- check_crossover: the safety mechanism the PR calls "the important part" -----------------------

class _FakeCfg:
    """Stands in for EscConfig: canned read(), records set()/restart()."""
    instances = []

    def __init__(self, link, index):
        self.link, self.index = link, index
        self.sets, self.restarts = [], 0
        _FakeCfg.instances.append(self)

    def read(self):
        return _FakeCfg.canned

    def set(self, **kw):
        self.sets.append(kw)

    def restart(self):
        self.restarts += 1


@pytest.fixture
def fake_cfg(monkeypatch):
    _FakeCfg.instances = []
    _FakeCfg.canned = {"settings": {"sine_cross_up": 34, "sine_cross_dn": 239}}
    monkeypatch.setattr(curvepush, "EscConfig", _FakeCfg)
    return _FakeCfg


def test_crossover_check_passes_when_the_profile_states_no_bytes(fake_cfg):
    prof = SpeedProfile(SPAN, pole_pairs=7, crossover={"up_erpm": 1400.0, "dn_erpm": 1308.0})
    assert curvepush.check_crossover(_host(), 1, prof, False) is True


def test_crossover_check_passes_on_a_match(fake_cfg):
    prof = _profile(SPAN, up=34 * curvepush.SINE_CROSS_UP_ERPM_PER_UNIT, dn=239.0)
    prof.crossover["bytes"] = [34, 239]
    assert curvepush.check_crossover(_host(), 1, prof, False) is True
    assert fake_cfg.instances[-1].sets == []


def test_crossover_check_REFUSES_on_a_mismatch(fake_cfg):
    prof = _profile(SPAN, up=36 * curvepush.SINE_CROSS_UP_ERPM_PER_UNIT, dn=239.0)
    prof.crossover["bytes"] = [36, 239]
    assert curvepush.check_crossover(_host(), 1, prof, False) is False
    assert fake_cfg.instances[-1].sets == []          # refusing must not write anything


def test_crossover_check_applies_when_asked(fake_cfg):
    prof = _profile(SPAN, up=36 * curvepush.SINE_CROSS_UP_ERPM_PER_UNIT, dn=239.0)
    prof.crossover["bytes"] = [36, 239]
    assert curvepush.check_crossover(_host(), 1, prof, True) is True
    assert fake_cfg.instances[-1].sets == [{"sine_cross_up": 36, "sine_cross_dn": 239}]
    assert fake_cfg.instances[-1].restarts >= 1       # the ESC must be restarted, not left in bootloader


def test_crossover_check_rejects_a_self_inconsistent_profile(fake_cfg):
    # bytes say 36 (= 1406 eRPM) but up_erpm says 1400*2 — the ESC check would pass while the device
    # received a different seam, which is the exact failure this guard exists to prevent.
    prof = _profile(SPAN, up=2800.0, dn=1308.0)
    prof.crossover["bytes"] = [36, 239]
    with pytest.raises(SystemExit, match="self-inconsistent"):
        curvepush.check_crossover(_host(), 1, prof, False)


# --- main() exit codes: the contract automation sees -----------------------------------------------

def test_main_returns_2_when_the_crossover_is_refused(fake_cfg, tmp_path):
    prof = _profile(SPAN, up=36 * curvepush.SINE_CROSS_UP_ERPM_PER_UNIT, dn=239.0)
    prof.crossover["bytes"] = [36, 239]
    p = tmp_path / "p.yaml"
    prof.save(str(p))
    assert curvepush.main([str(1), str(p), "--dry-run"]) == 2


def test_main_returns_0_on_a_clean_push(fake_cfg, tmp_path):
    prof = _profile(SPAN, up=34 * curvepush.SINE_CROSS_UP_ERPM_PER_UNIT, dn=239.0)
    prof.crossover["bytes"] = [34, 239]
    p = tmp_path / "p.yaml"
    prof.save(str(p))
    assert curvepush.main([str(1), str(p), "--dry-run"]) == 0


# --- the `motor` step is BEST-EFFORT: it must not abort the push -----------------------------------

class _MotorErrHost:
    """The sim host, except that `motor ...` fails the way the firmware's `err motor:` reply fails.

    `EscHost.cmd` (pico_esc/link.py) raises RuntimeError on any reply line starting with `err`, and
    since 2026-07-31 the firmware answers `motor` with one whenever no parametric curve exists at the
    ESC's cross_up (>= 48: the handoff is above the cmd 700 the curve is sampled to). Deliberately a
    wrapper rather than a subclass — pico_esc/sim.py is pinned byte-for-byte through its RNG draw
    order by test_regression_baseline.py and must not be touched.
    """

    def __init__(self):
        self.inner = SimEncEscHost(SimClock())
        self.motor_calls = 0

    def cmd(self, line, *a, **kw):
        if line.startswith("motor "):
            self.motor_calls += 1
            raise RuntimeError("device: err motor: no 6-step curve at cross_up 50 (its handoff is "
                               "refused, or above the cmd 700 the curve is sampled to)")
        return self.inner.cmd(line, *a, **kw)


def test_a_motor_command_that_cannot_generate_a_curve_does_not_abort_the_push(fake_cfg, tmp_path,
                                                                              monkeypatch, capsys):
    """README.md's documented command line, on an ESC at a cross_up with no parametric curve.

    `--motor` exists for its SIDE EFFECT (kv/pp/v into the controller's parametric model, so the
    soft-sensor's duty map is right), not for the parametric FF curve — which the measured curve this
    tool uploads is about to replace anyway. The firmware applies kv/pp/v before it discovers it has no
    curve to emit, so the side effect has already landed when the error arrives. Aborting there was
    pure loss: it skipped `push()` entirely, so the ONE feed-forward that is valid at that cross_up
    never reached the device.
    """
    prof = _profile(SPAN, up=34 * curvepush.SINE_CROSS_UP_ERPM_PER_UNIT, dn=239.0)
    prof.crossover["bytes"] = [34, 239]
    p = tmp_path / "p.yaml"
    prof.save(str(p))
    host = _MotorErrHost()
    monkeypatch.setattr(curvepush, "open_host", lambda opts: host)

    rc = curvepush.main([str(1), str(p), "--dry-run", "--motor", "350,7,11.2"])

    assert host.motor_calls == 1                    # it really did try, and really did fail
    assert rc == 0                                  # ...and the push still verified clean
    out = capsys.readouterr().out
    assert "note: no parametric curve at this cross_up" in out
    assert "measured curve below is what drives the FF" in out
    # the curve is LIVE on the device, which is the whole point of the tool
    meta, got = curvepush.readback(host, 1)
    assert meta["measured"] == "1" and int(meta["n"]) == len(SPAN)


@pytest.mark.parametrize("msg", [
    "device: err bad-args (motor <i> <kv> <pp> <v>)",   # rejected BEFORE setMotor: kv/pp/v did NOT land
    "device: err motor: disarm first",                  # ditto
    "read timed out",                                   # not a cross_up problem at all
])
def test_a_motor_failure_that_is_not_the_no_curve_case_still_aborts(fake_cfg, tmp_path, monkeypatch,
                                                                    msg):
    """The best-effort wrapper above must stay NARROW, and this is what pins that.

    Only one `motor` failure means "kv/pp/v landed, the curve did not": the `no 6-step curve` reply.
    `err bad-args` and `err motor: disarm first` are rejected before `setMotor` runs, so the side
    effect did NOT land and the note's claim would be false; a timeout is a dead link, and diagnosing
    it as "no parametric curve at this cross_up" would send the operator after the wrong fault. A
    bare `except Exception` swallowed all three — this test fails if anyone widens it back.
    """
    prof = _profile(SPAN, up=34 * curvepush.SINE_CROSS_UP_ERPM_PER_UNIT, dn=239.0)
    prof.crossover["bytes"] = [34, 239]
    p = tmp_path / "p.yaml"
    prof.save(str(p))

    class _OtherErrHost(_MotorErrHost):
        def cmd(self, line, *a, **kw):
            if line.startswith("motor "):
                self.motor_calls += 1
                raise RuntimeError(msg)
            return self.inner.cmd(line, *a, **kw)

    host = _OtherErrHost()
    monkeypatch.setattr(curvepush, "open_host", lambda opts: host)
    with pytest.raises(RuntimeError, match=re.escape(msg)):
        curvepush.main([str(1), str(p), "--dry-run", "--motor", "350,7,11.2"])
    assert host.motor_calls == 1
