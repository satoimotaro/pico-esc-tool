# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""The firmware<->host contract for the `sense` line, pinned from the host side.

`sense` gained fields for the soft-sensor validity window (`why`, `cmdfloor`, `loadok`, `vbok`) and the
staged thermal limiter (`temp`, `tpeak`, `tlim`, `tstate`). The rule the firmware follows is
APPEND-ONLY — the prefix and the existing key order are a contract — and these tests are what makes a
violation of it fail here rather than silently on the bench, in three directions:

1. a canonical NEW-format line parses, and every documented key lands in the sampled row;
2. an OLD-format line (a Pico that has not been reflashed) still parses without raising — the new keys
   simply read as absent, never as a wrong value;
3. a MISSING or `err` reply yields NaNs / zeros, never a plausible-looking healthy row. driftsoak's
   guards compare with `>=`, and every such test on a NaN is False, so a row that silently reads
   "healthy" would switch the temperature / overspeed / stall aborts off for an unattended run.

No serial port, no firmware: the `dev` here is a stub that returns canned lines.
"""
import math

import driftsoak

NEW = ("sense|1|vest=9.14|vref=9.20|load=0.06|vbatt=11.30|valid=1|floor=374|dhat=-15.0|settled=1.00"
       "|trim=11.6|cmd=652|why=ok|cmdfloor=588|loadok=1|vbok=1|temp=52|tpeak=57|tlim=1.00|tstate=ok")
OLD = ("sense|1|vest=9.14|vref=9.20|load=0.06|vbatt=11.30|valid=1|floor=374|dhat=-15.0|settled=1.00"
       "|trim=11.6|cmd=652")
TELE = "tele|1596|11.20|1.40|52|0"
ENCV = "encv|123456|1596.00|400|0"


class StubDev:
    """Minimal EscHost stand-in: `sense` returns `sense_line`, `tele`/`encv` are canned."""

    def __init__(self, sense_line=NEW, sense_raises=False):
        self.sense_line = sense_line
        self.sense_raises = sense_raises
        self.sent = []

    def cmd(self, line, timeout=None):
        self.sent.append(line)
        if line.startswith("sense"):
            if self.sense_raises:
                raise RuntimeError("device: err no-telem")
            return [self.sense_line] if self.sense_line else []
        if line.startswith("tele"):
            return [TELE]
        if line.startswith("encv"):
            return [ENCV]
        return []


def test_new_format_parses_every_documented_key():
    s = driftsoak.sample(StubDev(), 1)
    assert s["trim"] == 11.6 and s["cmd"] == 652.0
    assert s["dhat"] == -15.0 and s["settled"] == 1.0
    assert s["vest"] == 9.14 and s["vbatt"] == 11.30 and s["load"] == 0.06
    assert s["valid"] == 1
    # the B.6 validity-window fields
    assert s["why"] == "ok" and s["loadok"] == 1 and s["vbok"] == 1
    # the B.7 thermal fields
    assert s["tlim"] == 1.00 and s["tstate"] == "ok"
    # and the covariates that come from `tele` / `encv`, unchanged
    assert s["tele_rpm"] == 1596.0 and s["tempC"] == 52.0 and s["enc_rpm"] == 1596.0


def test_a_closed_gate_reports_which_condition_closed_it():
    """The gate is stricter than the old rpm-only test, so valid=0 must carry a reason."""
    line = NEW.replace("valid=1", "valid=0").replace("why=ok", "why=cmdlow").replace("loadok=1", "loadok=0")
    s = driftsoak.sample(StubDev(line), 1)
    assert s["valid"] == 0 and s["why"] == "cmdlow" and s["loadok"] == 0


def test_a_thermal_derate_is_visible_in_the_row():
    line = NEW.replace("tlim=1.00", "tlim=0.62").replace("tstate=ok", "tstate=derate")
    s = driftsoak.sample(StubDev(line), 1)
    assert s["tlim"] == 0.62 and s["tstate"] == "derate"


def test_old_format_line_still_parses():
    """A Pico running pre-B.6 firmware: the new keys are absent, not wrong."""
    s = driftsoak.sample(StubDev(OLD), 1)
    assert s["trim"] == 11.6 and s["valid"] == 1        # everything that existed still works
    assert s["why"] == "" and s["tstate"] == ""         # absent reads as absent
    assert s["loadok"] == 0 and s["vbok"] == 0
    assert math.isnan(s["tlim"])


def test_key_order_and_prefix_are_not_relied_on_beyond_the_contract():
    """Parsing is by key, so an appended key cannot shift any existing one."""
    s = driftsoak.sample(StubDev(NEW + "|somethingnew=7"), 1)
    assert s["cmd"] == 652.0 and s["why"] == "ok"


def test_missing_sense_reply_yields_nans_not_a_healthy_row():
    s = driftsoak.sample(StubDev(sense_line=""), 1)
    for k in ("trim", "cmd", "dhat", "settled", "vest", "vbatt", "load", "tlim"):
        assert math.isnan(s[k]), f"{k} should be NaN when `sense` says nothing"
    assert s["valid"] == 0 and s["why"] == "" and s["loadok"] == 0 and s["vbok"] == 0


def test_err_reply_yields_nans_not_a_healthy_row():
    s = driftsoak.sample(StubDev(sense_raises=True), 1)
    assert math.isnan(s["trim"]) and math.isnan(s["tlim"])
    assert s["valid"] == 0 and s["why"] == ""
    # the tele-derived guards must still be populated: a sense failure is not a telemetry failure
    assert s["tele_rpm"] == 1596.0 and s["tempC"] == 52.0
