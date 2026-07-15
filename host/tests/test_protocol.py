# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""Protocol freeze: every command + response literal the host speaks must still exist,
byte-for-byte, in the RP2040 firmware source. This is the guard that stops the two sides of
the wire protocol from silently drifting apart during a refactor.
"""
import os

import pytest

from pico_esc import protocol

FW = os.path.join(os.path.dirname(__file__), "..", "..", "src", "apps", "esc_tool.cpp")


@pytest.fixture(scope="module")
def firmware_src():
    with open(FW, encoding="utf-8") as fh:
        return fh.read()


@pytest.mark.parametrize("cmd", protocol.COMMANDS)
def test_command_literal_in_firmware(firmware_src, cmd):
    # commands are matched with strcmp(cmd, "..."), so the quoted token must appear verbatim
    assert f'"{cmd}"' in firmware_src, f'command "{cmd}" missing from esc_tool.cpp'


@pytest.mark.parametrize("resp", protocol.RESPONSES)
def test_response_literal_in_firmware(firmware_src, resp):
    assert resp in firmware_src, f"response literal {resp!r} missing from esc_tool.cpp"


@pytest.mark.parametrize("err", protocol.ERRORS)
def test_error_literal_in_firmware(firmware_src, err):
    assert err in firmware_src, f"error literal {err!r} missing from esc_tool.cpp"


def test_response_tags_are_prefixes():
    # host parsers do line.startswith(TAG); these must match the response formats above
    assert protocol.TAG_ESC == "esc|"
    assert protocol.TAG_CFG == "cfg|"
    assert protocol.TAG_ENC == "enc|"
    assert protocol.TAG_TELE == "tele|"
    for tag in (protocol.TAG_ESC, protocol.TAG_CFG, protocol.TAG_DEV,
                protocol.TAG_DATA, protocol.TAG_ENC, protocol.TAG_TELE):
        assert any(r.startswith(tag) for r in protocol.RESPONSES)
