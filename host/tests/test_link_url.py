# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""EscHost over a pyserial URL (socket://) — the path a UART-console Pico behind a TCP relay uses.

No hardware: a thread plays the firmware on a loopback socket and answers the line protocol.
"""
import socket
import threading

import pytest

from pico_esc.link import EscHost


def _fake_pico(srv: socket.socket):
    conn, _ = srv.accept()
    with conn, conn.makefile("rwb", buffering=0) as f:
        for raw in f:
            line = raw.decode().strip()
            if line == "ping":
                f.write(b"id esc_tool v1\r\nok\r\n")
            elif line == "bad":
                f.write(b"err bad-args\r\n")
            elif line == "quit":
                return


@pytest.fixture
def url():
    srv = socket.socket()
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    t = threading.Thread(target=_fake_pico, args=(srv,), daemon=True)
    t.start()
    yield f"socket://127.0.0.1:{srv.getsockname()[1]}"
    srv.close()


def test_a_command_round_trips_over_tcp(url):
    h = EscHost(url)
    try:
        assert h.cmd("ping", timeout=5) == ["id esc_tool v1"]
    finally:
        h.ser.write(b"quit\n")
        h.close()


def test_err_still_raises_over_tcp(url):
    h = EscHost(url)
    try:
        with pytest.raises(RuntimeError, match="bad-args"):
            h.cmd("bad", timeout=5)
    finally:
        h.ser.write(b"quit\n")
        h.close()
