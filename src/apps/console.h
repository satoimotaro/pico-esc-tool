// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 satoimotaro
//
// console — the ONE text stream the CLI reads commands from and every `#` notice is printed to.
// It used to be `Serial` (USB CDC) hard-wired in ~180 places, which left a board with no USB host
// (an ROV talking to its Pi over a UART, or a bench fixture relaying the CLI over Wi-Fi) no way in.
// Rebind it ONCE, before EscTool::begin() prints anything; the line protocol itself is unchanged, so
// host/esctool.py works over either transport.
#pragma once
#include <Arduino.h>

namespace console {
inline Stream* io = &Serial;
inline Stream& out() { return *io; }
}
