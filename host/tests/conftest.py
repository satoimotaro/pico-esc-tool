# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""Make the host/ directory importable so the tests can `import pico_esc` / the CLIs."""
import os
import sys

HOST = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if HOST not in sys.path:
    sys.path.insert(0, HOST)
