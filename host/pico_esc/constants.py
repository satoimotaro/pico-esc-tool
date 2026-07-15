# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""pico_esc.constants — plant / geometry constants shared by the sim and the controller.

These live here (not in control.py) so the simulated host (sim.py) can import the plant gain
without depending on the controller module. control.py re-exports them so the historical
`from pico_esc.control import COUNTS_PER_REV / FULLSCALE_RPM / KFF_COMPUTED` (and the posctl ->
tune_sine_amp chain) keep resolving. Values are byte-for-byte the originals from posctl.py.
"""
from __future__ import annotations

COUNTS_PER_REV = 4096              # AS5600 12-bit

# Firmware S1 full-scale: mechanical RPM at |thrust|=1000. This is the PLANT GAIN the
# feedforward (--kff) inverts, so it is tied to the firmware fixed-point constants:
#   eRPM      = Rcp * (1<<SINE_RCP_SHIFT) * (F_TIMER2/SINE_TICK_T2) / 65536 * (60/6)
#   mech RPM  = eRPM / POLE_PAIRS,   Rcp ≈ 2.047 * thrust  (bidir DShot mapping)
# with the ESC-firmware asm EQUs SINE_TICK_T2=4000, SINE_RCP_SHIFT=3, Timer2=4 MHz,
# POLE_PAIRS=7 => 356.97 mech RPM / kff 0.4669. Printed by
# ESC-firmware/tools/sim/sine_drive_model.py (stepper section) — keep these in sync if
# either the asm EQUs or the sim change (the startup note in posctl flags a mismatch).
FULLSCALE_RPM = 357.0
KFF_COMPUTED = 1000.0 / (FULLSCALE_RPM * 6.0)   # thrust per deg/s implied by FULLSCALE_RPM (~0.467)
