# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 satoimotaro
"""pico_esc.constants — plant / geometry constants shared by the sim and the controller.

These live here (not in control.py) so the simulated host (sim.py) can import the plant gain
without depending on the controller module. control.py re-exports them so the historical
`from pico_esc.control import COUNTS_PER_REV / FULLSCALE_RPM / KFF_COMPUTED` (and the posctl ->
tune_sine_amp chain) keep resolving. Values are byte-for-byte the originals from posctl.py.
"""
from __future__ import annotations

COUNTS_PER_REV = 4096              # AS5600 12-bit; a 2-pole shaft magnet, so this IS one MECHANICAL
                                   # rev (hand-turn confirmed ~4096 ticks/turn) — encoder speed is
                                   # true mechanical RPM.

POLE_PAIRS = 7                     # 12N14P motor. IMPORTANT: the ESC `tele` line is ALREADY mechanical
                                   # RPM — the RP2040 firmware divides the DShot eRPM by pole pairs
                                   # (ESC_MOTOR_POLES/2, esc_session.h) before sending it. So do NOT
                                   # divide tele.rpm by POLE_PAIRS to get mech (that double-division
                                   # made a real BEMF lock read as 1/7). POLE_PAIRS is only for the
                                   # OTHER direction: converting a mechanical RPM to eRPM to compare
                                   # against the firmware's electrical Cross_Up/Cross_Dn thresholds.

# A `tele` frame counts as a LIVE 6-step telemetry sample once |mech RPM| exceeds this floor;
# the garbage first-few-frames-after-arm (and forced-sine, where telemetry is stale) read ~0 and
# are rejected. Keyed on MECHANICAL RPM — the firmware pre-divides eRPM by pole pairs before
# sending `tele`, so this is NOT an electrical threshold (the historical name TELE_MIN_ERPM in
# crossover.py is the same value, re-exported from here). Used both by the crossover measurement
# loop and by VelocityController to arm/disarm its closed-loop PI trim.
TELE_MIN_MECH_RPM = 50.0

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
