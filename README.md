# ESC-controller (thrust-controller)

Standalone **RP2040 (Pico)** controller for BLHeli-S ESCs — setup, firmware flashing,
DShot control, and telemetry for underwater ROV thrusters. **No flight controller
required.** Built with the **Pico SDK (C/C++) + PIO**.

Part of the UWR ESC project. Design docs are kept in the workspace `.ai/` (local).

## Why

Drone ESCs can't self-configure (need an FC), are tuned for high RPM, are hard for a
plain MCU to read telemetry from, and an FC can only drive ~4 of them — too few for an
ROV. This board fixes that: configure/flash/drive/monitor many BLHeli-S ESCs directly
from a PC/SBC.

## Layout

```
firmware/
  src/        app entry, command dispatch, board pins
  include/    public headers
  pio/        DShot TX + bidirectional DShot RX PIO programs
  lib/
    dshot/         DShot150/300/600 TX
    telemetry/     eRPM (bidir DShot) + optional KISS telem UART
    rpm_filter/    eRPM → mechanical RPM, dropout rejection
    esc_setup/     BLHeli-S EEPROM parameter read/write   [core, hard]
    flash/         BLHeli-S 1-wire bootloader flashing     [core, hard]
    pc_iface/      host link (USB-C CDC first)
host/         optional PC-side CLI
construction/ wiring / pcb (KiCad) / cad (case)
docs/         protocol notes, build & run guide
```

## Status

Early scaffold — see the workspace phase plan (`.ai/architecture/phase-plan.md`),
Phase A0. See `docs/protocol-notes.md` for the BLHeli-S/DShot references.

## Target hardware

Bring-up on a bare Pico + LittleBee Spring 30A (BLHeli-S). Generalizes to other
BLHeli-S ESCs later.

## Build (planned)

Pico SDK + CMake. Toolchain and instructions land with Phase A0.
