# ESC-controller (thrust-controller)

Standalone **RP2040 (Pico)** controller for BLHeli-S ESCs — setup, firmware flashing,
DShot control, and telemetry for underwater ROV thrusters. **No flight controller
required.**

- **Build system:** PlatformIO, **earlephilhower Arduino-Pico core** (wraps the Pico
  SDK + PIO). `Serial` = USB-C CDC — the Phase-1 host link.
- **DShot/telemetry:** via **[pico-bidir-dshot](https://github.com/bastian2001/pico-bidir-dshot)**
  (GPL-3.0) — DShot TX + bidirectional eRPM + Extended DShot Telemetry.

Design docs live in the workspace `.ai/` (local, not pushed here). License: GPL-3.0-or-later.

## Why

Drone ESCs can't self-configure (need an FC), are tuned for high RPM, are hard for a
plain MCU to read telemetry from, and an FC can only drive ~4 — too few for an ROV.
This controller configures/flashes/drives/monitors many BLHeli-S ESCs from a PC/SBC.

## Layout (PlatformIO)

```
platformio.ini    envs: [pico], [picow]; lib_deps = pico-bidir-dshot
src/main.cpp      A0 baseline: 1 ESC over bidir DShot, USB-CDC command loop
lib/              our modules (auto-discovered) — see lib/README.md
  esc_dshot/  esc_telemetry/  rpm_filter/  esc_setup/*  esc_flash/*  pc_iface/
construction/     wiring / pcb (KiCad) / cad (case)
docs/             protocol notes
                  (* = BLHeli-S 1-wire tools, the hard core — Phase A1)
```

## Build & flash

**Windows PlatformIO (recommended):** open this folder (works over
`\\wsl$\Ubuntu-24.04\home\satoi\UWR_ESC_ws\ESC-controller`), pick env `pico` or
`picow`, Build, then flash (UF2: hold BOOTSEL, drag `.uf2`, or PlatformIO Upload).

**CLI:**
```
pio run -e pico                 # build
pio run -e pico -t upload       # flash
pio device monitor -b 115200    # serial
```

## A0 usage (Serial Monitor, newline mode)

```
E          enable Extended DShot Telemetry
A          arm
T1000      throttle 1000 (0-2000)
D          disarm (throttle 0)
C3         special command 3 (beacon), only when stopped
?          reprint header
```
Prints `Thrott  RPM  Volt  Amp  Temp  Stress  Status`. Set `SIGNAL_PIN` and
`MOTOR_POLES` in `src/main.cpp` for your wiring/motor. See `construction/wiring/`.

## Status

**Phase A0** (see `.ai/architecture/phase-plan.md`). Baseline single-ESC DShot +
telemetry app is in place; next: multi-channel + the 1-wire setup/flash spike.
