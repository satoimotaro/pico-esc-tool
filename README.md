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
platformio.ini        envs: [picow] app, [esc_host] unified tool, [spike_*] diagnostics
src/main.cpp          A0 baseline: 1 ESC over bidir DShot, USB-CDC command loop
src/apps/esc_host.cpp unified host-driven firmware (serial command protocol) for the CLI
host/esctool.py       PC-side CLI: list / read ESCs (set/apply/flash coming)
lib/                  our modules (auto-discovered) — see lib/README.md
  blheli_bl/*  esc_setup/*  esc_flash/*   BLHeli-S 1-wire bootloader: connect/read/write/flash
  esc_dshot/  esc_telemetry/  rpm_filter/  pc_iface/
construction/         wiring / pcb (KiCad) / cad (case)
docs/                 protocol notes; lib/blheli_bl/PROTOCOL.md = the bootloader reference
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

## Host CLI (esctool)

Flash the unified **`esc_host`** firmware, then drive ESCs from a PC over USB serial:

```
pio run -e esc_host -t upload
python host/esctool.py list                       # scan & list connected ESCs
python host/esctool.py read 0 -o config.yaml      # export one ESC's config to YAML
python host/esctool.py set 0 motor_direction=Reversed beep_strength=60
python host/esctool.py apply all host/profiles/blheli-s-default.yaml   # a profile -> every ESC
python host/esctool.py run 0                       # end the session (restart the ESC)
```

Config commands hold the ESC in a **bootloader session** (motor off, no repeated reboots) and
reuse it across commands; the ESC only restarts on `run`/`disconnect` (or with `-r`). Use an
ESC index or `all`. Multi-ESC is driven by the firmware's `ESC_PINS[]` list. Needs `pyserial`
(`pip install pyserial`); auto-detects the Pico by USB VID 2E8A. Flashing firmware from the CLI
is the next phase (today, flashing is via the `spike_program` env).

## A0 usage (Serial Monitor, newline mode)

```
E          enable Extended DShot Telemetry
A          arm

0      throttle 1000 (0-2000)
D          disarm (throttle 0)
C3         special command 3 (beacon), only when stopped
?          reprint header
```
Prints `Thrott  RPM  Volt  Amp  Temp  Stress  Status`. Set `SIGNAL_PIN` and
`MOTOR_POLES` in `src/main.cpp` for your wiring/motor. See `construction/wiring/`.

## Status

**Phase A1 — BLHeli-S 1-wire bootloader tools proven on hardware** (EFM8BB21). Working
end-to-end: bootloader connect, config **read** (CRC-verified), config **write**
(read-modify-write + verify), and **firmware flash** with a layout/MCU compatibility
guard — app-only (bootloader + EEPROM preserved), and the firmware's own default config
is auto-applied from the HEX after flashing. Unified host firmware `esc_host` + `esctool`
CLI (list / read) are in place; next: `set`/`apply`(YAML profiles)/`flash` from the CLI,
BLHeli-S default profiles, and multi-ESC. (Phase-plan detail in the workspace `.ai/`.)
