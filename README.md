# pico-esc-tool

An **RP2040 (Pico / Pico W)** tool for **BLHeli-S ESCs**: configure, flash firmware, and drive
them over DShot with telemetry — **no flight controller required.** Built for underwater ROV
thrusters (many ESCs from one PC/SBC), but works with any BLHeli-S SiLabs ESC.

Developed and tested on **ReadyToSky 45A and 30A** ESCs (SiLabs **EFM8BB21**).

It has two halves:

- **`esc_host`** — firmware for the Pico that speaks the BLHeli-S 1-wire bootloader on the ESC
  signal line and exposes a small serial protocol.
- **`esctool`** — a BLHeli-Configurator-like **CLI** on the PC that drives it: list ESCs, read /
  write settings (YAML profiles), and flash firmware — with a layout/MCU **compatibility guard**.

## Features

- **Discover / read / write** BLHeli-S config over the signal wire (CRC-verified, read-back checked).
- **YAML profiles** — export a config, edit, and `apply` it (single ESC or `all`).
- **Firmware flashing** from the CLI, app-only (bootloader + your settings-page are never touched),
  refused unless the HEX's layout + MCU match the ESC. The firmware's own default config is
  auto-applied after a flash.
- **Bootloader session** — config commands keep the ESC connected (motor off) and reuse it, so a
  batch of commands doesn't reboot the ESC between each; it restarts only on `run`/`disconnect`.
- **Multi-ESC** — one signal pin per ESC (`ESC_PINS[]`); target an index or `all`.
- **DShot control + bidirectional eRPM telemetry** baseline app (via
  [pico-bidir-dshot](https://github.com/bastian2001/pico-bidir-dshot)).

## Requirements

- **Hardware:** RP2040 board; each ESC's signal wire on a GPIO (default **GP10**) with a common
  ground. See `construction/wiring/`.
- **Firmware build:** [PlatformIO](https://platformio.org/) (earlephilhower Arduino-Pico core;
  fetched automatically).
- **Host CLI:** Python 3.8+ and `pyserial` (`pip install pyserial`; `pyyaml` optional).

## Quick start

Build and flash the unified firmware to the Pico:

```
pio run -e esc_host -t upload
```

Then drive ESCs from the PC (the Pico is auto-detected by its USB VID 2E8A):

```
python host/esctool.py list                                   # scan & list connected ESCs
python host/esctool.py read 0 -o config.yaml                  # export ESC 0's config to YAML
python host/esctool.py set 0 motor_direction=Reversed beep_strength=60
python host/esctool.py apply all host/profiles/blheli-s-default.yaml
python host/esctool.py flash 0 J_H_25_REV16_7.HEX --yes       # flash matching BLHeli-S firmware
python host/esctool.py run 0                                  # end the session, restart the ESC
```

## CLI reference

| Command | What it does |
|---|---|
| `list` | Scan `ESC_PINS[]` and print each ESC (signature, layout, name, firmware). |
| `connect <i\|all>` | Enter the bootloader and hold the session (motor off). |
| `read <i\|all> [-o file.yaml]` | Read config; print YAML or write it to a file. |
| `set <i\|all> key=value ...` | Change settings (enum names OK, e.g. `motor_direction=Reversed`). |
| `apply <i\|all> profile.yaml` | Apply a profile's `settings:` block (`--name`, `--with-name`). |
| `flash <i> file.hex --yes` | Compat-check then erase/program/verify the app + apply defaults (`--force` to override). |
| `run` / `disconnect` `[i]` | End the session and restart the held ESC(s). |

Config commands hold the ESC in a bootloader session (motor off) and **do not restart it** — add
`-r` to restart after a single command, or `run`/`disconnect` when done. Get the firmware HEX for
your ESC's layout from [github.com/bitdump/BLHeli](https://github.com/bitdump/BLHeli)
(`BLHeli_S SiLabs/Hex files/`).

## Wi-Fi web tool (`esc_web`, Pico W)

Flash `esc_web`, connect your phone/PC to the Pico W's Wi-Fi Access Point (SSID `pico-esc-tool`),
and open `http://192.168.4.1` for a configurator-style browser UI — scan ESCs, read, and edit
settings with no cables. Change `AP_SSID` / `AP_PASS` at the top of `src/apps/esc_web.cpp`.

```
pio run -e esc_web -t upload
```

First slice: scan / read / set / disconnect; firmware flashing from the browser is next. (Pico W
only; both radio and DShot use PIO — validate on the bench.)

## Safety

- **Flash is app-only.** The 1-wire bootloader (top of flash) is never overwritten — it's how the
  tool talks to the ESC — so a failed flash is recoverable by re-flashing.
- **Compatibility guard.** `flash` refuses a HEX whose layout tag or MCU signature doesn't match
  the connected ESC (wrong FET map / wrong chip), unless `--force`.
- Every write is **read-back verified** on the device.

## PlatformIO environments

| Env | Purpose |
|---|---|
| `esc_host` | **USB tool** — host-driven firmware for the `esctool` CLI (over USB serial). |
| `esc_web` | **Wi-Fi web tool** (Pico W) — AP + browser configurator (see below). |
| `picow` / `pico` | DShot control + telemetry baseline app (`src/main.cpp`). |
| `spike_*` | Standalone bring-up/diagnostic firmwares for the bootloader work. |

Both `esc_host` and `esc_web` share the bootloader logic in `src/apps/esc_session.h`.

## Layout

```
platformio.ini          build environments
src/main.cpp            DShot + telemetry baseline app (USB-CDC command loop)
src/apps/esc_host.cpp   unified host-driven firmware (serial protocol for esctool)
host/esctool.py         the PC CLI
host/profiles/          example YAML profiles
lib/blheli_bl/          BLHeli-S 1-wire bootloader client  (PROTOCOL.md = reference)
lib/esc_setup/          config read / write (read-modify-write a flash page)
lib/esc_flash/          Intel-HEX parse, program/verify, compatibility check
construction/wiring/    how to wire an ESC signal line to the Pico
docs/                   notes; lib/blheli_bl/PROTOCOL.md is the bootloader reference
```

## DShot baseline app (`picow` env)

`src/main.cpp` drives one ESC over bidirectional DShot from the serial monitor (115200, newline):
`E` enable telemetry, `A` arm, a number = throttle (0–2000), `D` disarm, `C3` beacon (when stopped),
`?` reprint header. Set the signal pin / motor pole count near the top of the file for your wiring.

## Status & roadmap

**BLHeli-S 1-wire tooling is proven on hardware** (EFM8BB21): connect, read, write, and flash all
work via `esctool`, with the compat guard and auto-default-config. Next phases: telemetry + RPM
filtering polish, and a Wi-Fi / BLE link to the Pico W for a wireless GUI.

## License

Copyright (C) 2026 satoimotaro. **GPL-3.0-or-later** — see [`LICENSE`](LICENSE).

Third-party: DShot / telemetry via
[pico-bidir-dshot](https://github.com/bastian2001/pico-bidir-dshot) (GPL-3.0). The BLHeli-S 1-wire
protocol was implemented with reference to Betaflight and BLHeli_S (both GPL). BLHeli-S firmware
images belong to their authors ([github.com/bitdump/BLHeli](https://github.com/bitdump/BLHeli))
and are **not** distributed here.
