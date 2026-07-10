# pico-esc-tool

An **RP2040 (Pico / Pico W)** tool for **BLHeli-S ESCs**: configure, flash firmware, and drive
them over DShot with telemetry — **no flight controller required.** Built for underwater ROV
thrusters (many ESCs from one PC/SBC), but works with any BLHeli-S SiLabs ESC.

Developed and tested on **ReadyToSky 45A and 30A** ESCs (SiLabs **EFM8BB21**).

It has two parts:

- **`esc_tool`** — one Pico W firmware that configures/flashes BLHeli-S ESCs over the 1-wire
  bootloader **and** spins thrusters over DShot, exposed over USB serial and (in SETUP mode) a
  Wi-Fi web UI. No reflashing to switch jobs.
- **`esctool`** — a BLHeli-Configurator-like **CLI** on the PC that drives it over USB: list ESCs,
  read / write settings (YAML profiles), and flash firmware — with a layout/MCU **compatibility guard**.

**Modes** (chosen at boot by an optional GPIO, or the `mode` command): **SETUP** brings up the
Wi-Fi AP + web configurator and per-thruster test; **DRIVE** turns Wi-Fi off and accepts
per-thruster commands with a deadman. cmd_vel→thruster mixing is left to the host/Pi (RL-friendly).

## Features

- **Discover / read / write** BLHeli-S config over the signal wire (CRC-verified, read-back checked).
- **YAML profiles** — export a config, edit, and `apply` it (single ESC or `all`).
- **Firmware flashing** from the CLI, app-only (bootloader + your settings-page are never touched),
  refused unless the HEX's layout + MCU match the ESC. The firmware's own default config is
  auto-applied after a flash.
- **Bootloader session** — config commands keep the ESC connected (motor off) and reuse it, so a
  batch of commands doesn't reboot the ESC between each; it restarts only on `run`/`disconnect`.
- **Multi-ESC** — one signal pin per ESC (`PINS[]` in `src/apps/esc_session.h`); target an index or `all`.
- **Firmware-aware thruster drive** — the spin path detects the ESC firmware and picks the DShot
  variant: **normal DShot** for stock BLHeli-S (throttle only), **bidirectional DShot** for
  Bluejay/JESC (with telemetry). Arms the ESC, then holds throttle with a deadman auto-stop.
- **Reversible / 3D** — if the ESC is configured reversible, throttle becomes a **signed thrust**
  (−full … 0 = stop … +full); one-way ESCs use a plain 0–100% throttle.
- **Live telemetry** (bidir firmware) — eRPM always; temperature + stress via EDT; voltage/current
  when the ESC has the sensors. DShot via
  [pico-bidir-dshot](https://github.com/bastian2001/pico-bidir-dshot).

## Requirements

- **Hardware:** RP2040 board; each ESC's signal wire on a GPIO (default **GP10**) with a common
  ground. See `construction/wiring/`.
- **Firmware build:** [PlatformIO](https://platformio.org/) (earlephilhower Arduino-Pico core;
  fetched automatically).
- **Host CLI:** Python 3.8+ and `pyserial` (`pip install pyserial`; `pyyaml` optional).

## Quick start

Build and flash the unified firmware to the Pico:

```
pio run -e esc_tool -t upload
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
| `list` | Scan all configured pins and print each ESC (signature, layout, name, firmware). |
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

## Driving thrusters

`esc_tool` spins each ESC over DShot — from the Wi-Fi web UI (**Spin test**) or the USB-serial API:

```
arm <i> [normal|bidir]    # arm ESC i (AUTO picks bidir for Bluejay/JESC, else normal DShot)
throttle <i> <0..2000>    # one-way throttle (armed)
thrust <i> <-1000..1000>  # reversible/3D: signed thrust, 0 = stop (armed)
tele <i>                  # rpm | volts | amps | tempC | stress   (bidir firmware only)
disarm <i>                # stop + release the ESC
```

Arming streams zero throttle for ~3 s (BLHeli-S won't spin until it has been armed), so wait for the
start-up beeps before applying throttle. A **deadman** re-zeros the throttle if no command arrives
within 500 ms — resend regularly to keep it spinning (the web UI heartbeats automatically).

**Firmware & telemetry.** Stock BLHeli-S understands only normal DShot (throttle, no telemetry).
eRPM + temperature telemetry and reversible support work best on **Bluejay** (or JESC); its motor
PWM frequency (24 / 48 / 96 kHz) is chosen by *which hex you flash*. Get Bluejay from
[github.com/bird-sanctuary/bluejay](https://github.com/bird-sanctuary/bluejay). The Pico exposes a
generic **per-thruster** driver — cmd_vel→thruster mixing stays on the host/Pi (RL/sim-friendly).

## Wi-Fi web tool (SETUP mode)

In SETUP mode (the default), `esc_tool` brings up a Wi-Fi Access Point. Connect your phone/PC to
the SSID `pico-esc-tool` and open `http://192.168.4.1` for a configurator-style browser UI: scan
ESCs, read/edit settings, run a per-thruster spin test with live telemetry, and **flash firmware**
(upload a `.hex`) — no cables. Change `AP_SSID` / `AP_PASS` (and `MODE_PIN`) at the top of
`src/apps/esc_tool.cpp`.

Browser flashing is the same app-only, layout/MCU-guarded flow as the CLI (bootloader preserved,
firmware defaults applied); it runs page-by-page in the background with a progress readout. Pick the
`.hex`, choose the ESC, and Flash (tick *force* to override the compat guard).

Wi-Fi is a **surface/bench** affordance — 2.4 GHz does not travel through water, so a deployed
underwater craft is driven over the tether/host, not Wi-Fi. (Pico W only; radio and DShot both use
PIO — validate the combination on the bench.)

## Safety

- **Flash is app-only.** The 1-wire bootloader (top of flash) is never overwritten — it's how the
  tool talks to the ESC — so a failed flash is recoverable by re-flashing.
- **Compatibility guard.** `flash` refuses a HEX whose layout tag or MCU signature doesn't match
  the connected ESC (wrong FET map / wrong chip), unless `--force`.
- Every write is **read-back verified** on the device.

## PlatformIO environments

| Env | Purpose |
|---|---|
| `esc_tool` | **The tool** — unified firmware (USB serial CLI + Wi-Fi web UI + DShot spin). |
| `picow` / `pico` | DShot control + telemetry baseline app (`src/main.cpp`). |
| `spike_*` | Standalone bring-up/diagnostic firmwares for the bootloader work. |

`esc_tool` builds on the shared bootloader/session/drive logic in `src/apps/esc_session.h`.

## Layout

```
platformio.ini          build environments
src/main.cpp            DShot + telemetry baseline app (USB-CDC command loop)
src/apps/esc_tool.cpp   unified firmware (USB serial + Wi-Fi web + DShot spin)
src/apps/esc_session.h  shared bootloader session + drive/spin (used by esc_tool)
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

**Proven on hardware** (EFM8BB21, ReadyToSky 45A/30A): bootloader connect / read / write / flash
with the compat guard and auto-default-config; DShot spin with eRPM + EDT telemetry on **Bluejay**
and throttle-only spin on **stock BLHeli-S** (auto-selected); reversible / 3D signed thrust; and the
Wi-Fi web configurator + spin test (Wi-Fi AP and DShot coexist on the Pico W's PIO). Next: multi-
thruster simultaneous drive, firmware flashing from the browser, and RPM filtering / telemetry→
thrust mapping on the host.

## License

Copyright (C) 2026 satoimotaro. **GPL-3.0-or-later** — see [`LICENSE`](LICENSE).

Third-party: DShot / telemetry via
[pico-bidir-dshot](https://github.com/bastian2001/pico-bidir-dshot) (GPL-3.0). The BLHeli-S 1-wire
protocol was implemented with reference to Betaflight and BLHeli_S (both GPL). BLHeli-S firmware
images belong to their authors ([github.com/bitdump/BLHeli](https://github.com/bitdump/BLHeli))
and are **not** distributed here.
