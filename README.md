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
- **Multi-ESC** — one signal pin per ESC (`ESC_SIGNAL_PINS` in `src/apps/esc_config.h`); target an index or `all`.
- **Firmware-aware thruster drive** — the spin path detects the ESC firmware and picks the DShot
  variant: **normal DShot** for stock BLHeli-S (throttle only), **bidirectional DShot** for
  Bluejay/JESC (with telemetry). Arms the ESC, then holds throttle with a deadman auto-stop.
- **Reversible / 3D** — if the ESC is configured reversible, throttle becomes a **signed thrust**
  (−full … 0 = stop … +full); one-way ESCs use a plain 0–100% throttle.
- **Live telemetry** (bidir firmware) — eRPM always; temperature + stress via EDT; voltage/current
  when the ESC has the sensors. DShot via
  [pico-bidir-dshot](https://github.com/bastian2001/pico-bidir-dshot).
- **Closed-loop position control** (`host/posctl.py`) — an AS5600 encoder on the Pico + a host
  cascade controller (position → velocity → thrust), with direction auto-cal and full safety
  aborts. See [Position control](#position-control-hostposctlpy--as5600-encoder).

## Requirements

- **Hardware:** RP2040 board; each ESC's signal wire on a GPIO (defaults **GP10**, plus **GP11** for
  a 2nd ESC) with a common ground. All wiring — signal pins, mode pin, Wi-Fi, motor poles — is
  configured in one place, `src/apps/esc_config.h`. See `construction/wiring/`.
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
enc                       # read the AS5600 encoder: raw|ang|deg|md|ml|mh|agc|mag
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

## Position control (`host/posctl.py` + AS5600 encoder)

Closed-loop **shaft-position** control: an **AS5600** magnetic encoder on the Pico's I2C0 feeds a
host-side cascade controller (outer position → inner velocity → signed `thrust`). The Pico reads
the encoder (`enc` command); the control loop runs on the PC.

**Wiring** (AS5600 → Pico): `SDA→GP16`, `SCL→GP17`, `VCC→3V3`, `GND→GND`, `DIR→GND` (I2C addr
`0x36`). Check it first with `enc` — you want `md=1` (magnet detected), `ml=mh=0`, and `agc` mid-range.

**ESC setup:** position control needs **reversible** drive, so set the ESC to 3D mode and use bidir
(Bluejay) firmware:

```
python host/esctool.py set 1 motor_direction=Bidirectional      # reversible thrust
# (re-apply your tuned profile after any firmware flash — flashing resets the config page)
```

**Usage:**

```
python host/posctl.py move --deg 720                 # rotate to +720° from the current pose
python host/posctl.py step --seq 360,-360,720        # relative moves, one after another
python host/posctl.py hold --deg 0                   # actively hold a position
python host/posctl.py move --deg 360 --dry-run       # simulated motor, no hardware (safe smoke test)
```

Key flags: `--esc-index 1`, `--tol 12` (deg deadband), `--tmax 300` / `--tmin 40` (thrust limits),
`--vmax 400` (deg/s cap), `--kp/--kd/--kpv/--ki` (gains), `--max-secs` / `--max-revs` / `--vel-abort`
(runaway aborts), `--csv` (log to `host/reports/`). On start it **auto-calibrates direction** (a
brief probe spin to learn whether `+thrust` increases or decreases the encoder count — no need to
match wiring/DIR polarity); override with `--invert-encoder` / `--no-autocal`.

**Safety:** the loop keep-alives the ESC under the 500 ms deadman, guards the encoder unwrap, and
**always disarms** on completion, abort, or Ctrl-C. It aborts + disarms on lost magnet, stuck/
implausible encoder reads, over-velocity, wrong-way runaway, or the time/rev limits.

> **6-step limitation (important).** On stock 6-step/BEMF firmware the motor can't rotate below
> ~185 RPM, so it can't creep to a target: `hold` is excellent (friction holds the rotor, ~0.1°),
> but a **`move` overshoots by ~2 revolutions** before settling within `--tol`. Precise low-speed
> positioning needs an open-loop **sine drive mode** in the ESC firmware (the BlueGill #3 roadmap);
> `posctl.py` is the instrument that quantifies this and will command that mode once it lands.

## Wi-Fi web tool (SETUP mode)

In SETUP mode (the default), `esc_tool` brings up a Wi-Fi Access Point. Connect your phone/PC to
the SSID `pico-esc-tool` and open `http://192.168.4.1` for a configurator-style browser UI: scan
ESCs, read/edit settings, run a per-thruster spin test with live telemetry, and **flash firmware**
(upload a `.hex`) — no cables. The AP SSID/password and the mode pin are set in
`src/apps/esc_config.h`.

Browser flashing is the same app-only, layout/MCU-guarded flow as the CLI (bootloader preserved,
firmware defaults applied); it runs page-by-page in the background with a progress readout. Pick the
`.hex`, choose the ESC, and Flash (tick *force* to override the compat guard).

**Firmware library.** An uploaded `.hex` can be **Saved to library** — it persists on the Pico's
LittleFS partition (nothing is stored in this repo) and shows up in a list tagged by layout. From
then on you just tap **Flash to ESC _n_** — no re-uploading, which is handy from a phone. The list
flags entries whose layout doesn't match the selected ESC. (Set the partition size with
`board_build.filesystem_size` in `platformio.ini`.)

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
| `esc_tool` | **The tool** — unified firmware (USB serial CLI + Wi-Fi web UI + DShot spin). *The default build.* |
| `dshot_demo` | Standalone DShot drive demo (`src/apps/dshot_demo.cpp`); Pico W by default, set `board = rpipico` for a plain Pico. |
| `spike_*` | Standalone bring-up/diagnostic firmwares for the bootloader work. |

`esc_tool` builds on the shared bootloader/session/drive logic in `src/apps/esc_session.h`.

## Layout

```
platformio.ini          build environments
src/apps/esc_config.h   hardware config — EDIT THIS for your wiring (pins, Wi-Fi, motor)
src/apps/esc_tool.cpp   the tool: unified firmware (USB serial + Wi-Fi web + DShot spin)
src/apps/esc_session.h  shared bootloader session + drive/spin (used by esc_tool)
src/apps/dshot_demo.cpp standalone DShot drive demo (pico / picow envs)
src/apps/spike_*.cpp    standalone bring-up / diagnostic firmwares
host/esctool.py         the PC CLI
host/posctl.py          closed-loop shaft-position controller (AS5600 encoder)
host/autocal.py         per-thruster low-speed auto-calibration
host/profiles/          example YAML profiles
lib/blheli_bl/          BLHeli-S 1-wire bootloader client  (PROTOCOL.md = reference)
lib/esc_setup/          config read / write (read-modify-write a flash page)
lib/esc_flash/          Intel-HEX parse, program/verify, compatibility check
construction/wiring/    how to wire an ESC signal line to the Pico
docs/                   notes; lib/blheli_bl/PROTOCOL.md is the bootloader reference
```

## DShot demo (`dshot_demo` env)

`src/apps/dshot_demo.cpp` drives one ESC over bidirectional DShot from the serial monitor (115200,
newline): `E` enable telemetry, `A` arm, a number = throttle (0–2000), `D` disarm, `C3` beacon (when
stopped), `?` reprint header. It's a minimal standalone reference (the full tool is `esc_tool`); pins
and motor poles come from `src/apps/esc_config.h`.

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
