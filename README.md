# pico-esc-tool

An **RP2040 (Pico / Pico W)** tool for **BLHeli-S ESCs**: configure them, flash firmware, and drive
them over DShot with telemetry — including **closed-loop RPM control on-device** — **no flight
controller required.** Built for underwater ROV thrusters (many ESCs from one PC/SBC), but works with
any BLHeli-S SiLabs ESC. Developed on **ReadyToSky 45A / 30A** (SiLabs **EFM8BB21**), Bluejay firmware.

Two parts:

- **`main`** — one Pico W firmware, composed of objects: you declare your ESCs in `src/main.cpp` and it
  gives you config/flash over the 1-wire bootloader **and** DShot drive (RAW thrust *or* closed-loop
  RPM), over USB serial and (in SETUP mode) a Wi-Fi web UI. No reflashing to switch jobs.
- **`esctool`** (`host/esctool.py`) — a BLHeli-Configurator-like **CLI** on the PC: list ESCs, read /
  write settings (YAML profiles), and flash firmware, with a layout/MCU **compatibility guard**.

## Quick start

```
pio run -e main -t upload                                      # build + flash the firmware (default env)
python host/esctool.py list                                   # scan & list connected ESCs
python host/esctool.py read 0 -o config.yaml                  # export ESC 0's config to YAML
python host/esctool.py set 0 motor_direction=Reversed
python host/esctool.py flash 0 firmware.hex --yes             # flash matching BLHeli-S/Bluejay firmware
```

The Pico is auto-detected (USB VID 2E8A). Wiring — signal pins, mode pin, Wi-Fi, motor poles — lives
in one file, `src/apps/esc_config.h`; see `construction/wiring/`.

## The firmware: declare your ESCs in `main.cpp`

The firmware is a composition root. Each ESC is a `Thruster` object carrying its **own** config —
DShot bitrate, motor pole count, calibrated speed curve, and PI gains — and you compose them:

```cpp
static Thruster esc0(&profiles::M_LINEAR, ESC_DSHOT_KBAUD, ESC_MOTOR_POLES);        // RAW only
static Thruster esc1(&profiles::M_930KV_12N14P_6STEP, /*dshotKbaud=*/300, /*poles=*/14);
static Thruster* THRUSTERS[] = { &esc0, &esc1 };
static EscTool  tool(THRUSTERS, 2);            // the config/flash + serial + Wi-Fi surface (optional)

void setup() {
    for (uint8_t i = 0; i < 2; i++) THRUSTERS[i]->bind(i);
    esc1.applyGains(profiles::M_930KV_12N14P_6STEP_GAINS);   // per-motor gains from the profile
    tool.begin();
}
void loop() { tool.poll(); }                   // serial + Wi-Fi + each ESC's RPM loop
```

- **Add an ESC:** add a pin to `ESC_SIGNAL_PINS` (`esc_config.h`) and a `Thruster` here.
- **Standalone ROV (no PC tool):** skip `EscTool` and drive the ESCs straight from `loop()` —
  `t->setRpm(mix[i]); t->poll(); escs::spinPoll();`. The `EscTool` module is only the operator surface.
- `EscTool` = config/flash + USB-serial CLI + Wi-Fi UI; `Thruster` = one ESC (config + drive +
  velocity). The proven dual-core DShot/bootloader engine underneath is `src/apps/esc_session.h`.

## Driving thrusters (USB serial)

```
arm <i> [normal|bidir]      # arm ESC i (AUTO: bidir for Bluejay/JESC, else normal DShot)
throttle <i> <0..2000>      # RAW one-way throttle          (armed)
thrust  <i> <-1000..1000>   # RAW reversible/3D signed thrust, 0 = stop
rpm     <i> <mech-rpm>      # CLOSED-LOOP velocity target (signed); needs a calibrated profile
gain    <i> kp|ki|trim|slew <v>   # tune the closed loop live
tele    <i>                 # rpm | volts | amps | tempC | stress   (bidir firmware only)
disarm  <i>                 # stop + release
```

Arming streams zero throttle ~3 s (BLHeli-S won't spin until armed); a **deadman** re-zeros if no
command arrives within 500 ms. `throttle`/`thrust` are RAW (open loop); `rpm` engages the closed loop
and the ESC holds the target speed. Stock BLHeli-S = normal DShot (throttle only, no telemetry);
**Bluejay/JESC** = bidirectional DShot with eRPM/EDT telemetry and reversible support.

## Closed-loop RPM control

Signed target mechanical RPM → signed DShot thrust: a feed-forward from the motor's calibrated curve
plus a PI trim on the telemetry eRPM, whose authority fades where telemetry goes stale. It is
ESC-agnostic (works on stock Bluejay using only standard bidir-DShot telemetry). The control law is
`lib/vel_control/` (portable, host-unit-tested); design notes in `docs/velctl-generalization.md`.

**Per-motor profiles are generated from calibration, not hand-typed.** The YAML velcal profiles in
`host/profiles/` are the single source of truth; regenerate the firmware header after a velcal:

```
python host/gen_profile_header.py            # host/profiles/vel_*.yaml -> src/apps/profiles_gen.h
```

Each `vel_<name>.yaml` becomes `profiles::M_<NAME>` + `M_<NAME>_GAINS` for a `Thruster` to use.

## Host CLI reference

| Command | What it does |
|---|---|
| `list` | Scan all pins, print each ESC (signature, layout, name, firmware). |
| `read <i\|all> [-o f.yaml]` | Read config; print YAML or write it. |
| `set <i\|all> key=value ...` | Change settings (enum names OK, e.g. `motor_direction=Reversed`). |
| `apply <i\|all> profile.yaml` | Apply a profile's `settings:` block. |
| `flash <i> file.hex --yes` | Compat-check then erase/program/verify the app + apply defaults (`--force` to override). |
| `run` / `disconnect` `[i]` | End the bootloader session and restart the ESC(s). |

Config commands hold the ESC in a bootloader session (motor off) and don't restart it between
commands (batch-friendly). Flash is **app-only** (the bootloader is never overwritten, so a bad flash
is recoverable) and **refused on a layout/MCU mismatch** unless `--force`; every write is read-back
verified. Get Bluejay from [bird-sanctuary/bluejay](https://github.com/bird-sanctuary/bluejay) or
BLHeli-S from [bitdump/BLHeli](https://github.com/bitdump/BLHeli) — firmware images are not bundled.

## Wi-Fi web tool (SETUP mode)

At boot the mode pin (or the `mode` command) picks **SETUP** (Wi-Fi AP on) or **DRIVE** (radio off).
In SETUP, connect to the SSID `pico-esc-tool` → `http://192.168.4.1` for a browser UI: scan ESCs,
edit settings, run a per-thruster spin/RPM test with live telemetry, and flash firmware (upload a
`.hex`, or save it to the Pico's on-device library and re-flash with one tap). Same app-only,
layout-guarded flow as the CLI. Wi-Fi is a bench affordance — 2.4 GHz doesn't travel underwater, so a
deployed craft is driven over its tether/host.

## Position control (`host/posctl.py`)

A host-side closed-loop **shaft-position** servo using an **AS5600** encoder on the Pico (I2C0,
`SDA→GP16 SCL→GP17`, addr `0x36`). Targets BlueGill S1/S2 sine mode so the rotor can creep and hold at
low speed. `python host/posctl.py move --deg 720` (add `--dry-run` for a hardware-free smoke test).
The encoder is a **calibration/position instrument** — the RPM closed loop above is sensorless.

## Requirements

- **Hardware:** RP2040 board; each ESC's signal wire on a GPIO (defaults GP10, GP11) with common
  ground. All wiring in `src/apps/esc_config.h`.
- **Firmware:** [PlatformIO](https://platformio.org/) (earlephilhower Arduino-Pico core, fetched
  automatically). Envs: **`main`** (default), `esc_tool` (legacy monolithic fallback).
- **Host:** Python 3.8+ and `pyserial` (`pip install pyserial`; `pyyaml` optional).

## Layout

```
src/main.cpp             the firmware composition root — DECLARE YOUR ESCs here
src/apps/esc_config.h    hardware config — EDIT for your wiring (pins, Wi-Fi, motor)
src/apps/thruster.h      Thruster: one ESC (config + drive + closed-loop velocity)
src/apps/esc_tool_app.h  EscTool: the config/flash + serial + Wi-Fi surface (references Thrusters)
src/apps/esc_session.h   the dual-core DShot + 1-wire bootloader engine (HAL)
src/apps/profiles.h      per-motor curves — includes the generated profiles_gen.h
src/apps/profiles_gen.h  GENERATED from host/profiles/*.yaml (do not edit by hand)
lib/vel_control/         portable closed-loop velocity controller (host-unit-tested)
lib/{blheli_bl,esc_setup,esc_flash}/   bootloader client / config codec / HEX flash
host/esctool.py          the PC CLI                host/posctl.py   position controller
host/gen_profile_header.py   YAML profile -> C++ header codegen
host/profiles/           calibrated YAML profiles (source of truth for firmware curves)
docs/                    design notes (velctl-generalization.md, etc.)
```

Legacy standalone bring-up apps (`dshot_demo`, `spike_*`, `encoder_test`) live on branch
`archive/legacy-apps`.

## License

Copyright (C) 2026 satoimotaro. **GPL-3.0-or-later** — see [`LICENSE`](LICENSE). DShot/telemetry via
[pico-bidir-dshot](https://github.com/bastian2001/pico-bidir-dshot) (GPL-3.0); the BLHeli-S 1-wire
protocol was implemented with reference to Betaflight and BLHeli_S (both GPL). Firmware images belong
to their authors and are **not** distributed here.
