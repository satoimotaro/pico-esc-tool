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

**"BlueGill"**, where it appears below, is our fork of the [Bluejay](https://github.com/mathiasvr/bluejay)
ESC firmware, which adds low-speed **forced-commutation sine drive** and a **sine↔BEMF crossover** so a
low-KV thruster can creep and hold below the speed where sensorless 6-step can lock. It is a separate
project; this tool works with stock Bluejay and BLHeli-S too, and the BlueGill-only features are the ones
labelled S1/S2/S3 or `sine_*`.

> **Before you touch the bench or the vehicle, read [`docs/README.md`](docs/README.md).** It carries the
> six behaviours that will bite you (telemetry lies below the crossover seam; zero throttle is a brake),
> which numbers are measured versus fitted, and the current hardware state.

## Quick start

```
pio run -e main -t upload                                      # build + flash the firmware (default env)
python host/esctool.py list                                   # scan & list connected ESCs
python host/esctool.py read 0 -o config.yaml                  # export ESC 0's config to YAML
python host/esctool.py set 0 motor_direction=Reversed
python host/esctool.py flash 0 firmware.hex --yes             # flash matching BLHeli-S/Bluejay firmware
```

The Pico is auto-detected (USB VID 2E8A). For a board with no USB host, `pio run -e main_uart` puts
the same CLI on UART0 (GP0 TX / GP1 RX, 921600); behind a TCP relay, every host tool takes
`--port socket://<ip>:<port>`. Wiring — signal pins, mode pin, Wi-Fi, motor poles — lives
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

- **Add an ESC or move it to another GPIO:** `pins <gpio> <gpio> ...` (ESC-index order, 1..8 pins),
  then reboot the Pico; `pins` shows the list, `pins default` goes back to `ESC_SIGNAL_PINS`
  (`esc_config.h`). `main.cpp` declares a `Thruster` for each of the 8 possible ESCs; ESC 2..7 run
  RAW until you give them a profile. Which pins have an ESC on them is learned, not configured
  ([`docs/esc-lifecycle.md`](docs/esc-lifecycle.md) "Signal pins and presence").
- **Standalone ROV (no PC tool):** skip `EscTool` and drive the ESCs straight from `loop()` —
  `t->setRpm(mix[i]); t->poll(); escs::spinPoll();`. The `EscTool` module is only the operator surface.
- `EscTool` = config/flash + USB-serial CLI + Wi-Fi UI; `Thruster` = one ESC (config + drive +
  velocity). The proven dual-core DShot/bootloader engine underneath is `src/apps/esc_session.h`.

## Driving thrusters (USB serial)

```
arm <i> [normal|bidir]      # arm ESC i (AUTO: bidir for Bluejay/JESC, else normal DShot)
throttle <i> <0..2000>      # RAW one-way throttle          (armed)
thrust  <i> <-1000..1000>   # RAW reversible/3D signed thrust, 0 = stop (full scale at +-953)
rpm     <i> <mech-rpm>      # CLOSED-LOOP velocity target (signed); 0 = clean stop; needs a profile
gain    <i> kp|ki|kd|dtau|trim|slew|dob|dobtau|dobmax|dobsettle <v>   # tune the closed loop live
tele    <i>                 # rpm | volts | amps | tempC | stress   (bidir firmware only)
disarm  <i>                 # stop + release
cfg     [show|save|load|clear]   # persist/restore motor + gains + DOB + vbatt cal on the Pico's flash
curve   <i> [begin <n>|add <t> <r> [s|l]|commit <pp> <up> <dn>]   # upload a measured FF curve
pwm     <i> <1000..2000|stop>          # servo PWM (50 Hz) instead of DShot; see the web tool below
pwmch   <ch> <1000..2000|stop>         # the same by motor-test channel, value mapped per ESC (needs mtmode)
mtmode  <mask>                        # motor test: bit c = channel c sends 3D values (web tool)
pins    [<gpio>... | default]         # the ESC signal pins; a change is used from the next boot
```

Arming streams zero throttle ~3 s (BLHeli-S won't spin until armed); a **deadman** re-zeros if no
command arrives within 500 ms. A Bluejay/BLHeli-derived bidir ESC that stalls (eRPM 0 under throttle
for 1.5 s) is armed again and held at 0 until it is sent 0. `throttle`/`thrust` are RAW (open loop);
`rpm` engages the closed loop and the ESC holds the target speed. Stock BLHeli-S = normal DShot (throttle only, no telemetry);
**Bluejay/JESC** = bidirectional DShot with eRPM/EDT telemetry and reversible support.

## Closed-loop RPM control

Signed target mechanical RPM → signed DShot thrust: a feed-forward from the motor's calibrated curve
plus a PI trim on the telemetry eRPM (with an optional derivative-on-measurement term, `kd`), whose
authority fades where telemetry goes stale. It is ESC-agnostic (works on stock Bluejay using only
standard bidir-DShot telemetry). The control law is `lib/vel_control/` (portable, host-unit-tested);
design notes in `docs/velctl-generalization.md`.

`rpm 0` is a **clean stop**: the controller holds neutral while staying armed, and (on BlueGill sine
firmware) the ESC parks itself so the rotor really stops and the next non-zero target restarts warm —
no re-arm. Gains are plant-dependent; set them per motor in `main.cpp` (`esc1.vc.kd = ...`) or live
with `gain`. `slew_rpm_s` trades overshoot for rise time (a gentler slew overshoots less).

**Evaluate/tune the loop:** `host/rpm_sweep.py` drives a stepped RPM schedule, logs the tracking at
the full telemetry rate (and an AS5600 shaft encoder as ground-truth, if present), and writes a
self-contained HTML report with per-step metrics (steady-state error, overshoot, rise, settle):

```
python host/rpm_sweep.py 1 --out host/reports/run       # sweep ESC 1, write run.json + run.html
python host/rpm_sweep.py 1 --set slew=1500 --set kd=0.01 # push gains first, then sweep
```

**Per-motor profiles are generated from calibration, not hand-typed.** The YAML velcal profiles in
`host/profiles/` are the single source of truth; regenerate the firmware header after a velcal:

```
python host/gen_profile_header.py            # host/profiles/vel_*.yaml -> src/apps/profiles_gen.h
```

Each `vel_<name>.yaml` becomes `profiles::M_<NAME>` + `M_<NAME>_GAINS` for a `Thruster` to use.

A measured curve can also be pushed **at runtime**, with no recompile — the fastest path from a velcal
to a working loop, and what the disturbance observer needs (it inverts the forward map, so a curve that
is off reads model error as load):

```
python host/curvepush.py 1 host/profiles/<name>.yaml --motor 350,7,11.2 --gains --save
```

> **A measured curve is only valid at the `sine_cross_up` it was measured at.** BlueGill's
> `cross_rescale_duty` maps the 6-step demand to an affine function of throttle *relative to* Cross_Up,
> so moving that threshold shifts the whole command→speed mapping. Bench-measured on the 350KV with
> pure feed-forward: the same profile is **+0.3 %** at its own `cross_up=36` and **+27…+46 %** at
> `cross_up=34`, while the parametric model is the exact mirror image (−0.6 % at 34, −16…−28 % at 36).
> `curvepush` reads the ESC and refuses on a mismatch; `--apply-crossover` writes the profile's bytes.

`cfg save` persists the curve (and the motor identity, gains and `vcal` scale) to the Pico's flash;
without it they are lost on the next reset.

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

## ESC Web Configurator (`web/`)

One page in the browser (Chrome / Edge, Web Serial) that stands in for both esc-configurator and the
AM32 configurator: read and edit settings for **BLHeli_S 16.x, Bluejay, BlueGill and AM32** side by
side, apply **presets** (the same built-ins as the firmware's `preset` command, plus your own, kept in
the browser and exported as JSON), run a **motor test**, and **flash** a local `.hex` (layout/MCU
guarded, settings carried over). It talks Betaflight MSP + the BLHeli 4-way passthrough, so it works
with this firmware or with any Betaflight flight controller. Static files, no build step:
`python3 -m http.server -d web` and open `http://localhost:8000`, or the GitHub Pages copy
(`.github/workflows/pages.yml`). Known-good settings per motor: [`docs/motor-settings.md`](docs/motor-settings.md).
Connected to this firmware, the settings tab also shows the ESC signal pins and edits them (the
`pins` command; the Pico uses a saved list after it restarts). ESC k is signal pin k everywhere (the
CLI index, the configurators' ESC and motor k), whether an ESC is on it or not; an empty pin costs
at most 0.5 s in a read (an ESC about 1.5 s to enter its bootloader), and a settings write first checks that the ESC is still the one read.

The motor test re-sends its values every 100 ms and the firmware stops the motors after 2 s of MSP
silence, after the USB port closes, or when a passthrough starts; the page also stops on Esc/Space,
on leaving the tab, and when hidden.

On this firmware the motor test can also send **servo PWM** (50 Hz, 1000–2000 µs) instead of DShot,
for firmwares that take it: AM32 and stock BLHeli_S, not Bluejay (DShot only). The CLI form is
`pwm <i> <µs|stop>` (1000..2000; `0`, junk and anything below 1000 are refused: a typo must not become a width the ESC reads as full reverse).
The ESC must have been read first (`scan`, a configurator read, or a DShot arm), because the stop pulse
is 1500 µs on a 3D ESC; the first 5 s after switching hold that stop pulse so the ESC re-detects the
protocol and arms, and after that a width passes only once one stop width has been sent. Fed like
`throttle`: 500 ms without a value falls back to the stop pulse (also while the board is busy), and
3 s without one after the arm the pin leaves PWM. Bootloader commands are refused while a PWM pin
runs. There is no telemetry on PWM. The ESC lifecycle (hot-plug, async reads), how rev1 carries the
rev0 rules into it, and the known limits of both: [`docs/esc-lifecycle.md`](docs/esc-lifecycle.md).

## Position control (`host/posctl.py`)

A host-side closed-loop **shaft-position** servo using an **AS5600** encoder on the Pico (I2C0,
`SDA→GP16 SCL→GP17`, addr `0x36`). Targets BlueGill S1/S2 sine mode so the rotor can creep and hold at
low speed. `python host/posctl.py move --deg 720` (add `--dry-run` for a hardware-free smoke test).
The encoder is a **calibration/position instrument** — the RPM closed loop above is sensorless.

## ESC soft-sensor (voltage + load, no extra sensors)

Infer **effective supply voltage** and a **relative load/thrust** signal from the DShot telemetry RPM plus
the motor model — no voltage, current or force sensor. `V_eff = rpm/(KV·duty) = V_supply − I·R/duty`, so
`V_eff ≈ V_supply` at light load and droops under load.

```
motor 1 350 7 11.1        # set the motor model (kv pp v), then spin into 6-step with `rpm ...`
sense 1                   # vest|vref|load|vbatt|valid|why|... (see docs for the full field list)
sense 1 zero              # baseline the no-load reference (load = vref - vest thereafter)
sense 1 vcal 11.2         # calibrate vbatt to a known battery voltage
cfg save                  # persist motor/gains/DOB/vcal to flash (otherwise RAM-only)
```

Good for a rough battery gauge and for fouled-prop / entanglement detection. Valid only inside a
deliberately narrow window (command above the derived handoff, duty below saturation, held 0.3 s);
outside it `valid=0` and `why=` names the blocking condition, because a number you cannot tell apart from
a good one is worse than no number. **`--cross-up` is required** on `softsensor`/`obschar`/`duty_sweep`/
`sim_obs` — there is no safe default and the wrong one is a silent 13–45 % error.
→ [`docs/soft-sensor.md`](docs/soft-sensor.md)

### The measured plant

`host/pico_esc/plant.py` + `measured.py` are the plant model: structure fixed by physics, numbers in a
provenance-tagged inventory keyed by mechanical configuration (propeller on vs bare shaft — never mix
them). Moving to water changes exactly one callable (`PlantParams.drag`), and `PlantParams` is frozen so
that is a property of the type rather than a convention.

```
python3 host/plantid.py provenance      # where every number came from, and what each preset borrows
python3 host/plantid.py validate --all  # replay the committed fixtures offline, print the out-of-sample table
python3 host/plantid.py validate --pair # the controlled bare-shaft vs propeller comparison
```

→ the plant inventory (measured separately per installation; see `host/pico_esc/plant.py` for the structure and `host/plantid.py --help` for how to build one)

## Requirements

- **Hardware:** RP2040 board; each ESC's signal wire on a GPIO (defaults GP10, GP11; the panel
  GP16..19; any 1..8 GPIOs with `pins`) with common ground. All wiring in `src/apps/esc_config.h`.
- **Firmware:** [PlatformIO](https://platformio.org/) (earlephilhower Arduino-Pico core, fetched
  automatically). Envs: **`main`** (default), `main_uart` (CLI on UART0), `main_panel` (the panel board).
  The legacy `esc_tool` env is frozen at tag `legacy-esc-tool`.
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
web/                     ESC Web Configurator (static page: settings, presets, motor test, flash)
host/esctool.py          the PC CLI                host/posctl.py   position controller
host/rpm_sweep.py        velocity-loop sweep + HTML evaluation report (rpm_sweep_report.py)
host/gen_profile_header.py   YAML profile -> C++ header codegen
host/profiles/           calibrated YAML profiles (source of truth for firmware curves)
host/pico_esc/measured.py    the measured plant inventory (one provenance record per number)
host/pico_esc/plant.py       the plant model (firmware / rotor / load, in three separable layers)
host/plantid.py              offline fit + out-of-sample validation of the plant (no hardware)
host/tests/data/             committed bench fixtures + plant_fixtures.json (the manifest)
docs/README.md           START HERE — operational facts, confidence tiering, doc map
docs/                    the plant inventory (kept per-installation), soft-sensor.md, velocity-control.md, protocol-notes.md
```

Legacy standalone bring-up apps (`dshot_demo`, `spike_*`, `encoder_test`) live on branch
`archive/legacy-apps`.

## License

Copyright (C) 2026 satoimotaro. **GPL-3.0-or-later** — see [`LICENSE`](LICENSE). DShot/telemetry via
[pico-bidir-dshot](https://github.com/bastian2001/pico-bidir-dshot) (GPL-3.0); the BLHeli-S 1-wire
protocol was implemented with reference to Betaflight and BLHeli_S (both GPL). Firmware images belong
to their authors and are **not** distributed here.
