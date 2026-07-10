# lib/ — controller modules

PlatformIO auto-discovers each subfolder here as a library. Each module is self-contained
(its own headers/sources) so it can be reused and tested.

| Module | Responsibility | Status |
|--------|----------------|--------|
| `blheli_bl/` | **BLHeli-S 1-wire bootloader client** — connect, device ID, read/write flash, erase, run. `PROTOCOL.md` is the wire reference. | proven on HW |
| `esc_setup/` | BLHeli-S config decode/encode + full-page read-modify-write (on `blheli_bl`). | proven on HW |
| `esc_flash/` | Intel-HEX parse, program/verify, and layout/MCU compatibility check (on `blheli_bl`). | proven on HW |

`blheli_bl` is the shared foundation: `esc_setup` (config) and `esc_flash` (firmware) both do
their 1-wire I/O through it. The unified firmware `src/apps/esc_host.cpp` uses these to serve the
`host/esctool.py` CLI; the `src/apps/spike_*.cpp` builds are standalone bring-up/diagnostics.

External dependency: **pico-bidir-dshot** (GPL-3.0) via `lib_deps` in `platformio.ini` — DShot TX +
bidirectional eRPM + Extended DShot Telemetry, used for DShot control and bootloader-entry priming.
