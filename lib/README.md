# lib/ — controller modules

PlatformIO auto-discovers each subfolder here as a library. Keep each module
self-contained (its own headers/sources) so it can be reused and unit-tested.

| Module | Responsibility | Status |
|--------|----------------|--------|
| `esc_dshot/`     | Channel abstraction over `pico-bidir-dshot` (N ESCs, arm state, mode switch) | stub (A0 uses lib directly in `src/main.cpp`) |
| `esc_telemetry/` | Normalize EDT frames (eRPM, V, A, temp, stress, status) into a struct | stub |
| `rpm_filter/`    | eRPM → mechanical RPM (÷ pole pairs), dropout/checksum rejection, smoothing | stub |
| `blheli_bl/`     | **BLHeli-S 1-wire bootloader client** (connect, device ID, read/write mem)  **[core, hard]** | scaffold + `PROTOCOL.md` (spike, research in progress) |
| `esc_setup/`     | BLHeli-S EEPROM parameter decode/encode (on `blheli_bl`)  **[core, hard]** | scaffold + `EEPROM.md` |
| `esc_flash/`     | Firmware `.hex` program/verify (on `blheli_bl`)  **[core, hard]** | scaffold (Phase A1) |
| `pc_iface/`      | Host command parser/dispatch (USB-C CDC first) | stub (A0 inline in `src/`) |

`blheli_bl` is the shared foundation: `esc_setup` (EEPROM params) and `esc_flash`
(firmware) both do their I/O through it. A0 spikes exercise it via `src/apps/spike_*.cpp`
(build envs `spike_flash` / `spike_setup`). Protocol constants: `blheli_bl/PROTOCOL.md`.

External dependency: **pico-bidir-dshot** (GPL-3.0) via `lib_deps` in `platformio.ini`
— provides DShot TX + bidirectional eRPM + Extended DShot Telemetry decoding.
