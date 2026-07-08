# lib/ — controller modules

PlatformIO auto-discovers each subfolder here as a library. Keep each module
self-contained (its own headers/sources) so it can be reused and unit-tested.

| Module | Responsibility | Status |
|--------|----------------|--------|
| `esc_dshot/`     | Channel abstraction over `pico-bidir-dshot` (N ESCs, arm state, mode switch) | stub (A0 uses lib directly in `src/main.cpp`) |
| `esc_telemetry/` | Normalize EDT frames (eRPM, V, A, temp, stress, status) into a struct | stub |
| `rpm_filter/`    | eRPM → mechanical RPM (÷ pole pairs), dropout/checksum rejection, smoothing | stub |
| `esc_setup/`     | BLHeli-S EEPROM parameter read/write (1-wire)  **[core, hard]** | stub (Phase A1) |
| `esc_flash/`     | BLHeli-S 1-wire bootloader flashing  **[core, hard]** | stub (Phase A1) |
| `pc_iface/`      | Host command parser/dispatch (USB-C CDC first) | stub (A0 inline in `src/`) |

External dependency: **pico-bidir-dshot** (GPL-3.0) via `lib_deps` in `platformio.ini`
— provides DShot TX + bidirectional eRPM + Extended DShot Telemetry decoding.
