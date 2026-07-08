# esc_telemetry

Normalize Extended DShot Telemetry (EDT) frames from `pico-bidir-dshot` into a single
per-channel struct: `{ erpm, voltage_mV, current_A, temp_C, stress, status }`.

- eRPM handled here; mechanical RPM conversion lives in `rpm_filter`.
- Optional: KISS/BLHeli telemetry-UART fallback for boards that route the telem pad
  (see `.ai/architecture/interfaces.md` §3).

Status: stub. A0 reads EDT inline in `src/main.cpp`.
