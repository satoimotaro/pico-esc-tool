# esc_dshot

Thin channel-management layer over **pico-bidir-dshot** (`BidirDShotX1` / `DShotX4`).

Planned responsibility:
- Own N ESC channels; map channel → PIO/SM (bidir: ≤8 ESCs; forward X4: more).
- Per-channel arm/disarm + RUN ↔ CONFIG/BOOTLOADER mode switch (shared signal wire
  with `esc_setup`/`esc_flash`).
- Uniform `setThrottle(ch, v)` / `readTelemetry(ch)` API.

A0 uses the library directly in `src/main.cpp` (single channel). Promote to this
module when adding multi-channel (Phase A1).
