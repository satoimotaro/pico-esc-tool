# Protocol notes — pico-esc-tool

References for the interfaces this firmware implements. Fuller design in the
workspace `.ai/architecture/interfaces.md` (local).

## DShot (control)
- DShot150/300/600, 11-bit throttle + telemetry bit + CRC. Generated via RP2040 PIO.

## Bidirectional DShot (telemetry)
- Same wire, half-duplex; ESC returns GCR-encoded eRPM. Mechanical RPM = eRPM / pole pairs.
- Requires firmware support (stock BLHeli-S may need bidir enabled; BlueJay supports it).

## KISS / BLHeli telemetry UART (optional)
- 115200 8N1 periodic frame: temp, voltage, current, mAh, eRPM, CRC8. Needs telem pad.

## BLHeli-S 1-wire bootloader (flashing)  [hard]
- Over the signal wire: enter bootloader → device ID (SiLabs EFM8 BB2x) → erase → write → verify.
- Reference: **esc-configurator** (JS) bootloader/flash modules; BLHeliSuite.

## EEPROM parameter block (setup)  [hard]
- Fixed BLHeli-S layout (arm, direction, timing, PWM freq, beacon, temp protection…).
- Reference: **esc-configurator** settings descriptors (versioned per firmware).

## Host link
- Phase 1: USB-C CDC serial. Commands: SCAN / ARM / DISARM / SET / TELEM / SETUP / FLASH.

## AS5600 encoder — `enc` and `encv` (de-aliased)
- `enc|raw|ang|deg|md|ml|mh|agc|mag` — one raw AS5600 sample (12-bit angle 0..4095 + magnet health).
  Sourced from the on-device tracker snapshot (single owner of I2C0), not a fresh blocking read.
- `encv|accum|rpm|samples|md` — **de-aliased velocity**, the honest speed source. The RP2040 samples
  RAW_ANGLE at ~1.25 kHz (`As5600Tracker`, `src/apps/as5600.h`), unwraps into a signed accumulator
  (`accum`, native ticks), and computes a 20 ms-window signed `rpm`. Poll it instead of unwrapping
  `enc` at the host tick rate.
- **Why it matters — the encoder reads ELECTRICAL angle.** On this bench the AS5600 faces the motor's
  14-magnet bell, so its angle advances `pole_pairs` (7) electrical cycles per *mechanical* rev.
  Sampled at the host's ~50 Hz that ALIASES far below the naive Nyquist guess — reverse (slightly
  faster) folded into garbage and produced fake "over-commutation" (apparent slip 3-21) that did not
  exist. Sampling on-device at ~1.25 kHz removes the aliasing at any real speed. Because `encv.rpm` is
  ELECTRICAL, mechanical RPM = `encv.rpm / pole_pairs`; at true BEMF lock `encv.rpm == tele eRPM`
  (slip ~1.0) in BOTH directions. The host `crossover._Ticker` prefers `encv` (÷pole_pairs to mech)
  and only falls back to host-side `enc` unwrap when the firmware/sim lacks `encv`.
