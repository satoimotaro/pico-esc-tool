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
  (`accum`, ticks; the AS5600 is a **2-pole shaft magnet** = 4096 ticks / mechanical rev, hand-turn
  confirmed), and computes a 20 ms-window signed **mechanical** `rpm`. Poll it instead of unwrapping
  `enc` at the host tick rate.
- **Why it matters — high speed aliases the slow host sampler.** The rotor really spins fast in
  6-step (~6000-9000 mech for this 930 KV motor), far past the host's ~50 Hz Nyquist (~1350 mech).
  The old host-side unwrap of `enc` aliased there — worse in reverse (slightly faster) — producing a
  fake "over-commutation" (apparent slip 3-21) that did not exist. Sampling on-device at ~1.25 kHz
  removes the aliasing at any real speed.
- **Both speed sources are MECHANICAL RPM.** `tele.rpm` is already mechanical — the firmware divides
  the DShot eRPM by the motor pole pairs (`ESC_MOTOR_POLES/2`, `esc_session.h`) before sending it.
  The 2-pole encoder is mechanical too. So lock quality is `slip = |tele.rpm| / |encv.rpm| ~= 1.0`
  at true BEMF lock in BOTH directions — do NOT divide either by pole pairs again (that latent
  double-division made a real lock read as ~0.143 = 1/7). The host `crossover._Ticker` prefers `encv`
  and falls back to host-side `enc` unwrap only when the firmware/sim lacks `encv`.
