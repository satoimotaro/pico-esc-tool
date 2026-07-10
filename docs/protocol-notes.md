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
