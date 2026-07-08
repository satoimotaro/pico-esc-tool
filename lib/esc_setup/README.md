# esc_setup  [core, hard]

BLHeli-S **EEPROM parameter block** read/write over the 1-wire signal path — configure
the ESC standalone (no flight controller): arming, motor direction, timing, PWM
frequency, beacon, temperature protection, etc.

Reference to port: **esc-configurator** settings descriptors (versioned per firmware),
and BLHeliSuite. See `.ai/architecture/interfaces.md` §5.

Plan (Phase A1):
1. Enter bootloader / config access (shared with `esc_flash`).
2. Read EEPROM block → decode against a versioned layout map (read-only first).
3. Modify + write-back + verify.

Status: stub. Early read-only spike scheduled in Phase A0.
