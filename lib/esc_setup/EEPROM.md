# BLHeli-S EEPROM parameter layout

Offset → parameter map for the SiLabs BLHeli-S config block. Being filled from
**esc-configurator** settings descriptors and **BLHeli_S** `Eep_` definitions.
`[TODO:proto]` = pending research.

- EEPROM base address: `[TODO:proto]` (placeholder `0x1A00` in code)
- Block length: `[TODO:proto]`
- Layout/version byte offset + meaning: `[TODO:proto]`
- Layout name string (e.g. `#S_H_50#`) offset/length: `[TODO:proto]`

| Offset | Param | Encoding / range | Source |
|--------|-------|------------------|--------|
| `[TODO]` | motor direction (normal/reversed/bidir) | | |
| `[TODO]` | PPM min throttle | | |
| `[TODO]` | PPM max throttle | | |
| `[TODO]` | PPM center throttle | | |
| `[TODO]` | beep strength | | |
| `[TODO]` | beacon strength | | |
| `[TODO]` | beacon delay | | |
| `[TODO]` | motor timing | | |
| `[TODO]` | PWM frequency | | |
| `[TODO]` | demag compensation | | |
| `[TODO]` | temperature protection | | |
| `[TODO]` | low-voltage protection | | |
| `[TODO]` | brake on stop | | |
| `[TODO]` | startup power | | |

## Notes
- Keep the full raw block; do read-modify-write to avoid corrupting fields we don't decode.
- Layout revision gates the offset map — support the LittleBee Spring 30A's revision first.
