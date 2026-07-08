# BLHeli-S EEPROM parameter layout (layout rev 33)

Offset → parameter map for the SiLabs BLHeli-S config block. Verified in **both**
esc-configurator `Blheli/eeprom.js` + `BlheliS/settings.js` **and** BLHeli_S.asm
`CSEG AT 1A00h` (`Eep_*` labels). Implemented in `esc_setup.cpp`.

- **Base address:** `0x1A00` (EFM8BB1/BB2). **Block length:** `0x70` = 112 bytes.
- Read byte `0x02` (layout revision) first and branch: **33** (fw 16.3–16.5) vs
  **32** (16.0–16.2, TEMP_PROTECT is a plain bool there).
- Offsets below are relative to 0x1A00. "unused" = FF placeholder kept for BLHeli
  (non-S) layout compatibility.

| Off | Param | Encoding |
|-----|-------|----------|
| 0x00 | main revision | e.g. 16 |
| 0x01 | sub revision | e.g. 7 |
| 0x02 | **layout revision** | 33 (0x21) current; 32 older |
| 0x09 | startup power | enum 1–13 → 0.031…1.50 (default 9 = 0.50) |
| 0x0B | **motor direction** | 1=Normal 2=Reversed 3=Bidir(3D) 4=Bidir-rev |
| 0x0D–0x0E | mode init signature | L,H → 0x55AA=multi 0xA55A=main 0x5AA5=tail |
| 0x0F | programming-by-TX | 1=on 0=off |
| 0x15 | **commutation timing** | 1=Low 2=MedLow 3=Med 4=MedHigh 5=High (def 3) |
| 0x19 | PPM min throttle | µs = 1000 + 4×byte (def 37 → 1148 µs) |
| 0x1A | PPM max throttle | µs = 1000 + 4×byte (def 208 → 1832 µs) |
| 0x1B | beep strength | 1–255 (def 40) |
| 0x1C | beacon strength | 1–255 (def 80) |
| 0x1D | beacon delay | 1=1m 2=2m 3=5m 4=10m 5=∞ (def 4) |
| 0x1F | demag compensation | 1=Off 2=Low 3=High (def 2) |
| 0x21 | PPM center throttle | µs = 1000 + 4×byte (bidir only; def 122 → 1488 µs) |
| 0x23 | temperature protection | rev33: 0=Off,1=80°C…7=140°C (def 7); rev32: bool |
| 0x24 | low-RPM power protection | bool (def 1) |
| 0x27 | brake on stop | bool (def 0) |
| 0x28 | LED control | bitmap, 2 bits/LED |
| 0x40 | layout tag (16B ASCII) | e.g. `#FVTLibee30A#` — ESC-JS-sourced position |
| 0x50 | MCU tag (16B ASCII) | ESC-JS-sourced position |
| 0x60 | name (16B ASCII) | `Eep_Name` (asm `CSEG AT 1A60h`) |

## Write policy (Phase A1)
Read the full 112-byte block, modify only decoded fields, write back (read-modify-write)
to preserve bytes we don't decode. Verify by re-reading. Gate the offset map on layout
revision (support the LittleBee's revision first).

## Caveats
- 0x40/0x50 tag positions come from ESC-JS only; the asm defines `Eep_*` through 0x29
  and `Eep_Name` at 0x60. Treat tag offsets as configurator-sourced.
