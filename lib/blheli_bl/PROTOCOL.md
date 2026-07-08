# BLHeli-S 1-wire bootloader — protocol notes

Reference for `blheli_bl`. Being filled from **Betaflight 4-way** (`serial_4way_*`)
and **esc-configurator** (see `.ai/architecture/interfaces.md` §4/§5). Sections mirror
the research questions; `[TODO:proto]` marks values still to confirm.

> Research agent running (2026-07-08). Populate each section with the **exact** value
> and cite the source file/URL. Do not guess byte values.

## §A. Physical layer (1-wire)
- Baud rate: `[TODO:proto]`
- Framing (data/parity/stop), inverted?: `[TODO:proto]`
- Half-duplex single-wire (TX/RX shared), turnaround handling: `[TODO:proto]`
- Pico transport choice (SerialPIO single-pin vs bit-bang): `[TODO:proto]`

## §B. Entering / holding the bootloader
- Power-up window behaviour: `[TODO:proto]`
- BootInit / hello bytes: `[TODO:proto]`
- Keep-alive: `[TODO:proto]`

## §C. Command set (byte-level frames + CRC)
| Command | Frame format | Response | Notes |
|---------|--------------|----------|-------|
| BootInit / connect | `[TODO:proto]` | | |
| Keep-alive | `[TODO:proto]` | | |
| Device info / signature | `[TODO:proto]` | | |
| Set address | `[TODO:proto]` | | |
| Read flash | `[TODO:proto]` | | |
| Read EEPROM | `[TODO:proto]` | | |
| Erase page (A1) | `[TODO:proto]` | | |
| Write flash (A1) | `[TODO:proto]` | | |
| Write EEPROM (A1) | `[TODO:proto]` | | |
| Run / exit | `[TODO:proto]` | | |

- CRC algorithm + polynomial: `[TODO:proto]`

## §D. Device signatures
- EFM8BB21 (LittleBee Spring 30A) signature bytes: `[TODO:proto]`
- Signature → name table source: `[TODO:proto]`

## §E. EEPROM parameter layout
See `../esc_setup/EEPROM.md` (offset map lives there).

## §F. Porting gotchas (Pico)
- `[TODO:proto]`

## Sources
- Betaflight `serial_4way_avrootloader.c/.h`, `serial_4way.c` — `[url]`
- esc-configurator bootloader/flash + eeprom descriptors — `[url]`
- BLHeli_S source (EEPROM/`Eep_` defs) — `[url]`

## Confidence & unverified items
- `[TODO:proto]`
