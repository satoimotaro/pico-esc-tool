# BLHeli-S (SiLabs EFM8BB) 1-wire bootloader — protocol reference

Authoritative values for `blheli_bl`, read line-by-line from the reference sources
(not summarized). Implemented in `blheli_bl.cpp`.

**Sources**
- **BF-AVR** — Betaflight `src/main/io/serial_4way_avrootloader.c/.h` — the canonical C
  implementation of the SiLabs "BLB" bootloader (author 4712, from H. Reddmann's
  AVRootloader). **Byte-level authority for direct Pico↔ESC.**
- **BF-4WAY** — Betaflight `serial_4way.c/.h` — device-match + pin config.
- **ESC-JS** — esc-configurator `Hardware/Silabs.js`, `Blheli/eeprom.js`,
  `BlheliS/settings.js` — signatures + EEPROM layout (it drives an FC via 4-way MSP;
  it does **not** bit-bang the wire, so it is not the transport authority).
- **BLHELI-S** — `bitdump/BLHeli` → `BLHeli_S SiLabs/BLHeli_S.asm` (Rev 16.7, layout 33).

## §A. Physical layer (1-wire)
- **19200 baud, 8N1, non-inverted** (idle HIGH, start bit LOW). Bit time = **52 µs**
  (`#define BIT_TIME 52`). Sample offset into start bit = 3/4 bit = **39 µs**.
- **Half-duplex, single wire** — TX and RX share the signal pin. RX/idle = **input +
  pull-up** (`IOCFG_IPU`); TX = **push-pull output** (`IOCFG_OUT_PP`).
- Pico transport: bit-banged in `blheli_bl.cpp` (`txByte`/`rxByte`), mirroring BF-AVR
  `suart_putc_`/`suart_getc_`; interrupts disabled per byte for timing.

## §B. Entering / holding the bootloader
- **BootInit (hello), 17 bytes, sent WITHOUT CRC:**
  `00 00 00 00 00 00 00 00  0D  42 4C 48 65 6C 69 ("BLHeli")  F4 7D`
- **Reply, 8 bytes, no CRC/ACK:** `"471"` + BootMsgLast + **sigHi** + **sigLo** +
  BootVersion + BootPages. Valid iff first 3 bytes == `34 37 31` ("471") and signature≠0.
- **Keep-alive:** send `FD 00` (+CRC); **alive iff reply ACK == 0xC1** (`brERRORCOMMAND`)
  — 0xFD is an invalid command, so the NAK proves the BL is running. (Not a failure!)
- Entry: a powered idle BLHeli-S jumps to its BL when it sees BootInit; if unresponsive,
  power-cycle the ESC while streaming BootInit (BL listens only briefly at boot).

## §C. Command set, framing & CRC
Command bytes: `RUN 0x00 · PROG_FLASH 0x01 · ERASE_FLASH 0x02 · READ_FLASH_SIL 0x03 ·
READ_EEPROM 0x04 · PROG_EEPROM 0x05 · KEEP_ALIVE 0xFD · SET_BUFFER 0xFE · SET_ADDRESS 0xFF`.
ACK codes: `brSUCCESS 0x30 · brERRORVERIFY 0xC0 · brERRORCOMMAND 0xC1 · brERRORCRC 0xC2 · brNONE 0xFF`.

**Stateful CRC (critical):** BootInit and its reply carry **no CRC**. After a successful
connect (signature≠0), **every** sent frame appends `CRC_lo CRC_hi`, and every data read
returns `data… CRC_lo CRC_hi ACK`. `blheli_bl` gates this on `connected_`.

**CRC-16:** poly **0xA001** (reflected CRC-16/IBM-ARC), init 0x0000, LSB-first,
transmitted **little-endian** (lo then hi). See `crcAdd()`.

**Addresses** are 16-bit **big-endian** (hi then lo); `0xFFFF` = keep current.
**Length byte** `0x00` = 256; max 256 bytes/transfer.

| Operation | Payload (before CRC) | Reply |
|---|---|---|
| Connect / BootInit | `00×8 0D "BLHeli" F4 7D` (no CRC) | 8B `"471"+…+sig+ver+pages` (no CRC/ACK) |
| Keep-alive | `FD 00` | ACK==0xC1 = alive |
| Set address | `FF 00 ADDR_H ADDR_L` | ACK==0x30 |
| Set buffer | `FE 00 lenHi lenLo` then `data…` | ACK…0x30 |
| Read flash | setAddr, then `03 N` | `N data + CRC + ACK` |
| Read EEPROM | setAddr, then `04 N` | `N data + CRC + ACK` |
| Erase page | setAddr, then `02 01` | ACK==0x30 (slow) |
| Write flash | setAddr, setBuffer, then `01 01` | ACK==0x30 |
| Write EEPROM | setAddr, setBuffer, then `05 01` | ACK==0x30 |
| Run / exit | `00 00` | — (disconnects) |

**Read-EEPROM (dump config):** `FF 00 1A 00` → ACK; `04 70` → 112 data + CRC + ACK.

## §D. Device signatures (ESC-JS `Silabs.js`)
| Sig word | MCU | page | flash | boot addr | eeprom |
|---|---|---|---|---|---|
| `0xE8B1` | EFM8BB10x | 512 | 8192 | 0x1C00 | 0x1A00 |
| **`0xE8B2`** | **EFM8BB21x** (LittleBee Spring 30A) | 512 | 8192 | 0x1C00 | **0x1A00** |
| `0xE8B5` | EFM8BB51x | 2048 | 63485 | 0xF000 | 0x3000 |

SiLabs BLB match (BF-4WAY): `0xE800 < word < 0xF900`. → LittleBee returns **`E8 B2`**.

## §E. EEPROM parameter layout
See `../esc_setup/EEPROM.md` for the full offset→field map (implemented in `esc_setup`).

## §F. Porting gotchas (implemented / to verify on bench)
1. **Stateful CRC** — none on connect, CRC after. (done: `connected_` gate.)
2. **Line turnaround** — flip output→input before the ESC replies. (done: `setRx()` after send; may need a µs guard — verify.)
3. **Pull-up to the ESC MCU rail (EFM8 = 3.3 V typ). RP2040 is 3.3 V, NOT 5 V tolerant** —
   match the ESC logic rail, not the battery. Add ~10 kΩ pull-up. **Verify LittleBee rail.**
4. 19200 non-inverted; PIO/UART alt clock ≈ 1e6/52.
5. Bootloader entry: stream BootInit / power-cycle; never let a motor spin during this.
6. **DShot vs bootloader share the pin** — fully idle DShot (tri-state, pull-up holds high)
   before the handshake; don't resume motor signal until after `run()`/exit.
7. Length byte 0 = 256.
8. Big-endian address, little-endian CRC — easy to swap.
9. Keep-alive "alive" = 0xC1 NAK, not success.
10. EFM8BB51 differs (eeprom 0x3000, page 2048) — BB1/BB2 use 0x1A00/512.

## Confidence & unverified
- **High** (verbatim from source): all command bytes, ACK codes, CRC poly 0xA001,
  BootInit bytes, 19200/52 µs, 8N1 non-inverted, half-duplex pin modes, frame layouts,
  signatures, EEPROM base/size/offsets, defaults/enums.
- **Unverified:** exact BL power-up listen-window (behavioral, not a constant);
  0x40/0x50 tag positions are ESC-JS-sourced (asm defines up to 0x29 + Name@0x60);
  BootVersion/Pages reply bytes informational; LittleBee logic-rail voltage — confirm
  on hardware before wiring the pull-up. **All transport timing untested on hardware.**
