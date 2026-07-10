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
- **Entry (no power-cycle, source-confirmed via BLHeli_S.asm `init_no_signal`):** the
  RUNNING app jumps to the BL itself. Sequence: (1) feed a valid signal, then STOP it
  (signal loss); (2) hold the line **continuously HIGH, glitch-free, for ≥~60ms** — the
  app times out (RC-timeout counter #10 polled ~10ms ⇒ up to ~100ms), reaches
  `init_no_signal`, verifies the line is high for **15ms with zero intervening lows**, then
  `ljmp 1C00h` into the BL; (3) ONLY THEN stream BootInit. Sending BootInit's LOW start
  bits during the 15ms check aborts the jump (`jnb RTX_PORT.RTX_PIN,bootloader_done`) →
  the app keeps running/beeping. We use HIGH_HOLD_MS=200 for margin. Betaflight's
  `esc4wayInit` does exactly this (motorDisable + input-pullup + setEscHi), then waits an
  MSP round-trip (100ms+) before `BL_ConnectEx`. Once in the BL it waits ~250ms×250 for
  the first start bit, so overshooting the hold is harmless. Bidir (inverted) DShot idles
  HIGH, so priming with it is *consistent* with the required hold-high. Fallback if the
  running-app path ever fails: power-cycle the ESC while streaming BootInit (BL also
  listens briefly at reset) — see `spike_boot`.

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

## §G. Bench-confirmed on hardware (2026-07-10, EFM8BB21 sig E8 B2)

Full pipeline proven: connect → read → write → firmware flash. Corrections vs earlier notes:

- **Read the SiLabs config with FLASH-read `0x03`, not EEPROM-read `0x04`.** EFM8 has no EEPROM;
  BLHeli-S/BlueJay config lives in FLASH at 0x1A00. `0x04` returns 0 bytes and wedges the parser.
  `readEeprom`→ use `readFlash(0x1A00,…)`.
- **TX→RX turnaround gap is REQUIRED.** After the ESC transmits a reply it needs time to switch
  back to RX; a command sent immediately after an ACK is missed entirely (ESC 100% silent,
  raw `edges=0`). Fix: `sendCmd` does `if (connected_) delay(5ms)` before every post-connect
  command. This was the true cause of the long-standing "read failed" (the READ cmd right after
  the SET_ADDRESS ack was never heard). Fast OE-register turnaround was *too* fast for the ESC.
- **Read response = `[data][CRC-lo][CRC-hi][ACK]`** — there IS a trailing ACK byte (0x30) after
  the CRC (BF `BL_ReadBuf`: "with CRC read 3 more"). `readBuf` must read it.
- **SET_BUFFER header gets NO ack** — the device waits for the buffer bytes (BF asserts brNONE);
  the SUCCESS ack comes only after the data. Waiting for an ack after the header hangs writes.
- **Frames confirmed:** SET_ADDRESS `{FF,0,hi,lo}`+CRC→0x30; keepAlive `{FD,0}`+CRC→C1;
  read `{cmd,len}`+CRC; SET_BUFFER `{FE,0,hi,lo}` (len 256→`{FE,0,1,0}`). Latencies ~28µs.
- **Flash write = app-only.** A stock BLHeli-S HEX contains app (0x0000-0x19FF) + eeprom identity
  (~0x1A40 tags) + bootloader (0x1C00+). Over the 1-wire BL you MUST NOT reflash the BL you speak
  through: flash 0x0000-0x19FF, capture the eeprom identity for the compat check, SKIP everything
  ≥0x1A00. Page erase = 512 B (EFM8BB21); writeFlash ≤256 B/call.
- **Compatibility guard** (`esc_flash::checkCompatibility`): the ESC's bootloader signature +
  its config LAYOUT tag (0x1A40) must match the HEX's MCU tag (0x1A50) + LAYOUT tag. Demonstrated:
  wrong layout → BLOCK, matching → allow. Prevents flashing a wrong-FET-map image.
- **Note:** flashing app-only leaves the EEPROM (0x1A00) untouched, so config `name`/version keep
  their pre-flash values; identify the RUNNING firmware by reading APP flash (0x0000), not EEPROM.
