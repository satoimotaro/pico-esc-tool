# ESC lifecycle: identify, drive, hot-plug (design, not yet built)

Status: agreed design for the release after rev0. rev0 ships the current code with the two
fixes listed at the end; nothing below is implemented yet.

## Requirements

- R1 As a configurator, never ask the user for parameters. What an ESC is (firmware, 3D or not,
  bidir or not) comes from reading it through its bootloader, for any supported firmware.
- R2 Plugging or unplugging an ESC while others spin never stops or confuses the others. A newly
  plugged bidir ESC is read on its own and becomes usable.
- R3 Never mix up 3D and non-3D. No non-zero throttle goes to an ESC whose direction mode is not
  known for the ESC that is on the pin now.
- R4 The same core serves the ROV, where identity can come from telemetry (a firmware that reports
  an id) or from declared config instead of the bootloader.
- R5 Obvious to drive: start the motor test, it spins.

## What is wrong today

- `escs::spinArm` reads the ESC through its bootloader whenever its cached info is unknown, and
  core0 waits for core1 the whole time (`runOp`). Every DShot stream on the board stops for that
  long: ~1.3–3.4 s for a present ESC, ~11.5 s for an absent one.
- The cache (`infoKnown/infoBidir/infoRev`) is RAM only, is not tied to the ESC that is on the pin
  now, and a failed read still arms in normal DShot with the stale direction.
- The rev0 idle hold re-arms an ESC 5 s after it is stopped; for an ESC whose read failed that is a
  fresh bootloader visit, so one dead ESC could stall everything every 16 s.

## Design

### One state per pin

| state | meaning | may send |
|---|---|---|
| EMPTY | nothing seen | zero throttle (bidir), to detect a plug-in |
| PENDING | something appeared, not read yet | zero throttle |
| READING | core1 is in its bootloader | nothing (core1 owns the pin) |
| READY | identity known | zero throttle |
| RUNNING | being driven | any throttle, mapped by identity |
| LOST | its telemetry stopped | zero throttle; identity dropped |
| FAILED | the read failed | nothing; no automatic retry |

- Only READY and RUNNING accept a non-zero command (R3). Commands that arrive in any other state are
  refused with the state's name, never queued and never waited on.
- The pin has exactly one owner at a time: core0 while DShot or PWM runs, core1 while in the
  bootloader. Ownership changes only on a state change.
- LOST drops the identity and goes to PENDING when telemetry returns. A swap always passes through
  LOST, so a new ESC is never driven with the old one's identity (R2, R3).
- FAILED is left only by a user action (`scan`, a read from the configurator) or, for bidir, by
  telemetry appearing.

### Asynchronous reads

- core0 never blocks on the bootloader. It queues a read, keeps streaming to every other pin, and
  picks up the result on a later loop.
- core1 runs one bootloader job at a time from the queue. The BLHeli_S prime (zero-throttle burst,
  then line high) becomes timed steps instead of `delay()`.
- LittleFS writes (config, present mask) run only while core1 is idle. A flash write pauses the
  other core, which would cut a bit-banged bootloader byte in half.

### Hot-plug

- Empty and READY pins get bidir zero throttle. Bluejay and AM32 answer as soon as they are powered,
  which moves the pin to PENDING and queues a read. Zero throttle is safe for an unread ESC.
- Telemetry absent for 300–500 ms (not a few bad frames) is LOST.
- Non-bidir ESCs (stock BLHeli_S) cannot be seen arriving or leaving. Decision: do not re-read them
  on every start. The UI marks them "swap not detected: read again after changing it" and they are
  read by `scan` or by the configurator's read. Bidir ESCs are handled automatically.

### Identity sources and policies

| | identity from | telemetry LOST while running |
|---|---|---|
| configurator (tool) | bootloader read (R1) | stop that ESC only, report |
| ROV | telemetry id or declared config; `scan` at launch | keep streaming, drop to open loop, report |

Only these two columns differ; the state machine and the async reads are shared (R4). On the ROV a
mismatch between declared and read identity keeps that thruster in FAILED.

### Commands are semantic

Callers ask for signed thrust (or rpm); the pin's identity maps it to DShot or PWM values. A reverse
request to a non-3D ESC is refused and reported, not clamped to zero silently.

### What disappears

- The persisted AM32 mask (`armMask`): the idle hold covers every READY pin, and the passthrough
  always waits the AM32 time (2–3 s more per session).
- Re-validation flags after config writes, flashes and passthrough: those all end in READING.
- Writes never use cached identity: the bootloader session that writes reads signature and layout
  first and compares (already so in the web flasher's `flashCheck`; to be checked for the CLI and
  for the cached `fwHave` reply).

## rev0 (the current code, shipped first)

- Idle hold: start holding only when nothing else is being driven, and never retry an ESC whose
  read failed until `scan` or a configurator read.
- PWM output (50 Hz servo pulses) for firmwares that take it (AM32, stock BLHeli_S; not Bluejay,
  which accepts DShot only): its own deadman, the idle hold leaves PWM pins alone, and switching
  back to DShot needs no reboot.
- Known limits, in the release notes: a bootloader read on the drive path still stalls every
  stream; swaps are not detected; a stale direction after a failed read is possible.
