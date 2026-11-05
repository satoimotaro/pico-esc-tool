# ESC lifecycle: identify, drive, hot-plug (design, not yet built)

Status: agreed design for the release after rev0. rev0 ships the current code with the
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
- The idle hold covers every wired ESC (Bluejay and BLHeli_S beacon too), not only AM32.
- A bidir arm completes only once the ESC answers with telemetry and has taken the EDT enable.
  Bluejay with "require EDT to arm" never started after a passthrough when the motor test sent
  throttle at once: EDT went out only in a zero-throttle gap after the arm.
- The end of a passthrough re-reads each ESC while it is still in its bootloader, so the motor test
  that follows arms without another bootloader visit.
- The 3D/one-way meaning of a motor-test value is fixed per session on both sides. The page takes
  each motor's meaning once when the test starts and keeps it until the test has stopped, even if an
  ESC is forgotten or re-read meanwhile; it declares it (`mtmode <mask>`, answered `mtmode|xx`) and
  repeats the declaration ahead of every frame. Without an answer it treats every motor as FEATURE_3D
  says. The firmware latches its FEATURE_3D answer when a client reads it, until USB DTR falls or it
  is read again with no test running. A declaration ends with the test, when DTR falls, and after
  2 s without MSP (a loop held up by a bootloader read does not count). An undeclared channel takes
  an ESC over only above the reported stop.
- Known limits, in the release notes:
  - A bootloader read on the drive path still stalls every stream.
  - Swaps are not detected. A bidir ESC that answered and then goes silent for 1 s while armed is
    reported (code 7); after 3 prompt polls in a row the page forgets its read and locks its slider.
  - A failed read leaves the ESC unknown (no stale direction is used), and it is not retried
    automatically.
  - Stall re-arm (Bluejay/BLHeli-derived bidir only, not AM32): eRPM 0 under throttle for 1.5 s, or
    no telemetry for 0.5 s once it has turned, re-arms the ESC (EDT included). A throttle below the
    start point therefore re-arms after 1.5 s. The target is then held at 0 until a 0 is sent, so a
    jam is never restarted on its own. After 3 restarts without a run in between, only a 0 held for
    1 s frees it.
  - A session ends on DTR fall (USB). Transports without DTR (the `main_uart` build, a WebSocket
    bridge) only end it on MSP silence: a client that reconnects within 2 s inherits the old test.
  - The firmware's `mtmode` declaration and the FEATURE_3D latch are RAM only; the page sends the
    declaration again on each frame.
  - MOTOR_TELEMETRY's mAh field carries the motor-test state (escs::UiState: 2 arming, 3 armed,
    4 idle, 5 absent, 6 stalled, 7 silent; 1 reserved) with 0x5A in the high byte. Betaflight
    Configurator shows it as about 23 Ah. The page reads it only from a board that says PICO.
  - PWM: a width sent during the 5 s arm is not kept. After the arm, only the stop width passes
    until one stop width has come (reported as arming, code 2, so the page keeps its slider locked).
    The 500 ms deadman also runs from a timer IRQ, so it holds while core0 is blocked. Every PWM
    pin goes to its stop width during any bootloader visit, and CLI bootloader commands are refused
    while a PWM pin is fed above its stop. A pin unfed for 3 s after its arm leaves PWM, and the idle
    hold takes it back. There is no hardware watchdog: a core0 hang with interrupts off keeps the
    last width.
  - `thrust` and the 3D motor-test range reach full scale at ±953. Past that the value is clamped,
    because the next 47 units would fall into the other direction's band.
