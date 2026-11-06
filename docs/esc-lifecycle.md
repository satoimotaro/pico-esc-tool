# ESC lifecycle: identify, drive, hot-plug (design; phase 1 built)

Status: phase 1 is implemented (`lib/esc_lifecycle`, `src/apps/esc_session.h`); phase 2 is still
design. See "Phase 1 and phase 2" below for the split and for where the code refines this design.
Hot-plug timing and the LOST threshold have not had a hardware bench pass yet. rev1 carries rev0's
drive rules into it ("rev1" below).

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
- FAILED is left only by a user action (`scan`, a read from the configurator). It sends nothing, so
  it cannot see telemetry appear either: a read that fails without any telemetry seen on the pin goes
  to EMPTY instead (refinement 4 below).

### Asynchronous reads

- core0 never blocks on the bootloader. It queues a read, keeps streaming to every other pin, and
  picks up the result on a later loop.
- core1 runs one bootloader job at a time from the queue. The BLHeli_S prime (zero-throttle burst,
  then line high) becomes timed steps instead of `delay()`.
- LittleFS writes (config, the pin list) run only while core1 is idle. A flash write pauses the
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

## Phase 1 and phase 2

Landed in phase 1:

- `lib/esc_lifecycle/esc_lifecycle.h`: the state machine (`lc::Pin`), the identity rules, the 3D map
  and the semantic drive mapping, with a native g++ test.
- `escs::` keeps one `lc::Pin` per pin. `spinPoll()` feeds it telemetry and time, runs the idle hold
  (bidir zero on EMPTY/PENDING/LOST, zero in the identity's protocol on READY) and so detects a bidir
  ESC being plugged in or pulled out. `idle off` stops the hold, and with it the hot-plug detection.
- Reads on the drive path (`spinArm`, hot-plug, the motor test) run on core1 as one asynchronous job
  (`Op::READID`: prime, line high, the 1 s / 4 s / 4 s ladder, learn, run). core0 keeps streaming and
  collects the result on a later loop.
- The explicit config ops and the 4-way passthrough stay synchronous, but every core0 wait pumps the
  other pins' frames, telemetry and deadman.
- LOST handling with the tool/ROV switch (`escs::setPolicy`), `escs::drive()` with refusals
  (`err not-armed`, `err not-armed (state X)`, `err reverse-on-one-way (ESC i is not 3D)`), and a
  `state` CLI command.
- The AM32 mask is gone, and the passthrough always waits the AM32 time. (The present mask,
  `/cfg/present`, went later too: see "Signal pins and presence" below.)
- LittleFS writes run under `escs::FsGuard` (core1 parked in its idle branch); a refusal answers
  `err busy (ESC read in progress, retry)`.

- One bootloader line: a read cannot run while `scan`, `read` or `enter` holds an ESC in its
  bootloader. A pin waiting to be read wins once the session is idle: no op on it for 5 s, and never
  a session that has been written (an `erase`/`writeflash` sequence must not see its ESC run
  mid-image; `run` ends it). The held session is then learned and run, and the next config op on
  that ESC enters it again. `arm` of a pin that must be read first runs a foreign held session at
  once (as 4baed53's `arm` did), unless it was written: then it answers
  `err busy (ESC <s> is being written; run it first)`. `arm` on a pin not read yet answers
  `arming (reading first, then ~3s) (state X)`; hosts poll `state` for RUNNING
  (`host/pico_esc/drive.py` `wait_read`).

Phase 2:

- The explicit config commands (`scan`, `read`, `enter`, `editpage`, `erase`, `writeflash`,
  `readflash`, `preset`, web flash, `blprobe`) go asynchronous with deferred replies.
- ROV identity from a telemetry id or declared config (a mismatch keeps the pin FAILED), and `scan`
  at ROV launch.
- The RPM controller falling back to open loop on LOST in ROV mode (phase 1 keeps streaming and
  reports `# esc i: lost, open loop`).
- Signed rpm as a semantic command, and `spinThrust` on a non-3D ESC in RPM mode (phase 1 sends zero
  for a negative command there).
- The legacy `esc_tool.cpp` negative-throttle path.

Where the code refines the design:

1. READY accepts `arm`, not a bare non-zero command: non-zero needs RUNNING and an armed stream.
   After `disarm` or a firmware abort, host scripts rely on `err not-armed`, and rev0's idle hold
   re-enabled throttle by itself. A READY pin whose zero stream is already armed in the asked
   protocol goes RUNNING with no new arm window, so `arm` and the motor test stay fast.
2. A read that fails without fresh telemetry at its start goes to EMPTY, not FAILED (for example
   `arm` on a bare pin, or `scan` after an ESC was pulled). FAILED sends nothing, which avoids a read-fail, reboot, telemetry, re-read loop.
3. Hot-plug debounce: EMPTY or LOST becomes PENDING after 3 valid packets within 250 ms, each counted
   at least 15 ms after the previous one (so they span 30 ms or more). LOST is no
   valid packet for 400 ms once the pin has answered on its current stream (the reboot gap after a
   read is expected). A bidir READY pin that never answers within 8 s stays READY and is reported
   once (`# esc i: no telemetry within 8 s ...`); a pull is then not seen on it. Its identity is then
   unconfirmed: if it ever answers, the pin goes to PENDING and is read again before it is driven
   (an ESC swapped before its first answer is never driven with the old identity). RUNNING keeps the
   "armed after 8 s anyway" rule for an ESC that never answers. The driver keeps only the newest
   telemetry word per read, and core0 polls every ~200 us: the 15 ms spacing is what makes 3 samples
   real evidence. A failed read is FAILED only if telemetry was fresh (under 400 ms old) when the read
   started; a read after a session, a passthrough or a LOST finds stale telemetry and goes to EMPTY.
4. Writes never trust a cached reply. A 4-way re-select of a non-AM32 ESC sends a fresh BootInit and
   adopts a different reply as a new ESC; AM32 answers only the first BootInit, so it still re-adopts
   the cached reply if a keepalive answers (a swap there is caught by the re-read after the
   passthrough). Two failed keepalives in a row mark the CLI session lost, and the next op enters
   again with a fresh CONNECT. A write makes closing the session read the identity again before RUN.

## rev1: rev0's drive rules on the lifecycle

rev1 is phase 1 rebased onto rev0. Every rev0 rule below still holds; this is where each one
lives now and how it meets the lifecycle states.

- Motor-test state (MSP_MOTOR_TELEMETRY mAh field, 0x5A marker) is one enum, `lc::Ui`
  (`escs::UiState`), mapped from the pin's state by `lc::uiCode`:
  - 1: PENDING or READING.
  - 2: an arm waiting for its read, a DShot arm window, the hold's stream still arming, or a PWM
    arm or PWM waiting for its first stop width.
  - 3: armed (READY on an armed hold stream, or RUNNING).
  - 4: READY with no stream.
  - 5: EMPTY, LOST or FAILED.
  - 6: RUNNING and stalled: re-arming, or held at 0.
  - 7: an answered bidir stream at zero, not held, that has been silent for 200 ms
    (`lc::SILENT_MS`), or the ROV open loop.

  The page locks 1, 2, 5 and 6. Under the tool policy a pulled ESC goes LOST 400 ms after its
  last packet, so 7 shows for one poll at most. The page's gone rule (3 prompt polls in a row,
  read Bluejay/BlueGill/AM32 only) therefore also counts 5 once that motor has reported a live
  code since the last test start or stop, read or passthrough exit (a firmware-side FAILED learn
  right after a read does not erase the page's read).
- Stall watch (`lc::Stall`, run in `escs::detail::streamPin`): the same thresholds and the same
  hold until a zero command, STALL_MAX, and no stall judgement for AM32. It watches an armed
  RUNNING bidir stream under throttle that has answered (`lc::Pin::stallWatched`); one that never
  answered is not judged. Under throttle its silence belongs to the stall watch, not to LOST
  (`lc::Pin::stallWatch`), because a failed start beeps without telemetry. Its verdicts:
  - ZERO: eRPM read 0 for 1.5 s. The clock starts only on an eRPM reading of 0, so forced sine (EDT
    flowing, no eRPM) is never a stall, as long as a valid EDT packet comes at least every 500 ms
    (STALL_SILENT_MS); sparser EDT is judged as silence.
  - QUIET: not turned yet and no valid packet of any kind for 0.5 s (the failed start).
  - SILENT: the same total silence after it has turned. Under the tool policy that is LOST with the
    identity dropped (`lc::Pin::onStall`), so an ESC swapped in meanwhile is read before it is
    driven. Under the ROV policy it is the open loop (keep streaming, `# esc i: lost, open loop`),
    as LOST while running is there.

  ZERO and QUIET re-arm the same stream (zero, answer, EDT; `lc::Pin::onRearm`) with the identity
  kept and the target held at 0, under either policy. A re-arm that gets no answer within 8 s is
  LOST. At zero the 400 ms LOST rule applies as before.
- The loop-gap guard is `gapCheck`: a pin not polled for 50 ms moves its lifecycle clocks on by
  the gap, and its stall clocks restart. Five gaps in a row are reported once
  (`# esc i: core0 loop slow ...`), because a loop slow on every pass keeps LOST, SILENT and the
  stall silence rule from firing. SILENT is not reported for a pin not polled in the last 50 ms.
  A loop slower than 50 ms on every pass therefore disables LOST detection (and SILENT and the
  stall silence rule) altogether; SLOW_LOOP is the only report of it. `lc::stamp` replaces
  `millis()|1`.
- PWM: `lc::pwmGate` drops widths sent during the arm and needs one stop width first. The IRQ
  deadman stays. Every sync bootloader visit (`runOp`, `enterBootloader`) sends the stop width
  while it waits, because pump() keeps PWM going. `pwmBusy` refuses CLI bootloader commands. A pin
  unfed for 3 s after its arm goes back to READY, and the hold takes it. `pwmBegin` refuses when no
  alarm slot is free for the IRQ deadman.
- Unchanged from rev0:
  - the FEATURE_3D latch, the `mtmode` declaration and its acknowledgement, and the session end on
    DTR fall or MSP silence (fourway.h, `motorTestPoll`);
  - SET_MOTOR 0 as stop;
  - the undeclared 3D rule;
  - the page's mode snapshot, 全体, stop streaming, the stopping flag, the readAll pipe guard,
    forget on disconnect, and the lossless TUNE/MELODY fields.
- Known limits added by rev1:
  - A swap under throttle that keeps telemetry flowing in eRPM 0 readings (ZERO) or that never
    turned (QUIET) is re-armed with the old identity. An ESC plugged in within the following 8 s
    answers the re-arm as the old one, though held at 0 until a zero command. If the old identity
    was 3D and the new ESC is one-way, a forward command is then about 50 % throttle there, and
    "reverse" runs forward too. A pull after the motor has turned (SILENT) drops the identity
    (tool policy).
  - A jam while the motor is running may show up as total silence (SILENT) rather than as eRPM 0
    readings (ZERO), depending on what Bluejay sends as it drops out of its run. Under the tool
    policy that ESC then goes LOST and is read again through its bootloader instead of being
    re-armed. This needs a bench check.
  - AM32 is not stall-watched, so under throttle it is under the plain 400 ms LOST rule: a
    telemetry decode dropout longer than that stops it in the web tool (tool policy) and drops its
    identity; it is read again through its bootloader when it answers.
  - A bidir ESC whose telemetry never decodes stays READY (armed after 8 s anyway) and is not
    stall-watched; a pull or swap on it is not seen, like stock BLHeli_S.
  - The rpm loop (`Thruster::setRpm`) is for reversible (3D) ESCs only: its command is 3D thrust,
    and on a one-way ESC a positive command lands in the 3D forward band (about half throttle at
    command 1). Mapping it by identity is phase 2.
  - Bluejay's beacon (minutes at zero throttle) and its stall beeps send no telemetry. If one lasts
    longer than 400 ms at zero, the pin goes LOST and is read again when it answers. This needs a
    bench check.

## Signal pins and presence

Which GPIOs carry ESCs, and which of them have an ESC on them now, are two separate things, and
neither needs a rebuild or a `scan` any more.

### The pin list

- `ESC_SIGNAL_PINS` (`esc_config.h`) is the default. `pins 16 17 18 19` stores a list in
  `/cfg/pins`, `pins default` removes it, and `pins` shows it (`pins|now|<cfg|default>|...`,
  `pins|next|...`, `pins|default|...`, then `ok`). The list is read once at boot
  (`EscTool::begin`, before the hold or the encoder starts), so a change takes effect after a reboot;
  the answer says so (`# pins: saved; reboot to apply`). The web page has the same in its settings
  tab, on this firmware only.
- A list is 1..8 GPIOs in ESC-index order, each GP0..GP29, none twice, none used by this build:
  the Pico W radio (GP23/24/25/29), the LCD bus on the panel (GP5..15), the UART console on
  `main_uart` (GP0/1), the mode pin (GP22). Anything else answers `err bad-pins (<reason>)`, e.g.
  `err bad-pins (GP25: Pico W radio)`. The rule is `lib/esc_lifecycle/esc_pins.h`, with a native
  test; the default list is checked against it at compile time. A stored list that this build
  refuses (written by another env) is ignored at boot with a `# pins:` line, and the default is used.
- An ESC on GP16 or GP17 switches the AS5600 encoder off (its I2C0 is on those pins), as before.
- Every per-ESC array is sized `escs::MAX` (8); `escs::COUNT` is this boot's count. `main.cpp`
  declares eight `Thruster`s; `EscTool` uses the first `COUNT`. `cfg load` restores every stored one,
  `cfg save` stores the first `COUNT`.
- The legacy `esc_tool` env keeps the compile-time list.

### One index per pin

- ESC k is `PINS[k]` everywhere: the CLI index, MSP motor k (MOTOR, MOTOR_CONFIG, MOTOR_TELEMETRY,
  SET_MOTOR, `mtmode` bit k, `pwmch k`), the passthrough channel k and the web page's ESC k. The MSP
  motor count is the number of configured pins, so a configurator that reads it once at connect
  (the AM32 configurator hides its Read button at 0) always sees them. No numbering depends on which
  ESCs answer, so a plug-in or a pull, mid-test or not, never moves a channel onto another ESC.
- A pin with no ESC reports state 5 (absent) in MOTOR_TELEMETRY; the page locks its slider.
- Writes check the ESC again: the page re-reads the channel (bootloader reply and settings) before a
  settings write and refuses one that differs from what it read (`読み込んだときと違います`); a
  firmware flash already reads it fresh.

### Presence

- Nothing is stored. A pin is wired when its lifecycle state is not EMPTY: a bidir ESC is found by
  the hold's telemetry, any ESC by a read (`scan`, a configurator read, an arm). `/cfg/present` is
  deleted at boot. `escs` reports (`escs|<n>/<count>|<indices>`); its old arguments are ignored with
  a note.
- The passthrough probes per channel. MSP_SET_PASSTHROUGH answers at once with every configured pin
  (esc-configurator times out an MSP reply after 1 s, long before the 4.5 s settle). A channel's
  first InitFlash waits out the shared settle once, then sends its own pin one BootInit (500 ms on a
  bare pin). A pin that does not answer gets the full entry ladder (prime, 1 s, then 2 x 4 s) only
  if its lifecycle knew an ESC there when the passthrough began (`lc::Pin::ptRetry`: that ESC may be
  stuck in its app) or it answered earlier in the same passthrough. An EMPTY pin is absent: its
  InitFlash fails after the probe, and every retry of it in that passthrough fails at once. A ladder
  is charged to its own channel's InitFlash only. A stock BLHeli_S that was never read is EMPTY,
  but it is in its bootloader after the settle and answers the probe.
- So a missing ESC costs 0.5 s in a passthrough instead of ~11 s. With 4 pins and ESCs on the first
  two, reading all four channels costs the settle, two answers, and 1 s for the two empty pins
  (6.7 s on the bench).

### Known limits

- The idle hold streams to every configured pin, so an empty pin still holds a PIO SM. Bidir fits
  only pio0's 4 SMs on a Pico W (the radio is on pio1), normal DShot goes to pio1's 3 free SMs
  first (`newBidir` / `newNormal`). A pin past that budget reports `init-fail` (`state`, `spinMode`)
  and the hold tries it again every second (`drvInitFail` / `drvRetryMs`); it gets an SM when one is
  freed (a pin read as normal DShot moves to pio1, a pin goes FAILED or is in a bootloader). With
  more than 4 pins, list the wired ones first: an ESC on a pin with no SM is not seen arriving and
  cannot be armed until one is free. No new scheduler is planned for this.
- The passthrough primes in turns when there are more pins than free SMs (500 ms more for pins 5..8).
- A configurator shows an empty pin as a channel whose read failed (esc-configurator logs a read
  failure; the web page logs `ESC n: つながっていません` and goes on to the next channel).
- esc-configurator retries an InitFlash for about 6 s. The ~11 s ladder of a pin whose ESC was
  pulled (below) does not fit in that: that channel may fail once in esc-configurator (the web page
  waits 20 s), and reads fine on the next read.
- A pulled ESC stays wired until its pin goes EMPTY (a read that finds nothing): LOST is still "an
  ESC was here". `escs` lists it until then, and the next passthrough gives that pin the full entry
  ladder once (~11 s) before it is called absent.
- An empty pin is confirmed absent once a read finds nothing there (READ_EMPTY) or a passthrough
  probe gets no answer on a pin that was EMPTY before it (`lc::Pin::absent`). The MSP motor test
  then leaves it alone (a stock configurator sends every channel, and each empty pin would queue a
  ~10 s read ladder in front of the real ESCs); it reports absent (5). Any telemetry packet or a read
  that succeeds clears the latch; `arm i` and `read i` still try the pin. So a stock BLHeli_S (no
  telemetry) plugged into a pin confirmed absent is not armed by a configurator's motor test until a
  read (the page's, a configurator's, `scan`) finds it.
- An ESC left in its bootloader (for example, the Pico rebooted mid-passthrough) sends no telemetry,
  so the boot and hot-plug detection leave its pin EMPTY until a passthrough or `read i` finds it.
  The page's read and esc-configurator both find it; a CLI `read i` holds the session until `run i`.
- Changing the pin list needs a reboot; there is no `reboot` command.

## rev0 (the first release)

- Idle hold: start holding only when nothing else is being driven, and never retry an ESC whose
  read failed until `scan` or a configurator read.
- PWM output (50 Hz servo pulses) for firmwares that take it (AM32, stock BLHeli_S; not Bluejay,
  which accepts DShot only): its own deadman, the idle hold leaves PWM pins alone, and switching
  back to DShot needs no reboot.
- The idle hold covers every wired ESC (Bluejay and BLHeli_S beacon too), not only AM32.
- A bidir arm completes only once the ESC answers with telemetry and has taken the EDT enable.
  Bluejay with EDT safety arm (FORCE_EDT_ARM) never started after a passthrough when the motor test sent
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
  - A bootloader read on the drive path still stalls every stream (rev1: async, see above).
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
    4 idle, 5 absent, 6 stalled, 7 silent; 1 reserved in rev0, "being read" from rev1) with 0x5A in
    the high byte. Betaflight
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
