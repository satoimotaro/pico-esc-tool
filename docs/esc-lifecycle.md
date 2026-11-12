# ESC lifecycle: identify, drive, hot-plug

How the firmware knows what is on each ESC signal pin, when it may drive it, and how it notices a
plug-in, a pull or a swap. The code is `lib/esc_lifecycle/esc_lifecycle.h` (the pure rules, with
native tests), `src/apps/esc_session.h` (`escs::`, which runs them on the pins) and
`src/apps/fourway.h` (the 4-way passthrough). `test/sim` runs `escs::` on the host against ESC models.

Contents:
1. Requirements.
2. Current behaviour.
3. Known limits.
4. Not built yet.
5. History: rev0, phase 1, rev1.

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

## Current behaviour

### One state per pin

Every pin has one `lc::Pin` (`escs::detail::ch[i].pin`). `spinPoll()` feeds it telemetry and time
and puts on the pin what its state allows (`lc::Pin::output`, with the idle hold on).

| state | meaning | on the wire (hold on) | motor-test code |
|---|---|---|---|
| EMPTY | nothing seen | bidir zero, to detect a plug-in | 5 |
| PENDING | something appeared, not read yet | bidir zero | 1 (2 if an arm waits) |
| READING | in its bootloader (core1 owns the pin) | nothing | 1 (2 if an arm waits) |
| READY | identity known | zero in the identity's protocol | 3 armed, 2 arming, 4 no stream, 7 silent |
| RUNNING | being driven | the drive stream, mapped by identity | 3, 2 arming, 6 stalled, 7 silent / open loop |
| LOST | its telemetry stopped; identity dropped | bidir zero | 5 |
| FAILED | the read failed | nothing; no automatic retry | 5 |

Transitions (`lc::Pin`, events in `lc::Ev`):

- EMPTY or LOST → PENDING (APPEARED): 3 valid packets within 250 ms, each counted at least 15 ms
  after the previous one (`APPEAR_PKTS`, `APPEAR_WINDOW_MS`, `APPEAR_GAP_MS`). The driver keeps only
  the newest telemetry word and core0 polls every ~200 us, so the spacing is what makes 3 samples
  real evidence.
- PENDING → EMPTY (GONE): found by telemetry (not asked for), then silent for `LOST_MS` while it
  waits for its read.
- PENDING → READING → READY: the async read (below). An `arm` that waited for it goes on to RUNNING.
- A read that fails → FAILED if telemetry was fresh (under `LOST_MS` old) when it started, else EMPTY
  (READ_EMPTY: e.g. `arm` on a bare pin, `scan` after a pull, a read after a session or a
  passthrough). FAILED sends nothing, so this keeps a read-fail, reboot, telemetry, re-read loop out.
  FAILED is left only by a user action (`scan`, `read i`, a configurator read).
- READY → RUNNING: `arm`. A READY pin whose zero stream is already armed in the asked protocol goes
  RUNNING with no new arm window, so `arm` and the motor test stay fast. RUNNING → READY: `disarm`,
  stop, a PWM pin left unfed.
- READY or RUNNING → LOST: no valid packet for `LOST_MS` = 400 ms once the pin has answered on its
  current stream (the reboot gap after a read is expected). Under throttle on a stall-watched stream
  the silence is the stall watch's instead (below). LOST → PENDING when telemetry returns, so a swap
  always passes through LOST and a new ESC is never driven with the old one's identity (R2, R3).
- No answer within `ANSWER_MAX_MS` = 8 s on a READY bidir stream (15 s, `ANSWER_MAX_TUNE_MS`, with a
  startup melody): reported once (NO_ANSWER, `# esc i: no telemetry within 8 s ...`) and kept READY,
  armed anyway. Its identity is then unconfirmed: if it ever answers, the pin goes PENDING and is read
  again before it is driven.
- The first answer after a read more than `ANSWER_DUE_MS` = 1.5 s after the stream started (and
  before `ANSWER_MAX_MS`) sends the pin to PENDING, once (LATE_ANSWER): it may be another ESC, swapped
  in while the read one booted. Not with a startup melody. Measured (`state` shows `ans=`): Bluejay
  0.21 0.75 s, AM32 2.21 0.03 s.

Hidden sub-states on top of the seven (fields of `lc::Pin`):

- `queued`: PENDING by request (user, arm, an aborted read, a write); never times out to EMPTY.
- `armWanted`: an `arm` came before the identity; RUNNING once the read is OK.
- `unconfirmed`, `fresh` / `lateOnce`: the NO_ANSWER and LATE_ANSWER rules above.
- `absent`: confirmed absent (a read found nothing, or a passthrough probe missed a pin that was
  EMPTY before it). Only implicit arms (a configurator's motor test sends every channel) skip such a
  pin; `arm i` and `read i` still try it. Any packet or a successful read clears it.
- `prePt`: the state when a passthrough began (how long its probe polls, what it returns to).
- `openLoop`: ROV, RUNNING with its telemetry gone.

### Reads

- Reads on the drive path (hot-plug, `arm` of an unread pin, the motor test) run on core1 as one
  asynchronous job, `Op::READID`: prime, line high, the entry below, learn, run its app. core0 keeps
  streaming to every other pin and collects the result on a later loop. One job at a time, round
  robin over PENDING pins.
- The explicit config ops (`scan`, `read`, `enter`, `editpage`, `erase`, `writeflash`, `readflash`,
  `preset`, web flash, `blprobe`) and the 4-way passthrough are synchronous, but every core0 wait
  pumps the other pins' frames, telemetry, deadman and PWM.
- One bootloader line. A read cannot run while `scan`, `read` or `enter` holds an ESC in its
  bootloader. A pin waiting to be read wins once the session is idle: no op on it for 5 s
  (`SESSION_IDLE_MS`), and never a session that has been written (an `erase`/`writeflash` sequence
  must not see its ESC run mid-image; `run` ends it). The held session is then learned and run.
  `arm` of a pin that must be read first runs a foreign held session at once, unless it was written:
  then `err busy (ESC <s> is being written; run it first)`. `arm` on a pin not read yet answers
  `arming (reading first, then ~3s) (state X)`; hosts poll `state` for RUNNING
  (`host/pico_esc/drive.py` `wait_read`).
- Writes never trust a cached reply. A write makes closing the session read the identity again
  before RUN. Two failed keepalives in a row mark the session lost, and the next op enters again with
  a fresh CONNECT. A 4-way re-select of a non-AM32 ESC sends a fresh BootInit and adopts a different
  reply as a new ESC; AM32 answers only the first BootInit, so it re-adopts the cached reply if a
  keepalive answers (a swap there is caught by the re-read after the passthrough). The page re-reads
  the channel before a settings write and refuses one that differs (`読み込んだときと違います`); a
  firmware flash reads it fresh anyway (`flashCheck`).
- Identity from the bootloader (`learnHere`): AM32 by its flash code (`lc::am32EepFor`; an unknown
  code is not read), 3D from `bi_direction`, the melody from `tune[]`. SiLabs: bidir from the layout
  revision at 0x1A02 (Bluejay 200..209, BlueGill 226, `lc::silabsBidir`) or a name with "JESC", 3D
  from the direction byte, the melody from 0x70.
- LittleFS writes (config, the pin list) run under `escs::FsGuard`: core1 parks in its idle branch
  first, because a flash write pauses the other core and would cut a bit-banged bootloader byte in
  half. A refusal answers `err busy (ESC read in progress, retry)`.

### Policies

| | identity from | telemetry LOST while running |
|---|---|---|
| configurator (tool) | bootloader read (R1) | stop that ESC only, report (`lost, stopped`) |
| ROV | bootloader read today (see "Not built yet") | keep streaming, report (`# esc i: lost, open loop`) |

`escs::setPolicy`. Only these two columns differ; the state machine and the reads are shared (R4).

### Driving

- One drive path: `escs::drive(i, v)`. A semantic value, signed thrust -1000..1000 on a 3D ESC,
  throttle 0..2000 on a one-way one, mapped to DShot by the identity (`lc::mapDrive`). Refused, with
  the target untouched and the deadman not fed, unless RUNNING and armed: `err not-armed`,
  `err not-armed (state X)`, and `err reverse-on-one-way (ESC i is not 3D)` (a reverse request to a
  one-way ESC is refused, not clamped). The rpm loop (`Thruster`) goes through it too, and `rpm` is
  refused on a one-way ESC (`err needs-3d`).
- Protocol: bidir DShot (eRPM/EDT telemetry) when the identity says so (Bluejay, BlueGill, JESC,
  AM32), else normal DShot, which every BLHeli_S takes.
- Arm (`lc::ArmGate`): zero for `ARM_MS` = 3 s. A bidir ESC counts as armed only once it answers and
  has taken the EDT enable, plus `ARM_ANSWER_HOLD_MS` of zero after its first answer; one that never
  answers is armed after `ANSWER_MAX_MS` anyway. EDT (`lc::EdtSched`) is sent at the stream start,
  once armed, and after every run until a fresh eRPM 0, then once a second: Bluejay with
  FORCE_EDT_ARM refuses a start without it and clears it at the end of every run.
- Deadman: a target not fed for 500 ms (`SPIN_DEADMAN_MS`) goes to 0.
- Stall watch (`lc::Stall`, Bluejay/BLHeli-derived bidir only, not AM32): an armed RUNNING stream
  under throttle that has answered (`lc::Pin::stallWatched`). Its verdicts:
  - ZERO: eRPM 0 for 1.5 s. The clock starts only on an eRPM reading of 0, so forced sine (EDT
    flowing, no eRPM) is never a stall while a valid EDT packet comes at least every 500 ms
    (`STALL_SILENT_MS`).
  - QUIET: not turned yet and no valid packet for 0.5 s (a failed start beeps without telemetry).
  - SILENT: the same silence after it has turned. Tool policy: LOST, identity dropped. ROV: the open
    loop.

  ZERO and QUIET re-arm the same stream (zero, answer, EDT) with the identity kept and the target
  held at 0 until a 0 is sent, so a jam is never restarted on its own. A re-arm with no answer within
  8 s is LOST. After 3 restarts without a run in between (`STALL_MAX`), only a 0 held for 1 s frees
  it.
- Loop-gap guard (`gapCheck`): a pin not polled for 50 ms moves its lifecycle clocks on by the gap,
  and its stall clocks restart. Five gaps in a row are reported once (`# esc i: core0 loop slow ...`).
- PWM (50 Hz servo pulses) for firmwares that take it (AM32, stock BLHeli_S; not Bluejay): only on a
  READY or RUNNING pin, stop width 1000 us, or 1500 us on a 3D ESC. A width sent during the 5 s arm is
  not kept, and after it only the stop width passes until one stop width has come (`lc::pwmGate`).
  The 500 ms deadman also runs from a timer IRQ, every sync bootloader visit sends the stop width, CLI
  bootloader commands are refused while a pin is fed above its stop, and a pin unfed for 3 s after
  its arm goes back to READY and the hold.
- Motor test (MSP): the state goes in MOTOR_TELEMETRY's mAh field with 0x5A in the high byte
  (`lc::Ui`, `lc::uiCode`; codes in the state table). The page locks 1, 2, 5 and 6, and forgets its
  read of a motor (Bluejay, BlueGill or AM32 only) after 3 prompt polls in a row of 7, or of 5 once that motor reported a live code since the
  last test start or stop, read or passthrough exit. The 3D/one-way meaning of a value is fixed per
  test on both sides: the page declares it (`mtmode <mask>`, answered `mtmode|xx`) ahead of every
  frame, and without an answer treats every motor as FEATURE_3D says. The firmware latches its
  FEATURE_3D answer when a client reads it, until USB DTR falls or it is read again with no test
  running. An undeclared channel takes an ESC over only above the reported stop. The mapping is
  `lib/esc_lifecycle/motor_test.h`. A test session ends on DTR fall or after 2 s without MSP.

## Signal pins and presence

Which GPIOs carry ESCs, and which of them have an ESC on them now, are two separate things, and
neither needs a rebuild or a `scan`.

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
- An ESC on GP16 or GP17 switches the AS5600 encoder off (its I2C0 is on those pins).
- `escs::detail::ch[]` is sized `escs::MAX` (8); `escs::COUNT` is this boot's count. `main.cpp`
  declares eight `Thruster`s; `EscTool` uses the first `COUNT`. `cfg load` restores every stored one,
  `cfg save` stores the first `COUNT`.

### One index per pin

- ESC k is `PINS[k]` everywhere: the CLI index, MSP motor k (MOTOR, MOTOR_CONFIG, MOTOR_TELEMETRY,
  SET_MOTOR, `mtmode` bit k, `pwmch k`), the passthrough channel k and the web page's ESC k. The MSP
  motor count is the number of configured pins, so a configurator that reads it once at connect
  (the AM32 configurator hides its Read button at 0) always sees them. No numbering depends on which
  ESCs answer, so a plug-in or a pull, mid-test or not, never moves a channel onto another ESC.
- A pin with no ESC reports state 5 (absent) in MOTOR_TELEMETRY; the page locks its slider.

### Presence

- Nothing is stored. A pin is wired when its lifecycle state is not EMPTY: a bidir ESC is found by
  the hold's telemetry, any ESC by a read (`scan`, a configurator read, an arm). `escs` reports
  (`escs|<n>/<count>|<indices>`); its old arguments are ignored with a note. The old present mask
  `/cfg/present` is deleted at boot.
- Each held pin takes a PIO SM. Bidir fits only pio0's 4 SMs on a Pico W (the radio is on pio1);
  normal DShot goes to pio1's 3 free SMs first (`newBidir` / `newNormal`). A pin past that budget
  reports `init-fail` and the hold tries it again every second.

### Passthrough

- MSP_SET_PASSTHROUGH answers at once with every configured pin (esc-configurator times out an MSP
  reply after 1 s). Every pin is READING for the whole passthrough: no hold, no async read.
- `fwBegin` primes every pin at once (in turns, when there are more pins than free SMs), holds the
  lines high and shares the quiet window. A pin with a known bidir ESC is primed with bidir DShot,
  and if that ESC did not answer (booting, playing its melody) its own InitFlash primes it again until
  it answers. Every other pin is primed with normal DShot.
- Each channel's InitFlash then polls its own pin with BootInit: up to `ENTRY_MAX_MS` after the lines
  went high if its lifecycle knew an ESC there when the passthrough began (`lc::Pin::ptRetry`) or it
  answered earlier in the same passthrough, else up to `ENTRY_QUIET_MS + ENTRY_BARE_MS` (at least
  500 ms either way). A miss is final for that passthrough; retries fail at once. A channel whose ESC
  was run in this passthrough (DeviceReset, e.g. after a flash) and is selected again gets one fresh
  full entry. A stock BLHeli_S that was never read is EMPTY, but it is in its bootloader after the
  quiet window and answers the probe.
- No InitFlash reply waits more than 800 ms (300 ms for one that came in with the passthrough;
  `INIT_BUDGET_MS`, `INIT_BUDGET_FIRST_MS` in `fourway.h`). esc-configurator times every 4-way reply
  out after 1 s, sends the command again, and takes whatever frame comes next as its answer, so a
  late reply would answer the command after it. While its pin is still priming, quiet or polling,
  InitFlash answers `0x07` (pending, Betaflight's `ACK_D_UNKNOWN_ERROR`), and the next InitFlash for
  that channel carries on toward the same deadline. esc-configurator retries on any NACK (250, 500,
  750, 1000 ms apart, 5 tries in its read, about 6.5 s); the web page retries on `0x07` only, for up
  to 20 s.
- `fwRun` (DeviceReset, or the end) learns the ESC again while it is still in its bootloader, so the
  motor test that follows arms without another bootloader visit. `fwEnd` sends a pin that was never
  selected back to EMPTY/FAILED if it started there, else to PENDING (it may have been swapped).

### Bootloader entry timing

- One entry timeline, `lc::Entry`, for `read i` / `scan` / `enter` and every passthrough channel
  (spread over its pending InitFlash answers). The async `Op::READID` uses the same timing.
  - Prime: DShot zero for `ENTRY_PRIME_MS` = 500 ms (BLHeli_S needs valid frames first).
  - Line high, then `ENTRY_QUIET_MS` = 1000 ms with no traffic at all.
  - BootInit polled until an answer or `ENTRY_MAX_MS` = 5000 ms after line high.
- The prime waits for an answer. A pin where a bidir ESC is known is primed with bidir DShot until
  that ESC answers, up to `ENTRY_PRIME_MAX_MS` = 3 s, or `ENTRY_PRIME_TUNE_MS` = 15 s when its settings
  have a startup melody (`Identity::tune`). An ESC that was just run out of its bootloader is deaf
  while it boots and plays its melody, and a prime that ended inside the melody left it waiting in its
  app. Bench (2026-11-07), Bluejay 0.21 with its melody on: two page reads back to back both find it
  (10 s each), and so does a DeviceReset followed by InitFlash.
- Measured on 2026-11-07 with `blpoll <i> <holdMs> <pollMs>` (line high right after the zero-throttle
  hold stream, app running):
  - AM32 2.21 (ARIA, startup tune off): a single BootInit answers from 700 ms; polling from 0 ms
    does not disturb it (answered at 538 ms, 6/6).
  - Bluejay 0.21 (EFM8BB21): a single BootInit answers from 700 ms; polling from 0 or 300 ms keeps
    it out of its bootloader (no answer within 6 s, 6/6). That is why the quiet window has no
    traffic.
  - Quiet 800 or 1000 ms, then poll: both answer the first BootInit (814 / 1014 ms), 12/12.
- Costs: a present ESC about 1.5 s (prime + quiet + first answer) plus its read; an absent pin
  through `read i` or an async read 5.5 s. In a passthrough an EMPTY pin costs at most 0.5 s. A page
  read with two ESCs on four pins takes about 3.2 s on the bench.

### Timing constants

| constant | value | file | what |
|---|---|---|---|
| `LOST_MS` | 400 ms | esc_lifecycle.h | silence that is LOST (and PENDING → EMPTY) |
| `APPEAR_PKTS` / `_WINDOW_MS` / `_GAP_MS` | 3 / 250 / 15 ms | esc_lifecycle.h | the plug-in debounce |
| `ANSWER_MAX_MS` / `_TUNE_MS` | 8 / 15 s | esc_lifecycle.h | NO_ANSWER; armed anyway |
| `ANSWER_DUE_MS` | 1.5 s | esc_lifecycle.h | a later first answer after a read is LATE_ANSWER |
| `SILENT_MS` | 200 ms | esc_lifecycle.h | code 7 |
| `ENTRY_*` | see above | esc_lifecycle.h | the bootloader entry |
| `ARM_MS` / `ARM_ANSWER_HOLD_MS` | 3 s / 300 ms | esc_lifecycle.h | the arm window |
| `EDT_*` | 1 s refresh, bursts of 20 / 10 | esc_lifecycle.h | the EDT enable |
| `STALL_ZERO_MS` / `_SILENT_MS` / `_MAX` / `_REST_MS` | 1.5 s / 0.5 s / 3 / 1 s | esc_lifecycle.h | the stall watch |
| `SPIN_DEADMAN_MS` | 500 ms | esc_session.h | the deadman (DShot and PWM) |
| `PWM_ARM_MS` / `PWM_RELEASE_MS` | 5 s / 3 s | esc_session.h | PWM arm, release |
| `POLL_GAP_MS` / `SLOW_GAPS` | 50 ms / 5 | esc_session.h | the loop-gap guard |
| `SESSION_IDLE_MS` | 5 s | esc_session.h | an idle held session yields to a read |
| `INIT_BUDGET_MS` / `_FIRST_MS` | 800 / 300 ms | fourway.h | the longest InitFlash reply |

## Known limits

Swaps and identity:

- Non-bidir ESCs (stock BLHeli_S) cannot be seen arriving or leaving. They are not re-read on every
  start; they are read by `scan` or a configurator read. The rule for the user: after changing the
  wiring (moving an ESC to another pin, or plugging one in during a motor test), press
  「ESC を読み込む」 again (or `read i` / `scan`). A motor that went away mid-test stays locked for the
  rest of that test, and the settings card stays cleared until the next read.
- A swap faster than the old ESC's own first answer after a read (Bluejay 0.75 s) is not seen. With
  a melody on, the LATE_ANSWER check is off: its melody covers any swap, and a read again would only
  play it again.
- A swap under throttle that keeps telemetry flowing in eRPM 0 readings (ZERO) or that never turned
  (QUIET) is re-armed with the old identity. An ESC plugged in within the following 8 s answers the
  re-arm as the old one, though held at 0 until a zero command. If the old identity was 3D and the
  new ESC is one-way, a forward command is then about 50 % throttle there, and "reverse" runs forward
  too. A pull after the motor has turned (SILENT) drops the identity (tool policy).
- A bidir ESC whose telemetry never decodes stays READY (armed after 8 s anyway) and is not
  stall-watched; a pull or swap on it is not seen, like stock BLHeli_S.
- A pulled ESC stays wired until its pin goes EMPTY (a read that finds nothing): LOST is still "an
  ESC was here". `escs` lists it until then, and the next passthrough gives that pin the long probe.
- A stock BLHeli_S (no telemetry) plugged into a pin confirmed absent is not armed by a configurator's
  motor test until a read (the page's, a configurator's, `scan`) finds it.
- An ESC left in its bootloader (the Pico rebooted mid-passthrough, or an ESC moved during one, whose
  new pin held the line high while it powered up) sends no telemetry, so its pin stays EMPTY until a
  passthrough or `read i` finds it. A CLI `read i` holds the session until `run i`.

Timing:

- A startup melody makes an ESC deaf for seconds after every read: the bench Bluejay answers 8.3 s
  after a read with it on, 0.75 s with it off (`state` shows `ans=` and `tune=`). The page waits up
  to 20 s; esc-configurator gives up on an InitFlash after about 6.5 s, so read it there once the
  melody has ended.
- `ENTRY_MAX_MS` (5 s from line high) is measured on one AM32 build with its startup tune off. An
  AM32 build that takes longer to reach its bootloader is missed: in a passthrough that channel fails
  for that passthrough, elsewhere the read fails and `read i` tries again.
- The quiet window must stay free of BootInit: Bluejay polled earlier than ~700 ms after line high
  never entered its bootloader. A Bluejay/BLHeli_S build slower than the measured one would need a
  longer `ENTRY_QUIET_MS`.
- A pin with no bidir ESC known is primed with normal DShot. Whether a stock BLHeli_S also counts
  inverted frames is not measured (no stock BLHeli_S on the bench); `test/sim` runs both cases.
- A loop slower than 50 ms on every pass disables LOST detection (and SILENT and the stall silence
  rule) altogether; SLOW_LOOP is the only report of it.
- Bluejay's beacon (minutes at zero throttle) and its stall beeps send no telemetry. One longer than
  400 ms at zero makes the pin LOST, and it is read again when it answers. Needs a bench check.
- A jam while the motor is running may show up as total silence (SILENT) rather than as eRPM 0
  (ZERO), depending on what Bluejay sends as it drops out of its run. Under the tool policy that ESC
  then goes LOST and is read again instead of being re-armed. Needs a bench check.
- AM32 is not stall-watched, so under throttle it is under the plain 400 ms LOST rule: a telemetry
  decode dropout longer than that stops it in the web tool and drops its identity.

Resources and the UI:

- The idle hold streams to every configured pin, so an empty pin still holds a PIO SM. With more than
  4 pins, list the wired ones first: an ESC on a pin with no SM is not seen arriving and cannot be
  armed until one is freed (a pin read as normal DShot moves to pio1, a pin goes FAILED or is in a
  bootloader). The passthrough primes in turns (500 ms more for pins 5..8).
- A configurator shows an empty pin as a channel whose read failed (esc-configurator logs a read
  failure; the web page logs `ESC n: つながっていません` and goes on).
- Changing the pin list needs a reboot; there is no `reboot` command.
- Under the tool policy code 7 shows for one poll at most (LOST follows 200 ms later).

Motor test:

- A session ends on DTR fall (USB). Transports without DTR (the `main_uart` build, a WebSocket
  bridge) only end it on MSP silence: a client that reconnects within 2 s inherits the old test.
- The `mtmode` declaration and the FEATURE_3D latch are RAM only; the page sends the declaration
  again on each frame.
- The mAh field reads as about 23 Ah in Betaflight Configurator. The page reads it only from a board
  that says PICO.
- There is no hardware watchdog: a core0 hang with interrupts off keeps the last PWM width.
- `thrust` and the 3D motor-test range reach full scale at ±953. Past that the value is clamped,
  because the next 47 units would fall into the other direction's band.

## Not built yet

- The explicit config commands asynchronous with deferred replies.
- ROV identity from a telemetry id or declared config (a mismatch keeps the pin FAILED), and `scan`
  at ROV launch.
- The RPM controller falling back to open loop on LOST in ROV mode (today the stream keeps going and
  `# esc i: lost, open loop` is reported).

## History

### rev0 (the first release, PR #12)

- An idle hold that started only when nothing else was driven, and never retried an ESC whose read
  failed until `scan` or a configurator read. It covered every wired ESC (Bluejay and BLHeli_S
  beacon too), not only AM32.
- PWM output, with its own deadman; switching back to DShot needs no reboot.
- A bidir arm completes only once the ESC answers with telemetry and has taken the EDT enable:
  Bluejay with FORCE_EDT_ARM never started after a passthrough when the motor test sent throttle at
  once (EDT went out only in a zero-throttle gap after the arm).
- The end of a passthrough re-reads each ESC while it is still in its bootloader.
- The per-test 3D meaning (`mtmode`, the FEATURE_3D latch) and the stall re-arm.
- Its limits, fixed since: a bootloader read on the drive path stalled every stream; swaps were not
  detected (a bidir ESC silent for 1 s while armed was reported as code 7, and the page forgot it
  after 3 polls); a failed read left the ESC unknown and was not retried.

### Phase 1 (PR #13) and what it replaced

Before it:

- `escs::spinArm` read the ESC through its bootloader whenever its cached info was unknown, and core0
  waited for core1 the whole time: every DShot stream on the board stopped for ~1.3–3.4 s for a
  present ESC, ~11.5 s for an absent one.
- The cache (`infoKnown/infoBidir/infoRev`) was RAM only, not tied to the ESC on the pin now, and a
  failed read still armed in normal DShot with the stale direction.
- The idle hold re-armed an ESC 5 s after it was stopped; for an ESC whose read failed that was a
  fresh bootloader visit, so one dead ESC could stall everything every 16 s.

Phase 1 brought the state machine, the async read, hot-plug, the tool/ROV switch, semantic drive
with refusals, the `state` command and `FsGuard`. The persisted AM32 mask (`armMask`) went, and later
the present mask (`/cfg/present`, deleted at boot). Where the code refined the design: READY takes
`arm`, not a bare non-zero command; a read that fails without fresh telemetry goes to EMPTY; the
plug-in debounce; writes never trust a cached reply.

### rev1 and after

- rev1 (PR #13) carried rev0's drive rules onto the lifecycle: `lc::Ui` for the motor-test codes,
  `lc::Stall`, the loop-gap guard, `lc::pwmGate`.
- PR #14: the pin list at run time, one index per pin, presence without a scan, the quiet + poll
  entry (it replaced a fixed ladder of a 1 s hold, then 4 s twice, and a 4.5 s settle).
- PR #15, #16: a fresh passthrough per read, InitFlash replies within 1 s (`0x07` pending), a one-way
  ESC refusing 3D thrust, no guessed AM32 block.
- PR #17: one drive path; `lc::ArmGate`, `lc::EdtSched`, `motor_test.h`; the legacy `esc_tool` env
  retired (tag `legacy-esc-tool`).
- PR #18: `test/sim`.
- PR #19: one entry timeline (`lc::Entry`), the melody limits, LATE_ANSWER, the normal-DShot prime.
  Before it, the 8 s NO_ANSWER rule took a melody ESC as unconfirmed and read it again, which played
  the melody again: a read every 9 s, for good.
