# esc_lifecycle

One state per ESC signal pin, the identity rules, and the semantic drive mapping, as a pure C++ header that `src/apps/esc_session.h` acts on.

## Setup

Needs a host `g++` with C++17. The header includes only standard headers, so it builds without PlatformIO.

```sh
# 1. run the native tests (every transition the firmware relies on; the signal pin list; the
#    motor-test mapping)
cd lib/esc_lifecycle/test
g++ -std=c++17 -O2 -Wall -Wextra -Werror -o /tmp/tlc test_esc_lifecycle.cpp && /tmp/tlc
g++ -std=c++17 -O2 -Wall -Wextra -Werror -o /tmp/tpins test_esc_pins.cpp && /tmp/tpins
g++ -std=c++17 -O2 -Wall -Wextra -Werror -o /tmp/tmt test_motor_test.cpp && /tmp/tmt
# 2. build the firmware that uses it
cd ../../.. && pio run -e main
```

### What it decides

- `lc::Pin` holds the state (EMPTY, PENDING, READING, READY, RUNNING, LOST, FAILED). It is fed with `now` in ms, one valid telemetry packet per loop, read results and user requests, and it returns events (`lc::Ev`).
- `output(hold)` says what may go on the wire: nothing, bidir zero (detection), zero in the identity's protocol, or the drive stream.
- `gate()` and `mapDrive()` turn a command into a DShot value. A reverse request to a one-way ESC is refused, never clamped. `escs::drive()` is the only way a drive value reaches the wire.
- `ArmGate` decides when a drive stream counts as armed (the window, the first answer, the EDT enable sent). `EdtSched` decides which frames carry the EDT enable (at the start, once armed, after every run until the ESC is seen stopped).
- `identFromAm32()` / `identFromSilabs()` are the rules for reading bidir and 3D from a bootloader read: AM32 by its flash-size code (an unknown code is no identity), SiLabs bidir by the layout revision byte.
- `motor_test.h` (`mt::driveValue()`, `mt::pwmValue()`, `mt::wanted()`) maps a configurator's motor-test value onto an ESC for every pairing of the channel's meaning and the ESC's: reverse only when both are 3D.
- `ptRetry()` says whether the passthrough's probe polls a pin the long way (to 5 s after the lines went high: an ESC was known there) or briefly (EMPTY). A miss either way is final for that passthrough.
- `esc_pins.h` (`escpins::parse()`, `escpins::check()`) reads a signal pin list from text and decides whether a build may use it (1..8 distinct GPIOs, none reserved).

The timing constants and the reasons behind them are in `esc_lifecycle.h`. The design and its phase 1 / phase 2 split are in `docs/esc-lifecycle.md`. The command deadman (`escs::SPIN_DEADMAN_MS`, 500 ms) stays in `esc_session.h`: a refused command does not feed it.
