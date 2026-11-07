# esc_lifecycle

One state per ESC signal pin, the identity rules, and the semantic drive mapping, as a pure C++ header that `src/apps/esc_session.h` acts on.

## Setup

Needs a host `g++` with C++17. The header includes only standard headers, so it builds without PlatformIO.

```sh
# 1. run the native tests (every transition the firmware relies on; the signal pin list)
cd lib/esc_lifecycle/test
g++ -std=c++17 -O2 -Wall -Wextra -Werror -o /tmp/tlc test_esc_lifecycle.cpp && /tmp/tlc
g++ -std=c++17 -O2 -Wall -Wextra -Werror -o /tmp/tpins test_esc_pins.cpp && /tmp/tpins
# 2. build the firmware that uses it
cd ../../.. && pio run -e main
```

### What it decides

- `lc::Pin` holds the state (EMPTY, PENDING, READING, READY, RUNNING, LOST, FAILED). It is fed with `now` in ms, one valid telemetry packet per loop, read results and user requests, and it returns events (`lc::Ev`).
- `output(hold)` says what may go on the wire: nothing, bidir zero (detection), zero in the identity's protocol, or the drive stream.
- `gate()` and `mapDrive()` turn a command into a DShot value. A reverse request to a one-way ESC is refused, never clamped.
- `identFromAm32()` / `identFromSilabs()` are the rules for reading bidir and 3D from a bootloader read.
- `ptRetry()` says whether the passthrough's probe polls a pin the long way (to 5 s after the lines went high: an ESC was known there) or briefly (EMPTY). A miss either way is final for that passthrough.
- `esc_pins.h` (`escpins::parse()`, `escpins::check()`) reads a signal pin list from text and decides whether a build may use it (1..8 distinct GPIOs, none reserved).

The timing constants and the reasons behind them are in `esc_lifecycle.h`. The design and its phase 1 / phase 2 split are in `docs/esc-lifecycle.md`. The command deadman (`escs::SPIN_DEADMAN_MS`, 500 ms) stays in `esc_session.h`: a refused command does not feed it.
