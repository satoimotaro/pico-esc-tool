# escs simulation

`src/apps/esc_session.h` and `src/apps/fourway.h`, compiled unchanged on the host against stand-ins for the Pico (`stubs/`) and a model of the ESC on each pin (`sim.h`), run through the scenarios the bench found.

## Setup

Needs Linux (ucontext) and `g++` with C++17. No PlatformIO, no hardware.

```sh
make -C test/sim                 # build to /tmp/escs_sim and run
SIM_TRACE=1 /tmp/escs_sim        # with the lifecycle events, as the CLI prints them
```

### What it checks

- **An invariant on every DShot frame.** A non-zero throttle goes only to a RUNNING pin whose identity is the ESC's (3D and bidir), in its protocol, within the deadman of a drive command. Its value is exactly what that command asked for, as the real ESC reads the wire.
- **Scenarios:**
  - boot and the async reads;
  - Bluejay with FORCE_EDT_ARM restarting after 0.2..2 s stops;
  - AM32 3D in both directions;
  - a pull at zero and under throttle, and a 3D swap;
  - the passthrough with esc-configurator's 1 s reply timeout and retries, including DeviceReset;
  - stock BLHeli_S.
- **Known limits.** They are printed as `note:` lines, not checked.

### How the simulation works

- **Time.** One clock. core0 (the test, like `loop()`) and core1 (`core1Poll()`, like `loop1()`) run as two cooperative contexts. Whichever core is due first runs whenever the other waits, so a run is deterministic.
- **ESC models.** They follow the bench measurements and the firmware sources; `sim.h` lists each rule. One rule is not measured: whether a stock BLHeli_S takes the inverted bidir prime as the frames its bootloader entry needs (`Spec::bidirPrimes`). The test runs both answers.
- **A failing check is a lead, not proof.** Confirm it on the bench before changing the firmware: the models can be wrong.
