# vel_control — closed-loop velocity control for a bidir-DShot ESC

Portable, hardware-free C++ library: signed **target mechanical RPM in → signed DShot thrust out**.
It is the on-device (RP2040) home of the control law verified in Python
(`host/pico_esc/velocity.py`) — see `docs/velctl-generalization.md` (Phase A2).

## What it does

- **Feed-forward** from a per-motor calibrated `SpeedProfile` (thrust ↔ mech-RPM curve), plus
- a **PI trim** on `(target − measured mech RPM)` whose authority **fades with telemetry liveness**:
  above the firmware sine↔6-step seam the eRPM telemetry is live and the loop closes; below it
  (forced sine) telemetry is stale and the controller degrades to pure feed-forward.
- Back-calculation anti-windup on both the `±trim_max` and the outer `±tmax` clamps; over-speed /
  stall guards on the *live* measurement; optional down-catch staging through ~0.

It is **ESC-agnostic** — it needs only standard bidir-DShot telemetry, so it runs on stock
Bluejay/BLHeli-S as well as BlueGill.

## Usage — the declaring `main` owns the (plant-dependent) gains

```cpp
#include "vel_control.h"

// 1. a calibrated curve (points are static, no allocation)
static const vel::CurvePoint CURVE[] = { {0,0}, {620,2576}, {800,7173}, {900,9447}, {1000,11000} };
static const vel::Crossover  SEAM    = { 1500.0f, 1350.0f };          // up/dn eRPM (optional)
vel::SpeedProfile prof(CURVE, 5, /*pole_pairs=*/7, &SEAM);

// 2. a backend adapter over your DShot engine (implements vel::EscIo)
MyEscIo io1(/*index=*/1);

// 3. the controller — set gains DIRECTLY (this is the requested style)
vel::VelocityController esc1(io1, prof);
esc1.kp = 0.03f;  esc1.ki = 0.12f;  esc1.trim_max = 400.0f;   // per-motor, plant-dependent
esc1.slew_rpm_s = 4000.0f;

// 4. drive it: set a target, then step() every loop with the real elapsed dt
esc1.setTarget(5000.0f);                 // signed mech RPM
for (;;) {
    float dt = /* seconds since last step */;
    if (esc1.step(dt) != vel::Status::OK) { /* over-speed / stall / over-temp — motor stopped */ }
}
```

The **gains are plant-dependent** (a real 930KV 6-step plant is ~30× the sim), so they live on the
controller and are set by the app — the built-in `DEFAULT_GAINS` are only the sim-tuned starting
point. See `src/apps/vel_demo.cpp` for a full RP2040 example (an `EscIo` over `escs::` + a `vel <rpm>`
serial interface): `pio run -e vel_demo -t upload -t monitor`.

## The one invariant

The telemetry rpm your `EscIo::readTele` returns is **already mechanical** — the ESC firmware
pre-divides the DShot eRPM by pole pairs. Use it directly; dividing by pole pairs again is the 1/7
double-division bug. Pole pairs is used *only* to convert an rpm to eRPM for the seam classifier.

`readTele` must return **false when there is no fresh live frame** (forced sine / dropout / pre-arm),
not the last held value — that staleness is how the PI authority fades out. The RP2040 adapter uses
the `rpmStampMs` freshness field on `escs::Telem`.

## Test

`test/test_vel_control.cpp` is a native (host g++) test — the library pulls in no Arduino/PIO headers,
so it runs anywhere and mirrors the Python reference test (`host/tests/test_velctl_closedloop.py`):

```sh
cd test && g++ -std=c++17 -O2 -Wall -o /tmp/tvc test_vel_control.cpp && /tmp/tvc
```

A deliberately mis-scaled feed-forward (×1.25) must converge within 5 % once the PI closes on live
telemetry, while pure FF (`kp=ki=0`) misses by ~19 %; a stall must abort.
