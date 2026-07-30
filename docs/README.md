# Start here

BlueGill drives BLHeli-S/Bluejay ESCs over DShot for low-KV ROV thrusters: an RP2040 runs the
operational core (feed-forward + closed-loop velocity, soft-sensing), and `host/` is the lab and
calibration layer. This page is the one you read before touching the bench, the vehicle, or the model.
Everything below is either a measured fact or an explicit statement that something is NOT known.

## Six things that will bite you

1. **Below the crossover seam, `tele` is not a measurement — it is a constant.** It reads exactly
   **182.0 rpm** for every command from 60 to 450 while the rotor actually goes 39 → 170 rpm, and only
   becomes honest from about cmd 520. Never close a loop or judge a regime on `tele` down there.
2. **The rotor never freely coasts. Zero commanded throttle is a BRAKE**, and it engages on a fixed
   ~0.30 s delay rather than at a speed. `brake_on_stop=0` does not disable it, and neither does
   `disarm`, letting the deadman expire, or holding zero while armed. Operationally: cutting thrust
   *brakes* a thruster, it does not windmill.
3. **Below about cmd 210 the rotor does not rotate.** It oscillates about zero and reverses direction
   several times per second. A mean speed there is not a speed, and the plant model deliberately
   refuses to report one (`coherent=False`).
4. **A stall is data, not an error.** When the velocity controller hits `ABORT_STALL` it commands zero
   and disarms, so every later write answers `err not-armed`. Guard your cleanup paths — three host
   tools once discarded the very trace they existed to capture by letting that raise.
5. **`trim` and `cmd` are ESC COMMAND units (~10.3 rpm/unit), not rpm.** So are `trim_max`/`dob_max`.
6. **Raw `thrust` needs a keep-alive** (200 ms, 500 ms deadman) and does **not** update `sense`.

## What is actually trustworthy

The measured behaviour is solid; the *mechanism models* that explain it are much weaker. Operating
decisions ride on the first row, the simulator's brake law rides on the third.

| Confidence | What | Evidence |
|---|---|---|
| **High** — repeated, both mechanical configurations | the 182 rpm telemetry floor; the zero-throttle brake and its fixed delay; the seam at cmd 525–530 with no resolvable hysteresis; startup overshoot 53–56 %; `rpm 2400` aborting while `rpm 1200` starts; the down-step not stalling | 3/3 or better per configuration, three cut paths, and the brake delay independently re-derived from raw encoder position |
| **Medium** | absolute inertia (`J`); the `deadman`/`zero_held` engagement delays | `J` is inferred through the drag table and carries a **measured ±8.4 %** bound; the non-`disarm` delays were never recorded prop-on. The prop/bare *ratio* (+29 %) is firmer than either absolute |
| **Low / not validated** | `L_PHASE_FITTED = 1.296 mH`; `BRAKE_CUTOUT_RPM = 670`; the brake torque law's structure; `deadman`/`zero_held` brake torque; the sine pull-out's slip dependence; the 60/80 °C thermal thresholds; anything about water | all fitted or assumed — see below |

**Why `L` matters and what it is.** `L` is not a measurement. It is the single free parameter of an
*assumed* mechanism — a short-circuit/asynchronous brake, `τ = K_T²·ω_slip·R/(R² + (pp·ω_slip·L)²)`,
which peaks at `ω_e = R/L`. It was introduced because the measured deceleration peaks and rolls off on
both sides, which is that law's signature, and it was then fitted to the brake data. It has moved 2× as
the surrounding model changed (0.682 → 1.335 → 1.296 mH). It is also the only parameter of that law with
an independent physical check available: **put an LCR meter across two motor phases.** Expect ~2.6 mH
line-to-line. If it reads ~1 mH, the brake cut-out is absorbing torque that belongs in the law, and the
brake model is curve-fitting rather than physics.

**One real out-of-sample result, for calibration of how much to trust the rest.** The brake law was
fitted on bare-shaft `disarm` decays only (enforced structurally: `plantid.fit` asserts it never sees
prop-on data) and then replayed against the propeller-fitted decays with nothing refitted — duration
error −4.9…+4.9 %, as good as in-sample. That is genuine evidence the *structure* is right. It is also
one inertia perturbation on one motor: not a different machine, and not water.

**The drag table is not drag.** `DRAG_TABLE_PROP` *falls* with speed, which no drag law permits, so it
is lumped bearing and iron loss rather than propeller drag. The ω² coefficient that water needs is
completely unmeasured. `drag` is therefore a swappable callable and no coefficient has been invented.

## Current state — read before flashing or releasing

- **The firmware on the device is NOT the firmware in the tree.** Workstream B (the soft-sensor validity
  gate and the staged thermal limiter) is committed but **has never been flashed or executed on
  hardware** — the running firmware still answers the old 11-field `sense` line and rejects
  `gain warntemp`. Every bench measurement in `docs/measured-plant.md` was taken on pre-B firmware.
- **There is no thermal protection running.** Even once flashed, nothing ties the EDT integer °C to a
  real winding or junction temperature; EDT reports the ESC's own MCU. RAW drive gets no thermal
  limiting at all, because only the RPM submode runs `vc.step()`.
- **Release is deferred** until a real thruster has been tested in water. Do not merge or tag before it.
- The propeller is currently fitted; the bench is 350 KV at 11.2 V with `cross_up=36 cross_dn=239`.

## Which document answers which question

| Question | Document |
|---|---|
| How do I drive it / what are the commands? | [`../README.md`](../README.md) |
| What does the plant do, and which numbers are measured vs fitted? | [`measured-plant.md`](measured-plant.md) |
| How do I get voltage and load out of eRPM alone, and when is it valid? | [`soft-sensor.md`](soft-sensor.md) |
| How does closed-loop velocity control work, incl. thermal limiting? | [`velocity-control.md`](velocity-control.md) |
| DShot / telemetry / bootloader / encoder wire formats | [`protocol-notes.md`](protocol-notes.md) |
| Why the architecture is shaped this way (historical design spec) | [`velctl-generalization.md`](velctl-generalization.md) |

Host tools all take `--dry-run` and `--analyze`, and always disarm in a `finally` block. The measurement
tools are `sinesweep.py` (forced-sine regime vs encoder truth), `steptrace.py` (commanded step at full
poll rate), `coastdown.py` (drag and the brake), and `plantid.py` (offline fit/validate/provenance).
