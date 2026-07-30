# Start here

BlueGill drives BLHeli-S/Bluejay ESCs over DShot for low-KV ROV thrusters: an RP2040 runs the
operational core (feed-forward + closed-loop velocity, soft-sensing), and `host/` is the lab and
calibration layer. This page is the one you read before touching the bench, the vehicle, or the model.
Everything below is either a measured fact or an explicit statement that something is NOT known.

## Six things that will bite you

1. **Below the crossover seam, `tele` is not a measurement — it is a constant.** It reads exactly
   fixed value for every command across the whole forced-sine range while the rotor speed varies by
   several times, and only becomes honest at the seam. (On our 7-pole-pair bench it sits at exactly
   182 rpm from cmd 60 to 450, honest from about cmd 520 — measure where yours sits.) Never close a
   loop or judge a regime on `tele` below the seam.
2. **The rotor never freely coasts. Zero commanded throttle is a BRAKE**, and it engages on a fixed
   ~0.30 s delay rather than at a speed. `brake_on_stop=0` does not disable it, and neither does
   `disarm`, letting the deadman expire, or holding zero while armed. Operationally: cutting thrust
   *brakes* a thruster, it does not windmill.
3. **Below about cmd 210 the rotor does not rotate.** It oscillates about zero and reverses direction
   several times per second. A mean speed there is not a speed, and the plant model deliberately
   refuses to report one (`coherent=False`).
4. **A stall is data, not an error.** When the velocity controller hits `ABORT_STALL` it commands zero
   and disarms, so every later write answers `err not-armed`. Guard your cleanup paths: if that
   exception propagates it will tear down the run and discard the trace of the stall itself.
5. **`trim` and `cmd` are ESC COMMAND units (~10.3 rpm/unit), not rpm.** So are `trim_max`/`dob_max`.
6. **Raw `thrust` needs a keep-alive** (200 ms, 500 ms deadman) and does **not** update `sense`.

## What is actually trustworthy

The measured *behaviour* above is solid — repeated across runs and across two mechanical configurations.
The *mechanism models* that explain it are much weaker, and the distinction matters because operating
decisions ride on the first and the simulator's brake law rides on the second.

- **Measured and repeated** — the telemetry floor, the zero-throttle brake and its fixed delay, the seam
  location and the absence of hysteresis, the startup overshoot, the target-dependent startup abort, the
  down-step not stalling. Treat these as facts about this ESC.
- **Inferred through a calibration** — inertia, and anything derived from the drag table. The drag table
  is the weak link: measured on a driven rotor, it *falls* with speed, which no drag law permits, so it
  is lumped bearing and iron loss rather than propeller drag. Inertia inferred through it therefore
  carries a real uncertainty, which you can bound yourself by measuring it at two different speeds and
  comparing the two answers.
- **Fitted parameters of an assumed mechanism** — the brake torque law's inductance and cut-out speed.
  These are the least trustworthy things in the model, and are tagged OPEN in the inventory for that
  reason.

**On the brake law specifically**, because it is easy to over-trust: the deceleration after a cut peaks
and then rolls off on both sides, which is the signature of a short-circuit / asynchronous brake,
`tau = K_T^2*w_slip*R/(R^2 + (pp*w_slip*L)^2)`, peaking at `w_e = R/L`. `L` is therefore **not a
measurement** — it is that law's single free parameter, adopted because the shape fits and then fitted to
the decay. It is also the only parameter of the law with an independent physical check available: **put
an LCR meter across two motor phases** and compare against roughly twice your fitted `L` (line-to-line).
If they disagree badly, the cut-out term is absorbing torque that belongs in the law, and the brake model
is curve-fitting rather than physics. Do that before relying on it.

**Calibrate in situ.** Every number in a plant inventory describes one motor, one wiring loom and one
supply — roughly a third of the total resistance is wiring, which is per-installation and corrodes. The
design assumes you re-measure on your own hardware rather than inheriting someone else's constants, which
is why the tools ship and the numbers do not.

## Before you flash or drive

- **Check what is actually running.** The firmware in a tree is not necessarily the firmware on the
  device. `sense <i>` and `gain <i> warntemp` tell you which generation you are talking to.
- **Thermal protection is opt-in and it is a backstop, not a guarantee.** The only thermal observable is
  the ESC's own MCU temperature over EDT — it is **not** a winding or junction temperature, and nothing
  here ties the reported integer °C to either. **RAW drive (`thrust`/`throttle`) gets no thermal limiting
  at all**, because only the RPM submode runs the controller. That includes every measurement tool.
- **Pass `--profile` explicitly.** The feed-forward curve is motor-specific; there is no safe default.

## Which document answers which question

| Question | Document |
|---|---|
| How do I drive it / what are the commands? | [`../README.md`](../README.md) |
| How do I get voltage and load out of eRPM alone, and when is it valid? | [`soft-sensor.md`](soft-sensor.md) |
| How does closed-loop velocity control work, incl. thermal limiting? | [`velocity-control.md`](velocity-control.md) |
| DShot / telemetry / bootloader / encoder wire formats | [`protocol-notes.md`](protocol-notes.md) |
| Why the architecture is shaped this way (historical design spec) | [`velctl-generalization.md`](velctl-generalization.md) |

Host tools all take `--dry-run` and `--analyze`, and always disarm in a `finally` block. The measurement
tools are `sinesweep.py` (forced-sine regime vs encoder truth), `steptrace.py` (commanded step at full
poll rate), `duty_sweep.py` (the effective duty map) and `coastdown.py` (drag and the brake window).
`host/pico_esc/plant.py` is the plant STRUCTURE; the numbers that go in it are yours to measure.
