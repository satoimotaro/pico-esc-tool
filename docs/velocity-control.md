# Velocity control — velctl (feed-forward + Phase A1 closed loop)

`velctl` runs a BlueGill ESC at a target mechanical **speed** by inverting one per-motor
calibrated curve produced by `velcal`, and (Phase A1) trims that feed-forward with a PI loop on
bidir-DShot telemetry whose authority **fades with telemetry liveness**. It is ADDITIVE on
`host/pico_esc/`: it adds `pico_esc.velocity` (`SpeedProfile`, `VelocityController`,
`measure_steady_speed`) and the `velcal.py` / `velctl.py` CLIs, and extends the dry-run
simulator; it changes no wire protocol, no existing CLI, and no existing library surface.

> **Phase A1 = the Python control-law reference / bench tool.** This host loop is where the
> control law is written, tuned, and PROVEN in the `SimEncEscHost` dry-run. The runtime home is
> the Pico C++ port (Phase A2); A1 stays as the calibration / bench-verification tool.

## Terminology — read this first

"**thrust**" in this tool is the **signed ESC command** (−1000..+1000, bidirectional DShot) —
the controllable **input we SET**. It is **NOT** physical force or thrust, and there is **no
force sensor** anywhere in the loop. The name is historical (it is the `thrust` wire command).

The calibrated curve maps:

```
    ESC command ("thrust", −1000..+1000)  ->  measured mechanical RPM   (via the AS5600 encoder)
```

At runtime `velctl` inverts it:

```
    target RPM  ->  ESC command            (SpeedProfile.thrust_for(rpm))
```

So `velctl speed --rpm 320` means "spin at 320 RPM"; the controller looks up the ESC drive
command for 320 RPM from the curve. `--rpm` is a **speed**, never a force. Reverse is a
negative `--rpm` (the inverse is odd-symmetric).

## Design

`velcal` sweeps the ESC command 0 → `--max-thrust` and, at each level, measures the steady
mechanical RPM with the encoder (settle, then average — `measure_steady_speed`, adapted from
`tune_sine_amp._drive_measure`). The result is a monotonic `SpeedProfile` YAML:

```yaml
motor: 930kv_12n14p
pole_pairs: 7
source: sim
crossover: {up_erpm: 2100.0, dn_erpm: 1600.0, bytes: [54, 195]}
points:
- {thrust: 0.0,   rpm: 0.0}
- {thrust: 500.0, rpm: 178.0}
- ...
```

`velctl` loads it, slews the setpoint at `--slew` RPM/s (so a step never demands a full-scale
command jump), and each 50 Hz tick sends `SpeedProfile.thrust_for(setpoint)` (the feed-forward)
**plus a telemetry-fed PI trim** over the existing signed `thrust` command (see the closed-loop
section below). Where telemetry is stale (forced sine, below the seam) the trim fades out and the
command is pure feed-forward.

YAML I/O reuses the config codec's `config._emit_yaml` / `config.load_yaml`; the loader
strict-**rejects** a non-monotonic curve (thrust must be strictly increasing, rpm
non-decreasing) because a non-monotonic curve inverts ambiguously.

## One curve subsumes the S3 crossover

BlueGill's S3 firmware runs **forced-sine** commutation at low speed (precise, slow: full
scale ≈ 357 mech RPM) and hands off to fast, efficient **6-step BEMF** commutation above a
configured eRPM (`sine_cross_up`), dropping back below `sine_cross_dn` (a hysteresis band).
The two regimes have different command→speed laws, so there is a discontinuity — a **handoff
jump** — at the seam.

Because `velcal` measures across the **whole range with the crossover enabled**, that jump is
baked into the points. The single calibrated curve therefore **subsumes** the discontinuity:
pure feed-forward at runtime needs **no regime knowledge**, because inverting the one curve
already lands on the right command on either side, and the firmware performs the transition
itself. `velctl --crossover` just writes the profile's `sine_cross_up`/`sine_cross_dn` bytes
(via `esc.config.set` → editpage, the only EEPROM writer) and restarts the app.

Note: the swept curve has an unreachable **gap** across the seam (the speeds between the top
of the sine regime and the bottom of the load-line are not produced by any single command).
Inverting into that gap returns a command on the jump segment — open-loop, no extrapolation.

## Sensorless vs. --encoder

Sensorless is the **default**. The encoder is used only:

- by `velcal`, once, to build the curve (encoder-in-the-loop calibration); and
- optionally at runtime with `velctl --encoder`, as an **independent verify-log column**
  (`enc_rpm`) that is written to the CSV but **never feeds back into the command** (the closed
  loop trims on **telemetry**, not the encoder — the encoder stays a pure cross-check).

## Phase A1: the closed loop (PI trim faded by telemetry liveness)

Each 50 Hz tick the command is:

```
    cmd = SpeedProfile.thrust_for(setpoint) + w * trim
```

- **Feed-forward** `thrust_for(setpoint)` carries the motion (and already subsumes the crossover
  handoff jump, since `velcal` measured across it).
- **PI trim** on the speed error: `err = setpoint − meas`, `trim = clamp(kp·err + ∫ki·err dt,
  ±trim_max)`. The measurement `meas` is the bidir-DShot `tele` frame's `rpm`, which is **already
  mechanical RPM** (the firmware pre-divides eRPM by pole pairs) — it is used **directly, never
  divided by `POLE_PAIRS`** (that double-division was a real 1/7 bug). We re-attach the commanded
  sign: `meas = copysign(|rpm|, setpoint)`.
- **Authority fade** `w ∈ [0,1]`: a `tele` frame is LIVE only when `|rpm| > TELE_MIN_MECH_RPM`
  (50). While a live frame is seen, `w` ramps 0→1 over `--blend-secs`; while it is stale, `w`
  ramps 1→0 and the integrator is **reset** when `w` hits 0. So above the seam (6-step, live
  telemetry) the loop closes; below it (forced sine, stale telemetry) it degrades smoothly to
  pure feed-forward. Authority is keyed **purely on tele-liveness** — never on the profile's
  crossover or capabilities — so an ESC-agnostic stock profile still runs.
- **Anti-windup** is back-calculation on **both** clamps — the `±trim_max` trim clamp and the
  outer `ESC.thrust` `--tmax` clamp — mirroring `PositionController.step`: on saturation the
  integrator is folded back to the value the delivered command implies, so it can't wind up.
- **Guards** (all on the LIVE measurement, never on config): an over-speed abort on the measured
  mech RPM, and — only when the *command's* implied regime is above the seam yet telemetry stays
  stale for `--stall-secs` — a stall abort (the ESC never reached 6-step). The stall check keys on
  the **command's** implied regime, not the nominal setpoint, because near the seam a target can
  classify `"line"` while the command still sits in the sine "gap" (a legitimate pure-FF point).

Tune with `--kp` / `--ki` / `--trim-max` / `--blend-secs`; `--kp 0 --ki 0` is pure feed-forward.
`--debug-csv` appends `tele_rpm,trim` diagnostic columns (the default CSV header is unchanged).

`VelocityController.regime(rpm)` classifies a speed as `"sine"` or `"line"` from the profile's
crossover; it is **advisory** (display, stall, down-catch), never the PI authority signal — that
is tele-liveness (see the SEAM CAVEAT in its docstring: the nominal-target classifier disagrees
with the firmware's actual regime near the seam).

### set_speed down-catch staging

If the profile has a crossover and the ESC does **not** advertise `capabilities.down_catch`,
a `set_speed` that drops **from above the seam to below it** is staged: the setpoint is first
routed through ~0 (so the rotor is re-acquired **from below**) and then promoted to the real
target once we are at ~0 / telemetry has gone stale. This is inert (a direct set) when the
profile has no crossover, the ESC advertises `down_catch`, or the move is not a line→sine descent.

### profile capabilities

`SpeedProfile` carries an optional `capabilities` block (round-trips through the YAML alongside
`crossover`). A profile with no block behaves as all-flags-false. Recognised flags: `down_catch`
(firmware can re-catch commutation crossing the seam from above) and `sine_lowspeed` (a proven
low-speed forced-sine regime). Absent/false → the conservative default (down-catch staging on).

## Safety (reused, not reimplemented)

All of `velctl`/`velcal`'s safety is the same machinery `posctl` uses:

- every command goes through `ESC.thrust` → `PosDrive.send_thrust` (the single clamp/choke);
- one command per ~20 ms tick (`_pace`) in **every** loop branch, including `velcal`'s
  settle/measure waits — well under the firmware's 500 ms spin deadman;
- temperature is polled (`tele`) every 0.5 s and the run **aborts** over `--max-temp` (the
  EFM8BB21 has no current sensing — this is the only thermal backstop at hold);
- on **every** exit path (normal, error, SIGINT/SIGTERM) the ESC is disarmed;
- `--dry-run` never opens a serial port and never writes EEPROM.

## S3 firmware is BENCH-UNTESTED — warning

All development and verification here is against the **simulator** (`SimEncEscHost`), which is
a **MODEL, not hardware truth**. The S3 sine↔BEMF crossover firmware has **never been run on
the bench**. Green sim tests do **not** validate the S3 firmware. The first real
`velcal --crossover` run will be the firmware's **first hardware crossover test** — supervise
it, start with a conservative split, and watch the telemetry temperature. The shipped default
profile `host/profiles/vel_930kv_12n14p_sim.yaml` is **sim-derived** and marked as such;
replace it with a bench `velcal` run before trusting it on hardware.
