# Velocity control — velctl v1 (sensorless feed-forward)

`velctl` runs a BlueGill ESC at a target mechanical **speed** without a runtime feedback
sensor, by inverting one per-motor calibrated curve produced by `velcal`. It is ADDITIVE on
`host/pico_esc/`: it adds `pico_esc.velocity` (`SpeedProfile`, `VelocityController`,
`measure_steady_speed`) and the `velcal.py` / `velctl.py` CLIs, and extends the dry-run
simulator; it changes no wire protocol, no existing CLI, and no existing library surface.

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
command jump), and each 50 Hz tick sends `SpeedProfile.thrust_for(setpoint)` over the existing
signed `thrust` command. There is **no runtime feedback** — pure feed-forward.

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
  (`enc_rpm`) that is written to the CSV but **never feeds back into the command** in v1.

## Deferred: the eRPM-PI closed loop (documented seam)

v1 is feed-forward only. A future phase adds a runtime eRPM/encoder **PI trim** on
`(setpoint − measured)`. The seam is already in place and is a genuine no-op today:

- `VelocityController._closed_loop_trim(setpoint, meas_rpm)` returns `0.0` (v1), and the run
  loop already threads the measured speed into it — the next phase fills in the PI here.
- `VelocityController.regime(rpm)` classifies a speed as `"sine"` or `"line"` from the
  profile's crossover, so the future trim can be **regime-aware** (different gains either side
  of the handoff). It is informational in v1.

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
