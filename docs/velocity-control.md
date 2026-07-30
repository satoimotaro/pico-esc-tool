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
  EFM8BB21 has no current sensing). This used to be the *only* thermal backstop at hold; since B.7
  the firmware carries its own staged limiter as well (below). The host abort is still the one that
  fires first on the bench — the host defaults (60 C for `autocal`/`xover_debug`, 80 C for
  `driftsoak`/`velcal`/`posctl`) bracket the firmware band, and it covers **RAW** drive too, which
  the firmware limiter does not;
- on **every** exit path (normal, error, SIGINT/SIGTERM) the ESC is disarmed;
- `--dry-run` never opens a serial port and never writes EEPROM.

## Thermal protection (firmware, `vel::VelocityController`)

> **The EDT temperature is the ESC's own MCU, not the winding.** It is measured on the ESC board by
> the EFM8BB21 itself. There is no winding sensor on this hardware and Bluejay's EDT emits
> temperature / status / debug / demag only (no volts, no amps), so this is the *only* thermal
> observable that exists here — and it is a lagging proxy for the thing that actually burns.

Two stages, on top of each other:

| stage | when | effect |
|---|---|---|
| **derate** | `warn_temp ≤ T < max_temp` | the command **ceiling** is scaled linearly from `1.0` down to `temp_min_scale`; the loop keeps running |
| **abort** | `T ≥ max_temp` for `3` consecutive readings | `Status::ABORT_TEMP` — thrust 0, the app stops the motor and falls back to RAW |

Defaults (`vel::DEFAULT_TEMP_*`, applied to **every** Thruster in `src/main.cpp`): **60 °C** warn,
**80 °C** abort, **0.35** floor. The band is chosen to sit *entirely below* the ESC's own protection so
the two can never fight: BlueGill/Bluejay's limiter (`Modules/Scheduler.asm` + `Settings.asm`) is a
step-down PWM staircase (255→200→150→100→50, i.e. ~100/80/60/40/20 %, in ~5 °C steps) whose **lowest
selectable onset is 80 °C** (`Layouts/Base.inc`, `TEMP_LIMIT_2S`/`TEMP_LIMIT_1S`).

Why a **command ceiling** and not a scaled setpoint: a scaled setpoint hides the limit inside the
tracking error, and the disturbance observer then reads the reduction as *load* and fights it. As a
ceiling it reuses the existing outer-clamp path, and it stays visible in `cmd`. While the ceiling is
the binding constraint the controller also **freezes the integrator** (no windup against a limit it
cannot beat — the release is then bump-free), **freezes `d_hat`** and **suppresses `ABORT_STALL`**: we
are the ones withholding the command, so neither "load" nor "the ESC failed to reach 6-step" is a
sound inference.

**Fail open, but never silently.** `tstate` (in `sense`) is one of:

| `tstate` | meaning | limit |
|---|---|---|
| `none` | no temperature has *ever* been reported (pre-arm, EDT not enabled, or an adapter with no `readTemp`) | `1.00` — behaviour is **bit-identical** to the limiter being disabled; there is no limp mode |
| `ok` | fresh reading below `warn_temp` | `1.00` |
| `derate` | inside the band | `< 1.00` |
| `stale` | a temperature *was* being seen and stopped arriving for `temp_stale_secs` (2 s) | released to `1.00` — **no thermal protection**, and the app prints a notice |

`none` is the one state with no transition to announce, and the most likely real failure: EDT is only
enabled **post-arm** (`esc_session.h` sends DShot command 13 a few times after the arm window), so if
that never takes, the session runs unprotected and the only evidence is the absence of a line nobody was
waiting for. So it is timed out instead: after 5 s of closed-loop drive with no temperature at all the app
says so once, per ESC, per arm.

Temperature is read through `EscIo::readTemp(float& tempC, uint32_t& stampMs)`, which is **independent
of eRPM freshness** — forced sine has stale eRPM and is the hottest regime, so a limiter riding on
`readTele` would be blind exactly where it is needed. The adapter is **stateless**: it reports the last
decoded value and the stamp of the frame it came from, and the controller derives both edges from that —
what counts as a *new reading* (the stamp moved) and when to age one out (`temp_stale_secs` since it last
moved). There is deliberately **one** staleness policy, in one place: an adapter-side freshness window
plus a controller-side one would have summed on the first-read-after-a-gap path.
That also makes the abort debounce count **EDT frames, not control ticks** — 3 consecutive over-limit
readings, because EDT temperature is an integer °C in a frame that can garble and one bad frame must not
stop a thruster underwater. (An adapter that simply returned its held value every tick would otherwise
have collapsed the debounce to ~15 ms; the native suite pins the frame-counting behaviour at one frame
per 20 ticks.)

Runtime: `gain <i> warntemp <C>` / `gain <i> maxtemp <C>` (`maxtemp 0` disables the check entirely,
`warntemp 0` disables only the derate); `cfg show` reports both; `sense <i>` reports
`temp`/`tpeak`/`tlim`/`tstate`. These thresholds are **deliberately not persisted** by `cfg save`, so a
power cycle always restores the conservative compile-time band and loosening it is a conscious act.

**RAW drive is NOT protected.** Only the RPM submode runs `vc.step()`, so `thrust` / `throttle` (and
the sinesweep / steptrace / coastdown tools that use them) get no firmware thermal limiting at all —
the host-side `--max-temp` abort is still the only backstop there.

Known limitation: the abort stage still *stops* the motor. On a real ROV, cutting a thruster underwater
is arguably worse than holding `temp_min_scale`; the existing `ABORT_TEMP` semantics were kept
deliberately (smallest change, and the derate makes reaching the abort much less likely). Also: none of
these thresholds is bench-validated — nothing correlates 60/80 °C EDT integers to real junction
temperature on this board revision yet.

## Default profile hazard — the shipped default is SIM-DERIVED, and for the wrong motor

`host/velctl.py`'s `DEFAULT_PROFILE` is `host/profiles/vel_930kv_12n14p_sim.yaml`: a **930 KV** profile
derived from the **simulator** (`SimEncEscHost`), not from hardware. The bench now runs a **350 KV**
motor. Running velctl without an explicit `--profile` therefore feeds the controller a feed-forward curve
for a different motor, produced by a model. Always pass `--profile`; for the current bench that is
`host/profiles/f2838_350kv_bench_20260726.yaml`.

(The 29 files in `host/profiles/` are largely historical calibration runs. Only the `f2838_350kv_*` ones
describe the current bench.)

What has and has not been bench-verified, as of 2026-07-30:

- **The sine↔6-step crossover HAS now been run on hardware, extensively**, on the 350 KV bench. It hands
  off between cmd 525 and 530 on both legs with no resolvable hysteresis, and a step down through the
  seam does not stall (3/3 in both mechanical configurations). See [`measured-plant.md`](measured-plant.md).
- **The thermal limiter in this document has NEVER executed on hardware.** It is committed but unflashed —
  the running firmware still answers the old 11-field `sense` line and rejects `gain warntemp`. Green
  unit tests do not validate it. Its 60/80 °C thresholds are argued from the ESC's own assembly, not from
  any measurement tying EDT integer °C to a real winding or junction temperature.
- **RAW drive (`thrust`/`throttle`) gets no thermal limiting at all**, because only the RPM submode runs
  `vc.step()`. That includes every measurement tool (`sinesweep`, `steptrace`, `coastdown`).
