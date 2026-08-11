# Azimuth closed loop — state, and what is actually blocking zero steady-state error

Session 2026-08-12, overnight. Hardware left disarmed and stationary.

## The design premise that simplifies everything

Azimuth angle only matters **while the propeller turns**. Nothing has to hold a heading at rest, so
the forced-sine regime is not needed for holding and the 6-step carrier mode is sufficient on its
own. What remains is (a) the loop while spinning and (b) getting there without throwing the heading
away.

## What works now

```
python3 azimuth.py hold --deg 0 --carrier=-600 --spinup-mode two-stage --spinup-secs 6 \
        --spinup-smax 50 --spinup-max-azimuth-rpm 500 --max-secs 25 --smax 40 \
        --max-azimuth-rpm 300 --kp 15 --ki 3.0 --allow-weak-magnet
```

- Both motors carried at |600|, never nearer than 60 counts to the 515 seam.
- The loop runs end to end and has converged to **−1.6 deg** on a good run.
- **Spin-up strategy decided by measurement**: two-stage **+7.8 deg**, ramp +182, jump −210.
  Snapping past the seam minimises time in the band where `s` has no authority — drift accrued
  there can never be recovered.

## Two bugs that no amount of tuning could have fixed

1. **`sign_b` defaulted to the `+1` placeholder, not this pod's measured `-1`.** The mixer was
   commanding the two motors in *opposite* directions — the maximum-azimuth configuration — while
   the caller believed it was carrying them together. Found in a trace reading `A=-589` with
   `B=+610`. Fixing it cut spin-up drift from −2706 deg to −289 and let the hold phase complete
   instead of aborting every time. This is exactly what `identify()` and the module docstring warn
   about, and it was still walked into; the default is now the measured value.

2. **`PositionController` hands off to a passive hold inside `tol`** — thrust 0 and the integral
   zeroed. That is right for `posctl`, whose S1 detent holds position with no power. It is wrong
   here: the pod is spinning, `s=0` means trim only, and the null drifts (+24 → +45 rpm inside one
   30 s run). The loop reached tolerance, went open-loop, drifted out, and landed anywhere — final
   errors of −1.6, −9.8, −34.4 and −233.3 deg across four *identically configured* runs. The
   integral is precisely what nulls that drift, so under a carrier the handoff is disabled.

## Why the transition is violent, and it is physics not tuning

The ESCs start the two motors on **independent ramps**, so there is always an interval with one
turning and the other not. The mismatch is therefore up to a **whole motor**, which no trim can
cancel. And — as the operator observed watching it — **in air the differential sends nearly all of
that to the azimuth**, because the pod's bearings are a far lighter load than the propeller. Hence
`--spinup-smax` (its own authority, capped so both commands stay in 6-step) and its own rate limit.

Two consequences worth carrying forward:

- `AZIMUTH_RPM_PER_S = 7.26` is an **in-air, unloaded-propeller** figure. In water the load split
  changes and it must be re-measured.
- Spin-up drift is **stochastic** — repeats gave +7.8, −74, −108, −200, −425, and once +1862 deg.
  It depends on how the two ESCs happen to start, which the host does not control.

## What is NOT the bottleneck — measured, not assumed

The host-side loop was the obvious suspect. It is not:

| | per tick | ceiling |
|---|---|---|
| `enc` only | 1.73 ms | 579 Hz |
| `enc` + 2 × `thrust` (a real tick) | **3.12 ms** | **320 Hz** |

At the 50 Hz tick that leaves 17 ms of slack, and ~3 ms of latency is about 1.5 deg of lag even at
500 deg/s. **Serial latency is not what is limiting this.** Recorded because it was about to be
asserted as the cause.

## The most likely remaining limit, and the next thing to try

**The controller is fed from a host-side 50 Hz unwrap of `enc`, which is exactly what `encv` exists
to replace.** `PositionController.update_encoder` unwraps the 12-bit raw angle between ticks and
rejects any delta above `DELTA_FAULT_FRAC` of half a revolution as a possible alias. During the
large transients seen here (peaks of several thousand deg/s) that threshold is reachable: 3000 deg/s
is 683 counts per 20 ms tick. A rejected sample does not integrate, so **position freezes exactly
when the pod is moving fastest** — which would produce precisely the observed signature of a loop
that sometimes recovers and sometimes ends hundreds of degrees out.

`encv` already solves this in firmware: the RP2040 samples the AS5600 at ~1.25 kHz, unwraps into a
signed accumulator and computes a de-aliased velocity, and it was built for this exact failure
(reverse crossover, where the host-side unwrap aliased). The tracker was confirmed running at ~1240
samples/s on this pod.

**Next step: drive the loop from `encv`'s accumulator and velocity instead of unwrapping `enc` on
the host.** That removes the aliasing rejection, gives a velocity that is not a 50 Hz difference of
a noisy angle, and should be tried before any further gain tuning.

Second candidate, if that is not enough: move the loop onto the Pico. There is headroom to do it
(320 Hz is achievable over serial, so the loop is not I/O-bound), but on-device it would run at the
tracker's rate with no round trip at all, and `main.cpp` already composes per-thruster loops.

## Standing caveats

- The magnet is at `AGC=128` (saturated) and runs on `--allow-weak-magnet` — zero margin. Tracking
  was clean over 10.8 hand-turned revolutions, but any dropped sample looks like a stalled axis.
- `_suspect()` deliberately treats `ML=1` as a warning and only `MD=0`/`MH=1` as invalid. Passing
  `not r.healthy` instead made **every** tick a fault, so the servo ran blind and reported
  `err 0.00, worst 0.00` — a perfect-looking run with no feedback at all.
- The propeller always runs in this mode. Stationary pointing is out of scope by design.

---

## 2026-08-12 later — the velocity clamp, and why pure PD had been "unstable"

Two operator suggestions, both correct, and the second one exposed a structural mistake.

**Clamp the velocity error before integrating it.** `ivel += ki*(vsp - vel)*dt` was being fed a raw
plant velocity reaching thousands of deg/s while `vsp` was bounded by `vmax`, so the term degenerated
into an integrator of *velocity* and slammed to the rail. Bounding it (`verr_clamp`) let `ki` rise
10–30x and dropped the best settled error from 6.9 deg to **2.0 deg**.

**Tune PD first, then add a little I.** Attempting this revealed why pure PD had appeared to run away
at 5300–7700 deg: **`vmax` was throttling the proportional path.** `vsp` is clamped to `vmax` and the
feedforward is `kff*vsp`, so P could never command more than `kff*vmax` counts — at `vmax=200` that
is **4.6 counts out of an `smax` of 40**. Nine tenths of the authority was reachable only through the
integral. It was never instability; it was a proportional term with almost no authority against a
null offset needing 15–30 counts.

At `vmax=2000` the same PD is bounded and stable:

| config | settled \|err\| (last 8) | mean |
|---|---|---|
| kp 8, kd 0.6, **ki 0**, vmax 2000 | 20.6 – 135.0 | **73.1** |
| kp 8, kd 1.5, ki 0, vmax 2000 | 55.9 – 302.7 | 179.3 |
| kp 8, kd 0.6, ki 0.1, vmax 2000 | 10.9 – 161.5 | 100.8 |
| kp 8, kd 0.6, ki 0.3, vmax 2000 | 60.7 – 140.2 | 99.6 |

**Adding I did not beat pure PD on the mean.** That is the finding that matters for tuning direction:
the residual is a **limit cycle, not a steady offset**, so an integrator has nothing to null. Firmware
defaults are now kp 8, kd 0.6, ki 0.1, vmax 2000, verr_clamp 50 — the best measured minimum — but the
honest reading is that further gain sweeping is exhausted.

## What to do next, and why gain sweeping is the wrong tool now

The residual is an oscillation whose frequency is set by the plant's lag, which has never been
measured — every gain choice so far rests on an *assumed* 0.1–0.2 s motor time constant.

**Identify the plant.** With both motors carried and the loop OFF, step `s` by a known amount and
record the azimuth velocity response from `encv`. That yields the real gain (counts → deg/s) and the
real time constant. From those, the stable maximum gain and the expected limit-cycle frequency are
*calculated*, and the loop can be designed instead of searched. It also gives the number needed to
decide whether the residual can be tuned away at all or is bounded by the actuator's own lag.

Second: the spin-up transient still dominates the peak error, and the two ESCs starting on
independent ramps is the cause. Coordinating those starts in firmware is the remaining lever.
