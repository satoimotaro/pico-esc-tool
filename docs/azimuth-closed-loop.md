# Azimuth closed loop — state, and what is actually blocking zero steady-state error

Session 2026-08-12, overnight. Hardware left disarmed and stationary.

## The design premise that simplifies everything

Azimuth angle only matters **while the propeller turns**. Nothing has to hold a heading at rest, so
the forced-sine regime is not needed for holding and the 6-step carrier mode is sufficient on its
own. What remains is (a) the loop while spinning and (b) getting there without throwing the heading
away.

## What works now

**THE CONVERGED CONFIGURATION — kp 3, ki 0.5, kd 0.3.** Measured 2026-08-11T17:44Z:

```
python3 azimuth.py hold --deg 0 --carrier=-600 --spinup-mode two-stage --spinup-secs 4 \
        --spinup-smax 50 --spinup-max-azimuth-rpm 500 --max-secs 18 --smax 40 \
        --max-azimuth-rpm 300 --kp 3 --ki 0.5 --kd 0.3 --allow-weak-magnet
#  -> pos +0.00  err +0.00  worst 225.53
```

`pos_deg` only ever moves in whole encoder counts (0.0879 deg), so `+0.00` means the pod came to
rest on **count 0 exactly** — and the passive-hold handoff was already disabled by then
(`ctrl.tol = -1.0`, 17:25Z), so this is the closed loop holding, not an open-loop coast. The
operator also disturbed the pod by hand and it returned. This is the only run on this pod that
has demonstrably converged.

- Both motors carried at |600|. Worst-case seam margin is **15 counts**, not 60 — see below.
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
10–30x. **The "2.0 deg" this was reported as is not a result** — it is the *minimum* of the range
`2.0 .. 416.1` (peak 416.1), against `6.9 .. 84.6` (peak 138.8) before. On any comparable reading
that run was worse. Its `vmax` was also never set: it inherited **120** from the previous sweep's
last trial, because gains persist on the device across `az off`/`az on`.

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

---

## 2026-08-12 — THE PORT COPIED THE WRONG RUN. Read this before touching a gain.

Re-read of the 8/11 overnight session log. Three findings, in order of how much they change.

### 1. The configuration that converged was discarded on a transient metric

Two host runs finished near zero. The selection between them was made on `worst`:

| gains | spin-up drift | final err | `worst` | picked? |
|---|---|---|---|---|
| **kp 3, ki 0.5, kd 0.3** | −160.7 | **+0.00** | 225.5 | no |
| kp 8, ki 1.5, kd 0.6 | −25.9 | −2.20 | **30.06** | **yes** |

`worst` is `max|target − pos|` over the hold phase, which on this pod is dominated by the excursion
right after spin-up. **It is a transient metric and says nothing about whether the loop settles.**
kp 8 / ki 1.5 was then shown to be irreproducible on repeats (−184.8, +0.88, −74.2) and *that* is
the configuration `src/apps/azimuth.h` was ported from, and every on-device sweep since is a
descendant of it. **kp 3 / ki 0.5 / kd 0.3 has never been run on the Pico.**

The damping estimate agrees with the measurement. With the plant as a velocity source of lag τ and
position as its integral, `τs² + (1+kd)s + kp = 0`, so ζ = (1+kd)/(2√(kp·τ)). At τ = 0.15 s:

| gains | ζ | outcome |
|---|---|---|
| **kp 3, kd 0.3** | **0.97** | converged |
| kp 5, kd 0.3 | 0.75 | −453.8 |
| kp 8, kd 0.6 | 0.73 | scatters −184.8 … +0.88 |
| kp 15, kd 0.6 | 0.53 | saturated limit cycle (seen in the trace) |

τ is still **assumed**, so the ζ column is an ordering, not a number. But the ordering matches every
run, and only the converged one is near critical damping.

### 2. Every "runaway" is one cause: nothing was left to supply the null bias

`s_trim = -29.8` is an estimate of a null that also drifts (+24 → +45 rpm inside one 30 s run), so
5–30 counts of bias must come from somewhere. Only two paths exist: the integral, or the
proportional path — and the P path is capped at `kff·vmax` counts.

| ki | vmax | P can supply | result |
|---|---|---|---|
| **0.5** | 200 | 4.6 counts | **converged** (host, kp 3) |
| 0.05 | 200 | 4.6 counts | runaway 10834 – 14856 |
| 0 | 200 | 4.6 counts | runaway 5299 – 7694 |
| 0 | 2000 | 40 counts | bounded, mean 73 — P carries the bias |

So **`ki = 0 RUNS AWAY` (in `azimuth.h`) is wrong**: it was not evidence that an integral is needed
for stability, only that at `vmax = 200` nothing could supply the bias. The port cut `ki` 1.5 → 0.05
while leaving `vmax = 200`, which removed both paths at once. `vmax = 2000` later restored the P
path — a different, worse way to solve the same problem. `ki` and `vmax` must be sized together.

### 3. Convergence and steady-state error were being measured as one number

`settled |err| min..max` over 6–8 samples of `az status`, polled at **0.6–1.2 s**, cannot separate
"stopped 70 deg off" from "oscillating ±70 deg", and aliases anything above ~0.4 Hz. Worse, the
later sweeps read only `err` — **`vel` was not recorded at all**, and `vel → 0` is the definition of
convergence. There is no time series from any azimuth run; `host/reports/` holds only autocal CSVs.

Report these three separately, over the last 5 s, and nothing else:

| quantity | measure | role |
|---|---|---|
| **convergence** | `rms(vel)`, `peak|vel|` | **primary** |
| limit cycle | peak-to-peak of `pos` | the failure mode |
| steady-state error | `mean(err)` | secondary — this is what I is for |

### 2026-08-14 — RAW `thrust` IS ZEROED AFTER 500 ms. Read this before any bench work.

`esc_session.h:50/301`: `SPIN_DEADMAN_MS = 500`, and `spinPoll()` sets the target to 0 once that
long has passed since the last command. A one-shot `thrust i v` from the host therefore holds for
half a second, after which the motor is commanded to zero and falls out of 6-step into forced
sine — where telemetry honestly reports **0 rpm** and `s` has almost no azimuth authority.

Every raw-drive test on 2026-08-14 slept 1.5-3 s before reading, so all of them measured a motor
that had already been zeroed. That manufactured a whole afternoon of false hardware conclusions —
"neither motor turns", "the azimuth axis does not move at any s", "an ESC has failed" — on a rig
that was fine. `bench.py check` (byte-identical), `esctool read` (`layout_revision: 226`,
`sine_cross_up: 36`) and the RPM submode (2431 rpm held) all confirmed the hardware independently.

The `az` loop and the RPM submode refresh every tick and are unaffected, so the on-device results
stand. Host-side raw driving must go through `azimuth.py:_raw_hold()`, never a bare `d.cmd`.

**Still open, and NOT answered by any of the above:** whether carrier |600| is still in 6-step
under the current (heavier) propeller at 15.2 V. Measure it with a keepalive, at low speed.

### Order of work — steps 1 and 4 are now BUILT, bench-ready

1. **`az dump`** — ✅ built. A 2048-sample ring of `(t, pos, vel, s, ivel)` recorded inside `poll()`
   at the loop's own rate (decimated, default ~200 Hz ⇒ the last ~10 s). RAM 48.9 % → 61.4 %.
   Refused while the loop runs — it prints thousands of lines and `poll()` does not run meanwhile.
   The header carries the **full configuration**, so a trace can always say what produced it.
2. **Reproduce the converged host config on the Pico**: kp 3, ki 0.5, kd 0.3, vmax 200, smax 40,
   `verr_clamp` disabled. These are the **defaults of `azimuth.py trace`** — the firmware's own
   defaults are deliberately left alone until they are re-measured.
   ```
   python3 azimuth.py trace --secs 20 --name kp3
   ```
3. **Hand-disturbance test with the trace running** — the only evidence of real stiffness.
   ```
   python3 azimuth.py trace --secs 25 --poke-at 12 --name poke
   ```
4. **Identify the plant** — ✅ built. `az step` drives an open-loop step with both motors carried
   and no feedback, self-terminating (≤ 10 s) with the rate abort still live.
   ```
   python3 azimuth.py step --step-s 20 --step-secs 3
   ```
   The host fits `v(t) = K·Δs·(1 − e^(−t/τ))` by regression over the whole rise and prints the
   **computed** kp for a chosen ζ, via ζ = (1+kd)/(2√(kp·τ)). Δs is taken from the *delivered*
   command minus the trim, because `beginStep` clamps to ±smax.
5. **Carrier −600 → −640.** With `s ∈ [-69.8, +10.2]`, motor B bottoms at **530** against the 515
   seam — 15 counts. In the 17:38Z trace every excursion is immediately preceded by `SAT` with
   B = −531. −640 leaves 55 counts. `trace`/`step` warn when the margin drops below 20.

`host/pico_esc/aztrace.py` holds the parsing, the three metrics, the seam-margin channel and the
step fit; it is pure computation and fully covered by `host/tests/test_aztrace.py` without
hardware — including a regression test that a pod *stopped* 70 deg off and a pod *swinging*
±70 deg score differently.

## Superseded: what to do next, and why gain sweeping is the wrong tool now

The residual is an oscillation whose frequency is set by the plant's lag, which has never been
measured — every gain choice so far rests on an *assumed* 0.1–0.2 s motor time constant.

**Identify the plant.** With both motors carried and the loop OFF, step `s` by a known amount and
record the azimuth velocity response from `encv`. That yields the real gain (counts → deg/s) and the
real time constant. From those, the stable maximum gain and the expected limit-cycle frequency are
*calculated*, and the loop can be designed instead of searched. It also gives the number needed to
decide whether the residual can be tuned away at all or is bounded by the actuator's own lag.

Second: the spin-up transient still dominates the peak error, and the two ESCs starting on
independent ramps is the cause. Coordinating those starts in firmware is the remaining lever.

---

## 2026-08-15 — THE ABORT WAS NEVER THE LOOP, AND THE PLANT IS NOT WHAT THE FIRMWARE THINKS

Read this before touching a gain, a mixer sign, or the rate guard. It retires several sections above.

### 1. The abort is the spin-up. No gain can reach it.

`az trace --kp 0 --ki 0 --kd 0 --rpm-trim 0 --rpm-off-max 0` gives the position loop **literally zero
authority**: both ESCs carry the identical speed target, `u` is clamped to 0, `r` is clamped to 0, and
nothing the controller computes can move the pod. **3/3 runs still aborted on the rate guard**, 1.2 s
in, at 3896 / 4109 / 4231 deg/s (`host/reports/az_ZERO{0,1,2}_20260815-0117*.csv`).

That single experiment invalidates every gain comparison in this file that was scored on run length or
on a window that included the transition. The swept parameter has no causal path to the event that
stops the run.

### 2. "Held" versus "aborted" was a coin flip on a common-mode transient

Across all 70 traces from 8/14-8/15, **every** run — the nine that lasted and the sixty-one that did
not — peaks at **3100-4500 deg/s** shortly after the seam. The guard was a single 4000 debounced over
3 ticks. A run "held" when its peak happened not to sit above 4000 for three consecutive samples.
`az_clamp` reached 4090 and survived 9.6 s; `az_H1` reached 3693 and died at 1.7 s. Nothing separates
them but luck.

**Fixed** (`src/apps/azimuth.h`): the guard is now phase-dependent — `spin_max_rate` (6000) while
`rpm_phase_` is true and for `spin_guard_secs` (0.6 s) after the pair reports up, `max_rate` (4000)
for the hold. The host loop has been phase-dependent since 8/11; the firmware had not caught up.
With it, runs reach 2.7-11.4 s instead of 1.2-2.4 s.

### 3. The trace was recording stale telemetry as measurement

`record_` wrote `ta.rpm` whenever `ta.valid`, with no freshness test — while the handover gate 170
lines above it had always required `(ms - rpmStampMs) < TELE_FRESH_MS`. `Telem.rpm` is unsigned and
KEEPS its last 6-step value after the ESC falls back to forced sine, so a trace could show two
perfectly matched motors while one had actually dropped out. **Fixed**: `freshRpm_()` writes -1 when
the number would not be a measurement. Every motor-speed reading in this file taken before 8/15
should be treated as unqualified.

### 4. `rpm_trim = -137` is wrong by a factor of three, and the true null is NOT DELIVERABLE

Measured statically with `host/azrpmcal.py` (no loop, no abort in the way — the azimuth is free to
rotate continuously, which is what makes a static measurement possible at all):

| spin_rpm | slope (deg/s per rpm of offset) | null |
|---------:|--------------------------------:|-----:|
| 800      | 5.89                            | **r = -366** |
| 1200     | 8.98                            | **r = -463** |

The slope ~6 was right. **The null is not -137, it is around -366**, and it moves with speed. The
firmware's `rpm_trim -137` therefore feeds forward less than half of what standing still requires, so
the integral rails on every run — which is what the "saturated limit cycle" in the 8/15 traces is.

Worse: at spin_rpm 800 the null needs the low motor at 800 - 366 = **434 rpm**, whose 6-step command
is 525 against a 515 seam. The ESC cannot hold it. Measured deliveries at spin_rpm 1200:
`b_tgt 500 -> 356`, `600 -> 332`, `700 -> 554`, `800 -> 700`, `900 -> 915 OK`. **The low-speed side
under-delivers by up to 270 rpm, and it is always the side the azimuth authority pushes down.**
So the pod cannot be commanded to its own null at any thrust setting tried.

### 5. RETRACTED AND REPLACED — see §7. The first version of this section claimed motor A was
40x weaker than motor B and diagnosed a slipping coupling. Both numbers and the diagnosis came from
single-variable sweeps taken while the host was polling telemetry through the integration window.
The corrected measurement is in §7.

### 6. What the encoder is NOT

Ruled out with `host/encwatch.py`, which reads AS5600 STATUS / AGC / MAG and the tracker's own sample
counter alongside the accumulator:
- **Not stray field from the motors.** AGC is 128 flat at rest, through the spin and after; MAG moves
  1342-1430 (~2 %); md/ml/mh never change.
- **Not a starved tracker.** Sample rate falls 1016 -> 626/s under load (worth knowing) but never
  stalls, and the velocity estimate decays correctly as its 20 ms window slides off an event.
- **The motion is real.** 7.9 revolutions in 2.5 s at matched targets, confirmed by three independent
  paths (trace accumulator, `encv`, and `azrpmcal`'s integration).

The azimuth genuinely advances in ~80-150 deg lurches over ~40-50 ms separated by ~80 ms of stillness
at 3-10 Hz — the "8 Hz disturbance" of the earlier sections, seen directly rather than through a
spectrum. The operator reports the axis turns **smoothly by hand**, so it is not binding or backlash.

### New tools

- `host/encwatch.py` — encoder health + net rotation + delivered speeds through a commanded spin.
- `host/azrpmcal.py` — static rpm-space plant: `--r` sweeps the offset, `--pairs` names both speeds
  directly so the balance ratio can be swept with both motors clear of the seam.

### 7. THE MEASUREMENT WAS BROKEN, AND THE CORRECTED PLANT HAS A COMMON-MODE TERM

**What was wrong with §5.** `azrpmcal` polled `tele 0` / `tele 1` in a tight loop *while integrating
the encoder*, i.e. it drove USB round trips into the same core0 loop that has to call
`As5600Tracker::poll()` every 800 us. Under that load the same operating point (A 900, B 900) read
**2507 deg/s in one sweep and 5014 in another** — and a hardware conclusion was drawn from a
difference that was the instrument. The fix is one line of discipline: sample the motors OUTSIDE the
window and leave the link silent inside it. Repeatability then goes from 2x to **+/-7 %**.

**Aliasing was the obvious suspect and it is NOT the cause.** The unwrap folds each step into
+/-2048 counts, so a stall longer than half a revolution of travel silently subtracts a whole turn —
and at 2500-9500 deg/s half a rev is only 19-72 ms. `samples()/s` cannot detect that: a mean of 700/s
is consistent with a 40 ms stall every second. New `encd` command reports the numbers that can:
**worst gap 1.9-2.4 ms, worst single step 60-272 counts against the 2048 fold — margin 7.5-34x.**
The tracker was never close. Use `encd`, never the sample rate, to certify a run.

**The corrected plant.** 22 link-silent points across two operating points, least squares:

    rate = -443  -1.06*|A|  +5.04*|B|          (deg/s;  rms residual 289 vs a ~250 noise floor)

Motor A is not decoupled — it opposes, at **1/4.8** of B's leverage, not 1/40. That now agrees with
the braked-solo test (~1.9-4x) and with the null-offset estimate (2.3-2.7x); the 40x was the outlier.

Split into the two channels the mechanism is supposed to have:

    rate = p*(|B| - |A|)  +  q*(|B| + |A|)     p = 3.05 (difference, the INTENDED channel)
                                               q = 1.99 (COMMON MODE, 65 % of p)

**There is a common-mode term two thirds the size of the intended one.** A bevel differential whose
carrier is the azimuth gives the difference alone (`ka = -kb`); this pod also turns in proportion to
TOTAL THRUST. That is why no A/B ratio holds it still below A = 4.8*B, and why every trim, gain and
guard in this file was chasing a moving null.

**It is probably not aerodynamic.** Prop reaction torque scales as w^2, so between the 1257 rpm and
1947 rpm operating points (a 1.55x speed change) `q/p` should have grown by 1.55. Measured:
**0.65 -> 0.70, i.e. flat.** Caveat: the high-speed fit is much weaker (rms 746, and the ESCs
themselves stop delivering reliably up there — the same command gave rpm_b 1654 and 2032 on two
repeats), so treat this as evidence against w^2 rather than proof. If it is geometric it will NOT go
away in water.

**THE HAND CHECK THAT SETTLES IT, with no power and no torque.** The fit predicts, per revolution of
one motor with the other held:

    turn A by 10 revolutions, B held  ->  azimuth moves  1.8 revolutions   (63.6 deg per rev of A)
    turn B by 10 revolutions, A held  ->  azimuth moves  8.4 revolutions   (302 deg per rev of B)

A ratio of **4.75**. If the hand check gives ~4.75 the asymmetry is geometric — count teeth, the two
inputs are not equally geared. If it gives ~1.0, the electrical measurement is still wrong and
nothing in this section survives. Slip is the remaining alternative, but it is now the weaker one:
a slipping element transmits a torque-limited amount, so `q/p` would move with speed, and it does not.

### 8. THE WIRING HYPOTHESIS, TESTED IN CONFIG — it is NOT reversed, and reversing it is worse

The geometry, stated correctly (operator, 2026-08-15) — with SIGNED speeds and the same physical
direction taken as positive:

    azimuth   = (w_A + w_B) / 2          propeller = (w_A - w_B) / 2

so holding the angle means **equal speed, opposite direction**, and full thrust falls out for free.
§7 was written against the wrong mental model (a pure difference-of-magnitudes) and its "common-mode
term" language should be read in that light: what it actually measured was the two motors'
contributions failing to cancel.

If the phase order on one ESC were swapped, the pair would CO-rotate in the pod frame and the azimuth
would track the sum instead — which is what the numbers looked like (matched -800 gives ~600 rpm of
azimuth where counter-rotation gives 0). So it was tested directly, in ESC config, not in code:

    python3 host/esctool.py set 1 motor_direction=Bidirectional-Reversed -r

**Result: strictly worse, and unambiguous.** With both ESCs commanded -800:

    original config (Bidirectional)          +1506 / +1567 deg/s
    ch1 Bidirectional-Reversed               -4749 / -4749 / -4760 deg/s   (0.2 % spread)

The magnitude roughly TRIPLES and the sign inverts: the two motors' contributions go from **opposing**
to **adding**. The original config already has them counter-rotating. Reverted, and
`bench.py check` is byte-identical to `20260815T035246Z-before-ch1-direction-reverse`.

**The reversal is also the best validation the plant fit has had**, because it is out of sample. From
§7's `rate = -443 -1.06*|A| +5.04*|B|`, flipping B's physical direction flips the sign of its term:

    predicted  -5352      measured  -4750          (12 % — and it predicted the SIGN CHANGE)
    predicted  d(rate)/dA < 0 with B reversed:  measured -0.61 deg/s per rpm (fit says -1.06)

So `ka` is genuinely negative and motor A genuinely opposes motor B. The structure of the fit stands.

**What is left is the imbalance, and it is the whole problem.** The two contributions oppose, as the
geometry requires, but with gains in a ratio of about **4.8 : 1**. A bevel differential's two side
gears have the SAME ratio to the carrier by construction, so equal speeds must cancel — and here they
leave ~600 rpm on the table.

**Precision caveat, stated because §5 was over-claimed once already.** Within a run, repeatability is
+/-5 %. ACROSS the ESC-restart cycle the same three operating points read 4 %, 11 % and 23 % higher
than before it, and one point (800/800, taken immediately after a restart) read 1536 where the fit
predicts 2705. The 4.8:1 structure is solid and out-of-sample validated; the individual coefficients
are not nailed to better than ~20 %, and there is real nonlinearity below ~900 rpm.

**The hand check still decides it, and it is now sharper.** Hold one motor, turn the other by hand,
count azimuth revolutions. For a plain bevel differential with one side gear HELD, the carrier turns
half a revolution per input revolution — the same number for either input, whatever the ratio between
carrier and encoder. The fit instead predicts:

    turn A 10 revolutions, B held  ->  azimuth  1.8 rev      (63.6 deg per rev of A)
    turn B 10 revolutions, A held  ->  azimuth  8.4 rev      (302  deg per rev of B)

**Equal numbers => the mechanism is fine and the fault is electrical/measurement.**
**A ~4.8x split => one input path is not transmitting what it should**, and no gain, trim, mixer sign
or ESC direction setting can compensate for it — as §8 has now demonstrated for the last of those.

### 9. RETRACTION — the ESC telemetry is not the plant's input, and §§5/7/8's conclusions rest on it

**The evidence.** Two runs, same instrument (the device trace), same steady window (t > 3 s), same
delivered speeds to within 4 rpm:

|                                | rpm_a | rpm_b | mean(rpm_b - rpm_a) | azimuth drift |
|--------------------------------|------:|------:|--------------------:|--------------:|
| `az_ki02`  (loop CLOSED)       |  1010 |   593 |              -415.5 |    **+2 deg/s** |
| `az_FIXR3` (open loop, fixed r)|  1006 |   597 |              -414.1 | **+1089 deg/s** |

And sample by sample, over ~1500 samples each, the correlation between `(rpm_b - rpm_a)` and the
measured azimuth velocity is **+0.065** and **-0.125** — i.e. none. For a rigid differential
`azimuth = (w_A + w_B)/2` holds instantaneously and exactly, so that correlation must be +/-1.

**Conclusion: `Telem.rpm` does not track the true instantaneous rotor speed.** It is polled and
averaged well below the 3-10 Hz band the pod actually moves in, so the modulation that determines the
azimuth is invisible to it. Fixing the staleness (§3) made it honest about WHEN it was measured; it
did not make it fast.

**Therefore the following are VOID and must not be carried forward:**
- §7's fit `rate = -443 -1.06*|A| +5.04*|B|`, and everything derived from it: the "4.8:1 imbalance",
  the "common-mode term", the leverage ratios, the hand-check predictions of 1.8 vs 8.4 revolutions.
- §5's "motor A is 40x weaker / the coupling slips".
- §4's "the null is not deliverable". The delivery figures were tele readings too.
- §8's *interpretation*. Its EXPERIMENT stands and is worth keeping: setting ch1
  `motor_direction=Bidirectional-Reversed` took matched -800 from +1506 to -4749 deg/s, so the
  original config already counter-rotates. Reverted; `bench.py check` byte-identical.

**There was never any evidence against the geometry.** `azimuth = (w_A + w_B)/2`,
`propeller = (w_A - w_B)/2`, hold = equal speed and opposite direction. A null exists and has been
observed: `az_ki02` drifted **+26 deg over 11 s**.

**A second instrument bug, found the same way.** `_trace_once` decided a run held with
`tr.duration >= 0.85 * secs`. But the ring is TRACE_N 2048 at decim 5, which saturates near 11.7 s —
so ANY run longer than that was reported ABORTED. Three runs that held the full 14 s with peak |vel|
2858-2901 against a 4000 guard were all filed as failures. FIXED: `_trace_once` now reads `az status`
field 1 (`on()`) BEFORE stopping the loop, and the printed duration is labelled `ring`.

**With the phase-dependent guard (§2) and the honest metric, runs hold — the first in this thread:**

    kp 0    ki 0.2   ->  3/3 held,  peak |vel| 2858-2901,  pos pk-pk 672-800 deg,  err ~-2500 deg
    kp 0.1  ki 0.2   ->  2/3 held,  and WORSE on angle (err -3518..-4172, pk-pk 1164-1805)
    kp 0.5  ki 0.3   ->  0/3 held,  peak |vel| 4019-4331 — the proportional path destabilises

That is an ordinary tuning problem, and it is where this work actually stands: the rate is held, the
ANGLE is not (kp = 0 has no proportional path, and raising kp destabilises before it helps).

**How to identify the plant from here: not with telemetry.** The AS5600 is the only sensor fast and
trustworthy enough (§6, and `encd` certifies it per run). Identify from the COMMAND — `r`, what the
loop actually sets — to the azimuth, which is exactly what `az step` was built for. `host/azrpmcal.py`
is still useful for that, but ONLY its encoder-derived rate; treat its `rpm_a`/`rpm_b` columns as
provenance, never as a regressor.

### 10. THE BINDING CONSTRAINT, measured at three thrust levels — and what shipped

**The azimuth null requires a speed ratio the 6-step seam forbids.** Encoder-derived rate (never a
telemetry regression — see §9) against COMMANDED offset `r`, link silent, 2.5 s windows, 2 repeats:

| spin_rpm | slope (deg/s per rpm) | null `r` | low motor at the null | can the ESC hold that? |
|---------:|----------------------:|---------:|----------------------:|------------------------|
|      800 |                  ~6.0 |    ~-400 |             ~400 rpm  | **no** |
|     1200 |                  6.19 |     -667 |              533 rpm  | **no** (650 cmd -> 354 delivered) |
|     1600 |                  2.57 |    -1927 |            negative   | **not even physical** |

Raising thrust does not help: the null moves out FASTER than the authority grows. Lowering thrust
does not help either: the ratio is scale-free, so the low motor lands on the seam at every level.
The measured low-side delivery floor is ~800 rpm (commanded -> delivered: 1200->1168, 1000->1030,
850->835, 700->550, 650->354).

Confirmed end to end: at spin_rpm 800 with the loop given every count of authority the seam allows
(`--rpm-floor 400 --rpm-off-max 400 --rpm-trim -300`), 3/3 runs still drift **+550 deg/s** with `u`
on its rail and die on the 4000 guard at 5.1 s.

**So the loop is not mis-tuned; it is out of actuator.** The lever that would change this is the
LOW-SPEED VELOCITY HOLD, not the azimuth gains: every rpm by which the low motor's floor comes down
is an rpm of azimuth authority. That is a `lib/vel_control` problem (FF curve + sense gate + PI at
the bottom of the range), and it is the next piece of real work. The other lever is water — the
azimuth axis in air has almost no damping, so any residual torque integrates straight into speed.

**What shipped this session** (all built, flashed, 401 host tests pass):

*Firmware (`src/apps/azimuth.h`, `as5600.h`, `esc_tool_app.h`)*
1. **Phase-dependent rate guard** — `spin_max_rate` 6000 while crossing the seam + `spin_guard_secs`
   0.6 s after, then `max_rate` 4000. Crossing the seam is not a fault (§2).
2. **`wrap_err`** — the position error is folded to +/-180 deg. The azimuth is a continuously
   rotating axis; servoing the unwrapped accumulator made any kp > 0 try to unwind whole
   revolutions. This is what took `err_mean` from -2500 deg to -11 deg.
3. **`rpm_floor` + `rLimit_()`** — azimuth authority is now `min(rpm_offset_max, spin_rpm -
   rpm_floor)`, so it scales with thrust instead of being a constant that lands the low motor on
   the seam.
4. **`cross_zero_offset`** — the rpm path now delivers r = 0 until both motors report up, so the
   pair crosses the seam on identical commands. The raw path has had this rule since 8/14; the rpm
   path never got it, and `rpm_trim` was tearing the pair apart from the first tick.
5. **`freshRpm_`** — the trace no longer records stale telemetry as measurement.
6. **`encd`** — worst inter-sample gap, worst unwrapped step, and the margin to the 2048 fold.
   The number that certifies the encoder per run. Never use samples/s for this.

*Host*
7. `_trace_once` asks the DEVICE (`az status`) whether the loop stopped itself, instead of inferring
   it from a trace ring that saturates at ~11.7 s and called three 14 s holds failures.
8. `metrics()` gained **`err_abs_mean`** and **`net_rot`**, printed first. On a rotating axis the
   mean of a wrapped error tends to zero however fast the pod spins — two runs reported
   `err_mean` -11 and -31 deg while turning +1997 and +1155 deg net at mean|err| 82-85 (a free spin
   is 90). Judge a hold on those two, never on `err_mean`.
9. **`host/encwatch.py`** — encoder health + net rotation + delivered speeds through a commanded spin.
10. **`host/azrpmcal.py`** — static plant identification. `--r` sweeps the offset, `--pairs` names
    both speeds directly. **The host must stay silent through the integration window** — polling
    telemetry inside it loads core0 and was worth a factor of two in the answer (§7).

---

## 11. RETRACTED — the camera did not show what I said it showed

This section claimed the AS5600 was on the propeller shaft, because a 2 s burst showed the pod
apparently motionless while the encoder accumulated 26.4 revolutions. **That inference is wrong on
three counts and the conclusion is withdrawn.**

1. **Aliasing.** The capture ran at 20.5 fps against a claimed 26.4 rev/s — **1.29 revolutions per
   frame**. A body sampled just past one turn per frame looks nearly still. The camera itself is
   fine (41 frames in 2.0 s, zero duplicates, 48-52 ms intervals); the sampling rate was never
   compared against the thing being measured. Exactly the mistake §7 made with telemetry.
2. **Rotational symmetry.** The pod is a cylindrical duct seen near end-on. Rotation about its own
   axis does not change its silhouette at all, so "the frames look identical" is not evidence of
   anything.
3. **It contradicts two established facts.** Hand-turning the azimuth axis has been verified to
   track the encoder cleanly (6.30 turns, on record), and the host loop held the angle to a couple
   of degrees on 8/11 — which is impossible if `enc` were reading the propeller.

The 0.96 encoder/motor ratio at matched speeds is still a real, unexplained measurement. It is NOT
licence to conclude where the sensor is mounted. **The premise to check first is whether the two
motors, both commanded -800, are actually counter-rotating** — that has never been verified, and if
they are co-rotating then azimuth = (wA + wB)/2 = the motor speed, which is what the 0.96 says.

**Lesson for anything below this line:** before a measurement is allowed to overturn the geometry,
check that the instrument's sample rate exceeds the motion, and that the observable can distinguish
the two hypotheses at all. Three instruments have now failed that test in one session — telemetry
(§9), the trace-length completion metric (§9), and this camera burst.

**What still stands, and is still needed** — none of it depended on the encoder being an azimuth
angle, and all of it will be required the moment it is:
§9's telemetry finding, and the six firmware/host fixes in §10 (phase-dependent + sustained-rotation
rate guard, `wrap_err`, `rpm_floor`/`rLimit_()`, `cross_zero_offset`, `freshRpm_`, `encd`, the
device-asked completion test, `err_abs_mean`/`net_rot`). Plus, added this session:
- **`thrust_rpm` / `thrust_secs`** — cross the seam at a survivable speed, capture, THEN ramp thrust,
  with the authority growing as it rises. The crossing transient scales with thrust (2735-2901 deg/s
  at spin_rpm 800 vs 5983-6351 at 2000), so asking for the operating thrust at t=0 asks the crossing
  to happen there.
- **The guard is now about SUSTAINED rotation, not a peak.** `over_ >= 3` at the ~1 kHz tick meant
  three MILLISECONDS above threshold ended a run. Now: a leaky time-above-`max_rate` budget
  (`over_secs`, default 0.5 s) plus a separate instantaneous `hard_rate` ceiling. A transient is
  allowed; turning fast for a while is not.
- **`host/podcam.py`** — per-pixel temporal variance and encoder-tagged frames. Use it whenever an
  electrical measurement says something the geometry forbids.

---

## 12. THE MECHANISM, from the CAD — and what the firmware has actually been commanding

Source: `/srv/share/azimuth_docs/azimuth_module.step` + the section screenshot, plus the operator's
description ("デフの上側と下側をそれぞれモータで回している。等速ならデフごと回って、差があれば
上と下が廻ってそれによってプロペラが廻る"). All three agree, and so does the bench data.

### What it is

A bevel differential, driven from BELOW by two coaxial vertical shafts:

- Two motors sit **side by side** in the base, each driving one of two **coaxial vertical shafts**
  (one hollow around the other) through **one external spur mesh each**.
- Each coaxial shaft ends in a **bevel side gear** — one facing down from above, one facing up from
  below.
- Between them, on a **horizontal cross axle**, sit two bevel **planet** gears. **Those planets ARE
  THE PROPELLER SHAFTS** — they carry the two props, which is why the part list has
  `3D_propeller5_3` twice with one marked `(ミラー)`: contra-rotation is structural, not commanded.
- The carrier — the housing holding that cross axle, plus the duct and both props — is the
  **AZIMUTH** body, rotating about the vertical axis.
- A **central shaft runs from the carrier down through both hollow coaxial shafts** to a magnet
  holder just above the base plate. That is the AS5600 pickoff: it reads the carrier against the
  fixed base. **The encoder is on the azimuth axis.** (§11 was right to retract.)

### The kinematics — and they are the OPPOSITE of what the code assumes

With `wU`, `wL` the two coaxial shafts (motor space, same sense positive):

    azimuth   (carrier orbit)  =  (wU + wL) / 2        <- the COMMON mode
    propeller (planet spin)    =  (wU - wL) / 2        <- the DIFFERENTIAL mode

So **holding the angle requires the two motors to run in OPPOSITE directions**, and the thrust is
exactly that anti-symmetric part. Equal-and-same-direction is maximum azimuth rate and ZERO thrust.

`applyRpm_` sends:

    a = sgn*spin_rpm + r        common = sgn*spin_rpm   -> AZIMUTH
    b = sgn*spin_rpm - r        anti   = r              -> PROPELLER

**The roles are swapped.** `spin_rpm`, meant as thrust, is the azimuth command; `r`, meant as the
azimuth command, is the thrust. (`apply_` has the same shape: common = the carrier `c`, anti = `s`.)
This is only correct if motor B is physically mirrored so that `wL = -b_cmd` — the assumption behind
`sign_b = -1`. The CAD shows two like motors side by side, each with a single external spur mesh,
i.e. **not** mirrored.

### The measurement says the same thing

- Encoder rate / motor rate at MATCHED commands: **0.964 / 0.965 / 0.954** at 384 / 806 / 1209 rpm.
  Predicted by the above: **1.00**. The pod was doing exactly what it was told.
- No value of `r` ever nulled the azimuth, at any thrust, and the fitted "nulls" (-366, -667, -1927)
  were nonsense that moved with thrust — because `r` does not reach the azimuth at all.
- Every "runaway" scaled with `spin_rpm`, and the rate guard was measuring commanded rotation.
- The props should have been nearly still throughout (thrust = r ~ 137 rpm).

### The one thing that does NOT fit, and the test for it

Setting ch1 `motor_direction=Bidirectional-Reversed` should make same-signed commands drive the two
coaxial shafts oppositely, i.e. azimuth -> 0 and the props -> full speed. Measured instead:
**-4749 / -4749 / -4760 deg/s** (0.2 % spread) — same magnitude as before, opposite sign. Either the
3D-mode direction bit did not change the physical rotation the velocity loop delivers, or the
pairing is the other way round and something else explains the 0.96.

**Test, no instruments needed** (do NOT run the bench unattended):

    python3 host/encwatch.py --rpm -800 --rpm-b 800 --secs 4 --name opposite

and simply LOOK. Opposite commands should give **props spinning hard, pod still**. If instead the pod
spins and the props are lazy, the pairing is inverted and §12's fix must be applied the other way.

### The fix, prepared but NOT enabled

`az gain mixswap 1` (host `--mix-swap`) exchanges the two channels in `applyRpm_`:

    a = +thrust + u_az          common = u_az     -> AZIMUTH (zero holds the angle)
    b = -thrust + u_az          anti   = thrust   -> PROPELLER

It defaults to OFF so nothing changes silently. Note it commands one ESC in reverse, and the reverse
crossover on this stack is uncharacterised — the alternative is to do the same job in ESC config
(one channel `Bidirectional-Reversed`), which keeps both commands in the proven forward half. That
is the experiment above.

---

## 13. THE NULL IS A SPEED RATIO OF ~4.3, AND IT WAS JUST BEYOND EVERY SWEEP

§12's "the channels are swapped" is **wrong as stated**, and `mix_swap` is not the answer. Here is
what the pod actually does, from the cleanest experiment in this file.

### Two commands, two unknowns

At LOW thrust, with both motors delivering an identical 310 rpm (so nothing is under-delivered and
no ratio is being inferred from a fit):

    same sign      A -400, B -400   ->  +1248 deg/s          (net over 4 s)
    opposite sign  A -400, B +400   ->  -1995 / -2005 deg/s  (two runs, 0.5 % apart)

Solving directly, with signed motor speeds:

    azimuth rate = -1.21 * wA  +  5.24 * wB      (deg/s per rpm)

The independent 22-point least-squares fit from earlier the same day gave **-1.06 / +5.04**. Two
completely different experiments, the same answer.

**So the azimuth is neither the pure sum nor the pure difference.** Neither §12's model (null at
opposite signs) nor the firmware's (null at equal magnitudes) is right: opposite signs gave -2000,
equal magnitudes gave +1248. The weighting is **~4.3 : 1**.

**And that is geometry, not a fault.** The two coaxial shafts have different diameters — one runs
inside the other — so the spur gears driving them are necessarily different, and their reductions
differ by construction. There is nothing to fix in the mechanism.

### Where the null actually is

    hold  <=>  wA = 4.3 * wB,  SAME sign      i.e. a speed RATIO, not an rpm offset

Static confirmation, ratio swept 2.5 -> 5.5 at low thrust:

    delivered (1012, 239) ratio 4.24  ->   144 deg/s     <- the closest to a null ever measured here
    delivered (1419, 323) ratio 4.39  ->   355
    delivered (1830, 338) ratio 5.41  ->   590
    delivered (2253, 454) ratio 4.96  ->  1025

Every earlier sweep peaked at **ratio 3.5** and every `rpm_trim` ever tried (-137 / -300 / -450 /
-600) corresponds to a ratio of **1.5 / 2.2 / 3.0 / 3.7**. The null was always just past the end.

### Two structural fixes this exposed

1. **The trim was inside the authority clamp.** `constrain(u*rpm_per_count + rpm_trim, +/-lim)` made
   the feedforward and the authority share one budget, so setting `rpm_offset_max` to "how far the
   loop may move" silently clipped the trim too. With the null at r = -434 and
   `rpm_offset_max` 150, the delivered r could never pass -150 and the pod turned 9900 deg in 12 s.
   Now `r = rpm_trim + constrain(u*rpm_per_count, +/-lim)` — the name means what it says.
2. **The null scales with thrust, so a fixed `rpm_trim` is right at exactly one thrust setting** and
   wrong the instant `thrust_rpm` ramps. New `null_ratio` (host `--null-ratio`) states it as the
   ratio and derives `r_trim = thrust * (1 - ratio) / (1 + ratio)` live. 0 keeps the fixed trim.

### Where it got to

Closed loop, spin_rpm 700, at the measured null, over ~12.4 s:

| configuration                            | net rotation (deg) |
|------------------------------------------|--------------------|
| any earlier trim (ratio 1.5-3.7)          | 9860 - 10454 |
| trim -434, clamp too small (r clipped)    | 9860 - 10454 |
| trim -434, clamp large enough             | **256 - 970** |
| kp 0.3 ki 0.5, off_max 200                | **-70 - 626** (median 167 = 13 deg/s) |

**A 10-60x reduction, and one run drifted -70 deg in 12 s.** The azimuth is under control for the
first time in this file.

### What is NOT yet established

`mean|err|` sits at 60-110 deg against 90 for a free spin, so the pod is **holding its RATE, not a
HEADING** — it wanders over most of the circle while barely drifting. And the run-to-run spread is
still large: the same configuration gave a median net rotation of 167 deg in one session and 1420 in
the next. **Three repeats cannot separate the gain configurations tried** (kp 0.3/ki 0.5 looked best,
but kp 0.5/ki 0.2 and the `null_ratio` re-run overlap it). Do not pick a gain on this data — the
structural result is what is solid.

Next: more repeats per configuration before any gain claim, then attack the heading (not just the
rate). `--rpm-off-max 300` and `--vel-lpf 1.0` both made it worse and are not the direction.

---

## 14. THE DRIFT IS SOLVED. The heading holds sometimes. What actually mattered.

All numbers below are **6 repeats per configuration**, spin_rpm 700, `--null-ratio 4.24`,
`--rpm-off-max 200`, 12 s windows. Net rotation is over the whole window; a free spin gives
`mean|err|` ~90 deg.

### Gain was never the variable

    kp   0.3 / 0.6 / 1.5 / 3.0  (ki 0.5, verr_clamp 0)
      -> net rotation median  946 / 802 / 943 / 1030 deg,  mean|err| 91 / 93 / 94 / 96

**A tenfold change in kp does nothing.** The loop is at its rail 17-25 % of the time and
`mean|ivel|` is 16-18 out of a 27.5 rail — **the velocity integrator was eating the entire authority
budget**, leaving the proportional path nothing to work with. And it was eating it on noise: `ivel`
integrates `vsp - velf`, `velf` was unfiltered, and the disturbance is rms 1100 deg/s. The integral
was random-walking against the disturbance instead of trimming a bias.

### What mattered: bounding the integral's input

`verr_clamp` exists for exactly this and the host default was **0 (disabled)**, so every run in this
file until now had it off.

| verr_clamp | net rotation, median (spread) |
|-----------:|-------------------------------|
| 0          | 802 (150 .. 1222) |
| 50         | 386 (-68 .. 808) |
| **15**     | **-15.4 (-144 .. +48)** |

**A 60x reduction, and the sign now straddles zero** instead of always drifting one way. Median net
drift is **1.3 deg/s over 12 s**. The drift problem is solved.

With the integral tamed, kp still did not help — 0.6 / 1.5 / 3.0 gave -15 / 272 / 760 deg. Raising it
just pushes `vsp` outside the clamp again.

### The heading needed a term that did not exist

Drift solved, `mean|err|` sat at 105-121 against 90 for a free spin: the pod had **stopped, in the
wrong place**. `ki` integrates the VELOCITY error and removes a rate bias; **nothing in this loop ever
integrated POSITION**, so a standing heading offset had no path to zero. Added `ki_pos` — the outer
loop becomes a PI producing the velocity setpoint, which is the ordinary cascade — with
back-calculation against the same `vmax` clamp and a hold while `rpm_phase_` is still crossing.

| ki_pos | net rotation median | mean\|err\| median (best run) |
|-------:|--------------------:|------------------------------:|
| 0      | -15.4 | 114.5 (78.6) |
| 0.05   | 413.5 | 99.5 (81.6) |
| **0.2**| **16.9** | 109.5 (**19.7**) |
| 0.5    | -24.4 | 115.1 (37.2)  *(1/6 aborted)* |
| 1.0    | 788.5 | 114.4 (51.9) |

At `ki_pos 0.2` one run held to **mean|err| 19.7 deg** — the first genuine heading hold in this file —
while the drift stayed near zero. But the median is still ~110, so **it holds the heading sometimes
and not usually**. Run-to-run variability is now the dominant unknown, not the gains.

### Best configuration on record

    --spin-rpm 700 --null-ratio 4.24 --rpm-off-max 200 --rpm-floor 0 \
    --kp 0.6 --ki 0.5 --kd 0 --vel-lpf 0 --verr-clamp 15 --ki-pos 0.2

### What is still not established

- **`null_ratio` thrust-invariance.** Swept at ~2.5x the thrust (B ~700), the null is not resolvable:
  the low motor's delivery scatters +/-30 % (target 700 -> 574-980) and the azimuth rate scatters with
  it. Low thrust is where this is measurable AND controllable. Do not enable `thrust_rpm` ramping
  until the ratio is confirmed at a second thrust.
- **Why some runs hold the heading and most do not.** That is the next question, and it is about
  repeatability of the spin-up state, not about a gain.
- `--rpm-off-max 300` and `--vel-lpf 1.0` were both clearly worse. Not the direction.

---

## 15. WHY THE HEADING CONVERGES ONLY SOMETIMES — the null is razor-thin, and motor B scatters

Every number below is **6 repeats**, 12 s windows, `net rotation` in deg over the window,
`mean|err|` against 90 for a free spin.

### The full sweep, one axis at a time from the best point

| axis | values | net rotation, median |
|------|--------|----------------------|
| `null_ratio` | 4.00 / **4.24** / 4.45 | 937 / **17** / 2582 |
| `spin_rpm`   | **700** / 900 (rho 4.24) / 900 (rho 4.8) / 2000 | **17** / 4237 / 1641 / 11163 |
| `verr_clamp` | 0 / **15** / 50 | 802 / **-15** / 386 |
| `ki_pos`     | 0 / 0.05 / **0.2** / 0.5 / 1.0 | -15 / 414 / **17** / -24 / 789 |
| `ki`         | **0.5** / 1.0 / 2.0 | **17** / 1436 / 1342 |
| `kp`         | 0.3 / **0.6** / 1.5 / 3.0 | 946 / 802 / 943 / 1030 *(verr_clamp 0)* |
| `rpm_off_max`| **200** / 300 | 167 / 1299 |
| `vel_lpf`    | **0** / 1.0 | 167 / 2061 |

### The null is razor-thin

**+/-0.2 in `null_ratio` costs 50-150x.** 4.00 -> 937, 4.24 -> 17, 4.45 -> 2582.

At `null_ratio` 4.24 and spin_rpm 700 the low motor sits at **|b| = 267 rpm**, and its delivery
scatters ~+/-10 % run to run. That moves the EFFECTIVE ratio by ~8 %, i.e. 4.24 -> 3.9-4.6 — straight
off the edge of a null that is sharp at the 0.2 level. **That is the whole story of the run-to-run
variability**, and the per-run data says so directly (the six `ki_pos 0.2` runs, sorted by heading
error):

    mean|err|  34.5  63.7  89.4 102.9 136.9 153.3
    delivered ratio  4.16  4.40  4.65  4.69  4.53  4.39
    delivered rpm_b   287   272   256   258   264   262

The two runs that held the heading are the two with the most rpm_b. Nothing else separates them —
the loop is never saturated any more (`rail% = 0` in all six, `mean|ivel|` 3.5-8.5 of a 27.5 rail).

### And the ratio grows with thrust, faster than the low motor can follow

Two-point measurement (same-sign vs opposite-sign at matched magnitude), which gives both
coefficients directly:

| matched speed | kA | kB | null ratio |
|--------------:|---:|---:|-----------:|
| 310 rpm | -1.21 | +5.24 | **4.32** |
| 780 rpm | -0.74 | +5.37 | **7.22** |

`kB` is stable; **`kA` weakens with speed**, so the required ratio climbs. Meanwhile the low motor's
deliverable floor climbs too: at thrust 2000, commanding `b` 444 delivers 623, so the achievable
ratio caps at **~5.2** while the null needs **~7.2**. Swept rho 3 -> 8 there, the rate never crosses
zero (2099-4684 deg/s). Ramping thrust 700 -> 2000 with `null_ratio` fixed destroyed the hold
outright (net rotation 10215-11689, mean|err| 90 = free).

**So low thrust is not a preference, it is the only regime where the null is reachable.** Do not
enable `thrust_rpm` ramping.

### Where it stands

    --spin-rpm 700 --null-ratio 4.24 --rpm-off-max 200 --rpm-floor 0 \
    --kp 0.6 --ki 0.5 --kd 0 --vel-lpf 0 --verr-clamp 15 --ki-pos 0.2

- **Drift: solved.** Net rotation median **-15 to +17 deg over 12 s** (~1 deg/s), from ~10000.
- **Heading: intermittent.** Best runs `mean|err|` **19.7 / 34.5 deg**; median ~110.

### The bottleneck, now specific

Not a gain, not the mechanism, not the encoder: **motor B's delivery repeatability at ~270 rpm.**
It is the only variable that tracks the heading error, and the null's sharpness converts its ~10 %
scatter into a total loss of convergence. Two ways at it:

1. Make the delivery repeatable at ~270 rpm (`lib/vel_control`: FF curve, sense gate, PI at the
   bottom of the range). This is no longer a vague "low speed is hard" — it is one operating point
   with a measured requirement.
2. Close the ratio loop on the DELIVERED speeds rather than the commanded ones. eRPM is honest in
   6-step (the operator's guidance), and at 270 rpm mech / 7 pole pairs that is 1890 eRPM — worth
   checking whether it is still trustworthy there before relying on it.

---

## 16. THE NULL DRIFTS. A FIXED FEEDFORWARD CANNOT WORK — it has to be identified online.

### What pico-esc-tool already provides (reviewed before adding anything)

The INNER loops are not the problem and already carry the machinery this needs:
`vel::VelocityController` has the calibrated FF curve (`SpeedProfile`), a **disturbance observer**
(`dob`, on this bench at **0.80** — "rejects load steps far faster than the PI"), the sense gate,
`trim_max` / `dob_max` under one `fb_max` budget, and `track_min_frac` so a telemetry floor is not
mistaken for authority. Nothing there needs work for the azimuth. Everything below is the OUTER loop.

### Priority 1, transient over-rotation: the obvious FF fix does not work, and why

Spinning up AT the null ratio (rather than matched) is the right idea — a matched pair is maximum
azimuth rate, so the current spin-up asks the pod to spin before the loop exists. It was implemented
and **it fails on the handover gate**: at the null the low motor is asked for ~270 rpm, and
`Telem.rpm` reads a constant ~182 below the seam, so any threshold low enough for it to pass is also
passed by a motor that is not turning. The handover fires early, the loop starts from a bad state,
and net rotation went **17 -> 2868-3506 deg**. Reverted, with the reason recorded in the source so it
is not re-attempted.

What survives: **`trim_secs`** — the null feedforward now fades in over 0.5 s after the handover
instead of stepping (the pair splitting ~4:1 in one tick is itself a disturbance).

### The finding that reframes priorities 2-4: THE BENCH IS NOT STATIONARY

The identical configuration, 6 repeats each, measured across one day:

    net rotation median:   17   ...   1235-1568   ...   3096      (deg / 12 s)

At `trim_secs 0` the code is logically identical to the run that gave 17, and it gave 3096. **The
earlier result was not reproducible.** And the reason is visible in the telemetry:

    ESC 0: 18 C      ESC 1: 34 C

**A 16 C asymmetry, on the motor that dominates the azimuth** (kB +5.24 vs kA -1.21). Temperature
moves delivered speed for a given command, the null is sharp at the 0.2-in-ratio level (+/-0.2 costs
50-150x), and B alone carries ~4x the leverage. **Thermal drift on its own is enough to destroy the
hold**, and the bench had been running for hours.

### The structural consequence

**`null_ratio` cannot be a constant, and neither can any fixed feedforward.** A calibration measured
cold is wrong warm; a calibration measured on one run is wrong on the next. That is why:

- gains swept out flat (the operating point moves underneath them),
- the best configuration is not reproducible session to session,
- and it is hopeless to chase "the" null by static sweeps, which is what most of this file did.

**The loop's integral is already an online null estimator** — that is what `ivel` and `ki_pos` are
doing. The problem is that it has to track a razor-sharp, drifting null with an authority budget it
shares with the disturbance. So the direction for all four of the operator's priorities
(transient FF, practical range, setpoint tracking, stiffness) is the same:

1. **Identify the null online and explicitly**, rather than trimming toward it implicitly. The
   azimuth rate is measured at 1 kHz and `r` is known; `d(rate)/dr` is ~6.1 deg/s per rpm and stable.
   An explicit estimator of the null offset, updated slowly, separates "where is the null now" from
   "reject this disturbance" — which is exactly the split the current single integrator cannot make.
2. **Then** the FF for the spin-up transient becomes the current null estimate rather than a
   compile-time constant, and setpoint tracking and stiffness can be tuned against a stationary
   plant instead of a moving one.
3. Log ESC temperature with every azimuth run. It is already in `Telem` and was never recorded.

### Do not repeat

- Static ratio sweeps to find "the" null. It moves.
- Comparing configurations measured minutes apart without a null re-identification between them.
- Spinning up already split (§16 above), or relative handover gates (the 182 rpm telemetry floor).

---

## 17. THE FIRST VALID PLANT ID — and it says motor A is not connected to the azimuth

**2026-08-15, after the method reset.** Every earlier plant number in this file was taken either with
one motor braked (void: with one input braked the carrier/planet split is set by FRICTION, not by the
constraint) or from a same-sign/opposite-sign pair that moves BOTH channels at once (void: it
confounds the kinematic term with prop reaction torque against an axis that is free in air).

This section is the experiment §2e of the checkpoint asked for, and it is the first plant
identification in this file that obeys the method rule.

### The experiment

`host/azsumdiff.py`. **Both motors driven at every single point**, one channel varied at a time:

- `--pairs -1000 -1400 ... -1000 1400` — A held at -1000, B swept -1400 -> +1400 through zero
- `--pairs -1400 -1000 ... 1400 -1000` — B held at -1000, A swept -1400 -> +1400 through zero

Discipline: link SILENT through the 2 s integration window; `encd` certifies every point against the
2048-count unwrap fold (margins 10-20, i.e. 10-20x); `sense` records each ESC's own `cmd`/`why` so
"delivered" is the ESC's opinion, not just a magnitude comparison; ESC tempC logged at every point
(A 14-17 C, B 30-32 C — the §16 asymmetry is still there). 14/16 points delivered to within 22 rpm
with `why=ok` on both channels.

**Supply: 11.2 V.** The first run of the session was taken with the config still on `v=15.20` while
the bench was at 11.2; re-pointing both channels (`motor i 350 7 11.20`, `cfg save`) moved the answer
by 2 % (-6487 -> -6362 deg/s), so the FF supply constant was NOT what any of this turned on. Recorded
because it was checked, not because it mattered.

**`settle` matters and 1.2 s was not enough.** At `settle 1.2` motor B read 1360-1486 rpm against a
1000 rpm target and looked like a broken reverse crossover. At `settle 2.0` the same points deliver
to within 22 rpm. **There is no reverse-delivery fault**: B tracks +800/+1000/+1200/+1400 as well as
it tracks the negative side, and `cmd` is symmetric (-579..-651 forward, +604..+683 reverse).

### The result

    rate = +27  +0.455*A  -5.430*B        deg/s,  rms residual 325 (noise floor ~250), 14 points

in rpm of azimuth per rpm of motor, and against what the mechanism requires:

    measured    kA = +0.076        kB = -0.905
    1:1 bevel   kA = +-0.500       kB = +-0.500

**The azimuth follows motor B almost exactly 1:1, and motor A does essentially nothing to it —
a ratio of 11.9 : 1.** A was swept across 2800 rpm *including a full sign reversal* and the azimuth
rate moved from 4665 to 6124 deg/s: it barely noticed.

This is NOT the §13 "4.3:1 leverage" resurrected. That number came from voided experiments and had a
different structure (both motors contributing, in a ratio). This says something stronger and cleaner:
**one input reaches the carrier at ~1:1 and the other does not reach it at all.**

### What that means mechanically

Both motors held their commanded speeds (`why=ok`, within 22 rpm), so both INPUTS were imposed. For a
rigid bevel differential, imposing both inputs determines the carrier exactly: `(wA + wB)/2`. It
measurably does not. So something between motor A and its side gear is not rigid.

Solving for what A's shaft must actually be doing, with the physical sense `w = -command`:

    carrier = 0.905*wB - 0.076*wA        vs the required  0.5*wA + 0.5*wB
    => A's SIDE GEAR is turning at ~0.81*wB, in the same direction as B's, almost regardless of
       what motor A is told to do.

That is the signature of **motor A's shaft spinning nearly free and being dragged by the rest of the
mechanism** — a slipping coupling / grub screw / disengaged spur mesh on the A path. It also explains,
in one stroke, why every "asymmetric leverage" number in this file (40:1, 4.8:1, 4.3:1, now 11.9:1)
came out different: slip is torque- and speed-dependent, so the apparent ratio moves with the
operating point. The operator has stated the gearing is 1:1 on both sides, so an 11.9:1 ratio is not
geometry.

### The consequence for control, and it is a hard one

With `kA` ~ 0 the azimuth has exactly one actuator, and holding still means holding B still:

    rate = 0   =>   B = 5 + 0.084*A  rpm      (A -1400 -> B -112;  A +1400 -> B +122)

**The null needs motor B somewhere between -120 and +122 rpm.** The 6-step seam at 11.2 V is around
590 rpm. So the hold condition is roughly 5x below anything this ESC can deliver — which is exactly
what §10 measured ("the null requires a speed the seam forbids") and §15 ("the null is razor-thin"),
now with the reason attached rather than as a ratio that mysteriously moved.

**So: closed-loop azimuth hold is not reachable by tuning, and was never going to be.** Not a gain,
not `null_ratio`, not `mix_swap`, not an online null estimator, not `verr_clamp`. Every one of those
was trying to place an operating point that the actuator cannot reach, on a plant with one of its two
actuators disconnected. Stop tuning until the drivetrain is fixed.

### The next action is a hand check, with no power and no instruments

The electrical work has taken this as far as it goes. Three checks, in order, everything unpowered:

1. **Continuity of the A path.** Turn motor A's rotor by hand and watch A's coaxial shaft. Nothing
   moving = the coupling slips, and that is the whole finding. (This is a solo test, but it is a
   binary continuity check on one drive path, not a measurement of the carrier/planet split, so the
   method rule does not apply to it.)
2. **Both inputs imposed, same direction, equal amounts** (e.g. 10 turns each, by hand). A healthy
   1:1 differential turns the carrier 10 turns and the props not at all.
3. **Both inputs imposed, opposite directions, equal amounts.** The carrier should stand still and
   the props should spin.

2 and 3 are fully constrained kinematically — friction cannot redistribute anything when both inputs
are held by hand — so they test the geometry itself, and they are the check §7 and §8 kept proposing
and never got.

**If the A path turns out to be sound**, then `kA` ~ 0 is real geometry, the pod has one azimuth
actuator by construction, and the control problem changes shape entirely: hold requires B near zero,
so the work moves to `lib/vel_control` at 50-250 rpm (§10's low-speed hold), and thrust has to come
from somewhere other than the difference.

### New tool

`host/azsumdiff.py` — the azimuth plant with both motors driven throughout. `--sum`/`--diff` sweep one
channel at a time; `--pairs` names both signed targets; `--signs M` runs the four sign combinations at
magnitude M and scores them against every hypothesis that a single point cannot separate (`Telem.rpm`
is unsigned, so one point can never say whether a motor commanded positive actually reversed).
Records `sense` cmd/why and ESC tempC per point.

---

## 18. THE COUPLING WAS SLIPPING. A GRUB SCREW FIXED IT, AND THE PLANT IS NOW TEXTBOOK.

**2026-08-16.** The operator's own hypothesis — "it probably slips at speed" — was right, added a grub
screw, and the same two sweeps at the same operating points now give:

| | kA | kB | \|kB/kA\| | rms residual |
|---|---:|---:|---:|---:|
| §17, before | +0.076 | -0.905 | 11.9 | 325 deg/s |
| **after** | **+0.493** | **-0.501** | **1.0** | **27 deg/s** |
| 1:1 bevel differential | ±0.500 | ±0.500 | 1.0 | — |

The coefficients are the textbook values. **And the residual fell 12x, from 325 to 27 deg/s.** The
"~250 deg/s noise floor" this file has assumed throughout was never noise — it was the slip. So was
the "azimuth advances in 80-150 deg lurches at 3-10 Hz" of §6, which no amount of encoder forensics
could explain because the encoder was reporting it correctly.

Point by point, against `rate = 3.0*(A - B)` with nothing fitted:

    B held -1000:  A -1400 -1200 -1000  -800   +800  +1000  +1200  +1400
      measured      -1201  -598    +2   +601  +5420  +5945  +6527  +7060
      predicted     -1200  -600     0   +600  +5400  +6000  +6600  +7200

    A held -1000:  B -1400 -1200 -1000  -800   +800  +1000  +1200  +1400
      measured      +1213  +607    +2   -597  -5409  -5947  -6600  -7263
      predicted     +1200  +600     0   -600  -5400  -6000  -6600  -7200

At `A = B = -1000` the pod is **stationary: +2 deg/s, and `encd` alias margin 2048**, i.e. the tracker
saw no motion at all.

### The model, and why the firmware was right all along

    rate = 3.0 * (A_cmd - B_cmd)  deg/s          azimuth = (A-B)/2,  propeller = (A+B)/2

in COMMAND space. That is `azimuth = (wA + wB)/2` in PHYSICAL space with motor B mirrored, which is
exactly the `sign_b = -1` the firmware has always assumed. `applyRpm_` sends `a = spin + r`,
`b = spin - r`, so `A - B = 2r` and

    rate = 6.0 * r   deg/s          thrust = spin_rpm  (the common mode)

**`rpm_per_count 7.26` was derived from "an assumed 6.0 deg/s per rpm". The assumption is now measured
exact.** So the mixer, the feedforward sizing and the meaning of `r` were correct from the beginning.

### What this retires

**§12's "the roles are swapped" is wrong**, and `mix_swap` must stay OFF (it was never validated and
is now known to be unnecessary — leave the default). **§13's speed-ratio null (4.24), §14's
`null_ratio`, §15's "the null is razor-thin and motor B scatters", and §16's "the null drifts, it has
to be identified online" are all retired**: there is one null, it is at **r = 0**, it does not move
with thrust, and it puts BOTH motors at `spin_rpm`, comfortably above the 6-step seam. The entire
"the null requires a speed the seam forbids" problem (§10) is gone.

Set **`rpm_trim = 0`** and **`null_ratio = 0`**. Every nonzero trim in this file was compensating for
a slipping coupling.

### The lesson worth keeping

Three weeks of this file are a plant model fitted to a mechanical fault. The tell was there the whole
time and was read as noise: the residual never went below ~250 deg/s, the "null" moved with every
operating point, and repeat runs of an identical configuration differed by 100x. **A plant that will
not fit a linear model at a fixed operating point, and whose fitted parameters move with speed, is
reporting a mechanical fault, not asking for a better estimator.** §16 concluded "identify the null
online" — the right response was to go and find what was slipping.

Also vindicated: the operator's method rule. It voided the solo tests that had produced the 40:1 and
4.8:1 numbers, and it forced the both-motors-driven sweep (§17) that localised the fault to one
drive path — which is what prompted looking at the coupling at all.

---

## 19. THE HEADING HOLDS. Best configuration, and the two gains that mattered.

**2026-08-16, on the repaired drivetrain (§18).** The goal of this file — hold a commanded angle under
thrust — is met. Thrust is `spin_rpm` 1200 on both motors (the common mode), 11.2 V.

    --spin-rpm 1200 --rpm-trim 0 --vel-lpf 0 --rpm-off-max 60 \
    --kp 0.5 --ki 0 --kd 0 --ki-pos 0.05

    3/3 runs, 13 s:  mean|err| 1.0 / 2.1 / 2.3 deg     pos pk-pk 1.8-4.0 deg
                     net rotation -3 .. +1.5 deg       peak |vel| 9 deg/s

For scale, a free spin gives `mean|err|` ~90, and the best single run anywhere earlier in this file
was 19.7 deg with a median of ~110.

### Three changes got there, in order of size

**1. Authority had to shrink 7x.** `rpm_off_max` 400 was inherited from the broken plant, where large
offsets were needed to move at all. With `rate = 6.0*r` the same 400 rpm is **±2400 deg/s against a
`vmax` of 200 deg/s** — a 12x actuator overrange, so any error slammed the rail. **60** matches the
actuator to the setpoint range. This alone took `mean|err|` from 89 to 21 and `pos pk-pk` from 809
to 67.

**2. `ki` had to go to ZERO, and that killed a limit cycle.** Six repeats at `ki 0.3` gave one clean
run and five near-identical oscillations — `pos pk-pk` 65.9 / 68.7 / 66.2 / 66.7 / 66.7,
`peak |vel|` 284-289, i.e. a **1.37 Hz limit cycle** (from A = pk-pk/2 = 33.5 deg and
`peak|vel| = omega*A`; `rms/peak` = 0.66 ~ 1/sqrt(2), so a clean sinusoid), not run-to-run scatter.

The reason is structural, and it is a consequence of §18. With the drivetrain repaired the
feedforward is **exactly unity**:

    vsp -> u = kff*vsp -> r = 7.26*u = 7.26 * 0.02296 * vsp = 0.1667*vsp
    rate = 6.0*r = 1.000 * vsp

So the FF alone tracks the setpoint, and `ki` — sized when the FF was worthless against a slipping
coupling — is pure excess loop gain. Setting it to 0: `pos pk-pk` 67 -> **1.4**, `rms(vel)` 190 ->
**3.2**, `peak|vel|` 288 -> **9**, `net_rot` -> +0/+1/+1 deg.

**3. `ki_pos` 0.05 removed the standing offset.** At `ki 0` the pod held rock steady but 3.7-6.3 deg
off target — a position bias, which a VELOCITY integrator cannot see (the velocity error is zero).
`ki_pos` is the term for it (§14 added it for exactly this reason).

| `ki_pos` | steady-state err | `mean\|err\|` | `pos pk-pk` |
|---:|---:|---:|---:|
| 0 | -5.7 | 5.7 | 1.4 |
| **0.05** | **-2.1** | **2.1** | **3.0** |
| 0.15 | -1.8 | 2.2 | 7.4 |

0.15 buys nothing on error and triples the ripple. **0.05.**

### What this retires

`--vel-lpf` (0; 1.0 is the default and adds lag), `--verr-clamp` (irrelevant now — it existed because
`ivel` was random-walking against an rms-1100 disturbance that was the slip), and the §14/§15 gain
tables, which were all measured against a broken plant. **Do not carry any gain in this file forward
across §18** — re-tune from the values above.

### Not yet established

- **Setpoint tracking.** Everything above holds `tgt = 0`; commanding a NEW angle and slewing to it
  is untested on the repaired drivetrain.
- **Stiffness.** No disturbance rejection measured (`--poke-at` prompts for a hand push).
- **Higher thrust.** Only `spin_rpm` 1200 tried. The null is now at `r = 0` independent of thrust, so
  `thrust_rpm` ramping should finally be safe — but it is untested, and §15's ban on it was written
  against the broken plant.

---

## 20. THE LOOP IS AT 75 % OF ITS THEORETICAL BOUND. The rest is in `lib/vel_control`.

**2026-08-16.** `az step` was finally run (it existed since §9 and was never used), so the plant's
DYNAMICS are measured rather than assumed:

    tau  0.0819 s      gain 59.14 deg/s per command count      (R^2 0.909, open loop)
    -> every gain in this file before now ASSUMED tau 0.1-0.2 s

Two things fall straight out.

**`kff` was 36 % too high.** It was sized as `1/(rpm_per_count * 6.0)` = 0.02296 from the *static*
gain. The dynamic fit requires `1/59.14` = **0.01691**. The feedforward had been over-delivering by a
third, which is an overshoot source in its own right.

**The suggested `kp` is 3.05 for zeta=1** — and independently, the 1.37 Hz limit cycle of §19 implies
~-90 deg of actuator phase at 8.6 rad/s, hence `omega_c` ~ 3.6-4.5 rad/s. Two unrelated routes, same
answer. The gains were a factor of 6-9 too slow, not wrong in kind.

### Best configuration (6/6 runs, and the setpoint step)

    --spin-rpm 1200 --rpm-trim 0 --vel-lpf 0 --rpm-off-max 60 \
    --kp 6.0 --ki 0 --kd 0.5 --kff 0.01691 --vmax 400 --ki-pos 2.0

    hold      mean|err| 0.1-0.3 deg · pos pk-pk 0.8-1.5 · net_rot <= 0.6 deg/13 s · ss err 0.0
    90 deg step   rise(90%) 0.55-0.67 s · overshoot 0.6-2.8 % · final error < 0.35 deg

Progression this session: `mean|err|` 88.9 -> 21.1 -> 2.1 -> 1.1 -> **0.2 deg**; steady-state error
-5.7 -> -2.1 -> -1.1 -> **0.0 deg**.

**`ki_pos` scales with bandwidth, and that is what killed the steady-state error.** It is not an
independent knob: the PI zero belongs at ~`omega_c/5`, so `ki_pos ~ 0.6*kp`. At `kp` 0.5 the measured
optimum was 0.05 — exactly what that rule predicts — which is why sweeping `ki_pos` alone did nothing
(§19) and why raising `kp` let it go to 2.0 and take the offset with it. The 2 deg "steady-state
error" was never a floor; it was a 10 s integrator measured in a 13 s window.

**`kd` is no longer destabilising.** The source says it is "DELIBERATELY ZERO: the disturbance sits at
~8 Hz" — that disturbance was the slipping coupling (§18). `kd` 0.5 now improves every metric.

### Where the limit actually is, and it is NOT the motors

| | rate capability |
|---|---|
| drivetrain, demonstrated in the §18 sweeps (+-1400 rpm) | **+-8400 deg/s** |
| `rpm_off_max` 60 | +-490 deg/s |
| actually reached on a 90 deg step | ~200 deg/s |

**The pod uses ~2 % of what the drivetrain can do.** The binding constraint is the 82 ms lag, and
`host/models/350kv_*_sysid.json` says where it lives:

    single motor velocity loop:  tau 0.0955-0.1673 s,  pure delay 0.02-0.04 s
    azimuth loop (this section): tau 0.0819 s

**The azimuth plant's lag IS the ESC velocity loop.** The differential and the AS5600 add essentially
nothing. With `tau` 0.082 and ~30 ms of delay, `∠G = -45 deg` at ~8 rad/s, and the position loop
already spends -90 deg on its own integrator, so `omega_c <= ~8 rad/s`, i.e. **`kp <= ~8`**. Measured:
`kp` 6.0 is clean 6/6; **`kp` 10 (with `kd` 0.8, `rpm_off_max` 120, `vmax` 600) produced a consistent
+4 deg standing offset** on 3/3 runs. The bound is real and we are at 75 % of it.

**So further speed is not available from azimuth gains.** The next factor lives in
`lib/vel_control` (`kp` 0.06, `ki` 0.25, `dob` 0.80) — every ms taken off the inner speed loop raises
the ceiling above.

### Two structural gaps, both now specific

1. **The azimuth angle is NOT absolute.** `poll()` computes `pos_ = (accum - base_)*360/4096` and
   `base_` is re-zeroed at the handover (`azimuth.h:414`), so `--deg 90` means "+90 from wherever the
   spin-up happened to end", not a hull-referenced heading. The AS5600 is an absolute encoder within
   one turn and that information is being discarded. **This must change before the pod goes on a
   vehicle.**
2. **The integral is slow by construction, and feedforward is the way out.** The PI zero sits at
   `ki_pos/kp` = 0.33 rad/s (a 3 s time constant) because the loop has TWO integrators — the plant's
   `rate -> position` and the controller's — so the zero must sit well below `omega_c`. Raising it
   destabilises (`ki_pos` 0.15 at `kp` 0.5 tripled the ripple; §19). The fix is not a bigger gain but
   a **2-DOF controller**: feed the setpoint derivative forward so the integrator only trims residual
   bias instead of doing the tracking. The plant is known exactly now, so this is cheap.

### Still not measured

**Disturbance rejection.** `az trace --poke-at N` prompts for a hand push and nothing has used it.
Rejection bandwidth is the same `omega_c`, so the numbers above predict it — but predicted is not
measured.

### 20a. OPEN, AND IT DECIDES WHICH LEVER WORKS: is the 82 ms MECHANICAL or FEEDBACK-LIMITED?

Written down because §20 above leans one way and the evidence is not conclusive.

**Where the loop rate is NOT the problem.** `spinPoll()` runs every core0 `loop()` — there is no fixed
control period. The AS5600 tracker shares that loop and was measured at **1016/s at rest, 626/s under
load** (§6), so the Pico tick is **1.0-1.6 ms against an 82 ms plant, i.e. 50:1.** The Pico control
period is not the bottleneck and raising it buys nothing.

**The two candidate explanations for the 82 ms:**

1. **Feedback-limited.** The velocity loops are on the PICO but close on **eRPM from the ESC**, and
   that signal is bad at this timescale: §9 measured its correlation with true azimuth velocity at
   0.07, and the `az step` trace shows it scattering +/-60 rpm sample-to-sample at 155 Hz while the
   AS5600 moves smoothly. `esc_config.h:29` is **DShot300**, and `esc_session.h:305` shows EDT
   sharing frames — every VOLTAGE / CURRENT / TEMPERATURE / STRESS / STATUS frame is an eRPM frame
   that did not arrive. If this is the cause, the levers are DShot600, trimming EDT, and moving the
   azimuth channel off the velocity loop.
2. **Mechanical.** `host/models/*_sysid.json` calls its fitted tau a "first-order **rotor** lag", and
   `docs/measured-plant.md` carries a full `J*dw/dt = tau_drive - tau_drag` model with `J` taken from
   the free-coast window. A ~100 ms rotor+propeller time constant is entirely plausible. **If this is
   the cause, none of the levers in (1) help at all** — the motor still has to physically change
   speed — and the only lever is TORQUE AUTHORITY, of which the pod is currently using ~2 %.

Do not trust the sysid taus uncritically: they spread 0.096 / 0.167 / 0.185 across configurations,
and `measured-plant.md` itself warns that the earlier `tau_sine = 0.18` "was fitted to cogging noise".

**THE TEST, ~10 minutes.** Run `az step` unchanged, then raise the inner velocity-loop gains
(`gain <i> kp` / `gain <i> ki`, currently 0.06 / 0.25) and run the identical step again.

    tau falls        -> FEEDBACK-limited. Pursue DShot600, EDT trimming, and the hybrid drive below.
    tau unchanged    -> MECHANICAL. All of that is wasted effort; go after torque authority instead.

Settle this BEFORE spending time on the hybrid drive.

### 20b. The hybrid drive, stated correctly (it is NOT two controllers per motor)

The idea in §20 — rpm-drive for the common mode, raw for the difference — is right in spirit but must
not be built as two paths racing for one ESC. Each motor already receives a SUM of two components
(`a = spin + r`, `b = spin - r`), exactly like a quadcopter mixer; that part is not new. The hazard is
that a raw throttle term added on top is a DISTURBANCE to the velocity loop, which will spend its
authority cancelling it, so the azimuth channel would be high-passed and lose its DC gain.

The correct form is feedforward, not competition: **keep the velocity-loop setpoint at `spin +/- r`
exactly as now** (so DC is handled and the loop sees no error), and additionally feed forward the
throttle step that the loop is going to arrive at anyway — which `vel::VelocityController` already
knows, because it carries the calibrated `SpeedProfile` curve. Nothing fights; the transient simply
stops waiting for the loop.

Note this only pays off under branch (1) above. Under branch (2) the velocity loop was never the
delay, and the feedforward would be anticipating a step the rotor cannot follow any faster anyway.

---

## 21. §20a IS ANSWERED — FEEDBACK-LIMITED. And §20's plant numbers were fitted to a limit cycle.

**2026-08-16, later the same session.** Running the §20a test found the test's own instrument broken
first, so this section retires two numbers §20 rests on before it answers the question.

### The plant ID was measuring its own pre-roll oscillation

`az step` runs a CLOSED-loop pre-roll and then freezes the loop and adds the step open-loop
(`azimuth.h:490`), so the fit's baseline `v0` is the pre-roll's velocity. **The pre-roll was never
settled**: `az step`'s defaults are `kp 0.5 / ki 0.3 / vel_lpf 1.0 / rpm_off_max 200` — the
pre-repair gain set — which on the repaired plant limit-cycles at **±1000 deg/s**. `v0` was therefore
whatever phase the capture happened to land on, and the same is true of `step_base_u_`.

Re-running the identical command reproduced the failure and its randomness:

| run | pre-roll | base_u | delivered u | v_inf | fit said |
|---|---|---:|---:|---:|---|
| §20 (`az_lag_id`) | limit cycle | +7.5 | +27.5 | +1183 | tau 0.0819, gain **59.14** |
| 2026-08-16 (`az_lag_a1`) | limit cycle | −25.1 | −5.1 | −218 | tau 3.58, gain **−11.16** |

Both are v_inf/u = **43 deg/s per count**. The fits disagree because `dv = v_inf − v0` was taken
against a meaningless `v0`.

### The plant, measured with a pre-roll that actually holds

Same `az step`, but with the §20 hold gains pushed for the pre-roll
(`--kp 6.0 --ki 0 --kd 0.5 --ki-pos 2.0 --vel-lpf 0 --vmax 400 --rpm-off-max 100 --step-s 10`),
so the pod is genuinely still when the step lands (`v0` −2 / 0 / 0 deg/s, pre-roll rms 3-4 deg/s):

    3/3 runs:  gain 43.74 / 43.54 / 43.48 deg/s per count      t63 0.257 / 0.256 / 0.262 s
               tau  0.153 / 0.165 / 0.158 s (first-order)      t90 0.348 / 0.347 / 0.340 s
                                                               overshoot +31 / +34 / +35 %

**The gain is the mechanism's own number**: `6.0 deg/s per rpm × 7.26 rpm/count = 43.56`. §18 measured
that plant exactly, and the step now agrees with it to 0.4 %.

**So `kff` = 1/43.6 = 0.02296 — the ORIGINAL static value. §20's "`kff` was 36 % too high, use
0.01691" is withdrawn**; it was the bogus 59.14 inverted. A/B on the repaired pod, back to back,
`--deg 90`, 3 runs each, everything else at the §20 best config:

| | `kff` 0.01691 (§20) | **`kff` 0.02296** |
|---|---:|---:|
| mean\|err\| (median / worst) | 0.8 / 1.0 deg | **0.2 / 0.2 deg** |
| pos pk-pk | 1.7-2.2 deg | **0.9-1.0 deg** |
| net rotation, 13 s | +1.4 .. +1.8 deg | **−0.3 .. +0.4 deg** |
| steady-state error | +0.7 .. +1.0 deg | **−0.2 .. +0.1 deg** |
| time to \|err\| < 1 deg after a 90 deg step | 7.4 / 8.1 / 9.0 s | **5.2 / 4.7 / 4.0 s** |

The fast part of the 90 deg step is identical (both reach ~78 deg in ~0.45 s); the under-sized
feedforward showed up entirely as the long tail and the standing error. **Use `kff 0.02296.**

**And the response is not first-order.** It overshoots ~30 % and settles back — the inner velocity
PI's own overshoot. A single `tau` is an idealisation, which is part of why the historical taus
spread 0.082 / 0.096 / 0.167 / 0.185. Compare runs with **t63 / t90 / overshoot**, now printed by
`az step`.

### 21a. THE ANSWER: the 0.35 s is FEEDBACK-limited, and the gain lever is nearly exhausted

The §20a test, run as specified — identical step, inner velocity gains raised (`gain <i> kp|ki`):

| inner `kp`/`ki` | t63 | t90 | pre-roll | notes |
|---|---:|---:|---|---|
| 0.06 / 0.25 (as configured) | 0.257 / 0.256 / 0.262 s | 0.348 / 0.347 / 0.340 s | settled, rms 3-4 | 3/3 repeatable |
| **0.12 / 0.50 (2×)** | **0.188 / 0.189 s** | **0.269 / 0.252 s** | settled, rms 10-19 | **t90 −27 %** |
| 0.18 / 0.75 (3×) | 0.328 / 0.394 s | 0.402 / 0.449 s | **NOT settled** | reverses at onset, plateau wanders, steady rate down to ~320 |

**tau falls when the inner loop is made faster ⇒ FEEDBACK-LIMITED, not rotor mechanics.** The rotor
inertia branch (2) of §20a is refuted: a mechanical time constant does not care what `vel_control`'s
gains are.

But the headroom in *gain alone* is one doubling: 3× reverses the pod at the step onset and its
pre-roll no longer holds, which is the loop running out of phase margin against a noisy, delayed
eRPM. That is exactly the picture §20a branch (1) predicted, so the remaining levers are the ones
that improve the FEEDBACK rather than push against it:

1. **DShot600** — `esc_config.h:29` is still 300.
2. **Trim EDT** — every VOLTAGE/CURRENT/TEMPERATURE/STRESS/STATUS frame is an eRPM frame that did
   not arrive (`esc_session.h:305`).
3. **§20b's feedforward hybrid** — now known to pay off, since branch (1) is the live one.

### Tooling: the instrument now fails closed

`fit_step` (`host/pico_esc/aztrace.py`) had two defects that let all of this through silently, both
now fixed and covered by tests (`403 passed`):

- **It trusted the configured `step_s`.** `poll()` clamps a second time,
  `u = constrain(step_base_u_ + step_s_, ±u_max)` with `u_max = rpm_off_max/rpm_per_count`, and that
  clamp never reaches the host. The step is now read from the delivered command column.
- **It accepted any baseline.** It now REFUSES the fit when the pre-roll's velocity rms exceeds 10 %
  of the step response, naming the reason. Re-fitting this session's traces: the §20-style runs and
  both 3× runs are refused; the settled runs pass. `t63`, `t90` and `overshoot` are returned and
  printed alongside `tau`.

**The lesson, and it is the same one as §18 in a different costume:** a measurement whose *baseline*
is an oscillation returns a number, not a measurement — and the number was believed for a whole
tuning campaign because nothing in the tool asked whether the pod was actually still.

---

## 22. THE HEADING IS ABSOLUTE NOW — opt-in, persisted, and it refuses rather than guesses.

**2026-08-16.** §20's structural gap 1 is closed. `poll()` re-zeroed `base_` at the handover, so
`--deg 90` meant "+90 from wherever the spin-up ended" — an origin that is not a heading at all,
since the pod wanders during the RPM ramp (measured below: up to ~90 deg). The AS5600 is an
**absolute** encoder within one turn and that information was being discarded.

### What was added

- **`az zero [<deg>|show|clear]`** — park the pod, declare its current physical heading (default 0),
  and the reference is persisted. It is stored as the **RAW_ANGLE count**, not the tracker's
  accumulator: `accum_` starts at 0 on every boot and is meaningful only within a session, whereas
  RAW_ANGLE is absolute within a turn and therefore the only part that survives a power cycle.
- **Its own blob, `/cfg/azimuth.bin`** (magic + version + CRC), NOT a field in the per-ESC `Record`.
  The reference is a property of the rig, so storing it per-ESC would keep one copy per ESC and make
  "which is authoritative" a question; a separate file also means a heading calibration never bumps
  `CFG_VERSION`, i.e. never costs the operator their gains.
- **`az gain absolute 0|1`** / **`azimuth.py --absolute`**. Default OFF, so every existing recipe
  behaves exactly as before.
- **It refuses without a reference.** `az on` with `absolute=1` and no stored zero prints
  `err az: absolute mode has no heading zero — run az zero first` rather than silently falling back
  to relative (which would point "forward" at the magnet's arbitrary mounting angle). `az zero show`
  prints `head=unset`, never a number — `%f` of NAN reads as "inf" and looks like a measurement.
- The handover now clears only the genuine transition artefacts (`ivel_`, `velf_`, `ipos_`); the
  heading survives it. `az status` and the trace header carry `absolute` and `zeroraw`.

### Verified on the pod, 2026-08-16

    az zero              -> raw 2043 = 0.00 deg
    --absolute --deg 90  -> ended raw 3072 = 90.44 deg     (hold: mean|err| 0.2, pk-pk 1.0)
    --absolute --deg 0   -> came BACK to raw 2045 = 0.18 deg

The second run is the whole point: under the old behaviour "0" meant "stay where you are", because
the handover had just declared the pod's current position to be zero. Refusal path checked too:
with the zero cleared, `az on` errors instead of starting.

### Two things this exposes, both operational

1. ~~**The spin-up wander is much bigger than the ±10 deg the hold traces suggested.**~~
   **WITHDRAWN — see §23.** That claim came from reading 176.9 deg off a trace whose ring had
   WRAPPED, so its `t = 0` was already ~1 s into the run and past the handover: the pod was being
   *slewed to target*, not wandering. Written one paragraph below the gotcha that explains exactly
   this failure. The measured pre-handover excursion is 2.4–4.9 deg at the shipped ramp rate.
   What survives: a wrapped error always takes the shorter path *from where the pod is now*, so a
   large excursion before capture would flip the direction of travel — it just never gets large.
2. **Read the heading while the loop HOLDS it, not after `az off`.** One run held 88.7 deg and read
   77.3 deg after shutdown — the pod settles when the motors brake.

### Gotcha found while measuring this: the trace ring drops the START of a long run

`TRACE_N` is 2048 samples, oldest-overwritten, so at `decim 5` (~155 Hz) a run longer than **~13 s**
silently loses its beginning — `az on`, the spin-up and the handover — and `t=0` in the dump is then
a mid-run sample. That is exactly how a 176.9 deg mid-slew sample was first read as "the heading the
run started from". A full ring (`n = 2048`) is the only tell, and `_az_pull` now says so out loud.
The §21 traces are unaffected (n = 2012-2013, i.e. not full).

---

## 23. THE SPIN-UP: the typical drift is 3-5 deg and already optimal. The TAIL was the real fault.

**2026-08-16.** §22's next step was "bound the spin-up wander". Measuring it first retired the
premise and moved the target.

**Wander is defined from the ENCODER ALONE**: `pos(hand_ms) − pos(0)`, where `hand_ms` is the new
handover stamp in the trace header. It has to be, because **below the seam the ESC is in forced sine
and there is no eRPM at all** — the `rpm_a/rpm_b` columns in that window are not measurements and
nothing here rests on them. (`hand_ms` exists because without it a trace cannot separate "the pod
wandered" from "the pod was slewing to target" — which is exactly the mistake §22 made.)

### The ramp rate is already at its optimum, in both directions

Each run targets the pod's CURRENT heading, so the commanded move is ~0 and anything before the
handover is drift:

| `spin_slew` | t_hand | pre-handover excursion, per run |
|---|---:|---|
| 150 rpm/s | 2.90 s | 4.8 / 8.0 / 10.5 deg |
| **300 (shipped default)** | **1.51 s** | **2.4 / 2.6 / 3.4 / 4.5 / 4.5 / 4.9 deg** |
| 900 | 1.13 s | 1.7 / 16.7 / 32.3 deg |
| 2000 | 1.15 s | 0.6 / 32.1 / 33.8 deg |

Slower drifts for longer; faster enlarges the disturbance at the crossing faster than it shortens the
window. **300 is not a leftover — it is the measured optimum.**

### Two engineered fixes, both REFUTED by measurement

Both are implemented, default OFF, and kept because a ruled-out fix is worth more than an untried one:

- **Synchronised seam crossing** (`snap_rpm`/`snap_hold`): dwell below the seam, then command both
  motors past it in ONE `poll()` tick, so the two DShot frames leave microseconds apart instead of
  the pair straddling the seam for however long the ramp takes to cover their `vscale` difference
  (1.0000 vs 0.9225). Measured: **0.4 / 3.2 / 14.2 deg at 450 rpm, 13.4 / 22.2 / 46.8 at 480** —
  worse. At 480 the dwell itself sits on the seam, i.e. it *holds* the straddle it was meant to
  remove.
- **Bounded authority below the seam** (`spin_r_max`, gated by `senseRpmFloor()` so the pair can
  never straddle the seam): the offset had been forced to zero for the entire ramp, and the encoder
  is the one sensor that works down there. Measured: **8.0 / 9.8 / 9.8 / 11.0 deg at 40 rpm,
  4.4 / 9.8 / 20.9 / 28.1 at 80** — worse. In forced sine the azimuth's response to an rpm offset is
  not the 6-step plant the gains were sized against, so the "correction" mostly adds excursion.

### The tail was the real fault, and it is now caught

Across the heading sweep two runs went to **−93 deg and −737 deg** (two full revolutions). The trace
says exactly what happened: **B reported 0 while A ran 197-449 rpm for ~0.7 s** — one motor had not
caught in forced sine. Through the crossing the offset is zero by construction, so a whole-motor
mismatch turns the pod at ~1000 deg/s until the laggard arrives and the handover finally fires (2.23 s
instead of 1.51 s). No gain can answer that, and no eRPM exists to catch it early.

**`spin_asym_secs` (default 0.5 s, ON) ends the run after that long of one-motor-only running**, and
`az status` now names why a run stopped (`stop=rate|hardrate|spintimeout|asymstart|command`) — "it
just stopped" is the one thing a bench log must never say. It fired once in the 8-run campaign:

    before the guard:  ran to -737 deg over 2.2 s, then recovered and held
    after  the guard:  stopped at 1.31 s with the pod 5.9 deg off

### Where the loop stands, 8 runs, ~95 deg absolute steps (7 completed, 1 guard-aborted)

    error at capture   67-103 deg      settle to <1 deg   2.1 / 3.7 / 4.4 / 4.5 / 4.7 / 6.9 / 7.6 s
    hold mean |err|    0.11-1.06 deg (median 0.25)        hold pk-pk  0.5-2.1 deg

The fast phase covers most of the step in ~0.5 s and the last degree takes seconds — the slow
position integrator of §20's structural gap 2, unchanged and still the next real gain.
Graphs: the artifact published 2026-08-16 (transient, steady state, guard before/after, and every
configuration tried).

---

## 24. THE SLOW TAIL WAS A STALE ANTI-WINDUP RESIDUE, NOT THE POSITION INTEGRAL.

**2026-08-16.** §20's structural gap 2 blamed the slow approach on the PI zero sitting at 0.33 rad/s
"by construction". The traces say otherwise.

### What the trace actually showed

At `err = +11.95 deg` the proportional path alone asks for `kff*kp*err = +1.65` counts. **The loop was
delivering `s = 0.00`.** Something was cancelling P, and the `ivel` column names it:

    t+0.1 s   err +96.2   s +8.20   ivel -0.90     <- fast phase, command at the u_max rail
    t+2.0 s   err +17.0   s -0.10   ivel -0.90
    t+3.0 s   err +13.1   s +0.00   ivel -0.90     <- 13 deg off, commanding nothing
    t+4.0 s   err  +9.3   s +0.90   ivel +0.00     <- residue clears, error collapses in ~1 s

**`ivel` is the velocity integrator and `ki` is 0 in the shipped config.** With no integration it can
never unwind — but the anti-windup back-calculation still WRITES it: `ivel_ = uc - uff` whenever the
command clamps. The fast phase saturates against `u_max`, parks −0.90 counts there (−6.5 rpm of
offset, ~−39 deg/s of standing command bias), and that residue then cancels the proportional term for
the rest of the run.

**Back-calculation is only valid while the integrator is live.** With `ki == 0` the clamp residue now
goes nowhere: `ivel_ = (ki > 0) ? constrain(uc - uff, ±u_max) : 0`.

### The position integrator had the same trap, in principle

`ipos_` was also back-calculated against the `vmax` clamp: at `err` 97 deg, `kp*err` = 582 against
`vmax` 400, so it solves for `ipos_ ≈ -91 deg-s` — an integrator holding a charge that cancels P.
Replaced with **conditional integration**: integrate only when the output is not clamped, or when the
error drives out of the clamp. The integrator keeps what it has learned about the standing bias and
learns nothing from a saturated transient.

### Measured, on the same 90 deg absolute steps

| | time to reach \|err\| < 5 deg |
|---|---|
| before | 0.72 / 4.19 / 4.25 / 4.35 / 4.36 / 4.43 s — **median 4.30** |
| after | 0.62 / 0.67 / 0.67 / 0.70 / 0.72 / 0.73 / 0.74 / 0.75 / 0.75 / 0.79 / 0.80 / 5.01 s — **median 0.74** |

**5.8x faster to the target**, with the hold unchanged (mean\|err\| median 0.25 deg). The step itself
now looks like the plant: 89.7 → 43.4 → −5.9 deg in 0.8 s.

### Diagnosis discipline worth repeating

Two hypotheses were tested and REFUTED before the right one:
- "`vmax` saturation and its back-calculation are the cause" — then a step small enough never to
  saturate should have no tail. 30 deg steps settled in 5.8-6.7 s, no better. Refuted.
- "`ki_pos` is simply too slow" — with `ki_pos` 0 the same step is inside 2 deg in **2.0 s**, so the
  integrator is not too slow, it is holding the wrong value. (It is still needed: that run left a
  0.5 deg standing offset, and one of three left 66 deg.)

### What is left, and it is not a gain

After the fix the run reaches the target in 0.8 s and then **dithers ±1-3 deg** for a few seconds
before all samples are inside 1 deg. In that window the loop asks for 0.5-1.0 counts — **4-8 rpm of
offset out of 1200**, at the very bottom of what an integer DShot command and the ESC's own velocity
loop can resolve, with stiction on top. The remaining "settle time" is that floor, not the
controller. The levers there are command resolution or a deliberate dither, not tuning.
