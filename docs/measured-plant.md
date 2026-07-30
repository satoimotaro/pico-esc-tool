# The measured 350KV plant

What the ESC + F2838 350KV actually does, with every number traced to the tool, the file and the
**mechanical configuration** it came from. Code: `host/pico_esc/measured.py` (the numbers),
`host/pico_esc/plant.py` (the structure), `host/plantid.py` (fit + validate, offline).

```bash
python3 host/plantid.py provenance         # the inventory, mech config per row
python3 host/plantid.py fit --config bare_shaft_350kv_20260730
python3 host/plantid.py validate --all     # the out-of-sample table
python3 host/sim_obs.py --cross-up 36      # plant + observer + DOB + startup study
```

## Why this replaced the fitted model

`host/sim_obs.py` used to model this plant with four constants and a deadtime: a 0.18 s forced-sine
lag, a 0.025 s 6-step lag, a 250 rpm regime threshold, a 0.28 damping ratio, and a 0.28 s up-handoff
deadtime. It reported ~79 % VAF. None of it was physical:

- The 0.18 s sine lag cannot be rotor mechanics. `J*R/K_T^2` is **8.3 ms** at R = 0.7 ohm and
  **14.2 ms** at the measured `R_total = 1.20` — 13 to 22x smaller. It was fitted to the sysid step
  `65 -> 407`, where the rotor wanders **-29..249 rpm** around a **145 rpm** forced field: the
  dispersion is larger than the signal, so there is no first-order rise there to fit. That lag was
  fitted to cogging noise, and the module's `_smooth()` existed to make the fit look clean.
- The 250 rpm threshold plus 0.28 s deadtime encoded the up-handoff as a switch and a dead time. The
  measured fold shows no resolvable hysteresis (both legs cross in the same cmd 515..520 bracket), and
  the "dead time" is a duty ramp against a duty clamp: the rotor sits at **360-415 rpm** (not zero) for
  ~0.26 s and then covers the last 1800 rpm in **40 ms**.
- Every one of those constants absorbs drag, so changing the load invalidates all four. That is the
  failure this rewrite exists to end: **water must change one callable, not four numbers.**

## The three layers

1. **Firmware / electrical.** `duty6(cmd, cross_up)`, the forced-sine field law
   `rpm = cmd/1000 * 2499 / pp`, the seam as `max(sine field, KV*V*eff*duty6)`, the up-only crossover
   slew, the `tele` floor. Nothing here knows the rotor exists.
2. **Rotor.** `J*dw/dt = tau_drive(cmd, w) - tau_drag(w)` with ONE voltage-source torque law:

   ```
   tau_drive = K_T * (V*eff*duty_elec - w/KV_rad) * R / (R^2 + (pp*w*L)^2)
   ```

   At `duty = 0` it becomes `-K_T^2*w*R/(R^2+(pp*w*L)^2)`, the textbook short-circuit brake: **the brake
   is this law with the duty removed**, not a second model. `tau_drive(0, w) == -tau_short_circuit(w)`
   is asserted exactly (which is why `K_T` is written as the exact `60/(2*pi*KV)` and not the rounded
   `9.5493/KV`).

   ### `duty6` is a LOADED curve, and forgetting that counts drag twice

   `DUTY6_A/B` were fitted as `rpm_measured/(KV*V)` on a real bench, so the drag of that rotor is
   already inside them: `KV*V*eff*duty6(cmd)` is the speed the rotor **settles at**, not the speed a
   frictionless rotor would reach. Subtracting `tau_drag` on top of that drive law counts drag twice —
   and the first version of this model did exactly that. Integrated to convergence it landed **12.2 %
   below `steady()` at PROP cmd 543**, 5.4 % low at 680 and 2.9 % low at BARE 600, while this document
   claimed it could not. It was invisible because the equilibrium test inspected `tau_6step` *without*
   drag and every static test called `steady()` instead of integrating.

   The fix recovers the **electrical** duty the winding actually sees:

   ```
   duty_elec(cmd) = duty6(cmd) + tau_drag_ref(w6) * (R^2 + X^2) / (K_T * R * V * eff),  w6 = KV*V*eff*duty6
   ```

   `tau_drag_ref` (`PlantParams.drag_ref`) is the drag the **calibration** was taken against — a
   property of `duty6`, not of the rotor being simulated — so it must not move when the load changes.
   In air, where `drag == drag_ref`, the offset cancels and the ODE's fixed point **is** `steady(cmd)`;
   `test_step_converges_to_steady` integrates to convergence and asserts it to 2e-4 relative, at four
   commands per configuration and from both above and below. In water `drag > drag_ref` and the plant
   slows. The correction is 12-16 % of duty over the measured commands, i.e. exactly the size of the
   deficit it removes.

   Also clamped now (it was not): `duty_6step` and `electrical_duty` at 1.0 and `sixstep_rpm` at
   `KV*V*eff`, mirroring `MotorModel.rpm_6step`, which always clamped. Unclamped, `sixstep_rpm` grew
   linearly for ever — +15 % over the no-load top at cmd 900 and +42 % at 1000, **inside** this
   codebase's own `tmax=800` envelope, with two layers of the same architecture disagreeing on a basic
   physical bound.
3. **Load.** One `drag` callable. `plant.with_drag(params, f)` is the whole water interface.

Forced sine is deliberately a **different** law: below the seam the ESC forces commutation and a
synchronous rotor runs at exactly that rate with a load angle, not at a slip. So the sine branch is a
synchronous lock saturated at `SINE_PULLOUT_NM`. Modelling it with the short-circuit law instead
predicts the down-step ~4x too fast. The lock structure is *required* by the measurement — the rotor
settles at exactly the field speed, which a torque proportional to slip could never do against drag.
The saturation VALUE, however, does not survive a second trace: see the `680 -> 407` miss below.

## Parameters

| parameter | value | units | kind | mech config | transfers? |
|---|---|---|---|---|---|
| `KV` | 350 | rpm/V | measured | prop 2026-07-26 | yes (same motor) |
| `POLE_PAIRS` | 7 | - | measured | prop | yes |
| `K_T` | 0.0272837 | N*m/A | derived | prop | yes |
| `V_SUPPLY_PROP` / `V_SUPPLY_BARE` | 11.25 / 11.2 | V | measured | each | no (a parameter, not a state) |
| `EFF_350KV` | 1.0 | - | measured | prop | yes (fits both curves to <=2.1 %) |
| `R_TOTAL` | 1.20 +- 0.26 | ohm | measured | prop | yes, but 34 % is wiring and it corrodes |
| `J_PROP` | 9.761e-6 | kg*m^2 | **identified** (free coast) | prop 2026-07-30 | **no** |
| `J_BARE_FITTED` | 7.585e-6 | kg*m^2 | **identified** (free coast) | bare 2026-07-30 | **no** |
| `J_SHAPE_UNCERTAINTY_PCT` | 8.4 | % | **measured** | prop (two speeds) | yes — it is the drag table's error |
| `L_PHASE_FITTED` | 1.296 | mH | **fitted** (brake phase, `disarm` only) | bare | **OPEN** (but see the out-of-sample transfer below) |
| `BRAKE_CUTOUT_RPM` | 670 | rpm | **fitted** (brake phase, `disarm` only) | bare | **OPEN** |
| `DRAG_TABLE_PROP` | 5.90 -> 4.16 | mN*m | measured | prop | **no — and it is not prop drag.** Also the `duty6` reference drag, and the only air drag law |
| `FREE_DECEL_BARE/PROP_RPM_S` | 5283 / 4089 | rpm/s | measured (`disarm`, thrust-800) | each | no |
| `CUT_RELEASE_S` | 0.170 / 0.332 / 0.024 | s | measured | both (`disarm` only) | **yes for `disarm`**, assumed for the other two |
| `BRAKE_ENGAGE_S` | 0.137 / 0.113 / 0.072 | s | measured | both (`disarm` only) | **yes for `disarm`**, assumed for the other two |
| `BRAKE_PHASE_S` / `_REV` | 0.107/0.062/0.033 s, 1.18/0.51/0.62 rev | - | measured | bare | data only — **not** a model parameter |
| `SINE_PULLOUT_NM` | 8.24 | mN*m | fitted | bare | no |
| `SINE_COHERENT_CMD_MIN` | 210 | cmd | measured | both | **yes** (controlled pair) |
| `SINE_SPREAD_TABLE_BARE` | 92..234 | rpm | measured | both | **yes** (controlled pair; it does NOT shrink) |
| `DUTY_SLEW_UP_PER_S` / `DUTY_CROSSOVER_CLAMP` | 1.6002 / 0.1243 | duty/s, duty | fitted | prop | **OPEN** — both go through the prop drag table |
| `TELE_FLOOR_MECH` | 182 | mech rpm | measured | both | **yes** |
| `TELE_LIVE_S` | 0.109 | s | measured | bare | yes |
| `TORQUE_PER_CMD` | 0.68 | mN*m/cmd | measured | prop | **reporting only** |
| `STARTUP_OVERSHOOT_PCT` / `_T_PEAK_S` | 53.5 / 1.19 | %, s | measured | both | **yes** (controlled pair: under 1 pp) |
| `STARTUP_ABORT_TARGET_RPM` | 2400 | rpm | measured | both | **yes** — and it is a CONTROL failure, not inertia |

### How each unmeasured parameter is IDENTIFIED (this was wrong in the first version)

`R_total` is pinned (measured) throughout, and nothing below is fitted against a metric that contains
the two fixed cut delays.

- **`J` comes from the FREE-COAST window** — the phase after the drive is removed and before the brake
  bites, the one phase of the decay with no ESC torque in it. `J = tau_drag(w_mid)/alpha_free`, straight
  off the measured drag table with nothing fitted: `4.197 mN*m / 5283 rpm/s = 7.585e-6` bare and
  `9.761e-6` prop-on, both from `--thrust 800 --cut disarm` runs so they are a controlled pair.
  `disarm` **only**: the six bare decays span three cut modes and read 5042-5436 rpm/s on the four
  `disarm` runs but 6074/6113 on `deadman`/`zero_held` — 15 % faster, for a phase that by definition has
  no ESC torque in it. Those two also have much shorter free windows (0.072 / 0.113 s vs 0.129-0.143 s),
  so their "free" phase is probably already partly braked. An earlier constant averaged all six, read
  5550 rpm/s, and biased `J` 5 % low.
  The first version claimed `J_BARE` was "pinned by the brake-peak magnitude". That was **false**:
  nothing in that fit's cost function constrained the brake torque scale at all.
- **`(L, BRAKE_CUTOUT_RPM)` come from the BRAKE PHASE** — revolutions travelled and duration, both
  measured **from engagement**, on the four `disarm` decays. Two features x four decays for two
  parameters. See the metric note below for why this replaced time-to-stop.
- **`SINE_PULLOUT_NM`** from the open-loop down-step, bracketed above and below by physics: it must
  exceed the drag it holds against (5.90 mN*m over the coherent band) or the measured synchronous lock
  could not exist, and it must sit below the sine stall torque `K_T*V*(45/255)/R = 44.9 mN*m`. It lands
  at 7.58, i.e. 1.3x the drag and 17 % of the stall bound.
- **The crossover pair** from the single `407 -> 680` step: the plateau LEVEL pins the clamp and its
  DURATION pins the slew, so they *are* separable from each other (the first version said otherwise).
  What is missing is corroboration at any other command.

### THE BRAKE METRIC: revolutions and duration from ENGAGEMENT, not time-to-stop

Time-to-stop measured from `t = 0` is **~72 % fixed cut delay** (0.307 s of 0.406 s on `disarm`). It
therefore stays inside 13 % while the brake torque is out by 2x, and that is precisely how the first
version of this model certified a 2x-too-strong brake as "out of sample within 11 %". Demonstrated
directly in `test_time_to_stop_from_zero_is_delay_dominated_and_says_so`: halving `L` (roughly doubling
the brake torque at these speeds) moves time-to-stop by ~1 % and brake-phase revolutions by ~27 % —
a >10x difference in sensitivity, asserted as that ratio.

`plantid.py validate --brake` now prints both, so the gap is visible rather than inferred.

`R_total`'s +-0.26 ohm band propagates into `L` (1.02..1.58 mH) but **not** into `J`: the free-coast
window has no electrical term in it. `plantid.py fit` prints the band.

**`L` is a prediction, not a measurement.** Check it with an LCR meter across two phases (expect ~2L
line-to-line, so ~2.6 mH at 1 kHz) before reusing it anywhere. Note that this L puts the short-circuit
brake's peak at **41.0 mN*m at 1263 rpm**, while the pointwise deceleration residual (measured decel
minus the drag table, per 20 ms window) puts it nearer **60 mN*m at ~1900 rpm**. The trajectory fit uses
the integral and is far less noisy than single-window residuals, so it wins — but the two disagree by
~1.5x and that is the honest uncertainty on the brake torque scale.

**`J_bare` is identified, not measured.** Only `J_PROP` was ever measured directly. `J_BARE_FITTED`,
`L_PHASE_FITTED`, `BRAKE_CUTOUT_RPM` and the bare-shaft brake numbers are a set: never pair `J_PROP`
with them.

## Which cross_up each dataset was recorded at, and how we know

The prop-on datasets are **cross_up 34**; the 2026-07-30 bare-shaft fixtures are **cross_up 36**. This
needed evidence because no fixture CSV carries a `cross_up` column and the git history looks like it
says 36 for both: the bench was recalibrated at 36 on 2026-07-26 17:46 (`011acf8`), the prop-on
dynamics dataset was committed 2026-07-27 00:17 (`235eb56`) with no reflash commit in between, and the
2026-07-29 session note records 36.

Settled with data:

- Two no-load duty sweeps recorded minutes apart (`reports/duty_sweep_20260729_220535.csv` = A,
  `..._223122.csv` = B) are offset by **exactly 30 command counts at every point** — a pure intercept
  shift, which is what `cross_rescale_duty` does.
- The ESC was read directly and reports `sine_cross_up: 36`; at cmd 600 the encoder measured 1064.3 and
  1069.3 rpm on two runs. Sweep A interpolates to **1072.8** there (0.4-0.8 % away), sweep B to
  **1391.7** (30 % away, far beyond any load effect). So **A = 36, B = 34**.
- Interpolating both to the prop-on sysid's three 6-step commands:

  | cmd | postfix measured | sweep B (34) | sweep A (36) |
  |---|---|---|---|
  | 543 | 794.4 | 800.1 (+0.7 %) | 465.7 (-41.4 %) |
  | 612 | 1510.4 | 1514.6 (+0.3 %) | 1194.8 (-20.9 %) |
  | 680 | 2212.7 | 2213.6 (+0.0 %) | 1894.6 (-14.4 %) |

Reconciled with the git history by this project's recurring gotcha: **`esctool.py flash` wipes the ESC
config to firmware defaults** and leaves no commit trace. The velcal commit describes the config velcal
ran under; the sysid that followed sat at the post-flash value. Also derived from the same pair: the
30-count offset over 2 units gives `K = 0.0398` duty/unit, between the code's `DUTY6_CROSSUP_K = 0.0421`
and the theoretical 0.0391 — so the existing term's sign and magnitude are sound.

The full evidence is in `host/tests/data/plant_fixtures.json` under `_cross_up_provenance`.

## The controlled bare-vs-prop pair (2026-07-30 evening)

The operator refitted the propeller and re-took the whole set with the same tools, the same protocol and
the same starting speeds. `python3 host/plantid.py validate --pair` prints this comparison; it is
measured-vs-measured with no model in it, so nothing in it can be satisfied by tuning a parameter.

### First: a confound in the original pair

The prop-on decay this inventory originally used (`coastdown_20260730_011123.csv`) was taken with a
**closed-loop `--rpm 2200`** spin-up and starts at ~2210 rpm, while the bare-shaft set used **open-loop
`--thrust 800`** and starts at ~3130. Because the brake engages on a fixed DELAY, engagement speed
tracks the starting speed:

| dataset | spin-up | start rpm | w_on (engagement) | brake rev | brake dur |
|---|---|---|---|---|---|
| bare | `thrust 800` | 3121-3128 | 2349-2409 | 1.34 | 0.116 s |
| prop, OLD | `rpm 2200` | 2205-2214 | 1536-1606 | 0.99 | 0.101 s |
| prop, NEW | `thrust 800` | 3127-3137 | 2531-2741 | 1.97 | 0.157 s |

So any bare-vs-prop conclusion drawn between the first two rows was confounded by starting speed. Row 3
is the protocol-matched partner. (Row 3 engaging *higher* than row 1 from the same start is exactly what
a slower free coast requires — a third independent confirmation of the fixed-delay brake, and one that
also rules out a fixed *commutation count*, which would predict the opposite ordering.)

**`plant_fixtures.json` now records `spin_up` for every coastdown fixture**, because it is known to
matter. The old file is kept, tagged `role: prop_confounded`, because replaying it is what measures the
drag table's shape error (below).

### What the propeller changes: inertia and the free coast, and nothing else found

| | free decel | w_on | brake dur | brake rev | J |
|---|---|---|---|---|---|
| bare shaft | 5283 rpm/s | 2386 | 0.116 s | 1.34 | 7.585e-6 |
| prop fitted | 4089 rpm/s | 2605 | 0.157 s | 1.97 | 9.761e-6 |
| | **-23 %** | | | | **+29 %** |

### What it does not change — six things, each measured in both configurations

| behaviour | bare | prop | difference |
|---|---|---|---|
| seam bracket (fine sweep, both legs) | cmd **525..530** | cmd **525..530** | none resolvable |
| fold hysteresis | none resolvable | none resolvable | none |
| `tele` below the seam | **exactly 182.0**, cmd 60..450 | **exactly 182.0**, cmd 60..450 | none |
| `TELE_LIVE_S` bracket | last 0 at 0.105-0.108 s | last 0 at 0.105-0.108 s | none |
| worst sine spread (up / down) | 234 / 174 rpm | 238 / 211 rpm | no improvement |
| reversing region | to cmd 330 / 210 | to cmd 270 / 180 | no improvement |
| down-step through the seam | 1048 -> 175 rpm, no stall 3/3 | 1054 -> 175 rpm, no stall 3/3 | same |
| startup overshoot (rpm 1200) | 53/53/54 %, peak t 1.18-1.21 s | 53/53/54 %, peak t 1.20-1.23 s | **under 1 pp** |
| startup at rpm 2400 | ABORT_STALL, peak 657-780 rpm | ABORT_STALL, peak 700-827 rpm | **same failure** |

### Two expectations falsified, both with design consequences

1. **The propeller's inertia does NOT help the rotor follow the forced field.** More inertia was expected
   to tighten the sine-region spread and shrink the reversing band. Neither happened (238 vs 234 rpm
   worst spread). So the sine dispersion and the coherence band are load-INDEPENDENT for this pair, which
   is why `SINE_COHERENT_CMD_MIN`, `SINE_SPREAD_TABLE_BARE` and `SINE_REVERSALS_BARE` are now tagged
   `mech_independent=True` and their `borrowed` entries have been deleted. (Water, with real damping, is
   still untested.)
2. **The propeller does NOT fix the closed-loop `rpm 2400` startup stall.** It aborts in all three runs
   from a barely higher peak, at the same time. **So the target-dependent startup failure is a
   CONTROL / crossover-gap problem, not an inertia problem** — adding rotor inertia is not a fix and must
   not be proposed as one. Both falsifications are pinned by tests in `test_plant_replay.py` so they
   cannot quietly come back.

### A measured bound on the drag table's shape error

The same prop-on rotor gives `J = 9.002e-6` at w_mid 1876 (the old rpm-2200 file) and `9.761e-6` at
w_mid 2836 (the new thrust-800 file) — **8.4 % apart for a quantity that cannot depend on speed.** Since
`J` is inferred as `drag_at(w_mid)/alpha`, that gap is the error in `DRAG_TABLE_PROP`'s *speed
dependence* — the very shape this document already argues is bearing/iron loss rather than prop drag.
It is recorded as `J_SHAPE_UNCERTAINTY_PCT`, and it is the honest uncertainty on **both** inertias. This
replaces an argument with a measurement.

## Load dependence of the four known behaviours

| behaviour | law load-dependent? | observed numbers load-dependent? | why |
|---|---|---|---|
| 1. active brake at zero throttle | **timing: NO for `disarm` (measured in both). torque: transfers well, but still tagged OPEN. MODE-DEPENDENT.** | YES | The two delays were measured in BOTH configurations for `disarm` and agree within 4 % (`deadman`/`zero_held` were only ever recorded bare-shaft, so for those the tag is an assumption). The torque scale was fitted bare-shaft on `disarm` only; replaying it on the PROTOCOL-MATCHED prop-on decays gives brake-phase duration **-4.9..+4.9 %** and revolutions **-11.3..+12.9 %** — as good as the in-sample fit, across a +29 % inertia change, with nothing refitted. `L_PHASE_FITTED` and `BRAKE_CUTOUT_RPM` nevertheless stay **OPEN**: one inertia change on the same motor is not a different machine and is not water, and the cut-out is an ESC state transition never observed independently. `deadman`/`zero_held` still brake 2-3x harder from the same speed, which no single torque reproduces. |
| 2. forced-sine slip / cogging / reversal floor | pull-out torque NO (set by sine amplitude) | **YES, strongly** | The torque is fixed by the sine amplitude; the load torque it must beat is not. The incoherent band and the dispersion will shrink with damping (strongly in water). |
| 3. seam location + smooth ramp | **mostly NO** | mostly no | Set by `cross_up`, KV, V and `duty6` — firmware/electrical. Load shifts it only via 6-step droop (~1-2 % prop-on in air). One parameter-free rule fits both configurations' departure brackets. |
| 4. startup overshoot and the target-dependent `ABORT_STALL` | **YES, strongly** | YES | Closed loop, and gated on the plant's ability to traverse the crossover gap. Do not port to any other rotor. |

### The "peak moves with load" question, resolved

`66ff9ef` flagged an open question: the brake's deceleration peak was measured at **1455-1596 rpm**
bare-shaft but **865-973 rpm** prop-on, and a purely electrical slip law should peak at the same rotor
speed regardless of load.

**The peak location never moved. The engagement is delayed, and the rotor had reached a different speed
by then.** Measured, per decay, by a two-segment fit of encoder position:

| fixture | drive released at | brake bites at | rotor speed then |
|---|---|---|---|
| `coastdown_noprop` x3 (disarm) | 0.162-0.181 s | +0.129-0.138 s | 2349-2409 rpm |
| `cutmode_noprop` disarm | 0.163 s | +0.143 s | 2390 rpm |
| `cutmode_noprop` deadman | 0.332 s | +0.113 s | 2392 rpm |
| `cutmode_noprop` zero_held | 0.024 s | +0.072 s | 2451 rpm |
| `coastdown` (prop, rpm-2200 spin-up) x3 | 0.163-0.167 s | +0.128-0.137 s | 1513-1606 rpm |
| `coastdown_prop` (prop, thrust-800) x3 | 0.157-0.167 s | +0.130-0.140 s | 2531-2741 rpm |

The delay is the same in every row; the rotor is simply at a different speed when the brake arrives, and
that speed tracks the STARTING speed. The last row is the decisive one: with the propeller fitted AND the
bare-shaft spin-up, engagement moves to 2531-2741 rpm — **higher** than bare-shaft, because the free
coast is slower. So the 865-973 rpm figure that raised the original question was the closed-loop
spin-up's lower starting speed, not the propeller. So the shape is **not** evidence
against an electrical law — but it is not evidence for load-independence either, and the brake torque is
therefore left tagged **OPEN**.

Independently re-derived during review from raw encoder position with a centred sliding fit — not via
`coast_start`, not via the fitting window — with the same structure in all three `coastdown_noprop` runs
and all three prop-on runs. It also discriminates against the obvious alternative: a fixed *commutation
count* would predict a 0.19 s prop-on engage delay against the measured 0.128-0.137 s.

### What the brake law does NOT explain

- **The mode dependence is in the TORQUE, not only the delay.** From the same ~2400 rpm the measured
  brake phase is **0.107 s / 1.18 rev (`disarm`), 0.062 s / 0.51 rev (`deadman`), 0.033 s / 0.62 rev
  (`zero_held`)** — a systematic ~3x ordering. One short-circuit torque cannot be all three. The model
  is fitted to `disarm` (the default, and the mode the prop-on decays used) and is therefore **+84 % on
  `deadman` and +255 % on `zero_held`** in brake-phase duration. `zero_held` also drives the rotor
  **through zero** (the trace springs back to 105-148 rpm), consistent with the ESC actively servoing a
  stationary field while armed rather than applying a passive short. Carried as data
  (`measured.BRAKE_PHASE_S` / `_REV`) and asserted as the factor in
  `test_the_mode_dependence_is_in_the_torque_and_is_not_modelled`, so a future "fix" that quietly
  splits the difference fails the test.
- **The brake CUTS OUT at low speed, and that is measured.** Subtract the drag table's prediction from
  the measured deceleration and the brake's contribution runs ~60 krpm/s at 1900 rpm, ~10 krpm/s at 600,
  ~4 krpm/s at 450 and **zero (or below) by ~250 rpm**, in all four `disarm` decays. The short-circuit
  law's own low-speed branch falls only as `w` and would still be ~36 krpm/s at 450 rpm — 7x too much.
  A short circuit the ESC has stopped **commutating** (BEMF zero-crossing detection fails at low speed)
  is stationary in the rotor frame, so its torque averages out over a revolution: that is a cut-out, not
  a decay, and it is the one mechanism that also explains the spring-back through zero. Modelled as one
  extra parameter, gated as a STATE so `tau_short_circuit` stays exactly `tau_6step` at duty 0.
  Without it, no `L` fits: the shape needs more torque high and less torque low than the law allows, and
  the best single-`L` compromise misses the brake-phase duration by 30-41 % at every `disarm` decay.
- **A free shape exponent was rejected, and that decision still stands.** Generalising the law to
  `2x^n/(1+x^{2n})` fits with `n ~ 2.1` for a ~17 % cost improvement and an implausible `J_bare` of
  3.0e-6 (less than half the prop-on inertia, for the same rotor minus a light propeller). The residual
  it was papering over is the low-speed cut-out above, which now has a mechanism and one parameter
  instead of a fractional exponent.

### `DRAG_TABLE_PROP` is not prop drag — and it is the only air drag law there is

The table **falls** with speed (5.90 -> 4.16 mN*m over 753 -> 2922 rpm). No drag law permits that: aero
goes as `w^2`, viscous as `w`, Coulomb is flat. It is a lumped bearing/iron loss plus a `K_T*I_dc`
measurement bias.

**The propeller adds nothing resolvable in air**, on three independent lines:

1. the bare-shaft static curve at cmd 555/570/600 matches a `duty6` that was **calibrated prop-on** to
   <= 1 %, which it could not if the two rotors had materially different drag at those speeds;
2. the unphysical fall itself — bearing and iron losses are propeller-independent;
3. the controlled pair's own numbers: the propeller changes the free-coast deceleration by -23 % while
   changing `J` by +29 %, so `tau_drag = J*alpha` comes out at **4.20 mN*m bare and 4.18 mN*m prop-on**
   at their respective window midpoints — the drag torque itself is unchanged to 0.5 %.

So there is **one** drag law, `drag_at`, used as both `drag` and `drag_ref` for both presets.

CAVEAT, now quantified: the controlled pair also bounds how wrong that table's SHAPE is — the same rotor
gives `J` 8.4 % apart at two window midpoints (`J_SHAPE_UNCERTAINTY_PCT`). A table with the correct speed
dependence would give the same `J` at both.

An earlier version carried a separate viscous bare-shaft coefficient, justified by "a 4.3 mN*m Coulomb
torque could not permit holding 96..214 rpm on a tiny sine amplitude". That justification was **false**
— `SINE_PULLOUT_NM` is 7.6 mN*m and the sine stall torque is ~45 mN*m, both above 4.3 — and the viscous
law was contradicted by the data as well: it under-predicts the prop-on free deceleration by 31 %. Both
the parameter and the reasoning are gone.

**Consequence: the `w^2` coefficient water needs is UNMEASURED.** `PlantParams.drag` is a callable so
that water replaces it without touching anything else — but supplying a plausible-looking `k*w^2` fitted
to this table would be inventing data.

## Out-of-sample validation achieved

`python3 host/plantid.py validate --all` prints this table. Summary:

| check | criterion | achieved |
|---|---|---|
| prop-on static curve, cmd 543/612/680 | <= 4 % | +1.25 .. +2.08 % |
| bare-shaft static, cmd 555/570/600 both legs | <= 3 % | +0.01 .. +0.98 % |
| **`step()` integrated to convergence vs `steady()`** | must agree | **-0.000 %** at 4 commands x 2 configs, from above and below |
| wrong `cross_up` on the same points | > 15 % | 13.7 .. 45.0 % (**one point at 13.7 %**, mean 30 %) |
| seam cmd at cross_up 36 | [510, 525] | 514.7 |
| seam cmd at cross_up 34 | [470, 545] | 481.8 |
| nothing exceeds `KV*V*eff` | (new) | agrees with `MotorModel.rpm_6step` to 1e-9 at cmd 700..2000 |
| `tele` vs 60 sinesweep rows | max 25, mean 12 rpm | max **33.8**, mean 8.5 rpm |
| `tele` on startup, t < 0.5 s | 5 rpm | 4.9 / **26.3** / 1.5 rpm (2 of 3 runs) |
| **brake phase, 4 `disarm` decays (IN SAMPLE)** | — | duration -8.6 .. +7.7 %, revolutions -8.7 .. +11.4 % |
| **brake phase, 3 PROTOCOL-MATCHED prop-on decays (OUT OF SAMPLE)** | — | duration **-4.9 .. +4.9 %**, revolutions **-11.3 .. +12.9 %** |
| brake phase, 3 CONFOUNDED prop-on decays (rpm-2200 spin-up) | — | duration +20.9 .. +31.2 % — the protocol, not the brake |
| brake phase, `deadman` / `zero_held` | — | **+86 % / +259 %** on duration — NOT modelled, asserted as the factor |
| time-to-stop from t=0 (delay-dominated, for comparison) | — | -6.2 .. +6.1 % on `disarm`/prop, +64.2 % on `zero_held` |
| cut delays vs `coastdown.coast_start()` | 30 ms | 0 / 18 / 26 ms |
| down-step half-time (IN SAMPLE — see below) | 50 % | **+1 %** (25.0 vs 24.9 ms) |
| down-step, prop-on (verified from `host/reports/`, not committed) | — | 1054 -> 175 rpm, no stall 3/3, undershoot 24-28 % vs bare 30-35 % |
| down-step trough | 100-200 rpm | 171 rpm, no stall, settles on the field |
| `407 -> 680` time to 95 % | 25 % | +12.1 % (0.496 vs 0.442 s) |
| `407 -> 680` plateau | 300-500 rpm | 389-390 rpm (measured 360-415) |
| `680 -> 407` down-crossing | — | **VAF 25.7 %, model ~2x too slow** — documented miss, see below |
| forced sine, cmd 270..510 | 30 % | 28.4 % (worst: cmd 360 up leg) |
| VAF, `407 -> 680` | reported | 99.2 % |
| VAF, sine-region steps | `n/a` | `n/a (cogging-dominated)`, by design |
| transferability | asdict-identical minus `drag` | True, `drag_ref` unmoved, monotone slowdown, brake untouched |

### What is genuinely out-of-sample, and what is not

- **OUT OF SAMPLE:** the three **protocol-matched** prop-on decays
  (`coastdown_prop_20260730_211609.csv`). They reuse the bare-shaft-fitted `L` and `BRAKE_CUTOUT_RPM`
  with the prop-on `J`, across a +29 % inertia change, and nothing was refitted for them —
  `plantid.fit` asserts it never sees prop-on data, and
  `test_brake_phase_out_of_sample_on_the_prop_on_decays` asserts the hold-out structurally. **This is the
  only out-of-sample evidence the brake law has, and it is now a good one:** -4.9..+4.9 % on duration, no
  worse than the in-sample fit.
  Iteration 1 claimed this at "within 11 %" using time-to-stop from `t = 0`, of which ~72 % is fixed
  delay; that claim was retracted. Iteration 2 measured it properly but on the *confounded* file and got
  +13..+24 %. Part of that error was the protocol, not the brake: with the current parameters the
  confounded file still reads +21..+31 % while the matched one reads -5..+5 %.
- **NOT out of sample:** the down-step (`SINE_PULLOUT_NM` was identified from it) and the `407 -> 680`
  step (both crossover parameters were). Those rows are consistency checks.

### Misses, all asserted at the achieved value

1. **`tele` fold sweep 33.8 rpm vs 25.** One row (cmd 545 up leg) where `tele` reads 487 against an
   encoder mean of 453 (+7.5 %) mid-seam. Mean over 60 rows is 8.5 <= 12.
2. **`tele` startup run 2, 26.3 rpm vs 5.** One row of 158 (t = 0.474 s) where the rotor transiently
   reached 208 rpm while `tele` still reported the 182 floor — no BEMF lock yet. Runs 1 and 3 are 4.9
   and 1.5 rpm.
3. **`cross_up` guard 13.7 % at cmd 680 vs the 15 % floor.** The shift is a fixed *duty* offset, so it
   is relatively smallest at the largest duty. Mean 30 %, max 45 %.
4. **`deadman` / `zero_held` brake phase +86 % / +259 %.** The mode-dependent torque, unmodelled by
   design and asserted as the factor.
5. **`680 -> 407` down-crossing, VAF 25.7 %.** The measured descent covers 2226 -> 497 rpm in 40 ms
   (~43 krpm/s); the model, with the constant pull-out torque identified at cmd 480, manages ~13 krpm/s
   and takes 78 ms to the half-value against a measured 40 ms. So the forced-sine braking torque **grows
   with slip** and one saturation value cannot cover both descents. The lock structure is still required
   (the rotor settles at exactly the field speed), but the saturation is not constant. Added to the
   not-modelled list rather than papered over with a second fitted coefficient on a single extra trace.
6. **The brake torque scale disagrees with the pointwise residual by ~1.5x** (41.0 mN*m at 1263 rpm from
   the trajectory fit vs ~60 mN*m at ~1900 rpm from per-window residuals).
7. **`J` carries a measured +-8.4 % uncertainty** (`J_SHAPE_UNCERTAINTY_PCT`) from the drag table's speed
   dependence, and it propagates into everything fitted against it — `L`, the cut-out and the sine
   pull-out.

## The startup overshoot: an honest negative result

The measured bare-shaft startup overshoots **53-54 %** with `t_peak` 1.19 s, settling by 3.0-3.2 s. The
hypothesis was that this is the `tele` floor (182 rpm reported while the rotor is slower) feeding
integrator windup. `host/sim_obs.py` section (c) tests it by running the SAME plant twice and changing
only the sensor model.

The 182 rpm FLOOR and the `TELE_LIVE_S` startup SILENCE are now **separate knobs**
(`MeasuredEscHost(tele_floor=, tele_live=)`) so each is tested alone rather than both at once.

**The hypothesis does not survive.** In the only comparison where an overshoot number means anything —
the two gain settings that actually settle — removing both sensor lies moves the overshoot **0.3 % ->
0.7 %**, i.e. very slightly *worse*. The floor can only lie while the rotor is below 182 rpm, and on this
plant that window is under 0.1 s — far too short to wind an integrator up by 500 rpm.

Criterion (a) is **not met** as the plan framed it: with gains that settle the overshoot is 0.3 %,
nowhere near 25 %. With the shipped gains the peak does exceed the 1200 rpm target (by 18 %, and by 34 %
with the sensor lies removed) but the loop **never settles** — a ~1030 rpm tail ripple, so any
"overshoot %" against that tail mean is not a number. The plant is no longer *structurally* incapable of
overshooting (the old fitted-lag model could not overshoot at all), which is the only part of (a) that
survives, and it is not offered as a reproduction of the 53-54 % measurement.

What the model does show is that the loop's problem is the **seam's incremental gain**: at cross_up 36
the steady speed goes 186 -> 259 rpm over cmd 515 -> 522 and then 259 -> 1194 rpm over cmd 522 -> 612, so
any integral trim large enough to cross the seam bang-bangs across it. That is the Lake note "do NOT close aggressive feedback across the crossover gap"
reproduced from measured numbers, and it is the same seam gain that makes `rpm 2400` `ABORT_STALL`
bare-shaft while `rpm 1200` starts.

The magnitude was already explicitly deferred (53-54 % came from the FIRMWARE `vel::VelocityController`;
`host/pico_esc/velocity.py` is a mirror with different gains). The **mechanism** is now also reported as
unresolved rather than claimed. Nothing was tuned to hit 53 %.

## Deliberately not modelled

1. The instantaneous cogging waveform and individual reversals below cmd 210. Carried as a dispersion
   table plus `coherent=False`: the plant declines to be a speed source there rather than lying.
2. Absolute torque / thrust in N*m. `R_total` is 34 % wiring, per-installation, and corrodes.
   `TORQUE_PER_CMD` is reporting only.
3. Thermal derating and the staged limiter. No fixture has thermal data (`tempC` is a flat 31 C).
4. Electrical dynamics faster than ~1 ms (`L/R ~ 1.1 ms`): the current loop is algebraic.
5. Phase-current waveform, commutation ripple, demag, PWM. Cycle-average only.
6. Battery sag as a state. `V_supply` is a parameter.
7. Reverse direction, and any `sine_cross_dn` other than 239.
8. Water. No water data exists; the drag callable is the interface.
9. Prop inertia separately from rotor inertia.
10. The mechanism of the 182 rpm `tele` floor. 182 mech = 1274 eRPM, identical in both configurations
    and at every cmd 60..450. Candidates — a bidir-DShot eRPM encoding floor, or the ESC reporting a
    forced-commutation intention — are distinguishable only with a different-PP motor.
11. **The brake torque's dependence on the cut mode.** `deadman` and `zero_held` brake 2-3x harder than
    `disarm` from the same speed. The model fits `disarm` and carries the other two as data.
12. **The forced-sine pull-out torque's dependence on slip.** One constant value fits the `600 -> 480`
    down-step and is ~2x too weak on the `680 -> 407` down-crossing (miss 5 above). A slip-dependent
    saturation would need a second coefficient and a second trace to identify it.
13. **The crossover ramp's command scaling.** Because `applied_duty` returns `min(slew, clamp)`, the ramp
    is visible for only ~80 ms and the clamp holds the remaining ~320 ms, so the plateau ends at
    `electrical_duty(cmd)/slew` — LATER for a LARGER command (0.18 s at cmd 560, 0.42 s at 680, 0.63 s at
    800). For a BEMF-lock handoff that is backwards. Only cmd 680 was ever measured, so this is
    behaviourally a command-scaled dead time plus a duty clamp, and **the scaling is an untested
    prediction**. It differs from the old fixed 0.28 s deadtime in that the plateau LEVEL is a modelled
    speed rather than zero, which is what the trace shows; do not read more into it than that.

## Re-measurement recipe

When the propeller goes back on (or any rotor changes), each number below is re-derived by exactly one
command. **Re-apply the whole ESC config after every flash** — `esctool.py flash` wipes it, which is how
the cross_up confusion above happened in the first place — and record the `cross_up` you are running.

| number(s) | command | notes |
|---|---|---|
| `cross_up` itself | `python3 host/esctool.py dump` | read it; never assume it |
| `DUTY6_A/B`, `DUTY6_CROSSUP_K` | `python3 host/duty_sweep.py 1 --cross-up <byte>` twice, at two `cross_up` | `--cross-up` is required and is stamped into the CSV |
| `V_SUPPLY_*`, `EFF_350KV` | the same sweeps + a PSU reading | `eff` is the residual scale |
| `DRAG_TABLE_PROP` | `duty_sweep` + PSU current, `tau = K_T*I_dc` | and re-read the warning above before believing it |
| `J_PROP` / `J_BARE_FITTED`, `FREE_DECEL_*` | `python3 host/coastdown.py 1 --thrust 800 --repeats 3` | **use `--thrust 800`, NOT `--rpm 2200`** — the spin-up sets where the brake engages, and mixing protocols is what confounded the original bare-vs-prop pair. `disarm` cut only. Only the pre-brake window is a drag measurement |
| `CUT_RELEASE_S`, `BRAKE_ENGAGE_S` | `coastdown.py 1 --thrust 800 --cut {disarm,deadman,zero}` | three runs; `--analyze` accepts the `mode` column |
| both `J`s | the free-coast window of those decays, then `python3 host/plantid.py fit` | `J = tau_drag(w_mid)/alpha_free`; nothing electrical enters it. Re-take BOTH configurations with the same protocol or the comparison is worthless |
| `J_SHAPE_UNCERTAINTY_PCT` | the same rotor at two different starting speeds | the spread between the two `J` estimates IS the drag table's shape error |
| `L_PHASE_FITTED`, `BRAKE_CUTOUT_RPM` | the BRAKE PHASE of the `disarm` decays, then `plantid.py fit` | revolutions + duration from ENGAGEMENT; check `L` with an LCR meter |
| `BRAKE_PHASE_S` / `_REV` | all three `--cut` modes | data, not parameters — re-measure to see whether the 3x torque ordering persists |
| `TELE_FLOOR_MECH`, `SINE_SPREAD_TABLE_*`, `SINE_COHERENT_CMD_MIN` | `python3 host/sinesweep.py 1 --updown` | encoder-only; the AS5600 must be fitted |
| the seam brackets | `python3 host/sinesweep.py 1 --cmds 495,505,510,515,520,...` | the fine fold sweep |
| `SINE_PULLOUT_NM` | `python3 host/steptrace.py 1 --mode thrust --from 600 --to 480` | do NOT decimate: the half-time is ~24 ms. Also record `--from 680 --to 407`, which is where one constant value fails |
| `DUTY_SLEW_UP_PER_S`, `DUTY_CROSSOVER_CLAMP` | `python3 host/sysid.py` (the `407 -> 680` step) | not separable from each other on one step |
| `STARTUP_*` | `python3 host/steptrace.py 1 --mode rpm --to 1200 --startup` | closed loop; re-check `--to 2400` for the abort. Measured identical in both configurations, so a change here means the FIRMWARE or the crossover moved, not the rotor |

`steptrace.py` now writes a `cmd` column, so traces recorded from 2026-07-30 onward are self-describing
for replay (the prop-on repeats carry it, and their manifest entries say "RECORDED" rather than
"inferred"). `--analyze` reads both schemas.

**Record the spin-up method.** Every coastdown fixture in `plant_fixtures.json` now carries `spin_up`,
and the manifest names each fixture's CAMPAIGN so `cross_up` is not guessed from the filename —
`coastdown_20260730_011123.csv` is dated the 30th but belongs to the 2026-07-26 cross_up-34 calibration.
`cross_up` and `mech_config` are independent axes; neither implies the other.
