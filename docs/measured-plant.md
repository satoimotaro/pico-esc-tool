# The measured 350KV plant

A plant model whose **structure** is fixed by physics and whose **numbers** are a provenance-tagged
inventory, so that moving the thruster to water changes one callable (`drag`) and nothing else.
Read [`README.md`](README.md) first for the operational facts and the confidence tiering.

Code: `host/pico_esc/measured.py` (the numbers + `PROVENANCE`), `host/pico_esc/plant.py` (the structure),
`host/plantid.py` (offline `fit` / `validate` / `provenance`). It replaced a fitted model whose four
constants — `tau_sine=0.18`, `tau_line=0.025`, `cross_mech=250`, `zeta_sine=0.28` — were not physical
quantities; `tau_sine` in particular was fitted to cogging noise (the rotor wanders 19–249 rpm on the
step it came from), and rotor mechanics are `J*R/K_T² = 8–16 ms`, 4–20× smaller than the 0.18 s claimed.

## The three layers

1. **Firmware / electrical.** `duty6(cmd, cross_up)`, the forced-sine field law
   `rpm = cmd/1000 * 2499 / pp`, the seam as `max(sine field, KV*V*eff*duty6)`, the up-only crossover
   slew, the `tele` floor. Nothing here knows the rotor exists. **The seam is not a parameter** — it
   emerges where the two branches cross (cmd 514.7 at `cross_up=36`, 481.8 at 34), and there is no
   hysteresis parameter because none is measurable.
2. **Rotor.** `J*dw/dt = tau_drive(cmd, w) - tau_drag(w)` with ONE voltage-source torque law:

   ```
   tau_drive = K_T * (V*eff*duty_elec - w/KV_rad) * R / (R^2 + (pp*w*L)^2)
   ```

   At `duty = 0` this becomes `-K_T^2*w*R/(R^2+(pp*w*L)^2)`, the textbook short-circuit brake: **the
   brake is this law with the duty removed**, not a second model. `tau_drive(0, w) == -tau_short_circuit(w)`
   is asserted exactly, which is why `K_T` is written as the exact `60/(2*pi*KV)`.
3. **Load.** One `drag` callable. `plant.with_drag(params, f)` is the whole water interface.

**`duty6` is a LOADED curve — do not subtract drag twice.** `DUTY6_A/B` were fitted as
`rpm_measured/(KV*V)` on a real bench, so that rotor's drag is already inside them: `KV*V*eff*duty6(cmd)`
is the speed the rotor **settles at**, not what a frictionless rotor would reach. The electrical duty the
winding actually sees is therefore

```
duty_elec(cmd) = duty6(cmd) + tau_drag_ref(w6) * (R^2 + X^2) / (K_T * R * V * eff),   w6 = KV*V*eff*duty6
```

where `tau_drag_ref` (`PlantParams.drag_ref`) is the drag the **calibration** was taken against — a
property of `duty6`, not of the rotor being simulated — so it must not move when the load changes. In
air, where `drag == drag_ref`, the offset cancels and the ODE's fixed point **is** `steady(cmd)`
(`test_step_converges_to_steady` integrates to convergence and asserts it to 2e-4 relative, at four
commands per configuration, from above and below). In water `drag > drag_ref` and the plant slows. The
correction is 12–16 % of duty over the measured commands.

`duty_6step`/`electrical_duty` are clamped at 1.0 and `sixstep_rpm` at `KV*V*eff`, matching
`MotorModel.rpm_6step`.

**Forced sine is deliberately a different law.** Below the seam the ESC forces commutation and a
synchronous rotor runs at exactly that rate with a load angle, not at a slip — so the sine branch is a
synchronous lock saturated at `SINE_PULLOUT_NM`. The lock structure is *required* by the measurement (the
rotor settles at exactly the field speed, which a slip-proportional torque could never do against drag);
using the short-circuit law there predicts the down-step ~4× too fast. The saturation **value** does not
survive a second trace — see the `680 → 407` miss below.

## Parameters

| parameter | value | units | kind | mech config | transfers? |
|---|---|---|---|---|---|
| `KV` | 350 | rpm/V | measured | prop 2026-07-26 | yes (same motor) |
| `POLE_PAIRS` | 7 | - | measured | prop | yes |
| `K_T` | 0.0272837 | N*m/A | derived | prop | yes |
| `V_SUPPLY_PROP` / `_BARE` | 11.25 / 11.2 | V | measured | each | no (a parameter, not a state) |
| `EFF_350KV` | 1.0 | - | measured | prop | yes (fits both curves to <=2.1 %) |
| `R_TOTAL` | 1.20 +- 0.26 | ohm | measured | prop | yes, but 34 % is wiring and it corrodes |
| `J_PROP` | 9.761e-6 | kg*m^2 | identified (free coast) | prop 2026-07-30 | **no** |
| `J_BARE_FITTED` | 7.585e-6 | kg*m^2 | identified (free coast) | bare 2026-07-30 | **no** |
| `J_SHAPE_UNCERTAINTY_PCT` | 8.4 | % | measured | prop (two speeds) | yes — it IS the drag table's error |
| `L_PHASE_FITTED` | 1.296 | mH | **fitted** (brake phase, `disarm`) | bare | **OPEN** |
| `BRAKE_CUTOUT_RPM` | 670 | rpm | **fitted** (brake phase, `disarm`) | bare | **OPEN** |
| `DRAG_TABLE_PROP` | 5.90 -> 4.16 | mN*m | measured | prop | **no — and it is not prop drag.** Also the `duty6` reference drag |
| `FREE_DECEL_BARE/PROP_RPM_S` | 5283 / 4089 | rpm/s | measured (`disarm`, thrust-800) | each | no |
| `CUT_RELEASE_S` | 0.170 / 0.332 / 0.024 | s | measured | both (`disarm` only) | yes for `disarm`, assumed otherwise |
| `BRAKE_ENGAGE_S` | 0.137 / 0.113 / 0.072 | s | measured | both (`disarm` only) | yes for `disarm`, assumed otherwise |
| `BRAKE_PHASE_S` / `_REV` | 0.107/0.062/0.033 s, 1.18/0.51/0.62 rev | - | measured | bare | **data only, not a parameter** |
| `SINE_PULLOUT_NM` | 8.24 | mN*m | fitted | bare | no |
| `SINE_COHERENT_CMD_MIN` | 210 | cmd | measured | both | **yes** (controlled pair) |
| `SINE_SPREAD_TABLE_BARE` | 92..234 | rpm | measured | both | **yes** — it does NOT shrink with load |
| `DUTY_SLEW_UP_PER_S` / `DUTY_CROSSOVER_CLAMP` | 1.6002 / 0.1243 | duty/s, duty | fitted | prop | **OPEN** — both reach the prop drag table |
| `TELE_FLOOR_MECH` | 182 | mech rpm | measured | both | **yes** |
| `TELE_LIVE_S` | 0.109 | s | measured | bare | yes |
| `TORQUE_PER_CMD` | 0.68 | mN*m/cmd | measured | prop | **reporting only** |
| `STARTUP_OVERSHOOT_PCT` / `_T_PEAK_S` | 53.5 / 1.19 | %, s | measured | both | **yes** (controlled pair, under 1 pp) |
| `STARTUP_ABORT_TARGET_RPM` | 2400 | rpm | measured | both | **yes** — a CONTROL failure, not inertia |

`R_total` is pinned throughout, and nothing is fitted against a metric containing the two cut delays.

### How each unmeasured parameter is identified

- **`J` comes from the FREE-COAST window** — after the drive is removed and before the brake bites, the
  one phase with no ESC torque in it. `J = tau_drag(w_mid)/alpha_free`, straight off the drag table with
  nothing fitted: `4.197 mN*m / 5283 rpm/s = 7.585e-6` bare, `9.761e-6` prop-on, both from
  `--thrust 800 --cut disarm` so they are a controlled pair.
  **`disarm` only.** The four `disarm` runs read 5042–5436 rpm/s but `deadman`/`zero_held` read
  6074/6113 — 15 % faster for a phase that by definition has no ESC torque — and their free windows are
  much shorter (0.072/0.113 s vs 0.129–0.143 s), so their "free" phase is probably already partly braked.
  Averaging all six biases `J` ~5 % low.
- **`(L, BRAKE_CUTOUT_RPM)` come from the BRAKE PHASE** — revolutions and duration, both measured **from
  engagement**, on the four `disarm` decays: two features × four decays for two parameters.
- **`SINE_PULLOUT_NM`** from the open-loop down-step, bracketed by physics: it must exceed the drag it
  holds against (5.90 mN*m over the coherent band) or the measured synchronous lock could not exist, and
  sit below the sine stall torque `K_T*V*(45/255)/R = 44.9 mN*m`. It lands at 8.24.
- **The crossover pair** from the single `407 → 680` step: the plateau LEVEL pins the clamp, its DURATION
  pins the slew. They are separable from each other; what is missing is corroboration at any other command.

### The brake metric: revolutions and duration from ENGAGEMENT

Time-to-stop measured from `t = 0` is **~72 % fixed cut delay** (0.307 s of 0.406 s on `disarm`), so it
stays inside 13 % while the brake torque is out by 2×. That is not a hypothetical: it is how an earlier
version of this model certified a 2×-too-strong brake as "out of sample within 11 %". Halving `L`
(roughly doubling the brake torque here) moves time-to-stop ~1 % but brake-phase revolutions ~27 % — a
>10× sensitivity difference, asserted as that ratio in
`test_time_to_stop_from_zero_is_delay_dominated_and_says_so`. `plantid.py validate --brake` prints both.

`R_total`'s ±0.26 ohm band propagates into `L` (1.02–1.58 mH) but **not** into `J`: the free-coast window
has no electrical term. Note this `L` puts the brake peak at 41.0 mN*m at 1263 rpm, while the pointwise
deceleration residual puts it nearer 60 mN*m at ~1900 rpm. The trajectory fit uses the integral and is
far less noisy, so it wins — but the ~1.5× disagreement is the honest uncertainty on the torque scale.

**Never pair `J_PROP` with the bare-shaft brake set.** `J_BARE_FITTED`, `L_PHASE_FITTED`,
`BRAKE_CUTOUT_RPM` and the bare brake numbers are one set; a preset test enforces it.

## Bare shaft vs propeller — the controlled pair

Taken with the same tools, protocol and starting speeds (2026-07-30). **The propeller changes only
inertia and the free coast.**

| quantity | bare | prop | verdict |
|---|---|---|---|
| `J` | 7.585e-6 | 9.761e-6 | **+29 %, real** |
| free-coast decel | 5283 rpm/s | 4089 rpm/s | −23 % |
| seam (fine sweep, both legs) | cmd 525–530 | cmd 525–530 | identical |
| fold hysteresis | none resolvable | none resolvable | identical |
| `tele` below the seam | exactly 182.0 | exactly 182.0 | identical |
| sine worst spread (up/down) | 234 / 174 rpm | 238 / 211 rpm | no improvement |
| down-step through the seam | 1048→175, no stall 3/3 | 1054→176, no stall 3/3 | same |
| startup overshoot (rpm 1200) | 53/53/54 % | 53/53/54 % | identical |
| startup at rpm 2400 | ABORT_STALL t=2.35 s | ABORT_STALL t=2.37 s | **identical failure** |

Two expectations were falsified, and both are pinned by tests so they cannot quietly return:
the propeller's inertia does **not** help the rotor follow the forced field, and it does **not** fix the
closed-loop `rpm 2400` stall — so that failure is a control / crossover-gap problem, not inertia.

## Validation achieved

| check | criterion | achieved |
|---|---|---|
| prop-on static, cmd 543/612/680 | ≤ 4 % | +1.25 … +2.08 % |
| bare static, cmd 555/570/600, both legs | ≤ 3 % | +0.01 … +0.98 % |
| `step()` integrated to convergence vs `steady()` | must agree | **−0.000 %** (4 cmds × 2 configs, both directions) |
| nothing exceeds `KV*V*eff` | new | agrees with `MotorModel` to 1e-9 |
| seam cmd @36 / @34 | [510,525] / [470,545] | 514.7 / 481.8 |
| **brake phase, 4 bare `disarm` (in sample)** | — | dur −8.6…+7.7 %, rev −8.7…+11.4 % |
| **brake phase, 3 prop-on (OUT OF SAMPLE, nothing refitted)** | — | **dur −4.9…+4.9 %, rev −11.3…+12.9 %** |
| down-step half-time | 50 % | +1 % |
| `407→680` t95 / plateau | 25 % / 300–500 rpm | +12.1 % / 389–390 (meas 360–415) |
| `407→680` VAF | reported | 99.3 % |
| forced sine, cmd 270…510 | 30 % | 28.4 % |
| transferability (swap `drag`, compare `asdict` minus it) | identical | pass, `drag_ref` unmoved |

The brake law being fitted on bare `disarm` only is enforced structurally — `plantid.fit` asserts it
never sees prop-on data — so that out-of-sample row is real evidence the *structure* is right. It is
still one inertia perturbation on one motor: not a different machine, and not water. `L_PHASE_FITTED`
and `BRAKE_CUTOUT_RPM` stay tagged OPEN.

### Known misses, each asserted at its achieved value

1. `tele` fold sweep max error **33.8 rpm** vs a 25 rpm criterion — one row, cmd 545 mid-seam (mean 8.5).
2. `tele` startup run 2, **26.3 rpm** vs 5 — one row of 158, rotor at 208 rpm before BEMF lock.
3. `cross_up` guard **13.7 %** at cmd 680 vs a >15 % floor — the shift is a fixed *duty* offset, so it is
   relatively smallest at the largest duty (mean 30 %, max 45 %).
4. `deadman` / `zero_held` brake phase **+86 % / +259 %** — mode-dependent torque, not modelled by design.
5. `680 → 407` down-crossing **VAF 25.7 %, ~2× too slow** — one constant pull-out fits `600 → 480` and
   fails here, i.e. the sine pull-out grows with slip.
6. Brake torque scale disagrees ~1.5× between the trajectory fit and the pointwise residual.
7. `J` carries **±8.4 %**; everything fitted against it inherits that.

### Startup overshoot: an honest negative result

The plant *can* exceed a 1200 rpm target by 18–34 % with the shipped gains, where the old fitted-lag sim
could not overshoot at all — but it limit-cycles across the seam rather than settling, so an "overshoot %"
against that tail is meaningless and the tool prints `LIMIT CYCLE`. The hypothesis that the 182 rpm
telemetry lie *causes* the measured 53–56 % overshoot is **not supported**: removing both the floor and
the startup silence changes the result by 0.4 pp. What the model does reproduce from measured numbers is
the seam's incremental gain (186→259 rpm over cmd 515→522, then 259→1194 over 522→612) — the same gain
that makes `rpm 2400` abort. Nothing was tuned toward 53 %.

## Two gotchas that have already cost time

**`cross_up` is not inferable from a filename.** The tracked prop-on `350kv_postfix_sysid_raw.json` is
`cross_up=34`; the 2026-07-30 fixtures are 36. Git history looks like it says 36 for both — the
reconciliation is that **`esctool.py flash` wipes the ESC config and leaves no commit trace.** Proof of
the assignment: the two duty sweeps are offset exactly 30 command counts, the 36 sweep matches a
directly-read `cross_up=36` run (1073 vs 1064 rpm), and the postfix data matches the 34 sweep to 0.0 % at
cmd 680. `plant_fixtures.json` records each fixture's CAMPAIGN, because `cross_up` and `mech_config` are
independent axes and neither implies the other.

**The spin-up method is a confound.** Because the brake engages on a fixed delay, a lower starting speed
means a lower speed at engagement. The original prop-on coastdown used closed-loop `--rpm 2200` and the
bare one open-loop `--thrust 800`, which attributed ~1000 rpm of protocol difference to "the propeller".
Every coastdown fixture now carries `spin_up`; match it before comparing.

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
8. **Water.** No water data exists; the `drag` callable is the interface.
9. Prop inertia separately from rotor inertia.
10. The mechanism of the 182 rpm `tele` floor. 182 mech = 1274 eRPM, identical in both configurations at
    every cmd 60..450. A bidir-DShot encoding floor and the ESC reporting a forced-commutation intention
    are distinguishable only with a different-pole-pair motor.
11. The brake torque's dependence on cut mode. `deadman`/`zero_held` brake 2–3× harder than `disarm` from
    the same speed; the model fits `disarm` and carries the other two as data.
12. The forced-sine pull-out's dependence on slip (miss 5). A slip-dependent saturation needs a second
    coefficient and a second trace to identify it.
13. The crossover ramp's command scaling. Because `applied_duty` returns `min(slew, clamp)`, the ramp is
    visible ~80 ms and the clamp holds the remaining ~320 ms, so the plateau ends at
    `electrical_duty(cmd)/slew` — LATER for a LARGER command (0.18 s at cmd 560, 0.42 s at 680, 0.63 s at
    800), which is backwards for a BEMF-lock handoff. Only cmd 680 was measured, so this is behaviourally
    a command-scaled dead time plus a duty clamp and **the scaling is an untested prediction.**

## Re-measurement recipe

Each number is re-derived by exactly one command. **Re-apply the whole ESC config after every flash** —
`esctool.py flash` wipes it — and record the `cross_up` you are running.

| number(s) | command | notes |
|---|---|---|
| `cross_up` itself | `host/esctool.py dump` | read it; never assume it |
| `DUTY6_A/B`, `DUTY6_CROSSUP_K` | `host/duty_sweep.py 1 --cross-up <byte>`, twice at two `cross_up` | `--cross-up` is required and stamped into the CSV |
| `V_SUPPLY_*`, `EFF_350KV` | the same sweeps + a PSU reading | `eff` is the residual scale |
| `DRAG_TABLE_PROP` | `duty_sweep` + PSU current, `tau = K_T*I_dc` | re-read the warning above before believing it |
| both `J`s, `FREE_DECEL_*` | `host/coastdown.py 1 --thrust 800 --repeats 3`, then `host/plantid.py fit` | **`--thrust 800`, NOT `--rpm 2200`** — the spin-up sets where the brake engages. `disarm` only. Re-take BOTH configurations with the same protocol or the comparison is worthless |
| `CUT_RELEASE_S`, `BRAKE_ENGAGE_S` | `coastdown.py 1 --thrust 800 --cut {disarm,deadman,zero}` | `--analyze` accepts the `mode` column |
| `J_SHAPE_UNCERTAINTY_PCT` | the same rotor at two starting speeds | the spread between the two `J` estimates IS the drag table's shape error |
| `L_PHASE_FITTED`, `BRAKE_CUTOUT_RPM` | the brake phase of the `disarm` decays, then `plantid.py fit` | from ENGAGEMENT; **check `L` with an LCR meter** |
| `BRAKE_PHASE_S` / `_REV` | all three `--cut` modes | data, not parameters |
| `TELE_FLOOR_MECH`, `SINE_*` | `host/sinesweep.py 1 --updown` | encoder-only; the AS5600 must be fitted |
| the seam brackets | `host/sinesweep.py 1 --cmds 495,505,510,515,520,...` | the fine fold sweep |
| `SINE_PULLOUT_NM` | `host/steptrace.py 1 --mode thrust --from 600 --to 480` | do NOT decimate (half-time ~24 ms). Also record `--from 680 --to 407`, where one constant fails |
| the crossover pair | `host/sysid.py` (the `407 → 680` step) | needs a second command to corroborate |
| `STARTUP_*` | `host/steptrace.py 1 --mode rpm --to 1200 --startup` | also `--to 2400` for the abort. Identical in both configurations, so a change here means the FIRMWARE or the crossover moved, not the rotor |
