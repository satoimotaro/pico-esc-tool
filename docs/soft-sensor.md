# ESC soft-sensor — supply voltage + relative load from eRPM (no extra sensors)

The ESC already streams **mechanical RPM** over bidirectional DShot. Combined with a motor model
(KV, pole-pairs) and the commanded throttle, that RPM lets you infer two things the board has **no
dedicated sensor** for:

- **effective supply voltage** (`vest`) — a rough battery-voltage / brown-out signal, and
- a **relative load / thrust** signal (`load`) — grows with current → torque → thrust, useful for
  detecting a fouled prop, entanglement, or water-flow load with no force or current sensor.

It is BlueGill/Bluejay-independent in principle, but needs the motor's **6-step (BEMF) regime** to be
live (the forced-sine low-speed region reports a *virtual* eRPM that can't be inverted).

## Principle

In 6-step the firmware runs at a duty set by the throttle command, and the motor turns at
`rpm = KV · V_supply · eff · duty(cmd)`. Invert it:

```
V_eff = rpm / (KV · duty(cmd)) = V_supply − I·R / duty
```

- At **no / light load** (`I≈0`) → `V_eff ≈ V_supply`.
- Under load the `I·R/duty` term grows, so **`V_eff ≤ V_supply` always** — a single reading sits
  *below* the supply by the load droop.
- `duty(cmd)` is a universal firmware/DShot shape (`DUTY6_A·cmd + DUTY6_B`, see
  `lib/vel_control/motor_model.h`); `eff` lumps KV error + duty-model error + the light-load IR and is
  removed by a one-time calibration (below).

## Prerequisites

1. Tell the ESC its motor so the model is right:
   ```
   motor <i> <kv> <pp> <v>        # e.g. motor 1 350 7 11.1  (must disarm first; validated kv>0 pp>=1 v>0)
   ```
2. Spin it **in 6-step** (a real RPM target, closed loop): `rpm <i> <target>`. Anywhere outside the
   validity window below — and that is stricter than "above the crossover" — the sample is invalid
   (`valid=0`, with `why=` naming the condition).

## The `sense` command

```
sense <i>            -> sense|<i>|vest=<V>|vref=<V>|load=<V>|vbatt=<V>|valid=<0|1>|floor=<rpm>
                        |dhat=<rpm>|settled=<0..1>|trim=<cmd units>|cmd=<int>
                        |why=<str>|cmdfloor=<cmd>|loadok=<0|1>|vbok=<0|1>
                        |temp=<C>|tpeak=<C>|tlim=<0..1>|tstate=<str>
sense <i> zero       -> baseline the reference at the current vest, and record the command it was
                        taken at (refused unless the validity window is open — `why=` says why not)
sense <i> vcal <V>   -> calibrate vbatt to a known battery voltage (needs the light-load tier, `vbok`)
```

New keys are always **appended**; the prefix and the existing key order are a contract with the host
tools (`host/driftsoak.py`, `host/vbatt.py`, `host/loadtest.py`), pinned by
`host/tests/test_sense_line.py`.

| field   | meaning |
|---------|---------|
| `vest`  | low-passed `V_eff` estimate (gated to the validity window). ≈ supply at light load, *before* calibration it may be biased (see below). |
| `vref`  | the **learned reference**, captured by `sense <i> zero`. `0` until you baseline it. |
| `load`  | `vref − vest` — `0` until baselined, `0` when `loadok=0`, `≈0` at the zeroing condition, and **grows positive under load** (droop). |
| `valid` | `1` only inside the validity window below; `0` in forced sine / startup / stopped / saturation. |
| `why`   | which condition is closing the window: `ok`, `notlive`, `cmdlow`, `dutyhigh`, `rpmlow`, `settling`. |
| `cmdfloor` | the `\|command\|` needed to open the window, **derived** from the configured seam (not typed). |
| `loadok` | `load` is comparable to the point `sense zero` was taken at (see the window below). |
| `vbok`   | this sample is in the light-load subset the `vbatt`/`vcal` tier is built on. |
| `temp` / `tpeak` | last / peak **ESC MCU** temperature (EDT). Not the winding — see below. |
| `tlim` / `tstate` | the firmware thermal limiter's command-ceiling scale and state (`none`/`ok`/`derate`/`stale`) — **[docs/velocity-control.md § Thermal protection](velocity-control.md#thermal-protection-firmware-velvelocitycontroller)**. |

Firmware API (per `Thruster`, `src/apps/thruster.h`): `senseVoltage()`, `senseLoad()`, `senseVref()`,
`senseValid()`, `senseZero()`, `senseLoadOk()`, `senseVbattOk()`, `senseWhy()`, `senseCmdFloor()`.
`arm()` resets the reference **and** the window, so each run re-baselines; `stop()` resets the window
(a motionless motor must never report a stale `vest` as valid).

## The validity window (what `valid=1` actually promises)

The old gate was `vc.live() && |tele rpm| ≥ lineFloor()` — **both terms read from telemetry**. Below the
seam the ESC *synthesises* an eRPM: on 2026-07-30 the bench measured a constant virtual **182 rpm
reported for every command from 60 to 450** while the rotor really went 39 → 170 rpm, and telemetry
only became honest from ~cmd 520 (cmd 530: tele 319 vs encoder 316; cmd 540: 436 vs 431). A gate that
reads that fiction cannot detect its own invalidity, so `sense` printed a number the caller could not
tell apart from a good one.

So the window is keyed on the **command we chose**, not on the telemetry we are trying to qualify
(`lib/vel_control/sense_gate.h`, natively tested in `test/test_sense_gate.cpp`):

| condition | threshold | why that number |
|---|---|---|
| command floor | `\|cmd\| ≥ 1.05 × seam_erpm/2499 × 1000` (enter), `× 1.02` (stay) | in forced sine `mech = cmd/1000 × SINE_FULLSCALE_ERPM/pp`, so the handoff **command** is a firmware property and the pole pairs cancel. The seam is `max(1333, sine_cross_up × 39.0625)` — **derived from the configured byte**, so `crossup <i> <byte>` moves the floor with it. At the bench's `cross_up = 36` (seam 1406 eRPM, handoff command 563) → **591 enter / 574 stay**, vs the measured honest onset of ~520. Not larger, because the `loadtest` point (1200 rpm ⇒ cmd ≈ 614) must stay inside: 614/563 = 1.091, so 5 % fits with 3.8 % to spare and 9 % would have excluded it. |
| duty ceiling | `duty6(cmd) ≤ 0.90` | the duty map is an **affine fit** to a modulator that cannot exceed full modulation, so near duty 1 it is extrapolating past where it was measured and `rpm6step` saturates against `noLoadTop` — `vest` would be inverting a duty the ESC is not delivering (≈ cmd 837 at `cross_up = 36`, above every operating point in use). |
| rpm floor | the profile's own 6-step landing (`floor=`) | secondary: catches "commanded high, rotor never caught". |
| dwell | 0.30 s continuously in-window | longer than the 130 ms worst-case 6-step plant tau from sysid, so the estimate is taken settled. **Leaving the window invalidates on the same tick** (fail-safe direction). |
| hysteresis | enter 1.05, stay 1.02 | a command dithering at the floor must not chatter the flag — every transition re-seeds the `vest` filter. |

Two sub-tiers on top, deliberately **separate flags** rather than extra terms of `valid` — folding them
in would make `load` unavailable exactly when a load appears, which is the sensor's whole product:

- **`loadok`** — `|duty6(cmd) − duty6(zerocmd)| ≤ 0.10`. `load` is a *difference* against the zero
  point, and the residual deficit is operating-point dependent, so a reading taken far away is two
  different biases subtracted. Outside the band `load` reports `0`, not a wrong number.
  The band is in **duty, not command units**, for three reasons: duty is what the deficit scales with;
  the duty map's large negative intercept makes a *percentage of the command* a wildly different
  fraction of the duty depending where you are (±15 % of cmd 652 was duty 0.15…0.67 ≈ 590…2600 rpm —
  nearly the whole envelope, at which width the flag degenerates into "a zero point exists"); and a
  *difference* of two duties is invariant to `cross_up`, which shifts both by the same constant.
  0.10 is bounded from both sides. **Floor:** `load` must survive load — holding rpm through a droop
  `d` costs `duty × 1/(1−d)`, so the 10 % droop that closes `vbok` moves duty 0.411 → 0.457, i.e.
  **0.046**; a tighter band than that would close `loadok` exactly when a load appears. (In command
  units that droop is only ~17 counts / 2.6 % — which is *why* `load` survives load.) **Ceiling:** it
  must still mean "the same operating point" — 0.10 duty is ±38 counts ≈ **±390 rpm**, so the
  driftsoak point stays comparable to itself while a deliberate move to 1200 rpm does not.
- **`vbok`** — droop `≤ 0.10 × vref` (light load). `vcal` is a *multiplicative* scale, so it can only
  cancel the residual `I·R/duty` deficit near the operating point it was taken at; restricting the
  rolling max to light load is what makes "`V_eff` → `V_supply`" true by construction. With no `vref`
  captured yet the rolling max is itself the light-load selector, so the tier stays open.

**This trades availability for trustworthiness on purpose.** `sense` now reports `valid=0` in cases
that used to print a number — that *is* the change. `why=` exists so that a closed window is never
mistaken for a regression, and so a failed lock says which condition blocked it.

None of these thresholds is bench-validated (they were chosen with no hardware available). They
live as named constants in one place (`sense_gate.h`) for exactly that reason.

## Load / thrust sensing (relative)

1. Spin steady at a known light-load point, `sense <i> zero` → captures `vref`.
2. Read `sense <i>` while operating: `load` rises as current (→ torque → thrust) increases; a sudden
   jump = added load (fouled prop, entanglement, water-flow). It is **relative**, not calibrated force.

Bench check (F2838 350KV): `load ≈ 0.00 V` at no load after `sense zero`, going positive under real
draw. (A single steady point can't separate V from load; the *change* in `load` is what's meaningful.)

### `load` and `dhat` are relative to the CALIBRATION CONDITION — not to physics

There is no absolute-torque path here, and the reason is measured, not stylistic. The 4-wire DC
measurement put `R_total = 1.20 ± 0.26 Ω`, split **winding 0.79 Ω + connectors/wiring 0.41 Ω** — i.e.
**~34 % of the resistance the `I·R/duty` term is built on is wiring**. That share is *per installation*
(cable length, crimps, connector count) and it **corrodes**, which on a saltwater ROV is a drift with
no schedule. Any "newton-metres from `vest`" claim would therefore have been a claim about one
particular loom on one particular day, so absolute torque was abandoned deliberately.

What survives that, by construction, is **in-situ calibration**: run `sense <i> zero` (and `sense <i>
vcal`) **in the water, in the installed configuration**, and the installation bias is absorbed into the
reference itself. Everything downstream is then a *difference against that reference* — which is
exactly why `loadok` restricts the reading to a band around the command it was captured at. Model error
after the `cross_up` duty fix is ~1 %; the residual is the no-load `I·R/duty` deficit, and it is
**operating-point dependent and flips worst case between air (worst at low rpm) and water (worst at
high rpm)** — so the same reference is *not* transferable between the two.

## Supply-voltage / battery estimate

Because `V_eff ≤ V_supply`, the **maximum** (or a high percentile) of `V_eff` over gated, steady
samples approaches the true supply — the lightest-load / highest-duty moment. Higher rpm/duty shrinks
the `I·R/duty` droop, so high-command samples are closest.

**Calibration is per motor+config — NOT per battery, and NOT per run *provided you persist it*.** The
bias is a linear scale, so one known-voltage point fixes it; on the firmware that scale lives in RAM
until you write it to the Pico's flash:

```
sense 1 vcal 11.2      # set the scale from a known battery voltage (needs a live vbatt estimate)
cfg save               # persist it (with the motor identity, gains and DOB tunables) — otherwise the
                       # calibration is lost on the next reset and `vbatt` silently reads uncalibrated
```

The host prototype stores its own scale in a file instead:

```
python host/vbatt.py calibrate --known-v 11.2     # spins, captures gated high-percentile V_eff,
                                                  # stores scale = known_V / V_eff  (host/vbatt_cal.json)
python host/vbatt.py monitor                      # reports V_batt = scale · rolling-high-percentile(V_eff)
```

Gating (rejects garbage / the crossover region): on the firmware the rolling max only advances while
`valid=1` **and** `vbok=1` (the light-load tier) **and** the rpm is a margin above the 6-step landing —
which is also why `sense <i> vcal` is refused until that tier has produced an estimate. The host
prototype uses `valid==1` + mech rpm above a floor + a steady setpoint. 1/5 Hz sampling is plenty for a
battery gauge.

Bench result (real supply 11.2 V): raw `V_eff.p90 = 9.14 V` → `scale = 1.2254` → `monitor` reports
**11.30 V (~1 %)**. Enough for a rough remaining-charge gauge / low-voltage warning. The voltage→%
mapping is a separate, per-chemistry table (not provided here).

## `dhat` — the disturbance observer's load estimate

`sense` also reports `dhat`, the DOB's estimate of the load in **rpm** (`rpmFor(last command) −
measured`, low-passed): the same physical quantity as `load` in different units, and the one an upper
layer (attitude / thrust allocation) actually wants per thruster. It is 0 while the observer is off
(`gain <i> dob 0`) or gated — outside solid 6-step, or while the setpoint is still slewing (`settled`
reports that gate, 0…1).

**`dhat` is only load if the feed-forward curve is accurate**, because the observer inverts that curve;
otherwise it reports the model error instead. Bench, 350KV at 1600 rpm with no load:

| feed-forward curve | mean `dhat` |
|---|---|
| measured (`curvepush`, matched `cross_up`) | **−15 rpm** (10 % of `dob_max`) |
| parametric (`motor`), wrong at that `cross_up` | **+325 rpm** (217 % of `dob_max` — clamped) |

So push a measured curve before trusting `dhat` (see the README's `curvepush` note, especially the
`sine_cross_up` dependency). The `dob_max` clamp exists precisely so a bad curve cannot over-drive.

**The DOB has never been tested against a real mechanical load.** Every number quoted for it comes from
a **synthetic, injected** disturbance: `host/dob_bench.py` adds a transient offset to the ESC command
that the loop does not know about ("No external load rig: we inject an INPUT disturbance… runs PI+FF
then PI+FF+DOB against the SAME injected disturbance"), and the native sim
(`lib/vel_control/test/test_vel_control.cpp`) models load as a **constant additive rpm deficit** —
precisely the disturbance shape the observer assumes. Those results confirm the *implementation*, not
the physics: a real propeller load is ~rpm², which **tilts** the curve rather than offsetting it, making
`d_hat` speed-dependent. The only load-ish hardware number on record is the bench 1650 → 1596 rpm at a
1600 setpoint. Treat `dhat` as a *relative* load channel (same caveat as `load`, above) until a real
load rig has been run.

## `trim` / `cmd` — the control effort, i.e. the drift channel

A closed velocity loop is **blind to plant drift by design**: as the winding heats (R +0.393 %/°C) or
the magnets weaken (KV +0.11 %/°C), the loop simply works harder and `rpm` keeps reporting the target.
The effort is where the drift is visible.

| field  | meaning |
|--------|---------|
| `trim` | the blended PI correction actually added to the feed-forward, in **ESC command units** — the same units as `ff` and `cmd` (thrust, −1000…1000), *not* rpm (`vc.trim()`; signed, bounded by `gain <i> trim` = `trim_max`). |
| `cmd`  | the command actually sent, `= clamp(ff(setpoint) + trim + dob_comp)` (`vc.command()`). Logged so a drift trace is self-contained — no need to re-evaluate the FF curve offline. |

To read `trim` as rpm, divide by the curve slope at that operating point: on the 350KV bench the
6-step segment runs ≈**10.3 rpm per command unit**, so 1 unit of `trim` ≈ 10 rpm of correction.
(`trim_max`, `dob_max` and `dob`'s internal `comp` are all in these same command units — `comp`
divides `d_hat` by `rpmSlopeAt()` precisely to get there.)

At a **fixed setpoint**, `trim` walking away from its cold value is the drift signal; `rpm` will not
move until `trim` saturates at `trim_max`. Rough scale on the 350KV bench: a 40 °C rise is worth
≈70 rpm at 1600 rpm, i.e. only ≈**7 command units** of `trim` against a `trim_max` of 80 — small, but
well clear of saturation and resolvable at the 0.1-unit print precision.

Caveat: `trim` responds to **load** exactly as it responds to drift. Interpreting it as drift needs a
benign reference condition (a known setpoint at known low load), or — on a multi-thruster vehicle —
cross-thruster comparison, where common-mode is the environment and an outlier is the unit.

## Offline analysis — `host/softsensor.py`

Fit `V_eff` and a load-droop trend from a `(cmd, rpm)` sweep (e.g. a `sysid` curve or a `cmd_measure`
run): `v_eff` (least-squares supply), per-point V, an `rpm²` load coefficient, and a `load_present`
flag. Useful to characterise a motor offline; the firmware `sense` is the live version.

### `--cross-up` is REQUIRED (2026-07-30)

`softsensor.duty(cmd, cross_up)` and `softsensor.estimate(kv, pp, samples, cross_up)` have **no
default** for `cross_up`, and calling them without it raises `TypeError`. `host/obschar.py`,
`host/duty_sweep.py`, `host/softsensor.py` and `host/sim_obs.py` all take `--cross-up` on the CLI.

The default used to be 34 (the value `DUTY6_A/B` were fitted at) while the bench ran 36. That is a
fixed **duty** offset of `0.0421 × 2`, so it is a ~0.6 % error in `V_eff` at cmd 780 and a ~44 % error
at cmd 570 — and because `estimate()` charges every unexplained deficit to *load*, the droop and
`k_load` numbers silently absorbed it as if it were physics. `host/sim_obs.py` imported `duty` bare and
so validated a model of a **different ESC** than the hardware it was compared against. A wrong
`cross_up` costs 13.7–45.0 % on the static curve (`python3 host/plantid.py validate --static`), so the
argument is required rather than defaulted: a `TypeError` is a better answer than a plausible number.

Read the byte with `python3 host/esctool.py dump`; never assume it, and re-read it after every flash
(`esctool.py flash` wipes the config to firmware defaults).

## The rotor NEVER coasts — zero throttle is a BRAKE

This is not a soft-sensor property, but it governs every "sample opportunistically while it spins"
assumption below, so it belongs with them. Measured on the bare shaft (no propeller), commit `5c205b2`:

- **3100 rpm → dead stop in 0.19–0.54 s.** A free rotor of this inertia would take many seconds.
- The deceleration **PEAKS PART-WAY DOWN and then falls away again**: ~3500–5000 rpm/s above 2500 rpm,
  peaking at ~48 000 rpm/s at **1455–1596 rpm** (3/3, ±5 %), back to ~4000 rpm/s below 500 rpm. The
  *rise* alone already rules out passive drag — aerodynamic (∝ω²), viscous (∝ω) and Coulomb (constant)
  are all flat or falling and none can grow 10× while the speed collapses. The *roll-off* on the far
  side additionally rules out a constant brake: a peak with fall-off on both sides is the signature of
  a **slip-dependent** torque (short-circuit / asynchronous braking, which peaks near ω_e = R/L).
  **RESOLVED 2026-07-30** (`docs/measured-plant.md`): the peak location never moved. The brake
  ENGAGES on a fixed delay — the ESC keeps driving for 0.17 s (`disarm`), the rotor then coasts on drag
  alone for a further 0.13 s, and only then does the brake bite. Bare-shaft the rotor is at ~2390 rpm by
  then and prop-on at ~1540 rpm, which is exactly the ratio the "moving peak" (1455–1596 vs 865–973)
  showed. Both delays were measured in **both** configurations and agree within 5 %, so they are ESC
  state-machine timings. **Re-confirmed a third time on 2026-07-30 evening**, propeller refitted and the
  whole set re-taken with the same `--thrust 800` spin-up: engagement moves to 2531–2741 rpm, i.e.
  *higher* than bare-shaft, because the free coast is slower. Engagement tracks the STARTING speed, not
  the load — and the old 865–973 rpm prop-on figure came from a lower-starting closed-loop spin-up, not
  from the propeller. The brake **torque** transfers well on that protocol-matched pair (brake-phase
  duration −5.9…+3.3 % across a +29 % inertia change, nothing refitted), but `L` stays tagged OPEN: one
  inertia change on the same motor is not a different machine and is not water.
- **Identical via all three routes**: `disarm`, letting the 500 ms deadman expire, and holding a zero
  target while armed. There is no "coast" path among them.
- **`brake_on_stop = 0` does not disable it.** The braking is not that setting.

Operational consequence: **cutting thrust brakes a thruster, it does not windmill.** For an ROV that
means no free-spinning drag term to plan around when a thruster is idled — and, for this document, that
the "spinning" window from which the soft-sensor can sample ends abruptly rather than tapering, so
there is no slow-coast tail to sample during. It also means the last `sense` reading before a stop is
the last one you get; there is no post-command coast-down to observe.

## Limitations & notes

- **Only inside the validity window.** Stopped / forced-sine / near-crossover / duty-saturated /
  within the 0.3 s dwell → `valid=0`, with `why=` naming the condition. In an ROV, sample
  opportunistically whenever a thruster is running **above the command floor** — and note that the
  window is *deliberately* narrower than "the motor is spinning".
- **Absolute voltage needs the calibration** (and `cfg save` to survive a reset); raw `vest` is biased (~18 % low on the bench
  350KV). Relative trends (load, battery draining) work even uncalibrated.
- **Can't separate V from load at a single point** — that's why voltage uses *max-over-time* (lightest
  load) and load uses *deviation from a learned no-load baseline*. Time-multiplex if you need both.
- Winding temperature moves `R`; the max-at-light-load / high-duty selection keeps the `I·R/duty` term
  small, so temperature sensitivity stays low.
- `host/vbatt.py` is a **host prototype**; a lean rolling-max version can move into firmware later
  (expose `vbatt` from `sense`) for host-independent, always-on monitoring.
