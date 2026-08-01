# Bringing up a new thruster, to the point of an in-water test

The order below is not arbitrary. Each step either produces something the next one needs, or is the
last chance to check something before the instrument that could check it is gone.

**The governing constraint: in the water there is no encoder.** The deployed system is sensorless —
DShot eRPM telemetry and nothing else. So every number the controller depends on must be obtainable
from KV, pole pairs, supply voltage and eRPM. The encoder is a *bench* instrument, it exists for
exactly one session, and anything you did not verify with it while it was attached is unverifiable
afterwards.

Two numbers drive everything: **KV** and **pole pairs**. `MotorModel(kv, pole_pairs, v_supply,
eff, cross_up)` takes nothing else, and the firmware's `motor <i> <kv> <pp> <v> [cross_up]` takes
nothing else. Per-motor inertia, drag tables and winding inductance belong to the simulation layer
(`pico_esc.measured`, used only by `coastdown`/`plantid`/`sim_obs`) and are **not** on this path.

---

## 1. Get KV and pole pairs, and verify the pole pairs while you still can

Pole pairs = poles / 2, from the datasheet or by counting magnets (a 12N14P is 7). KV from the
nameplate is good enough to start — step 6 fits the residual scale.

**Verify the pole count on the bench, with the encoder, before the thruster goes near water.** The
firmware reports `tele.rpm` already divided by the *configured* pole pairs, so:

    real_pp = configured_pp * tele_rpm / enc_rpm

A wrong pole count scales every speed silently and there is no second opinion in the water. Expect
the ratio to land within ~1 % (0.5 % once BEMF-locked); anything else means the count is wrong, not
that the encoder is noisy.

## 2. Choose `sine_cross_up`

This picks where the ESC hands off from forced sine to 6-step. The handoff fires at a **minimum-
modulation duty floor**, not at a speed, so `handoff_cmd()` is the same command on any KV:

| `sine_cross_up` | 34 | 36 | 40 | 44 |
|---|---|---|---|---|
| handoff command | 484 | 515 | 578 | 640 |

**The usable window is 34..47.**

- Below 34 the ESC **refuses to hand off** and stays in forced sine forever — measured, not assumed:
  cross_up 30 never handed off on either leg. `handoff_possible()` is the predicate.
- Above 47 (at the default `cmd_hi=700`) fewer than two 6-step points fit in the sampled range and
  `to_profile()` **raises** rather than emit a curve whose upper half ramps backwards. Left
  unguarded that silently mis-scales the controller, which is worse than an exception.

Prefer **34**, the lowest handoff, unless something specifically needs the seam higher: it gives the
smallest forced-sine region, and the forced-sine region is the one you can never calibrate in water.

## 3. Predict the profile and the ESC config

```
python3 host/motorpack.py --kv 300 --pp 7 --v 14.8 --name my_thruster predict
```

```
Motor my_thruster: KV=300 PP=7 V=14.8V eff=1.000
  no-load top   : 4440 rpm
  sine top      : 357 mech (PP-governed)
  handoff       : 173 mech (cmd 484) ; 6-step landing ~223 rpm ; gap 173-223
  6-step curve  : rpm = KV*V*eff*(0.00265*cmd -1.233 )
  ESC config    : cross_up=34 cross_dn=255 low_rpm_prot=0
```

It writes `host/profiles/<name>_parametric.yaml` and prints the exact `esctool set` line.

**Note the gap.** Between the sine top and the 6-step landing there is a band the motor cannot hold.
On the 930KV bench the gap *collapsed* onto the landing, so treat the printed gap as an upper bound
and do not plan a duty cycle that lives inside it.

## 4. The down-crossover ships DISABLED — decide it deliberately

`motorpack` emits `sine_cross_dn=255`. That is **≥ the firmware's 0xF0 disable threshold**: the ESC
stays in 6-step all the way down instead of returning to sine. This is deliberate — a seamless
down-handoff is motor-specific and claiming one from a parametric model would be an invention.

Enable it only after watching *this* motor decelerate through the seam on the bench:

```
python3 host/motorpack.py --kv 300 --pp 7 --v 14.8 --seamless-down predict   # -> sine_cross_dn=239
```

## 5. Apply the config, then snapshot it

```
python3 host/motorpack.py --kv 300 --pp 7 --v 14.8 apply
python3 bench.py snapshot my-thruster-initial --firmware "BlueGill S4" \
        --motor "<what it is>" --supply-v <measured, off the PSU panel>
```

`--supply-v` is the **measured** supply. The ESC's `vbatt` is a soft sensor, not a voltmeter.

`esctool set` refuses to write settings the ESC's EEPROM layout revision cannot take, so a
mismatched firmware fails loudly rather than writing at wrong offsets.

## 6. Never flash by hand again

```
python3 bench.py flash <image>.hex <label> --firmware "<name>"     # snapshot->flash->restore->verify->snapshot->diff
python3 bench.py check                                            # before the FIRST spin, every time
```

A flash resets **14 settings** to firmware defaults. This is not a theoretical hazard — measured on
the bench, one setting at a time, against a good config settling at 1600 rpm:

| post-flash default | result |
|---|---|
| `startup_power_max = 5` | **0 rpm — will not start at all** |
| `low_rpm_power_protection = 9` | 380 rpm |
| `sine_cross_up = 255` | 230 rpm (handoff refused; never leaves forced sine) |
| `motor_direction = Normal` | 180 rpm |

**Three of the four still start.** The dangerous outcome is not a dead motor, it is one running at a
quarter to a ninth of the right speed while looking healthy. `bench.py check` names the drift and
exits non-zero.

## 7. Dry bench run — spend the encoder here

```
python3 bench.py check                                    # config first, always
python3 host/startlog.py 1 --repeats 20 --kv 300 --v 14.8 --cross-up 34 \
        --journal <bench-registry>/measurements/startlog_journal.csv
```

Every attempt must come back `CAUGHT` at the predicted speed. A `SLOW` row means it started and is
running wrong — the mode that hides — and is classified against `KV*V*duty6(cmd, cross_up)`.

While the encoder is attached, also: verify the pole count (step 1), watch a deceleration through
the seam if you intend to enable `cross_dn`, and run `host/duty_sweep.py --r 0` if you want the duty
map confirmed on this motor. Analyse duty sweeps at **`--r 0`**; the I·R term is mis-derived at low
duty and over-corrects.

## 8. Self-calibrate on telemetry alone

```
python3 host/motorpack.py --kv 300 --pp 7 --v 14.8 --name my_thruster selfcal
```

Spins a few 6-step points, reads `tele` eRPM, fits `eff`, re-emits the profile. **No encoder.** This
is the step that transfers to water, and it is the one to re-run after the thruster is wet and
loaded — the parametric curve is a *bare-shaft, in-air* fit, and a propeller in water is a different
load by orders of magnitude.

## 9. Closed loop, last

- **Gains do not transfer between motors.** Tune in situ.
- **The FF curve is only valid at the `sine_cross_up` it was measured at** — moving `cross_up`
  shifts the whole command→speed mapping. `curvepush.py` reads the ESC and refuses on a mismatch.
- Do not trust closed-loop steady-state numbers taken while polling the CLI hard: at ~230 Hz the
  poll starves the control loop on core0 and reads ~3 % low. Open loop is immune.

## 10. Set the guards before the water — they are the only thing that stops it

| guard | default | note |
|---|---|---|
| `over_speed_rpm` | 0 → auto `max(2*maxRpm, 1200)` | debounced over **3 live frames**; one garbled frame once stopped a healthy thruster |
| `stall_secs` | 1.0 s | fires when commanding into 6-step but telemetry never goes live; owns the dropout case |
| `warn_temp` / `max_temp` | 0 / 0 = **off** | derate over the band, abort at `max_temp`. Both must be set for the derate. EDT temp is the ESC MCU, not the winding |
| `slew_rpm_s` | 200 | keeps the first command gentle; the main lever on startup overshoot |
| `stop_below_rpm` | 0 | a target at/below this is a real STOP — speed cannot be held sensorlessly near zero |

Set `warn_temp`/`max_temp` explicitly. They default to off, and they are deliberately **not**
persisted: a power cycle restores the conservative compile-time band.

## 11. What to capture in the water, so one session is enough

Assume you get one wet session. Record, per run: the command, `tele` eRPM, `sense` (`vest`, `load`,
`dhat`, `temp`, `tstate`), the controller status, and the **ESC config fingerprint** — `startlog`
stamps that on every row so a bad run can be told from a bad config afterwards.

Sweep commands across the whole 6-step range and hold each long enough to settle. Include at least
one deliberate stop and restart: startup is the failure mode with an open, unreproduced case
against it.

**Do not try to calibrate the sine region in water.** There is no encoder, and the ESC correctly
reports `tele = 0` throughout forced sine — the region is honestly unmeasurable there. Take it on
the bench or not at all.

---

## Standing rules

- **Never** restore from a checked-in profile. Profiles are intentions; snapshots are observations,
  and a profile had already drifted from this bench in eight settings.
- **Never** flash without `bench.py flash`, and never spin before `bench.py check`.
- The measured supply is the PSU panel, not `vbatt` — and the panel is not what the ESC sees once
  amps flow, if anything resistive sits between them.
