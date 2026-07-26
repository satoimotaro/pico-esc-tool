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
2. Spin it **in 6-step** (a real RPM target, closed loop): `rpm <i> <target>`. Below the sine↔6-step
   crossover the sample is invalid (`valid=0`).

## The `sense` command

```
sense <i>            -> sense|<i>|vest=<V>|vref=<V>|load=<V>|valid=<0|1>
sense <i> zero       -> baseline the no-load reference at the current vest (errors if not BEMF-live)
```

| field   | meaning |
|---------|---------|
| `vest`  | low-passed `V_eff` estimate (gated to live 6-step). ≈ supply at light load, *before* calibration it may be biased (see below). |
| `vref`  | the **learned no-load reference**, captured by `sense <i> zero`. `0` until you baseline it. |
| `load`  | `vref − vest` — `0` until baselined, `≈0` at no load, and **grows positive under load** (droop). |
| `valid` | `1` only while the loop is BEMF-live 6-step; `0` in forced sine / startup / stopped. |

Firmware API (per `Thruster`, `src/apps/thruster.h`): `senseVoltage()`, `senseLoad()`, `senseVref()`,
`senseValid()`, `senseZero()`. `arm()` resets the reference so each run re-baselines.

## Load / thrust sensing (relative)

1. Spin steady at a known light-load point, `sense <i> zero` → captures `vref`.
2. Read `sense <i>` while operating: `load` rises as current (→ torque → thrust) increases; a sudden
   jump = added load (fouled prop, entanglement, water-flow). It is **relative**, not calibrated force.

Bench check (F2838 350KV): `load ≈ 0.00 V` at no load after `sense zero`, going positive under real
draw. (A single steady point can't separate V from load; the *change* in `load` is what's meaningful.)

## Supply-voltage / battery estimate

Because `V_eff ≤ V_supply`, the **maximum** (or a high percentile) of `V_eff` over gated, steady
samples approaches the true supply — the lightest-load / highest-duty moment. Higher rpm/duty shrinks
the `I·R/duty` droop, so high-command samples are closest.

**Calibration is one-time, per motor+config — NOT per startup, NOT per battery.** The bias is a linear
scale; one known-voltage point fixes it forever:

```
python host/vbatt.py calibrate --known-v 11.2     # spins, captures gated high-percentile V_eff,
                                                  # stores scale = known_V / V_eff  (host/vbatt_cal.json)
python host/vbatt.py monitor                      # reports V_batt = scale · rolling-high-percentile(V_eff)
```

Gating (rejects garbage / the crossover region): `valid==1` + mech rpm above a floor + steady setpoint.
1/5 Hz sampling is plenty for a battery gauge.

Bench result (real supply 11.2 V): raw `V_eff.p90 = 9.14 V` → `scale = 1.2254` → `monitor` reports
**11.30 V (~1 %)**. Enough for a rough remaining-charge gauge / low-voltage warning. The voltage→%
mapping is a separate, per-chemistry table (not provided here).

## Offline analysis — `host/softsensor.py`

Fit `V_eff` and a load-droop trend from a `(cmd, rpm)` sweep (e.g. a `sysid` curve or a `cmd_measure`
run): `v_eff` (least-squares supply), per-point V, an `rpm²` load coefficient, and a `load_present`
flag. Useful to characterise a motor offline; the firmware `sense` is the live version.

## Limitations & notes

- **Only while spinning in live 6-step.** Stopped / forced-sine / near-crossover → `valid=0`. In an
  ROV, sample opportunistically whenever a thruster is running.
- **Absolute voltage needs the one-time calibration**; raw `vest` is biased (~18 % low on the bench
  350KV). Relative trends (load, battery draining) work even uncalibrated.
- **Can't separate V from load at a single point** — that's why voltage uses *max-over-time* (lightest
  load) and load uses *deviation from a learned no-load baseline*. Time-multiplex if you need both.
- Winding temperature moves `R`; the max-at-light-load / high-duty selection keeps the `I·R/duty` term
  small, so temperature sensitivity stays low.
- `host/vbatt.py` is a **host prototype**; a lean rolling-max version can move into firmware later
  (expose `vbatt` from `sense`) for host-independent, always-on monitoring.
