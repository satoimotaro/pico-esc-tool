# Differential azimuth thruster — bring-up log

One pod: two F2838 BLDCs into a differential, wired to **GP10 (ESC index 0)** and **GP11 (index 1)**.
The upper cylinder is the azimuth axis and carries the propeller inside it. Both motors are fixed to
the pod body — there is no azimuth servo, the differential *is* the steering actuator
(`/srv/share/書類/diff_azimuth_auv.md`: F2838 ×6 → 差動アジマスポッド ×3, 350KV).

Snapshots and evidence live in the `bench-registry` repo (observations belong outside this one).

---

## The two modes, and what they imply

Per the operator, with ω0/ω1 the two signed motor speeds:

| commanded | result |
|---|---|
| ω0 = −ω1 (counter-rotating, equal speed) | azimuth **held**, propeller turns → pure thrust |
| ω0 = ω1 (same direction) | azimuth axis rotates → steering |

So the differential decouples into a **sum** channel and a **difference** channel:

```
s = ω0 + ω1   ->  azimuth rate     ψ̇ = s / (2·N_a)
d = ω0 − ω1   ->  propeller speed  ω_p = d / (2·N_p)      inverse:  ω0 = (s+d)/2 ,  ω1 = (s−d)/2
```

`N_a`, `N_p` and **the signs** are not known yet and must be identified on the bench, not assumed —
the design memo describes the same mechanism as "2モーターの**速度差**でアジマス角を作る", which is
the opposite convention to the operator's. Both descriptions are self-consistent under a mirrored
bevel, and picking wrong swaps steering for thrust. §3 below identifies it directly.

**Two consequences worth stating before any tuning:**

- **"Hold angle while thrusting" is not an open-loop counter-rotation.** Nominal ω0 = −ω1 is the
  *feedforward*; two different motors on two different ESCs under different loads will never be
  exactly equal, and any mismatch integrates straight into azimuth drift. The azimuth loop has to
  stay closed and keep trimming `s` around zero while `d` is large. Open-loop counter-rotation will
  slowly rotate the pod.
- **Propeller speed is relative to the carrier.** While the azimuth is slewing, `d` no longer maps to
  absolute prop speed. Holding is clean; slewing-while-thrusting cross-couples by construction.

## Control architecture — why the encoder placement is the whole design

The encoder is **one AS5600 reading the azimuth axis directly**. That is the right place for it, and
it is what makes this tractable:

- **Sum channel (azimuth) — closed loop on the AS5600.** Azimuth positioning is a *low-speed
  position* problem, which is exactly where this stack's sensorless feedback is worthless: DShot
  telemetry has a hard floor (a constant 182 rpm below the seam on this bench) and honestly reports
  `tele = 0` throughout forced sine. Integrating that to get an angle would drift without bound. A
  direct azimuth encoder removes the dependency entirely — the sum channel never needs telemetry.
- **Difference channel (propeller) — open loop / eRPM.** The prop runs high, in the 6-step region
  where eRPM telemetry is honest. The existing `velctl` path applies unchanged.

`host/posctl.py` already implements the control law we need — cascade PID, outer position → velocity
setpoint, inner `Kff·vel_sp + Ki·∫err` → signed `thrust`, with detent hold inside tolerance. What it
does **not** have is the mixer: today it drives one ESC and reads the AS5600 on *that same rotor*.
Here the controller output is `s`, which must be split across both ESCs, and `enc` reads the azimuth
rather than a rotor. **That is a mixer layer on top of posctl, not a new controller.**

Note posctl is written for BlueGill **S1 forced-commutation** mode, where the ESC creeps far below
the 6-step floor and holds with detent torque at thrust 0 — that hold is what makes a static angle
hold work without pulsing. ch1 is currently `sine_mode: 2` (S2). S1 vs S2 for the azimuth channel is
an open decision: S1 buys the detent hold, S2 buys resolution. Quoted hold resolution is ~one detent
(12N14P ⇒ ~8.6° at the rotor), which the azimuth reduction `N_a` divides down.

---

## Session 2026-08-11 — pre-spin audit. **Nothing was written; nothing was spun.**

### Confirmed good

| check | result |
|---|---|
| Pico link (`/dev/ttyACM0`, VID 2E8A) | `id esc_tool v1` |
| ESC signal pins | `pins 2 10 11` — two channels, as wired |
| Both ESCs powered + talking 1-wire | idx 0 and idx 1, sig `E8B2`, layout `#A_H_30#`, fw `0.21` |
| Pico runtime config persisted | `cfg\|stored=1`, both channels `kv=350 pp=7` — matches the F2838 350KV in the memo |
| ch1 (GP11) ESC EEPROM | **byte-identical** to `20260801T103254Z-verify-after-flash-defaults-experiment` (BlueGill S4) |

### Blocker 1 — ch0 (GP10) is at post-flash defaults

`bench.py diff` ch0 vs ch1 → **16 settings differ**, including all four this bench has already
measured one at a time as start-killers:

| setting | ch0 | ch1 | measured effect of the ch0 value |
|---|---|---|---|
| `startup_power_max` | **5** | 25 | **0 rpm — will not start at all** |
| `low_rpm_power_protection` | **9** | 0 | 380 rpm (a quarter) |
| `sine_cross_up` | **255** | 36 | 230 rpm — handoff refused, never leaves forced sine |
| `motor_direction` | **Normal** | Bidirectional | 180 rpm |

ch0 has never been configured. **In a differential this is worse than a dead channel.** With ch0 not
starting, a commanded pure-thrust (ω0 = −ω1) delivers ω1 only, so `s ≠ 0` and **the pod steers when
it was told to hold**. That reads as a mechanical fault in the differential and it is a config fault.

> Every pre-spin guard here is per-channel: `bench.py check` compares one live ESC to one snapshot.
> Nothing checks the two channels of one pod against **each other**. On a single-motor bench that
> check could not fail, so it does not exist. For a differential it is the first thing to run.

The fix is to mirror ch1's snapshot onto index 0 and verify byte-for-byte. Attempted; the harness
**denied the write** as a hardware-mutating action. Needs an explicit go-ahead.

### Blocker 2 — the azimuth AS5600 is not on the bus. **The firmware was audited and is not at fault.**

`enc`, `encv`, `enc raw|md|mag|agc` all return `err no-encoder`. The code path was checked end to end:

- `enc_.begin()` runs in `EscTool::begin()` (`esc_tool_app.h:165`), after `LittleFS.begin()`, before
  anything can contend for the bus.
- `enc_.poll()` runs in `EscTool::poll()` (`:209`) **unconditionally, every loop**, on the same core
  (core0) that called `begin()` — not gated behind SETUP/DRIVE mode, not behind arming.
- `As5600Tracker::poll()` re-reads RAW_ANGLE at ~1 kHz and sets `present_ = true` on **any** successful
  read (`as5600.h:61`), so a hot-plug recovers within a millisecond.
- GP16 (SDA) / GP17 (SCL) is a valid I2C0 pair on RP2040 and has never been anything else — `git log -S`
  finds the assignment unchanged since the encoder was introduced.

So this is **not** boot order, not mode gating, and not a stale pin constant. `poll()` is retrying a
thousand times a second and failing every time: the sensor is not ACKing on I2C. Check, in order:
3V3/GND at the sensor; **SDA on GP16 = physical pin 21, SCL on GP17 = physical pin 22** — physical
pins 16/17 are GP12/GP13, which is the classic way this gets miswired; bus pull-ups; continuity of the
run into the pod; and whether the part is an AS5600**L** (configurable address — 0x36 is fixed only on
the plain AS5600).

Without it there is no azimuth feedback at all, and the sum channel is the half of this mechanism
that cannot be run open-loop.

### Correction

An earlier revision of this log recorded that the camera was not pointed at the thruster and that the
frame showed a desk lamp. **That was wrong** — the black T-shaped object centre-frame is the pod, the
upper cylinder being the azimuth axis. The camera framing is correct and that finding is withdrawn.
(The 2026-08-01 comparison frame really did show the PSU panel; the camera was re-aimed since, as
intended.) Live view from the tailnet: `https://tarossas-g10.tailc78d3b.ts.net/camera`.

### Standing hazard: the loom

The supply loop is a **measured 0.254 Ω**. Above ~1 A the PSU panel is not what the ESC sees (1.27 V
lost at 5 A). One bare-shaft motor stayed under that; **two motors turning a propeller will not**, and
the memo budgets ~10 A per motor. Replace the loom before any loaded run — every voltage-derived
number is wrong until then.

### State left on the hardware

As found. Both ESCs read and released (`esctool run`); no config applied, no channel armed, no motor
spun. Captured in `bench-registry/snapshots/20260811T041059Z-azimuth-ch0` and `…041108Z-azimuth-ch1`.

---

## Implemented while the bench is blocked

Experiments are stood down until the operator is back; the code is written and tested against the
sim so the bench session is spent on the mechanism, not on typing.

| new | what it is |
|---|---|
| `host/pico_esc/differential.py` | `DifferentialMixer` — the (s,d) ↔ (A,B) kinematics, saturation policy and sign identification. Pure, no I/O. Plus `DualDrive`, two `PosDrive` sessions armed and disarmed as one pod. |
| `host/azimuth.py` | CLI: `identify` / `move` / `hold`. Reuses `PositionController` **unchanged**. |
| `host/tests/test_differential.py` | 16 tests. Suite is 380 passed / 5 xfailed, no regressions. |

Three things are deliberately encoded rather than left to the bench:

- **Saturation sheds `d`, never `s`.** Clamping A and B independently would distort *both* channels —
  a saturating turn would inject thrust it was never asked for. Losing angle hold points the thruster
  somewhere unintended; losing thrust just slows the vehicle. `test_saturation_never_injects_thrust_
  into_a_turn` pins this.
- **`DualDrive` cannot arm one channel.** A pod with one channel live is the state that steers when
  told to hold. Identification, which genuinely needs one motor at a time, uses `PosDrive` directly.
- **`identify()` raises on a near-zero rate** instead of returning a sign. A sign inferred from noise
  swaps steering and thrust, and would look like a mechanical fault.

`--dry-run` exercises the code path, it does not model the pod: `SimEncEscHost` is one rotor with one
encoder, so there is no differential in the loop and no second motor to mismatch. The mixer kinematics
are covered properly by the unit tests instead.

## Bench order, once the two blockers clear

1. **Mirror ch0 from ch1's snapshot**, verify byte-for-byte, re-diff ch0 vs ch1 → require zero drift.
   Never from a checked-in profile: profiles are intentions, and one had already drifted here in
   eight settings.
2. **Encoder sanity**: `enc` returns a live angle; turn the azimuth by hand and watch `encv` accumulate
   monotonically with the right sign. Establish the azimuth zero.
3. **Identify the mixer — one motor at a time, lowest command, on camera.** Drive ch0 alone and
   measure the azimuth rate on the encoder → `a0`; then ch1 alone → `a1`.
   `a0 ≈ +a1` means azimuth ∝ sum (the operator's convention); `a0 ≈ −a1` means ∝ difference (the
   memo's). This also gives `N_a`, and it is the first run that produces a number.
4. **Azimuth PID with `d = 0`** — pure positioning, no thrust. Reuse posctl's law through the mixer.
   Gains do not transfer between motors or mechanisms; tune in situ.
5. **Hold-under-thrust last**: ramp `d` with the azimuth loop closed and measure angle drift against
   `d`. This is the mode that fails silently if the loop is left open.
