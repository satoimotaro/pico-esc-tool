# BlueGill velocity control & generalization — design spec

Status: DRAFT for review (2026-07-17). Turns the "don't make the ESC a per-motor special" goal into a
concrete layered design. Companion to `ESC-firmware/docs/sine-drive-design.md` and the current-state
memory. Terms: **mech RPM** = mechanical; **eRPM** = electrical (mech × pole_pairs); on this stack the
`tele` line is ALREADY mechanical (firmware divides eRPM by pole pairs).

## 1. Goals & principles
- **The ESC firmware is GENERIC and config-driven.** All motor specifics live in a **profile (config)**,
  never in hand-tuned firmware. Changing motors = run calibration → apply a profile, NOT edit/flash code.
- **The encoder (AS5600) is a CALIBRATION INSTRUMENT, not a runtime part.** Runtime is 100% config-driven
  and encoder-free. The encoder's only irreplaceable jobs: independent lock/over-commutation verification,
  and (if used) the down-catch sector — done ONCE per motor model on the bench.
- **"できるやつだとできる機能" (capability model).** Calibration measures what a motor *can* do (smooth
  low-speed sine? reliable down-catch?) and writes capability flags. The firmware enables only the
  features the profile declares; incapable motors gracefully degrade (e.g. 6-step-only, dead-band down).
- **Load handled at runtime, not calibration.** The electrical params are load-independent (calibrate
  once, reuse). The one load-dependent thing (thrust→speed) is corrected by the runtime closed loop.
- **Portable / reusable.** The closed-loop velocity controller uses only standard bidir-DShot telemetry,
  so it runs on ANY such ESC (stock Bluejay/BLHeli-S included), with BlueGill features layered on top.
- **Keep the constructor-injection style.** Controllers take injected deps + keyword config
  (`ESC(link, index, *, ...)`, `SpeedProfile(points, *, ...)`), no globals, `clock` injectable for sim.

## 2. Layered architecture
| Layer | Owns | Motor-dependence |
|---|---|---|
| **ESC firmware (EFM8)** | Generic drive: stock 6-step (always) + S2 sine + S3 crossover + down-catch as **config-gated features**. Reads pole pairs / timing / demag / crossover / catch params from config. | Config only (no code edits) |
| **Pico controller (RP2040)** | **Closed-loop velocity control** (target RPM → PI on the tele speed + feed-forward), the **speed-command interface**, and (bench only) owns the encoder for calibration. | None (generic) |
| **Host (Python)** | **autocal** (measures the motor with the encoder, writes the profile), profile library, `apply`. | Absorbed into the profile |

Runtime data path: `target RPM (signed) → Pico velctl → DShot thrust → ESC (config-driven regime) → motor`.
Feedback: `ESC bidir-DShot tele-eRPM → Pico velctl`. No encoder at runtime.

## 3. Calibration strategy
### What actually needs the encoder
| Item | Encoder need | Encoderless alternative |
|---|---|---|
| pole pairs | none | spec (magnets / 2) |
| KV | none | spec |
| thrust→speed (6-step) | mostly none | **tele-eRPM IS the true speed** → sweep & record |
| thrust→speed (sine) | none | forced commutation → computed (FULLSCALE_RPM constant) |
| comm_timing / demag (true BEMF lock) | **precise with encoder** | minimize demag/stress telemetry (approx proxy) |
| crossover speed | precise with encoder | sine cap (~357 mech, fixed) + BEMF-lock floor (demag proxy) |
| **down-catch sector + feasibility** | **confirmed with encoder** | runtime BEMF self-detect (capable motors only) |

The encoder's irreducible value = **independent truth**: detect over-commutation / aliasing, verify true
lock (slip = tele_mech / enc_mech ≈ 1.0), and confirm/seed the catch. Everything else has a workable
encoderless path.

### Load-independent (bench once, reuse per model) vs load-dependent (runtime)
- **Load-INDEPENDENT (electrical): pole pairs, comm_timing, demag, crossover, catch sector/feasibility,
  KV.** Prop/water don't change BEMF timing. → **Calibrate a motor MODEL once on the bench with the
  encoder → save profile → same-model motors just apply it (no re-cal, no encoder).**
- **Load-DEPENDENT: the thrust→speed curve.** Changes with prop/water. → Used only as a feed-forward
  SEED; the runtime tele-eRPM PI corrects the residual. No mounted re-cal required.

### Recommended workflow (4 tiers)
1. **Spec defaults (0 encoder, instant):** from KV + pole count + bus voltage compute `pole_pairs`,
   max speed, crossover placement (below the sine cap), and BLHeli-S default timing/demag → any motor
   runs 6-step + crossover immediately (baseline).
2. **Bench cal, once per model (encoder):** refine the electrical params to true lock (slip≈1.0),
   the crossover floor, and the **catch sector + feasibility** → save as the model's profile
   (load-independent → reusable, encoder removed afterward).
3. **Runtime closed loop (tele-eRPM PI, encoderless):** absorbs the mounted prop/water load.
4. **(optional) In-situ sensorless refinement:** demag/stress proxies to re-verify lock / re-seed the
   feed-forward under the mounted state — encoderless, approximate.

## 4. Profile format (YAML, selectable, git-managed)
Extend the existing `SpeedProfile` / config YAML (`config.load_yaml`, `SpeedProfile.from_dict`,
`_emit_yaml`, `esctool apply`). Profiles live in `host/profiles/*.yaml` (already git-managed);
`setup` SELECTS a profile (no re-measure). Shape (superset of today's files):
```yaml
identity: { name, layout, mcu, eeprom_revision, ... }   # existing
motor:    { model, kv, poles, bus_voltage }             # NEW: identity of the motor a profile targets
settings: { motor_direction, comm_timing, demag_compensation, sine_mode, sine_*, ... }  # existing ESC cfg
crossover: { up_erpm, dn_erpm }                          # existing
capabilities:                                           # NEW: what calibration proved this motor can do
  sine_lowspeed: true
  down_catch: false            # e.g. weak-BEMF 930KV -> false (dead-band fallback); strong-BEMF -> true
  catch_sector: null           # seeded by encoder cal when down_catch=true
control:                       # NEW: per-motor closed-loop PI gains (plant-dependent, like the curve)
  kp: 0.03                     # bench-tuned; velctl merges CLI flag > this > velocity.DEFAULT_GAINS
  ki: 0.12                     # 930KV 6-step plant gain ~23 mech RPM/cmd-unit (~30x the sim), so the
  trim_max: 400.0             #   sim defaults (0.4/1.5/200) SATURATE here -> profile MUST carry its own
  blend_secs: 0.3
speed_profile:                 # existing SpeedProfile: thrust<->mech-RPM curve (FF seed), pole_pairs, regimes
  pole_pairs: 7
  points: [[t,rpm], ...]
  crossover: { up_erpm, dn_erpm }
```
Consequence: a profile is a motor's "capability certificate". The ESC/velctl read `capabilities`
and `control` and enable/tune only what's proven. Gains are plant-dependent (the FF curve and the PI
gain both scale with the motor), so they ride in the profile exactly like the speed curve — absent a
`control:` block the runtime uses `velocity.DEFAULT_GAINS`. If profiles proliferate, version them in
their own git-tracked dir/repo.

## 5. General closed-loop velocity controller (Pico, ESC-agnostic) — AS BUILT (A1)
Build on the existing `VelocityController` + `SpeedProfile` scaffolding. It is **one PI whose
AUTHORITY fades with telemetry liveness** — NOT a low/high split and NOT a blend keyed on the
crossover band:
- **Interface:** signed **target mech RPM** in → signed DShot thrust out. Constructor-injected:
  `VelocityController(esc, profile, *, kp=…, ki=…, trim_max=…, blend_secs=…, …)`; clock via `run(clock)`.
- **Feedback:** the `tele` frame's `rpm`, which is **already MECHANICAL** (the firmware pre-divides eRPM
  by pole pairs) — used **directly, never `/POLE_PAIRS`**. Available on ANY bidir-DShot ESC → **works on
  stock Bluejay / BLHeli-S too**. A frame is LIVE only when `|rpm| > TELE_MIN_MECH_RPM` (50).
- **Single PI, authority faded on tele-liveness (NEVER on the crossover band):** each tick
  `cmd = thrust_for(setpoint) + w·trim`, `trim = clamp(kp·err + ∫ki·err dt, ±trim_max)`,
  `err = setpoint − copysign(|tele.rpm|, setpoint)`. The weight `w∈[0,1]` ramps 0→1 while a live frame is
  seen and 1→0 while it is stale (over `blend_secs`); the integrator only accumulates on a live frame and
  is reset at `w==0`. So above the seam (6-step, live tele) the loop closes; below it (forced sine, stale
  tele) it degrades **smoothly** to pure feed-forward — with **no** knowledge of where the seam is.
  Back-calculation anti-windup on both the `trim_max` clamp and the outer ESC `tmax` clamp.
- **regime() is ADVISORY only** (display / stall heuristic / down-catch routing), never the authority
  signal. The stall guard keys on the profile's genuinely 6-step-REACHABLE floor (`_line_floor`) so a
  "gap" target whose command runs in sine is pure FF, not a stall — and uses the PROFILE's own curve, not
  the sim's plant gain.
- **Capability-driven:** `capabilities.down_catch` false/absent → `set_speed` from above the seam to below
  it stages through ~0 (re-catch from below). `sine_lowspeed` reserved for low-speed routing. Absent
  capabilities → conservative defaults (works for a stock profile with no crossover/capabilities).
- **Safety:** reuse the deadman keep-alive (`_pace`/DT) + all-path disarm; over-speed on the LIVE
  measurement; stall only when a 6-step-reachable setpoint stays stale for `stall_secs`; temp abort off the
  SAME tele frame (`max_temp=0` disables only the temp check, not the feedback).

## 6. Speed-command interface (regime hidden)
Upper layer (FC/host) commands **only a target speed** (signed). The Pico velctl + ESC config decide
sine vs 6-step internally; low-speed → sine (if capable). The caller never sees the regime. This keeps
the whole sine/6-step/catch complexity inside the ESC (config) + Pico controller.

## 7. Implementation priority — runtime loop LIVES ON THE PICO (C++); verify in Python first
**Decision (2026-07-17): the runtime closed loop's home is the Pico firmware (C++)** — self-contained
(no host PC in the water), low-latency (the Pico speaks DShot + reads telemetry directly, not at the
host's 50 Hz serial rate), and it already owns DShot TX / bidir telemetry / the encoder. The Python
`VelocityController` is NOT the deployment target — it is the **control-law reference + sim/bench
verification harness + calibration tool** (same stack autocal uses). So:

- **Phase A1 (now, Python): prove the control law + get initial gains.** Make `velocity.py`
  `VelocityController` a real closed loop (tele-mech PI + FF + tele-live blend, ESC-agnostic, constructor
  DI). Verify it CONVERGES in the `SimEncEscHost` dry-run first, then on the current 930KV 6-step range
  (the operating range is 6-step → tele feedback is valid). This de-risks the design and pins kp/ki/
  trim_max before any C++ is written. Deliverable: a verified reference implementation + a bench/cal tool.
  **STATUS — DONE (Python A1), HARDWARE-VERIFIED.** Implemented: FF + PI on `(target − tele_mech)`
  (`tele.rpm` used as mechanical, no `/POLE_PAIRS`), PI authority faded on tele-liveness (0↔1 over
  `blend_secs`, integrator reset at `w==0`), back-calculation anti-windup on both the `trim_max` and
  outer `tmax` clamps, over-speed + stall guards keyed on the live measurement / command-implied regime,
  and `set_speed` down-catch staging. The dry-run simulator now models the honest telemetry regime
  (stale in forced sine, live in 6-step). Convergence is PROVEN in `tests/test_velctl_closedloop.py`:
  a ×1.25-rpm-mis-scaled FF (≈80% thrust) still converges ≤5% while `kp=ki=0` (pure FF) misses ≈20%.
  **Hardware (2026-07-17, 930KV 6-step, target 5000 mech):** the loop runs on real hardware, reads
  `tele` correctly (sign OK), and drove a deliberately ×1.3-mis-scaled FF from a 22.7% pure-FF error
  to **0.0%**. Key finding: the **sim gains (0.4/1.5) are ~30× too hot for the real plant** (930KV
  6-step ≈23 mech RPM/cmd-unit vs sim ≈0.8) and saturate the trim rail — the real motor wants
  `kp≈0.03, ki≈0.12, trim_max≈400`, so **gains are now a per-profile `control:` block** (§4), merged
  CLI > profile > `velocity.DEFAULT_GAINS`. See §5/§8 answers below; ported to C++ in A2.
- **Phase A2 (Pico C++): PORT the proven law to the RP2040 firmware. — DONE (builds + native-verified;
  awaits on-motor hardware run).** The control law is a **portable, hardware-free library** at
  `lib/vel_control/vel_control.h` (`SpeedProfile` + `VelocityController` + an injected `EscIo` backend),
  a faithful port of A1. The declaring **main owns the plant-dependent gains and sets them directly**
  (`esc1.kp = 0.03f; esc1.ki = 0.12f; ...`) — deps (backend + profile) are constructor-injected, gains
  are public members (built-in `DEFAULT_GAINS` = the sim starting point). `step(dt)` runs one tick, so
  the app closes the loop every core0 pass (faster than the host's 50 Hz). Demo app
  `src/apps/vel_demo.cpp` (env `vel_demo`): an `EscIo` over `escs::` (with a new `rpmStampMs` freshness
  field so a stale held rpm reads as sine, not live) + a `vel <rpm>` / `stop` / live-gain-tune serial
  interface. Both `vel_demo` and the default `esc_tool` build clean. The port is verified by a native
  g++ test (`lib/vel_control/test/`) mirroring the Python reference: a ×1.25-mis-scaled FF converges
  ≤5 % with the PI (0.1 %) while pure FF misses ~19 %, and a stall aborts. NEXT: flash `vel_demo` and
  run on the motor (displaces `esc_tool` on the Pico); tune gains live with `g kp …`. Upper layer
  (FC/host) then commands only a target speed. Curve/gains/capabilities still ride in the profile the
  Pico holds — a host-side codegen from the YAML profile to the C++ `CurvePoint[]` is a small follow-up.
- **Phase A2 hardware run — DONE.** Flashed on the 930KV: `vel 5000` mech → the on-device loop crosses
  into 6-step (authority 0→1), the PI backs the command off the FF and tele settles on 5000 (~0 % error,
  ~2 % ripple). `vel_demo` has since been folded into the integrated firmware (A3, below).
- **Phase A3 (Pico C++): INTEGRATED object-oriented firmware — DONE + hardware-verified.** Replaces the
  "flash a separate app per job" model with ONE default build (`src/main.cpp`, env `main`). DRIVE has two
  submodes per ESC: **RAW** (direct thrust/throttle) and **RPM** (the closed loop; `poll()` runs
  `vc.step(dt)` each core0 pass). Serial keeps every `esc_tool` command byte-compatible with
  `host/esctool.py` and adds `rpm <i> <v>` / `gain <i> <kp|ki|trim|slew> <v>` + a web RPM control;
  `esc_tool` is kept as a legacy fallback env. Verified on the 930KV: `scan` reads the ESC identity
  (config path), `arm 1`→`rpm 1 5000` converges to **0.2 %**, `thrust 1 300` drives RAW and disengages
  the loop, `gain`/`mode`/`disarm` all work.
- **Phase A3.1 — composable refactor (DONE + hardware-verified).** Split responsibilities so the
  composition root owns the ESCs: **`main.cpp` DECLARES the `class Thruster` objects** (`src/apps/thruster.h`),
  each carrying its **own per-ESC config** — DShot bitrate, motor pole count (plumbed additively into the
  `escs::` HAL via `setKbaud`/`setPoles`, 0 = global default), calibrated `SpeedProfile`
  (`src/apps/profiles.h`, e.g. `profiles::M_930KV`), and PI gains (`esc1.vc.kp = 0.03f;` in `setup()`).
  **`class EscTool`** (`src/apps/esc_tool_app.h`, was the monolithic `EscManager`) is now a **composable
  module that REFERENCES the Thrusters** (`Thruster**`, does not own them) and provides only the
  operator surface (config/flash + serial CLI + Wi-Fi). So a **bare ROV** skips `EscTool` and drives the
  Thrusters directly (`t->setRpm(mix); t->poll(); escs::spinPoll();`). Legacy bring-up apps
  (spikes/demos) archived to branch `archive/legacy-apps`. HAL change is additive; both `main` and
  `esc_tool` build; re-verified on the 930KV (RPM 0.2 %, RAW, config all OK).
- **Phase B (later, calibration): capability autocal.** Extend `autocal` to write the `motor` +
  `capabilities` profile fields (spec defaults + encoder refinement of lock/crossover/catch); on the
  KV=300 motor, re-test the down-catch (3× BEMF should resolve the over-commutation) and set `down_catch`
  per result. Load-independent electrical params → per-model profile (reusable, encoder removed after).

## 8. Open questions / decisions for the review
1. velctl gain scheduling: single PI with regime blend, or separate low/high controllers?
2. FF seed source when a profile has no measured curve yet: pure spec-computed (KV×V×duty) acceptable?
3. Profile identity/selection: match by `motor.model` string, or an explicit `--profile` at setup?
4. Where does the regime blend live — velctl (host/Pico) knowledge of the crossover band, or a signal
   from the ESC (e.g. tele going live) as the regime indicator? (Prefer: tele-live as the 6-step signal.)
5. Do we keep the down-catch firmware in-tree (config-gated, off by default) pending the KV=300 test, or
   leave it out until proven?
