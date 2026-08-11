# Differential azimuth thruster — bring-up log

Two BLDCs, one pod, wired to **GP10 (ESC index 0)** and **GP11 (ESC index 1)**. This is the log of
taking it from "just built" to a first spin, following `docs/thruster-bringup.md`. It records what
the hardware actually reported, not what it was supposed to be.

Snapshots referenced here live in the `bench-registry` repo (observations belong outside this one).

---

## Session 2026-08-11 — pre-spin audit. **The spin did not happen; two blockers.**

### What is confirmed good

| check | result |
|---|---|
| Pico link (`/dev/ttyACM0`, VID 2E8A) | `id esc_tool v1` |
| ESC signal pins | `pins 2 10 11` — two channels, as wired |
| Both ESCs powered + talking 1-wire | idx 0 and idx 1, sig `E8B2`, layout `#A_H_30#`, fw `0.21` |
| Pico runtime config persisted | `cfg\|stored=1` — both channels `kv=350 pp=7` |
| ch1 (GP11) ESC EEPROM | **byte-identical** to `20260801T103254Z-verify-after-flash-defaults-experiment` (BlueGill S4) |

`tele 0` / `tele 1` return `err no-telem` and `enc` returns `err no-encoder`. Both are expected here:
telemetry only flows while frames are being sent, and there is no AS5600 on a real pod.

### Blocker 1 — ch0 (GP10) is sitting at post-flash defaults

`bench.py diff 20260811T041059Z-azimuth-ch0 20260811T041108Z-azimuth-ch1` → **16 settings differ.**
Four of them are the exact four this bench has already measured, one at a time, against a good config:

| setting | ch0 (GP10) | ch1 (GP11) | measured effect of the ch0 value |
|---|---|---|---|
| `startup_power_max` | **5** | 25 | **0 rpm — will not start at all** |
| `low_rpm_power_protection` | **9** | 0 | 380 rpm (a quarter) |
| `sine_cross_up` | **255** | 36 | 230 rpm — handoff refused, never leaves forced sine |
| `motor_direction` | **Normal** | Bidirectional | 180 rpm |

Plus `sine_mode 255` vs `2` (forced sine off entirely), `sine_cross_dn 255` vs `239`,
`pwm_frequency 48` vs `24`, `startup_power_min 204` vs `30`, and eight more.

ch0 has never been configured — it is a stock ESC. **Spinning the pod in this state would not have
been a test of the thruster.** Either the motor sits dead and the mechanism looks broken, or worse it
turns at a quarter speed against a correctly-driven ch1, and a *differential* pod driven by a matched
pair that is not matched will yaw when it was commanded to translate. That reads as a mechanical
fault, and it is a config fault.

> **New failure mode this repo did not have before.** Every existing pre-spin guard is per-channel:
> `bench.py check` compares one live ESC against one snapshot. Nothing checks that the two channels
> of one thruster agree with *each other*. On a single-motor bench that check did not exist because
> it could not fail. It is now the first thing to run.

### Blocker 2 — the camera is not pointed at the thruster

The session premise was that the camera framing shows the pod. It does not. A live frame from
`/dev/video0` (HD Pro Webcam C920, the only camera on the machine — `video1` is its metadata node and
will not open) shows a desk lamp head, a BenQ monitor, a window with blinds and a bookshelf. There is
no motor, propeller, ESC or pod anywhere in frame.

Evidence: `bench-registry/evidence/20260811-azimuth-bringup/camera-as-found.jpg`.

The camera has been **moved** since the last bench session: the lake's last capture
(`derived/camera/latest.jpg`, 2026-08-01T19:48Z) is the same device framing the **PSU panel**
(HANMATEK HM310 reading 11.20 V / 0.005 A). That framing is gone. The daemon is also not running —
`/dev/video0` opened directly with no contention, and `latest.json` is ten days stale.

A first spin of a newly built mechanism with no eyes on it is not a test, it is a hope. Held.

### Not yet established — needed before the spin means anything

- **What motors are in the pod.** Both Pico channels are configured `kv=350 pp=7`, inherited from the
  bare-shaft F2838 bench motor. If the pod uses anything else, every speed silently mis-scales and
  there is no second opinion. `docs/thruster-bringup.md` §1 wants the pole count *verified*, and the
  encoder that would verify it is not on this pod.
- **Whether propellers are fitted.** Changes the first-spin hazard completely.
- **The supply, measured.** `cfg` carries `v=11.10` (ch0) and `v=11.20` (ch1) — those are configured
  numbers, not observations, and they disagree with each other.

### Standing hazard, unresolved: the loom

The supply loop is a **measured 0.254 Ω**. Above ~1 A the PSU panel is no longer what the ESC sees
(1.27 V lost at 5 A). One bare-shaft motor stayed under that. **Two motors turning propellers will
not.** The loom is to be replaced before any loaded or in-water run — a pod test is exactly the run
that first exceeds it, and every voltage-derived number is wrong until it is.

### State left on the hardware

Nothing was written. Both ESCs were read and released (`esctool run`); no config was applied, no
motor was armed, no channel was spun. As-found state is captured in
`bench-registry/snapshots/20260811T041059Z-azimuth-ch0` and `…041108Z-azimuth-ch1`.

### To resume

1. Re-aim the camera at the pod and confirm the frame.
2. Confirm the pod's motor KV / pole count, and whether props are fitted.
3. Mirror the known-good config onto ch0 — `bench.py restore 20260811T041108Z-azimuth-ch1` against
   index 0, **not** a checked-in profile — then re-diff ch0 against ch1 and require zero drift.
4. Re-check `motor <i> <kv> <pp> <v>` on both channels against the real motor.
5. Only then spin, one channel at a time, lowest command first, on camera.
