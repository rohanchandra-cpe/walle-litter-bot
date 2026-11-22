<!-- The Phase 4 plan as the user approved it in plan mode on 2026-11-21, before any code
     was written (from the session's plan file). The same plan with "As built" notes on how the
     work differed is docs/plans/phase-4-plan.md (in the Phase 4 commit, 4eee476). -->

# Phase 4 plan: servo and bin monitoring

## Context
At a bin stop the level ESP32-CAM sees the bin's rim at the top edge of the frame, so anything
on top of the bin is cut off. This was Phase 3's weak case: on-top items were cut off or
misnamed. Phase 4 adds a tilting camera (a FITEC FS5106B servo) and a real bin check:
- face the bin;
- count litter from a level photo;
- tilt up, judge the overflow from what is above the rim;
- tilt back;
- store `overflow_level`, `on_top_count`, `litter_count` and `outcome`.

This replaces today's `method="photo_only"` placeholder. The bins are opaque, so there is no
fill level.

Phase 3's item-accuracy box also moved here (commit `177ec2b`). On the 52 photos so far:

| Light | Recall | Precision |
|---|---|---|
| Daylight | 0.91 | 0.84 |
| Room lamps | 0.69 | 0.70 |

The target is recall and precision ≥ 0.80 and category ≥ 0.90 **in both lights**, on photos
taken with the tilted camera at the bin stops.

**Branch:** `walle-phase-4` currently points at `3dea433` and has no commits of its own. The
other session's commit `177ec2b` (daylight shots, light-gated zoom, handoff docs) is on
`walle-phase-3`, which is checked out now. First step after approval:
`git branch -f walle-phase-4 walle-phase-3 && git checkout walle-phase-4`. The uncommitted
handoff moves, `_props.jpg`, weights, `.DS_Store` and `.vscode/` come along untouched.

## Decisions (go in the ROADMAP decision log)

| # | Decision | Outcome |
|---|---|---|
| 1 | Command path | **4th value on `/pico/imu/std`** = servo pulse in µs (≤ 0 or missing = keep, the same rule as the other 3 values). Reliable QoS, and the value stays set. The 3-value senders are unaffected. **Readback:** the commanded pulse is the 5th value of `/pico/wheel_ticks`; readers of `[0..3]` are unaffected. No libmicroros rebuild. *(user, 2026-11-21)* |
| 2 | Calibration location | **On the host, in YAML.** The host turns degrees into µs (`level_us`, `us_per_deg`, `min_deg`/`max_deg`), so recalibrating needs no reflash. The firmware only clamps to a hard safe µs range, slews gently, and starts at a boot pulse. |
| 2 | Servo hardware | **FS5106B, 50 Hz PWM on GP22** (PWM slice 3; the motors use slices 1 and 6). Starting pulse range 1000–2000 µs, widened after calibration. **Power:** not the 9 V directly (see "Power" below). |
| 3 | Bin check | **Two looks:** <br>- **Level look:** the dwell photo after centering → `litter_count`. <br>- **Up look:** tilt `up_deg` (about 20°, from geometry: the rim lands in the lower third of the frame) → `on_top_count` and the height above the rim. <br>Each count comes from one photo, so there is no cross-photo dedupe. If the up photo fails, fall back to the level photo's `on_bin` items (`method: level_only`). |
| 3 | Overflow level | `max(level by count, level by height above the rim)`. Thresholds go in YAML, first set from the sim mounds (count 1–3 / 4–6 / 7+; height bands from `LAYER_STEP_M`), then checked on the robot. |
| 3 | "Full" | **Dropped** from the done criteria. The sim states are clean, overflowing 1/2/3, litter, and overflowing plus litter. *(user)* |
| 4 | Litter | `around_bin` items whose `obj_xy` is within `litter_radius_m` (≈ 0.4 m) of the bin's footprint. Items with no ground point aren't counted but are recorded. This keeps the clutter behind bin 2 out of the count. |
| 5 | Distance | **None measured.** The layout plus centering already give it. Recorded as diagnostics: the distance from the bin's pixel width, and the latest HC-SR04 `echo_us` (free, and shows whether the sensor ever answers). The depth model isn't used, so the missing `depth_calibration.json` doesn't block anything. The ROADMAP work item changes (scope change, in the ledger). |
| 7 | Backing up | Kept as the documented fallback only; not built. |
| 8 | Robot accuracy | Outcome right in ≥ 0.8 of bin checks; overflow level within ±1; litter count within ±1. About 3 laps × 4 bins over several setups. You record the ground truth per bin. *(user)* |
| 9 | Item accuracy (from Phase 3) | Judged on a **new labeled set, `data/photos/phase4/`**, taken at bin stops with the servo: <br>- for every setup, a level shot plus an up shot (`tilt_deg` per shot); <br>- the same on-bin and around-bin setups as Phase 3's shots 7–14, 25, 28 and 43–52; <br>- each setup in daylight **and** under the lamps; about 12 setups × 2 tilts × 2 lights ≈ 48 shots. <br>Scored per light with `eval_photos.py`. On-bin items are scored on the up shots, around-bin items on the level shots, which is exactly how the bin check uses them. The light-gated zoom is re-run under the lamps with the tilt (`--zoom`), and the gate is kept or changed on the results. We draft the shot list together before you take it. |

**Power (your question about the 9 V supply):** don't power the servo from 9 V directly. The
FS5106B is rated 4.8–6 V; at 9 V it overheats and fails. Use a 5–6 V buck converter or UBEC
(≥ 3 A) fed from the 9 V motor supply, with its ground tied to the Pico's. That keeps the
servo's current spikes off the camera's 5 V rail. The Pico's 3.3 V signal drives the FS5106B
directly. If the 9 V source is a PP3 battery, it is too weak for motors plus servo.

## Work

### 1. Firmware (`walle-pilot/firmware/pico_pilot/`)
- New `servo.c`/`servo.h`:
  - GP22, 50 Hz (clkdiv 125 → 1 µs ticks, wrap 19999);
  - `SERVO_MIN_US`/`SERVO_MAX_US` hard clamp and `SERVO_BOOT_US` 1500;
  - slew limit (about 1000 µs/s) to soften current spikes and camera shake;
  - `servo_update()` called in the main loop.
- `pico_pilot.c`:
  - `imu_std_buffer[4]` with capacity 4; `v[3] > 0` → `servo_set_us(v[3])`;
  - `wheel_ticks_buffer[5]`, size 5, with `[4]` = current servo µs;
  - log line and header comment updated.
- The drive watchdog and `drive_control.c` are untouched. The servo moves only the camera.
- Add `servo.c` to `CMakeLists.txt`. I give you the exact build and flash commands; you flash.
- Readers check: `wheel_odometry` and friends read `data[0:2]` / `[2:4]`. Grep and confirm
  none assume `len == 4`.

### 2. Host-side servo control (`walle-app/servo/`)
- `config/servo.yaml`: `level_us`, `us_per_deg` (sign = up), `min_deg`/`max_deg`,
  `settle_s`, `up_deg`, `readback_timeout_s`. There is also a `sim:` block.
- `servo/tilt.py`:
  - `deg_to_us` / `us_to_deg`;
  - `Tilt(node)`: publishes `[0, 0, 0, us]` on `/pico/imu/std` (zeros keep the IMU values);
  - reads the readback from wheel ticks `[4]`; `set(deg)` waits for the readback, then
    `settle_s`.
- `servo/servo_test.py`: a CLI that sweeps or sets angles and prints the readback. It is the
  "servo moves to a commanded angle" check on the robot, and the calibration aid: you measure
  the camera angle with a phone inclinometer and write `level_us`/`us_per_deg`.

### 3. Geometry with tilt (`walle-app/detect/geometry.py`)
- `Camera(..., tilt_deg=0)` gets a full pinhole ray model with pitch:
  - `ray(u, v)`, `bearing_deg(u, v=None)`, `ground_point`, `elevation_deg(v)`;
  - a projection for expected bin px.
  The level camera (tilt 0) gives exactly today's numbers, so the Phase 3 tests must still
  pass unchanged.
- `height_above_rim(item_bbox, dist_m, cam, bin_h)` for the up look.
- `analyze_bin_stop` takes the tilt through `cam`; the `on_bin` test is unchanged (bbox
  relative to the bin's top).

### 4. Bin check (`walle-app/detect/bin_check.py` + route follower)
- `judge_bin(level_an, up_an, bin, cfg)`, pure functions:
  - litter by radius;
  - on-top count;
  - max height above the rim;
  - `overflow_level`, `outcome`;
  - diagnostics.
- `config/detect.yaml` new `bin_check:` block: thresholds, `litter_radius_m`.
- `route_follower.py` dwell step:
  - after `detect_at_bin` (level), `tilt.set(up_deg)` → photo → analyze → `judge_bin`;
  - `tilt.set(0)` in a `finally` before any further motion;
  - `send()` refuses motion while the readback tilt is non-level (log it and level first).
  Results go to `bin_checks.json` in the run folder and to the event in `summary.json`.
  A bin check error never aborts the drive (same pattern as detection).
  Centering stays level.
- `--no-tilt` flag / `route.yaml` `bin_check.enabled` for runs without the servo.

### 4b. Tilted photo set (`walle-app/detect/`, `walle-app/data/photos/phase4/`)
- `phase4/shots.yaml`: the shot list we draft together, with a `tilt_deg` field and a
  `look: level | up` field per shot.
- `capture_set.py`:
  - records `tilt_deg` in `labels.yaml`;
  - an optional `--tilt-cmd` hook (or a printed instruction) so you set the angle with
    `servo_test.py` before each shot.
  - The robot sits still: the Pico watchdog keeps the wheels stopped, since nothing sends
    `/pico/cmd_vel`.
- `eval_photos.py`:
  - passes each shot's `tilt_deg` into `Camera` (the place report uses the tilted geometry);
  - `--set-dir` for the phase 4 set;
  - reports per light and per look (on-bin scored on the up shots, around-bin on the level
    shots).

### 5. Memory (`walle-app/memory/route_records.py`)
- If `bin_checks.json` exists, `add_bin_check(...)` with the judged values and
  `method="tilt"`/`"level_only"`. Otherwise keep `photo_only`.
- `import_runs.py --rebuild` then rebuilds them. Update `memory/README.md`.

### 6. Sim (`walle-sim/`)
- URDF:
  - `camera_tilt_joint` (revolute about y, limits from `servo.yaml`), with a small tilt link
    at today's camera position; `camera_link` is fixed to it;
  - `JointPositionController` plugin.
  Only the joint is added; the rest of the model is unchanged.
- `sim.launch.py`: bridge `/sim/camera_tilt` (Float64) → the gz `cmd_pos` topic.
- `sim_pico_bridge.py`:
  - `on_std` reads `[3]` µs → slews → publishes the angle with the `sim:` calibration;
  - adds the 5th tick value.
- `sim_truth.py`: `project()` applies the pitch. `TruthDetector` takes a `tilt_fn` (the
  follower's readback angle).
- New scenario `config/scenarios/bincheck.yaml`: overflowing 2, litter 1, clean, litter 6,
  so that with `mixed` every state is covered.
- `sim_trials.py bin-check` + `make sim-bin-check SCENARIO=…`:
  - drives a lap with the bin check;
  - compares each bin's judged values with the layout's true values;
  - checks the joint angle (from `/joint_states`) against the command within 1°;
  - prints a per-bin table and PASS/FAIL. Sim PASS = every outcome and overflow level right,
    and litter exact (ground-truth detector).

### 7. Tests (`walle-app/tests/`)
- tilt=0 geometry equals today's;
- ray/ground point round-trip with tilt;
- `deg_to_us` round-trip and clamping;
- `judge_bin` cases;
- `route_records` with `bin_checks.json`;
- `sim_truth` with pitch vs `geometry`.

### 8. Docs
Update:
- ROADMAP Phase 4: wording (overflow, not fill), boxes, decisions, the distance scope change;
- `LEDGER.md` entry;
- `CLAUDE.md` topics table (`/pico/imu/std` 4th value, ticks 5th) and the servo hardware row;
- copy this plan to `docs/plans/phase-4-plan.md`.

Not committed until you ask. The handoff moves, `_props.jpg`, weights, `.DS_Store` and
`.vscode/` stay untouched.

## Needed from you (doesn't block the sim work)
- How the FS5106B mounts and how far it can tilt before the camera hits the body. This sets
  `min_deg`/`max_deg`.
- A 5–6 V buck/UBEC for the servo, and wiring: signal on GP22, common ground.
- The tilted photo set (decision 9), in daylight and lamp light, once the servo works and is
  calibrated.

## Verification
1. `python3 -m unittest discover -s walle-app/tests` on the host (3.13) and in both
   containers (3.10).
2. Sim:
   - `make sim-launch WORLD=park SCENARIO=mixed OBSTACLE=none GUI=false PROFILE=plastic_pid`;
   - then `make sim-bin-check` → PASS;
   - repeat with `SCENARIO=bincheck`;
   - `make sim-route-check MISPLACE=12` still passes (no Phase 3 regression).
   - A Gazebo photo at `up_deg` is saved so we can see the rim is in view.
3. Robot (you):
   - build and flash;
   - `servo_test.py` sweep, readback matches, and calibration;
   - full route with the bin check over the agreed setups;
   - accuracy against your per-bin ground truth (outcome ≥ 0.8, level ±1, litter ±1),
     recorded in the ledger.
4. Item accuracy:
   - you take the phase 4 set;
   - then run in the container:
     `docker exec walle-pilot bash -lc "cd /workspace && python3 walle-app/detect/eval_photos.py --set-dir walle-app/data/photos/phase4"`
     (and again with `--zoom`);
   - PASS when recall and precision are ≥ 0.80 and category ≥ 0.90 in each light. Don't
     score while the robot is driving (they share Ollama).
