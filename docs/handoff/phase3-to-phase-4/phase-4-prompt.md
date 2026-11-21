# Start prompt: Phase 4, servo and bin monitoring

Paste everything below the line into a new Claude Code session opened in the repository.

---

You are starting **Phase 4 (Servo and bin monitoring)** of the WALL-E application in this
repository. Work with me (the user) as the project owner; I run the real robot.

## First, orient yourself

1. **Read the project docs.** Follow the "Start of every session" steps in `CLAUDE.md`.
   - Read `ROADMAP.md` in full: Phase 4 especially, and Phase 6, which will ask about bins.
   - Read the latest entries in `LEDGER.md`.
   - `docs/handoff/phase3-to-phase-4/session-2026-11-19-summary.md` is the short version of
     Phase 3.
   - `walle-app/memory/README.md` describes the store, including `bin_checks`.
2. **Branch.**
   - Phase 3 is commit `3dea433` on `walle-phase-3`, in a **draft PR that is not merged**.
   - If it has been merged by the time you start, create `walle-phase-4` from
     `origin/main`. If not, ask me whether to branch from `walle-phase-3`.
   - Bring these along untouched, and ask me before committing any of them:
     - my move of the old handoff docs, including `docs/handoff/phase3-to-phase-4/`;
     - `walle-app/data/photos/_props.jpg`.
   - Never commit the 340 MB CLIP copies (`weights/`, `walle-app/weights/`,
     `walle-pilot/weights/`), `.DS_Store` or `.vscode/`.
3. **Enter plan mode before writing any Phase 4 code** (`CLAUDE.md` working rule). Read the
   code listed below and settle the open decisions with me. When I approve, copy the plan to
   `docs/plans/phase-4-plan.md`.

## What Phase 4 must deliver (from `ROADMAP.md`)

Tilt the camera up at a bin, judge how overflowing it is, and count litter around it:
- servo control in the Pico firmware, and a servo joint on the simulated robot;
- a bin check at each bin stop: face the bin (Phase 3 centering does this), take photos at
  one or more tilt angles, judge the overflow, and count litter on the ground around it;
- distance to the bin from ultrasonic and/or the calibrated depth model, as in RQ3;
- results go to the memory store.

Done when:
- the servo moves to a commanded angle on the robot and in sim;
- in sim, bins set to empty / full / overflowing / litter-around are classified correctly;
- on the robot, the same states with the real bins, accuracy recorded (I run this);
- item accuracy carried over from Phase 3, now with the tilted camera (section below).

**Reconcile the wording in plan mode.** The roadmap still says "fill level", but Phase 2
decided (with me) that the bins are opaque and the camera can't see inside them. A bin check
records:
- `overflow_level`: 0 = nothing above the rim … 3;
- `on_top_count`;
- `litter_count`;
- `outcome`: `overflowing`, `litter` or `clean`.

There is no fill level. "Full" in the done criteria needs a definition we can see (for
example, items level with the rim) or should be dropped.

## Item accuracy, carried over from Phase 3

Phase 3's accuracy box was left open on purpose (my choice, 2026-11-21), and Phase 4 now
has a "done when" box for it. With the **tilted camera at the bin stops**, the target is
recall and precision ≥ 0.80 and category ≥ 0.90, in daylight **and** under the room
lamps.

**Where it stands:** 52 labeled photos, `results/detect/20261121_154929`.

| Light | Recall | Precision |
|---|---|---|
| Daylight | 0.91 | 0.84 |
| Room lamps | 0.69 | 0.70 |
| All | 0.787 | 0.768 |

- Category is 0.945, bins are found 100%, and the median is 8.1 s per photo.
- The failing case is items on top of a bin under lamp light: too few pixels, a warm and
  noisy image, and the rim at the top edge of the frame.

**What's already in place:**
- **Gated zoom:** a zoomed second search over the bin's rim runs only in good light
  (`vlm.zoom_bin_top`, `max_warm_cast: 9`: mean red − blue of the photo; my lamps measure
  11–22, daylight 2–7).
  - In daylight it lifted on-bin recall from 0.73 to 0.93; under the lamps it hurt (0.75 →
    0.56).
  - With the camera tilted, try it under lamp light again.
- **Photo-set tools:** `detect/capture_set.py` (shot list → photos + labels, `--light`,
  `--only` for retakes) and `detect/eval_photos.py` (`--ids`, `--zoom`) are the tools for a
  tilted-camera set. Plan the set with me.

**If Ollama is slow:** don't run an eval while I drive the robot; they share Ollama on the
host.

## The code you are building on

### Firmware (`walle-pilot/firmware/pico_pilot/`, Pico 2 W, C, Pico SDK, micro-ROS over UDP)

- **micro-ROS:** at its limit of 3 publishers and 2 subscriptions, all used.
  - Publishers: `/pico/wheel_ticks`, `/pico/imu/data`, `/pico/drive/status`.
  - Subscriptions: `/pico/cmd_vel`, `/pico/imu/std`.
  - Adding a servo topic means rebuilding `libmicroros` with higher limits, or carrying the
    angle in an unused field (e.g. `/pico/cmd_vel` `angular.x`). That's the first open
    decision.
- **Pins in use:**

  | Pins | Use |
  |---|---|
  | GP0, GP1 | BNO055 I²C |
  | GP2–GP5, GP10–GP12 | TB6612 motor driver |
  | GP16, GP17 | HC-SR04 |
  | GP18–GP21 | encoders |

  Candidates for a servo PWM pin: GP6–9, GP13–15, GP22, GP26–28. Check the board first.
- **Timing and safety:** ticks and IMU are published at 50 Hz. The drive has a closed-loop
  PI wheel-speed loop and a **500 ms watchdog** that stops the motors. Keep the watchdog.
- **Power:** a 9 V supply drives the motors; a separate 5 V supply runs the Pico, the
  HC-SR04 and the ESP32-CAM. A weak 5 V supply resets the camera. A servo draws current
  spikes, so decide how it is powered.
- **Secrets:** `wifi_config.h` is git-ignored; never commit it.

### Camera and geometry

- **Camera:** ESP32-CAM (OV3660), `GET http://192.168.1.201/capture`, 640 × 480 JPEG.
  - Nominal FOV 60 × 47° (`walle-app/config/park.yaml` `robot.camera`). The bin's pixel
    width matched 60° horizontally.
  - It sits 10.3 cm above the floor and 12.5 cm ahead of the axle.
- **Bins:** black plastic, 27.7 × 21.1 × 29.2 cm. They stand **0.88 m out** on each
  corner's diagonal (`park.yaml` `offset_m`). Bins 1 and 3 are trash, bins 2 and 4
  recycling.
- **What the level camera sees at a bin stop:** the bin's rim at the **top edge of the
  frame**. Anything on top of the bin is cut off; a milk jug on top was never fully in view.
  This is why the servo exists.
- **Fallback if the servo fails: back up.** The robot backs ~0.35–0.45 m along the diagonal,
  takes the photo, and drives back. That's in the ROADMAP Phase 4 open decisions with the
  geometry. The drive code only goes forward today.

### Route follower (`walle-app/route/route_follower.py`)

The bin stop runs:
1. `face_bin` → `turn_to`.
2. **`center_bin()`** (Phase 3):
   - photo → detect → pulse by bearing / (parallax × learned `turn_scale`);
   - ±15° search if no bin is seen;
   - up to 3 corrections; the record goes in `summary.json`.
3. The dwell: `hold_still()` takes the photo.
4. **`detect_at_bin()`:** writes `detections.json` (items with `place` `on_bin` /
   `around_bin`, `obj_xy`, `rim_visible`).

The bin check with tilt fits after centering. Other things already in place:
- **Moves:** pulses (`pulse()`) for ≤ 20°, `turn_to` above that.
- **Sensor guard:** `route.yaml` `sensor_guard`. Every motion command goes through `send()`,
  which stops and waits on stale Pico data.
- **Robot run commands:** `make launch` in one shell, then
  `python3 /workspace/walle-app/route/route_follower.py --scenario default --camera-url http://192.168.1.201/capture`.

### Detection (`walle-app/detect/`, `config/detect.yaml`)

- **Detector:** `make_detector()` gives YOLO-World for bins plus `qwen3.5:4b` in Ollama for
  items (find, then name each crop).
  - About 5–8 s per photo.
  - `geometry.analyze_bin_stop(...)` turns detections into bin bearing, `rim_visible`,
    items with `place`, and `obj_xy`.
  - It assumes a level camera (`Camera.ground_point`, `bearing_deg`). **Tilt changes this
    geometry:** the ground-plane position, the rim test and the expected bin size all need
    the tilt angle.
- **Accuracy:** items on top of a bin under lamp light are the weak case. See "Item
  accuracy, carried over from Phase 3" above.

### Memory store (`walle-app/memory/`)

- **Write API:** `MemoryStore.add_bin_check(run_id, bin_id, overflow_level=, on_top_count=,
  litter_count=, outcome=, pose=, photo=, method=)`. Today every bin stop writes one with
  `method="photo_only"` (not assessed), in `memory/route_records.py`. Phase 4 assesses
  them.
- **Detections:** `add_detection` rows come from `detections.json`.
- **Rebuilds:** `import_runs.py --rebuild` rebuilds the store from run folders, so bin-check
  results must also be saved in the run folder.

### Sim (`walle-sim/`)

- **Robot model:** the camera is a fixed joint, `camera_joint` in
  `walle-sim/urdf/gazebo_car.urdf.xacro`. A servo needs a revolute joint and a way to
  command it.
  - The robot model is kept as Module 6's on purpose; add the joint, don't redesign the
    model.
  - The simulated Pico (`nodes/sim_pico_bridge.py`) speaks the same topics as the real
    one, so it should carry the servo the same way.
- **Detector stand-in:** `detect/sim_truth.py` projects the true layout through a **level**
  camera (`sim.camera` in `detect.yaml`). It needs the tilt too.
- **Park scenarios:** they set bin states (`ok`, `overflowing` levels 1–3, `litter` 0–6;
  `mixed` has one of each). Items above the rim are a bin's `mound`. That is the ground
  truth for the sim check.
- **Commands:**
  - `make sim-launch WORLD=park SCENARIO=mixed OBSTACLE=none GUI=false PROFILE=plastic_pid`;
  - `make sim-route-check RUN=… DWELL=3 MISPLACE=12` (add `HIDE_CUT=1`, `BLACKOUT="20 1.5"`).
- **Gotchas:**
  - The world is built at launch from `park.yaml`: relaunch after changing it.
  - The follower refuses layouts with an obstacle (Phase 5): keep `OBSTACLE=none`.

### Distance (`walle-pilot/depth/`, `walle-pilot/range/`)

- **Depth model:** `walle-pilot/depth/depth_estimator.py` `DepthEstimator` reads its
  calibration from `walle-pilot/config/depth_calibration.json` (part of the RQ3 study:
  reuse it, don't change it). **That file is not in the checkout** (checked 2026-11-21);
  ask me where it is before relying on the depth model. Depth Anything reads Gazebo
  renders poorly, so calibration is rejected in sim.
- **HC-SR04:** `echo_us` in `/pico/wheel_ticks` (−1 timeout, −2 no sensor). It has never
  answered on the robot; it may be a power problem. Ask me before relying on it.
- **Do we need a distance at all?** The bin's distance is already known from the layout,
  centering and the bin's pixel width. Decide in plan mode whether a measured distance is
  worth it.

## Real-world conditions (may change; I'll tell you)

- **Servo:** I have one. It is not wired or in the firmware yet; ask me which servo and how
  it mounts.
- **Track:** the square is taped on corrugated plastic sheets in my room. Bin 2's corner
  has clutter and a white wall behind it.
- **Light:** night runs use room lamps.
- **Containers:** never recreate only the `walle-pilot` container (the micro-ROS Agent
  shares its network). After a Dockerfile change: `make build` / `make sim-build`, then
  `make down && make up`.

## Open decisions to settle in plan mode

Recommend an option for each, and record the outcomes in the `ROADMAP.md` decision log.
1. **Servo command path:** rebuild `libmicroros` with more publishers and subscriptions, or
   carry the angle in an unused `/pico/cmd_vel` field. Is a position readback needed?
2. **Servo hardware:**
   - PWM pin, frequency and pulse range;
   - power (the 5 V supply already resets the camera when weak);
   - mounting, the tilt range, and calibration (the angle against a reference).
3. **The bin check:**
   - which tilt angles: one look up, or a level look plus an up look;
   - how photos at several tilts are combined (the same item seen twice);
   - how `overflow_level` 0–3 is judged from what is above the rim (item count, height
     above the rim, or both);
   - what "full" means, given we can't see inside.
4. **Litter around the bin:** the level photo's `around_bin` items (already counted). Should
   it also depend on distance from the bin (`obj_xy`)?
5. **Distance:** ultrasonic, depth model, the layout and bin width, or none.
6. **Sim:**
   - the servo joint;
   - `sim_truth.py` with tilt;
   - a headless check that reports, per bin and scenario state, the expected against the
     judged outcome.
7. **Backing up:** keep it only as the documented fallback, or build it now as well.
8. **Accuracy target** for the robot check, and how I set up the ground truth: overflow
   level and litter per bin.

## How to work

- **Sim first,** with a headless one-command check that reports pass/fail. A phase is done
  only after I run it on the real robot.
- **Firmware:** keep the 500 ms watchdog and the emergency stop path. Flashing is mine; give
  me the exact build and flash steps.
- **Don't change frozen files:** `walle-pilot/config/pilot.yaml`, `walle-pilot/pilot/`,
  `walle-pilot/data/schedule/`, the logs under `walle-pilot/data/`, and the depth
  calibration. Reuse `walle-pilot/vision/` and `walle-pilot/odometry/move_test.py`, but don't
  change their behavior.
- **Settings** go in YAML under `config/`; results go in CSV/JSON under `results/` or
  `data/`. New nodes use `rclpy` and the `/x/ask` + `/x/result` JSON pattern.
- **Test** on the host (Python 3.13) and in the containers (3.10):
  `python3 -m unittest discover -s walle-app/tests`.
- **Ledger and roadmap:** add a `LEDGER.md` entry before ending any turn that changed files:
  what was verified and how, what wasn't, and any scope drift. Tick the `ROADMAP.md` boxes
  as they are met.
- **Git:** don't commit or push unless I ask. `gh` is not installed; I open PRs in the
  browser.
