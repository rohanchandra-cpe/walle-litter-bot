# Start prompt: Phase 5, obstacle detection and avoidance

Paste everything below the line into a new Claude Code session opened in the repository.

---

You are starting **Phase 5 (Obstacle detection and avoidance)** of the WALL-E application in
this repository. Work with me (the user) as the project owner; I run the real robot.

## First, orient yourself

1. **Read the project docs.** Follow the "Start of every session" steps in `CLAUDE.md`.
   - Read `ROADMAP.md` in full: Phase 5 especially, and Phase 6, whose agent asks "Run your
     route and check the path for any obstructions" and "Where are the obstacles on the route
     right now?".
   - Read the latest entries in `LEDGER.md`.
   - `docs/handoff/phase4-to-phase-5/session-2026-11-21-summary.md` is the short version of
     Phase 4.
   - `walle-app/memory/README.md` describes the store, including `obstacles` and
     `obstacle_sightings`.
2. **Branch.**
   - Phase 4 is commit `4eee476` on `walle-phase-4`, in a PR that may not be merged yet.
     Phase 3 is merged to `main` (PRs #9 and #10).
   - If Phase 4 has been merged, create `walle-phase-5` from `origin/main`. If not, ask me
     whether to branch from `walle-phase-4`.
   - **Run `git status -sb` right before every commit.** In Phase 4 a commit landed on the
     wrong branch because I had switched it in between.
   - Bring these along untouched, and ask me before committing any of them:
     - my move of the old handoff docs into `docs/handoff/phase*-to-phase*/`;
     - `walle-app/data/photos/_props.jpg`.
   - Never commit the 340 MB CLIP copies (`weights/`, `walle-app/weights/`,
     `walle-pilot/weights/`), `.DS_Store` or `.vscode/`.
3. **Check the ledger's last entry.** The previous session's final entry is "Commit on the
   wrong branch, fixed; Phase 4 → 5 handoff". It sits in a second commit on `walle-phase-4`,
   with this prompt. If `main` doesn't have it, the Phase 4 PR was merged before that commit
   was pushed: ask me.
4. **Enter plan mode before writing any Phase 5 code** (`CLAUDE.md` working rule). Read the
   code listed below and settle the open decisions with me. When I approve, copy the plan to
   `docs/plans/phase-5-plan.md`.

## What Phase 5 must deliver (from `ROADMAP.md`)

Notice something on the path, identify it, drive around it, and rejoin the route:
- detection on the path: ultrasonic distance ahead, confirmed by the camera;
- when blocked: stop, identify the object (Phase 3), record it (Phase 2), plan a detour
  around it, and rejoin the route at the next reachable waypoint;
- a fallback when there is no way around: report and stop, or skip that waypoint.

Done when:
- in sim, obstacles placed at random on the route are detected, recorded, and passed in N of
  M runs without collisions;
- on the robot, it passes a real obstacle on the route and rejoins it (I run this);
- obstacle positions in the memory store are within X cm of truth (sim).

N, M and X are to be agreed in plan mode.

**Check the ultrasonic sensor before relying on it.** The roadmap's plan leans on it.
- In Phase 4 the HC-SR04 was **unplugged** (my choice), so its `echo_us` −2 ("no response")
  in those runs means nothing.
- It didn't answer in the earlier phases either; that cause was never checked.
- I'm plugging it back in before Phase 5.

First thing: with the robot side up (`make launch`), watch `echo_us` in `/pico/wheel_ticks`
(`ros2 topic echo /pico/wheel_ticks`) with something at a known distance ahead:
- a positive echo time that tracks the distance means it works (distance ≈
  echo_us × 343 / 2 / 1e6 m);
- −2 means still no response (power or wiring);
- −1 means an echo that never ended.

It works in the sim.

## What Phase 4 left you (useful here)

- **A camera that tilts** (FS5106B servo, calibrated: `walle-app/config/servo.yaml`,
  `walle-app/servo/tilt.py`):
  - range −15° (down) to +55° (up); 10 µs/deg; level at 1575 µs;
  - `Tilt(node, cal).set(deg)` waits for the readback;
  - the follower refuses to drive while it is tilted. Looking down while driving would mean
    changing that rule, deliberately.
- **Where the level camera sees the floor:** it sits 12.5 cm high, 12.5 cm ahead of the axle
  and 1.1 cm left of center. Its bottom row meets the floor about **0.29 m ahead of the lens
  (0.41 m ahead of the axle)**; nothing nearer is in view. Tilted 15° down, the near edge
  comes in to about 0.28 m ahead of the axle.
- **Route guards** (route follower, `route.yaml` `sensor_guard`):
  - stale Pico data → stop and wait;
  - a Pico restart → stop, wait 2 s, repair the estimate from the wheels;
  - an estimate jump (> 5 cm or > 20° beyond the wheels in 0.25 s) → stop and repair;
  - every motion command goes through `send()`, so a detour must too.
  - Robot run `20261122_142402` showed why: a Pico restart sent the fused estimate 40 cm
    sideways, and the robot drove the whole lap 23 cm off.
- **`detect/replay_bin_checks.py`** re-judges a run against ground truth. It is a pattern
  for an obstacle-scoring script.

## The code you are building on

### Route follower (`walle-app/route/route_follower.py`, `route_geometry.py`, `config/route.yaml`)

- **Plan:** `rg.build_plan` → steps `drive` (corner to corner), `face_bin`, `dwell`, `turn`.
- **Sides:**
  - `drive_side` → `side_leg` follows the LINE between corners: cross-track correction,
    20 Hz commands;
  - speed 0.08–0.15 m/s (gain on the distance left);
  - a side that isn't finished within 30 s aborts the run;
  - coast compensation is learned.
- **Turns:** `turn_to` (absolute heading, 1.0–1.5 rad/s); `pulse()` for ≤ 20°.
- **Bin stops:** centering (Phase 3), dwell photo, the bin check with the tilt (Phase 4).
- **It refuses a layout with an obstacle:** `main()` exits with "avoiding it is Phase 5".
  That's the switch to replace.
- **Robot footprint** (`park.yaml` `robot`): half width 0.086 m, turn radius 0.14 m,
  margin 0.05 m.

### Sensing

- **HC-SR04:** `/pico/wheel_ticks` `[left, right, ping_seq, echo_us, servo_us]`.
  - One ping every 60 ms.
  - `echo_us` is the echo time; −1 = echo timeout, −2 = no response.
  - `ping_seq` goes up by one per finished ping, answered or not.
  - On the robot it sits below the camera at the front. In the sim it is modeled as a ±15°
    cone of GPU-lidar rays (`walle-sim/urdf/walle_sim.urdf.xacro`), turned into an echo time
    by `walle-sim/nodes/sim_pico_bridge.py`.
  - RQ3 (`walle-pilot/range/`, frozen study) compared it with the depth model.
- **Camera:** ESP32-CAM, `GET http://192.168.1.201/capture`, 640 × 480.
- **Detector:** `walle-app/detect/detector.py` `make_detector()`: YOLO-World for bins,
  `qwen3.5:4b` in Ollama on the host for items.
  - **5–8 s per photo**, so the robot can't watch continuously while driving: stop and look,
    or look at intervals.
  - `geometry.Camera.ground_point` turns an item box's bottom edge into a floor position
    (`obj_xy`), accurate to a few cm in the sim.
  - Phase 3's photo set had "route" shots (items on the floor 30–60 cm ahead); daylight
    recall was 0.90, lamp light weaker.

### Memory store (`walle-app/memory/store.py`)

- `report_obstacle(run_id, label, category, obj_xy, side=, pose=, detection_id=, action=)`:
  matches an obstacle still on the route within `obstacle_match_radius_m` 0.10 m (same
  side), or records a new one.
- `mark_absent(run_id, obstacle_id)` records that an obstacle has cleared.
- `current_obstacles()` and `obstacle_history()` answer Phase 6's questions.
- `place='obstacle'` exists for detections.
- `import_runs.py --rebuild` rebuilds the store from run folders, so obstacle records must
  also be saved in the run folder, as `detections.json` / `bin_checks.json` are.

### Park and sim

- **Obstacles in the layout:** `park.py` `build_obstacle` places ONE obstacle per layout on
  a side, at 0.25–0.75 along it (`park.yaml` `obstacle.at_range`), with `item`, `pose`
  (upright / lying), `side`, `at` and `position`. It refuses spots too near a corner.
  - Scenarios: `obstacle_only` (seed 3), `mixed` (random obstacle); `OBSTACLE=none|random|1-4`
    at launch.
- **Sim items** (`park.yaml` `items`): `plastic_bottle`, `can`, `paper_cup`, `wrapper`:
  colored cylinders and spheres, 5–20 cm.
- **The sim's ground-truth detector** (`detect/sim_truth.py`) projects bins and their items,
  **not the obstacle**. YOLO-World sees nothing in Gazebo, so this stand-in needs the
  obstacle added.
- **Commands:**
  - `make sim-launch WORLD=park SCENARIO=obstacle_only GUI=false PROFILE=plastic_pid`;
  - `make sim-route-check` (`MISPLACE`, `HIDE_CUT`, `BLACKOUT`, `RESTART`, `JUMP`);
  - `make sim-bin-check`;
  - `make sim-park-check LOOK_AT="obstacle"` saves photos.
- **Relaunch for each scenario:** the world is built at launch, and a headless start takes
  about 30 s.
- **Collisions:** `sim_trials.py` has Gazebo ground truth (`/sim/ground_truth`) for robot and
  model poses. Collision checking would be new.

## Real-world conditions (may change; I'll tell you)

- **Track:** a 1 m square taped on corrugated plastic sheets in my room, with carpet beyond.
  Bins stand 0.88 m out on each corner's diagonal; bin 2's corner has clutter. A detour must
  fit on the sheets, or I need to know how far it may go.
- **Obstacles:** my fake trash (milk jug, coffee can, tissue box, carton, bags, paper
  towels). Most are bigger than the sim's items; some are flat (bags, foil), which ultrasound
  may not see.
- **Power:** 9 V battery for the motors, a 5.7 V buck off it for the servo, a separate 5 V
  supply for the Pico, the HC-SR04 and the camera. A weak 5 V supply resets the camera.
- **Light:** room lamps at night, daylight by day. Detection is much worse under the lamps.
- **Robot runs** end with the follower asking for the tape-measured end pose. Its results
  are written only after that.
- **Containers:** never recreate only the `walle-pilot` container (the micro-ROS Agent shares
  its network). Ollama runs on the host. Don't run a detector eval while I drive.

## Open decisions to settle in plan mode

Recommend an option for each, and record the outcomes in the `ROADMAP.md` decision log.
1. **How the path is sensed:**
   - the HC-SR04, once the check above shows it answering (if it doesn't, ask me to check
     its power and wiring); what it reports for a flat bag or a soft paper towel;
   - camera only (stop every N cm and look; the floor is in view from 0.41 m ahead of the
     axle, or nearer with the camera tilted down);
   - or both.
   What can see a flat bag?
2. **When to look:**
   - with a 5–8 s detector, stop-and-look points along each side (how many?);
   - or only when the ultrasonic reports something within a range;
   - or a fast first pass (e.g. YOLO-World alone, or simple floor/colour cues) with the
     slow model confirming.
3. **The detour:**
   - which side (inside the square, or outside, where the bins are);
   - its width from the object's size and position plus the footprint and margin;
   - how to check the side is clear (the camera turned, a ping);
   - rejoining the line or the next corner.
   The square is 1 m and the obstacle sits 0.25–0.75 along a side.
4. **What counts as an obstacle:** a size or height below which the robot drives on (and
   whether that's safe for the wheels).
5. **The fallback:** no way around → report and stop, or skip to the next corner.
6. **Recording:**
   - `report_obstacle` with `obj_xy` from the photo (ground point) or from the robot's pose
     plus the ultrasonic range;
   - `mark_absent` when a known obstacle is no longer there on a later lap;
   - the run-folder file for rebuilds.
7. **Sim:**
   - the obstacle in `sim_truth.py`;
   - a headless check that runs N scenarios with random obstacles and reports passes,
     collisions (ground truth) and position error;
   - the targets N/M and X.
8. **Interplay with the guards:**
   - a detour is unusual motion, so the jump guard and the sensor guard must not
     false-alarm on it;
   - the tilt rule if the camera looks down while driving.
9. **The robot test:** which obstacle, where, how many runs, and what I measure.

## How to work

- **Sim first,** with a headless one-command check that reports pass/fail. A phase is done
  only after I run it on the real robot.
- **Robot safety:** every motion still goes through `send()` (the guards) and keeps Ctrl+C.
  The Pico stops 500 ms after the last `/pico/cmd_vel`; don't remove that watchdog.
  Firmware: flashing is mine; give me the exact build and flash steps.
- **Don't change frozen files:** `walle-pilot/config/pilot.yaml`, `walle-pilot/pilot/`,
  `walle-pilot/data/schedule/`, the logs under `walle-pilot/data/`, and the depth
  calibration. Reuse `walle-pilot/range/` and `walle-pilot/odometry/`, but don't change their
  behavior.
- **Avoid tuning to one scene.** In Phase 4, rules tuned on one robot run looked perfect on
  that run and weaker on new setups. Freeze the rules, vary the setup between robot runs,
  and score with a replay script.
- **Settings** go in YAML under `config/`; results go in CSV/JSON under `results/` or
  `data/`. New nodes use `rclpy` and the `/x/ask` + `/x/result` JSON pattern.
- **Test** on the host (Python 3.13) and in the containers (3.10):
  `python3 -m unittest discover -s walle-app/tests`.
- **Ledger and roadmap:** add a `LEDGER.md` entry before ending any turn that changed files:
  what was verified and how, what wasn't, and any scope drift. Tick the `ROADMAP.md` boxes
  as they are met.
- **Git:** don't commit or push unless I ask, and check the branch right before committing.
  `gh` is not installed; I open PRs in the browser.
