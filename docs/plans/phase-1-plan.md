# Phase 1 plan: route following

## Context

Phase 1 of `ROADMAP.md`: drive the 1 m park square from `walle-app/park.py`, stop at the bin
corners, report progress, and return to the start. Only the encoders and IMU (EKF) track
position. The user asked me to build on their module 7 and module 8 square-driving code.
- **Module 8** (`odometry/move_test.py`, `odometry_node.py`, `firmware/pico_fusion`) is
  byte-identical to `walle-pilot/odometry/` and `walle-pilot/firmware/pico_pilot/`. Its
  `Mover` steers by `/odometry/filtered`, with stop-and-turn moves and coast compensation.
  In its sim square (1 % wheel mismatch), EKF steering ended 1.1 cm from the start;
  wheels alone ended 17 cm off.
- **Module 7** `firmware/pico_pid` has a per-wheel PI speed loop in firmware
  (feed-forward, ramp, anti-windup). It uses a telemetry publisher and a gains
  subscription, so it does not fit next to the current firmware's 5 micro-ROS slots as is.

The main risk is heading: a 5° error after a turn (the pilot's EKF turns) puts the robot
about 8.7 cm off the line by the end of a 1 m side. `Mover.straight` holds the heading the
robot *has* when the side starts, so it carries that turn error into the side.

## Decisions (settled with the user; go into the ROADMAP decision log)

| # | Decision |
|---|---|
| 1 | **Control runs on the computer side** (a ROS node using the EKF), built on module 8's `Mover`. Firmware unchanged. Module 7's firmware PID is held back as the fix if drift measurements show the wheel speeds are the problem; the sim `PidFirmware` models it. |
| 2 | **Pass (sim):** 1 lap, end within **5 cm** of the start (ground truth). Plus: stopped facing every bin. |
| 3 | **Corners:** stop and turn in place, as in module 8. |
| 4 | **Bin stops:** at the corner, turn to face the bin's center, wait **15 s** (config), then turn to the next side. |
| 5 | **Bins are boxes** 30 × 18 × 36 cm (L × W × H), wide face toward the square. `shape` key kept so a cylinder can come back. **Bins moved out** so the whole bag fits in the frame from the corner (the user's choice). |
| 6 | **Drift measured first, and treated as a priority:** record per-corner and end error in sim and on the robot. If the robot misses 5 cm, spend time fixing it before Phase 2 (escalation below). |
| 7 | **Safety:** the user starts each run and watches with Ctrl+C. Ctrl+C sends stop. Each move has a timeout. The Pico's 500 ms watchdog stays. No approval prompt (that is Phase 6). |

## What gets built

### 1. Park config: box bins, offset that frames the bin (`walle-app/`)
- `config/park.yaml` `bins`: `shape: box`, `length_m: 0.30`, `width_m: 0.18`,
  `height_m: 0.36` (radius kept, used only for `shape: cylinder`). New `camera` block under
  `robot`: `height_m` (MEASURE, placeholder 0.10), `forward_m` (camera ahead of the axle,
  MEASURE, placeholder 0.07), `vertical_fov_deg` 47, `horizontal_fov_deg` 60,
  `frame_margin` 0.10 (headroom above and below the bin, so it is centered rather than
  edge-to-edge).
- `park.py`: `offset_m: auto` becomes the larger of (a) clearance: the box's corners clear
  of both sides and of the turn circle (replaces the radius formula), and (b) **framing**:
  the bin's top and bottom fit inside the vertical field of view, and its width inside the
  horizontal field, seen from the corner. With the placeholders, (b) wins at about 0.8 m.
  Bin yaw = outward diagonal. Litter and spill placement use the box footprint.
  Each bin's layout gets `shape`, `size`, `yaw_deg` and `view_heading_deg` (the heading
  from the corner to the bin's center).
- Tests in `tests/test_park.py`: the box clearance, the framing offset, the yaw, and a
  cylinder config that still loads.
- `walle-sim/worlds/park_world.py`: emit a `<box>` with the yaw. The mound and the rim
  marker follow the shape.

### 2. Route follower (`walle-app/route/`)
- `route_follower.py`: a node that subclasses `Mover` (imported from
  `walle-pilot/odometry/move_test.py`, which stays unchanged). It reuses `Mover`'s
  subscriptions, `send`/`stop`, `lead()` coast compensation, and the sim
  `coast_time = 0` switch.
  - **Sides:** follow the line from corner to corner (not just "drive straight").
    Steering = route heading + `atan(k · cross-track)`, clamped. The robot stops when the
    distance *along the side* to the corner reaches the tolerance (the RQ1-B rule: gain ×
    remaining, clamped 0.05–0.15 m/s, plus coast lead). A turn error is therefore corrected
    on every side instead of building up.
  - **Turns:** to an **absolute** heading (the next side's heading, or the bin's view
    heading), not "+90° from wherever I am".
  - **Bin corners:** turn to `view_heading_deg`, wait `dwell_s`, turn to the next side.
  - **Progress:** a `std_msgs/String` JSON on `/walle/route/status` (run id, waypoint,
    state: driving/turning/at_bin/done/aborted, distance driven, pose), also printed.
  - **Log:** `walle-app/results/route/<run_id>/` with `trajectory.csv` (EKF, wheels and,
    in sim, ground truth at 10 Hz), `events.csv` (per corner: arrival pose, heading error
    to the bin) and `summary.json`.
  - **Safety:** Ctrl+C → `stop()` in `finally`; each move times out; it refuses to start
    without fresh `/odometry/filtered`.
  - Usage: `python3 walle-app/route/route_follower.py --scenario default [--bins N] [--dwell 15]`.
    The bin list comes from the same scenario as the sim, so sim and robot runs match.
- `config/route.yaml`: speeds, gains, tolerances, `dwell_s: 15`, cross-track gain, timeouts.

### 3. Sim check (`walle-sim/`)
- `harness/sim_trials.py route-check`: move the robot to the start pose (ground truth),
  run the follower for N laps (default 1), then report per corner: position error (ground
  truth vs the corner), heading error to the bin at each stop, and the **bin fully in
  frame** check (projects the box's 8 corners into the sim camera using its pose and FOV,
  and saves a photo at each stop). Then the end error vs 5 cm → PASS/FAIL, and the
  results go to `walle-sim/data/<run>/route/`.
- `Makefile`: `make sim-route-check SCENARIO=... LAPS=1`.
- New `carpet` profile in `sim_profiles.yaml`: encoder overcount (slip, scale about 1.03),
  motor mismatch 1.02/0.98, slower spin-up, gyro bias. Run on `ideal`, `pilot_firmware`
  and `carpet`. Note: the Gazebo robot is Module 6's geometry, so sim error is a software
  check, not a prediction of the robot's error.

### 4. Robot run and drift record
- The user runs `make up` + `make launch`, puts the robot on the taped start mark, and runs
  the follower. Afterwards a prompt asks for the tape-measured end offset (x, y, cm) and
  heading (deg), and per corner whether it stopped facing the bin. These go into the run's
  `summary.json` and one row in `walle-app/results/route/robot_runs.csv`.
- Before the robot run, the user measures and enters the camera height and forward
  offset, then places the bags at the offset `park.py` prints.

### Drift escalation (only if the robot misses 5 cm)
In order, each checked in sim first: tune `coast_time` and turn tolerance on carpet →
calibrate wheel diameter and track on carpet → cross-track gain → firmware wheel PID from
module 7 (sim `pid_firmware` profile first, reflash with your OK) → a visual cue (bin or
tape line, decided then).

## Files
- Changed: `walle-app/config/park.yaml`, `walle-app/park.py`, `walle-app/tests/test_park.py`,
  `walle-sim/worlds/park_world.py`, `walle-sim/harness/sim_trials.py`,
  `walle-sim/config/sim_profiles.yaml`, `Makefile`, `walle-app/README.md`, `ROADMAP.md`,
  `LEDGER.md`, `CLAUDE.md` (commands).
- New: `walle-app/route/route_follower.py`, `walle-app/config/route.yaml`,
  `walle-app/tests/test_route.py` (the geometry: cross-track, along-side distance, absolute
  headings), `docs/plans/phase-1-plan.md` (this plan).
- Not touched: everything under `walle-pilot/`, the firmware.

## Verification
1. `python3 -m unittest discover -s walle-app/tests` (host and container).
2. `make sim-launch WORLD=park SCENARIO=default GUI=false`, then `make sim-park-check`
   (boxes in place, photos), then `make sim-route-check` on `ideal`, `pilot_firmware` and
   `carpet`: PASS = end ≤ 5 cm, every bin stop facing its bin and fully in frame.
3. Then the robot run (you), and the drift record above. Phase 1 is done when the robot
   run is recorded.

---

## Changes while building (2026-11-16; details in `LEDGER.md`)

The first `pilot_firmware` sim run failed (corners overshot 5–7 cm, turns 7–19°, EKF heading
drift during the 15 s waits). Added to the follower, as the first step of the drift
escalation (tune coast and tolerances):
- separate coast for sides and turns, kept on in Gazebo (walle-sim's firmware models spin
  down like the robot; module 8 set it to 0 there), learned from each move's overshoot;
- up to 2 correction moves when a corner is missed by > 1.5 cm or a turn by > 3°;
- gyro drift: a 10 s still period at the start measures the EKF heading's drift at rest,
  the bin waits add to it, and the drift is removed while driving and while waiting
  (zero-velocity update when the encoders show no motion);
- a `carpet_pid` sim profile, to see what the firmware PID would gain on carpet.
