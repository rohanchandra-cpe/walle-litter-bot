# Start prompt: Phase 1, route following

Paste everything below the line into a new Claude Code session opened in the repository.

---

You are starting **Phase 1 (Route following)** of the WALL-E application in this repository.
Work with me (the user) as the project owner; I run the real robot.

## First, orient yourself

1. Follow the "Start of every session" steps in `CLAUDE.md`. Read `ROADMAP.md` in full
   (Phase 1 especially) and the latest entries in `LEDGER.md`.
2. Check the branch. Phase 0 is on `implement-core-walle-functionality` (commit `e1b61c2` and later). If it
   is merged to `main`, create `walle-phase-1` from `origin/main`; if not, ask me whether to
   branch from `implement-core-walle-functionality`.
3. **Enter plan mode before writing any code** (`CLAUDE.md` working rule). Read the code
   listed below, settle the open decisions with me, and when I approve, copy the plan to
   `docs/plans/phase-1-plan.md`.

## What Phase 1 must deliver (from `ROADMAP.md`)

Drive the park route from the park config, waypoint to waypoint, stop at bin waypoints,
report progress, and return to the start, closed loop, on dead reckoning (encoders + IMU).
Done when:
- in sim, the robot completes the route N times in a row and ends within X cm of the start
  (we set N and X in plan mode);
- on the robot, it completes the route and stops at each bin waypoint (I run this);
- end-of-route position error is measured on the robot and recorded.

## The code you are building on

**Park (Phase 0).** `walle-app/park.py` `load_park(scenario, bins=, obstacle=, seed=)` gives
`route.corners`, `route.waypoints`, `route.start`, and `bins` (corner, position, size, state).
The route is a 1 m square, CCW 1 → 2 → 3 → 4 → 1, corner 1 = origin, start facing corner 2
(+x). The route follower should read the route from here, not hard-code it. Config:
`walle-app/config/park.yaml`, scenarios in `walle-app/config/scenarios/`.

**Firmware (`walle-pilot/firmware/pico_pilot/`)**, what the robot runs today:
- `pico_pilot.c`: micro-ROS over Wi-Fi UDP. Publishes `/pico/wheel_ticks`
  `[left, right, ping_seq, echo_us]` and `/pico/imu/data` at 50 Hz, `/pico/drive/status`
  (0 stopped, 1 driving, 2 watchdog stop). Subscribes `/pico/cmd_vel` (only `linear.x` and
  `angular.z` used) and `/pico/imu/std`. All 3 publishers and 2 subscriptions of the
  micro-ROS library are used.
- `drive_control.c`: **open loop**. Twist → wheel speeds (separation 0.127 m) → duty
  `MIN_DUTY + (1 − MIN_DUTY)·|v / 0.40|` with `MIN_DUTY = 0.40`, so any small command jumps
  to 40 % duty (a 0.15 m/s command drives at about 0.27 m/s). Limits 0.30 m/s, 1.5 rad/s.
  The **500 ms watchdog** stops the wheels without new commands; never remove it.
- Encoders: 4320 counts per wheel turn, wheel diameter 0.0635 m. BNO055 in IMU mode; IMU
  data is published only after the gyro calibrates.

**Robot-side ROS (`walle-pilot/`).** `odometry/odometry_node.py` → `/wheel/odom`;
`config/ekf.yaml` + `launch/pilot.launch.py` → `/odometry/filtered` (wheels + IMU);
`odometry/move_test.py` has a `Mover` with straight/turn helpers (a useful reference, but it
belongs to the study; reuse or import, don't modify). The RQ1-B encoder-targeted stopping
rule is in `config/pilot.yaml` (gain × remaining distance, clamped 0.05–0.15 m/s, coast
compensation). Treat everything under `walle-pilot/` as read-only unless I agree otherwise.

**Simulator (`walle-sim/`).** `nodes/sim_pico_bridge.py` stands in for the Pico with the
same topics, so the real robot-side code runs unchanged. `nodes/firmware_models.py` has
`OpenLoopPwmFirmware` (a line-for-line copy of `drive_control.c`) and a `PidFirmware`
candidate (open-loop feedforward + per-wheel PI on encoder speed, kp 1.2, ki 6.0). Profiles
in `config/sim_profiles.yaml`: `ideal`, `pilot_firmware` (open loop, motor gains
1.01/0.99), `pid_firmware`, `stress`. `harness/sim_trials.py` is the test harness (add a
subcommand for the route check, like Phase 0's `park-check`); `harness/sim_world.py` reads
ground truth. Park world: `make sim-launch WORLD=park SCENARIO=default GUI=false`.

**Pilot results** (`walle-pilot/data/results/pilot_summary.md`, n = 5, pilot scale): 50 cm
straight drives end within about 1 cm (lateral error up to 2.7 cm); ~90° open-loop turns
have heading errors around 5° (EKF) to 7° (wheel-only), worse clockwise (9–15°). The route
has four 90° turns per lap, so heading error is likely to be the main source of drift.

## Real-world conditions (may change; I'll tell you)

- Bins for now are paper bags, **30 × 18 × 36 cm** (L × W × H); plastic bins come later.
  `park.yaml` models bins as cylinders, so a bin-shape decision is needed (see below).
- The square is not taped out yet. Testing is on **carpet** (cardboard later), so expect
  more slip than in the pilot. Consider a sim profile for that.
- The robot runs on the current firmware.

## Open decisions to settle in plan mode

Recommend an option for each; record the outcomes in the `ROADMAP.md` decision log.
1. Closed-loop control on the computer (a ROS node) or in the Pico firmware (C). Weigh:
   the sim tests the exact computer-side code; firmware needs a reflash, and the sim
   `PidFirmware` models it; Wi-Fi delay and drops matter for computer-side control.
2. Pass criteria: N laps in a row, end within X cm of the start (sim), plus what to measure
   on the robot.
3. Corner behaviour: stop and turn in place vs. a rounded turn.
4. Bin stops: where (at the corner, or turned to face the bin) and how long. The actual bin
   check is Phase 4.
5. Bin shape in `park.yaml` for rectangular bags (box vs. circumscribed cylinder), and how
   bins are oriented.
6. Drift: measure first; decide later whether markers (e.g. AprilTags) are needed.
7. Safety: the route follower must keep an emergency-stop path. Decide how a run is
   started and stopped (approval for the whole run is Phase 6's question, but a stop
   command or topic is needed now).

## How to work

- Sim first: a headless one-command check that reports pass/fail, before anything runs on
  the robot. A phase is done only after I run it on the real robot.
- Settings in YAML under `config/`, results in CSV/JSON under `results/` or `data/`.
  New nodes use `rclpy`; follow the `/x/ask` + `/x/result` JSON pattern if a request/response
  interface is needed.
- Add a `LEDGER.md` entry before ending any turn that changed files: what was verified and
  how, what wasn't, and any scope drift. Tick the `ROADMAP.md` boxes as they are met.
- Don't commit or push unless I ask. Never commit `wifi_config.h`.
