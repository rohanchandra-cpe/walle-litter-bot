# WALL-E application

The WALL-E park inspection robot (see [`ROADMAP.md`](../ROADMAP.md)). It reuses the robot code
in `walle-pilot/` without changing the frozen research-study files.

This folder holds the park (Phase 0: its geometry, the test scenarios, and the loader that the
simulator and the robot both use), the route follower (Phase 1), and the memory store
(Phase 2, [`memory/README.md`](memory/README.md)).

```
config/park.yaml        the fixed park: route square, bin size and placement, robot footprint,
                        and the items the simulator can place
config/scenarios/       what is in the park for one test: bins, their states, the obstacle
park.py                 loads park.yaml + a scenario (+ overrides) -> the full layout
config/route.yaml       route follower settings: speeds, gains, tolerances, bin wait, coast
route/route_follower.py drives the route on the EKF (Phase 1); route_geometry.py its math
results/route/          route runs: one folder per run, robot_runs.csv (one row per robot run)
memory/                 the memory store (Phase 2): write API, queries, schema, run import
config/memory.yaml      memory store settings; data/memory/ holds robot.sqlite, sim.sqlite
agent/                  the WALL-E ROSA agent (Phase 6): walle_agent.py, walle_tools.py, runner.py
config/agent.yaml       agent settings: LLM (qwen3.5:4b), store, approval, follower command
tests/                  python3 -m unittest discover -s walle-app/tests
```

## The park

A 1 m × 1 m square route. Corner 1 is the origin; the robot starts there facing corner 2 and
drives counter-clockwise 1 → 2 → 3 → 4 → 1. Each corner can have a bin, set **outside** the
corner on the line from the square's center through it. Obstacles stand **on** the route, at
most one per lap.

```
  bin 4           bin 3
       4 ------ 3
       |        |
       |        |
       1 ------ 2
  bin 1           bin 2
       ^ start, facing 2
```

**Bins** are boxes (`bins.shape: box`): black plastic bins, 27.7 cm wide, 21.1 cm deep and
29.2 cm tall (measured 2026-11-18), with the wide face toward the square. `shape: cylinder`
with `radius_m` still works for round bins. `bins.offset_m: auto` (distance from the corner
to the bin's center, along the diagonal) is the larger of:
- **clearance**: every point of the bin clear of the robot driving either side (each side
  extended past the corner as a full line) and turning on the corner: 0.43 m;
- **framing** (`frame_from_corner: true`): the robot on the corner, facing the bin, sees the
  whole bin with `robot.camera.frame_margin` of the frame left free: **0.71 m** with the
  measured camera (10.3 cm high, 12 cm ahead of the axle; 47° × 60° nominal field of view).

Put the bins at the positions `python3 walle-app/park.py` prints (bin 1 at (−0.502, −0.502)).

Each bin has a **type** (`bins.types`, 2026-11-18): bins 1 and 3 are trash, bins 2 and 4 are
recycling. A scenario can override one (`type: recycling`). The memory store uses it to judge
recycling correctness.

## Scenarios

```yaml
bins:                                 # leave a corner out = no bin there
  1: {state: ok}
  2: {state: overflowing, level: 1}   # level 1 slightly over the rim ... 3 heavily
  3: {state: ok, litter: 3}           # items on the ground beside the bin
  4: {state: overflowing, level: 3, litter: 2}
obstacle: {side: random, at: random, item: random}   # or: obstacle: none
seed: 7
```

- `state`: `ok` or `overflowing` (with `level` 1–3). Overflowing bins get a heap of items above
  the rim, taller at higher levels; levels 2 and 3 also spill 1 or 2 items at the base.
- `litter`: 0–6 items lying on the ground beside the bin, always on the outward side so they
  never land on the route.
- `obstacle`: `side` 1–4 (side k runs from corner k to k+1), `at` 0.25–0.75 along the side,
  `item` from the catalog, `pose` `upright` (default) or `lying`. Any of them can be `random`.
- `seed`: every random choice (items, positions) comes from it, so a scenario is
  reproducible. Each bin and the obstacle have their own random stream: removing a bin never
  moves the obstacle.

Shipped scenarios: `default` (4 ok bins, no obstacle), `mixed` (one of everything),
`obstacle_only` (4 ok bins, one random obstacle).

**Item catalog.** `items:` in `park.yaml` lists what the *simulator* can place, with each
item's ground-truth category (trash or recycling). It does not limit what the robot will
recognize: detection will be open-vocabulary (YOLO-World, Phase 3).

## Commands

From the repository root, with the simulator container up (`make sim-up`):

```bash
make sim-park SCENARIO=mixed                       # print the layout, no Gazebo
make sim-launch WORLD=park SCENARIO=mixed GUI=false
make sim-park-check LOOK_AT="bin_2 bin_4 obstacle" # everything in place? photos of each
```

Overrides, without editing a scenario: `BINS=0-4` (keep the first N corners' bins; more than
the scenario has adds `ok` bins), `OBSTACLE=none|random|1-4` (1–4 = that side), `SEED=n`.

```bash
make sim-launch WORLD=park SCENARIO=mixed BINS=2 OBSTACLE=none
```

`park-check` compares every bin, heap item, litter item and the obstacle with Gazebo's
ground truth (1 cm tolerance), fails on anything missing or extra, and saves robot-camera
photos to `walle-sim/data/<RUN>/photos/`.

Without the simulator: `python3 walle-app/park.py --scenario mixed --bins 2 --yaml`.

## Route following (Phase 1)

`route/route_follower.py` drives the route: corner to corner, stop and turn in place on each
corner, turn to face each bin and wait (`dwell_s`, 15 s), then on to the next side, and back
to the start facing the way it started. It steers by `/odometry/filtered` (the EKF: wheels +
BNO055) and publishes `/pico/cmd_vel`; the firmware is unchanged. It is built on module 8's
`Mover` (`walle-pilot/odometry/move_test.py`, imported, not changed), with:

- **line following**: each side follows the line between the corners (cross-track
  correction), so a turn that ends a few degrees off is corrected on the next side;
- **absolute headings**: turns go to the park's headings, so turn errors don't add up;
- **learned coast**: each move stops early by (speed × coast); the coast (separate for sides
  and turns) is updated from each move's overshoot. A corner missed by more than 1.5 cm gets
  up to 2 short correction moves; a turn missed by more than 3° gets up to 3 **correction
  pulses** (2026-11-19). A pulse is one turn command and a stop after a set time; it turns
  about gain × (time − 0.04 s), and the gain is learned from each pulse (start 110°/s, from
  walle-sim). A pulse is skipped if even the shortest would overshoot by more than the miss.
  Slow correction turns stuck and then lurched 8–13° when asked for 4–6° (robot run
  `20261119_043639`). The summary lists every pulse. This needs the closed-loop
  wheel speed firmware (2026-11-18): the old open-loop firmware turned at about 3 rad/s for
  any command, so a correction overshot by 10–15°. Turns run at 1.0–1.5 rad/s and the
  slowest drive speed is 0.08 m/s: below about 0.1 m/s per wheel the motors stick and
  lurch;
- **gyro drift**: it stands still 10 s at the start to measure the EKF heading's drift at rest
  (the gyro bias), adds each bin wait to that measurement, removes the drift from the heading
  while driving, and drops what built up during each wait (zero-velocity update, only when the
  encoders show the robot did not move);
- **progress** on `/walle/route/status` (JSON: state, corner, bin, distance driven, pose),
  printed as it goes; **Ctrl+C stops the robot** at any time (and the Pico stops 500 ms after
  the last command). A side not finished in 30 s aborts the run.

Each run writes `results/route/<run>/`: `trajectory.csv` (fused, wheels-only and, in Gazebo,
true pose at 10 Hz), `summary.json` (events per corner and bin, end errors, learned coasts,
measured gyro drift), `layout.json`, and `photos/` (one per bin stop, if a camera URL is set:
up to 3 tries, 3 s apart, within the bin wait; a failed stop records `photo_error`).
The run and one bin check per bin stop (photo only, not yet assessed) also go into the
memory store (`data/memory/sim.sqlite` or `robot.sqlite`; `--no-memory` skips it).

### In Gazebo

```bash
make sim-launch WORLD=park SCENARIO=default GUI=false PROFILE=plastic_pid
make sim-route-check                  # LAPS=1, DWELL=15 (seconds at each bin)
make sim-route-check COAST=0.35       # wrong starting coast: turns miss, pulses correct them
```

`route-check` puts the robot on the start, runs the follower, and judges it with Gazebo's
ground truth: PASS if the run finishes, ends within 5 cm of the start, and every bin was
**wholly in the camera frame** at its stop (the box's corners projected into the simulated
camera). Results and photos go to `walle-sim/data/<RUN>/route/`.

### On the robot

1. Mark the start (corner 1) and the four corners on the floor; put the bins at the positions
   `python3 walle-app/park.py` prints, wide face toward the square. The Pico must run the
   closed-loop firmware (`walle-pilot/firmware/pico_pilot/build/pico_pilot.uf2`, 2026-11-18).
2. `make up`, then `make launch` (odometry + EKF). Wait for `/odometry/filtered`.
3. Put the robot on the start mark facing corner 2, then in another terminal (`make shell`):

   ```bash
   python3 /workspace/walle-app/route/route_follower.py --scenario default \
       --camera-url http://CAMERA_IP/capture        # --bins N, --dwell 15, --laps 1
   ```

   Keep a hand on Ctrl+C. The robot stands still 10 s first (gyro drift), then drives.
4. At the end it asks for the tape-measured end offset from the start mark (x toward corner 2,
   y toward corner 4, cm), the end heading error, and whether each bin was centered in view.
   These go into the run's `summary.json` and a row of `results/route/robot_runs.csv`.


## The WALL-E agent (Phase 6)

One ROSA agent runs the park from plain-language prompts. Plan:
[`docs/plans/phase-6-plan.md`](../docs/plans/phase-6-plan.md).

- **Motion tools** start the route follower as a subprocess (`agent/runner.py`), so every
  follower guard applies:
  - `run_route` drives one lap;
  - `check_bin` drives the partial route to one bin and back (`--trip-bin N`);
  - `look_ahead` takes one down-tilted photo (`--look-only`);
  - `stop_robot` stops.

  Each asks `[APPROVAL NEEDED] … Allow? [y/N]` once, and that covers the whole run.
  **Ctrl+C stops the robot**, and the Pico stops by itself 500 ms after the last command.
- **Memory tools** only read the store (`memory/store.py` queries). The agent adds the
  store's name and run count to every answer from memory.
- ROSA's built-in ROS tools are left out, so the model can't publish to `/pico/cmd_vel`
  itself.

### Robot acceptance

1. Set up the park: `make up`, `make launch`, and the robot on corner 1 facing corner 2.
   - Bin 1 overflowing (~4 items on top).
   - A can on side 1, about halfway.
   - The camera at `config/agent.yaml` `route.camera_url`.
2. In `make shell`, start the agent:

   ```bash
   python3 /workspace/walle-app/agent/walle_agent.py
   ```

   It should say `motion REAL ROBOT` and `memory 'robot'`.
3. Send the prompts in this order, approving each motion:
   1. "Run your route and check the path for any obstructions." (a lap, 5–8 min)
   2. "Check Bin 1 to see if it is overflowing. If it is, estimate how much." (turn in place)
   3. "Historically, what objects have you seen the most?"
   4. "Are people generally recycling the correct objects?"
   5. "Which bins overflow most often?"
   6. "Where are the obstacles on the route right now?"
   7. "Check Bin 4." (side 4 driven clockwise and back)
   8. "Is anything in front of you right now?"
4. Check each answer:
   - Motion prompts: the right action, no contact, and the answer matches what is on the track.
   - History prompts: run `python3 walle-app/agent/facts.py` (host or container) **after**
     the motion runs, since they add to the store, and compare it with the answers.
