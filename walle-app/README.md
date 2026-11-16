# WALL-E application

The WALL-E park inspection robot (see [`ROADMAP.md`](../ROADMAP.md)). It reuses the robot code
in `walle-pilot/` without changing the frozen research-study files.

This folder holds the park (Phase 0: its geometry, the test scenarios, and the loader that the
simulator and the robot both use) and the route follower (Phase 1).

```
config/park.yaml        the fixed park: route square, bin size and placement, robot footprint,
                        and the items the simulator can place
config/scenarios/       what is in the park for one test: bins, their states, the obstacle
park.py                 loads park.yaml + a scenario (+ overrides) -> the full layout
config/route.yaml       route follower settings: speeds, gains, tolerances, bin wait, coast
route/route_follower.py drives the route on the EKF (Phase 1); route_geometry.py its math
results/route/          route runs: one folder per run, robot_runs.csv (one row per robot run)
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

**Bins** are boxes (`bins.shape: box`): the interim paper bags, 30 × 18 × 36 cm (L × W × H),
with the wide 30 cm face toward the square. `shape: cylinder` with `radius_m` still works for
round bins. `bins.offset_m: auto` (distance from the corner to the bin's center, along the
diagonal) is the larger of:
- **clearance**: every point of the bin clear of the robot driving either side (each side
  extended past the corner as a full line) and turning on the corner: 0.43 m for the bags;
- **framing** (`frame_from_corner: true`): the robot on the corner, facing the bin, sees the
  whole bin with `robot.camera.frame_margin` of the frame left free: **0.83 m** for the bags
  with the placeholder camera (10 cm high, 7 cm ahead of the axle, 47° × 60° field of view).

**Measure the camera on the robot** (`robot.camera.height_m`, `forward_m`) and put the bags
at the offset `python3 walle-app/park.py` prints (bin 1 for example at (−0.587, −0.587)).

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
  and turns) is updated from each move's overshoot. A corner missed by more than 1.5 cm, or a
  turn by more than 3°, gets up to 2 short correction moves. Needed because the open-loop
  firmware cannot drive slowly (MIN_DUTY 0.40: any turn spins at about 3 rad/s);
- **gyro drift**: it stands still 10 s at the start to measure the EKF heading's drift at rest
  (the gyro bias), adds each bin wait to that measurement, removes the drift from the heading
  while driving, and drops what built up during each wait (zero-velocity update, only when the
  encoders show the robot did not move);
- **progress** on `/walle/route/status` (JSON: state, corner, bin, distance driven, pose),
  printed as it goes; **Ctrl+C stops the robot** at any time (and the Pico stops 500 ms after
  the last command). A side not finished in 30 s aborts the run.

Each run writes `results/route/<run>/`: `trajectory.csv` (fused, wheels-only and, in Gazebo,
true pose at 10 Hz), `summary.json` (events per corner and bin, end errors, learned coasts,
measured gyro drift), `layout.json`, and `photos/` (one per bin stop, if a camera URL is set).

### In Gazebo

```bash
make sim-launch WORLD=park SCENARIO=default GUI=false PROFILE=pilot_firmware
make sim-route-check                  # LAPS=1, DWELL=15 (seconds at each bin)
```

`route-check` puts the robot on the start, runs the follower, and judges it with Gazebo's
ground truth: PASS if the run finishes, ends within 5 cm of the start, and every bin was
**wholly in the camera frame** at its stop (the box's corners projected into the simulated
camera). Results and photos go to `walle-sim/data/<RUN>/route/`.

### On the robot

1. Mark the start (corner 1) and the four corners on the floor; put the bags at the positions
   `python3 walle-app/park.py` prints, wide face toward the square.
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

