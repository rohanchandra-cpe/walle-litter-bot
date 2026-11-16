# WALL-E application

The WALL-E park inspection robot (see [`ROADMAP.md`](../ROADMAP.md)). It reuses the robot code
in `walle-pilot/` without changing the frozen research-study files.

So far (Phase 0) this folder holds the park: its geometry, the test scenarios, and the loader
that the simulator uses now and the robot will use from Phase 1.

```
config/park.yaml        the fixed park: route square, bin size and placement, robot footprint,
                        and the items the simulator can place
config/scenarios/       what is in the park for one test: bins, their states, the obstacle
park.py                 loads park.yaml + a scenario (+ overrides) -> the full layout
tests/test_park.py      python3 -m unittest discover -s walle-app/tests
```

## The park

A 1 m × 1 m square route. Corner 1 is the origin; the robot starts there facing corner 2 and
drives counter-clockwise 1 → 2 → 3 → 4 → 1. Each corner can have a bin, set **outside** the
corner on the line from the square's center through it, far enough out that the robot clears
it while driving either side and while turning on the corner (`bins.offset_m: auto`; 0.34 m
on the diagonal for a 20 cm bin). Obstacles stand **on** the route, at most one per lap.

```
  bin 4           bin 3
       4 ------ 3
       |        |
       |        |
       1 ------ 2
  bin 1           bin 2
       ^ start, facing 2
```

The bin size (20 cm across, 20 cm tall) is a placeholder: measure the real bins and update
`bins.radius_m` and `bins.height_m`. The auto offset follows.

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
