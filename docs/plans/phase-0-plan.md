<!-- Approved plan for ROADMAP Phase 0, 2026-11-15. What was actually built is in LEDGER.md. -->

# Phase 0: park config + configurable Gazebo park world

## Context
Phase 0 of `ROADMAP.md`. `walle-sim/` is now on `main` (PR #5) and this branch is rebased onto
it. The user wants a simple test park: a **1 m × 1 m square route**, **bins at the corners**,
**at most one obstacle per lap** on a straight side, and the ability to change the **number of
bins**, **each bin's state**, and **whether an obstacle appears**, for testing.

Decisions from the user (go into the ROADMAP decision log):
- One obstacle per lap at most (side chosen or random).
- Bins sit **just outside** each corner; the robot drives the square itself.
- Bin states: **overflowing** (with a degree) and **litter around** (combinable). A bin that
  is neither is "ok". No empty/half/full levels, no trash/recycling bin type for now.
- Application code lives in a new top-level **`walle-app/`**.

## Design

**One source of truth, two layers**
- `walle-app/config/park.yaml` — fixed geometry shared by robot, sim and (later) agent:
  square side 1.0 m, origin at corner 1 with the robot starting there facing +x, route
  corners 1→2→3→4 counter-clockwise, bin size (Ø20 cm × 20 cm placeholder until the real bins
  are measured), bin offset outward along the corner bisector, and an item catalog
  (`plastic_bottle`, `can` → recycling; `paper_cup`, `wrapper` → trash) with simple shapes.
  **The catalog only says what the simulator can place** (and gives the ground-truth label).
  It must not limit detection: Phase 3's open-vocabulary detector (YOLO-World) reports
  whatever it sees, and trash/recycling for an unseen item is decided by the classifier, not
  looked up in this list. A comment in `park.yaml` and the README will say so.
- `walle-app/config/scenarios/*.yaml` — what is in the park for one test:
  ```yaml
  bins:            # omit a corner = no bin there
    1: {state: ok}
    2: {state: overflowing, level: 2}   # level 1 slight … 3 heavy
    3: {state: ok, litter: 3}           # items on the ground beside it
    4: {state: overflowing, level: 3, litter: 2}
  obstacle: {side: random, at: random, item: random}   # or `obstacle: none`; side 1–4, at 0.25–0.75
  seed: 7
  ```
  Shipped scenarios: `default` (4 ok bins, no obstacle), `mixed` (the example above),
  `obstacle_only` (4 ok bins, random obstacle).
- **Quick overrides** at launch without editing files: `BINS=<n>` (keep the first n corners),
  `OBSTACLE=none|random|<side>`, `SEED=<n>`.

**Shared loader — `walle-app/park.py`** (new; robot code will import it in Phase 1)
- `load_park(scenario, overrides)` → resolved layout: corner points, route waypoints, each
  bin's id/position/state/level/litter positions, obstacle id/side/position/item/category.
  Random choices come from the seed, so a scenario is reproducible.
- Sanity checks that raise clear errors: bin footprint clears the path by at least robot
  half-width + margin (with a 20 cm bin this forces the offset to ~0.33 m on the diagonal, so
  the default offset is computed, not guessed), obstacle away from corners, litter placed on
  the outward side of the bin so it never lands on the route.
- `python3 walle-app/park.py --scenario mixed [--bins 3] [--obstacle 2]` prints the layout.

**Gazebo world — `walle-sim/worlds/park_world.py`** (new)
- Reuses `header`, `FOOTER`, `material`, `mark` from `walle-sim/worlds/generate_worlds.py`
  (import, no copy).
- Draws: tape square on the floor, corner marks, a start pad; each bin as a static cylinder
  with a dark disk at the rim (the opening); overflowing = a mound of item shapes above the
  rim whose height grows with `level`, plus one item spilled at the base; litter = item
  shapes on the floor outside the bin; the obstacle = one catalog item on its side.
- Writes `walle-sim/worlds/generated/park.sdf` and `park_info.yaml` (spawn, view, anchors,
  and the resolved layout as ground truth). `generated/` is git-ignored: scenarios are the
  committed artifact.

**Hook into the existing sim (small edits)**
- `walle-sim/launch/sim.launch.py`: when `world:=park`, run the generator with the new
  `scenario`, `bins`, `obstacle`, `seed` launch args before starting Gazebo, and read the
  spawn from `park_info.yaml`. Other worlds unchanged.
- `walle-sim/harness/sim_world.py` `load_world_info()`: also merge `generated/park_info.yaml`
  if present, so `World`/ground truth work in the park.
- `walle-sim/harness/sim_trials.py`: add a `park-check` command — compares every expected
  model (bins, mound/litter items, obstacle) against `/sim/ground_truth` positions and
  reports pass/fail; also saves one robot-camera photo to the run folder.
- `Makefile`: pass `SCENARIO`, `BINS`, `OBSTACLE`, `SEED` through `sim-launch`; add
  `make sim-park-check`.

**Docs**: copy this plan into the repo as `docs/plans/phase-0-plan.md` (phase plans kept
there from now on, named `phase-N-plan.md`); `walle-app/README.md` (park, scenarios, overrides); `CLAUDE.md` (walle-sim now
merged, `walle-app/` in the layout, park commands); `ROADMAP.md` Phase 0 ticks + decision
log; `LEDGER.md` entry.

Out of scope now: moving/adding things in a running world without relaunching, real-world
park measurement (user), bin types.

## Verification (sim container: `make sim-up`)
1. `python3 walle-app/park.py` for each scenario and with overrides (`--bins 2`,
   `--obstacle none`, two seeds) — layouts print, same seed gives same layout, clearance check
   rejects a too-small bin offset.
2. Generate each scenario and validate the SDF with `ign sdf -k`.
3. `make sim-launch WORLD=park SCENARIO=mixed GUI=false`, then `make sim-park-check` and
   `make sim-check` — all models found at the expected positions, robot publishes as usual.
4. Relaunch with `BINS=2 OBSTACLE=none` and re-run `park-check` — only 2 bins, no obstacle.
5. Look at the saved camera photo (and optionally the GUI at localhost:6080) to confirm
   overflowing/litter bins look different from ok ones.
6. Confirm the RQ worlds still launch (`WORLD=motion_area`, `sim-check`).
