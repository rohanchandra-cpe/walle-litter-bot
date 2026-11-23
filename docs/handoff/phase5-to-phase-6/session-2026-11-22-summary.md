# Session summary: Phase 5, obstacle detection and avoidance (2026-11-22 to 2026-11-23)

Paste this into a new session (or use it as the `/compact` instruction) to carry over what
this session did. The durable record is `LEDGER.md`; this is the short version.

---

You are continuing work on the WALL-E repository (`walle-litter-bot`). Read `CLAUDE.md`,
`ROADMAP.md` and the latest `LEDGER.md` entries first. What happened in the previous
session:

**Branch.**
- **Phase 4:** merged to `main` in PR #12, with its handoff commit.
- **Phase 5:** built on `walle-phase-5`, committed as **`f5e3f27`**, and merged to `main`
  in **PR #13** (`4cce337`).
- **Phase 6:** `walle-phase-6` was cut from `origin/main` after that merge. Its first
  commit, **`bd1614e`**, cleans the robot memory store (below). Pushed; no PR yet.
- **Still uncommitted, on purpose:**
  - `walle-app/data/photos/_props.jpg`;
  - the three 340 MB CLIP weight copies (`weights/`, `walle-app/weights/`,
    `walle-pilot/weights/`; never commit them);
  - `.DS_Store` and `.vscode/`;
  - this folder, `docs/handoff/phase5-to-phase-6/`, until the user says to commit it.

  Ask the user before committing any of these.
- **Phase 5 is ✅ closed with recorded limits** (see Results).

**Decisions** (ROADMAP decision log, 2026-11-22 and 2026-11-23):
- **Sensing:**
  - **Corner look:** before each side, the robot stands on its corner, tilts the camera
    15° down (the sim clamps to −10°) and takes one photo. The floor is in view from
    ~0.28 m ahead of the axle to the far corner, flat items included.
  - **Sonar:** while driving, the HC-SR04 stops the robot at 3 pings in a row ≤ 0.35 m
    that are also nearer than the end of the current move.
- **HC-SR04 check (robot, Step 0):** it answers once plugged back in.
  - 40 cm reads 40.0 cm; the milk jug at ~30 cm reads 32 cm, steady to ±0.5 µs; noisier
    at 80 cm. The sensor is 8.8 cm above the floor (the sim keeps 7 cm).
  - A flat bag or paper towel gives **no echo**; a crumpled bag does (27 cm). Only the
    camera finds flat items.
- **Detour:** inside the square only (left of the counter-clockwise route).
  - The shape is a rectangle of turns in place and straight legs: out, past, back to the
    line or straight to the next corner.
  - Sideways distance = object radius + half width 0.086 + margin 0.05 + uncertainty
    0.03 m. The widest allowed detour is **0.50 m** (was 0.40; a flat bag needed 0.41).
  - Turn back only past an assumed **0.25 m depth** beyond the object's near face, since
    a photo can't see depth; near a corner this shrinks to the measured size.
  - **Neighbouring obstacles share one detour.**
  - **Every detour is checked against obstacles found earlier on other sides** (its turns
    and legs keep the margin).
- **Fallback:** no way around → stop, record it, end the run with status `blocked`.
- **What counts as an obstacle:** anything ≥ **4 cm** wide as seen (was "avoid
  everything"; changed after a ~2 cm speck stopped a run).
- **Far sightings are unconfirmed:** an item the corner look places more than 0.7 m away
  gets a second look from 0.6 m before anything is done about it. A sonar echo also
  confirms it.
- **Tape:** the detector drops boxes its find step names with "tape"/"marking"
  (`detect.yaml` `vlm.reject_names`). **Loose tape patches on the test floor are removed**
  (user): the detector reads a flat tape rectangle as paper, and no prompt wording fixed
  it.
- **Recording:**
  - `obstacles.json` per run: every look, its items with the model's own words, the
    rejected boxes, and each obstacle's record.
  - The memory import calls `report_obstacle`, and `mark_absent` for store obstacles on
    sides the run drove without seeing them.
  - **Reviewed false positives** go in a per-run `obstacle_review.json` that the import
    applies; the robot's own log stays as written.
- **Targets (met):** sim 9 of 10 with no collisions and positions within 10 cm; robot no
  contact in at least 2 of 3 runs.

**What was built:**
- **`walle-app/route/obstacle.py`** (no ROS):
  - `footprint_from_box`: floor position, radius, seen width, distance;
  - `blocks_side`, `SonarTrigger`;
  - `plan_detour` / `plan_detour_group` / `group_ahead`;
  - `plan_clearance`, `min_clearance`.
- **`route_follower.py`:**
  - `look_side`, `relook` and `drive_with_obstacles`;
  - a sonar `watch` in `side_leg`, `on_sonar_hit`;
  - `ObstacleBlocked` → status `blocked`;
  - flags `--no-obstacles` and `--sim-hide-obstacle`;
  - the follower is never told where the obstacle is.
- **`route_geometry.build_plan(look_sides=)`:** a `look_side` step before each side.
- **`config/route.yaml` `obstacle:`** (with a sim override: the sensor is 7 cm ahead
  there).
- **Detector:** `vlm.reject_names`; the not_trash line mentions tape.
- **`memory/route_records.py`:** `record_obstacles`, `load_obstacle_review`.
- **`detect/replay_obstacles.py`:** scores a run against the truth, or
  `--truth "SIDE,ALONG_M[,CROSS_CM]"`.
- **Sim:**
  - `sim_truth.py` projects the obstacle;
  - scenarios `obstacle_near`, `obstacle_far`, `obstacle_lying`;
  - profile `plastic_pid_sonar_dead`;
  - `route-check` scores obstacles (found and passed, position, false obstacles,
    collision against the true outline, memory);
  - **`harness/obstacle_trials.py` + `make sim-obstacle-check`**: 10 cases, each in its
    own headless world.
- **Tests:** 134 in `walle-app/tests/` (`test_obstacle.py` is new).

**Results:**
- **Sim** (`make sim-obstacle-check`): **9/10**, zero collisions, positions 0.3–9.4 cm.
  - The miss: sonar only, object near the far corner → `blocked`. Its size is unknown,
    so the default 10 cm radius leaves no room to turn.
- **Detector eval** (Phase 3 set, after the tape changes): recall 0.812, precision 0.812,
  category 0.973. Read as "no regression"; the tape filter removed nothing in that set.
- **Robot**, nine runs. Placements were "about", not tape-measured:

  | Run | Obstacle | Outcome |
  |---|---|---|
  | `20261122_204947` | can, side 1 | passed; floor tape on side 3 read as two items → blocked |
  | `20261122_211939` | can, side 1 | passed; a 2 cm speck past corner 4 placed inside the turn zone → blocked |
  | `20261123_034402` / `035324` / `040037` | can / jug / bag | can and jug passed (5.4 / 5.1 cm); a tape patch detoured each time; bag blocked at 0.41 m > 0.40 |
  | `20261123_042112` / `042752` | can / jug (tape removed) | passed, no contact, 5.6 / 7.3 cm, end 0 cm |
  | `20261123_043408` | bag | passed (7.3 cm), then **touched**: side 3's detour around a leftover tape patch swept into the bag on side 2 (fixed afterwards: the other-sides check) |
  | `20261123_045424` | bag | three tape patches on side 1 still in place (one detour); bag blocked at corner 3 (its far edge 16 cm away, needs 16.5) |

- **Limits recorded at close:**
  - flat paper-like things on the floor are read as paper;
  - far camera positions are ~30% short at 1 m (the down-tilted camera is uncalibrated;
    the 1.00 m cross tape read 0.69 m);
  - sonar-only objects near a side's end block;
  - grouping and the other-sides check are unit-tested only (the sim has one obstacle per
    lap);
  - labels can be wrong (the plastic bag was once `paper_bag`).
- **Memory cleanup (`bd1614e`):**
  - 9 false obstacle records (tape and the speck) are listed in `obstacle_review.json` in
    six run folders, plus one relabel (`paper_bag` → `plastic_bag`).
  - `robot.sqlite` was rebuilt (old copy: `robot.sqlite.bak-20261123`). It now holds 20
    runs and 6 obstacles (can ×2, jug ×2, bag ×2).
  - **Current: only the plastic bag on side 2**, from run `045424`. If the bag has been
    picked up, a lap over side 2 clears it.

**Gotchas:**
- **The sim harness must stop every node of a launch.** A first 10-case batch scored 3/10
  because twelve leftover `odometry_node.py` copies all published `/wheel/odom`; the
  jump guard then "repaired" the estimate from garbage.
  - `obstacle_trials.py` now kills each launch's process group and refuses to start while
    any sim node is left over.
  - When it refuses, clear the leftovers with bracketed `pkill` inside the container.
- **`make sim-obstacle-check` needs no launch running.** It starts its own worlds, ~2.5 min
  per case.
- **The detector's naming step sees only a crop.** A flat white rectangle looks like paper
  to it; context-based prompting doesn't help. Keep the test floor clear.
- **Whether a flat tape patch gets flagged varies between runs:** the same patch can be
  clear in one run and an obstacle in the next.
- **Robot results are written only after the end-of-run measurement prompts.**
  `obstacles.json` appears then. Pressing Enter skips them.
- **zsh:** a `for x in "a b"` loop doesn't word-split `$x`. Write each command out.
- **The checkout can change under you:** the user merged PR #13 and switched to `main`
  mid-session. **Check `git status -sb` right before every commit.**
- **`gh` is not installed;** the user opens PRs in the browser.

**Open:**
- **Calibrating the down-tilted camera:** about 5 minutes with the robot (the can at
  tape-measured 0.5 / 1.0 m).
- **The other-sides clearance check** has not acted on the robot yet.
- **Next:** Phase 6 (the WALL-E ROSA agent). Start prompt in
  `docs/handoff/phase5-to-phase-6/phase-6-prompt.md`.
