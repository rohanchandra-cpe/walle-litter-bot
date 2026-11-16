# Work ledger

A running record of the work done in this repository, so a new session (or a new agent) can
see what has been done and why, and so changes in scope are noticed instead of drifting in.

## Rules

- **Append only, newest entry at the bottom.** Don't rewrite old entries. To correct one, add
  a new entry that says what was wrong.
- **One entry per piece of work** (usually one session, or one turn that changed files).
- **Write it before ending the turn**, while the details are fresh.
- **Be exact about verification.** "Ran in sim, passed", "ran on the robot", "not run",
  "only read the code". Never claim a check that didn't happen.
- **Scope check every time.** Compare what was asked with what was done, and with the
  current phase in `ROADMAP.md`. If they differ, say how and why under "Scope".

## Entry template

```markdown
### YYYY-MM-DD: <short title>

- **Phase:** <roadmap phase number and name, or "none">
- **Branch / commit:** <branch> @ <short hash, or "uncommitted">
- **Asked for:** <the user's request, in a sentence or two; quote key words>
- **Done:**
  - <change> (`path/to/file`)
- **Verified:** <what was checked, and how: sim / robot / tests / not verified>
- **Scope:** <"In scope" or "Drift: <what moved, why, and whether the user agreed>">
- **Decisions:** <decisions made, also copied into the ROADMAP decision log; or "none">
- **Open / next:** <what is left, what the next session should start with>
```

---

## Background (before this ledger)

Work done before the ledger started, summarized from git history:

- **Research study code (RQ1–RQ3)** in `walle-pilot/`: pilot protocol scripts, frozen config,
  Pico firmware with the HC-SR04, depth calibration, and analysis (PRs #1–#3).
- **ROSA agents and tools** for odometry, vision, depth, and range, including distance tools
  (PRs #2–#3).
- **Layout consolidation** (PR #4): `rosa_agent/` folded into the subsystem folders; root
  `Makefile` for Compose.
- **Gazebo simulator** `walle-sim/` with test worlds for RQ1–RQ3 and an inspection room for
  the agents (commit `7fc6c67`, branch `gazebo-test-environments`, **not yet merged into
  `main`**). The simulated robot uses the Module 6 model on purpose.

---

## Entries

### 2026-11-15: Planning docs for the WALL-E application

- **Phase:** none (set-up before Phase 0)
- **Branch / commit:** `walle-roadmap-docs` (from `origin/main` @ `0acd5d0`) @ see git log
- **Asked for:** start the WALL-E application work beyond the research questions; create a new
  branch off `main` with a `CLAUDE.md`, a `ROADMAP.md`, and a work ledger, and work with plan
  mode at the start of each phase and sim-first checks.
- **Done:**
  - Agent guidance: session start steps, working rules, hardware, topics, layout, commands
    (`CLAUDE.md`)
  - Eight phases with work, "done when" criteria, and open decisions (`ROADMAP.md`)
  - This ledger with its rules and template (`LEDGER.md`)
- **Verified:** documentation only, nothing run. Facts in `CLAUDE.md` were taken from the code
  on `origin/main` (firmware topic setup in `pico_pilot.c`, `Makefile`, `docker-compose.yml`,
  `walle-pilot/README.md`, `ROSA_AGENT.md`) and from `walle-sim/README.md` on
  `gazebo-test-environments`.
- **Scope:** In scope.
- **Decisions:** Branch from `origin/main` rather than local `main`, which was behind (missing
  PR #4).
- **Open / next:**
  - `walle-sim/` is not on `main`, so sim-first checks can't run from this branch until it is
    merged (Phase 0, first item).
  - The servo is not in the firmware, and the micro-ROS library has no free publisher or
    subscription slot (see Phase 4 open decisions).
  - Next session: plan mode for Phase 0.

### 2026-11-15: Phase 0, park config and configurable Gazebo park world

- **Phase:** 0, Foundations
- **Branch / commit:** `walle-roadmap-docs` (rebased onto `origin/main` @ `e425042`, which now
  has `walle-sim/` from PR #5) @ see git log
- **Asked for:** start Phase 0 with a very simple park: a "1 meter by 1 meter square", bins
  "on each corner", obstacles "on the straight paths between the bins", "1 obstacle per
  square", and the ability to "change the number of bins, the state of the bins, and whether
  or not an obstacle appears for testing purposes". Plan approved in plan mode with answers:
  one obstacle per lap, bins just outside the corners, states overflowing + litter around,
  code in `walle-app/`. The user asked that the item catalog not limit future YOLO-World
  detection, and that the plan file be named like "phase 0 plan".
- **Done:**
  - Park geometry, robot footprint, bin size (placeholder), sim item catalog
    (`walle-app/config/park.yaml`)
  - Scenarios `default`, `mixed`, `obstacle_only` (`walle-app/config/scenarios/`)
  - Loader with overrides (bins, obstacle, seed), clearance checks, per-bin/obstacle seeded
    random streams, CLI (`walle-app/park.py`); 10 unit tests (`walle-app/tests/test_park.py`)
  - Gazebo world generator into git-ignored `worlds/generated/`
    (`walle-sim/worlds/park_world.py`, `walle-sim/.gitignore`)
  - `world:=park` + `scenario`/`bins`/`obstacle`/`park_seed` launch args
    (`walle-sim/launch/sim.launch.py`); generated park info merged into the world info
    (`walle-sim/harness/sim_world.py`); `park-check` command with `--look-at` photos
    (`walle-sim/harness/sim_trials.py`)
  - `SCENARIO`/`BINS`/`OBSTACLE`/`SEED`/`LOOK_AT` variables, `sim-park`, `sim-park-check`
    targets (`Makefile`)
  - Docs: `walle-app/README.md`, `walle-sim/README.md` (park row), `CLAUDE.md` (layout,
    commands, plans folder; removed the "walle-sim not merged" note), `ROADMAP.md` (Phase 0
    ticks, decision log), approved plan copied to `docs/plans/phase-0-plan.md`
- **Verified:** in the sim container (headless, RTF 1.00):
  - unit tests pass on host and in the container; generated SDF for all 3 scenarios and an
    override case is `ign sdf -k` valid;
  - `world:=park scenario:=mixed`: `sim-check` all OK; `park-check` PASS, 25 models at their
    positions (0.0 cm error), no strays; photos of bins 1–4 and the obstacle show ok bins,
    a level-1 and a level-3 heap, litter, and the upright bottle on side 3;
  - relaunched with `BINS=2 OBSTACLE=none`: `park-check` PASS, only bins 1–2, no obstacle;
  - `motion_area` still launches and `sim-check` is all OK (RQ worlds unaffected).
  - Not verified: anything on the real robot or a real park (nothing to run there yet). The
    RQ harness commands (rq1/rq2/rq3) were not re-run; their code paths are unchanged
    except `load_world_info()`, which only adds the park entry.
- **Scope:** In scope. Small additions beyond the request: a `park-check` command with
  photos (needed to verify), a `docs/plans/` folder (user asked for a readable plan name).
  Litter is placed only on the outward side of each bin so it never blocks the route; from
  the route it is partly hidden behind the bin. Revisit if bin checks need to see it.
- **Decisions:** all in the ROADMAP decision log under 2026-11-15, Phase 0.
- **Open / next:**
  - User: measure the real bins (update `bins.radius_m`/`height_m`) and tape out the real
    square; then tick the last Phase 0 box.
  - Changing the park inside a running simulation (without relaunching) is not supported.
  - Next phase: Phase 1 (route following), starting in plan mode.

### 2026-11-16: Real-world facts for Phase 1, and session handoff prompts

- **Phase:** between 0 and 1
- **Branch / commit:** `implement-core-walle-functionality` (renamed from `walle-roadmap-docs`
  at the user's request; the old remote branch was deleted, no PR existed) @ after `e1b61c2`
  (Phase 0 is committed and pushed, not merged to `main`)
- **Asked for:** record the user's answers about the real setup; write a summary prompt of
  this session and a prompt that starts Phase 1 in a new session ("a separate session for
  each phase").
- **Done:**
  - Recorded in `ROADMAP.md` (Phase 0 last box, Phase 1 known conditions and a new open
    decision): interim bins are Trader Joe's paper bags, 30 × 18 × 36 cm (L × W × H); plastic
    bins from Target later; square not taped yet; testing on carpet (cardboard later); robot
    still runs on the current firmware. The user said these may change.
  - `docs/handoff/session-2026-11-15-summary.md` (what this session did) and
    `docs/handoff/phase-1-prompt.md` (start prompt for the Phase 1 session).
- **Verified:** documentation only. Firmware facts in the Phase 1 prompt were read from
  `pico_pilot.c`, `drive_control.c`, `walle-sim/nodes/firmware_models.py`,
  `walle-sim/config/sim_profiles.yaml`, and `walle-pilot/data/results/pilot_summary.md`.
- **Scope:** In scope. `park.yaml` was NOT changed for the paper bags: bins are cylinders in
  the model, and the shape change is left to the Phase 1 plan.
- **Decisions:** handoff prompts live in `docs/handoff/`.
- **Open / next:**
  - Merge `implement-core-walle-functionality` to `main` so Phase 1 branches from `origin/main`.
  - Phase 1 plan mode: bin shape for the paper bags, PID location, N/X pass criteria, corner
    and bin-stop behaviour, carpet slip in the sim profile.

### 2026-11-16: Phase 1, route follower, box bins, sim route check

- **Phase:** 1, Route following
- **Branch / commit:** `walle-phase-1` (from `implement-core-walle-functionality` @ `3c01fd9`,
  at the user's request; Phase 0 is not merged to `main`) @ uncommitted
- **Asked for:** Phase 1 in plan mode. The user's answers: build on module 7 and module 8
  (`Ollama/module-7-kinematics-control`, `module-8-sensor-fusion`, firmware + odometry);
  1 lap, end within 5 cm; stop and turn in place; at each corner "turn to face the bin so that
  the whole bin is in the center of the camera frame", wait 15 s; bins as rectangles ("might
  become cylinders later"); measuring drift is "EXTREMELY important"; the user starts runs and
  stops them with Ctrl+C. Asked whether the whole bag must fit, the user chose "move bins
  farther out".
- **Done:**
  - Module 8's `move_test.py`, `odometry_node.py`, `transforms.py` and `pico_fusion` firmware
    are byte-identical to `walle-pilot/`; module 7's `pico_pid` (firmware wheel PI) was read,
    not used. Plan approved, copied to `docs/plans/phase-1-plan.md` (+ "changes while
    building" section).
  - Box bins: `shape: box` 30 × 18 × 36 cm, wide face toward the square; clearance now
    checks the bin's outline against both sides as full lines and the turn circle (0.43 m);
    new `robot.camera` block and `frame_from_corner` set the offset so the whole bin is in
    frame from the corner (0.83 m). Layout gains `shape`, `size_m`, `yaw_deg`,
    `view_heading_deg`. Cylinders still load (`walle-app/config/park.yaml`,
    `walle-app/park.py`, `walle-sim/worlds/park_world.py`).
  - Route follower subclassing module 8's `Mover` (imported, unchanged): line following,
    absolute-heading turns, bin stops, learned coast, correction moves, gyro-drift
    measurement and zero-velocity update, `/walle/route/status`, run logs, end-of-run tape
    measurement prompt and `results/route/robot_runs.csv` (`walle-app/route/route_follower.py`,
    `walle-app/route/route_geometry.py`, `walle-app/config/route.yaml`).
  - Tests: 5 new park tests (box clearance, framing, yaw, refusal, cylinder), 5 route
    geometry/plan tests (`walle-app/tests/`).
  - `route-check` harness command (start pose, follower as subprocess, ground-truth judging,
    bin-in-frame projection, photos) and `make sim-route-check`; `carpet` and `carpet_pid`
    sim profiles (`walle-sim/harness/sim_trials.py`, `walle-sim/config/sim_profiles.yaml`,
    `Makefile`).
  - Docs: `walle-app/README.md` (bins, route following, robot steps), `walle-sim/README.md`,
    `CLAUDE.md` (layout, commands), `ROADMAP.md` (Phase 1 status, tick, decisions).
- **Verified:**
  - 20 unit tests pass on the host (Python 3.13) and in the sim container.
  - Gazebo, world `park`, scenario `default` (4 bins, no obstacle), headless, RTF 1.00:
    `park-check` PASS with the box bins (0.0 cm); photo shows the whole bag centered.
  - `route-check`, 1 lap, 15 s waits, final code, one run per profile, all PASS:
    `ideal` 0.6 cm; `pilot_firmware` 1.6 cm (corners 1.5–1.7 cm); `carpet` 4.7 cm (corners
    1.0–4.7 cm); `carpet_pid` 1.3 cm. Every bin wholly in frame; bearing to the bin up to
    9° (`pilot_firmware`) and 12° (`carpet_pid`), mostly on the first bin, before the turn
    coast is learned.
  - Earlier failing runs, kept in `walle-sim/data/phase1/route/` (git-ignored): open-loop
    `pilot_firmware` with module 8's settings ended 5.2 cm off (2 s waits) and 7.3 cm
    (15 s waits) and missed the frame at all 4 bins; `carpet` 8.3 and 10.6 cm before the
    gyro-drift estimate.
  - **Not verified:** nothing has run on the real robot. The carpet profiles' values are
    guesses, and `carpet` passes by only 0.3 cm in a single run (run-to-run spread is not
    measured). `ideal` reports +0.8°/min of rest drift although it has no gyro bias,
    probably the EKF settling after the harness teleports the robot; harmless here but not
    explained. `--feedback /wheel/odom` and multi-lap runs were not run in sim.
- **Scope:** Drift, inside the approved plan's escalation step 1 but beyond what the plan
  listed: learned coast with correction moves, and gyro-bias estimation with a zero-velocity
  update (plus a 10 s still period before each run) were added because the first sim runs
  failed. The `carpet_pid` profile was added to size the firmware option. Told to the user
  in the session; recorded in the plan's "changes while building".
- **Decisions:** Phase 1 decisions 1–7 in the ROADMAP decision log (2026-11-16).
- **Open / next:**
  - User: measure the camera (`robot.camera.height_m`, `forward_m` in `park.yaml`), place
    the bags where `park.py` prints, and run the follower on the carpet. Record the tape end
    error; tick the two remaining Phase 1 boxes.
  - If the robot misses 5 cm: next escalation steps (wheel diameter and track calibrated on
    carpet, then module 7's firmware wheel PID: sim shows 4.7 → 1.3 cm on carpet).
  - Run several sim laps per profile to measure spread.
