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

### 2026-11-18: Camera height measured; no firmware change for the first robot run

- **Phase:** 1, Route following
- **Branch / commit:** `walle-phase-1` @ after `596ba16`, uncommitted
- **Asked for:** the user set up the scene, measured the camera lens "10.3cm off the ground",
  and asked whether new firmware is needed before `make up` / `make launch`.
- **Done:** `robot.camera.height_m` 0.10 → 0.103 (`walle-app/config/park.yaml`). The auto bin
  offset drops from 0.83 to 0.82 m: bins at (±0.580 / 1.580) instead of 0.587 / 1.587, a 0.7 cm
  shift in x and y. `forward_m` is still the 0.07 m placeholder.
- **Verified:** unit tests pass (host); `park.py` prints the new positions. Nothing run on the
  robot or in Gazebo.
- **Scope:** In scope. Answer given: no reflash; Phase 1 runs on the current `pico_pilot`
  firmware by design (decision 1). The firmware PID stays the later fix.
- **Decisions:** none.
- **Open / next:** user measures `forward_m` (lens ahead of the wheel-axle midpoint) and runs
  the route follower on the robot; record the tape end error.

### 2026-11-18: Camera forward offset measured; bags move out to 0.87 m

- **Phase:** 1, Route following
- **Branch / commit:** `walle-phase-1` @ after `596ba16`, uncommitted
- **Asked for:** the user measured the lens "about 12 cm" ahead of the wheel-axle midpoint.
- **Done:** `robot.camera.forward_m` 0.07 → 0.12 (`walle-app/config/park.yaml`). The auto bin
  offset grows from 0.82 to 0.87 m: bins at (±0.615 / 1.615), 3.5 cm farther out in x and y
  than the 0.580 positions. Camera geometry is now all measured (height 0.103 m, forward
  0.12 m); the fields of view are still the OV3660's nominal values, not measured.
- **Verified:** unit tests pass (host). Computed framing from the corner: bag top at 21.3° of
  the 23.5° half-height, sides at 12.8° of 30°. Gazebo not re-run: its camera stays at
  0.07 m forward, so the bags are only farther from it and the earlier PASS still bounds it.
- **Scope:** In scope.
- **Decisions:** none.
- **Open / next:** user moves the bags to the printed positions and runs the follower on
  the robot.

### 2026-11-18: Phase 1 robot run on carpet; bin centering not good enough

- **Phase:** 1, Route following
- **Branch / commit:** `walle-phase-1` @ after `596ba16`, uncommitted
- **Asked for:** the user ran the follower on the robot (run `20261118_195156`) and reported:
  navigation "very good", ended "almost exactly back where it started"; only bin 2 was
  centered in its photo, bins 1, 3, 4 were not; the robot turned "in increments of maybe
  10-15 degrees to find the bin"; no bin top was visible. Asked whether it can turn
  "smoothly and slowly" to center the bin.
- **Before this run:** the first attempt (`20261118_194215`) aborted with no
  `/odometry/filtered`: the Pico had not connected to the micro-ROS Agent. The Pico
  firmware was rebuilt from the current source (the Nov 5 `build/pico_pilot.uf2` still had
  the HC-SR04 on GP6/GP7, from before `0061cc8`) and flashed; it connected and published at
  50 Hz. The HC-SR04 does not answer (`echo -2`); not needed for Phase 1. Two more runs
  aborted before this one (`194621`, `195103`; rows in `robot_runs.csv`).
- **Done:** read the run log and photos; ticked the two robot boxes in `ROADMAP.md`.
- **Verified (from the run's files):** 1 lap, 4 bins, 15 s waits, status done, 4.04 m
  driven. End error: tape 0 cm / 0° (user), EKF 0.6 cm / +0.5°, wheels alone 22.8 cm /
  +22.7° (carpet slip; the BNO055 heading is what holds the route). Measured gyro drift at
  rest 0.0005 °/s over 70 s of still periods (the sim profiles assumed 0.05–0.1 °/s).
  Learned coasts: sides 0.225 s, turns 0.157 s. Estimated heading error when facing each bin:
  bin 2 −9.8°, bin 3 −0.7°, bin 4 −10.5°, bin 1 −14.2°. The bin 4 trace shows why: first
  turn 3.4° short of 135°, then a correction to +12.5°, a second to −10.5°. The open-loop
  firmware cannot turn by less than about 15°, so the correction moves (added in sim for
  `correct_tol_deg` 3°) made the centering worse on the robot.
- **Correction to the run's record:** at the end-of-run prompt every bin was answered
  "centered", so `robot_runs.csv` says `bins_faced 4/4`; the user's report says 1/4.
  `robot_runs.csv` is left as written (append-only); this entry is the correction.
- **Not explained yet:** in the photos the bin's place does not match the estimated heading
  error alone (bin 3: −0.7° but the bin is at the right edge). Bin placement, bin size, or the
  camera's aim on the robot could each explain it; the photos show large gray plastic bins,
  not the 30 × 18 × 36 cm paper bags in `park.yaml`.
- **Scope:** In scope (the Phase 1 robot run).
- **Decisions:** none yet. Proposed to the user: firmware closed-loop wheel speed
  (module 7's PI, plan escalation step) for slow, smooth turns; until then, no turn
  corrections smaller than one correction move can make.
- **Open / next:** user: bin dimensions and placement used in this run. Decide the
  smooth-turning approach. Bin tops: the Phase 4 servo.

### 2026-11-18: New bin size; closed-loop wheel speed in the Pico firmware

- **Phase:** 1, Route following
- **Branch / commit:** `walle-phase-1` @ after `596ba16`, uncommitted
- **Asked for:** the bins are black plastic, rectangular, "29.2cm x 27.7cm x 21.1 cm" in the
  order "height, width, depth"; update the bin size, and "add closed-loop wheel speed to the
  firmware (not in a new folder) for slow, smooth turns". The user keeps the study's
  original firmware in another folder to revert to.
- **Done:**
  - Bins 0.277 × 0.211 × 0.292 m (wide face × depth × height); auto offset 0.71 m, bins at
    (±0.502 / 1.502). The park test now reads the bin size from the config
    (`walle-app/config/park.yaml`, `walle-app/tests/test_park.py`).
  - `walle-pilot/firmware/pico_pilot/drive_control.c`: open-loop PWM map replaced with
    per-wheel closed-loop speed control from module 7's `wheel_control.c`: feed-forward
    (0.432 m/s at full PWM) + PI (KP 0.1, KI 5.0, fixed: no free micro-ROS subscription for a
    gains topic), 50 Hz, speed filter 0.5, 0.5 m/s² setpoint ramp, anti-windup, MIN_DUTY
    0.25 dead-zone skip. A zero command stops at once; the 500 ms watchdog is kept. Same
    API and topics, so `pico_pilot.c` and the micro-ROS setup are unchanged. Built:
    `build/pico_pilot.uf2` (2026-11-18 20:10), no warnings. **Not flashed.**
  - Sim `PidFirmware` rewritten to mirror the new C line for line; `pid_firmware` and
    `carpet_pid` profiles use its constants (`carpet_pid` also stalls below duty 0.30)
    (`walle-sim/nodes/firmware_models.py`, `walle-sim/config/sim_profiles.yaml`).
  - Comments and docs: `route.yaml`, `route_follower.py`, `walle-app/README.md`,
    `walle-sim/README.md`, `CLAUDE.md` (hardware row, sim profile note), `ROADMAP.md`.
- **Verified:**
  - Unit tests pass (host). Offline model check (firmware model + motor plant): commanding
    0.3 / 0.6 rad/s turns gives 2.58 / 2.77 rad/s with the old firmware (matches the robot)
    and 0.30 / 0.60 with the new one (0.30 / 0.58 on `carpet_pid`).
  - Gazebo `route-check`, new bins, 1 lap: `pid_firmware` (3 s waits, RTF 0.69) PASS, end
    1.5 cm, bins 0.4–2.0° off center; `carpet_pid` (15 s waits, RTF 1.00) PASS, end 0.5 cm,
    bins 0.5–3.8° off center, **0 correction moves** (every bin a single turn). All bins
    wholly in frame.
  - **Not verified:** nothing on the robot. The PI gains are module 7's, tuned on the course
    robot, not WALL-E on carpet; MIN_DUTY 0.25 may stall under load (the PI then pushes the
    effort up, as modeled by `carpet_pid`'s stall). The RQ1–RQ3 sim worlds and harness were
    not re-run with the new model (`pilot_firmware`, their default, is unchanged).
- **Scope:** Drift, agreed: `walle-pilot/` was read-only for this work; the user asked for
  the firmware change in `pico_pilot/` itself. The RQ1–RQ3 results were measured on the
  open-loop firmware; re-running the study on this firmware would not reproduce them.
- **Decisions:** ROADMAP decision log, 2026-11-18 (firmware PI in place; new bin size).
- **Open / next:** user: move the bins to (±0.502 / 1.502), flash the new UF2, run the
  follower; check turns are smooth and the bins centered in the photos. If the robot creeps
  or stalls on carpet, the gains/MIN_DUTY need tuning on the robot.

### 2026-11-18: Robot run 2 (closed-loop firmware): jerky; motor stiction found and retuned in sim

- **Phase:** 1, Route following
- **Branch / commit:** `walle-phase-1` @ after `596ba16`, uncommitted
- **Asked for:** the user ran the closed-loop firmware (run `20261119_015242`): "moved slower
  in general, and the movement was jerky", "almost fidget when it turned"; 2 of 4 bins
  centered. The test surface is now corrugated plastic sheets, not carpet.
- **Found (from the run's files):**
  - Turning itself was accurate: estimated heading error at every bin within 0.5° (before:
    up to 14°), no correction moves. End: tape 0 cm, EKF 0.2 cm. Wheels alone 38.8 cm /
    34.6° off (the wheels slip in turns on the plastic). Gyro drift 0.0008 °/s.
  - Jerk: a turn commanded at 17–34°/s started at ~100°/s and then swung 14–54°/s, and
    slow approaches switched between stopped and moving: stiction. At 0.3–0.6 rad/s each
    wheel was asked for 2–4 cm/s; a stuck wheel needs more power to start than to keep
    turning, so it sticks, the integral winds up, it lurches, and repeats.
  - Off-center bins 1 and 3 are not from turning (heading was right). In the photos the bins
    look far closer than the planned 0.48 m to the lens (bin 2's bottom at the frame's
    bottom edge suggests ~0.25–0.3 m), and off the diagonal: bin placement, to check.
- **Done:**
  - Sim motor plant: `breakaway_duty` (stiction; default 0 = off, other profiles unchanged).
    Fitted offline: breakaway 0.30–0.35 with stall 0.25 reproduces the robot's turn profile.
    New `plastic_pid` profile (`walle-sim/nodes/firmware_models.py`,
    `walle-sim/config/sim_profiles.yaml`).
  - Gains searched offline against that model (KP, KI, MIN_DUTY, filter; breakaway 0.30 and
    0.35): KP 0.1 → 1.0, SPEED_FILTER 0.5 → 0.3 (KI 5.0, MIN_DUTY 0.25 kept). This roughly
    halves the start lurch and ripple, but no setting makes wheel speeds under ~0.1 m/s
    smooth. Firmware rebuilt (`drive_control.c`, `build/pico_pilot.uf2` 21:00). **Not
    flashed.**
  - Follower: turns 1.0–1.5 rad/s (was 0.3–0.6), slowest drive 0.08 m/s (was 0.05), turn
    coast starts at 0.15 s (`walle-app/config/route.yaml`). Docs: READMEs, `CLAUDE.md`,
    `ROADMAP.md` known conditions.
- **Verified:** Gazebo `route-check`, `plastic_pid`, 3 s waits: PASS, end 1.1 cm, bins
  +9.7° (first bin, coast 0.06 s then) / +2.1 / +2.0 / −0.1°, 3 corrections; true turn rate
  ramps up, holds 77–93°/s, slows smoothly (robot before: 14–54°/s swings). Not verified:
  the 0.15 s turn-coast start (set from that run's learned 0.154 s, not re-run); the
  `pid_firmware`/`carpet_pid` profiles with the new gains; anything on the robot. The
  stiction model is a fit to one run's 10 Hz log, not a motor measurement.
- **Scope:** In scope (making the agreed firmware work on the robot).
- **Decisions:** none new; tuning only.
- **Open / next:** user: check bin placement (center 0.71 m from the corner mark along the
  diagonal; front face center 0.60 m), flash the new UF2, run again; report smoothness.

### 2026-11-18: Robot run 3 (retuned firmware, 1.0–1.5 rad/s turns): smooth

- **Phase:** 1, Route following
- **Branch / commit:** `walle-phase-1` @ after `596ba16`, uncommitted
- **Asked for:** the user ran the retuned firmware (run `20261119_021229`, corrugated
  plastic): "a lot smoother"; bin 2 "perfectly centered", bins 1 and 2 centered at the
  prompt, bins 3 and 4 off, probably bin placement. Asked whether the robot identifies the
  bin or turns to a fixed angle.
- **Verified (run files):** done; end tape 0 cm / 0°, EKF 0.4 cm / +1.7°; wheels alone
  25.1 cm / 23.5°. Estimated heading error when facing each bin: bin 2 +1.0°, bin 3 +0.1°,
  bin 4 +5.5°, bin 1 −1.6°; 8 correction moves in the lap; learned coasts sides 0.16 s,
  turns 0.117 s. Bin 3 off-center with a 0.1° heading error = placement; bin 4 is partly the
  robot (5.5°).
- **Answer given:** the robot turns to a fixed heading computed from `park.yaml` (corner to
  the bin's configured center) on the EKF; it does not look for the bin. Centering on the
  bin itself needs detection (Phase 3) and is a candidate for Phase 4's bin check.
- **Scope:** In scope. No files changed except this entry.
- **Open / next:** bin 4's 5.5° and the 8 corrections: not investigated yet. Commit the
  Phase 1 work; decide whether vision-based centering goes into Phase 3/4.

### 2026-11-18: Phase 1 closed; bin centering moved to Phase 3; Phase 2 handoff

- **Phase:** 1 → 2
- **Branch / commit:** `walle-phase-1` @ after `596ba16`, uncommitted
- **Asked for:** the user proposed that the YOLO phase also "identify the bins, and then look
  for trash", and center the camera that way; said smooth motion and route following are
  established and Phase 1 is done; asked for a session summary and a Phase 2 start prompt
  structured like the existing ones in `docs/handoff/`.
- **Done:**
  - `ROADMAP.md`: Phase 1 ✅ (all three boxes were ticked earlier); Phase 3 work and done
    criteria gain bin detection and centering at each bin stop; decision log rows for the
    move and the turn-speed floor; Phase 0's real-park note updated (black bins entered,
    square on plastic sheets, taping not recorded).
  - `docs/handoff/session-2026-11-16-summary.md` (this session) and
    `docs/handoff/phase-2-prompt.md` (start prompt for Phase 2).
- **Verified:** documentation only. Facts checked against the run files, `git status`, and
  the code (`route_follower.py` outputs, `park.py` layout fields, `walle-pilot/vision/`
  outputs). No database code exists in the repository yet.
- **Scope:** Drift, agreed: bin centering (part of Phase 1's "turn to face the bin so the
  whole bin is centered") moves to Phase 3, by the user's decision.
- **Decisions:** ROADMAP decision log, 2026-11-18 (Phase 1 done; bin centering → Phase 3).
- **Open / next:** commit the post-`596ba16` work (firmware, retune, bins, docs) and decide
  whether `walle-app/results/` (robot run logs) is committed. Then Phase 2 in a new session.

### 2026-11-18: Phase 2, memory store (SQLite + recall), built and checked in sim

- **Phase:** 2, Memory store
- **Branch / commit:** `walle-phase-2` (from `origin/main` @ `b93eec2`, Phase 1 merged in
  PR #7) @ uncommitted. The user's uncommitted move of the old handoff docs into
  `docs/handoff/phase0-to-phase1/` and `phase1-to-phase-2/` came along, untouched.
- **Asked for:** start Phase 2 from `docs/handoff/phase1-to-phase-2/phase-2-prompt.md`; plan
  mode, settle the open decisions. User answers: SQLite **plus a vector store**; separate
  files per source; DB git-ignored and rebuilt from the run logs; bins 2 and 4 are recycling.
  Two corrections to the plan: the robot "will never be able to look into the bin itself"
  (opaque bins, low camera): only the top and around the bin are visible; and a bin that is
  not overflowing is checked for litter, and with none the robot "should move on".
- **Done:**
  - Memory store: `walle-app/memory/` (`store.py` write API + queries, `schema.sql`,
    `embed.py` Ollama + fake embedder, `route_records.py`, `import_runs.py`, `README.md`
    with the schema), `walle-app/config/memory.yaml`; DB files in `walle-app/data/memory/`,
    git-ignored (`walle-app/.gitignore`).
  - Bin types: `bins.types` in `walle-app/config/park.yaml`, `type` on each layout bin and
    a scenario `type:` override in `walle-app/park.py` (random streams unchanged).
  - Route follower: `t_utc` on each `arrived` / `facing_bin` event; at the end it records the
    run + one `photo_only` bin check per bin stop (`--no-memory` skips; a store error only
    prints a warning) (`walle-app/route/route_follower.py`).
  - 14 new tests (`walle-app/tests/test_memory.py`).
  - Docs: `walle-app/README.md`, `CLAUDE.md`, `ROADMAP.md` (Phase 2 ticks, decisions),
    plan in `docs/plans/phase-2-plan.md`.
- **Verified:**
  - Unit tests, 34 total, pass on the host (Python 3.13, ResourceWarnings as errors) and in
    both `walle-pilot` and `walle-sim` (3.10).
  - Container restart: wrote a fake run to a scratch store from `walle-pilot`,
    `docker compose up -d --force-recreate pilot`, read it back identical. The scratch file
    was deleted after.
  - Sim: `make sim-route-check DWELL=3` on `plastic_pid` / `default`, twice:
    - The first run (on a Gazebo world launched before bin types existed) passed and was
      recorded, but exposed a bug I had introduced (`NameError` after recording: `main`'s
      return line was moved into the new function). Fixed.
    - The second run (after relaunching the world) passed (end 0.1 cm) with no error, and
      recorded the run plus 4 bin checks with types in `sim.sqlite`. Embeddings were
      computed from inside the container through `host.docker.internal`.
  - `import_runs.py --rebuild` put the 3 committed robot runs (12 bin checks) into
    `robot.sqlite`. These runs predate bin types, so their `bin_type` is NULL.
  - Real `nomic-embed-text` recall on the host returns sensible matches (run end errors,
    bin 3 photos).
  - Not verified:
    - a robot run writing to the store by itself (user);
    - detections, obstacles and assessed bin checks from real producers (Phases 3–5); only
      the fake runs in the tests exercise them;
    - concurrency was tested with two connections in one process only, not two processes;
    - the fake embedder can't tell "bin 4" from "bin 2", so real recall quality was only
      spot-checked.
- **Scope:** In scope, with small additions:
  - `t_utc` on route events (so bin checks carry their own time);
  - a `meta` table (a file knows its source);
  - `bin_type` copied onto detections;
  - `obstacle_history()` and `counts()` reads.
  - Field naming: the plan's `where` field is called `place` (`where` is an SQL keyword),
    and the value is `on_bin`, not `in_bin`. The sim world left running 3 days was relaunched
    with `default` / `plastic_pid`; it is still running headless.
- **Decisions:** ROADMAP decision log, 2026-11-18, Phase 2 (five rows).
- **Open / next:**
  - User: run the follower on the robot once; it should print `Memory: run … recorded in
    …/robot.sqlite`.
  - Then commit and close Phase 2. Phase 3 writes detections through `add_detection` (bins:
    `place='on_bin'`/`around_bin` with `bin_id`) and must also leave them in the run folder so
    `import_runs.py` can rebuild them (the importer reads only route summaries today).

### 2026-11-18: Correction: the restart check cut the Pico off

- **Phase:** 2, Memory store
- **Branch / commit:** `walle-phase-2` @ uncommitted
- **What went wrong:** the container-restart check (previous entry) recreated only
  `walle-pilot` (`docker compose up -d --force-recreate pilot`). The micro-ROS Agent shares
  pilot's network (`network_mode: "service:pilot"`), so it was left in the old, dead network
  namespace. The user's next robot run aborted with "No /odometry/filtered received", because
  `/pico/wheel_ticks` had 0 publishers.
- **Fixed:** `docker compose up -d --force-recreate --no-deps micro-ros-agent`. The Pico
  reconnected without a reset; `/pico/wheel_ticks` has 1 publisher and `/odometry/filtered`
  publishes.
- **Lesson:** restart the robot stack as a whole (`make down && make up`, then reset the Pico),
  never the pilot container alone.
- **Open / next:** the aborted run left `walle-app/results/route/20261119_043359/` (status
  aborted, source unknown, not in the memory store); delete it or keep it.

### 2026-11-18: Phase 2 robot check: the run recorded itself; Phase 2 done

- **Phase:** 2, Memory store
- **Branch / commit:** `walle-phase-2` @ uncommitted
- **Asked for:** the user ran the follower on the robot (run `20261119_043639`, default
  scenario) and saw the `Memory: run … recorded` message.
- **Verified (by query of `robot.sqlite` and the run's files):** the run is recorded: done,
  4.006 m, EKF end 0.3 cm, tape 0 cm. It has 4 `photo_only` bin checks with types (1, 3
  trash; 2, 4 recycling) and event times; all 20 sentences are embedded. The aborted
  `20261119_043359` folder is gone (deleted by the user).
- **Found, not investigated (not Phase 2):**
  - No photo at bins 2 and 4: the camera request failed (the reason was printed in the
    user's terminal, not saved). The records have `photo` NULL there.
  - Estimated heading error at the bins: bin 2 +5.3°, bin 3 −1.3°, bin 4 −9.4°,
    bin 1 +0.2°. The previous run had at most 5.5°.
  - The user marked no bin as centered.
  - The follower's motion code is unchanged in Phase 2 (only event times and the memory
    call were added).
- **Scope:** In scope.
- **Decisions:** none.
- **Open / next:** commit Phase 2. Look into the camera misses and the larger bin heading
  errors before Phase 3 depends on the bin photos.

### 2026-11-19: Investigation: missed bin photos and bin heading errors (run 20261119_043639)

- **Phase:** 2 (robot check follow-up; the causes belong to Phase 1 and the hardware)
- **Branch / commit:** `walle-phase-2` @ uncommitted
- **Asked for:** find out why bins 2 and 4 got no photo and were 5.3° / −9.4° off.
- **Found:**
  - **Photos:** the camera answered `timed out` at bin 2 and `Connection refused` at bin 4.
    Bins 3 and 1 worked, and all 12 photos in the earlier runs had worked. The user believes
    power was the cause and recharged both supplies: the 9 V motor battery and the 5 V supply
    for the Pico, HC-SR04 and ESP32-CAM. That fits: a sagging 5 V supply browns out the
    ESP32-CAM when its Wi-Fi transmits. A low 5 V may also be why the HC-SR04 does not answer
    (unchecked). Not confirmed: after the recharge the camera did not answer from the host
    (ping and HTTP), probably because it was not powered on again yet.
  - **Heading:** the trajectory shows the main bin turn ending 4–6° short. Each correction
    move then went 8–13° (asked for 4–6°), and the follower kept the last result even when
    worse: bin 4 was −3.7° after the main turn and −9.4° after two corrections. A correction
    starts from standstill at 1.0 rad/s (≈0.09 m/s per wheel, below the ~0.1 m/s stiction
    edge), so it lurches. The same pattern was in the first robot run (−9.8, −10.5, −14.2°).
    A weak motor battery may have made it worse; it is not the root cause.
- **Done:** nothing changed in code.
- **Verified:** read the run's `trajectory.csv` and `summary.json`, the follower's turn code,
  and the ESP32-CAM sketch (one attempt per photo, 5 s timeout, no retry; the error is not
  saved in the run log); compared with the three earlier robot runs.
- **Scope:** In scope (investigation only).
- **Decisions:** none.
- **Open / next:** rerun on fresh batteries to confirm the photos; then decide on the
  proposed fixes: photo retries within the bin wait, and a small-turn correction (a timed
  pulse; never end worse than it started), sim first. Phase 3 bin centering needs small turns.

### 2026-11-19: Pulse corrections for small turns, and photo retries

- **Phase:** 1 follow-up (found in the Phase 2 robot check). Needed by Phase 3's bin
  centering.
- **Branch / commit:** `walle-phase-2` @ uncommitted
- **Asked for:** "fix both": the bin heading errors (corrections overshooting) and the missed
  photos. The user found that power caused the camera misses and recharged both supplies.
- **Done:**
  - Timed-pulse corrections (`walle-app/route/route_follower.py`, `route_geometry.py`):
    - `plan_pulse` / `learn_pulse_gain` hold the math.
    - `turn_to` corrects misses up to 20° with a pulse: one turn command, then a stop after
      the planned time. Bigger misses still get a normal turn.
    - A pulse is skipped when even the shortest one would overshoot by more than the miss.
    - The gain is learned from each pulse.
    - Up to 3 corrections per turn (was 2).
    - `pulses` and `pulse_gain_end_dps` go into `summary.json` and the printed summary.
  - Photo retries: up to 3 tries per bin stop, 3 s apart, all within the bin wait. Each event
    records `photo_tries` and `photo_error`. Settings in `walle-app/config/route.yaml`.
  - `route-check --coast-time` / `make sim-route-check COAST=…` (`walle-sim/harness/sim_trials.py`,
    `Makefile`) to force turn misses.
  - 3 unit tests (`walle-app/tests/test_route.py`). Docs: `walle-app/README.md`,
    `CLAUDE.md`, ROADMAP decision log.
- **Verified:**
  - Unit tests: 37 pass on the host.
  - Gazebo `plastic_pid` pulse probe (scratch script, deleted): a pulse turns about
    110°/s × time, whatever rate is commanded (0.1 s → 7°, 0.2 s → 19–22°, 0.3 s → 29–31°).
    A single 50 ms command sometimes doesn't break free.
  - Photo retries in the sim container:
    - refused connection: 3 tries in 9 s;
    - timeout: 2 tries in 13 s;
    - the sim camera: 1 try.
    All fit within a 15 s wait.
  - `make sim-route-check DWELL=3`, all PASS:
    - normal coast: 0.4 cm; 2 pulses, 3.3° → 1.2° and 3.4° → 1.1°.
    - COAST=0.35: 1.1 cm; pulses −14.6 → 0.0°, +7.9 → −1.1°, −5.8 → +1.2°; worst bin 2.7°.
    - COAST=0.0: 1.4 cm; +13.9 → −3.5 → +1.0°, −7.9 → −0.9°; worst bin 2.5°.
    - Baseline, the old slow-turn corrections at COAST=0.35 (`pulse_max_deg` 0 for one run,
      restored after): PASS 1.2 cm, but 10 corrections and bin 3 left at −5.2°.
  - **Not verified on the robot.** The sim's stiction doesn't reproduce the robot's
    8–13° lurches, so the improvement in sim is modest. On the robot the learned gain has to
    adapt from 110°/s to whatever the lurch gives. The camera fix depends on the recharged
    5 V supply; the retries only cover short dropouts.
- **Scope:** Drift, agreed: Phase 1 motion code changed during Phase 2, at the user's
  request ("fix both").
- **Decisions:** ROADMAP decision log 2026-11-19 (pulse corrections, photo retries).
- **Open / next:** user: a robot run on the charged supplies. Check the printed "correction
  pulses" line, the bin heading errors, and that all 4 photos arrive. Then commit.

### 2026-11-19: Robot run with pulse corrections and charged supplies: good

- **Phase:** 1 follow-up / 2
- **Branch / commit:** `walle-phase-2` @ uncommitted
- **Asked for:** the user ran the follower on the robot (run `20261120_014029`): "looks fine";
  bins not centered "because of my placement". Asked if Phase 3 can start.
- **Verified (run files and `robot.sqlite`):**
  - Run: done; EKF end 0.3 cm, tape 0 cm.
  - 3 pulses, each landing within 0.6°: +5.5 → −0.6°, +3.7 → +0.2°, +3.2 → −0.3°. Learned
    gain 110 → 118 °/s; the robot's pulses behave like the sim's, with no lurch.
  - Bin heading errors +2.5 / −0.2 / +0.3 / +1.0° (worst before: +5.3 / −9.4°).
  - 4 of 4 photos on the first try.
  - The run recorded itself in `robot.sqlite` (5 runs, 20 bin checks).
  - Bins 3 and 1 (marked not centered) were faced within 1°. Their photos show each bin at
    the right edge and much nearer than the planned 0.71 m (it fills the frame's height, no
    top visible): **placement**, as the user said.
- **Scope:** In scope.
- **Decisions:** none.
- **Open / next:**
  - Commit Phase 2 and merge it to `main` (Phase 3 branches from `origin/main`).
  - Before Phase 3's real-photo set: place the bins at 0.71 m on the diagonal
    (`python3 walle-app/park.py` prints the positions). Phase 3 centers on the detected bin,
    but the top of a bin placed too close is out of view.

### 2026-11-19: Phase 0 closed; Phase 2 committed

- **Phase:** 0 and 2
- **Branch / commit:** `walle-phase-2` @ see git log (PR to `main`)
- **Asked for:** tick Phase 0's "square taped out" box ("I have it taped out"); commit
  Phase 2 and open a PR.
- **Done:** `ROADMAP.md`: Phase 0 ✅ (last box ticked, track marking recorded as tape).
  Committed the Phase 2 work, the pulse and photo fixes, and the robot run logs
  `20261119_043639` and `20261120_014029` (the record that `import_runs.py` rebuilds
  `robot.sqlite` from).
- **Verified:** the user's statement (not seen).
- **Scope:** In scope. Not committed: the user's own move of the handoff docs into
  `docs/handoff/phase0-to-phase1/` and `phase1-to-phase-2/`, plus `.DS_Store` and `.vscode/`.
- **Decisions:** none.
- **Open / next:** merge the PR, then Phase 3 in plan mode from `origin/main`.
