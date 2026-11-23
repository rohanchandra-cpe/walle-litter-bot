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

### 2026-11-19: Phase 2 merged; Phase 3 handoff

- **Phase:** 2 → 3
- **Branch / commit:** `walle-phase-2` (merged to `main` in PR #8, `2a42319`, by the user)
  @ uncommitted
- **Asked for:** a summary of this session and a start prompt for the Phase 3 agent,
  structured like the documents in `docs/handoff/phase0-to-phase1/` and
  `docs/handoff/phase1-to-phase-2/`.
- **Done:** `docs/handoff/phase2-to-phase-3/session-2026-11-18-summary.md` and
  `docs/handoff/phase2-to-phase-3/phase-3-prompt.md`.
- **Verified:** documentation only. Facts were checked against the code (`camera_node.py`,
  `vision_node.py`, `sim_camera_server.py`, `park_world.py`, the ESP32-CAM sketch, the route
  follower), the run files, and the containers' Python packages (2026-11-19: torch 2.14 CPU,
  numpy 2.2.6, no ultralytics; the sim's apt OpenCV fails with numpy 2).
- **Scope:** In scope.
- **Decisions:** none.
- **Open / next:** Phase 3 in a new session from `origin/main`. Neither this entry nor the
  handoff folder is committed yet.

### 2026-11-19: Phase 3 started: plan, detector stack, photo-set tools

- **Phase:** 3 (Object detection and trash/recycling classification)
- **Branch / commit:** `walle-phase-3` (from `origin/main` `2a42319`, no upstream set) @ uncommitted
- **Asked for:** start Phase 3 from `docs/handoff/phase2-to-phase-3/phase-3-prompt.md`: plan
  mode, settle the open decisions, then build. The user asked that the photo set be taken with
  the ESP32-CAM (answer: yes, never a phone, for the scored set) and that Claude guide it.
- **Done:**
  - Plan approved; copied to `docs/plans/phase-3-plan.md`. Decisions in the ROADMAP log;
    Phase 3 marked 🟨 with the agreed targets.
  - Both Dockerfiles: libGL, pip 26.3 (needed to build CLIP from git), setuptools < 80
    (colcon), torchvision 0.29.2 from the CPU index (the PyPI build breaks with torch
    2.14.1+cpu: "torchvision::nms does not exist"), ultralytics 8.4.215, opencv-python
    5.1.0.95, CLIP pinned to a commit; `yolo settings sync=False`; `~/.cache/clip` → 
    `walle-app/models/clip`. Images rebuilt; containers recreated with `make down && make up`
    and `make sim-down && make sim-up`.
  - `walle-app/config/detect.yaml`: model, thresholds, bin prompts, the user's item labels,
    sim-only prompts, centering and place settings, eval targets.
  - `walle-app/detect/`: `detector.py` (YOLO-World, prompt → label → category),
    `geometry.py` (bearing, cut-off bins, on_bin/around_bin, ground-plane obj_xy),
    `probe_prompts.py`, `capture_set.py`, `eval_photos.py`.
  - `walle-app/data/photos/phase3/shots.yaml`: 42-shot list (6 bin-only incl. measured
    sideways shifts of 10/20 deg, 24 at bin stops, 10 on the route/carpet, 2 empty).
  - `make detect-eval`; `walle-app/models/` git-ignored; `CLAUDE.md` layout/commands.
  - `walle-app/tests/test_detect.py` (14 tests incl. one with the model).
- **Verified:**
  - Both containers import numpy 2.2.6, cv2 5.1.0 (pip build shadows the sim's apt one),
    torch 2.14.1+cpu, torchvision 0.29.2+cpu, ultralytics 8.4.215, clip; rclpy still works.
  - Tests: 60 pass in both containers; on the host 59 + 1 skipped (no ultralytics).
  - Latency on the 18 robot bin photos, container CPU, 640 px: S median 0.32 s, M 0.77 s.
  - Bin prompts probed alone on those 18 photos (bins too close, tops cut off): "black bin"
    best, 11/18 at conf 0.10 with S (M 1/18); its best box lay on the bin in 16/18. Bin
    `min_conf` set to 0.03 (a bin is known to be there at a stop). **Not settled**: needs the
    new photos with the bins at 0.71 m.
  - `capture_set.py` dry-run against a fake local camera (take, retake, skip, resume);
    `eval_photos.py` dry-run on 4 old photos with made-up labels (runs, scores, writes report).
  - **Not done:** no labeled photos yet, so no accuracy measured; no follower, memory, node
    or sim work yet (plan steps 5–7).
- **Scope:** In scope. `setuptools` < 80 and the pip upgrade are image changes beyond the
  listed packages, needed to install them.
- **Decisions:** ROADMAP decision log 2026-11-19 (Phase 3, five rows).
- **Open / next:** user: take the 42 shots with `capture_set.py` (robot on 5 V only, bins at
  0.71 m); then `make detect-eval`, tune prompts, settle the bin prompt. Meanwhile: follower
  centering, `detections.json`, importer, `detect_node.py`, sim prompts and check.

### 2026-11-19: Phase 3: follower centering + detections, memory import, sim stand-in; lens FOV doubt

- **Phase:** 3
- **Branch / commit:** `walle-phase-3` @ uncommitted
- **Asked for:** while the user takes the photo set, build the robot side (plan step 5–6).
- **Done:**
  - `route_follower.py`: `center_bin()` after `face_bin` (photo, detect, pulse/turn toward
    the detected bin, ≤ 3 corrections, ≤ 25° from the configured heading, no bin = keep the
    heading and record why; detector errors never abort the run); detection on the dwell
    photo; `<run>/detections.json`; `centering` and `detected` in `summary.json`;
    `--no-detect`, `--truth-layout`; `route.yaml` `detect`, `center_bins`.
  - `route_geometry.centering_target()`; `memory/route_records.py` reads `detections.json`
    (so `import_runs.py --rebuild` keeps detections).
  - `detect/detect_node.py` (`/detect/ask` → `/detect/result`).
  - **Scope change (user agreed, replaces the plan's "sim prompts"):** YOLO-World sees
    nothing in the Gazebo park (flat-colored shapes; no prompt > 0.05 on 4 bin photos, not
    even a bin filling the frame). `detect/sim_truth.py` is a ground-truth stand-in: it
    projects the true bins/items through the sim camera (0.10 m high, 0.07 m ahead,
    60 × 46.8°, from the URDF), hides items behind a bin, and boxes items resting on top only
    above the near rim. `detect.yaml` `sim:` now holds the sim camera, not prompts.
  - `sim_trials.py route-check --misplace-deg` (`make sim-route-check MISPLACE=12`): the
    follower is told each bin is swung round its corner; checks centering (truth), detected
    bin, places, ground positions (≤ 15 cm), and detections in `sim.sqlite`. The world's
    layout is saved as `layout_truth.json` (the follower's own `layout.json` overwrote it in
    the first run).
  - `capture_set.py`: a shot counts as done only if its photo exists (the user deleted
    `shot_01.jpg`; it would have been skipped).
- **Verified:**
  - Tests: 66 (host: 65 + 1 skipped); containers all pass (before the last test was added:
    65 in both).
  - `detect_node.py` live in walle-sim (camera_node on the sim camera): answered in 0.31 s.
  - Gazebo `plastic_pid`, mixed, OBSTACLE=none, DWELL=3, MISPLACE=12: run 1 FAIL (bin 2's
    overhanging bottles called around_bin: stand-in occlusion bug, fixed); run
    `20261120_031658` **PASS**, 15/15 checks: bins centered to +0.6 / −0.5 / −0.0 / −1.0°
    true bearing in 2–3 looks (9–13° from the configured heading); places all right; ground
    positions ≤ 5.2 cm off; 15 detections in `sim.sqlite` = `detections.json`; end 1.5 cm.
  - `import_runs.py --source sim --rebuild` into a temp file: both sim runs' 30 detections
    rebuilt from the run folders.
  - **Not verified:** nothing on the robot; YOLO-World accuracy (no labeled photos yet).
- **Lens:** the user's `shot_01.jpg` (bin at 0.71 m) has the bin's top cut off. With the
  nominal 60 × 47° FOV the top should be ~25 px inside the frame. The bin's bottom row and
  widths fit ~53 × 41° (typical ~66° diagonal ESP32-CAM lens). With that, `park.py`'s
  framing rule would put the bins at ~0.79–0.81 m. Not changed yet: waiting for two
  tape-measure photos to measure the FOV (and any camera tilt).
- **Scope:** Drift, agreed: sim uses a ground-truth stand-in instead of sim prompts. Pending
  (needs the user's agreement): moving the bins farther out if the measured FOV confirms.
- **Decisions:** sim stand-in (ROADMAP log, below).
- **Open / next:** FOV photos → `park.yaml` camera FOV, bin offset, shot list shifts → user
  takes the photo set → `make detect-eval`. ROADMAP: "in sim, detections written to the
  memory store" met (stand-in); sim centering met, robot centering not yet.

### 2026-11-20: Phase 3: photo set done; YOLO-World fails on items; VLM detector (gemma4) tried

- **Phase:** 3
- **Branch / commit:** `walle-phase-3` @ uncommitted
- **Asked for:** the user finished the 42-shot set (29 at night, 13 in daylight; corrected
  labels themselves) and chose "Option 2" (a vision model names items). "Accuracy first,
  slower is fine."
- **Done:**
  - `capture_set.py --light`; labels carry `light` (1–29 night, 30–42 day).
  - Plan amendment (`docs/plans/phase-3-plan.md`) and ROADMAP decision row: two-step item
    detection, tune on shots 1–29, report 30–42 as unseen.
  - `detector.py`: `VLMDetector` (bins by YOLO-World; items FOUND by gemma4 on the whole
    photo, free-form name + box, then each crop NAMED by gemma4: describe, then a label or
    not_trash) and `make_detector()`; `detect.yaml` `method: vlm`, `vlm:` (model, url,
    per-label descriptions). Follower and `eval_photos.py` (`--method`, `--vlm-model`,
    `--ids`) use it.
  - `detect.yaml` `eval.latency_median_s` 1.0 → 15.0 (user: accuracy first). **To confirm
    with the user.**
  - `park.yaml` (previous turn, user unconfirmed): `offset_m: 0.88`, camera `forward_m:
    0.125`; shot list shifts for 0.755 m; `eval.bin_distance_m: 0.755`. 2 park tests fail
    on it (they assume the auto offset).
- **Verified (all on shots 1–29, night; 30–42 not scored yet, kept unseen):**
  - YOLO-World S, item prompts: 0/36 items at conf 0.25; bins 29/29.
  - YOLO-World S, generic prompts as a finder: drawn on all 29; misses foil, most bags,
    floor towels, the paper bag. Not usable as the find step.
  - gemma4 one call with a label list: recall 0.28, precision 0.26 ("foil" everywhere).
  - gemma4 find (asked for "trash") + name crops: recall 0.28, precision 0.48, category
    1.0 (clean jugs/cans/tubs never found).
  - gemma4 find ("loose, movable objects") + name crops: **recall 0.47, precision 0.46,
    category 0.89, bins 1.0, median 5.0 s (p90 11.3 s)**. Spinach bags 0/9 (called
    plastic_bag; both trash); with the two bag labels merged: recall 0.65, precision 0.64.
    Items on top of a bin are missed most.
  - qwen3.5:4b boxes were misplaced (other coordinate order; not pursued).
  - Tests: 66 run; 2 park failures (above); the rest pass on host and walle-pilot.
  - Reports: `walle-app/results/detect/20261120_19*/`.
- **Scope:** Drift, agreed in direction: "Option 2" became gemma4 for both finding and
  naming (YOLO-World could not find the items). Latency target relaxed (to confirm).
- **Decisions:** ROADMAP 2026-11-20 (two-step item detection).
- **Open / next:** user: labels for shots 31/32 (the bin from shot 30, with the milk jug,
  shows in the background); retake shot 7; confirm the bin distance (2 park tests), the
  latency target, and which way to push accuracy (see the session's options).

### 2026-11-20: Bag labels merged, qwen3.5:4b as the item model, unseen day shots scored

- **Phase:** 3
- **Branch / commit:** `walle-phase-3` @ uncommitted
- **Asked for:** the user retook shots 7, 31 and 32, agreed to merge the grocery and spinach
  bag labels, then "continue with what you were doing". Asked whether a remote (cloud)
  vision model could be used; after discussing it, shelved it as a possible alternative for
  the paper.
- **Done:**
  - `spinach_bag` merged into `plastic_bag`: `detect.yaml` (items, prompts, describe),
    `labels.yaml`, `shots.yaml`, `capture_set.py` header, `tests/test_detect.py`.
  - `VLMDetector.box_order()`: `detect.yaml` `vlm.box_order` is now `{default: yxyx, qwen:
    xyxy}` (matched on the model name's start); qwen reports x first whatever it's asked.
  - `detect.yaml` `vlm.model`: gemma4 → `qwen3.5:4b`.
  - ROADMAP decision rows: bag merge; remote models shelved.
- **Verified:**
  - Retaken shots 7, 31, 32 look right (31 has a small yellowish object far back by the TV
    stand, left unlabeled).
  - Night shots 1–29 (tuning set), merged labels: gemma4 recall 0.581, precision 0.581,
    category 0.895, median 5.2 s (`results/detect/20261120_230527`); qwen3.5:4b recall
    0.698, precision 0.714, category 0.892, median 5.1 s (`20261120_231243`). Both FAIL the
    0.8 targets. Items on top of bins and the paper bag are still the weak spots.
  - Day shots 30–42, unseen, scored once with qwen3.5:4b: recall 0.90, precision 0.90,
    category 1.0, bin 1/1, median 8.1 s: PASS (`20261120_231443`). Caveat: these shots are
    mostly items on the floor with no bin (only shot 30 has a bin); they are easier than
    the night bin shots. All 42 together: about 48/63 items found (0.76), still under 0.8.
  - Tests on the host: 66 run, only the 2 known `test_park` failures (bin distance).
- **Scope:** In scope. A remote-model detector was discussed, not built (user shelved it).
- **Decisions:** ROADMAP 2026-11-20 rows (bag merge; remote models shelved). Model switch to
  qwen3.5:4b recorded here and in `detect.yaml`.
- **Open / next:** the accuracy checkbox is NOT ticked: on-bin night shots are at ~0.70.
  Optional ~10 daylight repeats of night bin setups would show whether light or the bin
  setting causes the gap. Still to confirm with the user: latency target 15 s, bin distance
  (2 park tests). Then the robot centering run.

### 2026-11-20: Latency target and bin distance confirmed; sim re-checked at 0.88 m

- **Phase:** 3
- **Branch / commit:** `walle-phase-3` @ uncommitted
- **Asked for:** the user confirmed the 15 s latency target ("not expecting sub 1 second
  performance") and that the bins were placed 0.88 m out on the diagonal, though for many
  photos the robot was moved back or aside to get the bin's top and sides in view. The
  daylight repeats wait until tomorrow.
- **Done:**
  - `tests/test_park.py`: the two auto-offset tests now set `offset_m: auto` themselves
    (park.yaml fixes 0.88 m).
  - ROADMAP: latency box ticked; decision row (15 s target, 0.88 m bins); centering note.
  - `detect.yaml` latency comment marked confirmed.
  - Found the running Gazebo world was built before the 0.88 m change (bins at 0.71 m), so
    the 2026-11-19 sim pass was at 0.71 m. Relaunched the park world (bins now 0.88 m).
- **Verified:**
  - Tests: all pass on the host, in walle-pilot and in walle-sim (1 model test skipped on
    the host).
  - `make sim-route-check RUN=phase3 DWELL=3 MISPLACE=12` at 0.88 m: PASS 15/15. Bins
    centered within 2.5° true bearing (2–3 looks), bins 0.34–0.41 wide / 0.60–0.64 tall
    of the half-frame, places right, ground error ≤ 6.3 cm, 16 detections in `sim.sqlite`
    (`walle-sim/data/phase3/route/20261120_232353`).
- **Scope:** In scope.
- **Decisions:** ROADMAP 2026-11-20 (latency ≤ 15 s; bins at 0.88 m).
- **Open / next:** the photo set's bin distances vary (robot moved for framing), so the
  bin-bearing numbers on shots 3–6 are a rough check only. Tomorrow: ~10 daylight repeats
  of night on-bin setups (shots 43-52 added to shots.yaml: `capture_set.py --light day`), then
  decide on the accuracy box. Then the robot centering run.
  Nothing committed.

### 2026-11-20: Zoomed rim search (tried, left off); detect_node uses make_detector

- **Phase:** 3
- **Branch / commit:** `walle-phase-3` @ uncommitted
- **Asked for:** "go ahead with the zoomed search above the bin fix and the detection-node
  fix" while the user sets up the robot centering run.
- **Done:**
  - `detector.py`: `VLMDetector.find()` (FIND on the whole photo or a zoomed region),
    `rim_region()` (window over the biggest bin's top), `merge_boxes()`/`overlap()`;
    `VLM_ZOOM_PROMPT`. Switch `detect.yaml` `vlm.zoom_bin_top` (enabled: false).
    `eval_photos.py --zoom` turns it on for one run. Result shape unchanged (adds
    `zoom_region` when used).
  - `detect_node.py`: `make_detector` (config method, default vlm), `-p method:=yolo`;
    its log handles `conf` None.
  - `tests/test_detect.py`: `TestZoom` (window, no-bin case, merge).
- **Verified:**
  - Tests: host 69 OK (1 skipped), walle-pilot OK. detect_node not run under ROS.
  - Shots 1–29 with `--zoom`: recall 0.674, precision 0.674, category 0.892, median 8.5 s,
    p90 14.2 s (`results/detect/20261120_234001`); without: 0.698 / 0.714, 5.1 s. Per on-bin
    item: the zoom finds something on the rim more often (shot 8 jug) but the naming step
    gets on-bin items wrong at night (jug → plastic_bag/plastic_bottle, can →
    yogurt_container, grocery bag → paper_towel). Shots 14 and 28: nothing found on the rim
    either way. So at night the limit is naming on-bin items, not finding them.
  - The robot run had not started when this scoring ran (as far as known), so Ollama was
    not shared with it.
- **Scope:** In scope. The zoom stays off (no gain, slower).
- **Decisions:** none (zoom off pending the day repeats).
- **Open / next:** score shots 43–52 with and without `--zoom`; the user's robot centering
  run (bins 2 and 3 slid ~10 cm; items: bin 1 none, bin 2 jug on top, bin 3 towel + foil on
  the floor, bin 4 grocery bag on top + can on the floor).

### 2026-11-21: First robot run with centering and detection (20261121_000650), reviewed

- **Phase:** 3
- **Branch / commit:** `walle-phase-3` @ uncommitted
- **Asked for:** the user ran the follower on the robot (bins 0.88 m out, bins 2 and 3 slid
  ~10 cm; items: bin 1 none, bin 2 milk jug on top, bin 3 towel + foil on the floor, bin 4
  grocery bag on top + can on the floor) and asked "What do you think of the run?". User's
  notes: EKF good, ended ~1 cm from the start; bin 2 bad (jug cut off and against a white
  wall, clutter behind, centering wandered); bins 1, 3, 4 good.
- **Done:** review only (no code changed). Re-ran YOLO-World on the run's photos to see the
  bin boxes; rebuilt the robot store into a scratch file to compare.
- **Verified (from summary.json, detections.json, photos, robot.sqlite):**
  - Route: done; EKF end error 1.1 cm, heading -1.1 deg; user measured 1.0 cm, 0 deg.
  - Centering: bin 4 centered in 1 look (-0.4 deg); bin 1 in 2 looks (+2.75 deg). Bin 3
    NOT within 3 deg by the code (-15.1 -> +13.4 -> -10.3 -> +4.2, "still +4.2 off after 3
    corrections"), though the final photo looks fine. Bin 2 failed (ended +24.6 off; final
    photo shows the bin's edge at the left).
  - Cause 1, bin 2 (detection): look 0 did not detect the (cut-off) bin and took a tiny
    0.24-conf box (16 x 26 px, under the side table) as the bin; look 2 had the real bin
    (conf 0.21, ~-3 deg) but `pick_bin` chose a 0.03-conf, 46 px box at -17.6 deg because it
    was nearer the EXPECTED bearing (-20 deg from the config, as the robot had already turned
    toward the real bin). Then it turned -26.7 deg away.
  - Cause 2, bins 1 and 3 (turning): every correction overshoots. Per look pair, the bin
    moves in the camera ~1.6x the heading change the EKF reports (bin 3: 28.5 / 17.7, 23.7 /
    14.7, 14.5 / 9.0; bin 1: 6.1 / 3.7; bin 2's real-bin pair: 7.9 / 4.9). Camera parallax
    (camera 12.5 cm ahead of the axle, bin ~0.88 m) explains ~1.17x; the rest (~1.37x) means
    short pulses turn more than the EKF records 0.3 s after them. The bin's pixel width
    (253 px at ~0.65 m) matches the 60 deg FOV, so the FOV is not the cause. In sim the ratio
    was 1.1-1.5 too (2-3 looks).
  - Detections (4 recorded, 3 right): bin 1 nothing (right); bin 3 paper_towel + foil,
    around_bin, obj_xy plausible in front of the bin (right); bin 4 can, around_bin (right)
    but the grocery bag on top was missed: the bin's top is at the frame's top edge, the bag
    is cut off. Bin 2: milk jug missed (above the frame) and a false "can" (the orange object
    on the side table, at the frame's top).
  - Memory: 4 rows in `robot.sqlite`; `import_runs.py --rebuild` into a scratch store gives
    the same 4 rows and 6 runs.
- **Scope:** In scope.
- **Decisions:** none.
- **Open / next:** centering on the robot is NOT done. Proposed fixes: drop bin boxes far
  smaller than the expected bin (not cut at an edge) before picking; learn the camera's
  bearing change per degree of turn during centering and divide by it. Items on top at 0.88 m
  are cut off: the servo (Phase 4) is the real answer; record as a limit.

### 2026-11-21: Centering fixes from the robot run; backing-up fallback noted for Phase 4

- **Phase:** 3 (and a Phase 4 note)
- **Branch / commit:** `walle-phase-3` @ uncommitted
- **Asked for:** "fix the bugs and check in the sim", and whether the robot could back up to
  see the bin's top if the servo can't be made to work.
- **Done:**
  - `detect/geometry.py`: `plausible_bin()` (box at least `min_size_frac` 0.5 of the bin's
    expected px width and height; a box cut at a side edge judged on height, at the top on
    width); `pick_bin()` now takes the most confident plausible box, ties to the nearest
    expected bearing (was: nearest expected bearing). `analyze_bin_stop()` takes
    `expected_bin_px` as (width, height) (a lone number = width only, no size check).
  - `route/route_geometry.py`: `parallax_factor()`, `learn_turn_scale()`.
  - `route/route_follower.py`: `centering_cfg()` (sim override), expected bin height passed
    in; each correction asks for bearing / (parallax x turn_scale); turn_scale learned from
    each pair of looks, kept across bins; recorded per look and as
    `centering_turn_scale_end` in summary.json.
  - `config/detect.yaml` centering: `min_size_frac`, `turn_scale` 1.35 (robot run),
    `turn_scale_limits`, `turn_scale_gain`; `sim.centering.turn_scale` 1.0.
  - Tests: pick (confidence first, ties), the robot run's junk and real boxes, side-cut bin,
    parallax, turn-scale learning (from bin 3's numbers), wrong-way and tiny turns ignored.
  - ROADMAP: Phase 4 open decision gains the backing-up fallback (geometry: ~0.35–0.45 m
    back to see 25 cm above the rim); Phase 3 centering note updated.
- **Verified:**
  - Tests: host 76 OK (1 skipped); walle-pilot OK.
  - Replay with YOLO-World on saved photos: photo set bins found 30/30 (shift bearings
    unchanged); run 20261121_000650 bin_2 look 2 now picks the real bin (-3.1 deg, conf 0.21);
    looks 0 and 3 (only junk boxes) now give "no bin". Bins 1, 3, 4 picks unchanged.
  - `make sim-route-check RUN=phase3 DWELL=3 MISPLACE=12` (bins 0.88 m): PASS 15/15, bins
    within 1.4 deg true bearing in 2–3 looks; learned turn_scale stays ~1.03 in sim (EKF and
    camera agree there; the sim's first overshoot is the pulse itself, which the EKF sees).
    `walle-sim/data/phase3/route/20261121_034633`.
  - NOT verified on the robot.
- **Scope:** In scope; the backing-up idea is recorded, not built.
- **Decisions:** none new (Phase 4 option recorded as open).
- **Open / next:** robot re-run of centering. Known gap: if the first look finds no bin
  (e.g. bin cut off at the frame edge and not detected), centering stops and keeps the
  configured heading; a small search turn (look ±15 deg) would cover it.

### 2026-11-21: Robot re-run after the centering fixes (20261121_035910), reviewed

- **Phase:** 3
- **Branch / commit:** `walle-phase-3` @ uncommitted
- **Asked for:** review of the re-run. User: "Still some issues with Bin 2, but it got closer
  to the center of the bin. It still struggled to identify the milk jug."
- **Done:** review only. Re-ran YOLO-World on the bin 2 photos (default and extra prompts).
- **Verified (summary.json, detections.json, photos, robot.sqlite):**
  - Route: done; EKF end 0.6 cm, -2.2 deg; user measured 0 cm, 0 deg.
  - Bins 1 and 4: centered on the first look (+2.2, -1.8 deg). Bin 3: centered in 3 looks
    (-18.3 -> +15.1 -> -2.1). The learned turn scale came out 1.345, then 1.362 (start 1.35),
    so the robot run's ~1.37 is confirmed. Bin 3's swing was the first PULSE itself: asked
    -11.6, the EKF recorded -21.4 (pulse gain had been pulled down to 72.5 deg/s by two tiny
    route-correction pulses), not the camera/EKF mismatch.
  - Bin 2: "no bin detected" on the first look, so centering stopped and kept the configured
    heading (as designed after the fix; no junk chased this time). The bin is plainly in the
    photo but cut off at the left edge; YOLO-World scores it 0.016-0.024 (below 0.03), with
    a cushion at 0.011. From the visible edge, bin 2 sits ~20 deg left of the camera axis,
    i.e. ~17-18 deg (~27 cm along its face) from its configured heading; run 1 agrees (real
    bin at heading ~-27 vs configured -45).
  - Milk jug on bin 2: entirely above the frame (bin rim at y ~25 px of 480). It cannot be
    detected at 0.88 m with a level camera; not a detector failure. Bin 2's false "can" is a
    metal bowl under the side table.
  - Detections: bin 1 none (right), bin 3 paper_towel + foil around_bin (right), bin 4 can
    (right; bag on top cut off), bin 2 false can. Same 4 rows in robot.sqlite.
- **Scope:** In scope.
- **Decisions:** none.
- **Open / next:** search turn when the first look finds no bin (look toward the side a
  cut-off dark box suggests, or +-15 deg); ask the user whether bin 2 was placed ~27 cm off
  on purpose. Items on top at 0.88 m need the servo or the backing-up fallback (Phase 4).

### 2026-11-21: Search turn when bin centering finds no bin

- **Phase:** 3
- **Branch / commit:** `walle-phase-3` @ uncommitted
- **Asked for:** "Yes, add it" (a search turn when the first look finds no bin; run
  20261121_035910's bin 2 was cut off at the frame edge and not detected).
- **Done:**
  - `detect/geometry.py` `search_order()`: offsets +-`search_deg` from the configured bin
    heading; the side where a bin-tall box (any confidence) is cut off at the frame edge goes
    first, else left first.
  - `route/route_follower.py` `center_bin()`: no bin before any was found -> turn to each
    search offset and look again (search looks don't count as corrections); none found ->
    "no bin detected (searched ...)" and the record keeps `searched_deg`. `--sim-hide-cut-bins`.
  - `detect/sim_truth.py`: `hide_cut_bins` (miss a bin whose box runs off the frame's side,
    as YOLO-World did on the robot). Harness `--hide-cut-bins`, `make sim-route-check HIDE_CUT=1`.
  - `config/detect.yaml` centering `search_deg: 15.0`.
  - Tests: search order (the run's real boxes: bin cut left at 0.016 beats a cushion cut
    right at 0.011), sim hiding.
- **Verified:**
  - Tests: host 78 OK (1 skipped), walle-pilot OK.
  - `make sim-route-check RUN=phase3 DWELL=3 MISPLACE=20 HIDE_CUT=1`: PASS 15/15. Bins 1, 2,
    3 not seen on the first look; found by the search (bin 2 on +15, bins 1 and 3 on -15
    after +15); all 4 within 1.2 deg true bearing, 3-4 looks
    (`walle-sim/data/phase3/route/20261121_041117`).
  - `make sim-route-check RUN=phase3 DWELL=3 MISPLACE=12`: PASS, no searches, 2 looks each
    (`20261121_041248`).
  - NOT run on the robot. Correction found while checking: YOLO-World was asked only for
    boxes >= 0.03, so the 0.017 cut-off box never reached the hint. Added `bin.hint_conf:
    0.01` (`detector.py`: fainter bin boxes come back in low_conf only, never used as the
    bin). Replay of the run's bin 2 look 0 now: no bin chosen, hint box (0.017, cut at the
    left) -> search +15 (left) first, toward the real bin. walle-pilot tests OK after it.
- **Scope:** In scope.
- **Decisions:** none.
- **Open / next:** robot run to check bin 2 (and the others) with the search.

### 2026-11-21: Robot run with the search turn (20261121_041923): all bins centered

- **Phase:** 3
- **Branch / commit:** `walle-phase-3` @ uncommitted
- **Asked for:** review of the run. User: all 4 bins faced; ended ~2 cm from the start;
  "a slight jerk when it got to Bin 4 where it jerked left, but then it corrected itself."
- **Done:** review only. ROADMAP: robot centering box ticked.
- **Verified (summary.json, detections.json, trajectory.csv, photos, robot.sqlite):**
  - Route: done; EKF end 0.07 cm, +0.1 deg; user measured 2 cm, 0 deg.
  - Centering, all 4 within 3 deg: bin 2 none on look 0, search +15 (left, as the hint
    said), then +0.74 deg; bin 3 -18.1 -> +0.5 (one correction); bin 4 +1.2 (first look);
    bin 1 +4.3 -> +0.7. Learned turn scale 1.348 -> 1.399 (start 1.35).
  - Detections: bin 1 none (right); bin 3 paper_towel + foil around_bin (right); bin 4 can
    around_bin (right; the bag on top cut off by the frame); bin 2: the bottom of the milk
    jug shows as a white strip at the frame's top edge, reported as yogurt_container,
    on_bin, recycling (place and category right, label wrong). No false bowl this time.
    4 rows in robot.sqlite.
  - The jerk at bin 4: after the turn to face bin 4 (wheels: stopped at ~141 deg by t=115.2 s),
    the FUSED heading kept turning at about the last turn rate (128 -> 58 deg by 116.15 s,
    later 139 -> -153 deg over 118.7-121.8 s) while the wheel odometry did not move. That is
    the EKF predicting on with no new sensor data (most likely a Wi-Fi/UDP gap in
    /pico/imu/data and /pico/wheel_ticks; not confirmed, the log has no message times). The
    follower acted on the ghost heading: a real pulse of ~+73 deg by the wheels (the pulse
    record: asked -12.9, "moved" 46.1), then corrections back. It recovered when data
    returned (re-anchor at 122.1 s); the bin was then centered at +1.2 deg. The pulse gain
    was knocked to 264 deg/s by that outlier and relearned over 4 pulses.
- **Scope:** In scope (review). The dropout handling is Phase 1 route code; not changed.
- **Decisions:** none.
- **Open / next:** a stale-sensor guard in the follower (stop and wait instead of turning
  when /pico/wheel_ticks or /pico/imu/data are older than ~0.3 s; skip pulse-gain learning
  on outliers). Phase 3 left: the accuracy box (daylight shots 43-52 tomorrow).

### 2026-11-21: Sensor guard in the route follower (stop on stale Pico data, repair the estimate)

- **Phase:** 3 (fixes Phase 1 route code, found in the Phase 3 robot run)
- **Branch / commit:** `walle-phase-3` @ uncommitted
- **Asked for:** "Go ahead and add the guard" (after run 20261121_041923: the EKF heading
  spun on with no sensor data and the follower made a real 73 deg pulse at it). Also asked
  whether Phase 4 can start before the Phase 3 daylight photos (answered in the session).
- **Done:**
  - `route/route_follower.py` (RouteFollower; `walle-pilot` Mover untouched): stamps
    `/pico/wheel_ticks` and `/pico/imu/data` arrivals; `send()` turns any motion command into
    a stop while a stream is older than `max_age_s` and waits until both are fresh for
    `resume_after_s` (gives up after `give_up_s`: the run ends); waits don't count against
    side/turn timeouts. `repair_estimate()`: after a gap, re-anchors the fused estimate at
    the pose from just before the gap moved by the wheels' motion across it
    (`rg.bridge_gap`; encoder counts are cumulative). Pulses with a gap or an outlier move
    (`rg.pulse_outlier`, > 2.5x or < 1/2.5 of plan) don't teach the pulse gain; centering
    doesn't learn turn_scale across a gap; still periods with a gap don't feed the gyro bias.
    summary.json: `sensor_gaps`, `sensor_guard_waits`, `estimate_repairs`; pulses flagged
    `sensor_gap` / `outlier`.
  - `config/route.yaml` `sensor_guard` (enabled, max_age_s 0.4, resume_after_s 0.6,
    give_up_s 20, pulse_outlier 2.5).
  - walle-sim: `sim_pico_bridge.py` `/sim/blackout` (Float32 s: drop all ticks and IMU for
    that long; commands still pass); harness `--blackout EVERY FOR` + a check that the guard
    acted; `make sim-route-check BLACKOUT="20 1.5"`.
  - Tests: pulse outlier (the run's 46 deg pulse), gaps_in, bridge_gap.
- **Verified:**
  - Tests: host OK, walle-pilot OK.
  - Sim, first try (guard without the repair), `BLACKOUT="20 1.5" MISPLACE=12`: FAIL, ended
    8.1 cm off: the robot stopped, but the EKF carried the estimate on ~8 cm at the last
    speed, so it thought it reached corner 4. Added the repair.
  - Sim with the repair, same settings: PASS 16/16; 6 gaps, 2 waits while driving (1.7 s
    each), repairs removed 11.2 and 8.8 cm of ghost motion; end 3.9 cm; all bins centered
    within 1.4 deg (`walle-sim/data/phase3/route/20261121_043326`).
  - Sim without blackouts: PASS, no gaps seen, end 0.3 cm (`20261121_043459`).
  - NOT run on the robot. The robot gaps are inferred (no message times were logged); the
    next run's `sensor_gaps` will show them.
- **Scope:** Drift (agreed): Phase 1 route robustness fixed during Phase 3.
- **Decisions:** none.
- **Open / next:** robot run to see the guard on real Wi-Fi. Phase 3: daylight shots.

### 2026-11-21: Phase 3 committed; handoff to Phase 4

- **Phase:** 3 → 4
- **Branch / commit:** `walle-phase-3` @ `3dea433` (Phase 3, committed at the user's
  request; the user opened a draft PR). The handoff docs and this entry are uncommitted.
- **Asked for:** "commit Phase 3 with a sensible message", then a session summary and a
  Phase 4 start prompt, structured like the earlier ones in `docs/handoff/`.
- **Done:**
  - Commit `3dea433`: 133 files. Left out: the user's handoff-folder moves,
    `walle-app/data/photos/_props.jpg`, three copies of the CLIP weights (`weights/`,
    `walle-app/weights/`, `walle-pilot/weights/`), `.DS_Store`, `.vscode/`.
  - `docs/handoff/phase3-to-phase-4/session-2026-11-19-summary.md` and `phase-4-prompt.md`.
    The prompt includes the Phase 3 leftovers (score daylight shots 43–52), the firmware
    facts (micro-ROS limit, pins in use, watchdog), and the "fill level" vs overflow-only
    wording to reconcile.
  - ROADMAP status row for Phase 3: "all done but item accuracy".
- **Verified:** the prompt's facts were checked against the repository (firmware pins,
  publishers and subscriptions, `add_bin_check` signature, sim camera joint). Found that
  `walle-pilot/config/depth_calibration.json`, which `depth_estimator.py` reads, is not in
  the checkout; the prompt says so. Nothing run.
- **Scope:** In scope. The user moves on to Phase 4 with Phase 3's accuracy box open
  (agreed).
- **Decisions:** none.
- **Open / next:** Phase 4 in a new session; daylight shots on 2026-11-22.

### 2026-11-21: Daylight repeats (shots 43–52) scored

- **Phase:** 3 (leftover)
- **Branch / commit:** `walle-phase-3` @ uncommitted (photos and labels added by the user's
  `capture_set.py --light day` run)
- **Asked for:** the user took shots 43–52 (daylight repeats of the night on-bin setups 7–14,
  25, 28).
- **Done:** checked the 10 photos against their labels; scored them with and without
  `--zoom`; compared with the same setups at night. No code changed.
- **Verified:**
  - Photos: items on top are fully in frame (unlike the robot's 0.88 m bin stops).
    Shots 45 and 47 look alike: a white/red grocery bag AND a tan crumpled bag (looks like
    the Chipotle paper bag of shot 50), but each is labeled with one `plastic_bag` only.
    Asked the user; not changed.
  - Shots 43–52, no zoom: recall 0.733, precision 0.786, category 1.0, median 7.8 s
    (`results/detect/20261121_153411`). With zoom: recall 0.933, precision 0.778, category
    1.0, median 9.4 s (`20261121_153549`).
  - Same setups at night (7–14, 25, 28): no zoom recall 0.75 / precision 0.86; zoom 0.56 /
    0.64 (from `20261120_231243`, `20261120_234001`).
  - Reading: without the zoom, daylight changes nothing; with it, daylight lifts recall from
    0.56 to 0.93. The zoomed crop helps when the light is good and hurts in lamp light.
    Small sets (15–16 items each): treat as indications.
- **Scope:** In scope.
- **Decisions:** none yet (accuracy box and zoom setting to decide with the user).
- **Open / next:** user: confirm shots 45/47 contents; decide the accuracy box (and whether
  to switch the zoom on, e.g. only in good light).

### 2026-11-21: Zoomed rim search on, but only in good light; all 52 photos re-scored

- **Phase:** 3 (accuracy box)
- **Branch / commit:** `walle-phase-3` @ uncommitted
- **Asked for:** the user fixed shots 45 and 47 (two plastic bags each, brown and gray) and
  chose "Try the zoom in good light".
- **Done:**
  - `detect/detector.py`: `warm_cast(img)` (mean red − mean blue), `zoom_light_ok()`; the
    zoom runs only when the cast is ≤ `max_warm_cast`; results carry `warm_cast` and
    `zoom_skipped`.
  - `config/detect.yaml` `vlm.zoom_bin_top`: `enabled: true`, `max_warm_cast: 9.0`.
  - Tests: the gate and the config.
- **Verified:**
  - Light measures on the 52 photos: mean brightness overlaps (night 85–162, day 146–162:
    the camera's auto-exposure), edges overlap; the warm cast separates cleanly (lamps
    11.2–21.5, daylight 2.4–7.0). It separates the user's warm lamps from daylight, not dark
    from bright in general.
  - All 52, gated zoom (`results/detect/20261121_154929`): recall 0.787, precision 0.768,
    category 0.945, bins 1.0, median 8.1 s: FAIL by a hair on recall and precision.
    Daylight 35 items: recall 0.91, precision 0.84 (on-bin setups 0.88 / 0.83, floor shots
    0.90 / 0.82). Lamp light 45 items: 0.69 / 0.70 (zoom skipped on all of them).
  - Tests: host OK, walle-pilot OK.
- **Scope:** In scope.
- **Decisions:** zoom on in good light (user's choice); ROADMAP row added.
- **Open / next:** the accuracy box: met in daylight, not under the lamps, overall just
  under. The user decides how to record it.

### 2026-11-21: Phase 3 item-accuracy target carried into Phase 4

- **Phase:** 3 → 4
- **Branch / commit:** `walle-phase-3` @ uncommitted
- **Asked for:** "Leave open for Phase 4" (how to record the accuracy box, after the 52-photo
  score: daylight 0.91 / 0.84, lamp light 0.69 / 0.70, overall 0.787 / 0.768).
- **Done:** ROADMAP: Phase 3's accuracy box stays unticked, with the numbers and a pointer;
  Phase 4 gains a "done when" box for item accuracy with the tilted camera (same targets);
  status row and decision log updated. Phase 4 start prompt updated (leftovers done, the
  carried target added).
- **Verified:** docs only.
- **Scope:** Drift, agreed by the user: one Phase 3 criterion moves to Phase 4, because the
  servo changes the view that fails (items on top of a bin, at 0.88 m, under lamp light).
- **Decisions:** ROADMAP 2026-11-21 (accuracy target carried to Phase 4).
- **Open / next:** Phase 4 in a new session.

### 2026-11-21: Phase 4 plan and build: camera tilt servo, bin check (sim)

- **Phase:** 4 (Servo and bin monitoring)
- **Branch / commit:** `walle-phase-4` (moved to `walle-phase-3` @ `177ec2b` once the other
  session's daylight-shots commit landed; Phase 3's PR is still unmerged) @ uncommitted
- **Asked for:** start Phase 4 from the handoff prompt. In plan mode the user approved the
  plan (`docs/plans/phase-4-plan.md`) and said "Proceed with the plan and ask me if you
  need anything"; the servo isn't wired or measured yet.
- **Done:**
  - Firmware (`walle-pilot/firmware/pico_pilot/`):
    - `servo.c`/`servo.h`: FS5106B on GP22, 50 Hz, clamp 600–2400 µs, boot 1500 µs,
      slew 1000 µs/s. The PWM divider comes from the system clock (150 MHz on the Pico 2 W).
    - `pico_pilot.c`: a 4th value of `/pico/imu/std` sets the pulse; a 5th value of
      `/pico/wheel_ticks` reads it back.
    - `CMakeLists.txt`.
    - Watchdog and drive code untouched.
  - Host servo (`walle-app/servo/`):
    - `tilt.py` (`Calibration`, `Tilt`); `servo_test.py` (sweep / `--deg` / `--us` /
      `--stay`; `--check` against the Gazebo joint);
    - `config/servo.yaml`: calibration values are **placeholders**; `up_deg` 15.
  - Geometry: `Camera` has a tilt (and an optional lens-off-axis offset), plus
    `ray` / `project` / `height_at` / `expected_box_px`. At tilt 0 the numbers are unchanged.
    `sim_truth.py` projects with the tilt.
  - Bin check:
    - `detect/bin_check.py` (`judge_bin`) and `detect.yaml` `bin_check`;
    - `route_follower.py`: after the dwell photo, tilt up → photo → analyze → judge →
      level;
    - `send()` refuses motion while the camera is tilted (it levels it first);
    - writes `bin_checks.json`;
    - `--no-tilt`, `route.yaml` `bin_check`;
    - no servo answering → `level_only`.
  - Memory:
    - `route_records.py` stores judged checks (`tilt` / `level_only`);
    - with two looks, `on_bin` comes from the up photo and `around_bin` from the level
      photo;
    - `schema.sql` comment and `memory/README.md` updated.
  - Sim:
    - `camera_tilt_joint` + JointPositionController (`walle_sim.urdf.xacro`); one line of
      the Module 6 model changed (`camera_joint`'s parent);
    - `/sim/camera_tilt` bridge;
    - the simulated Pico models the servo;
    - `sim_trials.py bin-check` / `make sim-bin-check`;
    - scenario `bincheck.yaml`.
  - Photo-set tools:
    - `capture_set.py`: `look` / `tilt_deg` / `light` per shot, and prints the servo
      command;
    - `eval_photos.py`: tilt per photo, scores each look on the places the bin check uses,
      `by_light` / `by_look`, `--by-light` checks;
    - **draft** shot list `data/photos/phase4/shots.yaml` (48 shots).
  - Tests: `walle-app/tests/test_bin_check.py` (16 tests).
  - Docs: ROADMAP Phase 4 rewritten (overflow, not fill; boxes; 3 decision-log rows);
    `CLAUDE.md`; the plan copied with "as built" notes.
- **Verified:**
  - Unit tests 99/99 OK (1 skip on the host) on the host (3.13), `walle-pilot` and
    `walle-sim` (3.10).
  - Firmware **compiles** cleanly (scratch build with the user's Pico SDK 2.3.1 toolchain).
    **Not flashed, not run on the Pico.**
  - Sim (Gazebo, `plastic_pid`, headless, RTF 1.0):
    - `servo_test.py --sweep --sim --check` PASS: the joint is within 0.01° at −10, 0, 15
      and 30°;
    - rendered photos confirm the camera looks up;
    - `make sim-bin-check DWELL=3` PASS on `mixed` (levels 1 and 3, litter, clean) and on
      `bincheck` (level 2, litter, clean): every outcome, level and visible-litter count is
      right, the counts reach `sim.sqlite`, and it never drove tilted;
    - `make sim-route-check MISPLACE=12` PASS (Phase 3 centering unchanged; bin checks
      right with misplaced bins).
  - Eval tool smoke test on 2 Phase 3 photos in the container (runs; scores not meaningful
    on 2 photos).
- **Scope:**
  - In scope, with the plan's agreed changes: distance not measured, "full" dropped.
  - Drift (mine, reported to the user): the height-above-rim rule for the overflow level was
    planned but is **off**. Measured over 40 sim mounds per level, it overlaps between
    levels; the count doesn't.
  - Found: sim litter is mostly placed behind the bins (`park.py` `LITTER_SPREAD_DEG` 70°
    from outward), where no camera on the route can see it. `sim-bin-check` judges against
    the visible litter. In `bincheck`, bin 2's only litter item is hidden, so the truth is
    "litter" and the judge says "clean". Not changed: it's park.py (Phases 0–3), so I'm
    asking the user.
- **Decisions:** 3 rows in the ROADMAP decision log (command path and servo hardware; the bin
  check; dropped / kept items and targets).
- **Open / next:**
  - User: wire the servo (5–6 V buck/UBEC, GP22, common ground), build and flash, then run
    `servo_test.py --sweep` and calibrate `servo.yaml`: `level_us`, `us_per_deg`,
    `min_deg` / `max_deg`, `lens_from_pivot_m`, and the camera's level position in
    `park.yaml` if the mount moved it.
  - Agree the tilted shot list, then take it in day and night light and score it
    `--by-light` (also `--zoom` under the lamps).
  - Robot bin-check runs against the user's per-bin ground truth.
  - Decide whether sim litter should be placed where the robot can see it.
  - Sensor guard on the robot is still unrun (from Phase 3).

### 2026-11-21: Sim litter in view; camera position on the tilt mount

- **Phase:** 4 (Servo and bin monitoring)
- **Branch / commit:** `walle-phase-4` @ uncommitted
- **Asked for:** "Make it visible" (sim litter, which was mostly hidden behind the bins).
  Also: the camera is the same distance forward as before, but now 12.5 cm up from the
  ground and 1.1 cm left of the robot's center.
- **Done:**
  - `park.py` `place_on_floor`:
    - spills and litter go on the corner side of the bin (within 80° of the inward
      direction), with their center within `LITTER_VIEW_DEG` 24° of the camera's axis at
      the bin stop;
    - `LITTER_GAP_M` widened to 0.03–0.25 m. A full level-3 bin plus 6 litter failed on
      98 of 200 seeds before, with the same rate in either direction; now 0 of 200;
    - falls back to the outward side only when the corner side has no room (a cylinder bin
      0.34 m from its corner, test_park).
  - Camera position:
    - `park.yaml` `robot.camera`: `height_m` 0.125 (was 0.103), `forward_m` 0.125
      unchanged, new `left_m` 0.011;
    - `Camera` takes `left_m` (`project`, `ground_point`, expected box);
    - `sim_truth` uses it too; the sim camera keeps 0 (`detect.yaml` `sim.camera`);
    - the follower's `expected_bin` bearing and distance are measured from the lens.
  - `data/photos/phase3/camera.yaml` (10.3 cm, 0 left): `eval_photos.py` reads a set's own
    camera file, so the Phase 3 photos keep the camera they were taken with.
  - `analyze_bin_stop`: an item box cut off at the frame's bottom gets no `obj_xy`. In the
    up look, litter close in front of the bin was placed 18–20 cm off; it doesn't affect
    the counts, which come from the level photo.
  - `sim_trials.py` bin-check now wants the full litter count (and reports how many were
    seen). Tests:
    - lens-offset round trip;
    - park camera values;
    - every floor item in view over 25 seeds × 4 bins, by the ground-truth camera;
    - no floor position when cut at the bottom;
    - the sim judge test uses the full litter count.
  - ROADMAP decision row; plan "as built" note.
- **Verified:**
  - Unit tests 103 OK (1 skip on the host); OK in `walle-pilot` and `walle-sim` too.
  - Sim, headless, `plastic_pid`:
    - `make sim-bin-check DWELL=3` PASS on `mixed`: litter 3/3 and 4/4 seen and counted;
    - PASS on `bincheck`: bin 2's single litter item is now seen and judged "litter";
      bin 4 6/6;
    - `make sim-route-check MISPLACE=12` PASS.
  - The first `mixed` run after moving the litter FAILED on up-look floor positions
    (18–20 cm); fixed as above, then PASS.
  - Robot: nothing run (servo not yet wired).
- **Scope:** In scope (the user's answer to the open question, and their measurement).
- **Decisions:** litter placement and camera position (ROADMAP decision log).
- **Open / next:**
  - Still unknown: `servo.yaml` calibration (placeholders) and `lens_from_pivot_m`; the
    user gave the level position only.
  - The 12.5 cm height shifts what the level and up looks frame a little (the rim now sits
    lower in the level photo); `up_deg` 15 stays until the robot photos show otherwise.
  - Rest as in the previous entry.

### 2026-11-21: Robot servo flashed and calibrated (first pass)

- **Phase:** 4 (Servo and bin monitoring)
- **Branch / commit:** `walle-phase-4` @ uncommitted
- **Asked for:** help calibrating the camera servo after wiring it.
- **Done:**
  - Built the firmware in the user's `build/` folder (clean; the user flashed it).
  - `servo.yaml` (robot) set from the user's measurements: level 1640 µs; a shorter pulse
    tilts up; 1740 µs = 10° down → `us_per_deg` −10; safe range 1000–1750 µs →
    `min_deg` −10, `max_deg` 60.
- **Verified:**
  - On the robot (user): every `servo_test.py --us` move's readback matched its command
    (800–1900 µs); the 9 V motor supply is separate from the servo's 5.7 V buck.
  - One twitch at power-up with no signal (old firmware), then it held: normal.
  - `us_per_deg` is NOT confirmed. The up-side reading (1440 µs = 6° up, i.e. 33 µs/deg)
    disagrees with the down side (10 µs/deg); I took the down side, which matches the servo
    spec. Pending check: `--deg 15` should read 15° up on the inclinometer.
  - Unit tests OK on the host.
- **Scope:** In scope.
- **Decisions:** none new.
- **Open / next:**
  - Confirm 15° up, then:
    - the servo box in ROADMAP Phase 4 (robot half);
    - `lens_from_pivot_m`;
    - robot bin-check runs.

### 2026-11-21: Servo rate re-measured

- **Phase:** 4
- **Branch / commit:** `walle-phase-4` @ uncommitted
- **Asked for:** the user re-measured the tilt against the floor. `--deg 15` (1490 µs, at the
  old 10 µs/deg) read about 18° up.
- **Done:** `servo.yaml` (robot) `us_per_deg` −8.3 (150 µs / 18°), `min_deg` −12, `max_deg`
  70; 15° up is now 1516 µs.
- **Verified:** mapping printed; unit tests OK. Not re-checked on the robot yet: `--deg 15`
  should now read about 15° up.
- **Scope:** In scope. **Decisions:** none.
- **Open / next:**
  - `level_us` 1640 came from the first (phone) method; re-check that 1640 µs reads 0° with
    the floor method.
  - Then `lens_from_pivot_m` and the robot bin-check runs.

### 2026-11-21: Servo level re-measured

- **Phase:** 4
- **Branch / commit:** `walle-phase-4` @ uncommitted
- **Asked for:** the user found level at 1575 µs with the floor method, and said the 15° test
  "looks good".
- **Done:** `servo.yaml` `level_us` 1575.
- **Verified:** not on the robot. Inconsistent: that 15° test sent 1516 µs (still with the
  1640 level), only 59 µs from the true level, which implies about 4 µs/deg, not the −8.3
  in the file. The earlier 1490 µs = 18° reading also gives about 4.7. The rate stays at
  −8.3 (marked unsettled) until a longer move is measured.
- **Scope:** In scope. **Decisions:** none.
- **Open / next:**
  - The user measures the angle at 1275 µs; then set `us_per_deg` and the limits from it.

### 2026-11-21: Servo calibration settled

- **Phase:** 4
- **Branch / commit:** `walle-phase-4` @ uncommitted
- **Asked for:** the user measured 1275 µs (300 µs up from the 1575 µs level) at about 30°
  up, against the floor, and sent a photo of the mount at that angle (it agrees).
- **Done:** `servo.yaml` (robot): `level_us` 1575, `us_per_deg` −10, `min_deg` −15 (1725 µs),
  `max_deg` 55 (1025 µs), inside the measured 1000–1750 µs range; 15° up = 1425 µs. The
  short-move readings (4–5 µs/deg) were reading error on small angles.
- **Verified:** mapping printed; unit tests OK. On the robot: readbacks matched every
  command; the angles are the user's estimates. Still to see: `--deg 15` under the final
  values reads about 15°.
- **Scope:** In scope. **Decisions:** none.
- **Open / next:**
  - `lens_from_pivot_m`: from the photo the lens sits well above the servo's axis (several
    cm), so it moves when the camera tilts; the user to measure it with the camera level.
  - Then tick the robot half of the servo box, and the robot bin-check runs.

### 2026-11-21: Lens offset from the servo axis

- **Phase:** 4
- **Branch / commit:** `walle-phase-4` @ uncommitted
- **Asked for:** the user measured the lens 4 cm above the servo horn's center, its edge 1 cm
  ahead.
- **Done:** `servo.yaml` `lens_from_pivot_m` [0.01, 0.04].
- **Verified:** the geometry prints the lens at 12.5 cm ahead / 12.5 cm up at level, and
  ~1 cm farther back at 15° up; unit tests OK. Robot: not run.
- **Scope:** In scope. **Decisions:** none.
- **Open / next:** `--deg 15` check on the robot, then the full route with the bin check.

### 2026-11-21: Robot servo box ticked

- **Phase:** 4
- **Branch / commit:** `walle-phase-4` @ uncommitted
- **Asked for:** the user confirmed `--deg 15` reads about 15° up, and asked what overflow
  levels 0–3 mean before setting the scene for the first robot bin-check run.
- **Done:** ROADMAP Phase 4: the servo box ticked (sim and robot).
- **Verified:** user's measurement on the robot (about 15°, against the floor).
- **Scope:** In scope. **Decisions:** none.
- **Open / next:** the user's robot run with the bin check, with ground truth noted per bin.

### 2026-11-22: First robot run with the bin check (20261122_134315)

- **Phase:** 4
- **Branch / commit:** `walle-phase-4` @ uncommitted
- **Asked for:** the user ran the route with the bin check and gave the ground truth:
  - bin 1 clean;
  - bin 2: 1 item on top;
  - bin 3: 2 on top (plastic bag, foil) and 3 around (paper towel, yogurt lid, plastic bag);
  - bin 4: 1 on top (cardboard box) and 3 around (box, plastic bottle, plastic wrapping).
  Also asked why `results/detect/` has no folder: only `eval_photos.py` writes there; a
  route run keeps its detections in its own folder.
- **Done:** scored the run (no code changed). The results were written only after the user
  answered the end-of-run measurement prompts (the follower was waiting there).
- **Verified (robot):**
  - Route: done, all 4 bins centered, end error measured 0 cm. A 1.85 s sensor gap was
    handled by the guard. 13 tilt moves all reached their target; never drove tilted.
    HC-SR04 `echo_us` −2 throughout (no response, as before).
  - Bin checks:

    | Bin | Truth | Judged | Right? |
    |---|---|---|---|
    | 1 | clean | overflowing L1 | wrong: a 17×19 px background box, 35 cm above the rim, called a paper bag on top |
    | 2 | L1 | overflowing L1, top 1 | right |
    | 3 | overflowing L1, 2 on top, 3 litter | litter L0, litter 2 | wrong: up-photo bin only at conf 0.025, dropped, so its items were all around_bin; yogurt lid missed |
    | 4 | overflowing L1, 1 on top, 3 litter | overflowing L1, top 1, litter 1 | outcome right; bottle cut at frame bottom gets no floor position (my 2026-11-21 rule); clear wrapping not detected |

  - Outcome 2/4 = 0.50 (target 0.80: FAIL); level within ±1 4/4; litter within ±1 3/4.
  - Bin 4's level photo took the Ziploc box for the bin (the real bin, partly hidden by the
    bottle, wasn't detected).
  - Diagnostic: the distance from bin width is 0.57–0.83 m vs 0.757 expected.
- **Scope:** In scope (Phase 4 robot check).
- **Decisions:** none yet. Proposed to the user:
  1. accept a low-confidence bin of plausible size near straight ahead in the up look;
  2. on_bin only when the item rests on the rim (bottom ≤ ~10 cm above it);
  3. count bottom-cut floor items in front of the bin as litter;
  4. prefer bin boxes near straight ahead after centering.
- **Open / next:** the user's answer on the fixes; then sim + replay on this run's photos,
  and another robot run.

### 2026-11-22: Bin-stop judging fixes from the first robot run

- **Phase:** 4
- **Branch / commit:** `walle-phase-4` @ uncommitted
- **Asked for:** "Yes, all four" (the fixes proposed after run 20261122_134315).
- **Done:**
  - `geometry.analyze_bin_stop` takes `stop=` for the bin-check photos (not centering, not
    the Phase 3 eval), with settings in `detect.yaml` `bin_check`:
    1. a faint bin box (low_conf) within `low_conf_max_offset_deg` 8° is the bin when none
       passed;
    2. an on_bin item must rest on the rim: its bottom ≤ `on_top_max_gap_m` 0.10 m above it,
       otherwise `ignored`;
    3. an around_bin item cut at the frame's bottom gets the frame-bottom floor point
       (`obj_xy_from: frame_bottom`), only when that row is nearer than the bin (level look);
    4. bin boxes only within `pick_max_offset_deg` 12° of straight ahead.
  - The follower passes `stop=` for the level and up looks and records `ignored` in
    `detections.json`.
  - New `detect/replay_bin_checks.py`: re-judges a run's saved detections with the current
    rules, against `--truth`.
  - Sim harness: skips frame-bottom estimates in its floor-position check.
  - 4 unit tests, built on this run's real boxes.
  - CLAUDE.md layout; ROADMAP decision row.
- **Verified:**
  - Replay of 20261122_134315: outcome 4/4 (run: 2/4); levels all exact.
    - bin 1: speck ignored (32 cm above the rim) → clean;
    - bin 3: faint up-look bin (0.025) → overflowing, 2 on top;
    - bin 4: bottle counted → litter 2 of 3.
    - Litter is still 2/3 on bins 3 and 4 (yogurt lid, clear wrapping: detector misses).
    - Saved detections only, the detector not re-run.
  - Unit tests 107 OK (host; both containers OK).
  - Sim: `sim-bin-check` PASS on `bincheck` (before fix 4's final form) and on `mixed`
    (after); `sim-route-check MISPLACE=12` PASS.
  - Found and fixed on the way: fix 4 first measured "near" from the configured bearing,
    which threw out every real bin when bins were misplaced (MISPLACE=12 FAIL, 10
    problems). It now measures from straight ahead (`stop.ahead_deg`, default 0).
  - Robot: not re-run.
- **Scope:** In scope.
- **Decisions:** the four rules (ROADMAP decision log, 2026-11-22).
- **Open / next:** a second robot run to check the rules live; the detector misses (clear
  plastic, flat lids) belong to the item-accuracy box (tilted photo set).

### 2026-11-22: Second robot run (20261122_142402): bin 2 missed, start-of-run veer

- **Phase:** 4 (and Phase 1 route robustness)
- **Branch / commit:** `walle-phase-4` @ uncommitted
- **Asked for:** the user ran again with the same scene; replay showed 3/4 outcomes right.
  The robot "veered right" on the first side and ended 23 cm right of the start, a first.
- **Done:** analysis only (no code changed).
- **Verified (robot):**
  - Score: outcome 3/4 = 0.75 (target 0.80); level within ±1 4/4; litter within ±1 4/4
    (bin 3 2/3, bin 4 3/3).
  - Bin 2 judged clean: the bag on top hangs over the front of the rim, its box bottom 23 px
    below the bin box's top (margin 8 px), so it was around_bin. Same pattern as run 1's
    bin 3 level photo (9.5 px). The robot was ~0.42 m from bin 2 (box 364 px wide vs ~200
    expected) because of the offset below.
  - Veer:
    - In the first 0.5 s of driving, the fused estimate moved +0.40 m sideways (y) with its
      heading at 0° and the wheels moving 7 cm forward: an impossible 0.8 m/s. A glitch,
      not motion. The follower steered right to "get back", and the wheel odometry shows the
      real path ending 23 cm right of the line, kept for the whole lap.
    - Trigger: the Pico opened a NEW micro-ROS session at 14:24:04.99, 2.3 s after the
      follower started (the only other one today was at 13:43:04, the power-up before run
      1). `wheel_odometry` logged "Encoder counts jumped (Pico restarted?)" at 14:24:05.2.
    - Sensor guard: one 0.48 s gap; the repair moved 0 cm.
    - The exact path from restart to a 40 cm EKF step isn't pinned down from the logs.
- **Scope:** In scope (robot check); the veer is Phase 1 robustness found in Phase 4.
- **Decisions:** none yet. Proposed:
  1. on_bin margin relative to the bin box (~12% of its height) for overhanging items;
  2. the follower refuses to start or re-anchors when the Pico restarts;
  3. an estimate-jump guard: fused motion far beyond what the wheels and the speed allow →
     stop and repair from the wheels.
- **Open / next:** ask the user whether the Pico was reset or power-bumped at the start.

### 2026-11-22: Pico-restart and estimate-jump guards (route follower)

- **Phase:** 1 robustness, found in Phase 4
- **Branch / commit:** `walle-phase-4` @ uncommitted
- **Asked for:** "I like fixes 2 and 3". On fix 1 (the overhang margin) the user was concerned
  about rules hyper-specific to the current scene. Answered: fix 1 is geometry, not tied to
  item counts, but the real risk is checking rules on the scene they were tuned on. Fix 1
  is held back; future runs vary the setup with the rules frozen, scored with the replay
  tool (and the tilted photo set).
- **Done:**
  - `route_geometry.py`: `estimate_jump` (fused vs wheel motion over a window) and
    `pico_restarted` (ping_seq goes backwards, or the counts jump > 20000).
  - `route_follower.py`:
    - restart detection on each tick, recorded as a sensor gap;
    - while running: stop, `wait_settled` (2 s fresh), repair from the wheels;
    - during the start hold: take the start pose again (up to 3 tries);
    - `check_jump` every 0.25 s while moving: > 5 cm or > 20° more than the wheels → stop
      and repair from the wheels over that window;
    - `summary.json` `pico_restarts_s`, `estimate_jumps`;
    - settings in `route.yaml` `sensor_guard`.
  - walle-sim:
    - `/sim/restart` in `sim_pico_bridge.py` (counts and ping_seq from 0, offline, IMU
      heading from 0, calibration again, servo to boot);
    - `harness/jump_ekf.py` (shifts the EKF via `/set_pose`);
    - `route-check --restart AT FOR` / `--jump AT DY`, `make ... RESTART= JUMP=`.
  - Tests: 4 guard tests. CLAUDE.md commands; ROADMAP decision row.
- **Verified:**
  - Unit tests 111 OK (host, both containers).
  - Sim (`mixed`, `plastic_pid`):
    - `RESTART="5 2"` (during the start hold): detected, start pose retaken, PASS (3.9 cm);
    - `RESTART="13 2"` (driving): detected, stopped 2.0 s, repaired, PASS (1.3 cm);
    - `JUMP="13 0.30"`: caught (29 cm), repaired 30 cm, PASS (0.8 cm);
    - no false alarms with `MISPLACE=12`, `COAST=0.35`, `BLACKOUT="20 1.5"` and
      `sim-bin-check`, all PASS.
  - First try of restart detection by counts alone MISSED a restart early in a session
    (counts still small); fixed with ping_seq. The firmware bumps ping_seq for every ping,
    answered or not, so it works with the robot's silent HC-SR04.
  - Offline replay of the jump test on robot trajectories:
    - run 20261122_134315: no alarms (max excess 1 cm / 4°);
    - run 20261122_142402: alarm at 10.4 s (28 cm), 0.3 s into the drive;
    - run 20261121_041923: alarms at 115–119 s, the Wi-Fi ghost spin (a true glitch).
  - Robot: not run with the guards yet. Why the restart produced a 40 cm EKF step is still
    not pinned down.
- **Scope:** Drift (agreed): Phase 1 robustness during Phase 4.
- **Decisions:** ROADMAP decision log, 2026-11-22 (guards; fix 1 held back).
- **Open / next:**
  - Robot run with the guards and a varied scene (rules frozen), scored with
    `replay_bin_checks.py`.
  - Ask the user whether the Pico was reset or power-bumped at the start (possible brownout).

### 2026-11-22: Third robot run (20261122_145845): new scene, rules frozen

- **Phase:** 4
- **Branch / commit:** `walle-phase-4` @ uncommitted
- **Asked for:** the user ran a new scene with the rules unchanged:
  - bin 1: 2 on top;
  - bin 2: 1 litter (a milk jug);
  - bin 3: 3 on top, 3 litter;
  - bin 4: 6 on top. Backlit plastic wrapping; a shampoo bottle hidden behind the foil and
    tissue box. The user notes a robot that drove around the bin would see more, but there's
    no time to build that.
- **Done:** analysis only. A throwaway trial of the held-back fix 1 (overhang margin) on all
  three runs; no code changed.
- **Verified (robot):**
  - Route: done, end measured 0 cm, no veer. No Pico restarts, no estimate jumps; one 0.88 s
    Wi-Fi gap handled.
  - Score: outcome 3/4 (bin 2's litter missed); level within ±1 4/4; litter within ±1 3/4
    (bin 3: 1 of 3).
  - Why:
    - bin 2: the jug's box covered only its top 70 px, so it floated above the horizon with
      no floor point (a detector box error);
    - bin 3: the green bag on the floor wasn't detected (it was in run 1), the flat lid
      missed, and 3 items on top came back as one box;
    - bin 4: 2 of 6 on top seen (the foil ball and backlit wrapping missed, the shampoo
      hidden); L1 vs L2, within ±1;
    - bin 1: the paper towel on the rim overhangs 18–23 px, so it was around_bin. The fix 1
      case on a THIRD scene with a different item.
  - Overall:
    - runs 2–3, rules frozen after run 1: outcome 6/8 = 0.75 (below 0.80), level ±1 8/8,
      litter ±1 7/8;
    - all three with today's rules: 10/12 = 0.83 (run 1 is what the rules were tuned on).
  - Fix 1 trial (margin max(8 px, 12% of the bin box height)): run 3 bin 1's on-top count
    becomes exact (2); no outcome changes and no regressions across the three runs.
  - Run 2's bin 2 stays wrong under fix 1: the robot was 23 cm off, so the rests-on-rim
    height (which uses the planned distance) read 13 cm. The jump guard targets that offset.
- **Scope:** In scope.
- **Decisions:** none yet. Fix 1 has evidence from a scene it wasn't tuned on; asking the user.
  The robot accuracy box stays open; ROADMAP notes the numbers.
- **Open / next:**
  - Decide fix 1.
  - Most remaining misses are detector misses: Phase 3's item-accuracy box and the tilted
    photo set.
  - Scoring note: items no camera at the stop can see (bin 4's shampoo) count against the
    robot; the user may want to score against visible items.

### 2026-11-22: Overhang fix; Phase 4 closed; commit

- **Phase:** 4 → 5
- **Branch / commit:** `walle-phase-4` (the Phase 4 commit; the user opens the PRs, phase 3
  first)
- **Asked for:** "apply the fix, and then commit the changes to Phase 4". Before that the
  user judged the camera close to its limit and asked whether to call it good enough and
  move on (agreed).
- **Done:**
  - Fix 1: in the bin-check photos an item may hang `bin_check.rim_margin_frac` 0.12 of the
    bin box's height below its top edge and still be on the bin (at least the old 8 px);
    one unit test from run 3's real boxes. Phase 3's rule (eval, centering) is unchanged.
  - ROADMAP:
    - Phase 4 ✅ closed with recorded limits;
    - robot bin-check box accepted below target (0.75 fair / 0.83 all);
    - item accuracy (tilted photo set) deferred;
    - decision row.
- **Verified:**
  - Unit tests 112 OK (host, both containers).
  - `make sim-bin-check` PASS (`mixed`).
  - Replays:
    - run 3: bin 1's on-top count is now exact (2); outcomes 3/4;
    - runs 1 and 2 unchanged (4/4, 3/4).
- **Scope:** In scope. Phase 4's two unmet boxes are closed by the user's decision, not met.
- **Decisions:** Phase 4 closed with limits (ROADMAP decision log).
- **Not committed, on purpose:**
  - the user's handoff-doc moves (`docs/handoff/` deletions and the `phase*-to-phase*`
    folders);
  - `walle-app/data/photos/_props.jpg`;
  - the CLIP weight copies, `.DS_Store`, `.vscode/`.
- **Open / next:** the user merges Phase 3, then Phase 4; then Phase 5 (start in plan mode).

### 2026-11-22: Commit on the wrong branch, fixed; Phase 4 → 5 handoff

- **Phase:** 4 → 5
- **Branch / commit:** `walle-phase-4` @ `4eee476` (this entry and the handoff docs are
  uncommitted)
- **Asked for:**
  - after the Phase 4 commit, "I can't checkout to main or walle-phase-4 because I have
    untracked changes. I don't want to do a git stash";
  - then a session summary, a Phase 5 start prompt structured like the earlier handoffs,
    and the initial Phase 4 plan in `docs/plans`.
- **Done:**
  - The Phase 4 commit had landed on `walle-phase-3`: the user switched the checkout there
    (reflog 10:52:45) 3 minutes before I committed, and I didn't re-check the branch.
    - `walle-phase-4` was set to the commit (`4eee476`);
    - `walle-phase-3` was reset to `177ec2b` (mixed, so the working tree was untouched);
    - all 104 committed files were checked byte for byte against the disk.
  - To leave `walle-phase-3` without a stash: the working tree equalled `4eee476` (checked
    again), so HEAD was pointed at `walle-phase-4` and the index refreshed. No file changed.
  - `docs/handoff/phase4-to-phase-5/session-2026-11-21-summary.md` and `phase-5-prompt.md`.
    The prompt flags three things for plan mode:
    - the HC-SR04 never answering, which the roadmap's Phase 5 relies on;
    - the level camera seeing the floor only from ~0.41 m ahead of the axle;
    - the 5–8 s detector.
  - `docs/plans/phase-4-initial-plan.md`: the plan as approved in plan mode, before the
    "as built" notes in `phase-4-plan.md`.
- **Verified:**
  - Branch positions and the byte-for-byte file check (git).
  - The prompt's facts were checked against `walle-phase-4`: obstacle code in `park.py`,
    the memory store's obstacle tables, `sim_truth.py` without the obstacle, the follower's
    obstacle refusal, route speeds and footprint, the sim HC-SR04 model.
  - Nothing run.
- **Scope:** In scope.
- **Decisions:** none.
- **Found afterwards (user):** the HC-SR04 was unplugged during the Phase 4 robot runs, which
  explains `echo_us` −2 in all of them (the bin-check `echo_us` diagnostic and earlier notes
  that it "never answered" in Phase 4). The user plugs it back in for Phase 5. The prompt
  now starts Phase 5 by checking that it answers.
- **Open / next:** the user merges the Phase 4 PR; Phase 5 in a new session from
  `docs/handoff/phase4-to-phase-5/phase-5-prompt.md`. This entry, the new handoff folder and
  `phase-4-initial-plan.md` are committed (user's go-ahead). The user's handoff-doc moves
  and `_props.jpg` are not.

### 2026-11-22: Phase 5 started: branch, orientation, plan approved

- **Phase:** 5 (obstacle detection and avoidance)
- **Branch / commit:** `walle-phase-5` (from `origin/main` @ `4d5c4ee`, upstream unset) @
  uncommitted
- **Asked for:** read the Phase 5 start prompt and the Phase 4 summary, then "Yes, start".
- **Done:**
  - Phase 4 was merged (PR #12, with the handoff commit and the last ledger entry), so the
    branch was cut from `origin/main` per the prompt.
  - Read `ROADMAP.md`, the ledger, the memory README, the follower (plan, `send()` guards,
    `side_leg`, `turn_to`, the obstacle refusal), `park.py build_obstacle`, `sim_truth.py`,
    the store's obstacle API, the sim HC-SR04 model and `park_world.py`.
  - Plan mode: the nine open decisions settled with the user; plan in
    `docs/plans/phase-5-plan.md`; ROADMAP Phase 5 marked 🟨, N/M/X filled in, decision row.
- **Verified:** nothing run; only read the code.
- **Notes for the build:**
  - The sim HC-SR04 is an ideal lidar cone 7 cm up: it sees every sim item, flat or not.
    The real one's behavior on flat and soft items is unknown (Step 0).
  - At 0.25 along a side, a small obstacle's near face can be 0.19 m ahead of the axle,
    nearer than the tilted camera's 0.28 m: the sonar or a box cut off at the frame's
    bottom has to place it. That's one of the fixed sim cases.
  - The sim camera is the Module 6 one (0.10 m high, 0.07 m ahead), not the robot's.
- **Scope:** In scope.
- **Decisions:** Phase 5 plan (ROADMAP decision log, 2026-11-22).
- **Open / next:** Step 0, the HC-SR04 check on the robot (user): echo at 20/40/80 cm, a
  flat bag and a paper towel at 30 cm, the sensor's height. Then build sim first, as in the
  plan.

### 2026-11-22: Phase 5 Step 0: the HC-SR04 answers (robot)

- **Phase:** 5
- **Branch / commit:** `walle-phase-5` @ uncommitted
- **Asked for:** the user ran the plan's Step 0 check (`ros2 topic echo /pico/wheel_ticks`).
- **Done:** recorded the results; no code changed.
- **Verified (robot, user's readings; distance = echo_us × 343 / 2e6):**

  | Placed at | echo_us | Reads | Spread |
  |---|---|---|---|
  | 20 cm | 1025–1026 | 17.6 cm | ±0.5 µs |
  | 40 cm | 2332–2333 | 40.0 cm | ±0.5 µs |
  | 80 cm | 4687–4780 | 80.4–82.0 cm | ~1.6 cm |
  | milk jug, ~30 cm | 1876 ± 3 | 32.2 cm | ±0.05 cm |

  - It answers and tracks distance. The −2 in Phase 4 was the unplugged sensor (as found).
  - Steady at the 0.35 m trigger range; noisier at 80 cm (beyond what Phase 5 uses).
  - The 20 cm reading is 2.4 cm short while 40 cm is exact. That's likely where the tape was
    measured from, not checked.
  - The sensor's center is 8.8 cm above the floor (sim: 7 cm; not changed, see below).
  - Idle `servo_us` read 1500 (firmware start value; the follower levels at 1575).
- **Not yet measured:** a flat bag and a paper towel at 30 cm (asked).
- **Scope:** In scope.
- **Decisions:** none. Keeping the sim sensor at 7 cm: the walle-sim robot is the Module 6
  model on purpose, and its camera is lower too.
- **Open / next:** the flat-bag and paper-towel readings, then the sim build.

### 2026-11-22: Phase 5 Step 0: flat items (robot)

- **Phase:** 5
- **Branch / commit:** `walle-phase-5` @ uncommitted
- **Asked for:** the user tested the flat items from the Step 0 check.
- **Verified (robot, user):**
  - A paper towel and the spinach bag lying flat: below the sensor's beam (8.8 cm up), so no
    echo from them.
  - The bag crumpled up: echo_us ≈ 1560 (26.8 cm), so it is seen once it has some height.
- **Meaning for the plan:** as the plan assumed, flat items are found only by the corner
  photo; the sonar is the trigger for items with height. A flat item the camera also misses
  would be driven over (a flat bag or towel, low risk to the wheels). No change to the plan.
  The robot's flat-bag run (side 2, robot test) checks the camera-only path.
- **Scope:** In scope.
- **Decisions:** none.
- **Open / next:** Step 0 is done; start the sim build.

### 2026-11-22: Phase 5 built; sim check 9/10

- **Phase:** 5
- **Branch / commit:** `walle-phase-5` @ uncommitted
- **Asked for:** "Yes, start building" (the approved plan, `docs/plans/phase-5-plan.md`).
- **Done:**
  - `walle-app/route/obstacle.py` (new, no ROS):
    - side frame and echo → metres;
    - floor position and radius from a box (`footprint_from_box`, flags boxes cut off at
      the frame's bottom);
    - corridor test (`blocks_side`);
    - `SonarTrigger`: 3 pings ≤ 0.35 m, nearer than the leg's end;
    - `plan_detour`: an approach on the line (backward if too near), then out to the left,
      past, back; clamped to the far corner, with a depth allowance;
    - `min_clearance` (body rectangle vs a point or circle).
  - `route_geometry.build_plan(look_sides=)`: a `look_side` step before each side.
  - `route_follower.py`:
    - the obstacle refusal is gone; the follower is never given the obstacle;
    - `look_side`: tilt down (−15°, the sim clamps to −10°), photo, detect, level;
    - the sonar `watch` in `side_leg` / `drive_side`;
    - `drive_with_obstacles` (detours; the sonar refines or adds obstacles);
    - fallback `ObstacleBlocked` → status `blocked`;
    - `obstacles.json`; summary `obstacles`, `sides_done`; `--no-obstacles`,
      `--sim-hide-obstacle`;
    - the detector and servo now load for obstacle looks even without bin checks;
    - `layout.json` keeps the given world's obstacle.
  - `config/route.yaml` `obstacle:` section (sim override: sensor 7 cm ahead).
  - `detect/sim_truth.py`: projects the obstacle (`hide_obstacle`).
  - `memory/route_records.py` `record_obstacles`:
    - a detection (place `obstacle`) + `report_obstacle` per record;
    - `mark_absent` for store obstacles on driven sides that weren't seen, stamped with the
      run's end time (rebuild-safe).
  - `detect/replay_obstacles.py` (sim truth or `--truth "SIDE,ALONG_M[,CROSS_CM]"`).
  - Sim:
    - scenarios `obstacle_near` (can at 0.25), `obstacle_far` (cup at 0.75),
      `obstacle_lying` (bottle);
    - profile `plastic_pid_sonar_dead`;
    - `sim_trials.py route-check` accepts obstacles and scores them (found and passed,
      position, false obstacles, collision vs the true outline, memory);
    - the Phase 3 memory count leaves out obstacle detections;
    - `harness/obstacle_trials.py` + `make sim-obstacle-check`.
  - `tests/test_obstacle.py` (15 tests). `CLAUDE.md` commands and layout; the plan's "as
    built" notes; ROADMAP sim boxes ticked.
- **Verified:**
  - Unit tests 127 OK on the host (3.13) and in both containers (3.10).
  - Sim, no obstacle (`mixed`, route-check, before the last edits): PASS. Four corner looks
    "clear", no sonar stops, end 0.8 cm, every bin check right.
  - Sim, `make sim-obstacle-check` (plastic_pid, dwell 5 s): **9/10 PASS**; zero
    collisions in all 10; positions 0.3–9.4 cm.
    - Closest pass: the `far` case, 2.9 cm (under the 5 cm margin; position error 2.5 cm).
    - Miss: `camera_miss` (sonar only). Default radius 0.10 m at 0.80 along → "no room to
      turn after it" → blocked, stopped 30 cm clear. The fallback worked; the lap didn't
      finish.
  - Memory rebuild from the 22 sim run logs twice: identical stores; obstacles cleared on
    later runs as designed.
  - **Not verified:** anything on the robot. `sim-bin-check` was not rerun (the obstacle
    cases ran every bin check, all OK). The `mixed` no-obstacle check wasn't rerun after
    the last obstacle-path edits.
- **Found on the way:**
  - The first 10-case batch scored 3/10 because the harness's `stop_world` missed
    `odometry_node.py`. Twelve copies published `/wheel/odom` at once; the jump guard fired
    on the mess, and its repairs threw the estimate off.
  - Fixed: process groups, plus a refusal to start with leftovers. That batch's results
    are void.
  - The same batch showed the depth problem (the lying bottle) and a too-round collision
    check. Both were fixed before the valid batch.
- **Scope:** In scope. Added beyond the plan: `assumed_depth_m` (a planning rule from a
  sim failure) and `max_hits_per_side`.
- **Decisions:** none new for the user yet. The depth allowance is a safety rule (config);
  flagged for review.
- **Open / next:**
  - The robot test (user): 3 runs (coffee can side 1 ~0.5; milk jug side 3 ~0.35; flat bag
    side 2 ~0.6), scored with `replay_obstacles.py`.
  - Sonar-only obstacles near a far corner block: decide whether that's acceptable or
    whether a sonar-only object should get a smaller default size or a second look.
  - Nothing committed.

### 2026-11-22: First Phase 5 robot run (20261122_204947); tape rejected, grouped detours

- **Phase:** 5
- **Branch / commit:** `walle-phase-5` @ uncommitted
- **Asked for:** the user ran the robot test (coffee can on side 1): "It successfully found
  the coffee can and avoided it, but then it hallucinated an obstacle on the side from Bin
  3 to Bin 4". Then "Implement the proposed fixes".
- **Robot run 20261122_204947 (user's run, rules as built):**
  - Side 1: can found by the corner look at 0.48 m (r 4 cm, a clean box), detoured 20 cm
    inside, rejoined. The user's placement was "~0.5"; not tape-measured.
  - Bins 2 and 3: centered; bin checks ran.
  - Side 3: the look returned `paper_towel` at 0.79 (r 15 cm) and `paper_cup` at 0.44.
    The side was empty floor; both boxes were floor tape. A re-run of the detector on the
    photo: the FIND step named all three boxes "tape", the NAME step still picked
    paper_towel and paper_cup for two of them (its not_trash list had no tape).
  - The follower detoured round the cup (35 cm inside), which also passed the "towel".
    It then tried a second detour from beyond it ("no room to turn after it") and stopped
    `blocked`. The fallback was safe; the logic was wrong.
  - The far tape line (~1.0 m along) was placed at 0.79 m: positions far from the robot
    are ~20 cm off. Likely the −15° tilt's real angle (calibrated on large moves only);
    not checked.
  - `robot.sqlite` now holds the two false obstacles as current on side 3; a later clean
    run on side 3 clears them (not removed by hand).
- **Done:**
  - `detect/detector.py` + `config/detect.yaml` `vlm.reject_names` [tape, tapes, marking,
    markings]: a FIND name with one of these words is dropped before the NAME step (kept
    in low_conf, `rejected_by`). The NAME prompt's not_trash list adds "tape or markings
    stuck on the floor".
  - `route/obstacle.py` `plan_detour_group` (one rectangle round several objects;
    `plan_detour` is the one-object case) and `group_ahead` (adds every pending obstacle
    whose near face is within the detour's reach).
  - `route_follower.py`: detours by group; a blocked group is refused before moving; each
    look keeps the VLM's names/descriptions and the boxes it rejected (`rejected`).
  - Tests: grouping (neighbours cleared, this run's geometry refused up front, far
    objects kept apart), tape names. 131 in all.
- **Verified:**
  - Unit tests: 131 OK on the host (one skip, as before) and in both containers.
  - `make detect-eval` (Phase 3 set, 52 photos): recall 0.787 → 0.812, precision 0.768 →
    0.812, category 0.945 → 0.973, bins 1.0, median 7.8 s. No box in the set was named
    tape, so the filter removed nothing there; the change comes from the prompt line and
    model variation. Read as "no regression", not as a gain.
  - Sim regression `make sim-obstacle-check CASES="random_11 far lying"`: 3/3, positions
    0.7–9.7 cm, no collisions.
  - **Not verified:** two obstacles in the sim (park.py allows one per lap, a Phase 0
    decision; changing it needs the user); the fixes on the robot.
- **Scope:** In scope (fixes to Phase 5 from its robot test). The detector change also
  touches Phase 3/4 bin-stop detections, checked by the eval above.
- **Decisions:** none recorded in the roadmap yet; the tape rule is a detector setting
  (`detect.yaml`), flagged for the user.
- **Open / next:**
  - Rerun the robot test (same can run, then the milk jug and the flat bag).
  - Optional: check the −15° tilt angle (far positions ~20 cm off).
  - Optional: allow a second obstacle in the sim to test grouping end to end.
  - Nothing committed.

### 2026-11-22: Second robot run (20261122_211939); re-look from closer, size floor

- **Phase:** 5
- **Branch / commit:** `walle-phase-5` @ uncommitted
- **Asked for:** the user reran the coffee can: "unfortunately it hallucinated on side 3
  again". After the diagnosis: "Let's start with 1 and 3 and see if that makes a
  difference" (re-look from closer; a size floor).
- **Robot run 20261122_211939 (rules as of the previous entry):**
  - Side 1: can at 0.47 m, detoured, rejoined (as in the first run).
  - Side 3: no tape reported (the tape filter worked). One `paper_towel` ("tissue") box,
    13 × 7 px, placed at 0.92 m → inside the corner turn zone → `blocked`.
  - The box is a real ~2 cm speck, seen in both runs' photos, BEYOND the corner 4 cross
    line in the image. The model placed the cross line (1.00 m from the camera at corner
    3) at 0.69 m. At −11° tilt it would read 0.96 m, and the speck ~1.66 m (off the
    route). Far positions are ~30% short: the tilt angle or the nominal 47° vertical FOV
    is off. Not calibrated (deferred, option 2).
- **Done:**
  - `route/obstacle.py` `footprint_from_box`: also returns `width_m` (as seen, before
    the radius clamp) and `dist_m` (center from the axle).
  - Size floor: `min_size_m` 0 → 0.04 m, judged on `width_m` (the clamped radius could
    never be under 6 cm).
  - Re-look: a corner-look item farther than `trust_range_m` 0.70 m is unconfirmed. The
    robot drives on the line (sonar on) to `relook_at_m` 0.60 m before it, looks again
    (`relook`), drops the unconfirmed ones (`action` `not_confirmed`) and keeps what the
    closer look finds (confirmed if within 0.70 m). A sonar echo on an unconfirmed one
    confirms it. At most `max_relooks` 3 per side; no photo → act on the corner look.
  - `not_confirmed` records are skipped by the memory import, `replay_obstacles.py` and
    the sim check.
  - Tests: width/distance, the run's speck (under the floor, far), import skips
    unconfirmed. 132 in all.
  - ROADMAP decision row (the size floor replaces "avoid everything", the user's call).
- **Verified:**
  - Unit tests 132 OK on the host (one skip) and in both containers.
  - `make sim-obstacle-check`: 9/10 again, positions 0.3–8.9 cm, no collisions.
    - `far` (cup at 0.75): corner look unconfirmed → re-look → confirmed → detour.
    - The other eight passing cases were within 0.7 m: no re-look.
    - Same miss as before: `camera_miss` (sonar only, near the far corner) → `blocked`.
  - Not run on the robot.
  - Run 211939's speck replayed through the new rules (unit test only): under 4 cm wide,
    and farther than 0.7 m, so it'd be both dropped and re-looked.
- **Scope:** In scope (fixes from the robot test). The size floor reverses a plan-mode
  decision, with the user's agreement.
- **Decisions:** ROADMAP decision log, 2026-11-22 (size floor, re-look, tape, grouping).
- **Open / next:**
  - The robot rerun (can on side 1, then the milk jug and the flat bag).
  - Calibrating the down-tilted camera (option 2) when the user has time.
  - `robot.sqlite` still holds run 204947's two tape "obstacles" and run 211939's speck as
    current on side 3. A clean pass on side 3 clears them.
  - Nothing committed.

### 2026-11-23: Robot test runs (6), tape patches removed, detour width 0.50 m

- **Phase:** 5
- **Branch / commit:** `walle-phase-5` @ uncommitted
- **Asked for:** the user ran the three planned robot tests twice. The first set showed a
  false obstacle in each run. The user then removed the loose tape from the floor and ran
  the set again: "The runs were a lot better."
- **First set (rules as of the previous entry; placements "about", not tape-measured):**

  | Run | Real obstacle | Result | Position | False obstacle |
  |---|---|---|---|---|
  | `20261123_034402` | can, side 1 ~0.5 | passed, end 0 cm | 5.4 cm | side 3, 0.43 m |
  | `20261123_035324` | milk jug, side 3 ~0.35 | passed, end 0 cm | 5.1 cm | side 1, 0.40 m |
  | `20261123_040037` | flat bag, side 2 ~0.6 | found, blocked: detour 0.41 m > max 0.40 | 12.2 cm | none |

  - Both false obstacles were flat tape rectangles left of the line (an older layout's
    tape), found as "paper"/"tissue", so `reject_names` didn't apply.
  - A reworded NAME prompt describing flat tape patches was tested on six photos
    (3 false, can, jug, bag): identical results. The NAME step sees only the crop, and a
    cropped flat white rectangle looks like paper. Prompt change not kept.
- **Second set (tape patches removed by the user; same rules):**

  | Run | Real obstacle | Result | Position | False obstacle |
  |---|---|---|---|---|
  | `20261123_042112` | can, side 1 ~0.5 | passed, end 0 cm (est; not measured) | 5.6 cm | none |
  | `20261123_042752` | milk jug, side 3 ~0.35 | passed, end 0 cm | 7.3 cm | none |
  | `20261123_043408` | flat bag, side 2 ~0.6 | passed (detour 30 cm inside, rejoined at corner 3), end 0 cm | 7.3 cm | side 3, 0.41 m |

  - The remaining false obstacle is one more tape rectangle still on the floor near
    corner 3 (left of the line; visible in that look's photo).
  - The bag was labelled `paper_bag` (recycling) instead of `plastic_bag` (trash).
  - Run 043408's bin_1 centering lost a photo (camera timeout); unrelated to Phase 5.
- **Done:** `route.yaml` `max_offset_m` 0.40 → 0.50 (keeps ≥ 0.5 m from the opposite
  side). Proposed last turn ("unless you object"); not objected to.
- **Verified:** unit tests OK on the host after the change; not rerun in the containers
  or the sim (config value only; the sim's widest detour is ~0.27 m).
- **Scope:** In scope.
- **Decisions:** detour width 0.50 m (ROADMAP decision row below).
- **Open / next:**
  - The robot done box ("passes a real obstacle and rejoins it, no contact in ≥ 2 of 3")
    looks met: 3/3 in the second set. Not ticked until the user confirms there was no
    contact.
  - The last tape patch near corner 3.
  - `robot.sqlite` holds the false obstacles from the test runs; a clean pass clears them.
  - Nothing committed.

### 2026-11-23: Contact with the bag (run 20261123_043408); detours clear of other sides' obstacles

- **Phase:** 5
- **Branch / commit:** `walle-phase-5` @ uncommitted
- **Asked for:** the user reported "It touched the plastic bag in the last run when it was
  trying to avoid the patch it hallucinated on side 3".
- **Cause (reconstructed from the run's obstacles.json):**
  - The bag lay across side 2's line at 0.67 m, its inner end reaching toward corner 3.
  - Side 3's detour (round the false tape patch at 0.41 m) turned in place at (0.85,
    0.70), 15 cm inside corner 3. Its turn circle (0.14 m) overlapped the bag's footprint
    by ~11 cm.
  - The planner only considered obstacles on the side being driven. A real second
    obstacle there would have done the same.
- **Done:**
  - `route/obstacle.py` `plan_clearance`: the gap between a detour's swept area (a turn
    circle at each leg's start; body ±half width, ±turn radius along each leg) and round
    keep-out objects.
  - `route_follower.py`: every detour is checked against all obstacles this run found
    (other sides, not `not_confirmed`). Under the margin → `blocked`, with which obstacle
    and how close.
  - A test built from the run's numbers: the plan's clearance to the bag is < 0; the same
    bag 37 cm farther down clears.
  - ROADMAP: robot box ticked (can and jug runs without contact; 2/3 as agreed, the bag
    contact and its fix noted); decision row; Phase 5 status 🟨 "every done box met".
- **Verified:**
  - Unit tests 133 OK on the host and in the robot container. The sim container wasn't
    rerun.
  - Sim `CASES="random_11 far"`: 2/2. The sim has one obstacle per world, so this check
    never has a keep-out there; it shows only that nothing else changed.
  - **Not verified:** the fix on the robot (would have stopped `blocked` instead of
    touching the bag).
- **Scope:** In scope.
- **Decisions:** ROADMAP decision log, 2026-11-23.
- **Open / next:**
  - The user decides whether Phase 5 is closed. Suggested first: remove the last tape
    patch near corner 3 and rerun the bag case.
  - Nothing committed.

### 2026-11-23: Last bag run (20261123_045424); Phase 5 closed; commit

- **Phase:** 5 → 6
- **Branch / commit:** `walle-phase-5` (this commit; pushed to origin for the user's PR)
- **Asked for:** the user reran the bag case: "it hallucinated an obstacle on side 1, and
  then couldn't get around the bag on side 2". Then: "Let's move on with preparing for
  Phase 6. Please add the ledger entry and commit the current code, and please push it up
  to origin."
- **Run 20261123_045424 (rules as of the previous entry):**
  - Side 1: three flat tape rectangles still on the floor, left of the line, at about 0.39
    / 0.53 / 0.67 m. Each was named "white paper" → `paper_towel`, and all three went into
    one 34 cm detour (grouping as designed).
    - The same patches were in runs 042752 and 043408, where side 1 read clear. Whether a
      patch is flagged varies from run to run.
  - Side 2: the bag (28 cm wide) at 0.70 m, its far edge 0.84 m, 0.16 m from corner 3.
    - The turn at the corner needs 0.165 m (turn radius 0.14 + half the margin), so the
      robot reported `blocked` before moving.
    - In run 043408 the same bag read 0.67 m and passed.
    - The park's own rule allows a 14 cm-radius object only up to 0.67 along. The robot
      did what the agreed fallback says. No rule changed to make it pass (no tuning to one
      scene).
  - The other-sides clearance check never came into play (no detour was planned near a
    known obstacle). It is still not exercised on the robot.
- **Done:**
  - ROADMAP: Phase 5 ✅ closed with recorded limits (decision row).
  - This entry; the commit of all Phase 5 work since `origin/main` @ `4d5c4ee`. That
    includes:
    - the plan (`docs/plans/phase-5-plan.md`);
    - code, configs, scenarios, sim harness and tests;
    - the nine Phase 5 robot run folders (`results/route/20261122_204947` …
      `20261123_045424`) and `robot_runs.csv`;
    - the detector eval `results/detect/20261122_210854`.
- **Left out of the commit, on purpose:** `.DS_Store`, `.vscode/`,
  `walle-app/data/photos/_props.jpg` (as in Phase 4: ask first), and the CLIP weight
  copies (`weights/`, `walle-app/weights/`, `walle-pilot/weights/`).
- **Verified:** unit tests 133 OK (host, robot container). Sim and robot results as in the
  entries above. The confirmation run with the bag at ~0.5 m was not done (user's call to
  move on).
- **Scope:** In scope. Phase 5 closes on its done boxes, with the limits in the ROADMAP
  decision row.
- **Decisions:** Phase 5 closed with recorded limits (ROADMAP decision log).
- **Open / next:**
  - The user opens the PR from `walle-phase-5` and merges it; Phase 6 (the WALL-E ROSA
    agent) starts in plan mode.
  - `robot.sqlite` lists false obstacles from these test runs (tape patches, the speck).
    Before Phase 6 relies on "where are the obstacles right now?", rebuild it or let a
    clean lap clear them.
  - Optional: calibrate the down-tilted camera; remove the last tape patches on side 1.
