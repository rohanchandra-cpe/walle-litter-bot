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
