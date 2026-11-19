# Start prompt: Phase 2, memory store

Paste everything below the line into a new Claude Code session opened in the repository.

---

You are starting **Phase 2 (Memory store)** of the WALL-E application in this repository.
Work with me (the user) as the project owner; I run the real robot.

## First, orient yourself

1. Follow the "Start of every session" steps in `CLAUDE.md`. Read `ROADMAP.md` in full
   (Phase 2 especially, and Phases 3–6, which will write to and read from the store) and the
   latest entries in `LEDGER.md`. `docs/handoff/session-2026-11-16-summary.md` is the short
   version of Phase 1.
2. Check the branch. Phase 1 is on `walle-phase-1` (commit `596ba16` and later; Phase 0 on
   `implement-core-walle-functionality` is under it, and neither is merged to `main`). If
   `walle-phase-1` has uncommitted changes, tell me before doing anything. If Phase 1 is
   merged to `main`, create `walle-phase-2` from `origin/main`; if not, ask me whether to
   branch from `walle-phase-1`.
3. **Enter plan mode before writing any code** (`CLAUDE.md` working rule). Read the code
   listed below, settle the open decisions with me, and when I approve, copy the plan to
   `docs/plans/phase-2-plan.md`.

## What Phase 2 must deliver (from `ROADMAP.md`)

A persistent store of everything the robot observes, so it can answer history questions
later (Phase 6). Records each carry time, run id and position: route runs, detections
(label, trash or recycling, confidence, photo), obstacles (where, what, cleared or not),
and bin checks (bin id, fill estimate, overflow degree, litter count around it, photo).
Read queries the agent will need: most-seen objects, which bins overflow most often,
current obstacles on the route, recycling correctness. Sim runs and robot runs are kept
apart. Done when:
- the schema is written down and agreed with me;
- a test records a fake run through the write API, then the read queries return the
  expected answers;
- the store survives a container restart (it lives in the mounted project folder).

## The code you are building on

**Route runs already produce records (Phase 1).** `walle-app/route/route_follower.py` writes
`walle-app/results/route/<run_id>/`:
- `summary.json`: run id, source (`sim`/`robot`), scenario, status, distance, end errors,
  measured gyro drift, and `events`: one `arrived` per corner (poses: fused EKF, wheels, and
  in sim, ground truth) and one `facing_bin` per bin (heading error, photo path);
- `trajectory.csv` (10 Hz poses), `layout.json` (the park layout used), `photos/`.

Robot runs are also one row each in `results/route/robot_runs.csv`, with the tape-measured
end error. Progress goes out live on `/walle/route/status` (JSON). The route follower is the
natural first writer to the store. Today the bin stop only takes a photo; the real bin check
(fill level, litter) is Phase 4, and detections are Phase 3.

**Park (`walle-app/park.py`, `config/park.yaml`, `config/scenarios/`).**
`load_park(scenario, bins=, obstacle=, seed=)` gives the layout. That covers the route
corners, and bins with `id`/`name`/`corner`/`position`/`size_m`/`state`/`overflow_level`/
`litter`. It also covers the obstacle (item, category trash/recycling, side, position). In sim,
scenarios set bin states and litter, so they can serve as ground truth for the store's tests.
Bins have **no type** (trash vs recycling bin) yet: "recycling correctness" needs one.

**Existing vision outputs (`walle-pilot/vision/`, reuse, don't change).** `vision_node.py`
answers `/vision/ask` with JSON on `/vision/result` (vision-language model via Ollama) and
saves analyzed photos under `walle-pilot/vision/results/images/`, with a
`vision_log.csv`. Phase 3 replaces detection with YOLO-World, which will write to your store.

**Conventions (`CLAUDE.md`).** Python 3.10 (ROS 2 Humble container); settings in YAML under
`config/`; results under `results/`/`data/`; ROS nodes follow the `/x/ask` + `/x/result`
JSON pattern with a request id. The whole repository is mounted at `/workspace` in both the
robot (`make up`) and sim (`make sim-up`) containers, so a file under `walle-app/` persists
across container restarts. Phase 6's ROSA agent will query the store through
`langchain_core` tools.

## Real-world conditions (may change; I'll tell you)

- Bins: black plastic, 27.7 × 21.1 × 29.2 cm (W × D × H), one per corner, 0.71 m out on the
  diagonal. The square is on corrugated plastic sheets.
- The robot runs the closed-loop firmware (2026-11-18). Route following works on the robot
  (ends within about 1 cm); the HC-SR04 is not answering yet.
- Bin centering will use YOLO bin detection in Phase 3 (decided 2026-11-18).

## Open decisions to settle in plan mode

Recommend an option for each; record the outcomes in the `ROADMAP.md` decision log.
1. Storage: SQLite (the default candidate) or something else; plain tables, or also a
   vector store for free-text recall.
2. Schema: tables and fields for runs, detections, obstacles and bin checks. Positions in
   the park frame. How a detection links to a run, a bin and a photo. Whether to keep a
   "current state" (e.g. obstacles on the route now) or derive it from history.
3. Photos: stored (copied or referenced by path), and for how long.
4. "Recycling correctly": what it means in the data (an item seen in or near a bin of the
   wrong type?). That needs a bin type (trash / recycling) in `park.yaml`.
5. Sim vs robot separation: a `source` field, or separate database files.
6. The write path: a Python API that nodes import, a ROS node with `/memory/ask` +
   `/memory/result`, or both. Who writes in Phase 2: the route follower (runs and bin stops),
   and fake runs in the test.
7. Location and safety of the file: under `walle-app/data/`? Committed or git-ignored?
   Concurrent writers (the follower and a detection node at once).

## How to work

- Sim first where it applies: the store's own check is a headless unit or integration test
  with one command and a pass/fail result. Then, a sim route run should write its records, and
  a robot run should too (I run that).
- Settings in YAML under `config/`, results in CSV/JSON under `results/` or `data/`.
- Don't change the frozen study files (`walle-pilot/config/pilot.yaml`, `walle-pilot/pilot/`,
  `walle-pilot/data/schedule/`, logs under `walle-pilot/data/`).
- Add a `LEDGER.md` entry before ending any turn that changed files: what was verified and
  how, what wasn't, and any scope drift. Tick the `ROADMAP.md` boxes as they are met.
- Don't commit or push unless I ask. Never commit `wifi_config.h`.
