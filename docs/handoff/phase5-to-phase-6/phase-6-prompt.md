# Start prompt: Phase 6, the WALL-E ROSA agent

Paste everything below the line into a new Claude Code session opened in the repository.

---

You are starting **Phase 6 (WALL-E ROSA agent)** of the WALL-E application in this
repository. Work with me (the user) as the project owner; I run the real robot.

## First, orient yourself

1. **Read the project docs.** Follow the "Start of every session" steps in `CLAUDE.md`.
   - Read `ROADMAP.md` in full: Phase 6 especially, and Phase 7 (the stretch metrics and
     LLM judge, which will score this agent's answers).
   - Read the latest entries in `LEDGER.md`.
   - `docs/handoff/phase5-to-phase-6/session-2026-11-22-summary.md` is the short version of
     Phase 5.
   - `walle-app/memory/README.md` describes the store and its read queries; most of the
     agent's answers come from there.
   - `walle-pilot/ROSA_AGENT.md` describes the existing ROSA agents; follow their pattern.
2. **Branch.**
   - Phase 5 is merged to `main` (PR #13, `4cce337`).
   - **`walle-phase-6` already exists** (cut from `origin/main` after that merge) with one
     commit, `bd1614e` (the memory store cleanup). Work on it. Check that it is current with
     `origin/main` and tell me if it isn't.
   - **Run `git status -sb` right before every commit.** I sometimes switch branches between
     your turns.
   - Bring these along untouched, and ask me before committing any of them:
     - `docs/handoff/phase5-to-phase-6/` (this prompt and the summary), if still
       uncommitted;
     - `walle-app/data/photos/_props.jpg`.
   - Never commit the 340 MB CLIP copies (`weights/`, `walle-app/weights/`,
     `walle-pilot/weights/`), `.DS_Store`, `.vscode/` or the `.sqlite` stores.
3. **Enter plan mode before writing any Phase 6 code** (`CLAUDE.md` working rule). Read the
   code listed below and settle the open decisions with me. When I approve, copy the plan to
   `docs/plans/phase-6-plan.md`.

## What Phase 6 must deliver (from `ROADMAP.md`)

One ROSA agent that runs the whole thing from plain-language prompts:
- tools that call the finished capabilities: run the route, check a bin, look for
  obstacles, report an obstacle, and query the memory store;
- a system prompt describing the park, the bins and the tools.

Acceptance prompts, run in sim and on the robot:
1. "Run your route and check the path for any obstructions."
2. "Check Bin 1 to see if it is overflowing. If it is, estimate how much."
3. "Historically, what objects have you seen the most?"
4. "Are people generally recycling the correct objects?"
5. "Which bins overflow most often?"
6. "Where are the obstacles on the route right now?"

Done when:
- each acceptance prompt gives a correct action or answer in sim;
- each acceptance prompt works on the robot (I run this);
- history answers match the memory store's contents (checked by query).

## The code you are building on

### Existing ROSA agents (`walle-pilot/`, the pattern to follow)

- **Agents:** `odometry/odom_agent.py`, `vision/vision_agent.py`, `depth/depth_agent.py` and
  `range/range_agent.py` (read `range_agent.py` first; it is the shortest).
  - Each builds `ROSA(ros_version=2, llm=ChatOllama(...), tools=[...],
    prompts=RobotSystemPrompts(...))`;
  - `temperature=0`, `num_ctx=16384`, `max_iterations` 20 (range) or 40 (two of the
    others), `accumulate_chat_history=False`;
  - a plain `input("You> ")` loop.
- **Tools** are `langchain_core.tools.tool` functions that return short JSON-ish text.
  Their critical instructions tell the model to report only what the tools returned and
  never to invent a reading.
- **Approval:** `odometry/odom_tools.py` `_approved()` asks `[APPROVAL NEEDED] … Allow?
  [y/N]` on the terminal before any motion. Every motion tool does this today.
- **Models:** Ollama on the host (`OLLAMA_URL`, default
  `http://host.docker.internal:11434`). Installed there now: `qwen3.5:4b` (the vision
  agent's planner, and the detector's item model), `gemma4`, `llama3.2`,
  `nomic-embed-text` (the store's embeddings).
- **Packages:** both images carry `jpl-rosa`, `langchain-ollama<1.0` and
  `langchain-core>=0.3.52,<1.0`.
- **Frozen:** these files belong to the finished study. Reuse them (import), don't change
  their behavior.

### The route follower (`walle-app/route/route_follower.py`)

It's a CLI, `main(argv)`, that runs one lap.
- **Steps per lap:** start hold (10 s gyro calibration), then for each side:
  - **look:** a tilted-down photo for obstacles;
  - **drive:** with the sonar watching, and a detour if needed;
  - **at a bin's corner:** face the bin, center on it, a 15 s dwell photo, then the bin
    check (tilt up, judge, level).
- **Duration:** about **5–8 minutes on the robot**, ~2.5 min in Gazebo with a 5 s dwell.
- **Progress:** `/walle/route/status` (`std_msgs/String` JSON: state, step, corner, pose)
  while it runs.
- **Outputs in `results/route/<run_id>/`:**
  - `summary.json` (status `done` / `blocked` / `aborted`, reason, events);
  - `detections.json`, `bin_checks.json`, `obstacles.json`, `trajectory.csv`, photos;
  - then the run is recorded in the memory store.
- **On the robot** it ends with tape-measure prompts on stdin (`--no-measure` skips them).
  The run's files are written only after them, so an agent that starts the follower must
  pass `--no-measure` or answer them.
- **Useful flags:**
  - `--scenario`, `--bins N` (keep the first N corners' bins), `--no-bins`;
  - `--dwell`, `--laps`;
  - `--camera-url`, `--no-obstacles`, `--no-tilt`, `--no-memory`;
  - `--run-dir`.
- **Safety:**
  - Ctrl+C stops it; the Pico also stops by itself 500 ms after the last command.
  - Guards: stale data, Pico restart, estimate jump, never driving with the camera
    tilted.
  - Keep all of them, whatever the agent does.
- **There is no "go check one bin" mode.** "Check Bin 1" today means a full lap with bin
  1. `--bins 1` keeps only bin 1 (corner 1 is where the lap starts and ends). A partial
  route to one bin and back would be new motion code, so sim first. This is an open
  decision.

### Detection and bins (`walle-app/detect/`)

- **`detect_node.py`:** answers `/detect/ask` → `/detect/result` (the JSON pattern) with
  the same detector the follower uses. It needs `/camera/image/compressed`
  (`walle-pilot/vision/camera_node.py`, `launch/vision.launch.py`). A "look now" tool can
  use it without driving.
- **Bin check results** (`bin_check.py`, `bin_checks.json`):
  - `overflow_level` 0–3 by the count of items on top (1 / 4 / 7 → levels 1 / 2 / 3);
  - `on_top_count`, `litter_count`;
  - `outcome`: `overflowing` / `litter` / `clean`;
  - `method`: `tilt` / `level_only`.
  - **"Estimate how much"** = the level and the count. The bins are opaque, so there is no
    fill level (Phase 2/4 decision).
- **Accuracy limits** (Phases 3–5): robot bin-check outcome 0.75 on fair runs; items under
  lamp light weaker; plastic bags sometimes labelled `paper_bag`; flat tape on the floor
  read as paper. Answers should not claim more than the data supports.

### The memory store (`walle-app/memory/store.py`)

- **Read queries** (each returns plain dicts and lists, ready for JSON):

  | Question | Call |
  |---|---|
  | most-seen objects | `most_seen(limit=5, since=None, place=None)` |
  | which bins overflow most | `bin_overflow_ranking()` |
  | last check of a bin | `last_bin_check(bin_id)` |
  | obstacles now | `current_obstacles()`; `obstacle_history(id)` |
  | recycling correctness | `recycling_correctness()` (judged only by items on top of a bin, with a `basis` sentence saying so) |
  | anything else, in words | `recall(text, k=5)` (embeddings, keyword fallback) |
  | recent runs | `runs(limit=10)` |

- **Two stores, kept apart:**
  - **`robot.sqlite`**: rebuilt clean on 2026-11-23. 20 runs, 79 detections, 70 bin
    checks, 6 obstacles. **Current obstacle: the plastic bag on side 2**, from the last
    Phase 5 test run; if it's gone, a lap over side 2 clears it.
  - **`sim.sqlite`**: 74 runs, 578 detections, 284 bin checks, 28 obstacles. It has
    accumulated from every sim check, Phase 5's included, so its history is whatever the
    checks happened to place.
- **Rebuilds:** both stores rebuild from run folders (`memory/import_runs.py --rebuild`;
  `--source sim --runs <folder>` for sim).
  - Reviewed false obstacle records live in per-run `obstacle_review.json`, which the
    import applies.
  - A fixed fixture store may serve the history checks better than the live sim store.
    That's an open decision.
- Memory writes happen in the follower at the end of each run. The agent reads; it should
  not need to write, except perhaps "report an obstacle" (ROADMAP's tool list).

### Sim (`walle-sim/`)

- **The world is built at launch:**
  `make sim-launch WORLD=park SCENARIO=mixed GUI=false PROFILE=plastic_pid` (a ~30 s
  headless start). The scenario sets the bin states and the obstacle (`mixed`,
  `obstacle_only`, `bincheck`, `obstacle_near` / `far` / `lying`, `OBSTACLE=none|random|1-4`,
  `SEED`).
- **Ground truth:**
  - `/sim/ground_truth` gives the poses;
  - `detect/sim_truth.py` stands in for the detector (YOLO-World sees nothing in Gazebo);
  - `route-check` (`walle-sim/harness/sim_trials.py`) already compares a lap's bins and
    obstacle with the world.
- **Existing one-command checks:** `make sim-route-check`, `make sim-bin-check`,
  `make sim-obstacle-check`. A Phase 6 check would run the acceptance prompts against a
  launched world and compare actions and answers with the truth or the store. It should
  be headless and report pass/fail.
- **Gotchas:**
  - Stop a headless launch with bracketed `pkill` inside the container.
  - `sim-obstacle-check` refuses to start while any sim node is left over.
  - The sim camera and robot are the Module 6 ones (camera 0.10 m high), not the robot's.

## Real-world conditions (may change; I'll tell you)

- **Track:** a 1 m square taped on corrugated plastic sheets in my room, carpet beyond.
  - Bins stand 0.88 m out on each corner's diagonal; bins 1 and 3 are trash, 2 and 4
    recycling.
  - Loose tape patches were removed for Phase 5 (some may remain near corners 1 and 3).
- **Obstacles and trash:** my fake trash (milk jug, coffee can, tissue box, carton, bags,
  paper towels, foil, yogurt container).
- **Power:** a 9 V battery for the motors, a 5.7 V buck for the servo, and a separate 5 V
  supply for the Pico, the HC-SR04 and the camera.
- **Light:** room lamps at night, daylight by day. Detection is weaker under the lamps.
- **Containers:**
  - Never recreate only the `walle-pilot` container (the micro-ROS Agent shares its
    network).
  - Ollama runs on the host, and the detector, the embeddings and the agent's LLM all
    share it.
  - Don't run a detector eval while I drive.

## Open decisions to settle in plan mode

Recommend an option for each, and record the outcomes in the `ROADMAP.md` decision log.
1. **Which LLM runs the agent** (ROADMAP open decision):
   - a local Ollama model (`qwen3.5:4b` is installed; how reliable is a 4B model at
     choosing tools and passing arguments?);
   - a larger local model;
   - or a hosted model. Remote vision models were shelved in Phase 3 (kept local); the
     planner is a separate question.

   Measure tool-choice accuracy and latency on the six prompts before choosing.
2. **Approval rule** (ROADMAP open decision): today every motion asks a person. A route run
   probably needs one approval for the whole run, with Ctrl+C and the Pico watchdog kept.
3. **How the agent runs the route:**
   - start the follower as a subprocess (its CLI, `--no-measure`, then read
     `summary.json`), or import and drive it in-process, or make it a ROS node/action;
   - how the agent waits 5–8 minutes (ROSA's iteration and timeout limits) and reports
     progress (`/walle/route/status`);
   - how it stops a run.
4. **"Check Bin 1":** a full lap with `--bins 1`, or a new partial route (drive to bin 1's
   corner, face it, check it, return), which would be new motion code, sim first. Or
   answer from `last_bin_check(1)` when a check is recent.
5. **"Check the path for any obstructions":** the lap with obstacles on, reporting from
   `obstacles.json`; or also a look-only tool (`/detect/ask`) that doesn't drive. And what
   "report an obstacle" means as a tool.
6. **Grounding the answers:**
   - every history answer comes from a store query, not from the model's memory;
   - say which store (sim or robot);
   - say the limits (recycling correctness only counts items seen on top of a bin; labels
     can be wrong).
7. **Sim acceptance check:**
   - a headless script that sends the six prompts to the agent in a launched world and
     checks the tool calls and the answers against the store and the world;
   - whether history prompts run against a fixed fixture store;
   - the pass targets.
8. **Robot acceptance:** which prompts on the robot, the setup (bin states, an obstacle),
   and what I check.

## How to work

- **Sim first,** with a headless one-command check that reports pass/fail. A phase is done
  only after I run it on the real robot.
- **Robot safety:**
  - every motion still goes through the follower's `send()` (its guards) and keeps Ctrl+C;
  - the Pico stops 500 ms after the last `/pico/cmd_vel`; don't remove that watchdog;
  - an agent must never be the only way to stop the robot.
- **Don't change frozen files:** `walle-pilot/config/pilot.yaml`, `walle-pilot/pilot/`,
  `walle-pilot/data/schedule/`, the logs under `walle-pilot/data/`, and the depth
  calibration. Reuse `walle-pilot/odometry/`, `vision/` and `range/` (the existing agents
  and tools), but don't change their behavior.
- **Settings** go in YAML under `config/`; results go in CSV/JSON under `results/` or
  `data/`. New nodes use `rclpy` and the `/x/ask` + `/x/result` JSON pattern.
- **Test** on the host (Python 3.13) and in the containers (3.10):
  `python3 -m unittest discover -s walle-app/tests`. 134 tests today; some skip on the host
  (no PIL, no ultralytics).
- **Ledger and roadmap:** add a `LEDGER.md` entry before ending any turn that changed files:
  what was verified and how, what wasn't, and any scope drift. Tick the `ROADMAP.md` boxes
  as they are met.
- **Git:** don't commit or push unless I ask, and check the branch right before committing.
  `gh` is not installed; I open PRs in the browser.
