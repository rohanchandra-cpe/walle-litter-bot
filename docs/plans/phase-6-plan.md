# Phase 6 plan: the WALL-E ROSA agent

## Context

Phases 1–5 built the capabilities: the route follower (a lap with corner looks, sonar,
detours, bin stops and bin checks), the detector, and the memory store with its read
queries. Each one is run by hand from a CLI. Phase 6 adds one ROSA agent that runs them
from plain-language prompts and answers history questions from the store. It's done when
the six acceptance prompts give a correct action or answer in sim and on the robot, and
the history answers match the store.

Two user choices (2026-11-23) add motion work:
- **"Check Bin N" drives a partial route:** to that bin's corner by the shorter way, check
  it, and come back the same way. It's new motion code (clockwise legs), so it goes sim
  first.
- **A look-only tool, `look_ahead`:** one down-tilted photo from where the robot stands. It
  works in Gazebo too.

The LLM gets chosen by a model test: `qwen3.5:4b` against one larger local model.

## Decisions (go in the ROADMAP decision log when approved)

1. **LLM.** A stubbed model test (step 1) runs the six prompts on `qwen3.5:4b` and on one
   larger local model. I check the Ollama tag and its size and ask before pulling it. The
   smallest model that meets the target wins. Target over 6 prompts × 5 repeats: right tool
   and arguments ≥ 28/30; the fact check passes ≥ 27/30; record median and max latency.
   Nothing is hosted.
2. **Approval:** one `[APPROVAL NEEDED] … Allow? [y/N]` per motion tool call, which covers
   the whole run. The prompt says what it will do, how long it takes, and that Ctrl+C stops
   the robot. The Pico's 500 ms watchdog and all the follower guards stay.
   - `--auto-approve` exists only for the sim check, and the agent refuses it unless
     `/sim/ground_truth` is being published (Gazebo).
   - **ROSA's ~50 built-in tools are blacklisted**, so the model has only WALL-E's tools.
     That includes `ros2_topic_pub`, which could publish to `/pico/cmd_vel` around the
     follower's guards.
3. **How the agent runs the follower:** as a subprocess of its CLI, with `--no-measure`. The
   follower itself barely changes, so its guards stay as tested.
   - The tool blocks until the run ends. ROSA's AgentExecutor has no time limit, and
     `max_iterations` (12) counts tool calls, not time.
   - Progress: the follower's output goes to the terminal live, prefixed `[route]`.
     `/walle/route/status` stays as it is.
   - Stopping: Ctrl+C reaches both processes (same process group). The follower stops the
     robot and writes its files; the tool waits up to 10 s, kills it if it hangs, and
     reports "stopped". The tool never sends motion itself.
   - It returns a short digest built from `summary.json`, `bin_checks.json` and
     `obstacles.json`.
4. **"Check Bin N" = a partial route** (`--trip-bin N`):
   - **Bin 1** (the start corner): face bin 1, check it, turn back. No driving.
   - **Bin 2:** side 1 forward, check, side 1 back.
   - **Bin 3:** sides 1–2 forward and back.
   - **Bin 4:** side 4 driven clockwise, corner 1 → 4, and back.
   - Obstacle looks and detours run on every leg. On a clockwise leg the detour goes inside
     the square, which is to the robot's right.
   - Records keep the counter-clockwise side numbers.
5. **Path and obstacles:**
   - Prompt 1 runs a full lap with obstacles on.
   - `look_ahead` takes one photo without driving (`--look-only`: the follower's
     `look_side` step on its own, so the tilt, detector, sim truth detector and
     `obstacles.json` are all reused). It refuses while a run is going.
   - `report_obstacles` is read-only: obstacles currently in the store, plus the last run's
     `obstacles.json`, with when each was last seen. Prompt 6 is answered from it, with the
     age of that information, and no motion.
6. **Grounding:**
   - History tools call the store queries only. Each result names the store (`robot` or
     `sim`, its path, its run count) and carries a `limits` line: recycling is judged only
     from items on top of a bin; labels can be wrong; there's no fill level, only the
     overflow level and the item count.
   - The store is the sim one when the agent sees `/sim/ground_truth`, otherwise the robot
     one; `--store` overrides that.
   - Critical instructions: report only what the tools returned, name the store, state the
     limits, never estimate.
7. **Sim acceptance check:** `make sim-agent-check` is headless and starts its own worlds,
   like `obstacle_trials`.
   - **History prompts 3–6** run against a **fixed fixture store** (deterministic, built
     into a temp file) so their expected answers are known. 3 repeats each must all pass:
     12/12.
   - **Action prompts 1–2** run in a launched world. The pass needs all of:
     - the right tool and arguments;
     - the run is `done`;
     - the obstacle is found and passed, and the bin outcome is right against the world
       (reusing `sim_trials` scoring);
     - the answer states facts that match the run files.
   - **One `look_ahead` prompt** with an obstacle on side 1.
   - **The fact check:** the expected facts appear in the answer, and every number in the
     answer appears in that turn's tool output (no invented numbers).
   - **Trip checks:** `make sim-trip-check BIN=2|3|4` must meet the route-check bars: end
     ≤ 5 cm from the start, and the bin is judged right. Clockwise obstacle cases are added
     to `sim-obstacle-check`: no collisions, and every obstacle is passed or reported
     `blocked`. The existing 10 cases must still reach 9/10.
8. **Robot acceptance (you run it):**
   - **Setup:** bin 1 overflowing (~4 items on top), a can on side 1, the rest as they are.
   - **Prompts:** all six in order, plus "Check Bin 4" (clockwise legs) and one
     "Is anything in front of you?".
   - `agent/facts.py --store robot` prints what the history answers should say, so you can
     compare.
   - **You check:** the right action; no contact; answers match the facts printout and what
     is really on the track.

## Build order (sim first at every step)

1. **Agent skeleton and history tools (no motion), then the model test.**
   - `walle-app/agent/walle_agent.py`: built like `range_agent.py` (ROSA, `ChatOllama`,
     `temperature=0`, `num_ctx=16384`, `accumulate_chat_history=False`, an
     `input("You> ")` loop), with the built-ins blacklisted.
   - `walle-app/agent/walle_tools.py`, read tools: `most_seen_objects(place)`,
     `bin_overflow_history()`, `recycling_report()`, `report_obstacles()`, `last_bin_check(bin)`,
     `recent_runs()`, `recall_memory(question)`. They wrap `MemoryStore`
     (`memory/store.py`) and use its `path=` argument for the fixture store.
   - Every tool appends to a per-turn call log, which the check uses.
   - `walle-app/agent/fixture_store.py`: a known history (most-seen label, bin ranking, wrong
     items, a current obstacle).
   - `walle-app/agent/bench.py`: motion tools stubbed with digests from real run folders;
     6 prompts × 5 repeats per model; scores and latencies to `results/agent/bench/`.
   - `walle-app/config/agent.yaml`: model, URL, store, timeouts, `max_iterations`, approval
     text.
2. **`look_ahead`:** the follower's `--look-only` mode, a plan with one `look_side` step from
   where the robot stands. Positions come relative to the robot, and also on side 1 when it
   is at the start pose. Recorded as run kind `look` in `memory/route_records.py`. Sim
   check, then the tool.
3. **`run_route(check_bins)`:** `walle-app/agent/runner.py` handles the subprocess, the
   output pass-through, SIGINT handling, and reading the run files into the digest. In sim
   the layout comes from the world (`--layout-json` / `--truth-layout` from
   `walle-sim/worlds/generated/park_info.yaml`, as `sim_trials.py` does). On the robot it is
   `--scenario default --camera-url …` from `agent.yaml`.
4. **Partial route, `--trip-bin N`:**
   - `route_geometry.build_trip()`: the shorter way out, the bin stop, a 180° turn, the same
     way back, then a turn to the start heading.
   - `drive` / `look_side` steps carry a direction.
   - `route/obstacle.py`: an `inside` sign (+1 counter-clockwise, −1 clockwise) for the
     detour and clearance planners. Counter-clockwise behavior must stay identical (the
     existing tests and the 10 obstacle cases).
   - The follower keeps counter-clockwise side numbers in `sides_done` and obstacle
     records, and the memory kind is `trip`.
   - Unit tests for `build_trip` and for mirrored detours. Then `make sim-trip-check`, the
     clockwise obstacle cases, and the `check_bin(bin)` tool: the trip's bin result plus the
     previous stored check, for comparison.
5. **`make sim-agent-check`** (`walle-sim/harness/agent_trials.py`), all of decision 7.
6. **Robot acceptance**, decision 8.
7. **Ledger entries** each turn; ROADMAP boxes and decision log; README sections in
   `walle-app/README.md` and `CLAUDE.md` (layout and commands).

## Critical files

- **New:** `walle-app/agent/` (`walle_agent.py`, `walle_tools.py`, `runner.py`,
  `fixture_store.py`, `bench.py`, `facts.py`), `walle-app/config/agent.yaml`,
  `walle-sim/harness/agent_trials.py`, `walle-app/tests/test_agent.py`, `test_trip.py`.
- **Changed:** `walle-app/route/route_follower.py` (`--trip-bin`, `--look-only`),
  `route_geometry.py` (`build_trip`), `obstacle.py` (the inside sign),
  `memory/route_records.py` (run kinds), `walle-sim/harness/obstacle_trials.py` (clockwise
  cases), `sim_trials.py` (`trip-check`), the `Makefile`.
- **Reused, not changed:** `walle-pilot/odometry/odom_tools.py` (`stop_robot`, the
  approval wording), `range_agent.py` (the pattern), `memory/store.py` (its queries).
- **Frozen, untouched:** `walle-pilot/config/pilot.yaml`, `walle-pilot/pilot/`, the
  schedule and logs.

## Verification

- `python3 -m unittest discover -s walle-app/tests` on the host and in both containers.
- Model test results, and a choice recorded with the numbers.
- `make sim-trip-check BIN=2/3/4`, `make sim-obstacle-check` (old 10 cases ≥ 9/10, plus the
  clockwise cases), `make sim-agent-check` (all targets in decision 7).
- `agent/facts.py --store robot` checked against direct store queries.
- The robot runs (decision 8), done by you. Nothing involving the robot runs while you
  drive, and Ollama-heavy tests don't run then either.
- After approval this plan is copied to `docs/plans/phase-6-plan.md`. Nothing is committed
  without asking.
