# Session summary: Phase 2, memory store (2026-11-18 to 2026-11-19)

Paste this into a new session (or use it as the `/compact` instruction) to carry over what
this session did. The durable record is `LEDGER.md`; this is the short version.

---

You are continuing work on the WALL-E repository (`walle-litter-bot`). Read `CLAUDE.md`,
`ROADMAP.md` and the latest `LEDGER.md` entries first. What happened in the previous
session:

**Branch.** Phase 1 had been merged to `main` (PR #7), so `walle-phase-2` was cut from
`origin/main`. All the work is in commit `ad5da3e`, merged to `main` in **PR #8**
(`2a42319`). Phases 0, 1 and 2 are all ✅ in `ROADMAP.md`.

One part of the working tree is still uncommitted: the user's own move of the old handoff
docs into `docs/handoff/phase0-to-phase1/` and `docs/handoff/phase1-to-phase-2/`. Git sees
it as 4 deleted files plus new folders. Also uncommitted: this folder,
`docs/handoff/phase2-to-phase-3/`, plus `.DS_Store` and `.vscode/`, which should not be
committed. Ask the user before committing any of these.

**Decisions** (ROADMAP decision log, 2026-11-18 and 2026-11-19):
- **Storage:** SQLite (stdlib) plus a vector store, because the user asked for one. Each
  record gets a sentence in `memory_text`, embedded with Ollama `nomic-embed-text` on the
  host. Search is cosine in pure Python. Without Ollama, recall falls back to keywords, and
  `embed_pending()` adds the embeddings later.
- **Files:** one per source, `walle-app/data/memory/robot.sqlite` and `sim.sqlite`, both
  git-ignored. `walle-app/memory/import_runs.py --rebuild` rebuilds them from the committed
  run logs in `walle-app/results/route/`, which stay the record.
- **Photos** are referenced by repo-relative path, not copied.
- **Journal:** rollback journal, not WAL, because of Docker Desktop bind mounts.
- **Bin types:** `park.yaml` `bins.types`: bins 1 and 3 trash, bins 2 and 4 recycling. A
  scenario can override one bin with `type:`.
- **What the robot can see at a bin.** The user pointed out that the bins are opaque and the
  camera is too low to see inside them. The robot sees only what sticks up above the rim and
  the litter on the ground around the bin. So a bin check records:
  - `overflow_level` (0 = nothing above the rim … 3);
  - `on_top_count`;
  - `litter_count`;
  - `outcome`: `overflowing`, `litter` (not overflowing, but litter around it), or `clean`
    (the robot moves on, and the check is still recorded).

  There is no fill level. Recycling correctness compares only items above the rim
  (`place='on_bin'`) with the bin's type. Litter around a bin (`around_bin`) is listed but
  not judged.
- **Current obstacles** are derived from sighting history. A report within 0.10 m on the
  same side is the same obstacle. `mark_absent` records that an obstacle has cleared.
- **Write path:** a Python API (`MemoryStore`), with no ROS node. The route follower writes
  each run and one `photo_only` bin check per bin stop.
- **Turn corrections (2026-11-19):** misses up to 20° are now corrected with **timed
  pulses**. A pulse is one turn command, then a stop after a set time. Its gain is learned,
  starting at 110°/s. A pulse is skipped if even the shortest one would overshoot by more
  than the miss. Up to 3 corrections per turn.
- **Photo retries:** up to 3 tries per bin stop, 3 s apart, within the bin wait.

**What was built:**
- `walle-app/memory/`:
  - `store.py`: the write API, and the queries `most_seen`, `bin_overflow_ranking`,
    `last_bin_check`, `current_obstacles`, `obstacle_history`, `recycling_correctness`,
    `recall`, `runs`, `counts`;
  - `schema.sql`, `embed.py`, `route_records.py`, `import_runs.py`;
  - `README.md`, which writes out the schema.
- `walle-app/config/memory.yaml` holds the store's settings.
- `walle-app/park.py` puts a `type` on each bin.
- `walle-app/route/route_follower.py`:
  - each event gets a `t_utc` time;
  - it records the run in the memory store at the end (`--no-memory` skips this);
  - `pulse()` and `photo_with_retries()`;
  - `summary.json` gains `pulses`, `photo_tries` and `photo_error`.
- `route_geometry.py` gains `plan_pulse()` and `learn_pulse_gain()`.
- `make sim-route-check COAST=0.35` (`route-check --coast-time`) forces turn misses so the
  pulses run.
- Tests: 37 in total in `walle-app/tests/`, 17 of them new.

**Results:**
- **Tests:** pass on the host and in both containers.
- **Container restart:** a recreated container read back the same data.
- **Gazebo (`plastic_pid`):** route checks pass, ending 0.1–1.4 cm from the start. With
  turns forced to miss, each pulse cut the miss to 0–1.2°. The old slow-turn corrections
  left one bin 5.2° off after 10 correction moves.
- **Robot, run `20261119_043639`** (weak batteries):
  - Bins 2 and 4 got no photo (timeout, then connection refused).
  - Bins 2 and 4 were 5.3° and −9.4° off. Correction turns asked for 4–6° stuck and then
    lurched 8–13°.
  - The user recharged both supplies; power was the cause of the missing photos.
- **Robot, run `20261120_014029`** (pulses, charged batteries):
  - Ended 0 cm from the start by tape.
  - 3 pulses, each landing within 0.6°; learned gain 118°/s.
  - Bin heading errors were within 2.5°, and all 4 photos arrived on the first try.
  - The run recorded itself in `robot.sqlite`, which now holds 5 runs and 20 bin checks.
  - Bins 3 and 1 were off-center in their photos because of placement: the robot faced them
    within 1°, but they were much nearer than 0.71 m.

**Gotchas:**
- **Never recreate only the `walle-pilot` container.** The micro-ROS Agent uses its network
  (`network_mode: "service:pilot"`), so it is cut off and the Pico's topics vanish. Use
  `make down && make up`, or recreate the agent too
  (`docker compose up -d --force-recreate --no-deps micro-ros-agent`).
- **Power:** the robot has a 9 V supply for the motors and a separate 5 V supply for the
  Pico, the HC-SR04 and the ESP32-CAM. A weak 5 V supply resets the ESP32-CAM mid-run. It
  may also explain why the HC-SR04 never answers (unchecked).
- **Python on the host is 3.13; the containers run 3.10.** Test in both.
- **`gh` is not installed on the host.** The user opens PRs in the browser.
- **No sim is running:** the `walle-sim` container was restarted on 2026-11-19. Start a park
  world with `make sim-launch WORLD=park ... GUI=false`.

**Open:**
- Old robot runs (before 2026-11-18 evening) have no bin types in the store (NULL).
- Concurrent writers were tested from one process only.
- `import_runs.py` reads only route summaries. Phase 3 must leave its detections in the run
  folder and extend the importer, so a rebuild keeps them.
- Next: Phase 3 in a new session; start prompt in
  `docs/handoff/phase2-to-phase-3/phase-3-prompt.md`.
