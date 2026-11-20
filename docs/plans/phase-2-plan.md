# Phase 2 plan: memory store

Approved by the user 2026-11-18 (after two changes: opaque bins are judged only by what
shows above the rim, and a not-overflowing bin is checked for litter, then left).

## Context

WALL-E must answer history questions in Phase 6 ("what have you seen most?", "which bins
overflow most often?", "where are the obstacles now?", "are people recycling correctly?").
Phases 3–5 will produce detections, bin checks and obstacles; Phase 2 builds the store they
write to, so records exist from the moment those capabilities do. Today the only producer is
the Phase 1 route follower, which writes `walle-app/results/route/<run_id>/summary.json`
(committed). Branch: `walle-phase-2` from `origin/main` (Phase 1 merged in PR #7).

## Decisions (user answers 2026-11-18; go into the ROADMAP decision log)

1. **Storage:** SQLite (stdlib `sqlite3`), plain tables, **plus a vector store** for free-text
   recall: a `memory_text` table holding one sentence per record and its embedding (float32
   BLOB), from Ollama `nomic-embed-text` (already on the host). Search is brute-force cosine
   in pure Python (thousands of rows at most; no numpy/sqlite-vec dependency, so it runs in
   both containers and on the host). If Ollama is unreachable, the text is stored with no
   embedding (embedded later by `embed_pending()`) and `recall()` falls back to keyword search.
2. **Sim vs robot:** separate files, same schema: `walle-app/data/memory/robot.sqlite`,
   `sim.sqlite`. The store is opened by source; every row also carries `source`.
3. **Git:** both files git-ignored. `memory/import_runs.py` rebuilds a DB from the committed
   `results/route/<run>/summary.json` + `layout.json`, so the JSON logs stay the record.
   (Later phases must also leave their records in the run folder for this to stay true; noted
   in the README.)
4. **Bin types:** `park.yaml` gets `bins.types: {1: trash, 2: recycling, 3: trash, 4:
   recycling}`; a scenario may override per bin (`type:`). `park.py` puts `type` on each bin
   in the layout; existing tests still pass.
5. **Recycling correctness:** the robot is too short to see into the (opaque) bins, so it
   only sees items **on top of** a bin (overflowing above the rim) and items on the ground
   **around** it. An item seen on top of a bin (detection with `bin_id` and `where='on_bin'`)
   counts as put in that bin: correct when its category matches the bin's type. Score =
   correct / classified on-top items, per bin and overall, with counts of wrong items by
   label. The answer says it is based only on what was visible above the rim (a bin that is
   not overflowing gives no evidence). Litter on the ground around a bin is reported
   separately (by category, with the bin's type), not scored as right or wrong. In sim the
   overflow `mound` items are exactly the on-top items, so they are the ground truth.
6. **Current state:** derived from history, not a mutable table. Obstacles have sightings;
   an obstacle is "current" if its latest sighting says `present`. A later run that passes
   the spot and doesn't see it adds a `present=0` sighting (that is how "cleared" is
   recorded). Obstacle matching: same side, within `match_radius_m` (config, 0.10 m).
7. **Photos:** referenced by path relative to the repo root (they already live in the run
   folder), not copied; kept as long as the run folder is.
8. **Write path:** a Python API (`MemoryStore`) that nodes and ROSA tools import. No
   `/memory/ask` ROS node in Phase 2 (Phase 6 tools can import the API directly; add the node
   then if needed). Phase 2 writers: the route follower and the fake-run test.
9. **File safety:** one short transaction per write, `busy_timeout` 5 s, rollback journal
   (`journal_mode=DELETE`, not WAL: WAL's shared-memory file is unreliable on Docker Desktop
   bind mounts), foreign keys on, `PRAGMA user_version` for the schema version.

## Schema (`walle-app/memory/schema.sql`, also written out in `walle-app/memory/README.md`)

Positions are in the park frame (m, deg). Times are UTC ISO strings.

- `runs`: `run_id` PK, `source`, `kind` (`route` / `fake` / later `check_bin`…), `scenario`,
  `started_utc`, `ended_utc`, `status`, `reason`, `laps`, `distance_m`, `end_error_m`
  (fused EKF), `measured_error_cm` (tape, robot), `run_dir`, `layout_json`, `notes`.
- `detections`: `id`, `run_id` FK, `t_utc`, robot pose `x,y,heading_deg`, `label`,
  `category` (trash / recycling / unknown), `confidence`, `bbox_json`, object position
  `obj_x,obj_y` (nullable), `where` (`route` / `on_bin` / `around_bin` / `obstacle`),
  `bin_id` (nullable), `photo`, `model`.
- `bin_checks`: `id`, `run_id` FK, `t_utc`, `bin_id`, `bin_type`, pose, `overflow_level`
  (0 = nothing above the rim … 3 = heavily overflowing, nullable until Phase 4; no fill
  fraction, since the fill below the rim can't be seen), `on_top_count` (nullable),
  `litter_count` (nullable), `outcome` (`overflowing` / `litter` / `clean` / null until
  Phase 4), `photo`, `method` (`photo_only` in Phase 2; Phase 4 sets its own). A bin is
  often not overflowing: then the check looks for litter around it, and with no litter the
  robot moves on. That `clean` check is still recorded (overflow 0, litter 0), so "how often
  is bin N fine" can be answered.
- `obstacles`: `id`, `source`, `first_seen_utc`, `first_run_id`, `side`, `obj_x,obj_y`,
  `label`, `category`.
- `obstacle_sightings`: `id`, `obstacle_id` FK, `run_id` FK, `t_utc`, `present` (0/1),
  pose, `detection_id` (nullable FK), `action` (`passed` / `stopped` / null).
- `memory_text`: `id`, `kind`, `ref_id`, `run_id`, `t_utc`, `text`, `embedding` BLOB
  (nullable), `embed_model`.

## Files

- `walle-app/memory/store.py`: `MemoryStore(source, path=None, embedder=...)`.
  Writes: `start_run`, `end_run`, `add_detection`, `add_bin_check`, `report_obstacle`
  (matches or creates, adds a present sighting), `clear_obstacle` / `mark_absent`.
  Reads (the agent's needs): `most_seen(limit, since)`, `bin_overflow_ranking()` (per bin:
  checks, times overflowing, mean level, times with litter, times clean),
  `current_obstacles()`, `recycling_correctness()`, `runs(limit)`, `last_bin_check(bin_id)`,
  `recall(text, k)`. Each read returns plain dicts (JSON-ready for ROSA tools).
- `walle-app/memory/embed.py`: Ollama embedder (`OLLAMA_URL`, model from config, timeout),
  plus a deterministic fake embedder for tests.
- `walle-app/memory/import_runs.py`: CLI to rebuild a DB from `results/route/*/`.
- `walle-app/memory/schema.sql`, `walle-app/memory/README.md` (schema, how to query).
- `walle-app/config/memory.yaml`: db folder, file names per source, embed model and URL,
  `match_radius_m`, timeouts.
- `walle-app/config/park.yaml` + `walle-app/park.py`: bin `type` (decision 4).
- `walle-app/route/route_follower.py`: after the summary is written, record the run and one
  `bin_check` per bin stop (`method: photo_only`, photo path, pose; fill/overflow/litter
  null). Wrapped so a store failure prints a warning and never affects driving or the JSON
  log. Flag `--no-memory`. Reuses the existing `summary` dict and `events`.
- `walle-app/tests/test_memory.py`: see Verification.
- `.gitignore`: `walle-app/data/memory/*.sqlite*`.
- Docs: `walle-app/README.md`, `CLAUDE.md` (layout + commands), `ROADMAP.md` (ticks,
  decisions), `LEDGER.md` entry, plan copied to `docs/plans/phase-2-plan.md`.

Not touched: anything under `walle-pilot/` (frozen study files, vision node).

## Verification

1. `python3 -m unittest discover -s walle-app/tests` (host and container): existing 20 tests
   plus new ones that, on a temp DB with the fake embedder:
   - record two fake runs built from scenario ground truth (`mixed`: bin 2 level 1, bin 4
     level 3; bin 1 clean; bin 3 not overflowing but with litter; mound items become `on_bin`
     detections, litter becomes `around_bin`; the obstacle bottle on side 3), then check
     `most_seen`, `bin_overflow_ranking` (bin 4 first), `recycling_correctness` (expected
     ratio computed from the mound items vs the bin types), `current_obstacles` (present
     after run 1, gone after run 2 marks it absent);
   - sim and robot stores stay apart; reopening the file keeps the data; `recall` finds the
     overflow record; the Ollama-down fallback stores text with no embedding.
2. Container restart: `make up`, write a fake run into `robot.sqlite` from the container,
   `make down && make up`, read it back (I run this).
3. Sim: `make sim-route-check` (`PROFILE=plastic_pid`) writes the run and 4 bin checks to
   `sim.sqlite`; check by query.
4. `import_runs.py` rebuilds `robot.sqlite` from the 3 committed robot runs; check by query.
5. Robot: you run the follower once; it writes to `robot.sqlite`.
6. Real Ollama embeddings: one `recall` call against `nomic-embed-text` on the host.
