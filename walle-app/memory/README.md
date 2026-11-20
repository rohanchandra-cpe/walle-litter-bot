# WALL-E memory store (Phase 2)

Everything the robot observes, kept so it can answer history questions later (Phase 6's
ROSA agent). SQLite (Python's `sqlite3`, no extra dependency) plus a small vector store for
free-text recall. Plan: [`docs/plans/phase-2-plan.md`](../../docs/plans/phase-2-plan.md).

```
store.py          MemoryStore: the write API and the read queries
schema.sql        the tables (schema version 1)
embed.py          Ollama embedder (nomic-embed-text) and a fake one for tests
route_records.py  route follower summary.json -> records
import_runs.py    rebuild a store from the committed run logs
../config/memory.yaml   file names, Ollama model and URLs, obstacle match radius
../data/memory/   robot.sqlite, sim.sqlite (git-ignored)
```

## Where the data lives

- **One file per source.** `walle-app/data/memory/robot.sqlite` and `sim.sqlite` have the
  same schema. `MemoryStore("robot")` or `MemoryStore("sim")` opens one. A file remembers
  its source and refuses to open as the other one. Sim records can't get into robot answers.
- **The files are git-ignored. The run logs are the record.** `results/route/<run>/`
  (`summary.json`, `layout.json`, photos) is committed, and
  `python3 walle-app/memory/import_runs.py --rebuild` rebuilds `robot.sqlite` from it. Later
  phases must also leave their records in the run folder (as JSON) so a rebuild keeps
  working.
- **The files survive container restarts.** The repository is mounted at `/workspace` in
  both containers. Checked 2026-11-18 by writing from `walle-pilot`, recreating the
  container, and reading the data back.
- **Photos are referenced, not copied.** Paths are stored relative to the repository root
  (`walle-app/results/route/<run>/photos/bin_2_step2.jpg`). They work on the host and in
  the containers, and they last as long as the run folder does.
- **Writers can run at the same time.** Each writer opens its own `MemoryStore`, and every
  write is one short transaction (a writer waits up to `busy_timeout_s` for another
  writer's lock). The journal is the rollback journal, not WAL, because WAL's shared memory
  is unreliable on Docker Desktop bind mounts.

## Schema

Positions are in the park frame (x toward corner 2, y toward corner 4; m and deg). Times
are UTC ISO strings. Every table has `source`.

| Table | One row per | Key fields |
|---|---|---|
| `runs` | run (route run, fake run in a test, later others) | `run_id`, `kind`, `scenario`, `status`, `started_utc`/`ended_utc`, `distance_m`, `end_error_m` (EKF), `measured_error_cm` (tape), `run_dir`, `layout_json` |
| `detections` | object seen in a photo | `label`, `category` (trash / recycling / unknown), `confidence`, `bbox_json`, robot pose, `obj_x`/`obj_y`, `place`, `bin_id`, `bin_type`, `photo`, `model` |
| `bin_checks` | look at one bin | `bin_id`, `bin_type`, pose, `overflow_level` (0–3, NULL = not assessed), `on_top_count`, `litter_count`, `outcome`, `photo`, `method` |
| `obstacles` | distinct obstacle on the route | `side`, `obj_x`/`obj_y`, `label`, `category`, first seen |
| `obstacle_sightings` | each time the robot passed one | `obstacle_id`, `run_id`, `present` (1 seen / 0 gone), `detection_id`, `action` |
| `memory_text` | sentence describing a record (for recall) | `kind`, `ref_id`, `text`, `embedding` (float32), `embed_model` |

`place` says where a detection was seen:
- `route`: on the route;
- `on_bin`: above a bin's rim;
- `around_bin`: on the ground next to a bin;
- `obstacle`: an obstacle on the route.

**What the robot can see at a bin.** The bins are opaque and the camera sits too low to see
inside them. It sees what sticks up **above the rim** and what lies **on the ground around**
the bin. A bin check records:
- the overflow level (0 = nothing above the rim … 3 = heavily overflowing);
- the number of items on top;
- the number of litter items around the bin;
- an `outcome`:
  - `overflowing`: anything above the rim;
  - `litter`: not overflowing, but litter around it;
  - `clean`: neither, so the robot moves on.

A `clean` check is still recorded, which is how "how often is bin N fine" gets answered.
There is no fill level, because the fill below the rim can't be seen. Phase 1 route runs
record `method: photo_only` checks: the photo and the pose, with no assessment yet. Phase 4
fills in the assessment.

**Current obstacles are derived from history.** An obstacle is on the route now if its
latest sighting is `present = 1`. A report within `obstacle_match_radius_m` (0.10 m, same
side) of an obstacle still on the route is the same obstacle seen again. A run that passes
the spot and finds nothing calls `mark_absent()`, which records that the obstacle was
cleared.

**Bin types.** `park.yaml` `bins.types`: bins 1 and 3 are trash, bins 2 and 4 are
recycling. A scenario can override one bin (`type:`). Records copy the bin's type from the
run's layout. Runs from before Phase 2 have no types (NULL).

## Writing

```python
import sys; sys.path.insert(0, "/workspace/walle-app")
from memory.store import MemoryStore

with MemoryStore("sim") as mem:
    run = mem.start_run(kind="route", scenario="mixed", layout=layout, run_dir=run_dir)
    mem.add_bin_check(run, bin_id=4, overflow_level=3, on_top_count=10, litter_count=2,
                      pose=(0.0, 1.0, 135.0), photo=photo_path, method="yolo_world")
    det = mem.add_detection(run, "can", "recycling", place="on_bin", bin_id=4,
                            confidence=0.82, pose=(0.0, 1.0, 135.0), photo=photo_path)
    mem.report_obstacle(run, "plastic_bottle", "recycling", (0.60, 1.00), side=3,
                        detection_id=det, action="passed")
    mem.end_run(run, status="done", distance_m=4.0)
```

The route follower records every run itself, at the end, unless `--no-memory` is given. If
the store fails, the follower prints a warning; the run and its JSON log are unaffected.

## Reading (what the agent will ask)

| Question | Call |
|---|---|
| "What objects have you seen the most?" | `most_seen(limit=5, since=None, place=None)` |
| "Which bins overflow most often?" | `bin_overflow_ranking()`: per bin, checks, times overflowing, mean and max level, times with litter, times clean |
| "Check bin 1" (the last result) | `last_bin_check(1)` |
| "Where are the obstacles right now?" | `current_obstacles()`; `obstacle_history(id)` |
| "Are people recycling correctly?" | `recycling_correctness()`: items above the rim vs the bin's type, overall and per bin, wrong items by label, litter around bins listed separately, and a `basis` sentence stating that limit |
| anything else, in words | `recall("text", k=5)` |
| recent runs | `runs(limit=10)` |

Every read returns plain dicts and lists, ready for JSON.

**Free-text recall.** Each record also gets a sentence in `memory_text`, embedded with
Ollama `nomic-embed-text`. `recall()` ranks the sentences by cosine similarity, in pure
Python (no numpy, so it runs in both containers and on the host). If Ollama can't be
reached, the sentence is kept without an embedding, `recall()` falls back to keyword
matching, and `embed_pending()` adds the embeddings later. The Ollama URLs tried are
`$OLLAMA_URL`, then `host.docker.internal`, then `localhost`.

## Commands

```bash
python3 -m unittest discover -s walle-app/tests             # incl. test_memory.py (no ROS/Ollama)
python3 walle-app/memory/import_runs.py --rebuild           # robot.sqlite from results/route/
python3 walle-app/memory/import_runs.py --source sim --runs walle-sim/data/default/route
```
