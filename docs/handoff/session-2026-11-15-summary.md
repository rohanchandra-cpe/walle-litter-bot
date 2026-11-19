# Session summary: planning docs + Phase 0 (2026-11-15 to 2026-11-16)

Paste this into a new session (or use it as the `/compact` instruction) to carry over what
this session did. The durable record is `LEDGER.md`; this is the short version.

---

You are continuing work on the WALL-E repository
(`walle-litter-bot`, branch `implement-core-walle-functionality`). Read `CLAUDE.md`, `ROADMAP.md` and the
latest `LEDGER.md` entries first. What happened in the previous session:

**Background.** The research study (RQ1 timed vs encoder-targeted drive, RQ2 wheel-only vs
EKF heading, RQ3 HC-SR04 vs monocular depth) is done: robot code in `walle-pilot/`, Gazebo
test worlds in `walle-sim/` (merged to `main` in PR #5). The study files are frozen
(`walle-pilot/config/pilot.yaml`, `walle-pilot/pilot/`, `walle-pilot/data/schedule/`, logs).

**Tooling decision.** The user considered GSD and Serena MCP and chose neither. Instead:
`CLAUDE.md` (agent guide), `ROADMAP.md` (phases 0–7 with "done when" boxes and open
decisions), plan mode at the start of every phase, sim-first checks, and an append-only
`LEDGER.md` that records work done and scope drift. Approved plans go to
`docs/plans/phase-N-plan.md`. One session per phase; start prompts go in `docs/handoff/`.

**Commits on `implement-core-walle-functionality`** (pushed, not merged to `main`):
- `07b9507` CLAUDE.md, ROADMAP.md, LEDGER.md
- `e1b61c2` Phase 0 (committed by the user)

**Phase 0 built (sim done):**
- `walle-app/config/park.yaml`: 1 m square, CCW route, corner 1 = origin and start facing
  corner 2; robot footprint (half width 0.086 m, turn radius 0.14 m, margin 0.05 m); bins
  as cylinders (placeholder r 0.10 m, h 0.20 m) placed outside each corner on the diagonal
  with an auto offset (0.34 m); a sim-only item catalog (bottle, can → recycling; cup,
  wrapper → trash). The catalog must not limit detection: Phase 3 uses open-vocabulary
  YOLO-World.
- `walle-app/config/scenarios/` `default`, `mixed`, `obstacle_only`: per-bin state `ok` /
  `overflowing` (level 1–3) + `litter` 0–6; at most one obstacle per lap, side 1–4, 0.25–0.75
  along it; seeded per-component random streams.
- `walle-app/park.py`: `load_park(scenario, bins=, obstacle=, seed=)` → full layout
  (waypoints, bins, obstacle) with clearance checks; 10 unit tests in `walle-app/tests/`.
- `walle-sim/worlds/park_world.py` generates `worlds/generated/park.sdf` at launch;
  `make sim-launch WORLD=park SCENARIO=mixed BINS=2 OBSTACLE=none SEED=n GUI=false`;
  `make sim-park-check LOOK_AT="bin_2 obstacle"` compares against ground truth (1 cm) and
  saves photos. Verified PASS for `mixed` and `mixed BINS=2 OBSTACLE=none`.
- Gotcha: stop headless sims with bracketed `pkill -f "[i]gn gazebo"` inside the container.

**Real-world facts from the user (2026-11-16, may change):**
- Interim bins: Trader Joe's paper bags, 30 × 18 × 36 cm (L × W × H). Plastic bins from
  Target later. Not yet in `park.yaml` (it models cylinders).
- Square not taped out yet. Testing on carpet (cardboard later).
- The robot still runs on the current firmware.

**Open:** Phase 0's last box (real park measured and entered). Next: Phase 1 (route
following) in a new session, prompt in `docs/handoff/phase-1-prompt.md`.
