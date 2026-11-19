# Session summary: Phase 1, route following (2026-11-16 to 2026-11-18)

Paste this into a new session (or use it as the `/compact` instruction) to carry over what
this session did. The durable record is `LEDGER.md`; this is the short version.

---

You are continuing work on the WALL-E repository (`walle-litter-bot`, branch
`walle-phase-1`). Read `CLAUDE.md`, `ROADMAP.md` and the latest `LEDGER.md` entries first.
What happened in the previous session:

**Branch.** `walle-phase-1` was cut from `implement-core-walle-functionality` (Phase 0 is not
merged to `main`; the user chose to branch from it). Commit `596ba16` holds the first Phase 1
build. Everything after it (firmware closed loop, retuning, new bins, these docs) may still
be uncommitted: check `git status`.

**Starting point.** The user's module 7 and module 8 code (`Ollama/module-7-kinematics-control`,
`Ollama/module-8-sensor-fusion`) drove a 1 m square with an EKF. Module 8's `move_test.py`,
odometry and firmware turned out to be byte-identical to `walle-pilot/`; module 7's
`pico_pid` (firmware wheel-speed PI) became the firmware change below.

**Decisions (ROADMAP decision log, 2026-11-16 and 2026-11-18):**
- Route control on the computer side, on the EKF (`/odometry/filtered`); the follower
  subclasses module 8's `Mover` (imported from `walle-pilot/odometry/move_test.py`, not
  changed).
- Pass: 1 lap, end within 5 cm of the start; every bin wholly in frame at its stop.
- Corners: stop and turn in place to absolute park headings; sides follow the line.
- Bin stops: turn to face the bin's configured center, wait 15 s, take a photo.
- Bins are boxes, wide face toward the square, far enough out that the camera sees the
  whole bin from the corner. Now black plastic bins: 27.7 cm wide, 21.1 cm deep, 29.2 cm tall
  (offset 0.71 m: bin 1 at (−0.502, −0.502)). Camera measured: 10.3 cm high, 12 cm ahead of
  the axle.
- Safety: the user starts runs and stops them with Ctrl+C; per-move timeouts; the Pico's
  500 ms watchdog is kept.
- **Firmware changed (with the user's OK, in place, not a new folder):**
  `walle-pilot/firmware/pico_pilot/drive_control.c` now has closed-loop wheel speed (feed-
  forward + PI, KP 1.0, KI 5.0, filter 0.3, MIN_DUTY 0.25, 50 Hz). The RQ1–RQ3 study ran on
  the old open-loop firmware; the user keeps that original elsewhere.
- Bin centering moves to Phase 3: YOLO will detect the bins as well as trash, and the robot
  will center the detected bin (the Phase 1 follower turns to a fixed heading).

**What was built:**
- `walle-app/route/route_follower.py` + `route_geometry.py` + `config/route.yaml`: line
  following, absolute turns, bin stops with photos (`--camera-url`), coast compensation
  learned from each move's overshoot, up to 2 correction moves, gyro-drift measurement
  (10 s still at the start, plus every bin wait) and zero-velocity update,
  `/walle/route/status` (JSON), and a run log in `walle-app/results/route/<run>/`
  (`trajectory.csv`, `summary.json`, `layout.json`, `photos/`) plus one row per robot run in
  `results/route/robot_runs.csv` (tape measurements asked at the end).
- `walle-app/park.py`: box bins, clearance against full side lines, framing offset from
  `robot.camera`; layout gains `shape`, `size_m`, `yaw_deg`, `view_heading_deg`.
- `walle-sim`: `route-check` harness command (`make sim-route-check`), ground-truth judging
  and a bin-in-frame projection check; motor stiction in the plant (`breakaway_duty`); the
  `pid` firmware model mirrors the new C line for line; profiles `carpet`, `carpet_pid`,
  `plastic_pid` (the current robot: closed loop on corrugated plastic).
- 20 unit tests in `walle-app/tests/`.

**Results.**
- Sim (`plastic_pid`): end 1.1 cm, smooth turns.
- Robot, three good runs (carpet, then corrugated plastic). All ended 0 cm from the start by
  tape (EKF 0.2–0.6 cm). Wheels alone would have ended 23–39 cm off, so the BNO055 heading is
  what holds the route. Measured gyro drift is about 0.001 °/s.
- First runs were jerky. The old open-loop firmware turned at about 3 rad/s for any command;
  the first closed-loop gains (module 7's) stick-slipped at slow speeds. Fix: KP 1.0, and
  the follower never asks for less than about 0.1 m/s per wheel (turns 1.0–1.5 rad/s,
  slowest drive 0.08 m/s).
- Last run: smooth; bins 1 and 2 centered, 3 and 4 off (placement, plus 5.5° at bin 4).

**Gotchas.**
- The Pico looks for the micro-ROS Agent only at boot: after `make up`, reset it if
  `/odometry/filtered` is silent.
- The firmware builds with the VS Code Pico toolchain:
  `~/.pico-sdk/ninja/v1.13.2/ninja -C walle-pilot/firmware/pico_pilot/build`, then copy
  `build/pico_pilot.uf2` to `/Volumes/RP2350` (BOOTSEL).
- The HC-SR04 does not answer (`echo -2`): power or wiring. Phase 5 needs it.
- Stop a headless sim from outside with bracketed `pkill` patterns, and kill leftover
  nodes (`sim_pico_bridge`, `odometry_node`, `ekf_node`) before relaunching with another
  profile.

**Open:**
- Phase 0's last box (the square taped out) is still unticked.
- Bin 4's 5.5° and the 8 correction moves in the last run were not investigated.
- `walle-app/results/` (robot runs) is untracked; decide whether to commit it.
- Next: Phase 2 (memory store) in a new session; start prompt in `docs/handoff/phase-2-prompt.md`.
