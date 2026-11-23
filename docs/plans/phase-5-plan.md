# Phase 5 plan: obstacle detection and avoidance

## Context

The route follower drives the 1 m square and refuses any layout with an obstacle
(`route_follower.py:1122`, "avoiding it is Phase 5"). Phase 5 replaces that refusal. The
robot must notice an object on a side, identify it (Phase 3 detector), record it (Phase 2
store), drive around it, and rejoin the route, or report and stop when it can't.

The sim already places one obstacle per layout on a side at 0.25–0.75 along it
(`park.py build_obstacle`), and models the HC-SR04 as a ±15° lidar cone
(`sim_pico_bridge.py`). The sim's ground-truth detector (`detect/sim_truth.py`) doesn't
project the obstacle yet.

## Decisions (agreed 2026-11-22; go into the ROADMAP decision log)

1. **Sensing = corner look + sonar.**
   - Before each side (the start, and after each corner's `turn`), the camera tilts 15°
     down while the robot stands still and takes one photo.
   - Floor in view: from about 0.28 m ahead of the axle to the far corner. That covers the
     0.25–0.75 band, flat items included.
   - While driving, the HC-SR04 is the stop trigger for anything the photo missed.
   - The camera levels before any motion: the existing `send()` tilt rule, unchanged.
2. **When to look:**
   - one detector call per side, at its corner;
   - one more when the sonar stops the robot for something the corner look didn't find.
   - No looks while driving.
3. **Detour:**
   - **Inside only** (left of the CCW route), as a rectangle of turn-in-place moves and
     straight legs.
   - Lateral offset `d = r_o + half_width (0.086) + margin (0.05) + uncertainty (0.03)`.
   - The forward leg runs until the axle is `r_o + turn_radius (0.14) + margin` past the
     obstacle's center. The end is clamped to the corner, so it rejoins there at worst.
   - Each leg reuses `side_leg` (it follows any a→b line), and each turn reuses `turn_to`.
     So every command goes through `send()` and its guards.
4. **What counts as an obstacle:** anything boxed by the detector inside the side's corridor,
   or confirmed by the sonar. There is no drive-over size: the robot avoids everything
   (default; it's a setting, `obstacle.min_size_m: 0`).
5. **Fallback: report and stop.**
   - When:
     - the sonar triggers on a detour leg;
     - the detour leaves the allowed area;
     - the obstacle is bigger than `max_radius_m`.
   - Then: stop, record the obstacle with `action: "blocked"`, and end the run with status
     `blocked` and the reason.
6. **Recording:**
   - `obj_xy` comes from the camera (ground point of the box's bottom center, pushed back by
     `r_o` along the ray).
   - If there's no usable box (none, or cut off at the bottom of the frame), it comes from
     the sonar: pose + 0.125 m + range + `r_o`.
   - Both estimates are logged.
   - `r_o` comes from the box's bottom-corner ground points (clamped 0.03–0.15 m), or a
     default of 0.10 m when there's no box.
   - Saved per run in `obstacles.json`. `route_records.py` turns it into `report_obstacle`
     calls.
   - At import, `mark_absent` is called for any store obstacle on a side the run completed
     that has no matching report. That keeps it rebuild-safe, with no database access during
     the run.
7. **Sim:**
   - `sim_truth.py` projects `layout["obstacle"]` too (group `obstacle`).
   - `make sim-obstacle-check RUNS=10`, a headless one-command check. For each case it
     relaunches the park world, runs the follower and scores:
     - detected and recorded;
     - position error against truth;
     - collision (ground-truth robot rectangle vs the obstacle's footprint, from
       `trajectory.csv`);
     - lap end within 5 cm.
   - **Target: 9 of 10 runs pass, position within X = 10 cm.**
   - The 10 cases:
     - random seeds, items and poses;
     - the hard cases: at 0.25, at 0.75, a lying bottle, one with `PROFILE=ultrasonic_dead`
       (camera only), and one where the detector misses the obstacle (sonar only).
8. **Guards:**
   - Detour legs are ordinary sides and turns, so the jump and stale-data guards apply as
     they are.
   - Each leg gets its own timeout.
   - The tilt rule doesn't change: the robot only tilts while stopped.
9. **Robot test (user):**
   - 3 runs, changing the setup between them with the rules frozen:
     - the coffee can on side 1 at about 0.5;
     - the milk jug on side 3 at about 0.35;
     - a flat bag on side 2 at about 0.6 (camera-only path).
   - You measure: obstacle center from the corner (tape), any contact, and the end pose.
   - Pass: passes and rejoins with no contact in at least 2 of 3; positions scored with the
     replay script.

## Step 0: HC-SR04 check (robot, before any code)

- With `make launch`, run `ros2 topic echo /pico/wheel_ticks` with a box at 20, 40 and
  80 cm. `echo_us × 343 / 2e6` should track the distance.
- Then try the flat bag and a paper towel at 30 cm, and note what it reports.
- Also measure the sensor's height above the floor (the sim assumes 7 cm).
- If it returns −2, check power and wiring. If it can't be fixed, the plan still works,
  camera only: the `ultrasonic_dead` case covers that.

## Build (sim first)

| File | Change |
|---|---|
| `walle-app/route/obstacle.py` (new, no ROS) | corridor test for detections; `obj_xy`/`r_o` from a box or a sonar range; `plan_detour(a, b, s_o, r_o, robot) -> legs` or a reason it can't; sonar filter (k=3 consecutive pings ≤ `trigger_m` 0.35, range < distance left to the corner, not an already-known obstacle) |
| `walle-app/route/route_geometry.py` | `build_plan` adds a `look_side` step before each `drive` |
| `walle-app/route/route_follower.py` | `look_side` (tilt down, photo, detect, level); `side_leg` takes a `stop_if` sonar hook and returns "blocked"; `drive_side` runs the known-obstacle plan (drive to the detour start → legs → rest of the side); the fallback; `obstacles.json`; summary fields; the refusal in `main()` removed |
| `walle-app/config/route.yaml` | `obstacle:` section (trigger_m, confirm_pings, uncertainty_m, default/max radius, tilt_deg −15, min_size_m, enabled) |
| `walle-app/detect/sim_truth.py` | project the obstacle |
| `walle-app/memory/route_records.py` | `obstacles.json` → `report_obstacle` / `mark_absent` |
| `walle-app/detect/replay_obstacles.py` (new) | score a run's `obstacles.json` against truth (sim layout) or `--truth "side,at"` (robot) |
| `walle-sim/harness/sim_trials.py` + `Makefile` | `obstacle-check`: the case list, relaunch per case (bracketed `pkill`), collision and position scoring, PASS/FAIL table |
| `walle-app/tests/test_obstacle.py` (new) | detour geometry (clearances, corner clamp, too big), corridor filter, sonar filter, `obj_xy`, import with `mark_absent` |

Reused: `geometry.Camera.ground_point`, `robot_to_park`, `Tilt.set`/`level`,
`analyze`/`look`, `store.report_obstacle`/`mark_absent`, `rg.side_progress`,
`park.Geometry.footprint`.

Frozen files aren't touched. There's no firmware change.

## Verification

- `python3 -m unittest discover -s walle-app/tests` on the host and in both containers.
- Sim: `make sim-route-check` and `make sim-bin-check` still pass with no obstacle (no
  regressions).
- `make sim-obstacle-check RUNS=10` → 9 or more of 10 PASS, position error ≤ 10 cm, zero
  collisions.
- `import_runs.py --rebuild` reproduces the same `current_obstacles()`.
- Robot: Step 0, then the 3 test runs above (user), scored with `replay_obstacles.py`.
- Afterwards: ledger entry, ROADMAP boxes and decisions, and this plan copied to
  `docs/plans/phase-5-plan.md`.

## As built (2026-11-22)

- **Sonar trigger:** `confirm_pings` 3, `trigger_m` 0.35 m. It only counts a range nearer
  than the end of the current leg, so nothing beyond the corner, or beyond a planned
  detour start, stops the robot.
- **Detour depth allowance (added after the first sim batch):** a photo shows an object's
  width, not its depth along the view. A bottle lying along the path looked 6 cm wide but
  was 20 cm deep. So the turn back waits until `assumed_depth_m` 0.25 m past the near face
  is cleared; near the far corner this may shrink to the measured size.
- **A sonar hit near a known obstacle** refines its position (it's nearer than the camera
  put it) and the detour is replanned. More than `max_hits_per_side` (6) stops on one side
  → blocked.
- **The follower is never told where an obstacle is.** In Gazebo the world's obstacle
  stays in the truth layout (the ground-truth detector, and `obstacles.json`'s truth).
- **The harness** launches each case in its own process group and refuses to start while
  any sim node is left over. Without that, twelve stale `odometry_node.py` copies
  corrupted the first batch.
- **Sim result:** 9/10 pass. The miss is `camera_miss`: with the sonar alone the size is
  unknown (default radius 0.10 m), so an object 0.26 m from the far corner left "no room to
  turn after it", and the robot stopped as `blocked`, safely.
