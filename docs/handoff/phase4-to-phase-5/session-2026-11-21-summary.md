# Session summary: Phase 4, servo and bin monitoring (2026-11-21 to 2026-11-22)

Paste this into a new session (or use it as the `/compact` instruction) to carry over what
this session did. The durable record is `LEDGER.md`; this is the short version.

---

You are continuing work on the WALL-E repository (`walle-litter-bot`). Read `CLAUDE.md`,
`ROADMAP.md` and the latest `LEDGER.md` entries first. What happened in the previous
session:

**Branch.**
- **Phase 3:** merged to `main` in PRs #9 and #10 (`ed7b9b6`). That includes the other
  session's commit `177ec2b` (daylight shots, light-gated zoom, the accuracy target moved
  into Phase 4).
- **Phase 4:** `walle-phase-4` was cut from `walle-phase-3` at `177ec2b`. All of Phase 4 is
  in commit **`4eee476`**, pushed. The user opened a PR for it; it was **not merged** when
  this was written.
- **Phase 4 is ✅ closed with recorded limits** (see Results).

**A commit mix-up, fixed.** The user had switched the checkout to `walle-phase-3` (for the
Phase 3 PR) 3 minutes before the Phase 4 commit, so the commit first landed there. It was
moved:
- `walle-phase-4` got `4eee476`;
- `walle-phase-3` went back to `177ec2b`;
- the checkout was later pointed at `walle-phase-4` without touching any file, after
  checking that all 104 files matched the commit byte for byte.

**Check `git status -sb` right before every commit.**

**The handoff is a second commit on `walle-phase-4`,** after `4eee476`. It holds:
- the ledger entry "Commit on the wrong branch, fixed; Phase 4 → 5 handoff" (the mix-up
  above, and the HC-SR04 note below);
- this folder;
- `docs/plans/phase-4-initial-plan.md`: the plan exactly as approved, before the "as built"
  notes in `phase-4-plan.md`.

**Still uncommitted, on purpose:**
- the user's move of the old handoff docs into `docs/handoff/phase*-to-phase*/`;
- `walle-app/data/photos/_props.jpg`;
- three copies of the 340 MB CLIP weights (`weights/`, `walle-app/weights/`,
  `walle-pilot/weights/`; never commit them), `.DS_Store` and `.vscode/`.

Ask the user before committing any of these.

**Decisions** (ROADMAP decision log, 2026-11-21 and 2026-11-22):
- **Servo command path:**
  - a 4th value on `/pico/imu/std` is the servo pulse in µs (≤ 0 or missing keeps it);
  - the pulse being output comes back as a 5th value of `/pico/wheel_ticks`;
  - no micro-ROS rebuild; 3-value senders and 4-value readers are unaffected.
- **Servo hardware:**
  - a FITEC FS5106B on GP22, 50 Hz;
  - powered from a 5.7 V buck off the 9 V battery (the motors stay on 9 V), ground shared;
    it is a 4.8–6 V servo, **never 9 V directly**;
  - the firmware clamps to 600–2400 µs and slews 1000 µs/s; degrees live in
    `walle-app/config/servo.yaml`, so recalibrating needs no reflash.
- **Calibration (robot):**
  - level 1575 µs, 10 µs/deg, a **shorter pulse tilts up**;
  - safe range 1000–1750 µs, so limits −15° to +55°;
  - the lens is 1 cm ahead of and 4 cm above the tilt axis (`lens_from_pivot_m`).
  - Short moves read 4–5 µs/deg (reading error on small angles); the 300 µs move settled it.
- **Camera on the tilt mount:** 12.5 cm high (was 10.3), 12.5 cm ahead of the axle, 1.1 cm
  left of center. `park.yaml` `robot.camera` has a new `left_m`; the Phase 3 photos keep
  their old camera in `data/photos/phase3/camera.yaml`.
- **Bin check = two looks.**
  - The level dwell photo counts litter on the floor; an up photo at **15°** judges what is
    on top. At 15° the frame holds the whole bin plus 44 cm above its rim.
  - The camera levels before any motion: `send()` refuses to drive while it is tilted.
- **Overflow level by count on top:** 1 / 4 / 7 items → levels 1 / 2 / 3. The height above
  the rim is recorded but not used: over 40 sim mounds per level the heights overlap
  between levels, the counts don't.
- **Litter:** floor items within 0.4 m of the bin's sides.
- **Dropped or kept as fallback:**
  - "full" dropped (opaque bins);
  - no measured distance: layout and centering give it; the pixel-width distance and the
    HC-SR04 echo are logged;
  - backing up stays the documented fallback, not built.
- **Sim litter is placed where the robot can see it** (user): on the corner side of each
  bin, within 24° of the camera's axis at the bin stop. Before, it was mostly behind the
  bins.
- **Bin-stop judging rules** (user-approved, from the robot runs; `detect.yaml` `bin_check`;
  only for the bin-check photos, not Phase 3's eval or centering):
  - the bin box must be within 12° of straight ahead (measured from the photo's center,
    because centering just faced the bin);
  - a faint bin box (low_conf) within 8° counts when none passed;
  - an item on top must rest on the rim (bottom ≤ 10 cm above it), and may hang 12% of the
    bin box's height below its top edge (soft items drape);
  - a floor item cut off at the bottom of the level photo is placed at the frame's bottom
    row.
- **Route follower guards** (user-approved, after robot run `20261122_142402`):
  - a **Pico restart** (`ping_seq` going backwards, or the counts jumping > 20000) stops the
    robot until the data has been fresh for 2 s, then repairs the estimate from the wheels;
    during the start hold, the start pose is taken again;
  - the **estimate-jump guard:** every 0.25 s while moving, the fused estimate may not move
    > 5 cm or turn > 20° more than the wheels; otherwise stop and repair from the wheels.
- **Phase 4 closed with limits** (user): "approaching the limit of what the camera can do".
  - The robot bin-check box is accepted at 0.75 (target 0.80).
  - The tilted-photo item-accuracy check (carried from Phase 3) is **deferred**: it
    measures the detector, not the robot. Its shot list is drafted in
    `data/photos/phase4/shots.yaml`.

**What was built:**
- **Firmware** (`walle-pilot/firmware/pico_pilot/`): `servo.c`/`servo.h`; `pico_pilot.c`
  reads the 4th `imu/std` value and sends the 5th tick value; `CMakeLists.txt`. The PWM
  divider comes from `clock_get_hz`: the Pico 2 W runs at 150 MHz.
- **`walle-app/servo/`:** `tilt.py` (`Calibration`, `Tilt`) and `servo_test.py` (`--deg`,
  `--us`, `--sweep`, `--stay`, `--sim --check`).
- **`walle-app/detect/`:**
  - `geometry.py`: `Camera` with tilt, lens offset and `left_m` (`ray`, `project`,
    `height_at`, `expected_box_px`); `analyze_bin_stop(..., stop=)` holds the bin-stop rules;
  - `bin_check.py`: `judge_bin`, `expected_from_layout`;
  - `replay_bin_checks.py`: re-judges a run's saved detections with the current rules,
    against `--truth "bin:level,on_top,litter"`;
  - `sim_truth.py` with tilt and lens offset;
  - `capture_set.py` / `eval_photos.py`: look, tilt and light per shot, `--by-light`, a
    per-set `camera.yaml`.
- **Route follower:**
  - the bin check after the dwell photo: tilt up → photo → judge → level;
  - writes `bin_checks.json`; `--no-tilt`; `route.yaml` `bin_check`;
  - the restart and jump guards; `summary.json` has `pico_restarts_s`, `estimate_jumps`,
    `tilt_moves`, `tilt_guard`.
- **Memory:** `route_records.py` stores judged checks (`method` `tilt` / `level_only`). A
  bin stop with both looks keeps `on_bin` from the up photo and `around_bin` from the level
  photo.
- **`park.py`:** litter in view (`LITTER_VIEW_DEG`, gap up to 25 cm, outward only as a
  fallback).
- **walle-sim:**
  - `camera_tilt_joint` + JointPositionController; one line of the Module 6 model changed
    (`camera_joint`'s parent);
  - `/sim/camera_tilt` bridge; the simulated Pico models the servo;
  - `/sim/restart`;
  - `harness/jump_ekf.py` (moves the EKF via `/set_pose`);
  - `make sim-bin-check`; `RESTART="at for"` and `JUMP="at dy"` options;
  - the `bincheck` scenario.
- **Tests:** 112 in total in `walle-app/tests/`; `test_bin_check.py` is new, plus guard
  tests in `test_route.py`.

**Results:**
- **Tests:** pass on the host and in both containers.
- **Sim:**
  - `servo_test --sweep --sim --check`: the joint is within 0.01° at −10/0/15/30°;
  - `sim-bin-check` PASS on `mixed` and `bincheck`;
  - restart (start hold and driving) and jump (30 cm) tests PASS;
  - no false alarms with `MISPLACE=12`, `COAST=0.35` or `BLACKOUT="20 1.5"`.
- **Robot runs** (ground truth from the user):

  | Run | Outcome right | Notes |
  |---|---|---|
  | `20261122_134315` | 2/4, 4/4 replayed with today's rules | the run the rules were tuned on |
  | `20261122_142402` | 3/4 | bin 2's bag drapes over the rim; the robot drove the lap 23 cm off after a Pico restart 2 s in |
  | `20261122_145845` (new scene, rules frozen) | 3/4 | no veer; ended 0 cm |

  - **The fair test** (runs 2–3): outcome 6/8 = 0.75; level within ±1 8/8; litter within
    ±1 7/8.
  - All three with today's rules: 10/12.
  - **The misses left are the detector:** items not found (the green bag in run 3, a flat
    lid, backlit wrapping), a milk jug boxed only at its top 70 px, a pile on top returned
    as one box, and items no camera at the stop could see.
- **The veer** (run 2): the fused estimate went 40 cm sideways in 0.5 s while the wheels
  moved 7 cm forward. The follower steered right to "get back".
  - The Pico had opened a new micro-ROS session 2.3 s after the follower started.
  - Replaying the jump test on the logged trajectories: it fires at 10.4 s in run 2 and in
    the Phase 3 Wi-Fi ghost spin (run `20261121_041923`, 115–119 s), and not in run 1.
  - How the restart became a 40 cm EKF step is not pinned down.

**Gotchas:**
- **Robot results are written only after the end-of-run measurement prompts.** In run 1 the
  user hadn't answered them yet, so the folder had photos but no `summary.json` or
  `bin_checks.json`.
- **`results/detect/`** gets folders only from `eval_photos.py`; route runs keep their
  detections in their own folder.
- **Run ids are UTC** (container clock); the host is EST. Run `20261122_142402` was 09:24
  local.
- **zsh:** `git show $B:walle-app/...` breaks (`$B:w` is a history modifier). Write the
  branch name out, or use `${B}`.
- **The firmware builds** with the user's VS Code Pico toolchain
  (`~/.pico-sdk/ninja/v1.13.2/ninja` in `walle-pilot/firmware/pico_pilot/build`). The
  user flashes with BOOTSEL + `picotool` or by drag-and-drop.
- **The servo twitches once** at power-up before it has a signal; that is normal.
- **The HC-SR04 was unplugged during the Phase 4 robot runs** (the user, afterwards), which
  explains `echo_us` −2 ("no response") in every one of them. It didn't answer in earlier
  phases either (unchecked). The user plugs it back in for Phase 5. With no answer it still
  bumps `ping_seq`, which the restart guard uses.
- **The sim:**
  - a park world takes about 30 s to come up headless;
  - relaunch it for each scenario: the world is built at launch;
  - the sim camera is the Module 6 one (0.10 m high, 0.07 m ahead, left 0), not the robot's.
- **Recreating containers:** never recreate only `walle-pilot`; Ollama runs on the host.

**Open:**
- **The Pico restart in run 2:** cause unknown. The user wasn't asked to confirm a reset; a
  brownout is possible.
- **The guards on the robot:** never triggered there yet. Watch the follower's output for
  "PICO RESTARTED" or "ESTIMATE JUMP".
- **Phase 3's item-accuracy box:** still open (deferred).
- **Next:** Phase 5 (obstacle detection and avoidance). Start prompt in
  `docs/handoff/phase4-to-phase-5/phase-5-prompt.md`.
