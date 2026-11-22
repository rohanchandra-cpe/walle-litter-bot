# WALL-E roadmap

The goal: a robot that drives a fixed route through a model park, checks trash bins for
overflow and for litter piling up around them, spots obstacles on its path, identifies them
as trash or recycling, drives around them, remembers what it has seen, and does all of this
when asked in plain language through a ROSA agent.

**Focus:** navigation, object detection, trash identification, bin monitoring, and ROSA
commands. Metrics and an LLM judge are stretch goals.

## How to use this file

- One phase at a time. Start each phase in **plan mode**: read the code it touches, settle its
  open decisions with the user, then build.
- Every phase is **sim first**: it is checked in Gazebo before it runs on the robot.
- A phase is done when every box under "Done when" is ticked. Tick boxes as they are met, and
  write the date and a short note in `LEDGER.md`.
- Scope changes go in `LEDGER.md` first, then here with the user's agreement.

**Status key:** ⬜ not started · 🟨 in progress · ✅ done

| # | Phase | Status |
|---|---|---|
| 0 | Foundations: sim merged, park defined | ✅ done 2026-11-19 |
| 1 | Route following | ✅ done 2026-11-18 (bin centering moved to Phase 3) |
| 2 | Memory store | ✅ done 2026-11-18 |
| 3 | Object detection and trash/recycling classification | 🟨 all done but item accuracy, carried into Phase 4 (2026-11-21) |
| 4 | Servo and bin monitoring | ✅ closed 2026-11-22 with recorded limits: robot bin checks 0.75 on the fair runs (target 0.80, accepted); item accuracy deferred |
| 5 | Obstacle detection and avoidance | ⬜ |
| 6 | WALL-E ROSA agent | ⬜ |
| 7 | Stretch: metrics and LLM judge | ⬜ |

Why this order: driving the route comes first because every later phase happens along it.
The memory store comes early so that detections, bin checks, and obstacles are recorded from
the moment they exist, instead of being added on later. Detection comes before avoidance and
bin checks because both depend on it. The agent comes last because it connects the finished
capabilities.

---

## Phase 0: Foundations

Get the ground ready: simulator on `main`, the park described once in config, and a Gazebo
world that matches it.

**Work**
- Merge `gazebo-test-environments` (`walle-sim/`) into `main`.
- Define the park in one config file: route waypoints, bin ids and positions, start pose.
  The robot, the simulator world, and the agent all read the same file.
- Build a Gazebo park world from that config: route, bins (empty / full / overflowing / litter
  around), and a place to drop obstacles.
- Decide where the WALL-E application code lives.

**Done when**
- [x] `walle-sim/` is on `main` and `make sim-check` passes. (PR #5; check passed 2026-11-15)
- [x] Park config file exists and is the single source for route and bin positions.
      (`walle-app/config/park.yaml` + `walle-app/park.py`)
- [x] Park world launches in Gazebo with the bins in the configured places.
      (`make sim-park-check`: PASS for `mixed` and for `mixed BINS=2 OBSTACLE=none`)
- [x] Real park layout measured and entered (user): real bin size, and the square taped out.
      (2026-11-16: interim bins are paper bags, 30 × 18 × 36 cm (L × W × H), **entered**
      2026-11-16 as box bins (Phase 1). Plastic bins from Target come later. Square not
      taped yet.) (2026-11-18: now black plastic bins, 27.7 × 21.1 × 29.2 cm (W × D × H),
      entered; the square is on corrugated plastic sheets.) (2026-11-19: the user confirmed
      the square is taped out.)

Plan: [`docs/plans/phase-0-plan.md`](docs/plans/phase-0-plan.md).

**Open decisions**
- ~~Location of the application code~~ → `walle-app/` (decided 2026-11-15).
- ~~Track shape and size, number of bins~~ → 1 m square, up to 4 bins, one per corner
  (decided 2026-11-15). The real track is marked with tape (2026-11-19).
- ~~Coordinate frame~~ → corner 1 is the origin, start there facing corner 2 (decided
  2026-11-15).

---

## Phase 1: Route following

Drive the route from the park config, waypoint to waypoint, and come back to the start.

**Work**
- A waypoint follower on `/odometry/filtered` that publishes `/pico/cmd_vel`.
- Closed-loop speed and heading control. The pilot showed open-loop driving is off (a
  0.15 m/s command drives at about 0.27 m/s); `walle-sim/nodes/firmware_models.py` has a PID
  candidate to test before writing it in C.
- Stops at bin waypoints and reports progress (current waypoint, distance driven).
- Drift handling: dead reckoning only (encoders + IMU), so error grows with distance. Measure
  how much, and decide whether a re-localization cue is needed.

**Done when**
- [x] In sim: the robot completes the route N times in a row and ends within X cm of the
      start. (N = 1 lap, X = 5 cm, set in plan mode; also: every bin wholly in frame at its
      stop.) `make sim-route-check`, 15 s bin waits, 2026-11-16: PASS on `ideal` 0.6 cm,
      `pilot_firmware` 1.6 cm, `carpet` 4.7 cm, `carpet_pid` 1.3 cm (one run each).
- [x] On the robot: completes the route and stops at each bin waypoint (user).
      (2026-11-18, run `20261118_195156`, carpet. Only bin 2 was centered in its photo; no
      bin top was in view. Open, see the ledger.)
- [x] End-of-route position error measured on the robot and recorded.
      (2026-11-18: 0 cm by tape ("almost exactly back where it started"), EKF 0.6 cm;
      wheels alone would have ended 22.8 cm / 22.7° off. `results/route/robot_runs.csv`.)

**Known conditions (2026-11-16, may change):** testing on carpet (cardboard later), so expect
more wheel slip than in the pilot; the robot runs on the current firmware (open-loop PWM map,
`MIN_DUTY` 0.40, so low speeds come out fast). **Updated 2026-11-18:** the test surface is
now corrugated plastic sheets, and the firmware has closed-loop wheel speed control.

Plan: [`docs/plans/phase-1-plan.md`](docs/plans/phase-1-plan.md).

**Open decisions**
- ~~PID in firmware (C) vs. on the computer side~~ → route control on the computer side
  (2026-11-16); **plus** closed-loop wheel speed in the firmware (2026-11-18), because the
  open-loop firmware turned at about 3 rad/s for any command and could not center the bins.
- ~~Bin shape in `park.yaml`~~ → boxes, wide face toward the square, placed so the whole bin
  is in frame from the corner (0.83 m out with the placeholder camera; measure the camera).
- Acceptable drift → 5 cm after 1 lap. Whether markers are needed is decided after the
  robot runs are measured (drift escalation list in the plan).

---

## Phase 2: Memory store

A persistent store of everything the robot observes, so it can answer history questions.

**Work**
- A database (SQLite is the default candidate) with a small write API used by later phases.
- Records, each with time, run id, and position: route runs, detections (label, trash or
  recycling, confidence, photo), obstacles (where, what, cleared or not), bin checks (bin id,
  fill estimate, overflow degree, litter count around it, photo).
- Read queries the agent will need: most-seen objects, which bins overflow most often,
  current obstacles on the route, recycling correctness.
- Sim runs and robot runs kept apart (a `source` field or separate files).

**Done when**
- [x] Schema written down and agreed. (2026-11-18: plan approved; `walle-app/memory/README.md`,
      `schema.sql`)
- [x] Write API used by a test that records a fake run, then the read queries return the
      expected answers. (`walle-app/tests/test_memory.py`: two fake runs from the `mixed`
      scenario's ground truth; host and both containers, 2026-11-18)
- [x] The store survives a container restart (lives in the mounted project folder).
      (2026-11-18: written in `walle-pilot`, container recreated, read back)
- Also checked: two `make sim-route-check` runs wrote their run and 4 bin checks to
  `sim.sqlite`; the 3 committed robot runs were imported into `robot.sqlite`. Robot run
  `20261119_043639` (2026-11-18) recorded itself in `robot.sqlite`: run + 4 bin checks.

Plan: [`docs/plans/phase-2-plan.md`](docs/plans/phase-2-plan.md).

**Open decisions**
- ~~SQLite vs. something else; vector store~~ → SQLite plus a vector store (Ollama
  `nomic-embed-text` embeddings in SQLite, cosine in Python) (2026-11-18).
- ~~Whether photos are stored, and for how long~~ → referenced by repo-relative path, kept with
  the run folder (2026-11-18).
- ~~What "recycling correctly" means~~ → items seen **on top of** a bin (above the rim; the
  robot can't see into the opaque bins) against the bin's type; litter around bins reported
  separately (2026-11-18).

---

## Phase 3: Object detection and trash/recycling classification

Find objects in camera photos and say whether each is trash or recycling.

**Work**
- A detection node using YOLO-World (open-vocabulary) on `/camera/image/compressed`, following
  the `/x/ask` + `/x/result` pattern.
- A vocabulary of the fake trash items, and a mapping from item to trash or recycling, in
  config.
- **Detect the bins too** (added 2026-11-18, moved from Phase 1): at each bin stop, find the
  bin in the photo and turn until it is centered, instead of trusting the fixed heading from
  `park.yaml` (Phase 1 turns to the configured bin position, so a misplaced bin ends up off
  center). Then look for trash around it.
- Check it on real ESP32-CAM photos first (offline, no driving), then in sim.
- Detections go to the memory store.

**Done when**
- [ ] On a labeled set of real ESP32-CAM photos of the fake trash: detection and
      classification accuracy measured and above a target agreed in plan mode.
      Measured 2026-11-21 on 52 photos (`results/detect/20261121_154929`): daylight recall
      0.91 / precision 0.84, lamp light 0.69 / 0.70, all 0.787 / 0.768 (target 0.80);
      category 0.945, bins 1.0, median 8.1 s. **Carried into Phase 4** (user, 2026-11-21):
      the tilted camera changes the view that fails.
- [x] Runs fast enough on the computer's CPU for use while driving (latency measured).
      (2026-11-20: median 5.1 s night / 8.1 s day per photo, p90 ≤ 11 s, with YOLO-World
      for bins + qwen3.5:4b in Ollama for items; target ≤ 15 s agreed with the user. The
      robot is stopped during detection.)
- [x] In sim, detections from the park world are written to the memory store. (2026-11-19,
      ground-truth stand-in `detect/sim_truth.py`; YOLO-World sees nothing in Gazebo)
- [x] At each bin stop the robot centers the detected bin in the frame (sim, then robot).
      Sim done 2026-11-19 (`make sim-route-check MISPLACE=12`, all 4 within 1°); re-run
      2026-11-20 with bins at 0.88 m: PASS, all 4 within 2.5°. Robot 2026-11-21
      (`20261121_000650`): bins 1 and 4 centered, bin 3 4.2° off after 3 corrections,
      bin 2 failed (junk box taken as the bin; pulses overshoot ~1.6x). Fixed 2026-11-21
      (size check, confidence-first pick, learned turn scale); sim PASS again. Robot re-run
      `20261121_035910`: bins 1, 3, 4 centered (learned scale 1.36); bin 2 not found on the
      first look (cut off at the frame edge, ~18° off its configured heading). Search turn
      added 2026-11-21 (sim PASS with bins 20° off and cut bins hidden). Robot 2026-11-21
      `20261121_041923`: all 4 centered within 3° (bin 2 found by the +15° search; learned
      turn scale 1.35–1.40). Done.

**Open decisions:** settled 2026-11-19, see the decision log and
`docs/plans/phase-3-plan.md`. Agreed targets on ~40 labeled ESP32-CAM photos: detection
recall and precision ≥ 80%, trash/recycling correct ≥ 90% of items found, bin found ≥ 95%,
median latency ≤ 1 s per photo on the container CPU. Centering tolerance 3°.

---

## Phase 4: Servo and bin monitoring

Tilt the camera up at a bin, judge how far it overflows, and count litter around it. The bins
are opaque, so there is no fill level (decided in Phase 2): a bin check records
`overflow_level` 0–3, `on_top_count`, `litter_count` and `outcome` (overflowing / litter /
clean). Plan: [`docs/plans/phase-4-plan.md`](docs/plans/phase-4-plan.md).

**Work**
- Servo control in the Pico firmware (FS5106B on GP22), and a servo joint on the simulated
  robot (`camera_tilt_joint`).
- A bin check at each bin stop, after Phase 3 centering: the level photo counts the litter
  on the floor; the camera tilts up (`servo.yaml` `up_deg`), the up photo judges what is
  above the rim; the camera levels again before any motion (`detect/bin_check.py`).
- ~~Distance to the bin from ultrasonic and/or the calibrated depth model, as in RQ3.~~
  Dropped (agreed 2026-11-21): the layout and centering already give the distance. The
  distance from the bin's pixel width and the HC-SR04 `echo_us` are recorded as diagnostics.
- Results go to the memory store (`bin_checks.json` in the run folder → `bin_checks`).

**Done when**
- [x] Servo moves to a commanded angle on the robot and in sim. *(Sim 2026-11-21:
      `servo_test.py --sweep --sim --check`, joint within 0.01° at −10/0/15/30°. Robot
      2026-11-21: calibrated (level 1575 µs, 10 µs/deg), readback matched every command,
      `--deg 15` read about 15° up against the floor (user).)*
- [x] In sim: bins set to clean / overflowing (levels 1–3) / litter around / overflowing with
      litter are classified correctly ("full" dropped: it can't be seen). `make
      sim-bin-check` PASS on `mixed` and `bincheck` (2026-11-21).
- [ ] On the robot: the same states with the real bins, accuracy recorded (user). Target:
      outcome right in ≥ 0.8 of bin checks, overflow level within ±1, litter count within ±1.
      *(2026-11-22, three runs; the rules were frozen after run 1, so runs 2–3 are the fair
      test: outcome 6/8 = 0.75, level ±1 8/8, litter ±1 7/8. All three with today's rules:
      10/12 = 0.83. The misses left are detection: items not found, a box covering only part
      of an item, items on top merged into one box.)* **Accepted below target (user,
      2026-11-22):** the limit is the camera, lighting, model and one view per bin, not the
      judging rules; more rule tuning would fit these scenes rather than improve the robot.
- [ ] Item accuracy carried from Phase 3: on labeled ESP32-CAM photos taken with the tilted
      camera at the bin stops, recall and precision ≥ 0.80 and category ≥ 0.90, in daylight
      and under the room lamps (`detect/eval_photos.py --set-dir
      walle-app/data/photos/phase4 --by-light`; shot list drafted, not yet agreed or taken).
      **Deferred (user, 2026-11-22):** it measures the detector, not the robot; useful for
      the paper. Last measured: daylight 0.91 / 0.84, lamp light 0.69 / 0.70 (Phase 3 set).

**Decisions (2026-11-21, plan mode):** see the decision log. Servo command on a 4th value of
`/pico/imu/std`, readback as a 5th value of `/pico/wheel_ticks`; calibration in
`config/servo.yaml`; two looks (level + up); overflow level by count of items on top;
litter within 0.4 m of the bin; no measured distance; backing up stays the fallback.

**Fallback if the servo can't be made to work (user's idea, 2026-11-21): back up.** Not
built. At the bin stop the robot backs straight away from the bin, along the corner's
diagonal into the square, takes the photo, drives forward the same distance on the EKF, and
resumes the route. Geometry (nominal 47° vertical FOV, camera 10.3 cm high): to see 25 cm
above a 29 cm rim the camera needs ~1.0 m to the bin's near face, vs ~0.65 m now, so back up
~0.35–0.45 m. Costs: the bin and items look ~35% smaller (harder naming); the square's inside
must be clear; the drive code only goes forward today.

---

## Phase 5: Obstacle detection and avoidance

Notice something on the path, identify it, drive around it, and rejoin the route.

**Work**
- Detection on the path: ultrasonic distance ahead, confirmed by the camera.
- When blocked: stop, identify the object (Phase 3), record it (Phase 2), plan a detour
  around it, and rejoin the route at the next reachable waypoint.
- A fallback when there is no way around (report and stop, or skip that waypoint).

**Done when**
- [ ] In sim: obstacles placed at random on the route are detected, recorded, and passed in
      N of M runs without collisions.
- [ ] On the robot: passes a real obstacle on the route and rejoins it (user).
- [ ] Obstacle positions in the memory store are within X cm of truth (sim).

**Open decisions**
- The HC-SR04 sees only straight ahead: how wide the detour is, and how to check the side is
  clear (turn and ping, or the camera).
- Whether the robot avoids an object it identifies as small enough to ignore.

---

## Phase 6: WALL-E ROSA agent

One ROSA agent that runs the whole thing from plain-language prompts.

**Work**
- Tools that call the finished capabilities: run the route, check a bin, look for obstacles,
  report an obstacle, and query the memory store.
- A system prompt describing the park, the bins, and the tools.
- Acceptance prompts, run in sim and on the robot:
  - "Run your route and check the path for any obstructions."
  - "Check Bin 1 to see if it is overflowing. If it is, estimate how much."
  - "Historically, what objects have you seen the most?"
  - "Are people generally recycling the correct objects?"
  - "Which bins overflow most often?"
  - "Where are the obstacles on the route right now?"

**Done when**
- [ ] Each acceptance prompt gives a correct action or answer in sim.
- [ ] Each acceptance prompt works on the robot (user).
- [ ] History answers match the memory store's contents (checked by query).

**Open decisions**
- Which LLM runs the agent (local Ollama model vs. a hosted model) and the latency and cost
  of each.
- Approval rule: today every movement asks a person. A full route run probably needs one
  approval for the run, with emergency stop kept.

---

## Phase 7 (stretch): Metrics and LLM judge

Measure how well WALL-E does its job.

**Work**
- Metrics, e.g.: route completion rate and position error, obstacle detection
  precision/recall, classification accuracy, bin fill estimate error, time per lap, false
  stops.
- A scripted evaluation in sim that runs a set of scenarios and reports the metrics.
- An LLM judge that scores the agent's answers and actions against the ground truth of each
  scenario.

**Done when**
- [ ] Metrics defined and computed automatically from a sim evaluation run.
- [ ] LLM judge scores agree with a human scoring on a sample of runs (agreement measured).

---

## Decision log

Decisions made during the phases, newest first. Each links to its ledger entry.

| Date | Phase | Decision |
|---|---|---|
| 2026-11-22 | 4 | Phase 4 closed with recorded limits (user): "approaching the limit of what the camera can do" (cheap camera and microcontroller, room lamps, a small model, one view per bin). Robot bin checks accepted at 0.75 outcome on the runs the rules weren't tuned on (level ±1 8/8, litter ±1 7/8); the tilted-photo item-accuracy check is deferred. Last rule added: an item on top may hang 12% of the bin box's height below its top edge (soft items drape over the rim; seen on three scenes). Next: Phase 5. |
| 2026-11-22 | 1/4 | Route follower guards after robot run 20261122_142402 (a Pico restart 2 s in; the fused estimate then went 40 cm sideways and the robot drove the lap 23 cm off), user-approved: a Pico restart (ping_seq going backwards, or the counts jumping > 20000) stops the robot until its data has been fresh 2 s, then repairs the estimate from the wheels; during the start hold, the start pose is taken again. Every 0.25 s while moving, the fused estimate may not move > 5 cm or turn > 20° more than the wheels; else stop and repair from the wheels. The overhang margin for items on a bin's rim (fix 1) is held back until varied setups show it again (user: avoid tuning to one scene). |
| 2026-11-22 | 4 | Bin-stop judging rules after the first robot run (user-approved): bin boxes only near straight ahead (12°; measured from the photo's center, since centering just faced the bin); a faint bin box (low_conf) within 8° is the bin when none passed; an item on top must rest on the rim (bottom ≤ 10 cm above it); a floor item cut off at the level photo's bottom is placed at the frame's bottom row and can count as litter. |
| 2026-11-21 | 4 | Sim litter and spills go on the corner side of each bin, within 24° of the camera's axis at the bin stop, so the robot can see them; before, they went mostly behind the bin (user's choice). The camera on the tilt mount is 12.5 cm high, 12.5 cm ahead of the axle, 1.1 cm left of center (user's measurement; `park.yaml` `robot.camera`). |
| 2026-11-21 | 4 | Camera servo command: a 4th value on `/pico/imu/std` (pulse in µs; ≤ 0 or missing keeps it), readback as a 5th value of `/pico/wheel_ticks`. No micro-ROS rebuild; 3-value senders and 4-value readers are unaffected (user's choice). Servo: FITEC FS5106B, 50 Hz on GP22, powered from a 5–6 V buck/UBEC off the 9 V supply (not 9 V directly: it is a 4.8–6 V servo), ground shared with the Pico. Degrees ↔ µs and the tilt limits live in `walle-app/config/servo.yaml` (no reflash to recalibrate); the firmware clamps to 600–2400 µs and slews 1000 µs/s. |
| 2026-11-21 | 4 | Bin check = two looks: the level dwell photo for litter, an up photo (15° up: the whole bin plus 44 cm above its near rim) for what is on top. Overflow level by count of items on top (1 / 4 / 7 → levels 1 / 2 / 3); the height-above-rim rule is off because it overlaps between levels in sim. Litter = floor items within 0.4 m of the bin's sides. Never drive tilted (the follower levels first). |
| 2026-11-21 | 4 | "Full" dropped from the done criteria; no measured distance (layout + centering give it; pixel-width distance and HC-SR04 echo are logged); backing up stays the documented fallback, not built. Robot target: outcome ≥ 0.8, level ±1, litter ±1 (user's choices). Item accuracy (from Phase 3) judged on a new tilted photo set, `data/photos/phase4/`, scored per light. |
| 2026-11-21 | 3→4 | Phase 3's item-accuracy box stays open and its target (recall and precision ≥ 0.80, category ≥ 0.90) is checked again in Phase 4 with the tilted camera, in daylight and lamp light. Measured on 52 photos: daylight 0.91 / 0.84, lamp light 0.69 / 0.70, all 0.787 / 0.768 (user's choice). |
| 2026-11-21 | 3 | The zoomed second search over the bin's rim runs only in good light (photo color cast, red − blue ≤ 9; the user's lamps measure 11–22). On the on-bin setups it lifted daylight recall 0.73 → 0.93 but cut lamp-light recall 0.75 → 0.56 (user's choice). |
| 2026-11-21 | 1/3 | The route follower stops and waits when the Pico's ticks or IMU are > 0.4 s old, and afterwards re-anchors the fused estimate on the wheels' motion across the gap (robot run 20261121_041923: the EKF spun on with no data and the follower made a real 73° pulse at the ghost heading). `route.yaml` `sensor_guard`; walle-sim `/sim/blackout` tests it. |
| 2026-11-20 | 3 | Latency target for detection: median ≤ 15 s per photo (was ≤ 1 s, set for YOLO-World alone). User: "I am not expecting sub 1 second performance" with local models. Bins sit 0.88 m out from their corner (`park.yaml` `offset_m`, confirmed by the user); the auto offset stays tested on its own. |
| 2026-11-20 | 3 | Grocery and spinach bags are one label, `plastic_bag` (user's choice; both trash). The model could not tell them apart (spinach bags 0/9). |
| 2026-11-20 | 3 | Remote (cloud) vision models, e.g. Gemini, are shelved: not tried, kept as an alternative for the paper's discussion of local-model limits (user's choice). Detection stays local (Ollama on the host). |
| 2026-11-20 | 3 | Items are detected in two steps (user chose "Option 2"): YOLO-World with generic prompts proposes boxes, `gemma4` names each crop (or rejects it). YOLO-World alone found 0/36 items on the night photos. Slower is accepted for accuracy. Tune on shots 1–29, report 30–42 as unseen; fallback = fine-tune YOLO. Bins stay with YOLO-World (29/29). |
| 2026-11-19 | 3 | Sim detection is a ground-truth stand-in (`detect/sim_truth.py`), not sim prompts: YOLO-World scored < 0.05 on every Gazebo object. The code after the model is the real one; accuracy is judged on real photos only (user chose this). |
| 2026-11-19 | 3 | Fake trash (user's labels): trash = crumpled paper towels, plastic grocery bag, spinach bag; recycling = aluminum coffee can, gallon milk jug, yogurt container + lid, crumpled foil, crumpled Chipotle paper bag. Extras (recycling): chicken-stock carton, tissue box, paper egg carton. Mapping in `walle-app/config/detect.yaml`, sharing names and categories with `park.yaml` `items` (a test checks they agree). |
| 2026-11-19 | 3 | YOLO-World (`ultralytics`, start with `yolov8s-worldv2`) runs in the containers on CPU, pinned in both Dockerfiles; budget median ≤ 1 s per photo; weights cached in `walle-app/models/` (git-ignored). `gemma4` not used in Phase 3. |
| 2026-11-19 | 3 | Accuracy is judged on ~40 real ESP32-CAM photos (never phone photos), labeled by item lists (no boxes): recall/precision ≥ 80%, category ≥ 90%, bin found ≥ 95%. Sim checks plumbing and centering only, with sim-only prompts for the primitive shapes. |
| 2026-11-19 | 3 | Bin centering between `face_bin` and `dwell`: nearest bin detection to the expected bearing, tolerance 3°, up to 3 corrections (pulses ≤ 20°), at most 25° from the configured heading; no bin found = keep the fixed heading and record it. Detector is a Python module the follower imports (robot stopped while it runs), plus a `/detect/ask` node for Phase 6. |
| 2026-11-19 | 3 | Memory at a bin stop: `on_bin` = box above the bin's top edge and overlapping it; `around_bin` otherwise; all `around_bin` when the rim is out of view. `obj_xy` from the box's bottom edge on the ground plane (not for `on_bin`). Below `min_conf` dropped (kept in `detections.json`); unmapped labels = `unknown`. Detections saved per run in `detections.json` and rebuilt by `import_runs.py`. |
| 2026-11-19 | 1 | Small heading corrections (≤ 20°) are timed pulses with a learned gain, skipped if the shortest would overshoot by more than the miss; up to 3 per turn. Slow correction turns stuck and lurched 8–13° on the robot. Photos: up to 3 tries, 3 s apart, within the bin wait. Needed for Phase 3's bin centering. |
| 2026-11-18 | 2 | Memory store: SQLite (stdlib) + vector store (Ollama `nomic-embed-text`, embeddings in SQLite, cosine in Python; keyword fallback without Ollama). Python API only (`walle-app/memory/store.py`); a `/memory/ask` node only if Phase 6 needs it. |
| 2026-11-18 | 2 | Sim and robot in separate files (`walle-app/data/memory/{robot,sim}.sqlite`), both git-ignored; `import_runs.py` rebuilds them from the committed run logs. Photos referenced by repo-relative path. Rollback journal (not WAL) for Docker bind mounts. |
| 2026-11-18 | 2 | Bin types: bins 1, 3 trash; 2, 4 recycling (`park.yaml` `bins.types`, scenario `type:` override). |
| 2026-11-18 | 2 | The robot can't see into the opaque bins: a bin check records what is above the rim (overflow 0–3, items on top) and the litter around it; outcome overflowing / litter / clean (clean = move on, still recorded). No fill level. Recycling correctness = items above the rim vs the bin's type; litter around bins listed, not judged. |
| 2026-11-18 | 2 | Current obstacles derived from history (latest sighting present), matched within 0.10 m on the same side; `mark_absent` records a cleared one. |
| 2026-11-18 | 1→3 | Phase 1 done: smooth closed-loop motion and route following on the robot. Centering the camera on the bin moves to Phase 3: YOLO detects the bins as well as trash, and the robot centers the detected bin. |
| 2026-11-18 | 1 | Turns at 1.0–1.5 rad/s, slowest drive 0.08 m/s: below ~0.1 m/s per wheel the motors stick and lurch. Firmware PI retuned (KP 1.0, filter 0.3) in sim against a stiction model fitted to the robot. |
| 2026-11-18 | 1 | Closed-loop wheel speed (module 7's PI, fixed gains) goes into `walle-pilot/firmware/pico_pilot/drive_control.c` itself, not a new folder (user: the study's original firmware is kept elsewhere). |
| 2026-11-18 | 1 | Bins are now black plastic boxes: 27.7 cm wide, 21.1 cm deep, 29.2 cm tall (offset 0.71 m). |
| 2026-11-16 | 1 | Route control runs on the computer side (ROS node on the EKF, built on module 8's `Mover`); firmware unchanged. Firmware wheel PID (module 7) held back as the drift fix. |
| 2026-11-16 | 1 | Pass: 1 lap, end within 5 cm of the start (sim truth; tape on the robot), every bin wholly in frame at its stop. |
| 2026-11-16 | 1 | Corners: stop and turn in place, to absolute park headings; sides follow the line between corners. |
| 2026-11-16 | 1 | Bin stops: on the corner, turn to face the bin's center, wait 15 s. |
| 2026-11-16 | 1 | Bins are boxes (30 × 18 × 36 cm bags), wide face toward the square, moved out so the camera sees the whole bin from the corner (`frame_from_corner`). Cylinders still supported. |
| 2026-11-16 | 1 | Drift is a priority: measured per corner and at the end, in sim and on the robot; the follower measures the gyro drift at rest and removes it. |
| 2026-11-16 | 1 | Safety: the user starts each run and stops it with Ctrl+C; per-move timeouts; Pico watchdog kept. |
| 2026-11-15 | 0 | Application code lives in a new top-level `walle-app/`. |
| 2026-11-15 | 0 | Park = 1 m × 1 m square, CCW route, corner 1 = origin and start (facing corner 2). |
| 2026-11-15 | 0 | Bins sit outside the corners on the diagonal (auto offset keeps the robot clear); up to 4. |
| 2026-11-15 | 0 | At most one obstacle per lap, on a side, 0.25–0.75 along it. |
| 2026-11-15 | 0 | Bin states: `ok` or `overflowing` (level 1–3), plus `litter` count around it. No fill levels or bin types yet. |
| 2026-11-15 | 0 | Test setups are scenario files + launch overrides (`BINS`, `OBSTACLE`, `SEED`); the park world is generated at launch. |
| 2026-11-15 | 0 | The sim item catalog only lists what the simulator can place; detection stays open-vocabulary (Phase 3). |
| 2026-11-15 | all | Approved phase plans are kept in `docs/plans/phase-N-plan.md`. |
