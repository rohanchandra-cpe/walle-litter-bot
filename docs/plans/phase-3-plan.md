# Phase 3 plan: object detection, trash/recycling classification, bin centering

Approved by the user 2026-11-19 (after two additions: the scored photo set is taken with
the ESP32-CAM, not a phone; Claude guides the photo set with a shot list and capture script).

## Context

Phases 0–2 are done: the robot drives the 1 m route, faces each bin on the EKF, takes one
photo per bin stop, and records runs in the memory store. It doesn't yet know what is in the
photos. In the last robot run, two bins were off-center because they had been placed badly.
Phase 3 adds an open-vocabulary detector (YOLO-World):
- it finds the user's fake trash and labels each item trash or recycling;
- it finds the bin and centers it;
- it writes what it sees to the memory store.

Phase 4 (bin checks) and Phase 5 (obstacles) build on this. Branch `walle-phase-3` from
`origin/main` (already created). Copy this plan to `docs/plans/phase-3-plan.md` on approval.

## Decisions (go into the ROADMAP decision log)

1. **Fake trash.** The labels are the user's, US curbside style.
   - Labeled set, 8 items:
     - trash: crumpled paper towels, plastic grocery bag, spinach bag;
     - recycling: aluminum coffee can, gallon milk jug, yogurt container + lid, crumpled
       aluminum foil, crumpled Chipotle paper bag.
   - The trash items appear more often in the photos, so both categories are about equally
     represented.
   - Unseen extras, kept in the vocabulary and all recycling: chicken-stock carton, tissue
     box, paper-pulp egg carton.
   - The mapping lives in a new `walle-app/config/detect.yaml`, linked to `park.yaml`
     `items` by name: the four sim items keep their names and categories, and a test makes
     sure the two files agree. The sim list still doesn't limit detection.
2. **YOLO-World.**
   - Runs in the containers, on CPU.
   - Packages: `ultralytics`, OpenAI `clip` for the text prompts, and
     `opencv-python-headless` (a numpy-2 build that shadows the sim's broken apt cv2), added
     to both Dockerfiles with exact pins.
   - Starts with `yolov8s-worldv2`; moves to `m` only if accuracy needs it and latency
     allows.
   - Budget: median ≤ 1.0 s per photo at 640 px. The robot is stopped during every
     detection.
   - Weights are cached in `walle-app/models/` (git-ignored). They download once, on the
     first run.
3. **`gemma4`:** not used in Phase 3, since a 10 s+ answer doesn't fit the stop. It can come
   back later as an optional second opinion if Phase 4 or 6 wants one.
4. **Labeled set:** about 40 real ESP32-CAM photos.
   - Each has 1–3 items, at bin stops (on top of a bin and around it) and on the route.
   - Backgrounds: the taped plastic sheets and the carpet.
   - **Every photo is taken with the robot's ESP32-CAM, not a phone**: the same low
     viewpoint (10 cm), resolution, JPEG quality and sensor that the robot uses when it
     drives. Phone photos may be used only for extra prompt tuning, never in the scored
     set. To take them, power only the 5 V side, so the motors stay off.
   - Labels are lists, not boxes: `labels.yaml` gives each photo its items with count and
     place, plus whether a bin is visible.
   - Targets:
     - detection recall ≥ 80% and precision ≥ 80% (items matched by name and count per
       photo);
     - trash/recycling correct for ≥ 90% of the items found;
     - the bin found in ≥ 95% of photos that contain one;
     - median latency ≤ 1 s.
5. **Sim:** real photos judge accuracy; sim checks the plumbing and the centering.
   - A sim-only prompt set (`sim_prompts` in `detect.yaml`, e.g. "red cup" → `paper_cup`)
     is used only when `source=sim`, so the real pipeline runs unchanged.
6. **Bin centering** (between `face_bin` and `dwell`):
   - Prompts: "trash can" / "black bin" / "waste bin"; pick the best on the real photos.
   - Choose the bin detection nearest the expected bearing. Bearing = atan((cx − 320) / fx),
     with fx from the 60° HFOV, checked against a known angle on the robot.
   - When the box touches the left or right edge, the bin is partly out of view: aim at the
     visible edge plus half the bin's expected pixel width (from the bin's distance in the
     layout).
   - Tolerance 3° (the same as `correct_tol_deg`). At most 3 corrections, using `pulse()`
     for ≤ 20° and `turn_to` above that.
   - Never more than 25° from the configured heading.
   - No bin found: keep the fixed heading and record `centered: false` and the reason.
   - The dwell photo (taken after centering) is the bin photo. The centering photos are
     saved too (`bin_N_stepK_c{j}.jpg`).
7. **Interface:**
   - A Python module, `walle-app/detect/detector.py`, imported by the follower. It is
     loaded once at startup and called in the main thread while the robot is stopped. The
     Pico watchdog stops the robot, Ctrl+C still works, and each call has a time limit.
   - A thin `detect_node.py` serves `/detect/ask` + `/detect/result` (JSON, request id) on
     `/camera/image/compressed`, following the pattern of `vision_node.py`, for Phase 6.
   - `--no-detect` turns detection off in the follower.
8. **Memory at a bin stop:**
   - **Place:**
     - `on_bin` if the item's box overlaps the bin's box horizontally and its bottom edge
       is above the bin's top edge (plus a margin);
     - `around_bin` otherwise.
     - If the bin's top is cut off (its box touches the top of the frame), nothing above
       the rim is visible, so everything counts as `around_bin`, and the run records
       `rim_visible: false`.
   - **`obj_xy`:** for `around_bin` and `route` items, estimated on the ground plane from
     the box's bottom edge (camera 0.103 m high, 12 cm ahead, robot level); none for
     `on_bin`. Checked against sim ground truth.
   - **Confidence:**
     - Detections under `min_conf` (≈0.25, set in config) are dropped and kept only in
       `detections.json`.
     - Labels not in the mapping get category `unknown`.

## Files

**New**
- `walle-app/config/detect.yaml`: the model, image size, confidence threshold, the
  vocabulary (label → prompts, category), `sim_prompts`, bin prompts, and the centering
  settings.
- `walle-app/detect/detector.py`: `Detector`.
  - `detect(jpeg) -> [{label, category, conf, bbox}]`, with the bin as its own class.
  - It sets the classes once per vocabulary.
- `walle-app/detect/geometry.py` (pure, unit-tested): pixel to bearing, partly visible
  edge, `on_bin`/`around_bin` place, and the ground-plane `obj_xy`.
- `walle-app/detect/detect_node.py`: the ROS `/detect/ask` node.
- `walle-app/detect/eval_photos.py`: the headless offline check. It runs the labeled set
  and reports per-photo results, recall, precision, classification, bin found and latency
  (median and p90), and prints PASS/FAIL against the targets. Results go to
  `walle-app/results/detect/<date>/`.
- `walle-app/data/photos/phase3/`: the real photos plus `labels.yaml`, made with the user.
- `walle-app/tests/test_detect.py`: geometry, place rules, config agreement with
  `park.yaml`, and the importer round trip with a stub detector (no model needed).

**Changed**
- `walle-app/route/route_follower.py`:
  - a `center_bin()` step after `turn_to` in `face_bin`;
  - after the dwell photo, detection runs and its results go into the event and
    `<run>/detections.json`;
  - `summary.json` gains `centering` per bin (bearings before and after, pulses,
    `centered`).
  - `pulse()`, `turn_to` and `photo()` are reused unchanged.
- `walle-app/route/route_geometry.py`, if a helper fits there.
- `walle-app/memory/route_records.py`: reads `detections.json` and calls `add_detection`
  with the place, bin, pose, `obj_xy`, bbox, photo and model. `import_runs.py` therefore
  rebuilds them automatically.
- `walle-app/config/route.yaml`: `detect: true`, plus the centering switch.
- `Dockerfile`, `walle-sim/docker/Dockerfile`: the pinned packages. This needs a container
  rebuild: `make down && make up` (never recreate only `walle-pilot`).
- `Makefile`:
  - `make detect-eval` (offline photo set);
  - `make sim-detect-check`: a park route run with `SCENARIO=mixed OBSTACLE=none`. It
    checks that detections land in `sim.sqlite` with the right bin and place against
    ground truth, and that each bin ends within 3° true bearing after centering, with bins
    deliberately misplaced by a scenario override. It prints PASS/FAIL.
- `.gitignore`: `walle-app/models/`.
- `ROADMAP.md`, `LEDGER.md`, `walle-app/README.md`, `CLAUDE.md` (layout and commands).

Frozen files (`walle-pilot/pilot*`, `pilot.yaml`, `data/`) and `vision_node.py` are not
touched.

## Order of work

1. **Packages:**
   - pin and install in both Dockerfiles, and rebuild;
   - check that `import ultralytics, cv2, clip` works with numpy 2.2.6 in both containers;
   - measure the latency of S and M on an existing bin photo.
2. Write `detect.yaml`, `detector.py` and `geometry.py`, with unit tests.
3. **Bin prompts on the existing robot photos.** The bins in the 14 robot photos are
   unlabeled for trash, which fits the bin-detection part. Pick a bin prompt.
4. **User, with my guidance:** take about 40 labeled photos.
   - I write a numbered shot list. Each shot says which items to use, where they go (on
     the bin, around it, on the route; mat or carpet; near or far) and where the robot
     sits.
   - `capture_set.py` steps through it: it prints the setup, waits for Enter, saves
     `/capture` as `shot_NN.jpg`, and fills in that shot's `labels.yaml` entry from the
     shot list. You only fix an entry if what you placed differs.
   - I check each batch of photos (framing, items in view) before we go on. Then run
   `make detect-eval` and tune the prompts and threshold. The checkbox for accuracy and
   latency is ticked here.
5. **Follower:** centering and detection in the follower; detections into
   `detections.json`, the memory store and the importer.
6. **Sim:** `sim_prompts` and the `make sim-detect-check` pass, with centering on
   misplaced bins.
7. **User, robot run:** bins centered, detections in `robot.sqlite`, `import_runs.py
   --rebuild` keeps them.

## Verification

- `python3 -m unittest discover -s walle-app/tests`, on the host (3.13) and in both
  containers (3.10). Tests that need the model are skipped when `ultralytics` is missing.
- `make detect-eval`: PASS on the targets, and its report committed under
  `walle-app/results/detect/`.
- `make sim-detect-check` (`PROFILE=plastic_pid`): PASS. Detections appear in
  `sim.sqlite` (`MemoryStore("sim").most_seen()`), and the bins end within 3° after
  centering.
- **Robot run** (user): `route_follower.py --camera-url http://192.168.1.201/capture`.
  - Each bin is centered (from the photos and from `centering` in `summary.json`);
  - detections are in `robot.sqlite`;
  - `import_runs.py --rebuild` gives the same counts.
- The ledger entry and the ROADMAP ticks go in at each step.

## Risks

- YOLO-World may miss crumpled items (foil, paper towels, bags) at the camera's low angle.
  The prompts can be tuned per item; if needed, go to size M, or fall back to a "crumpled
  paper" / "plastic bag" generic prompt.
- Telling the Chipotle bag from the paper towels (both crumpled paper, but different
  categories) is the hardest case, and is reported on its own.
- The first run needs internet in the container to download the weights and CLIP.

## Amendment 2026-11-20: two-stage detector (user approved "Option 2")

On the first 29 labeled photos YOLO-World S found 0 of 36 items at conf 0.25; below the
threshold its names were mostly wrong (yogurt tub → "milk jug" 0.53; foil, cans, towels and
bags not seen). Bins were found in 29/29. So item detection becomes two steps:

1. **Find:** YOLO-World with generic prompts (trash, bag, crumpled paper, bottle, can,
   container, ...) at a low threshold proposes boxes; the bin prompts stay as they are.
2. **Name:** each proposal is cropped and sent to `gemma4` (Ollama on the host), which picks
   one of the vocabulary labels or "not trash" (rejects background). The category comes from
   the label, as before. Slower (seconds per item) is accepted: accuracy comes first.

Plain YOLO-World S and M are scored alongside for comparison. To keep the score honest,
methods and settings are chosen on shots 1–29 (night) only; shots 30–42 (day, mostly route)
are reported separately as unseen. If the find step misses items, the fallback is
fine-tuning YOLO on the user's items (needs boxed training photos).
