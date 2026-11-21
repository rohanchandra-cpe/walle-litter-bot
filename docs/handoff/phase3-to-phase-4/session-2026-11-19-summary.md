# Session summary: Phase 3, detection, classification, bin centering (2026-11-19 to 2026-11-21)

Paste this into a new session (or use it as the `/compact` instruction) to carry over what
this session did. The durable record is `LEDGER.md`; this is the short version.

---

You are continuing work on the WALL-E repository (`walle-litter-bot`). Read `CLAUDE.md`,
`ROADMAP.md` and the latest `LEDGER.md` entries first. What happened in the previous
session:

**Branch.** `walle-phase-3` was cut from `origin/main` (Phase 2 merged in PR #8). All of
Phase 3 is in commit `3dea433`. The user opened a **draft PR** for it, which is not merged.
Phase 3 is 🟨 in `ROADMAP.md`: every box is ticked except item accuracy.

Still uncommitted, on purpose:
- the user's move of the old handoff docs into `docs/handoff/phase*-to-phase*/` (4 deleted
  files plus new folders), and this folder, `docs/handoff/phase3-to-phase-4/`;
- `walle-app/data/photos/_props.jpg` (the user's photo of the props);
- three copies of the 340 MB CLIP weights in `weights/`, `walle-app/weights/` and
  `walle-pilot/weights/` (never commit them);
- `.DS_Store` and `.vscode/`.

Ask the user before committing any of these.

**Decisions** (ROADMAP decision log, 2026-11-19 to 2026-11-21):
- **Fake trash (the user's labels):**
  - trash = paper towels, plastic bags;
  - recycling = coffee can, milk jug, yogurt container, foil, Chipotle paper bag;
  - extras = carton, tissue box, egg carton.

  Spinach and grocery bags were **merged into one label, `plastic_bag`** (the model could
  not tell them apart). The mapping is in `walle-app/config/detect.yaml`.
- **Detector:** two-step, all local.
  - YOLO-World (`yolov8s-worldv2`, ultralytics 8.4.215, CPU in the containers) finds the
    **bins**: 30/30 in the photo set.
  - A vision model in Ollama on the host finds the **items** in the whole photo, then names
    each crop or rejects it.
  - The model is **`qwen3.5:4b`**: recall 0.70 vs `gemma4`'s 0.58 on the same photos, same
    speed. Qwen boxes are x-first: `vlm.box_order`.
  - YOLO-World alone found 0/36 items, which is why the vision model does the items.
- **Cloud vision models** (e.g. Gemini) are shelved, kept as an alternative for the user's
  paper.
- **Targets:**
  - recall and precision ≥ 0.80;
  - category ≥ 0.90;
  - bin found ≥ 0.95;
  - latency median ≤ 15 s per photo (raised from 1 s, confirmed by the user).
- **Sim:** Gazebo shapes fool YOLO-World, so `detect/sim_truth.py` projects the true layout
  instead (a ground-truth stand-in). Accuracy is judged on real photos only.
- **Bins stand 0.88 m out** from each corner on the diagonal (`park.yaml` `offset_m`). The
  camera is 12.5 cm ahead of the axle (`forward_m: 0.125`).

**What was built:**
- `walle-app/detect/`:
  - `detector.py` (`Detector`, `VLMDetector`, `make_detector`);
  - `geometry.py`: camera model, bearing, `plausible_bin`, `pick_bin` (most confident
    plausible box), `search_order`, `on_bin` / `around_bin`, ground-plane `obj_xy`;
  - `sim_truth.py`, `eval_photos.py`, `capture_set.py`, `probe_prompts.py`;
  - `detect_node.py` (`/detect/ask`).
- `walle-app/data/photos/phase3/`: 42 labeled ESP32-CAM photos, plus 10 daylight repeats
  in `shots.yaml` (43–52), not yet taken.
- **Route follower:**
  - `center_bin()`: photo, detect, pulse by bearing / (parallax × learned turn scale), up
    to 3 corrections; a ±15° search if no bin is seen;
  - `detect_at_bin()`: writes `detections.json`, which `memory/route_records.py` imports
    into the store.
- **Sensor guard** in the follower (`route.yaml` `sensor_guard`):
  - stale Pico ticks or IMU (> 0.4 s) → stop and wait;
  - afterwards, re-anchor the EKF estimate on the wheels' motion across the gap;
  - no learning across gaps.
- **walle-sim:**
  - `make sim-route-check` with `MISPLACE=deg`, `HIDE_CUT=1` and `BLACKOUT="every for"`;
  - `/sim/blackout` in `sim_pico_bridge.py`.
- A zoomed second search over the bin rim (`vlm.zoom_bin_top`) runs **only in good
  light**, judged by the photo's warm cast (red − blue ≤ 9). Under the user's lamps it hurt
  (on-bin recall 0.75 → 0.56); in daylight it helped (0.73 → 0.93).

**Results:**
- **Photo set, night shots 1–29 (tuning):** recall 0.70, precision 0.71, category 0.89,
  median 5.1 s. FAIL against 0.80.
- **Day shots 30–42 (unseen):** 0.90 / 0.90 / 1.00, median 8.1 s, PASS. But they are mostly
  items on the floor; all 42 photos together give about 0.76.
- **Where it fails:** items **on top of a bin**. At 0.88 m the bin's rim is at the top
  edge of the frame, so things on top are cut off (the milk jug never fully in view), and
  what does show is often misnamed at night.
- **Sim:** every check passes:
  - `MISPLACE=12`;
  - `MISPLACE=20 HIDE_CUT=1`, where the search finds the bins;
  - `BLACKOUT="20 1.5"`, where the guard repairs 9–11 cm of ghost motion.
- **Robot runs:**
  - `20261121_000650`: bin 2 failed, because a junk box was taken as the bin and every
    pulse overshot about 1.6×.
  - `20261121_035910`: bin 2 was not found because it was cut off at the edge.
  - `20261121_041923`: **all 4 bins centered within 3°** (bin 2 via the search), with the
    learned turn scale at 1.35–1.40. A Wi-Fi gap made the EKF spin on a ghost heading at
    bin 4; the follower made a real 73° pulse at it, which led to the guard.
  - Detections in these runs:
    - bins 1, 3 and 4 right every time;
    - bin 2's on-top jug cut off by the frame, at best reported as `yogurt_container`,
      `on_bin`, recycling.

**Gotchas:**
- **Recreating containers:** never recreate only the `walle-pilot` container (the
  micro-ROS Agent shares its network). After a Dockerfile change: `make build` /
  `make sim-build`, then `make down && make up`.
- **Ollama:** runs on the host (`host.docker.internal:11434`). If an eval runs while the
  robot drives, they share it and stops get slower.
- **YOLO-World bin scores are low** (0.14–0.8) and drop near the frame edge. Faint boxes
  down to 0.01 come back in `low_conf`, but only as search hints.
- **Pulses on the robot turn about 1.37× what the EKF records 0.3 s later.** The centering
  turn scale absorbs this; route turns don't need it.
- **The sim's ground truth** stands in for the detector (all confidence 1.0), so ties in
  `pick_bin` go to the expected bearing.
- **Python versions:** the host is 3.13 (no PIL, no ultralytics; model tests skip); the
  containers are 3.10.
- **`gh` is not installed;** the user opens PRs.
- **Headless sim:** stop it with bracketed `pkill -f "[i]gn gazebo"` inside the container.

**After the handoff (2026-11-21):**
- The user took daylight shots 43–52 and fixed the labels of 45 and 47 (two plastic bags
  each).
- All 52 photos with the light-gated zoom (`results/detect/20261121_154929`):

  | Light | Recall | Precision |
  |---|---|---|
  | Daylight | 0.91 | 0.84 |
  | Lamp light | 0.69 | 0.70 |
  | All | 0.787 | 0.768 |

- **The user left the accuracy box open and carried the target into Phase 4,** to be met
  with the tilted camera, in both lights.

**Open:**
- **Sensor guard on the robot:** not yet run there. The next robot run's `summary.json`
  `sensor_gaps` shows whether the gaps are real.
- Next: Phase 4 (servo and bin monitoring). Start prompt in
  `docs/handoff/phase3-to-phase-4/phase-4-prompt.md`.
