# Start prompt: Phase 3, object detection and trash/recycling classification

Paste everything below the line into a new Claude Code session opened in the repository.

---

You are starting **Phase 3 (Object detection and trash/recycling classification)** of the
WALL-E application in this repository. Work with me (the user) as the project owner; I run
the real robot.

## First, orient yourself

1. **Read the project docs.** Follow the "Start of every session" steps in `CLAUDE.md`.
   - Read `ROADMAP.md` in full: Phase 3 especially, and Phases 4–6, which use detection.
   - Read the latest entries in `LEDGER.md`.
   - `docs/handoff/phase2-to-phase-3/session-2026-11-18-summary.md` is the short version of
     Phase 2.
   - `walle-app/memory/README.md` describes the memory store you will write to.
2. **Branch from `origin/main`.** Phases 0–2 are merged to `main` (Phase 2: PR #8,
   `2a42319`). Create `walle-phase-3` from `origin/main`; local `main` may be stale.
   - The working tree may hold my uncommitted move of the old handoff docs into
     `docs/handoff/phase*-to-phase*/` folders. Bring it along untouched, and ask me before
     committing it.
   - Never commit `.DS_Store` or `.vscode/`.
3. **Enter plan mode before writing any code** (`CLAUDE.md` working rule). Read the code
   listed below and settle the open decisions with me. When I approve, copy the plan to
   `docs/plans/phase-3-plan.md`.

## What Phase 3 must deliver (from `ROADMAP.md`)

Find objects in camera photos and say whether each is trash or recycling:
- a detection node using YOLO-World (open-vocabulary), following the `/x/ask` + `/x/result`
  pattern;
- a vocabulary of the fake trash items, and a mapping from each item to trash or recycling,
  in config;
- **bin detection and centering:** at each bin stop, find the bin in the photo and turn
  until it is centered, instead of trusting the fixed heading from `park.yaml`;
- detections written to the memory store;
- checked on real ESP32-CAM photos first (offline, no driving), then in sim.

Done when:
- on a labeled set of real ESP32-CAM photos of the fake trash, detection and classification
  accuracy is measured and above a target we agree in plan mode;
- it runs fast enough on the computer's CPU for use while driving (latency measured);
- in sim, detections from the park world are written to the memory store;
- at each bin stop the robot centers the detected bin in the frame (sim, then robot).

## The code you are building on

### Camera

**ESP32-CAM** (`walle-pilot/firmware/esp32cam_camera/esp32cam_camera.ino`):
- OV3660 sensor; `GET /capture` returns a fresh 640 × 480 JPEG (quality 12, flipped
  vertically in firmware, about 14 KB), at `http://192.168.1.201/capture` on my network.
- Field of view in `walle-app/config/park.yaml` `robot.camera`: 60° horizontal × 47°
  vertical (nominal).
- The camera sits 10.3 cm above the floor and 12 cm ahead of the axle.

**ROS side** (`walle-pilot/vision/`):
- `camera_node.py` publishes `/camera/image/compressed` from the ESP32 (`source:=http`) or
  from Gazebo (`source:=sim`).
- `vision_node.py` answers `/vision/ask` with an Ollama vision model (default `gemma4`).
  It is the pattern to follow; reuse it, but don't change it.

**In sim**, `walle-sim/nodes/sim_camera_server.py` serves the Gazebo camera at
`http://localhost:8080/capture`, so the same code works in sim and on the robot.

### Real photos you already have

`walle-app/results/route/<run>/photos/bin_N_stepK.jpg`, from 4 robot runs: about 14 bin
photos, black plastic bins, the robot on the corner facing the bin. There is no fake trash
in them yet. The labeled trash set is still to be made with me.

### Route follower (Phase 1, `walle-app/route/route_follower.py`)

The bin stop is two steps:
1. `face_bin`: `turn_to(view_heading)` turns to the configured bin center on the EKF.
2. `dwell` (15 s): `hold_still()` takes the photo 1 s in (`photo_with_retries`), then
   finishes the wait (zero-velocity update).

Centering belongs between these two steps.

For small turns, use `self.pulse(heading, err_deg)`, not a slow turn. Slow turns stick and
then lurch 8–13° on this robot.
- A pulse is one turn command and a stop after a set time, with a learned gain. On the robot
  the gain is about 118°/s, and 3–6° pulses land within 0.6°.
- `rg.plan_pulse` skips a pulse that would overshoot by more than the miss.

A pixel offset converts to a bearing through the camera's field of view: about 30° from the
image center to the edge. Check the lens against a known angle.

`summary.json` events record per-bin poses and photo paths. Bins 3 and 1 in the last robot
run were off-center only because of placement (faced within 1°), which is exactly what
centering should fix.

### Memory store (Phase 2, `walle-app/memory/store.py`)

`MemoryStore("robot" | "sim")`:
- `add_detection(run_id, label, category, place=..., bin_id=..., confidence=..., pose=...,
  obj_xy=..., bbox=..., photo=..., model=...)`.
- `category` is `trash`, `recycling` or `unknown`.
- `place` is one of:
  - `route`;
  - `on_bin`: above a bin's rim (counts as put in that bin, for recycling correctness);
  - `around_bin`: on the ground next to it;
  - `obstacle`.
- The bin's type comes from the run's layout.
- `add_bin_check(...)` is Phase 4's. Phase 3 may record detections at the bin stops.
- **Rebuilds:** the `.sqlite` files are git-ignored. `memory/import_runs.py` rebuilds them
  from the run folders, and today it reads only route summaries. Detections must also be
  saved in the run folder (e.g. `detections.json`), and the importer extended.

### Park and sim

- **Park:** `walle-app/park.py` `load_park(...)` gives bins with `id`, `type`, `position`,
  `size_m`, `view_heading_deg`, `state`, `mound` (items above the rim), `spill` and
  `litter` (items on the ground around the bin), each item with `item` and `category`.
  Scenarios `mixed` / `obstacle_only` place them. That is ground truth for sim checks.
- **Sim objects are primitive shapes.** `park.yaml` `items`: `plastic_bottle`, `can` →
  recycling; `paper_cup`, `wrapper` → trash. They are colored cylinders and spheres
  (`walle-sim/worlds/park_world.py`). A real-object detector probably won't recognize them.
- **The sim's item list must not limit detection**, which stays open-vocabulary.
- **Commands:** `make sim-launch WORLD=park SCENARIO=mixed GUI=false PROFILE=plastic_pid`,
  `make sim-park-check LOOK_AT="bin_2 obstacle"` (saves photos), `make sim-route-check`.
- **The route follower refuses a layout with an obstacle** (avoidance is Phase 5). Use
  `OBSTACLE=none` for route runs.

### Software environment (checked 2026-11-19)

- Host: Apple M3 Pro, 12 cores. Both containers are aarch64, CPU only (Docker on macOS has
  no GPU).
- `walle-pilot`: torch 2.14 (CPU), numpy 2.2.6, PIL, onnxruntime. No `ultralytics`, no
  OpenCV.
- `walle-sim`: the same, plus an apt OpenCV built for numpy 1.x that **fails to import with
  numpy 2.2.6**. Pin versions carefully.
- Adding packages means changing the Dockerfiles (`Dockerfile`, `walle-sim/docker/Dockerfile`)
  and rebuilding. Or run the detector on the host (native, MPS) and reach it over HTTP.
- Ollama models on the host: `gemma4`, `qwen3.5:4b`, `llama3.2`, `nomic-embed-text`.

## Real-world conditions (may change; I'll tell you)

- **Bins:** black plastic, 27.7 × 21.1 × 29.2 cm (W × D × H), opaque. They stand 0.71 m out
  on each corner's diagonal (`python3 walle-app/park.py` prints the positions). In the last
  run some stood too close, so their tops were out of view. Bins 1 and 3 are trash, bins 2
  and 4 recycling.
- **The camera is too low to see into a bin.** It sees what is above the rim and what lies
  around the bin. Tilting the camera up (servo) is Phase 4.
- **Track and firmware:** the square is taped out on corrugated plastic sheets; carpet shows
  beyond them in the photos. The robot runs the closed-loop firmware.
- **Power:** 9 V for the motors, a separate 5 V for the Pico, the HC-SR04 and the camera.
  Charge both before robot runs; a weak 5 V supply resets the camera. The HC-SR04 has never
  answered.
- **Never recreate only the `walle-pilot` container:** the micro-ROS Agent shares its
  network, and the Pico's topics vanish.

## Open decisions to settle in plan mode

Recommend an option for each, and record the outcomes in the `ROADMAP.md` decision log.
1. **Fake trash:** which items, their trash/recycling labels, and where that mapping lives.
   Keep it separate from the sim's `park.yaml` `items`, or link the two?
2. **YOLO-World:** model size against CPU latency, and the latency budget. Where it runs:
   in the container (CPU) or on the host (MPS, over HTTP). The packages and pins needed.
3. **The existing `gemma4` vision model:** keep it as a second opinion (e.g. for trash vs
   recycling), for descriptions only, or not at all.
4. **The labeled real photo set:** how many photos, which items and backgrounds, how they are
   labeled (boxes, or labels only), and the accuracy target for detection and for
   classification.
5. **Sim:** do sim objects get real-looking meshes and textures, or is accuracy judged on
   real photos while sim checks only the plumbing (node → memory store, centering loop)?
6. **Bin centering:**
   - the bin prompt;
   - centering tolerance (degrees or pixels);
   - maximum pulses;
   - what happens when no bin is detected (keep the fixed heading and record it);
   - whether the photo is retaken after centering.
7. **Interface:** a detection node with `/detect/ask` + `/detect/result` (JSON, request id),
   a Python import used by the follower, or both. How the follower calls it during a bin stop
   without blocking the robot's safety (Ctrl+C, timeouts, the Pico watchdog).
8. **What goes into the memory store at a bin stop:**
   - telling `on_bin` from `around_bin` in a photo whose rim may be out of view;
   - whether to estimate `obj_xy` (ground plane from the box's bottom edge, using the camera
     height) or leave it empty;
   - what happens to low-confidence detections.

## How to work

- **Sim first,** with a headless one-command check that reports pass/fail. Here that means
  an offline accuracy and latency script on the photo set, then a sim run that writes
  detections and centers bins. A phase is done only after I run it on the real robot.
- **Don't change frozen files:** `walle-pilot/config/pilot.yaml`, `walle-pilot/pilot/`,
  `walle-pilot/data/schedule/`, and the logs under `walle-pilot/data/`. Reuse
  `walle-pilot/vision/`, but don't change its behavior.
- **Settings** go in YAML under `config/`; results go in CSV/JSON under `results/` or
  `data/`. New nodes use `rclpy`.
- **Test** on the host (Python 3.13) and in the containers (3.10):
  `python3 -m unittest discover -s walle-app/tests`.
- **Ledger and roadmap:** add a `LEDGER.md` entry before ending any turn that changed files:
  what was verified and how, what wasn't, and any scope drift. Tick the `ROADMAP.md` boxes
  as they are met.
- **Git:** don't commit or push unless I ask. Never commit `wifi_config.h`. `gh` is not
  installed; I open PRs in the browser.
