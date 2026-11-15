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
| 0 | Foundations: sim merged, park defined | ⬜ |
| 1 | Route following | ⬜ |
| 2 | Memory store | ⬜ |
| 3 | Object detection and trash/recycling classification | ⬜ |
| 4 | Servo and bin monitoring | ⬜ |
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
- [ ] `walle-sim/` is on `main` and `make sim-check` passes.
- [ ] Park config file exists and is the single source for route and bin positions.
- [ ] Park world launches in Gazebo with the bins in the configured places.
- [ ] Real park layout measured and entered (user).

**Open decisions**
- Location of the application code: a new top-level folder (e.g. `walle-app/`) vs. inside
  `walle-pilot/`. It must reuse `odometry/`, `vision/`, `depth/` without changing the frozen
  study code.
- Track shape and size, number of bins, and how the real track is marked (tape, none).
- Coordinate frame: is the start pose the origin each run?

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
- [ ] In sim: the robot completes the route N times in a row and ends within X cm of the
      start. (N and X set in plan mode.)
- [ ] On the robot: completes the route and stops at each bin waypoint (user).
- [ ] End-of-route position error measured on the robot and recorded.

**Open decisions**
- PID in firmware (C) vs. on the computer side.
- Acceptable drift, and whether to add markers (e.g. AprilTags at bins, or a known bin
  position the camera can line up on) to correct it.

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
- [ ] Schema written down and agreed.
- [ ] Write API used by a test that records a fake run, then the read queries return the
      expected answers.
- [ ] The store survives a container restart (lives in the mounted project folder).

**Open decisions**
- SQLite vs. something else. Plain tables vs. adding a vector store for free-text recall.
- Whether photos are stored, and for how long.
- What "recycling correctly" means in the data: an item seen in or near a bin of the wrong
  type? That needs bins to have a type (trash / recycling).

---

## Phase 3: Object detection and trash/recycling classification

Find objects in camera photos and say whether each is trash or recycling.

**Work**
- A detection node using YOLO-World (open-vocabulary) on `/camera/image/compressed`, following
  the `/x/ask` + `/x/result` pattern.
- A vocabulary of the fake trash items, and a mapping from item to trash or recycling, in
  config.
- Check it on real ESP32-CAM photos first (offline, no driving), then in sim.
- Detections go to the memory store.

**Done when**
- [ ] On a labeled set of real ESP32-CAM photos of the fake trash: detection and
      classification accuracy measured and above a target agreed in plan mode.
- [ ] Runs fast enough on the computer's CPU for use while driving (latency measured).
- [ ] In sim, detections from the park world are written to the memory store.

**Open decisions**
- Which fake trash items, and their trash/recycling labels.
- YOLO-World model size vs. CPU latency; whether to keep the existing vision-language model
  (`gemma4` through Ollama) as a second opinion or for descriptions.
- Gazebo objects won't look like the real items: in sim, does detection use real-looking
  textures, or is it checked mainly on real photos?

---

## Phase 4: Servo and bin monitoring

Tilt the camera up at a bin, estimate how full it is, and count litter around it.

**Work**
- Servo control in the Pico firmware, and a servo joint on the simulated robot.
- A bin check: drive to the bin waypoint, face the bin, take photos at one or more tilt
  angles, estimate fill level and overflow degree, and count litter on the ground around it.
- Distance to the bin from ultrasonic and/or the calibrated depth model, as in RQ3.
- Results go to the memory store.

**Done when**
- [ ] Servo moves to a commanded angle on the robot and in sim.
- [ ] In sim: bins set to empty / full / overflowing / litter-around are classified correctly.
- [ ] On the robot: the same states with the real bins, accuracy recorded (user).

**Open decisions**
- **Firmware limit:** micro-ROS is at 3 publishers / 2 subscriptions, all used. Options:
  rebuild `libmicroros` with higher limits, or carry the servo angle in an unused field of
  `/pico/cmd_vel` (e.g. `angular.x`). The second needs no rebuild but is a hack.
- Which Pico pin and PWM settings for the servo; its power supply (servos draw spikes).
- How to express "degree of overflow": a scale (e.g. 0 = empty … 4 = overflowing a lot), or a
  percentage. Ground truth for it during testing.

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
| | | |
