# CLAUDE.md

Guidance for Claude Code (and any other agent) working in this repository.

## Start of every session

1. Read [`ROADMAP.md`](ROADMAP.md): the phases, what "done" means for each, and the open decisions.
2. Read the most recent entries in [`LEDGER.md`](LEDGER.md): what has already been done, what
   was left open, and where the scope has moved.
3. Check the branch (`git status -sb`) and compare with `origin/main`. Local `main` can be
   stale; branch from `origin/main`.

## Working rules

- **Plan mode at the start of each roadmap phase.** Before writing code for a new phase, enter
  plan mode, read the code the phase touches, and agree the plan with the user. Resolve that
  phase's open decisions in `ROADMAP.md` there, not silently in code. Once approved, copy the
  plan to `docs/plans/phase-N-plan.md`.
- **Sim first.** Every new capability is built and checked in Gazebo (`walle-sim/`) before it
  runs on the robot. A phase is not done until its sim check passes *and* the user has run it
  on the real robot. Prefer checks that run headless with one command and report pass/fail.
- **Keep the ledger.** Before ending a turn that changed files, add an entry to `LEDGER.md`
  (format at the top of that file). Be honest in it: say what was verified, how, and what was
  not. Record scope drift when the work moves away from what was asked.
- **Keep the roadmap current.** Tick off "done" criteria as they are met, and record decisions
  when they are made. Don't silently rewrite a phase's goals: changing scope goes in the
  ledger first, then into the roadmap with the user's agreement.
- **Don't touch the frozen research protocol without asking.** `walle-pilot/config/pilot.yaml`,
  `walle-pilot/pilot/`, `walle-pilot/data/schedule/`, and the logs under `walle-pilot/data/`
  belong to the RQ1–RQ3 study. Logs are append-only. The WALL-E application work may reuse
  this code but must not change its behavior.
- **Robot safety.** Any tool that moves the robot keeps an emergency stop path. The Pico stops
  by itself 500 ms after the last `/pico/cmd_vel`. Don't remove that watchdog.
- Never commit secrets: `wifi_config.h` files are git-ignored; keep it that way.

## Project overview

WALL-E is a low-cost inspection rover for a model park. The repository holds two things:

1. **Research study (RQ1–RQ3)**, done: motion control (timed vs encoder-targeted drive),
   heading estimation (wheel-only vs EKF), and object ranging (HC-SR04 vs monocular depth).
   See [`walle-pilot/README.md`](walle-pilot/README.md).
2. **The WALL-E application**, in progress: drive a fixed route, check trash bins for
   overflow, detect and classify obstacles as trash or recycling, drive around them, remember
   everything it sees, and answer questions through a ROSA agent. See `ROADMAP.md`.

### Hardware

| Part | Role | Notes |
|---|---|---|
| Raspberry Pi Pico 2 W | motor control, encoders, IMU, ultrasonic; micro-ROS over Wi-Fi UDP 8888 | firmware in `walle-pilot/firmware/pico_pilot/` |
| TB6612 driver + 2 geared DC motors | differential drive | 4320 encoder counts per wheel turn |
| BNO055 IMU | heading | I²C on GP0/GP1 |
| HC-SR04 ultrasonic | forward range | TRIG GP17, ECHO GP16 through a divider (ECHO is 5 V) |
| ESP32-CAM (OV3660) | photos over HTTP (`/capture`) | firmware in `walle-pilot/firmware/esp32cam_camera/` |
| Servo | tilts the camera to look up at bins and tall objects | **not wired into the firmware yet** |

**Firmware limit:** the micro-ROS library in `libmicroros/` allows exactly 3 publishers and 2
subscriptions, and all 5 are used. Adding the servo needs either a library rebuild or reusing
an existing message (see the open decision in `ROADMAP.md`, Phase 4).

### ROS 2 topics (robot side)

| Topic | Type | Direction |
|---|---|---|
| `/pico/cmd_vel` | `geometry_msgs/Twist` | to Pico |
| `/pico/imu/std` | `std_msgs/Float32MultiArray` | to Pico |
| `/pico/wheel_ticks` | `std_msgs/Int32MultiArray` `[left, right, ping_seq, echo_us]` | from Pico; `echo_us` −1 = timeout, −2 = no sensor |
| `/pico/imu/data` | `sensor_msgs/Imu` | from Pico |
| `/pico/drive/status` | `std_msgs/Int32` | from Pico |
| `/wheel/odom`, `/odometry/filtered` | `nav_msgs/Odometry` | wheel odometry node, EKF |
| `/camera/image/compressed` | `sensor_msgs/CompressedImage` | camera node |
| `/vision/ask` → `/vision/result`, `/depth/ask` → `/depth/result` | `std_msgs/String` (JSON) | tool ↔ node request/response |

## Repository layout

```
Dockerfile, docker-compose.yml, Makefile   robot-side containers (ROS 2 Humble, arm64)
walle-pilot/
  config/            pilot.yaml (frozen study settings), ekf.yaml
  launch/            pilot.launch.py (odometry + EKF; fake:=true), vision/depth launch files
  firmware/          Pico 2 W (C, micro-ROS) and ESP32-CAM (Arduino/PlatformIO)
  odometry/ vision/ depth/ range/   ROS nodes + ROSA agents and tools per subsystem
  pilot/             RQ1–RQ3 study scripts (frozen protocol)
  tools/fake_pico.py simulated Pico for rehearsal without hardware
  ROSA_AGENT.md      the existing ROSA agents and tools
walle-app/           the WALL-E application (ROADMAP phases); see walle-app/README.md
  config/park.yaml   the park: 1 m square route, bins, robot footprint, sim item catalog
  config/scenarios/  test scenarios: which bins, their state, the obstacle
  park.py            park.yaml + scenario (+ overrides) -> layout; shared by sim and robot
  config/route.yaml  route follower settings
  route/             route_follower.py (Phase 1: drives the route on the EKF)
walle-sim/           Gazebo simulator; worlds/park_world.py builds the park world at launch
docs/plans/          approved phase plans (phase-N-plan.md)
docs/handoff/        start prompts for each phase's session, session summaries
```

`walle-sim/` has its own Compose project (ROS domain 42, no UDP 8888) and a simulated Pico
that speaks the same topics, so the code under test is the real code. The application code
imports from `walle-pilot/` but must not change the frozen study files.

## Commands

All robot-side commands run from the repository root on the host, or inside the container:

```bash
make up        # start the pilot + micro-ROS Agent containers
make shell     # shell in the pilot container (cwd /workspace/walle-pilot, ROS sourced)
make launch    # ros2 launch launch/pilot.launch.py
make fake      # same with tools/fake_pico.py instead of the Pico
make down
```

Simulator: `make sim-up`, `make sim-launch WORLD=<name>`, `make sim-shell`, `make sim-check`,
`make sim-down`. Details in `walle-sim/README.md`.

Park (the WALL-E test world):

```bash
make sim-launch WORLD=park SCENARIO=mixed GUI=false     # BINS=0-4 OBSTACLE=none|random|1-4 SEED=n
make sim-park-check LOOK_AT="bin_2 obstacle"            # layout vs ground truth, saves photos
make sim-route-check                                    # Phase 1: drive the route, PASS if <= 5 cm
python3 -m unittest discover -s walle-app/tests         # park + route tests (host or container)
```

Route follower on the robot (after `make launch`, robot on corner 1 facing corner 2):

```bash
python3 /workspace/walle-app/route/route_follower.py --scenario default --camera-url http://CAMERA_IP/capture
```

To stop a headless launch from outside, `pkill` inside the container with bracketed patterns
(e.g. `pkill -f "[i]gn gazebo"`) so the command doesn't match and kill its own shell.

ROSA agents talk to Ollama at `OLLAMA_URL` (default `http://host.docker.internal:11434`).
Ollama runs on the host, not in the container.

## Conventions

- Python 3.10 (ROS 2 Humble). Nodes use `rclpy`; ROSA tools use `langchain_core.tools.tool`.
- Tool ↔ node communication uses an `/x/ask` + `/x/result` String pair carrying JSON, with a
  request id. Follow that pattern for new nodes.
- Settings go in YAML under `config/`, not in code. Results and logs go in CSV/JSON under a
  `results/` or `data/` folder.
- Every tool that moves the robot asks a person for approval first (existing agents). Whether
  the WALL-E route runner keeps that rule is an open decision (`ROADMAP.md`, Phase 6).
