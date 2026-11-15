# WALL-E in Gazebo

Test environments for the three research questions of *WALL-E: A Controlled Evaluation of
Motion Control, Heading Estimation, and Object Ranging for a Low-Cost Inspection Rover*
(draft Section IV-A). Use them to try a new firmware, agent, or tool in simulation first,
then on the robot.

| RQ | Comparison | World | Harness command |
|---|---|---|---|
| RQ1 | A: calibrated timed 50 cm vs B: encoder-targeted stop | `rq1_lane` or `motion_area` | `rq1`, `motion --only straight` |
| RQ2 | wheel-only vs EKF heading on the same ~90° open-loop turn | `rq2_protractor` or `motion_area` | `rq2`, `motion --only turn` |
| RQ3 | HC-SR04 vs calibrated monocular depth, 4 objects × 8 distances × 2 angles | `rq3_ranging`, `rq3_ranging_dim` | `rq3`, `range`, `calibrate-depth` |
| agents | ROSA tools (look, approach, search_for, …) | `inspection_room` | the agents in `walle-pilot/` |

## How it works

The simulated robot speaks exactly the robot's interface, so the code under test is the
real code, not a copy:

```
pilot scripts, ROSA tools ──/pico/cmd_vel──► sim_pico_bridge.py ──► Gazebo (Module 6 robot)
   ▲    ▲                                     firmware model (firmware_models.py)
   │    └──/wheel/odom, /odometry/filtered── walle-pilot/launch/pilot.launch.py (odometry + EKF)
   │                                              ▲ /pico/wheel_ticks [l, r, ping_seq, echo_us]
   │                                              ▲ /pico/imu/data, /pico/drive/status
   └── http://localhost:8080/capture ── sim_camera_server.py ◄── Gazebo camera
harness/sim_trials.py: places the robot and objects, reads ground truth (/sim/ground_truth)
```

- **`nodes/sim_pico_bridge.py`** replaces the Pico. Wheel joint angles become encoder counts
  (4320 per turn), Gazebo's gyro becomes a BNO055-style heading, a cone of lidar rays becomes
  HC-SR04 echo times (`-1` timeout, `-2` no response), and the 500 ms watchdog and drive status
  behave like the firmware.
- **`nodes/firmware_models.py`** is where firmware logic lives in simulation. `open_loop_pwm`
  is a line-for-line copy of `firmware/pico_pilot/drive_control.c`. It reproduces the pilot's
  two surprises: a 0.15 m/s command really drives at about 0.27 m/s, and a 2.62 s turn
  overshoots by a full rotation. `pid` is a closed-loop candidate you can test before
  writing it in C.
- **`nodes/sim_camera_server.py`** stands in for the ESP32-CAM, so `range_trial.Camera`,
  `session_check.py`, and `camera_node.py` download photos from a URL just as on the robot.
- **`harness/sim_trials.py`** takes the operator's and the measurer's jobs. It places the
  robot on the lane or protractor (with a little alignment error), places objects on the
  rays, and records ground truth from Gazebo. It calls the pilot's own
  `motion_trial.run_command`, `range_trial.read_ultrasonic`, and `range_trial.Camera`, and
  writes the same CSV columns. `pilot/analyze.py` and `pilot/suggest_durations.py` then run on
  simulated data unchanged.
- **`config/sim_pilot.yaml`** is the simulated robot's `pilot.yaml`. The protocol settings
  are the same; only the simulated hardware's geometry, durations, camera URL and field of
  view differ. The container sets `WALLE_CONFIG`, `WALLE_DATA_DIR` and
  `WALLE_DEPTH_CALIBRATION`. With those unset, the pilot scripts behave exactly as before.

**Isolation from the robot.** The simulator runs in its own Compose project, on ROS domain
42, with no micro-ROS Agent or UDP 8888, and writes only to `walle-sim/data/<run>/`
(git-ignored). You can run it next to `make up` without simulated `/pico/*` topics reaching
a real session. Simulated runs are never study data.

**Clocks.** The robot-side stack (the bridge, odometry, the EKF, the scripts) runs on the
computer's clock, as on the real robot. Only physics runs on simulation time. Keep the
real-time factor near 1: in testing it was 1.00 headless and about 0.9 with the Gazebo
window open, so run trials with `GUI=false`. `check` prints it, and every trial logs it.

**What the default profile reproduces.** `pilot_firmware` copies the current
`drive_control.c`. In testing it showed the pilot's own patterns: a 0.15 m/s command driving
at 0.27 m/s, strategy B stopping short (about 48.6 cm), and a slight drift to the right. The
numbers are not the robot's (see the end of this file).

## Quick start

From the repository root (the first build takes a while):

```bash
make sim-up
```

In one terminal, start a world. The Gazebo window is at http://localhost:6080/vnc.html:

```bash
make sim-launch WORLD=motion_area
```

In another terminal, open a shell and check that everything publishes:

```bash
make sim-shell
```

```bash
python3 /workspace/walle-sim/harness/sim_trials.py --run base check
```

`H=/workspace/walle-sim/harness/sim_trials.py` shortens the commands below.

### RQ1 and RQ2

Calibrate in simulation first, exactly as on the robot (draft IV-C):

```bash
python3 $H --run base shakedown straight-A --runs 5
python3 $H --run base shakedown straight-A --runs 5 --duration 2.0
python3 $H --run base shakedown straight-B --runs 5
python3 $H --run base shakedown turn-ccw --runs 5
python3 $H --run base shakedown turn-cw --runs 5
python3 $H --run base suggest
```

Copy the suggested `timed_duration_s`, `turn_duration_s`, and `targeted_coast_time_s` into
**`walle-sim/config/sim_pilot.yaml`**, never into the robot's `pilot.yaml`. Then:

```bash
python3 $H --run base rq1 --blocks 15
python3 $H --run base rq2 --turns 16
python3 $H --run base analyze
```

To run the frozen seed-42 schedule instead (warm-ups included, RQ1 and RQ2 interleaved),
use `motion --session 1`, then `2`, then `3` in `motion_area`. The schedule is generated into
the run folder the first time.

### RQ3

Start the ranging room:

```bash
make sim-launch WORLD=rq3_ranging GUI=false
```

Then calibrate the depth model on the red ball and run the 64-placement grid:

```bash
python3 $H --run base calibrate-depth
python3 $H --run base rq3
python3 $H --run base analyze
```

`calibrate-depth` places the red ball at 8 distances that are not test distances, then
fits the calibration with `depth/fit_depth.py` into `walle-sim/config/depth_calibration.json`.
`--no-camera` runs the ultrasonic only, which is fast and needs no depth model.
`range --session N` runs the frozen schedule. `rq3_ranging_dim` is the same room with a
quarter of the light, to stress the color masks.

Results land in `walle-sim/data/base/results/` (`pilot_summary.md` and the figures).
`logs/sim_truth.csv` adds what only a simulator knows: exact poses, the unwrapped turn
(it flags a full rotation a protractor cannot show), the largest lateral excursion, and
the real-time factor.

## Testing a change

| You changed | Do this |
|---|---|
| **Firmware** (`firmware/pico_pilot/*.c`) | Mirror the logic in a `FirmwareModel` subclass in `nodes/firmware_models.py`, add a profile in `config/sim_profiles.yaml`, and compare runs: `make sim-launch PROFILE=pilot_firmware` + `--run before`, then `PROFILE=my_firmware` + `--run after`. |
| **Host code** (odometry, EKF config, `motion_trial.py`, `range_trial.py`, depth) | Nothing to mirror: the sim runs your files directly. Rerun the same commands into a new `--run` and compare the summaries. |
| **An agent or a tool** (ROSA) | `make sim-launch WORLD=inspection_room`. The sim publishes the robot's topics, so the motion and ultrasonic tools work as they are. For camera tools, start `launch/vision.launch.py` or `depth.launch.py` with `camera_url:=http://localhost:8080/capture` (or `sim:=true`). Ollama on the host is reachable at `host.docker.internal`. |
| **Hardware assumptions** | Edit or add a profile: motor mismatch, encoder scale, IMU bias, ultrasonic dropouts (`ultrasonic_dead` reproduces the pilot's RQ3 failure), Wi-Fi stalls, or operator alignment error. `stress` combines several. |

Each run folder's `sim_runs.jsonl` records the world, the full profile, the config, and the
commit for every command, so a comparison is reproducible.

## Files

```
docker/                 Gazebo + noVNC + pilot dependencies image (docker-compose.sim.yml)
urdf/gazebo_car.urdf.xacro   Module 6 robot, verbatim (edit the robot here)
urdf/walle_sim.urdf.xacro    includes it and adds the HC-SR04 cone (the only addition)
config/sim_pilot.yaml   the simulated robot's pilot.yaml (WALLE_CONFIG)
config/sim_profiles.yaml   firmware and hardware profiles
config/bridge.yaml      Gazebo <-> ROS topics (raw sim topics live under /sim/*)
worlds/generate_worlds.py  writes every world + world_info.yaml (anchors, object sizes)
launch/sim.launch.py    world + robot + bridges + simulated Pico/camera + pilot.launch.py
nodes/                  sim_pico_bridge.py, firmware_models.py, sim_camera_server.py
harness/                sim_trials.py (protocol runner), sim_world.py (ground truth, set_pose)
data/<run>/             logs, schedule, results, photos of each run (git-ignored)
```

## Known differences from the robot

The robot model is Module 6's, unchanged on purpose. Make these model changes later to
bring the numbers closer to WALL-E's:

- **Geometry.** The sim uses 60 mm wheels and a 145 mm track. WALL-E has about 64.8 mm and
  127 mm. `sim_pilot.yaml` uses the model's values, so the odometry is calibrated to the sim.
- **Plugin limits.** The DiffDrive plugin caps commands at 0.3 m/s and 1.5 rad/s. The pilot
  firmware's turn needs about 2.7 rad/s on this geometry, so sim turns are slower than real
  ones, and `T_psi` differs (the bridge warns once).
- **Sensor heights.** The camera is about 10 cm above the floor (8.26 cm on the robot), and
  the HC-SR04 is 3 cm below it. The camera is 320×240 with a 60° field of view (OV3660: about
  66°).
- **Ultrasonic physics.** The cone has 21×13 rays over ±15°, taking the nearest return
  (`ultrasonic_half_beam` in `urdf/walle_sim.urdf.xacro`).
  Grazing floor returns are dropped. Specular misses on angled surfaces are not modeled; use
  `timeout_prob` to inject them. The short yellow wrapper falls between rays at the far
  distances, much like the real geometry challenge.
- **Objects.** These are simple shapes: cylinders for the cup, bottle, and bin, and spheres
  for the wrapper and the ball. Their nearest surface on any ray is exactly one radius from
  the center, which keeps the reference range exact. Rendered flat colors are easier for the
  color mask than real objects.
- **Depth model.** Depth Anything V2 runs on flat-shaded renders and reads them badly. In
  testing, 0.35 m came out as about 1.3 m, and `fit_depth.py` rejected the fit as outside its
  allowed range, so no calibration was saved. In simulation, RQ3's camera arm tests the
  pipeline (capture, masks, bearing, timing, failure logging, the analysis), not the depth
  accuracy. Textured floors and walls would make the renders more realistic. A sim
  calibration says nothing about the real camera, so the two calibration files are kept
  apart.
- **Moving objects.** The harness moves the RQ3 objects as static models with Gazebo's
  `set_pose`. The reference range comes from the pose Gazebo reports back (`truth=gt` in the
  row's notes); if that ever lags, it falls back to the commanded position (`truth=commanded`).
- **Determinism.** Physics adds no noise of its own. Run-to-run spread comes only from the
  profile (placement error, sensor noise, dropouts, Wi-Fi), so simulated SDs say nothing
  about the robot's. A rare wall-clock hiccup can shorten a timed run. Such trials carry
  `SIM_TIMING_GLITCH` in their notes and `timing_glitch` in `sim_truth.csv`.
