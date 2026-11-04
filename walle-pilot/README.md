# WALL-E pilot

Code for the pilot run of *WALL-E: A Controlled Evaluation of Motion Control, Heading
Estimation, and Object Ranging for a Low-Cost Inspection Rover*. Everything here follows the
experimental protocol in Section IV of the draft:

| RQ | Comparison | Script |
|---|---|---|
| RQ1 (primary) | A: calibrated timed 50 cm drive vs B: encoder-targeted stop | `pilot/motion_trial.py` |
| RQ2 | wheel-only vs EKF heading on the same open-loop ~90° turn | `pilot/motion_trial.py` |
| RQ3 (exploratory) | HC-SR04 vs Depth Anything V2 (+ calibration) on the same placement | `pilot/range_trial.py` |

Fixed scripts issue every command; the LLM agent is not involved.

The pilot is a rehearsal of the main study. Its results are labeled PILOT and are not
pooled with main-study data.

## What is in this folder

```
config/pilot.yaml           every frozen setting (geometry, speeds, durations, ranging, tolerances)
config/ekf.yaml             robot_localization EKF (copied from Module 8)
config/depth_calibration.json   written by depth/fit_depth.py
launch/pilot.launch.py      wheel odometry + IMU mount + EKF (fake:=true for rehearsal)
firmware/pico_pilot/        Pico 2 W firmware: Module 8 + HC-SR04 on GP6/GP7
firmware/esp32cam_camera/   ESP32-CAM photo server (copied from Module 8)
odometry/                   wheel odometry node (copied from Module 8)
depth/                      Depth Anything V2 wrapper, calibration fit, model download
pilot/make_schedule.py      frozen schedule from seed 42
pilot/session_check.py      start-of-session checklist + configuration manifest
pilot/motion_trial.py       RQ1/RQ2 trials and shakedown runs
pilot/suggest_durations.py  T_A, T_psi, and coast-time suggestions from shakedown runs
pilot/range_trial.py        RQ3 placements, depth calibration, field-of-view check
pilot/analyze.py            the planned analysis -> data/results/pilot_summary.md
tools/fake_pico.py          simulated robot for rehearsing without hardware
rosa_agent/                 ROSA agent and custom tools (not used in the experiments)
data/                       schedules, logs, bags, photos, results (see "Data files")
```

## One-time setup

### 1. Wiring: add the HC-SR04

| HC-SR04 | Connect to |
|---|---|
| VCC | 5 V rail (BreadVolt) |
| GND | common ground |
| TRIG | Pico GP6 |
| ECHO | 1 kΩ in series to Pico GP7, and 2 kΩ from GP7 to GND |

**ECHO is a 5 V output and the Pico's pins are 3.3 V only, so never wire ECHO directly to GP7.**

Mount the sensor directly below the camera. Its transducer face should sit in the camera
lens's vertical plane, or record the offset in `ranging.ultrasonic_offset_m`. The existing
Module 8 wiring is unchanged: encoders on GP18–21, BNO055 on GP0/GP1, TB6612 on GP2–5 and
GP10–12.

### 2. Firmware

1. Copy `firmware/pico_pilot/wifi_config.example.h` to `wifi_config.h` and fill it in. Use
   the computer's IP as `ROS_AGENT_IP`. Do the same for `firmware/esp32cam_camera/`. The real
   `wifi_config.h` files are git-ignored.
2. Build `firmware/pico_pilot` with the Raspberry Pi Pico VS Code extension, the same way as
   in Module 8. Then flash `build/pico_pilot.uf2`. The micro-ROS library is included, in
   `libmicroros/`.
3. The ESP32-CAM sketch is unchanged from Module 8. Reflash it only if you need to.

The firmware publishes `/pico/wheel_ticks` as `[left, right, ping_seq, echo_us]`. The ping
data shares that message because the micro-ROS library allows only 3 publishers. A new ping
starts every 60 ms. `echo_us` is -1 for an echo timeout and -2 when the sensor does not
respond. The USB serial log prints the ping every second.

### 3. Containers

From the `ollama/` folder, after stopping Module 8's containers (they also use UDP 8888):

```bash
docker compose -f module-8-sensor-fusion/docker-compose.yml down
```

```bash
docker compose --profile pilot up -d --build pilot micro-ros-agent
```

```bash
docker exec -it walle-pilot bash
```

Every command below runs inside that shell, from `/workspace/walle-pilot`. Open a second
shell (`docker exec -it walle-pilot bash`) and leave the launch file running in it:

```bash
ros2 launch launch/pilot.launch.py
```

Download the depth model once (99 MB, saved in `depth/hf_cache`):

```bash
python3 depth/download_model.py
```

### Rehearse without the robot

`ros2 launch launch/pilot.launch.py fake:=true` starts `tools/fake_pico.py` instead of using
the Pico. You can then run every motion script end to end and practice the prompts. Fake
runs are not data. Delete `data/logs/*` (keep the two templates) and the bags before the real
pilot.

## Before session 1 (calibration, then freeze)

1. **Measure the geometry** (draft III-A). Measure the wheel diameter and the wheel-center
   separation, and turn each wheel exactly one revolution to check the 4320 counts. Put the
   values in `geometry:` in `config/pilot.yaml`. They feed the odometry, the EKF, and the
   RQ1-B stopping rule.
2. **Make the schedule** (seed 42; refuses to overwrite):
   ```bash
   python3 pilot/make_schedule.py
   ```
3. **Ground-truth repeatability.** Each measurer measures five representative poses five
   times. Record them in `data/logs/gt_repeatability.csv` with columns `measurer, quantity,
   pose_id, repeat, value`. `quantity` is one of `forward_cm`, `lateral_cm`, `turn_deg` or
   `range_m`.
4. **Shakedown, at least 5 runs of each command.** These runs never enter the comparison.
   ```bash
   python3 pilot/motion_trial.py --shakedown straight-A
   ```
   Also run `straight-B`, `turn-ccw` and `turn-cw`. Try other durations with
   `--duration 3.5`. Then run:
   ```bash
   python3 pilot/suggest_durations.py
   ```
   Copy the suggested `timed_duration_s`, `turn_duration_s` and `targeted_coast_time_s` into
   `config/pilot.yaml`, and confirm them with a few more shakedown runs. With six fresh AAs,
   also check that the 0.15 m/s timing still holds.
5. **Camera field of view (OV3660).** Put the red ball on the 20° ray, then on the −20° ray,
   and run this each time:
   ```bash
   python3 pilot/range_trial.py --hfov-check
   ```
   Set `ranging.hfov_deg` to the average. Also set `ranging.camera_url`.
6. **Depth calibration.** Use the red ball on the 0° ray at 8 distances between about 0.35
   and 1.85 m, none of which are test distances:
   ```bash
   python3 pilot/range_trial.py --calibrate
   ```
   ```bash
   python3 depth/fit_depth.py
   ```
   `fit_depth.py` chooses scale-only or scale + power by leave-one-out error. It writes
   `config/depth_calibration.json`.
7. **Check that both sensors respond:**
   ```bash
   python3 pilot/range_trial.py --check red
   ```
8. **Freeze.** Commit `config/` and `data/schedule/` to the `pilot-run` branch. Every log row
   records the commit plus a hash of the frozen files (`cfg-…`). A change after this point is
   therefore visible, and must be labeled as a post hoc amendment.

## Each session (1, 2, 3)

```bash
python3 pilot/session_check.py --session 1
```

```bash
python3 pilot/motion_trial.py --session 1
```

```bash
python3 pilot/range_trial.py --session 1
```

- **`session_check.py`** asks for the supply voltages, the ground offset (must be under
  0.05 V), the TB6612 VCC/STBY reading (must be at least 2.7 V), the e-stop test, the
  temperature, the lux reading and the floor. It holds the robot still for the IMU, then
  measures the topic rates, the ultrasonic pings and the camera. It writes
  `data/manifests/session_N.json`.
- **`motion_trial.py`** runs the session's schedule in order. It starts with the 2 warm-ups,
  which are excluded in advance. Then come the RQ1 blocks (A and B back to back, randomized
  order) and the RQ2 turns, interleaved. For each trial it:
  - waits for you to place the robot;
  - resets both estimates and records a bag;
  - drives, then waits for rest;
  - asks the measurer for the ground truth before showing any estimate. It never shows the
    A/B label before the truth is saved.

  Ctrl+C while the robot is moving is the emergency stop.
- **`range_trial.py`** shows where to put each object. You type the true range (horizontal,
  from the shared reference plane to the object's nearest surface). It then reads both
  sensors in the scheduled order.

**Failures and invalid trials**, as in draft IV-E:
- **Robot failures stay in the data.** These are lane exits, collisions, stalls, emergency
  stops, timeouts, ultrasonic timeouts and missing masks. A measurable endpoint is kept even
  outside tolerance. An unmeasurable one is left blank, never set to 5 cm.
- **Outside interference** (someone bumped the robot, or the reference was lost): answer
  "y" to the outside-interference question. The script repeats that trial at the end of the
  session under the same condition.

**Double measurement:** for about 20 % of trials, chosen at random, the second partner
measures independently. Record their values in `data/logs/double_measure.csv`
(`trial_id, quantity, value, measurer`). `quantity` is a motion log column, such as
`true_forward_cm`.

## Analysis

```bash
python3 pilot/analyze.py
```

This writes `data/results/pilot_summary.md`, `pilot_summary.json` and the figures:

- **RQ1 and RQ2:** a paired one-sided t-test on complete pairs, an unadjusted 95 % t
  interval, Cohen's dz, an exact sign-flip sensitivity test, and Holm correction across RQ1
  and RQ2.
- **RQ3 (descriptive):** valid fraction, AE/APE on valid readings, common-valid paired
  differences, joint success, latency, and breakdowns by object, angle and distance.
- **Accounting:** attempted, failed, externally invalid and missing counts for every
  condition.

## Data files

| File | One row per | Written by |
|---|---|---|
| `data/schedule/motion_schedule.csv`, `range_schedule.csv` | scheduled trial / placement | `make_schedule.py` |
| `data/logs/session_log.csv` | session checklist | `session_check.py` |
| `data/logs/motion_log.csv` | motion trial attempt | `motion_trial.py` |
| `data/logs/shakedown_log.csv` | shakedown run | `motion_trial.py --shakedown` |
| `data/logs/range_log.csv` | placement attempt (per-ping status, timing, file names) | `range_trial.py` |
| `data/logs/depth_calibration_log.csv` | calibration photo | `range_trial.py --calibrate` |
| `data/logs/gt_repeatability.csv`, `double_measure.csv` | hand measurement | you |
| `data/manifests/session_N.json` | session: commit, config, hashes, model revision, versions | `session_check.py` |
| `data/bags/<trial>_a<attempt>/` | motion trial (ticks, IMU, odom, EKF, commands) | `motion_trial.py` |
| `data/range/<placement>_a<attempt>_*` | photo, mask, depth map (`.npy`), view | `range_trial.py` |

Logs are append-only, so never edit a row by hand. If something was entered wrong, write a
note in the notes column of a repeat, or in your lab notes. Bags are committed with the
logs. Photos and depth maps are git-ignored because they are large: back them up, and add
them to the reproducibility package deliberately.

## Choices to confirm against the protocol

- **RQ1-B sends zero angular command, like A** (`targeted_hold_heading: false`). The two
  strategies therefore differ only in the stopping rule and the speed ramp. Module 8's
  `move_test.py` also held the heading. Set the option to `true` if you want that, and say
  so in the paper.
- **The coast time starts at 0.2 s,** as in the draft. Module 8's `move_test.py` used
  0.06 s. Confirm it in shakedown.
- **Within a session, RQ1 blocks and RQ2 turns are interleaved at random.** The two warm-ups
  come first.
- **Each RQ1 block's A/B order is randomized but balanced overall** (8 AB, 7 BA). The RQ3
  first sensor is balanced 32/32.
- **Camera range = calibrated axial depth / cos(bearing).** The depth is the median over the
  color mask, eroded 2 pixels to skip the blurry edge. The model input is resized to
  364×364; `range_log.csv` records the tensor shape.
- **Ultrasonic range = median of the valid pings,** valid only with at least 3 of 5. The
  speed of sound is corrected with the session temperature.
- **IMU calibration status is inferred from the IMU message's covariance.** The firmware
  reports 100× the variance until the gyro is calibrated, and publishes nothing before its
  first calibration.
