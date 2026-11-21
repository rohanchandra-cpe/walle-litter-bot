"""Run the paper's RQ1, RQ2 and RQ3 protocols in Gazebo, with Gazebo as the measurer.

The robot side is the real pilot code: commands come from pilot/motion_trial.py
(run_command: timed A, encoder-targeted B, open-loop turns), ranging from
pilot/range_trial.py (read_ultrasonic, Camera), estimates from wheel odometry and the
EKF that launch/sim.launch.py starts. Only the people are replaced: this script
places the robot and objects (with a little alignment error) and reads the ground
truth from Gazebo, where the measurer would use the tape and the protractor. Rows go
into the same CSV logs, with the same columns, so pilot/analyze.py and
pilot/suggest_durations.py run on simulated data unchanged.

Every run lives in its own folder, walle-sim/data/<run>/ (logs, schedule, results,
photos), so firmware or tool variants can be compared side by side and nothing ever
mixes with walle-pilot/data/.

Run inside the sim container while launch/sim.launch.py is running:
    python3 /workspace/walle-sim/harness/sim_trials.py check
    python3 /workspace/walle-sim/harness/sim_trials.py --run base shakedown straight-A --runs 5
    python3 /workspace/walle-sim/harness/sim_trials.py --run base suggest
    python3 /workspace/walle-sim/harness/sim_trials.py --run base rq1 --blocks 15
    python3 /workspace/walle-sim/harness/sim_trials.py --run base rq2 --turns 16
    python3 /workspace/walle-sim/harness/sim_trials.py --run base motion --session 1   # frozen schedule
    python3 /workspace/walle-sim/harness/sim_trials.py --run base calibrate-depth      # world rq3_ranging
    python3 /workspace/walle-sim/harness/sim_trials.py --run base rq3                  # all 64 placements
    python3 /workspace/walle-sim/harness/sim_trials.py --run base range --session 1    # frozen schedule
    python3 /workspace/walle-sim/harness/sim_trials.py --run base analyze
    python3 /workspace/walle-sim/harness/sim_trials.py park-check --look-at bin_2 obstacle  # world park
    python3 /workspace/walle-sim/harness/sim_trials.py --run route route-check --laps 1   # world park

Simulated runs are never study data. Their logs say so (session "sim" or the
schedule's session, a SIM note on every row, and sim_runs.jsonl beside them).
"""

import argparse
import json
import math
import os
import random
import subprocess
import sys
import time

SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_DIR = os.path.dirname(SIM_DIR)
PILOT_DIR = os.path.join(REPO_DIR, "walle-pilot")
SIM_CONFIG = os.path.join(SIM_DIR, "config", "sim_pilot.yaml")
SELF = os.path.abspath(__file__)
SIM_DEPTH_CALIBRATION = os.path.join(SIM_DIR, "config", "depth_calibration.json")
# Depth calibration distances: 0.35-1.85 m, none of them a test distance (draft IV-C)
CALIBRATION_DISTANCES = [0.35, 0.55, 0.70, 0.90, 1.10, 1.35, 1.60, 1.85]
# Everything only a simulator knows, one row per motion trial (logs/sim_truth.csv).
# gt_* is measured from the marks, as logged in motion_log.csv; move_* is the motion
# from where the robot actually stood; place_err_* is the operator's placement error.
SIM_TRUTH_FIELDS = ["trial_id", "attempt", "timestamp_utc", "world", "profile", "rtf",
                    "gt_start_x", "gt_start_y", "gt_start_yaw_deg", "gt_end_x", "gt_end_y",
                    "gt_end_yaw_deg", "place_err_forward_cm", "place_err_lateral_cm",
                    "place_err_yaw_deg", "gt_forward_cm", "gt_lateral_cm",
                    "move_forward_cm", "move_lateral_cm", "gt_turn_unwrapped_deg",
                    "gt_turn_wrapped_deg", "move_turn_deg", "lane_exit", "max_abs_lateral_cm",
                    "timing_glitch"]


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", help="run name: data go to walle-sim/data/<run>/ "
                        "(default: $WALLE_DATA_DIR, else 'default')")
    parser.add_argument("--seed", type=int, default=42, help="seed for ad hoc orders and placement noise")
    parser.add_argument("--exact-truth", action="store_true",
                        help="log Gazebo's exact ground truth instead of rounding it to the "
                        "tape's 1 mm and the protractor's 1 deg")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("check", help="is everything publishing? (like session_check.py)")
    p = sub.add_parser("shakedown", help="tuning runs -> shakedown_log.csv")
    p.add_argument("task", choices=["straight-A", "straight-B", "turn-ccw", "turn-cw"])
    p.add_argument("--runs", type=int, default=5)
    p.add_argument("--duration", type=float, help="try this command duration (s)")
    sub.add_parser("suggest", help="pilot/suggest_durations.py on this run's shakedown log")
    p = sub.add_parser("rq1", help="ad hoc RQ1 blocks (A and B back to back, balanced order)")
    p.add_argument("--blocks", type=int, default=15)
    p = sub.add_parser("rq2", help="ad hoc RQ2 turns (balanced CCW/CW)")
    p.add_argument("--turns", type=int, default=16)
    p = sub.add_parser("motion", help="a session's scheduled motion trials (warm-ups included)")
    p.add_argument("--session", type=int, required=True)
    p.add_argument("--only", choices=["straight", "turn"], help="skip the other task")
    p = sub.add_parser("rq3", help="ad hoc RQ3 grid: every object x distance x angle")
    p.add_argument("--objects", nargs="+", help="subset of ranging.objects")
    p.add_argument("--no-camera", action="store_true", help="ultrasonic only (no depth model)")
    p = sub.add_parser("range", help="a session's scheduled RQ3 placements")
    p.add_argument("--session", type=int, required=True)
    p.add_argument("--no-camera", action="store_true")
    sub.add_parser("calibrate-depth", help="red ball at 8 distances, then depth/fit_depth.py")
    sub.add_parser("analyze", help="pilot/analyze.py on this run")
    p = sub.add_parser("park-check", help="world park: is every bin, item and obstacle where "
                       "the scenario put it? Saves robot-camera photos")
    p.add_argument("--look-at", nargs="*", default=[], metavar="TARGET",
                   help="bin_1 ... bin_4 or obstacle: put the robot facing each and save a "
                   "photo (then back to the start)")
    p = sub.add_parser("route-check", help="world park: run walle-app's route follower "
                       "from the start pose and judge it against Gazebo's ground truth")
    p.add_argument("--laps", type=int, default=1)
    p.add_argument("--tolerance-cm", type=float, default=5.0,
                   help="pass if the robot ends within this of the start (Phase 1: 5 cm)")
    p.add_argument("--dwell", type=float, help="seconds at each bin (default route.yaml)")
    p.add_argument("--feedback", help="/odometry/filtered (default) or /wheel/odom")
    p.add_argument("--coast-time", type=float, help="start both coasts at this (s): a wrong "
                   "value makes turns miss, to exercise the correction pulses")
    p.add_argument("--misplace-deg", type=float, default=0.0,
                   help="Phase 3: tell the follower each bin is this far round its corner "
                   "from where it really is (alternating sides), so it must center on the "
                   "bin it detects")
    p.add_argument("--hide-cut-bins", action="store_true",
                   help="Phase 3: the ground-truth detector misses bins cut off at the frame's "
                   "side (as on the robot), so centering must search")
    p.add_argument("--blackout", type=float, nargs=2, metavar=("EVERY_S", "FOR_S"),
                   help="sensor-guard test: every EVERY_S s drop all ticks and IMU for FOR_S s "
                   "(/sim/blackout)")
    p.add_argument("--center-tol-deg", type=float, default=3.0,
                   help="Phase 3: a centered bin is within this of straight ahead (truth)")
    for name in ("shakedown", "rq1", "rq2", "motion"):
        sub.choices[name].add_argument("--bag", action="store_true", help="record ros2 bags")
    return parser.parse_args()


ARGS = parse_args()
RUN_NAME = ARGS.run or os.path.basename(os.environ.get("WALLE_DATA_DIR", "default").rstrip("/"))
RUN_DIR = os.path.join(SIM_DIR, "data", RUN_NAME)
# Point the pilot code at the simulated robot's config and this run's folder BEFORE
# importing it (common.py reads these once)
os.environ["WALLE_DATA_DIR"] = RUN_DIR
os.environ.setdefault("WALLE_CONFIG", SIM_CONFIG)
os.environ.setdefault("WALLE_DEPTH_CALIBRATION", SIM_DEPTH_CALIBRATION)
sys.path[:0] = [os.path.join(PILOT_DIR, "pilot"), os.path.dirname(os.path.abspath(__file__))]

import common  # noqa: E402
import motion_trial  # noqa: E402
import robot_io  # noqa: E402
from common import (append_row, commit_label, load_config, next_scheduled, read_rows,  # noqa: E402
                    utc_now, wrap_deg)
from sim_world import (REST_Z, ROBOT, GroundTruth, World, lane_exit,  # noqa: E402
                       wait_new_frames)


def pilot_env():
    return dict(os.environ)


def run_pilot_script(script, *args):
    """Run a walle-pilot script on this run's data (same env), e.g. analyze.py."""
    return subprocess.call([sys.executable, os.path.join(PILOT_DIR, *script.split("/")), *args],
                           cwd=PILOT_DIR, env=pilot_env())


# ===== connection =====
class Sim:
    """The pilot's RobotIO node plus the ground-truth node, spinning together."""

    def __enter__(self):
        self.ctx = robot_io.Robot()
        self.robot = self.ctx.__enter__()
        self.gt = GroundTruth()
        self.ctx.executor.add_node(self.gt)
        if not self.gt.wait_ready((ROBOT,), timeout=20.0):
            self.__exit__(None, None, None)
            raise SystemExit("No ground truth on /sim/ground_truth. Is launch/sim.launch.py "
                             "running, and the robot spawned?")
        self.world = World(self.gt)
        end = time.monotonic() + 3.0
        while self.gt.profile is None and time.monotonic() < end:
            time.sleep(0.1)
        self.profile = self.gt.profile or {}
        return self

    def __exit__(self, *exc):
        try:
            self.ctx.executor.remove_node(self.gt)
            self.gt.destroy_node()
        finally:
            self.ctx.__exit__(*exc)
        return False

    def wait_robot(self, keys=("ticks", "wheel", "ekf", "imu")):
        missing = self.robot.wait_for(keys, timeout=15.0)
        if missing:
            raise SystemExit(f"No data from: {', '.join(missing)}. Is launch/sim.launch.py "
                             "running with pilot:=true?")

    def placement(self):
        return self.profile.get("placement", {"position_std_m": 0.0, "yaw_std_deg": 0.0})

    def describe(self, rtf=None):
        rtf = self.gt.rtf() if rtf is None else rtf
        return (f"SIM world={self.world.name} profile={self.profile.get('name', '?')}"
                + (f" rtf={rtf:.2f}" if rtf else ""))


def log_invocation(sim, extra=None):
    os.makedirs(RUN_DIR, exist_ok=True)
    entry = {"timestamp_utc": utc_now(), "argv": sys.argv[1:], "commit": commit_label(),
             "world": sim.world.name if sim else None,
             "profile": sim.profile if sim else None, "config": common.CONFIG_FILE}
    entry.update(extra or {})
    with open(os.path.join(RUN_DIR, "sim_runs.jsonl"), "a") as f:
        f.write(json.dumps(entry) + "\n")


def warn_rtf(sim):
    rtf = sim.gt.rtf()
    if rtf is not None and rtf < 0.9:
        print(f"WARNING: Gazebo runs at {rtf:.2f}x real time. Timed commands are timed by the "
              "computer's clock, as on the robot, so a slow simulation covers less ground. "
              "Launch with gui:=false, or calibrate durations at this speed.")
    return rtf


# ===== RQ1 / RQ2: one motion trial with automatic ground truth =====
def measure(value, step):
    return value if ARGS.exact_truth else round(value / step) * step


def run_motion(sim, trial, attempt, cfg, log_path, duration=None, bag=False):
    robot, gt, world = sim.robot, sim.gt, sim.world
    motion = cfg["motion"]
    straight = trial["task"] == "straight"
    if duration is None:
        duration = motion["timed_duration_s"] if straight else motion["turn_duration_s"]
    targeted = straight and trial["strategy"] == "B"
    label = trial["task"] + (f" {trial['strategy']}" if straight else f" {trial['direction']}")
    anchor_key = "lane_start" if straight else "protractor"

    # The operator: place the robot on the start mark
    robot.stop()
    world.place_robot(anchor_key, sim.placement())
    time.sleep(1.0)
    sim.wait_robot(("ticks", "wheel", "ekf"))
    robot.reset_estimates()
    time.sleep(1.0)
    starts = {k: robot.pose(k) for k in ("wheel", "ekf")}
    ticks0 = robot.latest_ticks()
    gt_start = gt.pose(ROBOT)
    yaw0 = gt.heading_unwrapped()
    bag_proc, bag_name = (motion_trial.start_bag(f"{trial['trial_id']}_a{attempt}")
                          if bag else (None, ""))

    gt.start_track()
    robot.start_tick_recording()
    try:
        stop_reason, completion = motion_trial.run_command(robot, trial, motion, duration)
    except KeyboardInterrupt:
        robot.stop()
        stop_reason, completion = "emergency_stop", float("nan")
        print("\n  EMERGENCY STOP: robot stopped.")
    time.sleep(motion["settle_s"])
    tick_msgs, max_gap = robot.stop_tick_recording()
    track = gt.stop_track()
    ends = {k: robot.pose(k) for k in ("wheel", "ekf")}
    ticks1 = robot.latest_ticks()
    gt_end = gt.pose(ROBOT)
    turn_unwrapped = math.degrees(gt.heading_unwrapped() - yaw0)
    motion_trial.stop_bag(bag_proc)
    rtf = gt.rtf(seconds=max(2.0, (completion or 0) + motion["settle_s"]))

    # The measurer: tape and protractor, from Gazebo. Like the real measurer, measure from
    # the MARKS (start line and centerline, protractor 0 deg), so the operator's placement
    # error is part of the reference, while the robot's estimates start from wherever it
    # actually stood (draft IV-D).
    anchor = world.anchor(anchor_key)
    marks = (anchor[0], anchor[1], math.radians(anchor[2]))
    start3 = (gt_start[0], gt_start[1], gt_start[3])
    end3 = (gt_end[0], gt_end[1], gt_end[3])
    fwd_m, lat_m, _ = robot_io.local_change(marks, end3)            # logged: from the marks
    move_fwd_m, move_lat_m, _ = robot_io.local_change(start3, end3)   # the motion itself
    place_x, place_y, place_yaw = robot_io.local_change(marks, start3)
    turn_marks = turn_unwrapped + place_yaw                           # pointer read vs 0 deg
    turn_wrapped = wrap_deg(turn_marks)
    exited = straight and lane_exit(track, anchor, world.info["lane"])
    max_lat = 0.0
    for x, y, _ in track:
        _, lat, _ = robot_io.local_change(marks, (x, y, 0.0))
        max_lat = max(max_lat, abs(lat))
    glitch = (max_gap == max_gap and max_gap > 50.0) or (rtf is not None and rtf < 0.97)

    gate = ""
    if stop_reason in ("timeout", "emergency_stop"):
        gate = stop_reason
    elif exited:
        gate = "lane_exit"
    elif (straight and move_fwd_m < 0.02) or (not straight and abs(turn_unwrapped) < 5.0):
        gate = "stall"
    truth = {"completed": gate == "", "gate_violation": gate,
             "failure_reason": f"sim: {gate} detected from ground truth" if gate else "",
             "endpoint_measured": True, "external_invalid": False, "invalid_reason": ""}
    if straight:
        truth.update(true_forward_cm=measure(100 * fwd_m, 0.1),
                     true_lateral_cm=measure(100 * lat_m, 0.1))
    else:
        truth["true_turn_deg"] = measure(turn_wrapped, 1.0)
    note = sim.describe(rtf)
    if not straight:
        note += f" gt_turn_unwrapped={turn_marks:.1f}"
        if abs(turn_marks - turn_wrapped) > 1.0:
            note += " FULL_ROTATION_HIDDEN"
    if glitch:
        note += f" SIM_TIMING_GLITCH(max_gap={max_gap:.0f}ms)"
    truth["notes"] = note

    row = {k: trial.get(k, "") for k in ("trial_id", "session_id", "task", "strategy",
                                         "direction", "block_id", "order_in_block", "warmup")}
    row.update(attempt=attempt, timestamp_utc=utc_now(), commit=commit_label(),
               operator="sim", measurer="gazebo", motor_supply_v="",
               imu_calibrated=robot.imu_calibrated,
               cmd_duration_s="" if targeted else duration,
               completion_s=completion, stop_reason=stop_reason,
               tick_msgs=tick_msgs, max_tick_gap_ms=max_gap,
               enc_left_delta=ticks1[0] - ticks0[0], enc_right_delta=ticks1[1] - ticks0[1],
               bag=bag_name, **truth)
    for key in ("wheel", "ekf"):
        forward, left, turn = robot_io.local_change(starts[key], ends[key])
        row.update({f"{key}_forward_m": forward, f"{key}_left_m": left, f"{key}_turn_deg": turn})
    append_row(log_path, motion_trial.FIELDS, row)
    append_row(os.path.join(common.LOG_DIR, "sim_truth.csv"), SIM_TRUTH_FIELDS, {
        "trial_id": trial["trial_id"], "attempt": attempt, "timestamp_utc": row["timestamp_utc"],
        "world": world.name, "profile": sim.profile.get("name", ""), "rtf": rtf,
        "gt_start_x": gt_start[0], "gt_start_y": gt_start[1],
        "gt_start_yaw_deg": math.degrees(gt_start[3]), "gt_end_x": gt_end[0],
        "gt_end_y": gt_end[1], "gt_end_yaw_deg": math.degrees(gt_end[3]),
        "place_err_forward_cm": 100 * place_x, "place_err_lateral_cm": 100 * place_y,
        "place_err_yaw_deg": place_yaw,
        "gt_forward_cm": 100 * fwd_m, "gt_lateral_cm": 100 * lat_m,
        "move_forward_cm": 100 * move_fwd_m, "move_lateral_cm": 100 * move_lat_m,
        "gt_turn_unwrapped_deg": turn_marks, "gt_turn_wrapped_deg": turn_wrapped,
        "move_turn_deg": turn_unwrapped,
        "lane_exit": exited, "max_abs_lateral_cm": 100 * max_lat, "timing_glitch": glitch})

    # Report, like motion_trial.show_result, plus the simulator's view
    tol = cfg["tolerances"]
    if straight:
        err = row["true_forward_cm"] - 100 * motion["straight_target_m"]
        ok = (abs(err) <= tol["distance_cm"] and abs(row["true_lateral_cm"]) <= tol["lateral_cm"]
              and truth["completed"])
        print(f"{trial['trial_id']:>12} {label:<12} true {row['true_forward_cm']:6.1f} cm "
              f"(err {err:+5.1f}, lat {row['true_lateral_cm']:+5.1f})  wheel "
              f"{100 * row['wheel_forward_m']:6.1f}  ekf {100 * row['ekf_forward_m']:6.1f}  "
              f"{'ok' if ok else 'OUT'} {gate}" + ("  [sim timing glitch]" if glitch else ""))
    else:
        we = wrap_deg(row["wheel_turn_deg"] - row["true_turn_deg"])
        ee = wrap_deg(row["ekf_turn_deg"] - row["true_turn_deg"])
        print(f"{trial['trial_id']:>12} {label:<12} true {row['true_turn_deg']:+7.1f} deg "
              f"(unwrapped {turn_marks:+7.1f})  wheel err {we:+6.1f}  ekf err {ee:+6.1f}  "
              f"{gate}")
        if abs(turn_marks - turn_wrapped) > 1.0:
            print("             WARNING: the robot turned a full extra rotation that a protractor "
                  "cannot show (draft IV-F). Check the duration against the encoder totals.")
    return row


def balanced(rng, items, n):
    labels = [items[i % len(items)] for i in range(n)]
    rng.shuffle(labels)
    return labels


def adhoc_trials(kind, n, rng):
    prefix = time.strftime("%m%d%H%M")
    trials = []
    if kind == "rq1":
        for b, order in enumerate(balanced(rng, ["AB", "BA"], n), start=1):
            for pos, strategy in enumerate(order, start=1):
                trials.append({"trial_id": f"SIM{prefix}-B{b:02d}{strategy}", "session_id": "sim",
                               "task": "straight", "strategy": strategy, "direction": "",
                               "block_id": f"SIM{prefix}-B{b:02d}", "order_in_block": pos,
                               "warmup": "0"})
    else:
        for t, direction in enumerate(balanced(rng, ["ccw", "cw"], n), start=1):
            trials.append({"trial_id": f"SIM{prefix}-T{t:02d}", "session_id": "sim",
                           "task": "turn", "strategy": "", "direction": direction,
                           "block_id": "", "order_in_block": "", "warmup": "0"})
    return trials


def ensure_schedule():
    if not os.path.exists(common.MOTION_SCHEDULE) or not os.path.exists(common.RANGE_SCHEDULE):
        print("No schedule in this run yet: generating it with pilot/make_schedule.py (seed 42).")
        if run_pilot_script("pilot/make_schedule.py") != 0:
            raise SystemExit("make_schedule.py failed.")


def cmd_motion(sim, cfg):
    warn_rtf(sim)
    sim.wait_robot()
    log_invocation(sim)
    if ARGS.command == "shakedown":
        task, detail = ARGS.task.split("-")
        for _ in range(ARGS.runs):
            count = len(read_rows(common.SHAKEDOWN_LOG)) + 1
            trial = {"trial_id": f"SD{count:03d}", "session_id": "shakedown", "task": task,
                     "strategy": detail if task == "straight" else "",
                     "direction": detail if task == "turn" else "", "warmup": "0"}
            run_motion(sim, trial, 1, cfg, common.SHAKEDOWN_LOG, ARGS.duration, ARGS.bag)
        print(f"\nNow: python3 {SELF} --run {RUN_NAME} suggest")
        return
    if ARGS.command in ("rq1", "rq2"):
        n = ARGS.blocks if ARGS.command == "rq1" else ARGS.turns
        for trial in adhoc_trials(ARGS.command, n, random.Random(ARGS.seed)):
            run_motion(sim, trial, 1, cfg, common.MOTION_LOG, bag=ARGS.bag)
    else:
        ensure_schedule()
        schedule = [r for r in read_rows(common.MOTION_SCHEDULE)
                    if not ARGS.only or r["task"] == ARGS.only]
        while True:
            trial, attempt = next_scheduled(ARGS.session, schedule, read_rows(common.MOTION_LOG),
                                            "trial_id")
            if trial is None:
                break
            run_motion(sim, trial, attempt, cfg, common.MOTION_LOG, bag=ARGS.bag)
    print(f"\nDone. Analyze: python3 {SELF} --run {RUN_NAME} analyze")


# ===== RQ3 =====
def place_target(sim, name, distance, angle):
    world = sim.world
    world.park_objects()
    commanded = world.place_object(name, distance, angle)
    time.sleep(0.5)
    if not wait_new_frames(count=3):
        print("  WARNING: no new camera frame after moving the object (is the camera server up?)")
    true_m, source = world.true_range(name, commanded)
    return true_m, source


def disabled_camera():
    return ({"cam_valid": False, "cam_failure": "camera_disabled"}, None, None, None)


def run_range(sim, camera, cfg, placements):
    import range_trial
    world = sim.world
    world.anchor("sensor_station")
    sim.robot.stop()
    world.place_robot("sensor_station")
    time.sleep(1.0)
    sim.wait_robot(("ticks", "ping"))
    temp_c = float(cfg["ranging"]["default_temp_c"])
    log_invocation(sim, {"camera": camera is not None})
    for placement, attempt in placements:
        name = placement["object"]
        true_m, source = place_target(sim, name, float(placement["distance_m"]),
                                      float(placement["angle_deg"]))
        order = (["ultrasonic", "camera"] if placement["first_sensor"] == "ultrasonic"
                 else ["camera", "ultrasonic"])
        results, outputs = {}, (None, None, None)
        for sensor in order:
            if sensor == "ultrasonic":
                results.update(range_trial.read_ultrasonic(sim.robot, cfg, temp_c))
            else:
                cam, *outputs = camera.read(placement["color"]) if camera else disabled_camera()
                results.update(cam)
        stem = f"{placement['placement_id']}_a{attempt}"
        files = range_trial.save_outputs(stem, *outputs)
        row = {k: placement[k] for k in ("placement_id", "session_id", "order", "object",
                                         "color", "distance_m", "angle_deg", "first_sensor")}
        row.update(attempt=attempt, timestamp_utc=utc_now(), commit=commit_label(),
                   operator="sim", measurer="gazebo", temp_c=temp_c,
                   true_range_m=measure(true_m, 0.001), external_invalid=False,
                   invalid_reason="", notes=f"{sim.describe()} truth={source}",
                   **results, **files, **(camera.describe() if camera else {}))
        append_row(common.RANGE_LOG, range_trial.FIELDS, row)
        line = [f"{placement['placement_id']:>12} {name:<15} {placement['distance_m']:>5} m "
                f"{placement['angle_deg']:>3} deg  true {row['true_range_m']:.3f}"]
        for key, label in (("us", "us"), ("cam", "cam")):
            if row.get(f"{key}_valid"):
                r = row[f"{key}_range_m"]
                line.append(f"{label} {r:.3f} ({100 * abs(r - true_m) / true_m:4.1f}%)")
            else:
                why = row.get("cam_failure") if key == "cam" else row.get("us_ping_status")
                line.append(f"{label} invalid ({why})")
        print("  ".join(line))
    sim.world.park_objects()


def make_camera(cfg, disabled):
    if disabled:
        return None
    import range_trial
    return range_trial.Camera(cfg)


def cmd_rq3(sim, cfg):
    camera = make_camera(cfg, ARGS.no_camera)
    if ARGS.command == "rq3":
        ranging = cfg["ranging"]
        objects = ARGS.objects or list(ranging["objects"])
        rng = random.Random(ARGS.seed)
        grid = [(o, d, a) for o in objects for d in ranging["distances_m"]
                for a in ranging["angles_deg"]]
        rng.shuffle(grid)
        firsts = balanced(rng, ["ultrasonic", "camera"], len(grid))
        prefix = time.strftime("%m%d%H%M")
        placements = [({"placement_id": f"SIM{prefix}-R{i:02d}", "session_id": "sim", "order": i,
                        "object": o, "color": ranging["objects"][o], "distance_m": d,
                        "angle_deg": a, "first_sensor": firsts[i - 1]}, 1)
                      for i, (o, d, a) in enumerate(grid, start=1)]
    else:
        ensure_schedule()
        schedule = read_rows(common.RANGE_SCHEDULE)
        placements = []
        done = read_rows(common.RANGE_LOG)
        while True:
            p, attempt = next_scheduled(ARGS.session, schedule, done, "placement_id")
            if p is None:
                break
            placements.append((p, attempt))
            done.append({"placement_id": p["placement_id"], "external_invalid": "0"})
    run_range(sim, camera, cfg, placements)
    print(f"\nDone. Analyze: python3 {SELF} --run {RUN_NAME} analyze")


def cmd_calibrate(sim, cfg):
    import range_trial   # puts walle-pilot/depth on the path
    import depth_estimator
    camera = range_trial.Camera(cfg)
    world = sim.world
    world.place_robot("sensor_station")
    time.sleep(1.0)
    log_invocation(sim)
    name = "red_ball"
    color = depth_estimator.parse_color(cfg["ranging"]["calibration_object"])
    for i, d in enumerate(CALIBRATION_DISTANCES, start=1):
        true_m, _ = place_target(sim, name, d, 0.0)
        cam, image, depth, mask = camera.read(color)
        stem = f"calib_{len(read_rows(common.DEPTH_CALIBRATION_LOG)) + 1:02d}"
        files = range_trial.save_outputs(stem, image, depth, mask)
        append_row(common.DEPTH_CALIBRATION_LOG, range_trial.CALIBRATION_FIELDS, {
            "timestamp_utc": utc_now(), "commit": commit_label(),
            "true_m": measure(true_m, 0.001),
            "target_found": "True" if cam["cam_valid"] else "False",
            "target_raw_m": cam.get("cam_raw_z_m", ""), "bearing_deg": cam.get("cam_bearing_deg", ""),
            "model": depth_estimator.MODELS[camera.size], "photo": files["photo"]})
        print(f"  {true_m:.3f} m: " + (f"model says {cam['cam_raw_z_m']:.3f} m" if cam["cam_valid"]
                                       else f"not found ({cam['cam_failure']})"))
    world.park_objects()
    print(f"\nFitting -> {os.environ['WALLE_DEPTH_CALIBRATION']}")
    run_pilot_script("depth/fit_depth.py")


def cmd_check(sim, cfg):
    print(f"World: {sim.world.name}   profile: {sim.profile.get('name', '(not published)')}   "
          f"firmware model: {sim.profile.get('firmware', '?')}")
    print(f"Config: {common.CONFIG_FILE}\nRun folder: {RUN_DIR}")
    rtf = warn_rtf(sim)
    print(f"Real-time factor: {rtf:.2f}" if rtf else "Real-time factor: unknown (no /clock)")
    rates = sim.robot.rates(3.0)
    for key, label in (("ticks", "/pico/wheel_ticks"), ("imu", "/pico/imu/data"),
                       ("wheel", "/wheel/odom"), ("ekf", "/odometry/filtered")):
        print(f"  [{'OK' if rates[key] > 20 else '--'}] {label:22s} {rates[key]:5.1f} Hz")
    ping = sim.robot.ping
    print(f"  [{'OK' if ping and ping[1] != -2 else '--'}] ultrasonic ping        "
          f"{'seq %d, echo %d us' % ping if ping else 'none'}")
    from sim_world import camera_frames
    frames = camera_frames()
    print(f"  [{'OK' if frames else '--'}] camera server          "
          f"{'%d frames' % frames if frames is not None else 'not reachable on :8080'}")
    print(f"  [{'OK' if sim.robot.imu_calibrated else '--'}] IMU calibrated")
    x, y, _, yaw = sim.gt.pose(ROBOT)
    print(f"  ground truth: robot at ({x:.3f}, {y:.3f}), heading {math.degrees(yaw):.1f} deg")
    anchors = ", ".join(sim.world.info.get("anchors", {}))
    print(f"  this world supports: {anchors}")


PARK_TOLERANCE_M = 0.01
BIN_PHOTO_DISTANCE_M = 0.65   # robot axle to bin center for park-check photos


def park_expected(layout):
    """Every model the scenario should have put in the world: name -> (x, y)."""
    out = {}
    for b in layout["bins"]:
        out[b["name"]] = b["position"][:2]
        for kind in ("mound", "spill", "litter"):
            out.update({i["name"]: i["position"][:2] for i in b[kind]})
    if layout["obstacle"]:
        out["obstacle"] = layout["obstacle"]["position"][:2]
    return out


def park_view_pose(layout, target):
    """Where the robot stands to photograph a target: inside the square on the line through
    the bin's corner, far enough back that the whole bin (and its litter) is in view, facing
    the bin; or 0.35 m before the obstacle on its side, facing along the side."""
    if target == "obstacle":
        o = layout["obstacle"]
        if o is None:
            raise SystemExit("This scenario has no obstacle.")
        side = math.radians(o["yaw_deg"]) - math.pi / 2
        x, y = o["position"][0] - 0.35 * math.cos(side), o["position"][1] - 0.35 * math.sin(side)
        return x, y, side
    bins = {b["name"]: b for b in layout["bins"]}
    if target not in bins:
        raise SystemExit(f"No {target} in this scenario (bins: {', '.join(bins) or 'none'}).")
    b = bins[target]
    cx, cy = layout["route"]["corners"][b["corner"]]
    bx, by = b["position"]
    yaw = math.atan2(by - cy, bx - cx)
    back = BIN_PHOTO_DISTANCE_M
    return bx - back * math.cos(yaw), by - back * math.sin(yaw), yaw


def save_photo(path):
    import urllib.request
    with urllib.request.urlopen("http://localhost:8080/capture", timeout=5.0) as reply:
        data = reply.read()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as f:
        f.write(data)
    return path


def cmd_park_check(sim):
    layout = sim.world.info.get("park")
    if layout is None:
        raise SystemExit(f"World '{sim.world.name}' is not the park. Launch world:=park.")
    expected = park_expected(layout)
    print(f"Park scenario '{layout['scenario']}' (seed {layout['seed']})"
          + (f", overrides {layout['overrides']}" if layout["overrides"] else ""))
    sim.gt.wait_ready(tuple(expected), timeout=10.0)
    failures = 0
    for name, (x, y) in expected.items():
        pose = sim.gt.pose(name)
        if pose is None:
            print(f"  [FAIL] {name:18s} missing from Gazebo")
            failures += 1
            continue
        err = math.hypot(pose[0] - x, pose[1] - y)
        ok = err <= PARK_TOLERANCE_M
        failures += not ok
        print(f"  [{'OK' if ok else 'FAIL'}] {name:18s} ({pose[0]:6.3f}, {pose[1]:6.3f})  "
              f"error {err * 100:.1f} cm")
    with sim.gt.lock:
        present = set(sim.gt.poses)
    strays = sorted(n for n in present - set(expected) if n.startswith("bin_") or n == "obstacle")
    for name in strays:
        print(f"  [FAIL] {name:18s} in Gazebo but not in the scenario")
    failures += len(strays)
    print(f"  {len(layout['bins'])} bins, obstacle: "
          f"{layout['obstacle']['item'] + ' on side %d' % layout['obstacle']['side'] if layout['obstacle'] else 'none'}")

    if ARGS.look_at:
        start = layout["route"]["start"]
        for target in ARGS.look_at:
            x, y, yaw = park_view_pose(layout, target)
            sim.world.set_pose(ROBOT, x, y, REST_Z, yaw)
            time.sleep(0.5)
            if not wait_new_frames(3):
                print(f"  [FAIL] no new camera frames for {target}")
                failures += 1
                continue
            path = save_photo(os.path.join(RUN_DIR, "photos",
                                           f"park_{layout['scenario']}_{target}.jpg"))
            print(f"  photo of {target}: {path}")
        sim.world.set_pose(ROBOT, start[0], start[1], REST_Z, math.radians(start[2]))
    print("PASS" if not failures else f"FAIL ({failures} problems)")
    if failures:
        raise SystemExit(1)


# ===== Phase 1: route following =====
ROUTE_FOLLOWER = os.path.join(REPO_DIR, "walle-app", "route", "route_follower.py")
# The Gazebo camera (urdf/gazebo_car.urdf.xacro): 60 deg wide, 320 x 240, level, mounted
# camera_x ahead of and camera_z above base_link
SIM_CAMERA = {"forward_m": 0.07, "up_m": 0.07, "hfov_rad": 1.047, "aspect": 240 / 320}


def bin_corners(b):
    """The 8 corners of a bin (box or cylinder's bounding box) in the world frame."""
    length, width, height = b["size_m"]
    yaw = math.radians(b.get("yaw_deg", 0.0))
    c, s = math.cos(yaw), math.sin(yaw)
    bx, by = b["position"]
    out = []
    for dx in (-width / 2, width / 2):            # box x: depth along the diagonal
        for dy in (-length / 2, length / 2):      # box y: the wide face
            for z in (0.0, height):
                out.append((bx + dx * c - dy * s, by + dx * s + dy * c, z))
    return out


def bin_in_frame(b, pose):
    """Where the bin's corners land in the simulated camera's image, from robot pose
    (x, y, yaw_deg): the largest |horizontal| and |vertical| position as a fraction of the
    half-frame (<= 1 means inside the photo)."""
    x, y, yaw = pose[0], pose[1], math.radians(pose[2])
    cx, cy = x + SIM_CAMERA["forward_m"] * math.cos(yaw), y + SIM_CAMERA["forward_m"] * math.sin(yaw)
    cz = REST_Z + SIM_CAMERA["up_m"]
    th = math.tan(SIM_CAMERA["hfov_rad"] / 2)
    tv = th * SIM_CAMERA["aspect"]
    worst_h = worst_v = 0.0
    for px, py, pz in bin_corners(b):
        dx, dy = px - cx, py - cy
        fwd = dx * math.cos(yaw) + dy * math.sin(yaw)
        left = -dx * math.sin(yaw) + dy * math.cos(yaw)
        if fwd <= 0.01:
            return float("inf"), float("inf")
        worst_h = max(worst_h, abs(left / fwd) / th)
        worst_v = max(worst_v, abs((pz - cz) / fwd) / tv)
    return worst_h, worst_v


def misplace_bins(layout, deg):
    """A copy of the layout with each bin swung `deg` round its corner (bin 1 +deg, bin 2
    -deg, ...): where the follower THINKS the bins are, to test bin centering."""
    out = json.loads(json.dumps(layout))
    corners = {int(k): v for k, v in out["route"]["corners"].items()}
    for i, b in enumerate(out["bins"]):
        a = math.radians(deg if i % 2 == 0 else -deg)
        cx, cy = corners[int(b["corner"])][:2]
        dx, dy = b["position"][0] - cx, b["position"][1] - cy
        b["position"] = [round(cx + dx * math.cos(a) - dy * math.sin(a), 4),
                         round(cy + dx * math.sin(a) + dy * math.cos(a), 4)]
        b["view_heading_deg"] = round(b["view_heading_deg"] + math.degrees(a), 3)
        b["yaw_deg"] = round(b.get("yaw_deg", 0.0) + math.degrees(a), 3)
    return out


def detection_checks(summary, layout, out_dir):
    """Phase 3 checks on a run with detection (sim: the ground-truth stand-in): each bin
    centered within --center-tol-deg (truth), the detected bin is the right one, items have
    the right place (mound = on_bin, spill / litter = around_bin), around-bin positions within
    15 cm, and every detection reached the memory store (sim.sqlite)."""
    path = os.path.join(out_dir, "detections.json")
    stops = [ev for ev in summary["events"] if ev["event"] == "facing_bin"]
    if not os.path.exists(path):
        if any("centering" in ev for ev in stops):
            return [("detections.json written", False, "missing")]
        return []                                  # detection off for this run
    with open(path) as f:
        dets = json.load(f)
    checks = []
    for ev in stops:
        c = ev.get("centering") or {}
        err = ev.get("true_bearing_error_deg")
        checks.append((f"{ev['bin']} centered", bool(c.get("centered")) and err is not None
                       and abs(err) <= ARGS.center_tol_deg,
                       f"true bearing {err:+.1f} deg, {len(c.get('looks', []))} looks, "
                       f"{c.get('offset_from_config_deg', 0):+.1f} deg from the configured "
                       "heading" + (f" ({c['reason']})" if c.get("reason") else "")))
    groups = {"mound": "on_bin", "spill": "around_bin", "litter": "around_bin"}
    truth_xy = {it["name"]: it["position"][:2] for b in layout["bins"]
                for g in groups for it in b.get(g, [])}
    n_items = 0
    for photo in dets.get("photos", []):
        bd = photo.get("bin_detection") or {}
        wrong_place, xy_err = [], []
        for d in photo["items"]:
            n_items += 1
            if groups.get(d.get("truth_group")) != d["place"]:
                wrong_place.append(f"{d.get('truth')} as {d['place']}")
            if d.get("obj_xy") and d.get("truth") in truth_xy:
                t = truth_xy[d["truth"]]
                xy_err.append(math.hypot(d["obj_xy"][0] - t[0], d["obj_xy"][1] - t[1]))
        checks.append((f"{photo['bin']} detections", bd.get("truth") == photo["bin"]
                       and not wrong_place and all(e <= 0.15 for e in xy_err),
                       f"bin seen: {bd.get('truth')}, {len(photo['items'])} items "
                       f"({sum(d['place'] == 'on_bin' for d in photo['items'])} on top)"
                       + (f", wrong place: {', '.join(wrong_place)}" if wrong_place else "")
                       + (f", ground position error max {max(xy_err) * 100:.1f} cm"
                          if xy_err else "")))
    db = os.path.join(REPO_DIR, "walle-app", "data", "memory", "sim.sqlite")
    try:
        import sqlite3
        con = sqlite3.connect(db)
        stored = con.execute("SELECT COUNT(*) FROM detections WHERE run_id = ?",
                             (summary["run_id"],)).fetchone()[0]
        con.close()
    except Exception as e:
        stored = f"unreadable ({e})"
    checks.append(("detections in the memory store", stored == n_items,
                   f"{stored} in {os.path.relpath(db, REPO_DIR)}, {n_items} in detections.json"))
    return checks


def cmd_route_check(sim):
    layout = sim.world.info.get("park")
    if layout is None:
        raise SystemExit(f"World '{sim.world.name}' is not the park. Launch world:=park.")
    if layout["obstacle"]:
        raise SystemExit("This park has an obstacle on the route (Phase 5). Relaunch with "
                         "OBSTACLE=none, or SCENARIO=default.")
    sim.wait_robot(("ticks", "wheel", "ekf", "imu"))
    rtf = warn_rtf(sim)
    out_dir = os.path.join(RUN_DIR, "route", time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)
    # the world's real layout; the follower writes its own (maybe misplaced) layout.json
    layout_path = os.path.join(out_dir, "layout_truth.json")
    with open(layout_path, "w") as f:
        json.dump(layout, f, indent=2)

    sx, sy, syaw = layout["route"]["start"]
    sim.world.set_pose(ROBOT, sx, sy, REST_Z, math.radians(syaw))
    time.sleep(2.0)                   # let the estimates settle on the new pose
    print(f"{sim.describe(rtf)}  scenario '{layout['scenario']}', "
          f"{len(layout['bins'])} bins, {ARGS.laps} lap(s)")
    follower_layout = layout_path
    if ARGS.misplace_deg:
        follower_layout = os.path.join(out_dir, "layout_follower.json")
        with open(follower_layout, "w") as f:
            json.dump(misplace_bins(layout, ARGS.misplace_deg), f, indent=2)
        print(f"Follower told each bin is {ARGS.misplace_deg:.0f} deg round its corner from "
              "where it is (centering must find it).")
    cmd = [sys.executable, ROUTE_FOLLOWER, "--layout-json", follower_layout,
           "--truth-layout", layout_path, "--run-dir", out_dir,
           "--laps", str(ARGS.laps), "--camera-url", "http://localhost:8080/capture",
           "--no-measure"]
    if ARGS.dwell is not None:
        cmd += ["--dwell", str(ARGS.dwell)]
    if getattr(ARGS, "hide_cut_bins", False):
        cmd += ["--sim-hide-cut-bins"]
    if ARGS.feedback:
        cmd += ["--feedback", ARGS.feedback]
    if ARGS.coast_time is not None:
        cmd += ["--coast-time", str(ARGS.coast_time)]
    blackouts = None
    if getattr(ARGS, "blackout", None):
        every, hold = ARGS.blackout
        blackouts = subprocess.Popen(
            ["bash", "-c", f"sleep 25; while true; do ros2 topic pub --once /sim/blackout "
                           f"std_msgs/msg/Float32 '{{data: {hold}}}' >/dev/null 2>&1; "
                           f"sleep {every}; done"])
        print(f"Blackouts: every ~{every:.0f} s, no ticks or IMU for {hold:.1f} s "
              "(first after 25 s).")
    try:
        code = subprocess.call(cmd)
    finally:
        if blackouts:
            blackouts.terminate()
    summary_path = os.path.join(out_dir, "summary.json")
    if not os.path.exists(summary_path):
        raise SystemExit(f"FAIL: the route follower wrote no summary (exit {code}).")
    with open(summary_path) as f:
        summary = json.load(f)

    bins = {b["name"]: b for b in layout["bins"]}
    checks = []
    checks.append(("run finished", summary["status"] == "done",
                   summary["status"] + (f" ({summary['reason']})" if summary.get("reason") else "")))
    truth = summary["end_errors"].get("ground_truth")
    if truth:
        checks.append((f"ends within {ARGS.tolerance_cm:.0f} cm of the start",
                       truth["position_m"] * 100 <= ARGS.tolerance_cm,
                       f"{truth['position_m'] * 100:.1f} cm, heading {truth['heading_deg']:+.1f} deg"))
    else:
        checks.append(("ground truth recorded", False, "none in summary.json"))
    corner_rows = []
    for ev in summary["events"]:
        if ev["event"] == "arrived" and "true_error_m" in ev:
            corner_rows.append((ev["corner"], ev["true_error_m"] * 100,
                                ev["poses"]["ground_truth"][2]))
        if ev["event"] == "facing_bin":
            g = ev["poses"].get("ground_truth")
            h, v = bin_in_frame(bins[ev["bin"]], g) if g else (float("inf"),) * 2
            ev["in_frame_h"], ev["in_frame_v"] = round(h, 3), round(v, 3)
            checks.append((f"{ev['bin']} whole bin in frame", h <= 1.0 and v <= 1.0,
                           f"bearing to bin {ev.get('true_bearing_error_deg', float('nan')):+.1f} deg, "
                           f"extent {h:.2f} wide / {v:.2f} tall of the half-frame"
                           + (f", photo {ev['photo']}" if ev.get("photo") else "")))
    checks += detection_checks(summary, layout, out_dir)
    if getattr(ARGS, "blackout", None):
        gaps, waits = summary.get("sensor_gaps", []), summary.get("sensor_guard_waits", [])
        bad = [p for p in summary.get("pulses", []) if p.get("sensor_gap") or p.get("outlier")]
        checks.append(("sensor guard acted on the blackouts", bool(gaps) and bool(waits),
                       f"{len(gaps)} gaps seen, {len(waits)} waits (stopped "
                       f"{sum(w['seconds'] for w in waits):.1f} s in all), {len(bad)} pulses "
                       "kept out of the gain"))
    faced = {ev["bin"] for ev in summary["events"] if ev["event"] == "facing_bin"}
    for name in bins:
        if name not in faced:
            checks.append((f"{name} stop", False, "the robot never faced it"))

    print("\nPer corner (Gazebo truth):")
    for corner, err_cm, heading in corner_rows:
        print(f"  corner {corner}: {err_cm:5.1f} cm from the corner, heading {heading:+7.1f} deg")
    print("\nChecks:")
    for name, ok, detail in checks:
        print(f"  [{'OK' if ok else 'FAIL'}] {name}: {detail}")
    failures = sum(not ok for _, ok, _ in checks)
    result = {"profile": sim.profile.get("name"), "rtf": rtf, "laps": ARGS.laps,
              "tolerance_cm": ARGS.tolerance_cm, "pass": failures == 0,
              "checks": [{"name": n, "ok": ok, "detail": d} for n, ok, d in checks],
              "corners": [{"corner": c, "true_error_cm": round(e, 2), "true_heading_deg": h}
                          for c, e, h in corner_rows], "summary": summary}
    with open(os.path.join(out_dir, "route_check.json"), "w") as f:
        json.dump(result, f, indent=2)
    log_invocation(sim, {"route_check": out_dir, "pass": failures == 0})
    sim.world.set_pose(ROBOT, sx, sy, REST_Z, math.radians(syaw))
    print(f"Results: {out_dir}")
    print("PASS" if not failures else f"FAIL ({failures} problems)")
    if failures:
        raise SystemExit(1)


def main():
    cfg = load_config()
    if ARGS.command == "analyze":
        raise SystemExit(run_pilot_script("pilot/analyze.py"))
    if ARGS.command == "suggest":
        raise SystemExit(run_pilot_script("pilot/suggest_durations.py"))
    random.seed(ARGS.seed)
    with Sim() as sim:
        if ARGS.command == "check":
            cmd_check(sim, cfg)
        elif ARGS.command in ("shakedown", "rq1", "rq2", "motion"):
            cmd_motion(sim, cfg)
        elif ARGS.command in ("rq3", "range"):
            cmd_rq3(sim, cfg)
        elif ARGS.command == "calibrate-depth":
            cmd_calibrate(sim, cfg)
        elif ARGS.command == "park-check":
            cmd_park_check(sim)
        elif ARGS.command == "route-check":
            cmd_route_check(sim)


if __name__ == "__main__":
    main()
