"""Run RQ1 straight runs and RQ2 turns from the frozen schedule, one trial at a time.

Run inside the container while launch/pilot.launch.py is running:
    python3 pilot/motion_trial.py --session 1          # the next trial of session 1, then the next...
    python3 pilot/motion_trial.py --session 1 --once   # just one trial
    python3 pilot/motion_trial.py --shakedown straight-A            # tuning runs (shakedown_log.csv)
    python3 pilot/motion_trial.py --shakedown turn-ccw --duration 2.7

For each trial: place the robot, press Enter, and the script resets both pose
estimates, records a ros2 bag, drives the command, waits for the robot to rest,
and asks the measurer for the ground truth BEFORE showing any estimate. The
strategy (A/B) is never printed before the ground truth is saved.

Ctrl+C while the robot moves is the emergency stop: the robot stops and the run
is logged as an emergency-stop failure. Lifting the robot also works: the Pico
stops by itself 500 ms after the last command.
"""

import argparse
import math
import os
import signal
import subprocess
import time

from common import (BAG_DIR, MOTION_LOG, MOTION_SCHEDULE, SHAKEDOWN_LOG, append_row, ask,
                    ask_float, ask_yes, commit_label, load_config, next_scheduled, read_rows,
                    session_value, utc_now)
from robot_io import Robot, local_change

FIELDS = [
    "trial_id", "attempt", "session_id", "task", "strategy", "direction", "block_id",
    "order_in_block", "warmup", "timestamp_utc", "commit", "operator", "measurer",
    "motor_supply_v", "imu_calibrated", "cmd_duration_s", "completion_s", "stop_reason",
    "tick_msgs", "max_tick_gap_ms", "enc_left_delta", "enc_right_delta",
    "wheel_forward_m", "wheel_left_m", "wheel_turn_deg",
    "ekf_forward_m", "ekf_left_m", "ekf_turn_deg",
    "completed", "gate_violation", "failure_reason", "endpoint_measured",
    "true_forward_cm", "true_lateral_cm", "true_turn_deg",
    "external_invalid", "invalid_reason", "notes", "bag",
]
GATES = {"1": "lane_exit", "2": "collision", "3": "stall", "4": "emergency_stop",
         "5": "timeout", "6": "other"}
BAG_TOPICS = ["/pico/wheel_ticks", "/pico/imu/data", "/wheel/odom", "/odometry/filtered",
              "/pico/cmd_vel", "/pico/drive/status"]


# ===== motion commands =====
class Interrupted(Exception):
    pass


def drive_timed(robot, v, w, duration, rate, deadline):
    end = time.monotonic() + duration
    while time.monotonic() < min(end, deadline):
        robot.send(v, w)
        time.sleep(1.0 / rate)
    robot.stop()
    return "timeout" if time.monotonic() >= deadline else "commanded"


def drive_targeted(robot, motion, deadline):
    """RQ1-B: ramp toward 0.5 m of wheel odometry and stop there (minus the coast)."""
    start = robot.pose("wheel")
    target = motion["straight_target_m"]
    rate = motion["command_rate_hz"]
    while time.monotonic() < deadline:
        forward, _, turn_deg = local_change(start, robot.pose("wheel"))
        remaining = target - forward
        coast = max(0.0, robot.speed("wheel")[0]) * motion["targeted_coast_time_s"]
        if remaining <= motion["targeted_tolerance_m"] + coast:
            robot.stop()
            return "commanded"
        v = min(motion["targeted_max_speed_mps"],
                max(motion["targeted_min_speed_mps"], motion["targeted_gain"] * remaining))
        w = (max(-0.5, min(0.5, -2.0 * math.radians(turn_deg)))
             if motion["targeted_hold_heading"] else 0.0)
        robot.send(v, w)
        time.sleep(1.0 / rate)
    robot.stop()
    return "timeout"


def wait_for_rest(robot, motion, deadline):
    while time.monotonic() < deadline:
        if robot.seconds_since_wheels_moved() >= motion["rest_window_s"]:
            return True
        time.sleep(0.02)
    return False


def run_command(robot, trial, motion, duration):
    """Drive one trial. Returns (stop_reason, completion seconds)."""
    t0 = time.monotonic()
    deadline = t0 + motion["run_timeout_s"]
    rate = motion["command_rate_hz"]
    if trial["task"] == "straight" and trial["strategy"] == "A":
        reason = drive_timed(robot, motion["timed_speed_mps"], 0.0, duration, rate, deadline)
    elif trial["task"] == "straight":
        reason = drive_targeted(robot, motion, deadline)
    else:
        sign = 1.0 if trial["direction"] == "ccw" else -1.0
        reason = drive_timed(robot, 0.0, sign * motion["turn_rate_radps"], duration, rate,
                             deadline)
    rested = wait_for_rest(robot, motion, deadline)
    if not rested:
        robot.stop()
        reason = "timeout"
    completion = time.monotonic() - robot.seconds_since_wheels_moved() - t0
    return reason, max(0.0, completion)


# ===== ros2 bag =====
def start_bag(name):
    os.makedirs(BAG_DIR, exist_ok=True)
    path = os.path.join(BAG_DIR, name)
    if os.path.exists(path):
        path += time.strftime("_%H%M%S")
    process = subprocess.Popen(["ros2", "bag", "record", "-o", path] + BAG_TOPICS,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                               start_new_session=True)
    time.sleep(1.5)   # let it subscribe before the robot moves
    return process, os.path.relpath(path, os.path.dirname(BAG_DIR))


def stop_bag(process):
    if process is None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.kill()


# ===== ground truth =====
def ask_ground_truth(trial, stop_reason):
    out = {}
    if stop_reason in ("timeout", "emergency_stop"):
        print(f"  The run ended by {stop_reason.replace('_', ' ')}.")
        completed = False
        gate = stop_reason
    else:
        completed = ask_yes("  Did the run finish without a lane exit, collision, stall, "
                            "or emergency stop?")
        gate = ""
        if not completed:
            choice = ask("  What happened? 1 lane exit, 2 collision, 3 stall, "
                         "4 emergency stop, 6 other: ", choices=["1", "2", "3", "4", "6"])
            gate = GATES[choice]
    out["completed"] = completed
    out["gate_violation"] = gate
    out["failure_reason"] = ask("  Failure details: ", allow_blank=True) if gate else ""

    measured = ask_yes("  Can the end pose be measured?")
    out["endpoint_measured"] = measured
    if measured and trial["task"] == "straight":
        out["true_forward_cm"] = ask_float("  Forward displacement of the axle pin from "
                                           "the start line (cm): ")
        out["true_lateral_cm"] = ask_float("  Lateral offset from the lane centerline "
                                           "(cm, + = left): ")
    elif measured:
        out["true_turn_deg"] = ask_float("  Heading change on the protractor "
                                         "(deg, + = counter-clockwise): ")

    invalid = ask_yes("  Was the trial invalidated by OUTSIDE interference or loss of the "
                      "reference measurement (not a robot failure)?")
    out["external_invalid"] = invalid
    out["invalid_reason"] = ask("  What happened: ", allow_blank=False) if invalid else ""
    out["notes"] = ask("  Notes (slip, sounds, anything unusual; Enter for none): ",
                       allow_blank=True)
    return out


def show_result(trial, row, cfg):
    tol = cfg["tolerances"]
    print(f"\n  Saved. Condition: {trial['task']}"
          + (f" strategy {trial['strategy']}" if trial["task"] == "straight" else
             f" {trial['direction']}"))
    print(f"  Completion {row['completion_s']:.2f} s, largest message gap "
          f"{row['max_tick_gap_ms']:.0f} ms")
    if trial["task"] == "straight":
        print(f"  wheel odometry forward {100 * row['wheel_forward_m']:.1f} cm,"
              f"  EKF forward {100 * row['ekf_forward_m']:.1f} cm")
        if row.get("true_forward_cm") not in ("", None):
            error = row["true_forward_cm"] - 100 * cfg["motion"]["straight_target_m"]
            ok = abs(error) <= tol["distance_cm"] and abs(row["true_lateral_cm"]) <= tol["lateral_cm"]
            print(f"  true forward {row['true_forward_cm']:.1f} cm (error {error:+.1f} cm), "
                  f"lateral {row['true_lateral_cm']:+.1f} cm -> "
                  f"{'within' if ok and row['completed'] else 'outside'} tolerance")
    else:
        print(f"  wheel heading change {row['wheel_turn_deg']:+.1f} deg,"
              f"  EKF {row['ekf_turn_deg']:+.1f} deg")
        if row.get("true_turn_deg") not in ("", None):
            for key in ("wheel", "ekf"):
                err = (row[f"{key}_turn_deg"] - row["true_turn_deg"] + 180) % 360 - 180
                print(f"  {key} heading error {err:+.1f} deg")


# ===== one trial =====
def run_trial(robot, trial, attempt, cfg, log_path, people, duration=None, record_bag=True):
    motion = cfg["motion"]
    if duration is None:
        duration = (motion["timed_duration_s"] if trial["task"] == "straight"
                    else motion["turn_duration_s"])
    is_targeted = trial["task"] == "straight" and trial["strategy"] == "B"
    label = trial["task"] + ("" if trial["task"] == "straight" else f" {trial['direction']}")
    print(f"\n=== {trial['trial_id']} (attempt {attempt}): {label}"
          + ("  [warm-up, excluded]" if trial.get("warmup") == "1" else "") + " ===")
    if trial["task"] == "straight":
        print("  Place the axle pin on the start mark, pointer along the lane centerline.")
    else:
        print("  Place the axle pin over the protractor center, pointer on 0 deg.")
    supply = ask_float("  Motor supply voltage now (V, Enter to skip): ", allow_blank=True)
    input("  Clear the lane, hand on the emergency stop, then press Enter to start...")

    missing = robot.wait_for(("ticks", "wheel", "ekf"), timeout=5.0)
    if missing:
        raise SystemExit(f"No data from: {', '.join(missing)}. Is the launch file running "
                         "and the robot connected?")
    robot.reset_estimates()
    time.sleep(1.0)
    starts = {k: robot.pose(k) for k in ("wheel", "ekf")}
    ticks0 = robot.latest_ticks()
    bag, bag_name = start_bag(f"{trial['trial_id']}_a{attempt}") if record_bag else (None, "")

    robot.start_tick_recording()
    try:
        stop_reason, completion = run_command(robot, trial, motion, duration)
    except KeyboardInterrupt:
        robot.stop()
        stop_reason, completion = "emergency_stop", float("nan")
        print("\n  EMERGENCY STOP: robot stopped.")
    time.sleep(motion["settle_s"])
    tick_msgs, max_gap = robot.stop_tick_recording()
    ends = {k: robot.pose(k) for k in ("wheel", "ekf")}
    ticks1 = robot.latest_ticks()
    stop_bag(bag)

    print("  Robot at rest. Measure now; estimates are shown after you enter the truth.")
    truth = ask_ground_truth(trial, stop_reason)

    row = {k: trial.get(k, "") for k in ("trial_id", "session_id", "task", "strategy",
                                         "direction", "block_id", "order_in_block", "warmup")}
    row.update(attempt=attempt, timestamp_utc=utc_now(), commit=commit_label(),
               operator=people[0], measurer=people[1], motor_supply_v=supply,
               imu_calibrated=robot.imu_calibrated,
               cmd_duration_s="" if is_targeted else duration,
               completion_s=completion, stop_reason=stop_reason,
               tick_msgs=tick_msgs, max_tick_gap_ms=max_gap,
               enc_left_delta=ticks1[0] - ticks0[0], enc_right_delta=ticks1[1] - ticks0[1],
               bag=bag_name, **truth)
    for key in ("wheel", "ekf"):
        forward, left, turn = local_change(starts[key], ends[key])
        row.update({f"{key}_forward_m": forward, f"{key}_left_m": left, f"{key}_turn_deg": turn})
    append_row(log_path, FIELDS, row)
    show_result(trial, row, cfg)


def main():
    parser = argparse.ArgumentParser(description="Run scheduled RQ1/RQ2 motion trials")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--session", type=int, help="run this session's schedule in order")
    group.add_argument("--shakedown", choices=["straight-A", "straight-B", "turn-ccw", "turn-cw"],
                       help="a tuning run, logged to shakedown_log.csv")
    parser.add_argument("--duration", type=float,
                        help="shakedown only: try this command duration (s)")
    parser.add_argument("--once", action="store_true", help="run one trial and exit")
    parser.add_argument("--no-bag", action="store_true", help="do not record a ros2 bag")
    args = parser.parse_args()
    if args.duration is not None and not args.shakedown:
        parser.error("--duration is only for shakedown runs; main trials use pilot.yaml")

    cfg = load_config()
    with Robot() as robot:
        if args.shakedown:
            people = (ask("Operator initials: "), ask("Measurer initials: "))
            task, detail = args.shakedown.split("-")
            count = len(read_rows(SHAKEDOWN_LOG)) + 1
            trial = {"trial_id": f"SD{count:03d}", "session_id": "shakedown", "task": task,
                     "strategy": detail if task == "straight" else "",
                     "direction": detail if task == "turn" else "", "warmup": "0"}
            run_trial(robot, trial, 1, cfg, SHAKEDOWN_LOG, people, args.duration,
                      not args.no_bag)
            return

        if not os.path.exists(MOTION_SCHEDULE):
            raise SystemExit("No schedule yet. Run: python3 pilot/make_schedule.py")
        if session_value(args.session, "session_id") is None:
            raise SystemExit(f"Run the checklist first: python3 pilot/session_check.py "
                             f"--session {args.session}")
        people = (session_value(args.session, "operator", ""),
                  session_value(args.session, "measurer", ""))
        schedule = read_rows(MOTION_SCHEDULE)
        while True:
            trial, attempt = next_scheduled(args.session, schedule, read_rows(MOTION_LOG),
                                            "trial_id")
            if trial is None:
                print(f"\nSession {args.session}: every motion trial is done.")
                return
            run_trial(robot, trial, attempt, cfg, MOTION_LOG, people,
                      record_bag=not args.no_bag)
            if args.once or not ask_yes("\nNext trial?"):
                return


if __name__ == "__main__":
    main()
