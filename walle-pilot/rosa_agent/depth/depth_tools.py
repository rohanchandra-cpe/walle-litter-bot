"""ROSA tools that measure distance with the camera and drive to an object.

measure_distance(target) sends a question on /depth/ask and waits for the depth
node's answer on /depth/result. approach_to(target, stop_distance_m) drives toward an
object in short steps, measuring before every step, and stops at the distance you ask
for. Every answer is also added to /workspace/depth/results/depth_log.csv.

Safety: a depth estimate from one camera can be wrong, so approach_to
  - never drives more than 0.30 m between two measurements,
  - asks for your approval once before it starts,
  - checks drives close to the object against the fused position estimate (the measured
    distance must fall by about as much as the robot drove) and stops if it does not,
  - stops if it has driven much farther than the first measurement allowed.
"""

import csv
import json
import math
import os
import sys
import threading
import time
import uuid

import rclpy
from langchain_core.tools import tool
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "odometry"))
import odom_tools  # noqa: E402
from move_test import local_change  # noqa: E402

ANSWER_TIMEOUT = 120.0        # s; generous for slow computers and the first question
LOG = os.path.join(HERE, "results", "depth_log.csv")
LOG_FIELDS = ["time", "target", "true_m", "target_found", "target_raw_m", "target_m",
              "target_size_m", "bearing_deg", "ahead_raw_m", "ahead_m", "left_m", "right_m", "scale", "power",
              "model", "seconds", "image", "depth_image", "error"]

# Allowed stopping distances. The model is unreliable very close to an object: Exercise 1
# helps you find the smallest distance that works for your camera.
MIN_STOP_M, MAX_STOP_M = 0.5, 1.5
MAX_STEP_M = 0.30             # longest drive between two measurements
MAX_TURN_DEG = 40.0           # longest turn between two measurements
CENTER_TOLERANCE_DEG = 6.0    # target closer than this to straight ahead counts as centered
ARRIVE_TOLERANCE_M = 0.04     # closer than this to the stop distance counts as arrived
MAX_ACTIONS = 15              # turns plus drives in one approach_to
MIN_PROGRESS = 0.25           # close to the object, depth must fall by this share of each drive
NEAR_CHECK_M = 1.5            # ...checked only below this distance: farther away it is too noisy
MAX_TRAVEL_FACTOR = 1.5       # stop after driving this many times the first measured distance


class DepthClient(Node):
    """Sends questions to the depth node and waits for the matching answer."""

    def __init__(self):
        if not rclpy.ok():
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        super().__init__("rosa_depth_client")
        self.answers = {}
        self.cond = threading.Condition()
        self.ask_pub = self.create_publisher(String, "/depth/ask", 10)
        self.create_subscription(String, "/depth/result", self.on_result, 10)
        self.executor_ = SingleThreadedExecutor()
        self.executor_.add_node(self)
        self.thread = threading.Thread(target=self.executor_.spin, daemon=True)
        self.thread.start()

    def on_result(self, msg):
        try:
            answer = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self.cond:
            self.answers[answer.get("id")] = answer
            self.cond.notify_all()

    def ask(self, target="", true_m=None):
        # Give ROS discovery up to 5 s to find the depth node the first time
        end = time.time() + 5.0
        while self.count_subscribers("/depth/ask") == 0 and time.time() < end:
            time.sleep(0.2)
        if self.count_subscribers("/depth/ask") == 0:
            return {"target": target,
                    "error": "No depth node. Is launch/depth.launch.py running?"}
        request_id = uuid.uuid4().hex
        self.ask_pub.publish(String(data=json.dumps({"id": request_id, "target": target})))
        with self.cond:
            self.cond.wait_for(lambda: request_id in self.answers, timeout=ANSWER_TIMEOUT)
            answer = self.answers.pop(request_id, None)
        if answer is None:
            answer = {"target": target, "error": "No answer from the depth node in time."}
        log(answer, true_m)
        return answer

    def close(self):
        self.executor_.shutdown()
        self.thread.join(timeout=2.0)
        self.destroy_node()


def log(answer, true_m=None):
    os.makedirs(os.path.dirname(LOG), exist_ok=True)
    new_file = not os.path.exists(LOG)
    with open(LOG, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=LOG_FIELDS, extrasaction="ignore")
        if new_file:
            writer.writeheader()
        writer.writerow({"time": time.strftime("%Y-%m-%d %H:%M:%S"),
                         "true_m": "" if true_m is None else true_m, **answer})


_client = None


def get_client():
    global _client
    if _client is None:
        _client = DepthClient()
    return _client


def close_client():
    global _client
    if _client is not None:
        _client.close()
        _client = None


def side(bearing):
    return f"{abs(bearing):.0f} deg {'left' if bearing > 0 else 'right'}"


def format_answer(answer):
    """One short line the planning model can read reliably."""
    if answer.get("error"):
        return f"Depth error: {answer['error']} Do not move."
    ahead = f"ahead_m: {answer['ahead_m']:.2f}"
    if not answer.get("target"):
        return f"{ahead} | left_m: {answer['left_m']:.2f} | right_m: {answer['right_m']:.2f}"
    if not answer["target_found"]:
        return f"target_found: no | {ahead}"
    return (f"target_found: yes | distance_m: {answer['target_m']:.2f} | "
            f"bearing: {side(answer['bearing_deg'])} | {ahead}")


def fmt_pose(robot):
    x, y, th = robot.pose()
    return f"x = {x:.2f} m, y = {y:.2f} m, heading = {math.degrees(th):.0f} deg"


def _start(description):
    """Common checks before moving. Returns the robot, or an error message."""
    robot = odom_tools.get_robot()
    if not robot.ready():
        return None, ("No /odometry/filtered. Is the sensor fusion launch running? "
                      "The robot did not move.")
    if not odom_tools._approved(description):
        return None, "The human declined. The robot did not move."
    return robot, None


# ===== ROSA tools =====
@tool
def measure_distance(target: str) -> str:
    """Take a photo and measure distance in meters with the camera. target: the color and
    name of one object, for example 'red ball' (colors: red, yellow, green, blue). Returns
    target_found (yes/no), distance_m, bearing (degrees left or right of straight ahead),
    and ahead_m (the distance straight ahead). With an empty target it returns only the
    distances ahead, left, and right."""
    return format_answer(get_client().ask(str(target).strip()))


@tool
def approach_to(target: str, stop_distance_m: str) -> str:
    """Drive to a colored object and stop a measured distance from it: turn toward the
    object and drive in short steps, measuring before every step. target: color and name,
    for example 'red ball'. stop_distance_m: how far from the object to stop, in meters,
    between 0.5 and 1.5, for example '0.6'. The object must be visible: if it is not, the
    tool says so and does not move."""
    target = str(target).strip()
    stop = odom_tools._number(stop_distance_m)
    if stop is None or not MIN_STOP_M <= stop <= MAX_STOP_M:
        return (f"Invalid stop distance: use {MIN_STOP_M} to {MAX_STOP_M} meters. "
                "The robot did not move.")
    client = get_client()
    first = client.ask(target)
    if first.get("error"):
        return format_answer(first)
    if not first["target_found"]:
        return (f"The {target} is not visible. The robot did not move. "
                "Turn toward it and measure again.")
    start_m = first["target_m"]
    robot, error = _start(f"Approach the {target} (measured {start_m:.2f} m away) and stop "
                          f"{stop:.2f} m from it: up to {MAX_ACTIONS} short moves, "
                          "measuring before each one")
    if error:
        return error

    travelled, turns, previous = 0.0, 0, None
    notes = []
    answer = first
    for action in range(MAX_ACTIONS):
        if action:
            answer = client.ask(target)
            if answer.get("error"):
                return format_answer(answer)
        summary = (f"{turns} turn{'' if turns == 1 else 's'}, {travelled:.2f} m driven by the "
                   f"fused estimate; first measurement {start_m:.2f} m")
        if not answer["target_found"]:
            return (f"Lost sight of the {target} ({summary}). Fused pose: {fmt_pose(robot)}. "
                    "Measure again or turn to find it.")
        distance, bearing = answer["target_m"], answer["bearing_deg"]
        if previous is not None:    # did the depth agree with how far the robot drove?
            before, drove = previous
            previous = None
            notes.append(f"drove {drove:.2f} m, depth fell {before - distance:.2f} m")
            fell = before - distance
            if drove >= 0.15 and before <= NEAR_CHECK_M and fell < MIN_PROGRESS * drove:
                return (f"Stopped: the robot drove {drove:.2f} m but the measured distance fell "
                        f"only {fell:.2f} m, so the depth estimate cannot be "
                        f"trusted ({summary}). Fused pose: {fmt_pose(robot)}.")
        if abs(bearing) > CENTER_TOLERANCE_DEG:
            turn = max(-MAX_TURN_DEG, min(MAX_TURN_DEG, bearing))
            robot.turn(math.radians(turn))    # positive bearing = target on the left
            turns += 1
            time.sleep(0.3)
            continue
        step = min(MAX_STEP_M, distance - stop)
        if step < ARRIVE_TOLERANCE_M:
            return (f"Arrived: the {target} is {distance:.2f} m ahead by the depth model "
                    f"(goal {stop:.2f} m) ({summary}). Fused pose: {fmt_pose(robot)}. "
                    + "; ".join(notes))
        if travelled > MAX_TRAVEL_FACTOR * start_m:
            return (f"Stopped: the robot drove {travelled:.2f} m but the first measurement "
                    f"was only {start_m:.2f} m, so the depth estimate cannot be trusted. "
                    f"Fused pose: {fmt_pose(robot)}.")
        pose_before = robot.pose()
        robot.straight(step)
        time.sleep(0.3)
        drove = local_change(pose_before, robot.pose())[0]
        travelled += drove
        previous = (distance, drove)
    return (f"Stopped after {MAX_ACTIONS} moves without arriving ({summary}). "
            f"Fused pose: {fmt_pose(robot)}.")
