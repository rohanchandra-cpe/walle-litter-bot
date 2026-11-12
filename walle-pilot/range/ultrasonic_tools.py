"""ROSA tools that identify an object with the camera and range it with the HC-SR04.

measure_ultrasonic() reads the next pings from /pico/wheel_ticks and reports the median,
using the same settings as the pilot (`ranging` in config/pilot.yaml).
identify_and_range(target) asks the vision node what it sees, then takes that reading.
Every reading is also added to range/results/range_log.csv.

The HC-SR04 reports the nearest surface in a cone straight ahead, which is not always the
object the camera named. So identify_and_range only measures when the target is in the
center third of the photo, and warns when something else is in the way. Neither tool
moves the robot.
"""

import csv
import os
import statistics
import sys
import threading
import time

import rclpy
from langchain_core.tools import tool
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "vision"))
sys.path.append(os.path.join(HERE, "..", "pilot"))
import vision_tools  # noqa: E402
from common import load_config  # noqa: E402
from robot_io import RobotIO  # noqa: E402

MIN_RANGE_M, MAX_RANGE_M = 0.02, 4.0   # the HC-SR04's rated range
FIRST_PING_TIMEOUT = 5.0               # s; ROS discovery can take a few seconds
STATUS = {-1: "timeout", -2: "no_response", 0: "no_ping"}
LOG = os.path.join(HERE, "results", "range_log.csv")
LOG_FIELDS = ["time", "target", "position", "distance_m", "valid_pings", "spread_m",
              "ping_status", "obstacle_ahead", "image", "description", "error"]


def speed_of_sound(temp_c):
    return 331.3 + 0.606 * temp_c


def summarize(pings, ranging):
    """Turn (seq, echo_us) pings into one reading. echo_us is None for a lost message.

    The reading has distance_m (the median), or no_echo (nothing reflected), or error
    (the sensor or its data is not working).
    """
    wanted = ranging["pings"]
    echoes = [echo for _, echo in pings] + [None] * (wanted - len(pings))
    speed = speed_of_sound(ranging["default_temp_c"])
    ranges, statuses = [], []
    for echo in echoes:
        if echo is None:
            statuses.append("lost")
        elif echo <= 0:
            statuses.append(STATUS.get(echo, f"code{echo}"))
        else:
            distance = speed * echo * 1e-6 / 2.0 - ranging["ultrasonic_offset_m"]
            if MIN_RANGE_M <= distance <= MAX_RANGE_M:
                ranges.append(distance)
                statuses.append(f"ok:{echo}")
            else:
                statuses.append("out_of_range")
    reading = {"valid_pings": len(ranges), "pings": wanted, "ping_status": "|".join(statuses)}
    if len(ranges) >= ranging["min_valid_pings"]:
        reading["distance_m"] = statistics.median(ranges)
        reading["spread_m"] = max(ranges) - min(ranges)
    elif statuses.count("no_response") > wanted / 2:
        reading["error"] = ("The HC-SR04 is not answering. Check its power and wiring, and "
                            "that the firmware uses the pins it is wired to.")
    elif statuses.count("lost") > wanted / 2:
        reading["error"] = "Most ping messages were lost. Is the Pico still connected?"
    else:
        reading["no_echo"] = True
    return reading


class UltrasonicClient:
    """A RobotIO node spinning in the background, used here only to read pings."""

    def __init__(self):
        if not rclpy.ok():
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.ranging = load_config()["ranging"]
        self.node = RobotIO("rosa_ultrasonic_client")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def measure(self):
        if self.node.wait_for(("ping",), timeout=FIRST_PING_TIMEOUT):
            return {"error": "No ultrasonic data on /pico/wheel_ticks. Is the Pico connected "
                             "and running the pilot firmware?"}
        pings = self.node.collect_pings(self.ranging["pings"],
                                        self.ranging["acquire_timeout_s"])
        if not pings:
            return {"error": "The Pico stopped sending pings. Is it still connected?"}
        return summarize(pings, self.ranging)

    def close(self):
        self.executor.shutdown()
        self.thread.join(timeout=2.0)
        self.node.destroy_node()


_client = None


def get_client():
    global _client
    if _client is None:
        _client = UltrasonicClient()
    return _client


def close_client():
    global _client
    if _client is not None:
        _client.close()
        _client = None


def log(reading, answer=None):
    os.makedirs(os.path.dirname(LOG), exist_ok=True)
    new_file = not os.path.exists(LOG)
    with open(LOG, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=LOG_FIELDS, extrasaction="ignore")
        if new_file:
            writer.writeheader()
        writer.writerow({"time": time.strftime("%Y-%m-%d %H:%M:%S"), **(answer or {}), **reading})


def format_reading(reading):
    """One short line the planning model can read reliably."""
    if reading.get("error"):
        return f"Ultrasonic error: {reading['error']}"
    pings = f"valid_pings: {reading['valid_pings']}/{reading['pings']}"
    if reading.get("no_echo"):
        return (f"distance_m: none | {pings} | no steady echo: nothing within about "
                f"{MAX_RANGE_M:.0f} m, or the surface is soft or angled")
    return f"distance_m: {reading['distance_m']:.2f} | {pings} | spread_m: {reading['spread_m']:.2f}"


def read_range(answer=None):
    reading = get_client().measure()
    log(reading, answer)
    return format_reading(reading)


# ===== ROSA tools =====
@tool
def measure_ultrasonic() -> str:
    """Measure the distance in meters to the nearest surface straight ahead with the
    ultrasonic sensor. It cannot tell what the surface is. Returns distance_m, how many
    pings were valid, and spread_m (how much they disagreed)."""
    return read_range()


@tool
def identify_and_range(target: str = "") -> str:
    """Take a photo to identify an object, then measure how far away it is with the
    ultrasonic sensor. target: a short name such as 'red ball', or empty to identify
    whatever is straight ahead. If the target is off to one side, no distance is measured
    and the result says which way to turn first."""
    target = str(target).strip()
    answer = vision_tools.get_client().ask(target)
    if answer.get("error"):
        return vision_tools.format_answer(answer)
    described = f"description: {answer['description']}"
    if not target:
        return (f"object_ahead: {answer['obstacle_ahead']} | {read_range(answer)} | "
                f"the distance is to the nearest surface straight ahead | {described}")
    if not answer["target_visible"]:
        return f"target_visible: no | distance not measured | {described}"
    seen = f"target_visible: yes | position: {answer['position']} | size: {answer['size']}"
    if answer["position"] != "center":
        return (f"{seen} | distance not measured: the ultrasonic sensor only points straight "
                f"ahead, so turn {answer['position']} 15 degrees and call this again | "
                f"{described}")
    text = f"{seen} | {read_range(answer)}"
    if not answer["path_clear"]:
        text += (f" | warning: {answer['obstacle_ahead']} is also ahead, so this may be the "
                 f"distance to it instead of the {target}")
    return f"{text} | {described}"
