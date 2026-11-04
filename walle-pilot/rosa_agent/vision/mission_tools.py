"""Mission tools: camera + movement in one step, with the safety rules in the code.

A planning model given only low-level tools (look, turn, drive) often skips steps:
it drives without looking, or invents a destination. These tools keep the rules
in Python, so they hold no matter which model plans:

    drive_forward_if_clear(meters)  takes a photo first; refuses if anything is ahead
    search_for(target)              look, turn 45 degrees left, look again, up to a full circle
    approach(target)                turn toward the target and drive in short steps,
                                    looking before every step, until it looks large

Each tool asks for your approval once before it starts moving.
"""

import math
import os
import sys
import time

from langchain_core.tools import tool

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "odometry"))
import odom_tools  # noqa: E402
import vision_tools  # noqa: E402

SEARCH_STEP_DEG = 45.0     # degrees per search turn; 8 turns = a full circle
CENTER_STEP_DEG = 15.0     # degrees per turn while centering on the target
APPROACH_STEP_M = 0.30     # meters per drive step while approaching
MAX_FORWARD_M = 0.50       # longest single drive_forward_if_clear
MAX_APPROACH_STEPS = 20


def _fmt_pose(robot):
    x, y, th = robot.pose()
    return f"x = {x:.2f} m, y = {y:.2f} m, heading = {math.degrees(th):.0f} deg"


def _start(description):
    """Common checks before moving. Returns the robot, or an error message."""
    robot = odom_tools.get_robot()
    if not robot.ready():
        return None, "No /odometry/filtered. Is the sensor fusion launch running? The robot did not move."
    if not odom_tools._approved(description):
        return None, "The human declined. The robot did not move."
    return robot, None


@tool
def drive_forward_if_clear(meters: str) -> str:
    """Drive forward a short distance, but only if the camera sees nothing in the way.
    meters: up to 0.5, for example '0.3'. Takes a photo first and refuses if blocked."""
    dist = odom_tools._number(meters)
    if dist is None or not 0 < dist <= MAX_FORWARD_M:
        return f"Invalid distance: 0 < meters <= {MAX_FORWARD_M}. The robot did not move."
    answer = vision_tools.get_client().ask("")
    if answer.get("error"):
        return vision_tools.format_answer(answer)
    if not answer["path_clear"]:
        return (f"Refused: {answer['obstacle_ahead']} is ahead. The robot did not move. "
                f"Camera: {answer['description']}")
    robot, error = _start(f"Path is clear. Drive forward {dist} m")
    if error:
        return error
    robot.straight(dist)
    time.sleep(0.3)
    return f"Drove forward {dist} m. Fused pose now: {_fmt_pose(robot)}."


@tool
def search_for(target: str) -> str:
    """Find an object by looking around: look, then turn 45 degrees left and look again,
    up to a full circle. Stops as soon as the object is visible. target: e.g. 'red ball'."""
    target = str(target).strip()
    client = vision_tools.get_client()
    answer = client.ask(target)
    if answer.get("error"):
        return vision_tools.format_answer(answer)
    if answer["target_visible"]:
        return f"Found the {target} without turning. " + vision_tools.format_answer(answer)
    robot, error = _start(f"Search for the {target}: turn up to 360 deg in "
                          f"{SEARCH_STEP_DEG:.0f} deg steps, looking after each turn")
    if error:
        return error
    turned = 0.0
    while turned < 360.0 - 1e-6:
        robot.turn(math.radians(SEARCH_STEP_DEG))
        turned += SEARCH_STEP_DEG
        time.sleep(0.3)
        answer = client.ask(target)
        if answer.get("error"):
            return vision_tools.format_answer(answer)
        if answer["target_visible"]:
            return (f"Found the {target} after turning {turned:.0f} deg left. "
                    + vision_tools.format_answer(answer))
    return f"The {target} was not found after a full circle. Fused pose: {_fmt_pose(robot)}."


@tool
def approach(target: str) -> str:
    """Drive to a visible object: turn toward it and drive in short steps, looking before
    every step, until it looks large (close). Stops if the path is blocked or the object
    is lost. Call search_for first. target: e.g. 'red ball'."""
    target = str(target).strip()
    client = vision_tools.get_client()
    robot, error = _start(f"Approach the {target}: up to {MAX_APPROACH_STEPS} short moves, "
                          f"looking before each one")
    if error:
        return error
    turns = drives = 0
    for _ in range(MAX_APPROACH_STEPS):
        answer = client.ask(target)
        if answer.get("error"):
            return vision_tools.format_answer(answer)
        summary = (f"({turns} turn{'' if turns == 1 else 's'}, "
                   f"{drives} drive step{'' if drives == 1 else 's'})")
        if not answer["target_visible"]:
            return (f"Lost sight of the {target} {summary}. Fused pose: {_fmt_pose(robot)}. "
                    "Call search_for to find it again.")
        if answer["position"] in ("left", "right"):
            sign = 1.0 if answer["position"] == "left" else -1.0
            robot.turn(math.radians(sign * CENTER_STEP_DEG))
            turns += 1
        elif answer["size"] == "large":
            return (f"Arrived: the {target} is centered and close {summary}. "
                    f"Fused pose: {_fmt_pose(robot)}.")
        elif not answer["path_clear"]:
            return (f"Stopped: {answer['obstacle_ahead']} is between the robot and the "
                    f"{target} {summary}. Fused pose: {_fmt_pose(robot)}.")
        else:
            robot.straight(APPROACH_STEP_M)
            drives += 1
        time.sleep(0.3)
    return (f"Stopped after {MAX_APPROACH_STEPS} moves without arriving. "
            f"Fused pose: {_fmt_pose(robot)}.")