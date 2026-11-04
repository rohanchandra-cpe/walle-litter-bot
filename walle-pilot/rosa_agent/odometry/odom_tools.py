"""ROSA tools that navigate with the fused position estimate: know where the robot is,
and go there.

Every move uses /odometry/filtered (the EKF output: wheels + IMU) as feedback, the
same closed-loop moves as move_test.py,
so the robot stops when ODOMETRY says the move is done. Commands go to
/pico/cmd_vel: the real Pico, or sim_pico_bridge.py in Gazebo.
"""

import math
import threading
import time

import rclpy
from langchain_core.tools import tool
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from move_test import Mover, local_change
from transforms import distance_and_bearing

MAX_DISTANCE = 2.0        # m, per drive command
MAX_GOAL_RANGE = 3.0      # m, how far go_to may travel
REQUIRE_APPROVAL = True   # ask the human before every move


class OdomRobot(Mover):
    """move_test's Mover, plus its own executor thread for use inside the agent."""

    def __init__(self):
        if not rclpy.ok():
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        super().__init__(node_name="rosa_odom_robot")
        self._waited = False
        self.executor_ = SingleThreadedExecutor()
        self.executor_.add_node(self)
        self.thread = threading.Thread(target=self.executor_.spin, daemon=True)
        self.thread.start()

    def ready(self, timeout=5.0):
        """True once /odometry/filtered arrives. The first time, also waits for the simulation's
        reference topics (as move_test.py does) so Gazebo's true pose is reported."""
        if self._waited:
            return self.pose() is not None
        try:
            self.wait_for_odom(timeout)
        except RuntimeError:
            return False
        self._waited = True
        return True

    def close(self):
        self.stop()
        self.executor_.shutdown()
        self.thread.join(timeout=2.0)
        self.destroy_node()
        rclpy.try_shutdown()


_robot = None


def get_robot():
    global _robot
    if _robot is None:
        _robot = OdomRobot()
    return _robot


def close_robot():
    global _robot
    if _robot is not None:
        _robot.close()
        _robot = None


def _number(text):
    try:
        value = float(str(text).strip().split()[0])
    except (ValueError, IndexError):
        return None
    return value if math.isfinite(value) else None


def _word(text, allowed):
    word = str(text).strip().lower()
    return word if word in allowed else None


def _approved(description):
    if not REQUIRE_APPROVAL:
        return True
    return input(f"\n[APPROVAL NEEDED] {description}. Allow? [y/N] ").strip().lower() in ("y", "yes")


def _fmt(pose):
    return f"x = {pose[0]:.3f} m, y = {pose[1]:.3f} m, heading = {math.degrees(pose[2]):.1f} deg"


def _report(robot, starts, description, relative=True):
    """What the fused estimate measured for this move, plus Gazebo's true motion when available.

    relative=True reports the move itself (forward, left, turn); relative=False
    reports where the robot ended up (used by go_to).
    """
    time.sleep(0.5)   # let the robot settle
    s, e = starts["odom"], robot.pose("odom")
    lines = [f"{description}. Done."]
    if relative:
        f, l, t = local_change(s, e)
        lines.append(f"Fused estimate measured: forward {f:.3f} m, left {l:.3f} m, "
                     f"turned {t:+.1f} deg.")
    lines.append(f"Fused pose now: {_fmt(e)}.")
    truth_s, truth_e = starts["ground_truth"], robot.pose("ground_truth")
    if truth_s is not None and truth_e is not None:
        if relative:
            tf, tl, tt = local_change(truth_s, truth_e)
            lines.append(f"Gazebo true motion: forward {tf:.3f} m, left {tl:.3f} m, "
                         f"turned {tt:+.1f} deg.")
        else:
            lines.append(f"Gazebo true pose: {_fmt(truth_e)}.")
    return "\n".join(lines)


def _start(description):
    """Common checks before a move. Returns (robot, starts) or an error message."""
    robot = get_robot()
    if not robot.ready():
        return None, ("No /odometry/filtered. Is the sensor fusion launch running? "
                      "The robot did not move.")
    if not _approved(description):
        return None, "The human declined. The robot did not move."
    return robot, {k: robot.pose(k) for k in robot.poses}


# ===== ROSA tools =====
# Direction comes from a WORD and size from a positive NUMBER: small models often
# get signs wrong, and they copy short argument lists more reliably.
@tool
def get_pose() -> str:
    """Report where the robot is now, from the fused estimate: x and y in meters, heading in degrees."""
    robot = get_robot()
    if not robot.ready():
        return "No /odometry/filtered. Is the sensor fusion launch running?"
    text = f"Fused pose: {_fmt(robot.pose('odom'))} (start = 0, 0, 0)."
    truth = robot.pose("ground_truth")
    if truth is not None:
        text += f" Gazebo true pose: {_fmt(truth)}."
    return text


@tool
def drive_distance(direction: str, meters: str) -> str:
    """Drive straight a distance, measured by the fused estimate. direction: 'forward' or 'backward'.
    meters: the distance exactly as the user wrote it, for example '1.0'."""
    d, dist = _word(direction, ("forward", "backward")), _number(meters)
    if d is None or dist is None or not 0 < abs(dist) <= MAX_DISTANCE:
        return (f"Invalid drive: direction 'forward' or 'backward', 0 < meters <= {MAX_DISTANCE}. "
                "The robot did not move.")
    dist = abs(dist) if d == "forward" else -abs(dist)
    description = f"Drive {d} {abs(dist)} m"
    robot, starts = _start(description)
    if robot is None:
        return starts
    robot.straight(dist)
    return _report(robot, starts, description)


@tool
def turn(direction: str, degrees: str) -> str:
    """Turn in place by an angle, measured by the fused estimate. direction: 'left' or 'right'.
    degrees: a positive angle up to 180, for example '90'."""
    d, deg = _word(direction, ("left", "right")), _number(degrees)
    if d is None or deg is None or not 0 < abs(deg) <= 180:
        return "Invalid turn: direction 'left' or 'right', 0 < degrees <= 180. The robot did not move."
    angle = abs(deg) if d == "left" else -abs(deg)
    description = f"Turn {d} {abs(deg)} deg"
    robot, starts = _start(description)
    if robot is None:
        return starts
    robot.turn(math.radians(angle))
    return _report(robot, starts, description)


@tool
def go_to(x: str, y: str) -> str:
    """Drive to a point in the world (odom) frame: turn toward it, then drive straight.
    x: meters forward of the start point. y: meters to the LEFT of the start point
    (negative = right). Use the numbers exactly as the user wrote them, for example '1.0'."""
    gx, gy = _number(x), _number(y)
    if gx is None or gy is None:
        return "Invalid goal. The robot did not move."
    robot = get_robot()
    if not robot.ready():
        return ("No /odometry/filtered. Is the sensor fusion launch running? "
                "The robot did not move.")
    dist, bearing = distance_and_bearing(robot.pose(), gx, gy)
    if dist > MAX_GOAL_RANGE:
        return f"Refused: the goal is {dist:.2f} m away (limit {MAX_GOAL_RANGE} m). The robot did not move."
    side = "left" if bearing >= 0 else "right"
    description = (f"Go to ({gx}, {gy}): turn {abs(math.degrees(bearing)):.1f} deg {side}, "
                   f"then drive {dist:.2f} m")
    robot, starts = _start(description)
    if robot is None:
        return starts
    robot.turn(bearing)
    time.sleep(0.3)
    dist, _ = distance_and_bearing(robot.pose(), gx, gy)   # re-aim after the turn
    robot.straight(dist)
    report = _report(robot, starts, description, relative=False)
    miss = math.hypot(robot.pose()[0] - gx, robot.pose()[1] - gy)
    return report + f"\nFused estimate distance from the goal: {miss:.3f} m."


@tool
def stop_robot() -> str:
    """Stop the robot immediately."""
    get_robot().stop()
    return "Stop command sent. The robot is stopped."
