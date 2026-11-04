"""2D coordinate transformations between the world (odom) frame and the robot frame.

pose = (x, y, theta): the robot's position in the world, heading in radians,
counter-clockwise from the world +x axis. Robot frame: +x forward, +y left.
"""

import math


def wrap_angle(angle):
    """Wrap an angle in radians into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def robot_to_world(pose, px, py):
    """A point (px, py) in the ROBOT frame -> the same point in the WORLD frame."""
    x, y, theta = pose
    c, s = math.cos(theta), math.sin(theta)
    return x + px * c - py * s, y + px * s + py * c


def world_to_robot(pose, gx, gy):
    """A point (gx, gy) in the WORLD frame -> the same point in the ROBOT frame."""
    x, y, theta = pose
    dx, dy = gx - x, gy - y
    c, s = math.cos(theta), math.sin(theta)
    return dx * c + dy * s, -dx * s + dy * c


def distance_and_bearing(pose, gx, gy):
    """Distance (m) and bearing (rad, + = left) from the robot to a world point."""
    xr, yr = world_to_robot(pose, gx, gy)
    return math.hypot(xr, yr), math.atan2(yr, xr)


if __name__ == "__main__":
    # Worked examples
    pose = (1.0, 0.5, math.radians(30))
    print("robot_to_world 0.4 m ahead ->", [round(v, 3) for v in robot_to_world(pose, 0.4, 0.0)])
    print("world_to_robot goal (2.0, 1.5) ->", [round(v, 3) for v in world_to_robot(pose, 2.0, 1.5)])
    d, b = distance_and_bearing(pose, 2.0, 1.5)
    print(f"distance {d:.3f} m, bearing {math.degrees(b):.1f} deg")
