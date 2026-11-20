"""Route geometry with no ROS: the plan of a lap, line following, and the park <-> odom frame.

The park frame is park.yaml's: corner 1 at the origin, x toward corner 2. The robot's
estimates (/odometry/filtered, /wheel/odom) start wherever the EKF started, so the route
follower anchors them: the pose an estimate reports at the start of a run is the park's
start pose. Everything here takes and returns (x, y, theta) tuples, theta in radians.
"""

import math


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Anchor:
    """Maps poses between an estimate's own frame and the park frame, given one pose that
    is known in both (the start). With heading_rate (rad/s, a gyro bias estimate), the
    estimate is also turned back by heading_rate x (t - t0) about the anchor point, which
    removes a constant heading drift that began at t0."""

    def __init__(self, est_start, park_start, t0=0.0, heading_rate=0.0):
        self.est, self.park = est_start, park_start
        self.rot = wrap(park_start[2] - est_start[2])   # est frame -> park frame
        self.t0, self.heading_rate = t0, heading_rate

    def _rot(self, t):
        return self.rot - (self.heading_rate * (t - self.t0) if t is not None else 0.0)

    def to_park(self, pose, t=None):
        rot = self._rot(t)
        dx, dy = pose[0] - self.est[0], pose[1] - self.est[1]
        c, s = math.cos(rot), math.sin(rot)
        return (self.park[0] + c * dx - s * dy, self.park[1] + s * dx + c * dy,
                wrap(pose[2] + rot))

    def to_est(self, pose, t=None):
        rot = self._rot(t)
        dx, dy = pose[0] - self.park[0], pose[1] - self.park[1]
        c, s = math.cos(-rot), math.sin(-rot)
        return (self.est[0] + c * dx - s * dy, self.est[1] + s * dx + c * dy,
                wrap(pose[2] - rot))


class BiasEstimate:
    """Gyro bias from the heading drift seen while the robot stood still (rad/s): the
    drift over all still periods divided by their total time."""

    def __init__(self):
        self.drift, self.seconds = 0.0, 0.0

    def add(self, drift_rad, seconds):
        if seconds > 0:
            self.drift += drift_rad
            self.seconds += seconds

    @property
    def rate(self):
        return self.drift / self.seconds if self.seconds > 0 else 0.0


def side_progress(pose, a, b):
    """Progress along the side from point a to point b.
    Returns (along, remaining, cross): metres driven along the side from a, metres left to
    b measured along the side, and the signed distance from the line (+ = left of it)."""
    hx, hy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(hx, hy)
    ux, uy = hx / length, hy / length
    dx, dy = pose[0] - a[0], pose[1] - a[1]
    along = dx * ux + dy * uy
    cross = -dx * uy + dy * ux
    return along, length - along, cross


def line_heading(side_heading, cross, gain, max_correction):
    """Heading to steer for while following a line: the side's own heading, turned back
    toward the line by atan(gain x cross-track error), at most max_correction (rad)."""
    return wrap(side_heading - clamp(math.atan(gain * cross), -max_correction, max_correction))


def build_plan(layout, laps=1, dwell_s=15.0, face_bins=True):
    """The route as a list of steps, in the park frame:
        ("drive", corner_from, corner_to, (ax, ay), (bx, by), heading)
        ("face_bin", corner, bin_name, heading)       turn to face the bin
        ("dwell", corner, bin_name, seconds)
        ("turn", corner, heading)                     turn to the next side (or the start)
    """
    route = layout["route"]
    corners = {int(k): tuple(v) for k, v in route["corners"].items()}
    bins = {b["corner"]: b for b in layout["bins"]}
    start = route["start"]
    # the corner the robot starts on
    first = min(corners, key=lambda c: math.hypot(corners[c][0] - start[0],
                                                  corners[c][1] - start[1]))
    steps, here = [], first
    wps = route["waypoints"]
    for lap in range(laps):
        for i, wp in enumerate(wps):
            to = int(wp["corner"])
            heading = math.radians(wp["heading_deg"])
            steps.append(("drive", here, to, corners[here], corners[to], heading))
            if face_bins and to in bins:
                b = bins[to]
                steps.append(("face_bin", to, b["name"], math.radians(b["view_heading_deg"])))
                steps.append(("dwell", to, b["name"], float(dwell_s)))
            last = lap == laps - 1 and i == len(wps) - 1
            nxt = math.radians(start[2]) if last else math.radians(wps[(i + 1) % len(wps)]["heading_deg"])
            steps.append(("turn", to, nxt))
            here = to
    return steps


# ----- small heading corrections: timed pulses -----
# A small correction can't be a slow turn: below ~0.1 m/s per wheel the wheels stick, the
# firmware's PI winds up, and the robot lurches 8-13 deg when asked for 4-6 (robot run
# 20261119_043639). A short pulse at a rate the wheels can hold turns about
# gain x (duration - dead) degrees, nearly whatever rate is commanded (walle-sim
# plastic_pid: ~110 deg/s, ~0.04 s dead time). The duration sets the angle.

def plan_pulse(err_deg, gain_dps, dead_s, min_s, max_s):
    """Duration (s) of a pulse that should turn |err_deg|, or None when even the shortest
    pulse would overshoot by more than the error (the correction would make it worse)."""
    need = abs(err_deg)
    t = dead_s + need / gain_dps
    if t < min_s:
        smallest = gain_dps * (min_s - dead_s)
        if smallest - need >= need - 1e-9:   # would land at least as far off, other side
            return None
        t = min_s
    return min(t, max_s)


def learn_pulse_gain(gain_dps, duration_s, moved_deg, dead_s, adapt_gain, lo=30.0, hi=400.0):
    """Update the pulse gain from one pulse: it turned moved_deg (toward the target)."""
    active = duration_s - dead_s
    if active < 0.02 or moved_deg <= 0.5:    # too short to tell, or it didn't break free
        return gain_dps
    seen = moved_deg / active
    return clamp(gain_dps + adapt_gain * (seen - gain_dps), lo, hi)
