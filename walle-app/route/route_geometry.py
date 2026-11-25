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


def build_plan(layout, laps=1, dwell_s=15.0, face_bins=True, look_sides=False):
    """The route as a list of steps, in the park frame:
        ("look_side", corner_from, corner_to, (ax, ay), (bx, by), heading)
                                   look down the side for obstacles (Phase 5; look_sides)
        ("drive", corner_from, corner_to, (ax, ay), (bx, by), heading)
    (build_trip's look_side / drive steps add a direction: +1 counter-clockwise, -1 not)
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
            if look_sides:
                steps.append(("look_side", here, to, corners[here], corners[to], heading))
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


def side_of(c_from, c_to, n_corners=4):
    """The leg c_from -> c_to between neighbouring corners: (side, direction). Side k runs
    from corner k to k+1 (counter-clockwise, park.py's numbering); direction is +1 when the
    leg drives it that way, -1 when it drives it clockwise (k+1 -> k)."""
    if c_to == c_from % n_corners + 1:
        return c_from, 1
    if c_from == c_to % n_corners + 1:
        return c_to, -1
    raise ValueError(f"corners {c_from} and {c_to} are not neighbours")


def trip_path(n_bin_corner, first=1, n_corners=4):
    """The corners from `first` to the bin's corner by the shorter way round the square
    (counter-clockwise on a tie). [first] when the bin is on the start corner."""
    ccw = (n_bin_corner - first) % n_corners
    cw = (first - n_bin_corner) % n_corners
    step = 1 if ccw <= cw else -1
    path, c = [first], first
    for _ in range(min(ccw, cw)):
        c = (c - 1 + step) % n_corners + 1
        path.append(c)
    return path


def build_trip(layout, bin_corner, dwell_s=15.0, look_sides=False, face_bin=True):
    """Phase 6's partial route ("check bin N", docs/plans/phase-6-plan.md decision 4): from
    the start corner to bin_corner by the shorter way, the bin stop there (face, dwell), a
    turn back (about 180 deg), the same corners back, and a turn to the start heading.
    Bin 1 (the start corner): no driving. Same step kinds as build_plan; look_side and
    drive steps carry a 7th element, the direction (+1 counter-clockwise, -1 clockwise),
    and their corners are the leg's (corner_from is where the leg starts)."""
    route = layout["route"]
    corners = {int(k): tuple(v) for k, v in route["corners"].items()}
    bins = {b["corner"]: b for b in layout["bins"]}
    start = route["start"]
    first = min(corners, key=lambda c: math.hypot(corners[c][0] - start[0],
                                                  corners[c][1] - start[1]))
    if bin_corner not in corners:
        raise ValueError(f"no corner {bin_corner}")
    path = trip_path(bin_corner, first, len(corners))
    steps = []

    def leg(c0, c1):
        _, d = side_of(c0, c1, len(corners))
        a, b = corners[c0], corners[c1]
        h = math.atan2(b[1] - a[1], b[0] - a[0])
        return a, b, h, d

    legs = list(zip(path, path[1:]))
    back = [(c1, c0) for c0, c1 in reversed(legs)]
    for k, (c0, c1) in enumerate(legs):
        a, b, h, d = leg(c0, c1)
        if k or abs(wrap(h - math.radians(start[2]))) > 1e-6:
            steps.append(("turn", c0, h))            # onto this leg (bin 4: from the start)
        if look_sides:
            steps.append(("look_side", c0, c1, a, b, h, d))
        steps.append(("drive", c0, c1, a, b, h, d))
    if face_bin and bin_corner in bins:
        bn = bins[bin_corner]
        steps.append(("face_bin", bin_corner, bn["name"], math.radians(bn["view_heading_deg"])))
        steps.append(("dwell", bin_corner, bn["name"], float(dwell_s)))
    for c0, c1 in back:
        a, b, h, d = leg(c0, c1)
        steps.append(("turn", c0, h))
        if look_sides:
            steps.append(("look_side", c0, c1, a, b, h, d))
        steps.append(("drive", c0, c1, a, b, h, d))
    steps.append(("turn", first, math.radians(start[2])))
    return steps


def build_look(layout, ahead_m=None):
    """Phase 6's look-only plan: one look_side step from the start pose along the heading it
    faces (side 1 when the robot is on the start corner facing corner 2)."""
    route = layout["route"]
    corners = {int(k): tuple(v) for k, v in route["corners"].items()}
    start = route["start"]
    h = math.radians(start[2])
    first = min(corners, key=lambda c: math.hypot(corners[c][0] - start[0],
                                                  corners[c][1] - start[1]))
    nxt = first % len(corners) + 1
    length = ahead_m or math.dist(corners[first], corners[nxt])
    a = (float(start[0]), float(start[1]))
    b = (a[0] + length * math.cos(h), a[1] + length * math.sin(h))
    return [("look_side", first, nxt, a, b, h, 1)]


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


def pulse_outlier(gain_dps, duration_s, dead_s, moved_deg, ratio):
    """A pulse that turned more than ratio x what it was planned to (gain x active time),
    or less than 1/ratio of it: not normal pulse behavior (a lost stop message, a sensor
    gap), so it must not teach the gain."""
    planned = gain_dps * max(duration_s - dead_s, 0.0)
    if planned < 1.0:
        return False
    return moved_deg > ratio * planned or moved_deg < planned / ratio


def bridge_gap(park_before, wheel_before, wheel_after):
    """The park pose after a sensor gap: the pose just before it, moved by what the wheels
    measured across it. The encoder counts are cumulative, so the wheel odometry is right
    again once ticks arrive; the EKF, which predicted on at the last speed through the gap,
    is not (sim blackout: it carried on 8 cm the robot never drove)."""
    dx, dy = wheel_after[0] - wheel_before[0], wheel_after[1] - wheel_before[1]
    c, s = math.cos(wheel_before[2]), math.sin(wheel_before[2])
    fwd, left = c * dx + s * dy, -s * dx + c * dy          # in the robot frame at the start
    h = park_before[2]
    return (park_before[0] + math.cos(h) * fwd - math.sin(h) * left,
            park_before[1] + math.sin(h) * fwd + math.cos(h) * left,
            wrap(h + wrap(wheel_after[2] - wheel_before[2])))


def estimate_jump(fused0, fused1, wheel0, wheel1, max_extra_m, max_extra_deg):
    """Did the fused estimate move in a way the wheels can't explain? Over one short window,
    compare how far it moved and turned with how far the wheels did. A differential drive
    can't slide sideways or turn without its wheels turning, so a large excess is a glitch
    (robot run 20261122_142402: the fused estimate went 40 cm sideways in 0.5 s while the
    wheels moved 7 cm). Returns (extra_m, extra_deg, is_jump)."""
    d_f = math.hypot(fused1[0] - fused0[0], fused1[1] - fused0[1])
    d_w = math.hypot(wheel1[0] - wheel0[0], wheel1[1] - wheel0[1])
    t_f = abs(math.degrees(wrap(fused1[2] - fused0[2])))
    t_w = abs(math.degrees(wrap(wheel1[2] - wheel0[2])))
    extra_m, extra_deg = d_f - d_w, t_f - t_w
    return extra_m, extra_deg, extra_m > max_extra_m or extra_deg > max_extra_deg


def pico_restarted(last, now, max_counts):
    """A Pico restart, from two consecutive /pico/wheel_ticks [left, right, ping_seq, ...]:
    ping_seq (one per finished HC-SR04 ping, answered or not) starts again from 0 at boot,
    so it going BACKWARDS is a restart even early on, when the encoder counts are still small;
    or the counts moved more in one message than the wheels can turn (the wheel odometry
    node's test)."""
    if last is None:
        return False
    if len(now) >= 3 and len(last) >= 3 and now[2] < last[2]:
        return True
    return abs(now[0] - last[0]) > max_counts or abs(now[1] - last[1]) > max_counts


def gaps_in(gaps, t0, t1):
    """The sensor gaps [(start, end, topic), ...] (monotonic s) that overlap [t0, t1]."""
    return [g for g in gaps if g[1] >= t0 and g[0] <= t1]


def centering_target(heading_now, bearing_deg, heading_cfg, max_offset_deg):
    """Bin centering (Phase 3): the heading (rad) that puts a bin seen at bearing_deg (left
    positive, from the camera's axis) in the middle of the frame, kept within max_offset_deg
    of the configured bin heading heading_cfg. Returns (target, clamped)."""
    target = heading_now + math.radians(bearing_deg)
    off = wrap(target - heading_cfg)
    lim = math.radians(max_offset_deg)
    if abs(off) <= lim:
        return wrap(target), False
    return wrap(heading_cfg + math.copysign(lim, off)), True


def parallax_factor(dist_cam_m, forward_m):
    """How much a small turn about the axle moves a thing dist_cam_m ahead of the camera in
    the camera's view, the camera being forward_m ahead of the axle: (d + f) / d."""
    d = max(dist_cam_m, 0.05)
    return (d + forward_m) / d


def learn_turn_scale(scale, bearing_before, bearing_after, turned_deg, parallax, gain,
                     limits=(0.7, 2.5), min_turn_deg=1.5):
    """Bin centering: update the turn scale (camera bearing change per recorded degree of
    turn, over parallax) from one pair of looks at the same bin. Turns smaller than
    min_turn_deg are too noisy to learn from."""
    if abs(turned_deg) < min_turn_deg:
        return scale
    seen = (bearing_before - bearing_after) / turned_deg / parallax
    if seen <= 0:
        return scale                  # moved the wrong way: a different box, don't learn
    seen = min(max(seen, limits[0]), limits[1])
    return min(max(scale + gain * (seen - scale), limits[0]), limits[1])
