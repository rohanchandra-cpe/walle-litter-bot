"""Obstacles on the route (ROADMAP Phase 5), with no ROS: where a detected object is, whether
it blocks the side, the sonar stop trigger, and the detour around it.

Decided in plan mode (docs/plans/phase-5-plan.md, 2026-11-22):
- before each side the robot looks down the side from its corner (camera tilted down) and
  an item boxed inside the side's corridor is an obstacle; while driving, the HC-SR04 stops
  it for anything the look missed (config/route.yaml obstacle.confirm_pings pings in a row
  within trigger_m, and nearer than the end of the current leg);
- the detour goes INSIDE the square (left of the counter-clockwise route) as a rectangle of
  turns in place and straight legs: out sideways, past, back to the line (or to the corner);
  on a clockwise leg (Phase 6's trips, docs/plans/phase-6-plan.md) inside is to the robot's
  RIGHT: the planners take an `inside` sign, +1 counter-clockwise (the default), -1 clockwise;
- no way around (too big, too wide, the sonar blocks a detour leg) -> report and stop.

Frames: the park frame (x, y, heading rad); along a side a -> b, `along` is metres from a
and `cross` is + to the LEFT of the line (inside the square on the CCW route).
"""

import math

SOUND_MPS = 343.0


def side_unit(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    n = math.hypot(dx, dy)
    return dx / n, dy / n, n


def to_side(pt, a, b):
    """(along, cross) of a park point on the side a -> b."""
    ux, uy, _ = side_unit(a, b)
    dx, dy = pt[0] - a[0], pt[1] - a[1]
    return dx * ux + dy * uy, -dx * uy + dy * ux


def from_side(along, cross, a, b):
    """The park point `along` metres from a on the side a -> b, `cross` to the left."""
    ux, uy, _ = side_unit(a, b)
    return (a[0] + along * ux - cross * uy, a[1] + along * uy + cross * ux)


def echo_to_m(echo_us):
    """HC-SR04 echo time (us) -> range (m); None for -1 (timeout), -2 (no response)."""
    if echo_us is None or echo_us <= 0:
        return None
    return echo_us * SOUND_MPS / 2e6


def robot_to_park(pt, pose):
    """Robot-frame point (x forward from the axle, y left) -> park, pose heading in rad."""
    x, y, h = pose
    c, s = math.cos(h), math.sin(h)
    return (x + c * pt[0] - s * pt[1], y + s * pt[0] + c * pt[1])


def footprint_from_box(bbox, cam, pose, cfg, edge_px=4):
    """Where an item boxed in a photo is on the floor, from its box's bottom edge.
    cam: detect.geometry.Camera of the photo (tilt included); pose: (x, y, heading rad).
    Returns {"center": park (x, y), "near": park (x, y), "radius_m", "width_m" (as seen,
    before radius_m's clamp), "dist_m" (center from the axle), "cut": bool} or None
    when the bottom edge is above the horizon. cut: the box touches the frame's bottom, so
    the near edge is nearer than the frame shows (the position is then a guess: the
    follower prefers the sonar's)."""
    x1, _, x2, y2 = bbox
    cut = y2 >= cam.h - edge_px
    v = min(y2, cam.h - 1.0)
    near = cam.ground_point((x1 + x2) / 2.0, v)
    left = cam.ground_point(x1, v)
    right = cam.ground_point(x2, v)
    if near is None or left is None or right is None:
        return None
    width = math.hypot(left[0] - right[0], left[1] - right[1])
    r = min(max(width / 2.0, cfg["min_radius_m"]), cfg["max_box_radius_m"])
    # push the near edge back by r, along the ray from the lens through it
    lx, ly = cam.forward_m, cam.left_m
    dx, dy = near[0] - lx, near[1] - ly
    n = math.hypot(dx, dy) or 1.0
    center = (near[0] + r * dx / n, near[1] + r * dy / n)
    return {"center": robot_to_park(center, pose), "near": robot_to_park(near, pose),
            "radius_m": round(r, 4), "width_m": round(width, 4),
            "dist_m": round(math.hypot(center[0], center[1]), 4), "cut": bool(cut)}


def sonar_center(pose, range_m, sensor_forward_m, radius_m):
    """An object's center from a sonar range: straight ahead of the sensor, + its radius."""
    return robot_to_park((sensor_forward_m + range_m + radius_m, 0.0), pose)


def corridor_half_m(robot, radius_m):
    """Half width of the strip along the line the robot's body sweeps, grown by an object's
    radius: an object whose center is within it, on either side, is in the way."""
    return robot["half_width_m"] + robot["margin_m"] + radius_m


def blocks_side(center, radius_m, a, b, robot, start_along=0.0, end_slack_m=0.05):
    """True if an object at `center` (park) is in the way on the side a -> b ahead of
    start_along (the robot's place on the side). Returns (blocks, along, cross)."""
    along, cross = to_side(center, a, b)
    _, _, length = side_unit(a, b)
    ahead = start_along - radius_m <= along <= length + end_slack_m
    return (ahead and abs(cross) <= corridor_half_m(robot, radius_m)), along, cross


class SonarTrigger:
    """confirm pings in a row within trigger_m and nearer than the end of the current leg.
    feed() one ping at a time; it returns the median of the confirming ranges when it fires,
    else None. Pings that time out or don't answer break the run of confirmations."""

    def __init__(self, cfg, sensor_forward_m):
        self.trigger_m = float(cfg["trigger_m"])
        self.confirm = int(cfg["confirm_pings"])
        self.fwd = float(sensor_forward_m)
        self.slack = float(cfg.get("leg_end_slack_m", 0.0))
        self.hits = []

    def reset(self):
        self.hits = []

    def feed(self, echo_us, remaining_m):
        """remaining_m: the axle's distance left to the end of the current leg."""
        r = echo_to_m(echo_us)
        if r is None or r > self.trigger_m or r >= remaining_m - self.fwd + self.slack:
            self.hits = []
            return None
        self.hits.append(r)
        if len(self.hits) < self.confirm:
            return None
        got = sorted(self.hits[-self.confirm:])[self.confirm // 2]
        self.hits = []
        return got


def plan_detour(a, b, obstacle_along, obstacle_cross, radius_m, robot, cfg, here_along,
                depth_m=None, inside=1):
    """The detour around one object: plan_detour_group with a group of one."""
    return plan_detour_group(a, b, [(obstacle_along, obstacle_cross, radius_m)], robot, cfg,
                             here_along, depth_m, inside)


def plan_detour_group(a, b, objects, robot, cfg, here_along, depth_m=None, inside=1):
    """The detour around objects on the side a -> b (park points), inside the square.
    objects: [(along, cross, radius_m), ...]; one rectangle goes round all of them (robot
    run 20261122_204947: a detour round one object already passed a second one beside it,
    which the follower then tried to detour round again from beyond it).

    Returns {"approach": leg or None, "legs": [3 legs], "start_along", "end_along",
    "offset_m", "depth_m", "rejoins_corner"} or {"error": reason}; a leg is (from_xy, to_xy,
    heading_rad). The approach runs on the line to the start point (forward, or backward if
    the robot is too near to turn); the legs go out to the left, past, and back to the line;
    the end is clamped to corner b, so at worst it rejoins there.
    Clearances: turning in place sweeps turn_radius_m around the axle, passing sweeps
    half_width_m to the side; both keep margin_m + uncertainty_m from the objects' edges.
    Depth: a photo shows an object's width, not how far it runs away from the camera (a
    bottle lying along the path looked 6 cm wide and was 20 cm deep, 2026-11-22), so the
    turn back waits until depth_m (default max(the group's measured length,
    assumed_depth_m)) past the nearest face is cleared; near corner b that allowance may
    shrink to the measured length.
    inside: +1 the detour goes to the LEFT of a -> b (the counter-clockwise route), -1 to the
    right (a clockwise leg); the plan is then the mirror image across the line a -> b."""
    inside = 1 if inside >= 0 else -1
    ux, uy, length = side_unit(a, b)
    heading = math.atan2(uy, ux)
    unc = float(cfg["uncertainty_m"])
    m, tr = robot["margin_m"], robot["turn_radius_m"]
    biggest = max(r for _, _, r in objects)
    if biggest > float(cfg["max_radius_m"]):
        return {"error": f"object radius {biggest:.2f} m > max_radius_m {cfg['max_radius_m']}"}
    near = min(al - r for al, _, r in objects)
    far = max(al + r for al, _, r in objects)
    if depth_m is None:
        depth_m = max(far - near, float(cfg.get("assumed_depth_m", 0.0)))
    left_edge = max(max(inside * cr, 0.0) + r for _, cr, r in objects)   # inside edge
    offset = left_edge + robot["half_width_m"] + m + unc
    if offset > float(cfg["max_offset_m"]):
        return {"error": f"detour {offset:.2f} m wide > max_offset_m {cfg['max_offset_m']}"}
    start = near - (tr + m + unc)
    back = -float(cfg.get("max_back_past_corner_m", 0.0))
    if start < back:
        # near corner a: give up the uncertainty allowance first (park.py keeps
        # turn_radius + margin + the item's size from each corner)
        if near - (tr + m) < back - 1e-9:
            return {"error": f"object {near:.2f} m along the side: no room to turn before it"}
        start = back
    end = min(near + depth_m + tr + m + unc, length)
    if end - far < tr + m * 0.5:
        return {"error": f"object {length - far:.2f} m from the corner: no room to turn "
                         "after it"}
    p0 = from_side(start, 0.0, a, b)
    p1 = from_side(start, inside * offset, a, b)
    p2 = from_side(end, inside * offset, a, b)
    p3 = from_side(end, 0.0, a, b)
    here = from_side(here_along, 0.0, a, b)
    left, right = heading + inside * math.pi / 2, heading - inside * math.pi / 2  # out, back
    approach = ((here, p0, heading) if abs(start - here_along) > float(cfg.get("leg_min_m", 0.01))
                else None)              # on the line (backward if start < here)
    legs = [(p0, p1, wrap(left)), (p1, p2, heading), (p2, p3, wrap(right))]
    return {"approach": approach, "legs": legs, "start_along": round(start, 4),
            "end_along": round(end, 4), "offset_m": round(offset, 4),
            "depth_m": round(depth_m, 4), "rejoins_corner": end >= length - 1e-9,
            "inside": inside}


def plan_clearance(plan, keepouts, robot, step=0.01):
    """Smallest gap (m) between a detour's swept area and round objects it must not touch
    (keepouts: [((x, y), radius_m), ...], e.g. obstacles found on other sides). Each leg
    starts with a turn in place (a circle of turn_radius_m round the axle), then drives
    straight (the body: half_width_m to each side, turn_radius_m ahead and behind, the
    farthest the body reaches from the axle). Robot run 20261123_043408: side 3's detour
    turned 15 cm inside corner 3 and its turn swept into the bag on side 2."""
    if not keepouts:
        return math.inf
    tr = robot["turn_radius_m"]
    body = {"half_width_m": robot["half_width_m"], "body_front_m": tr, "body_back_m": tr}
    best = math.inf
    for p, q, h in all_legs(plan):
        for c, r in keepouts:
            best = min(best, math.dist(p, c) - r - tr)          # the turn at the leg's start
        n = max(1, int(math.dist(p, q) / step))
        poses = [(p[0] + (q[0] - p[0]) * k / n, p[1] + (q[1] - p[1]) * k / n, h)
                 for k in range(n + 1)]
        for c, r in keepouts:
            best = min(best, min_clearance(poses, c, r, body))
    return best


def group_ahead(pending, robot, cfg, a, b, here_along, inside=1):
    """The first pending obstacle (sorted by along) and every later one whose near face is
    within the detour's reach: before its turn back point plus a turn's clearance. Grows
    until no more join. Returns (group, plan)."""
    group = [pending[0]]
    while True:
        plan = plan_detour_group(a, b, [(o["along"], o["cross"], o["radius_m"]) for o in group],
                                 robot, cfg, here_along, inside=inside)
        reach = (plan["end_along"] if "error" not in plan else
                 max(o["along"] + o["radius_m"] for o in group)) \
            + robot["turn_radius_m"] + robot["margin_m"]
        more = [o for o in pending if o not in group and o["along"] - o["radius_m"] < reach]
        if not more:
            return group, plan
        group += more


def all_legs(plan):
    """The approach (if any) and the three detour legs, in order."""
    return ([plan["approach"]] if plan.get("approach") else []) + plan["legs"]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def min_clearance(path_poses, center, radius_m, robot):
    """Smallest gap (m) between the robot's body and a round object along a list of poses
    (x, y, heading rad); < 0 is a collision. The body is a rectangle: half_width_m to each
    side, and from body_back_m behind the axle to body_front_m ahead of it."""
    hw = robot["half_width_m"]
    front, back = robot["body_front_m"], robot["body_back_m"]
    best = math.inf
    for x, y, h in path_poses:
        c, s = math.cos(h), math.sin(h)
        dx, dy = center[0] - x, center[1] - y
        fx, ly = c * dx + s * dy, -s * dx + c * dy      # the object in the robot's frame
        qx = min(max(fx, -back), front)
        qy = min(max(ly, -hw), hw)
        best = min(best, math.hypot(fx - qx, ly - qy) - radius_m)
    return best
