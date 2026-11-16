"""The WALL-E park: load the fixed geometry and a test scenario, and resolve the layout.

    python3 walle-app/park.py                          # default scenario, summary
    python3 walle-app/park.py --scenario mixed         # a file in config/scenarios/
    python3 walle-app/park.py --scenario mixed --bins 2 --obstacle none
    python3 walle-app/park.py --scenario obstacle_only --obstacle 3 --seed 11 --yaml

config/park.yaml holds what never changes between tests (the square, bin size and
placement, the robot's footprint, the items the simulator can place). A scenario holds
what is in the park for one test: which corners have a bin, each bin's state, and whether
there is an obstacle. Overrides change a scenario without editing it:

    bins      keep the first N corners' bins (N above the scenario's count adds "ok" bins)
    obstacle  none | random | 1-4 (the side it is on; side k runs from corner k to k+1)
    seed      a different seed redraws every random choice

load_park() returns a plain dict (YAML-safe), the single description of the layout used by
the Gazebo world generator and, later, by the robot's route and the agent. Random choices
come from the seed, one stream per bin and one for the obstacle, so changing the number of
bins never moves the obstacle.
"""

import argparse
import copy
import math
import os
import random
import sys

import yaml

APP_DIR = os.path.dirname(os.path.abspath(__file__))
PARK_FILE = os.path.join(APP_DIR, "config", "park.yaml")
SCENARIO_DIR = os.path.join(APP_DIR, "config", "scenarios")
CORNERS = (1, 2, 3, 4)
STATES = ("ok", "overflowing")
# Overflow mound: items per layer above the rim, and items spilled at the base, per level
MOUND_LAYERS = {1: [3], 2: [4, 2], 3: [5, 3, 2]}
LAYER_STEP_M = 0.035
LITTER_GAP_M = (0.03, 0.12)       # gap between the bin's side and a litter item
LITTER_SPREAD_DEG = 70.0          # litter stays within this angle of the outward direction


class ParkError(ValueError):
    """A park or scenario that cannot be built as asked."""


# ===== loading =====
def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f) or {}


def scenario_path(scenario):
    if os.path.exists(scenario):
        return scenario
    path = os.path.join(SCENARIO_DIR, f"{scenario}.yaml")
    if not os.path.exists(path):
        names = sorted(f[:-5] for f in os.listdir(SCENARIO_DIR) if f.endswith(".yaml"))
        raise ParkError(f"No scenario '{scenario}'. Choose one of: {', '.join(names)}, "
                        "or give a path to a YAML file.")
    return path


def load_park(scenario="default", bins=None, obstacle=None, seed=None, park_file=PARK_FILE):
    """Resolve park.yaml + a scenario (+ overrides) into the full layout dict."""
    park = load_yaml(park_file)
    path = scenario_path(scenario)
    scen = apply_overrides(load_yaml(path), bins, obstacle, seed)
    name = os.path.splitext(os.path.basename(path))[0]

    geo = Geometry(park)
    layout = {
        "scenario": name,
        "seed": scen["seed"],
        "overrides": {k: v for k, v in (("bins", bins), ("obstacle", obstacle), ("seed", seed))
                      if v is not None},
        "route": {
            "side_m": geo.side,
            "corners": {c: list(geo.corner(c)) for c in CORNERS},
            "start": geo.start_pose(),
            "waypoints": geo.waypoints(),
        },
        "bins": [],
        "obstacle": None,
    }
    for corner in sorted(scen["bins"]):
        layout["bins"].append(build_bin(geo, park, corner, scen["bins"][corner], scen["seed"]))
    if scen["obstacle"] is not None:
        layout["obstacle"] = build_obstacle(geo, park, scen["obstacle"], scen["seed"])
    return layout


def apply_overrides(scen, bins, obstacle, seed):
    """Validate a scenario and apply the command-line overrides to it."""
    scen = copy.deepcopy(scen)
    unknown = set(scen) - {"bins", "obstacle", "seed"}
    if unknown:
        raise ParkError(f"Unknown scenario keys: {', '.join(sorted(unknown))}")
    raw_bins = scen.get("bins") or {}
    bins_out = {}
    for key, spec in raw_bins.items():
        corner = int(key)
        if corner not in CORNERS:
            raise ParkError(f"Bin corner {key} is not one of {CORNERS}.")
        bins_out[corner] = dict(spec or {})
    if bins is not None:
        bins = int(bins)
        if not 0 <= bins <= len(CORNERS):
            raise ParkError(f"bins must be 0-{len(CORNERS)}, not {bins}.")
        kept = {c: bins_out[c] for c in sorted(bins_out)[:bins]}
        for c in CORNERS:                 # add plain bins at free corners if asked for more
            if len(kept) >= bins:
                break
            kept.setdefault(c, {"state": "ok"})
        bins_out = kept
    scen["bins"] = bins_out

    obs = scen.get("obstacle", "none")
    if obs in (None, "none", False):
        obs = None
    elif not isinstance(obs, dict):
        raise ParkError("obstacle must be 'none' or a mapping like {side: random, at: random}.")
    if obstacle is not None:
        value = str(obstacle).lower()
        if value == "none":
            obs = None
        else:
            obs = dict(obs or {})
            obs["side"] = "random" if value == "random" else int(value)
            obs.setdefault("at", "random")
            obs.setdefault("item", "random")
    scen["obstacle"] = obs
    scen["seed"] = int(seed if seed is not None else scen.get("seed", 0))
    return scen


# ===== geometry =====
class Geometry:
    def __init__(self, park):
        route, robot, bins = park["route"], park["robot"], park["bins"]
        self.side = float(route["side_m"])
        self.start_corner = int(route.get("start_corner", 1))
        self.half_width = float(robot["half_width_m"])
        self.turn_radius = float(robot["turn_radius_m"])
        self.margin = float(robot["margin_m"])
        self.bin_shape = bins.get("shape", "cylinder")
        if self.bin_shape == "box":
            # The wide face (length) faces the square; depth (width) runs along the diagonal
            self.bin_length, self.bin_width = float(bins["length_m"]), float(bins["width_m"])
            self.bin_half = (self.bin_width / 2, self.bin_length / 2)   # (outward, across)
        elif self.bin_shape == "cylinder":
            r = float(bins["radius_m"])
            self.bin_length = self.bin_width = 2 * r
            self.bin_half = (r, r)
        else:
            raise ParkError(f"bins.shape must be box or cylinder, not '{self.bin_shape}'.")
        self.bin_height = float(bins["height_m"])
        # Smallest distance from the bin's center to its side (mound spread, item spacing)
        self.bin_inner = min(self.bin_half)
        self.camera = robot.get("camera")
        self.items = park["items"]
        self.frame_from_corner = bool(bins.get("frame_from_corner", False))
        self.bin_clear_offset = self.clearance_offset()
        self.bin_frame_offset = self.framing_offset() if self.frame_from_corner else 0.0
        self.bin_offset = self.check_bin_offset(bins.get("offset_m", "auto"))

    def corner(self, c):
        s = self.side
        return {1: (0.0, 0.0), 2: (s, 0.0), 3: (s, s), 4: (0.0, s)}[c]

    def outward(self, c):
        """Unit vector from the square's center out through corner c."""
        x, y = self.corner(c)
        dx, dy = x - self.side / 2, y - self.side / 2
        n = math.hypot(dx, dy)
        return dx / n, dy / n

    def next_corner(self, c):
        return c % len(CORNERS) + 1

    def side_heading(self, side):
        (x0, y0), (x1, y1) = self.corner(side), self.corner(self.next_corner(side))
        return math.atan2(y1 - y0, x1 - x0)

    def start_pose(self):
        x, y = self.corner(self.start_corner)
        return [x, y, round(math.degrees(self.side_heading(self.start_corner)), 3)]

    def waypoints(self):
        """Corners in driving order, ending back at the start, each with the heading the
        robot drives along to reach it."""
        out, c = [], self.start_corner
        for _ in CORNERS:
            nxt = self.next_corner(c)
            x, y = self.corner(nxt)
            out.append({"corner": nxt, "x": x, "y": y,
                        "heading_deg": round(math.degrees(self.side_heading(c)), 3)})
            c = nxt
        return out

    # ----- bins -----
    def bin_extent(self, rel):
        """Distance from the bin's center to its side, in the direction `rel` radians from
        the outward diagonal."""
        hx, hy = self.bin_half
        if self.bin_shape == "cylinder":
            return hx
        c, s = abs(math.cos(rel)), abs(math.sin(rel))
        return min(hx / c if c > 1e-9 else float("inf"), hy / s if s > 1e-9 else float("inf"))

    def dist_to_bin(self, x, y, center, yaw):
        """Distance from a floor point to the bin's footprint (0 inside it)."""
        dx, dy = x - center[0], y - center[1]
        if self.bin_shape == "cylinder":
            return max(0.0, math.hypot(dx, dy) - self.bin_half[0])
        c, s = math.cos(yaw), math.sin(yaw)
        lx, ly = abs(dx * c + dy * s), abs(-dx * s + dy * c)
        return math.hypot(max(0.0, lx - self.bin_half[0]), max(0.0, ly - self.bin_half[1]))

    def bin_outline(self, center, yaw, n=72):
        """Points around the bin's footprint on the floor (its outline)."""
        out = []
        for k in range(n):
            a = 2 * math.pi * k / n
            d = self.bin_extent(a)
            out.append((center[0] + d * math.cos(yaw + a), center[1] + d * math.sin(yaw + a)))
        return out

    def dist_to_side_lines(self, x, y):
        """Distance to the nearest side of the square, each side extended past its corners
        as a full line (the robot may overshoot a corner before it turns)."""
        best = float("inf")
        for c in CORNERS:
            (x0, y0), (x1, y1) = self.corner(c), self.corner(self.next_corner(c))
            dx, dy = x1 - x0, y1 - y0
            best = min(best, abs((x - x0) * dy - (y - y0) * dx) / math.hypot(dx, dy))
        return best

    def bin_is_clear(self, center, yaw):
        """The bin's outline is clear of the robot driving the route (sides as full lines)
        and turning on the corners."""
        for x, y in self.bin_outline(center, yaw):
            if self.dist_to_side_lines(x, y) < self.half_width + self.margin - 1e-9:
                return False
            if self.dist_to_nearest_corner(x, y) < self.turn_radius + self.margin - 1e-9:
                return False
        return True

    def clearance_offset(self):
        """Smallest offset (1 mm steps) at which a bin outside corner 1 is clear."""
        ux, uy = self.outward(1)
        yaw = math.atan2(uy, ux)
        for mm in range(0, 5001):
            d = mm / 1000
            if self.bin_is_clear((d * ux, d * uy), yaw):
                return d
        raise ParkError("No bin offset up to 5 m keeps the bin clear of the route.")

    def framing_offset(self):
        """Smallest offset at which the camera, with the robot on the corner facing the bin,
        sees the whole bin: top, bottom and both sides inside the frame, with
        `frame_margin` of each half-frame left free."""
        cam = self.camera
        if not cam:
            raise ParkError("bins.frame_from_corner needs robot.camera in park.yaml.")
        keep = 1.0 - float(cam.get("frame_margin", 0.0))
        tv = math.tan(math.radians(float(cam["vertical_fov_deg"])) / 2) * keep
        th = math.tan(math.radians(float(cam["horizontal_fov_deg"])) / 2) * keep
        hc = float(cam["height_m"])
        near = max((self.bin_height - hc) / tv, hc / tv, (self.bin_length / 2) / th)
        return near + float(cam["forward_m"]) + self.bin_half[0]

    def check_bin_offset(self, offset):
        need = self.bin_clear_offset
        if offset in (None, "auto"):
            return math.ceil(max(need, self.bin_frame_offset) * 100 - 1e-9) / 100
        offset = float(offset)
        if offset + 1e-9 < need:
            raise ParkError(f"bins.offset_m {offset:.3f} m is too small: this bin needs at "
                            f"least {need:.3f} m to stay clear of the robot.")
        return offset

    def dist_to_route(self, x, y):
        best = float("inf")
        for c in CORNERS:
            (x0, y0), (x1, y1) = self.corner(c), self.corner(self.next_corner(c))
            dx, dy = x1 - x0, y1 - y0
            t = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / (dx * dx + dy * dy)))
            best = min(best, math.hypot(x - (x0 + t * dx), y - (y0 + t * dy)))
        return best

    def dist_to_nearest_corner(self, x, y):
        return min(math.hypot(x - cx, y - cy) for cx, cy in map(self.corner, CORNERS))

    # ----- items -----
    def item(self, name):
        if name not in self.items:
            raise ParkError(f"Unknown item '{name}'. Items: {', '.join(self.items)}")
        return self.items[name]

    def footprint(self, name, pose):
        """Radius of the item's footprint on the floor."""
        it = self.item(name)
        if it["shape"] == "sphere":
            return it["radius_m"]
        return it["radius_m"] if pose == "upright" else it["length_m"] / 2

    def thickness(self, name, pose):
        it = self.item(name)
        if it["shape"] == "sphere":
            return 2 * it["radius_m"]
        return it["length_m"] if pose == "upright" else 2 * it["radius_m"]

    def placed(self, model, name, x, y, z_base, pose, yaw):
        it = self.item(name)
        return {"name": model, "item": name, "category": it["category"],
                "position": [round(x, 4), round(y, 4),
                             round(z_base + self.thickness(name, pose) / 2, 4)],
                "pose": pose, "yaw_deg": round(math.degrees(yaw), 2)}


# ===== bins =====
def build_bin(geo, park, corner, spec, seed):
    unknown = set(spec) - {"state", "level", "litter"}
    if unknown:
        raise ParkError(f"Bin {corner}: unknown keys {', '.join(sorted(unknown))}")
    state = spec.get("state", "ok")
    if state not in STATES:
        raise ParkError(f"Bin {corner}: state must be one of {STATES}, not '{state}'.")
    levels = int(park["bins"].get("overflow_levels", 3))
    level = None
    if state == "overflowing":
        level = int(spec.get("level", 2))
        if not 1 <= level <= levels or level not in MOUND_LAYERS:
            raise ParkError(f"Bin {corner}: level must be 1-{levels}, not {level}.")
    elif "level" in spec:
        raise ParkError(f"Bin {corner}: 'level' only applies to an overflowing bin.")
    litter = int(spec.get("litter", 0))
    if not 0 <= litter <= int(park["bins"].get("litter_max", 6)):
        raise ParkError(f"Bin {corner}: litter must be 0-{park['bins'].get('litter_max', 6)}.")

    rng = random.Random(f"{seed}-bin-{corner}")
    cx, cy = geo.corner(corner)
    ux, uy = geo.outward(corner)
    bx, by = cx + geo.bin_offset * ux, cy + geo.bin_offset * uy
    yaw = math.atan2(uy, ux)
    name = f"bin_{corner}"
    out = {"id": corner, "name": name, "corner": corner,
           "position": [round(bx, 4), round(by, 4)],
           "shape": geo.bin_shape, "height_m": geo.bin_height,
           # box: length = the wide face toward the square, width = depth along the diagonal
           "size_m": [geo.bin_length, geo.bin_width, geo.bin_height],
           "yaw_deg": round(math.degrees(yaw), 3),
           # heading the robot turns to on the corner to face the bin's center
           "view_heading_deg": round(math.degrees(yaw), 3),
           "state": state, "overflow_level": level,
           "mound": [], "spill": [], "litter": []}
    items = sorted(geo.items)

    if level:
        for layer, count in enumerate(MOUND_LAYERS[level]):
            z_base = geo.bin_height + layer * LAYER_STEP_M
            spread = geo.bin_inner * (0.7 - 0.15 * layer)
            for _ in range(count):
                item = rng.choice(items)
                a, r = rng.uniform(0, 2 * math.pi), spread * math.sqrt(rng.random())
                out["mound"].append(geo.placed(f"{name}_mound_{len(out['mound'])}", item,
                                               bx + r * math.cos(a), by + r * math.sin(a),
                                               z_base, "lying", rng.uniform(-math.pi, math.pi)))
    if geo.bin_shape == "cylinder":
        out["radius_m"] = geo.bin_half[0]
    taken = [(bx, by, geo.bin_inner)]
    for kind, count in (("spill", (level or 1) - 1), ("litter", litter)):
        for i in range(count):
            out[kind].append(place_on_floor(geo, rng, items, f"{name}_{kind}_{i}",
                                            (bx, by), (ux, uy), taken))
    return out


def place_on_floor(geo, rng, items, model, center, outward, taken):
    """An item lying on the floor beside the bin, on its outward side, clear of the route,
    the corners, the bin, and the other items."""
    base = math.atan2(outward[1], outward[0])
    for _ in range(500):
        item = rng.choice(items)
        foot = geo.footprint(item, "lying")
        a = base + math.radians(rng.uniform(-LITTER_SPREAD_DEG, LITTER_SPREAD_DEG))
        d = geo.bin_extent(a - base) + rng.uniform(*LITTER_GAP_M) + foot
        x, y = center[0] + d * math.cos(a), center[1] + d * math.sin(a)
        if geo.dist_to_route(x, y) < geo.half_width + geo.margin + foot:
            continue
        if geo.dist_to_nearest_corner(x, y) < geo.turn_radius + geo.margin + foot:
            continue
        if geo.dist_to_bin(x, y, center, base) < LITTER_GAP_M[0] + foot:
            continue
        if any(math.hypot(x - tx, y - ty) < tr + foot + 0.01 for tx, ty, tr in taken[1:]):
            continue
        taken.append((x, y, foot))
        return geo.placed(model, item, x, y, 0.0, "lying", rng.uniform(-math.pi, math.pi))
    raise ParkError(f"No room for {model}: too much litter for the space beside the bin.")


# ===== obstacle =====
def build_obstacle(geo, park, spec, seed):
    unknown = set(spec) - {"side", "at", "item", "pose"}
    if unknown:
        raise ParkError(f"obstacle: unknown keys {', '.join(sorted(unknown))}")
    rng = random.Random(f"{seed}-obstacle")
    side = spec.get("side", "random")
    side = rng.choice(CORNERS) if side == "random" else int(side)
    if side not in CORNERS:
        raise ParkError(f"obstacle side must be 1-4 or random, not {side}.")
    item = spec.get("item", "random")
    item = rng.choice(sorted(geo.items)) if item == "random" else item
    pose = spec.get("pose", "upright")
    if pose not in ("upright", "lying"):
        raise ParkError("obstacle pose must be upright or lying.")
    lo, hi = park["obstacle"].get("at_range", [0.25, 0.75])
    at = spec.get("at", "random")
    at = round(rng.uniform(lo, hi), 3) if at == "random" else float(at)
    # Far enough from both corners that the robot does not hit it while turning there
    clear = geo.turn_radius + geo.margin + geo.footprint(item, pose)
    if min(at, 1 - at) * geo.side < clear:
        raise ParkError(f"obstacle at {at:.2f} of side {side} is within {clear:.2f} m of a "
                        "corner, where the robot turns.")
    (x0, y0), (x1, y1) = geo.corner(side), geo.corner(geo.next_corner(side))
    x, y = x0 + at * (x1 - x0), y0 + at * (y1 - y0)
    out = geo.placed("obstacle", item, x, y, 0.0, pose, geo.side_heading(side) + math.pi / 2)
    out.update({"side": side, "at": at})
    return out


# ===== summary =====
def summary(layout):
    lines = [f"Scenario '{layout['scenario']}' (seed {layout['seed']})"
             + (f", overrides {layout['overrides']}" if layout["overrides"] else ""),
             f"Route: {layout['route']['side_m']} m square, start {layout['route']['start']}, "
             f"corners {' -> '.join(str(w['corner']) for w in layout['route']['waypoints'])}"]
    if not layout["bins"]:
        lines.append("Bins: none")
    for b in layout["bins"]:
        state = (f"overflowing (level {b['overflow_level']}, {len(b['mound'])} items over the "
                 f"rim, {len(b['spill'])} spilled)" if b["state"] == "overflowing" else "ok")
        litter = ", ".join(f"{i['item']}" for i in b["litter"]) or "none"
        size = " x ".join(f"{v * 100:.0f}" for v in b["size_m"])
        lines.append(f"  {b['name']} at ({b['position'][0]:.3f}, {b['position'][1]:.3f}), "
                     f"{b['shape']} {size} cm, face it at {b['view_heading_deg']:.0f} deg: "
                     f"{state}; litter: {litter}")
    o = layout["obstacle"]
    lines.append("Obstacle: none" if o is None else
                 f"Obstacle: {o['item']} ({o['category']}, {o['pose']}) on side {o['side']} at "
                 f"{o['at']:.2f} -> ({o['position'][0]:.3f}, {o['position'][1]:.3f})")
    return "\n".join(lines)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--scenario", default="default", help="name in config/scenarios/ or a path")
    p.add_argument("--bins", type=int, help="keep the first N corners' bins")
    p.add_argument("--obstacle", help="none | random | 1-4")
    p.add_argument("--seed", type=int)
    p.add_argument("--yaml", action="store_true", help="print the full layout as YAML")
    a = p.parse_args(argv)
    try:
        layout = load_park(a.scenario, a.bins, a.obstacle, a.seed)
    except ParkError as e:
        raise SystemExit(f"park: {e}")
    print(yaml.safe_dump(layout, sort_keys=False) if a.yaml else summary(layout))


if __name__ == "__main__":
    sys.exit(main())
