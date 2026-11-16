"""Build the Gazebo park world from walle-app's park config and a scenario.

    python3 walle-sim/worlds/park_world.py                         # default scenario
    python3 walle-sim/worlds/park_world.py --scenario mixed --bins 3 --obstacle 2 --seed 5

launch/sim.launch.py runs this automatically for world:=park. It writes, into the
git-ignored worlds/generated/ folder:

    park.sdf         the world: tape square, corner marks, start pad, bins, overflow
                     mounds, litter, and the obstacle
    park_info.yaml   what harness/sim_world.py needs (spawn, view, anchors) plus the
                     resolved layout from walle-app/park.py, which is the ground truth

The layout comes only from walle-app/park.py, so the simulator and the robot read the
same park. Scenarios are the committed files; the generated world is rebuilt every launch.
"""

import argparse
import math
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(os.path.dirname(HERE))
GENERATED = os.path.join(HERE, "generated")
WORLD_NAME = "park"
sys.path[:0] = [HERE, os.path.join(REPO_DIR, "walle-app")]

import park  # noqa: E402  walle-app/park.py
from generate_worlds import FOOTER, TAPE, header, mark, material  # noqa: E402

BIN_RGB = [0.15, 0.45, 0.20]       # green plastic bin
OPENING_RGB = [0.05, 0.05, 0.05]   # the dark opening seen from above
START_RGB = [0.2, 0.6, 0.3]
CORNER_RGB = [0.8, 0.1, 0.1]


def bin_model(b):
    x, y = b["position"]
    r, h = b["radius_m"], b["height_m"]
    geom = f"<cylinder><radius>{r}</radius><length>{h}</length></cylinder>"
    return f"""
    <!-- {b['name']}: {b['state']}{' level %d' % b['overflow_level'] if b['overflow_level'] else ''} -->
    <model name="{b['name']}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {h / 2:.4f} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry>{geom}</geometry></collision>
        <visual name="visual">
          <geometry>{geom}</geometry>
          {material(BIN_RGB)}
        </visual>
        <visual name="opening">
          <pose>0 0 {h / 2 + 0.0005:.4f} 0 0 0</pose>
          <geometry><cylinder><radius>{r - 0.008:.4f}</radius><length>0.001</length></cylinder></geometry>
          {material(OPENING_RGB)}
        </visual>
      </link>
    </model>"""


def item_model(placed, items):
    it = items[placed["item"]]
    x, y, z = placed["position"]
    yaw = math.radians(placed["yaw_deg"])
    if it["shape"] == "sphere":
        geom, roll = f"<sphere><radius>{it['radius_m']}</radius></sphere>", 0.0
    else:
        geom = (f"<cylinder><radius>{it['radius_m']}</radius>"
                f"<length>{it['length_m']}</length></cylinder>")
        roll = math.pi / 2 if placed["pose"] == "lying" else 0.0
    return f"""
    <!-- {placed['item']} ({placed['category']}) -->
    <model name="{placed['name']}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} {z:.4f} {roll:.5f} 0 {yaw:.5f}</pose>
      <link name="link">
        <collision name="collision"><geometry>{geom}</geometry></collision>
        <visual name="visual">
          <geometry>{geom}</geometry>
          {material(it['rgb'])}
        </visual>
      </link>
    </model>"""


def route_marks(layout):
    corners = layout["route"]["corners"]
    side = layout["route"]["side_m"]
    parts = []
    for c in (1, 2, 3, 4):
        (x0, y0), (x1, y1) = corners[c], corners[c % 4 + 1]
        yaw = math.atan2(y1 - y0, x1 - x0)
        parts.append(mark(f"route_side_{c}", (x0 + x1) / 2, (y0 + y1) / 2, yaw, side, 0.012))
        parts.append(mark(f"route_corner_{c}", x0, y0, 0.0, 0.03, 0.03, CORNER_RGB))
    sx, sy, syaw = layout["route"]["start"]
    parts.append(mark("start_pad", sx - 0.06, sy, math.radians(syaw), 0.22, 0.20, START_RGB)
                 .replace("0.0005 0 0", "0.0002 0 0"))   # under the tape
    return "".join(parts)


def all_items(layout):
    out = [i for b in layout["bins"] for kind in ("mound", "spill", "litter") for i in b[kind]]
    if layout["obstacle"]:
        out.append(layout["obstacle"])
    return out


def build(scenario="default", bins=None, obstacle=None, seed=None, out_dir=GENERATED):
    """Write park.sdf and park_info.yaml; return (sdf path, info dict)."""
    layout = park.load_park(scenario, bins, obstacle, seed)
    items = park.load_yaml(park.PARK_FILE)["items"]
    body = route_marks(layout)
    body += "".join(bin_model(b) for b in layout["bins"])
    body += "".join(item_model(i, items) for i in all_items(layout))

    os.makedirs(out_dir, exist_ok=True)
    sdf = os.path.join(out_dir, f"{WORLD_NAME}.sdf")
    with open(sdf, "w") as f:
        f.write(header(WORLD_NAME).replace(
            "generate_worlds.py: edit that script", "park_world.py from walle-app config: edit "
            "the scenario") + body + FOOTER)

    side = layout["route"]["side_m"]
    info = {"worlds": {WORLD_NAME: {
        "spawn": layout["route"]["start"],
        "view": {"target": [side / 2, side / 2, 0.0], "offset": [-0.9, -1.6, 1.8]},
        "anchors": {"start": layout["route"]["start"],
                    **{f"corner_{c}": [*xy, 0.0] for c, xy in layout["route"]["corners"].items()}},
        "park": layout,
    }}}
    with open(os.path.join(out_dir, f"{WORLD_NAME}_info.yaml"), "w") as f:
        f.write("# Generated by park_world.py: rebuilt every launch of world:=park\n")
        yaml.safe_dump(info, f, sort_keys=False)
    return sdf, info


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--scenario", default="default")
    p.add_argument("--bins", type=int)
    p.add_argument("--obstacle")
    p.add_argument("--seed", type=int)
    p.add_argument("--out", default=GENERATED, help="output folder (default worlds/generated)")
    a = p.parse_args()
    try:
        sdf, info = build(a.scenario, a.bins, a.obstacle, a.seed, a.out)
    except park.ParkError as e:
        raise SystemExit(f"park: {e}")
    print(park.summary(info["worlds"][WORLD_NAME]["park"]))
    print(f"wrote {os.path.relpath(sdf, REPO_DIR)} and {WORLD_NAME}_info.yaml")


if __name__ == "__main__":
    main()
