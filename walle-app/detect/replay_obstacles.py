"""Score a route run's obstacle records against the truth (ROADMAP Phase 5).

Reads <run>/obstacles.json (written by the route follower) and compares each obstacle's
recorded position with where it really was:
- Gazebo runs carry the truth (the world's obstacle) in the file;
- robot runs take it from --truth "SIDE,ALONG_M[,CROSS_CM]": the side (1-4, the corner it
  starts from), the tape-measured distance of the object's CENTER from that corner along
  the side (m), and optionally how far LEFT of the tape line it was (cm, + = inside).

    python3 walle-app/detect/replay_obstacles.py walle-app/results/route/20261123_101500
    python3 walle-app/detect/replay_obstacles.py RUN --truth "2,0.52" --truth "3,0.35,2"

Prints each record (found by the corner look or the sonar, position from the camera or the
sonar, the action) and its error, and writes <run>/obstacle_replay.json.
"""

import argparse
import json
import math
import os
import sys

APP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [APP, os.path.join(APP, "route")]
import obstacle as ob  # noqa: E402
import park  # noqa: E402


def parse_truth(text, layout):
    parts = [p.strip() for p in text.split(",")]
    side, along = int(parts[0]), float(parts[1])
    cross = float(parts[2]) / 100.0 if len(parts) > 2 else 0.0
    corners = {int(k): tuple(v) for k, v in layout["route"]["corners"].items()}
    a, b = corners[side], corners[side % 4 + 1]
    return {"side": side, "along_m": along, "position": list(ob.from_side(along, cross, a, b))}


def score(data, truths, tol_cm=10.0):
    rows, used = [], set()
    for o in data.get("obstacles", []):
        if o.get("action") == "not_confirmed":
            continue                    # a far sighting a closer look didn't confirm
        row = {k: o.get(k) for k in ("id", "side", "label", "category", "found_by",
                                     "position_source", "action", "obj_xy", "camera_xy",
                                     "sonar_xy", "radius_m")}
        cands = [(i, t) for i, t in enumerate(truths) if t["side"] == o.get("side")]
        if o.get("obj_xy") and cands:
            i, t = min(cands, key=lambda it: math.dist(it[1]["position"], o["obj_xy"]))
            used.add(i)
            row["truth"] = t["position"]
            for key in ("obj_xy", "camera_xy", "sonar_xy"):
                if o.get(key):
                    row[f"{key.split('_')[0]}_error_cm"] = round(
                        math.dist(o[key], t["position"]) * 100, 1)
            row["within_tol"] = row["obj_error_cm"] <= tol_cm
        else:
            row["false_obstacle"] = True
        rows.append(row)
    missed = [t for i, t in enumerate(truths) if i not in used]
    return rows, missed


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("run", help="a route run folder (has obstacles.json)")
    p.add_argument("--truth", action="append", default=[],
                   help='robot: "SIDE,ALONG_M[,CROSS_CM]" per real obstacle (repeat)')
    p.add_argument("--tol-cm", type=float, default=10.0)
    a = p.parse_args(argv)
    path = os.path.join(a.run, "obstacles.json")
    if not os.path.exists(path):
        raise SystemExit(f"No obstacles.json in {a.run} (obstacles off, or the run never "
                         "ended: robot results are written after the measurement prompts).")
    with open(path) as f:
        data = json.load(f)
    layout_path = os.path.join(a.run, "layout.json")
    layout = (json.load(open(layout_path)) if os.path.exists(layout_path)
              else park.load_park("default"))
    if a.truth:
        truths = [parse_truth(t, layout) for t in a.truth]
    elif data.get("truth"):
        t = data["truth"]
        truths = [{"side": t["side"], "along_m": t["at"] * layout["route"]["side_m"],
                   "position": t["position"][:2]}]
    else:
        truths = []
        print("No truth: Gazebo writes it; for a robot run give --truth.")
    rows, missed = score(data, truths, a.tol_cm)
    print(f"Run {data.get('run_id')} ({data.get('source')}): sides driven "
          f"{data.get('sides_done')}, {len(rows)} obstacle record(s)")
    for r in rows:
        err = (f"error {r['obj_error_cm']:.1f} cm"
               + (f" (camera {r['camera_error_cm']:.1f}" if "camera_error_cm" in r else " (")
               + (f", sonar {r['sonar_error_cm']:.1f}" if "sonar_error_cm" in r else "") + ")"
               if "obj_error_cm" in r else "no matching truth")
        print(f"  #{r['id']} side {r['side']}: {r['label']} ({r['category']}), found by the "
              f"{r['found_by']}, position from the {r['position_source']}, {r['action']}; "
              f"{err}")
    for t in missed:
        print(f"  MISSED: the obstacle on side {t['side']} at {t['along_m']:.2f} m")
    ok = rows and not missed and all(r.get("within_tol") for r in rows)
    out = {"run": a.run, "tol_cm": a.tol_cm, "truth": truths, "records": rows,
           "missed": missed, "pass": bool(ok)}
    with open(os.path.join(a.run, "obstacle_replay.json"), "w") as f:
        json.dump(out, f, indent=1)
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
