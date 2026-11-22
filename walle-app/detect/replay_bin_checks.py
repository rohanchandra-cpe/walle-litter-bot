"""Re-judge a route run's bin checks with the current rules (ROADMAP Phase 4). No model, no ROS.

Reads <run>/detections.json and layout.json, rebuilds each photo's detections from what the
run saved (the chosen bin box, the items, the faint low_conf boxes), runs today's
geometry.analyze_bin_stop + bin_check.judge_bin, and prints them beside the run's own
bin_checks.json and, with --truth, the ground truth. The detector is NOT run again, so a
change is down to the judging rules alone. (Bin boxes the run saw but did not choose are
not in detections.json; they can't be replayed.)

    python3 walle-app/detect/replay_bin_checks.py walle-app/results/route/20261122_134315 \
        --truth "1:0,0,0" "2:1,1,0" "3:1,2,3" "4:1,1,3"      # bin:level,on_top,litter
"""

import argparse
import json
import math
import os
import sys

APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, APP_DIR)

import park  # noqa: E402
from detect import bin_check as bc  # noqa: E402
from detect import geometry as g  # noqa: E402
from detect.detector import load_config  # noqa: E402
from servo.tilt import Calibration  # noqa: E402


def raw_detections(photo):
    """The photo's detections as the detector gave them (as far as the run saved them)."""
    dets = []
    if photo.get("bin_detection"):
        dets.append({k: v for k, v in photo["bin_detection"].items() if k != "faint"})
    for it in photo.get("items", []) + photo.get("ignored", []):
        dets.append({k: it.get(k) for k in ("kind", "label", "category", "conf", "bbox",
                                             "prompt", "truth", "truth_group") if k in it})
    return dets


def analyze(photo, b, cam_cfg, pivot, cfg):
    w, h = photo.get("size") or (640, 480)
    tilt = photo.get("tilt_deg") or 0.0
    cam = g.Camera.from_park(cam_cfg, w, h, tilt_deg=tilt, lens_from_pivot_m=pivot)
    x, y, hd = photo["pose"]
    hr = math.radians(hd)
    lx = x + cam_cfg["forward_m"] * math.cos(hr) - cam_cfg.get("left_m", 0.0) * math.sin(hr)
    ly = y + cam_cfg["forward_m"] * math.sin(hr) + cam_cfg.get("left_m", 0.0) * math.cos(hr)
    dx, dy = b["position"][0] - lx, b["position"][1] - ly
    bearing = math.degrees(math.atan2(dy, dx)) - hd
    bearing = (bearing + 180.0) % 360.0 - 180.0
    dist = max(math.hypot(dx, dy), 0.1)
    width_m = b["size_m"][0] if b.get("shape") == "box" else 2 * b.get("radius_m", 0.1)
    height_m = b.get("height_m") or b["size_m"][2]
    expected = (cam.expected_box_px(width_m, dist, 0.0, height_m) if tilt else
                (cam.width_px(width_m, dist), cam.fy * height_m / dist))
    an = g.analyze_bin_stop(raw_detections(photo), cam, pose=photo["pose"],
                            expected_bearing_deg=bearing, expected_bin_px=expected, cfg=cfg,
                            stop={"bin_dist_m": dist, "bin_height_m": height_m,
                                  "low_conf_bins": photo.get("low_conf", [])})
    return an, cam, dist


def parse_truth(items):
    out = {}
    for t in items or []:
        bin_id, _, vals = t.partition(":")
        level, top, litter = (int(v) for v in vals.split(","))
        out[int(bin_id)] = {"overflow_level": level, "on_top_count": top,
                            "litter_count": litter,
                            "outcome": "overflowing" if level else
                            ("litter" if litter else "clean")}
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("run_dir")
    p.add_argument("--truth", nargs="*", help="bin:level,on_top,litter for each bin")
    a = p.parse_args(argv)
    cfg = load_config()
    with open(os.path.join(a.run_dir, "detections.json")) as f:
        photos = json.load(f)["photos"]
    with open(os.path.join(a.run_dir, "layout.json")) as f:
        bins = {b["name"]: b for b in json.load(f)["bins"]}
    old = {}
    path = os.path.join(a.run_dir, "bin_checks.json")
    if os.path.exists(path):
        with open(path) as f:
            old = {(c["step"], c["bin"]): c for c in json.load(f)["checks"]}
    source = (json.load(open(os.path.join(a.run_dir, "summary.json"))).get("source")
              if os.path.exists(os.path.join(a.run_dir, "summary.json")) else "robot")
    cam_cfg = park.load_yaml(park.PARK_FILE)["robot"]["camera"]
    pivot = Calibration.load(sim=source == "sim").lens_from_pivot_m
    truth = parse_truth(a.truth)

    stops = {}
    for ph in photos:
        stops.setdefault((ph["step"], ph["bin"]), {})[ph.get("look", "level")] = ph
    keys = ("outcome", "overflow_level", "on_top_count", "litter_count")
    right = {"old": 0, "new": 0}
    print(f"{'bin':6s} {'run (old rules)':32s} {'replay (new rules)':32s} truth")
    for (step, name), looks in sorted(stops.items()):
        b = bins[name]
        lvl, lcam, dist = analyze(looks["level"], b, cam_cfg, pivot, cfg)
        up, ucam = None, None
        if "up" in looks:
            up, ucam, _ = analyze(looks["up"], b, cam_cfg, pivot, cfg)
        rec = bc.judge_bin(lvl, up, b, cfg, level_cam=lcam, up_cam=ucam, dist_m=dist,
                           edge_px=cfg["centering"].get("edge_px", 4))
        o = old.get((step, name), {})
        fmt = lambda c: (f"{c.get('outcome')} L{c.get('overflow_level')} "  # noqa: E731
                         f"top {c.get('on_top_count')} lit {c.get('litter_count')}")
        t = truth.get(b["id"])
        line = f"{name:6s} {fmt(o) if o else '-':32s} {fmt(rec):32s} {fmt(t) if t else '-'}"
        if t:
            for k, c in (("old", o), ("new", rec)):
                right[k] += c.get("outcome") == t["outcome"]
        print(line)
        notes = []
        if (up or {}).get("bin", {}) and up["bin"].get("faint"):
            notes.append(f"up look: faint bin used (conf {up['bin'].get('conf')})")
        for an, look in ((lvl, "level"), (up, "up")):
            for it in (an or {}).get("ignored", []):
                notes.append(f"{look}: ignored {it['label']} ({it['ignored']})")
            for it in (an or {}).get("items", []):
                if it.get("obj_xy_from") == "frame_bottom":
                    notes.append(f"{look}: {it['label']} cut at the frame bottom, placed at "
                                 f"{it['obj_xy']}")
        for n in notes:
            print(f"         - {n}")
    if truth:
        n = len(truth)
        print(f"\noutcome right: run {right['old']}/{n}, replay {right['new']}/{n}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
