"""The bin check (ROADMAP Phase 4): judge a bin from its bin-stop photos. Pure Python, no ROS.

At each bin stop, after centering, the follower has two analysed photos
(geometry.analyze_bin_stop):
- the LEVEL look (the dwell photo): litter on the floor around the bin;
- the UP look (camera tilted servo.yaml up_deg): what is above the rim.
Each count comes from one photo, so nothing is counted twice. Without an up photo (no servo,
or it failed) the level photo's on_bin items are used instead (method level_only); at 0.88 m
the level camera cuts off most of what is on top, so that undercounts.

The bins are opaque: there is no fill level. A check records
    overflow_level  0 (nothing above the rim) .. 3, the larger of the level by count of items
                    on top and the level by the highest item's height above the rim (the
                    height rule is off in detect.yaml: it doesn't separate the levels);
    on_top_count, litter_count, outcome (overflowing / litter / clean, as memory/store.py).
Settings: config/detect.yaml bin_check.
"""

import math

from detect import geometry as g

DEFAULTS = {
    "count_levels": [1, 4, 7],       # this many items on top -> level 1, 2, 3
    "height_levels_m": [None, None, None],    # highest item this far above the rim -> 1, 2, 3
    "litter_radius_m": 0.40,         # floor items within this of the bin's sides are litter
    "min_conf": 0.0,                 # items below this confidence are ignored
}


def settings(cfg):
    out = dict(DEFAULTS)
    out.update((cfg or {}).get("bin_check", {}))
    return out


def level_from(value, thresholds):
    """The highest level (1-based position in thresholds) whose threshold value reaches;
    0 if none. A None threshold is never reached (that level comes from the other rule)."""
    if value is None:
        return 0
    return max([i + 1 for i, t in enumerate(thresholds) if t is not None and value >= t],
               default=0)


def dist_to_footprint(pt, b):
    """Distance (m) from a park-frame floor point to the bin's footprint (0 inside it)."""
    bx, by = b["position"][:2]
    if b.get("shape") == "box":
        yaw = math.radians(b.get("yaw_deg", 0.0))
        dx, dy = pt[0] - bx, pt[1] - by
        lx = math.cos(yaw) * dx + math.sin(yaw) * dy        # along the bin's depth
        ly = -math.sin(yaw) * dx + math.cos(yaw) * dy       # along its wide face
        hx, hy = b["size_m"][1] / 2.0, b["size_m"][0] / 2.0
        return math.hypot(max(abs(lx) - hx, 0.0), max(abs(ly) - hy, 0.0))
    return max(math.hypot(pt[0] - bx, pt[1] - by) - b.get("radius_m", 0.1), 0.0)


def litter_items(level_an, b, cfg):
    """(counted, not counted) around_bin items of the level look: counted = on the floor
    (obj_xy known) within litter_radius_m of the bin's sides."""
    s = settings(cfg)
    counted, other = [], []
    for d in (level_an or {}).get("items", []):
        if d.get("place") != "around_bin" or (d.get("conf") or 1.0) < s["min_conf"]:
            continue
        xy = d.get("obj_xy")
        if xy is not None and dist_to_footprint(xy, b) <= s["litter_radius_m"]:
            counted.append(d)
        else:
            other.append(d)
    return counted, other


def on_top_items(an, cam, b, dist_m, cfg, edge_px=4):
    """on_bin items of a look, each with height_above_rim_m (from its box's top edge, at the
    bin's distance) and cut_top (its top runs off the frame: the height is a lower bound)."""
    s = settings(cfg)
    bin_h = b.get("height_m") or (b["size_m"][2] if b.get("size_m") else 0.29)
    out = []
    for d in (an or {}).get("items", []):
        if d.get("place") != "on_bin" or (d.get("conf") or 1.0) < s["min_conf"]:
            continue
        x1, y1, x2, _ = d["bbox"]
        z = cam.height_at(y1, dist_m, (x1 + x2) / 2.0) if cam else None
        out.append({"label": d.get("label"), "category": d.get("category"),
                    "conf": d.get("conf"), "bbox": d["bbox"],
                    "height_above_rim_m": None if z is None else round(z - bin_h, 3),
                    "cut_top": y1 <= edge_px})
    return out


def bin_distance_from_width(an, cam, b, edge_px=4):
    """The bin's distance from the lens (m) from its box width, if no side is cut off."""
    chosen = (an or {}).get("bin")
    if not chosen or cam is None:
        return None
    if g.cut_edges(chosen["bbox"], cam, edge_px) & {"left", "right"}:
        return None
    width_m = b["size_m"][0] if b.get("shape") == "box" else 2 * b.get("radius_m", 0.1)
    w = chosen["bbox"][2] - chosen["bbox"][0]
    return round(cam.fx * width_m / w, 3) if w > 0 else None


def judge_bin(level_an, up_an, b, cfg, level_cam=None, up_cam=None, dist_m=None, edge_px=4):
    """Judge one bin. level_an / up_an: analyze_bin_stop results (up_an None = no up photo).
    b: the bin from the layout (position, size_m / radius_m, height_m, yaw_deg).
    dist_m: the bin center's distance ahead of the lens (the follower's expected_dist_m).
    Returns the record for bin_checks.json (the store's fields + what they came from)."""
    s = settings(cfg)
    method = "tilt" if up_an is not None else "level_only"
    top_an, top_cam = (up_an, up_cam) if up_an is not None else (level_an, level_cam)
    on_top = on_top_items(top_an, top_cam, b, dist_m, cfg, edge_px) if dist_m else []
    litter, other = litter_items(level_an, b, cfg)
    heights = [d["height_above_rim_m"] for d in on_top if d["height_above_rim_m"] is not None]
    top_h = max(heights) if heights else None
    by_count = level_from(len(on_top), s["count_levels"])
    by_height = level_from(top_h, s["height_levels_m"]) if on_top else 0
    level = min(3, max(by_count, by_height))
    outcome = "overflowing" if level > 0 else ("litter" if litter else "clean")
    return {
        "method": method, "overflow_level": level, "on_top_count": len(on_top),
        "litter_count": len(litter), "outcome": outcome,
        "level_by_count": by_count, "level_by_height": by_height,
        "top_height_above_rim_m": top_h,
        "top_cut_off": any(d["cut_top"] for d in on_top),
        "rim_visible": (top_an or {}).get("rim_visible"),
        "bin_found": {"level": bool((level_an or {}).get("bin")),
                      "up": None if up_an is None else bool(up_an.get("bin"))},
        "on_top": on_top,
        "litter": [{k: d.get(k) for k in ("label", "category", "conf", "bbox", "obj_xy")}
                   for d in litter],
        "not_litter": [{k: d.get(k) for k in ("label", "category", "conf", "bbox", "obj_xy")}
                       for d in other],
        "dist_from_width_m": bin_distance_from_width(level_an, level_cam, b, edge_px),
    }


def expected_from_layout(b):
    """What a sim bin truly is (park.py layout): the values the judge should give."""
    level = int(b.get("overflow_level") or 0)
    litter = len(b.get("litter", [])) + len(b.get("spill", []))
    return {"overflow_level": level, "on_top_count": len(b.get("mound", [])),
            "litter_count": litter,
            "outcome": "overflowing" if level > 0 else ("litter" if litter else "clean")}
