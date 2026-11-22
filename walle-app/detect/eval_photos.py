"""Score the detector on the labeled ESP32-CAM photo set (ROADMAP Phase 3). Headless.

    python3 walle-app/detect/eval_photos.py                    # data/photos/phase3, config model
    python3 walle-app/detect/eval_photos.py --model yolov8m-worldv2.pt
    make detect-eval

Reads labels.yaml (written by capture_set.py) and runs the detector on every photo. Labels are
lists, not boxes, so items are matched by count:
- recall / precision: per photo and label, found = min(reported, true);
- category: per photo and category, right = min(reported, true), divided by the number of
  items both sides can be matched on (min of reported and true totals);
- bin found: photos that contain a bin and in which a bin is detected;
- place (on_bin / around_bin, report only): for matched labels, the place the follower would
  record;
- bearing (report only): bin-shift shots, detected bin bearing vs atan(shift / distance).
Prints PASS or FAIL against config/detect.yaml `eval`, and writes report.json and
per_photo.csv to results/detect/<time>/. Exit code 0 = PASS, 1 = FAIL.

Phase 4 (--set-dir walle-app/data/photos/phase4 --by-light): a photo's tilt_deg goes into the
camera geometry, and a photo with a `look` is scored as the bin check uses it: an up look on
the items on top of the bin (on_bin), a level look on the items on the floor (around_bin,
route), both on the truth and on what was reported. --by-light also requires recall,
precision and category to pass in EACH light (day, night) on its own.
"""

import argparse
import csv
import json
import math
import os
import statistics
import sys
import time
from collections import Counter

import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from detect import geometry as g  # noqa: E402
from detect.detector import APP_DIR, load_config, make_detector  # noqa: E402

SET_DIR = os.path.join(APP_DIR, "data", "photos", "phase3")


LOOK_PLACES = {"up": {"on_bin"}, "level": {"around_bin", "route"}}


def score_photo(truth, dets, cam, cfg):
    """Counts for one photo. truth: labels.yaml entry; dets: detector output list."""
    analysis = g.analyze_bin_stop(dets, cam, cfg=cfg) if truth.get("bin") else None
    items = analysis["items"] if analysis else [d for d in dets if d["kind"] == "item"]
    t_items = truth["items"]
    places = LOOK_PLACES.get(truth.get("look"))
    if places:                       # Phase 4: score what the bin check takes from this look
        t_items = [i for i in t_items if i["place"] in places]
        items = [i for i in items if i.get("place", "route") in places]
    truth = dict(truth, items=t_items)
    t_lab = Counter(i["label"] for i in truth["items"])
    p_lab = Counter(i["label"] for i in items)
    found = {lab: min(n, p_lab[lab]) for lab, n in t_lab.items()}
    t_cat = Counter(cfg["items"].get(i["label"], {}).get("category", "unknown")
                    for i in truth["items"])
    p_cat = Counter(i["category"] or "unknown" for i in items)
    out = {"id": truth["id"], "file": truth["file"],
           "true": dict(t_lab), "reported": dict(p_lab),
           "true_items": sum(t_lab.values()), "reported_items": sum(p_lab.values()),
           "found": sum(found.values()),
           "category_right": sum(min(n, p_cat[c]) for c, n in t_cat.items()),
           "category_matchable": min(sum(t_lab.values()), sum(p_lab.values())),
           "light": truth.get("light"), "look": truth.get("look"),
           "has_bin": bool(truth.get("bin")), "bin_found": None, "bin_bearing_deg": None,
           "expected_bearing_deg": None, "rim_visible": None,
           "place_right": 0, "place_checked": 0, "missed": [], "extra": []}
    out["missed"] = sorted((t_lab - p_lab).elements())
    out["extra"] = sorted((p_lab - t_lab).elements())
    if analysis:
        out["bin_found"] = analysis["bin"] is not None
        out["bin_bearing_deg"] = analysis["bin_bearing_deg"]
        out["rim_visible"] = analysis["rim_visible"]
        shift = truth.get("bin_shift_cm") or 0
        out["expected_bearing_deg"] = round(math.degrees(math.atan(
            shift / 100.0 / cfg["eval"]["bin_distance_m"])), 2)
        t_place = Counter((i["label"], i["place"]) for i in truth["items"])
        p_place = Counter((i["label"], i["place"]) for i in items)
        out["place_right"] = sum(min(n, p_place[k]) for k, n in t_place.items())
        out["place_checked"] = sum(found.values())
    return out


def summarize(rows, seconds, targets):
    def ratio(a, b):
        return round(a / b, 3) if b else None
    tot = lambda k: sum(r[k] for r in rows)  # noqa: E731
    bins = [r for r in rows if r["has_bin"]]
    s = {"photos": len(rows),
         "recall": ratio(tot("found"), tot("true_items")),
         "precision": ratio(tot("found"), tot("reported_items")),
         "category": ratio(tot("category_right"), tot("category_matchable")),
         "bin_found": ratio(sum(1 for r in bins if r["bin_found"]), len(bins)),
         "place": ratio(tot("place_right"), tot("place_checked")),
         "latency_median_s": round(statistics.median(seconds), 3) if seconds else None,
         "latency_p90_s": round(sorted(seconds)[int(0.9 * (len(seconds) - 1))], 3)
         if seconds else None}
    per_label = {}
    for r in rows:
        for lab, n in r["true"].items():
            d = per_label.setdefault(lab, {"true": 0, "found": 0, "reported": 0})
            d["true"] += n
            d["found"] += min(n, r["reported"].get(lab, 0))
        for lab, n in r["reported"].items():
            per_label.setdefault(lab, {"true": 0, "found": 0, "reported": 0})["reported"] += n
    s["per_label"] = dict(sorted(per_label.items()))
    errs = [abs(r["bin_bearing_deg"] - r["expected_bearing_deg"]) for r in bins
            if r["bin_found"] and r["bin_bearing_deg"] is not None]
    s["bin_bearing_abs_error_deg"] = {"median": round(statistics.median(errs), 2),
                                      "max": round(max(errs), 2)} if errs else None
    s["checks"] = {}
    for key, target in targets.items():
        if key == "bin_distance_m":
            continue
        value = s.get(key)
        ok = value is not None and (value <= target if key.startswith("latency") else
                                    value >= target)
        s["checks"][key] = {"value": value, "target": target, "pass": ok}
    s["pass"] = all(c["pass"] for c in s["checks"].values())
    return s


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--set-dir", default=SET_DIR)
    p.add_argument("--model", help="weights in walle-app/models/ (default: config)")
    p.add_argument("--out", default=os.path.join(APP_DIR, "results", "detect"))
    p.add_argument("--method", choices=("vlm", "yolo"), help="default: config method")
    p.add_argument("--vlm-model", help="Ollama model for --method vlm (default: config)")
    p.add_argument("--ids", help="only these shots, e.g. 1-29 or 30-42 or 3,7,9")
    p.add_argument("--zoom", action="store_true", help="turn on vlm.zoom_bin_top for this run")
    p.add_argument("--by-light", action="store_true",
                   help="recall, precision and category must pass in each light on its own")
    a = p.parse_args(argv)

    cfg = load_config()
    if a.zoom:
        cfg["vlm"].setdefault("zoom_bin_top", {})["enabled"] = True
    with open(os.path.join(a.set_dir, "labels.yaml")) as f:
        photos = yaml.safe_load(f)["photos"]
    park_cam = yaml.safe_load(open(os.path.join(APP_DIR, "config", "park.yaml")))["robot"]["camera"]
    set_cam = os.path.join(a.set_dir, "camera.yaml")     # where the camera was for THIS set
    if os.path.exists(set_cam):
        with open(set_cam) as f:
            park_cam = dict(park_cam, **(yaml.safe_load(f) or {}))
    from servo.tilt import Calibration
    pivot = Calibration.load().lens_from_pivot_m
    if a.ids:
        keep = set()
        for part in a.ids.split(","):
            lo, _, hi = part.partition("-")
            keep.update(range(int(lo), int(hi or lo) + 1))
        photos = [p for p in photos if p["id"] in keep]
    kw = {"model": a.model}
    if (a.method or cfg.get("method")) == "vlm" and a.vlm_model:
        kw["vlm_model"] = a.vlm_model
    det = make_detector(cfg=cfg, method=a.method, **kw)
    rows, seconds, raw = [], [], {}
    for truth in photos:
        res = det.detect(os.path.join(a.set_dir, truth["file"]), keep_low=True)
        seconds.append(res["seconds"])
        raw[truth["file"]] = res
        w, h = res.get("size") or (640, 480)
        cam = g.Camera.from_park(park_cam, w, h, tilt_deg=truth.get("tilt_deg") or 0.0,
                                 lens_from_pivot_m=pivot)
        r = score_photo(truth, res["detections"], cam, cfg)
        r["seconds"] = res["seconds"]
        rows.append(r)
        flag = "ok " if not (r["missed"] or r["extra"]) and r["bin_found"] is not False else "-- "
        print(f"{flag}{truth['file']}: found {r['found']}/{r['true_items']}"
              + (f", missed {r['missed']}" if r["missed"] else "")
              + (f", extra {r['extra']}" if r["extra"] else "")
              + ("" if r["bin_found"] is None else
                 f", bin {'at %+.1f deg' % r['bin_bearing_deg'] if r['bin_found'] else 'NOT found'}"
                 + (f" (expected {r['expected_bearing_deg']:+.1f})"
                    if r["expected_bearing_deg"] else ""))
              + f"  [{res['seconds']:.2f} s]")
    s = summarize(rows, seconds, cfg["eval"])
    s["by_light"], s["by_look"] = {}, {}
    for key, group in (("by_light", "light"), ("by_look", "look")):
        for val in sorted({r[group] for r in rows if r.get(group)}):
            sub = [r for r in rows if r.get(group) == val]
            sub_s = summarize(sub, [r["seconds"] for r in sub], cfg["eval"])
            s[key][val] = {k: sub_s[k] for k in ("photos", "recall", "precision", "category")}
            if key == "by_light" and a.by_light:
                for k in ("recall", "precision", "category"):
                    target = cfg["eval"].get(k)
                    if target is not None:
                        s["checks"][f"{k} ({val})"] = {
                            "value": sub_s[k], "target": target,
                            "pass": sub_s[k] is not None and sub_s[k] >= target}
    s["pass"] = all(c["pass"] for c in s["checks"].values())
    s.update({"model": det.model_name, "ids": a.ids, "set": os.path.relpath(a.set_dir, APP_DIR),
              "config": {k: cfg[k] for k in ("min_conf", "iou", "imgsz")} |
              {"bin_min_conf": cfg["bin"]["min_conf"]}})

    out_dir = os.path.join(a.out, time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "report.json"), "w") as f:
        json.dump({"summary": s, "photos": rows, "detections": raw}, f, indent=1)
    with open(os.path.join(out_dir, "per_photo.csv"), "w", newline="") as f:
        cols = ["id", "file", "true_items", "reported_items", "found", "category_right",
                "category_matchable", "has_bin", "bin_found", "bin_bearing_deg",
                "expected_bearing_deg", "rim_visible", "place_right", "place_checked",
                "seconds"]
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)

    print(f"\n{s['photos']} photos, model {s['model']}")
    for lab, d in s["per_label"].items():
        print(f"  {lab:18s} found {d['found']}/{d['true']}, reported {d['reported']}")
    for key in ("by_light", "by_look"):
        for val, d in s[key].items():
            print(f"  {key[3:]} {val:6s}: {d['photos']} photos, recall {d['recall']}, "
                  f"precision {d['precision']}, category {d['category']}")
    print(f"  place right (report only): {s['place']}; bin bearing error: "
          f"{s['bin_bearing_abs_error_deg']}; latency p90 {s['latency_p90_s']} s")
    for key, c in s["checks"].items():
        print(f"  {'PASS' if c['pass'] else 'FAIL'}  {key:17s} {c['value']}  (target {c['target']})")
    print(f"{'PASS' if s['pass'] else 'FAIL'}  ({os.path.relpath(out_dir, APP_DIR)})")
    return 0 if s["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
