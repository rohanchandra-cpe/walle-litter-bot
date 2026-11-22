"""Turn a route follower run log (summary.json + layout.json) into memory store records.

Used by the route follower at the end of each run and by import_runs.py to rebuild a store
from the committed run logs. A Phase 1 run records the run itself and one bin check per bin
stop: the photo and the pose, not yet assessed (method photo_only; Phase 4 assesses bins).
Since Phase 3 a run folder may hold detections.json (the follower's YOLO-World results at the
bin stops); each item in it becomes one detection (on_bin / around_bin, with the bin).
Since Phase 4 a run folder may hold bin_checks.json (the judged bin checks, detect/bin_check.py):
then a bin stop's check carries its overflow level, counts and outcome (method tilt or
level_only) instead of photo_only. A bin stop with an up look (camera tilted) has two photos:
its on_bin items come from the up photo and its around_bin items from the level photo, as in
the judge, so nothing is stored twice.
"""

import json
import os


def bin_ids(layout):
    return {b["name"]: b["id"] for b in (layout or {}).get("bins", [])}


def record_route_summary(store, summary, layout, run_dir):
    """Write one route run. Returns the run id. The store's source must match the run's."""
    if summary.get("source") != store.source:
        raise ValueError(f"run {summary.get('run_id')} is a {summary.get('source')} run, "
                         f"not {store.source}.")
    run_id = summary["run_id"]
    started = summary.get("started_utc")
    store.start_run(run_id=run_id, kind="route", scenario=summary.get("scenario"),
                    layout=layout, started_utc=started, run_dir=run_dir,
                    laps=summary.get("laps"))
    ids = bin_ids(layout)
    checks = load_bin_checks(run_dir)
    for ev in summary.get("events", []):
        if ev.get("event") != "facing_bin" or ev.get("bin") not in ids:
            continue
        photo = os.path.join(run_dir, ev["photo"]) if ev.get("photo") else None
        pose = (ev.get("poses") or {}).get("odom")
        c = checks.get((ev.get("step"), ev["bin"]))
        if c:
            store.add_bin_check(run_id, ids[ev["bin"]], overflow_level=c["overflow_level"],
                                on_top_count=c["on_top_count"],
                                litter_count=c["litter_count"], outcome=c["outcome"],
                                pose=pose, photo=photo, method=c["method"],
                                t_utc=c.get("t_utc") or ev.get("t_utc") or started)
        else:
            store.add_bin_check(run_id, ids[ev["bin"]], pose=pose, photo=photo,
                                method="photo_only", t_utc=ev.get("t_utc") or started)
    record_detections(store, run_id, run_dir)
    measured = summary.get("measured") or {}
    store.end_run(run_id, status=summary.get("status"), ended_utc=summary.get("ended_utc"),
                  reason=summary.get("reason"), distance_m=summary.get("distance_m"),
                  end_error_m=(summary.get("end_errors") or {}).get("odom", {})
                  .get("position_m"),
                  measured_error_cm=measured.get("position_error_cm"))
    return run_id


def load_bin_checks(run_dir):
    """{(facing step, bin name): check} from <run_dir>/bin_checks.json, or {}."""
    path = os.path.join(run_dir, "bin_checks.json")
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        data = json.load(f)
    return {(c.get("facing_step"), c.get("bin")): c for c in data.get("checks", [])}


def record_detections(store, run_id, run_dir):
    """Add the items in <run_dir>/detections.json (if any). Returns how many were added.
    Detections below min_conf were already left out by the follower (kept in the file as
    low_conf, not stored)."""
    path = os.path.join(run_dir, "detections.json")
    if not os.path.exists(path):
        return 0
    with open(path) as f:
        data = json.load(f)
    photos = data.get("photos", [])
    with_up = {(p.get("step"), p.get("bin")) for p in photos if p.get("look") == "up"}
    n = 0
    for photo in photos:
        look = photo.get("look", "level")
        has_up = (photo.get("step"), photo.get("bin")) in with_up
        for d in photo.get("items", []):
            if has_up and (look, d.get("place")) in (("level", "on_bin"), ("up", "around_bin")):
                continue                # the other look of this bin stop has it
            store.add_detection(
                run_id, d["label"], d.get("category") or "unknown", place=d["place"],
                confidence=d.get("conf"), pose=photo.get("pose"), obj_xy=d.get("obj_xy"),
                bin_id=photo.get("bin_id"), bbox=d.get("bbox"),
                photo=os.path.join(run_dir, photo["photo"]) if photo.get("photo") else None,
                model=photo.get("model") or data.get("model"), t_utc=photo.get("t_utc"))
            n += 1
    return n
