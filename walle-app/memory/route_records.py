"""Turn a route follower run log (summary.json + layout.json) into memory store records.

Used by the route follower at the end of each run and by import_runs.py to rebuild a store
from the committed run logs. A Phase 1 run records the run itself and one bin check per bin
stop: the photo and the pose, not yet assessed (method photo_only; Phase 4 assesses bins).
"""

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
    for ev in summary.get("events", []):
        if ev.get("event") != "facing_bin" or ev.get("bin") not in ids:
            continue
        photo = os.path.join(run_dir, ev["photo"]) if ev.get("photo") else None
        store.add_bin_check(run_id, ids[ev["bin"]], pose=(ev.get("poses") or {}).get("odom"),
                            photo=photo, method="photo_only",
                            t_utc=ev.get("t_utc") or started)
    measured = summary.get("measured") or {}
    store.end_run(run_id, status=summary.get("status"), ended_utc=summary.get("ended_utc"),
                  reason=summary.get("reason"), distance_m=summary.get("distance_m"),
                  end_error_m=(summary.get("end_errors") or {}).get("odom", {})
                  .get("position_m"),
                  measured_error_cm=measured.get("position_error_cm"))
    return run_id
