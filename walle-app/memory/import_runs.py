"""Rebuild (or top up) a memory store from route run logs.

The .sqlite files are git-ignored; the committed run logs under walle-app/results/route/
are the record. This reads every <run>/summary.json + layout.json there and adds the runs
the store doesn't have yet.

    python3 walle-app/memory/import_runs.py                      # robot runs -> robot.sqlite
    python3 walle-app/memory/import_runs.py --rebuild            # start the file afresh
    python3 walle-app/memory/import_runs.py --source sim --runs walle-sim/data/runs/route
"""

import argparse
import glob
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from memory.route_records import load_run_review, record_route_summary  # noqa: E402
from memory.store import APP_DIR, MemoryStore, db_path, load_config  # noqa: E402


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--source", default="robot", choices=("robot", "sim"))
    p.add_argument("--runs", default=os.path.join(APP_DIR, "results", "route"),
                   help="folder holding <run_id>/summary.json")
    p.add_argument("--db", help="store file (default: config/memory.yaml)")
    p.add_argument("--rebuild", action="store_true", help="delete the store file first")
    p.add_argument("--no-embed", action="store_true", help="store text without embeddings")
    a = p.parse_args(argv)

    path = a.db or db_path(a.source, load_config())
    if a.rebuild and os.path.exists(path):
        os.remove(path)
        print(f"Removed {path}")
    store = MemoryStore(a.source, path=path, embedder=None if a.no_embed else "config")
    added = skipped = 0
    for summary_path in sorted(glob.glob(os.path.join(a.runs, "*", "summary.json"))):
        run_dir = os.path.dirname(summary_path)
        with open(summary_path) as f:
            summary = json.load(f)
        layout = None
        if os.path.exists(os.path.join(run_dir, "layout.json")):
            with open(os.path.join(run_dir, "layout.json")) as f:
                layout = json.load(f)
        rid = summary.get("run_id")
        if summary.get("source") != a.source:
            print(f"  skip {rid}: a {summary.get('source')} run")
            skipped += 1
            continue
        review = load_run_review(run_dir)
        if review.get("leave_out"):
            print(f"  skip {rid}: left out after review ({review['leave_out']})")
            skipped += 1
            continue
        if store.has_run(rid):
            skipped += 1
            continue
        record_route_summary(store, summary, layout, run_dir)
        print(f"  added {rid} ({summary.get('status')}, {summary.get('scenario')})")
        added += 1
    print(f"{path}: {added} runs added, {skipped} skipped; {store.counts()}")
    store.close()


if __name__ == "__main__":
    sys.exit(main())
