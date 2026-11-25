"""A fixed memory store with a known history, for checking the agent's history answers
(Phase 6 plan, decision 7). The live sim store holds whatever the sim checks happened to
place; this one is built the same way every time, so the right answers are known (FACTS).

    python3 walle-app/agent/fixture_store.py /tmp/fixture.sqlite      # build it, print FACTS

History, five route runs on 2026-11-20 (the park's default layout: bins 1 and 3 trash,
2 and 4 recycling):
- seen most: milk_jug 9, can 6, paper_towel 4, foil 3, coffee_can 2;
- bin 3 overflows most (4 of 5 checks), then bin 4 (2 of 5), bin 2 (1 of 5), bin 1 never;
- items on top of bins: 6 in the right bin, 2 wrong (foil in recycling bin 2, a can in
  trash bin 3): 75 % correct;
- obstacles: a milk_jug on side 1 (runs 1-2, cleared in run 3), a coffee_can on side 3 at
  0.40 m from corner 3 (runs 4-5), still there.
"""

import json
import os
import sys

AGENT_DIR = os.path.dirname(os.path.abspath(__file__))
APP_DIR = os.path.dirname(AGENT_DIR)
if APP_DIR not in sys.path:
    sys.path.insert(0, APP_DIR)

import park  # noqa: E402
from memory.store import MemoryStore  # noqa: E402

RUNS = [f"20261120_0{h}0000" for h in range(5)]     # 00:00 ... 04:00 UTC
NOW_UTC = "2026-11-20T06:00:00+00:00"                # "now" for the checks (hours_ago)

FACTS = {
    "most_seen": ["milk_jug", "can", "paper_towel", "foil", "coffee_can"],
    "most_seen_times": {"milk_jug": 9, "can": 6, "paper_towel": 4, "foil": 3, "coffee_can": 2},
    "overflow_ranking": [3, 4, 2, 1],
    "overflow_times": {3: 4, 4: 2, 2: 1, 1: 0},
    "recycling": {"correct": 6, "wrong": 2, "correct_percent": 75,
                  "wrong_items": ["foil", "can"]},
    "obstacles_now": [{"label": "coffee_can", "side": 3, "along_m": 0.40, "hours_ago": 1.8}],
    "cleared": ["milk_jug"],
}


def t(run, minute):
    return f"{run[:4]}-{run[4:6]}-{run[6:8]}T{run[9:11]}:{minute:02d}:00+00:00"


def build(path):
    if os.path.exists(path):
        os.remove(path)
    layout = park.load_park("default", obstacle="none")
    with MemoryStore("sim", path=path, embedder=None) as mem:
        jug = None
        for i, run in enumerate(RUNS):
            mem.start_run(run_id=run, kind="route", scenario="fixture", layout=layout,
                          started_utc=t(run, 0), laps=1)
            # bins: (on top [(label, category)], around [(label, category)])
            looks = {
                1: ([], [("can", "recycling")]),
                2: ([("foil", "trash")] if i == 2 else [], [("milk_jug", "recycling")]),
                3: ([("paper_towel", "trash")] + ([("can", "recycling")] if i == 3 else [])
                    if i < 4 else [],
                    [("foil", "trash")] if i < 2 else []),
                4: ([("milk_jug", "recycling")] if i < 2 else [], []),
            }
            for b, (top, around) in looks.items():
                when = t(run, 5 * b)
                level = 1 if top else 0
                mem.add_bin_check(run, b, overflow_level=level, on_top_count=len(top),
                                  litter_count=len(around), method="tilt", t_utc=when)
                for label, cat in top:
                    mem.add_detection(run, label, cat, place="on_bin", bin_id=b,
                                      confidence=0.8, t_utc=when)
                for label, cat in around:
                    mem.add_detection(run, label, cat, place="around_bin", bin_id=b,
                                      confidence=0.7, t_utc=when)
            # obstacles
            if i < 2:
                d = mem.add_detection(run, "milk_jug", "recycling", place="obstacle",
                                      obj_xy=(0.50, 0.0), confidence=0.8, t_utc=t(run, 2))
                jug = mem.report_obstacle(run, "milk_jug", "recycling", (0.50, 0.0), side=1,
                                          detection_id=d, action="passed", t_utc=t(run, 2))
            elif i == 2:
                mem.mark_absent(run, jug, t_utc=t(run, 2))
            if i >= 3:
                d = mem.add_detection(run, "coffee_can", "trash", place="obstacle",
                                      obj_xy=(0.60, 1.0), confidence=0.8, t_utc=t(run, 12))
                mem.report_obstacle(run, "coffee_can", "trash", (0.60, 1.0), side=3,
                                    detection_id=d, action="passed", t_utc=t(run, 12))
            mem.end_run(run, status="done", ended_utc=t(run, 30), distance_m=4.0)
    return path


if __name__ == "__main__":
    out = build(sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        APP_DIR, "results", "agent", "fixture.sqlite"))
    print(f"Fixture store: {out}")
    print(json.dumps(FACTS, indent=1))
