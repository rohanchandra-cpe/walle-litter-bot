"""Generate the frozen trial schedule from the seed in config/pilot.yaml.

    python3 pilot/make_schedule.py            # writes data/schedule/*.csv (refuses to overwrite)
    python3 pilot/make_schedule.py --force    # regenerate (only before any main-study trial)

motion_schedule.csv, per session, in run order:
    2 warm-up runs (one A, one B; excluded in advance), then RQ1 blocks and RQ2 turns
    interleaved at random. Each RQ1 block is an A run and a B run back to back, in a
    random order (balanced: 8 blocks AB, 7 BA). RQ2 has 8 CCW and 8 CW turns.
range_schedule.csv: every object x distance x angle once, shuffled and split across
    sessions; which sensor reads first is balanced (32 each) and assigned at random.
The trial scripts run the schedule in order and log the realized order and time.
"""

import argparse
import csv
import os
import random

from common import MOTION_SCHEDULE, RANGE_SCHEDULE, SCHEDULE_DIR, load_config

MOTION_FIELDS = ["trial_id", "session_id", "order", "task", "strategy", "direction",
                 "block_id", "order_in_block", "warmup"]
RANGE_FIELDS = ["placement_id", "session_id", "order", "object", "color", "distance_m",
                "angle_deg", "first_sensor"]


def balanced(rng, items, n):
    """n labels split as evenly as possible between `items`, in a random order."""
    labels = [items[i % len(items)] for i in range(n)]
    rng.shuffle(labels)
    return labels


def motion_schedule(cfg, rng):
    study = cfg["study"]
    sessions = study["sessions"]
    n_blocks = study["rq1_blocks_per_session"] * sessions
    turns_per_session = study["rq2_turns_per_session"]
    block_orders = balanced(rng, ["AB", "BA"], n_blocks)
    directions = balanced(rng, ["ccw", "cw"], sum(turns_per_session))

    rows, block_index, turn_index, block_number = [], 0, 0, 0
    for session in range(1, sessions + 1):
        units = []
        for _ in range(study["rq1_blocks_per_session"]):
            units.append(("block", None, block_orders[block_index]))
            block_index += 1
        for _ in range(turns_per_session[session - 1]):
            units.append(("turn", None, directions[turn_index]))
            turn_index += 1
        rng.shuffle(units)

        session_rows = [
            {"task": "straight", "strategy": "A", "warmup": 1},
            {"task": "straight", "strategy": "B", "warmup": 1},
        ]
        for kind, block, detail in units:
            if kind == "block":
                block_number += 1   # numbered in run order
                for position, strategy in enumerate(detail, start=1):
                    session_rows.append({"task": "straight", "strategy": strategy,
                                         "block_id": f"B{block_number:02d}",
                                         "order_in_block": position, "warmup": 0})
            else:
                session_rows.append({"task": "turn", "direction": detail, "warmup": 0})
        for order, row in enumerate(session_rows, start=1):
            row.update(session_id=session, order=order,
                       trial_id=f"S{session}-M{order:02d}" + ("W" if row["warmup"] else ""))
            rows.append(row)
    return rows


def range_schedule(cfg, rng):
    ranging = cfg["ranging"]
    placements = [(obj, color, d, a)
                  for obj, color in ranging["objects"].items()
                  for d in ranging["distances_m"]
                  for a in ranging["angles_deg"]]
    if len(placements) != cfg["study"]["rq3_placements"]:
        raise SystemExit(f"The config gives {len(placements)} placements, "
                         f"not {cfg['study']['rq3_placements']}.")
    rng.shuffle(placements)
    first = balanced(rng, ["ultrasonic", "camera"], len(placements))
    sessions = cfg["study"]["sessions"]
    sizes = [len(placements) // sessions + (1 if i < len(placements) % sessions else 0)
             for i in range(sessions)]
    rows, start = [], 0
    for session, size in enumerate(sizes, start=1):
        for order, i in enumerate(range(start, start + size), start=1):
            obj, color, distance, angle = placements[i]
            rows.append({"placement_id": f"S{session}-R{order:02d}", "session_id": session,
                         "order": order, "object": obj, "color": color,
                         "distance_m": f"{distance:.2f}", "angle_deg": angle,
                         "first_sensor": first[i]})
        start += size
    return rows


def write(path, fields, rows):
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: row.get(k, "") for k in fields})


def main():
    parser = argparse.ArgumentParser(description="Generate the pilot schedule")
    parser.add_argument("--force", action="store_true", help="overwrite existing schedules")
    args = parser.parse_args()
    cfg = load_config()
    if not args.force and (os.path.exists(MOTION_SCHEDULE) or os.path.exists(RANGE_SCHEDULE)):
        raise SystemExit("The schedule already exists and is frozen. Use --force only "
                         "before any main-study trial has been run.")
    os.makedirs(SCHEDULE_DIR, exist_ok=True)
    rng = random.Random(cfg["study"]["seed"])
    motion = motion_schedule(cfg, rng)
    ranging = range_schedule(cfg, rng)
    write(MOTION_SCHEDULE, MOTION_FIELDS, motion)
    write(RANGE_SCHEDULE, RANGE_FIELDS, ranging)
    straight = sum(1 for r in motion if r["task"] == "straight" and not r["warmup"])
    turns = sum(1 for r in motion if r["task"] == "turn")
    print(f"Seed {cfg['study']['seed']}: {straight} straight runs, {turns} turns, "
          f"{len(ranging)} placements over {cfg['study']['sessions']} sessions.")
    print(f"Wrote {MOTION_SCHEDULE}\n      {RANGE_SCHEDULE}")


if __name__ == "__main__":
    main()
