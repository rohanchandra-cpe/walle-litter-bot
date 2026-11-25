"""What the WALL-E agent's history answers should say, straight from the memory store
(Phase 6 plan, decision 8): compare the agent's answers with this printout.

    python3 walle-app/agent/facts.py                    # robot store
    python3 walle-app/agent/facts.py --store sim
    python3 walle-app/agent/facts.py --store-path /tmp/fixture.sqlite --store sim

It runs the same store queries the agent's tools call (memory/store.py), with no LLM.
"""

import argparse
import os
import sys

AGENT_DIR = os.path.dirname(os.path.abspath(__file__))
APP_DIR = os.path.dirname(AGENT_DIR)
sys.path.insert(0, APP_DIR)

from memory.store import MemoryStore  # noqa: E402


def pct(x):
    return "n/a" if x is None else f"{round(100 * x)}%"


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--store", choices=("robot", "sim"), default="robot")
    p.add_argument("--store-path")
    a = p.parse_args(argv)
    with MemoryStore(a.store, path=a.store_path, embedder=None) as mem:
        c = mem.counts()
        print(f"Store: {a.store} ({mem.path}), {c['runs']} runs, {c['detections']} detections, "
              f"{c['bin_checks']} bin checks, {c['obstacles']} obstacles\n")

        print("3. Historically, what objects have you seen the most?")
        for r in mem.most_seen(limit=5):
            print(f"   {r['label']:15s} {r['category']:10s} seen {r['times_seen']:3d} times "
                  f"in {r['runs_seen']} runs")

        print("\n4. Are people generally recycling the correct objects?")
        rc = mem.recycling_correctness()
        o = rc["overall"]
        print(f"   overall: {o['correct']} correct, {o['wrong']} wrong, {o['unclassified']} "
              f"unclassified -> {pct(o['correct_rate'])} correct")
        for b in rc["per_bin"]:
            print(f"   bin {b['bin_id']} ({b['bin_type']}): {b['correct']} correct, {b['wrong']} "
                  f"wrong ({pct(b['correct_rate'])})")
        for w in rc["wrong_items"][:5]:
            print(f"   wrong: {w['label']} ({w['category']}) in bin {w['bin_id']} "
                  f"({w['bin_type']}) x{w['times']}")

        print("\n5. Which bins overflow most often?")
        for r in mem.bin_overflow_ranking():
            if r["assessed"]:
                print(f"   bin {r['bin_id']} ({r['bin_type']}): {r['times_overflowing']} of "
                      f"{r['assessed']} checks ({pct(r['overflow_rate'])}), max level "
                      f"{r['max_level']}")

        print("\n6. Where are the obstacles on the route right now?")
        now = mem.current_obstacles()
        if not now:
            print("   none")
        for r in now:
            print(f"   {r['label']} ({r['category']}) on side {r['side']} at "
                  f"({r['position'][0]:.2f}, {r['position'][1]:.2f}) m, last seen "
                  f"{r['last_seen_utc']} (run {r['last_run_id']}), seen {r['times_seen']}x")

        print("\n2. Check Bin 1 (the stored check before the new one)")
        b = mem.last_bin_check(1)
        if b:
            items = mem.bin_check_items(b["run_id"], 1)
            print(f"   {b['t_utc']} run {b['run_id']}: {b['outcome']}, level "
                  f"{b['overflow_level']}, {b['on_top_count']} on top "
                  f"{[i['label'] for i in items['on_top']]}, {b['litter_count']} around")


if __name__ == "__main__":
    main()
