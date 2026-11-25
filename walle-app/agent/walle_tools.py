"""ROSA tools for the WALL-E agent (ROADMAP Phase 6).

History tools read the memory store (walle-app/memory/store.py) and nothing else, so every
history answer comes from a query. Each result names the store it came from and carries the
limits of that data. Motion tools hand the work to a runner (agent/runner.py: the route
follower as a subprocess, with all its guards); each asks a person for approval once, and
the approval covers the whole run.

configure() must be called first (walle_agent.py does it). Every tool call is appended to
CALLS (name, arguments, output) so the model test and the sim check can see what was done.
"""

import json
import math
import os
import sys
from datetime import datetime, timezone

import yaml
from langchain_core.tools import tool

AGENT_DIR = os.path.dirname(os.path.abspath(__file__))
APP_DIR = os.path.dirname(AGENT_DIR)
REPO_DIR = os.path.dirname(APP_DIR)
CONFIG_FILE = os.path.join(APP_DIR, "config", "agent.yaml")
if APP_DIR not in sys.path:
    sys.path.insert(0, APP_DIR)

import park  # noqa: E402
from memory.store import MemoryStore, repo_relative  # noqa: E402

BINS = (1, 2, 3, 4)
PLACES = ("any", "on_bin", "around_bin", "obstacle")
LIMITS = {
    "labels": "Labels come from a small camera model and can be wrong (a plastic bag was "
              "once read as a paper bag).",
    "sightings": "times_seen counts sightings in photos, not distinct objects: an item left "
                 "in place is counted again on every run that sees it.",
    "bins": "The bins are opaque: the robot sees only what is above the rim and on the "
            "ground around a bin. There is no fill level, only an overflow level (0-3, by "
            "the count of items on top) and the item counts.",
    "obstacles": "An obstacle stays listed until a run drives its side and does not see it.",
}


def load_config(path=CONFIG_FILE):
    with open(path) as f:
        return yaml.safe_load(f)


class Context:
    """What the tools need: the store, the runner, the approval prompt, the call log."""
    source = None             # "robot" or "sim"
    store_path = None         # None = the store's own file
    cfg = None
    runner = None             # agent/runner.py Runner (or a stub in the model test)
    approve = None            # callable(description) -> bool
    now = None                # callable() -> datetime (tests freeze it)
    corners = None


CTX = Context()
CALLS = []


def configure(source, store_path=None, cfg=None, runner=None, approve=None, now=None):
    CTX.source = source
    CTX.store_path = store_path
    CTX.cfg = cfg or load_config()
    CTX.runner = runner
    CTX.approve = approve or ask_approval
    CTX.now = now or (lambda: datetime.now(timezone.utc))
    CTX.corners = {int(k): v for k, v in park.load_park("default")["route"]["corners"].items()}
    CALLS.clear()


def ask_approval(description):
    answer = input(f"\n[APPROVAL NEEDED] {description} Ctrl+C stops the robot at any time. "
                   "Allow? [y/N] ")
    return answer.strip().lower() in ("y", "yes")


def _store():
    return MemoryStore(CTX.source, path=CTX.store_path)


def _out(name, args, result):
    text = json.dumps(result, separators=(", ", ": "), default=str)
    CALLS.append({"tool": name, "args": args, "output": text})
    return text


def _hours_ago(t_utc):
    if not t_utc:
        return None
    try:
        t = datetime.fromisoformat(str(t_utc).replace("Z", "+00:00"))
    except ValueError:
        return None
    if t.tzinfo is None:
        t = t.replace(tzinfo=timezone.utc)
    return round((CTX.now() - t).total_seconds() / 3600.0, 1)


def _day(t_utc):
    return str(t_utc)[:16].replace("T", " ") + " UTC" if t_utc else None


def _percent(x):
    return None if x is None else int(round(100 * x))


def store_info(mem):
    last = mem.runs(limit=1)
    return {"store": CTX.source, "file": repo_relative(mem.path), "runs": mem.counts()["runs"],
            "last_run": _day(last[0]["started_utc"]) if last else None}


def _bin_number(text):
    """'1', 'bin 1', 'Bin_1', 'one' -> 1; None if it isn't a bin."""
    words = {"one": 1, "two": 2, "three": 3, "four": 4}
    t = str(text).strip().lower().replace("bin", "").replace("_", " ").strip()
    if t in words:
        return words[t]
    try:
        n = int(float(t))
    except ValueError:
        return None
    return n if n in BINS else None


def where_on_route(xy, side=None):
    """'side 2 (corner 2 to corner 3), 0.45 m from corner 2' for a park point."""
    if xy is None or xy[0] is None:
        return f"side {side}" if side else "position unknown"
    best = None
    for s in BINS if side is None else (int(side),):
        a, b = CTX.corners[s], CTX.corners[s % 4 + 1]
        ux, uy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(ux, uy)
        along = ((xy[0] - a[0]) * ux + (xy[1] - a[1]) * uy) / length
        cross = ((xy[1] - a[1]) * ux - (xy[0] - a[0]) * uy) / length
        if best is None or abs(cross) < abs(best[2]):
            best = (s, along, cross)
    s, along, _ = best
    return (f"side {s} (corner {s} to corner {s % 4 + 1}), {max(along, 0.0):.2f} m from "
            f"corner {s}")


# ===== history tools (read the memory store only) =====

@tool
def most_seen_objects(place: str = "any") -> str:
    """The objects the robot has seen most often, from its memory store: label, category
    (trash or recycling), times seen, runs seen in, last seen. place: 'any' (default),
    'on_bin' (on top of bins), 'around_bin' (litter on the ground by bins) or 'obstacle'
    (on the route)."""
    p = str(place).strip().lower()
    p = p if p in PLACES else "any"
    with _store() as mem:
        rows = mem.most_seen(limit=int(CTX.cfg["history"]["most_seen_limit"]),
                             place=None if p == "any" else p)
        res = {"from": store_info(mem), "place": p,
               "objects": [{"label": r["label"], "category": r["category"],
                            "times_seen": r["times_seen"], "runs_seen": r["runs_seen"],
                            "last_seen": _day(r["last_seen_utc"])} for r in rows],
               "limits": [LIMITS["sightings"], LIMITS["labels"]]}
    return _out("most_seen_objects", {"place": place}, res)


@tool
def bin_overflow_history() -> str:
    """How often each bin has overflowed, from the memory store, most often first: checks
    judged, times overflowing, overflow percent, average and highest overflow level, times
    with litter around it, times clean."""
    with _store() as mem:
        rows = mem.bin_overflow_ranking()
        res = {"from": store_info(mem),
               "bins": [{"bin": r["bin_id"], "type": r["bin_type"],
                         "checks_judged": r["assessed"],
                         "times_overflowing": r["times_overflowing"],
                         "overflow_percent": _percent(r["overflow_rate"]),
                         "mean_level_when_overflowing": r["mean_level"],
                         "max_level": r["max_level"], "times_litter": r["times_litter"],
                         "times_clean": r["times_clean"],
                         "last_check": _day(r["last_check_utc"])}
                        for r in rows if r["assessed"]],
               "limits": [LIMITS["bins"], LIMITS["labels"]]}
    return _out("bin_overflow_history", {}, res)


@tool
def recycling_report() -> str:
    """Whether items end up in the right bins (trash in trash bins, recycling in recycling
    bins), from the memory store: correct and wrong counts overall and per bin, the wrong
    items, and the litter seen around the bins."""
    with _store() as mem:
        r = mem.recycling_correctness()
        o = r["overall"]
        res = {"from": store_info(mem),
               "overall": {"correct": o["correct"], "wrong": o["wrong"],
                           "unclassified": o["unclassified"],
                           "correct_percent": _percent(o["correct_rate"])},
               "per_bin": [{"bin": b["bin_id"], "type": b["bin_type"], "correct": b["correct"],
                            "wrong": b["wrong"], "correct_percent": _percent(b["correct_rate"])}
                           for b in r["per_bin"]],
               "wrong_items": [{"label": w["label"], "category": w["category"],
                                "in_bin": w["bin_id"], "bin_type": w["bin_type"],
                                "times": w["times"]} for w in r["wrong_items"][:5]],
               "litter_around_bins": [{"bin": a["bin_id"], "category": a["category"],
                                       "items": a["n"]} for a in r["litter_around_bins"]],
               "limits": [r["basis"], LIMITS["labels"]]}
    return _out("recycling_report", {}, res)


@tool
def report_obstacles() -> str:
    """The obstacles on the route right now, from the memory store: where each one is (side
    and distance from the corner), what it is, when it was last seen and how many hours ago,
    and the last run that looked for obstacles. Does not move the robot."""
    with _store() as mem:
        rows = mem.current_obstacles()
        last_look = None
        for run in mem.runs(limit=50):
            d = run.get("run_dir")
            if d and os.path.exists(os.path.join(REPO_DIR, d, "obstacles.json")):
                with open(os.path.join(REPO_DIR, d, "obstacles.json")) as f:
                    sides = json.load(f).get("sides_done", [])
                last_look = {"run_id": run["run_id"], "kind": run["kind"],
                             "time": _day(run["started_utc"]),
                             "hours_ago": _hours_ago(run["started_utc"]),
                             "status": run["status"], "sides_checked": sides}
                break
        res = {"from": store_info(mem),
               "obstacles_now": [{"label": r["label"], "category": r["category"],
                                  "where": where_on_route(r["position"], r["side"]),
                                  "side": r["side"],
                                  "position_m": [round(v, 2) if v is not None else None
                                                 for v in r["position"]],
                                  "first_seen": _day(r["first_seen_utc"]),
                                  "last_seen": _day(r["last_seen_utc"]),
                                  "hours_ago": _hours_ago(r["last_seen_utc"]),
                                  "times_seen": r["times_seen"]} for r in rows],
               "last_obstacle_look": last_look,
               "limits": [LIMITS["obstacles"], LIMITS["labels"]]}
    return _out("report_obstacles", {}, res)


def _count(labels):
    """Labels -> {label: count}, most first (the model copies counts; it miscounts lists)."""
    counts = {}
    for label in labels:
        counts[label] = counts.get(label, 0) + 1
    return dict(sorted(counts.items(), key=lambda kv: (-kv[1], str(kv[0]))))


def _bin_check_record(mem, bin_id):
    c = mem.last_bin_check(bin_id)
    if c is None:
        return None
    items = mem.bin_check_items(c["run_id"], bin_id)
    return {"bin": bin_id, "type": c["bin_type"], "time": _day(c["t_utc"]),
            "hours_ago": _hours_ago(c["t_utc"]), "run_id": c["run_id"],
            "outcome": c["outcome"], "overflow_level": c["overflow_level"],
            "items_on_top": c["on_top_count"], "litter_around": c["litter_count"],
            "method": c["method"],
            "on_top": _count(i["label"] for i in items["on_top"]),
            "around": _count(i["label"] for i in items["around"])}


@tool
def last_bin_check(bin: str) -> str:
    """The most recent stored check of one bin, from the memory store (no driving): when,
    outcome (overflowing, litter or clean), overflow level 0-3, items on top and litter
    around it. bin: the bin number, '1' to '4'."""
    n = _bin_number(bin)
    if n is None:
        return _out("last_bin_check", {"bin": bin}, {"error": "bin must be 1, 2, 3 or 4"})
    with _store() as mem:
        res = {"from": store_info(mem), "check": _bin_check_record(mem, n),
               "limits": [LIMITS["bins"], LIMITS["labels"]]}
    return _out("last_bin_check", {"bin": bin}, res)


@tool
def recent_runs() -> str:
    """The robot's most recent runs, from the memory store: kind, time, status (done,
    blocked, aborted) and why it stopped."""
    with _store() as mem:
        rows = mem.runs(limit=int(CTX.cfg["history"]["recent_runs_limit"]))
        res = {"from": store_info(mem),
               "runs": [{"run_id": r["run_id"], "kind": r["kind"], "time": _day(r["started_utc"]),
                         "hours_ago": _hours_ago(r["started_utc"]), "status": r["status"],
                         "reason": r["reason"]} for r in rows]}
    return _out("recent_runs", {}, res)


@tool
def recall_memory(question: str) -> str:
    """Search the memory store for records related to a question in words, when no other
    tool fits. Returns the closest stored sentences with their times."""
    with _store() as mem:
        rows = mem.recall(str(question), k=int(CTX.cfg["history"]["recall_k"]))
        res = {"from": store_info(mem),
               "records": [{"time": _day(r["t_utc"]), "text": r["text"]} for r in rows],
               "limits": [LIMITS["labels"]]}
    return _out("recall_memory", {"question": question}, res)


# ===== motion tools (the route follower, through the runner) =====

def _minutes(kind):
    m = CTX.cfg["route"]["minutes"][kind]
    return m["sim" if CTX.source == "sim" else "robot"]


def _motion(name, args, description, kind, call):
    if CTX.runner is None:
        return _out(name, args, {"error": "Motion is not available in this session. The "
                                          "robot did not move."})
    if not CTX.cfg["approval"]["required"] or CTX.approve(
            f"{description}. Takes {_minutes(kind)} min."):
        return _out(name, args, call())
    return _out(name, args, {"status": "declined",
                             "error": "The person declined. The robot did not move."})


@tool
def run_route(check_bins: str = "yes") -> str:
    """Drive one full lap of the route (corner 1 -> 2 -> 3 -> 4 -> 1). On every side the
    robot looks for obstacles and drives around them; it reports each obstacle it found and
    whether it got past. check_bins: 'yes' (default) also checks all four bins for
    overflow, 'no' only drives and looks for obstacles. Takes several minutes; asks a person
    first."""
    bins = str(check_bins).strip().lower() not in ("no", "false", "0", "none")
    desc = ("Drive one lap of the route, looking for obstacles on every side"
            + (" and checking bins 1-4" if bins else ", no bin checks"))
    return _motion("run_route", {"check_bins": check_bins}, desc, "lap",
                   lambda: CTX.runner.run_route(check_bins=bins))


@tool
def check_bin(bin: str) -> str:
    """Drive to one bin, check it for overflow and litter, and drive back to the start
    (looking for obstacles on the way). Reports the outcome, the overflow level 0-3, the
    items on top and the litter around it, plus the previous stored check. bin: the bin
    number, '1' to '4'. Asks a person first."""
    n = _bin_number(bin)
    if n is None:
        return _out("check_bin", {"bin": bin}, {"error": "bin must be 1, 2, 3 or 4. The robot "
                                                         "did not move."})
    with _store() as mem:
        before = _bin_check_record(mem, n)

    def call():
        res = CTX.runner.check_bin(n)
        res["previous_check"] = before
        res["limits"] = [LIMITS["bins"], LIMITS["labels"]]
        return res
    desc = (f"Check bin {n}: " + ("turn to face it on the start corner, check it, turn back"
                                  if n == 1 else
                                  f"drive to corner {n}, check bin {n}, drive back to the start"))
    return _motion("check_bin", {"bin": bin}, desc, "trip", call)


@tool
def look_ahead() -> str:
    """Take one photo of the floor ahead (the camera tilts down, then level again) and
    report any obstacles in it: what, how far ahead, left or right. The robot does not
    drive. Asks a person first, because the camera moves."""
    return _motion("look_ahead", {}, "Tilt the camera down, take one photo of the floor ahead "
                   "and tilt it back (no driving)", "look", lambda: CTX.runner.look_ahead())


@tool
def stop_robot() -> str:
    """Stop the robot now."""
    if CTX.runner is None:
        return _out("stop_robot", {}, {"status": "no runner: nothing is moving"})
    return _out("stop_robot", {}, {"status": CTX.runner.stop()})


HISTORY_TOOLS = [most_seen_objects, bin_overflow_history, recycling_report, report_obstacles,
                 last_bin_check, recent_runs, recall_memory]
MOTION_TOOLS = [run_route, check_bin, look_ahead, stop_robot]
TOOLS = MOTION_TOOLS + HISTORY_TOOLS
