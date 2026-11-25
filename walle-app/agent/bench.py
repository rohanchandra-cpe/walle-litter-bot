"""Model test for the WALL-E agent (Phase 6 plan, decision 1): which LLM chooses the tools
and answers correctly, and how fast. No robot and no sim: the motion tools get a stub
runner that returns the summaries of real robot runs, and the history tools read the
fixture store (agent/fixture_store.py), so every right answer is known.

    python3 /workspace/walle-app/agent/bench.py --models qwen3.5:4b --repeats 5
    python3 /workspace/walle-app/agent/bench.py --models qwen3.5:4b BIGGER --reasoning both

Scores per prompt and repeat: the right tool first with the right argument, no motion for a
history question; and the fact check: the expected facts are in the answer and every
number in it was in a tool's output (or the prompt). Results go to
walle-app/results/agent/bench/<time>/. Needs Ollama on the host (don't run while the
robot is being driven: the detector shares it).
"""

import argparse
import json
import os
import re
import statistics
import sys
import tempfile
import time
from datetime import datetime

AGENT_DIR = os.path.dirname(os.path.abspath(__file__))
APP_DIR = os.path.dirname(AGENT_DIR)
sys.path.insert(0, AGENT_DIR)
sys.path.insert(0, APP_DIR)

import fixture_store  # noqa: E402
import runner as runner_mod  # noqa: E402
import walle_agent  # noqa: E402
import walle_tools  # noqa: E402

ROUTE_RUNS = os.path.join(APP_DIR, "results", "route")
LAP_RUN = "20261123_042112"          # robot lap: a can on side 1, driven around; bins clean
BIN_RUN = "20261122_145845"          # robot lap where bin 1 overflowed (a paper bag on top)
MOTION = {"run_route", "check_bin", "look_ahead"}

# prompt, expected tool, argument check, facts (regexes on the lower-cased answer, "_" -> " ")
PROMPTS = [
    ("Run your route and check the path for any obstructions.", "run_route", None,
     [r"\bcan\b", r"side\s*1"]),
    ("Check Bin 1 to see if it is overflowing. If it is, estimate how much.", "check_bin",
     ("bin", 1), [r"overflow", r"paper bag", r"level\s*(of\s*)?1|1\s*item|one item"]),
    ("Historically, what objects have you seen the most?", "most_seen_objects", None,
     [r"milk jug", r"\b9\b"]),
    ("Are people generally recycling the correct objects?", "recycling_report", None,
     [r"\b75\s*(%|percent)"]),
    ("Which bins overflow most often?", "bin_overflow_history", None, [r"bin\s*3"]),
    ("Where are the obstacles on the route right now?", "report_obstacles", None,
     [r"coffee can", r"side\s*3"]),
]
EXTRA = [("Is anything in front of you right now?", "look_ahead", None, [r"\bcan\b"])]


class StubRunner:
    """Real run summaries instead of driving."""

    def run_route(self, check_bins=True):
        return runner_mod.digest_run(os.path.join(ROUTE_RUNS, LAP_RUN), "route")

    def check_bin(self, n):
        res = runner_mod.digest_run(os.path.join(ROUTE_RUNS, BIN_RUN), "trip")
        res["bins_checked"] = [b for b in res["bins_checked"] if b["bin"] == n]
        res["kind"] = "trip"
        return res

    def look_ahead(self):
        return {"kind": "look", "status": "done", "sides_checked_for_obstacles": [1],
                "obstacles_found": [{"label": "can", "category": "recycling",
                                     "where": "side 1 (corner 1 to corner 2), 0.45 m from "
                                              "corner 1", "ahead_m": 0.45, "left_m": 0.02,
                                     "found_by": "look", "what_the_robot_did": "only seen"}]}

    def stop(self):
        return "stopped"


NUM = re.compile(r"(?<![\w.])-?\d+(?:\.\d+)?")
LIST_MARK = re.compile(r"^\s*\d+[.)]\s", re.M)


def numbers(text):
    out = set()
    for m in NUM.findall(LIST_MARK.sub(" ", text)):
        try:
            out.add(round(float(m), 3))
        except ValueError:
            pass
    return out


def norm(text):
    return str(text).lower().replace("_", " ")


def score(prompt, tool, arg, facts, calls, answer):
    first = calls[0] if calls else None
    tool_ok = first is not None and first["tool"] == tool
    if tool_ok and arg:
        key, want = arg
        tool_ok = walle_tools._bin_number(first["args"].get(key, "")) == want
    if tool not in MOTION and any(c["tool"] in MOTION for c in calls):
        tool_ok = False
    text = norm(answer)
    missing = [f for f in facts if not re.search(f, text)]
    seen = set()
    for c in calls:
        seen |= numbers(c["output"])
    seen |= numbers(prompt)
    small = sorted(n for n in seen if 0 < n < 1000)
    seen |= {round(x + y, 3) for i, x in enumerate(small) for y in small[i:]}   # "6 of 8"
    invented = sorted(n for n in numbers(answer) if n not in seen)
    return {"tool_ok": tool_ok, "facts_ok": not missing and not invented,
            "missing": missing, "invented": invented}


def run_model(model, reasoning, prompts, repeats, store_path, cfg):
    cfg = json.loads(json.dumps(cfg))
    cfg["llm"]["reasoning"] = reasoning
    cfg["approval"]["required"] = False
    walle_tools.configure("sim", store_path=store_path, cfg=cfg, runner=StubRunner(),
                          now=lambda: datetime.fromisoformat(fixture_store.NOW_UTC))
    agent = walle_agent.build_agent(cfg, model=model, verbose=False)
    rows = []
    for rep in range(repeats):
        for prompt, tool, arg, facts in prompts:
            t0 = time.time()
            try:
                answer = walle_agent.respond(agent, prompt)
            except Exception as e:                      # noqa: BLE001
                answer = f"An error occurred: {e}"
            dt = time.time() - t0
            calls = list(walle_tools.CALLS)
            s = score(prompt, tool, arg, facts, calls, answer)
            rows.append(dict(s, model=model, reasoning=reasoning, repeat=rep, prompt=prompt,
                             seconds=round(dt, 1), calls=[{"tool": c["tool"], "args": c["args"]}
                                                          for c in calls], answer=answer))
            mark = ("ok " if s["tool_ok"] else "TOOL") + (" ok " if s["facts_ok"] else " FACT")
            print(f"  [{mark}] {dt:5.1f} s  {prompt[:50]:50s} -> "
                  f"{[c['tool'] for c in calls]} {s['missing'] or ''}{s['invented'] or ''}",
                  flush=True)
    return rows


def summarize(rows, n_main):
    main = [r for r in rows if r["prompt"] in {p[0] for p in PROMPTS}]
    secs = [r["seconds"] for r in main]
    return {"tool_ok": sum(r["tool_ok"] for r in main), "facts_ok": sum(r["facts_ok"] for r in main),
            "of": len(main), "median_s": round(statistics.median(secs), 1) if secs else None,
            "max_s": max(secs) if secs else None,
            "extra_ok": sum(r["tool_ok"] and r["facts_ok"] for r in rows if r not in main)}


def main(argv=None):
    p = argparse.ArgumentParser(description="Model test for the WALL-E agent")
    p.add_argument("--models", nargs="+", default=["qwen3.5:4b"])
    p.add_argument("--repeats", type=int, default=5)
    p.add_argument("--reasoning", choices=("default", "on", "off", "both"), default="default")
    p.add_argument("--only", type=int, nargs="+", help="prompt numbers (1-7) to run")
    a = p.parse_args(argv)
    cfg = walle_tools.load_config()
    prompts = PROMPTS + EXTRA
    if a.only:
        prompts = [prompts[i - 1] for i in a.only]
    modes = {"default": [None], "on": [True], "off": [False], "both": [True, False]}[a.reasoning]
    out_dir = os.path.join(APP_DIR, "results", "agent", "bench", time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)
    tmp = tempfile.mkdtemp()
    store = fixture_store.build(os.path.join(tmp, "fixture.sqlite"))
    table = []
    for model in a.models:
        for mode in modes:
            label = model + ("" if mode is None else f" (reasoning {'on' if mode else 'off'})")
            print(f"\n=== {label}: {len(prompts)} prompts x {a.repeats}")
            rows = run_model(model, mode, prompts, a.repeats, store, cfg)
            s = summarize(rows, len(PROMPTS))
            table.append(dict(s, model=label))
            name = label.replace(":", "_").replace(" ", "_").replace("(", "").replace(")", "")
            with open(os.path.join(out_dir, f"{name}.json"), "w") as f:
                json.dump({"summary": s, "rows": rows}, f, indent=1)
    print("\nModel                                  tool+args   facts   median s   max s   look")
    for s in table:
        print(f"{s['model']:38s} {s['tool_ok']:3d}/{s['of']:<3d}    {s['facts_ok']:3d}/{s['of']:<3d}"
              f"   {s['median_s']!s:>7}   {s['max_s']!s:>5}   {s['extra_ok']}")
    with open(os.path.join(out_dir, "summary.json"), "w") as f:
        json.dump(table, f, indent=1)
    print(f"\nResults: {out_dir}")


if __name__ == "__main__":
    main()
