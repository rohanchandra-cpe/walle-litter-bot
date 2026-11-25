"""Phase 6 sim check: the WALL-E agent's acceptance prompts (docs/plans/phase-6-plan.md,
decision 7). Headless, one command, PASS or FAIL.

- history: prompts 3-6 against the fixture store (walle-app/agent/fixture_store.py, a
  known history), each --repeats times (default 3). Each must call the right memory tool,
  never a motion tool, and pass the fact check: the expected facts are in the answer and
  every number in it came from a tool (walle-app/agent/bench.py score()).
- route: prompt 1 in its own world (scenario mixed: two overflowing bins, litter, one
  obstacle). The agent must call run_route, and the run must end done; the obstacle must
  be found on its true side within --tol-cm and driven around; every bin must be judged
  right against the world; the answer must name the obstacle and its side.
- bin1: prompt 2 (scenario bincheck: bin 1 overflowing, level 2). check_bin(1) must run
  (the follower's --trip-bin 1), judge bin 1 right, and the answer must give its level.
- look: "Is anything in front of you right now?" (bincheck with an obstacle on side 1).
  look_ahead must run (--look-only) and report the obstacle on side 1.

Run inside the sim container with no other launch running (`make sim-agent-check`):

    python3 /workspace/walle-sim/harness/agent_trials.py                  # everything
    python3 /workspace/walle-sim/harness/agent_trials.py --only history
    python3 /workspace/walle-sim/harness/agent_trials.py --only route bin1 look

Needs Ollama on the host (the agent's LLM). Each world case takes ~3-6 min.
"""

import argparse
import json
import math
import os
import re
import subprocess
import sys
import tempfile
import time
from datetime import datetime

SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_DIR = os.path.dirname(SIM_DIR)
APP_DIR = os.path.join(REPO_DIR, "walle-app")
sys.path.insert(0, os.path.join(SIM_DIR, "harness"))
sys.path.insert(0, os.path.join(APP_DIR, "agent"))
sys.path.insert(0, APP_DIR)

import obstacle_trials as ot  # noqa: E402  (world launch and stop helpers)

LOOK_PROMPT = "Is anything in front of you right now?"
WORLDS = {
    "route": {"scenario": "mixed", "prompt": 0, "tool": "run_route"},
    "bin1": {"scenario": "bincheck", "prompt": 1, "tool": "check_bin", "bin": 1},
    "look": {"scenario": "bincheck", "obstacle": "1", "prompt": None, "tool": "look_ahead"},
}


def norm(text):
    return str(text).lower().replace("_", " ")


def make_agent(store, store_path=None, runner=None, now=None):
    import walle_agent
    import walle_tools
    cfg = walle_tools.load_config()
    walle_tools.configure(store, store_path=store_path, cfg=cfg, runner=runner,
                          approve=lambda d: print(f"[auto-approved in Gazebo] {d}") or True,
                          now=now)
    return walle_agent.build_agent(cfg, verbose=False)


def history_cases(a):
    import bench
    import fixture_store
    import walle_agent
    import walle_tools
    path = fixture_store.build(os.path.join(tempfile.mkdtemp(), "fixture.sqlite"))
    agent = make_agent("sim", store_path=path,
                       now=lambda: datetime.fromisoformat(fixture_store.NOW_UTC))
    results = []
    for prompt, tool, arg, facts in bench.PROMPTS[2:6]:
        for rep in range(a.repeats):
            t0 = time.time()
            answer = walle_agent.respond(agent, prompt)
            calls = list(walle_tools.CALLS)
            s = bench.score(prompt, tool, arg, facts, calls, answer)
            ok = s["tool_ok"] and s["facts_ok"]
            results.append({"name": f"history: {prompt[:40]} #{rep + 1}", "pass": ok,
                            "seconds": round(time.time() - t0, 1),
                            "calls": [c["tool"] for c in calls], "missing": s["missing"],
                            "invented": s["invented"], "answer": answer})
            print(f"  [{'PASS' if ok else 'FAIL'}] {prompt[:50]:50s} {[c['tool'] for c in calls]} "
                  f"{s['missing'] or ''}{s['invented'] or ''}", flush=True)
    return results


def world_layout():
    import yaml
    with open(os.path.join(SIM_DIR, "worlds", "generated", "park_info.yaml")) as f:
        return yaml.safe_load(f)["worlds"]["park"]["park"]


def score_world(name, case, layout, calls, answer, a):
    """The run the agent started, against the world."""
    from detect.bin_check import expected_from_layout
    checks = []
    motion = [c for c in calls if c["tool"] in ("run_route", "check_bin", "look_ahead")]
    first = motion[0] if motion else None
    want_args = case.get("bin")
    tool_ok = first is not None and first["tool"] == case["tool"]
    if tool_ok and want_args:
        import walle_tools
        tool_ok = walle_tools._bin_number(first["args"].get("bin")) == want_args
    checks.append(("right tool", tool_ok, [(c["tool"], c["args"]) for c in calls]))
    checks.append(("one motion run", len(motion) == 1, f"{len(motion)} motion calls"))
    if not first:
        return checks
    res = json.loads(first["output"])
    checks.append(("run done", res.get("status") == "done",
                   f"{res.get('status')} {res.get('reason') or res.get('error') or ''}"))
    text = norm(answer)
    # bins against the world
    bins = {b["corner"]: b for b in layout["bins"]}
    for b in res.get("bins_checked") or []:
        t = expected_from_layout(bins[b["bin"]])
        ok = (b["outcome"], b["overflow_level"]) == (t["outcome"], t["overflow_level"])
        checks.append((f"bin {b['bin']} judged right", ok,
                       f"{b['outcome']} level {b['overflow_level']} (truth {t['outcome']} "
                       f"level {t['overflow_level']})"))
        if t["outcome"] == "overflowing":
            named = bool(re.search(rf"bin\s*{b['bin']}\b[^\n]*overflow", text)
                         or re.search(rf"^\|\s*(bin\s*)?{b['bin']}\s*\|[^\n]*overflow", text,
                                      re.M))          # a table row (run 20261123_185359)
            checks.append((f"answer: bin {b['bin']} overflowing", named,
                           "named" if named else "not named on its line"))
        if case["tool"] == "check_bin":
            checks.append(("answer gives the level", str(b["overflow_level"]) in text,
                           f"level {b['overflow_level']}"))
    if case["tool"] == "check_bin":
        checks.append(("bin 1 checked", [b["bin"] for b in res.get("bins_checked") or []] == [1],
                       str([b["bin"] for b in res.get("bins_checked") or []])))
    # the obstacle against the world
    o = layout.get("obstacle")
    if o:
        found = res.get("obstacles_found") or []
        mine = [r for r in found if f"side {o['side']}" in r.get("where", "")]
        checks.append(("obstacle found on its side", bool(mine),
                       f"truth {o['item']} side {o['side']} at {o['at']:.2f}; found "
                       f"{[(r['label'], r['where']) for r in found]}"))
        if mine and case["tool"] == "run_route":
            checks.append(("obstacle driven around",
                           mine[0]["what_the_robot_did"] == "drove around it",
                           mine[0]["what_the_robot_did"]))
        if mine:
            rec = _record(res, o, layout)
            if rec is not None:
                checks.append((f"obstacle within {a.tol_cm:.0f} cm", rec <= a.tol_cm,
                               f"{rec:.1f} cm"))
            label = norm(mine[0]["label"])
            checks.append(("answer names the obstacle", label in text, label))
            checks.append(("answer gives its side",
                           bool(re.search(rf"side\s*{o['side']}", text)), f"side {o['side']}"))
    # item counts: "N <label>" in the answer must be a count the run gave for that label
    # (run 20261123_183505: "6 plastic bottles" where the run had 4)
    allowed = {}
    for b in res.get("bins_checked") or []:
        for group in ("on_top", "litter"):
            for label, n in (b.get(group) or {}).items():
                allowed.setdefault(norm(label), set()).add(n)
    wrong = []
    for label, counts in allowed.items():
        for m in re.finditer(rf"(\d+)\s+{re.escape(label)}s?\b", text):
            if int(m.group(1)) not in counts:
                wrong.append(f"{m.group(0)} (run: {sorted(counts)})")
    checks.append(("item counts as the run gave them", not wrong, "; ".join(wrong) or "ok"))
    import bench
    invented = sorted(n for n in bench.numbers(answer)
                      if n not in set().union(*[bench.numbers(c["output"]) for c in calls],
                                              bench.numbers(LOOK_PROMPT)))
    checks.append(("no invented numbers", not invented, str(invented)))
    return checks


def _record(res, o, layout):
    """Distance (cm) of the run's obstacle record from the truth, from obstacles.json."""
    run_dir = res.get("_run_dir")
    if not run_dir:
        return None
    try:
        with open(os.path.join(run_dir, "obstacles.json")) as f:
            recs = [r for r in json.load(f)["obstacles"]
                    if r.get("side") == o["side"] and r.get("obj_xy")
                    and r.get("action") != "not_confirmed"]
    except (OSError, ValueError, KeyError):
        return None
    if not recs:
        return None
    return min(math.hypot(r["obj_xy"][0] - o["position"][0],
                          r["obj_xy"][1] - o["position"][1]) for r in recs) * 100


def world_case(name, a, log_dir):
    import bench
    import runner as runner_mod
    import walle_agent
    import walle_tools
    case = WORLDS[name]
    launch = (f"ros2 launch {ot.LAUNCH} world:=park profile:={ot.PROFILE} gui:=false "
              f"scenario:={case['scenario']} bins:=scenario "
              f"obstacle:={case.get('obstacle', 'scenario')} park_seed:=scenario")
    log = open(os.path.join(log_dir, f"{name}_launch.log"), "w")
    proc = subprocess.Popen(["bash", "-c", launch], stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)
    t0 = time.time()
    try:
        if not ot.world_up(a.launch_timeout):
            return {"name": name, "pass": False, "error": "world did not come up"}
        time.sleep(3)
        layout = world_layout()
        cfg = walle_tools.load_config()
        if a.dwell is not None:
            cfg["route"]["extra_args"] = list(cfg["route"].get("extra_args") or []) + \
                ["--dwell", str(a.dwell)]
        run = runner_mod.Runner(cfg, sim=True)
        orig = runner_mod.digest_run

        def keep_dir(run_dir, kind="route"):        # the scorer needs the run folder too
            d = orig(run_dir, kind)
            d["_run_dir"] = run_dir
            return d
        runner_mod.digest_run = keep_dir
        walle_tools.configure("sim", cfg=cfg, runner=run,
                              approve=lambda d: print(f"[auto-approved in Gazebo] {d}") or True)
        agent = walle_agent.build_agent(cfg, verbose=False)
        prompt = LOOK_PROMPT if case["prompt"] is None else bench.PROMPTS[case["prompt"]][0]
        print(f"You> {prompt}", flush=True)
        answer = walle_agent.respond(agent, prompt)
        print(f"WALL-E> {answer}", flush=True)
        calls = list(walle_tools.CALLS)
        runner_mod.digest_run = orig
        checks = score_world(name, case, layout, calls, answer, a)
        for c in calls:                              # keep the log readable
            try:
                out = json.loads(c["output"])
                out.pop("_run_dir", None)
                c["output"] = json.dumps(out)
            except ValueError:
                pass
        for n, ok, d in checks:
            print(f"  [{'OK' if ok else 'FAIL'}] {n}: {d}")
        return {"name": name, "pass": all(ok for _, ok, _ in checks),
                "minutes": round((time.time() - t0) / 60, 1), "prompt": prompt,
                "answer": answer, "calls": calls,
                "checks": [{"name": n, "ok": ok, "detail": str(d)} for n, ok, d in checks]}
    finally:
        ot.stop_world(proc)
        log.close()


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--only", nargs="*", help="history, route, bin1, look")
    p.add_argument("--repeats", type=int, default=3, help="history prompt repeats")
    p.add_argument("--tol-cm", type=float, default=10.0)
    p.add_argument("--dwell", type=float, default=5.0, help="bin wait (s), to save time")
    p.add_argument("--launch-timeout", type=float, default=120.0)
    a = p.parse_args()
    parts = a.only or ["history", "route", "bin1", "look"]
    bad = [x for x in parts if x != "history" and x not in WORLDS]
    if bad:
        raise SystemExit(f"Unknown part {bad}; parts: history, {', '.join(WORLDS)}")
    if any(x in WORLDS for x in parts) and ot.leftovers():
        raise SystemExit("Sim nodes are already running in this container; stop them first "
                         "(each case launches its own world):\n" + "\n".join(ot.leftovers()))
    log_dir = os.path.join(SIM_DIR, "data", "agent_check", time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(log_dir, exist_ok=True)
    results = []
    for part in parts:
        print(f"\n===== {part} =====", flush=True)
        if part == "history":
            results += history_cases(a)
        else:
            results.append(world_case(part, a, log_dir))
        with open(os.path.join(log_dir, "agent_check.json"), "w") as f:
            json.dump({"results": results}, f, indent=1)
    passed = sum(r["pass"] for r in results)
    print(f"\n===== Phase 6 agent check: {passed} of {len(results)} passed =====")
    for r in results:
        fails = [f"{c['name']}: {c['detail']}" for c in r.get("checks", []) if not c["ok"]]
        print(f"  [{'PASS' if r['pass'] else 'FAIL'}] {r['name']}"
              + (f"  {r.get('error')}" if r.get("error") else "")
              + (f"  missing {r['missing']}" if r.get("missing") else "")
              + (f"  invented {r['invented']}" if r.get("invented") else "")
              + "".join(f"\n        - {x}" for x in fails))
    print(f"Logs: {log_dir}")
    ok = passed == len(results)
    print("PASS" if ok else "FAIL")
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
