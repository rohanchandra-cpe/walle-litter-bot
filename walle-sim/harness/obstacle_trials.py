"""Phase 5 sim check: obstacles at random places on the route, one world per case.

For each case it launches the park world headless (the world is built at launch), waits
for Gazebo's ground truth, runs `sim_trials.py route-check` (the route follower with the
ground-truth detector; obstacle found, recorded within --tol-cm, no collision, lap ends
within 5 cm, memory store), stops the world, and moves on. Prints a table and PASS when
at least --need cases of --runs pass (ROADMAP Phase 5: 9 of 10, 10 cm).

Run inside the sim container with no other launch running (`make sim-obstacle-check`):

    python3 /workspace/walle-sim/harness/obstacle_trials.py                 # all 10 cases
    python3 /workspace/walle-sim/harness/obstacle_trials.py --only near lying
    python3 /workspace/walle-sim/harness/obstacle_trials.py --runs 3        # the first 3

Cases: five random obstacles (scenario obstacle_only, other seeds), the three hard
scenarios (obstacle_near at 0.25, obstacle_far at 0.75, obstacle_lying: a bottle across
the path), one with the sonar dead (the camera alone), one the camera misses (the sonar
alone). Each takes ~4-6 min (a 30 s launch, then the lap with its bin stops).
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time

SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TRIALS = os.path.join(SIM_DIR, "harness", "sim_trials.py")
LAUNCH = os.path.join(SIM_DIR, "launch", "sim.launch.py")
PROFILE = "plastic_pid"            # the robot's firmware + surface

CASES = [
    {"name": f"random_{s}", "scenario": "obstacle_only", "seed": s} for s in (11, 12, 13, 14, 15)
] + [
    {"name": "near", "scenario": "obstacle_near"},
    {"name": "far", "scenario": "obstacle_far"},
    {"name": "lying", "scenario": "obstacle_lying"},
    {"name": "sonar_dead", "scenario": "obstacle_only", "seed": 16,
     "profile": "plastic_pid_sonar_dead"},
    {"name": "camera_miss", "scenario": "obstacle_only", "seed": 17, "args": ["--hide-obstacle"]},
]


def sh(cmd):
    return subprocess.call(["bash", "-c", cmd])


# Every node a park launch starts (a leftover from one case corrupts the next: 2026-11-22,
# twelve odometry_node.py copies all published /wheel/odom). Bracketed so pkill doesn't
# match its own shell (CLAUDE.md).
WORLD_PATTERNS = ("[r]os2 launch", "[i]gn gazebo", "[g]z sim", "[r]uby.*gz",
                  "[s]im_pico_bridge", "[o]dometry_node", "[e]kf_node", "[p]arameter_bridge",
                  "[c]amera_", "[r]obot_state_publisher", "[j]ump_ekf")


def leftovers():
    out = subprocess.run(["bash", "-c", "pgrep -af '" + "|".join(WORLD_PATTERNS) + "'"],
                         capture_output=True, text=True).stdout.strip()
    return [line for line in out.splitlines() if "obstacle_trials" not in line]


def stop_world(proc=None):
    if proc is not None:                 # the whole launch's process group first
        for sig, wait in ((signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 1)):
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                break
            time.sleep(wait)
    for sig in ("INT", "KILL"):
        for pat in WORLD_PATTERNS:
            sh(f"pkill -{sig} -f '{pat}' 2>/dev/null")
        time.sleep(2)
    left = leftovers()
    if left:
        raise SystemExit("Could not stop the world; still running:\n" + "\n".join(left))


def world_up(timeout_s):
    end = time.time() + timeout_s
    while time.time() < end:
        if sh("timeout 5 ros2 topic echo --once /sim/ground_truth >/dev/null 2>&1") == 0 and \
                sh("timeout 5 ros2 topic echo --once /odometry/filtered >/dev/null 2>&1") == 0:
            return True
        time.sleep(2)
    return False


def run_case(case, a, log_dir):
    profile = case.get("profile", PROFILE)
    launch = (f"ros2 launch {LAUNCH} world:=park profile:={profile} gui:=false "
              f"scenario:={case['scenario']} bins:=scenario obstacle:=scenario "
              f"park_seed:={case.get('seed', 'scenario')}")
    log = open(os.path.join(log_dir, f"{case['name']}_launch.log"), "w")
    proc = subprocess.Popen(["bash", "-c", launch], stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)
    t0 = time.time()
    try:
        if not world_up(a.launch_timeout):
            return {"name": case["name"], "pass": False, "error": "world did not come up"}
        time.sleep(3)
        cmd = [sys.executable, TRIALS, "--run", a.run, "route-check",
               "--obstacle-tol-cm", str(a.tol_cm)] + case.get("args", [])
        if a.dwell is not None:
            cmd += ["--dwell", str(a.dwell)]
        out_path = os.path.join(log_dir, f"{case['name']}_check.log")
        with open(out_path, "w") as out:
            code = subprocess.call(cmd, stdout=out, stderr=subprocess.STDOUT)
        text = open(out_path).read()
        print(text[-2500:] if len(text) > 2500 else text)
        result = {"name": case["name"], "pass": code == 0, "exit": code,
                  "minutes": round((time.time() - t0) / 60, 1)}
        for line in text.splitlines():
            if line.startswith("Results: "):
                rc = os.path.join(line.split("Results: ", 1)[1].strip(), "route_check.json")
                if os.path.exists(rc):
                    with open(rc) as f:
                        rj = json.load(f)
                    result["results"] = os.path.dirname(rc)
                    result["checks"] = {c["name"]: (c["ok"], c["detail"]) for c in rj["checks"]}
        return result
    finally:
        stop_world(proc)
        log.close()


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--runs", type=int, default=len(CASES), help="the first N cases")
    p.add_argument("--only", nargs="*", help="case names (random_11 ... camera_miss)")
    p.add_argument("--need", type=int, help="passes needed (default 9 of 10, scaled)")
    p.add_argument("--tol-cm", type=float, default=10.0)
    p.add_argument("--dwell", type=float, default=5.0, help="bin wait (s), to save time")
    p.add_argument("--run", default="obstacle_check", help="walle-sim/data/<run>/")
    p.add_argument("--launch-timeout", type=float, default=120.0)
    a = p.parse_args()
    cases = [c for c in CASES if c["name"] in a.only] if a.only else CASES[:a.runs]
    if not cases:
        raise SystemExit(f"No such case. Cases: {', '.join(c['name'] for c in CASES)}")
    need = a.need if a.need is not None else max(1, round(len(cases) * 0.9 - 1e-9))
    if leftovers():
        raise SystemExit("Sim nodes are already running in this container; stop them first "
                         "(each case launches its own world):\n" + "\n".join(leftovers()))
    log_dir = os.path.join(SIM_DIR, "data", a.run, "obstacle_check",
                           time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(log_dir, exist_ok=True)
    results = []
    for k, case in enumerate(cases, 1):
        print(f"\n===== case {k}/{len(cases)}: {case['name']} ({case['scenario']}"
              + (f", seed {case['seed']}" if "seed" in case else "")
              + (f", {case['profile']}" if "profile" in case else "")
              + (f", {' '.join(case['args'])}" if case.get("args") else "") + ") =====",
              flush=True)
        results.append(run_case(case, a, log_dir))
        with open(os.path.join(log_dir, "obstacle_check.json"), "w") as f:
            json.dump({"cases": results, "need": need, "tol_cm": a.tol_cm}, f, indent=1)
    passed = sum(r["pass"] for r in results)
    print(f"\n===== Phase 5 obstacle check: {passed} of {len(results)} passed "
          f"(need {need}; position within {a.tol_cm:.0f} cm) =====")
    for r in results:
        fails = [f"{n}: {d}" for n, (ok, d) in (r.get("checks") or {}).items() if not ok]
        pos = (r.get("checks") or {}).get(f"obstacle position within {a.tol_cm:.0f} cm")
        print(f"  [{'PASS' if r['pass'] else 'FAIL'}] {r['name']:12s} "
              + (pos[1].split(' (')[0] if pos else "")
              + (f"  {r.get('error')}" if r.get("error") else "")
              + ("".join(f"\n        - {x}" for x in fails)))
    print(f"Logs: {log_dir}")
    ok = passed >= need
    print("PASS" if ok else "FAIL")
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
