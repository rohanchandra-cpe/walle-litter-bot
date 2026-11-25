"""How the WALL-E agent moves the robot: the route follower's CLI as a subprocess
(Phase 6 plan, decision 3). The follower keeps every guard it has (stale data, Pico
restart, estimate jump, never driving tilted, per-move timeouts); this file never sends a
motion command itself, only a stop.

- run_route(): one lap (obstacle looks and detours on, bins optional);
- check_bin(n): the partial route to bin n and back (follower --trip-bin n);
- look_ahead(): one down-tilted photo where the robot stands (follower --look-only);
- stop(): Ctrl+C to a running follower, else a zero /pico/cmd_vel.

The follower's output is passed through to the terminal ("[route] ..."). Ctrl+C in the
terminal reaches the follower too (same process group): it stops the robot and writes its
files, and the tool reports the run as stopped. Each run's files are turned into a short
summary for the model (digest_run), which the model test also uses on old runs.

In Gazebo the layout comes from the launched world (walle-sim/worlds/generated/
park_info.yaml, as the sim checks do) and runs go to walle-sim/data/agent/; on the robot it
is agent.yaml's scenario, and runs go to walle-app/results/route/ like any other run.
"""

import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime

AGENT_DIR = os.path.dirname(os.path.abspath(__file__))
APP_DIR = os.path.dirname(AGENT_DIR)
REPO_DIR = os.path.dirname(APP_DIR)
FOLLOWER = os.path.join(APP_DIR, "route", "route_follower.py")
SIM_PARK_INFO = os.path.join(REPO_DIR, "walle-sim", "worlds", "generated", "park_info.yaml")
SIM_RUNS = os.path.join(REPO_DIR, "walle-sim", "data", "agent")
sys.path.insert(0, AGENT_DIR)

ACTIONS = {"detour": "drove around it", "blocked": "stopped: no way around it",
           "not_confirmed": "a closer look found nothing there", None: "only seen"}


def _load(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def _minutes(summary):
    try:
        t0 = datetime.fromisoformat(summary["started_utc"])
        t1 = datetime.fromisoformat(summary["ended_utc"])
    except (KeyError, TypeError, ValueError):
        return None
    return round((t1 - t0).total_seconds() / 60.0, 1)


def count_labels(labels):
    """{'plastic_bottle': 4, 'can': 2}, most first: the model copies counts, it doesn't count
    (sim agent check 20261123_183505: the 4B read a 10-item list as 6 bottles, not 4)."""
    counts = {}
    for label in labels:
        counts[label] = counts.get(label, 0) + 1
    return dict(sorted(counts.items(), key=lambda kv: (-kv[1], str(kv[0]))))


def digest_bins(run_dir, layout=None):
    data = _load(os.path.join(run_dir, "bin_checks.json")) or {}
    types = {b["corner"]: b.get("type") for b in (layout or {}).get("bins", [])}
    out = []
    for c in data.get("checks", []):
        out.append({"bin": c.get("bin_id"), "type": types.get(c.get("bin_id")),
                    "outcome": c.get("outcome"), "overflow_level": c.get("overflow_level"),
                    "items_on_top": c.get("on_top_count"),
                    "on_top": count_labels(i.get("label") for i in c.get("on_top", [])),
                    "litter_around": c.get("litter_count"),
                    "litter": count_labels(i.get("label") for i in c.get("litter", [])),
                    "method": c.get("method")})
    return sorted(out, key=lambda b: (b["bin"] is None, b["bin"] or 0))


def digest_obstacles(run_dir):
    import walle_tools
    data = _load(os.path.join(run_dir, "obstacles.json"))
    if data is None:
        return None, []
    out = []
    for o in data.get("obstacles", []):
        rec = {"label": o.get("label"), "category": o.get("category"),
               "where": walle_tools.where_on_route(o.get("obj_xy"), o.get("side"))
               if walle_tools.CTX.corners else f"side {o.get('side')}",
               "found_by": o.get("found_by"),
               "what_the_robot_did": ACTIONS.get(o.get("action"), o.get("action"))}
        if o.get("ahead_m") is not None:          # look-only: relative to the robot
            rec["ahead_m"] = o["ahead_m"]
            rec["left_m"] = o.get("left_m")
        out.append(rec)
    return data.get("sides_done", []), out


def digest_run(run_dir, kind="route"):
    """The run's files -> what the model needs to answer (no file paths, no raw boxes)."""
    s = _load(os.path.join(run_dir, "summary.json"))
    if s is None:
        return {"status": "no record", "error": f"The run wrote no summary.json in {run_dir}."}
    layout = _load(os.path.join(run_dir, "layout.json"))
    sides, obstacles = digest_obstacles(run_dir)
    end = (s.get("end_errors") or {}).get("ground_truth") or (s.get("end_errors") or {}).get("odom")
    res = {"run_id": s.get("run_id"), "kind": kind, "status": s.get("status"),
           "reason": s.get("reason"), "minutes": _minutes(s),
           "distance_m": round(s["distance_m"], 2) if s.get("distance_m") is not None else None,
           "end_error_cm": round(100 * end["position_m"], 1) if end else None}
    if kind != "look":
        res["bins_checked"] = digest_bins(run_dir, layout)
    if sides is not None:
        res["sides_checked_for_obstacles"] = sides
        res["obstacles_found"] = obstacles
        if not obstacles:
            res["obstacles_note"] = ("no obstacles found" if sides or kind == "look"
                                     else "no side was checked")
    return res


class Runner:
    def __init__(self, cfg, sim=False):
        self.cfg = cfg["route"]
        self.sim = bool(sim)
        self.proc = None
        self._cmd_pub = None

    # ----- the follower -----
    def _base(self):
        cmd = [sys.executable, "-u", FOLLOWER, "--no-measure"]
        if self.sim:
            import yaml
            with open(SIM_PARK_INFO) as f:
                layout = yaml.safe_load(f)["worlds"]["park"]["park"]
            run_dir = os.path.join(SIM_RUNS, datetime.now().strftime("%Y%m%d_%H%M%S"))
            os.makedirs(run_dir, exist_ok=True)
            truth = os.path.join(run_dir, "layout_truth.json")
            with open(truth, "w") as f:
                json.dump(layout, f, indent=2)
            cmd += ["--layout-json", truth, "--truth-layout", truth, "--run-dir", run_dir,
                    "--camera-url", self.cfg["sim_camera_url"]]
        else:
            cmd += ["--scenario", self.cfg["scenario"], "--camera-url", self.cfg["camera_url"]]
        return cmd + [str(a) for a in self.cfg.get("extra_args") or []]

    def _run(self, extra, kind):
        cmd = self._base() + extra
        print(f"[route] starting: {' '.join(os.path.relpath(c, REPO_DIR) if c.startswith(REPO_DIR) else c for c in cmd[1:])}")
        run_dir = None
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, text=True, bufsize=1, cwd=REPO_DIR)
        interrupted = False
        try:
            for line in self.proc.stdout:
                line = line.rstrip("\n")
                print(f"[route] {line}", flush=True)
                if line.startswith("Run log: "):
                    run_dir = line[len("Run log: "):].strip()
            self.proc.wait()
        except KeyboardInterrupt:
            interrupted = True
            self._interrupt(signalled=True)
            for line in self.proc.stdout or []:
                print(f"[route] {line.rstrip()}", flush=True)
                if line.startswith("Run log: "):
                    run_dir = line[len("Run log: "):].strip()
        finally:
            code = self.proc.poll()
            self.proc = None
        if run_dir is None:
            res = {"status": "stopped" if interrupted else "failed",
                   "error": f"The follower ended (exit code {code}) without saving a run."}
        else:
            res = digest_run(run_dir, kind)
        if interrupted:
            res["stopped_by"] = "Ctrl+C"
            raise KeyboardInterrupt(json.dumps(res))
        return res

    def _interrupt(self, signalled=False):
        """Stop a running follower and wait for it to stop the robot and save.
        signalled: Ctrl+C in the terminal has already reached it (same process group), so
        don't send another: a second one broke its stop and lost the run's files (run
        20261125_200224). Only if it is still running after stop_wait_s does it get one (or,
        when not signalled, at once); then it is killed. A zero /pico/cmd_vel follows either
        way, and the Pico's 500 ms watchdog stops the motors once commands end."""
        p = self.proc
        if p is not None and p.poll() is None:
            wait = float(self.cfg.get("stop_wait_s", 10))
            try:
                if not signalled:
                    p.send_signal(signal.SIGINT)
                p.wait(timeout=wait)
            except subprocess.TimeoutExpired:
                try:
                    if signalled:
                        p.send_signal(signal.SIGINT)
                        p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    pass
                if p.poll() is None:
                    p.kill()
                    p.wait(timeout=5)
        try:
            self._zero_cmd_vel()
        except Exception as e:            # the follower stopped it; the watchdog backs it up
            print(f"[route] (backup stop command not sent: {e})", flush=True)

    # ----- what the tools call -----
    def run_route(self, check_bins=True):
        return self._run([] if check_bins else ["--no-bins"], "route")

    def check_bin(self, n):
        return self._run(["--trip-bin", str(int(n))], "trip")

    def look_ahead(self):
        return self._run(["--look-only"], "look")

    def stop(self):
        if self.proc is not None and self.proc.poll() is None:
            self._interrupt()
            return "Stopped the running route; the robot is stopped."
        self._zero_cmd_vel()
        return "Nothing was running. Sent a stop command; the robot is stopped."

    def _zero_cmd_vel(self):
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.signals import SignalHandlerOptions
        if not rclpy.ok():
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        if self._cmd_pub is None:
            self._node = rclpy.create_node("walle_agent_stop")
            self._cmd_pub = self._node.create_publisher(Twist, "/pico/cmd_vel", 10)
        for _ in range(5):
            self._cmd_pub.publish(Twist())
            time.sleep(0.05)

    def close(self):
        if self.proc is not None:
            self._interrupt()
        if self._cmd_pub is not None:
            self._node.destroy_node()
