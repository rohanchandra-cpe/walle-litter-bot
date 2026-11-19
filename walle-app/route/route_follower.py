"""WALL-E route follower (ROADMAP Phase 1): drive the park route on the EKF estimate.

Drives the square from walle-app/park.py corner to corner, stops on each corner and turns
in place, turns to face each bin and waits there, and comes back to the start facing the
way it started. Run it while the robot side is up (real robot: `make launch`; Gazebo:
`make sim-launch WORLD=park ...`), from the repository root or inside the container:

    python3 walle-app/route/route_follower.py                      # default scenario, 1 lap
    python3 walle-app/route/route_follower.py --scenario default --bins 2 --dwell 15
    python3 walle-app/route/route_follower.py --camera-url http://192.168.1.50/capture

Put the robot on corner 1 facing corner 2 first: the pose the EKF reports when the run
starts is taken to be the park's start pose. PRESS CTRL+C AT ANY TIME TO STOP THE ROBOT.
The Pico also stops by itself 500 ms after the last command.

Built on module 8's move_test.Mover (walle-pilot/odometry/move_test.py, imported, not
changed): its feedback subscriptions, send/stop, coast compensation, and turns. What is new:
- sides follow the LINE between corners (cross-track correction), so a turn that ends a
  few degrees off is corrected on the next side instead of carried along it;
- turns go to ABSOLUTE park headings, so turn errors do not add up;
- coast compensation is learned during the run (separately for sides and turns) from each
  move's overshoot, and a corner or turn that still misses by more than a tolerance gets
  a short correction move (with the old open-loop firmware these overshot; the
  closed-loop wheel speed firmware of 2026-11-18 turns slowly enough to land them);
- the EKF heading drifts with the gyro bias. The follower stands still for a few seconds
  at the start and measures that drift, adds every bin wait to the measurement, removes
  the drift from the heading while driving, and drops what built up during each wait
  (a zero-velocity update: the encoders show the robot did not move);
- progress on /walle/route/status (JSON) and a run log in walle-app/results/route/<run>/.
On the real robot it then asks for the tape-measured end position (skip with --no-measure).
"""

import argparse
import csv
import json
import math
import os
import sys
import threading
import time
from datetime import datetime, timezone

APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_DIR = os.path.dirname(APP_DIR)
sys.path[:0] = [APP_DIR, os.path.dirname(os.path.abspath(__file__)),
                os.path.join(REPO_DIR, "walle-pilot", "odometry")]

import yaml  # noqa: E402

import park  # noqa: E402
import route_geometry as rg  # noqa: E402

ROUTE_CONFIG = os.path.join(APP_DIR, "config", "route.yaml")
SIM_ROBOT = "walle"          # the robot's model name in walle-sim


def load_config(path=ROUTE_CONFIG):
    with open(path) as f:
        return yaml.safe_load(f)


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def make_follower_class():
    """Import ROS only when a run starts, so the geometry and tests need no ROS."""
    from move_test import Mover, yaw
    from std_msgs.msg import String

    class RouteFollower(Mover):
        def __init__(self, cfg, layout, run_dir):
            super().__init__(node_name="walle_route", feedback_topic=cfg["feedback_topic"])
            self.cfg, self.layout, self.run_dir = cfg, layout, run_dir
            self.status_pub = self.create_publisher(String, cfg["status_topic"], 10)
            self.anchors = {}
            self.state = {"state": "starting", "step": None, "corner": None}
            self.distance = 0.0
            self.events = []
            self.logging = False
            # Coast compensation (Mover.lead): separate for sides and turns, learned during
            # the run from each move's overshoot
            self.coast_side = cfg["coast"]["side_s"]
            self.coast_turn = cfg["coast"]["turn_s"]
            self.corrections = 0
            self.zero_velocity_deg = 0.0     # heading drift removed while standing still
            self.bias = rg.BiasEstimate()    # gyro bias, from the still periods
            self.holds = []

        # walle-sim names the robot "walle", not module 8's "course_robot"
        def _on_truth(self, msg):
            for t in msg.transforms:
                if t.child_frame_id == SIM_ROBOT:
                    with self.lock:
                        self.poses["ground_truth"] = (t.transform.translation.x,
                                                      t.transform.translation.y,
                                                      yaw(t.transform.rotation))

        @property
        def in_sim(self):
            return self.count_publishers("/sim/ground_truth") > 0

        # ----- frames -----
        def anchor(self):
            start = self.layout["route"]["start"]
            park_start = (start[0], start[1], math.radians(start[2]))
            for key in ("odom", "wheel"):
                if self.pose(key) is not None:
                    self.anchors[key] = rg.Anchor(self.pose(key), park_start)

        def park_pose(self, key="odom"):
            """An estimate in the park frame; ground truth is in it already. The fused
            estimate also has the measured gyro bias removed (see hold_still)."""
            p = self.pose(key)
            if p is None:
                return None
            if key == "ground_truth":
                return p
            return self.anchors[key].to_park(p, time.monotonic() if key == "odom" else None)

        # ----- progress -----
        def report(self, **change):
            self.state.update(change)
            p = self.park_pose()
            msg = dict(self.state, distance_m=round(self.distance, 3), time_utc=utc_now(),
                       pose=[round(p[0], 3), round(p[1], 3), round(math.degrees(p[2]), 1)]
                       if p else None)
            self.status_pub.publish(String(data=json.dumps(msg)))
            return msg

        def logger(self):
            """trajectory.csv at log_rate_hz; also adds up the distance driven."""
            period = 1.0 / self.cfg["log_rate_hz"]
            t0, last = time.monotonic(), None
            path = os.path.join(self.run_dir, "trajectory.csv")
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["time_s", "source", "x_m", "y_m", "theta_deg", "state", "corner"])
                while self.logging:
                    t = round(time.monotonic() - t0, 2)
                    for key, label in (("odom", "fused"), ("wheel", "wheel"),
                                       ("ground_truth", "ground_truth")):
                        p = self.park_pose(key) if (key in self.anchors or
                                                    key == "ground_truth") else None
                        if p:
                            w.writerow([t, label, f"{p[0]:.4f}", f"{p[1]:.4f}",
                                        f"{math.degrees(p[2]):.2f}", self.state["state"],
                                        self.state["corner"]])
                    p = self.park_pose()
                    if last and p:
                        self.distance += math.hypot(p[0] - last[0], p[1] - last[1])
                    last = p
                    time.sleep(period)

        # ----- moves -----
        def drive_side(self, a, b, heading):
            """Follow the side a -> b and stop on b. If the robot ends more than
            correct_tol_m past (or short of) b, creep back (or on) along the side."""
            sc = self.cfg["side"]
            self.side_leg(a, b, heading, follow_line=True, learn=True)
            for _ in range(sc["max_corrections"]):
                _, remaining, _ = rg.side_progress(self.park_pose(), a, b)
                if abs(remaining) <= sc["correct_tol_m"]:
                    break
                self.corrections += 1
                self.side_leg(a, b, heading, follow_line=False, learn=False)

        def side_leg(self, a, b, heading, follow_line, learn):
            sc, cc = self.cfg["side"], self.cfg["coast"]
            max_corr = math.radians(sc["max_correction_deg"])
            end = time.time() + sc["timeout_s"]
            dt = 1.0 / self.cfg["command_rate_hz"]
            _, remaining, _ = rg.side_progress(self.park_pose(), a, b)
            sign = 1 if remaining >= 0 else -1          # -1: overshot, back up
            self.coast_time = self.coast_side
            v_stop = 0.0
            while True:
                if time.time() > end:
                    self.stop()
                    raise RuntimeError(f"side to {b} not finished in {sc['timeout_s']} s")
                p = self.park_pose()
                _, remaining, cross = rg.side_progress(p, a, b)
                with self.lock:
                    v_now = self.rates[0]
                if sign * remaining <= sc["tolerance_m"] + self.lead(0, sign):
                    v_stop = abs(v_now)
                    break
                v = sign * rg.clamp(sc["speed_gain"] * abs(remaining), sc["min_speed_mps"],
                                    sc["max_speed_mps"])
                # Short corrections (and backing up) hold the side's heading only
                aim = (rg.line_heading(heading, cross, sc["cross_track_gain"], max_corr)
                       if follow_line and sign > 0 else heading)
                w = rg.clamp(sc["heading_gain"] * rg.wrap(aim - p[2]),
                             -sc["max_turn_rps"], sc["max_turn_rps"])
                self.send(v, w)
                time.sleep(dt)
            self.stop()
            self.settle()
            # Learn the coast: how far past the stop point it went, per m/s at the stop.
            # Only from full moves: a correction starts from rest and spins down differently.
            _, remaining, _ = rg.side_progress(self.park_pose(), a, b)
            over = -sign * remaining
            if learn and cc["adapt"] and v_stop > 0.03:
                self.coast_side = rg.clamp(self.coast_side + cc["adapt_gain"] * over / v_stop,
                                           0.0, cc["max_s"])

        def turn_to(self, heading):
            """Turn in place to an absolute park heading, then correct a miss larger than
            correct_tol_deg with up to max_corrections more turns."""
            tc = self.cfg["turn"]
            self.turn_leg(heading, learn=True)
            for _ in range(tc["max_corrections"]):
                if abs(rg.wrap(heading - self.park_pose()[2])) <= math.radians(tc["correct_tol_deg"]):
                    break
                self.corrections += 1
                self.turn_leg(heading, learn=False)

        def turn_leg(self, heading, learn):
            """Module 8's Mover.turn, to an absolute heading, with its own learned coast."""
            tc, cc = self.cfg["turn"], self.cfg["coast"]
            tol = math.radians(tc["tolerance_deg"])
            err = rg.wrap(heading - self.park_pose()[2])
            sign = math.copysign(1.0, err)
            end = time.time() + abs(err) / tc["min_rate_rps"] * 1.5 + 5
            dt = 1.0 / self.cfg["command_rate_hz"]
            self.coast_time = self.coast_turn
            w_stop = 0.0
            while time.time() < end:
                err = rg.wrap(heading - self.park_pose()[2])
                with self.lock:
                    w_now = self.rates[1]
                if sign * err <= tol + self.lead(1, sign):
                    w_stop = abs(w_now)
                    break
                self.send(0.0, sign * rg.clamp(1.5 * abs(err), tc["min_rate_rps"],
                                               tc["max_rate_rps"]))
                time.sleep(dt)
            self.stop()
            self.settle()
            over = -sign * rg.wrap(heading - self.park_pose()[2])
            if learn and cc["adapt"] and w_stop > 0.2:
                self.coast_turn = rg.clamp(self.coast_turn + cc["adapt_gain"] * over / w_stop,
                                           0.0, cc["max_s"])

        def hold_still(self, seconds, photo_name=None):
            """Wait in place. The EKF heading drifts while the robot stands still (gyro
            bias). If the wheel encoders show the robot did not move:
            - the drift seen during the wait is added to the gyro bias estimate, and
            - the fused estimate is re-anchored to the pose it had when the wait began
              (a zero-velocity update), drifting from then on at minus the bias estimate.
            Returns (photo path or None, heading drift in deg or None)."""
            p0, w0 = self.park_pose(), self.pose("wheel")
            raw0, t0 = self.pose("odom")[2], time.monotonic()
            t_end = time.time() + seconds
            time.sleep(min(1.0, seconds))
            shot = self.photo(photo_name) if photo_name else None
            time.sleep(max(0.0, t_end - time.time()))
            w1 = self.pose("wheel")
            zc = self.cfg["zero_velocity"]
            if not zc["enabled"] or w0 is None or w1 is None:
                return shot, None
            moved = math.hypot(w1[0] - w0[0], w1[1] - w0[1])
            turned = abs(rg.wrap(w1[2] - w0[2]))
            if moved > zc["max_move_m"] or turned > math.radians(zc["max_turn_deg"]):
                return shot, None
            raw1, t1 = self.pose("odom")[2], time.monotonic()
            drift = math.degrees(rg.wrap(self.park_pose()[2] - p0[2]))
            if zc["estimate_bias"]:
                self.bias.add(rg.wrap(raw1 - raw0), t1 - t0)
            rate = self.bias.rate if zc["estimate_bias"] else 0.0
            self.anchors["odom"] = rg.Anchor(self.pose("odom"), p0, t1, rate)
            self.zero_velocity_deg += drift
            self.holds.append({"seconds": round(t1 - t0, 2), "raw_drift_deg":
                               round(math.degrees(rg.wrap(raw1 - raw0)), 3),
                               "bias_dps": round(math.degrees(self.bias.rate), 4)})
            return shot, round(drift, 3)

        def settle(self):
            time.sleep(self.cfg["turn"]["settle_s"])

        def snapshot(self):
            out = {}
            for key in ("odom", "wheel", "ground_truth"):
                p = self.park_pose(key) if (key in self.anchors or key == "ground_truth") else None
                if p:
                    out[key] = [round(p[0], 4), round(p[1], 4), round(math.degrees(p[2]), 2)]
            return out

        def photo(self, name):
            url = self.cfg.get("camera_url")
            if not url:
                return None
            import urllib.request
            try:
                with urllib.request.urlopen(url, timeout=5.0) as reply:
                    data = reply.read()
            except OSError as e:
                print(f"  (no photo: {e})")
                return None
            path = os.path.join(self.run_dir, "photos", f"{name}.jpg")
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "wb") as f:
                f.write(data)
            return os.path.relpath(path, self.run_dir)

        def run(self, steps):
            bins = {b["name"]: b for b in self.layout["bins"]}
            for i, step in enumerate(steps):
                kind = step[0]
                if kind == "drive":
                    _, c_from, c_to, a, b, heading = step
                    print(self.fmt(self.report(state="driving", step=i, corner=c_to)))
                    self.drive_side(a, b, heading)
                    snap = self.snapshot()
                    ev = {"step": i, "event": "arrived", "corner": c_to, "target": list(b),
                          "poses": snap, "distance_m": round(self.distance, 3),
                          "coast_side_s": round(self.coast_side, 3),
                          "coast_turn_s": round(self.coast_turn, 3),
                          "corrections": self.corrections}
                    if "ground_truth" in snap:
                        g = snap["ground_truth"]
                        ev["true_error_m"] = round(math.hypot(g[0] - b[0], g[1] - b[1]), 4)
                    self.events.append(ev)
                elif kind == "face_bin":
                    _, corner, name, heading = step
                    print(self.fmt(self.report(state="facing_bin", step=i, corner=corner,
                                               bin=name)))
                    self.turn_to(heading)
                    self.events.append(self.bin_event(i, corner, bins[name], heading))
                elif kind == "dwell":
                    _, corner, name, seconds = step
                    print(self.fmt(self.report(state="at_bin", step=i, corner=corner, bin=name))
                          + f"  waiting {seconds:.0f} s")
                    shot, drift = self.hold_still(seconds, f"{name}_step{i}")
                    if shot:
                        self.events[-1]["photo"] = shot
                    if drift is not None:
                        self.events[-1]["held_heading_drift_deg"] = drift
                    self.state.pop("bin", None)
                elif kind == "turn":
                    _, corner, heading = step
                    print(self.fmt(self.report(state="turning", step=i, corner=corner)))
                    self.turn_to(heading)

        def bin_event(self, i, corner, b, heading):
            snap = self.snapshot()
            ev = {"step": i, "event": "facing_bin", "corner": corner, "bin": b["name"],
                  "target_heading_deg": round(math.degrees(heading), 2), "poses": snap}
            est = snap["odom"]
            ev["est_heading_error_deg"] = round(rg.wrap(math.radians(est[2]) - heading)
                                                * 180 / math.pi, 2)
            if "ground_truth" in snap:
                g = snap["ground_truth"]
                bearing = math.atan2(b["position"][1] - g[1], b["position"][0] - g[0])
                ev["true_bearing_error_deg"] = round(math.degrees(
                    rg.wrap(bearing - math.radians(g[2]))), 2)
            return ev

        def fmt(self, msg):
            p = msg["pose"]
            where = f"({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+6.1f} deg)" if p else "(no pose)"
            extra = f" {msg['bin']}" if msg.get("bin") else ""
            if msg["corner"] is None:
                return f"[{msg['state']:10s}] est {where}"
            return (f"[{msg['state']:10s}] corner {msg['corner']}{extra}  est {where}  "
                    f"driven {msg['distance_m']:.2f} m")

    return RouteFollower


def end_report(node, started, status, reason):
    start = node.layout["route"]["start"]
    snap = node.snapshot()
    out = {"started_utc": started, "ended_utc": utc_now(), "status": status,
           "reason": reason, "source": "sim" if node.in_sim else "robot",
           "distance_m": round(node.distance, 3), "start": start, "end_poses": snap,
           "end_errors": {}}
    for key, p in snap.items():
        out["end_errors"][key] = {
            "position_m": round(math.hypot(p[0] - start[0], p[1] - start[1]), 4),
            "dx_m": round(p[0] - start[0], 4), "dy_m": round(p[1] - start[1], 4),
            "heading_deg": round((p[2] - start[2] + 180) % 360 - 180, 2)}
    return out


def ask_measurements(summary):
    """Real robot: the tape-measured end pose and whether it faced each bin."""
    print("\nMeasure where the robot stopped, relative to the start mark (park frame: "
          "x toward corner 2, y toward corner 4). Press Enter to skip a value.")

    def ask(prompt, cast=float):
        raw = input(prompt).strip()
        try:
            return cast(raw) if raw else None
        except ValueError:
            return None

    m = {"dx_cm": ask("  end x offset (cm): "), "dy_cm": ask("  end y offset (cm): "),
         "heading_deg": ask("  end heading error (deg, + = turned left of start): "),
         "faced_bins": {}}
    for ev in summary["events"]:
        if ev["event"] == "facing_bin":
            m["faced_bins"][ev["bin"]] = ask(f"  {ev['bin']} centered in view? (y/n): ",
                                             lambda s: s.lower().startswith("y"))
    m["notes"] = input("  notes (surface, anything odd): ").strip()
    if m["dx_cm"] is not None and m["dy_cm"] is not None:
        m["position_error_cm"] = round(math.hypot(m["dx_cm"], m["dy_cm"]), 1)
    return m


def append_robot_row(results_dir, run_id, summary):
    m = summary.get("measured", {})
    est = summary["end_errors"].get("odom", {})
    path = os.path.join(results_dir, "robot_runs.csv")
    new = not os.path.exists(path)
    with open(path, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["run_id", "started_utc", "status", "scenario", "laps", "bins",
                        "measured_dx_cm", "measured_dy_cm", "measured_error_cm",
                        "measured_heading_deg", "est_error_cm", "distance_m",
                        "bins_faced", "notes"])
        faced = m.get("faced_bins", {})
        w.writerow([run_id, summary["started_utc"], summary["status"], summary["scenario"],
                    summary["laps"], len(summary["bins"]), m.get("dx_cm"), m.get("dy_cm"),
                    m.get("position_error_cm"), m.get("heading_deg"),
                    round(est.get("position_m", 0) * 100, 1), summary["distance_m"],
                    f"{sum(1 for v in faced.values() if v)}/{len(faced)}", m.get("notes", "")])
    return path


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="Drive the WALL-E park route (Ctrl+C stops).")
    p.add_argument("--scenario", default="default", help="which bins (config/scenarios/)")
    p.add_argument("--bins", type=int, help="keep the first N corners' bins")
    p.add_argument("--layout-json", help="use this resolved layout (route-check passes the "
                   "running Gazebo world's)")
    p.add_argument("--laps", type=int)
    p.add_argument("--dwell", type=float, help="seconds at each bin")
    p.add_argument("--no-bins", action="store_true", help="drive the square without bin stops")
    p.add_argument("--feedback", help="/odometry/filtered (default) or /wheel/odom")
    p.add_argument("--coast-time", type=float, help="start both coasts at this (s); default "
                   "route.yaml coast")
    p.add_argument("--camera-url", help="photo at each bin stop, e.g. http://IP/capture")
    p.add_argument("--run-dir", help="write the log here instead of results/route/<run>/")
    p.add_argument("--config", default=ROUTE_CONFIG)
    p.add_argument("--no-measure", action="store_true",
                   help="real robot: do not ask for the tape-measured end position")
    return p.parse_args(argv)


def main(argv=None):
    a = parse_args(argv)
    cfg = load_config(a.config)
    for key, val in (("laps", a.laps), ("dwell_s", a.dwell), ("feedback_topic", a.feedback),
                     ("camera_url", a.camera_url)):
        if val is not None:
            cfg[key] = val
    if a.no_bins:
        cfg["face_bins"] = False
    if a.layout_json:
        with open(a.layout_json) as f:
            layout = json.load(f)
    else:
        layout = park.load_park(a.scenario, bins=a.bins, obstacle="none")
    if layout.get("obstacle"):
        raise SystemExit("This layout has an obstacle on the route; avoiding it is Phase 5. "
                         "Use a scenario without one (or OBSTACLE=none in Gazebo).")
    steps = rg.build_plan(layout, cfg["laps"], cfg["dwell_s"], cfg["face_bins"])

    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    results_dir = os.path.join(APP_DIR, cfg["results_dir"])
    run_dir = a.run_dir or os.path.join(results_dir, run_id)
    os.makedirs(run_dir, exist_ok=True)

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = make_follower_class()(cfg, layout, run_dir)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    started, status, reason = utc_now(), "aborted", None
    log_thread = None
    try:
        node.wait_for_odom()
        # Mover zeroes its coast in Gazebo (module 6's robot stopped instantly); walle-sim's
        # firmware models spin down like the robot, so the follower keeps its own coasts.
        if a.coast_time is not None:
            node.coast_side = node.coast_turn = a.coast_time
        node.anchor()
        node.state["run_id"] = run_id
        bins = [b["name"] for b in layout["bins"]] if cfg["face_bins"] else []
        print(f"Run {run_id}: {cfg['laps']} lap(s), bins {', '.join(bins) or 'none'}, "
              f"dwell {cfg['dwell_s']:.0f} s, steering by {cfg['feedback_topic']}, "
              f"coast {node.coast_side:.2f} s (sides) / {node.coast_turn:.2f} s (turns), "
              f"{'Gazebo' if node.in_sim else 'REAL ROBOT'}."
              "\nCtrl+C stops the robot.")
        node.logging = True
        log_thread = threading.Thread(target=node.logger, daemon=True)
        log_thread.start()
        hold = cfg["zero_velocity"]["start_hold_s"]
        if hold > 0:
            print(node.fmt(node.report(state="calibrating", step=-1, corner=None))
                  + f"  standing still {hold:.0f} s to measure the gyro drift")
            node.hold_still(hold)
            print(f"  gyro drift at rest: {math.degrees(node.bias.rate) * 60:+.2f} deg/min")
        node.run(steps)
        status = "done"
    except KeyboardInterrupt:
        reason = "stopped with Ctrl+C"
        print("\nInterrupted.")
    except Exception as e:            # a timeout or lost feedback: stop, record, report
        reason = str(e)
        print(f"\nRun aborted: {e}")
    finally:
        node.stop()
        print("Robot stopped.")
        time.sleep(0.5)
        node.logging = False
        if log_thread:
            log_thread.join(timeout=2.0)
        summary = {"run_id": run_id, "scenario": layout["scenario"],
                   "laps": cfg["laps"], "bins": [b["name"] for b in layout["bins"]]
                   if cfg["face_bins"] else [], "config": cfg,
                   "coast_end_s": {"side": round(node.coast_side, 3),
                                   "turn": round(node.coast_turn, 3)},
                   "corrections": node.corrections,
                   "zero_velocity_heading_deg": round(node.zero_velocity_deg, 3),
                   "gyro_bias_dps": round(math.degrees(node.bias.rate), 4),
                   "holds": node.holds}
        if node.anchors:
            summary.update(end_report(node, started, status, reason))
        else:
            summary.update({"started_utc": started, "status": status, "reason": reason,
                            "source": "unknown", "distance_m": 0.0, "end_errors": {}})
        summary["events"] = node.events
        node.report(state=status)
        in_sim = node.in_sim
        ex.shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()

    print_summary(summary)
    if not in_sim and not a.no_measure and sys.stdin.isatty() and node.anchors:
        try:
            summary["measured"] = ask_measurements(summary)
        except (EOFError, KeyboardInterrupt):
            print("\n(measurement skipped)")
    with open(os.path.join(run_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    with open(os.path.join(run_dir, "layout.json"), "w") as f:
        json.dump(layout, f, indent=2)
    if not in_sim and node.anchors:
        print(f"Recorded in {append_robot_row(results_dir, run_id, summary)}")
    print(f"Run log: {run_dir}")
    return 0 if status == "done" else 1


def print_summary(s):
    print(f"\nRun {s['run_id']}: {s['status']}" + (f" ({s['reason']})" if s.get("reason") else "")
          + f", drove {s['distance_m']:.2f} m")
    for ev in s["events"]:
        if ev["event"] == "arrived":
            extra = (f"  true error {ev['true_error_m'] * 100:.1f} cm"
                     if "true_error_m" in ev else "")
            print(f"  corner {ev['corner']}: est {ev['poses'].get('odom')}{extra}")
        else:
            extra = (f", true bearing to bin {ev['true_bearing_error_deg']:+.1f} deg"
                     if "true_bearing_error_deg" in ev else "")
            print(f"  {ev['bin']}: heading error (est) {ev['est_heading_error_deg']:+.1f} deg"
                  + extra)
    labels = {"odom": "fused EKF", "wheel": "wheels only", "ground_truth": "Gazebo truth"}
    for key, e in s.get("end_errors", {}).items():
        print(f"  end vs start, {labels[key]:12s}: {e['position_m'] * 100:5.1f} cm "
              f"(dx {e['dx_m'] * 100:+.1f}, dy {e['dy_m'] * 100:+.1f}), "
              f"heading {e['heading_deg']:+.1f} deg")


if __name__ == "__main__":
    sys.exit(main())
