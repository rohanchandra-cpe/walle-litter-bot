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
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Int32MultiArray, String

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
            # Small heading corrections are timed pulses; their gain is learned too
            self.pulse_gain = cfg["turn"]["pulse"]["gain_dps"]
            self.pulses = []
            self.photo_log = {}              # photo name -> {"tries": n, "error": str|None}
            # Phase 3: YOLO-World (detect/detector.py), set by main() when detection is on
            self.detector = None
            self.detect_cfg = None
            self.detections = []             # one entry per bin-stop photo -> detections.json
            self.truth_bins = {}             # Gazebo: where the bins really are (--truth-layout)
            self.turn_scale = None           # centering: learned at the first bin (centering_cfg)
            # Phase 4: the camera tilt servo (servo/tilt.py) and the bin checks
            self.tilt = None                 # Tilt, set by main() when the servo answers
            self.bin_check_on = False
            self.bin_checks = []             # one record per bin stop -> bin_checks.json
            self.last_level = None           # (analysis, photo) of the last level look
            self.tilt_guard = []             # motion refused because the camera was tilted
            self.last_echo_us = None         # HC-SR04, /pico/wheel_ticks [3] (diagnostic)
            # Sensor guard: when each Pico stream last arrived, the gaps seen, the waits made
            self.guard = cfg.get("sensor_guard", {"enabled": False})
            self.last_seen = {}              # topic -> monotonic time of the last message
            self.gaps = []                   # (start, end, topic) of every gap > max_age_s
            self.guard_waits = []
            self.waited_s = 0.0              # total time spent waiting for sensor data
            self.last_good = None            # (raw odom, raw wheel, t) at the last fresh tick
            self.gap_start = None            # last_good when the first unrepaired gap began
            self.repairs = []
            # Pico restarts and estimate jumps (robot run 20261122_142402)
            self.last_counts = None          # [left, right, ping_seq] of the last ticks
            self.restarts = []               # monotonic times of detected Pico restarts
            self.restart_pending = False     # a restart not yet recovered from
            self.jumps = []                  # estimate jumps caught (summary.json)
            self.jump_ref = None             # (fused park pose, wheel pose, raw odom, t)
            self.t_start = time.monotonic()
            self.create_subscription(Int32MultiArray, "/pico/wheel_ticks", self._on_ticks,
                                     qos_profile_sensor_data)
            self.create_subscription(Imu, "/pico/imu/data", lambda m: self._seen("/pico/imu/data"),
                                     qos_profile_sensor_data)

        def _on_ticks(self, msg):
            if len(msg.data) >= 4:
                self.last_echo_us = int(msg.data[3])
            if len(msg.data) >= 2:
                counts = tuple(int(v) for v in msg.data[:3])
                if rg.pico_restarted(self.last_counts, counts,
                                     int(self.guard.get("restart_counts", 20000))):
                    self._on_restart()
                self.last_counts = counts
            self._seen("/pico/wheel_ticks")

        def _on_restart(self):
            """The Pico restarted: everything it measured since is suspect until the data
            settles. Treated as a sensor gap (no learning across it, repair from the wheels,
            which the wheel odometry carries across the count jump)."""
            now = time.monotonic()
            with self.lock:
                self.restarts.append(now)
                self.gaps.append((now, now, "pico_restart"))
                self.restart_pending = True
                if self.gap_start is None and self.last_good is not None:
                    self.gap_start = self.last_good
            print(f"  PICO RESTARTED ({now - self.t_start:.1f} s into the run): stopping "
                  "until its data settles")

        # ----- sensor guard -----
        def _seen(self, topic):
            now = time.monotonic()
            odom, wheel = self.pose("odom"), self.pose("wheel")
            with self.lock:
                last = self.last_seen.get(topic)
                gap = last is not None and now - last > float(self.guard.get("max_age_s", 0.3))
                if gap:
                    self.gaps.append((last, now, topic))
                    if self.gap_start is None and self.last_good is not None:
                        self.gap_start = self.last_good
                elif topic == "/pico/wheel_ticks" and odom is not None and wheel is not None:
                    self.last_good = (odom, wheel, now)
                self.last_seen[topic] = now

        def repair_estimate(self):
            """After a sensor gap: put the fused estimate back at the pose it had when the
            data stopped, moved by what the wheels measured across the gap (bridge_gap)."""
            with self.lock:
                start, self.gap_start = self.gap_start, None
            if start is None or "odom" not in self.anchors or self.stale_topics():
                return
            odom0, wheel0, t0 = start
            self.jump_ref = None
            anchor = self.anchors["odom"]
            before = anchor.to_park(odom0, t0)
            now_est = self.park_pose()
            target = rg.bridge_gap(before, wheel0, self.pose("wheel"))
            t = time.monotonic()
            self.anchors["odom"] = rg.Anchor(self.pose("odom"), target, t, anchor.heading_rate)
            jump = (math.hypot(now_est[0] - target[0], now_est[1] - target[1]) * 100,
                    math.degrees(rg.wrap(now_est[2] - target[2])))
            self.repairs.append({"moved_back_cm": round(jump[0], 1),
                                 "turned_back_deg": round(jump[1], 1),
                                 "state": self.state.get("state"),
                                 "corner": self.state.get("corner")})
            print(f"  estimate repaired after the gap: {jump[0]:.1f} cm, {jump[1]:+.1f} deg "
                  "of motion that did not happen removed")

        def wait_settled(self, settle_s):
            """Stop; wait until every Pico stream has been fresh for settle_s."""
            Mover.stop(self)
            t0, fresh_since = time.monotonic(), None
            while True:
                now = time.monotonic()
                if now - t0 > float(self.guard.get("give_up_s", 20.0)) + settle_s:
                    raise RuntimeError("the Pico's data did not come back after a restart")
                if self.stale_topics() or self.pose("odom") is None:
                    fresh_since = None
                elif fresh_since is None:
                    fresh_since = now
                elif now - fresh_since >= settle_s:
                    return round(now - t0, 2)
                time.sleep(0.05)

        def recover_restart(self):
            """After a Pico restart while running: wait for its data to settle, then put the
            estimate back from the wheels (repair_estimate)."""
            waited = self.wait_settled(float(self.guard.get("restart_settle_s", 2.0)))
            with self.lock:
                self.restart_pending = False
            self.guard_waits.append({"stale": ["pico_restart"], "seconds": waited,
                                     "state": self.state.get("state"),
                                     "corner": self.state.get("corner")})
            print(f"  Pico data settled after {waited:.1f} s, carrying on")
            self.repair_estimate()

        def check_jump(self):
            """Estimate-jump guard: every window_s while moving, compare the fused motion
            with the wheels' (rg.estimate_jump). On a jump: stop, and repair the estimate
            from the wheels over that window. Returns True if it stopped."""
            jc = self.guard.get("jump") or {}
            if not (self.guard.get("enabled") and jc.get("enabled")) or "odom" not in self.anchors:
                return False
            now, win = time.monotonic(), float(jc.get("window_s", 0.25))
            f, w, raw = self.park_pose(), self.pose("wheel"), self.pose("odom")
            if f is None or w is None:
                return False
            ref = self.jump_ref
            if ref is None or now - ref[3] > 3 * win:     # idle in between: start afresh
                self.jump_ref = (f, w, raw, now)
                return False
            if now - ref[3] < win:
                return False
            extra_m, extra_deg, jump = rg.estimate_jump(
                ref[0], f, ref[1], w, float(jc.get("max_extra_m", 0.05)),
                float(jc.get("max_extra_deg", 20.0)))
            if not jump:
                self.jump_ref = (f, w, raw, now)
                return False
            Mover.send(self, 0.0, 0.0)
            self.jumps.append({"t": round(now - self.t_start, 2),
                               "extra_cm": round(extra_m * 100, 1),
                               "extra_deg": round(extra_deg, 1),
                               "state": self.state.get("state"),
                               "corner": self.state.get("corner")})
            print(f"  ESTIMATE JUMP: {extra_m * 100:.0f} cm / {extra_deg:.0f} deg more than "
                  "the wheels moved: stopping, repairing it from the wheels")
            with self.lock:
                self.gaps.append((ref[3], now, "estimate_jump"))
                self.gap_start = (ref[2], ref[1], ref[3])
            self.repair_estimate()
            return True

        def stale_topics(self):
            """Pico streams older than max_age_s (only streams seen at least once)."""
            if not self.guard.get("enabled"):
                return []
            now, age = time.monotonic(), float(self.guard.get("max_age_s", 0.3))
            with self.lock:
                return [t for t, last in self.last_seen.items() if now - last > age]

        def gaps_since(self, t0):
            """Sensor gaps overlapping [t0, now], the one still open included."""
            now = time.monotonic()
            with self.lock:
                done = rg.gaps_in(self.gaps, t0, now)
            return done + [(now, now, t) for t in self.stale_topics()]

        def wait_for_sensors(self, stale):
            """Stop and wait until every Pico stream has been fresh for resume_after_s."""
            t0 = time.monotonic()
            Mover.stop(self)
            print(f"  sensor gap ({', '.join(stale)}): stopped, waiting for data")
            g = self.guard
            fresh_since = None
            while True:
                now = time.monotonic()
                if now - t0 > float(g.get("give_up_s", 20.0)):
                    self.guard_waits.append({"stale": stale, "seconds": round(now - t0, 2),
                                             "gave_up": True})
                    raise RuntimeError(f"no data from {', '.join(stale)} for "
                                       f"{g.get('give_up_s', 20.0)} s")
                if self.stale_topics():
                    fresh_since = None
                elif fresh_since is None:
                    fresh_since = now
                elif now - fresh_since >= float(g.get("resume_after_s", 0.6)):
                    break
                time.sleep(0.05)
            self.guard_waits.append({"stale": stale, "seconds": round(time.monotonic() - t0, 2),
                                     "state": self.state.get("state"),
                                     "corner": self.state.get("corner")})
            self.waited_s += time.monotonic() - t0
            print(f"  sensor data back after {time.monotonic() - t0:.1f} s, carrying on")
            self.repair_estimate()

        def send(self, v, w):
            """Mover.send, except that a motion command while a Pico stream is stale stops
            the robot and waits for the data instead (stop commands always go out)."""
            if (v or w) and self.restart_pending:
                Mover.send(self, 0.0, 0.0)
                self.recover_restart()
                return                       # the caller's loop re-reads the pose
            if (v or w) and self.check_jump():
                return
            if (v or w) and self.tilt is not None and not self.tilt.is_level():
                # never drive with the camera tilted (Phase 4): stop, level it, and let the
                # caller's loop re-read the pose
                Mover.send(self, 0.0, 0.0)
                rec = {"t": round(time.monotonic(), 2), "readback_us": self.tilt.readback()}
                try:
                    self.tilt.level()
                    rec["leveled"] = True
                except Exception as e:          # noqa: BLE001 (still refuses to move)
                    rec["error"] = str(e)
                    time.sleep(0.5)
                self.tilt_guard.append(rec)
                print(f"  camera was tilted ({rec['readback_us']} us): leveling before moving")
                return
            if v or w:
                stale = self.stale_topics()
                if stale:
                    self.wait_for_sensors(stale)
                    return                   # the caller's loop re-reads the pose
                if self.gap_start is not None:    # a gap ended while the robot stood still
                    self.repair_estimate()
                    return
            Mover.send(self, v, w)

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
            waited0 = self.waited_s
            while True:
                if time.time() > end + (self.waited_s - waited0):
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
            correct_tol_deg with up to max_corrections more moves: a timed pulse for a
            miss up to pulse_max_deg (a slow turn would stick and lurch), else a turn."""
            tc = self.cfg["turn"]
            self.turn_leg(heading, learn=True)
            for _ in range(tc["max_corrections"]):
                err = math.degrees(rg.wrap(heading - self.park_pose()[2]))
                if abs(err) <= tc["correct_tol_deg"]:
                    break
                if abs(err) <= tc["pulse_max_deg"]:
                    if not self.pulse(heading, err):
                        break                    # the shortest pulse would make it worse
                else:
                    self.turn_leg(heading, learn=False)
                self.corrections += 1

        def pulse(self, heading, err_deg):
            """One timed turn pulse toward `heading` (err_deg away). Returns False if no
            pulse was made because even the shortest would overshoot by more than err."""
            pc = self.cfg["turn"]["pulse"]
            dur = rg.plan_pulse(err_deg, self.pulse_gain, pc["dead_s"], pc["min_s"], pc["max_s"])
            if dur is None:
                self.pulses.append({"err_deg": round(err_deg, 2), "skipped": True,
                                    "gain_dps": round(self.pulse_gain, 1)})
                return False
            sign = math.copysign(1.0, err_deg)
            stale = self.stale_topics()
            if stale:                        # don't start a pulse on a stale heading
                self.wait_for_sensors(stale)
                err_deg = math.degrees(rg.wrap(heading - self.park_pose()[2]))
                sign = math.copysign(1.0, err_deg)
                dur = rg.plan_pulse(err_deg, self.pulse_gain, pc["dead_s"], pc["min_s"],
                                    pc["max_s"])
                if dur is None:
                    return False
            h0 = self.park_pose()[2]
            t0 = time.monotonic()
            # one command, then the stop: the Pico holds a command until the next (500 ms
            # watchdog), so the pulse length is the time between the two messages
            self.send(0.0, sign * pc["rate_rps"])
            time.sleep(dur)
            self.stop()
            self.settle()
            gaps = self.gaps_since(t0)
            if gaps:                         # let the estimate catch up before reading it
                self.wait_for_sensors(sorted({g[2] for g in gaps}))
            moved = sign * math.degrees(rg.wrap(self.park_pose()[2] - h0))
            gain0 = self.pulse_gain
            outlier = rg.pulse_outlier(gain0, dur, pc["dead_s"], moved,
                                       float(self.guard.get("pulse_outlier", 2.5)))
            if not gaps and not outlier:
                self.pulse_gain = rg.learn_pulse_gain(self.pulse_gain, dur, moved,
                                                      pc["dead_s"], pc["adapt_gain"])
            after = math.degrees(rg.wrap(heading - self.park_pose()[2]))
            rec = {"err_deg": round(err_deg, 2), "duration_s": round(dur, 3),
                   "moved_deg": round(moved, 2), "err_after_deg": round(after, 2),
                   "gain_dps": round(gain0, 1)}
            if gaps:
                rec["sensor_gap"] = True
            if outlier:
                rec["outlier"] = True
            self.pulses.append(rec)
            return True

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
            waited0 = self.waited_s
            while time.time() < end + (self.waited_s - waited0):
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
            tg = t0
            t_end = time.time() + seconds
            time.sleep(min(1.0, seconds))
            shot = self.photo_with_retries(photo_name, t_end) if photo_name else None
            time.sleep(max(0.0, t_end - time.time()))
            w1 = self.pose("wheel")
            zc = self.cfg["zero_velocity"]
            if not zc["enabled"] or w0 is None or w1 is None:
                return shot, None
            moved = math.hypot(w1[0] - w0[0], w1[1] - w0[1])
            turned = abs(rg.wrap(w1[2] - w0[2]))
            if moved > zc["max_move_m"] or turned > math.radians(zc["max_turn_deg"]):
                return shot, None
            if self.gaps_since(tg):          # the EKF drifted on no data: not gyro bias
                self.holds.append({"seconds": round(time.monotonic() - tg, 2),
                                   "skipped": "sensor gap"})
                self.repair_estimate()
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

        def photo_with_retries(self, name, t_end):
            """Up to camera_tries photo requests, all within the wait (ending at t_end)."""
            if not self.cfg.get("camera_url"):
                return None
            timeout = float(self.cfg.get("camera_timeout_s", 5.0))
            tries = int(self.cfg.get("camera_tries", 1))
            log = self.photo_log[name] = {"tries": 0, "error": None}
            for i in range(tries):
                if i and time.time() + timeout > t_end:
                    break                        # no time left in the wait
                log["tries"] += 1
                shot, error = self.photo(name, timeout)
                if shot:
                    log["error"] = None
                    return shot
                log["error"] = error
                print(f"  (no photo, try {i + 1}/{tries}: {error})")
                time.sleep(min(float(self.cfg.get("camera_retry_gap_s", 3.0)),
                               max(0.0, t_end - time.time() - timeout)))
            return None

        def photo(self, name, timeout=5.0):
            """One photo request. Returns (path relative to the run folder, None) or
            (None, error)."""
            url = self.cfg.get("camera_url")
            import urllib.request
            try:
                with urllib.request.urlopen(url, timeout=timeout) as reply:
                    data = reply.read()
            except OSError as e:
                return None, str(e)
            path = os.path.join(self.run_dir, "photos", f"{name}.jpg")
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "wb") as f:
                f.write(data)
            return os.path.relpath(path, self.run_dir), None

        def run(self, steps):
            bins = {b["name"]: b for b in self.layout["bins"]}
            for i, step in enumerate(steps):
                kind = step[0]
                if kind == "drive":
                    _, c_from, c_to, a, b, heading = step
                    print(self.fmt(self.report(state="driving", step=i, corner=c_to)))
                    self.drive_side(a, b, heading)
                    snap = self.snapshot()
                    ev = {"step": i, "event": "arrived", "t_utc": utc_now(), "corner": c_to,
                          "target": list(b),
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
                    try:
                        centering = self.center_bin(i, bins[name], heading)
                    except Exception as e:      # detection must never abort the drive
                        self.stop()
                        centering = {"centered": False, "reason": f"error: {e}"}
                        print(f"  centering {name}: error, keeping the heading: {e}")
                    ev = self.bin_event(i, corner, bins[name], heading)
                    if centering:
                        ev["centering"] = centering
                    self.events.append(ev)
                elif kind == "dwell":
                    _, corner, name, seconds = step
                    print(self.fmt(self.report(state="at_bin", step=i, corner=corner, bin=name))
                          + f"  waiting {seconds:.0f} s")
                    shot, drift = self.hold_still(seconds, f"{name}_step{i}")
                    if shot:
                        self.events[-1]["photo"] = shot
                    tried = self.photo_log.get(f"{name}_step{i}")
                    if tried:
                        self.events[-1]["photo_tries"] = tried["tries"]
                        if tried["error"]:
                            self.events[-1]["photo_error"] = tried["error"]
                    if drift is not None:
                        self.events[-1]["held_heading_drift_deg"] = drift
                    self.last_level = None
                    if shot and self.detector:
                        try:
                            self.events[-1]["detected"] = self.detect_at_bin(i, bins[name], shot)
                        except Exception as e:
                            self.events[-1]["detect_error"] = str(e)
                            print(f"  detection at {name} failed: {e}")
                    if self.bin_check_on and self.last_level:
                        try:            # a bin check must never abort the drive either
                            self.events[-1]["bin_check"] = self.check_bin(
                                i, bins[name], self.events[-1]["step"])
                        except Exception as e:
                            self.events[-1]["bin_check_error"] = str(e)
                            print(f"  bin check at {name} failed: {e}")
                        finally:
                            self.level_camera()
                    self.state.pop("bin", None)
                elif kind == "turn":
                    _, corner, heading = step
                    print(self.fmt(self.report(state="turning", step=i, corner=corner)))
                    self.turn_to(heading)

        # ----- Phase 3: detection at the bin stops -----
        def camera_settings(self):
            """park.yaml robot.camera; in Gazebo with detect.yaml's sim camera on top."""
            from detect.sim_truth import camera_settings
            if self.detector and self.detector.source == "sim":
                return camera_settings(self.detect_cfg)
            return park.load_yaml(park.PARK_FILE)["robot"]["camera"]

        def centering_cfg(self):
            """detect.yaml centering; in Gazebo with detect.yaml's sim centering on top."""
            cc = dict(self.detect_cfg["centering"])
            if self.detector and self.detector.source == "sim":
                cc.update(self.detect_cfg.get("sim", {}).get("centering", {}))
            return cc

        def camera_model(self, size, tilt_deg=0.0):
            from detect.geometry import Camera
            pivot = self.tilt.cal.lens_from_pivot_m if self.tilt else None
            return Camera.from_park(self.camera_settings(), *size, tilt_deg=tilt_deg,
                                    lens_from_pivot_m=pivot)

        def tilt_deg(self):
            """The camera's tilt now (servo readback), 0 if there is no servo."""
            a = self.tilt.angle_deg() if self.tilt else None
            return a or 0.0

        def expected_bin(self, b):
            """The bin's bearing from the camera axis (deg, left +) on the current estimate,
            and its distance from the camera (m)."""
            x, y, h = self.park_pose()
            cam = self.camera_settings()
            fwd, left = cam["forward_m"], cam.get("left_m", 0.0)
            # from the lens, which is fwd ahead of the axle and left of the center line
            lx = x + fwd * math.cos(h) - left * math.sin(h)
            ly = y + fwd * math.sin(h) + left * math.cos(h)
            dx, dy = b["position"][0] - lx, b["position"][1] - ly
            bearing = math.degrees(rg.wrap(math.atan2(dy, dx) - h))
            return bearing, max(math.hypot(dx, dy), 0.1)

        def analyze(self, shot, b, tilt_deg=0.0, stop=False):
            """Detect in a saved photo and sort what is seen for bin b (the camera tilted
            tilt_deg up when it was taken). stop: a photo of the bin check, after centering
            (geometry.analyze_bin_stop's stop rules). Returns (analysis, detector result, pose
            [x, y, heading_deg]); the analysis carries the Camera it used ("camera")."""
            from detect import geometry as dg
            with open(os.path.join(self.run_dir, shot), "rb") as f:
                res = self.detector.detect(f.read(), keep_low=True)
            cam = self.camera_model(res["size"], tilt_deg)
            bearing, dist = self.expected_bin(b)
            width_m = b["size_m"][0] if b.get("shape") == "box" else 2 * b.get("radius_m", 0.1)
            height_m = b.get("height_m") or (b["size_m"][2] if b.get("size_m") else 0.29)
            p = self.park_pose()
            pose = [round(p[0], 4), round(p[1], 4), round(math.degrees(p[2]), 2)]
            if tilt_deg:
                expected_px = cam.expected_box_px(width_m, dist, 0.0, height_m)
            else:                            # Phase 3's level formula, unchanged
                expected_px = (cam.width_px(width_m, dist), cam.fy * height_m / dist)
            stop_info = None
            if stop:
                stop_info = {"bin_dist_m": dist, "bin_height_m": height_m,
                             "low_conf_bins": res.get("low_conf", [])}
            an = dg.analyze_bin_stop(res["detections"], cam, pose=pose,
                                     expected_bearing_deg=bearing,
                                     expected_bin_px=expected_px,
                                     cfg=self.detect_cfg, stop=stop_info)
            an["expected_bearing_deg"] = round(bearing, 2)
            an["expected_dist_m"] = round(dist, 3)
            an["expected_bin_px"] = expected_px
            an["camera"] = cam
            an["search_order"] = dg.search_order(
                [d for d in res["detections"] + res.get("low_conf", []) if d["kind"] == "bin"],
                cam, self.detect_cfg["centering"].get("edge_px", 4), an["expected_bin_px"],
                self.centering_cfg().get("search_deg", 15.0))
            return an, res, pose

        def look(self, name, b):
            """One photo + detection. Returns (photo path, analysis, detector result) or
            (None, None, error string)."""
            shot, error = self.photo(name, float(self.cfg.get("camera_timeout_s", 5.0)))
            if not shot:
                return None, None, error
            an, res, _ = self.analyze(shot, b)
            return shot, an, res

        def center_bin(self, i, b, heading):
            """Turn until the DETECTED bin is in the middle of the frame (Phase 3): photo,
            detect, pulse (or turn) by the bin's bearing, up to max_corrections times, never
            farther than max_offset_deg from the configured heading. No bin on the first
            look: try search_deg each side of the configured heading (search_order); still
            none: keep the heading and say so. Returns the record for summary.json, or None
            if off."""
            if not (self.detector and self.cfg.get("center_bins", True)):
                return None
            cc = self.centering_cfg()
            if self.turn_scale is None:
                self.turn_scale = float(cc.get("turn_scale", 1.0))
            limits = tuple(cc.get("turn_scale_limits", (0.7, 2.5)))
            forward = self.camera_settings()["forward_m"]
            out = {"centered": False, "looks": [], "turn_scale_start": round(self.turn_scale, 3)}
            prev = None                      # (bearing, heading_deg) of the last look
            searches = None                  # search offsets still to try (set on a miss)
            tried = []
            found_any = False
            j, n = 0, 0                      # corrections made, photos taken
            while True:
                t_look = time.monotonic()
                shot, an, res = self.look(f"{b['name']}_step{i}_c{n}", b)
                n += 1
                if shot is None:
                    out["reason"] = f"no photo: {res}"
                    break
                look = {"photo": shot, "seconds": res["seconds"],
                        "expected_bearing_deg": an["expected_bearing_deg"],
                        "bin_bearing_deg": an["bin_bearing_deg"],
                        "bin_conf": an["bin"]["conf"] if an["bin"] else None,
                        "heading_deg": round(math.degrees(self.park_pose()[2]), 2)}
                out["looks"].append(look)
                err = an["bin_bearing_deg"]
                parallax = rg.parallax_factor(an["expected_dist_m"], forward)
                if err is not None and prev is not None and not self.gaps_since(prev[2]):
                    turned = look["heading_deg"] - prev[1]
                    turned = (turned + 180.0) % 360.0 - 180.0
                    self.turn_scale = rg.learn_turn_scale(
                        self.turn_scale, prev[0], err, turned, parallax,
                        float(cc.get("turn_scale_gain", 0.6)), limits)
                    look["turn_scale"] = round(self.turn_scale, 3)
                prev = (err, look["heading_deg"], t_look) if err is not None else None
                if err is None and not found_any and float(cc.get("search_deg", 0)) > 0:
                    if searches is None:
                        searches = list(an["search_order"])
                    if searches:
                        off = searches.pop(0)
                        tried.append(off)
                        target = rg.wrap(heading + math.radians(off))
                        look["search_deg"] = off
                        move = math.degrees(rg.wrap(target - self.park_pose()[2]))
                        print(f"  centering {b['name']}: no bin, looking {off:+.0f} deg "
                              f"from the configured heading")
                        if abs(move) <= self.cfg["turn"]["pulse_max_deg"]:
                            self.pulse(target, move)
                        else:
                            self.turn_to(target)
                        continue
                    out["reason"] = (f"no bin detected (searched "
                                     f"{', '.join(f'{o:+.0f}' for o in tried)} deg)")
                    break
                if err is not None:
                    found_any = True
                if err is None or abs(err - an["expected_bearing_deg"]) > cc["max_offset_deg"]:
                    out["reason"] = ("no bin detected" if err is None else
                                     f"bin seen {err:+.1f} deg, too far from the expected "
                                     f"{an['expected_bearing_deg']:+.1f}")
                    break
                if abs(err) <= cc["tolerance_deg"]:
                    out["centered"] = True
                    break
                if j == int(cc["max_corrections"]):
                    out["reason"] = f"still {err:+.1f} deg off after {j} corrections"
                    break
                j += 1
                h_now = self.park_pose()[2]
                # the camera sees the bin move parallax x turn_scale x the recorded turn
                turn_for = err / (parallax * self.turn_scale)
                target, clamped = rg.centering_target(h_now, turn_for, heading,
                                                      cc["max_offset_deg"])
                look["clamped"] = clamped
                move = math.degrees(rg.wrap(target - h_now))
                print(f"  centering {b['name']}: bin at {err:+.1f} deg, turning {move:+.1f}")
                if abs(move) <= self.cfg["turn"]["pulse_max_deg"]:
                    if not self.pulse(target, move):
                        out["reason"] = "the shortest pulse would overshoot"
                        break
                else:
                    self.turn_to(target)
            if tried:
                out["searched_deg"] = tried
            out["turn_scale_end"] = round(self.turn_scale, 3)
            out["offset_from_config_deg"] = round(math.degrees(
                rg.wrap(self.park_pose()[2] - heading)), 2)
            last = out["looks"][-1]["bin_bearing_deg"] if out["looks"] else None
            print(f"  centering {b['name']}: "
                  + ("centered" if out["centered"] else f"NOT centered ({out.get('reason')})")
                  + (f", bin at {last:+.1f} deg" if last is not None else "")
                  + f", {out['offset_from_config_deg']:+.1f} deg from the configured heading")
            return out

        def detect_at_bin(self, i, b, shot):
            """Detect items in the bin photo (taken during the wait, after centering) and
            keep them for detections.json. Returns a short count for the event."""
            an, res, pose = self.analyze(shot, b, stop=True)
            self.last_level = (an, shot)
            self.detections.append({
                "step": i, "t_utc": utc_now(), "place_kind": "bin_stop", "look": "level",
                "tilt_deg": 0.0, "bin": b["name"],
                "bin_id": b["id"], "photo": shot, "pose": pose, "model": res["model"],
                "source": res["source"], "seconds": res["seconds"], "size": res["size"],
                "bin_detection": an["bin"], "bin_bearing_deg": an["bin_bearing_deg"],
                "rim_visible": an["rim_visible"], "items": an["items"],
                "ignored": an.get("ignored", []),
                "low_conf": res.get("low_conf", [])})
            items = an["items"]
            print(f"  {b['name']}: " + (", ".join(
                f"{d['label']} ({d['category']}, {d['place']}"
                + (f", {d['conf']:.2f})" if d.get("conf") is not None else ")")
                for d in items) or "no items") + f"  [{res['seconds']:.2f} s]")
            return {"items": len(items), "on_bin": sum(d["place"] == "on_bin" for d in items),
                    "around_bin": sum(d["place"] == "around_bin" for d in items),
                    "bin_found": an["bin"] is not None, "rim_visible": an["rim_visible"]}

        # ----- Phase 4: the bin check (tilt up, judge, tilt back) -----
        def level_camera(self):
            """Put the camera level again (the send() guard retries if this fails)."""
            if self.tilt is None:
                return
            try:
                self.tilt.level()
            except Exception as e:              # noqa: BLE001
                print(f"  could not level the camera: {e}")

        def up_photo(self, name):
            """The up look's photo: up to camera_tries requests."""
            timeout = float(self.cfg.get("camera_timeout_s", 5.0))
            error = None
            for k in range(int(self.cfg.get("camera_tries", 1))):
                shot, error = self.photo(name, timeout)
                if shot:
                    return shot, None
                time.sleep(float(self.cfg.get("camera_retry_gap_s", 3.0)) if k else 1.0)
            return None, error

        def check_bin(self, i, b, facing_step):
            """Judge bin b (detect/bin_check.py): the level look is the dwell photo already
            analysed (detect_at_bin); the up look tilts the camera servo.yaml up_deg, takes
            a photo and analyses it. Without a servo, or if the up look fails, the judge
            uses the level photo alone (method level_only). Returns a short summary."""
            from detect.bin_check import judge_bin
            level_an, level_shot = self.last_level
            up_an, up_shot, up_error, tilt_used = None, None, None, None
            if self.tilt is not None:
                try:
                    tilt_used = self.tilt.set(self.tilt.cal.up_deg)
                    up_shot, up_error = self.up_photo(f"{b['name']}_step{i}_up")
                    if up_shot:
                        up_an, res, pose = self.analyze(up_shot, b, tilt_used, stop=True)
                        self.detections.append({
                            "step": i, "t_utc": utc_now(), "place_kind": "bin_stop",
                            "look": "up", "tilt_deg": round(tilt_used, 2), "bin": b["name"],
                            "bin_id": b["id"], "photo": up_shot, "pose": pose,
                            "model": res["model"], "source": res["source"],
                            "seconds": res["seconds"], "size": res["size"],
                            "bin_detection": up_an["bin"],
                            "bin_bearing_deg": up_an["bin_bearing_deg"],
                            "rim_visible": up_an["rim_visible"], "items": up_an["items"],
                            "ignored": up_an.get("ignored", []),
                            "low_conf": res.get("low_conf", [])})
                except Exception as e:          # noqa: BLE001 (judge on the level look)
                    up_error = str(e)
                finally:
                    self.level_camera()
            rec = judge_bin(level_an, up_an, b, self.detect_cfg,
                            level_cam=level_an["camera"],
                            up_cam=up_an["camera"] if up_an else None,
                            dist_m=level_an["expected_dist_m"],
                            edge_px=self.detect_cfg["centering"].get("edge_px", 4))
            rec.update({"step": i, "facing_step": facing_step, "t_utc": utc_now(),
                        "bin": b["name"], "bin_id": b["id"], "level_photo": level_shot,
                        "up_photo": up_shot, "up_tilt_deg": (None if tilt_used is None
                                                             else round(tilt_used, 2)),
                        "expected_dist_m": level_an["expected_dist_m"],
                        "echo_us": self.last_echo_us})
            if up_error:
                rec["up_error"] = up_error
            if self.in_sim and b["name"] in self.truth_bins:
                from detect.bin_check import expected_from_layout
                rec["truth"] = expected_from_layout(self.truth_bins[b["name"]])
            self.bin_checks.append(rec)
            print(f"  bin check {b['name']} ({rec['method']}): {rec['outcome']}, overflow "
                  f"level {rec['overflow_level']} ({rec['on_top_count']} on top"
                  + (f", top {rec['top_height_above_rim_m'] * 100:.0f} cm over the rim"
                     if rec["top_height_above_rim_m"] is not None else "")
                  + f"), litter {rec['litter_count']}"
                  + (f"  [up look failed: {up_error}]" if up_error else ""))
            return {k: rec[k] for k in ("method", "outcome", "overflow_level", "on_top_count",
                                        "litter_count")}

        def bin_event(self, i, corner, b, heading):
            snap = self.snapshot()
            ev = {"step": i, "event": "facing_bin", "t_utc": utc_now(), "corner": corner,
                  "bin": b["name"],
                  "target_heading_deg": round(math.degrees(heading), 2), "poses": snap}
            est = snap["odom"]
            ev["est_heading_error_deg"] = round(rg.wrap(math.radians(est[2]) - heading)
                                                * 180 / math.pi, 2)
            if "ground_truth" in snap:
                g = snap["ground_truth"]
                pos = self.truth_bins.get(b["name"], b)["position"]
                bearing = math.atan2(pos[1] - g[1], pos[0] - g[0])
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
    p.add_argument("--truth-layout", help="Gazebo: the world's real layout, for the ground-"
                   "truth detector, when --layout-json gives the follower a different one "
                   "(bins moved, to test centering)")
    p.add_argument("--sim-hide-cut-bins", action="store_true",
                   help="Gazebo: the ground-truth detector misses bins cut off at the frame's "
                   "side, as YOLO-World did on the robot (tests the search turn)")
    p.add_argument("--no-detect", action="store_true",
                   help="no YOLO-World at the bin stops (no centering, no detections)")
    p.add_argument("--no-tilt", action="store_true",
                   help="bin check without the camera servo (level photo only)")
    p.add_argument("--no-memory", action="store_true",
                   help="do not record the run in the memory store (walle-app/data/memory/)")
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
        truth = layout
        if a.truth_layout:
            with open(a.truth_layout) as f:
                truth = json.load(f)
            node.truth_bins = {b["name"]: b for b in truth["bins"]}
        if cfg.get("detect", True) and cfg["face_bins"] and cfg.get("camera_url") \
                and not a.no_detect:
            load_detector(node, truth, a.sim_hide_cut_bins)
        if node.detector and cfg.get("bin_check", True):
            setup_bin_check(node, use_tilt=not a.no_tilt)
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
        for attempt in range(3):
            if hold > 0:
                print(node.fmt(node.report(state="calibrating", step=-1, corner=None))
                      + f"  standing still {hold:.0f} s to measure the gyro drift")
                node.hold_still(hold)
                print(f"  gyro drift at rest: {math.degrees(node.bias.rate) * 60:+.2f} deg/min")
            if not node.restart_pending:
                break
            # the Pico restarted during the start (run 20261122_142402): the start pose was
            # taken from data that no longer holds. Wait for it to settle and take it again.
            print("  the Pico restarted during the start: taking the start pose again")
            node.wait_settled(float(node.guard.get("restart_settle_s", 2.0)))
            with node.lock:
                node.restart_pending, node.gap_start = False, None
            node.anchor()
            node.bias = rg.BiasEstimate()
        else:
            raise RuntimeError("the Pico kept restarting during the start")
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
                   "pulse_gain_end_dps": round(node.pulse_gain, 1),
                   "centering_turn_scale_end": (round(node.turn_scale, 3)
                                                if node.turn_scale is not None else None),
                   "pulses": node.pulses,
                   "sensor_gaps": [{"topic": g[2], "seconds": round(g[1] - g[0], 2)}
                                   for g in node.gaps],
                   "sensor_guard_waits": node.guard_waits,
                   "estimate_repairs": node.repairs,
                   "zero_velocity_heading_deg": round(node.zero_velocity_deg, 3),
                   "gyro_bias_dps": round(math.degrees(node.bias.rate), 4),
                   "holds": node.holds,
                   "pico_restarts_s": [round(t - node.t_start, 2) for t in node.restarts],
                   "estimate_jumps": node.jumps,
                   "tilt_moves": node.tilt.log if node.tilt else [],
                   "tilt_guard": node.tilt_guard}
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
    if node.detector:
        with open(os.path.join(run_dir, "detections.json"), "w") as f:
            json.dump({"run_id": run_id, "model": node.detector.model_name,
                       "source": node.detector.source, "photos": node.detections}, f, indent=1)
    if node.bin_check_on:
        with open(os.path.join(run_dir, "bin_checks.json"), "w") as f:
            json.dump({"run_id": run_id, "source": summary.get("source"),
                       "checks": node.bin_checks}, f, indent=1)
    if not in_sim and node.anchors:
        print(f"Recorded in {append_robot_row(results_dir, run_id, summary)}")
    if not a.no_memory and node.anchors:
        record_in_memory(summary, layout, run_dir)
    print(f"Run log: {run_dir}")
    return 0 if status == "done" else 1


def load_detector(node, truth_layout, hide_cut_bins=False):
    """YOLO-World for the bin stops (Phase 3). In Gazebo, where YOLO-World sees nothing, the
    ground-truth stand-in (detect/sim_truth.py) projects truth_layout into the camera. If the
    detector can't be loaded (no ultralytics, no weights), the run goes on without it."""
    try:
        from detect.detector import load_config as load_detect_config, make_detector
        node.detect_cfg = load_detect_config()
        t0 = time.time()
        if node.in_sim:
            from detect.sim_truth import TruthDetector
            from servo.tilt import Calibration
            node.detector = TruthDetector(truth_layout,
                                          lambda: node.park_pose("ground_truth"),
                                          cfg=node.detect_cfg, hide_cut_bins=hide_cut_bins,
                                          tilt_fn=node.tilt_deg,
                                          lens_from_pivot_m=Calibration.load(
                                              sim=True).lens_from_pivot_m)
        else:
            node.detector = make_detector(source="robot", cfg=node.detect_cfg)
        print(f"Detector: {node.detector.model_name} ({node.detector.source}), "
              f"loaded in {time.time() - t0:.1f} s")
    except Exception as e:
        node.detector = None
        print(f"Detector NOT loaded, no centering or detections this run: {e}")


def setup_bin_check(node, use_tilt=True):
    """Phase 4: the bin check at each bin stop. With the servo (its readback arrives on
    /pico/wheel_ticks [4]) it adds the up look; without it the check judges on the level
    photo alone. Leaves the camera level."""
    node.bin_check_on = True
    if not use_tilt:
        print("Bin check: level photo only (--no-tilt)")
        return
    try:
        from servo.tilt import Calibration, Tilt
        tilt = Tilt(node, Calibration.load(sim=node.in_sim))
        t_end = time.monotonic() + 3.0
        while time.monotonic() < t_end and not tilt.has_servo():
            time.sleep(0.1)
        if not tilt.has_servo():
            print("Bin check: NO servo readback on /pico/wheel_ticks (firmware without the "
                  "servo?): level photo only")
            return
        tilt.level()
        node.tilt = tilt
        print(f"Bin check: camera servo ready, up look at {tilt.cal.up_deg:+.0f} deg")
    except Exception as e:                      # noqa: BLE001 (the drive goes on)
        print(f"Bin check: servo not usable ({e}): level photo only")


def record_in_memory(summary, layout, run_dir):
    """Add the run and its bin stops to the memory store (ROADMAP Phase 2). Never fails the
    run: the JSON log above is the record, and import_runs.py can add it later."""
    try:
        from memory.route_records import record_route_summary
        from memory.store import MemoryStore
        with MemoryStore(summary["source"]) as mem:
            record_route_summary(mem, summary, layout, run_dir)
            print(f"Memory: run {summary['run_id']} recorded in {mem.path}")
    except Exception as e:                      # noqa: BLE001 (the run itself is done)
        print(f"Memory: run not recorded ({e}); add it later with "
              "walle-app/memory/import_runs.py")


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
            photo = ("" if "photo_tries" not in ev else
                     (f", photo on try {ev['photo_tries']}" if ev["photo_tries"] > 1 else "")
                     if ev.get("photo") else
                     f", NO photo ({ev.get('photo_error')})")
            print(f"  {ev['bin']}: heading error (est) {ev['est_heading_error_deg']:+.1f} deg"
                  + extra + photo)
    made = [p for p in s.get("pulses", []) if not p.get("skipped")]
    if s.get("pulses"):
        print(f"  correction pulses: {len(made)} made, {len(s['pulses']) - len(made)} skipped, "
              f"gain {s.get('pulse_gain_end_dps')} deg/s at the end; "
              + ", ".join(f"{p['err_deg']:+.1f}->{p['err_after_deg']:+.1f}" for p in made))
    labels = {"odom": "fused EKF", "wheel": "wheels only", "ground_truth": "Gazebo truth"}
    for key, e in s.get("end_errors", {}).items():
        print(f"  end vs start, {labels[key]:12s}: {e['position_m'] * 100:5.1f} cm "
              f"(dx {e['dx_m'] * 100:+.1f}, dy {e['dy_m'] * 100:+.1f}), "
              f"heading {e['heading_deg']:+.1f} deg")


if __name__ == "__main__":
    sys.exit(main())
