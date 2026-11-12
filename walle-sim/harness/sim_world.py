"""Gazebo-side helpers for the harness: ground truth, and moving things in the world.

This is the simulator's stand-in for the measurer's steel tape and protractor (draft
IV-D) and for the operator's hands. It only ever reads Gazebo's true state
(/sim/ground_truth) and moves models with Gazebo's set_pose service; it never reads a
robot estimate, so the ground truth stays independent of what is being tested.
"""

import json
import math
import os
import random
import re
import subprocess
import threading
import time
import urllib.request

import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD_INFO = os.path.join(SIM_DIR, "worlds", "world_info.yaml")
ROBOT = "walle"
REST_Z = 0.031            # base_link height with the wheels on the floor (+1 mm)
WHEEL_HALF_TRACK = 0.0725 + 0.013   # wheel center offset + half the tire width (model)
CAMERA_STATUS_URL = "http://localhost:8080/status"


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def load_world_info():
    with open(WORLD_INFO) as f:
        return yaml.safe_load(f)


class GroundTruth(Node):
    """Latest true pose of every model, the robot's unwrapped heading, and the RTF."""

    def __init__(self):
        super().__init__("sim_ground_truth")
        self.lock = threading.Lock()
        self.poses = {}               # name -> (x, y, z, yaw)
        self.unwrapped = 0.0          # robot heading, continuous (counts full turns)
        self.last_yaw = None
        self.track = None             # samples while a trial runs
        self.sim_time = None
        self.clock_samples = []       # (wall, sim)
        self.profile = None
        self.create_subscription(TFMessage, "/sim/ground_truth", self.on_poses, 10)
        self.create_subscription(Clock, "/clock", self.on_clock, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/sim/profile", self.on_profile, latched)

    def on_poses(self, msg):
        with self.lock:
            for t in msg.transforms:
                name = t.child_frame_id.split("::")[-1]
                # Link and visual entries share short names ("link"): keep model names only
                if name in ("link", "visual", "collision", "base_link"):
                    continue
                p, q = t.transform.translation, t.transform.rotation
                pose = (p.x, p.y, p.z, yaw_of(q))
                self.poses[name] = pose
                if name == ROBOT:
                    if self.last_yaw is not None:
                        self.unwrapped += wrap(pose[3] - self.last_yaw)
                    self.last_yaw = pose[3]
                    if self.track is not None:
                        self.track.append((pose[0], pose[1], self.unwrapped))

    def on_clock(self, msg):
        with self.lock:
            self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
            now = time.monotonic()
            if not self.clock_samples or now - self.clock_samples[-1][0] > 0.1:
                self.clock_samples.append((now, self.sim_time))
                self.clock_samples = self.clock_samples[-600:]

    def on_profile(self, msg):
        with self.lock:
            self.profile = json.loads(msg.data)

    # ----- reading -----
    def pose(self, name=ROBOT):
        with self.lock:
            return self.poses.get(name)

    def heading_unwrapped(self):
        with self.lock:
            return self.unwrapped

    def start_track(self):
        with self.lock:
            self.track = []

    def stop_track(self):
        with self.lock:
            track, self.track = self.track or [], None
        return track

    def rtf(self, seconds=3.0):
        """Real-time factor over the last `seconds` of wall time, or None."""
        with self.lock:
            samples = list(self.clock_samples)
        if len(samples) < 2:
            return None
        end = samples[-1]
        start = next((s for s in samples if end[0] - s[0] <= seconds), samples[0])
        if end[0] - start[0] < 0.5:
            return None
        return (end[1] - start[1]) / (end[0] - start[0])

    def wait_ready(self, names=(ROBOT,), timeout=20.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if all(self.pose(n) is not None for n in names):
                return True
            time.sleep(0.1)
        return False


class World:
    """The running Gazebo world: its layout (world_info.yaml) and set_pose."""

    def __init__(self, gt, name=None):
        self.gt = gt
        self.name = name or find_world()
        info = load_world_info()
        if self.name not in info["worlds"]:
            raise SystemExit(f"World '{self.name}' is not in worlds/world_info.yaml.")
        self.info = info["worlds"][self.name]
        self.objects = info["objects"]
        self.sensor_x = info["sensor_x"]

    def anchor(self, key):
        if key not in self.info.get("anchors", {}):
            raise SystemExit(f"World '{self.name}' has no {key.replace('_', ' ')}. Launch "
                             f"a world that has one, e.g. "
                             f"{'motion_area' if key in ('lane_start', 'protractor') else 'rq3_ranging'}.")
        return self.info["anchors"][key]

    def has(self, key):
        return key in self.info.get("anchors", {})

    # ----- moving models -----
    def set_pose(self, model, x, y, z, yaw=0.0):
        req = (f'name: "{model}", position: {{x: {x:.5f}, y: {y:.5f}, z: {z:.5f}}}, '
               f"orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2):.6f}, w: {math.cos(yaw / 2):.6f}}}")
        cmd = ["ign", "service", "-s", f"/world/{self.name}/set_pose",
               "--reqtype", "ignition.msgs.Pose", "--reptype", "ignition.msgs.Boolean",
               "--timeout", "3000", "--req", req]
        for _ in range(3):
            out = subprocess.run(cmd, capture_output=True, text=True)
            if "true" in out.stdout:
                return True
            time.sleep(0.3)
        raise SystemExit(f"Gazebo did not move {model}: {out.stdout.strip()} {out.stderr.strip()}")

    def place_robot(self, anchor_key, placement=None):
        """The operator's job: put the axle pin on the mark, pointer on the line. With a
        small random alignment error from the profile, like a real placement."""
        x, y, yaw_deg = self.anchor(anchor_key)
        placement = placement or {"position_std_m": 0.0, "yaw_std_deg": 0.0}
        x += random.gauss(0.0, placement["position_std_m"])
        y += random.gauss(0.0, placement["position_std_m"])
        yaw = math.radians(yaw_deg + random.gauss(0.0, placement["yaw_std_deg"]))
        self.set_pose(ROBOT, x, y, REST_Z, yaw)

    def object_height(self, name):
        o = self.objects[name]
        return o["height"] / 2 if o["shape"] == "cylinder" else o["radius"]

    def park_objects(self):
        for name, (x, y) in self.info.get("storage", {}).items():
            pose = self.gt.pose(name)
            if pose is None or math.hypot(pose[0] - x, pose[1] - y) > 0.01:
                self.set_pose(name, x, y, self.object_height(name))

    def sensor_origin(self):
        x, y, _, yaw = self.gt.pose(ROBOT)
        return x + self.sensor_x * math.cos(yaw), y + self.sensor_x * math.sin(yaw), yaw

    def place_object(self, name, distance, angle_deg):
        """Nearest surface `distance` m from the sensor reference point, on the ray at
        `angle_deg` (+ = left) from the robot's TRUE heading. Returns the commanded center."""
        sx, sy, yaw = self.sensor_origin()
        a = yaw + math.radians(angle_deg)
        r = distance + self.objects[name]["radius"]
        cx, cy = sx + r * math.cos(a), sy + r * math.sin(a)
        self.set_pose(name, cx, cy, self.object_height(name))
        return cx, cy

    def true_range(self, name, commanded=None):
        """Horizontal distance from the sensor reference point to the object's nearest
        surface (draft IV-D), from Gazebo's poses. ("gt" or "commanded", meters)."""
        sx, sy, _ = self.sensor_origin()
        pose = self.gt.pose(name)
        source = "gt"
        if commanded is not None and (pose is None or math.hypot(pose[0] - commanded[0],
                                                                 pose[1] - commanded[1]) > 0.005):
            # Gazebo did not (yet) report the moved static model: fall back, and say so
            pose, source = (commanded[0], commanded[1]), "commanded"
        return math.hypot(pose[0] - sx, pose[1] - sy) - self.objects[name]["radius"], source


def find_world():
    out = subprocess.run(["ign", "service", "-l"], capture_output=True, text=True).stdout
    names = sorted(set(re.findall(r"^/world/([^/]+)/set_pose$", out, re.M)))
    if len(names) != 1:
        raise SystemExit("Could not find exactly one running Gazebo world "
                         f"(found {names or 'none'}). Start launch/sim.launch.py first.")
    return names[0]


def camera_frames():
    """Frames the simulated ESP32-CAM has received so far, or None if it is not up."""
    try:
        with urllib.request.urlopen(CAMERA_STATUS_URL, timeout=2.0) as reply:
            return json.loads(reply.read())["frames"]
    except OSError:
        return None


def wait_new_frames(count=2, timeout=10.0):
    start = camera_frames()
    if start is None:
        return False
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        now = camera_frames()
        if now is not None and now >= start + count:
            return True
        time.sleep(0.05)
    return False


def lane_exit(track, start, lane):
    """True if a wheel's outer edge crossed a lane edge line (or the far end) during the run.
    `start` is the lane start anchor (x, y, yaw_deg); track: [(x, y, unwrapped yaw)]."""
    ox, oy, oyaw = start[0], start[1], math.radians(start[2])
    c, s = math.cos(oyaw), math.sin(oyaw)
    far_end = lane["length"] - lane["back"]
    for x, y, heading in track:
        dx, dy = x - ox, y - oy
        lx, ly = dx * c + dy * s, -dx * s + dy * c
        spread = WHEEL_HALF_TRACK * abs(math.cos(heading - oyaw))
        if abs(ly) + spread > lane["half_width"] or lx > far_end:
            return True
    return False
