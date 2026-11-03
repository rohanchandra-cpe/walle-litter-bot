"""One ROS node that talks to the robot for the pilot scripts.

It sends drive commands, resets both pose estimates, and keeps the latest:
    /pico/wheel_ticks   [left, right, ping_seq, echo_us] and when each arrived
    /wheel/odom         wheel-only pose (x, y, yaw) and speeds
    /odometry/filtered  EKF pose (x, y, yaw) and speeds
    /pico/imu/data      whether the IMU reports a calibrated gyroscope
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty, Int32MultiArray

# The firmware reports 10x the orientation SD (100x the variance) while the
# gyroscope is not calibrated, so a variance above this means "not calibrated".
CALIBRATED_ORIENTATION_VARIANCE = 10.0 * math.radians(2.0) ** 2


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def local_change(start, end):
    """Motion from start to end pose, in the start pose's frame: (forward m, left m, turn deg)."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    c, s = math.cos(start[2]), math.sin(start[2])
    turn = math.degrees(math.atan2(math.sin(end[2] - start[2]), math.cos(end[2] - start[2])))
    return dx * c + dy * s, -dx * s + dy * c, turn


class RobotIO(Node):
    def __init__(self, name="pilot"):
        super().__init__(name)
        self.lock = threading.Lock()
        self.ticks = None             # (left, right)
        self.ticks_changed_at = 0.0   # monotonic time the counts last changed
        self.tick_times = []          # arrival times, while recording
        self.recording_ticks = False
        self.ping = None              # (seq, echo_us)
        self.pings = {}               # seq -> echo_us, while collecting
        self.collecting_pings = False
        self.poses = {"wheel": None, "ekf": None}
        self.speeds = {"wheel": (0.0, 0.0), "ekf": (0.0, 0.0)}
        self.imu_calibrated = None
        self.counts = {"ticks": 0, "wheel": 0, "ekf": 0, "imu": 0}

        self.cmd = self.create_publisher(Twist, "/pico/cmd_vel", 10)
        self.odom_reset = self.create_publisher(Empty, "/odom_reset", 10)
        self.set_pose = self.create_publisher(PoseWithCovarianceStamped, "/set_pose", 10)
        self.create_subscription(Int32MultiArray, "/pico/wheel_ticks", self._on_ticks,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, "/wheel/odom",
                                 lambda m: self._on_odom("wheel", m), 10)
        self.create_subscription(Odometry, "/odometry/filtered",
                                 lambda m: self._on_odom("ekf", m), 10)
        self.create_subscription(Imu, "/pico/imu/data", self._on_imu, qos_profile_sensor_data)

    # ----- incoming data -----
    def _on_ticks(self, msg):
        now = time.monotonic()
        with self.lock:
            self.counts["ticks"] += 1
            counts = (msg.data[0], msg.data[1]) if len(msg.data) >= 2 else None
            if counts is not None and counts != self.ticks:
                self.ticks_changed_at = now
            self.ticks = counts
            if self.recording_ticks:
                self.tick_times.append(now)
            if len(msg.data) >= 4:
                self.ping = (msg.data[2], msg.data[3])
                if self.collecting_pings:
                    self.pings.setdefault(msg.data[2], msg.data[3])

    def _on_odom(self, key, msg):
        p = msg.pose.pose
        with self.lock:
            self.counts[key] += 1
            self.poses[key] = (p.position.x, p.position.y, yaw(p.orientation))
            self.speeds[key] = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def _on_imu(self, msg):
        with self.lock:
            self.counts["imu"] += 1
            self.imu_calibrated = msg.orientation_covariance[0] <= CALIBRATED_ORIENTATION_VARIANCE

    def pose(self, key):
        with self.lock:
            return self.poses[key]

    def speed(self, key):
        with self.lock:
            return self.speeds[key]

    def latest_ticks(self):
        with self.lock:
            return self.ticks

    def seconds_since_wheels_moved(self):
        with self.lock:
            return time.monotonic() - self.ticks_changed_at

    # ----- outgoing commands -----
    def send(self, v, w):
        m = Twist()
        m.linear.x, m.angular.z = float(v), float(w)
        self.cmd.publish(m)

    def stop(self):
        for _ in range(5):
            self.send(0.0, 0.0)
            time.sleep(0.05)

    def reset_estimates(self):
        """Zero the wheel odometry and the EKF (robot_localization's /set_pose topic)."""
        self.odom_reset.publish(Empty())
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "odom"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.orientation.w = 1.0
        for i in (0, 7, 14, 21, 28, 35):
            pose.pose.covariance[i] = 1e-6
        self.set_pose.publish(pose)

    # ----- tick timing (network gaps) -----
    def start_tick_recording(self):
        with self.lock:
            self.tick_times = []
            self.recording_ticks = True

    def stop_tick_recording(self):
        """(messages received, largest gap between them in ms) since start_tick_recording."""
        with self.lock:
            self.recording_ticks = False
            times = list(self.tick_times)
        if len(times) < 2:
            return len(times), float("nan")
        gaps = [b - a for a, b in zip(times, times[1:])]
        return len(times), 1000.0 * max(gaps)

    # ----- ultrasonic pings -----
    def collect_pings(self, count, timeout):
        """The next `count` pings that finish after this call: a list of (seq, echo_us),
        with echo_us None for a ping whose message was lost. Fewer if it times out."""
        with self.lock:
            start_seq = self.ping[0] if self.ping else None
            self.pings = {}
            self.collecting_pings = True
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self.lock:
                if start_seq is None and self.ping:
                    start_seq = self.ping[0]
                    self.pings = {}
                newer = sorted(s for s in self.pings if start_seq is not None and s > start_seq)
                if newer and newer[-1] - start_seq >= count:
                    break
            time.sleep(0.01)
        with self.lock:
            self.collecting_pings = False
            got = dict(self.pings)
        if start_seq is None:
            return []
        wanted = range(start_seq + 1, start_seq + 1 + count)
        seen_last = max(got) if got else start_seq
        return [(s, got.get(s)) for s in wanted if s <= seen_last]

    def rates(self, seconds=3.0):
        """Messages per second of each stream, measured over `seconds`."""
        with self.lock:
            before = dict(self.counts)
        time.sleep(seconds)
        with self.lock:
            after = dict(self.counts)
        return {k: (after[k] - before[k]) / seconds for k in after}

    # ----- readiness -----
    def wait_for(self, keys=("ticks", "wheel", "ekf"), timeout=10.0):
        """Wait until each named stream has sent something; returns the missing ones."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            missing = self.missing(keys)
            if not missing:
                return []
            time.sleep(0.1)
        return self.missing(keys)

    def missing(self, keys):
        with self.lock:
            have = {"ticks": self.ticks is not None, "wheel": self.poses["wheel"] is not None,
                    "ekf": self.poses["ekf"] is not None, "imu": self.counts["imu"] > 0,
                    "ping": self.ping is not None}
        return [k for k in keys if not have[k]]


class Robot:
    """Starts ROS and a RobotIO node spinning in the background: `with Robot() as robot:`."""

    def __enter__(self):
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = RobotIO()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        return self.node

    def __exit__(self, *exc):
        try:
            self.node.stop()
        finally:
            self.executor.shutdown()
            self.thread.join(timeout=2.0)
            self.node.destroy_node()
            rclpy.try_shutdown()
        return False
