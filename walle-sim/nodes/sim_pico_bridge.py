"""The simulated Pico: makes the Gazebo robot look exactly like WALL-E's firmware.

Real robot (firmware/pico_pilot)         Simulation (this node)
--------------------------------         --------------------------------------------
subscribes /pico/cmd_vel            ->   firmware model (firmware_models.py) -> motor
                                         plant -> /sim/cmd_vel (Gazebo DiffDrive)
publishes /pico/wheel_ticks  50 Hz  <-   wheel joint angles (/joint_states) -> counts,
  [left, right, ping_seq, echo_us]       plus the HC-SR04 model on /sim/ultrasonic/points
publishes /pico/imu/data     50 Hz  <-   Gazebo IMU (/sim/imu) -> BNO055 in IMU mode:
                                         heading integrated from the gyro, gravity removed
publishes /pico/drive/status        <-   0 stopped, 1 driving, 2 stopped by the 500 ms
                                         command watchdog
subscribes /pico/imu/std            ->   live covariance tuning, as in the firmware

So pilot.launch.py (odometry + EKF), the pilot scripts, and the ROSA tools run
unchanged against Gazebo.

Clocks: like the real Pico, this node runs on the computer's clock (its 50 Hz loop,
the watchdog, the 60 ms pings, the IMU stamps), and so does the whole robot-side
stack. Only the physics runs on simulation time. Keep the real-time factor near 1
(gui:=false helps); harness/sim_trials.py measures and logs it for every trial. How the hardware behaves (firmware logic, motor mismatch,
IMU drift, ultrasonic failures, Wi-Fi stalls) comes from a profile in
config/sim_profiles.yaml, published as JSON on /sim/profile for the logs.

    python3 sim_pico_bridge.py --ros-args -p profile:=pilot_firmware
"""

import json
import math
import os
import random
import sys
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, PointCloud2
from std_msgs.msg import Float32MultiArray, Int32, Int32MultiArray, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from firmware_models import MotorPlant, make_firmware  # noqa: E402
from profiles import load_profile  # noqa: E402

GRAVITY = 9.80665
LOOP_S = 0.02           # PUBLISH_PERIOD_MS 20
STATUS_STOPPED, STATUS_DRIVING, STATUS_TIMEOUT = 0, 1, 2


def diagonal(variance):
    return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]


def nearest_return(msg, min_range, max_range, floor_z):
    """Distance (m) of the nearest lidar return in the cone, ignoring the floor, or None."""
    fields = {f.name: f.offset for f in msg.fields}
    if not all(k in fields for k in "xyz") or msg.point_step == 0:
        return None
    endian = ">" if msg.is_bigendian else "<"
    dtype = np.dtype({"names": ["x", "y", "z"], "formats": [endian + "f4"] * 3,
                      "offsets": [fields["x"], fields["y"], fields["z"]],
                      "itemsize": msg.point_step})
    count = len(msg.data) // msg.point_step
    if count == 0:
        return None
    pts = np.frombuffer(bytes(msg.data), dtype=dtype, count=count)
    x, y, z = (pts[k].astype(np.float64) for k in ("x", "y", "z"))
    r = np.sqrt(x * x + y * y + z * z)
    ok = np.isfinite(r) & (r >= min_range) & (r < max_range - 1e-3) & (z > floor_z)
    return float(r[ok].min()) if ok.any() else None


class DelayLine:
    """Wi-Fi model: drop messages, or stall and then deliver the backlog at once."""

    def __init__(self, net, publish):
        self.net, self.publish = net, publish
        self.queue = deque()
        self.hold_until = -1.0

    def send(self, now, msg):
        if random.random() < self.net["tick_drop_prob"]:
            return
        if now >= self.hold_until and random.random() < self.net["burst_prob"]:
            self.hold_until = now + self.net["burst_hold_s"]
        self.queue.append(msg)
        if now >= self.hold_until:
            while self.queue:
                self.publish(self.queue.popleft())


class SimPicoBridge(Node):
    def __init__(self):
        super().__init__("sim_pico_bridge")
        self.declare_parameter("profile", "pilot_firmware")
        # The Gazebo model's true geometry (urdf/gazebo_car.urdf.xacro), used only to
        # turn wheel speeds back into the DiffDrive plugin's twist
        self.declare_parameter("model_wheel_separation", 0.145)
        self.declare_parameter("model_max_linear", 0.3)
        self.declare_parameter("model_max_angular", 1.5)
        # Height of the HC-SR04 above the floor, to recognize floor returns
        self.declare_parameter("ultrasonic_height_m", 0.07)
        self.declare_parameter("seed", 0)                # 0 = random
        p = lambda n: self.get_parameter(n).value  # noqa: E731

        self.profile = load_profile(p("profile"))
        if p("seed"):
            random.seed(int(p("seed")))
        fw = self.profile["firmware_params"]
        self.firmware = make_firmware(self.profile["firmware"], fw)
        self.plant = MotorPlant(self.profile["motor"])
        self.model_sep = float(p("model_wheel_separation"))
        self.model_max = (float(p("model_max_linear")), float(p("model_max_angular")))
        self.us = self.profile["ultrasonic"]
        self.imu_p = self.profile["imu"]
        self.floor_z = -(float(p("ultrasonic_height_m")) - self.us["floor_reject_m"])

        self.start_time = None
        self.last_cmd_time = None
        self.status = STATUS_STOPPED
        self.next_status_time = 0.0
        self.counts = (0, 0)
        self.wheel_angle = None
        self.ping_seq, self.echo_us = 0, 0
        self.next_ping = 0.0
        self.nearest = None
        self.yaw = 0.0
        self.last_imu_time = None
        self.std = [self.imu_p["orientation_std_deg"], self.imu_p["gyro_std_rad_s"],
                    self.imu_p["accel_std_m_s2"]]
        self.clamp_warned = False

        self.ticks_pub = self.create_publisher(Int32MultiArray, "/pico/wheel_ticks",
                                               qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, "/pico/imu/data", qos_profile_sensor_data)
        self.status_pub = self.create_publisher(Int32, "/pico/drive/status", 10)
        self.twist_pub = self.create_publisher(Twist, "/sim/cmd_vel", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.profile_pub = self.create_publisher(String, "/sim/profile", latched)
        self.profile_pub.publish(String(data=json.dumps(self.profile, sort_keys=True)))

        self.ticks_link = DelayLine(self.profile["network"], self.ticks_pub.publish)
        self.imu_link = DelayLine(self.profile["network"], self.imu_pub.publish)

        self.create_subscription(Twist, "/pico/cmd_vel", self.on_cmd, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.create_subscription(Imu, "/sim/imu", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/sim/ultrasonic/points", self.on_points,
                                 qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, "/pico/imu/std", self.on_std, 10)
        self.last_loop = None
        self.create_timer(LOOP_S, self.loop)
        self.get_logger().info(
            f"Simulated Pico ready, profile '{self.profile['name']}' "
            f"(firmware model '{self.profile['firmware']}'). /pico/cmd_vel -> Gazebo; "
            "Gazebo -> /pico/wheel_ticks, /pico/imu/data, /pico/drive/status")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ----- firmware: commands, watchdog, motors -----
    def on_cmd(self, msg):
        self.firmware.command(msg.linear.x, msg.angular.z)
        self.last_cmd_time = self.now()

    def loop(self):
        now = self.now()
        if self.start_time is None:
            self.start_time = now
        dt = LOOP_S if self.last_loop is None else max(0.0, now - self.last_loop)
        self.last_loop = now
        if dt == 0.0:
            return

        # drive_control_update: stop if no command for 500 ms
        if (self.status == STATUS_DRIVING and self.last_cmd_time is not None
                and now - self.last_cmd_time > self.profile["firmware_params"]["watchdog_s"]):
            self.firmware.stop()
            self.set_status(STATUS_TIMEOUT, now)
        elif self.last_cmd_time is not None and now - self.last_cmd_time <= \
                self.profile["firmware_params"]["watchdog_s"]:
            self.set_status(STATUS_DRIVING if self.firmware.driving() else STATUS_STOPPED, now)
        if now >= self.next_status_time:
            self.publish_status(now)

        out = self.firmware.update(dt, self.counts)
        if self.firmware.outputs_speed:
            left, right = self.plant.update(dt, speeds=out)
        else:
            left, right = self.plant.update(dt, duties=out)
        self.send_twist(left, right)

        self.ping(now)
        ticks = Int32MultiArray()
        ticks.data = [int(self.counts[0]), int(self.counts[1]), self.ping_seq, self.echo_us]
        self.ticks_link.send(now, ticks)

    def set_status(self, status, now):
        if status != self.status:
            self.status = status
            if status == STATUS_TIMEOUT:
                self.get_logger().warn("No command for 500 ms: motors stopped (watchdog).")
            self.publish_status(now)

    def publish_status(self, now):
        self.status_pub.publish(Int32(data=self.status))
        self.next_status_time = now + 1.0

    def send_twist(self, left, right):
        v = (left + right) / 2.0
        w = (right - left) / self.model_sep
        if (abs(v) > self.model_max[0] + 1e-6 or abs(w) > self.model_max[1] + 1e-6) \
                and not self.clamp_warned:
            self.clamp_warned = True
            self.get_logger().warn(
                f"Wheel speeds need {v:.2f} m/s, {w:.2f} rad/s, beyond the Gazebo model's "
                f"limits ({self.model_max[0]} m/s, {self.model_max[1]} rad/s): the plugin "
                "clamps them. Shown once.")
        msg = Twist()
        msg.linear.x, msg.angular.z = v, w
        self.twist_pub.publish(msg)

    # ----- encoders -----
    def on_joints(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if "left_wheel_joint" not in pos or "right_wheel_joint" not in pos:
            return
        cpr = self.profile["firmware_params"]["counts_per_rev"]
        enc = self.profile["encoder"]
        self.counts = (round(pos["left_wheel_joint"] / (2 * math.pi) * cpr * enc["left_scale"]),
                       round(pos["right_wheel_joint"] / (2 * math.pi) * cpr * enc["right_scale"]))

    # ----- HC-SR04 -----
    def on_points(self, msg):
        self.nearest = nearest_return(msg, 0.02, self.us["max_range_m"] + 0.5, self.floor_z)

    def ping(self, now):
        if now < self.next_ping:
            return
        self.next_ping = now + self.us["ping_period_s"]
        self.ping_seq += 1
        r = self.nearest
        if random.random() < self.us["no_response_prob"]:
            self.echo_us = -2
        elif (r is None or r > self.us["max_range_m"]
              or random.random() < self.us["timeout_prob"]):
            self.echo_us = -1
        else:
            c = 331.3 + 0.606 * self.us["sound_temp_c"]
            r = max(0.02, r + random.gauss(0.0, self.us["noise_std_m"]))
            self.echo_us = int(round(2.0 * r / c * 1e6))

    # ----- BNO055 -----
    def on_std(self, msg):
        for i, value in enumerate(msg.data[:3]):
            if value > 0.0:
                self.std[i] = float(value)

    def on_imu(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        gyro_z = (msg.angular_velocity.z + math.radians(self.imu_p["gyro_bias_dps"])
                  + random.gauss(0.0, math.radians(self.imu_p["gyro_noise_dps"])))
        if self.last_imu_time is not None:
            dt = t - self.last_imu_time
            if 0.0 < dt < 0.5:
                self.yaw = math.atan2(math.sin(self.yaw + gyro_z * dt),
                                      math.cos(self.yaw + gyro_z * dt))
        self.last_imu_time = t
        now = self.now()
        if self.start_time is None or now - self.start_time < self.imu_p["calibration_delay_s"]:
            return   # the firmware publishes nothing until the gyro is calibrated

        yaw = self.yaw
        q = self.imu_p["quantize_deg"]
        if q > 0:
            yaw = math.radians(round(math.degrees(yaw) / q) * q)
        out = Imu()
        out.header.stamp = self.get_clock().now().to_msg()   # firmware: time-synced wall clock
        out.header.frame_id = "imu_link"
        out.orientation.z = math.sin(yaw / 2.0)
        out.orientation.w = math.cos(yaw / 2.0)
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity.z = gyro_z
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration.z -= GRAVITY   # the BNO055 fusion output is gravity-free
        out.orientation_covariance = diagonal(math.radians(self.std[0]) ** 2)
        out.angular_velocity_covariance = diagonal(self.std[1] ** 2)
        out.linear_acceleration_covariance = diagonal(self.std[2] ** 2)
        self.imu_link.send(now, out)


def main():
    rclpy.init()
    node = SimPicoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
