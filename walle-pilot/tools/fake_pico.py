"""A simulated Pico for rehearsing the pilot without the robot.

    ros2 launch launch/pilot.launch.py fake:=true

Publishes the same topics as the pilot firmware: /pico/wheel_ticks (with
ultrasonic pings), /pico/imu/data, and /pico/drive/status, from the commands on
/pico/cmd_vel. The motion has a little lag, coast, wheel-size mismatch, and noise,
so the scripts see realistic numbers, but nothing here is data: never log fake
runs as pilot trials.

    --range 1.2      distance the fake ultrasonic sensor reports (m)
"""

import argparse
import math
import random

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32, Int32MultiArray

WHEEL_DIAMETER = 0.0635
WHEEL_SEPARATION = 0.127
COUNTS_PER_REV = 4320
LEFT_SCALE, RIGHT_SCALE = 1.01, 0.99     # unequal wheels: drift, like a real robot
TIME_CONSTANT = 0.12                     # s, motor lag (gives a coast after a stop)
TIMEOUT = 0.5                            # s without a command: stop, like the firmware


class FakePico(Node):
    def __init__(self, fake_range):
        super().__init__("fake_pico")
        self.range = fake_range
        self.cmd = (0.0, 0.0)
        self.last_cmd = -1.0
        self.wheel = [0.0, 0.0]          # wheel speeds, m/s
        self.counts = [0.0, 0.0]
        self.heading = 0.0
        self.ping_seq, self.echo, self.next_ping = 0, 0, 0.0
        self.ticks_pub = self.create_publisher(Int32MultiArray, "/pico/wheel_ticks",
                                               qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, "/pico/imu/data", qos_profile_sensor_data)
        self.status_pub = self.create_publisher(Int32, "/pico/drive/status", 10)
        self.create_subscription(Twist, "/pico/cmd_vel", self.on_cmd, qos_profile_sensor_data)
        self.timer = self.create_timer(0.02, self.step)
        self.get_logger().info("Fake Pico running: these are NOT real measurements.")

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.last_cmd = self.now()

    def step(self):
        dt, now = 0.02, self.now()
        v, w = self.cmd if now - self.last_cmd < TIMEOUT else (0.0, 0.0)
        targets = (v - w * WHEEL_SEPARATION / 2, v + w * WHEEL_SEPARATION / 2)
        alpha = dt / TIME_CONSTANT
        for i, target in enumerate(targets):
            self.wheel[i] += alpha * (target - self.wheel[i])
            if abs(self.wheel[i]) < 0.002 and target == 0.0:
                self.wheel[i] = 0.0
        true_left, true_right = self.wheel[0] * LEFT_SCALE, self.wheel[1] * RIGHT_SCALE
        self.heading += (true_right - true_left) / WHEEL_SEPARATION * dt
        per_meter = COUNTS_PER_REV / (math.pi * WHEEL_DIAMETER)
        self.counts[0] += self.wheel[0] * dt * per_meter
        self.counts[1] += self.wheel[1] * dt * per_meter

        if now >= self.next_ping:
            self.next_ping = now + 0.06
            self.ping_seq += 1
            if random.random() < 0.05:
                self.echo = -1
            else:
                r = self.range + random.gauss(0.0, 0.004)
                self.echo = int(2 * r / (331.3 + 0.606 * 21.0) * 1e6)

        ticks = Int32MultiArray()
        ticks.data = [int(self.counts[0]), int(self.counts[1]), self.ping_seq, self.echo]
        self.ticks_pub.publish(ticks)

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "imu_link"
        yaw = self.heading + random.gauss(0.0, 0.002)
        imu.orientation.w, imu.orientation.z = math.cos(yaw / 2), math.sin(yaw / 2)
        imu.angular_velocity.z = (true_right - true_left) / WHEEL_SEPARATION + random.gauss(0, 0.005)
        imu.linear_acceleration.z = 9.81
        for cov, var in ((imu.orientation_covariance, math.radians(2.0) ** 2),
                         (imu.angular_velocity_covariance, 0.02 ** 2),
                         (imu.linear_acceleration_covariance, 0.3 ** 2)):
            cov[0] = cov[4] = cov[8] = var
        self.imu_pub.publish(imu)
        moving = any(abs(s) > 0.0 for s in self.wheel)
        self.status_pub.publish(Int32(data=1 if moving else 0))


def main():
    parser = argparse.ArgumentParser(description="Simulated Pico for rehearsals")
    parser.add_argument("--range", type=float, default=1.0)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = FakePico(args.range)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
