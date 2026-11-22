"""Make the fused estimate jump (sim test of the route follower's estimate-jump guard).

Reads the EKF's pose once from /odometry/filtered and sends it back on /set_pose (the
robot_localization EKF's reset input) moved DY m to the robot's LEFT, as the estimate did on
the robot in run 20261122_142402 (40 cm sideways in 0.5 s). The robot doesn't move.

    python3 jump_ekf.py 0.30
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


def main():
    dy = float(sys.argv[1]) if len(sys.argv) > 1 else 0.30
    rclpy.init()
    node = rclpy.create_node("walle_jump_ekf")
    got = {}
    node.create_subscription(Odometry, "/odometry/filtered",
                             lambda m: got.setdefault("odom", m), 10)
    pub = node.create_publisher(PoseWithCovarianceStamped, "/set_pose", 10)
    t_end = time.time() + 5.0
    while "odom" not in got and time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if "odom" not in got:
        print("no /odometry/filtered")
        return 1
    o = got["odom"]
    q = o.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = o.header.frame_id
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose = o.pose.pose
    msg.pose.pose.position.x -= dy * math.sin(yaw)
    msg.pose.pose.position.y += dy * math.cos(yaw)
    msg.pose.covariance = [0.0] * 36
    for i in (0, 7, 35):
        msg.pose.covariance[i] = 1e-4
    for _ in range(3):
        pub.publish(msg)
        time.sleep(0.05)
    print(f"EKF moved {dy:+.2f} m to the robot's left")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
