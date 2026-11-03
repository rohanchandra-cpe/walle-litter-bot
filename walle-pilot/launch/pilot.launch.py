"""WALL-E pilot: wheel odometry + BNO055 IMU -> robot_localization EKF.

Copied from Module 8's sensor_fusion.launch.py. Starts the wheel odometry node
(/wheel/odom), the IMU's position on the robot, and the EKF, which publishes
/odometry/filtered. The wheel geometry comes from config/pilot.yaml, so the
odometry and the RQ1-B stopping rule always use the same measured values.

Run inside the container (from /workspace/walle-pilot):
    ros2 launch launch/pilot.launch.py
    Fake robot, no hardware:  ros2 launch launch/pilot.launch.py fake:=true
    Upside-down IMU (chip facing the floor): ... imu_roll:=3.14159
"""

import os

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PILOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ODOMETRY_NODE = os.path.join(PILOT_DIR, "odometry", "odometry_node.py")
FAKE_PICO = os.path.join(PILOT_DIR, "tools", "fake_pico.py")
EKF_CONFIG = os.path.join(PILOT_DIR, "config", "ekf.yaml")
with open(os.path.join(PILOT_DIR, "config", "pilot.yaml")) as f:
    GEOMETRY = yaml.safe_load(f)["geometry"]


def generate_launch_description():
    sim = LaunchConfiguration("sim")
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="false",
                              description="true in Gazebo (uses simulation time)"),
        DeclareLaunchArgument("fake", default_value="false",
                              description="true: tools/fake_pico.py stands in for the robot"),
        # Uncertainty (1 standard deviation) of the wheel speed and turn rate
        DeclareLaunchArgument("wheel_speed_std", default_value="0.03"),
        DeclareLaunchArgument("wheel_turn_rate_std", default_value="0.10"),
        # Where the BNO055 sits on the robot, measured from base_link (the
        # midpoint between the wheels): meters and radians.
        DeclareLaunchArgument("imu_x", default_value="0.0"),
        DeclareLaunchArgument("imu_y", default_value="0.0"),
        DeclareLaunchArgument("imu_z", default_value="0.04"),
        DeclareLaunchArgument("imu_roll", default_value="0.0"),
        DeclareLaunchArgument("imu_yaw", default_value="0.0"),

        # Wheel encoders -> /wheel/odom (no TF: the EKF owns odom -> base_link)
        ExecuteProcess(cmd=[
            "python3", ODOMETRY_NODE, "--ros-args",
            "-p", ["use_sim_time:=", sim],
            "-p", f"wheel_diameter:={GEOMETRY['wheel_diameter_m']}",
            "-p", f"wheel_separation:={GEOMETRY['wheel_separation_m']}",
            "-p", f"counts_per_rev:={GEOMETRY['counts_per_rev']}",
            "-p", ["linear_speed_std:=", arg("wheel_speed_std")],
            "-p", ["turn_rate_std:=", arg("wheel_turn_rate_std")],
        ], output="screen"),

        # Rehearsal without hardware: a simulated Pico on the same topics
        ExecuteProcess(condition=IfCondition(arg("fake")),
                       cmd=["python3", FAKE_PICO], output="screen"),

        # base_link -> imu_link: tells the EKF how the IMU is mounted.
        # In Gazebo the robot model (URDF) already publishes this transform.
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="imu_mount", condition=UnlessCondition(sim),
             arguments=["--x", arg("imu_x"), "--y", arg("imu_y"), "--z", arg("imu_z"),
                        "--roll", arg("imu_roll"), "--pitch", "0", "--yaw", arg("imu_yaw"),
                        "--frame-id", "base_link", "--child-frame-id", "imu_link"],
             output="screen"),

        # The EKF: /wheel/odom + /pico/imu/data -> /odometry/filtered + TF
        Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node",
             parameters=[EKF_CONFIG, {"use_sim_time": sim}], output="screen"),
    ])
