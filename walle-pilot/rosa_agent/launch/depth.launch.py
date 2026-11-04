"""Module 9 depth lab: camera node + depth node.

Starts camera_node.py (photos -> /camera/image/compressed) and depth_node.py
(questions on /depth/ask -> Depth Anything V2 on the CPU -> answers on /depth/result).

Run inside the container:
    Real robot:   ros2 launch /workspace/launch/depth.launch.py camera_url:=http://CAMERA_IP/capture
    Gazebo:       ros2 launch /workspace/launch/depth.launch.py sim:=true
    Bigger model: ... model_size:=base     (downloads 390 MB the first time)
    Faster:       ... input_size:=266
    Known ball:   ... object_diameter_m:=0.14   (also reports a distance from its size)
    Camera already running (for example vision.launch.py): ... camera:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution

CAMERA_NODE = "/workspace/vision/camera_node.py"
DEPTH_NODE = "/workspace/depth/depth_node.py"


def generate_launch_description():
    sim = LaunchConfiguration("sim")
    camera = LaunchConfiguration("camera")
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="false",
                              description="true in Gazebo (uses the simulated camera)"),
        DeclareLaunchArgument("camera", default_value="true",
                              description="false if another launch already runs camera_node.py"),
        DeclareLaunchArgument("camera_url", default_value="http://192.168.1.50/capture",
                              description="the ESP32-CAM's photo address"),
        DeclareLaunchArgument("rate", default_value="2.0", description="photos per second"),
        DeclareLaunchArgument("model_size", default_value="small",
                              description="small, base, or large"),
        DeclareLaunchArgument("input_size", default_value="364",
                              description="model input size in pixels (multiple of 14)"),
        DeclareLaunchArgument("hfov_deg", default_value="66.0",
                              description="camera horizontal field of view in degrees"),
        DeclareLaunchArgument("object_diameter_m", default_value="0.0",
                              description="real diameter of the round target (0 = off)"),

        # Camera: the ESP32-CAM over Wi-Fi, or Gazebo's camera
        ExecuteProcess(condition=IfCondition(AndSubstitution(camera, NotSubstitution(sim))),
                       output="screen", cmd=[
            "python3", CAMERA_NODE, "--ros-args",
            "-p", "source:=http", "-p", ["url:=", arg("camera_url")],
            "-p", ["rate:=", arg("rate")]]),
        ExecuteProcess(condition=IfCondition(AndSubstitution(camera, sim)),
                       output="screen", cmd=[
            "python3", CAMERA_NODE, "--ros-args",
            "-p", "source:=sim", "-p", "use_sim_time:=true",
            "-p", ["rate:=", arg("rate")]]),

        # Depth: photo + question -> Depth Anything V2 -> distances in meters
        ExecuteProcess(output="screen", cmd=[
            "python3", DEPTH_NODE, "--ros-args",
            "-p", ["use_sim_time:=", sim],
            "-p", ["model_size:=", arg("model_size")],
            "-p", ["input_size:=", arg("input_size")],
            "-p", ["hfov_deg:=", arg("hfov_deg")],
            "-p", ["object_diameter_m:=", arg("object_diameter_m")]]),
    ])
