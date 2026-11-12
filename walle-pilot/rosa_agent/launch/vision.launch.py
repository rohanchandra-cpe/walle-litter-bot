"""Module 8 camera vision lab: camera node + vision node.

Starts camera_node.py (photos -> /camera/image/compressed) and vision_node.py
(questions on /vision/ask -> Ollama vision model -> answers on /vision/result).

Run inside the container:
    Real robot:  ros2 launch /workspace/walle-pilot/rosa_agent/launch/vision.launch.py camera_url:=http://CAMERA_IP/capture
    Gazebo:      ros2 launch /workspace/walle-pilot/rosa_agent/launch/vision.launch.py sim:=true
    Other model: ... vision_model:=gemma3
    Dim room:    ... camera_url:=http://CAMERA_IP/capture?flash=1
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

CAMERA_NODE = "/workspace/walle-pilot/rosa_agent/vision/camera_node.py"
VISION_NODE = "/workspace/walle-pilot/rosa_agent/vision/vision_node.py"


def generate_launch_description():
    sim = LaunchConfiguration("sim")
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="false",
                              description="true in Gazebo (uses the simulated camera)"),
        DeclareLaunchArgument("camera_url", default_value="http://192.168.1.50/capture",
                              description="the ESP32-CAM photo URL from its serial output"),
        DeclareLaunchArgument("rate", default_value="2.0", description="photos per second"),
        DeclareLaunchArgument("vision_model",
                              default_value=os.environ.get("VISION_MODEL", "gemma4")),

        # Camera: the ESP32-CAM over Wi-Fi, or Gazebo's camera
        ExecuteProcess(condition=UnlessCondition(sim), output="screen", cmd=[
            "python3", CAMERA_NODE, "--ros-args",
            "-p", "source:=http", "-p", ["url:=", arg("camera_url")],
            "-p", ["rate:=", arg("rate")]]),
        ExecuteProcess(condition=IfCondition(sim), output="screen", cmd=[
            "python3", CAMERA_NODE, "--ros-args",
            "-p", "source:=sim", "-p", "use_sim_time:=true",
            "-p", ["rate:=", arg("rate")]]),

        # Vision: photo + question -> Ollama vision model -> answer
        ExecuteProcess(output="screen", cmd=[
            "python3", VISION_NODE, "--ros-args",
            "-p", ["use_sim_time:=", sim],
            "-p", ["model:=", arg("vision_model")]]),
    ])