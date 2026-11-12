"""WALL-E in Gazebo: a test world, the robot, the simulated Pico and ESP32-CAM, and the
pilot's own odometry + EKF launch file.

Run inside the sim container (make sim-shell):
    ros2 launch /workspace/walle-sim/launch/sim.launch.py world:=motion_area
    ros2 launch /workspace/walle-sim/launch/sim.launch.py world:=rq3_ranging gui:=false
    ros2 launch /workspace/walle-sim/launch/sim.launch.py profile:=pid_firmware

Arguments:
    world     rq1_lane, rq2_protractor, motion_area, rq3_ranging, rq3_ranging_dim,
              inspection_room (worlds/*.sdf)
    profile   a hardware profile from config/sim_profiles.yaml (default pilot_firmware)
    gui       true: Gazebo window in the browser desktop (http://localhost:6080/vnc.html)
    pilot     true: also start walle-pilot/launch/pilot.launch.py sim:=true
              (wheel odometry + EKF), exactly what runs next to the real robot
    seed      sim_pico_bridge random seed (0 = random)

After it is up, the robot looks like the real one on the ROS graph:
    /pico/cmd_vel, /pico/wheel_ticks, /pico/imu/data, /pico/drive/status,
    /wheel/odom, /odometry/filtered, and http://localhost:8080/capture
plus simulator-only ground truth on /sim/ground_truth (never an input to the robot).
"""

import math
import os

import yaml

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

SIM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO_DIR = os.path.dirname(SIM_DIR)
ROBOT_FILE = os.path.join(SIM_DIR, "urdf", "walle_sim.urdf.xacro")
BRIDGE_FILE = os.path.join(SIM_DIR, "config", "bridge.yaml")
WORLD_INFO = os.path.join(SIM_DIR, "worlds", "world_info.yaml")
SIM_CONFIG = os.path.join(SIM_DIR, "config", "sim_pilot.yaml")
PILOT_LAUNCH = os.path.join(REPO_DIR, "walle-pilot", "launch", "pilot.launch.py")
PICO_BRIDGE = os.path.join(SIM_DIR, "nodes", "sim_pico_bridge.py")
CAMERA_SERVER = os.path.join(SIM_DIR, "nodes", "sim_camera_server.py")
ROBOT_NAME = "walle"


def look_at(target, offset=(-0.6, -1.0, 0.8)):
    """GUICamera request: the Gazebo window's camera looking down at `target`."""
    eye = [t + o for t, o in zip(target, offset)]
    dx, dy, dz = (t - e for t, e in zip(target, eye))
    yaw, pitch = math.atan2(dy, dx), math.atan2(-dz, math.hypot(dx, dy))
    cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    return (f"pose: {{position: {{x: {eye[0]:.3f}, y: {eye[1]:.3f}, z: {eye[2]:.3f}}}, "
            f"orientation: {{x: {-sy * sp:.4f}, y: {cy * sp:.4f}, z: {sy * cp:.4f}, w: {cy * cp:.4f}}}}}")


def truthy(value):
    return str(value).lower() in ("1", "true", "yes")


def setup(context):
    get = lambda name: context.launch_configurations[name]  # noqa: E731
    world, profile, gui = get("world"), get("profile"), truthy(get("gui"))
    world_file = os.path.join(SIM_DIR, "worlds", f"{world}.sdf")
    with open(WORLD_INFO) as f:
        info = yaml.safe_load(f)["worlds"]
    if world not in info or not os.path.exists(world_file):
        raise RuntimeError(f"Unknown world '{world}'. Choose one of: {', '.join(info)}")
    x, y, yaw_deg = info[world]["spawn"]

    # The pilot scripts and pilot.launch.py read the SIMULATED robot's config
    os.environ.setdefault("WALLE_CONFIG", SIM_CONFIG)
    robot_description = ParameterValue(Command(["xacro ", ROBOT_FILE]), value_type=str)
    sim_time = {"use_sim_time": True}

    actions = [
        ExecuteProcess(cmd=["ign", "gazebo", "-s", "-r", "-v", "2", world_file],
                       output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description, **sim_time}],
             output="screen"),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             parameters=[{"config_file": BRIDGE_FILE, **sim_time}], output="screen"),
        # Ground truth for the harness: every model's true pose (static ones included)
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="ground_truth_bridge",
             arguments=[f"/world/{world}/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V"],
             remappings=[(f"/world/{world}/pose/info", "/sim/ground_truth")],
             parameters=[sim_time], output="screen"),
        # The simulated Pico and ESP32-CAM run on the computer's clock, like the hardware
        ExecuteProcess(cmd=["python3", PICO_BRIDGE, "--ros-args",
                            "-p", f"profile:={profile}", "-p", f"seed:={get('seed')}"],
                       output="screen"),
        ExecuteProcess(cmd=["python3", CAMERA_SERVER, "--ros-args",
                            "-p", f"profile:={profile}", "-p", "port:=8080"],
                       output="screen"),
        # Give the server time to load the world before spawning the robot
        TimerAction(period=5.0, actions=[Node(
            package="ros_gz_sim", executable="create", output="screen",
            arguments=["-world", world, "-topic", "robot_description", "-name", ROBOT_NAME,
                       "-x", str(x), "-y", str(y), "-z", "0.04",
                       "-Y", str(math.radians(yaw_deg))])]),
    ]
    if gui:
        view = info[world].get("view", {"target": [x + 0.3, y, 0.0]})
        camera = look_at(view["target"], view.get("offset", (-0.6, -1.0, 0.8)))
        actions += [
            ExecuteProcess(cmd=["nice", "-n", "10", "ign", "gazebo", "-g", "-v", "1"],
                           output="screen"),
            TimerAction(period=8.0, actions=[ExecuteProcess(cmd=[
                "bash", "-c",
                "for i in $(seq 1 30); do ign service -s /gui/move_to/pose "
                "--reqtype ignition.msgs.GUICamera --reptype ignition.msgs.Boolean "
                f"--timeout 2000 --req '{camera}' > /dev/null 2>&1 && break; sleep 2; done"])]),
        ]
    if truthy(get("pilot")):
        # The robot's own launch file, on the computer's clock as on the robot (sim:=false:
        # robot_io stamps /set_pose with that clock). The IMU mount matches the URDF's
        # imu_link, which robot_state_publisher also publishes.
        actions.append(TimerAction(period=7.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PILOT_LAUNCH),
            launch_arguments={"sim": "false", "imu_x": "0.0", "imu_y": "0.0",
                              "imu_z": "0.05"}.items())]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="motion_area"),
        DeclareLaunchArgument("profile", default_value="pilot_firmware"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("pilot", default_value="true"),
        DeclareLaunchArgument("seed", default_value="0"),
        OpaqueFunction(function=setup),
    ])
