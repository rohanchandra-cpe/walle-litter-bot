"""ROSA agent that navigates with the fused estimate: "where am I?" and "go there".

Run inside the container while launch/sensor_fusion.launch.py is running (Gazebo or the
real robot):
    python3 /workspace/odometry/odom_agent.py

Example requests:
    Where are you?
    Drive forward 1 meter.
    Turn left 90 degrees.
    Go to x 1.0, y 0.5.
    Drive forward 0.5 meters, then turn right 90 degrees, then tell me where you are.
"""

import os

from langchain_ollama import ChatOllama
from rosa import ROSA, RobotSystemPrompts

import odom_tools

OLLAMA_MODEL = os.environ.get("OLLAMA_MODEL", "llama3.2")
OLLAMA_URL = os.environ.get("OLLAMA_URL", "http://host.docker.internal:11434")


def build_agent():
    llm = ChatOllama(model=OLLAMA_MODEL, temperature=0, num_ctx=16384, base_url=OLLAMA_URL)
    prompts = RobotSystemPrompts(
        embodiment_and_persona=(
            "You drive a small two-wheel robot that knows its position from an EKF that "
            "fuses wheel odometry with its BNO055 IMU. "
            "x is meters forward of the start point, y is meters to the left of it, and the "
            "heading is in degrees (0 = the start direction, + = turned left)."),
        about_your_capabilities=(
            "get_pose() reports where the robot is. drive_distance(direction, meters) drives "
            "forward or backward a distance. turn(direction, degrees) turns left or right in "
            "place. go_to(x, y) drives to a point. stop_robot() stops the robot. Every move "
            "stops when the fused estimate says it is done, and reports what it measured."),
        critical_instructions=(
            "Call one tool for each step the user asks for, in the order they asked. Copy "
            "every number exactly as the user wrote it, for example '0.5'. Use the words "
            "'left', 'right', 'forward', 'backward' for directions, never negative numbers. "
            "After the tools finish, report the numbers from the tool results exactly. "
            "Never claim the robot moved unless a tool reported it."),
    )
    return ROSA(ros_version=2, llm=llm,
                tools=[odom_tools.get_pose, odom_tools.drive_distance, odom_tools.turn,
                       odom_tools.go_to, odom_tools.stop_robot],
                prompts=prompts, verbose=True, streaming=False,
                accumulate_chat_history=False, max_iterations=10)


def main():
    odom_tools.get_robot()
    agent = build_agent()
    print(f"Sensor fusion robot agent ready (model: {OLLAMA_MODEL}). Type 'quit' to exit.")
    try:
        while True:
            try:
                request = input("You> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if request.lower() in ("quit", "exit"):
                break
            if request:
                try:
                    print(f"ROSA> {agent.invoke(request)}")
                except KeyboardInterrupt:
                    odom_tools.get_robot().stop()
                    print("\nInterrupted. The robot was stopped.")
    finally:
        odom_tools.close_robot()


if __name__ == "__main__":
    main()
