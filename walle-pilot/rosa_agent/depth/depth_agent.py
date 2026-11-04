"""ROSA agent that measures distance with the camera and drives to an object.

Run inside the container while the sensor fusion launch and the depth launch are
running (Gazebo or the real robot):
    python3 /workspace/depth/depth_agent.py

One model plans (OLLAMA_MODEL, default llama3.2, served by Ollama on your computer).
The depth model runs inside the depth node, on the CPU: no vision model is needed.

Example requests:
    How far away is the red ball?
    Drive to the red ball and stop 0.6 meters from it.
    What is the distance straight ahead?
"""

import os
import sys

from langchain_ollama import ChatOllama
from rosa import ROSA, RobotSystemPrompts

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "odometry"))
import depth_tools  # noqa: E402
import odom_tools  # noqa: E402

OLLAMA_MODEL = os.environ.get("OLLAMA_MODEL", "llama3.2")
OLLAMA_URL = os.environ.get("OLLAMA_URL", "http://host.docker.internal:11434")

PERSONA = (
    "You drive a small two-wheel robot car with a forward-facing camera. It knows its "
    "position from an EKF that fuses wheel odometry with its IMU: x is meters forward of "
    "the start point, y is meters to the left of it, and the heading is in degrees "
    "(+ = turned left). A depth model measures distances from camera photos; these "
    "measurements are estimates and can be wrong.")

TOOLS = [depth_tools.measure_distance, depth_tools.approach_to, odom_tools.turn,
         odom_tools.get_pose, odom_tools.stop_robot]

CAPABILITIES = (
    "measure_distance(target) takes a photo and reports the distance in meters to a colored "
    "object such as 'red ball', and the distance straight ahead. approach_to(target, "
    "stop_distance_m) drives to a colored object and stops the given distance from it, "
    "measuring before every step. turn(direction, degrees) turns in place. get_pose() "
    "reports the position. stop_robot() stops.")

INSTRUCTIONS = (
    "Use the words 'left' and 'right' for directions. To answer how far away an object "
    "is: call measure_distance with its color and name, then state the distance_m exactly "
    "as the tool returned it. To drive to an object: call approach_to with the object's "
    "color and name and the stop distance in meters; if the user gives no stop distance, "
    "use 0.6. If the object is not found, call turn left 45 degrees and measure_distance "
    "again, at most 8 times. Call one tool at a time. If a tool reports that it refused or "
    "stopped, do not repeat it: report what it said. Base every statement only on what the "
    "tools returned; never guess a distance.")


def build_agent():
    llm = ChatOllama(model=OLLAMA_MODEL, temperature=0, num_ctx=16384, base_url=OLLAMA_URL)
    prompts = RobotSystemPrompts(
        embodiment_and_persona=PERSONA,
        about_your_capabilities=CAPABILITIES,
        critical_instructions=INSTRUCTIONS,
    )
    return ROSA(ros_version=2, llm=llm, tools=TOOLS, prompts=prompts,
                verbose=True, streaming=False, accumulate_chat_history=False,
                max_iterations=40)


def main():
    odom_tools.get_robot()
    depth_tools.get_client()
    agent = build_agent()
    print(f"Depth robot agent ready (planner: {OLLAMA_MODEL}, tools: measure_distance, "
          "approach_to, turn, get_pose, stop_robot). Type 'quit' to exit.")
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
        depth_tools.close_client()
        odom_tools.close_robot()


if __name__ == "__main__":
    main()
