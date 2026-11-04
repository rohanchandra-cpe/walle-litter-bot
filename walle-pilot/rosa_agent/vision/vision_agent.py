"""ROSA agent that sees with the camera and moves with the fused position estimate.

Run inside the container while the sensor fusion launch and the vision launch are
running (Gazebo or the real robot):
    python3 /workspace/vision/vision_agent.py

Two models, both served by Ollama on your computer:
    OLLAMA_MODEL  (default llama3.2)  plans and calls the tools (this agent)
    VISION_MODEL  (default gemma4)    reads the photos (vision_node.py)

Two tool sets (AGENT_TOOLS):
    mission (default)  search_for, approach, drive_forward_if_clear: the safety rules
                       live in the tools, so the planner only picks the next step
    basic              look, turn, drive_distance, go_to: the planner must follow the
                       whole procedure itself (compare the two in Exercise 4)

Example requests:
    What do you see?
    Is the path ahead clear?
    Drive forward 0.5 meters.
    Find the red ball and drive to it.
"""

import os
import sys

from langchain_ollama import ChatOllama
from rosa import ROSA, RobotSystemPrompts

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "odometry"))
import mission_tools  # noqa: E402
import odom_tools  # noqa: E402
import vision_tools  # noqa: E402

# OLLAMA_MODEL = os.environ.get("OLLAMA_MODEL", "llama3.2")
OLLAMA_MODEL = "qwen3.5:4b"
OLLAMA_URL = os.environ.get("OLLAMA_URL", "http://host.docker.internal:11434")
AGENT_TOOLS = os.environ.get("AGENT_TOOLS", "mission")

PERSONA = (
    "You drive a small two-wheel robot car with a forward-facing camera. It knows its "
    "position from an EKF that fuses wheel odometry with its IMU: x is meters forward of "
    "the start point, y is meters to the left of it, and the heading is in degrees "
    "(+ = turned left).")

MISSION = {
    "tools": [vision_tools.look, vision_tools.describe_scene, mission_tools.search_for,
              mission_tools.approach, mission_tools.drive_forward_if_clear,
              odom_tools.turn, odom_tools.get_pose, odom_tools.stop_robot],
    "capabilities": (
        "look(target) and describe_scene() take a photo and report what the camera sees and "
        "whether the path ahead is clear. search_for(target) turns in place, looking, until "
        "it sees the object. approach(target) drives to a visible object, looking before "
        "every step. drive_forward_if_clear(meters) drives forward only if the camera sees "
        "nothing in the way. turn(direction, degrees) turns in place. get_pose() reports the "
        "position. stop_robot() stops."),
    "instructions": (
        "To find an object and drive to it: call search_for with the object's name; if it "
        "was found, call approach with the same name; then call get_pose and report. "
        "To drive forward, always use drive_forward_if_clear. Call one tool at a time. If a "
        "tool reports that it refused, stopped, or could not find something, do not repeat "
        "it: report what it said. Base every statement only on what the tools returned; "
        "never invent what the camera sees."),
}

BASIC = {
    "tools": [vision_tools.look, vision_tools.describe_scene, odom_tools.get_pose,
              odom_tools.drive_distance, odom_tools.turn, odom_tools.go_to,
              odom_tools.stop_robot],
    "capabilities": (
        "look(target) takes a photo and reports whether an object is visible, where it is "
        "(left, center, right), how big it looks (small, medium, large), and whether the "
        "path ahead is clear. describe_scene() takes a photo and describes it. get_pose() "
        "reports the position. drive_distance(direction, meters) drives forward or backward. "
        "turn(direction, degrees) turns in place. go_to(x, y) drives to a point. "
        "stop_robot() stops."),
    "instructions": (
        "Never drive forward unless the most recent look or describe_scene said "
        "path_clear: yes. Call one tool at a time and wait for its result. "
        "To find an object and drive to it, repeat: 1. Call look with the object's name. "
        "2. If target_visible is no: turn left 45 degrees, then look again; after 8 turns "
        "stop and say it was not found. 3. If position is left: turn left 15 degrees; if "
        "right: turn right 15 degrees; then look again. 4. If position is center and size "
        "is large: you have arrived; call get_pose and report. 5. If position is center and "
        "size is small or medium: if path_clear is yes, drive forward 0.3 meters, then look "
        "again; if not, stop and report what is blocking the way. Base every statement only "
        "on what the tools returned; never invent what the camera sees."),
}


def build_agent():
    config = MISSION if AGENT_TOOLS == "mission" else BASIC
    llm = ChatOllama(model=OLLAMA_MODEL, temperature=0, num_ctx=16384, base_url=OLLAMA_URL)
    prompts = RobotSystemPrompts(
        embodiment_and_persona=PERSONA,
        about_your_capabilities=config["capabilities"],
        critical_instructions=(
            "Use the words 'left', 'right', 'forward', 'backward' for directions and "
            "positive numbers for sizes. " + config["instructions"]),
    )
    return ROSA(ros_version=2, llm=llm, tools=config["tools"], prompts=prompts,
                verbose=True, streaming=False, accumulate_chat_history=False,
                max_iterations=40)


def main():
    odom_tools.get_robot()
    vision_tools.get_client()
    agent = build_agent()
    print(f"Vision robot agent ready (planner: {OLLAMA_MODEL}, vision: "
          f"{os.environ.get('VISION_MODEL', 'gemma4')}, tools: {AGENT_TOOLS}). "
          "Type 'quit' to exit.")
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
        vision_tools.close_client()
        odom_tools.close_robot()


if __name__ == "__main__":
    main()