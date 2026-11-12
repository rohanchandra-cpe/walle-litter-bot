"""ROSA agent that identifies objects with the camera and ranges them with the HC-SR04.

Run inside the container while launch/pilot.launch.py and launch/vision.launch.py
are running:
    python3 /workspace/walle-pilot/range/range_agent.py

Two models, both served by Ollama on your computer:
    OLLAMA_MODEL  (default qwen3.5:4b)  plans and calls the tools (this agent)
    VISION_MODEL  (default gemma4)      reads the photos (vision_node.py)

Example requests:
    What is in front of you and how far away is it?
    How far away is the red ball?
    How far is the nearest thing ahead?
"""

import os
import sys

from langchain_ollama import ChatOllama
from rosa import ROSA, RobotSystemPrompts

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "odometry"))
import odom_tools  # noqa: E402
import ultrasonic_tools  # noqa: E402
import vision_tools  # noqa: E402

OLLAMA_MODEL = os.environ.get("OLLAMA_MODEL", "qwen3.5:4b")
OLLAMA_URL = os.environ.get("OLLAMA_URL", "http://host.docker.internal:11434")

TOOLS = [ultrasonic_tools.identify_and_range, ultrasonic_tools.measure_ultrasonic,
         odom_tools.turn, odom_tools.get_pose, odom_tools.stop_robot]

PERSONA = (
    "You drive a small two-wheel robot car with a forward-facing camera and an ultrasonic "
    "distance sensor mounted below it. The sensor measures the nearest surface straight "
    "ahead.")

CAPABILITIES = (
    "identify_and_range(target) takes a photo to identify an object and measures how far "
    "away it is. measure_ultrasonic() measures the nearest surface straight ahead without "
    "identifying it. turn(direction, degrees) turns in place. get_pose() reports the "
    "position. stop_robot() stops.")

INSTRUCTIONS = (
    "Use the words 'left' and 'right' for directions and positive numbers for sizes. "
    "To identify an object or say how far away it is: call identify_and_range with the "
    "object's name, or with an empty target if the user did not name one. If the result "
    "says to turn, call turn as it says, then call identify_and_range again; do this at "
    "most 3 times. If target_visible is no, say it was not seen; do not search for it. "
    "Use measure_ultrasonic only when the user wants a distance and not an identification. "
    "Report the distance exactly as the tool gave it, and repeat any warning. If distance_m "
    "is none or the distance was not measured, say that no distance is available; never "
    "estimate one. Call one tool at a time. Base every statement only on what the tools "
    "returned; never invent what the camera sees.")


def build_agent():
    llm = ChatOllama(model=OLLAMA_MODEL, temperature=0, num_ctx=16384, base_url=OLLAMA_URL)
    prompts = RobotSystemPrompts(embodiment_and_persona=PERSONA,
                                 about_your_capabilities=CAPABILITIES,
                                 critical_instructions=INSTRUCTIONS)
    return ROSA(ros_version=2, llm=llm, tools=TOOLS, prompts=prompts, verbose=True,
                streaming=False, accumulate_chat_history=False, max_iterations=20)


def main():
    odom_tools.get_robot()
    vision_tools.get_client()
    ultrasonic_tools.get_client()
    agent = build_agent()
    print(f"Range agent ready (planner: {OLLAMA_MODEL}, vision: "
          f"{os.environ.get('VISION_MODEL', 'gemma4')}). Type 'quit' to exit.")
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
        ultrasonic_tools.close_client()
        vision_tools.close_client()
        odom_tools.close_robot()


if __name__ == "__main__":
    main()
