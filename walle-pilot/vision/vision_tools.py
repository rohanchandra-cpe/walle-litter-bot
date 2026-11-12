"""ROSA tools that let the agent see: ask the camera what it sees, then decide.

look(target) and describe_scene() send a question on /vision/ask and wait for the
vision node's answer on /vision/result. Every answer is also added to
/workspace/walle-pilot/vision/results/vision_log.csv for your evaluation.
"""

import csv
import json
import os
import threading
import time
import uuid

import rclpy
from langchain_core.tools import tool
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

ANSWER_TIMEOUT = 120.0   # s; the first question also loads the vision model
LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results", "vision_log.csv")
LOG_FIELDS = ["time", "target", "target_visible", "position", "size", "path_clear",
              "obstacle_ahead", "seconds", "model", "image", "description", "error"]


class VisionClient(Node):
    """Sends questions to the vision node and waits for the matching answer."""

    def __init__(self):
        if not rclpy.ok():
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        super().__init__("rosa_vision_client")
        self.answers = {}
        self.cond = threading.Condition()
        self.ask_pub = self.create_publisher(String, "/vision/ask", 10)
        self.create_subscription(String, "/vision/result", self.on_result, 10)
        self.executor_ = SingleThreadedExecutor()
        self.executor_.add_node(self)
        self.thread = threading.Thread(target=self.executor_.spin, daemon=True)
        self.thread.start()

    def on_result(self, msg):
        try:
            answer = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self.cond:
            self.answers[answer.get("id")] = answer
            self.cond.notify_all()

    def ask(self, target=""):
        # Give ROS discovery up to 5 s to find the vision node the first time
        end = time.time() + 5.0
        while self.count_subscribers("/vision/ask") == 0 and time.time() < end:
            time.sleep(0.2)
        if self.count_subscribers("/vision/ask") == 0:
            return {"target": target,
                    "error": "No vision node. Is launch/vision.launch.py running?"}
        request_id = uuid.uuid4().hex
        self.ask_pub.publish(String(data=json.dumps({"id": request_id, "target": target})))
        with self.cond:
            self.cond.wait_for(lambda: request_id in self.answers, timeout=ANSWER_TIMEOUT)
            answer = self.answers.pop(request_id, None)
        if answer is None:
            answer = {"target": target, "error": "No answer from the vision node in time."}
        log(answer)
        return answer

    def close(self):
        self.executor_.shutdown()
        self.thread.join(timeout=2.0)
        self.destroy_node()


def log(answer):
    os.makedirs(os.path.dirname(LOG), exist_ok=True)
    new_file = not os.path.exists(LOG)
    with open(LOG, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=LOG_FIELDS, extrasaction="ignore")
        if new_file:
            writer.writeheader()
        writer.writerow({"time": time.strftime("%Y-%m-%d %H:%M:%S"), **answer})


_client = None


def get_client():
    global _client
    if _client is None:
        _client = VisionClient()
    return _client


def close_client():
    global _client
    if _client is not None:
        _client.close()
        _client = None


def yes_no(value):
    return "yes" if value else "no"


def path_text(answer):
    if answer["path_clear"]:
        return "path_clear: yes"
    return f"path_clear: no ({answer['obstacle_ahead']} ahead)"


def format_answer(answer):
    """One short line the planning model can read reliably."""
    if answer.get("error"):
        return f"Camera error: {answer['error']} Do not move."
    if answer.get("target"):
        return (f"target_visible: {yes_no(answer['target_visible'])} | "
                f"position: {answer['position']} | size: {answer['size']} | "
                f"{path_text(answer)} | description: {answer['description']}")
    return f"{path_text(answer)} | description: {answer['description']}"


# ===== ROSA tools =====
@tool
def look(target: str) -> str:
    """Take a photo and look for one object. target: a short name such as 'red ball'.
    Returns target_visible (yes/no), position (left/center/right), size
    (small/medium/large), path_clear (yes/no, and what is ahead), and a description."""
    return format_answer(get_client().ask(str(target).strip()))


@tool
def describe_scene() -> str:
    """Take a photo and describe what the robot sees, and whether the path ahead is clear."""
    return format_answer(get_client().ask(""))