"""Vision node: answers questions about the camera image with an Ollama vision model.

Subscribes  /camera/image/compressed   sensor_msgs/CompressedImage   from camera_node.py
            /vision/ask                std_msgs/String   JSON {"id": "...", "target": "red ball"}
                                                         (target "" = describe the scene)
Publishes   /vision/result             std_msgs/String   JSON with the answer (below)

For every question it waits for a photo taken AFTER the question arrived (so the
robot has stopped moving), sends it to Ollama's /api/chat with a JSON schema, and
publishes:
    {"id", "target", "description", "target_visible", "position", "size",
     "obstacle_ahead", "path_clear", "seconds", "model", "image", "error"}
Each analyzed photo is saved in /workspace/walle-pilot/vision/results/images/ for your report.

Started by launch/vision.launch.py. Settings: model (default: VISION_MODEL or gemma4),
ollama_url (default: OLLAMA_URL or http://host.docker.internal:11434).
"""

import base64
import json
import os
import queue
import threading
import time
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

RESULTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results", "images")
FRESH_PHOTO_TIMEOUT = 5.0     # s to wait for a photo taken after the question

# The answer must have exactly these fields, in this order (Ollama "structured
# outputs"). The description comes FIRST: writing down what it sees before answering
# the other fields keeps the model from inventing objects.
ANSWER_SCHEMA = {
    "type": "object",
    "properties": {
        "description": {"type": "string"},
        "target_visible": {"type": "boolean"},
        "position": {"type": "string", "enum": ["left", "center", "right", "none"]},
        "size": {"type": "string", "enum": ["small", "medium", "large", "none"]},
        "obstacle_ahead": {"type": "string"},
    },
    "required": ["description", "target_visible", "position", "size", "obstacle_ahead"],
}

COMMON = (
    "You are the forward-facing camera of a small robot car that drives on the floor. "
    "Answer in this order. "
    "description: {description} ")

TARGET_PROMPT = COMMON.format(description=(
    "one or two sentences naming every object you see, its color, and whether it is on "
    "the left, in the center, or on the right of the image.")) + (
    "target_visible: true only if the target ({target}) appears in your description. "
    "position: which third of the image the target's center is in: left, center, or right "
    "(none if not visible). "
    "size: how wide the target looks compared with the image width: small (less than one "
    "tenth), medium, or large (more than one quarter); none if not visible. "
    "obstacle_ahead: the name of the object, other than the target, that stands in the "
    "center third of the image where the robot would drive into it if it drove straight "
    "ahead; write none if there is no such object. Walls do not count.")

SCENE_PROMPT = COMMON.format(description=(
    "two or three sentences naming every object you see, its color, and whether it is on "
    "the left, in the center, or on the right of the image.")) + (
    "There is no target: set target_visible false, position none, and size none. "
    "obstacle_ahead: the name of the object that stands in the center third of the image "
    "where the robot would drive into it if it drove straight ahead; write none if there "
    "is no such object. Walls do not count.")


def tidy(answer):
    """Make the fields consistent and add path_clear for the planner."""
    obstacle = answer["obstacle_ahead"].strip().strip(".")
    if obstacle.lower() in ("", "none", "nothing", "no", "n/a"):
        obstacle = "none"
    answer["obstacle_ahead"] = obstacle
    answer["path_clear"] = obstacle == "none"
    if not answer["target_visible"]:
        answer["position"] = answer["size"] = "none"
    return answer


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")
        self.declare_parameter("model", os.environ.get("VISION_MODEL", "gemma4"))
        self.declare_parameter("ollama_url", os.environ.get(
            "OLLAMA_URL", "http://host.docker.internal:11434"))
        self.model = self.get_parameter("model").value
        self.chat_url = self.get_parameter("ollama_url").value.rstrip("/") + "/api/chat"
        self.lock = threading.Lock()
        self.latest = None
        self.questions = queue.Queue()

        self.result_pub = self.create_publisher(String, "/vision/result", 10)
        self.create_subscription(CompressedImage, "/camera/image/compressed", self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(String, "/vision/ask", self.on_ask, 10)
        threading.Thread(target=self.worker, daemon=True).start()
        os.makedirs(RESULTS, exist_ok=True)
        self.get_logger().info(f"Vision node ready: model {self.model} at {self.chat_url}")

    def on_image(self, msg: CompressedImage):
        with self.lock:
            self.latest = msg

    def on_ask(self, msg: String):
        try:
            request = json.loads(msg.data)
        except json.JSONDecodeError:
            request = {"target": msg.data}
        request.setdefault("id", str(int(time.time() * 1000)))
        request["target"] = str(request.get("target", "")).strip()
        self.questions.put((request, self.get_clock().now()))

    def fresh_photo(self, asked_at):
        end = time.time() + FRESH_PHOTO_TIMEOUT
        while time.time() < end:
            with self.lock:
                msg = self.latest
            if msg is not None and Time.from_msg(msg.header.stamp) >= asked_at:
                return msg
            time.sleep(0.05)
        return None

    def worker(self):
        while rclpy.ok():
            request, asked_at = self.questions.get()
            result = {"id": request["id"], "target": request["target"], "model": self.model}
            try:
                photo = self.fresh_photo(asked_at)
                if photo is None:
                    result["error"] = "No new camera photo. Is the camera node running?"
                else:
                    result["image"] = self.save(photo, request["id"])
                    result.update(self.ask_model(bytes(photo.data), request["target"]))
            except Exception as error:   # never let one bad question stop the node
                result["error"] = f"Vision node error: {error}"
            self.result_pub.publish(String(data=json.dumps(result)))
            self.get_logger().info(
                f"[{result.get('seconds', 0):.1f} s] {request['target'] or 'scene'}: "
                f"{result.get('error') or result.get('description', '')}")

    def save(self, photo, request_id):
        os.makedirs(RESULTS, exist_ok=True)   # in case the folder was deleted
        name = f"{time.strftime('%Y%m%d_%H%M%S')}_{request_id[-6:]}.jpg"
        with open(os.path.join(RESULTS, name), "wb") as f:
            f.write(bytes(photo.data))
        return os.path.join("vision", "results", "images", name)

    def ask_model(self, jpeg, target):
        prompt = TARGET_PROMPT.format(target=target) if target else SCENE_PROMPT
        body = {
            "model": self.model,
            "stream": False,
            "think": False,                    # answer directly; thinking adds seconds
            "format": ANSWER_SCHEMA,
            "keep_alive": "30m",               # keep the model loaded between questions
            "options": {"temperature": 0, "num_ctx": 8192},
            "messages": [{"role": "user", "content": prompt,
                          "images": [base64.b64encode(jpeg).decode()]}],
        }
        start = time.time()
        try:
            request = urllib.request.Request(self.chat_url, data=json.dumps(body).encode(),
                                             headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(request, timeout=180) as reply:
                answer = json.loads(json.load(reply)["message"]["content"])
            answer = tidy({k: answer[k] for k in ANSWER_SCHEMA["required"]})
        except Exception as error:   # Ollama down, model missing, or a bad answer
            return {"error": f"Vision model failed: {error}",
                    "seconds": round(time.time() - start, 2)}
        answer["seconds"] = round(time.time() - start, 2)
        return answer


def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()