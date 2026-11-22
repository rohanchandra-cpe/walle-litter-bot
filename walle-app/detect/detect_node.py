"""Detection node: the detector on the camera image, asked over ROS (ROADMAP Phase 3).

Subscribes  /camera/image/compressed   sensor_msgs/CompressedImage   from camera_node.py
            /detect/ask                std_msgs/String   JSON {"id": "..."}
Publishes   /detect/result             std_msgs/String   JSON (below)

Same pattern as walle-pilot/vision/vision_node.py: each question waits for a photo taken
AFTER it arrived (so the robot has stopped), runs the detector, and publishes
    {"id", "detections": [{kind, label, category, prompt, conf, bbox}], "size", "seconds",
     "model", "source", "image", "error"}
The route follower doesn't use this node (it imports detect/detector.py directly); this is
for the ROSA agent (Phase 6) and for checking the detector live. It makes the same detector
as the follower and eval_photos.py (make_detector: config/detect.yaml `method`, default vlm =
YOLO-World bins + the Ollama model for items); `-p method:=yolo` for YOLO-World alone.

    python3 walle-app/detect/detect_node.py                         # config method
    python3 walle-app/detect/detect_node.py --ros-args -p source:=sim -p save_images:=true
"""

import json
import os
import queue
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from detect.detector import APP_DIR, make_detector  # noqa: E402

IMAGES = os.path.join(APP_DIR, "results", "detect", "node_images")
FRESH_PHOTO_TIMEOUT = 5.0     # s to wait for a photo taken after the question


class DetectNode(Node):
    def __init__(self):
        super().__init__("detect_node")
        self.declare_parameter("source", "robot")
        self.declare_parameter("save_images", False)
        self.declare_parameter("method", "")      # "" = config/detect.yaml method
        self.save_images = bool(self.get_parameter("save_images").value)
        self.detector = make_detector(source=self.get_parameter("source").value,
                                      method=self.get_parameter("method").value or None)
        self.lock = threading.Lock()
        self.latest = None
        self.questions = queue.Queue()
        self.result_pub = self.create_publisher(String, "/detect/result", 10)
        self.create_subscription(CompressedImage, "/camera/image/compressed", self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(String, "/detect/ask", self.on_ask, 10)
        threading.Thread(target=self.worker, daemon=True).start()
        self.get_logger().info(f"Detect node ready: {self.detector.model_name}, "
                               f"source {self.detector.source}")

    def on_image(self, msg):
        with self.lock:
            self.latest = msg

    def on_ask(self, msg):
        try:
            request = json.loads(msg.data)
            if not isinstance(request, dict):
                request = {}
        except json.JSONDecodeError:
            request = {}
        request.setdefault("id", str(int(time.time() * 1000)))
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
            result = {"id": request["id"], "model": self.detector.model_name,
                      "source": self.detector.source}
            try:
                photo = self.fresh_photo(asked_at)
                if photo is None:
                    result["error"] = "No new camera photo. Is the camera node running?"
                else:
                    data = bytes(photo.data)
                    if self.save_images:
                        result["image"] = self.save(data, request["id"])
                    result.update(self.detector.detect(data))
            except Exception as error:   # never let one bad question stop the node
                result["error"] = f"Detect node error: {error}"
            self.result_pub.publish(String(data=json.dumps(result)))
            found = ", ".join(d["label"] + (f" {d['conf']:.2f}" if d.get("conf") is not None else "")
                              for d in result.get("detections", []))
            self.get_logger().info(f"[{result.get('seconds', 0):.2f} s] "
                                   f"{result.get('error') or found or 'nothing'}")

    def save(self, data, request_id):
        os.makedirs(IMAGES, exist_ok=True)
        name = f"{time.strftime('%Y%m%d_%H%M%S')}_{request_id[-6:]}.jpg"
        with open(os.path.join(IMAGES, name), "wb") as f:
            f.write(data)
        return os.path.relpath(os.path.join(IMAGES, name), APP_DIR)


def main():
    rclpy.init()
    node = DetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
