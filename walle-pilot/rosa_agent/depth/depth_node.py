"""Depth node: distance to objects from one camera photo, in meters (Depth Anything V2).

Subscribes  /camera/image/compressed   sensor_msgs/CompressedImage   from camera_node.py
            /depth/ask                 std_msgs/String   JSON {"id": "...", "target": "red ball"}
                                                         (target "" = distances ahead only)
Publishes   /depth/result              std_msgs/String   JSON with the answer (below)

For every question it waits for a photo taken AFTER the question arrived (so the
robot has stopped moving), runs the model on the CPU, and publishes:
    {"id", "target", "ahead_m", "left_m", "right_m", "ahead_raw_m", ...,
     "target_found", "target_m", "target_raw_m", "target_size_m", "bearing_deg", "scale",
     "power",
     "model", "seconds", "image", "depth_image", "error"}
Each analyzed photo is saved, with its depth map beside it, in
/workspace/depth/results/images/ for your report.

Started by launch/depth.launch.py. Settings: model_size (small, base, or large),
input_size (pixels; smaller is faster), hfov_deg (the camera's horizontal field of view),
object_diameter_m (real diameter of the round target, to also report a distance from its size).
"""

import io
import json
import os
import queue
import threading
import time

import rclpy
from PIL import Image
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

import depth_estimator

RESULTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results", "images")
FRESH_PHOTO_TIMEOUT = 5.0     # s to wait for a photo taken after the question


def summary(result):
    if result.get("error"):
        return result["error"]
    text = f"ahead {result['ahead_m']:.2f} m"
    if result["target"]:
        text += (f", {result['target']} {result['target_m']:.2f} m" if result["target_found"]
                 else f", {result['target']} not found")
    return text


class DepthNode(Node):
    def __init__(self):
        super().__init__("depth_node")
        self.declare_parameter("model_size", "small")
        self.declare_parameter("input_size", 364)
        self.declare_parameter("hfov_deg", 66.0)
        self.declare_parameter("object_diameter_m", 0.0)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.hfov = float(p("hfov_deg"))
        self.diameter = float(p("object_diameter_m"))
        self.size, self.input_size = p("model_size"), int(p("input_size"))
        self.lock = threading.Lock()
        self.latest = None
        self.questions = queue.Queue()

        self.get_logger().info(f"Loading the {self.size} depth model (the first time, this "
                               "downloads it) ...")
        self.estimator = depth_estimator.DepthEstimator(self.size, self.input_size)
        # The first run is slower than the rest; do it now, not on your first question
        self.estimator.estimate(Image.new("RGB", (640, 480), (128, 128, 128)))

        self.result_pub = self.create_publisher(String, "/depth/result", 10)
        self.create_subscription(CompressedImage, "/camera/image/compressed", self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(String, "/depth/ask", self.on_ask, 10)
        threading.Thread(target=self.worker, daemon=True).start()
        os.makedirs(RESULTS, exist_ok=True)
        scale, power = depth_estimator.load_calibration()
        if (scale, power) == (1.0, 1.0):
            calibration = "not calibrated"
        else:
            calibration = f"calibration: {scale:.3f} x distance ** {power:.2f}"
        self.get_logger().info(f"Depth node ready: {self.estimator.name}, "
                               f"input {self.input_size} px, {calibration}")

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
            result = {"id": request["id"], "target": request["target"],
                      "model": f"{self.size} ({self.input_size} px)"}
            try:
                photo = self.fresh_photo(asked_at)
                if photo is None:
                    result["error"] = "No new camera photo. Is the camera node running?"
                else:
                    result.update(self.measure(photo, request))
            except Exception as error:   # never let one bad question stop the node
                result["error"] = f"Depth node error: {error}"
            self.result_pub.publish(String(data=json.dumps(result)))
            self.get_logger().info(f"[{result.get('seconds', 0):.1f} s] "
                                   f"{request['target'] or 'ahead'}: {summary(result)}")

    def measure(self, photo, request):
        image = Image.open(io.BytesIO(bytes(photo.data))).convert("RGB")
        start = time.time()
        depth = self.estimator.estimate(image)
        seconds = round(time.time() - start, 2)
        answer = depth_estimator.analyze(depth, image, request["target"], self.hfov,
                                         depth_estimator.load_calibration(), self.diameter)
        answer["seconds"] = seconds
        answer["image"], answer["depth_image"] = self.save(image, depth, answer, request["id"])
        return answer

    def save(self, image, depth, answer, request_id):
        os.makedirs(RESULTS, exist_ok=True)   # in case the folder was deleted
        stem = f"{time.strftime('%Y%m%d_%H%M%S')}_{request_id[-6:]}"
        image.save(os.path.join(RESULTS, stem + ".jpg"), quality=92)
        depth_estimator.depth_picture(image, depth, answer).save(
            os.path.join(RESULTS, stem + "_depth.jpg"), quality=90)
        base = os.path.join("depth", "results", "images", stem)
        return base + ".jpg", base + "_depth.jpg"


def main():
    rclpy.init()
    node = DepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
