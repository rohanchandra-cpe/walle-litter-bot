"""The simulated ESP32-CAM: serves the Gazebo camera at http://localhost:8080/capture.

The robot's camera is an HTTP photo server (firmware/esp32cam_camera), and
pilot/range_trial.py, pilot/session_check.py and vision/camera_node.py download
photos from ranging.camera_url. This node serves the latest /sim/camera frame the same
way, as a JPEG, so those scripts run unchanged in Gazebo. Query strings such as
?flash=1 are accepted and ignored.

    GET /capture   the latest frame as image/jpeg (503 before the first frame)
    GET /status    JSON: frames received, latest frame's simulation time

    python3 sim_camera_server.py --ros-args -p port:=8080
"""

import io
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from PIL import Image as PilImage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from profiles import load_profile  # noqa: E402


class SimCamera(Node):
    def __init__(self):
        super().__init__("sim_camera_server")
        self.declare_parameter("port", 8080)
        self.declare_parameter("profile", "pilot_firmware")
        camera = load_profile(self.get_parameter("profile").value)["camera"]
        self.latency = float(camera["capture_latency_s"])
        self.quality = int(camera["jpeg_quality"])
        self.lock = threading.Lock()
        self.frame = None
        self.frames = 0
        self.stamp = 0.0
        self.create_subscription(Image, "/sim/camera", self.on_image, qos_profile_sensor_data)
        port = int(self.get_parameter("port").value)
        server = ThreadingHTTPServer(("0.0.0.0", port), self.handler())
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.get_logger().info(f"Simulated ESP32-CAM: http://localhost:{port}/capture")

    def on_image(self, msg):
        with self.lock:
            self.frame = msg
            self.frames += 1
            self.stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def jpeg(self):
        with self.lock:
            msg = self.frame
        if msg is None or msg.encoding not in ("rgb8", "bgr8"):
            return None
        image = PilImage.frombytes("RGB", (msg.width, msg.height), bytes(msg.data))
        if msg.encoding == "bgr8":
            r, g, b = image.split()
            image = PilImage.merge("RGB", (b, g, r))
        buffer = io.BytesIO()
        image.save(buffer, format="JPEG", quality=self.quality)
        return buffer.getvalue()

    def status(self):
        with self.lock:
            return {"frames": self.frames, "stamp": self.stamp}

    def handler(self):
        camera = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                path = self.path.split("?")[0]
                if path == "/capture":
                    time.sleep(camera.latency)
                    data = camera.jpeg()
                    if data is None:
                        self.reply(503, "text/plain", b"no camera frame yet\n")
                    else:
                        self.reply(200, "image/jpeg", data)
                elif path == "/status":
                    self.reply(200, "application/json", json.dumps(camera.status()).encode())
                else:
                    self.reply(200, "text/plain", b"Simulated ESP32-CAM: GET /capture\n")

            def reply(self, code, kind, body):
                self.send_response(code)
                self.send_header("Content-Type", kind)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *args):
                pass

        return Handler


def main():
    rclpy.init()
    node = SimCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
