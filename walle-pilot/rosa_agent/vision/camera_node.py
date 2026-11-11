"""Camera node: photos in, /camera/image/compressed out.

Real robot (source:=http)                Simulation (source:=sim)
-------------------------                ----------------------------------------
downloads http://CAMERA_IP/capture  <-   ESP32-CAM photo server (esp32cam_camera.ino)
                                    <-   /sim/camera (Gazebo camera, raw pixels),
                                         compressed to JPEG like the ESP32-CAM

Publishes   /camera/image/compressed   sensor_msgs/CompressedImage   JPEG, `rate` per second

Started by launch/vision.launch.py. To run it by itself:
    python3 /workspace/walle-pilot/rosa_agent/vision/camera_node.py --ros-args -p url:=http://192.168.1.50/capture
"""

import io
import threading
import time
import urllib.request

import rclpy
from PIL import Image as PilImage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        self.declare_parameter("source", "http")        # "http" (ESP32-CAM) or "sim" (Gazebo)
        self.declare_parameter("url", "http://192.168.1.50/capture")
        self.declare_parameter("rate", 2.0)             # photos per second
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.source = p("source")
        self.url = p("url")
        self.period = 1.0 / float(p("rate"))
        self.pub = self.create_publisher(CompressedImage, "/camera/image/compressed",
                                         qos_profile_sensor_data)
        self.photos = 0
        self.failures = 0

        if self.source == "sim":
            self.latest = None
            self.create_subscription(Image, "/sim/camera", self.on_sim_image,
                                     qos_profile_sensor_data)
            self.create_timer(self.period, self.publish_sim_image)
            self.get_logger().info("Camera node ready: /sim/camera -> /camera/image/compressed")
        else:
            threading.Thread(target=self.http_loop, daemon=True).start()
            self.get_logger().info(f"Camera node ready: {self.url} -> /camera/image/compressed")

    def publish_jpeg(self, jpeg, stamp):
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_link"
        msg.format = "jpeg"
        msg.data = jpeg
        self.pub.publish(msg)
        self.photos += 1
        if self.photos == 1:
            self.get_logger().info(f"First photo published ({len(jpeg) // 1024} KB)")

    # ----- real robot: download photos from the ESP32-CAM -----
    def http_loop(self):
        while rclpy.ok():
            start = time.time()
            try:
                with urllib.request.urlopen(self.url, timeout=5.0) as reply:
                    jpeg = reply.read()
                self.publish_jpeg(jpeg, self.get_clock().now().to_msg())
                if self.failures:
                    self.get_logger().info("Camera reachable again")
                self.failures = 0
            except Exception as error:   # network errors must not stop the node
                self.failures += 1
                if self.failures in (1, 10) or self.failures % 60 == 0:
                    self.get_logger().warn(f"No photo from {self.url}: {error}")
            time.sleep(max(0.0, self.period - (time.time() - start)))

    # ----- simulation: compress Gazebo's raw camera image to JPEG -----
    def on_sim_image(self, msg: Image):
        self.latest = msg

    def publish_sim_image(self):
        msg, self.latest = self.latest, None
        if msg is None or msg.encoding != "rgb8":
            return
        image = PilImage.frombytes("RGB", (msg.width, msg.height), bytes(msg.data))
        buffer = io.BytesIO()
        image.save(buffer, format="JPEG", quality=85)
        self.publish_jpeg(buffer.getvalue(), msg.header.stamp)


def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()