"""Camera tilt servo (ROADMAP Phase 4): degrees <-> pulse widths, and commanding it over ROS.

The Pico has no servo topic (its micro-ROS library allows only 2 subscriptions): the pulse
width rides on /pico/imu/std as a 4th value ([0, 0, 0, us]: a value of 0 keeps each IMU
setting), and the pulse being output comes back as the 5th value of /pico/wheel_ticks. The
firmware slews ~1000 us/s, so a move is done when the readback equals the target.

    cal = Calibration.load(sim=False)          # config/servo.yaml
    tilt = Tilt(node, cal)                     # any rclpy node that is being spun
    tilt.set(15.0)                             # blocks until there (+ settle_s); TiltError if not
    tilt.level()
"""

import os
import threading
import time

import yaml

APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SERVO_CONFIG = os.path.join(APP_DIR, "config", "servo.yaml")
FIRMWARE_MIN_US, FIRMWARE_MAX_US = 600, 2400      # servo.h hard clamp


class TiltError(RuntimeError):
    """The servo didn't get there (no readback: old firmware, or the Pico is down)."""


def load_config(path=SERVO_CONFIG):
    with open(path) as f:
        return yaml.safe_load(f)


class Calibration:
    def __init__(self, level_us=1500.0, us_per_deg=10.0, min_deg=-10.0, max_deg=30.0,
                 lens_from_pivot_m=(0.0, 0.0), settle_s=0.6, readback_timeout_s=4.0,
                 level_tolerance_us=3, up_deg=15.0):
        if not us_per_deg:
            raise ValueError("servo.yaml us_per_deg must not be 0.")
        if min_deg > 0 or max_deg < 0:
            raise ValueError("servo.yaml min_deg..max_deg must include 0 (level).")
        self.level_us, self.us_per_deg = float(level_us), float(us_per_deg)
        self.min_deg, self.max_deg = float(min_deg), float(max_deg)
        self.lens_from_pivot_m = tuple(float(v) for v in lens_from_pivot_m)
        self.settle_s, self.readback_timeout_s = float(settle_s), float(readback_timeout_s)
        self.level_tolerance_us = float(level_tolerance_us)
        self.up_deg = float(up_deg)

    @classmethod
    def from_config(cls, cfg, sim=False):
        c = {k: v for k, v in cfg.items() if k != "sim"}
        if sim:
            c.update(cfg.get("sim", {}))
        keys = ("level_us", "us_per_deg", "min_deg", "max_deg", "lens_from_pivot_m",
                "settle_s", "readback_timeout_s", "level_tolerance_us", "up_deg")
        return cls(**{k: c[k] for k in keys if k in c})

    @classmethod
    def load(cls, sim=False, path=SERVO_CONFIG):
        return cls.from_config(load_config(path), sim)

    def clamp_deg(self, deg):
        return max(self.min_deg, min(self.max_deg, float(deg)))

    def deg_to_us(self, deg):
        """Pulse width for a tilt (clamped to min_deg..max_deg and the firmware's range)."""
        us = self.level_us + self.us_per_deg * self.clamp_deg(deg)
        return int(round(max(FIRMWARE_MIN_US, min(FIRMWARE_MAX_US, us))))

    def us_to_deg(self, us):
        return (float(us) - self.level_us) / self.us_per_deg

    def is_level(self, us):
        return us is not None and abs(float(us) - self.level_us) <= self.level_tolerance_us


class Tilt:
    """Command the tilt through a running rclpy node (it must be spun by someone)."""

    def __init__(self, node, cal):
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import Float32MultiArray, Int32MultiArray
        self.node, self.cal = node, cal
        self._msg_type = Float32MultiArray
        self.pub = node.create_publisher(Float32MultiArray, "/pico/imu/std", 10)
        self.lock = threading.Lock()
        self.readback_us = None          # pulse being output, from wheel_ticks[4]
        self.readback_t = None
        self.ticks_seen = False
        self.target_us = None
        self.log = []                    # every set(): for summary.json
        node.create_subscription(Int32MultiArray, "/pico/wheel_ticks", self._on_ticks,
                                 qos_profile_sensor_data)

    def _on_ticks(self, msg):
        with self.lock:
            self.ticks_seen = True
            if len(msg.data) >= 5:
                self.readback_us = int(msg.data[4])
                self.readback_t = time.monotonic()

    def readback(self):
        with self.lock:
            return self.readback_us

    def has_servo(self):
        with self.lock:
            return self.readback_us is not None

    def angle_deg(self):
        """The tilt the servo is at now (from the readback), or None."""
        us = self.readback()
        return None if us is None else self.cal.us_to_deg(us)

    def is_level(self):
        """True when the readback says level; also True with no readback at all (old
        firmware: there is no servo to have left tilted)."""
        us = self.readback()
        return us is None or self.cal.is_level(us)

    def send_us(self, us):
        msg = self._msg_type()
        msg.data = [0.0, 0.0, 0.0, float(us)]
        self.pub.publish(msg)

    def set(self, deg, settle=True):
        """Tilt to deg (clamped), wait for the readback to get there, then settle_s.
        Returns the angle reached. Raises TiltError if the readback never arrives."""
        us = self.cal.deg_to_us(deg)
        self.target_us = us
        t0 = time.monotonic()
        end = t0 + self.cal.readback_timeout_s + abs(
            us - (self.readback() or self.cal.level_us)) / 1000.0
        next_send = 0.0
        entry = {"deg": round(self.cal.us_to_deg(us), 2), "us": us}
        while True:
            now = time.monotonic()
            if now >= next_send:         # resend: /pico/imu/std is reliable, but cheap
                self.send_us(us)
                next_send = now + 0.5
            rb = self.readback()
            if rb is not None and abs(rb - us) <= 1:
                break
            if now > end:
                entry["error"] = ("no servo readback on /pico/wheel_ticks (firmware "
                                  "without the servo?)" if rb is None else
                                  f"servo stuck at {rb} us, asked {us} us")
                self.log.append(entry)
                raise TiltError(entry["error"])
            time.sleep(0.02)
        entry["seconds"] = round(time.monotonic() - t0, 2)
        if settle:
            time.sleep(self.cal.settle_s)
        self.log.append(entry)
        return self.cal.us_to_deg(self.readback())

    def level(self, settle=False):
        return self.set(0.0, settle=settle)
