"""RQ3: measure scheduled object placements with the HC-SR04 and with the camera + depth model.

Run inside the container while launch/pilot.launch.py is running (robot still):
    python3 pilot/range_trial.py --session 1      # scheduled placements, one after another
    python3 pilot/range_trial.py --check          # one reading of both, nothing logged
    python3 pilot/range_trial.py --calibrate      # depth calibration object (then fit_depth.py)
    python3 pilot/range_trial.py --hfov-check     # implied camera field of view from a known angle

For each placement: put the object where the script says, measure the true range
(horizontal, from the shared sensor reference plane to the object's nearest
surface), type it in, and the script takes both readings in the scheduled order.
It saves the photo, the color mask, the depth map, each ping's status, and timing.
A failed reading is logged as invalid: no range is made up for it.
"""

import argparse
import io
import math
import os
import statistics
import sys
import time
import urllib.request

import numpy as np
from PIL import Image

from common import (DEPTH_CALIBRATION_LOG, PILOT_DIR, RANGE_DIR, RANGE_LOG, RANGE_SCHEDULE,
                    append_row, ask, ask_float, ask_yes, commit_label, load_config,
                    next_scheduled, read_rows, session_value, utc_now)
from robot_io import Robot

sys.path.insert(0, os.path.join(PILOT_DIR, "depth"))
import depth_estimator  # noqa: E402

FIELDS = [
    "placement_id", "attempt", "session_id", "order", "object", "color", "distance_m",
    "angle_deg", "first_sensor", "timestamp_utc", "commit", "operator", "measurer", "temp_c",
    "true_range_m",
    "us_valid", "us_range_m", "us_valid_pings", "us_ping_status", "us_ping_ranges_m",
    "us_latency_ms",
    "cam_valid", "cam_failure", "cam_raw_z_m", "cam_z_m", "cam_bearing_deg", "cam_range_m",
    "cam_pixels", "cam_capture_ms", "cam_inference_ms", "cam_latency_ms",
    "depth_scale", "depth_power", "hfov_deg", "model", "input_size", "tensor_shape",
    "external_invalid", "invalid_reason", "notes", "photo", "mask", "depth_map",
]
CALIBRATION_FIELDS = ["timestamp_utc", "commit", "true_m", "target_found", "target_raw_m",
                      "bearing_deg", "model", "photo"]
STATUS = {-1: "timeout", -2: "no_response", 0: "no_ping"}


def speed_of_sound(temp_c):
    return 331.3 + 0.606 * temp_c


# ===== ultrasonic =====
def read_ultrasonic(robot, cfg, temp_c):
    ranging = cfg["ranging"]
    t0 = time.monotonic()
    pings = robot.collect_pings(ranging["pings"], ranging["acquire_timeout_s"])
    latency = 1000.0 * (time.monotonic() - t0)
    pings += [(None, None)] * (ranging["pings"] - len(pings))   # never arrived
    c = speed_of_sound(temp_c)
    statuses, ranges = [], []
    for _, echo in pings:
        if echo is None:
            statuses.append("lost")
        elif echo > 0:
            r = c * echo * 1e-6 / 2.0 - ranging["ultrasonic_offset_m"]
            statuses.append(f"ok:{echo}")
            ranges.append(r)
        else:
            statuses.append(STATUS.get(echo, f"code{echo}"))
    valid = len(ranges) >= ranging["min_valid_pings"]
    return {"us_valid": valid, "us_range_m": statistics.median(ranges) if valid else "",
            "us_valid_pings": len(ranges), "us_ping_status": "|".join(statuses),
            "us_ping_ranges_m": ";".join(f"{r:.4f}" for r in ranges),
            "us_latency_ms": latency}


# ===== camera + depth =====
class Camera:
    def __init__(self, cfg):
        ranging = cfg["ranging"]
        self.url, self.timeout = ranging["camera_url"], ranging["camera_timeout_s"]
        self.hfov = ranging["hfov_deg"]
        self.size, self.input_size = ranging["depth_model"], ranging["input_size"]
        print(f"Loading Depth Anything V2 ({self.size}, {self.input_size} px) on the CPU...")
        self.estimator = depth_estimator.DepthEstimator(self.size, self.input_size)
        self.calibration = depth_estimator.load_calibration()
        self.tensor_shape = ""
        self.estimator.estimate(Image.new("RGB", (640, 480), (128, 128, 128)))   # warm-up

    def photo(self):
        with urllib.request.urlopen(self.url, timeout=self.timeout) as reply:
            return Image.open(io.BytesIO(reply.read())).convert("RGB")

    def read(self, color):
        out = {"cam_valid": False, "cam_failure": ""}
        t0 = time.monotonic()
        try:
            image = self.photo()
        except OSError as error:
            out.update(cam_failure=f"capture_error: {error}",
                       cam_latency_ms=1000.0 * (time.monotonic() - t0))
            return out, None, None, None
        t1 = time.monotonic()
        depth = self.estimator.estimate(image)
        t2 = time.monotonic()
        answer = depth_estimator.analyze(depth, image, color, self.hfov, self.calibration)
        mask = depth_estimator.find_color(image, color)
        t3 = time.monotonic()
        out.update(cam_capture_ms=1000.0 * (t1 - t0), cam_inference_ms=1000.0 * (t2 - t1),
                   cam_latency_ms=1000.0 * (t3 - t0))
        if not answer.get("target_found"):
            out["cam_failure"] = "no_mask"
            return out, image, depth, mask
        bearing = answer["bearing_deg"]
        out.update(cam_valid=True, cam_raw_z_m=answer["target_raw_m"], cam_z_m=answer["target_m"],
                   cam_bearing_deg=bearing,
                   cam_range_m=answer["target_m"] / math.cos(math.radians(bearing)),
                   cam_pixels=answer["target_pixels"])
        return out, image, depth, mask

    def describe(self):
        return {"depth_scale": self.calibration[0], "depth_power": self.calibration[1],
                "hfov_deg": self.hfov, "model": depth_estimator.MODELS[self.size],
                "input_size": self.input_size,
                "tensor_shape": f"{self.input_size}x{self.input_size}"}


def save_outputs(stem, image, depth, mask):
    os.makedirs(RANGE_DIR, exist_ok=True)
    names = {"photo": "", "mask": "", "depth_map": ""}
    if image is None:
        return names
    base = os.path.join(RANGE_DIR, stem)
    image.save(base + "_photo.jpg", quality=95)
    names["photo"] = os.path.basename(base + "_photo.jpg")
    np.save(base + "_depth.npy", depth.astype(np.float16))
    names["depth_map"] = os.path.basename(base + "_depth.npy")
    if mask is not None:
        Image.fromarray((mask * 255).astype(np.uint8)).save(base + "_mask.png")
        names["mask"] = os.path.basename(base + "_mask.png")
    depth_estimator.depth_picture(image, depth).save(base + "_view.jpg", quality=85)
    return names


def session_temperature(session, cfg):
    value = session_value(session, "temp_c") if session else None
    return float(value) if value not in (None, "") else float(cfg["ranging"]["default_temp_c"])


# ===== modes =====
def run_session(robot, camera, cfg, session):
    if not os.path.exists(RANGE_SCHEDULE):
        raise SystemExit("No schedule yet. Run: python3 pilot/make_schedule.py")
    if session_value(session, "session_id") is None:
        raise SystemExit(f"Run the checklist first: python3 pilot/session_check.py "
                         f"--session {session}")
    if camera.calibration == (1.0, 1.0):
        print("WARNING: the depth model is not calibrated (config/depth_calibration.json). "
              "Run --calibrate and depth/fit_depth.py before main trials.")
    temp_c = session_temperature(session, cfg)
    people = (session_value(session, "operator", ""), session_value(session, "measurer", ""))
    schedule = read_rows(RANGE_SCHEDULE)
    while True:
        placement, attempt = next_scheduled(session, schedule, read_rows(RANGE_LOG),
                                            "placement_id")
        if placement is None:
            print(f"\nSession {session}: every placement is done.")
            return
        print(f"\n=== {placement['placement_id']} (attempt {attempt}) ===")
        print(f"  Place the {placement['object'].replace('_', ' ')} at "
              f"{placement['distance_m']} m on the {placement['angle_deg']} deg ray.")
        true_range = ask_float("  True range: reference plane to nearest surface (m): ")
        input("  Keep clear of the sensors, then press Enter to read...")
        order = (["ultrasonic", "camera"] if placement["first_sensor"] == "ultrasonic"
                 else ["camera", "ultrasonic"])
        results, outputs = {}, (None, None, None)
        for sensor in order:
            if sensor == "ultrasonic":
                results.update(read_ultrasonic(robot, cfg, temp_c))
            else:
                cam, *outputs = camera.read(placement["color"])
                results.update(cam)
        stem = f"{placement['placement_id']}_a{attempt}"
        files = save_outputs(stem, *outputs)

        invalid = ask_yes("  Was this placement invalidated by OUTSIDE interference "
                          "(someone in view, object moved, reference lost)?")
        row = {k: placement[k] for k in ("placement_id", "session_id", "order", "object",
                                         "color", "distance_m", "angle_deg", "first_sensor")}
        row.update(attempt=attempt, timestamp_utc=utc_now(), commit=commit_label(),
                   operator=people[0], measurer=people[1], temp_c=temp_c,
                   true_range_m=true_range, external_invalid=invalid,
                   invalid_reason=ask("  What happened: ") if invalid else "",
                   notes=ask("  Notes (Enter for none): ", allow_blank=True),
                   **results, **files, **camera.describe())
        append_row(RANGE_LOG, FIELDS, row)
        show(row, true_range)
        if not ask_yes("\nNext placement?"):
            return


def show(row, true_range):
    for key, label in (("us", "ultrasonic"), ("cam", "camera")):
        if row[f"{key}_valid"]:
            r = row[f"{key}_range_m"]
            print(f"  {label:10s} {r:.3f} m  (error {100 * (r - true_range):+.1f} cm, "
                  f"{100 * abs(r - true_range) / true_range:.1f} %)  "
                  f"{row[f'{key}_latency_ms']:.0f} ms")
        else:
            reason = row.get("cam_failure") if key == "cam" else row["us_ping_status"]
            print(f"  {label:10s} invalid ({reason})")


def run_check(robot, camera, cfg, color):
    temp_c = cfg["ranging"]["default_temp_c"]
    us = read_ultrasonic(robot, cfg, temp_c)
    cam, image, depth, mask = camera.read(color)
    print(f"Ultrasonic: {us['us_ping_status']}  -> "
          + (f"{us['us_range_m']:.3f} m" if us["us_valid"] else "invalid"))
    if cam["cam_valid"]:
        print(f"Camera ({color}): {cam['cam_range_m']:.3f} m, bearing {cam['cam_bearing_deg']:+.1f} "
              f"deg, {cam['cam_pixels']} px, {cam['cam_latency_ms']:.0f} ms "
              f"(inference {cam['cam_inference_ms']:.0f} ms)")
    else:
        print(f"Camera ({color}): invalid ({cam['cam_failure']})")
    if image is not None:
        save_outputs("check", image, depth, mask)
        print(f"Saved {RANGE_DIR}/check_*.jpg")


def run_calibration(camera, cfg):
    color = depth_estimator.parse_color(cfg["ranging"]["calibration_object"])
    print(f"Calibration object: {cfg['ranging']['calibration_object']}. Use 8 distances "
          "from about 0.35 to 1.85 m that are NOT test distances, on the 0 deg ray.")
    while True:
        true_m = ask_float("\nTrue distance to the object (m), or Enter to finish: ",
                           allow_blank=True)
        if true_m == "":
            print("Now fit: python3 depth/fit_depth.py")
            return
        cam, image, depth, mask = camera.read(color)
        stem = f"calib_{len(read_rows(DEPTH_CALIBRATION_LOG)) + 1:02d}"
        files = save_outputs(stem, image, depth, mask)
        append_row(DEPTH_CALIBRATION_LOG, CALIBRATION_FIELDS, {
            "timestamp_utc": utc_now(), "commit": commit_label(), "true_m": true_m,
            "target_found": "True" if cam["cam_valid"] else "False",
            "target_raw_m": cam.get("cam_raw_z_m", ""), "bearing_deg": cam.get("cam_bearing_deg", ""),
            "model": depth_estimator.MODELS[camera.size], "photo": files["photo"]})
        print(f"  model says {cam['cam_raw_z_m']:.3f} m" if cam["cam_valid"]
              else f"  object not found ({cam['cam_failure']})")


def run_hfov_check(camera, cfg):
    color = depth_estimator.parse_color(cfg["ranging"]["calibration_object"])
    angle = ask_float(f"Place the {cfg['ranging']['calibration_object']} on a ray at a known "
                      "angle (deg, + = left of the sensor axis). Angle: ")
    cam, image, _, _ = camera.read(color)
    if not cam["cam_valid"]:
        raise SystemExit(f"Object not found ({cam['cam_failure']}).")
    mask = depth_estimator.find_color(image, color)
    cx = float(np.nonzero(mask)[1].mean())
    offset = image.width / 2 - cx
    if abs(angle) < 1 or abs(offset) < 1:
        raise SystemExit("Use an angle of at least 10 deg so the object is off center.")
    focal = offset / math.tan(math.radians(angle))
    hfov = math.degrees(2 * math.atan(image.width / 2 / focal))
    print(f"Bearing with hfov {camera.hfov} deg: {cam['cam_bearing_deg']:+.1f} deg. "
          f"Implied horizontal field of view: {hfov:.1f} deg. Repeat at both sides and "
          "put the average into ranging.hfov_deg in config/pilot.yaml.")


def main():
    parser = argparse.ArgumentParser(description="RQ3 ranging placements")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--session", type=int)
    group.add_argument("--check", metavar="COLOR", nargs="?", const="red",
                       help="one reading of both sensors (default color: red)")
    group.add_argument("--calibrate", action="store_true")
    group.add_argument("--hfov-check", action="store_true")
    args = parser.parse_args()
    cfg = load_config()
    camera = Camera(cfg)
    if args.calibrate:
        run_calibration(camera, cfg)
        return
    if args.hfov_check:
        run_hfov_check(camera, cfg)
        return
    with Robot() as robot:
        if robot.wait_for(("ping",), timeout=5.0):
            print("WARNING: no ultrasonic data on /pico/wheel_ticks (is the pilot firmware "
                  "flashed?). Ultrasonic readings will be invalid.")
        if args.check:
            run_check(robot, camera, cfg, args.check)
        else:
            run_session(robot, camera, cfg, args.session)


if __name__ == "__main__":
    main()
