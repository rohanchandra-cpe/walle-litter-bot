"""Ground-truth stand-in for the detector in Gazebo (ROADMAP Phase 3, decided 2026-11-19).

YOLO-World sees nothing in the park world (flat-colored boxes and cylinders; no prompt scored
above 0.05). In sim the follower uses this instead: it projects the TRUE bins and items (the
world's layout + the robot's Gazebo pose) through the camera and returns boxes in the
detector's format. Everything after the model (centering, on_bin / around_bin, obj_xy,
detections.json, the memory store) is the real code, checked against ground truth.
The model's accuracy is judged on real ESP32-CAM photos (eval_photos.py), not here.

    det = TruthDetector(world_layout, pose_fn=lambda: node.park_pose("ground_truth"),
                        tilt_fn=lambda: tilt.angle_deg())   # Phase 4: the camera's tilt
    det.detect(jpeg_bytes)       # same result shape as Detector.detect
"""

import io
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import park  # noqa: E402
from detect.detector import load_config  # noqa: E402

MIN_PX = 3            # a box narrower or shorter than this (after clipping) is not "seen"


def camera_settings(cfg):
    """park.yaml robot.camera with detect.yaml sim.camera on top (the walle-sim model)."""
    cam = dict(park.load_yaml(park.PARK_FILE)["robot"]["camera"])
    cam.update(cfg.get("sim", {}).get("camera", {}))
    return cam


def box_corners(center, half, yaw):
    """8 corners of a box with half-extents (along its x, y, z) turned by yaw about z."""
    cx, cy, cz = center
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                x, y, z = sx * half[0], sy * half[1], sz * half[2]
                out.append((cx + c * x - s * y, cy + s * x + c * y, cz + z))
    return out


def item_half_extents(item, model):
    if model["shape"] == "sphere":
        r = model["radius_m"]
        return (r, r, r)
    r, half_len = model["radius_m"], model["length_m"] / 2.0
    # lying: the cylinder's axis is horizontal, along its yaw (park_world rolls it 90 deg)
    return (r, half_len, r) if item.get("pose") == "lying" else (r, r, half_len)


class TruthDetector:
    def __init__(self, layout, pose_fn, cfg=None, hide_cut_bins=False, tilt_fn=None,
                 lens_from_pivot_m=(0.0, 0.0), hide_obstacle=False):
        """hide_cut_bins: miss a bin whose box runs off the frame's left or right side, as
        YOLO-World did on the robot (run 20261121_035910, bin 2 scored 0.02).
        tilt_fn: the camera's tilt in deg (+ = up) when the photo is taken, or None (level).
        lens_from_pivot_m: [forward, up] of the lens from the tilt axis (servo.yaml sim).
        hide_obstacle: never report the route obstacle (Phase 5: a detector miss, so only
        the sonar can find it)."""
        self.cfg = cfg or load_config()
        self.hide_obstacle = hide_obstacle
        self.hide_cut_bins = hide_cut_bins
        self.layout, self.pose_fn = layout, pose_fn
        self.tilt_fn = tilt_fn
        self.lens_from_pivot_m = tuple(lens_from_pivot_m or (0.0, 0.0))
        self.last_tilt_deg = 0.0
        self.cam = camera_settings(self.cfg)
        self.models = park.load_yaml(park.PARK_FILE)["items"]
        self.model_name = "ground_truth"
        self.source = "sim"

    def lens(self, tilt_deg):
        """(forward_m, height_m) of the lens at this tilt: it turns about the tilt axis."""
        df, dz = self.lens_from_pivot_m
        t = math.radians(tilt_deg)
        px, pz = self.cam["forward_m"] - df, self.cam["height_m"] - dz
        return (px + df * math.cos(t) - dz * math.sin(t),
                pz + df * math.sin(t) + dz * math.cos(t))

    def project(self, pts, pose, w, h, tilt_deg=0.0):
        """Pixel box [x1, y1, x2, y2] of 3-D points seen from pose (x, y, heading rad) with
        the camera tilted tilt_deg up, clipped to the frame, and the nearest point's
        distance; None if out of view."""
        x, y, hd = pose
        fx = (w / 2.0) / math.tan(math.radians(self.cam["horizontal_fov_deg"]) / 2.0)
        fy = (h / 2.0) / math.tan(math.radians(self.cam["vertical_fov_deg"]) / 2.0)
        lens_f, lens_h = self.lens(tilt_deg)
        lens_l = float(self.cam.get("left_m", 0.0))           # left of the center line
        cx = x + lens_f * math.cos(hd) - lens_l * math.sin(hd)
        cy = y + lens_f * math.sin(hd) + lens_l * math.cos(hd)
        c, s = math.cos(hd), math.sin(hd)
        ct, st = math.cos(math.radians(tilt_deg)), math.sin(math.radians(tilt_deg))
        us, vs, near = [], [], math.inf
        for px, py, pz in pts:
            ahead = c * (px - cx) + s * (py - cy)     # level frame: ahead, left, up
            left = -s * (px - cx) + c * (py - cy)
            up = pz - lens_h
            fwd = ahead * ct + up * st                # along the tilted optical axis
            up_t = -ahead * st + up * ct
            if fwd < 0.02:
                return None                     # partly behind the camera: skip
            us.append(w / 2.0 - fx * left / fwd)
            vs.append(h / 2.0 - fy * up_t / fwd)
            near = min(near, ahead)
        box = [max(0.0, min(us)), max(0.0, min(vs)), min(float(w), max(us)),
               min(float(h), max(vs))]
        if box[2] - box[0] < MIN_PX or box[3] - box[1] < MIN_PX:
            return None
        return [round(v, 1) for v in box], near

    def detect(self, image, keep_low=False):
        t0 = time.time()
        if isinstance(image, (bytes, bytearray)):
            from PIL import Image
            image = Image.open(io.BytesIO(image))
        w, h = image.size
        pose = self.pose_fn()
        tilt = (self.tilt_fn() if self.tilt_fn else None) or 0.0
        self.last_tilt_deg = tilt
        dets = []
        bins = []
        for b in self.layout["bins"]:
            yaw = math.radians(b.get("yaw_deg", 0.0))
            bh = b["height_m"]
            if b.get("shape") == "box":
                half = (b["size_m"][1] / 2.0, b["size_m"][0] / 2.0, bh / 2.0)
            else:
                half = (b["radius_m"],) * 2 + (bh / 2.0,)
            seen = self.project(box_corners((*b["position"], bh / 2.0), half, yaw), pose, w, h,
                                tilt)
            if seen:
                bins.append((seen, b))          # still hides what is behind it
                edge = self.cfg.get("centering", {}).get("edge_px", 4)
                if self.hide_cut_bins and (seen[0][0] <= edge or seen[0][2] >= w - edge):
                    continue
                dets.append({"kind": "bin", "label": self.cfg["bin"]["label"],
                             "category": None, "prompt": "ground truth", "conf": 1.0,
                             "bbox": seen[0], "truth": b["name"]})
        placed = [(group, it) for b in self.layout["bins"]
                  for group in ("mound", "spill", "litter") for it in b.get(group, [])]
        if self.layout.get("obstacle") and not self.hide_obstacle:
            placed.append(("obstacle", self.layout["obstacle"]))   # Phase 5
        for group, it in placed:
            model = self.models[it["item"]]
            pts = box_corners(it["position"], item_half_extents(it, model),
                              math.radians(it.get("yaw_deg", 0.0)))
            seen = self.project(pts, pose, w, h, tilt)
            if not seen:
                continue
            seen = self.occlude(seen, bins, min(p[2] for p in pts))
            if seen is None:
                continue
            dets.append({"kind": "item", "label": it["item"],
                         "category": self.cfg["items"].get(it["item"], {})
                         .get("category", "unknown"),
                         "prompt": "ground truth", "conf": 1.0, "bbox": seen[0],
                         "truth": it["name"], "truth_group": group})
        return {"detections": dets, "seconds": round(time.time() - t0, 3), "size": [w, h],
                "model": self.model_name, "source": self.source, "tilt_deg": round(tilt, 2),
                **({"low_conf": []} if keep_low else {})}

    @staticmethod
    def occlude(seen, bins, bottom_z):
        """What a detector would box of an item with bins in front of it: None if it is wholly
        behind one; cut off at the bin's top edge if it shows only above it. An item resting
        on top of a bin (bottom at or above the rim) is seen from below the rim, so nothing of
        it shows under the bin's top edge, even where it overhangs the near side."""
        (x1, y1, x2, y2), near = seen
        for (bb, bnear), b in bins:
            on_top = bottom_z >= b["height_m"] - 0.01
            if (bnear >= near and not on_top) or x2 <= bb[0] or x1 >= bb[2]:
                continue                       # not behind this bin
            inside_x = x1 >= bb[0] and x2 <= bb[2]
            if inside_x and y1 >= bb[1] and y2 <= bb[3]:
                return None
            if inside_x and y1 < bb[1] < y2:
                y2 = bb[1]
        if y2 - y1 < MIN_PX:
            return None
        return [x1, y1, x2, round(y2, 1)], near
