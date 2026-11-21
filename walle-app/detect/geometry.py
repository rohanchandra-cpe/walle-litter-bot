"""Photo geometry for detections (ROADMAP Phase 3). Pure Python, no model, no ROS.

Conventions (the park's): angles in degrees, positive = to the LEFT (counter-clockwise).
Pixel u grows to the right, v grows down; the image is upright (the ESP32-CAM firmware flips
it). The camera is a pinhole with the field of view from park.yaml robot.camera, looking
level and straight ahead, height_m above the floor and forward_m ahead of the axle.
"""

import math


def focal_px(size_px, fov_deg):
    """Focal length in pixels for an image `size_px` wide (or tall) with that field of view."""
    return (size_px / 2.0) / math.tan(math.radians(fov_deg) / 2.0)


class Camera:
    def __init__(self, width=640, height=480, hfov_deg=60.0, vfov_deg=47.0, height_m=0.103,
                 forward_m=0.12):
        self.w, self.h = width, height
        self.fx, self.fy = focal_px(width, hfov_deg), focal_px(height, vfov_deg)
        self.height_m, self.forward_m = height_m, forward_m

    @classmethod
    def from_park(cls, cam, width=640, height=480):
        """From park.yaml robot.camera."""
        return cls(width, height, cam["horizontal_fov_deg"], cam["vertical_fov_deg"],
                   cam["height_m"], cam["forward_m"])

    def bearing_deg(self, u):
        """Bearing of pixel column u from the optical axis; positive = left."""
        return math.degrees(math.atan((self.w / 2.0 - u) / self.fx))

    def width_px(self, width_m, distance_m):
        """How wide something width_m wide looks at distance_m straight ahead."""
        return self.fx * width_m / max(distance_m, 1e-6)

    def ground_point(self, u, v):
        """Where the ray through pixel (u, v) meets the floor, in the ROBOT frame (x forward
        from the axle, y left), or None if the pixel is at or above the horizon."""
        down = (v - self.h / 2.0) / self.fy
        if down <= 1e-3:
            return None
        t = self.height_m / down                     # distance along the optical axis
        return (self.forward_m + t, t * (self.w / 2.0 - u) / self.fx)


def robot_to_park(pt, pose):
    """A robot-frame point into the park frame, given the pose (x, y, heading_deg)."""
    x, y, h = pose
    c, s = math.cos(math.radians(h)), math.sin(math.radians(h))
    return (x + c * pt[0] - s * pt[1], y + s * pt[0] + c * pt[1])


def cut_edges(bbox, cam, edge_px):
    """Which frame edges a box touches: a subset of {"left", "right", "top", "bottom"}."""
    x1, y1, x2, y2 = bbox
    out = set()
    if x1 <= edge_px:
        out.add("left")
    if x2 >= cam.w - edge_px:
        out.add("right")
    if y1 <= edge_px:
        out.add("top")
    if y2 >= cam.h - edge_px:
        out.add("bottom")
    return out


def bin_center_u(bbox, cam, edge_px, expected_px=None):
    """Pixel column of the bin's center. A box cut off at one side edge shows only part of the
    bin: then the center is half the bin's expected width in from its visible edge (or the
    box's own center, whichever is farther out, so a bin nearer than expected still pulls
    the turn the right way). Cut at both sides (the bin fills the frame): the box center."""
    x1, _, x2, _ = bbox
    mid = (x1 + x2) / 2.0
    cut = cut_edges(bbox, cam, edge_px)
    if expected_px and ("left" in cut) != ("right" in cut):
        if "left" in cut:
            return min(mid, x2 - expected_px / 2.0)
        return max(mid, x1 + expected_px / 2.0)
    return mid


def plausible_bin(d, cam, edge_px, expected_px=None, min_frac=0.5):
    """Whether a bin box is about the size the bin should look (expected_px = (width, height)
    in px at its expected distance). A box cut at a side edge is judged on its height only,
    one cut at the top on its width only. Robot run 20261121_000650: 16-46 px boxes (a bowl
    under a table, a cushion's edge) at conf 0.03-0.24 were taken for the ~250 px bin."""
    if not expected_px:
        return True
    x1, y1, x2, y2 = d["bbox"]
    cut = cut_edges(d["bbox"], cam, edge_px)
    w_ok = bool(cut & {"left", "right"}) or x2 - x1 >= min_frac * expected_px[0]
    h_ok = bool(cut & {"top", "bottom"}) or y2 - y1 >= min_frac * expected_px[1]
    return w_ok and h_ok


def pick_bin(bins, cam, expected_bearing_deg=0.0, expected_px=None, edge_px=4, min_frac=0.5):
    """The bin detection to center on: among boxes of a plausible size (plausible_bin), the
    most confident; ties (e.g. the sim's ground truth, all 1.0) go to the one nearest the
    expected bearing. Confidence first, because a bin moved off its configured spot is
    where the robot finds it, not where the config says (run 20261121_000650: a 0.03 box
    near the expected bearing beat the real 0.21 bin). None if there are none."""
    bins = [d for d in bins if plausible_bin(d, cam, edge_px, expected_px, min_frac)]
    if not bins:
        return None
    return min(bins, key=lambda d: (-(d.get("conf") or 0.0),
                                    abs(cam.bearing_deg((d["bbox"][0] + d["bbox"][2]) / 2.0)
                                        - expected_bearing_deg)))


def search_order(bin_boxes, cam, edge_px, expected_px, search_deg):
    """Bin centering found no bin: the headings to try, as offsets (deg, left +) from the
    configured bin heading: +search_deg and -search_deg. If some box (of any confidence) is
    cut off at a side edge and tall enough to be the bin, that side goes first: run
    20261121_035910's bin 2 was cut at the LEFT edge and scored 0.016-0.024."""
    left_first = True
    for d in sorted(bin_boxes, key=lambda d: -(d.get("conf") or 0.0)):
        cut = cut_edges(d["bbox"], cam, edge_px)
        side = cut & {"left", "right"}
        if len(side) == 1 and (not expected_px or
                               d["bbox"][3] - d["bbox"][1] >= 0.5 * expected_px[1]):
            left_first = "left" in side
            break
    s = abs(search_deg)
    return [s, -s] if left_first else [-s, s]


def item_place(item_bbox, bin_bbox, rim_visible, rim_margin_px, min_overlap):
    """on_bin if the item sits above the bin's rim (its bottom edge above the top of the bin's
    box, and at least min_overlap of its width over the bin); else around_bin. With the rim
    out of view nothing above it can be seen, so everything is around_bin."""
    if bin_bbox is None or not rim_visible:
        return "around_bin"
    ix1, _, ix2, iy2 = item_bbox
    bx1, by1, bx2, _ = bin_bbox
    overlap = max(0.0, min(ix2, bx2) - max(ix1, bx1)) / max(ix2 - ix1, 1e-6)
    if overlap >= min_overlap and iy2 <= by1 + rim_margin_px:
        return "on_bin"
    return "around_bin"


def analyze_bin_stop(dets, cam, pose=None, expected_bearing_deg=0.0, expected_bin_px=None,
                     cfg=None):
    """Turn one bin-stop photo's detections into what the follower records.

    expected_bin_px: the bin's (width, height) in px at its expected distance (a lone number
    = width only: then no size check).
    dets: detector output (dicts with kind "bin" | "item", label, category, conf, bbox).
    pose: robot pose (x, y, heading_deg) in the park frame when the photo was taken.
    Returns {"bin": the chosen bin detection or None, "bin_bearing_deg", "rim_visible",
             "items": [item dicts + place (+ obj_xy for around_bin items on the floor)]}.
    """
    cfg = cfg or {}
    cc, pc = cfg.get("centering", {}), cfg.get("place", {})
    edge = cc.get("edge_px", 4)
    if isinstance(expected_bin_px, (int, float)):
        expected_bin_px = (expected_bin_px, None)
    size = expected_bin_px if expected_bin_px and expected_bin_px[1] else None
    chosen = pick_bin([d for d in dets if d["kind"] == "bin"], cam, expected_bearing_deg,
                      size, edge, cc.get("min_size_frac", 0.5))
    out = {"bin": chosen, "bin_bearing_deg": None, "rim_visible": None, "items": []}
    if chosen:
        u = bin_center_u(chosen["bbox"], cam, edge, expected_bin_px and expected_bin_px[0])
        out["bin_bearing_deg"] = round(cam.bearing_deg(u), 2)
        out["rim_visible"] = "top" not in cut_edges(chosen["bbox"], cam, edge)
    for d in dets:
        if d["kind"] != "item":
            continue
        item = dict(d)
        item["place"] = item_place(d["bbox"], chosen and chosen["bbox"], out["rim_visible"],
                                   pc.get("rim_margin_px", 8), cc.get("min_overlap", 0.3))
        item["obj_xy"] = None
        if item["place"] == "around_bin" and pose is not None:
            x1, _, x2, y2 = d["bbox"]
            pt = cam.ground_point((x1 + x2) / 2.0, y2)
            if pt and math.hypot(*pt) <= pc.get("max_ground_range_m", 3.0):
                item["obj_xy"] = [round(v, 3) for v in robot_to_park(pt, pose)]
        out["items"].append(item)
    return out
