"""Photo geometry for detections (ROADMAP Phase 3). Pure Python, no model, no ROS.

Conventions (the park's): angles in degrees, positive = to the LEFT (counter-clockwise).
Pixel u grows to the right, v grows down; the image is upright (the ESP32-CAM firmware flips
it). The camera is a pinhole with the field of view from park.yaml robot.camera, looking
straight ahead, height_m above the floor and forward_m ahead of the axle, level or tilted up
by the Phase 4 servo (Camera tilt_deg).
"""

import math


def focal_px(size_px, fov_deg):
    """Focal length in pixels for an image `size_px` wide (or tall) with that field of view."""
    return (size_px / 2.0) / math.tan(math.radians(fov_deg) / 2.0)


class Camera:
    """Pinhole camera on the robot. tilt_deg pitches it about the robot's left axis (+ = looks
    up; the Phase 4 servo). height_m / forward_m are where the lens is when LEVEL; if the lens
    sits off the tilt axis (lens_from_pivot_m = [forward, up] from the axis, level), it moves
    on a circle around the axis as the camera tilts. tilt 0 gives the level camera exactly.
    left_m: the lens's offset to the left of the robot's center line (it looks straight
    ahead, parallel to the robot's heading)."""

    def __init__(self, width=640, height=480, hfov_deg=60.0, vfov_deg=47.0, height_m=0.103,
                 forward_m=0.12, tilt_deg=0.0, lens_from_pivot_m=(0.0, 0.0), left_m=0.0):
        self.w, self.h = width, height
        self.fx, self.fy = focal_px(width, hfov_deg), focal_px(height, vfov_deg)
        self.tilt_deg = float(tilt_deg or 0.0)
        t = math.radians(self.tilt_deg)
        self._c, self._s = math.cos(t), math.sin(t)
        df, dz = lens_from_pivot_m or (0.0, 0.0)
        px, pz = forward_m - df, height_m - dz             # the tilt axis
        self.forward_m = px + df * self._c - dz * self._s  # the lens, at this tilt
        self.height_m = pz + df * self._s + dz * self._c
        self.left_m = float(left_m or 0.0)

    @classmethod
    def from_park(cls, cam, width=640, height=480, tilt_deg=0.0, lens_from_pivot_m=None):
        """From park.yaml robot.camera (+ the tilt, and servo.yaml lens_from_pivot_m)."""
        return cls(width, height, cam["horizontal_fov_deg"], cam["vertical_fov_deg"],
                   cam["height_m"], cam["forward_m"], tilt_deg,
                   lens_from_pivot_m or cam.get("lens_from_pivot_m") or (0.0, 0.0),
                   cam.get("left_m", 0.0))

    def ray(self, u, v):
        """Direction (forward, left, up) in the ROBOT frame of the ray through pixel (u, v)
        (not unit length: its component along the optical axis is 1)."""
        a = (self.w / 2.0 - u) / self.fx                   # left, per unit along the axis
        b = (self.h / 2.0 - v) / self.fy                   # up, likewise
        return (self._c - b * self._s, a, self._s + b * self._c)

    def bearing_deg(self, u, v=None):
        """Horizontal bearing of pixel column u (at row v; the middle row if None) from the
        camera's forward direction (the robot's heading); positive = left."""
        f, left, _ = self.ray(u, self.h / 2.0 if v is None else v)
        return math.degrees(math.atan2(left, f))

    def elevation_deg(self, v):
        """Angle above horizontal of the ray through row v, in the middle column."""
        return self.tilt_deg + math.degrees(math.atan((self.h / 2.0 - v) / self.fy))

    def width_px(self, width_m, distance_m):
        """How wide something width_m wide looks at distance_m straight ahead."""
        return self.fx * width_m / max(distance_m, 1e-6)

    def project(self, pt):
        """Pixel (u, v) of a ROBOT-frame point (x forward from the axle, y left, z up), or
        None if it is behind the camera. Pixels may fall outside the frame."""
        dx, dy, dz = pt[0] - self.forward_m, pt[1] - self.left_m, pt[2] - self.height_m
        fwd = dx * self._c + dz * self._s
        up = -dx * self._s + dz * self._c
        if fwd <= 1e-6:
            return None
        return (self.w / 2.0 - self.fx * dy / fwd, self.h / 2.0 - self.fy * up / fwd)

    def expected_box_px(self, width_m, distance_m, z_bottom=0.0, z_top=0.29):
        """(width, height) in px of an upright face width_m wide, distance_m straight ahead
        of the lens, from z_bottom to z_top above the floor: the expected bin box. The
        height is clipped to the frame."""
        x = self.forward_m + distance_m
        top = self.project((x, self.left_m, z_top))
        bottom = self.project((x, self.left_m, z_bottom))
        if top is None or bottom is None:
            return (self.width_px(width_m, distance_m), None)
        v1, v2 = max(0.0, top[1]), min(float(self.h), bottom[1])
        return (self.width_px(width_m, distance_m), max(0.0, v2 - v1))

    def height_at(self, v, distance_m, u=None):
        """Height above the floor where the ray through (u, v) is distance_m ahead of the lens
        (along the robot's forward direction), e.g. an item's top over a bin."""
        f, _, z = self.ray(self.w / 2.0 if u is None else u, v)
        if f <= 1e-6:
            return None
        return self.height_m + distance_m * z / f

    def ground_point(self, u, v):
        """Where the ray through pixel (u, v) meets the floor, in the ROBOT frame (x forward
        from the axle, y left), or None if the pixel is at or above the horizon."""
        f, left, z = self.ray(u, v)
        if z >= -1e-3:
            return None
        t = self.height_m / -z
        return (self.forward_m + t * f, self.left_m + t * left)


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
                     cfg=None, stop=None):
    """Turn one bin-stop photo's detections into what the follower records.

    expected_bin_px: the bin's (width, height) in px at its expected distance (a lone number
    = width only: then no size check).
    dets: detector output (dicts with kind "bin" | "item", label, category, conf, bbox).
    pose: robot pose (x, y, heading_deg) in the park frame when the photo was taken.
    stop: for the photos taken at a bin stop AFTER centering (the Phase 4 bin check; not
    centering itself), a dict with
      bin_dist_m, bin_height_m: the bin center's distance ahead of the lens, its height;
      low_conf_bins: faint bin boxes (detector low_conf), used only when no bin passed;
      ahead_deg: where the bin should be in the photo (default 0: straight ahead);
    and detect.yaml bin_check's rules (robot run 20261122_134315):
      - pick_max_offset_deg: only bin boxes this near straight ahead (the robot has just
        centered on the bin; the configured bearing may be off when a bin isn't where
        park.yaml says) (a Ziploc box beside bin 4 was taken for it);
      - low_conf_max_offset_deg: a faint bin box this near is the bin when none passed
        (bin 3's up photo: the bin scored 0.025);
      - on_top_max_gap_m: an on_bin item must rest on the rim, its bottom no higher than
        this above it (a speck 35 cm above bin 1 was called a paper bag on top);
      - rim_margin_frac: an item may hang this share of the bin box's height below its top
        edge and still be on the bin (at least place.rim_margin_px): soft items drape over
        the rim (runs 20261122_134315/142402/145845, three scenes);
      - an around_bin item cut off at the frame's bottom, in a photo whose bottom row is
        nearer than the bin, gets the frame-bottom floor point (obj_xy_from: frame_bottom;
        its foot is just past it): the bottle in front of bin 4.
    Returns {"bin": the chosen bin detection or None, "bin_bearing_deg", "rim_visible",
             "items": [item dicts + place (+ obj_xy for around_bin items on the floor)],
             "ignored": [items that can't be on the bin or the floor (stop only)]}.
    """
    cfg = cfg or {}
    cc, pc = cfg.get("centering", {}), cfg.get("place", {})
    bc = cfg.get("bin_check", {})
    edge = cc.get("edge_px", 4)
    if isinstance(expected_bin_px, (int, float)):
        expected_bin_px = (expected_bin_px, None)
    size = expected_bin_px if expected_bin_px and expected_bin_px[1] else None
    min_frac = cc.get("min_size_frac", 0.5)
    ahead = float((stop or {}).get("ahead_deg", 0.0))

    def near(boxes, max_off):
        if max_off is None:
            return boxes
        return [d for d in boxes if abs(cam.bearing_deg(bin_center_u(
            d["bbox"], cam, edge, expected_bin_px and expected_bin_px[0]))
            - ahead) <= max_off]

    bins = [d for d in dets if d["kind"] == "bin"]
    if stop:
        bins = near(bins, bc.get("pick_max_offset_deg"))
    chosen = pick_bin(bins, cam, expected_bearing_deg, size, edge, min_frac)
    if chosen is None and stop and stop.get("low_conf_bins"):
        faint = near([d for d in stop["low_conf_bins"] if d["kind"] == "bin"],
                     bc.get("low_conf_max_offset_deg"))
        chosen = pick_bin(faint, cam, expected_bearing_deg, size, edge, min_frac)
        if chosen:
            chosen = dict(chosen, faint=True)
    out = {"bin": chosen, "bin_bearing_deg": None, "rim_visible": None, "items": [],
           "ignored": []}
    if chosen:
        u = bin_center_u(chosen["bbox"], cam, edge, expected_bin_px and expected_bin_px[0])
        out["bin_bearing_deg"] = round(cam.bearing_deg(u), 2)
        out["rim_visible"] = "top" not in cut_edges(chosen["bbox"], cam, edge)
    dist = (stop or {}).get("bin_dist_m")
    bin_h = (stop or {}).get("bin_height_m")
    for d in dets:
        if d["kind"] != "item":
            continue
        item = dict(d)
        x1, _, x2, y2 = d["bbox"]
        margin = pc.get("rim_margin_px", 8)
        if stop and chosen and bc.get("rim_margin_frac"):
            margin = max(margin, bc["rim_margin_frac"] * (chosen["bbox"][3] - chosen["bbox"][1]))
        item["place"] = item_place(d["bbox"], chosen and chosen["bbox"], out["rim_visible"],
                                   margin, cc.get("min_overlap", 0.3))
        item["obj_xy"] = None
        if item["place"] == "on_bin" and dist and bin_h and \
                bc.get("on_top_max_gap_m") is not None:
            z = cam.height_at(y2, dist, (x1 + x2) / 2.0)
            if z is not None and z - bin_h > bc["on_top_max_gap_m"]:
                item["ignored"] = f"bottom {100 * (z - bin_h):.0f} cm above the rim"
                out["ignored"].append(item)
                continue
        cut_bottom = "bottom" in cut_edges(d["bbox"], cam, edge)
        if item["place"] == "around_bin" and pose is not None and not cut_bottom:
            pt = cam.ground_point((x1 + x2) / 2.0, y2)
            if pt and math.hypot(*pt) <= pc.get("max_ground_range_m", 3.0):
                item["obj_xy"] = [round(v, 3) for v in robot_to_park(pt, pose)]
        elif item["place"] == "around_bin" and pose is not None and dist:
            # cut off at the bottom: its foot is just nearer than the frame's bottom row. Only
            # if that row is nearer than the bin (the level look; in the up look it lies
            # beyond the bin, and the position would be 18-20 cm off, 2026-11-21).
            pt = cam.ground_point((x1 + x2) / 2.0, cam.h - 1)
            if pt and pt[0] - cam.forward_m < dist - 0.05:
                item["obj_xy"] = [round(v, 3) for v in robot_to_park(pt, pose)]
                item["obj_xy_from"] = "frame_bottom"
        out["items"].append(item)
    return out
