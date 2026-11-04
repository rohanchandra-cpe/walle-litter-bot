"""Depth Anything V2 (metric, indoor): one photo in, distances in meters out.

This file has no ROS in it, so it also runs on its own (benchmark_depth.py,
download_model.py). depth_node.py uses it for the ROS lab.

    estimator = DepthEstimator("small")
    depth = estimator.estimate(photo)          # array (height, width), meters
    answer = analyze(depth, photo, "red ball") # distances ahead and to the red ball
"""

import json
import math
import os

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_FILE = os.path.join(HERE, "calibration.json")

# Metric models fine-tuned for INDOOR scenes (Hypersim), in Hugging Face format.
# They download by themselves the first time they are used.
MODELS = {
    "small": "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf",   # 24.8 M parameters, 99 MB
    "base": "depth-anything/Depth-Anything-V2-Metric-Indoor-Base-hf",     # 97.5 M parameters, 390 MB
    "large": "depth-anything/Depth-Anything-V2-Metric-Indoor-Large-hf",   # 335.3 M parameters, 1.3 GB
}

# Hue ranges on Pillow's 0-255 hue scale (255 = 360 degrees)
COLORS = {
    "red": [(0, 10), (245, 255)],
    "yellow": [(28, 53)],
    "green": [(64, 120)],
    "blue": [(135, 184)],
}
MIN_SATURATION = 110      # of 255: ignores white, gray, and washed-out colors
MIN_VALUE = 60            # of 255: ignores dark pixels
MIN_BLOB_FRACTION = 0.0004   # a colored blob needs at least this share of the photo
WINDOW_FRACTION = 0.10    # half-size of the "ahead / left / right" windows, as a share of the photo height
SCALE_RANGE = (0.2, 5.0)  # a calibration outside these ranges is ignored as a mistake
POWER_RANGE = (0.3, 1.5)


class DepthEstimator:
    def __init__(self, size="small", input_size=364):
        import torch
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation

        if size not in MODELS:
            raise ValueError(f"size must be one of {', '.join(MODELS)}")
        self.size, self.input_size, self.name = size, int(input_size), MODELS[size]
        self.torch = torch
        self.processor = AutoImageProcessor.from_pretrained(self.name, use_fast=False)
        self.model = AutoModelForDepthEstimation.from_pretrained(self.name).eval()

    def estimate(self, image):
        """Photo (Pillow RGB image) -> float32 array (height, width) of distances in meters."""
        torch = self.torch
        side = self.input_size   # the shorter side of the photo is resized to this many pixels
        inputs = self.processor(images=image, return_tensors="pt",
                                size={"height": side, "width": side})
        with torch.inference_mode():
            depth = self.model(**inputs).predicted_depth
            depth = torch.nn.functional.interpolate(
                depth[:, None], size=(image.height, image.width),
                mode="bilinear", align_corners=False)
        return depth[0, 0].numpy()


# ===== reading the depth map =====
def window_median(depth, cx, cy, half):
    height, width = depth.shape
    x0, x1 = max(0, int(cx - half)), min(width, int(cx + half))
    y0, y1 = max(0, int(cy - half)), min(height, int(cy + half))
    return float(np.median(depth[y0:y1, x0:x1]))


def parse_color(text):
    """The first color word in a target such as 'red ball' (None if there is none)."""
    for word in str(text).lower().replace(",", " ").split():
        if word in COLORS:
            return word
    return None


def find_color(image, color):
    """Largest blob of the given color: a boolean mask, or None if there is none."""
    hsv = np.asarray(image.convert("HSV")).astype(np.int16)
    hue, sat, val = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    mask = (sat >= MIN_SATURATION) & (val >= MIN_VALUE)
    in_range = np.zeros_like(mask)
    for low, high in COLORS[color]:
        in_range |= (hue >= low) & (hue <= high)
    labels, count = ndimage.label(mask & in_range)
    if count == 0:
        return None
    sizes = ndimage.sum(mask & in_range, labels, range(1, count + 1))
    best = int(np.argmax(sizes)) + 1
    if sizes[best - 1] < MIN_BLOB_FRACTION * mask.size:
        return None
    return labels == best


def load_calibration():
    """(scale, power) saved by fit_depth.py: real distance = scale x (model distance) ** power.
    (1.0, 1.0), meaning "not calibrated", if there is no usable calibration.json."""
    try:
        with open(CALIBRATION_FILE) as f:
            data = json.load(f)
        scale, power = float(data["scale"]), float(data.get("power", 1.0))
    except (OSError, ValueError, KeyError):
        return 1.0, 1.0
    if SCALE_RANGE[0] <= scale <= SCALE_RANGE[1] and POWER_RANGE[0] <= power <= POWER_RANGE[1]:
        return scale, power
    return 1.0, 1.0


def calibrated(raw, calibration):
    scale, power = calibration
    return scale * max(raw, 1e-3) ** power


def analyze(depth, image, target="", hfov_deg=66.0, calibration=(1.0, 1.0),
            object_diameter_m=0.0):
    """Distances ahead / left / right, and to the colored target if one is named.

    *_raw_m values are what the model says; *_m values are corrected with the
    calibration (scale, power). bearing_deg is positive when the target is to the left.
    If you give the real diameter of a round target, target_size_m is the distance
    from its size in the photo (a pinhole-camera estimate that needs no depth model).
    """
    height, width = depth.shape
    half = WINDOW_FRACTION * height
    out = {"scale": round(calibration[0], 4), "power": round(calibration[1], 4)}
    for name, x in (("left", width / 6), ("ahead", width / 2), ("right", 5 * width / 6)):
        raw = window_median(depth, x, height / 2, half)
        out[f"{name}_raw_m"] = round(raw, 3)
        out[f"{name}_m"] = round(calibrated(raw, calibration), 3)
    if not target:
        return out

    color = parse_color(target)
    if color is None:
        out["error"] = (f"Unknown color in '{target}'. Name the object with one of these "
                        f"colors: {', '.join(COLORS)} (for example 'red ball').")
        return out
    blob = find_color(image, color)
    out["target_found"] = blob is not None
    if blob is None:
        return out
    core = ndimage.binary_erosion(blob, iterations=2)   # skip the blurry edge pixels
    core = core if core.sum() >= 20 else blob
    ys, xs = np.nonzero(blob)
    cx, cy = float(xs.mean()), float(ys.mean())
    focal = (width / 2) / math.tan(math.radians(hfov_deg) / 2)
    raw = float(np.median(depth[core]))
    out.update(target_raw_m=round(raw, 3), target_m=round(calibrated(raw, calibration), 3),
               bearing_deg=round(math.degrees(math.atan2(width / 2 - cx, focal)), 1),
               target_pixels=int(blob.sum()), target_x=int(cx), target_y=int(cy))
    if object_diameter_m > 0:
        pixel_diameter = 2 * math.sqrt(blob.sum() / math.pi)
        out["target_size_m"] = round(focal * object_diameter_m / pixel_diameter, 3)
    return out


# ===== pictures for your report =====
def depth_picture(image, depth, answer=None, max_m=6.0):
    """The photo on the left and the depth map on the right (blue = near, red = far)."""
    from matplotlib import colormaps

    colors = colormaps["turbo"](np.clip(depth / max_m, 0.0, 1.0))[..., :3]
    right = Image.fromarray((colors * 255).astype(np.uint8))
    draw = ImageDraw.Draw(right)
    draw.text((6, 4), f"blue = near, red = far (0 to {max_m:.0f} m)", fill=(255, 255, 255))
    if answer and answer.get("target_found"):
        x, y = answer["target_x"], answer["target_y"]
        draw.ellipse((x - 5, y - 5, x + 5, y + 5), outline=(255, 255, 255), width=2)
        draw.text((x + 8, y - 6), f"{answer['target_m']:.2f} m", fill=(255, 255, 255))
    picture = Image.new("RGB", (image.width * 2, image.height))
    picture.paste(image, (0, 0))
    picture.paste(right, (image.width, 0))
    return picture
