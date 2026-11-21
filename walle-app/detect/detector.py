"""YOLO-World detector for WALL-E (ROADMAP Phase 3): bins and trash/recycling items.

    from detect.detector import Detector, load_config
    det = Detector()                      # source="robot" (default) or "sim"
    result = det.detect("photo.jpg")      # a path, JPEG bytes, or a PIL image
    result["detections"]                  # [{kind, label, category, prompt, conf, bbox}, ...]

Every prompt in config/detect.yaml becomes one YOLO-World class; each class maps back to its
label ("bin" or an item label) and category (trash | recycling | unknown). The text prompts
are encoded once, when the detector is made (CLIP), so each photo costs only the image model.

Runs on the CPU in the containers; ultralytics is imported only when a Detector is made, so
the rest of walle-app (and its tests) works without it. Weights are read from
walle-app/models/ and downloaded there the first time.
"""

import io
import os
import time

import yaml

APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_FILE = os.path.join(APP_DIR, "config", "detect.yaml")
MODELS_DIR = os.path.join(APP_DIR, "models")


def load_config(path=CONFIG_FILE):
    with open(path) as f:
        return yaml.safe_load(f)


def vocabulary(cfg):
    """The class list for the model: [(prompt, kind, label, category), ...]."""
    classes = [(p, "bin", cfg["bin"]["label"], None) for p in cfg["bin"]["prompts"]]
    for label, item in cfg["items"].items():
        classes += [(p, "item", label, item.get("category", "unknown")) for p in item["prompts"]]
    prompts = [c[0] for c in classes]
    if len(set(prompts)) != len(prompts):
        raise ValueError("config/detect.yaml: the same prompt is used twice.")
    return classes


def to_pil(image):
    from PIL import Image
    if isinstance(image, Image.Image):
        return image.convert("RGB")
    if isinstance(image, (bytes, bytearray)):
        return Image.open(io.BytesIO(image)).convert("RGB")
    return Image.open(image).convert("RGB")


class Detector:
    def __init__(self, source="robot", cfg=None, model=None):
        """source only labels the results (robot | sim); in Gazebo the follower uses
        detect/sim_truth.py instead, since the model sees nothing there."""
        from ultralytics import YOLOWorld   # here, so importing this module needs no ultralytics
        self.cfg = cfg or load_config()
        self.source = source
        self.model_name = model or self.cfg["model"]
        path = self.model_name if os.path.isabs(self.model_name) else os.path.join(
            MODELS_DIR, self.model_name)
        os.makedirs(MODELS_DIR, exist_ok=True)
        if not os.path.exists(path):
            cwd = os.getcwd()            # ultralytics downloads a bare name into the cwd
            try:
                os.chdir(MODELS_DIR)
                YOLOWorld(os.path.basename(path))
            finally:
                os.chdir(cwd)
        self.classes = vocabulary(self.cfg)
        self.model = YOLOWorld(path)
        self.model.set_classes([c[0] for c in self.classes])
        self.min_conf = {"item": float(self.cfg["min_conf"]),
                         "bin": float(self.cfg["bin"]["min_conf"])}
        # boxes down to hint_conf come back in low_conf (never used as a bin): bin centering
        # uses a faint box cut off at the frame edge to pick which way to search
        self.predict_conf = min(list(self.min_conf.values())
                                + [float(self.cfg["bin"].get("hint_conf", 1.0))])
        # warm up: the first call is much slower (allocations)
        from PIL import Image
        Detector.detect(self, Image.new("RGB", (640, 480)))

    def detect(self, image, keep_low=False):
        """Detect bins and items in one photo. Returns {"detections", "low_conf" (dropped
        items, only with keep_low), "seconds", "size": [w, h], "model"}."""
        img = to_pil(image)
        t0 = time.time()
        res = self.model.predict(img, imgsz=int(self.cfg["imgsz"]), conf=self.predict_conf,
                                 iou=float(self.cfg["iou"]), agnostic_nms=True,
                                 verbose=False)[0]
        seconds = time.time() - t0
        kept, low = [], []
        for cls, conf, box in zip(res.boxes.cls.tolist(), res.boxes.conf.tolist(),
                                  res.boxes.xyxy.tolist()):
            prompt, kind, label, category = self.classes[int(cls)]
            d = {"kind": kind, "label": label, "category": category, "prompt": prompt,
                 "conf": round(conf, 3), "bbox": [round(v, 1) for v in box]}
            (kept if conf >= self.min_conf[kind] else low).append(d)
        out = {"detections": kept, "seconds": round(seconds, 3), "size": list(img.size),
               "model": self.model_name, "source": self.source}
        if keep_low:
            out["low_conf"] = low
        return out


# Asking for "trash" missed clean upright items (a jug, a can, a tub are not "trash" to the
# model); "loose, movable objects" found them (2026-11-20, shots 1-29).
VLM_FIND_PROMPT = (
    "This photo is from a small robot's camera, low on the floor. List every loose, movable "
    "object you can see: on the floor, and on top of or sticking out of the black bin (cans, "
    "jugs, cups, tubs, bags, paper, foil, packaging, anything; for example {examples}). Do not "
    "list the black bin itself, furniture, walls, the floor, or things on shelves. One entry "
    "per separate object, with a short name and its box [ymin, xmin, ymax, xmax], 0-1000.")

# Items on top of a bin are small in the whole photo and were missed most (2026-11-20); a
# second FIND on a zoomed crop of the bin's rim gives the model more pixels for them.
VLM_ZOOM_PROMPT = (
    "This is a zoomed-in view of the top of a black bin, from a small robot's camera low on "
    "the floor. List every loose, movable object resting on the bin's rim or sticking up out "
    "of it, and any just beside it (cans, jugs, cups, tubs, bags, paper, foil, packaging, "
    "anything; for example {examples}). Do not list the black bin itself, furniture, walls, "
    "the floor, or things on shelves. One entry per separate object, with a short name and "
    "its box [ymin, xmin, ymax, xmax], 0-1000.")

VLM_NAME_PROMPT = (
    "This is a close crop of one object found on the floor by a robot (it was described as "
    "\"{name}\"). First describe it in one sentence: material, color, shape, shine. Then "
    "choose the label that matches best:\n{labels}\n- not_trash: not a piece of trash "
    "(furniture, a trash bin, floor, wall, shadow, a shoe...)")


class VLMDetector(Detector):
    """Two steps, by a vision-language model (Ollama, default gemma4): FIND items in the whole
    photo (free-form name + box), then NAME each one from a crop (describe, then pick a label
    or not_trash). Bins still come from YOLO-World, which finds them reliably; its item boxes
    are not used. Same result shape as Detector.detect; item conf is None (no score); each
    item keeps the model's "vlm_name" and "vlm_description"."""

    def __init__(self, source="robot", cfg=None, model=None, vlm_model=None):
        super().__init__(source=source, cfg=cfg, model=model)
        v = self.cfg["vlm"]
        self.vlm_model = vlm_model or v["model"]
        url = v.get("url") or os.environ.get("OLLAMA_URL", "http://host.docker.internal:11434")
        self.chat_url = url.rstrip("/") + "/api/chat"
        self.labels = list(self.cfg["items"])
        describe = v.get("describe", {})
        self.find_prompt = VLM_FIND_PROMPT.format(
            examples=", ".join(describe.get(k, k.replace("_", " ")) for k in self.labels))
        self.zoom_prompt = VLM_ZOOM_PROMPT.format(
            examples=", ".join(describe.get(k, k.replace("_", " ")) for k in self.labels))
        self.zoom = v.get("zoom_bin_top", {}) or {}
        self.label_lines = "\n".join(f"- {k}: {describe.get(k, k.replace('_', ' '))}"
                                      for k in self.labels)
        self.find_schema = {"type": "object", "properties": {"objects": {"type": "array",
            "items": {"type": "object", "properties": {
                "name": {"type": "string"},
                "box_2d": {"type": "array", "items": {"type": "number"}}},
                "required": ["name", "box_2d"]}}}, "required": ["objects"]}
        self.name_schema = {"type": "object", "properties": {
            "description": {"type": "string"},
            "label": {"type": "string", "enum": self.labels + ["not_trash"]}},
            "required": ["description", "label"]}
        self.model_name = f"{self.model_name}+{self.vlm_model}"

    def chat(self, prompt, jpeg, schema):
        import base64
        import json
        import urllib.request
        body = {"model": self.vlm_model, "stream": False, "think": False, "format": schema,
                "keep_alive": "30m", "options": {"temperature": 0, "num_ctx": 8192},
                "messages": [{"role": "user", "content": prompt,
                              "images": [base64.b64encode(jpeg).decode()]}]}
        req = urllib.request.Request(self.chat_url, data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=float(self.cfg["vlm"]["timeout_s"])) as reply:
            return json.loads(json.load(reply)["message"]["content"])

    def box_order(self):
        """yxyx or xyxy for this model: config box_order, a string or {family: order}."""
        order = self.cfg["vlm"].get("box_order", "yxyx")
        if isinstance(order, str):
            return order
        for family, o in order.items():
            if family != "default" and self.vlm_model.startswith(family):
                return o
        return order.get("default", "yxyx")

    @staticmethod
    def jpeg(img, quality=95):
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=quality)
        return buf.getvalue()

    def crop(self, img, box, pad=0.15, min_side=224):
        x1, y1, x2, y2 = box
        px, py = (x2 - x1) * pad, (y2 - y1) * pad
        c = img.crop((max(0, x1 - px), max(0, y1 - py), min(img.width, x2 + px),
                      min(img.height, y2 + py)))
        scale = max(1.0, min_side / max(1, min(c.size)))
        return c.resize((round(c.width * scale), round(c.height * scale))) if scale > 1 else c

    def find(self, img, prompt, region=None):
        """FIND step on img (or on the region [x1, y1, x2, y2] of it, zoomed): boxes in
        img pixels and the model's names, [(box, name), ...]."""
        x0, y0 = 0.0, 0.0
        view = img
        if region is not None:
            x0, y0 = region[0], region[1]
            view = img.crop(tuple(region))
            side = float(self.zoom.get("min_side", 448))
            scale = max(1.0, side / max(1, min(view.size)))
            if scale > 1:
                view = view.resize((round(view.width * scale), round(view.height * scale)))
        rw = (region[2] - region[0]) if region is not None else img.width
        rh = (region[3] - region[1]) if region is not None else img.height
        yxyx = self.box_order() == "yxyx"
        out = []
        for o in self.chat(prompt, self.jpeg(view), self.find_schema)["objects"]:
            b = o.get("box_2d") or []
            if len(b) != 4:
                continue
            y1, x1, y2, x2 = b if yxyx else (b[1], b[0], b[3], b[2])
            box = [round(x0 + min(x1, x2) * rw / 1000, 1), round(y0 + min(y1, y2) * rh / 1000, 1),
                   round(x0 + max(x1, x2) * rw / 1000, 1), round(y0 + max(y1, y2) * rh / 1000, 1)]
            if box[2] - box[0] >= 4 and box[3] - box[1] >= 4:
                out.append((box, o.get("name", "object")))
        return out

    def rim_region(self, bins, w, h):
        """The zoom window over the biggest bin's top: the rim, the space above it for items
        standing on it, and a little to each side; None if there is no bin or the window
        would be most of the photo anyway."""
        if not bins:
            return None
        x1, y1, x2, y2 = max((d["bbox"] for d in bins),
                             key=lambda b: (b[2] - b[0]) * (b[3] - b[1]))
        bw, bh = x2 - x1, y2 - y1
        z = self.zoom
        r = [max(0.0, x1 - z.get("side", 0.25) * bw), max(0.0, y1 - z.get("above", 1.0) * bh),
             min(float(w), x2 + z.get("side", 0.25) * bw), min(float(h), y1 + z.get("below", 0.3) * bh)]
        if (r[2] - r[0]) * (r[3] - r[1]) > 0.6 * w * h or r[3] - r[1] < 8:
            return None
        return [round(v, 1) for v in r]

    def detect(self, image, keep_low=False):
        img = to_pil(image)
        out = super().detect(img, keep_low=keep_low)
        out["detections"] = [d for d in out["detections"] if d["kind"] == "bin"]
        if keep_low:
            out["low_conf"] = [d for d in out["low_conf"] if d["kind"] == "bin"]
        t0 = time.time()
        found = self.find(img, self.find_prompt)
        region = None
        if self.zoom.get("enabled"):
            region = self.rim_region(out["detections"], *img.size)
            if region is not None:
                found = merge_boxes(self.find(img, self.zoom_prompt, region), found)
                out["zoom_region"] = region
        rejected = []
        for box, name in found:
            named = self.chat(VLM_NAME_PROMPT.format(name=name, labels=self.label_lines),
                              self.jpeg(self.crop(img, box)), self.name_schema)
            d = {"kind": "item", "label": named["label"],
                 "category": self.cfg["items"].get(named["label"], {}).get("category",
                                                                           "unknown"),
                 "prompt": self.vlm_model, "conf": None, "bbox": box,
                 "vlm_name": name, "vlm_description": named.get("description")}
            (rejected if named["label"] == "not_trash" else out["detections"]).append(d)
        out["vlm_seconds"] = round(time.time() - t0, 3)
        out["seconds"] = round(out["seconds"] + out["vlm_seconds"], 3)
        if keep_low:
            out["low_conf"] += rejected
        return out


def overlap(a, b):
    """IoU of two boxes, and how much of the smaller one lies inside the other."""
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    area = lambda r: max(0.0, r[2] - r[0]) * max(0.0, r[3] - r[1])  # noqa: E731
    union = area(a) + area(b) - inter
    return (inter / union if union else 0.0), (inter / min(area(a), area(b)) if min(area(a), area(b)) else 0.0)


def merge_boxes(first, rest, iou=0.3, inside=0.7):
    """first + the boxes of rest that are not the same object as one in first (IoU >= iou,
    or the smaller one mostly inside the other). first wins: the zoomed boxes are sharper."""
    out = list(first)
    for box, name in rest:
        if all(overlap(box, b)[0] < iou and overlap(box, b)[1] < inside for b, _ in first):
            out.append((box, name))
    return out


def make_detector(source="robot", cfg=None, method=None, **kw):
    """The detector config/detect.yaml `method` names: vlm (default) or yolo."""
    cfg = cfg or load_config()
    method = method or cfg.get("method", "yolo")
    return VLMDetector(source, cfg, **kw) if method == "vlm" else Detector(source, cfg, **kw)
