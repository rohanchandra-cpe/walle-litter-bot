"""Compare YOLO-World prompts on a set of photos (prompt tuning, ROADMAP Phase 3).

For each prompt, run the model with that prompt ALONE and report in how many photos it finds
something (at --conf), its median best score, and its median box. Prompts don't compete with
each other this way, so the numbers show how well each one fits the object by itself.

    python3 walle-app/detect/probe_prompts.py --prompts "trash can,black bin" \
        walle-app/results/route/*/photos/*.jpg
    python3 walle-app/detect/probe_prompts.py --model yolov8m-worldv2.pt --per-photo ...
"""

import argparse
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from detect.detector import MODELS_DIR, load_config, to_pil  # noqa: E402


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("photos", nargs="+")
    p.add_argument("--prompts", required=True, help="comma-separated")
    p.add_argument("--model", default=load_config()["model"])
    p.add_argument("--conf", type=float, default=0.10)
    p.add_argument("--per-photo", action="store_true", help="print each photo's best box")
    a = p.parse_args(argv)

    from ultralytics import YOLOWorld
    model = YOLOWorld(os.path.join(MODELS_DIR, a.model))
    images = [(os.path.relpath(f), to_pil(f)) for f in a.photos]
    print(f"{a.model}, {len(images)} photos, conf >= {a.conf}")
    for prompt in [s.strip() for s in a.prompts.split(",") if s.strip()]:
        model.set_classes([prompt])
        found, scores, rows = 0, [], []
        for name, img in images:
            r = model.predict(img, imgsz=640, conf=0.01, verbose=False)[0]
            best = max(zip(r.boxes.conf.tolist(), r.boxes.xyxy.tolist()), default=None)
            score = best[0] if best else 0.0
            scores.append(score)
            if score >= a.conf:
                found += 1
            rows.append((name, score, [round(v) for v in best[1]] if best else None))
        print(f"  {prompt:28s} found {found:2d}/{len(images)}  "
              f"median best {statistics.median(scores):.2f}  min {min(scores):.2f}")
        if a.per_photo:
            for name, score, box in rows:
                print(f"      {score:.2f} {box} {name}")


if __name__ == "__main__":
    sys.exit(main())
