"""Download a depth model once, and check that it runs on your computer's CPU.

    python3 depth/download_model.py            # the small model (99 MB)
    python3 depth/download_model.py small base # more than one (base is 390 MB)

Models are saved in depth/hf_cache inside your project folder. Copy that folder to
another computer to use the models there without downloading them again (then set
HF_HUB_OFFLINE=1 in the container to stop it looking online).
"""

import os
import sys
import time

from PIL import Image

import depth_estimator

SIZES = {"small": "99 MB", "base": "390 MB", "large": "1.3 GB"}


def main():
    sizes = sys.argv[1:] or ["small"]
    for size in sizes:
        if size not in depth_estimator.MODELS:
            raise SystemExit(f"Unknown size '{size}'. Choose from: {', '.join(SIZES)}")
    for size in sizes:
        print(f"{size}: {depth_estimator.MODELS[size]} (about {SIZES[size]}) ...")
        start = time.time()
        estimator = depth_estimator.DepthEstimator(size)
        print(f"  downloaded and loaded in {time.time() - start:.1f} s")
        gray = Image.new("RGB", (640, 480), (128, 128, 128))
        estimator.estimate(gray)                      # the first run is slower
        start = time.time()
        depth = estimator.estimate(gray)
        print(f"  one 640x480 photo on the CPU: {time.time() - start:.2f} s, "
              f"depth map {depth.shape[1]}x{depth.shape[0]} in meters: OK")
    print(f"Models are in {os.environ.get('HF_HOME', '~/.cache/huggingface')}")


if __name__ == "__main__":
    main()
