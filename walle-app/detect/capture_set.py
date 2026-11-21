"""Take the Phase 3 labeled photo set with the robot's ESP32-CAM, one shot at a time.

Walks through data/photos/phase3/shots.yaml: prints each shot's setup, waits for you to
arrange it, takes the photo, saves shot_NN.jpg, and writes the shot's entry into
labels.yaml from the shot list. Shots already in labels.yaml are skipped, so you can stop
(q) and carry on later. Runs on the host (no ROS); only the 5 V side needs power.

    python3 walle-app/detect/capture_set.py --camera-url http://192.168.1.201/capture
    python3 walle-app/detect/capture_set.py --camera-url ... --only 7 12    # retake shots

At each shot: Enter = take it, r = take it again after looking, s = skip, q = quit.
If what you placed differs from the shot, fix that entry in labels.yaml afterwards.
"""

import argparse
import os
import sys
import time
import urllib.request
from datetime import datetime, timezone

import yaml

APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SET_DIR = os.path.join(APP_DIR, "data", "photos", "phase3")

PLACE_TEXT = {"on_bin": "ON TOP of the bin", "around_bin": "on the floor by the bin",
              "route": "on the floor"}
WHERE_TEXT = {"corner": "robot on bin {bin}'s corner, facing the bin",
              "route": "robot on the tape line, facing along the side",
              "carpet": "robot on the carpet, facing away from the square"}
HEADER = """\
# Phase 3 labeled photo set: what is in each photo (written by detect/capture_set.py from
# shots.yaml; edit an entry if what you placed differed). Read by detect/eval_photos.py.
# items: one entry per object, place on_bin | around_bin | route.
# plastic_bag covers grocery AND spinach bags (merged 2026-11-20).
"""


def describe(shot):
    lines = [f"  where: {WHERE_TEXT[shot['where']].format(bin=shot.get('bin'))}"]
    if shot.get("bin_shift_cm"):
        side = "LEFT" if shot["bin_shift_cm"] > 0 else "RIGHT"
        lines.append(f"  slide the bin {abs(shot['bin_shift_cm'])} cm to the robot's {side}")
    if shot["items"]:
        for label, place in shot["items"]:
            lines.append(f"  - {label}: {PLACE_TEXT[place]}")
    else:
        lines.append("  no items")
    if shot.get("note"):
        lines.append(f"  note: {shot['note']}")
    return "\n".join(lines)


def grab(url, timeout=5.0, tries=3):
    for i in range(tries):
        try:
            with urllib.request.urlopen(url, timeout=timeout) as reply:
                data = reply.read()
            if len(data) > 2000:
                return data
            print(f"  (photo only {len(data)} bytes, trying again)")
        except OSError as e:
            print(f"  (no photo, try {i + 1}/{tries}: {e})")
        time.sleep(2.0)
    return None


def entry(shot, name):
    e = {"id": shot["id"], "file": name, "where": shot["where"],
         "bin": shot.get("bin"), "bin_shift_cm": shot.get("bin_shift_cm", 0),
         "items": [{"label": label, "place": place} for label, place in shot["items"]],
         "t_utc": datetime.now(timezone.utc).isoformat(timespec="seconds")}
    if shot.get("note"):
        e["note"] = shot["note"]
    return e


def save_labels(path, labels):
    with open(path, "w") as f:
        f.write(HEADER)
        yaml.safe_dump({"photos": [labels[k] for k in sorted(labels)]}, f, sort_keys=False,
                       default_flow_style=None, width=100)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--camera-url", required=True)
    p.add_argument("--set-dir", default=SET_DIR)
    p.add_argument("--only", type=int, nargs="+", help="these shot ids (retakes allowed)")
    p.add_argument("--light", help="lighting for these shots, recorded in labels.yaml, "
                   "e.g. day (sunlight in the room) or night (room lamps)")
    a = p.parse_args(argv)

    with open(os.path.join(a.set_dir, "shots.yaml")) as f:
        shots = yaml.safe_load(f)["shots"]
    labels_path = os.path.join(a.set_dir, "labels.yaml")
    labels = {}
    if os.path.exists(labels_path):
        with open(labels_path) as f:
            labels = {e["id"]: e for e in (yaml.safe_load(f) or {}).get("photos", [])}
    # a shot is done when it has a label AND its photo is in the folder
    labels = {k: e for k, e in labels.items()
              if os.path.exists(os.path.join(a.set_dir, e["file"]))}
    todo = [s for s in shots if (s["id"] in a.only if a.only else s["id"] not in labels)]
    print(f"{len(labels)} of {len(shots)} shots done; {len(todo)} to take. Camera {a.camera_url}")

    for n, shot in enumerate(todo, 1):
        print(f"\nShot {shot['id']} ({n}/{len(todo)})\n{describe(shot)}")
        while True:
            key = input("  Enter = take, s = skip, q = quit > ").strip().lower()
            if key in ("q", "s"):
                break
            data = grab(a.camera_url)
            if data is None:
                print("  No photo. Is the 5 V side on and the camera on Wi-Fi?")
                continue
            name = f"shot_{shot['id']:02d}.jpg"
            with open(os.path.join(a.set_dir, name), "wb") as f:
                f.write(data)
            labels[shot["id"]] = entry(shot, name)
            if a.light:
                labels[shot["id"]]["light"] = a.light
            save_labels(labels_path, labels)
            print(f"  saved {name} ({len(data) // 1024} KB)")
            if input("  look at it; r = retake, Enter = next > ").strip().lower() != "r":
                break
        if key == "q":
            break
    print(f"\n{len(labels)} of {len(shots)} shots in {os.path.relpath(labels_path)}")


if __name__ == "__main__":
    sys.exit(main())
