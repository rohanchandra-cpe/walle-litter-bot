"""Write the configuration manifest: exactly what code, settings, and models a session used.

    python3 pilot/manifest.py --session 1     # data/manifests/session_1.json
(session_check.py runs this for you.)

Records the git commit (and whether files were uncommitted), the full pilot.yaml
and hashes of every frozen file, the depth calibration, the firmware image hash,
the Hugging Face model revision, and package versions.
"""

import argparse
import glob
import json
import os
import platform
import subprocess
import sys

from common import (FROZEN_FILES, MANIFEST_DIR, PILOT_DIR, file_sha256, frozen_hash, git_commit,
                    load_config, utc_now)



def package_versions():
    versions = {"python": sys.version.split()[0]}
    for name in ("numpy", "scipy", "torch", "transformers", "PIL", "matplotlib", "yaml"):
        try:
            module = __import__(name)
            versions[name] = getattr(module, "__version__", "unknown")
        except ImportError:
            versions[name] = "not installed"
    try:
        out = subprocess.run(["dpkg-query", "-W", "-f", "${Package} ${Version}\n",
                              "ros-humble-robot-localization", "ros-humble-rclpy",
                              "ros-humble-rosbag2"], capture_output=True, text=True)
        for line in out.stdout.splitlines():
            package, version = line.split(" ", 1)
            versions[package] = version
    except OSError:
        pass
    versions["ros_distro"] = os.environ.get("ROS_DISTRO", "")
    return versions


def model_revision(model_id):
    """The commit of the model snapshot in the Hugging Face cache, or "" if not downloaded."""
    cache = os.environ.get("HF_HOME", os.path.expanduser("~/.cache/huggingface"))
    ref = os.path.join(cache, "hub", "models--" + model_id.replace("/", "--"), "refs", "main")
    try:
        with open(ref) as f:
            return f.read().strip()
    except OSError:
        return ""


def firmware_hash():
    images = sorted(glob.glob(os.path.join(PILOT_DIR, "firmware", "pico_pilot", "build",
                                           "*.uf2")))
    return {os.path.basename(p): file_sha256(p) for p in images}


def write_manifest(session):
    sys.path.insert(0, os.path.join(PILOT_DIR, "depth"))
    import depth_estimator

    cfg = load_config()
    commit, dirty = git_commit()
    model_id = depth_estimator.MODELS[cfg["ranging"]["depth_model"]]
    calibration_path = os.path.join(PILOT_DIR, "config", "depth_calibration.json")
    calibration = None
    if os.path.exists(calibration_path):
        with open(calibration_path) as f:
            calibration = json.load(f)
    manifest = {
        "session_id": session, "written_utc": utc_now(),
        "git_commit": commit, "uncommitted_changes": dirty, "frozen_hash": frozen_hash(),
        "config": cfg, "depth_calibration": calibration,
        "depth_model": {"id": model_id, "revision": model_revision(model_id),
                        "input_size": cfg["ranging"]["input_size"]},
        "file_sha256": {f: file_sha256(os.path.join(PILOT_DIR, f)) for f in FROZEN_FILES},
        "firmware_uf2_sha256": firmware_hash(),
        "packages": package_versions(),
        "platform": {"machine": platform.machine(), "kernel": platform.release(),
                     "container": os.path.exists("/.dockerenv")},
    }
    os.makedirs(MANIFEST_DIR, exist_ok=True)
    path = os.path.join(MANIFEST_DIR, f"session_{session}.json")
    if os.path.exists(path):
        path = os.path.join(MANIFEST_DIR, f"session_{session}_{utc_now()[:19].replace(':', '')}.json")
    with open(path, "w") as f:
        json.dump(manifest, f, indent=2)
    if dirty and commit != "none":
        print("NOTE: there are uncommitted changes in walle-pilot/. Commit the frozen "
              "configuration before main-study sessions.")
    if not calibration:
        print("NOTE: config/depth_calibration.json is missing (depth model not calibrated).")
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--session", required=True)
    args = parser.parse_args()
    print(f"Wrote {write_manifest(args.session)}")


if __name__ == "__main__":
    main()
