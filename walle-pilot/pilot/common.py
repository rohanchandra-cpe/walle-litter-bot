"""Shared helpers for the pilot scripts: paths, the frozen config, CSV logs, prompts.

Every log is an append-only CSV. Rows are never edited or deleted by these
scripts; a repeat of an invalid trial is a new row with a higher attempt number.
"""

import csv
import datetime
import hashlib
import math
import os
import subprocess

import yaml

PILOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# The Gazebo container (walle-sim/) points these at its own config and data folder,
# so simulated runs never mix with the robot's frozen config or real logs.
CONFIG_FILE = os.environ.get("WALLE_CONFIG") or os.path.join(PILOT_DIR, "config", "pilot.yaml")
DATA_DIR = os.environ.get("WALLE_DATA_DIR") or os.path.join(PILOT_DIR, "data")
SCHEDULE_DIR = os.path.join(DATA_DIR, "schedule")
LOG_DIR = os.path.join(DATA_DIR, "logs")
BAG_DIR = os.path.join(DATA_DIR, "bags")
RANGE_DIR = os.path.join(DATA_DIR, "range")
MANIFEST_DIR = os.path.join(DATA_DIR, "manifests")
RESULTS_DIR = os.path.join(DATA_DIR, "results")

MOTION_SCHEDULE = os.path.join(SCHEDULE_DIR, "motion_schedule.csv")
RANGE_SCHEDULE = os.path.join(SCHEDULE_DIR, "range_schedule.csv")
SESSION_LOG = os.path.join(LOG_DIR, "session_log.csv")
MOTION_LOG = os.path.join(LOG_DIR, "motion_log.csv")
SHAKEDOWN_LOG = os.path.join(LOG_DIR, "shakedown_log.csv")
RANGE_LOG = os.path.join(LOG_DIR, "range_log.csv")
DEPTH_CALIBRATION_LOG = os.path.join(LOG_DIR, "depth_calibration_log.csv")
REPEATABILITY_LOG = os.path.join(LOG_DIR, "gt_repeatability.csv")

# Files whose content defines the frozen configuration (relative to walle-pilot/)
FROZEN_FILES = [
    "config/pilot.yaml", "config/ekf.yaml", "config/depth_calibration.json",
    "data/schedule/motion_schedule.csv", "data/schedule/range_schedule.csv",
    "launch/pilot.launch.py", "odometry/odometry_node.py", "depth/depth_estimator.py",
    "pilot/motion_trial.py", "pilot/range_trial.py", "pilot/robot_io.py",
    "firmware/pico_pilot/pico_pilot.c", "firmware/pico_pilot/ultrasonic.c",
    "firmware/pico_pilot/drive_control.c", "firmware/pico_pilot/encoders.c",
]


def load_config():
    with open(CONFIG_FILE) as f:
        return yaml.safe_load(f)


def file_sha256(path):
    if not os.path.exists(path):
        return ""
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def git_commit():
    """(short commit hash, True if there are uncommitted changes), or ("none", True)."""
    try:
        commit = subprocess.run(["git", "-C", PILOT_DIR, "rev-parse", "--short", "HEAD"],
                                capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "-C", PILOT_DIR, "status", "--porcelain", "--", "."],
                               capture_output=True, text=True, check=True).stdout.strip()
        return commit, bool(dirty)
    except (OSError, subprocess.CalledProcessError):
        return "none", True


def frozen_hash():
    """First 10 hex digits of a hash over every frozen file's content."""
    digest = hashlib.sha256()
    overrides = {"config/pilot.yaml": CONFIG_FILE}   # hash the config actually in use
    for name in FROZEN_FILES:
        path = overrides.get(name, os.path.join(PILOT_DIR, name))
        digest.update(name.encode() + b"\0" + file_sha256(path).encode())
    return digest.hexdigest()[:10]


def commit_label():
    """The git commit (+"-dirty" if files changed), and always the frozen-files hash,
    so trials are traceable even when walle-pilot/ is not inside a git checkout."""
    commit, dirty = git_commit()
    label = "nogit" if commit == "none" else commit + ("-dirty" if dirty else "")
    return f"{label}+cfg-{frozen_hash()}"


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="milliseconds")


def wrap_deg(angle):
    """Wrap degrees into [-180, 180)."""
    return (angle + 180.0) % 360.0 - 180.0


# ===== CSV logs =====
def read_rows(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def append_row(path, fields, row):
    """Append one row, writing the header first if the file is new."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    new = not os.path.exists(path) or os.path.getsize(path) == 0
    if not new:
        with open(path, newline="") as f:
            header = next(csv.reader(f), [])
        if header != fields:
            raise SystemExit(f"{path} has different columns than this script writes. "
                             "Do not edit log headers; start a new file instead.")
    with open(path, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        if new:
            writer.writeheader()
        writer.writerow({k: fmt(row.get(k, "")) for k in fields})


def fmt(value):
    if isinstance(value, float):
        return "" if math.isnan(value) else f"{value:.6g}"
    if isinstance(value, bool):
        return "1" if value else "0"
    return value


def session_value(session, key, default=None):
    """The latest value of `key` logged by session_check.py for this session."""
    rows = [r for r in read_rows(SESSION_LOG) if r.get("session_id") == str(session)]
    for row in reversed(rows):
        if row.get(key) not in (None, ""):
            return row[key]
    return default


def next_scheduled(session, schedule, log, key):
    """The next scheduled item of a session and its attempt number, or (None, 0).

    First the items with no attempt yet, in schedule order; then any item whose
    last attempt was invalidated by outside interference (repeated at the session
    end under the same condition). Robot failures are outcomes and are not repeated.
    """
    rows = [r for r in schedule if r["session_id"] == str(session)]
    attempts = {}
    for entry in log:
        attempts.setdefault(entry[key], []).append(entry)
    for row in rows:
        if row[key] not in attempts:
            return row, 1
    for row in rows:
        tries = attempts[row[key]]
        if tries[-1]["external_invalid"] == "1":
            return row, len(tries) + 1
    return None, 0


# ===== prompts for the measurer and operator =====
def ask(prompt, choices=None, allow_blank=False):
    while True:
        answer = input(prompt).strip()
        if not answer and allow_blank:
            return ""
        if choices is None and answer:
            return answer
        if choices is not None and answer.lower() in choices:
            return answer.lower()
        if choices:
            print(f"  Please answer one of: {', '.join(choices)}")


def ask_yes(prompt):
    return ask(prompt + " [y/n] ", choices=["y", "n"]) == "y"


def ask_float(prompt, allow_blank=False):
    """A number, or "" when allow_blank and the answer is empty."""
    while True:
        answer = input(prompt).strip()
        if not answer and allow_blank:
            return ""
        try:
            return float(answer)
        except ValueError:
            print("  Please type a number" + (" (or press Enter to leave it blank)."
                                              if allow_blank else "."))
