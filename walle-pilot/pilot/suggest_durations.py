"""Suggest T_A, T_psi, and the RQ1-B coast setting from the shakedown runs.

    python3 pilot/suggest_durations.py

Reads data/logs/shakedown_log.csv (motion_trial.py --shakedown ...) and prints
suggestions. It does not change config/pilot.yaml: copy the values in by hand,
repeat a few shakedown runs with them, then freeze (commit) the file.

A protractor and the logged headings cannot show a turn past 180 degrees, so each
turn's full rotations are restored from the encoder counts before fitting.
"""

import math
import statistics

import numpy as np

from common import SHAKEDOWN_LOG, load_config, read_rows


def usable(rows, value_key):
    return [r for r in rows if r.get(value_key) not in ("", None) and r["completed"] == "1"
            and r["external_invalid"] != "1"]


def add_full_turn(row, geometry):
    """Copy of a turn row with the measured turn plus the full rotations it hides."""
    row = dict(row, true_turn_full_deg="")
    if row.get("true_turn_deg") in ("", None) or not row.get("enc_left_delta"):
        return row
    per_count = math.pi * geometry["wheel_diameter_m"] / geometry["counts_per_rev"]
    arc = int(row["enc_right_delta"]) - int(row["enc_left_delta"])
    encoder_deg = abs(math.degrees(arc * per_count / geometry["wheel_separation_m"]))
    measured = abs(float(row["true_turn_deg"]))
    row["true_turn_full_deg"] = measured + 360.0 * round((encoder_deg - measured) / 360.0)
    return row


def suggest(rows, value_key, target, label, setting, scale=1.0):
    rows = [r for r in usable(rows, value_key) if r["cmd_duration_s"]]
    if not rows:
        print(f"{label}: no usable shakedown runs yet.")
        return
    durations = np.array([float(r["cmd_duration_s"]) for r in rows])
    values = np.array([abs(float(r[value_key])) * scale for r in rows])
    print(f"{label}: {len(rows)} runs, mean {values.mean():.1f} (target {target}), "
          f"SD {values.std(ddof=1) if len(rows) > 1 else float('nan'):.1f}")
    if len(set(durations)) >= 2:
        slope, intercept = np.polyfit(durations, values, 1)
        best = (target - intercept) / slope
        print(f"  linear fit over {len(set(durations))} durations: "
              f"value = {intercept:.2f} + {slope:.2f} x T  ->  {setting}: {best:.3f} s")
    else:
        best = durations[0] * target / values.mean()
        print(f"  one duration tried ({durations[0]:.3f} s); proportional estimate  ->  "
              f"{setting}: {best:.3f} s")
        print("  (Try a second duration near it: the start and stop make the motion "
              "not quite proportional to time.)")


def main():
    cfg = load_config()
    rows = read_rows(SHAKEDOWN_LOG)
    if not rows:
        raise SystemExit("No shakedown runs yet: python3 pilot/motion_trial.py --shakedown straight-A")
    target_cm = 100 * cfg["motion"]["straight_target_m"]
    a = [r for r in rows if r["task"] == "straight" and r["strategy"] == "A"]
    b = [r for r in rows if r["task"] == "straight" and r["strategy"] == "B"]
    turns = [add_full_turn(r, cfg["geometry"]) for r in rows if r["task"] == "turn"]
    suggest(a, "true_forward_cm", target_cm, "Straight A (cm)", "motion.timed_duration_s")
    suggest(turns, "true_turn_full_deg", 90.0, "Turns (deg, both directions)", "motion.turn_duration_s")
    for direction in ("ccw", "cw"):
        sub = usable([r for r in turns if r["direction"] == direction], "true_turn_full_deg")
        if sub:
            print(f"  {direction}: mean {statistics.mean(float(r['true_turn_full_deg']) for r in sub):.1f} deg "
                  f"over {len(sub)} runs")

    b = usable(b, "true_forward_cm")
    if b:
        errors = [float(r["true_forward_cm"]) - target_cm for r in b]
        coast = cfg["motion"]["targeted_coast_time_s"]
        mean = statistics.mean(errors)
        print(f"Straight B: {len(b)} runs, mean signed error {mean:+.1f} cm "
              f"(coast time now {coast} s)")
        if abs(mean) > 1.0:
            # At about the minimum speed near the stop, 1 cm of overshoot is ~0.2 s of coast
            v = cfg["motion"]["targeted_min_speed_mps"]
            print(f"  {'over' if mean > 0 else 'under'}shoot: try motion.targeted_coast_time_s "
                  f"about {max(0.0, coast + mean / 100 / v):.2f} s")
    else:
        print("Straight B: no usable shakedown runs yet.")


if __name__ == "__main__":
    main()
