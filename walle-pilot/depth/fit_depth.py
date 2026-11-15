"""Calibrate the depth camera with a tape measure (copied from Module 8).

Log 8 measurements of the calibration object (a red ball) on the 0 deg ray at
distances from about 0.35 to 1.85 m that are NOT test distances:
    python3 pilot/range_trial.py --calibrate
Then run, from /workspace/walle-pilot:
    python3 depth/fit_depth.py                 # fit and save config/depth_calibration.json
    python3 depth/fit_depth.py --fit scale     # force the scale-only fit
    python3 depth/fit_depth.py --reset         # delete the calibration

Two fits are tried:
    scale          real = scale x model                  (one number)
    scale + power  real = scale x model ** power         (two numbers; fixes errors that
                                                          grow with distance)
The "leave-one-out" error predicts each measurement from a fit that did not see it: it
estimates how well the calibration will work on NEW distances. By default the fit with
the lower leave-one-out error is saved (the protocol chooses by leave-one-out, never by
test errors). Calibrate again whenever you move or re-aim the camera or change the model
size or input size, and commit the result before session 1.
"""

import argparse
import csv
import json
import os
import statistics

import numpy as np

import depth_estimator

# Pilot: rows come from range_trial.py --calibrate (a separate object and distances)
LOG = os.path.join(os.environ.get("WALLE_DATA_DIR")
                   or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data"),
                   "logs", "depth_calibration_log.csv")
MIN_FOR_POWER = 6


def percent_error(estimate, truth):
    return 100.0 * (estimate - truth) / truth


def fit_scale(raw, truth):
    return statistics.median(t / e for t, e in zip(truth, raw)), 1.0


def fit_power(raw, truth):
    power, log_scale = np.polyfit(np.log(raw), np.log(truth), 1)
    return float(np.exp(log_scale)), float(power)


def predict(fit, raw):
    return fit[0] * np.maximum(raw, 1e-3) ** fit[1]


def mean_abs_error(fit, raw, truth):
    return float(np.mean(np.abs(percent_error(predict(fit, raw), truth))))


def leave_one_out(fitter, raw, truth):
    errors = []
    for i in range(len(raw)):
        keep = np.arange(len(raw)) != i
        fit = fitter(raw[keep], truth[keep])
        errors.append(abs(percent_error(predict(fit, raw[i:i + 1]), truth[i:i + 1])[0]))
    return float(np.mean(errors))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--fit", choices=["auto", "scale", "power"], default="auto")
    parser.add_argument("--min", type=float, default=0.0, help="smallest real distance (m) to use")
    parser.add_argument("--max", type=float, default=99.0, help="largest real distance (m) to use")
    parser.add_argument("--reset", action="store_true", help="delete calibration.json")
    args = parser.parse_args()
    if args.reset:
        if os.path.exists(depth_estimator.CALIBRATION_FILE):
            os.remove(depth_estimator.CALIBRATION_FILE)
        print("Calibration deleted: distances are the model's own again.")
        return

    try:
        with open(LOG, newline="") as f:
            rows = list(csv.DictReader(f))
    except OSError:
        raise SystemExit("No depth_calibration_log.csv yet. Run: "
                         "python3 pilot/range_trial.py --calibrate")
    rows = [r for r in rows if r["true_m"] and r["target_found"] == "True" and r["target_raw_m"]
            and args.min <= float(r["true_m"]) <= args.max]
    if not rows:
        raise SystemExit("No calibration rows with the object found. "
                         "Run: python3 pilot/range_trial.py --calibrate")
    model = rows[-1]["model"]      # fit only the rows from the model you used last
    skipped = len([r for r in rows if r["model"] != model])
    rows = [r for r in rows if r["model"] == model]
    truth = np.array([float(r["true_m"]) for r in rows])
    raw = np.array([float(r["target_raw_m"]) for r in rows])
    if len(rows) < 3:
        raise SystemExit("Need at least 3 measurements with a real distance (6 or more is better).")

    scale_fit = fit_scale(raw, truth)
    can_power = len(rows) >= MIN_FOR_POWER and np.std(np.log(raw)) > 0.1
    power_fit = fit_power(raw, truth) if can_power else None

    print(f"Model: {model}   measurements: {len(rows)}" +
          (f"   (skipped {skipped} from other models)" if skipped else ""))
    print("  real (m)   model (m)   error now   scale only   scale + power")
    for t, e in zip(truth, raw):
        after_power = (f"{percent_error(predict(power_fit, np.array([e]))[0], t):+10.0f} %"
                       if power_fit else "         -")
        print(f"  {t:8.2f}   {e:9.2f}   {percent_error(e, t):+8.0f} %"
              f"   {percent_error(predict(scale_fit, np.array([e]))[0], t):+8.0f} %   {after_power}")
    print(f"Average error now:        {mean_abs_error((1.0, 1.0), raw, truth):.0f} %")
    loo_scale = leave_one_out(fit_scale, raw, truth) if len(rows) >= 3 else float("nan")
    print(f"Scale only:               {mean_abs_error(scale_fit, raw, truth):.0f} %   "
          f"(leave-one-out {loo_scale:.0f} %)   real = {scale_fit[0]:.3f} x model")
    chosen, name = scale_fit, "scale"
    if power_fit:
        loo_power = leave_one_out(fit_power, raw, truth)
        print(f"Scale + power:            {mean_abs_error(power_fit, raw, truth):.0f} %   "
              f"(leave-one-out {loo_power:.0f} %)   real = {power_fit[0]:.3f} x model ** {power_fit[1]:.2f}")
        if args.fit == "power" or (args.fit == "auto" and loo_power < loo_scale):
            chosen, name = power_fit, "scale + power"
    elif args.fit == "power":
        raise SystemExit(f"The power fit needs at least {MIN_FOR_POWER} measurements spread "
                         "over several distances.")
    if len(rows) < MIN_FOR_POWER or truth.max() - truth.min() < 0.8:
        print(f"Warning: use at least {MIN_FOR_POWER} measurements spread over at least 0.8 m, "
              "or the calibration may not be trustworthy.")

    scale, power = chosen
    if not (depth_estimator.SCALE_RANGE[0] <= scale <= depth_estimator.SCALE_RANGE[1]
            and depth_estimator.POWER_RANGE[0] <= power <= depth_estimator.POWER_RANGE[1]):
        raise SystemExit(f"The fit ({scale:.2f}, {power:.2f}) is outside the allowed range: "
                         "check your real distances. Calibration not saved.")
    with open(depth_estimator.CALIBRATION_FILE, "w") as f:
        json.dump({"scale": round(scale, 4), "power": round(power, 4),
                   "fit": name, "measurements": len(rows), "model": model}, f, indent=2)
    print(f"Saved config/depth_calibration.json ({name}). Commit it before session 1.")


if __name__ == "__main__":
    main()
