"""Start-of-session checklist: power, environment, live sensor checks, and the manifest.

Run inside the container while launch/pilot.launch.py is running:
    python3 pilot/session_check.py --session 1
    python3 pilot/session_check.py --session 1 --no-camera   # motion-only session

Asks for the supply and ground readings, the room conditions, and who is operating
and measuring; holds the robot still for the IMU; measures every topic's rate, the
ultrasonic ping status, and the camera; then appends one row to session_log.csv and
writes data/manifests/session_<n>.json. Motion and range trials refuse to start in
a session that has no checklist row.
"""

import argparse
import time
import urllib.request

from common import SESSION_LOG, append_row, ask, ask_float, ask_yes, commit_label, load_config, utc_now
from manifest import write_manifest
from robot_io import Robot

FIELDS = [
    "session_id", "timestamp_utc", "commit", "operator", "measurer",
    "motor_supply_v", "logic_supply_v", "ground_offset_v", "driver_vcc_stby_v",
    "emergency_stop_tested", "temp_c", "lux", "floor", "imu_still_s", "imu_calibrated",
    "rate_ticks_hz", "rate_imu_hz", "rate_wheel_odom_hz", "rate_ekf_hz",
    "ultrasonic_status", "camera_ok", "camera_ms", "warnings", "manifest", "notes",
]
IMU_STILL_S = 30


def check_camera(url, timeout):
    t0 = time.monotonic()
    try:
        with urllib.request.urlopen(url, timeout=timeout) as reply:
            size = len(reply.read())
        return size > 1000, 1000.0 * (time.monotonic() - t0)
    except OSError as error:
        print(f"  Camera error: {error}")
        return False, float("nan")


def main():
    parser = argparse.ArgumentParser(description="Start-of-session checklist")
    parser.add_argument("--session", type=int, required=True)
    parser.add_argument("--no-camera", action="store_true")
    args = parser.parse_args()
    cfg = load_config()
    warnings = []

    print(f"=== Session {args.session} checklist ===")
    row = {"session_id": args.session, "operator": ask("Operator initials: "),
           "measurer": ask("Measurer initials: ")}
    print("\nPower (multimeter):")
    row["motor_supply_v"] = ask_float("  Motor supply (six AA), V: ")
    row["logic_supply_v"] = ask_float("  Logic supply (5 V rail), V: ")
    row["ground_offset_v"] = ask_float("  Pico GND to TB6612 GND, V: ")
    row["driver_vcc_stby_v"] = ask_float("  TB6612 VCC and STBY at the chip, V: ")
    if abs(row["ground_offset_v"]) >= 0.05:
        warnings.append("ground offset >= 0.05 V")
        print("  STOP: ground offset of 0.05 V or more. Rewire the grounds directly "
              "(this caused the motor fade in Module 8).")
    if row["driver_vcc_stby_v"] < 2.7:
        warnings.append("TB6612 VCC/STBY < 2.7 V")
        print("  STOP: TB6612 VCC/STBY below its 2.7 V minimum.")
    row["emergency_stop_tested"] = ask_yes("  Emergency stop tested (Ctrl+C stops the robot)?")
    if not row["emergency_stop_tested"]:
        warnings.append("emergency stop not tested")

    print("\nEnvironment:")
    row["temp_c"] = ask_float("  Room temperature, deg C: ")
    row["lux"] = ask_float("  Light meter reading at the arena, lux (Enter to skip): ",
                           allow_blank=True)
    row["floor"] = ask("  Floor area / surface (e.g. 'living room carpet, lane 1'): ")
    if not ask_yes("  Ceiling lights on, blinds closed, area clear of people and objects?"):
        warnings.append("environment not as specified")

    with Robot() as robot:
        missing = robot.wait_for(("ticks", "wheel", "ekf"), timeout=10.0)
        if missing:
            warnings.append("no data: " + ", ".join(missing))
            print(f"\n  WARNING: no data from {', '.join(missing)}. Is the launch file "
                  "running, the agent up, and the robot powered?")
        input(f"\nHold the robot completely still for {IMU_STILL_S} s, then press Enter "
              "to start the timer...")
        for remaining in range(IMU_STILL_S, 0, -5):
            print(f"  {remaining} s...")
            time.sleep(5)
        row["imu_still_s"] = IMU_STILL_S
        if robot.wait_for(("imu",), timeout=5.0):
            warnings.append("no IMU data")
            print("  WARNING: no /pico/imu/data. The firmware publishes it only after the "
                  "gyroscope calibrates; check the BNO055 wiring and the Pico's USB log.")
        row["imu_calibrated"] = robot.imu_calibrated
        if not robot.imu_calibrated:
            warnings.append("IMU gyroscope not reported calibrated")
            print("  WARNING: the IMU is not reporting a calibrated gyroscope. Keep it still "
                  "longer and rerun; a 30 s wait alone does not establish calibration.")

        print("\nMeasuring topic rates for 3 s...")
        rates = robot.rates(3.0)
        row.update(rate_ticks_hz=rates["ticks"], rate_imu_hz=rates["imu"],
                   rate_wheel_odom_hz=rates["wheel"], rate_ekf_hz=rates["ekf"])
        print(f"  wheel ticks {rates['ticks']:.0f} Hz, IMU {rates['imu']:.0f} Hz, "
              f"wheel odom {rates['wheel']:.0f} Hz, EKF {rates['ekf']:.0f} Hz "
              "(expected about 50 each)")
        if min(rates.values()) < 40:
            warnings.append("a topic below 40 Hz")

        pings = robot.collect_pings(10, 2.0)
        status = {}
        for _, echo in pings:
            key = "lost" if echo is None else ("ok" if echo > 0 else
                                               {-1: "timeout", -2: "no_response"}.get(echo, "other"))
            status[key] = status.get(key, 0) + 1
        row["ultrasonic_status"] = " ".join(f"{k}:{v}" for k, v in sorted(status.items())) or "none"
        print(f"  ultrasonic pings (10): {row['ultrasonic_status']}")
        if status.get("ok", 0) < 5:
            warnings.append("ultrasonic mostly invalid")

    if not args.no_camera:
        ok, ms = check_camera(cfg["ranging"]["camera_url"], cfg["ranging"]["camera_timeout_s"])
        row["camera_ok"], row["camera_ms"] = ok, ms
        print(f"  camera: {'OK' if ok else 'NOT reachable'} ({ms:.0f} ms)")
        if not ok:
            warnings.append("camera not reachable")

    row["notes"] = ask("\nNotes (Enter for none): ", allow_blank=True)
    row["manifest"] = write_manifest(args.session).split("/data/", 1)[-1]
    row.update(timestamp_utc=utc_now(), commit=commit_label(), warnings="; ".join(warnings))
    append_row(SESSION_LOG, FIELDS, row)
    print(f"\nSaved session {args.session} to session_log.csv and {row['manifest']}.")
    if warnings:
        print("Fix before main trials: " + "; ".join(warnings))
    print("Next: run the two warm-up runs, which motion_trial.py schedules first:\n"
          f"  python3 pilot/motion_trial.py --session {args.session}")


if __name__ == "__main__":
    main()
