"""Move the camera tilt servo and check it got there (ROADMAP Phase 4).

The robot side must be up (`make launch` on the robot, `make sim-launch ...` in Gazebo). The
wheels never move: this sends nothing on /pico/cmd_vel. Run it in the container:

    python3 /workspace/walle-app/servo/servo_test.py --deg 15         # tilt up 15 deg
    python3 /workspace/walle-app/servo/servo_test.py --us 1600        # raw pulse (calibration)
    python3 /workspace/walle-app/servo/servo_test.py --sweep          # level, min, max, level
    python3 /workspace/walle-app/servo/servo_test.py --sweep --sim    # Gazebo calibration
    python3 /workspace/walle-app/servo/servo_test.py --deg 15 --stay  # stay tilted (photo sets)

Calibration steps are in walle-app/config/servo.yaml. Each move prints the pulse asked, the
readback from /pico/wheel_ticks [4], the angle that is, and the time it took. In Gazebo it
also prints the joint's true angle (/joint_states camera_tilt_joint), and --check makes the
run fail when any move ends more than 1 deg from where it was asked to go.
"""

import argparse
import math
import os
import sys
import threading
import time

APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, APP_DIR)

from servo.tilt import Calibration, Tilt, TiltError  # noqa: E402

JOINT = "camera_tilt_joint"


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--deg", type=float, nargs="+", help="tilt(s) to go to, deg (+ = up)")
    g.add_argument("--us", type=int, nargs="+", help="raw pulse width(s), us (calibration)")
    g.add_argument("--sweep", action="store_true", help="level, min_deg, max_deg, up_deg, level")
    p.add_argument("--sim", action="store_true", help="use servo.yaml's sim calibration")
    p.add_argument("--hold", type=float, default=1.0, help="seconds to stay at each angle")
    p.add_argument("--check", action="store_true",
                   help="Gazebo: FAIL if the joint ends > --tol deg from the asked angle")
    p.add_argument("--tol", type=float, default=1.0)
    p.add_argument("--stay", action="store_true",
                   help="leave the camera at the last angle (photo sets); default: level it")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    cal = Calibration.load(sim=args.sim)
    rclpy.init()
    node = Node("walle_servo_test")
    tilt = Tilt(node, cal)
    joint = {"deg": None}

    def on_joints(msg):
        if JOINT in msg.name:
            # URDF pitch about +y is + = DOWN; the tilt is + = up
            joint["deg"] = -math.degrees(msg.position[msg.name.index(JOINT)])

    node.create_subscription(JointState, "/joint_states", on_joints, 10)
    from rclpy.executors import SingleThreadedExecutor
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()

    if args.us:
        moves = [("us", u) for u in args.us]
    elif args.deg:
        moves = [("deg", d) for d in args.deg]
    else:
        moves = [("deg", d) for d in (0.0, cal.min_deg, cal.max_deg, cal.up_deg, 0.0)]

    t_wait = time.monotonic() + 3.0
    while time.monotonic() < t_wait and not tilt.has_servo():
        time.sleep(0.1)
    if not tilt.has_servo():
        print("No servo readback: /pico/wheel_ticks has no 5th value. Is the Phase 4 "
              "firmware flashed and the robot side up?")
    fails = []
    try:
        for kind, value in moves:
            if kind == "us":
                deg = cal.us_to_deg(value)
            else:
                deg = cal.clamp_deg(value)
                if deg != value:
                    print(f"  {value:+.1f} deg is outside {cal.min_deg:+.1f}..{cal.max_deg:+.1f}"
                          f": going to {deg:+.1f}")
            t0 = time.monotonic()
            try:
                if kind == "us":
                    tilt.send_us(value)
                    end = t0 + cal.readback_timeout_s + 2.0
                    while time.monotonic() < end and tilt.readback() != max(600, min(2400, value)):
                        time.sleep(0.02)
                    time.sleep(cal.settle_s)
                    reached = tilt.angle_deg()
                    if reached is None:
                        raise TiltError("no servo readback on /pico/wheel_ticks")
                else:
                    reached = tilt.set(deg)
            except TiltError as e:
                print(f"  {deg:+6.1f} deg: FAILED: {e}")
                fails.append(str(e))
                break
            time.sleep(max(0.0, args.hold))
            rb = tilt.readback()
            line = (f"  asked {deg:+6.1f} deg ({cal.deg_to_us(deg) if kind == 'deg' else value}"
                    f" us)  readback {rb} us = {reached:+6.1f} deg  "
                    f"[{time.monotonic() - t0:.1f} s]")
            if joint["deg"] is not None:
                err = joint["deg"] - deg
                line += f"  joint {joint['deg']:+6.2f} deg (error {err:+.2f})"
                if args.check and abs(err) > args.tol:
                    fails.append(f"joint {joint['deg']:+.2f} deg, asked {deg:+.2f}")
            print(line)
        if args.check:
            if joint["deg"] is None:
                fails.append("no /joint_states for camera_tilt_joint")
            print("PASS" if not fails else "FAIL: " + "; ".join(fails))
    finally:
        if not args.stay:
            try:
                tilt.level(settle=False)
            except TiltError:
                pass
        ex.shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
