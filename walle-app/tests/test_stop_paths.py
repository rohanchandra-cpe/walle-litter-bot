"""Stop paths found on the robot (run 20261125_200224): two launches publishing two poses,
and a second Ctrl+C breaking the follower's stop. No ROS needed."""

import os
import signal
import subprocess
import sys
import tempfile
import textwrap
import time
import unittest

APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(APP_DIR, "route"))
sys.path.insert(0, os.path.join(APP_DIR, "agent"))

import route_follower  # noqa: E402

try:
    import runner
except ImportError:       # no langchain/rclpy on this host
    runner = None


class FakeNode:
    def __init__(self, counts):
        self.counts = counts

    def count_publishers(self, topic):
        return self.counts.get(topic, 1)


class SinglePublisherTest(unittest.TestCase):
    cfg = {"feedback_topic": "/odometry/filtered"}

    def test_one_each_passes(self):
        route_follower.check_single_publishers(FakeNode({}), self.cfg, wait_s=0)

    def test_two_ekfs_refused(self):
        with self.assertRaises(RuntimeError) as e:
            route_follower.check_single_publishers(
                FakeNode({"/odometry/filtered": 2}), self.cfg, wait_s=0)
        self.assertIn("/odometry/filtered (2)", str(e.exception))
        self.assertIn("twice", str(e.exception))


CHILD = textwrap.dedent("""
    import signal, sys, time
    hits = []
    def on_int(sig, frame):
        hits.append(1)
        if len(hits) > 1:
            print("SECOND", flush=True)
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, on_int)
    print("ready", flush=True)
    try:
        while True:
            time.sleep(0.05)
    except KeyboardInterrupt:
        time.sleep(0.5)               # stopping the robot and saving
        print("saved", flush=True)
""")


@unittest.skipIf(runner is None, "runner needs the container's packages")
class InterruptTest(unittest.TestCase):
    """runner._interrupt: one SIGINT reaches the follower, never two."""

    def start(self):
        path = os.path.join(tempfile.mkdtemp(), "child.py")
        with open(path, "w") as f:
            f.write(CHILD)
        r = runner.Runner.__new__(runner.Runner)
        r.cfg = {"stop_wait_s": 3}
        r.sent_zero = 0
        r._zero_cmd_vel = lambda: setattr(r, "sent_zero", r.sent_zero + 1)
        r.proc = subprocess.Popen([sys.executable, "-u", path], stdout=subprocess.PIPE,
                                  text=True)
        self.assertEqual(r.proc.stdout.readline().strip(), "ready")
        return r

    def test_terminal_ctrl_c_not_repeated(self):
        r = self.start()
        r.proc.send_signal(signal.SIGINT)        # what the terminal sends to the group
        r._interrupt(signalled=True)
        out = r.proc.stdout.read()
        self.assertIn("saved", out)
        self.assertNotIn("SECOND", out)
        self.assertEqual(r.sent_zero, 1)

    def test_tool_stop_sends_one(self):
        r = self.start()
        r._interrupt()
        out = r.proc.stdout.read()
        self.assertIn("saved", out)
        self.assertNotIn("SECOND", out)

    def test_hung_follower_killed(self):
        r = self.start()
        r.proc.send_signal(signal.SIGSTOP)       # hangs: no answer to anything
        t0 = time.monotonic()
        r._interrupt(signalled=True)
        self.assertIsNotNone(r.proc.poll())
        self.assertLess(time.monotonic() - t0, 3 + 5 + 6)


if __name__ == "__main__":
    unittest.main()
