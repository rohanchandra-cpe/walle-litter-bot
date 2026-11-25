"""Phase 6 agent: the fixture store's known history, and the tools that read it (no Ollama,
no ROS). The tool tests need langchain_core (in the containers); they skip on a host
without it."""

import json
import os
import shutil
import sys
import tempfile
import unittest
from datetime import datetime

APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, APP_DIR)
sys.path.insert(0, os.path.join(APP_DIR, "agent"))

from agent import fixture_store  # noqa: E402
from memory.store import MemoryStore  # noqa: E402

try:
    import walle_tools
except ImportError:       # no langchain_core on this host
    walle_tools = None


class FixtureTest(unittest.TestCase):
    """The fixture store holds exactly the history its FACTS describe."""

    @classmethod
    def setUpClass(cls):
        cls.dir = tempfile.mkdtemp()
        cls.path = fixture_store.build(os.path.join(cls.dir, "fixture.sqlite"))
        cls.mem = MemoryStore("sim", path=cls.path, embedder=None)

    @classmethod
    def tearDownClass(cls):
        cls.mem.close()
        shutil.rmtree(cls.dir)

    def test_most_seen(self):
        rows = self.mem.most_seen(limit=5)
        self.assertEqual([r["label"] for r in rows], fixture_store.FACTS["most_seen"])
        self.assertEqual({r["label"]: r["times_seen"] for r in rows},
                         fixture_store.FACTS["most_seen_times"])

    def test_overflow_ranking(self):
        rows = self.mem.bin_overflow_ranking()
        self.assertEqual([r["bin_id"] for r in rows], fixture_store.FACTS["overflow_ranking"])
        self.assertEqual({r["bin_id"]: r["times_overflowing"] for r in rows},
                         fixture_store.FACTS["overflow_times"])

    def test_recycling(self):
        r = self.mem.recycling_correctness()
        f = fixture_store.FACTS["recycling"]
        self.assertEqual((r["overall"]["correct"], r["overall"]["wrong"]),
                         (f["correct"], f["wrong"]))
        self.assertEqual(round(100 * r["overall"]["correct_rate"]), f["correct_percent"])
        self.assertEqual(sorted(w["label"] for w in r["wrong_items"]), sorted(f["wrong_items"]))

    def test_obstacles(self):
        now = self.mem.current_obstacles()
        self.assertEqual([(o["label"], o["side"]) for o in now], [("coffee_can", 3)])
        self.assertEqual(now[0]["times_seen"], 2)

    def test_bin_check_items(self):
        items = self.mem.bin_check_items(fixture_store.RUNS[3], 3)
        self.assertEqual(sorted(i["label"] for i in items["on_top"]), ["can", "paper_towel"])
        self.assertEqual(items["around"], [])


@unittest.skipIf(walle_tools is None, "langchain_core not installed")
class ToolsTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.dir = tempfile.mkdtemp()
        cls.path = fixture_store.build(os.path.join(cls.dir, "fixture.sqlite"))

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.dir)

    def setUp(self):
        self.runner = FakeRunner()
        self.asked = []
        walle_tools.configure("sim", store_path=self.path, runner=self.runner,
                              approve=lambda d: self.asked.append(d) or self.allow,
                              now=lambda: datetime.fromisoformat(fixture_store.NOW_UTC))
        self.allow = True

    def call(self, tool, **args):
        return json.loads(tool.invoke(args))

    def test_history_outputs(self):
        out = self.call(walle_tools.most_seen_objects)
        self.assertEqual(out["objects"][0]["label"], "milk_jug")
        self.assertEqual(out["from"]["store"], "sim")
        self.assertEqual(out["from"]["runs"], 5)
        out = self.call(walle_tools.bin_overflow_history)
        self.assertEqual(out["bins"][0]["bin"], 3)
        self.assertEqual(out["bins"][0]["overflow_percent"], 80)
        out = self.call(walle_tools.recycling_report)
        self.assertEqual(out["overall"]["correct_percent"], 75)
        out = self.call(walle_tools.report_obstacles)
        o = out["obstacles_now"]
        self.assertEqual(len(o), 1)
        self.assertIn("side 3 (corner 3 to corner 4), 0.40 m from corner 3", o[0]["where"])
        self.assertEqual(o[0]["hours_ago"], fixture_store.FACTS["obstacles_now"][0]["hours_ago"])
        out = self.call(walle_tools.last_bin_check, bin="Bin 3")
        self.assertEqual(out["check"]["outcome"], "clean")      # run 5 found it clean
        self.assertEqual(len(walle_tools.CALLS), 5)
        self.assertEqual(self.asked, [])                         # reading never asks

    def test_bin_numbers(self):
        for text, n in (("1", 1), ("bin 2", 2), ("Bin_3", 3), ("four", 4), ("4.0", 4),
                        ("5", None), ("x", None)):
            self.assertEqual(walle_tools._bin_number(text), n, text)

    def test_motion_asks_once_and_calls_runner(self):
        out = self.call(walle_tools.check_bin, bin="1")
        self.assertEqual(len(self.asked), 1)
        self.assertEqual(self.runner.calls, [("check_bin", 1)])
        self.assertEqual(out["previous_check"]["bin"], 1)
        self.call(walle_tools.run_route, check_bins="no")
        self.assertEqual(self.runner.calls[-1], ("run_route", False))

    def test_declined_does_not_move(self):
        self.allow = False
        out = self.call(walle_tools.run_route)
        self.assertEqual(out["status"], "declined")
        self.assertEqual(self.runner.calls, [])

    def test_bad_bin_does_not_move(self):
        out = self.call(walle_tools.check_bin, bin="7")
        self.assertIn("error", out)
        self.assertEqual((self.asked, self.runner.calls), ([], []))

    def test_no_runner(self):
        walle_tools.configure("sim", store_path=self.path, runner=None)
        out = self.call(walle_tools.look_ahead)
        self.assertIn("not available", out["error"])


@unittest.skipIf(walle_tools is None, "langchain_core not installed")
class DigestTest(unittest.TestCase):
    """A run folder -> the short summary the model gets (runner.digest_run)."""

    def test_robot_lap(self):
        import runner
        walle_tools.configure("robot")
        run = os.path.join(APP_DIR, "results", "route", "20261123_042112")
        d = runner.digest_run(run, "route")
        self.assertEqual(d["status"], "done")
        self.assertEqual(d["sides_checked_for_obstacles"], [1, 2, 3, 4])
        self.assertEqual(len(d["obstacles_found"]), 1)
        o = d["obstacles_found"][0]
        self.assertEqual(o["label"], "can")
        self.assertTrue(o["where"].startswith("side 1 (corner 1 to corner 2), 0.45 m"))
        self.assertEqual(o["what_the_robot_did"], "drove around it")
        self.assertEqual([b["bin"] for b in d["bins_checked"]], [1, 2, 3, 4])
        self.assertEqual(d["bins_checked"][0]["type"], "trash")

    def test_counts_not_lists(self):
        import runner
        self.assertEqual(runner.count_labels(["can", "bottle", "can", "cup", "bottle", "can"]),
                         {"can": 3, "bottle": 2, "cup": 1})
        self.assertEqual(list(runner.count_labels(["b", "a"])), ["a", "b"])

    def test_missing_run(self):
        import runner
        self.assertEqual(runner.digest_run(tempfile.gettempdir() + "/no_such_run")["status"],
                         "no record")


class FakeRunner:
    def __init__(self):
        self.calls = []

    def run_route(self, check_bins=True):
        self.calls.append(("run_route", check_bins))
        return {"status": "done"}

    def check_bin(self, n):
        self.calls.append(("check_bin", n))
        return {"status": "done", "bin": n}

    def look_ahead(self):
        self.calls.append(("look_ahead",))
        return {"status": "done"}

    def stop(self):
        return "stopped"


if __name__ == "__main__":
    unittest.main()
