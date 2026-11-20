"""Checks for the memory store (walle-app/memory/, ROADMAP Phase 2). No ROS, no Ollama.
    python3 -m unittest discover -s walle-app/tests

A fake run is written through the write API from the `mixed` scenario's ground truth (what
a perfect bin check and detector would report), then the read queries must give the answers
that ground truth implies.
"""

import json
import os
import shutil
import sys
import tempfile
import unittest
from collections import Counter

APP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [APP, os.path.join(APP, "route")]
import park  # noqa: E402
from memory.embed import FakeEmbedder  # noqa: E402
from memory.route_records import record_route_summary  # noqa: E402
from memory.store import MemoryStore, MemoryStoreError, load_config  # noqa: E402

T1, T2 = "2026-11-18T10:00:00+00:00", "2026-11-18T11:00:00+00:00"


def fake_run(store, layout, run_id, t, obstacle_present=True):
    """Record one lap as a perfect observer would: a bin check per bin, every item above a
    rim as on_bin, every item on the ground around a bin (spill + litter) as around_bin, and
    the obstacle (or its absence)."""
    store.start_run(run_id=run_id, kind="fake", scenario=layout["scenario"], layout=layout,
                    started_utc=t)
    for b in layout["bins"]:
        corners = layout["route"]["corners"]     # int keys; str after a JSON round trip
        corner = corners.get(b["corner"], corners.get(str(b["corner"])))
        pose = (corner[0], corner[1], b["view_heading_deg"])
        ground = b["spill"] + b["litter"]
        store.add_bin_check(run_id, b["id"], overflow_level=b["overflow_level"] or 0,
                            on_top_count=len(b["mound"]), litter_count=len(ground),
                            pose=pose, photo=f"fake/{run_id}/{b['name']}.jpg",
                            method="ground_truth", t_utc=t)
        for item in b["mound"]:
            store.add_detection(run_id, item["item"], item["category"], place="on_bin",
                                bin_id=b["id"], confidence=0.9, pose=pose,
                                obj_xy=item["position"][:2], model="truth", t_utc=t)
        for item in ground:
            store.add_detection(run_id, item["item"], item["category"], place="around_bin",
                                bin_id=b["id"], confidence=0.9, pose=pose,
                                obj_xy=item["position"][:2], model="truth", t_utc=t)
    o = layout["obstacle"]
    if o:
        if obstacle_present:
            det = store.add_detection(run_id, o["item"], o["category"], place="obstacle",
                                      obj_xy=o["position"][:2], model="truth", t_utc=t)
            store.report_obstacle(run_id, o["item"], o["category"], o["position"][:2],
                                  side=o["side"], detection_id=det, action="passed", t_utc=t)
        else:
            for cur in store.current_obstacles():
                store.mark_absent(run_id, cur["id"], t_utc=t)
    store.end_run(run_id, status="done", ended_utc=t, distance_m=4.0, end_error_m=0.01)


class MemoryStoreTests(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.cfg = load_config()
        self.layout = park.load_park("mixed")
        self.bins = {b["id"]: b for b in self.layout["bins"]}
        self.opened = []

    def tearDown(self):
        for s in self.opened:
            s.close()
        shutil.rmtree(self.dir)

    def store(self, source="sim", name=None, embedder=None):
        path = os.path.join(self.dir, name or f"{source}.sqlite")
        s = MemoryStore(source, path=path, embedder=embedder or FakeEmbedder(), cfg=self.cfg)
        self.opened.append(s)
        return s

    def two_runs(self, s):
        fake_run(s, self.layout, "run_a", T1, obstacle_present=True)
        fake_run(s, self.layout, "run_b", T2, obstacle_present=False)

    # ----- the agent's questions -----
    def test_most_seen(self):
        s = self.store()
        self.two_runs(s)
        seen = Counter()
        for b in self.layout["bins"]:
            seen.update(i["item"] for i in b["mound"] + b["spill"] + b["litter"])
        seen = Counter({k: 2 * v for k, v in seen.items()})       # two runs
        seen[self.layout["obstacle"]["item"]] += 1                 # obstacle: first run only
        top = s.most_seen(limit=10)
        self.assertEqual({r["label"]: r["times_seen"] for r in top}, dict(seen))
        self.assertEqual(top[0]["label"], seen.most_common(1)[0][0])
        self.assertEqual(top[0]["runs_seen"], 2)
        cats = {k: v["category"] for k, v in park.load_yaml(park.PARK_FILE)["items"].items()}
        for r in top:
            self.assertEqual(r["category"], cats[r["label"]])

    def test_bin_overflow_ranking(self):
        s = self.store()
        self.two_runs(s)
        rank = s.bin_overflow_ranking()
        self.assertEqual([r["bin_id"] for r in rank], [4, 2, 1, 3])  # level 3, level 1, ok bins
        by = {r["bin_id"]: r for r in rank}
        self.assertEqual(by[4]["times_overflowing"], 2)
        self.assertEqual(by[4]["mean_level"], 3)
        self.assertEqual(by[2]["mean_level"], 1)
        self.assertEqual(by[1]["times_clean"], 2)                    # ok, no litter
        self.assertEqual(by[3]["times_litter"], 2)                   # ok, litter 3
        self.assertEqual(by[3]["times_clean"], 0)
        self.assertEqual(by[3]["times_overflowing"], 0)
        last = s.last_bin_check(3)
        self.assertEqual((last["outcome"], last["litter_count"]), ("litter", 3))
        self.assertEqual(s.last_bin_check(1)["outcome"], "clean")
        self.assertEqual(s.last_bin_check(4)["outcome"], "overflowing")

    def test_recycling_correctness(self):
        s = self.store()
        self.two_runs(s)
        correct = wrong = 0
        for b in self.layout["bins"]:
            for item in b["mound"]:
                if item["category"] == b["type"]:
                    correct += 1
                else:
                    wrong += 1
        r = s.recycling_correctness()
        self.assertEqual(r["overall"]["correct"], 2 * correct)
        self.assertEqual(r["overall"]["wrong"], 2 * wrong)
        self.assertAlmostEqual(r["overall"]["correct_rate"], round(correct / (correct + wrong), 3))
        # only overflowing bins give evidence
        self.assertEqual({b["bin_id"] for b in r["per_bin"]},
                         {b["id"] for b in self.layout["bins"] if b["mound"]})
        for w in r["wrong_items"]:
            self.assertNotEqual(w["category"], w["bin_type"])
        around = sum(n["n"] for n in r["litter_around_bins"])
        self.assertEqual(around, 2 * sum(len(b["spill"] + b["litter"])
                                         for b in self.layout["bins"]))
        self.assertIn("above a bin's rim", r["basis"])

    def test_current_obstacles(self):
        s = self.store()
        o = self.layout["obstacle"]
        fake_run(s, self.layout, "run_a", T1, obstacle_present=True)
        cur = s.current_obstacles()
        self.assertEqual(len(cur), 1)
        self.assertEqual((cur[0]["label"], cur[0]["side"]), (o["item"], o["side"]))
        self.assertAlmostEqual(cur[0]["position"][0], o["position"][0])
        # seen again 5 cm away: the same obstacle, not a new one
        s.start_run(run_id="run_a2", started_utc=T1)
        oid = s.report_obstacle("run_a2", o["item"], o["category"],
                                (o["position"][0] + 0.05, o["position"][1]), side=o["side"],
                                t_utc="2026-11-18T10:30:00+00:00")
        self.assertEqual(oid, cur[0]["id"])
        self.assertEqual(s.current_obstacles()[0]["times_seen"], 2)
        # the next run passes and it is gone
        fake_run(s, self.layout, "run_b", T2, obstacle_present=False)
        self.assertEqual(s.current_obstacles(), [])
        hist = s.obstacle_history(oid)
        self.assertEqual([h["present"] for h in hist], [1, 1, 0])

    # ----- keeping things apart, persistence, safety -----
    def test_sim_and_robot_apart(self):
        sim, robot = self.store("sim"), self.store("robot")
        self.two_runs(sim)
        self.assertEqual(robot.counts()["runs"], 0)
        self.assertEqual(robot.most_seen(), [])
        sim.close()
        with self.assertRaises(MemoryStoreError):        # a sim file opened as robot
            self.store("robot", name="sim.sqlite")

    def test_survives_reopen(self):
        s = self.store()
        self.two_runs(s)
        before = (s.counts(), s.bin_overflow_ranking(), s.recycling_correctness())
        s.close()
        s2 = self.store()
        self.assertEqual((s2.counts(), s2.bin_overflow_ranking(), s2.recycling_correctness()),
                         before)
        self.assertEqual([r["run_id"] for r in s2.runs()], ["run_b", "run_a"])

    def test_two_writers(self):
        a, b = self.store(), self.store()                 # two connections, one file
        a.start_run(run_id="r1", started_utc=T1, layout=self.layout)
        b.add_bin_check("r1", 2, overflow_level=1, on_top_count=3, litter_count=0)
        a.add_detection("r1", "can", "recycling", place="on_bin", bin_id=2)
        self.assertEqual(a.counts()["bin_checks"], 1)
        self.assertEqual(b.counts()["detections"], 1)

    def test_rejects_bad_records(self):
        s = self.store()
        with self.assertRaises(MemoryStoreError):
            s.add_bin_check("no_such_run", 1)
        s.start_run(run_id="r1", started_utc=T1, layout=self.layout)
        with self.assertRaises(MemoryStoreError):
            s.start_run(run_id="r1")
        with self.assertRaises(MemoryStoreError):
            s.add_detection("r1", "can", "metal")
        with self.assertRaises(MemoryStoreError):
            s.add_detection("r1", "can", "recycling", place="on_bin")   # no bin
        with self.assertRaises(MemoryStoreError):
            s.add_bin_check("r1", 1, overflow_level=5)
        self.assertEqual(s.counts()["bin_checks"] + s.counts()["detections"], 0)

    def test_bin_type_from_layout(self):
        s = self.store()
        s.start_run(run_id="r1", started_utc=T1, layout=self.layout)
        s.add_detection("r1", "can", "recycling", place="on_bin", bin_id=4)
        s.add_bin_check("r1", 1, overflow_level=0, litter_count=0)
        self.assertEqual(self.bins[4]["type"], "recycling")
        self.assertEqual(s._rows("SELECT bin_type FROM detections")[0]["bin_type"], "recycling")
        self.assertEqual(s.last_bin_check(1)["bin_type"], "trash")

    # ----- free-text recall -----
    def test_recall(self):
        s = self.store()
        self.two_runs(s)
        self.assertEqual(s.counts()["unembedded"], 0)
        hits = s.recall("bin 4 overflowing level", k=3)
        self.assertEqual(hits[0]["method"], "vector")
        self.assertEqual(hits[0]["kind"], "bin_check")        # the toy embedder can't tell
        self.assertIn("overflowing", hits[0]["text"])        # bin 4 from bin 2; Ollama can

    def test_recall_without_ollama(self):
        s = self.store(embedder=FakeEmbedder(down=True))
        self.two_runs(s)
        n = s.counts()
        self.assertEqual(n["unembedded"], n["memory_text"])          # text kept, no vectors
        hits = s.recall("which bin was overflowing", k=3)
        self.assertEqual(hits[0]["method"], "keyword")
        self.assertIn("overflowing", hits[0]["text"])
        s.embedder = FakeEmbedder()                                    # Ollama back
        self.assertEqual(s.embed_pending(), n["memory_text"])
        self.assertEqual(s.counts()["unembedded"], 0)

    # ----- route follower logs -----
    def test_route_summary_import(self):
        layout = park.load_park("default")
        summary = {"run_id": "20261118_120000", "source": "robot", "scenario": "default",
                   "laps": 1, "status": "done", "reason": None,
                   "started_utc": T1, "ended_utc": T2, "distance_m": 4.0,
                   "end_errors": {"odom": {"position_m": 0.004}},
                   "measured": {"position_error_cm": 0.0},
                   "events": [{"event": "arrived", "corner": 2},
                              {"event": "facing_bin", "bin": "bin_2", "photo":
                               "photos/bin_2_step2.jpg", "poses": {"odom": [1.0, 0.0, -44.0]},
                               "t_utc": "2026-11-18T10:01:00+00:00"},
                              {"event": "facing_bin", "bin": "bin_3",
                               "poses": {"odom": [1.0, 1.0, 45.0]}}]}
        run_dir = os.path.join(APP, "results", "route", summary["run_id"])
        r = self.store("robot")
        record_route_summary(r, summary, layout, run_dir)
        run = r.runs()[0]
        self.assertEqual((run["status"], run["measured_error_cm"]), ("done", 0.0))
        self.assertEqual(run["run_dir"], "walle-app/results/route/20261118_120000")
        c = r.last_bin_check(2)
        self.assertEqual((c["method"], c["overflow_level"], c["outcome"]),
                         ("photo_only", None, None))
        self.assertEqual(c["photo"], "walle-app/results/route/20261118_120000/photos/"
                                     "bin_2_step2.jpg")
        self.assertEqual(r.last_bin_check(3)["t_utc"], T1)        # no event time: run start
        self.assertEqual(r.bin_overflow_ranking()[0]["assessed"], 0)
        with self.assertRaises(ValueError):                         # robot run into sim
            record_route_summary(self.store("sim"), summary, layout, run_dir)


class ParkBinTypeTests(unittest.TestCase):
    def test_types(self):
        types = {b["id"]: b["type"] for b in park.load_park("default")["bins"]}
        self.assertEqual(types, {1: "trash", 2: "recycling", 3: "trash", 4: "recycling"})

    def test_scenario_override_and_bad_type(self):
        path = os.path.join(tempfile.mkdtemp(), "s.yaml")
        with open(path, "w") as f:
            json.dump({"bins": {1: {"state": "ok", "type": "recycling"}}, "obstacle": "none"},
                      f)
        self.assertEqual(park.load_park(path)["bins"][0]["type"], "recycling")
        with open(path, "w") as f:
            json.dump({"bins": {1: {"type": "compost"}}, "obstacle": "none"}, f)
        with self.assertRaises(park.ParkError):
            park.load_park(path)


if __name__ == "__main__":
    unittest.main()
