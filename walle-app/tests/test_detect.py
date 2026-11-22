"""Checks for detection (walle-app/detect/, ROADMAP Phase 3).
    python3 -m unittest discover -s walle-app/tests

Geometry, place rules and config need no model. The model test runs only where ultralytics
is installed (the containers) and the weights are in walle-app/models/.
"""

import importlib.util
import math
import os
import sys
import unittest

APP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [APP, os.path.join(APP, "route")]
import park  # noqa: E402
from detect import geometry as g  # noqa: E402
from detect.detector import (MODELS_DIR, VLMDetector, load_config, merge_boxes,  # noqa: E402
                             vocabulary)

CFG = load_config()
CAM = g.Camera()          # 640 x 480, 60 x 47 deg, 0.103 m high, 0.12 m ahead


def det(kind, bbox, label="bin", category=None, conf=0.5):
    return {"kind": kind, "label": label, "category": category, "conf": conf, "bbox": bbox}


class TestCamera(unittest.TestCase):
    def test_bearing(self):
        self.assertAlmostEqual(CAM.bearing_deg(320), 0.0)
        self.assertAlmostEqual(CAM.bearing_deg(0), 30.0, places=6)      # left edge = +30
        self.assertAlmostEqual(CAM.bearing_deg(640), -30.0, places=6)

    def test_ground_point_round_trip(self):
        # a point on the floor 0.5 m ahead of the camera, 0.1 m to the left
        x, y = 0.5, 0.1
        u = 320 - CAM.fx * y / x
        v = 240 + CAM.fy * CAM.height_m / x
        gx, gy = CAM.ground_point(u, v)
        self.assertAlmostEqual(gx, CAM.forward_m + x, places=6)
        self.assertAlmostEqual(gy, y, places=6)
        self.assertIsNone(CAM.ground_point(320, 240))      # horizon
        self.assertIsNone(CAM.ground_point(320, 100))      # above it

    def test_robot_to_park(self):
        px, py = g.robot_to_park((1.0, 0.0), (1.0, 2.0, 90.0))
        self.assertAlmostEqual(px, 1.0)
        self.assertAlmostEqual(py, 3.0)

    def test_from_park(self):
        cam = g.Camera.from_park(park.load_yaml(park.PARK_FILE)["robot"]["camera"])
        self.assertAlmostEqual(cam.bearing_deg(0), 30.0, places=6)
        self.assertAlmostEqual(cam.height_m, 0.125)       # on the Phase 4 tilt mount
        self.assertAlmostEqual(cam.left_m, 0.011)


class TestBin(unittest.TestCase):
    def test_cut_edges(self):
        self.assertEqual(g.cut_edges([0, 0, 300, 400], CAM, 4), {"left", "top"})
        self.assertEqual(g.cut_edges([100, 50, 500, 470], CAM, 4), set())

    def test_center_whole_bin(self):
        self.assertEqual(g.bin_center_u([200, 50, 400, 400], CAM, 4, 300), 300)

    def test_center_cut_on_right(self):
        # visible from 400 to the right edge; the bin is 400 px wide -> center at 600
        self.assertEqual(g.bin_center_u([400, 0, 640, 400], CAM, 4, 400), 600)
        # a bin nearer than expected (box wider than expected) keeps its own center
        self.assertEqual(g.bin_center_u([200, 0, 640, 400], CAM, 4, 300), 420)

    def test_center_cut_on_left(self):
        self.assertEqual(g.bin_center_u([0, 0, 214, 433], CAM, 4, 400), 14)

    def test_center_fills_frame(self):
        self.assertEqual(g.bin_center_u([0, 0, 640, 400], CAM, 4, 400), 320)

    def test_pick_most_confident_then_nearest_expected(self):
        left, right = det("bin", [0, 100, 100, 300], conf=0.9), det("bin", [400, 100, 560, 300])
        self.assertIs(g.pick_bin([left, right], CAM, 0.0), left)        # confidence first
        a, b = det("bin", [0, 100, 100, 300], conf=1.0), det("bin", [400, 100, 560, 300], conf=1.0)
        self.assertIs(g.pick_bin([a, b], CAM, 0.0), b)                   # tie: nearest expected
        self.assertIs(g.pick_bin([a, b], CAM, 25.0), a)
        self.assertIsNone(g.pick_bin([], CAM))

    # Robot run 20261121_000650, bin_2: YOLO-World's bin boxes; the bin was ~253 x 313 px.
    EXPECTED = (253.0, 313.0)

    def test_robot_run_junk_box_is_not_the_bin(self):
        look0 = [det("bin", [178.3, 262.8, 194.6, 289.0], conf=0.237)]   # a bowl, 16 x 26 px
        self.assertIsNone(g.pick_bin(look0, CAM, -1.5, self.EXPECTED))
        edge = [det("bin", [0.0, 300.0, 29.6, 352.7], conf=0.048)]       # cut at the left, short
        self.assertIsNone(g.pick_bin(edge, CAM, 0.0, self.EXPECTED))

    def test_robot_run_real_bin_beats_box_near_expected(self):
        look2 = [det("bin", [218.4, 35.7, 481.0, 348.7], conf=0.209),
                 det("bin", [0.0, 0.0, 213.2, 210.4], conf=0.052),
                 det("bin", [472.8, 211.3, 518.9, 249.8], conf=0.034)]
        chosen = g.pick_bin(look2, CAM, -20.33, self.EXPECTED)
        self.assertIs(chosen, look2[0])
        self.assertAlmostEqual(CAM.bearing_deg((218.4 + 481.0) / 2), -2.9, delta=0.2)

    def test_search_goes_first_toward_a_cut_off_bin(self):
        # run 20261121_035910, bin 2 look 0: the bin cut at the LEFT edge at 0.016, a cushion
        # cut at the right at 0.011
        boxes = [det("bin", [0, 25, 243, 351], conf=0.016), det("bin", [513, 180, 640, 332], conf=0.011)]
        self.assertEqual(g.search_order(boxes, CAM, 4, self.EXPECTED, 15.0), [15.0, -15.0])
        right = [det("bin", [450, 30, 640, 340], conf=0.02)]
        self.assertEqual(g.search_order(right, CAM, 4, self.EXPECTED, 15.0), [-15.0, 15.0])
        short = [det("bin", [500, 300, 640, 340], conf=0.02)]           # too short to be it
        self.assertEqual(g.search_order(short, CAM, 4, self.EXPECTED, 15.0), [15.0, -15.0])
        self.assertEqual(g.search_order([], CAM, 4, self.EXPECTED, 15.0), [15.0, -15.0])

    def test_bin_cut_at_a_side_is_judged_on_height(self):
        cut = det("bin", [0.0, 40.0, 120.0, 345.0])
        self.assertTrue(g.plausible_bin(cut, CAM, 4, self.EXPECTED))


class TestPlace(unittest.TestCase):
    BIN = [200, 150, 440, 420]

    def test_on_bin(self):
        self.assertEqual(g.item_place([280, 90, 340, 155], self.BIN, True, 8, 0.3), "on_bin")

    def test_beside_bin(self):
        self.assertEqual(g.item_place([460, 380, 520, 430], self.BIN, True, 8, 0.3),
                         "around_bin")

    def test_in_front_of_bin(self):
        # overlaps the bin horizontally but stands on the floor in front of it
        self.assertEqual(g.item_place([280, 360, 340, 440], self.BIN, True, 8, 0.3),
                         "around_bin")

    def test_rim_out_of_view(self):
        self.assertEqual(g.item_place([280, 90, 340, 155], self.BIN, False, 8, 0.3),
                         "around_bin")

    def test_analyze(self):
        dets = [det("bin", self.BIN), det("item", [280, 90, 340, 155], "can", "recycling"),
                det("item", [460, 380, 520, 430], "paper_towel", "trash")]
        out = g.analyze_bin_stop(dets, CAM, pose=(0.0, 0.0, 0.0), cfg=CFG)
        self.assertTrue(out["rim_visible"])
        self.assertAlmostEqual(out["bin_bearing_deg"], 0.0)
        on, around = out["items"]
        self.assertEqual((on["place"], on["obj_xy"]), ("on_bin", None))
        self.assertEqual(around["place"], "around_bin")
        x, y = around["obj_xy"]
        self.assertTrue(0.3 < x < 1.0 and y < 0, around["obj_xy"])   # ahead, to the right

    def test_analyze_no_bin(self):
        out = g.analyze_bin_stop([det("item", [10, 300, 60, 350], "foil", "recycling")], CAM,
                                 cfg=CFG)
        self.assertIsNone(out["bin"])
        self.assertEqual(out["items"][0]["place"], "around_bin")


class TestConfig(unittest.TestCase):
    def test_sim_items_agree_with_park(self):
        """Every item the simulator can place is in the vocabulary with the same category."""
        items = park.load_yaml(park.PARK_FILE)["items"]
        for name, item in items.items():
            self.assertIn(name, CFG["items"], f"{name} missing from detect.yaml")
            self.assertEqual(CFG["items"][name]["category"], item["category"], name)

    def test_categories(self):
        for name, item in CFG["items"].items():
            self.assertIn(item["category"], ("trash", "recycling"), name)

    def test_user_labels(self):
        """The user's labels (2026-11-19)."""
        trash = {n for n, i in CFG["items"].items() if i["category"] == "trash"}
        self.assertTrue({"paper_towel", "plastic_bag"} <= trash)
        self.assertFalse({"foil", "paper_bag", "can", "milk_jug", "yogurt_container"} & trash)

    def test_vocabulary(self):
        classes = vocabulary(CFG)
        self.assertTrue(any(c[1] == "bin" for c in classes))
        self.assertEqual(len({c[0] for c in classes}), len(classes))


class TestEvalScoring(unittest.TestCase):
    """eval_photos.score_photo / summarize, on hand-made detections (no model)."""

    def setUp(self):
        from detect import eval_photos
        self.ev = eval_photos

    def test_counts(self):
        truth = {"id": 1, "file": "a.jpg", "bin": 1, "bin_shift_cm": 0,
                 "items": [{"label": "paper_towel", "place": "on_bin"},
                           {"label": "can", "place": "around_bin"}]}
        dets = [det("bin", [200, 150, 440, 420]),
                det("item", [280, 90, 340, 155], "paper_bag", "recycling"),   # wrong label + cat
                det("item", [460, 380, 520, 430], "can", "recycling")]
        r = self.ev.score_photo(truth, dets, CAM, CFG)
        self.assertEqual((r["found"], r["true_items"], r["reported_items"]), (1, 2, 2))
        self.assertEqual((r["category_right"], r["category_matchable"]), (1, 2))
        self.assertEqual((r["place_right"], r["place_checked"]), (1, 1))
        self.assertTrue(r["bin_found"])
        self.assertEqual((r["missed"], r["extra"]), (["paper_towel"], ["paper_bag"]))
        s = self.ev.summarize([r], [0.3], CFG["eval"])
        self.assertEqual((s["recall"], s["precision"], s["category"]), (0.5, 0.5, 0.5))
        self.assertFalse(s["pass"])
        self.assertTrue(s["checks"]["latency_median_s"]["pass"])

    def test_route_photo_all_right(self):
        truth = {"id": 2, "file": "b.jpg", "bin": None,
                 "items": [{"label": "foil", "place": "route"}]}
        r = self.ev.score_photo(truth, [det("item", [10, 300, 60, 350], "foil", "recycling")],
                                CAM, CFG)
        s = self.ev.summarize([r], [0.3], CFG["eval"])
        for key in ("recall", "precision", "category"):
            self.assertEqual(s[key], 1.0)
        self.assertIsNone(s["bin_found"])      # no photo with a bin -> not judged (fails)


class TestSimTruth(unittest.TestCase):
    """The Gazebo ground-truth stand-in, seen from a bin stop, through the real analysis."""

    def setUp(self):
        from detect.sim_truth import TruthDetector, camera_settings
        self.layout = park.load_park("mixed", obstacle="none")
        self.b4 = next(b for b in self.layout["bins"] if b["id"] == 4)
        corner = self.layout["route"]["corners"][4]
        self.pose = (corner[0], corner[1], math.radians(self.b4["view_heading_deg"]))
        self.det = TruthDetector(self.layout, lambda: self.pose, cfg=CFG)
        self.cam = g.Camera.from_park(camera_settings(CFG), 320, 240)
        self.img = type("Img", (), {"size": (320, 240)})()     # only its size is used

    def test_bin_stop(self):
        res = self.det.detect(self.img)
        bins = [d for d in res["detections"] if d["kind"] == "bin"]
        self.assertEqual([d["truth"] for d in bins], ["bin_4"])
        pose_deg = (self.pose[0], self.pose[1], math.degrees(self.pose[2]))
        an = g.analyze_bin_stop(res["detections"], self.cam, pose=pose_deg, cfg=CFG)
        self.assertLess(abs(an["bin_bearing_deg"]), 0.5)
        self.assertTrue(an["rim_visible"])
        groups = {d["truth_group"]: set() for d in an["items"]}
        for d in an["items"]:
            groups[d["truth_group"]].add(d["place"])
        self.assertEqual(groups.get("mound"), {"on_bin"})
        for grp in ("spill", "litter"):
            if grp in groups:
                self.assertEqual(groups[grp], {"around_bin"}, grp)
        # around_bin items: obj_xy within 15 cm of the truth
        truth = {it["name"]: it["position"] for it in self.b4["spill"] + self.b4["litter"]}
        for d in an["items"]:
            if d["truth"] in truth and d["obj_xy"]:
                t = truth[d["truth"]]
                self.assertLess(math.hypot(d["obj_xy"][0] - t[0], d["obj_xy"][1] - t[1]), 0.15,
                                d["truth"])

    def test_every_bin_places(self):
        """At every bin stop of `mixed`: items on top are on_bin (incl. bin 2's bottles that
        overhang the near rim), items on the floor around_bin."""
        want = {"mound": "on_bin", "spill": "around_bin", "litter": "around_bin"}
        for b in self.layout["bins"]:
            corner = self.layout["route"]["corners"][b["corner"]]
            self.pose = (corner[0], corner[1], math.radians(b["view_heading_deg"]))
            an = g.analyze_bin_stop(self.det.detect(self.img)["detections"], self.cam, cfg=CFG)
            for d in an["items"]:
                self.assertEqual(d["place"], want[d["truth_group"]], f"{b['name']} {d['truth']}")

    def test_turned_away(self):
        self.pose = (self.pose[0], self.pose[1], self.pose[2] + math.radians(10))
        an = g.analyze_bin_stop(self.det.detect(self.img)["detections"], self.cam, cfg=CFG)
        # the camera is ahead of the axle, so turning moves it: expect the bearing of the
        # bin's center from where the camera now is (the box's projection adds a little)
        x, y, h = self.pose
        cx, cy = x + self.cam.forward_m * math.cos(h), y + self.cam.forward_m * math.sin(h)
        bx, by = self.b4["position"]
        expected = math.degrees(math.atan2(by - cy, bx - cx) - h)
        self.assertAlmostEqual(an["bin_bearing_deg"], expected, delta=1.0)


class TestSimHidesCutBins(unittest.TestCase):
    def test_cut_bin_hidden_whole_bin_kept(self):
        from detect.sim_truth import TruthDetector

        class Img:
            size = (320, 240)
        layout = park.load_park("default", bins=4)
        b = layout["bins"][1]
        corner = layout["route"]["corners"][b["corner"]]
        head = math.radians(b["view_heading_deg"])
        seen = TruthDetector(layout, lambda: (corner[0], corner[1], head), hide_cut_bins=True)
        bins = [d for d in seen.detect(Img())["detections"] if d["kind"] == "bin"]
        self.assertEqual([d["truth"] for d in bins], [b["name"]])
        off = TruthDetector(layout, lambda: (corner[0], corner[1], head + math.radians(28)),
                            hide_cut_bins=True)
        self.assertEqual([d for d in off.detect(Img())["detections"] if d["kind"] == "bin"], [])


class TestTurnScale(unittest.TestCase):
    def test_parallax(self):
        import route_geometry as rg
        self.assertAlmostEqual(rg.parallax_factor(0.755, 0.125), 1.166, places=3)

    def test_learns_the_robot_runs_ratio(self):
        import route_geometry as rg
        par = rg.parallax_factor(0.65, 0.125)
        s = 1.0
        # bin_3 in run 20261121_000650: -15.1 -> +13.4 after a recorded -17.7 deg turn
        s = rg.learn_turn_scale(s, -15.12, 13.38, -17.71, par, gain=1.0)
        self.assertAlmostEqual(s, 28.5 / 17.71 / par, places=2)
        self.assertGreater(s, 1.3)

    def test_corrected_turn_lands_on_the_bin(self):
        import route_geometry as rg
        par, scale = rg.parallax_factor(0.65, 0.125), 1.37
        bearing = -15.12
        turn = bearing / (par * scale)          # what the follower asks the EKF for
        self.assertAlmostEqual(bearing - turn * par * scale, 0.0)   # bin centered
        self.assertLess(abs(turn), abs(bearing))

    def test_ignores_tiny_and_wrong_way_turns(self):
        import route_geometry as rg
        self.assertEqual(rg.learn_turn_scale(1.2, 2.0, 1.0, 0.5, 1.1, 0.6), 1.2)
        self.assertEqual(rg.learn_turn_scale(1.2, -10.0, -14.0, -5.0, 1.1, 0.6), 1.2)
        self.assertEqual(rg.learn_turn_scale(1.2, 30.0, -30.0, 5.0, 1.0, 1.0), 2.5)   # limit


class TestSensorGuardMath(unittest.TestCase):
    def test_pulse_outlier(self):
        import route_geometry as rg
        # run 20261121_041923: planned ~12.9 deg (gain 115, 0.152 s), moved 46.1 by the EKF
        self.assertTrue(rg.pulse_outlier(115.3, 0.152, 0.04, 46.1, 2.5))
        self.assertFalse(rg.pulse_outlier(110.0, 0.178, 0.04, 16.6, 2.5))
        self.assertTrue(rg.pulse_outlier(110.0, 0.178, 0.04, 2.0, 2.5))
        self.assertFalse(rg.pulse_outlier(110.0, 0.045, 0.04, 9.0, 2.5))   # too short to judge

    def test_bridge_gap(self):
        import route_geometry as rg
        # stopped 8 cm short while the EKF went on: the wheels say 1 cm of coast, no turn
        park0 = (0.08, 1.0, math.pi)
        out = rg.bridge_gap(park0, (2.0, 3.0, 0.5), (2.0 + 0.01 * math.cos(0.5),
                                                     3.0 + 0.01 * math.sin(0.5), 0.5))
        self.assertAlmostEqual(out[0], 0.07, places=6)
        self.assertAlmostEqual(out[1], 1.0, places=6)
        self.assertAlmostEqual(out[2], math.pi, places=6)
        turned = rg.bridge_gap((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, 0.0, 1.2))
        self.assertAlmostEqual(turned[2], 0.2, places=6)

    def test_gaps_in(self):
        import route_geometry as rg
        gaps = [(1.0, 2.0, "imu"), (5.0, 5.5, "ticks")]
        self.assertEqual(rg.gaps_in(gaps, 1.5, 3.0), [gaps[0]])
        self.assertEqual(rg.gaps_in(gaps, 2.1, 4.9), [])
        self.assertEqual(rg.gaps_in(gaps, 0.0, 9.0), gaps)


class TestCenteringTarget(unittest.TestCase):
    def test_within_limit(self):
        import route_geometry as rg
        t, clamped = rg.centering_target(0.0, -8.0, 0.0, 25.0)
        self.assertAlmostEqual(math.degrees(t), -8.0)
        self.assertFalse(clamped)

    def test_clamped(self):
        import route_geometry as rg
        cfg = math.radians(-135.0)
        t, clamped = rg.centering_target(math.radians(-150.0), -20.0, cfg, 25.0)
        self.assertAlmostEqual(math.degrees(t), -160.0)
        self.assertTrue(clamped)


class TestDetectionsImport(unittest.TestCase):
    """detections.json in a run folder -> memory store (route_records), as import_runs.py
    rebuilds it."""

    def test_round_trip(self):
        import json
        import shutil
        import tempfile
        from memory.embed import FakeEmbedder
        from memory.route_records import record_route_summary
        from memory.store import MemoryStore
        tmp = tempfile.mkdtemp()
        try:
            layout = park.load_park("default")
            summary = {"run_id": "t1", "source": "sim", "scenario": "default", "laps": 1,
                       "started_utc": "2026-11-19T10:00:00+00:00", "status": "done",
                       "events": [{"event": "facing_bin", "bin": "bin_2",
                                   "poses": {"odom": [1.0, 0.0, -45.0]}}]}
            dets = {"run_id": "t1", "model": "yolov8s-worldv2.pt", "photos": [
                {"bin": "bin_2", "bin_id": 2, "photo": "photos/bin_2_step2.jpg",
                 "pose": [1.0, 0.0, -45.0], "t_utc": "2026-11-19T10:01:00+00:00",
                 "items": [
                     {"label": "can", "category": "recycling", "conf": 0.6,
                      "bbox": [1, 2, 3, 4], "place": "on_bin", "obj_xy": None},
                     {"label": "paper_towel", "category": "trash", "conf": 0.4,
                      "bbox": [5, 6, 7, 8], "place": "around_bin", "obj_xy": [1.3, -0.3]}],
                 "low_conf": [{"label": "foil", "conf": 0.1}]}]}
            with open(os.path.join(tmp, "detections.json"), "w") as f:
                json.dump(dets, f)
            store = MemoryStore("sim", path=os.path.join(tmp, "m.sqlite"),
                                embedder=FakeEmbedder())
            record_route_summary(store, summary, layout, tmp)
            rows = store._rows("SELECT label, category, place, bin_id, bin_type, obj_x, "
                               "model FROM detections ORDER BY label")
            store.close()
        finally:
            shutil.rmtree(tmp)
        self.assertEqual(len(rows), 2)                       # low_conf not stored
        can, towel = rows
        self.assertEqual((can["place"], can["bin_id"], can["bin_type"]),
                         ("on_bin", 2, "recycling"))
        self.assertEqual((towel["category"], towel["place"], towel["obj_x"]),
                         ("trash", "around_bin", 1.3))
        self.assertEqual(can["model"], "yolov8s-worldv2.pt")


class TestZoom(unittest.TestCase):
    """The zoomed rim search (vlm.zoom_bin_top): its window and how its boxes merge."""

    def det(self, **zoom):
        d = VLMDetector.__new__(VLMDetector)          # no model, no Ollama
        d.zoom = {"enabled": True, "above": 1.0, "below": 0.3, "side": 0.25, **zoom}
        return d

    def test_window_covers_the_rim_and_above(self):
        bins = [{"bbox": [200, 150, 300, 300]}, {"bbox": [10, 200, 30, 230]}]
        r = self.det().rim_region(bins, 640, 480)
        self.assertEqual(r, [175.0, 0.0, 325.0, 195.0])   # biggest bin; clipped at the top

    def test_no_bin_or_huge_bin_means_no_zoom(self):
        self.assertIsNone(self.det().rim_region([], 640, 480))
        self.assertIsNone(self.det().rim_region([{"bbox": [0, 240, 640, 480]}], 640, 480))

    def test_zoom_only_in_good_light(self):
        from detect.detector import zoom_light_ok
        cfg = {"max_warm_cast": 9.0}
        self.assertTrue(zoom_light_ok(3.9, cfg))       # daylight median
        self.assertFalse(zoom_light_ok(16.6, cfg))     # lamp-light median
        self.assertTrue(zoom_light_ok(30.0, {}))       # no limit set: always

    def test_config_zoom_on_with_a_light_gate(self):
        z = CFG["vlm"]["zoom_bin_top"]
        self.assertTrue(z["enabled"])
        self.assertIn("max_warm_cast", z)

    def test_zoomed_boxes_win_and_new_ones_are_kept(self):
        zoomed = [([210, 100, 260, 160], "jug")]
        whole = [([205, 95, 262, 165], "white jug"),       # same object: dropped
                 ([220, 120, 240, 140], "cap"),            # inside the jug: dropped
                 ([400, 350, 480, 420], "towel")]          # elsewhere: kept
        self.assertEqual([n for _, n in merge_boxes(zoomed, whole)], ["jug", "towel"])


@unittest.skipUnless(importlib.util.find_spec("ultralytics")
                     and os.path.exists(os.path.join(MODELS_DIR, CFG["model"])),
                     "ultralytics or the weights not here (run in a container)")
class TestModel(unittest.TestCase):
    def test_robot_bin_photo(self):
        from detect.detector import Detector
        d = Detector()
        photo = os.path.join(APP, "results", "route", "20261118_195156", "photos",
                             "bin_4_step10.jpg")
        out = d.detect(photo)
        self.assertEqual(out["size"], [640, 480])
        self.assertLess(out["seconds"], 3.0)
        for x in out["detections"]:
            self.assertIn(x["kind"], ("bin", "item"))
            self.assertEqual(len(x["bbox"]), 4)


if __name__ == "__main__":
    unittest.main()
