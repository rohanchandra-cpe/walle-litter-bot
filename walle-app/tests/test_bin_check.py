"""Checks for the Phase 4 bin check: the tilt servo's calibration, the tilted camera geometry,
the ground-truth stand-in with tilt, the judge, and the memory import.
    python3 -m unittest discover -s walle-app/tests
No ROS, no model.
"""

import json
import math
import os
import shutil
import sys
import tempfile
import unittest

APP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [APP, os.path.join(APP, "route")]
import park  # noqa: E402
from detect import bin_check as bc  # noqa: E402
from detect import geometry as g  # noqa: E402
from detect.detector import load_config  # noqa: E402
from servo.tilt import Calibration  # noqa: E402

CFG = load_config()


class TestCalibration(unittest.TestCase):
    def test_round_trip_and_clamp(self):
        cal = Calibration(level_us=1480, us_per_deg=-11.0, min_deg=-10, max_deg=30)
        for deg in (-10, 0, 7.5, 30):
            self.assertAlmostEqual(cal.us_to_deg(cal.deg_to_us(deg)), deg, delta=0.05)
        self.assertEqual(cal.deg_to_us(90), cal.deg_to_us(30))       # clamped to max_deg
        self.assertEqual(cal.deg_to_us(0), 1480)
        self.assertTrue(cal.is_level(1482) and not cal.is_level(1490))

    def test_firmware_range(self):
        cal = Calibration(level_us=2300, us_per_deg=20.0, min_deg=-10, max_deg=30)
        self.assertEqual(cal.deg_to_us(30), 2400)                    # servo.h clamp

    def test_config_loads(self):
        robot, sim = Calibration.load(), Calibration.load(sim=True)
        self.assertGreater(robot.up_deg, 0)
        self.assertTrue(robot.min_deg <= robot.up_deg <= robot.max_deg)
        self.assertEqual(sim.up_deg, robot.up_deg)

    def test_bad_config(self):
        with self.assertRaises(ValueError):
            Calibration(us_per_deg=0)
        with self.assertRaises(ValueError):
            Calibration(min_deg=5, max_deg=30)


class TestTiltedCamera(unittest.TestCase):
    def test_level_is_unchanged(self):
        """tilt 0 gives the level camera's numbers (the Phase 3 formulas)."""
        cam = g.Camera(tilt_deg=0.0)
        fx, fy = cam.fx, cam.fy
        for u, v in ((100, 300), (320, 400), (600, 250)):
            down = (v - 240) / fy
            t = 0.103 / down
            want = (0.12 + t, t * (320 - u) / fx)
            got = cam.ground_point(u, v)
            self.assertAlmostEqual(got[0], want[0], places=9)
            self.assertAlmostEqual(got[1], want[1], places=9)
            self.assertAlmostEqual(cam.bearing_deg(u), math.degrees(math.atan((320 - u) / fx)),
                                   places=9)
        self.assertIsNone(cam.ground_point(320, 240))

    def test_project_round_trip(self):
        for tilt in (-8.0, 0.0, 15.0, 25.0):
            cam = g.Camera(tilt_deg=tilt, lens_from_pivot_m=(0.02, 0.015))
            for pt in ((0.9, 0.1, 0.0), (0.7, -0.2, 0.0), (1.5, 0.3, 0.0)):
                uv = cam.project(pt)
                if not (0 <= uv[1] <= cam.h):
                    continue
                back = cam.ground_point(*uv)
                self.assertAlmostEqual(back[0], pt[0], places=6, msg=f"tilt {tilt}")
                self.assertAlmostEqual(back[1], pt[1], places=6)
            uv = cam.project((0.9, 0.0, 0.40))
            self.assertAlmostEqual(cam.height_at(uv[1], 0.9 - cam.forward_m), 0.40, places=6)

    def test_tilt_moves_things_down_the_frame(self):
        level, up = g.Camera(), g.Camera(tilt_deg=15.0)
        rim = (0.12 + 0.65, 0.0, 0.292)
        self.assertGreater(up.project(rim)[1], level.project(rim)[1])
        self.assertAlmostEqual(up.elevation_deg(240), 15.0)

    def test_lens_off_the_axis_moves(self):
        cam = g.Camera(height_m=0.10, forward_m=0.10, tilt_deg=90.0,
                       lens_from_pivot_m=(0.02, 0.0))
        self.assertAlmostEqual(cam.forward_m, 0.08, places=9)   # swung up over the axis
        self.assertAlmostEqual(cam.height_m, 0.12, places=9)


class TestCutAtBottom(unittest.TestCase):
    def test_no_floor_position_when_cut_at_bottom(self):
        cam = g.Camera(tilt_deg=15.0)
        dets = [{"kind": "item", "label": "can", "category": "recycling", "conf": 0.9,
                 "bbox": [300, 400, 340, 480]},
                {"kind": "item", "label": "can", "category": "recycling", "conf": 0.9,
                 "bbox": [100, 400, 140, 450]}]
        an = g.analyze_bin_stop(dets, cam, pose=(0.0, 0.0, 0.0), cfg=CFG)
        self.assertIsNone(an["items"][0]["obj_xy"])
        self.assertIsNotNone(an["items"][1]["obj_xy"])


class TestStopRules(unittest.TestCase):
    """The bin-stop rules from robot run 20261122_134315 (geometry.analyze_bin_stop stop=)."""
    CAM_UP = g.Camera(height_m=0.125, forward_m=0.125, tilt_deg=15.0)
    CAM = g.Camera(height_m=0.125, forward_m=0.125)
    STOP = {"bin_dist_m": 0.757, "bin_height_m": 0.292}

    def det(self, kind, bbox, label="bin", conf=0.5):
        return {"kind": kind, "label": label, "category": "recycling", "conf": conf,
                "bbox": bbox}

    def test_faint_bin_in_up_look(self):
        """Bin 3's up photo: the bin only as a 0.025 low_conf box; items on top."""
        items = [self.det("item", [195, 119, 434, 250], "foil")]
        faint = [self.det("bin", [190.0, 246.2, 438.1, 479.9], conf=0.025)]
        plain = g.analyze_bin_stop(items, self.CAM_UP, cfg=CFG, stop=dict(self.STOP))
        self.assertIsNone(plain["bin"])
        an = g.analyze_bin_stop(items, self.CAM_UP, cfg=CFG,
                                stop=dict(self.STOP, low_conf_bins=faint))
        self.assertTrue(an["bin"]["faint"])
        self.assertEqual(an["items"][0]["place"], "on_bin")
        far = [self.det("bin", [0, 246, 150, 479], conf=0.025)]           # way off to the side
        self.assertIsNone(g.analyze_bin_stop(items, self.CAM_UP, cfg=CFG, stop=dict(
            self.STOP, low_conf_bins=far))["bin"])

    def test_item_must_rest_on_the_rim(self):
        """Bin 1's up photo: a speck far above the rim is not on the bin."""
        dets = [self.det("bin", [178, 227, 434, 479.8], conf=0.4),
                self.det("item", [366.7, 44.5, 384.0, 63.3], "paper_bag"),
                self.det("item", [250, 150, 330, 228], "can")]
        an = g.analyze_bin_stop(dets, self.CAM_UP, cfg=CFG, stop=dict(self.STOP))
        self.assertEqual([d["label"] for d in an["items"]], ["can"])
        self.assertEqual(an["items"][0]["place"], "on_bin")
        self.assertEqual([d["label"] for d in an["ignored"]], ["paper_bag"])
        # without stop (Phase 3 eval, centering): unchanged
        self.assertEqual(len(g.analyze_bin_stop(dets, self.CAM_UP, cfg=CFG)["items"]), 2)

    def test_bottom_cut_litter_in_level_look_only(self):
        bottle = [self.det("item", [293, 176, 377, 478.1], "plastic_bottle")]
        an = g.analyze_bin_stop(bottle, self.CAM, pose=(0.0, 0.0, 0.0), cfg=CFG,
                                stop=dict(self.STOP))
        it = an["items"][0]
        self.assertEqual(it["obj_xy_from"], "frame_bottom")
        self.assertLess(it["obj_xy"][0], 0.125 + 0.757)                  # in front of the bin
        up = g.analyze_bin_stop(bottle, self.CAM_UP, pose=(0.0, 0.0, 0.0), cfg=CFG,
                                stop=dict(self.STOP))
        self.assertIsNone(up["items"][0]["obj_xy"])                      # beyond the bin: no

    def test_item_draped_over_the_rim_is_on_the_bin(self):
        """Run 20261122_145845, bin 1's up photo: a paper towel on the rim whose box ends
        18 px below the bin box's top; a bottle standing in front of the bin stays on the
        floor (its bottom is far below the rim)."""
        dets = [self.det("bin", [210.8, 213.8, 473.8, 480.0], conf=0.53),
                self.det("item", [293, 102, 515, 232], "paper_towel"),
                self.det("item", [300, 180, 360, 470], "plastic_bottle")]
        an = g.analyze_bin_stop(dets, self.CAM_UP, cfg=CFG, stop=dict(self.STOP))
        self.assertEqual([d["place"] for d in an["items"]], ["on_bin", "around_bin"])
        plain = g.analyze_bin_stop(dets, self.CAM_UP, cfg=CFG)       # Phase 3 rule: 8 px
        self.assertEqual(plain["items"][0]["place"], "around_bin")

    def test_bin_box_must_be_near_straight_ahead(self):
        """Bin 4's level photo: a Ziploc box at -14 deg was taken for the bin."""
        dets = [self.det("bin", [392.6, 340.7, 578.4, 472.9], conf=0.3)]
        self.assertIsNotNone(g.analyze_bin_stop(dets, self.CAM, cfg=CFG)["bin"])
        self.assertIsNone(g.analyze_bin_stop(dets, self.CAM, cfg=CFG,
                                             stop=dict(self.STOP))["bin"])


class TestLensOffset(unittest.TestCase):
    def test_left_offset_round_trip(self):
        cam = g.Camera(height_m=0.125, forward_m=0.125, left_m=0.011, tilt_deg=15.0)
        for pt in ((0.8, 0.0, 0.0), (0.7, -0.15, 0.0)):
            back = cam.ground_point(*cam.project(pt))
            self.assertAlmostEqual(back[0], pt[0], places=6)
            self.assertAlmostEqual(back[1], pt[1], places=6)
        # a point straight ahead of the LENS is in the middle column
        self.assertAlmostEqual(cam.project((0.9, 0.011, 0.0))[0], cam.w / 2, places=6)

    def test_park_camera(self):
        cam = park.load_yaml(park.PARK_FILE)["robot"]["camera"]
        self.assertEqual((cam["height_m"], cam["forward_m"], cam["left_m"]),
                         (0.125, 0.125, 0.011))


class TestLitterInView(unittest.TestCase):
    """park.py puts floor items on the corner side of each bin, within LITTER_VIEW_DEG of the
    camera's axis at the bin stop, so the robot can see them (2026-11-21)."""

    def test_every_floor_item_in_view(self):
        import yaml
        cam = park.load_yaml(park.PARK_FILE)["robot"]["camera"]
        for seed in range(1, 26):
            path = os.path.join(tempfile.mkdtemp(), "s.yaml")
            with open(path, "w") as f:
                yaml.safe_dump({"bins": {c: {"state": "overflowing", "level": 3, "litter": 6}
                                         for c in (1, 2, 3, 4)},
                                "obstacle": "none", "seed": seed}, f)
            layout = park.load_park(path, obstacle="none")
            for b in layout["bins"]:
                cx, cy = layout["route"]["corners"][b["corner"]]
                h = math.radians(b["view_heading_deg"])
                lx, ly = cx + cam["forward_m"] * math.cos(h), cy + cam["forward_m"] * math.sin(h)
                for it in b["spill"] + b["litter"]:
                    x, y = it["position"][:2]
                    brg = math.degrees(math.atan2(y - ly, x - lx) - h)
                    brg = (brg + 180) % 360 - 180
                    self.assertLessEqual(abs(brg), park.LITTER_VIEW_DEG + 1e-6)
                # and none is hidden: the ground-truth camera (robot geometry) boxes every one
                from detect.sim_truth import TruthDetector
                pose = (cx, cy, h)
                det = TruthDetector(layout, lambda: pose, cfg=CFG)
                det.cam = dict(cam)
                img = type("Img", (), {"size": (640, 480)})()
                seen = {d["truth"] for d in det.detect(img)["detections"]
                        if d.get("truth_group") in ("spill", "litter")}
                want = {it["name"] for it in b["spill"] + b["litter"]}
                self.assertEqual(seen, want, f"seed {seed} {b['name']}")


class TestSimTruthTilt(unittest.TestCase):
    """sim_truth's own projection with tilt agrees with geometry.Camera."""

    def test_agrees_with_geometry(self):
        from detect.sim_truth import TruthDetector, camera_settings
        layout = park.load_park("mixed", obstacle="none")
        for tilt in (0.0, 15.0):
            det = TruthDetector(layout, lambda: (0.0, 0.0, 0.0), cfg=CFG,
                                lens_from_pivot_m=(0.01, 0.02))
            cam = g.Camera.from_park(camera_settings(CFG), 320, 240, tilt_deg=tilt,
                                     lens_from_pivot_m=(0.01, 0.02))
            pts = [(0.8, 0.05, 0.0), (0.8, -0.05, 0.3)]
            box, _ = det.project(pts, (0.0, 0.0, 0.0), 320, 240, tilt)
            uvs = [cam.project(p) for p in pts]
            self.assertAlmostEqual(box[0], min(u for u, _ in uvs), delta=0.1)
            self.assertAlmostEqual(box[1], min(v for _, v in uvs), delta=0.1)
            self.assertAlmostEqual(box[3], max(v for _, v in uvs), delta=0.1)


class TestJudgeFromSim(unittest.TestCase):
    """The whole bin check on the ground-truth stand-in: every bin of `mixed` and `bincheck`,
    level look + up look, judged like the follower does."""

    def judge(self, layout, b, up=True):
        from detect.sim_truth import TruthDetector, camera_settings
        corner = layout["route"]["corners"][b["corner"]]
        pose = (corner[0], corner[1], math.radians(b["view_heading_deg"]))
        tilt = {"deg": 0.0}
        det = TruthDetector(layout, lambda: pose, cfg=CFG, tilt_fn=lambda: tilt["deg"])
        img = type("Img", (), {"size": (320, 240)})()
        settings = camera_settings(CFG)
        pose_deg = (pose[0], pose[1], b["view_heading_deg"])
        dist = math.hypot(b["position"][0] - corner[0], b["position"][1] - corner[1]) \
            - settings["forward_m"]
        lcam = g.Camera.from_park(settings, 320, 240)
        level = g.analyze_bin_stop(det.detect(img)["detections"], lcam, pose=pose_deg, cfg=CFG)
        up_an, ucam = None, None
        if up:
            tilt["deg"] = 15.0
            ucam = g.Camera.from_park(settings, 320, 240, tilt_deg=15.0)
            up_an = g.analyze_bin_stop(det.detect(img)["detections"], ucam, pose=pose_deg,
                                       cfg=CFG)
        return bc.judge_bin(level, up_an, b, CFG, level_cam=lcam, up_cam=ucam, dist_m=dist), level

    def test_every_state(self):
        for scen in ("mixed", "bincheck"):
            layout = park.load_park(scen, obstacle="none")
            for b in layout["bins"]:
                rec, level = self.judge(layout, b)
                t = bc.expected_from_layout(b)
                self.assertEqual(rec["method"], "tilt")
                self.assertEqual(rec["overflow_level"], t["overflow_level"],
                                 f"{scen} {b['name']}: {rec}")
                # all floor items are in view since 2026-11-21: the whole count
                self.assertEqual(rec["litter_count"], t["litter_count"], f"{scen} {b['name']}")
                self.assertEqual(rec["outcome"], t["outcome"], f"{scen} {b['name']}")

    def test_level_only_fallback(self):
        layout = park.load_park("mixed", obstacle="none")
        b = next(b for b in layout["bins"] if b["id"] == 1)          # ok, no litter
        rec, _ = self.judge(layout, b, up=False)
        self.assertEqual((rec["method"], rec["outcome"]), ("level_only", "clean"))


class TestJudgeRules(unittest.TestCase):
    B = {"name": "bin_1", "id": 1, "shape": "box", "position": [1.0, 0.0],
         "size_m": [0.277, 0.211, 0.292], "height_m": 0.292, "yaw_deg": 0.0}

    def item(self, place, bbox, xy=None, conf=0.8):
        return {"kind": "item", "label": "can", "category": "recycling", "conf": conf,
                "bbox": bbox, "place": place, "obj_xy": xy}

    def test_levels_and_litter_radius(self):
        cam = g.Camera(tilt_deg=15.0)
        on = [self.item("on_bin", [300, 100 + 5 * k, 340, 140]) for k in range(5)]
        level_an = {"items": [self.item("around_bin", [0, 400, 30, 430], xy=[0.80, 0.30]),
                              self.item("around_bin", [0, 400, 30, 430], xy=[2.5, 0.0]),
                              self.item("around_bin", [0, 400, 30, 430], xy=None)],
                    "bin": None}
        rec = bc.judge_bin(level_an, {"items": on, "bin": None}, self.B, CFG,
                           level_cam=g.Camera(), up_cam=cam, dist_m=0.75)
        self.assertEqual(rec["on_top_count"], 5)
        self.assertGreaterEqual(rec["overflow_level"], 2)            # 5 items: count level 2
        self.assertEqual(rec["litter_count"], 1)                     # far one + no-xy one out
        self.assertEqual(len(rec["not_litter"]), 2)
        self.assertEqual(rec["outcome"], "overflowing")

    def test_level_from(self):
        self.assertEqual(bc.level_from(0.105, [None, 0.095, 0.13]), 2)
        self.assertEqual(bc.level_from(0.2, [None, 0.095, 0.13]), 3)
        self.assertEqual(bc.level_from(0.05, [None, 0.095, 0.13]), 0)
        self.assertEqual(bc.level_from(5, [1, 4, 7]), 2)
        self.assertEqual(bc.level_from(None, [1, 4, 7]), 0)

    def test_clean_and_litter(self):
        empty = {"items": [], "bin": None}
        rec = bc.judge_bin(empty, empty, self.B, CFG, g.Camera(), g.Camera(tilt_deg=15),
                           dist_m=0.75)
        self.assertEqual((rec["overflow_level"], rec["outcome"]), (0, "clean"))
        lit = {"items": [self.item("around_bin", [0, 400, 30, 430], xy=[0.85, -0.2])],
               "bin": None}
        rec = bc.judge_bin(lit, empty, self.B, CFG, g.Camera(), g.Camera(tilt_deg=15),
                           dist_m=0.75)
        self.assertEqual(rec["outcome"], "litter")

    def test_footprint_distance(self):
        self.assertEqual(bc.dist_to_footprint([1.0, 0.0], self.B), 0.0)
        self.assertAlmostEqual(bc.dist_to_footprint([1.0, 0.3385], self.B), 0.2, places=6)


class TestMemoryImport(unittest.TestCase):
    def test_bin_checks_and_look_dedupe(self):
        from memory.route_records import record_route_summary
        from memory.store import MemoryStore
        try:
            from test_memory import FakeEmbedder
        except ImportError:
            from tests.test_memory import FakeEmbedder
        tmp = tempfile.mkdtemp()
        try:
            layout = park.load_park("default")
            run_dir = os.path.join(tmp, "run")
            os.makedirs(run_dir)
            summary = {"run_id": "20261121_120000", "source": "sim", "scenario": "default",
                       "laps": 1, "status": "done", "started_utc": "2026-11-21T12:00:00+00:00",
                       "ended_utc": "2026-11-21T12:05:00+00:00", "distance_m": 4.0,
                       "end_errors": {}, "events": [
                           {"event": "facing_bin", "step": 1, "bin": "bin_2",
                            "photo": "photos/bin_2_step2.jpg", "poses": {"odom": [1, 0, -45]}},
                           {"event": "facing_bin", "step": 4, "bin": "bin_3",
                            "poses": {"odom": [1, 1, 45]}}]}
            with open(os.path.join(run_dir, "bin_checks.json"), "w") as f:
                json.dump({"checks": [{"facing_step": 1, "bin": "bin_2", "method": "tilt",
                                       "overflow_level": 2, "on_top_count": 5,
                                       "litter_count": 1, "outcome": "overflowing"}]}, f)
            it = lambda place: {"label": "can", "category": "recycling", "place": place,  # noqa
                                "conf": 0.9, "bbox": [0, 0, 1, 1]}
            with open(os.path.join(run_dir, "detections.json"), "w") as f:
                json.dump({"photos": [
                    {"step": 2, "bin": "bin_2", "bin_id": 2, "look": "level",
                     "items": [it("on_bin"), it("around_bin")]},
                    {"step": 2, "bin": "bin_2", "bin_id": 2, "look": "up",
                     "items": [it("on_bin"), it("on_bin"), it("around_bin")]}]}, f)
            cfg = __import__("memory.store", fromlist=["load_config"]).load_config()
            with MemoryStore("sim", path=os.path.join(tmp, "s.sqlite"),
                             embedder=FakeEmbedder(), cfg=cfg) as s:
                record_route_summary(s, summary, layout, run_dir)
                c2, c3 = s.last_bin_check(2), s.last_bin_check(3)
                self.assertEqual((c2["method"], c2["overflow_level"], c2["outcome"]),
                                 ("tilt", 2, "overflowing"))
                self.assertEqual(c3["method"], "photo_only")
                n = s.db.execute("SELECT place, COUNT(*) FROM detections GROUP BY place") \
                    .fetchall()
                # on_bin from the up look (2), around_bin from the level look (1)
                self.assertEqual({r[0]: r[1] for r in n}, {"on_bin": 2, "around_bin": 1})
        finally:
            shutil.rmtree(tmp)


if __name__ == "__main__":
    unittest.main()
