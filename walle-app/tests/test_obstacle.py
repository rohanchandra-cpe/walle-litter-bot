"""Checks for walle-app/route/obstacle.py (Phase 5; no ROS needed).
    python3 -m unittest discover -s walle-app/tests"""

import math
import os
import sys
import unittest

APP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [APP, os.path.join(APP, "route")]
import obstacle as ob  # noqa: E402
import park  # noqa: E402
from detect.geometry import Camera  # noqa: E402

ROBOT = {"half_width_m": 0.086, "turn_radius_m": 0.14, "margin_m": 0.05,
         "body_front_m": 0.08, "body_back_m": 0.135}
CFG = {"trigger_m": 0.35, "confirm_pings": 3, "leg_end_slack_m": 0.0, "min_radius_m": 0.03,
       "max_box_radius_m": 0.15, "default_radius_m": 0.10, "max_radius_m": 0.20,
       "uncertainty_m": 0.03, "max_offset_m": 0.40, "assumed_depth_m": 0.25, "max_back_past_corner_m": 0.0,
       "leg_min_m": 0.01}
A, B = (0.0, 0.0), (1.0, 0.0)          # side 1, heading 0


def walk(legs, step=0.005):
    """Poses along the legs (straight moves) plus the turns in place between them."""
    poses = []
    for p, q, h in legs:
        n = max(1, int(math.hypot(q[0] - p[0], q[1] - p[1]) / step))
        for k in range(n + 1):
            poses.append((p[0] + (q[0] - p[0]) * k / n, p[1] + (q[1] - p[1]) * k / n, h))
    out = []
    for i, pose in enumerate(poses):        # turning in place sweeps every heading between
        if i and abs(ob.wrap(pose[2] - poses[i - 1][2])) > 1e-6:
            h0, dh = poses[i - 1][2], ob.wrap(pose[2] - poses[i - 1][2])
            out += [(pose[0], pose[1], h0 + dh * j / 36) for j in range(37)]
        out.append(pose)
    return out


class SideFrameTests(unittest.TestCase):
    def test_round_trip_on_side_2(self):
        a, b = (1.0, 0.0), (1.0, 1.0)
        pt = ob.from_side(0.4, 0.1, a, b)
        self.assertAlmostEqual(pt[0], 0.9)          # left of a +y heading is -x (inside)
        self.assertAlmostEqual(pt[1], 0.4)
        along, cross = ob.to_side(pt, a, b)
        self.assertAlmostEqual(along, 0.4)
        self.assertAlmostEqual(cross, 0.1)

    def test_echo(self):
        self.assertAlmostEqual(ob.echo_to_m(2332), 0.4, places=2)   # the robot, 40 cm
        self.assertIsNone(ob.echo_to_m(-1))
        self.assertIsNone(ob.echo_to_m(-2))

    def test_blocks_side(self):
        self.assertTrue(ob.blocks_side((0.5, 0.0), 0.04, A, B, ROBOT)[0])
        self.assertTrue(ob.blocks_side((0.5, -0.17), 0.04, A, B, ROBOT)[0])
        self.assertFalse(ob.blocks_side((0.5, -0.30), 0.04, A, B, ROBOT)[0])   # litter
        self.assertFalse(ob.blocks_side((1.4, 0.0), 0.04, A, B, ROBOT)[0])     # past b
        self.assertFalse(ob.blocks_side((0.2, 0.0), 0.04, A, B, ROBOT, start_along=0.4)[0])


class SonarTests(unittest.TestCase):
    def test_needs_three_in_a_row(self):
        t = ob.SonarTrigger(CFG, 0.125)
        us = int(0.30 * 2e6 / 343)
        self.assertIsNone(t.feed(us, 0.8))
        self.assertIsNone(t.feed(-1, 0.8))           # a timeout breaks the run
        self.assertIsNone(t.feed(us, 0.8))
        self.assertIsNone(t.feed(us, 0.8))
        self.assertAlmostEqual(t.feed(us, 0.8), 0.30, places=2)

    def test_ignores_far_and_beyond_the_leg(self):
        t = ob.SonarTrigger(CFG, 0.125)
        far = int(0.50 * 2e6 / 343)
        near = int(0.20 * 2e6 / 343)
        for _ in range(5):
            self.assertIsNone(t.feed(far, 2.0))      # beyond trigger_m
        for _ in range(5):
            self.assertIsNone(t.feed(near, 0.30))    # beyond the leg's end (0.30 - 0.125)
        for _ in range(2):
            t.feed(near, 0.60)
        self.assertIsNotNone(t.feed(near, 0.60))


class FootprintTests(unittest.TestCase):
    def test_box_of_a_known_can(self):
        """Project a can 0.6 m ahead with the robot's camera tilted down; the box gives back
        its center and radius."""
        cam = Camera(640, 480, 60, 47, 0.125, 0.125, tilt_deg=-15.0,
                     lens_from_pivot_m=(0.01, 0.04), left_m=0.011)
        r, cx = 0.05, 0.6
        pts = [(cx - r, -r, 0.0), (cx - r, r, 0.0), (cx - r, 0.0, 0.0)]
        us = [cam.project(p)[0] for p in pts]
        v = cam.project((cx - r, 0.0, 0.0))[1]
        bbox = [min(us), v - 60, max(us), v]
        fp = ob.footprint_from_box(bbox, cam, (0.0, 0.0, 0.0), CFG)
        self.assertFalse(fp["cut"])
        self.assertAlmostEqual(fp["width_m"], 2 * r, delta=0.01)     # as seen, unclamped
        self.assertAlmostEqual(fp["dist_m"], cx, delta=0.02)
        self.assertAlmostEqual(fp["radius_m"], r, delta=0.005)
        self.assertAlmostEqual(fp["center"][0], cx, delta=0.01)
        self.assertAlmostEqual(fp["center"][1], 0.0, delta=0.01)

    def test_a_speck_is_below_the_size_floor(self):
        """Robot run 20261122_211939: a 13 px box about a metre away (~2 cm) stopped the
        robot. Its seen width is under route.yaml min_size_m; the clamped radius is not."""
        import yaml
        cfg = yaml.safe_load(open(os.path.join(APP, "config", "route.yaml")))["obstacle"]
        cam = Camera(640, 480, 60, 47, 0.125, 0.125, tilt_deg=-15.0,
                     lens_from_pivot_m=(0.01, 0.04), left_m=0.011)
        fp = ob.footprint_from_box([293.1, 176.2, 306.6, 182.9], cam, (0.0, 0.0, 0.0), CFG)
        self.assertLess(fp["width_m"], cfg["min_size_m"])
        self.assertGreaterEqual(2 * fp["radius_m"], cfg["min_size_m"])
        self.assertGreater(fp["dist_m"], cfg["trust_range_m"])     # and it is far: re-look

    def test_cut_at_the_bottom(self):
        cam = Camera(640, 480, 60, 47, 0.125, 0.125, tilt_deg=-15.0)
        fp = ob.footprint_from_box([300, 400, 340, 480], cam, (0.0, 0.0, 0.0), CFG)
        self.assertTrue(fp["cut"])


class DetourTests(unittest.TestCase):
    def check_clear(self, plan, center, r):
        gap = ob.min_clearance(walk(ob.all_legs(plan)), center, r, ROBOT)
        self.assertGreater(gap, ROBOT["margin_m"] - 1e-6)
        return gap

    def test_mid_side(self):
        r, s = 0.04, 0.5
        plan = ob.plan_detour(A, B, s, 0.0, r, ROBOT, CFG, here_along=0.0)
        self.assertNotIn("error", plan)
        self.assertAlmostEqual(plan["offset_m"], r + 0.086 + 0.05 + 0.03)
        self.assertIsNotNone(plan["approach"])        # on the line first
        self.assertEqual(len(plan["legs"]), 3)       # out, past, back
        self.assertGreater(plan["legs"][0][1][1], 0)   # out to the LEFT (inside)
        self.check_clear(plan, (s, 0.0), r)
        self.assertFalse(plan["rejoins_corner"])

    def test_every_park_obstacle_fits(self):
        """Every spot park.py allows (0.25-0.75 along, any sim item and pose) has a detour
        that keeps the margin."""
        geo = park.Geometry(park.load_yaml(park.PARK_FILE))
        for item in geo.items:
            for pose in ("upright", "lying"):
                r = geo.footprint(item, pose)
                for s in (0.25, 0.4, 0.5, 0.6, 0.75):
                    if min(s, 1 - s) < geo.turn_radius + geo.margin + r:
                        continue                     # park.py refuses it
                    plan = ob.plan_detour(A, B, s, 0.0, r, ROBOT, CFG, here_along=0.0)
                    self.assertNotIn("error", plan, (item, pose, s))
                    self.check_clear(plan, (s, 0.0), r)

    def test_deep_object_along_the_path(self):
        """A bottle lying along the path: 6.4 cm wide in the photo (r 0.032), 20 cm deep.
        The turn back must clear its far end."""
        r, near = 0.032, 0.40
        plan = ob.plan_detour(A, B, near + r, 0.0, r, ROBOT, CFG, here_along=0.0)
        self.assertNotIn("error", plan)
        poses = walk(ob.all_legs(plan))
        gaps = [ob.min_clearance(poses, (near + t, c), 0.0, ROBOT)
                for t in (0.0, 0.05, 0.10, 0.15, 0.20) for c in (-r, 0.0, r)]
        self.assertGreater(min(gaps), ROBOT["margin_m"] - 1e-6)

    def test_near_the_far_corner_rejoins_there(self):
        plan = ob.plan_detour(A, B, 0.75, 0.0, 0.04, ROBOT, CFG, here_along=0.0)
        self.assertTrue(plan["rejoins_corner"])
        self.assertAlmostEqual(plan["legs"][-1][1][0], 1.0)
        self.assertAlmostEqual(plan["legs"][-1][1][1], 0.0)

    def test_too_close_backs_up(self):
        plan = ob.plan_detour(A, B, 0.6, 0.0, 0.04, ROBOT, CFG, here_along=0.42)
        first = plan["approach"]
        self.assertLess(first[1][0], first[0][0])     # the approach goes backward
        self.check_clear(plan, (0.6, 0.0), 0.04)

    def test_neighbours_get_one_detour(self):
        """Two objects near each other: one rectangle round both, clear of each."""
        objs = [{"id": 1, "along": 0.44, "cross": 0.15, "radius_m": 0.03},
                {"id": 2, "along": 0.62, "cross": 0.0, "radius_m": 0.04}]
        group, plan = ob.group_ahead(objs, ROBOT, CFG, A, B, 0.0)
        self.assertEqual([o["id"] for o in group], [1, 2])
        self.assertNotIn("error", plan)
        poses = walk(ob.all_legs(plan))
        for o in objs:
            self.assertGreater(ob.min_clearance(poses, ob.from_side(o["along"], o["cross"],
                                                                     A, B),
                                                o["radius_m"], ROBOT), ROBOT["margin_m"] - 1e-6)

    def test_run_204947_is_refused_before_moving(self):
        """Robot run 20261122_204947, side 3: a 'cup' at 0.44 and a wide 'towel' at 0.79
        (both floor tape). Planned as one group, there is no room before the corner, so the
        robot reports it before driving instead of after a first detour."""
        objs = [{"id": 3, "along": 0.4427, "cross": 0.1535, "radius_m": 0.03},
                {"id": 2, "along": 0.793, "cross": -0.0377, "radius_m": 0.15}]
        group, plan = ob.group_ahead(objs, ROBOT, CFG, A, B, 0.0)
        self.assertEqual(len(group), 2)
        self.assertIn("error", plan)

    def test_far_apart_stay_separate(self):
        long_side_b = (3.0, 0.0)
        objs = [{"id": 1, "along": 0.4, "cross": 0.0, "radius_m": 0.03},
                {"id": 2, "along": 2.0, "cross": 0.0, "radius_m": 0.03}]
        group, _ = ob.group_ahead(objs, ROBOT, CFG, A, long_side_b, 0.0)
        self.assertEqual([o["id"] for o in group], [1])

    def test_run_043408_detour_would_hit_side_2s_bag(self):
        """Robot run 20261123_043408: the (false) patch on side 3 at 0.41 m; its detour's
        turn 15 cm inside corner 3 swept into the bag found on side 2. The clearance check
        now catches it."""
        robot = dict(ROBOT, turn_radius_m=0.14)
        c3, c4 = (1.0, 1.0), (0.0, 1.0)
        plan = ob.plan_detour(c3, c4, 0.4103, 0.101, 0.0352, robot, CFG, 0.0)
        self.assertNotIn("error", plan)
        bag = ((1.0143, 0.6716), 0.1373)
        self.assertLess(ob.plan_clearance(plan, [bag], robot), 0.0)
        far = ((1.0143, 0.30), 0.1373)                  # the same bag 37 cm farther down
        self.assertGreater(ob.plan_clearance(plan, [far], robot), robot["margin_m"])
        self.assertEqual(ob.plan_clearance(plan, [], robot), math.inf)

    def test_too_big_or_too_wide(self):
        self.assertIn("error", ob.plan_detour(A, B, 0.5, 0.0, 0.25, ROBOT, CFG, 0.0))
        self.assertIn("error", ob.plan_detour(A, B, 0.5, 0.15, 0.15, ROBOT, CFG, 0.0))
        self.assertIn("error", ob.plan_detour(A, B, 0.1, 0.0, 0.05, ROBOT, CFG, 0.0))

    def test_collision_measure(self):
        self.assertLess(ob.min_clearance([(0.0, 0.0, 0.0)], (0.1, 0.0), 0.03, ROBOT), 0)
        self.assertAlmostEqual(ob.min_clearance([(0.0, 0.0, 0.0)], (0.0, 0.2), 0.03, ROBOT),
                               0.2 - 0.086 - 0.03)


if __name__ == "__main__":
    unittest.main()


class ImportTests(unittest.TestCase):
    """obstacles.json -> the memory store (memory/route_records.py record_obstacles)."""

    def run_dir(self, root, name, obstacles, sides):
        import json
        d = os.path.join(root, name)
        os.makedirs(d)
        with open(os.path.join(d, "obstacles.json"), "w") as f:
            json.dump({"obstacles": obstacles, "sides_done": sides}, f)
        return d

    def test_report_again_and_clear(self):
        import tempfile
        from memory.route_records import record_obstacles
        from memory.store import MemoryStore
        root = tempfile.mkdtemp()
        can = {"side": 2, "label": "can", "category": "recycling", "obj_xy": [1.0, 0.5],
               "action": "detour", "t_utc": "2026-11-22T10:00:00Z"}
        with MemoryStore("sim", path=os.path.join(root, "m.sqlite"), embedder=None) as s:
            for run, obs, sides, t in (("r1", [can], [1, 2, 3, 4], "10"),
                                       ("r2", [dict(can, obj_xy=[1.0, 0.56])], [1, 2], "11"),
                                       ("r3", [], [1], "12"),     # side 2 not driven
                                       ("r4", [], [1, 2, 3, 4], "13")):
                for o in obs:
                    o["t_utc"] = f"2026-11-22T{t}:00:00Z"
                s.start_run(run_id=run, started_utc=f"2026-11-22T{t}:00:00Z")
                record_obstacles(s, run, self.run_dir(root, run, obs, sides),
                                 t_utc=f"2026-11-22T{t}:30:00Z")
                now = s.current_obstacles()
                if run == "r1":                          # an unconfirmed far sighting
                    record_obstacles(s, run, self.run_dir(root, run + "x", [dict(
                        can, side=3, obj_xy=[0.2, 1.0], action="not_confirmed")], []))
                if run in ("r1", "r2", "r3"):
                    self.assertEqual(len(now), 1, run)   # r2: within 10 cm, the same one
                    self.assertEqual(now[0]["times_seen"], 1 if run == "r1" else 2)
                else:
                    self.assertEqual(now, [])            # r4 drove side 2: cleared


class TapeRejectTests(unittest.TestCase):
    """detect.yaml vlm.reject_names (robot run 20261122_204947: floor tape as trash)."""

    def test_names(self):
        from types import SimpleNamespace
        try:
            from detect.detector import VLMDetector, load_config
        except ImportError as e:              # no ultralytics on this host
            self.skipTest(str(e))
        words = {w.lower() for w in load_config()["vlm"]["reject_names"]}
        me = SimpleNamespace(reject_words=words)
        for name in ("tape", "Masking Tape", "white tape strip", "floor markings"):
            self.assertTrue(VLMDetector.rejects(me, name), name)
        for name in ("paper towel", "plastic milk jug", "metal drink or coffee can",
                     "crumpled paper"):
            self.assertFalse(VLMDetector.rejects(me, name), name)
