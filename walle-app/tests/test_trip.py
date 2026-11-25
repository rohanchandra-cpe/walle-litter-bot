"""Checks for Phase 6's partial route (route_geometry.build_trip, --trip-bin), the look-only
plan (build_look, --look-only) and the mirrored detours of route/obstacle.py (the `inside`
sign for clockwise legs). No ROS needed.
    python3 -m unittest discover -s walle-app/tests"""

import math
import os
import sys
import unittest

APP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [APP, os.path.join(APP, "route"), os.path.dirname(os.path.abspath(__file__))]
import obstacle as ob  # noqa: E402
import park  # noqa: E402
import route_geometry as rg  # noqa: E402
from test_obstacle import CFG, ROBOT, walk  # noqa: E402

LAYOUT = park.load_park("default")


def deg(a):
    return round(math.degrees(a))


def legs(steps):
    return [(s[1], s[2], s[6]) for s in steps if s[0] == "drive"]


class TripPlanTests(unittest.TestCase):
    def test_shorter_way(self):
        self.assertEqual(rg.trip_path(1), [1])
        self.assertEqual(rg.trip_path(2), [1, 2])
        self.assertEqual(rg.trip_path(3), [1, 2, 3])            # a tie: counter-clockwise
        self.assertEqual(rg.trip_path(4), [1, 4])                # clockwise, side 4

    def test_side_numbers_stay_counter_clockwise(self):
        self.assertEqual(rg.side_of(1, 2), (1, 1))
        self.assertEqual(rg.side_of(2, 1), (1, -1))
        self.assertEqual(rg.side_of(1, 4), (4, -1))
        self.assertEqual(rg.side_of(4, 1), (4, 1))
        with self.assertRaises(ValueError):
            rg.side_of(1, 3)

    def test_bin_1_does_not_drive(self):
        steps = rg.build_trip(LAYOUT, 1, dwell_s=5, look_sides=True)
        self.assertEqual([s[0] for s in steps], ["face_bin", "dwell", "turn"])
        self.assertEqual(steps[0][2], "bin_1")
        self.assertEqual(deg(steps[-1][2]), 0)                   # back to the start heading

    def test_bin_2_side_1_out_and_back(self):
        steps = rg.build_trip(LAYOUT, 2, dwell_s=5, look_sides=True)
        self.assertEqual(legs(steps), [(1, 2, 1), (2, 1, -1)])
        kinds = [s[0] for s in steps]
        self.assertEqual(kinds, ["look_side", "drive", "face_bin", "dwell", "turn",
                                 "look_side", "drive", "turn"])
        self.assertEqual(deg(steps[4][2]), 180)                  # the turn back
        self.assertEqual(deg(steps[-1][2]), 0)
        self.assertEqual(steps[2][2], "bin_2")
        self.assertEqual(deg(steps[2][3]), -45)

    def test_bin_3_two_sides(self):
        steps = rg.build_trip(LAYOUT, 3, dwell_s=5, look_sides=True)
        self.assertEqual(legs(steps), [(1, 2, 1), (2, 3, 1), (3, 2, -1), (2, 1, -1)])
        self.assertEqual([deg(s[5]) for s in steps if s[0] == "drive"], [0, 90, -90, 180])
        self.assertEqual([deg(s[2]) for s in steps if s[0] == "turn"], [90, -90, 180, 0])

    def test_bin_4_clockwise_on_side_4(self):
        steps = rg.build_trip(LAYOUT, 4, dwell_s=5, look_sides=True)
        self.assertEqual(legs(steps), [(1, 4, -1), (4, 1, 1)])
        self.assertEqual(steps[0][0], "turn")                    # from 0 deg to 90 first
        self.assertEqual(deg(steps[0][2]), 90)
        self.assertEqual([deg(s[5]) for s in steps if s[0] == "drive"], [90, -90])
        self.assertEqual(deg(steps[-1][2]), 0)

    def test_each_look_precedes_its_drive(self):
        for n in (2, 3, 4):
            steps = rg.build_trip(LAYOUT, n, look_sides=True)
            for i, s in enumerate(steps):
                if s[0] == "drive":
                    self.assertEqual(steps[i - 1][0], "look_side")
                    self.assertEqual(steps[i - 1][1:], s[1:])
            self.assertFalse([s for s in rg.build_trip(LAYOUT, n) if s[0] == "look_side"])

    def test_ends_where_it_started(self):
        for n in (2, 3, 4):
            drives = [s for s in rg.build_trip(LAYOUT, n) if s[0] == "drive"]
            self.assertEqual(drives[0][3], drives[-1][4])
            self.assertEqual(drives[0][3], (0.0, 0.0))

    def test_laps_unchanged(self):
        """build_plan's steps keep their 6 fields (counter-clockwise, no direction)."""
        for s in rg.build_plan(LAYOUT, look_sides=True):
            if s[0] in ("drive", "look_side"):
                self.assertEqual(len(s), 6)

    def test_look_only_is_side_1(self):
        steps = rg.build_look(LAYOUT)
        self.assertEqual(steps, [("look_side", 1, 2, (0.0, 0.0), (1.0, 0.0), 0.0, 1)])


class MirroredDetourTests(unittest.TestCase):
    def mirror(self, pt):
        return (pt[0], -pt[1])

    def test_clockwise_is_the_mirror_image(self):
        """inside=-1 on a -> b is inside=+1 mirrored across the line, for every case the
        Phase 5 tests use."""
        A, B = (0.0, 0.0), (1.0, 0.0)
        for along, cross, r, here in ((0.5, 0.0, 0.04, 0.0), (0.4, 0.03, 0.032, 0.0),
                                      (0.75, -0.02, 0.04, 0.0), (0.6, 0.0, 0.04, 0.42)):
            ccw = ob.plan_detour(A, B, along, cross, r, ROBOT, CFG, here)
            cw = ob.plan_detour(A, B, along, -cross, r, ROBOT, CFG, here, inside=-1)
            self.assertNotIn("error", cw)
            for k in ("start_along", "end_along", "offset_m", "depth_m", "rejoins_corner"):
                self.assertEqual(ccw[k], cw[k], k)
            self.assertEqual((ccw["inside"], cw["inside"]), (1, -1))
            for (p, q, h), (p2, q2, h2) in zip(ob.all_legs(ccw), ob.all_legs(cw)):
                for u, v in ((p, p2), (q, q2)):
                    self.assertAlmostEqual(u[0], v[0])
                    self.assertAlmostEqual(-u[1], v[1])
                self.assertAlmostEqual(ob.wrap(-h - h2), 0.0)

    def test_default_is_counter_clockwise(self):
        A, B = (0.0, 0.0), (1.0, 0.0)
        self.assertEqual(ob.plan_detour(A, B, 0.5, 0.01, 0.04, ROBOT, CFG, 0.0),
                         ob.plan_detour(A, B, 0.5, 0.01, 0.04, ROBOT, CFG, 0.0, inside=1))

    def test_side_4_clockwise_stays_inside_and_clear(self):
        """Bin 4's trip drives side 4 from corner 1 to corner 4 (heading +90): inside the
        square is +x, the robot's right. The detour must go there and keep the margin."""
        c1, c4 = (0.0, 0.0), (0.0, 1.0)
        for s in (0.25, 0.5, 0.75):
            r = 0.04
            center = ob.from_side(s, 0.0, c1, c4)
            plan = ob.plan_detour(c1, c4, s, 0.0, r, ROBOT, CFG, 0.0, inside=-1)
            self.assertNotIn("error", plan, s)
            out = plan["legs"][0]
            self.assertGreater(out[1][0], 0.1)                   # x > 0: inside the square
            self.assertAlmostEqual(out[2], 0.0)                  # turned right, to +x
            self.assertAlmostEqual(plan["legs"][2][2], math.pi)  # back to the line: -x
            for p, q, _ in ob.all_legs(plan):
                for pt in (p, q):
                    self.assertTrue(-1e-9 <= pt[0] <= 1.0 and -1e-9 <= pt[1] <= 1.0, pt)
            gap = ob.min_clearance(walk(ob.all_legs(plan)), center, r, ROBOT)
            self.assertGreater(gap, ROBOT["margin_m"] - 1e-6)
            # the counter-clockwise plan on this leg would leave the square (x < 0)
            wrong = ob.plan_detour(c1, c4, s, 0.0, r, ROBOT, CFG, 0.0)
            self.assertLess(wrong["legs"][0][1][0], 0.0)

    def test_clockwise_offset_counts_the_inside_edge(self):
        """An object off the line toward the inside (right, on a clockwise leg) widens the
        detour; one toward the outside does not."""
        A, B = (0.0, 0.0), (1.0, 0.0)
        base = ob.plan_detour(A, B, 0.5, 0.0, 0.04, ROBOT, CFG, 0.0, inside=-1)["offset_m"]
        right = ob.plan_detour(A, B, 0.5, -0.05, 0.04, ROBOT, CFG, 0.0, inside=-1)["offset_m"]
        left = ob.plan_detour(A, B, 0.5, 0.05, 0.04, ROBOT, CFG, 0.0, inside=-1)["offset_m"]
        self.assertAlmostEqual(right - base, 0.05)
        self.assertAlmostEqual(left, base)

    def test_group_ahead_passes_the_sign(self):
        A, B = (0.0, 0.0), (1.0, 0.0)
        objs = [{"id": 1, "along": 0.44, "cross": -0.15, "radius_m": 0.03},
                {"id": 2, "along": 0.62, "cross": 0.0, "radius_m": 0.04}]
        group, plan = ob.group_ahead(objs, ROBOT, CFG, A, B, 0.0, inside=-1)
        self.assertEqual([o["id"] for o in group], [1, 2])
        self.assertNotIn("error", plan)
        self.assertLess(plan["legs"][0][1][1], 0)                # out to the right
        poses = walk(ob.all_legs(plan))
        for o in objs:
            self.assertGreater(ob.min_clearance(
                poses, ob.from_side(o["along"], o["cross"], A, B), o["radius_m"], ROBOT),
                ROBOT["margin_m"] - 1e-6)


class RecordKindTests(unittest.TestCase):
    def test_kind_reaches_the_store(self):
        import json
        import tempfile
        from memory.route_records import record_route_summary
        from memory.store import MemoryStore
        with tempfile.TemporaryDirectory() as d:
            for kind, run_id in (("trip", "t1"), ("look", "l1"), (None, "r1")):
                run_dir = os.path.join(d, run_id)
                os.makedirs(run_dir)
                summary = {"run_id": run_id, "source": "sim", "status": "done",
                           "scenario": "default", "started_utc": "2026-11-23T00:00:00+00:00",
                           "ended_utc": "2026-11-23T00:01:00+00:00", "events": []}
                if kind:
                    summary["kind"] = kind
                with open(os.path.join(run_dir, "obstacles.json"), "w") as f:
                    json.dump({"sides_done": [], "obstacles": []}, f)
                with MemoryStore("sim", path=os.path.join(d, "m.sqlite")) as mem:
                    record_route_summary(mem, summary, LAYOUT, run_dir)
            with MemoryStore("sim", path=os.path.join(d, "m.sqlite")) as mem:
                kinds = {r["run_id"]: r["kind"] for r in mem.runs(limit=10)}
        self.assertEqual(kinds, {"t1": "trip", "l1": "look", "r1": "route"})


if __name__ == "__main__":
    unittest.main()
