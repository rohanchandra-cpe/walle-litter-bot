"""Checks for walle-app/route/route_geometry.py (no ROS needed).
    python3 -m unittest discover -s walle-app/tests"""

import math
import os
import sys
import unittest

APP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path[:0] = [APP, os.path.join(APP, "route")]
import park  # noqa: E402
import route_geometry as rg  # noqa: E402


class GeometryTests(unittest.TestCase):
    def test_side_progress(self):
        a, b = (1.0, 0.0), (1.0, 1.0)              # side 2: heading +90 deg
        along, remaining, cross = rg.side_progress((0.98, 0.4, 0.0), a, b)
        self.assertAlmostEqual(along, 0.4)
        self.assertAlmostEqual(remaining, 0.6)
        self.assertAlmostEqual(cross, 0.02)        # x < 1 is LEFT of a +y heading

    def test_line_heading_steers_back_and_is_clamped(self):
        h = math.pi / 2
        self.assertLess(rg.line_heading(h, 0.02, 4.0, math.radians(20)), h)   # left -> right
        self.assertGreater(rg.line_heading(h, -0.02, 4.0, math.radians(20)), h)
        self.assertAlmostEqual(rg.line_heading(h, 5.0, 4.0, math.radians(20)),
                               h - math.radians(20))
        self.assertAlmostEqual(rg.line_heading(h, 0.0, 4.0, 1.0), h)

    def test_anchor_round_trip_and_start_maps_to_start(self):
        est_start, park_start = (0.3, -0.2, math.radians(40)), (0.0, 0.0, 0.0)
        an = rg.Anchor(est_start, park_start)
        for v, w in zip(an.to_park(est_start), park_start):
            self.assertAlmostEqual(v, w)
        # 1 m "forward" in the estimate's frame is 1 m along park +x
        fwd = (0.3 + math.cos(math.radians(40)), -0.2 + math.sin(math.radians(40)),
               math.radians(40))
        x, y, th = an.to_park(fwd)
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(th, 0.0)
        for v, w in zip(an.to_est(an.to_park((1.2, 0.7, 2.0))), (1.2, 0.7, 2.0)):
            self.assertAlmostEqual(v, w)


class PlanTests(unittest.TestCase):
    def test_lap_visits_every_corner_faces_bins_and_ends_at_start_heading(self):
        layout = park.load_park("default", bins=2)         # bins on corners 1 and 2
        steps = rg.build_plan(layout, laps=1, dwell_s=15)
        drives = [s for s in steps if s[0] == "drive"]
        self.assertEqual([(s[1], s[2]) for s in drives], [(1, 2), (2, 3), (3, 4), (4, 1)])
        self.assertEqual([round(math.degrees(s[5])) for s in drives], [0, 90, 180, -90])
        faces = [(s[1], round(math.degrees(s[3]))) for s in steps if s[0] == "face_bin"]
        self.assertEqual(faces, [(2, -45), (1, -135)])
        self.assertEqual([s[3] for s in steps if s[0] == "dwell"], [15.0, 15.0])
        self.assertEqual(steps[-1][0], "turn")
        self.assertAlmostEqual(steps[-1][2], 0.0)          # back to the start heading
        # the bin stop sits between arriving and turning to the next side
        i = steps.index(next(s for s in steps if s[0] == "face_bin"))
        self.assertEqual([steps[i - 1][0], steps[i + 1][0], steps[i + 2][0]],
                         ["drive", "dwell", "turn"])

    def test_two_laps_and_no_bins(self):
        layout = park.load_park("default")
        self.assertEqual(sum(s[0] == "drive" for s in rg.build_plan(layout, laps=2)), 8)
        plain = rg.build_plan(layout, face_bins=False)
        self.assertFalse([s for s in plain if s[0] in ("face_bin", "dwell")])


if __name__ == "__main__":
    unittest.main()
