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


class PulseTests(unittest.TestCase):
    # walle-sim plastic_pid: gain ~110 deg/s, dead ~0.04 s
    def test_duration_grows_with_error(self):
        t5 = rg.plan_pulse(5.0, 110.0, 0.04, 0.06, 0.30)
        t10 = rg.plan_pulse(-10.0, 110.0, 0.04, 0.06, 0.30)
        self.assertAlmostEqual(t5, 0.04 + 5 / 110)
        self.assertAlmostEqual(t10, 0.04 + 10 / 110)
        self.assertEqual(rg.plan_pulse(60.0, 110.0, 0.04, 0.06, 0.30), 0.30)

    def test_skip_when_shortest_pulse_overshoots(self):
        # shortest pulse turns 110 * 0.02 = 2.2 deg: worth it for 1.5 deg, not for 1.0
        self.assertEqual(rg.plan_pulse(1.5, 110.0, 0.04, 0.06, 0.30), 0.06)
        self.assertIsNone(rg.plan_pulse(1.0, 110.0, 0.04, 0.06, 0.30))
        # a robot that lurches (gain 300): the shortest pulse is 6 deg, skip a 3 deg miss
        self.assertIsNone(rg.plan_pulse(3.0, 300.0, 0.04, 0.06, 0.30))

    def test_gain_learning(self):
        # asked 0.1 s at gain 110 (expects 6.6 deg) but it turned 13 deg: gain goes up
        g = rg.learn_pulse_gain(110.0, 0.10, 13.0, 0.04, 0.5)
        self.assertAlmostEqual(g, 110 + 0.5 * (13 / 0.06 - 110))
        self.assertEqual(rg.learn_pulse_gain(110.0, 0.10, 0.2, 0.04, 0.5), 110.0)  # stuck
        self.assertEqual(rg.learn_pulse_gain(110.0, 0.05, 3.0, 0.04, 0.5), 110.0)  # too short
        self.assertEqual(rg.learn_pulse_gain(110.0, 0.10, 90.0, 0.04, 0.5), 400.0)  # clamped


if __name__ == "__main__":
    unittest.main()


class EstimateGuardTests(unittest.TestCase):
    """Pico restart and estimate-jump guards (robot run 20261122_142402)."""

    def test_sideways_jump_is_caught(self):
        # that run, 0.5 s: fused +40 cm sideways, wheels 7 cm forward
        extra_m, _, jump = rg.estimate_jump((0, 0, 0), (-0.04, 0.37, 0), (0, 0, 0),
                                            (0.05, 0, -0.05), 0.05, 20.0)
        self.assertTrue(jump)
        self.assertGreater(extra_m, 0.3)

    def test_normal_driving_and_turning_pass(self):
        # 4 cm in 0.25 s, both agree within a cm; a 10 deg pulse the wheels see as 7
        self.assertFalse(rg.estimate_jump((0, 0, 0), (0.04, 0.002, 0), (0, 0, 0),
                                          (0.035, 0, 0), 0.05, 20.0)[2])
        self.assertFalse(rg.estimate_jump((0, 0, 0), (0, 0, math.radians(10)), (0, 0, 0),
                                          (0, 0, math.radians(7)), 0.05, 20.0)[2])

    def test_heading_jump_is_caught(self):
        self.assertTrue(rg.estimate_jump((0, 0, 0), (0, 0, math.radians(35)), (0, 0, 0),
                                         (0, 0, 0), 0.05, 20.0)[2])

    def test_pico_restarted(self):
        self.assertFalse(rg.pico_restarted(None, (0, 0, 0), 20000))
        self.assertFalse(rg.pico_restarted((51000, 50800, 40), (51040, 50830, 41), 20000))
        self.assertTrue(rg.pico_restarted((51000, 50800, 40), (3, 2, 0), 20000))
        # early in a session the counts are small, but ping_seq still goes backwards
        self.assertTrue(rg.pico_restarted((900, 850, 120), (0, 0, 1), 20000))
        self.assertFalse(rg.pico_restarted((900, 850), (0, 0), 20000))      # no seq: counts only
