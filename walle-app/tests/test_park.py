"""Checks for walle-app/park.py.   python3 -m unittest discover -s walle-app/tests"""

import math
import os
import sys
import tempfile
import unittest

import yaml

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import park  # noqa: E402


def write_scenario(d, text):
    path = os.path.join(d, "s.yaml")
    with open(path, "w") as f:
        f.write(text)
    return path


class ParkTests(unittest.TestCase):
    def test_every_shipped_scenario_loads(self):
        for f in os.listdir(park.SCENARIO_DIR):
            if f.endswith(".yaml"):
                park.load_park(f[:-5])

    def test_same_seed_same_layout_and_new_seed_changes_it(self):
        a, b = park.load_park("mixed"), park.load_park("mixed")
        self.assertEqual(a, b)
        c = park.load_park("mixed", seed=99)
        self.assertNotEqual(a["obstacle"]["position"], c["obstacle"]["position"])

    def test_bin_count_override_keeps_first_corners_and_obstacle(self):
        full = park.load_park("mixed")
        two = park.load_park("mixed", bins=2)
        self.assertEqual([b["corner"] for b in two["bins"]], [1, 2])
        self.assertEqual(two["bins"], full["bins"][:2])
        self.assertEqual(two["obstacle"], full["obstacle"])
        self.assertEqual(park.load_park("mixed", bins=0)["bins"], [])

    def test_more_bins_than_scenario_adds_ok_bins(self):
        with tempfile.TemporaryDirectory() as d:
            path = write_scenario(d, "bins: {3: {state: overflowing, level: 2}}\nobstacle: none\n")
            layout = park.load_park(path, bins=3)
        self.assertEqual([(b["corner"], b["state"]) for b in layout["bins"]],
                         [(1, "ok"), (2, "ok"), (3, "overflowing")])

    def test_obstacle_override(self):
        self.assertIsNone(park.load_park("mixed", obstacle="none")["obstacle"])
        o = park.load_park("default", obstacle="2")["obstacle"]
        self.assertEqual(o["side"], 2)
        self.assertAlmostEqual(o["position"][0], 1.0)      # side 2 is x = 1, corner 2 -> 3
        self.assertIsNotNone(park.load_park("default", obstacle="random")["obstacle"])

    def test_nothing_is_placed_on_the_route(self):
        for seed in range(30):
            layout = park.load_park("mixed", seed=seed)
            geo = park.Geometry(park.load_yaml(park.PARK_FILE))
            for b in layout["bins"]:
                x, y = b["position"]
                self.assertGreater(geo.dist_to_route(x, y) - b["radius_m"], geo.half_width)
                for i in b["spill"] + b["litter"]:
                    foot = geo.footprint(i["item"], i["pose"])
                    self.assertGreater(geo.dist_to_route(*i["position"][:2]) - foot,
                                       geo.half_width)
            o = layout["obstacle"]
            self.assertLess(geo.dist_to_route(*o["position"][:2]), 1e-9)   # ON the route
            self.assertGreater(geo.dist_to_nearest_corner(*o["position"][:2]), geo.turn_radius)

    def test_overflow_mound_grows_with_level(self):
        with tempfile.TemporaryDirectory() as d:
            heights = []
            for level in (1, 2, 3):
                path = write_scenario(d, f"bins: {{1: {{state: overflowing, level: {level}}}}}\n")
                b = park.load_park(path)["bins"][0]
                heights.append(max(i["position"][2] for i in b["mound"]))
                self.assertTrue(all(i["position"][2] > b["height_m"] for i in b["mound"]))
        self.assertEqual(heights, sorted(heights))

    def test_bad_input_is_refused(self):
        bad = ["bins: {5: {state: ok}}", "bins: {1: {state: full}}",
               "bins: {1: {state: ok, level: 2}}", "bins: {1: {state: overflowing, level: 9}}",
               "bins: {1: {state: ok, litter: 99}}", "obstacle: {side: 1, at: 0.05}",
               "obstacle: {item: banana}", "colour: red"]
        with tempfile.TemporaryDirectory() as d:
            for text in bad:
                with self.subTest(text=text), self.assertRaises(park.ParkError):
                    park.load_park(write_scenario(d, text))

    def test_too_small_bin_offset_is_refused(self):
        cfg = park.load_yaml(park.PARK_FILE)
        cfg["bins"]["offset_m"] = 0.2
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "park.yaml")
            with open(path, "w") as f:
                yaml.safe_dump(cfg, f)
            with self.assertRaises(park.ParkError):
                park.load_park("default", park_file=path)

    def test_auto_offset_clears_the_robot(self):
        geo = park.Geometry(park.load_yaml(park.PARK_FILE))
        perpendicular = geo.bin_offset / math.sqrt(2) - geo.bin_radius
        self.assertGreaterEqual(perpendicular + 1e-9, geo.half_width + geo.margin)
        self.assertGreaterEqual(geo.bin_offset - geo.bin_radius + 1e-9,
                                geo.turn_radius + geo.margin)


if __name__ == "__main__":
    unittest.main()
