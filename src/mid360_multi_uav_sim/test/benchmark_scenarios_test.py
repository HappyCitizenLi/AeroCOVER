#!/usr/bin/env python3

import importlib.util
import math
import os
import unittest

import yaml


ROOT = os.path.dirname(os.path.dirname(__file__))
CONFIG_ROOT = os.path.join(ROOT, "config", "benchmarks")
SPEC = importlib.util.spec_from_file_location(
    "benchmark_scenario", os.path.join(ROOT, "scripts", "benchmark_scenario.py"))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class BenchmarkScenariosTest(unittest.TestCase):
    def load(self, scene):
        with open(os.path.join(CONFIG_ROOT, scene + ".yaml"), encoding="utf-8") as stream:
            return yaml.safe_load(stream)

    def test_all_scenes_are_target_free_during_common_warmup(self):
        expected_targets = {"S01": 1, "S02": 1, "S03": 2, "S04": 4,
                            "S05": 1, "S06": 1, "S07": 1,
                            "S08A": 0, "S08B": 1, "S08C": 1,
                            "NEG01": 0, "NEG02": 0, "NEG03": 0,
                            "NEG04": 0}
        for scene, count in expected_targets.items():
            config = self.load(scene)
            self.assertEqual(config["schema_version"], 1)
            self.assertEqual(len(config["targets"]), count)
            self.assertGreaterEqual(config["score_start_s"], 10.0)
            self.assertLess(config["score_start_s"], config["score_end_s"])
            self.assertLessEqual(config["score_end_s"], config["duration_s"])
            for target in config["targets"]:
                self.assertGreaterEqual(target["spawn_time_s"], config["score_start_s"])
                self.assertEqual(target["waypoints"][0]["t"], 0.0)
                self.assertEqual(target["waypoints"][-1]["t"], config["duration_s"])

    def test_negative_controls_and_cold_start_contracts(self):
        for scene in ("NEG01", "NEG02", "NEG03", "NEG04"):
            config = self.load(scene)
            self.assertEqual(config["targets"], [])
            self.assertGreaterEqual(config["duration_s"], 120.0)
            self.assertGreater(config["score_end_s"] - config["score_start_s"],
                               108.0)
        self.assertEqual(self.load("S08A")["targets"], [])
        moving = self.load("S08B")["targets"][0]["waypoints"]
        self.assertGreater(len({(item["x"], item["y"], item["z"])
                                for item in moving}), 1)
        stationary = self.load("S08C")["targets"][0]["waypoints"]
        self.assertEqual(len({(item["x"], item["y"], item["z"])
                              for item in stationary}), 1)
        hover = self.load("IT11")["targets"][0]["waypoints"]
        self.assertEqual({(item["x"], item["y"], item["z"])
                          for item in hover}, {(8.0, 0.0, 1.75)})
        for scene in ("S08B", "S08C"):
            config = self.load(scene)
            target = config["targets"][0]
            observer_waypoints = MODULE.parse_waypoints(
                config["observer"]["waypoints"], "observer",
                config["duration_s"])
            target_waypoints = MODULE.parse_waypoints(
                target["waypoints"], "target", config["duration_s"])
            observer, _ = MODULE.sample(observer_waypoints,
                                        target["spawn_time_s"])
            position, _ = MODULE.sample(target_waypoints,
                                        target["spawn_time_s"])
            spawn_range = math.sqrt(sum(
                (position[index] - observer[index]) ** 2
                for index in range(3)))
            self.assertGreater(spawn_range, 21.5)
            self.assertLess(spawn_range, 22.5)
            self.assertLess(position[0], 24.5)
            self.assertGreater(position[2], 8.5)
            if scene == "S08B":
                next_position, _ = MODULE.sample(
                    target_waypoints, target["spawn_time_s"] + 1.0)
                self.assertGreater(math.sqrt(sum(
                    (next_position[index] - position[index]) ** 2
                    for index in range(3))), 0.5)

    def test_calibration_splits_are_disjoint_and_semantic(self):
        expected = {
            "CAL01": ("map_static", 0),
            "CAL02": ("map_moving_observer", 0),
            "CAL03": ("birth_sparse", 1),
            "CAL04": ("opportunity_return", 1),
            "CAL05": ("track_survival", 1),
            "CAL10": ("opportunity_geometry", 1),
            "CAL11": ("opportunity_correlation", 1),
            "CAL12": ("opportunity_range_angle", 1),
            "CAL13": ("opportunity_fill_factor", 1),
            "CAL14": ("opportunity_miss_likelihood", 1),
        }
        for scene, (suffix, targets) in expected.items():
            config = self.load(scene)
            self.assertEqual(config["scenario_id"], scene + "_" + suffix)
            self.assertEqual(len(config["targets"]), targets)
            self.assertNotIn(config["scenario_id"], {
                self.load(test_scene)["scenario_id"]
                for test_scene in ("S01", "S02", "S03", "S04", "S05",
                                   "S06", "S07", "S08A", "S08B", "S08C")})
        cal05 = self.load("CAL05")
        self.assertEqual(len({
            (item["x"], item["y"], item["z"])
            for item in cal05["targets"][0]["waypoints"]}), 1)
        self.assertGreater(len({
            (item["x"], item["y"], item["z"])
            for item in cal05["observer"]["waypoints"]}), 1)

    def test_required_semantic_stressors_are_declared(self):
        s01 = self.load("S01")
        self.assertIn("hover", s01["purpose"])
        ranges = {item["x"] for item in s01["targets"][0]["waypoints"]}
        self.assertTrue({5.0, 10.0, 20.0, 29.0}.issubset(ranges))

        s03 = self.load("S03")
        event_ids = {item["id"] for item in s03["events"]}
        self.assertEqual(event_ids, {
            "crossing_separation_0p5", "crossing_separation_1p0",
            "crossing_separation_2p0", "crossing_separation_5p0"})
        self.assertEqual(len(self.load("S04")["targets"]), 4)
        self.assertEqual(
            {item["id"] for item in self.load("S05")["events"]
             if item["id"].endswith("start")},
            {"occlusion_1s_start", "occlusion_3s_start", "occlusion_5s_start"})
        self.assertIn("takeoff", self.load("S06")["purpose"])
        s07 = self.load("S07")
        self.assertTrue(any(abs(item.get("roll", 0.0)) > 0.0
                            and abs(item.get("pitch", 0.0)) > 0.0
                            for item in s07["observer"]["waypoints"]))

    def test_interpolation_and_quaternion_are_finite(self):
        waypoints = [(0.0, (0.0,) * 6), (2.0, (2.0, 4.0, 6.0, 0.2, 0.4, 0.6))]
        values, rates = MODULE.sample(waypoints, 1.0)
        self.assertEqual(values[:3], (1.0, 2.0, 3.0))
        self.assertEqual(rates[:3], (1.0, 2.0, 3.0))
        q = MODULE.quaternion(*values[3:])
        self.assertAlmostEqual(sum(value * value for value in q), 1.0)
        self.assertTrue(all(math.isfinite(value) for value in q))


if __name__ == "__main__":
    unittest.main()
