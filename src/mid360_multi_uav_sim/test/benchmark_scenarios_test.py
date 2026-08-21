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
                            "S05": 1, "S06": 1, "S07": 1}
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
