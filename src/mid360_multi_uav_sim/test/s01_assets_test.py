#!/usr/bin/env python3
"""Pure trajectory/asset contracts for the collision-free S01 crossing."""

import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def _pose(value):
    return tuple(float(value[key]) for key in ("x", "y", "z", "yaw"))


def _target_collision_radius():
    model = ET.parse(
        PACKAGE / "models" / "simplified_target_uav" / "model.sdf"
    ).getroot().find("model")
    radius = 0.0
    for collision in model.findall(".//collision"):
        pose_text = collision.findtext("pose", "0 0 0 0 0 0")
        pose = tuple(float(part) for part in pose_text.split())
        cylinder = collision.find("geometry/cylinder")
        if cylinder is not None:
            extent = math.hypot(pose[0], pose[1]) + float(
                cylinder.findtext("radius")
            )
            radius = max(radius, extent)
    return radius


def _target_collision_half_height():
    model = ET.parse(
        PACKAGE / "models" / "simplified_target_uav" / "model.sdf"
    ).getroot().find("model")
    extent = 0.0
    for collision in model.findall(".//collision"):
        pose = tuple(float(part) for part in
                     collision.findtext("pose", "0 0 0 0 0 0").split())
        box = collision.find("geometry/box/size")
        cylinder = collision.find("geometry/cylinder")
        if box is not None:
            half = 0.5 * float(box.text.split()[2])
        elif cylinder is not None:
            half = 0.5 * float(cylinder.findtext("length"))
        else:
            raise AssertionError("unsupported collision primitive")
        extent = max(extent, abs(pose[2]) + half)
    return extent


class S01AssetsContract(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.scenario = yaml.safe_load(
            (PACKAGE / "config" / "scenarios" / "S01.yaml").read_text(
                encoding="utf-8"
            )
        )

    def test_timeline_and_continuous_transit(self):
        scenario = self.scenario
        self.assertEqual(scenario["schema_version"], 1)
        self.assertEqual(scenario["scenario"], "S01_time_separated_airway_crossing")
        self.assertEqual(scenario["seed"], 404)
        self.assertEqual(scenario["repeat_count"], 1)
        self.assertEqual(scenario["repeat_mode"], "restart")
        self.assertEqual(float(scenario["start_delay"]), 5.0)
        self.assertEqual(float(scenario["duration"]), 32.0)
        self.assertEqual(float(scenario["final_hold"]), 3.0)
        phases = scenario["phases"]
        self.assertEqual(
            [phase["id"] for phase in phases],
            ["track_establishment_hold", "continuous_crossing_transit",
             "post_crossing_hold"],
        )
        self.assertEqual(
            [float(phase["duration"]) for phase in phases], [3.0, 26.0, 3.0]
        )
        self.assertEqual(
            [phase["interpolation"] for phase in phases],
            ["hold", "linear", "hold"],
        )
        self.assertEqual(sum(float(p["duration"]) for p in phases), 32.0)

    def test_crossing_is_time_separated_safe_and_merge_eligible(self):
        scenario = self.scenario
        targets = scenario["targets"]
        self.assertEqual(_pose(targets["uav1"]["initial_pose"]),
                         (0.0, 0.0, 2.0, 0.0))
        start_a = _pose(targets["uav2"]["initial_pose"])
        start_b = _pose(targets["uav3"]["initial_pose"])
        goals = scenario["phases"][1]["targets"]
        end_a = _pose(goals["uav2"])
        end_b = _pose(goals["uav3"])
        self.assertEqual(start_a, (4.4, 0.0, 2.25, 0.0))
        self.assertEqual(start_b, (8.0, -3.9, 3.25, math.pi / 2.0))
        self.assertEqual(end_a, (12.2, 0.0, 2.25, 0.0))
        self.assertEqual(end_b, (8.0, 3.9, 3.25, math.pi / 2.0))

        contract = scenario["crossing_contract"]
        self.assertEqual(contract["geometry_mode"], "snapshot")
        speed = float(contract["speed_mps"])
        duration = float(scenario["phases"][1]["duration"])
        self.assertAlmostEqual((end_a[0] - start_a[0]) / duration, speed)
        self.assertAlmostEqual((end_b[1] - start_b[1]) / duration, speed)
        crossing_a = (8.0 - start_a[0]) / speed
        crossing_b = (0.0 - start_b[1]) / speed
        self.assertAlmostEqual(crossing_a,
                               float(contract["eastbound_crossing_time_sec"]))
        self.assertAlmostEqual(crossing_b,
                               float(contract["northbound_crossing_time_sec"]))
        self.assertAlmostEqual(crossing_b - crossing_a,
                               float(contract["crossing_time_separation_sec"]))

        minimum = math.inf
        for index in range(26001):
            t = duration * index / 26000.0
            a = np.asarray((start_a[0] + speed * t, start_a[1], start_a[2]))
            b = np.asarray((start_b[0], start_b[1] + speed * t, start_b[2]))
            minimum = min(minimum, float(np.linalg.norm(a - b)))
        self.assertAlmostEqual(minimum,
                               float(contract["minimum_center_separation_m"]),
                               places=9)
        half_height = _target_collision_half_height()
        self.assertAlmostEqual(
            half_height,
            float(contract["target_vertical_collision_half_extent_m"]),
        )
        clearance = abs(start_b[2] - start_a[2]) - 2.0 * half_height
        self.assertAlmostEqual(clearance,
                               float(contract["minimum_vertical_collision_clearance_m"]),
                               places=9)
        self.assertGreater(clearance, 0.80)
        self.assertLess(clearance,
                        float(contract["euclidean_cluster_tolerance_m"]))

    def test_launch_uses_open_world_snapshot_and_exact_spawns(self):
        launch = ET.parse(PACKAGE / "launch" / "gate_s01.launch").getroot()
        world = launch.find("include/arg[@name='world_name']")
        self.assertTrue(world.attrib["value"].endswith("/worlds/E0_open.world"))
        spawns = {
            node.attrib["name"]: node.attrib["args"]
            for node in launch.findall("node")
            if node.attrib.get("type") == "spawn_model"
        }
        self.assertEqual(set(spawns), {
            "spawn_uav1_observer", "spawn_uav2_target", "spawn_uav3_target"
        })
        self.assertIn("-x 4.4 -y 0 -z 2.25", spawns["spawn_uav2_target"])
        self.assertIn("-x 8 -y -3.9 -z 3.25", spawns["spawn_uav3_target"])
        preprocessor = next(
            include for include in launch.findall("include")
            if "mid360_ray_preprocessor" in include.attrib.get("file", "")
        )
        args = {arg.attrib["name"]: arg.attrib["value"]
                for arg in preprocessor.findall("arg")}
        self.assertEqual(args["ray_time_geometry_mode"], "snapshot")
        self.assertEqual(args["output_frame"], "world")


if __name__ == "__main__":
    unittest.main()
