#!/usr/bin/env python3
"""Pure asset and continuous collision contracts for the S1 formation."""

import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def _pose(value):
    return np.asarray([float(value[key]) for key in ("x", "y", "z")])


def _collision_extents():
    root = ET.parse(
        PACKAGE / "models" / "simplified_target_uav" / "model.sdf"
    ).getroot()
    horizontal_radius = 0.0
    half_height = 0.0
    for collision in root.findall(".//collision"):
        pose = tuple(float(value) for value in
                     collision.findtext("pose", "0 0 0 0 0 0").split())
        box = collision.find("geometry/box/size")
        cylinder = collision.find("geometry/cylinder")
        if cylinder is not None:
            radial = float(cylinder.findtext("radius"))
            horizontal_radius = max(
                horizontal_radius, math.hypot(pose[0], pose[1]) + radial
            )
            vertical = 0.5 * float(cylinder.findtext("length"))
        elif box is not None:
            size = tuple(float(value) for value in box.text.split())
            yaw = pose[5]
            half_x = 0.5 * (abs(math.cos(yaw)) * size[0] +
                            abs(math.sin(yaw)) * size[1])
            half_y = 0.5 * (abs(math.sin(yaw)) * size[0] +
                            abs(math.cos(yaw)) * size[1])
            horizontal_radius = max(
                horizontal_radius,
                math.hypot(pose[0], pose[1]) + math.hypot(half_x, half_y),
            )
            vertical = 0.5 * size[2]
        else:
            raise AssertionError("unsupported collision primitive")
        half_height = max(half_height, abs(pose[2]) + vertical)
    return horizontal_radius, half_height


def _smoothstep(value):
    return value * value * (3.0 - 2.0 * value)


class S1AssetsContract(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.scenario = yaml.safe_load(
            (PACKAGE / "config" / "scenarios" / "S1.yaml").read_text(
                encoding="utf-8"
            )
        )

    def test_exact_timeline_and_formation(self):
        scenario = self.scenario
        self.assertEqual(scenario["schema_version"], 1)
        self.assertEqual(scenario["scenario"],
                         "S1_three_target_vertical_formation")
        self.assertEqual(scenario["seed"], 1001)
        self.assertEqual(float(scenario["start_delay"]), 6.0)
        self.assertEqual(float(scenario["duration"]), 30.0)
        self.assertEqual(float(scenario["final_hold"]), 3.0)
        phases = scenario["phases"]
        self.assertEqual(
            [phase["id"] for phase in phases],
            ["horizontal_line_flight", "form_vertical_stack",
             "vertical_stack_flight", "restore_horizontal_line",
             "restored_line_flight"],
        )
        self.assertEqual([float(phase["duration"]) for phase in phases],
                         [6.0, 6.0, 8.0, 6.0, 4.0])
        self.assertEqual(sum(float(phase["duration"]) for phase in phases),
                         30.0)

        targets = scenario["targets"]
        initial = [_pose(targets[name]["initial_pose"])
                   for name in ("uav2", "uav3", "uav4")]
        np.testing.assert_allclose(
            initial,
            [[6.0, -3.0, 3.25], [6.0, 0.0, 3.25],
             [6.0, 3.0, 3.25]],
        )
        stack = [_pose(phases[1]["targets"][name])
                 for name in ("uav2", "uav3", "uav4")]
        np.testing.assert_allclose(
            stack,
            [[9.0, 0.0, 2.25], [9.0, 0.0, 3.25],
             [9.0, 0.0, 4.25]],
        )

    def test_line_is_separate_stack_is_merge_eligible_and_safe(self):
        contract = self.scenario["formation_contract"]
        radius, half_height = _collision_extents()
        self.assertAlmostEqual(radius,
                               float(contract["target_horizontal_collision_radius_m"]))
        self.assertAlmostEqual(half_height,
                               float(contract["target_vertical_collision_half_extent_m"]))
        line_spacing = float(contract["line_center_spacing_m"])
        vertical_spacing = float(contract["stack_vertical_center_spacing_m"])
        tolerance = float(contract["euclidean_cluster_tolerance_m"])
        line_clearance = line_spacing - 2.0 * radius
        stack_clearance = vertical_spacing - 2.0 * half_height
        self.assertAlmostEqual(
            line_clearance, float(contract["line_min_horizontal_clearance_m"])
        )
        self.assertAlmostEqual(
            stack_clearance, float(contract["stack_min_vertical_clearance_m"])
        )
        self.assertGreater(line_clearance, tolerance)
        self.assertGreater(stack_clearance, 0.0)
        self.assertLess(stack_clearance, tolerance)
        self.assertLess(vertical_spacing, tolerance)

        # A conservative three-target stack envelope remains below B0's
        # maximum component size even when measured by its 3-D diagonal.
        horizontal_span = 2.0 * 0.52
        vertical_span = 2.0 * vertical_spacing + 2.0 * half_height
        diagonal = math.sqrt(2.0 * horizontal_span ** 2 + vertical_span ** 2)
        self.assertLess(diagonal,
                        float(contract["b0_max_component_size_m"]))

    def test_compression_and_expansion_are_collision_free(self):
        radius, half_height = _collision_extents()
        # Adjacent aircraft follow delta=(0, 3(1-s), -s), where s is the
        # common smoothstep progress.  Conservative collision disks may
        # overlap horizontally or vertically, but never in both dimensions.
        minimum_center_distance = math.inf
        for index in range(10001):
            s = _smoothstep(index / 10000.0)
            horizontal = 3.0 * (1.0 - s)
            vertical = s
            minimum_center_distance = min(
                minimum_center_distance, math.hypot(horizontal, vertical)
            )
            self.assertFalse(
                horizontal <= 2.0 * radius and
                vertical <= 2.0 * half_height
            )
        self.assertGreater(minimum_center_distance, 0.94)

    def test_launch_is_fixed_observer_snapshot_with_four_models(self):
        launch = ET.parse(PACKAGE / "launch" / "gate_s1.launch").getroot()
        world = launch.find("include/arg[@name='world_name']")
        self.assertTrue(world.attrib["value"].endswith("/worlds/E0_open.world"))
        spawns = {
            node.attrib["name"]: node.attrib["args"]
            for node in launch.findall("node")
            if node.attrib.get("type") == "spawn_model"
        }
        self.assertEqual(set(spawns), {
            "spawn_uav1_observer", "spawn_uav2_target",
            "spawn_uav3_target", "spawn_uav4_target",
        })
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
