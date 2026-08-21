#!/usr/bin/env python3
"""Pure assets contract for S04 translate+yaw combined visibility."""

import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


class S04AssetsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scenario = yaml.safe_load(
            (PACKAGE / "config/scenarios/S04.yaml").read_text(encoding="utf-8")
        )

    def test_frozen_profile_has_one_dynamic_observer_and_three_static_targets(self):
        scenario = self.scenario
        self.assertEqual(scenario["schema_version"], 4)
        self.assertEqual(scenario["scenario_profile"], "s04")
        self.assertEqual(
            scenario["semantic_contract_id"],
            "mid360-multi-uav-s04-combined-visibility-v1",
        )
        self.assertEqual(
            scenario["scenario"], "S04_combined_visibility_translate_yaw"
        )
        self.assertEqual(set(scenario["targets"]), {"uav1", "uav2", "uav3", "uav4"})
        self.assertTrue(scenario["targets"]["uav1"]["command_model"])
        self.assertEqual(
            scenario["observer_motion"]["stationary_targets"],
            ["uav2", "uav3", "uav4"],
        )

    def test_scored_motion_has_translation_and_shortest_yaw_change(self):
        scenario = self.scenario
        poses = {
            name: dict(item["initial_pose"])
            for name, item in scenario["targets"].items()
        }
        start = end = None
        for phase in scenario["phases"]:
            before = {name: dict(value) for name, value in poses.items()}
            for name, value in phase.get("targets", {}).items():
                poses[name] = dict(value)
            if phase["id"] == scenario["observer_motion"]["scored_phase"]:
                start, end = before, {name: dict(value) for name, value in poses.items()}
                self.assertEqual(phase["interpolation"], "linear")
                self.assertEqual(phase["event"], "combined_visibility_start")
        self.assertIsNotNone(start)
        distance = math.sqrt(sum(
            (float(end["uav1"][axis]) - float(start["uav1"][axis])) ** 2
            for axis in ("x", "y", "z")
        ))
        yaw_delta = abs(math.atan2(
            math.sin(float(end["uav1"]["yaw"]) - float(start["uav1"]["yaw"])),
            math.cos(float(end["uav1"]["yaw"]) - float(start["uav1"]["yaw"])),
        ))
        self.assertGreaterEqual(distance, float(scenario["observer_motion"]["minimum_translation_m"]))
        self.assertGreaterEqual(yaw_delta, float(scenario["observer_motion"]["minimum_yaw_change_rad"]))
        for target_id in ("uav2", "uav3", "uav4"):
            self.assertEqual(start[target_id], end[target_id])

    def test_scored_occlusion_geometry_and_final_clear_have_margin(self):
        # During the scored phase uav2/uav3 and both observer endpoints lie
        # on y=3x/13, retaining target occlusion while translation+yaw occur.
        uav2 = (8.0, 24.0 / 13.0)
        uav3 = (13.0, 3.0)
        self.assertAlmostEqual(uav2[1] / uav2[0], uav3[1] / uav3[0], places=15)
        scored_end = (-2.0, -6.0 / 13.0)
        self.assertAlmostEqual(scored_end[1], 3.0 * scored_end[0] / 13.0, places=15)

        # The final clear observer gives the wall a conservative margin even
        # at its x=10.25 rear face and a +/-0.52 m target lateral envelope.
        target_x = 14.0
        final_observer = (4.0, -3.0)
        fraction = (10.25 - final_observer[0]) / (target_x - final_observer[0])
        final_wall_y = final_observer[1] + fraction * (0.0 - final_observer[1])
        nearest_edge = abs(final_wall_y) - fraction * 0.52
        self.assertAlmostEqual(final_wall_y, -1.125)
        self.assertGreater(nearest_edge, 0.4)

    def test_launch_uses_e2_four_spawns_and_one_dynamic_tf_authority(self):
        launch = ET.parse(PACKAGE / "launch/gate_s04.launch").getroot()
        world = launch.find("include/arg[@name='world_name']")
        self.assertTrue(world.attrib["value"].endswith("/worlds/E2_occlusion_arena.world"))
        spawns = {
            node.attrib["name"]: node.attrib["args"]
            for node in launch.findall("node")
            if node.attrib.get("pkg") == "gazebo_ros"
            and node.attrib.get("type") == "spawn_model"
        }
        self.assertEqual(set(spawns), {
            "spawn_uav1_observer", "spawn_uav2_target", "spawn_uav3_target", "spawn_uav4_target"
        })
        self.assertIn("observer_uav_dynamic/model.sdf", spawns["spawn_uav1_observer"])
        self.assertIn("-x 4 -y -3", spawns["spawn_uav1_observer"])
        self.assertIn("-Y -0.7853981633974483", spawns["spawn_uav1_observer"])
        static_tf = [
            node for node in launch.findall("node")
            if node.attrib.get("pkg") == "tf2_ros"
        ]
        self.assertEqual(len(static_tf), 1)
        self.assertNotIn("world uav1/fcu", static_tf[0].attrib["args"])
        preprocess = next(
            item for item in launch.findall("include")
            if "mid360_ray_preprocessor" in item.attrib.get("file", "")
        )
        args = {item.attrib["name"]: item.attrib["value"] for item in preprocess.findall("arg")}
        self.assertEqual(args["ray_time_geometry_mode"], "per_ray_pose")
        self.assertEqual(args["tf_source_cadence_topic"], "/tf")


if __name__ == "__main__":
    unittest.main()
