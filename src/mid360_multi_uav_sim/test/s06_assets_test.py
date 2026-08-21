#!/usr/bin/env python3
"""Strict pure YAML/XML/launch contracts for the S06 online-map fixture."""

import math
from pathlib import Path
import shlex
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
SEMANTIC_CONTRACT_ID = "mid360-multi-uav-s06-online-map-occlusion-v1"
TARGET_IDS = ("uav2", "uav3", "uav4")
TRUTH_IDS = ("uav1",) + TARGET_IDS
ONLINE_MAP_CONTRACT = {
    "initial_state": "empty_online_scores_no_apriori_loader",
    "apriori_loader_used": False,
    "initial_map_revision": 0,
    "bootstrap_phase": "map_bootstrap_clear_hold",
    "discovery_primitive": "background_wall/wall_collision",
    "scored_map_occluded_target": "uav4",
    "transition": "initial_unknown_to_committed_sure_occupied",
}


def _spawn_arguments(node):
    normalized = node.attrib["args"].replace(
        "$(find mid360_multi_uav_sim)", "MID360_MULTI_UAV_SIM"
    )
    tokens = shlex.split(normalized)
    values = {}
    index = 0
    while index < len(tokens):
        token = tokens[index]
        if token in ("-model", "-file", "-x", "-y", "-z", "-Y"):
            if index + 1 >= len(tokens):
                raise AssertionError("spawn argument {} lacks a value".format(token))
            values[token] = tokens[index + 1]
            index += 2
        else:
            index += 1
    return values


class S06AssetsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scenario_path = PACKAGE / "config/scenarios/S06.yaml"
        cls.launch_path = PACKAGE / "launch/gate_s06.launch"
        cls.world_path = PACKAGE / "worlds/E2_occlusion_arena.world"
        cls.scenario = yaml.safe_load(cls.scenario_path.read_text(encoding="utf-8"))
        cls.launch = ET.parse(cls.launch_path).getroot()
        cls.world = ET.parse(cls.world_path).getroot().find("world")

    def test_frozen_schema_v6_root_runtime_and_online_map_contract(self):
        scenario = self.scenario
        self.assertEqual(
            set(scenario),
            {
                "schema_version", "scenario_profile", "semantic_contract_id",
                "scenario", "world_frame", "seed", "repeat_count",
                "repeat_mode", "start_delay", "duration", "final_hold",
                "runtime", "randomization", "observer_motion",
                "online_map_contract", "targets", "phases",
            },
        )
        self.assertEqual(scenario["schema_version"], 6)
        self.assertEqual(scenario["scenario_profile"], "s06")
        self.assertEqual(scenario["semantic_contract_id"], SEMANTIC_CONTRACT_ID)
        self.assertEqual(scenario["scenario"], "S06_incomplete_map_online_occlusion")
        self.assertEqual(scenario["world_frame"], "world")
        self.assertEqual(scenario["seed"], 909)
        self.assertEqual((scenario["repeat_count"], scenario["repeat_mode"]), (1, "restart"))
        self.assertEqual(
            (float(scenario["start_delay"]), float(scenario["duration"]),
             float(scenario["final_hold"])),
            (5.0, 22.0, 3.0),
        )
        self.assertEqual(scenario["online_map_contract"], ONLINE_MAP_CONTRACT)
        self.assertEqual(scenario["runtime"], {
            "set_model_state_service": "/gazebo/set_model_state",
            "world_properties_service": "/gazebo/get_world_properties",
            "event_topic": "/mid360_multi_uav_sim/scenario_events",
            "publish_rate": 50.0,
            "service_timeout_wall": 30.0,
            "clock_timeout_wall": 30.0,
            "max_clock_stall_wall": 20.0,
            "max_consecutive_service_failures": 3,
            "require_use_sim_time": True,
        })
        self.assertEqual(
            scenario["randomization"]["position_offset_uniform"],
            {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
        )

    def test_four_aircraft_identity_motion_and_stationary_targets_are_exact(self):
        expected = {
            "uav1": (4.0, -3.0, 2.0, -math.pi / 4.0),
            "uav2": (8.0, 24.0 / 13.0, 2.25, 0.0),
            "uav3": (13.0, 3.0, 2.25, 0.0),
            "uav4": (14.0, 0.0, 2.25, 0.0),
        }
        self.assertEqual(set(self.scenario["targets"]), set(expected))
        for name, expected_pose in expected.items():
            target = self.scenario["targets"][name]
            self.assertEqual(target["model_name"], name)
            self.assertIs(target["command_model"], True)
            self.assertEqual(
                target["truth_topic"],
                "/mid360_multi_uav_sim/ground_truth/{}/odom".format(name),
            )
            expected_child = "uav1/fcu" if name == "uav1" else name + "/base_link"
            self.assertEqual(target["child_frame_id"], expected_child)
            actual_pose = tuple(
                float(target["initial_pose"][key]) for key in ("x", "y", "z", "yaw")
            )
            for actual, expected_value in zip(actual_pose, expected_pose):
                self.assertAlmostEqual(actual, expected_value, places=15)
        self.assertEqual(self.scenario["observer_motion"], {
            "observer": "uav1",
            "parent_frame": "world",
            "child_frame": "uav1/fcu",
            "tf_authority": "scenario_manager",
            "unique_dynamic_tf_authority": True,
            "minimum_translation_m": 2.0,
            "minimum_yaw_change_rad": 0.35,
            "scored_phase": "online_occlusion_scored",
            "stationary_targets": list(TARGET_IDS),
        })

    def test_five_phases_and_absolute_event_offsets_are_exact(self):
        phases = self.scenario["phases"]
        self.assertEqual(
            [(phase["id"], phase.get("event"), float(phase["duration"]),
              phase["interpolation"]) for phase in phases],
            [
                ("map_bootstrap_clear_hold", None, 4.0, "hold"),
                ("approach_online_occlusion", "map_bootstrap_end", 4.0, "linear"),
                ("online_occlusion_scored", "online_occlusion_start", 6.0, "linear"),
                ("leave_online_occlusion", "online_occlusion_end", 5.0, "linear"),
                ("final_clear_hold", "final_clear_start", 3.0, "hold"),
            ],
        )
        self.assertEqual(sum(float(phase["duration"]) for phase in phases), 22.0)
        cursor = 0
        offsets = {"cycle_start": 0}
        for phase in phases:
            event = phase.get("event")
            if event is not None:
                offsets[event] = cursor
            cursor += int(round(float(phase["duration"]) * 1.0e9))
        offsets.update({
            "cycle_complete": cursor,
            "final_hold_start": cursor,
            "scenario_complete": cursor + int(self.scenario["final_hold"] * 1.0e9),
        })
        self.assertEqual(offsets, {
            "cycle_start": 0,
            "map_bootstrap_end": 4_000_000_000,
            "online_occlusion_start": 8_000_000_000,
            "online_occlusion_end": 14_000_000_000,
            "final_clear_start": 19_000_000_000,
            "cycle_complete": 22_000_000_000,
            "final_hold_start": 22_000_000_000,
            "scenario_complete": 25_000_000_000,
        })
        expected_goals = {
            "approach_online_occlusion": (0.0, 0.0, 2.0, 0.0),
            "online_occlusion_scored": (-2.0, -6.0 / 13.0, 2.0, 0.4),
            "leave_online_occlusion": (4.0, -3.0, 2.0, math.pi / 4.0),
        }
        for phase in phases:
            if phase["id"] in expected_goals:
                self.assertEqual(set(phase["targets"]), {"uav1"})
                actual = tuple(
                    float(phase["targets"]["uav1"][key])
                    for key in ("x", "y", "z", "yaw")
                )
                for left, right in zip(actual, expected_goals[phase["id"]]):
                    self.assertAlmostEqual(left, right, places=15)
            else:
                self.assertNotIn("targets", phase)

    def test_e2_world_contains_the_discovery_primitive_and_no_aircraft(self):
        self.assertEqual(self.world.attrib["name"], "E2_occlusion_arena")
        includes = {
            item.findtext("name"): item.findtext("uri")
            for item in self.world.findall("include")
        }
        self.assertEqual(includes, {
            "open_ground": "model://mid360_open_ground",
            "background_wall": "model://background_wall",
            "pillar": "model://pillar",
        })
        wall = ET.parse(
            PACKAGE / "models/background_wall/model.sdf"
        ).getroot().find("model")
        self.assertEqual(wall.findtext("static"), "true")
        collisions = wall.findall(".//collision")
        self.assertEqual([item.attrib["name"] for item in collisions], ["wall_collision"])
        self.assertEqual(
            self.scenario["online_map_contract"]["discovery_primitive"],
            "background_wall/{}".format(collisions[0].attrib["name"]),
        )
        self.assertTrue(
            set(TRUTH_IDS).isdisjoint(includes),
            "aircraft must be launch-spawned rather than world fixtures",
        )

    def test_launch_reuses_e2_and_spawns_only_one_sensor_aircraft(self):
        scenario_arg = self.launch.find("arg[@name='scenario_config']")
        self.assertTrue(scenario_arg.attrib["default"].endswith(
            "/config/scenarios/S06.yaml"
        ))
        world_arg = self.launch.find("include/arg[@name='world_name']")
        self.assertTrue(world_arg.attrib["value"].endswith(
            "/worlds/E2_occlusion_arena.world"
        ))
        spawns = {
            values["-model"]: values
            for values in (
                _spawn_arguments(node)
                for node in self.launch.findall("node")
                if node.attrib.get("pkg") == "gazebo_ros"
                and node.attrib.get("type") == "spawn_model"
            )
        }
        self.assertEqual(set(spawns), set(TRUTH_IDS))
        for name in TRUTH_IDS:
            target = self.scenario["targets"][name]
            pose = target["initial_pose"]
            values = spawns[name]
            self.assertAlmostEqual(float(values["-x"]), float(pose["x"]), places=15)
            self.assertAlmostEqual(float(values["-y"]), float(pose["y"]), places=15)
            self.assertAlmostEqual(float(values["-z"]), float(pose["z"]), places=15)
            self.assertAlmostEqual(float(values.get("-Y", 0.0)), float(pose["yaw"]), places=15)
            expected_model = (
                "observer_uav_dynamic" if name == "uav1" else "simplified_target_uav"
            )
            self.assertTrue(values["-file"].endswith(
                "/models/{}/model.sdf".format(expected_model)
            ))
        observer = ET.parse(
            PACKAGE / "models/observer_uav_dynamic/model.sdf"
        ).getroot().find("model")
        target = ET.parse(
            PACKAGE / "models/simplified_target_uav/model.sdf"
        ).getroot().find("model")
        self.assertEqual(len(observer.findall(".//sensor")), 1)
        self.assertEqual(len(target.findall(".//sensor")), 0)
        self.assertEqual(len(target.findall(".//plugin")), 0)

    def test_launch_has_per_ray_geometry_one_tf_authority_and_no_apriori_loader(self):
        static_tf = [
            node for node in self.launch.findall("node")
            if node.attrib.get("pkg") == "tf2_ros"
            and node.attrib.get("type") == "static_transform_publisher"
        ]
        self.assertEqual(len(static_tf), 1)
        self.assertIn("uav1/fcu uav1/mid360_link", static_tf[0].attrib["args"])
        self.assertNotIn("world uav1/fcu", static_tf[0].attrib["args"])
        preprocess = next(
            item for item in self.launch.findall("include")
            if "mid360_ray_preprocessor" in item.attrib.get("file", "")
        )
        values = {
            item.attrib["name"]: item.attrib["value"]
            for item in preprocess.findall("arg")
        }
        self.assertEqual(values, {
            "ray_time_geometry_mode": "per_ray_pose",
            "output_frame": "world",
            "tf_source_cadence_observation_enabled": "true",
            "tf_source_cadence_topic": "/tf",
            "tf_source_cadence_parent_frame": "world",
            "tf_source_cadence_child_frame": "uav1/fcu",
        })
        managers = [
            node for node in self.launch.findall("node")
            if node.attrib.get("pkg") == "mid360_multi_uav_sim"
            and node.attrib.get("type") == "scenario_manager.py"
        ]
        self.assertEqual(len(managers), 1)
        self.assertEqual(
            managers[0].find("param[@name='scenario_file']").attrib["value"],
            "$(arg scenario_config)",
        )
        launch_text = self.launch_path.read_text(encoding="utf-8").lower()
        self.assertNotIn("apriori", launch_text)
        self.assertNotIn("map_loader", launch_text)

    def test_new_assets_have_no_absolute_filesystem_paths(self):
        for path in (self.scenario_path, self.launch_path, self.world_path):
            text = path.read_text(encoding="utf-8")
            for prefix in ("/home/", "/root/", "/tmp/", "/opt/", "/usr/"):
                self.assertNotIn(prefix, text, "{} contains {}".format(path, prefix))


if __name__ == "__main__":
    unittest.main()
