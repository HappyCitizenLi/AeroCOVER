#!/usr/bin/env python3
"""Strict pure XML/YAML/launch contracts for the S05 clutter fixture."""

import math
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
PRIMITIVES = {"box", "cylinder", "sphere", "plane"}


def _pose(value):
    return tuple(float(value[key]) for key in ("x", "y", "z", "yaw"))


def _spawn_args(value):
    return dict(re.findall(
        r"(-(?:file|model|x|y|z|Y))\s+(\$\(find [^)]+\)/\S+|\S+)",
        value,
    ))


class S05AssetsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scenario_path = PACKAGE / "config/scenarios/S05.yaml"
        cls.world_path = PACKAGE / "worlds/E3_cluttered.world"
        cls.launch_path = PACKAGE / "launch/gate_s05.launch"
        cls.scenario = yaml.safe_load(cls.scenario_path.read_text(encoding="utf-8"))
        cls.world = ET.parse(cls.world_path).getroot().find("world")
        cls.launch = ET.parse(cls.launch_path).getroot()

    def test_frozen_schema_v5_and_runtime_contract(self):
        scenario = self.scenario
        self.assertEqual(scenario["schema_version"], 5)
        self.assertEqual(scenario["scenario_profile"], "s05")
        self.assertEqual(
            scenario["semantic_contract_id"],
            "mid360-multi-uav-s05-cluttered-stress-v1",
        )
        self.assertEqual(scenario["scenario"], "S05_cluttered_five_target_stress")
        self.assertEqual(scenario["world_frame"], "world")
        self.assertEqual(scenario["seed"], 808)
        self.assertEqual((scenario["repeat_count"], scenario["repeat_mode"]),
                         (1, "restart"))
        self.assertEqual(
            (float(scenario["start_delay"]), float(scenario["duration"]),
             float(scenario["final_hold"])),
            (5.0, 24.0, 3.0),
        )
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
        self.assertEqual(scenario["randomization"]["position_offset_uniform"],
                         {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0})

    def test_one_observer_and_exactly_five_stationary_targets(self):
        expected = {
            "uav1": (-4.0, -5.0, 2.0, -0.35),
            "uav2": (7.0, -4.0, 2.25, 0.0),
            "uav3": (10.0, -1.0, 2.25, 0.0),
            "uav4": (12.0, 3.0, 2.25, 0.0),
            "uav5": (7.0, 5.0, 2.25, 0.0),
            "uav6": (15.0, 0.8, 2.25, 0.0),
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
            child = "uav1/fcu" if name == "uav1" else name + "/base_link"
            self.assertEqual(target["child_frame_id"], child)
            self.assertEqual(_pose(target["initial_pose"]), expected_pose)

        motion = self.scenario["observer_motion"]
        self.assertEqual(motion, {
            "observer": "uav1",
            "parent_frame": "world",
            "child_frame": "uav1/fcu",
            "tf_authority": "scenario_manager",
            "unique_dynamic_tf_authority": True,
            "minimum_translation_m": 5.0,
            "minimum_yaw_change_rad": 0.7,
            "scored_phase": "peak_stress_scored",
            "stationary_targets": ["uav2", "uav3", "uav4", "uav5", "uav6"],
        })

    def test_phase_sequence_and_peak_motion_are_exact(self):
        phases = self.scenario["phases"]
        self.assertEqual(
            [(item["id"], item.get("event"), float(item["duration"]),
              item["interpolation"]) for item in phases],
            [
                ("initial_stress_hold", None, 4.0, "hold"),
                ("enter_clutter", "initial_stress_end", 5.0, "linear"),
                ("peak_stress_scored", "stress_window_start", 8.0, "linear"),
                ("exit_clutter", "stress_window_end", 5.0, "linear"),
                ("final_stress_hold", "final_stress_start", 2.0, "hold"),
            ],
        )
        self.assertEqual(sum(float(item["duration"]) for item in phases), 24.0)
        expected_goals = {
            "enter_clutter": (-1.0, -2.0, 2.0, 0.0),
            "peak_stress_scored": (3.0, 2.0, 2.0, 0.75),
            "exit_clutter": (5.0, -3.0, 2.0, -0.4),
        }
        for phase in phases:
            if phase["id"] in expected_goals:
                self.assertEqual(set(phase["targets"]), {"uav1"})
                self.assertEqual(_pose(phase["targets"]["uav1"]),
                                 expected_goals[phase["id"]])
            else:
                self.assertNotIn("targets", phase)

        scored_start = expected_goals["enter_clutter"]
        scored_end = expected_goals["peak_stress_scored"]
        translation = math.sqrt(sum(
            (scored_end[index] - scored_start[index]) ** 2 for index in range(3)
        ))
        yaw_change = abs(math.atan2(
            math.sin(scored_end[3] - scored_start[3]),
            math.cos(scored_end[3] - scored_start[3]),
        ))
        self.assertGreaterEqual(translation, 5.0)
        self.assertGreaterEqual(yaw_change, 0.7)
        for phase in phases:
            self.assertTrue(
                set(phase.get("targets", {})).isdisjoint(
                    self.scenario["observer_motion"]["stationary_targets"]
                )
            )

    def test_gate_frame_is_static_three_box_collision_model(self):
        config = ET.parse(PACKAGE / "models/gate_frame/model.config").getroot()
        self.assertEqual(config.findtext("name"), "Cluttered Arena Gate Frame")
        self.assertEqual(config.findtext("sdf"), "model.sdf")
        model = ET.parse(PACKAGE / "models/gate_frame/model.sdf").getroot().find("model")
        self.assertEqual(model.attrib["name"], "gate_frame")
        self.assertEqual(model.findtext("static"), "true")
        collisions = model.findall(".//collision")
        self.assertEqual(
            {item.attrib["name"] for item in collisions},
            {"left_post_collision", "right_post_collision", "top_beam_collision"},
        )
        self.assertEqual(len(collisions), 3)
        for collision in collisions:
            geometry = collision.find("geometry")
            self.assertEqual([child.tag for child in geometry], ["box"])
            size = tuple(float(value) for value in geometry.findtext("box/size").split())
            self.assertEqual(len(size), 3)
            self.assertTrue(all(value > 0.0 for value in size))
        self.assertEqual(len(model.findall(".//mesh")), 0)

    def test_e3_has_unique_model_includes_and_fourteen_static_collisions(self):
        self.assertEqual(self.world.attrib["name"], "E3_cluttered")
        includes = self.world.findall("include")
        names = [item.findtext("name") for item in includes]
        uris = [item.findtext("uri") for item in includes]
        self.assertEqual(len(names), len(set(names)))
        self.assertTrue(all(name for name in names))
        self.assertTrue(all(uri.startswith("model://") for uri in uris))
        self.assertEqual(uris.count("model://mid360_open_ground"), 1)
        self.assertEqual(uris.count("model://gate_frame"), 2)
        self.assertEqual(uris.count("model://pillar"), 4)
        self.assertEqual(uris.count("model://sparse_wide_wall"), 3)
        self.assertEqual(len(includes), 10)

        model_dirs = {
            "model://mid360_open_ground": "mid360_open_ground",
            "model://gate_frame": "gate_frame",
            "model://pillar": "pillar",
            "model://sparse_wide_wall": "sparse_wide_wall",
        }
        collision_count = 0
        for uri in uris:
            model = ET.parse(
                PACKAGE / "models" / model_dirs[uri] / "model.sdf"
            ).getroot().find("model")
            self.assertEqual(model.findtext("static"), "true")
            collisions = model.findall(".//collision")
            self.assertGreater(len(collisions), 0)
            collision_count += len(collisions)
            for collision in collisions:
                tags = [child.tag for child in collision.find("geometry")]
                self.assertEqual(len(tags), 1)
                self.assertIn(tags[0], PRIMITIVES)
        self.assertGreaterEqual(collision_count, 14)

        forbidden = {"simplified_target_uav", "observer_uav_dynamic"}
        self.assertTrue(forbidden.isdisjoint(uri[len("model://"):] for uri in uris))
        self.assertTrue(set("uav{}".format(i) for i in range(1, 7)).isdisjoint(names))

    def test_launch_spawns_one_sensor_observer_and_five_sensor_free_targets(self):
        scenario_arg = self.launch.find("arg[@name='scenario_config']")
        self.assertTrue(scenario_arg.attrib["default"].endswith(
            "/config/scenarios/S05.yaml"
        ))
        world_arg = self.launch.find("include/arg[@name='world_name']")
        self.assertTrue(world_arg.attrib["value"].endswith(
            "/worlds/E3_cluttered.world"
        ))

        spawns = {
            node.attrib["name"]: _spawn_args(node.attrib["args"])
            for node in self.launch.findall("node")
            if node.attrib.get("pkg") == "gazebo_ros"
            and node.attrib.get("type") == "spawn_model"
        }
        self.assertEqual(set(spawns), {
            "spawn_uav1_observer", "spawn_uav2_target", "spawn_uav3_target",
            "spawn_uav4_target", "spawn_uav5_target", "spawn_uav6_target",
        })
        expected = {
            "uav1": (-4.0, -5.0, 2.0, -0.35),
            "uav2": (7.0, -4.0, 2.25, None),
            "uav3": (10.0, -1.0, 2.25, None),
            "uav4": (12.0, 3.0, 2.25, None),
            "uav5": (7.0, 5.0, 2.25, None),
            "uav6": (15.0, 0.8, 2.25, None),
        }
        for name, expected_pose in expected.items():
            key = "spawn_{}_{}".format(name, "observer" if name == "uav1" else "target")
            args = spawns[key]
            self.assertEqual(args["-model"], name)
            self.assertEqual(
                tuple(float(args[option]) for option in ("-x", "-y", "-z")),
                expected_pose[:3],
            )
            if expected_pose[3] is None:
                self.assertNotIn("-Y", args)
                self.assertTrue(args["-file"].endswith(
                    "/models/simplified_target_uav/model.sdf"
                ))
            else:
                self.assertEqual(float(args["-Y"]), expected_pose[3])
                self.assertTrue(args["-file"].endswith(
                    "/models/observer_uav_dynamic/model.sdf"
                ))

        observer = ET.parse(
            PACKAGE / "models/observer_uav_dynamic/model.sdf"
        ).getroot().find("model")
        target = ET.parse(
            PACKAGE / "models/simplified_target_uav/model.sdf"
        ).getroot().find("model")
        self.assertEqual(len(observer.findall(".//sensor")), 1)
        self.assertEqual(
            observer.findtext(".//plugin/per_ray_pose_static_scene_opt_in"), "true"
        )
        self.assertEqual(len(target.findall(".//sensor")), 0)
        self.assertEqual(len(target.findall(".//plugin")), 0)

    def test_launch_has_one_tf_authority_per_ray_pose_and_s05_manager(self):
        static_tf = [
            node for node in self.launch.findall("node")
            if node.attrib.get("pkg") == "tf2_ros"
            and node.attrib.get("type") == "static_transform_publisher"
        ]
        self.assertEqual(len(static_tf), 1)
        self.assertEqual(static_tf[0].attrib["name"], "uav1_fcu_to_mid360")
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
        self.assertEqual(managers[0].attrib["name"], "scenario_manager")
        self.assertEqual(
            managers[0].find("param[@name='scenario_file']").attrib["value"],
            "$(arg scenario_config)",
        )

    def test_assets_have_no_hard_coded_absolute_filesystem_paths(self):
        paths = [
            PACKAGE / "models/gate_frame/model.config",
            PACKAGE / "models/gate_frame/model.sdf",
            self.world_path,
            self.scenario_path,
            self.launch_path,
        ]
        for path in paths:
            text = path.read_text(encoding="utf-8")
            for prefix in ("/home/", "/root/", "/tmp/", "/opt/", "/usr/"):
                self.assertNotIn(prefix, text, "{} contains {}".format(path, prefix))


if __name__ == "__main__":
    unittest.main()
