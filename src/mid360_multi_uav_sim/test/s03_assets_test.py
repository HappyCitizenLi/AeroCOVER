#!/usr/bin/env python3
"""Pure XML/YAML/launch contracts for the S03 moving-observer asset line."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def _words(element, path):
    text = element.findtext(path)
    if text is None:
        raise AssertionError("missing XML value at {}".format(path))
    return tuple(float(value) for value in text.split())


def _pose(value):
    return tuple(float(value[key]) for key in ("x", "y", "z", "yaw"))


class S03AssetsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scenario_path = PACKAGE / "config" / "scenarios" / "S03.yaml"
        cls.scenario = yaml.safe_load(cls.scenario_path.read_text(encoding="utf-8"))

    def _model(self, name):
        return ET.parse(PACKAGE / "models" / name / "model.sdf").getroot().find(
            "model"
        )

    def test_schema_v2_contract_is_explicit_and_complete(self):
        scenario = self.scenario
        self.assertEqual(scenario["schema_version"], 2)
        self.assertEqual(scenario["scenario_profile"], "s03")
        self.assertEqual(
            scenario["semantic_contract_id"],
            "mid360-multi-uav-s03-observer-motion-v1",
        )
        self.assertEqual(scenario["scenario"], "S03_moving_observer_translation")
        self.assertEqual(scenario["repeat_count"], 1)
        self.assertEqual(scenario["repeat_mode"], "restart")
        self.assertEqual(
            set(scenario["observer_motion"]),
            {
                "observer",
                "parent_frame",
                "child_frame",
                "tf_authority",
                "unique_dynamic_tf_authority",
                "fixed_yaw",
                "minimum_translation_m",
                "scored_phase",
                "stationary_targets",
            },
        )
        motion = scenario["observer_motion"]
        self.assertEqual(motion["observer"], "uav1")
        self.assertEqual((motion["parent_frame"], motion["child_frame"]),
                         ("world", "uav1/fcu"))
        self.assertEqual(motion["tf_authority"], "scenario_manager")
        self.assertIs(motion["unique_dynamic_tf_authority"], True)
        self.assertEqual(float(motion["fixed_yaw"]), 0.0)
        self.assertGreater(float(motion["minimum_translation_m"]), 0.0)
        self.assertEqual(motion["scored_phase"], "observer_translation_scored")
        self.assertEqual(motion["stationary_targets"], ["uav2", "uav3"])

    def test_scored_window_has_real_translation_fixed_yaw_and_static_targets(self):
        scenario = self.scenario
        poses = {
            name: _pose(value["initial_pose"])
            for name, value in scenario["targets"].items()
        }
        scored_start = None
        scored_end = None
        for phase in scenario["phases"]:
            before = dict(poses)
            for name, value in phase.get("targets", {}).items():
                poses[name] = _pose(value)
            if phase["id"] == scenario["observer_motion"]["scored_phase"]:
                scored_start = before
                scored_end = dict(poses)
                self.assertEqual(phase["interpolation"], "linear")
                self.assertEqual(phase["event"], "observer_motion_scored_start")
        self.assertIsNotNone(scored_start)
        self.assertIsNotNone(scored_end)
        observer_delta = tuple(
            scored_end["uav1"][index] - scored_start["uav1"][index]
            for index in range(3)
        )
        self.assertEqual(observer_delta, (4.0, 0.0, 0.0))
        self.assertGreater(sum(value * value for value in observer_delta), 0.0)
        self.assertEqual(scored_start["uav1"][3], scored_end["uav1"][3])
        for name in ("uav2", "uav3"):
            self.assertEqual(scored_start[name], scored_end[name])

    def test_dynamic_observer_is_independent_and_explicitly_opts_in(self):
        static_observer = self._model("observer_uav")
        dynamic_observer = self._model("observer_uav_dynamic")
        self.assertEqual(static_observer.findtext("static"), "true")
        self.assertEqual(dynamic_observer.findtext("static"), "false")
        self.assertEqual(dynamic_observer.findtext("allow_auto_disable"), "false")
        self.assertEqual(dynamic_observer.findtext("link[@name='base_link']/gravity"), "false")
        self.assertEqual(len(dynamic_observer.findall(".//sensor")), 1)
        self.assertEqual(len(dynamic_observer.findall(".//plugin")), 1)
        plugin = dynamic_observer.find(".//plugin[@name='uav1_mid360_plugin']")
        self.assertIsNotNone(plugin)
        self.assertEqual(plugin.findtext("parentFrameName"), "uav1/fcu")
        self.assertEqual(plugin.findtext("frameName"), "uav1/mid360_link")
        self.assertEqual(plugin.findtext("ray_time_geometry_mode"), "per_ray_pose")
        self.assertEqual(plugin.findtext("per_ray_pose_static_scene_opt_in"), "true")
        self.assertIsNone(plugin.find("per_ray_pose_static_scene_only"))
        joint = dynamic_observer.find("joint[@name='mid360_fixed_joint']")
        self.assertEqual(joint.attrib.get("type"), "fixed")
        self.assertEqual(joint.findtext("parent"), "base_link")
        self.assertEqual(joint.findtext("child"), "mid360_link")

        target = self._model("simplified_target_uav")
        self.assertEqual(len(target.findall(".//sensor")), 0)
        self.assertEqual(len(target.findall(".//plugin")), 0)

    def test_sparse_world_and_wide_wall_use_local_primitive_assets(self):
        wall = self._model("sparse_wide_wall")
        self.assertEqual(wall.attrib.get("name"), "sparse_wide_wall")
        self.assertEqual(wall.findtext("static"), "true")
        collision = wall.find("link/collision[@name='wall_collision']")
        visual = wall.find("link/visual[@name='wall_visual']")
        self.assertIsNotNone(collision)
        self.assertIsNotNone(visual)
        self.assertEqual(_words(collision, "geometry/box/size"), (0.5, 30.0, 8.0))
        self.assertEqual(_words(visual, "geometry/box/size"), (0.5, 30.0, 8.0))

        world_path = PACKAGE / "worlds" / "E1_sparse.world"
        world = ET.parse(world_path).getroot().find("world")
        self.assertEqual(world.attrib.get("name"), "E1_sparse")
        includes = {value.findtext("uri"): value for value in world.findall("include")}
        self.assertEqual(
            set(includes),
            {"model://mid360_open_ground", "model://sparse_wide_wall"},
        )
        self.assertEqual(
            _words(includes["model://sparse_wide_wall"], "pose"),
            (25.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )
        self.assertNotIn("/home/", world_path.read_text(encoding="utf-8"))

    def test_launch_has_one_dynamic_tf_authority_and_per_ray_preprocessor(self):
        launch = ET.parse(PACKAGE / "launch" / "gate_s03.launch").getroot()
        scenario = launch.find("arg[@name='scenario_config']")
        self.assertTrue(scenario.attrib["default"].endswith("/config/scenarios/S03.yaml"))
        world = launch.find("include/arg[@name='world_name']")
        self.assertTrue(world.attrib["value"].endswith("/worlds/E1_sparse.world"))

        spawns = {
            node.attrib["name"]: node.attrib["args"]
            for node in launch.findall("node")
            if node.attrib.get("pkg") == "gazebo_ros"
            and node.attrib.get("type") == "spawn_model"
        }
        self.assertEqual(set(spawns), {
            "spawn_uav1_observer", "spawn_uav2_target", "spawn_uav3_target"
        })
        self.assertIn("/models/observer_uav_dynamic/model.sdf", spawns["spawn_uav1_observer"])
        self.assertIn("-model uav1", spawns["spawn_uav1_observer"])

        static_nodes = [
            node
            for node in launch.findall("node")
            if node.attrib.get("pkg") == "tf2_ros"
            and node.attrib.get("type") == "static_transform_publisher"
        ]
        self.assertEqual(len(static_nodes), 1)
        self.assertEqual(static_nodes[0].attrib.get("name"), "uav1_fcu_to_mid360")
        self.assertIn("uav1/fcu uav1/mid360_link", static_nodes[0].attrib["args"])
        self.assertNotIn("world uav1/fcu", static_nodes[0].attrib["args"])
        self.assertIsNone(launch.find("node[@name='world_to_uav1_fcu']"))

        preprocess_include = next(
            item for item in launch.findall("include")
            if "mid360_ray_preprocessor" in item.attrib.get("file", "")
        )
        args = {item.attrib["name"]: item.attrib["value"]
                for item in preprocess_include.findall("arg")}
        self.assertEqual(args["ray_time_geometry_mode"], "per_ray_pose")
        self.assertEqual(args["output_frame"], "world")

        managers = [
            node for node in launch.findall("node")
            if node.attrib.get("pkg") == "mid360_multi_uav_sim"
            and node.attrib.get("type") == "scenario_manager.py"
        ]
        self.assertEqual(len(managers), 1)
        self.assertEqual(managers[0].attrib.get("name"), "scenario_manager")


if __name__ == "__main__":
    unittest.main()
