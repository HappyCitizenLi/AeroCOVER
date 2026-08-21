#!/usr/bin/env python3
"""Pure schema-v4 S04 authority and motion-contract tests."""

import copy
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / "scripts" / "scenario_manager.py"
MODULE_SPEC = importlib.util.spec_from_file_location(
    "mid360_scenario_manager_s04_contract", SCRIPT
)
scenario_manager = importlib.util.module_from_spec(MODULE_SPEC)
sys.modules[MODULE_SPEC.name] = scenario_manager
MODULE_SPEC.loader.exec_module(scenario_manager)


class ScenarioManagerS04Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path = PACKAGE / "config" / "scenarios" / "S04.yaml"
        cls.document = yaml.safe_load(cls.path.read_text(encoding="utf-8"))

    def load_mutated(self, mutator, **overrides):
        document = copy.deepcopy(self.document)
        mutator(document)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "scenario.yaml"
            path.write_text(
                yaml.safe_dump(document, sort_keys=False), encoding="utf-8"
            )
            return scenario_manager.load_scenario(str(path), **overrides)

    def assert_invalid(self, mutator, pattern, **overrides):
        with self.assertRaisesRegex(scenario_manager.ScenarioError, pattern):
            self.load_mutated(mutator, **overrides)

    def test_exact_schema_v4_authority_loads(self):
        spec = scenario_manager.load_scenario(str(self.path))
        self.assertEqual(spec.schema_version, 4)
        self.assertEqual(spec.scenario_profile, "s04")
        self.assertEqual(
            spec.semantic_contract_id,
            scenario_manager.S04_SEMANTIC_CONTRACT_ID,
        )
        self.assertEqual(spec.scenario_id, scenario_manager.S04_SCENARIO_ID)
        self.assertEqual(spec.seed, 707)
        self.assertEqual(spec.start_delay_ns, 5_000_000_000)
        self.assertEqual(spec.cycle_duration_ns, 20_000_000_000)
        self.assertEqual(spec.final_hold_ns, 3_000_000_000)
        self.assertEqual(set(spec.targets), {"uav1", "uav2", "uav3", "uav4"})
        self.assertTrue(all(target.command_model for target in spec.targets.values()))
        self.assertEqual(
            spec.targets["uav1"].initial_pose,
            scenario_manager.PoseSpec(4.0, -3.0, 2.0, -math.pi / 4.0),
        )
        self.assertEqual(
            spec.targets["uav2"].initial_pose,
            scenario_manager.PoseSpec(8.0, 24.0 / 13.0, 2.25, 0.0),
        )
        self.assertEqual(
            spec.targets["uav3"].initial_pose,
            scenario_manager.PoseSpec(13.0, 3.0, 2.25, 0.0),
        )
        self.assertEqual(
            spec.targets["uav4"].initial_pose,
            scenario_manager.PoseSpec(14.0, 0.0, 2.25, 0.0),
        )

        motion = spec.observer_motion
        self.assertIsNotNone(motion)
        self.assertIsNone(motion.fixed_yaw)
        self.assertEqual(motion.tf_authority, "scenario_manager")
        self.assertEqual(motion.minimum_translation_m, 2.0)
        self.assertEqual(motion.minimum_yaw_change_rad, 0.35)
        self.assertEqual(motion.scored_phase, "combined_visibility_scored")
        self.assertEqual(motion.stationary_targets, ("uav2", "uav3", "uav4"))

    def test_exact_five_phase_timeline_and_events(self):
        spec = scenario_manager.load_scenario(str(self.path))
        self.assertEqual(
            [phase.phase_id for phase in spec.phases],
            [
                "initial_clear_hold",
                "approach_combined_occlusion",
                "combined_visibility_scored",
                "leave_combined_visibility",
                "final_clear_hold",
            ],
        )
        self.assertEqual(
            [phase.duration_ns for phase in spec.phases],
            [3_000_000_000, 4_000_000_000, 5_000_000_000, 5_000_000_000, 3_000_000_000],
        )
        self.assertEqual(
            spec.phase_starts_ns,
            (0, 3_000_000_000, 7_000_000_000, 12_000_000_000, 17_000_000_000),
        )
        self.assertEqual(
            [phase.interpolation for phase in spec.phases],
            ["hold", "linear", "linear", "linear", "hold"],
        )
        self.assertEqual(
            [phase.event for phase in spec.phases],
            [
                None,
                "initial_clear_end",
                "combined_visibility_start",
                "combined_visibility_end",
                "final_clear_start",
            ],
        )
        self.assertEqual(
            spec.phases[2].goals["uav1"],
            scenario_manager.PoseSpec(-2.0, -6.0 / 13.0, 2.0, 0.4),
        )
        self.assertEqual(
            spec.phases[3].goals["uav1"],
            scenario_manager.PoseSpec(4.0, -3.0, 2.0, math.pi / 4.0),
        )

    def test_scored_phase_simultaneously_translates_and_yaws(self):
        spec = scenario_manager.load_scenario(str(self.path))
        manager = object.__new__(scenario_manager.ScenarioManager)
        manager.spec = spec
        scored_index = next(
            index
            for index, phase in enumerate(spec.phases)
            if phase.phase_id == spec.observer_motion.scored_phase
        )
        scored = spec.phases[scored_index]
        states = manager._sample_phase(scored_index, scored.duration_ns // 2, 1)
        observer = states["uav1"]
        self.assertAlmostEqual(observer.pose.x, -1.0)
        self.assertAlmostEqual(observer.pose.y, -3.0 / 13.0)
        self.assertAlmostEqual(observer.pose.yaw, 0.2)
        self.assertAlmostEqual(observer.vx, -0.4)
        self.assertAlmostEqual(observer.vy, -6.0 / 65.0)
        self.assertAlmostEqual(observer.yaw_rate, 0.08)
        self.assertGreater(math.hypot(observer.vx, observer.vy), 0.0)
        self.assertGreater(abs(observer.yaw_rate), 0.0)
        for name in ("uav2", "uav3", "uav4"):
            self.assertEqual(states[name].pose, spec.targets[name].initial_pose)
            self.assertEqual(
                (states[name].vx, states[name].vy, states[name].vz, states[name].yaw_rate),
                (0.0, 0.0, 0.0, 0.0),
            )

    def test_schema_v4_rejects_unknown_keys_at_every_layer(self):
        mutations = (
            (lambda value: value.__setitem__("extra", 1), "scenario"),
            (lambda value: value["runtime"].__setitem__("extra", 1), "runtime"),
            (
                lambda value: value["targets"]["uav4"].__setitem__("extra", 1),
                "targets.uav4",
            ),
            (
                lambda value: value["observer_motion"].__setitem__("extra", 1),
                "observer_motion",
            ),
            (
                lambda value: value["phases"][2].__setitem__("extra", 1),
                r"phases\[2\]",
            ),
        )
        for mutation, pattern in mutations:
            with self.subTest(pattern=pattern):
                self.assert_invalid(mutation, pattern)

    def test_profile_identity_and_four_target_contract_fail_closed(self):
        mutations = (
            (
                lambda value: value.__setitem__("scenario_profile", "s06"),
                "scenario_profile",
            ),
            (
                lambda value: value.__setitem__("semantic_contract_id", "wrong"),
                "semantic_contract_id",
            ),
            (
                lambda value: value.__setitem__("scenario", "S03_moving_observer_translation"),
                "schema-v4 scenario",
            ),
            (lambda value: value["targets"].pop("uav4"), "stationary_targets"),
            (
                lambda value: value["targets"]["uav1"].__setitem__("command_model", False),
                "command_model: true",
            ),
            (
                lambda value: value["targets"]["uav2"]["initial_pose"].__setitem__("y", 1.8),
                "target 'uav2'",
            ),
            (
                lambda value: value["targets"]["uav4"].__setitem__("child_frame_id", "uav3/base_link"),
                "values must be unique",
            ),
        )
        for mutation, pattern in mutations:
            with self.subTest(pattern=pattern):
                self.assert_invalid(mutation, pattern)

    def test_motion_thresholds_and_stationary_targets_fail_closed(self):
        mutations = (
            (
                lambda value: value["observer_motion"].__setitem__("minimum_translation_m", 1.99),
                "observer_motion contract differs",
            ),
            (
                lambda value: value["observer_motion"].__setitem__("minimum_yaw_change_rad", 0.34),
                "observer_motion contract differs",
            ),
            (
                lambda value: value["observer_motion"].__setitem__("minimum_translation_m", 2.01),
                "observer_motion contract differs",
            ),
            (
                lambda value: value["observer_motion"].__setitem__("minimum_yaw_change_rad", 0.36),
                "observer_motion contract differs",
            ),
            (
                lambda value: value["observer_motion"].__setitem__("minimum_translation_m", 2.1),
                "translation",
            ),
            (
                lambda value: value["observer_motion"].__setitem__("minimum_yaw_change_rad", 0.41),
                "yaw change",
            ),
            (
                lambda value: value["observer_motion"].__setitem__("stationary_targets", ["uav2", "uav3"]),
                "every non-observer target",
            ),
        )
        for mutation, pattern in mutations:
            with self.subTest(pattern=pattern):
                self.assert_invalid(mutation, pattern)

        def move_target_in_scored_phase(value):
            value["phases"][2].setdefault("targets", {})["uav2"] = {
                "x": 8.1,
                "y": 24.0 / 13.0,
                "z": 2.25,
                "yaw": 0.0,
            }

        self.assert_invalid(move_target_in_scored_phase, "moves stationary targets")

    def test_exact_phase_endpoints_events_and_timeline_fail_closed(self):
        mutations = (
            (
                lambda value: value["phases"][1].__setitem__("event", "wrong"),
                "phase 1 contract differs",
            ),
            (
                lambda value: value["phases"][2].__setitem__("interpolation", "smoothstep"),
                "phase 2 contract differs",
            ),
            (
                lambda value: value["phases"][3]["targets"]["uav1"].__setitem__("yaw", 0.7),
                "phase 3 contract differs",
            ),
            (
                lambda value: value["phases"][4].__setitem__("event", "wrong"),
                "phase 4 contract differs",
            ),
            (
                lambda value: value.__setitem__("start_delay", 4.9),
                "timeline contract differs",
            ),
            (
                lambda value: value["runtime"].__setitem__("publish_rate", 40.0),
                "runtime authority contract differs",
            ),
        )
        for mutation, pattern in mutations:
            with self.subTest(pattern=pattern):
                self.assert_invalid(mutation, pattern)

        self.assert_invalid(
            lambda value: None,
            "timeline contract differs",
            duration_override=21.0,
        )

    def test_dynamic_tf_uses_world_to_observer_pose_with_yaw(self):
        spec = scenario_manager.load_scenario(str(self.path))
        state = scenario_manager.MotionState(
            scenario_manager.PoseSpec(-1.0, -3.0 / 13.0, 2.0, 0.2),
            vx=-0.4,
            vy=-6.0 / 65.0,
            yaw_rate=0.08,
        )
        transform = scenario_manager.build_dynamic_observer_transform(
            spec.observer_motion, state, 12_345_678_901
        )
        self.assertEqual(transform.header.frame_id, "world")
        self.assertEqual(transform.child_frame_id, "uav1/fcu")
        self.assertAlmostEqual(transform.transform.translation.x, -1.0)
        self.assertAlmostEqual(transform.transform.translation.y, -3.0 / 13.0)
        self.assertAlmostEqual(transform.transform.rotation.z, math.sin(0.1))
        self.assertAlmostEqual(transform.transform.rotation.w, math.cos(0.1))
        with mock.patch.object(
            scenario_manager.tf2_ros, "TransformBroadcaster"
        ) as constructor:
            marker = object()
            constructor.return_value = marker
            self.assertIs(
                scenario_manager.create_dynamic_tf_broadcaster(spec.observer_motion),
                marker,
            )
            constructor.assert_called_once_with()

    def test_schema_v1_v2_load_paths_remain_unchanged(self):
        expected = (
            ("S01.yaml", 1, None, None),
            ("S03.yaml", 2, "s03", 0.0),
        )
        for filename, version, profile, fixed_yaw in expected:
            with self.subTest(filename=filename):
                spec = scenario_manager.load_scenario(
                    str(PACKAGE / "config" / "scenarios" / filename)
                )
                self.assertEqual(spec.schema_version, version)
                self.assertEqual(spec.scenario_profile, profile)
                if version == 2:
                    self.assertEqual(spec.observer_motion.fixed_yaw, fixed_yaw)
                    self.assertEqual(spec.observer_motion.minimum_yaw_change_rad, 0.0)
                else:
                    self.assertIsNone(spec.observer_motion)


if __name__ == "__main__":
    unittest.main()
