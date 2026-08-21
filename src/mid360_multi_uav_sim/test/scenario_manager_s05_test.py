#!/usr/bin/env python3
"""Pure schema-v5 S05 authority, boundary and fail-closed tests."""

import copy
import importlib.util
import json
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
    "mid360_scenario_manager_s05_contract", SCRIPT
)
scenario_manager = importlib.util.module_from_spec(MODULE_SPEC)
sys.modules[MODULE_SPEC.name] = scenario_manager
MODULE_SPEC.loader.exec_module(scenario_manager)


class ScenarioManagerS05Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path = PACKAGE / "config" / "scenarios" / "S05.yaml"
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

    def test_exact_schema_v5_authority_loads(self):
        spec = scenario_manager.load_scenario(str(self.path))
        self.assertEqual(spec.schema_version, 5)
        self.assertEqual(spec.scenario_profile, "s05")
        self.assertEqual(
            spec.semantic_contract_id,
            "mid360-multi-uav-s05-cluttered-stress-v1",
        )
        self.assertEqual(spec.scenario_id, "S05_cluttered_five_target_stress")
        self.assertEqual(spec.seed, 808)
        self.assertEqual(spec.world_frame, "world")
        self.assertEqual(spec.repeat_count, 1)
        self.assertEqual(spec.repeat_mode, "restart")
        self.assertEqual(spec.start_delay_ns, 5_000_000_000)
        self.assertEqual(spec.cycle_duration_ns, 24_000_000_000)
        self.assertEqual(spec.final_hold_ns, 3_000_000_000)
        self.assertEqual(
            set(spec.targets),
            {"uav1", "uav2", "uav3", "uav4", "uav5", "uav6"},
        )
        self.assertTrue(all(target.command_model for target in spec.targets.values()))
        self.assertEqual(
            spec.targets["uav1"].initial_pose,
            scenario_manager.PoseSpec(-4.0, -5.0, 2.0, -0.35),
        )
        expected_targets = {
            "uav2": (7.0, -4.0),
            "uav3": (10.0, -1.0),
            "uav4": (12.0, 3.0),
            "uav5": (7.0, 5.0),
            "uav6": (15.0, 0.8),
        }
        for name, (x, y) in expected_targets.items():
            self.assertEqual(
                spec.targets[name].initial_pose,
                scenario_manager.PoseSpec(x, y, 2.25, 0.0),
            )

        motion = spec.observer_motion
        self.assertIsNotNone(motion)
        self.assertEqual(motion.observer, "uav1")
        self.assertEqual(motion.parent_frame, "world")
        self.assertEqual(motion.child_frame, "uav1/fcu")
        self.assertEqual(motion.tf_authority, "scenario_manager")
        self.assertTrue(motion.unique_dynamic_tf_authority)
        self.assertIsNone(motion.fixed_yaw)
        self.assertEqual(motion.minimum_translation_m, 5.0)
        self.assertEqual(motion.minimum_yaw_change_rad, 0.7)
        self.assertEqual(motion.scored_phase, "peak_stress_scored")
        self.assertEqual(
            motion.stationary_targets,
            ("uav2", "uav3", "uav4", "uav5", "uav6"),
        )

    def test_exact_phase_timeline_events_and_endpoints(self):
        spec = scenario_manager.load_scenario(str(self.path))
        self.assertEqual(
            [phase.phase_id for phase in spec.phases],
            [
                "initial_stress_hold",
                "enter_clutter",
                "peak_stress_scored",
                "exit_clutter",
                "final_stress_hold",
            ],
        )
        self.assertEqual(
            [phase.duration_ns for phase in spec.phases],
            [
                4_000_000_000,
                5_000_000_000,
                8_000_000_000,
                5_000_000_000,
                2_000_000_000,
            ],
        )
        self.assertEqual(
            spec.phase_starts_ns,
            (0, 4_000_000_000, 9_000_000_000, 17_000_000_000, 22_000_000_000),
        )
        self.assertEqual(
            [phase.interpolation for phase in spec.phases],
            ["hold", "linear", "linear", "linear", "hold"],
        )
        self.assertEqual(
            [phase.event for phase in spec.phases],
            [
                None,
                "initial_stress_end",
                "stress_window_start",
                "stress_window_end",
                "final_stress_start",
            ],
        )
        expected_observer_goals = (
            scenario_manager.PoseSpec(-4.0, -5.0, 2.0, -0.35),
            scenario_manager.PoseSpec(-1.0, -2.0, 2.0, 0.0),
            scenario_manager.PoseSpec(3.0, 2.0, 2.0, 0.75),
            scenario_manager.PoseSpec(5.0, -3.0, 2.0, -0.4),
            scenario_manager.PoseSpec(5.0, -3.0, 2.0, -0.4),
        )
        self.assertEqual(
            tuple(phase.goals["uav1"] for phase in spec.phases),
            expected_observer_goals,
        )

    def test_motion_sampling_is_continuous_and_all_five_targets_are_stationary(self):
        spec = scenario_manager.load_scenario(str(self.path))
        manager = object.__new__(scenario_manager.ScenarioManager)
        manager.spec = spec
        stationary = ("uav2", "uav3", "uav4", "uav5", "uav6")

        expected_midpoints = {
            0: (-4.0, -5.0, -0.35, 0.0, 0.0, 0.0),
            1: (-2.5, -3.5, -0.175, 0.6, 0.6, 0.07),
            2: (1.0, 0.0, 0.375, 0.5, 0.5, 0.09375),
            3: (4.0, -0.5, 0.175, 0.4, -1.0, -0.23),
            4: (5.0, -3.0, -0.4, 0.0, 0.0, 0.0),
        }
        for index, expected in expected_midpoints.items():
            with self.subTest(phase=index):
                phase = spec.phases[index]
                states = manager._sample_phase(index, phase.duration_ns // 2, 1)
                observer = states["uav1"]
                actual = (
                    observer.pose.x,
                    observer.pose.y,
                    observer.pose.yaw,
                    observer.vx,
                    observer.vy,
                    observer.yaw_rate,
                )
                for actual_value, expected_value in zip(actual, expected):
                    self.assertAlmostEqual(actual_value, expected_value)
                for name in stationary:
                    self.assertEqual(states[name].pose, spec.targets[name].initial_pose)
                    self.assertEqual(
                        (
                            states[name].vx,
                            states[name].vy,
                            states[name].vz,
                            states[name].yaw_rate,
                        ),
                        (0.0, 0.0, 0.0, 0.0),
                    )

        for index in range(len(spec.phases) - 1):
            with self.subTest(boundary=index):
                endpoint = manager._sample_phase(
                    index, spec.phases[index].duration_ns, 1
                )
                next_start = manager._sample_phase(index + 1, 0, 1)
                for name in spec.targets:
                    for endpoint_value, start_value in zip(
                        (
                            endpoint[name].pose.x,
                            endpoint[name].pose.y,
                            endpoint[name].pose.z,
                            endpoint[name].pose.yaw,
                        ),
                        (
                            next_start[name].pose.x,
                            next_start[name].pose.y,
                            next_start[name].pose.z,
                            next_start[name].pose.yaw,
                        ),
                    ):
                        self.assertAlmostEqual(endpoint_value, start_value)

        scored_start = spec.phases[1].goals["uav1"]
        scored_goal = spec.phases[2].goals["uav1"]
        self.assertGreaterEqual(
            scenario_manager._translation_distance(scored_start, scored_goal), 5.0
        )
        self.assertGreaterEqual(
            scenario_manager._yaw_distance(scored_start, scored_goal), 0.7
        )

    def test_event_payload_uses_the_generic_profile_and_motion_fields(self):
        spec = scenario_manager.load_scenario(str(self.path))
        manager = object.__new__(scenario_manager.ScenarioManager)
        manager.spec = spec
        manager.last_commanded_states = {
            name: scenario_manager.MotionState(spec.phases[2].goals[name])
            for name in spec.targets
        }
        manager.event_pub = mock.Mock()
        with mock.patch.object(scenario_manager.rospy, "loginfo"):
            manager._event("stress_window_start", 1, 14_000_000_000)
        payload = json.loads(manager.event_pub.publish.call_args.args[0].data)
        self.assertEqual(payload["scenario_profile"], "s05")
        self.assertEqual(
            payload["semantic_contract_id"],
            "mid360-multi-uav-s05-cluttered-stress-v1",
        )
        self.assertEqual(payload["dynamic_tf_authority"], "scenario_manager")
        self.assertEqual(set(payload["commanded_pose"]), set(spec.targets))
        self.assertNotIn("scenario_contract_id", payload)
        self.assertNotIn("observer_only", payload)

    def test_unknown_and_missing_keys_fail_closed_at_every_schema_layer(self):
        mutations = (
            (lambda value: value.__setitem__("extra", 1), "unknown keys"),
            (lambda value: value.pop("duration"), "missing keys"),
            (lambda value: value["runtime"].__setitem__("extra", 1), "unknown keys"),
            (
                lambda value: value["runtime"].pop("publish_rate"),
                "missing keys",
            ),
            (
                lambda value: value["targets"]["uav6"].__setitem__("extra", 1),
                "unknown keys",
            ),
            (
                lambda value: value["targets"]["uav6"].pop("initial_pose"),
                "missing keys",
            ),
            (
                lambda value: value["targets"]["uav6"]["initial_pose"].pop("yaw"),
                "missing keys",
            ),
            (
                lambda value: value["observer_motion"].__setitem__("extra", 1),
                "unknown keys",
            ),
            (
                lambda value: value["observer_motion"].pop("minimum_yaw_change_rad"),
                "missing keys",
            ),
            (
                lambda value: value["phases"][2].__setitem__("extra", 1),
                "unknown keys",
            ),
            (
                lambda value: value["phases"][2].pop("duration"),
                "missing keys",
            ),
        )
        for index, (mutation, pattern) in enumerate(mutations):
            with self.subTest(case=index):
                self.assert_invalid(mutation, pattern)

    def test_profile_runtime_randomization_and_timeline_mutations_fail_closed(self):
        mutations = (
            (lambda value: value.__setitem__("scenario_profile", "s04"), "scenario_profile"),
            (lambda value: value.__setitem__("semantic_contract_id", "wrong"), "semantic_contract_id"),
            (lambda value: value.__setitem__("scenario", "wrong"), "schema-v5 scenario"),
            (lambda value: value.__setitem__("seed", 807), "timeline contract"),
            (lambda value: value.__setitem__("world_frame", "map"), "parent_frame"),
            (lambda value: value.__setitem__("repeat_count", 2), "timeline contract"),
            (lambda value: value.__setitem__("repeat_mode", "ping_pong"), "timeline contract"),
            (lambda value: value.__setitem__("start_delay", 4.0), "timeline contract"),
            (lambda value: value.__setitem__("final_hold", 2.0), "timeline contract"),
            (lambda value: value["runtime"].__setitem__("publish_rate", 40.0), "runtime authority"),
            (
                lambda value: value["randomization"]["position_offset_uniform"].__setitem__("x", 0.1),
                "offsets must all be zero",
            ),
        )
        for index, (mutation, pattern) in enumerate(mutations):
            with self.subTest(case=index):
                self.assert_invalid(mutation, pattern)

    def test_missing_extra_and_forged_targets_fail_closed(self):
        def add_uav7(value):
            value["targets"]["uav7"] = copy.deepcopy(value["targets"]["uav6"])
            value["targets"]["uav7"]["model_name"] = "uav7"
            value["targets"]["uav7"]["truth_topic"] = (
                "/mid360_multi_uav_sim/ground_truth/uav7/odom"
            )
            value["targets"]["uav7"]["child_frame_id"] = "uav7/base_link"
            value["observer_motion"]["stationary_targets"].append("uav7")

        mutations = (
            (lambda value: value["targets"].pop("uav6"), "stationary_targets"),
            (add_uav7, "targets must be exactly"),
            (
                lambda value: value["targets"]["uav5"].__setitem__("model_name", "decoy"),
                "target 'uav5'",
            ),
            (
                lambda value: value["targets"]["uav5"].__setitem__(
                    "truth_topic", "/forged/uav5/odom"
                ),
                "target 'uav5'",
            ),
            (
                lambda value: value["targets"]["uav5"].__setitem__(
                    "child_frame_id", "forged/base_link"
                ),
                "target 'uav5'",
            ),
            (
                lambda value: value["targets"]["uav5"].__setitem__("command_model", False),
                "target 'uav5'",
            ),
            (
                lambda value: value["targets"]["uav5"]["initial_pose"].__setitem__("y", 5.1),
                "target 'uav5'",
            ),
        )
        for index, (mutation, pattern) in enumerate(mutations):
            with self.subTest(case=index):
                self.assert_invalid(mutation, pattern)

    def test_observer_and_trajectory_weakening_fail_closed(self):
        mutations = (
            (
                lambda value: value["observer_motion"].__setitem__("minimum_translation_m", 4.9),
                "observer_motion contract differs",
            ),
            (
                lambda value: value["observer_motion"].__setitem__("minimum_yaw_change_rad", 0.69),
                "observer_motion contract differs",
            ),
            (
                lambda value: value["observer_motion"].__setitem__("scored_phase", "enter_clutter"),
                "translation",
            ),
            (
                lambda value: value["observer_motion"].__setitem__(
                    "stationary_targets", ["uav2", "uav3", "uav4", "uav5"]
                ),
                "every non-observer target",
            ),
            (
                lambda value: value["observer_motion"].__setitem__("tf_authority", "forged"),
                "tf_authority",
            ),
            (
                lambda value: value["observer_motion"].__setitem__("unique_dynamic_tf_authority", False),
                "unique_dynamic_tf_authority",
            ),
            (
                lambda value: value["phases"][2]["targets"]["uav1"].__setitem__("x", 1.0),
                "translation",
            ),
            (
                lambda value: value["phases"][2]["targets"]["uav1"].__setitem__("yaw", 0.6),
                "yaw change",
            ),
            (
                lambda value: value["phases"][2].setdefault("targets", {}).__setitem__(
                    "uav2", {"x": 7.1, "y": -4.0, "z": 2.25, "yaw": 0.0}
                ),
                "moves stationary targets",
            ),
        )
        for index, (mutation, pattern) in enumerate(mutations):
            with self.subTest(case=index):
                self.assert_invalid(mutation, pattern)

    def test_phase_identity_events_and_exact_goals_fail_closed(self):
        mutations = (
            (lambda value: value["phases"][0].__setitem__("id", "hold"), "phase 0 contract"),
            (lambda value: value["phases"][1].__setitem__("duration", 4.0), "duration does not equal"),
            (lambda value: value["phases"][2].__setitem__("interpolation", "smoothstep"), "phase 2 contract"),
            (lambda value: value["phases"][1].__setitem__("event", "wrong"), "phase 1 contract"),
            (lambda value: value["phases"][2].pop("event"), "boundary event"),
            (lambda value: value["phases"][3]["targets"]["uav1"].__setitem__("y", -2.9), "phase 3 contract"),
            (lambda value: value["phases"][4].__setitem__("event", "wrong"), "phase 4 contract"),
        )
        for index, (mutation, pattern) in enumerate(mutations):
            with self.subTest(case=index):
                self.assert_invalid(mutation, pattern)

    def test_every_parameter_override_is_rejected_even_when_value_matches(self):
        overrides = (
            {"seed_override": 808},
            {"duration_override": 24.0},
            {"repeat_count_override": 1},
            {"final_hold_override": 3.0},
            {"start_delay_override": 5.0},
        )
        for override in overrides:
            with self.subTest(override=override):
                self.assert_invalid(
                    lambda value: None,
                    "does not permit parameter overrides",
                    **override,
                )

    def test_earlier_schema_load_paths_remain_unchanged(self):
        expected = (
            ("S01.yaml", 1, None, None),
            ("S03.yaml", 2, "s03", 0.0),
            ("S04.yaml", 4, "s04", None),
        )
        for filename, version, profile, fixed_yaw in expected:
            with self.subTest(filename=filename):
                spec = scenario_manager.load_scenario(
                    str(PACKAGE / "config" / "scenarios" / filename)
                )
                self.assertEqual(spec.schema_version, version)
                self.assertEqual(spec.scenario_profile, profile)
                if version == 1:
                    self.assertIsNone(spec.observer_motion)
                else:
                    self.assertEqual(spec.observer_motion.fixed_yaw, fixed_yaw)

    def test_dynamic_transform_preserves_s05_yaw(self):
        spec = scenario_manager.load_scenario(str(self.path))
        state = scenario_manager.MotionState(
            scenario_manager.PoseSpec(1.0, 0.0, 2.0, 0.375),
            vx=0.5,
            vy=0.5,
            yaw_rate=0.09375,
        )
        transform = scenario_manager.build_dynamic_observer_transform(
            spec.observer_motion, state, 14_000_000_000
        )
        self.assertEqual(transform.header.frame_id, "world")
        self.assertEqual(transform.child_frame_id, "uav1/fcu")
        self.assertAlmostEqual(transform.transform.rotation.z, math.sin(0.1875))
        self.assertAlmostEqual(transform.transform.rotation.w, math.cos(0.1875))


if __name__ == "__main__":
    unittest.main()
