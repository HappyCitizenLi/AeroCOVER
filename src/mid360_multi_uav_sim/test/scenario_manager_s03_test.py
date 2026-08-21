#!/usr/bin/env python3
"""Pure schema, motion and dynamic-TF helper tests for scenario_manager."""

import copy
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE / "scripts" / "scenario_manager.py"
MODULE_SPEC = importlib.util.spec_from_file_location(
    "mid360_scenario_manager_s03_contract", SCRIPT
)
scenario_manager = importlib.util.module_from_spec(MODULE_SPEC)
sys.modules[MODULE_SPEC.name] = scenario_manager
MODULE_SPEC.loader.exec_module(scenario_manager)


class ScenarioManagerS03Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.paths = {
            name: PACKAGE / "config" / "scenarios" / "{}.yaml".format(name)
            for name in ("S01", "S02", "S03", "S04", "S05", "S06")
        }
        cls.documents = {
            name: yaml.safe_load(path.read_text(encoding="utf-8"))
            for name, path in cls.paths.items()
        }

    def load_mutated(self, mutator):
        document = copy.deepcopy(self.documents["S03"])
        mutator(document)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "scenario.yaml"
            path.write_text(
                yaml.safe_dump(document, sort_keys=False), encoding="utf-8"
            )
            return scenario_manager.load_scenario(str(path))

    def assert_invalid(self, mutator, pattern):
        with self.assertRaisesRegex(scenario_manager.ScenarioError, pattern):
            self.load_mutated(mutator)

    def test_schema_v1_semantics_and_no_tf_authority_are_unchanged(self):
        for name in ("S01",):
            with self.subTest(name=name):
                spec = scenario_manager.load_scenario(str(self.paths[name]))
                self.assertEqual(spec.schema_version, 1)
                self.assertIsNone(spec.scenario_profile)
                self.assertIsNone(spec.semantic_contract_id)
                self.assertIsNone(spec.observer_motion)
                self.assertFalse(spec.targets["uav1"].command_model)

        with mock.patch.object(
            scenario_manager.tf2_ros, "TransformBroadcaster"
        ) as constructor:
            self.assertIsNone(scenario_manager.create_dynamic_tf_broadcaster(None))
            constructor.assert_not_called()

    def test_only_exact_schema_v2_s03_contract_can_command_observer(self):
        spec = scenario_manager.load_scenario(str(self.paths["S03"]))
        self.assertEqual(spec.schema_version, 2)
        self.assertEqual(spec.scenario_profile, "s03")
        self.assertEqual(
            spec.semantic_contract_id,
            scenario_manager.S03_SEMANTIC_CONTRACT_ID,
        )
        self.assertTrue(spec.targets["uav1"].command_model)
        self.assertIsNotNone(spec.observer_motion)

        self.assert_invalid(
            lambda value: value.__setitem__("schema_version", 1),
            "uav1 is the observer",
        )
        self.assert_invalid(
            lambda value: value.__setitem__("scenario_profile", "s05"),
            "scenario_profile",
        )
        self.assert_invalid(
            lambda value: value.__setitem__("semantic_contract_id", "wrong"),
            "semantic_contract_id",
        )
        self.assert_invalid(
            lambda value: value["targets"]["uav1"].__setitem__(
                "command_model", False
            ),
            "must set command_model: true",
        )

    def test_schema_v2_rejects_unknown_keys_at_every_contract_layer(self):
        mutations = (
            (lambda value: value.__setitem__("extra", 1), "scenario"),
            (lambda value: value["runtime"].__setitem__("extra", 1), "runtime"),
            (
                lambda value: value["targets"]["uav1"].__setitem__("extra", 1),
                "targets.uav1",
            ),
            (
                lambda value: value["observer_motion"].__setitem__("extra", 1),
                "observer_motion",
            ),
            (
                lambda value: value["phases"][0].__setitem__("extra", 1),
                r"phases\[0\]",
            ),
        )
        for mutation, pattern in mutations:
            with self.subTest(pattern=pattern):
                self.assert_invalid(mutation, pattern)

    def test_parent_child_and_unique_tf_authority_fail_closed(self):
        self.assert_invalid(
            lambda value: value["observer_motion"].__setitem__(
                "parent_frame", "map"
            ),
            "parent_frame",
        )
        self.assert_invalid(
            lambda value: value["observer_motion"].__setitem__(
                "child_frame", "uav1/base_link"
            ),
            "child_frame",
        )
        self.assert_invalid(
            lambda value: value["observer_motion"].__setitem__(
                "tf_authority", "another_node"
            ),
            "tf_authority",
        )
        self.assert_invalid(
            lambda value: value["observer_motion"].__setitem__(
                "unique_dynamic_tf_authority", False
            ),
            "unique_dynamic_tf_authority",
        )

    def test_motion_contract_rejects_zero_translation_yaw_and_target_motion(self):
        def zero_translation(value):
            value["phases"][1]["targets"]["uav1"]["x"] = 0.0

        self.assert_invalid(zero_translation, "translation")

        def changing_yaw(value):
            value["phases"][1]["targets"]["uav1"]["yaw"] = 0.1

        self.assert_invalid(changing_yaw, "yaw must remain fixed")

        def move_uncommanded(value):
            value["targets"]["uav2"]["command_model"] = False
            value["phases"][1].setdefault("targets", {})["uav2"] = {
                "x": 10.0,
                "y": -1.0,
                "z": 2.25,
                "yaw": 0.0,
            }

        self.assert_invalid(move_uncommanded, "changes uncommanded entity")

        def move_scored_target(value):
            value["phases"][1].setdefault("targets", {})["uav2"] = {
                "x": 10.0,
                "y": -1.0,
                "z": 2.25,
                "yaw": 0.0,
            }

        self.assert_invalid(move_scored_target, "moves stationary targets")

    def test_sampled_scored_motion_is_nonzero_and_targets_are_stationary(self):
        spec = scenario_manager.load_scenario(str(self.paths["S03"]))
        manager = object.__new__(scenario_manager.ScenarioManager)
        manager.spec = spec
        phase_index = next(
            index
            for index, phase in enumerate(spec.phases)
            if phase.phase_id == spec.observer_motion.scored_phase
        )
        midpoint = spec.phases[phase_index].duration_ns // 2
        states = manager._sample_phase(phase_index, midpoint, 1)
        self.assertAlmostEqual(states["uav1"].pose.x, 2.0)
        self.assertAlmostEqual(states["uav1"].vx, 0.5)
        self.assertAlmostEqual(states["uav1"].pose.yaw, 0.0)
        self.assertAlmostEqual(states["uav1"].yaw_rate, 0.0)
        for name in ("uav2", "uav3"):
            self.assertEqual(states[name].pose, spec.targets[name].initial_pose)
            self.assertEqual(
                (states[name].vx, states[name].vy, states[name].vz),
                (0.0, 0.0, 0.0),
            )

    def test_dynamic_tf_helpers_preserve_state_stamp_and_deduplicate(self):
        spec = scenario_manager.load_scenario(str(self.paths["S03"]))
        state = scenario_manager.MotionState(
            scenario_manager.PoseSpec(2.0, 0.0, 2.0, 0.0),
            vx=0.5,
        )
        stamp_ns = 12_345_678_901
        transform = scenario_manager.build_dynamic_observer_transform(
            spec.observer_motion, state, stamp_ns
        )
        self.assertEqual(transform.header.stamp.to_nsec(), stamp_ns)
        self.assertEqual(transform.header.frame_id, "world")
        self.assertEqual(transform.child_frame_id, "uav1/fcu")
        self.assertEqual(
            (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ),
            (2.0, 0.0, 2.0),
        )
        self.assertTrue(
            scenario_manager._should_publish_dynamic_tf(None, None, stamp_ns, state)
        )
        self.assertFalse(
            scenario_manager._should_publish_dynamic_tf(
                stamp_ns, state, stamp_ns, state
            )
        )
        with self.assertRaisesRegex(scenario_manager.ScenarioError, "conflicting"):
            scenario_manager._should_publish_dynamic_tf(
                stamp_ns,
                state,
                stamp_ns,
                scenario_manager.MotionState(
                    scenario_manager.PoseSpec(2.1, 0.0, 2.0, 0.0), vx=0.5
                ),
            )
        with self.assertRaisesRegex(scenario_manager.ScenarioError, "backwards"):
            scenario_manager._should_publish_dynamic_tf(
                stamp_ns, state, stamp_ns - 1, state
            )

    def test_manager_publishes_tf_once_and_only_after_successful_model_writes(self):
        spec = scenario_manager.load_scenario(str(self.paths["S03"]))
        sampler = object.__new__(scenario_manager.ScenarioManager)
        sampler.spec = spec
        states = sampler._sample_phase(1, spec.phases[1].duration_ns // 2, 1)

        manager = object.__new__(scenario_manager.ScenarioManager)
        manager.spec = spec
        manager.consecutive_failures = 0
        manager.last_commanded_states = {}
        calls = []
        manager._set_target = lambda target, state: calls.append(
            ("set", target.name, state)
        )
        manager._publish_dynamic_observer_tf = lambda state, stamp: calls.append(
            ("tf", stamp, state)
        ) or True
        manager._publish_truth = lambda name, target, state, stamp: calls.append(
            ("truth", name, stamp, state)
        )
        stamp_ns = 9_000_000_000
        self.assertTrue(manager._apply_and_publish(states, stamp_ns))
        self.assertEqual(
            [(item[0], item[1]) for item in calls[:3]],
            [("set", "uav1"), ("set", "uav2"), ("set", "uav3")],
        )
        self.assertEqual(calls[3][0], "tf")
        self.assertEqual(calls[3][1], stamp_ns)
        self.assertIs(calls[3][2], states["uav1"])
        self.assertEqual([item[0] for item in calls[4:]], ["truth"] * 3)

        calls.clear()
        self.assertTrue(manager._apply_and_publish(states, stamp_ns + 1))
        self.assertNotIn("set", [item[0] for item in calls])
        self.assertEqual(calls[0][0], "tf")
        self.assertEqual([item[0] for item in calls[1:]], ["truth"] * 3)

        broadcaster = mock.Mock()
        publisher = object.__new__(scenario_manager.ScenarioManager)
        publisher.spec = spec
        publisher.dynamic_tf_broadcaster = broadcaster
        publisher.last_dynamic_tf_stamp_ns = None
        publisher.last_dynamic_tf_state = None
        self.assertTrue(publisher._publish_dynamic_observer_tf(states["uav1"], stamp_ns))
        self.assertFalse(publisher._publish_dynamic_observer_tf(states["uav1"], stamp_ns))
        self.assertEqual(broadcaster.sendTransform.call_count, 1)
        sent = broadcaster.sendTransform.call_args.args[0]
        self.assertEqual(sent.header.stamp.to_nsec(), stamp_ns)
        self.assertEqual(sent.transform.translation.x, states["uav1"].pose.x)

    def test_broadcaster_factory_creates_exactly_one_s03_authority(self):
        spec = scenario_manager.load_scenario(str(self.paths["S03"]))
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

    def test_s03_model_ready_stamp_is_fresh_while_v1_baseline_is_preserved(self):
        dependency_stamp = 100_000_000
        current_stamp = 575_000_000
        for name in ("S03", "S01"):
            with self.subTest(name=name):
                spec = scenario_manager.load_scenario(str(self.paths[name]))
                manager = object.__new__(scenario_manager.ScenarioManager)
                manager.spec = spec
                required = [
                    target.model_name
                    for target in spec.targets.values()
                    if target.command_model
                ]
                manager.world_properties = mock.Mock(
                    return_value=mock.Mock(success=True, model_names=required)
                )
                manager.clock = mock.Mock()
                manager.clock.sample.return_value = (current_stamp, 0.0)
                observed = []
                manager._apply_and_publish = (
                    lambda states, stamp, enforce_failure_limit=False:
                    observed.append(stamp) or True
                )
                result = manager._wait_for_models({}, dependency_stamp)
                expected = current_stamp if name == "S03" else dependency_stamp
                self.assertEqual(result, expected)
                self.assertEqual(observed, [expected])


if __name__ == "__main__":
    unittest.main()
