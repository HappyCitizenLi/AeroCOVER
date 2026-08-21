#!/usr/bin/env python3
"""Pure schema-v6 S06 online-map authority and fail-closed tests."""

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
    "mid360_scenario_manager_s06_contract", SCRIPT
)
scenario_manager = importlib.util.module_from_spec(MODULE_SPEC)
sys.modules[MODULE_SPEC.name] = scenario_manager
MODULE_SPEC.loader.exec_module(scenario_manager)


class ScenarioManagerS06Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path = PACKAGE / "config" / "scenarios" / "S06.yaml"
        cls.document = yaml.safe_load(cls.path.read_text(encoding="utf-8"))
        cls.online_map_contract = {
            "initial_state": "empty_online_scores_no_apriori_loader",
            "apriori_loader_used": False,
            "initial_map_revision": 0,
            "bootstrap_phase": "map_bootstrap_clear_hold",
            "discovery_primitive": "background_wall/wall_collision",
            "scored_map_occluded_target": "uav4",
            "transition": "initial_unknown_to_committed_sure_occupied",
        }

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

    def test_exact_schema_v6_authority_and_online_map_contract_load(self):
        spec = scenario_manager.load_scenario(str(self.path))
        self.assertEqual(spec.schema_version, 6)
        self.assertEqual(spec.scenario_profile, "s06")
        self.assertEqual(
            spec.semantic_contract_id,
            "mid360-multi-uav-s06-online-map-occlusion-v1",
        )
        self.assertEqual(spec.scenario_id, "S06_incomplete_map_online_occlusion")
        self.assertEqual(spec.seed, 909)
        self.assertEqual(spec.world_frame, "world")
        self.assertEqual((spec.repeat_count, spec.repeat_mode), (1, "restart"))
        self.assertEqual(spec.start_delay_ns, 5_000_000_000)
        self.assertEqual(spec.cycle_duration_ns, 22_000_000_000)
        self.assertEqual(spec.final_hold_ns, 3_000_000_000)
        self.assertEqual(dict(spec.online_map_contract), self.online_map_contract)
        self.assertEqual(
            set(spec.targets), {"uav1", "uav2", "uav3", "uav4"}
        )
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
        self.assertEqual(motion.observer, "uav1")
        self.assertEqual(motion.tf_authority, "scenario_manager")
        self.assertIsNone(motion.fixed_yaw)
        self.assertEqual(motion.minimum_translation_m, 2.0)
        self.assertEqual(motion.minimum_yaw_change_rad, 0.35)
        self.assertEqual(motion.scored_phase, "online_occlusion_scored")
        self.assertEqual(motion.stationary_targets, ("uav2", "uav3", "uav4"))

    def test_exact_timeline_events_and_observer_endpoints(self):
        spec = scenario_manager.load_scenario(str(self.path))
        self.assertEqual(
            [phase.phase_id for phase in spec.phases],
            [
                "map_bootstrap_clear_hold",
                "approach_online_occlusion",
                "online_occlusion_scored",
                "leave_online_occlusion",
                "final_clear_hold",
            ],
        )
        self.assertEqual(
            [phase.duration_ns for phase in spec.phases],
            [
                4_000_000_000,
                4_000_000_000,
                6_000_000_000,
                5_000_000_000,
                3_000_000_000,
            ],
        )
        self.assertEqual(
            spec.phase_starts_ns,
            (0, 4_000_000_000, 8_000_000_000, 14_000_000_000, 19_000_000_000),
        )
        self.assertEqual(
            [phase.interpolation for phase in spec.phases],
            ["hold", "linear", "linear", "linear", "hold"],
        )
        self.assertEqual(
            [phase.event for phase in spec.phases],
            [
                None,
                "map_bootstrap_end",
                "online_occlusion_start",
                "online_occlusion_end",
                "final_clear_start",
            ],
        )
        self.assertEqual(
            tuple(phase.goals["uav1"] for phase in spec.phases),
            (
                scenario_manager.PoseSpec(4.0, -3.0, 2.0, -math.pi / 4.0),
                scenario_manager.PoseSpec(0.0, 0.0, 2.0, 0.0),
                scenario_manager.PoseSpec(-2.0, -6.0 / 13.0, 2.0, 0.4),
                scenario_manager.PoseSpec(4.0, -3.0, 2.0, math.pi / 4.0),
                scenario_manager.PoseSpec(4.0, -3.0, 2.0, math.pi / 4.0),
            ),
        )
        self.assertEqual(
            {
                "cycle_start": 0,
                "map_bootstrap_end": 4_000_000_000,
                "online_occlusion_start": 8_000_000_000,
                "online_occlusion_end": 14_000_000_000,
                "final_clear_start": 19_000_000_000,
                "cycle_complete": 22_000_000_000,
                "final_hold_start": 22_000_000_000,
                "scenario_complete": 25_000_000_000,
            },
            {
                "cycle_start": 0,
                "map_bootstrap_end": spec.phase_starts_ns[1],
                "online_occlusion_start": spec.phase_starts_ns[2],
                "online_occlusion_end": spec.phase_starts_ns[3],
                "final_clear_start": spec.phase_starts_ns[4],
                "cycle_complete": spec.cycle_duration_ns,
                "final_hold_start": spec.cycle_duration_ns,
                "scenario_complete": spec.cycle_duration_ns + spec.final_hold_ns,
            },
        )

    def test_trajectory_is_continuous_velocities_exact_and_targets_stationary(self):
        spec = scenario_manager.load_scenario(str(self.path))
        manager = object.__new__(scenario_manager.ScenarioManager)
        manager.spec = spec
        expected_midpoints = {
            0: (4.0, -3.0, -math.pi / 4.0, 0.0, 0.0, 0.0),
            1: (2.0, -1.5, -math.pi / 8.0, -1.0, 0.75, math.pi / 16.0),
            2: (-1.0, -3.0 / 13.0, 0.2, -1.0 / 3.0, -1.0 / 13.0, 1.0 / 15.0),
            3: (1.0, -45.0 / 26.0, (0.4 + math.pi / 4.0) / 2.0,
                1.2, -33.0 / 65.0, (math.pi / 4.0 - 0.4) / 5.0),
            4: (4.0, -3.0, math.pi / 4.0, 0.0, 0.0, 0.0),
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
                for name in ("uav2", "uav3", "uav4"):
                    self.assertEqual(states[name].pose, spec.targets[name].initial_pose)
                    self.assertEqual(
                        (states[name].vx, states[name].vy, states[name].vz,
                         states[name].yaw_rate),
                        (0.0, 0.0, 0.0, 0.0),
                    )

        for index in range(len(spec.phases) - 1):
            with self.subTest(boundary=index):
                endpoint = manager._sample_phase(
                    index, spec.phases[index].duration_ns, 1
                )
                next_start = manager._sample_phase(index + 1, 0, 1)
                for name in spec.targets:
                    self.assertEqual(endpoint[name].pose, next_start[name].pose)

        scored_start = spec.phases[1].goals["uav1"]
        scored_goal = spec.phases[2].goals["uav1"]
        self.assertGreaterEqual(
            scenario_manager._translation_distance(scored_start, scored_goal), 2.0
        )
        self.assertGreaterEqual(
            scenario_manager._yaw_distance(scored_start, scored_goal), 0.35
        )

    def test_event_payload_carries_the_exact_online_map_contract(self):
        spec = scenario_manager.load_scenario(str(self.path))
        manager = object.__new__(scenario_manager.ScenarioManager)
        manager.spec = spec
        manager.last_commanded_states = {
            name: scenario_manager.MotionState(spec.phases[2].goals[name])
            for name in spec.targets
        }
        manager.event_pub = mock.Mock()
        with mock.patch.object(scenario_manager.rospy, "loginfo"):
            manager._event("online_occlusion_start", 1, 8_000_000_000)
        payload = json.loads(manager.event_pub.publish.call_args.args[0].data)
        self.assertEqual(payload["scenario_profile"], "s06")
        self.assertEqual(
            payload["semantic_contract_id"],
            "mid360-multi-uav-s06-online-map-occlusion-v1",
        )
        self.assertEqual(payload["online_map_contract"], self.online_map_contract)
        self.assertEqual(payload["dynamic_tf_authority"], "scenario_manager")
        self.assertEqual(set(payload["commanded_pose"]), set(spec.targets))
        self.assertNotIn("scenario_contract_id", payload)

    def test_unknown_and_missing_keys_fail_closed_at_every_schema_layer(self):
        mutations = (
            (lambda value: value.__setitem__("extra", 1), "unknown keys"),
            (lambda value: value.pop("duration"), "missing keys"),
            (lambda value: value["runtime"].__setitem__("extra", 1), "unknown keys"),
            (lambda value: value["runtime"].pop("publish_rate"), "missing keys"),
            (lambda value: value["targets"]["uav4"].__setitem__("extra", 1), "unknown keys"),
            (lambda value: value["targets"]["uav4"].pop("initial_pose"), "missing keys"),
            (lambda value: value["targets"]["uav4"]["initial_pose"].pop("yaw"), "missing keys"),
            (lambda value: value["observer_motion"].__setitem__("extra", 1), "unknown keys"),
            (lambda value: value["observer_motion"].pop("minimum_yaw_change_rad"), "missing keys"),
            (lambda value: value["online_map_contract"].__setitem__("extra", 1), "unknown keys"),
            (lambda value: value["online_map_contract"].pop("transition"), "missing keys"),
            (lambda value: value["phases"][2].__setitem__("extra", 1), "unknown keys"),
            (lambda value: value["phases"][2].pop("duration"), "missing keys"),
        )
        for index, (mutation, pattern) in enumerate(mutations):
            with self.subTest(case=index):
                self.assert_invalid(mutation, pattern)

    def test_online_map_contract_missing_type_and_value_drift_fail_closed(self):
        mutations = (
            (lambda value: value.pop("online_map_contract"), "missing keys"),
            (lambda value: value["online_map_contract"].__setitem__("initial_state", "preloaded"), "online_map_contract differs"),
            (lambda value: value["online_map_contract"].__setitem__("apriori_loader_used", True), "online_map_contract differs"),
            (lambda value: value["online_map_contract"].__setitem__("apriori_loader_used", 0), "must be boolean"),
            (lambda value: value["online_map_contract"].__setitem__("initial_map_revision", 1), "online_map_contract differs"),
            (lambda value: value["online_map_contract"].__setitem__("initial_map_revision", False), "must be an integer"),
            (lambda value: value["online_map_contract"].__setitem__("bootstrap_phase", "wrong"), "online_map_contract differs"),
            (lambda value: value["online_map_contract"].__setitem__("discovery_primitive", "pillar/pillar_collision"), "online_map_contract differs"),
            (lambda value: value["online_map_contract"].__setitem__("scored_map_occluded_target", "uav3"), "online_map_contract differs"),
            (lambda value: value["online_map_contract"].__setitem__("transition", "unknown_to_occupied"), "online_map_contract differs"),
        )
        for index, (mutation, pattern) in enumerate(mutations):
            with self.subTest(case=index):
                self.assert_invalid(mutation, pattern)

    def test_identity_runtime_targets_motion_and_phase_drift_fail_closed(self):
        mutations = (
            (lambda value: value.__setitem__("scenario_profile", "s05"), "scenario_profile"),
            (lambda value: value.__setitem__("semantic_contract_id", "wrong"), "semantic_contract_id"),
            (lambda value: value.__setitem__("scenario", "wrong"), "schema-v6 scenario"),
            (lambda value: value.__setitem__("seed", 908), "timeline contract"),
            (lambda value: value.__setitem__("repeat_count", 2), "timeline contract"),
            (lambda value: value.__setitem__("repeat_mode", "ping_pong"), "timeline contract"),
            (lambda value: value.__setitem__("start_delay", 4.0), "timeline contract"),
            (lambda value: value.__setitem__("final_hold", 2.0), "timeline contract"),
            (lambda value: value["runtime"].__setitem__("publish_rate", 40.0), "runtime authority"),
            (lambda value: value["randomization"]["position_offset_uniform"].__setitem__("x", 0.1), "offsets must all be zero"),
            (lambda value: value["targets"]["uav4"]["initial_pose"].__setitem__("x", 14.1), "target 'uav4'"),
            (lambda value: value["observer_motion"].__setitem__("minimum_translation_m", 1.9), "observer_motion contract differs"),
            (lambda value: value["observer_motion"].__setitem__("minimum_yaw_change_rad", 0.3), "observer_motion contract differs"),
            (lambda value: value["observer_motion"].__setitem__("scored_phase", "approach_online_occlusion"), "observer_motion contract differs"),
            (lambda value: value["phases"][1].__setitem__("event", "wrong"), "phase 1 contract differs"),
            (lambda value: value["phases"][2].__setitem__("duration", 5.0), "duration does not equal"),
            (lambda value: value["phases"][2].__setitem__("interpolation", "smoothstep"), "phase 2 contract differs"),
            (lambda value: value["phases"][3]["targets"]["uav1"].__setitem__("y", -2.9), "phase 3 contract differs"),
        )
        for index, (mutation, pattern) in enumerate(mutations):
            with self.subTest(case=index):
                self.assert_invalid(mutation, pattern)

    def test_every_parameter_override_is_rejected_even_when_value_matches(self):
        overrides = (
            {"seed_override": 909},
            {"duration_override": 22.0},
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
            ("S01.yaml", 1, None),
            ("S03.yaml", 2, "s03"),
            ("S04.yaml", 4, "s04"),
            ("S05.yaml", 5, "s05"),
        )
        for filename, version, profile in expected:
            with self.subTest(filename=filename):
                spec = scenario_manager.load_scenario(
                    str(PACKAGE / "config" / "scenarios" / filename)
                )
                self.assertEqual(spec.schema_version, version)
                self.assertEqual(spec.scenario_profile, profile)
                self.assertIsNone(spec.online_map_contract)


if __name__ == "__main__":
    unittest.main()
