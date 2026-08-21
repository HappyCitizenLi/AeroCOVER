#!/usr/bin/env python3
"""Schema-v7 S02 authority, minimum-snap and fail-closed tests."""

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
    "mid360_scenario_manager_s02_contract", SCRIPT
)
scenario_manager = importlib.util.module_from_spec(MODULE_SPEC)
sys.modules[MODULE_SPEC.name] = scenario_manager
MODULE_SPEC.loader.exec_module(scenario_manager)


class ScenarioManagerS02Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.path = PACKAGE / "config" / "scenarios" / "S02.yaml"
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

    def test_exact_schema_v7_identity_timeline_and_authorities_load(self):
        spec = scenario_manager.load_scenario(str(self.path))
        self.assertEqual(spec.schema_version, 7)
        self.assertEqual(spec.scenario_profile, "s02")
        self.assertEqual(
            spec.semantic_contract_id,
            "mid360-multi-uav-s02-crossing-partial-occlusion-v3",
        )
        self.assertEqual(
            spec.scenario_contract_id,
            "ten-uav-c3-dynamics-and-avoidance-v2",
        )
        self.assertEqual(
            spec.scenario_id,
            "S02_ten_uav_minimum_snap_crossing_partial_occlusion",
        )
        self.assertEqual(spec.seed, 2027)
        self.assertEqual(spec.world_frame, "world")
        self.assertEqual((spec.repeat_count, spec.repeat_mode), (1, "restart"))
        self.assertEqual(spec.start_delay_ns, 8_000_000_000)
        self.assertEqual(spec.cycle_duration_ns, 63_000_000_000)
        self.assertEqual(spec.final_hold_ns, 4_000_000_000)
        self.assertEqual(
            set(spec.targets),
            {"uav{}".format(index) for index in range(1, 12)},
        )
        self.assertFalse(spec.targets["uav1"].command_model)
        self.assertTrue(all(
            spec.targets["uav{}".format(index)].command_model
            for index in range(2, 12)
        ))
        for index in range(1, 12):
            name = "uav{}".format(index)
            target = spec.targets[name]
            self.assertEqual(target.model_name, name)
            self.assertEqual(
                target.truth_topic,
                "/mid360_multi_uav_sim/ground_truth/{}/odom".format(name),
            )
            expected_child = "uav1/fcu" if index == 1 else name + "/base_link"
            self.assertEqual(target.child_frame_id, expected_child)

        self.assertEqual(
            [phase.phase_id for phase in spec.phases],
            [
                "converge_to_occlusion_a",
                "occlusion_hold_a",
                "outer_climb",
                "counterflow_exchange_a",
                "outer_descend",
                "occlusion_hold_b",
                "inner_descend",
                "counterflow_exchange_b",
                "inner_rise",
                "occlusion_hold_c",
                "fan_out",
            ],
        )
        self.assertEqual(
            [phase.duration_ns for phase in spec.phases],
            [
                6_000_000_000,
                3_000_000_000,
                4_000_000_000,
                13_000_000_000,
                4_000_000_000,
                3_000_000_000,
                4_000_000_000,
                13_000_000_000,
                4_000_000_000,
                3_000_000_000,
                6_000_000_000,
            ],
        )
        self.assertEqual(
            spec.phase_starts_ns,
            (
                0,
                6_000_000_000,
                9_000_000_000,
                13_000_000_000,
                26_000_000_000,
                30_000_000_000,
                33_000_000_000,
                37_000_000_000,
                50_000_000_000,
                54_000_000_000,
                57_000_000_000,
            ),
        )
        self.assertEqual(
            [phase.interpolation for phase in spec.phases],
            [
                "minimum_snap",
                "hold",
                "minimum_snap",
                "minimum_snap",
                "minimum_snap",
                "hold",
                "minimum_snap",
                "minimum_snap",
                "minimum_snap",
                "hold",
                "minimum_snap",
            ],
        )

    def test_minimum_snap_sampling_has_exact_endpoints_and_boundary_velocity(self):
        spec = scenario_manager.load_scenario(str(self.path))
        manager = object.__new__(scenario_manager.ScenarioManager)
        manager.spec = spec

        for index, phase in enumerate(spec.phases):
            start = manager._sample_phase(index, 0, 1)
            midpoint = manager._sample_phase(index, phase.duration_ns // 2, 1)
            endpoint = manager._sample_phase(index, phase.duration_ns, 1)
            expected_start = (
                {name: target.initial_pose for name, target in spec.targets.items()}
                if index == 0
                else spec.phases[index - 1].goals
            )
            for name in spec.targets:
                with self.subTest(phase=phase.phase_id, target=name):
                    self.assertEqual(start[name].pose, expected_start[name])
                    self.assertEqual(endpoint[name].pose, phase.goals[name])
                    self.assertEqual(
                        (start[name].vx, start[name].vy, start[name].vz,
                         start[name].yaw_rate),
                        (0.0, 0.0, 0.0, 0.0),
                    )
                    self.assertEqual(
                        (endpoint[name].vx, endpoint[name].vy,
                         endpoint[name].vz, endpoint[name].yaw_rate),
                        (0.0, 0.0, 0.0, 0.0),
                    )
                    if phase.interpolation == "hold":
                        self.assertEqual(midpoint[name], start[name])
                        continue
                    expected_pose = scenario_manager.PoseSpec(
                        0.5 * (expected_start[name].x + phase.goals[name].x),
                        0.5 * (expected_start[name].y + phase.goals[name].y),
                        0.5 * (expected_start[name].z + phase.goals[name].z),
                        0.5 * (expected_start[name].yaw + phase.goals[name].yaw),
                    )
                    self.assertTrue(
                        scenario_manager._pose_close(
                            midpoint[name].pose, expected_pose
                        )
                    )
                    derivative = 2.1875 / (
                        phase.duration_ns / float(scenario_manager.NSEC_PER_SEC)
                    )
                    self.assertAlmostEqual(
                        midpoint[name].vx,
                        derivative * (phase.goals[name].x - expected_start[name].x),
                    )
                    self.assertAlmostEqual(
                        midpoint[name].vy,
                        derivative * (phase.goals[name].y - expected_start[name].y),
                    )
                    self.assertAlmostEqual(
                        midpoint[name].vz,
                        derivative * (phase.goals[name].z - expected_start[name].z),
                    )

        # The seventh-order scale is C3 at every segment boundary.  The
        # manager only publishes pose/velocity, so acceleration and jerk are
        # checked from the exact polynomial implemented by _sample_phase.
        for tau in (0.0, 1.0):
            velocity = 140.0 * tau ** 3 * (1.0 - tau) ** 3
            acceleration = (
                420.0 * tau ** 2
                - 1680.0 * tau ** 3
                + 2100.0 * tau ** 4
                - 840.0 * tau ** 5
            )
            jerk = (
                840.0 * tau
                - 5040.0 * tau ** 2
                + 8400.0 * tau ** 3
                - 4200.0 * tau ** 4
            )
            self.assertAlmostEqual(velocity, 0.0)
            self.assertAlmostEqual(acceleration, 0.0)
            self.assertAlmostEqual(jerk, 0.0)

    def test_event_payload_carries_v7_contract_once(self):
        spec = scenario_manager.load_scenario(str(self.path))
        manager = object.__new__(scenario_manager.ScenarioManager)
        manager.spec = spec
        manager.last_commanded_states = {
            name: scenario_manager.MotionState(spec.phases[1].goals[name])
            for name in spec.targets
        }
        manager.event_pub = mock.Mock()
        with mock.patch.object(scenario_manager.rospy, "loginfo"):
            manager._event("occlusion_a_start", 1, 14_000_000_000)
        manager.event_pub.publish.assert_called_once()
        payload = json.loads(manager.event_pub.publish.call_args.args[0].data)
        self.assertEqual(payload["scenario_profile"], "s02")
        self.assertEqual(
            payload["semantic_contract_id"],
            "mid360-multi-uav-s02-crossing-partial-occlusion-v3",
        )
        self.assertEqual(
            payload["scenario_contract_id"],
            "ten-uav-c3-dynamics-and-avoidance-v2",
        )
        self.assertEqual(set(payload["commanded_pose"]), set(spec.targets))
        self.assertNotIn("dynamic_tf_authority", payload)

    def test_unknown_and_missing_keys_fail_closed_at_every_v7_layer(self):
        mutations = (
            (lambda value: value.__setitem__("extra", 1), "unknown keys"),
            (lambda value: value.pop("crossing_contract"), "missing keys"),
            (lambda value: value["runtime"].__setitem__("extra", 1), "unknown keys"),
            (lambda value: value["runtime"].pop("publish_rate"), "missing keys"),
            (lambda value: value["randomization"].__setitem__("extra", 1), "unknown keys"),
            (lambda value: value["targets"]["uav7"].__setitem__("extra", 1), "unknown keys"),
            (lambda value: value["targets"]["uav7"].pop("initial_pose"), "missing keys"),
            (lambda value: value["targets"]["uav7"]["initial_pose"].pop("yaw"), "missing keys"),
            (lambda value: value["crossing_contract"].__setitem__("extra", 1), "unknown keys"),
            (lambda value: value["crossing_contract"].pop("trajectory_model"), "missing keys"),
            (lambda value: value["crossing_contract"].pop("occlusion_center_offset_deg"), "missing keys"),
            (lambda value: value["crossing_contract"].pop("maximum_hold_pair_surface_gap_m"), "missing keys"),
            (lambda value: value["crossing_contract"]["occlusion_stages"][0].__setitem__("extra", 1), "unknown keys"),
            (lambda value: value["crossing_contract"]["occlusion_stages"][0]["pairs"][0].__setitem__("extra", 1), "unknown keys"),
            (lambda value: value["phases"][3].__setitem__("extra", 1), "unknown keys"),
            (lambda value: value["phases"][3].pop("duration"), "missing keys"),
        )
        for index, (mutation, pattern) in enumerate(mutations):
            with self.subTest(case=index):
                self.assert_invalid(mutation, pattern)

    def test_contract_identity_runtime_target_and_trajectory_drift_fail_closed(self):
        mutations = (
            (lambda value: value.__setitem__("scenario_profile", "s01"), "scenario_profile"),
            (lambda value: value.__setitem__("semantic_contract_id", "wrong"), "semantic_contract_id"),
            (lambda value: value.__setitem__("scenario_contract_id", "wrong"), "scenario_contract_id"),
            (lambda value: value.__setitem__("scenario", "wrong"), "schema-v7 scenario"),
            (lambda value: value.__setitem__("seed", 2028), "timeline contract"),
            (lambda value: value.__setitem__("repeat_count", 2), "timeline contract"),
            (lambda value: value.__setitem__("repeat_mode", "ping_pong"), "timeline contract"),
            (lambda value: value.__setitem__("start_delay", 7.0), "timeline contract"),
            (lambda value: value.__setitem__("final_hold", 3.0), "timeline contract"),
            (lambda value: value["runtime"].__setitem__("publish_rate", 40.0), "runtime authority"),
            (lambda value: value["randomization"]["position_offset_uniform"].__setitem__("x", 0.1), "offsets must all be zero"),
            (lambda value: value["targets"]["uav1"].__setitem__("command_model", True), "observer"),
            (lambda value: value["targets"]["uav7"].__setitem__("model_name", "target7"), "target 'uav7'"),
            (lambda value: value["targets"]["uav7"]["initial_pose"].__setitem__("x", 9.0), "target 'uav7'"),
            (lambda value: value["phases"][3].__setitem__("interpolation", "smoothstep"), "phase 3 contract differs"),
            (lambda value: value["phases"][3]["targets"]["uav2"].__setitem__("x", 2.4), "phase 3 contract differs"),
            (lambda value: value["phases"][5].__setitem__("event", "wrong"), "phase 5 contract differs"),
            (lambda value: value["phases"][10].__setitem__("duration", 5.0), "duration does not equal"),
        )
        for index, (mutation, pattern) in enumerate(mutations):
            with self.subTest(case=index):
                self.assert_invalid(mutation, pattern)

    def test_crossing_semantics_and_matching_mutations_fail_closed(self):
        def duplicate_near(value):
            pairs = value["crossing_contract"]["occlusion_stages"][0]["pairs"]
            pairs[1]["near"] = pairs[0]["near"]

        def valid_but_wrong_matching(value):
            pairs = value["crossing_contract"]["occlusion_stages"][1]["pairs"]
            pairs[0]["far"], pairs[1]["far"] = pairs[1]["far"], pairs[0]["far"]

        mutations = (
            (lambda value: value["crossing_contract"].__setitem__("geometry_mode", "deskewed"), "geometry_mode differs"),
            (lambda value: value["crossing_contract"].__setitem__("trajectory_model", "quintic"), "trajectory_model differs"),
            (lambda value: value["crossing_contract"].__setitem__("continuity_order", 2), "requires C3"),
            (lambda value: value["crossing_contract"].__setitem__("moving_target_count", 9), "ten moving targets"),
            (lambda value: value["crossing_contract"].__setitem__("occlusion_center_offset_deg", 1.7), "crossing geometry contract differs"),
            (lambda value: value["crossing_contract"].__setitem__("max_speed_mps", 0.0), "must be positive"),
            (duplicate_near, "not a perfect matching"),
            (valid_but_wrong_matching, "occlusion pair schedule differs"),
            (lambda value: value["crossing_contract"]["occlusion_stages"][2].__setitem__("phase", "fan_out"), "occlusion stage phases differ"),
        )
        for index, (mutation, pattern) in enumerate(mutations):
            with self.subTest(case=index):
                self.assert_invalid(mutation, pattern)

    def test_every_parameter_override_is_rejected_even_when_value_matches(self):
        overrides = (
            {"seed_override": 2027},
            {"duration_override": 63.0},
            {"repeat_count_override": 1},
            {"final_hold_override": 4.0},
            {"start_delay_override": 8.0},
        )
        for override in overrides:
            with self.subTest(override=override):
                self.assert_invalid(
                    lambda value: None,
                    "does not permit parameter overrides",
                    **override,
                )


if __name__ == "__main__":
    unittest.main()
