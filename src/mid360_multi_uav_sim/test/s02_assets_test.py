#!/usr/bin/env python3
"""Pure geometry, dynamics and launch contracts for the ten-target S02."""

import itertools
import math
from pathlib import Path
import shlex
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from numpy.polynomial import Polynomial
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
SENSOR_ORIGIN = np.asarray([0.0, 0.0, 2.25])
TIME_SCALE = Polynomial([0.0, 0.0, 0.0, 0.0, 35.0, -84.0, 70.0, -20.0])


def _pose(value):
    return np.asarray([float(value[key]) for key in ("x", "y", "z")])


def _real_unit_roots(polynomial):
    output = []
    for value in polynomial.roots():
        if abs(value.imag) <= 1.0e-10 and -1.0e-12 <= value.real <= 1.0 + 1.0e-12:
            output.append(min(1.0, max(0.0, float(value.real))))
    return output


def _scalar_extrema(order):
    derivative = TIME_SCALE.deriv(order)
    candidates = [0.0, 1.0] + _real_unit_roots(derivative.deriv())
    return [float(derivative(value)) for value in candidates]


def _minimum_segment_distance(first_start, first_end, second_start, second_end):
    relative_start = first_start - second_start
    relative_delta = (first_end - first_start) - (second_end - second_start)
    denominator = float(np.dot(relative_delta, relative_delta))
    progress = 0.0 if denominator <= 1.0e-15 else min(
        1.0,
        max(0.0, -float(np.dot(relative_start, relative_delta)) / denominator),
    )
    return float(np.linalg.norm(relative_start + progress * relative_delta))


def _collision_envelope():
    root = ET.parse(
        PACKAGE / "models" / "simplified_target_uav" / "model.sdf"
    ).getroot()
    horizontal = 0.0
    vertical = 0.0
    for collision in root.findall(".//collision"):
        pose_text = collision.findtext("pose", default="0 0 0 0 0 0")
        pose = [float(value) for value in pose_text.split()]
        geometry = collision.find("geometry")
        box = geometry.find("box")
        cylinder = geometry.find("cylinder")
        if box is not None:
            size = [float(value) for value in box.findtext("size").split()]
            local_radius = math.hypot(0.5 * size[0], 0.5 * size[1])
            half_height = 0.5 * size[2]
        elif cylinder is not None:
            local_radius = float(cylinder.findtext("radius"))
            half_height = 0.5 * float(cylinder.findtext("length"))
        else:
            raise AssertionError("unsupported S02 collision primitive")
        horizontal = max(horizontal, math.hypot(pose[0], pose[1]) + local_radius)
        vertical = max(vertical, abs(pose[2]) + half_height)
    return horizontal, vertical, math.hypot(horizontal, vertical)


class S02AssetsContract(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.scenario = yaml.safe_load(
            (PACKAGE / "config" / "scenarios" / "S02.yaml").read_text(
                encoding="utf-8"
            )
        )
        cls.names = ["uav{}".format(index) for index in range(2, 12)]
        cls.starts = {
            name: _pose(cls.scenario["targets"][name]["initial_pose"])
            for name in cls.names
        }
        cls.phase_segments = []
        prior = dict(cls.starts)
        cls.phase_end = {}
        for phase in cls.scenario["phases"]:
            goals = dict(prior)
            for name, pose in phase.get("targets", {}).items():
                if name in cls.names:
                    goals[name] = _pose(pose)
            cls.phase_segments.append((phase, prior, goals))
            cls.phase_end[phase["id"]] = goals
            prior = goals

    def test_strict_identity_timeline_and_authorities(self):
        scenario = self.scenario
        self.assertEqual(scenario["schema_version"], 7)
        self.assertEqual(scenario["scenario_profile"], "s02")
        self.assertEqual(
            scenario["scenario"],
            "S02_ten_uav_minimum_snap_crossing_partial_occlusion",
        )
        self.assertEqual(
            scenario["semantic_contract_id"],
            "mid360-multi-uav-s02-crossing-partial-occlusion-v3",
        )
        self.assertEqual(
            scenario["scenario_contract_id"],
            "ten-uav-c3-dynamics-and-avoidance-v2",
        )
        self.assertEqual(scenario["repeat_count"], 1)
        self.assertEqual(scenario["repeat_mode"], "restart")
        self.assertEqual(float(scenario["start_delay"]), 8.0)
        self.assertEqual(float(scenario["duration"]), 63.0)
        self.assertEqual(float(scenario["final_hold"]), 4.0)
        self.assertAlmostEqual(
            sum(float(phase["duration"]) for phase in scenario["phases"]),
            63.0,
        )
        self.assertEqual(
            set(scenario["targets"]),
            {"uav{}".format(index) for index in range(1, 12)},
        )
        self.assertFalse(scenario["targets"]["uav1"]["command_model"])
        self.assertTrue(all(
            scenario["targets"][name]["command_model"] for name in self.names
        ))
        self.assertEqual(len({
            scenario["targets"][name]["truth_topic"]
            for name in scenario["targets"]
        }), 11)

    def test_minimum_snap_is_c3_and_within_dynamics_contract(self):
        contract = self.scenario["crossing_contract"]
        for order in (1, 2, 3):
            derivative = TIME_SCALE.deriv(order)
            self.assertAlmostEqual(float(derivative(0.0)), 0.0, places=12)
            self.assertAlmostEqual(float(derivative(1.0)), 0.0, places=12)

        speed_factor = max(abs(value) for value in _scalar_extrema(1))
        acceleration_values = _scalar_extrema(2)
        acceleration_factor = max(abs(value) for value in acceleration_values)
        jerk_factor = max(abs(value) for value in _scalar_extrema(3))
        maximum_speed = maximum_acceleration = maximum_jerk = 0.0
        maximum_tilt = 0.0
        minimum_specific_thrust = math.inf
        maximum_specific_thrust = 0.0
        gravity = 9.81

        for phase, starts, goals in self.phase_segments:
            duration = float(phase["duration"])
            if phase["interpolation"] == "hold":
                self.assertTrue(all(np.allclose(starts[name], goals[name])
                                    for name in self.names))
                continue
            self.assertEqual(phase["interpolation"], "minimum_snap")
            for name in self.names:
                displacement = goals[name] - starts[name]
                length = float(np.linalg.norm(displacement))
                maximum_speed = max(maximum_speed, speed_factor * length / duration)
                maximum_acceleration = max(
                    maximum_acceleration,
                    acceleration_factor * length / (duration * duration),
                )
                maximum_jerk = max(
                    maximum_jerk,
                    jerk_factor * length / (duration ** 3),
                )
                for scalar in acceleration_values:
                    acceleration = scalar * displacement / (duration * duration)
                    required = acceleration + np.asarray([0.0, 0.0, gravity])
                    tilt = math.degrees(math.atan2(
                        float(np.linalg.norm(required[:2])), float(required[2])
                    ))
                    specific = float(np.linalg.norm(required)) / gravity
                    maximum_tilt = max(maximum_tilt, tilt)
                    minimum_specific_thrust = min(minimum_specific_thrust, specific)
                    maximum_specific_thrust = max(maximum_specific_thrust, specific)

        self.assertAlmostEqual(maximum_speed, 1.9088891, places=5)
        self.assertAlmostEqual(maximum_acceleration, 0.7043614, places=5)
        self.assertAlmostEqual(maximum_jerk, 1.2304688, places=5)
        self.assertAlmostEqual(maximum_tilt, 3.3110343, places=5)
        self.assertAlmostEqual(minimum_specific_thrust, 0.9281997, places=5)
        self.assertAlmostEqual(maximum_specific_thrust, 1.0718003, places=5)
        self.assertLessEqual(maximum_speed, float(contract["max_speed_mps"]))
        self.assertLessEqual(
            maximum_acceleration, float(contract["max_acceleration_mps2"])
        )
        self.assertLessEqual(maximum_jerk, float(contract["max_jerk_mps3"]))
        self.assertLessEqual(maximum_tilt, float(contract["max_tilt_deg"]))
        self.assertGreaterEqual(
            minimum_specific_thrust, float(contract["specific_thrust_min_g"])
        )
        self.assertLessEqual(
            maximum_specific_thrust, float(contract["specific_thrust_max_g"])
        )
        self.assertEqual(float(contract["max_yaw_rate_radps"]), 0.0)
        for phase in self.scenario["phases"]:
            self.assertTrue(all(float(pose["yaw"]) == 0.0
                                for pose in phase.get("targets", {}).values()))

    def test_continuous_avoidance_and_workspace_certificate(self):
        contract = self.scenario["crossing_contract"]
        horizontal, half_height, sphere = _collision_envelope()
        self.assertAlmostEqual(
            sphere, float(contract["conservative_collision_radius_m"]), places=12
        )
        global_minimum = math.inf
        observer_minimum = math.inf
        maximum_range = 0.0
        minimum_ground = math.inf
        elevation_minimum = math.inf
        elevation_maximum = -math.inf

        for _, starts, goals in self.phase_segments:
            for first, second in itertools.combinations(self.names, 2):
                global_minimum = min(
                    global_minimum,
                    _minimum_segment_distance(
                        starts[first], goals[first], starts[second], goals[second]
                    ),
                )
            for name in self.names:
                observer_minimum = min(
                    observer_minimum,
                    _minimum_segment_distance(
                        starts[name], goals[name], SENSOR_ORIGIN, SENSOR_ORIGIN
                    ),
                )
                maximum_range = max(
                    maximum_range,
                    float(np.linalg.norm(starts[name] - SENSOR_ORIGIN)),
                    float(np.linalg.norm(goals[name] - SENSOR_ORIGIN)),
                )
                minimum_ground = min(
                    minimum_ground, starts[name][2] - half_height,
                    goals[name][2] - half_height,
                )
                for progress in np.linspace(0.0, 1.0, 101):
                    point = (1.0 - progress) * starts[name] + progress * goals[name]
                    relative = point - SENSOR_ORIGIN
                    elevation = math.degrees(math.atan2(
                        relative[2], float(np.linalg.norm(relative[:2]))
                    ))
                    elevation_minimum = min(elevation_minimum, elevation)
                    elevation_maximum = max(elevation_maximum, elevation)

        clearance = global_minimum - 2.0 * sphere
        self.assertAlmostEqual(global_minimum, 2.1818546, places=6)
        self.assertAlmostEqual(clearance, 0.8397794, places=6)
        self.assertAlmostEqual(observer_minimum, 6.0972209, places=6)
        self.assertAlmostEqual(maximum_range, 10.0093688, places=6)
        self.assertAlmostEqual(minimum_ground, 1.56, places=9)
        self.assertAlmostEqual(elevation_minimum, -5.6473588, places=6)
        self.assertAlmostEqual(elevation_maximum, 18.8018129, places=6)
        self.assertGreaterEqual(
            global_minimum + 1.0e-7,
            float(contract["minimum_center_separation_m"]),
        )
        self.assertGreaterEqual(
            clearance + 1.0e-7,
            float(contract["minimum_conservative_clearance_m"]),
        )
        self.assertGreaterEqual(
            observer_minimum, float(contract["minimum_observer_center_distance_m"])
        )
        self.assertGreaterEqual(
            minimum_ground + 1.0e-9,
            float(contract["minimum_ground_clearance_m"]),
        )
        self.assertLessEqual(maximum_range, float(contract["maximum_sensor_range_m"]))
        self.assertGreaterEqual(
            elevation_minimum, float(contract["sensor_elevation_min_deg"])
        )
        self.assertLessEqual(
            elevation_maximum, float(contract["sensor_elevation_max_deg"])
        )

    def test_three_distinct_heavy_partial_occlusion_pair_schedules(self):
        contract = self.scenario["crossing_contract"]
        horizontal, _, sphere = _collision_envelope()
        expected_azimuth_offset = float(
            contract["occlusion_center_offset_deg"]
        )
        schedules = []
        for stage in contract["occlusion_stages"]:
            positions = self.phase_end[stage["phase"]]
            pairs = []
            for pair in stage["pairs"]:
                near = pair["near"]
                far = pair["far"]
                near_ray = positions[near] - SENSOR_ORIGIN
                far_ray = positions[far] - SENSOR_ORIGIN
                near_azimuth = math.atan2(near_ray[1], near_ray[0])
                far_azimuth = math.atan2(far_ray[1], far_ray[0])
                azimuth_offset = abs(math.atan2(
                    math.sin(far_azimuth - near_azimuth),
                    math.cos(far_azimuth - near_azimuth),
                ))
                self.assertAlmostEqual(
                    math.degrees(azimuth_offset),
                    expected_azimuth_offset,
                    places=7,
                )
                self.assertGreater(float(np.dot(near_ray, far_ray)), 0.0)
                near_range = float(np.linalg.norm(near_ray))
                far_range = float(np.linalg.norm(far_ray))
                self.assertLess(near_range, far_range)

                center_angle = math.acos(np.clip(
                    float(np.dot(near_ray, far_ray))
                    / (near_range * far_range),
                    -1.0,
                    1.0,
                ))
                near_half_angle = math.asin(sphere / near_range)
                far_half_angle = math.asin(sphere / far_range)
                # The conservative angular envelopes overlap, but the far
                # envelope is not fully contained by the near envelope.  This
                # is the geometry contract for heavy *partial* occlusion.
                self.assertGreater(
                    center_angle, abs(near_half_angle - far_half_angle)
                )
                self.assertLess(
                    center_angle, near_half_angle + far_half_angle
                )
                self.assertAlmostEqual(
                    math.degrees(center_angle), 1.7871773, places=6
                )
                center_distance = float(np.linalg.norm(positions[near] - positions[far]))
                surface_gap = center_distance - 2.0 * horizontal
                self.assertAlmostEqual(surface_gap, 0.8519051, places=6)
                self.assertLessEqual(
                    surface_gap,
                    float(contract["maximum_hold_pair_surface_gap_m"]),
                )
                self.assertLess(
                    surface_gap, float(contract["semantic_cluster_tolerance_m"])
                )
                pairs.append((near, far))
            self.assertEqual({pair[0] for pair in pairs},
                             {"uav{}".format(index) for index in range(2, 7)})
            self.assertEqual({pair[1] for pair in pairs},
                             {"uav{}".format(index) for index in range(7, 12)})
            schedules.append(frozenset(pairs))
        self.assertEqual(len(set(schedules)), 3)

    def test_launch_matches_initial_pose_and_model_authority(self):
        launch = ET.parse(PACKAGE / "launch" / "gate_s02.launch").getroot()
        spawns = [node for node in launch.findall("node")
                  if node.attrib.get("type") == "spawn_model"]
        self.assertEqual(len(spawns), 11)
        by_model = {}
        for node in spawns:
            tokens = shlex.split(node.attrib["args"])
            model = tokens[tokens.index("-model") + 1]
            by_model[model] = tokens
        self.assertEqual(set(by_model), {"uav{}".format(index)
                                         for index in range(1, 12)})
        for name in self.names:
            tokens = by_model[name]
            expected = self.starts[name]
            actual = np.asarray([
                float(tokens[tokens.index("-x") + 1]),
                float(tokens[tokens.index("-y") + 1]),
                float(tokens[tokens.index("-z") + 1]),
            ])
            np.testing.assert_allclose(actual, expected, atol=1.0e-12)
            self.assertIn("/models/simplified_target_uav/model.sdf", " ".join(tokens))


if __name__ == "__main__":
    unittest.main()
