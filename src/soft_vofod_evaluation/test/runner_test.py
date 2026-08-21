#!/usr/bin/env python3

import importlib.util
import os
import tempfile
import unittest

import yaml


ROOT = os.path.dirname(os.path.dirname(__file__))
SPEC = importlib.util.spec_from_file_location(
    "run_benchmark", os.path.join(ROOT, "scripts", "run_benchmark.py"))
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)


class RunnerTest(unittest.TestCase):
    def test_truth_configuration_is_evaluation_only_and_complete(self):
        scenario = {
            "scenario_id": "test", "world": "E2_occlusion_arena",
            "targets": [{"id": "uav2"}, {"id": "uav3"}],
        }
        config = RUNNER.truth_configuration(scenario, "snapshot")
        self.assertEqual(config["output_topic"], "/evaluation/visibility_ground_truth")
        self.assertEqual(len(config["truth_sources"]), 3)
        self.assertEqual(len(config["targets"]), 2)
        self.assertTrue(config["static_primitives"])
        for target in config["targets"]:
            self.assertEqual(len(target["primitives"]), 7)

    def test_observer_generation_changes_only_requested_sensor_knobs(self):
        source = """<sdf><stddev>0.0</stddev><ray_time_geometry_mode>per_ray_pose</ray_time_geometry_mode></sdf>"""
        with tempfile.TemporaryDirectory() as directory:
            source_path = os.path.join(directory, "source.sdf")
            destination = os.path.join(directory, "generated.sdf")
            with open(source_path, "w", encoding="utf-8") as stream:
                stream.write(source)
            RUNNER.generated_observer(source_path, destination, "snapshot", 0.02)
            with open(destination, encoding="utf-8") as stream:
                generated = stream.read()
            self.assertIn("<stddev>0.02</stddev>", generated)
            self.assertIn("<ray_time_geometry_mode>snapshot</ray_time_geometry_mode>", generated)

    def test_cli_value_validation(self):
        self.assertEqual(RUNNER.comma_list("A1,A3", RUNNER.ALGORITHMS), ("A1", "A3"))
        with self.assertRaises(Exception):
            RUNNER.comma_list("A4", RUNNER.ALGORITHMS)

    def test_calibration_overlay_selects_the_last_explicit_value(self):
        with tempfile.TemporaryDirectory() as directory:
            canonical = os.path.join(directory, "canonical.yaml")
            overlay = os.path.join(directory, "overlay.yaml")
            with open(canonical, "w", encoding="utf-8") as stream:
                yaml.safe_dump({"birth": {"min_groups": 3}}, stream)
            with open(overlay, "w", encoding="utf-8") as stream:
                yaml.safe_dump({"birth": {"min_groups": 5}}, stream)
            self.assertEqual(RUNNER.overlaid_value(
                (canonical, overlay), "birth", "min_groups"), 5)

    def test_master_port_uses_ros_master_uri(self):
        previous = os.environ.get("ROS_MASTER_URI")
        try:
            os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:11442"
            self.assertEqual(RUNNER.master_port(), 11442)
        finally:
            if previous is None:
                os.environ.pop("ROS_MASTER_URI", None)
            else:
                os.environ["ROS_MASTER_URI"] = previous

    def test_input_readiness_and_run_level_warmup_gate(self):
        state = ([], [
            ("/points", ["/soft_vofod"]),
            ("/rays", ["/soft_vofod"]),
        ], [])
        required = {"/points": "/soft_vofod", "/rays": "/soft_vofod"}
        self.assertTrue(RUNNER.subscriptions_ready(state, required))
        self.assertFalse(RUNNER.subscriptions_ready(
            ([], [("/points", ["/soft_vofod"])], []), required))
        recorder_state = ([], [
            ("/tracks", ["/record_123"]),
            ("/diagnostics", ["/record_123"]),
        ], [])
        self.assertTrue(RUNNER.recorder_subscriptions_ready(
            recorder_state, ("/tracks", "/diagnostics")))
        self.assertFalse(RUNNER.recorder_subscriptions_ready(
            recorder_state, ("/tracks", "/missing")))

        source = {
            "first_checked_ray_stamp": 4.2,
            "first_scored_input_stamp": 14.7,
            "first_target_spawn_stamp": 14.65,
        }
        evidence = {
            "first_input_ack_stamp": 4.2,
            "background_warmup_complete_stamp": 14.2,
            "first_scored_warmup_active": False,
        }
        RUNNER.validate_run_timing("B0", evidence, source)
        evidence["background_warmup_complete_stamp"] = 15.4
        with self.assertRaises(RUNNER.RunContractError) as raised:
            RUNNER.validate_run_timing("B0", evidence, source)
        self.assertEqual(raised.exception.status, "INVALID_WARMUP")
        evidence.update({
            "first_input_ack_stamp": 4.3,
            "background_warmup_complete_stamp": 14.2,
        })
        with self.assertRaises(RUNNER.RunContractError) as raised:
            RUNNER.validate_run_timing("B0", evidence, source)
        self.assertEqual(raised.exception.status, "INVALID_INPUT_HANDSHAKE")

    def test_aggregate_results_writes_group_and_paired_delta(self):
        def metrics(hota):
            return {
                "scenario": "test", "track_set": {"HOTA": hota},
                "coverage": {"track_frame_ratio": 1.0},
            }
        with tempfile.TemporaryDirectory() as directory:
            for algorithm, hota in (("A2", 0.4), ("A3", 0.7)):
                run = os.path.join(directory, "runs", algorithm, "S01", "N0",
                                   "seed_1")
                os.makedirs(run)
                with open(os.path.join(run, "metrics.json"), "w",
                          encoding="utf-8") as stream:
                    import json
                    json.dump(metrics(hota), stream)
            rows = RUNNER.aggregate_results(directory)
            self.assertEqual(len(rows), 2)
            with open(os.path.join(directory, "metrics", "aggregate.json"),
                      encoding="utf-8") as stream:
                import json
                aggregate = json.load(stream)
            self.assertEqual(aggregate["groups"]["A3/S01/N0"]["HOTA"]["mean"], 0.7)
            with open(os.path.join(directory, "metrics", "ablation_deltas.csv"),
                      encoding="utf-8") as stream:
                self.assertIn("0.299999", stream.read())


if __name__ == "__main__":
    unittest.main()
