#!/usr/bin/env python3

import math
from pathlib import Path
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

from mid360_spherical_capability_probe.probe_core import (
    CapabilityEvaluator,
    DiagnosticRayFramePairer,
    ProbeThresholds,
    RestartEpochGate,
    TrialAccumulator,
    driver_diagnostics_runtime_safe,
    diagnostic_ray_frame_metadata_matches,
    runtime_promotion_failure_reason,
)


def qualifying_trial(
        trial_id="restart_01", firmware="1.2.3.4",
        restart_marker="powerup_cnt:10", alignment_start=0,
        alignment_end=0, spherical_packet_end=102):
    thresholds = ProbeThresholds(
        minimum_packet_count=2,
        minimum_sample_count=4,
        minimum_zero_depth_count=2,
        minimum_nontrivial_angle_ratio=1.0,
        minimum_temporal_continuity_ratio=1.0,
        maximum_neighbor_angle_rad=0.2,
        required_restart_trials=2,
    )
    trial = TrialAccumulator(trial_id, thresholds)
    trial.update_driver_diagnostics({
        "device": "Mid360",
        "serial_number": "47MDL3C0012345",
        "firmware": firmware,
        "device_restart_marker": restart_marker,
        "pcl_data_type_3_accepted": "true",
        "spherical_packet_confirmed": "true",
        "timestamp_metadata_valid": "true",
        "timestamp_synchronized": "true",
        "diagnostic_header_stamp_matches_frame_stamp": "true",
        "diagnostic_ray_frame_metadata_matches": "true",
        "frame_alignment_exact": str(alignment_start == 0).lower(),
        "frame_alignment_mismatch_count": str(alignment_start),
        "single_device_stream": "true",
        "packet_count": "100",
        "spherical_packet_count": "100",
        "sample_count": "1000",
    })
    trial.update_driver_diagnostics({
        "packet_count": "102",
        "spherical_packet_count": str(spherical_packet_end),
        "sample_count": "1004",
        "frame_alignment_mismatch_count": str(alignment_end),
    })
    angles = (0.10, 0.11, 0.12, 0.13)
    for index, angle in enumerate(angles):
        direction = (math.sin(angle), 0.0, math.cos(angle))
        status = 0 if index in (1, 2) else 1
        trial.observe_sample(status, direction, 1000 + index)
    return thresholds, trial.finish()


class ProbeCoreTest(unittest.TestCase):
    def test_hardware_pipeline_feeds_pointcloud2_preprocessor(self):
        package_root = Path(__file__).resolve().parents[1]
        pipeline = ET.parse(
            str(package_root / "launch" / "hardware_spherical_pipeline.launch")
        ).getroot()
        pipeline_args = {
            element.attrib["name"]: element.attrib.get("default")
            for element in pipeline.findall("arg")
        }
        self.assertEqual(pipeline_args["xfer_format"], "0")
        driver_include = next(
            element for element in pipeline.findall("include")
            if "msg_MID360_spherical_rays.launch" in element.attrib["file"]
        )
        include_args = {
            element.attrib["name"]: element.attrib["value"]
            for element in driver_include.findall("arg")
        }
        self.assertEqual(include_args["xfer_format"], "$(arg xfer_format)")

        pairer = DiagnosticRayFramePairer(capacity=4)
        self.assertIsNone(pairer.add_ray_bundle(100, "old_ray"))
        self.assertIsNone(pairer.add_diagnostics(
            200, "powerup_cnt:11", {"frame": "new"}
        ))
        new_pair = pairer.add_ray_bundle(200, "new_ray")
        self.assertEqual(new_pair["marker"], "powerup_cnt:11")
        self.assertEqual(new_pair["ray_bundle"], "new_ray")
        old_pair = pairer.add_diagnostics(
            100, "powerup_cnt:10", {"frame": "old"}
        )
        self.assertEqual(old_pair["marker"], "powerup_cnt:10")
        self.assertEqual(old_pair["ray_bundle"], "old_ray")
        self.assertIsNone(pairer.add_ray_bundle(200, "duplicate"))
        bundle = SimpleNamespace(
            scan_id=17,
            pattern_start_index=300,
            rays=[object(), object()],
            header=SimpleNamespace(stamp=SimpleNamespace(to_nsec=lambda: 900)),
        )
        self.assertTrue(diagnostic_ray_frame_metadata_matches({
            "frame_scan_id": "17",
            "frame_pattern_start_index": "300",
            "frame_ray_count": "2",
            "frame_stamp_ns": "900",
        }, bundle))
        bundle.scan_id = 18
        self.assertFalse(diagnostic_ray_frame_metadata_matches({
            "frame_scan_id": "17",
            "frame_pattern_start_index": "300",
            "frame_ray_count": "2",
            "frame_stamp_ns": "900",
        }, bundle))
        pairer.clear_pending()

    def test_restart_epoch_gate_waits_for_a_distinct_boot(self):
        gate = RestartEpochGate()
        self.assertEqual(gate.observe("UNAVAILABLE"), gate.IGNORE)
        self.assertEqual(gate.observe("powerup_cnt:10"), gate.START)
        self.assertEqual(gate.observe("powerup_cnt:10"), gate.CONTINUE)
        gate.finish("powerup_cnt:10")
        self.assertTrue(gate.waiting_for_new_boot)
        self.assertEqual(gate.observe("powerup_cnt:10"), gate.IGNORE)
        self.assertEqual(gate.observe("powerup_cnt:11"), gate.START)
        self.assertFalse(gate.waiting_for_new_boot)
        self.assertEqual(gate.active_marker, "powerup_cnt:11")
        self.assertEqual(gate.observe("powerup_cnt:10"), gate.IGNORE)
        self.assertEqual(gate.active_marker, "powerup_cnt:11")

    def test_two_qualifying_restart_trials_enable_exact_mode(self):
        thresholds, first = qualifying_trial("restart_01")
        _, second = qualifying_trial(
            "restart_02", restart_marker="powerup_cnt:11"
        )
        evaluator = CapabilityEvaluator(thresholds)
        evaluator.add_trial(first)
        evaluator.add_trial(second)
        report = evaluator.report()
        self.assertTrue(report["exact_no_return_direction_supported"])
        self.assertEqual(report["ray_source_mode"], "hw_spherical_exact")
        self.assertEqual(report["restart_trial_count"], 2)

    def test_single_trial_fails_closed_to_calibrated_fallback(self):
        thresholds, trial = qualifying_trial()
        evaluator = CapabilityEvaluator(thresholds)
        evaluator.add_trial(trial)
        report = evaluator.report()
        self.assertFalse(report["exact_no_return_direction_supported"])
        self.assertEqual(report["ray_source_mode"], "calibrated_fallback")
        self.assertIn(
            "insufficient_independent_restart_trials", report["failure_reasons"]
        )

    def test_trivial_zero_depth_angles_are_rejected(self):
        thresholds, _ = qualifying_trial()
        trial = TrialAccumulator("trivial", thresholds)
        trial.update_driver_diagnostics({
            "device": "Mid360",
            "serial_number": "47MDL3C0012345",
            "firmware": "1.2.3.4",
            "device_restart_marker": "powerup_cnt:10",
            "pcl_data_type_3_accepted": "true",
            "spherical_packet_confirmed": "true",
            "timestamp_metadata_valid": "true",
            "timestamp_synchronized": "true",
            "diagnostic_header_stamp_matches_frame_stamp": "true",
            "diagnostic_ray_frame_metadata_matches": "true",
            "frame_alignment_exact": "true",
            "frame_alignment_mismatch_count": "0",
            "single_device_stream": "true",
            "packet_count": "100",
            "spherical_packet_count": "100",
            "sample_count": "1000",
        })
        trial.update_driver_diagnostics({
            "packet_count": "102",
            "spherical_packet_count": "102",
            "sample_count": "1004",
        })
        for index in range(4):
            trial.observe_sample(0 if index > 0 else 1, (0.0, 0.0, 1.0), index)
        result = trial.finish()
        self.assertFalse(result["qualifies"])
        self.assertIn("zero_depth_angles_trivial", result["failure_reasons"])

    def test_command_rejection_is_preserved(self):
        thresholds, _ = qualifying_trial()
        trial = TrialAccumulator("rejected", thresholds)
        trial.update_driver_diagnostics({
            "device": "Mid360",
            "serial_number": "47MDL3C0012345",
            "firmware": "1.2.3.4",
            "device_restart_marker": "powerup_cnt:10",
            "pcl_data_type_3_accepted": "false",
            "spherical_packet_confirmed": "true",
            "timestamp_metadata_valid": "true",
            "timestamp_synchronized": "true",
            "diagnostic_header_stamp_matches_frame_stamp": "true",
            "diagnostic_ray_frame_metadata_matches": "true",
            "frame_alignment_exact": "true",
            "frame_alignment_mismatch_count": "0",
            "single_device_stream": "true",
            "packet_count": "100",
            "spherical_packet_count": "100",
            "sample_count": "1000",
        })
        trial.update_driver_diagnostics({
            "packet_count": "102",
            "spherical_packet_count": "102",
            "sample_count": "1004",
        })
        for index in range(4):
            angle = 0.1 + index * 0.01
            trial.observe_sample(
                0 if index > 0 else 1,
                (math.sin(angle), 0.0, math.cos(angle)),
                index,
            )
        result = trial.finish()
        self.assertFalse(result["qualifies"])
        self.assertIn("pcl_data_type_3_not_accepted", result["failure_reasons"])

    def test_duplicate_trial_identifier_is_rejected(self):
        thresholds, trial = qualifying_trial("same")
        evaluator = CapabilityEvaluator(thresholds)
        evaluator.add_trial(trial)
        with self.assertRaises(ValueError):
            evaluator.add_trial(trial)

    def test_reused_boot_marker_cannot_enable_exact_mode(self):
        thresholds, first = qualifying_trial("restart_01")
        _, second = qualifying_trial("restart_02")
        evaluator = CapabilityEvaluator(thresholds)
        evaluator.add_trial(first)
        evaluator.add_trial(second)
        report = evaluator.report()
        self.assertFalse(report["exact_no_return_direction_supported"])
        self.assertIn(
            "device_restart_not_observed_between_trials",
            report["failure_reasons"],
        )

    def test_fixed_nontrivial_zero_depth_direction_is_rejected(self):
        thresholds, _ = qualifying_trial()
        trial = TrialAccumulator("fixed", thresholds)
        trial.update_driver_diagnostics({
            "device": "Mid360",
            "serial_number": "47MDL3C0012345",
            "firmware": "1.2.3.4",
            "device_restart_marker": "powerup_cnt:10",
            "pcl_data_type_3_accepted": "true",
            "spherical_packet_confirmed": "true",
            "timestamp_metadata_valid": "true",
            "timestamp_synchronized": "true",
            "diagnostic_header_stamp_matches_frame_stamp": "true",
            "diagnostic_ray_frame_metadata_matches": "true",
            "frame_alignment_exact": "true",
            "frame_alignment_mismatch_count": "0",
            "single_device_stream": "true",
            "packet_count": "100",
            "spherical_packet_count": "100",
            "sample_count": "1000",
        })
        trial.update_driver_diagnostics({
            "packet_count": "102",
            "spherical_packet_count": "102",
            "sample_count": "1004",
        })
        direction = (math.sin(0.1), 0.0, math.cos(0.1))
        for index in range(4):
            trial.observe_sample(0 if index > 0 else 1, direction, index)
        result = trial.finish()
        self.assertFalse(result["qualifies"])
        self.assertIn(
            "zero_depth_angles_not_changing", result["failure_reasons"]
        )

    def test_runtime_diagnostics_fail_closed(self):
        values = {
            "device": "Mid360",
            "serial_number": "47MDL3C0012345",
            "firmware": "1.2.3.4",
            "device_restart_marker": "powerup_cnt:10",
            "pcl_data_type_3_accepted": "true",
            "spherical_packet_confirmed": "true",
            "timestamp_metadata_valid": "true",
            "timestamp_synchronized": "true",
            "diagnostic_header_stamp_matches_frame_stamp": "true",
            "diagnostic_ray_frame_metadata_matches": "true",
            "frame_alignment_exact": "true",
            "frame_alignment_mismatch_count": "0",
            "single_device_stream": "true",
        }
        self.assertTrue(driver_diagnostics_runtime_safe(values))
        values["frame_alignment_exact"] = "false"
        self.assertFalse(driver_diagnostics_runtime_safe(values))
        values["frame_alignment_exact"] = "true"
        values["firmware"] = "UNAVAILABLE"
        self.assertFalse(driver_diagnostics_runtime_safe(values))
        values["firmware"] = "1.2.3.4"
        self.assertIsNone(runtime_promotion_failure_reason(
            values, 10.0, 9.5, 9.5, 9.5, 2.0, 2.0
        ))
        self.assertEqual(runtime_promotion_failure_reason(
            values, 12.1, 9.5, 11.9, 11.9, 2.0, 2.0
        ), "driver_diagnostics_timeout")
        self.assertEqual(runtime_promotion_failure_reason(
            values, 10.0, 9.9, 9.9, None, 2.0, 2.0
        ), "paired_driver_ray_frame_timeout")
        values["timestamp_synchronized"] = "false"
        self.assertEqual(runtime_promotion_failure_reason(
            values, 10.0, 9.9, 9.9, 9.9, 2.0, 2.0
        ), "runtime_driver_capability_revoked")

    def test_alignment_uses_trial_delta_not_process_lifetime(self):
        _, trial = qualifying_trial(
            alignment_start=7, alignment_end=7
        )
        self.assertTrue(trial["frame_alignment_exact"])
        self.assertTrue(trial["qualifies"])

    def test_raw_packets_cannot_substitute_for_spherical_packets(self):
        _, trial = qualifying_trial(spherical_packet_end=100)
        self.assertFalse(trial["qualifies"])
        self.assertIn("insufficient_packet_count", trial["failure_reasons"])

    def test_runtime_identity_is_bound_to_authenticated_device(self):
        values = {
            "device": "Mid360",
            "serial_number": "47MDL3C0012345",
            "firmware": "1.2.3.4",
            "device_restart_marker": "powerup_cnt:11",
            "pcl_data_type_3_accepted": "true",
            "spherical_packet_confirmed": "true",
            "timestamp_metadata_valid": "true",
            "timestamp_synchronized": "true",
            "diagnostic_header_stamp_matches_frame_stamp": "true",
            "diagnostic_ray_frame_metadata_matches": "true",
            "single_device_stream": "true",
            "frame_alignment_mismatch_count": "7",
        }
        self.assertTrue(driver_diagnostics_runtime_safe(
            values, 7, "47MDL3C0012345", "1.2.3.4", "powerup_cnt:11"
        ))
        values["serial_number"] = "47MDL3C0099999"
        self.assertFalse(driver_diagnostics_runtime_safe(
            values, 7, "47MDL3C0012345", "1.2.3.4", "powerup_cnt:11"
        ))

    def test_equal_timestamps_fail_zero_valid_continuity(self):
        thresholds = ProbeThresholds(
            minimum_packet_count=2,
            minimum_sample_count=4,
            minimum_zero_depth_count=2,
            minimum_nontrivial_angle_ratio=1.0,
            minimum_temporal_continuity_ratio=1.0,
            maximum_neighbor_angle_rad=0.2,
            required_restart_trials=2,
        )
        trial = TrialAccumulator("same_stamp", thresholds)
        trial.update_driver_diagnostics({
            "device": "Mid360",
            "serial_number": "47MDL3C0012345",
            "firmware": "1.2.3.4",
            "device_restart_marker": "powerup_cnt:10",
            "pcl_data_type_3_accepted": "true",
            "spherical_packet_confirmed": "true",
            "timestamp_metadata_valid": "true",
            "timestamp_synchronized": "true",
            "diagnostic_header_stamp_matches_frame_stamp": "true",
            "diagnostic_ray_frame_metadata_matches": "true",
            "single_device_stream": "true",
            "packet_count": "100",
            "spherical_packet_count": "100",
            "sample_count": "1000",
            "frame_alignment_mismatch_count": "0",
        })
        trial.update_driver_diagnostics({
            "packet_count": "102",
            "spherical_packet_count": "102",
            "sample_count": "1004",
            "frame_alignment_mismatch_count": "0",
        })
        for index, status in enumerate((1, 0, 0, 1)):
            angle = 0.1 + index * 0.01
            trial.observe_sample(
                status, (math.sin(angle), 0.0, math.cos(angle)), 1000
            )
        result = trial.finish()
        self.assertFalse(result["qualifies"])
        self.assertIn(
            "angle_temporal_continuity_failed", result["failure_reasons"]
        )

    def test_restart_marker_change_inside_trial_is_rejected(self):
        thresholds = ProbeThresholds(
            minimum_packet_count=1,
            minimum_sample_count=2,
            minimum_zero_depth_count=1,
            minimum_nontrivial_angle_ratio=1.0,
            minimum_temporal_continuity_ratio=1.0,
            maximum_neighbor_angle_rad=0.2,
            required_restart_trials=2,
        )
        trial = TrialAccumulator("mixed_boot", thresholds)
        base = {
            "device": "Mid360",
            "serial_number": "47MDL3C0012345",
            "firmware": "1.2.3.4",
            "device_restart_marker": "powerup_cnt:10",
            "pcl_data_type_3_accepted": "true",
            "spherical_packet_confirmed": "true",
            "timestamp_metadata_valid": "true",
            "timestamp_synchronized": "true",
            "diagnostic_header_stamp_matches_frame_stamp": "true",
            "diagnostic_ray_frame_metadata_matches": "true",
            "single_device_stream": "true",
            "packet_count": "10",
            "spherical_packet_count": "10",
            "sample_count": "100",
            "frame_alignment_mismatch_count": "0",
        }
        trial.update_driver_diagnostics(base)
        changed = dict(base)
        changed.update({
            "device_restart_marker": "powerup_cnt:11",
            "packet_count": "11",
            "spherical_packet_count": "11",
            "sample_count": "102",
        })
        trial.update_driver_diagnostics(changed)
        for index, status in enumerate((1, 0)):
            angle = 0.1 + index * 0.01
            trial.observe_sample(
                status, (math.sin(angle), 0.0, math.cos(angle)), index + 1
            )
        result = trial.finish()
        self.assertFalse(result["qualifies"])
        self.assertIn(
            "device_restart_marker_changed_within_trial",
            result["failure_reasons"],
        )

        identity_trial = TrialAccumulator("mixed_identity", thresholds)
        identity_trial.update_driver_diagnostics(base)
        changed_identity = dict(base)
        changed_identity.update({
            "serial_number": "47MDL3C0099999",
            "packet_count": "11",
            "spherical_packet_count": "11",
            "sample_count": "102",
        })
        identity_trial.update_driver_diagnostics(changed_identity)
        for index, status in enumerate((1, 0)):
            angle = 0.1 + index * 0.01
            identity_trial.observe_sample(
                status, (math.sin(angle), 0.0, math.cos(angle)), index + 1
            )
        identity_result = identity_trial.finish()
        self.assertFalse(identity_result["qualifies"])
        self.assertIn(
            "device_identity_changed_within_trial",
            identity_result["failure_reasons"],
        )


if __name__ == "__main__":
    unittest.main()
