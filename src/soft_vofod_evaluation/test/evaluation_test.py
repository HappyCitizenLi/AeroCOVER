#!/usr/bin/env python3

import importlib.util
import os
import tempfile
import unittest


ROOT = os.path.dirname(os.path.dirname(__file__))
SPEC = importlib.util.spec_from_file_location(
    "evaluate_bag", os.path.join(ROOT, "scripts", "evaluate_bag.py"))
METRICS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(METRICS)


def target(identifier, x, velocity=(0.0, 0.0, 0.0)):
    return {"id": identifier, "position": (x, 0.0, 0.0),
            "velocity": velocity}


class EvaluationTest(unittest.TestCase):
    def test_hungarian_metrics_are_one_to_one_and_report_identity_switch(self):
        frames = [
            {"time": 0.0, "observer": (0.0, 0.0, 0.0),
             "truth": [target("a", 1.0), target("b", 3.0)],
             "predictions": [target(10, 1.0), target(20, 3.0)]},
            {"time": 1.0, "observer": (0.0, 0.0, 0.0),
             "truth": [target("a", 1.0), target("b", 3.0)],
             "predictions": [target(20, 1.0), target(10, 3.0)]},
        ]
        result = METRICS.set_metrics(frames, 1.0)
        self.assertEqual(result["tp"], 4)
        self.assertEqual(result["fp"], 0)
        self.assertEqual(result["fn"], 0)
        self.assertEqual(result["id_switches"], 2)
        self.assertEqual(result["DetA"], 1.0)
        self.assertLess(result["AssA"], 1.0)
        self.assertLess(result["IDF1"], 1.0)

    def test_fragment_ttft_gospa_and_range_bins_are_finite(self):
        frames = [
            {"time": 0.0, "observer": (0.0, 0.0, 0.0),
             "truth": [target("a", 5.0)], "predictions": []},
            {"time": 1.0, "observer": (0.0, 0.0, 0.0),
             "truth": [target("a", 5.0)], "predictions": [target(1, 5.1)]},
            {"time": 2.0, "observer": (0.0, 0.0, 0.0),
             "truth": [target("a", 5.0)], "predictions": []},
            {"time": 3.0, "observer": (0.0, 0.0, 0.0),
             "truth": [target("a", 5.0)], "predictions": [target(1, 5.0)]},
        ]
        result = METRICS.set_metrics(frames, 1.0)
        self.assertAlmostEqual(result["TTFT_mean_s"], 1.0)
        self.assertEqual(result["fragmentations"], 1)
        self.assertGreater(result["GOSPA_mean"], 0.0)
        self.assertIn("0-10", result["range_position_RMSE_m"])

    def test_opportunity_calibration_and_map_contamination(self):
        opportunity = METRICS.opportunity_metrics([
            (0.0, [{"track_id": 1, "pd": 0.8, "effective": 4.0, "matched": True},
                   {"track_id": 2, "pd": 0.0, "effective": 0.0, "matched": False}]),
            (0.1, [{"track_id": 1, "pd": 0.8, "effective": 4.0,
                    "matched": False}])], detector_stamps=[0.0])
        self.assertLess(opportunity["Brier"], 0.03)
        self.assertEqual(opportunity["no_opportunity_samples"], 1)

        truth = [(0.0, [{"position": (1.0, 0.0, 1.0)}]),
                 (1.0, [{"position": (1.5, 0.0, 1.0)}])]
        path, _ = METRICS.target_path_voxels(truth)
        contaminated = next(iter(path))
        result = METRICS.map_metrics(
            [(2.0, {contaminated})], [], [], truth, "E0_open")
        self.assertGreater(result["target_contamination_ratio"], 0.0)
        self.assertLess(result["free_space_retention"], 1.0)

    def test_packet_ghost_and_diagnostic_metrics(self):
        truth = [(0.0, [{"position": (1.0, 0.0, 0.0),
                         "actual_returns": 1}])]
        packets = [(0.0, [
            {"position": (1.0, 0.0, 0.0), "point_count": 4},
            {"position": (8.0, 0.0, 0.0), "point_count": 1},
        ])]
        event = METRICS.event_metrics(packets, truth, 60.0)
        self.assertEqual(event["event_count"], 2)
        self.assertEqual(event["raw_anomaly_points_in_packets"], 5)
        self.assertEqual(event["packet_singleton_ratio"], 0.5)

        tracks = [
            {"stamp": 1.0, "id": 1, "state": "tentative",
             "existence": 0.6, "stale_s": 0.0},
            {"stamp": 2.0, "id": 1, "state": "confirmed",
             "existence": 0.8, "stale_s": 1.2},
        ]
        health = METRICS.track_health_metrics(tracks, 60.0, no_target=True)
        self.assertEqual(health["false_confirmed_tracks_per_min"], 1.0)
        self.assertEqual(health["confirmed_stale_samples"]["1.0"], 1)
        self.assertEqual(health["confirmed_stale_unique_tracks"]["1.0"], 1)
        diagnostics = METRICS.diagnostics_metrics([
            (1.0, {"processing_ms": 10.0, "track_count": 2.0}),
            (2.0, {"processing_ms": 20.0, "track_count": 3.0}),
        ])
        self.assertEqual(
            diagnostics["module_runtime_ms"]["processing_ms"]["p50"], 15.0)
        self.assertEqual(diagnostics["complexity"]["track_count"]["max"], 3.0)

    def test_resource_parser(self):
        with tempfile.NamedTemporaryFile("w", delete=False) as stream:
            stream.write("User time (seconds): 2.5\n")
            stream.write("System time (seconds): 0.5\n")
            stream.write("Percent of CPU this job got: 75%\n")
            stream.write("Elapsed (wall clock) time (h:mm:ss or m:ss): 0:04.00\n")
            stream.write("Maximum resident set size (kbytes): 12345\n")
            path = stream.name
        try:
            self.assertEqual(METRICS.parse_resource(path), 12345)
            parsed = METRICS.parse_resource_metrics(path)
            self.assertEqual(parsed["user_cpu_s"], 2.5)
            self.assertEqual(parsed["wall_elapsed_s"], 4.0)
        finally:
            os.unlink(path)

    def test_clutter_static_geometry_includes_walls_and_pillars(self):
        voxels = METRICS.static_voxels("E3_cluttered")
        self.assertIn(METRICS.quantize((18.0, 0.0, 4.0)), voxels)
        self.assertIn(METRICS.quantize((4.0, 8.0, 4.0)), voxels)
        self.assertIn(METRICS.quantize((5.6, 0.0, 2.0)), voxels)


if __name__ == "__main__":
    unittest.main()
