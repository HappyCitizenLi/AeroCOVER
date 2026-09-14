#!/usr/bin/env python3

import importlib.util
import json
import pathlib
import tempfile
import unittest
import yaml


ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "run_benchmark", ROOT / "scripts/run_benchmark.py")
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)
PAPER_SPEC = importlib.util.spec_from_file_location(
    "run_paper_minimal", ROOT / "scripts/run_paper_minimal.py")
PAPER = importlib.util.module_from_spec(PAPER_SPEC)
PAPER_SPEC.loader.exec_module(PAPER)


class RunnerTest(unittest.TestCase):
    def test_twenty_run_matrix_and_v2_tracker_override(self):
        import sys
        sys.path.insert(0,str(ROOT/'scripts'))
        from run_twenty_scene_suite import matrix
        rows=matrix()
        self.assertEqual(len(rows),20)
        self.assertEqual(len({(s,l) for s,l,*_ in rows}),20)
        self.assertEqual({s:sum(r[0]==s for r in rows) for s in ('OPEN','MT','OFFICE','FOREST')},
                         dict(OPEN=4,MT=4,OFFICE=6,FOREST=6))
        for scene,label,method,config,override in rows:
            self.assertTrue(config.exists())
            if label.endswith('-V2'):
                self.assertTrue(override.exists())
                detector=yaml.safe_load(config.read_text())
                self.assertEqual(detector['clustering']['tolerance'],1.5)
                self.assertEqual(detector['clustering']['max_size'],3.)

    def test_open_tracker_override_is_explicit(self):
        command=RUNNER.algorithm_command('VoFOD-Mid360','detector.yaml','open_tracker.yaml')
        self.assertIn('tracker_override_config:=open_tracker.yaml',command)
        self.assertFalse(any('tracker_override' in arg for arg in
            RUNNER.algorithm_command('VoFOD-Mid360','detector.yaml')))

    def test_scene_acceptance_is_bound_to_input_and_audit(self):
        import sys
        sys.path.insert(0,str(ROOT/'scripts'))
        from run_four_scene_suite import require_accepted_source
        with tempfile.TemporaryDirectory() as directory:
            root=pathlib.Path(directory)
            source=root/'sources/OPEN';source.mkdir(parents=True)
            (source/'source_manifest.json').write_text('{}')
            (source/'flight_audit.json').write_text('{"status":"FAIL"}')
            with self.assertRaises(RuntimeError):
                require_accepted_source(root,'OPEN',{'status':'FAIL'})
            accepted={'OPEN':dict(status='ACCEPTED_BY_USER',
                source_manifest_sha256=RUNNER.sha256(source/'source_manifest.json'),
                flight_audit_sha256=RUNNER.sha256(source/'flight_audit.json'))}
            (root/'accepted_sources.json').write_text(json.dumps(accepted))
            require_accepted_source(root,'OPEN',{'status':'FAIL'})
            (source/'flight_audit.json').write_text('{"status":"FAIL","changed":true}')
            with self.assertRaises(RuntimeError):
                require_accepted_source(root,'OPEN',{'status':'FAIL'})

    def test_recorder_message_loss_invalidates_measurement(self):
        with tempfile.TemporaryDirectory() as directory:
            log = pathlib.Path(directory)/'recorder.log'
            log.write_text('Recording to output.bag\n')
            RUNNER.validate_recorder_log(log)
            log.write_text('rosbag record buffer exceeded. Dropping oldest queued message.\n')
            with self.assertRaisesRegex(RuntimeError, 'dropped messages'):
                RUNNER.validate_recorder_log(log)

    def test_current_scope(self):
        self.assertTrue({
            "AeroCOVER-Mid360", "AeroCOVER-OS1",
            "VoFOD-Mid360-Adapted",
            "VoFOD-Original-OS1", "VoFOD-Mid360-Adapted-OS1",
            "AeroCOVER-A1", "AeroCOVER-A2",
            "AeroCOVER-A3", "AeroCOVER-A4", "AeroCOVER-A5",
        }.issubset(RUNNER.ALGORITHMS))
        self.assertEqual(
            RUNNER.SCENES,
            ("S1_near", "S1_far", "P01", "S2_new", "P02", "S3_new",
             "M1", "M2", "OPEN", "MT", "OFFICE", "FOREST"))
        self.assertEqual(RUNNER.NOISE_STDDEV_M["N03"], 0.03)
        self.assertEqual(
            set(RUNNER.WORLD_FILES),
            {"PW_open", "PW_office", "PW_forest_seed0", "PW_mt"})

    def test_current_assets(self):
        workspace = ROOT.parents[1]
        for scenario, world in (
                ("S1_near.yaml", "PW_open.world"),
                ("S1_far.yaml", "PW_open.world"),
                ("P01.yaml", "PW_office.world"),
                ("S2_new.yaml", "PW_office.world"),
                ("P02.yaml", "PW_forest_seed0.world"),
                ("S3_new.yaml", "PW_forest_seed0.world"),
                ("M1.yaml", "PW_open.world"),
                ("M2.yaml", "PW_open.world")):
            self.assertTrue((workspace /
                "src/mid360_multi_uav_sim/config/benchmarks" /
                scenario).is_file())
            self.assertTrue((workspace / "src/mid360_multi_uav_sim/worlds" / world).is_file())

    def test_all_paper_trajectories_pass_static_physical_validation(self):
        directory = ROOT.parents[1] / "src/mid360_multi_uav_sim/config/benchmarks"
        for name in RUNNER.SCENES:
            scenario = yaml.safe_load((directory / (name + ".yaml")).read_text())
            result = RUNNER.validate_physical_scenario(scenario)
            self.assertEqual(result["status"], "PASS", name)

    def test_ouster_commands_use_native_and_full_adapters(self):
        config = pathlib.Path("config.yaml")
        self.assertIn("ouster_original.launch", " ".join(
            RUNNER.algorithm_command("VoFOD-Original-OS1", config)))
        self.assertIn("aerocover_os1.launch", " ".join(
            RUNNER.algorithm_command("AeroCOVER-OS1", config)))
        self.assertIn("config:=config.yaml", RUNNER.algorithm_command("AeroCOVER-OS1", config))
        self.assertIn("tracker_radius_min:=0.6", " ".join(
            RUNNER.algorithm_command("VoFOD-Mid360-Adapted", config)))
        self.assertNotIn("VoFOD-Original-Mid360", RUNNER.ALGORITHMS)
        with self.assertRaises(ValueError):
            RUNNER.algorithm_command("VoFOD-Original-Mid360", config)
        adapted_os1 = " ".join(RUNNER.algorithm_command(
            "VoFOD-Mid360-Adapted-OS1", config))
        self.assertIn("ouster_original.launch", adapted_os1)
        self.assertIn("tracker_radius_min:=0.6", adapted_os1)

    def test_reduced_paper_matrix_uses_the_fixed_six_mid360_scenes(self):
        self.assertEqual(
            PAPER.PAIRED_SCENES,
            ("P01", "S2_new", "P02", "S3_new", "M1", "M2"))
        self.assertEqual(len(PAPER.ABLATIONS), 6)
        self.assertEqual(len(PAPER.OUSTER_SCENES), 8)
        self.assertEqual(PAPER.EXPECTED_RUNS, 70)
        self.assertNotIn("VoFOD-Original-Mid360", PAPER.PAPER_METHODS)

    def test_retired_method_is_not_resummarized(self):
        with tempfile.TemporaryDirectory() as directory:
            retired = pathlib.Path(directory) / "runs/S1_near/VoFOD-Original-Mid360"
            retired.mkdir(parents=True)
            (retired / "metrics.json").write_text("{}")
            self.assertEqual(PAPER.read_rows(pathlib.Path(directory)), ([], []))

    def test_ouster_keeps_all_rays_and_bounds_only_the_point_domain(self):
        workspace = ROOT.parents[1]
        config = yaml.safe_load((workspace /
            "src/aerocover_mid360/config/aerocover_os1_128.yaml").read_text())
        self.assertEqual(config["input"]["expected_rays_per_bundle"], 131072)
        self.assertEqual(config["time"], {
            "point_window_s": 0.50,
            "point_max_range_m": 42.0,
            "ray_fifo_s": 0.50,
        })
        self.assertEqual(config["performance"], {
            "connectivity_threads": 1,
            "shell_prefilter_threads": 8,
            "ray_insertion_threads": 8,
        })
        spawner = (workspace /
            "src/mid360_multi_uav_sim/scripts/benchmark_scenario.py").read_text()
        self.assertIn("use_gpu:=True horizontal_samples:=1024", spawner)

        vofod = yaml.safe_load((workspace /
            "src/vofod_mid360/config/sensors/ouster_os1_128.yaml").read_text())
        ranges = vofod["body_mask"]["blocked_pattern_ranges"]
        self.assertEqual(sum(last - first + 1 for first, last in ranges), 49714)
        self.assertTrue(all(0 <= first <= last < 262144
                            for first, last in ranges))
        self.assertTrue(all(first > previous_last + 1
                            for (_previous_first, previous_last),
                                (first, _last) in zip(ranges, ranges[1:])))

    def test_mid360_adaptation_is_global_and_original_is_unchanged(self):
        workspace = ROOT.parents[1]
        original = yaml.safe_load((workspace /
            "src/vofod_mid360/config/b0_mid360_canonical.yaml").read_text())
        adapted = yaml.safe_load((workspace /
            "src/vofod_mid360/config/b0_mid360_adapted.yaml").read_text())
        self.assertEqual(original["clustering"], {
            "tolerance": 1.5, "min_points": 2, "max_size": 3.0,
            "max_distance": 50.0, "background_distance": 1.5,
            "max_explore_distance": 3.0})
        self.assertEqual(adapted["clustering"], {
            "tolerance": 1.5, "min_points": 2, "max_size": 3.0,
            "max_distance": 50.0, "background_distance": 1.5,
            "max_explore_distance": 3.0})
        self.assertEqual(yaml.safe_load((workspace /
            "src/vofod_mid360/config/vofod_original.yaml").read_text()),
            {"background": {"mode": "native_rangefinder"}})
        self.assertEqual(yaml.safe_load((workspace /
            "src/vofod_mid360/config/vofod_mid360_adapted.yaml").read_text()),
            {"background": {"mode": "native_rangefinder",
                            "sufficient_points_ratio": 0.0001},
             "separate_background": {"min_sure_voxels": 1}})
        self.assertFalse((workspace /
            "src/vofod_mid360/config/vofod_mid360_st_init.yaml").exists())

    def test_ouster_source_does_not_overwrite_a_mid_only_source(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            source = root / "sources/S2_new"
            source.mkdir(parents=True)
            (source / "source.bag").touch()
            manifest = source / "source_manifest.json"
            manifest.write_text(json.dumps({
                "recorded_message_counts": {
                    "/uav1/mid360/rays_checked": 10,
                }
            }))
            self.assertEqual(
                PAPER.source_paths(root, "S2_new", "ouster")[0],
                root / "sources_ouster/S2_new")
            manifest.write_text(json.dumps({
                "recorded_message_counts": {
                    "/uav1/os_cloud_nodelet/points": 10,
                }
            }))
            self.assertEqual(
                PAPER.source_paths(root, "S2_new", "ouster")[0], source)
