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



class RunnerTest(unittest.TestCase):
    def test_four_scene_suite_from_a_relay_directory(self):
        import subprocess, sys
        source=ROOT/'scripts/run_four_scene_suite.py'
        with tempfile.TemporaryDirectory() as directory:
            relay=pathlib.Path(directory)/'relay.py'
            relay.write_text('from pathlib import Path\np='+repr(str(source))+'\nexec(compile(Path(p).read_text(),p,"exec"),{"__file__":p,"__name__":"__main__"})\n')
            result=subprocess.run([sys.executable,str(relay),'--help'],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertIn('record,mask,replay,report',result.stdout)

    def test_current_four_scene_matrix(self):
        import sys
        sys.path.insert(0, str(ROOT/'scripts'))
        from run_four_scene_suite import SCENES, METHODS
        self.assertEqual(SCENES, ('OPEN', 'MT', 'OFFICE', 'FOREST'))
        self.assertEqual(set(METHODS), {'AeroCOVER-Mid360','AeroCOVER-OS1','VoFOD-Mid360','VoFOD-OS1'})
        self.assertEqual(len({(scene,method) for scene in SCENES for method in METHODS}),16)


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
            "VoFOD-Mid360", "VoFOD-OS1",
            "AeroCOVER-A1", "AeroCOVER-A2",
            "AeroCOVER-A3", "AeroCOVER-A4", "AeroCOVER-A5",
        }.issubset(RUNNER.ALGORITHMS))
        self.assertEqual(
            RUNNER.SCENES,
            ("OPEN", "MT", "OFFICE", "FOREST"))
        self.assertEqual(RUNNER.NOISE_STDDEV_M["N03"], 0.03)
        self.assertEqual(
            set(RUNNER.WORLD_FILES),
            {"PW_open", "PW_office", "PW_forest_seed0", "PW_mt"})

    def test_current_assets(self):
        workspace = ROOT.parents[1]
        for scenario, world in (("OPEN.yaml","PW_open.world"), ("MT.yaml","PW_mt.world"),
                                ("OFFICE.yaml","PW_office.world"), ("FOREST.yaml","PW_forest_seed0.world")):
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
        config=pathlib.Path("config.yaml")
        self.assertIn("ouster_original.launch", RUNNER.algorithm_command("VoFOD-OS1", config))
        self.assertIn("aerocover_os1.launch", RUNNER.algorithm_command("AeroCOVER-OS1", config))
        self.assertIn("config:=config.yaml", RUNNER.algorithm_command("AeroCOVER-OS1",config))
        for retired in ("VOFOD", "VoFOD-Mid360-Adapted", "VoFOD-Original-OS1", "VoFOD-Original-Mid360"):
            self.assertNotIn(retired, RUNNER.ALGORITHMS)
            with self.assertRaises(ValueError): RUNNER.algorithm_command(retired, config)


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
            "src/vofod_mid360/config/sensors/ouster_os1_128_1024.yaml").read_text())
        ranges = vofod["body_mask"]["blocked_pattern_ranges"]
        self.assertEqual(sum(last - first + 1 for first, last in ranges), 24849)
        self.assertTrue(all(0 <= first <= last < 131072
                            for first, last in ranges))
        self.assertTrue(all(first > previous_last + 1
                            for (_previous_first, previous_last),
                                (first, _last) in zip(ranges, ranges[1:])))
