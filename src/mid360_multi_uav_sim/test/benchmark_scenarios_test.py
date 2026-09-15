#!/usr/bin/env python3

import pathlib
import math
import sys
from types import SimpleNamespace
from unittest import mock
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))


class ScenarioTest(unittest.TestCase):

    def test_current_generator_fingerprint(self):
        import hashlib, json
        from design_four_scenes import designs
        expected = {"OPEN":"b77a5637ec28df1bc2f788d83a146b0da14bf53308d5a3ba8ff042ce528cc30b","MT":"ce5c57c8d90da6264592b4808499f9adc142525664b8c8f7ffee852023130913","OFFICE":"c98815bdffa67bd35739fed535d725987c87a00def06ef5b78d91f02f89afb9e","FOREST":"f07d8109f6a33fa3d702418054efb058d0d631279bd50d53859fbe850c29a0da"}
        actual = {name:hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest()
                  for name,config in designs().items()}
        self.assertEqual(actual, expected)
    def test_timed_run_aborts_when_controller_loses_output(self):
        sys.path.insert(0,str(ROOT/'scripts'))
        import benchmark_scenario as module
        scenario=module.BenchmarkScenario.__new__(module.BenchmarkScenario)
        scenario.preflight=mock.Mock()
        scenario.start_timed_trajectories=mock.Mock(return_value=module.rospy.Time(2))
        scenario.event=mock.Mock()
        scenario.trajectory_execution='mrs_trajectory'
        scenario.score_start,scenario.score_end=1.,2.
        scenario.entities=[dict(physical='uav1')]
        scenario.control={'uav1':SimpleNamespace(output_enabled=False,active_tracker='LandoffTracker')}
        with mock.patch.object(module.rospy.Time,'now',return_value=module.rospy.Time(2)), \
             mock.patch.object(module.rospy,'is_shutdown',return_value=False), \
             mock.patch.object(module.rospy,'Rate'):
            with self.assertRaisesRegex(RuntimeError,'lost active MpcTracker'):
                scenario.run()

    def test_timed_trajectory_loads_before_start_with_fixed_step(self):
        sys.path.insert(0,str(ROOT/'scripts'))
        import benchmark_scenario as module
        scenario=module.BenchmarkScenario.__new__(module.BenchmarkScenario)
        scenario.entities=[dict(physical='uav1',waypoints=module.parse_waypoints(
            [dict(t=0,x=0,z=3),dict(t=1,x=1,z=3)],'observer',1.))]
        scenario.entities.append(dict(physical='uav2',waypoints=scenario.entities[0]['waypoints']))
        scenario.control_frame='ground_truth_origin'
        scenario.seed=12001
        scenario.duration=1.
        scenario.profile='continuous_cubic'
        scenario.tangent_scale=1.
        service=mock.Mock(return_value=SimpleNamespace(success=True,modified=False))
        with mock.patch.object(module.rospy,'wait_for_service'), \
             mock.patch.object(module.rospy,'ServiceProxy',return_value=service), \
             mock.patch.object(module.rospy.Time,'now',return_value=module.rospy.Time(2)):
            start=scenario.start_timed_trajectories()
        trajectory=service.call_args_list[0][0][0].trajectory
        self.assertEqual(start.to_sec(),2.)
        self.assertEqual(trajectory.header.stamp.to_sec(),0.)
        self.assertEqual(trajectory.dt,.05)
        self.assertEqual(len(trajectory.points),22)
        self.assertEqual(trajectory.points[-1].position.x,1.)
        self.assertFalse(trajectory.fly_now)
        self.assertEqual(service.call_count,4)
        self.assertTrue(service.call_args_list[1][0])
        self.assertFalse(service.call_args_list[2][0])

    def test_preflight_waits_for_flight_tracker(self):
        sys.path.insert(0, str(ROOT/'scripts'))
        import benchmark_scenario as module
        scenario = module.BenchmarkScenario.__new__(module.BenchmarkScenario)
        scenario.entities = [dict(physical='uav1', preflight_active=True,
                                  waypoints=[(0., (0., 0., .8, 0., 0., 0.))])]
        scenario.control = {'uav1': SimpleNamespace(output_enabled=False, active_tracker='NullTracker')}
        scenario.spawn_vehicle = scenario.wait_for_mrs = scenario.arm = mock.Mock()
        scenario.reference = mock.Mock(return_value=True)
        scenario.reference_error = mock.Mock(return_value=.05)
        scenario.start_delay, scenario.wait_for_start = 0., False
        def takeoff_finished():
            scenario.control['uav1'] = SimpleNamespace(output_enabled=True, active_tracker='MpcTracker')
        with mock.patch.object(module.rospy, 'wait_for_service'), \
             mock.patch.object(module.rospy, 'is_shutdown', return_value=False), \
             mock.patch.object(module.rospy, 'logwarn'), \
             mock.patch.object(module.rospy.Time, 'now', return_value=module.rospy.Time(2)), \
             mock.patch.object(module.rospy, 'Rate') as rate:
            rate.return_value.sleep.side_effect = takeoff_finished
            scenario.preflight()
        self.assertEqual(scenario.reference.call_count, 2)

    def test_reference_tail_and_mt_occlusion_contract(self):
        sys.path.insert(0, str(ROOT/'scripts'))
        from benchmark_scenario import parse_waypoints, sample
        raw = [dict(t=0,x=0),dict(t=.5,x=.5),dict(t=1.5,x=1.5)]
        duration = 1.
        self.assertGreater(raw[-1]['t'], duration)
        parsed = parse_waypoints(raw, 'observer', duration)
        first = sample(parsed, duration-.01, 'continuous_cubic', 1.)
        last = sample(parsed, duration, 'continuous_cubic', 1.)
        self.assertGreater(math.dist(first[0][:3], last[0][:3])/.01, .7)
        with self.assertRaises(ValueError):
            parse_waypoints(raw[:-2], 'observer', duration)
        mt = yaml.safe_load((ROOT/'config/benchmarks/MT.yaml').read_text())
        self.assertEqual(mt['allow_static_occlusion_targets'], ['uav2', 'uav3'])
        self.assertEqual(mt['maximum_moving_mutual_occlusion_s'], 1.)
        cylinder = ET.parse(ROOT/'worlds/PW_mt.world').find('.//collision/geometry/cylinder')
        self.assertEqual(float(cylinder.findtext('length')), 15.)


    def test_rangefinder_tf_matches_physical_mount(self):
        plugin = ET.parse(ROOT/'models/observer_rangefinder/model.sdf').find('.//plugin')
        self.assertEqual(plugin.findtext('parentFrameName'), 'uav1/fcu')
        launch = ET.parse(ROOT/'launch/benchmark.launch')
        args = launch.find("node[@name='benchmark_native_rangefinder_tf']").get('args').split()
        self.assertEqual([float(plugin.findtext(k)) for k in ('x', 'y', 'z')], list(map(float, args[:3])))
        r, p, y = [float(plugin.findtext(k))/2 for k in ('roll', 'pitch', 'yaw')]
        sr, cr, sp, cp, sy, cy = math.sin(r), math.cos(r), math.sin(p), math.cos(p), math.sin(y), math.cos(y)
        quaternion = (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                      cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)
        for actual, expected in zip(quaternion, map(float, args[3:7])):
            self.assertAlmostEqual(actual, expected, places=12)


    def test_open_19m_shapes_speed_and_range_margin(self):
        sys.path.insert(0,str(ROOT/'scripts'))
        from design_open_19m import open_19m_scene
        import numpy as np
        new=open_19m_scene()
        self.assertEqual(new['maximum_target_range_m'],19.)
        observer=np.array([[p[k] for k in ('x','y','z')] for p in new['observer']['waypoints']])
        self.assertTrue(np.all(observer[:,1]==0))
        self.assertTrue(np.all(observer[:,2]==3))
        t=np.array([p['t'] for p in new['observer']['waypoints']])
        for entity in new['targets']:
            target=np.array([[p[k] for k in ('x','y','z')] for p in entity['waypoints']])
            self.assertLess(np.linalg.norm(target-observer,axis=1).max(),18.8)
            self.assertGreater(np.linalg.norm(target-observer,axis=1).min(),2.)
        for point in new['targets'][0]['waypoints']:
            self.assertAlmostEqual(math.hypot(point['x']-1.,point['y']-7.8),4.8,places=5)
        star=np.array([[p[k] for k in ('x','y','z')] for p in new['targets'][1]['waypoints']])
        v=np.linalg.norm(np.diff(star,axis=0),axis=1)/np.diff(t)
        self.assertGreater(v.max(),7.2)
        self.assertLess(v.max(),8.)

    def test_four_scene_paper_contract(self):
        for name in ('OPEN', 'MT', 'OFFICE', 'FOREST'):
            config = yaml.safe_load((ROOT/'config/benchmarks'/f'{name}.yaml').read_text())
            self.assertEqual(config['mrs_custom_config'], 'four_scene_config.yaml')
            self.assertEqual(config['control_frame'], 'ground_truth_origin')
            self.assertEqual(len(config['targets']), {'OPEN':2,'MT':3,'OFFICE':1,'FOREST':1}[name])
            if name == 'MT':
                from current_scene_design import mt_scene
                self.assertEqual(config, mt_scene())
                self.assertEqual(config['moving_target_ids'], ['uav4'])
                self.assertEqual([t['id'] for t in config['targets']], ['uav2','uav3','uav4'])
            self.assertEqual(config['trajectory_execution'], 'mrs_trajectory')
            if name == 'OPEN':
                self.assertEqual(config['geometry_design']['star_points'], 5)
                self.assertEqual(config['geometry_design']['star_smoothing_s'], .25)
                self.assertEqual(config['maximum_target_range_m'],19.)
                self.assertEqual(config['geometry_design']['star_path'],'outline_with_entry')
            for sensor in ('mid360', 'os1'):
                vofod = yaml.safe_load((ROOT.parent/'vofod_mid360/config/four_scene_suite'/
                                        f'{name}_{sensor}.yaml').read_text())
                self.assertEqual(vofod['voxel_map']['voxel_size'], .25)
                self.assertEqual(vofod['operation_area']['center']['x'], 16)
                self.assertEqual(vofod['operation_area']['size']['x'], 88)
                self.assertEqual(vofod['clustering']['tolerance'], 1.5 if name in ('OPEN','MT') else .5)
                self.assertEqual(vofod['clustering']['max_size'], 3. if name in ('OPEN','MT') else 1.5)
                self.assertEqual(vofod['clustering']['min_points'], 1 if sensor == 'mid360' else 2)
                self.assertEqual(vofod['clustering']['background_distance'], 1.5 if name in ('OPEN','MT') else .3)
                self.assertEqual(vofod['background']['sufficient_points_ratio'],
                                 .0001 if sensor == 'mid360' else .15)
                self.assertEqual(vofod['separate_background']['min_sure_voxels'],
                                 1 if sensor == 'mid360' else 24)
        tracker = yaml.safe_load((ROOT.parent/'lidar_tracker_mid360/config/tracking.yaml').read_text())
        override=yaml.safe_load((ROOT.parent/'lidar_tracker_mid360/config/tracking_open_v2.yaml').read_text())
        self.assertEqual(override['input_filter']['downsample_leaf_size'],.5)
        self.assertEqual(override['association']['clustering_tolerance'],1.)
        self.assertEqual(override['association']['cluster'],dict(max_size=1.,min_background_dist=1.))
        self.assertEqual(override['lkf']['P']['radius'],dict(min=2.5,max=5.))
        self.assertEqual(tracker['input_filter']['downsample_leaf_size'], .25)
        self.assertEqual(tracker['association']['clustering_tolerance'], .5)
        self.assertEqual(tracker['association']['cluster']['max_size'], 1.5)
        self.assertEqual(tracker['association']['cluster']['min_background_dist'], .1)
        self.assertEqual(tracker['lkf']['P']['radius']['min'], .6)
        self.assertEqual(tracker['lkf']['P']['radius']['max'], 3.)
        self.assertEqual(tracker['lkf']['Q'], dict(position=.0001, velocity=.04, acceleration=.09))
        self.assertEqual(tracker['lkf']['P']['init'], dict(position=.09, velocity=1., acceleration=1.))
        self.assertEqual(tracker['lkf']['R']['coeff'], .09)

    def test_paper_scenarios_are_registered(self):
        files = sorted(
            path.name for path in (ROOT / "config/benchmarks").glob("*.yaml"))
        self.assertEqual(files, [
            "FOREST.yaml", "MT.yaml", "OFFICE.yaml", "OPEN.yaml"])
        worlds = sorted(
            path.name for path in (ROOT / "worlds").glob("*.world"))
        self.assertEqual(
            worlds, ["PW_forest_seed0.world", "PW_mt.world", "PW_office.world",
                     "PW_open.world"])


    def test_world_assets_are_intact(self):
        office = ET.parse(
            ROOT / "worlds/PW_office.world").getroot().find("world")
        self.assertIsNotNone(office)
        forest = ET.parse(
            ROOT / "worlds/PW_forest_seed0.world").getroot().find(
                "world/model[@name='planning_forest_seed0']")
        self.assertIsNotNone(forest)
        self.assertEqual(len(forest.findall("link")), 250)
        open_world = ET.parse(ROOT / "worlds/PW_open.world").getroot().find(
            "world")
        self.assertIsNotNone(open_world)
