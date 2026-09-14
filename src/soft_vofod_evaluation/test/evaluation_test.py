#!/usr/bin/env python3

import importlib.util
import pathlib
import unittest
from unittest import mock
from types import SimpleNamespace


ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "evaluate_bag", ROOT / "scripts/evaluate_bag.py")
EVALUATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EVALUATOR)


def item(identity, x, y=0.0, z=0.0):
    return {"id": identity, "position": (x, y, z),
            "velocity": (0.0, 0.0, 0.0)}


def frame(truth, predictions, stamp=0.0):
    return {"time": stamp, "truth": truth, "predictions": predictions,
            "observer": (0.0, 0.0, 0.0)}


class Hota3DPosTest(unittest.TestCase):
    def test_los_ignore_protects_visible_matches_and_counts_duplicate_fp(self):
        visible=[item('visible',0)];hidden=[item('hidden',.5)]
        predictions=[item('a',0),item('b',.5),item('duplicate',.6)]
        kept,count=EVALUATOR.ignore_occluded_predictions(visible,hidden,predictions,1.5)
        self.assertEqual(count,1);self.assertEqual([x['id'] for x in kept],['a','duplicate'])
        metric=EVALUATOR.set_metrics([frame(visible,kept)],1.5)
        self.assertEqual((metric['tp'],metric['fp'],metric['fn']),(1,1,0))

    def test_pillar_los_uses_geometry_not_returns(self):
        import sys
        sys.path.insert(0,str(ROOT/'scripts'))
        from run_benchmark import static_primitives
        primitives=static_primitives('PW_mt')
        self.assertTrue(EVALUATOR.static_center_los([12,0,3.2],[3.13,0,3.2],primitives))
        self.assertFalse(EVALUATOR.static_center_los([12,0,3.2],[-3.13,0,3.8],primitives))

    def test_hota_is_temporarily_disabled_in_default_metrics(self):
        self.assertFalse(EVALUATOR.COMPUTE_HOTA)
        metrics = EVALUATOR.set_metrics([frame([item('truth', 0)], [item('track', 0)])], 1.5)
        self.assertIsNone(metrics['HOTA'])
        self.assertEqual(metrics['tp'], 1)

    def test_main_matching_radius_is_1p5_and_retains_1m_sensitivity(self):
        self.assertEqual(EVALUATOR.MAIN_THRESHOLD_M, 1.5)
        self.assertIn(1.0, EVALUATOR.THRESHOLDS_M)
        frames = [frame([item('truth', 0)], [item('track', 1.2)])]
        self.assertEqual(EVALUATOR.set_metrics(frames, EVALUATOR.MAIN_THRESHOLD_M)['tp'], 1)
        self.assertEqual(EVALUATOR.set_metrics(frames, 1.0)['tp'], 0)

    @staticmethod
    def stamp(value):
        return SimpleNamespace(to_sec=lambda: value)

    def test_acquisition_end_is_read_not_assumed(self):
        header=SimpleNamespace(stamp=self.stamp(10.))
        rays=[SimpleNamespace(source=SimpleNamespace(offset_time_ns=n))
              for n in (0,12345000)]
        self.assertAlmostEqual(EVALUATOR.scan_end_stamp(SimpleNamespace(header=header,rays=rays)),10.012345)
        import struct
        cloud=SimpleNamespace(header=header,fields=[SimpleNamespace(name='t',datatype=6,offset=0)],
            height=1,width=2,point_step=4,row_step=8,is_bigendian=False,data=struct.pack('<II',0,0))
        self.assertEqual(EVALUATOR.scan_end_stamp(cloud),10.)

    def test_state_projection_and_truth_interpolation(self):
        vector=lambda x:SimpleNamespace(x=x,y=0.,z=0.)
        track=SimpleNamespace(last_prediction=self.stamp(10.),position=vector(0),
                              velocity=vector(10),acceleration=vector(0))
        position,velocity=EVALUATOR.prediction_at(track,10.1)
        self.assertAlmostEqual(position[0],1.)
        self.assertEqual(velocity[0],10.)
        with self.assertRaises(RuntimeError):EVALUATOR.prediction_at(track,9.9)
        truth=[(10.,dict(position=(0,0,0),velocity=(10,0,0))),
               (10.2,dict(position=(2,0,0),velocity=(10,0,0)))]
        self.assertAlmostEqual(EVALUATOR.interpolate_truth(truth,10.1)['position'][0],1.)
        with self.assertRaises(RuntimeError):EVALUATOR.interpolate_truth(truth,10.3)

    def test_source_frame_is_separate_from_state_time_and_missing_stays_missing(self):
        vector=lambda x:SimpleNamespace(x=x,y=0.,z=0.)
        track=SimpleNamespace(id=1,n_detections=3,confidence=1.,
            last_prediction=self.stamp(10.1),last_correction=self.stamp(10.),
            position=vector(1),velocity=vector(10),acceleration=vector(0))
        msg=SimpleNamespace(header=SimpleNamespace(stamp=self.stamp(10.1)),
            source_header=SimpleNamespace(stamp=self.stamp(10.)),tracks=[track])
        empty=SimpleNamespace(header=SimpleNamespace(stamp=self.stamp(10.3)),
            source_header=SimpleNamespace(stamp=self.stamp(10.2)),tracks=[])
        with mock.patch.object(EVALUATOR.rosbag,'Bag') as bag:
            bag.return_value.__enter__.return_value.read_messages.return_value=[
                ('/aerocover/tracks',msg,None),('/aerocover/tracks',empty,None)]
            frames,tracks,*_=EVALUATOR.load_run('unused','AeroCOVER-Mid360',10.,10.2,
                                               {10.:10.1,10.1:10.2,10.2:10.3})
        self.assertEqual(len(frames),2)
        self.assertEqual(dict(frames).get(10.2,[]),[])
        self.assertEqual(dict(frames)[10.1][0]['position'],(1.,0.,0.))
        self.assertAlmostEqual(tracks[0]['stale_s'],.1)
        self.assertEqual(tracks[0]['source_stamp'],10.)

    def test_buffer_posterior_is_assigned_to_actual_state_not_old_trigger(self):
        vector=lambda x:SimpleNamespace(x=x,y=0.,z=0.)
        def message(state,x):
            track=SimpleNamespace(id=1,n_detections=3,confidence=1.,
                last_prediction=self.stamp(state),last_correction=self.stamp(state),
                last_detection=self.stamp(10.),
                position=vector(x),velocity=vector(10),acceleration=vector(0))
            return SimpleNamespace(header=SimpleNamespace(stamp=self.stamp(state)),
                source_header=SimpleNamespace(stamp=self.stamp(10.)),tracks=[track])
        audit={}
        with mock.patch.object(EVALUATOR.rosbag,'Bag') as bag:
            bag.return_value.__enter__.return_value.read_messages.return_value=[
                ('/uav1/batch/b0/tracks',message(10.,0),self.stamp(10.11)),
                ('/uav1/batch/b0/tracks',message(10.1,99),self.stamp(10.21))]
            frames,*_=EVALUATOR.load_run('unused','VoFOD-Mid360',10.,11.,{10.:10.1,10.1:10.2},audit)
        self.assertAlmostEqual(frames[0][1][0]['position'][0],1.)
        self.assertAlmostEqual(frames[1][1][0]['position'][0],100.)
        self.assertEqual(audit['state_time_remapped_publications'],1)

    def test_state_snapshot_replacement_empty_and_no_retroactive_refill(self):
        msg=lambda s:SimpleNamespace(header=SimpleNamespace(stamp=self.stamp(s)),
                                    source_header=SimpleNamespace(stamp=self.stamp(9.9)),tracks=[])
        audit={};selector=EVALUATOR.StateTimeSelection({10.:10.1,10.1:10.2},audit)
        self.assertEqual(selector.choose(msg(10.),10.11),10.1)
        self.assertEqual(selector.choose(msg(10.),10.12),10.1)
        self.assertEqual(selector.choose(msg(10.1),10.21),10.2)
        self.assertIsNone(selector.choose(msg(10.),10.22))
        self.assertEqual(len(selector.selected),2)
        self.assertEqual(selector.rows[0]['decision'],'superseded_same_state')
        self.assertEqual(audit['late_past_state_rejected'],1)

    def test_state_selection_rejects_future_observation_but_audits_receipt_clock_lag(self):
        track=SimpleNamespace(last_prediction=self.stamp(10.),last_correction=self.stamp(10.2),last_detection=self.stamp(10.))
        msg=SimpleNamespace(header=SimpleNamespace(stamp=self.stamp(10.)),
                            source_header=SimpleNamespace(stamp=self.stamp(10.)),tracks=[track])
        audit={};selector=EVALUATOR.StateTimeSelection({10.:10.1},audit)
        self.assertIsNone(selector.choose(msg,10.3))
        self.assertEqual(audit['inconsistent_state_or_observation_time'],1)
        msg.tracks=[]
        audit={};selector=EVALUATOR.StateTimeSelection({10.:10.1},audit)
        self.assertEqual(selector.choose(msg,9.999),10.1)
        self.assertEqual(selector.choose(msg,10.099),10.1)
        self.assertEqual(audit['receipt_clock_before_state'],1)
        self.assertEqual(audit['receipt_clock_before_score'],2)

    def test_state_selection_never_uses_nearest_frame(self):
        msg=SimpleNamespace(header=SimpleNamespace(stamp=self.stamp(10.05)),
                            source_header=SimpleNamespace(stamp=self.stamp(10.)),tracks=[])
        selector=EVALUATOR.StateTimeSelection({10.:10.1,10.1:10.2},{})
        self.assertIsNone(selector.choose(msg,10.3))
        self.assertFalse(selector.selected)

    def test_state_selection_does_not_reorder_availability(self):
        msg=SimpleNamespace(header=SimpleNamespace(stamp=self.stamp(10.)),
                            source_header=SimpleNamespace(stamp=self.stamp(10.)),tracks=[])
        audit={};selector=EVALUATOR.StateTimeSelection({10.:10.1},audit)
        self.assertEqual(selector.choose(msg,10.3),10.1)
        self.assertIsNone(selector.choose(msg,10.2))
        self.assertEqual(audit['nonmonotonic_availability'],1)

    def test_completion_counts_exact_source_keys_once(self):
        msg=lambda t:SimpleNamespace(header=SimpleNamespace(stamp=self.stamp(t)),
                                      points_status=0,detections_status=1)
        audit={}
        with mock.patch.object(EVALUATOR.rosbag,'Bag') as bag:
            bag.return_value.__enter__.return_value.read_messages.return_value=[
                ('/uav1/batch/b0/lidar_tracker_mid360/frame_complete',msg(t),None)
                for t in (10.,10.,10.05)]
            EVALUATOR.load_run('unused','VoFOD-Mid360',10.,11.,{10.:10.1,10.1:10.2},audit)
        self.assertEqual(audit['completed_source_frames'],1)
        self.assertEqual(audit['expected_source_frames'],2)

    def test_mt_static_reference_includes_formation_cylinder(self):
        keys = EVALUATOR.static_voxels('PW_mt')
        self.assertIn(EVALUATOR.quantize((1.2, 0., 1.)), keys)
        self.assertIn(EVALUATOR.quantize((0., 0., 15.)), keys)

    def test_only_last_scored_free_snapshot_is_decoded(self):
        snapshots = [(1.2, {(0, 0, 0)}), (1.8, {(8, 8, 8)}),
                     (1.5, {(1, 2, 3), (4, 5, 6)}), (2.5, {(9, 9, 9)})]
        messages = []
        for stamp, keys in snapshots:
            message = SimpleNamespace(
                header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda t=stamp: t)),
                keys=keys)
            messages.append(('/uav1/vofod_mid360/free_voxels', message, None))
        with mock.patch.object(EVALUATOR.rosbag, 'Bag') as bag, \
                mock.patch.object(EVALUATOR, 'read_cloud_keys', side_effect=lambda m: m.keys) as decode:
            bag.return_value.__enter__.return_value.read_messages.return_value = messages
            result = EVALUATOR.load_run('unused', 'VoFOD-Mid360', 0., 2.)
            self.assertEqual(decode.call_count, 1)
            self.assertEqual(result[3], [snapshots[2]])
        # Preserve last-in-bag semantics even if header stamps are out of order.
        with mock.patch.object(EVALUATOR, 'static_voxels', return_value={(1, 2, 3)}):
            self.assertEqual(EVALUATOR.map_metrics([], snapshots[:3], [], 'PW_open'),
                             EVALUATOR.map_metrics([], result[3], [], 'PW_open'))

    def test_perfect_single_and_multi_target(self):
        for count in (1, 3):
            frames = [frame(
                [item("g{}".format(i), i * 3.0) for i in range(count)],
                [item("p{}".format(i), i * 3.0) for i in range(count)], t)
                for t in range(5)]
            result = EVALUATOR.hota_3d_pos(frames)
            self.assertAlmostEqual(result["HOTA"], 1.0)
            self.assertAlmostEqual(result["DetA"], 1.0)
            self.assertAlmostEqual(result["AssA"], 1.0)

    def test_two_of_ten_missed_is_point_eight_at_every_threshold(self):
        frames = [frame([item("g", 0.0)],
                        [] if t < 2 else [item("p", 0.0)], t)
                  for t in range(10)]
        result = EVALUATOR.hota_3d_pos(frames)
        for row in result["per_threshold"]:
            self.assertAlmostEqual(row["DetA"], 0.8)
            self.assertAlmostEqual(row["AssA"], 0.8)
            self.assertAlmostEqual(row["HOTA"], 0.8)

    def test_extra_duplicate_prediction_lowers_score(self):
        frames = [frame([item("g", 0.0)],
                        [item("p", 0.0), item("duplicate", 0.1)], t)
                  for t in range(5)]
        self.assertLess(EVALUATOR.hota_3d_pos(frames)["HOTA"], 1.0)

    def test_id_switch_lowers_association(self):
        stable = [frame([item("g", 0.0)], [item("p", 0.0)], t)
                  for t in range(6)]
        switched = [frame([item("g", 0.0)],
                          [item("p0" if t < 3 else "p1", 0.0)], t)
                    for t in range(6)]
        self.assertLess(EVALUATOR.hota_3d_pos(switched)["AssA"],
                        EVALUATOR.hota_3d_pos(stable)["AssA"])

    def test_one_metre_boundary_is_alpha_point_five(self):
        result = EVALUATOR.hota_3d_pos(
            [frame([item("g", 0.0)], [item("p", 1.0)])])
        rows = {round(row["alpha"], 2): row
                for row in result["per_threshold"]}
        self.assertEqual(rows[0.50]["TP"], 1)
        self.assertEqual(rows[0.55]["TP"], 0)

    def test_empty_sets_follow_official_zero_convention(self):
        result = EVALUATOR.hota_3d_pos([frame([], [])])
        self.assertEqual(result["HOTA"], 0.0)
        self.assertEqual(result["AssA"], 0.0)

    def test_global_id_rename_is_invariant(self):
        first = [frame([item("g", 0.0)], [item("p", 0.0)], t)
                 for t in range(4)]
        second = [frame([item("renamed-g", 0.0)],
                        [item("renamed-p", 0.0)], t) for t in range(4)]
        self.assertEqual(EVALUATOR.hota_3d_pos(first)["HOTA"],
                         EVALUATOR.hota_3d_pos(second)["HOTA"])


if __name__ == "__main__":
    unittest.main()
