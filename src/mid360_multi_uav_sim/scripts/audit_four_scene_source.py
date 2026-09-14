#!/usr/bin/env python3
"""Validate measured flight geometry and collect sensor-only body-mask evidence."""
import argparse
import json
from pathlib import Path

import numpy as np
import rosbag
from scipy.spatial.transform import Rotation
import yaml

from check_four_scenes import check


def cloud_array(message):
    types = {1:'i1', 2:'u1', 3:'i2', 4:'u2', 5:'i4', 6:'u4', 7:'f4', 8:'f8'}
    dtype = np.dtype(dict(names=[f.name for f in message.fields],
                          formats=[('>' if message.is_bigendian else '<')+types[f.datatype]
                                   for f in message.fields],
                          offsets=[f.offset for f in message.fields], itemsize=message.point_step))
    assert message.row_step == message.width*message.point_step
    return np.frombuffer(message.data, dtype=dtype)


def audit(directory):
    scenario = yaml.safe_load((directory/'scenario.yaml').read_text())
    entities = ['uav1']+[t['id'] for t in scenario['targets']]
    topics = {f'/mid360_multi_uav_sim/ground_truth/{e}/odom': e for e in entities}
    rows = {e: [] for e in entities}
    rangefinder_tf_count = 0
    bad_rangefinder_tf = []
    starts = []
    blocked = np.zeros(131072, dtype=bool)
    previous_raw_stamp = 0
    raw = dict(scans=0, min_rays=131072, max_rays=0, maximum_direction_residual_m=0.,
               invalid_nonzero_returns=0, nonincreasing_stamps=0, zero_stamps=0)
    with rosbag.Bag(str(directory/'source.bag')) as bag:
        for topic, msg, _ in bag.read_messages(topics=list(topics)+[
                '/mid360_multi_uav_sim/scenario_events', '/uav1/os_cloud_nodelet/points', '/tf_static']):
            if topic == '/tf_static':
                for transform in msg.transforms:
                    if transform.child_frame_id != 'uav1/native_rangefinder':
                        continue
                    rangefinder_tf_count += 1
                    p,q=transform.transform.translation,transform.transform.rotation
                    expected=np.array([.5,.5,-.5,.5])
                    actual=np.array([q.x,q.y,q.z,q.w])
                    if (transform.header.frame_id!='uav1/fcu' or
                            not np.allclose([p.x,p.y,p.z],[0,.0625,-.009],atol=1e-10) or
                            min(np.linalg.norm(actual-expected),np.linalg.norm(actual+expected))>1e-10):
                        bad_rangefinder_tf.append(transform.header.frame_id)
                continue
            if topic in topics:
                p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
                rows[topics[topic]].append([msg.header.stamp.to_sec(), p.x, p.y, p.z,
                                            q.x, q.y, q.z, q.w, v.x, v.y, v.z])
            elif topic.endswith('scenario_events'):
                event = json.loads(msg.data)
                if event['event'] == 'benchmark_start':
                    starts.append(event['sim_time'])
            else:
                raw_stamp = msg.header.stamp.to_nsec()
                raw['zero_stamps'] += int(raw_stamp == 0)
                raw['nonincreasing_stamps'] += int(raw_stamp <= previous_raw_stamp)
                previous_raw_stamp = raw_stamp
                assert (msg.width, msg.height) == (1024, 128), (msg.width, msg.height)
                array = cloud_array(msg)
                points = np.column_stack([array[a] for a in ('x', 'y', 'z')])
                ranges = np.linalg.norm(points, axis=1)
                valid = (array['range'] > 0) & np.isfinite(ranges) & (ranges >= .1)
                blocked |= valid & (ranges <= .6)
                az = np.tile(np.linspace(0, 2*np.pi, 1024), 128)
                el = np.repeat(np.linspace(-np.pi/8, np.pi/8, 128), 1024)
                directions = np.column_stack([np.cos(el)*np.cos(az), np.cos(el)*np.sin(az), np.sin(el)])
                residual = np.linalg.norm(points[valid]-directions[valid]*ranges[valid, None], axis=1)
                raw['maximum_direction_residual_m'] = max(raw['maximum_direction_residual_m'], float(residual.max(initial=0)))
                raw['invalid_nonzero_returns'] += int(np.count_nonzero((array['range'] > 0) & ~valid))
                raw['scans'] += 1
                raw['min_rays'] = min(raw['min_rays'], len(array))
                raw['max_rays'] = max(raw['max_rays'], len(array))
    assert len(starts) == 1, starts
    start = starts[0]
    arrays = {e: np.asarray(values) for e, values in rows.items()}
    for entity, table in arrays.items():
        assert table[0, 0] <= start+.05 and table[-1, 0] >= start+scenario['duration_s']-.05, entity
    def values(e, t):
        table = arrays[e]
        return np.array([np.interp(start+t, table[:, 0], table[:, i]) for i in range(1, 11)])
    def states(t):
        return [(values(e, t)[:3], values(e, t)[7:], [0., 0., 0.]) for e in entities]
    def rotation(t):
        return Rotation.from_quat(values('uav1', t)[3:7]).as_matrix()
    report = check(scenario, states_at=states, observer_rotation_at=rotation,
                   entity_rotation_at=lambda i, t: Rotation.from_quat(values(entities[i], t)[3:7]).as_matrix())
    report['maximum_acceleration_mps2'] = None
    report['truth_maximum_gap_s'] = {
        entity: float(np.diff(table[(table[:,0]>=start-.02)&
                                   (table[:,0]<=start+scenario['duration_s']+.02),0]).max(initial=0))
        for entity,table in arrays.items()}
    if max(report['truth_maximum_gap_s'].values()) > scenario.get(
            'maximum_truth_gap_s',.05 if 'required_peak_speed_mps' in scenario else float('inf')):
        report['status']='FAIL'
    report['note'] = f"Measured poses at {1000*report['sample_period_s']:g} ms; acceleration not assessed here. Raw body calibration uses no target truth."
    report['raw_os1'] = raw
    report['rangefinder_tf'] = dict(count=rangefinder_tf_count,bad_parents=bad_rangefinder_tf)
    if not rangefinder_tf_count or bad_rangefinder_tf:
        report['status']='FAIL'
    if (not raw['scans'] or raw['maximum_direction_residual_m'] > .01 or
            raw['invalid_nonzero_returns'] or raw['nonincreasing_stamps'] or raw['zero_stamps']):
        report['status'] = 'FAIL'
    report['body_mask_indices'] = np.flatnonzero(blocked).tolist()
    (directory/'flight_audit.json').write_text(json.dumps(report, indent=2)+'\n')
    print(directory.name, json.dumps({k:v for k,v in report.items() if k != 'body_mask_indices'}, indent=2), flush=True)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('source_directory', type=Path)
    audit(parser.parse_args().source_directory)
