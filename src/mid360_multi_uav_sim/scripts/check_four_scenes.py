#!/usr/bin/env python3
"""Check complete planned trajectories; live bag validation is still required."""
import json
import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'soft_vofod_evaluation/scripts'))
from run_benchmark import _trajectory_sample, _point_clearance, _ray_primitive_entry, static_primitives, target_primitives
from design_four_scenes import designs
from four_scene_geometry import office_triangles, mesh_distance, mesh_los, mesh_segment_distance


def thrust_rotation(acceleration):
    z=np.asarray(acceleration,dtype=float)+[0.,0.,9.81]
    if not np.all(np.isfinite(z)) or np.linalg.norm(z)<1e-9:
        raise ValueError('invalid thrust attitude')
    z/=np.linalg.norm(z)
    y=np.cross(z,[1.,0.,0.])
    if np.linalg.norm(y)<1e-9: raise ValueError('singular thrust/heading attitude')
    y/=np.linalg.norm(y)
    return np.column_stack((np.cross(y,z),y,z))


def check(config, step=None, states_at=None, observer_rotation_at=None, entity_rotation_at=None):
    step = config.get('geometry_sample_period_s',.05) if step is None else step
    world = config['world']
    primitives = static_primitives(world)
    mesh = office_triangles() if world == 'PW_office' else None
    if mesh is not None:
        low, high = mesh.min(axis=1), mesh.max(axis=1)
        primitives = primitives[:1]  # Actual mesh replaces old proxy walls.
    entities = [config['observer']] + config['targets']
    guarded = []
    for primitive in primitives:
        inflated = dict(primitive)
        if primitive['type'] == 'cylinder':
            inflated.update(radius=primitive['radius']+.1, length=primitive['length']+.2)
        elif primitive['type'] == 'box':
            inflated['size'] = [size+.2 for size in primitive['size']]
        else:
            inflated['pose'] = list(primitive['pose'])
            inflated['pose'][2] += .1
        guarded.append(inflated)
    report = dict(minimum_body_clearance_m=math.inf, minimum_separation_m=math.inf,
                  minimum_range_m=math.inf, maximum_range_m=0.,
                  minimum_elevation_deg=math.inf, maximum_elevation_deg=-math.inf,
                  maximum_speed_mps=0., maximum_acceleration_mps2=0., static_los_failures=0,
                  target_close_pass_fraction=0., sample_period_s=step, static_los_guard_failures=0,
                  unallowed_static_los_failures=0, unallowed_static_los_guard_failures=0)
    critical = {}
    occlusions = {}
    static_occlusions = {}
    allowed_static = set(config.get('allow_static_occlusion_targets', []))
    moving_ids = set(config.get('moving_target_ids', [t['id'] for t in config['targets']]))
    report['per_entity_maximum_speed_mps'] = {f'uav{i+1}':0. for i in range(len(entities))}
    report['per_entity_maximum_horizontal_speed_mps'] = {f'uav{i+1}':0. for i in range(len(entities))}
    report['maximum_horizontal_speed_mps'] = 0.
    body = target_primitives()
    close = total = 0
    for t in np.arange(0, config['duration_s']+step/2, step):
        t=min(float(t),config['duration_s'])
        states = (states_at(t) if states_at else
                  [_trajectory_sample(e['waypoints'], t, config['trajectory_profile'],
                                      config['trajectory_tangent_scale']) for e in entities])
        positions = [np.asarray(s[0]) for s in states]
        for i, (p, (_, v, a)) in enumerate(zip(positions, states)):
            report['maximum_speed_mps'] = max(report['maximum_speed_mps'], np.linalg.norm(v))
            report['per_entity_maximum_speed_mps'][f'uav{i+1}'] = max(
                report['per_entity_maximum_speed_mps'][f'uav{i+1}'],float(np.linalg.norm(v)))
            horizontal_speed=float(np.linalg.norm(v[:2]))
            report['per_entity_maximum_horizontal_speed_mps'][f'uav{i+1}'] = max(
                report['per_entity_maximum_horizontal_speed_mps'][f'uav{i+1}'],horizontal_speed)
            report['maximum_horizontal_speed_mps'] = max(report['maximum_horizontal_speed_mps'],horizontal_speed)
            report['maximum_acceleration_mps2'] = max(report['maximum_acceleration_mps2'], np.linalg.norm(a))
            clearance = min(_point_clearance(p, primitive) for primitive in primitives)
            obstacle_clearance = min((_point_clearance(p, primitive) for primitive in primitives if primitive['type']!='plane'),default=math.inf)
            if mesh is not None:
                nearby = mesh[np.all(high >= p-2, axis=1) & np.all(low <= p+2, axis=1)]
                if len(nearby):
                    mesh_gap = mesh_distance(p, nearby)
                    clearance = min(clearance, mesh_gap)
                    obstacle_clearance = min(obstacle_clearance, mesh_gap)
            clearance -= .43  # Rotors: hypot(.1812,.1812)+.1651 < .43 m.
            if clearance < report['minimum_body_clearance_m']:
                report['minimum_body_clearance_m'] = clearance
                critical['clearance'] = [float(t), i+1, p.tolist()]
            if i:
                close += obstacle_clearance-.43 < 1.
                total += 1
            for q in positions[i+1:]:
                report['minimum_separation_m'] = min(report['minimum_separation_m'], np.linalg.norm(q-p))
        for target_index, p in enumerate(positions[1:], 1):
            target_id = config['targets'][target_index-1]['id']
            distance = np.linalg.norm(p-positions[0])
            report['minimum_range_m'] = min(report['minimum_range_m'], distance)
            report['maximum_range_m'] = max(report['maximum_range_m'], distance)
            for mount in (.1414, .168):
                rotation = (observer_rotation_at(t) if observer_rotation_at else
                            thrust_rotation(states[0][2]) if config.get('planned_attitude_from_acceleration') else np.eye(3))
                origin = positions[0]+rotation @ [0, 0, mount]
                delta = p-origin
                sensor_delta = rotation.T @ delta
                elevation = math.degrees(math.atan2(sensor_delta[2], np.linalg.norm(sensor_delta[:2])))
                report['minimum_elevation_deg'] = min(report['minimum_elevation_deg'], elevation)
                report['maximum_elevation_deg'] = max(report['maximum_elevation_deg'], elevation)
                blocked = any(hit is not None and hit < 1-1e-9 for hit in
                              (_ray_primitive_entry(origin, delta, prim) for prim in primitives))
                guard_failed = any(hit is not None and hit < 1-1e-9 for hit in
                                   (_ray_primitive_entry(origin, delta, prim) for prim in guarded))
                if mesh is not None:
                    nearby = mesh[np.all(high >= np.minimum(origin, p), axis=1) &
                                  np.all(low <= np.maximum(origin, p), axis=1)]
                    blocked |= not mesh_los(origin, p, nearby)
                    near_guard = mesh[np.all(high >= np.minimum(origin, p)-.1, axis=1) &
                                      np.all(low <= np.maximum(origin, p)+.1, axis=1)]
                    guard_failed |= mesh_segment_distance(origin, p, near_guard) < .1
                if guard_failed:
                    report['static_los_guard_failures'] += 1
                    report['unallowed_static_los_guard_failures'] += int(target_id not in allowed_static)
                if blocked:
                    report['static_los_failures'] += 1
                    report['unallowed_static_los_failures'] += int(target_id not in allowed_static)
                    if mount == .168:
                        static_occlusions.setdefault(target_id, []).append(float(t))
                    critical.setdefault('los', [float(t), p.tolist()])
                if mount == .168:
                    for other_index, other in enumerate(positions[1:], 1):
                        if other_index == target_index:
                            continue
                        other_rotation = (entity_rotation_at(other_index, t) if entity_rotation_at else
                                          thrust_rotation(states[other_index][2]) if config.get('planned_attitude_from_acceleration') else np.eye(3))
                        local_origin = other_rotation.T @ (origin-other)
                        local_delta = other_rotation.T @ delta
                        if any(hit is not None and 0 < hit < 1 for hit in
                               (_ray_primitive_entry(local_origin, local_delta, prim) for prim in body)):
                            pair = f'uav{other_index+1}->uav{target_index+1}'
                            occlusions.setdefault(pair, []).append(float(t))
    report['target_close_pass_fraction'] = close/total
    report['critical'] = critical
    report['mutual_center_los_occlusion_s'] = {pair: len(stamps)*step for pair, stamps in occlusions.items()}
    intervals = {}
    for pair, stamps in occlusions.items():
        groups = [[stamps[0], stamps[0]+step]]
        for stamp in stamps[1:]:
            if stamp <= groups[-1][1]+1e-8:
                groups[-1][1] = stamp+step
            else:
                groups.append([stamp, stamp+step])
        intervals[pair] = groups
    report['mutual_center_los_intervals_s'] = intervals
    report['static_center_los_intervals_s'] = {}
    for identity, stamps in static_occlusions.items():
        groups = [[stamps[0], stamps[0]+step]]
        for stamp in stamps[1:]:
            if stamp <= groups[-1][1]+1e-8: groups[-1][1] = stamp+step
            else: groups.append([stamp, stamp+step])
        report['static_center_los_intervals_s'][identity] = groups
    report['longest_mutual_center_los_occlusion_s'] = max(
        (end-begin for groups in intervals.values() for begin, end in groups), default=0.)
    report['longest_moving_mutual_occlusion_s'] = max(
        (end-begin for pair, groups in intervals.items()
         if all(identity in moving_ids for identity in pair.split('->'))
         for begin, end in groups), default=0.)
    report['clearance_with_sampling_margin_m'] = report['minimum_body_clearance_m']-step*report['maximum_speed_mps']/2
    report['status'] = 'PASS' if (report['clearance_with_sampling_margin_m'] > .3 and
        report['minimum_separation_m'] > 1.3 and report['minimum_elevation_deg'] >= -5 and
        report['maximum_elevation_deg'] <= 20 and report['unallowed_static_los_failures'] == 0 and
        report['unallowed_static_los_guard_failures'] == 0 and
        report['minimum_range_m'] >= config.get('minimum_observer_target_distance_m',1.3 if config['scenario_id']=='MT' else 4) and
        report['maximum_range_m'] <= config.get('maximum_target_range_m',20 if config['scenario_id'] in ('OFFICE', 'FOREST') else 40) and
        report['maximum_speed_mps'] <= config.get('maximum_allowed_speed_mps',5) and
        report['maximum_horizontal_speed_mps'] >= config.get('required_peak_speed_mps',0) and
        report['maximum_acceleration_mps2'] <= config.get('maximum_allowed_acceleration_mps2',5) and
        report['longest_mutual_center_los_occlusion_s'] <= config.get('maximum_mutual_occlusion_s',math.inf) and
        (config['scenario_id'] != 'OFFICE' or report['target_close_pass_fraction'] > .5) and
        (config['scenario_id'] != 'FOREST' or report['target_close_pass_fraction'] > .25) and
        (config['scenario_id'] != 'MT' or
         report['longest_moving_mutual_occlusion_s'] <= config.get('maximum_moving_mutual_occlusion_s',3.5))) else 'FAIL'
    return report


if __name__ == '__main__':
    selected = sys.argv[1:] or list(designs())
    for name in selected:
        print(name, json.dumps(check(designs()[name]), indent=2), flush=True)
