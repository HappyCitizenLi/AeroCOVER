#!/usr/bin/env python3
"""Generate the fixed OPEN/MT/OFFICE/FOREST inputs; no metric-driven tuning."""
import math
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]


def waypoint(t, x, y, z, yaw=0):
    return dict(t=round(float(t), 6), x=round(float(x), 6), y=round(float(y), 6), z=round(float(z), 6), yaw=round(float(yaw), 6))


def scene(name, world, duration, observer, targets):
    return dict(schema_version=1, scenario_id=name, world=world, seed=12001,
                geometry_contract='four_scene_v1',
                mrs_custom_config='four_scene_config.yaml',
                mrs_world_config='four_scene_world.yaml', control_frame='ground_truth_origin',
                trajectory_profile='continuous_cubic', trajectory_tangent_scale=1.0,
                start_delay_s=4.0, duration_s=duration, score_start_s=1.0,
                score_end_s=duration-1, observer=dict(waypoints=observer),
                targets=[dict(id=f'uav{i+2}', preflight_active=True, waypoints=points)
                         for i, points in enumerate(targets)])


def designs():
    from design_four_scenes_v4 import designs_v4
    from design_open_19m import open_19m_scene
    configs=designs_v4()
    configs['OPEN']=open_19m_scene()
    return configs


if __name__ == '__main__':
    for name, config in designs().items():
        path = ROOT/'config/benchmarks'/f'{name}.yaml'
        path.write_text(yaml.safe_dump(config, sort_keys=False))
        print(path)
    # One frozen configuration per scene/sensor. Map covers the union of all
    # planned paths plus the 3 m detector exploration margin, not every sensor
    # endpoint (the original mapper deliberately clips rays at its map bounds).
    config_root = ROOT.parent/'vofod_mid360/config'
    output = config_root/'four_scene_suite'
    output.mkdir(exist_ok=True)
    for name in designs():
        for sensor in ('mid360', 'os1'):
            config = yaml.safe_load((config_root/'b0_mid360_canonical.yaml').read_text())
            # Ground z=0 is centered in layer 1 at 0.25 m resolution.
            config['operation_area'] = dict(center=dict(x=16, y=15, z=8.9375),
                                          size=dict(x=88, y=78, z=18.125))
            config['voxel_map']['voxel_size'] = .25
            config['clustering'].update(tolerance=1.5 if name in ('OPEN','MT') else .5,
                                        max_size=3. if name in ('OPEN','MT') else 1.5,
                                        background_distance=1.5 if name in ('OPEN','MT') else .3,
                                        min_points=1 if sensor == 'mid360' else 2)
            if sensor == 'mid360':
                config['background']['sufficient_points_ratio'] = .0001
                config['separate_background']['min_sure_voxels'] = 1
            (output/f'{name}_{sensor}.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
