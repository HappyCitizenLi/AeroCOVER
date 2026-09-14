# AeroCOVER four-scene simulation matrix

The active MRS/PX4 scenarios are:

| ID | World | Targets | Route |
|---|---|---:|---|
| OPEN | `PW_open` | 1 | continuous observer circle; five rising 5 m squares |
| MT | `PW_mt` | 5 | two hovering, three circling; 15 m cylinder |
| OFFICE | `PW_office` | 1 | connected corridors and northeast room |
| FOREST | `PW_forest_seed0` | 1 | full west-to-east near-tree traverse |

The following scenarios are retained for historical reproducibility:

| ID | World | Targets | Route |
|---|---|---:|---|
| S1_near | `PW_open` | 1 | open, near-range pass |
| S1_far | `PW_open` | 1 | open, far-range pass |
| P01 | `PW_office` | 1 | retained corridor trajectory |
| S2_new | `PW_office` | 1 | new 46 s route, distance and scan phase |
| P02 | `PW_forest_seed0` | 1 | retained tree-slalom trajectory |
| S3_new | `PW_forest_seed0` | 1 | new 44 s route, distance and scan phase |
| M1 | `PW_open` | 3 | lowered observer; approach, close crossing and separation |
| M2 | `PW_open` | 3 | lowered observer; simultaneously visible, staggered turns |

Routes use continuous cubic trajectories. The historical six-scene
Mid-360 subset is `P01`, `S2_new`, `P02`, `S3_new`, `M1`, `M2`.
The M1/M2 observer is 0.8 m lower than the superseded version so every target
lies in the common Mid-360/Ouster elevation field; raw-return checks are
required before algorithm replay.

Every world includes `observer_rangefinder`, a real Gazebo 5 x 5 ray sensor.
The OS1-128 path uses Gazebo 11 `gpu_ray` at 1024 x 128 and 10 Hz. Its 2*pi
horizontal sweep is rendered by Gazebo's internal three-camera path.
`rangefinder_follower.py` only moves that sensor rigidly with the observer; it
does not synthesize range from odometry. The MRS range plugin publishes
`/uav1/native_rangefinder`, and NativeInit transforms the measured endpoint
using its timestamped sensor frame.

Visualize a scenario:

```bash
source /opt/ros/noetic/setup.bash
source /home/uav/lyk/devel/setup.bash
/usr/bin/python3 /home/uav/lyk/src/mid360_multi_uav_sim/scripts/view_scenario.py OPEN --sensor paired --rviz
```

Record sources through `soft_vofod_evaluation/scripts/run_four_scene_suite.py` so
the scenario, source hash, physical checks and sensor-message counts are saved
together.

Current geometry rules, parameter changes and all four GUI commands are in
[`README_ZH.md`](../../results/aerocover_four_scene_comparison_v2/README_ZH.md).
