# mid360_multi_uav_sim

Deterministic Gazebo fixtures for a Mid-360 observer, static geometry and
simplified UAV targets. The
package owns scene geometry, target/observer trajectories, commanded ground
truth, scenario events, and the single dynamic `world -> uav1/fcu` TF authority
used by moving-observer scenes.

## Scenarios

The identifiers below are the complete current scenario set.  Scene geometry,
phase durations, seeds, and target poses are frozen by the YAML and pure
contract tests.

| Scene | Purpose | World | Observer | Targets | Seed | Timeline (start + cycle + final) |
|---|---|---|---|---:|---:|---:|
| S1 | three-target horizontal line to collision-free vertical stack and back | `E0_open.world` | fixed | 3 | 1001 | 6 + 30 + 3 s |
| S01 | time-separated, vertically deconflicted two-airway crossing | `E0_open.world` | fixed | 2 | 404 | 5 + 32 + 3 s |
| S02 | ten-target counter-flow crossing with three changing five-pair heavy partial occlusions | `E0_open.world` | fixed | 10 | 2027 | 8 + 63 + 4 s |
| S03 | fixed-yaw observer translation | `E1_sparse.world` | translating | 2 | 606 | 5 + 14 + 3 s |
| S04 | combined visibility under translation/yaw | `E2_occlusion_arena.world` | translating/yawing | 3 | 707 | 5 + 20 + 3 s |
| S05 | five-target clutter stress | `E3_cluttered.world` | translating/yawing | 5 | 808 | 5 + 24 + 3 s |
| S06 | online-map wall occlusion | `E2_occlusion_arena.world` | translating/yawing | 3 | 909 | 5 + 22 + 3 s |

`schema_version` is an immutable validation-contract version, not a duplicate
of the scene number.  The retained YAML files therefore intentionally use
versions 1, 1, 7, 2, 4, 5, and 6.

S02 first establishes all ten targets on two radial rings.  It then executes
three different perfect near/far occlusion matchings, separated by two
counter-flow exchanges.  Each hold uses a 1.8 degree azimuth-centre offset:
the projected silhouettes overlap heavily without complete containment, so
both targets retain ray support.  The exchanges use a 1.5 m altitude
deconfliction layer, while every moving segment uses a seventh-order
minimum-snap time law with zero velocity, acceleration, and jerk at its
boundaries.  The frozen contract verifies the complete continuous path (not
only waypoints): maximum speed 1.909 m/s, acceleration 0.705 m/s^2, jerk
1.231 m/s^3, minimum target centre separation 2.182 m, and at least 0.840 m
conservative body clearance.  All targets remain within the configured
Mid-360 range and elevation limits.

Targets are deterministic pose-authority fixtures: `scenario_manager.py`
writes each reference pose through `/gazebo/set_model_state`.  The S02
certificate therefore establishes a C3, multirotor-feasible reference and
continuous geometric avoidance margin.  It does **not** establish motor-level
dynamics, controller tracking error, aerodynamic interaction, or closed-loop
collision avoidance.

The fixed-observer launches publish a static `world -> uav1/fcu` transform and
use snapshot ray geometry.  S03-S06 let `scenario_manager.py` command `uav1`
and publish the sole dynamic transform; their preprocessor runs in
`per_ray_pose` mode.  Only the observer model contains a Mid-360 sensor.  All
target models are sensor-free.

## Layout

- `config/scenarios/S1.yaml` and `S01.yaml` through `S06.yaml`: exact scenario contracts.
- `launch/gate_s1.launch` and `gate_s01.launch` through `gate_s06.launch`: matching Gazebo gates.
- `scripts/scenario_manager.py`: strict YAML validation, deterministic motion,
  event publication, ground truth, and moving-observer TF authority.
- `worlds/`: the four retained world fixtures.
- `models/`: observer, target, and static primitive models.
- `test/`: seven pure asset contracts, five strict manager contracts, and three
  optional live Gazebo gates.

The stable interfaces are:

```text
/uav1/mid360/rays
/mid360_multi_uav_sim/scenario_events
/mid360_multi_uav_sim/ground_truth/uav1/odom
/mid360_multi_uav_sim/ground_truth/uav2/odom ... uav11/odom (when present)
```

Events are stable JSON payloads emitted after the corresponding commanded
state and odometry have been published.  The manager does not subscribe to
Gazebo model truth, so simulator callback timing cannot enter the perception
graph.

## Run

```bash
roslaunch mid360_multi_uav_sim gate_s1.launch
roslaunch mid360_multi_uav_sim gate_s01.launch
roslaunch mid360_multi_uav_sim gate_s02.launch
roslaunch mid360_multi_uav_sim gate_s03.launch
roslaunch mid360_multi_uav_sim gate_s04.launch
roslaunch mid360_multi_uav_sim gate_s05.launch
roslaunch mid360_multi_uav_sim gate_s06.launch
```

Every gate accepts `gui`, `paused`, `verbose`, `run_preprocessor`,
`run_scenario_manager`, and `scenario_config` arguments.  Parameter overrides
are rejected by the exact-contract schemas where doing so would weaken the
fixture.

## Verify

Build the package and run the non-live contracts with:

```bash
catkin build mid360_multi_uav_sim --force-cmake
python3 -m unittest discover -s src/mid360_multi_uav_sim/test -p 's*_assets_test.py'
python3 -m unittest discover -s src/mid360_multi_uav_sim/test -p 'scenario_manager_s*_test.py'
```

The live gates require ROS and Gazebo:

```bash
rostest mid360_multi_uav_sim gate_s03.test
rostest mid360_multi_uav_sim gate_s04.test
rostest mid360_multi_uav_sim gate_s05.test
```
