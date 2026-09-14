# Checked-ray preprocessors

## Mid-360

The Mid-360 node synchronizes the producer-authored raw `PointCloud2` and
complete `RayBundle` by scan identity. It verifies source ordering, duplicate
indices, return matching, ranges and transforms, then publishes one atomic
triplet:

```text
/uav1/mid360/points_valid
/uav1/mid360/points_world
/uav1/mid360/rays_checked
```

The checked bundle preserves each attempted ray's source index, status,
offset time, origin and unit world direction. `snapshot` uses one transform;
`rolling_scene`/the retained start-end interpolation mode transforms the
moving scan without a per-ray blocking TF lookup. A failed pair, identity,
geometry or TF check suppresses the derived triplet.

Simulation uses `source_mode=sim_exact`. The retained hardware configuration
starts fail-closed as `calibrated_fallback`; it cannot promote itself to an
exact no-return claim without an external validated source-mode authority.

## Ouster OS1-128 snapshot adapter

`ouster_snapshot_adapter.launch` consumes the actual MRS/Gazebo organized
cloud at `/uav1/os_cloud_nodelet/points` (2048 columns x 128 rings). It
publishes the unchanged native cloud plus:

```text
/uav1/ouster/points_world
/uav1/ouster/rays_checked
```

Every one of the 262,144 samples becomes a checked ray; valid returns become
world points with the same original index. There is no sampling. The MRS
sensor's `t` field is zero, so the adapter explicitly labels the geometry
`ouster_sim_snapshot`; it does not claim hardware rolling-scan fidelity. The
configured fcu-to-sensor translation is `(0, 0, 0.1414) m` from the rendered
MRS X500 sensor model.

Build and test:

```bash
source /opt/ros/noetic/setup.bash
source /home/uav/lyk/devel/setup.bash
catkin build mid360_ray_msgs mid360_ray_preprocessor
catkin run_tests mid360_ray_preprocessor
catkin_test_results build/mid360_ray_preprocessor
```

The retained core and ROS tests cover source-mode fail-closed behavior, TF
stamp caching, snapshot geometry, rolling-scene geometry and malformed input.
The formal Ouster records additionally retain the observed layout, message
counts and source hashes in their source manifests.
