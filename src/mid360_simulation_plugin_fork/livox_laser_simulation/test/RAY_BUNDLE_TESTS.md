# RayBundle integration tests

The test sensor model has no collision geometry. The empty world contains no
models or includes; the single-wall world contains exactly one finite box
collision. Both tests consume only the public RayBundle and legacy PointCloud2
topics and do not use Gazebo/model truth.

The validator checks every published direction against the 800,000-row CSV by
`pattern_index`, including a complete wrap after 41 bundles. It also validates
scan-ID continuity, uniform per-ray offsets, return/range contracts, legacy
field/index alignment, source diagnostics, and the first-ray positive-Z
regression. In wall mode every valid legacy endpoint must satisfy
`p ~= range * direction`; in empty mode every ray must be `NO_RETURN` while the
legacy endpoints remain zero.

After sourcing the workspace, run either smoke test headlessly:

```bash
roslaunch livox_laser_simulation ray_bundle_empty_test.launch
roslaunch livox_laser_simulation ray_bundle_wall_test.launch
```

For rostest reporting:

```bash
rostest livox_laser_simulation ray_bundle_empty.test
rostest livox_laser_simulation ray_bundle_wall.test
```

The shared launch accepts `world`, `model`, spawn pose, GUI/headless flags,
topics, expected frame, bundle count, and timeout as arguments. Its defaults are
`/uav1/mid360/rays_raw`, `/uav1/mid360/points_raw`, and `uav1/mid360`.

Generated test reports are intentionally not retained in the current workspace.
