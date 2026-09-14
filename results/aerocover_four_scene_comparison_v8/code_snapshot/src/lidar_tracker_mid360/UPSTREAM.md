# Upstream provenance

## Source identity

- Repository: `https://github.com/ctu-mrs/lidar_tracker`
- Commit: `a92b4db61060b47f1af6dcce122188ec021f2dcd`
- Upstream package name: `lidar_tracker`
- Local package name: `lidar_tracker_mid360`
- License: BSD 3-Clause; the complete upstream `LICENSE` is retained without
  modification.
- Upstream/local `LICENSE` SHA-256:
  `e72d1dd44759e619d198b9b6f20a921bae07b60fee91cbb8dc56b56d597f14aa`

The pinned upstream `package.xml` says `MIT`, but its repository-level license
file is the full BSD 3-Clause text. This port follows and preserves that actual
license file and declares `BSD-3-Clause` in its local manifest.

## Positive import whitelist

Only the following paths were imported from the pinned commit:

1. `CMakeLists.txt`
2. `LICENSE`
3. `README.md`
4. `cfg/covmat.cfg`
5. `config/tracking.yaml`
6. `include/lidar_tracker/LidarTracker.h`
7. `include/lidar_tracker/point_types.h`
8. `launch/lidar_tracker.launch`
9. `msg/Track.msg`
10. `msg/Tracks.msg`
11. `package.xml`
12. `plugins.xml`
13. `src/lidar_tracker.cpp`

The SHA-256 of the listed sequence encoded as repeated
`path + NUL + upstream blob + NUL` is
`2d6c058918b468b4b4b39b31291651573a7c6a70b68efa21a7a5826b2c052967`.
Upstream `.gitignore`, RViz configuration, and the complete `tmux/` simulation
tree were deliberately not imported.

## Controlled local diff

- Renamed the ROS package, C++ namespace, generated messages, headers,
  libraries, and nodelet plugin to `lidar_tracker_mid360`.
- Replaced `vofod` messages with `vofod_mid360` messages.
- Replaced the Ouster-specific point type with standard `pcl::PointXYZI` and
  removed all Ouster, image transport, image geometry, and cv_bridge
  dependencies.
- Rewired the primary cloud input to Mid-360 `points_world`; rewrote launch
  arguments so all baseline outputs occupy an independent tracker namespace.
- Made static-background rejection optional for tests and enabled in canonical B0. Empty
  background messages safely clear the KD-tree, and launch never maps
  VoFOD-Mid360 free voxels as occupied background.
- Retained the upstream nine-state CA-LKF, greedy nearest association, local
  Euclidean clustering/OBB filtering, merge decision, and uncertainty-radius
  deletion semantics.
- Extracted only the CA transition, covariance-radius predicate, nearest gate,
  and monotonic ID allocation into `LidarTrackerMid360Core` so those exact
  operations are deterministic and unit-testable.
- Fixed bounded safety defects without changing the baseline estimator class:
  the unreachable `lost_track` assignment, iterator-invalidating erase loops,
  negative time propagation, non-finite detections/cloud points/covariances,
  invalid zero-velocity orientation, cluster KD-tree/input mismatch, profiling
  map races, subscriber thread startup/teardown, and empty-cloud/background
  handling.
- Added pure-core and ROS synthetic tests. No IMM, JPDA, visibility state,
  ray-depth memory, or identity layer was added to this baseline package.
- Added a bounded, monotonic points/detections frame-completion barrier and
  `TrackerFrameComplete` output. It does not change accepted-frame estimator
  math or normal outputs; duplicate/regressive sensor stamps are rejected
  before processing so the completion contract remains causal and unique.
- Phase 1 removed the sensor-frame cloud/one-TF path and tracker-side self
  crop, fixed covariance determinant conversion from variance units to metres,
  and made the occupied VoFOD background source canonical.
- The canonical configuration restores the upstream `rmin=2.5 m`; the former
  `0.75 m` engineering variant is retained only in archived experiment output.
- The common launch keeps `2.5 m` by default. The globally frozen
  `VoFOD-Mid360-Adapted` runner overrides it to `0.6 m`; Original retains the
  canonical default.

This file documents provenance; it is not a claim that modified files remain
byte-identical to upstream.
