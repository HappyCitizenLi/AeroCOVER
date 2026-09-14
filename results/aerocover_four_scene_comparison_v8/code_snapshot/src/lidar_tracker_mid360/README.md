# lidar_tracker_mid360

Classic B0 tracker forked from CTU-MRS `lidar_tracker` commit
`a92b4db61060b47f1af6dcce122188ec021f2dcd`. It deliberately remains a
9-state constant-acceleration LKF with greedy nearest association, local
point clustering/OBB correction, overlap merge, and uncertainty deletion.

Canonical inputs:

```text
/uav1/vofod_mid360/detections
/uav1/mid360/points_world
/uav1/vofod_mid360/background_points
```

`points_world` must already be in the configured static frame. The old
sensor-frame input mode, bundle-level cloud TF, and tracker-side self crop are
removed so detector and tracker use identical per-ray geometry. Background
filtering is enabled in canonical B0 and only the occupied `background_points`
topic may feed its KD-tree.

Q now contains the paper's per-update variances (0.0001, 0.04, 0.09),
added once after propagation with the actual dt, bypassing mrs_lib's dt scaling.
P0 is fixed at (0.09, 1, 1), including position; R is 0.09.
These are squared Table II standard deviations. The position covariance radius is

```text
multiplier * sqrt(cbrt(det(P_position)))
```

which has units of metres. Singleton local clusters remain valid. The tracker
does not contain GNN/JPDA, visibility, scan opportunity, existence probability,
truth identity, or rolling-scan reconstruction.

Build/tests:

```bash
catkin build lidar_tracker_mid360
catkin run_tests lidar_tracker_mid360
catkin_test_results build/lidar_tracker_mid360
```

Use `roslaunch tclv_evaluation b0_canonical.launch` for experiments; the package
launch exists as a component/test entry, not as a second baseline definition.
