# VoFOD checked-ray research adapter

Controlled compatibility fork of CTU-MRS VoFOD commit
`7da9f33a878a586588f6a626b75cfeacac7824f7`. Provenance and redistribution
limits are in `UPSTREAM.md`.

The adapter consumes a synchronized world-frame point cloud and
`mid360_ray_msgs/CheckedRayBundle`. It validates scan IDs, original ray
indices, return status and endpoint geometry before preserving the upstream
occupancy update, close/far split, floating search, OBB detection, asynchronous
free-ray update and separate-background cleanup. The only active background
source is the original `sensor_msgs/Range` seed path.

Two declared configurations are used in the paper matrix:

These are pinned GitHub-default configurations, not an exact reproduction of
the paper's Table II: the paper uses a 0.25 m map voxel, while this fork uses
the upstream YAML default of 0.5 m. Tracker noise/initialization conventions
also differ from that table; see `results/aerocover_paper_main_comparison/VOFOD_PARAMETER_AUDIT_ZH.md`
in the workspace. No detector/tracker parameters were changed during this audit.

- `VoFOD-Original` (OS1 only in the paper matrix): `b0_mid360_canonical.yaml` plus `vofod_original.yaml` and
  the upstream 2.5 m tracker minimum radius. Its detector, map-maturity and
  sure-background thresholds retain the upstream values.
- `VoFOD-Mid360-Adapted`: `b0_mid360_adapted.yaml` plus
  `vofod_mid360_adapted.yaml` and a 0.6 m tracker minimum radius. One
  global sparse-sensor adaptation changes only the readiness gates to
  `sufficient_points_ratio=1e-4` and one sure voxel. P01/P02 development tests
  showed that changing voxel or clustering parameters reduced HOTA or raised
  FP, so the original 0.5 m voxel, 1.5 m clustering/background distance,
  `min_points=2` and `max_size=3.0 m` are retained.

Both use the fixed map `[-20,42] × [-12,26] × [-3,13] m`; there are no
scene-specific overrides. The Mid-360-adapted configuration is also replayed
unchanged on Ouster as a cross-sensor control.

The Ouster sensor overlay contains one global swept-body mask: 49,714 pattern
indices compacted into ranges, calibrated without truth labels from the four
open GPU sources. Both Original and Adapted use the same mask.

```bash
roslaunch tclv_evaluation ouster_original.launch

roslaunch tclv_evaluation b0_canonical.launch \
  b0_config:=$(rospack find vofod_mid360)/config/b0_mid360_adapted.yaml \
  method_config:=$(rospack find vofod_mid360)/config/vofod_mid360_adapted.yaml \
  tracker_radius_min:=0.6
```

Build and test with `catkin build vofod_mid360` and
`catkin run_tests vofod_mid360`.
