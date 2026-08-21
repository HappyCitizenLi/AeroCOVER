# VoFOD-Mid360 B0

Controlled compatibility fork of CTU-MRS VoFOD at
`7da9f33a878a586588f6a626b75cfeacac7824f7`; provenance and redistribution
limits are in `UPSTREAM.md`.

The detector consumes one exact pair:

```text
/uav1/mid360/points_world   sensor_msgs/PointCloud2
/uav1/mid360/rays_checked   mid360_ray_msgs/CheckedRayBundle
```

`points_world` and checked rays are produced by the same per-ray endpoint
formula. The detector reads the cloud coordinate and rejects the frame if it
differs from the checked endpoint by more than the configured 1 cm software
tolerance. Per-point `scan_id` and unique `original_index` are also checked.

B0 retains the upstream-style scalar occupancy map, component close/far split,
two background maturity gates, floating exploration, separated-background
cleanup, OBB-center detection, and bounded soft VALID_RETURN/NO_RETURN free
updates. It has no track feedback, truth, scan opportunity, existence, or
target quarantine.

Canonical startup uses a target-free background warm-up. During at least 10 s,
observed components are background candidates; completion additionally
requires both original map-maturity gates. `nadir_seed` has been removed.

Important outputs under `/uav1/vofod_mid360`:

```text
detections
background_points       # sure occupied background only
free_voxels             # free-score visualization; never a background input
map_update_diagnostics  # includes warm-up/maturity state
map_revision_evidence
status
query_voxels
reset
```

Run the frozen B0 through the single canonical pipeline:

```bash
roslaunch tclv_evaluation b0_canonical.launch
```

Build/tests:

```bash
catkin build vofod_mid360
catkin run_tests vofod_mid360
catkin_test_results build/vofod_mid360
```

The full frozen contract and parameter sources are in
`docs/B0_FROZEN_SPEC.md` and `docs/B0_PARAMETER_PROVENANCE.md`.
