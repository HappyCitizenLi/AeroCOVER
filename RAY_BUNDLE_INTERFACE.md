# RayBundle interface

## Purpose

`mid360_ray_msgs/RayBundle` carries every ray that the sensor attempted to emit
in one update, including valid returns, no returns, below-minimum-range returns,
and invalid range samples. It supplements the legacy PointCloud2 stream; it does
not replace or reinterpret that stream.

Raw simulation and checked downstream topics are:

```text
/uav1/mid360/points_raw   sensor_msgs/PointCloud2
/uav1/mid360/rays_raw     mid360_ray_msgs/RayBundle
/uav1/mid360/scan_identity mid360_ray_msgs/ScanIdentity
/uav1/mid360/ray_source_diagnostics  diagnostic_msgs/DiagnosticArray
/uav1/mid360/points_valid  sensor_msgs/PointCloud2
/uav1/mid360/points_world  sensor_msgs/PointCloud2
/uav1/mid360/rays_checked  mid360_ray_msgs/CheckedRayBundle
/uav1/mid360/ray_diagnostics  diagnostic_msgs/DiagnosticArray
```

Downstream algorithms must consume this common interface and must not recover a
direction from a legacy `(0,0,0)` PointCloud2 endpoint.

## Messages

### `mid360_ray_msgs/Ray`

```text
uint8 NO_RETURN=0
uint8 VALID_RETURN=1
uint8 BELOW_MIN_RANGE=2
uint8 INVALID_RANGE=3
uint8 UNKNOWN_STATUS=255

float32 dir_x
float32 dir_y
float32 dir_z
float32 range
float32 intensity
uint32 offset_time_ns
uint32 pattern_index
uint8 tag
uint8 line
uint8 return_status
```

### `mid360_ray_msgs/RayBundle`

```text
std_msgs/Header header
uint32 scan_id
uint32 pattern_start_index
float32 min_range
float32 max_range
mid360_ray_msgs/Ray[] rays
```

### `mid360_ray_msgs/CheckedRay`

`CheckedRay` supplements, rather than changes, the raw message ABI. It contains
the raw `Ray` unchanged, its source-array `original_index`, world-frame
`geometry_msgs/Point origin` and unit `geometry_msgs/Vector3 direction`, plus
explicit direction/transform-valid flags.

### `mid360_ray_msgs/ScanIdentity`

The companion binds a legacy PointCloud2 to `RayBundle.scan_id` without
changing the legacy point-field ABI. It records scan ID, pattern start, both
counts, and the point/ray source stamps. Producers publish it only after both
payloads were formed from one scan; the preprocessor validates all fields.

### `mid360_ray_msgs/CheckedRayBundle`

The checked bundle retains the raw scan ID, pattern index, range bounds and
source header. Its own header uses the configured output frame (default
`world`), and it records both `source_mode` and `ray_time_geometry_mode`. This
separate type is necessary because a moving sensor cannot represent 20,000
different origins with one `RayBundle.header`.

## Field invariants

- `header.frame_id` is the physical ray frame. Every `dir_*` vector is expressed
  in this frame.
- Each finite emitted direction is non-zero and satisfies
  `abs(norm(direction) - 1) < 1e-4`.
- `header.stamp` is the nominal first-ray time for the bundle and matching
  legacy PointCloud2. In default `snapshot` mode the
  historical timestamp/update sequence is retained; in explicit
  `per_ray_pose` mode it is authoritative Gazebo `SimTime` captured with the
  scan-start pose and twist under the physics mutex.
- `offset_time_ns` is an integer nanosecond offset relative to
  `header.stamp`; the first ray has offset zero and offsets are monotonic.
- `pattern_index` is the index in the complete CSV scan pattern, before output
  downsampling. It wraps modulo the pattern length.
- `pattern_start_index` equals the first ray's `pattern_index`.
- `scan_id` increases once per Gazebo sensor update and wraps as a `uint32`.
- ROS1 `Header.seq` is publisher-managed and is not the business scan key;
  `ScanIdentity.scan_id` explicitly links the cloud to `RayBundle.scan_id`.
- `range` is directly usable only when `return_status == VALID_RETURN`.
  Non-valid states publish `range=0` in this interface.
- The simulation source currently emits `tag=0`, matching the legacy type-2
  PointCloud2 behavior. `line` comes from the scan pattern (`index % 4`).

## Simulation direction convention

The fork intentionally reuses the exact direction operation used for Gazebo
raycasting and legacy point generation:

```cpp
ignition::math::Quaterniond rotation;
rotation.Euler(0.0, rotate_info.zenith, rotate_info.azimuth);
direction = rotation * ignition::math::Vector3d::UnitX;
```

The CSV third column is a polar angle from `+Z`; the existing loader stores it as
`csv_zenith - pi/2` before applying it as an Ignition pitch. Consequently, for
the raw CSV angle `theta` and azimuth `alpha`, the resulting direction is:

```text
[sin(theta) cos(alpha), sin(theta) sin(alpha), cos(theta)]
```

It is not correct to apply `z=sin(rotate_info.zenith)` to the already shifted
member. That would invert the vertical axis. The first Mid-360 CSV ray is used as
a regression check and has a positive Z component of approximately `0.78975`.

## Return-state classification

Classification uses the unmodified Gazebo `raw_range`, before the legacy
PointCloud2 path replaces non-valid ranges by zero:

```text
!isfinite(raw_range) or raw_range <= 0        -> INVALID_RANGE
0 < raw_range <= min_range                    -> BELOW_MIN_RANGE
raw_range >= max_range - epsilon              -> NO_RETURN
otherwise                                     -> VALID_RETURN
```

The original PointCloud2 publishing branch is kept independently. It still emits
zero endpoints for its non-valid samples, preserving upstream behavior.

## Timing policy

The plugin accepts:

```xml
<use_csv_time>true</use_csv_time>
<ray_point_rate>200000</ray_point_rate>
```

CSV timing is used only when it is finite, monotonic, and consistent with the
configured point rate. The audited `mid360-real-centr.csv` time column is the
integer sequence `1..800000`; treating it as seconds is physically impossible.
The simulation therefore reports `uniform_time_fallback` and computes offsets
from the configured point rate. Absolute ROS nanoseconds are never stored in a
floating-point Ray field.

The simulation geometry modes are deliberately narrow:

```text
snapshot (default):
  legacy SetPoints equations and one instantaneous collision snapshot

per_ray_pose (explicit opt-in only):
  constant world-frame parent-link twist extrapolation for each ray
  one ODE collision update against a scene assumed static for the scan
```

`per_ray_pose` requires
`<per_ray_pose_static_scene_opt_in>true</per_ray_pose_static_scene_opt_in>`;
unknown modes or a missing opt-in are rejected rather than downgraded. Its
scan-start `SimTime`, pose/twist capture, 20,000-ray construction and collision
update are protected by Gazebo's physics-update recursive mutex. The mode does
not model moving collision bodies, acceleration, jerk, or changing twist.
Nothing opts legacy models into this behavior: `snapshot` remains the default.

## Source publisher diagnostics

When RayBundle publication is enabled, the plugin publishes one
`diagnostic_msgs/DiagnosticArray` per bundle on
`/uav1/mid360/ray_source_diagnostics`. The first status is named
`mid360_ray_bundle/sim_exact`; `hardware_id=sim_exact`, and its message is either
`csv_time` or `uniform_time_fallback`. It contains:

```text
ray_count
valid_return_count
no_return_count
below_min_range_count
invalid_count
direction_norm_error_max
timestamp_monotonic_ratio
bundle_duration
exact_direction_available
source_mode
ray_time_geometry_mode
motion_model
scene_assumption
linear_speed_mps
angular_speed_radps
max_offset_sec
```

The first-round validator checks that the four return counts sum to
`ray_count`, the direction error is below `1e-4`, timestamp monotonicity is
exactly one, the uniform-time duration is `0.099995 s` for 20,000 rays at
200 kpoint/s, and the source/geometry modes are `sim_exact`/`snapshot`.
For explicit `per_ray_pose`, source diagnostics instead bind
`motion_model=constant_twist_world_velocity` and
`scene_assumption=static_scene_only` to the same RayBundle header.

## Preprocessor contract

`mid360_ray_preprocessor` approximately synchronizes each raw cloud and bundle.
It copies every accepted point record byte-for-byte, appends a `uint32
original_index`, and rejects non-finite coordinates, zero endpoints, points
outside the strict bundle range, and points in the configured observer self
box. Filtering the legacy cloud never removes a ray from `rays_checked`.

The two derived outputs form an ExactTime pair for downstream consumers:
`points_valid.header.stamp` is set to the matched
`RayBundle.header.stamp`, exactly equal to
`rays_checked.header.stamp`. The valid cloud keeps its physical source
`frame_id`, field layout, point bytes and `original_index`; only this
derived timestamp is canonicalized. The legacy `points_raw` header is not
modified.

In `snapshot` mode one transform at the first-ray stamp is used explicitly. In
`per_ray_pose` mode the node performs only start/end TF lookups and interpolates
translation and quaternion rotation in memory at every `offset_time_ns`; it
does not issue 20,000 blocking TF queries. A missing transform suppresses the
checked bundle, leaves successful point filtering available, and is reported
with `tf_missing_ratio=1`. This preprocessor interpolation is separate from the
simulation plugin's opt-in constant-twist collision geometry. In either case,
`points_valid` is only the filtered legacy point cloud with source bytes/frame
preserved and an `original_index` appended; it is **not** a deskewed point
cloud. Per-ray world origins/directions live in `rays_checked`.

Diagnostics distinguish the endpoint-query span from transform-source
sampling. `tf_query_span_sec` is zero in snapshot mode and equals the bundle
duration in `per_ray_pose`. By default
`tf_source_sample_max_gap_sec=not_observed`, because tf2 does not expose the
contributing sample spacing. The historical `tf_max_interval_sec` key remains
only as a labelled deprecated alias of the query span and must not be
interpreted as a measured TF sample gap.

S06 explicitly enables an observation-only cadence audit for the dynamic
`world -> uav1/fcu` edge on `/tf`. A separate callback queue records unique
message source stamps and can report a covering left/right pair plus the
largest adjacent observed gap. These values describe the configured topic edge
only:

```text
tf_source_cadence_semantics=observed_tf_topic_edge
tf_source_cadence_relation_to_lookup=not_tf2_actual_brackets
```

They are **not** tf2's private samples or actual interpolation brackets and do
not alter the two-lookup path. When the audit is disabled, no extra subscriber
is created and the max-gap value remains `not_observed`.

The consolidated `/uav1/mid360/ray_diagnostics` status reports point rejection
counts, source return counts, direction/timestamp checks, input index alignment,
TF lookup count/gap, exact-direction capability and both source/geometry modes.

The B-T4 static-wall gate is available to check this contract with independent
Gazebo P3D link truth and a scan-start snapshot counterfactual. It covers
static-scene observer geometry, not moving objects, MRS/PX4 or hardware.

## Source modes

The common semantic modes are:

```text
sim_exact
hw_spherical_exact
calibrated_fallback
```

`sim_exact` is supplied by the controlled Gazebo fork. The workspace also
supplies a controlled hardware-adapter software path based on
`livox_ros_driver2` commit `6b9356c` and Livox-SDK2 commit `6a94015`. With
`pcl_data_type=3`, the hook reads the untouched packet's raw
`depth/theta/phi/reflectivity/tag`, `time_type`, and all eight timestamp bytes
before upstream spherical-to-Cartesian conversion. RayBundle retains the
upstream point-cloud frame boundary; the adapter does not introduce a second
coordinate-frame interpretation. The eight-byte timestamp and its type remain
in driver frame state/diagnostics rather than changing the RayBundle ABI.
Focused tests cover the spherical angle convention, zero-depth direction
retention, return classification, timing provenance and alignment rejection.

For PTP/gPTP or GPS packets the selected frame stamp must equal the valid
decoded device timestamp. A no-sync packet instead receives a host
system-clock receipt stamp solely for compatibility framing. That stamp is
not device time, cannot support per-ray motion compensation, and is rejected
by the exact capability gate.

For each aligned frame, the driver emits diagnostics and RayBundle with the
same stamp and uses a frame-start `powerup_cnt` snapshot. Diagnostics additionally
carry `frame_scan_id`, `frame_pattern_start_index`, and `frame_ray_count`. The
probe first pairs by exact stamp, then requires those values to match RayBundle
`scan_id`, `pattern_start_index`, and ray count. Epoch start/reset clears every
unpaired diagnostic/ray pending entry, preventing a cached half from crossing
boots under a reused stamp. It binds device/serial/firmware/restart identity
within a trial and requires two physical power cycles with distinct
`powerup_cnt` before `hw_spherical_exact`. Promotion also requires fresh inputs
and valid synchronized PTP/gPTP or GPS metadata.

The probe republishes its mode as a heartbeat and publishes fallback on
shutdown. The hardware preprocessor accepts mode only from
`/mid360_spherical_capability_probe`, rejects dynamic `sim_exact` before it can
refresh heartbeat, and preserves the default 3 s wall-time lease. Exact-mode
lease expiry is sticky fallback until a fresh authorized fallback heartbeat
precedes requalification. The integrated hardware launch fixes
`xfer_format=0`. No physical Mid-360 was available, so firmware identity,
zero-depth angle retention and restart reproducibility remain unverified;
`hw_spherical_exact` must stay disabled and must not be claimed.

Until physical capability passes, `calibrated_fallback` is the safe mode. At
present it disables exact-direction/no-return semantics; it is not a
statistically calibrated coverage generator, and that generator remains
unimplemented. Exact directions must never be reconstructed or fabricated
from `line+timestamp` and the simulation CSV. Physical hardware remains
unverified.

## Downstream policy for no-return rays

`RayBundle` is a raw observation interface; the Gazebo publisher does not own or
modify an occupancy map. A `NO_RETURN` ray is not hard proof that every voxel to
maximum range is free; the downstream VoFOD-Mid360 mapper uses it as bounded,
weighted soft free-space evidence.

The faithful B0 mapping contract is:

```text
VALID_RETURN:
  free_end = min(range - valid_return_safety_margin, raycast_max_distance)
  soft weight = free_update_weight_valid_return

NO_RETURN:
  free_end = min(raycast_max_distance, reliable_no_return_distance)
  soft weight = free_update_weight_no_return > 0

BELOW_MIN_RANGE / INVALID_RANGE / UNKNOWN_STATUS:
  excluded from free-space updates by default
```

For a strict reproduction of the original VoFOD endpoint,
`reliable_no_return_distance >= raycast_max_distance`, so the `NO_RETURN`
endpoint is `raycast_max_distance`. A shorter reliability cap is a separately
reported policy choice. Body and self-occlusion masks are applied before either
valid-return or no-return updates. Traversed voxels accumulate weighted scores;
they are never hard-set free by one ray.

Because a no-return Ray carries `range=0`, consumers must select it with
`return_status == NO_RETURN` and construct the endpoint from `dir_*` plus the
configured distance. They must not use `range == 0` as the classifier: below-min
and invalid samples also carry zero range.

The same no-return ray has two additional, separate uses: it counts as an actual
emission in the scan-opportunity layer, and it may provide multi-ray,
multi-window soft negative evidence for track existence. Mapping evidence,
scan opportunity, and track evidence must not share one opaque decision or make
a single no-return ray equivalent to target absence.
