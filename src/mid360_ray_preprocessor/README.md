# Mid-360 ray preprocessor

This package synchronizes the raw compatibility `PointCloud2` and the complete
emitted `RayBundle` through their producer-authored `ScanIdentity`. It publishes:

- `/uav1/mid360/points_valid`: the original point records that pass finite,
  strict range, zero-point, and observer self-box checks. Every source field is
  copied byte-for-byte and a `uint32 original_index` field is appended.
- `/uav1/mid360/rays_checked`: `CheckedRayBundle` in `world` (configurable),
  with the unmodified source `Ray`, source index, and per-ray world origin and
  unit direction.
- `/uav1/mid360/points_world`: canonical world cloud with `x/y/z`, `intensity`,
  `original_index`, `offset_time_ns`, and `scan_id`.
- `/uav1/mid360/ray_diagnostics`: filtering, return-state, direction, timing,
  source provenance, and TF-availability diagnostics.

The raw cloud, bundle, and identity use ExactTime. `ScanIdentity` binds
`RayBundle.scan_id`, both source stamps, pattern start, and both counts. The
preprocessor also checks source order, duplicate indices, valid-return matching,
and point/ray range before publishing one derived triplet:

```text
points_valid.header.stamp
  = points_world.header.stamp
  = rays_checked.header.stamp
```

`points_valid` keeps the source physical frame, field layout and accepted point
bytes; `points_raw` keeps its original header. Any pair, transform, or endpoint
failure suppresses all three derived outputs.

The existing `Ray`/`RayBundle` ABI is not changed. The supplementary
`CheckedRay` messages carry geometry that cannot be represented by a single
bundle header when the observer moves during a scan.

## Source-mode authority

`source_mode_topic` is an optional `std_msgs/String` subscription intended for
a latched capability source. The only canonical values are:

| Mode | Exact directions | Contract |
| --- | --- | --- |
| `sim_exact` | yes | Controlled simulator emits every attempted ray. |
| `hw_spherical_exact` | yes | Physical spherical stream passed the independent two-restart capability probe. |
| `calibrated_fallback` | no | Fail-closed hardware mode; no exact no-return opportunity claim. |

The canonical mode is copied into every checked bundle and diagnostic. It is
also authoritative for `exact_direction_available`; a conflicting legacy
boolean cannot promote fallback to exact. Unsupported topic values are logged
and ignored, leaving the last valid mode unchanged.

The default configuration keeps `source_mode_topic: ""`, so no additional ROS
subscriber is created and existing simulation launches retain the static
`source_mode: sim_exact` behavior. Dynamic `sim_exact` is a separate input and
is forbidden by the hardware configuration. Hardware must instead start
fail-closed:

```bash
roslaunch mid360_ray_preprocessor preprocessor.launch \
  config:=$(rospack find mid360_ray_preprocessor)/config/hardware_spherical.yaml \
  source_mode:=calibrated_fallback \
  source_mode_topic:=/uav1/mid360/ray_source_mode
```

The integrated hardware path is:

```bash
roslaunch mid360_spherical_capability_probe hardware_spherical_pipeline.launch
```

That launch connects the probe's latched mode to the preprocessor. The probe
may publish `hw_spherical_exact` only after its physical two-power-cycle
contract passes, including valid synchronized PTP/gPTP or GPS timestamp
metadata and fresh same-stamp diagnostic/RayBundle pairing. A no-sync host
receipt stamp is not device or motion-compensation time and cannot enable this
mode. The probe watchdog republishes the current mode as a heartbeat and can
revoke the runtime back to `calibrated_fallback`.

When `source_mode_topic` is enabled, the preprocessor uses a wall-time lease;
the hardware config and launch default to 3 s. If an exact heartbeat expires,
the mode changes to `calibrated_fallback`, exact directions are disabled, and
revocation is sticky. A delayed exact heartbeat cannot restore exact mode; a
fresh fallback heartbeat must first clear the lease revocation before the
probe can requalify. Probe shutdown also attempts a final latched fallback.
The hardware configuration additionally sets
`source_mode_authority_node: /mid360_spherical_capability_probe` and checks the
ROS connection publisher name before accepting a mode or refreshing its
heartbeat. Messages from any other publisher and every dynamic `sim_exact`
message are ignored without extending the lease. This does not alter static
simulation mode when `source_mode_topic` is disabled.
The supplied hardware configuration deliberately starts in fallback and
contains placeholder self-exclusion dimensions; measure the real airframe mask
before using physical-flight evidence.

## TF modes

`snapshot` explicitly uses one transform at the first-ray timestamp. It
preserves the legacy Gazebo snapshot geometry contract.

`per_ray_pose` performs exactly two blocking TF lookups (bundle start/end),
then interpolates translation and quaternion rotation in memory using each
ray's `offset_time_ns`. It never performs a blocking lookup per ray. A TF
failure suppresses the complete derived triplet and is reported with
`tf_missing_ratio=1`.

Diagnostics report `tf_query_span_sec` separately from
`tf_source_sample_max_gap_sec`. By default the latter is `not_observed`
because tf2 does not expose which buffered samples satisfied an interpolated
lookup. The legacy `tf_max_interval_sec` field is an explicitly deprecated
query-span alias, not evidence of the dynamic broadcaster's sample cadence.

For a moving-observer fixture, an optional observation-only audit can monitor
one configured dynamic edge directly on a `tf2_msgs/TFMessage` topic:

```yaml
tf_source_cadence_observation_enabled: true
tf_source_cadence_topic: /tf
tf_source_cadence_parent_frame: world
tf_source_cadence_child_frame: uav1/fcu
tf_source_cadence_cache_capacity: 512
tf_source_cadence_subscriber_queue_size: 200
```

The observer uses its own callback queue and one-thread `AsyncSpinner`, so a
bundle callback waiting for its end-time tf2 lookup cannot prevent source
stamps from entering the audit cache. The bounded cache stores unique source
header stamps in time order. For bundle interval `[t0,t1]`, it selects the
latest observed stamp at or before `t0`, the earliest observed stamp at or
after `t1`, and reports the largest adjacent gap across that covering range.
It also reports missing-left/missing-right/no-sample status and cumulative
duplicate, out-of-order, and eviction counts.

This side channel does **not** alter the two-lookup interpolation path and does
not reveal tf2's private interpolation brackets. Its diagnostics therefore
state exactly:

```text
tf_source_cadence_semantics=observed_tf_topic_edge
tf_source_cadence_relation_to_lookup=not_tf2_actual_brackets
```

When disabled, the old contract remains: the max-gap value is
`not_observed`, and no extra TF subscriber or spinner is created. Moving-
observer fixtures may enable the audit explicitly for `world -> uav1/fcu`;
legacy snapshot launches leave it disabled. Preprocessor geometry acceptance
must not be rewritten as tracker or scene-level performance.

The cadence fields are:

```text
tf_source_sample_bracket_status
tf_source_sample_bracketed
tf_source_sample_max_gap_sec
tf_source_sample_left_bracket_ns
tf_source_sample_right_bracket_ns
tf_source_sample_covering_count
tf_source_stamp_cache_unique_count
tf_source_stamp_cache_capacity
tf_source_duplicate_stamp_count
tf_source_out_of_order_stamp_count
tf_source_evicted_stamp_count
```

## Run and test

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch mid360_ray_preprocessor preprocessor.launch

catkin build mid360_ray_msgs mid360_ray_preprocessor
catkin run_tests mid360_ray_preprocessor
catkin_test_results build/mid360_ray_preprocessor
```

Two synthetic rostests exercise both the snapshot transform and start/end
SE(3) interpolation with a cloud whose Livox-style fields must survive
filtering unchanged. Two additional headless Gazebo rostests reuse the
controlled `livox_laser_simulation` empty/single-wall harness at the production
20,000-ray bundle size. They verify zero valid points in empty space and exact
raw-field/original-index/`VALID_RETURN` alignment at the wall.

The source-mode tests additionally cover the canonical mode whitelist and a
live ROS subscription: latched fallback becomes authoritative, an invalid
update is ignored, a valid exact heartbeat is leased, lease expiry forces
sticky fallback, and a fresh fallback heartbeat is required before
requalification. The integrated hardware launch fixes driver `xfer_format=0`
so `/livox/lidar` remains PointCloud2. These tests address propagation and
fail-closed semantics, not the physical Mid-360 capability required to publish
hardware exact mode. Generated test results are not retained.
