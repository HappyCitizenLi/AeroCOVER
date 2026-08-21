# Physical Mid-360 spherical-direction capability

## Current verdict

```text
Physical Mid-360 tested: false
Firmware tested: UNAVAILABLE
exact_no_return_direction_supported: false
ray_source_mode: calibrated_fallback
```

The adapter, fail-closed probe, and source-mode propagation described here are
implemented software infrastructure. No physical Mid-360 is available in this
workspace, so this document
does **not** establish that any firmware preserves useful `theta/phi` for
`depth=0`.
`hw_spherical_exact` must remain disabled until the physical procedure below
produces two qualifying, independently power-cycled trials.

`calibrated_fallback` currently disables exact/no-return direction semantics.
It is not yet a calibrated statistical coverage generator.

## Audited upstream baseline

- controlled driver fork:
  `src/livox_ros_driver2_rayfork`, upstream commit
  `6b9356cadf77084619ba406e6a0eb41163b08039`;
- Livox-SDK2: `/home/uav/Livox-SDK2`, commit
  `6a940156dd7151c3ab6a52442d86bc83613bd11b`;
- driver license: retained Livox MIT and bundled third-party notices;
- hardware configuration:
  `src/livox_ros_driver2_rayfork/config/MID360_spherical_config.json`, with
  `pcl_data_type: 3`, one configured Mid-360, and zero driver extrinsic.

The official spherical structure provides `depth`, `theta`, `phi`,
`reflectivity`, and `tag`. The SDK packet also provides an eight-byte raw
timestamp, `time_type`, `time_interval`, `dot_num`, and data type. As in
upstream driver code, `line` is derived as `point_index % line_num`; it is not
a field of `LivoxLidarSpherPoint`.

## Adapter behavior

`SphericalRayBridge` observes each validated `RawPacket` on the same worker
thread immediately before upstream `ProcessSphericalPoint()`. It copies each
packed spherical sample into aligned storage, preserving zero-depth angles,
and applies the upstream convention

$$
\theta=\mathrm{raw\_theta}\frac{\pi}{18000},\qquad
\phi=\mathrm{raw\_phi}\frac{\pi}{18000},
$$

$$
\mathbf d=
\begin{bmatrix}
\sin\theta\cos\phi &
\sin\theta\sin\phi &
\cos\theta
\end{bmatrix}^{\!T}.
$$

`depth=0` becomes `NO_RETURN` without erasing this unit direction. Depth,
reflectivity, tag, derived line, point interval, selected frame stamp,
relative nanosecond offset, and scan-pattern index are retained in the common
RayBundle semantics. Before conversion, the bridge also copies `time_type`
and all eight raw timestamp bytes into its packet/frame state and exposes
their provenance in the paired diagnostics.

For synchronized gPTP/PTP (`time_type=1`) or GPS (`time_type=2`), the decoded
raw timestamp must be finite, nonzero, and equal to the selected frame stamp.
For no-sync packets (`time_type=0`), the compatibility frame stamp is only the
host system-clock receipt time; the original eight bytes remain auditable but
are not interpreted as device time. A no-sync host receipt stamp must never be
used as device time, per-ray motion-compensation time, or exact hardware
evidence. Exact qualification requires valid synchronized PTP/gPTP or GPS
metadata.

The bridge flushes at the exact upstream PointCloud2 frame boundary and
requires equal base timestamps and point/ray counts. It publishes diagnostics
and RayBundle with the same frame stamp. The diagnostic carries the
`powerup_cnt` snapshot captured when that frame began, rather than a later
state callback's marker, and also reports that frame's `scan_id`,
`pattern_start_index`, and ray count. Truncated payloads,
`dot_num=0`, unsupported types, zero/non-monotonic point intervals, timestamp
overflow, a second device on the common topic, or frame misalignment fail
closed. The original PointCloud2/CustomMsg/PCL/IMU outputs remain available;
RayBundle publication is opt-in.

## Capability decision contract

The live probe consumes `/uav1/mid360/rays_raw` plus the driver's spherical
diagnostics and initially publishes latched `calibrated_fallback`. One trial
qualifies only if all of these conditions hold:

- device is Mid-360 and serial/firmware/power-up counter are available;
- type-3 command response succeeds and spherical packets are observed;
- timestamp metadata is valid and declares synchronized gPTP/PTP or GPS, not
  the no-sync host-receipt fallback;
- driver diagnostics and RayBundle are paired by exactly equal frame stamp,
  and the diagnostic header equals its declared `frame_stamp_ns`;
- the paired diagnostic's `frame_scan_id`, `frame_pattern_start_index`, and
  `frame_ray_count` exactly match the RayBundle `scan_id`,
  `pattern_start_index`, and `len(rays)`;
- only one device feeds the common RayBundle stream;
- the trial adds at least 10 spherical packets and 1,000 RayBundle samples;
- no new point/ray frame-alignment mismatch occurs during the trial;
- at least one zero-depth sample exists and every zero-depth direction is
  finite and unit length within `1e-4`;
- at least 90% of zero-depth directions are non-trivial and their direction
  span is at least `0.001 rad`;
- adjacent `zero <-> valid` pairs have strictly positive time gaps no greater
  than `1 ms`, angular steps no greater than `5 degrees`, and at least 80%
  continuity success.

The paired frame's power-up snapshot feeds the trial accumulator. Each trial
binds one device type, serial number, firmware, and restart marker; any
within-trial identity or marker change fails closed. The final exact decision
additionally requires two qualifying trials from the same serial number and
firmware with different SDK `powerup_cnt` markers. A restart-epoch gate ignores
all samples from the old boot after the first trial and resets an unfinished
trial if the boot marker changes. On every epoch start/reset it clears all
unpaired diagnostic/ray pending entries, so a cached half-frame with a reused
stamp cannot cross the boot boundary. Renaming a trial or calling
`finish_trial` twice cannot satisfy this requirement.

Before the probe first publishes exact mode, it performs a wall/steady-time
freshness check over the latest driver diagnostic, latest RayBundle, and latest
same-stamp paired frame (default driver/ray deadlines are both 2 s). It also
rechecks synchronized timestamp provenance, authenticated identity/restart
marker, type-3/spherical/single-device state, and the alignment baseline. A
stale or unsafe candidate remains `calibrated_fallback` even if two stored
trial summaries qualify.

After promotion, a watchdog binds the authenticated serial, firmware, boot
marker, timestamp provenance, same-stamp frame pairing, and alignment counter.
Diagnostic/RayBundle/paired-frame timeout, identity change, new boot,
type-3/spherical loss, multi-device input, or a new alignment error immediately
latches `calibrated_fallback`; revocation is sticky for that probe process.
The watchdog republishes the active source mode as a heartbeat, including the
startup fallback. On shutdown the probe attempts one final latched fallback
publication and records `capability_probe_shutdown` when exact had been active.
The preprocessor grants exact mode only a default 3 s wall-time lease; an
expired heartbeat forces sticky fallback, and a delayed exact heartbeat cannot
clear the revocation until a fresh fallback heartbeat starts requalification.
In the hardware configuration, source-mode messages are accepted only when the
ROS publisher name is exactly `/mid360_spherical_capability_probe`; messages
from any other publisher are ignored before they can refresh the lease.
Dynamic `sim_exact` is also forbidden on this hardware authority path. Static
`sim_exact` remains available to simulation configurations with no dynamic
source-mode subscription.

## Safe build and software verification

Do not run the upstream `build.sh`; it deletes workspace build/devel/install
state. The bounded commands used here are:

```bash
catkin build livox_ros_driver2 --no-deps \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS1
catkin run_tests livox_ros_driver2 --no-deps

catkin build mid360_spherical_capability_probe --no-deps \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin run_tests mid360_spherical_capability_probe --no-deps

catkin build mid360_ray_preprocessor --no-deps \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin run_tests mid360_ray_preprocessor --no-deps
```

The test sources cover the driver bridge, capability decision core, launch
contract and preprocessor fail-closed behavior. Generated result files are not
retained in the current workspace. Software checks cannot substitute for a
physical firmware qualification.

## Required physical qualification procedure

1. Confirm that the configuration contains exactly one physical Mid-360 and
   zero driver extrinsic; express sensor pose through TF.
2. Launch
   `mid360_spherical_capability_probe/hardware_spherical_pipeline.launch`.
   Its default `xfer_format=0` is required so `/livox/lidar` remains the
   PointCloud2 input paired with the raw-ray stream by the preprocessor.
3. Confirm diagnostics report synchronized gPTP/PTP or GPS timestamp metadata;
   no-sync host receipt time is disqualifying. Record the raw eight-byte
   timestamp, `time_type`, frame-stamp source, device serial, firmware, type-3
   response, spherical confirmation, packet/sample/zero counts, direction
   span, zero-valid adjacency continuity, alignment, and first `powerup_cnt`.
4. After enough clean samples, call the probe's `finish_trial` service.
5. Physically power-cycle the same lidar. Do not restart or rename a trial as a
   substitute. Wait for a different `powerup_cnt` before collecting trial 2.
6. Finish trial 2 only while diagnostic, RayBundle, and their exact-stamp pair
   are fresh; then write the JSON report. Only a report whose every trial
   qualifies and whose final fields say both
   `exact_no_return_direction_supported=true` and
   `ray_source_mode=hw_spherical_exact` may enable exact hardware semantics.
7. Repeat after any firmware, device, network, timing, or driver change.

If any requirement fails, keep `calibrated_fallback`. Never reconstruct exact
hardware directions from `line + timestamp` and a simulation CSV; no validated
phase-synchronization mechanism exists here.
