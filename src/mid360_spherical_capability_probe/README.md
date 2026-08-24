# Mid-360 spherical capability probe

This package consumes the controlled Livox fork's raw-spherical diagnostics
and `mid360_ray_msgs/RayBundle`. It fails closed unless all of the following are
observed on a physical Mid-360 with a known firmware version:

- the data-type-3 command succeeded and packets are actually spherical;
- the stream contains one device with an available serial number and firmware;
- packet metadata identifies synchronized gPTP/PTP or GPS time and preserves
  the original eight-byte timestamp; a no-sync host-receipt stamp is rejected
  for exact qualification and motion-compensation evidence;
- driver diagnostics and RayBundle are paired by one exactly equal frame stamp,
  then cross-checked for equal `scan_id`, `pattern_start_index`, and ray count;
- packet and sample thresholds are met using increments observed within the
  current trial, rather than process-lifetime counters;
- zero-depth samples exist and retain finite, unit, non-trivial directions that
  change by at least the configured angular span;
- adjacent zero-depth-to-valid or valid-to-zero samples have a strictly
  positive bounded time interval and a bounded angular step;
- the same result qualifies in at least two explicitly separated restart
  trials on the same serial number and firmware, with two different
  `powerup_cnt` values.

Until those conditions pass, the latched source-mode topic is
`calibrated_fallback`. A single synthetic or live trial can never enable
`hw_spherical_exact` under the default two-restart contract.

## Trial and identity contract

The driver snapshots `powerup_cnt` when each RayBundle frame begins, then
publishes that frame's diagnostic and RayBundle with the same stamp. The probe
pairs those two messages by exact stamp and verifies that the diagnostic header
equals its declared `frame_stamp_ns`. It then requires diagnostic
`frame_scan_id`, `frame_pattern_start_index`, and `frame_ray_count` to match the
RayBundle `scan_id`, `pattern_start_index`, and number of rays. The first
accepted pair in a trial
establishes counter baselines and binds the trial to one device type, serial
number, firmware, and `powerup_cnt`; any within-trial identity or marker change
fails closed. Qualification uses deltas for spherical packets and samples;
generic raw-packet activity cannot substitute for spherical-packet activity. A
restart marker change inside an unfinished trial discards the old epoch and
starts clean acquisition. Every epoch start/reset clears the pairer's unpaired
diagnostic, ray, and pending-order caches, preventing a cached half-frame from
being reused across boots merely because the numeric stamp repeats.

After `finish_trial`, the probe waits for a different `powerup_cnt` before it
will accept the next trial. Distinct trial IDs alone are not restart evidence.
Across the two qualifying trials, the device type, serial number, and firmware
must be identical, while the two restart markers must differ. The final JSON
binds all of these values to the decision.

For angle evidence, the probe independently reads RayBundle directions. Every
zero-depth direction must be finite and unit length, the configured proportion
must be non-trivial, and the zero-depth set must change direction. Temporal
continuity is evaluated specifically on adjacent `zero <-> valid` pairs, using
both `maximum_neighbor_time_gap_ns` and `maximum_neighbor_angle_rad`; equal or
reversed timestamps fail. The default thresholds are documented in
`config/default.yaml`.

## Physical pipeline

Run the controlled driver, probe, and preprocessor together from the catkin
workspace:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch mid360_spherical_capability_probe hardware_spherical_pipeline.launch \
  trial_id:=mid360_fw_restart_01 \
  capability_output:=/tmp/mid360_spherical_capability.json

# After sufficient samples from the first boot:
rosservice call /mid360_spherical_capability_probe/finish_trial

# Physically power-cycle the same Mid-360. Wait until diagnostics report a
# different powerup_cnt, collect the second trial, then finish it:
rosservice call /mid360_spherical_capability_probe/finish_trial
rosservice call /mid360_spherical_capability_probe/write_report
```

The launch explicitly selects driver `xfer_format=0`, so `/livox/lidar` is the
PointCloud2 compatibility stream consumed by the preprocessor. It starts
hardware in `calibrated_fallback`, connects the probe's
latched `/uav1/mid360/ray_source_mode` output to the preprocessor, and uses the
single-device, zero-extrinsic type-3 driver configuration. The output JSON
includes device/serial/firmware and restart markers, type-3 acceptance,
spherical confirmation, trial-delta packet/sample/zero counts, finite and
non-trivial angle counts, angular span, adjacency continuity, alignment,
per-trial failure reasons, and the final exact-support/source-mode decision.

Before the second qualifying trial may promote exact mode, current driver
diagnostics, current RayBundle traffic, and a current exact-stamp paired frame
must all pass the default 2 s freshness deadlines. The promotion check also
revalidates synchronized timestamp metadata, the authenticated
serial/firmware/restart marker, type-3/spherical/single-device state, and the
alignment baseline. A no-sync packet uses host receipt time only for compatible
framing and can never pass this synchronized-time gate.

An operator must record that the marker change came from a real physical power
cycle. A process restart, a renamed trial, or two synthetic result objects is
not a substitute.

## Runtime revocation

Even after two physical trials qualify, exact mode remains conditional. The
watchdog immediately publishes latched `calibrated_fallback` if driver
diagnostics, RayBundle data, or a same-stamp diagnostic/ray pair times out. It
also revokes on unsynchronized/invalid timestamp provenance; a changed serial,
firmware, or authenticated restart marker; loss of type-3 acceptance,
spherical packets, single-device status, or exact frame alignment; or any new
alignment mismatch. Revocation is fail-closed for that probe process and is
recorded in the report; the device must be restarted and requalified rather
than silently regaining exact status.

The watchdog republishes the current mode as a heartbeat, including startup
fallback. On shutdown the node attempts a final latched
`calibrated_fallback` publication and records `capability_probe_shutdown` if
exact had been active. The connected preprocessor uses a default 3 s wall-time
lease: heartbeat expiry produces sticky fallback, and a later exact heartbeat
is ignored until a fresh fallback heartbeat begins requalification.
The hardware preprocessor accepts this heartbeat only from the exact ROS node
name `/mid360_spherical_capability_probe`; unauthorized publishers cannot renew
the lease, and dynamic `sim_exact` is rejected on the hardware path.

## Test and current status

```bash
source /opt/ros/noetic/setup.bash
catkin run_tests mid360_spherical_capability_probe
catkin_test_results build/mid360_spherical_capability_probe
```

The suite prevents the hardware
pipeline from drifting back to a CustomMsg-only configuration and exercises
exact-stamp plus scan/pattern/count diagnostic/ray pairing, synchronized-time
provenance, two restart
markers, trial-delta counters, identity/firmware binding, angular and adjacency
failure cases, restart-epoch gating, pre-promotion freshness, and runtime-safe
diagnostics.
Generated test results are not retained. Unit tests demonstrate logic behavior
only: no physical Mid-360, firmware
response, or zero-depth `theta`/`phi` stream has been observed, so hardware
capability remains **UNVERIFIED** and `hw_spherical_exact` must not be claimed.

`calibrated_fallback` is currently a fail-closed source-mode label only. A
statistical coverage model/generator has not been implemented; fallback does
not manufacture exact no-return directions and must not be presented as an
equivalent hardware visibility opportunity stream.
# HW-CAL record-only entry point

`hw_record_only.launch` records the raw/checked rays, raw/world points,
observer pose, optional target truth, time-sync diagnostics and TF without
starting a detector or tracker.  Select `trial_id:=HW-CAL01` through
`HW-CAL05`, provide an explicit `bag_path`, and override the observer/truth
topics for the measurement system in use.  A missing target-truth publisher
is acceptable only for HW-CAL01/02; it is not sufficient for target trials.
