#!/usr/bin/env python3
"""Fail-closed Gazebo/ODE B-T4 wall proof for ``per_ray_pose`` geometry.

The test reconstructs each emitted world ray from strictly bracketed, stamped
Gazebo P3D link truth.  A predicted front-face intersection must be returned at
the matching per-ray range.  The same measured range is also evaluated with a
scan-start (snapshot) pose:

* ``stationary`` is the zero-motion control; per-ray and snapshot endpoints
  must both hit and remain equivalent.
* ``yaw`` is the discriminating case; late rays must hit only under their
  timestamped poses, while the snapshot counterfactual has a large residual.

Unknown modes, missing diagnostics, missing pose brackets, sparse wall support,
or insufficient counterfactual separation are test failures, never skips.
"""

import bisect
import math
import sys
import threading
import time
import unittest

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from nav_msgs.msg import Odometry

from mid360_ray_msgs.msg import Ray, RayBundle


class ValidationError(RuntimeError):
    """A deterministic evidence-contract failure."""


def _bool_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _stamp_ns(stamp):
    return int(stamp.secs) * 1000000000 + int(stamp.nsecs)


def _finite(values):
    return all(math.isfinite(value) for value in values)


def _norm(vector):
    return math.sqrt(sum(component * component for component in vector))


def _normalize_quaternion(quaternion):
    norm = _norm(quaternion)
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValidationError("Gazebo truth contains a degenerate quaternion")
    return tuple(component / norm for component in quaternion)


def _slerp(first, second, ratio):
    first = _normalize_quaternion(first)
    second = _normalize_quaternion(second)
    dot = sum(a * b for a, b in zip(first, second))
    if dot < 0.0:
        second = tuple(-value for value in second)
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        return _normalize_quaternion(
            tuple(a + ratio * (b - a) for a, b in zip(first, second))
        )
    angle = math.acos(dot)
    sine = math.sin(angle)
    if abs(sine) <= 1.0e-12:
        return first
    first_weight = math.sin((1.0 - ratio) * angle) / sine
    second_weight = math.sin(ratio * angle) / sine
    return tuple(
        first_weight * a + second_weight * b for a, b in zip(first, second)
    )


def _rotate(quaternion, vector):
    x, y, z, w = _normalize_quaternion(quaternion)
    vx, vy, vz = vector
    # q * v * q^-1, expanded to avoid an optional tf dependency in the gate.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _yaw(quaternion):
    x, y, z, w = _normalize_quaternion(quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _percentile(values, quantile):
    if not values:
        raise ValidationError("cannot calculate a percentile from empty evidence")
    ordered = sorted(values)
    position = (len(ordered) - 1) * quantile
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    ratio = position - lower
    return ordered[lower] * (1.0 - ratio) + ordered[upper] * ratio


def _pose_from_odom(message):
    position = message.pose.pose.position
    orientation = message.pose.pose.orientation
    return (
        (position.x, position.y, position.z),
        _normalize_quaternion(
            (orientation.x, orientation.y, orientation.z, orientation.w)
        ),
    )


class PerRayWallValidator:
    """Collect and validate independent geometry evidence for one motion case."""

    def __init__(self):
        self.motion_case = str(rospy.get_param("~motion_case", "")).strip().lower()
        if self.motion_case not in ("stationary", "yaw"):
            raise ValidationError(
                "~motion_case must be exactly 'stationary' or 'yaw', got {!r}".format(
                    self.motion_case
                )
            )

        self.model_name = str(
            rospy.get_param("~model_name", "mid360_b_t4_per_ray_observer")
        )
        self.ray_topic = str(
            rospy.get_param("~ray_topic", "/b_t4/mid360/rays_raw")
        )
        self.diagnostics_topic = str(
            rospy.get_param(
                "~diagnostics_topic", "/b_t4/mid360/ray_source_diagnostics"
            )
        )
        self.odom_topic = str(
            rospy.get_param("~odom_topic", "/b_t4/observer/odom")
        )
        self.expected_frame = str(
            rospy.get_param("~expected_frame", "b_t4/mid360")
        )
        self.expected_world_frame = str(
            rospy.get_param("~expected_world_frame", "world")
        )
        self.expected_ray_count = int(rospy.get_param("~expected_ray_count", 20000))
        self.required_bundles = int(rospy.get_param("~required_bundles", 3))
        self.timeout_sec = float(rospy.get_param("~timeout_sec", 45.0))
        self.desired_yaw_rate = float(
            rospy.get_param("~desired_yaw_rate_radps", 1.0)
        )

        self.wall_plane_x = float(rospy.get_param("~wall_plane_x", 12.0))
        self.wall_interior_half_y = float(
            rospy.get_param("~wall_interior_half_y", 18.0)
        )
        self.wall_interior_half_z = float(
            rospy.get_param("~wall_interior_half_z", 8.0)
        )
        self.minimum_forward_x = float(
            rospy.get_param("~minimum_forward_x", 0.4)
        )
        self.maximum_expected_range = float(
            rospy.get_param("~maximum_expected_range", 30.0)
        )
        self.minimum_wall_candidates = int(
            rospy.get_param("~minimum_wall_candidates", 1000)
        )
        self.late_offset_sec = float(rospy.get_param("~late_offset_sec", 0.075))
        self.minimum_late_candidates = int(
            rospy.get_param("~minimum_late_candidates", 300)
        )
        self.minimum_discriminating_rays = int(
            rospy.get_param("~minimum_discriminating_rays", 100)
        )
        self.minimum_counterfactual_residual = float(
            rospy.get_param("~minimum_counterfactual_residual_m", 0.20)
        )

        self.maximum_odom_gap_sec = float(
            rospy.get_param("~maximum_odom_gap_sec", 0.02)
        )
        self.angular_speed_tolerance = float(
            rospy.get_param("~angular_speed_tolerance_radps", 0.15)
        )
        self.maximum_linear_speed = float(
            rospy.get_param("~maximum_linear_speed_mps", 0.02)
        )
        self.stationary_maximum_angular_speed = float(
            rospy.get_param("~stationary_maximum_angular_speed_radps", 0.02)
        )
        self.maximum_position_drift = float(
            rospy.get_param("~maximum_scan_position_drift_m", 0.01)
        )
        self.endpoint_p95_tolerance = float(
            rospy.get_param(
                "~endpoint_p95_tolerance_m",
                0.02 if self.motion_case == "stationary" else 0.06,
            )
        )
        self.endpoint_max_tolerance = float(
            rospy.get_param(
                "~endpoint_max_tolerance_m",
                0.05 if self.motion_case == "stationary" else 0.12,
            )
        )
        self.minimum_counterfactual_improvement = float(
            rospy.get_param("~minimum_counterfactual_improvement_m", 0.14)
        )
        self.minimum_counterfactual_ratio = float(
            rospy.get_param("~minimum_counterfactual_ratio", 4.0)
        )

        positive_values = (
            self.expected_ray_count,
            self.required_bundles,
            self.timeout_sec,
            self.wall_plane_x,
            self.minimum_forward_x,
            self.maximum_expected_range,
            self.minimum_wall_candidates,
            self.late_offset_sec,
            self.minimum_late_candidates,
            self.maximum_odom_gap_sec,
            self.endpoint_p95_tolerance,
            self.endpoint_max_tolerance,
        )
        if not _finite(tuple(float(value) for value in positive_values)) or any(
            float(value) <= 0.0 for value in positive_values
        ):
            raise ValidationError("B-T4 evidence thresholds must be finite and positive")
        if self.motion_case == "yaw" and (
            not math.isfinite(self.desired_yaw_rate)
            or abs(self.desired_yaw_rate) <= self.angular_speed_tolerance
        ):
            raise ValidationError("yaw case requires a finite, non-zero commanded yaw rate")

        self._lock = threading.RLock()
        self._finished = threading.Event()
        self._motion_ready = threading.Event()
        self._failure = None
        self._success = False
        self._commanded = False
        self._ready_sample_streak = 0

        self._odom = []
        self._odom_stamps = []
        self._pending_bundles = {}
        self._diagnostics = {}
        self._validated_scan_ids = set()
        self._summaries = []

        self._odom_subscriber = rospy.Subscriber(
            self.odom_topic, Odometry, self._odom_callback, queue_size=2000
        )
        self._command_model_and_wait_for_truth()
        self._ray_subscriber = rospy.Subscriber(
            self.ray_topic, RayBundle, self._ray_callback, queue_size=20
        )
        self._diagnostic_subscriber = rospy.Subscriber(
            self.diagnostics_topic,
            DiagnosticArray,
            self._diagnostics_callback,
            queue_size=20,
        )

        rospy.loginfo(
            "B-T4 per-ray wall validator armed: case=%s model=%s rays=%s odom=%s",
            self.motion_case,
            self.model_name,
            self.ray_topic,
            self.odom_topic,
        )

    def _command_model_and_wait_for_truth(self):
        service_name = "/gazebo/set_model_state"
        try:
            rospy.wait_for_service(service_name, timeout=min(20.0, self.timeout_sec))
        except rospy.ROSException as exc:
            raise ValidationError("Gazebo state service unavailable: {}".format(exc))

        state = ModelState()
        state.model_name = self.model_name
        state.reference_frame = "world"
        state.pose.orientation.w = 1.0
        if self.motion_case == "yaw":
            state.twist.angular.z = self.desired_yaw_rate

        proxy = rospy.ServiceProxy(service_name, SetModelState)
        deadline = time.monotonic() + min(20.0, self.timeout_sec)
        last_error = "model has not accepted state"
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                response = proxy(state)
                if response.success:
                    with self._lock:
                        self._commanded = True
                    break
                last_error = response.status_message
            except rospy.ServiceException as exc:
                last_error = str(exc)
            time.sleep(0.05)
        else:
            raise ValidationError(
                "could not initialize Gazebo model {!r}: {}".format(
                    self.model_name, last_error
                )
            )

        ready_deadline = time.monotonic() + min(15.0, self.timeout_sec)
        while not self._motion_ready.wait(0.05):
            if rospy.is_shutdown():
                raise ValidationError("ROS shut down before Gazebo motion truth stabilized")
            if time.monotonic() >= ready_deadline:
                raise ValidationError(
                    "timestamped Gazebo truth did not confirm commanded {} motion".format(
                        self.motion_case
                    )
                )
        with self._lock:
            # Retain enough pre-scan truth to bracket the first subscribed bundle,
            # but discard pre-command transients.
            self._odom = self._odom[-10:]
            self._odom_stamps = self._odom_stamps[-10:]

    def _set_failure_locked(self, error):
        if self._finished.is_set():
            return
        self._failure = str(error)
        rospy.logfatal("B-T4 %s validation failed: %s", self.motion_case, self._failure)
        self._finished.set()

    def _odom_callback(self, message):
        try:
            stamp_ns = _stamp_ns(message.header.stamp)
            if stamp_ns <= 0:
                raise ValidationError("Gazebo P3D truth has a zero timestamp")
            if message.header.frame_id != self.expected_world_frame:
                raise ValidationError(
                    "Gazebo P3D frame {!r}, expected {!r}".format(
                        message.header.frame_id, self.expected_world_frame
                    )
                )
            position, quaternion = _pose_from_odom(message)
            linear = message.twist.twist.linear
            angular = message.twist.twist.angular
            twist = (
                linear.x,
                linear.y,
                linear.z,
                angular.x,
                angular.y,
                angular.z,
            )
            if not _finite(position + quaternion + twist):
                raise ValidationError("Gazebo P3D truth contains non-finite state")

            with self._lock:
                if self._odom_stamps and stamp_ns < self._odom_stamps[-1]:
                    raise ValidationError("Gazebo P3D truth timestamps moved backwards")
                if self._odom_stamps and stamp_ns == self._odom_stamps[-1]:
                    self._odom[-1] = message
                else:
                    self._odom_stamps.append(stamp_ns)
                    self._odom.append(message)
                if len(self._odom) > 4000:
                    del self._odom[:1000]
                    del self._odom_stamps[:1000]

                if self._commanded:
                    linear_speed = _norm(twist[:3])
                    if self.motion_case == "stationary":
                        ready = (
                            linear_speed <= self.maximum_linear_speed
                            and _norm(twist[3:])
                            <= self.stationary_maximum_angular_speed
                        )
                    else:
                        ready = (
                            linear_speed <= self.maximum_linear_speed
                            and abs(angular.x) <= self.angular_speed_tolerance
                            and abs(angular.y) <= self.angular_speed_tolerance
                            and abs(angular.z - self.desired_yaw_rate)
                            <= self.angular_speed_tolerance
                        )
                    self._ready_sample_streak = (
                        self._ready_sample_streak + 1 if ready else 0
                    )
                    if self._ready_sample_streak >= 5:
                        self._motion_ready.set()
                self._process_pending_locked()
        except Exception as exc:
            with self._lock:
                self._set_failure_locked(exc)

    def _ray_callback(self, bundle):
        try:
            stamp_ns = _stamp_ns(bundle.header.stamp)
            with self._lock:
                if self._finished.is_set():
                    return
                if stamp_ns in self._pending_bundles:
                    raise ValidationError("duplicate RayBundle timestamp")
                self._pending_bundles[stamp_ns] = bundle
                if len(self._pending_bundles) > 20:
                    raise ValidationError(
                        "more than 20 bundles lack complete diagnostics/P3D brackets"
                    )
                self._process_pending_locked()
        except Exception as exc:
            with self._lock:
                self._set_failure_locked(exc)

    def _diagnostics_callback(self, diagnostics):
        try:
            if len(diagnostics.status) != 1:
                raise ValidationError(
                    "source diagnostics must contain exactly one status"
                )
            stamp_ns = _stamp_ns(diagnostics.header.stamp)
            with self._lock:
                if self._finished.is_set():
                    return
                self._diagnostics[stamp_ns] = diagnostics.status[0]
                if len(self._diagnostics) > 40:
                    for old_stamp in sorted(self._diagnostics)[:-30]:
                        if old_stamp not in self._pending_bundles:
                            self._diagnostics.pop(old_stamp, None)
                self._process_pending_locked()
        except Exception as exc:
            with self._lock:
                self._set_failure_locked(exc)

    def _process_pending_locked(self):
        if self._finished.is_set() or not self._odom_stamps:
            return
        for stamp_ns in sorted(tuple(self._pending_bundles)):
            bundle = self._pending_bundles[stamp_ns]
            if not bundle.rays:
                raise ValidationError("received an empty RayBundle")
            end_ns = stamp_ns + int(bundle.rays[-1].offset_time_ns)
            if stamp_ns not in self._diagnostics or self._odom_stamps[-1] < end_ns:
                continue
            if self._odom_stamps[0] > stamp_ns:
                raise ValidationError(
                    "no strictly earlier/equal Gazebo truth sample for scan {}".format(
                        bundle.scan_id
                    )
                )

            diagnostic = self._diagnostics.pop(stamp_ns)
            summary = self._validate_bundle(bundle, diagnostic)
            self._pending_bundles.pop(stamp_ns)
            self._validated_scan_ids.add(int(bundle.scan_id))
            self._summaries.append(summary)
            rospy.loginfo(
                "B-T4 %s scan=%d PASS candidates=%d late=%d discriminating=%d "
                "per_ray_p95=%.4fm per_ray_max=%.4fm snapshot_p50=%.4fm "
                "odom_gap_max=%.4fs yaw_rate=%.4frad/s",
                self.motion_case,
                bundle.scan_id,
                summary["candidate_count"],
                summary["late_count"],
                summary["discriminating_count"],
                summary["per_ray_p95"],
                summary["per_ray_max"],
                summary["snapshot_p50"],
                summary["maximum_odom_gap_sec"],
                summary["measured_yaw_rate"],
            )

            if len(self._summaries) >= self.required_bundles:
                self._success = True
                rospy.loginfo(
                    "B-T4 per-ray wall gate PASS: case=%s bundles=%d "
                    "counterfactual=%s",
                    self.motion_case,
                    len(self._summaries),
                    (
                        "indistinguishable_as_required_for_zero_motion"
                        if self.motion_case == "stationary"
                        else "snapshot_rejected_by_late_ray_residual"
                    ),
                )
                self._finished.set()
                return

        if self._pending_bundles:
            oldest = min(self._pending_bundles)
            # Once truth has advanced well beyond a scan, a missing exact-stamp
            # diagnostic cannot recover and must not be treated as a skip.
            if (
                self._odom_stamps[-1] > oldest + 1000000000
                and oldest not in self._diagnostics
            ):
                raise ValidationError(
                    "RayBundle at {} has no matching source diagnostic".format(oldest)
                )

    def _validate_diagnostic(self, bundle, status):
        if status.level != DiagnosticStatus.OK:
            raise ValidationError(
                "source diagnostic level={} message={!r}".format(
                    status.level, status.message
                )
            )
        values = {item.key: item.value for item in status.values}
        required = {
            "ray_count",
            "valid_return_count",
            "no_return_count",
            "below_min_range_count",
            "invalid_count",
            "direction_norm_error_max",
            "timestamp_monotonic_ratio",
            "bundle_duration",
            "source_mode",
            "exact_direction_available",
            "ray_time_geometry_mode",
            "motion_model",
            "scene_assumption",
            "linear_speed_mps",
            "angular_speed_radps",
            "max_offset_sec",
        }
        missing = required - set(values)
        if missing:
            raise ValidationError(
                "source diagnostics missing {}".format(sorted(missing))
            )
        exact_strings = {
            "source_mode": "sim_exact",
            "exact_direction_available": "true",
            "ray_time_geometry_mode": "per_ray_pose",
            "motion_model": "constant_twist_world_velocity",
            "scene_assumption": "static_scene_only",
        }
        for key, expected in exact_strings.items():
            if values[key] != expected:
                raise ValidationError(
                    "source diagnostic {}={!r}, expected {!r}".format(
                        key, values[key], expected
                    )
                )
        if int(values["ray_count"]) != self.expected_ray_count:
            raise ValidationError("diagnostic ray_count changed")
        counts = tuple(
            int(values[key])
            for key in (
                "valid_return_count",
                "no_return_count",
                "below_min_range_count",
                "invalid_count",
            )
        )
        if any(value < 0 for value in counts) or sum(counts) != self.expected_ray_count:
            raise ValidationError("diagnostic return counts are not conservative")
        numeric = {
            key: float(values[key])
            for key in (
                "direction_norm_error_max",
                "timestamp_monotonic_ratio",
                "bundle_duration",
                "linear_speed_mps",
                "angular_speed_radps",
                "max_offset_sec",
            )
        }
        if not _finite(tuple(numeric.values())):
            raise ValidationError("source diagnostic contains a non-finite metric")
        if numeric["direction_norm_error_max"] > 1.0e-4:
            raise ValidationError("source direction norm evidence failed")
        if abs(numeric["timestamp_monotonic_ratio"] - 1.0) > 1.0e-9:
            raise ValidationError("source offsets are not monotonic")
        actual_duration = bundle.rays[-1].offset_time_ns * 1.0e-9
        if actual_duration < 0.095 or actual_duration > 0.105:
            raise ValidationError("bundle duration is not a 10 Hz scan interval")
        if abs(numeric["bundle_duration"] - actual_duration) > 1.0e-6:
            raise ValidationError("diagnostic bundle_duration disagrees with RayBundle")
        if abs(numeric["max_offset_sec"] - actual_duration) > 1.0e-6:
            raise ValidationError("diagnostic max_offset_sec disagrees with RayBundle")
        if numeric["linear_speed_mps"] > self.maximum_linear_speed:
            raise ValidationError("plugin observed unexpected observer translation")
        if self.motion_case == "stationary":
            if numeric["angular_speed_radps"] > self.stationary_maximum_angular_speed:
                raise ValidationError("stationary plugin diagnostic reports rotation")
        elif (
            abs(numeric["angular_speed_radps"] - abs(self.desired_yaw_rate))
            > self.angular_speed_tolerance
        ):
            raise ValidationError("plugin did not observe the commanded yaw speed")
        return numeric

    def _interpolated_pose(self, messages, stamps, target_ns, cursor):
        while cursor + 1 < len(stamps) and stamps[cursor + 1] < target_ns:
            cursor += 1
        if cursor + 1 >= len(stamps) or stamps[cursor] > target_ns:
            raise ValidationError("ray time lacks a strict Gazebo truth bracket")
        lower_ns = stamps[cursor]
        upper_ns = stamps[cursor + 1]
        gap_sec = (upper_ns - lower_ns) * 1.0e-9
        if gap_sec <= 0.0 or gap_sec > self.maximum_odom_gap_sec:
            raise ValidationError(
                "Gazebo truth bracket gap {:.6f}s exceeds {:.6f}s".format(
                    gap_sec, self.maximum_odom_gap_sec
                )
            )
        lower_position, lower_quaternion = _pose_from_odom(messages[cursor])
        upper_position, upper_quaternion = _pose_from_odom(messages[cursor + 1])
        ratio = float(target_ns - lower_ns) / float(upper_ns - lower_ns)
        position = tuple(
            first + ratio * (second - first)
            for first, second in zip(lower_position, upper_position)
        )
        quaternion = _slerp(lower_quaternion, upper_quaternion, ratio)
        return position, quaternion, cursor, gap_sec

    def _validate_bundle(self, bundle, diagnostic):
        if bundle.header.frame_id != self.expected_frame:
            raise ValidationError(
                "RayBundle frame {!r}, expected {!r}".format(
                    bundle.header.frame_id, self.expected_frame
                )
            )
        if int(bundle.scan_id) in self._validated_scan_ids:
            raise ValidationError("duplicate scan_id in accepted evidence")
        if len(bundle.rays) != self.expected_ray_count:
            raise ValidationError(
                "RayBundle has {} rays, expected {}".format(
                    len(bundle.rays), self.expected_ray_count
                )
            )
        if bundle.rays[0].offset_time_ns != 0:
            raise ValidationError("first emitted ray does not have zero offset")
        source_diagnostic = self._validate_diagnostic(bundle, diagnostic)

        start_ns = _stamp_ns(bundle.header.stamp)
        end_ns = start_ns + int(bundle.rays[-1].offset_time_ns)
        lower = bisect.bisect_right(self._odom_stamps, start_ns) - 1
        upper = bisect.bisect_left(self._odom_stamps, end_ns) + 1
        lower = max(0, lower)
        upper = min(len(self._odom), upper)
        messages = self._odom[lower:upper]
        stamps = self._odom_stamps[lower:upper]
        if len(messages) < 2 or stamps[0] > start_ns or stamps[-1] < end_ns:
            raise ValidationError("bundle lacks complete P3D truth coverage")

        cursor = bisect.bisect_right(stamps, start_ns) - 1
        start_position, start_quaternion, cursor, start_gap = self._interpolated_pose(
            messages, stamps, start_ns, cursor
        )
        end_position, end_quaternion, _, end_gap = self._interpolated_pose(
            messages, stamps, end_ns, cursor
        )
        duration_sec = (end_ns - start_ns) * 1.0e-9
        position_drift = _norm(
            tuple(end - start for start, end in zip(start_position, end_position))
        )
        if position_drift > self.maximum_position_drift:
            raise ValidationError(
                "observer translated {:.6f}m during pure {} scan".format(
                    position_drift, self.motion_case
                )
            )
        yaw_delta = _wrap_angle(_yaw(end_quaternion) - _yaw(start_quaternion))
        measured_yaw_rate = yaw_delta / duration_sec
        if self.motion_case == "stationary":
            if abs(measured_yaw_rate) > self.stationary_maximum_angular_speed:
                raise ValidationError("stationary P3D truth rotated during the scan")
        elif abs(measured_yaw_rate - self.desired_yaw_rate) > self.angular_speed_tolerance:
            raise ValidationError(
                "P3D yaw rate {:.6f} differs from commanded {:.6f}".format(
                    measured_yaw_rate, self.desired_yaw_rate
                )
            )

        per_ray_residuals = []
        range_residuals = []
        late_snapshot_residuals = []
        late_equivalence_deltas = []
        discriminating_per_ray_residuals = []
        discriminating_snapshot_residuals = []
        candidate_count = 0
        late_count = 0
        previous_offset = -1
        cursor = bisect.bisect_right(stamps, start_ns) - 1
        maximum_gap = max(start_gap, end_gap)
        rotation_scale_residuals = {
            scale: [] for scale in (0.0, 0.5, 1.0, 1.5, 2.0)
        }

        for ray_index, ray in enumerate(bundle.rays):
            offset_ns = int(ray.offset_time_ns)
            if offset_ns < previous_offset:
                raise ValidationError("RayBundle offsets moved backwards")
            previous_offset = offset_ns
            local_direction = (float(ray.dir_x), float(ray.dir_y), float(ray.dir_z))
            if not _finite(local_direction) or abs(_norm(local_direction) - 1.0) > 1.0e-4:
                raise ValidationError(
                    "ray {} has invalid sensor-frame direction".format(ray_index)
                )
            if int(ray.return_status) not in (
                Ray.NO_RETURN,
                Ray.VALID_RETURN,
                Ray.BELOW_MIN_RANGE,
                Ray.INVALID_RANGE,
                Ray.UNKNOWN_STATUS,
            ):
                raise ValidationError("ray has an undefined return status")

            ray_ns = start_ns + offset_ns
            position, quaternion, cursor, gap = self._interpolated_pose(
                messages, stamps, ray_ns, cursor
            )
            maximum_gap = max(maximum_gap, gap)
            direction = _rotate(quaternion, local_direction)
            if direction[0] < self.minimum_forward_x:
                continue
            expected_range = (self.wall_plane_x - position[0]) / direction[0]
            if (
                not math.isfinite(expected_range)
                or expected_range <= float(bundle.min_range)
                or expected_range >= min(float(bundle.max_range), self.maximum_expected_range)
            ):
                continue
            expected_y = position[1] + expected_range * direction[1]
            expected_z = position[2] + expected_range * direction[2]
            if (
                abs(expected_y) > self.wall_interior_half_y
                or abs(expected_z) > self.wall_interior_half_z
            ):
                continue

            candidate_count += 1
            if int(ray.return_status) != Ray.VALID_RETURN:
                raise ValidationError(
                    "interior wall ray {} was not VALID_RETURN (status={})".format(
                        ray_index, ray.return_status
                    )
                )
            measured_range = float(ray.range)
            if not math.isfinite(measured_range):
                raise ValidationError("interior wall range is non-finite")
            per_ray_endpoint_x = position[0] + measured_range * direction[0]
            per_ray_residual = abs(per_ray_endpoint_x - self.wall_plane_x)
            range_residual = abs(measured_range - expected_range)
            per_ray_residuals.append(per_ray_residual)
            range_residuals.append(range_residual)

            # Diagnostic-only counterfactual sweep.  Acceptance remains bound
            # to the independently bracketed P3D pose above; the sweep makes a
            # yaw failure attributable without weakening the gate.
            scan_ratio = float(offset_ns) / float(end_ns - start_ns)
            for scale in rotation_scale_residuals:
                scaled_quaternion = _slerp(
                    start_quaternion, end_quaternion, scale * scan_ratio
                )
                scaled_direction = _rotate(scaled_quaternion, local_direction)
                scaled_endpoint_x = (
                    start_position[0] + measured_range * scaled_direction[0]
                )
                rotation_scale_residuals[scale].append(
                    abs(scaled_endpoint_x - self.wall_plane_x)
                )

            snapshot_direction = _rotate(start_quaternion, local_direction)
            snapshot_endpoint_x = (
                start_position[0] + measured_range * snapshot_direction[0]
            )
            snapshot_residual = abs(snapshot_endpoint_x - self.wall_plane_x)
            if offset_ns * 1.0e-9 >= self.late_offset_sec:
                late_count += 1
                late_snapshot_residuals.append(snapshot_residual)
                late_equivalence_deltas.append(
                    abs(snapshot_endpoint_x - per_ray_endpoint_x)
                )
                if snapshot_residual >= self.minimum_counterfactual_residual:
                    discriminating_per_ray_residuals.append(per_ray_residual)
                    discriminating_snapshot_residuals.append(snapshot_residual)

        if candidate_count < self.minimum_wall_candidates:
            raise ValidationError(
                "only {} unambiguous wall rays; need {}".format(
                    candidate_count, self.minimum_wall_candidates
                )
            )
        if late_count < self.minimum_late_candidates:
            raise ValidationError(
                "only {} late wall rays; need {}".format(
                    late_count, self.minimum_late_candidates
                )
            )
        per_ray_p95 = _percentile(per_ray_residuals, 0.95)
        per_ray_max = max(per_ray_residuals)
        range_p95 = _percentile(range_residuals, 0.95)
        if (
            per_ray_p95 > self.endpoint_p95_tolerance
            or range_p95 > self.endpoint_p95_tolerance
            or per_ray_max > self.endpoint_max_tolerance
        ):
            rospy.logerr(
                "B-T4 yaw diagnostic: measured_yaw_rate=%.6f plugin_angular=%.6f "
                "rotation_scale_p95=%s",
                measured_yaw_rate,
                source_diagnostic["angular_speed_radps"],
                ",".join(
                    "{:.1f}:{:.6f}".format(
                        scale, _percentile(residuals, 0.95)
                    )
                    for scale, residuals in sorted(
                        rotation_scale_residuals.items()
                    )
                ),
            )
            raise ValidationError(
                "timestamped wall residuals are too large: endpoint p95={:.6f} "
                "max={:.6f}, range p95={:.6f}".format(
                    per_ray_p95, per_ray_max, range_p95
                )
            )

        if self.motion_case == "stationary":
            snapshot_p50 = _percentile(late_snapshot_residuals, 0.50)
            snapshot_p95 = _percentile(late_snapshot_residuals, 0.95)
            equivalence_max = max(late_equivalence_deltas)
            if (
                snapshot_p95 > self.endpoint_p95_tolerance
                or equivalence_max > self.endpoint_p95_tolerance
            ):
                raise ValidationError(
                    "zero-motion snapshot control is not equivalent: "
                    "snapshot p95={:.6f}, endpoint delta max={:.6f}".format(
                        snapshot_p95, equivalence_max
                    )
                )
            discriminating_count = 0
        else:
            discriminating_count = len(discriminating_snapshot_residuals)
            if discriminating_count < self.minimum_discriminating_rays:
                raise ValidationError(
                    "only {} rays distinguish per-ray geometry from snapshot; need {}".format(
                        discriminating_count, self.minimum_discriminating_rays
                    )
                )
            discriminating_per_p95 = _percentile(
                discriminating_per_ray_residuals, 0.95
            )
            snapshot_p50 = _percentile(discriminating_snapshot_residuals, 0.50)
            improvement = snapshot_p50 - discriminating_per_p95
            ratio = snapshot_p50 / max(discriminating_per_p95, 0.001)
            if discriminating_per_p95 > self.endpoint_p95_tolerance:
                raise ValidationError("late discriminating rays miss the wall")
            if improvement < self.minimum_counterfactual_improvement:
                raise ValidationError(
                    "per-ray evidence improves snapshot by only {:.6f}m".format(
                        improvement
                    )
                )
            if ratio < self.minimum_counterfactual_ratio:
                raise ValidationError(
                    "per-ray/snapshot residual ratio {:.3f} is insufficient".format(
                        ratio
                    )
                )

        return {
            "candidate_count": candidate_count,
            "late_count": late_count,
            "discriminating_count": discriminating_count,
            "per_ray_p95": per_ray_p95,
            "per_ray_max": per_ray_max,
            "snapshot_p50": snapshot_p50,
            "maximum_odom_gap_sec": maximum_gap,
            "measured_yaw_rate": measured_yaw_rate,
        }

    def run(self):
        deadline = time.monotonic() + self.timeout_sec
        while not self._finished.wait(0.1):
            if rospy.is_shutdown():
                with self._lock:
                    self._failure = "ROS shut down before B-T4 evidence completed"
                break
            if time.monotonic() >= deadline:
                with self._lock:
                    self._failure = (
                        "timeout: validated={} pending={} diagnostics={} odom_samples={}".format(
                            len(self._summaries),
                            len(self._pending_bundles),
                            len(self._diagnostics),
                            len(self._odom),
                        )
                    )
                break
        if self._success:
            return 0
        rospy.logfatal(
            "B-T4 %s gate FAIL: %s",
            self.motion_case,
            self._failure or "unknown/inadequate evidence",
        )
        return 1


class PerRayWallValidatorRostest(unittest.TestCase):
    def test_b_t4_per_ray_wall_contract(self):
        try:
            validator = PerRayWallValidator()
            result = validator.run()
            self.assertEqual(
                result,
                0,
                validator._failure or "B-T4 validator returned non-zero",
            )
        except Exception as exc:
            self.fail("B-T4 validator setup failed: {}".format(exc))


def main():
    rospy.init_node("b_t4_per_ray_wall_validator", anonymous=False)
    if _bool_param("~rostest", False):
        import rostest

        rostest.rosrun(
            "livox_laser_simulation",
            "b_t4_per_ray_wall_validator",
            PerRayWallValidatorRostest,
        )
        return 0
    try:
        return PerRayWallValidator().run()
    except Exception as exc:
        rospy.logfatal("B-T4 validator setup failed: %s", exc)
        return 2


if __name__ == "__main__":
    sys.exit(main())
