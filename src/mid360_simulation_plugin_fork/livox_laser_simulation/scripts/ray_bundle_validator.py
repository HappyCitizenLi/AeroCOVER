#!/usr/bin/env python3
"""Integration validator for the Mid-360 RayBundle Gazebo test worlds.

The validator deliberately uses only the public RayBundle and legacy
PointCloud2 topics.  It does not subscribe to Gazebo state or any other truth
source.  ``mode:=empty`` checks the collision-free world, while ``mode:=wall``
checks the finite single-wall world and the index-preserving relationship
between the two public messages.
"""

import csv
import math
import sys
import threading
import time
import unittest
from array import array

import message_filters
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2

from mid360_ray_msgs.msg import Ray, RayBundle, ScanIdentity


class ValidationError(RuntimeError):
    """A deterministic validation failure."""


def _bool_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class RayBundleValidator:
    STATUS_NAMES = {
        Ray.NO_RETURN: "NO_RETURN",
        Ray.VALID_RETURN: "VALID_RETURN",
        Ray.BELOW_MIN_RANGE: "BELOW_MIN_RANGE",
        Ray.INVALID_RANGE: "INVALID_RANGE",
        Ray.UNKNOWN_STATUS: "UNKNOWN_STATUS",
    }

    def __init__(self):
        self.mode = str(rospy.get_param("~mode", "empty")).strip().lower()
        if self.mode not in ("empty", "wall", "bundle"):
            raise ValidationError("~mode must be one of: empty, wall, bundle")

        self.ray_topic = rospy.get_param("~ray_topic", "/uav1/mid360/rays_raw")
        self.point_topic = rospy.get_param("~point_topic", "/uav1/mid360/points_raw")
        self.identity_topic = rospy.get_param(
            "~identity_topic", "/uav1/mid360/scan_identity"
        )
        self.diagnostics_topic = rospy.get_param(
            "~diagnostics_topic", "/uav1/mid360/ray_source_diagnostics"
        )
        self.expected_frame = rospy.get_param("~expected_frame", "uav1/mid360")

        self.samples = int(rospy.get_param("~samples", 20000))
        self.downsample = int(rospy.get_param("~downsample", 1))
        if self.samples <= 0 or self.downsample <= 0:
            raise ValidationError("~samples and ~downsample must both be positive")
        default_count = (self.samples + self.downsample - 1) // self.downsample
        self.expected_ray_count = int(
            rospy.get_param("~expected_ray_count", default_count)
        )
        self.pattern_step = int(rospy.get_param("~pattern_step", self.downsample))
        if self.expected_ray_count <= 0 or self.pattern_step <= 0:
            raise ValidationError("expected ray count and pattern step must be positive")
        self.ray_point_rate = float(rospy.get_param("~ray_point_rate", 200000.0))
        if not math.isfinite(self.ray_point_rate) or self.ray_point_rate <= 0.0:
            raise ValidationError("~ray_point_rate must be finite and positive")
        self.expect_uniform_time = _bool_param("~expect_uniform_time", True)
        self.expected_last_offset_ns = int(
            round(
                (self.expected_ray_count - 1)
                * self.pattern_step
                / self.ray_point_rate
                * 1.0e9
            )
        )

        self.required_bundles = int(rospy.get_param("~required_bundles", 3))
        self.timeout_sec = float(rospy.get_param("~timeout_sec", 20.0))
        self.sync_slop_sec = float(rospy.get_param("~sync_slop_sec", 0.03))
        self.require_pattern_wrap = _bool_param("~require_pattern_wrap", True)
        self.require_csv_index_zero = _bool_param("~require_csv_index_zero", True)
        self.require_diagnostics = _bool_param("~require_diagnostics", True)

        self.direction_norm_tolerance = float(
            rospy.get_param("~direction_norm_tolerance", 1.0e-4)
        )
        self.csv_direction_tolerance = float(
            rospy.get_param("~csv_direction_tolerance", 1.0e-4)
        )
        self.zero_tolerance = float(rospy.get_param("~zero_tolerance", 1.0e-6))
        self.endpoint_abs_tolerance = float(
            rospy.get_param("~endpoint_abs_tolerance", 2.0e-3)
        )
        self.endpoint_rel_tolerance = float(
            rospy.get_param("~endpoint_rel_tolerance", 2.0e-4)
        )

        self.csv_file = rospy.get_param("~csv_file", "")
        configured_pattern_length = int(rospy.get_param("~pattern_length", 0))
        self.csv_directions = None
        csv_pattern_length = 0
        if self.csv_file:
            csv_pattern_length, self.csv_directions = self._read_csv_contract(
                self.csv_file
            )
        if configured_pattern_length > 0 and csv_pattern_length > 0:
            if configured_pattern_length != csv_pattern_length:
                raise ValidationError(
                    "configured pattern_length={} disagrees with CSV row count={}".format(
                        configured_pattern_length, csv_pattern_length
                    )
                )
        self.pattern_length = configured_pattern_length or csv_pattern_length
        if self.pattern_length <= 0:
            raise ValidationError("provide a non-empty ~csv_file or positive ~pattern_length")

        self._lock = threading.Lock()
        self._finished = threading.Event()
        self._failure = None
        self._success = False

        self.bundle_count = 0
        self.point_count = 0
        self.endpoint_count = 0
        self.max_norm_error = 0.0
        self.max_endpoint_error = 0.0
        self.max_stamp_delta = 0.0
        self.status_counts = {status: 0 for status in self.STATUS_NAMES}
        self.previous_pattern_index = None
        self.previous_scan_id = None
        self.observed_pattern_wrap = False
        self.observed_csv_index_zero = False
        self.csv_direction_checks = 0
        self.diagnostics_count = 0

        self._diagnostic_sub = rospy.Subscriber(
            self.diagnostics_topic,
            DiagnosticArray,
            self._diagnostics_callback,
            queue_size=10,
        )

        # Canonical producers stamp the compatibility cloud and RayBundle with
        # one explicit scan identity.
        if self.mode in ("empty", "wall"):
            self._ray_sub = message_filters.Subscriber(self.ray_topic, RayBundle)
            self._point_sub = message_filters.Subscriber(self.point_topic, PointCloud2)
            self._identity_sub = message_filters.Subscriber(
                self.identity_topic, ScanIdentity
            )
            self._sync = message_filters.TimeSynchronizer(
                [self._ray_sub, self._point_sub, self._identity_sub],
                queue_size=20,
            )
            self._sync.registerCallback(self._synchronized_callback)
        else:
            self._ray_sub = rospy.Subscriber(
                self.ray_topic, RayBundle, self._bundle_callback, queue_size=10
            )

        rospy.loginfo(
            "RayBundle validator mode=%s rays=%s cloud=%s expected_count=%d "
            "pattern_length=%d pattern_step=%d",
            self.mode,
            self.ray_topic,
            self.point_topic,
            self.expected_ray_count,
            self.pattern_length,
            self.pattern_step,
        )

    def _diagnostics_callback(self, diagnostics):
        try:
            with self._lock:
                if self._finished.is_set():
                    return
                if not diagnostics.status:
                    raise ValidationError("ray diagnostics contains no status")
                status = diagnostics.status[0]
                if status.level != DiagnosticStatus.OK:
                    raise ValidationError(
                        "ray diagnostics level={} message={!r}".format(
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
                    "exact_direction_available",
                    "source_mode",
                    "ray_time_geometry_mode",
                }
                missing = required - set(values)
                if missing:
                    raise ValidationError(
                        "ray diagnostics missing keys {}".format(sorted(missing))
                    )
                if int(values["ray_count"]) != self.expected_ray_count:
                    raise ValidationError("diagnostic ray_count disagrees with test contract")
                diagnostic_counts = (
                    int(values["valid_return_count"]),
                    int(values["no_return_count"]),
                    int(values["below_min_range_count"]),
                    int(values["invalid_count"]),
                )
                if any(count < 0 for count in diagnostic_counts):
                    raise ValidationError("ray diagnostics contains a negative return count")
                if sum(diagnostic_counts) != self.expected_ray_count:
                    raise ValidationError(
                        "diagnostic return counts do not sum to ray_count"
                    )
                diagnostic_norm_error = float(values["direction_norm_error_max"])
                if (
                    not math.isfinite(diagnostic_norm_error)
                    or diagnostic_norm_error > self.direction_norm_tolerance
                ):
                    raise ValidationError(
                        "diagnostic direction_norm_error_max violates tolerance"
                    )
                monotonic_ratio = float(values["timestamp_monotonic_ratio"])
                if not math.isfinite(monotonic_ratio) or abs(monotonic_ratio - 1.0) > 1.0e-9:
                    raise ValidationError(
                        "diagnostic timestamp_monotonic_ratio is not 1"
                    )
                bundle_duration = float(values["bundle_duration"])
                expected_duration = self.expected_last_offset_ns * 1.0e-9
                if self.expect_uniform_time and (
                    not math.isfinite(bundle_duration)
                    or abs(bundle_duration - expected_duration) > 1.0e-6
                ):
                    raise ValidationError(
                        "diagnostic bundle_duration={} differs from uniform expectation={}".format(
                            bundle_duration, expected_duration
                        )
                    )
                if values["exact_direction_available"].lower() != "true":
                    raise ValidationError("sim_exact directions are not marked available")
                if values["source_mode"] != "sim_exact":
                    raise ValidationError("unexpected source_mode={!r}".format(values["source_mode"]))
                if values["ray_time_geometry_mode"] != "snapshot":
                    raise ValidationError(
                        "unexpected ray_time_geometry_mode={!r}".format(
                            values["ray_time_geometry_mode"]
                        )
                    )
                if self.expect_uniform_time and status.message != "uniform_time_fallback":
                    raise ValidationError(
                        "expected uniform_time_fallback diagnostics, got {!r}".format(
                            status.message
                        )
                    )
                self.diagnostics_count += 1
                self._finish_if_complete()
        except Exception as exc:
            self._set_failure(exc)

    @staticmethod
    def _read_csv_contract(csv_file):
        first = None
        row_count = 0
        directions = (array("f"), array("f"), array("f"))
        try:
            with open(csv_file, "r", newline="") as stream:
                reader = csv.reader(stream)
                next(reader, None)  # Time/s, Azimuth/deg, Zenith/deg
                for row in reader:
                    if len(row) < 3 or not any(field.strip() for field in row):
                        continue
                    azimuth = math.radians(float(row[1]))
                    polar_from_positive_z = math.radians(float(row[2]))
                    direction = (
                        math.sin(polar_from_positive_z) * math.cos(azimuth),
                        math.sin(polar_from_positive_z) * math.sin(azimuth),
                        math.cos(polar_from_positive_z),
                    )
                    if not all(math.isfinite(component) for component in direction):
                        raise ValidationError(
                            "scan CSV row {} has a non-finite direction".format(
                                row_count + 1
                            )
                        )
                    if first is None:
                        first = direction
                    for components, component in zip(directions, direction):
                        components.append(component)
                    row_count += 1
        except (OSError, ValueError) as exc:
            raise ValidationError("cannot parse scan CSV {!r}: {}".format(csv_file, exc))

        if first is None or row_count == 0:
            raise ValidationError("scan CSV contains no data rows: {}".format(csv_file))

        if first[2] <= 0.0:
            raise ValidationError(
                "CSV first-ray regression: expected positive sensor-frame Z, got {:.9f}".format(
                    first[2]
                )
            )
        rospy.loginfo(
            "CSV contract: rows=%d first_direction=(%.9f, %.9f, %.9f); +Z locked",
            row_count,
            first[0],
            first[1],
            first[2],
        )
        return row_count, directions

    def _set_failure(self, exc):
        with self._lock:
            if self._finished.is_set():
                return
            self._failure = str(exc)
            rospy.logfatal("RayBundle validation failed: %s", self._failure)
            self._finished.set()

    def _bundle_callback(self, bundle):
        try:
            with self._lock:
                if self._finished.is_set():
                    return
                self._validate_bundle(bundle)
                self._finish_if_complete()
        except Exception as exc:  # callback exceptions otherwise only reach rosout
            self._set_failure(exc)

    def _synchronized_callback(self, bundle, cloud, identity):
        try:
            with self._lock:
                if self._finished.is_set():
                    return
                self._validate_bundle(bundle)
                self._validate_cloud(bundle, cloud)
                self._validate_identity(bundle, cloud, identity)
                self._finish_if_complete()
        except Exception as exc:  # callback exceptions otherwise only reach rosout
            self._set_failure(exc)

    def _validate_bundle(self, bundle):
        if self.previous_scan_id is not None:
            expected_scan_id = (self.previous_scan_id + 1) & 0xFFFFFFFF
            if int(bundle.scan_id) != expected_scan_id:
                raise ValidationError(
                    "scan_id discontinuity: {} -> {}, expected {}".format(
                        self.previous_scan_id, bundle.scan_id, expected_scan_id
                    )
                )
        if bundle.header.frame_id != self.expected_frame:
            raise ValidationError(
                "bundle frame {!r}, expected {!r}".format(
                    bundle.header.frame_id, self.expected_frame
                )
            )

        rays = bundle.rays
        if len(rays) != self.expected_ray_count:
            raise ValidationError(
                "bundle {} has {} rays, expected samples/downsample={}".format(
                    bundle.scan_id, len(rays), self.expected_ray_count
                )
            )
        if not rays:
            raise ValidationError("RayBundle is empty")
        if rays[0].offset_time_ns != 0:
            raise ValidationError(
                "bundle {} first offset_time_ns={}, expected 0".format(
                    bundle.scan_id, rays[0].offset_time_ns
                )
            )
        if self.expect_uniform_time and rays[-1].offset_time_ns != self.expected_last_offset_ns:
            raise ValidationError(
                "bundle {} last offset_time_ns={}, expected uniform {}".format(
                    bundle.scan_id,
                    rays[-1].offset_time_ns,
                    self.expected_last_offset_ns,
                )
            )
        if bundle.pattern_start_index != rays[0].pattern_index:
            raise ValidationError(
                "pattern_start_index={} but first ray index={}".format(
                    bundle.pattern_start_index, rays[0].pattern_index
                )
            )

        previous_offset = rays[0].offset_time_ns
        previous_in_bundle = None
        bundle_counts = {status: 0 for status in self.STATUS_NAMES}

        for ray_index, ray in enumerate(rays):
            direction = (ray.dir_x, ray.dir_y, ray.dir_z)
            if not all(math.isfinite(component) for component in direction):
                raise ValidationError(
                    "bundle {} ray {} has non-finite direction {}".format(
                        bundle.scan_id, ray_index, direction
                    )
                )
            norm = math.sqrt(sum(component * component for component in direction))
            if norm <= self.zero_tolerance:
                raise ValidationError(
                    "bundle {} ray {} has zero direction".format(bundle.scan_id, ray_index)
                )
            norm_error = abs(norm - 1.0)
            self.max_norm_error = max(self.max_norm_error, norm_error)
            if norm_error > self.direction_norm_tolerance:
                raise ValidationError(
                    "bundle {} ray {} direction norm {:.9f}, error {:.3g} > {:.3g}".format(
                        bundle.scan_id,
                        ray_index,
                        norm,
                        norm_error,
                        self.direction_norm_tolerance,
                    )
                )

            if ray.offset_time_ns < previous_offset:
                raise ValidationError(
                    "bundle {} offsets decrease at ray {}: {} -> {}".format(
                        bundle.scan_id,
                        ray_index,
                        previous_offset,
                        ray.offset_time_ns,
                    )
                )
            previous_offset = ray.offset_time_ns

            pattern_index = int(ray.pattern_index)
            if pattern_index < 0 or pattern_index >= self.pattern_length:
                raise ValidationError(
                    "bundle {} ray {} pattern_index={} outside [0,{})".format(
                        bundle.scan_id, ray_index, pattern_index, self.pattern_length
                    )
                )
            if previous_in_bundle is not None:
                expected = (previous_in_bundle + self.pattern_step) % self.pattern_length
                if pattern_index != expected:
                    raise ValidationError(
                        "bundle {} pattern discontinuity at ray {}: {} -> {}, expected {}".format(
                            bundle.scan_id,
                            ray_index,
                            previous_in_bundle,
                            pattern_index,
                            expected,
                        )
                    )
                if pattern_index < previous_in_bundle:
                    self.observed_pattern_wrap = True
            previous_in_bundle = pattern_index

            status = int(ray.return_status)
            if status not in self.STATUS_NAMES:
                raise ValidationError(
                    "bundle {} ray {} has undefined return_status={}".format(
                        bundle.scan_id, ray_index, status
                    )
                )
            bundle_counts[status] += 1
            self.status_counts[status] += 1

            if status == Ray.VALID_RETURN:
                if not math.isfinite(ray.range):
                    raise ValidationError("VALID_RETURN range is non-finite")
                if not (bundle.min_range < ray.range < bundle.max_range):
                    raise ValidationError(
                        "VALID_RETURN range {} outside ({},{})".format(
                            ray.range, bundle.min_range, bundle.max_range
                        )
                    )
            elif abs(ray.range) > self.zero_tolerance:
                raise ValidationError(
                    "non-valid ray {} carries nonzero range {}".format(ray_index, ray.range)
                )

            self._validate_csv_direction(pattern_index, direction)

        if self.previous_pattern_index is not None:
            expected = (
                self.previous_pattern_index + self.pattern_step
            ) % self.pattern_length
            if rays[0].pattern_index != expected:
                raise ValidationError(
                    "cross-bundle pattern discontinuity: {} -> {}, expected {}".format(
                        self.previous_pattern_index, rays[0].pattern_index, expected
                    )
                )
            if rays[0].pattern_index < self.previous_pattern_index:
                self.observed_pattern_wrap = True
        self.previous_pattern_index = int(rays[-1].pattern_index)
        self.previous_scan_id = int(bundle.scan_id)

        if self.mode == "empty":
            if bundle_counts[Ray.NO_RETURN] != len(rays):
                raise ValidationError(
                    "empty world has hits/invalid rays: {}".format(
                        self._format_counts(bundle_counts)
                    )
                )
        elif self.mode == "wall":
            disallowed = (
                bundle_counts[Ray.BELOW_MIN_RANGE]
                + bundle_counts[Ray.INVALID_RANGE]
                + bundle_counts[Ray.UNKNOWN_STATUS]
            )
            if disallowed:
                raise ValidationError(
                    "single-wall bundle contains invalid states: {}".format(
                        self._format_counts(bundle_counts)
                    )
                )
            if bundle_counts[Ray.VALID_RETURN] <= 0 or bundle_counts[Ray.NO_RETURN] <= 0:
                raise ValidationError(
                    "single finite wall must produce VALID_RETURN and NO_RETURN in the "
                    "same bundle: {}".format(self._format_counts(bundle_counts))
                )

        self.bundle_count += 1
        if self.bundle_count == 1 or self.bundle_count % 10 == 0:
            rospy.loginfo(
                "validated bundle=%d scan_id=%d statuses={%s} wrap=%s",
                self.bundle_count,
                bundle.scan_id,
                self._format_counts(bundle_counts),
                self.observed_pattern_wrap,
            )

    def _validate_csv_direction(self, pattern_index, direction):
        if pattern_index == 0:
            self.observed_csv_index_zero = True
        if pattern_index == 0 and direction[2] <= 0.0:
            raise ValidationError(
                "pattern_index=0 direction Z must be positive, got {:.9f}".format(
                    direction[2]
                )
            )
        if self.csv_directions is None:
            return
        expected = tuple(components[pattern_index] for components in self.csv_directions)
        component_error = max(
            abs(actual - expected)
            for actual, expected in zip(direction, expected)
        )
        if component_error > self.csv_direction_tolerance:
            raise ValidationError(
                "pattern_index={} direction {} disagrees with CSV {} (max error {:.3g})".format(
                    pattern_index, direction, expected, component_error
                )
            )
        self.csv_direction_checks += 1

    def _validate_cloud(self, bundle, cloud):
        if cloud.header.frame_id != self.expected_frame:
            raise ValidationError(
                "point cloud frame {!r}, expected {!r}".format(
                    cloud.header.frame_id, self.expected_frame
                )
            )
        stamp_delta = abs((bundle.header.stamp - cloud.header.stamp).to_sec())
        self.max_stamp_delta = max(self.max_stamp_delta, stamp_delta)
        if stamp_delta > 1.0e-9:
            raise ValidationError(
                "bundle/cloud source stamps differ by {:.9f}s".format(stamp_delta)
            )
        field_names = {field.name for field in cloud.fields}
        required_fields = {"x", "y", "z", "intensity", "tag", "line", "timestamp"}
        missing = required_fields - field_names
        if missing:
            raise ValidationError(
                "legacy PointCloud2 missing fields: {}".format(sorted(missing))
            )
        points = list(
            point_cloud2.read_points(
                cloud, field_names=("x", "y", "z"), skip_nans=False
            )
        )

        if len(points) != len(bundle.rays):
            raise ValidationError(
                "same-tick cloud has {} points but bundle has {} rays; indices cannot align".format(
                    len(points), len(bundle.rays)
                )
            )

        for point_index, (point, ray) in enumerate(zip(points, bundle.rays)):
            if not all(math.isfinite(component) for component in point):
                raise ValidationError(
                    "cloud point {} is non-finite: {}".format(point_index, point)
                )
            point_norm = math.sqrt(sum(component * component for component in point))

            if self.mode == "empty":
                if point_norm > self.zero_tolerance:
                    raise ValidationError(
                        "empty-world legacy point {} is nonzero: {}".format(
                            point_index, point
                        )
                    )
                continue

            if ray.return_status == Ray.VALID_RETURN:
                expected = (
                    ray.range * ray.dir_x,
                    ray.range * ray.dir_y,
                    ray.range * ray.dir_z,
                )
                error = math.sqrt(
                    sum((actual - target) ** 2 for actual, target in zip(point, expected))
                )
                self.max_endpoint_error = max(self.max_endpoint_error, error)
                allowed = self.endpoint_abs_tolerance + self.endpoint_rel_tolerance * max(
                    ray.range, point_norm
                )
                if error > allowed:
                    raise ValidationError(
                        "wall point/ray mismatch at index {}: p={} r*d={} error={} > {}".format(
                            point_index, point, expected, error, allowed
                        )
                    )
                self.endpoint_count += 1
            elif ray.return_status == Ray.NO_RETURN and point_norm > self.zero_tolerance:
                raise ValidationError(
                    "NO_RETURN legacy point {} is nonzero: {}".format(point_index, point)
                )

        self.point_count += len(points)

    @staticmethod
    def _validate_identity(bundle, cloud, identity):
        if (
            int(identity.scan_id) != int(bundle.scan_id)
            or int(identity.pattern_start_index) != int(bundle.pattern_start_index)
            or int(identity.point_count) != int(cloud.width * cloud.height)
            or int(identity.ray_count) != len(bundle.rays)
            or identity.point_source_stamp != cloud.header.stamp
            or identity.ray_source_stamp != bundle.header.stamp
        ):
            raise ValidationError("ScanIdentity does not match source payloads")

    def _finish_if_complete(self):
        if self.bundle_count < self.required_bundles:
            return
        if self.require_pattern_wrap and not self.observed_pattern_wrap:
            return
        if self.require_csv_index_zero and not self.observed_csv_index_zero:
            return
        if self.require_diagnostics and self.diagnostics_count <= 0:
            return
        if self.mode == "wall":
            if self.status_counts[Ray.VALID_RETURN] <= 0:
                return
            if self.status_counts[Ray.NO_RETURN] <= 0:
                return
            if self.endpoint_count <= 0:
                return

        self._success = True
        rospy.loginfo(
            "RayBundle validation PASS: mode=%s bundles=%d points=%d statuses={%s} "
            "csv_direction_checks=%d max_norm_error=%.3g max_endpoint_error=%.3g "
            "max_stamp_delta=%.6fs",
            self.mode,
            self.bundle_count,
            self.point_count,
            self._format_counts(self.status_counts),
            self.csv_direction_checks,
            self.max_norm_error,
            self.max_endpoint_error,
            self.max_stamp_delta,
        )
        self._finished.set()

    def _format_counts(self, counts):
        return ", ".join(
            "{}={}".format(self.STATUS_NAMES[status], counts.get(status, 0))
            for status in (
                Ray.VALID_RETURN,
                Ray.NO_RETURN,
                Ray.BELOW_MIN_RANGE,
                Ray.INVALID_RANGE,
                Ray.UNKNOWN_STATUS,
            )
        )

    def run(self):
        deadline = time.monotonic() + self.timeout_sec
        while not self._finished.wait(0.1):
            if rospy.is_shutdown():
                self._failure = "ROS shut down before validation completed"
                break
            if time.monotonic() >= deadline:
                self._failure = (
                    "timeout after {:.1f}s: bundles={} wrap={} csv_index_zero={} "
                    "statuses={{{}}}".format(
                        self.timeout_sec,
                        self.bundle_count,
                        self.observed_pattern_wrap,
                        self.observed_csv_index_zero,
                        self._format_counts(self.status_counts),
                    )
                )
                break

        if self._success:
            return 0
        rospy.logfatal("RayBundle validation FAIL: %s", self._failure or "unknown failure")
        return 1


class RayBundleValidatorRostest(unittest.TestCase):
    """xUnit adapter used only when the node is launched from a .test file."""

    def test_ray_bundle_contract(self):
        validator = RayBundleValidator()
        result = validator.run()
        self.assertEqual(
            result,
            0,
            validator._failure or "RayBundle validator returned a non-zero status",
        )


def main():
    rospy.init_node("ray_bundle_validator", anonymous=False)
    if _bool_param("~rostest", False):
        import rostest

        rostest.rosrun(
            "livox_laser_simulation",
            "ray_bundle_validator",
            RayBundleValidatorRostest,
        )
        return 0
    try:
        validator = RayBundleValidator()
        return validator.run()
    except Exception as exc:
        rospy.logfatal("RayBundle validator setup failed: %s", exc)
        return 2


if __name__ == "__main__":
    sys.exit(main())
