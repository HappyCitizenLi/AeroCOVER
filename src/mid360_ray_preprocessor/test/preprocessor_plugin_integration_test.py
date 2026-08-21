#!/usr/bin/env python3
"""End-to-end contract test against the controlled 20k-ray Gazebo plugin."""

import math
import threading
import time
import unittest

import message_filters
import rospy
import rostest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from mid360_ray_msgs.msg import CheckedRayBundle, Ray
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField


class ContractError(RuntimeError):
    pass


def require(condition, message):
    if not condition:
        raise ContractError(message)


class PluginPreprocessorIntegrationTest(unittest.TestCase):
    SOURCE_FIELDS = ("x", "y", "z", "intensity", "tag", "line", "timestamp")

    def setUp(self):
        self.mode = str(rospy.get_param("~mode", "empty"))
        require(self.mode in ("empty", "wall"), "mode must be empty or wall")
        self.required_pairs = int(rospy.get_param("~required_pairs", 2))
        self.timeout_sec = float(rospy.get_param("~timeout_sec", 28.0))
        self.expected_ray_count = int(rospy.get_param("~expected_ray_count", 20000))

        self.lock = threading.Lock()
        self.finished = threading.Event()
        self.failure = None
        self.pair_count = 0
        self.diagnostic_ok_count = 0
        self.checked_scan_ids = set()
        self.total_valid_points = 0

        self.raw_cloud_sub = message_filters.Subscriber(
            "/uav1/mid360/points_raw", PointCloud2
        )
        self.valid_cloud_sub = message_filters.Subscriber(
            "/uav1/mid360/points_valid", PointCloud2
        )
        self.world_cloud_sub = message_filters.Subscriber(
            "/uav1/mid360/points_world", PointCloud2
        )
        self.checked_rays_sub = message_filters.Subscriber(
            "/uav1/mid360/rays_checked", CheckedRayBundle
        )
        self.sync = message_filters.TimeSynchronizer(
            [
                self.raw_cloud_sub,
                self.valid_cloud_sub,
                self.world_cloud_sub,
                self.checked_rays_sub,
            ],
            queue_size=10,
        )
        self.sync.registerCallback(self._synchronized_callback)
        self.diagnostics_sub = rospy.Subscriber(
            "/uav1/mid360/ray_diagnostics",
            DiagnosticArray,
            self._diagnostics_callback,
            queue_size=10,
        )

    def _set_failure(self, exception):
        with self.lock:
            if self.failure is None:
                self.failure = str(exception)
                rospy.logerr("plugin/preprocessor validation failed: %s", self.failure)
            self.finished.set()

    def _update_completion_locked(self):
        if self.pair_count >= self.required_pairs and self.diagnostic_ok_count > 0:
            self.finished.set()

    @staticmethod
    def _assert_source_layout_preserved(raw_cloud, valid_cloud):
        require(
            valid_cloud.header.frame_id == raw_cloud.header.frame_id,
            "points_valid must preserve the raw cloud coordinate frame",
        )
        require(
            len(valid_cloud.fields) == len(raw_cloud.fields) + 1,
            "points_valid must append exactly one field",
        )
        for raw_field, valid_field in zip(raw_cloud.fields, valid_cloud.fields):
            require(
                raw_field.name == valid_field.name
                and raw_field.offset == valid_field.offset
                and raw_field.datatype == valid_field.datatype
                and raw_field.count == valid_field.count,
                "raw PointCloud2 field layout changed during filtering",
            )
        index_field = valid_cloud.fields[-1]
        require(index_field.name == "original_index", "missing original_index field")
        require(
            index_field.offset == raw_cloud.point_step
            and index_field.datatype == PointField.UINT32
            and index_field.count == 1,
            "original_index field has the wrong layout",
        )
        require(
            valid_cloud.point_step == raw_cloud.point_step + 4,
            "points_valid point_step does not include one uint32 index",
        )

    def _validate_checked_bundle(self, checked):
        require(checked.header.frame_id == "world", "checked bundle is not in world")
        require(
            checked.source_header.frame_id == "uav1/mid360",
            "checked bundle lost its source frame",
        )
        require(checked.source_mode == "sim_exact", "wrong checked source_mode")
        require(
            checked.ray_time_geometry_mode == "snapshot",
            "Gazebo plugin must be checked in snapshot mode",
        )
        require(
            len(checked.rays) == self.expected_ray_count,
            "checked bundle ray count is not samples/downsample=20000",
        )

        status_counts = {
            Ray.NO_RETURN: 0,
            Ray.VALID_RETURN: 0,
            Ray.BELOW_MIN_RANGE: 0,
            Ray.INVALID_RANGE: 0,
            Ray.UNKNOWN_STATUS: 0,
        }
        previous_offset = 0
        for index, checked_ray in enumerate(checked.rays):
            require(checked_ray.original_index == index, "checked ray index changed")
            require(checked_ray.direction_valid, "checked source direction is invalid")
            require(checked_ray.transform_valid, "checked ray has no world transform")
            require(
                abs(checked_ray.origin.x) < 1.0e-9
                and abs(checked_ray.origin.y) < 1.0e-9
                and abs(checked_ray.origin.z) < 1.0e-9,
                "identity TF did not produce a zero world origin",
            )

            source = checked_ray.source
            local_norm = math.sqrt(
                source.dir_x * source.dir_x
                + source.dir_y * source.dir_y
                + source.dir_z * source.dir_z
            )
            world_norm = math.sqrt(
                checked_ray.direction.x * checked_ray.direction.x
                + checked_ray.direction.y * checked_ray.direction.y
                + checked_ray.direction.z * checked_ray.direction.z
            )
            require(math.isfinite(local_norm), "source direction norm is non-finite")
            require(abs(local_norm - 1.0) < 1.0e-4, "source direction is not unit")
            require(abs(world_norm - 1.0) < 1.0e-9, "checked direction is not unit")
            require(
                abs(checked_ray.direction.x - source.dir_x / local_norm) < 2.0e-7
                and abs(checked_ray.direction.y - source.dir_y / local_norm) < 2.0e-7
                and abs(checked_ray.direction.z - source.dir_z / local_norm) < 2.0e-7,
                "identity TF changed the emitted direction",
            )
            require(
                source.offset_time_ns >= previous_offset,
                "checked source timestamps are not monotonic",
            )
            previous_offset = source.offset_time_ns
            require(
                source.return_status in status_counts,
                "checked source contains an undefined return status",
            )
            status_counts[source.return_status] += 1

        if self.mode == "empty":
            require(
                status_counts[Ray.NO_RETURN] == self.expected_ray_count,
                "empty world did not preserve 20,000 NO_RETURN rays",
            )
        else:
            require(status_counts[Ray.VALID_RETURN] > 0, "wall has no valid returns")
            require(status_counts[Ray.NO_RETURN] > 0, "finite wall has no no-returns")
        return status_counts

    def _validate_cloud_alignment(self, raw_cloud, valid_cloud, checked, counts):
        require(
            raw_cloud.width * raw_cloud.height == self.expected_ray_count,
            "raw compatibility cloud is not index-aligned with the 20,000 rays",
        )
        raw_names = tuple(field.name for field in raw_cloud.fields)
        require(
            all(name in raw_names for name in self.SOURCE_FIELDS),
            "raw cloud is missing Livox fields",
        )
        self._assert_source_layout_preserved(raw_cloud, valid_cloud)

        valid_count = valid_cloud.width * valid_cloud.height
        require(
            valid_count == counts[Ray.VALID_RETURN],
            "points_valid count disagrees with source VALID_RETURN count",
        )
        if self.mode == "empty":
            require(valid_count == 0, "empty world points_valid is not empty")
            return 0

        raw_points = list(
            point_cloud2.read_points(
                raw_cloud, field_names=self.SOURCE_FIELDS, skip_nans=False
            )
        )
        valid_points = list(
            point_cloud2.read_points(
                valid_cloud,
                field_names=self.SOURCE_FIELDS + ("original_index",),
                skip_nans=False,
            )
        )
        require(len(valid_points) == valid_count, "valid cloud layout/count mismatch")
        previous_index = -1
        for point in valid_points:
            source_index = int(point[-1])
            require(source_index > previous_index, "original_index order is not stable")
            require(source_index < len(raw_points), "original_index is out of bounds")
            previous_index = source_index
            require(
                tuple(point[:-1]) == tuple(raw_points[source_index]),
                "a raw Livox field changed while copying a valid point",
            )

            ray = checked.rays[source_index].source
            require(ray.return_status == Ray.VALID_RETURN, "valid point maps to non-valid ray")
            require(int(point[4]) == ray.tag, "point/ray tag mismatch")
            require(int(point[5]) == ray.line, "point/ray line mismatch")
            require(abs(float(point[3]) - ray.intensity) < 1.0e-5, "intensity mismatch")
            expected = (
                ray.range * ray.dir_x,
                ray.range * ray.dir_y,
                ray.range * ray.dir_z,
            )
            endpoint_error = math.sqrt(
                sum((float(point[axis]) - expected[axis]) ** 2 for axis in range(3))
            )
            require(
                endpoint_error <= 2.0e-3 + 2.0e-4 * ray.range,
                "valid point endpoint disagrees with range*direction",
            )
        return valid_count

    @staticmethod
    def _validate_world_cloud(world_cloud, checked):
        require(world_cloud.header.frame_id == checked.header.frame_id, "wrong world frame")
        required_fields = (
            "x",
            "y",
            "z",
            "intensity",
            "original_index",
            "offset_time_ns",
            "scan_id",
        )
        rows = list(
            point_cloud2.read_points(
                world_cloud, field_names=required_fields, skip_nans=False
            )
        )
        for row in rows:
            ray = checked.rays[int(row[4])]
            expected = (
                ray.origin.x + ray.source.range * ray.direction.x,
                ray.origin.y + ray.source.range * ray.direction.y,
                ray.origin.z + ray.source.range * ray.direction.z,
            )
            error = math.sqrt(
                sum((float(row[axis]) - expected[axis]) ** 2 for axis in range(3))
            )
            require(error < 1.0e-4, "points_world differs from checked-ray endpoint")
            require(int(row[5]) == ray.source.offset_time_ns, "offset_time_ns changed")
            require(int(row[6]) == checked.scan_id, "per-point scan_id changed")
        return len(rows)

    def _synchronized_callback(self, raw_cloud, valid_cloud, world_cloud, checked):
        try:
            require(
                valid_cloud.header.stamp == checked.header.stamp,
                "derived cloud/checked bundle are not an ExactTime pair",
            )
            require(
                raw_cloud.header.stamp == checked.header.stamp,
                "source cloud/checked bundle stamps are not identical",
            )
            counts = self._validate_checked_bundle(checked)
            valid_count = self._validate_cloud_alignment(
                raw_cloud, valid_cloud, checked, counts
            )
            require(
                self._validate_world_cloud(world_cloud, checked) == valid_count,
                "points_world count differs from points_valid",
            )
            with self.lock:
                require(
                    checked.scan_id not in self.checked_scan_ids,
                    "same checked scan was validated twice",
                )
                self.checked_scan_ids.add(checked.scan_id)
                self.pair_count += 1
                self.total_valid_points += valid_count
                self._update_completion_locked()
        except Exception as exception:
            self._set_failure(exception)

    def _diagnostics_callback(self, diagnostics):
        try:
            require(diagnostics.status, "preprocessor diagnostics is empty")
            status = diagnostics.status[0]
            if status.name != "mid360_ray_preprocessor/checked_rays":
                return
            # An initial TF startup race may legitimately emit an error before
            # the static transform arrives. Only count fully checked bundles.
            if status.level != DiagnosticStatus.OK:
                return
            values = {entry.key: entry.value for entry in status.values}
            required = {
                "ray_count",
                "scan_pair_ok",
                "scan_id",
                "point_source_stamp",
                "ray_source_stamp",
                "source_stamp_skew_ns",
                "valid_return_count",
                "matched_point_count",
                "duplicate_index_count",
                "missing_valid_return_count",
                "point_input_count",
                "point_valid_count",
                "direction_invalid_count",
                "timestamp_monotonic_ratio",
                "source_mode",
                "ray_time_geometry_mode",
                "tf_missing_ratio",
                "tf_missing_bundle_ratio",
                "tf_query_span_sec",
                "tf_source_sample_max_gap_sec",
                "tf_max_interval_semantics",
                "tf_lookup_count",
            }
            require(not (required - set(values)), "preprocessor diagnostics lacks required keys")
            require(values["ray_count"] == "20000", "diagnostic ray_count is not 20000")
            require(values["scan_pair_ok"] == "true", "scan pair contract failed")
            require(values["source_stamp_skew_ns"] == "0", "source stamps differ")
            require(values["duplicate_index_count"] == "0", "duplicate source index")
            require(values["missing_valid_return_count"] == "0", "missing valid return")
            require(
                values["valid_return_count"] == values["matched_point_count"],
                "not every valid return matched its source point",
            )
            require(values["point_input_count"] == "20000", "diagnostic cloud count is not 20000")
            require(values["direction_invalid_count"] == "0", "invalid checked directions")
            require(float(values["timestamp_monotonic_ratio"]) == 1.0, "timestamps are not monotonic")
            require(values["source_mode"] == "sim_exact", "diagnostic source_mode changed")
            require(values["ray_time_geometry_mode"] == "snapshot", "wrong TF mode diagnostic")
            require(values["tf_missing_ratio"] == "0", "diagnostics reports missing TF")
            require(
                values["tf_missing_bundle_ratio"] == "0",
                "diagnostics reports a missing TF bundle",
            )
            require(
                abs(float(values["tf_query_span_sec"])) < 1.0e-12,
                "snapshot diagnostics report a nonzero TF query span",
            )
            require(
                values["tf_source_sample_max_gap_sec"] == "not_observed",
                "diagnostics invent a TF source sample gap",
            )
            require(
                values["tf_max_interval_semantics"]
                == "deprecated_alias_of_tf_query_span_sec",
                "legacy TF interval key lacks explicit semantics",
            )
            require(values["tf_lookup_count"] == "1", "snapshot used more than one TF lookup")
            if self.mode == "empty":
                require(values["point_valid_count"] == "0", "empty diagnostic has valid points")
            else:
                require(int(values["point_valid_count"]) > 0, "wall diagnostic has no valid points")
            with self.lock:
                self.diagnostic_ok_count += 1
                self._update_completion_locked()
        except Exception as exception:
            self._set_failure(exception)

    def test_real_plugin_pipeline(self):
        completed = self.finished.wait(self.timeout_sec)
        self.assertTrue(completed, "timed out waiting for plugin/preprocessor outputs")
        with self.lock:
            failure = self.failure
            pair_count = self.pair_count
            diagnostic_count = self.diagnostic_ok_count
            total_valid_points = self.total_valid_points
        if failure is not None:
            self.fail(failure)
        self.assertGreaterEqual(pair_count, self.required_pairs)
        self.assertGreater(diagnostic_count, 0)
        if self.mode == "empty":
            self.assertEqual(total_valid_points, 0)
        else:
            self.assertGreater(total_valid_points, 0)
        rospy.loginfo(
            "validated %d real plugin pairs mode=%s total_valid_points=%d diagnostics=%d",
            pair_count,
            self.mode,
            total_valid_points,
            diagnostic_count,
        )


if __name__ == "__main__":
    rospy.init_node("preprocessor_plugin_integration_test")
    rostest.rosrun(
        "mid360_ray_preprocessor",
        "preprocessor_plugin_integration_test",
        PluginPreprocessorIntegrationTest,
    )
