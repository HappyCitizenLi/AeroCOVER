#!/usr/bin/env python3
"""Synthetic contract test for point filtering and both TF timing modes."""

import math
import struct
import threading
import time
import unittest

import rospy
import rostest
import tf2_ros
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import TransformStamped
from mid360_ray_msgs.msg import CheckedRayBundle, Ray, RayBundle, ScanIdentity
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField


class PreprocessorIntegrationTest(unittest.TestCase):
    POINTS = (
        (2.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (float("nan"), 1.0, 0.0),
        (50.0, 0.0, 0.0),
        (0.2, 0.0, 0.0),
        (0.0, 3.0, 0.0),
    )

    def setUp(self):
        self.mode = rospy.get_param("~mode")
        self.sensor_frame = rospy.get_param("~sensor_frame")
        self.input_points_topic = rospy.get_param("~input_points_topic")
        self.input_rays_topic = rospy.get_param("~input_rays_topic")
        self.input_scan_identity_topic = rospy.get_param("~input_scan_identity_topic")
        self.output_points_topic = rospy.get_param("~output_points_topic")
        self.output_world_points_topic = rospy.get_param("~output_world_points_topic")
        self.output_rays_topic = rospy.get_param("~output_rays_topic")
        self.diagnostics_topic = rospy.get_param("~diagnostics_topic")

        self.lock = threading.Lock()
        self.valid_cloud = None
        self.checked_bundle = None
        self.world_cloud = None
        self.diagnostics = None
        self.cloud_sub = rospy.Subscriber(
            self.output_points_topic, PointCloud2, self._cloud_callback, queue_size=2
        )
        self.rays_sub = rospy.Subscriber(
            self.output_rays_topic,
            CheckedRayBundle,
            self._rays_callback,
            queue_size=2,
        )
        self.world_cloud_sub = rospy.Subscriber(
            self.output_world_points_topic,
            PointCloud2,
            self._world_cloud_callback,
            queue_size=2,
        )
        self.diagnostics_sub = rospy.Subscriber(
            self.diagnostics_topic,
            DiagnosticArray,
            self._diagnostics_callback,
            queue_size=2,
        )
        self.cloud_pub = rospy.Publisher(
            self.input_points_topic, PointCloud2, queue_size=2
        )
        self.rays_pub = rospy.Publisher(
            self.input_rays_topic, RayBundle, queue_size=2
        )
        self.scan_identity_pub = rospy.Publisher(
            self.input_scan_identity_topic, ScanIdentity, queue_size=2
        )
        self.tf_broadcaster = (
            tf2_ros.TransformBroadcaster() if self.mode != "snapshot" else None
        )

    def _cloud_callback(self, message):
        with self.lock:
            self.valid_cloud = message

    def _rays_callback(self, message):
        with self.lock:
            self.checked_bundle = message

    def _world_cloud_callback(self, message):
        with self.lock:
            self.world_cloud = message

    def _diagnostics_callback(self, message):
        with self.lock:
            self.diagnostics = message

    @staticmethod
    def _transform(stamp, child, x, yaw):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "world"
        transform.child_frame_id = child
        transform.transform.translation.x = x
        transform.transform.rotation.z = math.sin(0.5 * yaw)
        transform.transform.rotation.w = math.cos(0.5 * yaw)
        return transform

    def _make_cloud(self, stamp, scan_id=42, source_indices=None):
        message = PointCloud2()
        message.header.seq = scan_id
        message.header.stamp = stamp
        message.header.frame_id = self.sensor_frame
        message.height = 1
        message.width = len(self.POINTS)
        message.fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 12, PointField.FLOAT32, 1),
            PointField("tag", 16, PointField.UINT8, 1),
            PointField("line", 17, PointField.UINT8, 1),
            PointField("timestamp", 20, PointField.FLOAT64, 1),
        ]
        if source_indices is not None:
            message.fields.append(
                PointField("original_index", 28, PointField.UINT32, 1)
            )
        message.is_bigendian = False
        message.point_step = 32 if source_indices is not None else 28
        message.row_step = message.point_step * message.width
        message.is_dense = False
        data = bytearray(message.row_step)
        for index, point in enumerate(self.POINTS):
            offset = index * message.point_step
            struct.pack_into("<fff", data, offset, *point)
            struct.pack_into("<f", data, offset + 12, 10.0 + index)
            struct.pack_into("<B", data, offset + 16, 20 + index)
            struct.pack_into("<B", data, offset + 17, index)
            struct.pack_into("<d", data, offset + 20, 1000.25 + index)
            if source_indices is not None:
                struct.pack_into("<I", data, offset + 28, source_indices[index])
        message.data = bytes(data)
        return message

    def _make_bundle(self, stamp, scan_id=42):
        bundle = RayBundle()
        bundle.header.seq = scan_id
        bundle.header.stamp = stamp
        bundle.header.frame_id = self.sensor_frame
        bundle.scan_id = scan_id
        bundle.pattern_start_index = 700
        bundle.min_range = 0.1
        bundle.max_range = 40.0
        for index in range(len(self.POINTS)):
            ray = Ray()
            ray.dir_x = 1.0
            ray.dir_y = 0.0
            ray.dir_z = 0.0
            ray.range = 2.0 if index == 0 else (3.0 if index == 5 else 0.0)
            ray.intensity = 100.0 + index
            ray.offset_time_ns = index * 20000000
            ray.pattern_index = 700 + index
            ray.tag = 30 + index
            ray.line = index
            ray.return_status = (
                Ray.VALID_RETURN if index in (0, 5) else Ray.NO_RETURN
            )
            bundle.rays.append(ray)
        return bundle

    def _make_identity(self, stamp, scan_id=42):
        identity = ScanIdentity()
        identity.header.stamp = stamp
        identity.header.frame_id = self.sensor_frame
        identity.scan_id = scan_id
        identity.pattern_start_index = 700
        identity.point_count = len(self.POINTS)
        identity.ray_count = len(self.POINTS)
        identity.point_source_stamp = stamp
        identity.ray_source_stamp = stamp
        return identity

    def _wait_for_connections(self):
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if (
                self.cloud_pub.get_num_connections()
                and self.rays_pub.get_num_connections()
                and self.scan_identity_pub.get_num_connections()
            ):
                return
            rospy.sleep(0.02)
        self.fail("preprocessor input subscriptions did not connect")

    def _assert_filtered_cloud(self, cloud, bundle_stamp):
        self.assertEqual(cloud.header.frame_id, self.sensor_frame)
        self.assertEqual(cloud.header.stamp, bundle_stamp)
        self.assertEqual(cloud.width, 2)
        field_names = [field.name for field in cloud.fields]
        self.assertEqual(
            field_names,
            ["x", "y", "z", "intensity", "tag", "line", "timestamp", "original_index"],
        )
        rows = list(
            point_cloud2.read_points(
                cloud,
                field_names=(
                    "x",
                    "y",
                    "z",
                    "intensity",
                    "tag",
                    "line",
                    "timestamp",
                    "original_index",
                ),
                skip_nans=False,
            )
        )
        self.assertEqual([row[7] for row in rows], [0, 5])
        for row, source_index in zip(rows, (0, 5)):
            expected_point = self.POINTS[source_index]
            self.assertAlmostEqual(row[0], expected_point[0], places=6)
            self.assertAlmostEqual(row[1], expected_point[1], places=6)
            self.assertAlmostEqual(row[2], expected_point[2], places=6)
            self.assertAlmostEqual(row[3], 10.0 + source_index, places=6)
            self.assertEqual(row[4], 20 + source_index)
            self.assertEqual(row[5], source_index)
            self.assertAlmostEqual(row[6], 1000.25 + source_index, places=9)

    def _assert_checked_bundle(self, bundle, source_stamp):
        self.assertEqual(bundle.header.frame_id, "world")
        self.assertEqual(bundle.header.stamp, source_stamp)
        self.assertEqual(bundle.source_header.frame_id, self.sensor_frame)
        self.assertEqual(bundle.source_header.stamp, source_stamp)
        self.assertEqual(bundle.scan_id, 42)
        self.assertEqual(bundle.pattern_start_index, 700)
        self.assertEqual(bundle.source_mode, "sim_exact")
        self.assertEqual(bundle.ray_time_geometry_mode, self.mode)
        self.assertEqual(len(bundle.rays), 6)
        for index, checked in enumerate(bundle.rays):
            self.assertEqual(checked.original_index, index)
            self.assertTrue(checked.direction_valid)
            self.assertTrue(checked.transform_valid)
            self.assertEqual(checked.source.pattern_index, 700 + index)
            self.assertEqual(checked.source.offset_time_ns, index * 20000000)
            self.assertAlmostEqual(checked.source.intensity, 100.0 + index, places=5)

        if self.mode == "snapshot":
            for checked in bundle.rays:
                self.assertAlmostEqual(checked.origin.x, 1.0, places=6)
                self.assertAlmostEqual(checked.origin.y, 2.0, places=6)
                self.assertAlmostEqual(checked.origin.z, 3.0, places=6)
                self.assertAlmostEqual(checked.direction.x, 0.0, places=6)
                self.assertAlmostEqual(checked.direction.y, 1.0, places=6)
                self.assertAlmostEqual(checked.direction.z, 0.0, places=6)
        else:
            for index, checked in enumerate(bundle.rays):
                alpha = index / 5.0
                yaw = alpha * math.pi / 2.0
                self.assertAlmostEqual(checked.origin.x, alpha, places=5)
                self.assertAlmostEqual(checked.origin.y, 0.0, places=6)
                self.assertAlmostEqual(checked.origin.z, 0.0, places=6)
                self.assertAlmostEqual(checked.direction.x, math.cos(yaw), places=5)
                self.assertAlmostEqual(checked.direction.y, math.sin(yaw), places=5)
                self.assertAlmostEqual(checked.direction.z, 0.0, places=6)

    def _assert_world_cloud(self, cloud, checked_bundle):
        self.assertEqual(cloud.header.frame_id, "world")
        self.assertEqual(cloud.header.stamp, checked_bundle.header.stamp)
        self.assertEqual(
            [field.name for field in cloud.fields],
            [
                "x",
                "y",
                "z",
                "intensity",
                "original_index",
                "offset_time_ns",
                "scan_id",
            ],
        )
        rows = list(
            point_cloud2.read_points(
                cloud,
                field_names=(
                    "x",
                    "y",
                    "z",
                    "intensity",
                    "original_index",
                    "offset_time_ns",
                    "scan_id",
                ),
                skip_nans=False,
            )
        )
        self.assertEqual([row[4] for row in rows], [0, 5])
        for row in rows:
            checked = checked_bundle.rays[row[4]]
            expected = (
                checked.origin.x + checked.source.range * checked.direction.x,
                checked.origin.y + checked.source.range * checked.direction.y,
                checked.origin.z + checked.source.range * checked.direction.z,
            )
            self.assertLess(
                math.sqrt(sum((row[axis] - expected[axis]) ** 2 for axis in range(3))),
                1.0e-5,
            )
            self.assertEqual(row[5], checked.source.offset_time_ns)
            self.assertEqual(row[6], checked_bundle.scan_id)
    def _assert_diagnostics(self, diagnostics):
        self.assertTrue(diagnostics.status)
        status = diagnostics.status[0]
        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "ok")
        values = {pair.key: pair.value for pair in status.values}
        self.assertEqual(values["ray_count"], "6")
        self.assertEqual(values["scan_pair_ok"], "true")
        self.assertEqual(values["scan_id"], "42")
        self.assertEqual(values["source_stamp_skew_ns"], "0")
        self.assertEqual(values["valid_return_count"], "2")
        self.assertEqual(values["matched_point_count"], "2")
        self.assertEqual(values["duplicate_index_count"], "0")
        self.assertEqual(values["missing_valid_return_count"], "0")
        self.assertEqual(values["no_return_count"], "4")
        self.assertEqual(values["point_input_count"], "6")
        self.assertEqual(values["point_valid_count"], "2")
        self.assertEqual(values["point_nonfinite_count"], "1")
        self.assertEqual(values["point_zero_count"], "1")
        self.assertEqual(values["point_range_rejected_count"], "1")
        self.assertEqual(values["point_self_rejected_count"], "1")
        self.assertEqual(values["input_index_alignment"], "true")
        self.assertEqual(values["tf_missing_ratio"], "0")
        self.assertEqual(values["tf_missing_bundle_ratio"], "0")
        self.assertEqual(values["tf_lookup_count"], "1" if self.mode == "snapshot" else "2")
        expected_span = 0.0 if self.mode == "snapshot" else 0.1
        self.assertAlmostEqual(float(values["tf_query_span_sec"]), expected_span, places=6)
        self.assertAlmostEqual(float(values["tf_max_interval_sec"]), expected_span, places=6)
        self.assertEqual(values["tf_source_sample_max_gap_sec"], "not_observed")
        self.assertEqual(
            values["tf_max_interval_semantics"],
            "deprecated_alias_of_tf_query_span_sec",
        )

    def test_preprocessor_contract(self):
        self._wait_for_connections()
        start_stamp = rospy.Time.now() - rospy.Duration(0.4)
        end_stamp = start_stamp + rospy.Duration(0.1)
        if self.tf_broadcaster is not None:
            transforms = [
                self._transform(start_stamp, self.sensor_frame, 0.0, 0.0),
                self._transform(end_stamp, self.sensor_frame, 1.0, math.pi / 2.0),
            ]
            for _ in range(10):
                self.tf_broadcaster.sendTransform(transforms)
                rospy.sleep(0.03)

        # A duplicate producer index is a core scan-contract failure and must
        # not leak any derived payload.
        duplicate_stamp = start_stamp - rospy.Duration(2.0)
        duplicate_cloud = self._make_cloud(
            duplicate_stamp, 40, (0, 0, 2, 3, 4, 5)
        )
        self.cloud_pub.publish(duplicate_cloud)
        self.rays_pub.publish(self._make_bundle(duplicate_stamp, 40))
        self.scan_identity_pub.publish(self._make_identity(duplicate_stamp, 40))
        deadline_wall = time.monotonic() + 3.0
        while not rospy.is_shutdown() and time.monotonic() < deadline_wall:
            rospy.sleep(0.02)
            with self.lock:
                if self.diagnostics is not None:
                    duplicate_diagnostics = self.diagnostics
                    break
        else:
            self.fail("timed out waiting for duplicate-index diagnostics")
        duplicate_values = {
            pair.key: pair.value
            for pair in duplicate_diagnostics.status[0].values
        }
        self.assertEqual(duplicate_diagnostics.status[0].level, DiagnosticStatus.ERROR)
        self.assertEqual(duplicate_values["scan_pair_ok"], "false")
        self.assertEqual(duplicate_values["duplicate_index_count"], "1")
        with self.lock:
            self.valid_cloud = None
            self.world_cloud = None
            self.checked_bundle = None
            self.diagnostics = None

        # ExactTime alone is insufficient: a foreign companion scan_id must
        # also fail closed.
        identity_stamp = start_stamp - rospy.Duration(1.0)
        foreign_identity = self._make_identity(identity_stamp, 41)
        foreign_identity.scan_id = 999
        self.cloud_pub.publish(self._make_cloud(identity_stamp, 41))
        self.rays_pub.publish(self._make_bundle(identity_stamp, 41))
        self.scan_identity_pub.publish(foreign_identity)
        deadline_wall = time.monotonic() + 3.0
        while not rospy.is_shutdown() and time.monotonic() < deadline_wall:
            rospy.sleep(0.02)
            with self.lock:
                if self.diagnostics is not None:
                    identity_diagnostics = self.diagnostics
                    break
        else:
            self.fail("timed out waiting for foreign-identity diagnostics")
        identity_values = {
            pair.key: pair.value
            for pair in identity_diagnostics.status[0].values
        }
        self.assertEqual(identity_diagnostics.status[0].level, DiagnosticStatus.ERROR)
        self.assertEqual(identity_values["scan_pair_ok"], "false")
        with self.lock:
            self.valid_cloud = None
            self.world_cloud = None
            self.checked_bundle = None
            self.diagnostics = None

        # ExactTime aligns stamps; ScanIdentity supplies the explicit scan_id
        # and source-count contract required before output.
        raw_cloud = self._make_cloud(start_stamp)
        raw_bundle = self._make_bundle(start_stamp)
        raw_identity = self._make_identity(start_stamp)
        self.cloud_pub.publish(raw_cloud)
        self.rays_pub.publish(raw_bundle)
        self.scan_identity_pub.publish(raw_identity)
        deadline = rospy.Time.now() + rospy.Duration(8.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.tf_broadcaster is not None:
                self.tf_broadcaster.sendTransform(transforms)
            rospy.sleep(0.05)
            with self.lock:
                if (
                    self.valid_cloud is not None
                    and self.world_cloud is not None
                    and self.checked_bundle is not None
                    and self.diagnostics is not None
                ):
                    valid_cloud = self.valid_cloud
                    world_cloud = self.world_cloud
                    checked_bundle = self.checked_bundle
                    diagnostics = self.diagnostics
                    break
        else:
            self.fail("timed out waiting for all preprocessor outputs")

        self._assert_filtered_cloud(valid_cloud, start_stamp)
        self._assert_checked_bundle(checked_bundle, start_stamp)
        self._assert_world_cloud(world_cloud, checked_bundle)
        self._assert_diagnostics(diagnostics)

        # A later callback carrying an older physical time is rejected as one
        # whole scan; no derived topic may leak from it.
        rospy.sleep(0.1)
        with self.lock:
            self.valid_cloud = None
            self.world_cloud = None
            self.checked_bundle = None
            self.diagnostics = None
        rollback_stamp = start_stamp - rospy.Duration(1.0)
        rollback_cloud = self._make_cloud(rollback_stamp, 43)
        rollback_bundle = self._make_bundle(rollback_stamp, 43)
        rollback_identity = self._make_identity(rollback_stamp, 43)
        self.cloud_pub.publish(rollback_cloud)
        self.rays_pub.publish(rollback_bundle)
        self.scan_identity_pub.publish(rollback_identity)
        deadline = rospy.Time.now() + rospy.Duration(3.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            rospy.sleep(0.05)
            with self.lock:
                if self.diagnostics is not None:
                    rollback_diagnostics = self.diagnostics
                    break
        else:
            self.fail("timed out waiting for rollback diagnostics")
        self.assertEqual(
            rollback_diagnostics.status[0].level, DiagnosticStatus.ERROR
        )
        rollback_values = {
            pair.key: pair.value
            for pair in rollback_diagnostics.status[0].values
        }
        self.assertEqual(rollback_values["scan_pair_ok"], "false")
        with self.lock:
            self.assertIsNone(self.valid_cloud)
            self.assertIsNone(self.world_cloud)
            self.assertIsNone(self.checked_bundle)


if __name__ == "__main__":
    rospy.init_node("preprocessor_integration_test")
    rostest.rosrun(
        "mid360_ray_preprocessor",
        "preprocessor_integration_test",
        PreprocessorIntegrationTest,
    )
