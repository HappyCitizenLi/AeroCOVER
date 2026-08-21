#!/usr/bin/env python3
"""Synthetic ROS contract for faithful Mid-360 B0 soft free updates.

The fixture publishes an ExactTime ``points_world``/``CheckedRayBundle`` pair
twice.  Each rejected category owns a spatially isolated ray corridor, while a
NaN-range NO_RETURN ray crosses the valid-return endpoint orthogonally.  This
lets the read-only voxel query prove safety-margin behavior, endpoint
protection, soft convergence, and absence of updates from rejected rays.
"""

import math
import struct
import threading
import time
import unittest

import rosgraph
import rospy
import rostest
from geometry_msgs.msg import Point, Vector3
from mid360_ray_msgs.msg import CheckedRay, CheckedRayBundle, Ray
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from vofod_mid360.msg import MapRevisionEvidence, MapUpdateDiagnostics
from vofod_mid360.srv import QueryVoxels, QueryVoxelsRequest


class B0SoftUpdateContract(unittest.TestCase):
    INIT_SCORE = -740.0
    FREE_SCORE = -1000.0

    # Probe positions are exact centres of 0.5 m voxels in the test map.
    PROBES = (
        ("valid_free", Point(2.25, 0.25, 0.25)),
        ("valid_margin", Point(3.75, 0.25, 0.25)),
        ("valid_endpoint", Point(4.25, 0.25, 0.25)),
        ("no_return_free", Point(4.25, -1.25, 0.25)),
        ("below_min", Point(2.25, 2.25, 0.25)),
        ("invalid_range", Point(2.25, 3.25, 0.25)),
        ("unknown_status", Point(2.25, 4.25, 0.25)),
        ("body_mask", Point(2.25, -1.25, 0.25)),
        ("missing_endpoint", Point(2.25, -2.25, 0.25)),
        ("bad_direction", Point(2.25, -3.25, 0.25)),
        ("bad_transform", Point(2.25, -4.25, 0.25)),
        ("nonpositive_endpoint", Point(2.25, 5.25, 0.25)),
        ("short_return_endpoint", Point(0.75, 5.25, 0.25)),
    )

    def setUp(self):
        self.timeout_wall = float(rospy.get_param("~timeout_wall_sec", 30.0))
        self.points_topic = rospy.get_param("~points_topic")
        self.rays_topic = rospy.get_param("~rays_topic")
        self.diagnostics_topic = rospy.get_param("~diagnostics_topic")
        self.map_revision_evidence_topic = rospy.get_param(
            "~map_revision_evidence_topic"
        )
        self.query_service = rospy.get_param("~query_service")
        self.production_node = rospy.get_param("~production_node")

        self.lock = threading.Lock()
        self.diagnostics = {}
        self.map_revision_evidence = {}
        self.points_pub = rospy.Publisher(
            self.points_topic, PointCloud2, queue_size=2
        )
        self.rays_pub = rospy.Publisher(
            self.rays_topic, CheckedRayBundle, queue_size=2
        )
        self.diagnostics_sub = rospy.Subscriber(
            self.diagnostics_topic,
            MapUpdateDiagnostics,
            self._diagnostics_callback,
            queue_size=10,
        )
        self.map_revision_evidence_sub = rospy.Subscriber(
            self.map_revision_evidence_topic,
            MapRevisionEvidence,
            self._map_revision_evidence_callback,
            queue_size=10,
        )
        self.query = rospy.ServiceProxy(self.query_service, QueryVoxels)

    def _diagnostics_callback(self, message):
        with self.lock:
            self.diagnostics[int(message.scan_id)] = message

    def _map_revision_evidence_callback(self, message):
        with self.lock:
            self.map_revision_evidence[int(message.scan_id)] = message

    def _wait_until(self, predicate, description):
        deadline = time.monotonic() + self.timeout_wall
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            value = predicate()
            if value:
                return value
            time.sleep(0.02)
        self.fail("timeout waiting for {}".format(description))

    def _wait_for_inputs(self):
        self._wait_until(
            lambda: self.points_pub.get_num_connections() > 0
            and self.rays_pub.get_num_connections() > 0
            and self.diagnostics_sub.get_num_connections() > 0
            and self.map_revision_evidence_sub.get_num_connections() > 0,
            "B0 inputs, diagnostics, and map evidence publishers",
        )

    def _wait_for_diagnostics(self, scan_id):
        def lookup():
            with self.lock:
                return self.diagnostics.get(scan_id)

        return self._wait_until(lookup, "diagnostics for scan {}".format(scan_id))

    def _wait_for_map_revision_evidence(self, scan_id):
        def lookup():
            with self.lock:
                return self.map_revision_evidence.get(scan_id)

        return self._wait_until(
            lookup, "map revision evidence for scan {}".format(scan_id)
        )

    def _assert_map_revision_evidence(self, value, diagnostics):
        self.assertEqual(value.schema_version, MapRevisionEvidence.SCHEMA_VERSION)
        self.assertEqual(
            value.bit_order, MapRevisionEvidence.BIT_ORDER_LINEAR_INDEX_LSB0
        )
        self.assertEqual(value.header, diagnostics.header)
        self.assertEqual(value.scan_id, diagnostics.scan_id)
        self.assertEqual(value.frame_token, 0)
        self.assertEqual(
            value.decision_map_view,
            MapRevisionEvidence.DECISION_MAP_VIEW_NOT_APPLICABLE,
        )
        self.assertEqual(value.decision_base_map_revision, 0)
        self.assertEqual(value.map_revision, diagnostics.map_revision)
        self.assertEqual(
            value.voxel_count,
            value.map_size_x * value.map_size_y * value.map_size_z,
        )
        packed_size = (value.voxel_count + 7) // 8
        self.assertEqual(len(value.sure_occupied_bits), packed_size)
        self.assertEqual(
            len(value.free_updated_this_revision_bits), packed_size
        )
        free_popcount = sum(
            bin(int(byte)).count("1")
            for byte in value.free_updated_this_revision_bits
        )
        self.assertEqual(
            free_popcount, value.free_updated_this_revision_count
        )
        self.assertEqual(
            value.free_updated_this_revision_count,
            diagnostics.free_updated_voxels,
        )
        used_tail_bits = value.voxel_count % 8
        if used_tail_bits:
            padding_mask = (0xFF << used_tail_bits) & 0xFF
            self.assertEqual(value.sure_occupied_bits[-1] & padding_mask, 0)
            self.assertEqual(
                value.free_updated_this_revision_bits[-1] & padding_mask, 0
            )

    @staticmethod
    def _make_points(stamp, scan_id):
        """Two valid endpoints with the points_world field contract."""
        endpoints = (
            # Accepted valid-return endpoint.
            (4.25, 0.25, 0.25, 0),
            # range <= safety margin: point evidence exists, free segment does not.
            (0.75, 5.25, 0.25, 9),
        )
        message = PointCloud2()
        message.header = Header(seq=scan_id, stamp=stamp, frame_id="world")
        message.height = 1
        message.width = len(endpoints)
        message.fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 12, PointField.FLOAT32, 1),
            PointField("tag", 16, PointField.UINT8, 1),
            PointField("line", 17, PointField.UINT8, 1),
            PointField("timestamp", 20, PointField.FLOAT64, 1),
            PointField("original_index", 28, PointField.UINT32, 1),
            PointField("offset_time_ns", 32, PointField.UINT32, 1),
            PointField("scan_id", 36, PointField.UINT32, 1),
        ]
        message.is_bigendian = False
        message.point_step = 40
        message.row_step = message.point_step * message.width
        message.is_dense = True
        data = bytearray(message.row_step)
        for row, (x, y, z, original_index) in enumerate(endpoints):
            offset = row * message.point_step
            struct.pack_into("<fff", data, offset, x, y, z)
            struct.pack_into("<f", data, offset + 12, 50.0 + row)
            struct.pack_into("<B", data, offset + 16, 0)
            struct.pack_into("<B", data, offset + 17, row)
            struct.pack_into("<d", data, offset + 20, stamp.to_sec())
            struct.pack_into("<I", data, offset + 28, original_index)
            struct.pack_into("<I", data, offset + 32, original_index * 1000)
            struct.pack_into("<I", data, offset + 36, scan_id)
        message.data = bytes(data)
        return message

    @staticmethod
    def _checked_ray(
        index,
        status,
        origin,
        direction,
        ray_range,
        pattern_index=None,
        direction_valid=True,
        transform_valid=True,
    ):
        checked = CheckedRay()
        checked.original_index = index
        checked.origin = Point(*origin)
        checked.direction = Vector3(*direction)
        checked.direction_valid = direction_valid
        checked.transform_valid = transform_valid

        source = Ray()
        source.dir_x, source.dir_y, source.dir_z = direction
        source.range = ray_range
        source.intensity = 100.0
        source.offset_time_ns = index * 1000
        source.pattern_index = index if pattern_index is None else pattern_index
        source.tag = 0
        source.line = index
        source.return_status = status
        checked.source = source
        return checked

    @classmethod
    def _make_bundle(cls, stamp, scan_id):
        bundle = CheckedRayBundle()
        bundle.header = Header(seq=scan_id, stamp=stamp, frame_id="world")
        bundle.source_header = Header(
            seq=scan_id, stamp=stamp, frame_id="synthetic_mid360"
        )
        bundle.scan_id = scan_id
        bundle.pattern_start_index = 0
        bundle.min_range = 0.1
        bundle.max_range = 10.0
        bundle.source_mode = "sim_exact"
        bundle.ray_time_geometry_mode = "snapshot"

        # 0: accepted valid return, endpoint available at x=4.25.
        bundle.rays.append(
            cls._checked_ray(
                0, Ray.VALID_RETURN, (0.25, 0.25, 0.25), (1.0, 0.0, 0.0), 4.0
            )
        )
        # 1: accepted NO_RETURN with NaN range. Its orthogonal path crosses the
        # valid endpoint voxel, which must remain protected by current points.
        bundle.rays.append(
            cls._checked_ray(
                1,
                Ray.NO_RETURN,
                (4.25, -5.25, 0.25),
                (0.0, 1.0, 0.0),
                float("nan"),
            )
        )
        bundle.rays.append(
            cls._checked_ray(
                2, Ray.BELOW_MIN_RANGE, (0.25, 2.25, 0.25), (1.0, 0.0, 0.0), 0.05
            )
        )
        bundle.rays.append(
            cls._checked_ray(
                3, Ray.INVALID_RANGE, (0.25, 3.25, 0.25), (1.0, 0.0, 0.0), float("nan")
            )
        )
        bundle.rays.append(
            cls._checked_ray(
                4, Ray.UNKNOWN_STATUS, (0.25, 4.25, 0.25), (1.0, 0.0, 0.0), 0.0
            )
        )
        bundle.rays.append(
            cls._checked_ray(
                5,
                Ray.NO_RETURN,
                (0.25, -1.25, 0.25),
                (1.0, 0.0, 0.0),
                float("nan"),
                pattern_index=999,
            )
        )
        # Valid return without a matching original_index endpoint is treated as
        # self-occluded/untrusted rather than carving free space.
        bundle.rays.append(
            cls._checked_ray(
                6, Ray.VALID_RETURN, (0.25, -2.25, 0.25), (1.0, 0.0, 0.0), 4.0
            )
        )
        # direction_valid is deliberately true: the non-unit vector must still
        # be rejected by the geometry/norm check.
        bundle.rays.append(
            cls._checked_ray(
                7, Ray.NO_RETURN, (0.25, -3.25, 0.25), (2.0, 0.0, 0.0), float("nan")
            )
        )
        bundle.rays.append(
            cls._checked_ray(
                8,
                Ray.NO_RETURN,
                (0.25, -4.25, 0.25),
                (1.0, 0.0, 0.0),
                float("nan"),
                transform_valid=False,
            )
        )
        # Endpoint exists, but range-margin <= 0, so the free segment is
        # rejected explicitly.
        bundle.rays.append(
            cls._checked_ray(
                9, Ray.VALID_RETURN, (0.75, 5.25, 0.75), (0.0, 0.0, -1.0), 0.5
            )
        )
        return bundle

    def _publish_scan(self, scan_id):
        # Wall time is used only to create a non-zero ExactTime key; the node is
        # not allowed to depend on /clock in this pure ROS contract.
        stamp = rospy.Time.from_sec(1000.0 + float(scan_id))
        self.points_pub.publish(self._make_points(stamp, scan_id))
        self.rays_pub.publish(self._make_bundle(stamp, scan_id))
        return self._wait_for_diagnostics(scan_id)

    def _query_all(self):
        request = QueryVoxelsRequest()
        request.points = [point for _name, point in self.PROBES]
        response = self.query(request)
        self.assertTrue(response.map_initialized)
        self.assertEqual(len(response.in_bounds), len(self.PROBES))
        self.assertEqual(len(response.scores), len(self.PROBES))
        self.assertEqual(len(response.update_counts), len(self.PROBES))
        self.assertEqual(len(response.last_update_classes), len(self.PROBES))
        self.assertTrue(all(response.in_bounds))
        return {
            name: (float(score), int(count), int(update_class))
            for (name, _point), score, count, update_class in zip(
                self.PROBES,
                response.scores,
                response.update_counts,
                response.last_update_classes,
            )
        }, int(response.map_revision)

    def _assert_diagnostics(self, value, previous_revision):
        self.assertEqual(value.header.frame_id, "world")
        self.assertEqual(value.source_mode, "sim_exact")
        self.assertEqual(value.ray_time_geometry_mode, "snapshot")
        self.assertEqual(value.input_point_count, 2)
        self.assertEqual(value.input_ray_count, 10)
        self.assertEqual(value.accepted_valid_return, 1)
        self.assertEqual(value.accepted_no_return, 1)
        self.assertEqual(value.rejected_below_min_range, 1)
        self.assertEqual(value.rejected_invalid_range, 1)
        self.assertEqual(value.rejected_unknown_status, 1)
        self.assertEqual(value.rejected_body_mask, 1)
        self.assertEqual(value.rejected_self_occlusion, 1)
        self.assertEqual(value.rejected_transform, 1)
        self.assertEqual(value.rejected_geometry, 1)
        self.assertEqual(value.rejected_nonpositive_endpoint, 1)
        self.assertEqual(value.start_outside_map, 0)
        self.assertTrue(value.first_input_ack)
        self.assertNotEqual(value.background_warmup_start_stamp, rospy.Time())
        self.assertLessEqual(value.background_warmup_start_stamp, value.header.stamp)
        self.assertEqual(value.background_warmup_complete_stamp, rospy.Time())
        self.assertFalse(value.background_warmup_active)
        self.assertTrue(value.background_warmup_complete)
        self.assertEqual(value.warmup_background_clusters, 0)
        self.assertFalse(value.background_points_sufficient)
        self.assertFalse(value.sure_background_sufficient)
        # This field counts positive voxel-length segments, not accepted rays.
        self.assertGreater(value.positive_ray_segments, 2)
        self.assertGreater(value.touched_voxels, 0)
        self.assertGreater(value.free_updated_voxels, 0)
        self.assertEqual(value.point_updated_voxels, 0)
        self.assertGreaterEqual(value.unknown_updated_voxels, 2)
        self.assertGreaterEqual(value.protected_voxels, 1)
        self.assertGreater(value.valid_return_path_m, 0.0)
        self.assertGreater(value.no_return_path_m, 0.0)
        self.assertAlmostEqual(value.free_update_weight_valid_return, 0.10, places=6)
        self.assertAlmostEqual(value.free_update_weight_no_return, 0.05, places=6)
        self.assertAlmostEqual(value.valid_return_safety_margin, 1.0, places=6)
        self.assertAlmostEqual(value.raycast_max_distance, 6.0, places=6)
        self.assertAlmostEqual(value.reliable_no_return_distance, 6.0, places=6)
        self.assertGreater(value.map_revision, previous_revision)
        self.assertTrue(value.update_before_classification)
        for duration in (
            value.point_update_ms,
            value.ray_accumulation_ms,
            value.ray_apply_ms,
            value.classification_ms,
            value.total_ms,
        ):
            self.assertGreaterEqual(duration, 0.0)

    @staticmethod
    def _algorithm_inputs(master, node_name):
        _publishers, subscribers, _services = master.getSystemState()
        return {
            topic for topic, nodes in subscribers if node_name in nodes
        }

    def _assert_graph_isolation(self):
        master = rosgraph.Master(rospy.get_name())
        inputs = self._algorithm_inputs(master, self.production_node)
        algorithm_inputs = inputs - {"/clock"}
        self.assertEqual(
            algorithm_inputs,
            {self.points_topic, self.rays_topic},
            "production B0 data subscriptions changed: {}".format(sorted(inputs)),
        )
        forbidden = (
            "/tracker",
            "/tclv",
            "/evaluation",
            "/ground_truth",
            "/gazebo",
            "/scenario_events",
        )
        self.assertFalse(
            any(fragment in topic for fragment in forbidden for topic in algorithm_inputs)
        )

    def test_soft_update_contract(self):
        rospy.wait_for_service(self.query_service, timeout=self.timeout_wall)
        self._wait_for_inputs()
        initial, initial_revision = self._query_all()
        for name, (score, count, _update_class) in initial.items():
            self.assertAlmostEqual(score, self.INIT_SCORE, places=5, msg=name)
            self.assertEqual(count, 0, name)

        first_diag = self._publish_scan(100)
        self._assert_diagnostics(first_diag, initial_revision)
        self._assert_map_revision_evidence(
            self._wait_for_map_revision_evidence(100), first_diag
        )
        first, first_revision = self._query_all()
        self.assertGreaterEqual(first_revision, first_diag.map_revision)

        # Accepted rays move scores softly toward FREE_SCORE, never hard-set.
        for name in ("valid_free", "no_return_free"):
            score, count, _update_class = first[name]
            self.assertGreater(score, self.FREE_SCORE, name)
            self.assertLess(score, self.INIT_SCORE, name)
            self.assertGreater(count, 0, name)

        # The valid safety margin is untouched. The orthogonal no-return ray
        # reaches the endpoint voxel, but current point evidence protects it.
        self.assertAlmostEqual(first["valid_margin"][0], self.INIT_SCORE, places=5)
        self.assertEqual(first["valid_margin"][1], 0)
        # Depending on point/background classification, point evidence may keep
        # the initial score or raise it; it must never be carved toward free.
        self.assertGreaterEqual(first["valid_endpoint"][0], self.INIT_SCORE)
        self.assertGreater(first["valid_endpoint"][1], 0)

        # Without warm-up, the short endpoint remains UNKNOWN point evidence;
        # it must not be promoted by a direction-specific startup shortcut.
        seed_score, seed_count, seed_class = first["short_return_endpoint"]
        self.assertAlmostEqual(seed_score, self.INIT_SCORE, places=5)
        self.assertGreater(seed_count, 0)
        self.assertEqual(seed_class, 5)  # VoFOD nodelet UNKNOWN_POINT.

        rejected_probes = (
            "below_min",
            "invalid_range",
            "unknown_status",
            "body_mask",
            "missing_endpoint",
            "bad_direction",
            "bad_transform",
            "nonpositive_endpoint",
        )
        for name in rejected_probes:
            score, count, _update_class = first[name]
            self.assertAlmostEqual(score, self.INIT_SCORE, places=5, msg=name)
            self.assertEqual(count, 0, name)

        second_diag = self._publish_scan(101)
        self._assert_diagnostics(second_diag, first_revision)
        self._assert_map_revision_evidence(
            self._wait_for_map_revision_evidence(101), second_diag
        )
        second, second_revision = self._query_all()
        self.assertGreaterEqual(second_revision, second_diag.map_revision)
        self.assertGreater(second_revision, first_revision)

        # A second identical frame converges monotonically without reaching the
        # hard free score. NO_RETURN's NaN source range did not suppress it.
        for name in ("valid_free", "no_return_free"):
            first_score, first_count, _ = first[name]
            second_score, second_count, _ = second[name]
            self.assertLess(second_score, first_score, name)
            self.assertGreater(second_score, self.FREE_SCORE, name)
            self.assertGreater(second_count, first_count, name)

        for name in rejected_probes:
            self.assertAlmostEqual(second[name][0], self.INIT_SCORE, places=5, msg=name)
            self.assertEqual(second[name][1], 0, name)

        self._assert_graph_isolation()


if __name__ == "__main__":
    rospy.init_node("b0_soft_update_test")
    rostest.rosrun(
        "vofod_mid360", "b0_soft_update_contract", B0SoftUpdateContract
    )
