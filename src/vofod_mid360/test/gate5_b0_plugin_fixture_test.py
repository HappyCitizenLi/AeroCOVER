#!/usr/bin/env python3
"""Real 20k-ray empty/wall contracts for the Mid-360 B0 adapter."""

import math
import threading
import time
import unittest

import rosgraph
import rospy
import rostest
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from vofod_mid360.msg import Detections, MapUpdateDiagnostics, Status


class Gate5B0PluginFixture(unittest.TestCase):
    def setUp(self):
        self.mode = str(rospy.get_param("~mode"))
        self.assertIn(self.mode, ("empty", "wall"))
        self.timeout_wall = float(rospy.get_param("~timeout_wall_sec", 35.0))
        self.required_scans = int(rospy.get_param("~required_scans", 10))
        self.lock = threading.Lock()
        self.failure = None
        self.diagnostics = []
        self.detections = []
        self.map_messages = 0
        self.map_points = 0
        self.marker_messages = 0
        self.detection_cloud_messages = 0
        self.status_active = 0
        self.free_cloud_messages = 0

        self.subscribers = [
            rospy.Subscriber(
                "/uav1/vofod_mid360/map_update_diagnostics",
                MapUpdateDiagnostics,
                self._diagnostics_callback,
                queue_size=100,
            ),
            rospy.Subscriber(
                "/uav1/vofod_mid360/detections",
                Detections,
                self._detections_callback,
                queue_size=100,
            ),
            rospy.Subscriber(
                "/uav1/vofod_mid360/voxel_map",
                Marker,
                self._map_callback,
                queue_size=10,
            ),
            rospy.Subscriber(
                "/uav1/vofod_mid360/detections_mks",
                MarkerArray,
                self._markers_callback,
                queue_size=20,
            ),
            rospy.Subscriber(
                "/uav1/vofod_mid360/detections_pc",
                PointCloud2,
                self._detection_cloud_callback,
                queue_size=20,
            ),
            rospy.Subscriber(
                "/uav1/vofod_mid360/free_voxels",
                PointCloud2,
                self._free_cloud_callback,
                queue_size=10,
            ),
            rospy.Subscriber(
                "/uav1/vofod_mid360/status",
                Status,
                self._status_callback,
                queue_size=20,
            ),
        ]

    def _record_failure(self, error):
        with self.lock:
            if self.failure is None:
                self.failure = str(error)
                rospy.logerr("B0 %s callback failure: %s", self.mode, error)

    def _diagnostics_callback(self, message):
        try:
            if message.header.frame_id != "world":
                raise AssertionError("diagnostics frame is not world")
            if message.input_ray_count != 20000:
                raise AssertionError("B0 did not consume 20,000 emitted rays")
            if message.accepted_valid_return + message.accepted_no_return != 20000:
                raise AssertionError("accepted valid/no-return counts do not conserve")
            if any(
                value
                for value in (
                    message.rejected_below_min_range,
                    message.rejected_invalid_range,
                    message.rejected_unknown_status,
                    message.rejected_body_mask,
                    message.rejected_self_occlusion,
                    message.rejected_transform,
                    message.rejected_geometry,
                    message.rejected_nonpositive_endpoint,
                )
            ):
                raise AssertionError("controlled plugin scan contained rejected rays")
            if not message.update_before_classification:
                raise AssertionError("B0 update/classification order changed")
            if message.free_updated_voxels <= 0 or message.positive_ray_segments <= 0:
                raise AssertionError("B0 produced no free-space update")
            if not math.isclose(message.free_update_weight_valid_return, 0.003, abs_tol=1e-7):
                raise AssertionError("valid-return B0 weight changed")
            if not math.isclose(message.free_update_weight_no_return, 0.003, abs_tol=1e-7):
                raise AssertionError("no-return B0 weight changed")
            if message.total_ms < 0.0 or not math.isfinite(message.total_ms):
                raise AssertionError("invalid B0 duration")
            if self.mode == "empty":
                if message.input_point_count != 0 or message.accepted_valid_return != 0:
                    raise AssertionError("empty fixture unexpectedly had valid returns")
                if message.accepted_no_return != 20000 or not message.empty_valid_cloud_processed:
                    raise AssertionError("empty B0 did not process all no-return rays")
                if message.detection_count != 0:
                    raise AssertionError("empty fixture produced a detection")
            else:
                if message.input_point_count <= 0 or message.accepted_valid_return <= 0:
                    raise AssertionError("wall fixture produced no valid returns")
                if message.accepted_no_return <= 0 or message.empty_valid_cloud_processed:
                    raise AssertionError("wall fixture lost no-return rays")
            with self.lock:
                self.diagnostics.append(message)
        except Exception as error:
            self._record_failure(error)

    def _detections_callback(self, message):
        try:
            if message.header.frame_id != "world":
                raise AssertionError("detection frame is not world")
            with self.lock:
                self.detections.append(message)
        except Exception as error:
            self._record_failure(error)

    def _map_callback(self, message):
        with self.lock:
            self.map_messages += 1
            self.map_points = max(self.map_points, len(message.points))

    def _markers_callback(self, _message):
        with self.lock:
            self.marker_messages += 1

    def _detection_cloud_callback(self, _message):
        with self.lock:
            self.detection_cloud_messages += 1

    def _free_cloud_callback(self, _message):
        with self.lock:
            self.free_cloud_messages += 1

    def _status_callback(self, message):
        with self.lock:
            if message.detection_enabled and message.detection_active:
                self.status_active += 1

    @staticmethod
    def _percentile(values, fraction):
        ordered = sorted(float(value) for value in values)
        index = int(round((len(ordered) - 1) * fraction))
        return ordered[index]

    def _check_graph_isolation(self):
        _publishers, subscribers, _services = rosgraph.Master(
            rospy.get_name()
        ).getSystemState()
        inputs = {
            topic
            for topic, nodes in subscribers
            if "/uav1/vofod_mid360" in nodes
        } - {"/clock"}
        self.assertEqual(
            inputs,
            {"/uav1/mid360/points_world", "/uav1/mid360/rays_checked"},
        )

    def test_real_plugin_b0_contract(self):
        deadline = time.monotonic() + self.timeout_wall
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.lock:
                ready = (
                    self.failure is not None
                    or (
                        len(self.diagnostics) >= self.required_scans
                        and len(self.detections) >= self.required_scans
                        and self.map_messages > 0
                        and self.marker_messages > 0
                        and self.detection_cloud_messages > 0
                        and (self.mode == "empty" or self.status_active > 0)
                        and self.free_cloud_messages > 0
                    )
                )
            if ready:
                break
            time.sleep(0.05)

        with self.lock:
            failure = self.failure
            diagnostics = list(self.diagnostics)
            detections = list(self.detections)
            counters = (
                self.map_messages,
                self.map_points,
                self.marker_messages,
                self.detection_cloud_messages,
                self.status_active,
                self.free_cloud_messages,
            )
        self.assertIsNone(failure, failure)
        self.assertGreaterEqual(len(diagnostics), self.required_scans)
        self.assertGreaterEqual(len(detections), self.required_scans)
        self.assertGreater(counters[0], 0)  # map messages
        self.assertGreater(counters[1], 0)  # visualized map voxels
        self.assertGreater(counters[2], 0)  # markers
        self.assertGreater(counters[3], 0)  # detection clouds
        self.assertGreater(counters[5], 0)  # free-space clouds
        if self.mode == "empty":
            self.assertEqual(counters[4], 0)  # warm-up cannot mature without returns
        else:
            self.assertGreater(counters[4], 0)
        revisions = [int(item.map_revision) for item in diagnostics]
        self.assertEqual(revisions, sorted(set(revisions)))
        if self.mode == "empty":
            self.assertTrue(all(not message.detections for message in detections))

        durations = [float(item.total_ms) for item in diagnostics]
        p50 = self._percentile(durations, 0.50)
        p95 = self._percentile(durations, 0.95)
        maximum = max(durations)
        self.assertLess(p95, 100.0)
        self._check_graph_isolation()
        rospy.loginfo(
            "B0 %s scans=%d total_ms p50=%.3f p95=%.3f max=%.3f",
            self.mode,
            len(durations),
            p50,
            p95,
            maximum,
        )


if __name__ == "__main__":
    rospy.init_node("gate5_b0_plugin_fixture_test")
    rostest.rosrun(
        "vofod_mid360", "gate5_b0_plugin_fixture", Gate5B0PluginFixture
    )
