#!/usr/bin/env python3

import threading
import time
import unittest

import rospy
import rostest
from diagnostic_msgs.msg import DiagnosticArray
from mid360_ray_msgs.msg import CheckedRay, CheckedRayBundle, Ray
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from soft_vofod_mid360.msg import FreeSpaceViolationEvents, SoftTracks
from std_msgs.msg import Header


FIELDS = [
    PointField("x", 0, PointField.FLOAT32, 1),
    PointField("y", 4, PointField.FLOAT32, 1),
    PointField("z", 8, PointField.FLOAT32, 1),
    PointField("intensity", 12, PointField.FLOAT32, 1),
    PointField("original_index", 16, PointField.UINT32, 1),
    PointField("offset_time_ns", 20, PointField.UINT32, 1),
    PointField("scan_id", 24, PointField.UINT32, 1),
]


class PipelineTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._events = {}
        self._tracks = {}
        self._diagnostics = {}
        self._points_pub = rospy.Publisher(
            "/test/points_world", PointCloud2, queue_size=2
        )
        self._rays_pub = rospy.Publisher(
            "/test/rays_checked", CheckedRayBundle, queue_size=2
        )
        self._event_sub = rospy.Subscriber(
            "/soft_vofod/events", FreeSpaceViolationEvents, self._on_events
        )
        self._track_sub = rospy.Subscriber(
            "/soft_vofod/tracks", SoftTracks, self._on_tracks
        )
        self._diagnostics_sub = rospy.Subscriber(
            "/soft_vofod/diagnostics", DiagnosticArray, self._on_diagnostics
        )

    def _on_events(self, message):
        with self._lock:
            self._events[message.header.seq] = message

    def _on_tracks(self, message):
        with self._lock:
            self._tracks[message.header.seq] = message

    def _on_diagnostics(self, message):
        with self._lock:
            self._diagnostics[message.header.stamp.to_nsec()] = message

    def _wait_for_connections(self):
        deadline = time.time() + 10.0
        while time.time() < deadline and not rospy.is_shutdown():
            if (
                self._points_pub.get_num_connections() > 0
                and self._rays_pub.get_num_connections() > 0
            ):
                return
            rospy.sleep(0.05)
        self.fail("SOFT-VoFOD input subscribers did not connect")

    def _wait_for_scan(self, scan_id):
        deadline = time.time() + 5.0
        while time.time() < deadline and not rospy.is_shutdown():
            with self._lock:
                if scan_id in self._events and scan_id in self._tracks:
                    return self._events[scan_id], self._tracks[scan_id]
            rospy.sleep(0.02)
        self.fail("timed out waiting for scan {} output".format(scan_id))

    def _wait_for_diagnostics(self, stamp):
        deadline = time.time() + 5.0
        while time.time() < deadline and not rospy.is_shutdown():
            with self._lock:
                message = self._diagnostics.get(stamp.to_nsec())
            if message is not None:
                return message
            rospy.sleep(0.02)
        self.fail("timed out waiting for diagnostics at {}".format(stamp))

    def _publish(self, scan_id, stamp, valid_return, retain_endpoint=True):
        header = Header(seq=scan_id, stamp=stamp, frame_id="world")
        rows = [(5.0, 0.0, 0.0, 1.0, 0, 0, scan_id)] \
            if valid_return and retain_endpoint else []
        points = point_cloud2.create_cloud(header, FIELDS, rows)

        source = Ray()
        source.dir_x = 1.0
        source.range = 5.0 if valid_return else 0.0
        source.intensity = 1.0 if valid_return else 0.0
        source.offset_time_ns = 0
        source.pattern_index = scan_id
        source.return_status = Ray.VALID_RETURN if valid_return else Ray.NO_RETURN
        checked = CheckedRay()
        checked.original_index = 0
        checked.source = source
        checked.origin.x = 0.0
        checked.direction.x = 1.0
        checked.direction_valid = True
        checked.transform_valid = True
        bundle = CheckedRayBundle()
        bundle.header = header
        bundle.source_header = header
        bundle.scan_id = scan_id
        bundle.pattern_start_index = scan_id
        bundle.min_range = 0.1
        bundle.max_range = 60.0
        bundle.source_mode = "test"
        bundle.ray_time_geometry_mode = "snapshot"
        bundle.rays = [checked]
        self._points_pub.publish(points)
        self._rays_pub.publish(bundle)

    def test_free_event_then_trajectory_birth(self):
        self._wait_for_connections()
        start = rospy.Time.now() + rospy.Duration(0.2)
        self._publish(0, start, True, retain_endpoint=False)
        events, tracks = self._wait_for_scan(0)
        self.assertEqual(len(events.events), 0)
        self.assertEqual(len(tracks.tracks), 0)
        diagnostic = self._wait_for_diagnostics(start)
        values = {item.key: item.value for item in diagnostic.status[0].values}
        self.assertEqual(values["first_input_ack"], "true")
        self.assertAlmostEqual(float(values["map_bootstrap_start_stamp"]),
                               start.to_sec(), places=6)

        # Cover at least three 0.2 s epochs regardless of the absolute ROS
        # time phase at which this rostest starts.
        for scan_id in range(1, 9):
            self._publish(
                scan_id, start + rospy.Duration(0.1 * scan_id), False
            )
            events, tracks = self._wait_for_scan(scan_id)
            self.assertEqual(len(events.events), 0)
            self.assertEqual(len(tracks.tracks), 0)

        packet_count = 0
        packet_point_counts = []
        born_track = None
        for scan_id in range(9, 19):
            self._publish(
                scan_id, start + rospy.Duration(0.1 * scan_id), True
            )
            events, tracks = self._wait_for_scan(scan_id)
            packet_count += len(events.events)
            packet_point_counts.extend(event.point_count for event in events.events)
            if tracks.tracks:
                born_track = tracks.tracks[0]

        self._publish(19, start + rospy.Duration(1.9), False)
        events, tracks = self._wait_for_scan(19)
        packet_count += len(events.events)
        packet_point_counts.extend(event.point_count for event in events.events)
        if tracks.tracks:
            born_track = tracks.tracks[0]

        self.assertGreaterEqual(packet_count, 3)
        self.assertTrue(all(count == 1 for count in packet_point_counts))
        self.assertIsNotNone(born_track)
        self.assertAlmostEqual(born_track.position.x, 5.0, places=3)


if __name__ == "__main__":
    rospy.init_node("soft_vofod_pipeline_test")
    rostest.rosrun("soft_vofod_mid360", "soft_vofod_pipeline", PipelineTest)
