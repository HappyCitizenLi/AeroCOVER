#!/usr/bin/env python3
import threading
import time
import unittest

import rospy
import rostest
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from lidar_tracker_mid360.msg import TrackerFrameComplete, Tracks
from vofod_mid360.msg import Detection, Detections


class FrameCompletionContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("frame_completion_contract", anonymous=True)
        cls._lock = threading.Lock()
        cls._event_sequence = 0
        cls._track_events = []
        cls._completion_events = []
        cls._detections_pub = rospy.Publisher(
            "/frame_completion_test/detections", Detections, queue_size=64
        )
        cls._points_pub = rospy.Publisher(
            "/frame_completion_test/points", PointCloud2, queue_size=64
        )
        cls._tracks_sub = rospy.Subscriber(
            "/frame_completion_test/tracks", Tracks, cls._tracks_cb,
            queue_size=50,
        )
        cls._completion_sub = rospy.Subscriber(
            "/frame_completion_test/frame_complete", TrackerFrameComplete,
            cls._completion_cb, queue_size=64,
        )
        deadline = time.time() + 12.0
        while time.time() < deadline and (
            cls._detections_pub.get_num_connections() == 0
            or cls._points_pub.get_num_connections() == 0
        ):
            rospy.sleep(0.05)
        if cls._detections_pub.get_num_connections() == 0:
            raise RuntimeError("tracker did not subscribe to detections fixture")
        if cls._points_pub.get_num_connections() == 0:
            raise RuntimeError("tracker did not subscribe to points fixture")

    @classmethod
    def _tracks_cb(cls, message):
        with cls._lock:
            cls._event_sequence += 1
            cls._track_events.append((cls._event_sequence, message))

    @classmethod
    def _completion_cb(cls, message):
        with cls._lock:
            cls._event_sequence += 1
            cls._completion_events.append((cls._event_sequence, message))

    @staticmethod
    def _empty_cloud(stamp):
        cloud = PointCloud2()
        cloud.header = Header(stamp=stamp, frame_id="world")
        cloud.height = 1
        cloud.width = 0
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = 0
        cloud.is_dense = True
        cloud.data = b""
        return cloud

    @staticmethod
    def _detections(stamp, positions):
        message = Detections()
        message.header = Header(stamp=stamp, frame_id="world")
        for index, x in enumerate(positions):
            detection = Detection()
            detection.id = index + 1
            detection.confidence = 0.9
            detection.detection_probability = 0.9
            detection.n_points = 8
            detection.position.x = x
            detection.position.y = 0.0
            detection.position.z = 1.0
            detection.covariance = [
                0.25, 0.0, 0.0,
                0.0, 0.25, 0.0,
                0.0, 0.0, 0.25,
            ]
            message.detections.append(detection)
        return message

    @staticmethod
    def _pcl_exact_now():
        # pcl::PCLHeader stores microseconds.  Quantization avoids introducing
        # a sub-microsecond negative dt when the exact same ROS stamp is
        # round-tripped through the point-cloud header.
        now = rospy.Time.now()
        return rospy.Time(now.secs, (now.nsecs // 1000) * 1000)

    def _wait_completion(self, stamp, timeout=8.0):
        deadline = time.time() + timeout
        stamp_ns = stamp.to_nsec()
        while time.time() < deadline and not rospy.is_shutdown():
            with self._lock:
                matches = [
                    item for item in self._completion_events
                    if item[1].header.stamp.to_nsec() == stamp_ns
                ]
            if matches:
                return matches[-1]
            rospy.sleep(0.02)
        self.fail("timed out waiting for frame completion at %d" % stamp_ns)

    def _assert_same_stamp_publications(self, stamp, expected_tracks):
        # Tracks and frame_complete use independent TCPROS connections, so
        # subscriber callback arrival order is not a valid cross-topic
        # contract.  Drain both connections and verify the exact same-stamp
        # publication cardinality instead.
        rospy.sleep(0.25)
        stamp_ns = stamp.to_nsec()
        with self._lock:
            track_events = [
                sequence for sequence, message in self._track_events
                if message.source_header.stamp.to_nsec() == stamp_ns
            ]
            completion_count = sum(
                message.header.stamp.to_nsec() == stamp_ns
                for _, message in self._completion_events
            )
        self.assertEqual(len(track_events), expected_tracks)
        self.assertEqual(completion_count, 1)

    def test_full_batch_empty_batch_fifo_and_duplicates(self):
        # A full two-detection batch must produce two detection-side Tracks
        # publications plus the points-side publication before completion.
        stamp_multiple = self._pcl_exact_now()
        self._detections_pub.publish(
            self._detections(stamp_multiple, [1.0, 20.0])
        )
        rospy.sleep(0.08)
        self._points_pub.publish(self._empty_cloud(stamp_multiple))
        _, multiple = self._wait_completion(stamp_multiple)
        self.assertEqual(multiple.completion_sequence, 0)
        self.assertEqual(multiple.points_status, TrackerFrameComplete.STATUS_OK)
        self.assertEqual(
            multiple.detections_status, TrackerFrameComplete.STATUS_OK
        )
        self.assertEqual(multiple.points_tracks_publications, 1)
        self.assertEqual(multiple.detections_tracks_publications, 2)
        self.assertEqual(multiple.detections_received, 2)
        self._assert_same_stamp_publications(stamp_multiple, 3)

        # Empty detections are completed work, not a missing half-frame.
        rospy.sleep(0.03)
        stamp_empty = self._pcl_exact_now()
        self._detections_pub.publish(self._detections(stamp_empty, []))
        rospy.sleep(0.08)
        self._points_pub.publish(self._empty_cloud(stamp_empty))
        _, empty = self._wait_completion(stamp_empty)
        self.assertEqual(empty.completion_sequence, 1)
        self.assertEqual(
            empty.detections_status, TrackerFrameComplete.STATUS_EMPTY
        )
        self.assertEqual(empty.detections_tracks_publications, 0)
        self.assertEqual(empty.detections_received, 0)
        self._assert_same_stamp_publications(stamp_empty, 1)

        # The subscriber callbacks must enqueue every message, rather than
        # exposing only the latest value to the workers.  Publish more than ten
        # frames per side without pacing so a one-slot latest-value handoff
        # deterministically loses work while the bounded FIFO retains it.
        burst_count = 16
        burst_start = self._pcl_exact_now()
        burst_stamps = [
            burst_start + rospy.Duration(0, index * 1000000)
            for index in range(burst_count)
        ]
        for stamp in burst_stamps:
            self._detections_pub.publish(self._detections(stamp, []))
        for stamp in burst_stamps:
            self._points_pub.publish(self._empty_cloud(stamp))

        self._wait_completion(burst_stamps[-1], timeout=12.0)
        rospy.sleep(0.25)
        burst_ns = {stamp.to_nsec() for stamp in burst_stamps}
        with self._lock:
            burst_completions = [
                (sequence, message)
                for sequence, message in self._completion_events
                if message.header.stamp.to_nsec() in burst_ns
            ]
            burst_tracks = [
                (sequence, message)
                for sequence, message in self._track_events
                if message.source_header.stamp.to_nsec() in burst_ns
            ]
        self.assertEqual(len(burst_completions), burst_count)
        self.assertEqual(len(burst_tracks), burst_count)
        self.assertEqual(
            [message.header.stamp.to_nsec() for _, message in burst_completions],
            [stamp.to_nsec() for stamp in burst_stamps],
        )
        self.assertEqual(
            [message.completion_sequence for _, message in burst_completions],
            list(range(2, 2 + burst_count)),
        )
        track_event_by_stamp = {
            message.source_header.stamp.to_nsec(): sequence
            for sequence, message in burst_tracks
        }
        for _, message in burst_tracks:
            self.assertTrue(all(track.last_prediction == message.header.stamp
                                for track in message.tracks))
        self.assertEqual(set(track_event_by_stamp), burst_ns)
        for _, message in burst_completions:
            self.assertEqual(message.points_status, TrackerFrameComplete.STATUS_OK)
            self.assertEqual(
                message.detections_status, TrackerFrameComplete.STATUS_EMPTY
            )
            self.assertEqual(message.points_tracks_publications, 1)
            self.assertEqual(message.detections_tracks_publications, 0)
            self.assertEqual(message.duplicate_inputs_dropped, 0)
            self.assertEqual(message.regressive_inputs_dropped, 0)
            self.assertEqual(message.incomplete_frames_dropped, 0)
            self.assertEqual(message.late_finishes_dropped, 0)

        # A duplicate input is rejected before processing.  It cannot publish
        # another Tracks message or another completion for the same stamp.
        rospy.sleep(0.03)
        stamp_duplicate = self._pcl_exact_now()
        duplicate_message = self._detections(stamp_duplicate, [40.0])
        self._detections_pub.publish(duplicate_message)
        rospy.sleep(0.15)
        self._detections_pub.publish(duplicate_message)
        rospy.sleep(0.15)
        self._points_pub.publish(self._empty_cloud(stamp_duplicate))
        _, duplicate = self._wait_completion(stamp_duplicate)
        self.assertEqual(duplicate.completion_sequence, 2 + burst_count)
        self.assertEqual(duplicate.detections_tracks_publications, 1)
        self.assertGreaterEqual(duplicate.duplicate_inputs_dropped, 1)
        self._assert_same_stamp_publications(stamp_duplicate, 2)

        with self._lock:
            completion_stamps = [
                message.header.stamp.to_nsec()
                for _, message in self._completion_events
            ]
        self.assertEqual(completion_stamps, sorted(completion_stamps))
        self.assertEqual(len(completion_stamps), len(set(completion_stamps)))


if __name__ == "__main__":
    rostest.rosrun(
        "lidar_tracker_mid360",
        "frame_completion_contract",
        FrameCompletionContract,
    )
