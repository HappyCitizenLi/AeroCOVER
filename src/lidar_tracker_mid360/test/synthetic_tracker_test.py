#!/usr/bin/env python3
import os
import threading
import time
import unittest

import rosgraph
import rospkg
import rospy
import rostest
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

from lidar_tracker_mid360.msg import Tracks
from vofod_mid360.msg import Detection, Detections


class SyntheticTrackerContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("synthetic_tracker_contract", anonymous=True)
        cls._lock = threading.Lock()
        cls._tracks = []
        cls._det_pub = rospy.Publisher(
            "/baseline_test/detections_fixture", Detections, queue_size=10
        )
        cls._points_pub = rospy.Publisher(
            "/baseline_test/points_world_fixture", PointCloud2, queue_size=10
        )
        cls._background_pub = rospy.Publisher(
            "/background_empty_test/empty_background_fixture",
            PointCloud2,
            queue_size=2,
        )
        cls._tracks_sub = rospy.Subscriber(
            "/baseline_test/tracks", Tracks, cls._tracks_cb, queue_size=20
        )
        deadline = time.time() + 12.0
        while time.time() < deadline and (
            cls._det_pub.get_num_connections() == 0
            or cls._points_pub.get_num_connections() == 0
            or cls._background_pub.get_num_connections() == 0
        ):
            rospy.sleep(0.05)
        if cls._det_pub.get_num_connections() == 0:
            raise RuntimeError("tracker did not subscribe to VoFOD-Mid360 detections")
        if cls._points_pub.get_num_connections() == 0:
            raise RuntimeError("tracker did not subscribe to points_world")
        if cls._background_pub.get_num_connections() == 0:
            raise RuntimeError("enabled background tracker did not subscribe")

    @classmethod
    def _tracks_cb(cls, message):
        with cls._lock:
            cls._tracks.append(message)

    def _wait_for(self, predicate, timeout=8.0):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            with self._lock:
                messages = list(self._tracks)
            for message in reversed(messages):
                if predicate(message):
                    return message
            rospy.sleep(0.03)
        self.fail("timed out waiting for tracker output")

    @staticmethod
    def _detection_message(x, detection_id):
        message = Detections()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "world"
        detection = Detection()
        detection.id = detection_id
        detection.confidence = 0.9
        detection.n_points = 8
        detection.position.x = x
        detection.position.y = 0.0
        detection.position.z = 1.0
        detection.covariance = [
            0.25, 0.0, 0.0,
            0.0, 0.25, 0.0,
            0.0, 0.0, 0.25,
        ]
        detection.detection_probability = 0.9
        message.detections = [detection]
        return message

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
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = 0
        cloud.is_dense = True
        cloud.data = b""
        return cloud

    def test_01_stable_ids_and_nearest_association(self):
        self._det_pub.publish(self._detection_message(1.0, 10))
        first = self._wait_for(lambda msg: len(msg.tracks) == 1)
        stable_id = first.tracks[0].id
        self.assertNotEqual(stable_id, 0)

        self._det_pub.publish(self._detection_message(1.15, 11))
        associated = self._wait_for(
            lambda msg: len(msg.tracks) == 1
            and msg.tracks[0].id == stable_id
            and msg.tracks[0].n_detections >= 2
        )
        self.assertEqual(associated.tracks[0].id, stable_id)

        self._det_pub.publish(self._detection_message(20.0, 12))
        separated = self._wait_for(lambda msg: len(msg.tracks) == 2)
        ids = {track.id for track in separated.tracks}
        self.assertIn(stable_id, ids)
        self.assertEqual(len(ids), 2)
        self.__class__._stable_ids = ids

        invalid = self._detection_message(2.0, 13)
        invalid.detections[0].position.x = float("nan")
        self._det_pub.publish(invalid)
        rospy.sleep(0.3)
        with self._lock:
            latest = self._tracks[-1]
        self.assertEqual(len(latest.tracks), 2)

    def test_02_local_pointcloud_clusters_correct_tracks(self):
        points = []
        for center_x in (1.0, 20.0):
            points.extend(
                [
                    (center_x - 0.08, -0.08, 1.0, 10.0),
                    (center_x + 0.08, -0.08, 1.0, 10.0),
                    (center_x - 0.08, 0.08, 1.0, 10.0),
                    (center_x + 0.08, 0.08, 1.0, 10.0),
                ]
            )
        header = Header(stamp=rospy.Time.now(), frame_id="world")
        cloud = point_cloud2.create_cloud(header, self._empty_cloud(header.stamp).fields, points)
        self._points_pub.publish(cloud)
        corrected = self._wait_for(
            lambda msg: len(msg.tracks) == 2
            and all(track.n_clusters >= 2 for track in msg.tracks)
        )
        self.assertEqual({track.id for track in corrected.tracks}, self._stable_ids)

    def test_03_uncertain_tracks_are_deleted(self):
        future_stamp = rospy.Time.now() + rospy.Duration(30.0)
        self._points_pub.publish(self._empty_cloud(future_stamp))
        removed = self._wait_for(lambda msg: len(msg.tracks) == 0)
        self.assertEqual(removed.header.frame_id, "world")

    def test_04_empty_background_is_accepted_when_enabled(self):
        self._background_pub.publish(self._empty_cloud(rospy.Time.now()))
        rospy.sleep(0.2)
        master = rosgraph.Master(rospy.get_name())
        _, subscribers, _ = master.getSystemState()
        nodes = dict(subscribers).get(
            "/background_empty_test/empty_background_fixture", []
        )
        self.assertIn("/background_empty_test/tracker", nodes)

    def test_05_graph_is_isolated_and_background_is_disabled(self):
        master = rosgraph.Master(rospy.get_name())
        publishers, subscribers, _ = master.getSystemState()
        tracker_node = "/baseline_test/tracker"
        tracker_subscriptions = {
            topic for topic, nodes in subscribers if tracker_node in nodes
        }
        tracker_publications = {
            topic for topic, nodes in publishers if tracker_node in nodes
        }
        self.assertIn("/baseline_test/detections_fixture", tracker_subscriptions)
        self.assertIn("/baseline_test/points_world_fixture", tracker_subscriptions)
        self.assertNotIn(
            "/baseline_test/must_remain_unsubscribed", tracker_subscriptions
        )
        self.assertIn("/baseline_test/tracks", tracker_publications)
        forbidden = ("free_voxels", "rays_checked", "tclv_tracker", "os_cloud")
        for topic in tracker_subscriptions | tracker_publications:
            self.assertFalse(any(token in topic for token in forbidden), topic)

    def test_06_source_has_no_ouster_or_image_transport_dependency(self):
        package_path = rospkg.RosPack().get_path("lidar_tracker_mid360")
        checked = [
            "CMakeLists.txt",
            "package.xml",
            "include/lidar_tracker_mid360/LidarTracker.h",
            "include/lidar_tracker_mid360/point_types.h",
            "src/lidar_tracker.cpp",
            "launch/lidar_tracker.launch",
        ]
        forbidden = ("ouster_ros", "image_transport", "image_geometry", "cv_bridge")
        for relative_path in checked:
            with open(os.path.join(package_path, relative_path), encoding="utf-8") as source:
                contents = source.read()
            for token in forbidden:
                self.assertNotIn(token, contents, relative_path)


if __name__ == "__main__":
    rostest.rosrun(
        "lidar_tracker_mid360",
        "synthetic_tracker_contract",
        SyntheticTrackerContract,
    )
