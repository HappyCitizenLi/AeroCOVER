#!/usr/bin/env python3

import threading
import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray
from mid360_ray_msgs.msg import CheckedRayBundle, Ray, RayBundle, ScanIdentity
import rospy
import rostest
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String


class SourceModeContractTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._checked = {}
        self._diagnostics = {}
        self._mode_pub = rospy.Publisher(
            "/test/source_mode/mode", String, queue_size=1, latch=True
        )
        self._points_pub = rospy.Publisher(
            "/test/source_mode/points_raw", PointCloud2, queue_size=2
        )
        self._rays_pub = rospy.Publisher(
            "/test/source_mode/rays_raw", RayBundle, queue_size=2
        )
        self._identity_pub = rospy.Publisher(
            "/test/source_mode/scan_identity", ScanIdentity, queue_size=2
        )
        self._checked_sub = rospy.Subscriber(
            "/test/source_mode/rays_checked",
            CheckedRayBundle,
            self._checked_callback,
            queue_size=4,
        )
        self._diagnostics_sub = rospy.Subscriber(
            "/test/source_mode/diagnostics",
            DiagnosticArray,
            self._diagnostics_callback,
            queue_size=4,
        )
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if (
                self._mode_pub.get_num_connections() > 0
                and self._points_pub.get_num_connections() > 0
                and self._rays_pub.get_num_connections() > 0
                and self._identity_pub.get_num_connections() > 0
            ):
                return
            rospy.sleep(0.02)
        self.fail("preprocessor subscribers did not connect")

    def _checked_callback(self, message):
        with self._lock:
            self._checked[message.header.stamp.to_nsec()] = message

    def _diagnostics_callback(self, message):
        if not message.status:
            return
        with self._lock:
            self._diagnostics[message.header.stamp.to_nsec()] = message.status[0]

    def _publish_pair(self, scan_id):
        stamp = rospy.Time.now()
        header = Header(stamp=stamp, frame_id="sensor")
        header.seq = scan_id
        cloud = point_cloud2.create_cloud_xyz32(header, [(1.0, 0.0, 0.0)])
        bundle = RayBundle()
        bundle.header = header
        bundle.scan_id = scan_id
        bundle.pattern_start_index = scan_id
        bundle.min_range = 0.1
        bundle.max_range = 70.0
        ray = Ray()
        ray.dir_x = 1.0
        ray.dir_y = 0.0
        ray.dir_z = 0.0
        ray.range = 1.0
        ray.intensity = 10
        ray.offset_time_ns = 0
        ray.pattern_index = scan_id
        ray.return_status = Ray.VALID_RETURN
        ray.tag = 0
        ray.line = 0
        bundle.rays = [ray]
        identity = ScanIdentity()
        identity.header = header
        identity.scan_id = scan_id
        identity.pattern_start_index = scan_id
        identity.point_count = 1
        identity.ray_count = 1
        identity.point_source_stamp = stamp
        identity.ray_source_stamp = stamp
        key = stamp.to_nsec()
        self._points_pub.publish(cloud)
        self._rays_pub.publish(bundle)
        self._identity_pub.publish(identity)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            with self._lock:
                if key in self._checked and key in self._diagnostics:
                    return self._checked[key], self._diagnostics[key]
            rospy.sleep(0.03)
        self.fail("timed out waiting for checked bundle and diagnostics")

    @staticmethod
    def _values(status):
        return {item.key: item.value for item in status.values}

    def test_latched_source_mode_is_authoritative_and_fail_closed(self):
        self._mode_pub.publish(String(data="calibrated_fallback"))
        rospy.sleep(0.2)
        checked, diagnostic = self._publish_pair(1)
        values = self._values(diagnostic)
        self.assertEqual(checked.source_mode, "calibrated_fallback")
        self.assertEqual(values["source_mode"], "calibrated_fallback")
        self.assertEqual(values["exact_direction_available"], "false")

        self._mode_pub.publish(String(data="unsupported_exact_claim"))
        rospy.sleep(0.2)
        checked, diagnostic = self._publish_pair(2)
        values = self._values(diagnostic)
        self.assertEqual(checked.source_mode, "calibrated_fallback")
        self.assertEqual(values["exact_direction_available"], "false")

        self._mode_pub.publish(String(data="sim_exact"))
        rospy.sleep(0.2)
        checked, diagnostic = self._publish_pair(3)
        values = self._values(diagnostic)
        self.assertEqual(checked.source_mode, "calibrated_fallback")
        self.assertEqual(values["allow_dynamic_sim_exact"], "false")

        self._mode_pub.publish(String(data="hw_spherical_exact"))
        rospy.sleep(0.2)
        checked, diagnostic = self._publish_pair(4)
        values = self._values(diagnostic)
        self.assertEqual(checked.source_mode, "hw_spherical_exact")
        self.assertEqual(values["source_mode"], "hw_spherical_exact")
        self.assertEqual(values["exact_direction_available"], "true")

        # A dead capability publisher cannot leave exact mode latched forever.
        rospy.sleep(0.8)
        checked, diagnostic = self._publish_pair(5)
        values = self._values(diagnostic)
        self.assertEqual(checked.source_mode, "calibrated_fallback")
        self.assertEqual(values["exact_direction_available"], "false")
        self.assertEqual(values["source_mode_lease_revoked"], "true")

        # A delayed exact heartbeat cannot undo a revocation. A fresh probe
        # must first publish its fail-closed startup mode, then requalify.
        self._mode_pub.publish(String(data="hw_spherical_exact"))
        rospy.sleep(0.2)
        checked, _ = self._publish_pair(6)
        self.assertEqual(checked.source_mode, "calibrated_fallback")
        self._mode_pub.publish(String(data="calibrated_fallback"))
        rospy.sleep(0.2)
        self._mode_pub.publish(String(data="hw_spherical_exact"))
        rospy.sleep(0.2)
        checked, diagnostic = self._publish_pair(7)
        values = self._values(diagnostic)
        self.assertEqual(checked.source_mode, "hw_spherical_exact")
        self.assertEqual(values["source_mode_lease_revoked"], "false")


if __name__ == "__main__":
    rospy.init_node("preprocessor_source_mode_contract_test")
    rostest.rosrun(
        "mid360_ray_preprocessor",
        "preprocessor_source_mode_contract_test",
        SourceModeContractTest,
    )
