#!/usr/bin/env python3

import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Point
from sensor_msgs.msg import Range
from vofod_mid360.srv import QueryVoxels, QueryVoxelsRequest


class NativeInitContract(unittest.TestCase):
    def test_range_seed_uses_transformed_measurement(self):
        topic = rospy.get_param("~range_topic")
        service = rospy.get_param("~query_service")
        publisher = rospy.Publisher(topic, Range, queue_size=2)
        rospy.wait_for_service(service, timeout=15.0)
        query = rospy.ServiceProxy(service, QueryVoxels)
        deadline = time.monotonic() + 10.0
        while publisher.get_num_connections() == 0 and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertGreater(publisher.get_num_connections(), 0)

        message = Range()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "test_garmin"
        message.radiation_type = Range.INFRARED
        message.min_range = 0.1
        message.max_range = 10.0
        message.range = 2.0
        publisher.publish(message)

        request = QueryVoxelsRequest()
        request.points = [Point(2.25, 0.25, 0.25)]
        while time.monotonic() < deadline:
            response = query(request)
            if response.update_counts and response.update_counts[0] > 0:
                break
            time.sleep(0.02)
        self.assertEqual(response.update_counts[0], 1)
        self.assertEqual(response.last_update_classes[0], 8)
        self.assertGreater(response.scores[0], -740.0)


if __name__ == "__main__":
    rospy.init_node("vofod_mid360_native_init_test")
    rostest.rosrun("vofod_mid360", "native_init_contract", NativeInitContract)
