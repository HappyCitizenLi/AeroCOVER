#!/usr/bin/env python3
"""Move the independent Gazebo ray sensor rigidly with the observer."""

import rospy
from gazebo_msgs.msg import ModelState
from nav_msgs.msg import Odometry


RELATIVE_POSITION = (0.0, 0.0625, -0.009)
RELATIVE_QUATERNION = (0.5, 0.5, -0.5, 0.5)


def multiply(first, second):
    ax, ay, az, aw = first
    bx, by, bz, bw = second
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate(quaternion, vector):
    pure = (vector[0], vector[1], vector[2], 0.0)
    inverse = (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3])
    rotated = multiply(multiply(quaternion, pure), inverse)
    return rotated[:3]


class RangefinderFollower:
    def __init__(self):
        self.publisher = rospy.Publisher(
            "/gazebo/set_model_state", ModelState, queue_size=1)
        self.last_stamp = rospy.Time(0)
        self.subscriber = rospy.Subscriber(
            "/uav1/ground_truth", Odometry, self.callback, queue_size=1)

    def callback(self, message):
        if not self.last_stamp.is_zero() and \
                (message.header.stamp - self.last_stamp).to_sec() < 0.01:
            return
        self.last_stamp = message.header.stamp
        orientation = message.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        offset = rotate(quaternion, RELATIVE_POSITION)
        output = ModelState()
        output.model_name = "observer_rangefinder"
        output.reference_frame = "world"
        output.pose.position.x = message.pose.pose.position.x + offset[0]
        output.pose.position.y = message.pose.pose.position.y + offset[1]
        output.pose.position.z = message.pose.pose.position.z + offset[2]
        result = multiply(quaternion, RELATIVE_QUATERNION)
        output.pose.orientation.x = result[0]
        output.pose.orientation.y = result[1]
        output.pose.orientation.z = result[2]
        output.pose.orientation.w = result[3]
        self.publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node("observer_rangefinder_follower")
    RangefinderFollower()
    rospy.spin()
