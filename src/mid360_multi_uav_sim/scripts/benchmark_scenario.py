#!/usr/bin/env python3
"""Generic target-free-warmup authority for SOFT-VoFOD benchmarks."""

import json
import math
import os
import time

import rospy
import yaml
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SpawnModel
from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import tf2_ros


FIELDS = ("x", "y", "z", "roll", "pitch", "yaw")


def number(value, context):
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("{} must be finite numeric".format(context))
    return float(value)


def quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rotation_transpose(vector, roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    matrix = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )
    return tuple(sum(matrix[row][column] * vector[row] for row in range(3))
                 for column in range(3))


def pose_message(values):
    output = Pose()
    output.position.x, output.position.y, output.position.z = values[:3]
    q = quaternion(*values[3:])
    output.orientation.x, output.orientation.y = q[0], q[1]
    output.orientation.z, output.orientation.w = q[2], q[3]
    return output


def parse_waypoints(raw, context, duration):
    if not isinstance(raw, list) or len(raw) < 2:
        raise ValueError("{} must contain at least two waypoints".format(context))
    output = []
    previous = -1.0
    for index, item in enumerate(raw):
        if not isinstance(item, dict):
            raise ValueError("{}[{}] must be a mapping".format(context, index))
        stamp = number(item.get("t"), "{}[{}].t".format(context, index))
        if stamp <= previous or stamp < 0.0 or stamp > duration:
            raise ValueError("{} waypoint times must increase inside duration".format(context))
        values = tuple(number(item.get(field, 0.0),
                              "{}[{}].{}".format(context, index, field))
                       for field in FIELDS)
        output.append((stamp, values))
        previous = stamp
    if output[0][0] != 0.0 or output[-1][0] != duration:
        raise ValueError("{} must cover t=0 through duration".format(context))
    return output


def sample(waypoints, elapsed):
    for index in range(1, len(waypoints)):
        if elapsed <= waypoints[index][0]:
            first_t, first = waypoints[index - 1]
            second_t, second = waypoints[index]
            alpha = max(0.0, min(1.0, (elapsed - first_t) / (second_t - first_t)))
            values = tuple(a + alpha * (b - a) for a, b in zip(first, second))
            rates = tuple((b - a) / (second_t - first_t)
                          for a, b in zip(first, second))
            return values, rates
    return waypoints[-1][1], (0.0,) * 6


class BenchmarkScenario:
    def __init__(self):
        path = os.path.abspath(rospy.get_param("~scenario_file"))
        with open(path, "r", encoding="utf-8") as stream:
            raw = yaml.safe_load(stream)
        if not isinstance(raw, dict) or raw.get("schema_version") != 1:
            raise ValueError("benchmark scenario schema_version must be 1")
        self.scenario_id = str(raw["scenario_id"])
        self.seed = int(raw["seed"])
        self.duration = number(raw["duration_s"], "duration_s")
        self.score_start = number(raw["score_start_s"], "score_start_s")
        self.score_end = number(raw["score_end_s"], "score_end_s")
        self.cold_start = raw.get("cold_start", False)
        if not isinstance(self.cold_start, bool):
            raise ValueError("cold_start must be boolean")
        if not 0.0 < self.score_start < self.score_end <= self.duration:
            raise ValueError("invalid scoring interval")
        self.start_delay = number(raw.get("start_delay_s", 2.0), "start_delay_s")
        self.observer = parse_waypoints(
            raw["observer"]["waypoints"], "observer.waypoints", self.duration)
        self.targets = []
        ids = set()
        for index, target in enumerate(raw["targets"]):
            target_id = str(target["id"])
            if target_id == "uav1" or target_id in ids:
                raise ValueError("target ids must be unique and exclude uav1")
            ids.add(target_id)
            spawn_time = number(target["spawn_time_s"], "target.spawn_time_s")
            if spawn_time < self.score_start and not self.cold_start:
                raise ValueError("targets may not spawn during target-free warmup")
            waypoints = parse_waypoints(
                target["waypoints"], "targets[{}].waypoints".format(index),
                self.duration)
            self.targets.append({"id": target_id, "spawn": spawn_time,
                                 "waypoints": waypoints, "spawned": False})
        events = raw.get("events", [])
        self.events = []
        event_ids = set()
        for item in events:
            event_id = str(item["id"])
            stamp = number(item["t"], "event.t")
            if event_id in event_ids or not 0.0 <= stamp <= self.duration:
                raise ValueError("invalid or duplicate benchmark event")
            event_ids.add(event_id)
            self.events.append((stamp, event_id))
        self.events += [(self.score_start, "scoring_start"),
                        (self.score_end, "scoring_end")]
        self.events.sort()
        if len({event_id for _, event_id in self.events}) != len(self.events):
            raise ValueError("custom events collide with scoring events")

        self.target_xml = open(
            rospy.get_param("~target_model_file"), "r", encoding="utf-8"
        ).read()
        self.set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.event_pub = rospy.Publisher(
            "/mid360_multi_uav_sim/scenario_events", String,
            queue_size=20, latch=True)
        self.truth_pubs = {
            "uav1": rospy.Publisher(
                "/mid360_multi_uav_sim/ground_truth/uav1/odom",
                Odometry, queue_size=20)
        }
        for target in self.targets:
            self.truth_pubs[target["id"]] = rospy.Publisher(
                "/mid360_multi_uav_sim/ground_truth/{}/odom".format(target["id"]),
                Odometry, queue_size=20)
        self.tf = tf2_ros.TransformBroadcaster()
        self.emitted = set()

    def event(self, event_id, stamp, **extra):
        payload = {"scenario": self.scenario_id, "seed": self.seed,
                   "event": event_id, "sim_time": stamp.to_sec()}
        payload.update(extra)
        self.event_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        rospy.loginfo("benchmark event: %s", payload)

    def publish_truth(self, entity_id, stamp, values, rates):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "world"
        message.child_frame_id = "{}/fcu".format(entity_id) if entity_id == "uav1" \
            else "{}/base_link".format(entity_id)
        message.pose.pose = pose_message(values)
        body_velocity = rotation_transpose(rates[:3], *values[3:])
        message.twist.twist.linear.x, message.twist.twist.linear.y = body_velocity[:2]
        message.twist.twist.linear.z = body_velocity[2]
        message.twist.twist.angular.x = rates[3]
        message.twist.twist.angular.y = rates[4]
        message.twist.twist.angular.z = rates[5]
        self.truth_pubs[entity_id].publish(message)

    def command(self, entity_id, values, rates):
        state = ModelState()
        state.model_name = entity_id
        state.reference_frame = "world"
        state.pose = pose_message(values)
        state.twist.linear.x, state.twist.linear.y, state.twist.linear.z = rates[:3]
        state.twist.angular.x, state.twist.angular.y, state.twist.angular.z = rates[3:]
        return self.set_state(state).success

    def spawn(self, target, values):
        response = self.spawn_model(
            target["id"], self.target_xml, "", pose_message(values), "world")
        if not response.success:
            raise RuntimeError("failed to spawn {}: {}".format(
                target["id"], response.status_message))
        target["spawned"] = True

    def run(self):
        rospy.wait_for_service("/gazebo/set_model_state", timeout=30.0)
        rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=30.0)
        while rospy.Time.now().is_zero() and not rospy.is_shutdown():
            time.sleep(0.02)
        start = rospy.Time.now() + rospy.Duration(self.start_delay)
        rate = rospy.Rate(50.0)
        while rospy.Time.now() < start and not rospy.is_shutdown():
            values, rates = sample(self.observer, 0.0)
            self.command("uav1", values, rates)
            self.publish_truth("uav1", rospy.Time.now(), values, rates)
            rate.sleep()
        self.event("benchmark_start", start,
                   score_start_s=self.score_start, score_end_s=self.score_end)

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            elapsed = max(0.0, (now - start).to_sec())
            observer, observer_rates = sample(self.observer, min(elapsed, self.duration))
            if not self.command("uav1", observer, observer_rates):
                raise RuntimeError("observer model is not available")
            self.publish_truth("uav1", now, observer, observer_rates)
            transform = TransformStamped()
            transform.header.stamp = now
            transform.header.frame_id = "world"
            transform.child_frame_id = "uav1/fcu"
            transform.transform.translation.x = observer[0]
            transform.transform.translation.y = observer[1]
            transform.transform.translation.z = observer[2]
            q = quaternion(*observer[3:])
            transform.transform.rotation.x, transform.transform.rotation.y = q[:2]
            transform.transform.rotation.z, transform.transform.rotation.w = q[2:]
            self.tf.sendTransform(transform)

            for target in self.targets:
                values, rates = sample(target["waypoints"], min(elapsed, self.duration))
                self.publish_truth(target["id"], now, values, rates)
                if not target["spawned"] and elapsed >= target["spawn"]:
                    self.spawn(target, values)
                    self.event("target_spawned", now, target_id=target["id"])
                if target["spawned"] and not self.command(target["id"], values, rates):
                    raise RuntimeError("target {} disappeared".format(target["id"]))

            for event_time, event_id in self.events:
                if event_id not in self.emitted and elapsed >= event_time:
                    self.emitted.add(event_id)
                    self.event(event_id, now, elapsed_s=elapsed)
            if elapsed >= self.duration:
                self.event("scenario_complete", now)
                rospy.sleep(0.2)
                return
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("benchmark_scenario")
    try:
        BenchmarkScenario().run()
    except Exception as exception:
        rospy.logfatal("benchmark scenario failed: %s", exception)
        raise
