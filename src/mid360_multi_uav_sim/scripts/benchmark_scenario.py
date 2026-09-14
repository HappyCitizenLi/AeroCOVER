#!/usr/bin/env python3
"""Run benchmark trajectories through the stock MRS X500 control chain."""

import json
import math
import os
import threading
import time

import rospy
import yaml
from geometry_msgs.msg import TransformStamped
from mrs_msgs.msg import ControlManagerDiagnostics
from mrs_msgs.msg import Reference
from mrs_msgs.srv import ReferenceStampedSrv, ReferenceStampedSrvRequest
from mrs_msgs.srv import TrajectoryReferenceSrv, TrajectoryReferenceSrvRequest
from mrs_msgs.srv import String as StringSrv
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger, TriggerResponse
import tf2_ros


FIELDS = ("x", "y", "z", "roll", "pitch", "yaw")
SPAWN_Z = 0.3
PREFLIGHT_REFERENCE_TOLERANCE_M = 0.1


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


def parse_waypoints(raw, context, duration):
    if not isinstance(raw, list) or len(raw) < 2:
        raise ValueError("{} must contain at least two waypoints".format(context))
    output = []
    previous = -1.0
    for index, item in enumerate(raw):
        if not isinstance(item, dict):
            raise ValueError("{}[{}] must be a mapping".format(context, index))
        stamp = number(item.get("t"), "{}[{}].t".format(context, index))
        if stamp <= previous or stamp < 0.0:
            raise ValueError("{} waypoint times must increase from zero".format(context))
        values = tuple(number(item.get(field, 0.0),
                              "{}[{}].{}".format(context, index, field))
                       for field in FIELDS)
        output.append((stamp, values))
        previous = stamp
    # A reference tail may extend past the recording end so a circling
    # observer does not acquire an artificial zero terminal velocity.
    if output[0][0] != 0.0 or output[-1][0] < duration:
        raise ValueError("{} must cover t=0 through duration".format(context))
    return output


def waypoint_tangent(waypoints, index, scale):
    if index == 0 or index == len(waypoints) - 1:
        return (0.0,) * len(waypoints[index][1])
    if waypoints[index][1] in (waypoints[index-1][1], waypoints[index+1][1]):
        return (0.0,) * len(waypoints[index][1])
    before_t, before = waypoints[index - 1]
    after_t, after = waypoints[index + 1]
    return tuple(scale * (second - first) / (after_t - before_t)
                 for first, second in zip(before, after))


def sample(waypoints, elapsed, profile="linear", tangent_scale=0.2):
    for index in range(1, len(waypoints)):
        if elapsed <= waypoints[index][0]:
            first_t, first = waypoints[index - 1]
            second_t, second = waypoints[index]
            duration = second_t - first_t
            alpha = max(0.0, min(1.0, (elapsed - first_t) / duration))
            if profile == "minimum_jerk":
                blend = 10.0 * alpha ** 3 - 15.0 * alpha ** 4 + 6.0 * alpha ** 5
                blend_rate = (30.0 * alpha ** 2 - 60.0 * alpha ** 3 +
                              30.0 * alpha ** 4) / duration
            elif profile == "linear":
                blend, blend_rate = alpha, 1.0 / duration
            elif profile == "continuous_cubic":
                first_rate = waypoint_tangent(
                    waypoints, index - 1, tangent_scale)
                second_rate = waypoint_tangent(
                    waypoints, index, tangent_scale)
                h00 = 2.0 * alpha ** 3 - 3.0 * alpha ** 2 + 1.0
                h10 = alpha ** 3 - 2.0 * alpha ** 2 + alpha
                h01 = -2.0 * alpha ** 3 + 3.0 * alpha ** 2
                h11 = alpha ** 3 - alpha ** 2
                dh00 = 6.0 * alpha ** 2 - 6.0 * alpha
                dh10 = 3.0 * alpha ** 2 - 4.0 * alpha + 1.0
                dh01 = -dh00
                dh11 = 3.0 * alpha ** 2 - 2.0 * alpha
                values = tuple(
                    h00 * a + h10 * duration * da +
                    h01 * b + h11 * duration * db
                    for a, da, b, db in zip(
                        first, first_rate, second, second_rate))
                rates = tuple(
                    (dh00 * a + dh10 * duration * da +
                     dh01 * b + dh11 * duration * db) / duration
                    for a, da, b, db in zip(
                        first, first_rate, second, second_rate))
                return values, rates
            else:
                raise ValueError("unsupported trajectory_profile: {}".format(profile))
            values = tuple(a + blend * (b - a) for a, b in zip(first, second))
            rates = tuple(blend_rate * (b - a) for a, b in zip(first, second))
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
        self.control_frame = str(raw.get("control_frame", "world_origin"))
        if self.control_frame not in ("world_origin", "ground_truth_origin"):
            raise ValueError("unsupported control_frame")
        self.seed = int(raw["seed"])
        self.duration = number(raw["duration_s"], "duration_s")
        self.score_start = number(raw["score_start_s"], "score_start_s")
        self.score_end = number(raw["score_end_s"], "score_end_s")
        self.start_delay = number(raw.get("start_delay_s", 2.0), "start_delay_s")
        self.wait_for_start = rospy.get_param("~wait_for_start", False)
        self.sensor_model = str(rospy.get_param("~sensor_model", "mid360"))
        if self.sensor_model not in ("mid360", "ouster", "paired"):
            raise ValueError("sensor_model must be mid360, ouster, or paired")
        self.sensor_noise = number(
            rospy.get_param("/mrs_mid360/noise_stddev", 0.03),
            "sensor_noise_stddev")
        if not isinstance(self.wait_for_start, bool):
            raise ValueError("wait_for_start must be boolean")
        self.profile = str(raw.get("trajectory_profile", "linear"))
        if self.profile not in ("linear", "minimum_jerk", "continuous_cubic"):
            raise ValueError(
                "trajectory_profile must be linear, minimum_jerk, or "
                "continuous_cubic")
        self.tangent_scale = number(
            raw.get("trajectory_tangent_scale", 0.2),
            "trajectory_tangent_scale")
        self.trajectory_execution = raw.get('trajectory_execution','references')
        if self.trajectory_execution not in ('references','mrs_trajectory'):
            raise ValueError('unsupported trajectory_execution')
        if not 0.0 < self.tangent_scale <= 1.0:
            raise ValueError("trajectory_tangent_scale must be in (0, 1]")
        if not 0.0 < self.score_start < self.score_end <= self.duration:
            raise ValueError("invalid scoring interval")

        self.entities = [{
            "logical": "uav1", "physical": "uav1", "motion_start": 0.0,
            "waypoints": parse_waypoints(
                raw["observer"]["waypoints"], "observer.waypoints", self.duration),
            "spawn_z": SPAWN_Z,
            "active": True, "preflight_active": True,
            "motion_announced": True,
        }]
        ids = {"uav1"}
        for index, target in enumerate(raw["targets"]):
            logical = str(target["id"])
            if logical in ids:
                raise ValueError("target ids must be unique and exclude uav1")
            ids.add(logical)
            motion_start = number(
                target.get("motion_start_s", 0.0), "target.motion_start_s")
            preflight_active = target.get("preflight_active", True)
            if not isinstance(preflight_active, bool):
                raise ValueError("target.preflight_active must be boolean")
            self.entities.append({
                "logical": logical,
                "physical": "uav{}".format(index + 2),
                "motion_start": motion_start,
                "waypoints": parse_waypoints(
                    target["waypoints"], "targets[{}].waypoints".format(index),
                    self.duration),
                "spawn_z": number(target.get("spawn_z", SPAWN_Z),
                                  "target.spawn_z"),
                "active": False, "preflight_active": preflight_active,
                "activation_requested": False,
                "motion_announced": False,
            })
        if len(self.entities) > 6:
            raise ValueError("benchmark.launch supports at most six MRS vehicles")
        if self.trajectory_execution == 'mrs_trajectory' and any(
                not e['preflight_active'] or e['motion_start'] != 0 for e in self.entities):
            raise ValueError('timed MRS trajectories require all vehicles active from the start')

        custom_events = []
        seen = set()
        for item in raw.get("events", []):
            event_id = str(item["id"])
            stamp = number(item["t"], "event.t")
            if event_id in seen or not 0.0 <= stamp <= self.duration:
                raise ValueError("invalid or duplicate benchmark event")
            seen.add(event_id)
            custom_events.append((stamp, event_id))
        self.events = sorted(custom_events + [
            (self.score_start, "scoring_start"),
            (self.score_end, "scoring_end"),
        ])
        if len({event_id for _, event_id in self.events}) != len(self.events):
            raise ValueError("custom events collide with scoring events")

        self.event_pub = rospy.Publisher(
            "/mid360_multi_uav_sim/scenario_events", String,
            queue_size=20, latch=True)
        self.truth_pubs = {
            entity["logical"]: rospy.Publisher(
                "/mid360_multi_uav_sim/ground_truth/{}/odom".format(
                    entity["logical"]), Odometry, queue_size=20)
            for entity in self.entities
        }
        self.truth = {}
        self.control = {}
        self.subscribers = [
            rospy.Subscriber("/{}/ground_truth".format(entity["physical"]),
                             Odometry, self.truth_callback,
                             callback_args=entity, queue_size=20)
            for entity in self.entities
        ] + [
            rospy.Subscriber(
                "/{}/control_manager/diagnostics".format(entity["physical"]),
                ControlManagerDiagnostics, self.control_callback,
                callback_args=entity, queue_size=5)
            for entity in self.entities
        ]
        self.references = {}
        self.reference_due = {}
        self.emitted = set()
        self.spawner = rospy.ServiceProxy("/mrs_drone_spawner/spawn", StringSrv)
        self.world_tf = tf2_ros.TransformBroadcaster()
        self.start_requested = threading.Event()
        self.ready_for_start = False
        self.start_service = rospy.Service(
            "~start", Trigger, self.start_callback)

    def start_callback(self, _request):
        if not self.wait_for_start:
            return TriggerResponse(
                success=False, message="manual start is disabled")
        if not self.ready_for_start:
            return TriggerResponse(
                success=False,
                message="scenario is not ready or has already started")
        self.start_requested.set()
        return TriggerResponse(success=True, message="scenario start accepted")

    def truth_callback(self, message, entity):
        self.truth[entity["physical"]] = message
        if rospy.is_shutdown():
            return
        try:
            if entity["physical"] == "uav1":
                transform = TransformStamped()
                transform.header = message.header
                transform.header.frame_id = "world"
                transform.child_frame_id = "uav1/fcu"
                transform.transform.translation.x = message.pose.pose.position.x
                transform.transform.translation.y = message.pose.pose.position.y
                transform.transform.translation.z = message.pose.pose.position.z
                transform.transform.rotation = message.pose.pose.orientation
                self.world_tf.sendTransform(transform)
            output = Odometry()
            output.header = message.header
            output.header.frame_id = "world"
            output.child_frame_id = "{}/base_link".format(entity["logical"])
            output.pose = message.pose
            output.twist = message.twist
            self.truth_pubs[entity["logical"]].publish(output)
        except rospy.ROSException:
            if not rospy.is_shutdown():
                raise

    def control_callback(self, message, entity):
        self.control[entity["physical"]] = message

    def event(self, event_id, stamp, **extra):
        payload = {"scenario": self.scenario_id, "seed": self.seed,
                   "event": event_id, "sim_time": stamp.to_sec()}
        payload.update(extra)
        self.event_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        rospy.loginfo("benchmark event: %s", payload)

    def spawn_vehicle(self, entity):
        first = entity["waypoints"][0][1]
        vehicle_id = int(entity["physical"][3:])
        # rolling_scene publishes 10 Hz scans but samples collision geometry at
        # the 250 Hz world physics cadence.
        observer_sensors = ""
        if vehicle_id == 1 and self.sensor_model in ("mid360", "paired"):
            observer_sensors += (" --enable-livox update_rate:=250 "
                                 "range:=40 noise:={:.9g}").format(
                                     self.sensor_noise)
        if vehicle_id == 1 and self.sensor_model in ("ouster", "paired"):
            # Gazebo 11 renders a 2*pi GPU lidar with three internal cameras.
            observer_sensors += (" --enable-ouster model:=OS1-128 "
                                 "use_gpu:=True horizontal_samples:=1024 "
                                 "update_rate:=10 noise:={:.9g}").format(
                                     self.sensor_noise)
        # Garmin closes the MRS flight-height loop only.  Detector input remains
        # the Mid-360 points/rays contract recorded by the benchmark runner.
        command = ("{} --x500 --enable-ground-truth --enable-rangefinder{} "
                   "--pos {:.9g} {:.9g} {:.9g} {:.9g}").format(
                       vehicle_id, observer_sensors, first[0], first[1],
                       entity["spawn_z"], first[5])
        response = self.spawner(command)
        if not response.success:
            raise RuntimeError("failed to spawn {}: {}".format(
                entity["physical"], response.message))

    def wait_for_mrs(self, entities):
        for entity in entities:
            physical = entity["physical"]
            rospy.wait_for_message("/{}/ground_truth".format(physical),
                                   Odometry, timeout=90.0)
            service = "/{}/control_manager/reference".format(physical)
            rospy.wait_for_service(service, timeout=120.0)
            self.references[physical] = rospy.ServiceProxy(
                service, ReferenceStampedSrv, persistent=True)

    @staticmethod
    def call_until_success(service, service_type, request=None, timeout=90.0):
        """Retry while PX4 API has advertised a service but is not ready yet."""
        rospy.wait_for_service(service, timeout=timeout)
        proxy = rospy.ServiceProxy(service, service_type)
        deadline = time.monotonic() + timeout
        last_message = "service did not become ready"
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                response = proxy() if request is None else proxy(request)
                if response.success:
                    return
                last_message = response.message
            except rospy.ServiceException as exception:
                last_message = str(exception)
            time.sleep(0.25)
        raise RuntimeError("{} failed: {}".format(service, last_message))

    def arm(self, entities):
        for entity in entities:
            service = "/{}/hw_api/arming".format(entity["physical"])
            self.call_until_success(service, SetBool, True)
            entity["active"] = True
        rospy.sleep(2.0)
        for entity in entities:
            self.offboard(entity)

    def offboard(self, entity):
        service = "/{}/hw_api/offboard".format(entity["physical"])
        self.call_until_success(service, Trigger)

    def reference(self, entity, values):
        request = ReferenceStampedSrvRequest()
        request.header.stamp = rospy.Time.now()
        request.header.frame_id = "{}/{}".format(entity["physical"], self.control_frame)
        request.reference.position.x = values[0]
        request.reference.position.y = values[1]
        request.reference.position.z = values[2]
        request.reference.heading = values[5]
        try:
            response = self.references[entity["physical"]](request)
            return response.success
        except rospy.ServiceException:
            self.references[entity["physical"]] = rospy.ServiceProxy(
                "/{}/control_manager/reference".format(entity["physical"]),
                ReferenceStampedSrv, persistent=True)
            return False

    def at_reference(self, entity, values,
                     tolerance=PREFLIGHT_REFERENCE_TOLERANCE_M):
        odometry = self.truth.get(entity["physical"])
        if odometry is None:
            return False
        position = odometry.pose.pose.position
        return math.sqrt((position.x - values[0]) ** 2 +
                         (position.y - values[1]) ** 2 +
                         (position.z - values[2]) ** 2) <= tolerance

    def reference_error(self, entity, values):
        odometry = self.truth.get(entity["physical"])
        if odometry is None:
            return float("inf")
        position = odometry.pose.pose.position
        return math.sqrt((position.x - values[0]) ** 2 +
                         (position.y - values[1]) ** 2 +
                         (position.z - values[2]) ** 2)

    def preflight(self):
        rospy.wait_for_service("/mrs_drone_spawner/spawn", timeout=60.0)
        for entity in self.entities:
            self.spawn_vehicle(entity)
        self.wait_for_mrs(self.entities)

        flight = [
            entity for entity in self.entities if entity["preflight_active"]
        ]
        self.arm(flight)
        if getattr(self,'trajectory_execution','references') == 'mrs_trajectory':
            for entity in flight:
                self.call_until_success('/{}/constraint_manager/set_constraints'.format(entity['physical']),
                                        StringSrv,'fast')

        deadline = time.monotonic() + 180.0
        rate = rospy.Rate(10.0)
        iteration = 0
        while not rospy.is_shutdown():
            ready = True
            errors = []
            for entity in flight:
                values = entity["waypoints"][0][1]
                accepted = self.reference(entity, values)
                error = self.reference_error(entity, values)
                errors.append("{}={:.3f}m".format(entity["physical"], error))
                diagnostics = self.control.get(entity['physical'])
                ready = (accepted and diagnostics is not None and diagnostics.output_enabled and
                         diagnostics.active_tracker == 'MpcTracker' and
                         error <= PREFLIGHT_REFERENCE_TOLERANCE_M and ready)
            if ready:
                break
            if time.monotonic() > deadline:
                raise RuntimeError("MRS preflight did not reach initial references: " +
                                   ", ".join(errors))
            if iteration % 50 == 0:
                rospy.logwarn("MRS preflight reference errors: %s",
                              ", ".join(errors))
            iteration += 1
            rate.sleep()

        hold_until = rospy.Time.now() + rospy.Duration(self.start_delay)
        while rospy.Time.now() < hold_until and not rospy.is_shutdown():
            for entity in flight:
                self.reference(entity, entity["waypoints"][0][1])
            rate.sleep()

        if not self.wait_for_start or rospy.is_shutdown():
            return
        self.ready_for_start = True
        service = rospy.resolve_name("~start")
        self.event("scenario_ready", rospy.Time.now(), start_service=service)
        rospy.logwarn(
            "Scenario ready and holding at the initial waypoints. Start with: "
            "rosservice call %s \"{}\"", service)
        try:
            while not self.start_requested.is_set() and not rospy.is_shutdown():
                for entity in flight:
                    self.reference(entity, entity["waypoints"][0][1])
                rate.sleep()
        finally:
            self.ready_for_start = False

    def activate_target(self, entity, now):
        if not entity["activation_requested"]:
            arming = "/{}/hw_api/arming".format(entity["physical"])
            self.call_until_success(arming, SetBool, True)
            rospy.sleep(2.0)
            self.offboard(entity)
            entity["activation_requested"] = True
            return False
        diagnostics = self.control.get(entity["physical"])
        if diagnostics is None or not diagnostics.output_enabled or \
                diagnostics.active_tracker != "MpcTracker":
            return False
        entity["active"] = True
        return True

    def run(self):
        while rospy.Time.now().is_zero() and not rospy.is_shutdown():
            time.sleep(0.02)
        self.preflight()
        if rospy.is_shutdown():
            return
        start = (self.start_timed_trajectories() if self.trajectory_execution == 'mrs_trajectory'
                 else rospy.Time.now())
        while rospy.Time.now() < start and not rospy.is_shutdown():
            rospy.sleep(.01)
        self.event("benchmark_start", start, score_start_s=self.score_start,
                   score_end_s=self.score_end, flight_stack="MRS_X500_PX4",
                   trajectory_execution=self.trajectory_execution,
                   trajectory_start_skew_s=getattr(self,'trajectory_start_skew_s',0.))
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            elapsed = max(0.0, (now - start).to_sec())
            if self.trajectory_execution == 'mrs_trajectory':
                for entity in self.entities:
                    diagnostics=self.control.get(entity['physical'])
                    if diagnostics is None or not diagnostics.output_enabled or diagnostics.active_tracker!='MpcTracker':
                        raise RuntimeError('timed trajectory lost active MpcTracker: '+entity['physical'])
            for entity in self.entities:
                if entity is not self.entities[0] and \
                        not entity["motion_announced"] and \
                        elapsed >= entity["motion_start"]:
                    if not entity["active"]:
                        if not self.activate_target(entity, now):
                            continue
                    entity["motion_announced"] = True
                    self.event(
                        "target_motion_started", now,
                        target_id=entity["logical"],
                        physical_vehicle=entity["physical"])
                due = self.reference_due.get(entity["physical"])
                if due is not None and now >= due:
                    del self.reference_due[entity["physical"]]
                if entity["active"] and due is None and self.trajectory_execution == 'references':
                    values, _ = sample(entity["waypoints"],
                                       min(elapsed, self.duration), self.profile,
                                       self.tangent_scale)
                    if self.reference(entity, values):
                        entity["reference_failures"] = 0
                    else:
                        entity["reference_failures"] = entity.get(
                            "reference_failures", 0) + 1
                        if entity["reference_failures"] >= 20:
                            raise RuntimeError(
                                "{} rejected 20 consecutive references".format(
                                    entity["physical"]))

            for event_time, event_id in self.events:
                if event_id not in self.emitted and elapsed >= event_time:
                    self.emitted.add(event_id)
                    self.event(event_id, now, elapsed_s=elapsed)
            if elapsed >= self.duration:
                self.event("scenario_complete", now)
                rospy.sleep(0.2)
                return
            rate.sleep()

    def start_timed_trajectories(self):
        requests=[]
        dt=.05
        for entity in self.entities:
            request=TrajectoryReferenceSrvRequest()
            trajectory=request.trajectory
            trajectory.header.frame_id=entity['physical']+'/'+self.control_frame
            trajectory.input_id=self.seed
            trajectory.dt=dt
            trajectory.use_heading=True
            trajectory.fly_now=False
            trajectory.loop=False
            for i in range(int(math.ceil(self.duration/dt))+2):
                values,_=sample(entity['waypoints'],min(i*dt,self.duration),self.profile,self.tangent_scale)
                reference=Reference()
                reference.position.x,reference.position.y,reference.position.z=values[:3]
                reference.heading=values[5]
                trajectory.points.append(reference)
            service='/'+entity['physical']+'/control_manager/trajectory_reference'
            rospy.wait_for_service(service,timeout=10.)
            requests.append((entity['physical'],rospy.ServiceProxy(service,TrajectoryReferenceSrv),request))
        for physical,service,request in requests:
            request.trajectory.header.stamp=rospy.Time(0)
            response=service(request)
            if not response.success or response.modified:
                raise RuntimeError('MRS trajectory rejected/modified: '+response.message)
        starters=[]
        for physical,_,_ in requests:
            service='/'+physical+'/control_manager/start_trajectory_tracking'
            rospy.wait_for_service(service,timeout=10.)
            starters.append(rospy.ServiceProxy(service,Trigger))
        start=rospy.Time.now()
        for service in starters:
            response=service()
            if not response.success:
                raise RuntimeError('MRS trajectory start failed: '+response.message)
        self.trajectory_start_skew_s=(rospy.Time.now()-start).to_sec()
        if self.trajectory_start_skew_s>.1:
            raise RuntimeError('trajectory start service skew exceeds 0.1 s')
        return start


if __name__ == "__main__":
    rospy.init_node("benchmark_scenario")
    try:
        BenchmarkScenario().run()
    except Exception as exception:
        rospy.logfatal("benchmark scenario failed: %s", exception)
        raise
