#!/usr/bin/env python3
"""Live S04 translate+yaw contract over real Gazebo and ROS traffic.

``/gazebo/model_states`` is consumed only by this test as an independent
oracle.  The wall and pillar checks below classify endpoints solely by their
frozen collision geometry; they deliberately make no semantic-label claim.
"""

import json
import math
import threading
import time
import unittest

import rosgraph
import rospy
import rostest
import tf2_ros
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from gazebo_msgs.msg import ModelStates
from mid360_ray_msgs.msg import CheckedRayBundle, Ray, RayBundle
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


SCENARIO_ID = "S04_combined_visibility_translate_yaw"
SEMANTIC_CONTRACT_ID = "mid360-multi-uav-s04-combined-visibility-v1"


def _frame(value):
    return str(value).lstrip("/")


def _stamp_ns(header):
    return int(header.stamp.to_nsec())


def _bundle_key(header):
    return (int(header.seq), _stamp_ns(header))


def _norm(values):
    return math.sqrt(sum(value * value for value in values))


def _yaw(quaternion):
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def _angle_error(first, second):
    return math.atan2(math.sin(first - second), math.cos(first - second))


def _rotate(quaternion, vector):
    """Rotate a vector without requiring the optional transformations module."""
    qx, qy, qz, qw = (
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w,
    )
    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


class GateS04Contract(unittest.TestCase):

    def setUp(self):
        self.timeout_wall_sec = float(rospy.get_param("~timeout_wall_sec", 175.0))
        self.post_complete_timeout_wall_sec = float(
            rospy.get_param("~post_complete_timeout_wall_sec", 12.0)
        )
        self.expected_ray_count = int(rospy.get_param("~expected_ray_count", 20000))
        self.maximum_tf_gap_sec = float(
            rospy.get_param("~maximum_tf_gap_sec", 0.20)
        )
        self.minimum_scored_translation_m = float(
            rospy.get_param("~minimum_scored_translation_m", 1.90)
        )
        self.minimum_scored_yaw_rad = float(
            rospy.get_param("~minimum_scored_yaw_rad", 0.35)
        )
        self.maximum_geometry_scans = int(
            rospy.get_param("~maximum_geometry_scans", 8)
        )
        self.minimum_geometry_scans = int(
            rospy.get_param("~minimum_geometry_scans", 3)
        )
        self.minimum_wall_hit_count = int(
            rospy.get_param("~minimum_wall_hit_count", 5)
        )
        self.minimum_pillar_hit_count = int(
            rospy.get_param("~minimum_pillar_hit_count", 5)
        )
        self.geometry_tolerance_m = float(
            rospy.get_param("~geometry_tolerance_m", 0.04)
        )
        if (
            self.timeout_wall_sec <= 0.0
            or self.post_complete_timeout_wall_sec <= 0.0
            or self.expected_ray_count != 20000
            or self.maximum_tf_gap_sec <= 0.0
            or self.maximum_tf_gap_sec > 0.20
            or self.minimum_scored_translation_m < 1.90
            or self.minimum_scored_yaw_rad < 0.35
            or self.minimum_geometry_scans < 3
            or self.maximum_geometry_scans < self.minimum_geometry_scans
            or self.minimum_wall_hit_count < 5
            or self.minimum_pillar_hit_count < 5
            or self.geometry_tolerance_m <= 0.0
            or self.geometry_tolerance_m > 0.04
        ):
            raise ValueError("S04 live-gate parameters weaken the frozen contract")

        self.lock = threading.RLock()
        self.failure = None
        self.complete_event = threading.Event()
        self.complete_sim_time = None
        self.cycle_start_sim_time = None
        self.combined_start_sim_time = None
        self.combined_end_sim_time = None
        self.events = []

        self.model_samples = {
            name: [] for name in ("uav1", "uav2", "uav3", "uav4")
        }
        self.raw_summaries = {}
        self.source_diagnostics = {}
        self.preprocessor_diagnostics = {}
        self.best_scored_checked_bundle = None
        self.best_scored_checked_motion = -math.inf
        self.scored_checked_count = 0
        self.geometry_summaries = []
        self.post_complete_checked_stamps = []
        self.post_cycle_source_ok_count = 0
        self.post_cycle_preprocessor_ok_count = 0

        self.dynamic_tf_authorities = set()
        self.dynamic_tf_samples = []
        self.static_world_fcu_authorities = set()
        self.static_sensor_authorities = set()

        self.subscribers = [
            rospy.Subscriber(
                "/gazebo/model_states", ModelStates, self._model_callback,
                queue_size=20,
            ),
            rospy.Subscriber(
                "/uav1/mid360/rays_raw", RayBundle, self._raw_callback,
                queue_size=3,
            ),
            rospy.Subscriber(
                "/uav1/mid360/rays_checked", CheckedRayBundle,
                self._checked_callback, queue_size=3,
            ),
            rospy.Subscriber(
                "/uav1/mid360/ray_source_diagnostics", DiagnosticArray,
                self._source_diagnostics_callback, queue_size=10,
            ),
            rospy.Subscriber(
                "/uav1/mid360/ray_diagnostics", DiagnosticArray,
                self._preprocessor_diagnostics_callback, queue_size=10,
            ),
            rospy.Subscriber(
                "/mid360_multi_uav_sim/scenario_events", String,
                self._event_callback, queue_size=100,
            ),
            rospy.Subscriber("/tf", TFMessage, self._tf_callback, queue_size=1000),
            rospy.Subscriber(
                "/tf_static", TFMessage, self._tf_static_callback, queue_size=100
            ),
        ]

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(40.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def _set_failure(self, error):
        with self.lock:
            if self.failure is None:
                self.failure = str(error)
                rospy.logerr("S04 contract callback failed: %s", self.failure)
        self.complete_event.set()

    @staticmethod
    def _diagnostic_values(message):
        values = {}
        for status in message.status:
            for item in status.values:
                values[item.key] = item.value
        return values

    @staticmethod
    def _caller_id(message):
        connection = getattr(message, "_connection_header", None) or {}
        return connection.get("callerid", "<missing-callerid>")

    def _model_callback(self, message):
        try:
            if (
                len(message.name) != len(message.pose)
                or len(message.name) != len(message.twist)
            ):
                raise ValueError("ModelStates name/pose/twist arrays are not aligned")
            now = rospy.Time.now().to_sec()
            samples = {}
            for index, name in enumerate(message.name):
                if name not in self.model_samples:
                    continue
                pose = message.pose[index]
                twist = message.twist[index]
                samples[name] = (
                    now,
                    float(pose.position.x),
                    float(pose.position.y),
                    float(pose.position.z),
                    _yaw(pose.orientation),
                    _norm((twist.linear.x, twist.linear.y, twist.linear.z)),
                    _norm((twist.angular.x, twist.angular.y, twist.angular.z)),
                )
            with self.lock:
                for name, sample in samples.items():
                    self.model_samples[name].append(sample)
        except Exception as error:
            self._set_failure(error)

    def _raw_callback(self, message):
        try:
            count = len(message.rays)
            if count != self.expected_ray_count:
                raise ValueError(
                    "raw RayBundle has {} rays, expected {}".format(
                        count, self.expected_ray_count
                    )
                )
            if _stamp_ns(message.header) <= 0:
                raise ValueError("raw RayBundle has a zero timestamp")
            if message.header.frame_id != "uav1/mid360_link":
                raise ValueError("raw RayBundle frame changed")
            if int(message.header.seq) != int(message.scan_id):
                raise ValueError("raw RayBundle seq/scan_id identity changed")
            if int(message.rays[0].offset_time_ns) != 0:
                raise ValueError("raw RayBundle first ray offset is not zero")
            monotonic = all(
                message.rays[index].offset_time_ns
                >= message.rays[index - 1].offset_time_ns
                for index in range(1, count)
            )
            indices = (0, count // 2, count - 1)
            summary = {
                "scan_id": int(message.scan_id),
                "pattern_start_index": int(message.pattern_start_index),
                "frame_id": str(message.header.frame_id),
                "count": count,
                "min_range": float(message.min_range),
                "max_range": float(message.max_range),
                "duration_ns": int(message.rays[-1].offset_time_ns),
                "monotonic": monotonic,
                "signatures": tuple(
                    (
                        int(message.rays[index].pattern_index),
                        int(message.rays[index].offset_time_ns),
                        int(message.rays[index].return_status),
                        float(message.rays[index].range),
                    )
                    for index in indices
                ),
            }
            key = _bundle_key(message.header)
            with self.lock:
                if key in self.raw_summaries:
                    raise ValueError("duplicate raw RayBundle identity")
                self.raw_summaries[key] = summary
        except Exception as error:
            self._set_failure(error)

    def _summarize_geometry(self, message):
        """Count only endpoints unambiguously on frozen E2 collision surfaces."""
        wall_hits = 0
        pillar_hits = 0
        wall_errors = []
        pillar_errors = []
        for checked in message.rays:
            source = checked.source
            if source.return_status != Ray.VALID_RETURN:
                continue
            endpoint_x = checked.origin.x + source.range * checked.direction.x
            endpoint_y = checked.origin.y + source.range * checked.direction.y
            endpoint_z = checked.origin.z + source.range * checked.direction.z

            # background_wall is [9.75, 10.25] x [-0.4, 0.4] x [0, 6].
            # From every scored observer pose its first reachable x face is 9.75.
            wall_error = abs(endpoint_x - 9.75)
            if (
                wall_error <= self.geometry_tolerance_m
                and -0.36 <= endpoint_y <= 0.36
                and 0.10 <= endpoint_z <= 5.90
            ):
                wall_hits += 1
                wall_errors.append(wall_error)

            # pillar is a radius-0.60 cylinder centred at (7, -6), z in [0, 4].
            pillar_error = abs(math.hypot(endpoint_x - 7.0, endpoint_y + 6.0) - 0.60)
            if (
                pillar_error <= self.geometry_tolerance_m
                and 0.10 <= endpoint_z <= 3.90
            ):
                pillar_hits += 1
                pillar_errors.append(pillar_error)
        return {
            "scan_id": int(message.scan_id),
            "wall_hits": wall_hits,
            "pillar_hits": pillar_hits,
            "maximum_wall_error_m": max(wall_errors) if wall_errors else None,
            "maximum_pillar_error_m": max(pillar_errors) if pillar_errors else None,
        }

    def _checked_callback(self, message):
        try:
            if len(message.rays) != self.expected_ray_count:
                raise ValueError(
                    "checked bundle has {} rays, expected {}".format(
                        len(message.rays), self.expected_ray_count
                    )
                )
            if message.header.frame_id != "world":
                raise ValueError("checked bundle output frame changed")
            if message.source_header.frame_id != "uav1/mid360_link":
                raise ValueError("checked bundle source frame changed")
            if message.header.stamp != message.source_header.stamp:
                raise ValueError("checked/source timestamps differ")
            if int(message.header.seq) != int(message.source_header.seq):
                raise ValueError("checked/source sequences differ")
            if message.source_mode != "sim_exact":
                raise ValueError("checked bundle source_mode changed")
            if message.ray_time_geometry_mode != "per_ray_pose":
                raise ValueError("checked bundle is not per_ray_pose")

            stamp_sec = message.source_header.stamp.to_sec()
            duration_sec = message.rays[-1].source.offset_time_ns * 1.0e-9
            origin_delta = _norm(
                (
                    message.rays[-1].origin.x - message.rays[0].origin.x,
                    message.rays[-1].origin.y - message.rays[0].origin.y,
                    message.rays[-1].origin.z - message.rays[0].origin.z,
                )
            )
            geometry_summary = None
            with self.lock:
                start = self.combined_start_sim_time
                end = self.combined_end_sim_time
                geometry_slots = len(self.geometry_summaries)
                complete_time = self.complete_sim_time
            # Keep the complete scan away from phase boundaries.  This prevents
            # a boundary bundle from masquerading as simultaneous translate+yaw.
            is_scored = (
                start is not None
                and stamp_sec >= start + 0.25
                and stamp_sec + duration_sec <= start + 4.75
                and (end is None or stamp_sec + duration_sec < end - 0.25)
            )
            if is_scored and geometry_slots < self.maximum_geometry_scans:
                geometry_summary = self._summarize_geometry(message)

            with self.lock:
                if is_scored:
                    self.scored_checked_count += 1
                    if origin_delta > self.best_scored_checked_motion:
                        self.best_scored_checked_motion = origin_delta
                        self.best_scored_checked_bundle = message
                    if (
                        geometry_summary is not None
                        and len(self.geometry_summaries) < self.maximum_geometry_scans
                    ):
                        self.geometry_summaries.append(geometry_summary)
                if complete_time is not None and stamp_sec > complete_time + 1.0e-3:
                    self.post_complete_checked_stamps.append(stamp_sec)
        except Exception as error:
            self._set_failure(error)

    def _source_diagnostics_callback(self, message):
        try:
            summary = {
                "level": max(
                    (int(status.level) for status in message.status), default=3
                ),
                "messages": tuple(status.message for status in message.status),
                "values": self._diagnostic_values(message),
            }
            stamp_sec = message.header.stamp.to_sec()
            with self.lock:
                self.source_diagnostics[_bundle_key(message.header)] = summary
                cycle_start = self.cycle_start_sim_time
            if cycle_start is not None and stamp_sec >= cycle_start:
                if summary["level"] != DiagnosticStatus.OK:
                    raise ValueError(
                        "post-cycle source diagnostic is not OK: {}".format(summary)
                    )
                with self.lock:
                    self.post_cycle_source_ok_count += 1
        except Exception as error:
            self._set_failure(error)

    def _preprocessor_diagnostics_callback(self, message):
        try:
            summary = {
                "level": max(
                    (int(status.level) for status in message.status), default=3
                ),
                "messages": tuple(status.message for status in message.status),
                "values": self._diagnostic_values(message),
            }
            stamp_sec = message.header.stamp.to_sec()
            with self.lock:
                self.preprocessor_diagnostics[_bundle_key(message.header)] = summary
                cycle_start = self.cycle_start_sim_time
            if cycle_start is not None and stamp_sec >= cycle_start:
                values = summary["values"]
                if summary["level"] >= DiagnosticStatus.ERROR:
                    raise ValueError(
                        "post-cycle preprocessor diagnostic is ERROR: {}".format(
                            summary
                        )
                    )
                if values.get("tf_missing_bundle_ratio") != "0":
                    raise ValueError(
                        "post-cycle preprocessor reported a missing/extrapolated TF"
                    )
                # An observed-edge WARN can mean only that the auxiliary /tf
                # timestamp cache had not yet seen the right sample.  It is
                # not a tf2 lookup/extrapolation failure.  Count only fully
                # bracketed OK samples here and require abundant evidence
                # below; the selected scored scan is checked exhaustively.
                if (
                    summary["level"] == DiagnosticStatus.OK
                    and values.get("tf_source_sample_bracketed") == "true"
                ):
                    with self.lock:
                        self.post_cycle_preprocessor_ok_count += 1
        except Exception as error:
            self._set_failure(error)

    def _event_callback(self, message):
        try:
            event = json.loads(message.data)
            required = (
                "scenario", "scenario_profile", "semantic_contract_id",
                "event", "cycle", "seed", "repeat_mode", "sim_time",
                "sim_time_ns", "commanded_pose", "dynamic_tf_authority",
            )
            missing = [key for key in required if key not in event]
            if missing:
                raise ValueError(
                    "scenario event lacks required fields {}".format(missing)
                )
            with self.lock:
                self.events.append(event)
                if event["event"] == "cycle_start":
                    self.cycle_start_sim_time = float(event["sim_time"])
                elif event["event"] == "combined_visibility_start":
                    self.combined_start_sim_time = float(event["sim_time"])
                elif event["event"] == "combined_visibility_end":
                    self.combined_end_sim_time = float(event["sim_time"])
                elif event["event"] == "scenario_complete":
                    self.complete_sim_time = float(event["sim_time"])
                    self.complete_event.set()
        except Exception as error:
            self._set_failure(error)

    def _tf_callback(self, message):
        try:
            authority = self._caller_id(message)
            matches = []
            for transform in message.transforms:
                if (
                    _frame(transform.header.frame_id) == "world"
                    and _frame(transform.child_frame_id) == "uav1/fcu"
                ):
                    matches.append(
                        (
                            transform.header.stamp.to_sec(),
                            float(transform.transform.translation.x),
                            float(transform.transform.translation.y),
                            float(transform.transform.translation.z),
                            _yaw(transform.transform.rotation),
                        )
                    )
            if not matches:
                return
            with self.lock:
                self.dynamic_tf_authorities.add(authority)
                self.dynamic_tf_samples.extend(matches)
        except Exception as error:
            self._set_failure(error)

    def _tf_static_callback(self, message):
        try:
            authority = self._caller_id(message)
            world_fcu = False
            sensor = False
            for transform in message.transforms:
                parent = _frame(transform.header.frame_id)
                child = _frame(transform.child_frame_id)
                world_fcu = world_fcu or (
                    parent == "world" and child == "uav1/fcu"
                )
                sensor = sensor or (
                    parent == "uav1/fcu" and child == "uav1/mid360_link"
                )
            with self.lock:
                if world_fcu:
                    self.static_world_fcu_authorities.add(authority)
                if sensor:
                    self.static_sensor_authorities.add(authority)
        except Exception as error:
            self._set_failure(error)

    def _snapshot(self):
        with self.lock:
            return {
                "failure": self.failure,
                "complete_sim_time": self.complete_sim_time,
                "events": list(self.events),
                "model_samples": {
                    name: list(samples)
                    for name, samples in self.model_samples.items()
                },
                "raw_summaries": dict(self.raw_summaries),
                "source_diagnostics": dict(self.source_diagnostics),
                "preprocessor_diagnostics": dict(self.preprocessor_diagnostics),
                "best_checked": self.best_scored_checked_bundle,
                "best_checked_motion": self.best_scored_checked_motion,
                "scored_checked_count": self.scored_checked_count,
                "geometry_summaries": list(self.geometry_summaries),
                "post_complete_checked_stamps": list(
                    self.post_complete_checked_stamps
                ),
                "post_cycle_source_ok_count": self.post_cycle_source_ok_count,
                "post_cycle_preprocessor_ok_count": (
                    self.post_cycle_preprocessor_ok_count
                ),
                "dynamic_tf_authorities": set(self.dynamic_tf_authorities),
                "dynamic_tf_samples": list(self.dynamic_tf_samples),
                "static_world_fcu_authorities": set(
                    self.static_world_fcu_authorities
                ),
                "static_sensor_authorities": set(self.static_sensor_authorities),
            }

    def _wait_for_runtime_evidence(self):
        deadline = time.monotonic() + self.timeout_wall_sec
        post_complete_deadline = None
        while time.monotonic() < deadline:
            snapshot = self._snapshot()
            if snapshot["failure"] is not None:
                self.fail(snapshot["failure"])
            if snapshot["complete_sim_time"] is not None:
                if post_complete_deadline is None:
                    post_complete_deadline = (
                        time.monotonic() + self.post_complete_timeout_wall_sec
                    )
                if snapshot["post_complete_checked_stamps"]:
                    return snapshot
                if time.monotonic() >= post_complete_deadline:
                    self.fail(
                        "scenario_complete arrived, but no later checked bundle "
                        "proved a current terminal dynamic-TF horizon"
                    )
            time.sleep(0.05)
        self.fail(
            "S04 runtime evidence timed out after {:.1f}s; events={} raw={} "
            "scored_checked={} tf={} geometry={}".format(
                self.timeout_wall_sec,
                [event.get("event") for event in self._snapshot()["events"]],
                len(self._snapshot()["raw_summaries"]),
                self._snapshot()["scored_checked_count"],
                len(self._snapshot()["dynamic_tf_samples"]),
                len(self._snapshot()["geometry_summaries"]),
            )
        )

    @staticmethod
    def _assert_pose(test, actual, expected, label):
        test.assertEqual(set(actual), {"x", "y", "z", "yaw"}, label)
        for key, value in expected.items():
            test.assertAlmostEqual(float(actual[key]), value, places=9, msg=label)

    def _check_events(self, snapshot):
        expected_names = [
            "cycle_start",
            "initial_clear_end",
            "combined_visibility_start",
            "combined_visibility_end",
            "final_clear_start",
            "cycle_complete",
            "final_hold_start",
            "scenario_complete",
        ]
        self.assertEqual(
            [event["event"] for event in snapshot["events"]], expected_names,
            "S04 event stream is missing, duplicated, reordered, or contains failure",
        )
        start_ns = int(snapshot["events"][0]["sim_time_ns"])
        expected_offsets_ns = (
            0, 3_000_000_000, 7_000_000_000, 12_000_000_000,
            17_000_000_000, 20_000_000_000, 20_000_000_000,
            23_000_000_000,
        )
        observer_poses = (
            (4.0, -3.0, 2.0, -math.pi / 4.0),
            (4.0, -3.0, 2.0, -math.pi / 4.0),
            (0.0, 0.0, 2.0, 0.0),
            (-2.0, -6.0 / 13.0, 2.0, 0.4),
            (4.0, -3.0, 2.0, math.pi / 4.0),
            (4.0, -3.0, 2.0, math.pi / 4.0),
            (4.0, -3.0, 2.0, math.pi / 4.0),
            (4.0, -3.0, 2.0, math.pi / 4.0),
        )
        target_poses = {
            "uav2": {"x": 8.0, "y": 24.0 / 13.0, "z": 2.25, "yaw": 0.0},
            "uav3": {"x": 13.0, "y": 3.0, "z": 2.25, "yaw": 0.0},
            "uav4": {"x": 14.0, "y": 0.0, "z": 2.25, "yaw": 0.0},
        }
        for index, event in enumerate(snapshot["events"]):
            self.assertEqual(event["scenario"], SCENARIO_ID)
            self.assertEqual(event["scenario_profile"], "s04")
            self.assertEqual(event["semantic_contract_id"], SEMANTIC_CONTRACT_ID)
            self.assertEqual(event["seed"], 707)
            self.assertEqual(event["cycle"], 1)
            self.assertEqual(event["repeat_mode"], "restart")
            self.assertEqual(event["dynamic_tf_authority"], "scenario_manager")
            self.assertEqual(
                int(event["sim_time_ns"]), start_ns + expected_offsets_ns[index]
            )
            self.assertAlmostEqual(
                float(event["sim_time"]), int(event["sim_time_ns"]) * 1.0e-9,
                places=9,
            )
            self.assertEqual(
                set(event["commanded_pose"]), {"uav1", "uav2", "uav3", "uav4"}
            )
            observer = observer_poses[index]
            self._assert_pose(
                self,
                event["commanded_pose"]["uav1"],
                {"x": observer[0], "y": observer[1], "z": observer[2], "yaw": observer[3]},
                "{} uav1 boundary pose".format(event["event"]),
            )
            for name, pose in target_poses.items():
                self._assert_pose(
                    self, event["commanded_pose"][name], pose,
                    "{} {} boundary pose".format(event["event"], name),
                )

        phase_contract = {
            "cycle_start": ("initial_clear_hold", 0),
            "initial_clear_end": ("approach_combined_occlusion", 1),
            "combined_visibility_start": ("combined_visibility_scored", 2),
            "combined_visibility_end": ("leave_combined_visibility", 3),
            "final_clear_start": ("final_clear_hold", 4),
            "cycle_complete": ("final_clear_hold", 4),
        }
        by_name = {event["event"]: event for event in snapshot["events"]}
        for name, (phase, phase_index) in phase_contract.items():
            self.assertEqual(by_name[name]["phase"], phase)
            self.assertEqual(by_name[name]["phase_index"], phase_index)
        self.assertAlmostEqual(by_name["final_hold_start"]["duration"], 3.0)
        self.assertEqual(by_name["scenario_complete"]["repeat_count"], 1)
        return by_name

    def _nearest_model_sample(self, samples, stamp):
        sample = min(samples, key=lambda item: abs(item[0] - stamp))
        self.assertLess(
            abs(sample[0] - stamp), 0.12,
            "ModelStates lacks a sample near scenario boundary {:.6f}".format(stamp),
        )
        return sample

    def _check_actual_motion(self, snapshot, events):
        for name, samples in snapshot["model_samples"].items():
            self.assertGreater(len(samples), 30, "missing actual {} poses".format(name))
            self.assertTrue(
                all(all(math.isfinite(value) for value in sample) for sample in samples),
                "non-finite actual pose/twist for {}".format(name),
            )

        expected_targets = {
            "uav2": (8.0, 24.0 / 13.0, 2.25, 0.0),
            "uav3": (13.0, 3.0, 2.25, 0.0),
            "uav4": (14.0, 0.0, 2.25, 0.0),
        }
        for name, expected in expected_targets.items():
            samples = snapshot["model_samples"][name]
            for axis, label in ((1, "x"), (2, "y"), (3, "z")):
                values = [sample[axis] for sample in samples]
                self.assertLess(max(values) - min(values), 0.04, "{} {} moved".format(name, label))
                self.assertLess(
                    max(abs(value - expected[axis - 1]) for value in values), 0.05,
                    "{} actual {} differs from frozen pose".format(name, label),
                )
            self.assertLess(
                max(abs(_angle_error(sample[4], expected[3])) for sample in samples),
                0.02,
            )
            self.assertLess(max(sample[5] for sample in samples), 0.03)
            self.assertLess(max(sample[6] for sample in samples), 0.03)

        observer = snapshot["model_samples"]["uav1"]
        self.assertLess(
            max(sample[3] for sample in observer)
            - min(sample[3] for sample in observer),
            0.04,
            "actual uav1 altitude changed during the S04 trajectory",
        )
        boundary_expectations = {
            "cycle_start": (4.0, -3.0, 2.0, -math.pi / 4.0),
            "combined_visibility_start": (0.0, 0.0, 2.0, 0.0),
            "combined_visibility_end": (-2.0, -6.0 / 13.0, 2.0, 0.4),
            "final_clear_start": (4.0, -3.0, 2.0, math.pi / 4.0),
            "scenario_complete": (4.0, -3.0, 2.0, math.pi / 4.0),
        }
        for name, expected in boundary_expectations.items():
            sample = self._nearest_model_sample(observer, float(events[name]["sim_time"]))
            self.assertLess(_norm(tuple(sample[i + 1] - expected[i] for i in range(3))), 0.12, name)
            self.assertLess(abs(_angle_error(sample[4], expected[3])), 0.035, name)

        scored_start = float(events["combined_visibility_start"]["sim_time"])
        scored_end = float(events["combined_visibility_end"]["sim_time"])
        scored = [
            sample for sample in observer
            if scored_start + 0.15 <= sample[0] <= scored_end - 0.15
        ]
        self.assertGreater(len(scored), 30, "actual scored motion is undersampled")
        self.assertGreater(max(sample[1] for sample in scored) - min(sample[1] for sample in scored), 1.75)
        self.assertGreater(max(sample[2] for sample in scored) - min(sample[2] for sample in scored), 0.35)
        yaw_span = max(sample[4] for sample in scored) - min(sample[4] for sample in scored)
        self.assertGreater(yaw_span, 0.34)
        self.assertGreater(max(sample[5] for sample in scored), 0.35)
        self.assertGreater(max(sample[6] for sample in scored), 0.06)
        self.assertLess(max(sample[6] for sample in scored), 0.11)

    def _check_tf_contract(self, snapshot, events):
        self.assertEqual(
            snapshot["dynamic_tf_authorities"], {"/scenario_manager"},
            "world -> uav1/fcu must have exactly one dynamic caller",
        )
        self.assertEqual(
            snapshot["static_world_fcu_authorities"], set(),
            "world -> uav1/fcu was also published statically",
        )
        self.assertEqual(
            snapshot["static_sensor_authorities"], {"/uav1_fcu_to_mid360"},
            "uav1/fcu -> Mid-360 must have one static caller",
        )

        # Deduplicate by exact ROS timestamp before measuring source cadence.
        samples = sorted(
            {sample[0]: sample for sample in snapshot["dynamic_tf_samples"]}.values(),
            key=lambda sample: sample[0],
        )
        self.assertGreater(len(samples), 500, "dynamic TF cadence is too sparse")
        gaps = [second[0] - first[0] for first, second in zip(samples, samples[1:])]
        self.assertTrue(gaps)
        self.assertGreater(min(gaps), 0.0)
        self.assertLess(max(gaps), self.maximum_tf_gap_sec)
        self.assertGreater(
            samples[-1][0], snapshot["complete_sim_time"] + 1.0e-3,
            "dynamic TF did not remain current after scenario_complete",
        )

        scored_start = float(events["combined_visibility_start"]["sim_time"])
        scored_end = float(events["combined_visibility_end"]["sim_time"])
        scored = [sample for sample in samples if scored_start <= sample[0] <= scored_end]
        self.assertGreater(len(scored), 150, "scored dynamic TF is undersampled")
        first = min(scored, key=lambda item: abs(item[0] - scored_start))
        last = min(scored, key=lambda item: abs(item[0] - scored_end))
        translation = _norm((last[1] - first[1], last[2] - first[2], last[3] - first[3]))
        yaw_change = abs(_angle_error(last[4], first[4]))
        self.assertGreaterEqual(translation, self.minimum_scored_translation_m)
        self.assertGreaterEqual(yaw_change, self.minimum_scored_yaw_rad)
        self.assertLess(abs(first[1]), 0.05)
        self.assertLess(abs(first[2]), 0.05)
        self.assertLess(abs(first[4]), 0.02)
        self.assertLess(abs(last[1] + 2.0), 0.05)
        self.assertLess(abs(last[2] + 6.0 / 13.0), 0.05)
        self.assertLess(abs(_angle_error(last[4], 0.4)), 0.02)
        midpoint = min(scored, key=lambda item: abs(item[0] - (scored_start + 2.5)))
        self.assertLess(abs(midpoint[1] + 1.0), 0.06)
        self.assertLess(abs(midpoint[2] + 3.0 / 13.0), 0.06)
        self.assertLess(abs(_angle_error(midpoint[4], 0.2)), 0.025)

    def _check_ray_contract(self, snapshot):
        checked = snapshot["best_checked"]
        self.assertIsNotNone(checked, "no complete scored checked bundle was received")
        self.assertGreater(snapshot["scored_checked_count"], 10)
        self.assertEqual(checked.header.frame_id, "world")
        self.assertEqual(checked.source_header.frame_id, "uav1/mid360_link")
        self.assertEqual(checked.source_mode, "sim_exact")
        self.assertEqual(checked.ray_time_geometry_mode, "per_ray_pose")
        self.assertEqual(len(checked.rays), self.expected_ray_count)
        self.assertGreater(snapshot["best_checked_motion"], 0.030)
        self.assertLess(snapshot["best_checked_motion"], 0.060)

        key = _bundle_key(checked.source_header)
        self.assertIn(key, snapshot["raw_summaries"], "checked scan lacks raw peer")
        raw = snapshot["raw_summaries"][key]
        self.assertEqual(raw["scan_id"], int(checked.scan_id))
        self.assertEqual(raw["pattern_start_index"], int(checked.pattern_start_index))
        self.assertEqual(raw["frame_id"], "uav1/mid360_link")
        self.assertEqual(raw["count"], self.expected_ray_count)
        self.assertAlmostEqual(raw["min_range"], 0.1, places=5)
        self.assertAlmostEqual(raw["max_range"], 40.0, places=5)
        self.assertTrue(raw["monotonic"])
        duration_sec = raw["duration_ns"] * 1.0e-9
        self.assertGreater(duration_sec, 0.095)
        self.assertLess(duration_sec, 0.105)

        indices = (0, len(checked.rays) // 2, len(checked.rays) - 1)
        checked_signatures = tuple(
            (
                int(checked.rays[index].source.pattern_index),
                int(checked.rays[index].source.offset_time_ns),
                int(checked.rays[index].source.return_status),
                float(checked.rays[index].source.range),
            )
            for index in indices
        )
        self.assertEqual(raw["signatures"], checked_signatures)

        statuses = {}
        previous_offset = -1
        maximum_source_norm_error = 0.0
        maximum_checked_norm_error = 0.0
        for index, checked_ray in enumerate(checked.rays):
            source = checked_ray.source
            self.assertGreaterEqual(int(source.offset_time_ns), previous_offset)
            previous_offset = int(source.offset_time_ns)
            self.assertEqual(int(checked_ray.original_index), index)
            self.assertTrue(checked_ray.transform_valid)
            self.assertTrue(checked_ray.direction_valid)
            values = (
                source.dir_x, source.dir_y, source.dir_z, source.range,
                source.intensity, checked_ray.origin.x, checked_ray.origin.y,
                checked_ray.origin.z, checked_ray.direction.x,
                checked_ray.direction.y, checked_ray.direction.z,
            )
            self.assertTrue(all(math.isfinite(value) for value in values))
            maximum_source_norm_error = max(
                maximum_source_norm_error,
                abs(_norm((source.dir_x, source.dir_y, source.dir_z)) - 1.0),
            )
            maximum_checked_norm_error = max(
                maximum_checked_norm_error,
                abs(_norm((checked_ray.direction.x, checked_ray.direction.y, checked_ray.direction.z)) - 1.0),
            )
            self.assertIn(
                int(source.return_status),
                (Ray.NO_RETURN, Ray.VALID_RETURN, Ray.BELOW_MIN_RANGE, Ray.INVALID_RANGE, Ray.UNKNOWN_STATUS),
            )
            statuses[int(source.return_status)] = statuses.get(int(source.return_status), 0) + 1
            if source.return_status == Ray.VALID_RETURN:
                self.assertGreater(float(source.range), float(checked.min_range))
                self.assertLess(float(source.range), float(checked.max_range))
            else:
                self.assertEqual(float(source.range), 0.0)
                self.assertEqual(float(source.intensity), 0.0)
        self.assertLess(maximum_source_norm_error, 1.0e-4)
        self.assertLess(maximum_checked_norm_error, 1.0e-4)
        self.assertGreater(statuses.get(Ray.VALID_RETURN, 0), 0)
        self.assertGreater(statuses.get(Ray.NO_RETURN, 0), 0)
        self.assertEqual(sum(statuses.values()), self.expected_ray_count)

        self.assertIn(key, snapshot["source_diagnostics"])
        source_diagnostic = snapshot["source_diagnostics"][key]
        self.assertEqual(source_diagnostic["level"], DiagnosticStatus.OK)
        source_values = source_diagnostic["values"]
        exact_source = {
            "source_mode": "sim_exact",
            "exact_direction_available": "true",
            "ray_time_geometry_mode": "per_ray_pose",
            "motion_model": "constant_twist_world_velocity",
            "scene_assumption": "static_scene_only",
        }
        for name, expected in exact_source.items():
            self.assertEqual(source_values[name], expected)
        self.assertEqual(int(source_values["ray_count"]), self.expected_ray_count)
        self.assertEqual(int(source_values["valid_return_count"]), statuses.get(Ray.VALID_RETURN, 0))
        self.assertEqual(int(source_values["no_return_count"]), statuses.get(Ray.NO_RETURN, 0))
        self.assertEqual(int(source_values["below_min_range_count"]), statuses.get(Ray.BELOW_MIN_RANGE, 0))
        invalid_count = self.expected_ray_count - sum(
            statuses.get(status, 0)
            for status in (Ray.VALID_RETURN, Ray.NO_RETURN, Ray.BELOW_MIN_RANGE)
        )
        self.assertEqual(int(source_values["invalid_count"]), invalid_count)
        self.assertLess(float(source_values["direction_norm_error_max"]), 1.0e-4)
        self.assertAlmostEqual(float(source_values["timestamp_monotonic_ratio"]), 1.0)
        self.assertAlmostEqual(float(source_values["bundle_duration"]), duration_sec, places=5)
        self.assertGreater(float(source_values["linear_speed_mps"]), 0.35)
        self.assertLess(float(source_values["linear_speed_mps"]), 0.48)
        self.assertGreater(float(source_values["angular_speed_radps"]), 0.06)
        self.assertLess(float(source_values["angular_speed_radps"]), 0.10)
        self.assertAlmostEqual(float(source_values["max_offset_sec"]), duration_sec, places=5)

        self.assertIn(key, snapshot["preprocessor_diagnostics"])
        preprocessor = snapshot["preprocessor_diagnostics"][key]
        self.assertEqual(preprocessor["level"], DiagnosticStatus.OK)
        values = preprocessor["values"]
        self.assertEqual(values["source_mode"], "sim_exact")
        self.assertEqual(values["exact_direction_available"], "true")
        self.assertEqual(values["ray_time_geometry_mode"], "per_ray_pose")
        self.assertEqual(values["tf_missing_ratio"], "0")
        self.assertEqual(values["tf_missing_bundle_ratio"], "0")
        self.assertEqual(values["input_index_alignment"], "true")
        self.assertEqual(values["direction_invalid_count"], "0")
        self.assertEqual(int(values["ray_count"]), self.expected_ray_count)
        self.assertEqual(int(values["valid_return_count"]), statuses.get(Ray.VALID_RETURN, 0))
        self.assertEqual(int(values["no_return_count"]), statuses.get(Ray.NO_RETURN, 0))
        self.assertEqual(values["tf_source_cadence_observation_enabled"], "true")
        self.assertEqual(values["tf_source_cadence_semantics"], "observed_tf_topic_edge")
        self.assertEqual(values["tf_source_cadence_relation_to_lookup"], "not_tf2_actual_brackets")
        self.assertEqual(values["tf_source_cadence_topic"], "/tf")
        self.assertEqual(values["tf_source_cadence_parent_frame"], "world")
        self.assertEqual(values["tf_source_cadence_child_frame"], "uav1/fcu")
        self.assertEqual(values["tf_source_sample_bracket_status"], "bracketed")
        self.assertEqual(values["tf_source_sample_bracketed"], "true")
        observed_gap = float(values["tf_source_sample_max_gap_sec"])
        self.assertGreater(observed_gap, 0.0)
        self.assertLess(observed_gap, self.maximum_tf_gap_sec)
        self.assertGreaterEqual(int(values["tf_source_sample_covering_count"]), 2)
        query_start_ns = checked.source_header.stamp.to_nsec()
        query_end_ns = query_start_ns + raw["duration_ns"]
        self.assertLessEqual(int(values["tf_source_sample_left_bracket_ns"]), query_start_ns)
        self.assertGreaterEqual(int(values["tf_source_sample_right_bracket_ns"]), query_end_ns)
        self.assertEqual(values["tf_max_interval_semantics"], "deprecated_alias_of_tf_query_span_sec")
        self.assertEqual(int(values["tf_lookup_count"]), 2)
        self.assertGreater(float(values["tf_query_span_sec"]), 0.095)
        self.assertLess(float(values["tf_query_span_sec"]), 0.105)
        self.assertGreater(snapshot["post_cycle_source_ok_count"], 100)
        self.assertGreater(snapshot["post_cycle_preprocessor_ok_count"], 100)
        self.assertTrue(snapshot["post_complete_checked_stamps"])
        return checked, indices

    def _check_per_ray_tf_geometry(self, checked, indices):
        start_stamp = checked.source_header.stamp
        end_stamp = start_stamp + rospy.Duration.from_sec(
            checked.rays[-1].source.offset_time_ns * 1.0e-9
        )
        start_tf = self.tf_buffer.lookup_transform(
            "world", "uav1/mid360_link", start_stamp, rospy.Duration(2.0)
        )
        end_tf = self.tf_buffer.lookup_transform(
            "world", "uav1/mid360_link", end_stamp, rospy.Duration(2.0)
        )
        translation = _norm(
            (
                end_tf.transform.translation.x - start_tf.transform.translation.x,
                end_tf.transform.translation.y - start_tf.transform.translation.y,
                end_tf.transform.translation.z - start_tf.transform.translation.z,
            )
        )
        yaw_change = abs(_angle_error(
            _yaw(end_tf.transform.rotation), _yaw(start_tf.transform.rotation)
        ))
        self.assertGreater(translation, 0.030)
        self.assertLess(translation, 0.060)
        self.assertGreater(yaw_change, 0.005)
        self.assertLess(yaw_change, 0.012)

        for index in indices:
            checked_ray = checked.rays[index]
            stamp = checked.source_header.stamp + rospy.Duration.from_sec(
                checked_ray.source.offset_time_ns * 1.0e-9
            )
            transform = self.tf_buffer.lookup_transform(
                "world", "uav1/mid360_link", stamp, rospy.Duration(2.0)
            )
            translation = transform.transform.translation
            self.assertLess(
                _norm((
                    checked_ray.origin.x - translation.x,
                    checked_ray.origin.y - translation.y,
                    checked_ray.origin.z - translation.z,
                )),
                0.025,
                "checked per-ray origin disagrees with timestamped TF",
            )
            expected_direction = _rotate(
                transform.transform.rotation,
                (
                    checked_ray.source.dir_x,
                    checked_ray.source.dir_y,
                    checked_ray.source.dir_z,
                ),
            )
            self.assertLess(
                _norm((
                    checked_ray.direction.x - expected_direction[0],
                    checked_ray.direction.y - expected_direction[1],
                    checked_ray.direction.z - expected_direction[2],
                )),
                1.0e-4,
                "checked per-ray direction disagrees with timestamped yaw TF",
            )

    def _check_e2_geometric_returns(self, snapshot):
        summaries = snapshot["geometry_summaries"]
        self.assertGreaterEqual(len(summaries), self.minimum_geometry_scans)
        wall_hits = sum(item["wall_hits"] for item in summaries)
        pillar_hits = sum(item["pillar_hits"] for item in summaries)
        self.assertGreaterEqual(
            wall_hits, self.minimum_wall_hit_count,
            "no adequate endpoints were geometrically on the E2 wall front face",
        )
        self.assertGreaterEqual(
            pillar_hits, self.minimum_pillar_hit_count,
            "no adequate endpoints were geometrically on the E2 pillar cylinder",
        )
        self.assertTrue(any(item["wall_hits"] for item in summaries))
        self.assertTrue(any(item["pillar_hits"] for item in summaries))
        rospy.loginfo(
            "S04 E2 endpoint geometry (not semantic labels): scans=%d wall=%d pillar=%d",
            len(summaries), wall_hits, pillar_hits,
        )

    def _check_truth_isolation(self):
        _publishers, subscribers, _services = rosgraph.Master(
            rospy.get_name()
        ).getSystemState()
        consumers = set(dict(subscribers).get("/gazebo/model_states", []))
        self.assertIn(rospy.get_name(), consumers)
        offenders = sorted(consumers - {rospy.get_name()})
        self.assertEqual(
            offenders, [],
            "production nodes illegally consume /gazebo/model_states",
        )

    def test_gate_s04_live_contract(self):
        snapshot = self._wait_for_runtime_evidence()
        self.assertIsNone(snapshot["failure"], snapshot["failure"])
        events = self._check_events(snapshot)
        self._check_actual_motion(snapshot, events)
        self._check_tf_contract(snapshot, events)
        checked, indices = self._check_ray_contract(snapshot)
        self._check_per_ray_tf_geometry(checked, indices)
        self._check_e2_geometric_returns(snapshot)
        self._check_truth_isolation()


if __name__ == "__main__":
    rospy.init_node("gate_s04_contract")
    rostest.rosrun(
        "mid360_multi_uav_sim", "gate_s04_contract", GateS04Contract
    )
