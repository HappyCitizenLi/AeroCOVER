#!/usr/bin/env python3
"""Live S03 contract for moving-observer, per-ray Mid-360 geometry.

This test intentionally consumes Gazebo model states as an independent test
oracle.  No production node is allowed to consume that topic.  Dynamic TF
authority is attributed only after filtering the exact world -> uav1/fcu
transform; counting every publisher or every transform on /tf would conflate
unrelated frames with competing authorities.
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


def _frame(value):
    return str(value).lstrip("/")


def _stamp_ns(header):
    return header.stamp.to_nsec()


def _bundle_key(header):
    return (int(header.seq), _stamp_ns(header))


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
    """Rotate a vector without adding a tf-transformations dependency."""
    qx, qy, qz, qw = (
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w,
    )
    vx, vy, vz = vector
    # q * [v, 0] * conjugate(q), in its compact cross-product form.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


class GateS03Contract(unittest.TestCase):

    def setUp(self):
        self.timeout_wall_sec = float(rospy.get_param("~timeout_wall_sec", 165.0))
        self.post_complete_timeout_wall_sec = float(
            rospy.get_param("~post_complete_timeout_wall_sec", 12.0)
        )
        self.expected_ray_count = int(rospy.get_param("~expected_ray_count", 20000))
        self.minimum_translation_m = float(
            rospy.get_param("~minimum_translation_m", 3.5)
        )
        self.wall_plane_x = float(rospy.get_param("~wall_plane_x", 24.75))
        self.wall_plane_tolerance_m = float(
            rospy.get_param("~wall_plane_tolerance_m", 0.02)
        )
        self.wall_endpoint_spread_tolerance_m = float(
            rospy.get_param("~wall_endpoint_spread_tolerance_m", 0.02)
        )
        self.minimum_wall_hit_offset_span_sec = float(
            rospy.get_param("~minimum_wall_hit_offset_span_sec", 0.025)
        )
        self.minimum_wall_hit_late_offset_sec = float(
            rospy.get_param("~minimum_wall_hit_late_offset_sec", 0.06)
        )
        self.maximum_wall_endpoint_slope_mps = float(
            rospy.get_param("~maximum_wall_endpoint_slope_mps", 0.12)
        )
        self.minimum_wall_hit_count = int(
            rospy.get_param("~minimum_wall_hit_count", 25)
        )

        self.lock = threading.Lock()
        self.failure = None
        self.complete_event = threading.Event()
        self.complete_sim_time = None
        self.events = []

        self.model_samples = {name: [] for name in ("uav1", "uav2", "uav3")}
        self.raw_summaries = {}
        self.source_diagnostics = {}
        self.preprocessor_diagnostics = {}
        self.best_checked_bundle = None
        self.best_checked_motion = -math.inf
        self.post_complete_checked_stamps = []

        self.dynamic_tf_authorities = set()
        self.dynamic_tf_stamps = []
        self.dynamic_tf_positions = []
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

        # This independently buffers the same TF traffic for timestamped
        # geometry checks.  Authority evidence comes from the explicit /tf
        # subscribers above, not from tf2's graph abstraction.
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(40.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def _set_failure(self, error):
        with self.lock:
            if self.failure is None:
                self.failure = str(error)
                rospy.logerr("S03 contract callback failed: %s", self.failure)
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
                    math.sqrt(
                        twist.linear.x ** 2
                        + twist.linear.y ** 2
                        + twist.linear.z ** 2
                    ),
                    math.sqrt(
                        twist.angular.x ** 2
                        + twist.angular.y ** 2
                        + twist.angular.z ** 2
                    ),
                )
            with self.lock:
                for name, sample in samples.items():
                    self.model_samples[name].append(sample)
        except Exception as error:
            self._set_failure(error)

    def _raw_callback(self, message):
        try:
            count = len(message.rays)
            indices = (0, count // 2, count - 1) if count else ()
            signatures = tuple(
                (
                    int(message.rays[index].pattern_index),
                    int(message.rays[index].offset_time_ns),
                    int(message.rays[index].return_status),
                    float(message.rays[index].range),
                )
                for index in indices
            )
            monotonic = all(
                message.rays[index].offset_time_ns
                >= message.rays[index - 1].offset_time_ns
                for index in range(1, count)
            )
            summary = {
                "scan_id": int(message.scan_id),
                "frame_id": str(message.header.frame_id),
                "count": count,
                "min_range": float(message.min_range),
                "max_range": float(message.max_range),
                "duration_ns": int(message.rays[-1].offset_time_ns) if count else 0,
                "monotonic": monotonic,
                "signatures": signatures,
            }
            with self.lock:
                self.raw_summaries[_bundle_key(message.header)] = summary
        except Exception as error:
            self._set_failure(error)

    def _checked_callback(self, message):
        try:
            count = len(message.rays)
            motion = -math.inf
            if count:
                motion = (
                    message.rays[-1].origin.x - message.rays[0].origin.x
                )
            stamp_sec = message.source_header.stamp.to_sec()
            with self.lock:
                if motion > self.best_checked_motion:
                    self.best_checked_motion = motion
                    self.best_checked_bundle = message
                if (
                    self.complete_sim_time is not None
                    and stamp_sec > self.complete_sim_time + 1.0e-3
                ):
                    self.post_complete_checked_stamps.append(stamp_sec)
        except Exception as error:
            self._set_failure(error)

    def _source_diagnostics_callback(self, message):
        try:
            summary = {
                "level": max(
                    (int(status.level) for status in message.status), default=3
                ),
                "values": self._diagnostic_values(message),
            }
            with self.lock:
                self.source_diagnostics[_bundle_key(message.header)] = summary
        except Exception as error:
            self._set_failure(error)

    def _preprocessor_diagnostics_callback(self, message):
        try:
            summary = {
                "level": max(
                    (int(status.level) for status in message.status), default=3
                ),
                "values": self._diagnostic_values(message),
            }
            with self.lock:
                self.preprocessor_diagnostics[_bundle_key(message.header)] = summary
        except Exception as error:
            self._set_failure(error)

    def _event_callback(self, message):
        try:
            event = json.loads(message.data)
            required = ("scenario", "event", "cycle", "sim_time")
            if any(key not in event for key in required):
                raise ValueError("scenario event lacks required JSON fields")
            with self.lock:
                self.events.append(event)
                if event["event"] == "scenario_complete":
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
                        )
                    )
            if not matches:
                return
            with self.lock:
                self.dynamic_tf_authorities.add(authority)
                for sample in matches:
                    self.dynamic_tf_stamps.append(sample[0])
                    self.dynamic_tf_positions.append(sample)
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
                "best_checked_bundle": self.best_checked_bundle,
                "best_checked_motion": self.best_checked_motion,
                "post_complete_checked_stamps": list(
                    self.post_complete_checked_stamps
                ),
                "dynamic_tf_authorities": set(self.dynamic_tf_authorities),
                "dynamic_tf_stamps": list(self.dynamic_tf_stamps),
                "dynamic_tf_positions": list(self.dynamic_tf_positions),
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
                        "scenario_complete was observed, but no later checked "
                        "bundle proved that the terminal dynamic TF remained live"
                    )
            time.sleep(0.05)
        self.fail("S03 runtime evidence timed out after {:.1f}s".format(
            self.timeout_wall_sec
        ))

    def _check_events_and_actual_motion(self, snapshot):
        names = [event["event"] for event in snapshot["events"]]
        self.assertIn("cycle_start", names)
        self.assertIn("observer_motion_scored_start", names)
        self.assertIn("observer_motion_scored_end", names)
        self.assertIn("scenario_complete", names)

        for name, samples in snapshot["model_samples"].items():
            self.assertGreater(len(samples), 20, "missing actual {} poses".format(name))
            self.assertTrue(
                all(all(math.isfinite(value) for value in sample) for sample in samples),
                "non-finite actual pose for {}".format(name),
            )

        observer = snapshot["model_samples"]["uav1"]
        xs = [sample[1] for sample in observer]
        ys = [sample[2] for sample in observer]
        zs = [sample[3] for sample in observer]
        yaws = [sample[4] for sample in observer]
        self.assertGreaterEqual(max(xs) - min(xs), self.minimum_translation_m)
        self.assertLess(max(ys) - min(ys), 0.04)
        self.assertLess(max(zs) - min(zs), 0.04)
        self.assertLess(
            max(abs(_angle_error(yaw, 0.0)) for yaw in yaws), 0.015,
            "actual uav1 yaw changed during translation",
        )

        for name in ("uav2", "uav3"):
            samples = snapshot["model_samples"][name]
            for axis, label in ((1, "x"), (2, "y"), (3, "z")):
                values = [sample[axis] for sample in samples]
                self.assertLess(
                    max(values) - min(values),
                    0.04,
                    "actual {} {} moved".format(name, label),
                )
            self.assertLess(
                max(abs(_angle_error(sample[4], 0.0)) for sample in samples),
                0.015,
                "actual {} yaw changed".format(name),
            )
            self.assertLess(
                max(sample[5] for sample in samples),
                0.03,
                "actual {} linear twist is not static".format(name),
            )
            self.assertLess(
                max(sample[6] for sample in samples),
                0.03,
                "actual {} angular twist is not static".format(name),
            )

    def _check_tf_authority(self, snapshot):
        self.assertEqual(
            snapshot["dynamic_tf_authorities"],
            {"/scenario_manager"},
            "world -> uav1/fcu must have exactly one dynamic TF caller",
        )
        self.assertEqual(
            snapshot["static_world_fcu_authorities"], set(),
            "world -> uav1/fcu was also published on /tf_static",
        )
        self.assertEqual(
            snapshot["static_sensor_authorities"],
            {"/uav1_fcu_to_mid360"},
            "uav1/fcu -> Mid-360 must have one static authority",
        )

        stamps = sorted(set(snapshot["dynamic_tf_stamps"]))
        self.assertGreater(len(stamps), 100, "dynamic TF cadence is too sparse")
        self.assertGreater(stamps[-1] - stamps[0], 5.0)
        positive_gaps = [
            second - first
            for first, second in zip(stamps, stamps[1:])
            if second > first
        ]
        self.assertTrue(positive_gaps)
        self.assertLess(max(positive_gaps), 0.20, "dynamic TF has a long time gap")
        self.assertGreater(
            stamps[-1], snapshot["complete_sim_time"] + 1.0e-3,
            "dynamic TF did not remain current after scenario_complete",
        )

    def _check_ray_and_diagnostic_contract(self, snapshot):
        checked = snapshot["best_checked_bundle"]
        self.assertIsNotNone(checked, "no checked bundle was received")
        self.assertEqual(checked.header.frame_id, "world")
        self.assertEqual(checked.source_header.frame_id, "uav1/mid360_link")
        self.assertEqual(checked.source_mode, "sim_exact")
        self.assertEqual(checked.ray_time_geometry_mode, "per_ray_pose")
        self.assertEqual(len(checked.rays), self.expected_ray_count)
        self.assertGreater(snapshot["best_checked_motion"], 0.025)

        key = _bundle_key(checked.source_header)
        self.assertIn(key, snapshot["raw_summaries"], "checked scan lacks raw peer")
        raw = snapshot["raw_summaries"][key]
        self.assertEqual(raw["scan_id"], checked.scan_id)
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

        monotonic = True
        all_indices = True
        all_transforms = True
        all_directions_valid = True
        all_finite = True
        max_source_norm_error = 0.0
        statuses = {}
        previous_offset = 0
        for index, ray in enumerate(checked.rays):
            source = ray.source
            monotonic = monotonic and source.offset_time_ns >= previous_offset
            previous_offset = source.offset_time_ns
            all_indices = all_indices and ray.original_index == index
            all_transforms = all_transforms and ray.transform_valid
            all_directions_valid = all_directions_valid and ray.direction_valid
            all_finite = all_finite and all(
                math.isfinite(value)
                for value in (
                    source.dir_x, source.dir_y, source.dir_z, source.range,
                    source.intensity, ray.origin.x, ray.origin.y, ray.origin.z,
                    ray.direction.x, ray.direction.y, ray.direction.z,
                )
            )
            norm = math.sqrt(
                source.dir_x * source.dir_x
                + source.dir_y * source.dir_y
                + source.dir_z * source.dir_z
            )
            max_source_norm_error = max(max_source_norm_error, abs(norm - 1.0))
            statuses[source.return_status] = statuses.get(source.return_status, 0) + 1
        self.assertTrue(monotonic)
        self.assertTrue(all_indices)
        self.assertTrue(all_transforms)
        self.assertTrue(all_directions_valid)
        self.assertTrue(all_finite)
        self.assertLess(max_source_norm_error, 1.0e-4)
        self.assertGreater(statuses.get(Ray.VALID_RETURN, 0), 0)
        self.assertGreater(statuses.get(Ray.NO_RETURN, 0), 0)

        self.assertIn(key, snapshot["source_diagnostics"])
        source_diagnostic = snapshot["source_diagnostics"][key]
        self.assertEqual(source_diagnostic["level"], DiagnosticStatus.OK)
        source_values = source_diagnostic["values"]
        self.assertEqual(source_values["source_mode"], "sim_exact")
        self.assertEqual(source_values["ray_time_geometry_mode"], "per_ray_pose")
        self.assertEqual(source_values["motion_model"], "constant_twist_world_velocity")
        self.assertEqual(source_values["scene_assumption"], "static_scene_only")
        self.assertEqual(int(source_values["ray_count"]), self.expected_ray_count)
        self.assertGreater(float(source_values["linear_speed_mps"]), 0.35)
        self.assertLess(float(source_values["linear_speed_mps"]), 0.65)
        self.assertLess(float(source_values["angular_speed_radps"]), 0.03)
        self.assertAlmostEqual(
            float(source_values["max_offset_sec"]), duration_sec, places=5
        )

        self.assertIn(key, snapshot["preprocessor_diagnostics"])
        preprocessor_diagnostic = snapshot["preprocessor_diagnostics"][key]
        self.assertEqual(preprocessor_diagnostic["level"], DiagnosticStatus.OK)
        values = preprocessor_diagnostic["values"]
        self.assertEqual(values["ray_time_geometry_mode"], "per_ray_pose")
        self.assertEqual(values["source_mode"], "sim_exact")
        self.assertEqual(values["tf_missing_bundle_ratio"], "0")
        self.assertEqual(values["tf_source_cadence_observation_enabled"], "true")
        self.assertEqual(values["tf_source_sample_bracketed"], "true")
        self.assertEqual(int(values["ray_count"]), self.expected_ray_count)
        self.assertEqual(values["input_index_alignment"], "true")
        self.assertEqual(
            values["tf_source_cadence_semantics"], "observed_tf_topic_edge"
        )
        self.assertEqual(
            values["tf_source_cadence_relation_to_lookup"],
            "not_tf2_actual_brackets",
        )
        self.assertEqual(values["tf_source_cadence_topic"], "/tf")
        self.assertEqual(values["tf_source_cadence_parent_frame"], "world")
        self.assertEqual(values["tf_source_cadence_child_frame"], "uav1/fcu")
        observed_gap = float(values["tf_source_sample_max_gap_sec"])
        self.assertGreater(observed_gap, 0.0)
        self.assertLess(observed_gap, 0.20)
        self.assertGreaterEqual(
            int(values["tf_source_sample_covering_count"]), 2
        )
        query_start_ns = checked.source_header.stamp.to_nsec()
        query_end_ns = query_start_ns + raw["duration_ns"]
        self.assertLessEqual(
            int(values["tf_source_sample_left_bracket_ns"]), query_start_ns
        )
        self.assertGreaterEqual(
            int(values["tf_source_sample_right_bracket_ns"]), query_end_ns
        )
        rospy.loginfo(
            "S03 TF cadence proof: semantics=%s gap=%.6f s covering=%s "
            "left=%s query=[%d,%d] right=%s duplicate=%s out_of_order=%s",
            values["tf_source_cadence_semantics"], observed_gap,
            values["tf_source_sample_covering_count"],
            values["tf_source_sample_left_bracket_ns"], query_start_ns,
            query_end_ns, values["tf_source_sample_right_bracket_ns"],
            values["tf_source_duplicate_stamp_count"],
            values["tf_source_out_of_order_stamp_count"],
        )
        self.assertEqual(
            values["tf_max_interval_semantics"],
            "deprecated_alias_of_tf_query_span_sec",
        )
        self.assertEqual(int(values["tf_lookup_count"]), 2)
        self.assertGreater(float(values["tf_query_span_sec"]), 0.095)
        self.assertLess(float(values["tf_query_span_sec"]), 0.105)

        self.assertTrue(snapshot["post_complete_checked_stamps"])
        self.assertGreater(
            max(snapshot["post_complete_checked_stamps"]),
            snapshot["complete_sim_time"] + 1.0e-3,
        )
        return checked, indices

    def _check_per_ray_tf_geometry(self, checked, indices):
        origins = [checked.rays[index].origin.x for index in indices]
        self.assertLess(origins[0], origins[1])
        self.assertLess(origins[1], origins[2])

        offsets = [checked.rays[index].source.offset_time_ns * 1.0e-9 for index in indices]
        measured_speed = (origins[-1] - origins[0]) / (offsets[-1] - offsets[0])
        self.assertGreater(measured_speed, 0.35)
        self.assertLess(measured_speed, 0.65)

        for index in indices:
            ray = checked.rays[index]
            stamp = checked.source_header.stamp + rospy.Duration.from_sec(
                ray.source.offset_time_ns * 1.0e-9
            )
            transform = self.tf_buffer.lookup_transform(
                "world", "uav1/mid360_link", stamp, rospy.Duration(2.0)
            )
            translation = transform.transform.translation
            self.assertLess(
                math.sqrt(
                    (ray.origin.x - translation.x) ** 2
                    + (ray.origin.y - translation.y) ** 2
                    + (ray.origin.z - translation.z) ** 2
                ),
                0.025,
                "checked per-ray origin disagrees with timestamped TF",
            )
            expected_direction = _rotate(
                transform.transform.rotation,
                (ray.source.dir_x, ray.source.dir_y, ray.source.dir_z),
            )
            direction_error = math.sqrt(
                (ray.direction.x - expected_direction[0]) ** 2
                + (ray.direction.y - expected_direction[1]) ** 2
                + (ray.direction.z - expected_direction[2]) ** 2
            )
            self.assertLess(direction_error, 1.0e-4)

    def _check_wide_wall_hits(self, checked):
        wall_hits = []
        for ray in checked.rays:
            source = ray.source
            if (
                source.return_status != Ray.VALID_RETURN
                or source.range <= 18.0
                or ray.direction.x <= 0.25
            ):
                continue
            endpoint_x = ray.origin.x + source.range * ray.direction.x
            endpoint_y = ray.origin.y + source.range * ray.direction.y
            endpoint_z = ray.origin.z + source.range * ray.direction.z
            # Interior bounds exclude ground/top/side-edge contacts.  The
            # >18 m range also excludes both target UAV silhouettes.
            if -14.5 <= endpoint_y <= 14.5 and 0.25 <= endpoint_z <= 7.75:
                wall_hits.append(
                    (source.offset_time_ns * 1.0e-9, endpoint_x)
                )

        self.assertGreaterEqual(
            len(wall_hits), self.minimum_wall_hit_count,
            "too few unambiguous wide-wall returns",
        )
        offsets = [sample[0] for sample in wall_hits]
        endpoint_xs = [sample[1] for sample in wall_hits]
        offset_span = max(offsets) - min(offsets)
        rospy.loginfo(
            "S03 wall candidates: hits=%d offset=[%.6f, %.6f] s "
            "endpoint_x=[%.6f, %.6f] m",
            len(wall_hits), min(offsets), max(offsets), min(endpoint_xs),
            max(endpoint_xs),
        )
        self.assertGreaterEqual(
            offset_span,
            self.minimum_wall_hit_offset_span_sec,
            "wide-wall samples do not cover enough of the scan to distinguish "
            "per-ray raycasting from snapshot ranges",
        )
        self.assertGreaterEqual(
            max(offsets),
            self.minimum_wall_hit_late_offset_sec,
            "wide-wall proof lacks late-scan rays with a resolvable snapshot "
            "counterfactual displacement",
        )

        max_error = max(abs(value - self.wall_plane_x) for value in endpoint_xs)
        self.assertLessEqual(
            max_error,
            self.wall_plane_tolerance_m,
            "per-ray endpoints do not lie on the wall collision front plane",
        )
        endpoint_spread = max(endpoint_xs) - min(endpoint_xs)
        self.assertLessEqual(
            endpoint_spread,
            self.wall_endpoint_spread_tolerance_m,
            "wall endpoints drift across the scan; ranges may still use "
            "snapshot geometry while origins use per-ray TF",
        )

        mean_offset = sum(offsets) / len(offsets)
        mean_endpoint = sum(endpoint_xs) / len(endpoint_xs)
        time_variance = sum((value - mean_offset) ** 2 for value in offsets)
        self.assertGreater(time_variance, 0.0)
        endpoint_slope = sum(
            (offset - mean_offset) * (endpoint - mean_endpoint)
            for offset, endpoint in wall_hits
        ) / time_variance
        self.assertLessEqual(
            abs(endpoint_slope),
            self.maximum_wall_endpoint_slope_mps,
            "wall endpoint x still follows scan offset; expected static-plane "
            "compensation from native per-ray raycasting",
        )
        rospy.loginfo(
            "S03 wall proof: hits=%d offset_span=%.6f s max_error=%.6f m "
            "spread=%.6f m slope=%.6f m/s",
            len(wall_hits), offset_span, max_error, endpoint_spread,
            endpoint_slope,
        )

    def _check_truth_isolation(self):
        _publishers, subscribers, _services = rosgraph.Master(
            rospy.get_name()
        ).getSystemState()
        consumers = set(dict(subscribers).get("/gazebo/model_states", []))
        self.assertIn(rospy.get_name(), consumers)
        offenders = sorted(consumers - {rospy.get_name()})
        self.assertEqual(
            offenders,
            [],
            "production nodes illegally consume /gazebo/model_states",
        )

    def test_gate_s03_live_contract(self):
        snapshot = self._wait_for_runtime_evidence()
        self.assertIsNone(snapshot["failure"], snapshot["failure"])
        self._check_events_and_actual_motion(snapshot)
        self._check_tf_authority(snapshot)
        checked, indices = self._check_ray_and_diagnostic_contract(snapshot)
        self._check_per_ray_tf_geometry(checked, indices)
        self._check_wide_wall_hits(checked)
        self._check_truth_isolation()


if __name__ == "__main__":
    rospy.init_node("gate_s03_contract")
    rostest.rosrun(
        "mid360_multi_uav_sim", "gate_s03_contract", GateS03Contract
    )
