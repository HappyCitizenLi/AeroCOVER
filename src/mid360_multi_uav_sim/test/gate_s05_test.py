#!/usr/bin/env python3
"""Live S05 clutter-stress contract over real Gazebo and ROS traffic.

``/gazebo/model_states`` is consumed only by this gate as an independent
oracle. Endpoint families are inferred solely from collision primitives parsed
from the frozen E3 world and referenced model SDF files. They are geometric
classifications, not semantic labels from the production pipeline.
"""

import json
import math
from pathlib import Path
import threading
import time
import unittest
import xml.etree.ElementTree as ET

import rosgraph
import rospy
import rostest
import tf2_ros
import yaml
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from gazebo_msgs.msg import ModelStates
from mid360_ray_msgs.msg import CheckedRayBundle, Ray, RayBundle
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


PACKAGE = Path(__file__).resolve().parents[1]
SCENARIO_ID = "S05_cluttered_five_target_stress"
SCENARIO_PROFILE = "s05"
SEMANTIC_CONTRACT_ID = "mid360-multi-uav-s05-cluttered-stress-v1"
TARGET_NAMES = ("uav2", "uav3", "uav4", "uav5", "uav6")
FAMILY_URIS = {
    "gate": "model://gate_frame",
    "pillar": "model://pillar",
    "wide_wall": "model://sparse_wide_wall",
}


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
    """Rotate a vector without an optional transformations dependency."""
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


def _parse_pose(text, label):
    values = tuple(float(value) for value in (text or "0 0 0 0 0 0").split())
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise ValueError("{} must contain six finite pose values".format(label))
    if abs(values[3]) > 1.0e-12 or abs(values[4]) > 1.0e-12:
        raise ValueError("{} has unsupported nonzero roll/pitch".format(label))
    return (values[0], values[1], values[2], values[5])


def _compose_pose(parent, child):
    cosine = math.cos(parent[3])
    sine = math.sin(parent[3])
    return (
        parent[0] + cosine * child[0] - sine * child[1],
        parent[1] + sine * child[0] + cosine * child[1],
        parent[2] + child[2],
        parent[3] + child[3],
    )


def _surface_error(point, primitive, tolerance):
    """Return distance to a parsed primitive surface, or None if out of span."""
    dx = point[0] - primitive["center"][0]
    dy = point[1] - primitive["center"][1]
    dz = point[2] - primitive["center"][2]
    cosine = primitive["cos_yaw"]
    sine = primitive["sin_yaw"]
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    if primitive["kind"] == "box":
        half_x, half_y, half_z = primitive["half_extents"]
        coordinates = (local_x, local_y, dz)
        halves = (half_x, half_y, half_z)
        if any(abs(value) > half + tolerance
               for value, half in zip(coordinates, halves)):
            return None
        return min(abs(abs(value) - half)
                   for value, half in zip(coordinates, halves))
    if primitive["kind"] == "cylinder":
        radial = math.hypot(local_x, local_y)
        radius = primitive["radius"]
        half_length = primitive["half_length"]
        errors = []
        if abs(dz) <= half_length + tolerance:
            errors.append(abs(radial - radius))
        if radial <= radius + tolerance:
            errors.append(abs(abs(dz) - half_length))
        return min(errors) if errors else None
    raise ValueError("unsupported primitive kind {}".format(primitive["kind"]))


def _load_e3_primitives():
    """Derive world collision geometry from E3 includes and model SDF files."""
    world_path = PACKAGE / "worlds" / "E3_cluttered.world"
    world = ET.parse(str(world_path)).getroot().find("world")
    if world is None or world.attrib.get("name") != "E3_cluttered":
        raise ValueError("frozen E3 world identity changed")
    includes = world.findall("include")
    include_names = [item.findtext("name") for item in includes]
    if len(include_names) != len(set(include_names)):
        raise ValueError("E3 model instance names are not unique")

    result = {family: [] for family in FAMILY_URIS}
    for family, required_uri in FAMILY_URIS.items():
        family_includes = [
            item for item in includes if item.findtext("uri") == required_uri
        ]
        if not family_includes:
            raise ValueError("E3 lacks required {} primitive family".format(family))
        model_name = required_uri[len("model://"):]
        model = ET.parse(
            str(PACKAGE / "models" / model_name / "model.sdf")
        ).getroot().find("model")
        if model is None or model.findtext("static") != "true":
            raise ValueError("{} model is not explicitly static".format(family))
        model_pose = _parse_pose(model.findtext("pose"), family + " model pose")
        links = model.findall("link")
        if not links:
            raise ValueError("{} model has no links".format(family))

        for include in family_includes:
            include_name = include.findtext("name")
            include_pose = _parse_pose(
                include.findtext("pose"), include_name + " include pose"
            )
            model_world_pose = _compose_pose(include_pose, model_pose)
            for link in links:
                link_pose = _parse_pose(
                    link.findtext("pose"), include_name + " link pose"
                )
                link_world_pose = _compose_pose(model_world_pose, link_pose)
                for collision in link.findall("collision"):
                    collision_pose = _parse_pose(
                        collision.findtext("pose"),
                        include_name + " collision pose",
                    )
                    world_pose = _compose_pose(link_world_pose, collision_pose)
                    geometry = collision.find("geometry")
                    children = list(geometry) if geometry is not None else []
                    if len(children) != 1 or children[0].tag not in (
                        "box", "cylinder"
                    ):
                        raise ValueError(
                            "{} collision is not one supported primitive".format(
                                include_name
                            )
                        )
                    primitive = {
                        "family": family,
                        "instance": include_name,
                        "collision": collision.attrib.get("name", ""),
                        "kind": children[0].tag,
                        "center": world_pose[:3],
                        "yaw": world_pose[3],
                        "cos_yaw": math.cos(world_pose[3]),
                        "sin_yaw": math.sin(world_pose[3]),
                    }
                    if children[0].tag == "box":
                        size = tuple(
                            float(value)
                            for value in children[0].findtext("size").split()
                        )
                        if len(size) != 3 or any(value <= 0.0 for value in size):
                            raise ValueError("invalid {} box size".format(family))
                        primitive["half_extents"] = tuple(
                            value * 0.5 for value in size
                        )
                    else:
                        radius = float(children[0].findtext("radius"))
                        length = float(children[0].findtext("length"))
                        if radius <= 0.0 or length <= 0.0:
                            raise ValueError("invalid {} cylinder size".format(family))
                        primitive["radius"] = radius
                        primitive["half_length"] = length * 0.5
                    result[family].append(primitive)

    expected_counts = {"gate": 6, "pillar": 4, "wide_wall": 3}
    actual_counts = {name: len(values) for name, values in result.items()}
    if actual_counts != expected_counts:
        raise ValueError(
            "frozen E3 collision counts changed: {} != {}".format(
                actual_counts, expected_counts
            )
        )
    return result


class GateS05Contract(unittest.TestCase):

    def setUp(self):
        self.timeout_wall_sec = float(rospy.get_param("~timeout_wall_sec", 220.0))
        self.post_complete_timeout_wall_sec = float(
            rospy.get_param("~post_complete_timeout_wall_sec", 12.0)
        )
        self.expected_ray_count = int(rospy.get_param("~expected_ray_count", 20000))
        self.maximum_tf_gap_sec = float(
            rospy.get_param("~maximum_tf_gap_sec", 0.20)
        )
        self.minimum_scored_translation_m = float(
            rospy.get_param("~minimum_scored_translation_m", 5.0)
        )
        self.minimum_scored_yaw_rad = float(
            rospy.get_param("~minimum_scored_yaw_rad", 0.70)
        )
        self.minimum_scored_bundles = int(
            rospy.get_param("~minimum_scored_bundles", 3)
        )
        self.maximum_geometry_scans = int(
            rospy.get_param("~maximum_geometry_scans", 8)
        )
        self.minimum_geometry_scans = int(
            rospy.get_param("~minimum_geometry_scans", 3)
        )
        self.minimum_hit_counts = {
            "gate": int(rospy.get_param("~minimum_gate_hit_count", 5)),
            "pillar": int(rospy.get_param("~minimum_pillar_hit_count", 5)),
            "wide_wall": int(
                rospy.get_param("~minimum_wide_wall_hit_count", 5)
            ),
        }
        self.geometry_tolerance_m = float(
            rospy.get_param("~geometry_tolerance_m", 0.04)
        )
        self.minimum_bundle_translation_m = float(
            rospy.get_param("~minimum_bundle_translation_m", 0.060)
        )
        self.maximum_bundle_translation_m = float(
            rospy.get_param("~maximum_bundle_translation_m", 0.080)
        )
        self.minimum_bundle_yaw_rad = float(
            rospy.get_param("~minimum_bundle_yaw_rad", 0.007)
        )
        self.maximum_bundle_yaw_rad = float(
            rospy.get_param("~maximum_bundle_yaw_rad", 0.012)
        )
        if (
            self.timeout_wall_sec <= 0.0
            or self.post_complete_timeout_wall_sec <= 0.0
            or self.expected_ray_count != 20000
            or self.maximum_tf_gap_sec <= 0.0
            or self.maximum_tf_gap_sec > 0.20
            or self.minimum_scored_translation_m < 5.0
            or self.minimum_scored_yaw_rad < 0.70
            or self.minimum_scored_bundles < 3
            or self.minimum_geometry_scans < 3
            or self.maximum_geometry_scans < self.minimum_geometry_scans
            or any(value < 5 for value in self.minimum_hit_counts.values())
            or self.geometry_tolerance_m <= 0.0
            or self.geometry_tolerance_m > 0.04
            or self.minimum_bundle_translation_m < 0.060
            or self.maximum_bundle_translation_m > 0.080
            or self.maximum_bundle_translation_m <= self.minimum_bundle_translation_m
            or self.minimum_bundle_yaw_rad < 0.007
            or self.maximum_bundle_yaw_rad > 0.012
            or self.maximum_bundle_yaw_rad <= self.minimum_bundle_yaw_rad
        ):
            raise ValueError("S05 live-gate parameters weaken the frozen contract")

        self.primitives = _load_e3_primitives()
        self.lock = threading.RLock()
        self.failure = None
        self.complete_event = threading.Event()
        self.complete_sim_time = None
        self.cycle_start_sim_time = None
        self.stress_start_sim_time = None
        self.stress_end_sim_time = None
        self.events = []

        self.model_samples = {
            name: [] for name in ("uav1",) + TARGET_NAMES
        }
        self.raw_summaries = {}
        self.source_diagnostics = {}
        self.preprocessor_diagnostics = {}
        self.best_scored_checked_bundle = None
        self.best_scored_checked_motion = -math.inf
        self.scored_checked_count = 0
        self.scored_ray_tf_summaries = []
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
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(45.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def _set_failure(self, error):
        with self.lock:
            if self.failure is None:
                self.failure = str(error)
                rospy.logerr("S05 contract callback failed: %s", self.failure)
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
        hits = {family: 0 for family in self.primitives}
        errors = {family: [] for family in self.primitives}
        for checked in message.rays:
            source = checked.source
            if source.return_status != Ray.VALID_RETURN:
                continue
            endpoint = (
                checked.origin.x + source.range * checked.direction.x,
                checked.origin.y + source.range * checked.direction.y,
                checked.origin.z + source.range * checked.direction.z,
            )
            for family, primitives in self.primitives.items():
                candidates = [
                    error for error in (
                        _surface_error(
                            endpoint, primitive, self.geometry_tolerance_m
                        )
                        for primitive in primitives
                    )
                    if error is not None and error <= self.geometry_tolerance_m
                ]
                if candidates:
                    hits[family] += 1
                    errors[family].append(min(candidates))
        return {
            "scan_id": int(message.scan_id),
            "hits": hits,
            "maximum_errors_m": {
                family: max(values) if values else None
                for family, values in errors.items()
            },
        }

    @staticmethod
    def _ray_tf_summary(message):
        samples = []
        for index in (0, len(message.rays) - 1):
            checked = message.rays[index]
            samples.append({
                "offset_time_ns": int(checked.source.offset_time_ns),
                "source_direction": (
                    float(checked.source.dir_x),
                    float(checked.source.dir_y),
                    float(checked.source.dir_z),
                ),
                "checked_origin": (
                    float(checked.origin.x),
                    float(checked.origin.y),
                    float(checked.origin.z),
                ),
                "checked_direction": (
                    float(checked.direction.x),
                    float(checked.direction.y),
                    float(checked.direction.z),
                ),
            })
        return {
            "key": _bundle_key(message.source_header),
            "stamp": message.source_header.stamp,
            "samples": tuple(samples),
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
            origin_delta = _norm((
                message.rays[-1].origin.x - message.rays[0].origin.x,
                message.rays[-1].origin.y - message.rays[0].origin.y,
                message.rays[-1].origin.z - message.rays[0].origin.z,
            ))
            with self.lock:
                start = self.stress_start_sim_time
                end = self.stress_end_sim_time
                geometry_slots = len(self.geometry_summaries)
                complete_time = self.complete_sim_time
            is_scored = (
                start is not None
                and stamp_sec >= start + 0.25
                and stamp_sec + duration_sec <= start + 7.75
                and (end is None or stamp_sec + duration_sec < end - 0.25)
            )
            geometry_summary = None
            if is_scored and geometry_slots < self.maximum_geometry_scans:
                geometry_summary = self._summarize_geometry(message)
            ray_tf_summary = self._ray_tf_summary(message) if is_scored else None

            with self.lock:
                if is_scored:
                    self.scored_checked_count += 1
                    self.scored_ray_tf_summaries.append(ray_tf_summary)
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
                elif event["event"] == "stress_window_start":
                    self.stress_start_sim_time = float(event["sim_time"])
                elif event["event"] == "stress_window_end":
                    self.stress_end_sim_time = float(event["sim_time"])
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
                    matches.append((
                        transform.header.stamp.to_sec(),
                        float(transform.transform.translation.x),
                        float(transform.transform.translation.y),
                        float(transform.transform.translation.z),
                        _yaw(transform.transform.rotation),
                    ))
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
                "scored_ray_tf_summaries": list(self.scored_ray_tf_summaries),
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
        latest = self._snapshot()
        self.fail(
            "S05 runtime evidence timed out after {:.1f}s; events={} raw={} "
            "scored_checked={} tf={} geometry={}".format(
                self.timeout_wall_sec,
                [event.get("event") for event in latest["events"]],
                len(latest["raw_summaries"]),
                latest["scored_checked_count"],
                len(latest["dynamic_tf_samples"]),
                len(latest["geometry_summaries"]),
            )
        )

    @staticmethod
    def _assert_pose(test, actual, expected, label):
        test.assertEqual(set(actual), {"x", "y", "z", "yaw"}, label)
        for key, value in expected.items():
            test.assertAlmostEqual(float(actual[key]), value, places=9, msg=label)

    def _check_loaded_schema(self):
        scenario_path = Path(rospy.get_param("/scenario_manager/scenario_file"))
        document = yaml.safe_load(scenario_path.read_text(encoding="utf-8"))
        self.assertEqual(scenario_path.name, "S05.yaml")
        self.assertEqual(document["schema_version"], 5)
        self.assertEqual(document["scenario_profile"], SCENARIO_PROFILE)
        self.assertEqual(document["semantic_contract_id"], SEMANTIC_CONTRACT_ID)
        self.assertEqual(document["scenario"], SCENARIO_ID)
        self.assertEqual(document["seed"], 808)
        self.assertEqual(document["repeat_count"], 1)
        self.assertEqual(document["repeat_mode"], "restart")

    def _check_events(self, snapshot):
        expected_names = [
            "cycle_start",
            "initial_stress_end",
            "stress_window_start",
            "stress_window_end",
            "final_stress_start",
            "cycle_complete",
            "final_hold_start",
            "scenario_complete",
        ]
        self.assertEqual(
            [event["event"] for event in snapshot["events"]], expected_names,
            "S05 event stream is missing, duplicated, reordered, or contains failure",
        )
        start_ns = int(snapshot["events"][0]["sim_time_ns"])
        expected_offsets_ns = (
            0, 4_000_000_000, 9_000_000_000, 17_000_000_000,
            22_000_000_000, 24_000_000_000, 24_000_000_000,
            27_000_000_000,
        )
        observer_poses = (
            (-4.0, -5.0, 2.0, -0.35),
            (-4.0, -5.0, 2.0, -0.35),
            (-1.0, -2.0, 2.0, 0.0),
            (3.0, 2.0, 2.0, 0.75),
            (5.0, -3.0, 2.0, -0.4),
            (5.0, -3.0, 2.0, -0.4),
            (5.0, -3.0, 2.0, -0.4),
            (5.0, -3.0, 2.0, -0.4),
        )
        target_poses = {
            "uav2": {"x": 7.0, "y": -4.0, "z": 2.25, "yaw": 0.0},
            "uav3": {"x": 10.0, "y": -1.0, "z": 2.25, "yaw": 0.0},
            "uav4": {"x": 12.0, "y": 3.0, "z": 2.25, "yaw": 0.0},
            "uav5": {"x": 7.0, "y": 5.0, "z": 2.25, "yaw": 0.0},
            "uav6": {"x": 15.0, "y": 0.8, "z": 2.25, "yaw": 0.0},
        }
        for index, event in enumerate(snapshot["events"]):
            self.assertEqual(event["scenario"], SCENARIO_ID)
            self.assertEqual(event["scenario_profile"], SCENARIO_PROFILE)
            self.assertEqual(event["semantic_contract_id"], SEMANTIC_CONTRACT_ID)
            self.assertEqual(event["seed"], 808)
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
                set(event["commanded_pose"]),
                {"uav1", "uav2", "uav3", "uav4", "uav5", "uav6"},
            )
            observer = observer_poses[index]
            self._assert_pose(
                self,
                event["commanded_pose"]["uav1"],
                {"x": observer[0], "y": observer[1],
                 "z": observer[2], "yaw": observer[3]},
                "{} uav1 boundary pose".format(event["event"]),
            )
            for name, pose in target_poses.items():
                self._assert_pose(
                    self, event["commanded_pose"][name], pose,
                    "{} {} boundary pose".format(event["event"], name),
                )

        phase_contract = {
            "cycle_start": ("initial_stress_hold", 0),
            "initial_stress_end": ("enter_clutter", 1),
            "stress_window_start": ("peak_stress_scored", 2),
            "stress_window_end": ("exit_clutter", 3),
            "final_stress_start": ("final_stress_hold", 4),
            "cycle_complete": ("final_stress_hold", 4),
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
            "uav2": (7.0, -4.0, 2.25, 0.0),
            "uav3": (10.0, -1.0, 2.25, 0.0),
            "uav4": (12.0, 3.0, 2.25, 0.0),
            "uav5": (7.0, 5.0, 2.25, 0.0),
            "uav6": (15.0, 0.8, 2.25, 0.0),
        }
        for name, expected in expected_targets.items():
            samples = snapshot["model_samples"][name]
            for axis, label in ((1, "x"), (2, "y"), (3, "z")):
                values = [sample[axis] for sample in samples]
                self.assertLess(
                    max(values) - min(values), 0.04,
                    "{} {} moved".format(name, label),
                )
                self.assertLess(
                    max(abs(value - expected[axis - 1]) for value in values),
                    0.05,
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
            "actual uav1 altitude changed during the S05 trajectory",
        )
        boundary_expectations = {
            "cycle_start": (-4.0, -5.0, 2.0, -0.35),
            "stress_window_start": (-1.0, -2.0, 2.0, 0.0),
            "stress_window_end": (3.0, 2.0, 2.0, 0.75),
            "final_stress_start": (5.0, -3.0, 2.0, -0.4),
            "scenario_complete": (5.0, -3.0, 2.0, -0.4),
        }
        for name, expected in boundary_expectations.items():
            sample = self._nearest_model_sample(
                observer, float(events[name]["sim_time"])
            )
            self.assertLess(
                _norm(tuple(sample[index + 1] - expected[index]
                            for index in range(3))),
                0.12, name,
            )
            self.assertLess(abs(_angle_error(sample[4], expected[3])), 0.035, name)

        scored_start = float(events["stress_window_start"]["sim_time"])
        scored_end = float(events["stress_window_end"]["sim_time"])
        scored = [
            sample for sample in observer
            if scored_start + 0.15 <= sample[0] <= scored_end - 0.15
        ]
        self.assertGreater(len(scored), 100, "actual scored motion is undersampled")
        self.assertGreater(
            max(sample[1] for sample in scored) - min(sample[1] for sample in scored),
            3.75,
        )
        self.assertGreater(
            max(sample[2] for sample in scored) - min(sample[2] for sample in scored),
            3.75,
        )
        yaw_span = max(sample[4] for sample in scored) - min(
            sample[4] for sample in scored
        )
        self.assertGreater(yaw_span, 0.70)
        self.assertGreater(max(sample[5] for sample in scored), 0.65)
        self.assertLess(max(sample[5] for sample in scored), 0.76)
        self.assertGreater(max(sample[6] for sample in scored), 0.08)
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
        samples = sorted(
            {sample[0]: sample for sample in snapshot["dynamic_tf_samples"]}.values(),
            key=lambda sample: sample[0],
        )
        self.assertGreater(len(samples), 650, "dynamic TF cadence is too sparse")
        gaps = [second[0] - first[0] for first, second in zip(samples, samples[1:])]
        self.assertTrue(gaps)
        self.assertGreater(min(gaps), 0.0)
        self.assertLess(max(gaps), self.maximum_tf_gap_sec)
        self.assertGreater(
            samples[-1][0], snapshot["complete_sim_time"] + 1.0e-3,
            "dynamic TF did not remain current after scenario_complete",
        )

        scored_start = float(events["stress_window_start"]["sim_time"])
        scored_end = float(events["stress_window_end"]["sim_time"])
        scored = [sample for sample in samples if scored_start <= sample[0] <= scored_end]
        self.assertGreater(len(scored), 250, "scored dynamic TF is undersampled")
        first = min(scored, key=lambda item: abs(item[0] - scored_start))
        last = min(scored, key=lambda item: abs(item[0] - scored_end))
        translation = _norm((
            last[1] - first[1], last[2] - first[2], last[3] - first[3]
        ))
        yaw_change = abs(_angle_error(last[4], first[4]))
        self.assertGreaterEqual(translation, self.minimum_scored_translation_m)
        self.assertGreaterEqual(yaw_change, self.minimum_scored_yaw_rad)
        self.assertLess(abs(first[1] + 1.0), 0.05)
        self.assertLess(abs(first[2] + 2.0), 0.05)
        self.assertLess(abs(first[4]), 0.02)
        self.assertLess(abs(last[1] - 3.0), 0.05)
        self.assertLess(abs(last[2] - 2.0), 0.05)
        self.assertLess(abs(_angle_error(last[4], 0.75)), 0.02)
        midpoint = min(scored, key=lambda item: abs(item[0] - (scored_start + 4.0)))
        self.assertLess(abs(midpoint[1] - 1.0), 0.06)
        self.assertLess(abs(midpoint[2]), 0.06)
        self.assertLess(abs(_angle_error(midpoint[4], 0.375)), 0.025)

    def _check_ray_contract(self, snapshot):
        checked = snapshot["best_checked"]
        self.assertIsNotNone(checked, "no complete scored checked bundle was received")
        self.assertGreaterEqual(
            snapshot["scored_checked_count"], self.minimum_scored_bundles
        )
        self.assertEqual(checked.header.frame_id, "world")
        self.assertEqual(checked.source_header.frame_id, "uav1/mid360_link")
        self.assertEqual(checked.source_mode, "sim_exact")
        self.assertEqual(checked.ray_time_geometry_mode, "per_ray_pose")
        self.assertEqual(len(checked.rays), self.expected_ray_count)
        self.assertGreaterEqual(
            snapshot["best_checked_motion"], self.minimum_bundle_translation_m
        )
        self.assertLessEqual(
            snapshot["best_checked_motion"], self.maximum_bundle_translation_m
        )

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
                abs(_norm((checked_ray.direction.x, checked_ray.direction.y,
                           checked_ray.direction.z)) - 1.0),
            )
            self.assertIn(
                int(source.return_status),
                (Ray.NO_RETURN, Ray.VALID_RETURN, Ray.BELOW_MIN_RANGE,
                 Ray.INVALID_RANGE, Ray.UNKNOWN_STATUS),
            )
            statuses[int(source.return_status)] = (
                statuses.get(int(source.return_status), 0) + 1
            )
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
        self.assertEqual(
            int(source_values["valid_return_count"]),
            statuses.get(Ray.VALID_RETURN, 0),
        )
        self.assertEqual(
            int(source_values["no_return_count"]),
            statuses.get(Ray.NO_RETURN, 0),
        )
        self.assertEqual(
            int(source_values["below_min_range_count"]),
            statuses.get(Ray.BELOW_MIN_RANGE, 0),
        )
        invalid_count = self.expected_ray_count - sum(
            statuses.get(status, 0)
            for status in (Ray.VALID_RETURN, Ray.NO_RETURN, Ray.BELOW_MIN_RANGE)
        )
        self.assertEqual(int(source_values["invalid_count"]), invalid_count)
        self.assertLess(float(source_values["direction_norm_error_max"]), 1.0e-4)
        self.assertAlmostEqual(
            float(source_values["timestamp_monotonic_ratio"]), 1.0
        )
        self.assertAlmostEqual(
            float(source_values["bundle_duration"]), duration_sec, places=5
        )
        self.assertGreater(float(source_values["linear_speed_mps"]), 0.65)
        self.assertLess(float(source_values["linear_speed_mps"]), 0.76)
        self.assertGreater(float(source_values["angular_speed_radps"]), 0.08)
        self.assertLess(float(source_values["angular_speed_radps"]), 0.11)
        self.assertAlmostEqual(
            float(source_values["max_offset_sec"]), duration_sec, places=5
        )

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
        self.assertEqual(
            int(values["valid_return_count"]), statuses.get(Ray.VALID_RETURN, 0)
        )
        self.assertEqual(
            int(values["no_return_count"]), statuses.get(Ray.NO_RETURN, 0)
        )
        self.assertEqual(values["tf_source_cadence_observation_enabled"], "true")
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
        self.assertEqual(values["tf_source_sample_bracket_status"], "bracketed")
        self.assertEqual(values["tf_source_sample_bracketed"], "true")
        observed_gap = float(values["tf_source_sample_max_gap_sec"])
        self.assertGreater(observed_gap, 0.0)
        self.assertLess(observed_gap, self.maximum_tf_gap_sec)
        self.assertGreaterEqual(int(values["tf_source_sample_covering_count"]), 2)
        query_start_ns = checked.source_header.stamp.to_nsec()
        query_end_ns = query_start_ns + raw["duration_ns"]
        self.assertLessEqual(
            int(values["tf_source_sample_left_bracket_ns"]), query_start_ns
        )
        self.assertGreaterEqual(
            int(values["tf_source_sample_right_bracket_ns"]), query_end_ns
        )
        self.assertEqual(
            values["tf_max_interval_semantics"],
            "deprecated_alias_of_tf_query_span_sec",
        )
        self.assertEqual(int(values["tf_lookup_count"]), 2)
        self.assertGreater(float(values["tf_query_span_sec"]), 0.095)
        self.assertLess(float(values["tf_query_span_sec"]), 0.105)
        self.assertGreater(snapshot["post_cycle_source_ok_count"], 100)
        self.assertGreater(snapshot["post_cycle_preprocessor_ok_count"], 100)
        self.assertTrue(snapshot["post_complete_checked_stamps"])

    def _check_every_scored_bundle_tf(self, snapshot):
        summaries = snapshot["scored_ray_tf_summaries"]
        self.assertEqual(len(summaries), snapshot["scored_checked_count"])
        self.assertGreaterEqual(len(summaries), self.minimum_scored_bundles)
        self.assertEqual(len({item["key"] for item in summaries}), len(summaries))
        translations = []
        yaw_changes = []
        for summary in summaries:
            transforms = []
            for sample in summary["samples"]:
                stamp = summary["stamp"] + rospy.Duration.from_sec(
                    sample["offset_time_ns"] * 1.0e-9
                )
                transform = self.tf_buffer.lookup_transform(
                    "world", "uav1/mid360_link", stamp, rospy.Duration(2.0)
                )
                transforms.append(transform)
                translation = transform.transform.translation
                self.assertLess(
                    _norm(tuple(
                        sample["checked_origin"][index]
                        - (translation.x, translation.y, translation.z)[index]
                        for index in range(3)
                    )),
                    0.025,
                    "checked per-ray origin disagrees with timestamped TF",
                )
                expected_direction = _rotate(
                    transform.transform.rotation, sample["source_direction"]
                )
                self.assertLess(
                    _norm(tuple(
                        sample["checked_direction"][index]
                        - expected_direction[index]
                        for index in range(3)
                    )),
                    1.0e-4,
                    "checked per-ray direction disagrees with timestamped TF",
                )
            first = transforms[0].transform
            last = transforms[-1].transform
            translation = _norm((
                last.translation.x - first.translation.x,
                last.translation.y - first.translation.y,
                last.translation.z - first.translation.z,
            ))
            yaw_change = abs(_angle_error(
                _yaw(last.rotation), _yaw(first.rotation)
            ))
            translations.append(translation)
            yaw_changes.append(yaw_change)
            self.assertGreaterEqual(translation, self.minimum_bundle_translation_m)
            self.assertLessEqual(translation, self.maximum_bundle_translation_m)
            self.assertGreaterEqual(yaw_change, self.minimum_bundle_yaw_rad)
            self.assertLessEqual(yaw_change, self.maximum_bundle_yaw_rad)
        rospy.loginfo(
            "S05 per-bundle first/last TF: bundles=%d translation=[%.6f, %.6f] "
            "yaw=[%.6f, %.6f]",
            len(summaries), min(translations), max(translations),
            min(yaw_changes), max(yaw_changes),
        )

    def _check_e3_geometric_returns(self, snapshot):
        summaries = snapshot["geometry_summaries"]
        self.assertGreaterEqual(len(summaries), self.minimum_geometry_scans)
        totals = {
            family: sum(item["hits"][family] for item in summaries)
            for family in self.primitives
        }
        for family, minimum in self.minimum_hit_counts.items():
            self.assertGreaterEqual(
                totals[family], minimum,
                "no adequate endpoints lay on parsed E3 {} primitive surfaces".format(
                    family
                ),
            )
            self.assertTrue(any(item["hits"][family] for item in summaries))
        rospy.loginfo(
            "S05 E3 endpoint geometry (not semantic labels): scans=%d gate=%d "
            "pillar=%d wide_wall=%d",
            len(summaries), totals["gate"], totals["pillar"],
            totals["wide_wall"],
        )

    def _check_truth_isolation(self):
        _publishers, subscribers, _services = rosgraph.Master(
            rospy.get_name()
        ).getSystemState()
        subscriber_map = dict(subscribers)
        consumers = set(subscriber_map.get("/gazebo/model_states", []))
        self.assertIn(rospy.get_name(), consumers)
        self.assertEqual(
            sorted(consumers - {rospy.get_name()}), [],
            "production nodes illegally consume /gazebo/model_states",
        )
        for name in ("uav1",) + TARGET_NAMES:
            topic = "/mid360_multi_uav_sim/ground_truth/{}/odom".format(name)
            self.assertEqual(
                subscriber_map.get(topic, []), [],
                "production nodes illegally consume {}".format(topic),
            )

    def test_gate_s05_live_contract(self):
        snapshot = self._wait_for_runtime_evidence()
        self.assertIsNone(snapshot["failure"], snapshot["failure"])
        self._check_loaded_schema()
        events = self._check_events(snapshot)
        self._check_actual_motion(snapshot, events)
        self._check_tf_contract(snapshot, events)
        self._check_ray_contract(snapshot)
        self._check_every_scored_bundle_tf(snapshot)
        self._check_e3_geometric_returns(snapshot)
        self._check_truth_isolation()


if __name__ == "__main__":
    rospy.init_node("gate_s05_contract")
    rostest.rosrun(
        "mid360_multi_uav_sim", "gate_s05_contract", GateS05Contract
    )
