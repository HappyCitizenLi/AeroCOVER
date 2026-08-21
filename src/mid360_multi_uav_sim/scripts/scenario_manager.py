#!/usr/bin/env python3
"""Deterministic Gazebo authority for the retained Mid-360 scenarios and S1.

The node is deliberately an open-loop *scenario authority*: target poses are
computed from simulation time, written through ``/gazebo/set_model_state``, and
published as timestamped commanded ground truth.  It never subscribes to
``/gazebo/model_states``.  This keeps Gazebo truth out of the perception graph
and makes runs independent of ROS callback/update jitter.

Events are latched, stable JSON strings.  Every boundary event is emitted only
after the matching target state and ground-truth odometry have been published.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import sys
import threading
import time
from dataclasses import dataclass, replace
from typing import Dict, List, Mapping, Optional, Tuple

import rospy
import yaml
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import (
    GetWorldProperties,
    SetModelState,
    SetModelStateRequest,
)
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
import tf2_ros


NSEC_PER_SEC = 1_000_000_000
ALLOWED_INTERPOLATIONS = (
    "hold",
    "linear",
    "smoothstep",
    "minimum_snap",
)
ALLOWED_REPEAT_MODES = ("restart", "ping_pong")
S03_SCENARIO_PROFILE = "s03"
S03_SCENARIO_ID = "S03_moving_observer_translation"
S03_SEMANTIC_CONTRACT_ID = "mid360-multi-uav-s03-observer-motion-v1"
DYNAMIC_TF_AUTHORITY = "scenario_manager"
S03_DYNAMIC_TF_AUTHORITY = DYNAMIC_TF_AUTHORITY
S04_SCENARIO_PROFILE = "s04"
S04_SCENARIO_ID = "S04_combined_visibility_translate_yaw"
S04_SEMANTIC_CONTRACT_ID = "mid360-multi-uav-s04-combined-visibility-v1"
S04_DYNAMIC_TF_AUTHORITY = DYNAMIC_TF_AUTHORITY
S04_SEED = 707
S04_START_DELAY_NS = 5 * NSEC_PER_SEC
S04_INITIAL_CLEAR_HOLD_NS = 3 * NSEC_PER_SEC
S04_APPROACH_OCCLUSION_NS = 4 * NSEC_PER_SEC
S04_SCORED_MOTION_NS = 5 * NSEC_PER_SEC
S04_LEAVE_OCCLUSION_NS = 5 * NSEC_PER_SEC
S04_FINAL_CLEAR_HOLD_NS = 3 * NSEC_PER_SEC
S04_FINAL_HOLD_NS = 3 * NSEC_PER_SEC
S04_MINIMUM_TRANSLATION_M = 2.0
S04_MINIMUM_YAW_CHANGE_RAD = 0.35
S05_SCENARIO_PROFILE = "s05"
S05_SCENARIO_ID = "S05_cluttered_five_target_stress"
S05_SEMANTIC_CONTRACT_ID = "mid360-multi-uav-s05-cluttered-stress-v1"
S05_DYNAMIC_TF_AUTHORITY = DYNAMIC_TF_AUTHORITY
S05_SEED = 808
S05_START_DELAY_NS = 5 * NSEC_PER_SEC
S05_INITIAL_STRESS_HOLD_NS = 4 * NSEC_PER_SEC
S05_ENTER_CLUTTER_NS = 5 * NSEC_PER_SEC
S05_PEAK_STRESS_NS = 8 * NSEC_PER_SEC
S05_EXIT_CLUTTER_NS = 5 * NSEC_PER_SEC
S05_FINAL_STRESS_HOLD_NS = 2 * NSEC_PER_SEC
S05_FINAL_HOLD_NS = 3 * NSEC_PER_SEC
S05_MINIMUM_TRANSLATION_M = 5.0
S05_MINIMUM_YAW_CHANGE_RAD = 0.7
S06_SCENARIO_PROFILE = "s06"
S06_SCENARIO_ID = "S06_incomplete_map_online_occlusion"
S06_SEMANTIC_CONTRACT_ID = "mid360-multi-uav-s06-online-map-occlusion-v1"
S06_DYNAMIC_TF_AUTHORITY = DYNAMIC_TF_AUTHORITY
S06_SEED = 909
S06_START_DELAY_NS = 5 * NSEC_PER_SEC
S06_MAP_BOOTSTRAP_CLEAR_HOLD_NS = 4 * NSEC_PER_SEC
S06_APPROACH_ONLINE_OCCLUSION_NS = 4 * NSEC_PER_SEC
S06_ONLINE_OCCLUSION_NS = 6 * NSEC_PER_SEC
S06_LEAVE_ONLINE_OCCLUSION_NS = 5 * NSEC_PER_SEC
S06_FINAL_CLEAR_HOLD_NS = 3 * NSEC_PER_SEC
S06_FINAL_HOLD_NS = 3 * NSEC_PER_SEC
S06_MINIMUM_TRANSLATION_M = 2.0
S06_MINIMUM_YAW_CHANGE_RAD = 0.35
S02_SCENARIO_PROFILE = "s02"
S02_SCENARIO_ID = "S02_ten_uav_minimum_snap_crossing_partial_occlusion"
S02_SEMANTIC_CONTRACT_ID = (
    "mid360-multi-uav-s02-crossing-partial-occlusion-v3"
)
S02_SCENARIO_CONTRACT_ID = "ten-uav-c3-dynamics-and-avoidance-v2"

S03_ROOT_KEYS = {
    "schema_version",
    "scenario_profile",
    "semantic_contract_id",
    "scenario",
    "world_frame",
    "seed",
    "repeat_count",
    "repeat_mode",
    "start_delay",
    "duration",
    "final_hold",
    "runtime",
    "randomization",
    "observer_motion",
    "targets",
    "phases",
}
S03_RUNTIME_KEYS = {
    "set_model_state_service",
    "world_properties_service",
    "event_topic",
    "publish_rate",
    "service_timeout_wall",
    "clock_timeout_wall",
    "max_clock_stall_wall",
    "max_consecutive_service_failures",
    "require_use_sim_time",
}
S03_TARGET_KEYS = {
    "model_name",
    "command_model",
    "truth_topic",
    "child_frame_id",
    "initial_pose",
}
S03_PHASE_REQUIRED_KEYS = {"id", "duration", "interpolation"}
S03_PHASE_OPTIONAL_KEYS = {"event", "targets"}
S03_OBSERVER_MOTION_KEYS = {
    "observer",
    "parent_frame",
    "child_frame",
    "tf_authority",
    "unique_dynamic_tf_authority",
    "fixed_yaw",
    "minimum_translation_m",
    "scored_phase",
    "stationary_targets",
}
S04_OBSERVER_MOTION_KEYS = {
    "observer",
    "parent_frame",
    "child_frame",
    "tf_authority",
    "unique_dynamic_tf_authority",
    "minimum_translation_m",
    "minimum_yaw_change_rad",
    "scored_phase",
    "stationary_targets",
}
S05_ROOT_KEYS = {
    "schema_version",
    "scenario_profile",
    "semantic_contract_id",
    "scenario",
    "world_frame",
    "seed",
    "repeat_count",
    "repeat_mode",
    "start_delay",
    "duration",
    "final_hold",
    "runtime",
    "randomization",
    "observer_motion",
    "targets",
    "phases",
}
S05_OBSERVER_MOTION_KEYS = {
    "observer",
    "parent_frame",
    "child_frame",
    "tf_authority",
    "unique_dynamic_tf_authority",
    "minimum_translation_m",
    "minimum_yaw_change_rad",
    "scored_phase",
    "stationary_targets",
}
S06_ROOT_KEYS = {
    "schema_version",
    "scenario_profile",
    "semantic_contract_id",
    "scenario",
    "world_frame",
    "seed",
    "repeat_count",
    "repeat_mode",
    "start_delay",
    "duration",
    "final_hold",
    "runtime",
    "randomization",
    "observer_motion",
    "online_map_contract",
    "targets",
    "phases",
}
S06_OBSERVER_MOTION_KEYS = {
    "observer",
    "parent_frame",
    "child_frame",
    "tf_authority",
    "unique_dynamic_tf_authority",
    "minimum_translation_m",
    "minimum_yaw_change_rad",
    "scored_phase",
    "stationary_targets",
}
S06_ONLINE_MAP_CONTRACT_KEYS = {
    "initial_state",
    "apriori_loader_used",
    "initial_map_revision",
    "bootstrap_phase",
    "discovery_primitive",
    "scored_map_occluded_target",
    "transition",
}
S06_ONLINE_MAP_CONTRACT = {
    "initial_state": "empty_online_scores_no_apriori_loader",
    "apriori_loader_used": False,
    "initial_map_revision": 0,
    "bootstrap_phase": "map_bootstrap_clear_hold",
    "discovery_primitive": "background_wall/wall_collision",
    "scored_map_occluded_target": "uav4",
    "transition": "initial_unknown_to_committed_sure_occupied",
}
S02_ROOT_KEYS = {
    "schema_version",
    "scenario_profile",
    "semantic_contract_id",
    "scenario_contract_id",
    "scenario",
    "world_frame",
    "seed",
    "repeat_count",
    "repeat_mode",
    "start_delay",
    "duration",
    "final_hold",
    "runtime",
    "randomization",
    "crossing_contract",
    "targets",
    "phases",
}
S02_CROSSING_CONTRACT_KEYS = {
    "geometry_mode",
    "moving_target_count",
    "trajectory_model",
    "continuity_order",
    "near_radius_m",
    "far_radius_m",
    "line_of_sight_slope",
    "altitude_deconfliction_m",
    "occlusion_center_offset_deg",
    "occlusion_hold_count",
    "exchange_count",
    "max_speed_mps",
    "max_acceleration_mps2",
    "max_jerk_mps3",
    "max_yaw_rate_radps",
    "max_tilt_deg",
    "specific_thrust_min_g",
    "specific_thrust_max_g",
    "conservative_collision_radius_m",
    "minimum_center_separation_m",
    "minimum_conservative_clearance_m",
    "minimum_observer_center_distance_m",
    "minimum_ground_clearance_m",
    "maximum_sensor_range_m",
    "sensor_elevation_min_deg",
    "sensor_elevation_max_deg",
    "semantic_cluster_tolerance_m",
    "maximum_hold_pair_surface_gap_m",
    "occlusion_stages",
}


class ScenarioError(RuntimeError):
    """A configuration or runtime failure that can be reported cleanly."""


@dataclass(frozen=True)
class PoseSpec:
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class MotionState:
    pose: PoseSpec
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0


@dataclass(frozen=True)
class TargetSpec:
    name: str
    model_name: str
    truth_topic: str
    child_frame_id: str
    command_model: bool
    initial_pose: PoseSpec


@dataclass(frozen=True)
class PhaseSpec:
    phase_id: str
    duration_ns: int
    interpolation: str
    goals: Mapping[str, PoseSpec]
    event: Optional[str]


@dataclass(frozen=True)
class RuntimeSpec:
    service_name: str
    world_properties_service: str
    event_topic: str
    publish_rate: float
    service_timeout_wall: float
    clock_timeout_wall: float
    max_clock_stall_wall: float
    max_consecutive_service_failures: int
    require_use_sim_time: bool


@dataclass(frozen=True)
class ObserverMotionSpec:
    observer: str
    parent_frame: str
    child_frame: str
    tf_authority: str
    unique_dynamic_tf_authority: bool
    fixed_yaw: Optional[float]
    minimum_translation_m: float
    scored_phase: str
    stationary_targets: Tuple[str, ...]
    minimum_yaw_change_rad: float = 0.0


@dataclass(frozen=True)
class ScenarioSpec:
    scenario_id: str
    seed: int
    world_frame: str
    repeat_count: int
    repeat_mode: str
    start_delay_ns: int
    final_hold_ns: int
    targets: Mapping[str, TargetSpec]
    phases: Tuple[PhaseSpec, ...]
    phase_starts_ns: Tuple[int, ...]
    cycle_duration_ns: int
    offsets: Mapping[str, PoseSpec]
    runtime: RuntimeSpec
    schema_version: int = 1
    scenario_profile: Optional[str] = None
    semantic_contract_id: Optional[str] = None
    scenario_contract_id: Optional[str] = None
    observer_motion: Optional[ObserverMotionSpec] = None
    online_map_contract: Optional[Mapping[str, object]] = None


def _mapping(value, context: str) -> Mapping:
    if not isinstance(value, dict):
        raise ScenarioError("{} must be a YAML mapping".format(context))
    return value


def _strict_keys(value, required, optional, context: str) -> Mapping:
    item = _mapping(value, context)
    missing = set(required) - set(item)
    unknown = set(item) - set(required) - set(optional)
    if missing:
        raise ScenarioError("{} is missing keys: {}".format(context, sorted(missing)))
    if unknown:
        raise ScenarioError("{} has unknown keys: {}".format(context, sorted(unknown)))
    return item


def _string(value, context: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ScenarioError("{} must be a non-empty string".format(context))
    return value.strip()


def _number(value, context: str, minimum: Optional[float] = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ScenarioError("{} must be a finite number".format(context))
    result = float(value)
    if not math.isfinite(result):
        raise ScenarioError("{} must be finite".format(context))
    if minimum is not None and result < minimum:
        raise ScenarioError("{} must be >= {}".format(context, minimum))
    return result


def _integer(value, context: str, minimum: int = 0) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ScenarioError("{} must be an integer".format(context))
    if value < minimum:
        raise ScenarioError("{} must be >= {}".format(context, minimum))
    return value


def _duration_ns(value, context: str, allow_zero: bool = False) -> int:
    seconds = _number(value, context, 0.0)
    result = int(round(seconds * NSEC_PER_SEC))
    if result < 0 or (result == 0 and not allow_zero):
        qualifier = "non-negative" if allow_zero else "positive"
        raise ScenarioError("{} must have a {} nanosecond duration".format(context, qualifier))
    return result


def _pose(value, context: str) -> PoseSpec:
    item = _mapping(value, context)
    allowed = {"x", "y", "z", "yaw"}
    unknown = set(item.keys()) - allowed
    if unknown:
        raise ScenarioError("{} has unknown keys: {}".format(context, sorted(unknown)))
    missing = allowed - set(item.keys())
    if missing:
        raise ScenarioError("{} is missing keys: {}".format(context, sorted(missing)))
    return PoseSpec(
        x=_number(item["x"], context + ".x"),
        y=_number(item["y"], context + ".y"),
        z=_number(item["z"], context + ".z"),
        yaw=_number(item["yaw"], context + ".yaw"),
    )


def _stable_uniform(seed: int, target: str, axis: str, limit: float) -> float:
    """Version-independent deterministic value in [-limit, limit]."""
    if limit == 0.0:
        return 0.0
    payload = "{}:{}:{}".format(seed, target, axis).encode("utf-8")
    integer = int.from_bytes(hashlib.sha256(payload).digest()[:8], "big")
    unit = integer / float((1 << 64) - 1)
    return (2.0 * unit - 1.0) * limit


def _add_pose(a: PoseSpec, b: PoseSpec) -> PoseSpec:
    return PoseSpec(a.x + b.x, a.y + b.y, a.z + b.z, a.yaw + b.yaw)


def _pose_close(a: PoseSpec, b: PoseSpec, tolerance: float = 1.0e-12) -> bool:
    return all(
        abs(left - right) <= tolerance
        for left, right in ((a.x, b.x), (a.y, b.y), (a.z, b.z), (a.yaw, b.yaw))
    )


def _translation_distance(a: PoseSpec, b: PoseSpec) -> float:
    return math.sqrt(
        (a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2
    )


def _yaw_distance(a: PoseSpec, b: PoseSpec) -> float:
    return abs(math.atan2(math.sin(b.yaw - a.yaw), math.cos(b.yaw - a.yaw)))


def _parse_s06_online_map_contract(value) -> Mapping[str, object]:
    item = _strict_keys(
        value,
        S06_ONLINE_MAP_CONTRACT_KEYS,
        set(),
        "online_map_contract",
    )
    parsed = {
        "initial_state": _string(
            item["initial_state"], "online_map_contract.initial_state"
        ),
        "apriori_loader_used": item["apriori_loader_used"],
        "initial_map_revision": _integer(
            item["initial_map_revision"],
            "online_map_contract.initial_map_revision",
            0,
        ),
        "bootstrap_phase": _string(
            item["bootstrap_phase"], "online_map_contract.bootstrap_phase"
        ),
        "discovery_primitive": _string(
            item["discovery_primitive"],
            "online_map_contract.discovery_primitive",
        ),
        "scored_map_occluded_target": _string(
            item["scored_map_occluded_target"],
            "online_map_contract.scored_map_occluded_target",
        ),
        "transition": _string(
            item["transition"], "online_map_contract.transition"
        ),
    }
    if not isinstance(parsed["apriori_loader_used"], bool):
        raise ScenarioError(
            "online_map_contract.apriori_loader_used must be boolean"
        )
    if parsed != S06_ONLINE_MAP_CONTRACT:
        raise ScenarioError("schema-v6 S06 online_map_contract differs")
    return parsed


def _parse_s02_crossing_contract(value) -> Mapping[str, object]:
    item = _strict_keys(
        value,
        S02_CROSSING_CONTRACT_KEYS,
        set(),
        "crossing_contract",
    )
    parsed = dict(item)
    expected_strings = {
        "geometry_mode": "snapshot",
        "trajectory_model": "seventh_order_minimum_snap",
    }
    for key, expected in expected_strings.items():
        if _string(item[key], "crossing_contract." + key) != expected:
            raise ScenarioError(
                "schema-v7 S02 crossing_contract.{} differs".format(key)
            )
    if _integer(
        item["moving_target_count"],
        "crossing_contract.moving_target_count",
        1,
    ) != 10:
        raise ScenarioError("schema-v7 S02 requires ten moving targets")
    if _integer(
        item["continuity_order"],
        "crossing_contract.continuity_order",
        0,
    ) != 3:
        raise ScenarioError("schema-v7 S02 requires C3 segment boundaries")
    if _integer(
        item["occlusion_hold_count"],
        "crossing_contract.occlusion_hold_count",
        1,
    ) != 3 or _integer(
        item["exchange_count"],
        "crossing_contract.exchange_count",
        1,
    ) != 2:
        raise ScenarioError(
            "schema-v7 S02 requires three occlusion holds and two exchanges"
        )

    positive_fields = (
        "near_radius_m",
        "far_radius_m",
        "line_of_sight_slope",
        "altitude_deconfliction_m",
        "occlusion_center_offset_deg",
        "max_speed_mps",
        "max_acceleration_mps2",
        "max_jerk_mps3",
        "max_tilt_deg",
        "specific_thrust_min_g",
        "specific_thrust_max_g",
        "conservative_collision_radius_m",
        "minimum_center_separation_m",
        "minimum_conservative_clearance_m",
        "minimum_observer_center_distance_m",
        "minimum_ground_clearance_m",
        "maximum_sensor_range_m",
        "semantic_cluster_tolerance_m",
        "maximum_hold_pair_surface_gap_m",
    )
    for key in positive_fields:
        parsed[key] = _number(item[key], "crossing_contract." + key, 0.0)
    if any(parsed[key] <= 0.0 for key in positive_fields):
        raise ScenarioError(
            "schema-v7 S02 crossing contract limits must be positive"
        )
    parsed["max_yaw_rate_radps"] = _number(
        item["max_yaw_rate_radps"],
        "crossing_contract.max_yaw_rate_radps",
        0.0,
    )
    parsed["sensor_elevation_min_deg"] = _number(
        item["sensor_elevation_min_deg"],
        "crossing_contract.sensor_elevation_min_deg",
    )
    parsed["sensor_elevation_max_deg"] = _number(
        item["sensor_elevation_max_deg"],
        "crossing_contract.sensor_elevation_max_deg",
    )
    if not (
        parsed["near_radius_m"] < parsed["far_radius_m"]
        and parsed["specific_thrust_min_g"]
        < parsed["specific_thrust_max_g"]
        and parsed["sensor_elevation_min_deg"]
        < parsed["sensor_elevation_max_deg"]
        and parsed["minimum_conservative_clearance_m"] > 0.0
        and parsed["maximum_hold_pair_surface_gap_m"]
        < parsed["semantic_cluster_tolerance_m"]
    ):
        raise ScenarioError("schema-v7 S02 crossing limits are inconsistent")

    stages = item["occlusion_stages"]
    if not isinstance(stages, list) or len(stages) != 3:
        raise ScenarioError(
            "crossing_contract.occlusion_stages must contain three stages"
        )
    near_targets = {"uav{}".format(index) for index in range(2, 7)}
    far_targets = {"uav{}".format(index) for index in range(7, 12)}
    schedules = []
    for stage_index, stage_value in enumerate(stages):
        context = "crossing_contract.occlusion_stages[{}]".format(stage_index)
        stage = _strict_keys(stage_value, {"phase", "pairs"}, set(), context)
        phase = _string(stage["phase"], context + ".phase")
        pairs = stage["pairs"]
        if not isinstance(pairs, list) or len(pairs) != 5:
            raise ScenarioError(context + ".pairs must contain five pairs")
        parsed_pairs = []
        for pair_index, pair_value in enumerate(pairs):
            pair_context = "{}.pairs[{}]".format(context, pair_index)
            pair = _strict_keys(
                pair_value, {"near", "far"}, set(), pair_context
            )
            near = _string(pair["near"], pair_context + ".near")
            far = _string(pair["far"], pair_context + ".far")
            if near not in near_targets or far not in far_targets:
                raise ScenarioError(pair_context + " violates near/far rings")
            parsed_pairs.append((near, far))
        if (
            {pair[0] for pair in parsed_pairs} != near_targets
            or {pair[1] for pair in parsed_pairs} != far_targets
        ):
            raise ScenarioError(context + " is not a perfect matching")
        schedules.append((phase, tuple(sorted(parsed_pairs))))
    if len({schedule[0] for schedule in schedules}) != 3 or len(
        {schedule[1] for schedule in schedules}
    ) != 3:
        raise ScenarioError("schema-v7 S02 occlusion schedules must be distinct")
    parsed["occlusion_stages"] = tuple(schedules)
    return parsed


def _validate_observer_motion_trajectory(
    targets: Mapping[str, TargetSpec],
    phases: Tuple[PhaseSpec, ...],
    motion: ObserverMotionSpec,
) -> None:
    observer_initial = targets[motion.observer].initial_pose
    if (
        motion.fixed_yaw is not None
        and abs(observer_initial.yaw - motion.fixed_yaw) > 1.0e-12
    ):
        raise ScenarioError(
            "observer initial yaw differs from observer_motion.fixed_yaw"
        )

    prior = {name: target.initial_pose for name, target in targets.items()}
    total_translation = 0.0
    scored_matches = []
    for phase in phases:
        observer_start = prior[motion.observer]
        observer_goal = phase.goals[motion.observer]
        if motion.fixed_yaw is not None and (
            abs(observer_start.yaw - motion.fixed_yaw) > 1.0e-12
            or abs(observer_goal.yaw - motion.fixed_yaw) > 1.0e-12
        ):
            raise ScenarioError(
                "observer yaw must remain fixed in phase '{}'".format(
                    phase.phase_id
                )
            )
        phase_translation = _translation_distance(observer_start, observer_goal)
        total_translation += phase_translation
        if phase.phase_id == motion.scored_phase:
            scored_matches.append((phase, observer_start, observer_goal))
            if phase.event is None:
                raise ScenarioError(
                    "moving-observer scored phase must declare a boundary event"
                )
            if phase.interpolation == "hold":
                raise ScenarioError("moving-observer scored phase must move the observer")
            if phase_translation < motion.minimum_translation_m:
                raise ScenarioError(
                    "scored observer translation {:.6f} m is below {:.6f} m"
                    .format(phase_translation, motion.minimum_translation_m)
                )
            phase_yaw_change = _yaw_distance(observer_start, observer_goal)
            if phase_yaw_change < motion.minimum_yaw_change_rad:
                raise ScenarioError(
                    "scored observer yaw change {:.6f} rad is below {:.6f} rad"
                    .format(phase_yaw_change, motion.minimum_yaw_change_rad)
                )
            moving_targets = [
                name
                for name in motion.stationary_targets
                if not _pose_close(prior[name], phase.goals[name])
            ]
            if moving_targets:
                raise ScenarioError(
                    "scored phase moves stationary targets {}".format(
                        sorted(moving_targets)
                    )
                )
        prior = dict(phase.goals)

    if len(scored_matches) != 1:
        raise ScenarioError(
            "observer_motion.scored_phase must identify exactly one phase"
        )
    if total_translation < motion.minimum_translation_m:
        raise ScenarioError("observer trajectory has no qualifying translation")


def _validate_s04_exact_contract(
    seed: int,
    world_frame: str,
    repeat_count: int,
    repeat_mode: str,
    start_delay_ns: int,
    final_hold_ns: int,
    targets: Mapping[str, TargetSpec],
    phases: Tuple[PhaseSpec, ...],
    cycle_duration_ns: int,
    runtime: RuntimeSpec,
    motion: ObserverMotionSpec,
) -> None:
    """Fail closed unless schema-v4 is the frozen four-aircraft S04 line."""
    if (
        seed != S04_SEED
        or world_frame != "world"
        or repeat_count != 1
        or repeat_mode != "restart"
        or start_delay_ns != S04_START_DELAY_NS
        or final_hold_ns != S04_FINAL_HOLD_NS
        or cycle_duration_ns
        != (
            S04_INITIAL_CLEAR_HOLD_NS
            + S04_APPROACH_OCCLUSION_NS
            + S04_SCORED_MOTION_NS
            + S04_LEAVE_OCCLUSION_NS
            + S04_FINAL_CLEAR_HOLD_NS
        )
    ):
        raise ScenarioError(
            "schema-v4 S04 seed/frame/repeat/timeline contract differs"
        )
    if (
        runtime.service_name != "/gazebo/set_model_state"
        or runtime.world_properties_service != "/gazebo/get_world_properties"
        or runtime.event_topic != "/mid360_multi_uav_sim/scenario_events"
        or abs(runtime.publish_rate - 50.0) > 1.0e-12
        or abs(runtime.service_timeout_wall - 30.0) > 1.0e-12
        or abs(runtime.clock_timeout_wall - 30.0) > 1.0e-12
        or abs(runtime.max_clock_stall_wall - 20.0) > 1.0e-12
        or runtime.max_consecutive_service_failures != 3
        or not runtime.require_use_sim_time
    ):
        raise ScenarioError("schema-v4 S04 runtime authority contract differs")

    expected_poses = {
        "uav1": PoseSpec(4.0, -3.0, 2.0, -math.pi / 4.0),
        "uav2": PoseSpec(8.0, 24.0 / 13.0, 2.25, 0.0),
        "uav3": PoseSpec(13.0, 3.0, 2.25, 0.0),
        "uav4": PoseSpec(14.0, 0.0, 2.25, 0.0),
    }
    if set(targets) != set(expected_poses):
        raise ScenarioError(
            "schema-v4 S04 targets must be exactly ['uav1', 'uav2', 'uav3', 'uav4']"
        )
    for name, expected_pose in expected_poses.items():
        expected_child = "uav1/fcu" if name == "uav1" else name + "/base_link"
        expected_topic = "/mid360_multi_uav_sim/ground_truth/{}/odom".format(name)
        target = targets[name]
        if (
            target.model_name != name
            or target.truth_topic != expected_topic
            or target.child_frame_id != expected_child
            or not target.command_model
            or not _pose_close(target.initial_pose, expected_pose)
        ):
            raise ScenarioError(
                "schema-v4 S04 target '{}' identity/topic/frame/command/pose differs"
                .format(name)
            )

    if (
        motion.observer != "uav1"
        or motion.parent_frame != "world"
        or motion.child_frame != "uav1/fcu"
        or motion.tf_authority != S04_DYNAMIC_TF_AUTHORITY
        or not motion.unique_dynamic_tf_authority
        or motion.fixed_yaw is not None
        or abs(motion.minimum_translation_m - S04_MINIMUM_TRANSLATION_M) > 1.0e-12
        or abs(motion.minimum_yaw_change_rad - S04_MINIMUM_YAW_CHANGE_RAD)
        > 1.0e-12
        or motion.scored_phase != "combined_visibility_scored"
        or motion.stationary_targets != ("uav2", "uav3", "uav4")
    ):
        raise ScenarioError("schema-v4 S04 observer_motion contract differs")

    expected_phases = (
        (
            "initial_clear_hold",
            S04_INITIAL_CLEAR_HOLD_NS,
            "hold",
            None,
            expected_poses,
        ),
        (
            "approach_combined_occlusion",
            S04_APPROACH_OCCLUSION_NS,
            "linear",
            "initial_clear_end",
            dict(expected_poses, uav1=PoseSpec(0.0, 0.0, 2.0, 0.0)),
        ),
        (
            "combined_visibility_scored",
            S04_SCORED_MOTION_NS,
            "linear",
            "combined_visibility_start",
            dict(expected_poses, uav1=PoseSpec(-2.0, -6.0 / 13.0, 2.0, 0.4)),
        ),
        (
            "leave_combined_visibility",
            S04_LEAVE_OCCLUSION_NS,
            "linear",
            "combined_visibility_end",
            dict(expected_poses, uav1=PoseSpec(4.0, -3.0, 2.0, math.pi / 4.0)),
        ),
        (
            "final_clear_hold",
            S04_FINAL_CLEAR_HOLD_NS,
            "hold",
            "final_clear_start",
            dict(expected_poses, uav1=PoseSpec(4.0, -3.0, 2.0, math.pi / 4.0)),
        ),
    )
    if len(phases) != len(expected_phases):
        raise ScenarioError("schema-v4 S04 requires exactly five phases")
    for index, (phase, expected) in enumerate(zip(phases, expected_phases)):
        phase_id, duration_ns, interpolation, event, goals = expected
        if (
            phase.phase_id != phase_id
            or phase.duration_ns != duration_ns
            or phase.interpolation != interpolation
            or phase.event != event
            or set(phase.goals) != set(goals)
            or any(
                not _pose_close(phase.goals[name], goal)
                for name, goal in goals.items()
            )
        ):
            raise ScenarioError(
                "schema-v4 S04 phase {} contract differs".format(index)
            )


def _validate_s05_exact_contract(
    seed: int,
    world_frame: str,
    repeat_count: int,
    repeat_mode: str,
    start_delay_ns: int,
    final_hold_ns: int,
    targets: Mapping[str, TargetSpec],
    phases: Tuple[PhaseSpec, ...],
    cycle_duration_ns: int,
    runtime: RuntimeSpec,
    motion: ObserverMotionSpec,
) -> None:
    """Fail closed unless schema-v5 is the frozen six-aircraft S05 line."""
    if (
        seed != S05_SEED
        or world_frame != "world"
        or repeat_count != 1
        or repeat_mode != "restart"
        or start_delay_ns != S05_START_DELAY_NS
        or final_hold_ns != S05_FINAL_HOLD_NS
        or cycle_duration_ns
        != (
            S05_INITIAL_STRESS_HOLD_NS
            + S05_ENTER_CLUTTER_NS
            + S05_PEAK_STRESS_NS
            + S05_EXIT_CLUTTER_NS
            + S05_FINAL_STRESS_HOLD_NS
        )
    ):
        raise ScenarioError(
            "schema-v5 S05 seed/frame/repeat/timeline contract differs"
        )
    if (
        runtime.service_name != "/gazebo/set_model_state"
        or runtime.world_properties_service != "/gazebo/get_world_properties"
        or runtime.event_topic != "/mid360_multi_uav_sim/scenario_events"
        or abs(runtime.publish_rate - 50.0) > 1.0e-12
        or abs(runtime.service_timeout_wall - 30.0) > 1.0e-12
        or abs(runtime.clock_timeout_wall - 30.0) > 1.0e-12
        or abs(runtime.max_clock_stall_wall - 20.0) > 1.0e-12
        or runtime.max_consecutive_service_failures != 3
        or not runtime.require_use_sim_time
    ):
        raise ScenarioError("schema-v5 S05 runtime authority contract differs")

    expected_poses = {
        "uav1": PoseSpec(-4.0, -5.0, 2.0, -0.35),
        "uav2": PoseSpec(7.0, -4.0, 2.25, 0.0),
        "uav3": PoseSpec(10.0, -1.0, 2.25, 0.0),
        "uav4": PoseSpec(12.0, 3.0, 2.25, 0.0),
        "uav5": PoseSpec(7.0, 5.0, 2.25, 0.0),
        "uav6": PoseSpec(15.0, 0.8, 2.25, 0.0),
    }
    if set(targets) != set(expected_poses):
        raise ScenarioError(
            "schema-v5 S05 targets must be exactly "
            "['uav1', 'uav2', 'uav3', 'uav4', 'uav5', 'uav6']"
        )
    for name, expected_pose in expected_poses.items():
        expected_child = "uav1/fcu" if name == "uav1" else name + "/base_link"
        expected_topic = "/mid360_multi_uav_sim/ground_truth/{}/odom".format(name)
        target = targets[name]
        if (
            target.model_name != name
            or target.truth_topic != expected_topic
            or target.child_frame_id != expected_child
            or not target.command_model
            or not _pose_close(target.initial_pose, expected_pose)
        ):
            raise ScenarioError(
                "schema-v5 S05 target '{}' identity/topic/frame/command/pose differs"
                .format(name)
            )

    if (
        motion.observer != "uav1"
        or motion.parent_frame != "world"
        or motion.child_frame != "uav1/fcu"
        or motion.tf_authority != S05_DYNAMIC_TF_AUTHORITY
        or not motion.unique_dynamic_tf_authority
        or motion.fixed_yaw is not None
        or abs(motion.minimum_translation_m - S05_MINIMUM_TRANSLATION_M)
        > 1.0e-12
        or abs(motion.minimum_yaw_change_rad - S05_MINIMUM_YAW_CHANGE_RAD)
        > 1.0e-12
        or motion.scored_phase != "peak_stress_scored"
        or motion.stationary_targets
        != ("uav2", "uav3", "uav4", "uav5", "uav6")
    ):
        raise ScenarioError("schema-v5 S05 observer_motion contract differs")

    expected_phases = (
        (
            "initial_stress_hold",
            S05_INITIAL_STRESS_HOLD_NS,
            "hold",
            None,
            expected_poses,
        ),
        (
            "enter_clutter",
            S05_ENTER_CLUTTER_NS,
            "linear",
            "initial_stress_end",
            dict(expected_poses, uav1=PoseSpec(-1.0, -2.0, 2.0, 0.0)),
        ),
        (
            "peak_stress_scored",
            S05_PEAK_STRESS_NS,
            "linear",
            "stress_window_start",
            dict(expected_poses, uav1=PoseSpec(3.0, 2.0, 2.0, 0.75)),
        ),
        (
            "exit_clutter",
            S05_EXIT_CLUTTER_NS,
            "linear",
            "stress_window_end",
            dict(expected_poses, uav1=PoseSpec(5.0, -3.0, 2.0, -0.4)),
        ),
        (
            "final_stress_hold",
            S05_FINAL_STRESS_HOLD_NS,
            "hold",
            "final_stress_start",
            dict(expected_poses, uav1=PoseSpec(5.0, -3.0, 2.0, -0.4)),
        ),
    )
    if len(phases) != len(expected_phases):
        raise ScenarioError("schema-v5 S05 requires exactly five phases")
    for index, (phase, expected) in enumerate(zip(phases, expected_phases)):
        phase_id, duration_ns, interpolation, event, goals = expected
        if (
            phase.phase_id != phase_id
            or phase.duration_ns != duration_ns
            or phase.interpolation != interpolation
            or phase.event != event
            or set(phase.goals) != set(goals)
            or any(
                not _pose_close(phase.goals[name], goal)
                for name, goal in goals.items()
            )
        ):
            raise ScenarioError(
                "schema-v5 S05 phase {} contract differs".format(index)
            )


def _validate_s06_exact_contract(
    seed: int,
    world_frame: str,
    repeat_count: int,
    repeat_mode: str,
    start_delay_ns: int,
    final_hold_ns: int,
    targets: Mapping[str, TargetSpec],
    phases: Tuple[PhaseSpec, ...],
    cycle_duration_ns: int,
    runtime: RuntimeSpec,
    motion: ObserverMotionSpec,
    online_map_contract: Mapping[str, object],
) -> None:
    """Fail closed unless schema-v6 is the frozen online-map S06 line."""
    if (
        seed != S06_SEED
        or world_frame != "world"
        or repeat_count != 1
        or repeat_mode != "restart"
        or start_delay_ns != S06_START_DELAY_NS
        or final_hold_ns != S06_FINAL_HOLD_NS
        or cycle_duration_ns
        != (
            S06_MAP_BOOTSTRAP_CLEAR_HOLD_NS
            + S06_APPROACH_ONLINE_OCCLUSION_NS
            + S06_ONLINE_OCCLUSION_NS
            + S06_LEAVE_ONLINE_OCCLUSION_NS
            + S06_FINAL_CLEAR_HOLD_NS
        )
    ):
        raise ScenarioError(
            "schema-v6 S06 seed/frame/repeat/timeline contract differs"
        )
    if (
        runtime.service_name != "/gazebo/set_model_state"
        or runtime.world_properties_service != "/gazebo/get_world_properties"
        or runtime.event_topic != "/mid360_multi_uav_sim/scenario_events"
        or abs(runtime.publish_rate - 50.0) > 1.0e-12
        or abs(runtime.service_timeout_wall - 30.0) > 1.0e-12
        or abs(runtime.clock_timeout_wall - 30.0) > 1.0e-12
        or abs(runtime.max_clock_stall_wall - 20.0) > 1.0e-12
        or runtime.max_consecutive_service_failures != 3
        or not runtime.require_use_sim_time
    ):
        raise ScenarioError("schema-v6 S06 runtime authority contract differs")
    if dict(online_map_contract) != S06_ONLINE_MAP_CONTRACT:
        raise ScenarioError("schema-v6 S06 online_map_contract differs")

    expected_poses = {
        "uav1": PoseSpec(4.0, -3.0, 2.0, -math.pi / 4.0),
        "uav2": PoseSpec(8.0, 24.0 / 13.0, 2.25, 0.0),
        "uav3": PoseSpec(13.0, 3.0, 2.25, 0.0),
        "uav4": PoseSpec(14.0, 0.0, 2.25, 0.0),
    }
    if set(targets) != set(expected_poses):
        raise ScenarioError(
            "schema-v6 S06 targets must be exactly "
            "['uav1', 'uav2', 'uav3', 'uav4']"
        )
    for name, expected_pose in expected_poses.items():
        expected_child = "uav1/fcu" if name == "uav1" else name + "/base_link"
        expected_topic = "/mid360_multi_uav_sim/ground_truth/{}/odom".format(name)
        target = targets[name]
        if (
            target.model_name != name
            or target.truth_topic != expected_topic
            or target.child_frame_id != expected_child
            or not target.command_model
            or not _pose_close(target.initial_pose, expected_pose)
        ):
            raise ScenarioError(
                "schema-v6 S06 target '{}' identity/topic/frame/command/pose differs"
                .format(name)
            )

    if (
        motion.observer != "uav1"
        or motion.parent_frame != "world"
        or motion.child_frame != "uav1/fcu"
        or motion.tf_authority != S06_DYNAMIC_TF_AUTHORITY
        or not motion.unique_dynamic_tf_authority
        or motion.fixed_yaw is not None
        or abs(motion.minimum_translation_m - S06_MINIMUM_TRANSLATION_M)
        > 1.0e-12
        or abs(motion.minimum_yaw_change_rad - S06_MINIMUM_YAW_CHANGE_RAD)
        > 1.0e-12
        or motion.scored_phase != "online_occlusion_scored"
        or motion.stationary_targets != ("uav2", "uav3", "uav4")
    ):
        raise ScenarioError("schema-v6 S06 observer_motion contract differs")

    expected_phases = (
        (
            "map_bootstrap_clear_hold",
            S06_MAP_BOOTSTRAP_CLEAR_HOLD_NS,
            "hold",
            None,
            expected_poses,
        ),
        (
            "approach_online_occlusion",
            S06_APPROACH_ONLINE_OCCLUSION_NS,
            "linear",
            "map_bootstrap_end",
            dict(expected_poses, uav1=PoseSpec(0.0, 0.0, 2.0, 0.0)),
        ),
        (
            "online_occlusion_scored",
            S06_ONLINE_OCCLUSION_NS,
            "linear",
            "online_occlusion_start",
            dict(
                expected_poses,
                uav1=PoseSpec(-2.0, -6.0 / 13.0, 2.0, 0.4),
            ),
        ),
        (
            "leave_online_occlusion",
            S06_LEAVE_ONLINE_OCCLUSION_NS,
            "linear",
            "online_occlusion_end",
            dict(
                expected_poses,
                uav1=PoseSpec(4.0, -3.0, 2.0, math.pi / 4.0),
            ),
        ),
        (
            "final_clear_hold",
            S06_FINAL_CLEAR_HOLD_NS,
            "hold",
            "final_clear_start",
            dict(
                expected_poses,
                uav1=PoseSpec(4.0, -3.0, 2.0, math.pi / 4.0),
            ),
        ),
    )
    if len(phases) != len(expected_phases):
        raise ScenarioError("schema-v6 S06 requires exactly five phases")
    for index, (phase, expected) in enumerate(zip(phases, expected_phases)):
        phase_id, duration_ns, interpolation, event, goals = expected
        if (
            phase.phase_id != phase_id
            or phase.duration_ns != duration_ns
            or phase.interpolation != interpolation
            or phase.event != event
            or set(phase.goals) != set(goals)
            or any(
                not _pose_close(phase.goals[name], goal)
                for name, goal in goals.items()
            )
        ):
            raise ScenarioError(
                "schema-v6 S06 phase {} contract differs".format(index)
            )


def _validate_s02_exact_contract(
    seed: int,
    world_frame: str,
    repeat_count: int,
    repeat_mode: str,
    start_delay_ns: int,
    final_hold_ns: int,
    targets: Mapping[str, TargetSpec],
    phases: Tuple[PhaseSpec, ...],
    cycle_duration_ns: int,
    runtime: RuntimeSpec,
    crossing_contract: Mapping[str, object],
) -> None:
    if (
        seed != 2027
        or world_frame != "world"
        or repeat_count != 1
        or repeat_mode != "restart"
        or start_delay_ns != 8 * NSEC_PER_SEC
        or final_hold_ns != 4 * NSEC_PER_SEC
        or cycle_duration_ns != 63 * NSEC_PER_SEC
    ):
        raise ScenarioError("schema-v7 S02 timeline contract differs")
    expected_targets = {"uav{}".format(index) for index in range(1, 12)}
    if set(targets) != expected_targets:
        raise ScenarioError("schema-v7 S02 targets must be uav1 through uav11")
    if targets["uav1"].command_model or any(
        not targets["uav{}".format(index)].command_model
        for index in range(2, 12)
    ):
        raise ScenarioError(
            "schema-v7 S02 requires one static observer and ten commanded targets"
        )

    if (
        runtime.service_name != "/gazebo/set_model_state"
        or runtime.world_properties_service != "/gazebo/get_world_properties"
        or runtime.event_topic != "/mid360_multi_uav_sim/scenario_events"
        or abs(runtime.publish_rate - 50.0) > 1.0e-12
        or abs(runtime.service_timeout_wall - 30.0) > 1.0e-12
        or abs(runtime.clock_timeout_wall - 30.0) > 1.0e-12
        or abs(runtime.max_clock_stall_wall - 20.0) > 1.0e-12
        or runtime.max_consecutive_service_failures != 3
        or not runtime.require_use_sim_time
    ):
        raise ScenarioError("schema-v7 S02 runtime authority contract differs")

    def ring_pose(radius: float, angle_deg: float, altitude: float) -> PoseSpec:
        angle = math.radians(angle_deg)
        return PoseSpec(
            round(radius * math.cos(angle), 9),
            round(radius * math.sin(angle), 9),
            altitude,
            0.0,
        )

    near_names = tuple("uav{}".format(index) for index in range(2, 7))
    far_names = tuple("uav{}".format(index) for index in range(7, 12))
    base_angles = {
        name: 72.0 * index for index, name in enumerate(near_names)
    }
    base_angles.update(
        {name: 72.0 * index for index, name in enumerate(far_names)}
    )

    def formation(
        near_offset_deg: float,
        near_altitude: float,
        far_offset_deg: float,
        far_altitude: float,
    ) -> Mapping[str, PoseSpec]:
        result = {"uav1": PoseSpec(0.0, 0.0, 2.0, 0.0)}
        result.update({
            name: ring_pose(7.5, base_angles[name] + near_offset_deg,
                            near_altitude)
            for name in near_names
        })
        result.update({
            name: ring_pose(9.65, base_angles[name] + far_offset_deg,
                            far_altitude)
            for name in far_names
        })
        return result

    expected_initial = formation(-18.0, 3.15, 18.0, 3.408)
    for name, expected_pose in expected_initial.items():
        target = targets[name]
        expected_child = "uav1/fcu" if name == "uav1" else name + "/base_link"
        expected_topic = "/mid360_multi_uav_sim/ground_truth/{}/odom".format(
            name
        )
        if (
            target.model_name != name
            or target.truth_topic != expected_topic
            or target.child_frame_id != expected_child
            or not _pose_close(target.initial_pose, expected_pose)
        ):
            raise ScenarioError(
                "schema-v7 S02 target '{}' identity/topic/frame/pose differs"
                .format(name)
            )

    expected_phases = (
        ("converge_to_occlusion_a", 6, "minimum_snap", "convergence_start"),
        ("occlusion_hold_a", 3, "hold", "occlusion_a_start"),
        ("outer_climb", 4, "minimum_snap", "occlusion_a_end"),
        ("counterflow_exchange_a", 13, "minimum_snap", "exchange_a_start"),
        ("outer_descend", 4, "minimum_snap", "exchange_a_end"),
        ("occlusion_hold_b", 3, "hold", "occlusion_b_start"),
        ("inner_descend", 4, "minimum_snap", "occlusion_b_end"),
        ("counterflow_exchange_b", 13, "minimum_snap", "exchange_b_start"),
        ("inner_rise", 4, "minimum_snap", "exchange_b_end"),
        ("occlusion_hold_c", 3, "hold", "occlusion_c_start"),
        ("fan_out", 6, "minimum_snap", "occlusion_c_end"),
    )
    expected_goals = (
        formation(0.0, 3.15, 1.8, 3.408),
        formation(0.0, 3.15, 1.8, 3.408),
        formation(0.0, 3.15, 1.8, 4.908),
        formation(72.0, 3.15, -70.2, 4.908),
        formation(72.0, 3.15, -70.2, 3.408),
        formation(72.0, 3.15, -70.2, 3.408),
        formation(72.0, 1.65, -70.2, 3.408),
        formation(144.0, 1.65, -142.2, 3.408),
        formation(144.0, 3.15, -142.2, 3.408),
        formation(144.0, 3.15, -142.2, 3.408),
        formation(126.0, 3.15, -126.0, 3.408),
    )
    if len(phases) != len(expected_phases):
        raise ScenarioError("schema-v7 S02 requires exactly eleven phases")
    for index, (phase, expected, goals) in enumerate(
        zip(phases, expected_phases, expected_goals)
    ):
        phase_id, duration_s, interpolation, event = expected
        if (
            phase.phase_id != phase_id
            or phase.duration_ns != duration_s * NSEC_PER_SEC
            or phase.interpolation != interpolation
            or phase.event != event
            or set(phase.goals) != set(goals)
            or any(
                not _pose_close(phase.goals[name], goal)
                for name, goal in goals.items()
            )
        ):
            raise ScenarioError(
                "schema-v7 S02 phase {} contract differs".format(index)
            )
        if any(abs(pose.yaw) > 1.0e-12 for pose in phase.goals.values()):
            raise ScenarioError("schema-v7 S02 yaw must remain fixed")

    declared_stage_phases = {
        stage[0] for stage in crossing_contract["occlusion_stages"]
    }
    if declared_stage_phases != {
        "occlusion_hold_a",
        "occlusion_hold_b",
        "occlusion_hold_c",
    }:
        raise ScenarioError("schema-v7 S02 occlusion stage phases differ")
    expected_schedules = (
        (
            "occlusion_hold_a",
            tuple(sorted((
                ("uav2", "uav7"),
                ("uav3", "uav8"),
                ("uav4", "uav9"),
                ("uav5", "uav10"),
                ("uav6", "uav11"),
            ))),
        ),
        (
            "occlusion_hold_b",
            tuple(sorted((
                ("uav2", "uav9"),
                ("uav3", "uav10"),
                ("uav4", "uav11"),
                ("uav5", "uav7"),
                ("uav6", "uav8"),
            ))),
        ),
        (
            "occlusion_hold_c",
            tuple(sorted((
                ("uav2", "uav11"),
                ("uav3", "uav7"),
                ("uav4", "uav8"),
                ("uav5", "uav9"),
                ("uav6", "uav10"),
            ))),
        ),
    )
    if tuple(crossing_contract["occlusion_stages"]) != expected_schedules:
        raise ScenarioError("schema-v7 S02 occlusion pair schedule differs")
    if (
        abs(crossing_contract["near_radius_m"] - 7.5) > 1.0e-12
        or abs(crossing_contract["far_radius_m"] - 9.65) > 1.0e-12
        or abs(crossing_contract["line_of_sight_slope"] - 0.12) > 1.0e-12
        or abs(crossing_contract["altitude_deconfliction_m"] - 1.5)
        > 1.0e-12
        or abs(crossing_contract["occlusion_center_offset_deg"] - 1.8)
        > 1.0e-12
        or abs(crossing_contract["semantic_cluster_tolerance_m"] - 1.25)
        > 1.0e-12
        or crossing_contract["max_yaw_rate_radps"] != 0.0
    ):
        raise ScenarioError("schema-v7 S02 crossing geometry contract differs")


def _motion_state_close(
    left: MotionState, right: MotionState, tolerance: float = 1.0e-12
) -> bool:
    return _pose_close(left.pose, right.pose, tolerance) and all(
        abs(a - b) <= tolerance
        for a, b in (
            (left.vx, right.vx),
            (left.vy, right.vy),
            (left.vz, right.vz),
            (left.yaw_rate, right.yaw_rate),
        )
    )


def _should_publish_dynamic_tf(
    last_stamp_ns: Optional[int],
    last_state: Optional[MotionState],
    stamp_ns: int,
    state: MotionState,
) -> bool:
    if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int) or stamp_ns < 0:
        raise ScenarioError("dynamic TF stamp must be a non-negative integer")
    if last_stamp_ns is None:
        if last_state is not None:
            raise ScenarioError("dynamic TF state exists without a prior stamp")
        return True
    if last_state is None:
        raise ScenarioError("dynamic TF stamp exists without a prior state")
    if stamp_ns < last_stamp_ns:
        raise ScenarioError(
            "dynamic TF stamp moved backwards from {} to {} ns".format(
                last_stamp_ns, stamp_ns
            )
        )
    if stamp_ns == last_stamp_ns:
        if not _motion_state_close(last_state, state):
            raise ScenarioError(
                "dynamic TF received conflicting MotionState at {} ns".format(
                    stamp_ns
                )
            )
        return False
    return True


def create_dynamic_tf_broadcaster(observer_motion):
    """Create one TF authority only for a validated moving-observer contract."""
    if observer_motion is None:
        return None
    return tf2_ros.TransformBroadcaster()


def build_dynamic_observer_transform(
    motion: ObserverMotionSpec, state: MotionState, stamp_ns: int
) -> TransformStamped:
    if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int) or stamp_ns < 0:
        raise ScenarioError("dynamic TF stamp must be a non-negative integer")
    message = TransformStamped()
    message.header.stamp = rospy.Time(
        stamp_ns // NSEC_PER_SEC, stamp_ns % NSEC_PER_SEC
    )
    message.header.frame_id = motion.parent_frame
    message.child_frame_id = motion.child_frame
    message.transform.translation.x = state.pose.x
    message.transform.translation.y = state.pose.y
    message.transform.translation.z = state.pose.z
    half = 0.5 * state.pose.yaw
    message.transform.rotation.x = 0.0
    message.transform.rotation.y = 0.0
    message.transform.rotation.z = math.sin(half)
    message.transform.rotation.w = math.cos(half)
    return message


def _get_optional_param(name: str):
    return rospy.get_param(name) if rospy.has_param(name) else None


def load_scenario(
    path: str,
    seed_override=None,
    duration_override=None,
    repeat_count_override=None,
    final_hold_override=None,
    start_delay_override=None,
) -> ScenarioSpec:
    """Parse and fully validate a scenario before touching Gazebo."""
    try:
        with open(path, "r", encoding="utf-8") as stream:
            raw = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise ScenarioError("cannot load scenario '{}': {}".format(path, exc))

    root = _mapping(raw, "scenario")
    schema_version = _integer(root.get("schema_version", 1), "schema_version", 1)
    if schema_version not in (1, 2, 4, 5, 6, 7):
        raise ScenarioError(
            "unsupported schema_version {}; expected 1, 2, 4, 5, 6, or 7".format(
                schema_version
            )
        )
    online_map_contract = None
    s02_crossing_contract = None
    if schema_version == 2:
        root = _strict_keys(root, S03_ROOT_KEYS, set(), "scenario")
        scenario_profile = _string(root["scenario_profile"], "scenario_profile")
        semantic_contract_id = _string(
            root["semantic_contract_id"], "semantic_contract_id"
        )
        if scenario_profile != S03_SCENARIO_PROFILE:
            raise ScenarioError(
                "schema-v2 scenario_profile must be '{}'".format(
                    S03_SCENARIO_PROFILE
                )
            )
        if semantic_contract_id != S03_SEMANTIC_CONTRACT_ID:
            raise ScenarioError(
                "schema-v2 semantic_contract_id must be '{}'".format(
                    S03_SEMANTIC_CONTRACT_ID
                )
            )
        scenario_contract_id = None
    elif schema_version == 4:
        root = _strict_keys(root, S03_ROOT_KEYS, set(), "scenario")
        scenario_profile = _string(root["scenario_profile"], "scenario_profile")
        semantic_contract_id = _string(
            root["semantic_contract_id"], "semantic_contract_id"
        )
        if scenario_profile != S04_SCENARIO_PROFILE:
            raise ScenarioError(
                "schema-v4 scenario_profile must be '{}'".format(
                    S04_SCENARIO_PROFILE
                )
            )
        if semantic_contract_id != S04_SEMANTIC_CONTRACT_ID:
            raise ScenarioError(
                "schema-v4 semantic_contract_id must be '{}'".format(
                    S04_SEMANTIC_CONTRACT_ID
                )
            )
        scenario_contract_id = None
    elif schema_version == 5:
        root = _strict_keys(root, S05_ROOT_KEYS, set(), "scenario")
        scenario_profile = _string(root["scenario_profile"], "scenario_profile")
        semantic_contract_id = _string(
            root["semantic_contract_id"], "semantic_contract_id"
        )
        if scenario_profile != S05_SCENARIO_PROFILE:
            raise ScenarioError(
                "schema-v5 scenario_profile must be '{}'".format(
                    S05_SCENARIO_PROFILE
                )
            )
        if semantic_contract_id != S05_SEMANTIC_CONTRACT_ID:
            raise ScenarioError(
                "schema-v5 semantic_contract_id must be '{}'".format(
                    S05_SEMANTIC_CONTRACT_ID
                )
            )
        if any(
            value is not None
            for value in (
                seed_override,
                duration_override,
                repeat_count_override,
                final_hold_override,
                start_delay_override,
            )
        ):
            raise ScenarioError("schema-v5 S05 does not permit parameter overrides")
        scenario_contract_id = None
    elif schema_version == 6:
        root = _strict_keys(root, S06_ROOT_KEYS, set(), "scenario")
        scenario_profile = _string(root["scenario_profile"], "scenario_profile")
        semantic_contract_id = _string(
            root["semantic_contract_id"], "semantic_contract_id"
        )
        if scenario_profile != S06_SCENARIO_PROFILE:
            raise ScenarioError(
                "schema-v6 scenario_profile must be '{}'".format(
                    S06_SCENARIO_PROFILE
                )
            )
        if semantic_contract_id != S06_SEMANTIC_CONTRACT_ID:
            raise ScenarioError(
                "schema-v6 semantic_contract_id must be '{}'".format(
                    S06_SEMANTIC_CONTRACT_ID
                )
            )
        if any(
            value is not None
            for value in (
                seed_override,
                duration_override,
                repeat_count_override,
                final_hold_override,
                start_delay_override,
            )
        ):
            raise ScenarioError("schema-v6 S06 does not permit parameter overrides")
        online_map_contract = _parse_s06_online_map_contract(
            root["online_map_contract"]
        )
        scenario_contract_id = None
    elif schema_version == 7:
        root = _strict_keys(root, S02_ROOT_KEYS, set(), "scenario")
        scenario_profile = _string(root["scenario_profile"], "scenario_profile")
        semantic_contract_id = _string(
            root["semantic_contract_id"], "semantic_contract_id"
        )
        scenario_contract_id = _string(
            root["scenario_contract_id"], "scenario_contract_id"
        )
        if scenario_profile != S02_SCENARIO_PROFILE:
            raise ScenarioError(
                "schema-v7 scenario_profile must be '{}'".format(
                    S02_SCENARIO_PROFILE
                )
            )
        if semantic_contract_id != S02_SEMANTIC_CONTRACT_ID:
            raise ScenarioError(
                "schema-v7 semantic_contract_id must be '{}'".format(
                    S02_SEMANTIC_CONTRACT_ID
                )
            )
        if scenario_contract_id != S02_SCENARIO_CONTRACT_ID:
            raise ScenarioError(
                "schema-v7 scenario_contract_id must be '{}'".format(
                    S02_SCENARIO_CONTRACT_ID
                )
            )
        if any(
            value is not None
            for value in (
                seed_override,
                duration_override,
                repeat_count_override,
                final_hold_override,
                start_delay_override,
            )
        ):
            raise ScenarioError("schema-v7 S02 does not permit parameter overrides")
        s02_crossing_contract = _parse_s02_crossing_contract(
            root["crossing_contract"]
        )
    else:
        scenario_profile = None
        semantic_contract_id = None
        scenario_contract_id = None

    scenario_id = _string(root.get("scenario"), "scenario")
    if schema_version == 2 and scenario_id != S03_SCENARIO_ID:
        raise ScenarioError(
            "schema-v2 scenario must be '{}'".format(S03_SCENARIO_ID)
        )
    if schema_version == 4 and scenario_id != S04_SCENARIO_ID:
        raise ScenarioError(
            "schema-v4 scenario must be '{}'".format(S04_SCENARIO_ID)
        )
    if schema_version == 5 and scenario_id != S05_SCENARIO_ID:
        raise ScenarioError(
            "schema-v5 scenario must be '{}'".format(S05_SCENARIO_ID)
        )
    if schema_version == 6 and scenario_id != S06_SCENARIO_ID:
        raise ScenarioError(
            "schema-v6 scenario must be '{}'".format(S06_SCENARIO_ID)
        )
    if schema_version == 7 and scenario_id != S02_SCENARIO_ID:
        raise ScenarioError(
            "schema-v7 scenario must be '{}'".format(S02_SCENARIO_ID)
        )
    world_frame = _string(root.get("world_frame", "world"), "world_frame")
    seed_value = root.get("seed", 0) if seed_override is None else seed_override
    seed = _integer(seed_value, "seed", 0)
    repeat_value = root.get("repeat_count", 1) if repeat_count_override is None else repeat_count_override
    repeat_count = _integer(repeat_value, "repeat_count", 1)
    repeat_mode = _string(root.get("repeat_mode", "restart"), "repeat_mode")
    if repeat_mode not in ALLOWED_REPEAT_MODES:
        raise ScenarioError("repeat_mode must be one of {}".format(ALLOWED_REPEAT_MODES))
    start_delay_value = root.get("start_delay", 0.0) if start_delay_override is None else start_delay_override
    final_hold_value = root.get("final_hold", 0.0) if final_hold_override is None else final_hold_override
    start_delay_ns = _duration_ns(start_delay_value, "start_delay", allow_zero=True)
    final_hold_ns = _duration_ns(final_hold_value, "final_hold", allow_zero=True)

    runtime_raw = _mapping(root.get("runtime", {}), "runtime")
    if schema_version in (2, 4, 5, 6, 7):
        runtime_raw = _strict_keys(runtime_raw, S03_RUNTIME_KEYS, set(), "runtime")
    service_name = _string(runtime_raw.get("set_model_state_service", "/gazebo/set_model_state"),
                           "runtime.set_model_state_service")
    world_properties_service = _string(
        runtime_raw.get("world_properties_service", "/gazebo/get_world_properties"),
        "runtime.world_properties_service",
    )
    event_topic = _string(runtime_raw.get("event_topic", "/mid360_multi_uav_sim/scenario_events"),
                          "runtime.event_topic")
    publish_rate = _number(runtime_raw.get("publish_rate", 50.0), "runtime.publish_rate", 0.1)
    service_timeout_wall = _number(runtime_raw.get("service_timeout_wall", 30.0),
                                   "runtime.service_timeout_wall", 0.1)
    clock_timeout_wall = _number(runtime_raw.get("clock_timeout_wall", 30.0),
                                 "runtime.clock_timeout_wall", 0.1)
    max_clock_stall_wall = _number(runtime_raw.get("max_clock_stall_wall", 20.0),
                                   "runtime.max_clock_stall_wall", 0.1)
    failures = _integer(runtime_raw.get("max_consecutive_service_failures", 3),
                        "runtime.max_consecutive_service_failures", 1)
    require_use_sim_time = runtime_raw.get("require_use_sim_time", True)
    if not isinstance(require_use_sim_time, bool):
        raise ScenarioError("runtime.require_use_sim_time must be boolean")
    runtime = RuntimeSpec(service_name, world_properties_service, event_topic, publish_rate,
                          service_timeout_wall, clock_timeout_wall, max_clock_stall_wall,
                          failures, require_use_sim_time)

    targets_raw = _mapping(root.get("targets"), "targets")
    if not targets_raw:
        raise ScenarioError("targets must not be empty")
    targets: Dict[str, TargetSpec] = {}
    for name in sorted(targets_raw.keys()):
        target_name = _string(name, "target name")
        item = _mapping(targets_raw[name], "targets.{}".format(target_name))
        if schema_version in (2, 4, 5, 6, 7):
            item = _strict_keys(
                item,
                S03_TARGET_KEYS,
                set(),
                "targets.{}".format(target_name),
            )
        model_name = _string(item.get("model_name", target_name),
                             "targets.{}.model_name".format(target_name))
        truth_topic = _string(item.get("truth_topic"),
                              "targets.{}.truth_topic".format(target_name))
        if not truth_topic.startswith("/"):
            raise ScenarioError("targets.{}.truth_topic must be absolute".format(target_name))
        child = _string(item.get("child_frame_id"),
                        "targets.{}.child_frame_id".format(target_name))
        command_model = item.get("command_model", True)
        if not isinstance(command_model, bool):
            raise ScenarioError("targets.{}.command_model must be boolean".format(target_name))
        initial = _pose(item.get("initial_pose"), "targets.{}.initial_pose".format(target_name))
        targets[target_name] = TargetSpec(target_name, model_name, truth_topic, child,
                                          command_model, initial)

    if "uav1" not in targets:
        raise ScenarioError("scenario requires observer entry 'uav1'")
    if schema_version in (1, 7) and targets["uav1"].command_model:
        raise ScenarioError("uav1 is the observer and must set command_model: false")
    if schema_version == 2 and set(targets) != {"uav1", "uav2", "uav3"}:
        raise ScenarioError(
            "schema-v2 S03 targets must be exactly ['uav1', 'uav2', 'uav3']"
        )
    if schema_version in (2, 4, 5, 6, 7):
        for label, values in (
            ("model_name", [target.model_name for target in targets.values()]),
            ("truth_topic", [target.truth_topic for target in targets.values()]),
            (
                "child_frame_id",
                [target.child_frame_id for target in targets.values()],
            ),
        ):
            if len(set(values)) != len(values):
                raise ScenarioError(
                    "schema-v{} target {} values must be unique".format(
                        schema_version, label
                    )
                )
    if schema_version in (2, 4, 5, 6) and not targets["uav1"].command_model:
        raise ScenarioError(
            "schema-v{} observer uav1 must set command_model: true".format(
                schema_version
            )
        )
    commanded_targets = [
        name for name, target in targets.items()
        if name != "uav1" and target.command_model
    ]
    if not commanded_targets:
        raise ScenarioError(
            "scenario requires at least one non-observer target with command_model: true"
        )

    observer_motion = None
    if schema_version in (2, 4, 5, 6):
        raw_motion = _strict_keys(
            root["observer_motion"],
            {
                2: S03_OBSERVER_MOTION_KEYS,
                4: S04_OBSERVER_MOTION_KEYS,
                5: S05_OBSERVER_MOTION_KEYS,
                6: S06_OBSERVER_MOTION_KEYS,
            }[schema_version],
            set(),
            "observer_motion",
        )
        observer = _string(raw_motion["observer"], "observer_motion.observer")
        parent_frame = _string(
            raw_motion["parent_frame"], "observer_motion.parent_frame"
        )
        child_frame = _string(
            raw_motion["child_frame"], "observer_motion.child_frame"
        )
        tf_authority = _string(
            raw_motion["tf_authority"], "observer_motion.tf_authority"
        )
        unique_tf = raw_motion["unique_dynamic_tf_authority"]
        if not isinstance(unique_tf, bool) or not unique_tf:
            raise ScenarioError(
                "observer_motion.unique_dynamic_tf_authority must be true"
            )
        fixed_yaw = (
            _number(raw_motion["fixed_yaw"], "observer_motion.fixed_yaw")
            if schema_version == 2
            else None
        )
        minimum_translation_m = _number(
            raw_motion["minimum_translation_m"],
            "observer_motion.minimum_translation_m",
            1.0e-6,
        )
        minimum_yaw_change_rad = (
            0.0
            if schema_version == 2
            else _number(
                raw_motion["minimum_yaw_change_rad"],
                "observer_motion.minimum_yaw_change_rad",
                1.0e-6,
            )
        )
        scored_phase = _string(
            raw_motion["scored_phase"], "observer_motion.scored_phase"
        )
        stationary_raw = raw_motion["stationary_targets"]
        if not isinstance(stationary_raw, list) or not stationary_raw:
            raise ScenarioError(
                "observer_motion.stationary_targets must be a non-empty sequence"
            )
        stationary_targets = tuple(
            _string(value, "observer_motion.stationary_targets[{}]".format(index))
            for index, value in enumerate(stationary_raw)
        )
        if len(set(stationary_targets)) != len(stationary_targets):
            raise ScenarioError("observer_motion.stationary_targets has duplicates")
        if observer != "uav1":
            raise ScenarioError("observer_motion.observer must be 'uav1'")
        if parent_frame != world_frame:
            raise ScenarioError(
                "observer_motion.parent_frame must equal world_frame"
            )
        if child_frame != targets[observer].child_frame_id:
            raise ScenarioError(
                "observer_motion.child_frame must equal uav1 child_frame_id"
            )
        if parent_frame == child_frame:
            raise ScenarioError(
                "observer_motion parent_frame and child_frame must differ"
            )
        if tf_authority != DYNAMIC_TF_AUTHORITY:
            raise ScenarioError(
                "observer_motion.tf_authority must be '{}'".format(
                    DYNAMIC_TF_AUTHORITY
                )
            )
        expected_stationary = set(targets) - {observer}
        if set(stationary_targets) != expected_stationary:
            raise ScenarioError(
                "observer_motion.stationary_targets must name every non-observer target"
            )
        observer_motion = ObserverMotionSpec(
            observer=observer,
            parent_frame=parent_frame,
            child_frame=child_frame,
            tf_authority=tf_authority,
            unique_dynamic_tf_authority=unique_tf,
            fixed_yaw=fixed_yaw,
            minimum_translation_m=minimum_translation_m,
            minimum_yaw_change_rad=minimum_yaw_change_rad,
            scored_phase=scored_phase,
            stationary_targets=stationary_targets,
        )

    random_raw = _mapping(root.get("randomization", {}), "randomization")
    if schema_version in (2, 4, 5, 6, 7):
        random_raw = _strict_keys(
            random_raw, {"position_offset_uniform"}, set(), "randomization"
        )
    limits_raw = _mapping(random_raw.get("position_offset_uniform", {}),
                          "randomization.position_offset_uniform")
    if schema_version in (2, 4, 5, 6, 7):
        limits_raw = _strict_keys(
            limits_raw,
            {"x", "y", "z", "yaw"},
            set(),
            "randomization.position_offset_uniform",
        )
    limits = {}
    for axis in ("x", "y", "z", "yaw"):
        limits[axis] = _number(limits_raw.get(axis, 0.0),
                               "randomization.position_offset_uniform.{}".format(axis), 0.0)
    if schema_version in (2, 4, 5, 6, 7) and any(value != 0.0 for value in limits.values()):
        raise ScenarioError(
            "schema-v{} randomization offsets must all be zero".format(
                schema_version
            )
        )
    offsets: Dict[str, PoseSpec] = {}
    for name in sorted(targets.keys()):
        offsets[name] = PoseSpec(*[
            _stable_uniform(seed, name, axis, limits[axis]) for axis in ("x", "y", "z", "yaw")
        ])
        targets[name] = replace(targets[name], initial_pose=_add_pose(targets[name].initial_pose,
                                                                      offsets[name]))

    phases_raw = root.get("phases")
    if not isinstance(phases_raw, list) or not phases_raw:
        raise ScenarioError("phases must be a non-empty YAML sequence")
    phases: List[PhaseSpec] = []
    prior_goals = {name: target.initial_pose for name, target in targets.items()}
    seen_phase_ids = set()
    seen_events = set()
    for index, phase_value in enumerate(phases_raw):
        context = "phases[{}]".format(index)
        item = _mapping(phase_value, context)
        if schema_version in (2, 4, 5, 6, 7):
            item = _strict_keys(
                item,
                S03_PHASE_REQUIRED_KEYS,
                S03_PHASE_OPTIONAL_KEYS,
                context,
            )
        phase_id = _string(item.get("id"), context + ".id")
        if phase_id in seen_phase_ids:
            raise ScenarioError("duplicate phase id '{}'".format(phase_id))
        seen_phase_ids.add(phase_id)
        duration_ns = _duration_ns(item.get("duration"), context + ".duration")
        interpolation = _string(item.get("interpolation", "smoothstep"), context + ".interpolation")
        if interpolation not in ALLOWED_INTERPOLATIONS:
            raise ScenarioError("{}.interpolation must be one of {}".format(
                context, ALLOWED_INTERPOLATIONS))
        event_value = item.get("event")
        event = None if event_value is None else _string(event_value, context + ".event")
        if event in seen_events:
            raise ScenarioError("phase event '{}' is duplicated".format(event))
        if event:
            seen_events.add(event)

        goals = dict(prior_goals)
        phase_targets = _mapping(item.get("targets", {}), context + ".targets")
        unknown_targets = set(phase_targets.keys()) - set(targets.keys())
        if unknown_targets:
            raise ScenarioError("{} references unknown targets {}".format(context,
                                                                          sorted(unknown_targets)))
        for name, value in phase_targets.items():
            nominal = _pose(value, "{}.targets.{}".format(context, name))
            goal = _add_pose(nominal, offsets[name])
            if (
                schema_version in (2, 4, 5, 6, 7)
                and not targets[name].command_model
                and not _pose_close(goal, prior_goals[name])
            ):
                raise ScenarioError(
                    "{} changes uncommanded entity '{}'".format(context, name)
                )
            goals[name] = goal
        if interpolation == "hold":
            changed = [name for name in goals if not _pose_close(goals[name], prior_goals[name])]
            if changed:
                raise ScenarioError("{} uses hold but changes targets {}".format(context, changed))
        phases.append(PhaseSpec(phase_id, duration_ns, interpolation, goals, event))
        prior_goals = goals

    sum_duration_ns = sum(phase.duration_ns for phase in phases)
    declared_duration = root.get("duration")
    if declared_duration is not None:
        declared_ns = _duration_ns(declared_duration, "duration")
        if abs(declared_ns - sum_duration_ns) > 1:
            raise ScenarioError("duration does not equal the sum of phase durations")

    if duration_override is not None:
        override_ns = _duration_ns(duration_override, "~duration")
        scaled = []
        remaining_ns = override_ns
        remaining_source_ns = sum_duration_ns
        for index, phase in enumerate(phases):
            if index == len(phases) - 1:
                new_ns = remaining_ns
            else:
                new_ns = max(1, int(round(remaining_ns * phase.duration_ns / remaining_source_ns)))
                max_allowed = remaining_ns - (len(phases) - index - 1)
                new_ns = min(new_ns, max_allowed)
            if new_ns <= 0:
                raise ScenarioError("~duration is too short for {} positive phases".format(len(phases)))
            scaled.append(replace(phase, duration_ns=new_ns))
            remaining_ns -= new_ns
            remaining_source_ns -= phase.duration_ns
        phases = scaled
        sum_duration_ns = sum(phase.duration_ns for phase in phases)

    starts = []
    cursor = 0
    for phase in phases:
        starts.append(cursor)
        cursor += phase.duration_ns

    if schema_version == 2:
        if repeat_count != 1 or repeat_mode != "restart":
            raise ScenarioError(
                "schema-v2 S03 requires repeat_count=1 and repeat_mode=restart"
            )
        _validate_observer_motion_trajectory(
            targets, tuple(phases), observer_motion
        )
    elif schema_version == 4:
        _validate_observer_motion_trajectory(
            targets, tuple(phases), observer_motion
        )
        _validate_s04_exact_contract(
            seed,
            world_frame,
            repeat_count,
            repeat_mode,
            start_delay_ns,
            final_hold_ns,
            targets,
            tuple(phases),
            sum_duration_ns,
            runtime,
            observer_motion,
        )
    elif schema_version == 5:
        _validate_observer_motion_trajectory(
            targets, tuple(phases), observer_motion
        )
        _validate_s05_exact_contract(
            seed,
            world_frame,
            repeat_count,
            repeat_mode,
            start_delay_ns,
            final_hold_ns,
            targets,
            tuple(phases),
            sum_duration_ns,
            runtime,
            observer_motion,
        )
    elif schema_version == 6:
        _validate_observer_motion_trajectory(
            targets, tuple(phases), observer_motion
        )
        _validate_s06_exact_contract(
            seed,
            world_frame,
            repeat_count,
            repeat_mode,
            start_delay_ns,
            final_hold_ns,
            targets,
            tuple(phases),
            sum_duration_ns,
            runtime,
            observer_motion,
            online_map_contract,
        )
    elif schema_version == 7:
        _validate_s02_exact_contract(
            seed,
            world_frame,
            repeat_count,
            repeat_mode,
            start_delay_ns,
            final_hold_ns,
            targets,
            tuple(phases),
            sum_duration_ns,
            runtime,
            s02_crossing_contract,
        )

    return ScenarioSpec(scenario_id, seed, world_frame, repeat_count, repeat_mode,
                        start_delay_ns,
                        final_hold_ns, targets, tuple(phases), tuple(starts),
                        sum_duration_ns, offsets, runtime,
                        schema_version=schema_version,
                        scenario_profile=scenario_profile,
                        semantic_contract_id=semantic_contract_id,
                        scenario_contract_id=scenario_contract_id,
                        observer_motion=observer_motion,
                        online_map_contract=online_map_contract)


class ClockMonitor:
    def __init__(self):
        self._lock = threading.Lock()
        self._stamp_ns: Optional[int] = None
        self._last_wall = 0.0
        self._subscriber = rospy.Subscriber("/clock", Clock, self._callback, queue_size=10)

    def _callback(self, message: Clock) -> None:
        with self._lock:
            self._stamp_ns = message.clock.to_nsec()
            self._last_wall = time.monotonic()

    def sample(self) -> Tuple[Optional[int], float]:
        with self._lock:
            return self._stamp_ns, self._last_wall

    def wait_for_clock(self, timeout_wall: float) -> int:
        deadline = time.monotonic() + timeout_wall
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            stamp_ns, _ = self.sample()
            if stamp_ns is not None:
                return stamp_ns
            rospy.rostime.wallsleep(0.02)
        if rospy.is_shutdown():
            raise ScenarioError("ROS shut down while waiting for /clock")
        raise ScenarioError("timed out after {:.1f}s wall time waiting for /clock".format(timeout_wall))


class ScenarioManager:
    def __init__(self, spec: ScenarioSpec):
        self.spec = spec
        self.clock = ClockMonitor()
        self.event_pub = rospy.Publisher(spec.runtime.event_topic, String, queue_size=50, latch=True)
        self.truth_pubs = {
            name: rospy.Publisher(target.truth_topic, Odometry, queue_size=20)
            for name, target in spec.targets.items()
        }
        self.truth_sequences = {name: 0 for name in spec.targets}
        self.service = rospy.ServiceProxy(spec.runtime.service_name, SetModelState, persistent=False)
        self.world_properties = rospy.ServiceProxy(
            spec.runtime.world_properties_service, GetWorldProperties, persistent=False
        )
        self.dynamic_tf_broadcaster = create_dynamic_tf_broadcaster(
            spec.observer_motion
        )
        self.last_dynamic_tf_stamp_ns: Optional[int] = None
        self.last_dynamic_tf_state: Optional[MotionState] = None
        self.consecutive_failures = 0
        self.last_commanded_states: Dict[str, MotionState] = {}
        self.started = False
        self.terminal = False
        self.current_cycle = 0
        self.last_sim_ns: Optional[int] = None
        self.last_progress_wall = time.monotonic()
        self.final_states = self._phase_endpoint_states(
            len(spec.phases) - 1, spec.repeat_count
        )
        rospy.on_shutdown(self._on_shutdown)

    @staticmethod
    def _shortest_yaw_delta(start: float, goal: float) -> float:
        return math.atan2(math.sin(goal - start), math.cos(goal - start))

    def _phase_start_poses(self, phase_index: int) -> Mapping[str, PoseSpec]:
        if phase_index == 0:
            return {name: target.initial_pose for name, target in self.spec.targets.items()}
        return self.spec.phases[phase_index - 1].goals

    def _cycle_pose(self, name: str, pose: PoseSpec, cycle: int) -> PoseSpec:
        if self.spec.repeat_mode != "ping_pong" or cycle % 2 == 1:
            return pose
        initial = self.spec.targets[name].initial_pose
        endpoint = self.spec.phases[-1].goals[name]
        reflected_yaw = initial.yaw + endpoint.yaw - pose.yaw
        reflected_yaw = math.atan2(math.sin(reflected_yaw), math.cos(reflected_yaw))
        return PoseSpec(
            initial.x + endpoint.x - pose.x,
            initial.y + endpoint.y - pose.y,
            initial.z + endpoint.z - pose.z,
            reflected_yaw,
        )

    def _phase_endpoint_states(
        self, phase_index: int, cycle: int = 1
    ) -> Mapping[str, MotionState]:
        return {
            name: MotionState(self._cycle_pose(name, pose, cycle))
            for name, pose in self.spec.phases[phase_index].goals.items()
        }

    def _sample_phase(
        self, phase_index: int, local_ns: int, cycle: int = 1
    ) -> Mapping[str, MotionState]:
        phase = self.spec.phases[phase_index]
        starts = self._phase_start_poses(phase_index)
        duration_s = phase.duration_ns / float(NSEC_PER_SEC)
        tau = min(1.0, max(0.0, local_ns / float(phase.duration_ns)))
        if phase.interpolation == "hold":
            scale, derivative = 0.0, 0.0
        elif phase.interpolation == "linear":
            scale, derivative = tau, 1.0 / duration_s
        elif phase.interpolation == "smoothstep":
            scale = tau * tau * (3.0 - 2.0 * tau)
            derivative = 6.0 * tau * (1.0 - tau) / duration_s
        elif phase.interpolation == "minimum_snap":
            # Seventh-order time scaling with zero velocity, acceleration and
            # jerk at both endpoints.  Consecutive segments and hold phases
            # therefore join with C3 continuity without an online optimizer.
            if tau <= 0.0:
                scale, derivative = 0.0, 0.0
            elif tau >= 1.0:
                scale, derivative = 1.0, 0.0
            else:
                tau2 = tau * tau
                tau3 = tau2 * tau
                tau4 = tau3 * tau
                scale = tau4 * (
                    35.0 + tau * (-84.0 + tau * (70.0 - 20.0 * tau))
                )
                derivative = (
                    140.0 * tau3
                    - 420.0 * tau4
                    + 420.0 * tau4 * tau
                    - 140.0 * tau4 * tau2
                ) / duration_s
        else:
            raise ScenarioError(
                "unsupported phase interpolation '{}'".format(
                    phase.interpolation
                )
            )

        result = {}
        for name in sorted(self.spec.targets.keys()):
            start = starts[name]
            goal = phase.goals[name]
            dx, dy, dz = goal.x - start.x, goal.y - start.y, goal.z - start.z
            dyaw = self._shortest_yaw_delta(start.yaw, goal.yaw)
            if tau <= 0.0:
                pose = start
            elif tau >= 1.0:
                pose = goal
            else:
                pose = PoseSpec(
                    start.x + scale * dx,
                    start.y + scale * dy,
                    start.z + scale * dz,
                    start.yaw + scale * dyaw,
                )
            state = MotionState(pose, derivative * dx, derivative * dy,
                                derivative * dz, derivative * dyaw)
            if self.spec.repeat_mode == "ping_pong" and cycle % 2 == 0:
                state = MotionState(
                    self._cycle_pose(name, state.pose, cycle),
                    -state.vx,
                    -state.vy,
                    -state.vz,
                    -state.yaw_rate,
                )
            result[name] = state
        return result

    @staticmethod
    def _ros_time(stamp_ns: int) -> rospy.Time:
        return rospy.Time(stamp_ns // NSEC_PER_SEC, stamp_ns % NSEC_PER_SEC)

    @staticmethod
    def _quaternion_z(yaw: float) -> Tuple[float, float, float, float]:
        half = 0.5 * yaw
        return 0.0, 0.0, math.sin(half), math.cos(half)

    def _set_target(self, target: TargetSpec, state: MotionState) -> None:
        request = SetModelStateRequest()
        request.model_state = ModelState()
        request.model_state.model_name = target.model_name
        request.model_state.reference_frame = self.spec.world_frame
        request.model_state.pose.position.x = state.pose.x
        request.model_state.pose.position.y = state.pose.y
        request.model_state.pose.position.z = state.pose.z
        qx, qy, qz, qw = self._quaternion_z(state.pose.yaw)
        request.model_state.pose.orientation.x = qx
        request.model_state.pose.orientation.y = qy
        request.model_state.pose.orientation.z = qz
        request.model_state.pose.orientation.w = qw
        request.model_state.twist.linear.x = state.vx
        request.model_state.twist.linear.y = state.vy
        request.model_state.twist.linear.z = state.vz
        request.model_state.twist.angular.z = state.yaw_rate
        response = self.service(request)
        if not response.success:
            raise ScenarioError("{} rejected model '{}': {}".format(
                self.spec.runtime.service_name, target.model_name, response.status_message))

    def _publish_truth(self, name: str, target: TargetSpec, state: MotionState, stamp_ns: int) -> None:
        message = Odometry()
        message.header.seq = self.truth_sequences[name]
        self.truth_sequences[name] += 1
        message.header.stamp = self._ros_time(stamp_ns)
        message.header.frame_id = self.spec.world_frame
        message.child_frame_id = target.child_frame_id
        message.pose.pose.position.x = state.pose.x
        message.pose.pose.position.y = state.pose.y
        message.pose.pose.position.z = state.pose.z
        qx, qy, qz, qw = self._quaternion_z(state.pose.yaw)
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw

        # nav_msgs/Odometry defines twist in child_frame_id.  ModelState uses
        # world-frame linear velocity, so rotate it into the target body frame.
        cosine, sine = math.cos(state.pose.yaw), math.sin(state.pose.yaw)
        message.twist.twist.linear.x = cosine * state.vx + sine * state.vy
        message.twist.twist.linear.y = -sine * state.vx + cosine * state.vy
        message.twist.twist.linear.z = state.vz
        message.twist.twist.angular.z = state.yaw_rate
        self.truth_pubs[name].publish(message)

    def _publish_dynamic_observer_tf(
        self, state: MotionState, stamp_ns: int
    ) -> bool:
        motion = self.spec.observer_motion
        if motion is None:
            if self.dynamic_tf_broadcaster is not None:
                raise ScenarioError(
                    "schema-v1 scenario unexpectedly owns a dynamic TF broadcaster"
                )
            return False
        if self.dynamic_tf_broadcaster is None:
            raise ScenarioError("dynamic observer TF broadcaster is unavailable")
        if not _should_publish_dynamic_tf(
            self.last_dynamic_tf_stamp_ns,
            self.last_dynamic_tf_state,
            stamp_ns,
            state,
        ):
            return False
        message = build_dynamic_observer_transform(motion, state, stamp_ns)
        self.dynamic_tf_broadcaster.sendTransform(message)
        self.last_dynamic_tf_stamp_ns = stamp_ns
        self.last_dynamic_tf_state = state
        return True

    def _apply_and_publish(self, states: Mapping[str, MotionState], stamp_ns: int,
                           enforce_failure_limit: bool = True) -> bool:
        try:
            for name in sorted(self.spec.targets.keys()):
                target = self.spec.targets[name]
                # Moving-observer schemas have stationary targets and long
                # holds. Reissuing identical synchronous Gazebo service calls
                # at 50 Hz can stall the sole dynamic-TF authority for several
                # scan periods. Once an identical state was successfully
                # committed, keep publishing TF/truth but avoid the redundant
                # model write. Static-observer schemas preserve their
                # historical behavior.
                previous = self.last_commanded_states.get(name)
                unchanged_moving_observer_state = (
                    self.spec.observer_motion is not None
                    and previous is not None
                    and _motion_state_close(previous, states[name])
                )
                if target.command_model and not unchanged_moving_observer_state:
                    self._set_target(target, states[name])
            if self.spec.observer_motion is not None:
                observer = self.spec.observer_motion.observer
                self._publish_dynamic_observer_tf(states[observer], stamp_ns)
            for name in sorted(self.spec.targets.keys()):
                self._publish_truth(name, self.spec.targets[name], states[name], stamp_ns)
            self.last_commanded_states = dict(states)
            self.consecutive_failures = 0
            return True
        except (rospy.ServiceException, ScenarioError) as exc:
            self.consecutive_failures += 1
            rospy.logerr_throttle(1.0, "scenario state update failed (%d/%d): %s",
                                  self.consecutive_failures,
                                  self.spec.runtime.max_consecutive_service_failures, exc)
            if (enforce_failure_limit and
                    self.consecutive_failures >= self.spec.runtime.max_consecutive_service_failures):
                raise ScenarioError("Gazebo state update failed {} consecutive times: {}".format(
                    self.consecutive_failures, exc))
            return False

    def _event(self, event: str, cycle: int, stamp_ns: int, **extra) -> None:
        payload = {
            "cycle": cycle,
            "event": event,
            "scenario": self.spec.scenario_id,
            "seed": self.spec.seed,
            "repeat_mode": self.spec.repeat_mode,
            "sim_time": stamp_ns / float(NSEC_PER_SEC),
            "sim_time_ns": stamp_ns,
        }
        if self.spec.scenario_profile is not None:
            payload["scenario_profile"] = self.spec.scenario_profile
            payload["semantic_contract_id"] = self.spec.semantic_contract_id
        if self.spec.scenario_contract_id is not None:
            payload["scenario_contract_id"] = self.spec.scenario_contract_id
        if self.spec.online_map_contract is not None:
            payload["online_map_contract"] = dict(
                self.spec.online_map_contract
            )
        if self.spec.observer_motion is not None:
            payload["dynamic_tf_authority"] = self.spec.observer_motion.tf_authority
        # Including the authoritative boundary pose removes cross-topic callback
        # ordering ambiguity in repeatability tests while Odometry remains the
        # full-rate truth interface.
        payload["commanded_pose"] = {
            name: {
                "x": round(state.pose.x, 12),
                "y": round(state.pose.y, 12),
                "yaw": round(state.pose.yaw, 12),
                "z": round(state.pose.z, 12),
            }
            for name, state in sorted(self.last_commanded_states.items())
        }
        payload.update(extra)
        self.event_pub.publish(String(data=json.dumps(payload, sort_keys=True, separators=(",", ":"))))
        rospy.loginfo("scenario event: %s", payload)

    def _initial_states(self) -> Mapping[str, MotionState]:
        return {name: MotionState(target.initial_pose) for name, target in self.spec.targets.items()}

    def _wait_for_models(self, states: Mapping[str, MotionState], stamp_ns: int) -> int:
        """Wait for spawn_model nodes without consuming /gazebo/model_states.

        Querying the model-name service before the first write avoids treating
        normal roslaunch spawn ordering as a failed state command.
        """
        required_models = {
            target.model_name
            for target in self.spec.targets.values()
            if target.command_model
        }
        deadline = time.monotonic() + self.spec.runtime.service_timeout_wall
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                world = self.world_properties()
                available = set(world.model_names) if world.success else set()
                if required_models.issubset(available):
                    apply_stamp_ns = stamp_ns
                    if self.spec.observer_motion is not None:
                        # A moving-observer scenario may spend several simulated
                        # tenths waiting for spawn_model. Do not seed the first dynamic TF with
                        # the stale pre-spawn dependency stamp, which would
                        # create an artificial startup hole.  Schema-v1 keeps
                        # its historical timestamp baseline.
                        current_stamp_ns, _ = self.clock.sample()
                        if current_stamp_ns is None:
                            rospy.rostime.wallsleep(0.1)
                            continue
                        apply_stamp_ns = current_stamp_ns
                    if self._apply_and_publish(
                        states, apply_stamp_ns, enforce_failure_limit=False
                    ):
                        return apply_stamp_ns
            except rospy.ServiceException:
                # Gazebo may still be completing API initialization.  The wall
                # deadline below remains authoritative and deterministic.
                pass
            rospy.rostime.wallsleep(0.1)
        if rospy.is_shutdown():
            raise ScenarioError("ROS shut down while waiting for target models")
        raise ScenarioError("target models were not writable through {} within {:.1f}s".format(
            self.spec.runtime.service_name, self.spec.runtime.service_timeout_wall))

    def _apply_boundary(self, states: Mapping[str, MotionState], stamp_ns: int) -> None:
        """Boundary states/events are mandatory; transient regular samples are not."""
        while not rospy.is_shutdown():
            if self._apply_and_publish(states, stamp_ns):
                return
            rospy.rostime.wallsleep(0.02)
        raise ScenarioError("ROS shut down while applying a scenario boundary")

    def _wait_for_dependencies(self) -> int:
        if self.spec.runtime.require_use_sim_time and not rospy.get_param("/use_sim_time", False):
            raise ScenarioError("/use_sim_time must be true")
        rospy.loginfo("waiting up to %.1fs for %s", self.spec.runtime.service_timeout_wall,
                      self.spec.runtime.service_name)
        try:
            rospy.wait_for_service(self.spec.runtime.service_name,
                                   timeout=self.spec.runtime.service_timeout_wall)
            rospy.wait_for_service(self.spec.runtime.world_properties_service,
                                   timeout=self.spec.runtime.service_timeout_wall)
        except rospy.ROSException as exc:
            raise ScenarioError("Gazebo service unavailable: {}".format(exc))
        rospy.loginfo("waiting up to %.1fs for /clock", self.spec.runtime.clock_timeout_wall)
        return self.clock.wait_for_clock(self.spec.runtime.clock_timeout_wall)

    def _check_clock(self, stamp_ns: int) -> None:
        now_wall = time.monotonic()
        if self.last_sim_ns is None or stamp_ns > self.last_sim_ns:
            self.last_progress_wall = now_wall
        elif stamp_ns < self.last_sim_ns:
            raise ScenarioError("/clock moved backwards from {} to {} ns".format(
                self.last_sim_ns, stamp_ns))
        elif now_wall - self.last_progress_wall > self.spec.runtime.max_clock_stall_wall:
            raise ScenarioError("/clock stalled for {:.1f}s wall time".format(
                now_wall - self.last_progress_wall))
        self.last_sim_ns = stamp_ns

    def _emit_phase_start(self, phase_index: int, cycle: int, stamp_ns: int,
                          states: Mapping[str, MotionState]) -> None:
        phase = self.spec.phases[phase_index]
        if phase.event:
            self._apply_boundary(states, stamp_ns)
            self._event(phase.event, cycle, stamp_ns, phase=phase.phase_id,
                        phase_index=phase_index)

    def run(self) -> None:
        dependency_stamp_ns = self._wait_for_dependencies()
        initial_states = self._initial_states()
        model_ready_stamp_ns = self._wait_for_models(
            initial_states, dependency_stamp_ns
        )
        self.started = True

        start_delay_end_ns = model_ready_stamp_ns + self.spec.start_delay_ns
        cycle = 0
        phase_index = -1
        cycle_start_ns = start_delay_end_ns
        next_boundary_ns = start_delay_end_ns
        mode = "start_delay"
        final_hold_end_ns = 0
        sleep_s = min(0.05, 1.0 / self.spec.runtime.publish_rate)

        rospy.loginfo(
            "loaded scenario=%s profile=%s seed=%d cycles=%d cycle_duration=%.3fs start_delay=%.3fs final_hold=%.3fs",
            self.spec.scenario_id,
            self.spec.scenario_profile or "schema-v1",
            self.spec.seed,
            self.spec.repeat_count,
            self.spec.cycle_duration_ns / float(NSEC_PER_SEC),
            self.spec.start_delay_ns / float(NSEC_PER_SEC),
            self.spec.final_hold_ns / float(NSEC_PER_SEC))

        while not rospy.is_shutdown() and mode != "done":
            stamp_ns, _ = self.clock.sample()
            if stamp_ns is None:
                rospy.rostime.wallsleep(sleep_s)
                continue
            self._check_clock(stamp_ns)

            # Process every crossed boundary, using its exact sim-time stamp.
            # This preserves deterministic event ordering even if Gazebo jumps.
            while mode != "done" and stamp_ns >= next_boundary_ns:
                boundary_ns = next_boundary_ns
                if mode == "start_delay":
                    cycle = 1
                    self.current_cycle = cycle
                    cycle_start_ns = boundary_ns
                    phase_index = 0
                    boundary_states = self._sample_phase(0, 0, cycle)
                    self._apply_boundary(boundary_states, boundary_ns)
                    self._event("cycle_start", cycle, boundary_ns,
                                phase=self.spec.phases[0].phase_id, phase_index=0)
                    mode = "cycle"
                    next_boundary_ns = cycle_start_ns + self.spec.phases[0].duration_ns
                    continue

                if mode == "cycle":
                    if phase_index + 1 < len(self.spec.phases):
                        phase_index += 1
                        phase_start_ns = cycle_start_ns + self.spec.phase_starts_ns[phase_index]
                        boundary_states = self._sample_phase(phase_index, 0, cycle)
                        self._emit_phase_start(phase_index, cycle, phase_start_ns, boundary_states)
                        next_boundary_ns = phase_start_ns + self.spec.phases[phase_index].duration_ns
                        continue

                    # Sample the exact endpoint before cycle_complete.  The
                    # A ping-pong contract maps the next cycle's phase-0 pose
                    # to this endpoint, avoiding an unobservable teleport.
                    cycle_endpoint = self._phase_endpoint_states(
                        len(self.spec.phases) - 1, cycle
                    )
                    self._apply_boundary(cycle_endpoint, boundary_ns)
                    self._event("cycle_complete", cycle, boundary_ns,
                                phase=self.spec.phases[-1].phase_id,
                                phase_index=len(self.spec.phases) - 1)
                    if cycle < self.spec.repeat_count:
                        cycle += 1
                        self.current_cycle = cycle
                        cycle_start_ns = boundary_ns
                        phase_index = 0
                        boundary_states = self._sample_phase(0, 0, cycle)
                        self._apply_boundary(boundary_states, boundary_ns)
                        self._event("cycle_start", cycle, boundary_ns,
                                    phase=self.spec.phases[0].phase_id, phase_index=0)
                        next_boundary_ns = cycle_start_ns + self.spec.phases[0].duration_ns
                    else:
                        mode = "final_hold"
                        final_hold_end_ns = boundary_ns + self.spec.final_hold_ns
                        self._apply_boundary(self.final_states, boundary_ns)
                        self._event("final_hold_start", cycle, boundary_ns,
                                    duration=self.spec.final_hold_ns / float(NSEC_PER_SEC))
                        next_boundary_ns = final_hold_end_ns
                    continue

                if mode == "final_hold":
                    self._apply_boundary(self.final_states, boundary_ns)
                    self._event("scenario_complete", cycle, boundary_ns,
                                repeat_count=self.spec.repeat_count)
                    self.terminal = True
                    mode = "done"

            if mode == "start_delay":
                states = initial_states
            elif mode == "cycle":
                phase_start_ns = cycle_start_ns + self.spec.phase_starts_ns[phase_index]
                states = self._sample_phase(
                    phase_index, stamp_ns - phase_start_ns, cycle
                )
            elif mode == "final_hold":
                states = self.final_states
            else:
                break
            self._apply_and_publish(states, stamp_ns)
            rospy.rostime.wallsleep(sleep_s)

        if rospy.is_shutdown() and not self.terminal:
            raise ScenarioError("ROS shut down before scenario completion")

        # Moving-observer contracts own the only dynamic world -> observer TF
        # authority. Gazebo and
        # the LiDAR sensor remain alive after the finite scenario timeline has
        # emitted scenario_complete, so letting this node exit would leave the
        # TF buffer frozen while new per-ray bundles continue to arrive.  Keep
        # the final commanded state, truth and dynamic TF current until the
        # enclosing launch is shut down.  Schema-v1 scenarios deliberately
        # retain their historical finite-process behavior.
        if self.terminal and self.spec.observer_motion is not None:
            while not rospy.is_shutdown():
                stamp_ns, _ = self.clock.sample()
                if stamp_ns is None:
                    rospy.rostime.wallsleep(sleep_s)
                    continue
                self._check_clock(stamp_ns)
                self._apply_and_publish(self.final_states, stamp_ns)
                rospy.rostime.wallsleep(sleep_s)

    def fail(self, reason: str) -> None:
        if self.started and not rospy.is_shutdown():
            stamp_ns, _ = self.clock.sample()
            if stamp_ns is not None:
                self._event("scenario_failed", self.current_cycle, stamp_ns, reason=reason)
        self.terminal = True

    def _on_shutdown(self) -> None:
        if self.started and not self.terminal:
            rospy.logwarn("scenario interrupted by ROS shutdown")


def default_scenario_path() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config", "scenarios", "S01.yaml"))


def main() -> int:
    rospy.init_node("mid360_scenario_manager", anonymous=False)
    manager: Optional[ScenarioManager] = None
    try:
        scenario_path = rospy.get_param("~scenario_file", default_scenario_path())
        spec = load_scenario(
            scenario_path,
            seed_override=_get_optional_param("~seed"),
            duration_override=_get_optional_param("~duration"),
            repeat_count_override=_get_optional_param("~repeat_count"),
            final_hold_override=_get_optional_param("~final_hold"),
            start_delay_override=_get_optional_param("~start_delay"),
        )
        manager = ScenarioManager(spec)
        manager.run()
        return 0
    except ScenarioError as exc:
        rospy.logfatal("scenario manager failed: %s", exc)
        if manager is not None:
            manager.fail(str(exc))
        return 2
    except rospy.ROSInterruptException:
        rospy.logwarn("scenario manager interrupted")
        return 130
    except Exception as exc:  # Keep launch failure explicit without a traceback loop.
        rospy.logfatal("unexpected scenario manager failure: %s", exc)
        if manager is not None:
            manager.fail("unexpected: {}".format(exc))
        return 3


if __name__ == "__main__":
    sys.exit(main())
