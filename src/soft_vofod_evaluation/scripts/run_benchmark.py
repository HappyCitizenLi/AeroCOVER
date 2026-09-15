#!/usr/bin/env python3
"""Record paired LiDAR inputs once and replay the paper methods."""

import argparse
import csv
import datetime
import hashlib
import json
import math
import os
import platform
import signal
import socket
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rosbag
import yaml


ALGORITHMS = ('VoFOD-Mid360', 'VoFOD-OS1', 'AeroCOVER-Mid360', 'AeroCOVER-OS1', 'AeroCOVER-A1', 'AeroCOVER-A2', 'AeroCOVER-A3', 'AeroCOVER-A4', 'AeroCOVER-A5')
SCENES = ('OPEN', 'MT', 'OFFICE', 'FOREST')
NOISE_LEVELS = ("N0", "N03")

WORLD_FILES = {
    "PW_office": "PW_office.world",
    "PW_forest_seed0": "PW_forest_seed0.world",
    "PW_open": "PW_open.world",
    "PW_mt": "PW_mt.world",
}
NOISE_STDDEV_M = {"N0": 0.0, "N03": 0.03}
REPLAY_ADVERTISE_DELAY_S = 1.0
REPLAY_DRAIN_DELAY_S = 30.0

PHYSICAL_LIMITS = {
    "observer_clearance_m": 0.40,
    "target_clearance_m": 0.35,
    "minimum_vehicle_separation_m": 1.30,
    "maximum_horizontal_speed_mps": 5.0,
    "maximum_horizontal_acceleration_mps2": 5.0,
    "maximum_vertical_speed_mps": 2.0,
    "maximum_vertical_acceleration_mps2": 3.0,
}


def nested(mapping, *keys):
    for key in keys:
        if not isinstance(mapping, dict):
            return None
        mapping = mapping.get(key)
    return mapping


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def combined_sha256(paths):
    digest = hashlib.sha256()
    for path in sorted(map(Path, paths)):
        digest.update(path.name.encode())
        digest.update(sha256(path).encode())
    return digest.hexdigest()


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def comma_list(value, allowed):
    values = tuple(item.strip() for item in value.split(",") if item.strip())
    invalid = sorted(set(values) - set(allowed))
    if invalid:
        raise argparse.ArgumentTypeError("unsupported values: " + ",".join(invalid))
    return values


def free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def algorithm_command(algorithm, config, tracker_override=None):
    root = Path(__file__).resolve().parents[3]
    if algorithm == "VoFOD-Mid360":
        return ["roslaunch", "tclv_evaluation", "b0_canonical.launch",
                f"b0_config:={config}",
                "method_config:=" + str(root / "src/vofod_mid360/config/"
                    "vofod_original.yaml"), "output:=log"] + (
                        [f"tracker_override_config:={tracker_override}"] if tracker_override else [])
    if algorithm == "VoFOD-OS1":
        return ["roslaunch", "tclv_evaluation", "ouster_original.launch",
                f"b0_config:={config}", "output:=log"] + (
                    [f"tracker_override_config:={tracker_override}"] if tracker_override else [])
    if algorithm == "AeroCOVER-OS1":
        return ["roslaunch", "aerocover_mid360", "aerocover_os1.launch",
                f"config:={config}", "output:=log"]
    if algorithm == "AeroCOVER-Mid360" or \
            algorithm.startswith("AeroCOVER-A"):
        return ["roslaunch", "aerocover_mid360", "aerocover_mvp.launch",
                f"config:={config}", "output:=log"]
    raise ValueError(f"unsupported algorithm: {algorithm}")


def recorded_topics(algorithm):
    if algorithm == "VoFOD-Mid360":
        return (
            "/uav1/batch/b0/tracks", "/uav1/vofod_mid360/background_points",
            "/uav1/vofod_mid360/free_voxels",
            "/uav1/vofod_mid360/map_update_diagnostics",
            "/uav1/vofod_mid360/profiling_info",
            "/uav1/batch/b0/lidar_tracker_mid360/profiling_info",
            "/uav1/batch/b0/lidar_tracker_mid360/frame_complete")
    if algorithm == "VoFOD-OS1":
        return (
            "/uav1/ouster_vofod/tracks",
            "/uav1/vofod_mid360/background_points",
            "/uav1/vofod_mid360/map_update_diagnostics",
            "/uav1/ouster_vofod/lidar_tracker/profiling_info",
            "/uav1/ouster_vofod/lidar_tracker/frame_complete")
    if algorithm == "AeroCOVER-OS1":
        return ("/aerocover_os1/tracks", "/aerocover_os1/background_points",
                "/aerocover_os1/diagnostics")
    if algorithm == "AeroCOVER-Mid360" or \
            algorithm.startswith("AeroCOVER-A"):
        return (
            "/aerocover/tracks", "/aerocover/background_points",
            "/aerocover/diagnostics")
    raise ValueError(f"unsupported algorithm: {algorithm}")


def terminate(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def wait_for_topic(topic, environment, process, timeout=45.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"process exited while waiting for {topic}")
        listed = subprocess.run(
            ["rostopic", "list"], env=environment, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        if listed.returncode == 0 and topic in listed.stdout.splitlines():
            return
        time.sleep(0.25)
    raise RuntimeError(f"timeout waiting for {topic}")


def target_primitives():
    output = [
        {"id": "body", "type": "cylinder", "pose": [0, 0, 0, 0, 0, 0],
         "radius": .10, "length": .10},
        {"id": "arm_a", "type": "box",
         "pose": [0, 0, 0, 0, 0, 0.7853981633974483],
         "size": [.52, .04, .04]},
        {"id": "arm_b", "type": "box",
         "pose": [0, 0, 0, 0, 0, -0.7853981633974483],
         "size": [.52, .04, .04]},
    ]
    for name, x, y in (("rotor_fl", .1812, .1812),
                       ("rotor_fr", .1812, -.1812),
                       ("rotor_rl", -.1812, .1812),
                       ("rotor_rr", -.1812, -.1812)):
        output.append({"id": name, "type": "cylinder",
                       "pose": [x, y, .057, 0, 0, 0],
                       "radius": .1651, "length": .02})
    return output




def static_primitives(world):
    output = [{"id": "ground", "type": "plane",
               "pose": [0, 0, 0, 0, 0, 0], "size": [160, 120, 0]}]
    if world == "PW_mt":
        model = ET.parse(Path(__file__).resolve().parents[3] /
            "src/mid360_multi_uav_sim/worlds/PW_mt.world").find("world/model[@name='formation_cylinder']")
        cylinder = model.find('link/collision/geometry/cylinder')
        output.append({"id": "formation_cylinder", "type": "cylinder",
                       "pose": list(map(float, model.findtext('pose').split())),
                       "radius": float(cylinder.findtext('radius')), "length": float(cylinder.findtext('length'))})
        return output
    if world == "PW_open":
        return output
    if world == "PW_office":
        output.extend([
            {"id": "office_corridor_north", "type": "box",
             "pose": [20.9, 9.166528, 1.4224, 0, 0,0],
             "size": [35.4, .1, 2.8448]},
            {"id": "office_corridor_south", "type": "box",
             "pose": [20.9, 7.333472, 1.4224, 0, 0, 0],
             "size": [35.4, .1, 2.8448]},
        ])
    elif world == "PW_forest_seed0":
        forest = ET.parse(
            Path(__file__).resolve().parents[3] /
            "src/mid360_multi_uav_sim/worlds/PW_forest_seed0.world"
        ).getroot().find("world/model[@name='planning_forest_seed0']")
        for link in forest.findall("link"):
            pose = list(map(float, link.findtext("pose").split()))
            cylinder = link.find("collision/geometry/cylinder")
            output.append({
                "id": link.attrib["name"], "type": "cylinder",
                "pose": pose,
                "radius": float(cylinder.findtext("radius")),
                "length": float(cylinder.findtext("length")),
            })
    else:
        raise ValueError(f"unsupported world: {world}")
    return output


def _ray_primitive_entry(origin, direction, primitive, epsilon=1.0e-9):
    """Closest nonnegative ray hit for the static primitive contract."""
    pose = primitive["pose"]
    if abs(pose[3]) > epsilon or abs(pose[4]) > epsilon:
        raise ValueError("static trace annotation supports yaw-only primitives")
    cosine, sine = math.cos(pose[5]), math.sin(pose[5])
    delta = (origin[0] - pose[0], origin[1] - pose[1],
             origin[2] - pose[2])
    local_origin = (cosine * delta[0] + sine * delta[1],
                    -sine * delta[0] + cosine * delta[1], delta[2])
    local_direction = (cosine * direction[0] + sine * direction[1],
                       -sine * direction[0] + cosine * direction[1],
                       direction[2])
    if primitive["type"] == "plane":
        if abs(local_direction[2]) <= epsilon:
            return None
        distance = -local_origin[2] / local_direction[2]
        x = local_origin[0] + distance * local_direction[0]
        y = local_origin[1] + distance * local_direction[1]
        size = primitive["size"]
        if distance < -epsilon or \
                (size[0] > 0.0 and abs(x) > size[0] / 2.0 + epsilon) or \
                (size[1] > 0.0 and abs(y) > size[1] / 2.0 + epsilon):
            return None
        return max(0.0, distance)
    if primitive["type"] == "box":
        entry, exit_distance = -math.inf, math.inf
        for value, slope, size in zip(
                local_origin, local_direction, primitive["size"]):
            half = size / 2.0
            if abs(slope) <= epsilon:
                if value < -half - epsilon or value > half + epsilon:
                    return None
                continue
            first, second = (-half - value) / slope, (half - value) / slope
            if first > second:
                first, second = second, first
            entry, exit_distance = max(entry, first), min(exit_distance, second)
            if entry > exit_distance + epsilon:
                return None
        return max(0.0, entry) if exit_distance >= -epsilon else None

    radius, half = primitive["radius"], primitive["length"] / 2.0
    x, y, z = local_origin
    dx, dy, dz = local_direction
    crossings = []
    a = dx * dx + dy * dy
    if a > epsilon:
        b, c = 2.0 * (x * dx + y * dy), x * x + y * y - radius * radius
        discriminant = b * b - 4.0 * a * c
        if discriminant >= -epsilon:
            root = math.sqrt(max(0.0, discriminant))
            for distance in ((-b - root) / (2.0 * a),
                             (-b + root) / (2.0 * a)):
                if -half - epsilon <= z + distance * dz <= half + epsilon:
                    crossings.append(distance)
    if abs(dz) > epsilon:
        for cap in (-half, half):
            distance = (cap - z) / dz
            if (x + distance * dx) ** 2 + (y + distance * dy) ** 2 \
                    <= radius * radius + epsilon:
                crossings.append(distance)
    if x * x + y * y <= radius * radius + epsilon and abs(z) <= half + epsilon:
        crossings.append(0.0)
    positive = [distance for distance in crossings if distance >= -epsilon]
    return max(0.0, min(positive)) if positive else None




def _waypoint_tangent(waypoints, index, scale):
    if index == 0 or index == len(waypoints) - 1:
        return (0.0, 0.0, 0.0)
    position = tuple(waypoints[index].get(axis, 0.) for axis in ('x', 'y', 'z'))
    if any(position == tuple(waypoints[j].get(axis, 0.) for axis in ('x', 'y', 'z'))
           for j in (index-1, index+1)):
        return (0.0, 0.0, 0.0)
    before, after = waypoints[index - 1], waypoints[index + 1]
    duration = float(after["t"]) - float(before["t"])
    return tuple(
        scale * (float(after.get(axis, 0.0)) -
                 float(before.get(axis, 0.0))) / duration
        for axis in ("x", "y", "z"))


def _trajectory_sample(waypoints, elapsed, profile, tangent_scale=.2):
    for index in range(1, len(waypoints)):
        if elapsed <= waypoints[index]["t"]:
            first, second = waypoints[index - 1], waypoints[index]
            duration = second["t"] - first["t"]
            alpha = max(0.0, min(1.0, (elapsed - first["t"]) / duration))
            if profile == "minimum_jerk":
                blend = 10.0 * alpha ** 3 - 15.0 * alpha ** 4 + 6.0 * alpha ** 5
                rate = (30.0 * alpha ** 2 - 60.0 * alpha ** 3 +
                        30.0 * alpha ** 4) / duration
                acceleration = (60.0 * alpha - 180.0 * alpha ** 2 +
                                120.0 * alpha ** 3) / duration ** 2
                delta = tuple(float(second.get(axis, 0.0)) -
                              float(first.get(axis, 0.0))
                              for axis in ("x", "y", "z"))
                return (
                    tuple(float(first.get(axis, 0.0)) + blend * change
                          for axis, change in zip(("x", "y", "z"), delta)),
                    tuple(rate * change for change in delta),
                    tuple(acceleration * change for change in delta))
            if profile == "continuous_cubic":
                first_rate = _waypoint_tangent(
                    waypoints, index - 1, tangent_scale)
                second_rate = _waypoint_tangent(
                    waypoints, index, tangent_scale)
                h00 = 2.0 * alpha ** 3 - 3.0 * alpha ** 2 + 1.0
                h10 = alpha ** 3 - 2.0 * alpha ** 2 + alpha
                h01 = -2.0 * alpha ** 3 + 3.0 * alpha ** 2
                h11 = alpha ** 3 - alpha ** 2
                dh00 = 6.0 * alpha ** 2 - 6.0 * alpha
                dh10 = 3.0 * alpha ** 2 - 4.0 * alpha + 1.0
                dh01 = -dh00
                dh11 = 3.0 * alpha ** 2 - 2.0 * alpha
                ddh00 = 12.0 * alpha - 6.0
                ddh10 = 6.0 * alpha - 4.0
                ddh01 = -ddh00
                ddh11 = 6.0 * alpha - 2.0
                first_position = tuple(
                    float(first.get(axis, 0.0)) for axis in ("x", "y", "z"))
                second_position = tuple(
                    float(second.get(axis, 0.0)) for axis in ("x", "y", "z"))
                position = tuple(
                    h00 * a + h10 * duration * da +
                    h01 * b + h11 * duration * db
                    for a, da, b, db in zip(
                        first_position, first_rate,
                        second_position, second_rate))
                velocity = tuple(
                    (dh00 * a + dh10 * duration * da +
                     dh01 * b + dh11 * duration * db) / duration
                    for a, da, b, db in zip(
                        first_position, first_rate,
                        second_position, second_rate))
                acceleration = tuple(
                    (ddh00 * a + ddh10 * duration * da +
                     ddh01 * b + ddh11 * duration * db) / duration ** 2
                    for a, da, b, db in zip(
                        first_position, first_rate,
                        second_position, second_rate))
                return position, velocity, acceleration
            delta = tuple(float(second.get(axis, 0.0)) -
                          float(first.get(axis, 0.0))
                          for axis in ("x", "y", "z"))
            return (
                tuple(float(first.get(axis, 0.0)) + alpha * change
                      for axis, change in zip(("x", "y", "z"), delta)),
                tuple(change / duration for change in delta),
                (0.0, 0.0, 0.0))
    position = tuple(float(waypoints[-1].get(axis, 0.0))
                     for axis in ("x", "y", "z"))
    return position, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)


def _point_clearance(point, primitive):
    if primitive["type"] == "plane":
        return point[2] - primitive["pose"][2]
    dx = point[0] - primitive["pose"][0]
    dy = point[1] - primitive["pose"][1]
    dz = point[2] - primitive["pose"][2]
    if primitive["type"] == "box":
        yaw = primitive["pose"][5]
        cosine, sine = math.cos(yaw), math.sin(yaw)
        local = (cosine * dx + sine * dy, -sine * dx + cosine * dy, dz)
        delta = tuple(max(0.0, abs(value) - size / 2.0)
                      for value, size in zip(local, primitive["size"]))
        return math.sqrt(sum(value * value for value in delta))
    radial = max(0.0, math.hypot(dx, dy) - primitive["radius"])
    vertical = max(0.0, abs(dz) - primitive["length"] / 2.0)
    return math.hypot(radial, vertical)


def validate_physical_scenario(scenario, sample_period_s=.02):
    """Reject opted-in kinematic scenes that violate conservative UAV limits."""
    if scenario.get("geometry_contract") == "four_scene_v1":
        helper = Path(__file__).resolve().parents[3] / "src/mid360_multi_uav_sim/scripts"
        sys.path.insert(0, str(helper))
        from check_four_scenes import check
        report = check(scenario, step=sample_period_s)
        if report["status"] != "PASS":
            raise ValueError(f"four-scene geometry validation failed: {report}")
        return report
    profile = scenario.get("trajectory_profile", "linear")
    if profile not in ("minimum_jerk", "continuous_cubic"):
        return {"status": "NOT_REQUESTED", "trajectory_profile": profile}
    tangent_scale = float(scenario.get("trajectory_tangent_scale", .2))
    if not math.isfinite(tangent_scale) or not 0.0 < tangent_scale <= 1.0:
        raise ValueError("trajectory_tangent_scale must be in (0, 1]")

    entities = [("uav1", scenario["observer"]["waypoints"], 0.0,
                 PHYSICAL_LIMITS["observer_clearance_m"])]
    entities.extend((target["id"], target["waypoints"],
                     float(target.get("motion_start_s", 0.0)),
                     PHYSICAL_LIMITS["target_clearance_m"])
                    for target in scenario["targets"])
    maximum_horizontal_speed = maximum_horizontal_acceleration = 0.0
    maximum_vertical_speed = maximum_vertical_acceleration = 0.0
    for entity_id, waypoints, _, _ in entities:
        if len(waypoints) < 2:
            raise ValueError(f"{entity_id} requires at least two waypoints")
        for first, second in zip(waypoints, waypoints[1:]):
            duration = float(second["t"]) - float(first["t"])
            if duration <= 0.0:
                raise ValueError(f"{entity_id} waypoint times must increase")
            delta = {axis: float(second.get(axis, 0.0)) -
                     float(first.get(axis, 0.0)) for axis in ("x", "y", "z")}
            if profile == "minimum_jerk":
                horizontal_distance = math.hypot(delta["x"], delta["y"])
                vertical_distance = abs(delta["z"])
                maximum_horizontal_speed = max(
                    maximum_horizontal_speed,
                    1.875 * horizontal_distance / duration)
                maximum_horizontal_acceleration = max(
                    maximum_horizontal_acceleration,
                    5.7735026919 * horizontal_distance / duration ** 2)
                maximum_vertical_speed = max(
                    maximum_vertical_speed,
                    1.875 * vertical_distance / duration)
                maximum_vertical_acceleration = max(
                    maximum_vertical_acceleration,
                    5.7735026919 * vertical_distance / duration ** 2)

    duration = float(scenario["duration_s"])
    primitives = static_primitives(scenario["world"])
    minimum_separation = math.inf
    minimum_clearance = math.inf
    critical_separation = critical_clearance = None
    elevations = {
        entity_id: {"min": math.inf, "max": -math.inf}
        for entity_id, _, _, _ in entities if entity_id != "uav1"}
    sample_count = int(math.ceil(duration / sample_period_s))
    for index in range(sample_count + 1):
        elapsed = min(duration, index * sample_period_s)
        active = []
        for entity_id, waypoints, spawn_time, radius in entities:
            if elapsed + 1.0e-9 < spawn_time:
                continue
            position, velocity, acceleration = _trajectory_sample(
                waypoints, elapsed, profile, tangent_scale)
            if profile == "continuous_cubic":
                maximum_horizontal_speed = max(
                    maximum_horizontal_speed,
                    math.hypot(velocity[0], velocity[1]))
                maximum_horizontal_acceleration = max(
                    maximum_horizontal_acceleration,
                    math.hypot(acceleration[0], acceleration[1]))
                maximum_vertical_speed = max(
                    maximum_vertical_speed, abs(velocity[2]))
                maximum_vertical_acceleration = max(
                    maximum_vertical_acceleration, abs(acceleration[2]))
            active.append((entity_id, position))
            for primitive in primitives:
                clearance = _point_clearance(position, primitive) - radius
                if clearance < minimum_clearance:
                    minimum_clearance = clearance
                    critical_clearance = (entity_id, primitive["id"], elapsed)
        for first_index, (first_id, first_position) in enumerate(active):
            for second_id, second_position in active[first_index + 1:]:
                separation = math.sqrt(sum(
                    (first_position[axis] - second_position[axis]) ** 2
                    for axis in range(3)))
                if separation < minimum_separation:
                    minimum_separation = separation
                    critical_separation = (first_id, second_id, elapsed)
        if float(scenario["score_start_s"]) <= elapsed <= \
                float(scenario["score_end_s"]):
            observer_position = active[0][1]
            for entity_id, position in active[1:]:
                horizontal = math.hypot(
                    position[0] - observer_position[0],
                    position[1] - observer_position[1])
                for sensor_mount_z in (0.1414, 0.168):
                    elevation = math.degrees(math.atan2(
                        position[2] - observer_position[2] - sensor_mount_z,
                        horizontal))
                    elevations[entity_id]["min"] = min(
                        elevations[entity_id]["min"], elevation)
                    elevations[entity_id]["max"] = max(
                        elevations[entity_id]["max"], elevation)
    measurements = {
        "maximum_horizontal_speed_mps": maximum_horizontal_speed,
        "maximum_horizontal_acceleration_mps2": maximum_horizontal_acceleration,
        "maximum_vertical_speed_mps": maximum_vertical_speed,
        "maximum_vertical_acceleration_mps2": maximum_vertical_acceleration,
    }
    for name, value in measurements.items():
        if value > PHYSICAL_LIMITS[name] + 1.0e-9:
            raise ValueError(f"{name} {value:.3f} exceeds limit")
    if minimum_clearance < -1.0e-6:
        entity_id, primitive_id, elapsed = critical_clearance
        raise ValueError(
            f"{entity_id} violates {primitive_id} clearance by "
            f"{-minimum_clearance:.3f} m at t={elapsed:.2f} s")
    if minimum_separation < \
            PHYSICAL_LIMITS["minimum_vehicle_separation_m"] - 1.0e-6:
        first_id, second_id, elapsed = critical_separation
        raise ValueError(
            f"{first_id}/{second_id} separation {minimum_separation:.3f} m "
            f"at t={elapsed:.2f} s is below limit")
    return {
        "status": "PASS",
        "trajectory_profile": profile,
        "sample_period_s": sample_period_s,
        "minimum_vehicle_separation_m": round(minimum_separation, 6),
        "minimum_static_clearance_m": round(minimum_clearance, 6),
        "planned_scoring_elevation_deg": {
            entity_id: {name: round(value, 6)
                        for name, value in bounds.items()}
            for entity_id, bounds in elevations.items()},
        **{name: round(value, 6) for name, value in measurements.items()},
        "limits": PHYSICAL_LIMITS,
    }


def truth_configuration(scenario, ray_mode, input_topic):
    sources = [{"id": "uav1", "topic":
                "/mid360_multi_uav_sim/ground_truth/uav1/odom"}]
    targets = []
    for target in scenario["targets"]:
        topic = "/mid360_multi_uav_sim/ground_truth/{}/odom".format(
            target["id"])
        sources.append({"id": target["id"], "topic": topic})
        targets.append({"id": target["id"], "truth_source": target["id"],
                        "primitives": target_primitives()})
    return {
        "input_topic": input_topic,
        "output_topic": "/evaluation/visibility_ground_truth",
        "world_frame": "world",
        "geometry_contract_id": scenario["scenario_id"] + "-primitive-v1",
        "ray_time_geometry_mode": ray_mode,
        "per_ray_require_static_targets": False,
        "maximum_truth_gap_sec": .25,
        "pending_bundle_limit": 100,
        "truth_buffer_size": 10000,
        "truth_sources": sources,
        "targets": targets,
        "static_primitives": static_primitives(scenario["world"]),
    }


EVENT_TOPIC = "/mid360_multi_uav_sim/scenario_events"


def source_timing_from_messages(messages, input_topic):
    first_input = first_spawn = benchmark_start = scoring_start = scoring_end = None
    first_input_record = None
    for topic, message, record_stamp in messages:
        if topic == input_topic:
            stamp = message.header.stamp.to_sec()
            first_input = stamp if first_input is None else min(first_input, stamp)
            first_input_record = record_stamp.to_sec() if \
                first_input_record is None else min(
                    first_input_record, record_stamp.to_sec())
            continue
        event = json.loads(message.data)
        stamp = event.get("sim_time")
        if event.get("event") == "benchmark_start":
            benchmark_start = stamp
            benchmark_record_start = record_stamp.to_sec()
        elif event.get("event") == "target_motion_started":
            first_spawn = stamp if first_spawn is None else min(first_spawn, stamp)
        elif event.get("event") == "scoring_start":
            scoring_start = stamp
        elif event.get("event") == "scoring_end":
            scoring_end = stamp
    if first_input is None or benchmark_start is None or \
            scoring_start is None or scoring_end is None:
        raise RuntimeError("source bag lacks input or scoring timing evidence")
    return {"first_checked_ray_stamp": first_input,
            "first_checked_ray_record_stamp": first_input_record,
            "benchmark_start_stamp": benchmark_start,
            "benchmark_start_record_stamp": benchmark_record_start,
            "first_target_spawn_stamp": first_spawn,
            "scoring_start_stamp": scoring_start,
            "scoring_end_stamp": scoring_end,
            "target_free_input_duration_s": max(
                0.0, (first_spawn if first_spawn is not None else
                      scoring_start) - first_input)}


def source_timing(path, input_topic):
    with rosbag.Bag(str(path)) as bag:
        events = list(bag.read_messages(topics=[EVENT_TOPIC]))
        benchmark_record_start = next(
            stamp.to_sec() for topic, message, stamp in events
            if json.loads(message.data).get("event") == "benchmark_start")
        first_ray = next(
            item for item in bag.read_messages(topics=[input_topic])
            if item[2].to_sec() >= benchmark_record_start - 1.0e-6)
        return source_timing_from_messages(events + [first_ray], input_topic)


def trim_to_benchmark(path, input_topic, algorithm_input_topics):
    timing = source_timing(path, input_topic)
    start = timing["benchmark_start_record_stamp"] - 1.0e-6
    first_input_records = {}
    with rosbag.Bag(str(path)) as source:
        for topic, message, record_stamp in source.read_messages(
                topics=algorithm_input_topics):
            first_input_records.setdefault(topic, record_stamp.to_sec())
            if len(first_input_records) == len(algorithm_input_topics):
                break
    if len(first_input_records) != len(algorithm_input_topics):
        raise RuntimeError("source bag lacks algorithm input evidence")
    timing["first_algorithm_input_record_stamp"] = min(
        first_input_records.values())
    if timing["first_algorithm_input_record_stamp"] >= start:
        timing["source_finalize_mode"] = "validated_rename"
        return timing
    trimmed = path.with_name("source.trimmed.bag")
    if trimmed.exists():
        trimmed.unlink()
    with rosbag.Bag(str(path)) as source, rosbag.Bag(
            str(trimmed), "w", compression=rosbag.Compression.LZ4) as output:
        static_transforms = [
            (message, stamp) for _, message, stamp in source.read_messages(
                topics=["/tf_static"])
            if stamp.to_sec() < start
        ]
        if static_transforms:
            replay_stamp = static_transforms[0][1].__class__.from_sec(start)
            for message, _ in static_transforms:
                output.write("/tf_static", message, replay_stamp)

        def copy_messages():
            for topic, message, stamp in source.read_messages():
                if stamp.to_sec() < start:
                    continue
                output.write(topic, message, stamp)
                if topic in (input_topic, EVENT_TOPIC):
                    yield topic, message, stamp

        timing = source_timing_from_messages(copy_messages(), input_topic)
    timing["first_algorithm_input_record_stamp"] = start
    timing["source_finalize_mode"] = "stream_trim"
    path.unlink()
    trimmed.replace(path)
    return timing


def run_record(arguments):
    workspace = Path(__file__).resolve().parents[3]
    output = Path(arguments.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    bag_path = output / "source.bag"
    pending_bag = output / "source.pending.bag"
    trimmed_bag = output / "source.trimmed.bag"
    manifest_path = output / "source_manifest.json"
    if bag_path.exists() and manifest_path.exists() and not arguments.force:
        manifest = json.loads(manifest_path.read_text())
        if sha256(bag_path) != manifest.get("source_bag_sha256"):
            raise RuntimeError("existing source bag hash mismatch")
        return
    if (bag_path.exists() or manifest_path.exists()) and not arguments.force:
        raise RuntimeError(
            "incomplete source exists; inspect it or rerun with --force")
    for stale in (bag_path, pending_bag, trimmed_bag,
                  Path(str(pending_bag) + ".active"), manifest_path):
        if stale.exists():
            stale.unlink()
    scenario_source = Path(arguments.scenario).resolve()
    scenario = yaml.safe_load(scenario_source.read_text())
    if not isinstance(scenario, dict) or scenario.get("schema_version") != 1:
        raise ValueError("scenario must use benchmark schema_version 1")
    scenario["seed"] = arguments.seed
    sensor = arguments.sensor
    if sensor not in ("mid360", "ouster", "paired"):
        raise ValueError("sensor must be mid360, ouster, or paired")
    physical_validation = validate_physical_scenario(scenario)
    requested_ray_mode = scenario.get("ray_time_geometry_mode", "rolling_scene")
    if requested_ray_mode != "rolling_scene":
        raise ValueError(
            "canonical MRS benchmark requires ray_time_geometry_mode=rolling_scene")
    scenario["ray_time_geometry_mode"] = "rolling_scene"
    scenario_path = output / "scenario.yaml"
    scenario_path.write_text(yaml.safe_dump(scenario, sort_keys=False))
    ray_mode = "rolling_scene"
    truth_path = output / "truth_evaluator.yaml"
    # The lightweight Mid-360 checked stream drives only the online truth
    # geometry evaluator. Ouster algorithms still receive the full raw
    # 131,072-sample (1024 x 128) scan during replay.
    truth_input_topic = "/uav1/mid360/rays_checked"
    use_visibility_evaluator = bool(scenario["targets"]) and sensor != "ouster"
    if not use_visibility_evaluator and truth_path.exists():
        truth_path.unlink()
    if use_visibility_evaluator:
        truth_path.write_text(yaml.safe_dump(
            truth_configuration(scenario, ray_mode, truth_input_topic),
            sort_keys=False))
    world = workspace / "src/mid360_multi_uav_sim/worlds" / WORLD_FILES[
        scenario["world"]]
    ros_port, gazebo_port = free_port(), free_port()
    environment = dict(
        os.environ,
        ROS_MASTER_URI=f"http://127.0.0.1:{ros_port}",
        GAZEBO_MASTER_URI=f"http://127.0.0.1:{gazebo_port}",
        GAZEBO_RANDOM_SEED=str(arguments.seed))
    processes = []
    logs = []
    started = utc_now()
    try:
        processes.append(subprocess.Popen(
            ["roscore", "-p", str(ros_port)], env=environment,
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
            start_new_session=True))
        time.sleep(1.0)
        if use_visibility_evaluator:
            subprocess.run(["rosparam", "load", str(truth_path),
                            "/visibility_ground_truth_evaluator"],
                           env=environment, check=True)
            truth_log = open(output / "truth_evaluator.log", "w", encoding="utf-8")
            logs.append(truth_log)
            evaluator = subprocess.Popen([
                str(workspace / "devel/lib/tclv_evaluation/visibility_ground_truth_evaluator"),
                "__name:=visibility_ground_truth_evaluator"], env=environment,
                stdout=truth_log, stderr=subprocess.STDOUT,
                start_new_session=True)
            processes.append(evaluator)
            wait_for_topic("/evaluation/visibility_ground_truth",
                           environment, evaluator, 20.0)
        topics = [
            "/clock", "/tf", "/tf_static",
            "/uav1/native_rangefinder",
            "/mid360_multi_uav_sim/scenario_events",
            "/mid360_multi_uav_sim/ground_truth/uav1/odom",
        ] + ["/mid360_multi_uav_sim/ground_truth/{}/odom".format(item["id"])
             for item in scenario["targets"]]
        if sensor in ("mid360", "paired"):
            topics += ["/uav1/mid360/points_world",
                       "/uav1/mid360/rays_checked"]
        if sensor in ("ouster", "paired"):
            topics.append("/uav1/os_cloud_nodelet/points")
        if use_visibility_evaluator:
            topics.append("/evaluation/visibility_ground_truth")
        simulation_log = open(output / "simulation.log", "w", encoding="utf-8")
        logs.append(simulation_log)
        launch_arguments = [
            "roslaunch", "mid360_multi_uav_sim", "benchmark.launch",
            f"world_file:={world}", f"scenario_file:={scenario_path}",
            f"sensor_noise_stddev:={NOISE_STDDEV_M[arguments.noise]}",
            f"ray_time_geometry_mode:={ray_mode}",
            f"sensor_model:={sensor}",
            f"gui:={str(bool(arguments.gui)).lower()}",
            "wait_for_start:=true", "output:=log"]
        launch_arguments.append(
            f"vehicle_count:={1 + len(scenario['targets'])}")
        mrs_config = workspace / "src/mid360_multi_uav_sim/config/mrs" / \
            scenario.get("mrs_custom_config", "paper_config.yaml")
        launch_arguments.append(f"mrs_custom_config:={mrs_config}")
        launch_arguments.append("mrs_world_config:=" + str(
            workspace / "src/mid360_multi_uav_sim/config/mrs" /
            scenario.get("mrs_world_config", "world_config.yaml")))
        launch = subprocess.Popen(launch_arguments,
            env=environment, stdout=simulation_log, stderr=subprocess.STDOUT,
            start_new_session=True)
        processes.append(launch)
        wait_for_topic("/uav1/os_cloud_nodelet/points" if sensor in
                       ("ouster", "paired") else
                       "/uav1/mid360/rays_checked", environment, launch,
                       timeout=120.0)
        subprocess.run(
            ["rostopic", "echo", "-n", "1",
             "/mid360_multi_uav_sim/scenario_events"],
            env=environment, check=True, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, timeout=360.0)
        source_recorder_log = open(output/'recorder.log','w',encoding='utf-8')
        logs.append(source_recorder_log)
        recorder = subprocess.Popen(
            ["rosbag", "record", "--lz4", "--buffsize=1024", "--min-space=256M", "-O",
             str(pending_bag), *topics],
            env=environment, stdout=source_recorder_log,
            stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(recorder)
        time.sleep(1.0)
        start_result = subprocess.run(
            ["rosservice", "call", "/benchmark_scenario/start", "{}"],
            env=environment, check=True, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        if "success: True" not in start_result.stdout:
            raise RuntimeError("benchmark start service rejected the run")
        timeout = (scenario["duration_s"] +
                   scenario.get("start_delay_s", 2.0)) * 20 + 360
        if launch.wait(timeout=timeout) != 0:
            raise RuntimeError("simulation launch failed")
        terminate(recorder)
    finally:
        while processes:
            terminate(processes.pop())
        for stream in logs:
            stream.close()
    # rosbag may rename the compressed .active file just after its parent
    # process exits. Give that final atomic rename a short grace period.
    deadline = time.time() + 60.0
    while not pending_bag.exists() and time.time() < deadline:
        time.sleep(0.1)
    if not pending_bag.exists() or pending_bag.stat().st_size == 0:
        raise RuntimeError("source recorder produced no bag")
    validate_recorder_log(output/'recorder.log')
    input_topic = "/uav1/mid360/rays_checked" if sensor == "mid360" else \
        "/uav1/os_cloud_nodelet/points"
    algorithm_input_topics = (
        ("/uav1/mid360/points_world", "/uav1/mid360/rays_checked")
        if sensor == "mid360" else
        ("/uav1/os_cloud_nodelet/points",)
        if sensor == "ouster" else
        ("/uav1/mid360/points_world", "/uav1/mid360/rays_checked",
         "/uav1/os_cloud_nodelet/points"))
    timing = trim_to_benchmark(
        pending_bag, input_topic, algorithm_input_topics)
    with rosbag.Bag(str(pending_bag)) as recorded:
        topic_info = recorded.get_type_and_topic_info().topics
        message_counts = {topic: info.message_count
                          for topic, info in topic_info.items()}
    if message_counts.get("/uav1/native_rangefinder", 0) == 0:
        raise RuntimeError(
            "source lacks the simulated rangefinder required by NativeInit")
    if sensor in ("ouster", "paired") and \
            message_counts.get("/uav1/os_cloud_nodelet/points", 0) == 0:
        raise RuntimeError("source lacks Ouster OS1-128 scans")
    mrs_config = workspace / "src/mid360_multi_uav_sim/config/mrs" / \
        scenario.get("mrs_custom_config", "paper_config.yaml")
    implementation_paths = [
        Path(__file__).resolve(),
        workspace / "src/mid360_multi_uav_sim/scripts/benchmark_scenario.py",
        workspace / "src/mid360_multi_uav_sim/scripts/rangefinder_follower.py",
        workspace / "src/mid360_multi_uav_sim/models/observer_rangefinder/model.sdf",
        workspace / "src/mid360_multi_uav_sim/launch/benchmark.launch",
        workspace / "src/mid360_multi_uav_sim/launch/mrs_vehicle.launch",
        mrs_config,
        workspace / "src/mid360_multi_uav_sim/config/mrs" /
        scenario.get("mrs_world_config", "world_config.yaml"),
        workspace / "src/mid360_simulation_plugin_fork/livox_laser_simulation/src/livox_points_plugin.cpp",
        workspace / "src/mid360_ray_preprocessor/src/mid360_ray_preprocessor_node.cpp",
        workspace / "src/tclv_evaluation/src/visibility_ground_truth_evaluator.cpp",
    ]
    if sensor in ("ouster", "paired"):
        implementation_paths.append(
            workspace / "src/mid360_ray_preprocessor/src/"
            "ouster_snapshot_adapter_node.cpp")
        implementation_paths += [
            workspace / "src/mid360_simulation_plugin_fork/livox_laser_simulation/src/mrs_3dlidar_plugin.cpp",
            workspace / "src/mid360_simulation_plugin_fork/livox_laser_simulation/include/livox_laser_simulation/scan_stamp_gate.h",
        ]
    commit = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=workspace, text=True).strip()
    manifest = {
        "schema_version": 1,
        "scenario_id": scenario["scenario_id"],
        "seed": arguments.seed,
        "seed_applied": True,
        "noise": arguments.noise,
        "recorder_commit": commit,
        "recorder_tree_sha256": combined_sha256(implementation_paths),
        "git_dirty": bool(subprocess.check_output(
            ["git", "status", "--porcelain"], cwd=workspace, text=True).strip()),
        "source_bag_sha256": sha256(pending_bag),
        "scenario_sha256": sha256(scenario_path),
        "sensor_model": "MRS X500 + " + sensor,
        "sensor_suite": sensor,
        "sensor_model_sha256": combined_sha256([
            Path("/opt/ros/noetic/share/mrs_uav_gazebo_simulation/models/"
                 "mrs_robots_description/sdf/x500.sdf.jinja"),
            Path("/opt/ros/noetic/share/mrs_uav_gazebo_simulation/models/"
                 "mrs_robots_description/sdf/component_snippets.sdf.jinja"),
            workspace / "src/mid360_multi_uav_sim/models/"
            "observer_rangefinder/model.sdf",
        ]),
        "world_sha256": sha256(world),
        "truth_config_sha256": sha256(truth_path) if truth_path.exists() else None,
        "ray_time_geometry_mode": "snapshot" if sensor in
        ("ouster", "paired") else ray_mode,
        "sensor_time_organization": {
            "mid360": "rolling_scene" if sensor in ("mid360", "paired") else None,
            "ouster": "single Gazebo snapshot; t field is zero" if sensor in
            ("ouster", "paired") else None,
        },
        "requested_ray_time_geometry_mode": requested_ray_mode,
        "flight_stack": "MRS_X500_PX4_MAVROS_MPC",
        "mrs_custom_config_sha256": sha256(mrs_config),
        "gazebo_gui": bool(arguments.gui),
        "physical_validation": physical_validation,
        "recorded_message_counts": message_counts,
        "started_utc": started,
        "finished_utc": utc_now(),
    }
    manifest.update(timing)
    if sensor in ("ouster", "paired"):
        publisher = workspace / "devel/lib/libMrsGazeboCommonResources_3DLidarGpuPlugin.so"
        manifest["ouster_publisher_binary"] = str(publisher.resolve())
        manifest["ouster_publisher_binary_sha256"] = sha256(publisher)
    pending_bag.replace(bag_path)
    write_json(manifest_path, manifest)


def validate_recorder_log(path):
    if 'buffer exceeded' in path.read_text():
        raise RuntimeError('output recorder dropped messages; invalid measurement')


def run_replay(arguments):
    workspace = Path(__file__).resolve().parents[3]
    output = Path(arguments.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    existing_manifest = output / "run_manifest.json"
    if existing_manifest.exists() and \
            json.loads(existing_manifest.read_text()).get("status") != "COMPLETE":
        for stale in output.glob("output*.bag*"):
            stale.unlink()
    default_configs = {
        'VoFOD-Mid360': workspace / "src/vofod_mid360/config/four_scene_suite/OPEN_mid360.yaml",
        'VoFOD-OS1': workspace / "src/vofod_mid360/config/four_scene_suite/OPEN_os1.yaml",
        'AeroCOVER-Mid360': workspace / "src/aerocover_mid360/config/aerocover_mvp.yaml",
        'AeroCOVER-OS1': workspace / "src/aerocover_mid360/config/aerocover_os1_128.yaml",
        'AeroCOVER-A1': workspace / "src/aerocover_mid360/config/ablation_a1_current_frame.yaml",
        'AeroCOVER-A2': workspace / "src/aerocover_mid360/config/ablation_a2_whole_history_extent.yaml",
        'AeroCOVER-A3': workspace / "src/aerocover_mid360/config/ablation_a3_without_shell.yaml",
        'AeroCOVER-A4': workspace / "src/aerocover_mid360/config/ablation_a4_without_full_chord.yaml",
        'AeroCOVER-A5': workspace / "src/aerocover_mid360/config/ablation_a5_one_shot_birth.yaml",
    }
    default_config = default_configs[arguments.algorithm]
    config = Path(arguments.config or default_config).resolve()
    method_configs = {
        'VoFOD-Mid360': workspace / "src/vofod_mid360/config/vofod_original.yaml",
        'VoFOD-OS1': workspace / "src/vofod_mid360/config/vofod_original.yaml",
    }
    method_config = method_configs.get(arguments.algorithm)
    source = Path(arguments.source).resolve()
    scenario = Path(arguments.scenario).resolve()
    scenario_data = yaml.safe_load(scenario.read_text())
    if not isinstance(scenario_data, dict) or scenario_data.get("schema_version") != 1:
        raise ValueError("scenario must use benchmark schema_version 1")
    seed = getattr(arguments, "seed", None)
    seed = int(scenario_data["seed"] if seed is None else seed)
    tracker_override = (workspace/'src/lidar_tracker_mid360/config/tracking_open_v2.yaml'
                        if scenario_data['scenario_id'] in ('OPEN','MT') and
                        arguments.algorithm in ('VoFOD-Mid360','VoFOD-OS1') else None)
    if getattr(arguments,'tracker_override',None):
        if arguments.algorithm not in ('VoFOD-Mid360','VoFOD-OS1'):
            raise ValueError('tracker override is only supported by canonical VoFOD methods')
        tracker_override=Path(arguments.tracker_override).resolve()
    with rosbag.Bag(str(source)) as source_bag:
        replay_topics = sorted(
            topic for topic in source_bag.get_type_and_topic_info().topics
            if topic != "/clock")
    source_manifest_path = source.with_name("source_manifest.json")
    if getattr(arguments, "require_source_manifest", False) and \
            not source_manifest_path.exists():
        raise FileNotFoundError(f"missing {source_manifest_path}")
    source_manifest = json.loads(source_manifest_path.read_text()) \
        if source_manifest_path.exists() else None
    if source_manifest:
        if source_manifest.get("schema_version") != 1 or \
                not source_manifest.get("recorder_commit") or \
                source_manifest.get("seed_applied") is not True:
            raise ValueError(
                "source manifest lacks schema, recorder commit, or seed authority")
        expected = (scenario_data["scenario_id"], seed)
        actual = (source_manifest.get("scenario_id"),
                  source_manifest.get("seed"))
        if actual != expected:
            raise ValueError(f"source manifest mismatch: expected {expected}, got {actual}")
        suite = source_manifest.get("sensor_suite", "mid360")
        needs_ouster = arguments.algorithm in ("AeroCOVER-OS1", "VoFOD-OS1")
        if needs_ouster and suite not in ("ouster", "paired"):
            raise ValueError("Ouster method requires an ouster or paired source")
        if not needs_ouster and suite not in ("mid360", "paired"):
            raise ValueError("Mid-360 method requires a mid360 or paired source")
    evaluator = Path(__file__).with_name("evaluate_bag.py")
    if arguments.algorithm in ("VoFOD-Mid360", "VoFOD-OS1"):
        algorithm_roots = (
            workspace / "src/vofod_mid360",
            workspace / "src/lidar_tracker_mid360",
            workspace / "src/tclv_evaluation/launch")
        binary_paths = (
            workspace / "devel/.private/vofod_mid360/lib/libVoFODMid360Core.so",
            workspace / "devel/.private/vofod_mid360/lib/libVoFODMid360.so",
            workspace / "devel/.private/lidar_tracker_mid360/lib/libLidarTrackerMid360Core.so",
            workspace / "devel/.private/lidar_tracker_mid360/lib/libLidarTrackerMid360.so")
    else:
        algorithm_roots = (workspace / "src/aerocover_mid360",
                           workspace / "src/aerocover_st_background")
        if arguments.algorithm == "AeroCOVER-OS1":
            algorithm_roots += (workspace / "src/mid360_ray_preprocessor",)
        binary_paths = (
            workspace / "devel/.private/aerocover_st_background/lib/libAeroCoverSTBackground.so",
            workspace / "devel/.private/aerocover_mid360/lib/libAeroCoverCore.so",
            workspace / "devel/.private/aerocover_mid360/lib/aerocover_mid360/aerocover_node")
    source_suffixes = (".cpp", ".h", ".launch", ".msg", ".yaml", ".xml")
    algorithm_sources = [
        path for root in algorithm_roots for path in root.rglob("*")
        if path.is_file() and
        (path.suffix in source_suffixes or path.name == "CMakeLists.txt")]
    catkin_profile = workspace / ".catkin_tools/profiles/default/config.yaml"
    profile = yaml.safe_load(catkin_profile.read_text()) \
        if catkin_profile.exists() else {}
    manifest = {
        "schema_version": 5,
        "status": "PLANNED" if arguments.dry_run else "RUNNING",
        "algorithm": arguments.algorithm,
        "seed": seed,
        "sensor_noise": source_manifest.get("noise")
        if source_manifest else None,
        "source": str(source),
        "source_sha256": sha256(source),
        "source_manifest": str(source_manifest_path)
        if source_manifest else None,
        "source_manifest_sha256": sha256(source_manifest_path)
        if source_manifest else None,
        "scenario": str(scenario),
        "scenario_sha256": sha256(scenario),
        "config": str(config),
        "config_sha256": sha256(config),
        "tracker_override_config": str(tracker_override) if tracker_override else None,
        "tracker_override_sha256": sha256(tracker_override) if tracker_override else None,
        "method_config": str(method_config) if method_config else None,
        "method_config_sha256": sha256(method_config)
        if method_config else None,
        "background_mode": "native_rangefinder"
        if arguments.algorithm in method_configs else "not_applicable",
        "map_execution_mode": "upstream_native"
        if arguments.algorithm in method_configs else "not_applicable",
        "checked_ray_mode": "ouster_sim_snapshot"
        if arguments.algorithm.endswith("OS1") else
        "mid360_sim_exact_rolling_scene",
        "upstream_vofod_commit":
        "7da9f33a878a586588f6a626b75cfeacac7824f7"
        if arguments.algorithm in method_configs else None,
        "upstream_tracker_commit":
        "a92b4db61060b47f1af6dcce122188ec021f2dcd"
        if arguments.algorithm in method_configs else None,
        "hota_protocol": "Disabled by current default evaluator; "
        "see metrics.hota_computed. Optional compatibility tests remain.",
        "trackeval_commit":
        "12c8791b303e0a0b50f753af204249e622d0281a",
        "metric_code_sha256": sha256(evaluator),
        "runner_sha256": sha256(Path(__file__).resolve()),
        "algorithm_tree_sha256": combined_sha256(algorithm_sources),
        "algorithm_binary_sha256": combined_sha256(
            path for path in binary_paths if path.exists()),
        "git_commit": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], text=True).strip(),
        "git_dirty": bool(subprocess.check_output(
            ["git", "status", "--porcelain"], text=True).strip()),
        "build": {
            "cmake_args": profile.get("cmake_args", []),
            "cxxflags": os.environ.get("CXXFLAGS", ""),
        },
        "ros": {
            "distro": os.environ.get("ROS_DISTRO"),
            "version": os.environ.get("ROS_VERSION"),
        },
        "machine": {
            "hostname": platform.node(),
            "platform": platform.platform(),
            "processor": platform.processor(),
            "cpu_count": os.cpu_count(),
            "gpu": "not used",
            "ram_bytes": os.sysconf("SC_PAGE_SIZE") *
            os.sysconf("SC_PHYS_PAGES"),
        },
        "command": algorithm_command(arguments.algorithm, config, tracker_override),
        "replay_rate": arguments.replay_rate,
        "output_recording": {"compression": "lz4", "buffer_mib": 1024},
        "created_unix_s": time.time(),
    }
    manifest_path = output / "run_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    if arguments.dry_run:
        print(json.dumps(manifest, indent=2))
        return

    port = free_port()
    environment = dict(os.environ, ROS_MASTER_URI=f"http://127.0.0.1:{port}")
    processes = []
    launch_log = None
    recorder_log = None
    output_bag = output / "output.bag"
    resource = output / "resource.txt"
    try:
        processes.append(subprocess.Popen(
            ["roscore", "-p", str(port)], env=environment,
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
            start_new_session=True))
        time.sleep(1.0)
        subprocess.run(
            ["rosparam", "set", "/use_sim_time", "true"],
            env=environment, check=True)
        launch_log = open(output / "algorithm.log", "w", encoding="utf-8")
        processes.append(subprocess.Popen(
            ["/usr/bin/time", "-v", "-o", str(resource),
             *manifest["command"]], env=environment, stdout=launch_log,
            stderr=subprocess.STDOUT, start_new_session=True))
        time.sleep(2.0)
        if processes[-1].poll() is not None:
            raise RuntimeError("algorithm exited before replay")
        recorder_log = open(output / "recorder.log", "w", encoding="utf-8")
        recorder = subprocess.Popen(
            ["rosbag", "record", "--lz4", "--buffsize=1024", "--min-space=256M", "-O", str(output_bag),
             *recorded_topics(arguments.algorithm)], env=environment,
            stdout=recorder_log, stderr=subprocess.STDOUT,
            start_new_session=True)
        processes.append(recorder)
        time.sleep(1.0)
        if recorder.poll() is not None:
            raise RuntimeError(
                f"output recorder exited before replay with status {recorder.returncode}")
        subprocess.run(
            ["rosbag", "play", "--clock", "--quiet", "--rate",
             str(arguments.replay_rate), "--delay",
             str(REPLAY_ADVERTISE_DELAY_S), str(source), "--topics",
             *replay_topics],
            env=environment, check=True)
        time.sleep(REPLAY_DRAIN_DELAY_S)
        if recorder.poll() is not None:
            raise RuntimeError(
                f"output recorder exited during replay with status {recorder.returncode}")
        terminate(processes.pop())  # recorder
        recorder_log.close()
        recorder_log = None
        validate_recorder_log(output/'recorder.log')
        terminate(processes.pop())  # algorithm; flushes /usr/bin/time output
        launch_log.close()
        launch_log = None
        subprocess.run([
            sys.executable, str(evaluator), "--source", str(source),
            "--run", str(output_bag), "--algorithm", arguments.algorithm,
            "--scenario", str(scenario), "--output", str(output),
            "--resource", str(resource), "--seed", str(seed)],
            env=environment, check=True)
        manifest["status"] = "COMPLETE"
        manifest["output_sha256"] = sha256(output_bag)
        manifest["metrics_sha256"] = sha256(output / "metrics.json")
        if arguments.cleanup_output_bag:
            output_bag.unlink()
    except BaseException as error:
        manifest["status"] = "FAILED"
        manifest["error"] = str(error)
        raise
    finally:
        while processes:
            terminate(processes.pop())
        if launch_log is not None:
            launch_log.close()
        if recorder_log is not None:
            recorder_log.close()
        manifest["finished_unix_s"] = time.time()
        manifest["wall_duration_s"] = (
            manifest["finished_unix_s"] - manifest["created_unix_s"])
        manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")


def aggregate_results(root):
    root = Path(root)
    rows = []
    run_directories = {
        path.parent for pattern in ("metrics.json", "run_manifest.json")
        for path in root.glob(f"runs/**/{pattern}")}
    for directory in sorted(run_directories):
        metrics_path = directory / "metrics.json"
        manifest_path = directory / "run_manifest.json"
        metrics = (json.loads(metrics_path.read_text())
                   if metrics_path.exists() else {})
        manifest = (json.loads(manifest_path.read_text())
                    if manifest_path.exists() else {"status": "COMPLETE"})
        scenario = (yaml.safe_load(Path(manifest["scenario"]).read_text())
                    if manifest.get("scenario") else {})
        track = metrics.get("track_set", {})
        runtime = metrics.get("runtime", {})
        epistemic = metrics.get("epistemic", {})
        rows.append({
            "algorithm": manifest.get("algorithm"),
            "scene": scenario.get("scenario_id", "UNKNOWN").split("_", 1)[0],
            "seed": manifest.get("seed"),
            "status": manifest.get("status", "UNKNOWN"),
            "HOTA": track.get("HOTA"),
            "DetA": track.get("DetA"),
            "AssA": track.get("AssA"),
            "IDF1": track.get("IDF1"),
            "precision": track.get("precision"),
            "recall": track.get("recall"),
            "tp": track.get("tp"),
            "fp": track.get("fp"),
            "fn": track.get("fn"),
            "position_RMSE_m": track.get("position_RMSE_m"),
            "velocity_RMSE_mps": track.get("velocity_RMSE_mps"),
            "TTFT_mean_s": track.get("TTFT_mean_s"),
            "births": epistemic.get("total_birth_count"),
            "matched_births": epistemic.get("matched_birth_count"),
            "false_births": epistemic.get("unmatched_birth_count"),
            "runtime_mean_ms": runtime.get("mean_ms"),
            "runtime_median_ms": runtime.get("median_ms"),
            "runtime_p95_ms": runtime.get("p95_ms"),
            "runtime_p99_ms": runtime.get("p99_ms"),
            "runtime_max_ms": runtime.get("max_ms"),
            "peak_rss_kib": runtime.get("peak_rss_kib"),
        })
    metrics_dir = root / "aggregated"
    metrics_dir.mkdir(parents=True, exist_ok=True)
    columns = tuple(rows[0]) if rows else (
        "algorithm", "scene", "seed", "status")
    with open(metrics_dir / "aggregate.csv", "w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)
    return rows


def parser():
    root = Path(__file__).resolve().parents[3]
    result = argparse.ArgumentParser()
    subparsers = result.add_subparsers(dest="command", required=True)
    record = subparsers.add_parser("record")
    record.add_argument("--scenario", required=True)
    record.add_argument("--seed", type=int, required=True)
    record.add_argument("--noise", choices=NOISE_LEVELS, default="N0")
    record.add_argument("--sensor", choices=("mid360", "ouster", "paired"),
                        default="mid360")
    record.add_argument("--output", required=True)
    record.add_argument("--force", action="store_true")
    record.add_argument("--gui", action="store_true")
    replay = subparsers.add_parser("replay")
    replay.add_argument("--source", required=True)
    replay.add_argument("--scenario", required=True)
    replay.add_argument("--algorithm", choices=ALGORITHMS, required=True)
    replay.add_argument("--seed", type=int)
    replay.add_argument("--require-source-manifest", action="store_true")
    replay.add_argument("--output", required=True)
    replay.add_argument("--config")
    replay.add_argument("--tracker-override", help="Explicit canonical VoFOD tracker YAML override")
    replay.add_argument("--dry-run", action="store_true")
    replay.add_argument("--replay-rate", type=float, default=1.0)
    replay.add_argument("--cleanup-output-bag", action="store_true")
    aggregate = subparsers.add_parser("aggregate")
    aggregate.add_argument("root")
    return result


def main():
    arguments = parser().parse_args()
    if arguments.command == "record":
        run_record(arguments)
    elif arguments.command == "replay":
        if arguments.replay_rate <= 0.0:
            raise ValueError("replay rate must be positive")
        run_replay(arguments)
    else:
        aggregate_results(arguments.root)


if __name__ == "__main__":
    main()
