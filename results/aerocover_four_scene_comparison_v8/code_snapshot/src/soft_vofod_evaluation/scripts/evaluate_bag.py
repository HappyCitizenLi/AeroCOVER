#!/usr/bin/env python3
"""Offline 3-D tracking evaluation; official TrackEval HOTA is temporarily disabled."""

import argparse
import bisect
import csv
import hashlib
import json
import math
import os
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.optimize import linear_sum_assignment
import rosbag
import yaml
from sensor_msgs import point_cloud2

if not hasattr(np, "float"):
    np.float = float  # TrackEval commit 12c8791 uses the removed NumPy alias.


MAIN_THRESHOLD_M = 1.5
COMPUTE_HOTA = False  # Temporarily disabled at the user's request; historical results remain intact.
THRESHOLDS_M = (0.5, 1.0, 1.5, 2.0)
HOTA_DISTANCE_SCALE_M = 2.0
HOTA_THRESHOLDS = tuple(float(value) for value in np.arange(0.05, 1.0, 0.05))
TRACKEVAL_COMMIT = "12c8791b303e0a0b50f753af204249e622d0281a"
PROVENANCE_NAMES = {
    6: "VOFOD", 8: "AEROCOVER",
}
SEMANTIC_BIRTH_LABELS = {
    8: "AEROCOVER_SHELL",
}


def file_sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def percentile(values, quantile):
    return float(np.percentile(values, quantile)) if values else None


def finite_mean(values):
    values = [value for value in values if value is not None and math.isfinite(value)]
    return float(np.mean(values)) if values else None


def ros_time_or(message, field, fallback):
    value = getattr(message, field, None)
    seconds = value.to_sec() if value is not None else 0.0
    return seconds if seconds > 0.0 else fallback


def nearest(frames, stamp, tolerance=0.15):
    if not frames:
        return None
    times = [item[0] for item in frames]
    index = bisect.bisect_left(times, stamp)
    candidates = []
    if index < len(frames):
        candidates.append(frames[index])
    if index > 0:
        candidates.append(frames[index - 1])
    result = min(candidates, key=lambda item: abs(item[0] - stamp))
    return result[1] if abs(result[0] - stamp) <= tolerance else None


def assignments(truth, predictions, threshold):
    if not truth or not predictions:
        return []
    truth_positions = np.asarray([item["position"] for item in truth])
    prediction_positions = np.asarray([item["position"] for item in predictions])
    costs = np.linalg.norm(
        truth_positions[:, None, :] - prediction_positions[None, :, :], axis=2)
    truth_count, prediction_count = costs.shape
    size = truth_count + prediction_count
    unmatched_cost = 1000.0
    assignment_cost = np.zeros((size, size), dtype=float)
    assignment_cost[:truth_count, :prediction_count] = np.where(
        costs <= threshold, costs, 1.0e9)
    assignment_cost[:truth_count, prediction_count:] = unmatched_cost
    assignment_cost[truth_count:, :prediction_count] = unmatched_cost
    rows, columns = linear_sum_assignment(assignment_cost)
    return [(int(row), int(column), float(costs[row, column]))
            for row, column in zip(rows, columns)
            if row < truth_count and column < prediction_count and
            costs[row, column] <= threshold]


def ignore_occluded_predictions(visible_truth, ignored_truth, predictions, threshold):
    """Protect visible matches; ignore at most one otherwise-unmatched prediction per hidden target."""
    if not ignored_truth:
        return predictions, 0
    protected = {column for _, column, _ in assignments(visible_truth, predictions, threshold)}
    indices = [i for i in range(len(predictions)) if i not in protected]
    ignored = {indices[column] for _, column, _ in
               assignments(ignored_truth, [predictions[i] for i in indices], threshold)}
    return [p for i,p in enumerate(predictions) if i not in ignored], len(ignored)


def static_center_los(origin, target, primitives):
    from run_benchmark import _ray_primitive_entry
    origin=np.asarray(origin); delta=np.asarray(target)-origin
    return not any(hit is not None and 0 < hit < 1 for hit in
                   (_ray_primitive_entry(origin,delta,p) for p in primitives))


def hota_3d_pos(frames):
    """Thin 3-D position adapter for TrackEval's official HOTA core."""
    from trackeval.metrics.hota import HOTA
    truth_ids = sorted({str(item["id"]) for frame in frames
                        for item in frame["truth"]})
    prediction_ids = sorted({str(item["id"]) for frame in frames
                             for item in frame["predictions"]})
    truth_index = {value: index for index, value in enumerate(truth_ids)}
    prediction_index = {
        value: index for index, value in enumerate(prediction_ids)}
    gt_by_frame = []
    predictions_by_frame = []
    similarities = []
    gt_detections = tracker_detections = 0
    for frame in frames:
        truth = frame["truth"]
        predictions = frame["predictions"]
        gt_by_frame.append(np.asarray(
            [truth_index[str(item["id"])] for item in truth], dtype=int))
        predictions_by_frame.append(np.asarray(
            [prediction_index[str(item["id"])] for item in predictions],
            dtype=int))
        gt_detections += len(truth)
        tracker_detections += len(predictions)
        if truth and predictions:
            distances = np.linalg.norm(
                np.asarray([item["position"] for item in truth])[:, None, :] -
                np.asarray([item["position"] for item in predictions])[None, :, :],
                axis=2)
            similarities.append(np.maximum(
                0.0, 1.0 - distances / HOTA_DISTANCE_SCALE_M))
        else:
            similarities.append(np.zeros((len(truth), len(predictions))))
    data = {
        "num_tracker_dets": tracker_detections,
        "num_gt_dets": gt_detections,
        "num_tracker_ids": len(prediction_ids),
        "num_gt_ids": len(truth_ids),
        "gt_ids": gt_by_frame,
        "tracker_ids": predictions_by_frame,
        "similarity_scores": similarities,
    }
    raw = HOTA().eval_sequence(data)
    per_threshold = []
    for index, alpha in enumerate(HOTA_THRESHOLDS):
        per_threshold.append({
            "alpha": alpha,
            "distance_boundary_m": HOTA_DISTANCE_SCALE_M * (1.0 - alpha),
            "HOTA": float(raw["HOTA"][index]),
            "DetA": float(raw["DetA"][index]),
            "AssA": float(raw["AssA"][index]),
            "DetRe": float(raw["DetRe"][index]),
            "DetPr": float(raw["DetPr"][index]),
            "LocA": float(raw["LocA"][index]),
            "TP": int(raw["HOTA_TP"][index]),
            "FP": int(raw["HOTA_FP"][index]),
            "FN": int(raw["HOTA_FN"][index]),
        })
    return {
        "protocol": "HOTA-3D-pos",
        "distance_scale_m": HOTA_DISTANCE_SCALE_M,
        "thresholds": list(HOTA_THRESHOLDS),
        "trackeval_commit": TRACKEVAL_COMMIT,
        "HOTA": finite_mean([item["HOTA"] for item in per_threshold]),
        "DetA": finite_mean([item["DetA"] for item in per_threshold]),
        "AssA": finite_mean([item["AssA"] for item in per_threshold]),
        "per_threshold": per_threshold,
    }


def gospa(truth, predictions, cutoff=MAIN_THRESHOLD_M, order=2.0, alpha=2.0):
    if not truth and not predictions:
        return 0.0
    if truth and predictions:
        costs = np.linalg.norm(
            np.asarray([item["position"] for item in truth])[:, None, :] -
            np.asarray([item["position"] for item in predictions])[None, :, :], axis=2)
        rows, columns = linear_sum_assignment(np.minimum(costs, cutoff) ** order)
        localization = sum(min(float(costs[row, column]), cutoff) ** order
                           for row, column in zip(rows, columns))
        unmatched = abs(len(truth) - len(predictions))
    else:
        localization = 0.0
        unmatched = len(truth) + len(predictions)
    return float((localization + cutoff ** order * unmatched / alpha) ** (1.0 / order))


def set_metrics(frames, threshold):
    tp = fp = fn = 0
    distances = []
    velocity_errors = []
    velocity_magnitude_errors = []
    velocity_direction_errors = []
    pair_counts = defaultdict(int)
    truth_match_counts = defaultdict(int)
    prediction_match_counts = defaultdict(int)
    truth_sequences = defaultdict(list)
    total_truth = total_predictions = 0
    gospa_values = []
    range_errors = defaultdict(list)

    for frame in frames:
        truth, predictions = frame["truth"], frame["predictions"]
        matches = assignments(truth, predictions, threshold)
        matched_truth = {row for row, _, _ in matches}
        matched_predictions = {column for _, column, _ in matches}
        tp += len(matches)
        fn += len(truth) - len(matches)
        fp += len(predictions) - len(matches)
        total_truth += len(truth)
        total_predictions += len(predictions)
        gospa_values.append(gospa(truth, predictions, threshold))
        by_truth = {item["id"]: None for item in truth}
        for row, column, distance in matches:
            gt, prediction = truth[row], predictions[column]
            pair = (gt["id"], prediction["id"])
            pair_counts[pair] += 1
            truth_match_counts[gt["id"]] += 1
            prediction_match_counts[prediction["id"]] += 1
            by_truth[gt["id"]] = prediction["id"]
            distances.append(distance)
            velocity_error = np.asarray(prediction["velocity"]) - np.asarray(gt["velocity"])
            velocity_errors.append(float(np.dot(velocity_error, velocity_error)))
            prediction_speed = float(np.linalg.norm(prediction["velocity"]))
            truth_speed = float(np.linalg.norm(gt["velocity"]))
            velocity_magnitude_errors.append(abs(prediction_speed - truth_speed))
            if prediction_speed > 0.1 and truth_speed > 0.1:
                cosine = float(np.dot(prediction["velocity"], gt["velocity"]) /
                               (prediction_speed * truth_speed))
                velocity_direction_errors.append(math.acos(max(-1.0, min(1.0, cosine))))
            observer = np.asarray(frame["observer"])
            target_range = float(np.linalg.norm(np.asarray(gt["position"]) - observer))
            if target_range < 10.0:
                range_bin = "0-10"
            elif target_range < 20.0:
                range_bin = "10-20"
            elif target_range < 30.0:
                range_bin = "20-30"
            else:
                range_bin = "30-40"
            range_errors[range_bin].append(distance)
        for truth_id, prediction_id in by_truth.items():
            truth_sequences[truth_id].append((frame["time"], prediction_id))

    deta = tp / float(tp + fp + fn) if tp + fp + fn else 0.0
    association_sum = 0.0
    for pair, count in pair_counts.items():
        denominator = (truth_match_counts[pair[0]] +
                       prediction_match_counts[pair[1]] - count)
        association_sum += count * count / float(denominator) if denominator else 0.0
    assa = association_sum / float(tp) if tp else 0.0

    truth_ids = sorted({pair[0] for pair in pair_counts})
    prediction_ids = sorted({pair[1] for pair in pair_counts})
    idtp = 0
    if truth_ids and prediction_ids:
        counts = np.asarray([[pair_counts[(truth_id, prediction_id)]
                              for prediction_id in prediction_ids]
                             for truth_id in truth_ids])
        rows, columns = linear_sum_assignment(-counts)
        idtp = int(sum(counts[row, column] for row, column in zip(rows, columns)))
    idfp, idfn = total_predictions - idtp, total_truth - idtp
    idf1 = 2.0 * idtp / float(2 * idtp + idfp + idfn) \
        if 2 * idtp + idfp + idfn else 0.0

    switches = fragments = 0
    completeness = []
    longest_gaps = []
    ttft = []
    reacquisition = []
    for sequence in truth_sequences.values():
        previous_id = None
        seen_match = False
        in_gap = False
        gap_start = None
        longest = 0.0
        first_time = sequence[0][0] if sequence else None
        first_match = None
        matched_count = 0
        for stamp, prediction_id in sequence:
            if prediction_id is not None:
                matched_count += 1
                if first_match is None:
                    first_match = stamp
                if previous_id is not None and prediction_id != previous_id:
                    switches += 1
                if seen_match and in_gap:
                    fragments += 1
                    reacquisition.append(stamp - gap_start)
                    longest = max(longest, stamp - gap_start)
                seen_match = True
                in_gap = False
                previous_id = prediction_id
            elif seen_match and not in_gap:
                in_gap = True
                gap_start = stamp
        if in_gap and sequence:
            longest = max(longest, sequence[-1][0] - gap_start)
        longest_gaps.append(longest)
        completeness.append(matched_count / float(len(sequence)) if sequence else 0.0)
        ttft.append(first_match - first_time if first_match is not None else None)

    return {
        "threshold_m": threshold,
        "tp": tp, "fp": fp, "fn": fn,
        "precision": tp / float(tp + fp) if tp + fp else 0.0,
        "recall": tp / float(tp + fn) if tp + fn else 0.0,
        "DetA": deta, "AssA": assa, "HOTA": math.sqrt(deta * assa) if COMPUTE_HOTA else None,
        "IDF1": idf1, "id_switches": switches, "fragmentations": fragments,
        "GOSPA_mean": finite_mean(gospa_values),
        "position_RMSE_m": math.sqrt(float(np.mean(np.square(distances))))
        if distances else None,
        "position_MAE_m": finite_mean(distances),
        "velocity_RMSE_mps": math.sqrt(float(np.mean(velocity_errors)))
        if velocity_errors else None,
        "velocity_magnitude_MAE_mps": finite_mean(velocity_magnitude_errors),
        "velocity_direction_MAE_rad": finite_mean(velocity_direction_errors),
        "TTFT_mean_s": finite_mean(ttft),
        "track_completeness_mean": finite_mean(completeness),
        "longest_tracking_gap_s": max(longest_gaps) if longest_gaps else None,
        "reacquisition_latency_mean_s": finite_mean(reacquisition),
        "false_deletion_count": fragments,
        "range_position_RMSE_m": {
            key: math.sqrt(float(np.mean(np.square(values)))) if values else None
            for key, values in sorted(range_errors.items())},
    }


def quantize(point, voxel_size=0.5):
    return tuple(int(math.floor(float(value) / voxel_size)) for value in point)


def target_path_voxels(truth_frames, voxel_size=0.5, radius=0.75):
    output = set()
    last_time = {}
    cells = int(math.ceil(radius / voxel_size))
    for stamp, targets in truth_frames:
        for target in targets:
            base = quantize(target["position"], voxel_size)
            for x in range(base[0] - cells, base[0] + cells + 1):
                for y in range(base[1] - cells, base[1] + cells + 1):
                    for z in range(base[2] - cells, base[2] + cells + 1):
                        key = (x, y, z)
                        output.add(key)
                        last_time[key] = stamp
    return output, last_time




def static_voxels(world, voxel_size=0.5):
    output = set()

    def box_surface(center, size):
        half = np.asarray(size, dtype=float) / 2.0
        axes = [np.arange(-half[i], half[i] + voxel_size, voxel_size)
                for i in range(3)]
        for fixed_axis in range(3):
            moving = [axis for axis in range(3) if axis != fixed_axis]
            for fixed in (-half[fixed_axis], half[fixed_axis]):
                for first in axes[moving[0]]:
                    for second in axes[moving[1]]:
                        point = np.asarray(center, dtype=float)
                        point[fixed_axis] += fixed
                        point[moving[0]] += first
                        point[moving[1]] += second
                        output.add(quantize(point, voxel_size))

    for x in np.arange(-10.0, 50.0 + voxel_size, voxel_size):
        for y in np.arange(-15.0, 15.0 + voxel_size, voxel_size):
            output.add(quantize((x, y, 0.0), voxel_size))
    if world == "PW_open":
        pass
    elif world == "PW_mt":
        model = ET.parse(Path(__file__).resolve().parents[3] /
                         'src/mid360_multi_uav_sim/worlds/PW_mt.world').find("world/model[@name='formation_cylinder']")
        pose = list(map(float,model.findtext('pose').split()))
        cylinder = model.find('link/collision/geometry/cylinder')
        radius,height = float(cylinder.findtext('radius')),float(cylinder.findtext('length'))
        bottom,top = pose[2]-height/2,pose[2]+height/2
        for angle in np.linspace(0., 2.*math.pi, 32, endpoint=False):
            for z in np.linspace(bottom,top,int(math.ceil(height/voxel_size))+1):
                output.add(quantize((pose[0]+radius*math.cos(angle), pose[1]+radius*math.sin(angle), z), voxel_size))
        for x in np.arange(-radius,radius+voxel_size,voxel_size):
            for y in np.arange(-radius,radius+voxel_size,voxel_size):
                if x*x+y*y <= radius**2:
                    output.add(quantize((pose[0]+x,pose[1]+y,top),voxel_size))
    elif world == "PW_office":
        box_surface((20.9, 9.166528, 1.4224), (35.4, .1, 2.8448))
        box_surface((20.9, 7.333472, 1.4224), (35.4, .1, 2.8448))
    elif world == "PW_forest_seed0":
        forest = ET.parse(
            Path(__file__).resolve().parents[3] /
            "src/mid360_multi_uav_sim/worlds/PW_forest_seed0.world"
        ).getroot().find("world/model[@name='planning_forest_seed0']")
        for link in forest.findall("link"):
            pose = list(map(float, link.findtext("pose").split()))
            cylinder = link.find("collision/geometry/cylinder")
            radius = float(cylinder.findtext("radius"))
            height = float(cylinder.findtext("length"))
            count = max(
                24, int(math.ceil(2.0 * math.pi * radius / voxel_size)))
            for angle in np.linspace(
                    0.0, 2.0 * math.pi, count, endpoint=False):
                # SDF pose is the cylinder centre, not the bottom of its trunk.
                for z in np.linspace(-height/2., height/2.,
                                     int(math.ceil(height/voxel_size))+1):
                    output.add(quantize((
                        pose[0] + radius * math.cos(angle),
                        pose[1] + radius * math.sin(angle),
                        pose[2] + z), voxel_size))
    else:
        raise ValueError(f"unsupported world: {world}")
    return output


def flatten(prefix, value, rows):
    if isinstance(value, dict):
        for key in sorted(value):
            flatten("{}.{}".format(prefix, key) if prefix else str(key), value[key], rows)
    elif isinstance(value, (list, tuple)):
        rows.append((prefix, json.dumps(value, sort_keys=True)))
    else:
        rows.append((prefix, value))


def parse_resource(path):
    return parse_resource_metrics(path).get("peak_rss_kib")


def parse_elapsed(value):
    fields = [float(item) for item in value.split(":")]
    if len(fields) == 2:
        return 60.0 * fields[0] + fields[1]
    if len(fields) == 3:
        return 3600.0 * fields[0] + 60.0 * fields[1] + fields[2]
    return None


def parse_resource_metrics(path):
    output = {}
    if not path or not os.path.exists(path):
        return output
    with open(path, encoding="utf-8") as stream:
        for line in stream:
            if "Maximum resident set size" in line:
                output["peak_rss_kib"] = int(line.rsplit(":", 1)[1].strip())
            elif "User time (seconds)" in line:
                output["user_cpu_s"] = float(line.rsplit(":", 1)[1].strip())
            elif "System time (seconds)" in line:
                output["system_cpu_s"] = float(line.rsplit(":", 1)[1].strip())
            elif "Percent of CPU" in line:
                output["wall_cpu_percent"] = float(
                    line.rsplit(":", 1)[1].strip().rstrip("%"))
            elif "Elapsed (wall clock) time" in line:
                output["wall_elapsed_s"] = parse_elapsed(
                    line.split("):", 1)[1].strip())
    return output


def read_cloud_keys(message):
    return {quantize((x, y, z)) for x, y, z in point_cloud2.read_points(
        message, field_names=("x", "y", "z"), skip_nans=True)}


def scan_end_stamp(message):
    """State evaluation epoch from acquisition metadata, never a fixed delay."""
    begin = message.header.stamp.to_sec()
    if hasattr(message, 'rays'):
        offset = max((ray.source.offset_time_ns for ray in message.rays), default=0)
    else:
        field = next((f for f in message.fields if f.name == 't'), None)
        if field is None:
            raise RuntimeError('raw sensor cloud lacks acquisition time field t')
        if field.datatype != 6:
            raise RuntimeError('expected uint32 nanosecond acquisition offsets')
        offsets = np.ndarray((message.height, message.width),
            dtype='>u4' if message.is_bigendian else '<u4', buffer=message.data,
            offset=field.offset, strides=(message.row_step, message.point_step))
        offset = int(offsets.max(initial=0))
    return begin + 1.e-9 * offset


def interpolate_truth(samples, stamp):
    times = [row[0] for row in samples]
    if not times or stamp < times[0]-1.e-9 or stamp > times[-1]+1.e-9:
        raise RuntimeError('ground truth does not bracket state evaluation time')
    right = min(bisect.bisect_left(times, stamp), len(times)-1)
    left = max(0, right-1)
    first, second = samples[left][1], samples[right][1]
    alpha = ((stamp-times[left])/(times[right]-times[left])
             if right != left and times[right] != times[left] else 0.)
    if isinstance(first, dict):
        result = dict(first)
        for key in ('position', 'velocity'):
            result[key] = tuple((1-alpha)*np.asarray(first[key])+alpha*np.asarray(second[key]))
        return result
    return tuple((1-alpha)*np.asarray(first)+alpha*np.asarray(second))


def prediction_at(track, stamp):
    state_stamp = track.last_prediction.to_sec()
    dt = stamp-state_stamp
    if state_stamp <= 0 or dt < -1.e-9:
        raise RuntimeError('invalid or future track state epoch')
    dt = max(0., dt)
    p = np.array([track.position.x, track.position.y, track.position.z])
    v = np.array([track.velocity.x, track.velocity.y, track.velocity.z])
    a = np.array([track.acceleration.x, track.acceleration.y, track.acceleration.z])
    if not np.isfinite(np.r_[p, v, a]).all():
        raise RuntimeError('nonfinite track state')
    return tuple(p+dt*v+.5*dt*dt*a), tuple(v+dt*a)


def load_source(path, input_topic, scenario, allow_empty_truth=False):
    truth_frames = []
    observers = []
    observer_rotations = []
    scenario_events = []
    input_frame_stamps = []
    frame_times = {}
    target_odometry = defaultdict(list)
    target_topics = {
        "/mid360_multi_uav_sim/ground_truth/{}/odom".format(item["id"]):
        str(item["id"]) for item in scenario.get("targets", [])}
    with rosbag.Bag(path) as bag:
        for topic, message, _ in bag.read_messages(topics=[
                "/evaluation/visibility_ground_truth",
                "/mid360_multi_uav_sim/ground_truth/uav1/odom",
                "/mid360_multi_uav_sim/scenario_events",
                input_topic, *target_topics]):
            if topic == "/evaluation/visibility_ground_truth":
                truth_frames.append((message.header.stamp.to_sec(), [{
                    "id": target.target_id,
                    "position": (target.position.x, target.position.y, target.position.z),
                    "velocity": (target.velocity.x, target.velocity.y, target.velocity.z),
                    "present": getattr(target, "present", True),
                    "in_range": getattr(target, "in_range", True),
                    "in_fov": getattr(target, "in_fov", True),
                    "line_of_sight": getattr(target, "line_of_sight", True),
                    "actual_returns": getattr(target, "true_actual_target_return_count", 0),
                } for target in message.targets]))
            elif topic.endswith("uav1/odom"):
                position = message.pose.pose.position
                observers.append((message.header.stamp.to_sec(),
                                  (position.x, position.y, position.z)))
                q=message.pose.pose.orientation
                rotation=np.array([q.x,q.y,q.z,q.w])
                if observer_rotations and np.dot(rotation,observer_rotations[-1][1])<0:
                    rotation=-rotation
                observer_rotations.append((message.header.stamp.to_sec(),tuple(rotation)))
            elif topic in target_topics:
                position = message.pose.pose.position
                velocity = message.twist.twist.linear
                target_odometry[target_topics[topic]].append((
                    message.header.stamp.to_sec(), {
                        "id": target_topics[topic],
                        "position": (position.x, position.y, position.z),
                        "velocity": (velocity.x, velocity.y, velocity.z),
                        "present": True, "in_range": True, "in_fov": True,
                        "line_of_sight": True, "actual_returns": 0,
                    }))
            elif topic == "/mid360_multi_uav_sim/scenario_events":
                try:
                    scenario_events.append(json.loads(message.data))
                except (TypeError, ValueError):
                    pass
            elif topic == input_topic:
                source_stamp = message.header.stamp.to_sec()
                key = round(source_stamp, 9)
                if key in frame_times:
                    raise RuntimeError('duplicate source-frame timestamp')
                input_frame_stamps.append(source_stamp)
                frame_times[key] = scan_end_stamp(message)
    truth_frames.sort()
    observers.sort()
    observer_rotations.sort(key=lambda row: row[0])
    score_start = next((item["sim_time"] for item in scenario_events
                        if item.get("event") == "scoring_start"), None)
    score_end = next((item["sim_time"] for item in scenario_events
                      if item.get("event") == "scoring_end"), None)
    if score_start is None or score_end is None or score_start >= score_end:
        raise RuntimeError("source bag is missing a valid scoring interval")
    truth_frames = [(stamp, targets) for stamp, targets in truth_frames
                    if score_start <= stamp <= score_end]
    input_frame_stamps = [stamp for stamp in input_frame_stamps
                          if score_start <= stamp <= score_end]
    frame_times = {round(stamp, 9):frame_times[round(stamp, 9)]
                   for stamp in input_frame_stamps}
    if target_odometry:
        # Score at every sensor frame. Visibility diagnostics may run at a
        # lower cadence than a 262k-sample Ouster stream; odometry interpolation
        # keeps the predeclared geometric domain from silently deleting FNs.
        for values in target_odometry.values():
            values.sort(key=lambda row: row[0])
        truth_frames = [(frame_times[round(stamp, 9)],
                         [interpolate_truth(target_odometry[target_id],
                                            frame_times[round(stamp, 9)])
                          for target_id in sorted(target_odometry)])
                        for stamp in input_frame_stamps]
    elif scenario.get('targets'):
        raise RuntimeError('state-time scoring requires recorded target odometry')
    if not truth_frames and not allow_empty_truth:
        raise RuntimeError("source bag contains no scored visibility truth")
    if not input_frame_stamps:
        raise RuntimeError("source bag contains no scored checked-ray input")
    if not truth_frames:
        truth_frames = [(frame_times[round(stamp, 9)], []) for stamp in input_frame_stamps]
    if scenario.get('los_only_target_ids'):
        from scipy.spatial.transform import Rotation
        from run_benchmark import static_primitives
        primitives=static_primitives(scenario['world'])
        mount=.1414 if input_topic=='/uav1/os_cloud_nodelet/points' else .168
        for stamp, targets in truth_frames:
            rotation=Rotation.from_quat(interpolate_truth(observer_rotations,stamp))
            origin=np.asarray(interpolate_truth(observers,stamp))+rotation.apply([0,0,mount])
            for target in targets:
                if target['id'] in scenario['los_only_target_ids']:
                    target['line_of_sight']=static_center_los(origin,target['position'],primitives)
    return (truth_frames, observers, scenario_events, score_start, score_end,
            input_frame_stamps, frame_times)


def load_run(path, algorithm, score_start, score_end, frame_times=None, time_audit=None):
    aerocover = algorithm.startswith("AeroCOVER")
    ouster = algorithm in ("AeroCOVER-OS1", "VoFOD-OS1", "VoFOD-Original-OS1",
                           "VoFOD-Mid360-Adapted-OS1")
    track_topic = ("/aerocover_os1/tracks" if algorithm == "AeroCOVER-OS1"
                   else "/aerocover/tracks" if aerocover
                   else "/uav1/ouster_vofod/tracks" if ouster
                   else "/uav1/batch/b0/tracks")
    background_topic = ("/aerocover_os1/background_points"
                        if algorithm == "AeroCOVER-OS1"
                        else "/aerocover/background_points" if aerocover
                        else "/uav1/vofod_mid360/background_points")
    free_topic = None if aerocover else "/uav1/vofod_mid360/free_voxels"
    diagnostics_topic = ("/aerocover_os1/diagnostics"
                         if algorithm == "AeroCOVER-OS1"
                         else "/aerocover/diagnostics" if aerocover
                         else "/uav1/vofod_mid360/map_update_diagnostics")
    topics = [track_topic, background_topic, diagnostics_topic]
    completion_topic = (None if aerocover else
        '/uav1/ouster_vofod/lidar_tracker/frame_complete' if ouster else
        '/uav1/batch/b0/lidar_tracker_mid360/frame_complete')
    if completion_topic:
        topics.append(completion_topic)
    if free_topic:
        topics.append(free_topic)

    frames_by_stamp = {}
    baseline_tracks = {}
    baseline_first_seen = {}
    background = []
    free = []
    last_free_message = None
    timing = []
    timing_by_source = {}
    time_audit = {} if time_audit is None else time_audit
    diagnostics = []
    completed_sources = set()
    with rosbag.Bag(path) as bag:
        for topic, message, _ in bag.read_messages(topics=topics):
            stamp = (message.header.stamp.to_sec()
                     if hasattr(message, "header") else 0.0)
            if topic == completion_topic:
                source_stamp = round(stamp, 9)
                if frame_times is not None and source_stamp in frame_times and \
                        message.points_status in (0,1) and message.detections_status in (0,1):
                    completed_sources.add(source_stamp)
            elif topic == track_topic:
                if frame_times is None:
                    raise RuntimeError('track scoring requires source-frame clock mapping')
                source_header = getattr(message, 'source_header', None)
                if source_header is None or source_header.stamp.to_sec() <= 0:
                    raise RuntimeError('Tracks lacks source_header; replay with the corrected publisher')
                source_stamp = round(source_header.stamp.to_sec(), 9)
                if source_stamp not in frame_times:
                    continue
                stamp = frame_times[source_stamp]
                # A delayed VoFOD callback may publish state already updated by
                # a later input. Never back-project that future information.
                latest_allowed = stamp if aerocover else source_stamp
                if any(max(track.last_prediction.to_sec(), track.last_correction.to_sec())
                       > latest_allowed+1.e-9 for track in message.tracks):
                    time_audit['future_state_publications_rejected'] = time_audit.get(
                        'future_state_publications_rejected', 0)+1
                    continue
                if aerocover and abs(message.header.stamp.to_sec()-stamp) > 1.e-9:
                    raise RuntimeError('AeroCOVER Tracks.header is not its decision epoch')
                predictions = []
                frame_tracks = []
                for track in message.tracks:
                    position, velocity = prediction_at(track, stamp)
                    confirmed = track.n_detections >= 2
                    track_id = int(track.id)
                    birth_time = baseline_first_seen.setdefault(
                        track_id, stamp)
                    frame_tracks.append({
                        "stamp": stamp,
                        "source_stamp": source_stamp,
                        "state_stamp": track.last_prediction.to_sec(),
                        "id": track_id,
                        "state": "active" if confirmed else "tentative",
                        "existence": float(track.confidence),
                        "stale_s": max(
                            0.0, stamp - track.last_correction.to_sec()),
                        "birth_time": birth_time,
                        "birth_evidence_type": 8 if aerocover else 6,
                        "position": position,
                    })
                    if confirmed:
                        predictions.append({
                            "id": track_id,
                            "position": position,
                            "velocity": velocity,
                        })
                frames_by_stamp[round(stamp, 9)] = predictions
                baseline_tracks[round(stamp, 9)] = frame_tracks
            elif (topic == background_topic and
                  score_start <= stamp <= score_end):
                background.append((stamp, read_cloud_keys(message)))
            elif (free_topic and topic == free_topic and
                  score_start <= stamp <= score_end):
                # map_metrics consumes only the last snapshot in bag order.
                last_free_message = (stamp, message)
            elif (topic == diagnostics_topic and
                  score_start <= stamp <= score_end):
                source_stamp = round(stamp, 9)
                if frame_times is not None:
                    if source_stamp not in frame_times:
                        continue
                    stamp = frame_times[source_stamp]
                timing_by_source[source_stamp] = (stamp, float(message.total_ms))
                if aerocover:
                    numeric = {}
                    for key in (
                            "point_count", "valid_return_count",
                            "no_return_count", "invalid_ray_count",
                            "fifo_ray_count", "fifo_span_s",
                            "expired_ray_count", "inserted_ray_count",
                            "residual_point_count", "component_count",
                            "active_candidate_count", "active_track_count",
                            "track_match_count", "track_birth_count",
                            "track_deletion_count",
                            "spatiotemporal_history_ready",
                            "point_fifo_scan_count",
                            "point_fifo_point_count",
                            "spatiotemporal_component_count",
                            "temporal_slice_count",
                            "spatiotemporal_background_component_count",
                            "propagated_background_component_count",
                            "spatiotemporal_target_component_count",
                            "shell_prefilter_observation_count",
                            "adaptive_shell_shrunk_count",
                            "adaptive_shell_blocked_count",
                            "shell_prefilter_max_observable_bins",
                            "shell_prefilter_max_supported_bins",
                            "self_support_violation_count",
                            "future_stamp_violation_count",
                            "reference_incremental_mismatch_count",
                            "input_ms", "cluster_ms", "evidence_ms",
                            "association_ms", "evidence_rebuild_ms",
                            "evidence_summary_ms", "ray_insertion_ms",
                            "validation_ms", "expiry_ms", "grid_build_ms",
                            "neighbor_union_ms", "temporal_slice_ms",
                            "shell_prefilter_ms", "total_ms"):
                        numeric[f"aerocover_{key}"] = float(
                            getattr(message, key, 0.0))
                    candidates = list(message.candidates)
                    if candidates:
                        numeric["aerocover_candidate_shell_mean"] = (
                            finite_mean([float(item.shell_coverage)
                                         for item in candidates]))
                        numeric["aerocover_shell_pass_count"] = float(sum(
                            bool(item.shell_pass) for item in candidates))
                    numeric["aerocover_births"] = float(len(message.births))
                    numeric["aerocover_shell_births"] = float(
                        len(message.births))
                    diagnostics.append((stamp, numeric))
                else:
                    numeric = {}
                    for key in message.__slots__:
                        value = getattr(message, key)
                        if isinstance(value, (bool, int, float)):
                            numeric[key] = float(value)
                    diagnostics.append((stamp, numeric))

    if last_free_message is not None:
        stamp, message = last_free_message
        free.append((stamp, read_cloud_keys(message)))
    all_tracks = sorted(
        [item for values in baseline_tracks.values() for item in values],
        key=lambda item: (item["stamp"], item["id"]))
    frames = sorted(frames_by_stamp.items())
    timing = sorted(timing_by_source.values())
    if frame_times is not None:
        time_audit['completed_source_frames'] = len(timing_by_source) if aerocover else len(completed_sources)
        time_audit['expected_source_frames'] = len(frame_times)
    return frames, all_tracks, background, free, timing, diagnostics


def aerocover_mechanism_metrics(path, truth_frames, score_start, score_end,
                                diagnostics_topic="/aerocover/diagnostics", frame_times=None):
    """Score AeroCOVER mechanism evidence without exposing truth to the node."""
    candidate_rows = []
    birth_rows = []
    violation_totals = defaultdict(int)
    with rosbag.Bag(path) as bag:
        for _, message, _ in bag.read_messages(
                topics=[diagnostics_topic]):
            stamp = message.header.stamp.to_sec()
            if not score_start <= stamp <= score_end:
                continue
            if frame_times is not None:
                stamp = frame_times.get(round(stamp, 9))
                if stamp is None:
                    continue
            truth = [item for item in (nearest(truth_frames, stamp) or [])
                     if item.get("present", True) and
                     item.get("in_range", True) and
                     item.get("in_fov", True)]
            snapshots = {int(item.candidate_id): item
                         for item in message.candidates}
            for item in message.candidates:
                position = np.asarray(
                    (item.centroid.x, item.centroid.y, item.centroid.z))
                distances = [np.linalg.norm(
                    position - np.asarray(target["position"]))
                             for target in truth]
                matched = bool(distances) and bool(
                    float(min(distances)) <= MAIN_THRESHOLD_M)
                candidate_rows.append({
                    "stamp": stamp,
                    "candidate_id": int(item.candidate_id),
                    "label": "target" if matched else "background_or_clutter",
                    "nearest_truth_distance_m": float(min(distances))
                    if distances else None,
                    "candidate_class": int(item.candidate_class),
                    "rejection_reason": item.rejection_reason,
                    "point_count": int(item.point_count),
                    "shell_coverage": float(item.shell_coverage),
                    "observable_bins": int(item.observable_bin_count),
                    "supported_bins": int(item.supported_bin_count),
                    "distinct_shell_scans": int(item.distinct_shell_scans),
                    "valid_return_shell_score":
                        float(item.valid_return_shell_score),
                    "no_return_shell_score":
                        float(item.no_return_shell_score),
                    "shell_pass": bool(item.shell_pass),
                    "birth_support_now": bool(item.birth_support_now),
                    "birth_history": item.birth_history,
                })
            for item in message.births:
                position = np.asarray(
                    (item.position.x, item.position.y, item.position.z))
                distances = [np.linalg.norm(
                    position - np.asarray(target["position"]))
                             for target in truth]
                matched = bool(distances) and bool(
                    float(min(distances)) <= MAIN_THRESHOLD_M)
                snapshot = snapshots.get(int(item.candidate_id))
                valid = no_return = 0.0
                if snapshot is not None:
                    valid = float(snapshot.valid_return_shell_score)
                    no_return = float(snapshot.no_return_shell_score)
                birth_rows.append({
                    "stamp": stamp,
                    "track_id": int(getattr(
                        item, "track_id", item.candidate_id)),
                    "candidate_id": int(item.candidate_id),
                    "evidence": "SHELL",
                    "matched_target": matched,
                    "nearest_truth_distance_m": float(min(distances))
                    if distances else None,
                    "ttft_s": float(item.ttft_s),
                    "birth_history": item.birth_history,
                    "shell_coverage": float(item.shell_coverage),
                    "point_count": int(item.point_count),
                    "valid_return_score": valid,
                    "no_return_score": no_return,
                    "no_return_fraction": no_return / (valid + no_return)
                    if valid + no_return > 0.0 else None,
                })
            for key in ("self_support_violation_count",
                        "future_stamp_violation_count",
                        "reference_incremental_mismatch_count"):
                violation_totals[key] += int(getattr(message, key))

    def labelled_distribution(key, label):
        return distribution([row[key] for row in candidate_rows
                             if row["label"] == label])

    evidence_counts = {
        name: sum(row["evidence"] == name for row in birth_rows)
        for name in ("SHELL",)
    }
    matched_counts = {
        name: sum(row["evidence"] == name and row["matched_target"]
                  for row in birth_rows)
        for name in ("SHELL",)
    }
    metrics = {
        "candidate_observations": len(candidate_rows),
        "target_candidate_observations": sum(
            row["label"] == "target" for row in candidate_rows),
        "background_candidate_observations": sum(
            row["label"] != "target" for row in candidate_rows),
        "target": {
            "shell_coverage": labelled_distribution(
                "shell_coverage", "target"),
            "observable_bins": labelled_distribution(
                "observable_bins", "target"),
            "supported_bins": labelled_distribution(
                "supported_bins", "target"),
        },
        "background": {
            "shell_coverage": labelled_distribution(
                "shell_coverage", "background_or_clutter"),
            "observable_bins": labelled_distribution(
                "observable_bins", "background_or_clutter"),
            "supported_bins": labelled_distribution(
                "supported_bins", "background_or_clutter"),
        },
        "birth_count": len(birth_rows),
        "matched_birth_count": int(sum(row["matched_target"]
                                        for row in birth_rows)),
        "false_birth_count": int(sum(not row["matched_target"]
                                      for row in birth_rows)),
        "birth_provenance_count": evidence_counts,
        "matched_birth_provenance_count": matched_counts,
        "birth_no_return_fraction": distribution([
            row["no_return_fraction"] for row in birth_rows
            if row["no_return_fraction"] is not None]),
        **dict(violation_totals),
    }
    return metrics, candidate_rows, birth_rows




def epistemic_metrics(scenario_id, tracks, diagnostics, duration, score_start,
                      truth_frames=()):
    del scenario_id, diagnostics
    by_id = defaultdict(list)
    for item in tracks:
        by_id[item["id"]].append(item)
    confirmed = {
        track_id: min(samples, key=lambda item: item["stamp"])
        for track_id, samples in by_id.items()
        if any(item["state"] == "active" for item in samples)
    }
    matched_ids = set()
    for track_id, samples in by_id.items():
        for item in samples:
            targets = nearest(truth_frames, item["stamp"]) or []
            if item["state"] == "active" and targets and min(
                    np.linalg.norm(np.asarray(item["position"]) -
                                   np.asarray(target["position"]))
                    for target in targets) <= MAIN_THRESHOLD_M:
                matched_ids.add(track_id)
                break
    provenance = {name: 0 for name in PROVENANCE_NAMES.values()}
    matched_provenance = {name: 0 for name in PROVENANCE_NAMES.values()}
    ttft = {name: [] for name in PROVENANCE_NAMES.values()}
    for track_id, item in confirmed.items():
        name = PROVENANCE_NAMES.get(item["birth_evidence_type"])
        if not name:
            continue
        provenance[name] += 1
        ttft[name].append(max(0.0, item["birth_time"] - score_start))
        if track_id in matched_ids:
            matched_provenance[name] += 1
    false_confirmed = len(set(confirmed) - matched_ids)
    return {
        "birth_provenance_count": provenance,
        "matched_birth_provenance_count": matched_provenance,
        "total_birth_count": len(confirmed),
        "matched_birth_count": len(matched_ids & set(confirmed)),
        "unmatched_birth_count": false_confirmed,
        "confirmed_false_tracks": false_confirmed,
        "birth_ttft_s": {
            key: distribution(value) for key, value in ttft.items()},
        "birth_success": bool(matched_ids & set(confirmed))
        if truth_frames else None,
        "matched_target_birth_recall": float(
            bool(matched_ids & set(confirmed))) if truth_frames else None,
        "false_target_confirmation_rate": (
            false_confirmed / float(len(confirmed)) if confirmed else 0.0),
        "false_births_per_min": (
            false_confirmed / max(duration / 60.0, 1.0e-9)),
    }



def distribution(values):
    values = [float(value) for value in values if math.isfinite(float(value))]
    return {
        "count": len(values),
        "sum": sum(values),
        "mean": finite_mean(values),
        "p50": percentile(values, 50),
        "p95": percentile(values, 95),
        "p99": percentile(values, 99),
        "max": max(values, default=None),
    }


def track_health_metrics(tracks, duration, no_target=False):
    confirmed = [item for item in tracks if item["state"] == "active"]
    tentative_ids = {item["id"] for item in tracks
                     if item["state"] == "tentative"}
    confirmed_ids = {item["id"] for item in confirmed}
    by_stamp = defaultdict(list)
    for item in tracks:
        by_stamp[item["stamp"]].append(item)
    confirmed_counts = [sum(item["state"] == "active" for item in frame)
                        for frame in by_stamp.values()]
    stale = {str(threshold): sum(item["stale_s"] > threshold
                                 for item in confirmed)
             for threshold in (0.5, 1.0, 3.0)}
    stale_tracks = {str(threshold): len({item["id"] for item in confirmed
                                        if item["stale_s"] > threshold})
                    for threshold in (0.5, 1.0, 3.0)}
    stale_existence = [item["existence"] for item in confirmed
                       if item["stale_s"] > 0.5]
    update_intervals = []
    for track_id in {item["id"] for item in tracks}:
        previous = None
        for item in sorted((item for item in tracks
                            if item["id"] == track_id),
                           key=lambda item: item["stamp"]):
            measurement_time = item["stamp"] - item["stale_s"]
            if previous is not None and measurement_time > previous + 1.0e-6:
                update_intervals.append(measurement_time - previous)
            previous = max(previous, measurement_time) \
                if previous is not None else measurement_time
    minutes = max(duration / 60.0, 1.0e-9)
    return {
        "unique_tentative_tracks": len(tentative_ids),
        "unique_confirmed_tracks": len(confirmed_ids),
        "false_tentative_tracks_per_min": len(tentative_ids) / minutes
        if no_target else None,
        "false_confirmed_tracks_per_min": len(confirmed_ids) / minutes
        if no_target else None,
        "confirmed_track_count_peak": max(confirmed_counts, default=0),
        "confirmed_track_count_final": confirmed_counts[-1]
        if confirmed_counts else 0,
        "confirmed_stale_samples": stale,
        "confirmed_stale_unique_tracks": stale_tracks,
        "max_stale_age_s": max((item["stale_s"] for item in confirmed),
                               default=0.0),
        "measurement_age_s": distribution(
            [item["stale_s"] for item in confirmed]),
        "measurement_update_interval_s": distribution(update_intervals),
        "longest_measurement_update_gap_s": max(
            update_intervals, default=None),
        "mean_stale_existence": finite_mean(stale_existence),
    }


def diagnostics_metrics(diagnostics):
    keys = {key for _, values in diagnostics for key in values}
    summaries = {
        key: distribution([values[key] for _, values in diagnostics
                           if key in values])
        for key in sorted(keys)
    }
    runtime_keys = sorted(key for key in summaries if key.endswith("_ms"))
    count_keys = sorted(key for key in summaries if not key.endswith("_ms"))
    return {
        "module_runtime_ms": {key: summaries[key] for key in runtime_keys
                              if key in summaries},
        "complexity": {key: summaries[key] for key in count_keys
                       if key in summaries},
    }


def initialization_metrics(algorithm, diagnostics):
    if algorithm.startswith("AeroCOVER"):
        return {"applicable": False}
    first_seed = next((values.get("first_seed_effective_stamp", stamp)
                       for stamp, values in diagnostics
                       if values.get("background_seed_effective", 0.0) > 0.5),
                      None)
    first_ready = next((values.get("detection_ready_stamp", stamp)
                        for stamp, values in diagnostics
                        if values.get("detection_ready", 0.0) > 0.5), None)
    return {
        "applicable": True,
        "background_mode": "native_rangefinder",
        "first_seed_effective_stamp_s": first_seed,
        "detection_ready_stamp_s": first_ready,
        "seed_effective": first_seed is not None,
        "detection_ready": first_ready is not None,
    }


def map_metrics(background, free, truth_frames, world):
    path, last_time = target_path_voxels(truth_frames)
    background_union = (
        set().union(*(keys for _, keys in background)) if background else set())
    final_background = background[-1][1] if background else set()
    final_free = free[-1][1] if free else set()
    contamination = background_union & path
    recovery_first = {}
    trail = 0.0
    for stamp, keys in background:
        for key in keys & path:
            trail = max(trail, stamp - last_time.get(key, stamp))
            if stamp >= last_time.get(key, stamp):
                recovery_first.setdefault(
                    key, stamp - last_time.get(key, stamp))
    static = static_voxels(world)
    false_free = final_free & static
    contamination_ratio = (
        len(contamination) / float(len(path)) if path else None)
    return {
        "target_contamination_ratio": contamination_ratio,
        "map_contamination_ratio": contamination_ratio,
        "background_recovery_latency_s": distribution(
            list(recovery_first.values())),
        "trail_duration_max_s": max(0.0, trail),
        "static_background_recall": (
            len(final_background & static) / float(len(static))
            if static else None),
        "false_free_rate": (
            len(false_free) / float(len(static)) if static else None),
        "certified_free_precision": (
            1.0 - len(false_free) / float(len(final_free))
            if final_free else None),
        "false_certified_free": len(false_free),
        "final_background_voxels": len(final_background),
        "final_free_voxels": len(final_free),
    }




def explicit_ttft(frames, scenario_events, score_start, named_event=None):
    first_visible = next((frame["time"] for frame in frames
                          if any(target.get("present", True)
                                 for target in frame["truth"])), None)
    first_match = next((frame["time"] for frame in frames
                        if assignments(frame["truth"], frame["predictions"],
                                       MAIN_THRESHOLD_M)), None)
    named_time = next((item.get("sim_time") for item in scenario_events
                       if named_event and item.get("event") == named_event), None)

    def latency(origin):
        return first_match - origin \
            if first_match is not None and origin is not None else None

    return {
        "first_match_stamp_s": first_match,
        "absolute_s": latency(score_start),
        "from_first_visible_s": latency(first_visible),
        "from_first_eligible_s": latency(first_visible),
        "named_event": named_event,
        "named_event_stamp_s": named_time,
        "from_named_event_s": latency(named_time),
    }


def per_target_analysis(frames, tracks, scenario_events, scenario,
                        algorithm=None):
    """Return per-truth birth/identity metrics and an auditable timeline."""
    target_configs = {str(item["id"]): item
                      for item in scenario.get("targets", [])}
    event_times = {item.get("event"): item.get("sim_time")
                   for item in scenario_events}
    track_samples = defaultdict(list)
    for item in tracks:
        track_samples[item["id"]].append(item)
    for samples in track_samples.values():
        samples.sort(key=lambda item: item["stamp"])

    timeline = []
    by_truth = defaultdict(list)
    cardinality_errors = []
    for frame in frames:
        matches = assignments(frame["truth"], frame["predictions"],
                              MAIN_THRESHOLD_M)
        matched = {row: (frame["predictions"][column], distance)
                   for row, column, distance in matches}
        cardinality_errors.append(abs(
            len(frame["truth"]) - len(frame["predictions"])))
        for index, truth in enumerate(frame["truth"]):
            prediction, distance = matched.get(index, (None, None))
            visible = bool(
                truth.get("present", True))
            row = {
                "stamp": frame["time"], "truth_id": truth["id"],
                "visible": visible,
                "truth_position": tuple(truth["position"]),
                "matched_track_id": prediction["id"] if prediction else None,
                "match_distance_m": distance,
                "track_position": tuple(prediction["position"])
                if prediction else None,
                "truth_cardinality": len(frame["truth"]),
                "active_track_cardinality": len(frame["predictions"]),
            }
            timeline.append(row)
            by_truth[str(truth["id"])].append(row)

    output = []
    dominant_truth_by_track = {}
    for truth_id in sorted(target_configs):
        samples = by_truth.get(truth_id, [])
        matched = [item for item in samples
                   if item["matched_track_id"] is not None]
        visible = [item for item in samples if item["visible"]]
        unique_ids = sorted({item["matched_track_id"] for item in matched})
        counts = defaultdict(int)
        for item in matched:
            counts[item["matched_track_id"]] += 1
        dominant = max(counts, key=counts.get) if counts else None
        if dominant is not None:
            dominant_truth_by_track[dominant] = truth_id

        switches = fragments = 0
        previous_id = None
        seen = in_gap = False
        for item in samples:
            track_id = item["matched_track_id"]
            if track_id is not None:
                if previous_id is not None and track_id != previous_id:
                    switches += 1
                if seen and in_gap:
                    fragments += 1
                seen, in_gap, previous_id = True, False, track_id
            elif seen:
                in_gap = True

        target_config = target_configs[truth_id]
        named_event = target_config.get("ttft_event") or \
            scenario.get("ttft_events", {}).get(truth_id) or \
            scenario.get("ttft_event")
        named_stamp = event_times.get(named_event)
        first_match = matched[0]["stamp"] if matched else None
        first_visible = visible[0]["stamp"] if visible else None
        birth_sample = min(track_samples.get(dominant, []),
                           key=lambda item: item["stamp"], default=None)
        birth_code = birth_sample.get("birth_evidence_type") \
            if birth_sample else None
        start_event = target_config.get("occlusion_start_event")
        end_event = target_config.get("occlusion_end_event")
        start_stamp, end_stamp = (event_times.get(start_event),
                                  event_times.get(end_event))
        before = [item for item in matched
                  if start_stamp is not None and item["stamp"] < start_stamp]
        after = [item for item in matched
                 if end_stamp is not None and item["stamp"] >= end_stamp]
        reacquired_same_id = None
        reacquisition_latency = None
        if start_stamp is not None and end_stamp is not None:
            before_id = before[-1]["matched_track_id"] if before else None
            after_id = after[0]["matched_track_id"] if after else None
            reacquired_same_id = before_id is not None and before_id == after_id
            reacquisition_latency = after[0]["stamp"] - end_stamp \
                if after else None
        output.append({
            "scene": scenario["scenario_id"].split("_", 1)[0],
            "algorithm": algorithm,
            "truth_id": truth_id,
            "birth_success": bool(matched),
            "birth_time": birth_sample.get("birth_time")
            if birth_sample else None,
            "TTFT_from_first_visible":
                first_match - first_visible
                if first_match is not None and first_visible is not None else None,
            "TTFT_from_first_eligible":
                first_match - first_visible
                if first_match is not None and first_visible is not None else None,
            "TTFT_from_named_event":
                first_match - named_stamp
                if first_match is not None and named_stamp is not None else None,
            "birth_provenance": PROVENANCE_NAMES.get(birth_code),
            "birth_semantic_label": SEMANTIC_BIRTH_LABELS.get(birth_code),
            "first_confirmed_track_id":
                matched[0]["matched_track_id"] if matched else None,
            "matched_recall": len([
                item for item in visible
                if item["matched_track_id"] is not None]) /
                float(len(visible)) if visible else None,
            "reacquisition_delay": reacquisition_latency,
            "old_id_recovered": reacquired_same_id,
            "per_target_birth_success": bool(matched),
            "per_target_TTFT_from_first_visible":
                first_match - first_visible
                if first_match is not None and first_visible is not None else None,
            "per_target_TTFT_from_first_eligible":
                first_match - first_visible
                if first_match is not None and first_visible is not None else None,
            "named_event": named_event,
            "per_target_TTFT_from_named_event":
                first_match - named_stamp
                if first_match is not None and named_stamp is not None else None,
            "per_target_birth_provenance": birth_code,
            "dominant_track_id": dominant,
            "unique_track_ids_per_truth": len(unique_ids),
            "unique_track_ids": unique_ids,
            "duplicate_births_per_truth": max(0, len(unique_ids) - 1),
            "IDSW": switches,
            "fragmentation": fragments,
            "reacquired_same_id": reacquired_same_id,
            "reacquisition_latency": reacquisition_latency,
            "wrong_reactivation_count": 0,
        })

    wrong_total = 0
    for track_id, samples in track_samples.items():
        previous = samples[0].get("reactivation_count", 0) if samples else 0
        expected_truth = dominant_truth_by_track.get(track_id)
        for sample in samples[1:]:
            count = sample.get("reactivation_count", 0)
            if count > previous and expected_truth is not None:
                frame = nearest([(item["time"], item) for item in frames],
                                sample["stamp"])
                targets = frame["truth"] if frame else []
                closest = min(targets, key=lambda target: np.linalg.norm(
                    np.asarray(sample["position"]) -
                    np.asarray(target["position"])), default=None)
                wrong = closest is None or str(closest["id"]) != expected_truth
                if wrong:
                    wrong_total += 1
                    for row in output:
                        if row["truth_id"] == expected_truth:
                            row["wrong_reactivation_count"] += 1
            previous = count

    return output, timeline, {
        "active_track_cardinality_MAE": finite_mean(cardinality_errors),
        "wrong_reactivation_count": wrong_total,
    }


def evaluate(source_bag, run_bag, algorithm, scenario_file, output_dir,
             resource_file=None, seed_override=None):
    with open(scenario_file, encoding="utf-8") as stream:
        scenario = yaml.safe_load(stream)
    no_target = not scenario.get("targets")
    input_topic = ("/uav1/os_cloud_nodelet/points"
                   if algorithm in ("AeroCOVER-OS1", "VoFOD-OS1", "VoFOD-Original-OS1",
                                    "VoFOD-Mid360-Adapted-OS1")
                   else "/uav1/mid360/rays_checked")
    (truth_frames, observers, scenario_events, score_start, score_end,
    input_frame_stamps, frame_times) = load_source(
         source_bag, input_topic, scenario, allow_empty_truth=no_target)
    time_audit = {'future_state_publications_rejected': 0}
    (track_frames, all_tracks, background, free, timing,
     diagnostics) = load_run(run_bag, algorithm, score_start, score_end,
                             frame_times, time_audit)
    if algorithm.startswith("AeroCOVER"):
        (aerocover_metrics, aerocover_candidate_rows,
         aerocover_birth_rows) = aerocover_mechanism_metrics(
             run_bag, truth_frames, score_start, score_end,
             "/aerocover_os1/diagnostics" if algorithm == "AeroCOVER-OS1"
             else "/aerocover/diagnostics", frame_times)
    else:
        aerocover_metrics, aerocover_candidate_rows, aerocover_birth_rows = \
            {}, [], []
    frames = []
    tracks_by_time = dict(track_frames)
    excluded_unobservable_truth = 0
    ignored_los_rows = []
    ignored_los_predictions = 0
    for stamp, truth in truth_frames:
        predictions = tracks_by_time.get(round(stamp, 9), [])
        observer = interpolate_truth(observers, stamp)
        scored_truth = []
        ignored_los = []
        for item in truth:
            delta = np.asarray(item["position"]) - np.asarray(observer)
            distance = float(np.linalg.norm(delta))
            elevation = abs(math.atan2(delta[2], math.hypot(delta[0], delta[1])))
            if item["present"] and distance <= 40.0 and \
                    elevation <= math.radians(22.5):
                if item['id'] in scenario.get('los_only_target_ids',[]) and not item['line_of_sight']:
                    ignored_los.append(item)
                    ignored_los_rows.append(dict(stamp=stamp,truth_id=item['id'],reason='static_LOS_occluded'))
                else:
                    scored_truth.append(item)
        ignored_truth = [item for item in truth if item["present"] and
                         item not in scored_truth and item not in ignored_los]
        excluded_unobservable_truth += len(ignored_truth)
        predictions = [prediction for prediction in predictions
                       if not any(np.linalg.norm(
                           np.asarray(prediction["position"]) -
                           np.asarray(target["position"])) <= MAIN_THRESHOLD_M
                                  for target in ignored_truth)]
        predictions, ignored_count = ignore_occluded_predictions(
            scored_truth, ignored_los, predictions, MAIN_THRESHOLD_M)
        ignored_los_predictions += ignored_count
        frames.append({"time": stamp,
                       "truth": scored_truth,
                       "predictions": predictions, "observer": observer})
    per_target_rows, tracking_timeline, multi_target_summary = \
        per_target_analysis(
            frames, all_tracks, scenario_events, scenario, algorithm)
    threshold_metrics = {str(threshold): set_metrics(frames, threshold)
                         for threshold in THRESHOLDS_M}
    hota = (hota_3d_pos(frames) if COMPUTE_HOTA else
            {"computed": False, "reason": "temporarily disabled by user", "HOTA": None,
             "per_threshold": []})
    main_track = dict(threshold_metrics[str(MAIN_THRESHOLD_M)])
    main_track["HOTA_1m_style"] = threshold_metrics["1.0"]["HOTA"]
    main_track["DetA_1m"] = threshold_metrics["1.0"]["DetA"]
    main_track["AssA_1m"] = threshold_metrics["1.0"]["AssA"]
    main_track["HOTA_main_threshold_style"] = main_track["HOTA"]
    main_track["HOTA"] = hota["HOTA"]
    if COMPUTE_HOTA:
        main_track["DetA"] = hota["DetA"]
        main_track["AssA"] = hota["AssA"]
    duration = score_end - score_start
    input_frame_count = len(input_frame_stamps)
    source_truth_coverage = min(1.0, len(truth_frames) / float(input_frame_count))
    frame_coverage = min(1.0, len(track_frames) / float(input_frame_count))
    timing_coverage = min(1.0, len(timing) / float(input_frame_count))
    completion_coverage = time_audit['completed_source_frames']/float(input_frame_count)
    resource = parse_resource_metrics(resource_file)
    cpu_time_s = resource.get("user_cpu_s", 0.0) + resource.get("system_cpu_s", 0.0)
    metrics = {
        "schema_version": 7,
        "los_scoring": {"only_targets": scenario.get('los_only_target_ids',[]),
                        "policy": "ignore occluded truth and one matching prediction; visible matches first",
                        "geometry": "sensor origin to target center, static world collision primitives at state epoch",
                        "ignored_truth_samples": len(ignored_los_rows),
                        "ignored_prediction_samples": ignored_los_predictions},
        "hota_computed": COMPUTE_HOTA,
        "main_matching_threshold_m": MAIN_THRESHOLD_M,
        "time_semantics": {
            "frame_selection": "source scan header within the declared scoring interval",
            "evaluation_epoch": "scan end from acquisition offsets; same epoch for every method",
            "state_alignment": "forward CA projection from last_prediction; no future states or nearest-frame fill",
            **time_audit,
        },
        "evaluator_sha256": file_sha256(__file__),
        "algorithm": algorithm,
        "scenario": scenario["scenario_id"],
        "seed": scenario["seed"] if seed_override is None else seed_override,
        "score_duration_s": duration,
        "track_set": main_track,
        "track_set_sensitivity": threshold_metrics,
        "HOTA_3D_pos": hota,
        "epistemic": epistemic_metrics(
            scenario["scenario_id"], all_tracks, diagnostics, duration,
            score_start, truth_frames),
        "map": map_metrics(
            background, free, truth_frames, scenario["world"]),
        "track_health": track_health_metrics(
            all_tracks, duration, no_target=no_target),
        "diagnostics": diagnostics_metrics(diagnostics),
        "initialization": initialization_metrics(algorithm, diagnostics),
        "aerocover": aerocover_metrics,
        "runtime": {
            "samples": len(timing),
            "mean_ms": finite_mean([value for _, value in timing]),
            "median_ms": percentile([value for _, value in timing], 50),
            "p95_ms": percentile([value for _, value in timing], 95),
            "p99_ms": percentile([value for _, value in timing], 99),
            "max_ms": max((value for _, value in timing), default=None),
            "peak_rss_kib": resource.get("peak_rss_kib"),
            "user_cpu_s": resource.get("user_cpu_s"),
            "system_cpu_s": resource.get("system_cpu_s"),
            "wall_elapsed_s": resource.get("wall_elapsed_s"),
            "wall_cpu_percent": resource.get("wall_cpu_percent"),
            "cpu_core_equivalent": cpu_time_s / duration if cpu_time_s else None,
            "processing_load_ratio": sum(value for _, value in timing) /
            (1000.0 * duration),
        },
        "source_input_frames": input_frame_count,
        "source_truth_frames": len(truth_frames),
        "excluded_unobservable_truth_samples": excluded_unobservable_truth,
        "track_frames": len(track_frames),
        "coverage": {
            "source_truth_ratio": source_truth_coverage,
            "track_frame_ratio": frame_coverage,
            "timing_frame_ratio": timing_coverage,
            "completion_frame_ratio": completion_coverage,
            "minimum_required_ratio": 0.95,
            "valid": source_truth_coverage >= 0.95 and
            frame_coverage >= 0.95 and timing_coverage >= 0.95 and completion_coverage >= 0.95,
        },
        "scenario_events": scenario_events,
        "ttft": explicit_ttft(
            frames, scenario_events, score_start, scenario.get("ttft_event")),
        "per_target": per_target_rows,
        "multi_target": multi_target_summary,
    }
    os.makedirs(output_dir, exist_ok=True)
    if algorithm.startswith("AeroCOVER"):
        candidate_columns = (
            "stamp", "candidate_id", "label", "nearest_truth_distance_m",
            "candidate_class", "rejection_reason", "point_count",
            "shell_coverage",
            "observable_bins", "supported_bins", "distinct_shell_scans",
            "valid_return_shell_score", "no_return_shell_score", "shell_pass",
            "birth_support_now", "birth_history")
        with open(os.path.join(output_dir, "aerocover_candidates.csv"), "w",
                  newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=candidate_columns)
            writer.writeheader()
            writer.writerows(aerocover_candidate_rows)
        birth_columns = (
            "stamp", "track_id", "candidate_id", "evidence", "matched_target",
            "nearest_truth_distance_m", "ttft_s", "birth_history",
            "shell_coverage",
            "point_count", "valid_return_score", "no_return_score",
            "no_return_fraction")
        with open(os.path.join(output_dir, "aerocover_births.csv"), "w",
                  newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=birth_columns)
            writer.writeheader()
            writer.writerows(aerocover_birth_rows)
    if COMPUTE_HOTA:
        with open(os.path.join(output_dir, "hota_per_threshold.csv"), "w",
                  newline="", encoding="utf-8") as stream:
            columns = ("alpha", "distance_boundary_m", "HOTA", "DetA", "AssA",
                       "DetRe", "DetPr", "LocA", "TP", "FP", "FN")
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            writer.writerows(hota["per_threshold"])
    with open(os.path.join(output_dir, "metrics.json"), "w", encoding="utf-8") as stream:
        json.dump(metrics, stream, indent=2, sort_keys=True)
        stream.write("\n")
    with open(os.path.join(output_dir, 'los_ignored_timeseries.csv'), 'w', newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=['stamp','truth_id','reason'])
        writer.writeheader();writer.writerows(ignored_los_rows)
    rows = []
    flatten("", metrics, rows)
    with open(os.path.join(output_dir, "metrics.csv"), "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("metric", "value"))
        writer.writerows(rows)
    with open(os.path.join(output_dir, "timing.csv"), "w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("stamp", "latency_ms"))
        writer.writerows(timing)
    with open(os.path.join(output_dir, "track_timeseries.csv"), "w", newline="",
              encoding="utf-8") as stream:
        columns = (
            "stamp", "source_stamp", "state_stamp", "id", "state", "existence", "stale_s",
            "birth_evidence_type", "birth_time", "position")
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(all_tracks)
    per_target_columns = (
        "scene", "algorithm",
        "truth_id", "birth_success", "birth_time",
        "TTFT_from_first_visible", "TTFT_from_first_eligible",
        "TTFT_from_named_event",
        "birth_provenance", "birth_semantic_label",
        "first_confirmed_track_id", "matched_recall",
        "IDSW", "fragmentation", "reacquisition_delay",
        "old_id_recovered",
        "per_target_birth_success",
        "per_target_TTFT_from_first_visible",
        "per_target_TTFT_from_first_eligible", "named_event",
        "per_target_TTFT_from_named_event", "per_target_birth_provenance",
        "dominant_track_id", "unique_track_ids_per_truth",
        "unique_track_ids", "duplicate_births_per_truth",
        "reacquired_same_id", "reacquisition_latency",
        "wrong_reactivation_count")
    with open(os.path.join(output_dir, "per_target_metrics.csv"), "w",
              newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=per_target_columns)
        writer.writeheader()
        writer.writerows(per_target_rows)
    timeline_columns = (
        "stamp", "truth_id", "visible", "truth_position",
        "matched_track_id", "match_distance_m", "track_position",
        "truth_cardinality", "active_track_cardinality")
    with open(os.path.join(output_dir, "tracking_timeline.csv"), "w",
              newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=timeline_columns)
        writer.writeheader()
        writer.writerows(tracking_timeline)

    diagnostic_keys = sorted({key for _, values in diagnostics for key in values})
    with open(os.path.join(output_dir, "diagnostics_timeseries.csv"), "w",
              newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=("stamp",) + tuple(diagnostic_keys))
        writer.writeheader()
        for stamp, values in diagnostics:
            writer.writerow({"stamp": stamp, **values})
    if not metrics["coverage"]["valid"]:
        raise RuntimeError(
            "incomplete source/replay: truth {:.1%}, track {:.1%}, timing {:.1%}, completion {:.1%} "
            "(minimum 95%)".format(
                source_truth_coverage, frame_coverage, timing_coverage, completion_coverage))
    return metrics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True)
    parser.add_argument("--run", required=True)
    parser.add_argument("--algorithm", required=True,
                        choices=(
                            "VOFOD", "AeroCOVER", "AeroCOVER-Mid360",
                            "AeroCOVER-OS1",
                            "VoFOD-Mid360", "VoFOD-OS1", "VoFOD-Mid360-Adapted", "VoFOD-Original-OS1",
                            "VoFOD-Mid360-Adapted-OS1",
                            "AeroCOVER-A1", "AeroCOVER-A2", "AeroCOVER-A3",
                            "AeroCOVER-A4", "AeroCOVER-A5"))
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--resource")
    parser.add_argument("--seed", type=int)
    arguments = parser.parse_args()
    evaluate(arguments.source, arguments.run, arguments.algorithm,
             arguments.scenario, arguments.output, arguments.resource,
             arguments.seed)


if __name__ == "__main__":
    main()
