#!/usr/bin/env python3
"""Offline one-to-one track/event/map/opportunity evaluation for rosbag runs."""

import argparse
import bisect
import csv
import hashlib
import json
import math
import os
from collections import defaultdict

import numpy as np
from scipy.optimize import linear_sum_assignment
import rosbag
import yaml
from sensor_msgs import point_cloud2


MAIN_THRESHOLD_M = 1.0
THRESHOLDS_M = (0.5, 1.0, 2.0)


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
    rows, columns = linear_sum_assignment(costs)
    return [(int(row), int(column), float(costs[row, column]))
            for row, column in zip(rows, columns)
            if costs[row, column] <= threshold]


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
        "DetA": deta, "AssA": assa, "HOTA": math.sqrt(deta * assa),
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

    def box_surface(center, size, yaw=0.0):
        half = np.asarray(size, dtype=float) / 2.0
        axes = [np.arange(-half[index], half[index] + voxel_size,
                          voxel_size) for index in range(3)]
        cosine, sine = math.cos(yaw), math.sin(yaw)
        for fixed_axis in range(3):
            moving = [axis for axis in range(3) if axis != fixed_axis]
            for fixed in (-half[fixed_axis], half[fixed_axis]):
                for first in axes[moving[0]]:
                    for second in axes[moving[1]]:
                        local = np.zeros(3)
                        local[fixed_axis] = fixed
                        local[moving[0]], local[moving[1]] = first, second
                        point = (center[0] + cosine * local[0] - sine * local[1],
                                 center[1] + sine * local[0] + cosine * local[1],
                                 center[2] + local[2])
                        output.add(quantize(point, voxel_size))

    def cylinder_surface(center, radius, height):
        count = max(24, int(math.ceil(2.0 * math.pi * radius / voxel_size)))
        for angle in np.linspace(0.0, 2.0 * math.pi, count, endpoint=False):
            for z in np.arange(0.0, height + voxel_size, voxel_size):
                output.add(quantize((
                    center[0] + radius * math.cos(angle),
                    center[1] + radius * math.sin(angle), z), voxel_size))

    for x in np.arange(-10.0, 30.0 + voxel_size, voxel_size):
        for y in np.arange(-15.0, 15.0 + voxel_size, voxel_size):
            output.add(quantize((x, y, 0.0), voxel_size))
    if world == "E1_sparse":
        for y in np.arange(-15.0, 15.0 + voxel_size, voxel_size):
            for z in np.arange(0.0, 8.0 + voxel_size, voxel_size):
                output.add(quantize((25.0, y, z), voxel_size))
    if world in ("E2_occlusion_arena", "E4_long_occlusion"):
        half_wall_y = 1.5 if world == "E4_long_occlusion" else 0.4
        for y in np.arange(-half_wall_y, half_wall_y + voxel_size, voxel_size):
            for z in np.arange(0.0, 6.0 + voxel_size, voxel_size):
                output.add(quantize((10.0, y, z), voxel_size))
        for angle in np.linspace(0.0, 2.0 * math.pi, 24, endpoint=False):
            for z in np.arange(0.0, 4.0 + voxel_size, voxel_size):
                output.add(quantize(
                    (7.0 + 0.6 * math.cos(angle), -6.0 + 0.6 * math.sin(angle), z),
                    voxel_size))
    if world == "E3_cluttered":
        box_surface((18.0, 0.0, 4.0), (0.5, 30.0, 8.0))
        box_surface((4.0, 8.0, 4.0), (0.5, 30.0, 8.0), math.pi / 2.0)
        box_surface((4.0, -8.0, 4.0), (0.5, 30.0, 8.0), math.pi / 2.0)
        for center in ((2.5, -4.0), (5.0, 0.0),
                       (8.0, 3.5), (11.0, -3.5)):
            cylinder_surface(center, 0.6, 4.0)
        for center, yaw in (((1.0, 0.0), 0.0), ((8.0, 1.0), 0.5)):
            cosine, sine = math.cos(yaw), math.sin(yaw)
            for local_y in (-2.5, 2.5):
                box_surface((center[0] - sine * local_y,
                             center[1] + cosine * local_y, 2.25),
                            (0.45, 0.45, 4.5), yaw)
            box_surface((center[0], center[1], 4.35),
                        (0.45, 5.45, 0.45), yaw)
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


def load_source(path, allow_empty_truth=False):
    truth_frames = []
    observers = []
    scenario_events = []
    input_frame_stamps = []
    with rosbag.Bag(path) as bag:
        for topic, message, _ in bag.read_messages(topics=[
                "/evaluation/visibility_ground_truth",
                "/mid360_multi_uav_sim/ground_truth/uav1/odom",
                "/mid360_multi_uav_sim/scenario_events",
                "/uav1/mid360/rays_checked"]):
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
                    "opportunity": target.true_emitted_ray_intersection_count,
                    "unblocked_opportunity": target.true_unblocked_ray_count,
                    "visibility": target.true_visibility_fraction,
                    "occlusion": target.true_occlusion_type,
                } for target in message.targets]))
            elif topic.endswith("uav1/odom"):
                position = message.pose.pose.position
                observers.append((message.header.stamp.to_sec(),
                                  (position.x, position.y, position.z)))
            elif topic == "/mid360_multi_uav_sim/scenario_events":
                try:
                    scenario_events.append(json.loads(message.data))
                except (TypeError, ValueError):
                    pass
            else:
                input_frame_stamps.append(message.header.stamp.to_sec())
    truth_frames.sort()
    observers.sort()
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
    if not truth_frames and not allow_empty_truth:
        raise RuntimeError("source bag contains no scored visibility truth")
    if not input_frame_stamps:
        raise RuntimeError("source bag contains no scored checked-ray input")
    if not truth_frames:
        truth_frames = [(stamp, []) for stamp in input_frame_stamps]
    return (truth_frames, observers, scenario_events, score_start, score_end,
            input_frame_stamps)


def load_run(path, algorithm, score_start, score_end):
    soft = algorithm != "B0"
    track_topic = "/soft_vofod/tracks" if soft else "/uav1/batch/b0/tracks"
    event_topic = "/soft_vofod/events"
    maintenance_topic = "/soft_vofod/maintenance_packets"
    memory_topic = "/soft_vofod/track_memory"
    opportunity_topic = "/soft_vofod/opportunity_debug"
    background_topic = "/soft_vofod/background_voxels" if soft \
        else "/uav1/vofod_mid360/background_points"
    candidate_topic = "/soft_vofod/candidate_background_voxels"
    free_topic = "/soft_vofod/free_voxels" if soft \
        else "/uav1/vofod_mid360/free_voxels"
    observed_free_topic = "/soft_vofod/observed_free_voxels"
    diagnostics_topic = "/soft_vofod/diagnostics" if soft \
        else "/uav1/vofod_mid360/map_update_diagnostics"
    frames_by_stamp = {}
    all_tracks = []
    events = []
    maintenance_packets = []
    opportunities = []
    background = []
    candidate_background = []
    free = []
    observed_free = []
    timing = []
    diagnostics = []
    with rosbag.Bag(path) as bag:
        for topic, message, _ in bag.read_messages(topics=[
                track_topic, memory_topic, event_topic, maintenance_topic,
                opportunity_topic, background_topic, candidate_topic,
                free_topic, observed_free_topic, diagnostics_topic]):
            stamp = message.header.stamp.to_sec() if hasattr(message, "header") else 0.0
            if topic == track_topic and score_start <= stamp <= score_end:
                predictions = []
                for track in message.tracks:
                    confirmed = (track.state in (
                        track.CONFIRMED, getattr(track, "OCCLUDED", -1))) if soft \
                        else track.n_detections >= 2
                    if soft:
                        state = "active" if track.state == track.CONFIRMED else \
                            "occluded" if track.state == getattr(track, "OCCLUDED", -1) \
                            else "dormant" if track.state == getattr(track, "DORMANT", -1) \
                            else "tentative" if track.state == track.TENTATIVE \
                            else "deleting"
                    if not confirmed:
                        continue
                    predictions.append({
                        "id": int(track.track_id if soft else track.id),
                        "position": (track.position.x, track.position.y, track.position.z),
                        "velocity": (track.velocity.x, track.velocity.y, track.velocity.z),
                    })
                frames_by_stamp[round(stamp, 9)] = predictions
            elif soft and topic == memory_topic and score_start <= stamp <= score_end:
                for track in message.tracks:
                    state = "active" if track.state == track.CONFIRMED else \
                        "occluded" if track.state == getattr(track, "OCCLUDED", -1) \
                        else "dormant" if track.state == getattr(track, "DORMANT", -1) \
                        else "tentative" if track.state == track.TENTATIVE \
                        else "deleting"
                    all_tracks.append({
                        "stamp": stamp,
                        "id": int(track.track_id),
                        "state": state,
                        "existence": float(track.existence_probability),
                        "reportability": float(getattr(
                            track, "reportability_score", 1.0)),
                        "reportable": bool(getattr(track, "reportable", True)),
                        "stale_s": float(track.time_since_last_measurement),
                        "birth_evidence_type": int(getattr(
                            track, "birth_evidence_type", 0)),
                        "last_evidence_type": int(getattr(
                            track, "last_evidence_type", 0)),
                        "reactivation_count": int(getattr(
                            track, "reactivation_count", 0)),
                        "position": (track.position.x, track.position.y,
                                     track.position.z),
                    })
            elif soft and topic in (event_topic, maintenance_topic) and \
                    score_start <= stamp <= score_end:
                packets = [{
                    "position": (item.position.x, item.position.y, item.position.z),
                    "points": [(point.x, point.y, point.z)
                               for point in getattr(item, "points", [])],
                    "point_count": int(getattr(item, "point_count", 1)),
                    "birth_evidence_type": int(getattr(
                        item, "birth_evidence_type", 0)),
                } for item in message.events]
                (events if topic == event_topic else maintenance_packets).append(
                    (stamp, packets))
            elif soft and topic == opportunity_topic and score_start <= stamp <= score_end:
                opportunities.append((stamp, [{
                    "track_id": int(track_id), "pd": float(pd),
                    "effective": float(effective), "matched": bool(matched),
                    "occlusion": float(occlusion)}
                    for track_id, pd, effective, matched, occlusion in zip(
                        message.track_ids, message.detection_probabilities,
                        message.effective_opportunities, message.matched,
                        getattr(message, "occlusion_probabilities",
                                [0.0] * len(message.track_ids)))]))
            elif topic == background_topic and score_start <= stamp <= score_end:
                background.append((stamp, read_cloud_keys(message)))
            elif soft and topic == candidate_topic and score_start <= stamp <= score_end:
                candidate_background.append((stamp, read_cloud_keys(message)))
            elif topic == free_topic and score_start <= stamp <= score_end:
                free.append((stamp, read_cloud_keys(message)))
            elif soft and topic == observed_free_topic and \
                    score_start <= stamp <= score_end:
                observed_free.append((stamp, read_cloud_keys(message)))
            elif topic == diagnostics_topic and score_start <= stamp <= score_end:
                if soft:
                    values = {item.key: item.value for status in message.status
                              for item in status.values}
                    if "processing_ms" in values:
                        timing.append((stamp, float(values["processing_ms"])))
                        numeric = {}
                        for key, value in values.items():
                            try:
                                numeric[key] = float(value)
                            except ValueError:
                                continue
                        diagnostics.append((stamp, numeric))
                else:
                    timing.append((stamp, float(message.total_ms)))
    frames = sorted(frames_by_stamp.items())
    return (frames, all_tracks, events, maintenance_packets, opportunities,
            background, candidate_background, free, observed_free, timing,
            diagnostics)


def event_metrics(events, truth_frames, duration):
    true_events = false_events = 0
    truth_with_return = truth_with_event = 0
    raw_endpoints = 0
    singleton_packets = 0
    false_by_voxel = defaultdict(list)
    for stamp, packets in events:
        targets = nearest(truth_frames, stamp) or []
        for packet in packets:
            point = packet["position"]
            raw_endpoints += packet["point_count"]
            singleton_packets += packet["point_count"] == 1
            if targets and min(np.linalg.norm(np.asarray(point) -
                                              np.asarray(target["position"]))
                               for target in targets) <= MAIN_THRESHOLD_M:
                true_events += 1
            else:
                false_events += 1
                false_by_voxel[quantize(point)].append(stamp)
    for stamp, targets in truth_frames:
        expected = [target for target in targets if target["actual_returns"] > 0]
        if not expected:
            continue
        truth_with_return += len(expected)
        packets = nearest(events, stamp) or []
        truth_with_event += sum(any(
            np.linalg.norm(np.asarray(packet["position"]) -
                           np.asarray(target["position"])) <= MAIN_THRESHOLD_M
            for packet in packets)
                                for target in expected)
    packet_count = true_events + false_events
    persistence = [max(stamps) - min(stamps) for stamps in false_by_voxel.values()]
    return {
        "precision": true_events / float(true_events + false_events)
        if true_events + false_events else 0.0,
        "recall": truth_with_event / float(truth_with_return)
        if truth_with_return else 0.0,
        "false_events_per_min": false_events / max(duration / 60.0, 1.0e-9),
        "event_count": packet_count,
        "raw_anomaly_points_in_packets": raw_endpoints,
        "packet_singleton_ratio": singleton_packets / float(packet_count)
        if packet_count else 0.0,
        "packets_per_true_target_frame": true_events / float(truth_with_return)
        if truth_with_return else None,
        "packet_purity": true_events / float(packet_count) if packet_count else 0.0,
        "false_packet_persistence_mean_s": finite_mean(persistence),
        "false_packet_persistence_max_s": max(persistence, default=0.0),
    }


def packet_continuity_metrics(packets, truth_frames, track_frames):
    packet_count = multi_truth = target_frames = shortage_frames = 0
    targets_without_packet = gate_rejections = 0
    purities = []
    innovations = []
    accelerations = []
    turning_segments = 0
    previous_velocity = {}
    previous_time = {}
    previously_tracked = set()
    for stamp, targets in truth_frames:
        visible = [target for target in targets
                   if target.get("present", True) and
                   target.get("line_of_sight", True) and
                   target.get("actual_returns", 0) > 0]
        frame_packets = nearest(packets, stamp) or []
        predictions = nearest(track_frames, stamp) or []
        packet_count += len(frame_packets)
        if visible:
            target_frames += 1
            shortage_frames += len(frame_packets) < len(visible)
        packet_truth = []
        for packet in frame_packets:
            points = packet["points"] or [packet["position"]]
            memberships = set()
            labels = []
            for point in points:
                distances = [np.linalg.norm(np.asarray(point) -
                                            np.asarray(target["position"]))
                             for target in visible]
                if distances and min(distances) <= MAIN_THRESHOLD_M:
                    label = int(np.argmin(distances))
                    memberships.add(label)
                    labels.append(label)
                else:
                    labels.append(-1)
            packet_truth.append(memberships)
            multi_truth += len(memberships) >= 2
            if labels:
                counts = [labels.count(label) for label in set(labels)
                          if label >= 0]
                purities.append(max(counts, default=0) / float(len(labels)))
        current_tracked = set()
        for target_index, target in enumerate(visible):
            has_packet = any(target_index in membership
                             for membership in packet_truth)
            targets_without_packet += not has_packet
            distances = [np.linalg.norm(np.asarray(prediction["position"]) -
                                        np.asarray(target["position"]))
                         for prediction in predictions]
            has_prediction = bool(distances and min(distances) <= MAIN_THRESHOLD_M)
            if has_prediction:
                current_tracked.add(target["id"])
                innovations.append(min(distances))
            if target["id"] in previously_tracked and has_packet and \
                    not has_prediction:
                gate_rejections += 1
            velocity = np.asarray(target["velocity"])
            if target["id"] in previous_velocity:
                dt = stamp - previous_time[target["id"]]
                if dt > 1.0e-6:
                    delta_velocity = velocity - previous_velocity[target["id"]]
                    accelerations.append(float(np.linalg.norm(delta_velocity) / dt))
                    first_norm = np.linalg.norm(previous_velocity[target["id"]])
                    second_norm = np.linalg.norm(velocity)
                    if first_norm > 0.1 and second_norm > 0.1:
                        cosine = float(np.dot(previous_velocity[target["id"]], velocity) /
                                       (first_norm * second_norm))
                        turning_segments += math.acos(max(-1.0, min(1.0, cosine))) > \
                            math.radians(15.0)
            previous_velocity[target["id"]] = velocity
            previous_time[target["id"]] = stamp
        previously_tracked = current_tracked
    return {
        "packet_count": packet_count,
        "packet_shortage_ratio": shortage_frames / float(target_frames)
        if target_frames else None,
        "multi_truth_packet_count": multi_truth,
        "multi_truth_packet_ratio": multi_truth / float(packet_count)
        if packet_count else 0.0,
        "packet_purity": finite_mean(purities),
        "target_without_packet_count": targets_without_packet,
        "true_packet_gate_rejection": gate_rejections,
        "track_unmatched_despite_truth_packet": gate_rejections,
        "target_acceleration_mps2": distribution(accelerations),
        "turning_segment_count": turning_segments,
        "predicted_truth_innovation_m": distribution(innovations),
    }


def epistemic_metrics(scenario_id, tracks, diagnostics, duration):
    confirmed_unknown_ids = {item["id"] for item in tracks
                             if item["state"] in ("active", "occluded") and
                             item["birth_evidence_type"] == 2}
    confirmed_certified_ids = {item["id"] for item in tracks
                               if item["state"] in ("active", "occluded") and
                               item["birth_evidence_type"] == 1}
    diagnostic_samples = [values for _, values in diagnostics]
    unresolved_samples = sum(values.get("unknown_candidates", 0.0) > 0.0
                             for values in diagnostic_samples)
    candidate_returns = sum(values.get("unresolved_candidate_returns", 0.0)
                            for values in diagnostic_samples)
    valid_returns = sum(values.get("valid_returns", 0.0)
                        for values in diagnostic_samples)
    stationary = "S08C" in scenario_id or "unknown_stationary" in scenario_id
    moving = "S08B" in scenario_id or "unknown_moving" in scenario_id
    hover = "hover" in scenario_id.lower()
    return {
        "false_unknown_static_confirmation": len(confirmed_unknown_ids)
        if stationary else None,
        "false_target_confirmation_rate":
            min(1.0, len(confirmed_unknown_ids)) if stationary else None,
        "unknown_unresolved_duration_s": duration * unresolved_samples /
        float(len(diagnostic_samples)) if diagnostic_samples else None,
        "unresolved_fraction": unresolved_samples /
        float(len(diagnostic_samples)) if diagnostic_samples else None,
        "background_candidate_fraction": candidate_returns /
        float(valid_returns) if valid_returns else None,
        "unknown_moving_birth_recall":
            float(bool(confirmed_unknown_ids)) if moving else None,
        "mapped_free_hover_birth_recall":
            float(bool(confirmed_certified_ids)) if hover else None,
    }


def lifecycle_metrics(tracks, truth_frames, scenario_events):
    by_id = defaultdict(list)
    for item in tracks:
        by_id[item["id"]].append(item)
    occluded_duration = dormant_duration = 0.0
    reactivations = []
    for samples in by_id.values():
        samples.sort(key=lambda item: item["stamp"])
        previous_reactivations = 0
        for first, second in zip(samples, samples[1:]):
            dt = max(0.0, second["stamp"] - first["stamp"])
            occluded_duration += dt if first["state"] == "occluded" else 0.0
            dormant_duration += dt if first["state"] == "dormant" else 0.0
            if second["reactivation_count"] > previous_reactivations:
                reactivations.append(second)
            previous_reactivations = second["reactivation_count"]
    correct = 0
    for item in reactivations:
        targets = nearest(truth_frames, item["stamp"]) or []
        correct += bool(targets and min(
            np.linalg.norm(np.asarray(item["position"]) -
                           np.asarray(target["position"]))
            for target in targets) <= MAIN_THRESHOLD_M)
    end_times = [item.get("sim_time") for item in scenario_events
                 if str(item.get("event", item.get("id", ""))).endswith("_end")]
    latencies = []
    for end_time in end_times:
        if end_time is None:
            continue
        later = [item["stamp"] - end_time for item in reactivations
                 if item["stamp"] >= end_time]
        if later:
            latencies.append(min(later))
    return {
        "occlusion_detected_duration_s": occluded_duration,
        "dormant_duration_s": dormant_duration,
        "reactivation_latency_s": distribution(latencies),
        "correct_reactivation_rate": correct / float(len(reactivations))
        if reactivations else None,
        "wrong_reactivation_rate": (len(reactivations) - correct) /
        float(len(reactivations)) if reactivations else None,
        "reactivation_count": len(reactivations),
    }


def opportunity_metrics(opportunities, detector_stamps=None):
    detector_stamps = ({round(stamp, 9) for stamp in detector_stamps}
                       if detector_stamps is not None else None)
    samples = [item for stamp, frame in opportunities for item in frame
               if detector_stamps is None or round(stamp, 9) in detector_stamps]
    if not samples:
        return {}
    brier = np.mean([(item["pd"] - float(item["matched"])) ** 2 for item in samples])
    nll = np.mean([-math.log(max(1.0e-9, item["pd"] if item["matched"]
                                      else 1.0 - item["pd"])) for item in samples])
    bins = {"0": [], "0-1": [], "1-3": [], "3-10": [], "gt10": []}
    pd_bins = {"0-0.25": [], "0.25-0.5": [], "0.5-0.75": [], "0.75-1": []}
    for item in samples:
        effective = item["effective"]
        key = "0" if effective == 0.0 else "0-1" if effective <= 1.0 \
            else "1-3" if effective <= 3.0 else "3-10" if effective <= 10.0 \
            else "gt10"
        bins[key].append(float(item["matched"]))
        pd = item["pd"]
        key = "0-0.25" if pd < 0.25 else "0.25-0.5" if pd < 0.5 \
            else "0.5-0.75" if pd < 0.75 else "0.75-1"
        pd_bins[key].append(float(item["matched"]))
    return {
        "Brier": float(brier), "NLL": float(nll),
        "recall_by_effective_opportunity": {
            key: finite_mean(values) for key, values in bins.items()},
        "miss_rate_by_PD": {
            key: 1.0 - finite_mean(values) if values else None
            for key, values in pd_bins.items()},
        "no_opportunity_samples": len(bins["0"]),
        "high_opportunity_misses": sum(
            1 for item in samples if item["effective"] > 3.0 and not item["matched"]),
    }


def return_probability_metrics(frames):
    bins = {key: {"frames": 0, "unblocked_intersections": 0,
                  "actual_returns": 0}
            for key in ("0-10", "10-20", "20-30", "30+")}
    for frame in frames:
        observer = np.asarray(frame["observer"])
        for target in frame["truth"]:
            target_range = float(np.linalg.norm(
                np.asarray(target["position"]) - observer))
            key = "0-10" if target_range < 10.0 else \
                "10-20" if target_range < 20.0 else \
                "20-30" if target_range < 30.0 else "30+"
            item = bins[key]
            item["frames"] += 1
            item["unblocked_intersections"] += int(
                target.get("unblocked_opportunity", 0))
            item["actual_returns"] += int(target.get("actual_returns", 0))
    for item in bins.values():
        denominator = item["unblocked_intersections"]
        item["p_ret"] = item["actual_returns"] / float(denominator) \
            if denominator else None
    return bins


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
    confirmed = [item for item in tracks
                 if item["state"] in ("active", "occluded") and
                 item.get("reportable", True)]
    tentative_ids = {item["id"] for item in tracks
                     if item["state"] == "tentative"}
    confirmed_ids = {item["id"] for item in confirmed}
    by_stamp = defaultdict(list)
    for item in tracks:
        by_stamp[item["stamp"]].append(item)
    confirmed_counts = [sum(item["state"] in ("active", "occluded") and
                            item.get("reportable", True) for item in frame)
                        for frame in by_stamp.values()]
    stale = {str(threshold): sum(item["stale_s"] > threshold
                                 for item in confirmed)
             for threshold in (0.5, 1.0, 3.0)}
    stale_tracks = {str(threshold): len({item["id"] for item in confirmed
                                        if item["stale_s"] > threshold})
                    for threshold in (0.5, 1.0, 3.0)}
    stale_existence = [item["existence"] for item in confirmed
                       if item["stale_s"] > 0.5]
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
        "mean_stale_existence": finite_mean(stale_existence),
    }


def diagnostics_metrics(diagnostics):
    keys = {key for _, values in diagnostics for key in values}
    summaries = {
        key: distribution([values[key] for _, values in diagnostics
                           if key in values])
        for key in sorted(keys)
    }
    runtime_keys = ("processing_ms", "classification_ms", "tracking_ms",
                    "map_commit_ms", "packet_split_ms", "imm_ms",
                    "hungarian_ms", "dormant_reacquisition_ms")
    count_keys = (
        "input_rays", "raw_anomaly_endpoints", "violation_packets",
        "maintenance_packets", "births", "matches", "track_count",
        "support_count", "tentative_weak_support_count",
        "unknown_candidates", "map_epoch_free_voxels",
        "map_epoch_background_voxels", "opportunity_full_scan_rays",
        "opportunity_candidate_rays")
    count_keys += (
        "observed_free_voxels", "certified_free_voxels",
        "certified_free_violation_packets", "unknown_motion_packets",
        "unknown_motion_rejections", "track_conditioned_split_count",
        "track_conditioned_split_packets", "split_points_assigned",
        "split_points_unassigned", "association_gate_rejections",
        "occlusion_association_rejections",
        "occluded_transitions", "dormant_entries", "dormant_reactivations",
        "dormant_expirations", "dormant_reacquisition_rejections")
    return {
        "module_runtime_ms": {key: summaries[key] for key in runtime_keys
                              if key in summaries},
        "complexity": {key: summaries[key] for key in count_keys
                       if key in summaries},
    }


def map_metrics(background, candidate_background, free, observed_free,
                truth_frames, world):
    path, last_time = target_path_voxels(truth_frames)
    background_union = set().union(*(keys for _, keys in background)) if background else set()
    final_background = background[-1][1] if background else set()
    final_free = free[-1][1] if free else set()
    final_observed_free = observed_free[-1][1] if observed_free else final_free
    contamination = background_union & path
    trail = 0.0
    for stamp, keys in background:
        for key in keys & path:
            trail = max(trail, stamp - last_time.get(key, stamp))
    static = static_voxels(world)
    candidate_first = {}
    for stamp, keys in candidate_background:
        for key in keys & static:
            candidate_first.setdefault(key, stamp)
    stable_first = {}
    for stamp, keys in background:
        for key in keys & static:
            stable_first.setdefault(key, stamp)
    assimilation_latency = [stable_first[key] - stamp
                            for key, stamp in candidate_first.items()
                            if key in stable_first and stable_first[key] >= stamp]
    observed_static = static & (set(candidate_first) | set(stable_first))
    observed_free_first = {}
    for stamp, keys in observed_free:
        for key in keys:
            observed_free_first.setdefault(key, stamp)
    certified_free_first = {}
    for stamp, keys in free:
        for key in keys:
            certified_free_first.setdefault(key, stamp)
    certification_latency = [certified_free_first[key] - stamp
                             for key, stamp in observed_free_first.items()
                             if key in certified_free_first and
                             certified_free_first[key] >= stamp]
    false_certified = final_free & static
    return {
        "target_contamination_ratio": len(contamination) / float(len(path))
        if path else None,
        "trail_duration_max_s": max(0.0, trail),
        "free_space_retention": 1.0 - len(contamination) / float(len(path))
        if path else None,
        "static_background_recall": len(final_background & static) / float(len(static))
        if static else None,
        "false_free_rate": len(final_free & static) / float(len(static))
        if static else None,
        "certified_free_precision": 1.0 - len(false_certified) /
        float(len(final_free)) if final_free else None,
        "certified_free_recall": len(final_free & final_observed_free) /
        float(len(final_observed_free)) if final_observed_free else None,
        "observed_to_certified_latency_s": distribution(
            certification_latency),
        "false_certified_free": len(false_certified),
        "final_observed_free_voxels": len(final_observed_free),
        "final_background_voxels": len(final_background),
        "final_free_voxels": len(final_free),
        "candidate_to_stable_latency_s": distribution(assimilation_latency),
        "candidate_static_voxels": len(candidate_first),
        "candidate_promoted_static_voxels": len(
            set(candidate_first) & set(stable_first)),
        "background_expansion_recall":
            len(final_background & observed_static) / float(len(observed_static))
            if observed_static else None,
    }


def evaluate(source_bag, run_bag, algorithm, scenario_file, output_dir,
             resource_file=None):
    with open(scenario_file, encoding="utf-8") as stream:
        scenario = yaml.safe_load(stream)
    no_target = not scenario.get("targets")
    (truth_frames, observers, scenario_events, score_start, score_end,
     input_frame_stamps) = load_source(source_bag, allow_empty_truth=no_target)
    (track_frames, all_tracks, events, maintenance_packets, opportunities,
     background, candidate_background, free, observed_free, timing,
     diagnostics) = load_run(run_bag, algorithm, score_start, score_end)
    frames = []
    for stamp, truth in truth_frames:
        predictions = nearest(track_frames, stamp) or []
        observer = nearest(observers, stamp) or (0.0, 0.0, 0.0)
        frames.append({"time": stamp,
                       "truth": [item for item in truth if item["present"]],
                       "predictions": predictions, "observer": observer})
    threshold_metrics = {str(threshold): set_metrics(frames, threshold)
                         for threshold in THRESHOLDS_M}
    duration = score_end - score_start
    input_frame_count = len(input_frame_stamps)
    source_truth_coverage = min(1.0, len(truth_frames) / float(input_frame_count))
    frame_coverage = min(1.0, len(track_frames) / float(input_frame_count))
    timing_coverage = min(1.0, len(timing) / float(input_frame_count))
    resource = parse_resource_metrics(resource_file)
    cpu_time_s = resource.get("user_cpu_s", 0.0) + resource.get("system_cpu_s", 0.0)
    metrics = {
        "schema_version": 2,
        "evaluator_sha256": file_sha256(__file__),
        "algorithm": algorithm,
        "scenario": scenario["scenario_id"],
        "seed": scenario["seed"],
        "score_duration_s": duration,
        "track_set": threshold_metrics[str(MAIN_THRESHOLD_M)],
        "track_set_sensitivity": threshold_metrics,
        "event": event_metrics(events, truth_frames, duration) if algorithm != "B0" else {},
        "packet_continuity": packet_continuity_metrics(
            maintenance_packets, truth_frames, track_frames)
        if algorithm != "B0" else {},
        "epistemic": epistemic_metrics(
            scenario["scenario_id"], all_tracks, diagnostics, duration)
        if algorithm != "B0" else {},
        "lifecycle": lifecycle_metrics(
            all_tracks, truth_frames, scenario_events)
        if algorithm != "B0" else {},
        "opportunity": opportunity_metrics(
            opportunities,
            [stamp for stamp, values in diagnostics
             if values.get("map_epochs_committed", 0.0) > 0.0]
            if algorithm != "B0" else None),
        "sensor_return_probability": return_probability_metrics(frames),
        "map": map_metrics(
            background, candidate_background, free, observed_free,
            truth_frames,
            scenario["world"]),
        "track_health": track_health_metrics(
            all_tracks, duration, no_target=no_target),
        "diagnostics": diagnostics_metrics(diagnostics),
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
        "track_frames": len(track_frames),
        "coverage": {
            "source_truth_ratio": source_truth_coverage,
            "track_frame_ratio": frame_coverage,
            "timing_frame_ratio": timing_coverage,
            "minimum_required_ratio": 0.95,
            "valid": source_truth_coverage >= 0.95 and
            frame_coverage >= 0.95 and timing_coverage >= 0.95,
        },
        "scenario_events": scenario_events,
    }
    os.makedirs(output_dir, exist_ok=True)
    with open(os.path.join(output_dir, "metrics.json"), "w", encoding="utf-8") as stream:
        json.dump(metrics, stream, indent=2, sort_keys=True)
        stream.write("\n")
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
        columns = ("stamp", "id", "state", "existence", "reportability",
                   "reportable", "stale_s", "birth_evidence_type",
                   "last_evidence_type", "reactivation_count", "position")
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(all_tracks)
    with open(os.path.join(output_dir, "opportunity_timeseries.csv"), "w",
              newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=("stamp", "track_id", "pd", "effective",
                                "matched", "occlusion"))
        writer.writeheader()
        for stamp, items in opportunities:
            for item in items:
                writer.writerow({"stamp": stamp, **item})
    diagnostic_keys = sorted({key for _, values in diagnostics for key in values})
    with open(os.path.join(output_dir, "diagnostics_timeseries.csv"), "w",
              newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=("stamp",) + tuple(diagnostic_keys))
        writer.writeheader()
        for stamp, values in diagnostics:
            writer.writerow({"stamp": stamp, **values})
    if not metrics["coverage"]["valid"]:
        raise RuntimeError(
            "incomplete source/replay: truth {:.1%}, track {:.1%}, timing {:.1%} "
            "(minimum 95%)".format(
                source_truth_coverage, frame_coverage, timing_coverage))
    return metrics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True)
    parser.add_argument("--run", required=True)
    parser.add_argument("--algorithm", required=True,
                        choices=("B0", "B1", "B2", "B3", "B4",
                                 "A1", "A2", "A3", "V3-A", "V3-B",
                                 "V3-C", "C0", "C1", "C2", "C3",
                                 "S04-base", "S04-split", "S04-IMM",
                                 "S04-split-IMM"))
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--resource")
    arguments = parser.parse_args()
    evaluate(arguments.source, arguments.run, arguments.algorithm,
             arguments.scenario, arguments.output, arguments.resource)


if __name__ == "__main__":
    main()
