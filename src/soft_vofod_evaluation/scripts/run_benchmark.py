#!/usr/bin/env python3
"""Record immutable Gazebo sources, replay fixed variants, and evaluate."""

import argparse
import contextlib
import csv
import datetime
import hashlib
import json
import math
import os
import random
import shutil
import signal
import statistics
import subprocess
import sys
import time
from urllib.parse import urlparse

import rosgraph
import rosbag
import rospkg
from roslib.packages import find_node
import yaml


ALGORITHMS = (
    "B0", "B1", "B2", "B3", "B4", "A1", "A2", "A3",
    "V3-A", "V3-B", "V3-C", "C0", "C1", "C2", "C3",
    "S04-base", "S04-split", "S04-IMM", "S04-split-IMM")
DEFAULT_SCENES = tuple("S{:02d}".format(index) for index in range(1, 8))
SCENES = DEFAULT_SCENES + (
    "S08A", "S08B", "S08C", "NEG01", "NEG02", "NEG03", "NEG04",
    "NEG05", "NEG06", "CAL01", "CAL02", "CAL03", "CAL04", "CAL05",
    "CAL06", "CAL07", "CAL08", "CAL09", "IT11")
WORLD_FILES = {
    "E0_open": "E0_open.world",
    "E1_sparse": "E1_sparse.world",
    "E2_occlusion_arena": "E2_occlusion_arena.world",
    "E3_cluttered": "E3_cluttered.world",
    "E4_long_occlusion": "E4_long_occlusion.world",
}


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def combined_sha256(paths):
    digest = hashlib.sha256()
    for path in sorted(paths):
        digest.update(os.path.basename(path).encode("utf-8"))
        digest.update(sha256(path).encode("ascii"))
    return digest.hexdigest()


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def write_json(path, value):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True)
        stream.write("\n")


SUMMARY_METRICS = {
    "HOTA": ("track_set", "HOTA"),
    "DetA": ("track_set", "DetA"),
    "AssA": ("track_set", "AssA"),
    "IDF1": ("track_set", "IDF1"),
    "id_switches": ("track_set", "id_switches"),
    "fragmentations": ("track_set", "fragmentations"),
    "GOSPA_mean": ("track_set", "GOSPA_mean"),
    "position_RMSE_m": ("track_set", "position_RMSE_m"),
    "velocity_RMSE_mps": ("track_set", "velocity_RMSE_mps"),
    "TTFT_mean_s": ("track_set", "TTFT_mean_s"),
    "track_completeness_mean": ("track_set", "track_completeness_mean"),
    "event_recall": ("event", "recall"),
    "event_false_per_min": ("event", "false_events_per_min"),
    "packet_singleton_ratio": ("event", "packet_singleton_ratio"),
    "packet_purity": ("event", "packet_purity"),
    "packet_shortage_ratio": ("packet_continuity", "packet_shortage_ratio"),
    "multi_truth_packet_ratio": (
        "packet_continuity", "multi_truth_packet_ratio"),
    "opportunity_Brier": ("opportunity", "Brier"),
    "target_contamination_ratio": ("map", "target_contamination_ratio"),
    "background_expansion_recall": ("map", "background_expansion_recall"),
    "certified_free_precision": ("map", "certified_free_precision"),
    "certified_free_recall": ("map", "certified_free_recall"),
    "false_unknown_static_confirmation": (
        "epistemic", "false_unknown_static_confirmation"),
    "unknown_moving_birth_recall": (
        "epistemic", "unknown_moving_birth_recall"),
    "mapped_free_hover_birth_recall": (
        "epistemic", "mapped_free_hover_birth_recall"),
    "correct_reactivation_rate": ("lifecycle", "correct_reactivation_rate"),
    "wrong_reactivation_rate": ("lifecycle", "wrong_reactivation_rate"),
    "false_confirmed_tracks_per_min": (
        "track_health", "false_confirmed_tracks_per_min"),
    "max_stale_age_s": ("track_health", "max_stale_age_s"),
    "runtime_p95_ms": ("runtime", "p95_ms"),
    "peak_rss_kib": ("runtime", "peak_rss_kib"),
    "cpu_core_equivalent": ("runtime", "cpu_core_equivalent"),
    "processing_load_ratio": ("runtime", "processing_load_ratio"),
    "source_truth_coverage": ("coverage", "source_truth_ratio"),
    "frame_coverage": ("coverage", "track_frame_ratio"),
}


def nested_value(value, path):
    for key in path:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value if isinstance(value, (int, float)) and not isinstance(value, bool) \
        and math.isfinite(value) else None


def quantile(values, probability):
    ordered = sorted(values)
    if not ordered:
        return None
    location = probability * (len(ordered) - 1)
    lower = int(math.floor(location))
    upper = int(math.ceil(location))
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (location - lower)


def aggregate_results(output_root):
    """Write run-level, grouped, and paired-ablation summaries."""
    rows = []
    runs_root = os.path.join(output_root, "runs")
    if os.path.isdir(runs_root):
        for directory, _, files in os.walk(runs_root):
            if "metrics.json" not in files:
                continue
            relative = os.path.relpath(directory, runs_root).split(os.sep)
            if len(relative) != 4 or not relative[3].startswith("seed_"):
                continue
            manifest_path = os.path.join(directory, "run_manifest.json")
            if os.path.exists(manifest_path):
                with open(manifest_path, encoding="utf-8") as stream:
                    if json.load(stream).get("status", "ok") != "ok":
                        continue
            algorithm, scene, noise, seed_name = relative
            with open(os.path.join(directory, "metrics.json"), encoding="utf-8") as stream:
                metrics = json.load(stream)
            row = {
                "algorithm": algorithm, "scene": scene, "noise": noise,
                "seed": int(seed_name[5:]), "scenario_id": metrics.get("scenario", ""),
            }
            row.update({name: nested_value(metrics, path)
                        for name, path in SUMMARY_METRICS.items()})
            rows.append(row)
    rows.sort(key=lambda row: (row["scene"], row["noise"], row["seed"],
                               row["algorithm"]))
    metrics_dir = os.path.join(output_root, "metrics")
    os.makedirs(metrics_dir, exist_ok=True)
    columns = ("algorithm", "scene", "scenario_id", "noise", "seed") + \
        tuple(SUMMARY_METRICS)
    with open(os.path.join(metrics_dir, "summary.csv"), "w", newline="",
              encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)

    grouped = {}
    random_generator = random.Random(0)
    for algorithm, scene, noise in sorted({
            (row["algorithm"], row["scene"], row["noise"]) for row in rows}):
        group_rows = [row for row in rows if
                      (row["algorithm"], row["scene"], row["noise"]) ==
                      (algorithm, scene, noise)]
        key = "/".join((algorithm, scene, noise))
        grouped[key] = {}
        for metric in SUMMARY_METRICS:
            values = [row[metric] for row in group_rows if row[metric] is not None]
            if not values:
                continue
            if len(values) == 1:
                bootstrap_means = values
            else:
                bootstrap_means = [statistics.mean(
                    random_generator.choice(values) for _ in values)
                    for _ in range(2000)]
            grouped[key][metric] = {
                "n": len(values), "mean": statistics.mean(values),
                "std": statistics.stdev(values) if len(values) > 1 else 0.0,
                "median": statistics.median(values),
                "q25": quantile(values, 0.25), "q75": quantile(values, 0.75),
                "bootstrap_mean_ci95": [quantile(bootstrap_means, 0.025),
                                         quantile(bootstrap_means, 0.975)],
            }
    write_json(os.path.join(metrics_dir, "aggregate.json"), {
        "schema_version": 1, "run_count": len(rows), "groups": grouped})

    paired = {(row["scene"], row["noise"], row["seed"], row["algorithm"]): row
              for row in rows}
    delta_columns = ("scene", "noise", "seed", "proposed_algorithm",
                     "comparator", "metric", "proposed_value",
                     "comparator_value", "proposed_minus_comparator")
    delta_rows = []
    for scene, noise, seed, algorithm in sorted(paired):
        if algorithm not in ("A3", "B4"):
            continue
        proposed = paired[(scene, noise, seed, algorithm)]
        comparators = ("B0", "A1", "A2") if algorithm == "A3" \
            else ("B0", "B1", "B2", "B3")
        for comparator in comparators:
            baseline = paired.get((scene, noise, seed, comparator))
            if baseline is None:
                continue
            for metric in SUMMARY_METRICS:
                if proposed[metric] is None or baseline[metric] is None:
                    continue
                delta_rows.append({
                    "scene": scene, "noise": noise, "seed": seed,
                    "proposed_algorithm": algorithm,
                    "comparator": comparator, "metric": metric,
                    "proposed_value": proposed[metric],
                    "comparator_value": baseline[metric],
                    "proposed_minus_comparator":
                    proposed[metric] - baseline[metric],
                })
    with open(os.path.join(metrics_dir, "ablation_deltas.csv"), "w",
              newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=delta_columns)
        writer.writeheader()
        writer.writerows(delta_rows)
    return rows


def load_yaml(path):
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def overlaid_value(paths, section, name):
    value = None
    for path in paths:
        section_values = (load_yaml(path) or {}).get(section, {})
        if name in section_values:
            value = section_values[name]
    if value is None:
        raise RuntimeError("missing configuration value {}/{}".format(
            section, name))
    return value


def master_port():
    return urlparse(os.environ.get(
        "ROS_MASTER_URI", "http://127.0.0.1:11311")).port or 11311


def stop(process, timeout=10.0):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=timeout)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()


def start(command, log_path, environment=None):
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    stream = open(log_path, "w", encoding="utf-8")
    process = subprocess.Popen(
        command, stdout=stream, stderr=subprocess.STDOUT,
        env=environment, start_new_session=True)
    process._benchmark_log = stream
    return process


def close_log(process):
    if process is not None and hasattr(process, "_benchmark_log"):
        process._benchmark_log.close()


def wait_for_master(timeout=15.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if rosgraph.is_master_online():
            return
        time.sleep(0.1)
    raise RuntimeError("ROS master startup timeout")


def wait_for_topic(topic, timeout=30.0, process=None):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if process is not None and process.poll() is not None:
            raise RuntimeError("launch exited before topic {} appeared".format(topic))
        result = subprocess.run(
            ["rostopic", "list"], stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, text=True, timeout=5.0)
        if topic in result.stdout.splitlines():
            return
        time.sleep(0.25)
    raise RuntimeError("startup timeout waiting for {}".format(topic))


def subscriptions_ready(system_state, required):
    subscribers = {topic: set(nodes) for topic, nodes in system_state[1]}
    return all(node in subscribers.get(topic, set())
               for topic, node in required.items())


def wait_for_subscriptions(required, timeout=30.0, process=None):
    deadline = time.time() + timeout
    master_api = rosgraph.Master("/soft_vofod_benchmark")
    while time.time() < deadline:
        if process is not None and process.poll() is not None:
            raise RuntimeError("launch exited before input subscribers were ready")
        if subscriptions_ready(master_api.getSystemState(), required):
            return
        time.sleep(0.1)
    raise RuntimeError("startup timeout waiting for input subscribers: {}".format(
        ", ".join(sorted(required))))


def recorder_subscriptions_ready(system_state, topics):
    subscribers = {topic: nodes for topic, nodes in system_state[1]}
    return all(any(node.startswith("/record_") or node == "/record"
                   for node in subscribers.get(topic, []))
               for topic in topics)


def wait_for_recorder_subscriptions(topics, process, timeout=30.0):
    deadline = time.time() + timeout
    master_api = rosgraph.Master("/soft_vofod_benchmark")
    while time.time() < deadline:
        if process.poll() is not None:
            raise RuntimeError("rosbag recorder exited before subscriptions were ready")
        if recorder_subscriptions_ready(master_api.getSystemState(), topics):
            return
        time.sleep(0.1)
    raise RuntimeError("startup timeout waiting for rosbag output subscriptions")


class RunContractError(RuntimeError):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


def validate_run_timing(algorithm, evidence, source_manifest):
    first_scored = source_manifest["first_scored_input_stamp"]
    first_source = source_manifest.get("first_checked_ray_stamp")
    first_ack = evidence.get("first_input_ack_stamp")
    if first_ack is None or first_ack > first_scored:
        raise RunContractError(
            "INVALID_INPUT_HANDSHAKE",
            "first input ack {} is later than first scored input {}".format(
                first_ack, first_scored))
    if first_source is not None and first_ack > first_source + 1.0e-6:
        raise RunContractError(
            "INVALID_INPUT_HANDSHAKE",
            "first input ack {} missed source input {}".format(
                first_ack, first_source))
    if algorithm != "B0":
        return
    complete = evidence.get("background_warmup_complete_stamp")
    gate = source_manifest.get("first_target_spawn_stamp")
    if gate is None:
        gate = source_manifest["scoring_start_stamp"]
    if complete is None or complete >= gate:
        raise RunContractError(
            "INVALID_WARMUP",
            "B0 warm-up completion {} must precede scoring/target gate {}".format(
                complete, gate))
    if evidence.get("first_scored_warmup_active") is not False:
        raise RunContractError(
            "INVALID_WARMUP", "B0 warm-up is active on the first scored frame")


@contextlib.contextmanager
def master(log_path):
    if rosgraph.is_master_online():
        raise RuntimeError("a ROS master is already running; refusing shared benchmark state")
    process = start(["roscore", "-p", str(master_port())], log_path)
    try:
        wait_for_master()
        yield
    finally:
        stop(process)
        close_log(process)
        deadline = time.time() + 10.0
        while rosgraph.is_master_online() and time.time() < deadline:
            time.sleep(0.1)


def target_primitives():
    output = [
        {"id": "body", "type": "box", "pose": [0, 0, 0, 0, 0, 0],
         "size": [0.50, 0.34, 0.18]},
        {"id": "arm_a", "type": "box", "pose": [0, 0, 0, 0, 0, 0.7853981633974483],
         "size": [1.00, 0.07, 0.05]},
        {"id": "arm_b", "type": "box", "pose": [0, 0, 0, 0, 0, -0.7853981633974483],
         "size": [1.00, 0.07, 0.05]},
    ]
    for name, x, y in (("rotor_fl", 0.35, 0.35), ("rotor_fr", 0.35, -0.35),
                       ("rotor_rl", -0.35, 0.35), ("rotor_rr", -0.35, -0.35)):
        output.append({"id": name, "type": "cylinder",
                       "pose": [x, y, 0.035, 0, 0, 0],
                       "radius": 0.17, "length": 0.025})
    return output


def static_primitives(world):
    output = [{"id": "ground", "type": "plane",
               "pose": [0, 0, 0, 0, 0, 0], "size": [100, 100, 0]}]
    if world == "E1_sparse":
        output.append({"id": "wide_wall", "type": "box",
                       "pose": [25, 0, 4, 0, 0, 0], "size": [0.5, 30, 8]})
    elif world in ("E2_occlusion_arena", "E4_long_occlusion"):
        output += [
            {"id": "background_wall", "type": "box",
             "pose": [10, 0, 3, 0, 0, 0],
             "size": [0.5, 3.0 if world == "E4_long_occlusion" else 0.8, 6]},
            {"id": "pillar", "type": "cylinder",
             "pose": [7, -6, 2, 0, 0, 0], "radius": 0.6, "length": 4},
        ]
    return output


def truth_configuration(scenario, ray_mode):
    sources = [{"id": "uav1",
                "topic": "/mid360_multi_uav_sim/ground_truth/uav1/odom"}]
    targets = []
    for target in scenario["targets"]:
        sources.append({"id": target["id"],
                        "topic": "/mid360_multi_uav_sim/ground_truth/{}/odom".format(
                            target["id"])})
        targets.append({"id": target["id"], "truth_source": target["id"],
                        "primitives": target_primitives()})
    return {
        "input_topic": "/uav1/mid360/rays_checked",
        "output_topic": "/evaluation/visibility_ground_truth",
        "world_frame": "world",
        "geometry_contract_id": scenario["scenario_id"] + "-primitive-v1",
        "ray_time_geometry_mode": ray_mode,
        "per_ray_require_static_targets": False,
        "maximum_truth_gap_sec": 0.25,
        "pending_bundle_limit": 100,
        "truth_buffer_size": 10000,
        "truth_sources": sources,
        "targets": targets,
        "static_primitives": static_primitives(scenario["world"]),
    }


def generated_observer(source_model, destination, ray_mode, noise_stddev):
    with open(source_model, encoding="utf-8") as stream:
        model = stream.read()
    model = model.replace(
        "<ray_time_geometry_mode>per_ray_pose</ray_time_geometry_mode>",
        "<ray_time_geometry_mode>{}</ray_time_geometry_mode>".format(ray_mode))
    model = model.replace("<stddev>0.0</stddev>",
                          "<stddev>{}</stddev>".format(noise_stddev))
    if ray_mode not in model or "<stddev>{}</stddev>".format(noise_stddev) not in model:
        raise RuntimeError("observer SDF generation did not replace required fields")
    with open(destination, "w", encoding="utf-8") as stream:
        stream.write(model)


class BenchmarkRunner:
    def __init__(self, arguments):
        self.arguments = arguments
        rospack = rospkg.RosPack()
        self.sim_root = rospack.get_path("mid360_multi_uav_sim")
        self.soft_root = rospack.get_path("soft_vofod_mid360")
        if not arguments.soft_config_overlay:
            arguments.soft_config_overlay = os.path.join(
                self.soft_root, "config", "no_overlay.yaml")
        self.evaluation_root = rospack.get_path("soft_vofod_evaluation")
        self.vofod_root = rospack.get_path("vofod_mid360")
        self.tracker_root = rospack.get_path("lidar_tracker_mid360")
        self.preprocessor_root = rospack.get_path("mid360_ray_preprocessor")
        self.truth_root = rospack.get_path("tclv_evaluation")
        self.sim_plugin_root = rospack.get_path("livox_laser_simulation")
        evaluator_nodes = find_node(
            "tclv_evaluation", "visibility_ground_truth_evaluator")
        if not evaluator_nodes:
            raise RuntimeError("visibility_ground_truth_evaluator is not built")
        self.truth_evaluator = evaluator_nodes[0]
        self.scenario_root = os.path.join(self.sim_root, "config", "benchmarks")
        self.noise_modes = load_yaml(os.path.join(
            self.evaluation_root, "config", "noise_modes.yaml"))
        self.required_warmup_s = float(load_yaml(os.path.join(
            self.vofod_root, "config", "b0_mid360_canonical.yaml"))[
                "background_warmup"]["duration_sec"])
        self.summary = []
        self.git_commit = self._git_commit()
        if shutil.disk_usage(arguments.output).free < 2 * 1024 ** 3:
            raise RuntimeError("less than 2 GiB free space for benchmark artifacts")

    @staticmethod
    def _git_commit():
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"], stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, text=True)
        return result.stdout.strip() if result.returncode == 0 else "uncommitted-no-commit"

    def scene_path(self, scene):
        path = os.path.join(self.scenario_root, scene + ".yaml")
        if not os.path.exists(path):
            raise RuntimeError("missing benchmark scene {}".format(scene))
        return path

    def source_directory(self, scene, noise, seed):
        return os.path.join(self.arguments.output, "source_bags", scene, noise,
                            "seed_{}".format(seed))

    def run_directory(self, algorithm, scene, noise, seed):
        return os.path.join(self.arguments.output, "runs", algorithm, scene, noise,
                            "seed_{}".format(seed))

    def source_implementation_paths(self):
        paths = [
            os.path.join(self.sim_root, "scripts", "benchmark_scenario.py"),
            os.path.join(self.sim_root, "launch", "benchmark.launch"),
            os.path.join(self.preprocessor_root, "src",
                         "mid360_ray_preprocessor_node.cpp"),
            os.path.join(self.truth_root, "src",
                         "visibility_ground_truth_evaluator.cpp"),
        ]
        for subtree in ("src", "include"):
            root = os.path.join(self.sim_plugin_root, subtree)
            if not os.path.isdir(root):
                continue
            for directory, _, files in os.walk(root):
                paths.extend(os.path.join(directory, name) for name in files
                             if os.path.splitext(name)[1] in
                             (".cpp", ".h", ".hpp"))
        return paths

    def algorithm_config_paths(self, algorithm):
        if algorithm == "B0":
            return [
                os.path.join(self.vofod_root, "config", "b0_mid360_canonical.yaml"),
                os.path.join(self.tracker_root, "config", "tracking.yaml"),
            ]
        config_name = "soft_vofod_v2_canonical.yaml" \
            if algorithm in ("A1", "A2", "A3", "B1", "B2", "B3", "B4") \
            else "soft_vofod_v3_canonical.yaml"
        paths = [os.path.join(self.soft_root, "config", config_name)]
        if algorithm in ("A1", "A2"):
            paths.append(os.path.join(
                self.soft_root, "config", algorithm.lower() + ".yaml"))
        if self.arguments.soft_config_overlay:
            paths.append(self.arguments.soft_config_overlay)
        return paths

    def algorithm_implementation_paths(self, algorithm):
        roots = (self.vofod_root, self.tracker_root) if algorithm == "B0" \
            else (self.soft_root,)
        paths = []
        for root in roots:
            for subtree in ("src", "include"):
                source_root = os.path.join(root, subtree)
                if not os.path.isdir(source_root):
                    continue
                for directory, _, files in os.walk(source_root):
                    paths.extend(os.path.join(directory, name) for name in files
                                 if os.path.splitext(name)[1] in
                                 (".cpp", ".h", ".hpp"))
        paths.append(os.path.join(
            self.truth_root if algorithm == "B0" else self.soft_root,
            "launch", "b0_canonical.launch" if algorithm == "B0"
            else "soft_vofod.launch"))
        return paths

    def complete_source_manifest(self, manifest_path, manifest):
        implementation = combined_sha256(self.source_implementation_paths())
        changed = False
        if manifest.get("source_implementation_sha256") != implementation:
            manifest["source_implementation_sha256"] = implementation
            changed = True
        if "target_free_input_duration_s" not in manifest or \
                "first_scored_input_stamp" not in manifest:
            timing = self.source_timing_contract(
                os.path.join(os.path.dirname(manifest_path), "source.bag"),
                expect_targets=not manifest.get("no_target_control", False))
            manifest.update(timing)
            changed = True
        if changed:
            write_json(manifest_path, manifest)
        return manifest

    def source_timing_contract(self, bag_path, expect_targets=True):
        first_input = None
        first_spawn = None
        scoring_start = None
        input_stamps = []
        with rosbag.Bag(bag_path) as bag:
            for topic, message, _ in bag.read_messages(topics=[
                    "/uav1/mid360/rays_checked",
                    "/mid360_multi_uav_sim/scenario_events"]):
                if topic.endswith("rays_checked"):
                    stamp = message.header.stamp.to_sec()
                    input_stamps.append(stamp)
                    if first_input is None:
                        first_input = stamp
                elif topic.endswith("scenario_events"):
                    event = json.loads(message.data)
                    if event.get("event") == "target_spawned":
                        first_spawn = event["sim_time"] if first_spawn is None \
                            else min(first_spawn, event["sim_time"])
                    elif event.get("event") == "scoring_start":
                        scoring_start = event["sim_time"]
        if first_input is None or scoring_start is None or \
                (expect_targets and first_spawn is None):
            raise RuntimeError("source bag is missing input/spawn/scoring timing evidence")
        first_scored = next(
            (stamp for stamp in input_stamps if stamp >= scoring_start), None)
        if first_scored is None:
            raise RuntimeError("source bag has no input at or after scoring start")
        target_free_gate = first_spawn if first_spawn is not None else scoring_start
        target_free = target_free_gate - first_input
        if (first_spawn is not None and first_spawn != scoring_start) or \
                target_free <= self.required_warmup_s:
            raise RuntimeError(
                "source target-free input is {:.3f}s; must exceed {:.3f}s and "
                "spawn exactly at scoring start".format(
                    target_free, self.required_warmup_s))
        return {
            "first_checked_ray_stamp": first_input,
            "first_target_spawn_stamp": first_spawn,
            "scoring_start_stamp": scoring_start,
            "first_scored_input_stamp": first_scored,
            "target_free_input_duration_s": target_free,
            "required_target_free_duration_s": self.required_warmup_s,
        }

    @staticmethod
    def run_timing_evidence(algorithm, bag_path, source_manifest):
        first_ack = None
        bootstrap_start = None
        warmup_complete = None
        first_scored_warmup_active = None
        scoring_stamp = source_manifest["first_scored_input_stamp"]
        topic = "/uav1/vofod_mid360/map_update_diagnostics" \
            if algorithm == "B0" else "/soft_vofod/diagnostics"
        with rosbag.Bag(bag_path) as bag:
            for _, message, _ in bag.read_messages(topics=[topic]):
                stamp = message.header.stamp.to_sec()
                if algorithm == "B0":
                    if first_ack is None and getattr(message, "first_input_ack", True):
                        first_ack = stamp
                        start = getattr(message, "background_warmup_start_stamp", None)
                        bootstrap_start = start.to_sec() if start else \
                            stamp - float(message.background_warmup_elapsed_sec)
                    explicit_complete = getattr(
                        message, "background_warmup_complete_stamp", None)
                    if explicit_complete and explicit_complete.to_sec() > 0.0:
                        warmup_complete = explicit_complete.to_sec()
                    elif warmup_complete is None and message.background_warmup_complete:
                        warmup_complete = stamp
                    if first_scored_warmup_active is None and stamp >= scoring_stamp:
                        first_scored_warmup_active = bool(
                            message.background_warmup_active)
                else:
                    values = {item.key: item.value for status in message.status
                              for item in status.values}
                    if values.get("first_input_ack") == "true" and first_ack is None:
                        first_ack = stamp
                        bootstrap_start = float(values["map_bootstrap_start_stamp"])
        return {
            "first_input_ack_stamp": first_ack,
            "bootstrap_start_stamp": bootstrap_start,
            "background_warmup_complete_stamp": warmup_complete,
            "first_scored_warmup_active": first_scored_warmup_active,
        }

    def record_source(self, scene, noise, seed):
        source_dir = self.source_directory(scene, noise, seed)
        bag_path = os.path.join(source_dir, "source.bag")
        manifest_path = os.path.join(source_dir, "source_manifest.json")
        if os.path.exists(bag_path) and os.path.exists(manifest_path) and not self.arguments.force:
            manifest = json.load(open(manifest_path, encoding="utf-8"))
            if sha256(bag_path) != manifest.get("source_bag_sha256"):
                raise RuntimeError("existing source bag hash mismatch")
            return bag_path, self.complete_source_manifest(manifest_path, manifest)
        os.makedirs(source_dir, exist_ok=True)
        scenario = load_yaml(self.scene_path(scene))
        scenario["seed"] = seed
        generated_scenario = os.path.join(source_dir, "scenario.yaml")
        with open(generated_scenario, "w", encoding="utf-8") as stream:
            yaml.safe_dump(scenario, stream, sort_keys=False)
        ray_mode = scenario.get(
            "ray_time_geometry_mode",
            "per_ray_pose" if scene == "S07" else "snapshot")
        observer_model = os.path.join(source_dir, "observer.sdf")
        generated_observer(
            os.path.join(self.sim_root, "models", "observer_uav_dynamic", "model.sdf"),
            observer_model, ray_mode, self.noise_modes[noise]["range_stddev_m"])
        truth_path = os.path.join(source_dir, "truth_evaluator.yaml")
        if scenario["targets"]:
            with open(truth_path, "w", encoding="utf-8") as stream:
                yaml.safe_dump(truth_configuration(scenario, ray_mode), stream,
                               sort_keys=False)
        world_path = os.path.join(
            self.sim_root, "worlds", WORLD_FILES[scenario["world"]])
        environment = os.environ.copy()
        environment["GAZEBO_RANDOM_SEED"] = str(seed)
        started = utc_now()
        processes = []
        try:
            with master(os.path.join(source_dir, "roscore.log")):
                launch = start([
                    "roslaunch", "mid360_multi_uav_sim", "benchmark.launch",
                    "world_file:=" + world_path,
                    "scenario_file:=" + generated_scenario,
                    "observer_model_file:=" + observer_model,
                    "ray_time_geometry_mode:=" + ray_mode,
                ], os.path.join(source_dir, "simulation.log"), environment)
                processes.append(launch)
                wait_for_topic("/uav1/mid360/rays_checked", 45.0, launch)
                evaluator = None
                if scenario["targets"]:
                    subprocess.run(
                        ["rosparam", "load", truth_path,
                         "/visibility_ground_truth_evaluator"], check=True)
                    evaluator = start([
                        self.truth_evaluator,
                        "__name:=visibility_ground_truth_evaluator",
                    ], os.path.join(source_dir, "truth_evaluator.log"))
                    processes.append(evaluator)
                    wait_for_topic(
                        "/evaluation/visibility_ground_truth", 20.0, evaluator)
                topics = [
                    "/clock", "/tf", "/tf_static",
                    "/uav1/mid360/points_world",
                    "/uav1/mid360/rays_checked",
                    "/mid360_multi_uav_sim/scenario_events",
                    "/mid360_multi_uav_sim/ground_truth/uav1/odom",
                ] + ["/mid360_multi_uav_sim/ground_truth/{}/odom".format(
                    target["id"]) for target in scenario["targets"]]
                if scenario["targets"]:
                    topics.append("/evaluation/visibility_ground_truth")
                recorder = start(
                    ["rosbag", "record", "--lz4", "-O", bag_path] + topics,
                    os.path.join(source_dir, "rosbag_record.log"))
                processes.append(recorder)
                timeout = (scenario["duration_s"] + scenario.get("start_delay_s", 2.0)) * 4.0 + 90.0
                try:
                    return_code = launch.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    raise RuntimeError("scenario completion timeout")
                if return_code != 0:
                    raise RuntimeError("simulation launch exited with {}".format(return_code))
                stop(recorder)
                if evaluator is not None and evaluator.poll() not in (None, 0):
                    raise RuntimeError("truth evaluator crashed")
        finally:
            for process in reversed(processes):
                stop(process)
                close_log(process)
        if not os.path.exists(bag_path) or os.path.getsize(bag_path) == 0:
            raise RuntimeError("source recorder produced no bag")
        timing_contract = self.source_timing_contract(
            bag_path, expect_targets=bool(scenario["targets"]))
        manifest = {
            "schema_version": 1, "scenario": scene,
            "scenario_id": scenario["scenario_id"], "noise_mode": noise,
            "noise_label": self.noise_modes[noise]["label"], "seed": seed,
            "source_bag_sha256": sha256(bag_path),
            "scenario_config_sha256": sha256(generated_scenario),
            "sensor_model_sha256": sha256(observer_model),
            "world_sha256": sha256(world_path),
            "truth_config_sha256": sha256(truth_path)
            if scenario["targets"] else None,
            "no_target_control": not bool(scenario["targets"]),
            "source_implementation_sha256": combined_sha256(
                self.source_implementation_paths()),
            "git_commit": self.git_commit, "started_utc": started,
            "finished_utc": utc_now(), "ray_time_geometry_mode": ray_mode,
        }
        manifest.update(timing_contract)
        write_json(manifest_path, manifest)
        return bag_path, manifest

    def algorithm_command(self, algorithm, resource_path):
        time_prefix = ["/usr/bin/time", "-v", "-o", resource_path]
        if algorithm == "B0":
            return time_prefix + ["roslaunch", "tclv_evaluation", "b0_canonical.launch",
                                  "output:=log"]
        legacy = algorithm in ("A1", "A2", "A3", "B1", "B2", "B3", "B4")
        config_path = os.path.join(
            self.soft_root, "config",
            "soft_vofod_v2_canonical.yaml" if legacy
            else "soft_vofod_v3_canonical.yaml")
        calibrated_paths = [config_path, self.arguments.soft_config_overlay]
        calibrated_groups = str(overlaid_value(
            calibrated_paths, "birth", "min_groups"))
        calibrated_survival = str(overlaid_value(
            calibrated_paths, "tracker", "survival_lambda_per_s"))
        defaults = {
            "groups": calibrated_groups, "opportunity": "true",
            "feedback": "true", "hungarian": "true",
            "survival_lambda": calibrated_survival, "split": "true",
            "imm": "true", "survival": "true", "reportability": "true",
            "dormant": "true", "certified": "true", "epistemic": "true",
        }
        overrides = {
            "A1": {"groups": "2", "opportunity": "false",
                   "feedback": "false", "hungarian": "false"},
            "A2": {"groups": "3", "opportunity": "false",
                   "feedback": "false", "hungarian": "false"},
            "A3": {"groups": "3"},
            "B1": {"groups": "2", "opportunity": "false",
                   "feedback": "false", "survival_lambda": "0.0"},
            "B2": {"groups": "3", "opportunity": "false",
                   "feedback": "false", "survival_lambda": "0.0"},
            "B3": {"feedback": "false", "reportability": "false",
                   "dormant": "false"},
            "B4": {"reportability": "false", "dormant": "false"},
            "V3-A": {"split": "false", "imm": "false", "dormant": "false"},
            "V3-B": {"dormant": "false"},
            "V3-C": {},
            "S04-base": {"split": "false", "imm": "false",
                         "dormant": "false"},
            "S04-split": {"imm": "false", "dormant": "false"},
            "S04-IMM": {"split": "false", "dormant": "false"},
            "S04-split-IMM": {"dormant": "false"},
            "C0": {"opportunity": "false", "survival": "false",
                   "reportability": "false", "dormant": "false"},
            "C1": {"survival": "false", "reportability": "false",
                   "dormant": "false"},
            "C2": {"dormant": "false"},
            "C3": {},
        }
        values = dict(defaults)
        values.update(overrides[algorithm])
        if legacy:
            values.update({"split": "false", "imm": "false",
                           "reportability": "false", "dormant": "false",
                           "certified": "false", "epistemic": "false"})
            values["survival"] = "false" if algorithm in ("A1", "A2", "B1", "B2") \
                else "true"
        return time_prefix + [
            "roslaunch", "soft_vofod_mid360", "soft_vofod.launch", "output:=log",
            "config:=" + config_path,
            "config_overlay:=" + self.arguments.soft_config_overlay,
            "birth_min_groups:=" + values["groups"],
            "opportunity_aware_existence:=" + values["opportunity"],
            "target_feedback:=" + values["feedback"],
            "hungarian_association:=" + values["hungarian"],
            "survival_lambda_per_s:=" + values["survival_lambda"],
            "track_conditioned_packet_split:=" + values["split"],
            "cv_ca_imm:=" + values["imm"],
            "survival_prediction:=" + values["survival"],
            "reportability_filtering:=" + values["reportability"],
            "dormant_reacquisition:=" + values["dormant"],
            "certified_free_detection:=" + values["certified"],
            "epistemic_unknown_birth:=" + values["epistemic"],
        ]

    def replay(self, algorithm, scene, noise, seed, source_bag, source_manifest):
        run_dir = self.run_directory(algorithm, scene, noise, seed)
        bag_path = os.path.join(run_dir, "output.bag")
        manifest_path = os.path.join(run_dir, "run_manifest.json")
        if os.path.exists(bag_path) and os.path.exists(manifest_path) and not self.arguments.force:
            manifest = json.load(open(manifest_path, encoding="utf-8"))
            if manifest.get("source_bag_sha256") != source_manifest["source_bag_sha256"]:
                raise RuntimeError("run manifest references a different source bag")
            manifest.setdefault("replay_rate", 1.0)
            manifest["git_commit"] = self.git_commit
            manifest["algorithm_config_sha256"] = combined_sha256(
                self.algorithm_config_paths(algorithm))
            manifest["algorithm_implementation_sha256"] = combined_sha256(
                self.algorithm_implementation_paths(algorithm))
            evidence = self.run_timing_evidence(
                algorithm, bag_path, source_manifest)
            manifest["input_timing"] = evidence
            try:
                validate_run_timing(algorithm, evidence, source_manifest)
                manifest["status"] = "ok"
            except RunContractError as exception:
                manifest["status"] = exception.status
                write_json(manifest_path, manifest)
                raise
            write_json(manifest_path, manifest)
            return bag_path, manifest
        os.makedirs(run_dir, exist_ok=True)
        resource_path = os.path.join(run_dir, "resource.txt")
        started = utc_now()
        processes = []
        try:
            with master(os.path.join(run_dir, "roscore.log")):
                subprocess.run(["rosparam", "set", "/use_sim_time", "true"], check=True)
                algorithm_process = start(
                    self.algorithm_command(algorithm, resource_path),
                    os.path.join(run_dir, "algorithm.log"))
                processes.append(algorithm_process)
                # The legacy tracker blocks its constructor in
                # ros::Time::waitForValid().  Seed simulated time before topic
                # discovery; source replay still supplies every scored clock.
                subprocess.run([
                    "rostopic", "pub", "-1", "/clock", "rosgraph_msgs/Clock",
                    "clock: {secs: 1, nsecs: 0}",
                ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    check=True, timeout=10.0)
                track_topic = "/uav1/batch/b0/tracks" if algorithm == "B0" \
                    else "/soft_vofod/tracks"
                wait_for_topic(track_topic, 30.0, algorithm_process)
                input_node = "/uav1/vofod_mid360" if algorithm == "B0" \
                    else "/soft_vofod"
                wait_for_subscriptions({
                    "/uav1/mid360/points_world": input_node,
                    "/uav1/mid360/rays_checked": input_node,
                }, 30.0, algorithm_process)
                output_topics = ([
                    "/uav1/batch/b0/tracks",
                    "/uav1/vofod_mid360/background_points",
                    "/uav1/vofod_mid360/free_voxels",
                    "/uav1/vofod_mid360/map_update_diagnostics",
                    "/uav1/vofod_mid360/profiling_info",
                    "/uav1/batch/b0/lidar_tracker_mid360/profiling_info",
                ] if algorithm == "B0" else [
                    "/soft_vofod/events", "/soft_vofod/maintenance_packets",
                    "/soft_vofod/tracks", "/soft_vofod/track_memory",
                    "/soft_vofod/background_voxels", "/soft_vofod/free_voxels",
                    "/soft_vofod/observed_free_voxels",
                    "/soft_vofod/candidate_background_voxels",
                    "/soft_vofod/opportunity_debug", "/soft_vofod/diagnostics",
                ])
                recorder = start(
                    ["rosbag", "record", "--lz4", "-O", bag_path] + output_topics,
                    os.path.join(run_dir, "rosbag_record.log"))
                processes.append(recorder)
                wait_for_recorder_subscriptions(output_topics, recorder)
                player = start([
                    "rosbag", "play", "--clock", "--delay=2.0", "--rate",
                    str(self.arguments.replay_rate), source_bag, "--topics",
                    "/tf", "/tf_static", "/uav1/mid360/points_world",
                    "/uav1/mid360/rays_checked",
                ], os.path.join(run_dir, "rosbag_play.log"))
                processes.append(player)
                while player.poll() is None:
                    if algorithm_process.poll() is not None:
                        raise RuntimeError("algorithm crashed during replay")
                    time.sleep(0.25)
                if player.returncode != 0:
                    raise RuntimeError("rosbag play failed with {}".format(player.returncode))
                time.sleep(1.0)
                stop(recorder)
        finally:
            for process in reversed(processes):
                stop(process)
                close_log(process)
        if not os.path.exists(bag_path) or os.path.getsize(bag_path) == 0:
            raise RuntimeError("algorithm recorder produced no output bag")
        manifest = {
            "schema_version": 1, "algorithm": algorithm, "scenario": scene,
            "noise_mode": noise, "seed": seed,
            "source_bag_sha256": source_manifest["source_bag_sha256"],
            "output_bag_sha256": sha256(bag_path),
            "algorithm_config_sha256": combined_sha256(
                self.algorithm_config_paths(algorithm)),
            "algorithm_implementation_sha256": combined_sha256(
                self.algorithm_implementation_paths(algorithm)),
            "git_commit": self.git_commit, "started_utc": started,
            "finished_utc": utc_now(), "replay_rate": self.arguments.replay_rate,
        }
        evidence = self.run_timing_evidence(
            algorithm, bag_path, source_manifest)
        manifest["input_timing"] = evidence
        try:
            validate_run_timing(algorithm, evidence, source_manifest)
            manifest["status"] = "ok"
        except RunContractError as exception:
            manifest["status"] = exception.status
            write_json(manifest_path, manifest)
            raise
        write_json(manifest_path, manifest)
        return bag_path, manifest

    def evaluate(self, algorithm, scene, noise, seed, source_bag, run_bag):
        run_dir = self.run_directory(algorithm, scene, noise, seed)
        command = [
            sys.executable, os.path.join(self.evaluation_root, "scripts", "evaluate_bag.py"),
            "--source", source_bag, "--run", run_bag, "--algorithm", algorithm,
            "--scenario", os.path.join(self.source_directory(scene, noise, seed),
                                        "scenario.yaml"),
            "--output", run_dir, "--resource", os.path.join(run_dir, "resource.txt"),
        ]
        result = subprocess.run(command, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True)
        with open(os.path.join(run_dir, "evaluation.log"), "w", encoding="utf-8") as stream:
            stream.write(result.stdout)
        if result.returncode != 0:
            raise RuntimeError("evaluation failed; see {}".format(
                os.path.join(run_dir, "evaluation.log")))

    def run(self):
        for scene in self.arguments.scenes:
            for noise in self.arguments.noise:
                for seed in self.arguments.seeds:
                    source_bag = source_manifest = None
                    current_algorithm = None
                    try:
                        print("[benchmark] {} {} seed {}: source".format(
                            scene, noise, seed), flush=True)
                        if self.arguments.record:
                            source_bag, source_manifest = self.record_source(scene, noise, seed)
                        else:
                            source_dir = self.source_directory(scene, noise, seed)
                            source_bag = os.path.join(source_dir, "source.bag")
                            source_manifest = json.load(open(
                                os.path.join(source_dir, "source_manifest.json"),
                                encoding="utf-8"))
                            source_manifest = self.complete_source_manifest(
                                os.path.join(source_dir, "source_manifest.json"),
                                source_manifest)
                        for algorithm in self.arguments.algorithms:
                            current_algorithm = algorithm
                            print("[benchmark] {} {} seed {}: {}".format(
                                scene, noise, seed, algorithm), flush=True)
                            run_bag = None
                            if self.arguments.replay:
                                run_bag, _ = self.replay(
                                    algorithm, scene, noise, seed,
                                    source_bag, source_manifest)
                                self.evaluate(
                                    algorithm, scene, noise, seed, source_bag, run_bag)
                                print("[benchmark] {} {} seed {}: {} ok".format(
                                    scene, noise, seed, algorithm), flush=True)
                            self.summary.append({
                                "scene": scene, "noise": noise, "seed": seed,
                                "algorithm": algorithm, "status": "ok"})
                    except Exception as exception:
                        self.summary.append({
                            "scene": scene, "noise": noise, "seed": seed,
                            "algorithm": current_algorithm,
                            "status": getattr(exception, "status", "failed"),
                            "error": str(exception)})
                        write_json(os.path.join(self.arguments.output,
                                                "benchmark_summary.json"), self.summary)
                        aggregate_results(self.arguments.output)
                        raise
        rows = aggregate_results(self.arguments.output)
        write_json(os.path.join(self.arguments.output, "benchmark_summary.json"), [{
            "algorithm": row["algorithm"], "scene": row["scene"],
            "noise": row["noise"], "seed": row["seed"], "status": "ok",
        } for row in rows])


def comma_list(value, allowed):
    output = tuple(item.strip() for item in value.split(",") if item.strip())
    unknown = sorted(set(output) - set(allowed))
    if not output or unknown:
        raise argparse.ArgumentTypeError("invalid values: {}".format(",".join(unknown)))
    return output


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--algorithms", default="B0,B1,B2,B3,B4")
    parser.add_argument("--scenes", default=",".join(DEFAULT_SCENES))
    parser.add_argument("--noise", default="N0")
    parser.add_argument("--seeds", default="1001")
    parser.add_argument("--record", action="store_true")
    parser.add_argument("--replay", action="store_true")
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--replay-rate", type=float, default=1.0)
    parser.add_argument("--output", default="artifacts")
    parser.add_argument("--soft-config-overlay", default="")
    arguments = parser.parse_args()
    arguments.algorithms = comma_list(arguments.algorithms, ALGORITHMS)
    arguments.scenes = comma_list(arguments.scenes, SCENES)
    arguments.noise = comma_list(arguments.noise, ("N0", "N1", "N2"))
    try:
        arguments.seeds = tuple(int(item) for item in arguments.seeds.split(","))
    except ValueError as exception:
        raise SystemExit("--seeds must be comma-separated integers") from exception
    if not arguments.record and not arguments.replay:
        arguments.record = arguments.replay = True
    if not math.isfinite(arguments.replay_rate) or arguments.replay_rate <= 0.0:
        raise SystemExit("--replay-rate must be finite and positive")
    arguments.output = os.path.abspath(arguments.output)
    if arguments.soft_config_overlay:
        arguments.soft_config_overlay = os.path.abspath(
            arguments.soft_config_overlay)
    os.makedirs(arguments.output, exist_ok=True)
    return arguments


if __name__ == "__main__":
    try:
        BenchmarkRunner(parse_arguments()).run()
    except Exception as exception:
        print("benchmark failed: {}".format(exception), file=sys.stderr)
        sys.exit(1)
