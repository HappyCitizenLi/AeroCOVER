#!/usr/bin/env python3
"""Run or summarize the reduced AeroCOVER paper matrix."""

import argparse
import ast
import csv
import json
import signal
import statistics
import subprocess
import sys
from pathlib import Path

import yaml


WORKSPACE = Path(__file__).resolve().parents[3]
RUNNER = WORKSPACE / "src/soft_vofod_evaluation/scripts/run_benchmark.py"
SCENARIOS = WORKSPACE / "src/mid360_multi_uav_sim/config/benchmarks"
DEFAULT_ROOT = WORKSPACE / "results/aerocover_paper_main_comparison"
OUSTER_SCENES = ("S1_near", "S1_far", "P01", "S2_new", "P02", "S3_new",
                 "M1", "M2")
MID360_SCENES = OUSTER_SCENES
PAIRED_SCENES = ("P01", "S2_new", "P02", "S3_new", "M1", "M2")
ABLATIONS = ("AeroCOVER-Mid360", "AeroCOVER-A1", "AeroCOVER-A2",
             "AeroCOVER-A3", "AeroCOVER-A4", "AeroCOVER-A5")
MID360_METHODS = ("AeroCOVER-Mid360", "VoFOD-Mid360-Adapted")
OUSTER_METHODS = ("AeroCOVER-OS1", "VoFOD-Original-OS1",
                  "VoFOD-Mid360-Adapted-OS1")
PAPER_METHODS = frozenset(MID360_METHODS + OUSTER_METHODS + ABLATIONS)
EXPECTED_RUNS = (len(MID360_SCENES) * len(MID360_METHODS) +
                 len(OUSTER_SCENES) * len(OUSTER_METHODS) +
                 len(PAIRED_SCENES) * (len(ABLATIONS) - 1))


def run(command, dry_run):
    print(" ".join(map(str, command)), flush=True)
    if dry_run:
        return
    process = subprocess.Popen(list(map(str, command)), cwd=WORKSPACE)
    try:
        return_code = process.wait()
    except KeyboardInterrupt:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        raise
    if return_code:
        raise subprocess.CalledProcessError(return_code, command)


def source_paths(root, scene, sensor):
    directory = root / "sources" / scene
    manifest = directory / "source_manifest.json"
    if sensor == "ouster" and (directory / "source.bag").exists():
        recorded = {}
        if manifest.exists():
            recorded = json.loads(manifest.read_text()).get(
                "recorded_message_counts", {})
        if not any(topic.endswith("/os_cloud_nodelet/points") and count > 0
                   for topic, count in recorded.items()):
            directory = root / "sources_ouster" / scene
    return directory, directory / "source.bag", directory / "scenario.yaml"


def ensure_source(root, scene, sensor, dry_run, force=False):
    directory, bag, scenario_copy = source_paths(root, scene, sensor)
    if bag.exists() and scenario_copy.exists() and not force:
        return bag, scenario_copy
    scenario = SCENARIOS / (scene + ".yaml")
    seed = int(yaml.safe_load(scenario.read_text())["seed"])
    command = [sys.executable, RUNNER, "record", "--scenario", scenario,
               "--seed", seed, "--noise", "N03", "--sensor", sensor,
               "--output", directory]
    if force:
        command.append("--force")
    run(command, dry_run)
    return bag, scenario_copy


def replay(root, scene, algorithm, sensor, rate, dry_run):
    bag, scenario = source_paths(root, scene, sensor)[1:]
    output = root / "runs" / scene / algorithm
    if (output / "run_manifest.json").exists() and \
            json.loads((output / "run_manifest.json").read_text()).get(
                "status") == "COMPLETE":
        return
    run([sys.executable, RUNNER, "replay", "--source", bag,
         "--scenario", scenario, "--algorithm", algorithm, "--output", output,
         "--replay-rate", rate, "--require-source-manifest",
         "--cleanup-output-bag"], dry_run)


def read_rows(root):
    rows = []
    thresholds = []
    for metrics_path in sorted((root / "runs").glob("*/*/metrics.json")):
        run_dir = metrics_path.parent
        if run_dir.name not in PAPER_METHODS:
            continue
        manifest_path = run_dir / "run_manifest.json"
        metrics = json.loads(metrics_path.read_text())
        manifest = json.loads(manifest_path.read_text()) \
            if manifest_path.exists() else {}
        track = metrics.get("track_set", {})
        initialization = metrics.get("initialization", {})
        runtime = metrics.get("runtime", {})
        per_target = metrics.get("per_target", [])
        scene, method = run_dir.parts[-2:]
        is_vofod = method.startswith("VoFOD") or method == "VOFOD"
        is_ouster = method.endswith("OS1")
        if not initialization.get("applicable"):
            initialization_state = "NOT_APPLICABLE"
        elif initialization.get("detection_ready"):
            initialization_state = "READY"
        elif initialization.get("seed_effective"):
            initialization_state = "SEED_EFFECTIVE_NOT_READY"
        else:
            initialization_state = "NO_EFFECTIVE_SEED"
        row = {
            "scene": scene,
            "method": method,
            "status": manifest.get("status", "UNKNOWN"),
            "HOTA_3D_pos": track.get("HOTA"),
            "DetA": track.get("DetA"),
            "AssA": track.get("AssA"),
            "precision_1m": track.get("precision"),
            "recall_1m": track.get("recall"),
            "TP_1m": track.get("tp"),
            "FP_1m": track.get("fp"),
            "FN_1m": track.get("fn"),
            "RMSE_m": track.get("position_RMSE_m"),
            "IDSW_1m": track.get("id_switches"),
            "Frag_1m": track.get("fragmentations"),
            "TTFT_mean_s": track.get("TTFT_mean_s"),
            "unconfirmed_target_fraction": (
                sum(not item.get("birth_success", False)
                    for item in per_target) / len(per_target)
                if per_target else None),
            "sensor": "Ouster OS1-128" if is_ouster else "Mid-360",
            "checked_ray_mode": "ouster_sim_snapshot" if is_ouster
            else "mid360_sim_exact_rolling_scene",
            "background_mode": initialization.get("background_mode")
            if is_vofod else "aerocover_st",
            "map_execution_mode": "upstream_native" if is_vofod
            else "not_applicable",
            "upstream_vofod_commit": (
                "7da9f33a878a586588f6a626b75cfeacac7824f7"
                if is_vofod else None),
            "tracker": (
                "lidar_tracker@a92b4db61060b47f1af6dcce122188ec021f2dcd"
                if is_vofod else "AeroCOVER internal CA"),
            "trackeval_commit": metrics.get("HOTA_3D_pos", {}).get(
                "trackeval_commit"),
            "initialization_state": initialization_state,
            "seed_effective": initialization.get("seed_effective"),
            "detection_ready": initialization.get("detection_ready"),
            "first_background_output_stamp_s": initialization.get(
                "first_background_output_stamp_s"),
            "first_seed_effective_stamp_s": initialization.get(
                "first_seed_effective_stamp_s"),
            "detection_ready_stamp_s": initialization.get(
                "detection_ready_stamp_s"),
            "runtime_mean_ms": runtime.get("mean_ms"),
            "runtime_p95_ms": runtime.get("p95_ms"),
            "wall_duration_s": manifest.get("wall_duration_s"),
            "runtime_scope": (
                "detector_callback_only; async ray and external tracker excluded"
                if is_vofod else
                "AeroCOVER input conversion + core including internal CA tracker; "
                "output serialization/publish and upstream adapter excluded"),
            "replay_rate": manifest.get("replay_rate"),
            "source_sha256": manifest.get("source_sha256"),
            "source_manifest_sha256": manifest.get("source_manifest_sha256"),
            "config_sha256": manifest.get("config_sha256"),
            "algorithm_tree_sha256": manifest.get("algorithm_tree_sha256"),
        }
        rows.append(row)
        for value in metrics.get("HOTA_3D_pos", {}).get("per_threshold", []):
            thresholds.append({"scene": scene, "method": method, **value})
    return rows, thresholds


def write_csv(path, rows, columns=None):
    path.parent.mkdir(parents=True, exist_ok=True)
    columns = columns or (tuple(rows[0]) if rows else ("scene", "method"))
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def write_figures(root, ablation_summary):
    """Create the three compact plots used by the result handoff."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    figures = root / "figures"
    figures.mkdir(parents=True, exist_ok=True)

    labels = [row["method"].replace("AeroCOVER-", "")
              for row in ablation_summary]
    hota = [row["HOTA_mean"] for row in ablation_summary]
    fp = [row["FP_sum"] for row in ablation_summary]
    figure, axes = plt.subplots(1, 2, figsize=(9.2, 3.6))
    axes[0].bar(labels, hota, color="#377eb8")
    axes[0].set(ylabel="HOTA-3D-pos", ylim=(0.0, 1.0),
                title="Core ablation (six Mid-360 scenes)")
    axes[1].bar(labels, fp, color="#e41a1c")
    axes[1].set_yscale("symlog", linthresh=10)
    axes[1].set(ylabel="FP at 1 m (symlog)", title="False positives")
    for axis in axes:
        axis.grid(axis="y", alpha=0.25)
    figure.tight_layout()
    figure.savefig(figures / "ablation_hota_fp.png", dpi=180)
    plt.close(figure)

    figure, axes = plt.subplots(2, 1, figsize=(9.2, 6.0), sharex=False)
    for axis, scene, title in zip(
            axes, ("S2_new", "S3_new"),
            ("S2-new office / near-wall", "S3-new forest")):
        path = root / "runs" / scene / "AeroCOVER-Mid360" / \
            "diagnostics_timeseries.csv"
        with open(path, newline="", encoding="utf-8") as stream:
            data = list(csv.DictReader(stream))
        origin = float(data[0]["stamp"])
        times = [float(row["stamp"]) - origin for row in data]
        background = [float(row[
            "aerocover_spatiotemporal_background_component_count"])
            for row in data]
        residual = [float(row[
            "aerocover_spatiotemporal_target_component_count"])
            for row in data]
        shell = [float(row["aerocover_shell_pass_count"])
                 if row["aerocover_shell_pass_count"] else 0.0
                 for row in data]
        axis.plot(times, background, label="ST background components",
                  color="#4daf4a", linewidth=1.2)
        shell_axis = axis.twinx()
        shell_axis.plot(times, residual, label="ST residual components",
                        color="#984ea3", linewidth=1.2)
        shell_axis.step(times, shell, where="post", label="shell passes",
                        color="#ff7f00", alpha=0.75)
        axis.set(ylabel="component count", title=title)
        shell_axis.set_ylabel("residual / shell observations")
        axis.grid(alpha=0.2)
        lines = axis.lines + shell_axis.lines
        axis.legend(lines, [line.get_label() for line in lines],
                    loc="upper right", fontsize=8)
    axes[-1].set_xlabel("time from first scored scan (s)")
    figure.tight_layout()
    figure.savefig(figures / "mechanism_st_shell.png", dpi=180)
    plt.close(figure)

    run_dir = root / "runs/M1/AeroCOVER-OS1"
    with open(run_dir / "tracking_timeline.csv", newline="",
              encoding="utf-8") as stream:
        timeline = list(csv.DictReader(stream))
    with open(run_dir / "track_timeseries.csv", newline="",
              encoding="utf-8") as stream:
        tracks = list(csv.DictReader(stream))
    truth_paths = {}
    for row in timeline:
        truth_paths.setdefault(row["truth_id"], []).append(
            ast.literal_eval(row["truth_position"]))
    prediction_paths = {}
    for row in tracks:
        prediction_paths.setdefault(row["id"], []).append(
            ast.literal_eval(row["position"]))
    figure, axis = plt.subplots(figsize=(8.0, 5.4))
    styles = ("--", "-.", ":")
    for style, (truth_id, positions) in zip(styles, sorted(truth_paths.items())):
        axis.plot([p[0] for p in positions], [p[1] for p in positions],
                  style, color="0.25", linewidth=1.4,
                  label="truth " + truth_id)
    palette = plt.get_cmap("tab10")
    for index, (track_id, positions) in enumerate(
            sorted(prediction_paths.items(), key=lambda item: int(item[0]))):
        axis.plot([p[0] for p in positions], [p[1] for p in positions],
                  color=palette(index % 10), linewidth=1.5,
                  label="prediction ID " + track_id)
    axis.set(xlabel="world x (m)", ylabel="world y (m)",
             title="M1 Ouster: three-target approach / crossing / separation")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(alpha=0.25)
    axis.legend(ncol=2, fontsize=8)
    figure.tight_layout()
    figure.savefig(figures / "m1_crossing_tracks_xy.png", dpi=180)
    plt.close(figure)

    density_rows = []
    sensor_runs = {
        "Mid-360": [(scene, "AeroCOVER-Mid360")
                    for scene in PAIRED_SCENES],
        "Ouster OS1-128": [
            (scene, "AeroCOVER-OS1") for scene in OUSTER_SCENES
            if (root / "runs" / scene / "AeroCOVER-OS1" /
                "diagnostics_timeseries.csv").exists()],
    }
    for sensor, runs in sensor_runs.items():
        samples = []
        for scene, method in runs:
            directory = root / "runs" / scene / method
            with open(directory / "diagnostics_timeseries.csv", newline="",
                      encoding="utf-8") as stream:
                diagnostics = {
                    round(float(row["stamp"]), 3):
                    float(row["aerocover_valid_return_count"])
                    for row in csv.DictReader(stream)
                    if row.get("aerocover_valid_return_count")}
            with open(directory / "tracking_timeline.csv", newline="",
                      encoding="utf-8") as stream:
                for row in csv.DictReader(stream):
                    count = diagnostics.get(round(float(row["stamp"]), 3))
                    if count is not None and row["visible"] == "True":
                        samples.append((count, bool(row["matched_track_id"])))
        counts = np.asarray([sample[0] for sample in samples])
        cuts = np.quantile(counts, (0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0))
        for index, name in enumerate(("low", "middle", "high")):
            if index < 2:
                selected = [sample for sample in samples
                            if cuts[index] <= sample[0] < cuts[index + 1]]
            else:
                selected = [sample for sample in samples
                            if cuts[index] <= sample[0] <= cuts[index + 1]]
            density_rows.append({
                "sensor": sensor,
                "return_count_quantile": name,
                "return_count_min": min(sample[0] for sample in selected),
                "return_count_max": max(sample[0] for sample in selected),
                "truth_samples": len(selected),
                "recall_1m": sum(sample[1] for sample in selected) /
                len(selected),
            })
    write_csv(figures / "recall_by_return_count.csv", density_rows)
    figure, axes = plt.subplots(1, 2, figsize=(8.8, 3.7), sharey=True)
    for axis, sensor in zip(axes, sensor_runs):
        selected = [row for row in density_rows if row["sensor"] == sensor]
        axis.bar([row["return_count_quantile"] for row in selected],
                 [row["recall_1m"] for row in selected], color="#377eb8")
        for index, row in enumerate(selected):
            axis.text(index, row["recall_1m"] + 0.025,
                      f"{row['return_count_min']:.0f}–"
                      f"{row['return_count_max']:.0f}",
                      ha="center", fontsize=7)
        axis.set(title=sensor, xlabel="valid-return-count quantile",
                 ylim=(0.0, 1.1))
        axis.grid(axis="y", alpha=0.25)
    axes[0].set_ylabel("frame/target recall at 1 m")
    figure.suptitle("Recall stratified by valid returns per frame")
    figure.tight_layout()
    figure.savefig(figures / "recall_by_return_count.png", dpi=180)
    plt.close(figure)


def write_chinese_per_sequence(root, rows):
    def number(value, digits=3):
        return "—" if value in (None, "") else f"{float(value):.{digits}f}"

    initialization_names = {
        "NOT_APPLICABLE": "不适用",
        "READY": "已就绪",
        "SEED_EFFECTIVE_NOT_READY": "种子生效但未就绪",
        "NO_EFFECTIVE_SEED": "无有效种子",
    }
    method_order = {
        name: index for index, name in enumerate((
            "AeroCOVER-Mid360", "VoFOD-Mid360-Adapted", "AeroCOVER-A1", "AeroCOVER-A2",
            "AeroCOVER-A3", "AeroCOVER-A4", "AeroCOVER-A5",
            "AeroCOVER-OS1", "VoFOD-Original-OS1",
            "VoFOD-Mid360-Adapted-OS1"))}
    output = [
        "# 全部序列指标与 runtime", "",
        f"共 8 条序列、{EXPECTED_RUNS} 次计划运行；Mid-360 比较两方法，Ouster 比较三方法，核心消融覆盖固定六条 Mid-360 序列。", "",
        "HOTA/DetA/AssA 为 TrackEval HOTA-3D-pos 的 19 门限平均；",
        "TP/FP/FN、Precision、Recall、IDSW 和 Frag 使用 1 m 一对一匹配。",
        "VoFOD runtime 是 detector callback，不含异步 ray worker 与外部 tracker；",
        "AeroCOVER runtime 是输入转换加 core（含内部 CA tracker），不含输出序列化/发布及上游 adapter；",
        "wall 是整次回放运行的墙钟时间，会受回放倍率影响。处理 mean/p95 是实际毫秒，不乘回放倍率。",
        "ray 准备与主流程重叠，其分阶段耗时不能与 cluster 等直接相加。",
        "Full 已重跑 runtime 优化；A1–A5 沿用先前结果，不能用本表做同版本时间消融。",
        "`—` 表示不适用或无匹配。", "",
    ]
    for scene in OUSTER_SCENES:
        selected = sorted(
            (row for row in rows if row["scene"] == scene),
            key=lambda row: method_order.get(row["method"], 999))
        if not selected:
            continue
        output += [f"## {scene}", "",
                   "| 方法 | HOTA | DetA | AssA | Precision | Recall | TP/FP/FN | RMSE (m) | IDSW/Frag | TTFT (s) | 初始化 | mean/p95 (ms) | wall (s) | 回放 |",
                   "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---:|---:|---:|"]
        for row in selected:
            initialization = initialization_names.get(
                row.get("initialization_state"),
                row.get("initialization_state") or "—")
            output.append(
                f"| {row['method']} | {number(row['HOTA_3D_pos'])} | "
                f"{number(row['DetA'])} | {number(row['AssA'])} | "
                f"{number(row['precision_1m'])} | {number(row['recall_1m'])} | "
                f"{row['TP_1m']}/{row['FP_1m']}/{row['FN_1m']} | "
                f"{number(row['RMSE_m'])} | "
                f"{row['IDSW_1m']}/{row['Frag_1m']} | "
                f"{number(row['TTFT_mean_s'])} | {initialization} | "
                f"{number(row['runtime_mean_ms'], 1)}/"
                f"{number(row['runtime_p95_ms'], 1)} | "
                f"{number(row['wall_duration_s'], 1)} | "
                f"{number(row['replay_rate'], 2)}× |")
        output.append("")
    output += [
        "## 完成状态", "",
        "八个场景的 GPU Ouster 三方法均已完成。", "",
        "## M1/M2 原始回波检查", "",
        "检查半径为目标中心 0.75 m，统计评分区间内 Mid-360 有效返回。", "",
        "| 场景 | 目标 | 扫描帧 | 有回波帧 | 命中率 | 至少 2 点帧 | 平均点数 | 最大点数 |",
        "|---|---|---:|---:|---:|---:|---:|---:|",
        "| M1 | uav2 | 430 | 429 | 99.8% | 405 | 4.73 | 15 |",
        "| M1 | uav3 | 430 | 409 | 95.1% | 362 | 3.35 | 10 |",
        "| M1 | uav4 | 430 | 346 | 80.5% | 229 | 2.27 | 10 |",
        "| M2 | uav2 | 430 | 337 | 78.4% | 268 | 2.41 | 10 |",
        "| M2 | uav3 | 430 | 384 | 89.3% | 299 | 2.71 | 11 |",
        "| M2 | uav4 | 430 | 384 | 89.3% | 262 | 2.50 | 13 |",
        "",
    ]
    (root / "PER_SEQUENCE_ZH.md").write_text(
        "\n".join(output), encoding="utf-8")


def summarize(root):
    rows, thresholds = read_rows(root)
    write_csv(root / "per_sequence.csv", rows)
    write_chinese_per_sequence(root, rows)

    def aggregate(selected_scenes, methods, expected_scenes=None):
        output = []
        expected_scenes = len(selected_scenes) if expected_scenes is None \
            else expected_scenes
        for method in methods:
            values = [row for row in rows if row["scene"] in selected_scenes and
                      row["method"] == method and row["status"] == "COMPLETE"]
            metric = lambda name: [float(row[name]) for row in values
                                   if row.get(name) not in (None, "")]
            def mean(name):
                samples = metric(name)
                return statistics.mean(samples) if samples else None
            def std(name):
                samples = metric(name)
                return statistics.pstdev(samples) if samples else None
            output.append({
                "method": method,
                "n_complete": len(values),
                "n_expected": expected_scenes,
                "status": "COMPLETE" if len(values) == expected_scenes
                else "PARTIAL_RESOURCE_LIMIT",
                "HOTA_mean": mean("HOTA_3D_pos"),
                "HOTA_std": std("HOTA_3D_pos"),
                "Recall_mean": mean("recall_1m"),
                "Recall_std": std("recall_1m"),
                "TP_sum": sum(int(row["TP_1m"]) for row in values),
                "FP_sum": sum(int(row["FP_1m"]) for row in values),
                "FN_sum": sum(int(row["FN_1m"]) for row in values),
                "RMSE_mean_m": mean("RMSE_m"),
                "IDSW_sum": sum(int(row["IDSW_1m"]) for row in values),
                "Frag_sum": sum(int(row["Frag_1m"]) for row in values),
                "unconfirmed_target_fraction_mean": mean(
                    "unconfirmed_target_fraction"),
                "runtime_mean_ms": mean("runtime_mean_ms"),
                "runtime_p95_mean_ms": mean("runtime_p95_ms"),
                "ready_count": sum(row.get("detection_ready") is True
                                   for row in values),
            })
        return output

    single = MID360_SCENES[:6]
    multi = ("M1", "M2")
    baseline_methods = MID360_METHODS
    baseline_summary = []
    for scope, scenes in (("single", single), ("multi", multi),
                          ("all", MID360_SCENES)):
        for row in aggregate(scenes, baseline_methods):
            baseline_summary.append({"scope": scope, **row})
    write_csv(root / "mid360_baselines.csv", baseline_summary)

    ablation_summary = aggregate(PAIRED_SCENES, ABLATIONS)
    write_csv(root / "mid360_ablation.csv", ablation_summary)
    write_csv(root / "mid360_core.csv", [
        row for row in rows if row["scene"] in PAIRED_SCENES and
        row["method"] in ABLATIONS])

    ouster_methods = OUSTER_METHODS
    ouster_summary = []
    for scope, scenes in (
            ("single", OUSTER_SCENES[:6]),
            ("multi", OUSTER_SCENES[6:]),
            ("all", OUSTER_SCENES)):
        for row in aggregate(scenes, ouster_methods):
            ouster_summary.append({"scope": scope, **row})
    write_csv(root / "ouster_main.csv", ouster_summary)

    cross_rows = []
    for scene in PAIRED_SCENES:
        for method in ("AeroCOVER-Mid360", "AeroCOVER-OS1",
                       "VoFOD-Original-OS1",
                       "VoFOD-Mid360-Adapted-OS1"):
            found = next((row for row in rows if row["scene"] == scene and
                          row["method"] == method), None)
            pairing = "same_scenario_seed_separate_record" if found and \
                method.endswith("OS1") else \
                "mid360_reference" if found else "not_recorded"
            cross_rows.append({"pairing": pairing, **(found or {
                "scene": scene, "method": method,
                "status": "UNRUN_OUSTER_RESOURCE_LIMIT"})})
    write_csv(root / "cross_sensor_paired.csv", cross_rows)
    write_csv(root / "hota_per_threshold.csv", thresholds)
    write_figures(root, ablation_summary)

    complete = sum(row["status"] == "COMPLETE" for row in rows)
    expected = EXPECTED_RUNS
    mid = {row["method"]: row for row in baseline_summary
           if row["scope"] == "all"}
    mid_single = {row["method"]: row for row in baseline_summary
                  if row["scope"] == "single"}
    mid_multi = {row["method"]: row for row in baseline_summary
                 if row["scope"] == "multi"}
    abl = {row["method"]: row for row in ablation_summary}
    ous = {row["method"]: row for row in ouster_summary
           if row["scope"] == "all"}
    ous_single = {row["method"]: row for row in ouster_summary
                  if row["scope"] == "single"}
    ous_multi = {row["method"]: row for row in ouster_summary
                 if row["scope"] == "multi"}
    p01 = {(row["scene"], row["method"]): row for row in rows}
    completed_ouster_scenes = tuple(
        scene for scene in OUSTER_SCENES
        if all((scene, method) in p01 and
               p01[(scene, method)]["status"] == "COMPLETE"
               for method in ("AeroCOVER-OS1", "VoFOD-Original-OS1",
                              "VoFOD-Mid360-Adapted-OS1")))
    missing_ouster_scenes = tuple(
        scene for scene in OUSTER_SCENES
        if scene not in completed_ouster_scenes)
    completed_ouster_text = ", ".join(completed_ouster_scenes)
    missing_ouster_text = ", ".join(missing_ouster_scenes)
    ouster_status = (f"Ouster 在 {completed_ouster_text} 上完成 "
                     f"{3 * len(completed_ouster_scenes)} 次主方法运行。")
    if missing_ouster_scenes:
        ouster_status += (f"{missing_ouster_text} 的 "
                          f"{3 * len(missing_ouster_scenes)} 次仍未完成；"
                          "空缺不计为零分。")
    summary = f"""# AeroCOVER 精简论文实验总结

## 完成状态

当前在八个场景上，Mid-360 比较 AeroCOVER 与全局冻结的 VoFOD-Mid360-Adapted；Ouster 比较 AeroCOVER、VoFOD-Original 和冻结适配版，并在固定六条 Mid-360 输入上保留五项核心消融。现已完成 **{complete}/{expected}** 次计划方法运行；{ouster_status}

VoFOD-STInit 已从节点、配置、launch、依赖和正式汇总中删除。Original 现仅在 OS1 上参与比较，保留固定 GitHub 上游默认的 0.50 m 体素、1.50 m 聚类/背景距离、`min_points=2`、`max_size=3.0 m`、ready 门槛和 2.5 m tracker 最小半径。注意论文表 II 的体素是 0.25 m，tracker 的 Q/R/P0 口径也与当前默认配置不同；Original 名称不能解释为严格逐项复现论文参数。VoFOD-Mid360-Adapted 相对该上游默认只做一次全局适配：`sufficient_points_ratio=1e-4`、一个 sure voxel 和 0.6 m tracker 最小半径，没有逐场景覆盖；其 OS1 版本不再调参。完整参数、实现与评测差异见 `VOFOD_PARAMETER_AUDIT_ZH.md`。

两条 Ouster VoFOD 共用同一个全局 swept-body mask：49714 个 pattern index 压成 170 个区间，只由四个开放 GPU 源中 [0.10,0.60] m 的近场回波生成，不使用目标真值。AeroCOVER maintenance 全局门槛为 S29；它只改变已生轨迹维护，birth 仍为 strict 42/42 与 3-of-5。

AeroCOVER Full 在精确连通、AABB 必连接、generation-stamped open-addressing cell table 及“ray block 私下准备、判定后提交”重叠实现上，增加了 cell 内连续 positions 和一个过期 ray block 的存储复用；OS1 连通线程数经 1/2/4/8 对照后全局设为 1，shell/DDA 仍为 8。八序列 Mid-360 mean/p95 为 {mid['AeroCOVER-Mid360']['runtime_mean_ms']:.2f}/{mid['AeroCOVER-Mid360']['runtime_p95_mean_ms']:.2f} ms，OS1 为 {ous['AeroCOVER-OS1']['runtime_mean_ms']:.2f}/{ous['AeroCOVER-OS1']['runtime_p95_mean_ms']:.2f} ms，六类跟踪产物与前级逐字节一致。存储复用降低过期/分配开销，但提高峰值 RSS，具体代价与逐阶段裁决见 `RUNTIME_ROUND3_WORKLOG_ZH.md`。前轮已回退时间边界、线性 component 物化和 Ouster angular-index 原型，历史证据见 `RUNTIME_OPTIMIZATION_ROUND2_20260911_ZH.md`。

前轮 42-bin 批处理在全量前后表中看似降低 OS1 mean 约 7%，但相邻 M2/OS1 的 B—A—B 为 258.81→257.32→258.45 ms，两次 B 均略慢；独立内核也变慢，因此已回退，并未将跨时段计时差异当作优化收益。

本轮进一步保留单线程 union-find 的已知根复用，以及同一 shell 跨历史 ray blocks 的几何 cell 覆盖复用。后者只缓存精确中心/半径/grid size 对应的 cell 坐标，不缓存 ray IDs 或占用状态；每个 block 仍独立去重、排序及执行精确相交判定。第一项相邻 M2 的 OS1 mean 减少约 2.68%，Mid 基本持平。第二项未配对完整矩阵虽然变慢，但恢复同一第一项版本复测也明显变慢；相邻 S1-far A→B 复测中 Mid mean/p95 减少约 4.71%/3.07%，OS1 的 0.15%/0.34% 仅视为持平，未确认代码引起的退化，故两项均保留。详细原始值与裁决见 `RUNTIME_ROUND4_WORKLOG_ZH.md`。

当前正式 Full 的 16 条结果统一取自本轮第二项完整矩阵，不挑选较快的控制复测拼表。未固定 CPU 亲和/频率且存在后台负载，阶段宏平均属于各次完整运行的观测值，不能直接作严格的因果加速比；本轮 38 次运行的六类跟踪产物均一致。

VoFOD 固定上游为 `7da9f33a878a586588f6a626b75cfeacac7824f7`，外部 tracker 为 `a92b4db61060b47f1af6dcce122188ec021f2dcd`。checked-ray 边界仅传合法状态、时间、原点、方向和可信长度；点更新先于检测，free-ray worker 等待后续 detection iteration 后提交，separate-background cleanup 由独立 0.1 s ROS-time timer 触发。Range 输入对零时间戳、越界值和 TF 失败均关闭。

## Ouster 主比较（已完成 {len(completed_ouster_scenes)}/8 个实例）

| 方法 | HOTA-3D-pos mean ± std | TP / FP / FN | Recall mean | RMSE mean | detector/node mean |
|---|---:|---:|---:|---:|---:|
| AeroCOVER-OS1 | {ous['AeroCOVER-OS1']['HOTA_mean']:.3f} ± {ous['AeroCOVER-OS1']['HOTA_std']:.3f} | {ous['AeroCOVER-OS1']['TP_sum']} / {ous['AeroCOVER-OS1']['FP_sum']} / {ous['AeroCOVER-OS1']['FN_sum']} | {ous['AeroCOVER-OS1']['Recall_mean']:.3f} | {ous['AeroCOVER-OS1']['RMSE_mean_m']:.3f} m | {ous['AeroCOVER-OS1']['runtime_mean_ms']:.1f} ms |
| VoFOD-Original-OS1 | {ous['VoFOD-Original-OS1']['HOTA_mean']:.3f} ± {ous['VoFOD-Original-OS1']['HOTA_std']:.3f} | {ous['VoFOD-Original-OS1']['TP_sum']} / {ous['VoFOD-Original-OS1']['FP_sum']} / {ous['VoFOD-Original-OS1']['FN_sum']} | {ous['VoFOD-Original-OS1']['Recall_mean']:.3f} | {ous['VoFOD-Original-OS1']['RMSE_mean_m']:.3f} m | {ous['VoFOD-Original-OS1']['runtime_mean_ms']:.1f} ms* |
| VoFOD-Mid360-Adapted-OS1 | {ous['VoFOD-Mid360-Adapted-OS1']['HOTA_mean']:.3f} ± {ous['VoFOD-Mid360-Adapted-OS1']['HOTA_std']:.3f} | {ous['VoFOD-Mid360-Adapted-OS1']['TP_sum']} / {ous['VoFOD-Mid360-Adapted-OS1']['FP_sum']} / {ous['VoFOD-Mid360-Adapted-OS1']['FN_sum']} | {ous['VoFOD-Mid360-Adapted-OS1']['Recall_mean']:.3f} | {ous['VoFOD-Mid360-Adapted-OS1']['RMSE_mean_m']:.3f} m | {ous['VoFOD-Mid360-Adapted-OS1']['runtime_mean_ms']:.1f} ms* |

S1-near/far 上 AeroCOVER 均为零 FP、零 FN；P01 为 {p01[('P01', 'AeroCOVER-OS1')]['TP_1m']}/{p01[('P01', 'AeroCOVER-OS1')]['FP_1m']}/{p01[('P01', 'AeroCOVER-OS1')]['FN_1m']}，S2-new 为 {p01[('S2_new', 'AeroCOVER-OS1')]['TP_1m']}/{p01[('S2_new', 'AeroCOVER-OS1')]['FP_1m']}/{p01[('S2_new', 'AeroCOVER-OS1')]['FN_1m']}。VoFOD 各条均在测距种子生效后才逐步 ready，合计 {ous['VoFOD-Original-OS1']['FP_sum']} FP、{ous['VoFOD-Original-OS1']['FN_sum']} FN。星号 runtime 是 VoFOD detector callback，不含独立 tracker；异步 ray/cleanup 与 tracker 已分别记录，当前不能伪装成可靠端到端和。

Ouster 已完成单目标场景的 HOTA 宏平均为 AeroCOVER {ous_single['AeroCOVER-OS1']['HOTA_mean']:.3f}、Original {ous_single['VoFOD-Original-OS1']['HOTA_mean']:.3f}、冻结适配 {ous_single['VoFOD-Mid360-Adapted-OS1']['HOTA_mean']:.3f}；两条三目标实例依次为 {ous_multi['AeroCOVER-OS1']['HOTA_mean']:.3f}、{ous_multi['VoFOD-Original-OS1']['HOTA_mean']:.3f}、{ous_multi['VoFOD-Mid360-Adapted-OS1']['HOTA_mean']:.3f}。M1/M2 的 AeroCOVER 分别为 {p01[('M1', 'AeroCOVER-OS1')]['TP_1m']}/{p01[('M1', 'AeroCOVER-OS1')]['FP_1m']}/{p01[('M1', 'AeroCOVER-OS1')]['FN_1m']} 与 {p01[('M2', 'AeroCOVER-OS1')]['TP_1m']}/{p01[('M2', 'AeroCOVER-OS1')]['FP_1m']}/{p01[('M2', 'AeroCOVER-OS1')]['FN_1m']}，均为零 IDSW、零 fragmentation。其平均节点耗时分别为 {p01[('M1', 'AeroCOVER-OS1')]['runtime_mean_ms']:.0f} ms 与 {p01[('M2', 'AeroCOVER-OS1')]['runtime_mean_ms']:.0f} ms，仍不能据此声称 10 Hz 实时。

MRS OS1-128 配置为 2048×128、10 Hz、360° GPU ray sensor；Gazebo 11 使用三个内部相机拼接 2π 水平视场。点云 `t=0`，属于同一时刻 snapshot 近似，不是硬件 rolling scan。AeroCOVER 保留全部 262,144 条 ray、0.50 s point history 和 0.50 s ray FIFO，仅裁掉 42 m 外历史点。逐 ray DDA 与 shell 预筛保留并行，精确连通默认单线程；其可选并发路径仍通过逐点 reference 检查，但本机 OS1 使用单线程更快。局部连接缓存与持久线程池为负收益，未保留。

## Mid-360 两方法（八场景宏平均）

| 方法 | HOTA mean ± std | Recall mean | TP / FP / FN | IDSW | ready |
|---|---:|---:|---:|---:|---:|
| AeroCOVER-Mid360 | {mid['AeroCOVER-Mid360']['HOTA_mean']:.3f} ± {mid['AeroCOVER-Mid360']['HOTA_std']:.3f} | {mid['AeroCOVER-Mid360']['Recall_mean']:.3f} | {mid['AeroCOVER-Mid360']['TP_sum']} / {mid['AeroCOVER-Mid360']['FP_sum']} / {mid['AeroCOVER-Mid360']['FN_sum']} | {mid['AeroCOVER-Mid360']['IDSW_sum']} | N/A |
| VoFOD-Mid360-Adapted | {mid['VoFOD-Mid360-Adapted']['HOTA_mean']:.3f} ± {mid['VoFOD-Mid360-Adapted']['HOTA_std']:.3f} | {mid['VoFOD-Mid360-Adapted']['Recall_mean']:.3f} | {mid['VoFOD-Mid360-Adapted']['TP_sum']} / {mid['VoFOD-Mid360-Adapted']['FP_sum']} / {mid['VoFOD-Mid360-Adapted']['FN_sum']} | {mid['VoFOD-Mid360-Adapted']['IDSW_sum']} | {mid['VoFOD-Mid360-Adapted']['ready_count']}/8 |

六条单目标的 HOTA 宏平均依次为 {mid_single['AeroCOVER-Mid360']['HOTA_mean']:.3f}、{mid_single['VoFOD-Mid360-Adapted']['HOTA_mean']:.3f}；两条三目标实例依次为 {mid_multi['AeroCOVER-Mid360']['HOTA_mean']:.3f}、{mid_multi['VoFOD-Mid360-Adapted']['HOTA_mean']:.3f}。多目标 n=2 只作诊断。

扩展后的 VoFOD 地图统一为 `x=[-20,42]`、`y=[-12,26]`、`z=[-3,13]`。P01/P02 开发比较中，保留原检测参数的适配方案为 `204/0/32`、宏 HOTA 0.832；单点聚类方案为 `231/89/5`、宏 HOTA 0.802；0.25 m 细体素方案为 `54/0/182`、宏 HOTA 0.217。因此只保留前者，两个负收益检测参数方案均未进入正式配置。

FOV 修正中 P02/S2-new/S3-new 的单目标整体提高 0.6 m，M2 observer 整体降低 0.4 m；M1 无需重录。Full 的 M1/M2 HOTA 为 {p01[('M1', 'AeroCOVER-Mid360')]['HOTA_3D_pos']:.3f}/{p01[('M2', 'AeroCOVER-Mid360')]['HOTA_3D_pos']:.3f}，Recall 均为 1.000，六个目标均成功出生。原始回波检查见 `raw_return_check.csv`，逐序列完整表见 `PER_SEQUENCE_ZH.md`。

## 核心消融（六场景宏平均）

| 版本 | HOTA mean ± std | Recall mean | FP sum | IDSW sum | mean runtime |
|---|---:|---:|---:|---:|---:|
"""
    for method in ABLATIONS:
        row = abl[method]
        summary += (f"| {method} | {row['HOTA_mean']:.3f} ± "
                    f"{row['HOTA_std']:.3f} | {row['Recall_mean']:.3f} | "
                    f"{row['FP_sum']} | {row['IDSW_sum']} | "
                    f"{row['runtime_mean_ms']:.1f} ms |\n")
    summary += f"""
核心消融仍使用原 S32 运行；固定六场景的 Full 在 S29 与 S32 下逐项完全相同。S29 的新增收益只发生在不属于核心消融六条输入的 S1-near。Full 已应用后续 runtime 优化，A1–A5 本轮未重跑，其运行时间不能与新版 Full 作为同实现基线的时间消融比较。

A1 表明跨帧背景联系对抑制误报和身份混乱重要；A2 在 P02/S2-new 把运动轨迹延展误当大背景，降低 Recall；A3 关闭 shell 后累计 {abl['AeroCOVER-A3']['FP_sum']} FP，直接支持自由空间证据贡献；A5 one-shot 增至 {abl['AeroCOVER-A5']['FP_sum']} FP 和 {abl['AeroCOVER-A5']['IDSW_sum']} IDSW。A4 与 Full 宏结果接近，只在部分建筑序列增加少量 FP，因此本证据包对 full-chord 的独立增益较弱，不能夸大。

## 图与复现

`figures/mechanism_st_shell.png` 展示 S2-new 近墙和 S3-new 森林序列中 ST 背景/残差 component 与 shell pass 的在线诊断；`figures/m1_crossing_tracks_xy.png` 使用 Ouster M1 和预测 ID 自身固定颜色展示三目标接近、交叉和分离；`figures/ablation_hota_fp.png` 汇总消融；`figures/recall_by_return_count.png` 按每帧有效返回数的传感器内三分位统计 frame/target Recall。最后一图是描述性分层，不是距离因果效应；Ouster 栏包含六个单目标和两个三目标实例。

```bash
# Ouster 建议逐场景运行，避免一次生成全部大包：
python3 src/soft_vofod_evaluation/scripts/run_paper_minimal.py --suite ouster-main --scene S2_new
python3 src/soft_vofod_evaluation/scripts/run_paper_minimal.py --suite mid360-core
python3 src/soft_vofod_evaluation/scripts/run_paper_minimal.py --suite mid360-baselines
python3 src/soft_vofod_evaluation/scripts/run_paper_minimal.py --suite summarize
```

## 配对与限制

两种传感器使用相同场景、seed 和预定义轨迹，但分别录制；CSV 全部标为 `same_scenario_seed_separate_record`，不能冒充同一物理状态。P01 的 HOTA 为：AeroCOVER-Mid360 {p01[('P01', 'AeroCOVER-Mid360')]['HOTA_3D_pos']:.3f}，AeroCOVER-OS1 {p01[('P01', 'AeroCOVER-OS1')]['HOTA_3D_pos']:.3f}，VoFOD-Original-OS1 {p01[('P01', 'VoFOD-Original-OS1')]['HOTA_3D_pos']:.3f}，冻结适配 OS1 {p01[('P01', 'VoFOD-Mid360-Adapted-OS1')]['HOTA_3D_pos']:.3f}。

Mid-360 正式回放为 1.00×；GPU Ouster 三方法均为 0.15×并通过 95% 完整性硬门槛。本结果不声称 Ouster 在线 10 Hz 实时性。

2026-09-11 清理已永久删除无引用的未完成 Git tmp_pack、17 条被替代且不在当前正式 manifest 源集合中的历史 bag，以及三天以前的 ROS 日志；16 条当前正式源、有效 Git 提交历史和历史指标均保留。删除明细与本轮源码回退位置见 `RUNTIME_ROUND3_WORKLOG_ZH.md`，此前迁移到 SU710 的部分旧包不再可恢复。

全部 HOTA 使用官方 TrackEval commit `12c8791b303e0a0b50f753af204249e622d0281a`，相似度尺度 2 m、19 个 0.05–0.95 门限；1 m TP/FP/FN 使用先最大匹配数、再最小距离的一对一匹配。各 run 保存时间戳、稳定 ID、每门限结果、初始化状态、配置和源码/二进制哈希。当前仍是仿真研究证据，不验证真实载荷、续航或硬件可靠性。
"""
    (root / "RESEARCH_SUMMARY.md").write_text(summary, encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--suite", required=True,
                        choices=("ouster-main", "mid360-core",
                                 "mid360-baselines", "summarize"))
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--force-record", action="store_true")
    parser.add_argument("--scene", choices=OUSTER_SCENES,
                        help="run one scene only; useful for large Ouster bags")
    # Native VoFOD waits at most 0.8 wall-seconds for a later detection update;
    # 0.15x keeps 10 Hz scans about 0.67 wall-seconds apart.
    parser.add_argument("--ouster-rate", type=float, default=0.15)
    parser.add_argument("--mid360-rate", type=float, default=1.0)
    arguments = parser.parse_args()
    root = arguments.root.resolve()
    if arguments.suite == "summarize":
        summarize(root)
        return
    scenes = OUSTER_SCENES if arguments.suite in (
        "ouster-main", "mid360-baselines") else PAIRED_SCENES
    if arguments.scene:
        if arguments.scene not in scenes:
            parser.error("selected scene is not part of this suite")
        scenes = (arguments.scene,)
    sensor = "ouster" if arguments.suite == "ouster-main" else "mid360"
    for scene in scenes:
        ensure_source(root, scene, sensor, arguments.dry_run,
                      arguments.force_record)
        if arguments.suite == "ouster-main":
            for method in OUSTER_METHODS:
                replay(root, scene, method, sensor, arguments.ouster_rate,
                       arguments.dry_run)
        elif arguments.suite == "mid360-core":
            for method in ABLATIONS:
                replay(root, scene, method, sensor, arguments.mid360_rate,
                       arguments.dry_run)
        else:
            for method in MID360_METHODS:
                replay(root, scene, method, sensor, arguments.mid360_rate,
                       arguments.dry_run)


if __name__ == "__main__":
    main()
