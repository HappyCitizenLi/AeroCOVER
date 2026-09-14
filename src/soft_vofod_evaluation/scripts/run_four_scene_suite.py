#!/usr/bin/env python3
"""One validated paired source; two algorithms × two sensors per scene."""
import argparse
import json
from pathlib import Path
import sys

import yaml

from run_benchmark import sha256
from run_paper_minimal import run

WORKSPACE = Path(__file__).resolve().parents[3]
SIM = WORKSPACE/'src/mid360_multi_uav_sim'
CONFIG = WORKSPACE/'src/vofod_mid360/config'
SCENES = ('OPEN', 'MT', 'OFFICE', 'FOREST')
METHODS = ('AeroCOVER-Mid360', 'VoFOD-Mid360', 'AeroCOVER-OS1', 'VoFOD-OS1')
RUNNER = Path(__file__).with_name('run_benchmark.py')
sys.path.insert(0, str(SIM/'scripts'))
from audit_four_scene_source import audit


def require_accepted_source(root, name, report):
    if report['status'] == 'PASS':
        return
    path=root/'accepted_sources.json'
    accepted=json.loads(path.read_text()).get(name,{}) if path.exists() else {}
    source=root/'sources'/name
    if (accepted.get('status')!='ACCEPTED_BY_USER' or
            accepted.get('source_manifest_sha256')!=sha256(source/'source_manifest.json') or
            accepted.get('flight_audit_sha256')!=sha256(source/'flight_audit.json')):
        raise RuntimeError(f'{name}: measured trajectory has not passed or has no matching user acceptance')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('stage', choices=('record', 'mask', 'replay', 'report'))
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--aerocover-only', action='store_true')
    arguments = parser.parse_args()
    methods = tuple(m for m in METHODS if not arguments.aerocover_only or m.startswith('AeroCOVER'))
    root = arguments.root.resolve()
    if arguments.stage == 'report':
        write_report(root, methods)
        return
    if arguments.stage == 'record':
        for name in SCENES:
            directory = root/'sources'/name
            scenario = SIM/'config/benchmarks'/f'{name}.yaml'
            if (directory/'source_manifest.json').exists():
                expected = yaml.safe_load(scenario.read_text())
                expected.update(seed=12001, ray_time_geometry_mode='rolling_scene')
                if yaml.safe_load((directory/'scenario.yaml').read_text()) != expected:
                    raise RuntimeError(f'{name}: existing source has a different trajectory')
            run([sys.executable, RUNNER, 'record', '--scenario', scenario,
                 '--seed', '12001', '--noise', 'N03', '--sensor', 'paired', '--output', directory], False)
            report = audit(directory)
            require_accepted_source(root,name,report)
        return
    reports = {}
    for name in SCENES:
        path = root/'sources'/name/'flight_audit.json'
        report = json.loads(path.read_text())
        require_accepted_source(root,name,report)
        reports[name] = report
    mask_path = CONFIG/'sensors/ouster_os1_128_1024.yaml'
    if arguments.stage == 'mask':
        prior=yaml.safe_load(mask_path.read_text())['body_mask']
        prior_indices={i for first,last in prior.get('blocked_pattern_ranges',[]) for i in range(first,last+1)}
        prior_indices.update(prior.get('blocked_pattern_indices',[]))
        indices = sorted(prior_indices.union(*(set(r['body_mask_indices']) for r in reports.values())))
        if not indices:
            raise RuntimeError('no self-return evidence; inspect the raw source')
        ranges = []
        for index in indices:
            if ranges and index == ranges[-1][1]+1:
                ranges[-1][1] = index
            else:
                ranges.append([index, index])
        config = yaml.safe_load(mask_path.read_text())
        config['body_mask']['blocked_pattern_ranges'] = ranges
        mask_path.write_text('# Generated from all four validated raw sources: self returns 0.10–0.60 m; no target truth.\n'+
                             yaml.safe_dump(config, sort_keys=False))
        evidence = dict(pattern_count=len(indices), ranges=len(ranges), mask_sha256=sha256(mask_path),
                        retained_prior_calibration=True,
                        sources={name: sha256(root/'sources'/name/'source_manifest.json') for name in SCENES})
        (root/'body_mask_manifest.json').write_text(json.dumps(evidence, indent=2)+'\n')
        print(json.dumps(evidence, indent=2), flush=True)
        return
    evidence = json.loads((root/'body_mask_manifest.json').read_text())
    assert evidence['mask_sha256'] == sha256(mask_path), 'mask changed after calibration'
    for name in SCENES:
        source = root/'sources'/name
        assert evidence['sources'][name] == sha256(source/'source_manifest.json'), 'source changed after calibration'
        for algorithm in methods:
            sensor = 'os1' if algorithm.endswith('OS1') else 'mid360'
            # The wide-map VoFOD publication/tracker chain needs headroom;
            # sensor stamps and algorithm configuration remain unchanged.
            rate = ('0.15' if algorithm == 'VoFOD-OS1' else
                    '0.25' if sensor == 'os1' or algorithm == 'VoFOD-Mid360' else '1.0')
            config = (CONFIG/'four_scene_suite'/f'{name}_{sensor}.yaml' if algorithm.startswith('VoFOD') else
                      WORKSPACE/'src/aerocover_mid360/config'/('aerocover_os1_128.yaml' if sensor == 'os1' else 'aerocover_mvp.yaml'))
            output = root/'runs'/name/algorithm
            manifest = output/'run_manifest.json'
            if manifest.exists():
                if json.loads(manifest.read_text())['status'] == 'COMPLETE':
                    print('Already complete, not replaying:', output, flush=True)
                    continue
                raise RuntimeError(f'Inspect incomplete one-shot run before retrying: {output}')
            run([sys.executable, RUNNER, 'replay', '--source', source/'source.bag',
                 '--scenario', source/'scenario.yaml', '--algorithm', algorithm,
                 '--config', config,
                 '--require-source-manifest', '--cleanup-output-bag', '--replay-rate', rate,
                 '--output', output], False)


def write_report(root, methods=METHODS):
    from evaluate_bag import MAIN_THRESHOLD_M, COMPUTE_HOTA
    for algorithm in methods:
        trees, binaries = set(), set()
        for name in SCENES:
            directory = root/'runs'/name/algorithm
            manifest = json.loads((directory/'run_manifest.json').read_text())
            if manifest['status'] != 'COMPLETE':
                continue
            assert sha256(directory/'metrics.json') == manifest['metrics_sha256'], directory
            assert sha256(Path(manifest['config'])) == manifest['config_sha256'], directory
            if manifest.get('tracker_override_config'):
                assert sha256(Path(manifest['tracker_override_config'])) == manifest['tracker_override_sha256'], directory
            assert sha256(Path(manifest['source_manifest'])) == manifest['source_manifest_sha256'], directory
            source = json.loads(Path(manifest['source_manifest']).read_text())
            assert manifest['source_sha256'] == source['source_bag_sha256'], directory
            assert sha256(Path(manifest['scenario'])) == manifest['scenario_sha256'] == source['scenario_sha256'], directory
            assert manifest['metric_code_sha256'] == sha256(RUNNER.with_name('evaluate_bag.py')), directory
            data = json.loads((directory/'metrics.json').read_text())
            assert data['coverage']['valid'], directory
            if algorithm.startswith('AeroCOVER'):
                for key in ('future_stamp_violation_count', 'self_support_violation_count'):
                    assert data['aerocover'][key] == 0, (directory, key)
            trees.add(manifest['algorithm_tree_sha256'])
            binaries.add(manifest['algorithm_binary_sha256'])
        assert len(trees) <= 1 and len(binaries) <= 1, f'{algorithm}: implementation changed within matrix'
    lines = ['# 四场景逐序列指标与 runtime', '',
             f'四场景；每个场景共享同一次 paired 输入，有效结果每方法一项，共 {len(SCENES)*len(methods)} 项。输入采集版本及异常尝试见 README_ZH.md。', '',
             f'TP/FP/FN、IDSW/Frag 使用 {MAIN_THRESHOLD_M:g} m 匹配；'+
             ('HOTA 使用既有 TrackEval 3D-pos 协议。' if COMPUTE_HOTA else '本批次暂停 HOTA 计算，表内以 — 表示，非零分。')+
             '1 m 及其他敏感性指标保存在各运行 metrics.json 中。',
             '本版按源扫描起点选择评分帧；以采集偏移得到扫描终点，各方法状态按 last_prediction 向前对齐到同一终点，真值按里程计插值。覆盖率按 source_header 精确对应，缺帧不再由邻近帧补齐；本版与旧时间口径数值不能直接混用。',
             'AeroCOVER runtime 包含输入整理、核心检测与内部跟踪，不含输出发布；VoFOD 为检测主体 total_ms，在 publishOutputs 前结束，不含输出构建/发布、异步 ray worker 和外部 tracker。两者计时边界不同，均不是端到端耗时，也不乘回放倍率。', '',
             '| 场景 | 方法 | HOTA | TP/FP/FN | IDSW/Frag | 位置 RMSE/m | mean/p95 ms | ready | 计时帧/输入帧 | 回放倍率 |',
             '|---|---|---:|---|---|---:|---|---|---|---:|']
    warnings = []
    def fmt(value):
        return '—' if value is None else f'{value:.3f}'
    for name in SCENES:
        for algorithm in methods:
            directory = root/'runs'/name/algorithm
            manifest = json.loads((directory/'run_manifest.json').read_text())
            if manifest['status'] != 'COMPLETE':
                data_path = directory/'metrics.json'
                data = json.loads(data_path.read_text()) if data_path.exists() else {}
                count = data.get('runtime', {}).get('samples', '—')
                total = data.get('source_input_frames', '—')
                lines.append(f"| {name} | {algorithm}（未通过） | — | — | — | — | — | "
                             f"{data.get('initialization', {}).get('detection_ready', '—')} | "
                             f"{count}/{total} | {manifest['replay_rate']}× |")
                warnings.append(f"{name}/{algorithm} 状态为 {manifest['status']}，不能计入有效主结果；"
                                "诊断数值保存在对应 metrics.json，未重跑算法。")
                continue
            data = json.loads((directory/'metrics.json').read_text())
            track, timing = data['track_set'], data['runtime']
            rejected = data.get('time_semantics', {}).get('future_state_publications_rejected', 0)
            if rejected:
                warnings.append(f'{name}/{algorithm} 拒绝了 {rejected} 条携带后续帧状态的迟到发布；未用未来状态倒推填补源帧。')
            ready = data.get('initialization', {}).get('detection_ready')
            ready_label = '不适用' if ready is None else '已就绪' if ready else '未就绪'
            lines.append(f"| {name} | {algorithm} | {fmt(track['HOTA'])} | "
                         f"{track['tp']}/{track['fp']}/{track['fn']} | "
                         f"{track['id_switches']}/{track['fragmentations']} | "
                         f"{fmt(track['position_RMSE_m'])} | {fmt(timing['mean_ms'])}/{fmt(timing['p95_ms'])} | "
                         f"{ready_label} | "
                         f"{timing['samples']}/{data['source_input_frames']} | {manifest['replay_rate']}× |")
            if any(data['coverage'][key] != 1.0 for key in
                   ('source_truth_ratio', 'timing_frame_ratio', 'track_frame_ratio')):
                warnings.append(f"注意：{name}/{algorithm} 的计时帧覆盖率为 "
                                f"{100*data['coverage']['timing_frame_ratio']:.4f}%，跟踪帧覆盖率为 "
                                f"{100*data['coverage']['track_frame_ratio']:.4f}%；"
                                "请结合该运行 metrics.json 的 coverage 字段解释。")
            completion = data['coverage'].get('completion_frame_ratio', 1.)
            if completion != 1.:
                warnings.append(f'{name}/{algorithm} 的源帧处理完成率为 {completion:.4%}。')
    lines += ['', '\n\n'.join(warnings), '', '## 实际飞行几何', '',
              '| 场景 | 最小机体净空/m | 扣除采样运动余量/m | 目标距离/m | 中心俯仰角/° | 静态 LOS | 近障碍时刻占比 | 最长互遮挡/s |',
              '|---|---:|---:|---|---|---|---:|---:|']
    for name in SCENES:
        sources = {}
        for algorithm in methods:
            manifest = json.loads((root/'runs'/name/algorithm/'run_manifest.json').read_text())
            sources.setdefault(Path(manifest['source']).parent, []).append(algorithm.replace('VoFOD-', ''))
        for directory, sensors in sources.items():
            audit = json.loads((directory/'flight_audit.json').read_text())
            label = name if len(sources) == 1 else f"{name}（{'/'.join(sensors)}）"
            lines.append(f"| {label} | {audit['minimum_body_clearance_m']:.3f} | "
                     f"{audit['clearance_with_sampling_margin_m']:.3f} | "
                     f"{audit['minimum_range_m']:.2f}–{audit['maximum_range_m']:.2f} | "
                     f"{audit['minimum_elevation_deg']:.2f}–{audit['maximum_elevation_deg']:.2f} | "
                     f"{'通过' if audit['static_los_failures'] == 0 else '允许的圆柱遮挡' if audit['unallowed_static_los_failures'] == 0 else '失败'} | "
                     f"{audit['target_close_pass_fraction']:.1%} | {audit['longest_moving_mutual_occlusion_s']:.2f} |")
    lines += ['', '## 时长与运动范围', '',
              '| 场景 | 时长/s | 计划最大速度/(m/s) | 实飞最大速度/(m/s) | 计划最大加速度/(m/s²) |',
              '|---|---:|---:|---:|---:|']
    for name in SCENES:
        source = root/'sources'/name
        scenario = yaml.safe_load((source/'scenario.yaml').read_text())
        planned = json.loads((source/'source_manifest.json').read_text())['physical_validation']
        measured = json.loads((source/'flight_audit.json').read_text())
        lines.append(f"| {name} | {scenario['duration_s']:.1f} | {planned['maximum_speed_mps']:.3f} | "
                     f"{measured['maximum_speed_mps']:.3f} | {planned['maximum_acceleration_mps2']:.3f} |")
    lines += ['', '上述最大值统计 observer 与全部目标；实际加速度未在本次实飞审计中估计，不能将计划加速度当实测值。',
              '规划与实飞几何检查的采样周期以各 source_manifest / flight_audit 为准。OFFICE/FOREST 优先满足门洞、树干净空与 FOV；FOREST 间距基本为 4.5 m，未覆盖到 20 m。']
    mt = json.loads((root/'sources/MT/flight_audit.json').read_text())
    intervals = '\n'.join(f"- {identity}："+'；'.join(
        f'{begin:.2f}–{end:.2f} s' for begin, end in groups)+'。'
        for identity, groups in mt['static_center_los_intervals_s'].items())
    lines += ['', '近障碍指目标机体包络外缘距离非地面静态碰撞几何小于 1 m；所有检查覆盖完整场景时间。互遮挡列仅统计运动目标之间。',
              'MT 允许圆柱遮挡悬停目标；这些时段仍纳入跟踪评分，不删除 FN。圆柱遮挡区间（场景起点后秒）：',
              '', intervals, '',
              '配置、GUI 指令及与旧版本的差异见本目录 README_ZH.md。', '']
    acceptance=root/'accepted_sources.json'
    if acceptance.exists():
        lines+=['## 用户接受的场景例外','']
        for name,item in json.loads(acceptance.read_text()).items():
            lines.append(f"- {name}：{item['reason']}；原始严格审计状态保留，接受仅绑定本次输入及审计摘要。")
    content = '\n'.join(lines)
    (root/'PER_SEQUENCE_ZH.md').write_text(content)
    local = WORKSPACE/'results'/root.name/'PER_SEQUENCE_ZH.md'
    local.parent.mkdir(parents=True, exist_ok=True)
    local.write_text(content)
    print(local, flush=True)


if __name__ == '__main__':
    main()
