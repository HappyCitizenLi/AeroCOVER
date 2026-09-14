#!/usr/bin/env python3
"""OPEN Mid360: ground centered in layer one, ready ratio unchanged."""
import csv
import json
from pathlib import Path
import sys
import yaml

ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parents[1]
sys.path.insert(0, str(WORKSPACE/'src/soft_vofod_evaluation/scripts'))
from run_benchmark import sha256
from run_paper_minimal import run

BASE = ROOT.parent/'vofod_map_floor_third_layer_v10'
CASES = [('OPEN', 'mid360', .0001)]
REPORT = ['# OPEN/VoFOD-Mid360：地面第一层中央对照', '',
          '仅新跑 OPEN/Mid-360 一项，ready 保持 0.0001。本次下界 −0.125 m，center.z=8.9375 m，size.z=18.125 m；地面位于第一层 [−0.125,0.125) 中央。表中并列第三层、第二层的历史结果，不重复运行。体素 0.25 m、xy、名义上界 18 m、实际接受上界 18.375 m 及网格相位不变。其他参数、输入、新时间协议、1.5 m 匹配不变；HOTA 不计算。试验配置独立保存，不修改默认配置。', '',
          '| 场景 | 传感器 | 下界/m | TP/FP/FN | IDSW/Frag | Recall | ready帧/评分帧 | 首次ready/s | runtime mean/p95 ms |',
          '|---|---|---:|---|---|---:|---|---:|---|']


def main():
    validate_only = '--validate' in sys.argv
    for scene, sensor, ratio in CASES:
        method = 'VoFOD-OS1' if sensor == 'os1' else 'VoFOD-Mid360'
        config = ROOT/'configs'/f'{scene}_{sensor}.yaml'
        old_config = BASE/'config_snapshot'/config.name
        expected = yaml.safe_load(old_config.read_text())
        old_ratio = expected['background']['sufficient_points_ratio']
        expected['background']['sufficient_points_ratio'] = ratio
        expected['operation_area']['center']['z'] = 8.9375
        expected['operation_area']['size']['z'] = 18.125
        assert ratio == old_ratio == .0001
        assert expected['voxel_map']['voxel_size'] == .25
        lower = expected['operation_area']['center']['z'] - expected['operation_area']['size']['z']/2
        assert lower == -.125 and lower + .5*.25 == 0
        assert yaml.safe_load(config.read_text()) == expected, config
        print('PASS only map z changed; ground centered in layer one:', config, flush=True)
        if validate_only:
            continue
        directory = ROOT/'runs'/scene/method
        manifest_path = directory/'run_manifest.json'
        if not manifest_path.exists():
            run([sys.executable, WORKSPACE/'src/soft_vofod_evaluation/scripts/run_benchmark.py',
                 'replay', '--source', BASE/'sources'/scene/'source.bag',
                 '--scenario', BASE/'sources'/scene/'scenario.yaml', '--algorithm', method,
                 '--config', config, '--require-source-manifest', '--replay-rate',
                 '.15' if sensor == 'os1' else '.25', '--output', directory], False)
        manifest = json.loads(manifest_path.read_text())
        original = json.loads((BASE/'runs'/scene/method/'run_manifest.json').read_text())
        assert manifest['status'] == 'COMPLETE'
        assert manifest['config_sha256'] == sha256(config)
        assert original['config_sha256'] == sha256(old_config)
        for key in ('source_sha256', 'source_manifest_sha256', 'scenario_sha256',
                    'algorithm_tree_sha256', 'algorithm_binary_sha256', 'metric_code_sha256',
                    'method_config_sha256', 'tracker_override_sha256'):
            assert manifest.get(key) == original.get(key), (scene, key)
        original_metrics = json.loads((BASE/'runs'/scene/method/'metrics.json').read_text())
        second = ROOT.parent/'vofod_open_floor_second_v13'/'runs'/scene/method
        for d, value in [(BASE/'runs'/scene/method, -.625), (second, -.375), (directory, -.125)]:
            m = json.loads((d/'metrics.json').read_text())
            assert m['coverage']['valid'] and not m['hota_computed']
            assert m['source_input_frames'] == original_metrics['source_input_frames']
            assert m['los_scoring']['ignored_truth_samples'] == original_metrics['los_scoring']['ignored_truth_samples']
            selected = [r for r in csv.DictReader((d/'publication_selection.csv').open()) if r['decision'] == 'selected']
            assert len(selected) == len({r['score_stamp'] for r in selected}) == m['track_frames']
            diagnostics = list(csv.DictReader((d/'diagnostics_timeseries.csv').open()))
            ready = sum(float(r['detection_ready']) > .5 for r in diagnostics)
            t, timing = m['track_set'], m['runtime']
            first = m['initialization']['detection_ready_stamp_s']
            first_text = '—' if first is None else f'{first:.3f}'
            REPORT.append(f"| {scene} | {sensor} | {value} | {t['tp']}/{t['fp']}/{t['fn']} | "
                          f"{t['id_switches']}/{t['fragmentations']} | {t['recall']:.4f} | {ready}/{len(diagnostics)} | "
                          f"{first_text} | {timing['mean_ms']:.3f}/{timing['p95_ms']:.3f} |")
    if not validate_only:
        REPORT.extend(['', '状态：COMPLETE，1/1。runtime 为检测主体单帧计时，不含外部 tracker，不乘回放倍率。Mid-360 0.25×回放，完整输出保留在 SU710。两次异步回放并非确定性重复，不能将所有差异都归因于参数。', '',
                       '结论和误差审计见 [README_ZH.md](README_ZH.md)。', ''])
        (ROOT/'PER_SEQUENCE_ZH.md').write_text('\n'.join(REPORT))
        print(ROOT/'PER_SEQUENCE_ZH.md', flush=True)


if __name__ == '__main__':
    main()
