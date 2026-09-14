#!/usr/bin/env python3
"""Replay eight unchanged inputs with only the map's lower z moved."""
import argparse
import copy
import csv
import json
from pathlib import Path
import sys
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'src/soft_vofod_evaluation/scripts'))
from run_state_time_pair import CASES, WORKSPACE, RUNNER, NEW
from run_benchmark import sha256
from run_paper_minimal import run

BASE = WORKSPACE/'results/vofod_map_floor_third_layer_v10'
CONFIGS = WORKSPACE/'src/vofod_mid360/config/four_scene_suite'


def validate_config(scene, sensor):
    name = f'{scene}_{sensor}.yaml'
    old_path = BASE/'config_snapshot'/name
    old = yaml.safe_load(old_path.read_text())
    current = yaml.safe_load((CONFIGS/name).read_text())
    expected = copy.deepcopy(old)
    expected['operation_area']['center']['z'] = 8.9375
    expected['operation_area']['size']['z'] = 18.125
    assert current == expected, name
    assert current['background']['sufficient_points_ratio'] == (.15 if sensor == 'os1' else .0001)
    assert current['voxel_map']['voxel_size'] == .25
    area = current['operation_area']
    lower = area['center']['z'] - area['size']['z']/2
    assert lower == -.125 and int((0-lower)/.25) == 0
    assert lower + .5*.25 == 0
    return CONFIGS/name


def report(root):
    lines = ['# VoFOD 地面第一层中央：四场景八项汇总', '',
             '同一输入、同一新时间协议（schema 9），1.5 m 匹配，MT 悬停遮挡 ignore，不计算 HOTA。OPEN/MT 保持 V2，OFFICE/FOREST 保持当前参数。',
             '只改变地图 z 中心/尺寸：8.6875/18.625 → 8.9375/18.125 m，下界 −0.625 → −0.125 m；体素 0.25 m，地面 z=0 位于第一层中央。xy、名义上界 18 m、实际接受上界 18.375 m 和网格相位均不变。',
             'z 层数 76→74；ready 为 Mid-360 0.0001、OS1 0.15。OPEN/Mid-360 复用 v14 已完成结果，其余七项新回放；没有重录源输入。八份默认配置及生成器已同步。',
             '旧基线为 v10 第三层中央结果，不使用 v11/v12 的 ready 调参结果。两次异步回放也可能有运行差异。', '',
             '| 场景 | 方法 | 地图 | TP/FP/FN | IDSW/Frag | Recall | 曾 ready | runtime mean/p95 ms | 输出/输入帧 |',
             '|---|---|---|---|---|---:|---|---|---|']
    trees = set()
    for scene, method in CASES:
        sensor = 'os1' if method.endswith('OS1') else 'mid360'
        config = validate_config(scene, sensor)
        directory = root/'runs'/scene/method
        manifest = json.loads((directory/'run_manifest.json').read_text())
        original = json.loads((BASE/'runs'/scene/method/'run_manifest.json').read_text())
        assert manifest['status'] == 'COMPLETE'
        assert manifest['config_sha256'] == sha256(config)
        assert original['config_sha256'] == sha256(BASE/'config_snapshot'/config.name)
        for key in ('source_sha256', 'source_manifest_sha256', 'scenario_sha256',
                    'algorithm_binary_sha256', 'metric_code_sha256',
                    'tracker_override_sha256', 'method_config_sha256'):
            assert manifest.get(key) == original.get(key), (scene, method, key)
        # Reused v14 ran with an external config, before defaults were updated.
        if (scene, method) != ('OPEN', 'VoFOD-Mid360'):
            trees.add(manifest['algorithm_tree_sha256'])
        assert manifest['metric_code_sha256'] == sha256(NEW)
        assert (directory/'output.bag').is_file()
        new_label = '第一层中央（复用v14）' if (scene, method) == ('OPEN', 'VoFOD-Mid360') else '第一层中央（新跑）'
        for label, d in [('第三层中央v10', BASE/'runs'/scene/method), (new_label, directory)]:
            m = json.loads((d/'metrics.json').read_text())
            assert m['coverage']['valid'] and not m['hota_computed']
            selected = [r for r in csv.DictReader((d/'publication_selection.csv').open())
                        if r['decision'] == 'selected']
            assert len(selected) == len({r['score_stamp'] for r in selected}) == m['track_frames']
            t, timing = m['track_set'], m['runtime']
            lines.append(f"| {scene} | {method} | {label} | {t['tp']}/{t['fp']}/{t['fn']} | "
                         f"{t['id_switches']}/{t['fragmentations']} | {t['recall']:.4f} | "
                         f"{m['initialization']['detection_ready']} | {timing['mean_ms']:.3f}/{timing['p95_ms']:.3f} | "
                         f"{m['track_frames']}/{m['source_input_frames']} |")
    assert len(trees) == 1
    lines += ['', '“曾 ready”仅表示至少一次进入 ready，不表示之后始终 ready。结果说明见 [README_ZH.md](README_ZH.md)。', '',
              'runtime 为 VoFOD 检测主体单帧实际计时，不含外部 tracker；不乘回放倍率。OS1 0.15×、Mid-360 0.25×回放。完整输出 bag 保留在 SU710。', '']
    (root/'PER_SEQUENCE_ZH.md').write_text('\n'.join(lines))
    print(root/'PER_SEQUENCE_ZH.md', flush=True)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('stage', choices=['validate', 'replay', 'report'])
    p.add_argument('--root', type=Path, required=True)
    args = p.parse_args()
    root = args.root.resolve()
    if args.stage == 'report':
        report(root)
        return
    for scene, method in CASES:
        sensor = 'os1' if method.endswith('OS1') else 'mid360'
        config = validate_config(scene, sensor)
        if args.stage == 'validate':
            print('PASS only map z changed:', config, flush=True)
            continue
        output = root/'runs'/scene/method
        manifest = output/'run_manifest.json'
        if manifest.exists():
            m = json.loads(manifest.read_text())
            assert m['status'] == 'COMPLETE', f'Inspect incomplete run: {output}'
            assert m['config_sha256'] == sha256(config)
            print('Already complete:', output, flush=True)
            continue
        run([sys.executable, RUNNER, 'replay', '--source', root/'sources'/scene/'source.bag',
             '--scenario', root/'sources'/scene/'scenario.yaml', '--algorithm', method,
             '--config', config, '--require-source-manifest', '--replay-rate',
             '.15' if sensor == 'os1' else '.25', '--output', output], False)
    if args.stage == 'replay':
        report(root)


if __name__ == '__main__':
    main()
