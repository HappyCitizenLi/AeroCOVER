#!/usr/bin/env python3
"""Replay eight unchanged inputs with only the map's lower z moved."""
import argparse
import copy
import csv
import json
from pathlib import Path
import sys
import yaml
from run_state_time_pair import CASES, WORKSPACE, RUNNER, NEW
from run_benchmark import sha256
from run_paper_minimal import run

BASE = WORKSPACE/'results/vofod_state_time_pair_v9'
CONFIGS = WORKSPACE/'src/vofod_mid360/config/four_scene_suite'


def validate_config(scene, sensor):
    name = f'{scene}_{sensor}.yaml'
    old_path = BASE/'config_snapshot'/name
    old = yaml.safe_load(old_path.read_text())
    current = yaml.safe_load((CONFIGS/name).read_text())
    expected = copy.deepcopy(old)
    expected['operation_area']['center']['z'] = 8.6875
    expected['operation_area']['size']['z'] = 18.625
    assert current == expected, name
    assert current['voxel_map']['voxel_size'] == .25
    area = current['operation_area']
    lower = area['center']['z'] - area['size']['z']/2
    assert lower == -.625 and int((0-lower)/.25) == 2
    assert lower + (2+.5)*.25 == 0
    return CONFIGS/name


def report(root):
    lines = ['# VoFOD 地图第三层地面对照：8 项', '',
             '同一输入、同一新时间协议（schema 9），1.5 m 匹配，MT 悬停遮挡 ignore，不计算 HOTA。OPEN/MT 保持 V2，OFFICE/FOREST 保持当前参数。',
             '只改变地图 z 中心/尺寸：7.5/21 → 8.6875/18.625 m，下界 −3 → −0.625 m；体素 0.25 m，地面 z=0 位于第三层中央。xy 范围及名义上界 18 m 不变。',
             'ceil(size/voxel)+1 分配规则下，实际 z 上界从 18.25 变为 18.375 m（不含上界）；z 层数 85 → 76。ready 比例仍为 Mid-360 0.0001、OS1 0.15，没有执行提高 ready 的四项。',
             '本次同时改变了地下深度和网格相位，不将结果单独归因于某一个因素。两批异步回放也可能有运行差异。', '',
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
        trees.add(manifest['algorithm_tree_sha256'])
        assert manifest['metric_code_sha256'] == sha256(NEW)
        assert (directory/'output.bag').is_file()
        for label, d in [('旧下界', BASE/'runs'/scene/method), ('第三层中央', directory)]:
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
    lines += ['', '“曾 ready”仅表示至少一次进入 ready，不表示之后始终 ready。逐帧 ready 和地面 FP 审计见 [地图影响分析](MAP_EFFECTS_ZH.md)。', '',
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
