#!/usr/bin/env python3
"""Four-scene 20-run matrix: V2 VoFOD for OPEN/MT, paired variants elsewhere."""
import argparse
import json
from pathlib import Path
import sys

from run_benchmark import sha256
from run_paper_minimal import run

WORKSPACE=Path(__file__).resolve().parents[3]
RUNNER=Path(__file__).with_name('run_benchmark.py')
SCENES=('OPEN','MT','OFFICE','FOREST')


def matrix():
    rows=[]
    for scene in SCENES:
        for method in ('AeroCOVER-OS1','AeroCOVER-Mid360','VoFOD-Mid360','VoFOD-OS1'):
            sensor='os1' if method.endswith('OS1') else 'mid360'
            config=(WORKSPACE/'src/aerocover_mid360/config'/('aerocover_os1_128.yaml' if sensor=='os1' else 'aerocover_mvp.yaml')
                    if method.startswith('AeroCOVER') else
                    WORKSPACE/'src/vofod_mid360/config/four_scene_suite'/f'{scene}_{sensor}.yaml')
            rows.append((scene,method,method,config,None))
        if scene in ('OFFICE','FOREST'):
            for sensor in ('mid360','os1'):
                method='VoFOD-OS1' if sensor=='os1' else 'VoFOD-Mid360'
                rows.append((scene,method+'-V2',method,
                             WORKSPACE/'src/vofod_mid360/config/four_scene_suite'/f'{scene}_{sensor}_v2.yaml',
                             WORKSPACE/'src/lidar_tracker_mid360/config/tracking_open_v2.yaml'))
    return rows


def report(root):
    lines=['# 四场景 20 组指标与 runtime','',
           '主匹配距离 1.5 m；HOTA 按用户要求不计算。MT 两个悬停目标非静态 LOS 时 ignore：不计 TP/FN，仅与遮挡目标一对一匹配的预测不计 FP，可见目标匹配优先。',
           'AeroCOVER 计时含输入整理、检测及内部跟踪，不含发布；VoFOD 为检测主体计时，不含输出构建/发布、异步 ray worker 及外部 tracker。均不乘回放倍率，不可当作同边界端到端耗时。','',
           '| 场景 | 方法／配置 | TP/FP/FN | IDSW/Frag | Recall | RMSE/m | mean/p95 ms | ready | 完成帧/输入帧 |',
           '|---|---|---|---|---:|---:|---|---|---|']
    fingerprints={};warnings=[]
    for scene,label,method,config,override in matrix():
        directory=root/'runs'/scene/label
        manifest=json.loads((directory/'run_manifest.json').read_text())
        assert manifest['status']=='COMPLETE',directory
        data=json.loads((directory/'metrics.json').read_text())
        assert data['coverage']['valid'],directory
        assert data['hota_computed'] is False and data['main_matching_threshold_m']==1.5
        assert manifest['metrics_sha256']==sha256(directory/'metrics.json')
        assert manifest['metric_code_sha256']==sha256(RUNNER.with_name('evaluate_bag.py'))
        assert manifest['config_sha256']==sha256(config)
        assert manifest['source_manifest_sha256']==sha256(Path(manifest['source_manifest']))
        source=json.loads(Path(manifest['source_manifest']).read_text())
        assert manifest['source_sha256']==source['source_bag_sha256']
        assert manifest['scenario_sha256']==source['scenario_sha256']==sha256(Path(manifest['scenario']))
        if manifest.get('tracker_override_config'):
            assert manifest['tracker_override_sha256']==sha256(Path(manifest['tracker_override_config']))
        fingerprints.setdefault(method,set()).add((manifest['algorithm_tree_sha256'],manifest['algorithm_binary_sha256']))
        if method.startswith('AeroCOVER'):
            assert data['aerocover']['future_stamp_violation_count']==0
            assert data['aerocover']['self_support_violation_count']==0
        track=data['track_set'];timing=data['runtime'];ready=data.get('initialization',{}).get('detection_ready')
        fmt=lambda value:'—' if value is None else f'{value:.3f}'
        ready_label='不适用' if ready is None else '是' if ready else '否'
        variant=('V2（Mid最少点数1）' if scene in ('OPEN','MT') and method.startswith('VoFOD') else '当前' if label==method and method.startswith('VoFOD') else '')
        lines.append(f"| {scene} | {label} {variant} | {track['tp']}/{track['fp']}/{track['fn']} | {track['id_switches']}/{track['fragmentations']} | {fmt(track['recall'])} | {fmt(track['position_RMSE_m'])} | {fmt(timing['mean_ms'])}/{fmt(timing['p95_ms'])} | {ready_label} | {data['time_semantics']['completed_source_frames']}/{data['source_input_frames']} |")
        if any(data['coverage'][k]<1 for k in ('track_frame_ratio','timing_frame_ratio','completion_frame_ratio')):
            warnings.append(f"{scene}/{label} 未达100%覆盖：{data['coverage']}。")
        if data['time_semantics']['future_state_publications_rejected']:
            warnings.append(f"{scene}/{label} 排除 {data['time_semantics']['future_state_publications_rejected']} 条携带未来状态的迟到发布，未倒推填帧。")
    assert all(len(v)==1 for v in fingerprints.values()),fingerprints
    lines+=['','## 完整性与说明','']+warnings+['','配置、输入、实飞检查及与历史版本区别见 README_ZH.md。','']
    (root/'PER_SEQUENCE_ZH.md').write_text('\n'.join(lines))
    print(root/'PER_SEQUENCE_ZH.md')


def main():
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['replay','os1-check','report']);p.add_argument('--root',type=Path,required=True)
    a=p.parse_args();root=a.root.resolve()
    if a.stage=='report':report(root);return
    evidence=json.loads((root/'body_mask_manifest.json').read_text())
    assert evidence['mask_sha256']==sha256(WORKSPACE/'src/vofod_mid360/config/sensors/ouster_os1_128_1024.yaml')
    for scene in SCENES:
        assert json.loads((root/'sources'/scene/'flight_audit.json').read_text())['status']=='PASS'
        assert evidence['sources'][scene]==sha256(root/'sources'/scene/'source_manifest.json')
    for scene,label,method,config,override in matrix():
        if a.stage=='os1-check' and not (scene in ('OPEN','FOREST') and method=='AeroCOVER-OS1'):continue
        output=root/'runs'/scene/label;manifest=output/'run_manifest.json'
        if manifest.exists():
            assert json.loads(manifest.read_text())['status']=='COMPLETE',f'Inspect incomplete attempt: {output}'
            print('Already complete:',output,flush=True);continue
        rate='1.0' if method=='AeroCOVER-Mid360' else '.15' if method=='VoFOD-OS1' else '.25'
        command=[sys.executable,RUNNER,'replay','--source',root/'sources'/scene/'source.bag',
                 '--scenario',root/'sources'/scene/'scenario.yaml','--algorithm',method,'--config',config,
                 '--require-source-manifest','--cleanup-output-bag','--replay-rate',rate,'--output',output]
        if override:command+=['--tracker-override',override]
        run(command,False)
        if scene in ('OPEN','FOREST') and method=='AeroCOVER-OS1':
            data=json.loads((output/'metrics.json').read_text())
            assert all(data['coverage'][k]==1 for k in ('track_frame_ratio','timing_frame_ratio','completion_frame_ratio')), 'OS1 repair requires every scored source frame'
            assert data['track_set']['fragmentations']==0,'OS1 Frag remains; inspect before proceeding'


if __name__=='__main__':main()
