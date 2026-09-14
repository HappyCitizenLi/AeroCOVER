#!/usr/bin/env python3
"""Eight unchanged VoFOD replays; old/new selection on each identical output bag."""
import argparse
import csv
import json
import shutil
from pathlib import Path
import sys
from run_benchmark import sha256
from run_paper_minimal import run

WORKSPACE=Path(__file__).resolve().parents[3]
RUNNER=Path(__file__).with_name('run_benchmark.py')
OLD=Path(__file__).with_name('evaluate_bag_v8_reference.py')
NEW=Path(__file__).with_name('evaluate_bag.py')
V8=WORKSPACE/'results/aerocover_four_scene_comparison_v8'
CASES=[(scene,method) for scene in ('OPEN','MT','OFFICE','FOREST')
       for method in ('VoFOD-OS1','VoFOD-Mid360')]


def check_pair(directory):
    pair=json.loads((directory/'pair_manifest.json').read_text())
    assert pair['new_evaluator_sha256']==sha256(NEW)
    assert pair['old_evaluator_sha256']==sha256(OLD)
    assert pair['new_metrics_sha256']==sha256(directory/'metrics.json')
    assert pair['old_metrics_sha256']==sha256(directory/'old_protocol/metrics.json')
    assert (directory/'output.bag').is_file()
    return pair


def report(root):
    lines=['# VoFOD 同输出旧／新时间协议配对评估','',
           '仅8项新回放：OPEN/MT沿用V2，OFFICE/FOREST为当前参数，不跑其V2。每个新输出bag同时按旧／新协议评分；不是从已清理的v8旧输出恢复数据。',
           '匹配距离1.5 m、MT悬停LOS ignore和输入保持不变，HOTA不计算。新协议按实际状态时刻归属到原评分网格，同刻取推进到新状态前最后一个合法完整快照；不回填历史状态。',
           'Mid-360仍从点云header状态向前对齐至扫描终点，OS1快照无需这一偏移。没有额外固定延迟截止；可用顺序按记录的发布事件序列，接收时钟差仅作审计，不能声称零延迟在线评估。','',
           '| 场景 | 方法 | 协议 | TP/FP/FN | IDSW/Frag | Recall | RMSE/m | 输出帧/输入帧 |',
           '|---|---|---|---|---|---:|---:|---|']
    extras=[];trees={}
    fmt=lambda x:'—' if x is None else f'{x:.3f}'
    for scene,method in CASES:
        d=root/'runs'/scene/method;check_pair(d)
        manifest=json.loads((d/'run_manifest.json').read_text())
        assert manifest['status']=='COMPLETE'
        frozen_config=root/'config_snapshot'/Path(manifest['config']).name
        assert manifest['config_sha256']==sha256(frozen_config if frozen_config.exists() else Path(manifest['config']))
        assert manifest['source_manifest_sha256']==sha256(Path(manifest['source_manifest']))
        if manifest.get('tracker_override_config'):
            assert manifest['tracker_override_sha256']==sha256(Path(manifest['tracker_override_config']))
        original=json.loads((V8/'runs'/scene/method/'run_manifest.json').read_text())
        for key in ('algorithm_tree_sha256','algorithm_binary_sha256','source_sha256','config_sha256','tracker_override_sha256'):
            assert manifest.get(key)==original.get(key),(scene,method,key)
        trees.setdefault(method,set()).add(manifest['algorithm_binary_sha256'])
        new=json.loads((d/'metrics.json').read_text());old=json.loads((d/'old_protocol/metrics.json').read_text())
        assert new['source_input_frames']==old['source_input_frames']
        assert new['los_scoring']['ignored_truth_samples']==old['los_scoring']['ignored_truth_samples']
        for name,m in [('旧',old),('新',new)]:
            assert m['coverage']['valid'] and m['hota_computed'] is False
            t=m['track_set']
            lines.append(f"| {scene} | {method} | {name} | {t['tp']}/{t['fp']}/{t['fn']} | {t['id_switches']}/{t['fragmentations']} | {fmt(t['recall'])} | {fmt(t['position_RMSE_m'])} | {m['track_frames']}/{m['source_input_frames']} |")
        a=new['time_semantics'];timing=new['runtime'];delay=a['selected_receipt_clock_offset_s']
        extras.append(f"| {scene} | {method} | {a.get('state_time_remapped_publications',0)} | {a.get('same_state_replacements',0)} | {a.get('late_past_state_rejected',0)} | {fmt(delay['p95'])} | {fmt(timing['mean_ms'])}/{fmt(timing['p95_ms'])} |")
        selected=[r for r in csv.DictReader((d/'publication_selection.csv').open()) if r['decision']=='selected']
        assert len(selected)==len({r['score_stamp'] for r in selected})==new['track_frames']
    assert all(len(v)==1 for v in trees.values())
    lines+=['','## 发布选择及计时','',
            '| 场景 | 方法 | 状态重映射发布 | 同刻替换 | 旧状态迟到拒绝 | 记录时钟差p95/s | runtime mean/p95 ms |',
            '|---|---|---:|---:|---:|---:|---|']+extras+['',
            '时钟差是记录的ROS接收时刻减评分时刻，仅供审计，不能当作真实可用延迟（记录时钟可能滞后采集时钟）；可用顺序依据发布事件序列，不用这个差值丢帧。runtime为同一次回放的VoFOD检测主体计时，不含外部tracker等；旧／新评分共享这份runtime。','']
    (root/'PER_SEQUENCE_ZH.md').write_text('\n'.join(lines))
    print(root/'PER_SEQUENCE_ZH.md',flush=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('stage',choices=['pilot','replay','rescore','report']);p.add_argument('--root',type=Path,required=True)
    a=p.parse_args();root=a.root.resolve()
    assert sha256(OLD)==sha256(V8/'code_snapshot/src/soft_vofod_evaluation/scripts/evaluate_bag.py')
    if a.stage=='report':report(root);return
    for scene,method in CASES:
        if a.stage=='pilot' and (scene,method)!=('OPEN','VoFOD-OS1'):continue
        output=root/'runs'/scene/method
        if a.stage=='rescore' and not (output/'run_manifest.json').exists():continue
        if a.stage!='rescore' and (output/'pair_manifest.json').exists():check_pair(output);print('Paired already:',output,flush=True);continue
        sensor='os1' if method.endswith('OS1') else 'mid360'
        config=WORKSPACE/'src/vofod_mid360/config/four_scene_suite'/f'{scene}_{sensor}.yaml'
        manifest=output/'run_manifest.json'
        if not manifest.exists():
            run([sys.executable,RUNNER,'replay','--source',root/'sources'/scene/'source.bag',
                 '--scenario',root/'sources'/scene/'scenario.yaml','--algorithm',method,'--config',config,
                 '--require-source-manifest','--replay-rate','.15' if sensor=='os1' else '.25','--output',output],False)
        m=json.loads(manifest.read_text());assert m['status']=='COMPLETE',f'Inspect retained incomplete run: {output}'
        if m['metric_code_sha256']!=sha256(NEW):
            assert a.stage=='rescore','Use rescore explicitly for an evaluator revision'
            archive=output/'selection_v1_clock_guard';archive.mkdir()
            shutil.copy2(manifest,archive/'run_manifest.json')
            if (output/'pair_manifest.json').exists():shutil.copy2(output/'pair_manifest.json',archive/'pair_manifest.json')
            for f in list(output.glob('*.csv'))+[output/'metrics.json']:
                shutil.move(str(f),str(archive/f.name))
            run([sys.executable,NEW,'--source',m['source'],'--run',output/'output.bag',
                 '--algorithm',method,'--scenario',m['scenario'],'--output',output,
                 '--resource',output/'resource.txt','--seed',str(m['seed'])],False)
            m['evaluation_revision']=dict(previous_manifest=str(archive/'run_manifest.json'),
                                         reason='Receipt clock can lag acquisition; use publication event order, not a clock-subtraction gate')
            m['metric_code_sha256']=sha256(NEW);m['metrics_sha256']=sha256(output/'metrics.json')
            manifest.write_text(json.dumps(m,indent=2)+'\n')
        old_output=output/'old_protocol'
        if old_output.exists():
            old_data=json.loads((old_output/'metrics.json').read_text())
            assert old_data['evaluator_sha256']==sha256(OLD) and old_data['algorithm']==method and old_data['scenario']==scene
        else:
            run([sys.executable,OLD,'--source',m['source'],'--run',output/'output.bag',
                 '--algorithm',method,'--scenario',m['scenario'],'--output',old_output,
                 '--resource',output/'resource.txt','--seed',str(m['seed'])],False)
        assert sha256(output/'output.bag')==m['output_sha256'],'Output changed between evaluations'
        pair=dict(status='COMPLETE',same_output_bag=str(output/'output.bag'),output_sha256=m['output_sha256'],
                  old_evaluator_sha256=sha256(OLD),new_evaluator_sha256=sha256(NEW),
                  old_metrics_sha256=sha256(old_output/'metrics.json'),new_metrics_sha256=sha256(output/'metrics.json'))
        (output/'pair_manifest.json').write_text(json.dumps(pair,indent=2)+'\n')


if __name__=='__main__':main()
