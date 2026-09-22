#!/usr/bin/env python3
"""Report only the user-retained experiments, without deleted baselines."""
import csv
import json
from pathlib import Path
import sys
import yaml
from cleanup import ROOT, VERSIONS, read, verify


def main(group=None, validate_only=False):
    index=verify()
    if validate_only:return
    records=[];lines=['# 清理后保留的实验结果', '',
        '保留 AeroCOVER-V8 的四场景×两传感器8项，以及 VoFOD V13–V16。共19条逻辑记录、18次独立回放；V15/OPEN/Mid-360复用V14，不重复计数。',
        '其余历史实验按已核验清单清理；正式Git历史和当前源码不做重写。四个必要源bag及保留运行数据已迁至 /data/lyk，各保留目录直接链接至新位置。原始manifest中的SU710路径仅记录运行时出处；当前路径以 retained_index.json 和 sources 链接为准。', '',
        '注意：8项AeroCOVER的output.bag在本次清理前就已不存在。本次完整保留其指标、CSV、日志、资源记录、manifest、配置和代码快照；VoFOD保留项的output.bag均保留。', '',
        '| 版本/场景/方法 | 下界z / ready | TP/FP/FN | IDSW/Frag | Recall | runtime mean/p95 ms | 输出/输入帧 | 来源 |',
        '|---|---|---|---|---:|---|---|---|']
    group_rows={v:[] for v in VERSIONS}
    for e in index['entries']:
        d=Path(e['run_dir']);m=read(d/'metrics.json');t=m['track_set'];rt=m['runtime']
        identity=f"{e['version']}/{e['scene']}/{e['method']}"
        reused=e['owner']!=identity
        cfg=yaml.safe_load(Path(e['config']).read_text())
        area=cfg.get('operation_area') if e['method'].startswith('VoFOD-') else None
        lower=area['center']['z']-area['size']['z']/2 if area else None
        ratio=cfg['background']['sufficient_points_ratio'] if area else None
        params='—' if area is None else f'{lower} / {ratio}'
        row=dict(id=identity,owner=e['owner'],reused=reused,config=e['config'],
                 map_lower_z_m=lower,ready_ratio=ratio,
                 tp=t['tp'],fp=t['fp'],fn=t['fn'],idsw=t['id_switches'],frag=t['fragmentations'],
                 recall=t['recall'],mean_ms=rt['mean_ms'],p95_ms=rt['p95_ms'],
                 track_frames=m['track_frames'],input_frames=m['source_input_frames'],
                 output_bag_exists=e['output_bag_existed'],run_dir=str(d))
        records.append(row)
        text=(f"| {identity} | {params} | {row['tp']}/{row['fp']}/{row['fn']} | {row['idsw']}/{row['frag']} | "
              f"{row['recall']:.4f} | {row['mean_ms']:.3f}/{row['p95_ms']:.3f} | "
              f"{row['track_frames']}/{row['input_frames']} | {e['owner'] if reused else '独立回放'} |")
        lines.append(text);group_rows[e['version']].append(text)
    lines += ['', 'runtime沿用各次原始评估口径；VoFOD为检测主体计时，不含外部tracker，不乘回放倍率。清理没有重跑或重新评分，未更改原始run_manifest/metrics/CSV。', '',
              '## 保留目录与复现配置', '']
    for v,name in VERSIONS.items():
        lines.append(f'- [{v}](../{name}/PER_SEQUENCE_ZH.md)')
    completed=ROOT/'deletion_completed.json'
    if completed.exists():
        cleanup=read(completed)
        freed={p:(cleanup['after'][p]['free']-before['free'])/2**30 for p,before in cleanup['before'].items()}
        lines += ['', '## 清理结果', '',
                  f"已永久删除清单中的{len(cleanup['deleted'])}个目标，合计释放约{sum(freed.values()):.2f} GiB。未保留回收站副本，不能通过本次操作直接撤销；若无其他备份，已删实验数据无法恢复。", '',
                  'Git只存在master，未删除分支、未重写正式提交历史；仅删除6个确认未使用的临时pack。清理后Git连通性检查通过，git garbage=0。', '',
                  '保留的18次独立回放所有原始运行文件及四个源目录在清理前后保持一致：小文件SHA逐字节核验，大bag核对原路径、大小和mtime；原manifest和metrics未改。保留目录无失效符号链接。', '']
        for p,value in freed.items():lines.append(f'- {p}：释放约{value:.2f} GiB。')
    lines += ['', '每项配置按原manifest SHA保存于 [configs](configs)，映射与文件完整性记录见 [retained_index.json](retained_index.json)。四个源数据入口为 [sources](sources)。', '',
              '配置与source实际路径均可供现有src/soft_vofod_evaluation/scripts/run_benchmark.py创建新的独立回放；请指定新的output目录，不覆盖保留结果。', '',
              '旧的保留版本replay.py已改成只检查/汇总现有结果的兼容入口，不再依赖已删V10对照。原脚本与原说明仅作为历史出处保存在各目录code_snapshot/pre_cleanup_reports，不作为当前执行入口。', '',
              '删除清单见 [deletion_plan.json](deletion_plan.json)，执行结果见 deletion_completed.json。此目录报告可用 `python3 results/retained_experiments/report.py` 重新生成，不启动任何实验。', '']
    if group is None:
        (ROOT/'INDEX_ZH.md').write_text('\n'.join(lines))
        with (ROOT/'RETAINED_EXPERIMENTS.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)
    for v,name in VERSIONS.items():
        if group and v!=group:continue
        root=ROOT.parent/name
        header=[f'# {v} 保留实验结果', '',
                '根据清理要求，仅展示本版本保留结果，不依赖已删除的历史基线。未重跑或重新评分；原始manifest、metrics与CSV保持不变。', '',
                '| 版本/场景/方法 | 下界z / ready | TP/FP/FN | IDSW/Frag | Recall | runtime mean/p95 ms | 输出/输入帧 | 来源 |',
                '|---|---|---|---|---:|---|---|---|']
        header += group_rows[v]+['','完整清理后索引与复现配置见 [总览](../retained_experiments/INDEX_ZH.md)。本版本误差审计见 [ERROR_AUDIT.json](ERROR_AUDIT.json)。','']
        (root/'PER_SEQUENCE_ZH.md').write_text('\n'.join(header))
        (root/'README_ZH.md').write_text(f'# {v} 保留结果\n\n本目录结果已保留，旧基线依赖已解除。\n\n- [指标与runtime](PER_SEQUENCE_ZH.md)\n- [保留总览与配置](../retained_experiments/INDEX_ZH.md)\n- [误差审计](ERROR_AUDIT.json)\n\n源数据为本目录sources下的直接链接；原始输出及指标保留。AeroCOVER的output.bag在本次清理前已被历史流程清理，未恢复也未再次删除。\n\n原说明/脚本保存在code_snapshot/pre_cleanup_reports，仅作历史出处，其中已删除基线的路径不再有效。当前replay.py只检查或重建保留结果报告，不启动新回放。\n')
    print(ROOT/'INDEX_ZH.md')


if __name__=='__main__':main(validate_only='--validate' in sys.argv)
