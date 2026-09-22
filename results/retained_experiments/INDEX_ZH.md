# 清理后保留的实验结果

保留 AeroCOVER-V8 的四场景×两传感器8项，以及 VoFOD V13–V16。共19条逻辑记录、18次独立回放；V15/OPEN/Mid-360复用V14，不重复计数。
其余历史实验按已核验清单清理；正式Git历史和当前源码不做重写。四个必要源bag及保留运行数据已迁至 /data/lyk，各保留目录直接链接至新位置。原始manifest中的SU710路径仅记录运行时出处；当前路径以 retained_index.json 和 sources 链接为准。

注意：8项AeroCOVER的output.bag在本次清理前就已不存在。本次完整保留其指标、CSV、日志、资源记录、manifest、配置和代码快照；VoFOD保留项的output.bag均保留。

| 版本/场景/方法 | 下界z / ready | TP/FP/FN | IDSW/Frag | Recall | runtime mean/p95 ms | 输出/输入帧 | 来源 |
|---|---|---|---|---:|---|---|---|
| AeroCOVER-V8/FOREST/AeroCOVER-Mid360 | — | 1795/0/0 | 0/0 | 1.0000 | 32.479/66.592 | 1795/1795 | 独立回放 |
| AeroCOVER-V8/FOREST/AeroCOVER-OS1 | — | 1934/0/0 | 0/0 | 1.0000 | 86.512/109.792 | 1934/1934 | 独立回放 |
| AeroCOVER-V8/MT/AeroCOVER-Mid360 | — | 1110/8/149 | 4/4 | 0.8817 | 40.084/53.637 | 500/500 | 独立回放 |
| AeroCOVER-V8/MT/AeroCOVER-OS1 | — | 1282/0/41 | 3/3 | 0.9690 | 106.691/124.299 | 526/526 | 独立回放 |
| AeroCOVER-V8/OFFICE/AeroCOVER-Mid360 | — | 1476/0/0 | 0/0 | 1.0000 | 21.780/32.634 | 1476/1476 | 独立回放 |
| AeroCOVER-V8/OFFICE/AeroCOVER-OS1 | — | 1495/0/0 | 0/0 | 1.0000 | 85.346/94.407 | 1495/1495 | 独立回放 |
| AeroCOVER-V8/OPEN/AeroCOVER-Mid360 | — | 1834/0/0 | 0/0 | 1.0000 | 45.394/71.317 | 917/917 | 独立回放 |
| AeroCOVER-V8/OPEN/AeroCOVER-OS1 | — | 2044/0/0 | 0/0 | 1.0000 | 103.889/122.124 | 1022/1022 | 独立回放 |
| V13/OPEN/VoFOD-Mid360 | -0.375 / 0.0001 | 1656/5180/178 | 0/3 | 0.9029 | 13.861/21.129 | 917/917 | 独立回放 |
| V14/OPEN/VoFOD-Mid360 | -0.125 / 0.0001 | 1656/6/178 | 0/3 | 0.9029 | 13.575/20.543 | 917/917 | 独立回放 |
| V15/FOREST/VoFOD-Mid360 | -0.125 / 0.0001 | 1744/3966/51 | 0/0 | 0.9716 | 58.649/98.101 | 1795/1795 | 独立回放 |
| V15/FOREST/VoFOD-OS1 | -0.125 / 0.15 | 1554/154/380 | 5/5 | 0.8035 | 139.804/276.791 | 1934/1934 | 独立回放 |
| V15/MT/VoFOD-Mid360 | -0.125 / 0.0001 | 979/106/280 | 4/7 | 0.7776 | 22.398/33.977 | 500/500 | 独立回放 |
| V15/MT/VoFOD-OS1 | -0.125 / 0.15 | 1060/0/263 | 3/3 | 0.8012 | 170.659/237.154 | 526/526 | 独立回放 |
| V15/OFFICE/VoFOD-Mid360 | -0.125 / 0.0001 | 1425/127/51 | 0/0 | 0.9654 | 26.862/35.978 | 1476/1476 | 独立回放 |
| V15/OFFICE/VoFOD-OS1 | -0.125 / 0.15 | 70/0/1425 | 1/1 | 0.0468 | 44.068/55.095 | 1495/1495 | 独立回放 |
| V15/OPEN/VoFOD-Mid360 | -0.125 / 0.0001 | 1656/6/178 | 0/3 | 0.9029 | 13.575/20.543 | 917/917 | V14/OPEN/VoFOD-Mid360 |
| V15/OPEN/VoFOD-OS1 | -0.125 / 0.15 | 1974/0/70 | 0/0 | 0.9658 | 152.495/164.321 | 1022/1022 | 独立回放 |
| V16/OFFICE/VoFOD-OS1 | -0.125 / 0.01 | 588/49/907 | 7/5 | 0.3933 | 44.277/56.541 | 1495/1495 | 独立回放 |

runtime沿用各次原始评估口径；VoFOD为检测主体计时，不含外部tracker，不乘回放倍率。清理没有重跑或重新评分，未更改原始run_manifest/metrics/CSV。

## 保留目录与复现配置

- [AeroCOVER-V8](../aerocover_four_scene_comparison_v8/PER_SEQUENCE_ZH.md)
- [V13](../vofod_open_floor_second_v13/PER_SEQUENCE_ZH.md)
- [V14](../vofod_open_floor_first_v14/PER_SEQUENCE_ZH.md)
- [V15](../vofod_floor_first_all_v15/PER_SEQUENCE_ZH.md)
- [V16](../vofod_office_os1_ready001_v16/PER_SEQUENCE_ZH.md)

## 清理结果

已永久删除清单中的67个目标，合计释放约153.65 GiB。未保留回收站副本，不能通过本次操作直接撤销；若无其他备份，已删实验数据无法恢复。

Git只存在master，未删除分支、未重写正式提交历史；仅删除6个确认未使用的临时pack。清理后Git连通性检查通过，git garbage=0。

保留的18次独立回放所有原始运行文件及四个源目录在清理前后保持一致：小文件SHA逐字节核验，大bag核对原路径、大小和mtime；原manifest和metrics未改。保留目录无失效符号链接。

- /home/uav/lyk：释放约3.27 GiB。
- /media/uav/SU710：释放约150.38 GiB。

每项配置按原manifest SHA保存于 [configs](configs)，映射与文件完整性记录见 [retained_index.json](retained_index.json)。四个源数据入口为 [sources](sources)。

配置与source实际路径均可供现有src/soft_vofod_evaluation/scripts/run_benchmark.py创建新的独立回放；请指定新的output目录，不覆盖保留结果。

旧的保留版本replay.py已改成只检查/汇总现有结果的兼容入口，不再依赖已删V10对照。原脚本与原说明仅作为历史出处保存在各目录code_snapshot/pre_cleanup_reports，不作为当前执行入口。

删除清单见 [deletion_plan.json](deletion_plan.json)，执行结果见 deletion_completed.json。此目录报告可用 `python3 results/retained_experiments/report.py` 重新生成，不启动任何实验。
