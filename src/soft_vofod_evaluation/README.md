# 当前四场景回放与评估

主矩阵：OPEN、MT、OFFICE、FOREST × Mid-360/OS1 × AeroCOVER/VoFOD，共16项。不存在当前70项/20项历史矩阵。

| 方法 | 配置 | 留存结果所用回放倍率 |
|---|---|---:|
| AeroCOVER-Mid360 | aerocover_mid360/config/aerocover_mvp.yaml | 1.0 |
| AeroCOVER-OS1 | aerocover_mid360/config/aerocover_os1_128.yaml（覆盖base） | 0.25 |
| VoFOD-Mid360 | vofod_mid360/config/four_scene_suite/场景_mid360.yaml | 0.25 |
| VoFOD-OS1 | vofod_mid360/config/four_scene_suite/场景_os1.yaml | 0.15 |

请明确指定场景配置；通用run_benchmark默认VoFOD配置是OPEN，不能直接拿来代表OFFICE/FOREST。A1–A5仅为当前AeroCOVER机制消融入口/回归配置，非额外历史算法版本，当前保留结果没有对应消融实验。

## 环境与入口

```bash
cd /home/uav/lyk
source /opt/ros/noetic/setup.bash
source /home/uav/lyk/devel/setup.bash
/usr/bin/python3 src/soft_vofod_evaluation/scripts/run_benchmark.py --help
/usr/bin/python3 src/soft_vofod_evaluation/scripts/evaluate_bag.py --help
```

- run_benchmark.py：单项record/replay/aggregate，支持源manifest、seed、配置和代码摘要、完整输出及覆盖率校验。新回放必须使用新output目录，不覆盖保留数据。
- run_four_scene_suite.py：四场景record/mask/replay/report自动流程，含源数据审计/用户批准摘要校验。mask阶段会更新校准配置，只有明确要重新校准时才运行；replay沿用原有清理输出bag策略，需保留完整bag时使用单项runner且不要传--cleanup-output-bag。
- evaluate_bag.py：当前评分器。保留实验报告请使用 results/retained_experiments/report.py，而非对已删除基线执行旧矩阵脚本。

## 当前评分协议

匹配距离1.5 m，主指标为TP/FP/FN、Precision/Recall、IDSW/Frag、IDF1及位置/速度误差。HOTA默认关闭；相关函数/requirements-hota.txt仅保留用于现有回归与格式兼容，不表示主实验执行HOTA。

评分网格由源扫描及采集偏移确定。AeroCOVER使用扫描末端状态；VoFOD按真实状态时刻、已用观测时间和发布事件顺序选择合法完整快照，同刻去重，不回填历史帧。接收时钟差只作审计，不能替代可用性顺序。MT悬停目标被柱遮挡时ignore：不计TP/FN，合法匹配预测不计FP。

runtime是原始记录的算法计时，不乘回放倍率。AeroCOVER包含输入整理/核心/内置tracker；VoFOD是检测主体，不含外部tracker完整耗时，二者不是统一端到端计时。

[完整报告](../../docs/AEROCOVER_VOFOD_CURRENT_FULL_REPORT_ZH.md) · [保留数据](../../results/retained_experiments/INDEX_ZH.md)。历史消息别名和指标字段在评估器中保留用于读取现有记录，生产评估计算文件本次不改。
