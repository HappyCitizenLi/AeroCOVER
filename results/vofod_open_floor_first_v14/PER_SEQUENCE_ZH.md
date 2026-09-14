# V14 保留实验结果

根据清理要求，仅展示本版本保留结果，不依赖已删除的历史基线。未重跑或重新评分；原始manifest、metrics与CSV保持不变。

| 版本/场景/方法 | 下界z / ready | TP/FP/FN | IDSW/Frag | Recall | runtime mean/p95 ms | 输出/输入帧 | 来源 |
|---|---|---|---|---:|---|---|---|
| V14/OPEN/VoFOD-Mid360 | -0.125 / 0.0001 | 1656/6/178 | 0/3 | 0.9029 | 13.575/20.543 | 917/917 | 独立回放 |

完整清理后索引与复现配置见 [总览](../retained_experiments/INDEX_ZH.md)。本版本误差审计见 [ERROR_AUDIT.json](ERROR_AUDIT.json)。
