# V16 保留实验结果

根据清理要求，仅展示本版本保留结果，不依赖已删除的历史基线。未重跑或重新评分；原始manifest、metrics与CSV保持不变。

| 版本/场景/方法 | 下界z / ready | TP/FP/FN | IDSW/Frag | Recall | runtime mean/p95 ms | 输出/输入帧 | 来源 |
|---|---|---|---|---:|---|---|---|
| V16/OFFICE/VoFOD-OS1 | -0.125 / 0.01 | 588/49/907 | 7/5 | 0.3933 | 44.277/56.541 | 1495/1495 | 独立回放 |

完整清理后索引与复现配置见 [总览](../retained_experiments/INDEX_ZH.md)。本版本误差审计见 [ERROR_AUDIT.json](ERROR_AUDIT.json)。
