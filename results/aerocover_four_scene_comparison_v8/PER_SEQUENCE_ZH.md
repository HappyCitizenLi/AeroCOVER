# AeroCOVER-V8 保留实验结果

根据清理要求，仅展示本版本保留结果，不依赖已删除的历史基线。未重跑或重新评分；原始manifest、metrics与CSV保持不变。

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

完整清理后索引与复现配置见 [总览](../retained_experiments/INDEX_ZH.md)。本版本误差审计见 [ERROR_AUDIT.json](ERROR_AUDIT.json)。
