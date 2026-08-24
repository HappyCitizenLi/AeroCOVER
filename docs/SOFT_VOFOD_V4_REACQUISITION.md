# SOFT-VoFOD V4 Dormant Reacquisition

## 1. 实现

V4 增加显式状态：

```text
DORMANT -> PRE_REACTIVATED -> CONFIRMED_ACTIVE
```

第一条 compatible F/U packet 经 provenance、背景距离、reachable bound、Mahalanobis 和一对一
assignment 后更新旧 ID，但不可报告。第二条 ordinary packet 必须在 1.0 s 内到达；成功才 active，
失败返回 dormant。诊断记录 pre latency、reportable latency、false pre rate、second-packet failure 和
provenance。

## 2. S05 单因素结果

R0/R1 使用同一 seed 1005 source，且为使 dormant mechanism 可触发，两者都启用相同 opportunity
geometry；唯一差异是显式两阶段开关。

| metric | R0 one-stage | R1 two-stage |
|---|---:|---:|
| HOTA | 0.57735 | 0.57735 |
| FN / FP | 176 / 12 | 176 / 12 |
| IDSW / frag | 0 / 3 | 0 / 3 |
| ghost >3 s | 0 | 0 |
| correct/wrong reactivation | 1 / 0 | 1 / 0 |
| pre false rate | n/a | 0 |
| second-packet failure | n/a | 0 |
| visible→reportable | 1.6823 s | 1.8823 s |
| runtime p95 | 80.149 ms | 81.336 ms |

两阶段安全但未降低 FN/HOTA，且增加 0.20 s 报告延迟；S05 FN 也未达到约 150 的目标。因此实现保留，
canonical `two_stage_dormant_reactivation=false`。

## 3. Occlusion state 验证

truth-only evaluator 在 S05 得到 84 个 eligible track frames、4 个 truth-occluded frames，混淆矩阵
`TP=0, FP=0, FN=4, TN=80`，recall 0、false-occlusion rate 0。在线 OCCLUDED 没有被真实触发，不能
作为论文核心贡献。当前可辩护贡献是 bounded dormant memory 与 provenance-aware reacquisition。
