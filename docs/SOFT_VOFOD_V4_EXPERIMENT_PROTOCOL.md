# SOFT-VoFOD V4 实验协议

## 1. 数据分割与不变量

- CAL01–16 只用于标定/机制诊断；S01–S08C、CS01–05 和 NEG 为 test/controls。
- 每个 paired comparison 只录一次 source bag；所有 variants 回放相同 SHA-256 输入。
- source 记录 truth，但算法回放只提供 `/tf`、`/tf_static`、`points_world`、`rays_checked`。
- evaluator 要求 truth/track/timing frame coverage 均至少 95%。
- 不允许按 scene 修改算法参数，不允许把不同 commit 的 phase 数字拼成 final main table。

## 2. 已执行的 V4 phase artifacts

| phase | artifact | metrics runs | 状态 |
|---|---|---:|---|
| opportunity CAL | `artifacts/v4_opportunity_cal` | 15 | 完成 |
| opportunity strict R5 | `artifacts/v4_r5_opportunity` | 40 | 完成，gate fail |
| epistemic triple | `artifacts/v4_epistemic_gate` | 6 | 完成 |
| dormant R0/R1 | `artifacts/v4_reactivation_gate` | 2 | 完成，gate fail |
| LOS CAL16/S04 | `artifacts/v4_los_gate` | 4 | 单 seed，收益不明确 |
| cold-start smoke | `artifacts/v4_cold_start` | 5 | 可运行，非统一 final commit |

大体积 bags 可删除，metrics、manifest、timeseries 和 logs 保留；需要 paired 复验时必须重录一个新 source
group，不能假定相同 Gazebo seed 会产生 bit-identical bag。

## 3. Cold-start 协议

CS01–CS05 标记 `cold_start: true`，target 可在 t=0 存在，scoring 在首个稳定 sensor frame 即开始。
SOFT launch 对这些场景把 `initialization/warmup_duration_s` 置 0；非 cold scenes 保持 canonical 10 s。
B0 若内部抑制 detection，成本保留在 TTFT/FN 中。

指标包括 TTFT、false confirmation、candidate→stable latency、observed→certified latency、
known→unknown continuity、false packets/min。当前五个 smoke runs 来自多个修正 commit，只能用于发现
failure mode；只有 CS03 已在当前 provenance 修正 `c52c006` 上重放。

## 4. 统计协议

最终统计必须在冻结 commit 上运行 all final scenes × N0/N1/N2 × 10 seeds，报告 mean/std、median/IQR、
bootstrap 95% CI 和 paired tests，并对多重比较做校正。HOTA/IDF1 等越大越好；FN/FP/IDSW/frag/runtime
越小越好。单 seed M1 结果只能作为 engineering gate，不能作为统计显著性。

## 5. Final matrix 状态

Phase 12 freeze 未通过，Phase 11 baselines 缺失，真实参数也未标定。因此 Phase 13 统一矩阵、Phase 14
real offline replay 和 Phase 15 flight validation 均未启动。这是严格 phase gate 的结果，不是缺失数据被
默认为通过。
