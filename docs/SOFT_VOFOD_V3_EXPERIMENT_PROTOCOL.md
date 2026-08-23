# SOFT-VoFOD V3 实验协议

## 1. 不变纪律

- CAL 只选全局参数，test scene 不调参；
- 同一 scene/noise/seed 只录一次 source bag，所有 variants 1.0× replay；
- 算法不读取 truth；truth-conditioned diagnostics 只在 evaluator 生成；
- 每个有效 run 要求 `status=ok`、coverage contract 通过，并记录 git/config/implementation/source hash；
- failed/invalid run 不进入均值，不以零填充 None；
- 磁盘受限时只在 metrics/CSV/log/manifest 验证后删除大 bag，并在报告注明不可从仓库恢复。

## 2. 分阶段门禁

| Phase | 数据 | 决策 |
|---|---|---|
| R1 | CAL06–08、NEG05/06/03/04、S08A | false-free ≤0.18，static recall ≥0.12，expansion 保持 |
| R2 | S08B、S08C、IT11 hover | moving U birth、static U 0 confirm、F hover birth 同时通过 |
| R3 | S04 四 variants × N0/N1 × 5 | paired packet/IMM 因果，FP/FN/frag 同降，IDSW 不恶化 |
| R4 | S05 四 variants × N0/N1 × 5 | FN/reacq 改善且 wrong reacq/ghost 有界 |
| R5 | C0–C3，统一 5 birth groups | 严格 opportunity/survival/dormant 单因素链 |
| R6 | 仅 R1–R5 通过后：S01–S08C，B0+V3 × 5 | 小矩阵无 regression |
| R7 | 仅四问题都改善后扩 10 seeds | 统计扩展，不用于继续调参 |

## 3. Variant 合同

所有 V3 variants 读取同一 canonical YAML。runner 的唯一差异是显式 booleans：

- V3-A：certified/epistemic on，split/IMM/dormant off；
- V3-B：V3-A + split/IMM；
- V3-C：全部 on；
- C0：opportunity/survival/reportability/dormant off；
- C1：只增加 opportunity；
- C2：再增加 survival/reportability；
- C3：再增加 dormant。

所有 C0–C3 的 `birth_min_groups=5`。R3/R4 的 variants 也不改变 birth、map、opportunity 或 feedback。

## 4. 统计与复核

报告每 variant/noise 的 5-seed mean，同时给出同 source 的 paired delta、win count 和 worst seed；
不把 N/A 转零。R3/R4 还检查 diagnostics 触发计数，避免模块未触发却声称因果。Runtime 使用每 run
p95 和 worst p95，正式工程门限为 100 ms。

产物布局：

```text
artifacts/v3_r1/
artifacts/v3_r2/
artifacts/v3_r3/
artifacts/v3_r4/
artifacts/v3_r5/
```

每个目录保留 `metrics/{summary.csv,aggregate.json,ablation_deltas.csv}` 和
`runs/<algorithm>/<scene>/<noise>/seed_<seed>/` 的小型证据文件。大 bag 在完成 hash/coverage/metric
检查后删除。
