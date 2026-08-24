# SOFT-VoFOD V4 Opportunity Observation Model

## 1. V3 失效原因

V3 把每条 raw ray 当独立 Bernoulli trial，并把 target sphere 硬相交等同于实体被照明：

\[
P_D=\min(P_{cap},1-\prod_j(1-q_j)).
\]

Mid360 相邻 rays 强相关，raw count 会令 `P_D` 过快饱和；一次稀疏未返回随后被解释为强负证据，
Bernoulli miss update 过度降低 existence，直接增加 FN/fragmentation 并降低 HOTA。这不是调高 cap 能
修复的独立性错误。

## 2. V4 分层模型

每条候选 ray 先计算 soft sphere geometry weight，foreground 或已确认前景 track 遮挡的 ray 权重为
0。方向落入同一 angular cell 的相关 rays 合并，得到 bounded effective cells 和 angular coverage。

模型拆为：

\[
P_D=P_{illum}\,P_{return\mid illum},
\qquad
P_{illum}=1-\exp(-\lambda n_{eff}).
\]

`P_return|illum` 使用独立 CAL 的 range bins；实体 fill factor 防止 sphere intersection 等同于完整
目标照明。每个 debug sample 保存 raw opportunity、effective cells、angular coverage、`P_illum`、
`P_return|illum`、最终 `P_D` 和 matched outcome。

## 3. 校准结果

CAL10–CAL14 与测试场景分离。15 个 calibration runs 的 target-return reliability 指标为：

| variant | Brier | NLL | ECE | high-confidence miss |
|---|---:|---:|---:|---:|
| O1 raw geometry | 0.195970 | 0.824616 | 0.167771 | 151 |
| O2 effective、无 range return | 0.261285 | 0.716540 | 0.341558 | 34 |
| O3 effective + range return | **0.137974** | **0.432318** | **0.097680** | 35 |

O3 在 Brier/NLL/ECE 上优于 O1/O2，因此 effective model 本身已具备可评价的 calibration；但该
结论只适用于仿真参数。30 m+ 的真实 `p_return` 仍未标定，不能称为论文最终物理模型。

## 4. 严格 R5 门控

S01–S07 的 N0/N1、seeds 1001–1005 使用同源 paired replay，共 40 runs：

| variant | HOTA | FP | FN | frag | IDSW | stale >3 s |
|---|---:|---:|---:|---:|---:|---:|
| O0 no opportunity miss | **0.655932** | 35.9 | **137.4** | 10.6 | 0.0 | 0.822782 |
| O1 raw | 0.585220 | **10.0** | 174.1 | **3.0** | 0.0 | 0.322683 |
| O2 effective/no range | 0.408244 | 23.1 | 196.0 | 4.4 | 0.6 | 0.712681 |
| O3 effective/range | 0.454376 | 12.2 | 189.4 | 3.6 | 0.6 | 0.323082 |

O3 的 10/10 paired HOTA 均低于 O0，mean delta `-0.201556`，Wilcoxon
`p=0.0009765625`。它降低 FP/fragmentation/stale ghost，但 FN/HOTA 显著恶化，未通过 V4 final gate。
因此 canonical `opportunity_aware_existence=false`；effective model 仅输出 diagnostics，不更新
existence。

## 5. 结论

“校准有所改善”与“可安全用于 track miss update”是两个不同命题。V4 证明了前者，但严格否定了
后者。恢复该模块需要真实 range×azimuth×elevation 数据以及一个不会把低 return probability 当作
强 miss 的观测模型；不得在 S01–S07 上继续调参。
