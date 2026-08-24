# SOFT-VoFOD V4 最终阶段报告（未冻结）

日期：2026-08-24。实现 HEAD：`ae1b4ff`（文档提交前）。结论：**V4 开发阶段修正已完成到 Phase 10
准备项，但 final algorithm gate 未通过，不能冻结、不能生成论文最终主表。**

## 1. Phase 状态

| Phase | 状态 | 结论 |
|---|---|---|
| 0 audit | 完成 | 冻结 V3/B0、公式、数据边界 |
| 1 effective cells | 完成 | bounded cells + soft geometry |
| 2 CAL/metrics | 完成 | Brier/NLL/ECE/reliability 可评估 |
| 3 strict R5 | 完成、失败 | O1/O2/O3 HOTA 均劣于 O0 |
| 4 sequential inference | 完成 | 无旧 D² fallback |
| 5 triple gate | 完成 | 明显提升，但 S08B=5.2 s 未过 `<5 s` |
| 6 two-stage dormant | 完成、失败 | safe，无收益，+0.20 s latency |
| 7 ray occlusion | 完成、失败 | truth recall=0，不作核心贡献 |
| 8 LOS covariance | 完成候选 | 单 seed 小收益，不够明确，关闭 |
| 9 cold-start | 协议完成、性能失败 | CS01–05 可运行；CS03最终0误确认；p95超限 |
| 10 HW record-only | recorder完成、采集阻塞 | 无物理数据/真值/授权 |
| 11 external baselines | 未完成 | 只有BL0，缺BL1/BL2 |
| 12 final freeze | 未通过 | 不冻结 |
| 13 final matrix | 未运行 | freeze/baseline前置条件失败 |
| 14 real offline | 未运行 | 无real bags |
| 15 flight | 未运行 | 无数据且不满足安全证据 |
| 16 docs | 完成 | 十份V4文档如实记录正/负结果 |

## 2. 关键实验结论

Opportunity O3 的 calibration 为 Brier/NLL/ECE `0.137974/0.432318/0.097680`，优于 O1/O2；但
strict R5 的 HOTA `0.454376` 对 O0 `0.655932`，FN `189.4` 对 `137.4`，paired Wilcoxon
`p=0.0009765625`。因此只保留 diagnostics。

Sequential U1 把 S08B TTFT `18.2→5.2 s`、HOTA `0.4971→0.9204`，同时 S08C false confirmation=0、
IT11 hover recall=1。它是 V4 最强正结果，但仍未满足 `<5 s` final gate。

R1 two-stage 在 S05 wrong reactivation=0，却不改善 HOTA/FN，并增加0.20 s report latency。Occlusion
truth recall=0。M1 在单个 S04 seed 上 FN `49→47`、HOTA `+0.00115`，证据不足。

Cold-start CS01–CS05 均可端到端运行。failure-driven 修正把同一 CS03 source 的 false confirmation
`4→1→0`；最终 CS03 不产生 active track。但初始 smoke 的 p95 为 106.8–174.2 ms，当前 CS03 为
120.7 ms；CS05 post-known→unknown recall 仅0.374。五个 runs 不在统一最终 commit，不能作为 final table。

## 3. 提示词要求的 20 个问题

1. **Opportunity 为什么在 V3 中造成 FN/HOTA 退化？** 相关 raw rays 被当独立试验，sphere 硬相交
   令 `P_D` 虚高，稀疏未返回被当强 miss，existence 过度下降。
2. **新 effective model 是否校准？** 在独立仿真 CAL10–14 上已校准和评价；真实 Mid360 尚未校准。
3. **Brier/NLL/ECE 是否改善？** O3 相对 O1/O2 三项均改善；这不等于 tracking gate 通过。
4. **新 opportunity 是否仍降低 FP/frag/ghost？** 是，O3 相对 O0 为 `35.9→12.2`、`10.6→3.6`、
   `0.823→0.323`。
5. **FN/HOTA 是否不再恶化？** 否；FN `+52.0`，HOTA `-0.201556`，显著退化。
6. **S08B TTFT 是否显著下降？** 是，18.2→5.2 s；但没有达到严格 `<5 s`。
7. **S08C 是否仍0 false confirmation？** 是；真正零预热 CS03 在最终因果修正后也为0。
8. **mapped-free hover 是否仍可快速 birth？** 是，IT11 recall=1，TTFT=0.797 s。
9. **S05 FN 是否接近/优于 B0？** 否；R0/R1 FN=176，未达到预设约150目标。
10. **two-stage 是否0 wrong reactivation？** 是；但没有性能收益，故关闭。
11. **ray-occlusion 是否被触发且有 precision/recall？** truth 有4帧 occlusion，在线TP=0、recall=0；否。
12. **LOS covariance 是否改善 S04？** 单 seed 有很小改善（FN -2、HOTA +0.00115），不足以进入 final。
13. **cold-start 是否可运行？** 是，CS01–05 均完成过端到端 smoke；但 runtime/continuity gates 失败，
    且需统一 current commit 重跑。
14. **真实 calibration 与仿真差异多大？** 未知；没有真实数据，不能估计。
15. **真实 opportunity calibration 是否成立？** 未验证。
16. **frozen V4 是否全面优于 B0？** 没有 frozen V4，也没有支持该结论的统一矩阵。
17. **对 external baselines 是否有统计优势？** 未知；BL1/BL2 尚未实现。
18. **runtime 是否仍 `<100 ms` p95？** 常规 phase 多数约80–96 ms；真正 cold-start 为
    106.8–174.2 ms，最终 CS03 120.7 ms，因此总体答案是否。
19. **哪些是主创新/工程支撑？** 可主张的候选创新是 certified background/free representation、
    epistemic sequential birth、opportunity-conditioned maintenance 的建模框架；packet split、IMM、Hungarian、
    recorder/evaluator 是工程支撑。由于 opportunity maintenance 未过 gate，论文主张必须进一步收缩。
20. **证据足以 T-RO/IJRR 投稿吗？** 不足。至少还缺真实 calibration/real tests、BL1/BL2、统一10-seed
    N0/N1/N2矩阵、cold runtime修复、S05和opportunity final gates。

## 4. 验证与可复现性

最终全工作区 build 为11/11 packages、无 warning/failure；串行全测为463 tests、0 errors、0 failures、
0 skipped，其中仿真包94/94、HW capability probe 15/15。核心 artifacts 均保留 metrics、
manifests、timeseries 和 logs；已删除的 bags 可由对应场景、noise、seed 重建，但新录数据必须重新 paired。

## 5. 下一次允许继续的顺序

1. 在 current commit 上重跑 CS01–CS05，并优化 map/output hot path 到 worst p95<100 ms；
2. 用 CAL-only evidence 将 S08B 稳定压到<5 s，而不破坏 S08C/hover；
3. 重新设计 opportunity miss likelihood，先过 strict R5；
4. 让 S05 FN 接近 B0，再决定 two-stage；
5. 采 HW-CAL01–05，冻结真实参数；
6. 实现 BL1/BL2；
7. 只有上述门全过，才冻结并运行统一 final matrix、real replay 和 flight validation。
