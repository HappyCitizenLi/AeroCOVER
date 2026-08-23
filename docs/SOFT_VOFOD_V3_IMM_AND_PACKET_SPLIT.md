# SOFT-VoFOD V3 IMM 与 Track-conditioned Packet Split

## 1. S04 诊断假设

V2 的高 FP/FN/fragmentation、低 IDSW 表明主要损失是 measurement continuity，而不是身份交换。V3
分别检验两个候选原因：global component 合并多个目标，以及 CV prediction 在转向/加速段 gate miss。

## 2. Maintenance split

Birth 仍使用 global short-time packet。Maintenance 在预测后才处理 mixed packet：packet 内每个点只
分给 Mahalanobis distance 最小且通过 gate 的 predicted track，随后按子集重新计算 robust
measurement/covariance。所有权唯一、singleton 保留、未通过任何 gate 的点不被强分配；一个普通
单目标 packet 不做人工 split。

实现还记录 `track_conditioned_split_count/packets` 与 assigned/unassigned points。该计数区分“功能
存在”和“本场景确实被触发”，避免只凭总指标把收益归因给 split。

## 3. 两模式 IMM

CV 使用 6D `[p,v]`，CA 使用 9D `[p,v,a]`。每次预测执行：

1. 按转移矩阵做 mode probability propagation；
2. CV/CA 间映射并做均值/协方差 mixing；
3. 分别用 white-acceleration / white-jerk process model 预测；
4. 对同一 packet 分别做 KF update 和 Gaussian likelihood；
5. 归一化 mode probabilities，再对 `[p,v]` 做 moment matching。

birth 从 CV posterior 初始化，CA acceleration 为零、协方差 9 m²/s⁴。每步对协方差显式对称化，
测试覆盖概率有限、PSD 和 no-NaN。

## 4. 严格 R3 消融

```text
S04-base       split off, IMM off
S04-split      split on,  IMM off
S04-IMM        split off, IMM on
S04-split-IMM  split on,  IMM on
```

其余 canonical 参数完全相同，N0/N1 × seeds 1001–1005 共享 source bag。报告 HOTA、FP/FN、
fragmentation、IDSW、packet shortage、multi-truth packet、gate rejection 和实际 split count。因果结论
只能由 paired deltas 与模块触发计数共同给出，结果见 `SOFT_VOFOD_V3_ABLATION.md`。
