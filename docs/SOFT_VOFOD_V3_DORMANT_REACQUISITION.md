# SOFT-VoFOD V3 Dormant Reacquisition

## 1. 目标

V2 在 S05 已压低 long ghost，但 confirmed track 常因 miss/existence 被删除，目标重现后又等待完整
5-group birth，造成 FN 和 latency。V3 保持 output ghost control，同时把高质量旧 ID 留在不发布的
dormant memory 中。

## 2. 生命周期

- `CONFIRMED_ACTIVE -> OCCLUDED`：完整 emitted rays 在 predicted near range 前稳定命中前景，
  occlusion probability ≥0.6；
- `OCCLUDED -> DORMANT`：距最后测量超过 1 s；
- 低 existence 的 confirmed active 也可进入 dormant，而不是立即销毁；
- `DORMANT -> DELETING`：dormant 超过 10 s，或 30 s hard timeout；
- 合法 packet 通过 reactivation gate 后恢复相同 ID。

Occluded/dormant 不施加 ordinary opportunity miss penalty。Existence 仍按 survival 传播；reportability
随 stale time、position covariance 和 lifecycle state 衰减，dormant 永不普通发布。

## 3. Dormant state 的保守化

进入 dormant 时锚定最近连续两帧可报告的可靠 posterior，速度均值归零、速度 covariance 增长；IMM
同步重置为保守 CV/zero-acceleration CA。这样不会用遮挡前的 stale CA acceleration 把 reachable
region 推到墙后错误位置，也不会让孤立 edge hit 覆盖长期锚点。

## 4. Reactivation gate

候选必须同时满足：

- packet 不在 stable/candidate background exclusion band；
- F packet，或 compatible U packet；far U 还需独立 packet pair 通过 χ² motion gate；
- dormant posterior Mahalanobis `D² <= 16.266`；
- 相对 last measurement 满足 `v_max Δt + 0.5 a_max Δt² + target_radius`；
- 一对一 Hungarian 分配，且 packet 未被 active track 使用。

reactivation 恢复 old ID 和 active state，但该帧 `last_evidence=TRACK_REACTIVATION`，不会立即发布；必须
再获一次 ordinary packet 才可 reportable。这样 evaluator 的 reacquisition 是可验证的轨迹连续性，
不是瞬时 ghost。

## 5. R4 消融

四变体只切换 IMM/dormant：`S05_base`、`S05_IMM`、`S05_dormant`、
`S05_IMM+dormant`。核心指标除 HOTA/FP/FN/fragmentation/IDSW 外，还包含 dormant duration/count、
reactivation latency、correct/wrong reactivation、max stale 和 runtime。结果及 wall-lock/anchor 修正链见
`SOFT_VOFOD_V3_ABLATION.md`。
