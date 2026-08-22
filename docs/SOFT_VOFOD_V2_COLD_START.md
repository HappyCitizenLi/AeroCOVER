# SOFT-VoFOD V2 Cold-start

## 1. 目标

proposed method 不要求 target-free warm-up。canonical 的 `initialization/warmup_duration_s` 为 0，
节点从第一帧开始同时维护：

- persistent background candidates；
- unresolved packets；
- trajectory-consistent target hypotheses。

固定 target-free warm-up 只保留为 B0 benchmark 控制条件，不能作为 B4 正确工作的前提。

## 2. unknown candidate

unknown component 以 world-frame centroid、voxel overlap 和 1 m match gate 跨 epoch 关联。候选
至少被 3 个 epoch、持续 1 s 观测后才晋升；优先只晋升达到逐 voxel 重复命中门槛的部分。若
没有重复 voxel，只有 centroid variance ≤0.15² m² 时才可整体晋升；1 s 未续观测则过期。

moving component 不会仅因单帧几何静止而写入 background。与 tentative track 重叠的 component
保持 weak/unresolved；confirmed track 才强保护。实现位于
`BackgroundMap::updateUnknownCandidate/advanceEpoch`（`src/soft_vofod_mid360/src/core.cpp:423/650`）。

## 3. CAL 证据

- CAL01 static map：修正为 per-voxel persistence 后，static recall 0.149，false confirmed 0；
- CAL02 moving observer/no target：expansion recall 0.974，birth/track 0；
- CAL03 将 birth group 3→5：cold-start birth 12→3、FP 88→56、HOTA 0.845→0.885；
- 所有选择在 CAL01–CAL05 完成，S01–S08 没有按场景调参。

## 4. S08 正式结果

每项均为 N0/N1 × 5 seeds 的 mean±sample SD。

| 场景 | B4 HOTA | FP | birth | confirmed IDs | expansion | candidate latency p95 |
|---|---:|---:|---:|---:|---:|---:|
| S08A no target | 0（不适用） | 0 | 0 | 0 | 0.609±0.013 | 15.67±0.97 s |
| S08B moving target | 0.981±0.010 | 0 | 1.5±0.53 | 1 | 0.766±0.014 | 场景地图统计 |
| S08C stationary object | 0.986±0.008 | 0 | 1.2±0.63 | 1 | 0.765±0.015 | 场景地图统计 |

S08A 证明 cold-start 可以在没有 fixed warm-up 的情况下保持 0 birth/track，并把新背景逐步纳入
地图；但 expansion 仅 0.609，false packets 仍为 12.53/min，尚非完全同化。

S08B 的 moving target 可以由 trajectory consistency 形成单轨。

S08C 是明确失败：10/10 run 都确认了一个 stationary track，而设计预期是保持
unresolved/candidate。HOTA 高只说明它和仿真 truth 位置一致，不说明分类语义正确；算法在仅 LiDAR
几何、无先验历史时仍不能可靠区分 unknown 中静止小物体与新静态背景。不得用该结果宣称解决了
静止 unknown object 辨识。

## 5. B0 warm-up 与 V2 readiness 的区别

B0 的 10 s warm-up 是 baseline 自身的 map maturity 合同。修复后的 runner 等待真实输入订阅，
manifest 记录 first-input ack、bootstrap start、complete 和 first scored 状态。S06/B0 正式 10 个
run 的 complete 为 14.045–14.362 s，首个 scored frame 均 `warmup_active=false`。

B4 没有这条 fixed-time 要求；其安全性来自 deferred assimilation 与候选/轨迹竞争，而不是把
target spawn 推迟到一个任意秒数之后。
