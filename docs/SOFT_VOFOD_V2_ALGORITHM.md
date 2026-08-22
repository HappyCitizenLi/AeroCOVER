# SOFT-VoFOD V2 算法

状态：实现完成，仿真验证完成；生产配置为
`src/soft_vofod_mid360/config/soft_vofod_v2_canonical.yaml`。本文描述 B4；B1–B3 仅用于消融。

## 1. 输入、输出与边界

算法只消费共同的、经过严格配对检查的在线输入：

```text
/uav1/mid360/points_world
/uav1/mid360/rays_checked
```

核心与 ROS adapter 均不订阅 truth、scene name、target ID 或 visibility truth。truth 只由离线
evaluator 使用。节点输出 violation packets、tracks、free/background/candidate voxels、opportunity
debug 和 diagnostics。

世界状态被明确拆成两部分：

\[
M_t^B=\text{persistent background memory},\qquad
\mathcal T_t=\text{dynamic target trajectories}.
\]

新观测通过“低频空间持久性”和“轨迹一致性”竞争归属，不再让每条 ray 立即改变长期世界模型。

## 2. 单帧处理顺序

`SoftVofodCore::processScan()` 位于 `src/soft_vofod_mid360/src/core.cpp:2421`，一个 scan 的顺序为：

1. 清理上一帧已标记 deleting 的轨迹，并验证 ray 时间/几何合同；
2. 按 10 ms micro-batch 处理 ray-level 几何，但长期地图只在 5 Hz epoch 边界提交；
3. 从上一 epoch 形成 free-space violation、unresolved 和 track-explained packets；
4. 预测现有 CV 轨迹，使用 packet-level GNN/Hungarian 做一对一关联和 KF 更新；
5. 只有未关联 violation packet 进入 birth buffer，按至少五个独立时间组拟合轨迹；
6. 对每条轨迹从当前真实 rays 计算 scan opportunity 和检测概率；
7. 每 scan 只做一次 survival/hit/miss existence 更新；
8. 只让 confirmed track 产生强 map support、free-carving 截断和 quarantine；
9. 累积本帧 return/free evidence，留待后续 map epoch 决策。

核心原则是：

\[
\text{ray physics}\to\text{opportunity},\quad
\text{short-term packet}\to\text{observation},\quad
\text{spatial persistence}\to\text{background},\quad
\text{trajectory consistency}\to\text{target}.
\]

## 3. 状态机

地图 voxel 状态为 `unknown → confident_free/candidate_background → stable_background`。stable
background 是保守长期记忆，不会因单条 coarse ray 穿过同 voxel 就被擦除。

轨迹状态为 `tentative → confirmed → deleting`：

- birth existence 0.6；
- existence ≥ 0.8 确认；
- existence ≤ 0.1 删除；
- tentative 最大年龄/无测量时间为 1.0/0.3 s；
- confirmed 无测量上限 6.0 s；
- 30 s hard timeout 只是安全兜底。

## 4. 主要模块与代码位置

| 模块 | 入口 |
|---|---|
| 背景概率与状态 | `BackgroundMap::query/updateState`，`core.cpp:186/269` |
| cold-start candidate | `BackgroundMap::updateUnknownCandidate`，`core.cpp:423` |
| violation packet | `BackgroundMap::packetizeViolationComponent`，`core.cpp:545` |
| deferred map epoch | `BackgroundMap::advanceEpoch`，`core.cpp:650` |
| CV transition/Q | `SoftVofodCore::transition/processNoise`，`core.cpp:1055/1064` |
| existence | `missedExistence/survivalExistence/hitExistence`，`core.cpp:1082–1119` |
| GNN/Hungarian | `SoftVofodCore::hungarian`，`core.cpp:1120` |
| trajectory birth | `bestBirthCandidate/createBirth`，`core.cpp:1358/1554` |
| confirmed support index | `supports/indexSupports`，`core.cpp:1645/1693` |
| ray angular index | `indexRays/nearbyOpportunityRays`，`core.cpp:1782/1832` |
| exact opportunity | `opportunity/returnProbability`，`core.cpp:1907/2009` |
| ROS 参数与合同 | `src/soft_vofod_mid360/src/soft_vofod_node.cpp:176–329` |

所有结构和默认值声明在 `include/soft_vofod_mid360/core.h`；运行时权威值来自 canonical YAML，
不是 C++ header 的 fallback default。

## 5. 有界性

- raw event 不拥有长期 support；
- confirmed support 用 voxel index 查询，support 数量约为 O(track count)；
- opportunity 先用 angular bins 筛 ray，再做精确 ray-sphere/occlusion；
- birth buffer 固定 2 s/256 packets，每 1 m spatial cell/epoch 最多一个 birth；
- duplicate merge 要同时满足位置、Mahalanobis、速度和时间历史门限；
- singleton packet 合法，没有用 point-count 门槛隐藏远距单点目标。

正式 B4 全矩阵中，support/track peak 最大均为 7，runtime p95 最大 50.66 ms，所有 100 个 B4
run 均低于 100 ms 门槛。

## 6. 已证明与未证明

500-run 仿真矩阵证明 V1 的 event/support/runtime storm 已消失，并证明 packetization、trajectory
birth、opportunity/survival 对各自中间量有效。它没有证明 B4 全面超过 B0：目标场景宏平均 HOTA
为 B4 0.7535、B0 0.7582。S04/S05 和 unknown 中静止物仍有明确失败，详见
`SOFT_VOFOD_V2_FINAL_REPORT.md`。
