# SOFT-VoFOD V3 已知限制

## 1. 信息论边界

下面两种情况仅凭当前 LiDAR geometry 无法无条件区分：

```text
unknown space + never previously observed + stationary object
  = new static background OR hovering UAV
```

V3 的正确行为是 unresolved/candidate，不是强制 target 或 background。只有此前 certified-free 的
占用证据、后续独立运动、外部语义或其他传感器才能打破歧义。因此 S08C 的 0 confirmation 是保守
认识论结果，不等于证明物体是背景。

## 2. 仿真与标定限制

- certified/free、motion χ²、IMM、dormant 和 reportability 参数来自当前仿真 CAL 或 engineering
  defaults，不是实机统计保证；
- 30 m+ return probability 仍为 0.5 fallback，尚无 range×angle 实机标定；
- surface margin 是固定 0.75 m，没有 per-point covariance/入射角自适应；
- 当前正式因果矩阵只有 5 seeds；未通过前置门禁时不会扩 10 seeds；
- 同一 seed 重录的 Gazebo source 并非逐 bit 确定；只允许在同一 immutable source 内做 paired
  因果比较，不能把两次重录的逐 seed 差值归因给算法；
- 大 bag 会因工作盘空间删除，只保留 hash、manifest、CSV、metrics 和日志；复查原始 topic 需确定性重录。

## 3. 算法限制

- packet split 仅在 global packet 真正覆盖多个 predicted gates 时触发；它不能恢复传感器根本没有
  返回的目标，也不能解决所有 packet shortage；
- CV/CA IMM 不含 coordinated-turn、JPDA/MHT 或 learned model，密集遮挡仍可能 gate miss；
- dormant reachable bound 很宽时仍依赖 F/U provenance 和 surface rejection 控制误激活；真实 clutter
  需要额外离线验证；
- reportability 抑制 ghost 的代价是遮挡期间不输出位置；该轨迹记忆不能直接当 collision-free 证据；
- S05 中 dormant 将 FN 从 V3 base 200.7 降至 173.1，但仍高于 B0 147.7；正式 S05 的 online
  occluded transition 未触发，主要使用 low-existence fallback；
- 严格 C0→C1 中 opportunity 使 FN 增加 74.1、HOTA 降低 0.3649，虽同时降低 FP/frag/ghost；
  因而当前 opportunity likelihood 不是独立净正收益；
- S08B 虽能 birth，但 final TTFT 18.1 s、HOTA 0.460，unknown-moving recall 稳健性不足；
- persistent stable background 没有长期 forgetting/rolling-map 重构，动态环境切换仍有限；
- map protection 的 contamination 净收益在 V2 几乎不可辨，V3 不能在缺少独立指标时宣称它有效。

## 4. 系统边界

尚未完成真实 Mid360 多目标采集、MRS/PX4 闭环、collision avoidance safety case、雨雾/多径/强反射
或 CPU contention 验证。即使仿真门禁通过，V3 也只是 record-only 实机数据采集候选，不是可直接
部署的飞行安全组件。是否进入 R6/R7/实机飞行以 `SOFT_VOFOD_V3_FINAL_REPORT.md` 的显式门禁结论
为准。

本轮 R4/R5 未完全通过预设门禁，因此 R6 小矩阵与 R7 10-seed 扩展均未运行。
