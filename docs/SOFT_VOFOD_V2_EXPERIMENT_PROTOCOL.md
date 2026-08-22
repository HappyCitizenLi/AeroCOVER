# SOFT-VoFOD V2 实验协议

## 1. 数据划分与调参纪律

- CAL：CAL01–CAL05，只用于选择 map、birth、return probability 和 survival 参数；
- negative controls：NEG01–NEG04；Phase 12 实际执行 NEG03/NEG04；
- test：S01–S07 与 S08A/B/C；禁止逐 scene 调参；
- 正式矩阵：N0/N1、seeds 1001–1005、B0–B4，共 10×2×5×5=500 runs。

canonical 在 CAL 后冻结，正式矩阵全部来自 Git commit
`4cc1fe71c20aae6f2110b925733c679f39dfd98e`。B0 config hash 唯一；B1–B4 共用同一 base config
file hash，显式 launch arguments 定义 groups 与模块组合。当前 config hash 本身不编码这些
launch overrides，复现时还必须同时使用 manifest 的 algorithm 和冻结 runner commit。

## 2. 算法版本

| 版本 | 内容 |
|---|---|
| B0 | VoFOD-Mid360 + classic tracker frozen baseline |
| B1 | deferred background + packet violation + simple/two-group birth；无 opportunity |
| B2 | B1 + three-group trajectory-before-confirmation |
| B3 | five-group trajectory birth + scan opportunity + survival/existence |
| B4 | B3 + confirmed-track-only map protection |

Hungarian/GNN 固定为 B1–B4 后端，不作为单独核心创新。旧 A1/A2/A3 只用于开发历史对比，不进入
最终论文消融。CAL03 将 final birth threshold 冻结为 5 groups，但 runner 保留 B2=3 groups，
所以正式 B2→B3 同时改变 birth threshold 与 opportunity/survival；它不是纯 opportunity
单因素。严格因果证据来自 Phase 12 中同为 3 groups 的 B2/B3 控制，最终报告不得混淆两者。

## 3. 场景

| 场景 | 目的 |
|---|---|
| S01 | 单目标 hover/range ladder/birth |
| S02 | radial sparse-return sweep |
| S03 | 两目标 0.5/1/2/5 m crossing |
| S04 | 四目标并行、交叉、高度和转向 |
| S05 | 1/3/5 s wall occlusion/reappearance |
| S06 | near-background takeoff/landing |
| S07 | moving-observer deskew stress |
| S08A | cold-start exploration/no target |
| S08B | unknown 区域 moving target |
| S08C | unknown 区域 stationary object |

场景 YAML 位于 `src/mid360_multi_uav_sim/config/benchmarks/`。S07/S08 使用 per-ray pose；其余按
各 YAML 固定几何执行。

## 4. record-once/replay-many

每个 scene/noise/seed 只录一个 immutable source bag，五算法复用同一 source SHA-256。正式执行：

1. 等待 algorithm 对 points/rays 的 input subscriptions；
2. 等待 output recorder 对全部 topics 建立 subscriptions；
3. B0 单独 1.0x replay；
4. B1–B4 在四个隔离 ROS masters 上并行 1.0x replay；
5. 任一子 run 非零、metrics/manifest 为空或 coverage invalid，则整个 source 组合失败。

runner 位于 `src/soft_vofod_evaluation/scripts/run_benchmark.py`。master port 取自
`ROS_MASTER_URI`；订阅握手在 `subscriptions_ready/wait_for_subscriptions`（行 300–316）和
`wait_for_recorder_subscriptions`（行 319–336）。

## 5. 有效性合同

每个 manifest 必须满足：

- `status=ok`、`replay_rate=1.0`；
- first-input ack 不晚于 source first scored input；
- B0 warm-up complete 早于 first target spawn/scoring，且 first scored frame 不 active；
- source truth、track 和 timing coverage ≥0.95；
- 同一 source triplet 的五算法 source SHA-256 相同；
- algorithm config/implementation SHA-256 和 Git commit 可追溯。

500 个正式 run 全部通过；最低 truth coverage 0.973684，track/timing 最低均为 1.0。

## 6. 指标

离线 evaluator 位于 `src/soft_vofod_evaluation/scripts/evaluate_bag.py`，输出：

- track set：HOTA@1m、DetA、AssA、IDF1、TP/FP/FN、IDSW、fragmentation、GOSPA、误差、TTFT；
- event：recall/precision、false/min、persistence、purity、singleton ratio；
- map：false-free、static recall、expansion recall、candidate latency、contamination；
- opportunity：Brier、NLL、按 \(P_D\) miss rate、按 range 的实际 return probability；
- health：tentative/confirmed false rate、unique/peak、0.5/1/3 s stale；
- runtime/complexity：module mean/p50/p95/p99/max、rays、raw endpoints、packets、births、tracks、supports。

无 target 场景不把 HOTA=0 当性能分数；只使用 false-event/track 和地图指标。None/不适用字段不以
零混入均值。

## 7. 产物与空间策略

正式结果位于：

```text
artifacts/runs/{B0,B1,B2,B3,B4}/{scene}/{N0,N1}/seed_<seed>/
artifacts/metrics/{summary.csv,aggregate.json,ablation_deltas.csv}
artifacts/phase14_parallel_logs/
```

聚合器扫描到 530 行，其中 30 行为 CAL/历史结果；正式报告严格按 500-entry 笛卡尔积和冻结提交
筛选。为控制磁盘，100 个 source bag 与 500 个 output bag 已删除，只能按确定性配置/种子重录；
metrics、manifest、时序 CSV、resource 和日志保留。

## 8. 最终验证

- `catkin build --no-status`：11/11 packages，无 warning/failure；
- 逐包串行 `catkin run_tests ... -j1`；
- `catkin_test_results build`：386 tests，0 errors/failures/skipped。

串行是因为多个 Gazebo rostest 共享固定服务名，并发会制造 spawn timeout，不是算法需求。
