# SOFT-VoFOD V2 最终实现与实验报告

日期：2026-08-22
结论：V2 已完成源码修正、CAL 标定、S01–S08C/N0/N1/5-seed/B0–B4 全矩阵和测试。它消除了
V1 的病态 event/support/runtime 正反馈，但**没有全面超过 B0**；S04、S05、S08C 和地图
false-free balance 仍需后续研究。

## 1. 修复前根因

源码审计和冻结 artifacts 给出的根因不是“参数略差”，而是粒度与生命周期错误：

1. V1 把每条 free-space violation ray endpoint 当成独立 target event；
2. 每个 raw event 又创建 0.75 m/2 s map support，support query 对每条 ray 线性扫描；
3. moving observer 下未同化的新背景持续变成 event，event→support→地图不更新→更多 event；
4. birth 和 maintenance 直接消费 raw endpoints，单个物体/静态表面生成重复 hypotheses；
5. `P_D=0` 时 existence 不下降，所有轨迹共用 30 s hard timeout，ghost 长期保留；
6. 长期 free/background evidence 在 10 ms micro-batch 直接写入，没有低频 epoch 和 ray-count
   saturation；
7. unknown 新背景没有持久性 assimilation 路径；
8. opportunity 最坏接近 tracks×rays×sigma×confirmed-tracks；
9. runner 只等 output advertise，未等 input/recorder subscriptions；B0 底层 subscribers queue
   还硬编码为 2，recorded replay 会丢首帧，造成 S06 warm-up 从错误的晚到输入才开始。

这也解释旧 A1/A2/A3 为什么普遍低于 B0：地图未成熟时 raw endpoint 数量主导了 birth/maintenance，
trajectory 和 GNN 只能在已经污染的 observation pool 上工作；A3 的 opportunity/feedback 又把
track/support 数量带入更高阶计算。S07 的 observer motion 使背景 violation 最密集，最终触发
event、track、support 和 CPU 同时爆炸。

## 2. 修改文件

Phase 1–14 相对 V1 baseline `56c61f2` 修改 37 个 tracked files，按职责为：

- B0 replay 合同：
  `src/vofod_mid360/{config/b0_mid360_canonical.yaml,msgs/MapUpdateDiagnostics.msg,
  src/vofod_nodelet.cpp,test/b0_soft_update_test.py}`；
- V2 core/adapter：
  `src/soft_vofod_mid360/{include/soft_vofod_mid360/core.h,src/core.cpp,
  src/soft_vofod_node.cpp,launch/soft_vofod.launch,msg/FreeSpaceViolationEvent.msg,
  config/default.yaml,config/no_overlay.yaml,config/soft_vofod_v2_canonical.yaml,
  config/soft_vofod_v2_calibration.yaml,test/core_test.cpp,test/pipeline_test.py}`；
- evaluator/runner：
  `src/soft_vofod_evaluation/{scripts/evaluate_bag.py,scripts/run_benchmark.py,
  test/evaluation_test.py,test/runner_test.py}`；
- 场景：`src/mid360_multi_uav_sim/config/benchmarks/{CAL01–CAL05,NEG01–NEG04,
  S08A–S08C}.yaml` 与 `test/benchmark_scenarios_test.py`；
- 阶段文档：`docs/{SOFT_VOFOD_V2_ROOT_CAUSE_EXPERIMENTS.md,IMPLEMENTATION_CHANGELOG.md,
  CURRENT_MRS_VOFOD_MID360_IMPLEMENTATION.md,
  SOFT_VOFOD_EXPERIMENT_REPORT.md,KNOWN_LIMITATIONS.md}`。

Phase 15 新增本文及 8 份专题文档，并更新 `src/soft_vofod_mid360/README.md`。完整变更可用
`git diff --name-status 56c61f2..HEAD` 审计。

## 3. 从生产路径移除的 V1 机制

- `one endpoint → one birth observation` 被 short-term spatial packet 取代；
- raw-event 0.75 m/2 s support 被删除；
- support/ray 全量线性查询被 voxel index 取代；
- micro-batch 长期 map commit 被 5 Hz deferred epoch 取代；
- raw-return maintenance pool 被 violation/unresolved/track-explained packets 取代；
- `P_D=0` 永不衰减和统一 30 s timeout 被 survival + 状态分离 deadline 取代；
- full-ray opportunity brute force 前增加 angular index；
- proposed method 固定 8 s target-free warm-up 被设为 0；
- 旧 A1/A2/A3 不再是最终消融，只保留冻结开发证据。

singleton packet 没有被删除或禁用；远距离单点 observation 仍合法。

## 4. 新增的 V2 模块

1. `MapEpochAccumulator` 语义：epoch traversal/return 累积与 free saturation；
2. connected background/free/unknown/track component 归属；
3. per-voxel cold-start candidate persistence；
4. spatiotemporal violation packetizer；
5. independent-group weighted CV trajectory birth；
6. packet-level rectangular Hungarian + CV-KF maintenance；
7. scan-level ray opportunity、range-conditioned return probability；
8. survival/hit/miss Bernoulli existence；
9. tentative/confirmed 分离 timeout 与 conservative duplicate merge；
10. confirmed-only support/quarantine、voxel support index、angular ray index；
11. first-input/warm-up/coverage/hash/replay-rate benchmark contracts；
12. map、packet、ghost、opportunity、runtime/complexity evaluator 指标。

为避免无必要抽象，这些能力集中在现有 `BackgroundMap` 和 `SoftVofodCore`，没有复制一套平行
V2 package 或为 rejected CAL candidates 留永久开关。

## 5. 公式与代码位置

| 公式/机制 | 代码位置 | 专题说明 |
|---|---|---|
| \(p_F,p_B,c\) | `core.cpp:186/269` | `SOFT_VOFOD_V2_MAP_ASSIMILATION.md` |
| \(w_F(n)=w(1-e^{-n/n_0})\) | `core.cpp:850–861` | 地图同化文档 |
| candidate persistence | `core.cpp:423–543` | `SOFT_VOFOD_V2_COLD_START.md` |
| packet median/covariance | `core.cpp:545–648` | `SOFT_VOFOD_V2_EVENT_PACKETIZATION.md` |
| CV \(F,Q\) | `core.cpp:1055–1080` | `SOFT_VOFOD_V2_TRACK_EXISTENCE.md` |
| miss/survival/hit existence | `core.cpp:1082–1119` | 轨迹存在文档 |
| rectangular Hungarian | `core.cpp:1120–1221` | 轨迹存在文档 |
| trajectory birth/refit | `core.cpp:1358–1600` | 轨迹存在文档 |
| support voxel index | `core.cpp:1645–1779` | 地图同化文档 |
| opportunity angular index | `core.cpp:1782–1905` | 轨迹存在文档 |
| \(P_D=1-\prod(1-o_ip_i)\) | `core.cpp:1250/1907–2007` | 轨迹存在文档 |
| scan-level lifecycle | `core.cpp:2421–2564` | 轨迹存在文档 |

主流程与所有公式的完整推导见 5 份算法专题文档；这里不重复抄一套容易漂移的伪实现。

## 6. 参数来源

运行时权威文件为 `src/soft_vofod_mid360/config/soft_vofod_v2_canonical.yaml`。来源分为：

| 参数组 | 最终值/来源 |
|---|---|
| 输入合同 | sync queue 32、geometry tolerance 0.01 m；来自 replay 丢首帧修复与既有配对合同 |
| map geometry | center/size `(10,0,5)/(40,30,16)` m、voxel 0.5 m；继承 V1 仿真工作体积 |
| map probabilities | evidence scale 5、confidence 0.6、free/background 0.7；V1 engineering defaults |
| map epoch | 5 Hz、free `n0=1`、weight 1；CAL01/02 验证，`n0=10` 被拒绝 |
| component | attach/separate 0.8/1.0 m、free ratio 0.5、track ratio 0.25；工程门限，经 CAL control 保留 |
| unknown candidate | 3 epochs/1 s、sigma 0.15 m、match 1 m、timeout 1 s；CAL01/02 与 per-voxel 修复 |
| packet | 0.03 s/0.75 m、sensor variance 0.1 m²、shape 0.35 m、floor 0.04 m²；工程值，低 covariance 被拒绝 |
| final birth | B3/B4 5 groups；CAL03 从 3→5。B1/B2 的 2/3 groups 是消融定义 |
| birth/KF gates | buffer 2 s/256、speed 15 m/s、residual 0.8 m、gate 11.345、\(\sigma_a=3\)；V1 engineering defaults，\(\sigma_a=6\) 被拒绝 |
| existence | 0.6/0.8/0.1、tentative 1/0.3 s、confirmed 6 s、hard 30 s；V2 lifecycle engineering gates |
| survival | \(\lambda=0.1/s\)；CAL05 与 0.05/s 比较后冻结 |
| return probability | 0–10/10–20/20–30/30+ m = 0.916/0.878/0.877/0.5；CAL04，最后一档未标定 fallback |
| opportunity | cap 0.95、range 60 m、margin 0.15 m、sigma scale 1；工程安全门限 |
| support | radius 0.75 m、sigma multiplier 2、cap 1.5 m、quarantine 2 s；V1/V2 engineering defaults，语义改为 confirmed-only |
| startup | B4 fixed warm-up 0；由 online persistence 取代。B0 保留自身 10 s benchmark warm-up |

只有表中明确标成 CAL 的值具有 CAL split 选择证据；其余仍是工程默认值，不能写成传感器实测
标定。`soft_vofod_v2_calibration.yaml` 最终为空，因为接受值已复制进唯一 canonical；被拒绝候选
没有留在生产配置。

## 7. S06/B0 warm-up 修复

旧 run：source first ray 4.228 s，B0 实际 warm-up start 5.427 s，target spawn 14.657 s，complete
15.428 s；完成晚于 target 0.771 s，旧结果应为 `INVALID_WARMUP`。

Phase 1 修复后 N0/seed1001：ack/start 4.228 s、complete 14.228 s、target spawn 14.657 s、first
scored `warmup_active=false`、status ok。修复前后 B0 HOTA/IDF1/TP/FP/FN 完全相同，说明修复的是
实验有效性而非改算法得分。

最终 10 个 S06/B0 run 的 ack/start 为 4.037–4.343 s，complete 为 14.045–14.362 s，首个 scored
frame 全部 inactive。根因是 runner input handshake 缺失和底层 queue=2，而不是 B0 必须超过
10 s 才能成熟。

## 8. S07 old A3、no-event-support 与 V2

| 指标 | old A3 | A3 no-event-support | V2 B4 N0/seed1001 |
|---|---:|---:|---:|
| event/raw | 17,837 | 16,402 | 830 raw |
| packets | 不适用 | 不适用 | 6 |
| support peak | 7,761 后验估计 | 234 | 1 直接测量 |
| TP/FP/FN | 176/15,434/14 | 176/10,790/14 | 183/0/7 |
| HOTA | 0.0867 | 0.1012 | 0.9814 |
| p95 | 1001.64 ms | 260.75 ms | 43.64 ms |
| processing load | 2.933 | 1.085 | <1（1.0x 完成） |

删除 event support 证明了主要复杂度根因，但仍留下 raw-event birth、ghost 和 full-ray
opportunity；完整 V2 的 deferred map、packet、lifecycle、index 和 confirmed feedback 共同闭环。
正式 S07 10-run B4 为 HOTA 0.980±0.006、0 FP/IDSW/fragmentation、p95 43.44±3.01 ms。

## 9. NEG03/NEG04 moving-observer 结果

均为 Phase 12 N0/seed1001、109 s scored、无 target：

| 指标 | NEG03 moving observer | NEG04 new-area clutter |
|---|---:|---:|
| false packets/min | 7.156 | 0.550 |
| false tentative/min | 2.202 | 0 |
| false confirmed/min | 0.550 | 0 |
| confirmed peak/final | 1/0 | 0/0 |
| max stale | 0.570 s | 0 |
| static recall | 0.0439 | 0.2287 |
| false-free | 0.2207 | 0.0428 |
| expansion | 0.7303 | 0.9754 |
| p95 | 48.45 ms | 41.18 ms |

结论是 track/support/runtime 有界，而不是地图已完美：NEG03 的 recall/false-free 仍差。

## 10. S01/S03/S05 根因闭环

- **S01：部分闭环。** B4 vs B0 HOTA 0.437 vs 0.326、FP 76.7 vs 85.3、IDSW 1.3 vs 5.2；
  但 FN 130.8 vs 100.5、frag 6.9 vs 5.5。重复 birth/ID 已改善，range-ladder recall 未闭环。
- **S03：闭环。** B4 vs B0 HOTA 0.989 vs 0.890、IDSW 0 vs 2.2、frag 1.8 vs 2.9；FP
  3.7 vs 0 是小代价。B1→B2 birth 22.8→13.5，B4 为 2.0。
- **S05：生命周期闭环，整体性能未闭环。** 同-group Phase 12 B2→B3 fragmentation 9→3、
  stale>3 s 78→1；正式 B4 stale>3 s 为 0、frag 2.9 vs B0 5.5。但 HOTA 0.254 vs 0.333、
  FN 193.7 vs 147.7，reacquisition/recall 仍失败。

## 11. Map 指标

100-run 宏平均：

| 算法 | static recall | false-free | expansion | contamination |
|---|---:|---:|---:|---:|
| B0 | 0.0627 | 0.1744 | 0.7532 | 0 |
| B1/B2/B3 | 0.1411 | 0.2249 | 0.7662 | 0.006301 |
| B4 | 0.1390 | 0.2247 | 0.7541 | 0.006267 |

static recall 提高，但 false-free 更差；B4 protection 没有显著降低 contamination。S08A B4
candidate→stable p95 为 15.67±0.97 s、expansion 0.609±0.013、0 birth/track。

## 12. Track 指标

9 个有 target 场景、90 runs 的宏平均：

| 算法 | HOTA | TP | FP | FN | IDSW | fragmentation |
|---|---:|---:|---:|---:|---:|---:|
| B0 | 0.7582 | 333.33 | 35.77 | 46.04 | 1.233 | 1.700 |
| B4 | 0.7535 | 311.79 | 39.73 | 67.59 | 0.656 | 2.844 |

B4 的 identity 更稳定，但 recall/fragmentation 尚未达到 B0。B4 N0/N1 target-scene HOTA 为
0.7581/0.7490，FP/run 34.49/44.98，说明 N1 有温和退化而非崩溃。

B1→B2 birth/run 10.21→7.13；B3/B4 全 100-run `>3 s` ghost 为 0；B4 max stale 最大
1.762 s。正式 B2→B3 因 groups 3→5 而有 confound，详见消融文档。

## 13. Runtime

100-run mean p95：B0/B1/B2/B3/B4 = 52.07/37.67/38.27/38.31/41.85 ms。B4 最低/最高
p95 为 34.60/50.65 ms，100/100 低于 100 ms。S07 B4 p95 43.44±3.01 ms；V1 S07/A3 的
1001.64 ms 病态失败已消失。

最终构建与测试：11/11 packages build，无 warning/failure；386 tests，0 errors/failures/skipped。

## 14. 失败场景与声明边界

- S04：B4 HOTA 0.894、FP 110、frag 13，显著弱于 B0；
- S05：fragmentation/ghost 改善但 FN/HOTA 失败；
- S06：B4 HOTA 0.679 vs B0 0.738、FP 56.3 vs 18.4；
- S08A：无 false track，但 false packets 12.53/min、expansion 不完整；
- S08C：10/10 错误确认 unknown stationary object；
- 全局 false-free 比 B0 差，map protection contamination 收益未证明；
- B2→B3 多 seed 消融有 birth-group confound；
- 真实 Mid360、完整 rolling collision、MRS/PX4 闭环均未证明。

## 15. 是否值得继续

值得继续，但当前应定位为研究原型而不是部署候选。N1 和 5 seeds 已完成，无需重复“先进入
N1”；下一步优先级为：

1. 在独立新 CAL/negative split 修 S04 multi-target fragmentation、S05 sparse reacquisition、
   S08C stationary ambiguity；
2. 统一 B2/B3 birth groups 后重跑严格单因素多 seed 消融；
3. 改善 free/background balance，并设计能真正测出 contamination 差异的场景；
4. 修复后扩至 10 seeds；
5. 再用真实 Mid360 record-once bags 重新标定 `p_ret(range)`、noise/covariance 和 map gates；
6. 最后才进入 MRS/PX4 闭环与安全决策。

在 S04/S05/S08C 未闭环前，不建议直接进入真实多机飞行。

## 七个问题的最终回答

1. **false-free 是否下降？** 相对 old S07 A3 下降；相对 B0 全矩阵没有，B4 0.2247 比
   0.1744 更差。
2. **static-background recall 是否提高？** 相对 B0 从 0.0627 提到 0.1390；但绝对值低，B4
   还略低于 B3。
3. **event/packet storm 是否消失？** 是；B4 packet 最大 60/run，support/track peak 最大 7，
   p95 最大 50.65 ms。
4. **duplicate birth 是否下降？** 是；B1→B2 10.21→7.13，B4 为 1.96 births/run。
5. **ghost tracks 是否消失？** 在当前仿真矩阵中是；B3/B4 `>3 s` ghost 均为 0。
6. **opportunity 是否真的降低 fragmentation？** 同为 3 groups 的 Phase 12 S05 控制为
   9→3；正式多 seed B2→B3 被 groups 变化混杂，不能扩大为多 seed 单因素证明。
7. **map protection 是否真的降低 contamination？** 没有证据；0.006301→0.006267，90 对中
   88 对完全相等。

综上，V2 的主要成功是把病态系统变成有界、可实时、可审计的系统；当前结果不支持“全面超过
B0”，但为下一轮针对性研究提供了可信基线。
