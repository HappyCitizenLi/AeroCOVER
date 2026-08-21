# 实现变更记录

## 2026-08-20 — Phase 0/1

### 公共输入修复

- 新增 `mid360_ray_msgs/ScanIdentity` companion；仿真插件和硬件 spherical fork 在创建同一
  scan 的 cloud/RayBundle 后发布 scan identity。
- 仿真 cloud 与 RayBundle 改用同一 source stamp；修复 PointXYZ OpenMP append 导致的
  source-order 重排。
- preprocessor 改为 ExactTime 三输入，并显式验证 `scan_id`、source stamps、
  `pattern_start_index`、counts、source order、duplicate、VALID_RETURN 匹配和 range。
- 任一 pair/TF/world-point 合同失败时不再发布部分派生结果。
- 新增 `/uav1/mid360/points_world`，坐标由 checked ray endpoint 直接生成；增加要求的
  `scan_pair_ok` 等诊断及时间回退拒绝。

### B0 修复

- detector 和 tracker 统一消费 `points_world`；tracker 删除旧 sensor-frame cloud TF 与
 二次 self crop。
- 删除 `nadir_seed` 类型、参数、诊断、测试和文档路径；canonical startup 改为 target-free
  background warm-up + map maturity gate。
- tracker background filter canonical 默认开启且只接 `background_points`。
- 修正 tracker covariance radius 的 variance-to-length 单位错误；Q/P0/R 文案统一为
  covariance/variance。
- `b0_evaluation.launch` 替换为唯一 `tclv_evaluation/b0_canonical.launch`；删除兼容
  `vofod_mid360/detect.launch` 和未使用的 nodelet manager 切换参数。
- detector config 更名为 `b0_mid360_canonical.yaml`。

### 验证

- `catkin build`：9/9 packages 成功，无 warning/failure（最终增量轮）。
- `catkin run_tests --no-status -j1`：9/9 packages 成功。
- `catkin_test_results build`：288 tests，0 errors，0 failures，0 skipped。
- canonical launch 静态展开为 detector/tracker 两个算法节点；6 个修改过的 Python 文件通过
  AST parse。
- 第一次测试发现并清理了已删除 `PointCloudScanInfo` 留下的 catkin 生成缓存；它不是源码，
  clean rebuild 后新 `ScanIdentity` 消息和所有依赖重新生成。

## 2026-08-21 — SOFT-VoFOD A3 算法阶段

### Proposed algorithm

- 新增单包 `soft_vofod_mid360`；复用 B0 已测试 `VoxelMap`/DDA 几何，不复制 B0 detector，
  不改 B0 行为。
- 实现 conservative four-state background evidence、旧地图 event 查询、independent-group
  trajectory-before-confirmation birth、6-state CV-KF、Mahalanobis/Hungarian GNN 与 singleton/
  local median packet。
- 实现逐真实 ray sigma-point opportunity、return/confirmed-track front occlusion、NO_RETURN
  opportunity、log-domain Bernoulli existence 与 confirm/delete hysteresis。
- 实现 event/active/deleted target quarantine；VALID_RETURN 和 NO_RETURN free carving 均在
  target support 近端截断。
- 新增 required event/track/map/opportunity/diagnostics outputs、algorithm-only canonical launch
  和统一默认参数。
- adapter 再验证 world geometry/scan contract；不依赖会被 ROS publisher 改写的
  `Header.seq`。stamp 回退拒绝，大时间跳变暂停 background endpoint evidence 和 birth。

### 测试与文档

- 新增 14 个 gtest cases 和 1 个 ROS integration testcase，覆盖 map/birth/KF/GNN/
  opportunity/existence/quarantine/truncation/hard-timeout 以及 free→event→birth 消息链。
- 局部 `catkin build soft_vofod_mid360` 成功；局部 test result 为 30 tests、0 error、
  0 failure、0 skipped。
- 最终 `catkin build` 为 10/10 packages 成功且无 build warning/failure；串行全量测试为
  10/10 packages 成功，`catkin_test_results build` 汇总 318 tests、0 error、0 failure、
  0 skipped（原 288 项回归全部保留）。
- 新增 `SOFT_VOFOD_ALGORITHM.md`、`SOFT_VOFOD_FORMULAS.md`、`KNOWN_LIMITATIONS.md` 与本阶段
  `FINAL_IMPLEMENTATION_REPORT.md`。
- benchmark runner、S01/S03/S05 指标和硬件标定明确留给后续实验阶段，未生成虚假结果。

## 2026-08-21 — Phase 2/9/10 仿真、消融与评估

### 算法和 ROS 适配修正

- 新增 A1/A2/A3 显式 switches；A1 使用 two-group birth，A2 使用 trajectory birth，A3
  开启 opportunity existence、target feedback 和 Hungarian。
- 加入 8 s map-only background initialization；warm-up 中 event birth 不进入 buffer。
- coarse grazing ray 不再把 candidate/stable endpoint voxel 当 free；修复地面预热伪轨迹。
- Hungarian 从 `max(rows,columns)` 方阵 padding 改为带每轨 dummy 列的矩形 shortest-
  augmenting-path；S01/A3 p95 从约 250 ms 降到约 53 ms并恢复满帧。
- SOFT adapter 与冻结 B0 对齐 retained-point 语义：被 preprocessor self/range filter 移除的
  VALID source return 不再使整帧 fail；subscriber 深度复用 `sync_queue_size`。

### 场景、truth 和 runner

- 新增 generic benchmark launch/timeline controller 与 S01–S07 YAML；目标只在 10 s
  target-free warm-up 后生成，S07 使用 per-ray pose。最终 source 实测首个 derived ray 到
  target spawn 为 10.409–10.495 s，避免只用 YAML 名义时间通过门禁。
- visibility truth 增加 present/in-range/in-FOV/line-of-sight/actual-return 字段。
- 新增 `soft_vofod_evaluation`：isolated roscore、record-once/replay-many、source/output/config/
  implementation hashes、B0 legacy time bootstrap、replay-rate manifest 和失败非零退出。
- evaluator 实现严格一对一 HOTA@1m/DetA/AssA/IDF1/IDSW/GOSPA、状态/连续性、event、
  opportunity、map、latency/CPU/RSS 指标；source truth、track、timing 三重 95% coverage gate。
- 聚合器输出逐 run summary、group statistics schema 和 paired A3 ablation deltas。

### 实验结果

- 完成 N0/seed1001 的 S01–S07 × B0/A1/A2/A3：7 source bags、28/28 valid runs、同场景
  source SHA 一致。
- A3 跨场景描述性 mean HOTA@1m 0.362，冻结 B0 为 0.676；当前实验没有证明 A3 优于 B0。
- S07/A3 p95 约 1.00 s、processing load ratio 2.93，0.1× 离线回放才满足完整性；该项明确
  判为非实时。
- N1/N2、多 seed、CAL/NEG 未执行，不作 publication-grade 统计声明。完整结果见
  `SOFT_VOFOD_EXPERIMENT_REPORT.md` 和 `artifacts/metrics/`。
- 最终 `catkin build` 11/11 成功且无 build warning；`catkin run_tests --no-status -j1`
  11/11 成功；`catkin_test_results build` 为 339 tests、0 error/failure/skipped。Gazebo tests
  共享固定服务名，因此全量验证固定使用串行模式。

## 2026-08-21 — SOFT-VoFOD V2 Phase 0 冻结审计

- 完整复核 V2 修正提示词、V1 实现/实验报告、核心源码、配置、launch、runner、manifests 与
  output bags；新增 `docs/V2_PRE_IMPLEMENTATION_AUDIT.md`。
- 确认当前 Git `master` 是 unborn branch，没有可解析 HEAD；旧 run manifests 的
  `uncommitted-no-commit` 属实，并用 SHA-256 冻结 V1 关键文件。
- 冻结 S07/A3 的 17,837 events、230 peak tracks、7,761 estimated peak supports、
  792.36 ms peak map commit 和 288.66 ms peak tracking。
- 确认 S06/B0 的实际 warm-up 从 5.427 s 开始、15.428 s 完成，晚于 14.657 s target spawn；
  旧 run 应为 `INVALID_WARMUP`，在 Phase 1 修复前不进入算法结论。
- 基线 `catkin build` 11/11 成功且无 build warning；串行 tests 11/11 成功；
  `catkin_test_results build` 为 339 tests、0 errors/failures/skipped。

## 2026-08-21 — SOFT-VoFOD V2 Phase 1 replay 有效性

- runner 播放前通过 ROS master system state 等待 detector/algorithm 的 `points_world` 和
  `rays_checked` subscriptions，不再用固定 sleep 代替输入就绪合同。
- B0 `MapUpdateDiagnostics` 新增 first-input ack、warm-up start/complete stamps；SOFT diagnostics
  新增 first-input ack 与 map-bootstrap start。
- source manifest 新增 `first_scored_input_stamp`；run manifest 新增 `input_timing` 和 `status`。
  ack 晚于首个 scored input、B0 complete 不早于 target spawn 或首个 scored frame 仍 active 时
  非零退出；invalid manifest 不进入 aggregate。
- 旧 S06/B0 run 可恢复地保存到 ignored `artifacts/v1_frozen/`；新 run 从 4.228 s 开始、
  14.228 s 完成 warm-up，早于 14.657 s spawn，status `ok`。
- 新旧 S06/B0 HOTA/IDF1/TP/FP/FN 完全相同（0.7423686/0.7106017/124/15/86）；28-run summary、
  aggregate 和 ablation deltas 已重新生成。
- 定向 build 9/9 成功且无 warning；evaluation/B0/SOFT tests 分别 9/57/40，零失败。

## 2026-08-21 — SOFT-VoFOD V2 Phase 2 event-support 正反馈

- 删除 raw event 的 0.75 m/2 s spherical support；raw event 只冻结自身 voxel 一个 10 ms
  micro-batch。tentative track 只使用物理半径，confirmed track 才增加 capped uncertainty margin；
  只有刚删除的 confirmed track 保留短时 support。
- 复用现有 `VoxelMap` geometry/DDA 建立 support-to-voxel index；ray 只收集沿 DDA cells 的候选，
  endpoint 只查询所在 voxel，不再对全部 supports 线性扫描。
- diagnostics 新增直接测量的 `support_count`；新增 raw event 不产生 map support 的单元测试。
- S07 no-event-support 单因素 run：support mean/peak 74.4/234（track peak 213），旧估计 peak
  7,761；map-commit mean/peak 208.3/792.4 ms 降到 37.2/49.5 ms；total p95 1001.6→260.7 ms。
- event count 17,837→16,402，false-free 0.354→0.328，FP 15,434→10,790；仍有 ghost tracks 和
  全 ray opportunity，故尚未达到 100 ms 门槛。
- `soft_vofod_mid360` build 3/3 成功、42 tests 零失败；S07 run 190/190 scored diagnostics，
  handshake/coverage gates 通过。

## 2026-08-21 — SOFT-VoFOD V2 Phase 3 survival 与 ghost-track 生命周期

- existence 每帧先用 `exp(-lambda_S * dt)` 做 survival prediction，再根据整帧聚合的
  detection opportunity 做一次 hit/miss Bayes update；`P_D=0` 不产生观测惩罚，但轨迹仍会
  随 survival 缓慢衰减。
- 修正最初把 survival/miss 放进 10 ms micro-batch 的时间尺度错误。该错误会把同一帧重复
  计罚，导致 S07 HOTA 0.101→0.0365、fragmentation 3→23；错误 run 已保存在
  `artifacts/v2_phase3_microbatch_bug/`，并新增跨 micro-batch 的帧级 existence 回归测试。
- tentative 使用 1.0 s confirmation/max-age deadline 和 0.3 s no-measurement deadline；
  confirmed 使用 6.0 s no-measurement deadline，30 s hard timeout 仅作 safety fallback。
- 新增保守 duplicate merge：同时要求 0.25 m absolute distance、position Mahalanobis、
  velocity、birth/last-measurement history 与 support overlap 通过；只保留状态/存在概率/
  正更新数/年龄/协方差更优者，不融合状态，0.5/1/2 m 邻近真实目标不会仅凭距离被合并。
- 修正版 S07 相对 Phase 2：HOTA 0.1012→0.1083、TP 176→187、FN 14→3、fragmentation
  3→0；track mean/peak/final 40.7/213/157→31.0/172/73，duplicate merge 触发 2 次；
  runtime mean/p95 108.5/260.7 ms→92.3/220.6 ms，processing load 1.085→0.923。
- `soft_vofod_mid360` build 3/3 成功、48 tests 零失败；S07 190/190 scored frames、输入握手和
  coverage gates 均通过。S07 仍有 8,134 FP 和 73 个末帧轨迹，后续 map assimilation 与
  packet-level birth/maintenance 仍是必要工作。
- 全工作区 `catkin build` 11/11 成功且无 build warning；串行 tests 11/11 成功，
  `catkin_test_results build` 汇总 348 tests、0 error/failure/skipped。

## 2026-08-21 — SOFT-VoFOD V2 Phase 4 deferred map epoch

- 生产 map 写路径改为 5 Hz/0.2 s epoch：micro-batch ray 只累积 voxel traversal，endpoint
  每 voxel/epoch 最多保留一次；长期 free/background 状态只在 epoch 边界提交。
- free traversal 采用 `w_F * (1 - exp(-n_F/n0))` 指数饱和；endpoint 与同 epoch 的 coarse
  free traversal 冲突时 endpoint 优先，避免静态表面被离散化 grazing ray 冲掉。
- 新增 map epoch commit 数、free/background voxel 数、raw/committed free evidence 诊断；
  新单测验证边界前不可见和 1000 vs 100 correlated rays 的长期增量比小于 1.01。
- S01/A3 单因素 run 共提交 156 epochs/31.2 s（约 5 Hz）；raw free evidence
  `1.093e8` 压缩为 `1.140e7`，比值 0.104。HOTA 0.294→0.576、TP 85→119、
  fragmentation 13→6、p95 53.0→43.3 ms，false-free 0.2639→0.2641。
- `soft_vofod_mid360` build 3/3 成功且无 warning，50 tests 零失败；S01 input/coverage
  gates 通过。该 phase 仍按 voxel 聚合 endpoint；连通域背景吸收在 Phase 5 实现。
- 全工作区 `catkin build` 11/11 成功且无 build warning；串行 tests 11/11 成功，
  `catkin_test_results build` 汇总 350 tests、0 error/failure/skipped。

## 2026-08-21 — SOFT-VoFOD V2 Phase 5 background component assimilation

- epoch valid returns 先按 0.5 m voxel downsample，再用确定性 26 邻域 connected components
  聚合；singleton 保持合法。component 计算历史 stable-background 距离、free/unknown 比例和
  tentative/confirmed track overlap，分类为 B/F/U/T。
- B component 必须与 stable background 邻接或小于 0.8 m attach distance 且 track overlap
  低，使用 supported evidence 快速扩展；F、T component 完全不写 background；U 暂时沿用
  保守 voxel persistence，正式 candidate manager 留给 Phase 6。
- 新增 B/F/U/T component 诊断；component seed 改用有序集合，避免新稳定背景的同 epoch
  扩展结果依赖 unordered hash 遍历顺序。
- 新单测验证 stable adjacency expansion、free-space violation 不被吸收、track-explained
  singleton 不写 background。局部 build 3/3 成功且无 warning，54 tests 零失败。
- S03 moving-observer run 在 246 epochs 中得到 B/F/U/T = 5067/41/248/373；static-background
  recall 0.080→0.180，p95 67.2→48.6 ms。event count/precision 完全不变为
  6405/0.99797；TP 700→762、FN 72→10，但 raw-event birth 仍造成 1254 FP，留给 Phase 7–9。
- 全工作区 `catkin build` 11/11 成功且无 build warning；串行 tests 11/11 成功，
  `catkin_test_results build` 汇总 354 tests、0 error/failure/skipped。

## 2026-08-21 — SOFT-VoFOD V2 Phase 6 cold-start candidate background

- U component 进入 component-level persistence manager，保存 centroid、voxel union、首次/
  最近观测、epoch 数和 Welford centroid variance；统一使用 3 epochs、1.0 s、0.15 m sigma
  标准，不再直接累积每 voxel background hits。
- U candidate 延伸到历史 free 的 F component 时继续保持 unresolved；低方差 candidate 晋升
  background，高方差 candidate 立即释放，超时 candidate 丢弃。candidate voxel index 用于在
  ray classification 中暂缓 unresolved raw returns，不创建 map support。
- SOFT 固定 `warmup_duration_s` 从 8 s 改为 0；readiness 由在线背景持久性产生，不读取 truth、
  scene name、target ID 或 scenario event。
- 新增 candidate/promoted/expired/unresolved-return 诊断；单测验证静态 U（包括后来落入历史
  free）只在持久性门限后晋升，移动 U 不晋升。局部 build 无 warning，58 tests 零失败。
- S06 第一次零 warm-up/1.0× run 因旧 raw-event birth 正反馈只有 44.8% coverage；加入 F↔U
  unresolved 竞争后为 51.4%，两份失败证据分别保存在 `artifacts/v2_phase6_unresolved_bug/`
  和 `artifacts/v2_phase6_unresolved_partial/`。
- 0.5× 诊断 run coverage 有效，candidate promotions/unresolved returns 为 4/8805；但旧
  raw-event 路径仍产生 7741 events、345 births、129 peak tracks，HOTA 0.071、p95 151 ms。
  该结果不算实时/质量通过，明确要求 Phase 7 将 birth 输入改为 epoch-classified F packets。
- 全工作区 `catkin build` 11/11 成功且无 build warning；串行 tests 11/11 成功，
  `catkin_test_results build` 汇总 358 tests、0 error/failure/skipped。

## 2026-08-21 — SOFT-VoFOD V2 Phase 7 violation packetizer

- 删除生产路径的 `raw endpoint -> result event -> birth_buffer`；只有 map epoch 明确分类为 F
  的 component 才生成 violation packets，raw return 不再建立 quarantine/support。
- F returns 先按 30 ms 窗口，再用 0.75 m 3D 距离 connected components 聚合；允许
  singleton。packet 发布 start/end stamp、point count、全部 original indices、median centroid、
  mean ray/anomaly/free confidence、min background distance 与 3x3 covariance。
- packet covariance 使用 sensor variance + shape variance + sampling floor + 实际 spread，绝不
  按 `N` 相除产生虚假精度；旧 `Event`/birth 接口暂时以 packet centroid 兼容，避免复制 tracker。
- 单测覆盖 10 returns→1 packet、singleton→1 packet、超过 packet gate→2 packets；ROS
  integration 覆盖 deferred packet→trajectory birth。局部 build 无 warning，62 tests 零失败。
- S06/1.0× coverage 恢复有效；scored event count 7741→508，diagnostic packets 986，track
  peak 129→93，p95 151→112 ms，HOTA 0.071→0.086。旧 birth 仍把 packets 反复生成 285
  tracks/births，FP 16,839；Phase 8 必须增加 packet consumption 与 per-cell/epoch birth cap。
- ROS integration warm-up 改为覆盖至少三个完整 map epochs，不再依赖测试启动时绝对 ROS
  time 与 0.2 s 边界的偶然相位；修正后全工作区串行 tests 11/11 成功，
  `catkin_test_results build` 为 362 tests、0 error/failure/skipped（build 11/11、无 warning）。

## 2026-08-21 — SOFT-VoFOD V2 Phase 8 packet trajectory birth

- 保留既有 3D CV pair-seed/RANSAC 流程，但 inlier 和 refit 改用 packet covariance 的
  Mahalanobis gate；weighted refit 同时使用 anomaly 与 inverse mean variance，初始位置协方差
  来自 packet covariance + residual spread。
- 一个 packet 仍只能被一个 birth 消费；成功 birth 后，预测 footprint 1 m 内所有未消费
  packets 一并 suppression。新增 1 m spatial cell/0.2 s epoch 的单次 birth cap，并清理过期
  cell records。
- 新增 suppressed packets/cell-cap diagnostics；单测验证未消费 packet footprint 被清空和同
  cell/epoch 第二次 birth 被拒绝。局部 build 无 warning，66 tests 零失败。
- S06/1.0× run 为 94.3% coverage，按 95% gate 判失败并保存到
  `artifacts/v2_phase8_realtime_fail/`；births 285→243、suppressed packets 880，但伪 birth
  分散在不同 wall cells，cell cap 未触发，不能靠收紧单 cell 参数解决。
- S06/0.5× 诊断 run coverage 有效：563 scored packets、244 births、98 peak tracks、HOTA
  0.084、p95 128 ms；maintenance 仍向所有 raw non-background returns 开放，Phase 9 必须
  改为 packet-only GNN。该结果不算实时/质量通过。
- 全工作区 `catkin build` 11/11 成功且无 build warning；串行 tests 11/11 成功，
  `catkin_test_results build` 汇总 366 tests、0 error/failure/skipped。

## 2026-08-21 — SOFT-VoFOD V2 Phase 9 packet-level maintenance + GNN

- tracker maintenance 输入从所有 non-background raw returns 收紧为 map epoch 的 F/U/T
  packets；raw return 只保留 map classification、opportunity 与 confirmed support protection，
  不再直接更新 KF 或作为 GNN anchor。
- association unit 改为 packet，继续使用 Mahalanobis gate + Hungarian（A2 ablation 保留
  greedy）；innovation covariance 使用 packet covariance，并按 packet 到 epoch boundary 的延迟
  做 CV time alignment。每个 packet 至多分配给一条 track，只有未分配 F packet 可进入 birth。
- existence 仍逐 scan 做 survival，但 hit/miss 只在 5 Hz packet detector epoch 提交时更新，避免
  10 Hz scan 中间帧被错误计为 detector miss。新增 F/U/T maintenance packet 诊断。
- 新增 raw return 不立即维护、U packet 在 epoch 边界可维护已有轨迹的单测；ROS integration
  验证 packet trajectory birth。局部 build 无 warning，68 tests 零失败。
- S06/1.0× run coverage 100%，p95 84.3 ms，首次回到 100 ms 实时门槛内；相对 Phase 8 的
  0.5× 有效诊断 run，births 244→109、peak tracks 98→53、FP 17769→9146、HOTA
  0.084→0.114。结果仍有 53 条末帧轨迹和大量墙面 FP，明确留给 confirmed-only map
  protection 与根因实验，不能判作最终质量达标。
- 全工作区 `catkin build` 11/11 成功且无 build warning；串行 tests 11/11 成功，
  `catkin_test_results build` 汇总 368 tests、0 error/failure/skipped。

## 2026-08-21 — SOFT-VoFOD V2 Phase 10 opportunity optimization

- 每个 10 ms micro-batch 只建立一次稀疏 unit-direction grid；每条 track 用包含 physical
  radius、全部 sigma offsets、batch 内 target motion 与 observer origin motion 的保守角锥查询
  nearby rays。候选内仍逐 ray 执行原 7 sigma-point intersection、真实 return 遮挡、confirmed
  front-track 遮挡和 log-domain `P_D`，没有降频或用近似 opportunity 替换。
- 新增 `opportunity_full_scan_rays`/`opportunity_candidate_rays` 诊断和离轴 ray 回归测试；测试
  验证剔除 100 条不相关 ray 后 `P_D` 与只含真实相交 ray 的结果完全一致。
- 严格对照暴露 source bag 在同一 record timestamp 下最多突发 12 对 cloud/rays；SOFT 的旧
  ExactTime queue=8 会丢开头输入。队列按实测 burst 加处理余量固定为 32，runner 给 rosbag
  advertise 1 s 连接时间，并要求 `first_input_ack` 不得晚于 source 首帧。4.627/4.427 s 起始的
  run 现判为 `INVALID_INPUT_HANDSHAKE`，有效对照两边均从 4.228 s 开始。
- S06/1.0× brute-force 与 indexed 的 HOTA/TP/FP/FN、events 和 map metrics 完全一致；
  opportunity candidate rays 142,195,501→204,104（0.144%），tracking mean
  24.3→5.88 ms，runtime p95 66.8→49.5 ms。两侧 coverage 均为 100%。
- 局部 `soft_vofod_mid360` build 无 warning，70 tests 零失败；evaluation runner 9 tests
  零失败。
- 全工作区 `catkin build` 11/11 成功且无 build warning；串行 tests 11/11 成功，
  `catkin_test_results build` 汇总 370 tests、0 error/failure/skipped。

## 2026-08-21 — SOFT-VoFOD V2 Phase 11 confirmed-only map protection

- 强 map support 现在只来自 confirmed tracks 与 recently-deleted-confirmed quarantine；半径为
  physical target radius + capped covariance margin。只有这些 support 能截断 VALID/NO_RETURN
  free carving、完全跳过 endpoint background promotion 并生成 T component。
- tentative 只保留 physical-radius weak overlap：不截断 free carving、不 quarantine endpoint。
  若邻接已知背景仍归 B，但该 voxel 只加普通 background weight（1，而不是 supported weight 5
  与立即晋升）；否则归 U，由 packet maintenance 与 spatial-persistence competition 决定，静止
  满 1 s 后仍可晋升背景。
- 新增 strong/weak support 分离诊断和 3 个回归测试，覆盖 tentative 不截断 NO_RETURN、非背景
  weak overlap 进入 U 后可被静态持久性吸收、已知背景邻接只用弱权重。局部 build 无 warning，
  76 tests 零失败。
- S06/1.0× 首帧均为 4.228 s、coverage 100%。相对 Phase 10，target contamination 同为
  0.025，但 HOTA 0.155→0.148、FP 4874→5374、static-background recall
  0.0507→0.0323，p95 49.5→50.4 ms。第一版把所有 tentative overlap 强制归 U 的结果几乎
  相同，仅少 1 个 B component，说明退化主因是移除 tentative free-carving/endpoint 强保护，
  而非弱 B 权重分支。该语义修正按规范保留，回归进入 Phase 12 根因闭环，不包装为性能提升。
- 全工作区 `catkin build` 11/11 成功且无 build warning；串行 tests 11/11 成功，
  `catkin_test_results build` 汇总 376 tests、0 error/failure/skipped。
