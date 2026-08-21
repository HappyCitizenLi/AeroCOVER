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
