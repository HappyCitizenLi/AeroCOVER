# SOFT-VoFOD V2 实施前冻结审计

状态日期：2026-08-21  
范围：V1 B0/A1/A2/A3 源码、配置、launch、runner、最终 N0/seed1001 artifacts  
目的：冻结 V2 修改前的真实状态；本文件不包含 V2 实验结果

## 1. 仓库与版本状态

工作区是 Git worktree，但 `master` 是 unborn branch：`git rev-parse --verify HEAD` 返回
`fatal: Needed a single revision`。因此当前 B0/A1/A2/A3 **没有真实工作区 commit**；run manifests
中的 `git_commit: uncommitted-no-commit` 与实际一致。全部 384 个非 ignored 文件目前均未跟踪。

固定上游来源为：

| 本地包 | 上游 commit |
|---|---|
| `mid360_simulation_plugin_fork` | `dca0420fee13e4dda40f8a609351f64da0a2b7ed` |
| `vofod_mid360` | `7da9f33a878a586588f6a626b75cfeacac7824f7` |
| `lidar_tracker_mid360` | `a92b4db61060b47f1af6dcce122188ec021f2dcd` |
| `livox_ros_driver2_rayfork` | `6b9356cadf77084619ba406e6a0eb41163b08039` |

V1 关键文件冻结 SHA-256：

| 文件 | SHA-256 |
|---|---|
| `include/soft_vofod_mid360/core.h` | `4b8f533ce4acce5fa7d702fec20b5bfda9a2aa24ac7aec9b2dcd5c17a9e5abdf` |
| `src/soft_vofod_mid360/src/core.cpp` | `81b15ea217e6dc0e40fab93873b366cc1e52b0b006a63a8ef678341561b44c1b` |
| `src/soft_vofod_mid360/src/soft_vofod_node.cpp` | `fed9f2899fe55651793b1edb1d5116edc084f40ddbd4206c572c4fe031ae3ffb` |
| `src/soft_vofod_mid360/config/default.yaml` | `7bfac9ae573e0580861df2acbf74d742e13f4a60da72bf6e0bbbdceb66deb36c` |
| `src/soft_vofod_evaluation/scripts/run_benchmark.py` | `657344795f153298e6b44af35730495f3fdbe2412721e9e71f851ecb838f2930` |
| `src/vofod_mid360/src/vofod_nodelet.cpp` | `325519de77ec19c78aa055cbca183c989bec272c430b99566df68c153456d2b4` |

`artifacts/`、`build/`、`devel/` 和 catkin logs 已由 `.gitignore` 排除，不属于源码冻结内容。

## 2. V1 真实执行路径

### 2.1 Map update

`SoftVofodCore::processScan()` 将一个 20,000-ray bundle 切成 10 ms micro-batches，并在每个
micro-batch 调用 `processBatch()`：

1. 用旧 `BackgroundMap` 查询 endpoint；
2. tracking/existence/support；
3. `truncateBeforeSupport()` 后调用 `carveFreeRays()`；
4. 对未保护 endpoint 调用 `observeBackground()`。

`carveFreeRays()` 在 micro-batch 内按 voxel 汇总 ray segment evidence，但每个 micro-batch 都会
立刻写长期 `free_evidence`；它不是 0.2 s map epoch，也没有 ray-count saturation。

### 2.2 Event

`processBatch()` 对每条合法 return 单独检查：旧状态必须为 `CONFIDENT_FREE`、free probability
过门、离 stable background 不小于 exclusion distance。每个通过的 endpoint 立即生成一个
`Event`，所以当前语义是 `one endpoint -> one event`。

### 2.3 Birth buffer

未被现有 track 使用的 events 进入 `birth_buffer_`。`bestBirthCandidate()` 枚举两个不同时间组的
CV hypotheses，从每组选择一个最小 residual event 后 refit；`createBirth()` 消费入选 events。
buffer 时长/容量为 2 s/256，A1/A2/A3 最小组数为 2/3/3。每个 micro-batch 最多 birth 8 条。

### 2.4 Support 创建、使用与删除

- `supports()` 为每条非 deleting track 建立 `target_radius + capped covariance inflation` support；
- deleting track 在 existence/timeout 删除时通过 `addQuarantine()` 建立 2 s support；
- A3 对**每个 raw event**调用 `addQuarantine()`，建立 0.75 m、2 s support；
- `prune()` 线性删除过期 `quarantines_`；
- `truncateBeforeSupport()` 对每条 free ray 线性遍历全部 supports；
- endpoint protection 再对每个 valid endpoint 线性遍历全部 supports。

当前没有 support spatial index，也没有 diagnostics `support_count`。S07 support 数只能由 event
timestamps、2 s lifetime 和 track count 后验重建。

### 2.5 Association

A1/A2 依 track 顺序 greedy 分配 raw endpoint anchor；A3 用矩形 shortest-augmenting-path
Hungarian。剩余 raw returns 在 0.75 m 内分配给最近 anchor，逐轴 median 后做 CV-KF correction。
maintenance pool 是所有“不与 stable background 一致”的 raw returns，不是 packet。

### 2.6 Opportunity

`opportunity()` 为每条 track 建立 7 个 position sigma points，然后遍历当前 micro-batch 所有
VALID_RETURN/NO_RETURN rays；可能相交时再遍历其他 confirmed tracks 做前景遮挡。当前最坏路径
仍接近 `tracks × rays × sigma_points × confirmed_tracks`，没有 angular/KD-tree index。

### 2.7 Existence 与 timeout

命中调用 `hitExistence()`，miss 调用 `missedExistence()`。当 `P_D=0` 时后者严格返回原 prior。
当前没有 survival prediction。所有状态共用 `hard_timeout_s=30`；没有 tentative deadline、
tentative/confirmed 分离 timeout 或 duplicate-track merge。

### 2.8 Startup

SOFT 节点从实际收到的第一帧开始 8 s warm-up，期间只禁 birth。B0 从实际收到的第一帧开始
10 s warm-up，并同时要求 map maturity；完成帧本身仍以 `warmup_active=true` 分类。

## 3. 当前参数冻结

### 3.1 SOFT V1 canonical

| 组 | 参数 |
|---|---|
| 输入/时间 | `micro_batch_dt=0.01 s`，sync queue 8，geometry tolerance 0.01 m，warm-up 8 s |
| map | center `(10,0,5)` m，size `(40,30,16)` m，voxel 0.5 m，evidence scale 5 |
| map gates | confidence 0.6，free/background probability 0.7/0.7，promotion 3 groups/1 s |
| map evidence | VALID free 1.0，NO_RETURN free 0.5，background 1.0 |
| carving | endpoint guard 0.5 m，target guard 0.2 m，NO_RETURN max 20 m |
| event | free probability 0.7，background exclusion/search 0.75/2.0 m，distance scale 1.0 m |
| birth | buffer 2 s/256，group 0.05 s，min groups 3，min duration 0.1 s |
| birth gates | pair dt 0.02–2 s，speed 15 m/s，residual 0.8 m，anomaly 1.5，suppression 1 m |
| KF | acceleration sigma 3 m/s²，measurement variance 0.1 m²，shape sigma 0.35 m |
| tracker | initial velocity variance 9，gate d² 11.345，anomaly weight 0.1，target radius 0.75 m |
| existence | birth/confirm/delete 0.6/0.8/0.1，clutter 0.001，hard timeout 30 s |
| feedback | support sigma 2，inflation cap 1.5 m，quarantine 2 s |
| opportunity | return probability 0.5，`P_D` cap 0.95，range 60 m，occlusion margin 0.15 m |

A1 overlay 只把 birth groups 改为 2，并关闭 opportunity/feedback/Hungarian；A2 使用 3 groups 并
关闭三项；A3 使用上述默认 3 groups 并开启三项。

### 3.2 B0 canonical

B0 map/cluster 为 0.5 m voxel、1.25 m component tolerance、2 minimum points、3 m max OBB；
VALID/NO_RETURN free weights 都是 0.003，endpoint guard/max distance 为 0.5/20 m。background
warm-up 为 10 s，map maturity 是 occupied ratio 0.01 加 24-sure connected voxels。

classic tracker 是 9-state CA-LKF，Q variance rates `[0.01,0.8,0.05]`，P0 variances
`[0.3,1.0,0.05]`，R 0.1 m²，uncertainty radius multiplier/min/max 1.5/0.75/5 m；至少 2 次 detector
hits 才 confirmed，并保留 local component update 与 overlap merge。

所有 V1 SOFT 参数都还是 engineering defaults；没有 CAL01–CAL05 独立标定依据。

## 4. S07/A3 冻结证据

来源：`artifacts/runs/A3/S07/N0/seed_1001/{metrics.json,output.bag}`，scoring 19 s/190 frames。

| 项目 | 冻结值 |
|---|---:|
| event count | 17,837 |
| mean / peak tracks | 84.62 / 230 |
| mean / peak estimated supports | 1,861.52 / 7,761 |
| mean / p95 / max total | 293.33 / 1001.64 / 1039.95 ms |
| mean / peak map commit | 208.30 / 792.36 ms |
| mean / peak tracking | 84.06 / 288.66 ms |
| processing load ratio | 2.933 |
| TP / FP / FN | 176 / 15,434 / 14 |
| HOTA / IDF1 | 0.0867 / 0.0178 |

estimated supports 是“最近 2 s events + 当前 tracks”的保守重建，不包含额外 deleting support，
因此不是节点直接发布值。峰值 support estimate 与 map-commit 时间相关系数为 0.999。

## 5. B0/S06 warm-up 冻结证据

| 事件 | stamp |
|---|---:|
| source first checked ray | 4.228 s |
| B0 first diagnostic / warm-up elapsed 0 | 5.427 s |
| target spawn / scoring start | 14.657 s |
| B0 warm-up complete frame | 15.428 s |

source manifest 的 target-free gap 是 10.429 s，但 B0 实际 warm-up gap 只有 9.230 s；完成比目标
spawn 晚 0.771 s。现有 scored-frame coverage 不检查 scoring 前输入，也不检查 warm-up complete
stamp，因此旧 run 被错误接纳为 valid。

## 6. 文档与代码/数据不一致

1. V2 提示词列出的 `/home/uav/lyk/SOFT_VOFOD_EXPERIMENT_REPORT.md` 和
   `/home/uav/lyk/CURRENT_MRS_VOFOD_MID360_IMPLEMENTATION.md` 不存在；真实文件位于 `docs/`。
2. `CURRENT_MRS_VOFOD_MID360_IMPLEMENTATION.md` 称 runner 必须在 B0 warm-up 门后引入目标；
   source bag 满足，但 S06/B0 replay 未满足，文档没有区分 source-level 与 run-level 合同。
3. `KNOWN_LIMITATIONS.md` 和旧阶段报告称 28-run matrix 已完成，但没有披露 S06/B0
   `INVALID_WARMUP`；当前实验报告已经披露，其他文档尚未同步。
4. runner 只等待 output topic advertise 并固定 sleep 1 s，没有 input-subscriber readiness、
   first-input ack 或 warm-up completion gate。
5. B0 `MapUpdateDiagnostics` 有 active/complete/elapsed，但没有显式 first-input/start/complete stamps；
   SOFT diagnostics 也没有 first-input/bootstrap-start ack。
6. 文档把 S07 support storm 数量作为分析结果，但节点没有直接输出 support count；它是后验估计。
7. V1 diagnostics 只有 classification/tracking/map-commit 粗分段，没有 packetization、assimilation、
   birth、opportunity、support-query 的独立 timing。
8. `FINAL_IMPLEMENTATION_REPORT.md` 的 10 packages/318 tests 是历史阶段值；当前最终记录是
   11 packages/339 tests。该文件已标为历史，但数字不能作为当前验证结果。
9. 当前代码没有提示词要求的 V2 canonical/calibration configs、B1–B4、NEG04、S08、CAL04/05
   或九份 V2 最终文档；这些是待实现项，不得在审计中写成已完成。

## 7. Phase 0 结论与进入门槛

V1 结构性根因已由源码和 artifacts 同时确认：raw endpoint event、per-event support、线性 support
query、无 survival、过宽 raw measurement pool，以及 micro-batch 长期 map commit。Phase 1 只修
replay 有效性并重放 S06/B0；在完成该门禁前不运行新大矩阵，也不修改旧结果来制造算法结论。
