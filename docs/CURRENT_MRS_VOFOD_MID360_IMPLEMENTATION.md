# 当前 MRS / Mid360 / VoFOD 实现与上游差异总结

状态日期：2026-08-21  
工作区：`/home/uav/lyk`  
阶段：Phase 0–10；SOFT-VoFOD 算法、S01–S07 开发消融与标准化评测完成

## 1. 当前生产链

```text
Mid360 points_raw + rays_raw + scan_identity
  -> mid360_ray_preprocessor
       ├─ points_valid（legacy source-frame 调试输出）
       ├─ points_world（canonical world-frame 输入）
       └─ rays_checked
  -> VoFOD-Mid360 B0
  -> classic lidar_tracker_mid360
```

唯一 B0 主入口是 `roslaunch tclv_evaluation b0_canonical.launch`。算法节点不订阅 truth。
同一公共输入现在也可进入独立 proposed 链：

```text
points_world + rays_checked
  -> soft_vofod_mid360
       conservative background -> event -> trajectory birth
       -> CV-KF/Hungarian -> scan opportunity/existence -> quarantine
```

SOFT-VoFOD 已实现 GNN/Hungarian，但按精简范围不实现 JPDA/MHT/PMBM 或神经网络。

本 overlay 没有修改 `/opt/ros/noetic` 或 `/home/uav/mrs_mid360_ws`，也没有验证
MRS/PX4 飞控闭环、气动或避障。

## 2. 上游边界

| 本地部分 | 固定上游 commit |
|---|---|
| `mid360_simulation_plugin_fork` | `dca0420fee13e4dda40f8a609351f64da0a2b7ed` |
| `vofod_mid360` | `7da9f33a878a586588f6a626b75cfeacac7824f7` |
| `lidar_tracker_mid360` | `a92b4db61060b47f1af6dcce122188ec021f2dcd` |
| `livox_ros_driver2_rayfork` | `6b9356cadf77084619ba406e6a0eb41163b08039` |

详细导入、license 和再分发边界见各目录 `UPSTREAM.md`。

## 3. 显式 scan identity

`RayBundle.scan_id` 是主键。不能把 ROS1 `Header.seq` 当业务主键，因为 publisher 会维护
自己的 sequence。生产端因此发布 `ScanIdentity` companion，包含：

- `scan_id`、`pattern_start_index`；
- `point_count`、`ray_count`；
- `point_source_stamp`、`ray_source_stamp`。

预处理器用 ExactTime 接收 cloud、RayBundle、ScanIdentity，并验证：

- metadata 与两侧 source header/count/pattern 一致；
- raw cloud 与 ray count 一致，source frame/stamp 一致；
- source order 严格保持，无 duplicate/out-of-range `original_index`；
- 每个 `VALID_RETURN` 有唯一合法源点，range 在
  `0.01 m + 0.001*r` 容差内；
- retained point 只能对应 `VALID_RETURN`；
- stamp 严格前进，时间回退拒绝。

任何 pair、TF 或 world endpoint 合同失败时，`points_valid`、`points_world`、
`rays_checked` 均不发布。诊断包含提示词要求的全部 scan-pair 字段。

## 4. 世界点云与运动补偿

`points_world` 是标准 `sensor_msgs/PointCloud2`，字段为：

```text
x y z intensity original_index offset_time_ns scan_id
```

每个 retained `VALID_RETURN` 直接使用：

```text
p_world = CheckedRay.origin + Ray.range * CheckedRay.direction
```

VoFOD 读取该坐标并在 1 cm 内复核 checked endpoint；tracker 直接使用 world cloud，删除了
旧 sensor-frame 整束 TF 和 tracker 内 self crop。preprocessor 已统一完成 self exclusion。

`snapshot` 使用 scan-start TF；`per_ray_pose` 使用首尾 TF 插值 observer pose。后者仍不重放
bundle 内移动 collision scene，准确边界见 `docs/ROLLING_SCAN_LIMITATION.md`。

## 5. Gazebo 与硬件 source

Gazebo 插件继续可选发布 legacy cloud ABI，同时发布完整 RayBundle 和 ScanIdentity。source
stamp 对三者一致；PointXYZ OpenMP 分支按 ray source index 原位写入，不再 nondeterministic
append。

硬件 spherical fork 在 cloud frame flush 边界发布 RayBundle/ScanIdentity，并把内部 scan_id
传到 cloud 生成链。物理 Mid360 的 exact zero-depth direction 资格仍未验证；能力探针通过两次
断电周期前只能使用 `calibrated_fallback`，不能进入当前 `sim_exact` B0 配置。

## 6. B0 detector/map

B0 保留上游风格的单标量 occupancy score、单尺度 component、历史背景 close/far、两个背景
成熟门、6-neighbor floating exploration、frontier mutation、separated-background cleanup 和
OBB-center detection。

当前 canonical 参数：

- map center/size `(10,0,5)` / `(40,30,16) m`，voxel `0.5 m`；
- cluster tolerance/min points/max OBB `1.25 m / 2 / 3.0 m`；
- background ratio `0.01`，sure component 至少 24 voxels；
- VALID/NO_RETURN free weight 均为 `0.003`；
- endpoint guard `0.5 m`，free/no-return 最大可靠距离 `20 m`。

map score 初值/unknown `-740`、point `0`、free target `-1000`；sure/new/frontier 阈值为
`-0.1/-300/-750`。`background_points` 是 sure occupied voxel，`free_voxels` 是分数低于
初值的自由更新可视化；二者语义不可互换。

## 7. 启动背景

`nadir_seed` 已完全删除。canonical B0 使用 target-free background warm-up：默认至少 10 s，
且 historical occupied ratio 和 24-sure connected component 两门均达到后才发布
`background_warmup_complete=true`。场景/runner 必须在该门之后引入 target 和计分。

算法不知道 target 是否存在，不订阅 truth；如果 source 在 warm-up 期间已有目标，污染背景是
输入协议违规，也是需要公开的限制。

## 8. classic tracker

tracker 保留：

- 9-state CA LKF、greedy nearest association；
- local radius crop、Euclidean cluster、OBB、singleton cluster；
- original-style uncertainty deletion 和 overlap merge；
- 至少两次 detector hit 的 confirmation；
- points/detections bounded FIFO 与 frame completion barrier。

canonical 参数为 Q variance rate `[0.01,0.8,0.05]`、P0 variance
`[0.3,1.0,0.05]`、R position variance `0.1 m²`、radius multiplier/min/max
`1.5/0.75/5.0 m`、downsample `0.5 m`、local tolerance/max OBB `1.0/1.0 m`。

协方差半径已修正为 `multiplier*sqrt(cbrt(det(P_position)))`，单位为米。background filter
canonical 固定开启，只接 `vofod_mid360/background_points`。

## 9. 仿真与评测

`mid360_multi_uav_sim` 仍通过 `/gazebo/set_model_state` 写确定性参考位姿，可作为 sensor/
perception fixture，不能证明飞控性能。新增通用 timeline controller 和 S01–S07 contracts；
target 在 target-free warm-up 后才生成。`tclv_evaluation` visibility truth 增加 present/range/FOV/
line-of-sight/actual-return 字段，truth 只进入 evaluation namespace。

`soft_vofod_evaluation` 提供 source-bag SHA 固定的 record-once/replay-many runner、B0/A1/A2/A3
开关、严格一对一 HOTA@1m/IDF1/GOSPA/TTFT/IDSW、opportunity/map/runtime 指标、95% coverage
门禁和聚合输出。已执行 N0/seed1001 的 7 source/28 run 开发矩阵。结果没有证明 A3 优于 B0，
S07/A3 还存在严重伪事件和非实时开销；完整数字及统计边界见
`docs/SOFT_VOFOD_EXPERIMENT_REPORT.md`。

## 10. SOFT-VoFOD A3

`soft_vofod_mid360` 是一个算法包，复用 B0 的 voxel geometry/DDA，但地图、event、track 和
existence 状态完全独立。它实现：

- 四态 conservative background evidence 和独立时间组 background promotion；
- confident-free violation event，event/target endpoint 不写背景；
- 至少三组一致 event 的 weighted CV trajectory birth，hover 合法；
- 6-state CV-KF、真实 `dt` white-acceleration Q、Mahalanobis/Hungarian 一一关联；
- singleton 与局部 median packet，不做全局 DBSCAN；
- 七点 sigma opportunity、真实 return/前方 confirmed track 遮挡、NO_RETURN opportunity；
- Bernoulli hit/miss existence、confirm/delete hysteresis、显式 hard-timeout reason；
- active/event/deleted-track quarantine 和 VALID/NO_RETURN target-near truncation；
- time rollback reject 与大时间跳变的 background/birth pause。

默认入口是 `roslaunch soft_vofod_mid360 soft_vofod.launch`；它只启动 proposed 算法，不重复
启动 sensor/preprocessor，也不订阅 truth。详细算法、公式和边界见
`docs/SOFT_VOFOD_ALGORITHM.md`、`docs/SOFT_VOFOD_FORMULAS.md`、
`docs/KNOWN_LIMITATIONS.md`。

## 11. 文档与运行

- 实施前审计：`docs/PRE_IMPLEMENTATION_AUDIT.md`
- 冻结规范：`docs/B0_FROZEN_SPEC.md`
- 参数来源：`docs/B0_PARAMETER_PROVENANCE.md`
- 变更记录：`docs/IMPLEMENTATION_CHANGELOG.md`
- rolling-scan 限制：`docs/ROLLING_SCAN_LIMITATION.md`
- SOFT-VoFOD 算法：`docs/SOFT_VOFOD_ALGORITHM.md`
- 公式映射：`docs/SOFT_VOFOD_FORMULAS.md`
- 已知限制：`docs/KNOWN_LIMITATIONS.md`
- 第二阶段报告：`docs/FINAL_IMPLEMENTATION_REPORT.md`
- S01–S07 实验、消融和结果：`docs/SOFT_VOFOD_EXPERIMENT_REPORT.md`

```bash
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
catkin run_tests --no-status -j1  # Gazebo fixtures 必须串行
roslaunch tclv_evaluation b0_canonical.launch
# 或在同一公共 source/preprocessor 后启动 proposed：
roslaunch soft_vofod_mid360 soft_vofod.launch
```

建议人工创建 tag `b0-mid360-frozen`；本工作区尚无提交，本阶段不会自动 tag 或 push。
