# SOFT-VoFOD 精简版算法规范

状态日期：2026-08-21  
实现包：`src/soft_vofod_mid360`  
算法版本：A3 minimal final

## 1. 范围与输入信任边界

SOFT-VoFOD（Scan-Opportunity-aware Free-space Trajectory VoFOD）与 B0 并列，不修改
`vofod_mid360` 或 `lidar_tracker_mid360` 的语义。算法只订阅公共预处理器输出：

```text
/uav1/mid360/points_world    sensor_msgs/PointCloud2
/uav1/mid360/rays_checked    mid360_ray_msgs/CheckedRayBundle
```

ROS adapter 使用 ExactTime，并再次检查 world frame、逐点 `scan_id`、唯一且 source-order
一致的 `original_index`、`offset_time_ns`、retained VALID_RETURN 一一对应、单位方向及 endpoint 误差。
ROS1 publisher 可能重写 `Header.seq`，所以它不作为业务 scan identity。空点云由 ExactTime
与 preprocessor 已验证的 `CheckedRayBundle` 配对；非空点云则以逐点 `scan_id` 复核。
source VALID_RETURN 若被公共 preprocessor 的 self/range filter 合法移除，则本算法和 B0 一样
将该 ray 标为不可用：既不跟踪，也不 carving；不会因此拒绝整帧。

算法没有 truth subscriber，也没有 scene、target ID、墙坐标或高度特判。

## 2. 单条处理链

```text
raw Mid360
 -> scan identity + world endpoint preprocessing
 -> 8 s map-only target-free initialization
 -> 10 ms micro-batch
 -> old conservative-background query
 -> free-space violation event
 -> existing-track GNN/Hungarian association
 -> robust packet median + CV-KF correction
 -> unassociated event trajectory birth
 -> per-ray sigma-point scan opportunity
 -> Bernoulli existence update
 -> active/deleted target quarantine support
 -> truncated VALID_RETURN/NO_RETURN free carving
 -> endpoint background commit
 -> events/tracks/map/debug output
```

每个 micro-batch 严格执行 `read old map -> tracking -> support -> map commit`。当前 endpoint
不会先写入地图再用于判断自身异常。

## 3. Conservative background map

`BackgroundMap` 使用 B0 已测试的 `vofod::VoxelMap` 几何和 Amanatides-Woo DDA，但保存独立的
`BackgroundVoxel`：free/background evidence、独立时间组计数、候选首末时间、quarantine
截止时间和四态枚举。

状态只有：

- `UNKNOWN`：证据不足；
- `CONFIDENT_FREE`：置信度和自由概率同时过门；
- `CANDIDATE_BACKGROUND`：未知区的重复静态 endpoint，尚未成熟；
- `STABLE_BACKGROUND`：独立时间组、持续时间、背景概率和置信度全部过门。

VALID_RETURN 自由更新终点为 `range - endpoint_guard_m`；NO_RETURN 最远只到
`max_no_return_free_range_m`，且其权重不大于 VALID_RETURN。两者若先碰到 active target 或
quarantine support，均在 `target_near - target_guard_m` 截止。

coarse voxel 中，grazing ray 可能在 endpoint guard 前经过另一个静态 endpoint 的同一格；
candidate/stable-background voxel 因而不再接收这类 free evidence，避免地面永远无法成熟。

endpoint 提交规则：

- event、已关联点或 target support 内点：不加背景证据，刷新 quarantine；
- stable-background-consistent 点：按独立时间组刷新背景证据；
- 未被 target 解释的 unknown 点：进入 candidate，满足成熟条件后才转 stable；
- quarantine 未到期时：不累计 candidate evidence。

因此已在 confident-free 空域出生并悬停的目标不会被逐帧吸收为背景。

## 4. Free-space violation event

每个合法 world return 查询旧地图。只有 endpoint voxel 是 `CONFIDENT_FREE`、自由概率过门，
且与最近 stable background 的距离不小于 exclusion radius 时才产生 event。event 保留真实
ray time、scan ID、source index、位置、射线方向、自由置信度、背景距离和 anomaly score。

event 是 birth 证据，不是 confirmed track。单个 event 不会直接创建轨迹。

## 5. Trajectory-before-confirmation birth

未被 existing track 使用的 events 进入有限时长、有限容量 buffer。事件按
`floor(ray_time / event_group_dt)` 分成独立组。

第一版采用确定性的两点 CV hypotheses：枚举来自不同组且满足时间/速度门的事件对，再对
整个窗口选取每组 residual 最小的一个 inlier，完成加权 CV refit。必须同时满足：

- 至少三个 independent groups；
- 最小时间跨度；
- 最大速度；
- 最大 residual RMS；
- 最小累计 anomaly score；
- 不与现有 track birth-suppression support 重叠。

零速度是合法 CV，所以 hover 可以 birth。成功后只创建 `TENTATIVE` track，并移除已消费的
组证据；buffer 上限 256 是明确的计算量边界。

## 6. CV-KF 与 GNN

track 使用 6-state `[position, velocity]` CV-KF，过程噪声按真实 micro-batch `dt` 的 white
acceleration 模型缩放。measurement covariance 为 sensor fallback variance 加 shape
inflation。

maintenance measurements 是所有“不与 stable background 一致”的 VALID_RETURN，不要求仍是
event。因此目标进入 unknown 或 anomaly score 下降后，已有轨迹仍可维护。

association 使用 Mahalanobis gate 和 Hungarian 全局一一匹配。实现使用带每轨 unmatched
dummy 列的矩形 shortest-augmenting-path，不把 `R×C` 补成 `max(R,C)²` 方阵。Hungarian 先给每条 track 一个
anchor；其余局部点只分配给最近 anchor 一次，在 `target_radius_m` 内取逐轴 median。singleton
直接更新，两个 track 不能共享一个 return。

## 7. Scan opportunity 与 existence

每条 track 的 3D position Gaussian 使用 mean 和三条主轴的正负 sigma points。对每一条真实
VALID_RETURN/NO_RETURN ray 判断有限 segment 是否与固定物理 target sphere 相交，不把整个
covariance 变成无限膨胀 sphere。

以下情况不计 opportunity：

- 射线不相交；
- 当前真实 return 位于 target near range 之前；
- 另一条 confirmed track 的预测 support 位于更前方。

NO_RETURN 射线仍可提供扫描机会。micro-batch `P_D` 由独立乘积模型累计并受全局 cap 限制。
无匹配时只有 `P_D>0` 才降低 existence；`P_D=0` 保持原值。命中时使用高斯 likelihood 与固定
clutter density 的 log-domain Bernoulli update。实际匹配本身证明存在一次观察机会，因此命中
更新的 `P_D` 下限为全局 `return_probability`，防止离散 sigma geometry 把真实命中算成零机会。

track 通过 `confirm_threshold/delete_threshold` 滞回在 tentative/confirmed/deleting 间转换；
`hard_timeout_s` 只是极长保险，并在输出中记录 `hard_timeout` reason。

## 8. Quarantine 与安全门

地图 support 半径是物理半径加有上限的位置 uncertainty inflation；opportunity 始终使用物理
半径。active track、event 和刚删除 track 都保护背景；删除后的固定 support 保持
`quarantine_duration_s`。

adapter 拒绝重复/回退 stamp。检测到超过 `max_time_jump_s` 的正向时间跳变时，该 scan 仍可
track predict 和 opportunity update，但暂停 endpoint background evidence 与 event birth，并在
diagnostics 中明确原因。目前没有 pose covariance 输入，因此 TF discontinuity/covariance gate
仍是公开限制。

从首个合法 scan 起默认先运行 8 s map-only initialization；这段时间仍用真实射线建立背景，
但不产生 event/birth，也不读取 scenario event 或 truth。ROS 两侧 subscriber 和 ExactTime
synchronizer 共用配置的 `sync_queue_size`。

## 9. 输出与运行

默认发布：

```text
/soft_vofod/events
/soft_vofod/tracks
/soft_vofod/background_voxels
/soft_vofod/free_voxels
/soft_vofod/candidate_background_voxels
/soft_vofod/opportunity_debug
/soft_vofod/diagnostics
```

地图点云 intensity 是观测置信度。track 输出 ID、位置、速度、6x6 covariance、existence、
状态、age、positive updates、累计有效机会、距上次 measurement 时间和删除原因。

```bash
source /opt/ros/noetic/setup.bash
source /home/uav/lyk/devel/setup.bash
roslaunch soft_vofod_mid360 soft_vofod.launch
```

该 launch 只启动算法；sensor/preprocessor 由调用者提供，以便 B0 与 SOFT-VoFOD 消费同一份
不可变输入。
