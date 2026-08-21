# Mid-360 仿真插件首轮修改说明

## 范围与来源

插件修改本身只实现统一原始射线接口、兼容输出和可重复的仿真验证；预处理已在独立
`mid360_ray_preprocessor` 包中实现，遮挡推理和跟踪不进入传感器插件。

受控源码位于：

```text
src/mid360_simulation_plugin_fork/livox_laser_simulation
```

它来自 `ctu-mrs/Mid360_simulation_plugin` 的提交
`dca0420fee13e4dda40f8a609351f64da0a2b7ed`。来源和复制范围见
`src/mid360_simulation_plugin_fork/UPSTREAM.md`。既有
`/home/uav/mrs_mid360_ws`、旧 Livox 工作空间和 `/opt/ros/noetic` 均未修改。

## 新消息和话题

新增 catkin 包 `mid360_ray_msgs`，定义：

```text
mid360_ray_msgs/Ray
mid360_ray_msgs/RayBundle
mid360_ray_msgs/ScanIdentity
```

工程测试模型发布：

```text
/uav1/mid360/points_raw       sensor_msgs/PointCloud2
/uav1/mid360/rays_raw         mid360_ray_msgs/RayBundle
/uav1/mid360/scan_identity    mid360_ray_msgs/ScanIdentity
/uav1/mid360/ray_source_diagnostics diagnostic_msgs/DiagnosticArray
```

完整字段约定见 `RAY_BUNDLE_INTERFACE.md`。

## 向后兼容策略

RayBundle 是独立的可选发布分支，`publish_ray_bundle` 默认 `false`。未配置任何新 SDF
参数时，插件仍只进入原 `publish_pointcloud_type` 分支。兼容模型保持以下接口：

```text
topic:       /livox/lidar
type:        sensor_msgs/PointCloud2
frame:       base_link
rate:        配置目标 10 Hz
width:       20000
point_step:  32
fields:      x, y, z, intensity, tag, line, timestamp
```

兼容模式下没有 `/uav1/mid360/rays_raw` 或 ray diagnostics 话题。原点云对无效/无返回
样本写零端点的行为没有改变。

## 新 SDF 参数

| 参数 | 默认值 | 作用 |
|---|---|---|
| `publish_ray_bundle` | `false` | 启用 RayBundle、ScanIdentity 和源级诊断 |
| `ray_bundle_topic` | `rays_raw` | RayBundle 话题 |
| `scan_identity_topic` | 从 ray topic 推导 | legacy cloud 与 `scan_id` 的 companion |
| `ray_bundle_frame` | `frameName` | 所有方向所在的物理传感器坐标系 |
| `ray_diagnostics_topic` | 从 ray topic 推导 | 源级 `DiagnosticArray` 话题；工程模型显式设为 `ray_source_diagnostics` |
| `use_csv_time` | `true` | 尝试使用经验证的 CSV 秒单位时间 |
| `ray_point_rate` | `200000` | uniform fallback 的物理点频 |
| `ray_time_geometry_mode` | `snapshot` | `snapshot` 或有界的 `per_ray_pose`；默认兼容路径不变 |
| `per_ray_pose_static_scene_opt_in` | `false` | 仅在显式为 `true` 时允许 `per_ray_pose`，确认碰撞场景在一帧内静止 |

未知几何模式会 fail-closed 并拒绝插件加载；`per_ray_pose` 缺少显式静态场景 opt-in
也会拒绝加载，不会静默回退。无效点频回退到 200 kpoint/s。相对 CSV 路径使用
`ros::package::getPath("livox_laser_simulation")` 解析，不再依赖编译机器的 `__FILE__`
路径。

## 射线方向

RayBundle 故意复用 raycast 原有的方向运算：

```cpp
rotation.Euler(0.0, rotate_info.zenith, rotate_info.azimuth);
direction = rotation * ignition::math::Vector3d::UnitX;
```

加载器已把 CSV 第三列从相对 `+Z` 的极角变换为
`rotate_info.zenith = csv_theta - pi/2`。因此最终传感器系方向为：

```text
[sin(theta) cos(azimuth), sin(theta) sin(azimuth), cos(theta)]
```

第 0 条 CSV 方向锁定为约
`(0.010812894, 0.613335663, 0.789748343)`，Z 为正。验证器按每条 Ray 的
`pattern_index` 核对完整 800,000 条方向序列，而非只检查首条。

## 返回状态

分类发生在 legacy 点云清零之前，使用未修改的 `raw_range`：

```text
!finite(raw_range) or raw_range <= 0 -> INVALID_RANGE
0 < raw_range <= min_range           -> BELOW_MIN_RANGE
raw_range >= max_range - 1e-6        -> NO_RETURN
otherwise                            -> VALID_RETURN
```

只有 `VALID_RETURN` 携带可用 `range` 和有限 intensity；其他状态的消息 range/intensity
均为零。无返回射线仍携带有限、非零、单位方向。本 Gazebo 传感器插件只发布原始观测，
不拥有占据地图，因此不会在插件内部执行 free-space carving。这不是禁止系统使用无返回
射线：当前下游 VoFOD-Mid360 B0 已根据 `return_status == NO_RETURN`、单位方向、
非零软更新权重和配置距离更新自由空间，不能从零 range 或 legacy 零端点推断终点。

## 时间、索引和几何模式

- 默认 `snapshot` 分支保留原来的 `SetPoints` 方程、时间戳位置和一次
  `rayShape->Update()` 顺序；未配置新参数的 legacy 点云路径不变。
- 显式 `per_ray_pose` 在扫描开始时读取父 link 的 world pose、world linear/angular
  velocity，并用 constant world-frame twist 外推每条 observer ray 的原点和方向。它不
  是从轨迹逐点采样，也不建模加速度、jerk 或扫描内 twist 变化。
- `per_ray_pose` 在同一 Gazebo physics recursive mutex 内冻结父 link，捕获权威
  `world->SimTime()` 作为 `header.stamp`，构造全部射线并执行 ODE
  `rayShape->Update()`。这避免 pose/twist 捕获和碰撞查询之间额外推进一段物理运动。
- `per_ray_pose` 仍只执行一次碰撞更新，因此只对扫描内静止的碰撞场景成立；移动墙、
  移动目标或其他动态 collision 不受支持。显式
  `per_ray_pose_static_scene_opt_in=true` 是对这一限制的确认，而不是解除限制。
- `offset_time_ns` 从零开始。uniform 模式为
  `round(i * downsample / ray_point_rate * 1e9)`；20,000 条、200 kpoint/s 的末条偏移
  为 `99,995,000 ns`。
- CSV 时间只有在有限、严格单调、步长和完整周期均与点频在 20% 内一致时才使用。
  当前文件的时间列为整数 `1..800000`，因此明确记录
  `uniform_time_fallback`，不会误当成秒。
- `pattern_index` 是 downsample 前的 CSV 模循环索引；`pattern_start_index` 与首条
  一致，`scan_id` 每个传感器更新递增。
- legacy cloud、RayBundle 和 ScanIdentity 使用同一个 source stamp；ScanIdentity
  显式绑定 scan ID、pattern、两侧 count/stamp，不依赖 publisher-managed Header.seq。
- RayBundle 的方向字段仍在物理 sensor frame 中；逐射线 observer pose 只改变 Gazebo
  碰撞射线的扫描起点 link-frame 几何，避免把 world rotation 重复应用到消息方向。

## 诊断

启用 RayBundle 时每帧在 `ray_source_diagnostics` 发布一个
`DiagnosticArray`；`ray_diagnostics` 保留给预处理器的综合检查。源状态包含：

```text
ray_count
valid_return_count
no_return_count
below_min_range_count
invalid_count
direction_norm_error_max
timestamp_monotonic_ratio
bundle_duration
exact_direction_available=true
source_mode=sim_exact
ray_time_geometry_mode=snapshot
motion_model=snapshot
scene_assumption=instantaneous_snapshot
linear_speed_mps
angular_speed_radps
max_offset_sec
```

`per_ray_pose` 时几何语义变为 `ray_time_geometry_mode=per_ray_pose`、
`motion_model=constant_twist_world_velocity` 和
`scene_assumption=static_scene_only`，并报告扫描起点速度和最大偏移。诊断 header 与
对应 RayBundle header 完全一致。

诊断 message 表示 `csv_time` 或 `uniform_time_fallback`；方向误差或时间单调性违反契约
时 level 为 `ERROR`。

## 测试资产

新增：

```text
models/ray_bundle_test/model.sdf
worlds/ray_bundle_empty.world
worlds/ray_bundle_single_wall.world
launch/ray_bundle_test.launch
launch/ray_bundle_empty_test.launch
launch/ray_bundle_wall_test.launch
scripts/ray_bundle_validator.py
test/ray_bundle_empty.test
test/ray_bundle_wall.test
models/b_t4_per_ray_observer/model.sdf
worlds/b_t4_per_ray_static_wall.world
launch/b_t4_per_ray_wall_test.launch
scripts/b_t4_per_ray_wall_validator.py
test/b_t4_per_ray_stationary_wall.test
test/b_t4_per_ray_yaw_wall.test
```

空场没有 ground plane、模型或碰撞体；传感器模型本身也没有实体 collision。单墙场只
含一个有限 box collision。验证器只消费公开 RayBundle、PointCloud2 和 diagnostics，
不读取 Gazebo/model truth。

测试源码检查 CSV 方向、完整 pattern 环回、scan-ID 连续、`p ~= r*d`、legacy
字段/索引、uniform duration 和诊断一致性。当前工作区不保留历史运行结果；测试入口见
包内 `CMakeLists.txt` 和 `test/` 目录。

B-T4 使用独立的 Gazebo P3D link truth、严格的逐射线时刻双侧里程计括号和
`x=12 m` 静态墙，检查 timestamped `per_ray_pose` 端点，并同时计算 scan-start
snapshot 反事实。它是传感器几何组件门，不是算法对比；动态碰撞体、非恒定运动、原生
MRS/PX4 飞行栈均在其合同之外。

## 仍未解决

- MRS apt 自带 `--enable-livox` 使用另一个插件库；下一阶段需工程自有 MRS 模型/宏，
  不能假设本 overlay 自动替换它。
- 轻量 fixture 不是 MRS/PX4，没有原生飞控、`world/common_origin` 或避碰栈合同。
- `per_ray_pose` 只支持扫描内静止碰撞场景和恒定 observer twist；移动碰撞体、
  加速度/jerk、扫描内变化 twist 和原生 MRS observer 均未覆盖。默认 `snapshot` 仍是
  兼容模式。
- 没有真实 Mid-360 硬件证据证明零深度球坐标仍包含有效方向；不得声称
  `hw_spherical_exact` 可用。
- `NO_RETURN` 不是整条最大量程路径的硬自由真值，但在 VoFOD 中必须作为有界、带权、
  多次累积的软自由空间证据；其地图用途必须与扫描机会和轨迹负证据分开。
