# 第一阶段实施前源码审计

审计日期：2026-08-20  
审计范围：`/home/uav/lyk` ROS1 Noetic/catkin overlay 及 B0 感知链  
基准状态：本文件记录第一阶段修改前的实际源码；修复结果见
[`IMPLEMENTATION_CHANGELOG.md`](IMPLEMENTATION_CHANGELOG.md)。

## 1. 仓库与包边界

审计开始时 `git status --short --branch` 为 `master` 尚无提交，工作区内文件均为未跟踪；
因此没有可用的历史 diff 基线。首次 `catkin build` 成功，发现 9 个包：

| catkin 包 | 实际路径 | 用途 |
|---|---|---|
| `mid360_ray_msgs` | `src/mid360_ray_msgs` | 原始/checked 射线消息 |
| `livox_laser_simulation` | `src/mid360_simulation_plugin_fork/livox_laser_simulation` | Gazebo 输入 |
| `livox_ros_driver2` | `src/livox_ros_driver2_rayfork` | 可选硬件输入 |
| `mid360_ray_preprocessor` | `src/mid360_ray_preprocessor` | 点/射线过滤、TF |
| `mid360_spherical_capability_probe` | `src/mid360_spherical_capability_probe` | 硬件资格门 |
| `vofod_mid360` | `src/vofod_mid360` | B0 detector/map |
| `lidar_tracker_mid360` | `src/lidar_tracker_mid360` | B0 classic tracker |
| `mid360_multi_uav_sim` | `src/mid360_multi_uav_sim` | 确定性场景夹具 |
| `tclv_evaluation` | `src/tclv_evaluation` | 真值几何 ABI/B0 适配器 |

没有修改 `/opt/ros/noetic` 或 `/home/uav/mrs_mid360_ws`。

## 2. 上游版本

| 本地部分 | 上游 | 固定 commit |
|---|---|---|
| 仿真插件 | `ctu-mrs/Mid360_simulation_plugin` | `dca0420fee13e4dda40f8a609351f64da0a2b7ed` |
| VoFOD | `ctu-mrs/vofod` | `7da9f33a878a586588f6a626b75cfeacac7824f7` |
| tracker | `ctu-mrs/lidar_tracker` | `a92b4db61060b47f1af6dcce122188ec021f2dcd` |
| 硬件驱动 | `Livox-SDK/livox_ros_driver2` | `6b9356cadf77084619ba406e6a0eb41163b08039` |

来源与再分发边界由各包 `UPSTREAM.md` 记录。提示词所写的
`/home/uav/lyk/CURRENT_MRS_VOFOD_MID360_IMPLEMENTATION.md` 实际位于
`docs/CURRENT_MRS_VOFOD_MID360_IMPLEMENTATION.md`。

## 3. 实施前真实 topic graph

```text
/uav1/mid360/points_raw (sensor frame, independent stamp)
/uav1/mid360/rays_raw   (sensor frame, scan_id, first-ray stamp)
        | ApproximateTime, 0.05 s
        v
mid360_ray_preprocessor
  ├─ /uav1/mid360/points_valid (sensor frame, stamp 被改为 ray stamp)
  ├─ /uav1/mid360/rays_checked (world frame)
  └─ /uav1/mid360/ray_diagnostics
        | ExactTime
        v
vofod_mid360
  ├─ /uav1/vofod_mid360/detections
  ├─ /uav1/vofod_mid360/background_points
  └─ /uav1/vofod_mid360/free_voxels

points_valid --一次 cloud-header TF--> lidar_tracker_mid360
detections ---------------------------> lidar_tracker_mid360
background_points --仅评测 launch 开启--> lidar_tracker_mid360
```

算法节点未订阅 ground truth。`free_voxels` 没有接到 tracker 的 occupied-background
KD-tree；评测适配器连接的是 `background_points`。

## 4. 实施前点/射线合同

- `RayBundle.header.stamp` 是首射线时间，`scan_id` 由生产端递增，
  `pattern_start_index` 应等于首射线 `pattern_index`。
- `CheckedRayBundle` 保留 source header、`scan_id`、`pattern_start_index`，每条
  `CheckedRay.original_index` 由 bundle 下标生成。
- `points_valid` 过滤非有限、零点、量程外和 self-box 点，复制原字段并追加
  `original_index`；输出次序跟随输入点云次序。
- 实施前没有 PointCloud2 与 `RayBundle.scan_id` 的显式连接。预处理器使用
  ApproximateTime 选对后把 `points_valid` 改成 ray stamp；下游 ExactTime 只能证明
  派生 stamp 相等，不能证明源 scan 相同。
- 仿真 `PointCloud2<PointXYZ>` 分支用 OpenMP critical append，可能改变源次序；这会使
  “点下标即射线下标”的假设失效。
- detector 检查 `original_index` 和 return status，但没有比较源点 range 与 `Ray.range`，
  也没有检查所有合法返回是否唯一匹配。
- TF 失败时实施前仍可先发布 `points_valid`，不满足整组 fail-closed。

## 5. 实施前 detector/tracker 几何

- `vofod_mid360` 不使用 `points_valid.xyz` 作为世界端点，而以
  `CheckedRay.origin + range * direction` 重建。
- `lidar_tracker_mid360` 使用 sensor-frame `points_valid`，在 cloud header 时刻执行一次
  TF；`per_ray_pose` 下这与 detector 的逐射线几何不同。
- 因此同一 raw return 在 detector 和 tracker 中可能得到不同世界位置；实施前没有
  `/uav1/mid360/points_world`。

## 6. 实施前 B0 map/output 定义

地图是边长 `0.5 m` 的单标量 VoFOD 分数图：初值/unknown `-740`、point `0`、
free-ray 目标 `-1000`；阈值为 sure obstacle `-0.1`、new obstacle `-300`、frontier
`-750`。当前返回先按历史图分 close/far，再写点证据、执行原风格 floating 分类，最后
同步提交有界自由射线。

- `background_points`：分数严格大于 `sure_obstacles=-0.1` 的 voxel center；它表示
  sure occupied background。
- `free_voxels`：分数低于 `init_score-1e-4` 的 voxel center；它是可视化自由更新结果，
  不是 occupied background，也不是目标存在概率。
- B/F/U 是当前 component 分类；地图本身没有 track identity、existence 或 opportunity。

## 7. 实施前真实参数

### detector/map

| 参数 | 值 |
|---|---:|
| map center / size | `(10,0,5)` / `(40,30,16) m` |
| voxel size | `0.5 m` |
| cluster tolerance / min points | `1.25 m` / `2` |
| max OBB / max distance | `3.0 m` / `50 m` |
| background distance / explore distance | `1.5 m` / `3.0 m` |
| background coverage ratio | `0.01` |
| separated-background | `0.1 s`, `0.8 m`, `24 sure points` |
| VALID/NO_RETURN free weight | `0.003 / 0.003` |
| valid endpoint guard | `0.5 m` |
| max free / reliable no-return range | `20 / 20 m` |
| output position sigma / probability | `0.1 / 0.5` |
| startup | `nadir_seed=true`, `84 deg` |

### tracker

| 参数 | 实施前值/语义 |
|---|---|
| state | 9-state CA `[p,v,a]` |
| Q diagonal | `[0.01, 0.8, 0.05]`, `mrs_lib` 以 `dt` 缩放 |
| P0 diagonal | `[0.3, 1.0, 0.05]` |
| fixed R diagonal | `0.1`（代码按 variance 使用，cfg 文案误称 standard deviation） |
| covariance radius | multiplier `1.5`, min/max `0.75/5.0 m` |
| downsample | `0.5 m` |
| local cluster | tolerance `1.0 m`, min points `1`, max OBB `1.0 m` |
| background filter | package launch 默认 false，评测 launch 显式 true |

协方差半径实现为 `multiplier*cbrt(det(P_pos))`；`det(P_pos)` 单位为 `m^6`，该表达式
单位为 `m^2`，却被当成米使用，是明确单位错误。

## 8. 文档与代码不一致/不唯一项

| 项目 | 审计结论 |
|---|---|
| “ExactTime 精确匹配源 scan” | 不成立；源端仅 ApproximateTime，随后改 stamp |
| “source order 保持” | filter 自身保持，但仿真 PointXYZ 分支可在进入 filter 前重排 |
| detector/tracker 同几何 | 不成立；逐射线 endpoint 与整束 TF 并存 |
| TF 失败 fail-closed | 不成立；`points_valid` 可先单独发布 |
| 唯一 B0 | 不成立；tracker 单包 launch 默认关闭背景过滤，评测 launch 开启 |
| `nadir_seed` 等价原 height rangefinder | 不成立；它是一束向下射线播种整个 component 的代理 |
| tracker Q/P0/R 单位 | YAML/代码按 covariance 数值使用，dynamic-reconfigure 文案误称标准差 |
| tracker covariance radius | 代码存在 variance-to-length 单位错误 |
| rolling scan | `per_ray_pose` 只修 observer，collision scene 在 bundle 内仍是受限快照 |

这些问题均属于 Phase 0/1 的公共合同和 B0 修复；审计未发现 scene name、target ID、
truth topic 或目标初始位置进入算法判据。
