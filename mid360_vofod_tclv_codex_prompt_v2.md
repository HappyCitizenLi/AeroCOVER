# Codex 提示词：基于实际发射射线的 Mid-360 扫描机会层、VoFOD-Mid360 基线与强化版多目标感知系统

> 使用方式：在当前 ROS/MRS/Mid360 工程根目录打开 Codex，将本文件完整内容作为任务提示词。  
> 目标平台：Ubuntu 20.04、ROS 1 Noetic、Gazebo 11、MRS UAV System。  
> 重要要求：先审计当前工作空间和已有可运行文件，再修改；不得凭空重建已经完成的部分。
>
> **V2 修正：保留原始 VoFOD 使用无返回射线更新自由空间的机制；上一版“无返回射线只用于扫描机会”的表述已删除。**

---

## 1. 你的角色

你是一名负责 ROS 1、Gazebo 11、MRS UAV System、Livox Mid-360 非重复扫描建模、三维占据地图、多目标检测与跟踪的高级机器人软件工程师。

请在当前已可运行的工程基础上，逐阶段实现：

1. observer UAV 搭载 Mid-360 的多无人机仿真场景；
2. **包含有效返回和无返回射线的完整发射射线消息链路**；
3. 适配 Mid-360 的 VoFOD 基线；
4. 轨迹条件化分层射线可见性强化版；
5. 与 VoFOD 的公平对比、消融、真值评估和批量实验。

不要一次提交无法验证的大改动。每个阶段都必须能够单独编译、启动、记录 rosbag、运行测试并给出验收结果。

---

## 2. 当前工程已经完成的工作

当前工程已完成：

- Ubuntu 20.04；
- ROS 1 Noetic；
- Gazebo 11；
- MRS UAV System；
- ROS、Gazebo 和 MRS 环境变量配置；
- MRS 原生 `one_drone` 仿真已成功运行；
- Mid-360 插件工作空间已创建；
- `ctu-mrs/Mid360_simulation_plugin` 已克隆并编译；
- 插件自带独立测试已成功运行；
- 已确认插件发布的 `sensor_msgs/PointCloud2` 包含：
  - `x`；
  - `y`；
  - `z`；
  - `intensity`；
  - `tag`；
  - `line`；
  - `timestamp`。

你必须先定位这些已有工作空间、launch 文件、模型文件、环境加载脚本和当前 Git 状态，不得假定用户目录名，不得硬编码 `/home/<user>/...`。

---

## 3. 本次必须采用的关键技术决策

### 3.1 不再用离线扫描覆盖率作为主扫描机会层

主方法的扫描机会层必须直接使用**当前时间窗口内实际发射的全部射线方向**，包括：

- 有有效返回的射线；
- 没有有效返回的射线；
- 低于最小量程或其他无效状态的射线。

离线扫描覆盖率只能保留为硬件不支持精确无返回方向时的降级方案，不能作为仿真主方法。

---

### 3.2 不要用 `(x,y,z)=(0,0,0)` 表示射线方向

当前 Mid-360 仿真插件内部在每条射线上保留了：

- `time`；
- `azimuth`；
- `zenith`；
- `line`。

插件在无有效返回时将量程设为零，随后输出的端点变成 `(0,0,0)`，从这个端点已经无法恢复原始射线方向。

正确做法是：

- 保留当前 PointCloud2 话题，保证已有测试和 VoFOD 基线兼容；
- 新增一个独立的 `RayBundle` 话题；
- 在量程被置零之前，从 `azimuth`、`zenith` 计算单位方向；
- 无论是否有返回，都发布该射线方向和返回状态；
- 算法不得从零点反推方向。

---

### 3.3 无返回射线同时用于 VoFOD 自由空间更新和扫描机会层，但两种用途必须分开建模

原始 VoFOD 的关键机制之一，就是利用已知发射方向的无返回射线建立自由空间：

- 对有效返回射线，从传感器原点更新到返回点前一个体素；
- 对 `range == 0` 的无返回射线，沿已知方向更新到配置的 `raycast/max_distance`；
- 射线经过体素不是被一次性硬置为自由，而是通过带权分数逐渐向自由状态收敛。

Mid-360 端口必须保留这一机制。否则在大面积开放空间中，只有命中物体的射线才能产生自由空间，VoFOD 所需的“目标点簇被已知自由空间包围”条件会明显退化，且与原始 VoFOD 的比较不公平。

必须将同一条无返回射线的三种用途分开：

1. **静态占据地图更新**

   无返回射线沿实际发射方向，在可靠自由更新距离内提供软自由证据：

   $$
   r_{\mathrm{free},j}
   =
   \min
   \left(
   r_{\mathrm{raycast,max}},
   r_{\mathrm{reliable,no-return}}
   \right).
   $$

   建议为 Mid-360 分别设置：

   ```yaml
   free_update_weight_valid_return: ...
   free_update_weight_no_return: ...
   raycast_max_distance: ...
   ```

   `free_update_weight_no_return` 可以低于有效返回射线路径的权重，但不能默认设为零。

2. **扫描机会层**

   有效返回和无返回的全部实际发射方向都用于统计某条轨迹预测实体是否被扫描：

   $$
   N_i^{\mathrm{emit}}
   =
   \sum_j
   \mathbf 1
   \left[
   \mathcal R_j
   \cap
   \mathcal E_i^{\mathrm{obj}}
   \neq\varnothing
   \right].
   $$

3. **目标漏检或存在性证据**

   穿过目标预测区域但无返回的射线只能作为软的负观测证据，不能由单条射线直接推出目标不存在。它必须结合射线数量、静态遮挡、目标间遮挡、距离、历史回波率和多个时间窗口使用。

因此，本项目不是“不使用无返回射线更新自由空间”，而是：

> **保留 VoFOD 的无返回射线自由空间软更新，同时额外利用同一批实际发射方向构建扫描机会层，并避免把无返回误当成确定的目标不存在证据。**

可选实现 `dynamic_aware_free_weighting`，在已有轨迹预测实体附近降低无返回射线的自由更新权重；该功能必须作为独立消融，不能替代忠实的 VoFOD-Mid360 基线。

---

### 3.4 仿真与实机使用同一个抽象接口

统一定义三种射线来源模式：

```yaml
ray_source_mode:
  sim_exact
  hw_spherical_exact
  calibrated_fallback
```

含义：

- `sim_exact`：从 Gazebo Mid-360 插件内部直接发布全部实际发射方向；
- `hw_spherical_exact`：真实 Mid-360 使用 Livox 球坐标原始包，保留 `depth/theta/phi`；
- `calibrated_fallback`：实机无法获得零深度样本的有效方向时，退化到统计覆盖模型。

所有后续模块只能依赖统一 `RayBundle` 接口，不得在算法内部针对 Gazebo 或实机写两套逻辑。

---

## 4. 已核实的上游代码事实

实现前请再次在本地确认以下代码，不要只相信本提示词。

### 4.1 仿真插件

仓库：

```text
ctu-mrs/Mid360_simulation_plugin
```

重点文件：

```text
livox_laser_simulation/include/livox_laser_simulation/livox_points_plugin.h
livox_laser_simulation/src/livox_points_plugin.cpp
livox_laser_simulation/scan_mode/mid360-real-centr.csv
```

其中 `AviaRotateInfo` 包含：

```cpp
double time;
double azimuth;
double zenith;
uint8_t line;
```

`InitializeRays()` 为每条发射射线构建 `points_pair`。当前发布函数会查询每条射线的 `range`，在无返回时把 `range` 设为零，再计算 `(x,y,z)`，所以输出零点丢失了方向。

### 4.2 真实 Livox 数据链路

官方仓库：

```text
Livox-SDK/Livox-SDK2
Livox-SDK/livox_ros_driver2
```

重点文件：

```text
Livox-SDK2/include/livox_lidar_def.h
livox_ros_driver2/src/comm/pub_handler.cpp
```

官方原始球坐标点结构包含：

```cpp
uint32_t depth;
uint16_t theta;
uint16_t phi;
uint8_t reflectivity;
uint8_t tag;
```

`livox_ros_driver2` 配置支持：

```json
"pcl_data_type": 3
```

即球坐标模式。当前驱动在 `ProcessSphericalPoint()` 中把 `depth/theta/phi` 转成 `x/y/z` 后，只保存笛卡尔点；若 `depth=0`，则方向信息会在此处丢失。

因此真实硬件精确模式应在转换前发布原始射线。

但是，公开接口并未足以证明所有 Mid-360 固件都会在 `depth=0` 时继续提供有效 `theta/phi`。必须实现硬件能力检测，实测确认后才能声明 `hw_spherical_exact` 可用。

---

## 5. 硬性工程约束

1. 不修改 `/opt/ros/noetic`。
2. 不直接修改系统安装的 MRS 包。
3. 上游包需要修改时，在当前工作空间建立 fork、overlay 或明确的补丁分支。
4. 不破坏已经成功运行的 Mid-360 独立测试。
5. 保留原始点云输出，新增功能通过新话题和配置开关启用。
6. 所有路径使用 `$(find package)`、ROS 参数或相对路径，不硬编码用户主目录。
7. 所有算法参数进入 YAML。
8. 感知算法禁止订阅目标 UAV 真值、Gazebo model state 或目标控制状态。
9. 真值只允许进入评估节点。
10. 所有随机场景必须记录随机种子。
11. 所有时间均使用 `/clock` 和仿真时间。
12. 所有点级时间应使用相对 `header.stamp` 的纳秒偏移，避免用 `double` 保存绝对纳秒造成精度损失。
13. 不将无返回射线当作检测点，不送入 PCL 欧氏聚类。
14. 不将一个合并点簇质心同时更新多条轨迹。
15. 每阶段先通过 smoke test，再进入下一阶段。

---

## 6. 建议的工作空间包结构

先审计现有包，避免重复；缺失时再创建：

```text
src/
├── mid360_ray_msgs/
├── mid360_simulation_plugin_fork/      # 或现有插件的受控补丁分支
├── mid360_ray_preprocessor/
├── mid360_multi_uav_sim/
├── vofod_mid360/
├── tclv_visibility/
├── tclv_tracker/
├── tclv_evaluation/
└── livox_ros_driver2_rayfork/          # 实机阶段，仿真阶段可先不编译
```

其中：

- `mid360_ray_msgs`：统一射线消息；
- `mid360_ray_preprocessor`：零点过滤、时间检查、TF/位姿插值、有效点云输出；
- `mid360_multi_uav_sim`：world、observer 挂载、目标轨迹、场景管理；
- `vofod_mid360`：Mid-360 兼容 VoFOD 基线；
- `tclv_visibility`：扫描机会、静态遮挡、目标间遮挡和分层射线解释；
- `tclv_tracker`：IMM/JPDA、可见性状态、轨迹存在概率和遮挡组管理；
- `tclv_evaluation`：真值、指标、rosbag 回放和批量实验；
- `livox_ros_driver2_rayfork`：真实硬件球坐标射线发布适配。

---

## 7. 统一射线消息设计

创建 `mid360_ray_msgs`。

### 7.1 `msg/Ray.msg`

建议定义为固定长度消息：

```text
uint8 NO_RETURN=0
uint8 VALID_RETURN=1
uint8 BELOW_MIN_RANGE=2
uint8 INVALID_RANGE=3
uint8 UNKNOWN_STATUS=255

float32 dir_x
float32 dir_y
float32 dir_z

float32 range
float32 intensity

uint32 offset_time_ns
uint32 pattern_index

uint8 tag
uint8 line
uint8 return_status
```

约定：

- `dir_*` 是 `header.frame_id` 下的单位方向；
- `range` 只有在 `VALID_RETURN` 时可直接作为测距；
- 其他状态允许 `range=0`，算法必须使用 `return_status`；
- `offset_time_ns` 相对 `RayBundle.header.stamp`；
- `pattern_index` 是扫描模式中的全局或模循环索引；
- 所有方向必须满足：

$$
\left|
\|\mathbf d_j\|-1
\right|<10^{-4}.
$$

### 7.2 `msg/RayBundle.msg`

```text
std_msgs/Header header

uint32 scan_id
uint32 pattern_start_index

float32 min_range
float32 max_range

mid360_ray_msgs/Ray[] rays
```

话题约定：

```text
/uav1/mid360/rays_raw
```

保留原始点云：

```text
/uav1/mid360/points_raw
```

预处理后有效点云：

```text
/uav1/mid360/points_valid
```

### 7.3 诊断消息或 diagnostic_updater

至少发布：

```text
ray_count
valid_return_count
no_return_count
invalid_count
direction_norm_error_max
timestamp_monotonic_ratio
bundle_duration
exact_direction_available
source_mode
```

---

## 8. 阶段 A：审计当前工程

第一步只做审计，不修改代码。

输出：

```text
WORKSPACE_AUDIT.md
```

必须包括：

1. 所有 catkin 工作空间；
2. 各工作空间 overlay 顺序；
3. MRS 环境加载脚本；
4. 当前可运行 `one_drone` 命令；
5. 当前 Mid-360 独立测试命令；
6. Mid-360 插件实际仓库路径、分支、commit；
7. 点云话题、frame、频率、字段；
8. 当前 SDF 中：
   - `samples`；
   - `downsample`；
   - `update_rate`；
   - `csv_file_name`；
   - `publish_pointcloud_type`；
9. 当前 TF 树；
10. 当前 Git 是否有未提交修改；
11. 推荐修改方案：fork、分支还是 overlay。

验收：

- 不改变任何已有运行结果；
- 文档中的命令可以复制执行；
- 所有不确定项明确标成 `TO_VERIFY`，不能猜测。

---

## 9. 阶段 B：修改仿真插件，发布全部发射射线

### 9.1 保持原输出不变

原有 PointCloud2 发布逻辑必须继续工作。

不要删除当前 `(0,0,0)` 无返回点，避免破坏已有测试。后续由预处理器过滤。

### 9.2 新增插件参数

在 SDF 中支持：

```xml
<publish_ray_bundle>true</publish_ray_bundle>
<ray_bundle_topic>/uav1/mid360/rays_raw</ray_bundle_topic>
<ray_bundle_frame>uav1/mid360</ray_bundle_frame>
<use_csv_time>true</use_csv_time>
<ray_point_rate>200000</ray_point_rate>
```

参数缺失时保持向后兼容。

### 9.3 射线方向

在量程判定前，对每个 `AviaRotateInfo` 计算：

$$
\mathbf d_j^L=
\begin{bmatrix}
\cos\zeta_j\cos\alpha_j\\
\cos\zeta_j\sin\alpha_j\\
\sin\zeta_j
\end{bmatrix},
$$

其中：

- $\alpha_j$：azimuth；
- $\zeta_j$：相对水平面的 elevation/当前代码中的 zenith 表达。

必须用现有插件坐标约定做单元测试，不能只按变量名猜测。

### 9.4 返回状态

保留 `raw_range`，不要先覆盖：

```cpp
const double raw_range = rayShape->GetRange(ray_index);
```

推荐状态：

```text
VALID_RETURN:
    minDist < raw_range < maxDist

NO_RETURN:
    raw_range >= maxDist - epsilon

BELOW_MIN_RANGE:
    0 < raw_range <= minDist

INVALID_RANGE:
    !isfinite(raw_range) 或 raw_range <= 0
```

对于 `NO_RETURN`：

- `dir_*` 仍然有效；
- `range` 可设为 0；
- `return_status=NO_RETURN`。

### 9.5 射线时间

优先使用 CSV 中的 `AviaRotateInfo.time`，但必须先检查：

- 单位；
- 单调性；
- 扫描周期；
- 索引环回。

若 CSV 时间不可可靠解释，则使用：

$$
\Delta t_j=
\frac{j}{f_{\mathrm{point}}},
$$

并在日志中明确使用 `uniform_time_fallback`。

`RayBundle.header.stamp` 应对应第一条射线的时间，`offset_time_ns` 从零开始。

不要继续使用“绝对 ROS 纳秒写入 double”的方式作为新射线接口。

### 9.6 必须添加的测试

#### B-T1：空世界射线测试

世界中不放任何可命中物体，检查：

- `rays.size() == samples/downsample`；
- 大部分或全部为 `NO_RETURN`；
- 所有方向有限且非零；
- 方向模长接近 1；
- `pattern_index` 连续并正确环回；
- 时间单调；
- 原点 PointCloud2 仍保持原行为。

#### B-T2：单墙测试

放置大平面墙：

- 部分射线 `VALID_RETURN`；
- 部分射线 `NO_RETURN`；
- 有效射线端点与原 PointCloud2 对齐；
- 对同一射线：

$$
\mathbf p_j \approx r_j\mathbf d_j.
$$

#### B-T3：扫描模式重复性测试

连续记录多个窗口：

- 射线方向序列与 CSV 模式一致；
- `currStartIndex` 和 `pattern_index` 无跳变；
- rosbag 回放后结果一致。

#### B-T4：运动 observer 测试

使用静态墙分别进行：

- observer 静止；
- observer 匀速平移；
- observer 匀速偏航。

检查插件实际射线几何是否按逐点时刻生成。

若插件只是同一时刻批量 raycast、但给点赋予不同时间戳，必须在 `KNOWN_LIMITATIONS.md` 记录，并新增配置：

```yaml
ray_time_geometry_mode:
  snapshot
  per_ray_pose
```

不要在没有验证的情况下进行“去畸变”。

---

## 10. 阶段 C：预处理器与射线时间/TF处理

实现 `mid360_ray_preprocessor`。

### 10.1 输入

```text
/uav1/mid360/points_raw
/uav1/mid360/rays_raw
/tf
/tf_static
```

### 10.2 输出

```text
/uav1/mid360/points_valid
/uav1/mid360/rays_checked
/uav1/mid360/ray_diagnostics
```

### 10.3 有效点过滤

PointCloud2 中满足以下条件才进入 `points_valid`：

$$
r_{\min}<\sqrt{x^2+y^2+z^2}<r_{\max},
$$

并且：

- `x/y/z` 有限；
- 不在 observer 自身排除框内；
- 不把 `(0,0,0)` 当作点；
- 保留原 `intensity/tag/line/timestamp` 和原始索引。

### 10.4 每条射线的世界坐标

对于射线 $j$：

$$
t_j=t_{\mathrm{header}}+\Delta t_j,
$$

查询或插值：

$$
{}^W\mathbf T_L(t_j).
$$

得到：

$$
\mathbf o_j^W=
{}^W\mathbf t_L(t_j),
$$

$$
\mathbf d_j^W=
{}^W\mathbf R_L(t_j)\mathbf d_j^L.
$$

不要把整帧统一使用末尾 TF，除非 `ray_time_geometry_mode=snapshot` 且已明确记录。

### 10.5 TF 性能

不能为 20,000 条射线逐条调用阻塞式 `lookupTransform()`。

应：

1. 在 bundle 起止时刻获取位姿；
2. 从 observer odometry/TF 缓存构建连续位姿插值；
3. 对每条射线执行内存中的 SE(3) 插值；
4. 发布 TF 缺失比例和最大时间间隔。

---

## 11. 阶段 D：真实 Mid-360 精确无返回方向能力检测

本阶段代码可以先完成，但没有真实硬件时不得声称测试通过。

### 11.1 fork 官方驱动

基于：

```text
Livox-SDK/livox_ros_driver2
```

创建受控 fork 或 overlay。

设置真实 Mid-360 配置：

```json
"pcl_data_type": 3
```

### 11.2 在球坐标转换前截获原始数据

在 `ProcessSphericalPoint()` 或更上游的数据处理层，读取：

```cpp
depth
theta
phi
reflectivity
tag
line
packet timestamp
point interval
```

直接转换为统一 `RayBundle`：

$$
\mathbf d=
\begin{bmatrix}
\sin\theta\cos\phi\\
\sin\theta\sin\phi\\
\cos\theta
\end{bmatrix}.
$$

注意官方球坐标定义中 $\theta$ 和 $\phi$ 的含义，必须根据 SDK 源码转换公式保持一致。

### 11.3 硬件能力探测程序

实现：

```text
mid360_spherical_capability_probe
```

输出：

```json
{
  "device": "Mid360",
  "firmware": "...",
  "pcl_data_type_3_accepted": true,
  "packet_count": 0,
  "sample_count": 0,
  "zero_depth_count": 0,
  "zero_depth_with_finite_angles": 0,
  "zero_depth_with_nontrivial_angles": 0,
  "angle_temporal_continuity_ratio": 0.0,
  "exact_no_return_direction_supported": false
}
```

判定要求：

- 球坐标模式命令被设备接受；
- 数据包确实为 spherical 类型；
- 存在 `depth=0` 样本；
- `depth=0` 样本的 `theta/phi` 有效且随扫描模式变化；
- 方向序列与邻近有效返回序列时间连续；
- 重启后结果可复现。

### 11.4 能力不足时的行为

若出现以下任一情况：

- Mid-360/固件拒绝 spherical 模式；
- 设备不发送零深度样本；
- 零深度样本的角度全为零或无效；
- 驱动数据包无法区分未发射与无返回；

则：

```text
exact_no_return_direction_supported = false
```

系统自动退回：

```text
calibrated_fallback
```

不得根据 `line + timestamp` 和仿真 CSV 强行重建真实 Mid-360 的精确方向，除非有经过验证的相位同步机制。

---

## 12. 阶段 E：基于实际发射射线的扫描机会层

这是本次修改的核心。

### 12.1 输入

- 全部实际发射射线 `RayBundle`；
- observer 每条射线时刻位姿；
- 所有已有轨迹预测；
- 每条轨迹的物理目标范围；
- 静态占据地图；
- 其他目标轨迹范围。

### 12.2 轨迹范围与不确定性必须分开

每条轨迹维护：

1. 目标物理范围：

$$
\mathcal E_i^{\mathrm{obj}}(t)
$$

2. 数据关联门：

$$
\mathcal E_i^{\mathrm{gate}}(t)
$$

不要直接把很大的跟踪协方差全部当作目标实体尺寸，否则会虚增“被扫描射线数”。

第一版可用球或轴对齐椭球表示无人机实体：

$$
(\mathbf p-\mathbf c_i)^\top
\mathbf A_i^{-1}
(\mathbf p-\mathbf c_i)
\le 1.
$$

### 12.3 射线—目标相交

第 $j$ 条射线：

$$
\mathcal R_j(s)=
\mathbf o_j+s\mathbf d_j,\qquad s\ge0.
$$

把它代入目标椭球，求二次方程。若判别式非负，且存在正的入射距离，则射线穿过目标预测实体。

记录：

$$
\rho_{ij}^{-},
\quad
\rho_{ij}^{+}.
$$

其中 $\rho_{ij}^{-}$ 是进入目标预测实体的最近距离。

### 12.4 实际扫描机会计数

定义：

$$
N_i^{\mathrm{emit}}
=
\sum_j
\mathbf 1
\left[
\mathcal R_j
\cap
\mathcal E_i^{\mathrm{obj}}(t_j)
\neq\varnothing
\right].
$$

注意：

- 有效返回射线计入；
- 无返回射线也计入；
- 低于最小量程的无效射线根据状态单独记录，默认不计入有效扫描支持；
- 只有当前 bundle 中真实存在的射线才计入；
- 不再查询离线覆盖率表来猜测当前是否扫描。

### 12.5 扫描支持分数

输出原始计数和归一化分数：

$$
q_i^{\mathrm{scan}}
=
1-
\exp
\left(
-\frac{N_i^{\mathrm{emit}}}{N_{\mathrm{ref}}}
\right).
$$

其中 $N_{\mathrm{ref}}$ 是配置参数，表示稳定观测所需的有效发射机会数量，不是离线扫描覆盖率。

同时输出：

```text
n_emitted_intersections
n_valid_return_intersections
n_no_return_intersections
n_below_min_intersections
```

初始状态规则可设置为：

```text
N_emit == 0:
    UNSAMPLED

0 < N_emit < N_min_support:
    LOW_SCAN_SUPPORT

N_emit >= N_min_support:
    SCANNED
```

### 12.6 可选的角度覆盖率

为避免多条射线集中于目标投影的一个小区域，可额外计算目标投影角度覆盖率：

$$
q_i^{\mathrm{coverage}}
=
\frac{
\sum_{b\in\mathcal B_i}
w_b
\mathbf 1[n_b>0]
}{
\sum_{b\in\mathcal B_i}w_b
}.
$$

其中：

- $\mathcal B_i$ 是目标预测投影的自适应球面网格；
- 有返回和无返回射线都能标记网格已采样；
- 网格分辨率必须进入 YAML；
- 主日志同时保留 $N_i^{\mathrm{emit}}$ 和 $q_i^{\mathrm{coverage}}$。

不要只输出一个不透明综合分数。

---

## 13. 扫描机会与遮挡必须分开计算

### 13.1 静态遮挡

对于与目标 $i$ 相交的射线 $j$，查询静态地图最近表面距离：

$$
\rho_j^{\mathrm{map}}.
$$

若：

$$
\rho_j^{\mathrm{map}}
<
\rho_{ij}^{-}
-
\epsilon_d,
$$

则该射线方向虽然被发射，但目标在此射线上被静态背景挡住。

### 13.2 目标间遮挡

对于其他目标 $m$，若：

$$
\rho_{mj}^{-}
<
\rho_{ij}^{-}
-
\epsilon_d,
$$

则目标 $m$ 在第 $j$ 条射线上位于目标 $i$ 前方。

### 13.3 未阻挡扫描机会

定义：

$$
N_i^{\mathrm{unblocked}}
=
\sum_j
\mathbf 1
\left[
j\text{ 与 }i\text{ 相交且在到达 }i\text{ 前无已知遮挡}
\right].
$$

进一步记录：

```text
n_map_blocked
n_target_blocked
n_unblocked
n_unblocked_valid_return
n_unblocked_no_return
```

### 13.4 有效观测机会

可定义：

$$
q_i^{\mathrm{obs}}
=
q_i^{\mathrm{scan}}
v_i^{\mathrm{map}}
v_i^{\mathrm{target}}.
$$

但必须发布各分量，不能只发布乘积。

---

## 14. 无返回射线如何影响轨迹

### 14.1 可以提供的证据

若一条无返回射线：

- 实际穿过目标预测实体；
- 没有被静态地图挡住；
- 没有被其他目标挡住；

则它构成：

```text
EXPECTED_VISIBLE_NO_RETURN
```

即目标区域被实际照射，但没有产生有效回波。

### 14.2 不能做的推断

单条无返回射线不能直接推出：

```text
目标不存在
```

因为无返回还受反射率、姿态、入射角和噪声影响。

必须累计多条、多窗口证据，并结合轨迹存在概率。

### 14.3 轨迹相关检测概率

建议保留：

$$
P_{D,i}
=
P_{D,0}
q_i^{\mathrm{scan}}
v_i^{\mathrm{map}}
v_i^{\mathrm{target}}
\rho_i^{\mathrm{return}}.
$$

其中：

- $q_i^{\mathrm{scan}}$ 由实际发射射线直接计算；
- $\rho_i^{\mathrm{return}}$ 才允许根据距离、入射角和历史点数进行经验标定；
- 不得再把离线扫描覆盖概率混入 $q_i^{\mathrm{scan}}$。

---

## 15. 阶段 F：创建分级背景环境

最终实验需要背景，但应逐级增加复杂性。

Codex 创建：

```text
mid360_multi_uav_sim/
├── worlds/
│   ├── E0_open.world
│   ├── E1_structured.world
│   ├── E2_occlusion_arena.world
│   └── E3_cluttered.world
├── models/
│   ├── background_wall/
│   ├── pillar/
│   ├── gate_frame/
│   └── simplified_target_uav/
├── config/scenarios/
│   ├── S00.yaml
│   ├── S01.yaml
│   ├── ...
│   └── S09.yaml
└── launch/
```

要求：

- 所有静态物体包含 `<collision>`；
- 静态物体设置 `<static>true</static>`；
- 碰撞几何优先使用 box/cylinder；
- 视觉网格与碰撞网格分离；
- world 和模型路径不硬编码；
- observer 为 `uav1`；
- target 为 `uav2...uavN`；
- 只有 observer 挂载 Mid-360。

### 15.1 推荐场景

| 场景 | observer | 目标 | 背景 | 目的 |
|---|---|---:|---|---|
| S00 | 静止 | 0 | 开放 | 全射线、无返回方向和零点测试 |
| S01 | 静止 | 1 | 开放 | 距离—回波点数统计 |
| S02 | 静止 | 1 | 后墙 | 有返回/无返回射线对应验证 |
| S03 | 静止 | 2 | 开放 | 纯目标间前后遮挡 |
| S04 | 静止 | 2 | 开放 | 点簇接近、合并、重新分开 |
| S05 | 静止 | 2 | 墙/柱 | 静态遮挡与目标遮挡区分 |
| S06 | 平移 | 2 | 稀疏背景 | observer 运动与射线时间 |
| S07 | 平移+偏航 | 3 | 遮挡场 | 综合可见性 |
| S08 | 运动 | 4～5 | 复杂背景 | 压力测试 |
| S09 | 运动 | 2～3 | 不完整地图 | 在线建图与遮挡混合 |

---

## 16. 阶段 G：VoFOD-Mid360 基线

### 16.1 需要保留的 VoFOD 核心

尽量保留：

- 体素占据分数；
- 背景、未知、可信自由状态；
- 点簇与背景远近分类；
- BFS/flood-fill 悬浮判定；
- 检测消息和可视化形式。

### 16.2 必须替换的 Ouster 假设

原 VoFOD 依赖固定有序点云和二维射线 LUT。Mid-360 端口必须移除：

```text
vertical_rays × horizontal_rays == cloud.size()
```

以及固定 Ouster 行列 mask 依赖。

改为使用：

- `points_valid` 进行聚类；
- `RayBundle` 中的全部实际发射射线进行自由空间更新；
- 有效返回射线更新到返回点前的安全裕量；
- 无返回射线更新到配置的 `raycast/max_distance`；
- 每条射线自己的方向、返回状态、量程和时间；
- 球面角度 mask 或机体自遮挡 mask。

### 16.3 基线中的无返回射线

为了忠实复现 VoFOD：

- B0 VoFOD-Mid360 基线必须同时使用有效返回和无返回射线更新自由空间；
- 有效返回射线的自由更新终点为：

  $$
  r_{\mathrm{free}}
  =
  \min
  \left(
  r_{\mathrm{return}}-\delta_v,
  r_{\mathrm{raycast,max}}
  \right);
  $$

- 无返回射线的自由更新终点为：

  $$
  r_{\mathrm{free}}
  =
  r_{\mathrm{raycast,max}};
  $$

- 两类射线均使用软分数更新，不能一次性把体素硬置为空闲；
- 被机体、自遮挡 mask 或无效状态屏蔽的射线不参与更新；
- B0 可以使用全部射线进行 VoFOD 地图更新，但不使用轨迹条件化扫描支持、遮挡状态或轨迹相关 $P_D$；
- 强化版默认与 B0 共享同一基础自由空间更新，额外利用全部发射方向构建扫描机会层；
- `dynamic_aware_free_weighting` 或延迟地图提交必须单独消融，不能混入 B0。

### 16.4 当前地图更新顺序

先实现忠实基线：

```text
当前点/有效返回与无返回射线更新地图
→ 使用更新后的地图分类点簇
```

强化版增加：

```text
使用上一时刻地图解释当前点
→ 完成轨迹/遮挡/身份判断
→ 最后提交地图更新
```

并设置消融比较。

---

## 17. 阶段 H：强化版完整流程

按以下顺序实现，不得跳步。

### H1：轨迹先预测

每帧处理点云前，预测全部轨迹到每条射线时刻。

### H2：VoFOD 粗聚类和悬浮证据

VoFOD 只提供：

```text
background
floating
unknown
```

粗点簇不再等价于目标实例。

### H3：轨迹条件化动态深度记忆

每条轨迹保存短期目标表面点：

$$
\mathcal H_i=
\{
\delta\mathbf p_m,t_m,w_m
\}.
$$

传播到当前预测中心：

$$
\widetilde{\mathbf p}_{i,m}(t_k)
=
\widehat{\mathbf c}_i(t_k)
+
\delta\mathbf p_m.
$$

### H4：实际发射射线扫描机会

使用阶段 E 的：

```text
N_emit
N_unblocked
q_scan
q_coverage
```

### H5：分层射线解释

每条当前有效返回解释为：

```text
STATIC
TRACK_i
NEW_FLOATING
NOISE
AMBIGUOUS
```

### H6：轨迹条件化二次分割

对同时落入多条轨迹门的粗点簇进行二次分割，输出：

```text
SINGLE_TRACK
SPLITTABLE_MERGE
UNSPLITTABLE_MERGE
NEW_FLOATING
STATIC_OR_UNKNOWN
```

### H7：可见性状态

至少包含：

```text
VISIBLE
PARTIALLY_OCCLUDED
TARGET_OCCLUDED
MAP_OCCLUDED
UNSAMPLED
LOW_SCAN_SUPPORT
EXPECTED_VISIBLE_MISS
AMBIGUOUS
```

其中：

- `UNSAMPLED`：没有发射射线穿过目标预测实体；
- `LOW_SCAN_SUPPORT`：有发射机会但不足；
- `TARGET_OCCLUDED`：被其他轨迹的更近目标层覆盖；
- `MAP_OCCLUDED`：被静态地图表面覆盖；
- `EXPECTED_VISIBLE_MISS`：有足够未阻挡实际发射射线，但没有一致返回。

### H8：可见性感知关联与轨迹管理

实现：

- IMM，至少 `Hover/CV/CA`；
- JPDA 或等价概率关联；
- 轨迹相关 $P_D$；
- 遮挡后方轨迹只预测；
- 禁止前方合并质心更新后方轨迹；
- 遮挡组内暂停普通近邻轨迹合并；
- 重新分开后延迟身份确认；
- 完全遮挡期间协方差自然增大。

---

## 18. 对比组和消融

至少实现：

### B0

```text
VoFOD-Mid360
+
原始风格 lidar_tracker
```

### B1

```text
VoFOD-Mid360
+
与强化版相同的 IMM/JPDA
+
无显式可见性
```

### B2

```text
VoFOD-Mid360
+
IMM/JPDA
+
仅预测角度重叠的普通几何遮挡
```

### P-valid-only

```text
强化版
+
扫描机会只使用有效返回射线方向
```

### P-all-rays

```text
强化版
+
扫描机会使用有效返回和无返回的全部实际发射方向
```

### P-all-rays-no-scan-state

```text
全部射线可用
+
但不区分 UNSAMPLED / LOW_SCAN_SUPPORT / OCCLUDED
```

这三个消融用于直接证明无返回射线方向和扫描机会状态是否真正有价值。

---

## 19. 仿真真值评估

创建独立评估节点，允许读取 Gazebo 真值，但禁止向算法输出。

至少输出：

```text
target_id
position
velocity
true_visibility_fraction
true_occlusion_type
true_occluder_id
true_emitted_ray_intersection_count
true_unblocked_ray_count
```

真值计算要求：

1. 使用仿真实际发射方向；
2. 对目标真实 collision 做射线相交；
3. 区分：
   - 未扫描；
   - 被静态物遮挡；
   - 被其他目标遮挡；
   - 被扫描且可见；
4. 不能把算法预测范围当作真值目标范围；
5. 评估话题位于独立命名空间；
6. 使用 ROS 图检查感知节点未订阅真值。

---

## 20. 指标

### 20.1 扫描机会层

- `N_emit` 误差；
- `N_unblocked` 误差；
- `UNSAMPLED` precision/recall；
- `LOW_SCAN_SUPPORT` precision/recall；
- 有效返回射线版与全部射线版的差异；
- 扫描支持状态切换延迟。

### 20.2 遮挡

- target-occlusion precision/recall/F1；
- map-occlusion precision/recall/F1；
- scan insufficiency 被误判为遮挡的比例；
- 遮挡者 ID 正确率；
- 遮挡起止检测延迟。

### 20.3 检测和跟踪

- precision；
- recall；
- localization RMSE；
- IDF1；
- HOTA；
- ID switches；
- fragmentation；
- track recall；
- rear-track survival rate；
- reacquisition latency；
- 重新分离后 ID 恢复正确率。

### 20.4 地图和实时性

- 动态目标残留占据体素；
- 错误自由空间更新率；
- 每帧总耗时；
- P50/P95/最大耗时；
- ray-target intersection 耗时；
- map raycast 耗时；
- JPDA 耗时；
- CPU、内存和消息带宽。

---

## 21. 性能实现要求

20,000 条射线、10 Hz、最多 5 个目标时，朴素射线—目标相交约 100,000 次/帧，可以接受。

但静态地图 raycast 不能无条件对全部射线执行。应：

1. 先做射线—轨迹实体相交；
2. 只对穿过至少一条轨迹预测实体的射线查询静态地图；
3. 对这些射线再做目标前后排序；
4. 缓存每条射线的最近静态表面距离；
5. 记录每个步骤耗时。

不得为了速度把无返回射线随机下采样而不记录。若需要下采样，必须：

- 固定可复现；
- 保留 `pattern_index`；
- 单独做精度—速度实验；
- 主结果报告下采样比例。

---

## 22. rosbag 与批量实验

每次场景至少记录：

```text
/clock
/uav1/mid360/points_raw
/uav1/mid360/rays_raw
/uav1/mid360/points_valid
/tf
/tf_static
observer state
scenario events
target ground truth
visibility ground truth
algorithm detections
algorithm tracks
algorithm visibility states
profiling
```

推荐：

1. 先运行场景，只记录原始传感器、TF 和真值；
2. 使用同一 rosbag 分别离线回放 B0、B1、B2、P-valid-only 和 P-all-rays；
3. 使用 `/use_sim_time=true`；
4. 输出统一 CSV/JSON；
5. 在线运行仅用于最终实时性测试。

创建：

```text
scripts/run_batch.py
scripts/replay_method.py
scripts/compute_metrics.py
config/experiment_matrix.yaml
```

---

## 23. 每阶段验收门

### Gate 1：工程审计

- `WORKSPACE_AUDIT.md` 完成；
- 原 one_drone 和插件测试仍可运行。

### Gate 2：RayBundle

- 空世界中无返回射线方向非零且单位化；
- 射线数量与 `samples/downsample` 一致；
- 时间单调；
- 原点 PointCloud2 行为未破坏。

### Gate 3：真实方向能力接口

- 仿真 `sim_exact` 通过；
- 实机适配代码能编译；
- 没有硬件时明确标记未验证；
- capability probe 完整。

### Gate 4：多无人机场景

- observer 挂载 Mid-360；
- 两个目标能可重复执行前后交叉；
- 感知节点无真值订阅。

### Gate 5：VoFOD-Mid360

- 单目标和多目标正常场景跑通；
- 检测输出、地图和可视化正常；
- 不再依赖固定 Ouster 组织点云。

### Gate 6：扫描机会层

- `N_emit` 与仿真真值一致；
- 有效返回版和全部射线版在稀疏回波场景中产生可解释差异；
- 无返回射线按照 VoFOD 逻辑参与软自由空间更新，更新距离、权重、mask 和状态过滤均通过测试。

### Gate 7：完整强化版

- 目标交叉期间不使用同一质心更新多轨迹；
- 后方轨迹进入正确遮挡状态；
- 重新分开后 ID 恢复；
- 指标和耗时可批量输出。

---

## 24. 文档交付物

必须生成：

```text
WORKSPACE_AUDIT.md
RAY_BUNDLE_INTERFACE.md
SIM_PLUGIN_MODIFICATIONS.md
HARDWARE_SPHERICAL_CAPABILITY.md
VOFOD_MID360_PORT.md
TCLV_ALGORITHM.md
EXPERIMENT_SCENARIOS.md
EXPERIMENT_MATRIX.md
RUNBOOK.md
KNOWN_LIMITATIONS.md
```

`KNOWN_LIMITATIONS.md` 必须明确：

- 仿真插件是否真实模拟逐点运动畸变；
- 实机当前固件是否验证零深度角度有效；
- 无返回射线在 VoFOD 中提供软自由空间证据，但不是一次观测即可成立的硬自由真值；
- 完全遮挡期间后方目标真实运动不可观测；
- 轨迹预测误差对射线—实体相交的影响；
- 仿真 collision 与真实无人机反射特性的差异。

---

## 25. Codex 的执行顺序

严格按以下顺序执行：

1. 审计工作空间并生成 `WORKSPACE_AUDIT.md`；
2. 创建 `mid360_ray_msgs`；
3. 在受控插件分支中新增 `RayBundle`；
4. 完成空世界、单墙和扫描模式测试；
5. 创建 `mid360_ray_preprocessor`；
6. 创建 observer + 两目标最小场景；
7. 创建扫描机会真值评估；
8. 实现基于实际射线的 `N_emit/N_unblocked`；
9. 创建 VoFOD-Mid360 端口；
10. 跑通 B0；
11. 实现 B1 和 B2；
12. 实现带身份动态深度层；
13. 实现全部射线扫描机会状态；
14. 实现 P-all-rays；
15. 完成消融和批量评估；
16. 最后实现/完善真实 `livox_ros_driver2` 球坐标适配和 capability probe。

每完成一个阶段：

- 编译；
- 运行测试；
- 保存命令；
- 保存关键日志；
- 更新对应文档；
- 明确下一阶段前置条件。

出现编译或运行错误时，先修复当前阶段，不要继续堆叠后续代码。

---

## 26. 首次执行时现在就做的任务

本次 Codex 首轮工作只完成以下内容，不要直接实现完整跟踪算法：

1. 输出 `WORKSPACE_AUDIT.md`；
2. 创建 `mid360_ray_msgs`；
3. 修改仿真插件，新增 `/uav1/mid360/rays_raw`；
4. 保留当前 `/uav1/mid360/points_raw` 行为；
5. 创建空世界和单墙测试；
6. 创建 `ray_bundle_validator.py` 或 C++ 测试节点；
7. 验证无返回射线方向可用；
8. 输出 `SIM_PLUGIN_MODIFICATIONS.md`；
9. 输出下一阶段准确命令和未解决问题。

首轮结束时给出：

```text
Changed files
Build commands
Run commands
Test results
Known failures
Next exact step
```

不要只给方案说明，必须实际创建文件、编译并运行能够在当前机器上执行的测试。
