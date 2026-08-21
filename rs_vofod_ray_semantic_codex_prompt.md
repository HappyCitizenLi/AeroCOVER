# Codex 提示词：射线语义驱动的 VoFOD 重构与可见性条件化多目标跟踪

> 工作名称：Ray-Semantic VoFOD，简称 **RS-VoFOD**。  
> 该名称仅用于工程实现，后续可根据论文叙事重新命名。  
> 目标平台：Ubuntu 20.04、ROS 1 Noetic、Gazebo 11、MRS UAV System、CTU-MRS Mid-360 仿真插件。  
> 使用方式：在当前工程根目录打开 Codex，将本文件完整内容作为任务提示词。  
> 重要原则：先审计现有工程，再逐阶段实现；不得一次性重写全部代码；每个阶段必须可编译、可启动、可记录、可测试。

---

# 1. 你的角色

你是一名高级机器人感知与 ROS 工程师，熟悉：

- ROS 1 Noetic；
- Gazebo 11；
- MRS UAV System；
- Livox Mid-360 非重复扫描；
- CTU-MRS Mid360 simulation plugin；
- VoFOD 三维占据地图和 BFS/flood-fill 浮空目标判定；
- LiDAR 多目标检测与跟踪；
- 卡尔曼滤波、数据关联、遮挡状态管理；
- C++17、PCL、Eigen、tf2、pluginlib、ROS message/filter；
- rosbag、rostest、gtest 和批量实验评估。

你的任务是在当前工程基础上实现一个**以实际发射射线为基本观测单位**的飞行目标检测与跟踪框架。

该框架不再采用：

```text
当前点云
→ VoFOD输出完整点簇和质心
→ 跟踪器
→ 射线模块只解释目标为什么漏检
```

而采用：

```text
全部实际发射射线
→ 有返回端点的背景/浮空候选/未知语义
→ 无返回射线的扫描与自由空间证据
→ 在轨迹预测范围内聚合射线证据
→ 同时形成目标量测和可见性状态
→ 条件化轨迹更新
```

---

# 2. 当前工程已经完成的工作

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
- 已确认 `sensor_msgs/PointCloud2` 中包含：
  - `x`；
  - `y`；
  - `z`；
  - `intensity`；
  - `tag`；
  - `line`；
  - `timestamp`。

已有讨论还确认：

- Mid-360 仿真插件内部保留每条发射射线的方向；
- 当前无返回射线在 PointCloud2 中被写成 `(0,0,0)`；
- 需要在插件内部新增完整射线消息，不能从零点恢复方向；
- VoFOD 原算法确实使用无返回射线更新自由空间；
- 无返回射线在本算法中同时承担：
  1. VoFOD 自由空间软更新；
  2. 当前扫描机会判断；
  3. 目标没有回波的软负证据；
- 无返回射线不能被当作“目标一定不存在”的硬证据。

---

# 3. 核心科学问题

本项目解决的是：

> 在搭载 Mid-360 的移动观察无人机感知多个非合作无人机目标时，如何利用有返回和无返回的实际发射射线，在稀疏、非重复扫描、目标接近、目标交叉和静态背景遮挡条件下，同时完成背景分离、飞行目标量测生成、目标可见性判断和轨迹身份维护？

原始 VoFOD 的基本处理单位是**当前点簇**。它先聚类，再利用历史占据地图判断整个点簇是：

```text
background
flying object
unknown
```

这种方式存在以下局限：

1. 一个粗点簇可能包含两个接近目标；
2. 后方目标完全遮挡时没有点簇；
3. 稀疏 Mid-360 回波可能只有一个或两个点，难以稳定形成点簇；
4. 点簇质心压缩了射线方向、返回距离和前后层次信息；
5. 点簇产生后再做射线分析，存在先压缩信息、再尝试恢复信息的问题。

本项目改为：

> 将每条发射射线作为统一观测单位；对有返回射线的端点生成静态空间语义，对无返回射线保留扫描和自由空间语义；随后在每条已有轨迹的预测物理范围中聚合射线证据，直接形成目标状态和轨迹量测。

---

# 4. 核心创新点

## 4.1 射线级空间语义，而不是点簇级最终检测

每条有返回射线的端点获得：

```text
BACKGROUND_CONSISTENT
FLOATING_CANDIDATE
UNKNOWN_RETURN
```

每条无返回射线获得：

```text
NO_RETURN
```

但必须注意：

> 不能让每个点完全独立执行 BFS 并直接声称它是飞行目标。

正确方式是：

- 射线是最终输出和后续推理的基本单位；
- 局部返回端点连通成分为每条射线提供空间上下文；
- 每个连通成分只执行一次 VoFOD 式背景连接和自由包围判断；
- 分量内所有返回射线继承该空间语义。

因此，这不是取消所有空间聚合，而是取消：

```text
一个点簇
=
一个目标实例
=
一个质心量测
```

---

## 4.2 实际发射射线驱动的轨迹级观测状态

使用当前窗口中的全部实际发射射线，判断每条轨迹预测物理范围是否被扫描，并区分：

```text
VISIBLE
UNSAMPLED
MAP_OCCLUDED
TARGET_OCCLUDED
VISIBLE_MISS
AMBIGUOUS
```

其中：

- `UNSAMPLED`：没有足够实际射线穿过目标预测实体；
- `MAP_OCCLUDED`：射线在到达目标前被静态地图表面阻挡；
- `TARGET_OCCLUDED`：射线在到达目标前先穿过另一目标预测实体；
- `VISIBLE`：存在足够浮空候选返回，并与轨迹预测一致；
- `VISIBLE_MISS`：目标被充分、未遮挡地扫描，但没有一致返回；
- `AMBIGUOUS`：证据不足以可靠进入其他状态。

---

## 4.3 同一批射线同时形成目标量测和可见性状态

对于已有轨迹，不再必须先得到完整 VoFOD 点簇。

在轨迹预测范围内聚合 `FLOATING_CANDIDATE` 射线端点，直接形成：

- 目标位置量测；
- 量测协方差；
- 当前支持点数量；
- 当前扫描支持；
- 当前遮挡状态。

这使检测与跟踪形成统一闭环。

---

## 4.4 已有目标和新目标采用不同聚合机制

已有目标有预测范围，因此使用：

```text
轨迹条件化的射线聚合
```

新目标没有预测范围，因此仍需要：

```text
浮空候选射线的轻量空间/短时聚合
```

新目标发现可以保留体素连通成分或短时 DBSCAN，但不能让新目标初始化机制反过来限制已有轨迹的量测更新。

---

# 5. 明确的非目标

第一版不要实现以下复杂模块：

- 深度神经网络；
- 点级可训练分类器；
- 完整 MHT；
- 全局 K-best 多假设；
- 目标姿态估计；
- 历史目标表面点的姿态传播；
- 复杂 RFS/PMBM；
- 在线学习扫描回波概率；
- 同时解决未知目标类别识别；
- 大规模城市地图；
- 五个以上目标的压力测试。

第一版目标是验证：

1. 射线语义能否替代点簇质心作为已有轨迹量测来源；
2. 实际射线能否正确区分未扫描、静态遮挡和目标间遮挡；
3. 在稀疏回波和目标交叉情况下，是否优于 VoFOD 点簇级检测和跟踪。

---

# 6. 推荐包结构

先审计现有工作空间。缺失时创建：

```text
src/
├── mid360_ray_msgs/
├── mid360_simulation_plugin_fork/
├── mid360_ray_preprocessor/
├── vofod_mid360_baseline/
├── rs_vofod_core/
├── rs_vofod_tracker/
├── mid360_multi_uav_sim/
└── rs_vofod_evaluation/
```

职责：

```text
mid360_ray_msgs
    统一射线、射线语义、轨迹可见性消息

mid360_simulation_plugin_fork
    在保留原 PointCloud2 的同时发布完整实际发射射线

mid360_ray_preprocessor
    过滤零点、检查时间、转换射线到世界坐标、输出有效点云

vofod_mid360_baseline
    保留 VoFOD 点簇级检测逻辑，适配 Mid-360 完整射线

rs_vofod_core
    返回端点局部成分、VoFOD式空间语义、射线级输出、地图更新

rs_vofod_tracker
    轨迹预测、射线—轨迹相交、可见性状态、量测形成和条件化更新

mid360_multi_uav_sim
    observer、目标 UAV、背景、轨迹脚本、场景配置

rs_vofod_evaluation
    真值、指标、rosbag回放、批量实验
```

---

# 7. 坐标系和时间约定

统一坐标系：

```text
W
    世界/地图坐标系

L(t)
    射线发射时刻 t 的 Mid-360 坐标系

B(t)
    observer UAV 机体坐标系

T_i
    第 i 条目标轨迹
```

对于第 $j$ 条射线：

$$
t_j=t_k+\Delta t_j,
$$

其中：

- $t_k$：当前 `RayBundle.header.stamp`；
- $\Delta t_j$：第 $j$ 条射线相对于 bundle 起始时刻的时间偏移；
- $t_j$：该射线的真实或仿真发射时刻。

所有射线几何计算必须在同一坐标系中进行，推荐转换到世界坐标系 $W$。

不能对 20,000 条射线逐条调用阻塞式 `tf2 lookupTransform()`。

应实现：

1. 获取 bundle 起止时间附近的 observer 位姿；
2. 建立内存中的位姿缓存；
3. 对每条射线进行 SE(3) 插值；
4. 输出 TF 缺失比例和最大插值间隔。

---

# 8. 统一射线数据结构

## 8.1 `Ray.msg`

创建：

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

含义：

- `dir_x, dir_y, dir_z`：射线在 `RayBundle.header.frame_id` 中的单位方向；
- `range`：有效返回距离；
- `offset_time_ns`：相对 bundle 起始时刻的纳秒偏移；
- `pattern_index`：当前射线在 Mid-360 扫描模式中的索引；
- `return_status`：有效返回、无返回或无效状态；
- `line`：激光通道；
- `tag`：仿真中可能全部为零，不能依赖其判断有效性。

## 8.2 `RayBundle.msg`

```text
std_msgs/Header header

uint32 scan_id
uint32 pattern_start_index

float32 min_range
float32 max_range

mid360_ray_msgs/Ray[] rays
```

## 8.3 `RaySemantic.msg`

```text
uint8 NO_RETURN=0
uint8 BACKGROUND_CONSISTENT=1
uint8 FLOATING_CANDIDATE=2
uint8 UNKNOWN_RETURN=3
uint8 INVALID=255

uint32 ray_index
uint32 component_id

uint8 semantic_label
float32 semantic_confidence

geometry_msgs/Point endpoint_sensor
geometry_msgs/Point endpoint_world

float32 range
float32 map_surface_distance
```

## 8.4 `RaySemanticBundle.msg`

```text
std_msgs/Header header
mid360_ray_msgs/RaySemantic[] semantics
```

## 8.5 `TrackVisibility.msg`

```text
uint8 VISIBLE=0
uint8 UNSAMPLED=1
uint8 MAP_OCCLUDED=2
uint8 TARGET_OCCLUDED=3
uint8 VISIBLE_MISS=4
uint8 AMBIGUOUS=5

int32 track_id
uint8 state
int32 occluder_track_id

uint32 n_emit
uint32 n_return
uint32 n_floating
uint32 n_background
uint32 n_unknown
uint32 n_no_return

uint32 n_map_blocked
uint32 n_target_blocked
uint32 n_unblocked

float32 q_scan
float32 q_map_block
float32 q_target_block
float32 q_floating_support
float32 confidence
```

---

# 9. 核心符号总表

| 符号 | 含义 |
|---|---|
| $k$ | 当前处理窗口索引 |
| $j$ | 当前窗口中的射线索引 |
| $i$ | 目标轨迹索引 |
| $q$ | 当前返回端点连通成分索引 |
| $t_j$ | 第 $j$ 条射线发射时刻 |
| $\mathbf o_j$ | 第 $j$ 条射线在世界坐标中的原点 |
| $\mathbf d_j$ | 第 $j$ 条射线在世界坐标中的单位方向 |
| $r_j$ | 第 $j$ 条射线的返回距离 |
| $\mathcal R_j$ | 第 $j$ 条射线 |
| $\mathbf p_j$ | 有返回射线的端点 |
| $\ell_j$ | 第 $j$ 条射线的语义标签 |
| $\mathcal M_{k-1}$ | 当前窗口处理前的历史静态占据地图 |
| $C_q$ | 第 $q$ 个局部返回端点连通成分 |
| $T_i$ | 第 $i$ 条目标轨迹 |
| $\hat{\mathbf x}_{i,k\|k-1}$ | 轨迹 $i$ 当前窗口的预测状态 |
| $\mathbf P_{i,k\|k-1}$ | 轨迹预测协方差 |
| $\mathcal E_i^{\mathrm{obj}}$ | 目标 $i$ 的预测物理实体范围 |
| $\mathcal E_i^{\mathrm{gate}}$ | 轨迹 $i$ 的数据关联门 |
| $\rho_{ij}^{-}$ | 射线 $j$ 进入目标 $i$ 预测实体的最近距离 |
| $\rho_{ij}^{+}$ | 射线 $j$ 离开目标 $i$ 预测实体的距离 |
| $\mathcal J_i^{\mathrm{emit}}$ | 穿过目标 $i$ 预测实体的射线集合 |
| $\mathcal J_i^{F}$ | 为目标 $i$ 提供浮空候选返回的射线集合 |
| $N_i^{\mathrm{emit}}$ | 穿过目标 $i$ 预测实体的实际发射射线数 |
| $N_i^F$ | 与目标 $i$ 一致的浮空候选返回数 |
| $N_i^B$ | 与目标范围相交但端点属于背景的射线数 |
| $N_i^U$ | 与目标范围相交但端点语义未知的射线数 |
| $N_i^{NR}$ | 穿过目标范围但没有返回的射线数 |
| $N_i^{\mathrm{map}}$ | 被静态地图表面阻挡的射线数 |
| $N_i^{\mathrm{target}}$ | 被其他目标预测实体阻挡的射线数 |
| $N_i^{\mathrm{unblocked}}$ | 到达目标前没有已知遮挡的射线数 |
| $q_i^{\mathrm{scan}}$ | 轨迹 $i$ 的扫描支持 |
| $q_i^{F}$ | 轨迹 $i$ 的浮空返回支持 |
| $\mathbf z_i$ | 由射线端点生成的轨迹量测 |
| $\mathbf R_i$ | 轨迹量测协方差 |
| $s_i$ | 轨迹当前可见性状态 |
| $e_i$ | 轨迹存在分数 |

---

# 10. 第一步：完整射线发布

修改 Mid-360 仿真插件，但必须保留原 PointCloud2 输出。

新增：

```text
/uav1/mid360/rays_raw
```

插件在把无返回距离改成零之前，必须保存：

- 射线方向；
- 原始 range；
- return status；
- line；
- tag；
- pattern index；
- 当前射线时间。

射线方向必须由插件当前使用的扫描角度计算，并通过单墙测试验证：

$$
\mathbf p_j
\approx
r_j\mathbf d_j.
$$

不要只根据变量名猜测 `azimuth/zenith` 定义。

必须完成：

```text
空世界测试
单墙测试
连续扫描模式索引测试
时间单调性测试
方向模长测试
```

---

# 11. 第二步：射线预处理

输入：

```text
/uav1/mid360/rays_raw
/uav1/mid360/points_raw
/tf
/tf_static
```

输出：

```text
/uav1/mid360/rays_world
/uav1/mid360/points_valid
/uav1/mid360/ray_diagnostics
```

## 11.1 有效返回点

对于有效返回：

$$
\mathbf p_j^L=r_j\mathbf d_j^L.
$$

过滤：

- 非有限坐标；
- $r_j\le r_{\min}$；
- $r_j\ge r_{\max}$；
- observer 自身碰撞盒；
- `(0,0,0)`；
- 无效 return status。

## 11.2 世界坐标射线

第 $j$ 条射线：

$$
\mathcal R_j(s)
=
\mathbf o_j+s\mathbf d_j,
\qquad s\ge0.
$$

其中：

$$
\mathbf o_j
=
{}^W\mathbf t_L(t_j),
$$

$$
\mathbf d_j
=
{}^W\mathbf R_L(t_j)\mathbf d_j^L.
$$

有返回端点：

$$
\mathbf p_j
=
\mathbf o_j+r_j\mathbf d_j.
$$

---

# 12. 第三步：轨迹预测

轨迹预测必须在当前射线证据聚合前完成。

第一版使用统一维度的 CV 或 Hover/CV IMM。

建议先实现 CV：

$$
\mathbf x_i=
\begin{bmatrix}
x_i&y_i&z_i&v_{x,i}&v_{y,i}&v_{z,i}
\end{bmatrix}^{\top}.
$$

预测：

$$
\hat{\mathbf x}_{i,k|k-1}
=
\mathbf F
\hat{\mathbf x}_{i,k-1|k-1},
$$

$$
\mathbf P_{i,k|k-1}
=
\mathbf F
\mathbf P_{i,k-1|k-1}
\mathbf F^\top
+
\mathbf Q.
$$

其中：

- $\mathbf F$：恒速度状态转移矩阵；
- $\mathbf Q$：过程噪声；
- $\hat{\mathbf x}_{i,k|k-1}$：当前窗口开始前的预测状态；
- $\mathbf P_{i,k|k-1}$：预测不确定性。

---

# 13. 目标物理范围与关联门必须分开

这是硬性要求。

## 13.1 目标物理范围

$$
\mathcal E_i^{\mathrm{obj}}
=
\left\{
\mathbf p:
(\mathbf p-\hat{\mathbf c}_i)^\top
\mathbf A_i^{-1}
(\mathbf p-\hat{\mathbf c}_i)
\le1
\right\}.
$$

其中：

- $\hat{\mathbf c}_i$：轨迹预测中心；
- $\mathbf A_i$：目标实体尺寸矩阵；
- $\mathcal E_i^{\mathrm{obj}}$：目标实体大致占据的真实空间范围。

第一版可用：

```text
球体
或
轴对齐椭球
```

不要直接用协方差构造目标实体范围。

## 13.2 数据关联门

$$
\mathcal E_i^{\mathrm{gate}}
=
\left\{
\mathbf p:
(\mathbf p-\hat{\mathbf c}_i)^\top
\mathbf S_i^{-1}
(\mathbf p-\hat{\mathbf c}_i)
\le\gamma_g
\right\},
$$

其中：

$$
\mathbf S_i
=
\mathbf P_{i,\mathrm{pos}}
+
\mathbf R_{\mathrm{base}}.
$$

含义：

- $\mathbf P_{i,\mathrm{pos}}$：轨迹位置预测协方差；
- $\mathbf R_{\mathrm{base}}$：基础量测不确定性；
- $\gamma_g$：关联门阈值；
- $\mathcal E_i^{\mathrm{gate}}$：允许量测匹配到该轨迹的统计区域。

目标物理范围用于：

```text
射线是否真实扫描目标
目标间几何遮挡
```

关联门用于：

```text
返回端点是否可能属于轨迹
```

不能混用。

---

# 14. 第四步：返回端点局部连通成分

只对有效返回端点构建局部图：

$$
G_k^{\mathrm{return}}
=
(V_k,E_k).
$$

其中：

- 每个顶点对应一个有返回射线端点；
- 两个端点满足局部邻近条件时连边。

建议使用体素邻接而不是全局 PCL 欧氏聚类。

## 14.1 体素化

将端点映射到体素：

$$
v_j
=
\operatorname{voxelIndex}
(\mathbf p_j).
$$

## 14.2 邻接

两个端点体素相邻时连接：

```text
6邻域
或
18邻域
或
26邻域
```

第一版建议 26 邻域。

可以增加距离约束：

$$
\|\mathbf p_a-\mathbf p_b\|
<
\tau_c.
$$

## 14.3 连通成分

得到：

$$
\mathcal C_k=
\{C_1,C_2,\dots,C_Q\}.
$$

注意：

> 这些成分不是最终目标实例，只是为射线端点提供空间上下文。

一个成分可以：

- 是静态背景的一部分；
- 是一个目标；
- 同时包含两个接近目标；
- 是背景边缘和目标混合；
- 只有一个点。

必须允许单点成分。

---

# 15. 第五步：VoFOD式空间语义分类

对每个返回端点成分 $C_q$，调用改造后的 VoFOD 分类器：

$$
g(C_q,\mathcal M_{k-1})
\rightarrow
\left(
L_q,
c_q
\right),
$$

其中：

- $L_q$：成分语义；
- $c_q$：语义置信度；
- $\mathcal M_{k-1}$：处理当前窗口之前的历史占据地图。

成分语义：

```text
BACKGROUND_CONSISTENT
FLOATING_CANDIDATE
UNKNOWN_RETURN
```

## 15.1 必须复用原 VoFOD 的核心逻辑

优先复用当前 VoFOD 源码中的：

- confident occupied；
- tentative occupied；
- uncertain；
- confident free；
- close-to-background 判断；
- far-cluster 判断；
- BFS/flood-fill；
- 是否通过未知体素连接背景；
- 是否到达搜索边界；
- 是否被已知自由空间包围。

不要重新凭经验写一个完全不同的分类器。

## 15.2 输出粒度变化

原 VoFOD 输出：

```text
一个飞行点簇
一个质心
```

RS-VoFOD 输出：

```text
成分内每条返回射线的空间语义
```

若第 $j$ 条射线端点属于成分 $C_q$，则：

$$
\ell_j=
\begin{cases}
B, & L_q=\mathrm{BACKGROUND},\\
F, & L_q=\mathrm{FLOATING\_CANDIDATE},\\
U, & L_q=\mathrm{UNKNOWN}.
\end{cases}
$$

无返回射线：

$$
\ell_j=NR.
$$

最终：

$$
\ell_j
\in
\{B,F,U,NR\}.
$$

含义：

- $B$：返回端点与静态背景一致；
- $F$：返回端点属于浮空候选成分；
- $U$：返回端点空间关系无法确定；
- $NR$：实际发射但无有效返回。

---

# 16. 防止轨迹自我确认

这是硬性设计原则。

必须保证：

$$
\text{背景/浮空/未知语义}
$$

只由：

- 当前返回端点；
- 历史静态占据地图；
- VoFOD 空间连接关系；

决定。

不能根据轨迹预测把一个背景或未知点直接改成浮空候选。

轨迹预测只能用于：

- 将已经是 `FLOATING_CANDIDATE` 的射线分配给某条轨迹；
- 计算目标是否被扫描；
- 判断遮挡；
- 形成轨迹量测。

否则错误轨迹会把附近噪声不断吸收为自身量测，形成自我确认。

---

# 17. 第六步：射线—目标预测实体相交

第 $j$ 条射线：

$$
\mathcal R_j(s)
=
\mathbf o_j+s\mathbf d_j.
$$

目标 $i$ 的预测实体：

$$
(\mathbf p-\hat{\mathbf c}_i)^\top
\mathbf A_i^{-1}
(\mathbf p-\hat{\mathbf c}_i)
\le1.
$$

令：

$$
\mathbf q_{ij}
=
\mathbf o_j-\hat{\mathbf c}_i.
$$

代入射线得到：

$$
a_{ij}s^2+b_{ij}s+c_{ij}=0,
$$

其中：

$$
a_{ij}
=
\mathbf d_j^\top
\mathbf A_i^{-1}
\mathbf d_j,
$$

$$
b_{ij}
=
2\mathbf q_{ij}^\top
\mathbf A_i^{-1}
\mathbf d_j,
$$

$$
c_{ij}
=
\mathbf q_{ij}^\top
\mathbf A_i^{-1}
\mathbf q_{ij}
-1.
$$

判别式：

$$
\Delta_{ij}
=
b_{ij}^2-4a_{ij}c_{ij}.
$$

若：

$$
\Delta_{ij}<0,
$$

则射线与目标实体不相交。

若：

$$
\Delta_{ij}\ge0,
$$

则交点参数：

$$
\rho_{ij}^{-}
=
\frac{-b_{ij}-\sqrt{\Delta_{ij}}}
{2a_{ij}},
$$

$$
\rho_{ij}^{+}
=
\frac{-b_{ij}+\sqrt{\Delta_{ij}}}
{2a_{ij}}.
$$

要求：

```text
rho_plus > 0
rho_minus 或 rho_plus 在传感器有效量程内
```

最近进入距离取：

$$
\rho_{ij}^{\mathrm{entry}}
=
\max(0,\rho_{ij}^{-}).
$$

---

# 18. 第七步：轨迹射线集合

轨迹 $i$ 的实际扫描射线集合：

$$
\mathcal J_i^{\mathrm{emit}}
=
\left\{
j:
\mathcal R_j
\cap
\mathcal E_i^{\mathrm{obj}}
\neq\varnothing
\right\}.
$$

实际扫描射线数：

$$
N_i^{\mathrm{emit}}
=
|\mathcal J_i^{\mathrm{emit}}|.
$$

这一步包括：

- 有返回射线；
- 无返回射线。

扫描支持：

$$
q_i^{\mathrm{scan}}
=
1-
\exp
\left(
-\frac{N_i^{\mathrm{emit}}}
{N_{\mathrm{ref}}}
\right).
$$

其中：

- $N_{\mathrm{ref}}$：经验配置的稳定扫描机会参考数量；
- $q_i^{\mathrm{scan}}\in[0,1]$；
- 它不表示目标一定能产生回波，只表示目标区域被实际射线覆盖的程度。

必须同时发布原始计数，不能只发布归一化分数。

---

# 19. 第八步：返回射线与轨迹身份匹配

对于一条有返回、语义为 $F$ 的射线，判断其端点是否可能属于轨迹 $i$。

## 19.1 物理深度一致性

若返回距离满足：

$$
r_j
\in
[
\rho_{ij}^{-}-\delta_r,
\rho_{ij}^{+}+\delta_r
],
$$

则该端点与目标实体深度一致。

其中：

- $\delta_r$：深度容差；
- 它用于吸收轨迹预测误差和目标范围误差。

## 19.2 关联门一致性

还要求：

$$
\mathbf p_j
\in
\mathcal E_i^{\mathrm{gate}}.
$$

候选集合：

$$
\mathcal I_j
=
\left\{
i:
\ell_j=F,
\mathbf p_j\in\mathcal E_i^{\mathrm{gate}},
r_j\text{ 与目标层一致}
\right\}.
$$

## 19.3 多轨迹共享射线

若：

$$
|\mathcal I_j|=1,
$$

直接分配给唯一候选轨迹。

若：

$$
|\mathcal I_j|>1,
$$

计算标准化代价：

$$
D_{ij}
=
\frac{
\operatorname{dist}
\left(
r_j,
[\rho_{ij}^{-},\rho_{ij}^{+}]
\right)^2
}{
\sigma_{r,i}^2
}
+
\lambda_p
(\mathbf p_j-\hat{\mathbf c}_i)^\top
\mathbf S_i^{-1}
(\mathbf p_j-\hat{\mathbf c}_i).
$$

其中：

- 第一项：返回深度与目标物理层的差异；
- 第二项：端点与轨迹预测中心的马氏距离；
- $\lambda_p$：空间距离权重。

如果最优和次优代价差：

$$
D_{2nd}-D_{best}
<
\tau_{\mathrm{amb}},
$$

则该射线标记为身份歧义，不用于任何轨迹位置量测。

不能把同一条射线复制给多条轨迹。

---

# 20. 第九步：静态地图遮挡

对于穿过目标 $i$ 的射线 $j$，从历史静态占据地图查询该方向最近可信静态表面距离：

$$
\rho_j^{\mathrm{map}}.
$$

若：

$$
\rho_j^{\mathrm{map}}
<
\rho_{ij}^{\mathrm{entry}}
-
\epsilon_{\mathrm{occ}},
$$

则该射线在到达目标前被静态背景挡住。

定义：

$$
N_i^{\mathrm{map}}
=
\sum_{j\in\mathcal J_i^{\mathrm{emit}}}
\mathbf 1
\left[
\rho_j^{\mathrm{map}}
<
\rho_{ij}^{\mathrm{entry}}
-
\epsilon_{\mathrm{occ}}
\right].
$$

其中：

- $\epsilon_{\mathrm{occ}}$：遮挡深度裕量；
- 用于降低地图离散误差和位姿误差造成的误判。

静态遮挡比例：

$$
q_i^{\mathrm{map}}
=
\frac{
N_i^{\mathrm{map}}
}{
\max(1,N_i^{\mathrm{emit}})
}.
$$

---

# 21. 第十步：目标间遮挡

若射线 $j$ 同时穿过目标 $m$ 和目标 $i$，且：

$$
\rho_{mj}^{\mathrm{entry}}
<
\rho_{ij}^{\mathrm{entry}}
-
\epsilon_{\mathrm{occ}},
$$

则目标 $m$ 在该射线方向上位于目标 $i$ 前方。

目标间遮挡计数：

$$
N_i^{\mathrm{target}}
=
\sum_{j\in\mathcal J_i^{\mathrm{emit}}}
\mathbf 1
\left[
\exists m\neq i:
\rho_{mj}^{\mathrm{entry}}
<
\rho_{ij}^{\mathrm{entry}}
-
\epsilon_{\mathrm{occ}}
\right].
$$

目标遮挡比例：

$$
q_i^{\mathrm{target}}
=
\frac{
N_i^{\mathrm{target}}
}{
\max(1,N_i^{\mathrm{emit}})
}.
$$

同时记录主要遮挡者：

$$
m_i^\star
=
\arg\max_m
N_{m\rightarrow i}.
$$

第一版不能只依赖预测几何就判定完全遮挡。

建议增加当前返回支持：

- 前方目标 $m$ 当前处于 `VISIBLE`；
- 或存在属于 $m$ 的浮空候选返回；
- 才提高 `TARGET_OCCLUDED` 置信度。

否则目标预测误差可能制造虚假遮挡。

---

# 22. 第十一步：轨迹证据统计

对每条轨迹统计：

$$
N_i^F
=
\left|
\mathcal J_i^F
\right|,
$$

其中：

$$
\mathcal J_i^F
=
\left\{
j:
\ell_j=F,
j\text{ 被唯一或可靠地分配给 }i
\right\}.
$$

背景返回数：

$$
N_i^B
=
\sum_{j\in\mathcal J_i^{\mathrm{emit}}}
\mathbf 1[\ell_j=B].
$$

未知返回数：

$$
N_i^U
=
\sum_{j\in\mathcal J_i^{\mathrm{emit}}}
\mathbf 1[\ell_j=U].
$$

无返回数：

$$
N_i^{NR}
=
\sum_{j\in\mathcal J_i^{\mathrm{emit}}}
\mathbf 1[\ell_j=NR].
$$

未阻挡射线数：

$$
N_i^{\mathrm{unblocked}}
=
N_i^{\mathrm{emit}}
-
N_i^{\mathrm{map}}
-
N_i^{\mathrm{target}},
$$

若一条射线同时满足两类遮挡，必须使用“最近遮挡层”分类，避免重复扣除。

浮空支持：

$$
q_i^F
=
1-
\exp
\left(
-\frac{N_i^F}
{N_F^{\mathrm{ref}}}
\right).
$$

未知比例：

$$
q_i^U
=
\frac{
N_i^U
}{
\max(1,N_i^{\mathrm{emit}})
}.
$$

---

# 23. 第十二步：可见性状态判定

第一版只保留六种状态。

必须实现优先级和滞回，不要用互相冲突的独立 if。

建议优先级：

```text
UNSAMPLED
MAP_OCCLUDED
TARGET_OCCLUDED
VISIBLE
VISIBLE_MISS
AMBIGUOUS
```

## 23.1 `UNSAMPLED`

若：

$$
N_i^{\mathrm{emit}}
<
N_{\min}^{\mathrm{scan}},
$$

则：

$$
s_i=\mathrm{UNSAMPLED}.
$$

含义：

> 当前实际发射射线不足以覆盖该目标预测实体。

## 23.2 `MAP_OCCLUDED`

若：

$$
N_i^{\mathrm{emit}}
\ge
N_{\min}^{\mathrm{scan}},
$$

且：

$$
q_i^{\mathrm{map}}
>
\tau_{\mathrm{map}},
$$

则：

$$
s_i=\mathrm{MAP\_OCCLUDED}.
$$

## 23.3 `TARGET_OCCLUDED`

若：

$$
q_i^{\mathrm{target}}
>
\tau_{\mathrm{target}},
$$

且主要遮挡者有当前返回支持，则：

$$
s_i=\mathrm{TARGET\_OCCLUDED}.
$$

## 23.4 `VISIBLE`

若：

$$
N_i^F
\ge
N_{\min}^{F},
$$

且：

$$
q_i^{\mathrm{map}}
\le
\tau_{\mathrm{map}},
$$

$$
q_i^{\mathrm{target}}
\le
\tau_{\mathrm{target}},
$$

则：

$$
s_i=\mathrm{VISIBLE}.
$$

## 23.5 `VISIBLE_MISS`

若：

$$
N_i^{\mathrm{unblocked}}
\ge
N_{\min}^{\mathrm{unblocked}},
$$

但：

$$
N_i^F
<
N_{\min}^{F},
$$

且未知比例不高，则：

$$
s_i=\mathrm{VISIBLE\_MISS}.
$$

含义：

> 当前有足够实际、未阻挡射线穿过目标预测区域，但没有得到可靠浮空候选返回。

## 23.6 `AMBIGUOUS`

其他情况：

$$
s_i=\mathrm{AMBIGUOUS}.
$$

常见原因：

- 轨迹预测范围太大；
- 未知语义比例过高；
- 目标间深度次序不稳定；
- 浮空返回不足；
- 地图置信度不足。

---

# 24. 状态滞回

为避免状态抖动，维护：

```text
candidate_state
candidate_count
confirmed_state
```

进入新状态要求连续：

$$
N_{\mathrm{enter}}
$$

个窗口满足。

退出遮挡状态要求连续：

$$
N_{\mathrm{exit}}
$$

个窗口恢复。

第一版建议：

```yaml
state_enter_frames: 2
state_exit_frames: 2
```

但参数必须进入 YAML。

---

# 25. 第十三步：由浮空射线直接形成轨迹量测

对于 `VISIBLE` 轨迹，收集：

$$
\mathcal P_i^F
=
\left\{
\mathbf p_j:
j\in\mathcal J_i^F
\right\}.
$$

第一版使用鲁棒加权中心：

$$
\mathbf z_i
=
\frac{
\sum_{j\in\mathcal J_i^F}
w_{ij}\mathbf p_j
}{
\sum_{j\in\mathcal J_i^F}
w_{ij}
}.
$$

建议权重：

$$
w_{ij}
=
w_{j}^{\mathrm{semantic}}
w_{ij}^{\mathrm{gate}}
w_{ij}^{\mathrm{depth}}.
$$

其中：

$$
w_{j}^{\mathrm{semantic}}
=
c_j,
$$

$c_j$ 是射线语义置信度。

关联门权重：

$$
w_{ij}^{\mathrm{gate}}
=
\exp
\left(
-\frac{1}{2}
(\mathbf p_j-\hat{\mathbf c}_i)^\top
\mathbf S_i^{-1}
(\mathbf p_j-\hat{\mathbf c}_i)
\right).
$$

深度权重：

$$
w_{ij}^{\mathrm{depth}}
=
\exp
\left(
-\frac{
e_{ij,r}^2
}{
2\sigma_{r,i}^2
}
\right),
$$

其中：

$$
e_{ij,r}
=
\operatorname{dist}
\left(
r_j,
[\rho_{ij}^{-},\rho_{ij}^{+}]
\right).
$$

第一版也可把所有可靠分配射线权重设为 1，但必须保留接口。

---

# 26. 量测协方差

点集协方差：

$$
\mathbf C_i
=
\frac{
\sum_j
w_{ij}
(\mathbf p_j-\mathbf z_i)
(\mathbf p_j-\mathbf z_i)^\top
}{
\sum_jw_{ij}
}.
$$

量测协方差：

$$
\mathbf R_i
=
\frac{
\mathbf C_i
}{
\max(1,N_i^F)
}
+
\mathbf R_{\min}
+
\mathbf R_{\mathrm{sparse}}.
$$

其中：

- $\mathbf R_{\min}$：传感器基础噪声；
- $\mathbf R_{\mathrm{sparse}}$：稀疏点补偿；
- 当 $N_i^F$ 较少时，$\mathbf R_{\mathrm{sparse}}$ 增大。

例如：

$$
\mathbf R_{\mathrm{sparse}}
=
\frac{
\lambda_s
}{
\max(1,N_i^F)
}
\mathbf I.
$$

---

# 27. 表面点—目标中心偏差风险

Mid-360 通常只能看到目标朝向传感器的一侧表面。

因此：

$$
\mathbf z_i
$$

不一定是真实目标质心。

第一版为了公平比较，应：

- VoFOD 基线和 RS-VoFOD 都使用返回点几何中心；
- 两者使用相同的跟踪后端；
- 通过增大量测协方差吸收表面偏差；
- 不在第一版加入复杂中心偏差补偿。

需要在文档中明确：

> 本项目第一阶段比较的是检测连续性、可见性判断和身份保持，不宣称解决精确三维质心恢复。

可选后续模块：

```text
surface_to_center_bias_compensation
```

不得混入第一版主方法。

---

# 28. 第十四步：条件化轨迹更新

## 28.1 `VISIBLE`

执行：

$$
\text{预测}
+
\text{量测修正}.
$$

允许：

- 更新位置和速度；
- 提升存在分数；
- 更新最近可见时间；
- 保存当前支持射线。

## 28.2 `UNSAMPLED`

只预测：

$$
\hat{\mathbf x}_{i,k|k}
=
\hat{\mathbf x}_{i,k|k-1}.
$$

存在分数不惩罚或只做极小衰减。

不能把未扫描当成漏检。

## 28.3 `MAP_OCCLUDED`

只预测：

- 不使用墙面点；
- 不降低或轻微降低存在分数；
- 保留轨迹；
- 记录静态遮挡状态。

## 28.4 `TARGET_OCCLUDED`

只预测：

- 不使用前方目标的返回；
- 禁止用同一射线或同一量测更新前后两条轨迹；
- 遮挡组内禁止普通近邻轨迹合并；
- 保留原 ID。

## 28.5 `VISIBLE_MISS`

只预测，但降低存在分数：

$$
e_{i,k}
=
\max
\left(
0,
e_{i,k-1}-\alpha_{\mathrm{miss}}
\right).
$$

连续多个 `VISIBLE_MISS` 后才允许删除。

## 28.6 `AMBIGUOUS`

只预测，采用比 `VISIBLE_MISS` 更小的惩罚：

$$
e_{i,k}
=
\max
\left(
0,
e_{i,k-1}-\alpha_{\mathrm{amb}}
\right),
$$

并要求：

$$
\alpha_{\mathrm{amb}}
<
\alpha_{\mathrm{miss}}.
$$

---

# 29. 轨迹存在分数

第一版不必实现复杂 Bernoulli 滤波，使用简单存在分数：

$$
e_i\in[0,1].
$$

更新：

$$
e_{i,k}
=
\operatorname{clip}
\left(
e_{i,k-1}
+
\Delta e(s_i),
0,1
\right).
$$

其中：

$$
\Delta e(s_i)
=
\begin{cases}
+\alpha_{\mathrm{hit}}, & s_i=\mathrm{VISIBLE},\\
0, & s_i=\mathrm{UNSAMPLED},\\
-\alpha_{\mathrm{occ}}, & s_i=\mathrm{MAP\_OCCLUDED},\\
-\alpha_{\mathrm{occ}}, & s_i=\mathrm{TARGET\_OCCLUDED},\\
-\alpha_{\mathrm{miss}}, & s_i=\mathrm{VISIBLE\_MISS},\\
-\alpha_{\mathrm{amb}}, & s_i=\mathrm{AMBIGUOUS}.
\end{cases}
$$

要求：

$$
\alpha_{\mathrm{occ}}
\ll
\alpha_{\mathrm{miss}},
$$

$$
\alpha_{\mathrm{amb}}
<
\alpha_{\mathrm{miss}}.
$$

轨迹删除条件：

```text
e_i < existence_delete_threshold
且
距离最近一次VISIBLE超过最小时间
且
当前不处于确认遮挡状态
```

---

# 30. 第十五步：新目标初始化

没有已有轨迹的新目标不能使用轨迹预测聚合。

使用未分配的 `FLOATING_CANDIDATE` 射线端点：

$$
\mathcal P_k^{\mathrm{new}}
=
\left\{
\mathbf p_j:
\ell_j=F,
j\text{ 未被已有轨迹可靠分配}
\right\}.
$$

第一版使用：

```text
体素连通成分
+
短时间持续性
```

初始化条件建议：

1. 当前成分浮空语义置信度足够；
2. 不靠近已有轨迹；
3. 至少包含 $N_{\min}^{\mathrm{birth}}$ 个返回点；
4. 或在连续 $K_{\mathrm{birth}}$ 个窗口中出现于动力学可达区域；
5. 不与静态背景连接；
6. 目标尺寸不超过上限。

对于单点稀疏目标，可设置：

```text
单帧不初始化
跨2到3个窗口持续后初始化
```

不能让单个孤立浮空候选点立即产生正式轨迹。

---

# 31. 第十六步：地图更新

为防止当前返回点先污染地图再分类自己，RS-VoFOD 使用：

```text
历史地图 M_{k-1}
→ 当前射线语义和轨迹判断
→ 最后提交地图更新，得到 M_k
```

基线 VoFOD 可保留原更新顺序，但必须记录差异并设计公平消融。

## 31.1 有效背景返回

对于：

$$
\ell_j=B,
$$

- 传感器到返回点前：自由空间软更新；
- 返回端点：占据软更新。

## 31.2 浮空候选返回

对于：

$$
\ell_j=F,
$$

- 传感器到返回点前：自由空间软更新；
- 返回端点：不强化长期静态占据；
- 可更新为动态/暂态状态；
- 防止飞行目标在静态地图留下占据拖尾。

## 31.3 未知返回

对于：

$$
\ell_j=U,
$$

- 射线路径可进行保守自由更新；
- 端点更新为未知或暂态占据；
- 不直接强化为可信背景。

## 31.4 无返回射线

对于：

$$
\ell_j=NR,
$$

沿实际发射方向更新至：

$$
r_j^{\mathrm{free}}
=
\min
\left(
r_{\mathrm{raycast,max}},
r_{\mathrm{no-return,reliable}}
\right).
$$

无返回射线只提供软自由证据。

建议分开设置：

```yaml
free_update_weight_valid_return: ...
free_update_weight_no_return: ...
```

第一版基线和 RS-VoFOD 使用相同无返回自由更新配置，避免不公平。

---

# 32. 完整逐帧算法伪代码

```text
Input:
    RayBundle R_k
    previous static occupancy map M_{k-1}
    previous tracks T_{k-1}
    observer pose buffer

Output:
    ray semantics
    target measurements
    visibility states
    updated tracks T_k
    updated map M_k

1. Validate RayBundle fields and timestamps.
2. Transform every ray origin and direction to world coordinates.
3. Predict every existing track to the current window.
4. Build physical target extents E_obj and association gates E_gate.
5. Extract valid return endpoints.
6. Build local voxel-connected return components.
7. For each component:
       query M_{k-1};
       run adapted VoFOD background/floating/unknown classification;
       assign the component label back to all member return rays.
8. Label no-return rays as NR.
9. For every track:
       intersect every relevant emitted ray with E_obj;
       obtain J_emit and entry/exit ranges.
10. For every floating-candidate return ray:
       find candidate tracks using E_gate and depth consistency;
       assign to a unique track if confidence is sufficient;
       otherwise mark identity ambiguous.
11. For every track:
       query static-map blocking for intersecting rays;
       compute target-to-target blocking;
       count N_emit, N_F, N_B, N_U, N_NR,
             N_map, N_target and N_unblocked.
12. Determine the candidate visibility state:
       UNSAMPLED / MAP_OCCLUDED /
       TARGET_OCCLUDED / VISIBLE /
       VISIBLE_MISS / AMBIGUOUS.
13. Apply state hysteresis.
14. For VISIBLE tracks:
       compute weighted ray-endpoint measurement z_i;
       compute measurement covariance R_i;
       run Kalman correction.
15. For other states:
       predict only;
       update existence score according to the state.
16. From unassigned floating-candidate rays:
       update new-target birth buffers;
       initialize confirmed new tracks.
17. Update the static occupancy map:
       background endpoints -> static occupied;
       floating endpoints -> dynamic/tentative;
       unknown endpoints -> conservative;
       valid-return paths -> free;
       no-return rays -> soft free to reliable range.
18. Publish diagnostics, ray semantics, visibility states,
    detections, tracks and profiling.
```

---

# 33. 节点划分

## 33.1 `mid360_ray_publisher`

职责：

- 仿真插件完整射线输出；
- 保留原 PointCloud2；
- 无返回方向输出；
- 扫描模式索引和时间。

## 33.2 `mid360_ray_preprocessor`

职责：

- 时间检查；
- TF 插值；
- 世界坐标射线；
- 零点过滤；
- self-mask；
- 诊断。

## 33.3 `rs_vofod_semantic_node`

职责：

- 有效返回端点体素化；
- 局部连通成分；
- VoFOD式成分分类；
- 射线语义输出；
- 延迟地图更新。

## 33.4 `rs_vofod_tracker_node`

职责：

- 轨迹预测；
- 目标物理范围；
- 数据关联门；
- 射线—实体相交；
- 静态遮挡；
- 目标间遮挡；
- 射线身份分配；
- 可见性状态；
- 量测和轨迹更新；
- 新目标初始化。

## 33.5 `rs_vofod_evaluator`

职责：

- 读取 Gazebo 真值；
- 只用于评估；
- 生成检测、跟踪和状态指标；
- 不向感知节点反馈真值。

---

# 34. ROS话题建议

输入：

```text
/uav1/mid360/rays_raw
/uav1/mid360/points_raw
/tf
/tf_static
```

中间输出：

```text
/uav1/mid360/rays_world
/uav1/mid360/points_valid
/rs_vofod/ray_semantics
/rs_vofod/components
/rs_vofod/occupancy_map
/rs_vofod/visibility_states
```

最终输出：

```text
/rs_vofod/detections
/rs_vofod/tracks
/rs_vofod/markers
/rs_vofod/profiling
```

评估：

```text
/sim_ground_truth/targets
/sim_ground_truth/visibility
/rs_vofod_evaluation/metrics
```

---

# 35. 参数文件

创建：

```text
config/rs_vofod.yaml
```

至少包含：

```yaml
ray:
  min_range: 0.3
  max_range: 30.0
  no_return_reliable_range: 20.0
  use_per_ray_pose: true

component:
  voxel_size: 0.15
  connectivity: 26
  max_point_distance: 0.35
  allow_single_point_component: true

vofod_semantic:
  background_distance_threshold: ...
  bfs_max_voxels: ...
  bfs_max_radius: ...
  free_score_threshold: ...
  occupied_score_threshold: ...
  unknown_confidence_threshold: ...

target_extent:
  radius_x: 0.35
  radius_y: 0.35
  radius_z: 0.20

association:
  gate_chi_square: ...
  depth_margin: 0.20
  ambiguity_margin: ...
  max_mahalanobis_distance: ...

visibility:
  min_scan_rays: ...
  min_unblocked_rays: ...
  min_floating_returns: ...
  map_occlusion_ratio: ...
  target_occlusion_ratio: ...
  max_unknown_ratio: ...
  enter_frames: 2
  exit_frames: 2

measurement:
  min_covariance: ...
  sparse_covariance_scale: ...
  semantic_weight_enabled: true
  gate_weight_enabled: true
  depth_weight_enabled: true

existence:
  hit_increment: ...
  occlusion_decrement: ...
  miss_decrement: ...
  ambiguous_decrement: ...
  delete_threshold: ...
  minimum_delete_delay: ...

birth:
  min_points_single_frame: ...
  confirmation_frames: ...
  max_speed: ...
  max_component_size: ...

map:
  raycast_max_distance: 20.0
  free_update_weight_valid_return: ...
  free_update_weight_no_return: ...
  background_occupied_weight: ...
  unknown_endpoint_weight: ...
  update_after_semantic_classification: true
```

所有阈值必须在运行日志中输出。

---

# 36. 风险清单及处理方式

## 36.1 单条射线不能独立证明浮空

风险：

- 墙边；
- 细杆；
- 地图孔洞；
- 位姿误差；
- 孤立噪声；

都可能产生非背景返回。

处理：

- 语义仍由局部连通成分和 VoFOD BFS 提供；
- 射线继承成分标签；
- 不对每个点独立执行最终飞行目标判定。

---

## 36.2 一个成分可能包含两个目标

风险：

- 两目标接近；
- 返回点在体素图中连接；
- 原始 VoFOD 会输出一个质心。

处理：

- 成分只提供 `FLOATING_CANDIDATE` 空间语义；
- 不将成分直接当成目标实例；
- 后续根据轨迹物理范围和深度层，将射线分配给不同轨迹；
- 不使用成分统一质心更新所有轨迹。

---

## 36.3 轨迹预测自我确认

风险：

- 错误轨迹不断吸收附近点；
- 轨迹预测把未知点解释成自身目标。

处理：

- 射线空间语义完全独立于轨迹预测；
- 只有已经被静态地图判为浮空候选的返回才可分配给轨迹；
- 背景点和未知点不能因为落入关联门而升级为浮空。

---

## 36.4 无返回不等于目标不存在

风险：

- 低反射率；
- 不利入射角；
- 目标边缘；
- 距离过远；
- 仿真与实机回波差异。

处理：

- 单条无返回只算软负证据；
- 只有 `VISIBLE_MISS` 才降低存在分数；
- `VISIBLE_MISS` 要求足够多未阻挡射线；
- 连续多窗口后才删除轨迹。

---

## 36.5 无返回射线自由空间误更新

风险：

- 真实设备无返回可能不是纯几何空闲；
- 直接更新到最大量程可能过强。

处理：

- 使用软权重；
- 设置 `no_return_reliable_range`；
- 仿真和实机参数分离；
- 基线和主方法保持相同自由空间更新，保证公平。

---

## 36.6 表面点到质心的系统偏差

风险：

- 只看到目标近侧表面；
- 返回中心不是真实无人机中心。

处理：

- 第一版基线和主方法使用相同中心量测策略；
- 增大量测协方差；
- 不把中心精度提升作为第一版主要贡献；
- 将中心偏差补偿留作后续模块。

---

## 36.7 目标物理范围和关联门混淆

风险：

- 预测协方差变大后，扫描机会被严重高估；
- 大量无关射线被认为扫描了目标。

处理：

- `E_obj` 只表示目标实体尺寸；
- `E_gate` 表示统计关联范围；
- 扫描和遮挡只使用 `E_obj`；
- 量测匹配使用 `E_gate`。

---

## 36.8 轨迹间共享射线

风险：

- 一条返回射线同时进入两个关联门；
- 重复更新多条轨迹；
- 两条轨迹向同一点收敛。

处理：

- 使用深度区间和马氏距离唯一分配；
- 代价差太小时标记为 ambiguous；
- 歧义射线不用于位置修正；
- 不允许复制射线。

---

## 36.9 目标间遮挡由预测误差虚构

风险：

- 两条预测轨迹重叠但实际未遮挡；
- 误判 `TARGET_OCCLUDED`。

处理：

- 除预测几何外，要求前方目标有当前浮空返回支持；
- 使用状态滞回；
- 遮挡深度设置裕量；
- 输出遮挡置信度。

---

## 36.10 静态地图误差造成错误遮挡

风险：

- 地图漂移；
- 当前动态目标残留；
- 体素离散；
- observer 位姿误差。

处理：

- 使用历史地图；
- 静态遮挡要求多个射线一致；
- 设置遮挡深度裕量；
- 动态目标端点不强化静态占据；
- 记录地图置信度。

---

## 36.11 完全遮挡期间目标运动不可观测

风险：

- 后方目标突然变向；
- 重新出现时预测偏差很大。

处理：

- 只预测并扩大协方差；
- 不声称恢复完全遮挡期间真实运动；
- 重新出现时允许更大关联门；
- 仍需限制门，避免错误重关联。

---

## 36.12 Mid-360逐点时间与运动畸变

风险：

- 插件可能只给点添加时间戳，但所有射线在同一 Gazebo 时刻 raycast；
- 强行逐点去畸变会过补偿。

处理：

- 实现 snapshot/per_ray_pose 两种模式；
- 用静态墙和运动 observer 验证；
- 未验证前不默认启用逐点补偿。

---

## 36.13 仿真无返回方向与实机不一致

风险：

- 仿真可精确获得全部射线方向；
- 实机固件可能不输出零深度样本角度。

处理：

- 算法使用统一 RayBundle；
- 实机必须通过球坐标 capability probe；
- 若无法获得精确无返回方向，降级到 calibrated fallback；
- 论文必须明确仿真和实机输入差异。

---

## 36.14 新目标单点误初始化

风险：

- 孤立噪声；
- 地图边缘；
- 偶然浮空候选。

处理：

- 单点不能单帧初始化正式轨迹；
- 需要跨窗口持续；
- 使用速度可达性；
- 需要达到最小存在证据。

---

## 36.15 计算量

风险：

- 每窗口约 20,000 条射线；
- 多轨迹射线相交；
- 静态地图 raycast；
- BFS。

处理：

1. 先用轨迹角度包围盒筛选射线；
2. 再做精确射线—椭球相交；
3. 静态地图 raycast 只对穿过轨迹范围的射线执行；
4. 每个返回成分只做一次 BFS；
5. 同一体素的地图查询缓存；
6. 输出各模块 P50/P95 时间。

---

## 36.16 基线比较不公平

风险：

- 主方法使用更强跟踪器；
- 基线不使用无返回射线；
- 地图参数不同；
- 输入 rosbag 不同。

处理：

至少设置：

```text
B0:
VoFOD-Mid360原始风格检测与跟踪

B1:
VoFOD点簇检测
+
与RS-VoFOD完全相同的CV/KF和轨迹管理
+
不使用射线级量测和可见性

P:
RS-VoFOD射线语义
+
相同CV/KF后端
+
可见性条件化更新
```

主结论优先比较 B1 与 P，以隔离前端和可见性机制的作用。

---

# 37. 最小实验设计

实验先只做三个场景。所有方法使用同一 rosbag 回放。

---

## 场景 S1：稀疏回波单目标与背景分离

### 目的

验证：

- 射线级浮空候选语义；
- 少量返回时已有轨迹能否持续更新；
- RS-VoFOD 是否比点簇级 VoFOD 更少漏检；
- 是否在背景墙附近产生额外误检。

### 场景

```text
observer:
    静止悬停

background:
    一面后墙
    两根柱体

target:
    一架无人机
    先悬停
    再缓慢横向运动
    距离从近到远变化
```

需要覆盖：

```text
多点返回
双点返回
单点返回
短时无返回
靠近背景边缘
```

### 指标

- 可见时检测 recall；
- false positive；
- 单点/双点窗口下轨迹更新成功率；
- 初始化延迟；
- 位置 RMSE；
- 射线语义准确率；
- runtime。

### 预期

RS-VoFOD 应在：

- 已有轨迹阶段；
- 点数低于 VoFOD 聚类阈值时；

保持更高轨迹更新率。

---

## 场景 S2：两目标交叉和前后遮挡

### 目的

验证：

- 一个浮空成分包含两个目标时，射线能否按轨迹分配；
- 后方目标是否避免被前方目标量测拉偏；
- 重新分开后是否保持原 ID。

### 场景

observer：

$$
\mathbf p_O=(0,0,4).
$$

目标 A：

$$
(8,-3,4)
\rightarrow
(8,3,4).
$$

目标 B：

$$
(12,3,4)
\rightarrow
(12,-3,4).
$$

两目标在观察方向上交叉，A 位于前方，B 位于后方。

设置两个子情况：

```text
S2a:
部分遮挡，两个目标仍有少量独立返回

S2b:
短时完全遮挡，后方目标无返回
```

### 指标

- ID switches；
- fragmentation；
- 后方轨迹存活率；
- `TARGET_OCCLUDED` precision/recall；
- 遮挡起止检测延迟；
- 重新分离 ID 恢复正确率；
- 遮挡期间预测 RMSE；
- 同一量测错误更新多轨迹的次数。

### 预期

RS-VoFOD 应：

- 不将一个合并成分质心同时更新两条轨迹；
- 后方轨迹进入 `TARGET_OCCLUDED`；
- 减少 ID switch 和轨迹合并。

---

## 场景 S3：静态墙遮挡与扫描不足区分

### 目的

验证：

- `MAP_OCCLUDED`；
- `UNSAMPLED`；
- `VISIBLE_MISS`；

是否能够正确区分。

### 场景

包含：

```text
一面窄墙
一个门框
一架目标无人机
```

目标依次经历：

1. 正常可见；
2. 飞到墙后；
3. 从墙后重新出现；
4. 位于当前 Mid-360 射线稀疏覆盖区域；
5. 再回到高覆盖区域。

observer 第一版保持静止，避免运动因素干扰。

### 指标

- `MAP_OCCLUDED` precision/recall；
- `UNSAMPLED` precision/recall；
- `VISIBLE_MISS` false alarm；
- 静态遮挡期间轨迹存活率；
- 重捕获延迟；
- 错误删除次数。

### 预期

RS-VoFOD 不应把：

```text
未扫描
```

和：

```text
被墙遮挡
```

统一当作普通漏检。

---

# 38. 实验方法和消融

至少比较：

## B0：原始风格 VoFOD

```text
VoFOD-Mid360点簇检测
+
原始风格tracker
```

## B1：公平跟踪器基线

```text
VoFOD-Mid360点簇检测
+
与P相同的CV/KF
+
相同轨迹初始化和删除参数
+
无可见性状态
```

## P：完整 RS-VoFOD

```text
射线空间语义
+
轨迹条件化射线聚合
+
实际射线扫描机会
+
静态/目标遮挡
+
条件化轨迹更新
```

建议增加两个轻量消融：

## P-no-state

```text
使用射线量测
但所有无量测统一按普通漏检处理
```

用于验证可见性状态是否必要。

## P-return-only

```text
扫描机会只使用有效返回射线
不使用无返回射线方向
```

用于验证完整发射射线是否必要。

---

# 39. 实验运行方式

推荐：

1. 先运行仿真，只记录：
   - 原始射线；
   - 原始点云；
   - TF；
   - observer状态；
   - target真值；
   - 可见性真值；
2. 使用同一 rosbag 回放：
   - B0；
   - B1；
   - P；
   - 两个消融；
3. 所有方法使用相同：
   - 输入；
   - 地图参数；
   - 目标尺寸；
   - 跟踪过程噪声；
   - 初始化条件；
   - 删除条件；
4. 在线运行只用于最终实时性测试。

每个场景至少：

```text
5个固定随机种子
或
5次可重复运行
```

随机变化只包括：

- 目标起始相位；
- 小幅速度扰动；
- 仿真点云噪声。

不能在每种方法中使用不同随机输入。

---

# 40. 真值评估

创建独立真值节点，仅用于评估。

输出：

```text
target_id
true_position
true_velocity
true_visible
true_map_occluded
true_target_occluded
true_occluder_id
true_emitted_ray_count
true_unblocked_ray_count
```

真值节点可以读取：

- Gazebo model states；
- target collision；
- 实际发射射线；
- 静态模型 collision。

感知节点严禁订阅这些真值话题。

必须用 `rqt_graph` 或自动测试确认无真值泄漏。

---

# 41. 关键指标

## 检测

- precision；
- recall；
- localization RMSE；
- 单点/双点窗口检测率；
- 轨迹条件化射线利用率；
- 背景边缘误检率。

## 跟踪

- IDF1；
- HOTA，可选；
- ID switches；
- fragmentation；
- track recall；
- rear-track survival rate；
- reacquisition latency；
- position RMSE；
- velocity RMSE。

## 可见性

- `UNSAMPLED` precision/recall；
- `MAP_OCCLUDED` precision/recall；
- `TARGET_OCCLUDED` precision/recall；
- `VISIBLE_MISS` false positive；
- 遮挡者 ID 正确率；
- 状态切换延迟。

## 计算

- 返回成分构建时间；
- VoFOD语义分类时间；
- 射线—轨迹相交时间；
- 地图遮挡查询时间；
- 轨迹更新总时间；
- 总帧时间 P50/P95/max；
- CPU；
- 内存；
- ROS消息带宽。

---

# 42. 可视化

RViz 至少显示：

- 有返回射线端点：
  - background；
  - floating candidate；
  - unknown；
- 无返回射线抽样显示；
- 轨迹预测物理范围；
- 关联门；
- 轨迹 ID；
- 轨迹可见性状态；
- 静态遮挡射线；
- 目标遮挡射线；
- 被分配给每条轨迹的浮空返回；
- 未分配浮空候选；
- 新目标 birth buffer；
- 当前目标量测。

不要一次显示全部 20,000 条射线导致 RViz 卡死。

增加可视化下采样参数，不影响算法内部数据。

---

# 43. 实现阶段和验收门

## Phase 0：工程审计

输出：

```text
WORKSPACE_AUDIT.md
```

内容：

- 工作空间路径；
- overlay顺序；
- 当前启动命令；
- 插件路径和分支；
- VoFOD路径；
- tracker路径；
- TF树；
- PointCloud2字段；
- 当前git修改；
- 推荐分支策略。

验收：

- 不改变现有工程；
- 原 one_drone 和插件测试仍可运行。

---

## Phase 1：完整射线接口

任务：

- 创建 `mid360_ray_msgs`；
- 修改插件发布 `RayBundle`；
- 保留 PointCloud2；
- 完成空世界和单墙测试。

验收：

- 无返回射线方向非零且单位化；
- 射线数正确；
- 时间单调；
- 有效返回满足 $\mathbf p\approx r\mathbf d$。

---

## Phase 2：Mid360兼容VoFOD基线

任务：

- 移除固定 Ouster 有序点云假设；
- 有效返回和无返回射线更新自由空间；
- 跑通点簇级 VoFOD；
- 接入基线 tracker。

验收：

- 单目标检测正常；
- 背景地图正常；
- 无返回自由更新正常；
- B0 可运行。

---

## Phase 3：射线语义前端

任务：

- 返回端点体素连通成分；
- 复用 VoFOD 分类器；
- 输出每条射线的 $B/F/U/NR$ 标签；
- 延迟地图更新。

验收：

- 一个成分只执行一次 BFS；
- 分量成员射线标签一致；
- 单点成分可处理；
- 背景墙和浮空目标可区分。

---

## Phase 4：轨迹条件化射线量测

任务：

- CV预测；
- 物理实体范围；
- 关联门；
- 射线—椭球相交；
- 浮空射线唯一分配；
- 加权量测和协方差。

验收：

- 已有轨迹在单点/双点返回时可形成量测；
- 同一射线不重复分配；
- 轨迹范围和关联门可视化正确。

---

## Phase 5：可见性状态

任务：

- 实际扫描支持；
- 静态遮挡；
- 目标遮挡；
- 六状态判定；
- 滞回；
- 条件化更新。

验收：

- S2 正确进入目标遮挡；
- S3 正确区分静态遮挡和未扫描；
- 后方轨迹不被前方量测更新。

---

## Phase 6：新目标初始化

任务：

- 未分配浮空候选；
- 当前成分；
- 短时 birth buffer；
- 轨迹初始化。

验收：

- 单点噪声不立即初始化；
- 持续目标可以初始化；
- 新目标与已有目标不重复。

---

## Phase 7：实验和比较

任务：

- 三个场景；
- B0、B1、P和两个消融；
- rosbag统一回放；
- CSV/JSON指标；
- 运行报告。

验收：

- 所有方法使用同一输入；
- 输出自动汇总；
- 能生成核心对比图和表。

---

# 44. 测试要求

## 单元测试

至少包括：

```text
ray_ellipsoid_intersection_test
ray_direction_normalization_test
ray_timestamp_monotonic_test
component_connectivity_test
semantic_label_inheritance_test
physical_extent_vs_gate_test
unique_ray_assignment_test
visibility_state_logic_test
existence_score_update_test
map_update_semantic_test
```

## 集成测试

```text
empty_world_ray_test
single_wall_semantic_test
single_target_sparse_return_test
two_target_crossing_test
wall_occlusion_test
rosbag_replay_determinism_test
no_ground_truth_subscription_test
```

---

# 45. 文档交付物

必须生成：

```text
WORKSPACE_AUDIT.md
RAY_INTERFACE.md
RS_VOFOD_ALGORITHM.md
SYMBOLS_AND_FORMULAS.md
VOFOD_BASELINE_PORT.md
RISK_REGISTER.md
EXPERIMENT_SCENARIOS.md
RUNBOOK.md
TEST_REPORT.md
KNOWN_LIMITATIONS.md
```

其中 `KNOWN_LIMITATIONS.md` 必须明确：

1. 返回端点语义仍依赖局部空间成分，不是单射线独立证明；
2. 完全遮挡期间后方目标真实运动不可观测；
3. 表面点中心不等于目标质心；
4. 实机无返回方向需硬件验证；
5. 地图和 observer 位姿误差影响遮挡判断；
6. 仿真碰撞几何和真实反射模型存在差异；
7. 第一版不解决目标姿态和形状变化；
8. 第一版新目标仍需轻量聚合。

---

# 46. Codex执行纪律

每个阶段完成后必须输出：

```text
Changed files
Build command
Run command
Test command
Test result
Known issue
Next exact step
```

禁止：

- 只写方案不创建文件；
- 未编译就继续下一阶段；
- 隐藏编译错误；
- 删除原插件输出；
- 修改系统 MRS 安装目录；
- 在算法节点订阅 target 真值；
- 使用目标真值调参；
- 把一个返回射线重复更新多条轨迹；
- 将轨迹预测用于改变背景/浮空语义；
- 把所有未匹配轨迹统一当普通漏检；
- 一次性实现所有高级模块。

---

# 47. Codex首轮现在只做的任务

第一次执行只完成：

1. 审计当前工程；
2. 输出 `WORKSPACE_AUDIT.md`；
3. 创建 `mid360_ray_msgs`；
4. 修改仿真插件发布完整 `RayBundle`；
5. 保留原 PointCloud2；
6. 创建空世界和单墙射线测试；
7. 创建射线 validator；
8. 输出 `RAY_INTERFACE.md`；
9. 编译并运行测试；
10. 给出 Phase 2 的准确实施入口。

首轮不要直接实现完整 RS-VoFOD。

---

# 48. 最终算法一句话概括

> RS-VoFOD首先利用历史占据地图和局部返回端点连通关系，为每条实际发射射线赋予背景一致、浮空候选、未知或无返回语义；随后在每条已有轨迹的预测物理范围内聚合这些射线，联合判断目标是否被扫描、被静态背景遮挡、被其他目标遮挡或应当可见但未返回，并直接由浮空候选射线形成轨迹量测，从而实现可见性条件化的多目标状态更新。
