# Codex 提示词：无扫描机会模块的射线语义 VoFOD 核心版

> 工程暂用名：**RS-VoFOD-Core**  
> 中文名称：**射线语义驱动的飞行目标检测与遮挡感知跟踪核心版**  
> 平台：Ubuntu 20.04、ROS 1 Noetic、Gazebo 11、MRS UAV System、CTU-MRS Mid-360 仿真插件  
> 使用方式：在当前工程根目录打开 Codex，将本文件完整内容作为任务提示词。  
> 核心要求：先审计现有工程，再分阶段实现；不得一次性重写全部系统；每阶段必须可编译、可启动、可测试、可回退。

---

# 1. 你的角色

你是一名高级机器人感知与 ROS 工程师，熟悉：

- ROS 1 Noetic；
- Gazebo 11；
- MRS UAV System；
- Livox Mid-360 非重复扫描；
- CTU-MRS Mid360 simulation plugin；
- VoFOD 的三维占据地图、自由空间更新和 BFS/flood-fill 浮空判定；
- LiDAR 多目标检测与跟踪；
- 卡尔曼滤波、数据关联、静态遮挡和目标间遮挡；
- C++17、PCL、Eigen、tf2、ROS message/filter；
- rosbag、rostest、gtest 和批量实验。

你的任务是在当前已运行工程基础上实现一个**以射线端点语义为前端、以统一轨迹预测支撑模型为聚合依据、以遮挡状态控制轨迹更新**的简化算法。

本版本明确删除“扫描机会模块”，不计算：

- 某条轨迹被多少实际射线穿过；
- `UNSAMPLED`；
- `VISIBLE_MISS`；
- 轨迹相关扫描概率；
- 无返回射线对目标存在性的负观测。

---

# 2. 当前工程基础

当前工程已经完成：

- Ubuntu 20.04；
- ROS 1 Noetic；
- Gazebo 11；
- MRS UAV System；
- ROS、Gazebo 和 MRS 环境变量配置；
- MRS 原生 `one_drone` 仿真已成功运行；
- Mid-360 插件工作空间已创建；
- `ctu-mrs/Mid360_simulation_plugin` 已克隆并编译；
- 插件独立测试已成功运行；
- 已确认 `sensor_msgs/PointCloud2` 包含 `x、y、z、intensity、tag、line、timestamp`。

当前已知的重要事实：

1. VoFOD 原算法使用有返回和无返回射线更新自由空间；
2. 当前 Mid-360 仿真插件把无返回射线写成 `(0,0,0)`；
3. 插件内部保留发射方向，因此应新增完整射线接口；
4. 在本核心版中：
   - 无返回射线只用于 VoFOD 自由空间软更新；
   - 无返回射线不进入轨迹状态判断；
   - 有返回射线端点进入背景、浮空候选和未知语义分类；
5. 轨迹状态只保留：
   - `VISIBLE`；
   - `MAP_OCCLUDED`；
   - `TARGET_OCCLUDED`；
   - `UNOBSERVED`。

---

# 3. 核心算法定位

原始 VoFOD：

```text
当前点云
→ 欧氏聚类
→ 依据历史占据地图分类整个点簇
→ 输出飞行点簇质心
→ 多目标跟踪
```

RS-VoFOD-Core：

```text
全部发射射线
→ 无返回射线更新自由空间
→ 有返回端点形成局部空间成分
→ VoFOD式背景/浮空候选/未知语义
→ 语义标签回写到每条返回射线
→ 浮空候选射线按统一预测支撑模型分配给轨迹
→ 直接形成已有轨迹量测
→ 无量测轨迹检查静态遮挡和目标间遮挡
→ 根据可见/遮挡/未观测状态更新轨迹
→ 未分配浮空候选用于初始化新目标
```

核心变化不是“每个点完全独立分类”，而是：

> 返回射线是最终观测单位，局部连通成分只提供空间上下文，不再被强制解释成一个目标实例，也不再只输出一个质心。

---

# 4. 三个核心创新

## 4.1 射线端点空间语义

有返回射线端点获得：

```text
BACKGROUND_CONSISTENT
FLOATING_CANDIDATE
UNKNOWN_RETURN
```

无返回射线获得：

```text
NO_RETURN
```

空间语义主要由历史占据地图和局部连通关系决定，轨迹预测不得反向修改语义。

## 4.2 已有轨迹直接聚合浮空候选射线

对于已有轨迹，不再要求先生成完整 VoFOD 目标点簇。只要浮空候选端点落入轨迹预测支撑模型，并且身份不歧义，就可以直接：

- 形成轨迹位置量测；
- 估计量测协方差；
- 判断轨迹当前可见。

## 4.3 遮挡状态条件化轨迹更新

对没有当前量测的轨迹，区分：

```text
MAP_OCCLUDED
TARGET_OCCLUDED
UNOBSERVED
```

避免墙面点更新目标、前方目标量测拉偏后方轨迹、交叉时轨迹错误合并和短时遮挡导致过早删除。

---

# 5. 删除扫描机会后的边界

本版本不再区分：

```text
当前没有射线扫描目标区域
```

和：

```text
当前区域被扫描但目标没有产生回波
```

两者在没有静态或目标遮挡解释时统一归为 `UNOBSERVED`。

因此本版本不能宣称解决：

- Mid-360 非重复扫描造成的未扫描判定；
- `UNSAMPLED` 与真实漏检的区分；
- 基于无返回射线的目标存在性负证据。

无返回射线只保留在 VoFOD 地图更新中。

---

# 6. 明确的非目标

第一版不要实现：

- 扫描机会统计；
- `UNSAMPLED`、`VISIBLE_MISS`；
- 轨迹相关检测概率；
- 点级学习分类器；
- 深度神经网络；
- 完整 JPDA/MHT/PMBM；
- 目标姿态估计；
- 历史目标表面点姿态传播；
- 表面—质心复杂补偿；
- 五个以上目标压力测试；
- 大规模城市环境。

第一版只验证：

1. 射线语义能否替代点簇质心，持续更新已有轨迹；
2. 两目标接近或交叉时，是否减少错误合并和 ID switch；
3. 静态遮挡和目标间遮挡是否能控制轨迹更新；
4. 在相同跟踪器下是否优于点簇级 VoFOD。

---

# 7. 推荐 ROS 包结构

先审计现有工程，缺失时创建：

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
    完整射线、射线语义、轨迹状态消息

mid360_simulation_plugin_fork
    保留原PointCloud2，同时发布完整射线

mid360_ray_preprocessor
    射线时间、TF、世界坐标转换、有效点过滤

vofod_mid360_baseline
    Mid-360适配后的点簇级VoFOD基线

rs_vofod_core
    返回端点局部成分、VoFOD式空间语义、地图更新

rs_vofod_tracker
    轨迹预测、统一预测支撑模型、射线分配、遮挡、条件化更新

mid360_multi_uav_sim
    observer、目标UAV、背景和场景管理

rs_vofod_evaluation
    真值、指标、rosbag回放和批量实验
```

---

# 8. 坐标系、索引与时间

| 符号 | 含义 |
|---|---|
| $W$ | 世界或地图坐标系 |
| $L(t)$ | 时刻 $t$ 的 Mid-360 坐标系 |
| $B(t)$ | observer UAV 机体坐标系 |
| $k$ | 当前处理窗口索引 |
| $j$ | 当前窗口射线索引 |
| $i$ | 目标轨迹索引 |
| $q$ | 返回端点局部成分索引 |

第 $j$ 条射线发射时间：

$$
t_j=t_k+\Delta t_j,
$$

其中：

- $t_k$：`RayBundle.header.stamp`；
- $\Delta t_j$：射线相对窗口起始时刻的时间偏移；
- $t_j$：射线发射时刻。

第一版必须支持：

```yaml
ray_geometry_mode:
  snapshot
  per_ray_pose
```

只有验证插件确实逐点模拟运动畸变后，才启用 `per_ray_pose`。

---

# 9. 完整射线消息

## 9.1 `Ray.msg`

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

字段含义：

- `dir_x, dir_y, dir_z`：单位发射方向；
- `range`：有效返回距离；
- `offset_time_ns`：相对 bundle 起始时间；
- `pattern_index`：扫描模式索引；
- `return_status`：返回状态；
- `tag`：仿真中可能固定为零，不能作为核心判据。

## 9.2 `RayBundle.msg`

```text
std_msgs/Header header
uint32 scan_id
uint32 pattern_start_index
float32 min_range
float32 max_range
mid360_ray_msgs/Ray[] rays
```

## 9.3 话题

```text
/uav1/mid360/points_raw
/uav1/mid360/rays_raw
/uav1/mid360/points_valid
/uav1/mid360/rays_world
```

保留原 PointCloud2，不破坏现有独立测试。

---

# 10. 射线几何

第 $j$ 条世界坐标射线：

$$
\mathcal R_j(s)=\mathbf o_j+s\mathbf d_j,\qquad s\ge0,
$$

其中：

- $\mathbf o_j\in\mathbb R^3$：世界坐标射线原点；
- $\mathbf d_j\in\mathbb R^3$：世界坐标单位方向；
- $s$：沿射线的距离参数。

世界坐标方向：

$$
\mathbf d_j={}^W\mathbf R_L(t_j)\mathbf d_j^L.
$$

射线原点：

$$
\mathbf o_j={}^W\mathbf t_L(t_j).
$$

有效返回端点：

$$
\mathbf p_j=\mathbf o_j+r_j\mathbf d_j.
$$

---

# 11. 预处理器

实现 `mid360_ray_preprocessor`。

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

必须实现：

1. 射线方向有限且单位化；
2. 射线时间单调；
3. `(0,0,0)` 无返回点不能进入有效点云；
4. observer 自身点过滤；
5. 射线和 PointCloud2 有效返回一致性：

$$
\|\mathbf p_j-r_j\mathbf d_j\|<\epsilon_p.
$$

不能为每条射线执行阻塞式 TF 查询，应使用位姿缓存和插值。

---

# 12. VoFOD-Mid360 基线

创建 `vofod_mid360_baseline`，尽量保留原 VoFOD：

- 占据体素分数；
- confident occupied；
- tentative occupied；
- uncertain；
- confident free；
- 有返回射线路径自由更新；
- 无返回射线自由更新；
- 当前点聚类；
- close-to-background；
- far cluster；
- BFS/flood-fill；
- 飞行点簇质心输出。

必须移除：

- 固定 Ouster 行列点云假设；
- `vertical_rays × horizontal_rays == cloud.size()`；
- 固定二维射线 LUT；
- 依赖 Ouster 行列的 mask。

改为使用 `RayBundle` 中每条射线实际方向。

基线和主方法必须使用相同：

- 体素大小；
- 自由更新权重；
- 无返回可靠距离；
- 地图阈值；
- tracker 过程噪声；
- 轨迹删除参数。

---

# 13. 返回端点局部成分

只对有效返回端点构建局部空间成分。

体素映射：

$$
v_j=\operatorname{voxelIndex}(\mathbf p_j).
$$

构建：

$$
G_k^{\mathrm{return}}=(V_k,E_k),
$$

其中：

- 每个顶点对应一个有效返回端点；
- 两端点体素为 26 邻域时连接；
- 可增加距离约束：

$$
\|\mathbf p_a-\mathbf p_b\|<\tau_c.
$$

得到连通成分：

$$
\mathcal C_k=\{C_1,C_2,\ldots,C_Q\}.
$$

这些成分只用于空间语义上下文，不等价于目标实例。必须允许：

- 单点成分；
- 两点成分；
- 一个成分包含两个接近目标；
- 背景和目标局部混合。

---

# 14. 射线端点空间语义

对于每个返回成分 $C_q$，调用改造后的 VoFOD 分类器：

$$
g(C_q,\mathcal M_{k-1})\rightarrow(L_q,c_q),
$$

其中：

- $\mathcal M_{k-1}$：处理当前窗口前的历史占据地图；
- $L_q$：成分语义；
- $c_q\in[0,1]$：语义置信度。

成分语义：

```text
BACKGROUND_CONSISTENT
FLOATING_CANDIDATE
UNKNOWN_RETURN
```

必须复用原 VoFOD 的：

- 与可信占据背景距离；
- 自由空间包围；
- 未知体素连通；
- BFS 是否到达背景；
- BFS 是否到达搜索边界；
- 搜索体素上限。

若第 $j$ 条射线端点属于成分 $C_q$：

$$
\ell_j=
\begin{cases}
B,&L_q=\mathrm{BACKGROUND},\\
F,&L_q=\mathrm{FLOATING\_CANDIDATE},\\
U,&L_q=\mathrm{UNKNOWN}.
\end{cases}
$$

无返回射线：

$$
\ell_j=NR.
$$

最终：

$$
\ell_j\in\{B,F,U,NR\}.
$$

---

# 15. 防止轨迹自我确认

硬性要求：返回端点的 $B/F/U$ 语义只能由：

- 历史占据地图；
- 当前返回端点；
- 局部连通成分；
- VoFOD 空间判据；

决定。

轨迹预测不能：

- 把 $B$ 改成 $F$；
- 把 $U$ 改成 $F$；
- 因为端点靠近轨迹，就提升浮空语义置信度。

轨迹预测只能用于：

- 将已有 $F$ 射线分配给轨迹；
- 形成目标量测；
- 判断遮挡。

---

# 16. 轨迹状态模型

第一版使用恒速度状态：

$$
\mathbf x_i=
\begin{bmatrix}
x_i&y_i&z_i&v_{x,i}&v_{y,i}&v_{z,i}
\end{bmatrix}^{\top}.
$$

预测：

$$
\hat{\mathbf x}_{i,k|k-1}=\mathbf F_k\hat{\mathbf x}_{i,k-1|k-1},
$$

$$
\mathbf P_{i,k|k-1}=\mathbf F_k\mathbf P_{i,k-1|k-1}\mathbf F_k^\top+\mathbf Q_k.
$$

预测中心：

$$
\hat{\mathbf c}_i=
\begin{bmatrix}
\hat x_i&\hat y_i&\hat z_i
\end{bmatrix}^{\top}.
$$

位置协方差：

$$
\mathbf P_{i,\mathrm{pos}}=\mathbf P_{i,k|k-1}(1\!:\!3,1\!:\!3).
$$

---

# 17. 统一预测支撑模型

本版本不再维护独立“物理门”和“关联门”。

统一预测支撑协方差：

$$
\mathbf\Sigma_i^{\mathrm{sup}}=
\mathbf\Sigma_i^{\mathrm{shape}}+
\alpha_P\mathbf P_{i,\mathrm{pos}}+
\mathbf\Sigma_i^{\mathrm{surface}}+
\mathbf\Sigma_i^{\mathrm{ego}}.
$$

各项含义：

- $\mathbf\Sigma_i^{\mathrm{shape}}$：目标实体尺寸；
- $\mathbf P_{i,\mathrm{pos}}$：轨迹预测位置不确定性；
- $\alpha_P$：预测误差膨胀系数；
- $\mathbf\Sigma_i^{\mathrm{surface}}$：可见表面点到目标中心的偏差；
- $\mathbf\Sigma_i^{\mathrm{ego}}$：observer 位姿、时间和去畸变误差；
- $\mathbf\Sigma_i^{\mathrm{sup}}$：统一预测支撑模型。

第一版使用轴对齐形状矩阵：

$$
\mathbf\Sigma_i^{\mathrm{shape}}=
\operatorname{diag}(\sigma_x^2,\sigma_y^2,\sigma_z^2).
$$

统一归一化距离：

$$
D_{ij}^2=
(\mathbf p_j-\hat{\mathbf c}_i)^\top
(\mathbf\Sigma_i^{\mathrm{sup}})^{-1}
(\mathbf p_j-\hat{\mathbf c}_i).
$$

使用同一模型的两个置信轮廓：

$$
\gamma_{\mathrm{occ}}<\gamma_{\mathrm{assoc}}.
$$

- $\gamma_{\mathrm{assoc}}$：较宽的返回关联轮廓；
- $\gamma_{\mathrm{occ}}$：较保守的遮挡几何轮廓；
- 两者不是两套独立门，只是同一预测支撑分布的两个置信层级。

设置每个半轴最大膨胀，防止遮挡期间协方差无限增大导致支撑区域失去物理意义。

---

# 18. 浮空候选射线分配

只处理：

$$
\ell_j=F.
$$

候选轨迹集合：

$$
\mathcal I_j=\{i:D_{ij}^2\le\gamma_{\mathrm{assoc}}\}.
$$

## 18.1 无候选

若 $|\mathcal I_j|=0$，射线进入新目标候选池。

## 18.2 唯一候选

若 $|\mathcal I_j|=1$，分配给唯一轨迹。

## 18.3 多候选

排序：

$$
D_{(1)j}^2\le D_{(2)j}^2\le\cdots.
$$

若：

$$
D_{(2)j}^2-D_{(1)j}^2\ge\tau_{\mathrm{amb}},
$$

分配给最优轨迹；否则标记 `AMBIGUOUS_ASSIGNMENT`。

歧义射线：

- 不用于任何轨迹位置量测；
- 不得重复分配；
- 可用于可视化和统计。

---

# 19. 轨迹浮空支持集合

轨迹 $i$ 获得：

$$
\mathcal J_i^F=\{j:\ell_j=F,\ j\rightarrow i\}.
$$

浮空支持数：

$$
N_i^F=|\mathcal J_i^F|.
$$

浮空支持点集：

$$
\mathcal P_i^F=\{\mathbf p_j:j\in\mathcal J_i^F\}.
$$

若：

$$
N_i^F\ge N_{\min}^F,
$$

则轨迹候选为 `VISIBLE`。

必须允许单点、双点和多点量测，并根据点数动态设置量测协方差。

---

# 20. 射线端点量测

轨迹量测：

$$
\mathbf z_i=
\frac{\sum_{j\in\mathcal J_i^F}w_{ij}\mathbf p_j}
{\sum_{j\in\mathcal J_i^F}w_{ij}}.
$$

第一版默认：

$$
w_{ij}=1.
$$

保留可选权重：

$$
w_{ij}=c_j\exp\left(-\frac12D_{ij}^2\right),
$$

其中 $c_j$ 是射线空间语义置信度。

第一版主实验建议使用简单均值，避免复杂权重影响叙事。

---

# 21. 量测协方差

样本协方差：

$$
\mathbf C_i=
\frac{1}{\max(1,N_i^F-1)}
\sum_{j\in\mathcal J_i^F}
(\mathbf p_j-\mathbf z_i)(\mathbf p_j-\mathbf z_i)^\top.
$$

量测协方差：

$$
\mathbf R_i=
\mathbf R_{\min}+
\frac{\lambda_{\mathrm{sparse}}}{\max(1,N_i^F)}\mathbf I+
\beta_C\mathbf C_i.
$$

- $\mathbf R_{\min}$：基础测距和位姿误差；
- $\lambda_{\mathrm{sparse}}$：稀疏量测惩罚；
- $\beta_C$：样本离散程度权重；
- $\mathbf R_i$：当前量测协方差。

只有一个点时令 $\mathbf C_i=0$，主要依赖基础协方差和稀疏惩罚。

---

# 22. 四种轨迹状态

```text
VISIBLE
MAP_OCCLUDED
TARGET_OCCLUDED
UNOBSERVED
```

## 22.1 `VISIBLE`

若：

$$
N_i^F\ge N_{\min}^F,
$$

且射线身份分配可靠，则：

$$
s_i=\mathrm{VISIBLE}.
$$

## 22.2 `MAP_OCCLUDED`

没有足够浮空量测时，检查传感器到预测支撑区域之间是否存在静态地图表面。

## 22.3 `TARGET_OCCLUDED`

没有足够浮空量测时，检查是否存在另一条当前可见轨迹位于前方并与目标投影重叠。

## 22.4 `UNOBSERVED`

若无足够浮空量测、无可靠静态遮挡、无可靠目标间遮挡：

$$
s_i=\mathrm{UNOBSERVED}.
$$

它统一包含未扫描、回波过弱、短时漏检、预测偏差和目标可能离开。

---

# 23. 静态遮挡判断

为了避免重新引入扫描机会，静态遮挡不遍历当前全部发射射线。

从传感器原点向统一预测支撑模型的代表方向做地图查询：

- 预测中心；
- 上、下、左、右；
- 可选四个对角方向。

形成：

$$
\mathcal U_i^{\mathrm{occ}}=\{\mathbf u_{i,1},\dots,\mathbf u_{i,M}\}.
$$

第 $m$ 个方向的预测目标深度：

$$
\rho_{i,m}^{\mathrm{target}}.
$$

地图最近可信占据深度：

$$
\rho_{i,m}^{\mathrm{map}}.
$$

若：

$$
\rho_{i,m}^{\mathrm{map}}<
\rho_{i,m}^{\mathrm{target}}-\epsilon_{\mathrm{occ}},
$$

则该方向被静态地图遮挡。

静态遮挡比例：

$$
q_i^{\mathrm{map}}=
\frac1M\sum_{m=1}^{M}
\mathbf1\left[
\rho_{i,m}^{\mathrm{map}}<
\rho_{i,m}^{\mathrm{target}}-\epsilon_{\mathrm{occ}}
\right].
$$

若：

$$
q_i^{\mathrm{map}}>\tau_{\mathrm{map}},
$$

则：

$$
s_i=\mathrm{MAP\_OCCLUDED}.
$$

该检查只表示几何遮挡解释，不表示当前一定有激光扫描目标。

---

# 24. 目标间遮挡判断

将轨迹 $m$ 和 $i$ 的统一支撑内层轮廓投影到传感器视角，得到：

$$
\Omega_m,\qquad\Omega_i.
$$

角度重叠率：

$$
\eta_{mi}=
\frac{|\Omega_m\cap\Omega_i|}
{\min(|\Omega_m|,|\Omega_i|)}.
$$

若：

$$
\eta_{mi}>\tau_\Omega,
$$

且：

$$
\rho_m<\rho_i-\epsilon_{\mathrm{occ}},
$$

并且前方轨迹当前：

$$
s_m=\mathrm{VISIBLE},
$$

则形成遮挡候选：

$$
m\rightarrow i.
$$

遮挡得分：

$$
\operatorname{score}(m\rightarrow i)=
\eta_{mi}\,c_m^{\mathrm{visible}}\,c_{mi}^{\mathrm{depth}}.
$$

选择得分最高的前方轨迹：

$$
m_i^\star=\arg\max_m\operatorname{score}(m\rightarrow i).
$$

超过阈值后：

$$
s_i=\mathrm{TARGET\_OCCLUDED}.
$$

不能仅凭两个预测中心接近就判定遮挡。

---

# 25. 状态判定顺序

必须互斥：

```text
1. 有可靠浮空量测：
       VISIBLE

2. 否则静态遮挡证据充分：
       MAP_OCCLUDED

3. 否则存在可靠前方可见轨迹：
       TARGET_OCCLUDED

4. 否则：
       UNOBSERVED
```

第一版优先静态遮挡，再判断目标遮挡。不要增加第五个状态。

---

# 26. 状态滞回

维护：

```text
candidate_state
candidate_count
confirmed_state
```

进入新状态要求连续 $N_{\mathrm{enter}}$ 个窗口，退出遮挡要求连续 $N_{\mathrm{exit}}$ 个窗口重新获得量测或遮挡证据消失。

建议：

```yaml
state_enter_frames: 2
state_exit_frames: 2
```

---

# 27. 条件化轨迹更新

## 27.1 `VISIBLE`

创新：

$$
\mathbf y_i=\mathbf z_i-\mathbf H\hat{\mathbf x}_{i,k|k-1},
$$

$$
\mathbf S_i=\mathbf H\mathbf P_{i,k|k-1}\mathbf H^\top+\mathbf R_i,
$$

$$
\mathbf K_i=\mathbf P_{i,k|k-1}\mathbf H^\top\mathbf S_i^{-1},
$$

$$
\hat{\mathbf x}_{i,k|k}=
\hat{\mathbf x}_{i,k|k-1}+\mathbf K_i\mathbf y_i.
$$

## 27.2 `MAP_OCCLUDED`

只预测：

$$
\hat{\mathbf x}_{i,k|k}=\hat{\mathbf x}_{i,k|k-1}.
$$

不使用地图表面点更新目标。

## 27.3 `TARGET_OCCLUDED`

只预测，并：

- 不使用前方目标返回；
- 禁止同一射线重复更新前后轨迹；
- 禁止遮挡组内普通近邻合并；
- 保留原 ID。

## 27.4 `UNOBSERVED`

只预测，并轻度降低轨迹存在分数。

---

# 28. 轨迹存在分数

定义：

$$
e_i\in[0,1].
$$

更新：

$$
e_{i,k}=\operatorname{clip}(e_{i,k-1}+\Delta e(s_i),0,1).
$$

$$
\Delta e(s_i)=
\begin{cases}
+\alpha_{\mathrm{hit}},&s_i=\mathrm{VISIBLE},\\
-\alpha_{\mathrm{map}},&s_i=\mathrm{MAP\_OCCLUDED},\\
-\alpha_{\mathrm{target}},&s_i=\mathrm{TARGET\_OCCLUDED},\\
-\alpha_{\mathrm{unobs}},&s_i=\mathrm{UNOBSERVED}.
\end{cases}
$$

要求：

$$
\alpha_{\mathrm{map}}\ll\alpha_{\mathrm{unobs}},
\qquad
\alpha_{\mathrm{target}}\ll\alpha_{\mathrm{unobs}}.
$$

删除条件：

```text
存在分数低于阈值
且
距离最近一次VISIBLE超过最小时间
且
当前不处于确认遮挡状态
```

因为没有扫描机会，`UNOBSERVED` 惩罚必须保守。

---

# 29. 遮挡期间协方差

没有量测时由过程模型自然增大协方差。

可选：

```yaml
occlusion_process_noise_scale: 1.0
unobserved_process_noise_scale: 1.2
```

第一版：

- 静态遮挡：正常过程噪声；
- 目标遮挡：正常或轻微增大；
- 未观测：适度增大。

不能人为固定后方轨迹位置。

---

# 30. 新目标初始化

未被已有轨迹分配的浮空候选：

$$
\mathcal P_k^{\mathrm{new}}=
\{\mathbf p_j:\ell_j=F,\ j\text{未分配已有轨迹}\}.
$$

使用当前局部体素成分和短时持续性建立 birth buffer。

初始化条件：

1. 浮空语义置信度足够；
2. 未被已有轨迹关联；
3. 当前点数达到阈值，或跨多个窗口持续；
4. 目标尺寸不超过上限；
5. 运动满足最大速度约束；
6. 单点不能在单帧立即初始化正式轨迹。

跨帧可达条件：

$$
\|\mathbf c_b-\mathbf c_a\|\le
v_{\max}(t_b-t_a)+\epsilon_{\mathrm{birth}}.
$$

---

# 31. 地图更新顺序

RS-VoFOD-Core：

```text
历史地图 M_{k-1}
→ 当前返回成分分类
→ 射线语义
→ 轨迹量测和遮挡判断
→ 最后提交地图更新得到 M_k
```

避免动态目标先写入地图，再被自己污染后的地图分类。

---

# 32. 地图更新规则

## 32.1 背景返回 $B$

- 射线路径更新自由；
- 返回端点强化静态占据。

## 32.2 浮空候选返回 $F$

- 射线路径更新自由；
- 返回端点不强化长期静态占据；
- 可写入动态或暂态状态。

## 32.3 未知返回 $U$

- 射线路径保守更新自由；
- 返回端点更新为未知或暂态占据。

## 32.4 无返回射线 $NR$

沿实际发射方向更新到：

$$
r_j^{\mathrm{free}}=
\min(r_{\mathrm{raycast,max}},r_{\mathrm{no-return,reliable}}).
$$

无返回射线只用于地图，不进入：

- 轨迹分配；
- 遮挡状态；
- 轨迹存在分数更新。

基线和主方法使用相同自由更新配置。

---

# 33. 完整逐帧伪代码

```text
Input:
    current RayBundle R_k
    previous static map M_{k-1}
    previous tracks T_{k-1}
    observer pose buffer

Output:
    ray semantics
    target measurements
    track states
    updated tracks T_k
    updated map M_k

1. Validate the ray bundle.
2. Transform emitted rays to the world frame.
3. Predict all existing tracks.
4. Build the unified predictive support model for each track.
5. Extract all valid return endpoints.
6. Build local voxel-connected return components.
7. Classify each component using the previous VoFOD map:
       BACKGROUND / FLOATING_CANDIDATE / UNKNOWN.
8. Propagate each component label back to member return rays.
9. Label no-return rays as NR.
10. For each floating-candidate return ray:
       compute normalized support distance to candidate tracks;
       assign to one track if identity is reliable;
       otherwise leave it unassigned/ambiguous.
11. For each track:
       collect assigned floating returns;
       if support is sufficient:
           set candidate state VISIBLE;
           build measurement and covariance.
       otherwise:
           query static-map occlusion;
           if reliable:
               MAP_OCCLUDED;
           else query visible foreground-track occlusion;
           if reliable:
               TARGET_OCCLUDED;
           else:
               UNOBSERVED.
12. Apply state hysteresis.
13. Update tracks:
       VISIBLE -> Kalman correction;
       MAP_OCCLUDED -> prediction only;
       TARGET_OCCLUDED -> prediction only and protect identity;
       UNOBSERVED -> prediction only with conservative existence decay.
14. Update new-target birth buffers using unassigned floating returns.
15. Initialize confirmed new tracks.
16. Update the occupancy map after semantic and track processing.
17. Publish diagnostics, visualization and profiling.
```

---

# 34. ROS 节点划分

## `mid360_ray_publisher`

- 发布完整实际射线；
- 保留 PointCloud2；
- 输出返回状态和时间。

## `mid360_ray_preprocessor`

- 射线校验；
- 时间和 TF；
- 世界坐标转换；
- 有效点过滤；
- self-mask。

## `rs_vofod_semantic_node`

- 返回端点体素化；
- 局部成分；
- VoFOD 式空间语义；
- 语义回写；
- 延迟地图更新。

## `rs_vofod_tracker_node`

- 轨迹预测；
- 统一预测支撑模型；
- 浮空射线分配；
- 量测构造；
- 静态遮挡；
- 目标遮挡；
- 四状态管理；
- 条件化更新；
- 新目标 birth。

## `rs_vofod_evaluator`

- 真值；
- 指标；
- 不向算法反馈真值。

---

# 35. 参数文件

创建 `config/rs_vofod_core.yaml`：

```yaml
ray:
  min_range: 0.3
  max_range: 30.0
  no_return_reliable_range: 20.0
  geometry_mode: snapshot

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

support_model:
  shape_sigma_x: ...
  shape_sigma_y: ...
  shape_sigma_z: ...
  prediction_scale: ...
  maximum_support_axis_x: ...
  maximum_support_axis_y: ...
  maximum_support_axis_z: ...
  surface_sigma_x: ...
  surface_sigma_y: ...
  surface_sigma_z: ...
  ego_sigma_x: ...
  ego_sigma_y: ...
  ego_sigma_z: ...
  association_threshold: ...
  occlusion_threshold: ...
  ambiguity_margin: ...

measurement:
  min_floating_returns: 1
  minimum_covariance: ...
  sparse_covariance_scale: ...
  sample_covariance_scale: ...

map_occlusion:
  sample_directions: 9
  depth_margin: ...
  occlusion_ratio_threshold: ...

target_occlusion:
  angular_overlap_threshold: ...
  depth_margin: ...
  confidence_threshold: ...

state:
  enter_frames: 2
  exit_frames: 2

existence:
  hit_increment: ...
  map_occlusion_decrement: ...
  target_occlusion_decrement: ...
  unobserved_decrement: ...
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

---

# 36. 关键风险与处理

## 36.1 单射线不能独立证明浮空

用局部返回成分提供空间上下文；每个成分只执行一次 VoFOD BFS；射线继承成分语义。

## 36.2 一个返回成分包含两个目标

成分只提供 `FLOATING_CANDIDATE`，不作为实例；后续按统一支撑模型把射线分别分配给轨迹。

## 36.3 轨迹预测自我确认

只有已被地图判为 $F$ 的端点可用于轨迹更新；轨迹不能修改 $B/F/U$；歧义射线不用于量测。

## 36.4 统一支撑模型过大

设置预测膨胀上限；遮挡使用内层阈值，关联使用外层阈值；输出每条轨迹支撑半轴。

## 36.5 统一支撑模型过小

包含表面偏差和 observer 误差；关联使用外层阈值；遮挡后允许有限增大。

## 36.6 表面点不是目标质心

基线与主方法使用相同几何中心策略；稀疏时增大量测协方差；第一版不宣称解决质心精确恢复。

## 36.7 无扫描机会后无法区分未扫描和漏检

`UNOBSERVED` 轻惩罚；删除需要最小时间；遮挡状态不删除；在限制中明确。

## 36.8 静态遮挡误判

多个代表方向投票；设置深度裕量；使用历史地图；浮空端点不强化静态占据；状态滞回。

## 36.9 目标间遮挡误判

要求前方轨迹当前 `VISIBLE`、角度重叠、明确深度顺序、使用内层支撑轮廓和状态滞回。

## 36.10 同一射线被多个轨迹使用

唯一分配；最优与次优差异不足则标记歧义；不允许复制。

## 36.11 完全遮挡期间不可观测

只预测；协方差自然增大；保留 ID；不声称恢复遮挡期间真实运动。

## 36.12 新目标孤立噪声

单点需跨窗口确认；使用最大速度约束和存在分数。

## 36.13 当前点污染地图

使用 $\mathcal M_{k-1}$ 分类，完成语义和轨迹处理后更新 $\mathcal M_k$。

## 36.14 仿真与实机差异

本核心版无返回方向只服务地图；若实机无法得到完整无返回方向，VoFOD自由空间会退化，必须明确硬件边界。

## 36.15 运动畸变

用静态墙验证；提供 `snapshot/per_ray_pose`；未验证前默认 snapshot。

## 36.16 计算量

同一体素只查询一次；每成分只做一次 BFS；轨迹先做粗角度筛选；静态遮挡只查 5 或 9 个方向；输出 P50/P95。

## 36.17 基线不公平

至少比较：

```text
B0:
VoFOD-Mid360 + 原始风格tracker

B1:
VoFOD-Mid360点簇检测
+ 与P相同的CV/KF和轨迹管理
+ 不使用射线级量测和遮挡状态

P:
RS-VoFOD-Core
+ 相同CV/KF
+ 射线级量测
+ 静态/目标遮挡条件化更新
```

论文核心结论优先比较 B1 与 P。

---

# 37. 最小实验设计

所有方法使用同一 rosbag 回放。

## S1：稀疏回波单目标

### 目的

验证单点和双点返回能否更新已有轨迹、是否比点簇级 VoFOD 减少漏检、射线语义在背景附近是否稳定。

### 场景

```text
observer：静止悬停
background：一面后墙、两根柱体
target：一架无人机，悬停后横向运动，距离逐渐增加
```

覆盖多点、双点、单点、短时无返回和靠近墙边。

### 指标

- 可见时检测 recall；
- false positive；
- 单点/双点窗口轨迹更新率；
- 初始化延迟；
- 位置 RMSE；
- 轨迹连续率；
- runtime。

### 预期

已有轨迹阶段，即使点数不足以满足 VoFOD 点簇阈值，RS-VoFOD-Core 仍能形成量测。

---

## S2：两目标交叉遮挡

observer：

$$
\mathbf p_O=(0,0,4).
$$

目标 A：

$$
(8,-3,4)\rightarrow(8,3,4).
$$

目标 B：

$$
(12,3,4)\rightarrow(12,-3,4).
$$

A 位于前方，B 位于后方。

子情况：

```text
S2a：部分遮挡，两个目标都有少量返回
S2b：短时完全遮挡，后方目标没有返回
```

### 指标

- ID switches；
- fragmentation；
- 后方轨迹存活率；
- `TARGET_OCCLUDED` precision/recall；
- 遮挡起止延迟；
- 重新分离 ID 正确率；
- 同一量测错误更新多轨迹次数；
- 遮挡期间位置 RMSE。

### 预期

不把一个成分质心同时更新两条轨迹；前方射线唯一分配；后方进入 `TARGET_OCCLUDED`；减少 ID switch。

---

## S3：静态墙遮挡

```text
observer：静止悬停
background：一面窄墙、一个门框
target：正常可见 → 飞到墙后 → 保持 → 再飞出
```

### 指标

- `MAP_OCCLUDED` precision/recall；
- 墙后轨迹存活率；
- 错误墙面更新次数；
- 重新捕获延迟；
- 位置 RMSE；
- 错误删除次数。

### 预期

墙后只预测，不使用静态墙返回更新目标。

---

# 38. 对比方法和消融

## B0

```text
VoFOD-Mid360点簇检测 + 原始风格tracker
```

## B1

```text
VoFOD-Mid360点簇检测
+ 与P完全相同的CV/KF
+ 相同轨迹初始化、存在分数和删除参数
+ 无射线级量测
+ 无遮挡条件化更新
```

## P

```text
射线端点空间语义
+ 统一预测支撑模型
+ 浮空射线直接量测
+ 静态/目标遮挡
+ 四状态条件化更新
```

## P-no-occ

```text
射线级量测
+ 不使用静态和目标遮挡
+ 无量测全部按UNOBSERVED处理
```

## P-cluster-measurement

```text
射线语义
+ 仍使用成分质心作为量测
+ 不进行轨迹条件化射线分配
```

---

# 39. 实验运行方式

1. 先运行仿真，只记录原始射线、原始点云、TF、observer 状态、目标真值和遮挡真值；
2. 用同一 rosbag 回放 B0、B1、P 和两个消融；
3. 所有方法使用相同地图参数、支撑尺寸、KF 过程噪声、初始化和删除参数；
4. 在线运行只用于实时性测试；
5. 每个场景建议 5 次可重复运行，随机变化仅包括目标起始相位、小幅速度扰动和点云噪声种子。

---

# 40. 真值与指标

评估节点可读取 Gazebo model states、目标 collision 和静态模型 collision，但感知节点不得订阅真值。

输出：

```text
target_id
true_position
true_velocity
true_map_occluded
true_target_occluded
true_occluder_id
```

指标：

- 检测：precision、recall、localization RMSE、单点/双点更新率、背景边缘误检；
- 跟踪：IDF1、ID switches、fragmentation、track recall、后方轨迹存活率、重捕获延迟、位置/速度 RMSE；
- 遮挡：两类遮挡 precision/recall、遮挡者 ID、状态延迟、错误墙面更新、错误前方量测更新后方轨迹；
- 计算：各模块耗时、总帧 P50/P95/max、CPU、内存和消息带宽。

---

# 41. RViz 可视化

至少显示：

- 背景一致、浮空候选和未知端点；
- 无返回射线抽样；
- 局部返回成分；
- 统一预测支撑内层和外层；
- 分配给各轨迹的浮空射线；
- 歧义射线；
- 轨迹 ID 和四状态；
- 静态遮挡代表射线；
- 目标间遮挡边；
- 当前轨迹量测；
- 新目标 birth buffer。

---

# 42. 测试要求

单元测试：

```text
ray_direction_normalization_test
ray_timestamp_monotonic_test
component_connectivity_test
semantic_label_inheritance_test
support_covariance_test
inner_outer_support_threshold_test
unique_ray_assignment_test
ambiguous_ray_rejection_test
measurement_covariance_sparse_test
map_occlusion_vote_test
target_occlusion_logic_test
track_state_transition_test
existence_score_update_test
map_update_semantic_test
```

集成测试：

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

# 43. 分阶段实现

## Phase 0：工程审计

输出 `WORKSPACE_AUDIT.md`，不得修改工程。

## Phase 1：完整射线接口

创建 `mid360_ray_msgs`，修改插件发布 `RayBundle`，保留 PointCloud2，完成空世界和单墙测试。

## Phase 2：VoFOD-Mid360 基线

移除 Ouster 固定组织点云假设；有返回和无返回更新自由空间；跑通点簇级 VoFOD 和 B0。

## Phase 3：射线语义

返回端点体素成分、复用 VoFOD 分类、输出 $B/F/U/NR$、延迟地图更新。

## Phase 4：统一支撑和射线量测

CV 预测、支撑协方差、内外层阈值、浮空射线唯一分配、量测和协方差。

## Phase 5：静态和目标遮挡

地图代表方向查询、目标投影重叠、四状态、滞回和条件化更新。

## Phase 6：新目标初始化

未分配浮空候选、birth buffer、新轨迹确认。

## Phase 7：实验

三个场景、B0/B1/P、两个消融、统一 rosbag 回放和结果汇总。

---

# 44. 每阶段验收格式

每阶段完成后输出：

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

- 未编译继续下一阶段；
- 隐藏错误；
- 修改系统 MRS 安装目录；
- 删除原插件输出；
- 算法节点订阅真值；
- 用轨迹预测修改 B/F/U 语义；
- 将同一射线更新多条轨迹；
- 把 `UNOBSERVED` 解释成确定目标消失；
- 擅自重新加入扫描机会模块。

---

# 45. 必须生成的文档

```text
WORKSPACE_AUDIT.md
RAY_INTERFACE.md
VOFOD_MID360_BASELINE.md
RS_VOFOD_CORE_ALGORITHM.md
SYMBOLS_AND_FORMULAS.md
UNIFIED_SUPPORT_MODEL.md
OCCLUSION_LOGIC.md
RISK_REGISTER.md
EXPERIMENT_SCENARIOS.md
RUNBOOK.md
TEST_REPORT.md
KNOWN_LIMITATIONS.md
```

`KNOWN_LIMITATIONS.md` 必须明确：

1. 无法区分未扫描与真实漏检；
2. `UNOBSERVED` 是混合状态；
3. 完全遮挡期间真实运动不可观测；
4. 表面点中心不等于真实质心；
5. 返回语义仍依赖局部成分；
6. 新目标仍需轻量聚合；
7. 实机无返回射线方向能力需要验证；
8. 仿真与真实反射特性存在差异。

---

# 46. Codex 首轮只做的任务

首次执行只完成：

1. 审计当前工程；
2. 输出 `WORKSPACE_AUDIT.md`；
3. 创建 `mid360_ray_msgs`；
4. 修改仿真插件发布完整 `RayBundle`；
5. 保留原 PointCloud2；
6. 创建空世界和单墙测试；
7. 创建射线 validator；
8. 输出 `RAY_INTERFACE.md`；
9. 编译并运行测试；
10. 给出 Phase 2 的准确入口。

首轮不要实现完整 RS-VoFOD-Core。

---

# 47. 最终算法一句话概括

> RS-VoFOD-Core 首先利用历史占据地图和局部返回端点连通关系，为每条有返回射线赋予背景一致、浮空候选或未知语义，并利用无返回射线维持 VoFOD 自由空间地图；随后通过融合目标尺寸、预测不确定性、表面偏差和 observer 误差的统一预测支撑模型，将浮空候选射线直接聚合为已有轨迹量测；对于没有量测的轨迹，仅判断其是否被静态地图或其他当前可见目标遮挡，最后按照可见、静态遮挡、目标遮挡或未观测状态执行条件化轨迹更新。
