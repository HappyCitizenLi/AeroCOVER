# Codex 实现提示词：Mid360 适配 VoFOD 基线修正 + SOFT-VoFOD 精简算法 + 仿真评测完善

## 0. 角色与总目标

你正在 `/home/uav/lyk` 工作区中继续开发一个 ROS1 Noetic / catkin 工程。研究目标是：

> observer UAV 搭载 Livox Mid-360，在未知或半未知环境中，利用完整已发射射线（包括无返回射线）对多个非合作 UAV 进行在线 3D 检测与跟踪。

当前已经存在一条可运行的 B0 感知链：

```text
Mid360 原始点云 + 完整已发射射线
  -> mid360_ray_preprocessor
  -> vofod_mid360
  -> lidar_tracker_mid360
```

你的任务不是推翻当前工程，而是：

1. **先审计并修正当前公共基础设施和 B0 基线中的明确问题**；
2. **冻结一个可复现、可公开说明的 `VoFOD-Mid360 B0` baseline**；
3. 在不污染 B0 的前提下，新建并实现精简版算法 **SOFT-VoFOD**：
   - conservative background occupancy memory；
   - free-space violation events；
   - trajectory-before-confirmation birth；
   - scan-opportunity-aware track existence；
   - 简单、稳定的多目标 GNN/Hungarian + KF 后端；
4. 完善仿真输入、ground truth、评测 runner、指标、消融实验与结果导出；
5. 所有新增算法必须严格禁止 truth leak、场景特判和按场景调参。

本任务只研究感知/多目标跟踪。当前轻量 Gazebo 场景通过 `/gazebo/set_model_state` 写入参考位姿，可用于确定性感知实验，但**不得声称已经验证 MRS/PX4 闭环、气动、避障或飞控性能**。

---

# 1. 开始前必须做的源码审计

首先阅读并核对：

```text
/home/uav/lyk/CURRENT_MRS_VOFOD_MID360_IMPLEMENTATION.md

src/mid360_simulation_plugin_fork/UPSTREAM.md
src/vofod_mid360/UPSTREAM.md
src/lidar_tracker_mid360/UPSTREAM.md
src/livox_ros_driver2_rayfork/UPSTREAM.md
```

然后检查实际源码、launch、YAML、msg 定义是否与总结文档一致。

必须输出一份：

```text
docs/PRE_IMPLEMENTATION_AUDIT.md
```

至少记录：

- 当前 git status；
- 所有相关包实际路径；
- 上游 commit；
- 当前 launch/YAML 的真实参数；
- 当前 topic graph；
- `points_raw / points_valid / rays_raw / rays_checked` 的 frame、stamp、scan_id、original_index 关系；
- B0 detection 输入与 tracker 输入是否使用同一几何；
- 当前 map state、background output 的真实定义；
- 当前 tracker 的 Q/P0/Z、background filter、downsample、cluster 参数；
- 所有发现的“文档与代码不一致”。

**不要先写新算法再审计。**

---

# 2. 不可破坏的工程约束

## 2.1 不修改系统安装

禁止修改：

```text
/opt/ros/noetic
/home/uav/mrs_mid360_ws
```

禁止覆盖系统 MRS 安装。

继续使用 `/home/uav/lyk` 的独立 catkin overlay。

## 2.2 保留 B0

`vofod_mid360` 与 `lidar_tracker_mid360` 必须继续作为 baseline。

允许：

- 修复确定性 bug；
- 修正公共输入时序/几何；
- 新增诊断；
- 新增独立 canonical B0 launch/YAML。

不允许：

- 把 SOFT-VoFOD 的 scan opportunity、birth buffer、target quarantine、existence score 等逻辑偷偷加入 B0；
- 按 S01/S02/... 场景名称写 if/else；
- 读取 ground truth；
- 根据目标初始位置、墙坐标、目标 ID 或场景脚本改变 B0 判据。

在完成基础修正后，生成：

```text
docs/B0_FROZEN_SPEC.md
```

并建议创建一个 git tag，例如：

```text
b0-mid360-frozen
```

不要自动 push。

---

# 3. 第一阶段：修正当前已实现部分

以下修正优先级高于新算法。

---

## 3.1 修正点云与完整射线的配对合同

当前预处理器采用 ApproximateTime 对齐原始点云和 `RayBundle`，随后把输出重写成相同时间戳供下游 ExactTime 消费。

这会造成一个潜在问题：

> 下游“ExactTime”只证明两个派生消息 stamp 相同，并不能证明它们原本确实来自同一个 scan。

必须建立**显式 scan identity 合同**。

### 要求

以 `RayBundle.scan_id` 为主键，并同时检查：

- source header stamp；
- `pattern_start_index`；
- bundle 内射线数；
- 每个 `VALID_RETURN` 的 `original_index`；
- `points_raw -> points_valid` 保留下来的 source order；
- 一个 `VALID_RETURN` 最多对应一个有效点；
- 一个有效点必须能对应一个合法 `VALID_RETURN`；
- 点的 range 与 `Ray.range` 在容差内一致；
- 不允许 duplicate `original_index`；
- 不允许跨 scan 配对。

若原始点云无法直接携带 `scan_id`，可以在仿真插件和硬件 fork 中增加一个轻量 companion metadata，但必须保持 legacy point cloud ABI 可选兼容。

### 不要做

不要仅靠：

```text
ApproximateTime -> restamp -> ExactTime
```

来宣称精确匹配。

### 新诊断

增加：

- `scan_pair_ok`
- `scan_id`
- `point_source_stamp`
- `ray_source_stamp`
- `source_stamp_skew_ns`
- `valid_return_count`
- `matched_point_count`
- `duplicate_index_count`
- `missing_valid_return_count`

任何核心合同失败时 fail closed。

---

## 3.2 统一 detector 与 tracker 的几何：新增逐点 deskew 世界点云

当前问题：

- `rays_checked` 可包含每束射线时刻的世界系 origin/direction；
- `points_valid` 本身仍是原始 sensor-frame 点；
- `vofod_mid360` 可以从 checked ray 重建世界端点；
- `lidar_tracker_mid360` 仍可能把整束点云按 cloud header 使用一次 TF。

这会导致：

> detector 与 tracker 对同一物理返回使用不同的世界几何。

这是当前最优先修正项。

### 新增输出

在 `mid360_ray_preprocessor` 中新增：

```text
/uav1/mid360/points_world
```

类型优先使用标准 `sensor_msgs/PointCloud2`，至少包含：

- x, y, z：世界坐标；
- intensity；
- original_index；
- offset_time_ns（若方便）；
- scan_id（可通过 companion/header 维护）。

对每个 `VALID_RETURN`：

$$
p_j^W=o_j^W+r_j u_j^W
$$

必须直接使用 `CheckedRay.origin + range * CheckedRay.direction` 构造世界点。

这样：

```text
points_world
```

与：

```text
rays_checked
```

严格来自同一几何。

### 修改 B0 tracker

让 canonical B0 tracker 使用 `points_world`，不要再对整束点云做一次 world TF。

保留旧输入模式作为 legacy option，但主实验不得使用。

### 验证测试

构造 observer 快速平移 + yaw 的单墙场景，验证：

- `points_world` 与 checked-ray endpoint 误差；
- snapshot / per_ray_pose 两种模式；
- detector 查询点、tracker 点完全一致；
- 不允许同一 raw return 在两个模块产生 >1 cm 的纯软件几何差异（仿真无噪声条件）。

---

## 3.3 不要把当前 `per_ray_pose` 误称为完整 rolling-scan 仿真

当前插件的 `per_ray_pose` 只补偿 observer 几何，并且当前 collision scene 在 bundle 内仍有限制。

因此必须明确区分：

### Tier A：算法开发/主消融

可继续使用确定性 sensor snapshot 或当前受控模式，但所有算法使用相同输入 bag。

### Tier B：rolling-scan stress test

后续实现一个可选的更真实模式，使至少 observer 与目标的 pose 都随 ray time 更新。

优先选择**微批次 rolling scan**而非 20,000 次 Gazebo 阻塞查询，例如：

- 1–5 ms 一个 geometry chunk；
- chunk 内使用一次 scene pose；
- ray 保留原 `offset_time_ns`；
- observer/targets 按真值轨迹插值到 chunk time；
- 静态墙体保持静态。

如果当前 Gazebo RaySensor 架构不适合，先写设计文档：

```text
docs/ROLLING_SCAN_LIMITATION.md
```

不要为了完成本任务而引入不可维护的大型自定义 physics engine。

主论文实验必须明确标注 Tier A/Tier B。

---

## 3.4 B0 启动背景：降低 `nadir_seed` 对 baseline 的歧义

当前 `nadir_seed` 是替代原 VoFOD 高度测距/先验地图的代理，而且一条合格向下射线可以播种整个当前 component。

问题：

- 它不是原高度测距仪的等价实现；
- 它可能把与地面/背景连接的目标或临时物体一起播种；
- 对多目标起飞场景尤其敏感。

### 修正策略

实现两种独立、明确的 B0 初始化方式：

#### 方式 A：background warm-up（主仿真实验默认）

场景分两阶段：

```text
[0, T_warmup): 只有 observer + 静态环境，无 target
[T_warmup, ...): target 出现并执行轨迹
```

要求：

- warm-up 阶段不向算法提供任何 truth；
- 只是传感器真实扫描背景；
- 所有方法使用完全相同的 warm-up bag；
- 推荐 `T_warmup` 可配置，例如 5–15 s；
- 只有达到 map maturity 条件后才进入正式评测。

#### 方式 B：apriori map（可选）

如果上游已有 PCD loader，移植为可选功能。

#### `nadir_seed`

保留，但：

- 默认不用于主对比；
- 单独做 startup ablation；
- 明确写进 baseline 文档。

---

## 3.5 B0 参数审计，不允许把大幅调参隐藏成“接口适配”

当前至少需要审计：

- voxel size 当前文档为 `0.5 m`，而原论文实验参数为另一取值；
- `cluster tolerance = 1.25 m`；
- background coverage ratio = `0.01`；
- tracker radius min = `0.75 m`；
- tracker Q/P0；
- fixed measurement covariance；
- local downsample；
- local clustering tolerance；
- max OBB。

### 要求

把所有参数分成三类：

```text
A. 纯接口必要参数
B. Mid360 全局标定参数
C. 算法语义参数
```

建立：

```text
config/b0_mid360_canonical.yaml
docs/B0_PARAMETER_PROVENANCE.md
```

主实验只能使用一个 canonical 参数集。

参数选择必须来自：

- 独立 calibration scenes；
- 或统一 sensitivity sweep。

禁止：

- S03 用一套；
- S04 又换一套；
- 测试结果不好再逐场景调参。

### clustering 特别要求

由于单尺度 Euclidean clustering 存在：

- d 太小 -> 一个 UAV 分裂；
- d 太大 -> 两个 UAV 合并；

必须增加 sensor-specific sensitivity 测试：

```text
cluster_tolerance ∈ {0.3, 0.5, 0.75, 1.0, 1.25, 1.5} m
```

至少统计：

- single-target split rate；
- two-target merge rate；
- 随 range 和 inter-target separation 的变化。

此实验用于说明 B0 的结构性限制，不是为了给 proposed method 挑最有利参数。

---

## 3.6 B0 tracker 的 background filter 必须统一

原 VoFOD tracker 会利用 occupancy map 的 occupied background 过滤局部点。

当前：

- standalone tracker 默认 background filter 关闭；
- evaluation launch 又显式开启。

这会造成“生产 B0”和“评测 B0”不是同一个算法。

### 修正

新增唯一 canonical launch：

```text
tclv_evaluation/b0_canonical.launch
```

明确：

```text
background_filter_enabled := true
background source := vofod_mid360/background_points
```

必须保证不会把 `free_voxels` 误接到 background KD-tree。

standalone launch 可以保留，但文档标明不是论文 baseline。

---

## 3.7 B0 tracker 参数与滤波模型审计

当前 9-state CA/LKF 可作为 B0 继续保留，但要检查：

- P0 acceleration 是否过小；
- Q 是否与真实 update rate 匹配；
- fixed measurement variance 的单位是 variance 还是 standard deviation；
- downsample 0.5 m 对 singleton target 是否有副作用；
- local cluster 是否允许 1-point cluster；
- duplicate merge 是否会删除两个真实近邻 target。

### 原则

B0 不需要升级成新 tracker；其缺点正是 baseline 的一部分。

但：

- 明显参数/单位错误要修；
- 参数必须冻结；
- 主实验中记录其 failure mode。

新增单元测试：

1. 单点 target track correction；
2. 两个相距 0.5/1.0/2.0 m 的 track；
3. crossing tracks；
4. prolonged no-measurement；
5. duplicate merge 不应在明显两个 truth-free synthetic track 上产生 iterator/NaN 问题。

---

## 3.8 NO_RETURN 自由更新必须区分 baseline 与 proposed method

### B0

保留当前“VoFOD-style no-return free ray”作为 baseline，但：

- 明确 `max_free_range`；
- 明确权重；
- 做 sensitivity；
- 不声称这是目标存在概率模型。

### SOFT-VoFOD

不能把 no-return 射线无条件 carving 穿过预测目标。

若 ray 与 tentative/confirmed track 的 target support 相交：

- 只允许 free 更新到 target support 近端之前；
- target support 及其后方暂不 free carving。

这样避免长期悬停/低反射目标被地图慢慢“挖掉”。

---

# 4. B0 baseline 的最终定义

冻结后的 baseline 名称统一为：

```text
B0: VoFOD-Mid360 + classic lidar_tracker
```

不要在论文/README 中称为“原始 VoFOD 完全复现”。

准确表述：

> B0 保留 VoFOD 的 occupancy-state、cluster classification、raycasting、separated-background removal 和原始风格 point-cloud-aided tracker，并对 Mid-360 的非 Ouster 接口、完整真实射线、时间戳、点类型和安全事务进行了必要适配，同时使用固定的 Mid360 全局标定参数。

B0 不包含：

- free-space violation event birth；
- conservative-background / target decomposition；
- scan-opportunity track state；
- target quarantine；
- existence probability；
- GNN/Hungarian（若 classic tracker 仍是 greedy）；
- truth information。

---

# 5. 第二阶段：实现 SOFT-VoFOD 精简版最终算法

算法名称：

```text
SOFT-VoFOD
Scan-Opportunity-aware Free-space Trajectory VoFOD
```

核心思想必须保持简单：

> 历史稳定背景由保守 occupancy memory 表示；飞行目标不再作为地图的一类 occupancy state，而表示为“历史自由空间违例事件形成的时空轨迹”。真实发射射线用于区分“没有扫描机会”和“有扫描机会但没有目标回波”。

整体链路：

```text
points_world + rays_checked
        |
        v
Conservative Background Map
        |
        +---- stable background / confident free / unknown / candidate background
        |
        v
Free-space Violation Events
        |
        v
Trajectory-before-confirmation Birth
        |
        v
GNN/Hungarian + CV-KF Multi-target Tracking
        |
        v
Scan-opportunity-aware Existence Update
        |
        +---- target quarantine / ray truncation feedback
        |
        v
Multi-UAV 3D Tracks
```

不要实现完整 PMBM、MHT、JPDA、神经网络、双时间尺度地图或 shadow-only birth。

---

# 6. 新包建议

优先保持包数量少：

```text
soft_vofod_mid360
soft_vofod_evaluation
```

继续复用：

```text
mid360_ray_msgs
mid360_ray_preprocessor
mid360_multi_uav_sim
```

若消息很多，再建立：

```text
soft_vofod_msgs
```

不要复制整份 B0 源码后大改；公共 DDA/ray utility 可以抽成共享 library，但必须保证 B0 行为通过 regression test 不变。

---

# 7. Conservative Background Map

## 7.1 表示原则

地图 `M^B` 只表示背景，不表示 target。

每个 voxel 至少保存：

```cpp
struct BackgroundVoxel {
  float free_evidence;
  float bg_evidence;

  uint32_t candidate_hits;
  ros::Time candidate_first_time;
  ros::Time candidate_last_time;

  ros::Time last_update;
  uint8_t state;
};
```

推荐状态：

```text
UNKNOWN
CONFIDENT_FREE
CANDIDATE_BACKGROUND
STABLE_BACKGROUND
```

不要复用 VoFOD 的“tentative occupied = 可能动态目标”语义。

---

## 7.2 自由与背景概率

定义：

$$
P_F(v)=\frac{F_v}{F_v+O_v+\epsilon}
$$

$$
P_B(v)=\frac{O_v}{F_v+O_v+\epsilon}
$$

观测置信度：

$$
C(v)=1-\exp\left(-\frac{F_v+O_v}{N_0}\right)
$$

建议状态：

```text
UNKNOWN:
  C < tau_C 且 candidate 未成熟

CONFIDENT_FREE:
  C >= tau_C
  P_F >= tau_F

STABLE_BACKGROUND:
  C >= tau_C
  P_B >= tau_B
  并满足 candidate persistence

CANDIDATE_BACKGROUND:
  新返回位于 UNKNOWN 中，但还未达到稳定背景条件
```

所有阈值配置化。

---

## 7.3 地图更新顺序

每个 micro-batch 必须严格：

```text
read old map
 -> classify returns/events
 -> update tracks
 -> determine quarantine/truncation
 -> commit map updates
```

禁止：

```text
先把当前 endpoint 写入 map
再用新 map 判断自己是不是 anomaly
```

---

## 7.4 VALID_RETURN free carving

对 ray：

```text
origin -> endpoint
```

free 更新到：

```text
range - endpoint_guard
```

如果 ray 在更近处先与某个 tentative/confirmed target support 相交，则最多更新到：

```text
target_near_range - target_guard
```

---

## 7.5 NO_RETURN free carving

NO_RETURN 只在真实合法 `rays_checked` 中使用。

使用：

```text
w_no_return <= w_valid_free
```

和配置化：

```text
max_no_return_free_range
```

若穿过 target support：

```text
truncate before target
```

不允许穿过 target support 一直更新成 free。

---

## 7.6 endpoint 更新

### 当前点属于 stable background

正常增加 `bg_evidence`。

### 当前点是 free-space violation event

```text
bg_evidence += 0
```

并进入 quarantine。

### 当前点与 tentative/confirmed target 关联

```text
bg_evidence += 0
```

并进入 quarantine。

### 当前点在 UNKNOWN 中，未被 target 解释

进入 `CANDIDATE_BACKGROUND`。

只有满足：

- 至少 `N_bg_promote` 个独立时间组；
- persistence >= `T_bg_promote`；
- 空间位置稳定；
- 从未被 target track 关联；
- 不处于 confident-free violation；

才提升为 `STABLE_BACKGROUND`。

---

# 8. Free-space Violation Event

对每个 valid world point：

```text
p_j
t_j
original_index
scan_id
ray direction
```

查询旧背景图。

若：

```text
voxel is CONFIDENT_FREE
AND free confidence >= threshold
AND not near STABLE_BACKGROUND
```

则生成：

```text
FreeSpaceViolationEvent
```

建议字段：

```text
Header header
uint64 scan_id
uint32 original_index
geometry_msgs/Point position
geometry_msgs/Vector3 ray_direction
float32 free_confidence
float32 background_distance
float32 anomaly_score
```

异常分数可简化为：

$$
a_j =
C(v_j)\,
P_F(v_j)\,
w_d(d_{\rm bg})
$$

例如：

$$
w_d=\min(1,d_{\rm bg}/d_0)
$$

或 sigmoid。

不要用语义 classifier。

---

# 9. 允许悬停目标

设计必须保证：

> target 一旦出现在已经成熟的 confident-free 空间，即使之后长期悬停，也持续产生 free-space violation，而不会逐渐被写入 stable background。

实现上依靠：

- target/event endpoint 不增加 bg evidence；
- track quarantine；
- no-return ray 在 target support 前截断。

增加专门 regression test：

```text
pre-map empty air
 -> spawn UAV
 -> UAV hover 20–60 s
 -> map voxel must not become STABLE_BACKGROUND
 -> track must remain available when scan opportunity intermittent
```

---

# 10. Trajectory-before-confirmation Birth

单个 anomaly point **不能直接成为 confirmed track**。

维护：

```text
birth_buffer_duration = T_birth
```

其中保存最近的未关联 free-space violation events。

---

## 10.1 independent event group

同一极短时间内的相邻 rays 高度相关。

按：

```text
event_group_dt
```

分组。

轨迹确认时至少需要来自不同 event group 的证据。

---

## 10.2 候选轨迹模型

第一版只用 3D constant velocity：

$$
x=
[p_x,p_y,p_z,v_x,v_y,v_z]^T
$$

对事件位置 `z_n`：

$$
z_n = H_n \theta + \epsilon_n
$$

其中：

$$
H_n=[I_3,\Delta t_n I_3]
$$

用 weighted least squares 拟合：

$$
\hat\theta=
\left(\sum H_n^T R_n^{-1}H_n\right)^{-1}
\left(\sum H_n^T R_n^{-1}z_n\right)
$$

测量 covariance：

$$
R_n = R_{\rm sensor,n}+\sigma_{\rm shape}^2 I
$$

若暂时没有 observer pose covariance，可使用配置 fallback，但接口必须预留。

---

## 10.3 birth gating

候选事件之间必须满足：

```text
dt_min <= dt <= dt_max
speed <= v_max
optional acceleration reachability
```

不要仅用 Euclidean distance 固定阈值。

RANSAC / graph tracklet 二选一：

### 推荐第一版：RANSAC-CV

- 从两个不同 event group 采样；
- 得到 CV hypothesis；
- 对窗口内事件做 Mahalanobis residual；
- 统计 inlier groups；
- weighted refit；
- 计算 residual RMS。

至少满足：

```text
min_birth_groups >= 3
min_birth_duration
max_birth_speed
max_birth_residual
min_total_anomaly_score
```

后生成 `tentative track`。

这样静止/悬停目标也可 birth：

```text
v ≈ 0
```

---

# 11. SOFT-VoFOD Tracker：CV-KF

不要沿用新的 9-state CA 作为第一版。

使用 6-state CV：

$$
x=[p^T,v^T]^T
$$

$$
F(\Delta t)=
\begin{bmatrix}
I & \Delta t I\\
0 & I
\end{bmatrix}
$$

使用 white-acceleration process：

$$
Q(\Delta t)=
\sigma_a^2
\begin{bmatrix}
\frac{\Delta t^4}{4}I & \frac{\Delta t^3}{2}I\\
\frac{\Delta t^3}{2}I & \Delta t^2 I
\end{bmatrix}
$$

注意：

> Q 必须随真实 Δt 缩放，不能把固定 diagonal Q 在不同 update rate 下直接复用。

测量模型：

$$
z=Hx+\nu,\quad H=[I,0]
$$

measurement covariance：

```text
sensor covariance + shape inflation
```

---

# 12. Existing track 的 measurement 来源

必须区分：

### Birth

只允许：

```text
high-confidence free-space violation events
```

### Track maintenance

允许使用：

```text
预测 gate 内、且不是 stable-background-consistent 的 VALID_RETURN
```

原因：

> target 一旦进入 UNKNOWN 或靠近背景，不能因为 anomaly score 下降就立即失去轨迹。

---

# 13. 微批次处理

不要真的为 200 k rays/s 建立 200 k ROS callbacks。

在一个 `RayBundle` 内按 `offset_time_ns` 划分：

```text
micro_batch_dt = 5–10 ms
```

每个 micro-batch：

1. track predict 到 batch/ray time；
2. 查询 map；
3. event extraction；
4. measurement association；
5. opportunity accumulate；
6. existence update；
7. map commit。

每个 event 保留真实 ray timestamp。

输出 track rate 可以低于 micro-batch rate，但内部状态必须按真实 Δt。

---

# 14. Measurement association：先用 GNN/Hungarian，不上 JPDA

本项目核心创新不是 data association。

第一版使用：

```text
Mahalanobis gating + Hungarian
```

cost：

$$
C_{ij}=d^2_{M,ij}+\lambda_a(1-a_j)
$$

其中：

- `d_M`：track prediction 与 candidate measurement 的 Mahalanobis distance；
- `a_j`：anomaly score，可选弱权重；
- 超出 gate 的 cost = INF。

### singleton 必须可用

不要要求 target measurement 至少 2/3 个点。

### 多点 packet

若一个 micro-batch 中同一 track 周围存在多个点：

1. Hungarian 先给 track 一个 anchor；
2. 再把 anchor 周围 `R_target` 内、且不属于其他已分配 track 的点聚合；
3. robust centroid / median 作为 KF measurement；
4. singleton 直接使用。

避免全局大尺度 DBSCAN 再次把两个 UAV 合并。

---

# 15. Scan Opportunity：核心实现

这是 proposed method 最关键部分之一。

对 track `i` 和 ray `j`：

- track position prediction：

$$
p_i(t_j),P_i(t_j)
$$

- target physical radius：

```text
R_target
```

不要用无限膨胀的大 covariance sphere 直接计数机会，否则 track uncertainty 越大，反而“机会越多”。

### 推荐：sigma-point opportunity

只对 3D position Gaussian 生成少量 sigma points：

```text
mean + ± principal axes
```

对每个 sigma point 判断：

```text
ray segment intersects sphere(center=sigma_point, radius=R_target)
```

得到：

$$
\eta_{ij}
=
\sum_n w_n
\mathbf 1[
\text{ray}_j \cap \text{targetSphere}(x_i^{(n)}) \neq \varnothing
]
$$

$$
0\le \eta_{ij}\le1
$$

---

## 15.1 在线遮挡判定

若当前 ray 的真实返回发生在预测 target near range 之前：

```text
return_range < target_near_range - occlusion_margin
```

则该 ray 被前景遮挡：

```text
eta_ij = 0
```

这不是 missed detection。

若前方存在另一 confirmed track，也按预测 near range 做前后排序。

---

## 15.2 micro-batch detection probability

对一个 batch：

$$
P_{D,i}
=
1-\prod_{j\in batch}
\left(1-p_{\rm ret}(r_i)\eta_{ij}\right)
$$

其中：

```text
p_ret(r)
```

第一版可以：

- 使用常数；
- 或 3–5 个 range bins。

必须来自统一 calibration，不允许按场景改变。

增加：

```text
P_D_max
```

防止大量强相关 rays 导致数值过度自信。

若：

```text
no opportunity
```

则：

$$
P_D=0
$$

---

# 16. Track existence probability

每条 track 保存：

```text
float existence_probability r
```

---

## 16.1 有 opportunity 但没有匹配 return

使用 Bernoulli missed-detection 更新：

$$
r^+
=
\frac{r^-(1-P_D)}
{1-r^-P_D}
$$

性质必须单元测试：

```text
P_D = 0 -> r+ = r-
P_D high and no measurement -> r decreases
```

所以：

> “没点”本身不是删轨理由；只有“真实有扫描机会但仍没有相容回波”才是负证据。

---

## 16.2 有匹配 measurement

定义 measurement likelihood：

$$
\ell(z)=
\mathcal N(z;H\hat x^-,S)
$$

与 clutter density：

$$
\kappa
$$

可使用：

$$
r^+
=
\frac{r^-P_D\ell}
{r^-P_D\ell+(1-r^-)\kappa}
$$

数值计算用 log-domain 防下溢。

`kappa` 从 no-target calibration 统计或使用固定空间密度。

---

## 16.3 track state

建议：

```text
SEED
TENTATIVE
CONFIRMED
DELETING
```

基于 existence hysteresis：

```text
confirm_threshold
delete_threshold
```

例如配置，不硬编码。

不要使用“连续漏检 N 帧”作为主要删轨逻辑。

可以保留极长 hard timeout 作为工程保险，但必须记录 reason。

---

# 17. Target quarantine 与背景解耦

每个 tentative/confirmed track 定义一个有限 target support：

```text
predicted center distribution + physical radius
```

在 map update 时：

### endpoint inside target support

不增加 bg evidence。

### free-space violation event

不增加 bg evidence。

### NO_RETURN ray passes target support

只 free carve 到 target near surface 之前。

### track deleted

quarantine 继续保留短时间：

```text
T_quarantine
```

避免刚丢轨就把悬停/短停目标吸收到背景。

这一步是“背景与飞行目标解耦”的关键实现。

---

# 18. 输出

SOFT-VoFOD 至少发布：

```text
/soft_vofod/events
/soft_vofod/tracks
/soft_vofod/background_voxels
/soft_vofod/free_voxels
/soft_vofod/candidate_background_voxels
/soft_vofod/opportunity_debug
/soft_vofod/diagnostics
```

每条 track 输出：

```text
track_id
stamp
position
velocity
covariance
existence_probability
state
age
num_positive_updates
cumulative_effective_opportunity
time_since_last_measurement
```

debug 消息不得包含 truth。

---

# 19. 第三阶段：完善仿真实验

当前 `E0_open / E1_sparse / E2_occlusion_arena / E3_cluttered` 与 S01–S06 可以复用，但要重建标准化 benchmark。

---

## 19.1 calibration 与 test 必须分离

新增：

```text
CAL01_sensor
CAL02_map
CAL03_tracker
```

只用于：

- p_ret(range)；
- map thresholds；
- process noise；
- measurement covariance；
- clutter density；
- birth thresholds。

主测试：

```text
S01–S07
```

参数冻结后运行。

---

# 20. 推荐测试场景

## S01：单目标 open-air + hover

目的：

- free-space violation；
- singleton/sparse return；
- hover 保持；
- time-to-first-track。

流程：

```text
background warm-up
-> target 出现在已建 free space
-> hover
-> 匀速移动
-> 再 hover
```

距离分段：

```text
5 / 10 / 20 / 30 / 40 m
```

---

## S02：range sweep + sparse/dropout

目标径向远离/接近 observer。

测试：

- return count 随距离；
- detection recall；
- opportunity-conditioned recall；
- track fragmentation；
- localization error。

---

## S03：双目标 crossing

设置最小 separation：

```text
0.5 / 1.0 / 2.0 / 5.0 m
```

测试：

- B0 cluster merge；
- greedy association failure；
- proposed GNN；
- IDSW；
- track coalescence。

---

## S04：4-target multi-UAV

四架 target：

- 平行；
- 交叉；
- 不同高度；
- 部分同步转向。

测试：

- scalability；
- HOTA/IDF1；
- CPU；
- memory。

可选 8-target 作为 stress，不要求主表全部完成。

---

## S05：墙体遮挡与重现

target 在墙后消失：

```text
1 s / 3 s / 5 s
```

测试：

- occluded rays 不应算 opportunity；
- false deletion；
- reacquisition latency；
- ID preservation。

---

## S06：near-background / takeoff / landing

包含：

- target 靠近墙；
- 从地面起飞；
- 短暂停；
- 进入 free space；
- 再靠近背景。

重点比较：

```text
VoFOD tentative-occupied/background trail
vs
SOFT-VoFOD target/background decomposition
```

---

## S07：observer motion / deskew stress

observer：

- 平移；
- 快速 yaw；
- roll/pitch；
- 组合运动。

target 同时运动。

测试：

- snapshot vs corrected per-ray world geometry；
- detector/tracker consistency；
- position RMSE；
- event false positive。

---

# 21. 传感器噪声与 dropout 分层

至少提供三档：

```text
N0 ideal
N1 realistic
N2 stress
```

不要凭空声称 N1 是真实 Mid360。

如果没有硬件标定数据：

- N1 命名为 `nominal_sim_noise`；
- 参数写清楚；
- 之后再用实测替换。

建议可控变量：

- range noise；
- observer pose translation noise；
- orientation noise；
- target-hit dropout；
- optional random isolated false return。

所有方法必须使用同一 source bag。

---

# 22. Benchmark 必须 record-once / replay-many

禁止 B0、B1、B2 每次各跑一遍不同随机 Gazebo。

为每个：

```text
scene × noise × seed
```

先生成 source bag：

```text
points_raw
rays_raw
TF/observer pose
target truth
scene event
```

然后离线 replay：

```text
B0
A1
A2
A3
```

保证输入完全一致。

建立 manifest：

```text
scenario
noise_mode
seed
source_bag_sha256
git_commit
config_sha256
algorithm
start/end time
```

---

# 23. Ground truth evaluator

`tclv_evaluation` 现有 ABI 可保留，但需要扩展为严格一对一、机会感知 truth。

对每个 target、每个 ray/micro-batch 至少计算：

```text
present
in_range
in_FOV
line_of_sight
num_emitted_ray_intersections
num_actual_target_returns   # 若仿真可获得 hit entity
occluded_by_static
occluded_by_target
```

这些 truth 只进入：

```text
soft_vofod_evaluation
```

绝不能进入算法 topic namespace。

---

# 24. 评估指标

不要只复刻 VoFOD 的 nearest-track + 3 m TP。

多目标必须使用一对一 assignment。

---

## 24.1 Detection/event 层

对 B0 detection：

- precision；
- recall；
- FP/min；
- localization RMSE。

对 SOFT event：

- event precision（可选）；
- free-space violation recall；
- event FP/min。

但不要把“event”与“confirmed track”强行当成同一种输出。

---

## 24.2 Track set 层：主指标

至少：

```text
HOTA
DetA
AssA
IDF1
ID switches
fragmentations
GOSPA 或 OSPA
```

匹配使用 Hungarian。

建议距离阈值做：

```text
0.5 / 1.0 / 2.0 m
```

主表选 1.0 m，其他放 sensitivity。

---

## 24.3 状态估计

对正确匹配轨迹：

```text
position RMSE
position MAE
velocity RMSE
velocity magnitude error
velocity direction error
```

同时按 range bin：

```text
0–10
10–20
20–30
30–40 m
```

---

## 24.4 轨迹连续性

必须新增：

```text
time-to-first-confirmed-track (TTFT)
track completeness
longest tracking gap
reacquisition latency
false deletion count
mean fragment duration
```

---

## 24.5 Scan-opportunity 专用指标

这是 proposed method 必须重点报告的指标：

```text
Recall | N_opp bin
miss rate | P_D bin
track deletion probability | no-opportunity interval
track deletion probability | high-opportunity miss interval
```

定义机会 bins，例如：

```text
N_opp = 0
(0,1]
(1,3]
(3,10]
>10
```

如果使用 probabilistic `P_D`，评估：

```text
Brier score
NLL
reliability diagram
```

---

## 24.6 背景地图专用指标

为了证明 background-target decomposition 有意义，增加：

### target contamination ratio

target truth 占据过的 voxel 中，被写成 `STABLE_BACKGROUND` 的比例。

### trail duration

target 离开后，其历史路径附近仍被认为 background/occupied 的持续时间。

### free-space retention

target hover 时，目标所在先验 confident-free 区域是否被错误改写。

### static background recall

真实静态背景 voxel 被正确稳定建图的比例。

### false-free rate

真实静态障碍被错误更新成 confident free 的比例。

---

## 24.7 计算性能

每个模块：

```text
mean latency
median
p95
p99
max
CPU
peak RSS
map voxel count
track count
```

不要只报告平均值。

---

# 25. 主消融设计

至少比较：

## B0

```text
VoFOD-Mid360 + classic tracker
```

## A1：Background decomposition only

```text
conservative background
+ free-space violation event
+ 简单 single-event/two-event tentative
+ 不使用 scan-opportunity existence
```

## A2：Trajectory birth

```text
A1
+ trajectory-before-confirmation birth
```

## A3：Final SOFT-VoFOD

```text
A2
+ scan-opportunity-aware existence
+ target quarantine
+ GNN/Hungarian
```

如果希望单独隔离 GNN，可再加：

```text
A2-Opp-Greedy
A3-Opp-GNN
```

但不要为了表格漂亮无限增加版本。

核心要证明：

1. trajectory birth 是否减少 sparse single-point false birth；
2. scan opportunity 是否降低 false deletion / fragmentation；
3. background-target decomposition 是否降低 map contamination/trails；
4. GNN 是否主要改善 IDSW，而不是 detection recall。

---

# 26. 统计协议

每个 stochastic 场景至少多个 seed。

建议：

```text
>= 10 seeds
```

如果运行代价太大，可先 5 个开发，最终 10 个。

报告：

- mean ± std；
- median + IQR；
- 95% bootstrap CI。

算法比较使用 paired samples，因为所有算法 replay 同一个 source bag。

禁止只挑最好一次运行。

---

# 27. no-target negative control

必须加入无目标场景：

```text
NEG01 open
NEG02 cluttered
NEG03 moving observer
```

至少运行 2–5 min 等效时间。

指标：

```text
false confirmed tracks / min
false tentative tracks / min
event FP / min
map stability
```

这是验证 trajectory-before-confirmation 非常重要的实验。

---

# 28. Baseline 与 proposed 的公平性

所有算法共享：

- 同一个 source bag；
- 同一个 preprocessor；
- 同一个 observer pose；
- 同一合法 ray set；
- 同一 operational region；
- 同一 warm-up；
- 同一 hardware/sim source。

不同算法不得拥有不同 target truth、visibility truth 或不同传感器数据。

如果 B0 与 proposed 因算法结构需要不同 map 参数，必须：

- 各自在 calibration split 上选择；
- 测试时冻结；
- 公开参数来源。

---

# 29. 结果目录

恢复规范化 artifacts，但不要覆盖历史不存在的数据。

建议：

```text
artifacts/
  source_bags/
  runs/
    B0/
    A1/
    A2/
    A3/
  metrics/
  plots/
  manifests/
  logs/
```

每次 run 包含：

```text
run_manifest.yaml
algorithm_config.yaml
metrics.json
metrics.csv
timing.csv
stdout.log
rosbag output
```

---

# 30. 自动化 runner

在 `soft_vofod_evaluation` 中实现：

```bash
run_benchmark.py
```

支持：

```bash
--algorithms B0,A1,A2,A3
--scenes S01,S02,S03
--noise N0,N1
--seeds 0,1,2,3,4
--replay
--output artifacts/
```

必须：

- 检测 ROS master；
- 检测 launch startup timeout；
- 检测算法 crash；
- 检测 bag end；
- 保存 git/config hashes；
- 单次失败不吞掉错误；
- 产生 machine-readable summary。

---

# 31. 单元测试与集成测试

至少补齐：

## ray/preprocessor

- scan_id pair；
- original_index；
- duplicate；
- invalid range；
- no-return；
- points_world == ray endpoint；
- TF interpolation；
- time rollback。

## map

- unknown -> free；
- unknown -> candidate bg；
- candidate -> stable bg；
- free violation 不写 bg；
- target quarantine；
- no-return truncation；
- valid endpoint guard。

## birth

- 1 event 不 confirmed；
- 3 consistent groups birth；
- random isolated events 不 birth；
- hover birth；
- impossible speed rejection。

## opportunity

- no intersect -> P_D=0；
- intersect + front occlusion -> P_D=0；
- intersect + no-return -> opportunity；
- uncertainty sigma points；
- P_D monotonic；
- cap 生效。

## existence

- P_D=0 miss 不降 r；
- high P_D miss 降 r；
- hit 提高 r；
- hysteresis；
- hard timeout reason。

## association

- 2 tracks / 2 measurements；
- crossing；
- singleton；
- one measurement cannot be assigned to two tracks。

---

# 32. 重要的 failure cases 必须主动保留

不要把下列情况用工程 trick 隐藏掉：

1. target 在系统启动前就静止于从未建图的 unknown 空间：
   - 无先验背景时信息上不可区分于新静态物体；
   - 应标为 limitation。

2. 两架目标完全共线且只产生一个 indistinguishable return：
   - identity 暂时不可观；
   - GNN 不能凭空解决。

3. target 非常靠近 stable background：
   - free-space violation evidence 可能弱；
   - existing track 可继续维护，但 birth 可能失败。

4. observer pose 错误：
   - 可能在大范围产生 false anomaly；
   - 需要诊断与 map-update pause，而不是 truth correction。

---

# 33. observer pose quality gate

增加可选安全门。

当：

```text
TF discontinuity
pose covariance too large
time jump
```

发生时：

- 暂停 stable background promotion；
- 可以继续 track predict；
- 降低或暂停 event birth；
- diagnostics 明确 reason。

不要偷偷使用 ground truth pose 修正。

---

# 34. README 与论文可用文档

最终生成：

```text
docs/B0_FROZEN_SPEC.md
docs/SOFT_VOFOD_ALGORITHM.md
docs/SOFT_VOFOD_FORMULAS.md
docs/EXPERIMENT_PROTOCOL.md
docs/METRICS_SPEC.md
docs/ABLATION_PLAN.md
docs/KNOWN_LIMITATIONS.md
docs/IMPLEMENTATION_CHANGELOG.md
```

`SOFT_VOFOD_ALGORITHM.md` 必须完整写清：

```text
raw Mid360
 -> preprocessing
 -> map query
 -> event
 -> birth
 -> association
 -> KF
 -> opportunity
 -> existence
 -> quarantine
 -> map commit
 -> trajectory output
```

每个公式解释所有符号。

---

# 35. 完成标准

本任务不是“代码能编译”就结束。

必须满足：

## Build

```bash
catkin build
```

成功。

## Tests

所有新增 gtest/rostest 通过。

## B0 regression

公共 preprocessor 修正后：

- B0 可稳定运行；
- canonical launch 唯一；
- 不依赖 truth；
- no-target 不产生明显持续假轨迹；
- map/track outputs finite。

## Proposed minimal demo

至少：

```text
S01 single hover
S03 two-target crossing
S05 wall occlusion
```

能完成 end-to-end。

## Evaluation

至少生成：

- HOTA/IDF1；
- RMSE；
- TTFT；
- fragmentation；
- IDSW；
- opportunity-conditioned recall；
- map contamination；
- runtime p95。

---

# 36. 实现顺序

严格按以下顺序推进：

```text
Phase 0  audit + tests
Phase 1  scan identity / points_world / canonical B0
Phase 2  rebuild evaluation runner + source-bag replay
Phase 3  conservative background map
Phase 4  free-space violation event
Phase 5  trajectory birth
Phase 6  CV-KF + GNN
Phase 7  scan opportunity + existence
Phase 8  quarantine feedback
Phase 9  ablation + benchmark
Phase 10 docs
```

每个 phase 完成后：

1. build；
2. unit test；
3. 最小集成测试；
4. 更新 changelog；
5. 再进入下一 phase。

---

# 37. 不允许的实现捷径

禁止：

```text
if scene_name == ...
if target_id == ...
if z > 某个专门为某场景写的高度...
读取 /ground_truth 后改变算法输出
用 GT visibility 作为在线 opportunity
根据 truth box 过滤点云
固定目标出生坐标
按场景切换 cluster threshold
```

允许 ground truth 的唯一位置：

```text
soft_vofod_evaluation
```

---

# 38. 最终需要向我汇报的内容

完成后输出一份：

```text
docs/FINAL_IMPLEMENTATION_REPORT.md
```

包括：

1. 修改了哪些文件；
2. 哪些是公共基础设施修复；
3. 哪些是 B0 baseline 修复；
4. 哪些是 SOFT-VoFOD 新算法；
5. 所有新增参数及默认值；
6. 所有公式与代码位置对应表；
7. 所有测试；
8. 所有 benchmark；
9. 失败场景；
10. 性能瓶颈；
11. 下一步真实 Mid360 接入事项。

不要只说“已完成”；必须给出可检查的文件路径、launch 命令和结果路径。

---

# 39. 算法核心不可偏离

实现过程中如果需要做取舍，优先保持以下三条：

$$
\boxed{
\text{背景由长期保守空间记忆表示，目标不写入背景}
}
$$

$$
\boxed{
\text{单个异常点不确认目标，轨迹一致性才产生 target}
}
$$

$$
\boxed{
\text{只有真实存在扫描机会时，未检测才是负证据}
}
$$

这三条是本研究的核心；PMBM、JPDA、神经网络、复杂背景生成模型都不是当前阶段目标。
