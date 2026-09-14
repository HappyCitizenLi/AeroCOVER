# 当前 AeroCOVER 与 VoFOD：实现、实验与分析

> 状态日期：2026-09-08  
> 事实来源：当前工作区源码、当前 YAML 配置、P01/P02 不可变 source bags、最新 COMPLETE run manifests 与 metrics.json。  
> 范围：仅包括 AeroCOVER、VoFOD-Mid360、P01 Office、P02 Forest。  
> 定位：这是当前工程事实说明，不是论文中的统计显著性结论，也不声称参数已经跨场景校准。

## 1. 结论摘要

当前两个算法使用同一组 world-frame Mid-360 点与逐射线语义输入，但检测思想不同：

| 算法 | 核心判据 | 跟踪器 | 当前 P01/P02 结果 |
|---|---|---|---|
| AeroCOVER | 小型时空回波 component 周围是否存在多帧、全方向、完整穿越的历史自由空间 shell | 包内 9 状态 CA + Hungarian | 299 TP / 0 FP / 15 FN；macro HOTA 0.9729 |
| VoFOD-Mid360 | 当前 component 是否远离已知背景，且不能经未知体素连接到背景或搜索边界 | 独立 lidar_tracker_mid360，9 状态 CA-LKF | 51 TP / 1117 FP / 263 FN；macro HOTA 0.1147 |

AeroCOVER 在这两条开发场景上实现了 100% precision、0 false birth、0 ID switch、0 fragmentation；代价是更严格的证据要求和更高的当前计算量。它的 338 帧加权平均 runtime 为 48.118 ms/scan。

VoFOD 的 detector 加权平均为 28.896 ms/scan，但该数字不包含独立 tracker 的耗时，因此不能直接作为完整 pipeline 与 AeroCOVER 的端到端速度比较。VoFOD 在 P01 只有部分目标召回且产生大量 false tracks，在 P02 没有任何目标 TP。

这些结论只覆盖：

- 每个场景一个目标；
- 每个场景一个 seed；
- 无额外测距噪声的 N0 输入；
- Gazebo Classic + MRS X500/PX4/MAVROS/MPC；
- P01/P02 两条已经参与算法开发和参数选择的路线。

因此，目前证据足以说明机制在这两个场景上成立，不足以说明跨地图泛化、真实硬件鲁棒性或多目标能力。

## 2. 系统边界与共同输入

### 2.1 数据链

~~~text
Gazebo Classic + MRS/PX4
        │
        ├─ Mid-360 每 scan 20,000 条 attempted rays
        │
        ├─ /uav1/mid360/points_world
        │    仅包含 VALID_RETURN endpoint
        │
        └─ /uav1/mid360/rays_checked
             每条 ray 保留 original_index、offset_time、origin、
             world-frame direction、range、return_status
                    │
          ┌─────────┴──────────┐
          │                    │
     AeroCOVER             VoFOD-Mid360
   检测 + 内置 CA      检测器 + 外部 CA tracker
          │                    │
   /aerocover/tracks   /uav1/batch/b0/tracks
          └─────────┬──────────┘
                    │
           同一 evaluator、1 m 主门限
~~~

共同接口定义在：

- [mid360_ray_msgs](../src/mid360_ray_msgs/msg/CheckedRayBundle.msg)
- [Mid-360 preprocessor](../src/mid360_ray_preprocessor/src/mid360_ray_preprocessor_node.cpp)
- [仿真传感器插件](../src/mid360_simulation_plugin_fork/livox_laser_simulation/src/livox_points_plugin.cpp)

两种算法都要求点云与 checked-ray bundle 时间戳完全相同，并使用同一 world frame。实验输入模式为 sim_exact，rolling-scene 几何在每条 ray 的时间点计算，而不是把整帧当作一个瞬时快照。

### 2.2 Ray 状态语义

| 状态 | 含义 | AeroCOVER | VoFOD |
|---|---|---|---|
| VALID_RETURN | 有可信 endpoint | 自由段终点为 range − 0.50 m，证据权重 1.0 | 自由段终点为 min(range − 0.50 m, 20 m)，更新权重 0.003 |
| NO_RETURN | 在可靠范围内没有返回 | 自由段长度 20 m，证据权重 0.25 | 自由段长度 20 m，更新权重 0.003 |
| BELOW_MIN_RANGE / INVALID_RANGE / UNKNOWN | 不可用或不可信 | 不进入 shell 证据 | 计入拒绝 diagnostics，不更新 map |

两者都把当前回波 endpoint 与自由射线语义分开处理；VALID_RETURN 的 0.50 m margin 用于避免把 endpoint 表面及其测距误差附近误当作自由空间。

## 3. 两个保留场景

### 3.1 P01 Office

配置：[P01.yaml](../src/mid360_multi_uav_sim/config/benchmarks/P01.yaml)  
世界：[PW_office.world](../src/mid360_multi_uav_sim/worlds/PW_office.world)

| 项目 | 当前值 |
|---|---|
| scenario_id | P01_planning_worlds_office_los_chase |
| seed | 6301 |
| 轨迹 | continuous cubic，tangent scale 0.8 |
| 运动时长 | 23.8 s |
| 评分区间 | 1.0–22.8 s，共 21.8 s / 218 帧 |
| 路线特征 | 开放区下降转弯，进入约 1.73 m 狭窄走廊，贴墙飞行并交替升降 |
| observer | 沿目标上一段路线追随，实测最小机间距 5.970 m |
| 实测最大水平速度/加速度 | 2.978 m/s / 3.073 m/s² |
| 实测最大垂直速度/加速度 | 0.250 m/s / 0.279 m/s² |
| 实测最小静态障碍物净空 | 0.350 m |

目标从 (−5.3, 14.6, 1.75) 平滑转向 (−1.2, 8.45, 2.25)，随后沿 y≈8.05 的窄走廊向东飞行至 x≈35.3；z 在约 1.6–2.3 m 间变化。observer 使用相似路线并落后一个 waypoint。

### 3.2 P02 Forest

配置：[P02.yaml](../src/mid360_multi_uav_sim/config/benchmarks/P02.yaml)  
世界：[PW_forest_seed0.world](../src/mid360_multi_uav_sim/worlds/PW_forest_seed0.world)

| 项目 | 当前值 |
|---|---|
| scenario_id | P02_planning_worlds_forest_los_chase |
| seed | 0 |
| 轨迹 | continuous cubic，tangent scale 0.8 |
| 运动时长 | 14.0 s |
| 评分区间 | 1.0–13.0 s，共 12.0 s / 120 帧 |
| 路线特征 | 250 根圆柱树干间的连续转弯、近树通过、升高与降低 |
| observer | 在 LOS 较安全一侧落后目标约 6–9 m；实测最小机间距 6.023 m |
| 实测最大水平速度/加速度 | 2.887 m/s / 2.843 m/s² |
| 实测最大垂直速度/加速度 | 0.380 m/s / 0.428 m/s² |
| 实测最小静态障碍物净空 | 0.572 m |

目标依次经过约 (8.26, 21.24, 1.7)、(14.08, 19.88, 2.5)、(18.85, 15, 1.8)、(25.52, 15, 2.7)、(31, 10, 1.9)。observer 的最后一段保持在目标北侧，以减少树干对 LOS 的遮挡。

### 3.3 不可变同源输入

| Scene | source bag SHA-256 |
|---|---|
| P01 | 9b57d3b28ddd4dc1881d861356f382a703fb483f02d64f75ca96f7b926eb6fd1 |
| P02 | 1037febff80273e314108bb23681ca6721f53aed5c704851d11028efed10edc9 |

位置：

~~~text
results/aerocover_vofod_p01_p02_1seed/sources/P01/N0/source.bag
results/aerocover_vofod_p01_p02_1seed/sources/P02/N0/source.bag
~~~

两个算法均 replay 这些 bag，因而传感器输入、真值和评分区间同源。P01 有 15 个 present 但不满足 range/FOV 的 truth samples 被主 tracking score 排除，P02 有 9 个；剩余可评分 truth 数分别为 203 和 111。

## 4. AeroCOVER 当前完整框架

### 4.1 当前唯一主流程

~~~text
ExactTime 输入与几何校验
  → 删除 1 s FIFO 中过期 points/rays
  → 当前 + FIFO 回波点做 0.30 m 精确三维连通
  → 历史背景标签传播
  → 按 scan 与回波时间切片，单切片尺寸 > 1.5 m 则整 component 为背景
  → 对当前非背景点形成 observation，估计帧内速度并 deskew
  → 构造 P95 + 0.10 m、半径上限 0.75 m 的目标球和 1 m shell
  → 仅用此前历史 rays 做 full-chord 42-bin shell 预筛
       ├─ strict 42/42：可进入 candidate / birth
       └─ S35：只能维护预测位置 1 m 内的已生 track
  → 已生 tracks：9 状态 CA 预测 + Hungarian 关联
  → 未分配 strict observations：candidate 关联与 3-of-5 birth
  → 最后才插入当前 points/rays，杜绝当前帧自证
~~~

主实现文件：

- [aerocover_core.cpp](../src/aerocover_mid360/src/aerocover_core.cpp)
- [ray_geometry.cpp](../src/aerocover_mid360/src/ray_geometry.cpp)
- [ca_tracker.cpp](../src/aerocover_mid360/src/ca_tracker.cpp)
- [aerocover_node.cpp](../src/aerocover_mid360/src/aerocover_node.cpp)
- [当前配置](../src/aerocover_mid360/config/aerocover_mvp.yaml)

当前代码不包含 core evidence、PlanePatch、旧 current-scan PCL clustering、外部 tracker、truth 输入或 dormant re-identification。检测与跟踪均在 aerocover_mid360 包内。

### 4.2 输入校验与因果顺序

ROS adapter 使用 ExactTime 同步点云和 ray bundle，并检查：

- frame 均为 world；
- source_mode 为 sim_exact 或 hw_spherical_exact；
- 每 bundle 恰好 20,000 rays；
- scan_id 与 decision stamp 严格单调；
- 点的 scan_id、original_index、offset time 与 ray 一致；
- VALID_RETURN 点满足 endpoint = origin + range × direction，容差 0.01 m；
- 坐标、强度、时间、方向与 range 有限；
- current ray 不得在当前 decision 前进入历史 FIFO；
- FIFO 不得含 future-stamped ray。

处理顺序的关键约束是：

~~~text
decision(scan k) 只能使用 scan < k 的 ray evidence
decision(scan k) 完成后，scan k 的 points/rays 才进入 FIFO
~~~

因此，目标当前帧自己的回波射线不能为自己的 shell 提供自由证据。实验中 future_stamp_violation、self_support_violation 均为 0。

### 4.3 一秒 point/ray 历史

point FIFO 与 ray FIFO 当前都由 ray_fifo_s=1.00 s 控制。10 Hz 数据下，稳定状态约保存 10 个历史 scans：

- P01/P02 的 ray FIFO 均约 200,001 条 active rays；
- P01 历史点均值约 84,730，P02 约 83,474；
- 整个旧 scan 过期时，按 scan block 一次删除其 accumulator 合计；
- FIFO 边界落在某 scan 内时，才逐 ray 撤销精确贡献。

配置中的 time/point_window_s=0.10 当前只被读取和合法性校验，没有用于 point FIFO 截断；实际 point history 仍是 ray_fifo_s=1.00 s。这是需要在后续清理的无效配置项，不应把它解释为当前算法仅保存 0.10 s 点历史。

### 4.4 精确时空连通 component

令当前点与 FIFO 点集合为 \(P=\{p_i\}\)。若

\[
\lVert p_i-p_j\rVert_2 \le r_c,\qquad r_c=0.30\ \mathrm{m},
\]

则两点之间存在无向边；连通分量即时空 component。这里的“时空”含义是：点来自约 1 s 的多个 scans，但连边条件本身只有精确三维欧氏距离，没有把时间作为第四维距离。

当前实现没有使用 PCL KdTree，而是：

1. 取 cell size

\[
h=\operatorname{nextafter}\left(\frac{r_c}{\sqrt 3},0\right)
\approx 0.1732\ \mathrm{m};
\]

2. 将点放入哈希网格；
3. 枚举预计算的 62 个单侧邻 cell offset；
4. 先做 cell AABB 距离下界排除；
5. 再做精确 point-to-point 平方距离判断；
6. 用 union-find 合并，最后生成扁平 component 索引。

选择 \(h<r_c/\sqrt3\) 的目的，是保证同一 cell 内任意两点都严格在连接阈值内，同时仍通过邻 cell 的精确距离判断保持与半径图相同的 membership。

### 4.5 背景传播与逐 scan 时间切片

对每个时空 component，先统计其 FIFO 历史点：

\[
\rho_{\mathrm{bg}} =
\frac{N_{\mathrm{historical\ bg}}}{N_{\mathrm{historical}}}.
\]

若同时满足

\[
N_{\mathrm{historical\ bg}}\ge3,\qquad
\rho_{\mathrm{bg}}\ge0.20,
\]

则整个 component 标记为 background，标签写回历史点并传给当前点。

若未传播为背景，则先按 (scan_id, stamp, original_index) 排序，再在同一 scan 内把相邻时间差不超过 0.010 s 的点放入同一 temporal slice。只检查至少 3 点的 slice。令 slice AABB 为

\[
e=\max(p)-\min(p).
\]

只要任一轴满足

\[
\max(e_x,e_y,e_z)>1.50\ \mathrm{m},
\]

整个时空 component 即标记为背景。这里检查的是“每个 scan 内的时间 slice 尺寸”，不是把一秒内移动路径的总尺寸当作目标尺寸；运动目标跨 scans 形成很长轨迹时不会仅因总轨迹长而被拒绝。

未标为背景的 current points 成为 residual，并以其当前 scan 空间 component 形成 observation。

### 4.6 Observation、帧内运动补偿与几何

当前 observation 的点先按 0.02 s 分为 motion bins。每个 bin 的中心使用 x/y/z 分量中位数。若至少有 3 个 bins，则对 bin center 做线性最小二乘：

\[
\hat v =
\frac{\sum_j(t_j-\bar t)(c_j-\bar c)}
     {\sum_j(t_j-\bar t)^2}.
\]

只有同时满足

\[
\lVert\hat v\rVert\le12\ \mathrm{m/s},\qquad
\mathrm{RMS}_{CV}\le0.50\ \mathrm{m}
\]

才接受帧内速度；否则令 \(\hat v=0\)。

每个点被补偿到 scan end：

\[
p_i'=p_i-\hat v(t_i-t_{\mathrm{end}}).
\]

位置 observation 是补偿后点的均值：

\[
z=\frac1N\sum_i p_i'.
\]

测量协方差为

\[
R=
\frac1N\left[
\frac1{N-1}\sum_i(p_i'-z)(p_i'-z)^\top
\right]
+\sigma_{\min}^2 I,\qquad \sigma_{\min}=0.20\ \mathrm{m},
\]

其中单点 observation 的散布项置零，最终再显式对称化。

令 \(d_i=\lVert p_i'-z\rVert\)，目标 component 球半径为

\[
r_{\mathrm{component}}
=\operatorname{clamp}
\left(P_{95}(\{d_i\})+0.10,\ 0.15,\ 0.75\right)\ \mathrm{m}.
\]

使用 P95 而非最大距离，可以降低少量离群点把目标球和 shell 整体撑大的影响；+0.10 m 给目标表面留安全余量；clamp 把半径限制在 0.15–0.75 m，即当前目标直径先验上限 1.50 m。

shell 半径为

\[
r_{\mathrm{inner}}=r_{\mathrm{component}}+0.05,\qquad
r_{\mathrm{outer}}=r_{\mathrm{inner}}+1.00.
\]

所以当前 outer radius 范围为 1.20–1.80 m。

### 4.7 精确 ray–sphere–shell 几何

对 ray origin \(o\)、单位方向 \(d\)、候选中心 \(c\) 和半径 \(R\)，定义

\[
q=o-c,\qquad b=q^\top d,\qquad
\Delta_R=b^2-(q^\top q-R^2).
\]

若 \(\Delta_R<0\)，ray 不与球相交；否则直线交点参数为

\[
t_{\pm}=-b\pm\sqrt{\Delta_R}.
\]

交点再裁剪到可信自由段 \([0,L]\)。outer sphere 的交段减去 inner sphere 交段，最多得到两个 shell intervals。实现对 outer/inner 共用 \(b\) 和 \(q^\top q\)，并使用固定长度为 2 的 interval 数组。

require_full_chord=true 时，outer sphere 的完整 chord 必须严格落在可信自由段内部：

\[
t_{\mathrm{outer,in}}>\epsilon,\qquad
t_{\mathrm{outer,out}}<L-\epsilon.
\]

这会排除以下伪证据：

- ray 从 outer sphere 内部出发；
- VALID_RETURN endpoint 停在 shell 中或 shell 边界；
- ray 只进入 shell、但没有从另一侧离开；
- 墙面自身的返回把墙面候选周围的一小段误计为“完整自由包围”。

因此，full chord 不是为了增加计算，而是明确区分“表面前方有一段自由”与“目标周围的自由区域被完整穿过”。

### 4.8 42 个方向 bin 如何形成 shell 证据

球面方向使用 42 个 Fibonacci directions：

\[
z_i=1-\frac{2(i+0.5)}{42},\quad
\phi_i=i\cdot2.3999632297,\quad
u_i=(\sqrt{1-z_i^2}\cos\phi_i,\sqrt{1-z_i^2}\sin\phi_i,z_i).
\]

每段 shell chord 以 0.10 m 步长采样。样本相对候选中心的方向被分配给点积最大的 \(u_i\)。令 ray \(r\) 在 bin \(b\) 中的累计 shell 长度为 \(\ell_{r,b}\)，单 ray 贡献为

\[
\delta_{r,b}=w_r\min\left(1,\frac{\ell_{r,b}}{0.20}\right),
\]

其中

\[
w_r=
\begin{cases}
1.0,&\text{VALID_RETURN},\\
0.25,&\text{NO_RETURN}.
\end{cases}
\]

同一 scan 内同一 bin 的所有 ray 先相加，再封顶为 1：

\[
g_{s,b}=\min\left(1,\sum_{r\in s}\delta_{r,b}\right).
\]

跨 scans 的密度为

\[
D_b=\sum_s g_{s,b}.
\]

bin 的判定：

- observable：至少一个历史 ray 对该 bin 有正贡献；
- supported：\(D_b\ge1.50\)；
- shell coverage：

\[
C=\frac{N_{\mathrm{supported}}}
        {\max(1,N_{\mathrm{observable}})}.
\]

当前 strict gate 同时要求：

\[
N_{\mathrm{observable}}=42,\quad
N_{\mathrm{supported}}=42,\quad
C\ge0.60,
\]

并且 distinct supporting scans ≥ 2、supported octants ≥ 2。因为 42/42 已经意味着 \(C=1\) 且方向覆盖全部球面，所以 0.60 coverage 与 2 octants 在当前 strict 配置下基本是冗余保护；它们主要对消融配置有意义。

“shell 42”不是 42 条 ray，也不是球体内有 42 个点，而是 42 个球面方向 bin 全部 observable 且全部达到跨 scan 密度 1.5。

### 4.9 Strict birth 与 S35 maintenance 的职责分离

每个 observation 先经过 pass-only shell evaluator：

- strict observation：42 observable / 42 supported；
- maintenance observation：至少 35 observable / 35 supported；
- 其余 observation 被丢弃。

S35 不参与新目标 birth。它只在已有 track 存在时启用，且后续 tracker 关联还必须满足 observation 与 CA prediction 的欧氏距离 ≤ 1.0 m。这样做的目的，是允许已确认目标在短时遮挡或方向采样不足时继续更新，同时不把较弱证据开放给新生 false track。

严格 observation 先排列，maintenance observation 后排列；birth_eligible_observation_count 只覆盖前者。因此代码结构上也保证 maintenance observation 不会进入 candidate birth。

### 4.10 Candidate、证据重建与 3-of-5 birth

未被 active track 分配的 strict observations 才进入 candidate 层。

Candidate 关联首先用 CV 预测：

\[
\hat p_k=p_{k-1}+v_{k-1}\Delta t.
\]

若位置距离 ≤ 1.0 m，直接形成候选 pair；否则只有在测量协方差和 11.345 卡方门内时才可能成为 pair。所有 pair 按距离、candidate ID、observation index 排序后做确定性一对一 greedy 分配。Candidate 层不是 Hungarian；Hungarian 仅用于 born tracks。

Candidate 的跨 scan CV 速度由最近最多 5 个 observations 做最小二乘。有效条件为：

\[
\lVert v\rVert\le12\ \mathrm{m/s},\qquad
\mathrm{RMS}_{CV}\le0.20\ \mathrm{m}.
\]

candidate evidence geometry 只有在以下任一条件成立时重建：

\[
\lVert c_{\mathrm{new}}-c_{\mathrm{evidence}}\rVert>0.20\ \mathrm{m},
\]

或

\[
\frac{|r_{\mathrm{new}}-r_{\mathrm{evidence}}|}
     {r_{\mathrm{evidence}}}>0.20.
\]

重建时，只查询每个历史 scan 的 ray spatial index 中与 outer sphere 可能相交的 rays，再做精确相交。未触发重建时，新的 rays 增量加入既有 scan × 42-bin accumulator；过期 ray/scan 精确撤销。

每次 candidate 被观测时记录一个 birth bit：

\[
b_k=
\begin{cases}
1,&\text{geometry eligible 且 strict shell pass},\\
0,&\text{否则}.
\end{cases}
\]

在最近 5 scans 中满足至少 3 个 true，且 candidate observation_count ≥ 3 时 birth：

\[
\sum_{j=k-4}^{k}b_j\ge3.
\]

Candidate 连续 missed scans > 2，或距离最后 observation > 1.0 s 时删除。Birth 初速度优先使用跨 scan candidate CV，其次使用帧内 motion fit，否则为 0。

### 4.11 内置 9 状态 CA Kalman tracker

状态为

\[
x=
[p_x,p_y,p_z,v_x,v_y,v_z,a_x,a_y,a_z]^\top.
\]

对时间间隔 \(\Delta t\)，转移矩阵为

\[
F=
\begin{bmatrix}
I&\Delta tI&\frac12\Delta t^2I\\
0&I&\Delta tI\\
0&0&I
\end{bmatrix}.
\]

模型使用连续白 jerk，\(q=\sigma_j^2\)，当前 \(\sigma_j=2.0\ \mathrm{m/s^3}\)。每个空间轴的过程噪声块为

\[
Q_1=q
\begin{bmatrix}
\Delta t^5/20&\Delta t^4/8&\Delta t^3/6\\
\Delta t^4/8&\Delta t^3/3&\Delta t^2/2\\
\Delta t^3/6&\Delta t^2/2&\Delta t
\end{bmatrix},
\]

三轴组成 9×9 的 \(Q\)。

预测为

\[
\hat x_k^-=F\hat x_{k-1},\qquad
P_k^-=FP_{k-1}F^\top+Q.
\]

测量矩阵 \(H=[I\ 0\ 0]\)。位置 innovation 与协方差为

\[
\nu=z-H\hat x^-,\qquad
S=HP^-H^\top+R.
\]

只有

\[
\nu^\top S^{-1}\nu\le11.345
\]

的 pair 才可关联。11.345 是当前三维 gate 参数。strict 与 maintenance observations 一起进入 born-track Hungarian，一对一最小化 Mahalanobis cost；maintenance pair 还要满足前述 1.0 m 欧氏门。

更新使用 Kalman gain

\[
K=P^-H^\top S^{-1},
\]

以及 Joseph covariance form：

\[
P=(I-KH)P^-(I-KH)^\top+KRK^\top.
\]

Birth covariance：

- position：当前 observation \(R\)；
- velocity：\(3.0^2I\)；
- acceleration：\(2.0^2I\)。

若任一位置标准差超过 5.0 m，或最后一次测量距当前时间超过 1.0 s，则删除 track。

### 4.12 当前参数表

| 模块 | 参数 | 当前值 | 具体作用 |
|---|---|---:|---|
| Input | sync_queue_size | 32 | ExactTime 同步队列容量 |
| Input | expected_rays_per_bundle | 20,000 | 完整 attempted-ray bundle 合约 |
| Input | geometry tolerance | 0.01 m | endpoint 与 ray 几何一致性 |
| Time | point_window_s | 0.10 s | 当前实现未用于截断；实际点历史由 ray_fifo_s 控制 |
| FIFO | ray_fifo_s | 1.00 s | points 与 rays 的实际历史窗口 |
| FIFO | unknown_timeout_s | 1.00 s | candidate 无观测超时 |
| Rays | return_endpoint_margin | 0.50 m | VALID_RETURN 自由段从 endpoint 前退 |
| Rays | no_return_trusted_range | 20.0 m | NO_RETURN 最大可信自由段 |
| Rays | valid/no-return weight | 1.0 / 0.25 | shell 证据权重 |
| Observation | min points / max extent | 1 / 3.0 m | 形成 observation 的最低点数与最终几何门 |
| Observation | measurement sigma floor | 0.20 m | centroid covariance 下限 |
| ST cluster | spatial_tolerance | 0.30 m | 当前+历史点精确连通距离 |
| Time slice | temporal_gap | 0.010 s | 同 scan slice 相邻点最大时间差 |
| Time slice | temporal_min_points | 3 | 执行尺寸检查的最小点数 |
| Background | max_slice_extent | 1.50 m | 任一轴超过即背景 |
| Background | min points / ratio | 3 / 0.20 | 历史背景标签传播 |
| Motion | time_bin | 0.02 s | 帧内速度拟合 |
| Motion | max speed / RMS | 12 m/s / 0.50 m | 帧内 CV 有效性 |
| Geometry | radius margin | 0.10 m | P95 外扩 |
| Geometry | radius clamp | 0.15–0.75 m | 目标球半径边界 |
| Shell | gap / thickness | 0.05 / 1.00 m | inner 与 outer shell |
| Shell | bins | 42 | Fibonacci 球面方向 |
| Shell | sample / norm | 0.10 / 0.20 m | chord 离散与单 ray 饱和长度 |
| Shell | density threshold | 1.50 | supported bin 阈值 |
| Shell | coverage threshold | 0.60 | supported/observable 比例；strict 42/42 下冗余 |
| Strict | observable/supported | 42 / 42 | 新目标证据门 |
| Strict | scans / octants | 2 / 2 | 独立历史与方向保护 |
| Birth | history/supports | 5 / 3 | 3-of-5 |
| Maintenance | bins / prediction gate | 35 / 1.0 m | 仅 born track 的弱化门 |
| Candidate | association / CV RMS / missed | 1.0 m / 0.20 m / 2 scans | candidate 连续性与删除 |
| Rebuild | translation / radius fraction | 0.20 m / 0.20 | 触发精确证据重建 |
| Evidence index | spatial hash cell | 1.0 m | per-scan ray segment 索引 |
| Audit | every N scans / tolerance | 100 / 1e−9 | incremental/reference 一致性检查 |
| Tracker | Mahalanobis gate | 11.345 | 3D track association |
| Tracker | jerk sigma | 2.0 m/s³ | CA process noise |
| Tracker | measurement floor | 0.20 m | Kalman R 最小特征值 |
| Tracker | initial velocity/acceleration sigma | 3.0 m/s / 2.0 m/s² | birth covariance |
| Tracker | max missed / max sigma | 1.0 s / 5.0 m | track 删除 |

当前 ablation flags 全部开启：use_shell_evidence、require_full_chord、use_no_return_rays、use_incremental_evidence、use_ray_spatial_index。它们是机制测试开关，不代表存在另一条当前 production 流程。

### 4.13 ROS 输出

| Topic | Type/内容 |
|---|---|
| /aerocover/tracks | lidar_tracker_mid360/Tracks；当前 9-state CA tracks |
| /aerocover/diagnostics | AeroCoverDiagnostics；输入、component、shell、birth、track 和 runtime |
| /aerocover/background_points | 当前 scan 被判 background 的点 |
| /aerocover/residual_points | 当前 scan 未判背景的 residual points |
| /aerocover/markers | candidate sphere/shell/directions 与 active track markers |

CandidateEvidence、BirthEvent 和 AeroCoverTrack 嵌入 diagnostics；没有另一个独立 candidate/birth topic。

### 4.14 当前性能实现

当前实现已经包含下列不改变判定语义的性能路径：

- point connectivity 使用精确网格 + union-find，不保存重复 positions 副本；
- thread-local scratch 重用 references、cells、CSR indices、union-find 与 temporal containers；
- ray 以 scan block 保存，ID 在块内连续，可直接定位；
- SpatialHash3D 为每个 scan 索引 ray segment，candidate 反向 query sphere；
- query sphere 先做 cell AABB–sphere 精确下界过滤；
- ray DDA 的 segment-cell buffer 重用；
- ray–sphere 的 offset、projection、offset² 只计算一次；
- shell intervals 与 42-bin 临时值使用固定数组；
- pass-only shell evaluator 在单 scan 的 42 bins 饱和时停止；
- 根据剩余 scans 的理论最大贡献做不可能通过的提前拒绝；
- 42/42 已满足时可提前成功；
- accumulator 只保存 scan × bin totals，精确贡献留在 RayRecord；
- candidate geometry 位移/半径变化未超过阈值时不重建；
- 每 100 scans 最多对一个 candidate 做一次全 FIFO reference audit。

这些优化后，evidence rebuild 已不是主耗时：P01/P02 平均仅 0.193/0.342 ms。主要成本转为一秒点历史的精确连通，以及对残余 observations 的 shell prefilter 和每帧 20,000 rays 的 spatial-index insertion。

## 5. VoFOD-Mid360 当前完整框架

### 5.1 版本边界

VoFOD-Mid360 是 CTU-MRS VoFOD 的受控兼容 fork，不是官方仓库未修改版本：

- VoFOD 上游仓库：https://github.com/ctu-mrs/vofod
- 固定上游 commit：7da9f33a878a586588f6a626b75cfeacac7824f7
- 外部 tracker 上游仓库：https://github.com/ctu-mrs/lidar_tracker
- tracker 固定 commit：a92b4db61060b47f1af6dcce122188ec021f2dcd

本地保留上游风格的 scalar occupancy、close/far component、floating exploration、OBB detection 与 separate-background cleanup；传感器边界改为 Mid-360 的 world-frame checked rays，并把并发 map commit 改成确定性的同步执行。完整本地边界见：

- [VoFOD provenance](../src/vofod_mid360/UPSTREAM.md)
- [tracker provenance](../src/lidar_tracker_mid360/UPSTREAM.md)
- [VoFOD README](../src/vofod_mid360/README.md)
- [tracker README](../src/lidar_tracker_mid360/README.md)

当前 P01/P02 冷启动使用固定 10-scan lag 与 persistent-structure certificate，不使用上游 Rangefinder seed。因此本文将其称为“受控 VoFOD-Mid360 baseline”，而不是“未经修改的原论文实现”。

### 5.2 检测器完整流程

~~~text
ExactTime points_world + rays_checked
  → 严格输入、索引和 endpoint 几何校验
  → 0.50 m aligned weighted voxel cloud
  → 1.50 m PCL Euclidean clustering + OBB
  → 更新 11-scan hit history，生成 persistent-structure certificate
  → 使用当前 point 更新前的 map 做 close/far partition
  → 当前 endpoint point evidence 写入 map，并保护 endpoint voxels
  → cold start ready 后，对 far components 做 floating BFS
  → floating component 生成 OBB-center detection
  → 分类完成后累加并提交 VALID_RETURN/NO_RETURN free-ray updates
  → 每 scan 执行 separate-background cleanup
  → 发布 detections、occupied background、free voxels 与 diagnostics
  → 独立 lidar_tracker_mid360 做 CA tracking
~~~

主要实现：

- [vofod_nodelet.cpp](../src/vofod_mid360/src/vofod_nodelet.cpp)
- [strict_baseline_core.cpp](../src/vofod_mid360/src/strict_baseline_core.cpp)
- [persistent_structure.cpp](../src/vofod_mid360/src/persistent_structure.cpp)
- [ray_update.cpp](../src/vofod_mid360/src/ray_update.cpp)
- [voxel_map.cpp](../src/vofod_mid360/src/voxel_map.cpp)
- [VoFOD 配置](../src/vofod_mid360/config/b0_mid360_canonical.yaml)
- [raycast 配置](../src/vofod_mid360/config/raycast_mid360.yaml)
- [tracker 配置](../src/lidar_tracker_mid360/config/tracking.yaml)

### 5.3 输入适配与 weighted voxel cloud

VoFOD adapter 检查：

- points/rays stamp 完全相同且严格单调；
- 两者均为 world frame；
- source_mode 满足 sensor config，实验为 sim_exact；
- bundle 恰好 20,000 rays；
- original_index 唯一且 VALID_RETURN endpoint 可一一对应；
- 点坐标与 origin + range × direction 的误差不超过 0.01 m；
- pattern mask、transform、return status 与 direction 合法。

当前仿真 body mask 开启但 blocked_pattern_indices 为空，因此不会屏蔽任何 pattern index。

合法 endpoint 以 0.50 m、与 voxel map 对齐的 VoxelGridWeighted 降采样。每个 voxel 输出一个代表点，其 PointXYZR.range 字段记录原始点数 \(n_i\)，后续 point update 会使用该计数而不是把每个代表点都视为相同强度。

降采样点用 PCL EuclideanClusterExtraction 聚类，tolerance 为 1.50 m。每个 component 计算 AABB 和 OBB；检测尺寸和位置均使用 OBB。

### 5.4 标量体素图

operation volume：

| 项目 | 当前值 |
|---|---:|
| center | (20, 0, 5) m |
| size | (60, 30, 16) m |
| origin | (−10, −15, −3) m |
| voxel size | 0.50 m |
| grid cells | 120 × 60 × 32 = 230,400 |

每个 voxel 只有一个 scalar score \(m_i\)。当前值与阈值顺序为：

\[
-1000=s_{\mathrm{free}}
<-750=\tau_{\mathrm{frontier}}
<-740=s_{\mathrm{init}}=s_{\mathrm{unknown}}
<-300=\tau_{\mathrm{new}}
<-0.1=\tau_{\mathrm{sure}}
<0=s_{\mathrm{occupied}}.
\]

解释：

- \(m_i>-0.1\)：sure occupied；
- \(m_i>-300\)：已知/新障碍背景；
- \(m_i\approx-740\)：unknown；
- \(m_i<-750\)：free 或 frontier-free。

这是单标量递推图，不单独保存占据概率与自由概率，也没有目标专用 identity layer。

### 5.5 10-scan cold start 与 persistent structure

hit_history 保存最多 lag+1=11 个 scans 的 occupied voxel sets。前 10 scans 会立即更新 map，但不启用 floating detection；第 11 个 scan 到来后 initialization_ready=true。

对每个当前 component：

1. 取它在当前 scan 覆盖的 voxel cells；
2. 以这组 cells 为种子，在 11-scan occupied union 上做 26 邻域连通；
3. 若区域接触 map 中已有 \(m_i>-300\) 的 background，则当前 component 直接 anchored/certified；
4. 否则只保留 hit scans ≥ 3 的 stable cells；
5. stable cells 必须至少 12 个；
6. PCA 主轴投影跨度必须至少 2.0 m；
7. 满足后，把当前 component 中 hit scans ≥ 3 的 cells 认证为 persistent background。

这个冷启动要求背景结构跨多帧稳定且大于目标 envelope，以减少直接把刚出现的小目标当作初始化背景的风险。

### 5.6 Close/far partition

close/far partition 使用“写入当前 point evidence 之前”的 map。只要 component 中任一点满足以下任一条件，整个 component 标为 background：

- 点所在 cell 属于当前 persistent certificate；
- 点的 0.90 m 邻域内存在 \(m_i>-300\) 的 map cell。

否则 component 暂时为 far/invalid，等待 floating 分类。这里是 any-point component veto：大型 component 只要一个点接近背景，整个 component 都会被吸收为背景。

### 5.7 Point evidence 更新

close component 的目标 score 为 0；其余 component 的目标 score 为 −740。对 weighted voxel 代表点 \(i\)，原始回波数为 \(n_i\)，

\[
\lambda_i=0.5^{n_i},
\]

\[
m_i^+=\lambda_i m_i^-+(1-\lambda_i)s_i,
\]

其中

\[
s_i=
\begin{cases}
0,&\text{close/background component},\\
-740,&\text{far/undetermined component}.
\end{cases}
\]

所有当前 endpoint voxels 同时被标为 protected，随后同 scan 的 free-ray update 不得把它们向 free 拉动。点 evidence 在 far floating BFS 之前写入，但 close/far 判定本身使用更新前 map。

### 5.8 Floating BFS

初始化完成后，只对 far components 继续检查。先应用：

\[
N_{\mathrm{weighted\ points}}\ge1,
\]

\[
\mathrm{OBB\ diagonal}\le1.50\ \mathrm{m},
\]

\[
\lVert c_{\mathrm{OBB}}-p_{\mathrm{observer}}\rVert\le40\ \mathrm{m}.
\]

搜索最大 voxel Manhattan radius 为

\[
d_{\max}=
\max\left(1,
\left\lfloor
\frac{\mathrm{OBB\ diagonal}+3.0}{0.50}
\right\rfloor\right).
\]

从 component 每个点所在 voxel 开始，exploreToGround 在 \(m_i>-750\) 的未知区域内进行 6 邻域 DFS。如果搜索：

- 遇到 \(m_i>-300\) 的背景；
- 到达 map 边界；
- 到达局部搜索边界；

则 component 被视为连接背景，分类为 unknown。只有所有点的搜索都没有连接到背景/边界时，component 才是 floating，并输出 detection。

因此 VoFOD 的核心问题是“是否能经非自由/未知空间连接到背景”；AeroCOVER 的核心问题是“候选周围是否被历史可信自由 chord 全方向包围”。两者不是同一个自由空间判据。

### 5.9 Detection 输出

floating component 的位置是 OBB center。检测 covariance 对角线当前直接设置为

\[
\operatorname{diag}(R_{\mathrm{det}})
=0.10\sqrt{r_{\mathrm{observer}}}.
\]

这里的值被写入 covariance 数组，而不是标准差字段。detection_probability 固定为 0.5。

置信度来自局部 submap。把 component cells 临时置为 free score 后，按

\[
u=\frac1N\sum_j\left(1-\frac{m_j}{-1000}\right),\qquad
\mathrm{confidence}=\operatorname{clamp}(e^{-u},0,1)
\]

计算；它不参与本文 evaluator 的真值关联门限。

### 5.10 Free-ray map update

每条合法 ray 先用精确 voxel DDA 得到各 cell 内的 path length。对 voxel \(i\)，记 VALID_RETURN 与 NO_RETURN 的总路径长度为 \(L_{i,v}\)、\(L_{i,n}\)，voxel diagonal 为

\[
d_{\mathrm{vox}}=\sqrt3\cdot0.50.
\]

归一化 exposure：

\[
E_i=
\frac{0.003L_{i,v}+0.003L_{i,n}}{d_{\mathrm{vox}}}.
\]

retain：

\[
\lambda_i=2^{-E_i}.
\]

非 protected voxel 更新为

\[
m_i^+=\lambda_i m_i^-+(1-\lambda_i)(-1000).
\]

VALID_RETURN 的 DDA 终点为 min(range−0.50 m, 20 m)；NO_RETURN 忽略消息中的 range 字段，使用 min(20 m, reliable range 20 m)。所有 rays 先按确定性顺序排序再累积，最后同步 commit，因此结果不依赖回调内部输入顺序。

### 5.11 Separate-background cleanup

每个 scan 在 ray update 后执行一次：

1. 取所有 \(m_i>-300\) 的 occupied voxels；
2. 以 0.8 m 尺度降采样并做 Euclidean clustering；
3. 统计每个 structure cluster 内 \(m_i>-0.1\) 的 sure voxels；
4. 只要场景中存在至少一个含 24 个 sure voxels 的强结构；
5. 对其余小型分离 occupied clusters 周围的 voxels 执行

\[
m_i^+=0.5m_i+0.5(-1000).
\]

这一规则试图消除与大背景结构分离的小型残留占据块，但在稀疏森林中，树干、枝叶和扫描碎片的连通关系仍可能产生大量 floating detections。

### 5.12 外部 lidar_tracker_mid360

VoFOD detection 进入独立 tracker。状态同样是 9 状态 CA：

\[
x=[p_x,p_y,p_z,v_x,v_y,v_z,a_x,a_y,a_z]^\top,
\]

转移矩阵与标准 CA 形式相同。当前配置：

| 参数 | 当前值 |
|---|---:|
| point-cloud buffer | 10 scans |
| minimum onboard detections for confirmed output | 2 |
| Q position / velocity / acceleration variance rate | 0.01 / 0.8 / 0.05 |
| initial P position / velocity / acceleration | 0.3 / 1.0 / 0.05 |
| measurement R coefficient | 0.1 |
| uncertainty radius multiplier | 1.5 |
| minimum search radius | 2.5 m |
| deletion raw uncertainty radius | 5.0 m |
| local cluster tolerance | 1.0 m |
| local OBB max diagonal | 1.0 m |
| minimum background distance | 1.0 m |

位置 covariance 的等体积半径为

\[
r_P=1.5\sqrt{\sqrt[3]{\det(P_{pos})}}.
\]

搜索半径取 \(\max(r_P,2.5)\)。一个 detection 被构造成 tentative track，并 replay 通过最多 10 个已缓存点云帧；每帧在预测半径内提取点，再做 1.0 m local clustering、OBB≤1.0 m 和 occupied-background distance filter，选择离预测最近的 cluster 修正状态。

tentative track 与已有 track 的关联是 greedy nearest：

\[
\lVert p_i-p_j\rVert<r_i+r_j.
\]

由于每侧半径最低 2.5 m，两个最小半径假设的关联门可达到 5 m。重叠 raw uncertainty spheres 的 tracks 会合并，并保留不确定性较小者。raw radius >5 m 时删除。

这套 tracker 没有 GNN/JPDA、显式 visibility、scan opportunity、existence probability 或 re-identification。

### 5.13 当前 VoFOD 参数表

| 模块 | 参数 | 当前值 | 作用 |
|---|---|---:|---|
| Input | sync queue | 32 | ExactTime 同步 |
| Input | expected rays | 20,000 | attempted-ray 合约 |
| Input | require expected count | true | ray 数不符则拒帧 |
| Input | endpoint tolerance | 0.01 m | 点/ray 一致性 |
| Operation area | center / size | (20,0,5) / (60,30,16) m | 固定 map 空间 |
| Map | voxel size | 0.50 m | scalar grid 分辨率 |
| Cluster | tolerance | 1.50 m | 当前 weighted points 聚类 |
| Cluster | minimum weighted points | 1 | 保留 singleton component |
| Cluster | max OBB diagonal | 1.50 m | floating 候选尺寸 |
| Cluster | max range | 40 m | 检测范围 |
| Close | background distance | 0.90 m | any-point 背景邻近门 |
| Explore | extension | 3.0 m | OBB 外 BFS 搜索距离 |
| Cold start | lag | 10 scans | 第 11 scan 起检测 |
| Persistence | scans/cells/extent | 3 / 12 / 2.0 m | 初始背景认证 |
| Map scores | occupied/unknown/free | 0 / −740 / −1000 | 标量状态目标 |
| Thresholds | sure/new/frontier | −0.1 / −300 / −750 | 背景与自由判定 |
| Rays | valid/no-return weight | 0.003 / 0.003 | free update exposure |
| Rays | margin/max/reliable no-return | 0.5 / 20 / 20 m | 可信自由段 |
| Cleanup | distance/sure count | 0.8 m / 24 | 分离背景清除 |
| Output | position sigma coefficient | 0.10 | covariance 对角线的距离缩放系数 |
| Detection | probability | 0.5 | 固定输出值 |
| Visualization | every N scans / free voxels | 5 / true | RViz map 发布节流 |
| Profiling | warning threshold | 100 ms | 单 scan 超时告警 |

dynamic_aware_free_weighting=false、delayed_map_commit=false；两项非 baseline 变体在当前配置中明确关闭。

### 5.14 ROS 输出与 pipeline 边界

| Topic | 内容 |
|---|---|
| /uav1/vofod_mid360/detections | floating OBB detections |
| /uav1/vofod_mid360/background_points | sure occupied background；tracker 的背景输入 |
| /uav1/vofod_mid360/free_voxels | free-score 可视化；绝不作为 occupied background 输入 |
| /uav1/vofod_mid360/map_update_diagnostics | ray/point/map/classification/runtime 统计 |
| /uav1/vofod_mid360/map_revision_evidence | 每次同步 commit 后的全图审计 bitsets |
| /uav1/vofod_mid360/status | initialization/detection 状态 |
| /uav1/batch/b0/tracks | 外部 lidar_tracker_mid360 最终 tracks |

唯一 canonical launcher 是 [b0_canonical.launch](../src/tclv_evaluation/launch/b0_canonical.launch)。它同时启动 detector 与 tracker，并且只把 occupied background_points 接入 tracker；free_voxels 不参与背景过滤。

## 6. 评估协议

### 6.1 同源 replay

P01/P02 各录制一次 source bag，两个算法从相同 bag replay。算法输出与 source truth 分离记录，再由同一个 [evaluate_bag.py](../src/soft_vofod_evaluation/scripts/evaluate_bag.py) 评分。

当前四个 runs 均满足：

- manifest status = COMPLETE；
- source truth coverage = 100%；
- track frame coverage = 100%；
- timing frame coverage = 100%；
- 主评估门限 = 1.0 m；
- 另保存 0.5 m 与 2.0 m sensitivity；
- GPU 未使用。

AeroCOVER replay rate 为 P01 0.60×、P02 0.50×；VoFOD 两者均为 0.50×。这些倍速用于让 replay 不因计算积压而丢帧；runtime 仍是节点内部每 scan 的 steady-clock 计算时间，不应把 wall replay 时长当成算法 runtime。

### 6.2 Truth inclusion

主 track-set score 对每帧只保留：

\[
\mathrm{present}\land\mathrm{in\_range}\land\mathrm{in\_fov}.
\]

主 score 当前没有把 line_of_sight 加入 truth inclusion。若 prediction 位于被排除 truth 的 1 m 内，该 prediction 也从 FP 计算中移除。

TTFT 的“first visible”定义更严格：

\[
\mathrm{present}\land\mathrm{in\_range}\land\mathrm{in\_fov}
\land\mathrm{LOS}\land\mathrm{actual\ returns}>0.
\]

因此 TTFT 与主 TP/FN 的 visibility 边界并不完全相同。本文所有 TTFT 均明确指 from first visible。

### 6.3 每帧关联和指标

每帧用 truth/prediction 的三维位置距离矩阵做 Hungarian，一对一匹配，并丢弃距离 >1 m 的 pair。

\[
\mathrm{Precision}=\frac{TP}{TP+FP},\qquad
\mathrm{Recall}=\frac{TP}{TP+FN}.
\]

当前 evaluator 的 detection accuracy：

\[
\mathrm{DetA}=\frac{TP}{TP+FP+FN}.
\]

对 truth-track pair 的全序列共同匹配数 \(n_{ij}\)，association accuracy 按当前实现聚合为

\[
\mathrm{AssA}=
\frac1{TP}\sum_{ij}
\frac{n_{ij}^2}
{n_{i\cdot}+n_{\cdot j}-n_{ij}}.
\]

报告的 HOTA 为单个 1 m localization threshold 下

\[
\mathrm{HOTA}=\sqrt{\mathrm{DetA}\cdot\mathrm{AssA}}.
\]

它不是官方 HOTA 常见的多 localization thresholds 积分/平均结果；论文中引用时必须称为当前 evaluator 的 1 m HOTA-style score，或把 evaluator 扩展到标准协议。

IDF1 使用全序列 truth ID 与 predicted ID 共同帧数矩阵，再做全局 Hungarian identity assignment。Position RMSE 和 velocity RMSE 只在成功匹配 pair 上计算。

ID switch：同一 truth 的连续成功匹配从一个 prediction ID 变为另一个。Fragmentation：truth 已经匹配后进入 gap，随后再次匹配。若算法完全没有 TP，IDSW=0、fragmentation=0 只是没有可形成 identity event，不能解释为跟踪稳定。

### 6.4 Birth 与 runtime

Birth 根据 track 第一次出现和 truth proximity 区分 matched/unmatched。AeroCOVER 还从 diagnostics 读取 shell candidate 和 birth provenance。

AeroCOVER runtime：

- diagnostics.total_ms = ROS 输入转换 input_ms + core processScan total；
- 包含其内置 CA tracker；
- peak RSS 来自整个算法进程。

VoFOD runtime：

- 来自 vofod_mid360 detector 的 MapUpdateDiagnostics.total_ms；
- 包含 detector 输入转换、点处理、分类、ray update 与 cleanup；
- 不包含独立 lidar_tracker_mid360；
- 因而仅可比较 detector 计算量，不能称为 VoFOD 完整 pipeline 的端到端 runtime。

## 7. 当前实验结果

### 7.1 Tracking accuracy

| Algorithm | Scene | HOTA | DetA | AssA | IDF1 | Precision | Recall |
|---|---|---:|---:|---:|---:|---:|---:|
| AeroCOVER | P01 | 0.9826 | 0.9655 | 1.0000 | 0.9825 | 1.0000 | 0.9655 |
| AeroCOVER | P02 | 0.9633 | 0.9279 | 1.0000 | 0.9626 | 1.0000 | 0.9279 |
| VoFOD | P01 | 0.2295 | 0.0742 | 0.7093 | 0.1138 | 0.0953 | 0.2512 |
| VoFOD | P02 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 |

| Algorithm | Scene | TP | FP | FN | Position RMSE | Velocity RMSE | TTFT |
|---|---|---:|---:|---:|---:|---:|---:|
| AeroCOVER | P01 | 196 | 0 | 7 | 0.0755 m | 0.4256 m/s | 0.8 s |
| AeroCOVER | P02 | 103 | 0 | 8 | 0.0755 m | 0.9156 m/s | 0.9 s |
| VoFOD | P01 | 51 | 484 | 152 | 0.3780 m | 0.9360 m/s | 3.9 s |
| VoFOD | P02 | 0 | 633 | 111 | N/A | N/A | N/A |

P02 VoFOD 的 RMSE 为 N/A，不是 0：因为没有任何成功 truth-track pair。

#### Localization-threshold sensitivity

| Algorithm | Scene | Threshold | TP / FP / FN | HOTA | Recall |
|---|---|---:|---:|---:|---:|
| AeroCOVER | P01 | 0.5 / 1.0 / 2.0 m | 196 / 0 / 7（全部相同） | 0.9826（全部相同） | 0.9655 |
| AeroCOVER | P02 | 0.5 / 1.0 / 2.0 m | 103 / 0 / 8（全部相同） | 0.9633（全部相同） | 0.9279 |
| VoFOD | P01 | 0.5 m | 42 / 493 / 161 | 0.2184 | 0.2069 |
| VoFOD | P01 | 1.0 m | 51 / 484 / 152 | 0.2295 | 0.2512 |
| VoFOD | P01 | 2.0 m | 66 / 469 / 137 | 0.2234 | 0.3251 |
| VoFOD | P02 | 0.5 / 1.0 / 2.0 m | 0 / 633 / 111（全部相同） | 0 | 0 |

AeroCOVER 在 0.5 m 门限下指标完全不变，说明所有已匹配位置误差都低于 0.5 m。VoFOD P01 把门限放宽到 2 m 只能增加部分 TP，仍无法解决大量 FP 和 identity association 问题；P02 在 2 m 下仍无 TP。

### 7.2 Identity 与生命周期

| Algorithm | Scene | Total births | Matched births | False births | Unique confirmed tracks | IDSW | Frag |
|---|---|---:|---:|---:|---:|---:|---:|
| AeroCOVER | P01 | 1 | 1 | 0 | 1 | 0 | 0 |
| AeroCOVER | P02 | 1 | 1 | 0 | 1 | 0 | 0 |
| VoFOD | P01 | 10 | 2 | 8 | 10 | 1 | 4 |
| VoFOD | P02 | 14 | 0 | 14 | 14 | 0 | 0 |

补充：

- AeroCOVER P01/P02 最长 measurement-update gap 为 0.20/0.50 s，track 没有删除或重生；
- VoFOD P01 最长 scored tracking gap 为 11.5 s；
- VoFOD P01 false births 约 22.02/min；
- VoFOD P02 false births 70.0/min；
- P02 VoFOD 在结束时仍有 7 个 confirmed tracks，峰值为 10，但全都未匹配真实目标。

### 7.3 合并结果

| Algorithm | TP / FP / FN | Micro precision | Micro recall | Macro HOTA | Macro IDF1 |
|---|---:|---:|---:|---:|---:|
| AeroCOVER | 299 / 0 / 15 | 1.0000 | 0.9522 | 0.9729 | 0.9725 |
| VoFOD | 51 / 1117 / 263 | 0.0437 | 0.1624 | 0.1147 | 0.0569 |

Macro 是两个场景等权平均；micro precision/recall 先汇总 TP/FP/FN。单一 seed 下不报告方差或置信区间。

### 7.4 AeroCOVER shell/candidate 机制

| Scene | Candidate observations | Target-labeled | Background-labeled | Target supported bins | Birth NO_RETURN fraction |
|---|---:|---:|---:|---:|---:|
| P01 | 20 | 16 | 4 | 42/42（全部 16 次） | 0.0797 |
| P02 | 21 | 16 | 5 | 42/42（全部 16 次） | 0.00183 |

P01 的 4 次 background-labeled candidate observations 也达到 42/42；P02 background-labeled observations 的 supported bins 均值为 40.2、最大 42。这说明：

1. 42-bin shell 是强局部几何证据，但不是单独保证 0 FP 的充分条件；
2. 3-of-5 时序确认、candidate 空间关联与背景传播共同阻止了这些瞬时背景候选 birth；
3. 两次真实 birth 的 NO_RETURN 贡献比例都很低，当前成功主要由 VALID_RETURN 自由段支撑，而不是依赖大量弱 NO_RETURN。

### 7.5 Runtime 与内存

| Algorithm | Scene | Mean | Median | p95 | p99 | Max | Peak RSS |
|---|---|---:|---:|---:|---:|---:|---:|
| AeroCOVER | P01 | 43.436 ms | 42.561 | 55.004 | 63.616 | 81.638 | 176,928 KiB |
| AeroCOVER | P02 | 56.624 ms | 56.831 | 69.502 | 74.056 | 78.171 | 181,092 KiB |
| VoFOD detector | P01 | 28.687 ms | 29.228 | 38.622 | 41.999 | 45.493 | 79,244 KiB |
| VoFOD detector | P02 | 29.274 ms | 29.661 | 36.861 | 38.215 | 41.898 | 79,780 KiB |

两场景各算法 runtime samples 都是 P01 218、P02 120。加权平均：

| Algorithm | Weighted mean | 10 Hz processing load |
|---|---:|---|
| AeroCOVER complete node | 48.118 ms/scan | 48.1% |
| VoFOD detector only | 28.896 ms/scan | 28.9% |

表面上 AeroCOVER 平均是 VoFOD detector 的 1.665×；但 VoFOD 缺少 tracker runtime，不能把该比率解释为完整系统 slowdown。两者在当前机器和降速 replay 下 max 都低于 100 ms。

实验机器：

~~~text
uav-ThinkStation-P360-Tower
Linux 5.15, x86_64
24 logical CPUs
RAM 33,348,501,504 bytes
GPU not used
ROS Noetic
Release build
~~~

### 7.6 AeroCOVER runtime 分解

下表为每 scan mean；父项与子项有包含关系，不能直接逐列相加。

| Module | P01 | P02 | 解释 |
|---|---:|---:|---|
| input conversion | 1.909 ms | 1.912 ms | ROS PointCloud2/ray 转换与一致性检查 |
| validation | 0.665 | 0.682 | core 输入与因果约束 |
| expiry | 4.513 | 5.075 | 旧 ray/scan 贡献撤销与 FIFO 删除 |
| clustering total | 16.375 | 21.484 | grid + union + temporal |
| └ grid build | 6.668 | 8.486 | 一秒点历史哈希网格 |
| └ neighbor union | 7.507 | 10.807 | 精确连接与 union-find |
| └ temporal slice | 0.469 | 0.575 | 背景传播和逐 scan slice |
| evidence total | 16.845 | 24.452 | shell、关联、rebuild、ray insertion 等 |
| └ shell prefilter | 6.008 | 13.133 | observations 的 42-bin pass-only gate |
| └ evidence rebuild | 0.193 | 0.342 | candidate 几何变化后的历史重建 |
| └ evidence summary | 0.0005 | 0.0010 | accumulator 汇总 |
| └ ray insertion | 10.632 | 10.964 | 20,000 rays 的 scan index 构建与增量贡献 |
| born-track association | 0.010 | 0.010 | CA gating + Hungarian |

P01/P02 complexity：

| Quantity per scan | P01 mean | P02 mean |
|---|---:|---:|
| Current return points | 8,603.8 | 8,633.3 |
| FIFO points | 84,729.9 | 83,474.5 |
| ST components | 161.9 | 980.7 |
| Temporal slices checked | 21.1 | 94.6 |
| Background ST components | 23.9 | 62.4 |
| Residual points | 87.0 | 248.4 |
| Strict target observations | 0.794 | 0.800 |
| Active candidates | 0.211 | 0.433 |
| FIFO rays | 200,001.0 | 200,001.0 |

P02 point count与 P01 接近，但时空 components 是约 6.1×、residual points 是约 2.9×。森林中的离散树干回波和碎片化空间连通使 neighbor-union 与 shell-prefilter 都更贵；这解释了 P02 比 P01 多出的约 13.2 ms mean runtime。Candidate rebuild 并不是当前主因。

### 7.7 VoFOD detector runtime 分解

| Module mean | P01 | P02 |
|---|---:|---:|
| point update path | 11.742 ms | 23.123 ms |
| classification | 1.291 | 0.981 |
| ray accumulation | 10.965 | 0.381 |
| ray apply | 0.268 | 0.0004 |
| separate background | 1.650 | 0.568 |
| detector total | 28.687 | 29.274 |

point update path 的计时从 weighted voxel filtering 开始，因此包含降采样、1.5 m clustering、cold-start/close partition 与 point evidence，不只是最后一个 map 写操作。

P01 office 中较长的有效/无返回自由路径使 ray DDA accumulation 显著；P02 forest 回波丰富、ray 更早终止，ray accumulation 很低，但点端降采样与 clustering 成为主项。两种场景的总 detector runtime 因此接近，但瓶颈不同。

### 7.8 辅助 map diagnostics

| Algorithm | Scene | Final background voxels | Approx. static-bg recall | Target contamination | Certified-free precision |
|---|---|---:|---:|---:|---:|
| AeroCOVER | P01 | 480 | 0.0315 | 0.1865 | N/A |
| AeroCOVER | P02 | 999 | 0.0285 | 0.0418 | N/A |
| VoFOD | P01 | 778 | 0.0362 | 0.1493 | 0.9842 |
| VoFOD | P02 | 137 | 0.0042 | 0.0000 | 0.6516 |

AeroCOVER 不发布持久 free-voxel map，所以 certified-free precision 为 N/A。两种算法的 background 输出语义也不同：AeroCOVER 是当前点的传播背景标签，VoFOD 是 scalar map 的 sure occupied voxels。该表只能作故障诊断，不能作为两算法地图质量的直接排名。

## 8. 结果分析

### 8.1 AeroCOVER 为什么在 P01/P02 上降低了 FP

效果来自四层组合，而不是某一个阈值：

1. 单 scan 大 slice 背景规则先剥离墙面、地面和树干连续结构；
2. 3 点/20% 历史标签传播使已识别背景在下一帧快速稳定；
3. full-chord shell 要求候选外围真实被历史自由段穿过，墙面自身 endpoint 不能自证；
4. strict 42/42 + 3-of-5 把瞬时 shell pass 与可 birth 的对象级时序一致性分开。

实验直接显示 shell 单独并非充分条件：P01 仍有 4 次 background-labeled 42/42 observations，但没有形成 false birth。因而当前可支持的创新表述应是“时空背景排除 + 因果 full-chord directional shell + 分离 birth/maintenance 的对象级确认”，不应简化成“42 bins 就能消除误报”。

### 8.2 AeroCOVER 的 FN 来源

P02 的 8 个 FN 全部位于评分开头 23.84–24.54 s，符合严格 shell evidence 与 3-of-5 birth 带来的确认延迟；评分段内 history_ready 已经为 true，不能把这 8 帧简单归因于“FIFO 尚未填满”。P01 的 7 个 scored misses 也都在第一次确认前；其中一个 truth sample 的 LOS=false，但因主 score 不以 LOS 过滤，仍计入 FN。

Birth 后：

- P01 最长 measurement gap 0.20 s；
- P02 最长 measurement gap 0.50 s；
- CA prediction 与 S35 maintenance 保持了同一 ID；
- 没有 post-birth fragmentation、ID switch 或 false deletion。

因此这两个场景中的 recall 损失主要是严格 cold-start/birth latency，而不是后段跟踪断裂。

### 8.3 VoFOD P01 失败模式

P01 的 target 紧贴窄走廊墙面。1.50 m Euclidean clustering 和 any-point 0.90 m close-background veto 容易把目标回波与墙面回波桥接成同一 component；只要其中一个点接近已知背景，整个 component 就被当作 background，造成大量 FN。

另一方面，墙角、门框或局部 map 尚未与主背景连通的碎片可能通过 floating BFS，生成 false detections。外部 tracker 只需 2 次 onboard detections 即可确认，且 detection-to-track 最小联合 gate 可达 5 m，因此这些碎片较容易成为持续 false tracks。

结果与该机制一致：

- recall 仅 0.2512；
- 484 FP；
- 10 births 中 8 个 false；
- 真实目标跨 2 个 track IDs，产生 1 IDSW、4 fragments；
- 最长 tracking gap 11.5 s。

这是基于源码路径与汇总结果的根因推断；要证明每个具体 false track 来自哪一处墙角，需要对 detection component、map score 与真值距离做逐帧 trace。

### 8.4 VoFOD P02 失败模式

P02 的 forest 是大量分离的细长圆柱结构。稀疏扫描下，树干回波可能：

- 没有达到 3 scans / 12 cells / 2 m persistent structure certificate；
- 与已有 \(m>-300\) 背景在 0.90 m 内不连通；
- 被周围 free voxels 截断，使 BFS 找不到背景或搜索边界；
- 因 OBB diagonal≤1.5 m 而满足 floating target 尺寸。

这样，静态树干碎片会被视为 floating。与此同时，真实目标在近树飞行时又可能被 1.50 m clustering 与 any-point close veto 吸收到树干 component 中。

结果是：

- 真实目标 0 TP / 111 FN；
- 633 FP；
- 14 births 全为 false；
- 结束仍有 7 个 confirmed false tracks。

辅助 map metric 中，P02 VoFOD final background voxels 仅 137，近似 static-background recall 为 0.0042，certified-free precision 为 0.6516。这些 map 指标依赖 evaluator 的简化静态 primitive 表示，不能视为完整地图精度，但与“背景认证不足、碎片被判 floating”的方向一致。

### 8.5 为什么 VoFOD 更快但结果更差

VoFOD detector：

- 固定 230,400-cell scalar map；
- 当前帧 weighted cloud 聚类；
- 不保存 1 s 原始点连接图；
- 不为每个 observation 查询 42-bin historical shell；
- 不在 detector runtime 中计外部 tracker。

AeroCOVER：

- 每帧对约 8–13 万历史+当前点建立精确连通；
- 维护约 20 万 rays 的一秒 FIFO 与 per-scan spatial index；
- 为残余 observation 做精确 ray–sphere–shell；
- runtime 包含内部 tracker。

因此 AeroCOVER 当前高约 19.2 ms/scan 的加权平均差异有明确结构来源。它用更高计算量换取本地几何排除能力；现有数据不能证明这种差异在加入 VoFOD tracker runtime、更多场景或真实硬件后仍保持同一比例。

### 8.6 当前最值得优化的 AeroCOVER 路径

按当前 profile，优先级应是：

1. P02 exact connectivity 的 grid build 与 neighbor union；
2. shell prefilter 的 observation 数量和 spatial query；
3. 固定每 scan 20,000 rays 的 spatial-index insertion；
4. FIFO expiry。

不应优先继续优化：

- born-track Hungarian，约 0.01 ms；
- evidence summary，约 0.001 ms；
- candidate rebuild，平均 <0.35 ms；
- ROS input conversion，约 1.91 ms。

进一步性能工作必须先保持五类评价 CSV 与 tracking metrics 不变，再比较同源 timing；否则就属于算法消融而不是纯性能优化。

## 9. 当前实现的研究贡献与证据边界

当前 AeroCOVER 相对于 VoFOD baseline 的核心技术组合是：

1. 在 rolling Mid-360 attempted-ray stream 上保持严格因果的历史自由空间证据；
2. 用 target-adaptive robust sphere 与 full-chord shell，把“目标周围自由”离散成 42 个方向的跨 scan 密度；
3. 用逐 scan 时间 slice 的尺寸，而不是一秒移动轨迹总尺寸，区分持续小型运动对象与大背景；
4. 将 strict birth gate 与 prediction-conditioned maintenance gate 分离；
5. 把检测与 9-state CA tracking 放在同一因果状态机中。

P01/P02 支持“该组合可在狭窄走廊和稀疏森林两个开发场景中实现低 FP、连续单 ID tracking”。尚未支持：

- shell 42、full chord、时空背景与 3-of-5 各自的独立贡献量；
- 对未见场景、不同树密度、不同建筑结构的泛化；
- 多目标近距离交叉时的 identity 稳定性；
- 长 LOS 遮挡后的重捕获；
- 真实 Mid-360 的 multipath、dust、rain、body return 与 calibration error；
- 相对标准 HOTA benchmark 或公开真实数据集的可比较结论。

如果用于论文，下一阶段最小必要证据是：

- 冻结当前参数，不再用 test scenes 调参；
- 新增 held-out maps 和每图多个随机 seeds；
- 做 full-chord、42/42、3-of-5、S35、ST-background 的最小正交消融；
- 增加多目标、长遮挡、range bins 与 sensor noise；
- 加入真实 hardware sequence；
- 将 evaluator 的 HOTA 改为标准多阈值协议；
- 端到端统计 VoFOD detector + tracker runtime；
- 报告 mean、median、置信区间与失败案例。

## 10. 可复现性与当前验证

### 10.1 当前结果目录

AeroCOVER：

- [AeroCOVER summary](../results/aerocover_s35_c030_ray_blocks_scan_saturation_p01_p02_1seed/SUMMARY.md)
- [AeroCOVER aggregate CSV](../results/aerocover_s35_c030_ray_blocks_scan_saturation_p01_p02_1seed/aggregated/aggregate.csv)

VoFOD 与 immutable sources：

- [VoFOD summary](../results/aerocover_vofod_p01_p02_1seed/SUMMARY.md)
- [VoFOD aggregate CSV](../results/aerocover_vofod_p01_p02_1seed/aggregated/aggregate.csv)

每个 run 目录还包含 metrics.json、per_target_metrics.csv、tracking_timeline.csv、diagnostics_timeseries.csv、timing.csv、resource.txt、algorithm.log 和 run_manifest.json。AeroCOVER 另含 candidate/birth CSV。

### 10.2 Provenance hashes

| Artifact | SHA-256 |
|---|---|
| AeroCOVER algorithm tree | 9cf8cc9f988e6dcd1c5bda0f4350458b6d2150b7d2c90ed097472320a1ab49af |
| AeroCOVER binary | d51a4b8bfbc9c313b813b1032fec19caf8b9d73aa8a35418d3641e00212c63d4 |
| AeroCOVER config | bed66d3a419d0336ec57ab7f7f470c94e1f8dd2f1b6330072b922f4638699176 |
| VoFOD algorithm tree | 3079404f8fd80594131306379234e8e650a023f4ef5028d7a8ab1d76640ffc80 |
| VoFOD binary | e16067cc07b1a0cce77a302364fc397d9bbc4fc18bb566f4372064333c01df65 |
| VoFOD primary config | 5cfcf343bd39c82b9e5d75af94eb694e06eb7ae46f9d3fd15f1bd879d7f96863 |
| Common evaluator | 2cdd4f2447464424b8b37908a644af2a103595702ab82a68f2a0cad82ff5f93c |

Run manifests 记录 base git commit e3cac13c49c400fab892d9392004b5b3b9ea1013 且 git_dirty=true；因此复现身份应以 algorithm tree、binary、config、source 和 evaluator hashes 的组合为准，不能只引用 git commit。

### 10.3 Build 与 tests

清理后工作区保留 10 个 ROS packages。当前完整 Release build 成功，测试结果：

~~~text
Summary: 219 tests, 0 errors, 0 failures, 0 skipped
~~~

复查命令：

~~~bash
cd /home/uav/lyk
source /opt/ros/noetic/setup.bash
catkin build -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
catkin run_tests
catkin_test_results build
~~~

### 10.4 同源 replay 命令

AeroCOVER P01：

~~~bash
cd /home/uav/lyk
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/soft_vofod_evaluation/scripts/run_benchmark.py replay \
  --source results/aerocover_vofod_p01_p02_1seed/sources/P01/N0/source.bag \
  --scenario results/aerocover_vofod_p01_p02_1seed/sources/P01/N0/scenario.yaml \
  --algorithm AeroCOVER \
  --config src/aerocover_mid360/config/aerocover_mvp.yaml \
  --output results/repro/P01/AeroCOVER \
  --replay-rate 0.60 --require-source-manifest --cleanup-output-bag
~~~

AeroCOVER P02：把两处 P01 改为 P02，并把 replay-rate 改为 0.50。

VoFOD P01：

~~~bash
cd /home/uav/lyk
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/soft_vofod_evaluation/scripts/run_benchmark.py replay \
  --source results/aerocover_vofod_p01_p02_1seed/sources/P01/N0/source.bag \
  --scenario results/aerocover_vofod_p01_p02_1seed/sources/P01/N0/scenario.yaml \
  --algorithm VOFOD \
  --config src/vofod_mid360/config/b0_mid360_canonical.yaml \
  --output results/repro/P01/VOFOD \
  --replay-rate 0.50 --require-source-manifest --cleanup-output-bag
~~~

VoFOD P02：把两处 P01 改为 P02。

### 10.5 GUI 场景复查

打开 P01、Gazebo 和 RViz，并在初始 waypoint 保持：

~~~bash
cd /home/uav/lyk
source /opt/ros/noetic/setup.bash
source devel/setup.bash
./devel/lib/mid360_multi_uav_sim/view_scenario.py \
  P01 --rviz --wait-for-start
~~~

准备好录屏后开始：

~~~bash
rosservice call /benchmark_scenario/start "{}"
~~~

P02 只需把 P01 改为 P02。若要在线显示算法输出，在另一个已 source 环境的终端、按需要启动其中一个：

~~~bash
roslaunch aerocover_mid360 aerocover_mvp.launch
~~~

或：

~~~bash
roslaunch tclv_evaluation b0_canonical.launch
~~~

不要同时让两个算法向同一输出命名空间发布；需要并排在线展示时应先显式 remap namespace。

## 11. 最终判断

在当前 P01/P02 同源实验内，AeroCOVER 明显优于受控 VoFOD-Mid360 baseline：它以约 48.1 ms/scan 的当前计算量，实现了 0 FP、0 false birth 和 0 identity break，并把主要误差压缩到严格 birth 造成的启动 FN。

当前最重要的客观保留意见有三点：

1. 只有两个参与开发的单目标、单 seed 仿真场景，不能据此宣称泛化；
2. AeroCOVER 的低 FP 是 shell、时空背景与 3-of-5 的组合结果，不能归因于 42/42 单一阈值；
3. VoFOD 的 28.9 ms/scan 不含 tracker，runtime 不是完整端到端公平比较。

因此，当前实现已经是一个清晰、可复现、机制有效的研究原型；下一步的价值不在继续针对 P01/P02 调参，而在冻结实现、补 held-out/real-world evidence，并使用标准化评估协议验证贡献。
