# Codex 后续修正与改进提示词：SOFT-VoFOD V2 重构、B0 公平性修复与 S07 根因闭环

## 0. 角色与任务目标

你正在 `/home/uav/lyk` 工作区中继续开发 ROS1 Noetic / catkin 工程。

当前已有：

```text
Mid360 raw points + complete emitted rays
 -> mid360_ray_preprocessor
 -> B0: vofod_mid360 + lidar_tracker_mid360
 -> A1/A2/A3: soft_vofod_mid360
 -> soft_vofod_evaluation
```

当前开发矩阵已经跑通 `N0 ideal × seed 1001 × S01–S07 × B0/A1/A2/A3`，但结果暴露出明确结构性问题：

- A1/A2/A3 总体显著差于 B0；
- S07/A3 出现 event storm、track storm、support storm 与 CPU 爆炸；
- proposed map 的 false-free 偏高、static-background recall 偏低；
- endpoint-level event 粒度过细；
- birth 后重复轨迹与 identity churn 严重；
- A3 将 opportunity、feedback、Hungarian 捆绑，无法严格归因；
- `P_D=0` 时 existence 完全冻结，而缺少 survival decay，导致 ghost tracks 长期存活；
- per-event spherical support 会反向阻碍背景恢复，构成正反馈；
- B0/S06 replay 存在 warm-up 输入握手缺口；
- target-free warm-up 作为 benchmark 控制条件可以保留，但不应继续作为 proposed method 的必要工作前提；
- 当前地图几乎在逐射线尺度积累 free/background evidence，这与背景本应低频、空间持久的统计属性不匹配。

本轮任务不是继续在现有 A3 上做局部调参，而是将 SOFT-VoFOD 重构为 **V2**：

> **逐射线只负责精确观测机会和几何；短时窗口负责 event packet；低频空间同化负责 background map；轨迹负责动态目标；只有达到轨迹级置信度后，目标才拥有有限的地图保护权。**

核心必须保持：

$$
\boxed{\text{背景与目标表示解耦}}
$$

$$
\boxed{\text{目标由历史自由空间违例 + 轨迹一致性确认}}
$$

$$
\boxed{\text{只有真实扫描机会存在时，未检测才是负证据}}
$$

但必须删除/修正当前 V1 中的错误实现：

$$
\boxed{\text{逐 endpoint 独立强 event}}
$$

$$
\boxed{\text{逐 ray 线性累积长期地图证据}}
$$

$$
\boxed{\text{unconfirmed event 直接生成大范围 map support}}
$$

$$
\boxed{P_D=0 \Rightarrow \text{轨迹无限保活}}
$$

---

# 1. 开始前必须完成的审计与冻结

首先阅读：

```text
/home/uav/lyk/SOFT_VOFOD_EXPERIMENT_REPORT.md
/home/uav/lyk/CURRENT_MRS_VOFOD_MID360_IMPLEMENTATION.md
```

并对照源码、launch、YAML、bag manifests、metrics、diagnostics。

生成：

```text
docs/V2_PRE_IMPLEMENTATION_AUDIT.md
```

必须记录：

1. 当前 B0/A1/A2/A3 的真实 git commit；
2. 当前 `soft_vofod_mid360` 中：
   - map update 路径；
   - event 产生路径；
   - birth buffer；
   - support 创建/删除路径；
   - association；
   - opportunity；
   - existence；
   - hard timeout；
3. 当前所有参数及默认值；
4. 当前 S07/A3 中：
   - event count；
   - support count；
   - track count；
   - map commit timing；
   - tracking timing；
5. 当前 B0/S06 warm-up 首帧、complete stamp、target spawn stamp；
6. 所有“文档与代码不一致”的地方。

本轮开始后，先不要运行新的大矩阵。

---

# 2. 第一阶段：先修实验有效性，不碰算法结论

## 2.1 修复 replay input handshake

当前 runner 只等待 output topic advertise 后固定 sleep，导致 source bag 开头可能未真正进入 detector。

必须新增算法输入就绪合同。

### B0

至少等待：

```text
points_world subscriber connected
rays_checked subscriber connected
```

并要求 B0 发布：

```text
first_input_ack
background_warmup_start_stamp
background_warmup_complete_stamp
```

### SOFT

同样发布：

```text
first_input_ack
map_bootstrap_start_stamp
```

runner 在真正开始 replay 前必须确认算法已 ready。

### run-level gate

最终结果必须满足：

```text
first_input_ack <= first_source_scored_input
```

对于 B0 warm-up：

```text
background_warmup_complete_stamp < first_target_spawn_stamp
```

且：

```text
first_scored_frame: warmup_active == false
```

否则该 run：

```text
status = INVALID_WARMUP
```

非零退出，不参与 aggregate。

---

## 2.2 重放 S06/B0

修复 handshake 后只重放：

```text
B0 × S06 × N0 × seed1001
```

重新生成：

```text
metrics.json
run_manifest.json
summary.csv
aggregate.json
ablation_deltas.csv
```

在此之前，不要把旧 S06 A1/A2 > B0 作为算法结论。

---

# 3. 第二阶段：切断当前 V1 的确定性正反馈

这一阶段先不重构地图，只做最小安全修复，并用 S07 验证。

## 3.1 删除 per-event spherical support

当前：

```text
every event
 -> 0.75 m spherical support
 -> duration 2 s
```

必须删除。

### 新规则

#### raw event

只能：

```text
进入 event buffer
可对其自身 voxel 设置极短 local quarantine
```

但不能：

- 建立半径球形 support；
- 截断周围大量 rays；
- 阻止周围 endpoints 背景更新。

推荐：

```text
event_local_quarantine <= one voxel
duration <= one map epoch
```

甚至第一版直接不做 raw-event quarantine。

#### tentative track

允许有限 support，但：

```text
radius = R_target + small margin
confidence weight < confirmed
duration <= tentative age
```

#### confirmed track

才拥有正式 map protection。

### 验收

S07/A3-like stress 下：

```text
support_count bounded
support_count << event_count
```

支持数必须与：

```text
number_of_tracks
```

同量级，而不是与：

```text
number_of_events
```

同量级。

---

## 3.2 support 数据结构不能继续线性扫描

即使 support 数下降，也不能继续：

```text
for each ray:
  for each support:
```

和：

```text
for each endpoint:
  for each support:
```

实现 spatial index。

可选：

```text
voxel hash
uniform grid
KD-tree rebuilt each map epoch
```

推荐简单 uniform grid：

```text
support_cell_size = 1–2 m
```

对 ray segment 用 DDA 查询相关 support cells。

目标：

```text
map commit complexity
```

不再与：

```text
num_rays × num_supports
```

线性相乘。

---

# 4. 第三阶段：修 ghost tracks

## 4.1 添加 survival prediction

当前 existence：

```text
P_D = 0 -> r unchanged
```

需要改成两步。

先 survival：

$$
r^-_k = p_S(\Delta t) r_{k-1}
$$

然后 miss update：

$$
r_k =
\frac{r^-_k(1-P_D)}
     {1-r^-_kP_D}
$$

推荐：

$$
p_S(\Delta t)=\exp(-\lambda_S\Delta t)
$$

其中：

```text
lambda_S
```

通过 CAL split 统一标定。

必须满足：

```text
P_D=0:
  no observation penalty
  but survival prior still slowly decays
```

这样：

> “没有观测证据”不再等于“永远存在”。

---

## 4.2 tentative 与 confirmed 使用不同生命周期

### tentative

要求：

```text
confirmation_deadline
max_tentative_age
max_tentative_no_measurement
```

tentative 没有足够新证据时快速删除。

### confirmed

允许较长遮挡，但：

```text
existence survival decay
max_confirmed_no_measurement
```

必须是 CAL 标定参数，不再使用当前近乎无意义的 30 s hard timeout。

hard timeout 仅做 safety fallback。

---

## 4.3 duplicate track merge

增加轨迹级去重。

对两个 track `i,j`，如果同时满足：

- position Mahalanobis distance 小；
- velocity difference 小；
- 最近 measurement time 接近；
- support 高度重叠；

则认为 duplicate。

选择：

```text
higher existence
longer age
more positive measurements
lower covariance
```

的 track 为主。

不要把两个真实近距离目标简单合并。

因此 merge gate 必须同时考虑：

$$
d_p,\ d_v,\ \text{history overlap}
$$

并做 S03 专门测试。

---

# 5. 第四阶段：重构背景地图为低频 Deferred Background Assimilation

这是 V2 的核心重构。

当前背景 map 不应继续在 10 ms micro-batch 中让每条 ray 独立线性累加长期状态。

新的地图分辨率仍可暂时保留：

```text
voxel_size = 0.5 m
```

先不同时改 voxel size，以减少变量。

---

# 6. 三个时间尺度

## 6.1 Ray level

频率：

```text
原始 200 k rays/s
```

只负责：

- per-ray observer compensation；
- opportunity；
- occlusion；
- 生成 epoch 内 free traversal statistics；
- 生成短时 return packet 输入。

不直接 commit 长期 map state。

## 6.2 Event packet level

时间窗口：

```text
20–50 ms
```

负责：

- 聚合短时异常 returns；
- 形成 target-level packet；
- existing track measurement；
- trajectory birth 输入。

## 6.3 Map assimilation level

频率：

```text
2–10 Hz
```

先默认：

```text
5 Hz
```

即：

```text
map_epoch = 0.2 s
```

通过 CAL 做：

```text
2 / 5 / 10 Hz
```

sensitivity。

背景状态只在 map epoch 边界 commit。

---

# 7. Map epoch 内的 free evidence aggregation

设一个 map epoch 为：

$$
W_m=[t_m,t_m+\Delta T_M)
$$

对 voxel $v$ 统计：

$$
n_F(v)=\sum_{j\in W_m} w_{jv}
$$

不要直接：

```text
free_evidence += n_F
```

改成饱和：

$$
\Delta F_m(v)
=
w_F
\left(1-\exp(-n_F(v)/n_0)\right)
$$

或实现配置选项：

```text
free_aggregation_mode:
  exponential_saturation
  capped_linear
```

默认 exponential saturation。

必须满足：

```text
1000 correlated rays
```

不会比：

```text
100 reliable rays
```

多 10 倍长期证据。

free evidence 表示：

> 本 epoch 内该 voxel 是否被可靠穿越。

而不是：

> 恰好有多少根高度相关 rays 穿过。

---

# 8. Endpoint 不再逐点直接 background commit

map epoch 内积累所有：

```text
world-frame valid returns
```

然后进行轻量空间聚合。

---

# 9. Background Assimilation Component / Packet

对 map epoch 内 returns：

1. voxel downsample；
2. 轻量 Euclidean connected components；
3. 允许 singleton component；
4. 不把 component 当 semantic object detector；
5. 仅作为 background assimilation unit。

每个 component $C$ 计算：

### 历史背景距离

$$
d_B(C)
=
\min_{p\in C,\ v\in M_B}
\|p-c_v\|
$$

### 历史 free 支持率

$$
q_F(C)
=
\frac{
|\{p\in C:\operatorname{state}(v(p))=\text{CONFIDENT\_FREE}\}|
}{
|C|
}
$$

### unknown 比例

$$
q_U(C)
$$

### target overlap

$$
q_T(C)
$$

其中 target overlap 只允许使用：

```text
tentative/confirmed tracks
```

不允许 raw events。

### world-static consistency

若 component 在多个 map epochs 中重复出现，维护 candidate background object/voxel history，计算：

```text
centroid displacement
shape overlap
support voxel overlap
```

---

# 10. Map component 分类规则

使用四类：

```text
B = BACKGROUND_SUPPORTED
F = FREE_SPACE_VIOLATION_PACKET
U = UNCOMMITTED_UNKNOWN
T = TRACK_EXPLAINED
```

## 10.1 B：历史背景支持

若：

```text
d_B(C) < d_attach
```

或：

```text
component voxel graph 与 STABLE_BACKGROUND 连通
```

并且：

```text
q_T low
```

则：

```text
B
```

对应 voxels 可以快速增加 background evidence。

意义：

> observer 移动后看到墙面的新邻接部分，可以通过历史背景空间连续性快速扩张，而不必要求同一个 0.5 m voxel 重复命中 1 s。

## 10.2 F：自由空间违例 packet

若：

```text
q_F(C) > tau_free_packet
d_B(C) > d_separate
not explained by track
```

则：

```text
F
```

输出一个：

```text
FreeSpaceViolationPacket
```

注意：

```text
component 可以只有 1 个点
```

所以仍然保留远距离 singleton UAV 检测能力。

## 10.3 U：新 UNKNOWN 表面

若 component 主要落在 UNKNOWN：

```text
U
```

不要立即当 target。

进入：

```text
candidate background / unresolved observation buffer
```

## 10.4 T：已有轨迹解释

若 component 与 tentative/confirmed track 高重合：

```text
T
```

不写 background。

---

# 11. 新区域背景的在线 assimilation

target-free warm-up 不再是 proposed method 必需条件。

对 UNKNOWN component $C$ 建立：

```text
CandidateBackgroundTrack
```

它不是 target track，只是世界静态 surface persistence tracker。

至少保存：

```text
centroid
voxel set
first_seen
last_seen
num_epochs
position variance
```

若满足：

```text
num_epochs >= K_bg
persistence >= T_bg
centroid variance <= sigma_bg
not explained by target trajectory
not repeatedly inside historical free
```

则：

```text
U -> B
```

也就是：

> 新背景通过空间持久性进入地图。

---

# 12. UNKNOWN 中的动态目标

UNKNOWN 中的 return 不能立刻视为 free-space violation，但也不能永久忽略。

建立：

```text
UnresolvedPacketBuffer
```

对于 U packets：

- 若世界坐标持续静止 -> candidate background；
- 若跨 epoch/短时窗口满足 target trajectory consistency -> 可触发 weak target birth；
- 若既不静止也不形成轨迹 -> 保持 unresolved 后超时丢弃。

定义核心竞争：

$$
\boxed{
\text{spatial persistence}
\quad vs. \quad
\text{trajectory consistency}
}
$$

这使 moving observer 在新区域中不依赖 target-free warm-up。

---

# 13. Event packetization：替代 raw endpoint event

当前：

```text
one endpoint -> one event
```

必须改成：

```text
short-time spatial packet -> one event
```

## 13.1 packet window

例如：

```text
packet_dt = 20–50 ms
```

先默认：

```text
30 ms
```

## 13.2 packet 聚合

对同一短窗内：

```text
F-class returns
```

按：

```text
R_packet ≈ target physical scale
```

聚合。

允许 singleton。

packet 至少保存：

```text
stamp_start
stamp_end
centroid / robust median
point_count
original ray indices
covariance
mean anomaly score
free-confidence
min background distance
```

同一目标在同一短窗内只应生成一个 packet。

## 13.3 packet covariance

不要把多个点简单除以 N 获得过度乐观 covariance。

至少：

$$
R_{\rm packet}
=
R_{\rm sensor}
+
R_{\rm shape}
+
R_{\rm sampling}
$$

其中：

- `R_sensor`：pose/range；
- `R_shape`：目标表面点不等于中心；
- `R_sampling`：稀疏点 packet spread。

singleton 仍合法，只是 covariance 更大。

---

# 14. Trajectory-before-confirmation V2

birth 输入改为：

```text
ViolationPacket
```

而不是 raw endpoint。

使用 3D CV。

至少：

```text
3 independent packet groups
```

推荐 birth：

```text
RANSAC CV
 -> Mahalanobis inliers
 -> weighted refit
```

必须：

- 一个 packet 只能被一个 birth candidate 消费；
- birth 后对附近未消费 packets 做 suppression；
- 不允许同一个 target footprint 在一个 epoch 内反复 birth 8 条轨迹。

新增：

```text
max_births_per_spatial_cell_per_epoch
```

作为防重复机制，但不能按场景调。

---

# 15. Existing-track maintenance measurement pool 重新收紧

当前：

```text
all returns not stable-background-consistent
```

过宽。

改成分级：

### 优先级 1

```text
F packets
```

### 优先级 2

```text
U packets near predicted track
```

允许已有 track 进入 unknown 区域后继续维护。

### 不允许

普通 raw return 直接进入 maintenance pool。

这样 background static recall 偏低时，不会把整片背景点云都开放给 tracker。

---

# 16. Association

第一版仍使用：

```text
Mahalanobis gating + Hungarian
```

但 association unit 改为：

```text
packet
```

而不是 raw endpoint anchor。

cost：

$$
C_{ij}
=
d^2_M(i,j)
+
\lambda_v d_v(i,j)
+
\lambda_a(1-a_j)
$$

其中：

- `d_M`：位置；
- `d_v`：若 packet 可估短时速度则可选，否则关闭；
- `a_j`：packet anomaly score。

不要引入 appearance network。

---

# 17. Scan Opportunity 保留逐射线

这一部分不要降频。

对 track $i$ 和 ray $j$：

$$
\eta_{ij}
=
P(\text{ray intersects physical target support})
$$

继续用 sigma-point opportunity。

## 17.1 遮挡

若当前 ray 在 target near range 之前有真实 return：

```text
eta_ij = 0
```

若前方 confirmed track 更近：

```text
eta_ij = 0
```

tentative track 不应强遮挡别的 confirmed track，除非置信度过门。

## 17.2 P_D

$$
P_D
=
1-\prod_j(1-p_{\rm ret}(r)\eta_{ij})
$$

继续 cap：

```text
P_D <= P_D_max
```

但 `p_ret(range)` 必须在 CAL 上标定。

---

# 18. opportunity 复杂度优化

不要：

```text
for every track:
  for every ray:
    7 sigma points
```

建立 ray angular index。

例如：

```text
azimuth/elevation bins
```

每条 track 根据预测角锥只查询可能相交的 rays。

或者对 `rays_checked` 建：

```text
unit-direction KD-tree
```

每个 micro-batch 重建一次。

目标是：

```text
O(num_tracks × nearby_rays)
```

而不是：

```text
O(num_tracks × all_rays)
```

---

# 19. Confirmed-track map protection

V2 中只允许：

```text
confirmed track
```

拥有强 map protection。

tentative 仅弱保护。

## 19.1 active confirmed support

定义：

```text
physical radius + capped uncertainty margin
```

不要让 covariance 越大 support 无限膨胀。

使用：

```text
uncertainty_margin <= max_support_inflation
```

## 19.2 NO_RETURN carving

若 ray 穿过 confirmed support：

```text
free carve only until target near range - guard
```

## 19.3 endpoint protection

只有 endpoint 位于 confirmed support 内：

```text
skip background promotion
```

tentative 可降低 promotion weight，但不完全禁止。

---

# 20. target-free warm-up 的新定位

## 20.1 B0

为了公平复现 B0 baseline，仍保留：

```text
target-free warm-up > B0 complete time
```

但 runner 必须使用 run-level completion gate。

## 20.2 SOFT-VoFOD V2

不再要求 target-free warm-up。

支持：

```text
startup_mode:
  cold_online
  target_free_warmup
  apriori_map
```

主算法默认：

```text
cold_online
```

warm-up 只作为 benchmark controlled condition。

---

# 21. 新实验设计：先做根因闭环，不跑全矩阵

## Phase V2-R0：实验有效性

只跑：

```text
S06/B0
```

确认 warm-up 修复。

## Phase V2-R1：event support 删除验证

只跑：

```text
S07
```

比较：

```text
old A3
A3-no-event-support
```

验收：

```text
support peak drastic drop
map commit p95 drastic drop
event count 不因 feedback 继续放大
```

## Phase V2-R2：低频地图同化

跑：

```text
NEG03 moving observer no target
S07
S01
```

重点：

```text
false-free
static-background recall
event FP/min
runtime
```

目标：

> moving observer 下新背景应通过 spatial persistence / connectivity 被吸收，而不是持续产生 violation。

## Phase V2-R3：packetization + birth

跑：

```text
S01
S03
S04
```

重点：

```text
birth count
duplicate tracks
FP
IDSW
HOTA
```

## Phase V2-R4：opportunity + survival

跑：

```text
S02
S05
S07
```

重点：

```text
fragmentation
false deletion
ghost track age
reacquisition
```

---

# 22. 重新设计消融版本

旧 A1/A2/A3 不再作为最终论文消融，只作为开发历史。

新版本：

## B0

```text
VoFOD-Mid360 + classic tracker
```

## B1

```text
Deferred Background Assimilation
+ packet-level free-space violation
+ simple packet-to-track birth
+ no opportunity
```

验证：

```text
背景模型 + packetization
```

## B2

```text
B1
+ trajectory-before-confirmation
```

验证：

```text
trajectory birth
```

## B3

```text
B2
+ scan-opportunity-aware existence
+ survival decay
```

验证：

```text
opportunity
```

## B4

```text
B3
+ confirmed-track map protection
```

验证：

```text
target/background feedback
```

不要把 Hungarian 单独作为核心创新。

GNN/Hungarian 可固定为 B1–B4 共用 tracker 后端。

---

# 23. Negative controls

必须新增：

```text
NEG01_static_observer_no_target
NEG02_static_clutter_no_target
NEG03_moving_observer_no_target
NEG04_new_area_exploration_no_target
```

至少 2–5 min 等效时间。

重点：

```text
false event / min
false tentative track / min
false confirmed track / min
false-free
static-background recall
candidate-background convergence time
```

NEG03/NEG04 是 V2 地图是否成功的核心门槛。

---

# 24. Cold-start 新区域场景

新增：

```text
S08_cold_start_exploration
```

observer 从初始区域移动到未建图区域。

包含：

### S08-A

无 target。

要求：

```text
new background -> candidate -> stable background
```

不能持续 event storm。

### S08-B

unknown 区域内 moving UAV。

要求：

```text
可以通过 trajectory consistency 形成 target
```

### S08-C

unknown 区域内 stationary UAV。

预期：

```text
保持 unresolved / candidate
```

不要强行判 target。

将其明确写进 limitation：

> 无先验历史时，unknown 中静止小物体与新静态背景在仅 LiDAR 几何下不可完全辨识。

---

# 25. 地图评估新增

除旧指标外增加：

## candidate-to-stable latency

新静态背景首次出现到被 stable assimilation 的时间。

## false-event persistence

同一静态背景区域产生 false event 持续多久。

## background expansion recall

observer 探索新区域后，新静态背景被纳入 map 的比例。

## free/background balance

每 epoch：

```text
free update voxels
background update voxels
candidate voxels
```

监控是否再次出现：

```text
free 快速增长
background 长期不增长
```

---

# 26. event packet 指标

增加：

```text
raw anomaly endpoints
violation packets
packets per true target per frame
false packets/min
packet purity
packet singleton ratio
```

目标：

> raw endpoint 数量可以很大，但进入 birth 的 packet 数应该接近 target-level observation 数量。

---

# 27. ghost track 指标

每场景增加：

```text
confirmed tracks with no measurement > 0.5s
> 1s
> 3s
max stale age
mean stale existence
```

并输出：

```text
existence vs time
P_D vs time
last measurement age vs time
```

用于验证 survival 修复。

---

# 28. 性能指标

分模块：

```text
classification
packetization
map assimilation
birth
association
KF
opportunity
support query
map commit
total
```

报告：

```text
mean
p50
p95
p99
max
```

还要记录：

```text
num rays
num packets
num events
num tracks
num supports
num candidate background objects
```

用于做复杂度相关性分析。

---

# 29. 参数标定

必须新建独立 CAL split：

```text
CAL01_map_static
CAL02_map_moving_observer
CAL03_birth_sparse
CAL04_opportunity_return
CAL05_track_survival
```

只允许在 CAL 上选择：

```text
map_epoch
free saturation n0
d_attach
d_separate
candidate promotion K_bg/T_bg
packet_dt
packet_radius
birth gates
sigma_a
measurement covariance
p_ret(range)
survival lambda
merge thresholds
timeout
```

冻结后再跑 test scenes。

禁止在 S01–S08 上逐场景调参。

---

# 30. 代码组织

建议 V2 不复制整包。

在：

```text
soft_vofod_mid360
```

内新增模块：

```text
background/
  DeferredBackgroundMap
  MapEpochAccumulator
  CandidateBackgroundManager

events/
  ViolationPacketizer
  UnresolvedPacketBuffer

tracking/
  CVKalmanTrack
  BirthManager
  GlobalNearestNeighbor
  TrackMerge
  TrackExistence

opportunity/
  RayOpportunityIndex
  OpportunityEvaluator

feedback/
  TrackSupportIndex
```

旧 V1 代码保留：

```text
legacy_v1/
```

或使用 git branch/tag，不要在生产路径中混杂多个行为开关到不可维护。

---

# 31. 配置文件

新增：

```text
config/soft_vofod_v2_canonical.yaml
config/soft_vofod_v2_calibration.yaml
```

主测试只使用 canonical。

配置必须记录：

```text
map_epoch_hz
free_saturation_n0
background_attach_distance
free_packet_ratio
unknown_promotion_epochs
unknown_promotion_time
packet_dt
packet_radius
birth_min_groups
birth_max_speed
process_noise_sigma_a
measurement_shape_sigma
track_confirm_threshold
track_delete_threshold
survival_lambda
tentative_timeout
confirmed_timeout
duplicate_merge thresholds
P_D_max
p_ret bins
support_radius
max_support_inflation
```

---

# 32. 单元测试

## map epoch

- 多 ray 同 voxel free evidence 饱和；
- free update 与 ray count 不线性无限增长；
- stable background adjacency expansion；
- unknown candidate promotion；
- target component 不写 background；
- event packet 不拥有 map support；
- confirmed track support 截断 no-return。

## packetization

- 10 个同目标 endpoints -> 1 packet；
- singleton -> 1 valid packet；
- 两个目标间距大于 packet gate -> 2 packets；
- crossing 时不跨 target 合并。

## survival/existence

- `P_D=0` 且 `p_S<1` -> r 缓慢下降；
- high opportunity miss -> 更快下降；
- hit -> 上升；
- confirmed 长遮挡不立即删除；
- ghost track 最终会删除。

## duplicate merge

- 同目标 duplicate 合并；
- 两个 0.5/1/2 m 真实目标不误合并；
- crossing 后 identity 不因 merge 崩溃。

## cold-start background

- 新墙进入视野 -> candidate -> stable；
- moving point -> 不稳定为 background；
- unknown stationary object -> unresolved/candidate，不强制 target。

---

# 33. 集成测试

至少：

```text
IT01 static wall + moving observer
IT02 hover target in mapped free space
IT03 target enters unknown region
IT04 two-target crossing
IT05 long occlusion
IT06 no-target S07-like observer motion
```

每个测试都必须：

```text
no NaN
bounded tracks
bounded supports
bounded runtime
```

---

# 34. 性能门槛

V2 至少达到：

### S07-like no-target / moving observer

```text
false confirmed tracks <= small bounded count
event/packet rate finite
p95 total processing < 100 ms on current machine
support count ~ O(track count)
```

### S03 crossing

```text
duplicate confirmed tracks significantly below V1
IDSW significantly below V1
```

### S05 occlusion

```text
opportunity-aware track fragmentation <= B2
ghost track population bounded
```

不要求第一轮立即超过 B0，但必须先消除 V1 的病态失败。

---

# 35. 最终全矩阵重新评估

只有以下门槛全部通过后才跑：

```text
S01–S08
N0/N1
>= 5 seeds
B0/B1/B2/B3/B4
```

推荐最终：

```text
10 seeds
```

如果计算资源允许。

使用 record-once/replay-many。

---

# 36. 结果声明边界

最终报告必须清楚区分：

### 已证明

- V2 是否消除 event/support storm；
- map 是否能在线扩张到新背景；
- packetization 是否降低重复 birth；
- opportunity 是否降低 false deletion/fragmentation；
- survival 是否解决 ghost tracks。

### 尚未证明

- 完整 rolling collision；
- MRS/PX4 闭环；
- 真实 Mid360 多目标；
- unknown 中静止 UAV 的立即辨识。

不得用仿真结果扩大声明。

---

# 37. 最终文档

生成：

```text
docs/SOFT_VOFOD_V2_ALGORITHM.md
docs/SOFT_VOFOD_V2_MAP_ASSIMILATION.md
docs/SOFT_VOFOD_V2_EVENT_PACKETIZATION.md
docs/SOFT_VOFOD_V2_TRACK_EXISTENCE.md
docs/SOFT_VOFOD_V2_COLD_START.md
docs/SOFT_VOFOD_V2_EXPERIMENT_PROTOCOL.md
docs/SOFT_VOFOD_V2_ABLATION.md
docs/SOFT_VOFOD_V2_KNOWN_LIMITATIONS.md
docs/SOFT_VOFOD_V2_FINAL_REPORT.md
```

---

# 38. 实现顺序

必须按以下顺序执行：

```text
Phase 0  audit
Phase 1  replay handshake + S06/B0 rerun
Phase 2  remove per-event support + support spatial index
Phase 3  survival + tentative timeout + duplicate merge
Phase 4  deferred map epoch accumulator
Phase 5  background component assimilation
Phase 6  cold-start candidate background
Phase 7  violation packetizer
Phase 8  trajectory birth V2
Phase 9  packet-level maintenance + GNN
Phase 10 opportunity optimization
Phase 11 confirmed-track-only map protection
Phase 12 root-cause experiments
Phase 13 CAL tuning
Phase 14 full benchmark
Phase 15 docs
```

每个 phase：

```text
build
unit tests
integration test
diagnostics review
commit/changelog
```

再进入下一 phase。

---

# 39. 严禁事项

禁止：

```text
读取 truth 改算法状态
按 scene_name 调参
按 target ID 处理
按墙坐标写特殊规则
按 S07 单独放宽阈值
用 GT visibility 代替在线 opportunity
用 GT target footprint 过滤 map
为了降低 FP 直接禁用 singleton
```

尤其禁止：

> 为了让结果好看，把 singleton event/packet 禁掉。

Mid360 的研究价值之一正是远距离可能只有单点，因此：

```text
singleton packet 必须合法
```

但一个短时间窗内多个同目标 endpoint 必须聚合。

---

# 40. 最终核心逻辑

SOFT-VoFOD V2 的方法论必须落实为：

$$
\boxed{
\text{Ray-level physics}
\rightarrow
\text{precise scan opportunity}
}
$$

$$
\boxed{
\text{Short-term spatial packet}
\rightarrow
\text{target observation}
}
$$

$$
\boxed{
\text{Low-frequency spatial persistence}
\rightarrow
\text{background assimilation}
}
$$

$$
\boxed{
\text{Trajectory consistency}
\rightarrow
\text{target confirmation}
}
$$

而不是：

$$
\text{每条 ray}
\rightarrow
\text{立即改变长期世界模型}.
$$

最终世界状态应明确分成：

$$
\boxed{
M_t^B
=
\text{persistent background memory}
}
$$

和：

$$
\boxed{
\mathcal T_t
=
\text{dynamic target trajectories}
}
$$

新观测在两者之间通过：

$$
\boxed{
\text{spatial persistence}
\quad vs.\quad
\text{trajectory consistency}
}
$$

竞争归属。

---

# 41. 完成后向我汇报

最终 `SOFT_VOFOD_V2_FINAL_REPORT.md` 必须包含：

1. 修复前根因；
2. 修改文件列表；
3. 删除的 V1 机制；
4. 新增的 V2 模块；
5. 所有公式与代码位置；
6. 所有参数来源；
7. S06/B0 warm-up 修复结果；
8. S07 old A3 vs no-event-support vs V2；
9. NEG03/NEG04 moving observer 结果；
10. S01/S03/S05 根因闭环；
11. map 指标；
12. track 指标；
13. runtime；
14. 失败场景；
15. 是否值得继续进入 N1/多 seed/真实 Mid360。

不要只输出“性能提升”。

必须逐条回答：

```text
false-free 是否下降？
static-background recall 是否提高？
event/packet storm 是否消失？
duplicate birth 是否下降？
ghost tracks 是否消失？
opportunity 是否真的降低 fragmentation？
map protection 是否真的降低 contamination？
```

只有这些因果链全部清楚后，才讨论“最终是否超过 B0”。
