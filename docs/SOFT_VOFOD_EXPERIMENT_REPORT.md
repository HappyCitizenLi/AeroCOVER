# SOFT-VoFOD 当前实现、S01–S07 结果与根因审计

状态日期：2026-08-21  
实验范围：`N0 ideal × seed 1001`；7 个 immutable source bags、28 个 B0/A1/A2/A3 runs  
审计范围：当前源码、run/source manifests、`metrics.json`、逐帧 diagnostics、tracks、events 和 opportunity 输出

## 1. 结论先行

当前实现和评估链已经能完整运行，但这轮结果不支持“精简版 SOFT-VoFOD A3 优于冻结 B0”。
七场景描述性均值如下：

| 方法 | mean HOTA@1m | mean IDF1 | IDSW 总数 | mean p95 ms |
|---|---:|---:|---:|---:|
| B0 | 0.676 | 0.619 | 13 | 53.3 |
| A1 | 0.378 | 0.290 | 198 | 32.6 |
| A2 | 0.388 | 0.310 | 191 | 32.7 |
| A3 | 0.362 | 0.272 | 222 | 188.9 |

需要先修正问题表述：A1/A2 并非在每个场景都差于 B0。S06 中 A1/A2 的 HOTA 都是 0.834，
高于 B0 的 0.742；但 S06/B0 后验审计发现了 replay warm-up 违约，因而这个优势暂时不能作为
干净的算法结论。其余场景和总体结果确实是 B0 更好。

根因按影响排序为：

1. **检测和出生粒度不对等。** B0 先把一帧端点做空间连通分量和 OBB，基本以“一目标一个
   detection”进入 tracker；A1/A2/A3 以单条 ray endpoint event 为原子，再从大量端点中反复拟合
   birth。一个真实目标或一片运动失配背景可产生很多合法但重复的轨迹假设。
2. **SOFT 背景图在本矩阵中更容易把背景通道判成 free，又更难恢复 stable background。**
   S01–S04 的 proposed false-free 约 0.264，B0 约 0.140；proposed static-background recall 约
   0.051–0.080，B0 约 0.124。event 的前提正是“return 落入 confident-free”，所以这个偏差直接
   扩大 anomaly 输入。
3. **轨迹级去重和 identity 约束不足。** proposed 只有出生时 1 m suppression，没有 B0 的
   `mergeSimilarTracks()`；Hungarian 只最小化当前 micro-batch 的几何代价，没有 appearance、
   identity-history、JPDA/MHT 或交叉约束。S03/A3 因而达到 99 次 IDSW，B0 只有 4 次。
4. **A3 的三个增量被捆绑，且尚未标定。** opportunity、target feedback、Hungarian 同时开启，
   当前主表不能判断哪一个单独有效。固定 `p_ret=0.5`、measurement covariance、clutter density
   和 30 s hard timeout 都是工程默认，不是 CAL split 的测量结果。
5. **S07/A3 是明确的正反馈和复杂度爆炸。** false events 产生逐 event 的 2 s 球形 support；
   map commit 对每条 ray/endpoint 线性扫描全部 supports。同时 opportunity 对每条 track 扫描
   rays 和 7 个 sigma points。评分窗口峰值约 7,761 个 active supports、230 条 tracks；实测
   support 数与 map-commit 时间的相关系数为 0.999。
6. **opportunity 保护了错误轨迹。** `P_D=0` 的 miss 按设计不降低 existence，而 30 s hard
   timeout 长于 S07 的 19 s scoring window。S07/A3 最后仍有 189 条 confirmed tracks，其中
   54 条超过 1 s 没有 measurement，最长 13.46 s；这些 ghost tracks 同时制造 FP、opportunity
   成本和 map supports。

S07/A3 的定位 RMSE 仍为 0.243 m、recall 为 0.926，但产生 15,434 个 frame-level FP，precision
只有 0.0113。它不是“目标位置估计坏了”，而是“在一个目标周围和运动背景上维护了大量错误
identity”。

最后，现有 28 runs 都通过了 source truth、track frame 和 timing frame 的 95% coverage 门禁，
但这不等于所有 warm-up 都公平。S06/B0 在 replay 中从 5.427 s 才开始内部 warm-up，目标在
14.657 s 出生，B0 到 15.428 s 才完成 warm-up；约 0.771 s 的有目标输入仍被强制当作背景。
因此当前矩阵应标记为**开发诊断矩阵**，修复 replay 首帧握手并重放 S06/B0 后才能冻结最终表。

## 2. 审计方法与结论强度

本报告把结论分为三档：

- **直接观测**：来自 `metrics.json` 或 output bag 中逐帧 diagnostics/tracks/events；
- **源码确认**：执行路径和复杂度可直接从当前 C++/Python 实现推出；
- **因果推断**：观测与源码一致，但仍需单因素消融才能严格归因。

本轮只有一个 noise mode、一个 seed，所有 mean、总数和相关系数都是开发样本的描述性统计，
不是置信区间或显著性结果。A3 同时改变三项开关，所以“GNN 导致退化”或“opportunity 导致
改善”都不能从 A2→A3 的差值单独推出。

## 3. 当前算法实现情况

### 3.1 输入与在线边界

`soft_vofod_mid360` 与 B0 并列运行，只订阅公共预处理后的：

```text
/uav1/mid360/points_world
/uav1/mid360/rays_checked
```

ROS adapter 使用 ExactTime，检查 world frame、`scan_id`、`original_index`、ray time、方向单位化
和 endpoint 几何一致性。算法没有 truth、scenario、target ID 或墙体坐标订阅。默认从首个实际
接收的合法 bundle 开始 8 s map-only 初始化；时间跳变时暂停 endpoint promotion 和 birth。

### 3.2 单条处理链

```text
world endpoints + checked rays
 -> 10 ms micro-batches
 -> 查询旧四态背景图
 -> confident-free 中的 free-space violation events
 -> existing-track association + packet median + CV-KF
 -> 未匹配 event 的多时间组 trajectory birth
 -> opportunity-aware Bernoulli existence
 -> active/deleted/event target supports
 -> support 前截断 free carving
 -> endpoint background commit
 -> tracks/events/map/opportunity/diagnostics
```

每个 micro-batch 遵守 `read old map -> tracking/support -> map commit`，当前 endpoint 不会先写图
再判断自身是否异常。

### 3.3 Conservative background map

地图是固定边界 0.5 m dense voxel grid，每格保存 free/background evidence、候选时间组、
quarantine 截止时间及四态：

- `UNKNOWN`；
- `CONFIDENT_FREE`；
- `CANDIDATE_BACKGROUND`；
- `STABLE_BACKGROUND`。

VALID_RETURN 在 `range - 0.5 m` 前 carving，NO_RETURN 最远 20 m。candidate/stable voxel 不再
接收 grazing-ray free evidence；stable background 一旦建立不会被后续 free ray 擦除。endpoint
需要至少 3 个时间组、至少 1 s、足够 background probability 才能 stable。event、已分配点和
support 内 endpoint 会刷新 quarantine，不累计背景候选。

这套保护能防止 hover target 被立即吸收入图，但也带来两个当前结果已观察到的代价：稀疏非重复
端点较难成为 stable background；错误 event/support 会阻止局部背景恢复。

### 3.4 Event 与 trajectory-before-confirmation birth

只有旧地图中 `CONFIDENT_FREE`、自由概率过门、且离 stable background 至少 0.75 m 的合法
return 才是 event。每个 event 仍是一条 ray endpoint，而不是 target-level component。

未被 existing track 使用的 events 进入 2 s、最多 256 个元素的 buffer，按 0.05 s 分组。birth
枚举两个不同组的 CV hypothesis，再从各组选择一个最小 residual inlier 做加权 refit；A1 需要
2 组，A2/A3 需要 3 组。每个 micro-batch 最多尝试创建 8 条轨迹。

成功 birth 的轨迹统一从 existence 0.6 开始。额外的第三组不会自动提高 birth existence，也
不会引入新的 target-level 去重；它主要改变延迟、fit 可用性和被消费的 event 集合。这解释了
A2 相对 A1 只有很小且不稳定的变化。

### 3.5 Maintenance、关联与状态

maintenance measurement 是所有“不与 stable background 一致”的合法 returns，不限于 events。
每条现有 track 先从单 endpoint 中选 anchor，再把 0.75 m 内的剩余点唯一分给最近 anchor，取逐轴
median 做 6-state CV-KF 更新。

A1/A2 使用按 track 顺序的 greedy anchor，A3 使用带 unmatched dummy columns 的矩形 Hungarian。
二者都只用 Mahalanobis distance 加很弱的 anomaly cost。当前没有：

- connected-component 级 target packet；
- duplicate-track merge；
- appearance/shape identity；
- crossing-aware association；
- JPDA/MHT/PMBM。

### 3.6 Opportunity、existence 与 feedback

A3 对每条 track 的 mean 和 3 个 covariance 主轴正负 sigma points，共 7 点，检查每条真实
VALID_RETURN/NO_RETURN ray 是否穿过物理 target sphere，并排除前景 return 和前方 confirmed
track 遮挡。micro-batch `P_D` 由独立乘积模型累计并 cap 到 0.95。

命中用 likelihood/clutter 的 Bernoulli update；miss 用
`r(1-P_D)/(1-rP_D)`。因此 `P_D=0` 时 existence 严格不变。这个语义本身合理，但当前没有独立的
survival decay，唯一兜底是 30 s 无 measurement hard timeout。

target feedback 有三层：

1. active track 的 uncertainty-inflated support；
2. deleted track 的短时 support；
3. **每个 event 一个持续 2 s 的 0.75 m 球形 support**。

这些 supports 既截断 free rays，又保护 sphere 内 endpoints。第三层在 event storm 中没有空间
合并或数量上限，是 S07/A3 的主要性能缺陷。

### 3.7 B0 并不是“更弱版本 A0”

| 环节 | 冻结 B0 | A1/A2/A3 |
|---|---|---|
| detector 输入原子 | 0.5 m 下采样后的空间连通分量、OBB center | 单 ray endpoint event |
| 出生 | floating component detection | 多时间组 endpoint CV fit |
| 维护 | local raw-point clustering + OBB + background-distance filter | 非 stable-consistent 点的 anchor/median packet |
| 动力学 | 9-state constant acceleration LKF | 6-state constant velocity KF |
| 重复轨迹 | covariance-radius merge | 仅 birth-time 1 m suppression |
| association | detection 到最近 track；局部点云修正 | greedy 或 Hungarian instantaneous GNN |
| existence | covariance/检测计数门 | Bernoulli opportunity，A1/A2 为固定 `P_D=0.5` |
| map feedback | 无 track feedback | A3 active/deleted/event supports |

所以结果差异不是“在同一检测器上换 association”造成的；前端观测粒度、动力学和轨迹管理都
发生了变化。

### 3.8 实验与评估基础设施

`soft_vofod_evaluation` 已实现 record-once/replay-many：每个 scene/noise/seed 只生成一份 source
bag，B0/A1/A2/A3 引用相同 source SHA-256。run manifest 保存输入、输出、配置、实现 hashes 和
replay rate。

评估实现了 fixed-threshold HOTA@1m、IDF1、IDSW、fragmentation、GOSPA、TTFT、completeness、
reacquisition、position/velocity error、event、opportunity、map 和 runtime 指标。主结果只对
confirmed tracks 与 present truth 做逐帧一对一匹配。

## 4. 场景、消融和公平性合同

| 场景 | 主要条件 | 几何模式 |
|---|---|---|
| S01 | 单目标 hover/range ladder：5/10/20/29 m | snapshot |
| S02 | 5→29→5 m 径向稀疏回波 sweep | snapshot |
| S03 | 双目标 crossing，0.5/1/2/5 m 分离 | snapshot |
| S04 | 四目标平行、交叉、分层、同步转向 | snapshot |
| S05 | 墙体遮挡 1/3/5 s 后重现 | snapshot |
| S06 | 近墙起飞、free-space hover、返回、降落 | snapshot |
| S07 | observer 平移/yaw/roll/pitch，目标同时运动 | per-ray pose |

S01 使用 29 m 而非 40 m，因为 B0 的公共 operational x 区间是 `[-10, 30) m`，不能为 proposed
方法单独扩图。S07 的 `per_ray_pose` 插值 observer geometry，但 bundle 内 collision scene 仍是
静态快照；它不是完整 rolling collision simulator。

| 方法 | birth groups | opportunity existence | target feedback | association |
|---|---:|---:|---:|---|
| B0 | VoFOD component | 否 | 否 | greedy |
| A1 | 2 | 否 | 否 | greedy |
| A2 | 3 | 否 | 否 | greedy |
| A3 | 3 | 是 | 是 | Hungarian |

A1/A2/A3 共用同一 map/KF/physical 参数；没有按场景调参。

## 5. 主结果

### 5.1 HOTA、IDF1 与 IDSW

每格为 `HOTA@1m / IDF1`：

| 场景 | B0 | A1 | A2 | A3 |
|---|---:|---:|---:|---:|
| S01 | 0.326 / 0.241 | 0.253 / 0.134 | 0.258 / 0.142 | 0.294 / 0.208 |
| S02 | 0.558 / 0.473 | 0.314 / 0.247 | 0.306 / 0.250 | 0.389 / 0.267 |
| S03 | 0.763 / 0.658 | 0.467 / 0.344 | 0.542 / 0.479 | 0.391 / 0.295 |
| S04 | 0.996 / 0.996 | 0.410 / 0.319 | 0.395 / 0.310 | 0.501 / 0.397 |
| S05 | 0.352 / 0.256 | 0.249 / 0.128 | 0.271 / 0.137 | 0.257 / 0.130 |
| S06 | 0.742 / 0.711 | 0.834 / 0.820 | 0.834 / 0.820 | 0.616 / 0.589 |
| S07 | 0.995 / 0.995 | 0.119 / 0.039 | 0.111 / 0.035 | 0.087 / 0.018 |

IDSW（B0/A1/A2/A3）：

| S01 | S02 | S03 | S04 | S05 | S06 | S07 |
|---:|---:|---:|---:|---:|---:|---:|
| 6/15/11/11 | 1/10/13/4 | 4/53/48/99 | 0/89/93/87 | 2/13/7/8 | 0/0/0/4 | 0/18/19/9 |

### 5.2 排除 S07 后的误差分解

S07 的事件风暴会支配全矩阵 FP，因此先单独看 S01–S06：

| 方法 | mean HOTA | mean IDF1 | TP / FP / FN | micro precision / recall | IDSW |
|---|---:|---:|---:|---:|---:|
| B0 | 0.623 | 0.556 | 2142 / 280 / 402 | 0.884 / 0.842 | 13 |
| A1 | 0.421 | 0.332 | 1791 / 831 / 753 | 0.683 / 0.704 | 180 |
| A2 | 0.434 | 0.356 | 1759 / 566 / 785 | 0.757 / 0.691 | 172 |
| A3 | 0.408 | 0.314 | 1963 / 1021 / 581 | 0.658 / 0.772 | 213 |

A3 比 A1/A2 找回更多 truth frames，但同时增加 FP 和 identity churn；这正是“recall 上升、
HOTA/IDF1 下降”的来源。S01–S07 的 mean position RMSE 分别为 B0 0.307 m、A1 0.345 m、
A2 0.347 m、A3 0.338 m，差距远小于 detection/association 指标，说明定位误差不是主因。

### 5.3 逐场景诊断

| 场景 | 主要失分机制 |
|---|---|
| S01 | range ladder 上 violation/birth 不连续；A3 只有 85 TP、125 FN，B0 为 113/97。A3 降低 FP，但 completeness 仍只有 0.405。 |
| S02 | proposed precision 很高但 recall 低；A3 为 110 TP/80 FN，B0 为 150/40。三组 birth 与远距稀疏回波造成更长漏检段。 |
| S03 | 目标 crossing 时大量同目标 surface endpoints 和近邻 tracks 争抢 anchor；A3 有 421 FP、99 IDSW，B0 为 0 FP、4 IDSW。 |
| S04 | A3 recall 0.934，但有 566 FP、87 IDSW；B0 为 3 FP、0 IDSW。CV + instantaneous GNN 没有保持四目标转向/交叉 identity。 |
| S05 | 遮挡后 violation birth/reacquisition 仍碎裂；A3 completeness 0.348，target contamination 0.0308，B0 contamination 为 0。 |
| S06 | A1/A2 在当前表中优于 B0；A3 在相同 1,070 events 下新增 16 FP、4 IDSW，说明退化来自 A3 bundled tracking/feedback，而不是 event front-end。但 B0 warm-up 违约，必须重放后再比较。 |
| S07 | observer motion 激发大面积 false violations；A3 用 feedback 和 opportunity 把事件风暴变成 track/support/CPU 风暴。 |

### 5.4 Map 与 event 证据

| 场景/方法 | false-free | static-background recall | event count | event FP/min |
|---|---:|---:|---:|---:|
| S03/B0 | 0.140 | 0.126 | — | — |
| S03/A3 | 0.265 | 0.080 | 6,405 | 20.0 |
| S04/B0 | 0.140 | 0.124 | — | — |
| S04/A3 | 0.264 | 0.056 | 4,595 | 200.8 |
| S07/B0 | 0.058 | 0.067 | — | — |
| S07/A1 | 0.115 | 0.034 | 11,078 | 32,469.5 |
| S07/A2 | 0.119 | 0.034 | 12,176 | 35,936.8 |
| S07/A3 | 0.354 | 0.031 | 17,837 | 53,813.7 |

S05/S06 的 target contamination 也没有验证 feedback 假设：B0 分别为 0/0.005，A1/A2/A3
分别都约为 0.0308/0.025。当前 map feedback 至少在这一个 seed 上没有带来净地图收益。

## 6. 为什么 A1/A2/A3 总体比 B0 差

### 6.1 前端把“目标表面采样”误当成“独立目标证据”

B0 的 Euclidean components 和 OBB center 在进入 tracker 前完成一次空间归并。proposed 的
event groups 只保证时间独立：同一时刻组内选一个 inlier，但一个真实目标在相邻时间组仍可支持
多个不同 CV hypotheses。旧 hypothesis 消费少量 events 后，buffer 中剩余表面点又可产生新
birth；每个 micro-batch 最多 8 个 birth 进一步放大这一点。

现有 1 m birth suppression 只检查新 candidate 与当前 track center，既不合并已存在重复 tracks，
也不把一个目标 footprint 内的 events 先变成唯一 packet。S03/S04 的 FP 和 IDSW 是这个结构性
缺口的直接表现。

### 6.2 背景图错误直接进入 event 定义

event 不是独立分类器；它完全依赖旧地图的 `CONFIDENT_FREE`。当前 proposed carving 对大量
rays 快速累计 free evidence，而 endpoint promotion 需要稀疏扫描在同一 0.5 m voxel 中跨 1 s
重复命中。运动/deskew 残差会让 endpoint 在邻近 voxels 间跳动，出现“路径很快 free、表面始终
不 stable”的组合。

B0 与 proposed 的内部 score 单位不同，不能只凭原始 weight 数值宣判；但最终 map 指标已经
证明参数/模型不匹配：proposed false-free 更高、static recall 更低。由于 violation 条件恰好
选择“return in free”，这个 map 偏差会被直接转化为 detector clutter。

### 6.3 Maintenance 的 measurement pool 过宽

birth 后，所有“不与 stable background 一致”的 returns 都能进入 association。地图 static recall
偏低时，大量普通背景也进入 measurement pool。Hungarian 虽然保证 anchor 一一对应，却不会判断
measurement 是否属于某个真实 target component；后续 0.75 m packet 只是围绕所选单点做局部
median，不能补回前端语义。

### 6.4 A2 只增加一组证据，未解决重复轨迹

A2 相对 A1 的唯一实质变化是 `min_groups: 2 -> 3`。它在 S03 减少 FP 556→305、HOTA
0.467→0.542，但在 S02/S04/S07 反而稍差。七场景 mean HOTA 只从 0.378 到 0.388，总 IDSW
从 198 到 191。第三组可减少部分偶然 birth，却没有 target-level clustering、track merge 或
identity model，因此效果小且方向不稳定。

### 6.5 A3 的 recall 收益被 ghost tracks 和 identity 成本抵消

A3 在 S01/S02/S04 提高 HOTA，在 S03/S06/S07 退化。opportunity 在遮挡或无 ray intersection
时保护存在概率，能减少部分 FN；但它也保护了预测到无机会区域的错误轨迹。hit update 使用尚未
标定的 likelihood/clutter，常把 existence 快速推近 1；之后若 `P_D=0`，miss 不再下降。

S06 是最清楚的组合证据：A1/A2 的 AssA 都是 1.0、无 IDSW；A3 的 AssA 降至 0.608，出现
4 IDSW 和 16 FP。由于 A1/A2/A3 的 event count 完全相同，这个差异位于 association/existence/
feedback 链，但当前消融无法再细分。

### 6.6 B0 的结构更贴合这些仿真目标

本矩阵中的目标是大小受限、空间连通的 UAV primitives。B0 恰好使用 connected component、
OBB size gate、background-distance filter、CA-LKF 和 duplicate-track merge；这些先验与场景高度
匹配。SOFT-VoFOD 的优势假设是“稀疏且间歇的 violation trajectory + opportunity-aware miss”，
但当前参数和实现先承受了 endpoint-level clutter，尚未到能体现该优势的阶段。

## 7. S07/A3 严重退化的完整因果链

### 7.1 结果形态：检测泛滥，不是定位崩溃

| 指标 | B0 | A1 | A2 | A3 |
|---|---:|---:|---:|---:|
| HOTA | 0.995 | 0.119 | 0.111 | 0.087 |
| IDF1 | 0.995 | 0.039 | 0.035 | 0.018 |
| TP / FP / FN | 188 / 0 / 2 | 118 / 3991 / 72 | 118 / 4315 / 72 | 176 / 15434 / 14 |
| precision / recall | 1.000 / 0.989 | 0.029 / 0.621 | 0.027 / 0.621 | 0.011 / 0.926 |
| position RMSE m | 0.230 | 0.248 | 0.246 | 0.243 |

A3 event precision 只有 0.0446，17,837 个 events 中约 95.5% 不是 1 m 内的 target event。A3
成功覆盖了真实目标，但同时每帧发布大量 confirmed false tracks，所以 DetA 只有 0.0113。

### 7.2 observer motion 是触发条件，但不能单独背锅

S07 使用 per-ray observer pose 插值；bundle 内动态 collision scene 仍是静态快照。observer 的
平移/姿态变化和 endpoint 离散会让同一静态表面在 0.5 m map 上跨 voxel，显著激发 false
free-space violations。

但 B0 消费完全相同的 `points_world/rays_checked`，仍得到 0 FP 和 HOTA 0.995。因此仿真限制是
stress 的触发器，不足以解释 A3 相对 B0 的全部差距。核心问题是 proposed map/event/feedback
对这种几何扰动不鲁棒。

### 7.3 event→support 的正反馈

源码中的循环为：

```text
false event
 -> addQuarantine(event, radius=0.75 m, duration=2 s)
 -> support 内 endpoint 不成为 stable background，free ray 在 support 前截断
 -> 旧 confident-free 区域不能被静态 endpoint 纠正
 -> 后续 return 继续成为 event
 -> 更多 births、tracks 和 supports
```

A1/A2 也会 quarantine event 所在的单 voxel，但 A3 额外建立球形 support，并为所有 active tracks
建立 uncertainty-inflated supports。这与 A3 的 event 数从 A1/A2 的 11,078/12,176 上升到
17,837、false-free 从约 0.12 上升到 0.354 一致。这个正反馈方向是强推断；严格归因仍需单独
关闭 event-sphere feedback 的一项消融。

### 7.4 map commit 为什么达到 0.8 s

评分窗口逐帧 diagnostics：

| 方法 | mean/p95 processing ms | mean tracking ms | mean map commit ms | mean/peak tracks |
|---|---:|---:|---:|---:|
| A1 | 30.7 / 45.3 | 4.4 | 25.4 | 25.4 / 129 |
| A2 | 30.3 / 43.2 | 4.1 | 25.3 | 26.6 / 137 |
| A3 | 293.3 / 1001.6 | 84.1 | 208.3 | 84.6 / 230 |

A3 的 classification 平均只有 0.36 ms。慢点不在 event 判定，而在其下游：

- `truncateBeforeSupport()` 对每条可 carving ray 遍历所有 supports；
- endpoint protection 又对每个 valid endpoint 遍历所有 supports；
- 每个 event support 存入线性 `vector`，2 s 内不合并；
- opportunity 对每条 track、每个 micro-batch 遍历 rays × 7 sigma points，并在可能相交时再
  遍历其他 confirmed tracks。

由 event timestamps 重建 2 s 存活窗口，A3 平均约 1,777 个、峰值 7,532 个 event quarantines；
加 tracks 后 supports 估计峰值 7,761。估计 support 数与 processing/map-commit/tracking 时间的
相关系数分别为 0.995/0.999/0.931。最慢帧为：

```text
stamp 31.406 s
processing 1039.95 ms
map commit 792.36 ms
tracking 246.66 ms
event quarantines ≈ 7532
tracks = 229
```

这条证据链直接确认了性能复杂度原因，不是离线 replay rate 的计时假象。

### 7.5 ghost tracks 为什么不消失

S07/A3 峰值帧有 230 条 tracks，其中 222 条 confirmed；52 条 confirmed track 已超过 1 s 没有
measurement，最长 11.16 s。评分末帧仍有 190/189 条 total/confirmed tracks，54 条超过 1 s
未更新，最长 13.46 s。

同一时刻 opportunity 输出中，峰值附近 38 条、末帧 41 条轨迹的 `P_D=0`；这些 unmatched
tracks 的 existence 不下降。A1/A2 使用固定 `P_D=0.5` miss update，峰值和末帧都没有任何
confirmed track 超过 0.5 s 未更新。这解释了为什么 A3 比 A1/A2 保留更多 false tracks。

30 s hard timeout 在 19 s scoring window 内几乎不会触发，所以它不能限制本场景的 ghost
population。ghost tracks 随后反过来增加 opportunity 的 track/ray work 和 feedback supports。

### 7.6 退化的时间演化

| scoring 区间 | events | births | mean/peak tracks | mean/p95 processing ms |
|---|---:|---:|---:|---:|
| 0–2 s | 72 | 9 | 1.1 / 3 | 31.9 / 37.9 |
| 4–6 s | 1,483 | 36 | 41.0 / 52 | 165.9 / 222.5 |
| 8–10 s | 1,389 | 28 | 104.8 / 119 | 381.6 / 421.3 |
| 14–16 s | 6,262 | 114 | 146.2 / 185 | 454.9 / 818.6 |
| 16–18 s | 4,531 | 83 | 217.2 / 230 | 951.3 / 1033.6 |

事件、birth、track、support 和 latency 同向增长，符合上述正反馈链。

### 7.7 0.1× replay 的含义

S07/A3 只有在 `replay_rate=0.1` 时才能无丢帧记录完整输出。`processing_ms` 是节点内部
steady-clock 测量，不受 bag 播放墙钟缩放；因此 p95 1.002 s 和 load ratio 2.93 仍是算法真实
负载。0.1× 只保证运输完整，不能把 A3 声称为实时。

## 8. 为什么 target-free warm-up 必须严格大于 B0 的 10 s

### 8.1 B0 状态机决定了必须使用严格不等式

B0 在处理一帧开头先计算：

```text
warmup_active = enabled && !background_warmup_complete
```

只要该值为 true，本帧全部 clusters 都通过 startup mask 强制标为 background。随后完成 point
evidence 和 far classification，帧末才检查：

```text
elapsed >= 10 s && background_points_sufficient && sure_background_sufficient
 -> background_warmup_complete = true
```

因此“让目标在首帧时间 + 10.000 s 出现”仍不安全：完成门限的这一帧在进入函数时
`warmup_active` 仍为 true，目标 cluster 会先被当作 background，状态只在帧尾翻转。10 Hz scan
离散、仿真启动抖动和首个 derived bundle 延迟还会增加边界不确定性。

所有方法共享同一 source bag，所以公共 target-free 时长必须满足较慢的 B0 10 s，而不是只满足
SOFT-VoFOD 自身的 8 s。严格 `>` 不是使用 truth 帮算法分类；它只是场景协议保证目标物理模型在
建图阶段不存在。

### 8.2 Source bag 合同已满足

| 场景 | 首个 source ray | target spawn | source target-free gap |
|---|---:|---:|---:|
| S01 | 4.045 | 14.538 | 10.493 s |
| S02 | 4.318 | 14.799 | 10.481 s |
| S03 | 4.217 | 14.702 | 10.485 s |
| S04 | 4.312 | 14.721 | 10.409 s |
| S05 | 4.130 | 14.624 | 10.494 s |
| S06 | 4.228 | 14.657 | 10.429 s |
| S07 | 4.207 | 14.702 | 10.495 s |

runner 目前据此执行 `target_free_input_duration_s > 10.0` 门禁，并要求 spawn 与 scoring start
完全相同。这个 source-level 检查是必要的，但后验审计证明它还不充分。

### 8.3 S06/B0 暴露了 replay-level 缺口

| 场景 | B0 warm-up 起始诊断 | B0 完成帧 | target spawn − 完成帧 | 判定 |
|---|---:|---:|---:|---|
| S01 | 4.045 | 14.145 | +0.393 s | 通过 |
| S02 | 4.318 | 14.418 | +0.381 s | 通过 |
| S03 | 4.217 | 14.217 | +0.485 s | 通过 |
| S04 | 4.312 | 14.313 | +0.408 s | 通过 |
| S05 | 4.130 | 14.130 | +0.494 s | 通过 |
| S06 | 5.427 | 15.428 | **−0.771 s** | **失败** |
| S07 | 4.207 | 14.207 | +0.495 s | 通过 |

S06/B0 第一条 `MapUpdateDiagnostics` 的 `background_warmup_elapsed_sec=0`，证明 B0 实际从
5.427 s 才建立 warm-up epoch，而不是从 source bag 的 4.228 s。runner 当前只等待 track output
topic 被 advertise，然后固定 sleep 1 s；它没有验证 detector 的两个输入 subscribers 已连接，
也没有等待首个 input acknowledgment。因而 source 开头约 1.2 s 输入在该 replay 中没有进入 B0
warm-up。

这不会触发现有 scored-frame coverage 门禁，因为丢失发生在 scoring 之前；但会污染 warm-up
公平性。S06 的目标在 14.657–15.428 s 间被 B0 startup mask 当成 background，可能压低其后续
结果。当前报告保留原数字用于诊断，但不把 S06 A1/A2 > B0 作为有效优越性声明。

### 8.4 正确的门禁应同时检查 source 和 run

最低限度需要：

1. replay 前验证算法节点已经订阅 `points_world` 和 `rays_checked`，而不只是 output topic 存在；
2. run 后从 B0 diagnostics 读取第一次 `background_warmup_complete=true` 的时间；
3. 强制 `warmup_complete_stamp < first_target_spawn_stamp`，并要求首个 scored frame 的
   `background_warmup_active=false`；
4. 门禁失败则整个 run 非零退出，不写入最终 aggregate；
5. 修复后至少重放 S06/B0，并重新生成 summary/aggregate/ablation deltas。

把 source margin 从约 0.5 s 增到 2 s 可以降低连接抖动风险，但不能替代 run-level 硬门禁。

## 9. 最小修正路线（按优先级）

### P0：先修实验有效性

- 增加 replay input-subscriber/first-frame handshake 和 B0 warm-up completion gate；
- 重放 S06/B0，重新评估当前矩阵；
- 在完成前将结果标为 development diagnostics，不生成“最终优于/劣于”的统计声明。

### P1：先切断 S07 的确定性放大器

最小且可验证的改动不是优化 Hungarian，而是停止为每个 event 永久追加独立球形 support。event
endpoint 已经进入 `BackgroundMap::quarantine()`；第一项单因素实验应只保留 active/deleted-track
supports，或把 event supports 按 voxel/time 合并并设置硬上限。验收条件至少包括：

- S07 event storm 下 support 数有界；
- p95 与 track/event 数不再超线性增长；
- S05/S06 target contamination 不恶化。

### P2：限制重复 birth 和 ghost tracks

- birth 前先把同一 micro-batch 的 event endpoints 合成 target-level spatial packets；
- 对 active tracks 增加 duplicate merge，不能只在 birth 时做 suppression；
- 为 tentative tracks 增加短 stale timeout；
- 为 confirmed existence 加时间连续的 survival model，或至少把 30 s hard timeout 改为经 CAL
  验证的上限；不要让 `P_D=0` 等价于在整段实验内永久保活。

### P3：把 A3 捆绑消融拆开

在独立 CAL/NEG bags 上至少增加：

```text
A2 + Hungarian only
A2 + opportunity only
A2 + feedback only
A2 + opportunity + Hungarian
```

否则无法判断 S06/S07 的具体责任模块，也无法合理调参。

### P4：标定后再扩实验

只在独立 CAL01/CAL02/CAL03 和 NEG01–NEG03 上统一标定 map evidence、`p_ret(range)`、covariance、
clutter、birth/merge 和 timeout。冻结参数后再运行 N1/N2、至少 5 seeds，并升级 S07 到动态
collision scene 的 Tier B rolling geometry。不得在 S01–S07 test bags 上逐场景调参。

## 10. 指标和声明边界

- HOTA 是固定 1 m 定位阈值版本，不是跨阈值积分 HOTA；
- opportunity Brier/NLL 只在已出生 proposed tracks 上统计，不覆盖 birth 前 FN；
- event target match 使用 primitive center，ray ABI 目前不携带 Gazebo hit entity；
- map truth 是 0.5 m primitive rasterization 近似；
- `false_deletion_count` 是 truth matching fragmentation proxy，不是全部内部 delete transitions；
- S03/S04 source truth coverage 分别约 99.2%/97.8%，通过 95% 门禁但不是 100%；
- 当前没有 N1/N2、多 seed、CAL、长时 negative controls 或真实 Mid-360 flight 结果；
- S07 `per_ray_pose` 不证明 MRS/PX4 闭环、气动、飞控或完整 rolling collision 性能。

## 11. Artifacts、复现与验证

主要机器可读输出：

- `artifacts/benchmark_summary.json`：28 个 run 状态；
- `artifacts/metrics/summary.csv`：逐 run 主指标；
- `artifacts/metrics/aggregate.json`：分组统计 schema；
- `artifacts/metrics/ablation_deltas.csv`：同 source paired deltas；
- `artifacts/source_bags/.../source_manifest.json`；
- `artifacts/runs/.../{run_manifest.json,metrics.json,timing.csv,output.bag}`。

已有 source bags 的标准 replay 命令（S07/A3 单独降速）：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
python3 src/soft_vofod_evaluation/scripts/run_benchmark.py \
  --algorithms B0,A1,A2 \
  --scenes S01,S02,S03,S04,S05,S06,S07 \
  --noise N0 --seeds 1001 --replay --output artifacts

python3 src/soft_vofod_evaluation/scripts/run_benchmark.py \
  --algorithms A3 --scenes S01,S02,S03,S04,S05,S06 \
  --noise N0 --seeds 1001 --replay --output artifacts

python3 src/soft_vofod_evaluation/scripts/run_benchmark.py \
  --algorithms A3 --scenes S07 --noise N0 --seeds 1001 \
  --replay --replay-rate 0.1 --output artifacts
```

S07/A3 在当前实现下需单独使用 `--replay-rate 0.1` 才能完整记录；在 P1 修复前不要用 1× 丢帧
结果替换当前质量指标。

最终源码验证记录：

```text
catkin build
  11/11 packages succeeded, no build warnings/failures

catkin run_tests --no-status -j1
  11/11 packages succeeded

catkin_test_results build
  339 tests, 0 errors, 0 failures, 0 skipped
```

Gazebo rostests 共享固定 `/gazebo` 服务名，必须串行执行；并行全测的跨 fixture 服务冲突不代表
算法或单个 gate 失败。
