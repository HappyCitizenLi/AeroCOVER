# 当前 AeroCOVER 与 VoFOD：框架、实现、配置和四场景实验总报告

> 本文以本次核查的**当前源码、生效配置、保留 manifest/metrics/逐帧 CSV、源数据 flight_audit**为依据。只整理与自检，未重跑实验、未修改算法或默认参数。旧的 `CURRENT_AEROCOVER_VOFOD_IMPLEMENTATION_RESULTS_AND_ANALYSIS.md` 主要描述早期 P01/P02，不能替代本报告。
>
> 主对比固定为 **AeroCOVER-V8 八项 + VoFOD-V15 八项**，其中 V15/OPEN/Mid-360 复用 V14。V13 和 V16 作为补充，不用某一场景调参后的最好结果替换主表。共有18次独立保留回放、19条版本记录；主对比为16项。
>
> 清理后的保留索引：[INDEX_ZH.md](../results/retained_experiments/INDEX_ZH.md)。8项 AeroCOVER 的 output.bag 在清理前就已不存在，但指标、逐帧 CSV、资源记录和代码/配置快照仍在；VoFOD V13–V16 完整输出仍保留。V10–V12 等原始结果已删除，不在本文中伪装成可重新核验的数据。

## 目录

1. [结论与边界](#1-结论与边界)
2. [系统输入与数据组织](#2-系统输入与数据组织)
3. [AeroCOVER 完整框架及实现](#3-aerocover-完整框架及实现)
4. [AeroCOVER 生效参数与优化状态](#4-aerocover-生效参数与优化状态)
5. [VoFOD 实现及相对论文和 GitHub 的改动](#5-vofod-实现及相对论文和-github-的改动)
6. [四个场景与传感器实验设置](#6-四个场景与传感器实验设置)
7. [评分协议与可比性](#7-评分协议与可比性)
8. [指标、runtime、资源与补充实验](#8-指标runtime资源与补充实验)
9. [结果分析与尚未解决的问题](#9-结果分析与尚未解决的问题)
10. [验证、复现与证据来源](#10-验证复现与证据来源)
11. [附录：当前 AeroCOVER 与 tracker YAML](#附录当前-aerocover-与-tracker-yaml)

## 1. 结论与边界

- AeroCOVER 在保留的 OPEN、OFFICE、FOREST 六个“场景×传感器”实例中，轨迹评分均为 FP=FN=IDSW=Frag=0。**这是1.5 m门限下的轨迹输出结果，不意味着每帧都有原始回波、strict detection或厘米级定位。**
- MT 的 AeroCOVER 漏检全部归于两架悬停目标；运动目标 uav4 两传感器下的匹配召回均为100%。Mid-360 仍有8 FP、149 FN、4 IDSW/4 Frag；OS1仍有41 FN、3 IDSW/3 Frag。
- 当前 VoFOD 主配置已经是**地面第一层中央**：z下界−0.125 m，Mid-360 ready=0.0001、OS1 ready=0.15。OPEN/MT使用所保留的V2参数族；OFFICE/FOREST使用较小聚类与tracker半径的参数族。
- 第一层边界显著减轻了地面附近FP，但不能把所有静态FP都解释成地面：FOREST/Mid-360仍有3966 FP，其中仅68个落在 `abs(z)<0.5 m` 高度带。
- OFFICE/OS1在主配置中仍只有4.68%召回。V16把ready降至0.01后召回为39.33%，但仍有888个首次匹配后的FN，说明主要后续损失没有由降低ready解决。
- 两算法不具有完全相同的先验或计时边界：AeroCOVER无持久占据图/测距仪种子；VoFOD使用体素图、测距仪及地图边界先验。AeroCOVER计时含输入转换与内置tracker，VoFOD表中runtime仅为检测主体、不含外部tracker。不能无条件把它们称为统一端到端计时比较。
- 本地VoFOD是兼容移植版，不是未经修改的上游代码。此前发现的**清理邻域重复写入与本地去重写回差异尚未修复**；本报告不能证明全部性能差异都是原论文机制单独造成的。

## 2. 系统输入与数据组织

### 2.1 共同数据链

```text
MRS X500 + PX4/MAVROS + Gazebo
    ├─ Mid-360滚动扫描 + 每条ray时间/位姿 → points_world + CheckedRayBundle
    └─ OS1 1024×128 GPU快照原始点云
           → ouster_snapshot_adapter（安装外参、body mask、索引/几何校验）
           → points_world + CheckedRayBundle
                       ├─ AeroCOVER：ST点背景 → shell → 内置CA tracker
                       └─ VoFOD：标量体素图 → 检测 → lidar_tracker_mid360

目标/observer真值与world几何 → 仅用于仿真、源数据审计、离线评分
native_rangefinder → VoFOD初始化；不送入AeroCOVER核心
```

算法不直接读取目标GT轨迹或离线LOS评分。必须区分：传感器数据生成和世界系变换使用仿真位姿，不能因此声称实验包含真实自定位误差、或不需要自机定位。当前噪声标记N03对应range标准差0.03 m，不等于原论文中包含自定位和姿态误差的整套噪声模型。

### 2.2 Checked ray 合约

- 一个输入scan同时提供点与ray；利用scan_id、original_index、offset_time_ns和时间戳进行对应。
- VALID_RETURN应有匹配点，且 `origin + range × direction` 与点坐标一致，默认几何容差0.01 m。
- NO_RETURN没有对应端点；被body mask屏蔽、状态未知、几何/TF无效的ray不能伪装成NO_RETURN提供free证据。
- Mid-360按每点/每ray采样时刻组织，OS1源数据是单个Gazebo快照，其t字段为0。即使源manifest上有requested rolling_scene，也应按 `sensor_time_organization` 区分传感器。
- OS1原始输入保留131072个pattern身份。固定body mask覆盖24849个pattern；屏蔽项标成不可用，不是删除后再把空位当free。
- “不抽样ray”指不对可用ray另做稀疏采样；AeroCOVER证据FIFO**不会保留不可用、无正可信长度/权重的ray作为证据**。不能把原始ray总数等同于证据FIFO条数。
- 自机安装参考：OS1相对fcu的z偏移0.1414 m；场景几何审计同时使用Mid-360的0.168 m安装高度。测距仪TF已经按实际模型父子关系核对，不能混用LiDAR与测距仪原点。

主要入口：

| 模块 | 源码/配置 |
|---|---|
| OS1独立转换及共享body mask | [ouster_snapshot_adapter_node.cpp](../src/mid360_ray_preprocessor/src/ouster_snapshot_adapter_node.cpp)、[转换参数](../src/mid360_ray_preprocessor/config/ouster_os1_128.yaml) |
| AeroCOVER输入转换/发布 | [aerocover_node.cpp](../src/aerocover_mid360/src/aerocover_node.cpp) |
| VoFOD节点 | [vofod_nodelet.cpp](../src/vofod_mid360/src/vofod_nodelet.cpp) |
| 共同mask及OS1生效ray数 | [ouster_os1_128_1024.yaml](../src/vofod_mid360/config/sensors/ouster_os1_128_1024.yaml) |

## 3. AeroCOVER 完整框架及实现

### 3.1 状态、职责和单scan流程

AeroCOVER由ROS适配层、独立ST背景库、ray几何/空间索引、candidate生命周期和内置9状态CA tracker组成。没有额外VoFOD tracker、持久静态占据图、PlanePatch、core占据证据、dormant重识别或目标GT输入。

```text
输入点/ray校验
  ├─ 异步准备当前ray block及DDA索引（私有，尚不可用）
  └─ 历史ray过期与旧证据扣除
       → 当前+历史点精确连通
       → 历史背景传播 / 同scan短时间切片尺度判定
       → 当前残余component
       → 帧内运动拟合、扫描末端补偿、观测/协方差
       → 最近ST背景点约束球壳外半径
       → 仅历史ray的S42 strict / S29 maintenance预筛
       → 已生轨迹CA预测、门控、Hungarian关联与更新/删除
       → 剩余strict观测关联candidate、新candidate抑制、3-of-5出生
       → commit当前ray block（仅供以后scan使用）
       → 发布扫描末端状态、诊断与可视化
```

`AeroCoverCore::processScan()` 执行上述顺序。点背景库会保留本scan的标签作为未来历史，但**当前scan的ray在本次检测/出生决策之后才commit**。因此并行预处理不等于当前ray可以证明当前候选。

状态层级：

| 层级 | 保存内容 | 退出方式 |
|---|---|---|
| Point history | 点、scan/time/original_index、点级背景标签 | 超出point_window的旧scan过期；不是持久地图 |
| Ray history | usable ray、可信线段、scan_id、空间索引、贡献引用 | FIFO过期；保留一个已过期block仅作内存复用 |
| Candidate | 最近观测、CV估计、shell几何、generation、证据累计、birth bit历史 | 出生后移除；失配>2scan或超时>1s移除 |
| Track | id、p/v/a、9×9协方差、上次观测时刻、观测计数 | 无观测>1s或位置不确定度超限/状态无效删除 |

### 3.2 精确空间连通和时间切片背景

实现位于 [background.cpp](../src/aerocover_st_background/src/background.cpp)。

1. 在历史窗口内的点与当前点之间，按欧氏距离≤0.30 m形成半径图。**时间不直接决定图边**；时间信息用于之后的背景判定。
2. 网格cell边长为 `nextafter(tolerance/sqrt(3),0)`，同cell点必然连通；采用62个确定性半邻域offset覆盖可能的跨cell连接，而不是简单只查26邻居。
3. cell内点按连续scratch存储；跨cell先做AABB距离界检查，必要时执行精确点对距离；union-find得到与半径图一致的component。
4. 对每个component，历史背景点数≥3且占**历史点数**比例≥0.20，可传播背景标签。分母不是当前+历史所有点。
5. 若尚未判背景，按scan_id、stamp、original_index排序；在同scan内，**相邻点时间差≤10 ms**连续分段。每段至少3点，若AABB任意轴跨度>1.5 m，则component判为背景。这里是相邻时间间隔规则，不是每段总时长固定10 ms。
6. 被判背景时，component内当前和历史点的标签都更新；未判背景的component只输出当前scan点索引，交给后续观测处理。

因此，快速运动目标一秒轨迹很长不必被整段当成大物体；但目标与墙/柱连通仍可能引起背景传播。大尺度首scan切片即可判背景，`spatiotemporal_history_ready`只是诊断，不是强制等FIFO填满后才能检测的开关。

过期粒度也有区别：点历史按scan末端时刻过期，跨过cutoff的scan会整体保留，并非逐点截成恰好1.00/0.50 s；因此Mid滚动scan内最早点可能早于名义窗界。ray历史则支持部分block逐ray过期。不能把两个FIFO的配置时长理解成完全相同的裁剪实现。

### 3.3 当前观测、运动补偿与协方差

实现：`makeObservation()`。

- 当前component按20 ms时间bin计算坐标逐维中位数中心；至少3个bin时拟合线性速度。
- 帧内CV拟合要求速度≤12 m/s、RMS≤0.50 m，否则使用零速度补偿；**拟合失败并不直接禁止出生**。
- 把点投影到scan末端：`p_i' = p_i + v × (t_end - t_i)`。
- 最终位置为补偿点的算术平均；measurement covariance为样本scatter/N再加 `0.20^2 I`，并作对称化。
- extent是补偿点AABB三轴长度，候选/观测最大轴跨度门限3 m。
- 对中心距离的95%分位数加0.10 m后截断到[0.15,0.75] m，作为component半径。不是最大离群点包络，也不是无人机已知真尺寸。

跨scan candidate另外拟合最近最多5次观测的CV；RMS≤0.20 m且速度≤12 m/s时有效，用于预测和出生初速度。**当前代码不把这个CV有效性作为3-of-5出生的强制条件**，初速度可回退至帧内拟合或0。

### 3.4 球壳及自适应障碍几何

默认：
```text
r_component = clamp(q95(point distance)+0.10, 0.15, 0.75)
r_inner     = r_component + 0.05
r_outer     = r_inner + 1.00
```

打开obstacle_adaptive_shell时，用**本scan传感器派生的ST背景点**建立KD-tree，查询离观测中心最近的背景距离d：
```text
r_outer_new = min(r_outer, d - 0.10)
若 r_outer_new - r_inner < 0.20，则拒绝这个观测
```

当前实现是**整个球壳统一缩小外半径**，不是沿每个方向裁剪成任意障碍贴合形状；没有读取world模型、地面高度或目标GT。没有背景点不代表free，仍需后续ray证明。该设计可能降低近墙shell被静态障碍截断的概率，但无法保证极窄空间中仍有可认证厚度。

### 3.5 历史可信线段与full-chord

[ray_geometry.cpp](../src/aerocover_mid360/src/ray_geometry.cpp) 对ray–sphere交点使用解析几何，求外球区间，再减去内球区间得到最多两段shell弦。

| ray类型 | 可信自由长度 | 证据权重 |
|---|---|---|
| VALID_RETURN | max(0, range−0.50 m) | 1.0 |
| NO_RETURN | 20 m | 0.25 |
| 不可用/屏蔽/无正长度 | 不参与 | 0 |

full-chord要求：外球交段不能贴住可信线段起点或终点，即ray须在可信自由段内完整进入并离开外球。只碰到前表面、在目标表面终止、或仅切线零长度均不能贡献。

**没有射线不等于free；没有端点也不自动等于可用NO_RETURN。** VALID_RETURN可提供超过20 m的可信段（取决于实际range减余量），20 m不是统一component距离门限。

### 3.6 42方向证据与scan内饱和

- 42个方向由Fibonacci球面采样固定生成。
- 解析求出的shell弦按最大0.10 m步长分段，以段中点相对中心方向的最大dot产品分配bin；并不是方向积分完全解析或连续球面逐点证明。
- 单ray对bin贡献：`delta(r,b)=w_r × min(1, L(r,b)/0.20)`。
- 同一个 `(scan_id, bin)` 内累计后封顶1：`G(s,b)=min(1, sum_r delta(r,b))`，避免一帧密集ray直接替代跨scan支持。
- 跨scan密度：`D(b)=sum_s G(s,b)`。D≥1.5的bin为supported；有有效正弦长贡献的bin为observable。
- coverage= supported_bins/max(1, observable_bins)。另统计有贡献的不同scan数、支持方向octant数和角跨度。scan_id分组是工程上的独立组定义，**不等同于统计学完全独立观测**。

出生strict门控：observable≥42、supported≥42、coverage≥0.60、至少2个支持scan、至少2个octant。默认42/42已经使coverage达到1，某些通用门限在当前strict设置下是冗余的，但实现仍保留。

已有轨迹的maintenance：observable和supported均≥29，其余coverage、支持scan、octant条件仍在。不是完全取消shell，也不能用maintenance观测创建出生candidate。

### 3.7 关联、出生与轨迹维护

**已生轨迹关联**在 [ca_tracker.cpp](../src/aerocover_mid360/src/ca_tracker.cpp)：

- CA预测到当前scan末端，状态x=[p,v,a]。
- 关联代价为位置innovation的Mahalanobis距离平方；门限11.345，使用全局Hungarian一对一匹配，并允许不匹配。
- maintenance观测必须满足欧氏距离≤1.5 m；上一scan已经失配的stale track，即使面对strict观测也要满足这道1.5 m门。
- 连续获得观测的track面对strict观测，代码主要使用统计门，不是无条件给所有strict关联加1.5 m硬门。
- 更新采用Joseph形式协方差更新。过程噪声是白jerk模型，sigma=2 m/s³：
```text
F(dt) = [[I, dt I, .5 dt² I], [0,I,dt I], [0,0,I]]
Q(dt) = sigma_jerk² *
        [[dt⁵/20, dt⁴/8, dt³/6],
         [dt⁴/8,  dt³/3, dt²/2],
         [dt³/6,  dt²/2, dt]] ⊗ I3
```
- 无测量仍可输出预测位置；未匹配超过1 s、任一位置轴sigma>5 m或数值状态无效时删除。没有dormant重识别，删除后的再次出生会分配新id。

**Candidate关联与birth**在 `associateCandidates()/evaluateCandidate()`：

- 只使用未分配给track的strict观测；候选关联是距离排序的确定性贪心，不是另一套Hungarian。
- candidate欧氏门1.0 m，代码另允许观测协方差形成的统计门通过；不要把这个1.0 m与track维护的1.5 m混淆。
- 创建新candidate前，如果观测距离任一已生track当前状态≤1.5 m，则抑制新candidate；它不是把附近观测无条件关联到那条track。
- geometry合格且shell通过时推入true，否则/失配推入false；最近5位至少3次通过并有至少3次观测，才出生。并非必须连续3次。
- 出生位置来自当前观测，初速度优先跨scan CV、再帧内速度、最后0；初始加速度0。出生candidate随后从候选池移除。

### 3.8 增量证据、空间索引与因果检查

- 每份贡献带candidate_id、generation、ray_id、scan_id/bin等信息，支持过期精确扣除；整scan过期可直接移除该scan组。
- 几何重建时旧generation失效，重新查询历史ray并按参考实现累计。
- 通用重建门为平移>0.20 m或半径变化>20%。但**自适应shell启用时，只要中心或内/外半径有变化也会重建**，不能把当前路径概括为“所有scan只做增量”。
- 空间索引为DDA遍历ray经过的cell，查询时对球体覆盖cell做保守筛选、ray_id去重并确定性排序，随后仍执行精确ray–sphere判断。
- 周期参考检查默认每100scan，在有匹配candidate时检查一个candidate；不是每帧每candidate的全量审计。单测使用更密集参考检查。
- 持续记录self_support_violation、future_stamp_violation、reference_incremental_mismatch；保留八项AeroCOVER诊断中三者均为0。这是观测到的检查结果，不等于对所有输入的形式化证明。

### 3.9 输出和实现位置

| 文件 | 职责 |
|---|---|
| [background.h/.cpp](../src/aerocover_st_background/include/aerocover_st_background/background.h) | 点历史、精确连通、时间切片、点标签传播 |
| [aerocover_core.cpp](../src/aerocover_mid360/src/aerocover_core.cpp) | 因果顺序、观测、shell预筛、候选、出生与ray提交 |
| [ray_geometry.cpp](../src/aerocover_mid360/src/ray_geometry.cpp) | 弦段、方向bin、scan饱和、证据累计与DDA索引 |
| [ca_tracker.cpp](../src/aerocover_mid360/src/ca_tracker.cpp) | 9状态CA、Hungarian、维护门、删除 |
| [aerocover_node.cpp](../src/aerocover_mid360/src/aerocover_node.cpp) | ROS同步、索引/几何校验、输入点范围过滤、状态时刻发布 |
| [消息目录](../src/aerocover_mid360/msg) | CandidateEvidence、BirthEvent、Track和诊断字段 |

输出前缀为 `/aerocover` 或 `/aerocover_os1`，包括tracks、diagnostics、background_points、residual_points及可选markers。Tracks消息的header表示决策/状态时刻，source_header保留来源scan；不能因借用了通用Tracks消息类型而误认为AeroCOVER运行了VoFOD的外部tracker。

## 4. AeroCOVER 生效参数与优化状态

### 4.1 两传感器配置差别

| 参数 | Mid-360 | OS1 |
|---|---|---|
| points/rays | /uav1/mid360/… | /uav1/ouster/… |
| expected rays | 20000 | 131072 |
| 输入同步队列 | 128 | 8 |
| point_window / ray_fifo | 1.00 / 1.00 s | 0.50 / 0.50 s |
| 点输入距离截断 | 未设置有限截断（默认最大double） | 42 m |
| ray spatial_hash_cell | 1 m | 2 m |
| connectivity workers | 1 | 1 |
| shell预筛 / ray索引准备 workers | 1 / 1 | 8 / 8 |
| markers | 开 | 关 |

OS1的42 m截断只限制送入ST背景处理的点，不缩短已校验有效ray的可信段、不做ray抽样。当前代码中没有找到“所有component先按20 m或40 m中心距离统一剔除”的独立门；**旧提示词中的40 m设想不能替代当前实现事实**。当前NO_RETURN=20 m，40 m只是单测/历史试验存在过的可选参数，不是默认值。

完整base YAML和OS1覆盖文件见附录；生产值以YAML叠加为准，而不是 `Config{}` 的裸构造默认值。裸默认中的0.40 m、6 bins、未启用full-chord等不可冒充当前主配置。

### 4.2 当前保留的性能实现

| 实现 | 当前情况与限制 |
|---|---|
| 精确网格+union-find剪枝 | 保留；同cell必连、已连通cell免重复点对、AABB必不连/必连界 |
| 近优先neighbor顺序、连续位置scratch | 保留；保留原point index输出次序，不把它泛称为全部SoA |
| generation-stamped开放寻址cell表 | 保留，复用bucket/storage |
| 当前ray准备与ST阶段重叠 | 保留，私有block直到决策结束才commit |
| shell observation和DDA cell并行 | 保留，OS1相应8 workers；关联、union更新/合并和commit保持确定性 |
| scan分组证据、整scan移除 | 保留，partial过期仍逐ray处理 |
| 一个过期ray block的容量复用 | 保留；缓存不参与因果FIFO，容量/RSS不是固定字节上限 |
| sphere cell cover/query scratch | 保留，几何key变化须重建，结果仍精确过滤 |
| angular index、persistent worker pool、scan-local connectivity cache、42-bin批量argmax原型 | 当前生产路径未使用；历史速度对照数据已清理，不在本报告重新声称其收益 |
| block min/max时间边界快路径 | 当前processScan仍逐历史ray检查future，expireRays仍有all_of/逐ray逻辑；不能因历史建议就写成已经完全实施 |
| “删除全部排序” | 不成立，时间点排序、观测排序、索引去重排序仍实际存在 |

### 4.3 消融配置存在，不等于当前有消融结果

A1关闭**点**时空历史；A2用whole-history extent；A3关闭shell；A4取消full-chord；A5把出生支持次数设为1。它们仍有小型YAML/测试实现，但历史消融实验数据已按要求删除，本文不生成或补造消融表。A1并不自动关闭ray FIFO，A5也不自动取消shell的跨scan证据要求。

## 5. VoFOD 实现及相对论文和 GitHub 的改动

### 5.1 版本基准：三者不能混称

- 论文：*On Onboard LiDAR-based Flying Object Detection*，TRO公开稿/arXiv v4。[论文](https://arxiv.org/pdf/2303.05404)
- 论文关联GitHub标签：`evaluation-for-paper`，commit `5dbe299321885fa85d9c49eb89e3433ee8e8f005`。
- 本地导入VoFOD基准：`7da9f33a878a586588f6a626b75cfeacac7824f7`；tracker基准：`a92b4db61060b47f1af6dcce122188ec021f2dcd`。[VoFOD来源记录](../src/vofod_mid360/UPSTREAM.md)、[tracker来源记录](../src/lidar_tracker_mid360/UPSTREAM.md)
- “VoFOD”“Original”“V2”是工作区方法/参数名称，不构成与未修改上游或原论文实验完全等价的保证。

### 5.2 当前本地流程

```text
actual rangefinder → TF到world → 少量背景种子
points + checked rays
  → weighted voxel cloud / 几何聚类
  → 查历史占据图做close/far partition
  → 先更新当前点证据
  → ready成立才进行far/floating连通判定
  → 检测位置、协方差、confidence与固定detection_probability
  → 外部 lidar_tracker_mid360

异步ray worker：忙则跳过新提交，按既有时序写回free更新
独立0.1s cleanup：识别不够确信的分离背景簇、写回清理
```

- 标量图初始/unknown=-740、point=0、ray=-1000；sure阈值−0.1、new occupied阈值−300、frontier阈值−750。
- ready需要占据体素计数严格大于Nx×Ny×ratio，并存在确信背景簇；它是动态条件，可能再次变false，不是一次永久置位。
- 历史占据邻域内的点簇判为background；far簇受最少点数、最大OBB对角线、最大距离门控，再搜索unknown是否连到背景或搜索边缘。
- 一旦簇内某个点被判连通，就不作为floating目标。检测器会更新地图，包括把部分已探索unknown写为frontier。
- 当前没有“必须等10scan”的fixed-lag/STInit/PersistentInit冷启动；唯一seed来源是实际rangefinder，而不是目标GT或手工整个地面点集。
- 输出confidence仍由局部图不确定性计算；detection_probability固定为0.5，二者不要混同。检测协方差沿用距离启发式，外部tracker的测量噪声另有固定R配置。

### 5.3 与原论文/GitHub的机制边界

论文提供体素映射、有限范围连通分类、分离背景清理和基于KF的tracker框架，但论文伪代码不等于全部工程实现。原GitHub的清理邻域、unknown写回和边界规则有额外实现细节。[论文III-B/III-D](https://arxiv.org/pdf/2303.05404)、[论文标签节点代码](https://raw.githubusercontent.com/ctu-mrs/vofod/5dbe299321885fa85d9c49eb89e3433ee8e8f005/src/vofod_nodelet.cpp)

| 项目 | 上游/论文与当前本地的关系 |
|---|---|
| 输入与传感器 | 上游面向组织化Ouster点云及LUT；本地使用逐ray状态、方向、原点、时间的CheckedRay合约，支持Mid-360与OS1快照 |
| 射线遍历 | 本地有边界/数值硬化的DDA、索引和端点保护；不能把移植版当成上游二进制 |
| self/body mask | 换成当前X500安装/四场景原始自机回波校准的共享pattern mask，OS1适配器同样应用；屏蔽项不作为无回波 |
| 外参/seed | rangefinder TF和输入有效性检查按实际安装修正；仅seed实际测距位置，未给VoFOD整张真值背景图 |
| map execution | 保留point-update-before-classification、异步ray工作与独立cleanup；但调度/安全结构经过移植 |
| cleanup重叠写回 | 上游重叠邻域可对同一体素重复更新；本地snapshot上收集unique索引后统一写回。这是仍需等价性核查的差异，不是已证明无影响 |
| 连通搜索 | 本地保留上游DFS和起始边界即连通规则；与论文BFS/欧氏搜索范围的描述应分别说明，不能泛称逐字节论文算法 |
| unknown→frontier | 上游节点已有，本地并非新加；但这不等于论文Algorithm 1明确列出了该写回步骤 |
| 状态输出/队列 | 本地增加source_header、状态时刻一致性、逐帧完成、受限FIFO和异常拒绝；不是原始“最新消息覆盖”接口 |
| tracker协方差半径 | 本地使用det(position covariance)的第六根；上游源码使用立方根，这是算法/实现差别，不是同一公式换名 |
| tracker关联 | 保留预测邻域取点、聚类、背景过滤、选择最近簇与重叠轨迹清理；不是AeroCOVER的全局Hungarian |
| apriori文件图 | 本地当前路径没有完整移植上游PCD先验载入；当前使用native_rangefinder |
| detector概率 | 组织化采样相关概率近似被固定0.5替代；不能宣称保留原概率模型全部语义 |

半径差别可直接核对：[上游getRadius](https://raw.githubusercontent.com/ctu-mrs/lidar_tracker/a92b4db61060b47f1af6dcce122188ec021f2dcd/src/lidar_tracker.cpp) 与 [本地tracker_core.cpp](../src/lidar_tracker_mid360/src/tracker_core.cpp)。本地用 `1.5 × det(P_position)^(1/6)`（并应用min radius）；不要把它写成“与原论文/上游完全一致”。

### 5.4 参数差别：当前主配置

GitHub下面数值以导入commit的配置为准，而不是所有上游历史版本。论文Table II的体素是0.25 m，GitHub该配置为0.5 m；本地0.25 m不能同时说成“偏离论文”和“与GitHub默认相同”。[上游检测参数](https://raw.githubusercontent.com/ctu-mrs/vofod/7da9f33a878a586588f6a626b75cfeacac7824f7/config/detection_params.yaml)

| 参数 | 论文/上游基准说明 | 本地主配置 |
|---|---|---|
| voxel_size | 论文0.25；GitHub0.5 m | 0.25 m |
| ready ratio | GitHub0.15 | Mid 0.0001；OS1 0.15；V16仅OFFICE另试0.01 |
| min_sure_voxels | 论文/上游24 | Mid 1；OS1 24 |
| d_cluster / d_close | 论文1.5/1.5 m | OPEN/MT 1.5/1.5；OFFICE/FOREST 0.5/0.3 |
| detector min_points | GitHub2 | Mid 1；OS1 2 |
| detector max_size | GitHub3 m | OPEN/MT 3；OFFICE/FOREST 1.5 |
| detector max_distance | 导入配置50 m | 50 m |
| ray dmax / weight | 20 m / 0.003 | 20 m / 0.003 |
| map范围 | 不是统一论文实验边界保证 | xy名义[-28,60]×[-24,54]，当前z下界−0.125 m |
| cleanup距离/周期 | 上游0.8 m / 0.1 s | 0.8 m / 0.1 s |

`evaluation-for-paper` 与导入commit的配置本身也不同，例如旧标签init=-500，导入配置init=-740；讨论“原版参数”必须带版本，而不是混用两份文件。

Tracker参数：[上游tracking.yaml](https://raw.githubusercontent.com/ctu-mrs/lidar_tracker/a92b4db61060b47f1af6dcce122188ec021f2dcd/config/tracking.yaml)。本地Q/P/R按论文Table II标准差平方配置，而非沿用GitHub默认方差。[论文Table II](https://arxiv.org/pdf/2303.05404)

| 参数 | GitHub导入配置 | OPEN/MT本地 | OFFICE/FOREST本地 |
|---|---|---|---|
| 输入leaf / 聚类距离 | 0.5 / 1.0 m | 0.5 / 1.0 | 0.25 / 0.5 |
| 最大簇对角线 / 背景距离余量 | 1.0 / 1.0 m | 1.0 / 1.0 | 1.5 / 0.1 |
| min/max不确定半径 | 2.5 / 5 m | 2.5 / 5 | 0.6 / 3 |
| Q对角线(p,v,a) | 0.01,0.8,0.05 | 0.0001,0.04,0.09 | 同左 |
| P0对角线(p,v,a) | 0.3,1,0.05 | 0.09,1,1 | 同左 |
| R系数 | 0.1 | 0.09 | 0.09 |
| buffer / detection计数门 | 10 / 2 | 10 / 2 | 10 / 2 |

共同tracker设置还包括：radius multiplier=1.5、cluster min_points=1/max_points=262144、max_pending_frames=128、background filter开启。OPEN/MT的 `tracking_open_v2.yaml` 只是参数覆盖，不能据名称就声称它与上游所有实现相同。

### 5.5 地图第一层为何影响地面FP

初始unknown=-740，DFS仅扩展高于−750的体素。分离背景清理会把其邻域向ray_score=-1000更新；单次0.5混合可将−740变成−870，不要求每个邻居被真实ray穿过。上游代码已有这种邻域更新和搜索后的unknown写回；因此“free侧图值”不必全部等同于直接观测到的自由空间。

本地 `exploreToGround()` 对**起始查询体素** `z_idx<=0` 直接返回连通，classifier随即不把相应簇当floating。把平地z=0放到[-0.125,0.125)这一第一层，会触发已有边界先验；第二/第三层不会仅凭层号触发。它不是新增的通用地面分割，也不是证明地下被测成free。

代码证据：[VoxelMap](../src/vofod_mid360/src/voxel_map.cpp)、[清理/分类](../src/vofod_mid360/src/strict_baseline_core.cpp)、[节点写回](../src/vofod_mid360/src/vofod_nodelet.cpp)。此前对V10地下快照的分析属于历史调查，原V10 bag已删；这里不声称本次重新读取了那些体素或已逐个追溯全部FP来源。

### 5.6 VoFOD 输出状态为何可以是预测

外部tracker处理新点云时先预测，再在预测邻域聚类/选择测量；没有合适测量时仍可保留预测。检测回调可能来自较早输入，再沿buffer推进至较新的状态时间。因此发布消息可能是：
- 当前观测校正后的后验；
- 没有当前校正的预测；
- 较早检测沿buffer推进后的较新状态。

这不天然等于使用未来数据。必须分别看state/header、source_header、last_prediction、last_correction、last_detection和发布事件顺序。当前评估不会把较新状态倒投回旧source帧，也不把接收时钟与采集时钟的微小差值直接当作不可用证据。

## 6. 四个场景与传感器实验设置

### 6.1 当前实际保留场景

旧的S1/P01/P02/M1/M2或五目标MT不作为当前输入定义。当前源配置均在 [retained_experiments/sources](../results/retained_experiments/sources)。

- **OPEN / PW_open**：observer沿y=0、z≈3 m的直线往复；uav2围绕(1,7.8)作半径4.8 m的上升圆轨迹，z约3.7→4.0 m；uav3在另一侧z=6 m按带高速进入段的圆角五尖星外轮廓运动，中心约(1,−11.1)，非规则各向同性五角星。实际设计使用x/y外尺度27.5/2.5 m并协调observer位置；所有目标实测距离小于19 m。
- **MT / PW_mt**：当前3个目标，不是5个。uav2=(3.13,0,2.7)、uav3=(−3.13,0,3.8)悬停，机体外缘到半径1.2 m圆柱约1.5 m；圆柱高15 m。uav4在半径16 m圆周运动，z=3.9 m；observer半径12 m追逐，z约3.1 m并有0.25 m中途降低。悬停目标允许柱体遮挡，只有LOS有效时计入目标评分；运动目标无该豁免。
- **OFFICE / PW_office**：一架目标经过多个走廊/转角，observer追逐；目标z≈1 m、observer≈0.8 m。允许转角短时距离小于4 m；不是“一条直线、始终4–20 m”的旧设想。
- **FOREST / PW_forest_seed0**：目标由西侧x≈−19.75进入并穿越到东侧x≈55.95，掠过障碍，z≈2.65 m；observer≈2.0 m在后方约4.5 m追逐。此版实际距离带较窄，不能宣称在该场景全面验证4–20 m距离范围。
- OFFICE/FOREST使用水泥风格地面视觉材质；纹理不是额外给算法的地面标签。

### 6.2 实飞审计与规划要求分开

下表来自保留源数据的 `flight_audit.json`，以20 ms采样实际位姿。距离为observer–目标中心距离，俯仰角包含安装高度及姿态变换；净空使用0.43 m机体包络，近障碍比例为目标机体包络到非地面障碍净空<1 m的采样占比。

| 场景 | 目标数/源seed | 场景时长s | 实测距离m | 俯仰角° | 净空/含采样余量m | 峰值速度m/s | 近障碍比例 |
|---|---|---|---|---|---|---|---|
| OPEN | 2/42 | 93.505 | 4.396/18.838 | -1.86/17.46 | 2.557/2.478 | 7.897 | 0.00% |
| MT | 3/12001 | 52.000 | 4.137/15.332 | -4.71/19.50 | 1.500/1.422 | 7.799 | 0.00% |
| OFFICE | 1/12001 | 149.570 | 2.609/4.009 | -2.31/14.11 | 0.350/0.308 | 4.152 | 81.38% |
| FOREST | 1/12001 | 181.357 | 4.498/4.588 | 0.96/17.57 | 0.392/0.350 | 4.236 | 52.00% |

| 场景 | 评分绝对时段s | Mid/OS1评分帧 | 逐机实测峰值速度m/s | 规划峰值加速度m/s² |
|---|---|---|---|---|
| OPEN | 36.296/127.896 | 917/1022 | uav1:3.51; uav2:1.30; uav3:7.90 | 3.795 |
| MT | 33.268/83.268 | 500/526 | uav1:5.85; uav2:0.00; uav3:0.00; uav4:7.80 | 3.746 |
| OFFICE | 34.012/181.612 | 1476/1495 | uav1:4.15; uav2:4.15 | 2.883 |
| FOREST | 35.736/215.136 | 1795/1934 | uav1:4.24; uav2:4.23 | 1.929 |


- 四场景审计状态均为PASS，truth最大间隔均约8 ms；采样加余量后的最小净空仍>0.30 m。
- OPEN/MT至少一架实飞峰值达到MRS fast 8 m/s参考的90%；OFFICE/FOREST至少一架达到50%。不是每架都达到：OPEN圆轨迹目标实际峰值仅约1.30 m/s。
- 保守共同FOV要求[-5°,20°]，上述实测范围均落在其中。场景审计是有限时间采样及几何模型检查，不是连续时间碰撞/LOS形式化证明。
- OPEN/OFFICE/FOREST未记录静态LOS失败；MT的失败计数由允许的悬停柱遮挡产生，未允许的静态LOS失败为0；运动目标互遮挡最长记录为0。
- **实飞最大加速度字段为null，审计说明明确“acceleration not assessed”。** 表中的规划峰值来自预检模型，不可冒充实际加速度已经验证。
- source_manifest的评分窗是数据时刻，和YAML名义duration/score_end经采样对齐后可能有小差别，以实际评分窗为准。每条序列开头约1秒不计入评分，不能把主表100%召回解释成从上电第一帧开始零延迟。

### 6.3 两传感器的共同与不同条件

| 项目 | Mid-360 | Ouster OS1 |
|---|---|---|
| 输入组织 | rolling_scene，逐ray时间 | 1024×128快照，t=0 |
| 每包原始ray数 | 20000 | 131072 |
| 垂直/共同FOV使用 | 不把标称FOV当证据；统一审计[-5°,20°] | 源模型45°垂直FOV，仍按共同保守区间审计 |
| 量程策略 | 实际回波 + 20m无回波可信范围 | 模型range 0.1–120m；算法可信段另有限制 |
| 原始噪声 | N03：range σ=0.03m | 同批N03；不是完整实飞定位误差模型 |
| 评分帧数 | 由Mid源header/offset网格确定 | 由快照源header确定，数量可不同 |

OS1源审计确认每帧131072 rays、无非增时间戳/零时间戳/invalid非零回波；最大方向几何残差约0.00033–0.00051 m。两传感器评分帧数不同来自采样组织和源发布时间，不应为了相等而复制或删去帧。

## 7. 评分协议与可比性

### 7.1 主表来源与时间语义

- AeroCOVER取保留V8八项；VoFOD取V15八项，其中OPEN/Mid取V14原结果。
- AeroCOVER metrics schema=7，评估器SHA：`5d09d14420e9e18f3502d2bc4d8df9278f432a62abdbc3a1809b58bcb86e507e`；VoFOD metrics schema=9，评估器SHA：`a2988629f751b35601652feddf8b666dfada0ea4c24f61787463541b81519758`。版本数字和文件摘要分开记录。
- 两组评估器文件并不相同。源码diff表明新StateTimeSelection用于VoFOD，AeroCOVER分支仍按scan-end状态/source映射；共同的1.5m匹配、score grid、MT LOS ignore和HOTA关闭规则保留。
- 但AeroCOVER output.bag已经不存在，所以本次没有将其在新评估器上完整重算。**不要把主表宣称为“同一评估器文件重新统一跑过”。**

VoFOD新协议检查last_prediction与state一致、last_correction/last_detection不晚于state、source不晚于state；同一状态推进前选最后合法完整快照，空快照也可替换。状态流推进之后不回填旧状态。接收时钟仅审计，不能冒充真实在线延迟；该协议并不宣称零延迟在线系统评估。

### 7.2 匹配与指标

- 每评分帧，GT与输出轨迹位置用1.5 m门限做一对一匹配；TP/FP/FN是目标–帧样本数，不是独立目标数或错误事件数。
- Precision=TP/(TP+FP)，Recall=TP/(TP+FN)；同时报告IDSW、Frag、IDF1、匹配位置/速度RMSE。
- RMSE只针对已匹配样本，低召回方法可能只匹配容易的一小段；不能用低RMSE掩盖大量FN。
- MT悬停目标无LOS时ignore：不计TP/FN，且在保护可见GT匹配后，匹配到被忽略GT的相应预测不计FP；重复预测仍可计FP。当前Mid/OS1分别忽略241/255个GT样本。其余场景没有这种柱体遮挡豁免。
- 轨迹重现时换id仍可造成IDSW/Frag；“忽略遮挡GT”不等于跨遮挡身份自动正确。
- HOTA关闭。旧文件里可能还有历史命名如`*_1m`，应读取实际main threshold=1.5 m，不靠字段名字推断阈值。
- 当前保留主表各项输出、计时、完成与真值覆盖率均100%。这表示评估输入/输出槽完整，不是全为TP或每帧都有检测。

### 7.3 回放倍率及runtime边界

| 算法 | Mid-360 | OS1 |
|---|---|---|
| AeroCOVER-V8 | 1.00× | **0.25×** |
| VoFOD-V15及保留补充项 | 0.25× | 0.15× |

这是从各run_manifest读取的实际值，不能沿用“所有Ouster都是0.15×”的概括。

- AeroCOVER的diagnostics.total_ms = 输入转换耗时 + core.processScan耗时；core包含ST、shell、内置tracker与ray准备等待等。常规ROS序列化/发布、上游OS1适配和源仿真不全部包含。
- VoFOD的total_ms为检测主体回调计时；异步ray/cleanup工作、外部tracker、转换和发布并非统一端到端计时。
- **绝对ms无需乘回放倍率。** 慢速回放改变到达间隔，且可能影响异步任务忙跳过/调度；绝对ms可读，但不能把不同倍率下的结果冒充严格同负载、同截止时间的实时对照。
- 峰值RSS来自原resource.txt计时进程树口径，不是整机占用或多个并发节点RSS简单相加。CPU秒与进程墙钟也不能混同。
- OS1的OPEN/MT AeroCOVER平均已接近或超过100 ms；慢放下100%覆盖不证明原速长期不积压。要宣称实时，应补原速稳态、尾延迟、排队和端到端统计。

## 8. 指标、runtime、资源与补充实验

### 8.1 固定主对比：16项

位置/速度RMSE单位分别为m和m/s；mean/p95为ms。数值直接来自保留metrics，不用历史文字手工拼造。

| 场景 | 传感器 | 算法/来源 | TP/FP/FN | Precision/Recall % | IDSW/Frag | IDF1 % | 位置/速度 RMSE | mean/p95 ms |
|---|---|---|---|---|---|---|---|---|
| OPEN | Mid360 | AeroCOVER-Mid360 | 1834/0/0 | 100.00/100.00 | 0/0 | 100.00 | 0.208/0.536 | 45.394/71.317 |
| OPEN | Mid360 | VoFOD-Mid360（V14复用） | 1656/6/178 | 99.64/90.29 | 0/3 | 94.74 | 0.337/0.700 | 13.575/20.543 |
| OPEN | OS1 | AeroCOVER-OS1 | 2044/0/0 | 100.00/100.00 | 0/0 | 100.00 | 0.066/0.231 | 103.889/122.124 |
| OPEN | OS1 | VoFOD-OS1 | 1974/0/70 | 100.00/96.58 | 0/0 | 98.26 | 0.104/0.523 | 152.495/164.321 |
| MT | Mid360 | AeroCOVER-Mid360 | 1110/8/149 | 99.28/88.17 | 4/4 | 66.22 | 0.194/0.359 | 40.084/53.637 |
| MT | Mid360 | VoFOD-Mid360 | 979/106/280 | 90.23/77.76 | 4/7 | 63.14 | 0.319/0.517 | 22.398/33.977 |
| MT | OS1 | AeroCOVER-OS1 | 1282/0/41 | 100.00/96.90 | 3/3 | 66.49 | 0.051/0.144 | 106.691/124.299 |
| MT | OS1 | VoFOD-OS1 | 1060/0/263 | 100.00/80.12 | 3/3 | 62.69 | 0.085/0.329 | 170.659/237.154 |
| OFFICE | Mid360 | AeroCOVER-Mid360 | 1476/0/0 | 100.00/100.00 | 0/0 | 100.00 | 0.102/0.164 | 21.780/32.634 |
| OFFICE | Mid360 | VoFOD-Mid360 | 1425/127/51 | 91.82/96.54 | 0/0 | 94.12 | 0.076/0.138 | 26.862/35.978 |
| OFFICE | OS1 | AeroCOVER-OS1 | 1495/0/0 | 100.00/100.00 | 0/0 | 100.00 | 0.045/0.053 | 85.346/94.407 |
| OFFICE | OS1 | VoFOD-OS1 | 70/0/1425 | 100.00/4.68 | 1/1 | 6.13 | 0.072/0.115 | 44.068/55.095 |
| FOREST | Mid360 | AeroCOVER-Mid360 | 1795/0/0 | 100.00/100.00 | 0/0 | 100.00 | 0.110/0.202 | 32.479/66.592 |
| FOREST | Mid360 | VoFOD-Mid360 | 1744/3966/51 | 30.54/97.16 | 0/0 | 46.48 | 0.091/0.190 | 58.649/98.101 |
| FOREST | OS1 | AeroCOVER-OS1 | 1934/0/0 | 100.00/100.00 | 0/0 | 100.00 | 0.043/0.058 | 86.512/109.792 |
| FOREST | OS1 | VoFOD-OS1 | 1554/154/380 | 90.98/80.35 | 5/5 | 27.90 | 0.090/0.260 | 139.804/276.791 |


### 8.2 MT按目标拆分

| 算法 | 目标 | 类型 | Recall % | IDSW/Frag |
|---|---|---|---|---|
| AeroCOVER-Mid360 | uav2 | 悬停 | 75.39 | 3/3 |
| AeroCOVER-Mid360 | uav3 | 悬停 | 87.66 | 1/1 |
| AeroCOVER-Mid360 | uav4 | 运动 | 100.00 | 0/0 |
| VoFOD-Mid360 | uav2 | 悬停 | 70.07 | 3/4 |
| VoFOD-Mid360 | uav3 | 悬停 | 72.08 | 1/3 |
| VoFOD-Mid360 | uav4 | 运动 | 88.20 | 0/0 |
| AeroCOVER-OS1 | uav2 | 悬停 | 96.43 | 2/2 |
| AeroCOVER-OS1 | uav3 | 悬停 | 92.52 | 1/1 |
| AeroCOVER-OS1 | uav4 | 运动 | 100.00 | 0/0 |
| VoFOD-OS1 | uav2 | 悬停 | 74.58 | 2/2 |
| VoFOD-OS1 | uav3 | 悬停 | 91.59 | 1/1 |
| VoFOD-OS1 | uav4 | 运动 | 78.14 | 0/0 |


### 8.3 runtime尾部与进程资源

| 场景/算法 | 倍率 | median/p99/max ms | 峰值RSS MiB | 进程墙钟s | user/system CPU s |
|---|---|---|---|---|---|
| OPEN/AeroCOVER-Mid360 | 1 | 44.010/79.028/86.897 | 476.5 | 131.3 | 58.9/0.9 |
| OPEN/VoFOD-Mid360 | 0.25 | 13.092/27.183/31.313 | 788.5 | 417.9 | 148.3/59.1 |
| OPEN/AeroCOVER-OS1 | 0.25 | 105.615/129.418/140.463 | 1177.3 | 418.2 | 210.0/18.3 |
| OPEN/VoFOD-OS1 | 0.15 | 149.544/237.150/246.050 | 791.5 | 673.4 | 390.9/75.9 |
| MT/AeroCOVER-Mid360 | 1 | 42.757/58.609/75.264 | 418.7 | 89.9 | 30.0/0.4 |
| MT/VoFOD-Mid360 | 0.25 | 23.499/36.163/37.240 | 791.4 | 253.2 | 72.0/15.9 |
| MT/AeroCOVER-OS1 | 0.25 | 109.357/131.271/137.620 | 1276.5 | 253.1 | 111.1/11.1 |
| MT/VoFOD-OS1 | 0.15 | 156.768/239.858/253.939 | 793.2 | 398.4 | 218.7/39.0 |
| OFFICE/AeroCOVER-Mid360 | 1 | 20.063/36.225/92.506 | 394.4 | 186.5 | 43.9/0.9 |
| OFFICE/VoFOD-Mid360 | 0.25 | 28.528/38.581/43.550 | 780.7 | 640.1 | 201.8/82.8 |
| OFFICE/AeroCOVER-OS1 | 0.25 | 85.282/97.454/111.353 | 820.3 | 639.9 | 176.4/30.0 |
| OFFICE/VoFOD-OS1 | 0.15 | 45.548/65.851/68.938 | 828.6 | 1043.4 | 276.5/111.2 |
| FOREST/AeroCOVER-Mid360 | 1 | 26.320/86.124/102.598 | 428.8 | 218.3 | 80.3/3.6 |
| FOREST/VoFOD-Mid360 | 0.25 | 51.200/112.519/116.652 | 805.2 | 767.3 | 334.6/124.2 |
| FOREST/AeroCOVER-OS1 | 0.25 | 84.013/124.103/152.253 | 1049.6 | 767.1 | 272.9/36.1 |
| FOREST/VoFOD-OS1 | 0.15 | 103.694/457.814/472.724 | 827.0 | 1255.0 | 617.1/131.2 |


### 8.4 AeroCOVER可观测耗时分解

各列单位ms，取保留诊断CSV均值。ray准备/插入与ST可能重叠，shell预筛又属于evidence阶段；**这些列不能相加冒充total**。关联包括候选与track部分的诊断累计口径。

| 场景/传感器 | 输入转换 | grid | union | shell预筛 | 关联 | ray准备/插入 | total |
|---|---|---|---|---|---|---|---|
| OPEN/Mid360 | 0.401 | 0.520 | 7.698 | 34.516 | 0.015 | 15.917 | 45.394 |
| OPEN/OS1 | 3.302 | 15.074 | 67.544 | 7.649 | 0.019 | 22.305 | 103.889 |
| MT/Mid360 | 0.410 | 0.892 | 7.894 | 28.249 | 0.017 | 16.404 | 40.084 |
| MT/OS1 | 3.365 | 15.141 | 59.990 | 17.457 | 0.023 | 21.987 | 106.691 |
| OFFICE/Mid360 | 0.589 | 3.828 | 7.338 | 5.737 | 0.011 | 5.964 | 21.780 |
| OFFICE/OS1 | 4.298 | 13.825 | 38.594 | 6.337 | 0.014 | 6.804 | 85.346 |
| FOREST/Mid360 | 0.513 | 2.741 | 6.398 | 19.421 | 0.011 | 11.401 | 32.479 |
| FOREST/OS1 | 3.919 | 15.036 | 38.348 | 10.149 | 0.014 | 12.261 | 86.512 |


由当前数据可见：OS1的精确点连通/union是重要热点；Mid在OPEN/MT的shell预筛开销突出，OFFICE则更均衡。不要直接引用已删旧实验中的“259 ms热点”或旧优化百分比描述当前结果。

### 8.5 FN阶段、ready与地面附近FP

首次匹配前/后FN来自已有CSV审计；地面附近是 `abs(z)<0.5 m` 的预测高度筛选，不是逐点确认物理地面回波。AeroCOVER没有VoFOD-style ready，“不适用”不是缺数据。

| 场景/算法 | 首次匹配前/后FN | 地面附近FP/总FP | ready帧/评分帧 | 首次ready时间s |
|---|---|---|---|---|
| OPEN/AeroCOVER-Mid360 | 0/0 | 0/0 | 不适用 | — |
| OPEN/VoFOD-Mid360 | 172/6 | 0/6 | 844/917 | 43.418 |
| OPEN/AeroCOVER-OS1 | 0/0 | 0/0 | 不适用 | — |
| OPEN/VoFOD-OS1 | 70/0 | 0/0 | 989/1022 | 39.320 |
| MT/AeroCOVER-Mid360 | 106/43 | 0/8 | 不适用 | — |
| MT/VoFOD-Mid360 | 245/35 | 29/106 | 442/500 | 39.116 |
| MT/AeroCOVER-OS1 | 16/25 | 0/0 | 不适用 | — |
| MT/VoFOD-OS1 | 258/5 | 0/0 | 412/526 | 43.584 |
| OFFICE/AeroCOVER-Mid360 | 0/0 | 0/0 | 不适用 | — |
| OFFICE/VoFOD-Mid360 | 51/0 | 31/127 | 1445/1476 | 37.223 |
| OFFICE/AeroCOVER-OS1 | 0/0 | 0/0 | 不适用 | — |
| OFFICE/VoFOD-OS1 | 545/880 | 0/0 | 941/1495 | 87.846 |
| FOREST/AeroCOVER-Mid360 | 0/0 | 0/0 | 不适用 | — |
| FOREST/VoFOD-Mid360 | 51/0 | 68/3966 | 1654/1795 | 37.748 |
| FOREST/AeroCOVER-OS1 | 0/0 | 0/0 | 不适用 | — |
| FOREST/VoFOD-OS1 | 39/341 | 2/154 | 1896/1934 | 39.348 |


### 8.6 保留补充实验：不替换主表

| 版本/场景/方法 | 下界z / ready | TP/FP/FN | IDSW/Frag | Recall % | mean/p95 ms |
|---|---|---|---|---|---|
| V13/OPEN/VoFOD-Mid360 | -0.375 / 0.0001 | 1656/5180/178 | 0/3 | 90.29 | 13.861/21.129 |
| V14/OPEN/VoFOD-Mid360 | -0.125 / 0.0001 | 1656/6/178 | 0/3 | 90.29 | 13.575/20.543 |
| V16/OFFICE/VoFOD-OS1 | -0.125 / 0.01 | 588/49/907 | 7/5 | 39.33 | 44.277/56.541 |


V14也是V15/OPEN/Mid的来源，不重复算新回放。V13只比较第二层与第一层；V16只改变同为第一层地图的OFFICE/OS1 ready为0.01。V10–V12原始数据已删除，本文不把旧摘要中的数字恢复成新的完整实验记录。

## 9. 结果分析与尚未解决的问题

### 9.1 AeroCOVER的优势和限制

- OPEN/FOREST/OFFICE的保留主结果无FP/FN，说明在这些具体输入、1.5m门限及既定评分窗下，ST+历史shell+CA输出组合有效；单seed和有限轨迹不能证明泛化。
- 历史shell提供区别于“地图分数推断free”的证据来源；full-chord和延后commit避免前表面回波/当前目标自证。
- 自适应外半径为近障碍目标提供了更小的可检验域，但仍保留最小0.20m厚度和strict证据条件；太近、背景点缺失/误标、回波断续时仍会失败。
- 小而静止的目标并不是由“零速度”规则直接剔除：出生不要求运动。但是静止/遮挡场景更难累积其周围充分历史自由视角，且可能连接到背景或受球壳空间限制。这是机制解释，不能当作每个FN已逐条回放验证的根因。
- MT的FN在审计中均属于悬停uav2/uav3。Mid FN按目标为111/38，OS1为17/24；运动uav4无FN、IDSW/Frag。
- Mid的8个MT FP来自已有轨迹：已保留的审计记录支持其与预测漂移/失配后输出有关，而不是新生地面轨迹。由于原output.bag缺失，本文不能补做每次严格观测和ray贡献的完整反事实审计。
- 1.5 m维护innovation门和Hungarian能限制错误关联，但不能恢复没有观测的目标，也不能保证删除后的身份恢复；当前没有dormant re-ID。

### 9.2 VoFOD当前失败模式

- OPEN/Mid在V14有6 FP、178 FN、3 Frag。172 FN发生在首次匹配前，6个在之后；移到第一层主要解决地面附近FP，没有解决启动和后续断续。
- MT/Mid有106 FP、280 FN，全部106 FP对应曾匹配过GT的轨迹，不能仅凭低高度认定都是独立地面假目标；候选/背景过滤、预测与遮挡重现仍需逐事件追查。
- OFFICE/OS1默认ready=0.15只有941/1495评分帧为ready，且整段有检测的帧很少。V16把ready降到0.01后，首次匹配前FN由545降至19，但首次匹配后仍为888。**仅降低ready不能解决ready后的主要检测/关联损失。**
- OFFICE/FOREST刻意贴近障碍，当前小d_close和小tracker半径虽做过适配，仍可能遇到目标与背景簇连接、背景距离筛除、回波稀疏及预测失配。分类/跟踪哪一层主导每次失败，需要候选级证据，本文不只凭最终FN给出唯一原因。
- FOREST/Mid的3966 FP主要不在地面附近；第一层边界不能消除树木/非地面静态背景碎片的潜在floating误判。FOREST/OS1保留380 FN，不能用减少FP代替召回评价。
- 地图边界、ready门限、min_sure_voxels、voxel_size和ray可信范围互相影响。当前场景地图xy很大，ready的分母又是Nx×Ny而不是可见区域大小，这解释了为何跨场景使用同一比例并不保证同一初始化时序。
- 这些都是当前本地移植/参数/输入上的观测与机理分析，不能直接升级成“原论文全部无效”或“所有FP均由本地改动造成”的结论。

### 9.3 不应作出的结论

1. 不宣称已对V10–V12重新复验：它们已清理。
2. 不宣称当前主对比包含消融验证：保留的是配置/测试，不是消融结果。
3. 不宣称VoFOD为完全未经修改的原版，也不宣称本地cleanup去重等价性已解决。
4. 不宣称无回波20m、点保留42m或旧40m设想可以相互替代。
5. 不宣称球壳“42 bins”是对整个连续球面的严格完备证明。
6. 不宣称无Fn轨迹输出代表无回波断档、无预测补偿或零初始化延迟。
7. 不宣称实飞加速度已通过审计；当前只有规划加速度与实飞速度/几何检查。
8. 不用低召回子集的RMSE、较少IDSW或更短runtime掩盖未输出目标。
9. 不把统计学泛化、实机实时性能和未知地形地面先验视为已验证。

### 9.4 后续验证优先级（尚未执行）

1. 先为VoFOD上游与本地建立相同地图、相同输入、相同调度条件的逐体素/候选对照，重点核查cleanup重叠写回，避免先把全部FP归因于任意一方。
2. 对两算法使用统一回放倍率和明确的端到端起止事件重新测延迟、排队、CPU/RSS；当前检测主体ms不能替代这项试验。
3. 保留双方完整输出，逐事件分析MT悬停目标、FOREST非地面FP和OFFICE ready后的FN，再决定改检测、关联还是初始化。
4. 补实飞加速度审计、独立seed/轨迹与真实传感器数据；不要只在当前四条输入上选择最佳参数。

## 10. 验证、复现与证据来源

### 10.1 本次验证

- 核对保留manifest、metrics摘要及配置快照；主表16项和补充记录来自 [retained_index.json](../results/retained_experiments/retained_index.json)。
- 对照V8快照，当前AeroCOVER核心、ST背景与OS1转换的已存C++/头文件/YAML/launch没有差异；当前算法二进制摘要匹配保留运行记录。
- 执行现有AeroCOVER core测试35项、ST背景测试7项，全部通过；评估测试22项通过。未重新编译整套工作区或重跑16项实验。
- 评估测试会提示可选BURST/pycocotools依赖缺失，但本次22项测试通过；HOTA/BURST未启用。不能把该可选提示说成进行了相应评估。
- 这些测试包括精确连通对照、full-chord、FIFO移除、索引不漏交、并行预筛、3-of-5、近轨迹出生抑制、自适应球壳和stale维护门等；测试通过不覆盖所有真实传感器异常。

### 10.2 可复现输入和配置

所有19条版本记录的原配置按SHA保存于 [retained_experiments/configs](../results/retained_experiments/configs)。完整body mask、raycast与tracker文件仍在源码目录；历史代码快照在 [AeroCOVER-V8 code_snapshot](../results/aerocover_four_scene_comparison_v8/code_snapshot)。

| 组/场景 | source SHA-256 |
|---|---|
| OPEN | 04c42b46f9e6e5c8dc116824e04beb8614b203a041df86840da8934dfc173b12 |
| MT | 75bf039c5bd36e38a228ad70c3d4a2bab6e3793821996f9adc4d642a39fe83df |
| OFFICE | f6ae323efbc0f14d0ce6171cd102836b865df9d5e4765711c6ce4466bbc7edc5 |
| FOREST | 93401b651d1517de664f054952a2d54fbe1c0c4df089ea40275a792a28613305 |


算法二进制汇总SHA：
- AeroCOVER：`bbe3606cdb24d3caf586789ba5b41d20ad2ec088517e095d58f50b58b7a73ad8`
- VoFOD及外部tracker：`0de24448282a83ffaffb8859e20f8f84920f992b15a871034363db7c32a05761`

大bag的摘要沿用原manifest；本次没有重新逐字节扫描所有大bag核验摘要。可执行配置、原始路径、输出是否存在和小文件完整性见保留索引；清理没有改变保留run_manifest/metrics。

### 10.3 GUI指令

本机需先加载ROS和工作区环境；直接打开新的终端不一定有rosrun/roslaunch。

```bash
cd /home/uav/lyk
source /opt/ros/noetic/setup.bash
source /home/uav/lyk/devel/setup.bash

# 以下每次只选择一个场景运行；需本地可用的图形显示环境。
/usr/bin/python3 /home/uav/lyk/src/mid360_multi_uav_sim/scripts/view_scenario.py OPEN --sensor paired --rviz
/usr/bin/python3 /home/uav/lyk/src/mid360_multi_uav_sim/scripts/view_scenario.py MT --sensor paired --rviz
/usr/bin/python3 /home/uav/lyk/src/mid360_multi_uav_sim/scripts/view_scenario.py OFFICE --sensor paired --rviz
/usr/bin/python3 /home/uav/lyk/src/mid360_multi_uav_sim/scripts/view_scenario.py FOREST --sensor paired --rviz
```

这里核对了入口和参数，**本次未重新启动GUI/仿真**。GUI默认YAML用于可视化；严格复现指标应使用保留source.bag及其旁边的scenario.yaml，而不是重新飞行后假定输入完全一样。

### 10.4 新回放示例

必须使用未占用的新输出目录，不覆盖保留结果。下例不是本次执行记录：

```bash
source /opt/ros/noetic/setup.bash
source /home/uav/lyk/devel/setup.bash
cd /home/uav/lyk

/usr/bin/python3 src/soft_vofod_evaluation/scripts/run_benchmark.py replay \
  --source results/retained_experiments/sources/OPEN/source.bag \
  --scenario results/retained_experiments/sources/OPEN/scenario.yaml \
  --algorithm AeroCOVER-Mid360 \
  --config results/retained_experiments/configs/AeroCOVER-V8_OPEN_AeroCOVER-Mid360.yaml \
  --require-source-manifest --replay-rate 1.0 \
  --output /media/uav/SU710/NEW_OPEN_AEROCOVER_RUN

/usr/bin/python3 src/soft_vofod_evaluation/scripts/run_benchmark.py replay \
  --source results/retained_experiments/sources/OFFICE/source.bag \
  --scenario results/retained_experiments/sources/OFFICE/scenario.yaml \
  --algorithm VoFOD-OS1 \
  --config results/retained_experiments/configs/V15_OFFICE_VoFOD-OS1.yaml \
  --require-source-manifest --replay-rate 0.15 \
  --output /media/uav/SU710/NEW_OFFICE_VOFOD_RUN
```

清理后的各实验replay.py是保留结果校验/报告兼容入口，不再依赖已删除V10，也不负责自动重跑；真正的新回放使用上述run_benchmark.py并指定新目录。全局源码的历史专用脚本可能仍描述已删除数据，不能据其文字判断当前数据还存在。

### 10.5 主要来源

- [AeroCOVER主流程](../src/aerocover_mid360/src/aerocover_core.cpp)、[CA tracker](../src/aerocover_mid360/src/ca_tracker.cpp)、[ST背景](../src/aerocover_st_background/src/background.cpp)、[ray几何](../src/aerocover_mid360/src/ray_geometry.cpp)。
- [当前VoFOD核心](../src/vofod_mid360/src/strict_baseline_core.cpp)、[节点](../src/vofod_mid360/src/vofod_nodelet.cpp)、[外部tracker](../src/lidar_tracker_mid360/src/lidar_tracker.cpp)。
- [保留实验索引与清理记录](../results/retained_experiments/INDEX_ZH.md)；对应V8/V13–V16的metrics和ERROR_AUDIT。
- [VoFOD论文](https://arxiv.org/pdf/2303.05404)、[论文关联节点代码](https://raw.githubusercontent.com/ctu-mrs/vofod/5dbe299321885fa85d9c49eb89e3433ee8e8f005/src/vofod_nodelet.cpp)、[导入版本检测配置](https://raw.githubusercontent.com/ctu-mrs/vofod/7da9f33a878a586588f6a626b75cfeacac7824f7/config/detection_params.yaml)。
- [上游tracker源码](https://raw.githubusercontent.com/ctu-mrs/lidar_tracker/a92b4db61060b47f1af6dcce122188ec021f2dcd/src/lidar_tracker.cpp)、[其默认配置](https://raw.githubusercontent.com/ctu-mrs/lidar_tracker/a92b4db61060b47f1af6dcce122188ec021f2dcd/config/tracking.yaml)。
- 来源文档记录了授权/再分发边界，本次未对外发布代码或声称原作者背书；实际部署或发表仍需独立的复现与来源审查。

## 附录：当前 AeroCOVER 与 tracker YAML

以下直接取当前文件。AeroCOVER先加载base再加载sensor覆盖，OS1覆盖文件不能单独当作全部参数。VoFOD的常量与场景族已在第5节列明；body mask的长pattern列表以文件和manifest为准，不在此重复数万条编号。

以下保留了文件中的历史注释。例如OS1注释中的“2.6 million rays”来自旧2048列设置，tracker覆盖注释中的“OPEN-only”未反映后来对MT的复用；这些注释不能覆盖上文已经核实的131072 rays、OPEN/MT覆盖范围和生效参数值。本次没有为整理文档而改动算法配置文件。

### A. AeroCOVER base

```yaml
# AeroCOVER-MVP research defaults. These are mechanism-check starting values,
# not statistically calibrated or claimed optimal parameters.
world_frame_id: world

input:
  points_world_topic: /uav1/mid360/points_world
  rays_checked_topic: /uav1/mid360/rays_checked
  sync_queue_size: 128
  expected_rays_per_bundle: 20000
  geometry_consistency_tolerance_m: 0.01

time:
  point_window_s: 1.00
  ray_fifo_s: 1.00
  unknown_timeout_s: 1.00

rays:
  # Keep shell evidence away from the occupied VALID_RETURN endpoint.
  return_endpoint_margin_m: 0.50
  no_return_trusted_range_m: 20.0
  valid_return_weight: 1.0
  no_return_weight: 0.25

cluster:
  min_points: 1
  max_extent_m: 3.0
  measurement_sigma_floor_m: 0.20

motion:
  max_speed_mps: 12.0
  time_bin_s: 0.02
  max_rms_m: 0.50

spatiotemporal:
  spatial_tolerance_m: 0.30
  temporal_gap_s: 0.010
  temporal_min_points: 3
  max_slice_extent_m: 1.50
  background_min_points: 3
  background_min_ratio: 0.20

candidate:
  association_gate_m: 1.0
  max_cv_residual_m: 0.20
  max_missed_scans: 2

evidence:
  component_radius_margin_m: 0.10
  component_radius_min_m: 0.15
  # Bound the robust evidence radius to a 1.5 m target diameter.
  component_radius_max_m: 0.75
  shell_gap_m: 0.05
  shell_thickness_m: 1.00
  obstacle_adaptive_shell: true
  shell_min_thickness_m: 0.20
  shell_obstacle_margin_m: 0.10
  shell_direction_bins: 42
  shell_sample_step_m: 0.10
  shell_length_norm_m: 0.20
  shell_bin_density_threshold: 1.50
  shell_coverage_threshold: 0.60
  min_observable_bins: 42
  min_supported_bins: 42
  min_shell_support_scans: 2
  min_supported_octants: 2
  spatial_hash_cell_m: 1.0
  rebuild_translation_m: 0.20
  rebuild_radius_fraction: 0.20
  # Full reference recomputation is a periodic audit; unit tests keep it at 1.
  reference_check_every_n_scans: 100
  reference_tolerance: 1.0e-9

birth:
  history_length: 5
  required_supports: 3

tracker:
  # Relax shell only for measurements near an already-born CA prediction.
  maintenance_min_shell_bins: 29
  maintenance_gate_m: 1.5
  max_missed_s: 1.00
  gate_chi2_3d: 11.345
  process_jerk_sigma_mps3: 2.0
  measurement_sigma_floor_m: 0.20
  initial_velocity_sigma_mps: 3.0
  initial_acceleration_sigma_mps2: 2.0
  max_position_sigma_m: 5.0

ablation:
  use_spatiotemporal_history: true
  use_whole_history_extent: false
  use_shell_evidence: true
  require_full_chord: true
  use_no_return_rays: true
  use_incremental_evidence: true
  use_ray_spatial_index: true

output:
  tracks_topic: /aerocover/tracks
  residual_points_topic: /aerocover/residual_points
  background_points_topic: /aerocover/background_points
  diagnostics_topic: /aerocover/diagnostics
  markers_topic: /aerocover/markers
  publish_markers: true
```

### B. AeroCOVER OS1覆盖

```yaml
input:
  points_world_topic: /uav1/ouster/points_world
  rays_checked_topic: /uav1/ouster/rays_checked
  expected_rays_per_bundle: 131072
  sync_queue_size: 8

# A full 1 s OS1-128 FIFO is about 2.6 million rays. Keep the ray evidence at
# 0.50 s, and bound point connectivity to 0.50 s and the 40 m score domain plus
# 2 m shell allowance. No ray is sampled or dropped by these point settings.
time:
  point_window_s: 0.50
  point_max_range_m: 42.0
  ray_fifo_s: 0.50

performance:
  connectivity_threads: 1
  shell_prefilter_threads: 8
  ray_insertion_threads: 8

evidence:
  spatial_hash_cell_m: 2.0
  min_observable_bins: 42
  min_supported_bins: 42

tracker:
  maintenance_min_shell_bins: 29

output:
  tracks_topic: /aerocover_os1/tracks
  residual_points_topic: /aerocover_os1/residual_points
  background_points_topic: /aerocover_os1/background_points
  diagnostics_topic: /aerocover_os1/diagnostics
  markers_topic: /aerocover_os1/markers
  publish_markers: false
```

### C. VoFOD外部tracker基础参数

```yaml
# general parameters
min_onboard_detection_count: 2
buffer_length: 10
transform_lookup_timeout: 0.5 # seconds
message_throttle_period: 1.0 # seconds

# Bounded event-time join used by the per-frame completion contract.  The same
# capacity bounds each lossless subscriber-to-worker FIFO; overflow shuts the
# tracker down visibly instead of replacing an older message with the latest.
# Inputs with duplicate or regressive stamps are rejected before processing.
frame_completion:
  max_pending_frames: 128

# Canonical B0 rejects the occupied background published by VoFOD-Mid360.
background_filter:
  enabled: true

# if there is no current track, publish a dummy message to reset the Rviz marker
no_track:
  publish_empty: false
  empty_state: [0.0, 0.0, -20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  empty_covariance_value: 0.0

# used just for visualization
prediction:
  horizon: 2.0 # seconds
  sampling_period: 0.5 # seconds

# parameters of input pointcloud filtering
input_filter:
  # leaf size of the VoxelGrid filter applied to the input pointcloud. Set to zero to disable.
  downsample_leaf_size: 0.25 # meters

# parameters of the system model, used for the LKF
lkf:
  # Paper Eq. (33): per-update variances, squared Table II standard deviations.
  Q:
    position: 0.0001
    velocity: 0.04
    acceleration: 0.09
  # Initial state variances.
  P:
    # radius = multiplier * sqrt(cbrt(det(position covariance))).
    radius:
      multiplier: 1.5
      # Adapted value: minimum summed association gate is 1.2 m.
      min: 0.6 # metres
      # if the radius of P gets larger than this value,
      # the track will be deleted
      max: 3.0 # metres
    # these values will be used for initialization of P0
    init:
      position: 0.09
      velocity: 1.0
      acceleration: 1.0
  # Fixed position measurement variance (m^2).
  R:
    coeff: 0.09

# parameters for the cluster-to-track association algorithm
association:
  clustering_tolerance: 0.5 # metres
  # used to filter out invalid clusters
  cluster:
    min_points: 1
    max_points: 262144
    max_size: 1.5 # metres
    min_background_dist: 0.1 # metres
```

### D. OPEN/MT tracker覆盖

```yaml
# OPEN-only v2 rollback explicitly approved by the user.
input_filter:
  downsample_leaf_size: 0.5
association:
  clustering_tolerance: 1.0
  cluster:
    max_size: 1.0
    min_background_dist: 1.0
lkf:
  P:
    radius:
      min: 2.5
      max: 5.0
```

### E. VoFOD raycast公共配置

```yaml
# Faithful VoFOD-Mid360 B0 soft free-space contract.
raycast:
  free_update_weight_valid_return: 0.003
  # Strict baseline starts with upstream's single ray weight. A different
  # no-return weight requires an explicit sensor-only calibration.
  free_update_weight_no_return: 0.003
  valid_return_safety_margin: 0.25
  max_distance: 20.0
  reliable_no_return_distance: 20.0
  direction_norm_tolerance: 0.0001

# Pattern-index mask applies equally to VALID_RETURN and NO_RETURN.  The empty
# list is explicit for the simulation gate; a calibrated hardware body mask is
# required before flight use.
body_mask:
  enabled: true
  default_allow: true
  blocked_pattern_indices: []

# These non-baseline variants are forbidden in faithful B0 and stay disabled.
dynamic_aware_free_weighting: false
delayed_map_commit: false
```
