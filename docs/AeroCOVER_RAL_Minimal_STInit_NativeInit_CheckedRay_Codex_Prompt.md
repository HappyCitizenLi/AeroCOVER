# AeroCOVER：共享时空背景 STInit + NativeInit 的精简科研验证提示词

> 将本文件完整交给能够访问现有工作区的 Codex。本版替代此前所有实验提示词，不叠加执行。  
> **主实验不变：同一 Ouster OS1-128 数据上的完整 AeroCOVER 与原始 VoFOD + 原生 tracker。**  
> **本次修订：移除 PersistentInit，替换为 `VoFOD-Mid360-STInit`，直接复用 AeroCOVER 的时空背景判别实现，不再使用持久结构认证。**  
> STInit 与 NativeInit 均保留上一版要求的固定上游原始地图提交/执行机制和 checked-ray 接口；NativeInit 仍使用原始测距仪辅助初始化。  
> Ouster 主对比、多目标、HOTA、五项消融及六条 Mid-360 补充输入不变：仍为 29 条正式传感器记录、94 次方法运行。STInit 替换旧分支，不另加第三个 VoFOD-Mid360 版本。  
> 本轮仅做科研所需的软件修改、仿真和离线评价，不包含真实 Mid-360 数据包、标定、实飞或实体无人机操作。完成清单不代表保证 RA-L 录用。

## 1. 任务边界：只围绕论文核心问题工作

你是一名 LiDAR 飞行目标检测与三维跟踪研究人员。请在现有 ROS Noetic / Gazebo Classic 工程内继续工作，保留 AeroCOVER 的检测与跟踪主线，不另起项目，不建设通用工程平台。

本次只回答三个问题：

1. **算法比较：** 在同一 Ouster 观测和 observer 位姿下，AeroCOVER 相比保持原机制的 VoFOD，检测、定位和多目标跟踪表现如何？
2. **机制贡献：** 时空背景处理、扫描内尺度判别、完整弦 shell 和时序确认是否各自提供了有效贡献？
3. **Mid-360 适用性与背景建立：** AeroCOVER 在 Mid-360 上表现如何？VoFOD 使用原始测距仪初始化，与使用 AeroCOVER 相同的时空背景判别建立背景时，分别表现如何？

第三项是同 Mid-360 输入上的适配与背景建立机制比较；跨传感器差异另由配对子集补充。两者都不是实际载荷、功耗、续航或真实硬件可靠性的验证，不从仿真分数推导这些结论。

### 1.1 分支的范围与命名

本轮两个 Mid-360 VoFOD 分支固定为 `VoFOD-Mid360-STInit` 和 `VoFOD-Mid360-NativeInit`。**STInit 表示使用 AeroCOVER 的时空背景判别进行背景初始化与在线补充，不表示持久结构认证的新名称。** 原 `VoFOD-Mid360-PersistentInit` 退出新 launch、方法列表和汇总表；旧 Adapted、Seeded、Oracle、同步提交版也不增加为额外方法。

保留旧代码、历史结果及共用消息定义，不进行清库。删除的是本轮执行路径中的 persistent certificate、11-scan hit-history 认证和固定 10-scan 等待，不删除 AeroCOVER 本身约 1 s 的点历史与历史背景标签传播。旧结果不能只改名称后当作 STInit 结果。

当前报告说明旧 fork 改过初始化、checked-ray 输入和地图提交 [S1，第 5.1 节]。本版保留“恢复原始地图执行”的既定要求，并将额外背景机制改为 AeroCOVER 的现成模块。**NativeInit 的检测器目标是原始初始化/检测/地图执行加 checked-ray 适配；STInit 在相同基础上，以共享时空背景模块替换背景来源。** 两者都不是未经修改的 Ouster 原版，原始算法主对比仍由 `VoFOD-Original-OS1` 承担。

两条 VoFOD 分支共用 checked-ray 接口、原始地图执行、背景来源以外的检测规则和同一外部 tracker。STInit 的共享背景模块及其种子接入是明确声明的分支改动，不能再声称它“只修改了接口”。它不接入 AeroCOVER 的 shell、候选确认或跟踪器，也不把原始 VoFOD 的候选聚类替换成 AeroCOVER 的 residual observations。

本轮不做：全面工程审计、通用传感器抽象层、多层哈希验收平台、CI/容器化、统一 tracker 对比系统、额外大型第三方基线、完整参数网格、八组合交互消融、长遮挡重识别、多等级误差扫描或全面性能优化。

多目标是能力验证，不新增 JPDA/MHT、学习型关联、重识别网络或“新型关联算法”贡献。保留 AeroCOVER 的 CA、Hungarian、S35 和已有缓存、索引；不引入 PCFT、PlanePatch 或目标 core 判据。

### 1.2 保留当前 AeroCOVER 方法

先读取实际存在的核心代码和实现报告，不盘点整个仓库。报告中的工作区是 `/home/uav/lyk`，以执行环境为准。重点位置为：

```text
src/aerocover_mid360/src/aerocover_core.cpp
src/aerocover_mid360/src/ray_geometry.cpp
src/aerocover_mid360/src/ca_tracker.cpp
src/aerocover_mid360/src/aerocover_node.cpp
src/aerocover_mid360/config/aerocover_mvp.yaml
src/vofod_mid360/src/vofod_nodelet.cpp
src/vofod_mid360/src/strict_baseline_core.cpp
src/vofod_mid360/src/ray_update.cpp
src/vofod_mid360/src/voxel_map.cpp
src/vofod_mid360/config/b0_mid360_canonical.yaml
src/vofod_mid360/UPSTREAM.md
src/lidar_tracker_mid360/config/tracking.yaml
src/soft_vofod_evaluation/scripts/evaluate_bag.py
src/soft_vofod_evaluation/scripts/run_benchmark.py
src/mid360_multi_uav_sim/config/benchmarks/
```

当前方法保持为 [S1，第 4 章]：

```text
当前点 + 约 1 s 历史点的三维空间连通
  -> 历史背景标签传播 + 扫描内时间切片尺寸检验
  -> 当前残余回波、运动补偿与目标支持球
  -> 仅使用此前扫描的 full-chord 方向自由空间证据
  -> strict observations 与 S35 maintenance 分流
  -> 已生轨迹 CA + 一对一 Hungarian
  -> 未分配 strict observations 的 candidate 关联与 3-of-5 birth
  -> 当前射线最后进入历史
```

跨帧连接不是四维距离聚类；实际点历史不能因清理无效配置被改成 0.1 s；S35 只能维护已生轨迹。所谓“无地图”仅指检测不依赖持久占据地图与已知背景种子，不扩大到导航系统。42/42 是离散的历史方向支持，不是连续闭合自由表面或无人机语义类别的充分证明。

## 2. 固定比较关系，不允许再把主次颠倒

| 层次 | 对比对象 | 输入与初始化 | 作用 |
|---|---|---|---|
| **主实验，必须完成** | `AeroCOVER-OS1` 与 `VoFOD-Original-OS1` | 同一份 OS1-128 仿真记录、同一 observer 位姿；VoFOD 保留原测距仪辅助初始化，AeroCOVER 不使用该种子 | 同传感器、原生端到端算法比较，包含单目标与多目标 |
| **Mid-360 补充比较** | `AeroCOVER-Mid360`、`VoFOD-Mid360-STInit`、`VoFOD-Mid360-NativeInit` | 同一份 Mid-360 输入与 observer 位姿；两个 VoFOD 分支共用原始地图执行机制及 checked-ray 适配；仅 NativeInit 使用测距仪背景种子 | 六条既有配对输入上的适用性与背景建立策略比较 |
| **Mid-360 核心消融** | `AeroCOVER-Mid360` 及其五个消融版本 | 与补充比较相同的六条输入，AeroCOVER 后端与其余参数保持一致 | 验证非重复扫描条件下的机制贡献 |
| **跨传感器补充，不新增场景** | `AeroCOVER-Mid360`、`AeroCOVER-OS1`、`VoFOD-Original-OS1` 的已完成配对结果 | 相同场景状态和预定义轨迹，分别模拟两种传感器 | 描述传感器与算法组合差异，不替代主算法比较 |

**同传感器不等于所有输入信息完全相同。** 原始 VoFOD 额外使用测距仪；这是保留其正常工作条件，必须在表注中说明。不要为了“相同输入”而拿掉它的测距仪，也不要给 AeroCOVER 接入地面种子。上述所有方法均不加载预建环境背景地图。Mid-360 补充比较也应标明：NativeInit 使用测距仪辅助建立背景，STInit 和 AeroCOVER 不使用该种子。

主表评价各自正常检测与跟踪流程。由于后端不同，端到端 HOTA 提升不能全部归为某个前端模块或新的关联算法；前端贡献由保持相同 AeroCOVER 后端的消融说明。本轮不再建立共同 tracker 系统。

### 2.1 原始 VoFOD 的最小接入

优先使用官方 VoFOD 与官方 `lidar_tracker` 的匹配版本，读取其现有 launch、传感器配置和初始化流程。实现报告给出的来源是 `ctu-mrs/vofod` 与 `ctu-mrs/lidar_tracker`；它们是查找原生代码的依据，不意味着本地修改版就是原版 [S1，第 5.1 节]。

原始版本必须保留背景初始化、占据更新、raycasting、floating 搜索、地图提交/执行机制及原生 tracker。允许少量编译兼容、topic、消息字段、TF/外参和 launch 适配，不把 STInit、旧 persistent-structure 或同步 map-commit 实现移植进 Ouster 原始基线。

只需在研究报告中记录官方来源、实际 commit 和少量兼容修改。不为此建设审计平台。旧 `lidar_tracker_mid360` 不能仅因名称相近就被当成官方原生 tracker。

在一个开放和一个建筑短开发片段上，检查地图是否覆盖预先规定的工作空间及必要搜索余量、坐标是否一致、测距仪是否有效，以及是否能在正常条件下初始化和检测。旧 Mid-360 地图配置不直接照搬；也不读取测试目标未来轨迹来逐条定制地图。

以原方法合理配置为起点，最多检查默认配置及两组有物理意义的替代配置，联动考虑体素、聚类和背景距离。开发片段可复用 P01/P02 的布局，但需要实际生成 Ouster 观测；旧 Mid-360 bag 不能改名充当 Ouster 数据。AeroCOVER 的传感器配置也只在开发数据上小预算校准，测试前固定，不逐场景找最优参数。

### 2.2 Ouster 与测距仪：只做必要输入适配

优先复用现有 MRS/Gazebo 的 Ouster OS1-128 扫描器与测距仪插件。只补足真实缺少的消息、时间和几何转换，不开发通用传感器框架。记录实际使用的线数、列数、帧率、视场和扫描时间组织即可。

同一次 Ouster 采样产生两种视图：原始 VoFOD 所需的原生点云/属性，以及 AeroCOVER 所需的点和射线。有效返回、无效记录、逐点时间和几何应能对应，不能重新采样一份更有利的数据给其中一方。

AeroCOVER 的必要适配只有：**支持 Ouster 实际输入数量与时间组织，复用现有几何和核心算法。** 不再强制所有输入恰好为 20,000 条，也不把 Ouster 默认抽成 20,000 条来迎合旧接口；Mid-360 原有输入检查仍可保留。不伪造无返回或缺失射线。

仿真必须让点、射线方向与起点来自同一运动时刻，例如有效返回满足：

$$
p_r=o_r+\rho_r d_r.
$$

不直接把 Mid-360 扫描图样改名为 Ouster；不把多目标分别扫描后拼接成一帧。近似 OS1 扫描模型如被采用，明确标为近似，不宣称复现了真实硬件全部扫描细节。

保留原始 VoFOD 所需的 Garmin/测距仪输入。优先用现有插件沿安装轴测距，不用无人机世界坐标高度冒充测量；没有确切 Garmin 型号模型时标为仿真测距仪，不虚构硬件参数。AeroCOVER 的检测不订阅该种子输入。

**资源边界：** 主对比不是“有空再做”的补证。链路缺失时优先完成最小原生接入；若确实需要重建传感器引擎或缺少不可替代的依赖，就如实标记主对比未完成，并继续可执行的 Mid-360 补充比较、消融和 HOTA。不得用两个 Mid-360 分支替代原始 Ouster 主对比，也不能改称剩余实验已完成主比较。

### 2.3 Mid-360 双分支：共享时空背景与原始初始化

**本次只替换背景来源，不重写 VoFOD 或 AeroCOVER。** 优先将 AeroCOVER 当前的背景判别抽成可直接调用的小模块，STInit 和 AeroCOVER Full 复用同一实现、参数和标签语义。两者各自维护运行状态；不要求先运行完整 AeroCOVER，再把它的检测结果喂给基线。

| 项目 | `VoFOD-Mid360-STInit` | `VoFOD-Mid360-NativeInit` |
|---|---|---|
| LiDAR 输入 | 共用 world-frame points + checked-ray 接口 | 与 STInit 相同 |
| 背景来源 | AeroCOVER 的跨帧连通、历史背景传播、扫描内时间切片尺度判别 | 固定上游 VoFOD 的测距仪辅助初始化 |
| 背景模块运行方式 | 每扫描在线产生背景点，供种子建立与后续补充 | 保留上游种子生成与后续背景处理方式 |
| 初始化等待 | 不设固定帧数等待；首次有效种子按原始入口生效后允许检测 | 按上游实际就绪条件，不继承 STInit 或旧 lag |
| 持久结构认证 | 完全退出新路径，无 stable-cell/PCA 认证兜底 | 同样禁用，不自动切换初始化策略 |
| 地图提交与执行 | 固定上游的执行入口、调度、必要同步与读写关系 | 完全相同 |
| 其余检测及 tracker | 保留原始候选聚类、close/far、floating、OBB、cleanup，以及上一版共用外部 tracker | 与 STInit 相同 |
| 明确的检测器改动 | checked-ray 适配 + 共享时空背景及种子接入 | checked-ray 适配 |

最后一行是实现目标，不是宣称现有代码已满足。共用 tracker 可沿用上一版的 `lidar_tracker_mid360`，只记录已有适配差异，不另开 tracker 重构任务。

#### 2.3.1 “跟 AeroCOVER 一样”指相同的点级背景判别

读取 `aerocover_core.cpp` 中 shell 之前的实际代码，复用以下处理 [S1，第 4.3–4.5 节]：

| 环节 | 必须保持的语义 |
|---|---|
| 输入点 | checked-ray 校验后的原始有效返回点，保留 scan ID、逐点时间与 original index；不能先用 VoFOD 的 0.50 m 体素代表点代替 |
| 历史窗口 | 与 AeroCOVER Full 相同的约 1.00 s 点 FIFO、过期规则和历史背景标签 |
| 连通 | 当前点与历史点的精确三维半径连通，当前阈值 0.30 m；不是四维距离聚类 |
| 历史传播 | 历史背景点不少于 3 个且占历史点比例至少 0.20 时，将整个 component 标为背景 |
| 尺度判断 | 按 scan、时间、原始索引排序；同 scan 内相邻时间差不超过 0.010 s 的 slice，至少 3 点才检查 |
| 大背景条件 | 任一有效 slice 的任一轴跨度超过 1.50 m，则整个 component 为背景；不是整段历史轨迹的跨度 |
| 输出 | 当前点的背景标签/索引，并按原实现写回历史标签；不输出目标确认或 shell 判定作为背景依据 |

例如，对有效 slice 的点集，使用其轴向跨度：

$$
e_a=\max_{p\in Q}p_a-\min_{p\in Q}p_a,
\qquad a\in\{x,y,z\},\qquad
\max(e_x,e_y,e_z)>1.50\ \mathrm{m}.
$$

其中 Q 是按相同 scan 内相邻时间间隔分出的、至少含 3 点的 slice，e 是其 AABB 跨度。直接复用原实现，不把 0.010 s 的相邻间隔条件改成整个 slice 的总时长上限。

以上是实现报告中的起始参数。若 AeroCOVER Full 在开发阶段统一调整过背景参数，STInit 使用同一最终参数组，不另设更宽松的背景阈值。历史点数为零时不触发历史传播；单个扫描也可按已有尺寸规则发现背景，不为“填满 1 s”增加强制等待。

**AeroCOVER 的这部分是逐帧背景判别，不是仅启动时运行一次。** 本版将“初始化”落实为 VoFOD 的背景来源：共享模块持续运行，用于首次种子与新观测背景的补充，不新增“一次性初始化 vs 持续更新”实验。背景标签的历史只来自该模块自身，不把 VoFOD 地图、检测结果或 tracker 输出反向当作历史标签，避免两边声称共享判据、实际却使用不同证据。

建议直接抽取/复用背景子函数，并保持 AeroCOVER Full 抽取前后的背景点与 residual 语义不变。每个方法独立实例化 FIFO 和标签状态，消融实例也不能修改 STInit 的 Full 配置。无需运行 AeroCOVER 的完整节点或维护 shell 射线索引，不应仅为获得背景标签而支付整套检测跟踪开销。

#### 2.3.2 只将背景结果接入 VoFOD 的种子入口

以下是本次新增的接入设计，不是原报告已实现的功能。令当前有效点集合为当前 scan 的原始返回点，背景模块输出标签后取：

$$
B_k=\{p\in P_k^{\mathrm{cur}}:b_k(p)=1\},
\qquad V_k^{\mathrm{seed}}=\{\operatorname{voxel}(p):p\in B_k\}.
$$

**共享的是点级背景判别，不是两算法最终的地图或候选集合。** 只将当前背景点落入的有效体素作为种子来源，去重后交给固定上游的背景种子/背景写入入口，使用原始种子赋值与地图保护语义。具体函数、可见时机和锁从上游代码确定。没有可复用入口时只加一个薄种子入口，不自行设计新的占据递推。

不填充 component 的整个 AABB、凸包或邻近空白体素，不把 residual 自动写成自由空间，也不把整个历史 FIFO 每帧重复注入地图。当前背景点的种子注入与正常点更新属于不同职责，按上游 seed 与点更新的关系处理；同一调用链不能意外执行两次整帧点证据更新。旧 hit-history、stable cells、PCA 主轴认证以及 certificate 在 close/far 中的直接否决都退出新路径。

**VoFOD 的检测输入仍是完整当前有效点云。** 不先删掉 AeroCOVER 判定的背景点再运行 VoFOD，不直接使用 AeroCOVER residual observations、运动补偿后的目标中心、shell 通过状态、3-of-5 或 S35。原始 VoFOD 自行进行体素化、候选聚类、close/far、floating 和 OBB 检测，再交给共用外部 tracker。ST 标签只供种子写入，不额外添加“任一 ST 背景标签就否决整 component”的旁路规则；原始地图驱动的 close/far 仍正常工作。

图示仅表示数据依赖，不规定地图任务的线程调度：

```text
同一份当前有效返回点 + checked rays
  +-> 共享 AeroCOVER 时空背景模块（独立点历史，不运行 shell/tracker）
  |     -> 当前背景点 -> 原始 VoFOD 的背景种子入口
  |
  +-> 完整当前点云 + checked-ray 几何
        -> 原始 VoFOD 聚类、地图、close/far、floating、OBB
        -> 两条 Mid-360 基线共用的外部 tracker
```

STInit 不使用测距仪种子。首个非空、地图范围内的背景种子按原始写入入口生效后，触发该分支的背景就绪；没有种子时不假定背景已建立，也不能在第 11 帧自动放行。输入处理及合法地图更新可按上游允许方式继续，不能为等待背景而停止更新时空点历史。使用原有任务完成/可见性机制确认种子生效，不增加每扫描全局等待。后续种子按相同入口处理，地图维护与清理仍由 VoFOD 负责，不能因此宣称它也变成“无地图”。

同一背景体素中可能混入目标点，原始聚类也可能将目标与背景连接；这些结果不通过真值拆分或私自删除种子规避。AeroCOVER 与 STInit 相同的是背景点判别，后续地图离散化和候选构造不同，最终背景集合不必相同。

#### 2.3.3 保留原始地图执行与 NativeInit

使用与 Ouster 主对比相同的固定 VoFOD 版本；S1 提供的参考 commit 为 `7da9f33a878a586588f6a626b75cfeacac7824f7`。S1 没有给出完整线程模型，需从实际上游代码恢复回调、任务启动/等待、锁、地图可见性及 cleanup 的触发关系。不能将旧确定性同步 core 外包一个异步任务就称为原版，也不能删掉上游本来必要的同步。

checked-ray 只提供合法状态、逐射线时间、原点、方向与可信段，并接入原始 raycasting/更新入口。恢复原版的点、free-ray、floating、cleanup 调用关系，不借接口或背景模块之名保留旧整帧排序归并提交、固定检测后提交或逐扫描全局屏障。endpoint 保护等细节按上游实际代码取舍，不未经核对一律删除。背景模块输出按 scan 关联为只读种子输入，使用原始地图访问规则消费，不从独立线程无保护地写图。

**NativeInit 保持原始测距仪辅助逻辑。** 使用同一上游的种子生成、有效性检查、背景写入和就绪条件；关闭 ST 背景模块与旧 persistent certificate，不自动回退 STInit，不把收到首条测距消息等同于种子有效。测距沿安装轴由仿真插件产生，不用世界坐标高度、地面真值或人工背景点代替。需要补录时只处理原六条记录中受影响的实例，不新增场景。

两条 VoFOD 分支保持相同地图范围、非背景参数、checked-ray 规则、原始调度、外部 tracker 及正式回放条件。STInit 的背景参数与 AeroCOVER Full 共享，其余 VoFOD 参数与 NativeInit 共享；这是两组不同的控制关系，不能混写成三个方法的所有参数完全相同。保留开发集上的小预算校准，不分别为两条 VoFOD 分支挑最有利配置。

#### 2.3.4 只做三项接入检查

1. **背景一致性：** 在同一短开发输入上，从相同空状态、相同原始点和最终背景参数出发，比对共享模块与 AeroCOVER shell 前的当前背景点索引；按 scan ID 与 original index 对齐。验证历史传播和 slice 尺寸路径均实际被调用，不只比较背景点数量。复用已有记录，不新增正式场景或全仓审计。
2. **分支边界：** STInit 没有 hit-history/PCA certificate、固定 10-scan 等待或 Garmin 种子，其背景子模块不依赖 shell/AeroCOVER tracker；NativeInit 没有 ST 或 persistent 兜底。确认种子只进入原始背景入口，没有 residual 预过滤、整 component 标签旁路或重复点更新，且两分支均未退回同步 commit。
3. **最小运行：** 回放一条已有短多目标记录，检查种子有效/检测就绪、正常任务收尾与稳定 ID，无明显丢任务、死锁或重复写入。已有符合上一版要求的地图执行检查可复用；不要求异步地图结果逐位一致，不新增线程/调度敏感性实验。

有效输入下未形成背景或没有轨迹是保留的算法结果；编译或接入失败是未完成，不能填零分冒充已测。用三项小检查完成接入后，直接替换原 PersistentInit 的六次正式运行，不比较“旧持久结构 vs 新 ST”作为额外论文实验。NativeInit 未受影响且满足既定协议的结果可复用；共用代码变化实际影响它时才重跑。

## 3. 实验规模：保持上一版数量，以 STInit 替换 PersistentInit

### 3.1 Ouster 主实验：十五条单目标、六条多目标

沿用现有 MRS X500 observer 与目标无人机模型，不另换飞行平台。所有下列输入都运行 `AeroCOVER-OS1` 与 `VoFOD-Original-OS1`。每个传感器记录只生成一次，双方离线回放同一份数据，不分别飞一条近似路线作为唯一对照。

| 条件 | 数量 | 最小设计 |
|---|---:|---|
| S1 开放单目标 | 5 | 地面可见、远离大障碍，包含一次约 3 s 悬停 |
| S2 建筑近墙单目标 | 5 | 未参与调参的布局或区域，包含门框、近墙通过和一次短时遮挡 |
| S3 森林单目标 | 5 | 新树木布局，包含树干间转弯及距离变化 |
| M1 双目标接近—交叉—分离 | 3 | 先建立两条轨迹，再接近和交叉，检验混合回波、重复轨迹与身份交换 |
| M2 三目标并行—转弯—短遮挡 | 3 | 三个目标同时可见、先后转弯，其中一个经历约 0.3–0.5 s 遮挡 |

每条建议 40–60 s。复用已有 world 和轨迹工具，P01/P02 留作开发，不以重新命名充当未见测试。重复序列改变预先指定的轨迹扰动、相遇时刻或扫描相位；同一确定性 bag 重放五次不能算五个独立精度样本。

两种方法从同一记录起点、空状态开始。场景可包含统一的正常启动前缀，但长度与目标进入时刻在测试前固定，双方接收相同历史；不得给一方预先建好的地图。开放场景应提供原方法正常的地面观测条件，不刻意用无法初始化的工况作为唯一比较。算法未及时初始化的后果保留在结果中，不删除相关序列或目标 FN。

observer 使用预定义轨迹，不由某个被测 tracker 控制。多个目标必须同时参与场景求交和相互遮挡；真值 ID、目标数和分组点云只能用于离线评价，不输入算法。交叉保持机体无碰撞，记录实际最小间距，不自动增加专用分裂器或复杂关联模块来掩盖失败。

只检查多候选/多轨迹是否被错误截成单个、关联是否一对一、S35 是否仅维护、预测 ID 是否稳定。历史 component 合并引起的问题，区分背景误删、observation 合并和跟踪关联错误，保留一个代表案例即可。

### 3.2 Mid-360：只生成固定六条配对输入

预先选择 **S2 前两条、S3 前两条、M1 第一条、M2 第一条**，共六个场景实例，生成对应的 Mid-360 观测及 NativeInit 所需的同步测距仪/TF 数据。保持上一版子集不变：四条单目标、两条多目标。

每条运行 `AeroCOVER-Mid360`、`VoFOD-Mid360-STInit`、`VoFOD-Mid360-NativeInit`，构成三方法同传感器补充比较。后两个均使用上一版要求的原始地图执行机制。AeroCOVER Full 与消融共用结果，两个基线各六次，**仍为十二次基线运行，不叠加旧 PersistentInit、旧同步版本或新增输入序列**。

三种方法从同一时刻、空状态开始，使用同一条输入和预先固定的评分域。沿用相同启动前缀与目标进入安排，不给某个分支额外预热、不裁掉初始化阶段。测距仪是 NativeInit 的方法输入，不是人为赠送已完成背景图。

除统一 HOTA、Recall、RMSE、IDSW 外，只记录首次有效背景种子时间、检测就绪时间及每目标 TTFT/未确认比例。STInit 的“首次背景输出”“种子在 VoFOD 中生效”“检测放行”需区分，不能用 FIFO 满、模块运行过或目标检测成功反推初始化完成。AeroCOVER 没有同类地图初始化步骤，该栏为“不适用”，不填 0 s。种子就绪也不等于全场地图收敛。有效运行但无轨迹仍保留全部目标 FN；缺依赖或未运行标为未完成。

NativeInit 与 STInit 的差距用于说明同一 checked-ray 接口、原始地图执行机制及非初始化配置下背景建立策略的影响，不将其全部解读为未经适配的原始 VoFOD 性能。STInit 与 AeroCOVER 使用相同点级背景判别，可检验改善是否只来自背景建立，但两者仍有候选表示、地图与 shell 判据、确认和 tracker 等差异，不能把全部差距声称为 shell 的单独贡献；shell 的直接证据仍由 A3/A4 消融提供。四条单目标和两条多目标分别汇总；多目标各只有一个场景实例，是补充诊断，不冒充全面多目标统计。这六条同时承担三方法比较、AeroCOVER 自身验证与核心消融，不另造第二套 Mid-360 主实验。

优先在同一次确定性场景状态回放中生成两种 LiDAR 观测，以便配对比较。若已有工具只能分别运行仿真，使用相同预定义状态轨迹并检查实际差异；不能只因 waypoint 相同就写成完全相同运动。无法可靠配对时，保留 Mid-360 自身结果，但不作严格跨传感器排序，不为此重建仿真引擎。

**跨传感器结果直接复用上述六条与其 Ouster 对应结果，零新增场景、零新增算法运行。** 使用事先固定的共同几何评分域，不根据哪台传感器实际打到点来筛选目标。`AeroCOVER-Mid360` 对 `VoFOD-Original-OS1` 同时改变传感器和算法，只能作为系统组合差异；`AeroCOVER-Mid360` 对 `AeroCOVER-OS1` 才是同框架的传感器表现比较。

### 3.3 只保留必要背景误报与轻度噪声

建筑、森林各增加一条约 120 s 的 **Ouster 无目标序列**，两种主方法均运行，报告错误出生数和每分钟错误出生率。不为负样本计算 recall/HOTA 排名，也不把短时零误报解释为长期零误报保证。

新仿真统一采用一档预先固定的轻度测距噪声，不做噪声扫描。例如：

$$
\widetilde\rho_r=\rho_r+\eta_r,\qquad
\eta_r\sim\mathcal N\bigl(0,(0.03\ \mathrm{m})^2\bigr),\qquad
\widetilde p_r=o_r+\widetilde\rho_r d_r.
$$

这是研究仿真条件，不是任一真实 LiDAR 的标定结果。同传感器各方法接收相同扰动后的记录；两种传感器采用相同噪声水平，但射线不同，不声称噪声逐点相同。仅扰动有效返回，点与射线字段同步更新，丢失或无效数据不改成 NO_RETURN。本轮保留已有理想自运动几何，不另外实现位姿漂移与时间误差矩阵。

## 4. 核心消融：Full + 五个版本，均使用 Mid-360

消融固定使用第 3.2 节六条 Mid-360 输入。Full 结果直接复用；五个变体分别回放六条输入，共三十次变体运行。**不要把 Ouster 的 Full 与 Mid-360 的消融放在一起比较。**

STInit、NativeInit 是 VoFOD 的基线分支，不是 AeroCOVER 的 A6/A7。共享背景模块按各运行实例持有配置；A1/A2 的修改只作用于对应 AeroCOVER 消融，不能污染 STInit 或 Full。消融表仍为 A0–A5 六行，不增加初始化、地图执行与各消融的组合矩阵。

| 编号 | 版本 | 唯一指定改动 | 主要问题 |
|---|---|---|---|
| A0 | Full | 完整 AeroCOVER | 参照 |
| A1 | Current-frame background | 仅当前帧空间聚类与尺寸过滤，关闭跨帧背景联系及传播；保留射线历史 | 历史结构信息是否必要 |
| A2 | Whole-history extent | 用整个历史 component 尺寸替代扫描内时间切片尺寸，其余背景处理不变 | 是否将运动轨迹延展误当成大背景 |
| A3 | Without shell | 跳过 shell，保留背景处理、基本几何门、原候选逻辑和 3-of-5 | 自由空间证据是否具有独立作用 |
| A4 | Without full chord | 允许可信历史自由段与 shell 的部分交叠提供证据，保留方向与跨扫描条件 | 完整穿越是否排除表面伪支持 |
| A5 | One-shot birth | strict shell 不变，一次合格观测即可出生 | 时序确认对误报和首次确认延迟的作用 |

除指定因素外，输入、参数、CA 和适用的维护规则保持一致。A3 中 shell 已关闭，不再以隐藏的 `strict=false` 拒绝全部候选；按保留的基本几何资格接入原确认与关联流程，不另创维护算法。A5 同时解除最低三次观测计数，不能只改窗口参数。A4 始终不能使用当前帧自支持。

至少检查一次开关实际生效。没有输出或结果完全一致时，先排除隐藏条件和失效配置，不为了让消融下降而改阈值。A2 沿用既有尺度阈值，不新增逐变体最优参数搜索。

不再拆分方向数、scan 内封顶、S35、运动补偿、缓存或索引消融。只保存已有的背景删除、strict pass、确认与删除事件等少量 diagnostics。若展示方向证据，选定少数帧完成全方向计算，不能把 early-exit 未计算项画成零。

这组消融支撑 Mid-360 条件下的机制贡献；Ouster 上只报告完整方法结果，不把未经消融的传感器条件写成每项模块都已独立验证。

## 5. 评价：保留 HOTA，但只做薄适配

### 5.1 统一评分边界与基本指标

评价全部最终确认轨迹，包括算法正常发布的预测维持状态，不只挑最接近真值的一条。同一序列使用统一评分时刻与几何域，不用未来输出插值填补漏帧；目标存在且位于域内即参与评分，零回波、短遮挡和尚未确认不用于删除 FN。预测也按同一几何域处理，不以“靠近某个被忽略真值”为由宽泛删除输出。

固定 1 m 指标单独匹配：先建立门内合法边，优先最大化匹配数量，再最小化距离。不要普通 Hungarian 后直接删超门限 pair；也不要把该匹配表传给 HOTA。

$$
P=\frac{TP}{TP+FP},\qquad
R=\frac{TP}{TP+FN},\qquad
F_1=\frac{2TP}{2TP+FP+FN}.
$$

这些指标评价确认轨迹，不冒充跟踪前检测精度。位置误差和每个真实目标的首次确认时间定义为：

$$
\begin{aligned}
\mathrm{RMSE}_{p}
&=\sqrt{\frac{1}{N_{\mathrm{match}}}
\sum_{i=1}^{N_{\mathrm{match}}}
\lVert\widehat p_i-p_i\rVert_2^2},\\
\mathrm{TTFT}_{g}
&=t_{g,\mathrm{first\ confirmed\ match}}-t_{g,\mathrm{first\ eligible}}.
\end{aligned}
$$

无匹配时 RMSE 为 N/A；未确认目标单列比例，TTFT 不填零。此起点不同于旧报告的 first-visible，需重算。错误出生在首次发布帧进行一对一匹配，重复轨迹不能都认作正确。多目标辅助 `IDSW_1m` 按同一真实目标最近两次成功匹配的预测 ID 变化计数，漏检不清空记忆，退出评分域后重入不自动视作同一连续区间；它不是额外实现的官方 CLEAR 指标。

### 5.2 HOTA-3D-pos 的固定协议

复用官方 TrackEval 的 HOTA 核心 [R1、R2]，只写一个提供每帧 GT ID、预测 ID、三维位置相似度矩阵的薄适配层，记录所用版本/commit。**不手写另一套近似 HOTA，不沿用旧报告单门限 HOTA-style 数值。**

沿用上一版已明确的本项目位置相似度，不是宣称存在唯一通用的三维 HOTA 协议：

$$
S_{ij}(t)=\max\left(0,1-
\frac{\lVert p_i(t)-\widehat p_j(t)\rVert_2}{d_0}\right),
\qquad d_0=2.0\ \mathrm{m}.
$$

采用 19 个相似度门限：

$$
\mathcal A=\{0.05,0.10,\ldots,0.95\}.
$$

0.50 对应 1 m 合法距离边界，但 HOTA 的匹配目标不同，不保证与基本指标的匹配一致。输入完整相似度矩阵，不能先在 1 m 裁剪再计算全部门限。

下面公式仅用于解释与小测试，实际匹配和计数交给官方实现：

$$
\begin{aligned}
\mathrm{DetA}_{\alpha}
&=\frac{TP_{\alpha}}{TP_{\alpha}+FP_{\alpha}+FN_{\alpha}},\\
\mathrm{AssA}_{\alpha}
&=\frac{1}{TP_{\alpha}}
\sum_{(i,j):\,n_{ij}^{\alpha}>0}
\frac{(n_{ij}^{\alpha})^2}
{n_i^{\mathrm{gt}}+n_j^{\mathrm{pr}}-n_{ij}^{\alpha}}.
\end{aligned}
$$

匹配次数按当前门限统计，轨迹长度包括完整评分序列的漏检和误报，不只统计成功匹配。无 TP 时遵循官方零值约定，不令 AssA 等于 1。

$$
\mathrm{HOTA}_{\mathrm{pos}}
=\frac{1}{19}\sum_{\alpha\in\mathcal A}
\sqrt{\mathrm{DetA}_{\alpha}\,\mathrm{AssA}_{\alpha}}.
$$

表中写 `HOTA-3D-pos`，交代 2 m 尺度与门限集合。它不能与采用不同表示、相似度或评分域的公开基准直接横比，也不能由平均 DetA 与平均 AssA 开方替代。

适配层保留稳定 ID、空预测帧、重复轨迹和全部可评分目标；ID 在每条序列内统一映射，不逐帧重新编号，不按目标拆开算单目标分数再伪装成多目标 HOTA。不同序列分别评价。

只补六类小测试：完美单/多目标、漏检、额外/重复预测、双目标 ID 交换、距离边界、空集合。必要检查：十帧单目标，仅后八帧有位置完全正确的同 ID 输出，各门限下应得到 DetA = 0.8、AssA = 0.8、HOTA = 0.8。ID 交换应降低关联分数；整条轨迹统一改 ID 名称应不变。

所有正式有目标结果及消融离线计算 HOTA/DetA/AssA，包括两个恢复原始地图执行的 Mid-360 分支；保存每门限结果，不另加门限扫描实验。初始化失败的有效运行保留空输出和全部评分帧，不仅在成功初始化子集上算分。依赖无法取得时标记 HOTA 未完成，不能用旧公式填补，也不重建整套 ROS/Python 环境。

### 5.3 运行时间只保留一项必要比较

同一计算机、相同构建与可视化条件下，记录输入就绪至对应轨迹输出的完整流程 mean/p95。AeroCOVER 包含内部 tracker，原始 VoFOD 包含检测器及原生 tracker，两个 Mid-360 VoFOD 分支均包含检测器及共同外部 tracker；原有 detector-only 数字不能替代。计时包含各自在线输入转换；STInit 还须包含独立运行的时空背景模块、种子接入及其点历史维护，不能免费读取离线 AeroCOVER 背景结果或省略该开销。不包含离线 HOTA、真值评价、仿真器或绘图。

原始 Ouster VoFOD 与两个 Mid-360 分支均保持上游实际执行机制，不能为方便计时插入逐扫描同步等待。异步任务提交耗时不是完整地图更新耗时，并行模块耗时也不能相加冒充端到端延迟。优先在 ROS 输入/输出边界记录；暂时不能可靠对齐就分列可测部分并说明，不虚造端到端数值。

同一传感器的正式比较使用共同、预先固定的回放速率及资源条件，记录实际速率。降速可能改变异步任务的重叠情况，不能把不同速率结果混成同一条件，也不能假定恢复原始调度后分数必然与旧同步结果一致。本轮仅保留下一段已有的 1× 检查，不展开线程数、回放速率或调度敏感性矩阵。

从 M2 选第一条，对两种 Ouster 主方法各做一次 1× 回放，并对已有 Mid-360 对应的 Full、STInit、NativeInit 作同样检查，观察持续积压或丢帧。未初始化时单独记录该状态，不以“没有处理出轨迹”证明跟踪流程实时。不新增性能场景，不用离线降速结果宣称机载实时。达不到实时就报告代价，不通过隐式减少 Ouster 输入改善数字。

## 6. 两张表、两类图，不增加论文展示负担

| 产物 | 内容 | 展示原则 |
|---|---|---|
| 主对比表 | **主体为 Ouster 上 AeroCOVER 与原始 VoFOD**；追加 Mid-360 三方法小分块，使用同一六条子集 | 单/多目标分组；保留 HOTA/DetA/AssA、Recall、RMSE，初始化成功数/耗时与 IDSW 可用邻近短段说明 |
| 消融表 | Mid-360 的 A0–A5，同一六条输入上的 HOTA、Recall、误报/错误出生率、定位误差或 TTFT | 优先选择最能解释机制的列，不把全部 CSV 指标挤进正文 |
| 机制与轨迹图 | 近墙/树干的背景与 shell 判别，加双目标交叉前后轨迹 | 预测 ID 使用自身固定颜色，不用真值重新着色掩盖身份交换 |
| 稀疏性结果图 | 复用已有结果，按返回点数或距离分层的 Recall | 不同传感器分开标明；零回波下成功可能来自预测维持，不增加数据采集 |

Mid-360 小分块固定包含 AeroCOVER、STInit、NativeInit，不能继续使用 PersistentInit 标签。表注写明：**两条 VoFOD 基线共用原始地图执行和 checked-ray 接口；STInit 以 AeroCOVER 相同的点级时空背景模块提供种子，NativeInit 使用原始测距仪初始化。** 共享的是背景判别，不是完整检测器；STInit 属于背景模块替换的混合基线，NativeInit 也仍有接口及已声明的跟踪适配，两者不称为完整原版。

初始化成功数及耗时可用一句话说明，不另加初始化专表。跨传感器结果优先保存在 CSV，必要时从相同六条配对子集选用，不拿六条 Mid-360 与二十一条 Ouster 的总均值横比。空间不足时先省略跨传感器额外展示，不删掉 Ouster 主对比或把新增分支替换成主基线。仍保持两张表、两类图。

逐序列计算，再在相同条件内报告 mean ± std 和样本数；HOTA 先对门限平均，再对序列平均，标为序列宏平均，不称为官方跨序列 pooled 分数。单/多目标、不同传感器分别汇总。错误出生按时长归一化，IDSW 总数或均值注明。消融 Full 只能使用相同六条 Mid-360 数据。

保存 P/R/F1、HOTA/DetA/AssA、RMSE、逐目标 TTFT、未确认比例、IDSW、误报与运行时间，但不强制追加 IDF1/MOTA。小样本不建设复杂显著性或 bootstrap 平台。HOTA 提升但 AssA 不变时，不能把改进全部说成关联创新。

本轮仿真主比较、核心消融、必要可视化和运行时间按约两页组织，为后续真实 Mid-360 研究预留篇幅。八页论文不需要放下所有日志和逐序列表，但关键对比不能全部移出正文。本轮不负责撰写整篇论文。

## 7. 执行、数量与交付

按以下顺序推进：**确认固定上游与既定原始地图执行路径 → 复用 AeroCOVER 背景子模块并接入 STInit 种子入口 → 保留 NativeInit 并完成第 2.3.4 节三项小检查 → 运行受影响的既定实验 → HOTA 与结果汇总。** 原始地图执行或 Ouster 主对比尚未接通时继续完成已有要求，不将旧同步路径当作备选。HOTA 沿用已实现协议；未完成时先跑小测试。不为本次替换重做未受影响的合格实验。

尽量扩展现有 `run_benchmark.py`。需要薄封装时，可实现下列入口；它们是待实现建议，不代表当前已存在：

```bash
python3 scripts/run_paper_minimal.py --suite ouster-main
python3 scripts/run_paper_minimal.py --suite mid360-core
python3 scripts/run_paper_minimal.py --suite mid360-baselines
python3 scripts/run_paper_minimal.py --suite summarize
```

| 入口 | 固定工作量 |
|---|---|
| `ouster-main` | 15 条单目标 + 6 条多目标 + 2 条无目标 Ouster 输入，每条运行 AeroCOVER 与原始 VoFOD，共 46 次方法运行 |
| `mid360-core` | 6 条配对 Mid-360 输入，运行 Full + 5 个消融，共 36 次方法运行；Full 同时供三方法补充比较使用 |
| `mid360-baselines` | 复用 6 条输入，运行 STInit 与 NativeInit，共 12 次方法运行；不运行 PersistentInit，不重复 Full；已有合格 NativeInit 可复用 |
| `summarize` | 只读已有轨迹与真值，重算统一指标，导出 Ouster 主结果、Mid-360 三方法、消融和配对补充，不重新仿真 |

正式证据包仍为 **29 条传感器记录、94 次方法运行**：Ouster 46 次，Mid-360 Full 6 次、消融变体 30 次、STInit 6 次、NativeInit 6 次。不含短开发片段与计时检查。本次以 STInit 的六次运行替换旧 PersistentInit，不在 94 次之外添加新方法。若 NativeInit 的实现、输入和协议均未改变，可复用其结果；共用地图或背景模块抽取确实改变其他方法时，只重跑受影响项，不预先宣称所有旧结果都失效或都可直接复用。

六条 Mid-360 记录补齐测距仪/TF 后，仍是相同六个正式输入实例；若补录改变了 LiDAR 或位姿，应对同一新记录统一重跑受影响方法，不能混用旧 Full 与新基线。共用场景实例不意味着两种传感器是同一份点云。跨传感器补充与 HOTA 不增加方法运行；不另做 Ouster 消融或第三套多目标矩阵。

最低交付：必要代码/配置、原生 Ouster launch 与薄输入适配、共用原始地图执行路径、共享时空背景子模块与 STInit 种子接入、STInit/NativeInit 两份配置、HOTA 与小检查、可运行命令，以及以下结果或现有等价组织：

```text
results/aerocover_paper_stinit_native_mapexec/
  per_sequence.csv
  ouster_main.csv
  mid360_core.csv
  mid360_baselines.csv
  mid360_ablation.csv
  cross_sensor_paired.csv
  hota_per_threshold.csv
  figures/
  RESEARCH_SUMMARY.md
```

保留时间戳与预测 ID，记录输入、方法、配置、上游版本、背景来源、地图执行模式、checked-ray 适配、回放速率和 HOTA 协议。建议背景模式分别为 `aerocover_st` 与 `native_rangefinder`，地图模式为 `upstream_native`；这些标签必须对应实际调用路径。STInit 记录所复用背景实现和参数来源即可，不建新登记平台。`mid360_baselines.csv` 汇总三个方法，AeroCOVER 行复用 Full。初始化字段只保留首次有效种子/检测就绪时间、状态与失败说明。

旧结果不覆盖，大型 bags 不重复复制，不删除未提交修改。结果复用须满足输入、实现、背景来源、地图执行、配置及评价协议一致。旧 PersistentInit 无论使用哪种地图执行，都不能改标签后当作 STInit；旧同步 NativeInit 也不能填入原始地图执行结果。汇总仅接受本版正式方法，不靠目录名称猜测版本。

`RESEARCH_SUMMARY.md` 简要说明：PersistentInit 已退出、共享背景代码/参数与一致性结果、种子入口、原始地图执行与 checked-ray 边界、STInit/NativeInit 差异，以及主要结果、代价和运行状态。说明 STInit 是否改善了初始化、后续是否仍有 VoFOD 判别失败，不要求它一定变好。区分未运行、未形成背景、已就绪但检测/跟踪失败；一份简短报告即可。

NativeInit 与 STInit 共用接口、原始地图执行、背景来源之外的检测参数及外部 tracker。STInit 持续提供背景种子，因此对照的是背景建立与在线补充策略，不是仅启动几帧的差异。其他核心改动若确实存在应注明。AeroCOVER 对 STInit 是共享背景判别下的系统对比，不是只差一个 shell 开关；不能以 Mid-360 补充完成代替 Ouster 原始主对比。

完成固定证据包即停止。若环境受限，交付实际可用修改、已完成结果和准确后续命令，不虚构结果，不自动回到此前 P0–P9 工程清单。

## 8. 文档与公式要求

使用 UTF-8 标准 Markdown。行内公式使用单个美元分隔符，块公式使用独立行上的两个美元分隔符；数学命令使用单个反斜线，不依赖自定义宏。不要把需要渲染的公式放进代码块。检查表格列数、围栏闭合与数学语法；未在用户本机打开时，不声称已在其 VS Code 环境验收。

## 依据与修订说明

**[S1] 实现事实：** `CURRENT_AEROCOVER_VOFOD_IMPLEMENTATION_RESULTS_AND_ANALYSIS(5).md`，状态日期 2026-09-08。第 4.3–4.5 节支持本版复用的点历史、三维连通、背景传播与时间切片规则；第 5.1、5.5、5.6 节说明要移除的旧持久结构路径和既有适配边界；第 6–7 章说明旧评价及计时限制。该报告未实现 STInit，也不证明 NativeInit/原始地图执行已恢复。共享代码抽取、点级背景转种子、就绪与持续补充规则是本次新增接入要求，不冒充现有事实。

**[S2] 修订基础：** `AeroCOVER_RAL_Minimal_NativeMapExec_CheckedRay_Codex_Prompt.md`。保留原始地图执行、checked-ray、NativeInit、同 Ouster 主比较、多目标、六条 Mid-360 子集、五项消融和 HOTA-3D-pos。仅以 AeroCOVER 共享时空背景 STInit 替换 PersistentInit，正式输入与方法数量不变。

**[R1] HOTA 官方实现：** TrackEval，核心文件 `trackeval/metrics/hota.py`。来源：`https://github.com/JonathonLuiten/TrackEval`。

**[R2] HOTA 定义：** Luiten et al., *HOTA: A Higher Order Metric for Evaluating Multi-Object Tracking*。来源：`https://arxiv.org/html/2009.07736`。

**[S3] 原始地图执行的实现依据：** 第 2.1 节实际采用的固定 VoFOD 上游源码及其对应配置。参考仓库来自 S1：`https://github.com/ctu-mrs/vofod`。Codex 必须读取该版本的实际代码再恢复执行路径；本提示词未通过外部源码核查认定某个具体线程模型。

实验规模、配对子集和评分协议是本项目研究设计，不是已完成结果或期刊硬性规定。本轮不验证真实传感器。

**开始执行：移除本轮 PersistentInit 路径，复用 AeroCOVER 的点级时空背景模块实现 STInit，并将当前背景点接入原始 VoFOD 的种子入口。保留 NativeInit、原始地图执行和 checked-ray，不接入 shell 或替换 VoFOD 候选/tracker。在原六条 Mid-360 输入上替换对应基线结果；Ouster 主对比、多目标、HOTA、五项消融和 94 次矩阵不变。**
