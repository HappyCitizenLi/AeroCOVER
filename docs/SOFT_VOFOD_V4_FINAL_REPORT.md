# SOFT-VoFOD V4 当前实现、实验结果与分析总报告

日期：2026-08-24

算法源码快照：`fd37bd0b41c90bc9b91c780b02bbf6e2f25d81a0`（本文后续提交只修改文档）

状态：**development candidate；未冻结为 final V4；不满足真实飞行或 T-RO/IJRR 最终投稿证据门槛。**

本文是当前 V4 实现与证据的总入口。专题文档继续保留公式、协议和复现细节，但结论以本文为准。

## 1. 结论先行

V4 没有另起一套 detector/tracker，而是在唯一的 `soft_vofod_mid360` production path 内完成四类修正：

1. 用 effective angular cells、soft geometry 和 range-conditioned return probability 重写 opportunity
   observation model；
2. 用静态点假设与 CV 假设的顺序似然比替代一次性 unknown-motion 判定；
3. 实现显式两阶段 dormant reacquisition、truth-only occlusion 评价和 LOS 各向异性 covariance 候选；
4. 增加真正零背景 cold-start 协议、因果 provenance 修正和 Mid360 record-only 标定入口。

实验给出的净结论不是“V4 已全面优于 B0”，而是：

- **当前最强正结果是 sequential epistemic birth。** S08B 的 HOTA 从 `0.4971` 提升到 `0.9204`，
  FN 从 `256` 降到 `52`，TTFT 从 `18.2 s` 降到 `5.2 s`；S08C 仍为 0 false confirmation，
  IT11 mapped-free hover recall 仍为 1。但 TTFT 仍比严格 `<5 s` 门槛慢 `0.2 s`。
- **新 opportunity 的 calibration 改善，但 track miss update 仍不安全。** O3 在 CAL10–14 上的
  Brier/NLL/ECE 最好；在严格 paired S05 gate 中却把 HOTA 从 `0.6559` 降到 `0.4544`，FN 从
  `137.4` 增到 `189.4`。因此只保留 diagnostics，不更新 existence。
- **两阶段重激活安全但无收益。** wrong reactivation 为 0，但 HOTA/FN 不变，报告延迟增加
  `0.20 s`；默认关闭。
- **在线 OCCLUDED 没有得到实证。** S05 的 4 个 truth-occluded frames 全部漏判，recall 为 0；
  当前真正起作用的是 dormant memory，不是 occlusion state。
- **LOS covariance 方向合理、收益不足。** 单个 S04 seed 仅提升 HOTA `0.001148`、减少 2 个 FN，
  不能进入默认配置。
- **cold-start 协议可运行，但性能门失败。** CS01–CS05 都完成过 smoke；同一 CS03 source 的错误确认
  经因果修正从 `4→1→0`，但五个场景的 p95 都超过 `100 ms`，且分属不同修正 commits。
- **没有真实 Mid360 指标，也没有 BL1/BL2 external baseline。** 因而不能冻结算法、生成论文最终主表、
  声称统计优于外部方法或进入闭环飞行。

## 2. 当前默认算法到底是什么

### 2.1 唯一数据流

```text
points_world + rays_checked
  -> 10 ms micro-batch / 5 Hz map epoch
  -> observed/certified free + candidate/stable background
  -> F: certified-free violation / U: unresolved packet
  -> sequential static-vs-CV epistemic inference
  -> provenance-preserving trajectory birth
  -> track-conditioned packet ownership
  -> CV/CA IMM + rectangular Hungarian
  -> survival / reportability / optional opportunity miss
  -> active / occluded / dormant / pre-reactivated / deleting
  -> reportable world-frame tracks
```

算法节点只读取公共的 `points_world` 和 `rays_checked`，不读取 visibility truth、scenario events、
Gazebo model state 或目标真值。truth 只进入独立 evaluator。V4 没有引入平行 package、JPDA/MHT、
scene-specific branch、旧 tracker 输入模式、`nadir_seed` 或 standalone launch。

### 2.2 当前 canonical 开关

当前仍复用 `soft_vofod_v3_canonical.yaml`；这是因为 V4 final gate 未通过，尚未创建虚假的“frozen V4”
配置。运行时以 canonical YAML 和 launch override 为准，而不是 `core.h` 中便于构造测试对象的字段默认值。

| 模块 | 当前值 | 实际含义 |
|---|---:|---|
| `sequential_unknown_inference` | `true` | U packet 必须通过顺序静态/运动判别才可 birth |
| `dormant_reacquisition` | `true` | 保留 V3 一阶段 dormant old-ID 恢复 |
| `effective_opportunity_cells` | `true` | 计算并发布 effective opportunity diagnostics |
| `range_conditioned_opportunity_return` | `true` | diagnostics 使用 range-conditioned return table |
| `opportunity_aware_existence` | `false` | opportunity 不参与 existence miss update |
| `two_stage_dormant_reactivation` | `false` | PRE_REACTIVATED 候选实现保留但不启用 |
| `packet_anisotropic_los_covariance` | `false` | 默认仍用原 measurement covariance |
| 普通场景 warm-up | `10 s` | 保持与既有 canonical 协议一致 |
| `cold_start: true` warm-up | `0 s` | 从首个稳定输入开始计成本，不隐藏初始化延迟 |

### 2.3 代码与职责

| 位置 | 当前职责 |
|---|---|
| `src/soft_vofod_mid360/include/soft_vofod_mid360/core.h` | map、packet、track、IMM、lifecycle、V4 diagnostics 数据契约 |
| `src/soft_vofod_mid360/src/core.cpp` | 唯一 map/event/birth/tracking/opportunity/dormant 实现 |
| `src/soft_vofod_mid360/src/soft_vofod_node.cpp` | ROS 参数、同步输入、输出和 diagnostics；不含第二套算法 |
| `src/soft_vofod_mid360/config/soft_vofod_v3_canonical.yaml` | 当前未冻结 V4 所复用的 canonical 参数 |
| `src/soft_vofod_mid360/launch/soft_vofod.launch` | ablation 开关及 cold-start warm-up override |
| `src/soft_vofod_evaluation/scripts/evaluate_bag.py` | tracking、map、epistemic、occlusion、cold-start、runtime 指标 |
| `src/soft_vofod_evaluation/scripts/run_benchmark.py` | record-once/replay-many、hash、coverage、warm-up 合同 |
| `src/mid360_spherical_capability_probe/launch/hw_record_only.launch` | 硬件 spherical pipeline 与标定 bag 录制，不启动算法 |

算法核心、头文件和 canonical YAML 的 SHA-256 分别为：

- `core.cpp`: `1d1b4f14ab2492f769cf57a9301cede2a59aca2e132d245730760677ab841186`；
- `core.h`: `a3718a7f22ba98b0f67095be9c07bc7257f7c667807c548b9fee0b7c79bf4acb`；
- canonical YAML: `3e9fb88d622358b00f2722c4b5143485b89aea821f962d1976e40faef6f85aff`。

## 3. 算法实现细节

### 3.1 两级自由空间与背景地图

自由空间按独立 5 Hz map epochs 累积。一次 traversal 只能形成 `OBSERVED_FREE`；只有满足独立 epoch、
持续时间、valid-return-backed evidence 和 background surface guard 的 voxel 才升级为
`CERTIFIED_FREE`。同一 epoch 内更多相关 rays 只增加饱和 evidence，不伪装成更多独立观测。

静态 return component 先进入 candidate/unresolved；只有持久或被静态假设接受后才进入 stable
background。confirmed target 的有限邻域可被保护，但 raw endpoint 和 raw event 不再直接写长期支持。
这切断了 V1 中 `event→support→地图无法纠正→更多 event` 的正反馈。

### 3.2 F/U packet 与 provenance

检测证据分成两种互不冒充的 provenance：

- `F`：目标返回违反了在目标出现前已经认证的 free space；
- `U`：区域尚无足够背景/free 知识，只能保留为 unresolved。

F 可以快速支持 mapped-free hover；U 不能仅因轨迹平滑就称为目标。birth 与 maintenance 分离：严格
provenance 约束新轨迹，已有轨迹预测门内则允许 ordinary return packet 维护，因此低速或 hover 目标
不会因 unknown-motion birth gate 被无条件删除。

### 3.3 Sequential epistemic birth

每条 U chain 同时维护两个假设：

- `H_B`：世界坐标中的静态点；
- `H_T`：带白加速度过程噪声的 CV pre-track。

每个独立时间组只累计一次 log-likelihood ratio：

\[
\Lambda_k=\Lambda_{k-1}+\log p(z_k\mid H_T)-\log p(z_k\mid H_B).
\]

当前固定 `tau_T=tau_B=6`，pre-track acceleration sigma 为 `1.0 m/s²`，initial velocity variance 为
`1.0 m²/s²`。只有 `Lambda>=6` 才允许 U-birth；`Lambda<=-6` 或超时则判为 static 并同化。旧的一次性
`D²` 判定不再 fallback，F/U chain 也不混合。

### 3.4 Cold-start 因果修正

真正零背景 CS03 暴露了两个此前 10 s target-free protocol 会隐藏的问题：

1. 固定地图边界上的静态墙只有部分可见时，component centroid 会随视角变化，形成伪运动链；
2. t=0 已存在的 stationary unknown 会先被看作静态，随后其自身附近形成的 free history 又把它升级成
   certified-free violation，造成循环论证。

当前修正为：紧贴固定 map boundary 的 U component 不允许 birth；static decision 写入受 voxel map
上界约束的 `static_unknown_history_voxels_`；随后与该 provenance 重叠的 F packet只可维护既有轨迹，
不能创建新轨迹。同一 CS03 source 的 false confirmation 因此从 4 降到 1，再降到 0。

这不是把 unknown birth 全局关闭：CS02 moving unknown 仍能出生。但它承认一个不可绕开的信息边界：
没有先验 free、外观或运动时，静止未知物体与静态背景在纯几何返回上不可辨识。此时诚实输出应是
unresolved/assimilation，而不是“静止 UAV 已确认”。

### 3.5 Packet ownership、IMM 与关联

新 birth 使用跨时间的短时空间 packet；已有轨迹在 prediction gate 内做 point ownership 唯一的
track-conditioned extraction。CV/CA IMM 完成 mixing、两模型 prediction/update likelihood、mode
probability normalization 和 moment matching，再由 rectangular Hungarian 做一对一 association。

这部分沿用已通过 V3 S04 gate 的组合：单独 IMM 会减少 FN，却可能制造 IDSW；和 conditioned ownership
联合后才同时改善 gate miss、FP/FN、fragmentation 和 identity。V4 没有再引入更重的 JPDA/MHT。

### 3.6 Lifecycle、dormant 与两阶段候选

existence 与 reportability 分离，状态包括 tentative、active、occluded、dormant、pre-reactivated 和
deleting。dormant 保存有时间上界的旧 ID/posterior；一阶段恢复继续作为 canonical 路径。

V4 候选两阶段路径为：

```text
DORMANT -> PRE_REACTIVATED --第二条 ordinary packet/1 s 内--> CONFIRMED_ACTIVE
                         \--超时或不兼容---------------------> DORMANT
```

第一条 compatible F/U packet 可更新旧 ID，但不报告；第二条普通 packet 才恢复 active。该机制避免单包
误激活，却没有改善当前 S05 的 packet shortage/FN，因而默认关闭。

### 3.7 Effective opportunity observation model

V3 把相邻 raw rays 当成独立 Bernoulli trials，并把 ray 与 target sphere 的硬相交等同于实体被有效照明：

\[
P_D=\min(P_{cap},1-\prod_j(1-q_j)).
\]

在非重复 Mid360 pattern 中，相邻方向和相邻时刻高度相关；raw count 会让 `P_D` 过快饱和，稀疏未返回
随后被解释为强 miss，existence 被过度压低。V4 将相关方向合并为 bounded angular cells，使用 soft
geometry、遮挡权重和 fill factor，并拆分为：

\[
P_D=P_{illum}P_{return\mid illum},\qquad
P_{illum}=1-\exp(-\lambda n_{eff}).
\]

每个 sample 输出 raw opportunity、effective cells、angular coverage、`P_illum`、
`P_return|illum`、最终 `P_D` 和 matched outcome。由于 tracking gate 失败，当前这些量只用于诊断；
`P_D` 不进入 existence miss update。

### 3.8 LOS covariance 候选

候选 measurement covariance 使用真实 ray direction `u`：

\[
R=R_{spread}+\sigma_\perp^2I+
(\sigma_\parallel^2-\sigma_\perp^2)uu^T.
\]

其中 `sigma_perp²=sensor_variance+sampling_floor`，
`sigma_parallel²=sigma_perp²+shape_sigma²`，没有新拟合参数。CAL16 确认误差沿 LOS 更大，但单 seed
tracking 收益太小，所以实现保留、默认关闭。

### 3.9 硬件标定入口

`hw_record_only.launch` 只启动硬件 spherical pipeline、capability probe 和 rosbag recorder，不运行
detector/tracker。预设记录 `points_raw`、`rays_raw`、`scan_identity`、`points_world`、`rays_checked`、
observer pose、target truth、time-sync/ray-source diagnostics、`tf` 和 `tf_static`。

当前工作区没有真实 Mid360 bag、PCAP/LVX、RTK/mocap/UWB truth，也没有两次断电周期的 exact
spherical-ray capability 记录。因此真实 `p_return`、grazing/no-return reliability、surface bias、LOS
covariance 和 opportunity calibration 全部仍是未测量项。

## 4. 实验协议与证据范围

### 4.1 数据边界

- CAL01–16 只用于 calibration/机制诊断；S/CS/NEG 场景用于 test 或 control；
- paired comparison 只录一次 source，各 variants 回放相同 source SHA-256；
- source 可包含 truth，但算法输入只给公共 TF、`points_world` 和 `rays_checked`；
- truth、track、timing frame coverage 都要求至少 95%；
- 不允许按场景改算法参数，也不允许把不同 commit 的 smoke 拼成 final main table；
- 相同 Gazebo seed 不能保证 bit-identical bag，因果结论只取同一个 immutable source 内的 paired replay。

### 4.2 当前 V4 artifacts

| 实验 | artifact | metrics 数 | 覆盖 | 结论 |
|---|---|---:|---|---|
| opportunity calibration | `artifacts/v4_opportunity_cal` | 15 | CAL10–14，O1–O3 | O3 calibration 最好 |
| opportunity tracking gate | `artifacts/v4_r5_opportunity` | 40 | **S05，N0/N1，5 seeds，O0–O3** | gate 失败 |
| epistemic triple gate | `artifacts/v4_epistemic_gate` | 6 | S08B/S08C/IT11，U0/U1 | 强正结果，TTFT差0.2 s |
| dormant gate | `artifacts/v4_reactivation_gate` | 2 | S05 seed1005，R0/R1 | 无净收益 |
| LOS gate | `artifacts/v4_los_gate` | 4 | CAL16/S04 seed1004，M0/M1 | 单 seed、小收益 |
| cold-start smoke | `artifacts/v4_cold_start` | 5 | CS01–05，N0 seed2101 | 混合 commits、runtime失败 |

共保留 72 份 V4 metrics。需要特别纠正证据边界：`v4_r5_opportunity` 的 40 行实际全部是 S05，
不是 S01–S07 全场景矩阵；它足以否定当前 opportunity miss update，但不能代替 Phase 13 的统一
S01–S07/N0–N2/10-seed final matrix。

大体积 bags 已在 metrics、manifest、timeseries 和 logs 校验后删除；现有数值仍可审计，但重新做 paired
实验时必须录制一组新的共同 source，不能把两次独立录制当成严格 paired samples。

## 5. 实验结果

### 5.1 Opportunity calibration：O1/O2/O3

CAL10–CAL14 的 15 个 runs 得到：

| variant | Brier | NLL | ECE | high-confidence miss |
|---|---:|---:|---:|---:|
| O1 raw geometry | 0.195970 | 0.824616 | 0.167771 | 151 |
| O2 effective cells，无 range return | 0.261285 | 0.716540 | 0.341558 | **34** |
| O3 effective cells + range return | **0.137974** | **0.432318** | **0.097680** | 35 |

O3 同时给出最低 Brier/NLL/ECE，说明 effective-cell + range-return 的概率输出比另外两个候选更接近
仿真 target-return outcome。但它只校准了仿真分布，尤其 30 m+ return probability 仍是 fallback，不能
外推到真实 Mid360。

### 5.2 Opportunity tracking gate：O0–O3

S05、N0/N1、seeds 1001–1005，同源 paired 共 40 runs：

| variant | HOTA | FP | FN | frag | IDSW | stale >3 s proxy |
|---|---:|---:|---:|---:|---:|---:|
| O0 no opportunity miss | **0.655932** | 35.9 | **137.4** | 10.6 | **0.0** | 0.822782 |
| O1 raw | 0.585220 | **10.0** | 174.1 | **3.0** | **0.0** | 0.322683 |
| O2 effective/no range | 0.408244 | 23.1 | 196.0 | 4.4 | 0.6 | 0.712681 |
| O3 effective/range | 0.454376 | 12.2 | 189.4 | 3.6 | 0.6 | **0.323082** |

O3 相对 O0：FP `-23.7`、fragmentation `-7.0`、stale proxy `-0.4997`，但 FN `+52.0`、HOTA
`-0.201556`。10/10 paired HOTA 都更差，Wilcoxon `p=0.0009765625`。这是统计明确的 recall/HOTA
退化，不是随机波动，故 `opportunity_aware_existence=false`。

### 5.3 Sequential epistemic triple gate：U0/U1

同源 seed1008：

| 场景/指标 | U0 | U1 sequential | 判定 |
|---|---:|---:|---|
| S08B HOTA | 0.497050 | **0.920358** | 显著改善 |
| S08B FN / FP | 256 / 0 | **52 / 0** | recall 大幅恢复 |
| S08B TTFT | 18.2 s | **5.2 s** | 大幅改善，仍未过 `<5 s` |
| S08C false confirmation | 0 | **0** | 无静态误确认回归 |
| IT11 hover recall | 1 | **1** | mapped-free hover 保持 |
| IT11 TTFT | 0.797 s | **0.797 s** | 无延迟回归 |
| worst runtime p95 | 94.079 ms | 96.334 ms | 两者均过 100 ms 门 |

S08B target 在 `14.596 s` 出现，首条 U packet 为 `14.821 s`，motion decision 为 `19.718 s`，decision
后约 `78 ms` 发布。主要延迟是取得足够可辨识的 sequential evidence，而不是 tracker 或 ROS 输出。

### 5.4 Dormant two-stage：R0/R1

S05 seed1005 使用同一 source；为触发 dormant，两者使用相同 opportunity geometry，唯一变量是两阶段开关：

| metric | R0 one-stage | R1 two-stage |
|---|---:|---:|
| HOTA | 0.577350 | 0.577350 |
| FN / FP | 176 / 12 | 176 / 12 |
| IDSW / frag | 0 / 3 | 0 / 3 |
| correct / wrong reactivation | 1 / 0 | 1 / 0 |
| pre-reactivation count | n/a | 3（F=1，U=2） |
| pre false / second-packet failure | n/a | 0 / 0 |
| visible→reportable | 1.6823 s | 1.8823 s |
| runtime p95 | 80.148 ms | 81.336 ms |

R1 证明两阶段状态机本身安全，但没有解决 S05 的检测缺口，反而增加固定的一包确认延迟；默认关闭。

### 5.5 Occlusion truth validation

S05 中共有 84 个 eligible track frames，truth-occluded 4 帧。在线状态混淆矩阵为
`TP=0, FP=0, FN=4, TN=80`，recall 0，false-occlusion rate 0。结果说明系统没有乱报遮挡，但也没有
检测到真实遮挡；不能把 `OCCLUDED` 写成已验证核心创新。当前 S05 的恢复主要来自 dormant fallback。

### 5.6 LOS covariance：M0/M1

CAL16 的 114 个 target packets 得到 LOS absolute error mean `0.270628 m`、perpendicular mean
`0.128558 m`、signed LOS mean `-0.216479 m`，支持 `sigma_parallel > sigma_perp` 的方向判断。

同源 S04 seed1004：

| metric | M0 isotropic | M1 LOS | delta M1-M0 |
|---|---:|---:|---:|
| true packet gate rejection | 0 | 0 | 0 |
| FN | 49 | 47 | -2 |
| fragmentation | 7 | 7 | 0 |
| HOTA | 0.968354 | 0.969502 | +0.001148 |
| runtime p95 | 80.583 ms | 82.665 ms | +2.082 ms |

baseline 已经没有 true-packet gate rejection，因此 covariance 形状没有可消除的主要 gate failure；
单 seed 的两帧 FN 差异不足以证明稳健收益，M1 默认关闭。

### 5.7 真正 cold-start：CS01–CS05

这些场景从 0 背景启动、SOFT warm-up 为 0，scoring 从约 `0.5 s` 开始。表中每行都是单个
N0/seed2101 smoke，不是统计均值：

| 场景 | commit | HOTA | TP/FP/FN | TTFT | 关键 cold 指标 | p95 |
|---|---|---:|---:|---:|---|---:|
| CS01 zero-background exploration | `6d52fc2` | 0 | 0/0/0 | N/A | false confirm=0；false packets=4.138/min | 131.061 ms |
| CS02 moving unknown | `235c8c6` | 0.890086 | 225/0/59 | 6.301 s | moving unknown 仍可 birth | 112.037 ms |
| CS03 stationary unknown | `c52c006` | 0 | 0/0/283 | N/A | false confirm=0；active IDs=0 | 120.713 ms |
| CS04 later certified corridor | `235c8c6` | 0.472953 | 68/0/236 | 24.1 s | recall/latency 明显不足 | 120.596 ms |
| CS05 known→unknown | `4a9220e` | 0.730035 | 186/42/121 | 2.601 s | ID保留；transition recall=0.374 | 112.289 ms |

CS03 的 HOTA=0/FN=283 不能简单解释为实现失败：场景 evaluator 把 stationary unknown 当 truth target，
而当前因果规则在无先验 free/外观/运动时有意不确认它。对 epistemic safety 来说 false confirmation=0 是
正确结果；对纯 tracking recall 来说它必然记为全漏检。这正是场景定义与可辨识性边界之间的冲突。

五行来自不同修正 commits；只有 CS03 在当前 provenance 实现 `c52c006` 上复验，所以不能横向组成
“current V4 cold-start table”。此外 5/5 p95 都超过 `100 ms`，当前 cold-start runtime gate 明确失败。

## 6. 根因分析

### 6.1 为什么 calibration 最好的 O3 仍显著降低 HOTA

`P_D` calibration 和“可以安全地作为 unmatched-track 的负证据”不是同一件事。现有证据支持以下链条：

1. effective cells 修正了单帧 raw-ray 独立性，但相邻时间窗、相似扫描方向和连续 track state 仍相关；
2. calibration outcome 是“目标附近是否有有效 return”，existence update 需要的却是“已有 track 在当前
   状态下若真实存在，是否应产生一个最终可关联 packet”；二者之间还隔着遮挡、packet shortage、
   component ownership、gate 和 association；
3. 一次物理 return 若未形成或未关联 packet，当前维护链会把它当成 miss；较高的 calibrated `P_D`
   因而仍可能产生过强负似然；
4. existence/reportability/deletion 是非线性的。一旦降过阈值，轨迹进入 dormant/deleting，后续恢复要
   再付 birth/reacquisition 延迟，最终表现为更长漏检段和 FN，而不仅是平滑的小分数变化。

O3 同时降低 FP、fragmentation 和 stale ghost，说明 miss update 确实在清理轨迹；问题是它无法区分 false
track 与“真实但本帧未形成可关联 packet”的 track，清理强度超过了 detector 当前 recall 能承受的范围。
下一轮需要标定 track-level packet observability/association likelihood，而不是继续在 S05 test source 上
调一个 cap。

### 6.2 为什么 S08B 仍比 5 s 门限慢 0.2 s

首包到 decision 用时约 `4.897 s`，decision 到 publish 只有约 `0.078 s`。因此瓶颈不是发布、Hungarian
或 Kalman update，而是 `Lambda` 达到 `tau_T=6` 前需要的独立运动证据。未知目标初段位移小、return
稀疏、U chain 的静态与 CV likelihood 尚接近，5 Hz map epoch/独立组约束又有意拒绝相关样本重复计票。

简单降低 `tau_T` 虽可能越过 5 s，却会直接消耗 S08C/CS03 的零误确认安全裕量。合理路径是在 CAL-only
数据上改进 likelihood/noise model 或独立组利用率，再用 S08B/S08C/IT11 三联 gate 验证；不能对 S08B
单场景追 0.2 s。

### 6.3 为什么 CS03 曾出现 4 个错误确认，最终又变成全 FN

初始错误的三个来源是边界墙的部分可见 component 漂移，另一个来源是 stationary object 用自身造成的
free history 给自己制造 F provenance。boundary guard 先消掉前三类；短期 quarantine 仍会过期，故只能
把 4 降到 1；将 static-unknown provenance 持久化到 bounded voxel index 后，循环论证才被根除并降到 0。

最终全 FN 是同一原则的另一面：系统拒绝从不可辨识的静态几何中伪造 target evidence。要检测此类目标，
必须增加任务允许的新信息，例如目标出现前 free history、外观/反射特征或其他传感器，而不是放松因果门。

### 6.4 为什么 cold-start p95 超过 100 ms

普通 replay 多数约 `80–96 ms`，cold-start 却为 `112–131 ms`。CS03 diagnostics 中单帧输入固定
20,000 rays，`observed_free_voxels` 峰值约 73k，单个 map epoch 的 free voxel 工作量峰值约 93k。
零背景启动把大规模 ray carving、candidate/background assimilation、certification、map growth 和
voxel diagnostics/publication 集中到评分早期；10 s warm-up 场景则把这段成本隐藏在评分前。

这是由 profiling counters 支持的工程推断，尚未做逐函数 cold-start flame/profile 归因。下一步应先量测
map update 与大消息 publication 的独立耗时，再减少重复 voxel traversal/序列化；不能通过恢复 warm-up
来“修复”指标，因为那只会重新隐藏成本。

### 6.5 为什么 two-stage 和 OCCLUDED 没有改善 S05

两阶段机制只回答“第一包是否立即报告”，不增加新 packet，也不修复 opportunity 导致的 existence drop。
R0 已经 wrong reactivation=0，因此 R1 没有错误可消除，只额外等待第二包，HOTA/FN 自然不变且 latency
增加。truth occlusion 只有 4 帧，在线 ray-based state 又一次都没触发；因此 S05 当前主问题仍是 packet
availability、maintenance recall 和 dormant 入口/出口，而不是缺少更多状态。

### 6.6 为什么 LOS 统计形态正确但收益很小

CAL16 的 LOS bias 明确大于横向误差，说明 covariance 方向合理；但 S04 M0 的 true-packet gate rejection
已经为 0，M1 无法从 association gate 中找回大量 observation。它只能轻微改变 filter weighting，故单 seed
只出现 `FN -2/HOTA +0.00115`。在真实 range×return-count 标定和多 seed 证据之前开启它只会扩大模型面。

## 7. 与历史 A1/A2/A3/B0 的关系

冻结 V1 开发矩阵为 S01–S07、N0、seed1001，共 28 runs：

| 方法 | mean HOTA | mean IDF1 | IDSW | mean p95 |
|---|---:|---:|---:|---:|
| B0 component/OBB + CA + duplicate merge | **0.676** | **0.619** | **13** | 53.3 ms |
| A1 raw endpoint + 2-group + greedy | 0.378 | 0.290 | 198 | 32.6 ms |
| A2 A1 + 3-group | 0.388 | 0.310 | 191 | 32.7 ms |
| A3 A2 + opportunity/feedback/Hungarian | 0.362 | 0.272 | 222 | 188.9 ms |

根因不是 Hungarian 或阈值单点错误，而是 observation 粒度不对等：B0 在 tracker 前把目标表面归并成
component/OBB center；A1–A3 把 raw endpoint 当原子，在未成熟地图上生成重复 event/hypothesis。
A2 多一组时间证据只能过滤偶然 birth，不能修复 target-level clustering、duplicate tracks 和 identity。
A3 又将 event supports、track feedback 和 opportunity 捆绑，提升部分 recall 的同时放大 ghost、ID churn
和复杂度。

S07/A3 是该正反馈的极端证据：HOTA `0.087`，TP/FP/FN=`176/15434/14`，mean/p95 processing
`293.3/1001.6 ms`，末帧仍有 189 条 confirmed tracks。observer motion/voxel discretization 触发大量
false violations；event support 阻止背景纠正，产生更多 event/birth/track；tracks 又扩大 support 和
opportunity 工作量。B0 使用相同输入仍为 HOTA `0.995`、FP=0，说明仿真几何只是触发器，算法正反馈才是
相对退化的主因。V2/V3/V4 已删除 raw-event support、改为 packet/map epoch/voxel index 和 bounded
lifecycle，旧 A1/A2/A3 仅保留为历史失败证据，不再是当前消融。

B0 的 10 s warm-up 必须满足严格 `target_spawn > warmup_complete`：B0 在一帧开头读取
`warmup_active`，帧末才置完成；刚好等于 10 s 的那一帧仍会把目标当背景。runner 现已检查 input
subscriber readiness、first-input acknowledgment、显式 completion stamp 和首个 scored frame 状态。
普通 S01–S07 source 的 target-free gap 为约 `10.409–10.495 s`；cold-start 则有意取消 SOFT warm-up，
并把 B0 自身初始化成本保留在 TTFT/FN 中。

## 8. Phase 状态与证据门

| Phase | 状态 | 当前结论 |
|---|---|---|
| 0 pre-implementation audit | 完成 | 冻结 V3/B0、公式、数据边界 |
| 1 effective opportunity cells | 完成 | bounded cells + soft geometry |
| 2 calibration metrics | 完成 | Brier/NLL/ECE/reliability 可评价 |
| 3 opportunity tracking gate | 完成、失败 | 当前 artifact 为 S05 40 runs；O1/O2/O3 均劣于 O0 |
| 4 sequential inference | 完成 | 无旧 D² fallback |
| 5 epistemic triple gate | 完成、未全过 | 强改善；S08B=5.2 s 未满足 `<5 s` |
| 6 two-stage dormant | 完成、失败 | safe，无收益，+0.20 s latency |
| 7 ray occlusion | 完成、失败 | truth recall=0，不作核心贡献 |
| 8 LOS covariance | 完成候选 | 单 seed 小收益，默认关闭 |
| 9 cold-start | 协议完成、性能失败 | CS01–05 可运行；CS03=0误确认；runtime/continuity不足 |
| 10 HW record-only | recorder完成、采集阻塞 | 缺物理数据、真值和现场授权 |
| 11 external baselines | 未完成 | 仅 BL0；缺 BL1/BL2 detector |
| 12 final freeze | 未通过 | 不创建 frozen V4 |
| 13 final matrix | 未运行 | freeze/baseline 前置条件失败 |
| 14 real offline replay | 未运行 | 无 real bags |
| 15 flight validation | 未运行 | 无真实证据且不满足安全前置条件 |
| 16 documentation | 完成 | 本文汇总实现、结果、负结论和边界 |

## 9. Baseline、统计与可复现性

### 9.1 Baseline 状态

BL0 为冻结 VoFOD-Mid360 B0，提交 `4cc1fe7`，由
`tclv_evaluation/launch/b0_canonical.launch` 启动。它与 SOFT 共享 source/input/truth/scoring/replay
合同。B0 的内部 10 s 初始化在 cold-start 中必须计入成本。

当前没有可独立运行的 BL1 range-adaptive clustering + temporal consistency + IMM/Hungarian，也没有
BL2 multi-frame accumulated-cloud detector + IMM。`lidar_tracker_mid360` 只是 B0 的 tracker，SOFT
内部 O/U/R/M ablation 也不是 external baseline。当前没有对外部方法的统计优势 claim。

### 9.2 最终统计尚未发生

真正 final matrix 必须在一个冻结 commit 上执行 final scenes × N0/N1/N2 × 10 seeds，报告 mean/std、
median/IQR、bootstrap 95% CI、paired tests 和多重比较校正。现有 72 个 V4 runs 是 phase gates：有些是
完整 paired 小矩阵，有些是单 seed smoke；不能拼成论文主表。

### 9.3 构建与测试

最近一次全工作区验证结果：

- `catkin build`：11/11 packages，0 warnings，0 failures；
- `catkin_test_results build --all`：463 tests，0 errors，0 failures，0 skipped；
- 其中 `soft_vofod_mid360` 150 个 core gtests + 2 个 pipeline tests，仿真包 94 tests，evaluation 18 tests，
  Mid360 capability probe 15 tests。

本文为 docs-only 修改，完成后只需要 Markdown/diff 校验，不重复运行昂贵仿真。

## 10. 当前可以与不可以主张的内容

### 10.1 有实现和证据支持

- 单一、truth-isolated、ray-aware 的在线 map→packet→birth→track production path；
- observed/certified free 与 candidate/stable background 的分层；
- F/U provenance 和 static-vs-CV sequential epistemic birth；
- cold-start boundary/static-unknown provenance 对 CS03 false confirmation 的因果修正；
- track-conditioned ownership、CV/CA IMM、Hungarian 和 bounded dormant old-ID memory；
- effective opportunity 的仿真 calibration diagnostics；
- 完整 evaluator/runner/hash/coverage/cold-start/HW record-only 工程链；
- 公开报告未通过的 opportunity、occlusion、two-stage、LOS 和 runtime 结果。

### 10.2 当前不能主张

- frozen/final V4 全面优于 B0；
- 已完成 S01–S07/N0–N2/10-seed V4 最终矩阵；
- opportunity-aware miss update 是净正贡献；
- OCCLUDED state 已可靠检测真实遮挡；
- two-stage reacquisition 或 LOS covariance 有统计显著收益；
- 优于 BL1/BL2/现代 external baselines；
- 真实 Mid360 opportunity/LOS calibration 成立；
- real offline、multi-UAV flight、机载实时性或闭环安全已验证；
- 当前证据足以形成 T-RO/IJRR 最终主张。

## 11. 后续工作的最短闭环顺序

1. 在当前 commit 上统一重跑 CS01–CS05，并用 profile 先把 cold-start worst p95 压到 `<100 ms`；
2. 只在 CAL split 改进 sequential likelihood/独立组利用率，再重跑 S08B/S08C/IT11，使 S08B 稳定
   `<5 s` 且不牺牲零误确认和 hover；
3. 将 opportunity 从 target-return calibration 升级为 track-level packet observability/miss likelihood，
   先过同源 S05 gate，再扩大到 S01–S07；
4. 修复 S05 packet availability/FN 后再判断 two-stage 是否还有存在价值；
5. 采集 HW-CAL01–05，完成两次断电 capability 验证并冻结真实 range×angle return/LOS 参数；
6. 实现真正独立的 BL1/BL2；
7. 只有上述门全部通过，才冻结 V4、运行 N0/N1/N2×10-seed final matrix、real replay 和 flight
   validation。

在这些前置项完成前，继续增加状态、分支或论文包装不会补上缺失证据；当前正确动作是保持未通过候选
关闭，并优先闭合 runtime、epistemic latency、opportunity recall 和 external/real-data 四个硬缺口。

## 12. 专题文档索引

- `V4_PRE_IMPLEMENTATION_AUDIT.md`：实现前审计、物理/统计边界；
- `SOFT_VOFOD_V4_ALGORITHM.md`：精简算法路径；
- `SOFT_VOFOD_V4_OPPORTUNITY_MODEL.md`：observation model 公式；
- `SOFT_VOFOD_V4_EPISTEMIC_BIRTH.md`：顺序判别与不可辨识性；
- `SOFT_VOFOD_V4_REACQUISITION.md`：dormant/two-stage/occlusion；
- `SOFT_VOFOD_V4_REAL_DATA_CALIBRATION.md`：HW-CAL 协议；
- `SOFT_VOFOD_V4_EXTERNAL_BASELINES.md`：baseline 公平合同；
- `SOFT_VOFOD_V4_EXPERIMENT_PROTOCOL.md`：数据分割、paired replay 和统计协议；
- `SOFT_VOFOD_V4_ABLATION.md`：O/U/R/M 单因素摘要；
- `SOFT_VOFOD_V4_KNOWN_LIMITATIONS.md`：限制清单。
