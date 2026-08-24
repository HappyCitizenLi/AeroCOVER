# SOFT-VoFOD V4 实施前审计

日期：2026-08-24

仓库 HEAD：`4b818c7f11911ada5ddf7c7aa432584a3abc4b7d`

production core 冻结提交：`d6b923d6ab7247e5de3ea2753e2e686f9b5d634e`

evaluator/runner 冻结提交：`7e690b324561f440a29dfe848e18a84c96ed1b2f`

B0 正式冻结/最后修改提交：`4cc1fe71c20aae6f2110b925733c679f39dfd98e`

本文在修改 V4 production algorithm 或 canonical 参数前，冻结 V3 的真实执行路径、公式、参数、
实验状态与可执行边界。用户未跟踪的 `.vscode/`、`1.md`、V3/V4 prompts 不属于本阶段改动。

## 1. 审计范围

已核对：

- V3/V2 最终报告、V1 实验报告和当前 Mid360/MRS 实现说明；
- `soft_vofod_mid360` core、ROS adapter、V3 canonical config 和 core tests；
- evaluator、runner、正式 V3 manifests/CSV/aggregate；
- Mid360 仿真/硬件 raw-ray、preprocessor 和 capability-probe 接口；
- `artifacts/v3_r1`、`v3_r2_final`、`v3_r3`、`v3_r4`、`v3_r5`。

源码 SHA-256：

| 文件 | SHA-256 |
|---|---|
| `core.cpp` | `b66158e98c1c304de32f51ded3bb6e3770c2938f52a0b9adac7e056386085550` |
| `core.h` | `1f8df98542b6b28ce6a29fff4d1094b3fb5649b8666aafeca49371b4715e23ba` |
| `soft_vofod_node.cpp` | `9742643ffb73b69674ce959c1500e7a2d80285bd077f1c4d33f2e88558236d04` |
| V3 canonical YAML | `4fff86b01080f0bbb1b6e76eb70916119e6738a6158a0ac932632fe3790c2693` |
| `run_benchmark.py` | `945afbca0ca9779638825fe15500d0480b703f13121408346eba885948ceb973` |
| `evaluate_bag.py` | `b07695b5ecaac3a77dc9186e8bb38807f96c2f36da6b3e6f99895b5022a58e7d` |

## 2. 当前 production path

```text
points_world + rays_checked
  -> 10 ms micro-batch / 5 Hz deferred map epoch
  -> observed/certified free + candidate/stable background
  -> F certified-free violation 或 U unresolved packet
  -> 5-group CV trajectory birth
  -> track-conditioned maintenance packet
  -> CV/CA IMM + Hungarian
  -> opportunity + survival + existence/reportability
  -> active/occluded/dormant/deleting
  -> reportable tracks + confirmed-track-limited map protection
```

只有 `src/soft_vofod_mid360` 这一条算法路径；没有平行 V4 package、raw endpoint birth、raw-event
support、scene switch、GT visibility/occlusion 输入、JPDA/MHT 或额外 motion model。

## 3. 当前 opportunity observation model

### 3.1 参数

| 参数 | canonical 值 | 证据来源 |
|---|---:|---|
| `p_ret` 0–10/10–20/20–30/30+ m | 0.916/0.878/0.877/0.5 | 前三档 CAL04；30+ 未标定 fallback |
| detection cap | 0.95 | engineering |
| max ray range | 60 m | engineering |
| occlusion margin | 0.15 m | engineering |
| sigma-point scale | 1.0 | engineering |
| physical sphere radius | 0.75 m | engineering UAV support |
| angular-index direction chord | 0.05 | 固定源码常量，只用于候选加速 |

### 3.2 Sigma-point support

对 track position covariance 做 self-adjoint eigendecomposition，构造 mean 和三个主轴的正负点：

\[
\{\mu,\mu\pm\sqrt{\lambda_1}v_1,
       \mu\pm\sqrt{\lambda_2}v_2,
       \mu\pm\sqrt{\lambda_3}v_3\}.
\]

七点等权。每个点按 track CV velocity 外推到 ray timestamp；没有 orientation、projected area、实体
fill factor 或 packet-return history。

### 3.3 现有公式

`RayAngularIndex` 只减少 sphere-nearby ray 搜索范围；同一方向 cell 内的 raw rays 仍各自进入概率
乘积。对 raw ray `j`，七个 sigma points 中未被遮挡的 sphere intersections 形成：

\[
q_j=\sum_s \frac{1}{7}I_{js}p_{ret}(\rho_{js}),
\qquad
P_D=\min\left(0.95,1-\prod_j(1-q_j)\right).
\]

因此相邻 rays 被当作独立 Bernoulli trials；sphere intersection 等同实体 illumination；大量同 cell
rays 会把 `P_D` 过快推到 cap。这与 V3 R5 的 FN/HOTA 退化方向一致。

### 3.4 当前遮挡判据

仅使用在线 ray geometry：

- VALID_RETURN 的实际 range 比 target sphere near range 至少短0.15 m，则记 foreground occlusion；
- 另一条 `confirmed_active` track 的 sphere near range 至少短0.15 m，则记 target occlusion；
- NO_RETURN 不提供 foreground range；
- `occlusion_probability = occlusion_evidence / intersection_evidence`；进入阈值0.6。

擦边 sphere intersection 是硬0/1，没有 normalized miss-distance soft weight。S05 正式矩阵没有触发
online `occluded` transition；CAL09 仅证明专门几何可触发该路径。

### 3.5 Miss、survival 和 reportability

Bernoulli miss 保持：

\[
r^+=\frac{r^-(1-P_D)}{1-r^-P_D}.
\]

它只在 scan 无 match、非 occluded/dormant、opportunity 开启且该 scan 至少提交一个 map epoch 时
应用。多 micro-batches 的 `P_D` 再用独立并集公式合并。每 scan 先做 survival：

\[
r^-\leftarrow r\exp(-0.1\Delta t).
\]

Reportability 为：

\[
q=r\,f_{state}\exp(-t_{stale}/1.0)
  \exp(-\sigma_{pos}/1.5),
\]

阈值0.2；active/tentative、occluded、dormant/deleting 的 state factor 分别为1、0.35、0。

## 4. 当前 epistemic birth

F packet 来自 `CERTIFIED_FREE` violation，可允许 hover；U packet 来自 unresolved component。二者
provenance 不混合。正式 birth 都仍需5个独立时间组、至少0.1 s、CV residual≤0.8 m、速度≤15 m/s。

U-motion 使用同一 fitted chain 的前后半 packet endpoint：

\[
D^2=(\bar z_{late}-\bar z_{early})^T
(R_{early}+R_{late})^{-1}
(\bar z_{late}-\bar z_{early})>16.266.
\]

这里的式子应读为标准二次型
`dᵀ(R_early+R_late)⁻¹d > 16.266`；当前是一次性99.9% χ² hard gate，不维护 static/moving
sequential posterior。S08C 已0 false confirmation，但 S08B final TTFT=18.1 s。

## 5. 当前 dormant/reactivation

状态为 tentative、confirmed-active、occluded、dormant、deleting，没有显式 `pre_reactivated`。
进入 dormant 时：

- 锚定最近至少连续两帧 reportable 的可靠 posterior position；
- 清零 velocity、CV/CA stale acceleration 和 cross covariance；
- 保存10 s，不普通发布。

Dormant 第一条 packet 的 gate 已包含：F 或 compatible-U provenance、background surface distance≥0.75 m、
Mahalanobis `D²≤16.266`、15 m/s + 6 m/s² reachable bound 和一对一 assignment。命中后当前实现直接
把内部 state 写为 `confirmed_active`，恢复旧 ID 和 existence floor；但 `last_evidence_type` 被标为
`track_reactivation`，reportability 显式阻止当帧发布。下一条 ordinary packet 才清除该标记并可能
发布。也就是说，两阶段行为已存在，但缺少显式中间 state、确认时限和专用指标。

## 6. 当前 CAL/test split

正式 CAL：

- CAL01–05：V2 map epoch、birth groups、`p_ret(range)`、survival；
- CAL06/07：certified epochs/duration candidates；
- CAL08：surface band；
- CAL09：survival/occlusion-dormant geometry。

接受到 canonical 的 V3 值为3 epochs、0.4 s、1 valid epoch、0.75 m、U gate16.266、survival0.1/s。
`soft_vofod_v3_calibration.yaml` 只保留候选说明，不在 test replay 时叠加。

正式 test/negative：S01–S08C、IT11、NEG01–06。R1 报告还使用 NEG/S08A 做工程门禁，因此其
8-case macro 不是纯 CAL-only 参数估计；该结果可做 V3 regression reference，不能作为 V4 新参数
选择集。`v3_calibration_*`、`v3_r*_pre_*`、invalid/validation 目录是开发证据，不进入正式均值。

V4 所要求 CAL10–16、OCC、CS 和 IT20–26 当前均不存在。

## 7. 当前 artifacts 与实验边界

| Phase | ok runs | 主要冻结 commit | 原始 bag |
|---|---:|---|---|
| V3 R1 | 5 | `4d18ea5` | 已删除 |
| V3 R2 final | 3 | `7e690b3` | 已删除 |
| V3 R3 | 40 | `d66bc98` | 已删除 |
| V3 R4 | 40 | `ac51069` | 已删除 |
| V3 R5 | 40 | `7e690b3` | 已删除 |

R4/R5 的每个 paired source group 具有相同 immutable source hash；正式 metrics/manifest/CSV/log
保留。相同 Gazebo seed 重录并非逐 bit 相同，后续因果比较必须在同一新录 source 内 paired replay。

V3 核心结果：R1 false-free0.10085；R2 S08C false confirm0、S08B TTFT18.1 s；R3 combo
HOTA0.95486；R4 combo FN173.1；R5 C0→C1 HOTA−0.36492、FN+74.1、FP−64.4。它们是 V4
regression reference，不可与新 commit 重录 source 做逐 seed直接差值。

## 8. 真实硬件接口状态

工作区已有接口链：

```text
Livox spherical payload
 -> points_raw + rays_raw + ScanIdentity
 -> mid360_ray_preprocessor
 -> points_world + rays_checked
```

硬件 bridge 会发布 spherical diagnostics；capability probe 支持 `hw_spherical_exact` 和
`calibrated_fallback`。当前 raw spherical payload 仍标记为
`hw_spherical_candidate_unverified`，物理 Mid360 exact zero-depth direction 尚未完成两次断电周期
验证；硬件 pipeline 默认 fallback。

仓库没有 HW-CAL01–05、真实 target truth 或任何 real Mid360 bag。因此不能在本阶段诚实报告真实
`p_return(range,azimuth,elevation)`、真实 LOS covariance、真实 opportunity reliability 或真实飞行
结果。可先实现 recorder/protocol，但数据采集需要外部硬件、场地、truth 与安全授权。

## 9. 修改前验证

- `catkin build --no-status -j2`：11/11 packages 成功，无 warning/failure；
- `catkin run_tests soft_vofod_mid360 soft_vofod_evaluation --no-status -j1`：两 package 成功；
- refreshed results：soft core 66/66，pipeline1/1，evaluator/runner16/16；
- 全工作区最近结果：443 tests，0 errors，0 failures，0 skipped。

## 10. V4 最小实现决策

Ponytail 约束下不新增概率框架或第二套 tracker：

1. 复用现有 `RayAngularIndex` 的 direction cells，把每 cell 合并成一次 effective opportunity；
2. 用 `P_D=1-exp(-lambda*n_eff)` 替换 raw-ray product，保留原 Bernoulli miss；
3. 在同一 core 中给 U chains 增加最小 sequential static-vs-CV log likelihood；
4. 把已有隐式两阶段 reactivation 显式化并加 timeout/指标，不重写 dormant；
5. LOS covariance 只做固定/分箱的单因素候选，不引入 JPDA/MHT；
6. 每个 phase 失败即按预设 gate 停止其下游扩矩阵，不为论文叙事保留净负模块。

审计完成前 canonical 未修改。下一步先实现 effective-cell observation model 和最小单元测试，再用
独立 CAL10–14 选择参数；不得在 S05/S08B 等 test scene 上调参。
