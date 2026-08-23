# SOFT-VoFOD V3 实施前审计

日期：2026-08-23  
审计基线：`ab16888cc101e669b46fba3e00eeef3418dc93ee`  
正式 V2 实验冻结提交：`4cc1fe71c20aae6f2110b925733c679f39dfd98e`

本文在修改 production algorithm 前冻结 V2/B4 的真实执行路径、参数、实验结果与当前证据缺口。
用户未跟踪的 `.vscode/`、`1.md` 和 V3 提示词不属于本审计改动。

## 1. 审计范围与证据

已逐行审计：

- `src/soft_vofod_mid360/{include/soft_vofod_mid360/core.h,src/core.cpp,
  src/soft_vofod_node.cpp,config/soft_vofod_v2_canonical.yaml,launch/soft_vofod.launch}`；
- `src/soft_vofod_evaluation/scripts/{run_benchmark.py,evaluate_bag.py}`；
- V1/V2 最终报告与当前实现说明；
- `artifacts/metrics/{summary.csv,aggregate.json,ablation_deltas.csv}`；
- S04/S05/S08C 的 30 个 B4 `metrics.json`、`diagnostics_timeseries.csv`、
  `track_timeseries.csv` 和 manifests。

正式 V2 矩阵为 10 scenes × N0/N1 × seeds 1001–1005 × B0–B4 = 500 runs；全部
`status=ok`、1.0× replay、coverage valid。原始 source/output bags 已按既定磁盘策略删除，因此
当前只能审计已保留逐帧 CSV；需要新增 truth-conditioned 指标时必须重录 source bag。

## 2. B0–B4 真实 launch override

runner 的真实覆盖不是只由 base config hash 表达：

| 版本 | birth groups | opportunity | target feedback | Hungarian | survival λ/s |
|---|---:|---:|---:|---:|---:|
| B0 | component detector | 否 | 否 | classic greedy | B0 自身规则 |
| B1 | 2 | 否 | 否 | 是 | 0 |
| B2 | 3 | 否 | 否 | 是 | 0 |
| B3 | 5 | 是 | 否 | 是 | 0.1 |
| B4 | 5 | 是 | 是 | 是 | 0.1 |

B0 启动 `tclv_evaluation/b0_canonical.launch`；B1–B4 均启动同一个
`soft_vofod_mid360/soft_vofod.launch` 并覆写上表参数。正式 B2→B3 同时改变 birth groups 与
opportunity/survival，是已知混杂；V3 必须统一 birth groups。

## 3. 当前地图状态与状态转移

V2 只有四个 `VoxelState`：

```text
UNKNOWN
CONFIDENT_FREE
CANDIDATE_BACKGROUND
STABLE_BACKGROUND
```

每个 voxel 保存 `free_evidence`、`background_evidence`、候选独立 group 次数/起止时间、最近更新时间
和 quarantine deadline。查询量为：

\[
p_F=E_F/(E_F+E_B),\qquad p_B=E_B/(E_F+E_B),\qquad
c=1-\exp(-(E_F+E_B)/5).
\]

`CONFIDENT_FREE` 的建立条件只有：

```text
confidence >= 0.6
free_probability >= 0.7
```

它没有独立 epoch count、跨时 persistence、view diversity、surface guard 或 VALID_RETURN-backed
比例。它同时被当作一般 free memory 和 target-detection-grade free，是当前 false-free 语义缺口。

`STABLE_BACKGROUND` 要求至少 3 个独立 group、持续至少 1 s、background probability ≥0.7、
confidence ≥0.6 且不在 quarantine；一旦 stable，后续 free ray 不擦除。

## 4. Free epoch aggregation

地图以 5 Hz epoch 更新。每条 VALID_RETURN traversal 权重 1.0，NO_RETURN 权重 0.5；同 epoch
同 voxel 的 ray contribution 先累计为 `raw`，提交时饱和：

\[
e_F=1-\exp(-raw/n_0),\qquad n_0=1.
\]

只有该 epoch 没有 endpoint return 的 voxel 才提交 free evidence。candidate/stable voxel 在 ray
tracing 时直接跳过 free accumulation。这个实现限制了单 epoch ray 数，但没有记录“独立 free
epochs”，所以一个高覆盖 epoch 已足以使 voxel 达到 `CONFIDENT_FREE`。

VALID_RETURN 与 NO_RETURN 只有权重不同，最终写入同一个 evidence/state；算法无法要求 certified
free 至少有 VALID_RETURN-backed traversal。

## 5. Background component assimilation

每个 epoch 的 endpoint voxels 做 26-neighbour connected components，按下列量分类：

- component 内当前 `CONFIDENT_FREE` 比例 `q_free`；
- unknown 比例 `q_unknown`；
- confirmed/weak tentative support 解释比例；
- stable background 邻接和最近 stable distance。

分类为 background-supported、free violation、unknown 或 track-explained。background-supported
component 以权重 5 快速进入 stable background；free violation 和 unknown 先尝试 candidate 路径；
confirmed track endpoints 不写背景，tentative overlap 只算 weak explanation。

## 6. Unknown candidate persistence

unknown component 以 centroid/voxel overlap 在 1 m 内跨 epoch 匹配。候选保存 first/last seen、
epoch count、centroid variance 和 per-voxel hit count。当前门限为 3 epochs、1 s、centroid sigma
≤0.15 m、timeout 1 s；满足门限的重复 voxels 被写为 stable background。

候选本身不是 birth observation。问题在于它附近的 traversal 可能很快建立 `CONFIDENT_FREE`；同一
stationary object 后续 endpoint 随 voxel hopping 落入该 free state 后，会被重新解释成普通
violation。当前没有 `UNRESOLVED_STATIC` 输出语义。

## 7. Packetization 与 birth provenance

epoch component 内先按 0.03 s 时间窗分组，再以 0.75 m 半径做 connected clustering；measurement
为逐轴 median，保留 singleton，并用 sample spread + sensor/shape/floor covariance。

birth buffer 只接收 unmatched violation packets；unresolved 和 track-explained packets只能维护
existing tracks。birth 需要 5 个独立时间组、至少 0.1 s、CV residual ≤0.8 m、3D gate 11.345、
总 anomaly ≥1.5，并受 1 m suppression 和每 1 m/map epoch 最多一次 birth 限制。

当前 `Event`、`BirthCandidate` 和 `Track` 均没有 provenance。所有可 birth packet 都隐式等价于
“return in `CONFIDENT_FREE`”；无法区分：

```text
CERTIFIED_FREE_VIOLATION
UNKNOWN_INDEPENDENT_MOTION
TRACK_REACTIVATION
```

CV trajectory 明确允许零速度。因此 V2 只能检查 worldline consistency，不能检查 unknown 中的
independent motion significance。

## 8. Maintenance、CV-KF 与 association

maintenance pool 是当前 map epoch 的 violation、unresolved 和 track-explained global packets。
每个 global packet 只有一个 centroid/covariance；若 component 混合多个 target，centroid 不会按
predicted track 拆分。Hungarian 在 track×packet 的 Mahalanobis + anomaly cost 上一一匹配。

状态是 6D `[p,v]` CV-KF，white-acceleration process noise σ=3 m/s²；不存在 CA mode、IMM mixing、
mode likelihood 或 moment matching。一个 mixed packet 只能分给一条 track，另一个 target 即使
有 raw points 也可能没有独立 measurement；CV 在同步转向/加减速时还会增大 innovation 并 gate
reject。

## 9. Opportunity、existence/survival 与 support

逐射线 opportunity 保留真实 VALID_RETURN/NO_RETURN，使用 mean + 6 个 covariance sigma points、
前景 return 遮挡、前方 confirmed track 遮挡和 angular ray index。`p_ret` range bins 为
0–10/10–20/20–30/30+ m = 0.916/0.878/0.877/0.5；最后一档是未标定 fallback。

existence 每 scan 先按 `exp(-λΔt)` survival，λ=0.1/s；hit 做 Bernoulli likelihood/clutter update；
仅在 map epoch commit 且 unmatched 时做 opportunity miss update。tentative/confirmed 的无测量
timeout 分别为 0.3/6 s，另有 30 s hard timeout和 0.1 delete threshold。

生命周期只有 `TENTATIVE/CONFIRMED/DELETING`。existence 同时决定“物理仍存在”和“是否应继续
输出”；没有 OCCLUDED、DORMANT、reportability 或 old-ID reacquisition。

只有 confirmed tracks 产生强 map support；tentative 只用于 endpoint component 的 weak explanation。
confirmed support 半径为 target radius 0.75 m + 2σ position uncertainty（cap 1.5 m），并通过 voxel
index 截断 free rays/保护 endpoints。删除 confirmed track 后只保留 2 s bounded quarantine。

## 10. S04 逐帧证据

10 个正式 B4 runs 的均值（范围）：

| 指标 | 均值 | min–max |
|---|---:|---:|
| HOTA | 0.8940 | 0.8008–0.9289 |
| FP / FN / fragmentation | 110 / 89.4 / 13 | FP 60–258，FN 70–119，frag 9–20 |
| IDSW | 0.8 | 0–7 |
| raw anomaly endpoints/run | 4588.1 | 4570–4618 |
| violation packets/run | 31.2 | 22–60 |
| maintenance packets/run | 1072.0 | 1023–1346 |
| matches/run | 439.8 | 420–493 |
| births/run | 4.3 | 4–6 |
| unique confirmed tracks | 4.8 | 4–7 |
| max stale | 0.392 s | 0.314–0.539 s |

按 scenario event 分段的 run 均值：

| 阶段 | maintenance packets | matches | births | mean tracks |
|---|---:|---:|---:|---:|
| parallel | 355.5 | 133.8 | 4.2 | 3.93 |
| crossing | 376.3 | 161.4 | 0.1 | 4.11 |
| turn | 340.2 | 144.6 | 0 | 4.10 |

S04 的 IDSW 和 stale 很低、track population 接近四个，但每阶段 measurement→match 转换都只有
约 38–43%，同时 FP/FN/fragmentation 高。这排除了“主要是长 ghost”解释，支持 measurement
continuity 缺口。保留 CSV 不含 packet point ownership、truth-conditioned gate rejection 或 target
acceleration，故当前还不能严格把损失拆成 under-segmentation 与 CV gate miss；V3 evaluator 必须
新增该证据后才能选择 packet split/IMM。

## 11. S05 逐帧证据

10 个正式 B4 runs：HOTA 0.2543、FP 101.7、FN 193.7、fragmentation 2.9、IDSW 2.8；每 run
只有 29.1 violation packets、3.5 births、79.7 matches，但 3.8 个 unique confirmed IDs。max stale
仅 0.532 s，且没有 confirmed-timeout deletion；平均 2.9 次 existence deletion。

在 1/3/5 s occlusion 后 1.5 s 内复用遮挡前 ID 的 run 数分别只有 2/10、2/10、3/10。当前轨迹
通常被 existence 删除，重现后重新等待完整 5-group birth，直接造成 FN、reacquisition latency 和
ID 变化。ghost 已受控；缺失的是 online occlusion state、dormant memory 与受物理约束的 old-ID
reactivation。

## 12. S08C 逐帧证据

10 个正式 B4 runs 全部出现 confirmed target。stationary object 出现后：

- first tentative 延迟均值 0.606 s，范围 0.375–1.058 s；
- false confirmation 延迟均值 0.966 s，范围 0.576–2.400 s；
- tentative→confirmed 平均只约 0.36 s；
- 每 run 约 1852.8 raw anomaly endpoints，但只形成 7.3 violation packets；
- 每 run 约 375.1 maintenance packets、157.1 matches；
- 10/10 都有且只有一个 confirmed ID，max stale 仅 0.313 s。

这不是 packet storm、ghost 或 association 错误，而是 birth semantics 错误：未独立认证的 free
state 给 stationary object 产生了合法 violation provenance，零速 CV trajectory 又能快速确认。
修复必须同时引入 certified free 与 unknown-motion significance，不能全局禁止低速 birth。

## 13. 当前参数来源

| 参数组 | 当前值 | 来源强度 |
|---|---|---|
| input | sync queue 32、geometry tolerance 0.01 m | replay contract / engineering |
| map geometry | center `(10,0,5)`、size `(40,30,16)` m、voxel 0.5 m | V1 仿真工作体积 |
| evidence | scale 5、confidence 0.6、free/background probability 0.7 | V1 engineering defaults |
| background promotion | 3 groups、1 s、weight 1 | V1 engineering defaults |
| map epoch | 5 Hz、free n0=1、epoch weight 1 | CAL01/CAL02；n0=10 rejected |
| component | attach/separate 0.8/1.0 m、free ratio 0.5、track ratio 0.25、supported weight 5 | engineering |
| unknown persistence | 3 epochs、1 s、sigma 0.15 m、match 1 m、timeout 1 s | CAL01/CAL02 + per-voxel fix |
| free evidence | VALID 1、NO_RETURN 0.5、endpoint guard 0.5 m、max NO_RETURN 20 m | engineering / V1 physical defaults |
| violation | free threshold 0.7、background exclusion 0.75 m、search 2 m、distance scale 1 m | engineering |
| packet | 0.03 s、radius 0.75 m、sensor variance 0.1 m²、shape 0.35 m、floor 0.04 m² | engineering；low covariance rejected |
| birth | 5 groups | CAL03，3 groups rejected for final B3/B4 |
| birth gates | buffer 2 s/256、group 0.05 s、duration 0.1 s、pair 0.02–2 s、speed 15 m/s、residual 0.8 m、gate 11.345、score 1.5 | engineering |
| birth suppression | 1 m、cell 1 m、1 birth/cell/epoch | engineering |
| CV-KF | σa=3 m/s²、R=0.1 m²、shape 0.35 m、P0 velocity=9 m²/s²、gate 11.345 | V1 engineering；σa=6 rejected |
| existence | birth/confirm/delete 0.6/0.8/0.1、clutter 0.001 | engineering |
| survival | λ=0.1/s | CAL05 vs 0.05/s |
| lifecycle | tentative age/no-measurement 1/0.3 s、confirmed 6 s、hard 30 s | engineering |
| duplicate merge | d² 1、distance 0.25 m、velocity 0.5 m/s、measurement/birth Δt 0.05/0.5 s | engineering |
| opportunity p_ret | 0.916/0.878/0.877 for <30 m | CAL04 |
| opportunity 30+ | 0.5 | uncalibrated fallback |
| opportunity geometry | cap 0.95、range 60 m、margin 0.15 m、sigma scale 1 | engineering |
| support | target 0.75 m、2σ、uncertainty cap 1.5 m、quarantine 2 s | engineering |
| startup | fixed warm-up 0 | V2 online assimilation design |

只有明确标为 CAL 的参数具有独立 CAL split 证据；其余不能宣称为真实 Mid360 标定值。

## 14. V3 最小实现边界

必须保留 V2 已证明有效的 deferred map、packet birth、singleton、逐射线 opportunity、bounded
support/index 和 record-once/replay-many。V3 只在现有 package/production path 增加：

1. independent-epoch certified free + surface guard；
2. packet/track birth provenance + unknown displacement significance；
3. truth-free track-conditioned maintenance split；
4. CV/CA IMM；
5. occluded/dormant/reportability + bounded reactivation；
6. 对应 evaluator evidence 和严格 C0–C3 overrides。

不新增平行 V3 package，不恢复 raw endpoint birth/support，不实现 CT/JPDA/MHT/PMBM，不按 scene
切参数。R1–R5 门禁未通过前不运行全矩阵或 10 seeds。

## 15. 修改前验证快照

在上述审计完成且尚未修改生产代码时执行：

- `catkin build --no-status`：11/11 packages 成功；
- `catkin run_tests --no-status -j1`：11/11 packages 成功；
- `catkin_test_results build`：386 tests，0 errors，0 failures，0 skipped。

因此，后续 V3 失败可与本节的干净基线直接区分；本审计文件先于任何 V3 生产实现提交。
