# SOFT-VoFOD V3 / V2.1 当前实现、实验结果与分析

日期：2026-08-24

实施前审计：`V3_PRE_IMPLEMENTATION_AUDIT.md`

production core 冻结提交：`d6b923d`

最终 evaluator/runner 合同提交：`7e690b3`

## 1. 结论先行

本轮在同一 `soft_vofod_mid360` production path 完成了四项窄修正：两级 free-space 语义、unknown
独立运动 birth、track-conditioned packet/CV-CA IMM、occluded/dormant/reportability。没有新增平行
package，没有恢复 raw endpoint birth/map support，也没有按 scene 切参数。

R1 certified-free、R2 三联 epistemic gate、R3 S04 相对 V3 base 均通过；R4 dormant 显著改善 S05
且不恢复 ghost，但 FN 仍比 B0 高 17.2%；R5 首次严格证明 opportunity 独立降低 FP/fragmentation/
stale，却在 10/10 pairs 中降低 HOTA并增加 FN。由于 R4/R5 未满足“全部前置问题明显改善”的门禁，
本轮没有运行 R6 的 B0+V3 200-run regression，也没有扩 R7 10 seeds。V3 仍是研究原型，不是部署或
真实多机飞行候选。

## 2. 实施内容

### 2.1 Certified Free

地图现在区分 `OBSERVED_FREE` 与 `CERTIFIED_FREE`。只有跨至少 3 个独立 5 Hz epochs、持续至少
0.4 s、含至少一个 VALID_RETURN-backed epoch、且不在 0.75 m background surface band 的 voxel
才可 certified。target violation 只使用 certified 层；同 epoch ray 数只形成饱和 evidence，不增加
独立 epoch 次数。

### 2.2 Unknown ambiguity

birth provenance 固定为 F violation、U independent motion、track reactivation。Unknown packet pair
使用 3D covariance-normalized displacement `D²>16.266`（99.9% χ²）而不是简单 speed threshold。
Stationary unknown 保持 unresolved；certified-free hover 和已 confirmed target 的低速 maintenance
不受该 birth gate 限制。

### 2.3 Dense maintenance

Birth 继续使用 global short-time packet；maintenance 在 predicted gates 内做点所有权唯一的
track-conditioned extraction。CV/CA IMM 实现 mixing、两模式 prediction/update likelihood、mode
probability normalization 和 moment matching。R3 证明两者存在强协同，但显式 multi-target packet
merge 很少，不应把收益简写成“把大 packet 拆开”。

### 2.4 Lifecycle

轨迹状态扩为 tentative/active/occluded/dormant/deleting，existence 与 reportability 分离。Dormant
不普通发布，保存有限 10 s；reactivation 同时要求 F/compatible-U provenance、surface rejection、
Mahalanobis 和物理 reachable gate。进入 dormant 时清除 stale CA acceleration/velocity，锚定至少连续
两帧可报告的可靠 posterior；reactivation 帧不立即发布，需 ordinary packet 再确认。

### 2.5 Evaluator 和协议

新增 certified、epistemic、packet continuity、IMM/split、lifecycle 和 module runtime 指标；进一步
显式输出 target-induced violation count、map contamination、target-path background recovery latency。
IT10/12/13/14/15 复用已有等价语义场景，避免复制 YAML；IT11 保留独立 certified-hover 场景。

### 2.6 Production 数据流

当前只有一条生产路径：

```text
points_world + rays_checked
  -> 10 ms micro-batch / 5 Hz deferred map epoch
  -> OBSERVED_FREE / CERTIFIED_FREE / CANDIDATE_BACKGROUND / STABLE_BACKGROUND
  -> F violation packets 或 U independent-motion packets
  -> 5-group birth
  -> track-conditioned maintenance packets
  -> CV/CA IMM + Hungarian association
  -> existence / reportability / active-occluded-dormant lifecycle
  -> reportable tracks
  -> confirmed-track-limited deferred map protection
```

没有 raw endpoint birth、raw-event map support、scene-specific 参数切换、旧 tracker 输入模式、
`nadir_seed` 或 standalone launch。Birth 与 maintenance 故意解耦：前者需要跨时间独立证据，后者可在
已有轨迹预测门内使用普通 return packet，因此 stationary hover 不会因 U-birth motion gate 被删除。

### 2.7 代码、配置和测试落点

| 位置 | 当前职责 |
|---|---|
| `src/soft_vofod_mid360/include/soft_vofod_mid360/core.h` | V3 状态、配置、track/IMM/lifecycle 数据契约 |
| `src/soft_vofod_mid360/src/core.cpp` | map、packet、birth、IMM、association、opportunity、dormant 的唯一算法实现 |
| `src/soft_vofod_mid360/src/soft_vofod_node.cpp` | ROS 参数、同步输入、消息发布和诊断输出；不含第二套算法 |
| `src/soft_vofod_mid360/config/soft_vofod_v3_canonical.yaml` | 唯一正式 V3 参数集 |
| `src/soft_vofod_mid360/msg/SoftTrack.msg` | provenance、IMM probability、reportability、reactivation 输出 |
| `src/soft_vofod_evaluation/scripts/evaluate_bag.py` | tracking/map/epistemic/lifecycle/runtime 指标 |
| `src/soft_vofod_evaluation/scripts/run_benchmark.py` | record-once/replay-many、variant 和 hash/coverage 合同 |
| `src/soft_vofod_mid360/test/core_test.cpp` | 66 个 core contracts 的主要来源 |

正式配置不按场景变化。关键冻结值为：5 Hz map epoch；certified-free 至少3 epochs、0.4 s、1个
VALID_RETURN epoch、0.75 m surface band；U motion gate `D²=16.266`；birth 5 groups；CV→CA/CA→CV
转移概率0.05/0.1；dormant timeout 10 s；reactivation gate `D²=16.266`；reportability threshold0.2；
target-free warm-up 10 s。

## 3. 关键实验结果

### 3.1 R1：Certified Free

CAL06–08、NEG03/04/05/06 和 S08A 的 8-case macro：

| false-free | static recall | certified precision | certified recall | expansion recall | false packets/min | worst p95 |
|---:|---:|---:|---:|---:|---:|---:|
| 0.10085 | 0.14516 | 0.97166 | 0.22129 | 0.80107 | 0.55179 | 97.33 ms |

`false-free<=0.18` 和 `static recall>=0.12` 两个工程门限均通过。S08A 单独的 false-free/static recall
为0.15092/0.06355，说明总体通过不代表每个 exploration geometry 都均衡；这是当前 map recall 的
主要残余风险。

### 3.2 R2：Epistemic 三联测试

| case | 语义结果 | HOTA | FN | TTFT | p95 |
|---|---|---:|---:|---:|---:|
| S08B moving unknown | birth recall=1 | 0.46018 | 268 | 18.1 s | 97.39 ms |
| S08C stationary unknown | false confirmation=0 | 0 | 340 | N/A | 96.10 ms |
| IT11 mapped-free hover | hover birth recall=1 | 0.98326 | 8 | 0.801 s | 83.56 ms |

三种输入分别证明“U-stationary 不误报”“U-moving 可 birth”“F-hover 可 birth”，没有用同一个
speed threshold 混淆三者。三份 source 的 target-free input 为10.328/10.495/10.489 s，严格超过
10 s warm-up。S08B 的18.1 s TTFT 和不同 source realization 下的 HOTA 波动说明语义正确但召回不稳。

### 3.3 R3：S04 track-conditioned packet / IMM

N0/N1×5，同 source paired 共40 runs：

| variant | HOTA | FP | FN | frag | IDSW | mean p95 |
|---|---:|---:|---:|---:|---:|---:|
| base | 0.87840 | 101.3 | 113.1 | 15.5 | 1.0 | 85.31 ms |
| split | 0.88133 | 86.0 | 111.2 | 13.5 | 0.3 | 84.98 ms |
| IMM | 0.79079 | 104.4 | 67.9 | 10.6 | 9.0 | 85.01 ms |
| split+IMM | 0.95486 | 29.4 | 51.6 | 8.3 | 0.1 | 84.63 ms |

组合相对 base：HOTA +0.07646、FP −71.9、FN −61.5、frag −7.2，四项在10/10 pairs 中方向一致，
IDSW −0.9；worst p95=86.96 ms。旧 B0 的 HOTA/FP/FN/frag 为0.98711/12.2/11.1/0.1，因此 V3
组合解决了当前 B4 退化，却没有全面达到 B0。

### 3.4 R4：S05 IMM / Dormant

N0/N1×5，同 source paired 共40 runs：

| variant | HOTA | FP | FN | frag | IDSW | track-gap reacq | reactivation/run | worst stale | worst p95 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| base | 0.28394 | 6.9 | 200.7 | 2.1 | 2.1 | 9.064 s | 0 | 0.321 s | 85.25 ms |
| IMM | 0.28421 | 6.8 | 200.6 | 2.1 | 2.1 | 9.053 s | 0 | 0.321 s | 87.53 ms |
| dormant | 0.58703 | 10.0 | 173.5 | 3.0 | 0 | 5.547 s | 3.3 | 0.395 s | 86.35 ms |
| IMM+dormant | 0.58867 | 9.6 | 173.1 | 3.0 | 0 | 5.534 s | 3.3 | 0.395 s | 86.01 ms |

组合相对 base：HOTA +0.30474、FN −27.6、IDSW −2.1，HOTA/FN 在10/10 pairs 改善；所有
reactivation correct rate=1、wrong rate=0。主要效应来自 dormant，而非 IMM。相对旧 B0 的 FN147.7，
最终 FN 仍高25.4（17.2%），所以 R4 只能判“显著改善且 ghost-safe”，不能判“超过 B0”。

### 3.5 R5：Opportunity 严格单因素

四个 variants 的 birth groups 都固定为5；N0/N1×5，共40 runs：

| variant | HOTA | FP | FN | frag | IDSW | worst stale | >3 s ghost/run | reactivation/run |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| C0 no opportunity | 0.64895 | 104.7 | 112.2 | 9.5 | 0 | 3.396 s | 0.4 | 0 |
| C1 + opportunity | 0.28403 | 40.3 | 186.3 | 5.1 | 2.4 | 1.921 s | 0 | 0 |
| C2 + survival/reportability | 0.28406 | 6.1 | 203.1 | 2.0 | 2.0 | 0.321 s | 0 | 0 |
| C3 + dormant | 0.58622 | 9.9 | 173.8 | 3.0 | 0 | 0.395 s | 0 | 3.2 |

| transition | ΔHOTA | ΔFP | ΔFN | Δfrag | ΔIDSW | paired 一致性 |
|---|---:|---:|---:|---:|---:|---|
| C0→C1 | −0.36492 | −64.4 | +74.1 | −4.4 | +2.4 | HOTA/FP/FN/frag 10/10 |
| C1→C2 | +0.00003 | −34.2 | +16.8 | −3.1 | −0.4 | FP 10/10 降，FN 9/10 升 |
| C2→C3 | +0.30216 | +3.8 | −29.3 | +1.0 | −2.0 | HOTA/FN 10/10 改善 |

Opportunity 独立降低 FP、fragmentation 和 long-stale ghost，却显著增加 FN、降低 HOTA；这是一项
明确 trade-off，不是净正收益。Survival/reportability 继续清理输出但不能恢复 recall；dormant 能恢复
部分 FN、old ID 和 HOTA。C3 worst p95=86.76 ms。

### 3.6 Map protection 重新解释

旧 B3/B4 的100个 paired runs 显示 target-induced violation packets/run 从341.46降至12.37，
而 contamination ratio 仅从0.006301降至0.006267，target-path trail 从0.970 s降至0.888 s。因此
map protection 的可信收益是 violation suppression，不是已清除 map contamination。

### 3.7 证据有效性

R3/R4/R5 的正式矩阵分别40 runs，全部 manifest/coverage/source-sharing 合同通过；R4/R5 大 bag
均已删除。R2 final 3 runs 也全部通过并删 bag。R1/R3 使用较早冻结 implementation/evaluator hash，
后续 lifecycle/evaluator-only 修改不改变其受测模块；报告不把它们伪装成同一 commit 的单矩阵。
所有最终 phase worst p95<100 ms，总体 worst=97.39 ms。

## 4. 根因与修正链

### 4.1 False-free

根因是 V2 单一 free state 把一次/同视角 traversal 当 detector-grade free，并缺少 surface band。
独立 epoch certification 和 surface guard 将 detector 使用层的 false-free 压到工程目标内，同时保留
static recall；不是靠全局降低 free recall。

### 4.2 S08C

根因不是 packet storm 或 association，而是把 worldline consistency 当 independent motion：零速 CV
轨迹同样有低 residual。F/U provenance 和 U motion χ² gate 修复了语义，同时保留 F-hover 与 moving-U。

### 4.3 S04

packet shortage 约 0.5，但 multi-truth packet 为 0、显式 split 极少；主要损失来自 maneuver prediction
和 global measurement ownership/shape。IMM 单独虽降 FN，却制造 IDSW；conditioned ownership 与 IMM
组合才同时压低 gate miss、FP/FN/frag 和 IDSW。

### 4.4 S05

V2 已会删除 ghost，但删除后只能重新等 5-group birth。Dormant old-ID memory 直接降低 FN/IDSW 和
重捕获延迟。开发中发现并修复了四个独立退化源：wall packet maintenance lock、far unknown 静止
误激活、stale CA acceleration/错误 anchor、reactivation 当帧直接发布。最终 wrong reactivation=0。
S05 正式场景本身没有触发 `occluded` transition，主要从 low-existence fallback 进入 dormant；CAL09
才是在线 ray-occlusion 路径的直接验证。

## 5. 对 15 个最终问题的逐条回答

1. **false-free 为什么高，修完是否下降？** V2 缺独立 epoch、surface guard 和 detector-grade free
   分层；R1 macro 降到 0.10085，低于 0.18 工程门限和旧 B0 约0.1744参考，但不是同 source 显著性检验。
2. **Certified-free precision/recall？** 8-case macro 为 0.97166/0.22129；S08A 为
   0.96177/0.20317。
3. **Static-background recall 是否保留？** macro 0.14516，高于 0.12 门限；S08A 单独仅0.06355，
   moving/exploration case 仍不均衡。
4. **S08C 是否仍 false confirm？** final regression 为 0；未把它强制判为 background，而是 unresolved。
5. **Mapped-free hover 是否可检测？** IT11 recall=1，HOTA0.98326，TTFT0.801 s。
6. **Unknown moving target 是否可 birth？** S08B recall=1；但 TTFT18.1 s、HOTA0.460，稳健召回仍弱。
7. **S04 根因是 merge、CV 还是两者？** 不是大量 merge；主要是 maneuver prediction 与 conditioned
   ownership/measurement 的协同。显式 split 只3次，multi-truth packet为0。
8. **Track-conditioned split 有效？** 单独小幅有效（FP101.3→86、frag15.5→13.5）；与 IMM 联合
   才产生大收益。不能声称“split 次数多”。
9. **IMM 是否真正降低 gate rejection/fragmentation？** combo 将 truth gate rejection 0.6/run→0、
   frag15.5→8.3；IMM 单独 frag10.6但 IDSW9、HOTA下降，故只能在 conditioned ownership 下采用。
10. **Dormant 是否降低 S05 FN？** R4 base200.7→combo173.1，10/10 pairs下降；仍未达到 B0 147.7。
11. **Opportunity 独立贡献是否严格消融？** 是。固定5 groups 后，C0→C1 FP/frag/stale下降，但
    HOTA 10/10下降、FN 10/10上升；独立净收益不成立。
12. **Ghost 是否重新出现？** Final C3 无 >3 s ghost，worst reported stale0.395 s；C0 的确恢复了
    0.4 >3 s ghost track/run，证明 output controls 有实际作用。
13. **Runtime 是否有界？** 是，所有最终 phase worst p95<100 ms；总体 worst97.39 ms。
14. **Map protection 真实收益？** B3→B4 target-induced violation 341.46→12.37/run；contamination
    0.006301→0.006267，几乎不变。收益是 violation suppression，不是 contamination。
15. **是否值得进入真实 Mid360 多目标飞行？** 不值得。可开始 record-only 实机标定/离线 replay；
    在 opportunity recall trade-off、S05 FN、S08B latency/稳定性和 R6 全场景 regression 解决前，不进入闭环飞行。

## 6. 为什么不运行 R6/R7

提示词明确要求只有 R1–R5 通过才跑 R6。当前 R4 仍未把 FN 降到接近/优于 B0，R5 又证明
opportunity 独立净收益为负。因此继续花费 200+ runs 只会把已知失败扩成更大表格，不会闭合因果。
R7 10 seeds 的前置条件同样不成立。这是预先定义 gate 的执行结果，不是算力不足或选择性停止。

## 7. 数据与复现说明

仿真 source 在相同 seed 下并非逐 bit 确定；R4 为统一 commit 重录少量 sources 时观测到数值变化。
因此所有因果只使用同一 immutable source 内的 paired variants，不比较两次独立重录的逐 seed 数字。
R4/R5 final manifests 各自拥有单 commit、单 algorithm/config/evaluator hash 与同 source hash 合同。

为控制工作盘，正式大 bag 在 metrics/manifest/CSV/log 验证后删除，不可从仓库恢复；可用冻结 scene、
seed、noise、commit 和 runner 重录。保留的 invalid/pre-fix 目录用于解释修正链，不进入正式均值。

### 7.1 证据索引

| Phase | 正式目录 | 保留内容 |
|---|---|---|
| R1 | `artifacts/v3_r1/` | certified-free calibration/test metrics |
| R2 | `artifacts/v3_r2_final/` | S08B/S08C/IT11 metrics、summary、manifest |
| R3 | `artifacts/v3_r3/` | 四 variant 40-run aggregate 和 paired deltas |
| R4 | `artifacts/v3_r4/` | 四 variant 40-run lifecycle aggregate 和 paired deltas |
| R5 | `artifacts/v3_r5/` | C0–C3 40-run strict ablation aggregate 和 paired deltas |

各目录的正式入口是 `metrics/summary.csv`、`metrics/aggregate.json`、
`metrics/ablation_deltas.csv` 和 `benchmark_summary.json`。当前 `artifacts/v3*` 下不再保留 `.bag`；旧
V1/V2 bag 未因本轮清理而删除。

### 7.2 最终验证

- `catkin build --no-status -j2`：11/11 packages 成功；
- `catkin run_tests --no-status -j1` + `catkin_test_results build`：443 tests，0 errors，
  0 failures，0 skipped；
- `soft_vofod_core_test`：66/66；
- evaluator/runner/scenario Python contract tests：21/21。

上述全量测试在最终 evaluator/runner 合同提交 `7e690b3` 上执行。随后只修改本报告和实验文档，
没有再改源码、配置或测试。

## 8. 下一步

只建议两项：

1. record-only 采集 HW-CAL01–05，重新估计 range×angle `p_ret`、grazing/no-return free reliability、
   point/pose noise 和 sparse packet 分布；
2. 在 CAL split 上重新设计 opportunity miss likelihood，使其保留 C1 的 ghost/frag 优势而不产生
   +74.1 FN，再重新从 R5 gate 开始；通过后才运行 R6。

不要增加更多 motion models、扩大 dormant gate、恢复 raw endpoint birth，或直接做真实多机闭环。
