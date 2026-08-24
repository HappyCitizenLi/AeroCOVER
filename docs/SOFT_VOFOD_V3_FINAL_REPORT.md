# SOFT-VoFOD V3 / V2.1 最终修正报告

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

## 3. 关键实验结果

完整表、paired delta 和 hash 合同见 `SOFT_VOFOD_V3_ABLATION.md`。摘要：

- R1 8-case macro：false-free 0.10085、static recall 0.14516、certified precision/recall
  0.97166/0.22129，worst p95 97.33 ms；
- R2：S08C false confirmation=0，S08B moving birth=1，IT11 hover birth=1，worst p95 97.39 ms；
- R3 split+IMM：HOTA0.95486、FP29.4、FN51.6、frag8.3、IDSW0.1；相对 base 10/10 HOTA/FP/FN/
  frag 改善；
- R4 IMM+dormant：HOTA0.58867、FP9.6、FN173.1、frag3、IDSW0，3.3 correct reactivations/run、
  wrong0，worst reported stale0.395 s、worst p9586.01 ms；
- R5 C3：HOTA0.58622、FP9.9、FN173.8、frag3、IDSW0，无 >3 s ghost，worst p9586.76 ms；
- 所有最终 phase worst p95 <100 ms。

R3/R4/R5 的正式矩阵分别 40 runs，全部 manifest/coverage/source-sharing 合同通过；R4/R5 大 bag
均已删除。R2 final 3 runs 也全部通过并删 bag。R1/R3 使用较早冻结 implementation/evaluator hash，
后续 lifecycle/evaluator-only 修改不改变其受测模块；报告不把它们伪装成同一 commit 的单矩阵。

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

### 7.1 最终验证

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
