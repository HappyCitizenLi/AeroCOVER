# SOFT-VoFOD V2 Phase 12 根因实验

日期：2026-08-21  
范围：N0 / seed1001 / record-once-replay-many / 1.0× replay。除特别说明外，所有 run
均满足首输入握手与 source/track/timing 95% coverage gate；本报告是开发根因证据，不是多
seed 论文结论。

## 1. 实验版本

- B1：deferred map + packet violation + two-packet simple birth；无 opportunity、survival、
  feedback；Hungarian 固定开启。
- B2：B1 + three-independent-group trajectory-before-confirmation。
- B3：B2 + scan opportunity + survival。
- B4：B3 + confirmed-track-only map feedback。
- 旧 A1/A2/A3 只保留为开发历史，不再作为最终消融。

Phase 10/11 后所有算法 run 均可在 1.0× 回放完成。S07 不再需要旧实验的 0.1×，S06
不再需要 0.5×。

## 2. Negative controls

NEG03/NEG04 均使用 120 s 场景、109 s scored interval，无 target。为控制磁盘，完成评估后
删除了可重录的 `source.bag` 和 `output.bag`；保留 scenario、manifest、SHA-256、metrics、
diagnostics/track/opportunity time series 与 resource log。

| 指标 | NEG03 moving observer | NEG04 new-area clutter |
|---|---:|---:|
| false packets/min | 7.156 | 0.550 |
| false tentative/min | 2.202 | 0 |
| false confirmed/min | 0.550 | 0 |
| confirmed peak / final | 1 / 0 | 0 / 0 |
| max stale age (s) | 0.570 | 0 |
| candidate→stable p50 (s) | 5.50 | 11.00 |
| candidate→stable p95 (s) | 17.22 | 34.50 |
| background expansion recall | 0.730 | 0.975 |
| static-background recall | 0.044 | 0.229 |
| false-free | 0.221 | 0.043 |
| runtime p95 (ms) | 48.45 | 41.18 |

结论：moving-observer/no-target 的 V1 病态 storm 已消失，track/support/runtime 有界；但
NEG03 的 false-free 和背景召回仍未达到令人满意的平衡，candidate 收敛尾部也过长。
NEG04 的 E3 几何最初未进入 evaluator，修正门框、立柱和三面墙的解析 surface voxel 后，
上述 map 指标才有效；旧的 recall=0.0107 不作算法结论。

## 3. Packetization 与剩余 false birth 根因

对 B4/A3 输出按同 stamp truth 重新分类。S01–S06 的 false packet 高度集中在：

```text
z ≈ 0
range ≈ 25–32 m
singleton / 2–3 point packet
```

也就是远距 grazing ground，而不是目标附近 duplicate packet。代表性 raw→packet 压缩：

| 场景 | raw anomaly endpoints | packets | 压缩倍数 | false packet z 中位 |
|---|---:|---:|---:|---:|
| S01 | 14,126 | 314 | 45.0× | 6.2e-8 m |
| S02 | 12,538 | 252 | 49.8× | 6.1e-8 m |
| S03 | 31,231 | 461 | 67.7× | 6.1e-8 m |
| S04 | 19,140 | 362 | 52.9× | 6.1e-8 m |
| S05 | 16,595 | 343 | 48.4× | 6.1e-8 m |

packetizer 已消除 raw endpoint storm，但地图仍会先把 coarse grazing ground voxel 标成 free；
这些静态 endpoint 随后成为 F packet，hover-compatible CV birth 又会把它们当静止轨迹。这解释
了“packets 已很少但 FP 仍高”。singleton 不能禁用，因此不能用 `point_count > 1` 掩盖问题。

曾测试两种 F→candidate 并行竞争：直接并行，以及不 suppress raw packet 的 competing 标记。
两者均失败并已从生产源码撤回。S01 的后者使 packets 314→363、births 59→76、FP
7437→7579，background voxels 仍为 0；说明当前跨 epoch centroid/timeout/variance 条件无法把
稀疏远距 ground 命中稳定为同一 candidate。相关小型控制 metrics 保存在
`artifacts/v2_phase12_pre_f_candidate/`，没有保留实验行为开关。

## 4. B1→B2：trajectory birth 的真实贡献

| 场景 | 版本 | births | unique confirmed | FP | IDSW | HOTA |
|---|---|---:|---:|---:|---:|---:|
| S01 | B1 | 272 | 305 | 26,021 | 8 | 0.049 |
|  | B2 | 142 | 224 | 24,091 | 19 | 0.041 |
| S03 | B1 | 269 | 316 | 48,264 | 45 | 0.073 |
|  | B2 | 192 | 261 | 44,940 | 46 | 0.074 |
| S04 | B1 | 316 | 330 | 27,176 | 51 | 0.111 |
|  | B2 | 181 | 261 | 26,631 | 38 | 0.118 |

三组 trajectory 将 births 降低 36–48%，confirmed IDs 和 stale population 同时下降；但 FP
只下降 2–7%，S01/S03 IDSW 未稳定改善。结论是 trajectory birth 有效但不是剩余 FP 的主因；
地面 F packet 输入不解决，任何合法 hover birth 都会继续产生静态伪轨迹。

## 5. B2→B3：opportunity + survival

| 场景 | 版本 | TP / FP / FN | fragmentation | stale>3 s tracks | max stale (s) | HOTA |
|---|---|---:|---:|---:|---:|---:|
| S05 | B2 | 140 / 27,272 / 130 | 9 | 78 | 5.922 | 0.053 |
|  | B3 | 79 / 20,871 / 191 | 3 | 1 | 3.712 | 0.031 |
| S07 | B2 | 185 / 447 / 5 | 0 | 1 | 3.390 | 0.313 |
|  | B3 | 185 / 229 / 5 | 0 | 0 | 0.591 | 0.400 |

opportunity/survival 明确减少 ghost age 和 S05 fragmentation，满足
`fragmentation(B3) <= fragmentation(B2)`；但 S05 TP 大幅下降，说明当前常数
`p_ret=0.5`/`P_D` 未在 return opportunity 上标定，high-opportunity miss 惩罚过强。

评估器原先把 10 Hz 中间帧也计作 packet-detector miss，Brier 被系统性污染。现已只在
`map_epochs_committed > 0` 的 5 Hz detector epochs 计算 opportunity calibration；生存率源码
本身从 Phase 9 起也只在这些 epoch 做 hit/miss，逐 scan survival 保持不变。

## 6. B3→B4：confirmed feedback

| 场景 | 版本 | packets | births | FP | max stale (s) | contamination | HOTA |
|---|---|---:|---:|---:|---:|---:|---:|
| S05 | B3 | 4,981 | 283 | 20,871 | 3.712 | 0.0308 | 0.031 |
|  | B4 | 343 | 61 | 12,061 | 1.120 | 0.0308 | 0.042 |
| S07 | B3 | 231 | 7 | 229 | 0.591 | 0 | 0.400 |
|  | B4 | 3 | 1 | 0 | 0.390 | 0 | 0.987 |

confirmed feedback 对 event/birth/ghost 正反馈非常关键，但本组没有证明 contamination 降低：
S05 相同，S07 均为 0。Phase 11 的 confirmed-only 语义必须保留；不能恢复 tentative 强保护
来换取表面 map 指标。

## 7. 当前门槛判断

- 已闭环：S07 event/support/runtime 病态退化；1× replay；opportunity angular complexity；
  NEG03/NEG04 bounded tracks/supports/runtime；trajectory birth 显著减少 birth 数。
- 部分闭环：opportunity 显著清理 ghost/fragmentation，但 S05 recall 退化，必须 CAL 标定。
- 未闭环：E0/static-observer 远距 grazing ground 的 free/background balance；S01–S05 的
  false packets/births/FP 仍高；S03 IDSW 没有稳定优于 B1。
- 因此暂不满足 Phase 14 全矩阵门槛。下一步只在独立 CAL splits 标定 map candidate 几何、
  `p_ret(range)`、survival/timeout；禁止在 S01–S08 上逐场景调参。

## 8. 证据位置

- 当前 run：`artifacts/runs/{B1,B2,B3,B4}/<scene>/N0/seed_1001/`
- NEG metrics：`artifacts/runs/A3/{NEG03,NEG04}/N0/seed_1001/`
- 根因前控制：`artifacts/v2_phase12_pre_f_candidate/`
- 每个 run：`metrics.json`、`diagnostics_timeseries.csv`、`track_timeseries.csv`、
  `opportunity_timeseries.csv`、`run_manifest.json`。
