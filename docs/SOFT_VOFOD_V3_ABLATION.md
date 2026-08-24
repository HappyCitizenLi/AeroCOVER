# SOFT-VoFOD V3 消融结果

日期：2026-08-24。所有均值只包含 `status=ok` 且 coverage valid 的 runs。R3–R5 都按同一
scene/noise/seed source 做 paired 比较；大 bag 在指标和 manifest 验证后删除。

## 1. R1：Certified Free

冻结配置为 3 independent epochs、0.4 s、至少 1 个 VALID_RETURN-backed epoch、0.75 m surface
band。CAL06–08 与 NEG03/04/05/06、S08A 各 N0/seed1001 的 8-case macro：

| false-free | static recall | certified precision | certified recall | expansion recall | false packets/min | worst p95 |
|---:|---:|---:|---:|---:|---:|---:|
| 0.10085 | 0.14516 | 0.97166 | 0.22129 | 0.80107 | 0.55179 | 97.33 ms |

工程门限 `false-free<=0.18`、`static recall>=0.12` 均通过。S08A 单独为 false-free 0.15092、static
recall 0.06355、certified precision 0.96177、recall 0.20317、expansion 0.62771，且无 tentative/
confirmed target。这里的 8-case macro 不是与 B0 同 source 的统计检验；它只用于 R1 工程 gate。

## 2. R2：Unknown epistemic gate

最终代码/evaluator 在新录 immutable source 上的 N0/seed1001：

| case | 主结果 | HOTA | FN | TTFT | p95 |
|---|---|---:|---:|---:|---:|
| S08B moving unknown | birth recall=1 | 0.46018 | 268 | 18.1 s | 97.39 ms |
| S08C stationary unknown | false confirmation=0 | 0 | 340 | N/A | 96.10 ms |
| IT11 certified-free hover | hover birth recall=1 | 0.98326 | 8 | 0.801 s | 83.56 ms |

三联语义 gate 通过，且 target-free input 分别为 10.328/10.495/10.489 s，严格大于 B0 的 10 s
门限。S08B 虽可 birth，但 TTFT 很长且 HOTA 对 source realization 敏感；这不是稳健召回证明。

## 3. R3：S04 packet/IMM

N0/N1×5，40 runs，冻结于同一 evaluator 定义：

| variant | HOTA | FP | FN | frag | IDSW | mean p95 |
|---|---:|---:|---:|---:|---:|---:|
| S04-base | 0.87840 | 101.3 | 113.1 | 15.5 | 1.0 | 85.31 ms |
| split | 0.88133 | 86.0 | 111.2 | 13.5 | 0.3 | 84.98 ms |
| IMM | 0.79079 | 104.4 | 67.9 | 10.6 | 9.0 | 85.01 ms |
| split+IMM | 0.95486 | 29.4 | 51.6 | 8.3 | 0.1 | 84.63 ms |

split+IMM 相对 base 的 10-pair mean delta：HOTA +0.07646（10/10 上升）、FP −71.9、FN −61.5、
frag −7.2（均 10/10 下降）、IDSW −0.9。worst p95=86.96 ms。

根因诊断不能简化成“packet merge”：所有 variants 的 packet-shortage ratio 都是 0.49955，
multi-truth packet count=0；combo 中显式 mixed-component split 总共只有 3 次。split 单独只有小幅收益，
IMM 单独降低 FN/frag 却把 IDSW 提到 9 并降低 HOTA；track-conditioned ownership/recomputed packet 与
IMM 联合后 truth-packet gate rejection 从 base 0.6/run 降为 0，且压住 IMM 的 identity instability。
因此主要根因是 maneuver prediction 与 conditioned measurement ownership 的协同，不是大量 component
merge。相对旧 B0（HOTA0.98711、FP12.2、FN11.1、frag0.1），仍有明显差距。

## 4. R4：S05 IMM/dormant

N0/N1×5，40 runs；40/40 ok，单 commit `ac51069`、单 implementation/config/evaluator hash、10/10
source groups 内四 variants 同 bag：

| variant | HOTA | FP | FN | frag | IDSW | track-gap reacq | lifecycle reactivation/run | event-end latency | worst stale | worst p95 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| base | 0.28394 | 6.9 | 200.7 | 2.1 | 2.1 | 9.064 s | 0 | N/A | 0.321 s | 85.25 ms |
| IMM | 0.28421 | 6.8 | 200.6 | 2.1 | 2.1 | 9.053 s | 0 | N/A | 0.321 s | 87.53 ms |
| dormant | 0.58703 | 10.0 | 173.5 | 3.0 | 0 | 5.547 s | 3.3 | 1.461 s | 0.395 s | 86.35 ms |
| IMM+dormant | 0.58867 | 9.6 | 173.1 | 3.0 | 0 | 5.534 s | 3.3 | 1.461 s | 0.395 s | 86.01 ms |

combo 相对 base：HOTA +0.30474（10/10）、FN −27.6（10/10）、IDSW −2.1；FP +2.7、frag +0.9。
所有 reactivation 的 correct rate=1、wrong rate=0。IMM 单独近似无效，dormant 是主效应。

相对旧 B4（HOTA0.2543、FP101.7、FN193.7、frag2.9、IDSW2.8），最终组合显著改善 HOTA/FP/FN/
IDSW 并保持 frag。相对 B0（HOTA0.33323、FP143.7、FN147.7、frag5.5、IDSW2.7），HOTA/FP/frag/
IDSW 更好，但 FN 仍高 25.4（17.2%），track-set gap 也更长。R4 因此是“显著改善但未完全达到 B0
FN 门标”。S05 中 `occluded` transition 未触发，收益来自 low-existence→dormant fallback；CAL09 独立
验证了 online occlusion→dormant→reactivation 2/2 正确。

## 5. R5：严格 opportunity 单因素

所有 variants 的 birth groups=5。N0/N1×5 共 40 runs；40/40 ok，单 commit `7e690b3`、单
implementation/config/evaluator hash，10/10 source groups 同 bag：

| variant | HOTA | FP | FN | frag | IDSW | worst stale | >3 s ghost tracks/run | reactivation/run | worst p95 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| C0 no opportunity | 0.64895 | 104.7 | 112.2 | 9.5 | 0 | 3.396 s | 0.4 | 0 | 86.81 ms |
| C1 + opportunity | 0.28403 | 40.3 | 186.3 | 5.1 | 2.4 | 1.921 s | 0 | 0 | 85.40 ms |
| C2 + survival/reportability | 0.28406 | 6.1 | 203.1 | 2.0 | 2.0 | 0.321 s | 0 | 0 | 86.61 ms |
| C3 + dormant | 0.58622 | 9.9 | 173.8 | 3.0 | 0 | 0.395 s | 0 | 3.2 | 86.76 ms |

严格 paired mean delta：

| transition | ΔHOTA | ΔFP | ΔFN | Δfrag | ΔIDSW | 方向一致性 |
|---|---:|---:|---:|---:|---:|---|
| C0→C1 | −0.36492 | −64.4 | +74.1 | −4.4 | +2.4 | HOTA/FP/FN/frag 10/10 同方向 |
| C1→C2 | +0.00003 | −34.2 | +16.8 | −3.1 | −0.4 | FP 10/10 降，FN 9/10 升 |
| C2→C3 | +0.30216 | +3.8 | −29.3 | +1.0 | −2.0 | HOTA/FN 10/10 改善 |

结论：opportunity 的独立贡献是抑制 FP/fragmentation/long stale，但以显著 FN/HOTA 代价换取；不能
称为独立净正收益。survival/reportability 主要继续压制 FP/输出 stale，也未恢复 recall。dormant
恢复一部分 FN、old ID 和 HOTA，所有 reactivation correct=1/wrong=0，但 C3 FN 仍高于 B0 17.7%。

## 6. Map protection 的真实作用

使用旧正式 B3(feedback off)/B4(feedback on) 100 个 paired runs 重新按现有字段计算：

| 指标 | B3 | B4 | paired mean delta |
|---|---:|---:|---:|
| target-induced violation packets/run | 341.46 | 12.37 | −329.09 |
| target contamination ratio | 0.006301 | 0.006267 | −0.0000335 |
| target-path trail max/run | 0.970 s | 0.888 s | −0.082 s |

因此已证实的主要收益是 violation suppression；contamination 净变化近似不可辨。V3 evaluator 已
显式输出 target-induced violation count、map contamination 和 target-path background recovery latency，
但尚未为新字段再跑一套 feedback off/on V3 矩阵。

## 7. Phase gate

| phase | 结论 |
|---|---|
| R1 | 通过工程门限 |
| R2 | 三联语义 gate 通过；moving-U recall 稳健性仍有限 |
| R3 | 相对 V3 base 显著通过，尚未接近 B0 全部指标 |
| R4 | 部分通过：ghost-safe 且显著改善，FN 未达 B0 |
| R5 | 因果闭环完成；opportunity 独立净收益不成立 |
| R6/R7 | 按提示词 gate 未运行 |
