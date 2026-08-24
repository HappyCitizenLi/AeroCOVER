# SOFT-VoFOD V4 单因素消融

## 1. Opportunity：O0–O3

只切 observation/miss model。O3 相对 O0 降低 FP `35.9→12.2`、fragmentation `10.6→3.6` 和 stale
ghost proxy `0.823→0.323`，但 FN `137.4→189.4`、HOTA `0.6559→0.4544`，10/10 paired HOTA
退化，故关闭。

## 2. Epistemic：U0/U1

只切旧 unknown birth 与 sequential evidence。S08B HOTA `0.4971→0.9204`、FN `256→52`、TTFT
`18.2→5.2 s`；S08C false confirmation 保持0，IT11 hover recall 保持1。sequential 进入当前候选，但
严格 `<5 s` final gate 仍差0.2 s。

## 3. Reacquisition：R0/R1

只切显式 PRE_REACTIVATED。HOTA/FN/FP/frag 完全不变，wrong reactivation 均0，R1 增加0.20 s
report latency。因此 two-stage 实现保留但关闭。

## 4. Measurement：M0/M1

CAL16 的 114 个 target packets 给出 LOS absolute error mean 0.271 m、perpendicular mean 0.129 m、
signed LOS mean -0.216 m，支持 `sigma_parallel > sigma_perp`。同源 S04 seed1004：

| metric | M0 isotropic | M1 LOS | delta M1-M0 |
|---|---:|---:|---:|
| true packet gate rejection | 0 | 0 | 0 |
| FN | 49 | 47 | -2 |
| fragmentation | 7 | 7 | 0 |
| HOTA | 0.968354 | 0.969502 | +0.001148 |
| runtime p95 | 80.583 ms | 82.665 ms | +2.082 ms |

单 seed 改善过小，不满足“明显改善”，M1 未进入 canonical。

## 5. Cold-start causal correction

这不是预注册 final ablation，而是 failure-driven correction。同一 CS03 source 上，初始实现有4个 false
confirmations；加入 boundary guard 和短期 history quarantine 后为1；把 static-unknown provenance
持久化到 bounded map voxel index 后为0。由于后续 CS01/02/04/05 尚未全部在同一最终 commit 重放，
该结果不能替代正式 single-factor matrix。

## 6. 冻结结论

当前进入候选主线的新增项只有 sequential epistemic inference 与 cold-start provenance correction。
new opportunity miss、two-stage reactivation、LOS anisotropy 均因各自 gate 未过而关闭。
