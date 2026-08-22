# SOFT-VoFOD V2 消融结果

## 1. 范围与统计口径

正式数据为 10 scenes × N0/N1 × 5 seeds × B0–B4，共 500 runs；全部来自提交 `4cc1fe7`，
1.0x replay、coverage valid。表中 track 宏平均只使用 9 个有 target 场景，S08A 单独用 false
track/event 指标。`±` 为跨 10 个 noise/seed runs 的 sample SD；总体表给每 run 宏平均。

## 2. 总体 track 指标

| 算法 | HOTA | TP/run | FP/run | FN/run | IDSW/run | frag/run |
|---|---:|---:|---:|---:|---:|---:|
| B0 | 0.7582 | 333.33 | 35.77 | 46.04 | 1.233 | 1.700 |
| B1 | 0.3791 | 339.70 | 650.82 | 39.68 | 21.911 | 2.844 |
| B2 | 0.3944 | 338.12 | 601.84 | 41.26 | 21.178 | 2.956 |
| B3 | 0.4851 | 316.86 | 308.30 | 62.52 | 14.578 | 2.100 |
| B4 | 0.7535 | 311.79 | 39.73 | 67.59 | 0.656 | 2.844 |

B4 与 B0 接近但未全面超过：HOTA -0.0046、FP/run +3.97、FN/run +21.54；IDSW/run
降低 0.578，但 fragmentation/run 增加 1.144。

## 3. 中间量

| 算法 | raw/run | packets/run | births/run | confirmed IDs/run | stale>3 s/run | p95 ms/run |
|---|---:|---:|---:|---:|---:|---:|
| B1 | 1796.27 | 344.68 | 10.21 | 6.76 | 2.18 | 37.67 |
| B2 | 1796.27 | 344.68 | 7.13 | 5.92 | 2.29 | 38.27 |
| B3 | 1796.27 | 344.68 | 6.06 | 4.57 | 0 | 38.31 |
| B4 | 1824.71 | 15.76 | 1.96 | 1.99 | 0 | 41.85 |

B4 p95 最大 50.66 ms；packets 最大 60/run；support/track peak 最大 7。

## 4. 关键场景 B0 vs B4

| 场景 | HOTA B0→B4 | FP B0→B4 | FN B0→B4 | IDSW B0→B4 | frag B0→B4 |
|---|---:|---:|---:|---:|---:|
| S01 | 0.326→0.437 | 85.3→76.7 | 100.5→130.8 | 5.2→1.3 | 5.5→6.9 |
| S02 | 0.565→0.579 | 62.3→9.2 | 42.4→63.1 | 1.0→1.0 | 1.0→1.0 |
| S03 | 0.890→0.989 | 0→3.7 | 17.7→12.6 | 2.2→0 | 2.9→1.8 |
| S04 | 0.987→0.894 | 12.2→110.0 | 11.1→89.4 | 0→0.8 | 0.1→13.0 |
| S05 | 0.333→0.254 | 143.7→101.7 | 147.7→193.7 | 2.7→2.8 | 5.5→2.9 |
| S06 | 0.738→0.679 | 18.4→56.3 | 85.9→89.4 | 0→0 | 0.3→0 |
| S07 | 0.994→0.980 | 0→0 | 2.2→7.4 | 0→0 | 0→0 |
| S08B | 0.995→0.981 | 0→0 | 3.6→12.6 | 0→0 | 0→0 |
| S08C* | 0.995→0.986 | 0→0 | 3.3→9.3 | 0→0 | 0→0 |

`*` S08C 的高 HOTA 不是成功：B4 在 10/10 run 把 unknown stationary object 确认为轨迹，违反
预期分类语义。

## 5. 消融解释

### B1→B2：trajectory birth

B1 two-group/simple birth 变为 B2 three-group trajectory birth。100-run birth/run 10.21→7.13，
FP/run 592.9→546.8，HOTA 0.341→0.355（含 S08A 的总体宏平均）。S03 birth 22.8→13.5；
模块确实减少 duplicate birth，但单独没有解决大量 false tracks。

### B2→B3：opportunity/survival，但正式矩阵有 confound

正式 B3 同时把 CAL-frozen birth groups 3→5，并开启 opportunity/survival，因此其 HOTA +0.0816、
FP -269.34、frag -0.77、stale>3 s -2.29/run 不能全部归因于 opportunity。

严格同为 3 groups 的 Phase 12 N0/seed1001 控制中，S05 B2→B3 fragmentation 9→3、
stale>3 s tracks 78→1；S07 stale>3 s 1→0、FP 447→229。这证明 opportunity/survival 能降低
ghost/fragmentation，但只是单 seed 控制，不是多 seed 单因素结论。

### B3→B4：confirmed feedback

这是正式矩阵中的干净模块差：groups/opportunity/survival 相同，只开启 confirmed map feedback。
100-run paired mean：packets -328.92、births -4.10、FP -241.71、IDSW -12.53、HOTA +0.2416；
但 frag +0.67，runtime p95 +3.53 ms。

map contamination 0.006301→0.006267，90 对中 88 对相等，仅 2 对下降；不能宣称 feedback
降低 contamination。它已证明的是 event/birth/track 正反馈抑制。

## 6. 地图消融

| 算法 | static recall | false-free | expansion | contamination |
|---|---:|---:|---:|---:|
| B0 | 0.0627 | 0.1744 | 0.7532 | 0 |
| B1/B2/B3 | 0.1411 | 0.2249 | 0.7662 | 0.006301 |
| B4 | 0.1390 | 0.2247 | 0.7541 | 0.006267 |

地图 assimilation 提高 static recall，但牺牲 false-free；B4 protection 略降低 recall/expansion，
contamination 差异近零。地图层不是“全面变好”。

## 7. 七个直接结论

1. false-free 是否下降：相对 old S07 A3 明显下降；相对 B0 全局没有，B4 更差 0.0503。
2. static-background recall 是否提高：相对 B0 提高 0.0762，但绝对值仍低，B4 还略低于 B3。
3. event/packet storm 是否消失：是；B4 packet 最大 60/run，p95 最大 50.66 ms。
4. duplicate birth 是否下降：是；B1→B2 birth 10.21→7.13，B4 为 1.96/run。
5. ghost tracks 是否消失：在本仿真矩阵中是；B3/B4 stale>3 s 为 0。
6. opportunity 是否降低 fragmentation：同-group 单 seed 控制为是；正式多 seed B2→B3 被
   groups 变化混杂，不能称多 seed 单因素证明。
7. map protection 是否降低 contamination：没有证据；差异近零。
