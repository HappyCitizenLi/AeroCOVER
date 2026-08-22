# SOFT-VoFOD V2 地图同化

## 1. 证据状态

每个 voxel 维护 free evidence \(F\) 与 background evidence \(B\)：

\[
p_F=\frac{F}{F+B},\qquad p_B=\frac{B}{F+B},\qquad
c=1-\exp\!\left(-\frac{F+B}{s_e}\right).
\]

canonical 使用 \(s_e=5\)、置信门限 0.6、free/background probability 门限 0.7。实现位于
`BackgroundMap::query/updateState`（`src/soft_vofod_mid360/src/core.cpp:186/269`）。

## 2. deferred epoch 与 free saturation

ray traversal 和 return 在 10 ms micro-batch 内只累积；长期 map 以 5 Hz 提交。一个 epoch 中，
同一 voxel 的相关 free rays 用饱和权重：

\[
w_F(n_F)=w_{epoch}\left(1-\exp\left(-\frac{n_F}{n_0}\right)\right),
\]

其中 canonical \(n_0=1\)、\(w_{epoch}=1\)。同 epoch 的 endpoint 优先于 coarse free traversal，
避免 grazing ray 把静态表面抹成 free。实现位于 `BackgroundMap::advanceEpoch()`，饱和写入在
`core.cpp:850–861`。

## 3. component 归属

epoch return voxels 先做 26-connected components。每个 component 计算：

\[
q_F=\frac{N_{free}}{N_{voxel}},\qquad
q_T=\frac{N_{confirmed\ track}}{N_{return}},\qquad
q_U=\frac{N_{unknown}}{N_{voxel}}.
\]

决策顺序为：

1. `q_T >= 0.25`：track-explained，不写 background；
2. 邻接 stable background 或距离小于 0.8 m：background-supported；
3. `q_F >= 0.5` 且离 background 超过 1.0 m：free violation；
4. 其余 unknown/weak-track component：进入 candidate 或 unresolved packet。

这段逻辑在 `BackgroundMap::advanceEpoch()`（`core.cpp:650`）。

## 4. background expansion 与 cold-start

已知背景邻接 component 可直接扩张。未知 component 则按 world-frame centroid/voxel overlap 跨
epoch 关联；至少 3 epochs、1 s 后，只晋升达到重复命中门槛的 voxels。若 component centroid
漂移但部分静态 voxels 重复出现，per-voxel persistence 仍能保留静态部分。入口为
`BackgroundMap::updateUnknownCandidate()`（`core.cpp:423`）。

candidate 超过 1 s 未再次观测会过期。这个机制让 proposed method 的固定 map-only warm-up 为
0 s；online readiness 来自持久性，而不是 truth 或场景事件。

## 5. target feedback

只有 confirmed track 产生强 support：

\[
r_{support}=r_{target}+\min\left(k_\sigma\sqrt{\lambda_{max}(P_{pos})},r_{cap}\right).
\]

canonical 为 target radius 0.75 m、\(k_\sigma=2\)、cap 1.5 m。free ray 在 support 近端前
0.2 m 截断，confirmed endpoint 被 quarantine；tentative 只作为 weak/unresolved 证据，不能强
截断 carving。实现位于 `supports/indexSupports/truncateBeforeSupport`（`core.cpp:1645–1779`）
以及 `processBatch()` 的 map 写入段（`core.cpp:2330–2414`）。

## 6. 正式结果

100-run 地图宏平均：

| 算法 | static-background recall | false-free | expansion recall | contamination* |
|---|---:|---:|---:|---:|
| B0 | 0.0627 | 0.1744 | 0.7532 | 0 |
| B1/B2/B3 | 0.1411 | 0.2249 | 0.7662 | 0.006301 |
| B4 | 0.1390 | 0.2247 | 0.7541 | 0.006267 |

`*` contamination 只在 90 个有 target 的 run 上定义。

结论必须同时保留两面：

- 相对 B0，B4 的 static-background recall 提高 0.0762；
- false-free 反而增加 0.0503，说明 free/background balance 未闭环；
- B3→B4 contamination 只下降 0.000034，90 个配对中 88 个完全相等，不能宣称 map
  protection 已降低 contamination；
- B4 的 expansion 与 B0 基本相同，不是全局提升。

NEG03 moving-observer/no-target 为 recall 0.0439、false-free 0.2207、expansion 0.7303；NEG04
new-area/no-target 为 0.2287/0.0428/0.9754。它们证明状态有界，但 NEG03 仍显示地图质量不足。
