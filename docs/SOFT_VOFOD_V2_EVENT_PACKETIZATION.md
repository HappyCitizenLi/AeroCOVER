# SOFT-VoFOD V2 事件 packetization

## 1. 为什么不再用 endpoint 作为 target observation

V1 把每个 free-space violation endpoint 直接当 event，并让 event 创建 0.75 m/2 s support。
S07 单个 run 因而产生 17,837 events、估计 peak support 7,761 和 p95 1001.64 ms。ray 是物理
采样，不等于独立目标观测；把二者等同会同时放大 birth、map feedback 和计算量。

V2 仍保留 raw anomaly endpoint 作为可诊断的物理证据，但只有短时间空间 packet 能进入关联和
birth。raw event 自身不拥有 map support。

## 2. violation 条件

合法 return endpoint 必须同时满足：

\[
state(x)=confident\_free,\quad p_F(x)\ge0.7,\quad
d(x,M^B)\ge0.75\text{ m}.
\]

分类使用写入本 batch 前的地图快照，位于 `SoftVofodCore::processBatch()`
（`src/soft_vofod_mid360/src/core.cpp:2090–2130`）。

## 3. packet 构造

`BackgroundMap::packetizeViolationComponent()`（`core.cpp:545`）先按 0.03 s 时间窗分组，再在
每个窗内对距离不超过 0.75 m 的 samples 做 union-find connected components。packet 位置取逐轴
median，时间取最后 stamp，方向/置信度/anomaly score 聚合，原始 ray indices 保留。

measurement covariance 为：

\[
R_{packet}=S_{sample}+
\left(\sigma^2_{sensor}+\sigma^2_{shape}+\sigma^2_{floor}\right)I,
\]

其中 canonical 为 0.1 m²、0.35 m、0.04 m²。这个协方差随后同时用于 Mahalanobis association
和 trajectory birth inlier gate。

singleton packet 是合法 packet；实现没有 `point_count > 1` 门槛。两个目标若距离超过 0.75 m
不会被合并，相关回归测试位于 `src/soft_vofod_mid360/test/core_test.cpp` 的 `Packetizer` suite。

## 4. packet 用途

- free-violation packet：可关联、可进入 birth；
- unresolved packet：只能维护已有轨迹，不能直接 birth；
- track-explained packet：维护现有轨迹，不反向写成 background；
- 未关联 violation packet 才进入 2 s/256 packet birth buffer。

因此“原始异常很多”和“产生很多 target hypotheses”不再是同一件事。

## 5. 消融结果

100-run 均值：

| 算法 | raw endpoints/run | packets/run | births/run | unique confirmed/run |
|---|---:|---:|---:|---:|
| B1 | 1796.27 | 344.68 | 10.21 | 6.76 |
| B2 | 1796.27 | 344.68 | 7.13 | 5.92 |
| B3 | 1796.27 | 344.68 | 6.06 | 4.57 |
| B4 | 1824.71 | 15.76 | 1.96 | 1.99 |

B1–B3 packet 数相同符合消融定义；B4 的 confirmed feedback 使后续 endpoint 被解释/保护，形成
闭环抑制。B4 最大仅 60 packets/run，support/track peak 最大 7。

S07/N0/seed1001 的直接历史链为：

| 版本 | raw/event | packet | support peak | FP | p95 |
|---|---:|---:|---:|---:|---:|
| old A3 | 17,837 events | 不适用 | 7,761（后验估计） | 15,434 | 1001.64 ms |
| no-event-support | 16,402 events | 不适用 | 234 | 10,790 | 260.75 ms |
| V2 B4 | 830 raw | 6 | 1（直接测量） | 0 | 43.64 ms |

这证明 event/support/runtime storm 已消失；它不等于所有 false events 都消失。S08A 无目标场景
仍有 12.53 false packets/min，但 survival 阻止了 birth 和 confirmed track。
