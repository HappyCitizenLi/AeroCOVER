# SOFT-VoFOD V2 轨迹、opportunity 与存在概率

## 1. packet-level CV-KF

状态为 \(x=[p_x,p_y,p_z,v_x,v_y,v_z]^T\)。真实 \(\Delta t\) 下：

\[
F=\begin{bmatrix}I&\Delta t I\\0&I\end{bmatrix},\qquad
Q=\sigma_a^2
\begin{bmatrix}
\frac{\Delta t^4}{4}I&\frac{\Delta t^3}{2}I\\
\frac{\Delta t^3}{2}I&\Delta t^2I
\end{bmatrix}.
\]

canonical \(\sigma_a=3\) m/s²。实现为 `transition/processNoise`
（`src/soft_vofod_mid360/src/core.cpp:1055–1080`）。packet covariance 直接进入 innovation covariance，
关联门限 \(d_M^2\le11.345\)。合法 costs 由矩形 Hungarian 做一对一全局最小分配；额外 tracks
用独立 dummy columns 保持 unmatched，不再因方阵 padding 相互争用。

## 2. trajectory-before-confirmation birth

未关联 violation packets 在 2 s buffer 中按 0.05 s group 去相关。每对不同 group 先提出 CV
hypothesis；每组最多选择一个最小 residual inlier，再做加权直线 refit。B4 要求至少 5 个独立
groups、持续至少 0.1 s、速度不超过 15 m/s、RMS residual 不超过 0.8 m，并同时过 packet
Mahalanobis gate。

同一 1 m cell 在一个 map epoch 最多一个 birth；新 birth 还会消费拟合轨迹 footprint 内的未用
packets。实现位于 `bestBirthCandidate/createBirth`（`core.cpp:1358–1600`）。hover 的零速轨迹
合法，因此没有用最小速度排除静止 UAV。

B1→B2 单因素结果：birth/run 10.21→7.13，unique confirmed/run 6.76→5.92，FP/run
592.9→546.8；HOTA 仅 0.341→0.355。trajectory birth 减少 duplicate hypotheses，但单独不足以
解决 ghost 和 feedback 正循环。

## 3. scan opportunity

每条 track 由均值和位置协方差三个特征轴上的 ±sigma 共 7 个点表示。ray angular index 先筛选
可能相交 rays，再对每个候选执行真实时间的 ray-sphere 近交点、VALID_RETURN 前景遮挡、
NO_RETURN 有效距离和 confirmed front-track 遮挡。

第 \(i\) 条 ray 的几何有效权重为 \(o_i\)，距离分箱回波概率为 \(p_{ret}(r_i)\)，则：

\[
P_D=\min\left(P_{D,max},1-\prod_i(1-o_i p_{ret}(r_i))\right).
\]

canonical \(P_{D,max}=0.95\)，CAL04 分箱为 `[0.916, 0.878, 0.877, 0.5]`，边界
`[10,20,30]` m；30 m+ 没有无遮挡 CAL 样本，0.5 是保守 fallback。实现为
`indexRays/nearbyOpportunityRays/opportunity/detectionProbability`
（`core.cpp:1782–2007`）。

## 4. existence 更新

每个 scan 只更新一次，不随 10 ms micro-batch 重复计罚。先做连续时间 survival：

\[
r^- = r\exp(-\lambda_S\Delta t),\qquad \lambda_S=0.1\ \mathrm{s^{-1}}.
\]

未命中且本 scan 已提交 map epoch 时：

\[
r^+=\frac{r^-(1-P_D)}{1-r^-P_D}.
\]

命中时用 packet likelihood \(L(z)\) 与 clutter density \(\kappa\)：

\[
r^+=\frac{r^-P_DL(z)}{r^-P_DL(z)+(1-r^-)\kappa}.
\]

代码使用 log-domain 计算 hit denominator，位于 `missedExistence/survivalExistence/hitExistence`
（`core.cpp:1082–1119`），scan-level 调用位于 `processScan()`（`core.cpp:2483–2557`）。

## 5. duplicate merge 与超时

duplicate merge 同时要求：绝对距离 ≤0.25 m、position Mahalanobis \(d^2\le1\)、速度差
≤0.5 m/s、最近测量时间差 ≤0.05 s、birth 时间差 ≤0.5 s，并要求 measurement/support 历史一致。
只保留 existence、更新数、年龄和 covariance 更优者，不平均两个状态。0.5/1/2 m 两真目标不会
只因空间接近而合并。

tentative/confirmed 分离 timeout，deleted confirmed 仅保留 2 s bounded quarantine。实现位于
`duplicateTracks/mergeDuplicateTracks/prune`（`core.cpp:1287–1344/1624`）。

## 6. 结果与因果边界

正式 B2 使用 3 groups，CAL 后的 B3/B4 使用 5 groups，因此正式 B2→B3 同时改变了 birth
门槛和 opportunity/survival，不是严格单因素。100 个配对 run 的观测差值为：

- `>3 s` unique ghost tracks：229→0；
- max stale age/run：4.078→0.426 s；
- fragmentation/run：2.66→1.89；
- FP/run：546.8→277.5；
- 但 TP/run 同时下降 19.14，FN 相应增加。

S05 的 fragmentation 为 8.5→1.9，但这组正式差值不能全部归因于 opportunity。严格同为
3 groups 的 Phase 12 N0/seed1001 控制中，S05 B2→B3 fragmentation 9→3、stale>3 s
78→1；这才是“opportunity/survival 降低 fragmentation”的直接证据。B4 S05 HOTA 0.254 仍
低于 B0 0.333，FN 193.7 高于 147.7，收益不能表述为无代价或多种子单因素证明。

B3/B4 的 100 个 run 均无 `>3 s` ghost；B4 max stale 最大 1.762 s，未出现长期 ghost population。
