# SOFT-VoFOD V3 Unknown-space Ambiguity

## 1. 判别边界

低 CV residual 只证明 worldline consistency，不证明物体相对世界发生独立运动。V2 因此会把 unknown
中的静止物当作“完美零速轨迹”。V3 把 birth evidence 固定为三类：

```text
CERTIFIED_FREE_VIOLATION
UNKNOWN_INDEPENDENT_MOTION
TRACK_REACTIVATION
```

## 2. Unknown birth

对两个不同时间组的 unknown packets，计算

\[
D^2=(z_k-z_0)^T(\Sigma_k+\Sigma_0)^{-1}(z_k-z_0).
\]

只有 `D² > 16.266`、pair 时间在 0.02–2 s、速度不超过 15 m/s，并且之后满足统一的 5-group
trajectory birth 条件时才可确认。实现没有使用简单的固定 speed threshold，也没有 scene special
case。静止/抖动 unknown packet 会进入 unresolved/candidate-background 路径，不会生成 confirmed
target。

## 3. 不破坏 hover 的两个例外

- 物体出现在已经 certified 的 free voxel：provenance 是 F，允许估计速度接近零；
- 已 confirmed target 进入 unknown 后悬停：这是 maintenance，不重新应用 unknown birth rule。

Dormant 中的 far-away unknown reactivation 同样要求一对独立 U packets 通过运动显著性；只在旧预测
邻域内时才允许连续性直接提供兼容证据。任意静态 surface packet 不可激活 dormant ID。

## 4. 三联 gate

R2 必须同时满足：S08C 0 false confirmation、S08B moving unknown 可 birth、IT11 mapped-free hover
可 birth。任何单项通过都不能证明语义修复完成。S08C 的正确输出是 unresolved，不是强制 background；
这也是纯 LiDAR 信息论边界的显式表达。
