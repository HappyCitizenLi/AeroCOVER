# SOFT-VoFOD V4 Epistemic Birth

## 1. 两个假设与顺序证据

U packet 不再用一次性速度或 χ² 阈值直接判 target。每条空间链同时维护：

- `H_B`：世界坐标中的静态点；
- `H_T`：带白加速度过程噪声的 CV pre-track。

每个独立时间组只更新一次 log-likelihood ratio。阈值固定为 `tau_T=tau_B=6`；pre-track
acceleration sigma 为 1.0 m/s²，initial velocity variance 为 1.0 m²/s²。未决 packet 不写 background；
static decision 后显式同化。U-birth 只能由 sequential `independent_motion` decision 解锁，旧 D² 路径
不会 fallback。

## 2. 三联门控证据

同源 seed 1008：

| case | U0 | U1 sequential | 结论 |
|---|---:|---:|---|
| S08B moving unknown HOTA | 0.497050 | **0.920358** | 显著提升 |
| S08B TTFT | 18.2 s | **5.2 s** | 大幅下降，仍未满足 final `<5 s` |
| S08B FP/FN | 0 / 256 | **0 / 52** | FN 大幅下降 |
| S08C false static confirmation | 0 | **0** | 保持 0 |
| IT11 mapped-free hover recall | 1 | **1** | 保持 1 |
| IT11 TTFT | 0.797 s | **0.797 s** | 无回归 |

S08B target 出现于 14.596 s，第一条 U packet 为 14.821 s，motion decision 为 19.718 s；decision 到
publish 仅约 78 ms，主要延迟是收集可辨识的 sequential evidence，而不是 ROS 发布或 tracker。

## 3. 真正 cold-start 暴露的问题与修正

零背景 CS03 初测产生 4 个错误 confirmed tracks：三个来自 map boundary 静态墙的稀疏链，一个来自
stationary target 在自身存在期间形成的 certified-free history。修正包含：

1. unknown chain 紧贴固定 map boundary 时不得 birth；
2. static-unknown 判定写入 bounded voxel provenance；
3. 后续与该 provenance 重叠的 F packet 可维护 track，但不能创建新 track。

同一 CS03 source 的 false confirmation 依次 `4 -> 1 -> 0`，最终 active IDs 为 0。CS02 moving
unknown 仍可 birth，说明不是全局禁用 unknown birth；最终一次已记录回放的 TTFT 为 6.301 s，较修正前
2.201 s 更保守。该 trade-off 需要多 seed 重新验证。

## 4. 信息论边界

静态未知物体与静态背景在只有几何点返回、没有先验 free history 时不可辨识。系统只能保持 unresolved
或同化为 background，不能诚实地声称识别“静止 UAV”。只有独立运动证据，或物体进入在它出现之前已
认证的自由空间，才能升级为 target evidence。
