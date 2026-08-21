# 已知限制

状态日期：2026-08-21

## 信息论与观测限制

1. target 在系统启动前就静止于从未建图的 unknown 空间时，没有先验背景便无法与新静态物体
   区分；当前实现会按 conservative candidate-background 处理，而不会凭空创建 target。
2. 两个完全共线目标若只形成一个不可区分 return，identity 暂时不可观；Hungarian 不能创造
   不存在的观测。
3. target 紧贴 stable background 时，birth 所需的 free-space violation 会被背景 exclusion
   抑制；已有 track 可用非 stable-consistent 点维护，但新 birth 可能失败。
4. 极低反射目标若长期既无返回又没有足够 emitted-ray intersection，existence 不下降也不增加；
   这是“无机会不作负证据”的预期语义。

## 当前精简实现边界

- `p_ret=0.5`、measurement covariance、clutter density 和各阈值是统一工程默认值，尚未由
  CAL01/CAL02/CAL03 标定，不能宣称是真实 Mid360 参数。
- map 是固定边界 dense grid，不滚动、不分页；超出 operational region 的 ray 不更新。
- nearest stable-background 查询是有界局部 voxel 搜索；大 exclusion radius 会增加 CPU。
- trajectory birth 使用确定性 pair hypotheses，对最多 256 个 buffer events 枚举并重评分；高
  anomaly clutter 是当前主要最坏情况计算瓶颈。该上限是显式降载，不是隐藏丢帧。
- association 在每个 10 ms micro-batch 的末时刻更新 KF；event 和 opportunity 仍保留逐 ray
  timestamp。更高动态场景可缩短 micro-batch，但必须统一标定计算预算。
- sigma-point opportunity 使用七点等权近似和固定物理 sphere，不是精确 target shape model。
- event/deleted-track quarantine 是短时固定球；未实现 shadow-only birth、JPDA/MHT/PMBM 或
  learned semantics。

## Pose、仿真和评测边界

- adapter 已处理 stamp 回退和大时间跳变，但输入没有 pose covariance，因此尚不能实现完整
  TF discontinuity/covariance quality gate。pose 错误可能造成大范围 false anomaly；不得用 truth
  修正。
- 当前 `per_ray_pose` 仍只插值 observer geometry；移动 collision scene 的 rolling-scan 限制见
  `docs/ROLLING_SCAN_LIMITATION.md`。
- S01–S07 的 N0/seed1001 开发矩阵已完成，但 N1/N2、多 seed、CAL split 和 2–5 min
  NEG01–NEG03 尚未执行，不能作统计显著性或真实 Mid360 声明。
- S07/A3 实测 p95 约 1.00 s、processing load ratio 2.93，必须用 0.1× 离线回放才能保证完整
  帧覆盖；当前 moving-observer 条件不实时。
- S07 的 proposed event FP/min 约 5.38 万，说明 observer-motion/per-ray 近似下背景一致性失败，
  不能用高 event recall 掩盖。
- 当前 fixed-threshold HOTA@1m 不是跨定位阈值积分 HOTA；opportunity Brier 仅在已出生 tracks
  上计算；map truth 是 0.5 m primitive rasterization。详细定义见实验报告。
- source truth evaluator 在 S03/S04 分别覆盖约 99.2%/97.8% scored input；已通过 95% 门禁，
  但不是逐输入帧 100% truth。
- 当前 Gazebo 参考轨迹不证明 MRS/PX4 闭环、气动、避障或飞控性能。
