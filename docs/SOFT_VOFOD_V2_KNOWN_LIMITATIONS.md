# SOFT-VoFOD V2 已知限制

## 已观察到的算法失败

1. **S04 多目标仍弱于 B0。** B4 HOTA 0.894 vs 0.987，FP 110 vs 12.2，fragmentation
   13.0 vs 0.1。四目标交叉下的 packet maintenance、existence 与反馈交互尚未闭环。
2. **S05 recall 代价过大。** fragmentation 下降，但 B4 HOTA 0.254 vs B0 0.333，FN
   193.7 vs 147.7。当前 opportunity/survival 对长遮挡后的稀疏重获仍偏保守。
3. **S08C 分类语义失败。** unknown 中静止物在 10/10 run 被确认为一条轨迹；仅几何下无法可靠
   区分静止 UAV 与新静态背景。HOTA 高不能掩盖这个失败。
4. **S08A 仍有 false packets。** 虽为 0 birth/track，false packets 仍为 12.53/min，expansion
   recall 仅 0.609。

## 地图限制

- B4 static-background recall 高于 B0，但 false-free 0.2247 高于 B0 0.1744，free/background
  balance 尚未解决；
- B3→B4 contamination 近似不变，confirmed protection 没有被正式矩阵证明能减少 contamination；
- stable background 当前是保守长期记忆，没有在线 forgetting/地图重构策略；
- candidate matching 仍基于 centroid/voxel overlap，快速视角变化和稀疏远距表面会使收敛变慢；
- NEG03 recall 0.0439、false-free 0.2207，moving-observer 地图质量仍不理想。

## 评估与标定限制

- 正式矩阵只有 5 seeds，不是推荐的 10 seeds；
- 正式 B2=3 birth groups，B3/B4=5 groups，B2→B3 多 seed 对比不是纯 opportunity 单因素；
- `algorithm_config_sha256` 哈希 base config files，不单独编码 launch overrides；必须结合 algorithm
  名称和 runner commit 才能复现；
- CAL04 的 30 m+ return probability 没有无遮挡样本，0.5 是未标定 fallback；
- CAL 都来自当前仿真传感器/几何，不可当作真实 Mid360 标定；
- 100 source bags 和 500 output bags 因磁盘约束删除，只保留 metrics/manifest/CSV/log；原始数据
  只能确定性重录，不能从仓库恢复；
- B0 不发布和 B4 相同的 existence/confirmed-state diagnostics，不能直接比较 B0 unique confirmed
  或 stale existence。

## 系统边界

- 尚未做真实 Mid360 多目标实验；
- 尚未接入 MRS/PX4 闭环或 collision avoidance decision；
- per-ray moving-observer 几何已验证，但完整 rolling collision 对运动物体仍未实现；
- 没有证明极端雨雾、强反射、多径或真实网络/CPU contention 下的行为；
- 当前地图为固定局部体积，不是大范围 SLAM/rolling map。

因此当前版本适合作为继续研究的仿真基线，不应直接作为飞行安全组件，也不能声明全面超过 B0。
