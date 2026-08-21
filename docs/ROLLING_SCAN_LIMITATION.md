# Rolling-scan 能力边界

当前 `snapshot` 与 `per_ray_pose` 都属于 Tier A 算法开发输入。

- `snapshot`：整束射线使用 scan-start observer pose 和一次 collision scene snapshot。
- `per_ray_pose`：用 scan 首尾 TF 对 observer 平移/旋转插值，并为每束射线构造不同
  world origin/direction；collision scene 在 bundle 内仍受静态快照假设约束。
- `/uav1/mid360/points_world` 消除了 detector/tracker 的纯软件几何差异，但没有让移动目标、
  移动墙或加速度/jerk 在 Gazebo collision query 中随 ray time 重放。

因此不得把当前 `per_ray_pose` 称为完整 rolling-scan simulator，也不得据此声称验证了
MRS/PX4 闭环、气动、飞控或避障。

后续 Tier B 应采用 1--5 ms geometry chunk：observer 与目标真值轨迹插值到 chunk time，
静态环境保持静态，每条 ray 保留原 `offset_time_ns`。只有 Tier B 实现及独立回归通过后，
论文实验才能标为 rolling-scan stress test；Phase 1 不引入自定义 physics engine。
