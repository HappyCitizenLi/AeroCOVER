# AeroCOVER 自适应球壳与 VoFOD 四场景统一适配方案

已有 v2 的 16 项指标是旧实现结果，不能用来证明后续改动有效，原始输入、参数快照及结果不改写。

## 当前已生效：用户最新指定的参数

四场景八份 VoFOD 运行配置及其生成器已更新：detector tolerance=0.5 m，min_points 为 Mid-360=1、OS1=2，max_size=1.5 m，d_close 全部为 0.3 m。Tracker 输入体素=0.25 m，聚类 tolerance=0.5 m，max_size=1.5 m。

AeroCOVER 的 C++ 默认值和基础 YAML 中 `no_return_trusted_range_m` 已从上一轮 40 m 改回 **20 m**，自适应球壳继续启用。VoFOD 的 ray 距离原本就为 20 m，本轮未改变。

最新追加要求已应用：Tracker 背景距离余量=0.1 m，不确定半径最小/最大值=0.6/3 m，并同步修改两条入口 launch 及 Tracker launch 的覆盖默认值。

未采用下方历史方案的其他建议：detector 体素仍为 0.25 m、检测距离仍为 50 m，初始化、噪声参数和可视化设置均不变。本轮未重跑正式实验。新的高速轨迹重设计需先确认速度基准、逐机峰值要求及圆柱 0.5 m 的净空定义；此前四场景轨迹尚未据此改写。

## 已实现的 AeroCOVER 改动

- 上一轮曾将 NO_RETURN 可信距离改为 40 m，现已按最新要求回到 20 m。OS1 launch 先加载该基础配置，因此同样生效；不改变有返回射线的 endpoint margin、点历史或 ray FIFO。
- 在 ST 分类完成后，用本帧已分类背景点建立 PCL KD-tree，查询每个残差 component 中心到最近背景点的距离 d。只使用传感器点云，不读取 world 几何、目标 ID 或真值。
- 内半径不变；外半径取 `min(原外半径, d - 0.10 m)`。最大厚度仍为 1.00 m，最小厚度为 0.20 m。放不下最小厚度时拒绝该观测，不强行把半径夹到一个穿过障碍的最小值。
- 未看到背景点时维持原球壳，不把未知空间当自由空间。点云间隙和漏分类意味着这不是对连续真实表面的避障保证；仍须通过历史 ray 的完整弦和支持 bin 判定。
- 42-bin 出生、29-bin 维护、3/5 birth、1 m prediction gate 及 1 s missed timeout 均保持不变。缩壳不重标定证据权重/归一化长度，不凭缩壳自动授予 shell pass。
- 自适应模式下，候选中心、内外半径任一改变均使该候选的增量证据缓存失效并重算，避免复用旧球壳证据。当前扫描的 ray 仍在当前判定之后提交。
- 新增 `adaptive_shell_shrunk_count`、`adaptive_shell_blocked_count` 诊断及 CSV 导出；KD-tree 构建和查询开销计入已有 `cluster_ms`。

配置开关为 `evidence/obstacle_adaptive_shell`，可关闭以做对照；旧版本复现还应使用旧参数快照中的 20 m NO_RETURN 配置。

40 m 可信距离针对当前仿真输入。真实设备必须验证 NO_RETURN 与无效/低反射/遮挡状态的区分，不能仅按标称量程宣告 40 m 自由空间。目标中心若到 40 m，球壳远端可能超过 40 m，本次距离上限并不保证整个评分边界都有完整 shell 证据。

## 历史建议：未整体采用的 VoFOD 统一适配方案

四场景共享同一组几何参数，仅 Mid-360/OS1 的最少点数和初始化门槛不同。它是新的适配版，不再称为完全原论文参数；尚无指标收益保证。

| 配置项 | 当前 | 建议 Mid-360 | 建议 OS1 |
|---|---:|---:|---:|
| `voxel_map/voxel_size` | 0.25 m | 0.20 m | 0.20 m |
| `clustering/tolerance` | 1.50 m | 0.50 m | 0.50 m |
| `clustering/min_points` | 2 | 1 | 2 |
| `clustering/max_size`（OBB 对角线） | 3.00 m | 1.20 m | 1.20 m |
| `clustering/background_distance` | OPEN/MT 1.5、其余 0.3 m | 0.20 m | 0.20 m |
| `clustering/max_distance` | 50 m | 40 m | 40 m |
| `background/sufficient_points_ratio` | 0.0001 / 0.15 | 0.0001（保留） | 0.15（保留） |
| `separate_background/min_sure_voxels` | 1 / 24 | 1（保留） | 24（保留） |
| `raycast/max_distance` | 20 m | 40 m | 40 m |
| `raycast/reliable_no_return_distance` | 20 m | 40 m，须满足上述输入可信条件 | 同左 |
| Tracker `input_filter/downsample_leaf_size` | 0.50 m | 0.20 m | 0.20 m |
| Tracker `association/clustering_tolerance` | 1.00 m | 0.40 m | 0.40 m |
| Tracker `association/cluster/max_size` | 1.00 m | 1.20 m | 1.20 m |
| Tracker `association/cluster/min_background_dist` | 1.00 m | 0.10 m | 0.10 m |
| Tracker `lkf/P/radius/min` | 2.50 m | 0.60 m | 0.60 m |
| Tracker `lkf/P/radius/max` | 5.00 m | 3.00 m | 3.00 m |
| `output/publish_free_voxels` | true | false（仅跟踪主实验） | false（仅跟踪主实验） |

继续保留地图范围、全局 body mask、Q/P0/R、协方差第六根、Tracker 至少两次 detector 检测确认、ray 权重 0.003、valid-return margin 0.25 m、max_explore_distance 3 m。背景点云发布仍保留供 Tracker 使用；不把降低辅助可视化发布频率当作检测算法优化。

修改半径时须同时修改 `lidar_tracker.launch` 的 `radius_min` 参数传入值：该 launch 会覆盖 tracking.yaml 中的最小半径，单改 YAML 不会生效。ray 距离位于单独加载的 `raycast_mid360.yaml`，不能只改场景的 b0 配置。

### 对四场景的作用和风险

- OPEN：降低两级降采样尺度，Mid-360 允许单点 detector 簇，减轻远距小目标被采样/最少点数门槛删除；40 m ray 范围与点检测范围一致。较小最大不确定半径可能增加长缺测后的删除，必须同时检查 Frag。
- MT：初始关联门由至少 5 m 降到至少 1.2 m，低于约 2.42 m 的相邻目标间距。但协方差增长后门仍会扩大，且顺序最近邻仍会复用同一轨迹，参数不能保证避免多目标吸收。
- OFFICE：较小聚类容差减轻目标与墙的连通，detector/Tracker 的背景距离一起调整。不能只把 detector 的 d_close 设为 0.2 m，就以为整条链只排斥 0.2 m。
- FOREST：缩小允许的簇尺寸和关联范围，减轻树枝/地面碎片被维持成轨迹；延长 ray 更新范围改善远处静态环境的建图。但降低最少点数和 d_close 也可能增加 FP，必须测量，不能保证方向性收益。

0.20 m 体素的固定地图体素数约为当前的 1.95 倍，40 m ray 遍历也更贵，须重新测 runtime/RSS。关闭 free_voxels 只减少辅助输出，不改背景建图，但相关 free-map 评估不再可用；如继续比较该辅助指标，需要保持输出开启并单独承担开销。两算法的 runtime 计时边界仍不同。

## 参数方案无法替代的机制改动（本轮不实施）

1. Tracker 当前按 `OBB 对角线 + min_background_dist` 查询背景，即使余量降到 0.1 m，仍可能拒绝近墙目标。真正按几何净空过滤，应改为簇表面/OBB 到背景的距离；半对角线只是较保守的替代，不能等同精确表面距离。
2. 将同帧检测关联改成批量一对一分配，并限制不确定轨迹的吸收范围，才能结构性处理 MT；仅缩小最小半径不够。
3. “未找到与地面连接”不等于“无人机”。FOREST 的新可见静态碎片需要更可靠的背景/自由空间证据和确认机制，不能靠放宽或收紧一个门槛完全解决。

## 验证顺序

当前应先比较固定球壳/20 m 与自适应球壳/20 m；40 m 仅保留为可选参考，不是当前默认配置。VoFOD 使用本文件开头的用户指定适配组，再用全部四场景、两传感器统一比较；保留旧版结果，检查 FP/FN/IDSW/Frag、各目标召回、ready 延迟、消息覆盖、runtime 和 RSS。最好另加未参与选参的轨迹或 seed 检查泛化。

本轮未重跑 16 项正式实验，不能据已有 v2 指标宣布 IDSW/Frag 已消除。

实现已编译；核心测试覆盖近障碍缩壳、空间不足拒绝、无 ray 不出生、40 m 与旧 20 m 的远距证据差别，以及变化球壳的缓存 generation、并行预筛和精确参考一致性。
