# B0: VoFOD-Mid360 + classic lidar_tracker

冻结日期：2026-08-20（Phase 1）

## 定义

B0 保留 VoFOD 的 occupancy-state、component classification、raycasting、
separated-background removal 和原风格 point-cloud-aided tracker，并对 Mid-360 的
非组织点云、完整真实射线、scan identity、逐点世界几何、确定性事务和输入安全作必要适配。
它不是“原始 VoFOD 完全复现”。

唯一主入口：

```bash
roslaunch tclv_evaluation b0_canonical.launch
```

算法入口不启动 sensor、TF、clock、scenario 或 truth。主实验必须先启动共同 source/replay
和 `mid360_ray_preprocessor`。

## 冻结数据合同

```text
points_raw + rays_raw + scan_identity
  -> ExactTime + explicit ScanIdentity validation
  -> points_world + rays_checked
  -> vofod_mid360
  -> detections + occupied background
  -> lidar_tracker_mid360
  -> tracks
```

`ScanIdentity.scan_id == RayBundle.scan_id`，并同时绑定 point/ray source stamp、
`pattern_start_index` 和两侧 count。预处理器验证 source order、唯一 `original_index`、
VALID_RETURN 唯一匹配及 range 容差；任何核心失败均不发布三个派生输出。

`points_world` 是 world-frame `PointCloud2`，字段为：

```text
x y z intensity original_index offset_time_ns scan_id
```

每点严格使用 `CheckedRay.origin + range*CheckedRay.direction`。detector 读取并复核该坐标，
tracker 直接使用该坐标且不再执行 bundle-level TF 或 sensor-frame self crop。

## 启动与地图

canonical B0 使用 10 s target-free background warm-up，不使用 `nadir_seed`。warm-up 中算法
只见传感器数据，不订阅 target/truth；观察到的 component 暂作 background，且必须同时满足：

1. warm-up duration 已达到；
2. historical occupied ratio gate 已达到；
3. 至少一个 24-sure connected-background component 已形成。

`MapUpdateDiagnostics.background_warmup_complete` 是外部场景进入正式评测的门。若 source
期间已有目标，warm-up 会污染背景；这是输入协议违规，不由 truth correction 修补。

`background_points` 只含 sure occupied voxel；tracker background filter 固定开启且只接该
topic。`free_voxels` 不得接入 background KD-tree。

## 冻结 tracker

- 9-state CA LKF、greedy nearest association、局部 Euclidean cluster/OBB、原风格 merge；
- Q/P0/R 按 covariance/variance 使用；Q 由 `mrs_lib` 乘真实 `dt`；
- covariance radius 为
  `multiplier * sqrt(cbrt(det(P_position)))`，单位为米；
- singleton local cluster 允许；background filter 开启；
- 输入只接受 configured world frame 的 `points_world`。

参数完整来源见 [`B0_PARAMETER_PROVENANCE.md`](B0_PARAMETER_PROVENANCE.md)。

## 明确不包含

- free-space violation birth、trajectory birth；
- scan-opportunity/existence、target quarantine；
- GNN/Hungarian、JPDA/MHT；
- truth、scene name、target ID/初值、墙坐标特判；
- 完整 rolling-scan dynamic-scene physics。

## 回归边界

Phase 1 tests 覆盖 scan identity、source order/range、fail-closed、snapshot/per-ray world
endpoint、time rollback、空场/墙体、B0 soft ray updates、warm-up mask、tracker singleton、
covariance units、frame completion 和长期无量测删除。冻结后如需改变算法语义参数，应创建
新的 baseline 版本；不要覆盖本规格。建议人工创建 tag `b0-mid360-frozen`，本任务不自动 tag/push。
