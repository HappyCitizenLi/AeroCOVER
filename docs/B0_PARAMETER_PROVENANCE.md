# B0 canonical 参数来源

状态：Phase 1 冻结，2026-08-20。主实验只能使用
`src/vofod_mid360/config/b0_mid360_canonical.yaml`、
`src/vofod_mid360/config/raycast_mid360.yaml`、
`src/vofod_mid360/config/sensors/mid360.yaml` 和
`src/lidar_tracker_mid360/config/tracking.yaml` 的组合；不得按场景覆盖算法参数。

## A. 纯接口必要参数

| 参数 | 默认值 | 来源/理由 |
|---|---:|---|
| `expected_rays_per_bundle` | `20000` | 受控仿真 source contract |
| `require_expected_ray_count` | `true` | B0 fail-closed |
| `sensor/source_mode_required` | `sim_exact` | 主仿真只接精确完整射线 |
| `input/geometry_consistency_tolerance_m` | `0.01` | detector 与 checked endpoint 软件一致性上限 |
| preprocessor stamp tolerance | `1 us` | source metadata 自检；当前 ExactTime 正常值为 0 |
| preprocessor range tolerance | `0.01 m + 0.001*r` | PointCloud float/传感器量化接口容差 |
| body mask default | allow，blocked list 空 | 仿真无机身命中；硬件必须另行标定 |

## B. Mid360 全局标定参数

| 参数 | 默认值 | 当前依据 |
|---|---:|---|
| detector voxel size | `0.5 m` | 既有 Mid360 B0 全局尺度；待 CAL02 sensitivity |
| component tolerance | `1.25 m` | 连接 `0.5 m` lattice 的 `(2,1,1)` 稀疏间隔 |
| background coverage ratio | `0.01` | 与独立 24-sure gate 联用的既有全局值 |
| valid/no-return free weight | `0.003 / 0.003` | 上游风格单一弱自由权重 |
| endpoint guard | `0.5 m` | 一个 detector voxel |
| free/no-return maximum range | `20 / 20 m` | 当前受控仿真可靠范围，需 CAL01 sensitivity |
| tracker radius min | `0.75 m` | 目标半尺寸和预测余量的全局尺度 |
| tracker input downsample | `0.5 m` | 既有全局值；singleton 仍被允许 |
| tracker local cluster tolerance/max OBB | `1.0 / 1.0 m` | 既有 Mid360 目标尺度 |

后续 clustering sensitivity 必须统一扫描
`{0.3,0.5,0.75,1.0,1.25,1.5} m`，按 range 和 inter-target separation 报告 split/merge；
结果只能用于全局冻结或揭示 B0 限制，不能产生场景配置。

## C. 算法语义参数

| 参数 | 默认值 | 语义 |
|---|---:|---|
| B0 min component points | `2` | 上游风格 floating component 门 |
| max component OBB/distance | `3.0 / 50 m` | B0 目标候选门 |
| background/explore distance | `1.5 / 3.0 m` | 历史背景 close/floating topology |
| separated background | `0.1 s`, `0.8 m`, `24` | 原风格孤立背景清理 |
| background warm-up | enabled, `10 s` | 无目标 sensor-only startup；还必须通过两个 map maturity gate |
| tracker state | 9-state CA | classic baseline，不升级为 proposed tracker |
| Q variance rates `[p,v,a]` | `[0.01,0.8,0.05]` | `mrs_lib` 在 predict 中乘真实 `dt` |
| P0 variances `[p,v,a]` | `[0.3,1.0,0.05]` | 冻结初值 |
| R position variance | `0.1 m²` | 固定 B0 measurement covariance |
| covariance radius multiplier/max | `1.5 / 5.0 m` | 等体积 1-sigma 半径与删除门 |
| confirm detection count | `2` | classic tracker 语义 |
| background filter | `true` | 只接 `vofod_mid360/background_points` |

Q/P0/R 数值均为 variance/covariance，不是 standard deviation。Phase 1 修复了 cfg 文案和
半径单位，但没有借接口适配之名重调这些数值。参数选择的正式 calibration/sensitivity
证据属于 Phase 2 benchmark 工作；在完成前不得声称这些值是硬件最优值。
