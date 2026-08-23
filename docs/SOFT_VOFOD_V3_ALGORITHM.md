# SOFT-VoFOD V3 算法

状态：V3 窄修正已进入同一 `soft_vofod_mid360` production path；本文描述 canonical V3-C，
V3-A/V3-B 和单因素 variants 仅由 runner launch override 关闭模块，不复制实现。

## 1. 方法边界

输入仍是严格配对并已完成逐射线 ego compensation 的 Mid360 world-frame points/rays。核心不读取
scene、truth、target ID 或可见性标签；truth 只进入离线 evaluator。V3 保留 V2 已验证的 5 Hz
deferred map、short-time packet birth、singleton packet、逐射线 opportunity 和 confirmed-track-limited
map support。

V3 的最小理论链为：

```text
complete emitted rays
 -> OBSERVED_FREE / CERTIFIED_FREE / persistent background
 -> certified-free violation OR unknown independent motion
 -> packet-level tracking + CV/CA IMM
 -> opportunity-aware existence + reportability
 -> occluded/dormant old-ID reacquisition
```

## 2. 单 scan 执行顺序

1. 校验 ray 时间/几何合同，并清除上一 scan 已标记 `DELETING` 的轨迹；
2. 用完整 scan 的角度索引计算前景遮挡，避免 5 Hz wall packet 把隐藏轨迹拉到墙上；
3. 按 10 ms micro-batch 分类 endpoint，但只在 5 Hz map epoch 提交长期地图；
4. 从 certified violation、unresolved unknown 和 track-explained endpoint 形成短时 packets；
5. 预测 CV/CA IMM；必要时按 predicted track 唯一拆分 mixed maintenance packet；
6. Hungarian 一对一关联并做 mode-conditioned KF update；
7. 未关联的合法 F/U packet 先尝试受限 dormant reacquisition，再进入 birth buffer；
8. 对 active/occluded track 用真实 rays 更新 opportunity、survival、existence 和 reportability；
9. confirmed target 仅产生有界 map support/quarantine；free/background evidence 留到 epoch 提交。

## 3. 两条互不混淆的 target birth 路径

`CERTIFIED_FREE_VIOLATION` 证明“目标出现前这里被多次独立观测为空”，所以允许接近零速的 hover
birth。`UNKNOWN_INDEPENDENT_MOTION` 不依赖 free 先验，但必须有跨独立时间组、协方差归一化的显著
位移；unknown 中持久而静止的返回保持 unresolved/candidate background。`TRACK_REACTIVATION` 只用于
恢复已有 confirmed ID，不创建新 ID。

## 4. 地图与轨迹状态

地图状态为：

```text
UNKNOWN -> OBSERVED_FREE -> CERTIFIED_FREE
UNKNOWN -> CANDIDATE_BACKGROUND -> STABLE_BACKGROUND
```

background surface uncertainty band 可以保留 observed-free 证据，但禁止 certified。轨迹状态为：

```text
TENTATIVE -> CONFIRMED_ACTIVE <-> OCCLUDED -> DORMANT -> DELETING
                                  ^             |
                                  +-- reacquire-+
```

`existence_probability` 表示物理存在信念；`reportability_score` 决定是否对外发布。Dormant 保留 ID 和
有界 reachable region，但不作为普通输出，也不参与 strict maintenance。

## 5. Canonical 参数与来源

运行时权威配置是 `config/soft_vofod_v3_canonical.yaml`；C++ 默认值只是缺参 fallback。

| 参数组 | canonical 值 | 来源 |
|---|---:|---|
| certified free | 3 epochs、0.4 s、至少 1 valid epoch | CAL06/07 候选中心值 |
| surface guard | 0.75 m | CAL08 候选中心值 |
| unknown motion | χ²(3) 99.9%，16.266 | CAL epistemic 候选 |
| packet split gate | association χ² gate 11.345 | engineering default |
| IMM transition | CV→CA 0.05，CA→CV 0.10 | engineering default |
| CV/CA noise | acceleration/jerk σ=3 | engineering default |
| occlusion/dormant | score 0.6，enter 1 s，timeout 10 s | engineering + CAL09 protocol |
| dormant gate | χ² 16.266，15 m/s，6 m/s² | conservative engineering bound |
| reportability | τ=1 s，σ scale=1.5 m，threshold=0.2 | engineering default |
| survival | λ=0.1/s | inherited CAL05/CAL09 candidate |

30 m 以上 `p_ret=0.5` 仍是 fallback；不能称为真实 Mid360 标定。

## 6. 有界性和实现位置

所有新增模块仍在 `core.h/core.cpp`：voxel 仅增加常数个 epoch counters；birth buffer 固定 2 s/256
packets；split 只处理当前 global packet 的 points；IMM 只有两个固定维度模式；dormant 保存时间上限
10 s；support 和 opportunity 继续使用 voxel/angular index。ROS adapter、launch 和 evaluator 只扩展
参数/diagnostics，不形成第二套算法。

最终门禁、数值结果和是否进入 R6/R7 见 `SOFT_VOFOD_V3_FINAL_REPORT.md`。
