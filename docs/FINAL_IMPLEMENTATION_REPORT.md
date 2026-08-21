# SOFT-VoFOD 第二阶段算法实现报告（历史阶段记录）

状态日期：2026-08-21  
范围：精简版最终算法 A3 的阶段性记录。后续 benchmark/evaluator 已完成，见
`SOFT_VOFOD_EXPERIMENT_REPORT.md`。

## 1. 结果

新增单包 `soft_vofod_mid360`，完成以下 truth-free 在线闭环：

```text
points_world + rays_checked
 -> conservative background
 -> free-space violation event
 -> trajectory-before-confirmation birth
 -> CV-KF + Hungarian GNN
 -> scan-opportunity Bernoulli existence
 -> target quarantine / ray truncation
 -> multi-target tracks and map/debug outputs
```

B0 源码和参数未被 proposed logic 修改。算法 launch 不启动另一份 sensor/preprocessor，确保它
可以与 B0 消费同一输入。

## 2. 文件与职责

| 路径/符号 | 职责 |
|---|---|
| `src/soft_vofod_mid360/include/soft_vofod_mid360/core.h` | 配置、地图/event/track 数据合同和纯核心 API |
| `src/soft_vofod_mid360/src/core.cpp` `BackgroundMap` | 四态背景 memory、概率、DDA free carving、candidate maturity |
| `core.cpp` `bestBirthCandidate/createBirth` | independent-group deterministic RANSAC-CV/refit birth |
| `core.cpp` `transition/processNoise/processBatch` | CV-KF、Mahalanobis gate、Hungarian、packet median |
| `core.cpp` `opportunity/missedExistence/hitExistence` | sigma-point opportunity、遮挡、existence |
| `core.cpp` `supports/truncateBeforeSupport` | active/deleted/event quarantine 与射线截断 |
| `src/soft_vofod_mid360/src/soft_vofod_node.cpp` | ExactTime/scan contract、参数、消息和 time-jump gate |
| `src/soft_vofod_mid360/msg` | event、track、opportunity 输出 ABI |
| `src/soft_vofod_mid360/config/default.yaml` | 唯一默认 A3 参数集 |
| `src/soft_vofod_mid360/launch/soft_vofod.launch` | 算法唯一入口 |
| `src/soft_vofod_mid360/test` | 纯核心回归和 ROS end-to-end free→event→birth 测试 |

公共基础设施与 B0 修复仍见 `docs/PRE_IMPLEMENTATION_AUDIT.md`、`docs/B0_FROZEN_SPEC.md`；
本阶段没有重复或回滚这些修复。

## 3. 参数

所有新增参数及默认值集中在 `src/soft_vofod_mid360/config/default.yaml`，分为：

- input/world frame、10 ms micro-batch；
- map geometry/evidence/maturity、VALID/NO_RETURN weights 和两个 guards；
- event free/background-distance gates；
- birth window/groups/time/speed/residual/anomaly/cap；
- KF Q/R/shape、GNN gate、target support、existence hysteresis/clutter/timeout；
- opportunity `p_ret/P_D cap/range/occlusion/sigma scale`；
- time-jump safety 和 map output period。

这些值是一个场景无关的开发配置，不标记为实测标定。后续只能在 calibration split 上统一
更新，不能按 S01/S03/S05 改参。

## 4. 公式对应

背景概率、event score、weighted CV birth、CV-KF、GNN cost、sigma opportunity、Bernoulli
existence 和 ray truncation 的完整公式、符号与函数映射见
`docs/SOFT_VOFOD_FORMULAS.md`。

## 5. 测试

新增 gtest 覆盖：

- unknown→free、unknown→candidate→stable、quarantine 禁止 promotion；
- CV `F/Q(dt)`；
- 1/2 event 不 birth、三组匀速 birth、hover birth、超速/不一致拒绝；
- 2×2/crossing/singleton Hungarian 和一一性；
- no-intersection、front return occlusion、NO_RETURN opportunity、sigma uncertainty、前方
  confirmed-track occlusion、单调性和 cap；
- `P_D=0` miss 不降 existence、high-`P_D` miss 下降、hit 上升；
- endpoint guard、hover 不入背景、VALID/NO_RETURN target truncation、confirm hysteresis 与
  hard-timeout reason。

ROS integration test 使用真实消息 ABI 和 ExactTime，先以 NO_RETURN 建立 free space，再发送
三个独立 hover return，验证 event 和 tentative trajectory birth。

本阶段局部结果：

```text
catkin build soft_vofod_mid360
  3/3 dependency packages succeeded

catkin run_tests soft_vofod_mid360 --no-status -j1
  30 tests, 0 errors, 0 failures, 0 skipped
```

最终全工作区结果：

```text
catkin build
  10/10 packages succeeded, no build warning/failure

catkin run_tests --no-status -j1
  10/10 packages succeeded

catkin_test_results build
  318 tests, 0 errors, 0 failures, 0 skipped
```

其中 Phase 0/1 原有 288 tests 全部保留，新包增加 30 tests。

## 6. 本历史阶段当时尚未完成的项目

本次没有实现或伪造：

- record-once/replay-many runner；
- HOTA/IDF1/GOSPA、TTFT、IDSW、map contamination、runtime p95；
- S01/S03/S05 Gazebo benchmark artifacts；
- CAL01/CAL02/CAL03 的真实参数标定；
- 完整 rolling collision scene 或真实 Mid360 flight validation。

这些项目后来已由 `soft_vofod_evaluation` 和 S01–S07 开发矩阵完成；本节只保留阶段边界，
最终声明以 `SOFT_VOFOD_EXPERIMENT_REPORT.md` 为准。

## 7. 失败场景、瓶颈与硬件下一步

强制保留的不可观/弱观测场景和实现边界见 `docs/KNOWN_LIMITATIONS.md`。主要性能风险是 dense
map 输出、有界 stable-neighbor 搜索和 anomaly buffer hypothesis 重评分。

真实 Mid360 接入前需要：

1. 完成两次断电周期的 spherical ray capability probe；
2. 用 CAL01 标定 `p_ret(range)`、range/pose covariance 和 clutter density；
3. 通过 pose covariance 或独立质量 topic 接入 TF/covariance gate；
4. 用 record-once bags 回放 B0/A3，同一输入冻结参数；
5. 再运行 S01/S03/S05 与 no-target negative controls。
