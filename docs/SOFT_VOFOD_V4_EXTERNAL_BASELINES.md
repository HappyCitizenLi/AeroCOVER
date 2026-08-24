# SOFT-VoFOD V4 外部 Baseline 状态与公平协议

## 1. 已有 baseline

BL0 是冻结的 VoFOD-Mid360 B0，固定提交 `4cc1fe7`，由
`tclv_evaluation/launch/b0_canonical.launch` 启动。它与 SOFT-VoFOD 共享 source bag、
`points_world`、`rays_checked`、observer pose、truth、计分区间和 replay rate。B0 内部 10 s background
initialization 在 cold-start 中必须计入 TTFT，不能剪掉。

## 2. 缺失 baseline

当前仓库没有可独立运行的：

- BL1 range-adaptive clustering + temporal consistency + IMM/Hungarian；
- BL2 multi-frame accumulated cloud detector + IMM；
- BL3 M-detector/dynamic-object style detector。

`lidar_tracker_mid360` 只是 B0 的 tracker，不是带独立 detector 的 BL1/BL2。SOFT 的 O/U/R/M
ablation 使用自身地图、packet 和 birth evidence，也不能改名为 external baseline。这样做会破坏算法独立性
和公平性，因此本轮没有制造“baseline”数字。

## 3. 接入契约

新增 baseline 必须只订阅 source 中所有算法都可用的信息，不得读 visibility truth、scenario events 或
仿真 model state。统一输出 world-frame track id/position/velocity/stamp，并由同一个 evaluator 计算 HOTA、
DetA、AssA、IDF1、IDSW、fragmentation、GOSPA、RMSE、TTFT 和 runtime。

每个 paired group 必须保存：source SHA-256、baseline source/config SHA-256、commit、seed、noise、输入握手、
覆盖率和 resource metrics。参数只能在 CAL split 上选择，最终 test matrix 固定后不得改动。

## 4. Phase 11 结论

Phase 11 **未完成**。恢复条件是实现并单测 BL1/BL2 detector，先过固定 source 的接口/覆盖率检查，再与
BL0 和冻结 SOFT-VoFOD 同时跑 N0/N1/N2×10 seeds。当前没有对外部 baseline 的统计优势 claim。
