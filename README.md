# AeroCOVER / VoFOD 当前工作区

当前场景为 OPEN、MT、OFFICE、FOREST；传感器为 Mid-360 和 OS1-128（1024列）。主方法是 AeroCOVER、VoFOD，各自有传感器专用入口。

- [完整框架、实现、参数与实验报告](docs/AEROCOVER_VOFOD_CURRENT_FULL_REPORT_ZH.md)
- [保留实验与源数据](results/retained_experiments/INDEX_ZH.md)
- [回放与评分](src/soft_vofod_evaluation/README.md)
- [场景与GUI](src/mid360_multi_uav_sim/README.md)

运行前加载 /opt/ros/noetic/setup.bash 与本工作区 devel/setup.bash。当前主评分匹配距离1.5 m，HOTA关闭。VoFOD主地图地面在第一层中央，下界−0.125 m；Mid ready=0.0001、OS1 ready=0.15。V13–V16的独立试验配置保留在结果目录，不自动覆盖默认值。

来源/许可证、保留实验的代码快照与原始manifest是复现证据，不是当前运行入口；不要对它们执行关键词批量替换。
