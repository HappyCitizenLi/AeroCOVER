# tclv_evaluation

可见性真值几何与两种VoFOD管线的薄组合层。真值只用于仿真/审计/评分，不输入检测器。

- b0_canonical.launch：Mid-360 checked-ray输入。
- ouster_original.launch：OS1 1024×128 snapshot adapter + checked-ray检测器 + 外部tracker。

文件名b0/original为当前接口兼容名称，不等于另一套历史实验方法。主实验应通过 run_benchmark.py 显式传入四场景配置；OPEN/MT的tracking_open_v2.yaml仍被使用，不应按v2文件名删除。

[当前配置与回放](../soft_vofod_evaluation/README.md) · [完整报告](../../docs/AEROCOVER_VOFOD_CURRENT_FULL_REPORT_ZH.md)
