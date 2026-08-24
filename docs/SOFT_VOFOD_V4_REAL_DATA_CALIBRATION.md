# SOFT-VoFOD V4 真实数据标定协议

## 1. 当前状态

工作区没有真实 Mid360 bag、PCAP/LVX、target RTK/mocap/UWB truth，也没有完成两次断电周期的 exact
spherical-ray capability 验证。因此下列真实量均为 **未测量**：

- range×azimuth×elevation×angular-coverage 的 `p_return`；
- grazing/no-return reliability；
- 真实 packet singleton/spread/surface bias；
- 真实 LOS covariance 和 opportunity Brier/NLL/ECE。

仿真的 30 m+ `p_return=0.5` 只是 fallback，不是论文最终参数。

## 2. 已准备的 record-only 入口

`mid360_spherical_capability_probe/launch/hw_record_only.launch` 只启动硬件 spherical pipeline、
capability probe 和 rosbag recorder，不启动 detector/tracker。它记录：

```text
points_raw, rays_raw, scan_identity,
points_world, rays_checked,
observer pose, target truth, time-sync diagnostics,
ray source mode, diagnostics, tf, tf_static
```

HW-CAL01/02 无目标时可以缺 target truth；HW-CAL03–05 缺 truth 即无效。每个 bag 必须附场景编号、
距离/角度、时钟源、外参版本、天气/材质、operator 和 SHA-256 manifest。

## 3. 必采场景

- HW-CAL01：静止 observer + 静态城市背景；
- HW-CAL02：移动 observer + 无目标；
- HW-CAL03：单 UAV 5/10/20/30/40 m range sweep；
- HW-CAL04：单 UAV hover；
- HW-CAL05：单 UAV near wall/trees/clutter。

真实数据应只用于拟合 range×coarse-angle return table、free reliability 和 LOS bins；冻结后再在独立
real test bags 上评价，禁止用同一 bag 调参与报告。

## 4. 阻塞条件

数据采集需要物理 Mid360、场地、truth system、时间同步和飞行安全授权，超出本地源码修改权限。本轮
只完成可执行 recorder 与协议，未生成任何虚假 HW/real 指标。
