# 当前 Mid-360 / Ouster 仿真插件分支

用于本工作区MRS X500四场景仿真，构建包为livox_laser_simulation。来源与授权记录见 [UPSTREAM.md](UPSTREAM.md) 和 [Ouster来源](livox_laser_simulation/OUSTER_UPSTREAM.md)。上游代码/传感器资源是运行依赖，不作为历史算法版本删除。

当前Mid-360使用rolling_scene逐ray时间/几何；Ouster使用Gazebo GPU的1024×128、360°快照输出（t字段为0）。两者由mid360_multi_uav_sim/benchmark.launch和当前MRS模型参数启动，不使用旧单传感器demo作为主实验。

[四场景与GUI](../mid360_multi_uav_sim/README.md) · [输入合约](../mid360_ray_preprocessor/README.md)

```bash
source /opt/ros/noetic/setup.bash
source /home/uav/lyk/devel/setup.bash
catkin build livox_laser_simulation --no-deps
```
