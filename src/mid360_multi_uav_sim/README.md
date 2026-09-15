# 当前四场景仿真

| 场景 | World | 目标数 | 当前轨迹 |
|---|---|---:|---|
| OPEN | PW_open | 2 | observer直线往复；上升圆与高速进入的圆角五尖星，目标距离限制19 m |
| MT | PW_mt | 3 | 两个悬停目标距柱面机体净空1.5 m，一个运动目标绕柱；15 m高柱 |
| OFFICE | PW_office | 1 | 多走廊/转角贴墙追逐，转角允许2–4 m近距离 |
| FOREST | PW_forest_seed0 | 1 | 从西到东穿越树林并近障碍飞行 |

使用MRS/PX4/MAVROS飞行栈。OS1为1024×128、360°GPU快照；Mid-360为滚动场景采样。rangefinder_follower.py刚性跟随实际observer，sensor_msgs/Range来自Gazebo传感器，不从高度伪造距离。

```bash
source /opt/ros/noetic/setup.bash
source /home/uav/lyk/devel/setup.bash
/usr/bin/python3 /home/uav/lyk/src/mid360_multi_uav_sim/scripts/view_scenario.py OPEN --sensor paired --rviz
```

OPEN可替换为MT/OFFICE/FOREST，每次只启动一个场景。GUI需要图形显示环境。

当前离线构造入口为 scripts/design_four_scenes.py；current_scene_design.py保存MT及追逐路径的共用构造，design_open_19m.py构造OPEN，config/routes/*.npy是当前路径依赖，不是旧实验垃圾。构造函数重组前后的四场景输出做完全一致性检查，现有四个YAML未改写。

记录/回放见 [评估说明](../soft_vofod_evaluation/README.md)。已有实验的精确输入使用 [保留源bag](../../results/retained_experiments/sources)，不要把重新飞行当成相同输入。

几何/速度/FOV实测范围和未验证的实际加速度边界见 [完整报告](../../docs/AEROCOVER_VOFOD_CURRENT_FULL_REPORT_ZH.md)。当前主评估只对MT悬停目标应用柱遮挡ignore。
