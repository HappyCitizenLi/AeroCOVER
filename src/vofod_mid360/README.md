# 当前 VoFOD checked-ray 适配

受控移植自CTU-MRS VoFOD commit 7da9f33a878a586588f6a626b75cfeacac7824f7。来源、许可证和再分发边界见 [UPSTREAM.md](UPSTREAM.md)。这不是未经修改的上游二进制，也不能由文件名original推断所有参数等同论文。

当前主实验只有 VoFOD-Mid360 / VoFOD-OS1，使用 config/four_scene_suite 中的八份场景配置：

- 地图体素0.25 m，xy中心(16,15)、尺寸(88,78) m；z中心8.9375、尺寸18.125 m，下界−0.125 m，地面位于第一层中央。
- Mid ready=0.0001、min_sure_voxels=1；OS1 ready=0.15、min_sure_voxels=24。
- OPEN/MT：聚类距离1.5 m、最大簇对角线3 m、background_distance=1.5 m。
- OFFICE/FOREST：聚类距离0.5 m、最大簇对角线1.5 m、background_distance=0.3 m。
- 最少聚类点数Mid=1、OS1=2；无回波可信距离20 m。OS1由sensor overlay覆盖为1024×128=131072 rays，固定body mask为24849个pattern。

输入为同步world点云与CheckedRayBundle。仅native_rangefinder作为种子；point-update-before-classification、异步ray worker和独立0.1 s背景清理保留。没有STInit/PersistentInit或人工10scan冷启动。ready动态判断占据体素计数与确信背景簇，不是永久置位。

config/b0_mid360_canonical.yaml是低层launch/合约测试的基础配置，不代表当前四场景主配置。b0.launch按场景配置→method→raycast→sensor叠加；config/vofod_original.yaml保留为现有launch/manifest兼容名称。

浮空DFS、地图边界直接连通和unknown写回等风险路径源自上游；本地清理写回去重等差异尚未解决，不能宣称所有FP与本地修改无关。[实现差异和实验分析](../../docs/AEROCOVER_VOFOD_CURRENT_FULL_REPORT_ZH.md)

运行时通过 tclv_evaluation 的 b0_canonical.launch / ouster_original.launch，显式传入对应场景b0_config；外部tracker配置由主runner按场景选择。

```bash
source /opt/ros/noetic/setup.bash
source /home/uav/lyk/devel/setup.bash
catkin build vofod_mid360 --no-deps
catkin run_tests vofod_mid360 --no-deps
```
