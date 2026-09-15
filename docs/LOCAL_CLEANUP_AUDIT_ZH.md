# 当前版本本地文件清理审计

## 范围与原则

用户要求清除过时描述和历史算法残留，同时保护当前算法与实验。清理前工作树干净，Git基线为 `326a4b1`。本次不提交、不重写Git历史；跟踪文件可从该提交按路径恢复。

当前主场景为 OPEN、MT、OFFICE、FOREST，主方法为 AeroCOVER-Mid360、AeroCOVER-OS1、VoFOD-Mid360、VoFOD-OS1。保留当前AeroCOVER机制消融/回归配置，但不再提供70项/20项历史矩阵或历史Original/Adapted方法入口。

## 实际清理

- 删除35个已跟踪历史文件：旧总报告/提示词、旧矩阵脚本、单独的旧协议评估副本、未编译且无调用的PersistentStructure、旧八场景YAML、过时维护阈值覆盖、未使用的V2/Adapted/2048列VoFOD配置。
- 提取当前使用的MT、OFFICE和FOREST构造函数至 `current_scene_design.py`，删除旧v2/v3/v4独立生成器及其未使用轨迹分支；当前四场景生成结果逐项摘要完全相同。
- `run_four_scene_suite.py` 仍是当前16项矩阵工具，只内联其原共享命令执行函数，解除对旧70项脚本的依赖。录制、mask更新、回放、源审计和中断处理逻辑保留。
- `run_benchmark.py` 移除退休方法别名/配置映射和旧场景注册；四种生产方法的启动命令与清理前相同。默认HOTA说明改为关闭，不改评估计算文件。
- 两个ROS合约测试原先引用退休Adapted文件，现改为参数完全等同的 `test/native_background.yaml`，没有删除这些安全测试。
- 更新根README及评估、场景、AeroCOVER、VoFOD、tracker、预处理器和仿真插件说明；主参数为1.5 m匹配/维护门、OS1 1024×128、第一层地图和当前ready比例。
- 当前YAML只修正少量注释，数值逐项核对不变。历史配置SHA仍以保留实验快照为准。
- 删除2个已失效的catkin脚本转发入口与8个无源文件的Python缓存。生成文件可重新构建，旧源文件可从Git恢复。

## 保护与验证

- **86个生产源码/接口/launch文件及评估器字节内容未变**；移除的PersistentStructure未被生产CMake编译，也没有当前调用者。
- 当前存续AeroCOVER、tracker、VoFOD场景、预处理器配置的解析值不变；四个当前场景YAML和原始输入未改写。
- 四场景生成结果SHA（规范化JSON）：
  - OPEN: `b77a5637ec28df1bc2f788d83a146b0da14bf53308d5a3ba8ff042ce528cc30b`
  - MT: `ce5c57c8d90da6264592b4808499f9adc142525664b8c8f7ffee852023130913`
  - OFFICE: `c98815bdffa67bd35739fed535d725987c87a00def06ef5b78d91f02f89afb9e`
  - FOREST: `f07d8109f6a33fa3d702418054efb058d0d631279bd50d53859fbe850c29a0da`
- 当前四种生产算法的 `algorithm_command(...)` 输出与清理前相同。
- 2个Python/场景包重新配置构建成功；四种生产launch使用 `roslaunch --files` 展开成功；所有工作区内静态launch/test文件引用检查通过。
- 场景测试10项；runner测试原9项及新增转发入口测试1项，共10项；评估测试22项通过。
- C++测试：AeroCOVER核心35、ST背景7、VoFOD核心23、tracker核心16、TF缓存7、source-mode2项通过。
- ROS合约：native_init、b0_soft_update、production_config三项通过。
- 保留实验的原manifest、metrics、CSV等运行文件和四个源目录通过清理索引的完整性检查；未重跑实验、未重算指标。
- 环境原本缺少rosrun，不属于本次删除；文档使用Python/roslaunch。新四场景catkin转发入口曾暴露模块导入路径问题，已修复并以独立relay目录测试覆盖。

这些检查约束当前计算、输入、参数及保留结果不变；并不宣称异步算法的新回放会逐帧复现相同调度或runtime。

## 明确保留的非“垃圾”

- `results/**/code_snapshot`、原manifest、冻结配置和来源/许可证记录：当前保留实验的复现证据，不执行批量替换。
- `UPSTREAM.md` / 上游CHANGELOG / LICENSE及供应商驱动、模型、LUT资源：来源或当前硬件/仿真依赖。历史导入叙述不是当前算法入口；已增加当前状态说明。
- `tracking_open_v2.yaml`：当前OPEN/MT仍实际加载，不能按v2名字删除。
- `mid360_ray_preprocessor/config/ouster_os1_128.yaml`：虽然文件名含128，内容是当前1024×128，已保留；删的是VoFOD目录里的旧2048列文件。
- `config/routes/*.npy`、场景契约字段 `four_scene_v1`、B0/topic/package名字：当前几何依赖或数据接口身份，不做破坏性更名。
- `b0_mid360_canonical.yaml`：低层launch/ROS合约测试仍引用，不是当前四场景主配置。未为“统一名字”改变其参数。
- 评估器中的旧方法名称/字段解码与HOTA可选函数：当前回归和保留数据格式兼容使用。主评分HOTA关闭，评估器SHA保持 `a2988629f751b35601652feddf8b666dfada0ea4c24f61787463541b81519758`。
- 已编译的PCD loader等兼容编译单元、生产C++内部ablation/reference分支：没有在此次清理中进行ABI/算法重构。不能只凭暂未使用的参数或历史命名判定可安全移除。

## 删除的跟踪文件

- `docs/CURRENT_AEROCOVER_VOFOD_IMPLEMENTATION_RESULTS_AND_ANALYSIS.md`
- `docs/AEROCOVER_ADAPTIVE_SHELL_VOFOD_PLAN_ZH.md`
- `docs/AeroCOVER_RAL_Minimal_STInit_NativeInit_CheckedRay_Codex_Prompt.md`
- `src/soft_vofod_evaluation/scripts/run_paper_minimal.py`
- `src/soft_vofod_evaluation/scripts/run_twenty_scene_suite.py`
- `src/soft_vofod_evaluation/scripts/run_state_time_pair.py`
- `src/soft_vofod_evaluation/scripts/run_map_floor_comparison.py`
- `src/soft_vofod_evaluation/scripts/evaluate_bag_v8_reference.py`
- `src/vofod_mid360/include/vofod/persistent_structure.h`
- `src/vofod_mid360/src/persistent_structure.cpp`
- `src/vofod_mid360/config/b0_mid360_adapted.yaml`
- `src/vofod_mid360/config/vofod_mid360_adapted.yaml`
- `src/vofod_mid360/config/vofod_mid360_native_init.yaml`
- `src/vofod_mid360/config/sensors/ouster_os1_128.yaml`
- `src/vofod_mid360/config/four_scene_suite/OFFICE_mid360_v2.yaml`
- `src/vofod_mid360/config/four_scene_suite/OFFICE_os1_v2.yaml`
- `src/vofod_mid360/config/four_scene_suite/FOREST_mid360_v2.yaml`
- `src/vofod_mid360/config/four_scene_suite/FOREST_os1_v2.yaml`
- `src/aerocover_mid360/config/maintenance_s28.yaml`
- `src/aerocover_mid360/config/maintenance_s29.yaml`
- `src/aerocover_mid360/config/maintenance_s30.yaml`
- `src/aerocover_mid360/config/maintenance_s31.yaml`
- `src/aerocover_mid360/config/maintenance_s32.yaml`
- `src/mid360_multi_uav_sim/config/benchmarks/S1_near.yaml`
- `src/mid360_multi_uav_sim/config/benchmarks/S1_far.yaml`
- `src/mid360_multi_uav_sim/config/benchmarks/P01.yaml`
- `src/mid360_multi_uav_sim/config/benchmarks/P02.yaml`
- `src/mid360_multi_uav_sim/config/benchmarks/S2_new.yaml`
- `src/mid360_multi_uav_sim/config/benchmarks/S3_new.yaml`
- `src/mid360_multi_uav_sim/config/benchmarks/M1.yaml`
- `src/mid360_multi_uav_sim/config/benchmarks/M2.yaml`
- `src/mid360_multi_uav_sim/scripts/design_four_scenes_v2.py`
- `src/mid360_multi_uav_sim/scripts/design_four_scenes_v3.py`
- `src/mid360_multi_uav_sim/scripts/design_four_scenes_v4.py`
- `src/mid360_multi_uav_sim/config/mrs/multi_target_config.yaml`

## 删除的生成文件

- `devel/lib/soft_vofod_evaluation/run_paper_minimal.py`
- `devel/.private/soft_vofod_evaluation/lib/soft_vofod_evaluation/run_paper_minimal.py`
- `src/soft_vofod_evaluation/scripts/__pycache__/run_paper_minimal.cpython-38.pyc`
- `src/soft_vofod_evaluation/scripts/__pycache__/run_state_time_pair.cpython-38.pyc`
- `src/soft_vofod_evaluation/scripts/__pycache__/run_map_floor_comparison.cpython-38.pyc`
- `src/soft_vofod_evaluation/scripts/__pycache__/run_twenty_scene_suite.cpython-38.pyc`
- `src/mid360_multi_uav_sim/scripts/__pycache__/design_four_scenes_v4.cpython-38.pyc`
- `src/mid360_multi_uav_sim/scripts/__pycache__/ground_rangefinder.cpython-38.pyc`
- `src/mid360_multi_uav_sim/scripts/__pycache__/design_four_scenes_v3.cpython-38.pyc`
- `src/mid360_multi_uav_sim/scripts/__pycache__/design_four_scenes_v2.cpython-38.pyc`

当前导航：[根README](../README.md)、[完整报告](AEROCOVER_VOFOD_CURRENT_FULL_REPORT_ZH.md)、[保留实验](../results/retained_experiments/INDEX_ZH.md)。
