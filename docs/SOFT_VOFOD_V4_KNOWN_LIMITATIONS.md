# SOFT-VoFOD V4 已知限制

1. **Final 未冻结。** Opportunity、S05、runtime 和 external baseline gates 尚未闭环。
2. **仿真 opportunity 不能外推真实 Mid360。** 30 m+ return rate 仍是 fallback；没有真实 angle bins。
3. **Opportunity calibration 与 tracking safety 分离。** O3 reliability 最好，但 miss update 仍显著增加 FN。
4. **S08B 尚差严格门限。** sequential TTFT 5.2 s，未达到 `<5 s`。
5. **S05 未接近目标。** R0/R1 FN 都为176，两阶段只增加0.20 s latency。
6. **OCCLUDED 无实证 recall。** S05 truth occlusion recall 为0，不能作为核心贡献。
7. **LOS 只有单 seed 小收益。** 没有 range×count 的真实拟合，也没有 R5 统计显著性。
8. **真正 cold-start 有性能风险。** 初始 smoke 的 p95 为106.8–174.2 ms；CS03 最终修正版为
   120.7 ms，均高于100 ms门限。map growth/voxel publication 尚未优化。
9. **Cold-start continuity 不足。** CS05 原始 smoke 的 post-transition recall 为0.374，虽保留同一 ID，
   但不构成稳定维护证据。
10. **Cold-start smoke 不是 final matrix。** 五场景分属不同修正 commits；仅 CS03 在当前 provenance
    implementation 上复验。
11. **静止未知目标不可辨识。** 无先验 free history、外观或运动时，它与背景信息论等价；系统只能
    unresolved/assimilation。
12. **Map boundary guard 限制 operational envelope。** 未知目标紧贴 map 边界时不会 birth；部署必须
    留足 map margin 或移动 map window。
13. **没有 BL1/BL2。** 不能声称优于现代 external baseline。
14. **没有真实数据与飞行。** HW recorder 已准备，但 HW-CAL、real replay、multi-UAV flight 均未执行。
15. **资源结果依赖当前桌面环境。** 仿真 replay p95 包含 ROS/Gazebo/recording contention，不等于机载
    CPU，但 final gate 仍按观测值判失败。
16. **大体积 bags 不是永久归档。** 部分 bags 为节省磁盘已删除；保留 manifests/metrics，重现 paired
    比较需要重新录制新的共同 source。

因此当前证据只支持“有前景的仿真算法与诚实的负结果”，不支持 T-RO/IJRR 定稿或真实多机闭环飞行。
