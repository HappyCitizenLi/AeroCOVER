# OFFICE/VoFOD-OS1 ready=0.01 实验

只新跑 OFFICE/OS1 一项。沿用当前 v15 第一层中央地图（下界 −0.125 m），仅将 background.sufficient_points_ratio 从 0.15 改为 0.01。检测、tracker、地图、传感器配置不变，默认配置不改。

Nx=353、Ny=313，严格大于 Nx×Ny×比例的条件对应至少 1105 个占据体素（原为 16574）；仍要求存在确信背景簇，min_sure_voxels=24 不变。

复用原 source.bag，0.15×回放，不重新采集。使用新时间协议和 1.5 m 匹配，HOTA 关闭；完整输出保存至 SU710。`replay.py --validate` 检查唯一参数差异，`replay.py` 运行并生成 PER_SEQUENCE_ZH.md。

## 完成结果

1/1 完成，完整指标及 runtime 见 [PER_SEQUENCE_ZH.md](PER_SEQUENCE_ZH.md)。1495/1495 评分帧，输出／计时／完成消息／真值覆盖率均为 100%；输入、算法、评估器和 tracker 摘要、同刻去重、metrics 摘要校验通过。默认 OFFICE/OS1 ready 仍为 0.15，未自动推广试验参数。

同地图对比 v15：TP/FP/FN 从 70/0/1425 变为 588/49/907，recall 4.68%→39.33%，IDSW/Frag 1/1→7/5。平均 runtime 44.068→44.277 ms，p95 55.095→56.541 ms。

首次 ready 从约 87.846 提前至 35.748 s，ready 帧数从 941 增至 1477（总1495帧）。[ERROR_AUDIT.json](ERROR_AUDIT.json) 显示首次匹配前 FN 545→19，但首次匹配后 FN 880→888。改善主要来自启动阶段，剩余 907 个 FN 中有 888 个在首次匹配之后；不能将主要后续损失归咎于 ready 比例。

整段有检测的帧数为 563，仍明显少于 1477 个 ready 帧；只有 3 个诊断帧是在存在确信背景簇时仍未 ready。49 个 FP 均来自曾匹配真实目标的轨迹，其中 6 个位于 `abs(z)<0.5 m`；不能单凭高度把它们全部归为新生地面误检。

本次只改比例，没有修改算法代码，也没有同时处理此前核查出的上游／本地清理写回差异。两次异步回放不是确定性重复，不能将所有微小差异都归因于参数。
