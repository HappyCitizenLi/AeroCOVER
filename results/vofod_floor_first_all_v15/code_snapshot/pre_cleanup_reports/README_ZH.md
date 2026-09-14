# 第一层中央地图：其余七项复验

用户要求：将所有场景地图下界改为 −0.125 m，使地面 z=0 位于第一层中央。Mid-360 ready=0.0001、OS1 ready=0.15；其他检测与 tracker 参数不变。OPEN/MT 沿用现有 V2 配置，OFFICE/FOREST 沿用当前配置。

八份默认配置和生成器同步为 center.z=8.9375、size.z=18.125，体素 0.25 m。xy、名义上界 18 m、实际接受上界 18.375 m 及网格相位不变。

OPEN/Mid-360 直接链接复用 v14 完整结果，不再次回放，不改其 manifest。其余七项依次回放，结果存 SU710，本地 runs 下为逐项链接。文件系统不支持在 SU710 内创建链接，因此复用链接放在本地；没有复制大体积历史 bag。

原始输入沿用 v10 的 source.bag；Mid-360 0.25×、OS1 0.15×，新时间协议、1.5 m 匹配、MT 悬停遮挡 ignore、HOTA 关闭均不变。保留完整输出。相对于 v10 唯一配置变化为地图的两个 z 字段。

`replay.py validate --root results/vofod_floor_first_all_v15` 校验配置；`replay.py replay --root results/vofod_floor_first_all_v15` 顺序回放并生成对照报告，自动跳过已完成项。报告区分复用与新跑，验证相同源输入、算法二进制、评估器及 tracker 配置。新七项源代码树摘要必须一致；v14 因默认 YAML 更新前运行，代码树摘要不同，但二进制与实际生效配置必须一致。

## 完成结果

COMPLETE：七项新回放、评分全部完成，OPEN/Mid-360 复用 v14，合计八项。完整指标与 runtime 见 [PER_SEQUENCE_ZH.md](PER_SEQUENCE_ZH.md)。输出／计时／完成消息／真值覆盖率均为 100%，每个评分时刻仅选一次，LOS ignore 数量与 v10 一致，metrics 摘要校验通过。完整输出保留，复用项的原 manifest 未改。

默认配置已按要求保留第一层中央地图；Mid-360 ready=0.0001，OS1 ready=0.15。没有采用 v11/v12 的试验比例，也未因为某项退化临时回退。

相比 v10：

- OPEN/OS1 主要指标不变；OPEN/Mid-360 的复用结果 FP 4236→6，TP/FN 与 IDSW/Frag 不变。
- MT/OS1 FP 57→0，MT/Mid-360 FP 6036→106；两者 TP/FN、IDSW/Frag 均不变。
- OFFICE/Mid-360 FP 269→127，TP/FN 不变。OFFICE/OS1 仍有 1425 个 FN、recall 仅 4.68%，不能因 FP=0 视为成功。
- FOREST/Mid-360 FP 6549→3966，TP/FN、IDSW/Frag 不变。FOREST/OS1 FP 2764→154，但 FN 299→380、recall 84.54%→80.35%，IDSW/Frag 7/7→5/5；这是伴随召回退化的结果，不是无损改进。

## 地面附近 FP 审计

复用既有 CSV 审计脚本生成 [ERROR_AUDIT.json](ERROR_AUDIT.json)，FP 总数与评估器逐项一致，已排除 MT 遮挡 ignore。下表是 `abs(z)<0.5 m` 的预测计数，表示地面附近，不等同于逐点证明物理误检来源。

| 场景 | 传感器 | v10 地面附近 FP | 第一层地面附近 FP | 第一层总 FP |
|---|---|---:|---:|---:|
| OPEN | OS1 | 0 | 0 | 0 |
| OPEN | Mid-360（复用） | 4230 | 0 | 6 |
| MT | OS1 | 57 | 0 | 0 |
| MT | Mid-360 | 5960 | 29 | 106 |
| OFFICE | OS1 | 0 | 0 | 0 |
| OFFICE | Mid-360 | 146 | 31 | 127 |
| FOREST | OS1 | 2353 | 2 | 154 |
| FOREST | Mid-360 | 2824 | 68 | 3966 |

MT/Mid-360 剩余 106 个 FP 均属于曾匹配真实目标的轨迹，不能仅按低高度认定都是新生地面误检。FOREST/Mid-360 剩余 FP 主要不在该地面高度带内，需要另行检查非地面背景／轨迹问题，不能靠此项地图修改解释全部来源。

## ready 与剩余 FN

| 场景 | 传感器 | ready 帧/评分帧 |
|---|---|---|
| OPEN | OS1 | 989/1022 |
| OPEN | Mid-360（复用） | 844/917 |
| MT | OS1 | 412/526 |
| MT | Mid-360 | 442/500 |
| OFFICE | OS1 | 941/1495 |
| OFFICE | Mid-360 | 1445/1476 |
| FOREST | OS1 | 1896/1934 |
| FOREST | Mid-360 | 1654/1795 |

OFFICE/OS1 首次 ready 仍约 87.846 s，首次匹配前 FN=545、首次匹配后 FN=880；整段仅 46 帧产生检测。降低地图下界层号未解决其主要漏检。

FOREST/OS1 首次匹配前 FN 仍为 39，首次匹配后 FN 从 v10 的 260 增至 341，新增 81 个 FN 均发生于首次匹配之后，不能归因于启动更晚。

第一层利用既有 [地图边界连通判定](../../src/vofod_mid360/src/voxel_map.cpp:618)，并非新增通用地面分割算法。它依赖这些场景中地面 z=0 的已知几何，不应直接推广为起伏地形下的保证。单次异步回放也不能分离所有调度波动；本报告只陈述此次受控配置对照的观测结果。
