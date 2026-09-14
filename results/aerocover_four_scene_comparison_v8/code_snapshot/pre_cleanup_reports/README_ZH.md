# 四场景20组实验：共享body mask、OS1完整帧、MT LOS ignore

## 完成状态与主结果

全部20项已完成并核验：OPEN4、MT4、OFFICE6、FOREST6；20个清单均COMPLETE，输入真值／跟踪输出／计时／完成事件四项覆盖率均100%，12项VoFOD均ready。各方法代码与二进制指纹一致，配置、输入清单和指标哈希通过检查，HOTA未计算。正式主表见 [PER_SEQUENCE_ZH.md](PER_SEQUENCE_ZH.md)。

- OPEN与FOREST的AeroCOVER-OS1源帧缺口消失：1022/1022、1934/1934；两场景均FP/FN/IDSW/Frag全0，保留的1 m敏感性评分也全0。FOREST原26帧自机FP不再出现。因此未重录这两场景。
- 新MT的运动目标uav4在两种AeroCOVER传感器下都100% recall、IDSW/Frag=0/0。LOS ignore后场景整体仍有悬停目标的可见时段漏检及中断：Mid-360为FP8/FN149、IDSW/Frag4/4，OS1为FP0/FN41、IDSW/Frag3/3，不宣称全场零错误。
- 同传感器各算法的MT忽略真值数一致：Mid-360为241，OS1为255。忽略预测数可因算法输出不同而不同，逐项保存在metrics.json与los_ignored_timeseries.csv中。
- OFFICE的VoFOD-V2两种传感器均TP=0，不可把其IDSW/Frag=0/0解读为稳定跟踪。FOREST中V2虽减少FP，却降低recall：Mid-360从0.908降至0.723，OS1从0.804降至0.419；只说明本批固定输入下的结果，不据此继续调参。
- VoFOD-OS1仍有自身的跟踪中断（如FOREST当前4次Frag、V2为7次），均不是输入／输出帧缺失造成。已消除的是此前讨论的AeroCOVER-OS1源帧缺口及其Frag，不能宣称所有算法所有Frag都消失。

成功评分后的可重建中间output.bag按既有流程移除；原始输入、逐帧CSV、指标、运行清单与日志均保留，旧v7结果不覆盖。代码与配置快照位于code_snapshot、config_snapshot。

## 用户确认与实现范围

- AeroCOVER-OS1独立snapshot adapter加载同一份VoFOD sensor body mask，保留131072条ray及原始索引；屏蔽方向标为INVALID_RANGE/不可用，不作为NO_RETURN或自由空间证据，不输出对应点。native原始消息保留，由VoFOD按同一mask处理。
- 转换输入与三个输出队列由1提高到32，保留突发扫描，不伪造时间戳，不按结果删帧。8帧/4 ms突发ROS测试两次通过，检查完整ray数、不可用状态及输出点数；构建无错误。未改变OS1采集端节奏，因此输入仍可能含4/8 ms间隔，不能宣称严格等频10 Hz。
- 共享mask保留v7已有标定，加入新四源自机证据的4个方向：1035、30535、31407、32891，未取消历史已屏蔽方向。最终24849个屏蔽方向，源绑定及mask哈希见body_mask_manifest.json。仅使用0.1–0.6 m近场self-return证据，不以目标跟踪指标拟合mask。
- HOTA继续不计算；tracker共享关联门限及主评分范围1.5 m不变，其余AeroCOVER检测／跟踪参数未调。

## 新MT与实飞

uav4高3.9 m；uav2/uav3位于柱体x正负两侧，中心半径为1.2+0.43+1.5=3.13 m，即机体外缘到柱面净空1.5 m，高度分别保持2.7/3.8 m。单运动目标沿原半径16 m、相位-0.15 rad轨迹绕行。

直接只降uav4高度的首版规划检查未通过：高速observer倾斜时，运动目标俯仰约-6.97°，因此在起飞前被拦截，没有生成飞行bag。配套将observer高度改为`3.1-0.25*sin(pi*t/52)^2`，起始/结束3.1 m，高速段最低2.85 m；水平轨迹、时长和速度任务不变，不放宽FOV要求。

新录制seed=12001、N03、paired，实飞审计PASS：净空1.4996 m，扣采样余量后1.4216 m，目标距离4.1373–15.3321 m，俯仰-4.7101°～19.4969°，水平峰值7.7995 m/s，无未允许静态LOS遮挡，无目标互遮挡，真值最大间隔8 ms。OS1原始575帧，每帧131072 rays，时间戳和几何检查通过。

## LOS评分（用户明确确认）

MT仅uav2/uav3在非LOS时ignore：不计TP/FN。先保护可见目标的匹配，再将剩余预测与遮挡目标按1.5 m一对一匹配，被匹配预测不计FP；重复预测和不匹配预测仍计FP，不无限忽略附近输出。

LOS由评分状态时刻的记录姿态插值、实际sensor安装高度及静态world碰撞几何计算sensor-origin→target-center线段，不由是否有返回点推断；同一传感器的两种算法／配置使用相同规则。可见区间之间若恢复为不同ID仍可计IDSW，遮挡本身不作为FN段。每运行导出los_ignored_timeseries.csv和los_scoring计数供审计。

## VoFOD配置与20项矩阵

OPEN、MT：每传感器AeroCOVER与VoFOD各一次，共4+4项。VoFOD均为已确认OPEN V2口径：detector聚类距离1.5 m、最大簇3 m、d_close=1.5 m；Mid-360最少点数保留1，OS1保留2。tracker使用tracking_open_v2.yaml：leaf=.5 m、聚类1 m、最大簇1 m、背景余量1 m、半径2.5–5 m。

OFFICE、FOREST：每传感器AeroCOVER、VoFOD当前、VoFOD-V2各一次，共6+6项。当前配置不覆盖；V2 detector另存`*_v2.yaml`，并显式加载同一V2 tracker override。V2也沿用已确认的Mid-360最少点数1。地图、背景就绪比例、min_sure_voxels、tracker Q/P0/R与第六根半径计算均不额外改动。

矩阵代码run_twenty_scene_suite.py已检查20个唯一运行目录，分布OPEN4/MT4/OFFICE6/FOREST6。评估18项、场景13项、运行器13项测试通过。

## 输入、执行与计时

MT新录；OPEN/OFFICE/FOREST先复用v7来源。OPEN/FOREST不是采集漏掉了已知缺口帧，先验证转换链路修复；只有修复后仍无法满足完整输出/零Frag，才进一步诊断或重录。旧输入、历史报告保留。

所有新输出保存至`/media/uav/SU710/aerocover_twenty_v8/runs`，本目录runs为链接。AeroCOVER-Mid360 1.0×、AeroCOVER-OS1与VoFOD-Mid360 0.25×、VoFOD-OS1 0.15×，均顺序运行。runtime是实际节点计时，不乘回放倍率；AeroCOVER与VoFOD的计时边界不同，不能当作同边界端到端耗时。

OPEN/FOREST两项完整AeroCOVER-OS1检查先行通过，随后作为20项中的正式结果复用，其余18项也已完成，没有重复计数。配置和实现全程冻结，未按结果继续调参。历史v7与本批次有mask、MT几何及LOS协议差异，不是单变量门限消融。
