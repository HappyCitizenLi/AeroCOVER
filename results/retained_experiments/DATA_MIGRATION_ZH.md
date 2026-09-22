# 实验数据迁移至 DATA

日期：2026-09-21。

将以下 7 个目录从 `/media/uav/SU710/<目录名>` 迁移至 `/data/lyk/<目录名>`：

- `aerocover_four_scene_v4_flow_20260913`
- `aerocover_open_19m_strict_20260913`
- `aerocover_twenty_v8`
- `vofod_open_floor_second_v13`
- `vofod_open_floor_first_v14`
- `vofod_floor_first_all_v15`
- `vofod_office_os1_ready001_v16`

共 284 个普通文件，37,804,902,092 字节（约 35.21 GiB）。使用 rsync 保留文件修改时间复制；逐目录执行 `rsync -rtcin --delete` **只读预检**，7 个目录均无内容或结构差异。删除前再次核对全部文件数量、大小、纳秒修改时间及目录差异，之后才删除 SU710 的对应原目录。

工作区 `results` 下 36 个符号链接已直接指向 DATA，并更新 `retained_index.json` 的实际数据路径。原索引副本保存在 `/data/lyk/retained_index_before_migration.json`。原始运行 manifest、指标、日志、配置和代码快照内容没有修改；其中 SU710 路径仅作为历史出处，当前位置使用新索引或工作区 `sources` / `runs` 链接。

切换链接及删除原件后，`python3 results/retained_experiments/cleanup.py verify` 均通过：18 次独立回放（19 条逻辑记录）和 4 个源数据目录保持完整。未重新运行或评分实验。

SU710 上“浦江实验”及 3 个演示文稿保持不动。已删除的原目录没有回收站副本，但完整数据保存在 DATA，可按上述映射复制回去。历史清理记录 `deletion_*.json` 不改写，`cleanup.py` 的旧清理计划也没有执行。
