# SOFT-VoFOD V3 Certified Free

## 1. V2 根因

V2 的 `CONFIDENT_FREE` 同时表示“某次 ray 看起来穿过”和“足以触发 target violation 的历史自由
空间”。同一 epoch 内大量同视角 rays 虽经饱和，仍可能一次达到概率/置信阈值；状态没有独立 epoch
数、持续时间、VALID_RETURN-backed 证据或 surface guard。grazing traversal 与 endpoint voxel hopping
因此会把背景边缘认证成 free，解释了 static recall 增加时 false-free 反而升高。

## 2. V3 语义

`OBSERVED_FREE` 只表示近期有可靠 traversal，可用于空间完整度和 background 辅助推断，不产生强
target event。`CERTIFIED_FREE` 才是 detector-grade free，canonical 条件为：

```text
free probability/confidence pass
AND independent free epochs >= 3
AND first-to-last epoch duration >= 0.4 s
AND VALID_RETURN-backed epochs >= 1
AND not surface guarded
AND not protected by confirmed target
```

同一 map epoch 每 voxel 的 raw ray weight 先以 `1-exp(-raw/n0)` 饱和，然后 epoch counter 只增 1。
VALID_RETURN/NO_RETURN 分别计数，权重为 1.0/0.5。这样 ray 密度不再冒充独立历史证据。

## 3. Surface uncertainty band

candidate/stable background 附近 0.75 m 被标为 surface-guarded。band 内可保留 observed free，但
`updateState()` 禁止升级为 certified；stable/background endpoint 更新会刷新邻域 guard。地图边界也
按保守策略 guard，防止被截断的 ray history 形成虚假认证。

事件分类只允许旧地图查询为 `CERTIFIED_FREE` 时产生
`CERTIFIED_FREE_VIOLATION`。若关闭 V3 ablation，runner 才回退到 V2-style free event 语义。

## 4. 保护与失效规则

- confirmed support 截断 free ray，并对 endpoint quarantine；
- candidate/stable endpoint 与 free evidence 同 epoch 竞争时，surface 语义优先；
- stable surface 附近的 packets 不允许 ordinary maintenance 或 dormant reactivation，防止 wall lock；
- certified free 被真实背景占用后可转向 candidate/stable，而不是永久不可变。

## 5. 证据范围

R1 使用 CAL06/07/08、NEG05/06/03/04 和 S08A，报告 false-free、certified precision/recall、static
recall、expansion 和 false packets/min。CAL 只选择全局参数，NEG/S08A 不反向调参。最终数值见
`SOFT_VOFOD_V3_ABLATION.md`；真实 Mid360 的 grazing/no-return 可靠性尚未标定。
