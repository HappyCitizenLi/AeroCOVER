# SOFT-VoFOD V4 算法实现说明

日期：2026-08-24。状态：**development candidate，未冻结为 final V4**。

## 1. 唯一执行路径

```text
points_world + rays_checked
  -> 10 ms micro-batch / 5 Hz map epoch
  -> observed/certified free + candidate/stable background
  -> F certified-free violation / U unresolved packet
  -> sequential static-vs-CV epistemic classification
  -> provenance-preserving trajectory birth
  -> track-conditioned packet ownership
  -> CV/CA IMM + Hungarian
  -> survival / reportability / optional opportunity miss
  -> active / occluded / dormant / pre-reactivated / deleting
```

V4 没有新增平行 package、JPDA/MHT、scene-specific branch 或 GT 输入。所有改动都在原
`soft_vofod_mid360` core 内。当前 canonical 有意关闭三个未过实验门的候选：

- `opportunity_aware_existence: false`；
- `two_stage_dormant_reactivation: false`；
- `packet_anisotropic_los_covariance: false`。

effective opportunity、两阶段重激活和 LOS covariance 的实现仍保留，以便可复现实验；它们不是当前
production claim。sequential unknown inference 保持开启。

## 2. 地图与事件

自由空间由独立 map epochs 累积，先成为 `OBSERVED_FREE`，满足独立 epoch、持续时间和 valid-return
条件后才成为 `CERTIFIED_FREE`。未知静态 component 先进入 unresolved，static hypothesis 胜出后才
同化为 background。

V4 冷启动修正增加了因果 provenance：如果一个区域先被观测为 unknown/static，随后因视角变化产生
certified-free violation，该 violation 可维护既有 track，但不能绕过 epistemic gate 创建新 track。
static-unknown provenance 用 voxel 索引持久保存，内存上界受固定 voxel map 限制。未知运动 birth 还会
拒绝紧贴 map boundary 的不完整 component，避免墙边采样漂移伪装成目标轨迹。

## 3. Epistemic birth

对每条 U chain 维护静态点和 CV 两个模型，累计

\[
\Lambda_k=\Lambda_{k-1}+\log p(z_k\mid H_T)-\log p(z_k\mid H_B).
\]

只有 `Lambda >= 6` 才允许 U-birth；`Lambda <= -6` 或 unresolved 时间到期判为 static。不存在旧的一次
χ² fallback。F 与 U provenance 不混合。mapped-free hover 仍可走 F 路径，而“自 t=0 已存在的未知
静态物体”不能利用其自身后来造成的 free 证据升级。

## 4. Tracking 与生命周期

packet ownership、CV/CA IMM 和 Hungarian 沿用 V3。dormant memory 保存旧 ID 和受限 posterior。
V4 实现了显式 `PRE_REACTIVATED`：第一条 compatible packet 只预激活且不可报告，1 s 内第二条 ordinary
packet 才恢复 active；失败返回 dormant。S05 门控未获收益，因此 canonical 继续使用原一阶段路径。

truth-only 遮挡评估显示在线 `OCCLUDED` 在 S05 的 recall 为 0，所以它不列入核心创新；dormant 仍是
遮挡后的主要生命周期机制。

## 5. Measurement 与 opportunity

LOS candidate 使用

\[
R=R_{spread}+\sigma_\perp^2I+
(\sigma_\parallel^2-\sigma_\perp^2)uu^T,
\]

其中现有量直接给出 `sigma_perp²=sensor_variance+sampling_floor`，
`sigma_parallel²=sigma_perp²+shape_sigma²`，不新增拟合参数。M1 在单个 S04 seed 上只有很小收益，故关闭。

effective opportunity 的完整定义见 `SOFT_VOFOD_V4_OPPORTUNITY_MODEL.md`。其校准优于其他 V4
candidate，但严格 R5 的 HOTA/FN 仍劣于无 miss-update 的 O0，所以仅保留诊断。

## 6. 可复现版本

本文生成前代码 HEAD 为 `ae1b4ff8279a0e9961b4dcc4574818d5e935e184`。核心源码 SHA-256：

- `core.cpp`: `1d1b4f14ab2492f769cf57a9301cede2a59aca2e132d245730760677ab841186`
- `core.h`: `a3718a7f22ba98b0f67095be9c07bc7257f7c667807c548b9fee0b7c79bf4acb`
- canonical YAML: `3e9fb88d622358b00f2722c4b5143485b89aea821f962d1976e40faef6f85aff`

最终状态和所有未通过门见 `SOFT_VOFOD_V4_FINAL_REPORT.md`。
