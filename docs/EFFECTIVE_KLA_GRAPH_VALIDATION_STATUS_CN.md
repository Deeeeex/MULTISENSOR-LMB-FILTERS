# Effective KLA 图验证阶段记录

日期：2026-06-11

本文档是 `effective-kla-graph-validation` 分支的独立阶段快照。主线长记录仍保留在
`docs/DUAL_THRESHOLD_EVENT_TRIGGER_RESEARCH_CN.md`；本文只记录当前证据链、候选收敛和最终验证口径。

## 当前问题

前一轮结果说明，默认双阈值事件触发与动态拓扑叠加后并不是“天然冲突”，更像是两层稀疏化串联后打断了单轮 KLA 融合真正依赖的有效信息流图。动态拓扑单独有效，是因为每步仍有完整后验沿预算边传播；默认事件触发失败，是因为部分边不发或只偶发 heavy，导致实际融合权重图的连通性下降、self weight mass 上升、label stale age 增大。

因此本阶段的核心验证目标不是继续加机制，而是验证以下判断：

1. 通过配置的关键不是 heavy payload，而是保持高频 light posterior 同步。
2. guarded dynamic topology 只有在有效 KLA 图不被破坏时才有收益。
3. mixed label-wise payload 可以作为压缩 ablation，但不能在当前融合实现下宣称提供额外 tracking/consensus 收益。
4. 如果周期 light posterior 已经足够，则事件触发主线需要重新降级为 ablation，而不是论文主贡献。

## 已实现部件

本分支已实现并验证以下实验部件：

- Effective KLA 图诊断：attempted、delivered、window-delivered、effective-weight 图的 lambda2。
- 标签级诊断：per-label connectivity violation、window label violation、label stale p90/p95。
- 融合权重诊断：self/light/heavy/stale weight mass、weight entropy。
- 动态拓扑诊断：topology churn、new-edge no-handshake、handshake count、handshake byte share。
- `alwaysLight` 事件策略，用于构造周期 light posterior 风险基线。
- `mixedPayloadEnabled`，用于 heavy 边上只保留高效用标签的 GM payload，其余标签降级为 light moment-matched payload。
- `forceNewEdgeHandshakeHeavy`，用于动态拓扑新边的 heavy handshake 变体。
- `topologyScoreMode='crossLayer'` 原型，用于测试显式融合价值驱动的拓扑打分。
- `modeAwareFusionWeights` 和 effective KLA graph guard 原型，用于测试按 payload mode 或 stale 状态调权。

关键实现入口：

- `RUN/GA/runMultisensorFilters_formation_4plus4_DualThresholdEventTriggerCompare.m`
- `multisensorLmb/runEventTriggeredDistributedLmbFilter.m`
- `multisensorLmb/buildMixedLmbPayload.m`
- `multisensorLmb/classifyLmbCommunicationEvent.m`
- `tests/test_dual_threshold_event_trigger.m`

## 阶段证据

### Seed=2 阶段矩阵

报告：`RUN/GA/GA_DUAL_THRESHOLD_EVENT_TRIGGER_N1_SEED1_20260610_150731.md`

| Arm | Bytes reduction | Local E-OSPA change | Consensus OSPA change | Effective-weight lambda2 | Gate |
|:--|--:|--:|--:|--:|:--:|
| Full method (default) | 58.0% | +9.8% | +22.7% | 0.268 | no |
| Full method + dynamic topology | 49.2% | +10.6% | +29.1% | 0.240 | no |
| Dynamic topology + new-edge handshake | 36.6% | +6.0% | +21.8% | 0.266 | no |
| Dynamic topology + handshake + light backbone | 60.4% | -0.8% | -0.7% | 0.372 | yes |
| Dynamic topology + mode-aware KLA graph | 58.5% | +5.2% | +20.7% | 0.270 | no |
| Dynamic topology + effective KLA graph guard | 56.1% | +6.2% | +19.5% | 0.279 | no |
| Light-floor mixed-label payload + dynamic topology | 61.0% | -0.5% | +1.0% | 0.370 | yes |

阶段结论：

- 单独 new-edge handshake 不能修复组合失败。
- 低阈值 light backbone 把 effective-weight lambda2 恢复到全通信基线附近，是 consensus 修复的主要来源。
- mode-aware KLA graph 和 effective KLA graph guard 没有带来正收益，当前不应进入主线。
- mixed label-wise payload 在 seed=2 上有额外字节收益，但 tracking/consensus 与 light-floor 主线基本一致。

### Development 5-trial 候选对决

报告：`RUN/GA/GA_DUAL_THRESHOLD_EVENT_TRIGGER_N5_SEED1_20260610_165506.md`

该轮使用 seeds 2-6，属于开发集，不作为 held-out 泛化证据。

| Arm | Bytes reduction | Local E-OSPA change | Consensus OSPA change | Position change | Card. change | Gate |
|:--|--:|--:|--:|--:|--:|:--:|
| Dynamic topology + handshake + light backbone | 58.6% | +0.2% | +1.0% | +0.9% | +3.5% | yes |
| Light-floor mixed-label payload + dynamic topology | 58.4% | +1.1% | +2.5% | +1.7% | +5.3% | yes |

阶段结论：

- 两个候选都通过 development gate。
- C1 在 consensus 和 cardinality 上更稳，且没有明显字节劣势。
- 仅凭 development seeds 不足以选择最终主线，因为 seeds 2-6 已参与方法调试。

### Held-out 5-trial 候选选择

报告：`RUN/GA/GA_DUAL_THRESHOLD_EVENT_TRIGGER_N5_SEED6_20260610_193640.md`

该轮使用 held-out seeds 7-11，冻结机制和阈值，不再调参。

| Arm | Bytes reduction | Local E-OSPA change | Consensus OSPA change | Position change | Card. change | Pass count | 结论 |
|:--|--:|--:|--:|--:|--:|:--:|:--|
| Periodic light posterior + guarded dynamic topology | 58.0% | +0.2% | +0.5% | +1.8% | +0.8% | 5/5 | Pareto，当时最强候选 |
| Old mainline: LightFloor-GuardedTopo | 50.6% | +0.9% | +1.7% | +7.4% | -2.8% | 4/5 | 通过均值 gate，但被 periodic light 支配 |
| C1: LightBackbone-GuardedTopo | 57.6% | +0.2% | +0.5% | +1.7% | +0.9% | 5/5 | 与 periodic light 近似持平但字节略高 |
| C2: MixedLabel-LightFloor-GuardedTopo | 57.3% | +0.9% | +1.7% | +7.4% | -2.8% | 4/5 | 比旧主线省字节，但不优于 periodic light |

held-out 结论：

- `Periodic light posterior + guarded dynamic topology` 从风险基线变成该阶段 Pareto arm。
- C1 的 new-edge handshake/light-backbone 与 periodic light 近似同指标，但 handshake 带来额外 byte share，当前没有必要作为主线。
- C2 的 tracking/consensus 与旧 light-floor 主线一致，主要价值是 payload compression ablation。
- 该阶段证据不支持把 C1 或 C2 写成最终主方法。

### Held-out 20-trial 候选选择

报告：`RUN/GA/GA_DUAL_THRESHOLD_EVENT_TRIGGER_N20_SEED11_20260611_095640.md`

该轮使用 held-out seeds 12-31，冻结机制和阈值，不再调参。

| Arm | Bytes reduction | Local E-OSPA change | Consensus OSPA change | Position change | Card. change | Pass count | 结论 |
|:--|--:|--:|--:|--:|--:|:--:|:--|
| Periodic light posterior + guarded dynamic topology | 58.5% | -0.5% | +0.4% | -7.4% | -0.0% | 15/20 | Pareto，N20 主候选 |
| Old mainline: LightFloor-GuardedTopo | 51.5% | +0.1% | +2.0% | -4.3% | -0.4% | 13/20 | 通过均值 gate，但被 periodic light 支配 |
| C1: LightBackbone-GuardedTopo | 57.9% | -0.5% | +0.4% | -7.4% | -0.1% | 15/20 | 与 periodic light 几乎同质，handshake 略增字节 |
| C2: MixedLabel-LightFloor-GuardedTopo | 58.2% | +0.1% | +2.0% | -4.3% | -0.4% | 13/20 | 压缩旧主线字节，但不优于 periodic light |

20-trial 结论：

- 四个通信节省候选都通过均值 gate，但在 N20 阶段，paired pass count 和 Pareto 关系继续支持 `Periodic light posterior + guarded dynamic topology`。
- C1 与 periodic light 的 tracking、consensus 和 effective KLA 图指标几乎一致，但 handshake byte share 为 0.022，没有换来收益。
- C2 与旧 light-floor 主线 tracking/consensus 基本一致，说明 mixed payload 只是压缩 ablation。
- 旧主线和 C2 的 effective-weight lambda2 为 0.363，低于 periodic light/C1 的 0.369；self weight mass 也更高。

### Final N50 组件验证

报告：`RUN/GA/GA_DUAL_THRESHOLD_EVENT_TRIGGER_N50_SEED31_20260611_113226.md`

该轮使用 held-out seeds 32-81，固定机制和阈值，对比静态 full、动态 full、
静态 light 和动态 light 四臂，用来拆开 payload 替代与 guarded topology
的贡献。

| Arm | Bytes reduction | Local E-OSPA change | Consensus OSPA change | Position change | Card. change | Pass count | 结论 |
|:--|--:|--:|--:|--:|--:|:--:|:--|
| Periodic full posterior + dynamic topology | -6.1% | +4.1% | +11.1% | +6.4% | +4.3% | 0/50 | 动态拓扑单独不成立，且平均字节上升 |
| Periodic light posterior on static topology | 58.6% | +0.0% | +0.0% | +0.0% | +0.0% | 50/50 | 最终 Pareto arm |
| Periodic light posterior + guarded dynamic topology | 58.3% | +0.1% | +0.7% | +4.7% | +0.6% | 35/50 | 通过均值 gate，但被静态 light 支配 |

N50 结论比 N20 更收敛：

- `Periodic light posterior on static topology` 与 `Periodic full posterior` 的
  local E-OSPA、consensus OSPA、position disagreement 和 cardinality
  dispersion 完全持平，同时节省约 58.6% 字节，并在 paired gate 上 50/50
  通过。
- `Periodic light posterior + guarded dynamic topology` 仍通过均值 gate，
  但字节略高、position disagreement 退化约 4.7%，paired pass count 为
  35/50，不再是 Pareto arm。
- `Periodic full posterior + dynamic topology` 平均字节反而增加约 6.1%，
  consensus OSPA 退化约 11.1%，确认 guarded topology 不是当前结果的主要
  通信节省来源。
- Effective KLA 诊断显示，静态 light 和静态 full 的 attempted/delivered/
  window/effective-weight 图指标完全一致；差别只是 heavy weight mass 被
  light weight mass 替代。这说明当前融合路径下 light payload 已足以复现
  full posterior 的单轮 moment-matched KLA 输入。

## 当前收敛判断

截至 final N50，最稳妥的解释已经从“guarded dynamic topology + light
同步”进一步收窄为：

```text
static topology + all-edge light posterior synchronization
```

这是当前单轮、moment-matched KLA 融合实现下唯一由 N50 支撑的主机制。它
保留静态 4+4 的有效融合图，只把 full posterior payload 替换为逐标签
moment-matched light payload，从而获得约 58.6% 字节下降且 tracking/
consensus 与 full posterior 完全持平。当前应把 `Periodic light posterior
on static topology` 视为最终强基线或主候选；`Periodic light posterior +
guarded dynamic topology` 只能作为通过 gate 但被支配的 topology ablation。

相反，原始双阈值事件触发里的 heavy/mixed 逻辑目前更适合作为 ablation：

- heavy payload 在当前融合函数中也会进入标签级矩匹配，尚未体现 full GM-LMB 在多峰保持或关联不确定性中的实际优势。
- mixed label-wise payload 可以降低 heavy 边冗余字节，但没有改变当前单轮融合结果。
- new-edge handshake 在更强断链或更剧烈动态拓扑场景下仍可能有价值，但不是当前 5-trial 通过配置的必要机制。
- guarded dynamic topology 在 N20 中看起来可作为强基线，但 N50 显示它不是
  payload 节省的必要条件，也不是当前场景下的 Pareto 改进。

## 下一步判断

N50 已经完成，当前不应继续追加同类长实验。后续只有两个清晰方向：

1. 若目标是短期可写的可靠结果，应把主张收窄为 `light posterior payload
   compression for static distributed KLA`，并把动态拓扑、事件触发、handshake
   和 mixed payload 都作为 negative/neutral ablations。
2. 若目标仍是事件触发或动态拓扑主贡献，需要先实现能让 heavy/full GM-LMB
   payload 在融合路径中产生真实差异的融合逻辑，或设计更强断链/request 场景。
   这属于新机制，不应在当前 N50 证据上直接扩大 claim。

已完成的 N50 复现实验入口保留为：

```matlab
addpath('RUN/GA');
[reportPath, summaryPath, summary] = ...
    runPeriodicLightGuardedTopologyFinalN50();
```

dry-run 配置检查：

```matlab
[~, ~, ~, config] = runPeriodicLightGuardedTopologyFinalN50(false);
```

## 后续写作边界

当前不应主张：

- heavy payload 已经在融合精度上发挥 full GM-LMB 优势。
- mixed label-wise payload 带来了 tracking 或 consensus 改善。
- new-edge handshake 是当前通过 gate 的必要机制。
- cross-layer topology score 已经优于 guarded topology。
- guarded dynamic topology 是当前 N50 结果中的 Pareto 主贡献。
- 双阈值事件触发是最终通信节省的必要机制。

当前可以主张：

- Effective KLA graph 诊断解释了默认事件触发失败与 light-backbone 成功的差异。
- 当前单轮 moment-matched KLA 更依赖高频 light posterior 同步，而不是稀疏 heavy 事件。
- 在当前融合实现中，静态 topology 的 periodic light posterior 可以以约 58.6%
  字节下降复现 full posterior 的 N50 tracking/consensus 结果。
- guarded dynamic topology、mixed payload 和 handshake 是已经实现的可复现实验项，
  但需要更强场景或更完整 GM-LMB 融合路径才能成为主线贡献。
