# 双阈值多指标事件触发 GA-LMB 原型

## 实现边界

首轮实现是 GA-LMB、逐有向边触发、后验消息交换和单轮同步邻域融合。未实现异步多轮共识，也不把仿真真值误差用于在线触发。

主入口：

```matlab
[stateEstimatesBySensor, diagnostics] = ...
    runEventTriggeredDistributedLmbFilter( ...
        model, measurements, sensorTrajectories, ...
        neighborMap, commConfig, triggerConfig);
```

对比实验入口：

```matlab
[reportPath, summary] = ...
    runMultisensorFilters_formation_4plus4_DualThresholdEventTriggerCompare( ...
        numberOfTrials, baseSeed, useFixedSeed, writeReport, ...
        thresholdProfile, armSelection, experimentOverrides);
```

`thresholdProfile` 支持 `loose/default/strict/all`。`experimentOverrides.calibration` 可复用已有标定结果，避免重复运行全通信标定轨迹。

## 每步流程

1. 每个节点独立执行 LMB prediction 和本地 measurement update。
2. 每条有向边根据发送端当前后验、最近成功广播参考、接收端邻居缓存和当前链路质量独立分类。
3. `none` 不发送；`light` 发送逐标签矩匹配单高斯 LMB；`heavy` 发送完整 GM-LMB。
4. 接收端只融合当前步成功投递的消息，不融合陈旧缓存。
5. 当前轮采用固定 Metropolis 权重；可选 Balanced 兼容 arm 保持所有 EMA 和 weight floor 为 0。

共享的单传感器更新函数是
`multisensorLmb/updateLmbWithSensorMeasurement.m`。现有
`runParallelUpdateLmbFilter` 与事件触发循环都调用该函数，避免两条路径的关联和 missed-detection 逻辑漂移。

## 触发函数

逐标签效用为：

```text
U_label = q_local * (
    0.20 * stateDifference
  + 0.20 * innovationNovelty
  + 0.35 * informationGain
  + 0.25 * neighborDisagreement)
```

边级效用取所有有效标签的最大值。各项均限制在 `[0,1]`：

- `stateDifference`：当前后验相对最近成功广播参考的状态、协方差和存在概率变化。
- `innovationNovelty`：归一化 NIS 新颖度乘鲁棒 NIS consistency penalty。
- `informationGain`：prediction-to-update 协方差 log-det 降幅和 Bernoulli 熵降幅。
- `neighborDisagreement`：与接收端缓存的该发送端后验做标签对齐比较。
- `q_local`：存在概率、位置协方差和关联置信度组成的本地可靠性。

本仓库每个 birth location 在每个时刻都会产生 `rB=0.03` 的新标签。若按
`existenceThreshold=0.01` 判断“新标签”，系统会退化为每步强制重事件。因此
`forceLabelExistenceThreshold` 默认设为 `0.5`，只有确认标签的出现或消失才强制重事件；普通低概率 birth 候选仍进入效用和 payload 逻辑。

## 链路门控

链路质量是名义成功率与最近 5 次实际投递成功率的均值：

- outage：`none`
- `< 0.35`：light 取消，heavy 降为 light
- `[0.35, 0.70)`：heavy 降为 light
- `>= 0.70`：保持原事件

当前 sensor-level 丢包率映射到发送端全部出边；`commConfig.pDropByEdge`
可直接覆盖为逐边矩阵。链路恢复时，若最近成功广播参考超过 10 步，则原始事件强制为 heavy，再经过链路门控。

## 诊断输出

`diagnostics` 包含：

- `rawEventType/eventType`
- `attempted/delivered/downgraded/forcedHeavy`
- `utility/informationGain/linkQuality/referenceAge`
- `payloadScalars/payloadBytes`
- 本地 innovation 和 association confidence
- attempted/delivered/window-delivered/effective-weight 图的 lambda2
- per-label effective connectivity violation、label stale age p90
- self/light/heavy/stale fusion weight mass 和 weight entropy
- 汇总 trigger/light/heavy/delivery rate、降级次数、估算负载

固定种子 1-trial 和 5-trial 结果分别见：

- `RUN/GA/GA_DUAL_THRESHOLD_EVENT_TRIGGER_N1_SEED1_20260609_205503.md`
- `RUN/GA/GA_DUAL_THRESHOLD_EVENT_TRIGGER_N5_SEEDS2_6_20260609.md`

当前没有配置达到通信与 tracking/consensus 联合升级门槛，结论是保留原型和失败模式，不进入 20/50 trials。

## 2026-06-09 进度记录

### 事件触发线结论

已完成 GA-LMB、逐有向边触发、后验消息交换、轻/重 payload 和链路门控降级的首轮原型。5-trial 固定拓扑筛选显示，默认完整方法相对周期全量通信可减少约 52.6% 估算字节，但 local E-OSPA 退化约 13.1%，consensus OSPA 退化约 25.0%，未达到“字节至少下降 30%、local E-OSPA 退化不超过 5%、三项 network disagreement 各不超过 10%”的升级标准。

已尝试两个补救变体：

- 有界陈旧缓存预测融合：100-step 局部运行中通信量和 local E-OSPA 均劣于默认完整方法，未继续。
- 缺边权重回灌 self-mass：20-step smoke 中 consensus OSPA 退化从默认的约 15.3% 放大到约 24.0%，判定为负向。

因此，双阈值多指标事件触发作为通信节省原型可以保留，但当前证据不足以支撑继续扩大试验或进入论文主线。

### 动态拓扑线结论

根据新的研究边界，补充了 topology-only 动态重构原型：不改触发阈值和 payload 内容，只决定每步“连谁、断谁、边权怎么变”。当前实现按链路可靠性、有效标签重叠和几何互补性构造无向边收益，在保持静态 4+4 相同 16 条无向边预算的条件下选择连通拓扑，并用代数连通度下限做修复。

实验入口仍为：

```matlab
ov = struct('includeDynamicTopologyVariants', true);
[reportPath, summary] = ...
    runMultisensorFilters_formation_4plus4_DualThresholdEventTriggerCompare( ...
        1, 1, true, true, 'default', [2 7], ov);
```

动态拓扑补充记录见：

- `RUN/GA/GA_DUAL_THRESHOLD_EVENT_TRIGGER_N1_SEED1_20260609_231700.md`

100-step seed=2 的 topology-only 隔离结果：

| Arm | Bytes | Local E-OSPA | Consensus OSPA | 结论 |
|:--|--:|--:|--:|:--|
| Periodic full posterior | 32783504 | 2.229 | 2.044 | 静态 4+4 周期全量基线 |
| Periodic full posterior + dynamic topology | 28157392 | 2.180 | 2.166 | 字节 -14.1%，local E-OSPA -2.2%，consensus OSPA +6.0% |

20-step 组合筛选显示，`Full method + dynamic topology` 虽能满足 30% 字节下降并改善 local E-OSPA，但 consensus OSPA 退化约 16.9%，说明事件触发缺边和拓扑重构误差会叠加。下一步应把动态拓扑作为独立研究线推进：固定周期全量或固定每步边预算，先做 5-trial 对比 `static 4+4 / reliability-only / reliability+overlap / reliability+overlap+lambda2 floor`，再决定是否重新与事件触发组合。
## 2026-06-10 通过配置

继续迭代后，找到一个满足 5-trial 升级门槛的组合：`Light-floor dual threshold + dynamic topology`。核心调整是从“少发邻居后验”转为“多数有效边发送 light 后验，少数高效用边发送 heavy 后验，同时用守卫式动态拓扑控制信息分发路径”。这避免了默认事件触发方案中未触发边导致的一致性损失。

最终配置：

- `thresholdLow = 0.20`
- `thresholdHigh = 0.3708`
- `forceInitialHeavy = false`
- `forceLabelChangeHeavy = false`
- `forceStaleHeavy = false`
- `poorLinkThreshold = 0.0`
- `dynamicTopologyEnabled = true`
- `dynamicTopologyEdgeBudget = 16`
- `topologyStaticEdgeBonus = 0.35`
- `topologyFallbackToBaseOnConnectivityFailure = true`

守卫式动态拓扑逻辑：

1. 按链路可靠性、有效标签重叠和几何互补性计算候选边收益。
2. 静态 4+4 边额外加锚点收益，减少无收益拓扑抖动。
3. 在固定 16 条无向边预算下选择连通拓扑。
4. 若代数连通度低于静态基线，则回退静态拓扑。

5-trial 结果见：

- `RUN/GA/GA_LIGHT_FLOOR_GUARDED_DYNAMIC_N5_SEEDS2_6_20260610.md`

相对周期全量后验基线：

| Metric | Result |
|:--|--:|
| Estimated bytes | -51.89% |
| Local E-OSPA | +1.10% |
| Consensus OSPA | +2.51% |
| Position disagreement | +1.73% |
| Cardinality dispersion | +5.26% |

该配置通过 5-trial 升级门槛。需要注意的是，当前融合函数对 light 和 heavy 输入都会做标签级矩匹配，因此 heavy 主要体现为 payload 层级的全量后验传输，而不是当前融合结果的显著差异来源。若后续要把 heavy 的作用写成方法贡献，需要进一步实现或评估 full GM-LMB payload 在关联不确定性、混合多峰保持或多轮共识中的作用。

### Effective KLA 图诊断补充

为验证“动态拓扑 + 事件触发失败来自有效信息流图被打断，而通过配置来自高频 light 同步保护有效图”的判断，新增了 attempted、delivered、window-delivered 和实际融合权重图的 lambda2，以及 per-label 窗口连通性、label stale age 和 self/light/heavy fusion weight mass 诊断。

100-step seed=2 复现实验见：

- `RUN/GA/GA_DUAL_THRESHOLD_EVENT_TRIGGER_N1_SEED1_20260610_102720.md`

| Arm | Bytes | Local E-OSPA | Consensus OSPA | Delivered lambda2 | Window lambda2 | Effective-weight lambda2 | Label violation | Self weight | Light weight | Heavy weight |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Periodic full posterior | 32783504 | 2.229 | 2.044 | 1.886 | 2.000 | 0.373 | 0.193 | 0.392 | 0.000 | 0.608 |
| Light-floor dual threshold + dynamic topology | 14378744 | 2.218 | 2.064 | 1.899 | 2.226 | 0.370 | 0.173 | 0.404 | 0.532 | 0.064 |

该诊断支持当前机制解释：通过配置没有继续稀疏化融合图，而是让 delivered/window effective graph 至少保持在全量基线附近，同时把多数邻居融合质量从 heavy payload 换成 light payload。候选 arm 的 `newEdgeNoHandshakeRate = 0.7925`，说明本轮通过结果并不依赖新边 heavy handshake；后续 handshake 可作为更强动态/断链场景的待验证补救，而不应写成当前 5-trial 通过配置的必要机制。

### 关键部件最小消融

按下一步计划，已做 100-step seed=2 的最小消融矩阵，固定同一组测量、丢包随机数和阈值标定，拆开验证 light-floor、dynamic topology、静态边锚点和 fallback 的贡献。报告见：

- `RUN/GA/GA_DUAL_THRESHOLD_EVENT_TRIGGER_N1_SEED1_20260610_122655.md`

| Arm | Bytes reduction | Local E-OSPA change | Consensus OSPA change | Position change | Card. change | Gate |
|:--|--:|--:|--:|--:|--:|:--:|
| Periodic full posterior + dynamic topology | 14.1% | -2.2% | +6.0% | -20.0% | +0.0% | no |
| Light-floor dual threshold | 56.3% | +0.7% | +2.1% | +7.9% | +1.1% | yes |
| Light-floor dual threshold + dynamic topology | 56.1% | -0.5% | +1.0% | -8.8% | +6.6% | yes |
| Light-floor + dynamic topology (no static bonus) | 51.2% | +4.0% | +11.1% | +8.2% | +18.7% | no |
| Light-floor + dynamic topology (no fallback) | 56.3% | +0.4% | +2.1% | -6.3% | +3.3% | yes |

第一结论是 light-floor 是通信节省和通过 gate 的主因：不换拓扑时已经能以约 56% 字节下降保持 local、consensus、position 和 cardinality 都在门槛内。第二结论是 guarded dynamic topology 在 light-floor 上有增益：local E-OSPA 从 +0.7% 改为 -0.5%，consensus OSPA 从 +2.1% 改为 +1.0%，position disagreement 从 +7.9% 改为 -8.8%。第三结论是静态 4+4 边锚点很重要，去掉 `topologyStaticEdgeBonus` 后虽然 lambda2 更高，但 effective-weight lambda2 降到 0.323，self weight mass 升到 0.452，并且 consensus/cardinality 超门槛。第四结论是 fallback 在 seed=2 不是主要贡献源：去掉 fallback 仍通过，但 local、consensus 和 position 都略弱于完整 guarded 配置。

### Mixed label-wise payload 初步验证

根据“标签级触发 + 边级打包”的意见，新增了 `mixedPayloadEnabled` 原型：当边级事件为 heavy 时，不再把整条边所有标签都作为 full GM-LMB 发送，而是只让高效用标签保留完整 Gaussian mixture，其余有效标签以 moment-matched light LMB 形式随包发送。该实现不改变 LMB object 结构，只改变每个 label 的 Gaussian component 数，因此当前融合路径仍可复用。

100-step seed=2 对比见：

- `RUN/GA/GA_DUAL_THRESHOLD_EVENT_TRIGGER_N1_SEED1_20260610_132613.md`

| Arm | Bytes | Local E-OSPA | Consensus OSPA | Position disagreement | Card. dispersion | Effective-weight lambda2 |
|:--|--:|--:|--:|--:|--:|--:|
| Light-floor dual threshold + dynamic topology | 14378744 | 2.2183 | 2.0636 | 6.2174 | 0.2425 | 0.370 |
| Light-floor mixed-label payload + dynamic topology | 12793496 | 2.2183 | 2.0636 | 6.2174 | 0.2425 | 0.370 |

mixed label-wise payload 在该 seed 上相对 guarded light-floor 继续减少约 11.0% 字节，且 local、consensus、position、cardinality 和 effective graph 诊断完全持平。相对周期全量基线 `32783504` bytes，总通信降幅从 guarded light-floor 的约 56.1% 提升到约 61.0%。这说明当前 heavy payload 中确实有一部分标签可以降级为 light，而不破坏单轮 KLA 融合结果。

同时实现了可选 `lightCovarianceInflationEnabled`，用于按 association confidence 和 mixture entropy 对 light covariance 做保守膨胀。20-step seed=2 初测中，`Light-floor mixed robust payload + dynamic topology` 的 local E-OSPA 和 consensus OSPA 均劣于未膨胀 mixed payload，因此该开关暂时保留为实验项，不进入当前主线配置。

### Cross-layer topology score 初步验证

根据“拓扑选择应服务 KLA 有效信息流，而不是只看链路/几何分数”的意见，新增了 `topologyScoreMode='crossLayer'` 原型。该模式在动态拓扑候选边打分中显式加入：

- 预测融合价值：相邻节点同标签后验的位置、存在概率和协方差差异。
- 预测触发强度：把上述融合价值映射到当前 light/heavy 阈值区间。
- 覆盖修复价值：只在一侧存在的活跃标签比例。
- 几何互补性和链路可靠性。
- 估算 payload 字节惩罚。

该分数只影响“连哪条边”，不改变 light-floor 触发阈值、mixed label-wise payload 或融合函数。20-step seed=2 smoke 结果如下：

| Arm | Bytes | Local E-OSPA | Consensus OSPA | Topology lambda2 | 结论 |
|:--|--:|--:|--:|--:|:--|
| Light-floor mixed-label payload + dynamic topology | 928208 | 2.9192 | 2.3731 | 2.0821 | 当前 mixed payload 对照 |
| Cross-layer topology, static bonus 0.20 | 982672 | 3.0004 | 2.0819 | 2.4124 | consensus 明显改善，但字节和 local 变差 |
| Cross-layer topology, static bonus 0.35 | 1021408 | 3.0513 | 2.2008 | 2.3553 | 静态锚点更强后仍不是更优 trade-off |

这个结果支持有效 KLA 图解释：更直接地优化预期融合价值和图连通性，确实能降低短程 consensus OSPA。但它同时提高触发/传输强度，并牺牲 local E-OSPA，因此目前不替代 guarded light-floor/mixed payload 主配置，也不进入 100-step 或 5-trial 升级。后续若要继续 cross-layer 方向，应先加入接收端 need/request vector 或边级 expected-value gate，而不是简单提高拓扑 score 的连通性偏好。
