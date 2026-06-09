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
