# Adaptive Fusion Weights (GA/AA)

本文档说明当前工程中的自适应融合权重结构、配置项，以及当前默认实验设置。

## 1. 当前权重结构

当前工程将自适应融合权重写成显式分层结构：

```text
baseScore_j(t) = mask_j(t) * covScore_j(t) * linkQuality_j(t)
rawScore_j(t)  = baseScore_j(t) * cardinalityConfidence_j(t) * innovationPenalty_j(t) * historyScore_j(t)
```

其中：

- `mask_j(t)`：可用性掩码
- `covScore_j(t)`：协方差质量项
- `linkQuality_j(t)`：链路质量项
- `cardinalityConfidence_j(t)`：存在性/基数置信度项
- `innovationPenalty_j(t)`：NIS 统计一致性惩罚项
- `historyScore_j(t)`：历史稳定性项

当前阶段为了先把 NIS 的作用分析清楚，默认设置为：

```text
useHistory = false
```

因此当前主实验中实际生效的结构更接近：

```text
rawScore_j(t) = mask_j(t) * covScore_j(t) * linkQuality_j(t) * cardinalityConfidence_j(t) * innovationPenalty_j(t)
```

## 2. 各项含义

### 2.1 协方差质量项 `covScore`

实现位置：

- `multisensorLmb/computeAdaptiveFusionWeights.m`

对 measurement-updated LMB 做 m-projection 后，用协方差 trace 构造：

```text
covScore = 1 / (eps + mean(trace(T)))
```

它的职责是度量“估计是否集中、是否精确”。

### 2.2 NIS 一致性惩罚项 `innovationPenalty`

实现位置：

- `multisensorLmb/generateLmbSensorAssociationMatrices.m`

当前不再把 NIS 当作“越小越好”的质量分数，而是只把它作为统计一致性惩罚项使用。

具体分析见：

- `docs/NIS_IMPLEMENTATION_AND_ANALYSIS_CN.md`

它的职责是度量“创新统计是否与理论模型一致”。

### 2.3 链路质量项 `linkQuality`

实现位置：

- `multisensorLmb/computeAdaptiveFusionWeights.m`

基于通信层统计：

```text
linkQuality = delivered / (delivered + dropped)
```

### 2.4 历史稳定性项 `historyScore`

实现位置：

- `multisensorLmb/computeAdaptiveFusionWeights.m`

当前该模块仍保留，但默认先关闭，用于避免与 NIS 的分析混杂。

### 2.5 存在性/基数置信度项 `existenceConfidence`

实现位置：

- `multisensorLmb/computeAdaptiveFusionWeights.m`

这个因子不是再去看状态协方差，也不是再去看链路是否送达，而是看 measurement-updated posterior 里每个 Bernoulli 的 existence probability `r` 是否足够“尖锐”。

对单个 Bernoulli：

```text
certainty = |2r - 1|
```

其中：

- `r` 接近 `0` 或 `1`，说明“存在/不存在”判断更明确，certainty 更高
- `r` 接近 `0.5`，说明存在性判决更模糊，certainty 更低

当前实现对每个传感器使用 existence probability 加权平均：

```text
weightedConfidence = sum(r .* |2r - 1|) / sum(r)
existenceConfidenceScore = minScore + (1 - minScore) * weightedConfidence^power
```

它的职责是度量“这个节点当前的目标存在性判断是否可靠”，因此更直接对应 cardinality 一致性问题。

## 3. 当前默认配置

当前推荐默认值如下：

```matlab
model.adaptiveFusion.enabled = true;
model.adaptiveFusion.emaAlpha = 0.7;
model.adaptiveFusion.minWeight = 0.05;

model.adaptiveFusion.useCovariance = true;
model.adaptiveFusion.useLinkQuality = true;
model.adaptiveFusion.useExistenceConfidence = false;
model.adaptiveFusion.existenceConfidenceMinScore = 0.85;
model.adaptiveFusion.existenceConfidencePower = 2.0;

model.adaptiveFusion.useNIS = true;
model.adaptiveFusion.robustNIS = true;
model.adaptiveFusion.robustNISMin = 0.3;

model.adaptiveFusion.nisQuantileEnabled = true;
model.adaptiveFusion.nisQuantile = 0.7;
model.adaptiveFusion.nisConsistencyConfidence = 0.7;
model.adaptiveFusion.nisPenaltyScale = 4.0;
model.adaptiveFusion.nisPenaltyMin = 0.3;
model.adaptiveFusion.nisPenaltyLowerScale = 1.0;
model.adaptiveFusion.nisPenaltyUpperScale = 6.0;
model.adaptiveFusion.nisPenaltyLowerPower = 2.0;
model.adaptiveFusion.nisPenaltyUpperPower = 2.0;
model.adaptiveFusion.nisEmaEnabled = true;
model.adaptiveFusion.nisEmaAlpha = 0.7;

model.adaptiveFusion.useHistory = false;
model.adaptiveFusion.historyEmaAlpha = 0.8;
model.adaptiveFusion.historyScale = 2.0;
model.adaptiveFusion.historyMinScore = 0.4;
model.adaptiveFusion.historyCovWeight = 0.4;
model.adaptiveFusion.historyInnovationWeight = 0.4;
model.adaptiveFusion.historyCardinalityWeight = 0.2;
```

## 4. 为什么当前先关掉 history

原因很直接：

1. `historyScore` 里会同时使用 `covDiff` 与 `innovationDiff`
2. 如果在研究 NIS 的同时保留 history，会引入额外的间接耦合路径
3. 这会让我们难以判断改造后的 NIS 本身是否真的有效

因此当前采取的策略是：

- 先把 `historyScore` 关掉
- 先把 `NIS` 从质量因子改造成一致性惩罚项
- 先验证 NIS 解耦后的边际收益
- 再决定是否重新引入 history

## 5. 回滚方式

如果需要回到更早的行为，可按以下方式关闭当前 NIS 解耦模块：

```matlab
model.adaptiveFusion.nisQuantileEnabled = false;
model.adaptiveFusion.nisEmaEnabled = false;
model.adaptiveFusion.useHistory = false;
```

若只是想恢复 history：

```matlab
model.adaptiveFusion.useHistory = true;
```

## 6. 当前实验建议

在当前阶段，更建议优先做以下对比：

1. `w/o NIS -> robust NIS -> NIS`，且固定 `useHistory = false`
2. 观察 decoupled NIS 对一致性 OSPA / RMSE / cardinality 的影响
3. 只有当 decoupled NIS 的收益稳定后，再把 history 加回去

## 7. 当前调优结果（2026-03-09）

在 decoupled NIS 的基础上，进一步把惩罚项调整为“不对称软惩罚”：

- 放宽一致性区间：`nisConsistencyConfidence = 0.7`
- 下侧弱惩罚：`nisPenaltyLowerScale = 1.0`
- 上侧强惩罚：`nisPenaltyUpperScale = 6.0`
- 上下侧均使用平方型软惩罚：`nisPenaltyLowerPower = 2.0`，`nisPenaltyUpperPower = 2.0`

20 次 Monte Carlo 的最新结果为：

```text
Comprehensive (OSPA) consensus: 1.811 -> 1.810 -> 1.901
Position (RMSE) consensus:      3.173 -> 3.153 -> 3.329
Cardinality consensus:          0.214 -> 0.209 -> 0.234
```

顺序仍为：

- `w/o NIS`
- `robust NIS`
- `NIS`

当前结论是：

1. 调优后的 `robust NIS` 已经把 OSPA 恢复到与 `w/o NIS` 基本持平
2. 同时仍保留了轻微的 RMSE 与基数一致性收益
3. 普通 `NIS` 仍然不稳定，当前不建议作为默认方案

## 8. Tiered 通信配置下的新结果（2026-03-22）

在新的分档异构丢包通信配置下：

```matlab
commConfig.pDropLevels = [0, 0.1, 0.2, 0.5];
commConfig.pDropLevelCounts = [1, 4, 1, 2];
```

我们进一步测试了在 `协方差 + 链路质量` 基础上加入新的存在性/基数置信度因子。

推荐配置为：

```matlab
model.adaptiveFusion.useCovariance = true;
model.adaptiveFusion.useLinkQuality = true;
model.adaptiveFusion.useExistenceConfidence = true;
model.adaptiveFusion.existenceConfidenceMinScore = 0.85;
model.adaptiveFusion.existenceConfidencePower = 2.0;
model.adaptiveFusion.useNIS = false;
```

对应的 `5 trial` 消融结果为：

```text
+link quality:          OSPA 1.877771, RMSE 1.800945, Card 0.245250
+existence confidence: OSPA 1.874840, RMSE 1.779820, Card 0.244500
```

当前结论是：

1. `existenceConfidence` 比 `freshness` 和 `robust NIS` 更适合作为 `协方差 + 链路质量` 之后的第三个因子
2. 它没有重复表达“状态精度”或“通信可靠性”，而是补了“存在性判决可信度”这一维
3. 在当前 tiered 通信配置下，它实现了 `OSPA / RMSE / Cardinality` 三项指标同时改善
