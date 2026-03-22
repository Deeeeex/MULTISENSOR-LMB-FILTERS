# Adaptive Fusion Weights (GA/AA)

本文档说明当前工程中的自适应融合权重结构、配置项，以及当前 `main` 分支上的推荐主线配置。

## 1. 当前权重结构

当前工程将自适应融合权重写成显式分层结构。共享质量骨架是：

```text
baseScore_j(t) = mask_j(t) * covScore_j(t) * linkQuality_j(t)
rawScore_j(t)  = baseScore_j(t) * existenceConfidence_j(t) * innovationPenalty_j(t) * historyScore_j(t)
```

其中：

- `mask_j(t)`：可用性掩码
- `covScore_j(t)`：协方差质量项
- `linkQuality_j(t)`：链路质量项
- `existenceConfidence_j(t)`：存在性/基数置信度项
- `innovationPenalty_j(t)`：NIS 统计一致性惩罚项
- `historyScore_j(t)`：历史稳定性项

如果启用 decoupled KLA，则在共享骨架上再拆出 spatial / existence 两条分支：

```text
spatialScore_j(t)   = blend(rawScore_j(t), spatialDedicatedScore_j(t), eta_s)
existenceScore_j(t) = blend(rawScore_j(t), existenceDedicatedScore_j(t), eta_e)
```

若进一步启用 structure-aware refinement，则对两支再叠加一个很弱的结构先验调制：

```text
spatialScore_j(t)   = spatialScore_j(t)   * spatialStructurePrior_j(t)^gamma_s
existenceScore_j(t) = existenceScore_j(t) * existenceStructurePrior_j(t)^gamma_e
```

当前 `main` 分支上的最佳主线配置里：

```text
useNIS = false
useHistory = false
useDecoupledKla = true
useStructureAwareKla = true
```

因此实际主实验最接近的结构是：

```text
rawScore_j(t)       = mask_j(t) * covScore_j(t) * linkQuality_j(t) * existenceConfidence_j(t)
spatialScore_j(t)   = weakly-refined decoupled spatial branch
existenceScore_j(t) = very-lightly-refined decoupled existence branch
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

### 2.6 弱结构先验解耦项 `structure-aware decoupled KLA`

实现位置：

- `multisensorLmb/computeAdaptiveFusionWeights.m`
- `multisensorLmb/runDistributedLmbFilter.m`

这一层不是新的主质量因子，而是加在三因子 baseline 上的一个很弱 refinement。

当前实现里：

- `spatial` 分支允许稍强一点的结构修正
- `existence` 分支只允许非常轻的结构修正
- 结构先验来自局部子图重叠和固定分档丢包率的可靠性信息

它的职责是：

- 在不破坏 `existenceConfidence` 主导作用的前提下，轻微改善 spatial consensus
- 保持 cardinality 不退化

## 3. 当前推荐配置

### 3.1 当前主线 best 配置

当前 `main` 分支推荐直接使用以下配置复现主线 best：

```matlab
model.adaptiveFusion.enabled = true;
model.adaptiveFusion.emaAlpha = 0.7;
model.adaptiveFusion.minWeight = 0.05;

model.adaptiveFusion.useCovariance = true;
model.adaptiveFusion.useLinkQuality = true;
model.adaptiveFusion.useExistenceConfidence = true;
model.adaptiveFusion.existenceConfidenceMinScore = 0.85;
model.adaptiveFusion.existenceConfidencePower = 2.0;

model.adaptiveFusion.useDecoupledKla = true;
model.adaptiveFusion.useStructureAwareKla = true;
model.adaptiveFusion.spatialDecouplingStrength = 0.5;
model.adaptiveFusion.existenceDecouplingStrength = 0.15;
model.adaptiveFusion.spatialStructureStrength = 0.35;
model.adaptiveFusion.existenceStructureStrength = 0.05;
model.adaptiveFusion.structureReliabilityPower = 0.25;
model.adaptiveFusion.structureReliabilityMinScore = 0.25;

model.adaptiveFusion.useNIS = false;
model.adaptiveFusion.useHistory = false;
```

主报告是：

- `RUN/GA/GA_TIERED_LINK_ABLATION_20260322_023216.md`

### 3.2 三因子 baseline 配置

如果只想回到结构 refinement 之前的三因子 baseline，推荐值如下：

```matlab
model.adaptiveFusion.enabled = true;
model.adaptiveFusion.emaAlpha = 0.7;
model.adaptiveFusion.minWeight = 0.05;

model.adaptiveFusion.useCovariance = true;
model.adaptiveFusion.useLinkQuality = true;
model.adaptiveFusion.useExistenceConfidence = true;
model.adaptiveFusion.existenceConfidenceMinScore = 0.85;
model.adaptiveFusion.existenceConfidencePower = 2.0;

model.adaptiveFusion.useDecoupledKla = false;
model.adaptiveFusion.useStructureAwareKla = false;
model.adaptiveFusion.useNIS = false;
model.adaptiveFusion.useHistory = false;
```

对应报告是：

- `RUN/GA/GA_TIERED_LINK_ABLATION_20260322_001613.md`

## 4. 为什么当前先关掉 history 和 NIS

原因很直接：

1. 当前主线 best 已经在 `NIS = off`、`history = off` 的设置下超过旧 best
2. `historyScore` 会引入额外的间接耦合路径，不利于判断三因子主线和弱结构 refinement 的真实边际收益
3. `NIS` 更适合作为二级一致性分析模块，而不是当前主线配置的一部分

因此当前采取的策略是：

- 主线先固定在 `covariance + link quality + existenceConfidence + weak structure-aware decoupling`
- `historyScore` 保持关闭
- `NIS` 保持关闭
- 等主线结果稳定后，再把 `NIS/history` 当作 secondary ablation 单独评估

## 5. 回滚方式

如果需要回退到三因子 existence-confidence baseline，可按以下方式关闭当前弱结构解耦层：

```matlab
model.adaptiveFusion.useDecoupledKla = false;
model.adaptiveFusion.useStructureAwareKla = false;
```

如果需要回到更早的 NIS 分析线，可按以下方式重新打开该模块：

```matlab
model.adaptiveFusion.useNIS = true;
model.adaptiveFusion.robustNIS = true;
model.adaptiveFusion.robustNISMin = 0.3;
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

1. `+link quality -> +existence confidence -> +structure-aware decoupled KLA`
2. 在更多 seed、更多通信等级下验证弱结构 refinement 是否稳定
3. 只有主线结果稳定后，再把 `NIS/history` 加回去做 secondary 分析

## 7. 历史 NIS 调优结果（2026-03-09，次线）

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

我们先测试了在 `协方差 + 链路质量` 基础上加入新的存在性/基数置信度因子，随后又继续测试了一个较弱的 structure-aware decoupled KLA 版本。

推荐配置为：

```matlab
model.adaptiveFusion.useCovariance = true;
model.adaptiveFusion.useLinkQuality = true;
model.adaptiveFusion.useExistenceConfidence = true;
model.adaptiveFusion.existenceConfidenceMinScore = 0.85;
model.adaptiveFusion.existenceConfidencePower = 2.0;
model.adaptiveFusion.useNIS = false;
```

existence-confidence baseline 的 `5 trial` 消融结果为：

```text
+link quality:          OSPA 1.877771, RMSE 1.800945, Card 0.245250
+existence confidence: OSPA 1.874840, RMSE 1.779820, Card 0.244500
```

在此基础上，当前 best 配置进一步采用：

```matlab
model.adaptiveFusion.useDecoupledKla = true;
model.adaptiveFusion.useStructureAwareKla = true;
model.adaptiveFusion.spatialDecouplingStrength = 0.5;
model.adaptiveFusion.existenceDecouplingStrength = 0.15;
model.adaptiveFusion.spatialStructureStrength = 0.35;
model.adaptiveFusion.existenceStructureStrength = 0.05;
model.adaptiveFusion.structureReliabilityPower = 0.25;
model.adaptiveFusion.structureReliabilityMinScore = 0.25;
model.adaptiveFusion.useNIS = false;
model.adaptiveFusion.useHistory = false;
```

对应的 `5 trial` 新结果为：

```text
+link quality:                    OSPA 1.877771, RMSE 1.800945, Card 0.245250
+structure-aware decoupled KLA:  OSPA 1.863592, RMSE 1.749731, Card 0.244500
```

当前结论是：

1. `existenceConfidence` 比 `freshness` 和 `robust NIS` 更适合作为 `协方差 + 链路质量` 之后的第三个因子
2. 它没有重复表达“状态精度”或“通信可靠性”，而是补了“存在性判决可信度”这一维
3. 在此基础上，一个足够弱的 structure-aware decoupled KLA 还能继续降低 `OSPA / RMSE`，并保持 `Cardinality` 不退化
4. 当前有效策略不是强化 existence 结构偏置，而是只给 existence 分支非常轻的结构调制
