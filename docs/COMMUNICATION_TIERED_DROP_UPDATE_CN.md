# 通信配置改动记录：分档丢包模型

本文档记录本轮通信层配置上的改动，重点说明为什么从“统一固定丢包率”切换到“分档异构丢包率”，以及这项改动在当前 formation `4+4` 实验中的使用方式和初步结果。

## 1. 改动背景

此前 Level 2 的默认链路模型是：

```matlab
commConfig.linkModel = 'fixed';
commConfig.pDrop = 0.2;
```

这意味着：

- 所有通过带宽筛选的传感器包，都使用同一个标量 `pDrop`
- 各节点在统计意义上是同分布掉包
- `linkQuality` 虽然会随时间变化，但节点间长期差异较弱

这类配置适合做“整体通信变差”的压力测试，但不利于体现“不同节点链路质量不同”这一类真实网络异构性。

从自适应权重的角度看，如果所有节点的掉包机制本来就几乎一样，那么 `linkQuality` 这个因子的可区分度会被压平。

## 2. 这次改了什么

本轮改动在 [common/applyCommunicationModel.m](../common/applyCommunicationModel.m) 中给 `fixed` 链路模式增加了两类新入口：

- `pDropBySensor`
  直接给出每个传感器的固定丢包率向量
- `pDropLevels + pDropLevelCounts`
  用离散档位和档位个数自动生成 `pDropBySensor`

解析优先级是：

1. `pDropBySensor`
2. `pDropLevels + pDropLevelCounts`
3. 标量 `pDrop`

也就是说，这次并没有推翻旧的 `pDrop` 写法，只是在其上增加了更强的异构配置能力。

## 3. 分档丢包模型的设计

### 3.1 设计目标

目标不是简单地“把每个传感器随机设成不同的连续丢包率”，而是构造一个更容易解释、更接近工程直觉的链路分层：

- 好链路
- 较好链路
- 一般链路
- 差链路

因此采用分档设计，而不是连续随机设计。

### 3.2 当前默认档位

当前 formation `4+4` 实验里使用：

```matlab
commConfig.pDropLevels = [0, 0.1, 0.2, 0.5];
commConfig.pDropLevelCounts = [1, 4, 1, 2];
```

对应含义：

- 1 个节点：`pDrop = 0`
- 4 个节点：`pDrop = 0.1`
- 1 个节点：`pDrop = 0.2`
- 2 个节点：`pDrop = 0.5`

对于 8 传感器场景，这组配置的平均值是：

$$
\frac{1 \times 0 + 4 \times 0.1 + 1 \times 0.2 + 2 \times 0.5}{8} = 0.2
$$

因此它与历史上常用的 `pDrop = 0.2` 在总体通信强度上是对齐的。

### 3.3 Trial 内与 Trial 间的行为

当前实现中：

- 每个 trial 开始时，会按 `pDropLevelCounts` 先生成一组长度为传感器数的 `pDropBySensor`
- 再对这组档位做一次随机打散，分配给各传感器
- 一旦分配完成，该 trial 内保持不变

这样做的原因是：

- trial 内固定，便于解释“某个节点长期链路较差”的效应
- trial 间打散，避免总是同一编号节点吃亏

## 4. 与旧配置相比的变化点

### 4.1 旧配置：统一固定丢包率

```matlab
commConfig.linkModel = 'fixed';
commConfig.pDrop = 0.2;
```

特点：

- 节点间同分布
- 易于实现
- 适合验证“整体通信压力”
- 不容易拉开自适应链路质量因子的作用

### 4.2 中间尝试：连续异构丢包率

在本轮探索中还测试过“每个 trial 随机生成一组连续 `pDropBySensor`，但总体平均仍约束在 `0.2` 附近”的方案。

特点：

- 异构性比统一 `pDrop` 强
- 能提升 `linkQuality` 的辨识度
- 但可解释性一般，不如分档清楚

### 4.3 当前推荐：分档异构丢包率

```matlab
commConfig.linkModel = 'fixed';
commConfig.pDrop = 0.2;
commConfig.pDropLevels = [0, 0.1, 0.2, 0.5];
commConfig.pDropLevelCounts = [1, 4, 1, 2];
```

特点：

- 总体平均仍然和旧配置对齐
- 节点间存在清晰的离散层级
- 更接近“网络里有好链路也有坏链路”的直觉
- 更适合作为论文或实验文档中的通信设置说明

## 5. 当前实验脚本

围绕这套最新通信配置，当前已经新增了三类脚本：

- [RUN/GA/runMultisensorFilters_formation_4plus4_TieredLinkCompare.m](../RUN/GA/runMultisensorFilters_formation_4plus4_TieredLinkCompare.m)
  固定权重 vs 自适应 robust NIS
- [RUN/GA/runMultisensorFilters_formation_4plus4_TieredLinkNISCompare.m](../RUN/GA/runMultisensorFilters_formation_4plus4_TieredLinkNISCompare.m)
  `w/o NIS -> robust NIS -> NIS`
- [RUN/GA/runMultisensorFilters_formation_4plus4_TieredLinkFreshnessCompare.m](../RUN/GA/runMultisensorFilters_formation_4plus4_TieredLinkFreshnessCompare.m)
  `robust NIS baseline -> robust NIS + freshness`
- [RUN/GA/runMultisensorFilters_formation_4plus4_TieredLinkAblation.m](../RUN/GA/runMultisensorFilters_formation_4plus4_TieredLinkAblation.m)
  `fixed weights -> +covariance -> +link quality -> +(extra factor)`

这些脚本共享同一组通信配置，因此可以在同一个通信口径下比较不同权重策略。

## 6. 当前验证结论

以下结果都来自 `5 trial`，并使用同一套分档通信配置。

### 6.1 固定权重 vs adaptive robust NIS

报告：

- [RUN/GA/GA_TIERED_LINK_COMPARE_20260321_191405.md](../RUN/GA/GA_TIERED_LINK_COMPARE_20260321_191405.md)

结果：

- `Comprehensive (OSPA)`: `2.585993 -> 1.908967`
- `Position (RMSE)`: `2.855428 -> 2.980071`
- `Cardinality`: `0.850250 -> 0.262250`

结论：

- 在分档异构链路下，自适应权重对综合 OSPA 和 cardinality 的改善非常明显
- 位置 RMSE 没有同步改善，甚至略差

### 6.2 NIS 的作用

报告：

- [RUN/GA/GA_TIERED_LINK_NIS_COMPARE_20260321_193628.md](../RUN/GA/GA_TIERED_LINK_NIS_COMPARE_20260321_193628.md)

结果：

- `Comprehensive (OSPA)`: `1.909267 -> 1.908967 -> 2.007700`
- `Position (RMSE)`: `2.934317 -> 2.980071 -> 3.173222`
- `Cardinality`: `0.267000 -> 0.262250 -> 0.300500`

结论：

- `robust NIS` 和 `w/o NIS` 基本打平
- `robust NIS` 在综合 OSPA 上略优，在 cardinality 上略优
- `non-robust NIS` 仍然明显更差

### 6.3 freshness 的作用

报告：

- [RUN/GA/Del_GA_TIERED_LINK_FRESHNESS_COMPARE_20260321_193131.md](../RUN/GA/Del_GA_TIERED_LINK_FRESHNESS_COMPARE_20260321_193131.md)

结果：

- `Comprehensive (OSPA)`: `1.908967 -> 1.909680`
- `Position (RMSE)`: `2.980071 -> 2.979829`
- `Cardinality`: `0.262250 -> 0.262500`

结论：

- `freshness` 因子在这套通信配置下仍然几乎不起作用
- 当前更值得保留的是“分档通信 + NIS”这条主线，而不是继续拓展 freshness
- 对这类已验证无效或收益不足的因素，相关报告统一加 `Del_` 前缀，表示“已尝试，但不进入当前主线”

### 6.4 新因子：existence confidence

报告：

- [RUN/GA/GA_TIERED_LINK_ABLATION_20260322_001613.md](../RUN/GA/GA_TIERED_LINK_ABLATION_20260322_001613.md)

这次测试的是在 `协方差 + 链路质量` 基础上，再加入一个新的“存在性/基数置信度”因子。它不是看链路是否丢包，也不是看协方差大小，而是看各 Bernoulli existence probability 是否足够尖锐。

推荐参数是：

```matlab
model.adaptiveFusion.useExistenceConfidence = true;
model.adaptiveFusion.existenceConfidenceMinScore = 0.85;
model.adaptiveFusion.existenceConfidencePower = 2.0;
model.adaptiveFusion.useNIS = false;
```

`5 trial` 结果：

- `+link quality`: `OSPA 1.877771`, `RMSE 1.800945`, `Cardinality 0.245250`
- `+existence confidence`: `OSPA 1.874840`, `RMSE 1.779820`, `Cardinality 0.244500`

结论：

- 这是目前在 tiered 通信配置下，第一个相对 `协方差 + 链路质量` baseline 实现三项指标同时改善的新因子
- 它比 `freshness` 更有效，也比当前 `robust NIS` 更适合放在 `协方差 + 链路质量` 之后作为第三个因子
- 从论文表述上，它补充的是“存在性判决可信度”，与“状态精度”和“通信可靠性”形成互补

### 6.5 当前最优：structure-aware decoupled KLA

报告：

- [RUN/GA/GA_TIERED_LINK_ABLATION_20260322_023216.md](../RUN/GA/GA_TIERED_LINK_ABLATION_20260322_023216.md)

这轮是在 `协方差 + 链路质量 + existence confidence` 的基础上，进一步做一个很弱的 structure-aware decoupled KLA：

- spatial 分支保留主要收益
- existence 分支只做很轻的结构调制，避免破坏 cardinality
- 结构先验同时参考局部子图重叠和固定分档丢包率

推荐参数是：

```matlab
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
model.adaptiveFusion.useNIS = false;
```

`5 trial` 结果：

- `+link quality`: `OSPA 1.877771`, `RMSE 1.800945`, `Cardinality 0.245250`
- `+structure-aware decoupled KLA`: `OSPA 1.863592`, `RMSE 1.749731`, `Cardinality 0.244500`

结论：

- 这是当前 tiered 通信配置下的最新 best
- 相比 `+link quality`，它继续同时改善三项 consensus 指标
- 相比上一版 `+existence confidence` best，`OSPA` 和 `RMSE` 继续下降，`Cardinality` 持平
- 当前有效配置的关键不是“强结构先验”，而是“在 existence baseline 上叠加很弱的 structure-aware decoupling”

## 7. 推荐口径

如果后续文档、实验或论文需要统一通信设置，建议优先使用分档异构口径：

```matlab
commConfig.level = 2;
commConfig.globalMaxMeasurementsPerStep = 80;
commConfig.sensorWeights = ones(1, numberOfSensors) / numberOfSensors;
commConfig.priorityPolicy = 'weightedPriority';
commConfig.measurementSelectionPolicy = 'random';
commConfig.linkModel = 'fixed';
commConfig.pDrop = 0.2;
commConfig.pDropLevels = [0, 0.1, 0.2, 0.5];
commConfig.pDropLevelCounts = [1, 4, 1, 2];
commConfig.maxOutageNodes = 1;
```

推荐理由：

- 与历史 `pDrop=0.2` 的总体强度可对齐
- 对 `linkQuality` 更敏感
- 更容易解释实验现象
- 更方便后续扩展到“档位个数变化”或“极差节点个数变化”的对比实验

如果后续要在这套通信配置下给出当前最佳动态权重组合，建议优先采用：

```matlab
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
model.adaptiveFusion.useNIS = false;
```

对应主报告为：

- [RUN/GA/GA_TIERED_LINK_ABLATION_20260322_023216.md](../RUN/GA/GA_TIERED_LINK_ABLATION_20260322_023216.md)

这套组合当前对应的是：

- `协方差`：状态精度
- `链路质量`：通信可靠性
- `存在性置信度`：目标存在性/基数判决可信度
- `弱结构先验解耦`：对 spatial 分支做主要 refinement，并只对 existence 分支做轻微调制
