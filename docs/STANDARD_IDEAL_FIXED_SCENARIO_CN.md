# 标准 Ideal Fixed 场景实验记录

记录日期：2026-03-27  
主脚本：

- `RUN/IDEAL/runMultisensorFilters_standard_fixed_ideal_compare.m`
- `RUN/IDEAL/runStandardFixedIdealDistributedCompare.m`

主报告：

- `RUN/IDEAL/STANDARD_FIXED_IDEAL_GA_AA_20260327_101436.md`
- `RUN/IDEAL/STANDARD_FIXED_IDEAL_DISTRIBUTED_20260327_110503.md`

相关图：

- `RUN/IDEAL/STANDARD_FIXED_IDEAL_SENSOR1_COMPARE_2.png`

## 1. 目的

这组实验不是复现 `4+4 formation`，而是额外补一个更简单、也更接近文献常用理想设置的标准场景，用来回答三个问题：

1. 在 `4 sensors + ideal communication + uniform weights + Fixed scenario` 下，固定权重 `GA/AA` 的基线表现如何
2. 当前主线最优配置 `current-best adaptive GA` 在这个标准 ideal 场景里是否仍然优于固定权重 `GA`
3. 如果没有明显增益，原因到底是“配置没生效”，还是“这个场景本来就不提供足够的结构收益”

## 2. 场景口径

当前统一使用以下设置：

- `numberOfSensors = 4`
- `scenarioType = Fixed`
- `simulationLength = 100`
- `commConfig.level = 0`
- `commConfig.pDropBySensor = [0 0 0 0]`
- `fusionWeighting = Uniform`
- `clutterRates = [5 5 5 5]`
- `detectionProbabilities = [0.9 0.9 0.9 0.9]`
- `measurementNoiseStd = [3 3 3 3]`

其中 ground truth 仍然沿用仓库原有的标准 `Fixed` benchmark，不是论文中的 `formation` 场景。

## 3. Centralized Ideal 基线

脚本：

- `RUN/IDEAL/runMultisensorFilters_standard_fixed_ideal_compare.m`

这里比较的是 centralized fixed-weight `GA` 与 centralized fixed-weight `AA`。

`5-trial` 结果：

| Filter | E-OSPA | H-OSPA | RMSE | Cardinality Error |
|:------|-------:|-------:|-----:|------------------:|
| GA | 1.931801 | 0.456857 | 1.515751 | 0.324000 |
| AA | 4.184006 | 0.499369 | 5.494451 | 0.198000 |

结论：

- 在这个标准 ideal 场景里，固定权重 `GA` 明显优于固定权重 `AA`
- 因此后续对比 `current-best adaptive GA` 时，主要参考对象应该是固定 `GA`，而不是 `AA`

## 4. 为什么 centralized ideal 下 current-best 几乎不比 fixed GA 强

最开始在 centralized ideal 场景上直接比较 `fixed GA` 与 `current-best adaptive GA`，结果几乎完全重合：

- `Fixed GA`: `E-OSPA 1.931801 / H-OSPA 0.456857 / RMSE 1.515751 / CardErr 0.324000`
- `Current-best adaptive GA`: `E-OSPA 1.932077 / H-OSPA 0.456856 / RMSE 1.517710 / CardErr 0.320000`

表面上看像是“adaptive 没起作用”，但实际排查结果不是这样。

### 4.1 融合权重不是 fixed weights

在同一条 run 上直接打印 `gaWeights` 后，可以看到 adaptive 权重从第 1 步开始就偏离均匀权重：

- `t=1`: `[0.254254 0.247675 0.240896 0.257174]`
- `t=10`: `[0.241047 0.246735 0.263954 0.248264]`
- `t=40`: `[0.249088 0.262242 0.244958 0.243712]`
- `t=100`: `[0.261086 0.243374 0.242464 0.253077]`

全程相对均匀权重 `[0.25 0.25 0.25 0.25]` 的最大偏离约为：

- `max deviation from uniform = 0.03426940 at t = 63`

所以这里不是“adaptive 分支没有生效”。

### 4.2 真正退化的是 structure-aware 先验

在 centralized `runParallelUpdateLmbFilter` 口径下，没有像 distributed runner 那样构造：

- `gaTopologyWeights`
- `gaSpatialStructurePrior`
- `gaExistenceStructurePrior`

因此在这个场景里实际打印出来的是：

- `spatialStructurePrior = [1 1 1 1]`
- `existenceStructurePrior = [1 1 1 1]`
- `linkQuality = [1 1 1 1]`

这意味着 `current-best` 里最关键的结构感知层在 centralized ideal 口径下实际上被抽空了，只剩下轻微的：

- `covariance`
- `existenceConfidence`
- `EMA smoothing`

所以它自然只能表现成“和 fixed GA 很接近”。

## 5. Distributed Ideal 版本

为了让 structure-aware 真的参与进来，这里额外构造了一个 distributed ideal 版本：

- 拓扑：`4-sensor ring`
- `neighborMap = [4 1 2; 1 2 3; 2 3 4; 3 4 1]`
- 通信仍然是 ideal

脚本：

- `RUN/IDEAL/runStandardFixedIdealDistributedCompare.m`

这里比较的是：

- `fixed distributed GA`
- `current-best adaptive distributed GA`

### 5.1 Structure prior 确实不再是全 1

`5-trial` 报告里打印出的结构先验为：

- `Sensor 1 spatial prior: [0.9 1.2 0.9]`
- `Sensor 1 existence prior: [1.071 0.8571 1.071]`
- 其余三个传感器同样是 ring 对称形式

所以这次可以确认：

- 结构先验已经真正进入融合逻辑
- 当前结果不再是“因为 structure-aware 根本没参与”

### 5.2 一致性指标

`5-trial` distributed ideal 的 consensus 结果为：

| Mode | OSPA | RMSE | Card |
|:-----|-----:|-----:|-----:|
| Fixed distributed GA | 1.692102 | 2.363234 | 0.048500 |
| Current-best adaptive distributed GA | 1.701932 | 2.359601 | 0.050500 |

对应结论：

- `Consensus RMSE` 略好
- `Consensus OSPA` 略差
- `Consensus Card` 略差

也就是说，`current-best adaptive GA` 在这个 distributed ideal ring 场景下没有稳定赢过 fixed distributed `GA`。

## 6. 原始 local tracking 指标

除了 consensus 之外，还补跑了各本地传感器相对 ground truth 的 tracking 指标。

`5-trial` 传感器平均结果：

- `Mean local E-OSPA: 2.172506 -> 2.173082`
- `Mean local H-OSPA: 0.475124 -> 0.475164`
- `Mean local RMSE: 1.780430 -> 1.780231`
- `Mean local CardErr: 0.317500 -> 0.316500`

各传感器结果：

| Sensor | E-OSPA (fixed) | E-OSPA (adaptive) | H-OSPA (fixed) | H-OSPA (adaptive) | RMSE (fixed) | RMSE (adaptive) | CardErr (fixed) | CardErr (adaptive) |
|:------:|---------------:|------------------:|---------------:|------------------:|-------------:|----------------:|----------------:|-------------------:|
| 1 | 2.167313 | 2.177490 | 0.475816 | 0.475835 | 1.775288 | 1.776051 | 0.316000 | 0.322000 |
| 2 | 2.178849 | 2.175807 | 0.474925 | 0.475008 | 1.781826 | 1.782754 | 0.322000 | 0.318000 |
| 3 | 2.144002 | 2.145330 | 0.475747 | 0.475750 | 1.772755 | 1.771313 | 0.302000 | 0.300000 |
| 4 | 2.199861 | 2.193700 | 0.474006 | 0.474063 | 1.791851 | 1.790804 | 0.330000 | 0.326000 |

结论：

- 原始 tracking 指标也几乎完全持平
- `RMSE / CardErr` 只有非常轻微的改善
- `E-OSPA / H-OSPA` 基本属于随机波动量级

## 7. 总结

这一组 standard ideal fixed 场景给出的信息比较明确：

1. `current-best adaptive GA` 在 centralized ideal 口径下看起来几乎等于 fixed `GA`，不是因为配置没生效，而是因为 structure prior 在该口径下退化成了全 `1`
2. 把场景改成 distributed ideal ring 之后，structure prior 的确生效了，但 `current-best adaptive GA` 仍然没有在 `5-trial` 平均上稳定优于 fixed distributed `GA`
3. 因此当前主线最优配置更像是：
   - 对 `4+4 formation + heterogeneous communication` 特别有效
   - 但在这个更简单、更标准的 ideal benchmark 上没有展现出稳定优势

## 8. 下一步建议

如果要继续沿这个场景深挖，优先顺序建议如下：

1. 在 `distributed ideal ring` 场景上单独重新调 `spatial/existence structure strength`
2. 重新启用并单独验证 `posterior-structure-consistency` 这条更本质的动态结构分支
3. 不再把 centralized ideal 版本直接称为完整 `current-best` 验证，因为它天然缺失 distributed structure prior
