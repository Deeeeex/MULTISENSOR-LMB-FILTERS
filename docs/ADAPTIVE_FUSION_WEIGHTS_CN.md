# Adaptive Fusion Weights (GA/AA)

本文档说明当前工程中自适应融合权重的实现结构、配置项以及与 NIS / 历史稳定性的关系。

## 1. 作用范围

当前自适应权重仅作用于 `GA` / `AA` 两类并行更新融合模式，入口如下：

- `multisensorLmb/computeAdaptiveFusionWeights.m`
- `multisensorLmb/runParallelUpdateLmbFilter.m`
- `multisensorLmb/generateLmbSensorAssociationMatrices.m`

当满足以下条件时，系统会在每个时刻动态更新融合权重：

- `model.lmbParallelUpdateMode` 为 `'GA'` 或 `'AA'`
- `model.adaptiveFusion.enabled = true`

## 2. 当前权重模型

当前实现已整理为显式的分层乘积结构：

```text
w_j(t) ∝ mask_j(t) * cov_j(t) * nis_j(t) * hist_j(t) * link_j(t)
```

其中：

- `mask_j(t)`：可用性掩码
  - 当前默认全 1
  - 未来可通过 `commStats.fusionMask` / `commStats.activeMask` 或 `model.adaptiveFusion.staticMask` 显式置零
- `cov_j(t)`：协方差可靠性分数
  - 由 measurement-updated Bernoulli 分布做 m-projection 后取协方差 trace
  - trace 越小，分数越高
- `nis_j(t)`：创新一致性分数
  - 由 NIS 统计得到
- `hist_j(t)`：历史稳定性分数
  - 惩罚近期波动较大的节点
- `link_j(t)`：链路质量分数
  - 由通信层丢包统计计算

最终原始分数为：

```text
rawScore_j(t) = mask_j(t) * cov_j(t) * nis_j(t) * hist_j(t) * link_j(t)
```

再经过归一化与时间 EMA 平滑得到最终融合权重。

## 3. 各因子说明

### 3.1 协方差分数 `covScore`

实现位置：`multisensorLmb/computeAdaptiveFusionWeights.m`

对每个传感器的 measurement-updated LMB 做 m-projection，得到协方差 `T`，并用：

```text
covScore = 1 / (eps + mean(trace(T)))
```

衡量当前空间分布的集中程度。

### 3.2 创新一致性分数 `innovationScore`

实现位置：

- `multisensorLmb/generateLmbSensorAssociationMatrices.m`
- `multisensorLmb/runParallelUpdateLmbFilter.m`

NIS 的主要流程：

1. 对每个量测计算 `NIS = nu' * Z^{-1} * nu`
2. 对每个量测取跨目标最小 NIS
3. 对该传感器全部量测做聚合，默认使用 `0.7` 分位数
4. 映射到 `[0, 1]` 分数

当前支持：

- `useNIS`
- `robustNIS`
- `robustNISMin`
- `robustNISMax`
- `nisQuantileEnabled`
- `nisQuantile`
- `nisEmaEnabled`
- `nisEmaAlpha`

其中 `nisEmaEnabled` 会在时间维度对创新一致性做平滑，降低权重抖动。

### 3.3 历史稳定性分数 `historyScore`

实现位置：`multisensorLmb/computeAdaptiveFusionWeights.m`

历史稳定性用于惩罚“近期不稳定”的节点。当前实现基于以下三类变化量：

- 协方差分数变化
- 创新一致性分数变化
- 期望基数变化

定义瞬时不稳定度：

```text
instability_j(t) =
    a * covDiff_j(t) +
    b * innovationDiff_j(t) +
    c * cardinalityDiff_j(t)
```

其中 `a,b,c` 由配置项控制。随后对不稳定度做时间 EMA：

```text
instabilityEma_j(t) =
    alpha * instabilityEma_j(t-1) +
    (1 - alpha) * instability_j(t)
```

最后映射为历史稳定性分数：

```text
historyScore_j(t) = exp(-scale * instabilityEma_j(t))
```

并夹紧到 `[historyMinScore, historyMaxScore]`。

### 3.4 链路质量分数 `linkQuality`

实现位置：`multisensorLmb/computeAdaptiveFusionWeights.m`

基于通信统计计算：

```text
linkQuality = delivered / (delivered + dropped)
```

需要 `commStats` 提供：

- `droppedByBandwidth`
- `droppedByLink`
- `droppedByOutage`

## 4. 时间平滑

当前有两层时间平滑：

1. `innovationConsistency` 的时间平滑
   - 配置项：`nisEmaEnabled`, `nisEmaAlpha`
2. 最终融合权重的时间平滑
   - 配置项：`emaAlpha`

这两层分别控制：

- NIS 因子本身的抖动
- 最终权重的抖动

## 5. 默认配置

当前建议默认值如下：

```matlab
model.adaptiveFusion.enabled = true;
model.adaptiveFusion.emaAlpha = 0.7;
model.adaptiveFusion.minWeight = 0.05;

model.adaptiveFusion.useNIS = true;
model.adaptiveFusion.robustNIS = true;
model.adaptiveFusion.robustNISMin = 0.3;

model.adaptiveFusion.nisQuantileEnabled = true;
model.adaptiveFusion.nisQuantile = 0.7;
model.adaptiveFusion.nisEmaEnabled = true;
model.adaptiveFusion.nisEmaAlpha = 0.7;

model.adaptiveFusion.useHistory = true;
model.adaptiveFusion.historyEmaAlpha = 0.8;
model.adaptiveFusion.historyScale = 2.0;
model.adaptiveFusion.historyMinScore = 0.4;
model.adaptiveFusion.historyCovWeight = 0.4;
model.adaptiveFusion.historyInnovationWeight = 0.4;
model.adaptiveFusion.historyCardinalityWeight = 0.2;
```

## 6. 回滚方式

如果需要快速回滚到较早版本的行为，可在脚本中关闭新增模块：

```matlab
model.adaptiveFusion.nisQuantileEnabled = false;
model.adaptiveFusion.nisEmaEnabled = false;
model.adaptiveFusion.useHistory = false;
```

这样会退回到：

- NIS 使用原有 mean / median 聚合
- 不对 NIS 做时间平滑
- 不使用历史稳定性惩罚

## 7. 当前局限

当前实现仍有几个明显限制：

- `mask` 机制已预留，但尚未与事件触发 / 动态拓扑真正联动
- 历史稳定性目前只使用局部统计变化，尚未显式引入异常检测或跨节点相关性
- NIS 仍然基于“量测最小 NIS”构造，面对高杂波或高密度目标时可能偏乐观

因此，下一阶段更合理的方向是：

1. 将事件触发与拓扑重构显式写入 `mask`
2. 继续验证 `historyScore` 的稳定性收益
3. 再决定是否进一步升级 NIS 的统计构造

## 8. 当前实验结论（2026-03-09）

在 `RUN/GA/runMultisensorFilters_formation_4plus4_NISCompare.m` 中，基于以下配置做了 20 次 Monte Carlo 对比：

- `useHistory = true`
- `nisQuantileEnabled = true`
- `nisQuantile = 0.7`
- `nisEmaEnabled = true`
- `nisEmaAlpha = 0.7`

对比顺序：

- `w/o NIS`
- `robust NIS`
- `NIS`

一致性指标均值结果为：

```text
Comprehensive (OSPA) consensus: 1.814 -> 1.811 -> 1.843
Position (RMSE) consensus:      3.158 -> 3.119 -> 3.239
Cardinality consensus:          0.215 -> 0.214 -> 0.217
```

当前可以先得出两个工程结论：

1. `robust NIS` 在现有动态权重框架下有小幅稳定增益，可以作为 GA 中更合理的默认 NIS 形式
2. 普通 `NIS` 即使叠加分位数聚合与时间平滑，整体仍不如 `robust NIS` 稳定

因此，后续如果继续做 GA 方向实验，建议优先保留：

- `useHistory = true`
- `robustNIS = true`
- `nisQuantileEnabled = true`
- `nisEmaEnabled = true`

而不是继续把普通 `NIS` 作为默认主方案。

## 9. History-only 对比结果（2026-03-09）

为单独评估 `historyScore` 的边际作用，新增了脚本：

- `RUN/GA/runMultisensorFilters_formation_4plus4_HistoryCompare.m`

该对比固定：

- `useNIS = false`

仅比较：

- `w/o history`
- `history`

20 次 Monte Carlo 的一致性指标均值为：

```text
Comprehensive (OSPA) consensus: 1.811 -> 1.814
Position (RMSE) consensus:      3.173 -> 3.158
Cardinality consensus:          0.214 -> 0.215
```

逐传感器均值结果也基本一致：

- 大多数传感器在开启 `history` 后，E-OSPA 没有明显改善
- RMSE 只有非常有限的变化，整体收益较弱

当前可以得到一个更明确的工程判断：

1. `history` 单独使用时，并不是强增益项
2. 它更像一个轻量的时间稳定化约束，而不是决定性提升来源
3. 当前这套定义下，`history` 的主要价值更可能是在与 `robust NIS` 组合时提供额外稳定性，而不是独立拉升指标

对应报告文件：

- `RUN/GA/GA_HISTORY_COMPARE_20260309_113545.md`
