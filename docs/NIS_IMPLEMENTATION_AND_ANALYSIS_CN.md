# NIS 实现与效果分析

本文整理当前工程中 NIS 的实现方式，以及为什么在 GA 中其增益有时并不明显。

## 1. 当前实现

核心代码位置：

- `multisensorLmb/generateLmbSensorAssociationMatrices.m`
- `multisensorLmb/runParallelUpdateLmbFilter.m`
- `multisensorLmb/computeAdaptiveFusionWeights.m`

当前流程如下。

### 1.1 逐量测计算 NIS

对每个目标、每个量测，先计算：

```text
nu = z - muZ
Z = C * P * C' + Q
NIS = nu' * Z^{-1} * nu
```

其中：

- `muZ` 为预测量测均值
- `Z` 为创新协方差

### 1.2 量测级聚合

对每个量测，保留跨目标的最小 NIS：

```text
nisMin(k) = min_i NIS_i(k)
```

这一步的含义是：用“最可能解释该量测”的目标来代表该量测的一致性。

### 1.3 传感器级聚合

对当前传感器的全部 `nisMin(k)` 进行聚合，得到 `nisAgg`。

当前默认逻辑：

- `nisQuantileEnabled = true` 时：使用分位数聚合
- 默认 `nisQuantile = 0.7`

当关闭分位数聚合后：

- `GA + robustNIS = true`：使用 `median`
- 其他情况：使用 `mean`

### 1.4 映射到创新一致性分数

若 `robustNIS = true`：

```text
nisNorm = nisAgg / dof
score = exp(-0.5 * nisNorm)
score = clamp(score, robustNISMin, robustNISMax)
```

若 `robustNIS = false`：

```text
score = 1 / (1 + nisAgg)
```

最终得到 `innovationScore`。

### 1.5 时间平滑

在 `runParallelUpdateLmbFilter.m` 中，当前还会对 `innovationScore` 做时间 EMA：

```text
innovation(t) = alpha * innovation(t-1) + (1 - alpha) * innovationRaw(t)
```

默认：

- `nisEmaEnabled = true`
- `nisEmaAlpha = 0.7`

### 1.6 融入自适应权重

在 `computeAdaptiveFusionWeights.m` 中，当前权重结构为：

```text
rawScore = mask * covScore * innovationScore * historyScore * linkQuality
```

因此 NIS 只是动态权重的一部分，并不是单独决定融合结果的唯一因素。

## 2. 为什么 NIS 理论上有效

在高斯量测模型下，NIS 近似服从自由度为量测维度的卡方分布。

因此：

- 当预测与量测匹配良好时，NIS 应落在合理区间
- 当量测异常、误配、杂波较多或模型失配时，NIS 会增大

把 NIS 转成 `[0,1]` 的一致性分数后，就能降低当前“创新不可信”的传感器在融合中的影响。

## 3. 为什么 GA 上效果可能不理想

### 3.1 NIS 只作用于权重，不改变局部更新

当前 NIS 影响的是融合权重，而不是单个传感器的局部滤波更新。

这意味着：

- 如果局部轨迹已经偏了
- 或者局部数据关联本身就存在误配

那么仅靠重分配权重，修正能力是有限的。

### 3.2 量测最小 NIS 容易偏乐观

当前 `nisMin(k)` 取的是“跨目标最小值”。这在高杂波或高密度目标下有一个问题：

- 总有可能某个目标对某个杂波量测给出较小 NIS
- 于是该量测被判定为“看起来还行”

这会让传感器级 NIS 聚合偏乐观。

### 3.3 GA 对权重变化较敏感

GA 融合本身对权重扰动更敏感。如果：

- `covScore`
- `innovationScore`
- `historyScore`

同时波动，那么即使 OSPA 改善，RMSE 也可能被放大。

### 3.4 NIS 容易被其他因子掩盖

当前权重是多个因子的乘积，NIS 的作用会被以下因素共同稀释或放大：

- 协方差分数
- 链路质量
- 历史稳定性

如果这些因子的波动更大，NIS 的边际贡献就不容易直接观察到。

## 4. 当前新增的改进

为减少前述问题，当前已经加入两项增强：

1. `0.7` 分位数聚合
   - 减少极小值对 `nisAgg` 的影响
2. 时间 EMA 平滑
   - 降低 NIS 分数在时间维度的抖动

这两项都支持关闭回滚。

## 4.1 20 次 Monte Carlo 对比结果（2026-03-09）

对比脚本：

- `RUN/GA/runMultisensorFilters_formation_4plus4_NISCompare.m`

本轮对比使用的关键配置：

- `useHistory = true`
- `nisQuantileEnabled = true`
- `nisQuantile = 0.7`
- `nisEmaEnabled = true`
- `nisEmaAlpha = 0.7`
- `robustNIS = true`
- `robustNISMin = 0.3`

对比顺序为：

- `w/o NIS`
- `robust NIS`
- `NIS`

20 次均值结果如下：

```text
Comprehensive (OSPA) consensus: 1.814 -> 1.811 -> 1.843
Position (RMSE) consensus:      3.158 -> 3.119 -> 3.239
Cardinality consensus:          0.215 -> 0.214 -> 0.217
```

从本轮结果看：

- `robust NIS` 相比 `w/o NIS` 有小幅正收益
- 普通 `NIS` 在三项一致性指标上都没有优势
- 即使加入分位数聚合、NIS 时间平滑与历史稳定性，普通 `NIS` 依然偏不稳定

逐传感器均值也支持同样结论：大多数传感器在 `robust NIS` 下的 E-OSPA / RMSE 略优于 `w/o NIS`，而普通 `NIS` 更容易出现回退。

对应报告文件：

- `RUN/GA/GA_NIS_COMPARE_20260309_112550.md`

## 5. 回滚方式

若希望退回到较早的 NIS 行为：

```matlab
model.adaptiveFusion.nisQuantileEnabled = false;
model.adaptiveFusion.nisEmaEnabled = false;
```

这样会退回为：

- NIS 聚合使用原有 `mean / median`
- 不对 NIS 分数做时间平滑

## 6. 下一步更值得验证的方向

相较于继续微调 `mean / median / quantile`，更值得做的方向是：

1. 只对高置信轨迹的关联量测计算 NIS
2. 将 NIS 改为窗口化统计一致性指标，而不是单时刻分数
3. 与历史稳定性做消融实验，确认 NIS 的真实边际收益

因此，当前更合理的研究路径不是继续单独调 NIS，而是放在完整动态权重框架里一起看。
