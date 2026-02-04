# NIS 实现与效果分析

本文整理当前工程内 NIS (Normalized Innovation Squared) 的实现细节，并说明为什么在 GA 中效果可能不稳定或增益有限。

## 当前实现（以代码为准）

核心文件：`multisensorLmb/generateLmbSensorAssociationMatrices.m`

对每个传感器 s、每个时刻 t：

1) 逐量测计算创新与 NIS

- 对每个目标 i、每个量测 k：
  - 预测量测均值 muZ（考虑移动传感器）
  - 创新向量 nu = z_k - muZ
  - 创新协方差 Z = C * P * C' + Q
  - NIS = nu' * Z^{-1} * nu

2) 量测级聚合

- 对每个量测 k，取所有目标的最小 NIS：
  - nisMin(k) = min_i NIS_i(k)

3) 传感器级聚合

- 对所有 nisMin(k) 聚合得到 nisAgg：
  - GA 且 robustNIS=on：取 median
  - 其他情况：取 mean

4) 映射为 [0, 1] 的创新一致性分数

- robust GA 模式：
  - dof = zDimension
  - nisNorm = nisAgg / dof
  - score = exp(-0.5 * nisNorm)
  - 再夹紧到 [robustNISMin, robustNISMax]
- 非 robust 模式：
  - score = 1 / (1 + nisAgg)

5) 融合权重中使用

核心文件：`multisensorLmb/computeAdaptiveFusionWeights.m`

- innovationScore 注入到 `commStats.innovationConsistency`
- 最终 rawScore：
  - rawScore = covScore * innovationScore * linkQuality
- 再经过 EMA 与 minWeight 归一化。

## 为什么 NIS 理论上有效

在高斯量测模型下，NIS 近似服从自由度为量测维度的卡方分布。创新一致性好时，NIS 应落在合理区间；当量测异常、误配或杂波较多时，NIS 会显著增大。将 NIS 作为“创新一致性评分”，就相当于降低不可靠传感器对融合的影响。

## 为什么 GA 上增益可能不明显或不稳定

1) NIS 只影响融合权重，不改变局部更新
   - 局部估计若已偏离，权重重分配的修正能力有限。

2) 量测级 min 聚合偏乐观
   - 高杂波/高密度目标情况下，nisMin 可能过小，无法反映整体质量。

3) GA 对权重波动较敏感
   - NIS 与协方差同时波动会导致权重抖动，可能让 RMSE 反而变差。

4) Robust 映射较保守
   - clamp 限制了 NIS 影响幅度，提高稳定性但降低了区分度。

## 实践上的结论

- 当某些传感器持续“创新不一致”时，NIS 有帮助。
- 在 GA 中，NIS 的影响容易被协方差与链路质量掩盖。
- 若 RMSE 恶化，多数情况是权重抖动或 nisMin 聚合偏乐观导致。

## 可选改进方向（未实现）

- 使用更高分位数（如 0.7）替代 min/mean 聚合，以反映整体一致性。
- 仅用“确认轨迹”的量测计算 NIS，剔除纯杂波量测。
- 对 nisAgg 做时间平滑，降低权重抖动。

