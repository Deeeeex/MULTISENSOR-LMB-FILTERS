# AdaptiveFusionWeights (GA/AA)

本文档说明基于自适应融合权重分配的 GA/AA 融合方案，以及当前代码中的实现入口与配置方式。

## 1. 作用范围

该机制只影响 GA/AA 融合权重，作用在多传感器并行更新流程中：

- 权重计算：`multisensorLmb/computeAdaptiveFusionWeights.m`
- 融合入口：`multisensorLmb/runParallelUpdateLmbFilter.m`
- 使用方式示例：`runMultisensorFilters.m`

当 `model.lmbParallelUpdateMode` 为 `'GA'` 或 `'AA'` 且开启自适应权重时，系统在每个时间步更新：

- `model.gaSensorWeights`
- `model.aaSensorWeights`

## 2. 当前权重构成

权重使用以下因子构造可靠性分数 `q_s`：

1. **协方差水平（covScore）**  
   - 由每个传感器的 measurement-updated distributions 计算  
   - 使用 m-projection 的协方差 trace，trace 越小权重越大

2. **链路质量（linkQuality）**  
   - 来自通信层 `commStats` 的丢包统计  
   - `linkQuality = delivered / (delivered + dropped)`

3. **创新一致性（innovationScore）**  
   - 当前为占位（默认全 1）  
   - 预留 `commStats.innovationConsistency` 接口  
   - TODO: 用 NIS 等统计替换占位实现

最终分数：

```
rawScore = covScore .* innovationScore .* linkQuality
```

然后归一化，并使用 EMA 做时间平滑：

```
weights = emaAlpha * prevWeights + (1 - emaAlpha) * rawWeights
```

可设置 `minWeight` 限制最小权重，避免单节点被完全压制。

## 3. 配置方式

在模型上配置 `adaptiveFusion` 字段即可启用：

```matlab
model.adaptiveFusion = struct();
model.adaptiveFusion.enabled = true;
model.adaptiveFusion.emaAlpha = 0.7;
model.adaptiveFusion.minWeight = 0.05;
```

默认参数位于 `multisensorLmb/computeAdaptiveFusionWeights.m`，未设置时使用：

- `emaAlpha = 0.7`
- `minWeight = 0.0`

## 4. commStats 依赖

为了使用链路质量因子，需要传入 `commStats`：

- `commStats.droppedByBandwidth`
- `commStats.droppedByLink`
- `commStats.droppedByOutage`

当前入口脚本已传递 `commStats`：

- `runMultisensorFilters.m`
- `runAdvancedPerformanceAnalysis.m`

## 5. 创新一致性占位说明

当前创新一致性为占位逻辑，保留接口但未实现 NIS 统计：

```matlab
% TODO: Replace with NIS-based consistency when available.
innovationScore = ones(1, numSensors);
```

后续接入方案建议：

1. 在 `multisensorLmb/generateLmbSensorAssociationMatrices.m` 中计算 NIS
2. 将 NIS 汇总写入 `commStats.innovationConsistency`
3. 在 `computeAdaptiveFusionWeights.m` 中替换占位逻辑

## 6. 注意事项

- 仅 GA/AA 使用该权重，PU 模式不受影响
- 权重更新发生在每个时间步的测量更新之后、轨迹融合之前
- 若 `rawScore` 全为 0，将退化为均匀权重
