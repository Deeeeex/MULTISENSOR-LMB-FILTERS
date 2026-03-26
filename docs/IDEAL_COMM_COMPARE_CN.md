# 理想通信条件下普通 GA 与 Structure-Aware KLA 对比

记录日期：2026-03-26  
脚本：`RUN/GA/runMultisensorFilters_formation_4plus4_IdealCommCompare.m`  
报告：`RUN/GA/GA_IDEAL_COMM_COMPARE_20260326_184508.md`

## 目的

这一组实验回答两个问题：

1. 在没有通信退化时，`structure-aware decoupled KLA` 相对普通 `GA` 是否仍然有效
2. 改善是否只体现在一致性指标上，还是也能体现在部分局部跟踪指标上

## 口径

- 场景：`4+4` distributed formation
- 过滤器：`GA-LMB`
- 数据关联：`LBP`
- 通信：`ideal`
- 具体配置：
  - `commConfig.level = 0`
  - `commConfig.pDrop = 0`
  - `commConfig.pDropBySensor = zeros(1, 8)`
- 对比对象：
  - `ordinary GA`
  - `structure-aware decoupled KLA`

当前 `structure-aware` 使用的配置为：

```matlab
model.adaptiveFusion.useCovariance = true;
model.adaptiveFusion.useLinkQuality = true;
model.adaptiveFusion.useExistenceConfidence = true;
model.adaptiveFusion.existenceConfidenceMinScore = 0.85;
model.adaptiveFusion.existenceConfidencePower = 2.0;
model.adaptiveFusion.useDecoupledKla = true;
model.adaptiveFusion.useStructureAwareKla = true;
model.adaptiveFusion.usePosteriorStructureConsistency = false;
model.adaptiveFusion.spatialDecouplingStrength = 0.5;
model.adaptiveFusion.existenceDecouplingStrength = 0.15;
model.adaptiveFusion.spatialStructureStrength = 0.45;
model.adaptiveFusion.existenceStructureStrength = 0.08;
model.adaptiveFusion.structureReliabilityPower = 0.30;
model.adaptiveFusion.useNIS = false;
```

## 5 Trial 结果

一致性指标：

- `Consensus OSPA`: `1.705549 -> 1.494474`
- `Consensus RMSE`: `1.525900 -> 1.289643`
- `Consensus Cardinality`: `0.160500 -> 0.139000`

局部指标（传感器平均）：

- `Local E-OSPA`: `1.949511 -> 1.876801`
- `Local H-OSPA`: `0.499996 -> 0.499994`
- `Local RMSE`: `1.441872 -> 1.369361`

各传感器局部指标：

| Sensor | E-OSPA (GA) | E-OSPA (SA) | H-OSPA (GA) | H-OSPA (SA) | RMSE (GA) | RMSE (SA) |
|:------:|------------:|------------:|------------:|------------:|----------:|----------:|
| 1 | 2.061 | 1.897 | 0.500 | 0.500 | 1.460 | 1.376 |
| 2 | 2.041 | 1.928 | 0.500 | 0.500 | 1.445 | 1.370 |
| 3 | 1.907 | 1.908 | 0.500 | 0.500 | 1.411 | 1.345 |
| 4 | 1.898 | 1.872 | 0.500 | 0.500 | 1.438 | 1.360 |
| 5 | 1.948 | 1.878 | 0.500 | 0.500 | 1.457 | 1.383 |
| 6 | 1.919 | 1.851 | 0.500 | 0.500 | 1.444 | 1.372 |
| 7 | 1.911 | 1.843 | 0.500 | 0.500 | 1.425 | 1.367 |
| 8 | 1.912 | 1.839 | 0.500 | 0.500 | 1.455 | 1.382 |

## 结论

1. 在理想通信条件下，`structure-aware decoupled KLA` 相对普通 `GA` 仍然保持明显优势，不是只在丢包场景下才有效
2. 优势不只体现在一致性三项上，局部 `E-OSPA` 和局部 `RMSE` 也有改善
3. 局部 `H-OSPA` 基本持平，说明方法没有靠牺牲集合层面的稳定性换取其他指标
4. 因此，这个方法的收益不应只解释为通信补偿，它也改变了理想通信下的分布式融合质量

## 论文建议口径

- 主线仍然放在 `tiered heterogeneous packet loss`
- 理想通信结果更适合作为 supporting evidence：
  - 证明方法不是只针对丢包补丁
  - 证明一致性提升之外，局部 `E-OSPA / RMSE` 也没有退化
