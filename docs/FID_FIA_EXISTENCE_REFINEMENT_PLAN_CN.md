# FID-FIA 存在性分支增强计划

## 目标

在不替换当前 `covariance + link quality + existence confidence + weak structure-aware decoupled KLA` 主方法的前提下，引入一个新的混合对比臂：`Cardinality-critical mode`。该方法保留当前方法的 spatial/position 优势，只把 FID-FIA 的信息几何可分辨性信号注入 existence/cardinality 分支。

## 实现设计

- 在 `computeAdaptiveFusionWeights.m` 中新增配置：
  - `useFidFiaExistence`
  - `fidFiaExistenceStrength`，通用默认值为 `0.5`
  - `fidFiaExistencePower`
  - `fidFiaExistenceMinScore`
- 复用现有 FID-FIA 计算路径：posterior m-projection、存在概率加权目标对、检测概率/FoV 感知 Fisher metric、pairwise FID 累积。
- 仅修改 decoupled KLA 的 existence 分支：
  - spatial branch 保持不变；
  - 计算归一化 `fidFiaScore`；
  - 映射为有下界的 `fidFiaExistenceScore`；
  - 用 `existenceScore = existenceScore .* fidFiaExistenceScore^strength` 做几何调制。
- 增加 debug 字段：
  - `debug.useFidFiaExistence`
  - `debug.fidFiaScore`
  - `debug.fidFiaExistenceScore`
  - `debug.fidFiaPairCounts`

## 主实验对比

新增 `finalArmMode='fidFiaExistenceRefinement'`，主实验对比顺序为：

`Fixed Metropolis -> FID-FIA baseline -> Balanced mode -> Cardinality-critical mode`

主实验新增 arm 使用更强的 existence-only FID-FIA 调制：

- `fidFiaExistenceStrength = 4.0`
- `fidFiaExistenceMinScore = 0.0`
- `existenceEmaAlpha = 0.0`
- `existenceMinWeight = 0.0`

这四个设置只作用于新增 arm 的 existence/cardinality 分支；spatial 分支仍沿用当前 structure-aware decoupled KLA 配置。

这样可以直接检验新方法是否同时继承：

- 当前方法的 OSPA consensus error/matched localization disagreement 优势；
- FID-FIA baseline 的 cardinality 一致性优势。

## 验收指标

20-trial 主实验完成后，优先对比 FID-FIA baseline：

- OSPA consensus error < `1.820229`
- matched localization disagreement < `1.647412`
- cardinality dispersion < `0.126188`
- local CardErr <= `0.392313`
- local RMSE 不应差于 FID-FIA baseline 的 `1.715746`

如果所有 consensus 指标都优于 FID-FIA，则可考虑把该混合方法提升为新的 headline 方法；否则保留为 cardinality refinement ablation。

## 20-Trial 结果

结果文件：

- `RUN/GA/GA_TIERED_LINK_ABLATION_N20_SEED1_20260512_155714.md`

| Arm | OSPA Consensus Error | Matched Localization Disagreement | Cardinality Dispersion | Local RMSE | Local CardErr |
| --- | ---: | ---: | ---: | ---: | ---: |
| FID-FIA baseline | 1.820229 | 1.647412 | 0.126188 | 1.715746 | 0.392313 |
| Balanced mode | 1.785873 | 1.562521 | 0.192938 | 1.598561 | 0.578688 |
| Cardinality-critical mode | 1.668961 | 1.528182 | 0.061062 | 1.704538 | 0.221563 |

结论：`Cardinality-critical mode` 在三个 primary network disagreement 指标上都超过 FID-FIA baseline，同时 local CardErr 也更低。它的 local RMSE 不如 Balanced mode，但仍略优于 FID-FIA baseline，因此可以提升为新的主实验 headline 方法。
