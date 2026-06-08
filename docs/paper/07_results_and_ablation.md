# Results And Ablation

当前结果章节使用：

- `RUN/GA/GA_RIGOROUS_COMPONENT_ABLATION_N50_SEED1_20260605_173327.md`
- `RUN/GA/GA_FID_FIA_EXISTENCE_NO_STABILIZATION_N50_SEED1_20260608_005220.md`

两者均使用 seeds `2:51` 和当前无 EMA/floor 配置。

## Retained Balanced result

| Metric type | Metric | Mean ± Std |
|---|---|---:|
| Disagreement | OSPA error | **1.695752 ± 0.045525** |
| Disagreement | Localization disagreement | **1.460800 ± 0.053302** |
| Disagreement | Cardinality dispersion | **0.094700 ± 0.025104** |
| Local | E-OSPA | **2.071760 ± 0.051491** |
| Local | RMSE | **1.636237 ± 0.045427** |
| Local | Cardinality error | **0.282850 ± 0.045723** |

该配置等价于严格消融中的 `+ structure-aware`，六个 EMA/floor 参数全部为零。

## Cardinality-critical result

| Metric type | Metric | Mean ± Std |
|---|---|---:|
| Disagreement | OSPA error | **1.713225 ± 0.048967** |
| Disagreement | Localization disagreement | **1.590492 ± 0.167107** |
| Disagreement | Cardinality dispersion | **0.061700 ± 0.015557** |
| Local | E-OSPA | **2.029918 ± 0.042433** |
| Local | RMSE | **1.744456 ± 0.160642** |
| Local | Cardinality error | **0.208750 ± 0.025855** |

## Main interpretation

1. Link quality 提供最大增益，是当前场景的主要证据。
2. Covariance 提供额外的 posterior-concentration 和 cardinality 信号。
3. Existence confidence 的独立增量很小，不能单独宣称为显著性能来源。
4. Branch decoupling 与 structure-aware correction 合并为一个 branch-aware refinement；其空间一致性增益较小，但 paired 方向稳定。
5. Cardinality-critical 在 Balanced 上只增加 existence-only FID-FIA：cardinality dispersion 改善 `34.8%`，local CardErr 改善 `26.2%`，但 localization disagreement 和 local RMSE 分别恶化 `8.9%` 和 `6.6%`。

## Evidence deferred until rerun

以下结果使用旧稳定化配置，不再进入当前主证据链：

- ideal-communication comparison
- communication-level sensitivity
- AA transfer experiment

这些实验后续需要使用无 EMA/floor 的当前配置重新运行。
