# Results And Ablation

当前结果章节使用：

- `RUN/GA/GA_RIGOROUS_COMPONENT_ABLATION_N50_SEED1_20260605_173327.md`
- `RUN/GA/GA_FID_FIA_EXISTENCE_NO_STABILIZATION_N50_SEED1_20260608_005220.md`
- `RUN/GA/mc50_20260527_172137/paper_tables_n50_synthesis.md`

所有结果使用 seeds `2:51` 和同一 tiered heterogeneous packet-loss
场景。Balanced 与 Cardinality-critical 使用当前无 EMA/floor 配置。
PD-weighted GA 和 FID-FIA-weighted GA 来自独立的 direct-baseline probes。

## Main operating-point tables

Table 1--3 只比较完整方法或完整 operating mode：

| Arm | OSPA | Loc. disagreement | Card. dispersion |
|---|---:|---:|---:|
| Fixed Metropolis | 2.383 ± 0.176 | 2.263 ± 0.340 | 0.650 ± 0.223 |
| PD-weighted GA | 2.177 ± 0.161 | 1.995 ± 0.233 | 0.588 ± 0.170 |
| FID-FIA-weighted GA | 1.818 ± 0.056 | 1.643 ± 0.109 | 0.123 ± 0.023 |
| Balanced mode | **1.696 ± 0.046** | **1.461 ± 0.053** | 0.095 ± 0.025 |
| Cardinality-critical mode | 1.713 ± 0.049 | 1.590 ± 0.167 | **0.062 ± 0.016** |

| Arm | E-OSPA | RMSE | CardErr |
|---|---:|---:|---:|
| Fixed Metropolis | 2.781 ± 0.129 | 1.630 ± 0.049 | 1.364 ± 0.260 |
| PD-weighted GA | 2.736 ± 0.116 | **1.563 ± 0.061** | 1.255 ± 0.205 |
| FID-FIA-weighted GA | 2.185 ± 0.050 | 1.734 ± 0.094 | 0.388 ± 0.047 |
| Balanced mode | 2.072 ± 0.051 | 1.636 ± 0.045 | 0.283 ± 0.046 |
| Cardinality-critical mode | **2.030 ± 0.042** | 1.744 ± 0.161 | **0.209 ± 0.026** |

`Covariance only / Link only / Existence only / Covariance + link` 等组件臂
只属于 Table 4 的严格 ablation，不进入主结果和 runtime 表。

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
