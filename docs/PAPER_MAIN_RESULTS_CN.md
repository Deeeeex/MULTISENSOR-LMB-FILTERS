# 当前论文主结果

## 证据口径

当前正文把证据分成两类：

- seeds：`2:51`
- Table 1--3 是完整 operating-point comparison
- PD/FID-FIA direct baselines 来自同场景、同 seeds 的独立 baseline probes
- Table 4 的核心组件消融来自 `GA_RIGOROUS_COMPONENT_ABLATION_N50_SEED1_20260605_173327.md`
- Cardinality-critical 来自 `GA_FID_FIA_EXISTENCE_NO_STABILIZATION_N50_SEED1_20260608_005220.md`
- Balanced 和 Cardinality-critical 使用当前无稳定化配置
- 主场景：8-sensor dual-formation GA-LMB，tiered heterogeneous packet loss
- Balanced 和 Cardinality-critical 均关闭所有 EMA 和 final-weight floor

旧 ideal-communication、communication-level 和 AA 结果使用了已经弃用的稳定化配置，不再作为当前方法的主证据。

## Table 1：主方法 Network disagreement

| Arm | OSPA | Loc. disagreement | Card. dispersion |
|---|---:|---:|---:|
| Fixed Metropolis | 2.383 ± 0.176 | 2.263 ± 0.340 | 0.650 ± 0.223 |
| PD-weighted GA | 2.177 ± 0.161 | 1.995 ± 0.233 | 0.588 ± 0.170 |
| FID-FIA-weighted GA | 1.818 ± 0.056 | 1.643 ± 0.109 | 0.123 ± 0.023 |
| Balanced mode | **1.696 ± 0.046** | **1.461 ± 0.053** | 0.095 ± 0.025 |
| Cardinality-critical mode | 1.713 ± 0.049 | 1.590 ± 0.167 | **0.062 ± 0.016** |

Balanced 相对 Fixed 的下降：

- OSPA：`28.8%`
- localization disagreement：`35.5%`
- cardinality dispersion：`85.4%`

## Table 2：主方法 Local safeguards

| Arm | E-OSPA | RMSE | CardErr |
|---|---:|---:|---:|
| Fixed Metropolis | 2.781 ± 0.129 | 1.630 ± 0.049 | 1.364 ± 0.260 |
| PD-weighted GA | 2.736 ± 0.116 | **1.563 ± 0.061** | 1.255 ± 0.205 |
| FID-FIA-weighted GA | 2.185 ± 0.050 | 1.734 ± 0.094 | 0.388 ± 0.047 |
| Balanced mode | 2.072 ± 0.051 | 1.636 ± 0.045 | 0.283 ± 0.046 |
| Cardinality-critical mode | **2.030 ± 0.042** | 1.744 ± 0.161 | **0.209 ± 0.026** |

## Table 3：主方法计算量

| Arm | Runtime (s) | Runtime/step (s) | Relative runtime |
|---|---:|---:|---:|
| Fixed Metropolis | 52.123 ± 7.932 | 0.521 | 1.000x |
| PD-weighted GA | 61.668 ± 5.376 | 0.617 | 1.233x |
| FID-FIA-weighted GA | 147.674 ± 23.957 | 1.477 | 2.833x |
| Balanced mode | 56.378 ± 9.626 | 0.564 | 1.082x |
| Cardinality-critical mode | 155.913 ± 18.220 | 1.559 | 2.991x |

## Table 4：组件归因

- Link quality 是当前 packet-loss 场景的主导因素。
- Covariance 有独立作用，并在 link backbone 上进一步改善 cardinality。
- Existence confidence 在当前 backbone 上的独立增量很小。
- Branch decoupling 与 weak structure-aware correction 作为一个联合的 branch-aware refinement：
  - OSPA 改善 `0.0145`，`50/50` trials
  - localization 改善 `0.0202`，`49/50` trials
  - cardinality 基本不变

`Covariance only / Link only / Existence only / Covariance + link` 等诊断臂
只放在 Table 4，不再进入 Table 1--3。

## EMA/floor 决策

相对无稳定化 structure-aware 配置，加入 EMA/floor：

- OSPA 恶化 `3.8%`
- localization disagreement 恶化 `5.1%`
- cardinality dispersion 恶化 `90.1%`
- local E-OSPA：`2.072 -> 2.293`
- local CardErr：`0.283 -> 0.551`
- 只有 local RMSE 改善：`1.636 -> 1.601`

因此 EMA/floor 不再属于推荐方法。Balanced 固定为：

```text
emaAlpha = 0
minWeight = 0
spatialEmaAlpha = 0
existenceEmaAlpha = 0
spatialMinWeight = 0
existenceMinWeight = 0
```

## Cardinality-critical mode

该模式定义为：

- 完整继承无 EMA/floor 的 Balanced mode；
- 仅令 `useFidFiaExistence = true`；
- 只调制 existence branch，不改变 spatial branch；
- 当前 50-trial 结果已经由 `runFidFiaExistenceNoStabilizationValidation(50, 1)` 生成。

相对 Balanced：

- OSPA consensus error：恶化 `1.0%`
- localization disagreement：恶化 `8.9%`
- cardinality dispersion：改善 `34.8%`
- local E-OSPA：改善 `2.0%`
- local RMSE：恶化 `6.6%`
- local CardErr：改善 `26.2%`

因此 Balanced 是 spatial-consensus / lower-cost mode，Cardinality-critical
是 cardinality-sensitive mode。
