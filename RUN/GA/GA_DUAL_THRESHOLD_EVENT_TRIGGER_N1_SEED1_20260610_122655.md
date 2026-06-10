# 双阈值多指标事件触发 GA-LMB 研究报告

- Trials: 1
- Seeds: `2`
- 仿真长度: 100
- 平均逐节点丢包率: `[0.1 0.1 0.2 0.1 0 0.5 0.5 0.1]`

本实验验证的组合是多指标耦合、链路门控降级和轻/重两级 LMB 后验负载；事件触发 LMB 与目标级信息增益触发已有文献先例，不作为单独新颖性主张。

- TSP 2022: https://doi.org/10.1109/TSP.2022.3154227
- TAES 2023: https://doi.org/10.1109/TAES.2022.3187390
- TCSII 2023: https://doi.org/10.1109/TCSII.2023.3238346

## 单试验全通信阈值标定

| Criterion | Loose | Default | Strict | Samples |
|:--|:--|:--|:--|--:|
| Multi-indicator | `[0.3004 0.3503]` | `[0.3236 0.3708]` | `[0.3413 0.3945]` | 3200 |
| Information gain | `[0.4222 0.4912]` | `[0.4527 1]` | `[0.4719 1]` | 3200 |

## 通信与性能结果

| Arm | Scalars | Bytes | Trigger | Light | Heavy | Delivery | Downgrades | Edges | Alg. conn. | Runtime (s) | Local E-OSPA | Local RMSE | Consensus OSPA | Position disagreement | Card. dispersion |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Periodic full posterior | 4097938 | 32783504 | 1.000 | 0.000 | 1.000 | 0.805 | 0.0 | 16.0 | 2.000 | 448.501 | 2.2289 | 6.7555 | 2.0437 | 6.8172 | 0.2275 |
| Periodic full posterior + dynamic topology | 3519674 | 28157392 | 1.000 | 0.000 | 1.000 | 0.835 | 0.0 | 16.0 | 2.326 | 335.276 | 2.1804 | 4.3686 | 2.1665 | 5.4522 | 0.2275 |
| Light-floor dual threshold | 1792346 | 14338768 | 0.978 | 0.911 | 0.068 | 0.805 | 99.0 | 16.0 | 2.000 | 298.152 | 2.2455 | 7.2215 | 2.0861 | 7.3563 | 0.2300 |
| Light-floor dual threshold + dynamic topology | 1797343 | 14378744 | 0.979 | 0.887 | 0.092 | 0.810 | 140.0 | 16.0 | 2.051 | 368.704 | 2.2183 | 6.3033 | 2.0636 | 6.2174 | 0.2425 |
| Light-floor + dynamic topology (no static bonus) | 1998195 | 15985560 | 0.974 | 0.875 | 0.099 | 0.840 | 121.0 | 16.0 | 2.270 | 370.262 | 2.3187 | 7.6062 | 2.2701 | 7.3753 | 0.2700 |
| Light-floor + dynamic topology (no fallback) | 1789056 | 14312448 | 0.976 | 0.896 | 0.080 | 0.809 | 127.0 | 16.0 | 2.046 | 296.895 | 2.2383 | 6.1436 | 2.0866 | 6.3874 | 0.2350 |

## Effective KLA 图诊断

| Arm | Attempted lambda2 | Delivered lambda2 | Window delivered lambda2 | Effective-weight lambda2 | Label conn. violation | Window label violation | Label stale p90 | New-edge no-handshake | Self weight | Light weight | Heavy weight | Weight entropy |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Periodic full posterior | 2.000 | 1.886 | 2.000 | 0.373 | 0.193 | 0.193 | 1.24 | 0.000 | 0.392 | 0.000 | 0.608 | 1.370 |
| Periodic full posterior + dynamic topology | 2.326 | 2.124 | 4.146 | 0.344 | 0.156 | 0.156 | 1.79 | 0.000 | 0.463 | 0.000 | 0.537 | 1.314 |
| Light-floor dual threshold | 1.993 | 1.868 | 1.997 | 0.374 | 0.187 | 0.187 | 1.27 | 1.000 | 0.399 | 0.553 | 0.048 | 1.350 |
| Light-floor dual threshold + dynamic topology | 2.044 | 1.899 | 2.226 | 0.370 | 0.173 | 0.173 | 1.30 | 0.792 | 0.404 | 0.531 | 0.064 | 1.350 |
| Light-floor + dynamic topology (no static bonus) | 2.266 | 1.875 | 4.067 | 0.323 | 0.149 | 0.149 | 1.47 | 0.839 | 0.452 | 0.489 | 0.059 | 1.313 |
| Light-floor + dynamic topology (no fallback) | 2.039 | 1.896 | 2.256 | 0.371 | 0.196 | 0.196 | 1.33 | 0.850 | 0.406 | 0.538 | 0.056 | 1.344 |

## 30% 通信节省升级门槛

| Arm | Bytes reduction | Local E-OSPA change | Consensus OSPA change | Position change | Card. change | Scale up | Failure reasons |
|:--|--:|--:|--:|--:|--:|:--:|:--|
| Periodic full posterior | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0 | payload reduction < 30% |
| Periodic full posterior + dynamic topology | 14.1% | -2.2% | 6.0% | -20.0% | 0.0% | 0 | payload reduction < 30% |
| Light-floor dual threshold | 56.3% | 0.7% | 2.1% | 7.9% | 1.1% | 1 | - |
| Light-floor dual threshold + dynamic topology | 56.1% | -0.5% | 1.0% | -8.8% | 6.6% | 1 | - |
| Light-floor + dynamic topology (no static bonus) | 51.2% | 4.0% | 11.1% | 8.2% | 18.7% | 0 | OSPA disagreement degradation > 10%; cardinality dispersion degradation > 10% |
| Light-floor + dynamic topology (no fallback) | 56.3% | 0.4% | 2.1% | -6.3% | 3.3% | 1 | - |

## Pareto 筛选

按估算字节数与 local E-OSPA 的二维非支配关系，当前 Pareto arms 为：`Periodic full posterior + dynamic topology`, `Light-floor dual threshold + dynamic topology`, `Light-floor + dynamic topology (no fallback)`。
