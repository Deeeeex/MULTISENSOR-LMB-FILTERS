# 双阈值多指标事件触发 GA-LMB 研究报告

- Trials: 50
- Seeds: `[32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63 64 65 66 67 68 69 70 71 72 73 74 75 76 77 78 79 80 81]`
- 仿真长度: 100
- 平均逐节点丢包率: `[0.18 0.226 0.206 0.206 0.188 0.19 0.184 0.22]`

本实验验证的组合是多指标耦合、链路门控降级和轻/重两级 LMB 后验负载；事件触发 LMB 与目标级信息增益触发已有文献先例，不作为单独新颖性主张。

- TSP 2022: https://doi.org/10.1109/TSP.2022.3154227
- TAES 2023: https://doi.org/10.1109/TAES.2022.3187390
- TCSII 2023: https://doi.org/10.1109/TCSII.2023.3238346

## 单试验全通信阈值标定

| Criterion | Loose | Default | Strict | Samples |
|:--|:--|:--|:--|--:|
| Multi-indicator | `[0.2968 0.3568]` | `[0.3288 0.3791]` | `[0.3486 0.4]` | 3200 |
| Information gain | `[0.4324 0.4814]` | `[0.4527 0.5505]` | `[0.4726 1]` | 3200 |

## 通信与性能结果

| Arm | Scalars | Bytes | Trigger | Light | Heavy | Delivery | Downgrades | Edges | Alg. conn. | Runtime (s) | Local E-OSPA | Local RMSE | Consensus OSPA | Position disagreement | Card. dispersion |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Periodic full posterior | 3471399 | 27771195 | 1.000 | 0.000 | 1.000 | 0.800 | 0.0 | 16.0 | 2.000 | 14.864 | 2.0652 | 4.6215 | 1.9501 | 4.7577 | 0.2026 |
| Periodic full posterior + dynamic topology | 3683182 | 29465456 | 1.000 | 0.000 | 1.000 | 0.835 | 0.0 | 16.0 | 2.173 | 16.610 | 2.1508 | 4.6678 | 2.1667 | 5.0640 | 0.2113 |
| Periodic light posterior on static topology | 1437917 | 11503339 | 1.000 | 1.000 | 0.000 | 0.800 | 0.0 | 16.0 | 2.000 | 15.322 | 2.0652 | 4.6215 | 1.9501 | 4.7577 | 0.2026 |
| Periodic light posterior + guarded dynamic topology | 1446243 | 11569943 | 1.000 | 1.000 | 0.000 | 0.802 | 0.0 | 16.0 | 2.038 | 15.655 | 2.0665 | 4.7272 | 1.9643 | 4.9806 | 0.2039 |

## Effective KLA 图诊断

| Arm | Attempted lambda2 | Delivered lambda2 | Window delivered lambda2 | Effective-weight lambda2 | Label conn. violation | Window label violation | Label stale p90 | Label stale p95 | Topology churn | New-edge no-handshake | Handshake count | Handshake byte share | Self weight | Light weight | Heavy weight | Weight entropy |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Periodic full posterior | 2.000 | 1.889 | 1.999 | 0.373 | 0.154 | 0.154 | 1.22 | 2.45 | 0.000 | 0.000 | 0.0 | 0.000 | 0.395 | 0.000 | 0.605 | 1.363 |
| Periodic full posterior + dynamic topology | 2.173 | 1.939 | 3.952 | 0.301 | 0.173 | 0.172 | 1.44 | 4.04 | 0.341 | 0.000 | 0.0 | 0.000 | 0.481 | 0.000 | 0.519 | 1.286 |
| Periodic light posterior on static topology | 2.000 | 1.889 | 1.999 | 0.373 | 0.154 | 0.154 | 1.22 | 2.45 | 0.000 | 1.000 | 0.0 | 0.000 | 0.395 | 0.605 | 0.000 | 1.363 |
| Periodic light posterior + guarded dynamic topology | 2.038 | 1.912 | 2.189 | 0.371 | 0.156 | 0.156 | 1.22 | 2.48 | 0.038 | 1.000 | 0.0 | 0.000 | 0.401 | 0.599 | 0.000 | 1.358 |

## 30% 通信节省升级门槛

| Arm | Bytes reduction | Local E-OSPA change | Consensus OSPA change | Position change | Card. change | Scale up | Failure reasons |
|:--|--:|--:|--:|--:|--:|:--:|:--|
| Periodic full posterior | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0 | payload reduction < 30% |
| Periodic full posterior + dynamic topology | -6.1% | 4.1% | 11.1% | 6.4% | 4.3% | 0 | payload reduction < 30%; OSPA disagreement degradation > 10% |
| Periodic light posterior on static topology | 58.6% | 0.0% | 0.0% | 0.0% | 0.0% | 1 | - |
| Periodic light posterior + guarded dynamic topology | 58.3% | 0.1% | 0.7% | 4.7% | 0.6% | 1 | - |

## Paired held-out delta summary

每个 trial 与同 seed 的 `Periodic full posterior` 成对比较。Bytes 为降幅，其他 delta 为退化百分比；pass count 使用 30%/5%/10%/10%/10% 主 gate。

| Arm | Bytes mean | Bytes median | Bytes worst | Local mean +- SE | Local p90 | Local worst | Consensus mean +- SE | Consensus p90 | Consensus worst | Pass count |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Periodic full posterior | 0.0% | 0.0% | 0.0% | 0.0 +- 0.0% | 0.0% | 0.0% | 0.0 +- 0.0% | 0.0% | 0.0% | 0/50 |
| Periodic full posterior + dynamic topology | -6.6% | -5.5% | -46.6% | 4.2 +- 0.5% | 8.9% | 11.0% | 11.2 +- 0.8% | 17.0% | 24.9% | 0/50 |
| Periodic light posterior on static topology | 58.5% | 58.7% | 55.2% | 0.0 +- 0.0% | 0.0% | 0.0% | 0.0 +- 0.0% | 0.0% | 0.0% | 50/50 |
| Periodic light posterior + guarded dynamic topology | 58.3% | 58.2% | 53.5% | 0.1 +- 0.2% | 1.4% | 2.8% | 0.7 +- 0.1% | 2.0% | 3.0% | 35/50 |

## Pareto 筛选

按估算字节数与 local E-OSPA 的二维非支配关系，当前 Pareto arms 为：`Periodic light posterior on static topology`。
