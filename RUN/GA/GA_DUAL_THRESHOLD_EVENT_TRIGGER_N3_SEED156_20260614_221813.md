# 双阈值多指标事件触发 GA-LMB 研究报告

- Trials: 3
- Seeds: `[157 158 159]`
- 仿真长度: 50
- 平均逐节点丢包率: `[0.2667 0.2 0.1333 0.2333 0.2 0.1333 0.2 0.2333]`

本实验验证的组合是多指标耦合、链路门控降级和轻/重两级 LMB 后验负载；事件触发 LMB 与目标级信息增益触发已有文献先例，不作为单独新颖性主张。

- TSP 2022: https://doi.org/10.1109/TSP.2022.3154227
- TAES 2023: https://doi.org/10.1109/TAES.2022.3187390
- TCSII 2023: https://doi.org/10.1109/TCSII.2023.3238346

## 单试验全通信阈值标定

| Criterion | Loose | Default | Strict | Samples |
|:--|:--|:--|:--|--:|
| Multi-indicator | `[0.2 0.4]` | `[0.25 0.6]` | `[0.35 0.8]` | 0 |
| Information gain | `[0.2 0.4]` | `[0.25 0.6]` | `[0.35 0.8]` | 0 |

## 通信与性能结果

| Arm | Scalars | Bytes | Attempts | Deliveries | Trigger | Light | Heavy | Delivery | Downgrades | Edges | Alg. conn. | Runtime (s) | Local E-OSPA | Local RMSE | Consensus OSPA | Position disagreement | Card. dispersion |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Periodic full posterior | 1523927 | 12191413 | 1600 | 1600 | 1.000 | 0.000 | 1.000 | 1.000 | 0.0 | 16.0 | 2.000 | 110.481 | 2.2787 | 5.5009 | 1.8411 | 4.8709 | 0.1792 |
| Periodic light posterior on static topology | 618323 | 4946581 | 1600 | 1600 | 1.000 | 1.000 | 0.000 | 1.000 | 0.0 | 16.0 | 2.000 | 79.173 | 2.2787 | 5.5009 | 1.8411 | 4.8709 | 0.1792 |

## Effective KLA 图诊断

| Arm | Attempted lambda2 | Delivered lambda2 | Window delivered lambda2 | Effective-weight lambda2 | Label conn. violation | Window label violation | Label stale p90 | Label stale p95 | Topology churn | New-edge no-handshake | Handshake count | Handshake byte share | Self weight | Light weight | Heavy weight | Weight entropy |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Periodic full posterior | 2.000 | 2.000 | 2.000 | 0.334 | 0.193 | 0.193 | 0.00 | 0.00 | 0.000 | 0.000 | 0.0 | 0.000 | 0.334 | 0.000 | 0.666 | 1.560 |
| Periodic light posterior on static topology | 2.000 | 2.000 | 2.000 | 0.334 | 0.193 | 0.193 | 0.00 | 0.00 | 0.000 | 1.000 | 0.0 | 0.000 | 0.334 | 0.666 | 0.000 | 1.560 |

## Topology risk attribution diagnostics

| Arm | Edge risk | Static risk | Non-static risk | New-edge risk | Label mismatch | Non-static mismatch | Fusion non-static weight | Fusion new-edge weight | Fusion risk-weighted edge risk | Fusion risk-weighted mismatch |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Periodic full posterior | 0.282 | 0.282 | 0.000 | 0.000 | 0.113 | 0.000 | 0.000 | 0.013 | 0.282 | 0.113 |
| Periodic light posterior on static topology | 0.282 | 0.282 | 0.000 | 0.000 | 0.113 | 0.000 | 0.000 | 0.013 | 0.282 | 0.113 |

## 30% 通信节省升级门槛

| Arm | Bytes reduction | Local E-OSPA change | Consensus OSPA change | Position change | Card. change | Scale up | Failure reasons |
|:--|--:|--:|--:|--:|--:|:--:|:--|
| Periodic full posterior | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0 | payload reduction < 30% |
| Periodic light posterior on static topology | 59.4% | 0.0% | 0.0% | 0.0% | 0.0% | 1 | - |

## Paired held-out delta summary

每个 trial 与同 seed 的 `Periodic full posterior` 成对比较。Bytes 为降幅，其他 delta 为退化百分比；pass count 使用 30%/5%/10%/10%/10% 主 gate。

| Arm | Bytes mean | Bytes median | Bytes worst | Local mean +- SE | Local p90 | Local worst | Consensus mean +- SE | Consensus p90 | Consensus worst | Pass count |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Periodic full posterior | 0.0% | 0.0% | 0.0% | 0.0 +- 0.0% | 0.0% | 0.0% | 0.0 +- 0.0% | 0.0% | 0.0% | 0/3 |
| Periodic light posterior on static topology | 59.5% | 58.4% | 58.3% | 0.0 +- 0.0% | 0.0% | 0.0% | 0.0 +- 0.0% | 0.0% | 0.0% | 3/3 |

## Pareto 筛选

按估算字节数与 local E-OSPA 的二维非支配关系，当前 Pareto arms 为：`Periodic light posterior on static topology`。
