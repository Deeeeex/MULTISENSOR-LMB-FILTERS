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

| Arm | Scalars | Bytes | Trigger | Light | Heavy | Delivery | Downgrades | Runtime (s) | Local E-OSPA | Local RMSE | Consensus OSPA | Position disagreement | Card. dispersion |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Local only | 0 | 0 | 0.000 | 0.000 | 0.000 | 0.000 | 0.0 | 71.109 | 3.4275 | 4.3896 | 4.0934 | 6.6240 | 0.3250 |
| Periodic full posterior | 4097938 | 32783504 | 1.000 | 0.000 | 1.000 | 0.805 | 0.0 | 299.693 | 2.2289 | 6.7555 | 2.0437 | 6.8172 | 0.2275 |
| Single-threshold information, full payload | 2416559 | 19332472 | 0.609 | 0.000 | 0.609 | 0.809 | 0.0 | 276.628 | 2.4595 | 6.6608 | 2.5601 | 7.7057 | 0.2338 |
| Dual-threshold information | 1981250 | 15850000 | 0.609 | 0.204 | 0.405 | 0.809 | 0.0 | 282.234 | 2.4595 | 6.6608 | 2.5601 | 7.7057 | 0.2338 |
| Multi-indicator without link gate | 1856347 | 14850776 | 0.650 | 0.207 | 0.444 | 0.806 | 0.0 | 259.960 | 2.4464 | 8.1483 | 2.5083 | 8.0337 | 0.2512 |
| Full method (default) | 1721086 | 13768688 | 0.650 | 0.316 | 0.334 | 0.806 | 351.0 | 234.734 | 2.4464 | 8.1483 | 2.5083 | 8.0337 | 0.2512 |

## 30% 通信节省升级门槛

| Arm | Bytes reduction | Local E-OSPA change | Consensus OSPA change | Position change | Card. change | Scale up | Failure reasons |
|:--|--:|--:|--:|--:|--:|:--:|:--|
| Local only | 100.0% | 53.8% | 100.3% | -2.8% | 42.9% | 0 | local E-OSPA degradation > 5%; OSPA disagreement degradation > 10%; cardinality dispersion degradation > 10% |
| Periodic full posterior | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0 | payload reduction < 30% |
| Single-threshold information, full payload | 41.0% | 10.3% | 25.3% | 13.0% | 2.7% | 0 | local E-OSPA degradation > 5%; OSPA disagreement degradation > 10%; position disagreement degradation > 10% |
| Dual-threshold information | 51.7% | 10.3% | 25.3% | 13.0% | 2.7% | 0 | local E-OSPA degradation > 5%; OSPA disagreement degradation > 10%; position disagreement degradation > 10% |
| Multi-indicator without link gate | 54.7% | 9.8% | 22.7% | 17.8% | 10.4% | 0 | local E-OSPA degradation > 5%; OSPA disagreement degradation > 10%; position disagreement degradation > 10%; cardinality dispersion degradation > 10% |
| Full method (default) | 58.0% | 9.8% | 22.7% | 17.8% | 10.4% | 0 | local E-OSPA degradation > 5%; OSPA disagreement degradation > 10%; position disagreement degradation > 10%; cardinality dispersion degradation > 10% |

## Pareto 筛选

按估算字节数与 local E-OSPA 的二维非支配关系，当前 Pareto arms 为：`Local only`, `Periodic full posterior`, `Full method (default)`。
