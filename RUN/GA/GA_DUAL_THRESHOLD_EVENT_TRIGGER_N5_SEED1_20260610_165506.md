# 双阈值多指标事件触发 GA-LMB 研究报告

- Trials: 5
- Seeds: `[2 3 4 5 6]`
- 仿真长度: 100
- 平均逐节点丢包率: `[0.28 0.06 0.2 0.2 0.22 0.22 0.32 0.1]`

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
| Periodic full posterior | 3469555 | 27756443 | 1.000 | 0.000 | 1.000 | 0.796 | 0.0 | 16.0 | 2.000 | 292.379 | 2.0508 | 4.2706 | 1.9473 | 4.3464 | 0.1988 |
| Dynamic topology + handshake + light backbone | 1436158 | 11489266 | 0.999 | 0.980 | 0.019 | 0.799 | 0.0 | 16.0 | 2.036 | 292.772 | 2.0541 | 4.2511 | 1.9666 | 4.3864 | 0.2058 |
| Light-floor mixed-label payload + dynamic topology | 1443934 | 11551475 | 0.977 | 0.863 | 0.114 | 0.798 | 146.6 | 16.0 | 2.035 | 328.516 | 2.0733 | 4.4594 | 1.9963 | 4.4218 | 0.2092 |

## Effective KLA 图诊断

| Arm | Attempted lambda2 | Delivered lambda2 | Window delivered lambda2 | Effective-weight lambda2 | Label conn. violation | Window label violation | Label stale p90 | New-edge no-handshake | Self weight | Light weight | Heavy weight | Weight entropy |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Periodic full posterior | 2.000 | 1.880 | 2.000 | 0.374 | 0.152 | 0.152 | 1.30 | 0.000 | 0.396 | 0.000 | 0.604 | 1.359 |
| Dynamic topology + handshake + light backbone | 2.036 | 1.899 | 2.203 | 0.372 | 0.150 | 0.150 | 1.30 | 0.376 | 0.402 | 0.587 | 0.010 | 1.354 |
| Light-floor mixed-label payload + dynamic topology | 2.026 | 1.863 | 2.195 | 0.369 | 0.146 | 0.144 | 1.41 | 0.794 | 0.409 | 0.513 | 0.078 | 1.334 |

## 30% 通信节省升级门槛

| Arm | Bytes reduction | Local E-OSPA change | Consensus OSPA change | Position change | Card. change | Scale up | Failure reasons |
|:--|--:|--:|--:|--:|--:|:--:|:--|
| Periodic full posterior | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0 | payload reduction < 30% |
| Dynamic topology + handshake + light backbone | 58.6% | 0.2% | 1.0% | 0.9% | 3.5% | 1 | - |
| Light-floor mixed-label payload + dynamic topology | 58.4% | 1.1% | 2.5% | 1.7% | 5.3% | 1 | - |

## Pareto 筛选

按估算字节数与 local E-OSPA 的二维非支配关系，当前 Pareto arms 为：`Periodic full posterior`, `Dynamic topology + handshake + light backbone`。
