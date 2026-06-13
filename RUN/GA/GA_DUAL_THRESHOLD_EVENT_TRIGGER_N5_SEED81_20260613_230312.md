# 双阈值多指标事件触发 GA-LMB 研究报告

- Trials: 5
- Seeds: `[82 83 84 85 86]`
- 仿真长度: 100
- 平均逐节点丢包率: `[0.24 0.18 0.28 0.18 0.2 0.14 0.12 0.26]`

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
| Recovery stress: Periodic light static topology | 943807 | 7550454 | 3200 | 2233 | 1.000 | 1.000 | 0.000 | 0.698 | 0.0 | 16.0 | 2.000 | 154.317 | 2.2064 | 3.3027 | 2.1164 | 3.1352 | 0.2940 |
| Recovery stress: Reliability-guarded dynamic topology | 1683654 | 13469235 | 3200 | 2911 | 1.000 | 1.000 | 0.000 | 0.910 | 0.0 | 16.0 | 2.364 | 264.129 | 2.0895 | 4.8397 | 1.9926 | 4.5605 | 0.2145 |

## Effective KLA 图诊断

| Arm | Attempted lambda2 | Delivered lambda2 | Window delivered lambda2 | Effective-weight lambda2 | Label conn. violation | Window label violation | Label stale p90 | Label stale p95 | Topology churn | New-edge no-handshake | Handshake count | Handshake byte share | Self weight | Light weight | Heavy weight | Weight entropy |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Recovery stress: Periodic light static topology | 2.000 | 0.092 | 0.393 | 0.017 | 0.850 | 0.552 | 32.72 | 47.77 | 0.000 | 1.000 | 0.0 | 0.000 | 0.423 | 0.577 | 0.000 | 1.267 |
| Recovery stress: Reliability-guarded dynamic topology | 2.364 | 2.224 | 3.958 | 0.361 | 0.160 | 0.160 | 0.42 | 6.46 | 0.308 | 1.000 | 0.0 | 0.000 | 0.417 | 0.583 | 0.000 | 1.411 |

## 30% 通信节省升级门槛

| Arm | Bytes reduction | Local E-OSPA change | Consensus OSPA change | Position change | Card. change | Scale up | Failure reasons |
|:--|--:|--:|--:|--:|--:|:--:|:--|
| Recovery stress: Periodic light static topology | NaN% | NaN% | NaN% | NaN% | NaN% | 0 | - |
| Recovery stress: Reliability-guarded dynamic topology | NaN% | NaN% | NaN% | NaN% | NaN% | 0 | - |

## Paired held-out delta summary

每个 trial 与同 seed 的 `Periodic full posterior` 成对比较。Bytes 为降幅，其他 delta 为退化百分比；pass count 使用 30%/5%/10%/10%/10% 主 gate。

| Arm | Bytes mean | Bytes median | Bytes worst | Local mean +- SE | Local p90 | Local worst | Consensus mean +- SE | Consensus p90 | Consensus worst | Pass count |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Recovery stress: Periodic light static topology | NaN% | NaN% | NaN% | NaN +- NaN% | NaN% | NaN% | NaN +- NaN% | NaN% | NaN% | 0/0 |
| Recovery stress: Reliability-guarded dynamic topology | NaN% | NaN% | NaN% | NaN +- NaN% | NaN% | NaN% | NaN +- NaN% | NaN% | NaN% | 0/0 |

## Pareto 筛选

按估算字节数与 local E-OSPA 的二维非支配关系，当前 Pareto arms 为：`Recovery stress: Periodic light static topology`, `Recovery stress: Reliability-guarded dynamic topology`。
