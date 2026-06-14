# 双阈值多指标事件触发 GA-LMB 研究报告

- Trials: 1
- Seeds: `96`
- 仿真长度: 100
- 平均逐节点丢包率: `[0.1 0.2 0.5 0 0.1 0.5 0.1 0.1]`

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
| Recovery stress: Periodic light static topology | 919378 | 7355024 | 3200 | 2245 | 1.000 | 1.000 | 0.000 | 0.702 | 0.0 | 16.0 | 2.000 | 183.231 | 2.1103 | 3.2974 | 2.0470 | 2.6912 | 0.2213 |
| Recovery stress: Reliability-guarded dynamic topology | 2078626 | 16629008 | 3200 | 2905 | 1.000 | 1.000 | 0.000 | 0.908 | 0.0 | 16.0 | 2.341 | 515.308 | 2.3317 | 8.4724 | 2.1940 | 8.9245 | 0.2487 |

## Effective KLA 图诊断

| Arm | Attempted lambda2 | Delivered lambda2 | Window delivered lambda2 | Effective-weight lambda2 | Label conn. violation | Window label violation | Label stale p90 | Label stale p95 | Topology churn | New-edge no-handshake | Handshake count | Handshake byte share | Self weight | Light weight | Heavy weight | Weight entropy |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Recovery stress: Periodic light static topology | 2.000 | 0.100 | 0.481 | 0.019 | 0.797 | 0.401 | 32.38 | 44.22 | 0.000 | 1.000 | 0.0 | 0.000 | 0.421 | 0.579 | 0.000 | 1.272 |
| Recovery stress: Reliability-guarded dynamic topology | 2.341 | 2.119 | 3.926 | 0.348 | 0.128 | 0.128 | 0.62 | 8.05 | 0.304 | 1.000 | 0.0 | 0.000 | 0.413 | 0.587 | 0.000 | 1.417 |

## Topology risk attribution diagnostics

| Arm | Edge risk | Static risk | Non-static risk | New-edge risk | Label mismatch | Non-static mismatch | Fusion non-static weight | Fusion new-edge weight | Fusion risk-weighted edge risk | Fusion risk-weighted mismatch |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Recovery stress: Periodic light static topology | 0.270 | 0.270 | 0.000 | 0.000 | 0.081 | 0.000 | 0.000 | 0.006 | 0.250 | 0.059 |
| Recovery stress: Reliability-guarded dynamic topology | 0.258 | 0.263 | 0.246 | 0.192 | 0.067 | 0.051 | 0.189 | 0.105 | 0.255 | 0.065 |

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

按估算字节数与 local E-OSPA 的二维非支配关系，当前 Pareto arms 为：`Recovery stress: Periodic light static topology`。

## Topology Risk Forensic Attribution

- Seed: `96`
- Worst position-delta time: `33` (delta 24.640)

| Hypothesis | Score |
|:--|--:|
| Wrong new edges | 0.218 |
| Over-strong dynamic weights | 0.183 |
| Topology churn | 0.847 |
| Label mismatch | 0.000 |

| Top delta time | Position delta | Non-static risk | Non-static mismatch | Non-static weight | New-edge weight | Eff. lambda2 |
|:--|--:|--:|--:|--:|--:|--:|
| 33 | 24.640 | 0.220 | 0.000 | 0.159 | 0.000 | 0.353 |
| 49 | 18.825 | 0.198 | 0.048 | 0.214 | 0.190 | 0.406 |
| 9 | 18.550 | 0.261 | 0.083 | 0.233 | 0.083 | 0.451 |
| 47 | 17.727 | 0.268 | 0.028 | 0.233 | 0.129 | 0.449 |
| 10 | 16.918 | 0.183 | 0.000 | 0.155 | 0.205 | 0.237 |
