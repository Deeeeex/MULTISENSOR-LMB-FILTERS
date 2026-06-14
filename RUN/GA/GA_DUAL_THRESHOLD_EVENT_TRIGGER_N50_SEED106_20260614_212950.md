# 双阈值多指标事件触发 GA-LMB 研究报告

- Trials: 50
- Seeds: `[107 108 109 110 111 112 113 114 115 116 117 118 119 120 121 122 123 124 125 126 127 128 129 130 131 132 133 134 135 136 137 138 139 140 141 142 143 144 145 146 147 148 149 150 151 152 153 154 155 156]`
- 仿真长度: 100
- 平均逐节点丢包率: `[0.194 0.2 0.188 0.21 0.238 0.152 0.226 0.192]`

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
| Recovery stress: Periodic light static topology | 976022 | 7808176 | 3200 | 2229 | 1.000 | 1.000 | 0.000 | 0.697 | 0.0 | 16.0 | 2.000 | 10.721 | 2.2091 | 3.8299 | 2.1272 | 3.6995 | 0.2861 |
| Recovery stress: Reliability-guarded dynamic topology | 1588255 | 12706040 | 3200 | 2916 | 1.000 | 1.000 | 0.000 | 0.911 | 0.0 | 16.0 | 2.350 | 15.557 | 2.0471 | 4.4618 | 1.9502 | 4.3242 | 0.1993 |
| Recovery stress: Balanced reliability repair topology | 1557453 | 12459621 | 3200 | 2943 | 1.000 | 1.000 | 0.000 | 0.920 | 0.0 | 16.0 | 2.000 | 14.199 | 1.9738 | 3.8664 | 1.7549 | 3.5340 | 0.1881 |
| Recovery stress: Role-matched reliability repair topology | 1553474 | 12427792 | 3200 | 2943 | 1.000 | 1.000 | 0.000 | 0.920 | 0.0 | 16.0 | 2.000 | 14.211 | 1.9701 | 3.9391 | 1.7572 | 3.4896 | 0.1904 |
| Recovery stress: Distance-balanced reliability repair topology | 1568901 | 12551206 | 3200 | 2942 | 1.000 | 1.000 | 0.000 | 0.919 | 0.0 | 16.0 | 2.000 | 14.349 | 1.9777 | 4.5905 | 1.7645 | 3.9842 | 0.1888 |

## Effective KLA 图诊断

| Arm | Attempted lambda2 | Delivered lambda2 | Window delivered lambda2 | Effective-weight lambda2 | Label conn. violation | Window label violation | Label stale p90 | Label stale p95 | Topology churn | New-edge no-handshake | Handshake count | Handshake byte share | Self weight | Light weight | Heavy weight | Weight entropy |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Recovery stress: Periodic light static topology | 2.000 | 0.081 | 0.377 | 0.015 | 0.869 | 0.581 | 37.34 | 47.56 | 0.000 | 1.000 | 0.0 | 0.000 | 0.423 | 0.577 | 0.000 | 1.266 |
| Recovery stress: Reliability-guarded dynamic topology | 2.350 | 2.195 | 3.972 | 0.356 | 0.159 | 0.159 | 0.43 | 6.49 | 0.309 | 1.000 | 0.0 | 0.000 | 0.418 | 0.582 | 0.000 | 1.412 |
| Recovery stress: Balanced reliability repair topology | 2.000 | 1.984 | 2.000 | 0.357 | 0.146 | 0.146 | 0.25 | 0.75 | 0.000 | 1.000 | 0.0 | 0.000 | 0.356 | 0.644 | 0.000 | 1.485 |
| Recovery stress: Role-matched reliability repair topology | 2.000 | 1.983 | 2.000 | 0.357 | 0.147 | 0.147 | 0.25 | 0.76 | 0.000 | 1.000 | 0.0 | 0.000 | 0.356 | 0.644 | 0.000 | 1.484 |
| Recovery stress: Distance-balanced reliability repair topology | 2.000 | 1.978 | 2.030 | 0.356 | 0.144 | 0.144 | 0.26 | 0.85 | 0.003 | 1.000 | 0.0 | 0.000 | 0.356 | 0.644 | 0.000 | 1.484 |

## Topology risk attribution diagnostics

| Arm | Edge risk | Static risk | Non-static risk | New-edge risk | Label mismatch | Non-static mismatch | Fusion non-static weight | Fusion new-edge weight | Fusion risk-weighted edge risk | Fusion risk-weighted mismatch |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Recovery stress: Periodic light static topology | 0.319 | 0.319 | 0.000 | 0.000 | 0.146 | 0.000 | 0.000 | 0.006 | 0.265 | 0.083 |
| Recovery stress: Reliability-guarded dynamic topology | 0.265 | 0.271 | 0.254 | 0.218 | 0.076 | 0.058 | 0.194 | 0.107 | 0.265 | 0.076 |
| Recovery stress: Balanced reliability repair topology | 0.263 | 0.259 | 0.274 | 0.000 | 0.077 | 0.087 | 0.161 | 0.006 | 0.263 | 0.077 |
| Recovery stress: Role-matched reliability repair topology | 0.263 | 0.259 | 0.275 | 0.000 | 0.078 | 0.088 | 0.161 | 0.006 | 0.263 | 0.078 |
| Recovery stress: Distance-balanced reliability repair topology | 0.262 | 0.259 | 0.273 | 0.003 | 0.077 | 0.085 | 0.161 | 0.007 | 0.263 | 0.078 |

## 30% 通信节省升级门槛

| Arm | Bytes reduction | Local E-OSPA change | Consensus OSPA change | Position change | Card. change | Scale up | Failure reasons |
|:--|--:|--:|--:|--:|--:|:--:|:--|
| Recovery stress: Periodic light static topology | NaN% | NaN% | NaN% | NaN% | NaN% | 0 | - |
| Recovery stress: Reliability-guarded dynamic topology | NaN% | NaN% | NaN% | NaN% | NaN% | 0 | - |
| Recovery stress: Balanced reliability repair topology | NaN% | NaN% | NaN% | NaN% | NaN% | 0 | - |
| Recovery stress: Role-matched reliability repair topology | NaN% | NaN% | NaN% | NaN% | NaN% | 0 | - |
| Recovery stress: Distance-balanced reliability repair topology | NaN% | NaN% | NaN% | NaN% | NaN% | 0 | - |

## Paired held-out delta summary

每个 trial 与同 seed 的 `Periodic full posterior` 成对比较。Bytes 为降幅，其他 delta 为退化百分比；pass count 使用 30%/5%/10%/10%/10% 主 gate。

| Arm | Bytes mean | Bytes median | Bytes worst | Local mean +- SE | Local p90 | Local worst | Consensus mean +- SE | Consensus p90 | Consensus worst | Pass count |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Recovery stress: Periodic light static topology | NaN% | NaN% | NaN% | NaN +- NaN% | NaN% | NaN% | NaN +- NaN% | NaN% | NaN% | 0/0 |
| Recovery stress: Reliability-guarded dynamic topology | NaN% | NaN% | NaN% | NaN +- NaN% | NaN% | NaN% | NaN +- NaN% | NaN% | NaN% | 0/0 |
| Recovery stress: Balanced reliability repair topology | NaN% | NaN% | NaN% | NaN +- NaN% | NaN% | NaN% | NaN +- NaN% | NaN% | NaN% | 0/0 |
| Recovery stress: Role-matched reliability repair topology | NaN% | NaN% | NaN% | NaN +- NaN% | NaN% | NaN% | NaN +- NaN% | NaN% | NaN% | 0/0 |
| Recovery stress: Distance-balanced reliability repair topology | NaN% | NaN% | NaN% | NaN +- NaN% | NaN% | NaN% | NaN +- NaN% | NaN% | NaN% | 0/0 |

## Pareto 筛选

按估算字节数与 local E-OSPA 的二维非支配关系，当前 Pareto arms 为：`Recovery stress: Periodic light static topology`, `Recovery stress: Role-matched reliability repair topology`。
