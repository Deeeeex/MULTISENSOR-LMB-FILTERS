# 双阈值多指标事件触发 GA-LMB 研究报告

- Trials: 20
- Seeds: `[87 88 89 90 91 92 93 94 95 96 97 98 99 100 101 102 103 104 105 106]`
- 仿真长度: 100
- 平均逐节点丢包率: `[0.25 0.19 0.21 0.195 0.205 0.145 0.21 0.195]`

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
| Recovery stress: Periodic light static topology | 994100 | 7952802 | 3200 | 2227 | 1.000 | 1.000 | 0.000 | 0.696 | 0.0 | 16.0 | 2.000 | 165.962 | 2.2240 | 4.3100 | 2.1428 | 4.0883 | 0.2923 |
| Recovery stress: Reliability-guarded dynamic topology | 1676422 | 13411373 | 3200 | 2912 | 1.000 | 1.000 | 0.000 | 0.910 | 0.0 | 16.0 | 2.360 | 268.400 | 2.0696 | 4.6763 | 1.9753 | 4.5553 | 0.2038 |

## Effective KLA 图诊断

| Arm | Attempted lambda2 | Delivered lambda2 | Window delivered lambda2 | Effective-weight lambda2 | Label conn. violation | Window label violation | Label stale p90 | Label stale p95 | Topology churn | New-edge no-handshake | Handshake count | Handshake byte share | Self weight | Light weight | Heavy weight | Weight entropy |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Recovery stress: Periodic light static topology | 2.000 | 0.079 | 0.367 | 0.014 | 0.874 | 0.609 | 33.80 | 45.68 | 0.000 | 1.000 | 0.0 | 0.000 | 0.424 | 0.576 | 0.000 | 1.265 |
| Recovery stress: Reliability-guarded dynamic topology | 2.360 | 2.193 | 4.038 | 0.357 | 0.167 | 0.167 | 0.47 | 6.84 | 0.321 | 1.000 | 0.0 | 0.000 | 0.417 | 0.583 | 0.000 | 1.411 |

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

## Topology Recovery Stress Gate

This stress-specific gate compares recovery dynamic topology against the static light graph under targeted bridge-link failures. It uses attempted message count as the communication budget because delivered payload bytes increase when the recovery topology successfully routes around failed links.

| Metric | Static | Recovery | Change | Win count |
|:--|--:|--:|--:|--:|
| Attempt count | 3200 | 3200 | 0.00% | equal=1 |
| Delivery count | 2227.4 | 2911.8 | 30.72% | 20/20 |
| Local E-OSPA | 2.2240 | 2.0696 | -6.94% | 19/20 |
| Consensus OSPA | 2.1428 | 1.9753 | -7.82% | 19/20 |
| Position disagreement | 4.0883 | 4.5553 | 11.42% | 11/20 |
| Cardinality dispersion | 0.2923 | 0.2038 | -30.28% | 19/20 |
| Effective-weight lambda2 | 0.0145 | 0.3566 | 2360.81% | 20/20 |

- Strict all-trial recovery gate: `0`
- Robust recovery gate: `1` (min wins `18/20`)
- Position-risk flag: `1`

| Seed | Delivery | Local E-OSPA | Consensus OSPA | Position | Card. | Eff. lambda2 |
|:--|--:|--:|--:|--:|--:|--:|
| 87 | 29.79% | -7.65% | -6.61% | 16.81% | -30.47% | 2327.30% |
| 88 | 31.25% | -6.96% | -9.47% | -7.57% | -11.17% | 2078.25% |
| 89 | 30.24% | -9.01% | -8.34% | -7.11% | -29.18% | 2016.34% |
| 90 | 30.01% | -7.58% | -9.70% | 23.30% | -36.64% | 2775.61% |
| 91 | 30.91% | -6.76% | -10.01% | 17.09% | -30.66% | 3097.92% |
| 92 | 30.51% | -10.92% | -11.86% | -20.02% | -48.29% | 1951.09% |
| 93 | 31.19% | -6.07% | -4.87% | 19.84% | -12.14% | 2437.58% |
| 94 | 30.64% | -4.17% | -8.18% | 30.78% | -33.09% | 2747.27% |
| 95 | 30.13% | -13.50% | -12.50% | -6.57% | -37.07% | 2769.38% |
| 96 | 29.40% | 10.49% | 7.18% | 231.62% | 12.43% | 1775.31% |
| 97 | 31.18% | -7.71% | -6.61% | 16.90% | -16.57% | 2054.35% |
| 98 | 31.91% | -5.03% | -4.11% | 113.07% | -27.43% | 2536.49% |
| 99 | 30.89% | -7.65% | -8.99% | -2.84% | -33.20% | 2131.90% |
| 100 | 31.54% | -6.29% | -8.20% | -13.70% | -23.08% | 2585.82% |
| 101 | 30.21% | -8.54% | -8.76% | -7.18% | -26.53% | 1958.48% |
| 102 | 30.48% | -5.65% | -5.38% | -29.48% | -15.50% | 4125.58% |
| 103 | 31.14% | -8.98% | -11.78% | 22.75% | -58.06% | 2194.36% |
| 104 | 31.33% | -8.47% | -8.64% | -10.65% | -42.67% | 2050.99% |
| 105 | 31.99% | -9.28% | -9.76% | -2.87% | -26.03% | 3555.73% |
| 106 | 29.78% | -7.43% | -8.24% | -8.95% | -31.11% | 2144.15% |
