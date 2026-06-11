# 双阈值多指标事件触发 GA-LMB 研究报告

- Trials: 20
- Seeds: `[12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31]`
- 仿真长度: 100
- 平均逐节点丢包率: `[0.245 0.21 0.11 0.175 0.19 0.21 0.21 0.25]`

本实验验证的组合是多指标耦合、链路门控降级和轻/重两级 LMB 后验负载；事件触发 LMB 与目标级信息增益触发已有文献先例，不作为单独新颖性主张。

- TSP 2022: https://doi.org/10.1109/TSP.2022.3154227
- TAES 2023: https://doi.org/10.1109/TAES.2022.3187390
- TCSII 2023: https://doi.org/10.1109/TCSII.2023.3238346

## 单试验全通信阈值标定

| Criterion | Loose | Default | Strict | Samples |
|:--|:--|:--|:--|--:|
| Multi-indicator | `[0.3008 0.3561]` | `[0.3264 0.3797]` | `[0.3473 0.3998]` | 3200 |
| Information gain | `[0.4268 0.481]` | `[0.4515 0.6429]` | `[0.4672 1]` | 3200 |

## 通信与性能结果

| Arm | Scalars | Bytes | Trigger | Light | Heavy | Delivery | Downgrades | Edges | Alg. conn. | Runtime (s) | Local E-OSPA | Local RMSE | Consensus OSPA | Position disagreement | Card. dispersion |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Periodic full posterior | 3386148 | 27089183 | 1.000 | 0.000 | 1.000 | 0.800 | 0.0 | 16.0 | 2.000 | 13.394 | 2.0707 | 5.0513 | 1.9493 | 4.8508 | 0.2026 |
| Periodic light posterior + guarded dynamic topology | 1406331 | 11250646 | 1.000 | 1.000 | 0.000 | 0.803 | 0.0 | 16.0 | 2.045 | 13.772 | 2.0613 | 4.4000 | 1.9578 | 4.4896 | 0.2025 |
| Old mainline: LightFloor-GuardedTopo | 1640603 | 13124826 | 0.973 | 0.862 | 0.112 | 0.803 | 144.4 | 16.0 | 2.045 | 14.027 | 2.0734 | 4.4916 | 1.9877 | 4.6413 | 0.2018 |
| C1: LightBackbone-GuardedTopo | 1424621 | 11396967 | 1.000 | 0.979 | 0.021 | 0.803 | 0.0 | 16.0 | 2.045 | 14.071 | 2.0612 | 4.3963 | 1.9580 | 4.4900 | 0.2024 |
| C2: MixedLabel-LightFloor-GuardedTopo | 1413981 | 11311845 | 0.973 | 0.862 | 0.112 | 0.803 | 144.4 | 16.0 | 2.045 | 14.279 | 2.0734 | 4.4916 | 1.9877 | 4.6413 | 0.2018 |

## Effective KLA 图诊断

| Arm | Attempted lambda2 | Delivered lambda2 | Window delivered lambda2 | Effective-weight lambda2 | Label conn. violation | Window label violation | Label stale p90 | Label stale p95 | Topology churn | New-edge no-handshake | Handshake count | Handshake byte share | Self weight | Light weight | Heavy weight | Weight entropy |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Periodic full posterior | 2.000 | 1.878 | 1.999 | 0.371 | 0.157 | 0.157 | 1.25 | 2.49 | 0.000 | 0.000 | 0.0 | 0.000 | 0.395 | 0.000 | 0.605 | 1.363 |
| Periodic light posterior + guarded dynamic topology | 2.045 | 1.907 | 2.220 | 0.369 | 0.161 | 0.161 | 1.25 | 2.57 | 0.043 | 1.000 | 0.0 | 0.000 | 0.402 | 0.598 | 0.000 | 1.358 |
| Old mainline: LightFloor-GuardedTopo | 2.024 | 1.857 | 2.212 | 0.363 | 0.159 | 0.159 | 1.35 | 2.65 | 0.042 | 0.823 | 0.0 | 0.000 | 0.411 | 0.514 | 0.075 | 1.333 |
| C1: LightBackbone-GuardedTopo | 2.045 | 1.907 | 2.218 | 0.369 | 0.161 | 0.161 | 1.25 | 2.57 | 0.043 | 0.374 | 66.6 | 0.022 | 0.402 | 0.587 | 0.011 | 1.358 |
| C2: MixedLabel-LightFloor-GuardedTopo | 2.024 | 1.857 | 2.212 | 0.363 | 0.159 | 0.159 | 1.35 | 2.65 | 0.042 | 0.823 | 0.0 | 0.000 | 0.411 | 0.514 | 0.075 | 1.333 |

## 30% 通信节省升级门槛

| Arm | Bytes reduction | Local E-OSPA change | Consensus OSPA change | Position change | Card. change | Scale up | Failure reasons |
|:--|--:|--:|--:|--:|--:|:--:|:--|
| Periodic full posterior | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0 | payload reduction < 30% |
| Periodic light posterior + guarded dynamic topology | 58.5% | -0.5% | 0.4% | -7.4% | -0.0% | 1 | - |
| Old mainline: LightFloor-GuardedTopo | 51.5% | 0.1% | 2.0% | -4.3% | -0.4% | 1 | - |
| C1: LightBackbone-GuardedTopo | 57.9% | -0.5% | 0.4% | -7.4% | -0.1% | 1 | - |
| C2: MixedLabel-LightFloor-GuardedTopo | 58.2% | 0.1% | 2.0% | -4.3% | -0.4% | 1 | - |

## Paired held-out delta summary

每个 trial 与同 seed 的 `Periodic full posterior` 成对比较。Bytes 为降幅，其他 delta 为退化百分比；pass count 使用 30%/5%/10%/10%/10% 主 gate。

| Arm | Bytes mean | Bytes median | Bytes worst | Local mean +- SE | Local p90 | Local worst | Consensus mean +- SE | Consensus p90 | Consensus worst | Pass count |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Periodic full posterior | 0.0% | 0.0% | 0.0% | 0.0 +- 0.0% | 0.0% | 0.0% | 0.0 +- 0.0% | 0.0% | 0.0% | 0/20 |
| Periodic light posterior + guarded dynamic topology | 58.4% | 58.0% | 55.5% | -0.3 +- 0.5% | 0.9% | 1.9% | 0.5 +- 0.4% | 1.4% | 3.7% | 15/20 |
| Old mainline: LightFloor-GuardedTopo | 51.5% | 51.2% | 47.4% | 0.2 +- 0.3% | 1.4% | 1.8% | 2.0 +- 0.4% | 3.6% | 6.0% | 13/20 |
| C1: LightBackbone-GuardedTopo | 57.8% | 57.5% | 54.9% | -0.4 +- 0.5% | 0.9% | 1.9% | 0.5 +- 0.4% | 1.4% | 3.7% | 15/20 |
| C2: MixedLabel-LightFloor-GuardedTopo | 58.2% | 58.4% | 54.6% | 0.2 +- 0.3% | 1.4% | 1.8% | 2.0 +- 0.4% | 3.6% | 6.0% | 13/20 |

## Pareto 筛选

按估算字节数与 local E-OSPA 的二维非支配关系，当前 Pareto arms 为：`Periodic light posterior + guarded dynamic topology`, `C1: LightBackbone-GuardedTopo`。
