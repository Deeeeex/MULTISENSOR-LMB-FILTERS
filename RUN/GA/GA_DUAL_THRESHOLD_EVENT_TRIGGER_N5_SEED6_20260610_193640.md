# 双阈值多指标事件触发 GA-LMB 研究报告

- Trials: 5
- Seeds: `[7 8 9 10 11]`
- 仿真长度: 100
- 平均逐节点丢包率: `[0.24 0.2 0.18 0.14 0.26 0.26 0.2 0.12]`

本实验验证的组合是多指标耦合、链路门控降级和轻/重两级 LMB 后验负载；事件触发 LMB 与目标级信息增益触发已有文献先例，不作为单独新颖性主张。

- TSP 2022: https://doi.org/10.1109/TSP.2022.3154227
- TAES 2023: https://doi.org/10.1109/TAES.2022.3187390
- TCSII 2023: https://doi.org/10.1109/TCSII.2023.3238346

## 单试验全通信阈值标定

| Criterion | Loose | Default | Strict | Samples |
|:--|:--|:--|:--|--:|
| Multi-indicator | `[0.2847 0.3472]` | `[0.3149 0.3717]` | `[0.3379 0.3915]` | 3200 |
| Information gain | `[0.4267 0.4896]` | `[0.4523 1]` | `[0.4691 1]` | 3192 |

## 通信与性能结果

| Arm | Scalars | Bytes | Trigger | Light | Heavy | Delivery | Downgrades | Edges | Alg. conn. | Runtime (s) | Local E-OSPA | Local RMSE | Consensus OSPA | Position disagreement | Card. dispersion |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Periodic full posterior | 3472346 | 27778768 | 1.000 | 0.000 | 1.000 | 0.798 | 0.0 | 16.0 | 2.000 | 311.691 | 2.0391 | 3.9770 | 1.9636 | 4.2644 | 0.2165 |
| Periodic light posterior + guarded dynamic topology | 1459375 | 11675002 | 1.000 | 1.000 | 0.000 | 0.800 | 0.0 | 16.0 | 2.029 | 298.753 | 2.0427 | 4.1278 | 1.9733 | 4.3405 | 0.2182 |
| Old mainline: LightFloor-GuardedTopo | 1714688 | 13717507 | 0.977 | 0.855 | 0.122 | 0.800 | 137.6 | 16.0 | 2.030 | 324.435 | 2.0567 | 4.2177 | 1.9978 | 4.5816 | 0.2105 |
| C1: LightBackbone-GuardedTopo | 1471515 | 11772123 | 0.999 | 0.981 | 0.018 | 0.801 | 0.0 | 16.0 | 2.029 | 322.835 | 2.0432 | 4.1280 | 1.9734 | 4.3372 | 0.2185 |
| C2: MixedLabel-LightFloor-GuardedTopo | 1482785 | 11862283 | 0.977 | 0.855 | 0.122 | 0.800 | 137.6 | 16.0 | 2.030 | 278.398 | 2.0567 | 4.2177 | 1.9978 | 4.5816 | 0.2105 |

## Effective KLA 图诊断

| Arm | Attempted lambda2 | Delivered lambda2 | Window delivered lambda2 | Effective-weight lambda2 | Label conn. violation | Window label violation | Label stale p90 | Label stale p95 | Topology churn | New-edge no-handshake | Handshake count | Handshake byte share | Self weight | Light weight | Heavy weight | Weight entropy |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Periodic full posterior | 2.000 | 1.862 | 1.995 | 0.364 | 0.141 | 0.141 | 1.25 | 2.44 | 0.000 | 0.000 | 0.0 | 0.000 | 0.395 | 0.000 | 0.605 | 1.362 |
| Periodic light posterior + guarded dynamic topology | 2.029 | 1.882 | 2.187 | 0.362 | 0.148 | 0.148 | 1.27 | 2.46 | 0.038 | 1.000 | 0.0 | 0.000 | 0.400 | 0.600 | 0.000 | 1.358 |
| Old mainline: LightFloor-GuardedTopo | 2.023 | 1.847 | 2.184 | 0.360 | 0.142 | 0.142 | 1.39 | 2.57 | 0.040 | 0.809 | 0.0 | 0.000 | 0.407 | 0.510 | 0.083 | 1.337 |
| C1: LightBackbone-GuardedTopo | 2.029 | 1.878 | 2.185 | 0.362 | 0.148 | 0.148 | 1.27 | 2.46 | 0.038 | 0.392 | 57.0 | 0.015 | 0.400 | 0.590 | 0.010 | 1.358 |
| C2: MixedLabel-LightFloor-GuardedTopo | 2.023 | 1.847 | 2.184 | 0.360 | 0.142 | 0.142 | 1.39 | 2.57 | 0.040 | 0.809 | 0.0 | 0.000 | 0.407 | 0.510 | 0.083 | 1.337 |

## 30% 通信节省升级门槛

| Arm | Bytes reduction | Local E-OSPA change | Consensus OSPA change | Position change | Card. change | Scale up | Failure reasons |
|:--|--:|--:|--:|--:|--:|:--:|:--|
| Periodic full posterior | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0 | payload reduction < 30% |
| Periodic light posterior + guarded dynamic topology | 58.0% | 0.2% | 0.5% | 1.8% | 0.8% | 1 | - |
| Old mainline: LightFloor-GuardedTopo | 50.6% | 0.9% | 1.7% | 7.4% | -2.8% | 1 | - |
| C1: LightBackbone-GuardedTopo | 57.6% | 0.2% | 0.5% | 1.7% | 0.9% | 1 | - |
| C2: MixedLabel-LightFloor-GuardedTopo | 57.3% | 0.9% | 1.7% | 7.4% | -2.8% | 1 | - |

## Paired held-out delta summary

每个 trial 与同 seed 的 `Periodic full posterior` 成对比较。Bytes 为降幅，其他 delta 为退化百分比；pass count 使用 30%/5%/10%/10%/10% 主 gate。

| Arm | Bytes mean | Bytes median | Bytes worst | Local mean +- SE | Local p90 | Local worst | Consensus mean +- SE | Consensus p90 | Consensus worst | Pass count |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Periodic full posterior | 0.0% | 0.0% | 0.0% | 0.0 +- 0.0% | 0.0% | 0.0% | 0.0 +- 0.0% | 0.0% | 0.0% | 0/5 |
| Periodic light posterior + guarded dynamic topology | 58.0% | 57.8% | 56.3% | 0.2 +- 0.4% | 0.9% | 1.3% | 0.5 +- 0.4% | 1.3% | 1.6% | 5/5 |
| Old mainline: LightFloor-GuardedTopo | 50.5% | 50.5% | 47.3% | 0.9 +- 0.2% | 1.2% | 1.3% | 1.7 +- 0.2% | 2.1% | 2.3% | 4/5 |
| C1: LightBackbone-GuardedTopo | 57.6% | 57.4% | 55.9% | 0.2 +- 0.4% | 0.9% | 1.3% | 0.5 +- 0.4% | 1.3% | 1.6% | 5/5 |
| C2: MixedLabel-LightFloor-GuardedTopo | 57.3% | 57.5% | 55.6% | 0.9 +- 0.2% | 1.2% | 1.3% | 1.7 +- 0.2% | 2.1% | 2.3% | 4/5 |

## Pareto 筛选

按估算字节数与 local E-OSPA 的二维非支配关系，当前 Pareto arms 为：`Periodic full posterior`, `Periodic light posterior + guarded dynamic topology`。
