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

| Arm | Scalars | Bytes | Trigger | Light | Heavy | Delivery | Downgrades | Edges | Alg. conn. | Runtime (s) | Local E-OSPA | Local RMSE | Consensus OSPA | Position disagreement | Card. dispersion |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Periodic full posterior | 4097938 | 32783504 | 1.000 | 0.000 | 1.000 | 0.805 | 0.0 | 16.0 | 2.000 | 541.060 | 2.2289 | 6.7555 | 2.0437 | 6.8172 | 0.2275 |
| Full method (default) | 1721086 | 13768688 | 0.650 | 0.316 | 0.334 | 0.806 | 351.0 | 16.0 | 2.000 | 400.375 | 2.4464 | 8.1483 | 2.5083 | 8.0337 | 0.2512 |
| Periodic full posterior + dynamic topology | 3519674 | 28157392 | 1.000 | 0.000 | 1.000 | 0.835 | 0.0 | 16.0 | 2.326 | 628.105 | 2.1804 | 4.3686 | 2.1665 | 5.4522 | 0.2275 |
| Full method + dynamic topology | 2080691 | 16645528 | 0.681 | 0.309 | 0.372 | 0.843 | 334.0 | 16.0 | 2.297 | 829.441 | 2.4661 | 6.2049 | 2.6394 | 6.6552 | 0.2462 |
| Dynamic topology + new-edge handshake | 2598419 | 20787352 | 0.745 | 0.207 | 0.538 | 0.832 | 206.0 | 16.0 | 2.332 | 489.558 | 2.3619 | 5.6668 | 2.4893 | 6.4192 | 0.2437 |
| Dynamic topology + handshake + light backbone | 1624000 | 12992000 | 1.000 | 0.977 | 0.023 | 0.809 | 0.0 | 16.0 | 2.051 | 405.517 | 2.2101 | 6.1669 | 2.0285 | 6.0632 | 0.2325 |
| Dynamic topology + mode-aware KLA graph | 1700843 | 13606744 | 1.000 | 0.975 | 0.025 | 0.809 | 0.0 | 16.0 | 2.050 | 386.410 | 2.3446 | 6.2395 | 2.4658 | 6.9405 | 0.2238 |
| Dynamic topology + effective KLA graph guard | 1801043 | 14408344 | 1.000 | 0.931 | 0.069 | 0.809 | 53.0 | 16.0 | 2.050 | 385.283 | 2.3669 | 7.3025 | 2.4421 | 6.8083 | 0.2475 |
| Light-floor mixed-label payload + dynamic topology | 1599187 | 12793496 | 0.979 | 0.887 | 0.092 | 0.810 | 140.0 | 16.0 | 2.051 | 415.147 | 2.2183 | 6.3033 | 2.0636 | 6.2174 | 0.2425 |

## Effective KLA 图诊断

| Arm | Attempted lambda2 | Delivered lambda2 | Window delivered lambda2 | Effective-weight lambda2 | Label conn. violation | Window label violation | Label stale p90 | New-edge no-handshake | Self weight | Light weight | Heavy weight | Weight entropy |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Periodic full posterior | 2.000 | 1.886 | 2.000 | 0.373 | 0.193 | 0.193 | 1.24 | 0.000 | 0.392 | 0.000 | 0.608 | 1.370 |
| Full method (default) | 1.531 | 1.184 | 2.000 | 0.268 | 0.237 | 0.197 | 3.23 | 0.250 | 0.527 | 0.203 | 0.270 | 1.010 |
| Periodic full posterior + dynamic topology | 2.326 | 2.124 | 4.146 | 0.344 | 0.156 | 0.156 | 1.79 | 0.000 | 0.463 | 0.000 | 0.537 | 1.314 |
| Full method + dynamic topology | 1.662 | 1.312 | 4.034 | 0.240 | 0.250 | 0.183 | 3.07 | 0.539 | 0.577 | 0.174 | 0.249 | 0.994 |
| Dynamic topology + new-edge handshake | 1.786 | 1.446 | 4.820 | 0.266 | 0.280 | 0.189 | 2.48 | 0.007 | 0.541 | 0.120 | 0.339 | 1.072 |
| Dynamic topology + handshake + light backbone | 2.051 | 1.925 | 2.234 | 0.372 | 0.179 | 0.179 | 1.22 | 0.311 | 0.399 | 0.588 | 0.013 | 1.366 |
| Dynamic topology + mode-aware KLA graph | 2.050 | 1.927 | 2.251 | 0.270 | 0.193 | 0.193 | 1.25 | 0.295 | 0.557 | 0.427 | 0.016 | 1.198 |
| Dynamic topology + effective KLA graph guard | 2.050 | 1.921 | 2.241 | 0.279 | 0.192 | 0.192 | 1.25 | 0.300 | 0.547 | 0.400 | 0.054 | 1.206 |
| Light-floor mixed-label payload + dynamic topology | 2.044 | 1.899 | 2.226 | 0.370 | 0.173 | 0.173 | 1.30 | 0.792 | 0.404 | 0.531 | 0.064 | 1.350 |

## 30% 通信节省升级门槛

| Arm | Bytes reduction | Local E-OSPA change | Consensus OSPA change | Position change | Card. change | Scale up | Failure reasons |
|:--|--:|--:|--:|--:|--:|:--:|:--|
| Periodic full posterior | 0.0% | 0.0% | 0.0% | 0.0% | 0.0% | 0 | payload reduction < 30% |
| Full method (default) | 58.0% | 9.8% | 22.7% | 17.8% | 10.4% | 0 | local E-OSPA degradation > 5%; OSPA disagreement degradation > 10%; position disagreement degradation > 10%; cardinality dispersion degradation > 10% |
| Periodic full posterior + dynamic topology | 14.1% | -2.2% | 6.0% | -20.0% | 0.0% | 0 | payload reduction < 30% |
| Full method + dynamic topology | 49.2% | 10.6% | 29.1% | -2.4% | 8.2% | 0 | local E-OSPA degradation > 5%; OSPA disagreement degradation > 10% |
| Dynamic topology + new-edge handshake | 36.6% | 6.0% | 21.8% | -5.8% | 7.1% | 0 | local E-OSPA degradation > 5%; OSPA disagreement degradation > 10% |
| Dynamic topology + handshake + light backbone | 60.4% | -0.8% | -0.7% | -11.1% | 2.2% | 1 | - |
| Dynamic topology + mode-aware KLA graph | 58.5% | 5.2% | 20.7% | 1.8% | -1.6% | 0 | local E-OSPA degradation > 5%; OSPA disagreement degradation > 10% |
| Dynamic topology + effective KLA graph guard | 56.1% | 6.2% | 19.5% | -0.1% | 8.8% | 0 | local E-OSPA degradation > 5%; OSPA disagreement degradation > 10% |
| Light-floor mixed-label payload + dynamic topology | 61.0% | -0.5% | 1.0% | -8.8% | 6.6% | 1 | - |

## Pareto 筛选

按估算字节数与 local E-OSPA 的二维非支配关系，当前 Pareto arms 为：`Periodic full posterior + dynamic topology`, `Dynamic topology + handshake + light backbone`, `Light-floor mixed-label payload + dynamic topology`。
