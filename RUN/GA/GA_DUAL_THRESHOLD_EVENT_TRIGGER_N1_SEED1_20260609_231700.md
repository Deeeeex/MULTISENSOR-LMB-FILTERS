# 动态拓扑信息分发路径重构补充记录

- Trials: 1
- Seed: `2`
- 仿真长度: 100
- 平均逐节点丢包率: `[0.1 0.1 0.2 0.1 0 0.5 0.5 0.1]`
- 对照边预算: 16 条无向边

## 实验边界

本轮只隔离检验“融合信息分发路径重构”：触发策略固定为周期全量后验发送，不讨论触发阈值、轻/重负载内容或事件分类。
动态拓扑在每一步按链路可靠性、有效标签重叠和几何互补性对候选边打分，在保持静态拓扑相同边预算的约束下重连，并用代数连通度下限修复避免过度稀疏。

## 100-step 单试验结果

| Arm | Bytes | Trigger | Edges | Alg. conn. | Runtime (s) | Local E-OSPA | Consensus OSPA | Relative bytes | Relative local | Relative consensus |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Periodic full posterior | 32783504 | 1.000 | 16.0 | 2.000 | 350.65 | 2.229 | 2.044 | 0.0% | 0.0% | 0.0% |
| Periodic full posterior + dynamic topology | 28157392 | 1.000 | 16.0 | n/a | 303.70 | 2.180 | 2.166 | -14.1% | -2.2% | +6.0% |

## 20-step 配对预筛结果

| Arm | Bytes | Trigger | Edges | Alg. conn. | Local E-OSPA | Consensus OSPA | Relative bytes | Relative local | Relative consensus |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Periodic full posterior | 2754440 | 1.000 | 16.0 | 2.000 | 3.199 | 2.391 | 0.0% | 0.0% | 0.0% |
| Full method (default) | 1576616 | 0.678 | 16.0 | 2.000 | 3.273 | 2.756 | -42.8% | +2.3% | +15.3% |
| Periodic full posterior + dynamic topology | 2740280 | 1.000 | 16.0 | 2.375 | 2.923 | 2.476 | -0.5% | -8.6% | +3.5% |
| Full method + dynamic topology | 1859784 | 0.769 | 16.0 | 2.425 | 3.080 | 2.794 | -32.5% | -3.7% | +16.9% |

## 判断

纯动态拓扑方向有信号：在同等边预算、周期全量通信下，local E-OSPA 下降，network disagreement 的 OSPA 退化仍在 10% 内。
它不应马上和双阈值事件触发叠加；短试验中组合 arm 虽然满足 30% 字节下降，但 consensus OSPA 退化超过 10%，说明触发缺边和拓扑重构的误差会叠加。

下一轮应先把拓扑收益函数做成独立研究线：固定周期全量或固定每步边预算，比较静态 4+4、动态拓扑、动态拓扑加连通度约束，再做 5-trial Pareto 筛选。
