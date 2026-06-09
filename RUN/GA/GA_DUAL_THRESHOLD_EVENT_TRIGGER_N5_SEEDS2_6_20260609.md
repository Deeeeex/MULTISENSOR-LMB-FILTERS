# 双阈值多指标事件触发 GA-LMB：5-trial Pareto 筛选

## 实验范围

- 场景：`4+4` 八传感器编队，100 步，固定 Metropolis 融合权重。
- 固定 trial seeds：`[2, 3, 4, 5, 6]`。
- 阈值标定：独立 seed 2 全通信轨迹；多指标阈值为：
  - loose：`[0.3004, 0.3503]`
  - default：`[0.3236, 0.3708]`
  - strict：`[0.3413, 0.3945]`
- 5-trial 阶段保留首轮 Pareto 候选：local、周期全量和完整方法 default。
- single-heavy 被 dual-information 支配；dual-information 和无门控多指标 arm 被完整方法 default 支配；loose 在 100 步复核中也被 default 支配。

本实验考察的组合贡献是多指标耦合、链路门控降级和轻/重两级 LMB 后验负载。事件触发 LMB 与目标级信息增益触发已有先例，不作为单独新颖性主张：

- TSP 2022: https://doi.org/10.1109/TSP.2022.3154227
- TAES 2023: https://doi.org/10.1109/TAES.2022.3187390
- TCSII 2023: https://doi.org/10.1109/TCSII.2023.3238346

## 平均结果

| Arm | Bytes | Scalars | Trigger | Light | Heavy | Delivery | Downgrades | Runtime (s) | Local E-OSPA | Local RMSE | Consensus OSPA | Position disagreement | Card. dispersion |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Local only | 0 | 0 | 0.000 | 0.000 | 0.000 | 0.000 | 0.0 | 115.5 | 3.4094 | 4.5540 | 4.0743 | 6.9643 | 0.3147 |
| Periodic full posterior | 27,756,443 | 3,469,555 | 1.000 | 0.000 | 1.000 | 0.796 | 0.0 | 486.0 | 2.0508 | 4.2706 | 1.9473 | 4.3464 | 0.1988 |
| Full method (default) | 13,155,811 | 1,644,476 | 0.665 | 0.336 | 0.329 | 0.792 | 403.6 | 542.8 | 2.3203 | 4.5470 | 2.4331 | 4.8098 | 0.2260 |

运行时间包含四个种子并行执行时的 CPU 竞争，只用于确认实现可运行，不适合作为严格速度比较。

## 升级门槛

相对周期全量基线，要求估算字节至少下降 30%，local E-OSPA 退化不超过 5%，三项 network disagreement 各不超过 10%。

| Arm | Bytes reduction | Local E-OSPA change | Consensus OSPA change | Position change | Card. change | 通过 |
|:--|--:|--:|--:|--:|--:|:--:|
| Full method (default) | 52.6% | +13.1% | +25.0% | +10.7% | +13.7% | 否 |

没有配置达到升级门槛，因此不启动 20/50 trials。

## Pareto 与失败模式

按 bytes/local E-OSPA 二维关系，周期全量和 full-default 为通信 arm 中的非支配点；local-only 是零通信端点。full-default 满足字节节省要求，但四项性能约束均至少有一项超限。

主要失败模式：

1. 单轮同步融合中，未触发边不提供任何邻居后验。通信节省直接减少了每步可用的空间信息，network disagreement 增幅明显。
2. 多指标 max-over-label 聚合对瞬态标签仍较敏感。将强制标签变化限定为 `r >= 0.5` 后避免了每步 birth 候选触发重事件；修正邻居缓存方向后，多指标 arm 在 1-trial 中略优于 information-only，但增益不足以通过门槛。
3. 链路门控本身能继续减少字节。在 1-trial 中，完整 default 相对无门控多指标 arm 从 14,850,776 降至 13,768,688 bytes，tracking 指标不变；主要性能损失来自触发稀疏化，而不是门控降级。
4. 下一轮应优先研究按标签保留最小更新频率、接收端预测补偿或有界陈旧缓存，而不是直接放宽全局阈值。任何改动仍需保证仿真真值不进入在线触发函数。
