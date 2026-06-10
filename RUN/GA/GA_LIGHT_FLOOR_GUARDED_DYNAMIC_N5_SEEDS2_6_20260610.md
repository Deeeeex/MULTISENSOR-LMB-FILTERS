# 守卫式动态拓扑 + 双阈值轻量同步：5-trial 通过记录

## 实验范围

- 场景：`4+4` 八传感器编队，100 步，GA-LMB 单轮邻域融合。
- Trial seeds：`[2, 3, 4, 5, 6]`。
- 基线：`Periodic full posterior`，沿用同一分支已有 5-trial 周期全量结果。
- 候选方法：`Light-floor dual threshold + dynamic topology`。

## 方法配置

候选方法的核心是把通信节省从“少融合邻居”改为“高频轻量同步 + 少量重更新 + 守卫式拓扑重构”：

- 双阈值触发：
  - `thresholdLow = 0.20`：超过低阈值发送 light moment-matched LMB。
  - `thresholdHigh = 0.3708`：超过高阈值发送 heavy full GM-LMB。
  - 不再强制初始、标签变化或缓存过期 heavy；这些情况由 light 同步覆盖，高阈值仍可触发 heavy。
- 链路门控：
  - 保留 outage 取消发送。
  - `poorLinkThreshold = 0.0`，避免低质量链路额外取消 light；链路质量主要进入拓扑收益。
- 动态拓扑：
  - 固定 16 条无向边预算。
  - 边收益结合链路可靠性、有效标签重叠和几何互补性。
  - 静态边加 `topologyStaticEdgeBonus = 0.35`，避免无收益抖动。
  - 若重构后代数连通度低于静态 4+4 基线，则回退静态拓扑。

## 5-trial 平均结果

| Arm | Bytes | Trigger | Light | Heavy | Edges | Alg. conn. | Local E-OSPA | Consensus OSPA | Position disagreement | Card. dispersion |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Periodic full posterior | 27756443 | 1.000 | 0.000 | 1.000 | 16.0 | 2.000 | 2.0508 | 1.9473 | 4.3464 | 0.1988 |
| Light-floor dual threshold + dynamic topology | 13354518 | 0.977 | 0.863 | 0.114 | 16.0 | 2.035 | 2.0733 | 1.9963 | 4.4218 | 0.2092 |

## 升级门槛审计

| Metric | Requirement | Result | Pass |
|:--|:--|--:|:--:|
| Estimated bytes | reduction >= 30% | 51.89% reduction | yes |
| Local E-OSPA | degradation <= 5% | +1.10% | yes |
| Consensus OSPA | degradation <= 10% | +2.51% | yes |
| Position disagreement | degradation <= 10% | +1.73% | yes |
| Cardinality dispersion | degradation <= 10% | +5.26% | yes |

## Per-seed candidate values

| Seed | Bytes | Trigger | Light | Heavy | Local E-OSPA | Consensus OSPA | Position disagreement | Card. dispersion | Alg. conn. |
|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| 2 | 14378744 | 0.979 | 0.887 | 0.093 | 2.2183 | 2.0636 | 6.2174 | 0.2425 | 2.0509 |
| 3 | 11948088 | 0.970 | 0.870 | 0.100 | 1.9773 | 1.9547 | 3.1937 | 0.1763 | 2.0310 |
| 4 | 13476744 | 0.982 | 0.850 | 0.132 | 2.0590 | 1.9356 | 3.5606 | 0.1950 | 2.0249 |
| 5 | 14253704 | 0.978 | 0.854 | 0.123 | 2.0666 | 2.0178 | 4.6119 | 0.2200 | 2.0261 |
| 6 | 12715312 | 0.976 | 0.854 | 0.121 | 2.0455 | 2.0097 | 4.5252 | 0.2125 | 2.0428 |

## 结论

该配置满足 5-trial 升级门槛。关键原因不是继续稀疏化融合边，而是让接收端大多数时候仍获得邻居后验，只把全量 GM-LMB 降级为轻量矩匹配 LMB；动态拓扑采用守卫式重构，避免收益函数短期波动破坏网络一致性。

后续若升级到 20/50 trials，应优先验证两点：

1. 当前融合路径会对 heavy 和 light 都做矩匹配，因此 heavy 主要是 payload 层级诊断，不显著改变融合结果。
2. 守卫式动态拓扑在当前 4+4 场景下平均代数连通度略高于静态基线，但需要在更强链路衰落或节点失效场景中验证“重构”贡献。
