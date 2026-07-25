# D12 动态拓扑三次配对筛查

- 数据：`RUN/GA/dynamic_topology/full_n3/DYNAMIC_TOPOLOGY_ORACLE_GAP_D12_HANDOVER_N3_20260725_153939.mat`
- 可追踪逐臂记录：`RUN/GA/dynamic_topology/full_n3/DYNAMIC_TOPOLOGY_RECORDS.csv`
- Seeds：`[7 17 27]`
- 重点窗口：handover，`[35 95]`
- 自动分类：`diagnostic-oracle-dominated`
- 解释：相对 geometry-static 存在候选动态信号，但两个一步诊断 oracle 在每个 seed 的 tracking 与 posterior 指标上都被 discrepancy 支配。它们不能充当上界或 GNN 教师，也不能据此推出解析策略已经充分；应暂停 GNN，先重做短时闭环价值定义并补齐最强固定图基线。 这是 N<=3 的方向信号，不是推断性证据。

## 重点窗口的主比较

| Seed | Geometry-static focus E-OSPA | Discrepancy focus E-OSPA | Tracking 改善 | Static posterior 分歧 | Discrepancy posterior 分歧 | Consensus 改善 | Bytes 偏差 |
|--:|--:|--:|--:|--:|--:|--:|--:|
| 7 | 68.5432 | 62.2636 | 9.16% | 0.4509 | 0.4111 | 8.82% | 1.37% |
| 17 | 65.3521 | 61.1697 | 6.40% | 0.4230 | 0.3853 | 8.91% | 0.61% |
| 27 | 67.6182 | 64.4098 | 4.74% | 0.4665 | 0.3987 | 14.55% | 0.20% |

## 每个 seed 的完整四臂记录

| Seed | Arm | Focus E-OSPA | Focus posterior 分歧 | Attempted bytes | 相对 static bytes | Churn | 不同候选数 |
|--:|:--|--:|--:|--:|--:|--:|--:|
| 7 | All-time geometry static | 68.5432 | 0.4509 | 35938264 | 0.00% | 0.0000 | 0 |
| 7 | Posterior discrepancy dynamic | 62.2636 | 0.4111 | 36431560 | 1.37% | 0.1320 | 34 |
| 7 | Exact one-step consensus oracle | 69.2545 | 0.4486 | 35917936 | 0.06% | 0.0076 | 6 |
| 7 | Exact one-step truth diagnostic oracle | 63.9923 | 0.4603 | 35952904 | 0.04% | 0.0000 | 1 |
| 17 | All-time geometry static | 65.3521 | 0.4230 | 35776528 | 0.00% | 0.0000 | 0 |
| 17 | Posterior discrepancy dynamic | 61.1697 | 0.3853 | 35994952 | 0.61% | 0.1333 | 35 |
| 17 | Exact one-step consensus oracle | 65.9283 | 0.4181 | 35774248 | 0.01% | 0.0076 | 5 |
| 17 | Exact one-step truth diagnostic oracle | 65.4729 | 0.4207 | 35741752 | 0.10% | 0.0061 | 5 |
| 27 | All-time geometry static | 67.6182 | 0.4665 | 35951944 | 0.00% | 0.0000 | 0 |
| 27 | Posterior discrepancy dynamic | 64.4098 | 0.3987 | 36022648 | 0.20% | 0.1347 | 28 |
| 27 | Exact one-step consensus oracle | 67.2261 | 0.4588 | 36053632 | 0.28% | 0.0015 | 2 |
| 27 | Exact one-step truth diagnostic oracle | 67.2261 | 0.4588 | 36053632 | 0.28% | 0.0015 | 2 |

## 配对汇总

- Posterior-discrepancy vs static：
  - focus E-OSPA 平均改善：6.77%，胜出 3/3 seeds；
  - focus posterior 分歧平均改善：10.76%，胜出 3/3 seeds；
  - attempted-byte 平均/最大偏差：0.73% / 1.37%；
  - topology 不可行率上界：0.0000。
  - 候选策略平均 churn / 不同候选数：0.1333 / 32.3。
- Consensus oracle vs posterior-discrepancy：
  - focus E-OSPA 平均改善：-7.79%，胜出 0/3 seeds；
  - focus posterior 分歧平均改善：-10.90%，胜出 0/3 seeds；
  - attempted-byte 平均/最大偏差：0.70% / 1.41%；
  - topology 不可行率上界：0.0000。
  - 候选策略平均 churn / 不同候选数：0.0056 / 4.3。
- Truth oracle vs posterior-discrepancy：
  - focus E-OSPA 平均改善：-4.73%，胜出 0/3 seeds；
  - focus posterior 分歧平均改善：-12.08%，胜出 0/3 seeds；
  - attempted-byte 平均/最大偏差：0.70% / 1.31%；
  - topology 不可行率上界：0.0000。
  - 候选策略平均 churn / 不同候选数：0.0025 / 2.7。

## 四个 arm 的均值

| Arm | Focus E-OSPA | Focus posterior 分歧 | Attempted bytes | Churn | 不同候选数 | 不可行率 | 总时间 s |
|:--|--:|--:|--:|--:|--:|--:|--:|
| All-time geometry static | 67.1712 | 0.4468 | 35888912 | 0.0000 | 0.0 | 0.0000 | 158.87 |
| Posterior discrepancy dynamic | 62.6143 | 0.3984 | 36149720 | 0.1333 | 32.3 | 0.0000 | 222.15 |
| Exact one-step consensus oracle | 67.4696 | 0.4418 | 35915272 | 0.0056 | 4.3 | 0.0000 | 821.62 |
| Exact one-step truth diagnostic oracle | 65.5638 | 0.4466 | 35916096 | 0.0025 | 2.7 | 0.0000 | 822.21 |

## 证据边界

- 三次 trial 只用于判断方向，不提供可靠置信区间，也不能支持论文级显著性主张。
- “Exact one-step oracle”只是在每一步精确枚举动作，不是整段闭环跟踪的全局性能上界。
- 当前 static 是按全时段几何距离选择的可行固定图，尚未穷举 48 个固定候选的 held-out tracking 表现；因此当前结果不能证明动态策略击败了最强静态基线。
- 两个一步诊断策略只访问了很少的不同候选图；当它们被可部署启发式支配时，不能把“负 oracle gap”解释成解析策略最优。
- 融合使用 componentwise powered-GM 近似；它保留多模态，但不是任意 Gaussian mixture 幂的精确闭式实现。
- attempted bytes 仅包含 payload；控制、ACK 与拓扑协商开销仍需在后续系统实验中单列。
