# V285：好来源不在当前近邻中，但盲选低协方差也不可靠

## Question

新增匹配目标的位置误差能否通过传递网络中已有的同标签信息来改善？

## Scope

复用 V284 单个 X36 种子的 40 步保存结果，包含全部 7,850 个新增匹配
位置案例。查询按同一标签查找来源；不使用真值跨标签拼接，不重新滤波，
不创建消息、不改变已有联合筛选。观察范围仅是已输出的分量均值和协方差。

## Risk Tier

L2 exploratory, self-check only。这是来源可用性与乐观上限分析，不是可部署
策略或新的性能结果，不支持升级论文主表与 Lark 最优策略表。

## Claims

| ID | 发现 | 证据 | 边界 |
| --- | --- | --- | --- |
| C1 | 当前选中入邻居、全部物理一跳邻居与全网同标签来源之间，有明显的空间信息差。 | E1, E2 | 按真值挑来源并允许保留自身；没有执行消息或融合。 |
| C2 | 差距在上一轮输出预测一步后仍然存在，不只依赖当前轮末信息。 | E2 | 预测不含当前新测量，也不包含多跳延迟、丢包与费用。 |
| C3 | 最小分量协方差不能单独构成可靠的全网选源规则，会伤害前几个编队。 | E2, E3 | 查询集是事后选定的，且分量协方差不代表完整 GM 后验风险。 |

## Evidence Ledger

E1：`common/buildDynamicTopologyGraphs.m` 的物理边条件是传感器距离不超过
通信范围。分析复用保存的轨迹和原 270 m 范围；全部实际发送边都在该范围
内，没有更改场景或重生成测量。

E2：执行 `V285_SPATIAL_RECOVERY_DECISION.md` 中的完整命令，基础分析
session 10096、物理一跳扩展 session 65950 均退出 0。扩展输出为：

```text
V285 complete: 7850 added-target queries, 78340 source-pool records; no filter or candidate run.
lag 0 global same-label pooled RMSE: receiver 34.730995, oracle 11.104842, min-trace 27.303868.
lag 1 global same-label pooled RMSE: receiver 34.786640, oracle 11.083094, min-trace 28.039628.
```

以下是在同一时间层内的理想选择误差，不是某个新算法达到的误差：

| 来源池 | 当前轮末：理想汇总 RMSE / m | 上一轮预测一步：理想汇总 RMSE / m |
| --- | ---: | ---: |
| 保持接收端当前输出 | 34.731 | 34.787 |
| 实际到达的入邻居 | 33.964 | 34.380 |
| 计划发送的入邻居 | 33.908 | 34.336 |
| 同编队来源 | 33.150 | 33.505 |
| 所有物理一跳来源 | 29.656 | 29.979 |
| 全网同标签来源 | 11.105 | 11.083 |

当前轮末层有 7,850 案例；上一轮预测层排除没有过去状态的第一步，共
7,818 案例。不能把两列差异当成同一组案例上的时效改善。

E3：同目录汇总 CSV 和报告保留了全部编队结果。上一轮预测层的全网
最小分量协方差选择，把总体汇总 RMSE 从 34.787 降到 28.040，但 49.09%
的案例变差；编队 1、2 分别从 19.500/13.350 变为 24.043/20.916。
它不是可以直接上线的可靠选源规则。

输出目录为 `evidence/tracking_aligned_v285/x36_same_label_spatial_availability_seed1301/`。
完整查询与来源记录保存在本地 MAT，全部来源池和编队聚合保存在追踪的 CSV。

## Verification Record

Self-check only。查询重新匹配所得的每个节点--时刻 RMSE，与已保存的官方
值最大差异为 1.42e-14 m，未触发原求解器回退。物理一跳池加入后，既有
四个池的结果不变。没有独立验证、递归效果验证或新通信预算执行。

证据格式检查（不是科学结论的独立验证）：

```sh
python3 /Users/dex/.codex/skills/auto-research/scripts/evidence_lint.py RUN/GA/dynamic_topology/V285_SOURCE_AVAILABILITY_FINDING.md
```

输出：`PASS: RUN/GA/dynamic_topology/V285_SOURCE_AVAILABILITY_FINDING.md`。

## Risk and Escalation

理想选源知道真值，并能无代价选择来源，还允许保留当前输出，因而乐观。
当前轮末估计不可能反过来服务同轮更早的决策。上一轮来源较早可得，但
实际跨编队传递仍需要时间、带宽以及完整后验融合，可能再次影响目标数量。
若要写成算法贡献，必须先有可执行动作、完整成本和固定路由对照。

## Reproducibility

复现命令、时间层、协方差语义和物理池构造记录在设计及脚本中。无需重跑
V284、改实验种子或重新生成场景。矩阵 A、过程噪声 R 沿用当前场景工厂的
常速模型；分析没有向滤波器或控制器提供真值。

```sh
octave --no-gui --quiet --eval "addpath(genpath(pwd)); analyzeSameLabelSpatialAvailabilityV285('RUN/GA/dynamic_topology/evidence/tracking_aligned_v284/x36_prefix40_seed1301/UNTOUCHED_PRIOR_EXCLUSION_V284.mat');"
```

## Open Issues

未检查未输出标签和隐藏 GM 分量的全部信息；当前结果不能证明完整网络
后验不存在其他来源。完整混合不确定性、来源关联可靠性以及多跳时间/预算
约束仍未解决。这里的汇总 RMSE 也不是官方平均逐时刻 RMSE。

## Recommendation

不重新启动单边、三步局部选边或盲目最小协方差方案。下一步先以完整混合
后验对当前输出位置的期望平方误差替代“只看选中分量协方差”的判据，
检查可靠选源是否有支持，再把它接入有时效与预算约束的跨编队传递动作。
这只是下一方法设计的依据，不是已经建立的新理论或已验证的新主线算法。
