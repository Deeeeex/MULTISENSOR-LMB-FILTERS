# 定向路由收益归因修正与下一阶段实验协议

## 当前结论

此前在 M24-hard 和 X36-clean-scale 上观察到的
`directed-reliability-w50` 正收益是真实的数值结果，但不能归因于
“动态拓扑”。对 M24/X36、seeds 7/17/27、完整 160 步的逐时审计表明：

- 每个场景和 seed 都只有一个 sender map；
- 窗口内 receiver 换路率为 0；
- 每个六节点编队的 sender map 恒为 `[2,1,1,1,1,1]`；
- 没有跨编队消息；
- 它与新注册的 `directed-fixed-index-w50` 在路由和 KLA 权重上逐项一致。

原因不是后验恰好稳定，而是链路模型和 tie-break 共同决定了固定答案：
同编队边的丢包率恒为 0.02，跨编队边至少为 0.05；最大可靠率出现并列时，
代码选择编号最小的 sender。因此，每个编队的第一个节点固定向其余五个
节点发送，第二个节点固定向第一个节点发送。

报告中此前约为 0.48 的 churn 来自条件续跑边界上的一次
“公共 static prefix → 固定定向图”切换，不是 `t=75–77` 内的在线换路。
新的报告同时给出：

- `prefixBoundaryRouteChangeFraction`；
- `withinWindowRouteChangeFraction`；
- `distinctDirectedRouteMapCount`；
- `fractionReceiverTimeDifferentFromFixedIndexMap`。

因此，当前证据只支持：

> 固定的 receiver-specific 单入边、等权 KLA，比当前无向多邻居
> static/local 对照更适合这两个开发检查点。

它不支持动态拓扑、链路自适应、后验驱动选边或学习贡献。

## 为什么这不是一个坏结果

该发现仍然指出了一个重要的结构问题：在当前 receiver 实现中，
“让每个节点对称融合多个邻居”未必优于“每个 receiver 只接受一个来源”。
但这只是下一阶段必须击败的强固定基线，而不是最终方法。

真正的动态方法至少需要同时回答两个问题：

1. 当前时刻为何要改变 sender 或融合权重；
2. 改变后是否优于同消息数、同字节量级的固定或预定时序策略。

如果只优于无向 static，而不能优于固定星形或简单轮转，就不能把收益归因
于在线决策。

## 已注册的公平对照

所有定向对照均使用 heavy posterior、每个 receiver 一条入消息，并显式
报告实际 attempted bytes。

| 实验臂 | 决策 | 作用 |
|:--|:--|:--|
| `directed-fixed-index-w50` | 每个编队固定 `[2,1,1,1,1,1]` | 当前 reliability 路由的精确冻结副本 |
| `directed-fixed-cycle-w50` | 每个节点固定接收下一个本编队角色 | 消除固定 leader，保持每个 sender/receiver 各一次 |
| `directed-round-robin-w50` | 五步遍历全部本编队 peer | 检验简单时间多样性是否已经足够 |
| `directed-physical-round-robin-p1-w50` | 在当前全部物理可达 sender 中按注册 phase 开环轮转 | 为可能使用跨编队边的候选提供同动作类别时序对照 |
| `directed-reliability-quality-c25-w70` | 当前可靠率加 0.25 倍 sender 质量分数 | 第一条真正读取 posterior 并可能换路的开发候选 |

归因筛查还必须把三个固定/轮转臂以候选相同的 `w=0.70` 再运行一次，
即 `directed-fixed-index-w70`、`directed-fixed-cycle-w70` 和
`directed-round-robin-w70`。否则候选相对 `w=0.50` 的收益可能只来自融合
权重，而不是在线 sender 选择。若候选在 focus window 使用了跨编队边，
还必须运行同权重的 `directed-physical-round-robin-p1-w70`，且该对照在每个
seed 的同一窗口都实际产生跨编队消息。报告中的 dynamic-attribution gate
会检查完整的同权重、同动作类别对照集合；同权重参照在这些臂中逐
seed 取 tracking 最好的一个，用于判断 sender 选择本身是否有效。此外，
最终效果门槛仍会在**所有已注册权重**的有效固定/时序对照中逐 seed 取
tracking 最优者；候选必须也胜过这个更强参照，不能靠选一个较弱的
source weight 获得有利对比。

“每步消息数相同”不保证实际 bytes 相同，因为不同 sender 的 GM-LMB
posterior 大小会随闭环历史变化。直接性能归因要求逐 seed attempted-byte
差异不超过 2%；否则只能比较通信—精度 Pareto 曲线。

## 第一条动态候选

当前注册候选的 sender 分数为

\[
s_{j\rightarrow i}(t)
=q_{ji}(t)+0.25\,[Q_j(t)-Q_i(t)],
\]

其中 \(q_{ji}\) 是链路成功率，\(Q_j-Q_i\) 是只由当前 LMB posterior
摘要得到的 sender 相对质量优势。每个 receiver 选择分数最高的物理
sender，KLA source weight 暂定为 0.70。

需要特别说明：对固定 receiver \(i\) 做 sender argmax 时，\(-Q_i\) 对
全部候选 sender 都是同一个常数，因此当前规则在每个 receiver 都获得一条
消息时，等价于按 \(q_{ji}+0.25Q_j\) 排序。也就是说，它是
**posterior-aware 的来源质量启发式**，还不是 receiver-need-aware 的成对
收益模型；只有消息总预算小于 receiver 数、需要跨 receiver 截断时，
\(-Q_i\) 才会影响接收方优先级。后续若要主张“按接收方需求选边”，必须
加入真正不会在 sender argmax 中抵消的 pairwise 特征或允许 no-message
动作。

系数 0.25 和权重 0.70 只来自 M24 seeds 7/11/17、`t=75` 的开发快照：
在 corrected Bernoulli-risk teacher 上，它是已测试简单公式中唯一在三个
seed 都给出正的瞬时期望风险改善的组合。它仍不是已验证方法，原因包括：

- 单快照不能证明闭环收益；
- 每个 seed 仍有部分 receiver 的动作比固定骨架更差；
- 尚未证明 M24 与 X36 都会在评估窗口内真实换路；
- 尚未超过 fixed cycle 和 round-robin；
- 当前调度读取全网 posterior，控制面字节尚未计入。

最新 D12 seed 7、`t=1–3` 的对抗性检查也说明为何必须加入跨编队时序
对照：候选相对只在编队内通信的三个对照看起来有巨大提升。只比较默认
phase 1 时，E-OSPA 从 29.5893 降到 28.6241，表面上还有 3.262% 增益；
但枚举 physical round-robin 的完整 11-phase 周期以及 `w=0.50/0.70`
后，phase 2、`w=0.70` 已达到 28.1670，反而比候选好 1.6229%。候选同时
不再 tail-safe，且虽然 attempted bytes 更少，却没有 tracking 改善，
所以不构成严格 Pareto 优势。这是单 seed、三步的开发期负例，只用于校准
归因逻辑，不能外推到 M24/X36。

完整 phase 扫描消除了这次 D12 结论对默认 phase 的依赖，但
physical round-robin 仍不是“最强跨编队固定图”。正式归因前还要增加
训练期冻结的跨编队 role/map 对照。当前 gate 最多支持“候选相对已注册
模板”的开发判断，不能表述为已经击败所有跨编队固定策略。

## 动态收益的最低归因门槛

M24 和 X36 必须分别满足，不能互相平均抵消：

1. 具有与候选完全相同 source weight 的 fixed-index、fixed-cycle 和
   round-robin 完整对照；候选若使用跨编队边，还必须具有同权重的
   physical-round-robin 对照。候选既要胜过同权重对照，也要相对所有
   已注册权重的有效固定/时序对照中逐 seed 最强者，平均 E-OSPA 至少
   改善 5%，且每个开发 seed 方向为正；
2. 最差节点不劣于最强对照；
3. 每个 seed 的 attempted-byte 差异不超过 2%，或在共同 byte budget
   曲线上形成严格 Pareto 优势；
4. `distinctDirectedRouteMapCount > 1`；
5. `withinWindowRouteChangeFraction > 0`；
6. 至少 10% receiver-time 动作不同于 fixed-index；
7. receiver coverage 为 100%，物理图和消息预算零违规；
8. 若主张编队间信息交接，必须出现非零跨编队消息，并进一步检查滑动窗口
   formation-level connectivity。

通过开发门后才进入 10-seed screening。seed 27 已用于场景几何审计，
只能作为“方法参数未见、场景已见”的 canary，不能作为最终
scenario-held-out 样本。正式 screening 应使用全新 seeds；任何根据其结果
改方法的 seed 都立即降级为 development。最终确认集应另行预注册，并在
代码、方法参数、时间窗和统计门槛全部冻结后首次打开。

## 可复现入口

- 全时静态性审计：`RUN/GA/auditDirectedReliabilityDynamics.m`
- 冻结审计结果：
  `RUN/GA/dynamic_topology/DIRECTED_RELIABILITY_DYNAMICITY_AUDIT_20260726_083638.md`
- D12 完整 phase 归因负例：
  `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_ORACLE_GAP_D12_HANDOVER_N1_20260726_091625.md`
- 一键配对 runner：`RUN/GA/runDynamicTopologyOracleGapScreen.m`
- 固定/轮转策略：`common/selectRegisteredDirectedRoutingPolicy.m`
- posterior-aware 候选：`common/selectAnalyticDirectedRoutingPolicy.m`
- 回归测试：`tests/test_dynamic_topology_scenarios.m`

本阶段的正确研究问题已经从“reliability 动态路由是否跨尺度有效”改为：

> 在相同的 receiver-specific 单消息动作空间中，posterior-aware 在线路由
> 能否稳定超过固定角色分配和简单时间轮转，并在 M24/X36 上形成可重复的
> tracking—communication 优势？
