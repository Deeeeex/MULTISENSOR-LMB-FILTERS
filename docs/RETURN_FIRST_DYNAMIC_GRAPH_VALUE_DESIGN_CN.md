# 基于多步回报的安全动态图价值学习

## 1. 为什么停止精确教师图模仿

此前两版集合提议器都把“复现当前风险教师选出的完整图”作为主要目标。
九个 M24 seed 的证据表明，这个目标与真正关心的闭环跟踪收益并不一致：

- sampled-bank set-softmax 在训练损失下降的同时，精确 projector 回放仍为
  0/81 状态命中；
- 加入全局 hard negative 后，最终训练状态也只命中 3/81；
- 即使完整复现当前风险教师，旧 X36-clean-scale 三步结果的平均收益也只有
  0.961%，不足以达到 5% 的实用门槛。

因此，继续增加同一 additive edge scorer 的容量或 hard-negative 轮数，
只是在改进一个错误的中间任务。新方法不再把某一张教师图当成唯一正确答案，
而是直接学习“在当前网络状态下执行候选图后，未来三步跟踪会改善多少”。

## 2. 先验证动作空间，而不是先训练模型

学习前必须确认 exact rolling-safe 可行集中同时存在 M24 和 X36 的有效动作。
冻结的开发筛查使用：

| 项目 | 设置 |
|:--|:--|
| 场景 | `m24-hard`、`x36-clean-scale` |
| 共享前缀 | 静态行为运行至 `t=75` |
| 条件窗口 | `t=75:77` |
| 融合 source weight | 0.50 |
| 强基线 | `directed-fixed-index-w50` |
| 安全层 | exact rolling-\(B=3\) projector |
| 通信形式 | 每个 receiver 每步恰好一个邻居 posterior |
| 诊断上限 | current-risk 与 minimax-risk privileged arms |

privileged arms 可以读取当前真值，只用于回答“这个可行集是否有上限”，
不能作为部署方法、验证结果或论文主结果。

相对同权重 fixed-index 强基线，动作空间通过的必要条件为：

1. 平均 E-OSPA 至少改善 5%；
2. 最差节点 E-OSPA 不退化；
3. attempted bytes 偏差不超过 2%；
4. 所有成熟 rolling-\(B=3\) 窗口强连通；
5. 不出现 topology infeasibility、safety emergency 或 policy repair。

候选的平均与最差节点 E-OSPA 还必须同时不差于 local-only。这样可以避免
在融合本身有害时，仅凭“比一个更差的通信基线好”制造虚假的拓扑收益。

M24 和 X36 都通过后，才允许生成回报数据和训练模型。若 M24 通过而 X36
失败，不得直接训练再用模型容量解释失败；应先扩展 joint action，例如把
receiver-specific source weight 或 local fallback 纳入动作，再重新测上限。

## 3. 拟采用的图级方法

模型输入由部署时可获得的信息构成：节点 posterior 摘要、相对几何、当前
链路可靠性、sender payload 大小，以及最近两步已选择/实际送达的有向图。
输入不含目标真值和未来链路结果。

模型不是独立地给每条边打分后取 top-k，而是对完整候选图
\(G_t\) 估计三步价值：

\[
Q_\theta(s_t,G_t)
= \mathbb{E}\!\left[
L_{\mathrm{ref},t:t+2}-L_{G_t,t:t+2}
\mid s_t,G_t
\right].
\]

图编码器分别汇聚节点、候选边和已选边，并加入已选边之间的交互项。这样，
同一条边的价值可以随“图里已经选了哪些边”而改变，避免 additive edge
energy 无法表达完整图组合效应的问题。聚合采用 permutation-invariant
sum/mean/max 统计，使 M24 与 X36 共用参数，不依赖固定节点编号或固定节点数。

推理时使用小宽度 beam search 逐步构造候选图；每个完整候选都必须经过
同一个 exact rolling-\(B=3\) projector。模型负责比较跟踪价值，projector
负责通信预算、物理可达性和滚动连通安全，两者职责分离。

## 4. 训练与证据顺序

1. 对每个冻结的 predecision 状态生成多张安全候选图；
2. 用 common random numbers 回放候选图与 fixed-index reference 的 H=3
   闭环结果；
3. 以 paired E-OSPA 改善为主标签，同时记录最差节点、通信量和安全状态；
4. 按完整 seed 切分训练、校准和测试，禁止相邻时间块跨 split；
5. 先要求候选 oracle 通过，再检查 critic 的 held-out 排序和闭环选择；
6. M24 held-out 通过后，冻结模型与搜索参数，才运行 X36；
7. paper-level 结果最后使用至少 30 个 paired seed，并报告置信区间。

这一路线的理论与工程贡献分别是：用置换不变的图级价值函数处理节点规模
变化和边间交互；用 exact projector 给学习动作提供可审计的滚动连通保证。
最终贡献是否成立仍由 M24/X36 的 held-out 闭环结果决定，而不是由训练损失
或教师图命中率决定。
