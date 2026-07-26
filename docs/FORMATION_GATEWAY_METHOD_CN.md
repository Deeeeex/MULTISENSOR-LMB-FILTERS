# 编队网关动态路由：当前方法与证据边界

## 1. 为什么改变方法

此前的 receiver-complete reliability 路由在 M24 和 X36 上有明显数值收益，
但全时段审计证明它始终使用同一张逐编队星形 sender map，没有跨编队消息，
因此只能说明“稀疏有向融合有价值”，不能说明动态拓扑有效。

在此基础上直接加入 source quality 也不成立。对每个 receiver 都接收一条消息
的策略而言，sender argmax 中的加性 receiver need 会抵消；M24 teacher cache
进一步显示，source quality 与真实单步收益的关系弱且不稳定。裸的逐 receiver
pairwise 公式虽有较高平均收益，却会选择少数严重有害的融合动作。

因此，当前方法不再让所有 receiver 自由换边，而是把动态决策限制为
**少量、可回退的跨编队信息交接**。

## 2. 方法结构

每个时刻先构造一张 balanced intra-formation round-robin 路由：
每个 receiver 恰好接收一条同编队 posterior。在线策略只判断是否将其中少量
路由替换为跨编队 gateway。

对 receiver \(i\)、物理可达 sender \(j\) 和 source weight \(\alpha\)，
当前 v1 utility 为

\[
U_{ij} =
q_{ji}\alpha \Delta r^+_{ij}
-50\alpha\Delta P^-_{ij}
-8\alpha^2 O_{ij}D_{ij},
\]

其中：

- \(q_{ji}\) 是当前链路成功概率；
- \(\Delta r^+_{ij}\) 表示 sender 相对 receiver 新增的目标存在信息；
- \(\Delta P^-_{ij}\) 表示 sender 在共同标签上较差的精度；
- \(D_{ij}\) 是共同标签的状态差异；
- \(O_{ij}\) 是有效标签重叠程度。

只有同时满足以下条件时，跨编队动作才可替换默认动作：

1. \(\Delta r^+_{ij}\ge 0.05\)，即 sender 必须带来足够的绝对信息新颖性；
2. \(U_{ij}-U_{i,j_{\mathrm{RR}}}>0.02\)，即它必须明显优于同权重默认路由；
3. 每个编队至多一个 gateway receiver；
4. 每个 sender 至多承担一个跨编队 gateway；
5. 所有动作必须属于当前物理可达图。

未通过门槛时保留 round-robin。因而每个时刻仍是每个 receiver 一条消息，
总消息数不变。这里保护的是 receiver coverage 和消息预算，不等价于保护编队级
信息流连通性；跨编队边仍可能只覆盖局部编队。后者必须单独检查，不能由
“每个 receiver 有一条入消息”推出。

## 3. 为什么同时需要绝对新颖性和相对 margin

只用相对 margin 会出现一个系统性错误：某条跨编队边可能只是比一个很差的
默认边稍好，自身并没有提供有用信息。M24 的反例中，这类动作具有正 utility
margin，却产生负 teacher residual。

绝对新颖性门槛排除了这种“相对更好但绝对无用”的替换。它与 margin 的作用
不同：

- 新颖性回答“是否真的带来新的目标信息”；
- margin 回答“相比本时刻默认接收对象是否值得切换”。

这是本轮代理审计中最重要的方法设计发现。

## 4. 当前代理证据

下表来自 M24-hard、t=75 的 corrected Bernoulli-risk teacher cache。
seeds 7/11 用于候选排序；seed 17 的标签不进入代码中的 selection key。
但整个研究过程已经检查过 seed 17，因此三者都应视为开发证据，不是最终
held-out 结果。

| Seed | 相对 round-robin 的平均风险改善 | 有害 gateway | Gateway 数 | Formation coverage | 编队图弱连通 | 字节比 |
|---:|---:|---:|---:|---:|:--:|---:|
| 7 | 6.01% | 0 | 3 | 75% | 否 | 99.17% |
| 11 | 5.22% | 0 | 2 | 50% | 否 | 99.36% |
| 17 | 7.24% | 0 | 2 | 50% | 否 | 100.69% |

这里的“风险改善”是 privileged one-step teacher proxy，不是闭环 E-OSPA
改善。修正后的代理门要求编队图至少弱连通，因此 training、validation 和
all-observed 三个 gate 均为 0。表中的局部风险信号可用于诊断，但不能再说
v1 通过代理门。

可复现实验：

```matlab
options = struct( ...
    'sourceWeightGrid', 0.50, ...
    'baselinePhaseGrid', 1, ...
    'positiveExistenceWeightGrid', 1, ...
    'positivePrecisionWeightGrid', 0, ...
    'negativeExistencePenaltyGrid', 0, ...
    'negativePrecisionPenaltyGrid', 50, ...
    'discrepancyPenaltyGrid', 8, ...
    'receiverNeedMultiplierGrid', 0, ...
    'marginThresholdGrid', [0.02, 0.025, 0.03], ...
    'minimumPositiveExistenceGrid', [0.04, 0.05, 0.06]);
auditPairwiseDirectedRoutingProxy([], options);
```

对应报告：
`RUN/GA/dynamic_topology/FORMATION_GATEWAY_PROXY_AUDIT_20260726_102218.md`。

## 5. 必须比较的强对照

代码注册了同 weight、同一条入消息/receiver 的四类对照：

- intra-formation round-robin；
- fixed gateway ring；
- rotating gateway ring；
- link-aware gateway。

候选 arm 为 `directed-gateway-v1-w50`。完整对照必须包含
`directed-fixed-gateway-p1-w50` 至 `p6`、
`directed-rotating-gateway-p1-w50` 至 `p6`，以及
`directed-link-gateway-w50`。只跑 phase 1 不能称为完整对照。

只有候选在短闭环中同时优于这些对照，并保持零物理违规、完整 receiver
coverage、可接受尾部和真实 route change，收益才可以归因于 posterior-aware
gateway handover。

## 6. 当前不能声称什么

- 不能声称 v1 已经改善 M24 或 X36 的闭环 tracking；
- 不能把 t=75 teacher proxy 写成 E-OSPA 或 consensus 结果；
- 不能把 seed 17 称为完全独立的 held-out seed；
- 不能声称 X36 已验证；当前还没有同等级的 X36 teacher-v2 cache；
- 不能声称阈值有理论最优性；v1 参数是开发期探索结果；
- 不能声称总通信量已完整核算；当前字节只覆盖 posterior payload。

M24 seed 7 的 t=75–77 配对闭环已经完成并失败，因此 v1 不扩展到 M24
seeds 11/17，也不进入 X36。

## 7. M24 短闭环判定

M24 seed 7、t=75–77 的 15 臂配对闭环已经完成，包括 fixed gateway 和
rotating gateway 各六个规范相位。最强注册对照是 phase-1
intra-formation round-robin。

| Arm | E-OSPA | Worst node | Posterior disagreement | MAP-set disagreement | Attempted bytes |
|:--|--:|--:|--:|--:|--:|
| Round-robin | 22.3297 | 34.8109 | 0.8982 | 24.2645 | 3,471,264 |
| Best fixed gateway, p1 | 23.4224 | 41.7655 | 0.8664 | 26.9219 | 3,236,208 |
| Best rotating gateway, p3 | 23.5871 | 42.7172 | 0.8710 | 27.8259 | 3,251,136 |
| Corrected link-aware gateway | 22.3611 | 37.9768 | 0.8602 | 24.8995 | 3,337,872 |
| Novelty-gated gateway v1 | 21.9550 | 35.1839 | 0.8816 | 25.9232 | 3,409,320 |

v1 相对最强对照只改善 1.6781%，低于 5% 门槛；worst node 从 34.8109
升至 35.1839，也没有通过严格尾部门。它的 attempted bytes 少 1.78%，
三个时刻均保持完整 receiver coverage、零物理违规，并产生三张不同 route
map 和 9.72% 跨编队消息。跨编队图每个时刻都不弱连通，但三步 union
弱连通；因此它通过了当前 joint-connectivity 结构门，最终仍因收益和尾部门
失败。

逐时刻 E-OSPA 更能说明问题：

| Time | Round-robin | Gateway v1 | 相对改善 |
|---:|---:|---:|---:|
| 75 | 22.081 | 21.710 | +1.68% |
| 76 | 21.683 | 23.877 | -10.12% |
| 77 | 23.225 | 20.278 | +12.69% |

单步 teacher proxy 在 t=75 给出的 6.01% 风险改善没有等比例转化为实际
E-OSPA；随后又出现一差一好的交替。这表明当前 surrogate 与 tracking
metric 对齐不足，并且逐时刻贪心 gateway 会引入明显的闭环反馈波动。
posterior disagreement 改善 1.85%，但 MAP-set disagreement 恶化 6.84%，
也说明 moment-level 接近并不等于离散目标集合更一致。

候选的 policy time 为 16.98 s，而 round-robin 接近 0；总运行时间分别为
42.53 s 和 25.90 s。即使忽略统计强度，v1 也没有形成足以抵消额外决策开销
的实际收益。

因此 v1 在此停止，不运行 M24 其他 seeds，也不进入 X36。下一版不能只调低
或调高现有阈值，而应改为：

1. 用相对强 round-robin 的多步 realized residual 作为训练目标；
2. 直接把 E-OSPA 与 worst-node/CVaR 约束纳入标签；
3. 用 lower-confidence-bound 决定是否 override；
4. 加入 dwell time 或切换迟滞，抑制逐时刻贪心反馈；
5. 在多个 seed、多个相隔时间块上训练/校准后，才重新做闭环筛查。

闭环报告：
`RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260726_104151.md`。

## 8. 从弱基线树到强基线残差

后续的 connected-tree v2 证明，硬投影到逐时刻弱连通的 rooted formation
tree 可以避免 v1 的编队图断裂。在 M24 的三个开发 seed 上，它相对
intra-formation round-robin 有明显均值改善；在 X36 seed 7 上也把
E-OSPA 从 44.1161 降到 42.5002，并把 worst node 从 61.2646 降到
56.3264。

但这不能作为最终贡献。固定索引星形是更强的同预算编队内对照；一旦把它
纳入，connected-tree v2 相对 round-robin 的收益主要包含了 backbone
差异。X36 的 3.6629% 也低于 5% 门槛。因此，v2 只证明了“连通安全投影
有用”，没有证明动态打分超过强固定路由。

## 9. v3：每步强连通环

strong-cycle v3 保留固定索引星形 backbone，每个编队只替换一个 receiver，
并把跨编队边精确投影成有向 Hamiltonian cycle。每个时刻的 formation graph
因此强连通；高置信边使用主 source weight，纯连通边使用较小 bridge
weight。

M24 seed 7、t=75–77 的同权重比较为：

| Arm | E-OSPA | Worst node | Attempted bytes |
|:--|--:|--:|--:|
| Fixed-index, w=0.70 | 19.2470 | 34.6420 | 3,512,112 |
| Strong cycle, bridge=0.05, w=0.70 | 18.5724 | 34.6420 | 3,507,576 |
| Strong cycle, bridge=0.10, w=0.70 | 18.5164 | 34.6420 | 3,505,704 |

最佳设置只改善 3.80%，仍低于 5%。把 bridge 从 0.10 增至
0.15/0.20/0.30 后，E-OSPA 依次回退到 18.6705/19.1942/19.4955，
且较大 bridge 开始恶化尾部。其原因不是优化器不够强：t=75 的
privileged 标签显示，每步强环在 seed 7 上被迫包含一条负 residual 边，
使 oracle 环上界也只有 2.88%。

因此，“每步强连通”约束过强；它把连通安全变成了不可避免的有害融合。

## 10. v4：两步联合强连通树

joint-tree v4 采用以下结构：

1. 固定索引星形作为强行为基线；
2. 当前时刻只发送 \(G-1\) 条跨编队树边；
3. 树投影同时满足 receiver coverage、跨边 sender load 不超过 1；
4. 当前树与上一时刻 formation graph 的并图必须强连通；
5. 因此单步只要求弱连通，两步窗口保护联合信息流。

树投影不是 pairwise top-1 贪心。代码会在每个候选 formation
arborescence 内精确求解受 sender 唯一性约束的 sensor endpoint 匹配，
避免“两个局部最优边共用 sender 后整棵树被误拒”的错误。

t=75 的 privileged 最优树相对固定索引 w=0.70 的一步风险改善为：

| Seed | Oracle tree gain | 最差所选 residual |
|---:|---:|---:|
| 7 | 7.27% | 正 |
| 11 | 15.90% | 正 |
| 17 | 9.93% | 正 |

M24 seed 7 的三步闭环结果进一步确认这个动作空间有效：

| Arm | E-OSPA | Worst node | Attempted bytes | Union strong |
|:--|--:|--:|--:|:--:|
| Fixed-index, w=0.70 | 19.2470 | 34.6420 | 3,512,112 | 否 |
| Analytic v4 | 19.2343 | 34.6420 | 3,256,920 | 是 |
| Privileged v4 | 17.9780 | 34.6420 | 3,453,696 | 是 |

privileged v4 相对强基线改善 6.59%，且尾部不退化、通信量下降 1.66%；
analytic v4 只改善约 0.07%。这把当前缺陷定位为 **edge scorer 不足**，
而不是图动作空间或闭环机制无效。

## 11. 当前学习证据

此前的 teacher-series 结论已经撤回。原因是旧生成器在 \(t>t_0\) 时直接
读取 `topologyActiveEdge(:,:,t-1)` 作为策略的上一时刻邻接矩阵，但前者
采用“sender 行、receiver 列”，策略接口采用“receiver 行、sender 列”。
遗漏转置会把 previous-edge 特征以及 previous/current 联合强连通约束整体
反向。因此，旧 `directed_teacher_oracle_v4_*` 数据、基于它得到的
ridge/kNN/structured-ranking 数值都不能继续作为学习失败或成功的证据。

修正版生成器和在线 filter 现在共用同一个方向转换函数，并用独立的
`ctxv2` 文件名和
`directed-teacher-series-v2-receiver-row-previous-adjacency` contract
隔离旧缓存。训练器还会强制检查：完整 seed 级 train/validation 隔离、
至少两个训练 seed、fixed-index-star 基线、previous/current 联合强连通、
训练与验证 proxy gate、truth-free action feature，以及 artifact
provenance。旧文件不会被静默复用。

这里还要区分两种“使用真值”。投影时直接读取的 action feature
确实不含真值；但是 privileged-v4 行为会改变后续 posterior 状态，所以
训练样本的**状态分布**仍由真值辅助的 teacher 产生，不能再表述为
“真值只进入标签”。这种数据最多支持 proxy/DAgger 开发，不能代替
无真值闭环验证。

修正版数据尚未完成重生成，因此当前只能保留“v4 动作空间具有
truth-assisted 闭环信号”这一架构判断，不能声称已有 deployable learned
v4，也不能依据旧 proxy 结果判断 ridge、kNN 或结构化排序孰优。下一步先
重生成独立 seed 的 `ctxv2` 数据，报告 exact projector 对 5% 门槛是否
可达，再冻结候选并与 fixed-index、link-tree 及完整注册的 cyclic-path
root/receiver-phase 控制族做无真值闭环比较。
