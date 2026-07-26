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
总消息数不变，不会因“安全门”造成节点停止接收和 effective graph 断裂。

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

| Seed | 相对 round-robin 的平均风险改善 | 有害 gateway | Gateway 数 | Formation coverage | 字节比 |
|---:|---:|---:|---:|---:|---:|
| 7 | 6.01% | 0 | 3 | 75% | 99.17% |
| 11 | 5.22% | 0 | 2 | 50% | 99.36% |
| 17 | 7.24% | 0 | 2 | 50% | 100.69% |

这里的“风险改善”是 privileged one-step teacher proxy，不是闭环 E-OSPA
改善。它只允许该方法进入短闭环筛查。

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
`RUN/GA/dynamic_topology/FORMATION_GATEWAY_PROXY_AUDIT_20260726_095226.md`。

## 5. 必须比较的强对照

代码注册了同 weight、同一条入消息/receiver 的四类对照：

- intra-formation round-robin；
- fixed gateway ring；
- rotating gateway ring；
- link-aware gateway。

候选 arm 为 `directed-gateway-v1-w50`。固定、轮换和链路对照分别为
`directed-fixed-gateway-p1-w50`、
`directed-rotating-gateway-p1-w50` 和
`directed-link-gateway-w50`。

只有候选在短闭环中同时优于这些对照，并保持零物理违规、完整 receiver
coverage、可接受尾部和真实 route change，收益才可以归因于 posterior-aware
gateway handover。

## 6. 当前不能声称什么

- 不能声称已经改善 M24 或 X36 的闭环 tracking；
- 不能把 t=75 teacher proxy 写成 E-OSPA 或 consensus 结果；
- 不能把 seed 17 称为完全独立的 held-out seed；
- 不能声称 X36 已验证；当前还没有同等级的 X36 teacher-v2 cache；
- 不能声称阈值有理论最优性；v1 参数是开发期探索结果；
- 不能声称总通信量已完整核算；当前字节只覆盖 posterior payload。

下一关是 M24 seed 7 的 t=75–77 配对闭环。只有候选相对最强注册对照达到
至少 5% 的 E-OSPA 改善、尾部不恶化且路由确实变化，才扩展到 M24
seeds 11/17，再进入 X36。
