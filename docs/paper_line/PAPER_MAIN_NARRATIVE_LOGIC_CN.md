# 论文主叙事逻辑整理

本文档用于统一当前论文的主叙事逻辑。目标不是重复技术细节，而是回答三个更关键的问题：

- 这篇论文真正要讲的核心问题是什么。
- 为什么当前方法值得写成一条完整主线。
- 正文各部分应该如何服务这条主线，哪些内容必须降级为 supporting evidence 或 appendix。

相关材料可配合阅读：

- [00_positioning.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/00_positioning.md)
- [01_abstract_and_title.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/01_abstract_and_title.md)
- [05_method_adaptive_kla.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/05_method_adaptive_kla.md)
- [07_results_and_ablation.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/07_results_and_ablation.md)
- [08_conclusion.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/08_conclusion.md)

## 一句话版本

这篇论文的主叙事不是“我们发明了一种全新的多目标滤波器”，而是：

**在异构通信约束下，固定 KLA 融合权重过于僵硬；如果让权重同时反映后验质量、实际链路质量和目标存在性决断度，就能更有效地提升分布式融合的一致性，而弱结构感知的 branch-decoupled refinement 可以在此基础上再做最后一层稳定增益。**

## 这篇论文真正要回答的问题

论文真正的中心问题应表述为：

**在 unknown cross-correlation 的前提下，分布式 GA-LMB / KLA 融合仍然是合理框架，但固定权重或纯拓扑权重无法适应异构通信与时变后验质量；那么，怎样设计 fusion weights，才能让网络中的各节点更接近同一个 fused picture？**

这里有三个关键词不能丢：

- `distributed KLA-based LMB fusion`
- `communication-constrained / heterogeneous communication`
- `consensus-oriented`

如果缺少其中任何一个，主线都会跑偏：

- 少了 `KLA-based LMB fusion`，文章会看起来像泛泛的通信自适应加权。
- 少了 `communication-constrained`，方法动机就不够强。
- 少了 `consensus-oriented`，实验指标和论文标题就会失焦。

## 主矛盾应该怎么讲

整篇论文的叙事张力，来自下面这组矛盾，而不是来自“我们又加了几个模块”：

1. KLA/GA 融合适合 unknown cross-correlation 的分布式场景。
2. 但传统 fixed / uniform / Metropolis 权重默认各节点贡献相近，过于静态。
3. 在异构通信条件下，节点的有效价值并不只由本地估计精度决定，还取决于信息是否被可靠送达。
4. 即使只看 posterior quality，covariance 也不足以表达一个节点在 target existence 上是否足够 decisiveness。
5. 所以真正需要自适应的不是滤波家族本身，而是 fusion weight allocation。

这就是论文的核心冲突。正文要不断围绕这组矛盾往下推进，而不是在不同模块之间平均分配篇幅。

## 核心论点链条

建议把全文的逻辑压缩成下面 5 句话。它们基本就是整篇论文的 argument spine。

1. 分布式 GA-LMB / KLA 融合在 unknown cross-correlation 下是合理且保守的基线。
2. 固定权重在异构通信网络中是脆弱的，因为它既不反映 posterior informativeness，也不反映 realized communication quality。
3. 有效的 adaptive weighting 至少要结合 `covariance + realized link quality + existence confidence` 三个因素。
4. 在这个三因子 backbone 之上，`weak structure-aware decoupled KLA` 能提供小但稳定的进一步收益。
5. 这种收益主要体现在跨节点 `consensus` 指标上，同时没有以明显损伤常规 local tracking quality 为代价。

其中第 3 句和第 4 句的主次关系必须清楚：

- 真正的 backbone 是三因子加权。
- `weak structure-aware decoupling` 是 refinement，不是方法本体。

## 为什么论文必须是 `consensus-oriented`

这是当前主叙事里最关键的 framing，不只是指标选择问题，而是论文“成功标准”的定义问题。

论文必须明确说明：

- 本文研究的是 distributed fusion，而不是单节点滤波器精度优化。
- 在 distributed fusion 问题里，更重要的问题是不同节点能否收敛到一致的多目标图景。
- 因此 `consensus OSPA`、`consensus RMSE`、`consensus cardinality disagreement` 应该是 `primary outcome`。
- 常规 `local E-OSPA / RMSE / cardinality error` 不应被删掉，但它们的角色是 `safeguard`，用于证明一致性提升不是靠 local collapse 换来的。

换句话说，论文不是在说：

“我们的 local tracking 指标全面更强，所以方法更好。”

而是在说：

“在分布式融合问题里，我们优先改善跨节点一致性，并验证这种改善没有显著破坏常规跟踪质量。”

这也是标题里保留 `consensus-oriented` 的根本原因。

## 方法叙事应该怎么展开

方法部分不应该写成“模块列表”，而应该写成“从问题矛盾自然推出设计”。

推荐顺序如下。

### 1. 先承认 KLA 基线合理

先明确：

- 本文不试图推翻 GA/KLA。
- unknown cross-correlation 下，KLA 仍然是合理的 conservative fusion rule。
- 真正需要改进的是 weight allocation，而不是另起一个全新滤波器家族。

这样做的好处是，读者不会误以为论文在过度 claim。

### 2. 先给 shared backbone，再谈 refinement

方法主干应该首先落在：

- `covariance`
- `realized link quality`
- `existence confidence`

因为这三项分别回答三个不同问题：

- 哪个节点后验更集中。
- 哪个节点的信息更可靠地送到了邻居。
- 哪个节点在目标存在性判断上更果断。

这三项构成方法的必要主线。

### 3. 把 existence confidence 讲成“缺失维度”

`existence confidence` 不是普通附加项，正文必须把它讲成三因子里的关键第三维：

- `covariance` 只反映空间集中度，不反映 cardinality / existence decisiveness。
- `link quality` 只反映信息传输可靠性，不反映 posterior content 本身是否 decisively supports existence。
- 因此 `existence confidence` 补上了此前两者共同缺失的一块。

这是方法里最值得被突出解释的新意之一。

### 4. decoupling 和 structure-aware 必须降一层表述

`branch-decoupled` 和 `weak structure-aware` 的正确叙事位置是：

- 它们不是另一个主方法。
- 它们是在三因子 backbone 已经成立以后，对 spatial / existence 两个分支进行轻量修正。
- 其中结构先验只是 mild prior，不是主导权重来源。

正文要刻意避免让读者产生下面这种理解：

“这篇论文的关键创新是 topology-aware fusion。”

因为当前证据并不支持这个说法。当前证据支持的是：

“topology-aware correction 有帮助，但只能弱用，且必须建立在三因子 adaptive backbone 之上。”

## 实验叙事应该怎么展开

结果部分的顺序，本质上就是主论证顺序。建议固定为下面 4 层。

### 第一层：headline 主场景

主场景必须是：

- dual-formation eight-sensor scenario
- tiered heterogeneous packet loss
- `fixed weights` 对比 `full adaptive`

这部分承担的任务不是“展示一堆数字”，而是先把主 claim 立住：

**在最符合本文问题设定的场景里，adaptive weighting 明显改善 consensus quality。**

### 第二层：因子消融

然后立即进入：

`fixed -> +covariance -> +link quality -> +existence confidence -> +weak structure-aware decoupled KLA`

这一层最重要，因为它回答的是“为什么方法有效”，而不只是“方法有效”。读者应该从这里得到 4 个结论：

1. `covariance` 必要，但不充分。
2. `link quality` 是异构通信下最关键的 communication-aware 增益来源。
3. `existence confidence` 是最有效的第三因子，尤其帮助 cardinality-related consensus。
4. `weak structure-aware decoupling` 是最终 refinement，而不是 backbone。

### 第三层：ideal communication supporting evidence

这一层的作用不是再证明一遍主结果，而是回答潜在质疑：

“你这个方法是不是只是在补救 packet loss？”

只要 ideal communication 下仍然有正向收益，就能把结论抬升为：

- 这不只是一个链路受损补丁。
- 这是一条更合理的 distributed fusion weighting rule。

### 第四层：generalization 和 appendix lines

最后再讲：

- communication-robustness trend
- AA generalization
- NIS / history / freshness / cardinality-consensus 等次线

这些内容的职责是补边界、补探索深度、证明你们试过别的方向，而不是分散主线。

## 正文各部分的叙事职责

建议把每一节的职责固定下来，写作时不要越界。

### Introduction

职责：

- 建立应用背景和 distributed fusion 的现实难点。
- 引出 KLA/GA-LMB 是合理基线。
- 明确 fixed weights 在 heterogeneous communication 下不够用。
- 引出三因子 adaptive weighting 的必要性。
- 给出 contribution list。

不要在这里做的事：

- 过早展开太多实现细节。
- 提前塞入 NIS、history、freshness 等次线模块。

### Problem Formulation

职责：

- 形式化 distributed LMB fusion with communication constraints。
- 把问题定义成 adaptive weight allocation，而不是新滤波器设计。
- 解释为什么要同时关心 posterior quality 和 realized link quality。
- 说明 evaluation target 是 consensus + local safeguard 的双层结构。

### Method

职责：

- 从 shared backbone 推出三因子设计。
- 强调 existence confidence 的必要性。
- 将 decoupled/structure-aware 处理成 mild refinement。
- 把 smoothing/min-weight safeguard 解释成 pragmatic stabilization，而不是 headline novelty。

### Results

职责：

- 用主场景先立主 claim。
- 用消融解释 gain 的来源。
- 用 ideal communication 说明方法不是单纯补链路损伤。
- 用 AA 和其他次线说明边界与泛化，但不抢主舞台。

### Conclusion

职责：

- 收束到“什么结论是当前证据真正支持的”。
- 明确哪些模块是主线，哪些只是 appendix evidence。
- 给出未来工作边界，但不要重新发明一个比正文更大的 claim。

## 证据层级必须怎么排

当前最稳的 evidence hierarchy 应该固定为：

### 第一层：必须重点写

- tiered heterogeneous packet-loss 下的 GA 主场景
- 因子消融：`fixed -> cov -> link -> existence -> weak structure-aware decoupled KLA`

### 第二层：明确是 supporting evidence

- ideal communication comparison
- communication-level robustness trend
- AA extension

### 第三层：明确是 appendix or negative evidence

- robust NIS
- history
- freshness
- association ambiguity
- posterior-structure-consistency
- cardinality-consensus add-on

这层级不能混。否则正文会失去聚焦，读者也会搞不清什么是 claim、什么是 exploratory attempt。

## 需要刻意避免的叙事误区

下面这些说法当前都不应该成为正文主张。

### 1. 不要把论文写成“新滤波器家族”

更准确的表述是：

- adaptive fusion-weight allocation for distributed GA-LMB / KLA fusion

而不是：

- a new distributed multi-object filter

### 2. 不要把 topology 当成主贡献

更准确的表述是：

- weak structure-aware prior refinement on top of a three-factor adaptive backbone

而不是：

- topology-driven fusion redesign

### 3. 不要 claim 普遍 local metric 全面提升

更准确的表述是：

- consensus gains are primary; local metrics serve as safeguards and show no obvious collapse

### 4. 不要让 NIS/history/freshness 抢主线

更准确的表述是：

- these directions were explored, but their evidence is weaker, more coupled, or mixed

## 推荐的论文总述模板

如果要用最简洁的方式向外解释这篇论文，可以按下面这套逻辑说：

1. 我们研究的是 communication-constrained distributed KLA-based LMB fusion。
2. 在这种问题里，固定权重不能反映节点 posterior quality 和实际通信可靠性的差异。
3. 因此我们把问题重写为 adaptive fusion-weight allocation。
4. 最有效的 backbone 是 `covariance + realized link quality + existence confidence`。
5. 在这个 backbone 上，`weak structure-aware decoupled KLA` 提供了小但稳定的最终增益。
6. 在 tiered heterogeneous packet-loss 的八传感器主场景中，这个设计显著改善了 network-level consensus。
7. ideal communication 和 AA 结果提供 supporting evidence，而 NIS/history/freshness 等被保留为附录次线。

## 给写作时的最终检查清单

如果一段话要进入正文主线，最好至少满足下面一条：

- 它在解释 fixed weights 为什么不够。
- 它在解释为什么需要 `covariance + link quality + existence confidence`。
- 它在解释为什么 `weak structure-aware decoupling` 只能是 refinement。
- 它在支持 `consensus-oriented` 的评价逻辑。
- 它在增强主场景或主消融的证据强度。

如果一段话不满足这些条件，它大概率应该：

- 被压缩，
- 被移到 appendix，
- 或者只保留一句交代。

## 当前最稳的结论表达

截至当前证据，论文最稳的 closing message 应该是：

**对于异构通信约束下的 distributed GA-LMB / KLA fusion，真正有效的不是更强的 topology 主导，而是一个以 `covariance + realized link quality + existence confidence` 为 backbone 的 communication-aware adaptive weighting 设计；在此基础上，弱结构感知的 branch-decoupled refinement 可以进一步改善 consensus OSPA 和 consensus RMSE，并保持良好的 cardinality agreement。**
