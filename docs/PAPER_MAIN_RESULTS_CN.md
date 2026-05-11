# 论文主要实验结果整理

本文当前工作的主线目标不是单纯提升单个节点的跟踪精度，而是在异构通信条件下提升分布式融合结果的跨节点一致性。因此，实验评价采用两层逻辑：

- `primary`：一致性指标，即 `consensus OSPA`、`consensus RMSE`、`consensus cardinality disagreement`
- `secondary`：常规 truth-referenced 跟踪指标，即 `local E-OSPA`、`local H-OSPA`、`local RMSE`、`local cardinality error`

整体判断标准是：在不明显牺牲常规跟踪精度的前提下，尽量提升融合一致性。

## 1. 主结果：tiered heterogeneous packet-loss 下的 8 传感器 GA-LMB 主场景

主场景是双四机编队的 `8-sensor distributed GA-LMB` 跟踪问题，通信设置为分层异构丢包。当前论文的 headline 方法是：

`covariance + link quality + existence confidence + weak structure-aware decoupled KLA`

对应结果如下。最新补充的外部 baseline 是 `Cao-Zhao FID-FIA baseline`，来源于 20-trial 主实验：

- [GA_TIERED_LINK_ABLATION_N20_SEED1_20260511_163852.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_ABLATION_N20_SEED1_20260511_163852.md)

### 1.1 一致性指标

`fixed weights -> Cao-Zhao FID-FIA baseline -> final adaptive`

| Arm | Consensus OSPA | Consensus RMSE | Consensus Cardinality |
| --- | ---: | ---: | ---: |
| fixed weights | 2.453677 | 2.335508 | 0.714625 |
| Cao-Zhao FID-FIA baseline | 1.820229 | 1.647412 | 0.126188 |
| +structure-aware decoupled KLA | 1.785873 | 1.562521 | 0.192938 |

相对固定权重基线：

- `Cao-Zhao FID-FIA baseline`：`OSPA` 下降 `25.82%`，`RMSE` 下降 `29.46%`，`cardinality disagreement` 下降 `82.34%`
- `+structure-aware decoupled KLA`：`OSPA` 下降 `27.22%`，`RMSE` 下降 `33.10%`，`cardinality disagreement` 下降 `73.00%`

这说明在异构链路条件下，自适应权重不仅改善了位置层面的一致性，也显著改善了跨节点的目标数一致性。FID-FIA 在 `Cardinality` 一致性上最好，而当前主方法在 `OSPA/RMSE` 一致性上最好，二者体现出清晰的 cardinality-vs-spatial tradeoff。

### 1.2 常规 local tracking 指标

同一主场景下，`fixed weights -> Cao-Zhao FID-FIA baseline -> final adaptive` 的常规 local 指标为：

| Arm | Local E-OSPA | Local H-OSPA | Local RMSE | Local CardErr |
| --- | ---: | ---: | ---: | ---: |
| fixed weights | 2.853096 | 0.500000 | 1.637556 | 1.454500 |
| Cao-Zhao FID-FIA baseline | 2.183127 | 0.500000 | 1.715746 | 0.392313 |
| +structure-aware decoupled KLA | 2.328672 | 0.500000 | 1.598561 | 0.578688 |

这回答了 FID-FIA 的 `Cardinality` 一致性问题：它不只是让各节点“更一致”，truth-referenced `local CardErr` 也最低，说明它确实更偏向正确的目标数估计。代价是它的 local `RMSE` 高于当前主方法；当前主方法仍然是空间精度和综合一致性最好的方案。

### 1.3 当前 paper message

这一组结果是全文最强的证据链。它支持的结论是：

- 固定权重在异构通信下过于僵硬
- 自适应权重必须至少同时考虑 `posterior concentration` 和 `realized communication quality`
- 仅有 `covariance + link quality` 还不够，`existence confidence` 提供了第三个关键判别维度
- 最终最优点不是强拓扑主导，而是建立在三因子 backbone 之上的 `weak structure-aware decoupled refinement`
- 外部 `FID-FIA` baseline 对 cardinality 非常强，但它会牺牲部分 spatial RMSE；因此论文主方法的定位应强调综合 OSPA/RMSE 优势，同时承认 FID-FIA 是一个强 cardinality baseline

主要来源：

- [GA_TIERED_LINK_ABLATION_N20_SEED1_20260511_163852.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_ABLATION_N20_SEED1_20260511_163852.md)
- [GA_TIERED_LINK_ABLATION_20260410_143517.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_ABLATION_20260410_143517.md)
- [06_results.tex](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/els-cas-templates/sections/06_results.tex)

## 2. 关键消融：从 fixed 到最终方法的逐步提升

主线消融顺序为：

`fixed -> +covariance -> +link quality -> +existence confidence -> +structure-aware decoupled KLA`

对应一致性指标如下：

| Arm | OSPA | RMSE | Cardinality |
| --- | ---: | ---: | ---: |
| fixed weights | 2.624065 | 2.702602 | 0.878750 |
| +covariance | 2.211513 | 2.410976 | 0.589500 |
| +link quality | 1.877771 | 1.800945 | 0.245250 |
| +existence confidence | 1.874840 | 1.779820 | 0.244500 |
| +structure-aware decoupled KLA | 1.862244 | 1.749608 | 0.244250 |

从这张消融表可以提炼出四点。

第一，`covariance weighting` 是必要的，但不是充分的。它说明后验集中程度确实能反映局部后验质量，但单独使用时，和最终方法之间仍有很大差距。

第二，`realized link quality` 是主场景中最关键的通信感知因子。加入它之后，三个一致性指标都出现最大的一次跳变，尤其 `cardinality disagreement` 从 `0.589500` 直接下降到 `0.245250`。

第三，`existence confidence` 是最有效的第三因子。它弥补了 `covariance + link quality` 不能表达“一个节点是否真正对目标存在性有把握”的缺陷。

第四，最终最优来自弱结构感知修正，而不是强拓扑驱动重构。`+structure-aware decoupled KLA` 相对 `+existence confidence` 的增益虽然不大，但在三项一致性指标上是稳定同向的，因此更适合作为最后一层 refinement。

主要来源：

- [06_results.tex](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/els-cas-templates/sections/06_results.tex)
- [GA_TIERED_LINK_ABLATION_20260326_182435.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_ABLATION_20260326_182435.md)

## 3. supporting evidence：理想通信条件下是否仍然有效

一个自然问题是：当前方法是不是只是在“补救丢包”？

理想通信实验的作用就是回答这个问题。该实验将通信退化去掉，在其余设置尽量保持一致的条件下，对比：

- `ordinary distributed GA`
- `structure-aware decoupled KLA`

### 3.1 一致性指标

- `consensus OSPA`: `1.706 -> 1.494`
- `consensus RMSE`: `1.526 -> 1.290`
- `consensus cardinality disagreement`: `0.161 -> 0.139`

### 3.2 常规 local 指标

- `local E-OSPA`: `1.950 -> 1.877`
- `local H-OSPA`: `0.500 -> 0.500`
- `local RMSE`: `1.442 -> 1.369`

这组结果说明两件事：

- 当前 refinement 并不只是因为链路差而起作用
- 即使在理想通信条件下，它依然能在一致性和常规局部指标上给出正向收益

因此，这个实验在论文里最适合作为 `supporting evidence`，用于增强方法机制的可信度，而不是取代主场景。

主要来源：

- [GA_IDEAL_COMM_COMPARE_20260326_184508.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_IDEAL_COMM_COMPARE_20260326_184508.md)
- [IDEAL_COMM_COMPARE_CN.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/IDEAL_COMM_COMPARE_CN.md)

## 4. supporting evidence：AA generalization

当前方法主线是围绕 `GA-LMB / KLA` 组织的，但它并不完全局限于这一种融合形式。现有的 `AA` 扩展实验表明，这套权重思想在另一条融合线里也有正向效果。

`AA` 实验中，adaptive 相对 baseline 的结果为：

- `consensus OSPA`: `4.349 -> 3.811`
- `consensus RMSE`: `19.098 -> 16.472`
- `consensus cardinality disagreement`: `0.421 -> 0.307`

不过，这组结果目前只是 supporting evidence，不宜和主场景并列成 headline，原因有两个：

- 数值层面虽然改善明显，但实验线不是本文主要叙事
- 论文当前最完整、最稳的证据链仍然是 `GA-LMB + tiered heterogeneous packet loss`

## 5. 次线和附录结果：做过，但不进正文主线

为了避免把正文叙事拉散，当前 paper 已明确把以下模块降级为次线或附录材料：

- `NIS`
- `history`
- `freshness`
- `cardinality-consensus`
- `association ambiguity`
- `posterior-structure-consistency`

原因不是这些方向完全无意义，而是目前证据不够稳定，或者与主线的 `covariance + link quality + existence confidence + weak structure-aware decoupling` 相比，边际收益太小、耦合太强，或者结果混合。

其中最典型的是 `NIS`：

- `w/o NIS -> robust NIS -> plain NIS`
- `OSPA: 1.909 -> 1.909 -> 2.008`
- `RMSE: 2.934 -> 2.980 -> 3.173`
- `Cardinality: 0.267 -> 0.262 -> 0.300`

这说明：

- `plain NIS` 明显有害
- `robust NIS` 虽然比 plain NIS 稳定，但也没有形成足够强的主线证据

因此，当前最合适的 paper positioning 是：

- 正文只保留主线四项
- 次线模块保留到附录，用来证明“这些方向我们试过，但它们不是当前版本最有效、最稳的答案”

## 6. 当前最稳的论文结论

基于现有实验结果，当前最稳的论文结论可以整理为下面三句话。

第一，在异构通信条件下，分布式融合的关键目标不应只看单节点跟踪精度，还应看网络层面的融合一致性；因此本文将 `consensus` 指标作为 `primary outcome`，将常规 local tracking 指标作为 `secondary safeguard`。

第二，最有效的 adaptive fusion-weight design 是：

`covariance + realized link quality + existence confidence + weak structure-aware decoupled KLA`

其中真正构成 backbone 的是前三项，最后一项是弱修正而不是主导权重来源。

第三，在当前最强证据链上，这个设计不仅显著改善了 `consensus OSPA / consensus RMSE / consensus cardinality disagreement`，而且没有带来常规 local tracking accuracy 的明显退化，因此已经形成一条比较完整、可 defend 的论文主线。
