# 论文主要实验结果整理

本文当前工作的主线目标不是单纯提升单个节点的跟踪精度，而是在异构通信条件下提升分布式融合结果的跨节点一致性，并评估这种一致性收益是否具备部署可行性。因此，实验评价采用三层逻辑：

- `primary`：一致性指标，即 `consensus OSPA`、`consensus position disagreement`、`consensus cardinality disagreement`
- `secondary`：常规 truth-referenced 跟踪指标，即 `local E-OSPA`、`local RMSE`、`local cardinality error`
- `efficiency`：计算量指标，即每个 arm 的过滤/融合 wall-clock runtime、per-step runtime、以及相对固定权重的 runtime ratio

整体判断标准是：在不明显牺牲常规跟踪精度、且计算量可接受的前提下，尽量提升融合一致性。

## 1. 主结果：tiered heterogeneous packet-loss 下的 8 传感器 GA-LMB 主场景

主场景是双四机编队的 `8-sensor distributed GA-LMB` 跟踪问题，通信设置为分层异构丢包。当前论文的 headline 方法更新为：

`covariance + link quality + existence confidence + weak structure-aware decoupled KLA + FID-FIA existence refinement`

其中 `FID-FIA existence refinement` 只进入 existence/cardinality 分支，spatial 分支仍保持当前 structure-aware decoupled KLA 的设计。最新 20-trial 主实验来源：

- [GA_TIERED_LINK_ABLATION_N20_SEED1_20260512_155714.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_ABLATION_N20_SEED1_20260512_155714.md)
- [GA_TIERED_LINK_ABLATION_N20_SEED1_20260511_163852.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_ABLATION_N20_SEED1_20260511_163852.md)
- 计算量补充实验：[GA_TIERED_LINK_ABLATION_N3_SEED1_20260515_105137.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_ABLATION_N3_SEED1_20260515_105137.md)

### 1.1 一致性指标

`Fixed Metropolis -> FID-FIA baseline -> Balanced mode -> Cardinality-critical mode`

| Arm | Consensus OSPA | Consensus Position Disagreement | Consensus Cardinality Disagreement |
| --- | ---: | ---: | ---: |
| Fixed Metropolis | 2.453677 | 2.335508 | 0.714625 |
| FID-FIA baseline | 1.820229 | 1.647412 | 0.126188 |
| Balanced mode | 1.785873 | 1.562521 | 0.192938 |
| Cardinality-critical mode | 1.668961 | 1.528182 | 0.061062 |

相对固定权重基线：

- `FID-FIA baseline`：`OSPA` 下降 `25.82%`，`position disagreement` 下降 `29.46%`，`cardinality disagreement` 下降 `82.34%`
- `Balanced mode`：`OSPA` 下降 `27.22%`，`position disagreement` 下降 `33.10%`，`cardinality disagreement` 下降 `73.00%`
- `Cardinality-critical mode`：`OSPA` 下降 `31.98%`，`position disagreement` 下降 `34.57%`，`cardinality disagreement` 下降 `91.46%`

这说明 FID-FIA 的信息几何信号确实适合修正 existence/cardinality 分支。纯 `FID-FIA baseline` 已经是很强的 cardinality baseline，但把 FID-FIA 只注入 existence 分支后，新的 `Cardinality-critical mode` 同时超过了 FID-FIA baseline 和 Balanced mode：它在三个 primary consensus 指标上都是当前最优。

### 1.2 常规 local tracking 指标

同一主场景下，常规 local 指标为：

| Arm | Local E-OSPA | Local RMSE | Local CardErr |
| --- | ---: | ---: | ---: |
| Fixed Metropolis | 2.853096 | 1.637556 | 1.454500 |
| FID-FIA baseline | 2.183127 | 1.715746 | 0.392313 |
| Balanced mode | 2.328672 | 1.598561 | 0.578688 |
| Cardinality-critical mode | 2.009084 | 1.704538 | 0.221563 |

这进一步确认 `Cardinality-critical mode` 不是把节点推向错误的一致目标数：它的 truth-referenced `local CardErr` 从 FID-FIA baseline 的 `0.392313` 进一步降到 `0.221563`，同时 `local E-OSPA` 也最好。代价是它的 local `RMSE` 不如 Balanced mode，但仍略优于 FID-FIA baseline，并满足 plan 里的 safeguard。

### 1.3 计算量补充实验

同一主场景下补充了 `3-trial` 计算量统计。计时口径只包括每个 arm 的 `runDistributedLmbFilter` 过滤/融合调用，不包括场景生成、通信模型采样和指标评估。

| Arm | Filter runtime (s) | Runtime/step (s) | Relative to fixed |
| --- | ---: | ---: | ---: |
| Fixed Metropolis | 47.334 +/- 0.732 | 0.473 | 1.000x |
| FID-FIA baseline | 137.830 +/- 3.479 | 1.378 | 2.913x |
| Balanced mode | 55.603 +/- 5.057 | 0.556 | 1.174x |
| Cardinality-critical mode | 143.107 +/- 2.950 | 1.431 | 3.023x |

这组计算量结果改变了选型表述：`Balanced mode` 只比固定权重多约 `17.4%` 的过滤/融合时间，却保留了主要的 spatial/position 共识增益；而 `FID-FIA baseline` 和 `Cardinality-critical mode` 都约为固定权重的 `3x`，说明 FID-FIA 相关的信息几何计算是主要额外成本。若算力或实时性是硬约束，优先推荐 `Balanced mode`；若目标数一致性、漏检/虚警支持的代价更高，再选择 `Cardinality-critical mode`。

### 1.4 当前 paper message

这一组结果是全文最强的证据链。它支持的结论是：

- 固定权重在异构通信下过于僵硬
- 自适应权重必须至少同时考虑 `posterior concentration` 和 `realized communication quality`
- 仅有 `covariance + link quality` 还不够，`existence confidence` 提供了第三个关键判别维度
- 旧的最优点是建立在三因子 backbone 之上的 `weak structure-aware decoupled refinement`
- 新的最优点是在保持该 spatial 分支的同时，把 FID-FIA 信息几何信号只注入 existence/cardinality 分支
- 外部 `FID-FIA baseline` 仍然是一个强 cardinality baseline，但新的 `Cardinality-critical mode` 已经在 `consensus OSPA / consensus position disagreement / consensus cardinality disagreement` 和 `local CardErr` 上同时超过它
- 计算量上，`Balanced mode` 是低额外开销的默认推荐；`Cardinality-critical mode` 是以约 `3x` fixed-runtime 换取最强 cardinality/consensus 表现的高精度选项

主要来源：

- [GA_TIERED_LINK_ABLATION_N20_SEED1_20260512_155714.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_ABLATION_N20_SEED1_20260512_155714.md)
- [GA_TIERED_LINK_ABLATION_N20_SEED1_20260511_163852.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_ABLATION_N20_SEED1_20260511_163852.md)
- [GA_TIERED_LINK_ABLATION_20260410_143517.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_ABLATION_20260410_143517.md)
- [06_results.tex](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/els-cas-templates/sections/06_results.tex)

## 2. 关键消融：从 fixed 到最终方法的逐步提升

主线消融顺序为：

`Fixed Metropolis -> Covariance-only adaptive -> Covariance-link adaptive -> Three-factor adaptive backbone -> Balanced mode`

对应一致性指标如下：

| Arm | Consensus OSPA | Consensus Position Disagreement | Consensus Cardinality Disagreement |
| --- | ---: | ---: | ---: |
| Fixed Metropolis | 2.624065 | 2.702602 | 0.878750 |
| Covariance-only adaptive | 2.211513 | 2.410976 | 0.589500 |
| Covariance-link adaptive | 1.877771 | 1.800945 | 0.245250 |
| Three-factor adaptive backbone | 1.874840 | 1.779820 | 0.244500 |
| Balanced mode | 1.862244 | 1.749608 | 0.244250 |

从这张消融表可以提炼出四点。

第一，`covariance weighting` 是必要的，但不是充分的。它说明后验集中程度确实能反映局部后验质量，但单独使用时，和最终方法之间仍有很大差距。

第二，`realized link quality` 是主场景中最关键的通信感知因子。加入它之后，三个一致性指标都出现最大的一次跳变，尤其 `cardinality disagreement` 从 `0.589500` 直接下降到 `0.245250`。

第三，`existence confidence` 是最有效的第三因子。它弥补了 `covariance + link quality` 不能表达“一个节点是否真正对目标存在性有把握”的缺陷。

第四，最终最优来自弱结构感知修正，而不是强拓扑驱动重构。`Balanced mode` 相对 `Three-factor adaptive backbone` 的增益虽然不大，但在三项一致性指标上是稳定同向的，因此更适合作为最后一层 refinement。

主要来源：

- [06_results.tex](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/paper/els-cas-templates/sections/06_results.tex)
- [GA_TIERED_LINK_ABLATION_20260326_182435.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_TIERED_LINK_ABLATION_20260326_182435.md)

## 3. supporting evidence：理想通信条件下是否仍然有效

一个自然问题是：当前方法是不是只是在“补救丢包”？

理想通信实验的作用就是回答这个问题。该实验将通信退化去掉，在其余设置尽量保持一致的条件下，对比：

- `Ordinary GA`
- `Balanced mode`
- `FID-FIA baseline`
- `Cardinality-critical mode`

### 3.1 一致性指标

- `consensus OSPA`: `1.706 -> 1.494`
- `consensus position disagreement`: `1.526 -> 1.290`
- `consensus cardinality disagreement`: `0.161 -> 0.139`

### 3.2 常规 local 指标

- `local E-OSPA`: `1.950 -> 1.877`
- `local RMSE`: `1.442 -> 1.369`

这组结果说明两件事：

- 当前 refinement 并不只是因为链路差而起作用
- 即使在理想通信条件下，它依然能在一致性和常规局部指标上给出正向收益

因此，这个实验在论文里最适合作为 `supporting evidence`，用于增强方法机制的可信度，而不是取代主场景。

主要来源：

- [GA_IDEAL_COMM_COMPARE_20260326_184508.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/RUN/GA/GA_IDEAL_COMM_COMPARE_20260326_184508.md)
- [IDEAL_COMM_COMPARE_CN.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/IDEAL_COMM_COMPARE_CN.md)

## 4. 附录结果：AA secondary route

当前正文主线明确选择 `GA-LMB / KLA`，因为 GA/KLA 更适合讲未知相关性下的保守融合、log-opinion-pool、以及 spatial/existence 分支解耦。因此 `AA` 不再进入正文主结果，只作为附录里的 secondary route 记录。

`AA` 实验中，adaptive 相对 baseline 的结果为：

- `consensus OSPA`: `4.349 -> 3.811`
- `consensus position disagreement`: `19.098 -> 16.472`
- `consensus cardinality disagreement`: `0.421 -> 0.307`

不过，这组结果目前只放附录，不和主场景并列，原因有两个：

- 数值层面虽然改善明显，但实验线不是本文主要叙事
- 论文当前最完整、最稳的证据链和理论推导都围绕 `GA-LMB + tiered heterogeneous packet loss`

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

`covariance + realized link quality + existence confidence + weak structure-aware decoupled KLA + FID-FIA existence refinement`

其中前三项构成 communication-aware backbone，`weak structure-aware decoupled KLA` 主要稳住 spatial/position 分支，而 `FID-FIA existence refinement` 只用于加强 existence/cardinality 分支。

第三，在当前最强证据链上，这个设计不仅显著改善了 `consensus OSPA / consensus position disagreement / consensus cardinality disagreement`，而且在 `local CardErr` 和 `local E-OSPA` 上也超过 FID-FIA baseline；虽然 local RMSE 不如 Balanced mode，但仍优于 FID-FIA baseline，因此已经形成一条更强、可 defend 的论文主线。

补充计算量统计后，选型建议应明确加入运行成本：`Balanced mode` 是算力/实时性更敏感时的默认配置；`Cardinality-critical mode` 是 cardinality 风险更高且可接受约 `3x` 过滤/融合耗时时的配置。
