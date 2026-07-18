# ICASSP Receiver-Induced Moment Exchange 实验进度与结果总表

更新日期：2026-07-18

状态：**主要的 50 组实验已经完成并锁定；现有结果已通过自动复核，不再重新运行编号 82--131 的实验**

对应论文：`Receiver-Induced Moment Exchange for Distributed LMB Fusion`

> 一句话结论：我们把接收端原本必做的信息归纳提前到了发送端。在已经完成并锁定的 50 组八传感器对照实验中，系统实际编码并准备发送的数据量平均减少 **58.28%**，而超过 111 万条保留目标记录的存在概率、状态估计和不确定性描述完全一致。

## 0. 面向非专业听众的汇报主线

> 阅读建议：本节适合组会、项目汇报和跨专业沟通，不要求听众理解论文中的算法缩写。第 1 节起保留技术名称、精确统计口径和复核入口，供进一步追问时使用。

### 0.1 这项工作要解决什么问题

在多传感器协同跟踪中，每个节点既要自己判断目标在哪里，也要把判断结果发给其他节点。为了表达一个目标可能出现的多种状态，节点内部通常会保存一份比较详细的描述。当目标数量增多、每个目标的候选状态变多时，节点之间需要发送的数据也会随之增加。

我们检查现有系统后发现，接收节点拿到这份详细描述时，第一步就会按固定规则把它归纳成几个核心信息：目标存在的可能性、目标状态的估计，以及这个估计可能偏差多大。后续处理实际只使用这些归纳结果，更细的候选状态不会继续参与计算。

可以把原来的过程理解为：**先把十页材料寄给对方，对方收到后再按固定模板整理成一页摘要。** 我们要回答的问题是，如果双方使用完全相同的整理规则，能否在寄出前就完成整理，只发送对方真正会使用的那一页？

这里需要强调，详细信息并非在任何场景下都无用。它只是对**当前这类接收方式**没有进一步影响；如果换一种接收方式，结论可能不同。

### 0.2 我们的方法是怎么想到的

我们的出发点不是发明新的跟踪或融合算法，而是重新设计节点之间传递信息的方式。具体做法很简单：

1. 先从接收端往回看，确认它后续计算真正会使用哪些信息。
2. 把接收端原本必做的归纳步骤提前到发送端。
3. 发送端只传递归纳后的内容，接收端的其余处理保持不变。

这项改动看似只是“少发一些内容”，但真正困难的是确认提前归纳后不会悄悄改变计算结果。例如，双方必须用完全相同的规则处理异常数据和计算误差；信息也必须经过真实的编码、送达和解码流程，而不能只在内存中比较两个理想化对象。

因此，方法设计遵循以下原则：

| 设计原则 | 通俗解释 | 为什么重要 |
|---|---|---|
| 从接收端的实际需求出发 | 先确认对方真正使用什么，再决定发送什么 | 避免把发送端的内部细节原样搬上网络 |
| 只提前一个既有步骤 | 只移动原本就会发生的信息归纳，不改变跟踪和融合算法 | 便于判断结果差异是否来自消息内容 |
| 两端采用同一套规则 | 发送端和接收端使用一致的数据整理和计算误差处理方法 | 避免“理论上相同、程序中却不同” |
| 按真实流程传递 | 两种消息都实际编码；成功送达后再实际解码 | 得到的是程序真正生成的消息大小，而不是粗略估算 |
| 采用严格配对比较 | 两组实验使用相同场景、传感器观测、连接关系、消息送达随机条件和处理设置 | 确保唯一变化是发送前是否先做归纳 |
| 分开回答两个问题 | 先检查结果是否不变，再统计数据量减少多少 | 不用通信收益掩盖跟踪结果的细小变化 |

### 0.3 我们如何确认结果可信

我们没有只看最终跟踪误差是否“差不多”，而是从四个层次逐步检查：

| 检查层次 | 做了什么 | 主要排除的疑问 |
|---|---|---|
| 原理检查 | 推导两种传递方式进入后续计算前所保留的信息是否一致 | 提前归纳是否在原理上改变当前接收端的输入 |
| 传递流程测试 | 检查实际编码、解码、计算误差处理以及异常输入 | 纸面上的一致性是否会被程序细节破坏 |
| 严格配对实验 | 50 组实验逐一共享场景和随机条件，只改变消息内容 | 数据量和跟踪结果的差异是否由其他设置造成 |
| 逐项核对 | 对保留下来的每条目标记录，分别比较存在概率、状态估计和不确定性 | 汇总指标相同是否只是四舍五入或误差抵消 |

我们还设置了一个“故意破坏条件”的检查：人为改变精简消息中的不确定性后，比较程序能够立刻发现差异。这说明核对流程确实对结果变化敏感，而不是无论输入如何都报告一致。

### 0.4 实验结果说明了什么

| 汇报问题 | 结果 | 面向非专业听众的解释 |
|---|---:|---|
| 实验规模 | 50 组配对实验；每组 8 个传感器、100 个时间步 | 不是单次演示，而是在相同对照条件下重复验证 |
| 准备发送的数据量 | 平均减少 **58.28%** | 在当前任务中，程序生成的消息缩小了一半以上 |
| 结果稳定性 | 50/50 组实验的数据量都减少 | 收益不是由少数特殊实验拉高 |
| 统计区间 | 95% 区间为 **57.92%--58.64%** | 在这批实验中，平均收益较为稳定 |
| 全流程检查 | 40,000 个传感器—时间点没有缺失或目标编号差异 | 两种传递方式沿完整跟踪流程保持同步 |
| 细粒度检查 | 1,119,037 条保留目标记录逐项完全一致 | 存在概率、状态估计和不确定性均未发现差异 |

这些结果支持一个范围明确的结论：**对于当前接收方式，我们可以在不改变已核验跟踪结果的前提下，显著减少程序实际编码并准备发送的数据。**

这项工作的主要价值也不只是“得到 58.28%”。更值得复用的是一种信息设计思路：网络消息不一定要照搬发送端保存的全部内部细节，可以先分析接收端真正会使用什么，再决定哪些内容需要跨节点传递。

### 0.5 汇报时必须主动说明的边界

- 这不是新的目标跟踪算法，也没有改变现有融合规则。
- “结果不变”只针对本文明确实现和测试的接收方式，不代表两种消息在一般意义上包含完全相同的信息。
- 58.28% 指程序实际编码并准备发送的消息内容，不包含无线协议的额外开销，因此不能直接换算成无线能耗、传输时延或网络总体速度。
- 当前实验覆盖一个由 8 个传感器组成的模拟场景、固定的传感器连接关系、统一数值格式和每个时间步一次融合；换场景后，节省比例需要重新测量。
- 如果后续处理还需要多个候选状态的完整细节，或采用不同计算规则、降低消息精度、重复利用同一条精简消息等方式，就必须重新验证。

### 0.6 一分钟口头汇报版本

在多传感器协同跟踪中，每个节点过去会把每个目标的完整、多候选描述都发给其他节点，但接收方拿到后，第一步就是按固定规则把它归纳成目标存在的可能性、状态估计，以及估计可能偏差多大。我们的改动就是把这一步提前到发送端，只发送接收方后续真正会使用的内容，而接收方的跟踪和融合算法保持不变。为了确认这不是纸面上的压缩，我们让两种方案经过同一套实际编码和解码流程，并做了 50 组严格配对实验。结果显示，程序准备发送的数据量平均减少 58.28%，同时超过 111 万条保留目标记录逐项完全一致。这个结论只适用于当前接收方式和实验设置，不能直接换算成无线能耗或时延收益。更普遍的启发是：通信格式不一定要照搬发送端的内部表示，而应从接收端真正使用的信息反向设计。

---

> 以下为技术复核区，面向参与论文撰写、实验维护和结果审计的读者。

## 1. 一屏总览

| 项目 | 冻结设置或结果 | 结论强度 | 备注 |
|---|---:|:---:|---|
| 研究问题 | sender-side projection 能否在保持 executed receiver 输出完全一致的同时减少应用层报文字节 | — | 只针对本文明确实现的 receiver contract |
| 对照臂 | Periodic full GM fusion message | — | 编码全部 GM components |
| 实验臂 | Periodic moment message on static topology | — | 每标签投影为单 Gaussian 后编码 |
| 唯一改变变量 | 是否在发送端编码前执行 canonical moment projection | High | 两臂的 topology、schedule、weights、labels、measurements、delivery uniforms 均配对 |
| 试验规模 | 50 paired trials，seeds 82--131 | High | 每 trial 100 filtering steps，8 sensors |
| 主通信结果 | attempted-byte reduction = **58.277264%** | High | 50 个 per-trial reduction 的算术平均 |
| 95% 区间 | **[57.923222%, 58.636095%]** | High | paired percentile bootstrap，10,000 resamples，seed 20270710 |
| 单 trial 范围 | **55.921689%--61.383973%** | High | 50/50 trials 均节省 attempted bytes |
| delivered-byte reduction | **58.267212%** | High | 50 个 per-trial delivered reductions 的平均 |
| 快照审计 | 40,000 retained post-step snapshots；0 missing；0 label-set mismatch | High | common `r>1e-2` retention 后，每臂每 trial 为 `8 × 100 = 800` snapshots |
| 字段审计 | 1,119,037 retained matched label instances；`max |delta r| = max |delta mu| = max |delta Sigma| = 0` | High | binary64 exact gate，不使用 tolerance；raw pre-retention equality 由命题给出 |
| mask 审计 | attempted masks 50/50 相同；delivered masks 50/50 相同 | High | 隔离 payload representation 之外的因果差异 |
| tracking/consensus | 两臂所有保存指标逐 trial 完全一致 | High | 字段级等价比 rounded metrics 更强 |
| 结论边界 | application-layer bytes；单图、同质传感器、binary64、每 step 一轮 fusion | — | 不外推到 airtime、energy、latency、multi-round 或 mixture-aware consumer |

## 2. 术语与统计口径

| 规范术语 | 本文含义 | 不应混用的说法 |
|---|---|---|
| `Full GM message` | 保留每个 active Bernoulli 的全部 Gaussian components 并编码 | `full posterior`（会误含 trajectory 等本地字段） |
| `Moment message` | 发送 `(label, r, mean, covariance)` 的单 Gaussian 表示 | `lossless compression`、一般 GM-KLA 等价 |
| Canonical moment projection `P` | 权重 sanitation + component covariance symmetrization + 每标签一、二阶矩匹配 | 近似一般 mixture-aware fusion 的充分统计量 |
| Canonical transport `T` | versioned encode/decode 的 posterior-content map；协方差按 `C(A)=(A+A^T)/2` 规范化 | 任意 raw object 的 bitwise identity map |
| `Attempted bytes` | delivery draw 之前已编码并尝试发送的 application-layer bytes | delivered bytes、airtime、IP/PHY traffic |
| `Delivered bytes` | delivery 成功且 receiver 实际解码的 application-layer bytes | attempted bytes |
| Per-trial reduction `rho_s` | `100 × (B_full,s - B_moment,s) / B_full,s` | ratio of aggregate totals |
| Retained post-step snapshot | common `r>1e-2` retention 后，某个 sensor、某个 filtering step 的 sorted labels、`r`、`mu`、`Sigma` | raw pre-retention fusion output、完整内部 tracker state |
| Exact match | label sets 相同且 `r`、`mu`、`Sigma` 的最大残差都等于 0 | 仅 rounded tracking metric 相同 |
| E-OSPA | Euclidean OSPA，`c=5, p=2` | 未注明 cutoff/order 的泛称 OSPA |
| 历史 artifact 名称 | 文件名保留 `FUSION_SUFFICIENT_MOMENT_EXCHANGE` 以绑定已发布证据 | 不代表 Fisher--Neyman sufficiency 或一般 density equivalence |

## 3. 实验设计与因果隔离

| 维度 | Full GM arm | Moment arm | 是否严格配对 | 审计方式 |
|---|---|---|:---:|---|
| Local LMB prediction/update | 相同 | 相同 | Yes | 同一 generic runner |
| Sensor count | 8 | 8 | Yes | frozen config |
| Simulation length | 100 | 100 | Yes | frozen config |
| Graph | 固定 4+4 topology，16 undirected edges | 同左 | Yes | arm config comparison |
| Fusion schedule | 每 filtering step 一轮 synchronous neighbor fusion | 同左 | Yes | trigger config comparison |
| Base fusion weights | 固定对称 degree-based：self `1/3`、四邻居各 `1/6` | 同左 | Yes | 配置字段历史名为 `metropolis`，但不得称为标准 Metropolis weights |
| Gaussian/Bernoulli weights | 同一组 presence-normalized weights | 同左 | Yes | executed receiver code path |
| Detection probability | 0.9 | 0.9 | Yes | frozen config |
| Mean clutter count | 3 | 3 | Yes | frozen config |
| Measurement-noise standard deviation | 3 | 3 | Yes | frozen config |
| Node drop probabilities | `[0, 0.1, 0.2, 0.5]`，multiplicities `[1,4,1,2]` | 同左 | Yes | 每 trial 随机分配，mean 0.2 |
| Delivery uniforms | 预生成 | 使用同一组预生成值 | Yes | attempted/delivered mask equality |
| Message schedule | Periodic / always heavy | Periodic / always light | Yes | event type 只区分 payload representation |
| Dynamic topology / link gate | Off | Off | Yes | frozen config |
| Stale cache / heartbeat / mixed payload | Off | Off | Yes | frozen config |
| Covariance inflation | Off | Off | Yes | 等价命题的必要条件；另设负控制 |
| Mode-aware weights | Off | Off | Yes | frozen config |
| 编码前 projection | No | Yes | **唯一差异** | arm contract |
| Wire codec | version 1, little-endian, binary64 | 同左 | Yes | codec round-trip tests |
| Byte oracle | encoded `uint8` array length | 同左 | Yes | 不使用标量数估算 |

## 4. 聚合结果大表

### 4.1 通信、跟踪与字段级等价

| 指标 | Full GM | Moment | 差异或节省 | 统计口径 / 解释 |
|---|---:|---:|---:|---|
| Mean attempted payload (`10^6 B/trial`) | 25.083704 | 10.457778 | -14.625926 × `10^6 B` | decimal，`10^6 B = 1 MB` |
| Median attempted payload (`10^6 B/trial`) | 24.975680 | 10.391720 | — | 50 trials |
| Attempted payload range (`10^6 B/trial`) | 21.366480--34.206160 | 8.616240--13.825360 | — | min--max |
| Total attempted payload (bytes) | 1,254,185,200 | 522,888,880 | -731,296,320 | 所有 trials 求和 |
| **Mean per-trial attempted reduction** | — | — | **58.277264%** | **论文主指标** |
| Attempted reduction 95% CI | — | — | **57.923222%--58.636095%** | paired percentile bootstrap |
| Attempted reduction median | — | — | 58.045087% | 50 per-trial ratios |
| Attempted reduction min--max | — | — | 55.921689%--61.383973% | 所有 trials 均为正 |
| Attempted ratio of aggregate totals | — | — | 58.308479% | 仅描述 totals；不得替代主指标 |
| Mean delivered payload (`10^6 B/trial`) | 20.090979 | 8.377969 | -11.713010 × `10^6 B` | 相同 delivery masks 下比较 |
| Median delivered payload (`10^6 B/trial`) | 19.882946 | 8.344306 | — | 50 trials |
| Delivered payload range (`10^6 B/trial`) | 17.215140--27.407228 | 7.017844--11.161628 | — | min--max |
| Total delivered payload (bytes) | 1,004,548,968 | 418,898,448 | -585,650,520 | 所有 trials 求和 |
| Mean per-trial delivered reduction | — | — | 58.267212% | 次级通信指标 |
| Delivered reduction min--max | — | — | 56.303901%--60.982302% | 50 per-trial ratios |
| Delivered ratio of aggregate totals | — | — | 58.299848% | 仅描述 totals |
| Mean local E-OSPA | 2.076120 | 2.076120 | 0 | `c=5, p=2`；lower is better |
| Mean consensus OSPA | 1.951280 | 1.951280 | 0 | mean pairwise sensor OSPA |
| Mean position disagreement | 4.613897 | 4.613897 | 0 | paired exact equality |
| Mean cardinality dispersion | 0.203375 | 0.203375 | 0 | paired exact equality |
| Sensor-time snapshots | 40,000 | 40,000 | 0 missing | `50 × 8 × 100` |
| Matched label instances | 1,119,037 | 1,119,037 | 0 missing labels | field-level comparator |
| Label-set mismatch count | 0 | 0 | 0 | hard gate |
| `max |delta r|` | — | — | **0** | binary64 exact |
| `max |delta mu|` | — | — | **0** | binary64 exact |
| `max |delta Sigma|` | — | — | **0** | binary64 exact |
| Attempted masks equal | — | — | 50/50 trials | hard gate |
| Delivered masks equal | — | — | 50/50 trials | hard gate |
| Exact receiver outputs | — | — | 50/50 trials | hard gate |

### 4.2 跨 trial 分布

| 指标 | Mean | Median | Min | Max | Sample SD |
|---|---:|---:|---:|---:|---:|
| Full attempted payload (MB) | 25.083704 | 24.975680 | 21.366480 | 34.206160 | 2.425183 |
| Moment attempted payload (MB) | 10.457778 | 10.391720 | 8.616240 | 13.825360 | 0.961911 |
| Attempted reduction (%) | 58.277264 | 58.045087 | 55.921689 | 61.383973 | 1.299334 |
| Full delivered payload (MB) | 20.090979 | 19.882946 | 17.215140 | 27.407228 | 1.977969 |
| Moment delivered payload (MB) | 8.377969 | 8.344306 | 7.017844 | 11.161628 | 0.780054 |
| Delivered reduction (%) | 58.267212 | 58.177234 | 56.303901 | 60.982302 | 1.219976 |
| Local E-OSPA | 2.076120 | 2.066412 | 1.893789 | 2.398509 | 0.098505 |
| Consensus OSPA | 1.951280 | 1.944550 | 1.799003 | 2.149897 | 0.079146 |
| Position disagreement | 4.613897 | 4.287878 | 2.233701 | 9.666191 | 1.479549 |
| Cardinality dispersion | 0.203375 | 0.206250 | 0.147500 | 0.253750 | 0.024743 |
| Label comparisons per trial | 22,380.74 | 22,152 | 18,579 | 28,782 | 1,886.39 |
| Snapshots per trial | 800 | 800 | 800 | 800 | 0 |

## 5. 逐 seed 通信与 exact audit 大表

以下 MB 均为 decimal MB；`Exact=Yes` 表示该 seed 的 label sets、`r`、`mu`、`Sigma` 全部通过严格零残差 gate。

| Seed | Full attempted (MB) | Moment attempted (MB) | Attempted reduction (%) | Full delivered (MB) | Moment delivered (MB) | Delivered reduction (%) | Compared labels | Exact |
|---:|---:|---:|---:|---:|---:|---:|---:|:---:|
| 82 | 24.076 | 10.210 | 57.592 | 19.640 | 8.127 | 58.618 | 21,858 | Yes |
| 83 | 23.528 | 9.649 | 58.990 | 19.213 | 7.760 | 59.611 | 20,888 | Yes |
| 84 | 25.804 | 10.632 | 58.796 | 20.703 | 8.622 | 58.355 | 22,889 | Yes |
| 85 | 23.735 | 10.153 | 57.224 | 18.800 | 8.054 | 57.161 | 21,730 | Yes |
| 86 | 23.197 | 9.317 | 59.833 | 18.577 | 7.525 | 59.494 | 20,449 | Yes |
| 87 | 27.443 | 10.908 | 60.253 | 22.206 | 8.888 | 59.972 | 23,581 | Yes |
| 88 | 24.983 | 10.880 | 56.451 | 19.889 | 8.610 | 56.710 | 23,044 | Yes |
| 89 | 25.890 | 11.046 | 57.336 | 20.786 | 8.653 | 58.369 | 23,664 | Yes |
| 90 | 21.366 | 8.616 | 59.674 | 17.295 | 7.018 | 59.422 | 18,579 | Yes |
| 91 | 25.883 | 11.077 | 57.204 | 20.864 | 8.911 | 57.290 | 23,268 | Yes |
| 92 | 24.463 | 9.945 | 59.349 | 19.530 | 7.909 | 59.504 | 21,489 | Yes |
| 93 | 23.381 | 9.927 | 57.541 | 18.596 | 7.765 | 58.242 | 21,006 | Yes |
| 94 | 27.381 | 11.699 | 57.273 | 22.167 | 9.400 | 57.593 | 24,753 | Yes |
| 95 | 28.692 | 11.904 | 58.510 | 23.151 | 9.477 | 59.063 | 25,291 | Yes |
| 96 | 27.037 | 10.762 | 60.197 | 21.569 | 8.550 | 60.360 | 23,174 | Yes |
| 97 | 24.421 | 10.256 | 58.002 | 19.575 | 8.069 | 58.778 | 21,894 | Yes |
| 98 | 27.474 | 11.601 | 57.774 | 21.739 | 9.274 | 57.340 | 24,484 | Yes |
| 99 | 23.423 | 9.332 | 60.160 | 18.646 | 7.495 | 59.800 | 20,038 | Yes |
| 100 | 24.969 | 10.349 | 58.553 | 19.990 | 8.367 | 58.147 | 22,509 | Yes |
| 101 | 26.463 | 11.664 | 55.922 | 21.207 | 9.242 | 56.422 | 24,832 | Yes |
| 102 | 24.499 | 9.948 | 59.395 | 19.564 | 7.879 | 59.728 | 21,245 | Yes |
| 103 | 25.559 | 11.012 | 56.913 | 20.365 | 8.790 | 56.839 | 23,582 | Yes |
| 104 | 25.011 | 10.571 | 57.733 | 19.877 | 8.366 | 57.911 | 22,972 | Yes |
| 105 | 25.975 | 10.886 | 58.088 | 20.570 | 8.738 | 57.520 | 23,278 | Yes |
| 106 | 23.155 | 9.406 | 59.379 | 18.403 | 7.559 | 58.924 | 20,415 | Yes |
| 107 | 29.182 | 11.269 | 61.384 | 23.333 | 9.104 | 60.982 | 23,758 | Yes |
| 108 | 25.981 | 11.242 | 56.730 | 21.005 | 9.114 | 56.611 | 24,055 | Yes |
| 109 | 25.037 | 10.435 | 58.323 | 19.981 | 8.412 | 57.901 | 22,113 | Yes |
| 110 | 25.265 | 10.128 | 59.911 | 19.930 | 8.119 | 59.262 | 21,766 | Yes |
| 111 | 25.845 | 11.212 | 56.617 | 20.873 | 9.121 | 56.304 | 23,874 | Yes |
| 112 | 34.206 | 13.825 | 59.582 | 27.407 | 11.162 | 59.275 | 28,782 | Yes |
| 113 | 22.743 | 9.667 | 57.492 | 18.619 | 7.809 | 58.057 | 20,613 | Yes |
| 114 | 25.596 | 10.921 | 57.332 | 20.509 | 8.713 | 57.517 | 23,187 | Yes |
| 115 | 24.318 | 10.593 | 56.440 | 19.516 | 8.475 | 56.574 | 22,191 | Yes |
| 116 | 30.425 | 12.078 | 60.302 | 24.623 | 9.805 | 60.178 | 25,481 | Yes |
| 117 | 26.330 | 10.903 | 58.593 | 21.002 | 8.808 | 58.063 | 23,403 | Yes |
| 118 | 21.567 | 8.835 | 59.034 | 17.215 | 7.082 | 58.861 | 19,263 | Yes |
| 119 | 23.409 | 9.879 | 57.798 | 18.665 | 7.763 | 58.407 | 21,330 | Yes |
| 120 | 22.304 | 9.402 | 57.845 | 17.614 | 7.506 | 57.384 | 20,180 | Yes |
| 121 | 23.322 | 9.899 | 57.555 | 18.643 | 7.990 | 57.144 | 21,297 | Yes |
| 122 | 24.348 | 10.460 | 57.040 | 19.694 | 8.323 | 57.741 | 22,296 | Yes |
| 123 | 23.086 | 10.136 | 56.093 | 18.848 | 8.212 | 56.428 | 21,812 | Yes |
| 124 | 22.594 | 9.653 | 57.276 | 18.024 | 7.765 | 56.918 | 20,798 | Yes |
| 125 | 21.694 | 9.079 | 58.151 | 17.404 | 7.357 | 57.726 | 19,720 | Yes |
| 126 | 25.434 | 10.907 | 57.116 | 20.113 | 8.732 | 56.583 | 23,723 | Yes |
| 127 | 25.317 | 10.214 | 59.655 | 20.422 | 8.310 | 59.307 | 21,906 | Yes |
| 128 | 23.999 | 9.900 | 58.750 | 18.869 | 7.751 | 58.923 | 21,254 | Yes |
| 129 | 23.260 | 9.941 | 57.262 | 18.557 | 7.911 | 57.366 | 21,268 | Yes |
| 130 | 21.977 | 8.947 | 59.288 | 17.271 | 7.218 | 58.208 | 19,480 | Yes |
| 131 | 29.141 | 11.612 | 60.152 | 23.493 | 9.288 | 60.464 | 24,575 | Yes |

## 6. 等价性、压力测试与负控制

| Gate / test | 覆盖对象 | 通过条件 | 当前结果 | 作用 |
|---|---|---|---|---|
| Independent moment oracle | mixture weights、means、covariances | 手工重算 moment 与 `projectLmbObjectMoments` 一致 | Pass | 降低“同一 helper 自证”的循环性 |
| Weight sanitation | negative / nonfinite / all-zero mapped weights | 归零、重归一化或 uniform fallback 与契约一致 | Pass | 固定 projection semantics |
| Covariance canonicalization | asymmetric component covariance | 使用 `C(A)=(A+A^T)/2` | Pass | 对齐 sender、codec、receiver |
| Cholesky regularization | ill-conditioned / singular covariance | 固定 jitter schedule 后得到 SPD；重复调用稳定 | Pass | 对齐论文 `R` 与实际数值路径 |
| Wire codec round-trip | labels、`r`、weights、means、upper-triangle covariance | canonical binary64 fields value-preserving | Pass | 检查真实 serialization boundary |
| Full-vs-moment fusion | 不同 labels、component counts、近对称与病态 covariance | fused labels、`r`、`mu`、`Sigma` exact | Pass | 验证 Proposition 1 的 executed path |
| Field mutation test | labels、`r`、`mu`、`Sigma` 分别扰动 | comparator 必须拒绝 exact match | Pass | 证明 audit 对字段变化敏感 |
| **Covariance-inflation negative control** | moment messages 加 `0.25 I` | `exactMatch=false` 且 covariance residual `>0` | **Pass** | 证明 claim 条件不是装饰性约束 |
| Snapshot capture | 2-sensor short filter run | 每 sensor/time 均产生紧凑 snapshot | Pass | 验证生产调用链的 capture 接口 |
| Evidence corruption tests | CSV cell、human-readable report number、artifact ownership | validator 必须拒绝 | Pass | 证明证据包不是只检查隐藏 aggregate |
| Frozen N50 validator | MAT → CSV/aggregates/bootstrap/Markdown | 逐行与逐字节重算一致 | Pass | 绑定最终论文数字 |

## 7. Claim--Evidence--Boundary 对照表

| 可写入论文的 claim | 直接证据 | 允许的措辞 | 禁止外推 |
|---|---|---|---|
| Sender-side projection preserves this receiver's fused output | `P∘T=P`、`T∘P=P`、字段级 N50 exact audit | `exact for the specified executed receiver` | 一般 GM-KLA density equivalence |
| Moment exchange reduces attempted application payload | 50 paired per-trial reductions；mean 58.277264%；CI | `reduced attempted application-layer bytes by 58.28% in this workload` | radio energy、latency、airtime、end-to-end traffic |
| Reduction is stable across frozen trials | 50/50 reductions positive；range 55.92%--61.38%；SD 1.30 pp | `all paired trials reduced attempted bytes` | 其他 state dimension、mixture multiplicity 或 sensor model 具有相同百分比 |
| Tracking behavior is unchanged | all saved metrics delta 0；`r/mu/Sigma` residual 0 | `all audited outputs and tracking metrics were identical` | mixture densities 相同 |
| Serialization boundary is covered | full encode/decode vs sender projection + encode/decode；codec tests | `crosses the actual application-layer serialization boundary` | packetization、MTU、retransmission、PHY framing |
| Exactness requires an explicit contract | negative control、common `C/P/R/T`、same masks/weights/labels | `under matched labels, weights, masks, and numerical conventions` | quantization、covariance inflation、different pruning、multi-round reuse |

## 8. 证据产物与只读复现

| 项目 | 值或入口 |
|---|---|
| Evidence schema | `fusion-sufficient-moment-exchange-v2` |
| Immutable batch identity | `confirmatory-primary-seeds82-131-v2` |
| Execution commit | `7974f10179a8973875bec9f301b8a5f84477d860` |
| Octave / workers | Octave `11.1.0`；fixed `maxWorkers=6`；仅为 execution provenance |
| Frozen config SHA-256 | `b21c92e99beb8e23371037e3ba5d690a2b447292036109534bfc45850fd07e6c` |
| Required-source manifest SHA-256 | `479bb72fbc0282692521a99b4bb569e912572215660fb2c22eeb1ae5a121de79` |
| MAT | `RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.mat` |
| MAT SHA-256 | `8ad63429d72a75e028bd7b9a1745bce7a047e4f9558443ea1bf547cfd69ab81f` |
| CSV | `RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.csv` |
| CSV SHA-256 | `1c1c4717ae68009417ff74d5e27c94e8e30fb5f8972b596328b3e12dc29db9a2` |
| Human/machine report | `RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.md` |
| Protocol | `docs/icassp2027_paper/EXPERIMENT_PROTOCOL_CN.md` |
| Figure manifest | `docs/icassp2027_paper/figures/figure_manifest.json` |
| Paper experiment text | `docs/icassp2027_paper/sections/04_experiments.tex` |

安全的只读验证命令：

```bash
octave-cli --quiet --eval "setPath; addpath('RUN/GA'); v=validateFusionSufficientEvidence('RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.mat','RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.csv','RUN/GA/GA_FUSION_SUFFICIENT_MOMENT_EXCHANGE_N50_SEEDS82_131.md'); disp(v);"
```

Focused tests：

```bash
octave-cli --quiet --eval "setPath; addpath('tests'); test_lmb_moment_projection; test_lmb_wire_codec; test_lmb_posterior_equivalence;"
/Users/dex/miniconda3/bin/python3 -m pytest -q tests/test_icassp2027_figures.py tests/test_icassp2027_experiment_report.py
```

> 注意：原 evidence report 中保存的 `runFusionSufficientMomentExchangeConfirmatory(true,50,81)` 是冻结时的历史 regeneration command。Seeds 82--131 已永久 burn，当前不得再次执行该命令；后续只允许读取和验证现有 artifacts。

## 9. 论文与补充材料的结果分配建议

| 位置 | 建议保留内容 | 原因 |
|---|---|---|
| Abstract | 58.28%、95% CI、1,119,037 exact outputs、receiver-specific boundary | 一句话同时回答收益、可信度和边界 |
| Main Table | Full/Moment mean attempted MB、delivered MB、local E-OSPA、consensus OSPA | ICASSP 四页内最紧凑的主结果 |
| Main Figure | 左：mean attempted/delivered footprints；右：50 个 paired attempted footprints | 展示绝对负载和跨 trial 稳定性 |
| Experiments prose | 40,000 snapshots、1,119,037 labels、三项 max residual=0 | 证明不是 rounded-metric coincidence |
| Discussion | byte formula、failure modes、application-layer boundary | 阻止把 58.28% 外推为普遍压缩率 |
| 本文档 / Supplement | 完整 50-seed 表、分布统计、测试与证据 hashes | 便于审计与复现，不挤占四页正文 |
