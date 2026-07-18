# ICASSP Receiver-Induced Moment Exchange 实验进度与结果总表

更新日期：2026-07-18

状态：**冻结的 primary N50 已完成；证据已通过只读 validator；不得重跑 seeds 82--131**

对应论文：`Receiver-Induced Moment Exchange for Distributed LMB Fusion`

> 一句话结论：在冻结的 50 个八传感器配对试验中，唯一改变发送端是否在编码前执行 canonical moment projection；moment message 将 attempted application-layer payload 平均降低 **58.277264%**（95% bootstrap CI：**57.923222%--58.636095%**），同时 common `r>1e-2` retention 之后的 40,000 个 sensor-time snapshots 和 1,119,037 个 retained label-instance comparisons 的 `r`、`mu`、`Sigma` 残差均严格为 0。

## 0. 论文主叙事：由接收端反推消息接口

> **核心定位：**本文不是提出新的 KLA/LMB 融合规则，也不是把标准 moment matching 包装成新的压缩算法；本文识别出一个固定接收端的首个不可逆映射，把该映射的输出提升为真实 wire message，并用显式的数值契约、传输契约和冻结配对实验给出一份可执行的等价性证书。

### 0.1 背景：发送内容与接收端实际使用的信息不匹配

分布式多目标跟踪需要在不集中原始量测的前提下交换各节点的后验信息。LMB 后验为每个标签维护存在概率 `r` 和条件状态密度；在本文软件中，后者是 Gaussian mixture (GM)，因而一个 active label 可能携带多个 component 的权重、均值和协方差。若直接发送 full GM，应用层报文会随 active labels、状态维数和每标签 mixture multiplicity 增长。

然而，本文审计的接收端不是保留完整 mixture 结构的 full-source LMB-density KLA。它执行的是一个 **custom presence-conditioned geometric-average receiver**：先按标签对齐实际出现的来源、在这些来源上重新归一化固定 base weights，再把每个来源的 label-wise GM 投影为 `(r, mu, Sigma)`，最后才计算 Gaussian overlap、存在概率和融合后的单 Gaussian。也就是说，full GM arm 先跨过真实 serialization boundary 传输全部 components，接收端随后立即丢弃其中不能通过一、二阶矩影响后续计算的结构。

这里的矛盾不是“GM 信息在一般意义上无用”，而是**发送端表示比这个特定接收端的可观测接口更丰富**。已有通信感知方法主要回答何时发送、选择哪些 components 或是否只交换紧凑状态；moment-preserving projection 本身也早已是标准工具。本文提出的不同问题是：

> 对一个已经确定的接收端，能否从其首个不可逆映射反推出更紧凑的消息接口，并证明该接口在真实 codec 和数值实现之后仍保持所有 receiver-observable fields？

### 0.2 方法设计：把首个不可逆映射移到发送端

把 executed receiver 写成

$$
\mathcal F_{\omega,\mathcal R}
=
\mathcal G_{\omega,\mathcal R}\circ\mathcal P ,
$$

其中各映射的职责必须固定，而不能只在概念层面说“发送 moments”：

| 符号 | 具体职责 | 为什么属于等价性契约 |
|---|---|---|
| `C` | 用 `C(A)=(A+A^T)/2` 规范化 component covariance | 防止 sender、codec 和 receiver 对近对称矩阵采用不同解释 |
| `P` | 清洗并重归一化 mixture weights，应用 `C`，逐标签保留 `r` 并计算 canonical `mu`、`Sigma` | 这是接收端读取 GM 后执行的首个不可逆映射 |
| `T` | versioned little-endian binary64 encode/decode 的 posterior-content map | 保证结论跨过实际 application-layer serialization boundary，而非只比较内存对象 |
| `R` | 固定 jitter schedule 的 deterministic Cholesky regularizer | 固定逆矩阵、log determinant 和 overlap 的实际数值语义 |
| `G_(omega,R)` | 按 active-label presence 归一化 source weights，再计算 `K`、`h`、`eta`、`q0`、`q1` 和最终 `(r,mu,Sigma)` | 明确哪些 downstream fields 必须被消息完整保留 |

设 `pi_i` 是 receiver `i` 的本地后验，`pi_-i` 是实际送达的邻居后验元组。两条路径的唯一设计差异是 `P` 位于 codec 的哪一侧：

| 路径 | 发送端与 wire payload | 接收端实际使用 |
|---|---|---|
| Full GM arm | `pi_-i -> T(pi_-i)`；编码每个 label 的全部 GM components | 解码后执行 `P`，再进入 `G_(omega,R)` |
| Moment arm | `pi_-i -> P(pi_-i) -> T(P(pi_-i))`；每个 label 只编码一个 canonical Gaussian | 解码后仍走同一接收端；`P` 再次执行但因 idempotence 不改变内容 |

在 admissible numerical domain 内，canonical projection 和 transport 满足

$$
\mathcal P\circ\mathcal T=\mathcal P,\qquad
\mathcal T\circ\mathcal P=\mathcal P .
$$

因此，在 active-label presence、delivery masks、presence-normalized effective source weights、`C/P/R/T` 完全相同，且 mode-aware weighting 与 covariance inflation 均关闭时，

$$
\mathcal F_{\omega,\mathcal R}
(\pi_i,\mathcal T\boldsymbol\pi_{-i})
=
\mathcal F_{\omega,\mathcal R}
(\pi_i,\mathcal T\mathcal P\boldsymbol\pi_{-i}) .
$$

这个等式表达的是 **executed receiver-output equivalence**：两条路径为后续的 `K`、`h`、`eta`、`q0` 和 `q1` 提供完全相同的输入，从而得到相同的融合后 `r`、`mu`、`Sigma`。它不声称 full GM 与 moment message 表示相同的 mixture density；本地 `pi_i` 也没有被当作网络报文处理。

### 0.3 设计思想：从“少发字段”升级为“可执行证书”

| 设计原则 | 本文的具体实现 | 避免的薄弱叙事 |
|---|---|---|
| **Receiver-first** | 先固定实际 consumer `F=G∘P`，再定义消息，而不是先设计一个通用 compact payload | 不把 receiver-specific 结论外推为一般 GM-KLA 等价 |
| **移动最早的丢弃点** | 将 receiver 原本必做的 `P` 移到 sender、编码之前；只删除该 receiver 后续无法观测的 mixture 结构 | 不把启发式 component pruning 误写成 exact 方法 |
| **固定 canonical semantics** | sender、codec、receiver 共用 weight sanitation、covariance symmetrization、moment projection 和 regularizer | 不用“数学上应当接近”代替可复现的浮点实现契约 |
| **跨真实传输边界验证** | 两臂均实际 encode；attempted bytes 在 delivery draw 前计数，成功送达后实际 decode | 不用 MATLAB object 大小或标量个数估算通信量 |
| **只改变一个因果变量** | seeds、trajectories、measurements、graph、schedule、labels、weights 和 delivery uniforms 全部配对；仅改变 sender-side projection | 不让 topology、触发策略或丢包差异混入 payload representation 的效果 |
| **把 exactness 与 savings 分开证明** | 命题和 property tests 证明 receiver contract；N50 audit 检查递归后的 retained fields；codec bytes 独立量化收益 | 不以 rounded tracking metrics 相同替代字段级等价 |
| **主动给出失效地图** | 明列 mixture-aware consumer、quantization、不同 numerical maps、covariance inflation、不同 pruning 和 multi-round reuse | 不把“在当前契约下 exact”写成无条件或普适结论 |

这里最关键的设计判断是：**消息格式应由 consumer 的信息需求决定，而不是由 producer 的内部表示直接决定。** 但只有当该 consumer 能分解为一个明确的不可逆投影和其后的确定性计算、且投影能够与 codec 组成稳定的 canonical map 时，才可以把这个判断提升为 exact interface claim。否则，moment message 只能被当作近似或启发式压缩。

### 0.4 论文论证链：背景、方法、证据与边界如何衔接

| 叙事节点 | 要回答的问题 | 本文给出的回答或证据 |
|---|---|---|
| 1. 背景 | 为什么 full GM communication 值得处理？ | 每标签报文随 component count 增长，而 distributed LMB 需要重复跨节点交换后验 |
| 2. 实现观察 | 为什么不是泛泛地做 GM compression？ | 当前 receiver 在任何融合计算前都会执行相同的 label-wise `P`，造成 producer representation 与 consumer interface 不匹配 |
| 3. 方法洞见 | 我们究竟改变了什么？ | 将 receiver 的首个不可逆映射 `P` 提前到 sender；融合规则 `G`、schedule 和 tracker 均不改变 |
| 4. 精确性主张 | 为什么提前投影不会改变该 receiver？ | `P∘T=P`、`T∘P=P` 加上 matched presence/weights/masks/numerics，给出 Proposition 1 的 executed-output equality |
| 5. 实现可信度 | 短代数是否掩盖了工程差异？ | versioned codec、independent oracle、regularization tests、field mutation tests 和 covariance-inflation negative control 覆盖真实调用链 |
| 6. 递归证据 | 单次融合等价是否延伸到完整 tracker？ | 冻结 N50 的 common-retention audit 覆盖 40,000 snapshots 与 1,119,037 retained label instances，字段最大残差均为 0 |
| 7. 通信收益 | exact interface 实际节省多少？ | 50 个配对 trials 的 mean attempted-byte reduction 为 58.277264%，且所有 trials 均为正；这是当前 workload 的 application-layer 结果 |
| 8. 结论边界 | 读者不应从结果推断什么？ | 不推断 mixture-density equality，也不推断 radio energy、latency、airtime、rate optimality 或其他 consumer/codec 下的等价性；N50 证据限于一个八传感器模型、固定 4+4 graph、binary64 codec 和每 step 一轮同步 fusion |

因此，正文应始终维持同一条主线：**表示不匹配 → receiver factorization → sender-side projection → executable equivalence certificate → frozen paired evidence → workload-specific savings 与显式 failure modes**。其中，58.28% 是该设计在当前 workload 下的结果，不是论文的概念起点；真正可复用的思想是“识别 consumer 的首个不可逆映射，并检验它能否成为跨 serialization boundary 的消息接口”。

### 0.5 可直接用于组内沟通的一段话

分布式 LMB 节点通常以 Gaussian mixtures 表示每个标签的状态，因此直接发送 full GM 会携带所有 component；但我们当前实现的 custom presence-conditioned geometric-average receiver 在真正融合前，必然先把每个来源、每个标签投影为存在概率和一、二阶矩。本文据此从接收端反推消息设计：把这个 canonical projection 从解码后移到发送端编码前，并用统一的 covariance canonicalization、deterministic regularizer 和 versioned binary64 codec 固定两条路径的数值与传输语义。Proposition 1 证明，在标签 presence、delivery masks、effective weights 和数值映射一致的 admissible domain 内，full GM message 与 moment message 对该 receiver 产生完全相同的融合输出；property tests、负控制及冻结的 50-trial 递归审计进一步验证了真实实现。在此契约下，moment message 将 attempted application-layer bytes 平均降低 58.28%，而 1,119,037 个 retained post-step label instances 的 `r`、`mu`、`Sigma` 均严格一致。因而本文的贡献是 receiver-induced message interface 及其 executable certificate，而不是新的 KLA、一般 GM density equivalence 或无线层压缩结论。

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
