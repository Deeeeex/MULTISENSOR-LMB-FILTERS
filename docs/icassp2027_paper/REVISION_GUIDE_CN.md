# Receiver-Induced Moment Exchange ICASSP 论文修改方向指南

更新日期：2026-07-10

适用分支：`codex/effective-kla-lmb-paper-revision`

目标稿件：`docs/icassp2027_paper/`（ICASSP 2027，最多 4 页技术内容；官方允许可选第 5 页仅放 references、funding acknowledgments 与 Compliance with Ethical Standards；当前 PDF 采用更严格的第 5 页仅 references）

> 本文档记录从旧四臂稿到 receiver-induced 稿的实施决策与历史问题；其中“旧稿/当前基线”描述保留作追溯，不代表最终状态。最终投稿状态、第二轮盲审与剩余作者确认项以 `FINAL_SUBMISSION_AUDIT_CN.md` 为准。

## 1. 结论先行

当前稿件不是“小修润色”问题，而是**中心论点与证据类型错位**：代码中的 full/light 等价主要来自接收端算子本来就先对每个标签做矩匹配，因此 N50 上的指标一致并不是一个偶然的实验发现，而是一个可以形式化说明、再用实现测试验证的结构性质。现稿把它包装成“图保持优于图稀疏化”的经验故事，又用配置不一致的 dynamic arms 支撑该叙事，容易被审稿人判为 tautological implementation optimization、ablation confounded 或 overclaim。

最终将论文收束为：

> **面向作者自定义 presence-conditioned geometric-average LMB receiver 的 receiver-induced moment message**。若接收端调用链只通过每标签存在概率与规范化一、二阶矩消费 GM-LMB 后验，则发送端可以先施加同一个投影，只传 `(label, r, mean, covariance)`；在相同 active labels、fixed symmetric degree-based/effective source weights、delivery masks、数值算子与规范化 binary64 transport 下，投影前后的 receiver-observable fields 相同。真实应用层编码与配对实验用于验证实现并量化通信收益，而不是用 N50 “发现”等价性。

本文中的 `fusion-sufficient` 仅指“对这里明确指定的融合算子，两个输入表示属于同一输出等价类”，不是 Fisher–Neyman 意义的统计充分性，也不表示两个 posterior density 相同。

最终工作标题：

> **Receiver-Induced Moment Exchange for Distributed LMB Fusion**

一句话贡献：

> We identify the first irreversible projection consumed by a custom presence-conditioned geometric-average LMB receiver, move it to the sender, certify receiver-observable field equivalence across serialization, and validate the resulting application-layer message with paired tests and a frozen multisensor evaluation.

该方向成立的前提是完成两个硬门槛：

1. 用明确、可往返解码的应用层 wire schema 替代“标量数乘 8”的模糊字节估算，并拆分 attempted 与 delivered bytes。
2. 从主论文删除配置不一致的 dynamic-topology arms 及其因果结论，只比较相同图、相同权重、相同丢包抽样下的 full 与 fusion-sufficient 两个 arm。

## 2. 当前基线与事实边界

### 2.1 已核验的实现事实

- `compressLmbPosterior.m` 对每个 active Bernoulli 的 GM 密度做矩匹配，将其改写为单高斯。
- `fuseLmbPosteriorsByLabel.m` 无论输入是 full GM 还是 light single-Gaussian，都会在 canonical Gaussian KLA 前再次对每个标签做矩匹配。
- 但两处当前并非同一个数值算子：发送端对零/非法权重使用 uniform fallback，接收端用 `sum(weights)+eps` 路径；发送端与接收端还可能重复做非幂等的 `rcond` jitter。对抗测试已在近奇异 covariance 上复现非零 full/light 差异，在零权重输入上差异更大。因此“结构上应等价”目前还不等于“代码已对所有合法边界输入实现等价”。
- 只有在抽出共享且幂等的投影、把 solve regularization 与 moment projection 分开后，才能在 active label set、存在概率、融合权重、调度/丢包结果一致且 light covariance inflation 关闭时，将静态两臂的数值一致作为实现的预期结果。
- 当前 `estimateLmbPayloadSize.m` 只计算字段对应的 double 标量数，并假定完整协方差矩阵；它不是序列化器，也没有证明实际可发送/可恢复。
- 当前通信循环只在 delivery 成功后构造 payload 并计数，所以现有 `payloadBytes` 实际代表 delivered scalar-equivalent bytes，不是 attempted/transmitted wire bytes。
- 当前 `compressLmbPosterior.m` 从完整 object struct 复制后改写 GM 字段，仍携带 trajectory 等非消息字段；它不是严格的数据传输对象。
- `effectiveWeightAlgebraicConnectivity` 对方向图做 OR/`max(W,W')` 对称化后计算无向 Laplacian 的第二特征值，只能称为 symmetrized connectivity proxy，不能当作时变有向 KLA 收敛或收缩率。

### 2.2 当前实验事实

- 已有 N50 报告使用配对 seeds 32--81：full-static 为 27.77M delivered scalar-equivalent bytes，light-static 为 11.50M，对应 58.6% 的旧口径下降；两者现有 tracking/consensus 聚合指标相同。
- seeds 32--81 与早期开发 seeds 分离，但最终 arm 是看过该 N50 后决定的，故不能继续称为独立 confirmatory holdout。准确表述应为 “disjoint evaluation batch used for final selection”。
- full-dynamic 与 light-dynamic 并非只改变 payload：二者的融合权重、static-edge bonus 和 fallback 设置不同。它们不能构成 payload×topology 的因果对照。
- 现有 Figure 2 由脚本硬编码聚合数值，不能自动追溯到实验 artifact；现有 Figure 1 的串联箭头也误导为三个顺序阶段，而真正要表达的是 full-receiver-projection 与 sender-projection 两条交换路径。

### 2.3 允许声称与禁止声称

允许声称：

- 对**本文明确实现的 single-round presence-conditioned geometric-average LMB receiver**，每标签矩消息是 consumer-/fusion-sufficient representation；该自定义 receiver 不是 full-source LMB-density KLA 的参考实现。
- 在明示条件下，发送前投影与接收后投影给出相同的融合输入统计量和融合输出。
- 某个明确 schema 的应用层 payload bytes 在配对实验中下降多少。

禁止或必须降级的声称：

- “light posterior 对一般 KLA-LMB/GM-KLA 都保持精确等价”。一般 GM 指数混合依赖完整密度形状，矩并不充分。
- “payload compression 比 event-triggered graph sparsification 更安全”。当前没有公平、同配置的 event-trigger baseline 支撑。
- “保持 `lambda_2` 就保证 KLA 信息流/收敛”。当前量只是对称化 proxy。
- “held-out 50 trials 证明方法泛化”。该批次已参与最终选择，只能作为 selection evidence；应新增冻结方案后的 confirmatory batch。
- “通信减少 58.6%”而不说明 schema、层级和 attempted/delivered 口径。旧值只能作为历史标量估算，最终数字须由新 codec 重算。

## 3. 对抗性模拟审稿

以下三份报告共享同一事实基础，但分别从理论正确性、实验/新颖性、叙事/可读性进行独立攻击。建议不是投票结果；任何一项致命问题都必须处理。

### Reviewer 1：理论与算法正确性

**总体评价：Major Revision / Weak Reject。** 论文捕捉到了一个有用的系统性质，但现稿没有把它写成可验证的算子分解，反而让读者误以为对一般 GM-LMB 的 KLA 做了无损压缩。

**优点**

- 关注接收端实际消费的后验表示，而不是盲目传输完整内部状态，问题具有现实意义。
- 相同静态图上的配对 full/light 设计有潜力给出干净的实现验证。
- LMB 的 label-wise 结构使充分表示可以明确到字段级，而非模糊的“特征压缩”。

**主要问题**

1. 缺少核心命题。应定义每标签矩投影 `P` 和实际接收端融合算子 `F`，明确证明 `F(X)=F(P(X))`，而不是把数值一致留给 N50 表格。
2. 当前等价性实际上由实现中的两次 moment matching 构成，容易被视为 tautology。论文必须说明新贡献不是“发现两个同代码路径结果相同”，而是识别 consumer-sufficient representation、把投影前移并给出边界。
3. “KLA-LMB”命名过宽。早期的 `projected KLA-LMB` 收窄仍可能暗示标准 density-level KLA；最终标题改为 `Distributed LMB Fusion`，正文只把 participating-label update 描述为 geometric-average/KLA form。
4. 缺少失效条件：不同标签剪枝、不同正则化、协方差膨胀、量化、mixture-aware consumer、多轮中间重投影或不同融合权重都可能破坏等价。
5. sender 与 receiver 各自实现 moment matching 会产生漂移风险。应抽出唯一 canonical projection helper，由压缩与融合共同调用。

**次要问题**

- 说明 covariance regularization 的确切时机和 epsilon；测试近奇异协方差。
- 报告 `r`、`mu`、`Sigma` 的最大绝对/相对差，而不只报告 E-OSPA 聚合值。
- 若序列化使用 packed upper triangle，解码后必须显式恢复对称性并验证正定/半正定处理一致。

**接收条件**

- 有一个条件完整的命题与简短证明。
- property/end-to-end tests 验证投影交换律和 codec round trip。
- 标题、摘要和结论都限定到 projected receiver。

### Reviewer 2：新颖性、实验设计与通信核算

**总体评价：Reject in current form，修正后可成为 focused ICASSP paper。** 当前对照存在混杂，字节指标不可复现，closest work 不完整；但若把理论性质与真实编码结合，贡献可以从“工程小技巧”提升为可审计的 representation design。

**优点**

- 58.6% 的旧估算表明 mixture payload 的冗余在该 benchmark 上并非微不足道。
- 配对随机种子、相同 local filtering 和相同 packet draws 是合适的比较基础。
- 论文规模适合 ICASSP 的短文：一个清晰命题、一个协议和一个严谨验证即可。

**主要问题**

1. full-dynamic 与 light-dynamic 配置不同，不能用来得出 topology 结论；应从主稿完全移除，而不是仅改成“ablation”。
2. `scalarCount × 8` 不是 wire bytes。没有字段宽度、header、component count、矩阵 packing、encode/decode 或 attempted traffic，58.6% 无法解释为真实通信量。
3. 只在 delivered 后计数会在丢包环境中低估发送尝试。至少同时报告 attempted application-layer bytes 和 delivered bytes；主通信成本用 attempted bytes，delivered bytes 作为接收信息量。
4. N50 seeds 32--81 已参与最终选择，不应再称 held-out。冻结方法和分析后应跑新的、从未看过的 paired seeds（建议 82--131），并预先固定两臂、指标和容差。
5. 相关工作遗漏了 event-triggered LMB、selective GM component sharing、state/covariance projection 等最接近路线。必须解释本文区别是“consumer-induced sufficient representation + equivalence conditions”，不是泛称通信压缩。
6. 当前图表来自硬编码结果。图必须从保存的 summary/CSV 自动生成，并在脚本中保存 source artifact path/commit。

**次要问题**

- 应明确 application layer 不含 MAC/PHY framing、MTU、重传和网络层 header；丢包模型也与 packet size 无关。
- fresh N50 前先用不相交的 seeds 做 N5 smoke，确认 two-arm runner 和 artifact schema。
- 不再使用任意 pass gates 作为科学显著性证据；报告配对差值、置信区间/分位数及预设数值等价容差。

**接收条件**

- 真实可往返 codec，attempted/delivered 分离。
- 公平 two-arm confirmatory evaluation，结果可由 artifact 一键重画。
- closest-work comparison 明确且引用可核验。

### Reviewer 3：论文故事、图表与可读性

**总体评价：Weak Reject。** 当前故事同时讲 payload、effective graph、dynamic topology 和安全性，四页正文无法把任何一条讲深。最好的论文其实藏在第一段实现观察里，应大胆删掉旁支。

**优点**

- “Transmit what the receiver uses”是容易记忆、适合短会论文的设计原则。
- full path 与 light path 可以用一张 commutative diagram 直观说明。
- 精确等价与显著字节减少的组合比一堆小幅 metric gain 更有辨识度。

**主要问题**

1. 标题 “Light Posterior Exchange” 太泛，也不告诉读者为何 light 足够；最终标题应突出 receiver-induced interface。
2. 摘要最后一句把论文拉回 event-trigger/topology 对抗，但正文没有公平证据，应删除。
3. Introduction 的“图保持”篇幅大于核心算子，贡献列表也混淆 method、perspective 与 validation。应按 `receiver consumes projection → move projection to sender → prove → encode → validate` 的因果链重写。
4. Figure 1 应是两条并行路径并在融合输出处合流，不是三个模块串联。Figure 2 应展示每 seed 的 attempted-byte reduction 和配对绝对 payload，而不是重复表格或把零残差审计做成 checklist。
5. 表格小字体和列过多。主表只保留 full vs sufficient 两臂：attempted bytes、delivered bytes、local E-OSPA、consensus OSPA、max state difference；动态 topology 和 symmetrized lambda proxy 均移出。
6. 页面分配失衡。第 5 页几乎空白，说明相关工作与引用不足；补最近工作可以自然占用 references-only page，但不得把正文挤到第 5 页。

**次要问题**

- 统一使用 `full GM message`、`fusion-sufficient moment message`，避免 light/compressed/projected 三套名称交替。
- 不用 “safer”“robust” 等无定义词；写清条件与观察。
- 结论应先重述 operator result，再给 benchmark 数字，最后列边界；不要重新讨论被删除的 dynamic topology。

**接收条件**

- 全文只有一个中心句，所有段落和图表都服务于它。
- 图中文字在最终双栏缩放后不少于 9 pt，可黑白辨认。
- 第 5 页只包含 references，正文严格在前 4 页。

## 4. Cross-review synthesis

三位 reviewer 的共同判断是：**现稿最有价值的不是一个新的 fusion algorithm，也不是 graph-preservation policy，而是一个由 receiver operator 导出的通信表示。** 修稿不能只在原故事上补实验；必须改变 claim hierarchy。

### 4.1 必须解决的共识问题

1. **理论化结构等价**：将经验“same metrics”提升为带条件的 `F = F ∘ P` 命题。
2. **真实化通信证据**：实现编码、解码与 attempted/delivered accounting，替换 scalar-equivalent 口径。
3. **删除混杂证据**：主文不再出现两个 dynamic arms 的性能比较和因果解释。
4. **冻结后再验证**：旧 N50 作为 development/selection evidence；新 seeds 作为 confirmatory evidence。
5. **收窄适用范围**：始终写 custom presence-conditioned geometric-average LMB receiver；仅说明 participating-label update 具有 KLA form，不推广到标准 full-source LMB-density KLA 或一般 GM-KLA。
6. **可追溯 artifact**：实验输出结构化保存，图表从 artifact 生成，报告记录 commit/config/seeds。

### 4.2 reviewer 之间的张力与裁决

- 理论 reviewer 希望突出精确命题，实验 reviewer 担心它显得显然。裁决：贡献写成“operator-induced message design”，命题保证正确性，codec 和 fresh benchmark 证明它能转化为可测通信收益；不宣称新的 KLA 算法。
- 实验 reviewer 希望真实 wire bytes，短文篇幅有限。裁决：实现最小、版本化、定长字段的 application-layer schema，正文只给公式和字段摘要，详细 layout 留在可复现实验文档/代码。
- 叙事 reviewer 希望彻底删除 topology。裁决：主文只用一句说明我们固定 schedule/graph 以隔离 payload 表示；不展示 dynamic 结果，也不以 topology 为贡献。

## 5. 外部新颖性与 closest-work 边界

修稿需核验并纳入以下最接近文献，避免把已有的 event-triggering 或 posterior projection 重新命名为新颖性：

| 路线 | 代表工作 | 与本文的关系 | 本文必须写清的区别 |
|---|---|---|---|
| Event-triggered LMB | Shen et al., IEEE TSP 2022, DOI `10.1109/TSP.2022.3154227` | 降低通信频率/事件发送 | 本文不改变消息路径或频率，只改变 receiver 所需的表示 |
| LMB communication scheduling | Shen et al., IEEE TCS-II 2023, DOI `10.1109/TCSII.2023.3238346` | 调度/触发式分布式 LMB | 不用当前不公平 dynamic arms 与其竞争 |
| Event-triggered distributed LMB | Shen et al., Signal Processing 2024, DOI `10.1016/j.sigpro.2023.109238` | 与问题场景高度接近 | 本文的核心是算子充分性及等价条件 |
| Event-triggered labeled RFS | Li et al., Signal Processing 2026, DOI `10.1016/j.sigpro.2025.110149` | 更新的触发式 labeled RFS 路线 | 作为 schedule reduction，对比 representation reduction |
| State/covariance projection | Xue et al., Electronics 2026, DOI `10.3390/electronics15020458` | 直接传状态与协方差 | 本文从具体 receiver factorization 推导字段并证明 fusion-output equivalence |
| Selective GM sharing | partial-consensus GM-PHD, IEEE TAES, DOI `10.1109/TAES.2018.2882960` | 选择/截断 mixture components | 本文不是选重要 components，而是传 projected consumer statistics |

引用加入前必须逐条从出版商或论文原文核对题名、作者、年份、页码与具体主张；不能只凭标题推断方法细节。

## 6. Claim–evidence matrix

| 论文 claim | 当前证据 | 缺口 | 修稿决定 | 最终证据门槛 |
|---|---|---|---|---|
| Receiver fusion 只消费每标签 `r, μ, Σ` | 两个 MATLAB 函数的实现路径 | sender/receiver moment matching 是重复实现 | 抽出 shared projection helper | 单元测试覆盖普通、零/非法权重、近奇异 covariance |
| Sender-side projection 保持 fusion output | N50 聚合指标相同 | 没有命题、字段级误差或随机 property test | 写 `F=F∘P` 命题并测试 | 固定随机 GM/labels/weights 下投影幂等且 full/projected `r, μ, Σ` bit-exact |
| Message 可实际传输与恢复 | scalar count 估算 | 无 schema/codec | 实现 versioned binary application-layer codec | full/light round trip；encoded length 是唯一 byte oracle |
| Moment message 减少通信 | 旧 N50 delivered 估算下降 58.6% | 不是 attempted wire bytes | 主报 attempted bytes，辅报 delivered bytes | 新 paired N50 artifact 给出每 seed reduction 与区间 |
| 性能与 full 相同 | 旧 N50 aggregate 完全相同 | batch 已用于 selection；缺字段级比较 | 冻结方案后跑 seeds 82--131 | 配对 tracking delta + state-level max diff；不只写 pass count |
| 方法不改变图/调度 | static arms 配置相同 | 报文尺寸不影响当前 drop model，需明示 | 保留固定 static two-arm design | runner assertion/metadata 记录相同 schedule/drop draws |
| 方法优于 dynamic/event-trigger | 配置混杂的 dynamic arms | 无公平证据 | 删除 claim | 不进入标题、摘要、贡献、图表、结论 |
| `λ₂` 解释有效信息流 | 对称化 proxy | 不能代表有向时变收敛 | 从主结果删除 | 若附录/日志保留，必须改名并限定为 diagnostic proxy |

## 7. 选定故事与被拒绝路线

### 7.1 选定：Story A — consumer-sufficient message

叙事链：

1. 分布式实现常直接发送内部 GM-LMB posterior，消息表示与 receiver computation 没有共同设计。
2. 本文 receiver 的单轮 projected KLA 映射先把每标签 GM 投影为 Bernoulli existence + Gaussian moments。
3. 因此该投影可以交换到发送端；定义并证明交换律及适用条件。
4. 设计一个只编码融合充分字段的 application-layer message，并用唯一 codec 统计 attempted/delivered bytes。
5. 用 property tests 验证输出等价，用冻结后的 paired multisensor batch 量化通信收益。

这条故事的“fancy”之处不在复杂机制，而在**把通信协议系统地推导自消费算子**。它提供一个可迁移的设计问题：对于任意分布式推断模块，先识别 downstream operator 的等价类，再选择消息表示。

### 7.2 拒绝：Story B — graph-preserving compression beats sparsification

拒绝原因：没有公平 event-trigger baseline，dynamic arms 配置混杂；“safer”也没有形式化定义。固定图只应作为控制变量，不是贡献。

### 7.3 拒绝：Story C — novel KLA/LMB fusion algorithm

拒绝原因：融合公式本身未改变；把 moment projection 前移不是新的 KLA 解法。该包装会引来最强的新颖性攻击。

### 7.4 拒绝：Story D — learned/adaptive compression

拒绝原因：当前没有 rate–distortion、量化或 learned codec；短期新增会扩大变量、破坏精确等价并稀释四页故事。

## 8. 修改方向与硬门槛

### Gate 1：统一投影算子

- 新增一个 canonical per-label moment projection helper。
- `compressLmbPosterior` 和 `fuseLmbPosteriorsByLabel` 只能通过该 helper 获取 moments。
- 投影只做权重归一、矩计算与对称化，不做 `rcond` jitter。每个 component covariance 必须先用与 codec 相同的 `(Σ_m+Σ_mᵀ)/2` canonicalization，再进入 mixture covariance 累加，避免“逐 component 对称化”和“最终再对称化”的浮点顺序差。solve 前使用唯一的 Cholesky-first regularizer：已是 SPD 的 covariance 原样通过；只有 Cholesky 失败时才按固定 schedule 加 jitter。precision solve 与 log determinant 复用同一份 regularized copy，`logDet` 内不得再次正则化。
- 禁止 light covariance inflation 进入主实验；若代码保留该功能，runner 必须断言为关闭。

通过标准：旧测试通过；新增 projection/property tests 使用固定 RNG seed `20270710`；`P(P(x))` 对已投影的 binary64 字段 bit-exact；paper 路径 full/light snapshot 的标签及 `r/μ/Σ` 完全一致（最大残差为 0）。

### Gate 2：真实 application-layer codec

- 设计版本化、little-endian、可解码 schema。v1 固定 24-byte header：magic `LMBW` 4 B，version/event type/state dimension/flags 各 1 B，sender/receiver 各 `uint16`，time index/object count/total bytes 各 `uint32`；`flags=uint8(3)`，其中 bit 0 表示 packed upper triangle、bit 1 表示 IEEE-754 binary64，其余位必须为 0。
- 每 object 编码 label、existence、component count；每 Gaussian 编码 weight、mean、对称 covariance 的 upper triangle。
- 每 object 固定头为 20 B：birth time/location 各 `uint32`、`r` 为 binary64、component count/reserved 各 `uint16`。每 component 为 `8[1+d+d(d+1)/2]` B；总长应严格满足 `24 + sum_l{20 + M_l 8[1+d+d(d+1)/2]}`。不要继续把所有 metadata 当 double。
- full 与 sufficient message 共用一个 codec，区别仅为 component count 与投影位置。
- successful delivery 必须走 decode 后的对象进入 receiver；不能只 encode 为了计数而继续传内存 struct。
- decoder 从 `model.object` 创建同字段模板，但 trajectory/timestamps 是接收端本地 bookkeeping，不进入消息；full 与 sufficient 两臂都遵循这一语义。
- application-layer 范围明确排除 MAC/PHY、MTU、网络 header 与重传；当前 drop draw 不依赖 payload size，作为实验限制披露。

通过标准：full/light round-trip test；header 有独立 endian/offset golden fixture；packed covariance 对称恢复；truncated/version/flags/dimension/metadata 越界错误可检测；`numel(bytes)` 是唯一 payload byte source。

### Gate 3：attempted/delivered accounting

- 在 delivery draw 前构造并编码 attempted message，记录 `attemptedPayloadBytes`。
- 仅 delivery 成功后 decode，记录 `deliveredPayloadBytes`。
- 保留 `payloadBytes` 作为 delivered compatibility alias，防止破坏旧 runner，但新论文不使用模糊名。
- summary 报告 payload delivery ratio：当 attempted bytes 为 0 时定义为 0，否则严格为 delivered/attempted；并断言两臂使用相同 attempted event mask 和 delivered mask。

通过标准：强制全丢包/全成功测试能区分 attempted 与 delivered；旧接口兼容；summary 维度与 edge-time mask 正确。

### Gate 4：公平且冻结的实验

- 新建 paper-specific two-arm runner：full-static 与 fusion-sufficient-static。
- 两臂固定相同 topology、fusion/existence weights、packet draws、label threshold、simulation length；禁用 dynamic fallback、static-edge bonus、covariance inflation 和按 light/heavy message type 改权的 mode-aware fusion，或显式断言两类 factor 完全相同。
- 先提交并 push runner/protocol，固定指标与首批 confirmatory seeds 82--131；再用完全不相交的 seeds 1001--1005 做 N5 smoke。若 smoke 触发任何代码/配置修改，必须重新提交并 push 后才能启动 confirmatory batch。
- 旧 seeds 32--81 只作为 development evidence，不再叫 held-out。
- 主结果报告每 seed attempted byte reduction、delivered byte reduction、tracking paired deltas、state-level max discrepancy。对每个 seed 定义 attempted-byte 百分比下降 `ρ_s=100(B_full,s-B_proj,s)/B_full,s`，主数字是 50 个 `ρ_s` 的算术均值；其 95% 区间固定为对这 50 个配对 `ρ_s` 有放回重采样的 percentile bootstrap（RNG seed `20270710`，10,000 resamples，2.5/97.5 percentiles），不以任意 acceptance gate 替代统计描述。

通过标准：runner dry-run 显示仅两臂且配置一致；结构化 `.mat`、machine-readable CSV 和 Markdown report 齐全；N50 完成且可追溯到 commit。

### Gate 5：数据驱动图表与论文重写

- Figure 1 改为 commutative diagram：`full GM → receiver projection → projected KLA` 与 `sender projection → moment wire message → projected KLA` 两条路径在输出处相等。
- Figure 2 从 confirmatory artifact 读取每 seed 数据，展示 byte reduction 分布和配对绝对 payload；fusion/state 零残差保留在命题、正文与结构化审计中，不硬编码 aggregate。
- 主表只保留公平两臂；移除 dynamic arms 和 effective lambda2。
- 标题、摘要、Introduction、Method、Experiments、Conclusion 全部围绕 Story A 重写。
- Related Work 加入 closest work，并用 frequency/schedule reduction、component selection、operator-sufficient representation 三类区分。
- 正文前 4 页；当前第 5 页仅 references（官方亦允许 funding acknowledgments 与 ethics compliance）；图中文字最终尺寸可读，无 overfull box 或内容裁切。

通过标准：figure tests 校验 source artifact/hash 与输出；PDF 文本与视觉 QA 通过；claim–evidence matrix 中每个保留 claim 都能指向命题、测试或 artifact。

## 9. 预注册式验收标准

在 fresh confirmatory run 前固定以下标准，运行后不得按结果改门槛：

### 9.1 正确性

- Canonical projection 在确定性单元测试及固定随机 property tests 中通过。
- codec round trip 后 label、component count、`r`、weights、means 以及严格对称 covariance bit-exact；允许输入的近似对称 covariance 以 `(Σ+Σᵀ)/2` 为唯一解码 oracle。
- 对同一组 full inputs，直接 full fusion 与先投影/编码/解码后的 fusion 输出具有完全相同的 label set 和 binary64 `r/μ/Σ`；confirmatory snapshot 中三个最大残差均必须为 0。

### 9.2 因果隔离

- two arms 的 seed、measurement/trajectory realization、attempted mask、delivered mask、topology、spatial weights 和 existence weights逐项相同。
- 唯一实验变量是 sender 是否在编码前施加 canonical projection。

### 9.3 通信核算

- 主指标为 attempted application-layer bytes；delivered bytes 单独报告。
- 所有 byte 值来自 codec byte-array length，不从 object struct 或 scalar count 反推。
- 报告 full/sufficient 的总量、每 seed 分布、配对百分比下降和固定的 paired percentile bootstrap 95% CI（RNG seed `20270710`，10,000 resamples）。

### 9.4 论文完整性

- 不出现 “generally lossless GM-KLA compression”“safer than event triggering”“held-out seeds 32--81”“dynamic topology proves ...” 等无证据表述。
- proposition 的 assumptions 与实验配置一一对应。
- 所有论文数字由结构化 artifact 生成并可由一条命令复现图表/表格。
- ICASSP PDF 满足 4+1 页规则；第 5 页无正文、图、表或 acknowledgment。

## 10. 执行顺序与关键提交

1. **Revision strategy checkpoint**：提交本指南和 implementation plan，push。
2. **Correctness checkpoint**：shared projection + tests，commit/push。
3. **Wire-accounting checkpoint**：codec + attempted/delivered integration + tests，commit/push。
4. **Experiment checkpoint**：paper runner、artifact schema 与 protocol 先 commit/push；完成 disjoint N5 smoke 后，若无改动则该 commit 直接冻结，若有改动则重新 commit/push 再冻结。
5. **Confirmatory evidence checkpoint**：运行 fresh N50，提交报告、CSV 与哈希一致的 89 KiB raw `.mat`；tee log 保持在 ignored `.superpowers/logs/`，commit/push。
6. **Paper checkpoint**：data-driven figures、references、full rewrite、PDF，commit/push。
7. **Final audit checkpoint**：全测试、artifact provenance、LaTeX/视觉/页数/claim audit，通过后最终 commit/push。

若 Gate 1--3 任一失败，停止新 N50。Primary identity 固定为 `confirmatory-primary-seeds82-131-v2`；在 worker directory 外取得 stable reservation 的瞬间即永久 burn `82--131`，不得通过删除目录、补 MAT 或 selective retry 复用。当前 runner 故意只允许 primary `82--131`，不预先启用 fallback。若该 identity 进入 FAILED/BURNED，必须保留 tombstone/attempt ledger，修复后以新 commit 提交预注册 amendment，才可增加未观察的 `132:181` 和新 identity。若 worker 已形成 hash-bound `COMPLETE_WORKERS` receipt 但主进程在 assemble/publish 阶段中断，则只能恢复 assembly/publish，不得重算 worker。若新的 disjoint batch 仍不满足预注册的精确等价条件，论文必须把 claim 改为近似 representation 并解释误差来源，不能只挑聚合 tracking 指标维持“exact”叙事。

## 11. Confirmatory 后的独立复核与最终裁决

三路只读 reviewer 在 N50 完成后重新审查了当前代码、证据、LaTeX 与渲染 PDF：

1. **技术正确性审查**确认 shared projection、codec、唯一变量与字段级 gate 成立，
   但发现 evidence commit 使原 production validator 因 `HEAD != execution commit`
   而拒绝只读重算。该问题已修复：execution/publish 继续使用严格 frozen-HEAD
   gate；read-only validation 改为核对 execution commit 为当前历史祖先，并在该
   commit 上重算 required-source manifest。原始 MAT 已按报告 SHA 纳入版本控制。
2. **新颖性/意义审查**将旧稿评为 Weak Reject，但判断 Story A 完整实现后可达到
   Borderline Accept / Weak Accept。其核心保留意见是避免把算子分解包装成新的
   KLA 算法，必须以 operator-induced interface design 化解 tautology 攻击。
3. **可读性/版面审查**确认旧 PDF 虽机械满足 4+1 页，但 Fig. 1/2 最终图内字约
   3--5 pt，Fig. 2 有重叠且仍硬编码旧四臂结果；第 5 页仅两条参考文献，closest
   work 覆盖不足。

复核后的 gate 状态：

| Gate | 状态 | 权威证据 |
|---|---|---|
| Shared projection / exact output | 通过 | focused tests；N50 `r/μ/Σ` 最大残差均为 0 |
| Typed codec / attempted-delivered accounting | 通过 | codec tests；receiver 使用 decode 后对象 |
| Frozen two-arm confirmatory batch | 通过 | seeds `82:131`，50/50 exact，masks 全相同 |
| Primary communication result | 通过 | mean `58.277264%`，95% CI `[57.923222,58.636095]%` |
| Independently reproducible evidence | 通过 | tracked MAT/CSV/MD；descendant-safe read-only validator |
| Evidence-driven figures | 通过 | deterministic manifest/test；无 dynamic arm 或硬编码 aggregate |
| Full manuscript rewrite | 通过 | 标题、摘要、Related Work、`P/F/G` 命题、two-arm 结果、Discussion 与结论均已重写 |
| 4+1 final PDF | 通过 | `tests/check_icassp2027_pdf.py`：exact 5 pages；当前第 5 页仅 references；全页 PNG 视觉复核 |
| Font/figure/float QA | 通过 | TeX Gyre Termes regular/bold/italic 无 Latin Modern fallback；图内估算最小 9.038 pt；Fig. 1/2 分置第 2/3 页，Table 1 位于第 4 页的 Experiments 段 |
| Final claim/style audit | 通过 | 无旧 dynamic/held-out/effective-graph 主张；closest work 与应用层 byte 边界已显式限定 |

最终 red-team 又要求收紧 Proposition 1：source-indexed active-label presence、
Gaussian/Bernoulli 共用的 fixed symmetric degree-based/effective source weights、selection/pruning、共同的 $\mathcal C/\mathcal P/\mathcal R$、禁用 covariance
inflation 与规范化 binary64 字段的 value-preserving transport 现在均列为条件；proof 逐项说明 missing-label 重归一化、
`K,h,eta,q0,q1` 不变，不再只写一行幂等恒等式。版面终审发现的 XeTeX `ptm`
字体回退、图内 5--6 pt 字号和结果图过早出现也已修复并进入自动 gate。

最终版本完成了 Story A 的全部实施步骤。两张图按双栏全宽排版，正文占前四页，
当前第 5 页仅列参考文献；技术图、公式、表格没有裁切或横向溢出。Discussion 正面
处理了三项最强反驳：代数恒等式是否循环验证、58.28% 是否可泛化、相同
canonical binary64 实现是否夸大外部有效性。当前裁决为 **implementation pass**，
而不是自动投稿许可；作者身份现已按 Jinhao Chen 第一作者、Tianyu Wo 通讯作者写入。ICASSP 2027 CFP、single-anonymous policy、4+1 页规则、funding/COI 和 ethics requirement 已于 2026-07-11 核对；作者同日确认本稿无 funding 且全体作者无相关 financial/nonfinancial interests，声明已写入 Acknowledgment。官方 detailed submission/template 链接仍为 404，paper kit 上线后还需复核模板。若后续改动重新引入 dynamic-topology superiority、一般 GM-KLA 等价或未
限定的 network-cost claim，本指南仍自动判定该版本失败。

## 12. 2026-07-11 最终对抗性复审与处置

本轮三路只读复审分别侧重技术正确性、原创性/相关工作、可读性/ICASSP 合规。主代理重新核对代码、Git provenance、冻结证据、当前 PDF 与 ICASSP 2027 官方页面后逐项裁决：

| 复审问题 | 处置 | 当前状态 |
|---|---|---|
| audited receiver 来源不明，容易被当成普遍 consensus-LMB 路径 | 明确为 authors' custom presence-conditioned geometric-average LMB receiver；标题移除 `KLA-LMB`，正文声明并非 full-source density-KLA reference implementation | 已解决 |
| moment matching 本身可能被包装成新意 | 引用 Runnalls 2007，并在摘要/引言明确贡献是跨 serialization 的 executable interface certificate | 已解决 |
| historical `metropolis` 配置名与实际权重不符 | 论文统一改为 fixed symmetric degree-based base weights，并报告 regular 4+4 图上 self `1/3`、四邻居各 `1/6`；protocol 保留历史字段名但加纠错说明 | 已解决 |
| Proposition 未定义极端 finite binary64 导致中间量非有限的边界 | 定义 admissible numerical domain：`P` 字段与所有 quadratic/logdet/exp/Bernoulli intermediates 有限，且每次 `R` 成功 | 已解决 |
| 命题把 self 也写成经过 transport | 改为 `F(pi_i,T pi_-i)=F(pi_i,T P pi_-i)`，对应实际 remote-message path | 已解决 |
| N50 snapshot 在 common retention 后捕获，旧措辞像 raw fusion output audit | 摘要、实验、结论和实验文档统一为 `retained post-step fields`；raw pre-retention equality 由 Proposition 1 建立 | 已解决 |
| 单轮命题到 100-step filter-level equality 缺显式桥梁 | Discussion 增加 deterministic retention/prediction/update 的归纳说明 | 已解决 |
| 场景不足以复现 workload-specific byte ratio | 正文补充 4-D CV state、2-D measurement、10 staggered targets、FoV、`r>1e-2` retention 与 degree-based weights | 已解决 |
| 普通 lossless/entropy coding 反驳未回应 | Discussion 明确 representation comparison 不是 entropy coding 或 rate-optimality claim | 已解决 |
| Fig. 1 输入语义与公式记号不一致 | 改为 local GM-LMB posterior、full-GM/moment codec 路径和全文一致的 `pi,F_(omega,R),T,P` | 已解决 |
| Fig. 2 identity 注释交叠、百分比符号歧义、`MB` 歧义 | 删除斜向文字，百分比直接按 reduction 展示，轴统一为 `10^6 B/trial` | 已解决 |
| 参考文献专名和电子 article number 格式 | 保护 Bayes/Kullback--Leibler/Bernoulli 大小写，Xue 2026 改为 `Art. no. 458`；保持 20 条正文引用 | 已解决 |
| Keywords/metadata 仍可能暗示 full KLA 或无法被上传工具识别 | `KLA fusion` 精确化为 `geometric-average fusion`；PDF title/author/keywords metadata 与正文绑定并纳入 checker | 已解决 |
| Funding 与 COI/无 COI 声明 | 作者确认本稿无 funding、全体作者无相关 financial/nonfinancial interests；官方示例已写入 Acknowledgment | 已解决 |
| 投稿 PDF/source 包容易与主稿漂移 | 增加确定性最小 source ZIP、内部 source manifest、upload SHA-256、隔离解压编译与严格 PDF gate | 已解决 |
| 最终 ICASSP 2027 template/submission instructions | 官方链接截至 2026-07-11 返回 404；当前 PDF 已按 CFP、single-anonymous 与 4+1 规则通过 | **EXTERNAL PENDING** |

官方核对入口：

- CFP/page rule：<https://2027.ieeeicassp.org/wp-content/uploads/sites/13/2019/08/ICASSP2027-CallForPapers-July1.pdf>
- Funding/COI 与 ethics policy：<https://2027.ieeeicassp.org/about/sps-policies/>
- Single-anonymous 与 review criteria：<https://2027.ieeeicassp.org/about/editorial-policies/>
- Detailed submission page（2026-07-11 为 404）：<https://2027.ieeeicassp.org/paper-submission-instructions/>

当前技术、版面、作者声明与隔离 submission-bundle 门禁均通过。正式 paper kit 上线并完成模板差异检查后，才可把状态提升为最终 **submission pass**。

`tests/check_icassp2027_pdf.py --require-submission-declarations` 是最终声明门禁：作者确认的无 funding、无 COI 文本写入后，该严格模式必须与普通 PDF gate 一并通过。
