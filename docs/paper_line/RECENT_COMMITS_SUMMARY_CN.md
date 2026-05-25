# 最近几个 Commit 总结

本文档总结 `paper` 分支最近 6 个实质性提交的主要内容，时间范围为 `2026-04-08` 到 `2026-04-15`。其中未单独展开 `6c5d64b Ignore local tooling artifacts`，因为该提交主要是本地工具产物清理，不属于论文主线变更。

## 一页概览

这几次提交的整体方向，不是去大幅改动核心滤波代码，而是围绕论文主线做三件事：

- 补齐论文素材与写作基础设施，包括图资产和 LaTeX 模板。
- 为自适应融合权重设计补上可 defend 的理论附录。
- 将论文叙事收敛到 `consensus-oriented` 主线，并把主结果、消融和 supporting evidence 组织成稳定表述。

## 按提交总结

| 日期 | Commit | 主题 | 主要内容 |
| --- | --- | --- | --- |
| 2026-04-15 | `64a1cf2` | 摘要收口与结果整理 | 精修摘要与标题文案；新增中文结果汇总文档，系统整理主场景、消融、理想通信 supporting evidence、AA 泛化和附录次线结果；将 manuscript 中的摘要同步到最新主结果口径。 |
| 2026-04-11 | `2d819f2` | 指标叙事与题目重构 | 明确把 `consensus` 指标设为主评价指标、常规 local tracking 指标设为 safeguard；将标题方向收敛到 `Communication-Aware Adaptive Weights for Consensus-Oriented Distributed KLA-Based LMB Fusion`；同步改写问题定义、实验、结果、结论和附录的叙事口径，并补充新的 tiered-link ablation 运行记录与参考文献。 |
| 2026-04-09 | `90e446a` | 理论附录加强 | 在 Appendix C 基础上补强推导，把自适应权重 backbone 解释为 entropy-regularized simplex allocation，并进一步形式化 branch-decoupled weighting、GA-LMB merging 的分支结构、KL 正则化图先验等理论解释；补充相关参考文献。 |
| 2026-04-09 | `4b7ade2` | 新增自适应融合理论附录 | 首次加入完整的 adaptive fusion 理论附录，新增工作推导笔记 `working_adaptive_weight_derivation.tex` 和论文附录 `appendix_c.tex`，把“branch-decoupled + weak structure-aware”这条方法线从经验设计提升为可写入正文/附录的理论说明。 |
| 2026-04-08 | `7adf420` | 建立论文 LaTeX 主体 | 引入 Elsevier CAS LaTeX 模板与构建脚本，建立 `manuscript.tex`、分章节正文、附录、参考文献和图文件目录，基本完成论文写作骨架；同时扩充 theory references，为后续理论附录和问题表述做准备。 |
| 2026-04-08 | `0778465` | 补充论文图形资产 | 新增 `figure1.svg` 和 `figure2.svg` 两个论文 SVG 资产，为系统总览图和权重因子分解图提供可直接纳入论文的矢量素材。 |

## 详细解读

### 1. `0778465`：先把论文图资产补齐

这是这一轮论文整理工作的起点。提交新增了两张关键的 SVG 图：

- `figure1.svg`：用于系统/方法整体结构表达。
- `figure2.svg`：用于 adaptive weight factorization 的概念表达。

这一步本身不改变实验或算法，但它为后续 LaTeX 成稿提供了最基础的视觉素材。

### 2. `7adf420`：把论文从零散笔记推进到可编译 manuscrpt

这个提交是写作基础设施层面的最大一次铺垫，核心动作包括：

- 引入 Elsevier CAS 模板、样例、样式文件和构建脚本。
- 新建 `manuscript.tex` 以及 `introduction`、`related work`、`problem formulation`、`method`、`experimental setup`、`results`、`conclusion`、附录等章节文件。
- 建立论文图文件目录和参考文献库，把已有结果和图资产接入可投稿的 LaTeX 结构。

可以把它看成“把论文工程搭起来”的提交。之后的工作不再是分散笔记，而是在正式 manuscrpt 上持续收敛。

### 3. `4b7ade2`：首次加入自适应融合理论附录

这个提交把当前方法的理论说明从正文外的想法，推进成正式附录材料。新增内容的重点包括：

- 独立推导笔记 `working_adaptive_weight_derivation.tex`。
- 正式论文附录 `appendix_c.tex`。
- 在 manuscrpt 中接入 Appendix C。

它的价值在于：开始系统解释为什么当前实现中的 `adaptive branch-decoupled fusion` 不是纯经验拼接，而是能够和 KLA/LMB 的结构对应起来。

### 4. `90e446a`：把理论附录从“有”提升到“更能 defend”

紧接着上一提交，这次重点不是再加框架，而是强化论证密度。附录中被进一步明确的理论点包括：

- adaptive weight backbone 可解释为带熵正则的 simplex 分配。
- GA-LMB merger 的 spatial / existence 分支结构可以被精确写出。
- Bernoulli-RFS 分解、aligned-LMB surrogate、branch-wise optimization、图先验 KL 正则化等关系被整理成命题级表述。

这一步的结果是：论文的理论支撑不再只是“有一个附录”，而是开始具备相对完整的解释链条。

### 5. `2d819f2`：把论文主线正式切到 `consensus-oriented`

这是最近几次提交里最关键的一次“叙事重构”。主要变化不是新实验本身，而是重新定义论文到底在证明什么。

这个提交明确了两层评价逻辑：

- `primary`：网络层共识误差指标，如 `OSPA consensus error`、`matched localization disagreement`、`cardinality dispersion`。
- `secondary`：常规 truth-referenced 的 local tracking 指标，用来证明一致性提升不是靠牺牲局部跟踪精度换来的。

同时，这个提交还做了几件配套工作：

- 标题方向从一般性的 adaptive fusion，收敛到 `communication-aware` 和 `consensus-oriented`。
- 问题定义中更明确地引入 `realized link quality`、`existence confidence`、branch-decoupled weighting 和 weak structure-aware refinement。
- 结果章节改成“主场景 + 因子消融 + ideal communication supporting evidence + 次线模块简述”的结构。
- 新增 `RUN/GA/GA_TIERED_LINK_ABLATION_20260410_143517.md`，把当前主结果对应的运行记录固化下来。

从论文写作角度看，这个提交完成了“主张收口”：论文不再泛泛讨论分布式融合，而是明确主打“异构通信条件下，如何提升跨节点一致性”。

### 6. `64a1cf2`：用摘要和结果整理把主线最终收口

这次提交是在上一轮叙事重构基础上的收尾与压缩，重点有两部分：

- 精修摘要和标题文案，使摘要直接对应最新 headline 结果。
- 新增 [PAPER_MAIN_RESULTS_CN.md](/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/docs/PAPER_MAIN_RESULTS_CN.md)，把当前最强证据链单独整理成中文总结。

这份结果整理文档做了几件很重要的事：

- 明确主场景是 `8-sensor distributed GA-LMB` 在 `tiered heterogeneous packet loss` 下的结果。
- 明确最强方法链是 `covariance + link quality + existence confidence + weak structure-aware decoupled KLA`。
- 用逐步消融说明 `covariance -> link quality -> existence confidence -> weak structure-aware refinement` 的贡献路径。
- 区分主结果、supporting evidence 和附录次线模块，避免正文叙事被 `NIS`、`history`、`freshness` 等次线分散。

可以说，这个提交完成了“对外表达版本”的整理，使当前论文已经具备比较明确的摘要、题目和结果主张。

## 整体演进脉络

如果把这 6 个提交串起来看，演进顺序非常清晰：

1. 先补论文图资产。
2. 再搭出可编译的 LaTeX manuscrpt 骨架。
3. 然后为 adaptive fusion 加入并强化理论附录。
4. 最后把论文问题定义、指标体系、标题和结果表述统一收敛到 `consensus-oriented` 主线。

因此，这一轮提交的本质不是“算法从零到一重写”，而是把已有实验和方法，逐步包装成一篇主线明确、论证更完整、结果更容易 defend 的论文稿。

## 当前阶段结论

截至这几次提交，论文主线已经基本稳定为：

- 研究对象：异构通信条件下的分布式 KLA-based LMB fusion。
- 核心方法：`covariance + realized link quality + existence confidence + weak structure-aware decoupled refinement`。
- 主要目标：优先提升跨节点 `consensus` 质量，而不是只追求单节点局部跟踪指标。
- 证据结构：tiered heterogeneous packet-loss 主场景为 headline，ideal communication 和 AA 作为 supporting evidence，`NIS/history/freshness` 等作为附录次线。

如果后续继续推进，这份提交序列之后最自然的工作通常会是两类：

- 继续补实验稳定性和正式表格/图注。
- 基于当前 manuscrpt 做语言润色、压缩篇幅和投稿格式对齐。
