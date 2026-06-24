# TAES 稿件 Readiness 审计

日期: 2026-06-25 04:57 CST

目标稿件: `Neighborhood Label-Barycenter LMB Fusion for Distributed Multi-Target Tracking under Unreliable Communication`

目标期刊: IEEE Transactions on Aerospace and Electronic Systems (TAES), Regular Paper, Technical area `Target Tracking and Multi-Sensor Systems`

## 当前判断

稿件已经进入 `content_ready_metadata_pending` 阶段: TAES 模板、核心方法叙事、理论性质、N50 主实验、ablation、runtime、带 CI/wins/sign-test p 的 baseSeed=11 held-out N50 robustness check、harsh packet-loss N50 stress check、topology-ring N50、partial-FOV N50 和 full-topology N50 scenario-family checks、以及 disclosure skeleton 都已经落到 `main.tex` 和 `generated/` evidence fragments 中。按“作者/基金/repository 可先占位”的项目约定，当前非元数据 mechanical/evidence/citation/PDF/source-bundle gates 已经闭合；`generated/SUBMISSION_READINESS_REPORT.md` 显示 metadata allowance 后无 blocking gate，也没有剩余内容 warning。本轮已把 reproducibility ledger 扩展为覆盖 primary AA N50、held-out AA N50、harsh-loss AA N50、scenario-family boundary checks、contextual GA N50 和 independent verifier，并把 topology-ring、partial-FOV 与 full-topology 均升级为 paper-grade scenario-family evidence；其中 full-topology 被解释为 zero-disagreement equivalence boundary，而不是额外增益。method section 和 readiness checker 已明确 projection 是 labels/moments-only 的 active-track 层，不重估 Bernoulli existence。稿件还不能标记为 portal-submission-ready，因为投稿元数据仍缺最后闭环:

1. 作者、单位、收稿/修回日期、期卷页/DOI、基金、repository DOI/URL、corresponding author、cover-letter signature、preprint/conflict/reviewer 等投稿元数据仍是占位符。
2. 当前实证已从主 tiered packet-loss formation 扩展到更严 harsh packet-loss profile、sparse topology-ring N50、partial-FOV N50 和 full-topology zero-disagreement ceiling；target maneuver、covariance-consistency 和 recursive-online 场景族验证仍会降低审稿风险。

## 04:57 Checkpoint

本轮把固定参数 full-topology N50 ceiling run 完成后接入证据链。原始报告为 `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED51_20260625_000938.md`，配置为 `Trials=50`、`baseSeed=51`、`scenarioLabel=full-topology-formation`、`neighborMapMode=full`，报告 SHA-256 为 `1424b0456ea5dbe6425a30b77aaabc8a45dbb92df064ae3d1bd64379b9f8effe`。

结果不是新的增益证据，而是理想 full-neighborhood 上界: 三个 arms 的 Network OSPA、localization disagreement、cardinality dispersion 全部为 `0.000000`，local E-OSPA/RMSE/CardErr 也完全相同，分别为 `1.634331`、`3.277037`、`0.073400`。因此 `extract_scenario_family_evidence.py` 将该场景分类为 `zero_disagreement_ceiling_equivalence`，generated scenario-family table 对零分母 reduction 记为 `n/a`，Discussion 只把它写成 equivalence boundary。

同步更新范围: `evidence_sources.json`、scenario-family extractor、generated manifest/section/summary sentence、正文 reproducibility/discussion wording、supplement/response package、claim-evidence map、reviewer risk register、figure/table audit、metadata checklist、AA generalization protocols 和 readiness checker。边界保持不变: full-topology 不能写成 deployed graph-local topology setting、不能写成额外 method gain，也不能触发 `H`、threshold、projection cutoff、barycenter weights、label rules 或 packet-loss settings 的回调搜索。

## 04:28 Checkpoint

本轮把 reference-only ablation 的理论支撑向前推进一步。`main.tex` 在 `Moment-space projection` 证明中补充 barycenter-versus-copying identity: 在固定匹配的 first-two-moment 坐标中，复制任一 matched reference input 的组内平方 moment error 等于 barycenter 最优误差再加上 `M||q_r-\bar{q}||^2`，因此 full barycenter 在内部 moment-space 目标上不劣于 reference-only copying，除非被复制的 reference 已经等于 barycenter。

该命题不新增实验证据，也不把内部 moment-space 不等式扩展成 truth-error guarantee；正文边界句明确说明若 assignment 错误，较低的组内 moment error 仍可能描述错误 group。`CLAIM_EVIDENCE_BOUNDARY_MAP.md` 和 `check_submission_readiness.py` 已同步这条命题及其边界，用来支撑 `matched posterior barycenters, not label copying alone` 的 ablation 叙事。

## 03:17 Checkpoint

本轮把 supplement decision 从“最终再决定”改成可检查的默认策略。当前 content-ready package 默认不上传独立 supplementary material；held-out N50 表留在正文，harsh packet-loss、topology/FOV 和 reproducibility ledger 作为 response-ready / source-bundle-ready evidence。只有最终 page budget、TAES portal、editor 或 reviewers 要求时，才从 `generated/stress_harsh_section.tex`、`generated/scenario_family_section.tex`、`generated/reproducibility_ledger_table.tex`，以及必要时移动出正文的 `generated/heldout_n50_section.tex` 中选择正式 supplement 内容。

`SUPPLEMENTARY_EVIDENCE_PACKAGE.md`、`SUPPLEMENTARY_README_DRAFT.md`、`SUBMISSION_PACKAGE_INDEX.md`、`FINAL_METADATA_CLOSURE_CHECKLIST.md` 和 `COVER_LETTER_AND_METADATA_DRAFT.md` 已同步该默认策略。`check_submission_readiness.py` 现在要求 supplement package/README 保留默认不上传、eligible block set、raw `RUN/` logs 和 scratch reports 禁止项，以及 parsed full-topology N50 只能作为 zero-disagreement equivalence boundary、不能写成 gain claim 的边界。

## 03:30 Checkpoint

本轮重查 TAES 官方 author-information、TAES home、technical-area descriptions 和 AESS AI-generated-content 页面，未发现与当前 package strategy 冲突的要求漂移。当前审计结论不变: 按 Regular Paper 投稿，technical area 选 `Target Tracking and Multi-Sensor Systems`，当前 9 页 PDF 低于 10 printed pages overlength-charge threshold，supplement/Code Ocean/DataPort/graphical abstract 均作为最终 portal metadata 决策而非 content-readiness 阻塞项。`docs/TAES_SUBMISSION_REQUIREMENTS_CN.md` 已记录本次 03:30 official-source recheck。

## 03:55 Checkpoint

本轮按 `nature-polishing` 的 algorithmic-paper / Results-Discussion 边界继续做低风险可读性收束。Fig. 3 的 report-driven reduction bar fragment 现在在每个 full/reference-only bar 末端显示 paired percentage reduction 数字，caption 同步说明 bar-end labels 的含义；这让主实验机制证据在图内自洽，不要求审稿人回查正文或表格才能读出 effect size。该改动只改变 `extract_n50_evidence.py` 生成的 LaTeX visual fragment 和 figure caption，不改变任何 raw report、metric、statistical test、source manifest 或 paper-facing claim。

本轮视觉检查 `tmp/pdf_visual_qa/main_all_p08.png` 确认 Fig. 3 的数字标签可读且未与 Table V、Discussion heading 或正文重叠。后续仍需在最终 metadata 替换后重跑 `./build.sh`、readiness checker 和 extracted source-bundle fallback。

## 04:01 Checkpoint

本轮把 reviewer-style preflight 中最容易漂移的边界转成机器检查: `check_submission_readiness.py` 新增 `next-stage generalization protocol` gate，要求 `docs/AA_NEXT_STAGE_GENERALIZATION_PROTOCOL_CN.md` 明确 maneuver/crossing、covariance/reliability 和 recursive-online A/B/C 是 fixed no-search future risk-reduction plans, not current manuscript evidence，也不是 portal upload 或 source-bundle evidence artifact。这样后续继续做方法级泛化实验时，不会把 smoke、mixed、negative 或尚未 gated 的结果误写成当前 TAES 稿件 claim。

`README.md` 和 `SUBMISSION_PACKAGE_INDEX.md` 已同步说明该 protocol 由 readiness checker 验证，但仍保持在 manuscript source bundle 之外。这个 checkpoint 不改变正文、实验数字、generated evidence 或 full-topology 状态；它只把“当前 paper 不等待 A/B/C extension，且不能把 future protocol 当 evidence”的投稿边界固化。

## 04:11 Checkpoint

本轮按 reviewer readability 视角重做 Fig. 1 的生成逻辑。`render_figures.py` 现在把 method pipeline 表达为 `input active neighborhood LMB outputs -> reference -> assignment -> barycenter -> output active tracks`，并在图内明确 output 只 pass through upstream existence score、rewrite labels/moments。`main.tex` caption 同步这一点，强调输入不是 global label dictionary。该改动只提高方法图的信息密度和复用可读性，不改变方法定义、实验结果、source reports 或 readiness claim。

视觉检查 `tmp/pdf_visual_qa/main_all_p04.png` 显示更新后的 Fig. 1 在单栏宽度下可读，没有明显文字溢出或与 Fig. 2/Structural Properties 正文重叠。full-topology ceiling run 仍未完成，本 checkpoint 没有把它接入 evidence chain。

## 04:20 Checkpoint

本轮新增 `FIGURE_TABLE_AUDIT.md`，把主文 Fig. 1、Fig. 2、Table I--V、Fig. 3 以及 response-ready/source-bundle 的 harsh-loss、scenario-family 和 reproducibility ledger tables 逐项映射到 reader task、source/generation path、visual QA target 和 claim boundary。`check_submission_readiness.py` 新增 `figure/table audit ledger` gate，`create_submission_bundle.py`、`README.md` 和 `SUBMISSION_PACKAGE_INDEX.md` 已同步把该 ledger 纳入 source-bundle/self-documenting submission package。

这个 checkpoint 不改变正文 claim、实验结果或图表内容；它把“图表不得引入 `CLAIM_EVIDENCE_BOUNDARY_MAP.md` 之外的 claim”“contextual GA rows 不能写成 paired AA-vs-GA significance”“full-topology ceiling report 未 gate 前不得入图表/claim”等规则变成可检查的投稿 QA。

## 03:09 Checkpoint

本轮把 cover letter 从“证据角色说明”提升为“编辑可直接看到核心证据”的投稿信草稿。`COVER_LETTER_AND_METADATA_DRAFT.md` 现在加入主 paired N50 的三项核心结果: network OSPA disagreement 降低 `81.59%`、local E-OSPA 降低 `17.15%`、RMSE 降低 `6.35%`；同时写明 reference-only RMSE 只有 `0.54%`，held-out N50 复现实验保留 full barycenter vs label copying 的 RMSE separation (`6.64%` vs `0.82%`)。`check_submission_readiness.py` 的 cover-letter positioning gate 已新增这些定量 marker，防止最终 metadata 替换时把编辑最需要看到的机制证据删掉。

## 02:57 Checkpoint

本轮补充 `Cardinality-equivalent control` 命题，并把它纳入 `check_submission_readiness.py` 的 `cardinality-equivalent ablation theory` gate。该命题说明 reference-only projection 与 full moment-barycenter projection 在同一 active-output 初始化、同一 median-cardinality reference-selection rule 下，每轮输出 cardinality 相同；因此 Card. disp. 与 CardErr 这类 cardinality-only 指标只能证明共享 reference-cardinality projection，不能单独证明 moment barycenter 的 spatial gain。Results 中对应表述已从事后解释改为由结构命题预测的机制隔离解释。

## 02:49 Checkpoint

本轮补充 paper-facing overclaim hygiene gate。`check_submission_readiness.py` 现在会扫描摘要和正文，禁止出现 unsupported universal、recursive、assignment-guarantee、fusion-weight-optimization 等过强 claim patterns，例如 replacing AA/KLA、optimizing fusion weights、guaranteeing correct assignment、validated recursive LMB、all target dynamics 或 all communication topologies。当前稿件通过该 gate；它和已有 implementation-alignment、conclusion boundary、stress/generalization boundary gates 一起，把 output-level active-track projection 的 claim 边界转成机器化保护。

## 02:43 Checkpoint

本轮按 reviewer-style preflight 修正摘要中的一个 scope 风险。旧摘要句子写成 scalar AA/KLA weights do not decide comparable components `before existence and spatial fusion`，容易被理解为本文 projection 进入或替代 Bernoulli existence consumer；这比当前实现和正文边界更强。新版摘要改为 active Bernoulli output components 的 label/moment correspondence，和 Method 中 `after the upstream AA existence update and thresholding`、`does not replace the Bernoulli existence consumer` 的实现边界一致。

`check_submission_readiness.py` 新增 `abstract active-output boundary` gate，要求摘要保留 active-output / label-moment correspondence markers，并禁止旧的 `before existence and spatial fusion` 过强说法。`./build.sh` 后该 gate、abstract sentence load、implementation-alignment wording、PDF visual QA、TeX overfull 和 source-bundle freshness 均通过。这个改动只收紧 claim boundary，不改变任何实验数字、generated evidence 或 full-topology 状态。

## 02:35 Checkpoint

本轮把 PDF visual QA 从代表页抽查升级为全页覆盖。`render_pdf_visual_qa.py` 现在每次构建会同时渲染六张代表页、9 页全页 PNG 和 `tmp/pdf_visual_qa/main_contact_sheet.png`；contact sheet 采用无字体依赖的缩略图拼接流程，避免 `magick montage` 在缺省字体配置下失败。`check_submission_readiness.py` 新增 `PDF visual QA full-page coverage` 和 `PDF visual QA contact sheet` gates，并把 stale-output cleanup 扩展为同时覆盖 `main_p*.png`、`main_all_p*.png` 和 `main_contact_sheet.png`。

本次 `./build.sh`、全页 contact-sheet 视觉检查和解压后的 `TAES_EVIDENCE_MODE=bundled ./build.sh` source-bundle fallback 均通过。`generated/SUBMISSION_READINESS_REPORT.md` 仍显示 `content_status=content_ready_metadata_pending`、metadata allowance 后无 blocking gate；唯一 pending 仍是作者/基金/repository 等投稿元数据占位符。这个 checkpoint 不改变正文 claim、实验结果或 evidence source，也不接入仍在运行的 full-topology N50。

## 02:23 Checkpoint

本轮把 TeX 版面日志纳入 readiness gate。`build.sh` 现在会把 LaTeX engine 输出写入 `tmp/build/latex_build.log`，`check_submission_readiness.py` 新增 `TeX build log` 和 `TeX overfull box warnings` 两个检查: 前者确认最新构建日志存在，后者要求 latest build log 中没有 `Overfull \hbox` 或 `Overfull \vbox`。当前构建记录了既有 underfull warnings，但没有 overfull warning；因此新增 gate 通过。这个改动把“是否有表格/文字伸出栏宽”的检查从人工盯终端转成机器化投稿 QA，不改变正文内容、实验结果或证据链。

## 02:11 Checkpoint

本轮清理 PDF visual-QA 的可复现性细节。`render_pdf_visual_qa.py` 现在每次渲染前删除旧的 `tmp/pdf_visual_qa/main_p*.png`，避免 9 页稿件目录里残留旧版 `main_p10_references.png`、manual screenshot 或过期页码图。`check_submission_readiness.py` 新增 `PDF visual QA stale-output cleanup` gate，要求 `tmp/pdf_visual_qa/` 中实际存在的 rendered page images 必须全部被当前 `generated/pdf_visual_qa.json` 引用。本次重建已删除 11 个旧渲染图，目录只剩当前 manifest 对应的 title/abstract、method、main-results、heldout-runtime、discussion-conclusion 和 references 六张代表页图。这个改动不影响正文内容、实验结果或 source bundle 证据，只减少最终人工逐页审阅时的 stale artifact 风险。

## 02:04 Checkpoint

本轮按 PDF 视觉审查修正 Table I。原表把三臂 hypothesis 写成段落式长文本，单栏下扫描负担偏高；新版改为 mechanism switch matrix，直接展示 `AA route`、`Ref. label`、`Moment avg.` 在 tuned baseline、reference-only 和 label-barycenter 三个 arm 中的 on/off 关系。对应的 claim 解释仍保留在 Experimental Setup 正文中。这个改动不改变实验设计、指标、evidence source 或 generated results；它只让机制隔离设计更接近审稿人快速扫描的表格形态，并消除了本轮试排时出现的 Table I overfull warning。

## 01:53 Checkpoint

本轮继续压缩第一页 abstract 的版面负载。上一版句子负载已经通过，但摘要尾句跨到第一页右栏顶部，会让 Index Terms 前出现一段未标注的 abstract continuation。新版摘要从 252 个词降到约 224 个词、13 句、最长 25 个词；仍保留 LMB/AA/KLA/OSPA/E-OSPA/RMSE/FOV 的首次定义，以及 `component-correspondence failure`、AA/KLA scalar-weight boundary、`reference-only ablation`、`A held-out 50-trial replication`、`full-versus-reference separation` 和 fixed-parameter boundary checks。该修改不改变任何实验、数值、generated evidence 或 full-topology 状态。

## 01:47 Checkpoint

本轮按用户确认的思路，把 `nature-polishing` 作为期刊稿件的 reader-path / claim-evidence-boundary 质量层使用，但不改变 TAES template、page-budget、source-bundle 或 submission metadata 约束。摘要已从较长的结果堆叠句收紧为 14 个短句，按 `problem -> gap -> method -> primary evidence -> held-out replication -> boundary -> implication` 展开；所有摘要句子均不超过 30 个词，同时保留 `component-correspondence failure`、AA/KLA scalar-weight boundary、`reference-only ablation`、`A held-out 50-trial replication` 和 `full-versus-reference separation` 等机器检查 marker。

`check_submission_readiness.py` 新增 `abstract sentence load` gate，把这个可读性要求固化为机械检查，防止后续压缩 abstract 时重新变成长句堆叠。这个 checkpoint 不新增实验、不改变任何手写或 generated metric、不接入仍在运行的 `full-topology` N50；它只提高第一页的期刊读者可读性和后续改稿的退化保护。

## 03:48 Checkpoint

本轮按 Nature-style polishing / reviewer audit 做了低风险正文收束，不改变任何实验数值、证据源或统计结论。`main.tex` 的 abstract 末句从 “matched barycenters repair the label/moment contract” 改成 “supply the label/moment contract”，降低机制措辞的过度修复感；同时把 fixed-parameter robustness wording 改成 preserve the same full-versus-reference separation，使 abstract 的证据边界更具体。

Results 中 runtime 句子从一个冒号长句拆成三句，明确 full/reference-only runtime overhead 分别为 `1.644` / `1.534` times fixed baseline，且 assignment/output rewriting 是主要开销来源。Discussion 中 harsh-loss、topology-ring、partial-FOV 的生成摘要现在独立成段，最后用一条更短的 boundary sentence 总结: these checks broaden packet-loss severity, sparse-topology coverage, and partial-field-of-view sensing, but still preserve formation-family assumptions and do not substitute for maneuvering-target or covariance-consistency studies。

验证状态: `python3 -m py_compile docs/paper/taes/manuscript/scripts/check_submission_readiness.py` 通过，`git diff --check` 通过，`./build.sh` 通过；PDF 仍为 `9` pages，readiness 仍为 `content_ready_metadata_pending` 且 metadata allowance 后无 blocking gates。视觉检查 `main_all_p08.png` 与 `main_all_p09.png` 显示 Discussion/Conclusion/References 分页可接受。full-topology run 仍在运行中，本 checkpoint 没有把它并入证据链。

## 01:35 Checkpoint

本轮按 Nature-style reviewer audit 的逻辑继续收束一个 wording 风险: `tuned spatial-KLA AA baseline` 容易被审稿人理解成针对当前数据集搜索式调参。`main.tex` 已把 paper-facing 叙事和机制隔离表统一为 `fixed target-wise spatial-KLA AA baseline` / `Fixed spatial-KLA AA`，并把 raw validation report 中的内部 implementation label 仅作为 provenance 处理。`COVER_LETTER_AND_METADATA_DRAFT.md`、`CLAIM_EVIDENCE_BOUNDARY_MAP.md`、`REVIEWER_RISK_REGISTER.md` 和生成脚本同步这一点。

`check_submission_readiness.py` 的 `fixed-design baseline wording` gate 现在要求正文保留 `fixed target-wise AA-LMB baseline`、`Raw validation reports retain an internal implementation label`、`all manuscript tables and comparisons use the fixed spatial-KLA AA baseline name`、`fixed parameterization rather than per-scenario retuning`、`no per-scenario search over projection cutoffs, barycenter weights, thresholds, or trial-specific label rules` 等 marker。这个 checkpoint 不改变实验参数、raw evidence 或 full-topology 状态，只把“方法贡献不是数据调参”变成可机械检查的投稿约束。

## 00:55 Checkpoint

本轮按 `nature-writing` / `nature-polishing` 的 algorithmic-paper 工作流，把 `CLAIM_EVIDENCE_BOUNDARY_MAP.md` 从单纯 claim/evidence 表扩展为完整的 manuscript argument ledger。新增 `One-Sentence Argument` 锁定本文的一行主张: unreliable peer-to-peer LMB tracking 中，target-wise AA weight routing 已固定后仍可能缺失 Bernoulli component correspondence；graph-local reference、assignment 和 first-two-moment barycenter projection 只修复 active output-track 的 label/moment correspondence，当前证据来自 paired N50 ablation、held-out replication 和 fixed-parameter boundary checks，而 recursive lifecycle、covariance consistency、maneuver/crossing behavior 仍在当前 claim 之外。

同一文件新增 `Reader Path` 和 `Section Job Map`，逐节规定 Abstract、Introduction、Related Work、Problem Formulation、Method、Structural Properties、Experimental Setup、Results、Discussion 和 Conclusion 的唯一主要任务、必须保留的边界和必须避免的过度表述。`check_submission_readiness.py` 已把这些 marker 纳入 `claim-evidence-boundary map` gate；`README.md` 和 `SUBMISSION_PACKAGE_INDEX.md` 也同步说明该 artifact 是一行主张、reader path、section-job、terminology、claim/evidence 与 non-claim 的统一内审入口。这个改动不新增科学 claim，也不改变正文数值；它的作用是在后续 metadata、cover letter、supplement 或 reviewer response 写作时，防止把当前 output-level projection 扩写成递归滤波、全局 label management 或 scalar-weight optimization。

## 00:59 Checkpoint

本轮按文献定位审查补齐 Related Work 的近年相邻工作覆盖。通过 DOI/CrossRef 元数据核对后，正文新增并引用两条最核心近年相关文献: multiview LMB fusion without feedback (`Jin2023MultiviewLMB`) 和 efficient label matching for distributed LMB tracking (`Ding2025EfficientLabelMatching`)。正文只做 bounded positioning: 承认这些是相邻基础和对照，明确本文增量是 active-output correspondence projection，并用 reference-only ablation 分离 label copying 与 matched posterior barycentering；不宣称这些 prior works 忽略 label matching 或不如本文。

`check_submission_readiness.py` 新增 `recent label-matching literature positioning` gate，要求 Related Work 保留 multiview、efficient-label-matching 和 reference-only ablation contrast markers。`CLAIM_EVIDENCE_BOUNDARY_MAP.md` 也新增对应 claim/evidence/boundary 行，防止后续把这些文献写成 strawman。试加四篇相邻文献会把 PDF 推到 10 页，因此主文保留最能支撑 correspondence-contract framing 的两篇；同时用这两篇替换一个较弱的 TechRxiv 预印本和一个泛用 consensus conference 引用，把有限 reference budget 转向更贴近 label/correspondence 的同领域工作。下一步重建时应确认 BibTeX DOI resolver 保持全通过，且 PDF 回到 9 页/10 页阈值以下。

## 01:02 Checkpoint

本轮按预投稿审稿视角把新增相邻文献纳入 `REVIEWER_RISK_REGISTER.md`。新增的风险项回答“本文和 multiview LMB fusion / efficient label matching 的差异是什么”: 当前 defensible contrast 不是说这些 prior works 忽略 labels 或不如本文，而是本文聚焦 active-output correspondence projection layer，并通过 reference-only ablation 分离 label copying 与 matched posterior barycentering 的机制贡献。`check_submission_readiness.py` 的 reviewer-risk gate 已同步要求 `Jin2023MultiviewLMB`、`Ding2025EfficientLabelMatching`、`active-output correspondence projection layer` 等 marker，防止后续 reviewer response planning 退回 strawman 式对比。

## 01:15 Checkpoint

本轮新增 `FINAL_METADATA_CLOSURE_CHECKLIST.md`，把当前 readiness report 中仍阻塞 portal submission 的 metadata placeholders 转成可执行替换清单。该文件覆盖 author/front-matter、affiliation/email/ORCID、funding、repository DOI/URL、Code Ocean、DataPort、preprint/conflict/reviewer fields、graphical/video abstract、supplementary-material decision，以及 post-acceptance-only 的 issue/DOI/date 字段。它明确 metadata closure 期间不得改变实验参数、evidence-source paths、generated metric fragments，也不得接入未完成的 full-topology 结果或把方法写宽成非 active-output label/moment correspondence projection。

`create_submission_bundle.py` 已把该清单纳入 deterministic source bundle；`check_submission_readiness.py` 新增 `final metadata closure checklist` gate，并把该文件加入 required artifacts 与 bundle-required paths；`README.md` 和 `SUBMISSION_PACKAGE_INDEX.md` 同步说明它是最终提交前的占位符闭环入口。这个 checkpoint 不改变主文科学内容，只减少实际 portal submission 前漏填或错填 metadata 的风险。

## 00:50 Checkpoint

本轮把原先混在中文审计里的 claim-to-evidence matrix 抽成独立英文投稿包 artifact: `CLAIM_EVIDENCE_BOUNDARY_MAP.md`。该文件不是新的数据源，也不应被正文引用；它用于最终投稿和回复审稿前逐条检查 paper-facing claims、manuscript location、evidence artifact、verification level、boundary wording、terminology ledger 和 explicit non-claims。这样可以防止 abstract、cover letter、supplement 或 response text 在最后阶段把 output-level active-track projection 写宽成递归 LMB 更新、AA/KLA 替代品、协方差一致性保证或 full-topology 未完成证据。

`create_submission_bundle.py` 已把该文件纳入 deterministic source bundle；`check_submission_readiness.py` 新增 `claim-evidence-boundary map` gate，并要求保留核心 claim/evidence/non-claim markers；`README.md`、`SUBMISSION_PACKAGE_INDEX.md` 和 `SUPPLEMENTARY_EVIDENCE_PACKAGE.md` 也同步说明它的内部 QA 用途和非数据源边界。下一步重建后应确认 source bundle freshness、readiness gate 和 extracted bundled build 都通过。

## 22:30 Checkpoint

本轮把投稿材料里的 `COVER_LETTER_AND_METADATA_DRAFT.md` 从“包内说明”加强为真正面向 TAES editor 的 cover letter。新增内容明确了 TAES fit: 问题来自 distributed aerospace/surveillance tracking 中 unreliable peer-to-peer communication 造成的 LMB component correspondence failure；方法不是 another scalar-weight search，而是 graph-local reference selection、assignment 和 matched posterior moment barycenter；证据链是 fixed-design and report-driven，包括 tuned AA baseline、reference-only ablation、contextual GA rows、held-out replication、harsh-loss stress、topology-ring 与 partial-FOV scenario-family checks；同时 cover letter 也主动说明 projection 是 active-output label-and-moment layer，不替代 Bernoulli existence update，并把 assignment ambiguity / lifecycle / covariance-aware recursive deployment 标成 limitations。

`check_submission_readiness.py` 同步新增 `cover letter paper-positioning markers` gate，防止后续元数据替换时把 cover letter 退化成只含模板声明的普通投稿信。该改动不改变正文科学 claim，不改变实验数值，只提升实际投稿包的编辑部入口质量。

## 22:19 Checkpoint

本轮继续按 algorithmic-paper 的预投稿风险检查推进 Discussion。当前证据链已经比较完整，剩余论文风险更多来自审稿人会问“什么时候会失败，以及递归部署要如何避免错误合并”。因此 `main.tex` 的 Discussion/Limitations 新增了 assignment ambiguity failure-mode 段: 当目标接近、邻域 cardinality 没有多数、或局部 posterior 明显偏置时，medoid reference 可能选择 plausible but wrong correspondence；递归部署应由 assignment-margin、covariance、track-age 或 reliability evidence 做 projection gate，并在 correspondence 不可信时回退到 reference-only 或 upstream AA output。

这不是新增方法 claim，也不是对当前数据调参，而是把方法层面的泛化边界写清楚。`check_submission_readiness.py` 同步新增 `projection failure-mode boundary wording` gate，用于防止后续压缩 Discussion 时删除这块审稿风险说明。

## 22:06 Checkpoint

本轮继续按预投稿审稿视角检查 Results 的机制解释。Paired reduction table (`tab:paired`) 中 reference-only 与 full 在 `Card. dispersion` 和 `CardErr` 上给出相同 reduction，这不是需要隐藏的现象，而是机制分解的一部分: 两者共享同一个 reference-label projection，因此 cardinality/label-set 指标应被解释为 label-set canonicalization 的收益；full 相对 reference-only 的额外 RMSE/localization separation 才是 matched moment barycenter 的证据。`main.tex` 的 paired-results 段已加入这一解释，避免审稿人把 cardinality 改善误读成 posterior barycenter 的独立贡献，或反过来质疑 reference-only/full tie。

这次修改不改变表格数值、不新增实验、不改变 conclusion，只把已有 ablation 的机制归因写得更可审。随后根据 PDF 渲染检查，将 runtime 成本段压缩为同一逻辑的短段，减少第 7 页底部孤句。

## 21:58 Checkpoint

本轮按 `nature-reviewer` 的预投稿审稿视角和 `nature-polishing` 的 algorithmic-paper/Introduction 规则复核了第一页叙事。主要风险不是实验证据不足，而是 Intro 的第一屏需要更快回答 TAES 读者的 `relevance -> novelty -> trust` 路径。因此已把引言前四段收紧为: networked surveillance/cooperative sensing 中 local track 必须可比，label set 是 fusion interface 而非内部 bookkeeping；AA/KLA 标量权重解决 probability-mass allocation，但不生成 Bernoulli component correspondence；本文问题被明确限定为 target-wise scalar weights 已正确进入 existence/spatial consumers 后，是否能用 graph-local projection 构造 missing component map；方法边界也提前写清楚为 output-space projection，不读取 global label set，也不替代 upstream Bernoulli existence update。

这次修改不新增实验、不改变主文数值、不扩展 claim，只增强 first-page problem framing 和 projection boundary。下一步重建时应重点检查第 1 页是否因 Intro 增长造成摘要/引言版式压缩，以及 `first-page narrative markers`、`implementation-alignment wording` 和 `paper-facing wording hygiene` 是否仍通过。

## 21:38 Checkpoint

本轮继续按期刊稿件第一屏读者路径做窄幅 polish。摘要末句已从 `base-seed-11` 这类内部运行标识改成 `held-out 50-trial replication` 和 `full-versus-reference separation`，保留 6.64% vs 0.82% 的机制分离数值，同时把 fixed-parameter boundary checks 写成 harsher packet loss、ring topology、partial field of view 三类泛化探针。`check_submission_readiness.py` 的 first-page narrative markers 也同步为检查论证功能，而不是强制要求摘要暴露内部 seed 标签。第一页 PDF 已渲染检查，标题、摘要、Index Terms、metadata placeholders 和引言首段均未出现重叠或坏版。

## 21:30 Checkpoint

当前分支仍为 `codex/aa-target-wise-fix`。本轮使用本地安装的 `nature-polishing` 期刊式 discussion guidance 做了一次很窄的正文 polish: 不新增 claim、不新增实验、不改主文手写数值，而是把 `extract_stress_evidence.py` 和 `extract_scenario_family_evidence.py` 生成的 Discussion 证据摘要从“报告式罗列”改成 claim/evidence/boundary 更清楚的解释性句子。stress 句现在明确是同一固定参数下的 packet-loss severity check；scenario-family 句现在明确 topology-ring 与 partial-FOV 是 topology / sensing-geometry probes，并保留 base seed、topology、FOV、full-vs-reference RMSE separation 等可追溯信息。

`./build.sh` 已重新生成 `main.pdf`、`generated/stress_harsh_summary_sentence.tex`、`generated/scenario_family_summary_sentence.tex`、PDF visual QA、readiness report 和 source bundle。`generated/SUBMISSION_READINESS_REPORT.md` 仍显示 `content_ready_metadata_pending`，metadata allowance 后没有 blocking gate；source bundle 的当前 SHA-256 以 `generated/SUBMISSION_BUNDLE_MANIFEST.md` 为准，避免在被打包的审计文档中记录自引用 hash。第 8 页 Discussion/Conclusion 渲染图已人工检查，未见文字重叠、断裂或空白页问题。

## 21:14 Checkpoint

当前分支为 `codex/aa-target-wise-fix`，上一已推送 checkpoint 为 `776f951 Synchronize TAES package evidence status`。本轮稿件 polish 把 implementation outline 从 4 步扩展为 5 步，在算法框和 caption 中显式写明 existence pass-through: projection consumes graph-neighborhood active outputs, does not read a global label set, does not re-estimate Bernoulli existence probabilities, and rewrites labels/moments only. `check_submission_readiness.py` 也新增了对应源级 markers，防止后续压缩 method section 时误把 projection 写成 existence 更新器。

partial-FOV fixed-parameter 35 deg N50 仍是当前 scenario-family 证据链的一部分。该 N50 报告显示 full operator 相对 fixed spatial-KLA AA baseline 的 network OSPA 降低 `43.25%`、local disagreement 降低 `85.49%`、RMSE 降低 `6.50%`，reference-only RMSE 只降低 `2.13%`；该结果支持“matched posterior barycenter 而不是 label copying 单独带来空间收益”的机制边界，同时保留 partial-FOV 下 local E-OSPA/CardErr 改善幅度较窄的边界信息。

partial-FOV 固定参数 N50 场景族升级已经完成，配置报告为 `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED41_20260624_182341.md`，`generated/SCENARIO_FAMILY_MANIFEST.md` 已把 topology-ring 与 partial-FOV 都标记为 `paper_grade`，smoke-tier scenario 数为 `0`。下一步不应围绕 partial-FOV 搜索阈值、barycenter 权重或 label rule；如果继续补实证，优先考虑真正泛化维度，例如 maneuvering target、covariance consistency、recursive-online fusion，或更大规模网络/目标数。

## Claim-to-Evidence Matrix

| Paper-facing claim | Manuscript location | Primary evidence | Verification level | Remaining gate |
| --- | --- | --- | --- | --- |
| AA/KLA scalar weights decide fusion mass/trust but do not solve Bernoulli component correspondence across local LMB posteriors. | Introduction, Problem Formulation, Structural Properties | AA fusion literature synthesis; `Weighting is not matching` label-swap counterexample in `main.tex` | Conceptual, citation-backed, and formalized by a minimal counterexample | Final language pass should keep this as the central motivation and avoid overclaiming all AA failures. |
| The proposed operator first canonicalizes labels by assignment and then fuses matched posterior moments. | Method, Fig. 1, compact algorithm box | `scripts/render_figures.py`; `generated/method_pipeline.tex`; algorithm step box in `main.tex`; `check_submission_readiness.py` method algorithm-box markers | Source-backed, machine-checked, and rendered in PDF | Final PDF visual check after every figure/table edit. |
| The proposed layer is a label-and-moment projection on active output tracks, not a replacement for the AA Bernoulli existence consumer. | Problem Formulation, Matched Moment Barycenter, Structural Properties, Experimental Setup | `multisensorLmb/applyCrossLocalLabelConsensusProjection.m`; `multisensorLmb/runDistributedLmbFilter.m`; revised `main.tex` wording | Code-aligned and manuscript-explicit | Keep existence-branch claims tied to upstream AA convex weighting; do not imply the projection estimates new existence probabilities. |
| The operator is graph-local in the neighborhood version and does not require global label-set access. | Method, Graph locality paragraph | `RUN/AA/runAaBalancedCardinalityValidation.m`; neighborhood N50 report | Code path and report-backed | Keep centralized upper-bound language out of the main claim. |
| Under stable matching and connected repeated neighborhood averaging, local moments converge to the centralized equal-weight barycenter. | Structural Properties | Analysis-scope assumption, stable-matching consensus proposition in `main.tex`, and the theory-to-experiment bridge paragraph | Theory now states active-track conditioning, fixed-neighborhood scope, and excluded lifecycle/crossing/cardinality cases before giving the consensus-limit result | Do not present as a finite-round guarantee; keep recursive lifecycle handling as a limitation. |
| Under fixed correspondence, the moment barycenter is the least-squares representative of a matched group and removes within-group mean disagreement. | Structural Properties | `Moment projection` proposition in `main.tex` | Algebraic identity plus first-two-moment mixture matching | Keep this as a fixed-assignment property; it does not prove that assignment is always correct. |
| Reference-only and full projections have identical cardinality-only metrics because both arms share the median-cardinality reference-control path. | Structural Properties, Results | `Cardinality-equivalent control` proposition in `main.tex`; identical Card. disp./CardErr effects in generated paired tables | Structural argument plus report-driven equality under paired trials | Use Card. disp. and CardErr as evidence for shared reference-cardinality projection, not for moment-barycenter localization. |
| Matched posterior barycenters, not label copying alone, drive the spatial tracking gain. | Results (`tab:paired`, `fig:n50`, `tab:heldout`) and Discussion | Main N50 full-vs-reference-only ablation; held-out baseSeed=11 N50 RMSE separation; harsh packet-loss baseSeed=21 N50 stress check; topology-ring baseSeed=31 N50 and partial-FOV baseSeed=41 N50 scenario-family checks; paired RMSE reductions/wins/sign-test p-values in generated held-out/stress/scenario fragments | Report-driven fragments plus independent local-metric verifier, held-out robustness gate, optional stress-evidence gate, and scenario-family source-hash gate | Maneuver validation, covariance-consistency validation, and recursive-online implementation remain useful broader-risk reductions. |
| Neighborhood label-barycenter improves local E-OSPA/RMSE/CardErr relative to the fixed spatial-KLA AA baseline in the N50 validation. | Results (`tab:n50`, `tab:paired`, `fig:n50`), Discussion stress/scenario sentences | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260623_232622.md`; `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED21_20260624_130730.md`; `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED31_20260624_162439.md`; `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED41_20260624_182341.md`; `generated/n50_evidence.json`; `generated/stress_harsh_evidence.json`; `generated/scenario_family_evidence.json`; `generated/N50_VERIFICATION_REPORT.md` | Local metrics independently recomputed for the main N50; harsh stress, topology-ring, and partial-FOV metrics are parsed through optional evidence gates and report-driven generated sentences | Current Discussion states that harsh stress broadens packet-loss severity, topology-ring broadens sparse-topology coverage, and partial-FOV broadens narrower sensing-geometry coverage, while still preserving the formation-family boundary. Maneuvering-target and covariance-consistency studies remain broader-risk items. |
| Neighborhood label-barycenter is lower than the two tracked GA reference rows on the six reported N50 disagreement/tracking metrics. | Results, contextual reference table | `RUN/GA/GA_TIERED_LINK_ABLATION_N50_SEED1_20260621_183039.md`; `generated/REFERENCE_BASELINE_MANIFEST.md` | Report-driven contextual comparison | Keep phrasing as contextual reference rows, not paired GA-vs-AA significance evidence. |
| Network disagreement and runtime numbers are reproducible from raw per-trial artifacts. | Results, reproducibility notes | `verify_n50_evidence.py`; `N50_VERIFICATION_REPORT.md` | Independently recomputed | None for network/runtime; keep source hash in manifest. |
| The main mechanism is not a base-seed-1 accident. | Results optional held-out fragment | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED11_20260624_094913.md`; `generated/HELDOUT_N50_MANIFEST.md`; `generated/heldout_n50_evidence.json`; `generated/heldout_n50_section.tex` | Strict held-out gate checks baseSeed=11, N50, trial seeds 12..61, three-arm metric coverage, paired evidence, and RMSE mechanism separation; the rendered table now shows reductions, absolute paired CI, wins, and sign-test p values | Keep the held-out table in the main paper by default; use it as supplement material only if a formal supplement is selected. |
| Runtime overhead is mainly due to repeated assignment/moment-barycenter operations, not hidden global communication. | Runtime paragraph, complexity paragraph | Runtime log and graph-locality method text | Independently recomputed runtime | Add a short scaling note if broader N or topology experiments are added. |
| Absolute runtime seconds are prototype-specific; the paper-facing runtime comparison is the relative cost within the paired Octave validation. | Experimental Setup, runtime paragraph | Local environment check: GNU Octave 11.1.0 on Apple M4, 16 GB memory | Manuscript-explicit and environment-backed | Keep absolute runtime claims modest; do not present them as hardware-independent benchmarks. |
| Each experiment arm isolates a specific mechanism rather than serving as an unconstrained tuning comparison. | Experimental Setup | Claim-to-arm mapping in `main.tex`; fixed-design statement that $H=3$, the existence threshold, projection cutoffs, barycenter weights, and trial-specific label rules are not searched per scenario | Manuscript-explicit and report-driven | Maintain this mapping when adding held-out or recursive-online experiments; held-out base-seed runs should remain robustness checks, not tuning sources. |

## TAES Compliance Gates

| Gate | Current state | Required before submission |
| --- | --- | --- |
| Official template | `IEEEtaes.cls` and `IEEEtaes.bst` are used by `main.tex`; `./build.sh` compiles the manuscript. | Rebuild after final edits; render pages and inspect figures/tables. |
| Template and author-guideline archive | `generated/SUBMISSION_READINESS_REPORT.md` now checks the local TAES requirements document, official template zip, and regular-paper template source as required artifacts. | Recheck online TAES/AESS pages immediately before final submission. |
| TAES requirements live-source refresh | `docs/TAES_SUBMISSION_REQUIREMENTS_CN.md` now records a 2026-06-24 live official-page refresh and no longer contains the stale local-TeX caveat; the readiness checker verifies portal/type/page-charge/format/AI/ORCID/preprint/technical-area markers. | Repeat this live-source refresh immediately before actual portal submission. |
| Clean source bundle | `./build.sh` now writes `tmp/submission_bundle/taes_label_barycenter_submission_source.zip` and `generated/SUBMISSION_BUNDLE_MANIFEST.md`; the zip includes `build.sh` plus the manuscript evidence/render/readiness scripts, and has been extracted and compiled with Tectonic. In a source-bundle directory without raw `RUN/` reports, `build.sh` falls back to the bundled `generated/` fragments. | Regenerate after final evidence or metadata edits; submit the current PDF plus a clean source bundle according to the portal instructions. |
| Submission package index | `SUBMISSION_PACKAGE_INDEX.md` now maps final portal uploads, internal QA artifacts, metadata placeholders, and the final rebuild/source-bundle verification sequence; the readiness checker requires these markers and the source bundle includes the index. | Keep this index synchronized after any final-upload, supplement, or metadata workflow change. |
| Manuscript type | Regular Paper is the selected target. | Submit as `Regular Paper`; technical area `Target Tracking and Multi-Sensor Systems`. |
| Page budget | Current manuscript is within the pre-overlength range; the readiness checker now treats 10 or more TAES-template pages as a warning because Regular Paper overlength charges start at 10 printed pages. | Keep estimated TAES pages below 10, or explicitly accept overlength charges before submission. |
| Title/abstract/keywords | The readiness checker now verifies that the title/abstract avoid `new`/`novel`, the abstract is a single paragraph without citation/footnote/display equation, and keywords are alphabetized. Current abstract front-loads the held-out full-versus-reference RMSE separation without exposing internal seed labels in the first-page prose. | Re-run `./build.sh` after final title/abstract edits. |
| First-page narrative | Abstract and introduction now state the residual correspondence failure more directly, specify the held-out 50-trial replication, and keep the proposed gain separated from scalar-weight routing. The readiness checker records explicit first-page story markers and paper-facing wording hygiene without requiring internal run identifiers in the abstract. | Keep the first page visually checked after any abstract or contribution edit. |
| Related-work positioning | Passed | `main.tex`; `check_submission_readiness.py` | Related Work now explicitly frames the method as a correspondence-map/output-projection layer orthogonal to AA/KLA weighting choices, rather than another density-pooling rule. The checker records these positioning markers so later compression does not erase the claim boundary. |
| Author metadata | Bracketed author/funding/repository placeholders, TAES front-matter placeholders, and cover-letter/portal checklist placeholders remain in submission-style prose. | Fill real author list, affiliations, receipt/revision dates if requested by the template stage, ORCID, funding, acknowledgments, corresponding author metadata, final issue/DOI fields, cover-letter signature/email, and preprint/conflict/reviewer decisions. |
| AI disclosure | Provisional Codex/OpenAI disclosure exists in Acknowledgment. | Recheck current IEEE/AESS wording and decide final disclosure scope. |
| Citations | Core bibliography has been source checked during draft construction; `verify_bibtex_dois.py` now resolves every BibTeX DOI through the DOI resolver, and the readiness checker verifies cited keys, DOI fields, uncited BibTeX entries, and same-hash DOI resolver coverage. | Final bibliography scan for malformed entries and unsupported claims after any citation edit. |
| Preprint/reuse | No final decision recorded. | Decide whether to post preprint and prepare IEEE-compliant preprint notice if needed. |
| Submission files | PDF builds locally; source bundle exists in manuscript directory. | Prepare clean source zip, PDF, cover metadata, and record no separate supplement upload unless a formal supplement is selected. |
| Machine-checkable readiness | `generated/SUBMISSION_READINESS_REPORT.md` is written by `./build.sh`; current portal status is `draft_with_pending_gates`, while content status ignores metadata placeholders by policy. | Close the manuscript and cover-letter metadata placeholders before actual portal submission; content review can proceed under the project convention that these placeholders are allowed. |
| Cover letter and portal metadata | `COVER_LETTER_AND_METADATA_DRAFT.md` now provides an editable cover-letter draft and portal metadata checklist; readiness checker verifies that its title, running head, journal, Regular Paper type, technical area, simulated-data statement, and AI disclosure remain synchronized with `main.tex`. | Replace author/funding/repository/preprint/conflict placeholders and recheck the current TAES portal wording before submission. |
| Final package handoff | `SUBMISSION_PACKAGE_INDEX.md` now separates final upload files from internal QA artifacts and records the rebuild order from placeholder replacement through bundled-source compilation. | Use it as the final submission checklist after metadata replacement. |

## Evidence Gates

| Gate | Status | Evidence | Next action |
| --- | --- | --- | --- |
| Report-driven tables/figure fragments | Passed | `extract_n50_evidence.py`; `generated/N50_EVIDENCE_MANIFEST.md` | Keep generated fragments read-only by convention. |
| Report-driven GA reference rows | Passed | `extract_reference_baselines.py`; `generated/REFERENCE_BASELINE_MANIFEST.md` | Maintain the contextual-comparison caveat in the manuscript. |
| Independent network disagreement verifier | Passed | `verify_n50_evidence.py`; `generated/N50_VERIFICATION_REPORT.md` | Maintain hash check against source report. |
| Independent runtime verifier | Passed | `verify_n50_evidence.py`; trial log parsing | Maintain relative-cost check after source report swap. |
| Independent local metric verifier | Passed | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260623_232622.md`; `RUN/AA/AA_TAES_N50_LOCAL_VERIFIER_RERUN_20260623_232621.log`; `generated/N50_VERIFICATION_REPORT.md` | Maintain `evidence_sources.json` as the single report/log source manifest. |
| Source-package reproducibility ledger | Passed | `scripts/render_reproducibility_ledger.py`; `generated/REPRODUCIBILITY_LEDGER_MANIFEST.md`; `generated/reproducibility_ledger.json`; `generated/reproducibility_ledger_table.tex` | Keep as source-package/response-ready provenance rather than importing the table into the main PDF; the generated ledger now separates primary, held-out, harsh-stress, scenario-family, GA, and verifier roles. |
| BibTeX DOI resolver verification | Passed for current bibliography | `scripts/verify_bibtex_dois.py`; `generated/BIBTEX_DOI_VERIFICATION.md`; `generated/bibtex_doi_verification.json` | Re-run `./build.sh` after any bibliography edit; the gate requires all BibTeX entries to resolve and the generated report hash to match the current `references.bib`. |
| Held-out scenario evidence | Passed for baseSeed=11 N50 robustness | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED11_20260624_094913.md`; `generated/HELDOUT_SANITY_MANIFEST.md`; `generated/HELDOUT_N50_MANIFEST.md`; `generated/heldout_n50_evidence.json`; `generated/heldout_n50_section.tex` | Keep the held-out table/paragraph in the manuscript by default; use it as supplement material only if it is moved out of the main paper or explicitly requested. The readiness checker requires base seed 11, at least 50 trials, parsed trial seeds, generated manifest/LaTeX fragment, all three arms, all manuscript metrics, paired CI/wins/p-values, and a visible full-barycenter-vs-reference-only RMSE separation check. Current PDF table explicitly shows CI and sign-test p for the held-out RMSE separation. |
| Optional harsh packet-loss stress evidence | Passed for fixed-design baseSeed=21 N50 stress check | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED21_20260624_130730.md`; `generated/STRESS_HARSH_MANIFEST.md`; `generated/stress_harsh_evidence.json`; `generated/stress_harsh_section.tex`; `generated/stress_harsh_summary_sentence.tex` | Keep the concise generated sentence in Discussion and the full table as response-ready evidence. Current stress N50 gives full-method reductions of `79.57%` network OSPA, `17.06%` local E-OSPA, `7.98%` RMSE, and `24.33%` CardErr under `[0.2, 0.35, 0.5, 0.7] / [1, 3, 2, 2]` packet loss. |
| Scenario-family topology evidence | Passed for fixed-design topology-ring baseSeed=31 N50 check | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED31_20260624_162439.md`; `generated/SCENARIO_FAMILY_MANIFEST.md`; `generated/scenario_family_evidence.json`; `generated/scenario_family_section.tex`; `generated/scenario_family_summary_sentence.tex` | Keep the concise generated sentence in Discussion and the full table as response-ready evidence. Current topology-ring N50 gives full-method reductions of `47.50%` network OSPA, `21.53%` local E-OSPA, `13.12%` RMSE, and `24.03%` CardErr; reference-only RMSE reduction is `5.17%`. |
| Scenario-family partial-FOV evidence | Passed for fixed-design partial-FOV 35 deg baseSeed=41 N50 check | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED41_20260624_182341.md`; `generated/SCENARIO_FAMILY_MANIFEST.md`; `generated/scenario_family_evidence.json`; `generated/scenario_family_section.tex`; `generated/scenario_family_summary_sentence.tex` | Keep the concise generated sentence in Discussion and the full table as response-ready evidence. Current partial-FOV N50 gives full-method reductions of `43.25%` network OSPA, `85.49%` local disagreement, `6.50%` RMSE, and `6.89%` local E-OSPA; reference-only RMSE reduction is `2.13%`. |
| PDF visual QA | Passed for current checkpoint and now machine-recorded | `./build.sh`; `scripts/render_pdf_visual_qa.py`; `generated/PDF_VISUAL_QA_MANIFEST.md`; `generated/pdf_visual_qa.json`; ImageMagick-rendered checks of the title/abstract page, method page, result tables/figure page, held-out/runtime page, Discussion/Conclusion page, final reference page, all 9 pages, and `tmp/pdf_visual_qa/main_contact_sheet.png` | Re-render final `main.pdf` after every manuscript-affecting checkpoint; the readiness checker now records representative pages, full-page coverage, contact sheet status, and stale-output cleanup. |
| Submission readiness checker | Passed for all non-metadata blocking gates | `check_submission_readiness.py`; `generated/SUBMISSION_READINESS_REPORT.md`; `generated/submission_readiness.json` | Checker reports both `portal_status` and `content_status`: the portal status remains blocked by metadata placeholders, while the content status ignores only those placeholders and is now `content_ready_metadata_pending`. It records first-page narrative markers, paper-facing wording hygiene, stress/generalization boundary wording, cover-letter/portal metadata source synchronization, optional harsh-stress N50 parsing, scenario-family source-hash freshness/evidence tiering, and whether the generated reproducibility ledger covers primary AA, held-out AA, harsh-loss AA, scenario-family boundary, contextual GA, and independent-verifier roles. |
| Cardinality-equivalent ablation theory | Passed | `main.tex`; `check_submission_readiness.py`; `generated/SUBMISSION_READINESS_REPORT.md` | New gate passes after rebuild: the manuscript preserves the cardinality-equivalent control proposition and the Results wording that maps Card. disp./CardErr to shared reference-cardinality projection. |
| Source-bundle rebuild check | Passed | `generated/SUBMISSION_BUNDLE_MANIFEST.md`; extracted source-bundle `TAES_EVIDENCE_MODE=bundled ./build.sh` compile | Re-run after final manuscript-affecting edits. The readiness checker now also verifies required reproducibility scripts, the submission package index, PDF visual-QA script, source-bundle manifest hash freshness, and the bundled-fragment fallback build mode. |

## Immediate Execution Order

1. Replace author/funding/repository/corresponding-author/OA/preprint/conflict placeholders once the real submission metadata is available.
2. Rebuild `main.pdf` and re-render the title/abstract page, method pages, main-results page, held-out/evidence page, Discussion/Conclusion page, and final reference page after any evidence or metadata edit.
3. Keep the held-out N50 CI/wins/p-value table in the main paper by default; do not create a separate supplement unless the final portal form, editor, reviewers, or page-budget decision requires it.
4. Keep the harsh packet-loss, scenario-family, and reproducibility-ledger blocks response-ready/source-bundle-ready; if a formal supplement is selected, record the exact files in the cover-letter metadata draft and portal form.
5. Preserve the full-topology N50 result only as a fixed-parameter zero-disagreement equivalence boundary; do not use it as a gain claim or a tuning source.
6. Consider target-maneuver, covariance-consistency, and recursive-online validation as the next evidence-risk reduction steps beyond the current fixed-parameter harsh-loss, topology-ring, partial-FOV, and full-topology ceiling checks.

## 00:09 Checkpoint

当前分支仍为 `codex/aa-target-wise-fix`。本轮按 target-journal 预投稿标准把后续工作从 `ml-paper` 式会议叙事切换到更严格的 Nature-style writing / reviewer preflight 口径，但投稿格式、模板、metadata 和 page budget 仍严格按 IEEE TAES。当前稿件的非元数据 gates 已经全绿，因此下一步不再做针对当前数据的搜索式调参，而是补充一个固定参数的 topology ceiling evidence: `full-topology` N50 已用 `AA_SCENARIO_FAMILY=full-topology`、`AA_SCENARIO_TRIALS=50`、`AA_SCENARIO_BASE_SEED=51` 启动。

Run handoff:

- PID: `34014`
- PID file: `RUN/AA/AA_TAES_SCENARIO_full_topology_N50_BASESEED51_20260625_000936.pid`
- Log: `RUN/AA/AA_TAES_SCENARIO_full_topology_N50_BASESEED51_20260625_000936.log`
- Follow-up command: `tail -f RUN/AA/AA_TAES_SCENARIO_full_topology_N50_BASESEED51_20260625_000936.log`

这条 run 只用于区分 topology bottleneck 和方法本身的 assignment/barycenter behavior。结果完成前不得接入 `evidence_sources.json`、不得写入 generated scenario-family fragment、不得扩张正文 claim；结果完成后也按 no-search rule 解释，不能反向调整 `H`、threshold、projection cutoff、barycenter weights、label rules 或 packet-loss settings。

## 00:15 Checkpoint

当前分支仍为 `codex/aa-target-wise-fix`。本轮按 `nature-reviewer` 的预投稿审稿视角，对当前 `main.tex`、cover-letter draft、supplementary evidence map、generated readiness report 和 source-bundle状态做了 bounded reviewer-style preflight。结论不是新增稿件 claim，而是把三类可能审稿关注固化到 `REVIEWER_RISK_REGISTER.md`: technical soundness 关注 recursive validity、assignment ambiguity、covariance quality 和 target-maneuver 缺口；originality/significance 关注 correspondence-contract framing 是否清楚地区分于 AA/KLA weighting；readability/reuse 关注主文证据路径是否足够简洁、response-ready evidence 是否不会喧宾夺主。

`REVIEWER_RISK_REGISTER.md` 现在新增 `Reviewer-Style Preflight Synthesis`、三位 reviewer emphasis、`Cross-Review Synthesis` 和 `Risk / Unsupported Claims`。其中明确写入 `full-topology` ceiling run 完成前不得进入正文、generated fragment 或 cover-letter evidence。`check_submission_readiness.py` 同步扩展 reviewer-risk gate，要求这些 marker 保留。这个改动提升的是投稿前 response planning 和 claim-boundary discipline，不改变正文数值、不接入新实验、不扩张 evidence scope。

## 00:20 Checkpoint

当前分支仍为 `codex/aa-target-wise-fix`。本轮检查了当前 PDF visual QA contact sheet: title/abstract、method、main-results、held-out/runtime、discussion/conclusion 和 references 代表页没有明显重叠、表格溢出或大块空白；第 6/7 页较密，但仍在 9 页 TAES 预算内且机器 visual QA 通过。随后检查 `runAaBalancedCardinalityValidation.m` 的 scenario override 支持范围，确认当前代码可以直接支撑 topology/FOV/full-topology，但 maneuver/crossing、covariance-consistency 和 recursive-online 不是简单换 seed 或阈值能完成的 evidence。

新增 `docs/AA_NEXT_STAGE_GENERALIZATION_PROTOCOL_CN.md`，把下一阶段真正方法层面的泛化验证拆成三个候选: `maneuver-crossing-assignment`、`covariance-mismatch-reliability`、`recursive-guarded-projection`。每个候选都写明研究问题、固定改变量/不变量、代码门槛、解释规则和 no-search rule。该文档明确当前 TAES submission 不应等待这些候选全部完成；它们是 reviewer-risk reduction 或后续 method-extension 设计，不能替代当前已验证的 output-level active-track claim。

## 00:25 Checkpoint

当前分支仍为 `codex/aa-target-wise-fix`。本轮补齐 submission package 的自洽性说明: `README.md` 和 `SUBMISSION_PACKAGE_INDEX.md` 现在都明确 `docs/AA_NEXT_STAGE_GENERALIZATION_PROTOCOL_CN.md` 是 repository-level internal planning protocol，不是当前 manuscript evidence、不是 portal upload，也不进入 deterministic source bundle 的 evidence artifact list。这个边界很重要，因为 `REVIEWER_RISK_REGISTER.md` 和本审计文档会引用下一阶段 protocol，但当前提交包的证据链仍只以已完成并被 parser/readiness gate 验证的 artifacts 为准。

## 22:45 Checkpoint

当前分支仍为 `codex/aa-target-wise-fix`。本轮按期刊投稿包而不是会议稿件的口径，新增 `SUPPLEMENTARY_EVIDENCE_PACKAGE.md` 作为 optional supplement / reviewer-response evidence 的单一索引层，把 held-out N50、harsh packet-loss、topology/FOV scenario family 和 reproducibility ledger 的生成片段统一映射到用途、状态和解释边界。该文件明确这些材料来自固定参数后的 robustness / boundary checks，不构成针对场景的 threshold、barycenter weight 或 label rule 搜索，也不替代 maneuvering-target、covariance-consistency、recursive-online validation 等更广泛风险项。

`SUBMISSION_PACKAGE_INDEX.md` 和 `README.md` 已同步把该 evidence package 纳入投稿包说明；`create_submission_bundle.py` 已把它加入 source bundle；`check_submission_readiness.py` 新增并调用 supplementary evidence package gate，检查候选补充材料、response-ready evidence、boundary control、四个生成片段入口、非调参声明以及 generated fragment 不手改规则。下一步必须通过 `./build.sh`、readiness report、PDF visual QA 和 extracted source-bundle rebuild 后，才能把这个 checkpoint 视为已验证。

## 22:50 Checkpoint

当前分支仍为 `codex/aa-target-wise-fix`。本轮按 algorithmic-paper 的 Discussion 结构做了一个很窄但重要的稿件修订：`DISCUSSION AND LIMITATIONS` 不再直接从 limitation 开始，而是先解释 paired / held-out / boundary evidence 共同支持的机制读法。新的开场明确：在 target-wise AA weight routing 已固定的条件下，reference-only projection 主要解释 network disagreement 和 cardinality effect，而主要 RMSE separation 需要 matched moment barycenters；因此本文方法应被读作 complement to AA/KLA weighting 的 correspondence-and-projection layer，而不是 replacement for density pooling。

`check_submission_readiness.py` 已新增 `discussion interpretation markers` gate，要求 Discussion 保留 fixed weight routing、reference-only partial effect、matched barycenter spatial separation、complementary-to-AA/KLA 和 not-density-pooling-replacement 这些 marker。这个 gate 的目的不是增加结果，而是防止后续为了压页或润色而把最关键的 ablation interpretation 删除。

## 22:59 Checkpoint

当前分支仍为 `codex/aa-target-wise-fix`。本轮按期刊稿件术语一致性做了一个投稿质量补丁：摘要中补齐 labeled multi-Bernoulli (LMB)、arithmetic-average (AA)、Kullback--Leibler average (KLA)、optimal sub-pattern assignment (OSPA)、expected OSPA (E-OSPA)、root-mean-square error (RMSE) 和 field of view (FOV) 的 first-use 定义；正文中补齐 LMB、FOV、OSPA、E-OSPA、RMSE 等首次使用定义，避免后续结果表和 generated Discussion 句里直接出现未定义缩写。

`check_submission_readiness.py` 已新增 `abbreviation first-use definitions` gate，分别检查 abstract 和 main text 的关键缩写定义。这个 gate 面向投稿 polish，不改变实验证据或主张，只减少 IEEE/TAES 格式审查和审稿人快速扫读时的术语粗糙风险。

## 23:05 Checkpoint

当前分支仍为 `codex/aa-target-wise-fix`。本轮继续按 algorithmic-paper 的 conclusion 规则做窄幅收束：结论末句不再写成 `can support future recursive LMB designs` 这种偏承诺式表述，而是改为 `output-level correspondence module`，并明确 `recursive LMB use still requires lifecycle and consistency guards`。这个改动与正文 Discussion 里的边界一致，不新增实验主张，同时避免结论尾句孤立跨页。

`check_submission_readiness.py` 已新增 `conclusion boundary wording` gate，要求结论同时保留 output-level scope 和 recursive-safeguard requirement。该 gate 的目的，是防止最终润色或压缩结论时把 evidence boundary 写宽，尤其是把当前输出层 projection 误写成已经验证的 recursive LMB update。

## 23:16 Checkpoint

当前分支仍为 `codex/aa-target-wise-fix`。本轮按 reviewer-preflight 的口径新增 `REVIEWER_RISK_REGISTER.md`，把可能的审稿质疑整理成 concern -> manuscript answer -> evidence/artifact -> residual boundary 的内部索引。它覆盖 not-another-AA/KLA-weighting-rule、label-copying-only、recursive LMB validity、equal moment barycenter、generality、runtime overhead 和 reproducibility integrity 等高风险点，但明确不是新的 claim source，也不能作为 data source 引用。

`README.md` 和 `SUBMISSION_PACKAGE_INDEX.md` 已同步把它纳入投稿包说明；`create_submission_bundle.py` 已把它加入 source bundle；`check_submission_readiness.py` 新增 `reviewer risk register` gate，检查表头、核心风险项和 do-not-cite boundary。该文件的作用是让后续回复审稿或补充材料决策从已验证证据出发，而不是临时扩张主张。

## 23:22 Checkpoint

当前分支仍为 `codex/aa-target-wise-fix`。本轮把 `Moment projection` 命题升级为 `Moment-space projection`，将 matched moment barycenter 明确写成 first-two-moment coordinate 上的 least-squares projection：先固定 assignment/correspondence，再对 $\mu_i$ 和 raw second moment $M_i=\Sigma_i+\mu_i\mu_i^T$ 的坐标取均值，并从 $\bar{M}-\bar{\mu}\bar{\mu}^T$ 恢复 covariance。该表述同时说明 $\bar{\Sigma}$ 的 positive semidefinite 来源和 within-group mean disagreement 被移除的代数身份。

`main.tex` 中的 Method 也新增一句边界说明：这是 moment-matching projection，不是 covariance-consistency guarantee。`check_submission_readiness.py` 新增 `moment-space projection markers` gate，要求保留 first-two-moment coordinates、moment-matching/covariance-consistency boundary、Moment-space projection 命题、unique least-squares minimizer、PSD 和 matched Gaussian mixture 等 marker。`REVIEWER_RISK_REGISTER.md` 同步把 equal moment barycenter 风险项改为 theory-backed but still covariance/reliability-limited 的表述。

## 23:30 Checkpoint

当前分支仍为 `codex/aa-target-wise-fix`。本轮按 Intro/Related Work 的投稿故事线做窄幅增强：第一段把 label set 明确称为 `correspondence contract`，强调即使 fusion rule 数学上有效，只要 contract 在传感器间不一致，仍可能组合错误 Bernoulli components；第二段把 AA/KLA 等规则定位为在 component correspondence 已给定之后分配 probability mass；第三段把本文设计问题写成 `correspondence-contract construction problem`，不是另一个 scalar-weight search。

Related Work 收尾同步压实了区别：本文不是 density-pooling rule，而是 output-space projection；这个区别也决定了评价设计，因此同时报告 network disagreement 和 truth-referenced tracking error。`check_submission_readiness.py` 新增 `correspondence-contract story markers` gate，防止后续润色时把核心故事退回到泛泛的 weight-fusion framing。

## 23:41 Checkpoint

当前分支仍为 `codex/aa-target-wise-fix`。本轮按 Results section 的读者路径做窄幅增强：`RESULTS` 不再直接从表格开始，而是先用一段 compact evidence chain 说明证据顺序。Table II 承担 primary paired network/local gain，Table III 提供 matched-seed GA context，Table IV 和 Fig. 3 分离 reference-only label effect 与 matched-barycenter effect，Table V 复验 held-out base-seed mechanism separation。

`check_submission_readiness.py` 新增 `results evidence-spine markers` gate，用于防止后续压页或润色时删掉 Results 的读者入口。该改动不改变任何表格数值、不新增实验主张，也不把 GA reference rows 改写成 paired significance evidence；它只是让 TAES 审稿人更快看清 primary result、context、mechanism ablation 和 robustness replication 的顺序。

## 23:58 Checkpoint

当前分支仍为 `codex/aa-target-wise-fix`。本轮按预投稿 reviewer-readability 视角重画 Fig. 1 的生成源：`method_pipeline.tex` 不再把 residual、reference、assignment、barycenter、output 和 iteration 都塞进一排小字框，而是改成更清楚的三层结构：上层说明 scalar AA weights choose probability mass, not component correspondence；中层给出 Reference、Assignment、Barycenter 三步；下层保留 Existence pass-through、Moment projection、H-round graph-local iteration 和 no global label dictionary 的边界。

`scripts/render_figures.py` 同步更新 SVG 和 LaTeX fragment 生成逻辑；`check_submission_readiness.py` 新增 `method figure mechanism markers` gate，检查 Fig. 1 的 mass-vs-correspondence、three-step projection、existence-pass-through 和 no-global-label markers。该改动不改变方法 claim 或实验结果，只降低 Fig. 1 对审稿人的阅读成本，并让图的机制信号更接近最终投稿质量。
