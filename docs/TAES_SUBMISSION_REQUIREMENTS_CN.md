# IEEE TAES 投稿模板与要求整理

日期: 2026-06-23

## Question

面向 `IEEE Transactions on Aerospace and Electronic Systems (TAES)`，下载并归档官方模板、作者须知和相关政策，并整理当前 AA label-barycenter paper 的投稿准备要求。

## Scope

包含:

- TAES 官方 LaTeX 模板 zip、regular paper 模板、correspondence 模板和 letter 模板归档。
- TAES 作者须知、TAES 主页、technical area descriptions、preprint policy、AI-generated content policy、IEEE article template page 和 IEEE Editorial Style Manual 的本地快照或 PDF。
- 以当前 `Neighborhood Label-Barycenter LMB Fusion for Distributed Multi-Target Tracking under Unreliable Communication` 为目标稿件的 TAES-specific 准备清单。

排除:

- 不替代投稿系统中的最终表单检查；提交前需要再次核对在线页面。
- 不处理版权表单或 OA 付款；这些只在接受后或作者选择 OA 时发生。
- 不声明模板编译已通过，因为当前本机没有 `pdflatex`/`latexmk`。

## Risk Tier

L2。投稿要求影响正式 submission 包和费用估计；本文档基于 2026-06-23 官方页面快照整理，提交前仍需人工复核。

## Downloaded Assets

已下载到仓库:

| Asset | Local path | Source |
| --- | --- | --- |
| TAES template zip | `docs/paper/taes/TAES_Template.zip` | `https://confcats-web-assets.s3.amazonaws.com/ieeeaess/documents/TAES+Template.zip` |
| Regular/original research template | `docs/paper/taes/template_regular/IEEE_TAES_orig-research/TAES_template.tex` | TAES template zip |
| Regular/original research sample PDF | `docs/paper/taes/template_regular/IEEE_TAES_orig-research/TAES_template.pdf` | TAES template zip |
| Correspondence template | `docs/paper/taes/template_correspondence/IEEE_TAES_correspondence/TAES_Corresp_template.tex` | TAES template zip |
| Correspondence sample PDF | `docs/paper/taes/template_correspondence/IEEE_TAES_correspondence/TAES_Corresp_template.pdf` | TAES template zip |
| Letter template | `docs/paper/taes/template_letter/IEEE_TAES_letter/TAES_Letter_template.tex` | TAES template zip |
| IEEE TAES class/style | `docs/paper/taes/template_regular/IEEE_TAES_orig-research/IEEEtaes.cls`, `IEEEtaes.bst` | TAES template zip |
| IEEE Editorial Style Manual | `docs/paper/taes/IEEE_Editorial_Style_Manual_for_Authors.pdf` | IEEE Author Center |
| TAES author-info snapshot | `docs/paper/taes/snapshots/taes_author_information_2026-06-23.html` | TAES Information for Authors |
| TAES home snapshot | `docs/paper/taes/snapshots/taes_home_2026-06-23.html` | TAES home |
| Technical areas snapshot | `docs/paper/taes/snapshots/taes_technical_area_descriptions_2026-06-23.html` | TAES technical area descriptions |
| Preprint policy snapshot | `docs/paper/taes/snapshots/taes_preprint_policy_2026-06-23.html` | TAES preprint policy |
| AI content policy snapshot | `docs/paper/taes/snapshots/ieee_aess_ai_generated_content_2026-06-23.html` | AESS AI-generated content page |
| IEEE templates page snapshot | `docs/paper/taes/snapshots/ieee_article_templates_2026-06-23.html` | IEEE Author Center |

Integrity hashes:

```text
TAES_Template.zip
6579ae30c43eaf695445e802603970c0c770269e3e88b3c28bee2cf99af8803e

IEEE_Editorial_Style_Manual_for_Authors.pdf
4891f9e7ae600ddc57628d46b242f5319b8924789841aac33b847001c2bedaba
```

## Recommendation

当前 AA label-barycenter 稿件应按 **Regular Paper** 准备，不按 Correspondence Item 准备。

理由:

- 我们需要完整讲清 distributed LMB tracking 问题、label split failure mode、neighborhood label-barycenter operator、理论性质、N50 validation、reference-only ablation、GA/AA references 和 runtime。
- TAES 对 Regular Paper 的定义是对一个问题区域的 well-rounded treatment；这更匹配当前稿件。
- Correspondence Item 更适合只表达一两个 concise points，当前工作如果压成 correspondence 会牺牲方法和实验完整性。

建议 technical area 选择: **Target Tracking and Multi-Sensor Systems**。

理由: TAES technical area description 明确覆盖 multi-target tracking、data fusion、decentralized/distributed detection 和 decentralized/distributed estimation。我们的稿件主 thrust 是 tracking/fusion，而不是 sensor resource management，因此比 `Networked Sensor Systems` 更合适。

## Claims

| ID | Claim | Confidence | Evidence IDs | Caveats |
| --- | --- | --- | --- | --- |
| C1 | TAES 新投稿应走 Atypon/REX submission portal，Regular Paper 选择 `Regular Paper` manuscript type。 | High | E1, E2 | 旧稿件或 2025-03-04 前 submission 可能在旧系统；新稿走 Atypon。 |
| C2 | Regular Paper 没有硬性 manuscript page limit，但接受后超过 10 printed pages 会产生每页 200 USD overlength charge。 | High | E1 | printed pages 以 IEEE production final proof 为准，模板页数只是估算。 |
| C3 | 投稿 PDF 必须使用两栏、单倍行距、10 pt 字号、指定 margins/columns；TAES 模板是最准确页数估算方式。 | High | E1, E3 | 本机未安装 TeX，尚未本地编译验证。 |
| C4 | TAES 是 single-anonymous peer review，至少两名 independent reviewers；会在 review 前和接受前进行 plagiarism screening。 | High | E2 | 不是 double-blind，投稿稿件无需匿名化。 |
| C5 | TAES Letters 已停止；当前只能在 Regular Paper 和 Correspondence Item 之间选。 | High | E2 | 模板 zip 仍含 letter 模板，但不要用于新投稿。 |
| C6 | IEEE/TAES 要求所有作者有 ORCID；accepted manuscripts 需要电子版权表单。 | High | E2, E4 | ORCID 在 submission/proof 环节都可能需要。 |
| C7 | AI-generated content 若进入论文文本、图、代码等，必须在 acknowledgments disclosure；failure to disclose 可能直接拒稿。 | High | E5 | 纯 grammar/editing 可不强制披露但建议披露；我们应保守披露 Codex/AI writing assistance。 |
| C8 | TAES 对 preprint 中立；TechRxiv/arXiv preprint 本身不影响 editorial decision。 | High | E6 | 接受后和发表后版本替换/版权声明要按 IEEE policy 做。 |

## Submission Requirements

### 1. 稿件类型

TAES 当前相关稿件类型:

- `Regular Paper`: 对问题区域做完整、well-rounded treatment。我们的 AA label-barycenter paper 应选这个。
- `Correspondence Item`: 只清晰表达一两个核心点，更短、更少铺垫；不是质量更低，而是 scope 更窄。
- `Letter`: TAES Letters category 已在 2022-12-31 停止，不作为新投稿选择。

### 2. 投稿系统

新稿件入口:

```text
https://ieee.atyponrex.com/journal/taes
```

投稿时需要选择:

- manuscript type: `Regular Paper`
- technical area: 推荐 `Target Tracking and Multi-Sensor Systems`

### 3. 格式要求

TAES author-info 页面要求 Regular Papers 和 Correspondence Items 投稿时使用一个格式:

- two-column
- single-spaced
- 10-point font
- top/bottom margins: 1 in / 25 mm
- left/right margins: 0.7 in / 18 mm
- column width: 3.45 in / 88 mm
- column spacing: 0.2 in / 5 mm
- submitted manuscript file: PDF

当前已下载的 regular paper 模板:

```text
docs/paper/taes/template_regular/IEEE_TAES_orig-research/TAES_template.tex
```

### 4. 页数与费用

TAES 没有硬性 manuscript page limit，但:

- Regular Paper 接受后超过 10 printed pages，每超一页收 200 USD。
- Correspondence Item 接受后超过 6 printed pages，每超一页收 200 USD。
- Open Access fee 不替代也不豁免 overlength page charges。
- printed pages 以 IEEE production final proof 为准；TAES two-column template 只是最准确估计方式。

对我们稿件的实际建议:

- 正文目标控制在 10 个 TAES template pages 内，最多不要明显超过 12 页。
- 如果超过 10 页，要提前决定是否接受 page charges，或把 centralized upper-bound、额外 ablation、长证明移到 supplementary/appendix。

### 5. Open Access 与版权

- TAES 是 hybrid journal，可选择 traditional publication model 或 Open Access model。
- TAES home 页面显示 IEEE hybrid OA APC 为 2800 USD。
- IEEE copyright form 只在 accepted manuscript 阶段需要；对应链接会发给 corresponding author。
- 选择 OA 不会免除 overlength page charges。

### 6. ORCID

IEEE journals 要求所有作者有 ORCID。提交或 proof review 时会需要 registered ORCID。

### 7. 审稿流程

TAES review 过程:

- Senior Editor 按 technical area 分配 submission。
- Associate Editor 管理 review。
- 三层 prescreening: Editor-in-Chief、Senior Editor、Associate Editor。
- 通过 prescreening 后进入 peer review。
- single-anonymous review: reviewers 匿名，authors 不匿名。
- 至少两名 independent reviewers。
- plagiarism screening 在 review 前和 acceptance 前进行。
- 决策类型: Reject, Major Revision, Minor Revision, Accept。
- major revision 通常会送回一个或多个原 reviewers；minor revision 通常由 Associate Editor 评估。

### 8. Revision 要求

Major/Minor Revision 时:

- 需要提交 revised two-column manuscript。
- 需要单独提交 `Response to Reviewers` 文件。
- 对 editors/reviewers 的所有问题逐点回应。
- 所有 text changes 要 highlighted 或 colored。

### 9. 原创性与重复投稿

TAES 只接收 scope 内原创材料。不得:

- plagiarism；
- 未引用/未明确界定地复用他人或自己的已发表材料；
- multiple submission；
- 不披露曾被其他 journal reviewed and rejected 的相近稿件。

如果稿件曾被任何 journal reviewed and rejected，投稿时必须披露，并提供此前 correspondence；需要说明为何重新提交。

### 10. Preprint

TAES 对 TechRxiv/arXiv 等 preprint 的态度是 neither encourages nor discourages；posting preprint 本身不影响 editorial decision。

注意:

- Accepted version 可以按 IEEE policy 放到个人/机构服务器，但需显著 IEEE copyright notice。
- Final published IEEE version 不应自行上传。
- 发表后需用 DOI 或 accepted version 替换此前电子版本。

### 11. AI-Generated Content

IEEE/TAES 要求:

- 如果文章中使用 AI-generated content，包括 text, figures, images, code，必须在 acknowledgments 披露。
- 披露要包含 AI system 名称、使用在哪些 sections、以及使用程度说明。
- 不披露 AI-generated content 是 policy violation，可能导致 direct rejection 和 further disciplinary action。
- 仅用于 editing/grammar enhancement 通常不强制披露，但建议披露。

对当前项目的建议:

```text
Acknowledgment 里应加入透明说明: Codex/OpenAI tools were used to assist with literature organization, code navigation, experiment-log summarization, and drafting support. All technical claims, derivations, experiments, and citations were checked and approved by the authors.
```

正式表述需要在投稿前按 IEEE 最新措辞再核对。

### 12. Title/Abstract 注意事项

AESS 建议避免在 title 和 abstract 中使用 `new` 或 `novel` 这类词。原因是 research article 默认报告新发现；标题和摘要应直接描述研究内容和可搜索关键词。

对当前题目建议:

```text
Neighborhood Label-Barycenter LMB Fusion for Distributed Multi-Target Tracking under Unreliable Communication
```

避免:

```text
A Novel Neighborhood Label-Barycenter Method ...
```

### 13. Abstract/Keywords

TAES template 对 abstract 和 keywords 的要求:

- Abstract 是单段。
- Abstract 应独立、准确反映全文。
- Abstract 不放 displayed equations、编号 citation 或 footnotes。
- Abstract 应包含三到四个不同关键词/短语，帮助检索，但避免过度重复。
- Keywords 按字母序，用逗号分隔。

### 14. 图表要求

TAES/IEEE template 中的图表准备要点:

- 图形格式: PS, EPS, TIFF, PDF, JPEG, PNG；最终文件中 figures 应单独提交。
- 图形宽度: 常见为 one-column 3.5 in / 88 mm 或 page-wide 7.16 in / 181 mm。
- 最大图形深度: 8.5 in / 216 mm。
- Color/grayscale figures 至少 300 dpi。
- Line art/table 至少 600 dpi。
- EPS/PDF/PS 需嵌入字体或转 outlines。
- 推荐字体: Times New Roman, Helvetica, Arial, Cambria, Symbol。

对当前稿件建议:

- Figure 1 用 vector PDF/EPS: local filters -> neighborhood reference -> Hungarian matching -> moment barycenter -> distributed outputs。
- Main results table 使用 TAES one-column 或 page-wide table，不要过密。
- N50 paired reductions 可以放一张 compact table；centralized consensus=0 upper-bound 放 appendix 或 supplementary。

## TAES-Specific Checklist for AA Label-Barycenter Paper

### 必做

- 使用 `template_regular/IEEE_TAES_orig-research/TAES_template.tex` 起草。
- 目标控制在 10 个 TAES template pages 内。
- 技术领域选 `Target Tracking and Multi-Sensor Systems`。
- 正文强调 distributed multi-target tracking，而不是泛泛的 information fusion framework。
- 摘要不使用 citation、footnote、display equation。
- 标题/摘要避免 `novel` 和 `new`。
- 相关工作补 TAES/RFS/AA density fusion precedent，尤其 `Arithmetic Average Density Fusion--Part III`。
- 补 Figure 1 和至少两张主表: N50 main comparison、reference-only ablation。
- 统一 AA/GA runtime denominator，或在表注中明确 runtime denominator 不同。
- 补 independent verifier 或至少独立复跑 N50。
- 写明当前 method 是 graph-local output-level iterative operator，若未完成 recursive online arm，不要声称是 recursive filtering update。
- Acknowledgments 准备 AI assistance disclosure。

### 强烈建议

- 增加 per-sensor/per-target qualitative example，展示 label split 和 same-label spatial spread 如何被修复。
- 增加至少一个 cross-scenario validation: 不同 packet-loss regime、topology、sensor count 或 partial-FOV stress。
- 增加 complexity discussion: Hungarian matching cost、neighborhood iterations、runtime `1.647x tuned AA`。
- 将 centralized cross-local projection 定位为 upper-bound diagnostic，不作为 main method headline。

### 不建议

- 不投 Letter 类型。
- 不把稿件压成 Correspondence，除非只保留一个极窄 theorem/ablation 点。
- 不把 `consensus=0` 写成主性能结论。
- 不隐藏 AI-assisted drafting 或 code navigation。

## Evidence Ledger

| ID | Type | Source or artifact | Supports |
| --- | --- | --- | --- |
| E1 | official author info | `docs/paper/taes/snapshots/taes_author_information_2026-06-23.html`; https://ieee-aess.org/publications/transactions-aes/author-information | manuscript types, submission portal, format, page charges, revision process, AI policy |
| E2 | official TAES home | `docs/paper/taes/snapshots/taes_home_2026-06-23.html`; https://ieee-aess.org/publications/taes | scope, single-anonymous review, plagiarism screening, OA/APC, ORCID, Letters discontinued |
| E3 | official template | `docs/paper/taes/TAES_Template.zip`; extracted regular/correspondence/letter templates | LaTeX source and local paper template |
| E4 | IEEE style manual | `docs/paper/taes/IEEE_Editorial_Style_Manual_for_Authors.pdf` | IEEE editorial style |
| E5 | AESS AI policy | `docs/paper/taes/snapshots/ieee_aess_ai_generated_content_2026-06-23.html`; https://ieee-aess.org/using-ai-generated-content-ieee-article-and-its-review | AI-generated content disclosure |
| E6 | TAES preprint policy | `docs/paper/taes/snapshots/taes_preprint_policy_2026-06-23.html`; https://ieee-aess.org/publications/transactions-aes/preprint-policy | preprint policy |
| E7 | technical areas | `docs/paper/taes/snapshots/taes_technical_area_descriptions_2026-06-23.html`; https://ieee-aess.org/publications/transactions-aes/technical-areas-editors/descriptions | target technical area selection |
| E8 | command | `sha256sum docs/paper/taes/TAES_Template.zip docs/paper/taes/IEEE_Editorial_Style_Manual_for_Authors.pdf` | downloaded file integrity hashes |

## Verification Record

Independence status: self-check only. 本文档由同一 worker lane 根据官方页面和本地下载文件整理，提交前需要人工复核在线系统。

已检查:

- 成功下载 TAES template zip，SHA-256 为 `6579ae30c43eaf695445e802603970c0c770269e3e88b3c28bee2cf99af8803e`。
- 解压后确认包含 `IEEE_TAES_orig-research.zip`、`IEEE_TAES_correspondence.zip`、`IEEE_TAES_letter.zip`。
- 解压 regular/original research template 后确认包含 `TAES_template.tex`、`TAES_template.pdf`、`IEEEtaes.cls`、`IEEEtaes.bst`、`IEEEtran_HOWTO.pdf`。
- 解压 correspondence template 后确认包含 `TAES_Corresp_template.tex`、`TAES_Corresp_template.pdf`、`IEEEtaes.cls`、`IEEEtaes.bst`、`IEEEtran_HOWTO.pdf`。
- 保存 TAES author-info、home、technical areas、preprint policy、AI content policy、IEEE templates page 的 HTML 快照。
- 下载 IEEE Editorial Style Manual PDF，SHA-256 为 `4891f9e7ae600ddc57628d46b242f5319b8924789841aac33b847001c2bedaba`。
- 当前环境未安装 `pdflatex` 或 `latexmk`，因此尚未本地编译模板。

## Risk and Escalation

主要风险:

- 在线投稿系统或 AESS 页面在未来更新；提交前必须重新核对页面。
- TAES template zip 内仍包含 letter 模板，但 TAES Letters 已停止，不能误用。
- 使用 AI 辅助写作若不披露可能导致 direct rejection。
- 超过 10 printed pages 会产生费用，且 OA fee 不抵扣 overlength charges。
- 如果当前 AA 方法仍是 output-level post-pass，TAES 审稿人可能要求更强 tracking-system justification。

升级条件:

- 正式提交前，必须由 corresponding author 登录 Atypon/REX 检查当前表单、technical area 和 mandatory files。
- 编译初稿前，需在本机或 CI 安装 TeX Live/MacTeX，并验证 `TAES_template.tex` 可编译。
- 如果计划上传 preprint，需要先确定版本和 IEEE copyright notice 管理方式。

## Reproducibility

下载与检查命令:

```bash
mkdir -p docs/paper/taes/template_raw docs/paper/taes/snapshots
curl -L 'https://confcats-web-assets.s3.amazonaws.com/ieeeaess/documents/TAES+Template.zip' -o docs/paper/taes/TAES_Template.zip
unzip -o docs/paper/taes/TAES_Template.zip -d docs/paper/taes/template_raw
unzip -o docs/paper/taes/template_raw/IEEE_TAES_orig-research.zip -d docs/paper/taes/template_regular
unzip -o docs/paper/taes/template_raw/IEEE_TAES_correspondence.zip -d docs/paper/taes/template_correspondence
unzip -o docs/paper/taes/template_raw/IEEE_TAES_letter.zip -d docs/paper/taes/template_letter
curl -L 'https://ieee-aess.org/publications/transactions-aes/author-information' -o docs/paper/taes/snapshots/taes_author_information_2026-06-23.html
curl -L 'https://ieee-aess.org/publications/taes' -o docs/paper/taes/snapshots/taes_home_2026-06-23.html
curl -L 'https://ieee-aess.org/publications/transactions-aes/technical-areas-editors/descriptions' -o docs/paper/taes/snapshots/taes_technical_area_descriptions_2026-06-23.html
curl -L 'https://ieee-aess.org/publications/transactions-aes/preprint-policy' -o docs/paper/taes/snapshots/taes_preprint_policy_2026-06-23.html
curl -L 'https://ieee-aess.org/using-ai-generated-content-ieee-article-and-its-review' -o docs/paper/taes/snapshots/ieee_aess_ai_generated_content_2026-06-23.html
curl -L 'https://journals.ieeeauthorcenter.ieee.org/wp-content/uploads/sites/7/IEEE-Editorial-Style-Manual-for-Authors.pdf' -o docs/paper/taes/IEEE_Editorial_Style_Manual_for_Authors.pdf
sha256sum docs/paper/taes/TAES_Template.zip docs/paper/taes/IEEE_Editorial_Style_Manual_for_Authors.pdf
python3 /Users/dex/.codex/skills/auto-research/scripts/evidence_lint.py docs/TAES_SUBMISSION_REQUIREMENTS_CN.md
```

## Open Issues

- 需要安装 TeX Live/MacTeX 后验证 regular template 可编译。
- 需要决定是否使用 traditional model 或 OA model。
- 需要确认所有作者 ORCID、affiliations、funding statement、corresponding author。
- 需要准备 AI assistance disclosure 的最终英文措辞。
- 需要确定是否上传 TechRxiv/arXiv preprint。
- 需要重新核对 Atypon/REX 表单是否要求 graphical abstract、data availability、conflict of interest 或 supplementary files。

## Final Recommendation

对当前 AA label-barycenter paper，使用 **TAES Regular Paper** 模板:

```text
docs/paper/taes/template_regular/IEEE_TAES_orig-research/TAES_template.tex
```

目标写作约束:

- 按 `Target Tracking and Multi-Sensor Systems` technical area 写。
- 控制在 10 个 TAES template pages 左右。
- 主文讲 distributed LMB multi-target tracking、label alignment、neighborhood barycenter、N50 tracking evidence 和 runtime。
- 把 centralized cross-local projection 放到 diagnostic/appendix。
- 提交前必须安装 TeX 环境并编译模板与初稿。
